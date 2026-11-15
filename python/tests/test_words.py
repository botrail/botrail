"""Words and analog inputs from Python (design-plc-resolution.md PF2): the
filter unit demo's bake, its loopback, a hand on the numbers while it
runs, its ST, the analog table, and the round trips."""

import sys
from pathlib import Path

import pytest

import botrail as bt
from botrail.seq import elapsed, rising, set_signal, set_word, signal, word

DEMO = Path(__file__).resolve().parents[2] / "examples" / "commissioning" / "filter_unit"
sys.path.insert(0, str(DEMO))


@pytest.fixture(scope="module")
def demo():
    import importlib.util

    spec = importlib.util.spec_from_file_location("filter_unit_demo", DEMO / "run_demo.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def baked(demo):
    scene = demo.build()
    return scene, demo.bake(scene)


def test_the_filter_unit_runs_on_its_setpoints(demo, baked) -> None:
    scene, tl = baked
    got = demo.summary(tl)
    # The warning trips where the scaled pressure reads the setpoint: 60
    # kPa on the first run, 40 after the operator keyed the lower one in.
    first, second = tl.signal("warn").rising_edges()
    assert tl.word("dp").value_at(first) == 60.0
    assert tl.word("dp").value_at(second) == 40.0
    assert got["pump_to_warn_s"] == [6.0, 3.5]
    assert [v for _, v in tl.word("dp_warn").samples] == [60.0, 40.0]
    # The alarm at 80 kPa stops the unit; the pressure falls with the flow.
    (alarm,) = tl.signal("alarm").rising_edges()
    assert tl.word("dp").value_at(alarm) == 80.0
    assert tl.word("dp_true").value_at(alarm + 0.6) == 0.0
    assert tl.word("dp_peak").samples[-1][1] == pytest.approx(80.0, abs=0.3)
    # The watchdog counted the valve's 400 ms stroke in milliseconds, and
    # stayed under the 600.
    assert max(v for _, v in tl.word("t_valve").samples) == 400.0
    assert tl.word("t_valve").integer and not tl.word("dp").integer
    assert not tl.signal("alarm_valve").rising_edges()
    # Deterministic, words included.
    again = demo.bake(scene)
    assert again.signals == tl.signals and again.words == tl.words


def test_the_loop_reads_measured_current_counts_scaled(demo, baked) -> None:
    scene, tl = baked
    first = tl.signal("warn").rising_edges()[0]
    (loop,) = tl.analog(first)
    assert (loop["name"], loop["unit"], loop["source"]) == ("dp", "kPa", "dp_true")
    assert loop["raw"] == 2400 and loop["scaled"] == 60.0 and loop["open"] is False
    assert loop["current"] == pytest.approx(13.6)
    assert loop["address"] == "%IW100"
    # The chain as a sheet: the range, the signal, the counts, the scaling.
    rows = [line.split(",") for line in scene.io_list("analog", demo.PROGRAMS).strip().splitlines()]
    assert rows[0][:5] == ["point", "host", "address", "node", "channel"]
    row = dict(zip(rows[0], rows[1]))
    assert row["point"] == "dp" and row["address"] == "%IW100" and row["measures"] == "dp_true"
    assert (row["range_lo"], row["range_hi"], row["unit"], row["signal"]) == ("0", "100", "kPa", "4-20mA")
    assert (row["raw_lo"], row["raw_hi"], row["counts_per_unit"], row["open_loop_counts"]) == ("0", "4000", "40", "-1000")
    assert row["scaling"] == "0 + (raw - 0) * 100 / 4000"


def test_the_loopback_through_the_plc_is_the_bake(demo, baked) -> None:
    scene, tl = baked
    assert demo.same(tl, demo.loopback(scene))


def test_a_hand_on_the_numbers_changes_what_the_unit_does(demo, baked) -> None:
    scene, _ = baked
    live = demo.live_checks(scene)
    # A watchdog shorter than the valve's stroke: a false valve alarm, and
    # the pump never starts.
    assert live["valve_time=200"] == {"alarm_valve": True, "pump": False, "t_valve_ms": 0.0}
    # A broken wire: 0 mA, a quarter of the span under the bottom count.
    wire = live["wire break"]
    assert wire["after"] == {"current": 0.0, "raw": -1000, "scaled": -25.0, "open": True}
    assert wire["alarm_wire"] is True and wire["run"] is False
    # The transmitter forced to 90 kPa: 3600 counts, the clogged alarm.
    assert live["dp forced to 90"] == {"raw": 3600.0, "alarm_dp": True, "dp_peak": 90.0}
    with bt.vc.loopback(scene, demo.PROGRAMS, host="PLC1", studio=False) as s:
        s.force("dp", 55.5)
        s.tick(2)
        assert s.forced == {"dp": 55.5} and s.word("dp") == 55.5
        s.unforce("dp")
        s.tick(2)
        assert s.forced == {} and s.word("dp") == 0.0
        # The counts are the module's; a measured word is not written.
        with pytest.raises(ValueError, match="force the analog input itself"):
            s.force("dp.raw", 100)
        with pytest.raises(ValueError, match="not a defined word"):
            s.write("dp", 5.0)


def test_words_are_io_like_signals(demo, baked) -> None:
    scene, _ = baked
    points = {(p.label, p.direction): p for p in scene.io_points(demo.PROGRAMS) if p.host == "PLC1"}
    dp = points[("dp", "input")]
    assert (dp.kind, dp.source, dp.status) == ("AI", "analog", "bound")
    # Read by the warning, the alarm, the wire-break and the peak rungs.
    assert {name for _, _, name in dp.readers} >= {"change the element soon", "element clogged",
                                                   "transmitter wire broken", "peak hold"}
    # A setpoint only the PLC reads is its own register; one the desk
    # writes crosses as a word; one declared an input comes from the wire.
    assert points[("dp_limit", "output")].status == "internal"
    assert points[("dp_warn", "input")].source == "signal:handshake"
    assert points[("valve_time", "input")].source == "signal:read-only"
    assert points[("t_valve", "output")].status == "internal"
    assert scene.io_report(demo.PROGRAMS).ok, scene.io_report(demo.PROGRAMS)


def test_the_st_declares_words_and_scales_the_counts(demo, baked) -> None:
    scene, _ = baked
    st = scene.st("PLC1", demo.PROGRAMS, cycle=False)
    for line in (
        "dp_eu := 0.0 + (INT_TO_REAL(dp) - 0.0) * 100.0 / 4000.0;",
        "warn := (run AND (dp_eu >= (INT_TO_REAL(dp_warn) / 10.0)));",
        "IF (dp_eu >= dp_limit) THEN alarm_dp := TRUE; END_IF;",
        "IF (dp < -100) THEN alarm_wire := TRUE; END_IF;",
        "IF (run AND NOT (valve_extended)) THEN t_valve := t_valve + 10; END_IF;",
        "IF (dp_eu > dp_peak) THEN dp_peak := dp_eu; END_IF;",
        "dp_limit : REAL := 80.0; (* word *)",
        "t_valve : INT := 0; (* word *)",
        "dp AT %IW100 : INT;",
        "valve_time AT %IW102 : INT;",
        "dp_warn AT %IW103 : INT;",
    ):
        assert line in st, f"missing `{line}` in\n{st}"
    with pytest.raises(ValueError, match="Structured Text"):
        scene.plcopen()


def test_words_survive_the_project_file_and_the_generated_python(demo, baked, tmp_path) -> None:
    scene, tl = baked
    path = tmp_path / "filter.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert reloaded.words == scene.words and reloaded.analogs == scene.analogs
    again = reloaded.simulate_sequences(demo.PROGRAMS)
    assert again.signals == tl.signals and again.words == tl.words
    code = scene.generate_python()
    assert 'scene.define_word("valve_time", 600, integer=True)' in code
    assert 'scene.add_analog("dp", source="dp_true", range=(0, 100), unit="kPa", raw=(0, 4000), place="dpt/body")' in code
    assert 'bt.seq.compare(bt.seq.word("dp"), ">=", bt.seq.word("dp_limit"))' in code
    assert '.move("t_valve", bt.seq.calc(bt.seq.word("t_valve"), "+", 10))' in code
    assert 'bt.seq.set_word("dp_warn", 40)' in code


def test_words_from_scratch_compare_move_and_wait() -> None:
    scene = bt.Scene()
    scene.define_signal("tick")
    scene.define_word("n", 0, integer=True)
    scene.define_word("half", 0, integer=True)
    scene.define_word("ratio", 0.0)
    scene.define_word("target", 0, integer=True)
    lad = scene.sequence("count")
    lad.rung(rising("tick")).move("n", word("n") + 1)
    lad.rung(word("n") >= 0).move("half", word("n") / 2).move("ratio", word("n") / 2.5)
    op = scene.sequence("pulses")
    for k in range(3):
        op.step(f"on{k}", [set_signal("tick")], elapsed(0.05))
        op.step(f"off{k}", [set_signal("tick", False)], elapsed(0.05))
    op.step("until", transition=word("half") == 1)       # 3 / 2, truncated
    op.step("preset", [set_word("target", word("n") * 2)], elapsed(0.02))   # a step moves a word in
    tl = scene.simulate_sequences(["count", "pulses"])
    assert tl.word("target").samples[-1][1] == 6.0
    assert tl.word("n").samples[-1][1] == 3.0
    assert tl.word("half").samples[-1][1] == 1.0
    assert tl.word("ratio").samples[-1][1] == pytest.approx(1.2)
    # What it waits on is named, with the words it compares.
    span = next((a, b) for name, a, b in tl.step_spans if name.endswith("on0"))
    assert tl.waiting(span[0])[0]["step"] == "on0"
    # Operators build conditions and one-instruction expressions.
    assert (word("a") >= 5) == {"type": "compare", "left": "a", "op": ">=", "right": 5.0}
    assert (word("a") != word("b"))["op"] == "!="
    assert bt.seq.calc(2, "*", word("a")).op == "*"
    with pytest.raises(TypeError, match="one instruction"):
        (word("a") + 1) * 2
    with pytest.raises(TypeError, match="a number"):
        word("a") >= "five"


def test_authoring_errors_name_the_word(demo) -> None:
    scene = demo.build()
    with pytest.raises(ValueError, match="already a signal"):
        scene.define_word("pump", 0.0)
    with pytest.raises(ValueError, match="unknown word `flow`"):
        scene.add_analog("fl", source="flow", range=(0, 10))
    lad = scene.sequence("extra")
    lad.rung(signal("run")).move("dp", 1.0)
    with pytest.raises(ValueError, match="read-only"):
        scene.simulate_sequences(demo.PROGRAMS + ["extra"])
    lad = scene.sequence("extra")
    lad.rung(word("ghost") > 1).coil("lamp_run")
    with pytest.raises(ValueError, match="unknown word `ghost`"):
        scene.simulate_sequences(demo.PROGRAMS + ["extra"])
    lad = scene.sequence("extra")
    lad.rung(signal("run")).move("dp_limit", 70)          # nobody else writes it: fine
    lad.rung(signal("run")).move("t_valve", 0)            # `filter` writes it
    with pytest.raises(ValueError, match="commanded by both"):
        scene.simulate_sequences(demo.PROGRAMS + ["extra"])
    # An analog input opens; it has no stuck level.
    scene = demo.build()
    scene.add_scenario("stuck", faults=[bt.io.stuck("dp", True)])
    with pytest.raises(ValueError, match="analog input"):
        scene.simulate_sequences(demo.PROGRAMS, scenario="stuck")


def test_an_open_loop_is_a_scenario_too() -> None:
    scene = bt.Scene()
    scene.define_signal("broken")
    scene.define_word("level", 50.0)
    scene.add_analog("lt", source="level", range=(0.0, 200.0), unit="mm", raw=(0, 16000))
    lad = scene.sequence("watch")
    lad.rung(word("lt.raw") < -500).coil("broken")
    scene.sequence("end").step("tail", transition=elapsed(0.1))
    scene.add_scenario("wire", faults=[bt.io.open("lt")])
    tl = scene.simulate_sequences(["watch", "end"])
    assert tl.word("lt.raw").samples == [(0.0, 4000.0)] and not tl.signal("broken").rising_edges()
    tl = scene.simulate_sequences(["watch", "end"], scenario="wire")
    assert tl.word("lt.raw").samples[-1][1] == -4000.0   # 0 mA: a quarter of the span under zero
    assert tl.word("lt").samples[-1][1] == -50.0
    assert tl.signal("broken").rising_edges() == [0.0]
