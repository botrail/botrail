"""Rungs and gates from Python (design-plc-resolution.md PF1): the ladder
line demo's bake, its loopback, its ST, and the round trips."""

import sys
from pathlib import Path

import pytest

import botrail as bt

DEMO = Path(__file__).resolve().parents[2] / "examples" / "commissioning" / "ladder_line"
sys.path.insert(0, str(DEMO))


@pytest.fixture(scope="module")
def demo():
    import importlib.util

    spec = importlib.util.spec_from_file_location("ladder_line_demo", DEMO / "run_demo.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def baked(demo):
    scene = demo.build()
    return scene, demo.bake(scene)


def test_the_ladder_line_runs_its_story(demo, baked) -> None:
    scene, tl = baked
    got = demo.summary(tl)
    assert got["cycles_started"] == 4 and got["works_discharged"] == 2
    # Run: latched by start, dropped at the end of the cycle the stop was
    # asked in, latched again for the step-mode run.
    assert [v for _, v in got["run"]] == [False, True, False, True]
    # The counter reaches its preset of two, and the reset clears it.
    assert [v for _, v in tl.word("made.count").samples] == [0.0, 1.0, 2.0, 0.0]
    assert [v for _, v in got["lot_lamp"]] == [False, True, False]
    # The dwell timers: on 0.8 s after their step, off with it.
    t1 = tl.signal("t_dwell1").rising_edges()
    dwell = [(a, b) for name, a, b in tl.step_spans if name == "auto/dwell 1"]
    assert t1[0] == pytest.approx(dwell[0][0] + demo.DWELL_S, abs=0.011)
    # Step mode: the chain leaves `wait` on the first press of the step
    # button, not when its condition came true (run, a scan after start).
    spans = {name: (a, b) for name, a, b in tl.step_spans if name.startswith("operator/")}
    waits = [(a, b) for name, a, b in tl.step_spans if name == "auto/wait"]
    pressed = spans["operator/step 1: press"][0]
    assert any(b == pytest.approx(pressed + 0.01, abs=1e-6) for _, b in waits)
    # The reset button puts the chain back in `wait` the scan after it.
    reset = spans["operator/reset: press"][0]
    assert any(a == pytest.approx(reset + 0.01, abs=1e-6) for a, _ in waits)
    # Deterministic.
    again = demo.bake(scene)
    assert again.signals == tl.signals and again.step_spans == tl.step_spans


def test_the_loopback_through_the_plc_is_the_bake(demo, baked) -> None:
    scene, tl = baked
    assert demo.same(tl, demo.loopback(scene))


def test_rungs_are_io_like_steps(demo, baked) -> None:
    scene, _ = baked
    points = {(p.label, p.direction): p for p in scene.io_points(demo.PROGRAMS) if p.host == "PLC1"}
    # The belt's coil is written by a rung of `outputs` (flat index: its
    # steps — none — then its rungs).
    conv = points[("conv", "output")]
    assert conv.writers == [("outputs", 0, "belt: the travel steps, or the manual jog")]
    assert points[("pb_start", "input")].readers[0][0] == "common"
    # Timers are relays of the PLC: no point.
    assert {p.status for (label, _), p in points.items() if label == "t_dwell1"} == {"internal"}
    # A relay only a gate reads is the PLC's own too (the gates are read).
    assert {p.status for (label, _), p in points.items() if label == "advance"} == {"internal"}
    assert scene.io_report(demo.PROGRAMS).ok, scene.io_report(demo.PROGRAMS)


def test_the_st_writes_rungs_gates_and_step_contacts(demo, baked) -> None:
    scene, _ = baked
    st = scene.st("PLC1", demo.PROGRAMS, cycle=False)
    assert "run := ((pb_start OR run) AND NOT (stop_done));" in st
    assert "IF (pb_stop AND run) THEN stop_req := TRUE; END_IF;" in st
    assert "t_t_dwell1(IN := auto_at_dwell_1, PT := T#800ms);" in st
    assert "c_made(CU := auto_at_clear, R := pb_reset, PV := 2);" in st
    assert "auto_at_wait := s_wait; (* read by another program *)" in st
    assert "IF pb_reset AND NOT first THEN" in st or "IF pb_reset THEN" in st
    assert "AND advance THEN" in st
    # The edge contact of the step-mode rung is sampled every scan.
    assert "r_pb_step(CLK := pb_step);" in st
    # Repeated multi-line IF headers are kept apart for OpenPLC's optimizer.
    assert "IF first THEN (* 2 *)" in st and "IF first THEN (* 3 *)" in st
    # Relays shared by the host's programs are globals.
    assert "stop_req : BOOL; (* relay *)" in st.split("CONFIGURATION")[1]
    with pytest.raises(ValueError, match="Structured Text"):
        scene.plcopen()


def test_rungs_survive_the_project_file_and_the_generated_python(demo, baked, tmp_path) -> None:
    scene, tl = baked
    path = tmp_path / "line.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    again = reloaded.simulate_sequences(demo.PROGRAMS)
    assert again.signals == tl.signals and again.step_spans == tl.step_spans
    code = scene.generate_python()
    assert 'sequence.gates(enable=bt.seq.signal("advance", True), reset=bt.seq.signal("pb_reset", True), cycle=True)' in code
    assert '.ton("t_dwell1", 0.8)' in code and '.ctu("made", 2)' in code
    assert 'bt.seq.step_active("auto", "wait")' in code and "bt.seq.not_(" in code


def test_authoring_errors_name_the_rung(demo) -> None:
    scene = demo.build()
    lad = scene.sequence("extra")
    lad.rung(bt.seq.elapsed(1.0)).coil("lamp_run")
    with pytest.raises(ValueError, match="belong to a step"):
        scene.simulate_sequences(["extra", "operator"])
    lad = scene.sequence("extra")
    lad.rung(bt.seq.signal("pb_jog")).coil("run")  # `common` drives it
    with pytest.raises(ValueError, match="commanded by both"):
        scene.simulate_sequences(demo.PROGRAMS + ["extra"])
    # Rungs alone never end a bake.
    with pytest.raises(ValueError, match="nothing ends this bake"):
        scene.simulate_sequences(demo.PLC_PROGRAMS)


def test_waits_name_a_rung_gated_step(demo, baked) -> None:
    scene, tl = baked
    span = next((a, b) for name, a, b in tl.step_spans if name == "auto/to st1")
    (w,) = [w for w in tl.waiting((span[0] + span[1]) / 2) if w["program"] == "auto"]
    assert w["step"] == "to st1"
    leaf = w["exits"][0]["unmet"][0]
    assert leaf["condition"] == "s1=1" and leaf["address"].startswith("%IX100.")
