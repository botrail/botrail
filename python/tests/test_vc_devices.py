"""Virtual commissioning, VC6 (design-virtual-commissioning.md): the device
vocabulary a PLC drives a cell with — word lanes (encoders, identification
readers), coil-driven cylinders, belt stoppers and accumulation, measurement
words scaled to a register's unit, declared words with a host, and the
parts that build them (`bt.parts.cylinder` / `stopper` / `tower_lamp`).
"""

import sys
from pathlib import Path

import pytest

import botrail as bt

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples" / "commissioning" / "openplc_feed_line"))


# ------------------------------------------------------------------ cells


def cylinder_cell(solenoid: str = "double") -> bt.Scene:
    """A press cylinder a PLC runs: extend, dwell, retract."""
    scene = bt.Scene()
    bt.parts.cylinder(scene, "press", (0.3, 0.0, 0.6), axis=(0, 0, -1), stroke=0.04, time=0.3,
                      solenoid=solenoid, pad=(0.04, 0.04, 0.01), encoder=True)
    sq = scene.sequence("cycle")
    sq.step("down", actions=[bt.seq.move_to("press", "extended")], transition=bt.seq.device_done("press"))
    sq.step("dwell", transition=bt.seq.elapsed(0.2))
    sq.step("up", actions=[bt.seq.move_to("press", "retracted")], transition=bt.seq.device_done("press"))
    scene.add_io_node("PLC1", kind="plc", programs=["cycle"],
                      channels=bt.io.di8(base="%IX0.0") + bt.io.do8(base="%QX0.0") + bt.io.word(2, base="%IW0", unit="mm"))
    scene.auto_assign_io()
    return scene


def belt_cell() -> bt.Scene:
    """Two parts on a belt running into a raised stopper, an identification
    reader at the stop, a tower lamp: the PLC reads the id and the encoder."""
    scene = bt.Scene()
    scene.add_box("bed", size=(2.0, 0.3, 0.05), position=(1.0, 0.0, 0.475))
    scene.add_box("a", size=(0.1, 0.1, 0.05), position=(0.2, 0.0, 0.525))
    scene.add_box("b", size=(0.1, 0.1, 0.05), position=(0.0, 0.0, 0.525))
    bt.parts.stopper(scene, "stop1", (1.0, 0.0, 0.5), stroke=0.03, time=0.2, blade=(0.02, 0.2, 0.04), raised=True)
    scene.add_conveyor("belt", zone_position=(1.0, 0.0, 0.55), zone_size=(2.0, 0.3, 0.2),
                       velocity=(0.5, 0.0, 0.0), running=False, encoder=True)
    scene.add_zone_sensor("eye", position=(0.9, 0.0, 0.525), size=(0.05, 0.3, 0.1), watch=["a", "b"],
                          ids={"a": 7, "b": 3})
    scene.define_signal("running")
    bt.parts.tower_lamp(scene, "tower", (1.5, 0.4, 0.0), tiers={"green": "running"})
    sq = scene.sequence("run")
    sq.step("feed", actions=[bt.seq.start("belt"), bt.seq.set_signal("running")], transition=bt.seq.elapsed(3.0))
    sq.step("release", actions=[bt.seq.move_to("stop1", "retracted")], transition=bt.seq.device_done("stop1"))
    sq.step("drain", transition=bt.seq.elapsed(1.0))
    sq.step("halt", actions=[bt.seq.stop("belt"), bt.seq.set_signal("running", False)])
    scene.add_io_node("PLC1", kind="plc", programs=["run"],
                      channels=bt.io.di8(base="%IX0.0") + bt.io.do8(base="%QX0.0") + bt.io.word(2, base="%IW0", unit="mm"))
    scene.declare_io("eye.id", role="input", host="PLC1")
    scene.auto_assign_io()
    return scene


def same_timeline(a, b) -> None:
    assert a.duration == b.duration
    assert a.signals == b.signals
    assert a.words == b.words
    assert a.step_spans == b.step_spans


# -------------------------------------------------------------- cylinder


def test_a_cylinder_is_two_coils_and_two_switches_on_the_exchange() -> None:
    scene = cylinder_cell()
    xc = bt.vc.exchange(scene, "PLC1")
    assert [p.label for p in xc.inputs] == ["press/extend", "press/retract"]
    assert [p.label for p in xc.outputs] == ["press.encoder", "press/extended", "press/retracted"]
    coil = xc.inputs[0]
    assert (coil.source, coil.value_type, coil.var) == ("device:coil", "BOOL", "press_extend")
    # No position word: the controller never sees a number for a cylinder.
    assert all(p.label != "press.position" for p in xc.points)
    # The encoder came out on the millimetre word channel: an INT the PLC reads.
    enc = next(p for p in xc.points if p.label == "press.encoder")
    assert (enc.value_type, enc.unit, enc.scale, enc.var, enc.address) == ("INT", "mm", 1000.0, "press_encoder", "%IW0")
    st = scene.st("PLC1")
    assert "press_extend := TRUE; (* extended *)" in st and "press_retract := FALSE; (* extended *)" in st
    assert "IF s_down AND (press_busy AND press_extended) THEN" in st
    assert "press_busy := press_extend AND NOT press_retract;" in st
    assert "IF s_down AND NOT press_extended THEN press_busy := TRUE; END_IF;" in st
    assert "IF s_up AND (press_busy AND press_retracted) THEN" in st
    # Only what the program uses is declared; the encoder is the exchange's.
    assert "press_encoder" not in st
    # The PLCopen export agrees: coils written, the switch awaited.
    xml = scene.plcopen(["cycle"])
    assert "press_extend := TRUE" in xml and "press_extended" in xml and "press_position" not in xml


def test_the_loopback_through_the_coils_is_the_bake_and_a_single_coil_springs_back() -> None:
    scene = cylinder_cell()
    bake = scene.simulate_sequence("cycle")
    with bt.vc.loopback(scene, ["cycle"], host="PLC1", studio=False) as s:
        s.run()
        same_timeline(s.timeline(), bake)
        rec = s.recording()
    extend = [v for _, _, label, v in rec.records if label == "press/extend"]
    retract = [v for _, _, label, v in rec.records if label == "press/retract"]
    # The image (retracted: the retract coil energised), out, back.
    assert extend == [False, True, False] and retract == [True, False, True]
    # The encoder word is the rod's travel, recorded as it moves.
    travel = bake.word("press.encoder")
    assert not travel.integer and travel.max() == pytest.approx(0.04) and travel.final_value() == pytest.approx(0.0)
    assert travel.reaches(0.04) == pytest.approx(0.3, abs=0.011)

    # An external PLC: the coil high drives it out, the switch answers.
    live = scene.open_plant(["cycle"], ["PLC1"], loopback=False)
    live.tick()
    live.apply("PLC1", {"press/extend": True})
    for _ in range(50):
        live.tick()
    out = dict(live.outputs("PLC1"))
    assert out["press/extended"] is True and out["press.encoder"] == 40
    live.apply("PLC1", {"press/extend": False})
    for _ in range(50):
        live.tick()
    assert dict(live.outputs("PLC1"))["press/extended"] is True, "a double solenoid holds"

    single = cylinder_cell(solenoid="single")
    xc = bt.vc.exchange(single, "PLC1")
    assert [p.label for p in xc.inputs] == ["press/extend"]
    live = single.open_plant(["cycle"], ["PLC1"], loopback=False)
    live.tick()
    live.apply("PLC1", {"press/extend": True})
    for _ in range(50):
        live.tick()
    assert dict(live.outputs("PLC1"))["press/extended"] is True
    live.apply("PLC1", {"press/extend": False})
    for _ in range(50):
        live.tick()
    assert dict(live.outputs("PLC1"))["press/retracted"] is True, "the spring returns it"
    st = single.st("PLC1")
    assert "press_extend := FALSE; (* retracted *)" in st and "press_retract :=" not in st
    assert "press_busy := NOT press_extend;" in st


def test_coil_refusals_name_the_problem() -> None:
    scene = bt.Scene()
    scene.add_box("rod", size=(0.02, 0.02, 0.06), position=(0, 0, 0.3))
    with pytest.raises(ValueError, match="unknown stop"):
        scene.add_linear_axis("cyl", objects=["rod"], axis=(0, 0, 1), speed=0.1, range=(0.0, 0.05),
                              stops={"retracted": 0.0, "extended": 0.05}, coils={"extend": "out"})
    with pytest.raises(ValueError, match="spring-return"):
        scene.add_linear_axis("cyl", objects=["rod"], axis=(0, 0, 1), speed=0.1, range=(0.0, 0.05),
                              stops={"a": 0.0, "b": 0.02, "c": 0.05}, coils={"extend": "c"})
    with pytest.raises(ValueError, match="solenoid"):
        bt.parts.cylinder(scene, "p", (0, 0, 0.5), solenoid="triple")


# ------------------------------------------------- belt, stopper, reader


def test_parts_queue_against_a_raised_stopper_and_run_on_when_it_drops() -> None:
    scene = belt_cell()
    tl = scene.simulate_sequence("run")
    # While the blade is up both parts stand at the stop: `a` against the
    # blade's back face (x = 0.99 - 0.05), `b` against `a`.
    assert tl.object_pose("a", 3.0)[0][0] == pytest.approx(0.94)
    assert tl.object_pose("b", 3.0)[0][0] == pytest.approx(0.84)
    # The blade dropped: they run on at belt speed again.
    assert tl.object_pose("a", 4.2)[0][0] > 1.2
    stop_lane = tl.signal("stop1/extended")
    assert stop_lane.value_at(0.0) and not stop_lane.value_at(3.3)
    # The belt ran 3.0 s, the 0.2 s the blade took to drop, and 1.0 s more.
    travel = tl.word("belt.encoder")
    assert travel.value_at(3.0) == pytest.approx(1.5, abs=1e-9) and travel.final_value() == pytest.approx(2.1, abs=1e-9)
    # The reader: nothing, then `a` (7), then `b` queued beside it (3), then
    # nothing once the queue has run out.
    ids = [v for _, v in tl.word("eye.id").samples]
    assert ids[:3] == [0, 7, 3] and ids[-1] == 0
    assert tl.word("eye.id").integer
    # The green tier follows `running`.
    assert any(l["signal"] == "running" and l["obstacle"] == "tower/green" for l in scene.lamps())


def test_a_declared_word_with_a_host_crosses_the_exchange_and_the_encoder_is_scaled() -> None:
    scene = belt_cell()
    xc = bt.vc.exchange(scene, "PLC1")
    labels = {p.label: p for p in xc.points}
    assert labels["eye.id"].side == "output" and labels["eye.id"].value_type == "INT"
    assert labels["eye.id"].source == "declared" and labels["eye.id"].var == "eye_id"
    assert labels["belt.encoder"].side == "output" and labels["belt.encoder"].unit == "mm"
    assert "stop1/extend" in labels and "stop1.position" not in labels
    with bt.vc.loopback(scene, ["run"], host="PLC1", studio=False) as s:
        s.run(for_=1.4)                       # `a` alone in the eye
        out = s.outputs("PLC1")
        assert out["eye.id"] == 7 and out["belt.encoder"] == 700, out
        assert s.word("belt.encoder") == pytest.approx(0.7) and s.word("eye.id") == 7
        s.run(for_=0.6)                       # `b` queued beside it: the smaller id
        assert s.outputs("PLC1")["eye.id"] == 3
        s.run()
        same_timeline(s.timeline(), scene.simulate_sequence("run"))
    # The Modbus image: the words are 16-bit input registers, the coils coils.
    link = bt.vc.modbus_server(xc, "127.0.0.1:5098", iec_base=0)
    rows = {r.label: (r.table, r.type) for r in link.rows}
    assert rows["eye.id"] == ("input_register", "INT")
    assert rows["belt.encoder"] == ("input_register", "INT")
    assert rows["stop1/extend"] == ("coil", "BOOL")
    # Refusals: a host that is not a node, a word nobody records.
    bad = belt_cell()
    bad.declare_io("eye.id", role="input", host="PLC9")
    assert any("PLC9" in str(f) for f in bad.auto_assign_io().findings)
    bad.declare_io("belt.encoder", role="input", host="PLC1")  # recorded: fine
    bad.declare_io("bed.encoder", role="input", host="PLC1")
    assert any("does not record" in str(f) for f in bad.auto_assign_io().findings)
    with pytest.raises(ValueError, match="scale by"):
        bt.io.channels("word", 1, unit="furlong")
        bt.Scene().add_io_node("P", kind="plc", channels=bt.io.channels("word", 1, unit="furlong"))


def test_a_measurement_on_a_word_channel_without_a_unit_is_a_finding() -> None:
    scene = bt.Scene()
    scene.add_box("strip", size=(0.6, 0.05, 0.005), position=(0.0, 0.0, 0.5))
    scene.add_linear_axis("feed", objects=["strip"], axis=(1, 0, 0), speed=0.2, range=(0.0, 0.6))
    sq = scene.sequence("go")
    sq.step("a", actions=[bt.seq.move_to("feed", 0.1)], transition=bt.seq.device_done("feed"))
    scene.add_io_node("PLC1", kind="plc", programs=["go"], channels=bt.io.di8(base="%IX0.0") + bt.io.word(1, base="%QW0"))
    report = scene.auto_assign_io()
    assert any(f.code == "unit" and "16-bit" in f.message for f in report.findings)
    xc = bt.vc.exchange(scene, "PLC1")
    pos = next(p for p in xc.points if p.label == "feed.position")
    assert (pos.value_type, pos.scale, pos.unit) == ("REAL", 1.0, None)
    assert "feed_position := 0.1;" in scene.st("PLC1")


# --------------------------------------------------------- persistence


def test_coils_encoders_ids_and_stoppers_survive_the_project_file(tmp_path) -> None:
    scene = belt_cell()
    path = tmp_path / "belt.botrail"
    scene.save_project(path)
    again = bt.Scene.load_project(path)
    same_timeline(again.simulate_sequence("run"), scene.simulate_sequence("run"))
    xc, xc2 = bt.vc.exchange(scene, "PLC1"), bt.vc.exchange(again, "PLC1")
    assert xc.to_markdown() == xc2.to_markdown()
    py = scene.generate_python()
    assert 'coils={"extend": "extended"}' in py and "encoder=True" in py
    assert 'ids={"a": 7, "b": 3}' in py and 'set_obstacle_stopper("stop1/pad", True)' in py
    assert 'declare_io("eye.id", role="input", host="PLC1")' in py
    assert '"unit":"mm"' in again.io_map().to_json().replace(" ", "")


# ------------------------------------------------------------ the demo


def test_the_feed_line_demo_indexes_three_parts_and_writes_millimetres() -> None:
    import run_demo as feed

    scene = feed.build()
    xc = bt.vc.exchange(scene, "PLC1")
    labels = {p.label: p for p in xc.points}
    assert labels["feed.position"].address == "%QW100" and labels["feed.position"].value_type == "INT"
    assert labels["feed.encoder"].address == "%IW100" and labels["feed.encoder"].unit == "mm"
    ref, rec = feed.reference(scene)
    assert feed.positions_written(rec) == feed.EXPECTED_POSITIONS
    assert [name for name, _, _ in ref.step_spans][:5] == ["ready", "run", "feed1", "press1", "down1"]
    assert ref.word("feed.encoder").max() == pytest.approx(0.3)
    assert ref.signal("press/extended").rising_edges() and len(ref.signal("press/extended").rising_edges()) == 3
    link = bt.vc.modbus_server(xc, "127.0.0.1:5097", iec_base=100)
    rows = {r.label: (r.table, r.address, r.type) for r in link.rows}
    assert rows["feed.position"] == ("holding_register", 0, "INT")
    assert rows["feed.encoder"] == ("input_register", 0, "INT")
    st = scene.st("PLC1", cycle=False)
    assert "feed_position := 100; (* 0.1 m *)" in st and "feed_position AT %QW100 : INT;" in st
