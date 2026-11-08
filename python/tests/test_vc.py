"""Virtual commissioning, VC1 (design-virtual-commissioning.md): the exchange
derived from the I/O map, the plant rollout driven through it, loopback
reproducing the bake bit for bit, replay reproducing the loopback, and a
hand on the wire (force / inject) showing up in the diff.
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
for sub in ("export", "basics", "machining"):
    sys.path.insert(0, str(EXAMPLES / sub))
sys.path.insert(0, str(Path(__file__).resolve().parent))

NS = {"p": "http://www.plcopen.org/xml/tc6_0201", "x": "http://www.w3.org/1999/xhtml"}


# ------------------------------------------------------------------ cells


def pick_cell() -> bt.Scene:
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene)
    pick.wire_cell(scene)
    return scene


def plc_master_cell() -> bt.Scene:
    """The two-arm cell of test_io_map with a PLC running `pick` and `belt`
    and driving both arms from outside their controllers (near's declared,
    far's implicit), a remote station and a robot controller running
    `watch`."""
    from test_io_map import author_two_arm, two_arm_cell

    scene = two_arm_cell()
    author_two_arm(scene)
    scene.add_io_node("PLC1", kind="plc", programs=["pick", "belt"],
                      channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.add_io_node("RIO1", kind="remote_io", uplink=("PLC1", "PROFINET"),
                      channels=bt.io.di8(base="%IX1.0"))
    scene.add_io_node("NEAR", kind="robot_controller", robots=["near"], programs=["watch"],
                      channels=bt.io.ur_standard())
    scene.bind_input("beam", "RIO1", "DI2", tag="Beam", field="-B1")
    scene.bind_output("belt", "PLC1", "DO0", tag="BeltRun")
    return scene


def tending_demo():
    """The machine-tending cell and its handshake — or a skip where the
    UR12e and Hand-E kit cannot be ordered: an offline machine, and CI,
    which has no `huggingface_hub`."""
    import machine_tending_demo as m

    pytest.importorskip("huggingface_hub", reason="the tending cell orders its arm from the catalog")
    try:
        return m.build()
    except ValueError as err:
        # IK, collision and catalog parsing regressions must fail the test.
        if "cannot reach the catalog dataset" not in str(err):
            raise
        pytest.skip(f"catalog unavailable: {err}")


def tending_cell():
    scene, hs = tending_demo()
    return scene, ["tend", hs.program]


def two_controller_cell():
    """A simulatable two-controller cell: a PLC runs the line — the belt,
    the far arm (from outside its controller, two commands → a program
    word), a handshake to the near arm's controller — and the near arm's
    own controller runs `watch`."""
    arm = bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")
    scene = bt.Scene(arm, name="near")
    scene.add_robot(arm, name="far", base_position=(1.5, 0.0, 0.0))
    for robot in ("near", "far"):
        scene.add_segment(f"{robot}_go", goal=[0.4, 0.4, -0.5, 0.2, 0.0, 0.0], robot=robot)
        scene.add_segment(f"{robot}_home", goal=[0.0] * 6, robot=robot)
    scene.add_box("part", size=(0.05, 0.05, 0.05), position=(0.5, 0.3, 0.03))
    scene.add_conveyor("belt", zone_position=(0.5, 0.3, 0.05), zone_size=(1.0, 0.2, 0.1),
                       velocity=(0.1, 0.0, 0.0), running=False)
    scene.add_beam_sensor("beam", frm=(0.7, 0.2, 0.03), to=(0.7, 0.4, 0.03), watch=["part"])
    for name in ("carrying", "cycle_done"):
        scene.define_signal(name)
    line = scene.sequence("line")
    line.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("beam"))
    line.step("stop", actions=[bt.seq.stop("belt"), bt.seq.motion("far_go")])
    line.step("carry", actions=[bt.seq.set_signal("carrying"), bt.seq.motion("far_home")])
    line.step("finish", actions=[bt.seq.set_signal("cycle_done")],
              transition=bt.seq.all_of(bt.seq.elapsed(0.5), bt.seq.robot_done("near")))
    watch = scene.sequence("watch")
    watch.step("wait", transition=bt.seq.rising("carrying"))
    watch.step("go", actions=[bt.seq.motion("near_go")])
    watch.step("home", actions=[bt.seq.motion("near_home")])
    scene.add_io_node("PLC1", kind="plc", programs=["line"],
                      channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.add_io_node("NEAR", kind="robot_controller", robots=["near"], programs=["watch"],
                      channels=bt.io.ur_standard())
    scene.bind_input("beam", "PLC1", "DI2", tag="Beam")
    scene.bind_output("belt", "PLC1", "DO0", tag="BeltRun")
    return scene, ["line", "watch"]


def same_timeline(a, b, *, skip_programs=(), samples=200, tol=0.0) -> None:
    """Bit-identical where it counts: duration, lanes, step bands (except
    those of `skip_programs`, whose steps an external controller owns) and
    every robot's joints along the cycle.

    `tol` is for a replay against its loopback: the commands are the same
    tick for tick, but a program's transitions also bookmark every robot's
    track (`exit_step` holds each configuration to the instant), and an
    external controller's program takes no transitions on the plant — so
    the tracks differ in bookmarks, and interpolating between them moves a
    held joint by an ulp here and there."""
    assert a.duration == b.duration
    assert a.robots == b.robots
    assert a.signals == b.signals

    def keep(span):
        name = span[0]
        return not any(name == p or name.startswith(p + "/") for p in skip_programs)

    assert [s for s in a.step_spans if keep(s)] == [s for s in b.step_spans if keep(s)]
    for robot in a.robots:
        for k in range(samples + 1):
            t = k * a.duration / samples
            qa, qb = a.sample(t, robot), b.sample(t, robot)
            if tol == 0.0:
                assert qa == qb, (robot, t)
            else:
                assert len(qa) == len(qb) and all(abs(x - y) <= tol for x, y in zip(qa, qb)), (robot, t)


def plcopen_globals(scene, sequences=None) -> dict:
    """`{variable: (type, address)}` of the PLCopen export's globals."""
    doc = ET.fromstring(scene.plcopen(sequences))
    out = {}
    for var in doc.findall(".//p:globalVars/p:variable", NS):
        ty = list(var.find("p:type", NS))[0].tag.split("}")[1]
        out[var.get("name")] = (ty, var.get("address"))
    return out


# --------------------------------------------------------------- exchange


def test_exchange_is_the_hosts_half_of_the_io_map_turned_round() -> None:
    scene = pick_cell()
    xc = bt.vc.exchange(scene, "UR")
    assert xc.host == "UR" and xc.programs == ["pick"]
    by_label = {p.label: p for p in xc.points}
    # The robot's own controller runs the program: the sensor it reads is
    # a plant output, the coils it writes are plant inputs — and its own
    # motions never cross (no entries).
    assert by_label["part_at_pick"].side == "output"
    assert by_label["part_at_pick"].source == "sensor"
    assert by_label["conv"].side == "input" and by_label["conv"].source == "device:run"
    assert by_label["vacuum"].side == "input" and by_label["vacuum"].value_type == "BOOL"
    assert by_label["conv"].port == 0 and by_label["conv"].node == "UR"
    assert xc.entries == []
    # Bindings travel: the tag-less UR channels carry their vendor port.
    assert by_label["part_at_pick"].channel == "DI2"
    # Renderings.
    md = xc.to_markdown()
    assert "Inputs — the controller writes" in md and "`part_at_pick`" in md
    assert xc.to_csv().splitlines()[0].startswith("label,var,side,type")
    import json

    assert json.loads(xc.to_json())["host"] == "UR"


def test_exchange_variables_and_addresses_match_the_plcopen_globals() -> None:
    scene = plc_master_cell()
    globals_ = plcopen_globals(scene)
    for host in ("PLC1", "NEAR"):
        xc = bt.vc.exchange(scene, host)
        assert xc.points, host
        for p in xc.points:
            assert p.var in globals_, (host, p.label, p.var)
            ty, address = globals_[p.var]
            assert ty == p.value_type, (host, p.label)
            # The export puts `AT` addresses on PLC-family nodes only; a
            # bound point of the exchange carries its channel's address
            # regardless, so the two agree wherever the export says one.
            if address is not None:
                assert p.address == address, (host, p.label)


def test_exchange_numbers_the_remote_robots_commands_like_the_export() -> None:
    scene = plc_master_cell()
    xc = bt.vc.exchange(scene, "PLC1")
    entries = {(robot, label): number for robot, number, _key, label in xc.entries}
    # Motions keep the scene-order numbering; both arms are remote from PLC1.
    assert entries[("near", "near_go")] == 1
    assert entries[("far", "far_go")] == 1 and entries[("far", "far_home")] == 2
    # `far` runs two commands from PLC1 → a program word; `near` one → none.
    labels = {p.label for p in xc.points}
    assert "far.program" in labels and "near.program" not in labels
    assert {"near.start", "far.start"} <= labels
    assert "far.done" in labels
    # The export spells the same numbers.
    xml = scene.plcopen()
    assert "far_program := 1; (* far_go *)" in xml
    assert "far_program := 2; (* far_home *)" in xml
    # NEAR's exchange sees its handshake input from PLC1 and far's done.
    near = bt.vc.exchange(scene, "NEAR")
    sides = {p.label: p.side for p in near.points}
    assert sides["carrying"] == "output"  # NEAR reads what PLC1 writes
    assert sides["far.done"] == "output"
    with pytest.raises(ValueError, match="not a host"):
        bt.vc.exchange(scene, "nobody")


# ------------------------------------------------------- loopback = bake


def test_loopback_reproduces_the_pick_cell_bake() -> None:
    scene = pick_cell()
    bake = scene.simulate_sequence("pick")
    with bt.vc.loopback(scene, ["pick"], host="UR") as s:
        s.run()
        tl = s.timeline()
        rec = s.recording()
    same_timeline(bake, tl)
    # The recording is the controller's side of the cycle: the conveyor
    # coil and the vacuum valve, with their times.
    labels = {label for _, _, label, _ in rec.records}
    assert {"conv", "vacuum"} <= labels
    assert rec.trace().edges("conv")[0]  # it rose at least once


def test_loopback_reproduces_the_two_controller_cell_from_either_side() -> None:
    scene, names = two_controller_cell()
    bake = scene.simulate_sequences(names)
    # The PLC as the external controller: robot commands by start pulse and
    # program word, the belt by its coil, the handshake signal by its wire.
    with bt.vc.loopback(scene, names, host="PLC1") as s:
        s.run()
        same_timeline(bake, s.timeline())
        rec = s.recording()
    labels = {label for _, _, label, _ in rec.records}
    assert {"far.start", "far.program", "belt", "carrying", "cycle_done"} <= labels
    words = [v for _, _, label, v in rec.records if label == "far.program" and v]
    assert words == [1, 2]  # far_go, then far_home — the export's numbers
    # The robot controller as the external host (its own robot's motion
    # fires directly; the handshake it reads comes from the plant).
    with bt.vc.loopback(scene, names, host="NEAR") as s:
        s.run()
        same_timeline(bake, s.timeline())
    # Both at once.
    with bt.vc.loopback(scene, names, host=["PLC1", "NEAR"]) as s:
        s.run()
        same_timeline(bake, s.timeline())


def test_loopback_reproduces_the_machine_tending_bake() -> None:
    scene, names = tending_cell()
    bake = scene.simulate_sequences(names, max_duration=240.0)
    for hosts in ("vmc/cnc", "<arm>", ["vmc/cnc", "<arm>"]):
        with bt.vc.loopback(scene, names, host=hosts, max_duration=240.0) as s:
            s.run()
            same_timeline(bake, s.timeline())


@pytest.mark.skipif(
    not (EXAMPLES / "basics" / "demo.py").exists(), reason="the basics demo is not here"
)
def test_loopback_reproduces_the_sequence_demo() -> None:
    pytest.importorskip("huggingface_hub", reason="the demo cell orders from the catalog")
    try:
        import sequence_demo as sd
        from demo import build_scene

        scene = build_scene()
    except Exception as e:  # pragma: no cover - offline CI
        pytest.skip(f"cannot build the demo cell here: {e}")
    name = sd.build_cycle(scene)
    bake = scene.simulate_sequence(name)
    with bt.vc.loopback(scene, [name], host="<panda>") as s:
        s.run()
        same_timeline(bake, s.timeline())


# ------------------------------------------------------- replay = loopback


def test_replay_reproduces_the_loopback_with_the_host_external() -> None:
    scene, names = tending_cell()
    with bt.vc.loopback(scene, names, host="vmc/cnc", max_duration=240.0) as s:
        s.run()
        reference = s.timeline()
        rec = s.recording()
    # The machine's controller is now outside: its programs are not
    # scanned, its writes come from the recording — the plant must follow
    # the same path. The external program's step bands are the
    # controller's own and do not exist on the plant side.
    with bt.vc.Session(scene, names, links={"vmc/cnc": bt.vc.replay(rec)},
                       max_duration=240.0) as s:
        s.run(for_=reference.duration)
        replayed = s.timeline()
    same_timeline(reference, replayed, skip_programs=("vmc",), tol=1e-9)
    assert not any(span[0].startswith("vmc/") for span in replayed.step_spans)
    # A recording survives a file.
    again = bt.vc.Recording.from_csv(rec.to_csv())
    assert again.records == rec.records
    again = bt.vc.Recording.from_json(rec.to_json())
    assert again.records == rec.records


def test_replay_drives_the_remote_robot_by_start_pulse_and_program_word() -> None:
    scene, names = two_controller_cell()
    with bt.vc.loopback(scene, names, host="PLC1") as s:
        s.run()
        reference = s.timeline()
        rec = s.recording()
    with bt.vc.Session(scene, names, links={"PLC1": bt.vc.replay(rec)}) as s:
        s.run(for_=reference.duration)
        replayed = s.timeline()
    same_timeline(reference, replayed, skip_programs=("line",), tol=1e-9)


def test_a_magazine_commanded_from_outside_is_refused_by_name() -> None:
    # `belt` starts the magazine, a cosmetic device: an external PLC has no
    # coil for it, so the plant refuses to run blind — and says what to do.
    scene = plc_master_cell()
    with pytest.raises(ValueError, match="magazine"):
        bt.vc.Session(scene, ["pick", "belt"], hosts=["PLC1"])
    # Declared an output, the feeder has a run coil and crosses like a belt.
    scene.declare_io("magazine", role="output")
    xc = bt.vc.exchange(scene, "PLC1", ["pick", "belt"])
    assert any(p.label == "magazine" and p.side == "input" for p in xc.points)


# ------------------------------------------------- the hand on the wire


def test_an_injected_fault_steers_the_branch_and_the_diff_names_it() -> None:
    scene = pick_cell()
    baseline = scene.simulate_sequence("pick")
    ng = scene.simulate_sequence("pick", scenario="ng_part")  # spec_ok starts false
    with bt.vc.loopback(scene, ["pick"], host="UR") as s:
        s.inject(bt.io.stuck("spec_ok", False))
        s.run()
        tl = s.timeline()
    # A contact stuck low from the start is the NG scenario, bit for bit.
    same_timeline(ng, tl)
    # The diff against the design names where the two parted: `spec_ok`
    # itself is a level on both sides (no edge to miss), the vacuum the
    # design switched on never did.
    d = baseline.diff(bt.trace.from_timeline(tl))
    assert not d.ok
    assert any(f["signal"] == "vacuum" and f["code"] == "missing_edge" for f in d.findings())


def test_force_and_release_pin_a_lane_and_let_it_go() -> None:
    scene = pick_cell()
    with bt.vc.loopback(scene, ["pick"], host="UR") as s:
        s.tick()  # the first scan: the belt starts, the part rides toward the beam
        s.force("part_at_pick", True)  # the beam trips at once
        assert s.signal("part_at_pick") is True
        s.run(for_=0.1)
        assert s.signal("part_at_pick") is True
        s.unforce("part_at_pick")
        s.tick()
        # The geometry speaks again: the part is not at the beam yet.
        assert s.signal("part_at_pick") is False
    with pytest.raises(ValueError, match="not a lane"):
        with bt.vc.loopback(scene, ["pick"], host="UR") as s:
            s.force("nothing", True)


def test_anchored_injection_waits_for_its_moment() -> None:
    scene = pick_cell()
    bake = scene.simulate_sequence("pick")
    with bt.vc.loopback(scene, ["pick"], host="UR") as s:
        # Open the gauge's wire when the part arrives: the check then reads
        # the NC-less default — false — and the part is rejected.
        s.inject(bt.io.open("spec_ok"), at=bt.vc.rising("part_at_pick"))
        s.run()
        tl = s.timeline()
    assert tl.signals != bake.signals
    assert tl.signal("spec_ok").value_at(tl.duration) is False
    spec = [e for e in tl.signals if e[0] == "spec_ok"][0][1]
    beam = [e for e in bake.signals if e[0] == "part_at_pick"][0][1]
    # It fell exactly when the beam rose, not before.
    assert spec[-1][0] == beam[1][0]


# -------------------------------------------------------------- refusals


def test_plant_refusals_name_the_problem() -> None:
    scene = pick_cell()
    # The robot's own controller cannot be external without a joint stream.
    with pytest.raises(ValueError, match="stream its joints"):
        scene.open_plant(["pick"], ["UR"])
    with pytest.raises(ValueError, match="not a host"):
        scene.open_plant(["pick"], ["PLC9"], loopback=True)
    with pytest.raises(ValueError, match="external host"):
        scene.open_plant(["pick"], [])
    live = scene.open_plant(["pick"], ["UR"], loopback=True)
    with pytest.raises(ValueError, match="not an input"):
        live.apply("UR", {"part_at_pick": True})
    with pytest.raises(ValueError, match="not an external host"):
        live.outputs("PLC1")
    # A session needs something to answer to.
    with pytest.raises(ValueError, match="external host"):
        bt.vc.Session(scene, ["pick"])


def test_session_predicates() -> None:
    scene = pick_cell()
    with bt.vc.loopback(scene, ["pick"], host="UR") as s:
        t = s.run(until=bt.vc.rising("part_at_pick"), timeout=30.0)
        assert s.signal("part_at_pick") is True and t > 0.0
        s.run(until=bt.vc.level("vacuum", True), timeout=30.0)
        assert s.signal("vacuum") is True
        s.run(until=bt.vc.elapsed(0.5))
        s.run(until=bt.vc.done("pick"), timeout=60.0)
        assert s.finished
        with pytest.raises(TimeoutError):
            s.run(until=lambda _s: False, timeout=0.05)


# ------------------------------------------------------- grasp on the coil


def coil_pick_cell() -> bt.Scene:
    """The pick cell grasping by its coil: `vacuum` takes and releases
    `part`, the program never attaches (design-virtual-commissioning.md VC2
    record: a PLC has no attach command)."""
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene, coil_grasp=True)
    pick.wire_cell(scene)
    return scene


def test_a_grasp_bound_to_its_coil_does_what_the_attach_did() -> None:
    by_action = pick_cell().simulate_sequence("pick")
    scene = coil_pick_cell()
    by_coil = scene.simulate_sequence("pick")
    # Same signals, same part trajectory: the coil's edges stand in for the
    # attach / detach steps (the grasp is at the coil's rise, 0.3 s before
    # the old `hold` step, while the arm stands still).
    assert by_coil.signals == by_action.signals
    for t in (0.0, 5.7, 6.5, 7.0, by_action.duration):
        pos, quat = by_coil.object_pose("part", t)
        ref_pos, ref_quat = by_action.object_pose("part", t)
        assert pos == pytest.approx(ref_pos, abs=1e-9) and quat == pytest.approx(ref_quat, abs=1e-9)
    assert [b["signal"] for b in scene.grasp_bindings()] == ["vacuum"]
    import export_urscript as pick

    # The coil takes the part only within the suction face's reach.
    assert scene.grasp_bindings()[0]["parts"] == ["part"]
    assert scene.grasp_bindings()[0]["reach_m"] == pytest.approx(pick.GRASP_REACH)
    # No attach anywhere a controller would see it: the PLCopen file calls no
    # grasp FB, and a PLC driving the arm numbers the motions only.
    assert "FB_Attach" not in scene.plcopen() and "FB_Detach" not in scene.plcopen()
    scene.add_io_node("UR", kind="robot_controller", robots=["simple_arm"], channels=bt.io.ur_standard())
    scene.add_io_node("PLC1", kind="plc", programs=["pick"],
                      channels=bt.io.di8(base="%IX100.0") + bt.io.do8(base="%QX100.0") + bt.io.word(count=2, base="%QW100"))
    scene.auto_assign_io()
    xc = bt.vc.exchange(scene, "PLC1")
    assert sorted(label for _, _, _key, label in xc.entries) == [
        "clear_bin", "clear_tray", "home", "pick_down", "place", "to_pick", "to_reject"]
    vacuum = next(p for p in xc.points if p.label == "vacuum")
    assert vacuum.note is not None and "grasp of `simple_arm`" in vacuum.note and "`part`" in vacuum.note
    assert "grasp of `simple_arm`" in xc.to_markdown()
    # The loopback through that exchange is the bake, part and all.
    with bt.vc.loopback(scene, ["pick"], host="PLC1") as s:
        s.run()
        tl = s.timeline()
    assert tl.signals == by_coil.signals
    assert tl.object_pose("part", tl.duration)[0] == pytest.approx(by_coil.object_pose("part", by_coil.duration)[0], abs=1e-9)


def test_an_authored_attach_on_a_coil_bound_tool_is_refused() -> None:
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene)  # attach / detach steps
    scene.bind_grasp("vacuum", parts=["part"])
    with pytest.raises(ValueError, match="bind_grasp"):
        scene.simulate_sequence("pick")
    with pytest.raises(ValueError, match="parts=.*reach_m="):
        scene.bind_grasp("vacuum")
    with pytest.raises(ValueError, match="unknown signal"):
        scene.bind_grasp("suction", parts=["part"])


def test_dropping_the_coil_by_force_releases_the_part() -> None:
    """A PLC force on the coil is a real level on the plant: forced low at
    the grasp, the cup lets go, the arm travels empty and the beam never
    clears — which the diff against the bake names."""
    scene = coil_pick_cell()
    bake = scene.simulate_sequence("pick")
    with bt.vc.loopback(scene, ["pick"], host="UR") as s:
        s.run(until=bt.vc.level("vacuum", True), timeout=30.0)
        gripped_at = s.t
        part_at_grip = s.timeline().object_pose("part", gripped_at)[0]
        s.force("vacuum", False)
        s.run()
        tl = s.timeline()
    beam = [edges for name, edges in tl.signals if name == "part_at_pick"][0]
    assert beam[-1][1] is True, "the part never left the beam"
    assert tl.object_pose("part", tl.duration)[0] == pytest.approx(part_at_grip, abs=1e-9)
    # The plant's own record against the bake: the beam the bake cleared
    # never cleared here.
    d = bake.diff(bt.trace.from_timeline(tl), tolerance=0.05)
    assert any(f["signal"] == "part_at_pick" and f["code"] == "missing_edge" for f in d.findings())


def test_a_coil_grasp_survives_the_project_file(tmp_path) -> None:
    scene = coil_pick_cell()
    scene.save_project(tmp_path / "pick.botrail")
    again = bt.Scene.load_project(tmp_path / "pick.botrail")
    assert again.grasp_bindings() == scene.grasp_bindings()
    assert again.simulate_sequence("pick").signals == scene.simulate_sequence("pick").signals


# ------------------------------------------------- VC3: the hand on the plant


def panel_cell() -> bt.Scene:
    """The OpenPLC demo's cell: the pick cell with a start button, a fault
    lamp and a timeout arm, driven by a monitoring PLC."""
    from pathlib import Path
    import importlib.util

    path = Path(__file__).resolve().parents[2] / "examples" / "commissioning" / "openplc_pick_cell" / "run_demo.py"
    spec = importlib.util.spec_from_file_location("openplc_pick_cell_demo", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.build()


def test_a_breakpoint_holds_the_plant_and_a_step_scans_once() -> None:
    scene = coil_pick_cell()
    bake = scene.simulate_sequence("pick")
    rose = next(t for t, v in dict(bake.signals)["part_at_pick"] if v)
    with bt.vc.loopback(scene, ["pick"], host="UR", studio=False) as s:
        s.break_on("part_at_pick")
        s.run(until=lambda s: s.paused, timeout=30.0)
        assert s.paused and s.halted is not None and s.halted.startswith("↑part_at_pick")
        assert s.t == pytest.approx(rose, abs=1e-9)
        held = s.t
        s.step(3)
        assert not s.paused  # scans are owed
        s.run(until=lambda s: s.paused, timeout=1.0)
        assert s.t == pytest.approx(held + 3 * s.dt, abs=1e-9)
        s.clear_break("part_at_pick")
        assert s.breaks == []
        s.resume()
        s.run()
        tl = s.timeline()
    # Holding and stepping never touched the plant's own clock: the run is
    # the bake, bit for bit.
    assert tl.signals == bake.signals


def test_a_press_holds_a_lane_for_its_time_and_lets_go() -> None:
    scene = coil_pick_cell()
    with bt.vc.loopback(scene, ["pick"], host="UR", studio=False) as s:
        s.press("part_at_pick", 0.25)
        assert s.forced == {"part_at_pick": True}
        s.run(for_=0.5)
        assert s.forced == {}
        s.force("part_at_pick", True)
        s.run(for_=0.2)
        assert s.forced == {"part_at_pick": True}
        s.unforce("part_at_pick")
        s.run(for_=0.2)
        tl = s.timeline()
    lane = dict(tl.signals)["part_at_pick"]
    # The beam reads low with the part upstream: the press is a quarter
    # second high, the later force holds it until released, and the sensor
    # follows its geometry again the scan after the force is lifted (a
    # release lands between scans; the next one reads the world).
    assert [v for _, v in lane[:4]] == [True, False, True, False]
    for (t, _), want in zip(lane[:4], (0.0, 0.25, 0.5, 0.7)):
        assert want - 1e-9 <= t <= want + s.dt + 1e-9, (t, want)


def test_a_recorded_clip_is_the_stretch_retimed() -> None:
    scene = coil_pick_cell()
    bake = scene.simulate_sequence("pick")
    with bt.vc.loopback(scene, ["pick"], host="UR", studio=False) as s:
        s.run(until=bt.vc.rising("vacuum"), timeout=30.0)
        start = s.t
        s.record()
        assert s.recording_from == pytest.approx(start)
        s.run(for_=2.0)
        clip = s.stop_record(publish=False)
        assert s.recording_from is None and s.clips == [clip]
        assert s.stop_record() is None
        s.run()
    assert clip.duration == pytest.approx(2.0, abs=1e-9)
    lanes = dict(clip.signals)
    # The coil is high throughout the clip: its level at the cut becomes the
    # initial level, with no edge inside.
    assert lanes["vacuum"] == [(0.0, True)]
    for t in (0.0, 0.5, 1.3, 2.0):
        assert clip.sample(t) == pytest.approx(bake.sample(start + t), abs=1e-9)
        pos, _ = clip.object_pose("part", t)
        ref, _ = bake.object_pose("part", start + t)
        assert pos == pytest.approx(ref, abs=1e-9)
    names = [name for name, _, _ in clip.step_spans]
    assert "grip" in names and "feed" not in names
    assert clip.step_spans[0][1] == 0.0 and clip.step_spans[-1][2] == pytest.approx(2.0, abs=1e-9)
    # `tl.clip` cuts the same stretch out of the bake.
    cut = bake.clip(start, start + 2.0)
    assert cut.signals == clip.signals and cut.step_spans == clip.step_spans
    assert cut.sample(1.0) == pytest.approx(clip.sample(1.0), abs=1e-9)
    with pytest.raises(ValueError, match="after start"):
        bake.clip(3.0, 2.0)


def test_a_monitoring_plc_reports_its_step_word() -> None:
    scene = panel_cell()
    xc = bt.vc.exchange(scene, "PLC1")
    step = next(p for p in xc.inputs if p.label == "pick.step")
    assert step.value_type == "INT" and step.source == "monitor" and step.initial == -1
    assert step.address is not None and step.address.startswith("%QW")
    st = scene.st("PLC1", cycle=False)
    assert "pick_step AT %QW" in st and "pick_step := 0;" in st and "pick_step := 1;" in st
    assert "pick_step := -1;" in st  # the end of the cycle parks the word
    # The plant records the word as the controller's step bands: here the
    # controller is by hand, so nothing else happens in the cell.
    with bt.vc.Session(scene, ["pick"], hosts=["PLC1"], studio=False) as s:
        s.apply("PLC1", {"pick.step": 0})
        s.run(for_=0.5)
        s.apply("PLC1", {"pick.step": 1})
        s.run(for_=0.5)
        s.apply("PLC1", {"pick.step": 2})
        s.run(for_=0.2)
        s.apply("PLC1", {"pick.step": -1})
        s.run(for_=0.2)
        tl = s.timeline()
    # A write lands at the host's slot in the next scan, so each band
    # changes one scan after the word was handed in.
    spans = [(name, round(a, 2), round(b, 2)) for name, a, b in tl.step_spans]
    assert spans == [("ready", 0.0, 0.51), ("feed", 0.51, 1.01), ("arrival", 1.01, 1.21)]
    # A step inside an arm is that select's decision: the chart dims the
    # other arm.
    with bt.vc.Session(scene, ["pick"], hosts=["PLC1"], studio=False) as s:
        s.apply("PLC1", {"pick.step": 2})
        s.run(for_=0.1)
        # Flat order: ready 0, feed 1, arrival 2, halt 3, descend 4, grip 5,
        # judge 6, place 7, release 8, to reject 9, drop 10, no part 11,
        # return 12.
        s.apply("PLC1", {"pick.step": 11})  # `no part`, the timeout arm's first step
        s.run(for_=0.1)
        tl = s.timeline()
    assert [name for name, _, _ in tl.step_spans] == ["arrival", "no part"]
    assert "arrival" in [name for name, _, _ in tl.step_spans]


def test_the_demo_cell_takes_the_timeout_arm_when_the_beam_is_forced_low() -> None:
    scene = panel_cell()
    assert scene.io_report().ok, scene.io_report()
    # The reference run: the operator presses start half a second in.
    with bt.vc.loopback(scene, ["pick"], host="PLC1", studio=False) as s:
        s.run(for_=0.5)
        s.press("panel/cycle_start")
        s.run()
        good = s.timeline()
    names = [name for name, _, _ in good.step_spans]
    assert "halt" in names and "no part" not in names and dict(good.signals)["fault"] == [(0.0, False)]
    # The beam forced low: the part passes unseen, the PLC's timer runs out,
    # the fault lamp comes on and the arm goes home.
    with bt.vc.loopback(scene, ["pick"], host="PLC1", studio=False) as s:
        s.force("part_at_pick", False)
        s.run(for_=0.5)
        s.press("panel/cycle_start")
        s.run()
        bad = s.timeline()
        status = s.status()
    names = [name for name, _, _ in bad.step_spans]
    assert "no part" in names and "halt" not in names
    fault = dict(bad.signals)["fault"]
    assert fault[-1][1] is True
    assert status["buttons"] == ["panel/cycle_start"] and status["forced"] == [["part_at_pick", False]]
    assert status["hosts"] == [{"name": "PLC1", "link": "loopback", "exchanges": 0, "error": None}]
    assert [f["signal"] for f in scene.lamps()] == ["fault"]


def test_the_lamp_and_the_monitor_survive_the_project_file(tmp_path) -> None:
    scene = panel_cell()
    scene.save_project(tmp_path / "panel.botrail")
    again = bt.Scene.load_project(tmp_path / "panel.botrail")
    assert again.lamps() == scene.lamps()
    assert bt.vc.exchange(again, "PLC1").to_json() == bt.vc.exchange(scene, "PLC1").to_json()
    code = scene.generate_python()
    assert 'monitor=True' in code and 'scene.add_lamp("panel/fault", signal="fault", obstacle="panel/fault/lens"' in code
