"""The derived I/O map (I0 of design/design-electrical.md).

`scene.io_points()` lists the I/O a cell needs to be built, derived from
how the sequences use the scene's names — nothing authored. These tests
pin the seven derivation rules on a hand-built cell and the golden lists of
the shipped demos (the ones whose assets are cached), so a rule change
shows up as a diff in the table, not as a surprise on the drawing.
"""

import os
import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

CACHE = Path(os.environ.get("BOTRAIL_CACHE_DIR") or Path.home() / ".cache" / "botrail")
HAS_FRANKA = (CACHE / "assets" / "franka" / "franka.usd").exists()
HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
HAS_CATALOG = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))


def points_by_label(scene, sequences=None) -> dict:
    """`{(label, direction): IoPoint}` — the table keyed the way it reads."""
    return {(p.label, p.direction): p for p in scene.io_points(sequences=sequences)}


# ------------------------------------------------------------- pick cell


@pytest.fixture()
def pick_cell():
    """`examples/export_urscript.py`: the cell that already carries its
    I/O list as a hand-written dict — the list the derivation must match."""
    import export_urscript as demo

    scene = demo.build_cell()
    demo.author_sequence(scene)
    return demo, scene


def test_pick_cell_derives_the_hand_written_io_list(pick_cell) -> None:
    demo, scene = pick_cell
    table = points_by_label(scene)
    # The dict on the drawing: inputs are contacts the program waits on,
    # outputs are coils it drives. Same names, same directions.
    assert {k for k in table if k[1] == "input"} == {(n, "input") for n in demo.INPUTS}
    assert {k for k in table if k[1] == "output"} == {(n, "output") for n in demo.OUTPUTS}
    beam = table[("part_at_pick", "input")]
    assert (beam.kind, beam.source) == ("DI", "sensor")
    # One robot driven → the program lives on that robot's controller.
    assert beam.host == "<simple_arm>"
    assert [name for _, _, name in beam.readers] == ["await part"]
    spec = table[("spec_ok", "input")]
    assert spec.source == "signal:read-only"  # a contact nobody writes: external input
    conv = table[("conv", "output")]
    assert (conv.kind, conv.source, conv.status) == ("DO", "device:run", "unbound")
    assert [name for _, _, name in conv.writers] == ["feed", "halt"]
    vacuum = table[("vacuum", "output")]
    assert vacuum.source == "signal:write-only"  # a coil candidate
    assert scene.io_report().findings == []
    assert scene.io_report().ok


def test_step_refs_carry_flat_indices(pick_cell) -> None:
    _, scene = pick_cell
    table = points_by_label(scene)
    # `await part` is the second step of `pick`; the tuple is
    # (sequence, flat index, name) — index first, name for display.
    assert table[("part_at_pick", "input")].readers == [("pick", 1, "await part")]
    # Branch arms read at the branching step: `judge` is flat step 5.
    (seq, index, name), = table[("spec_ok", "input")].readers
    assert (seq, name) == ("pick", "judge")
    assert index == 5


def test_io_list_formats(pick_cell, tmp_path: Path) -> None:
    _, scene = pick_cell
    csv = scene.io_list("csv")
    header, *rows = [l for l in csv.splitlines() if not l.startswith("#")]
    assert header.startswith("name,aspect,direction,kind,source,host,")
    assert len(rows) == 4
    assert csv.rstrip().splitlines()[-1] == "# <simple_arm>: DI 2, DO 2"
    md = scene.io_list("md")
    assert md.startswith("| name | aspect | direction |")
    assert "- `<simple_arm>`: DI 2, DO 2" in md
    import json

    data = json.loads(scene.io_list("json"))
    assert data["sequences"] == ["pick"]
    assert {p["name"] for p in data["points"]} == {"part_at_pick", "spec_ok", "conv", "vacuum"}
    assert data["summary"] == {"<simple_arm>": {"DI": 2, "DO": 2}}
    assert data["findings"] == []
    for ext in ("csv", "md", "json"):
        path = tmp_path / f"io.{ext}"
        scene.export_io_list(path)
        assert path.read_text(encoding="utf-8") == scene.io_list(ext)
    with pytest.raises(ValueError, match="unknown I/O list format"):
        scene.io_list("xlsx")
    with pytest.raises(ValueError, match="unknown extension"):
        scene.export_io_list(tmp_path / "io.txt")
    with pytest.raises(ValueError, match="unknown sequence"):
        scene.io_points(sequences=["nope"])


# ------------------------------------------------------- the seven rules


def two_arm_cell() -> bt.Scene:
    """Two simple arms in one program (→ the implicit `<cell>` host), a
    belt program, and a watcher on one arm's own controller."""
    arm = bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf")
    scene = bt.Scene(arm, name="near")
    scene.add_robot(arm, name="far", base_position=(1.5, 0.0, 0.0))
    for robot in ("near", "far"):
        scene.add_segment(f"{robot}_go", goal=[0.4, 0.4, -0.5, 0.2, 0.0, 0.0], robot=robot)
        scene.add_segment(f"{robot}_home", goal=[0.0] * 6, robot=robot)
    scene.add_box("part", size=(0.05, 0.05, 0.05), position=(0.5, 0.3, 0.03))
    scene.add_conveyor("belt", zone_position=(0.5, 0.3, 0.05), zone_size=(1.0, 0.2, 0.1),
                       velocity=(0.1, 0.0, 0.0), running=False)
    scene.add_conveyor("always_on", zone_position=(3.0, 3.0, 0.05), zone_size=(0.2, 0.2, 0.1),
                       velocity=(0.1, 0.0, 0.0), running=True)
    scene.add_beam_sensor("beam", frm=(0.5, 0.2, 0.03), to=(0.5, 0.4, 0.03), watch=["part"])
    scene.add_zone_sensor("lonely", position=(2.0, 2.0, 0.1), size=(0.2, 0.2, 0.2))
    scene.add_source("magazine", pool=["part"], park=(2.0, 0.0, 0.03), pitch=(0.1, 0.0, 0.0),
                     position=(0.3, 0.3, 0.03), interval=0.0)
    for name in ("carrying", "belt_ok", "spare", "estop_ok"):
        scene.define_signal(name)
    return scene


def author_two_arm(scene: bt.Scene) -> None:
    pick = scene.sequence("pick")
    pick.step("go", actions=[bt.seq.motion("near_go"), bt.seq.motion("far_go")])
    pick.step("carry", actions=[bt.seq.motion("far_home"), bt.seq.set_signal("carrying")],
              transition=bt.seq.all_of(bt.seq.done(), bt.seq.signal("belt_ok")))
    pick.step("check", transition=bt.seq.signal("estop_ok"))
    belt = scene.sequence("belt")
    belt.step("run", actions=[bt.seq.start("belt"), bt.seq.start("magazine"),
                              bt.seq.set_signal("belt_ok")],
              transition=bt.seq.signal("carrying"))
    watch = scene.sequence("watch")
    watch.step("wait", actions=[bt.seq.motion("near_go")], transition=bt.seq.rising("carrying"))
    watch.step("idle", transition=bt.seq.robot_done("far"))


def test_rules_on_a_two_arm_cell() -> None:
    scene = two_arm_cell()
    author_two_arm(scene)
    table = points_by_label(scene)

    # ⑥ `pick` drives both arms → it lands on <cell> with start/done per arm;
    # `far` runs two motions from there → a program-number word.
    for robot in ("near", "far"):
        assert table[(f"{robot}.start", "output")].host == "<cell>"
        assert table[(f"{robot}.done", "input")].kind == "DI"
    assert table[("far.program", "output")].kind == "Word"
    assert ("near.program", "output") not in table
    # `watch` drives only `near` → it lives on <near>: no near.* points there,
    # but it reads `robot_done("far")` from another host → far.done on <near>.
    hosts_of_far_done = {p.host for p in scene.io_points() if p.label == "far.done"}
    assert hosts_of_far_done == {"<cell>", "<near>"}
    # ② `carrying`: written by pick (<cell>), read by belt (<cell>) and by
    # watch (<near>) → a handshake wire, out on <cell>, in on <near>.
    out = table[("carrying", "output")]
    assert (out.source, out.host, out.status) == ("signal:handshake", "<cell>", "unbound")
    inp = table[("carrying", "input")]
    assert (inp.source, inp.host) == ("signal:handshake", "<near>")
    assert [(s, n) for s, _, n in inp.readers] == [("watch", "wait")]
    # ② `belt_ok`: written and read on <cell> only → a relay, no I/O.
    assert table[("belt_ok", "output")].status == "internal"
    # ④ `estop_ok`: read, never written → an external-input candidate.
    assert table[("estop_ok", "input")].source == "signal:read-only"
    # ① sensors: read → on the reader's host; unread → host None.
    assert table[("beam", "input")].host is None  # nobody reads it here
    assert table[("lonely", "input")].host is None
    # ⑤ devices: a run coil, a constant belt, a cosmetic magazine.
    assert table[("belt", "output")].source == "device:run"
    assert table[("always_on", "output")].status == "constant"
    mag = table[("magazine", "output")]
    assert mag.status == "cosmetic"
    assert [n for _, _, n in mag.writers] == ["run"]  # commanded, still cosmetic
    # ③ `spare` is defined but never used → not a point, an info finding.
    assert ("spare", "output") not in table

    report = scene.io_report()
    codes = sorted(f.code for f in report.findings)
    assert codes.count("implicit_host") == 2  # pick, belt
    assert "unreferenced" in codes and "word_unexpressible" in codes
    assert report.errors() == []
    unref = [f for f in report.infos() if f.code == "unreferenced"]
    assert any("`spare`" in f.message for f in unref)
    # A program set narrows the walk: without `watch`, `carrying` is a relay.
    narrowed = points_by_label(scene, sequences=["pick", "belt"])
    assert narrowed[("carrying", "output")].status == "internal"
    assert ("carrying", "input") not in narrowed


def test_name_clash_is_a_warning() -> None:
    scene = two_arm_cell()
    author_two_arm(scene)
    scene.define_signal("belt")  # a signal named like the conveyor
    scene.define_signal("agv.station")
    report = scene.io_report()
    clashes = [f for f in report.warnings() if f.code == "name_clash"]
    assert len(clashes) == 2
    assert not report.errors()  # warnings first; errors in a later version
    assert "warning name_clash" in str(clashes[0])


# ---------------------------------------------------- shipped demo goldens


def test_signal_track_kind() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    scene.add_box("part", size=(0.05, 0.05, 0.05), position=(0.5, 0.3, 0.03))
    scene.add_conveyor("belt", zone_position=(0.5, 0.3, 0.05), zone_size=(1.0, 0.2, 0.1),
                       velocity=(0.1, 0.0, 0.0), running=True)
    scene.add_beam_sensor("eye", frm=(0.6, 0.2, 0.03), to=(0.6, 0.4, 0.03), watch=["part"])
    scene.define_signal("seen")
    sq = scene.sequence("main")
    sq.step("wait", transition=bt.seq.signal("eye"))
    sq.step("mark", actions=[bt.seq.set_signal("seen"), bt.seq.stop("belt")])
    tl = sq.simulate()
    assert tl.signal("seen").kind == "signal"
    assert tl.signal("eye").kind == "sensor"
    assert tl.signal("belt").kind == "device"
    # `tl.signals` keeps its (name, edges) shape.
    assert [n for n, _ in tl.signals] == ["seen", "eye", "belt"]


@pytest.mark.skipif(not HAS_FRANKA, reason="Isaac Franka not in the botrail cache")
def test_golden_sequence_demo() -> None:
    import sequence_demo as sd
    from demo import build_scene

    scene = build_scene()
    sd.build_cycle(scene)
    table = points_by_label(scene)
    assert set(table) == {("beam_pick", "input"), ("carrying", "output"), ("conv", "output")}
    assert {p.host for p in table.values()} == {"<panda>"}
    assert table[("carrying", "output")].source == "signal:write-only"
    assert scene.io_report().findings == []


@pytest.mark.skipif(not HAS_FRANKA, reason="Isaac Franka not in the botrail cache")
def test_golden_dual_cell_demo() -> None:
    import dual_cell_demo as dc

    scene = dc.build_cell()
    dc.build_cycle(scene)
    # `clash` (the interlock-free variant) drives the same arms, so the
    # program set is named — as it would be for simulate_sequences.
    table = points_by_label(scene, sequences=["dual_pick"])
    real = {k: p for k, p in table.items() if p.status != "cosmetic"}
    assert set(real) == {
        ("beam_ahead", "input"), ("beam_pick", "input"),
        ("zone_near", "input"), ("zone_far", "input"),
        ("carrying_near", "output"), ("carrying_far", "output"),
        ("conv", "output"),
        ("near.start", "output"), ("near.done", "input"), ("near.program", "output"),
        ("far.start", "output"), ("far.done", "input"), ("far.program", "output"),
    }
    assert {p.host for k, p in real.items() if k[0] != "zone_near"} == {"<cell>"}
    assert real[("zone_near", "input")].host is None  # authored, read by nobody
    assert {k[0] for k, p in table.items() if p.status == "cosmetic"} == {
        "cartons", "cleats", "carton_out", "cleat_out"
    }
    codes = [f.code for f in scene.io_report(sequences=["dual_pick"]).findings]
    assert codes.count("implicit_host") == 1
    assert codes.count("word_unexpressible") == 2


@pytest.mark.skipif(not HAS_FRANKA, reason="Isaac Franka not in the botrail cache")
def test_golden_agv_cell_demo() -> None:
    import agv_cell_demo as ag

    scene = ag.build_scene()
    ag.build_cycle(scene)
    table = points_by_label(scene)
    assert set(table) == {
        ("agv", "input"), ("agv.dispatch", "output"), ("agv.station", "output"),
        ("dock_occupied", "input"), ("gate_zone", "input"), ("tray_loaded", "input"),
    }
    # One arm driven → everything sits on the robot's own controller and
    # `panda` gets no handshake points.
    assert {p.host for p in table.values()} == {"<panda>"}
    assert table[("agv.station", "output")].kind == "Word"
    assert [f.code for f in scene.io_report().findings] == ["word_unexpressible"]


# ------------------------------------------------ the assignment layer (I1)


def wired_pick_cell():
    import export_urscript as demo

    scene = demo.build_cell()
    demo.author_sequence(scene)
    demo.wire_cell(scene)
    return demo, scene


def test_wired_pick_cell_projects_the_same_script_as_the_dicts() -> None:
    demo, scene = wired_pick_cell()
    # Every point is bound on the UR node and the report is clean.
    assert all(p.status == "bound" and p.host == "UR" for p in scene.io_points())
    assert scene.io_report().findings == []
    runs = scene.simulate_scenarios(["pick"])
    projected = runs.to_script()
    explicit = runs.to_script(inputs=demo.INPUTS, outputs=demo.OUTPUTS)
    assert projected == explicit  # bit-identical: the bindings *are* the dicts
    # One bake alone still cannot carry the skipped arm, node or not.
    with pytest.raises(ValueError, match="never planned"):
        runs["baseline"].to_script(node="UR")
    # Explicit dicts win over the bindings, per key.
    swapped = runs.to_script(outputs={"vacuum": 7})
    assert "set_standard_digital_out(7, True)" in swapped
    assert "set_standard_digital_out(0, True)" in swapped  # conv still from the binding
    # The table carries the wiring columns.
    csv = scene.io_list("csv")
    assert "part_at_pick,,input,DI,sensor,UR,UR,DI2,,,BEAM1," in csv
    assert scene.io_map().bindings == [
        ("part_at_pick", "input", "UR", "DI2"),
        ("spec_ok", "input", "UR", "DI3"),
        ("conv", "output", "UR", "DO0"),
        ("vacuum", "output", "UR", "DO1"),
    ]


def test_inverted_wire_flips_tests_and_writes_only_in_the_script() -> None:
    demo, scene = wired_pick_cell()
    plain = scene.simulate_scenarios(["pick"])
    baseline_cycle = plain["baseline"].duration
    plain_script = plain.to_script()
    # NC wiring for the gauge contact and the vacuum valve: the bake is
    # untouched, the script tests and writes the opposite level.
    scene.bind_input("spec_ok", "UR", "DI3", invert=True, contact="nc")
    scene.bind_output("vacuum", "UR", "DO1", invert=True)
    runs = scene.simulate_scenarios(["pick"])
    assert runs["baseline"].duration == baseline_cycle
    script = runs.to_script()
    assert "if (not get_standard_digital_in(3)):" in script  # branch guard, inverted
    assert "if (get_standard_digital_in(3)):" in plain_script
    assert "set_standard_digital_out(1, False)" in script.split("# step 4")[1].split("# step 5")[0]
    assert "set_standard_digital_out(1, True)" in plain_script.split("# step 4")[1].split("# step 5")[0]


def test_own_robot_done_lowers_to_nothing() -> None:
    demo, scene = wired_pick_cell()
    # A program that asks after its own robot: the idle test a blocking
    # controller has already passed — no input needed, nothing emitted
    # (it used to demand an input port for the robot's own name).
    sq = scene.sequence("park")
    sq.step("go", actions=[bt.seq.motion("to_pick")])
    sq.step("settle", transition=bt.seq.robot_done("simple_arm"))
    sq.step("home", actions=[bt.seq.motion("home")], transition=bt.seq.all_of(bt.seq.done(), bt.seq.robot_done("simple_arm")))
    tl = scene.simulate_sequence("park")
    script = tl.to_script()
    assert "# step 2: settle" in script
    assert "get_standard_digital_in" not in script
    # ...and the derivation lists no `simple_arm.done` point for it.
    assert not any(p.label == "simple_arm.done" for p in scene.io_points())


def test_unbound_and_wiring_lints() -> None:
    demo, scene = wired_pick_cell()
    scene.unbind_output("vacuum")
    report = scene.io_report()
    codes = [(f.severity, f.code) for f in report.findings]
    # A coil candidate without a channel is a warning, not an error.
    assert codes == [("warning", "unbound")]
    assert "vacuum" in report.warnings()[0].message
    scene.unbind_input("part_at_pick")
    assert [f.code for f in scene.io_report().errors()] == ["unbound"]
    assert scene.io_report().errors()[0].at == [("pick", 1, "await part")]
    # Two points on one channel; an input on a DO channel; a stale binding.
    scene.bind_input("part_at_pick", "UR", "DI3")
    scene.bind_output("vacuum", "UR", "DI5")
    scene.bind_output("ghost", "UR", "DO7")
    codes = sorted(f.code for f in scene.io_report().findings)
    assert "duplicate" in codes and "kind" in codes and "stale_binding" in codes
    # What the scene can see immediately is refused on the spot.
    with pytest.raises(ValueError, match="unknown I/O node"):
        scene.bind_input("part_at_pick", "PLC9", "DI0")
    with pytest.raises(ValueError, match="unknown channel"):
        scene.bind_input("part_at_pick", "UR", "DI99")
    with pytest.raises(ValueError, match="unknown robot"):
        scene.add_io_node("X", kind="robot_controller", robots=["nobody"])
    with pytest.raises(ValueError, match="robots="):
        scene.add_io_node("X", kind="robot_controller")
    with pytest.raises(ValueError, match="no binding"):
        scene.unbind_output("nothing")
    scene.remove_io_node("UR")
    assert scene.io_map().bindings == []
    assert scene.io_report().findings == []  # no node → nothing is being assigned


def test_declarations_override_the_rules() -> None:
    demo, scene = wired_pick_cell()
    scene.declare_io("spec_ok", role="internal")           # a constant, not a contact
    scene.declare_io("vacuum", role="exclude")             # a flag, off the table
    scene.declare_io("door_ch1", role="input", kind="safe_di", safety=True, pair="door_ch2")
    scene.declare_io("door_ch2", role="input", kind="safe_di", safety=True, pair="door_ch1")
    table = points_by_label(scene)
    assert table[("spec_ok", "output")].status == "internal"
    assert ("vacuum", "output") not in table
    door = table[("door_ch1", "input")]
    assert (door.kind, door.source, door.safety, door.status) == ("SafeDI", "declared", True, "unbound")
    codes = [f.code for f in scene.io_report().findings]
    assert "stale_binding" in codes    # the vacuum binding lost its point
    assert "unbound" in codes          # the safety inputs want a channel
    with pytest.raises(ValueError, match="unknown role"):
        scene.declare_io("x", role="sideways")
    scene.undeclare_io("vacuum")
    assert ("vacuum", "output") in points_by_label(scene)
    with pytest.raises(ValueError, match="no I/O declaration"):
        scene.undeclare_io("vacuum")


def test_plc_master_nodes_move_programs_and_take_uplinked_channels() -> None:
    scene = two_arm_cell()
    author_two_arm(scene)
    scene.add_io_node("PLC1", kind="plc", programs=["pick", "belt"],
                      channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"), place="panel")
    scene.add_io_node("RIO1", kind="remote_io", uplink=("PLC1", "PROFINET"),
                      channels=bt.io.di8(base="%IX1.0"), model="ET200SP")
    scene.add_io_node("NEAR", kind="robot_controller", robots=["near"], programs=["watch"],
                      channels=bt.io.ur_standard())
    points = scene.io_points()
    hosts = {p.label: p.host for p in points if p.status != "cosmetic"}
    assert hosts["belt"] == "PLC1"
    # `watch` now runs on NEAR — its handshake input from `pick` (on PLC1)
    # sits on NEAR; near's start/done from PLC1 mirror onto NEAR.
    assert {p.host for p in points if p.label == "carrying" and p.direction == "input"} == {"NEAR"}
    assert {(p.direction, p.host) for p in points if p.label == "near.start"} == {("output", "PLC1"), ("input", "NEAR")}
    assert {(p.direction, p.host) for p in points if p.label == "near.done"} == {("input", "PLC1"), ("output", "NEAR")}
    assert {(p.direction, p.host) for p in points if p.label == "far.done"} == {("input", "PLC1"), ("input", "NEAR")}
    # A remote station's channel takes a PLC1 point through the uplink;
    # a robot controller's channel does not.
    scene.bind_input("beam", "RIO1", "DI2", tag="Beam", field="-B1")
    scene.bind_output("belt", "NEAR", "DO0")
    codes = {f.code for f in scene.io_report().findings}
    assert "host_mismatch" in codes
    scene.unbind_output("belt", node="NEAR")
    scene.bind_output("belt", "PLC1", "DO0", tag="BeltRun")
    row = [l for l in scene.io_list("csv").splitlines() if l.startswith("beam,")][0]
    assert row.startswith("beam,,input,DI,sensor,,RIO1,DI2,%IX1.2,Beam,-B1,")  # unhosted sensor, bound on RIO1
    assert not any(f.code == "host_mismatch" for f in scene.io_report().findings)
    # A program that drives `near` alone lives on NEAR by itself; it
    # scripts once the partner contact it reads is bound there.
    solo = scene.sequence("solo")
    solo.step("go", actions=[bt.seq.motion("near_go")], transition=bt.seq.robot_done("far"))
    tl = scene.simulate_sequences(["solo"])
    with pytest.raises(ValueError, match="bind `far.done`"):
        tl.to_script()
    scene.bind_input("far.done", "NEAR", "DI1")
    script = tl.to_script(io=scene.io_map())
    assert "while (not get_standard_digital_in(1)):" in script
    # A program listed by two nodes is an error; an unknown program too.
    scene.add_io_node("PLC2", kind="plc", programs=["pick", "nope"])
    codes = [f.code for f in scene.io_report().errors()]
    assert "program_multihost" in codes and "unknown_ref" in codes


def test_io_override_projects_a_newer_map_onto_an_older_bake() -> None:
    demo, scene = wired_pick_cell()
    scene.remove_io_node("UR")
    runs = scene.simulate_scenarios(["pick"])
    with pytest.raises(ValueError, match="bind it on the robot controller node"):
        runs.to_script()  # the snapshot has no bindings
    demo.wire_cell(scene)
    assert runs.to_script(io=scene.io_map()) == runs.to_script(inputs=demo.INPUTS, outputs=demo.OUTPUTS)
    with pytest.raises(ValueError, match="unknown I/O node"):
        runs.to_script(node="PLC9", io=scene.io_map())


def test_io_map_survives_the_project_and_the_generated_python(tmp_path: Path) -> None:
    demo, scene = wired_pick_cell()
    scene.declare_io("estop_ok", role="input", kind="safe_di", safety=True)
    scene.bind_input("part_at_pick", "UR", "DI2", tag="PartAtPick", field="BEAM1", invert=True, contact="nc",
                     voltage=24, logic="pnp", note="M12")
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    again = bt.Scene.load_project(path)
    assert again.io_map().to_json() == scene.io_map().to_json()
    assert again.io_list("csv") == scene.io_list("csv")
    py = scene.generate_python()
    assert 'scene.add_io_node("UR", kind="robot_controller", robots=["simple_arm"], channels=json.loads(' in py
    assert 'scene.bind_input("part_at_pick", "UR", "DI2", tag="PartAtPick", field="BEAM1", invert=True, contact="nc", voltage=24, logic="pnp", note="M12")' in py
    assert 'scene.declare_io("estop_ok", role="input", kind="safedi", safety=True)' in py
    # Renaming the robot follows into the controller node.
    scene.rename_robot("simple_arm", "arm")
    assert scene.io_map().to_json().count('"arm"') == 1
    assert all(p.host == "UR" for p in scene.io_points() if p.source != "declared")


def test_channel_templates() -> None:
    di = bt.io.di8(base="%IX0.6")
    assert [c["address"] for c in di][:3] == ["%IX0.6", "%IX0.7", "%IX1.0"]
    assert bt.io.ur_standard()[9] == {"id": "DO1", "kind": "do", "port": 1}
    assert bt.io.safe_di8(base="%IX4.0")[0] == {"id": "SDI0", "kind": "safe_di", "address": "%IX4.0"}
    assert bt.io.word(2, base="%IW")[1] == {"id": "W1", "kind": "word", "address": "%IW1"}
    assert bt.io.channels("di", 2, prefix="X", voltage=24, logic="pnp")[1] == {
        "id": "X1", "kind": "di", "voltage": 24, "logic": "pnp"
    }


# ---------------------------------------- auto-assign, lints, topology (I2)


def test_auto_assign_fills_in_table_order_and_keeps_existing_bindings() -> None:
    demo, scene = wired_pick_cell()
    scene.remove_io_node("UR")
    scene.add_io_node("UR", kind="robot_controller", robots=["simple_arm"], channels=bt.io.ur_standard())
    scene.bind_input("spec_ok", "UR", "DI5")          # by hand — kept
    report = scene.auto_assign_io()
    assert report.findings == [], str(report)
    assert scene.io_map().bindings == [
        ("spec_ok", "input", "UR", "DI5"),
        ("conv", "output", "UR", "DO0"),
        ("part_at_pick", "input", "UR", "DI0"),
        ("vacuum", "output", "UR", "DO1"),
    ]
    # Same call again: nothing moves. Reassign renumbers the automatic
    # bindings only — the hand-made one keeps its channel.
    scene.auto_assign_io()
    assert len(scene.io_map().bindings) == 4
    scene.unbind_input("part_at_pick")
    scene.bind_input("part_at_pick", "UR", "DI3")
    scene.auto_assign_io(reassign=True)
    assert ("spec_ok", "input", "UR", "DI5") in scene.io_map().bindings
    assert ("part_at_pick", "input", "UR", "DI3") in scene.io_map().bindings
    assert [l.split(",")[-1] for l in scene.io_list("csv").splitlines() if l.startswith(("conv,", "spec_ok,"))] == ["auto", ""]
    # The projected script follows the assigned ports.
    runs = scene.simulate_scenarios(["pick"])
    assert "get_standard_digital_in(3)" in runs.to_script()   # part_at_pick on DI3 now


def test_capacity_polarity_voltage_and_safety_lints() -> None:
    demo, scene = wired_pick_cell()
    scene.remove_io_node("UR")
    # A tiny 24 V PNP module: two DIs, one DO, one safety DI.
    channels = bt.io.channels("di", 2, "DI", port_from=0, voltage=24, logic="pnp") \
        + bt.io.channels("do", 1, "DO", port_from=0) + bt.io.safe_di8()[:1]
    scene.add_io_node("UR", kind="robot_controller", robots=["simple_arm"], channels=channels)
    # A 12 V NPN sensor on the PNP module, a standard signal on the safety
    # channel, and a declared safety input nobody reads.
    scene.bind_input("part_at_pick", "UR", "DI0", voltage=12, logic="npn")
    scene.bind_input("spec_ok", "UR", "SDI0")
    scene.declare_io("estop_ok", role="input", safety=True)
    scene.bind_input("estop_ok", "UR", "DI1")
    report = scene.auto_assign_io()          # conv takes DO0; vacuum finds no DO left
    codes = sorted(f.code for f in report.warnings())
    assert codes == ["capacity", "polarity", "safety", "safety", "safety_unread", "unbound", "voltage"], codes
    assert any("needs 2 DO but has 1" in f.message for f in report.warnings())
    # A two-channel safety pair wired onto different kinds is an error.
    scene.declare_io("door_ch1", role="input", kind="safe_di", safety=True, pair="door_ch2")
    scene.declare_io("door_ch2", role="input", kind="safe_di", safety=True, pair="door_ch1")
    scene.add_io_node("SAFE", kind="safety_plc", uplink="UR", channels=bt.io.safe_di8(base="%IX9.0")[:2] + bt.io.di8()[:1])
    scene.bind_input("door_ch1", "SAFE", "SDI0")
    scene.bind_input("door_ch2", "SAFE", "DI0")
    assert "safety_pair" in [f.code for f in scene.io_report().errors()]
    scene.bind_input("door_ch2", "SAFE", "SDI1")
    assert "safety_pair" not in [f.code for f in scene.io_report().errors()]
    # Two programs driving one coil — the ownership rule without a bake.
    other = scene.sequence("other")
    other.step("also", actions=[bt.seq.set_signal("vacuum", False)])
    assert "multiple_drivers" in [f.code for f in scene.io_report().errors()]
    assert "multiple_drivers" not in [f.code for f in scene.io_report(sequences=["pick"]).errors()]


def test_topology_renders_the_wiring(tmp_path: Path) -> None:
    demo, scene = wired_pick_cell()
    mmd = scene.io_topology()
    assert mmd.startswith("flowchart LR")
    assert 'subgraph sg_host_UR["UR (robot controller: simple_arm)"]' in mmd
    assert "prog_pick" in mmd                                    # the program sits in its host
    assert 'sensor_part_at_pick -->|"part_at_pick → UR.DI2"| host_UR' in mmd
    assert 'host_UR -->|"conv → UR.DO0"| device_conv' in mmd
    dot = scene.io_topology("dot", layers=["io"])
    assert dot.startswith("digraph io_map {") and '"host:UR" -> "device:conv"' in dot
    assert "prog:pick" not in dot                                # programs belong to the functional layer
    for ext in ("mmd", "dot", "json"):
        path = tmp_path / f"topo.{ext}"
        scene.export_topology(path)
        assert path.read_text(encoding="utf-8") == scene.io_topology({"mmd": "mermaid", "dot": "dot", "json": "json"}[ext])
    import json
    data = json.loads(scene.io_topology("json"))
    assert {n["kind"] for n in data["nodes"]} >= {"host", "program", "sensor", "device", "field"}
    assert {e["kind"] for e in data["edges"]} == {"io"}          # a single-controller cell: no handshakes
    with pytest.raises(ValueError, match="unknown topology layer"):
        scene.io_topology(layers=["power"])


@pytest.mark.skipif(not HAS_CATALOG, reason="botrail catalog not in the HF cache")
def test_golden_weld_line_three_stages() -> None:
    """Demo 2: the same line, three placements — nothing declared, a PLC
    master with declared robot controllers, and the stations running their
    own programs. The I/O table changes shape each time; auto-assign then
    wires the last stage completely."""
    import weld_line_demo as wl

    scene, line, riders = wl.build_line()
    poses = wl.teach(scene, line, riders)
    for st in wl.STATIONS:
        wl.build_station_program(scene, st, poses, bodies=wl.BODIES)
    wl.build_transfer_program(scene, riders, gated=True)

    def real():
        return {(p.label, p.direction, p.host): p for p in scene.io_points() if p.status != "cosmetic"}

    # Stage 1 — nothing declared: every program is on <cell> (two arms per
    # station program, no arm for the transfer), so both arms of both
    # stations get start/done there and the inter-program signals are relays.
    stage1 = real()
    assert {k[2] for k in stage1} == {"<cell>"}
    assert stage1[("st1_done", "output", "<cell>")].status == "internal"
    assert stage1[("moving", "output", "<cell>")].status == "internal"
    assert ("st1_lh.start", "output", "<cell>") in stage1 and ("st2_rh.done", "input", "<cell>") in stage1
    assert stage1[("line.index", "output", "<cell>")].kind == "DO"
    assert stage1[("line", "input", "<cell>")].source == "device:done"
    assert scene.io_report().errors() == []

    # Stage 2 — a PLC runs everything, the stations are declared cabinets:
    # the arms' handshakes mirror onto ST1/ST2, the signals stay relays.
    scene.add_io_node("PLC1", kind="plc", programs=["transfer", "st1", "st2"],
                      channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.add_io_node("ST1", kind="robot_controller", robots=["st1_lh", "st1_rh"], channels=bt.io.ur_standard())
    scene.add_io_node("ST2", kind="robot_controller", robots=["st2_lh", "st2_rh"], channels=bt.io.ur_standard())
    stage2 = real()
    assert ("st1_lh.start", "output", "PLC1") in stage2 and ("st1_lh.start", "input", "ST1") in stage2
    assert ("st1_lh.done", "input", "PLC1") in stage2 and ("st1_lh.done", "output", "ST1") in stage2
    assert stage2[("st1_done", "output", "PLC1")].status == "internal"
    assert len(stage2) == 24

    # Stage 3 — the stations run their own programs: the arm handshakes
    # vanish, `st*_done` becomes ST → PLC wires and `moving` fans out
    # PLC → ST1, ST2.
    scene.add_io_node("PLC1", kind="plc", programs=["transfer"],
                      channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.add_io_node("ST1", kind="robot_controller", robots=["st1_lh", "st1_rh"], programs=["st1"],
                      channels=bt.io.ur_standard())
    scene.add_io_node("ST2", kind="robot_controller", robots=["st2_lh", "st2_rh"], programs=["st2"],
                      channels=bt.io.ur_standard())
    stage3 = real()
    assert not any(k[0].endswith(".start") or k[0].endswith(".done") for k in stage3)
    assert stage3[("st1_done", "output", "ST1")].source == "signal:handshake"
    assert stage3[("st1_done", "input", "PLC1")].source == "signal:handshake"
    assert {k[2] for k in stage3 if k[0] == "moving" and k[1] == "input"} == {"ST1", "ST2"}
    assert stage3[("st1_arc", "output", "ST1")].source == "signal:write-only"
    assert len(stage3) == 12
    # Auto-assign wires the whole line; addresses count up on the PLC.
    report = scene.auto_assign_io()
    assert report.findings == [], str(report)
    rows = {l.split(",")[0] + "/" + l.split(",")[2]: l for l in scene.io_list("csv").splitlines() if not l.startswith("#")}
    assert rows["body_at_head/input"].split(",")[6:9] == ["PLC1", "DI0", "%IX0.0"]
    assert rows["line/input"].split(",")[6:9] == ["PLC1", "DI1", "%IX0.1"]
    assert rows["line/output"].split(",")[6:9] == ["PLC1", "DO0", "%QX0.0"]
    assert scene.io_list("csv").rstrip().splitlines()[-3:] == ["# PLC1: DI 4, DO 2", "# ST1: DI 1, DO 2", "# ST2: DI 1, DO 2"]
    # The wiring diagram shows the fan-out and the two station wires.
    mmd = scene.io_topology(layers=["wiring"])
    assert 'host_PLC1 ==>|"moving"| host_ST1' in mmd and 'host_PLC1 ==>|"moving"| host_ST2' in mmd
    assert 'host_ST1 ==>|"st1_done"| host_PLC1' in mmd
    # A deliberate mismatch is named: a 12 V NPN beam on the PLC's PNP DI.
    scene.remove_io_node("PLC1")
    scene.add_io_node("PLC1", kind="plc", programs=["transfer"],
                      channels=bt.io.di16(base="%IX0.0", voltage=24, logic="pnp") + bt.io.do16(base="%QX0.0"))
    scene.bind_input("body_at_head", "PLC1", "DI0", voltage=12, logic="npn")
    codes = {f.code for f in scene.io_report().warnings()}
    assert {"polarity", "voltage"} <= codes
    # An open beam at the head of the line: the transfer never sees a body
    # and both stations wait for the first index — the diagnosis names
    # every stalled program and the wire.
    scene.add_scenario("beam_open", faults=[bt.io.open("body_at_head")])
    with pytest.raises(ValueError, match="programs waiting at: st1/b1_p1_start, st2/b1_p1_start, transfer/p1_load — forced: body_at_head=false"):
        scene.simulate_sequences(["st1", "st2", "transfer"], scenario="beam_open", max_duration=400.0)


# ------------------------------------------ faults and the handshake spec (I3)


def test_faults_pin_inputs_and_the_timeout_names_them() -> None:
    """Demo 1 (5): a stuck beam is a scenario, and the sweep collects the
    stall with the forced point in the diagnosis, next to `ng_part`."""
    demo, scene = wired_pick_cell()
    scene.add_scenario("beam_stuck", faults=[bt.io.stuck("part_at_pick", False)])
    scene.add_scenario("gauge_open", faults=[bt.io.open("spec_ok")])
    scene.add_scenario("clean", faults=[])
    runs = scene.simulate_scenarios(["pick"], max_duration=30.0)
    assert runs.names == ["baseline", "ng_part", "gauge_open", "clean"]
    assert runs.errors == {
        "beam_stuck": "timed out after 30s waiting in step 1 (`await part`) — forced: part_at_pick=false"
    }
    # An open gauge wire reads low on a normally-open wiring: the reject
    # arm, the same path `ng_part` takes by setting the signal.
    assert runs["gauge_open"].branches == runs["ng_part"].branches == [("pick", "judge", 1)]
    # A scenario without faults bakes bit-identically to baseline.
    base, clean = runs["baseline"], runs["clean"]
    assert (clean.duration, clean.signals, clean.step_spans) == (base.duration, base.signals, base.step_spans)
    # A pinned input is a level, not an edge: wire the beam NC and the open
    # wire reads *true* — and the edge-triggered `await part` still stalls,
    # with the value it saw in the diagnosis.
    scene.bind_input("part_at_pick", "UR", "DI2", invert=True, contact="nc")
    scene.add_scenario("beam_open", faults=[bt.io.open("part_at_pick")])
    with pytest.raises(ValueError, match="`await part`\\) — forced: part_at_pick=true"):
        scene.simulate_sequences(["pick"], scenario="beam_open", max_duration=30.0)
    # Only inputs can be forced, and only input wires opened.
    scene.add_scenario("bad", faults=[bt.io.stuck("conv", True)])
    with pytest.raises(ValueError, match="a device's running lane is an output"):
        scene.simulate_sequences(["pick"], scenario="bad")
    scene.add_scenario("bad", faults=[bt.io.open("vacuum")])
    with pytest.raises(ValueError, match="no input wire to open"):
        scene.simulate_sequences(["pick"], scenario="bad")
    scene.add_scenario("bad", faults=[bt.io.stuck("part_at_pick", True), bt.io.open("part_at_pick")])
    with pytest.raises(ValueError, match="forced twice"):
        scene.simulate_sequences(["pick"], scenario="bad")
    with pytest.raises(ValueError, match="unknown fault kind"):
        scene.add_scenario("bad", faults=[{"target": "x", "kind": "short"}])
    # The live scene is untouched by all of it.
    assert scene.simulate_sequence("pick").duration == base.duration


def test_stuck_signals_drop_writes_and_open_wires_follow_their_polarity() -> None:
    scene = two_arm_cell()
    author_two_arm(scene)
    programs = ["pick", "belt"]
    # `check` waits on `estop_ok`, a contact nobody writes — the E-stop
    # healthy input. Nothing sets it, so the baseline stalls there.
    with pytest.raises(ValueError, match="programs waiting at: pick/check"):
        scene.simulate_sequences(programs, max_duration=20.0)
    scene.add_scenario("healthy", faults=[bt.io.stuck("estop_ok", True)])
    tl = scene.simulate_sequences(programs, scenario="healthy", max_duration=20.0)
    assert tl.signal("estop_ok").edges == [(0.0, True)]        # a level from t = 0, not an edge
    # A stuck-low `carrying` drops pick's own set: the belt program never
    # sees it and waits forever, and the diagnosis lists both forces in
    # scenario order.
    scene.add_scenario("relay_stuck", faults=[bt.io.stuck("estop_ok", True), bt.io.stuck("carrying", False)])
    with pytest.raises(ValueError, match="programs waiting at: belt/run — forced: estop_ok=true, carrying=false"):
        scene.simulate_sequences(programs, scenario="relay_stuck", max_duration=20.0)
    # An open E-stop wire: low on the usual wiring (the cell stops), high on
    # an inverted one — the polarity decides, and the run shows which.
    scene.add_scenario("estop_open", faults=[bt.io.open("estop_ok")])
    with pytest.raises(ValueError, match="pick/check — forced: estop_ok=false"):
        scene.simulate_sequences(programs, scenario="estop_open", max_duration=20.0)
    scene.add_io_node("PLC1", kind="plc", programs=programs, channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.bind_input("estop_ok", "PLC1", "DI0", invert=True, contact="nc")
    tl = scene.simulate_sequences(programs, scenario="estop_open", max_duration=20.0)
    assert tl.signal("estop_ok").edges == [(0.0, True)]
    assert tl.scenario == "estop_open"


def test_faults_survive_the_project_and_the_generated_python(tmp_path: Path) -> None:
    demo, scene = wired_pick_cell()
    scene.add_scenario("beam_stuck", faults=[bt.io.stuck("part_at_pick", False)])
    scene.add_scenario("gauge_open", signals={"spec_ok": True}, faults=[bt.io.open("spec_ok")])
    path = tmp_path / "faults.botrail"
    scene.save_project(path)
    again = bt.Scene.load_project(path)
    assert again.scenario_names == ["ng_part", "beam_stuck", "gauge_open"]
    runs = again.simulate_scenarios(["pick"], max_duration=30.0)
    assert list(runs.errors) == ["beam_stuck"]
    assert runs["gauge_open"].branches == [("pick", "judge", 1)]     # the fault wins over the initial value
    code = scene.generate_python()
    assert 'scene.add_scenario("beam_stuck", faults=[bt.io.stuck("part_at_pick", False)])' in code
    assert 'scene.add_scenario("gauge_open", signals={"spec_ok": True}, faults=[bt.io.open("spec_ok")])' in code


def test_handshake_spec_lists_the_lines_between_controllers(tmp_path: Path) -> None:
    scene = two_arm_cell()
    author_two_arm(scene)
    scene.add_io_node("PLC1", kind="plc", programs=["belt"], channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.add_scenario("healthy", signals={"estop_ok": True})
    tl = scene.simulate_sequences(["pick", "belt"], scenario="healthy", max_duration=20.0)
    md = tl.handshake_spec()
    assert md.startswith("# Handshake spec — pick + belt (healthy)\n")
    # Two signals cross (belt_ok PLC1 → <cell>, carrying back), both arms
    # get start / done lines from <cell>, far a program word (two motions),
    # and the belt its run line.
    assert "2 handshake signal(s), 5 robot line(s), 1 device line(s)." in md   # `always_on` runs unbidden: no line
    assert "## `belt_ok` — PLC1 → <cell>\n" in md and "## `carrying` — <cell> → PLC1\n" in md
    assert "## `far.start` — <cell> → <far>\n" in md and "## `far.done` — <far> → <cell>\n" in md
    assert "| written by | `belt/run` |\n| waited by | `pick/carry` |" in md
    assert "| issued by | `pick/go`, `pick/carry` |" in md
    # The waveforms are the bake's: carrying rises when `carry` is entered
    # and stays; the far arm's busy spans are its merged moves.
    carry = tl.step_span("pick/carry")
    assert f"| high | {carry.start:.3f}–{tl.duration:.3f} s |" in md
    busy = tl.robot_busy("far")
    assert len(busy) == 2 and busy == [(m[1], m[2]) for m in tl.moves("far")]
    assert sum(b - a for a, b in busy) == pytest.approx(tl.busy_seconds("far"), abs=1e-12)
    assert f"| busy | {busy[0][0]:.3f}–{busy[0][1]:.3f}, {busy[1][0]:.3f}–{busy[1][1]:.3f} s |" in md
    # The done contact blinks between the two chained moves — a sub-tick
    # idle the sheet keeps at 3 decimals rather than hiding.
    assert f"| done (idle) | {busy[0][1]:.3f}–{busy[1][0]:.3f}, {busy[1][1]:.3f}–{tl.duration:.3f} s |" in md
    # Wiring after the bake: `io=` labels the ends with the channels.
    scene.auto_assign_io()
    wired = tl.handshake_spec(io=scene.io_map())
    assert "| `belt_ok` | signal | PLC1 · DO2 [%QX0.2] | <cell> |" in wired     # always_on DO0, belt DO1
    assert "| `belt` | device:run | PLC1 · DO1 [%QX0.1] | belt |" in wired
    assert "| `carrying` | signal | <cell> | PLC1 · DI1 [%IX0.1] |" in wired
    out = tmp_path / "handshake.md"
    tl.export_handshake_spec(out, io=scene.io_map())
    assert out.read_text(encoding="utf-8") == wired
    # A single-controller bake has nothing crossing.
    demo, cell = wired_pick_cell()
    spec = cell.simulate_sequence("pick").handshake_spec()
    assert "0 handshake signal(s), 0 robot line(s), 1 device line(s)." in spec  # the conveyor run line


# ------------------------------------------------ I6: node_down, dialects, diff


def test_node_down_opens_every_input_wired_on_the_node() -> None:
    scene = two_arm_cell()
    author_two_arm(scene)
    programs = ["pick", "belt"]
    scene.add_io_node("PLC1", kind="plc", programs=programs, channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.add_io_node("RIO1", kind="remote_io", uplink=("PLC1", "PROFINET"), channels=bt.io.di8(base="%IX1.0"))
    scene.bind_input("estop_ok", "RIO1", "DI0", invert=True, contact="nc")   # healthy = closed, inverted
    scene.bind_input("beam", "RIO1", "DI1")
    scene.auto_assign_io()
    # The station drops off: its inputs open with their own polarity —
    # the NC E-stop reads *healthy* (true), the beam low. `check` waits on
    # estop_ok, so the run completes; the forced list names both.
    scene.add_scenario("rio_down", faults=[bt.io.node_down("RIO1")])
    tl = scene.simulate_sequences(programs, scenario="rio_down", max_duration=20.0)
    assert tl.signal("estop_ok").edges == [(0.0, True)] and tl.signal("beam").edges == [(0.0, False)]
    # The controller drops: the station's inputs hang off it, so the same
    # two open (its own points here are outputs and relays — nothing else
    # to force). A node with nothing to open, and a non-node, are refused.
    scene.add_scenario("plc_down", faults=[bt.io.node_down("PLC1")])
    tl2 = scene.simulate_sequences(programs, scenario="plc_down", max_duration=20.0)
    assert tl2.signal("estop_ok").edges == [(0.0, True)]
    scene.add_io_node("SPARE", kind="remote_io", uplink=("PLC1", "PROFINET"), channels=bt.io.di8())
    scene.add_scenario("spare_down", faults=[bt.io.node_down("SPARE")])
    with pytest.raises(ValueError, match="opens nothing"):
        scene.simulate_sequences(programs, scenario="spare_down", max_duration=20.0)
    scene.add_scenario("ghost_down", faults=[bt.io.node_down("PLC9")])
    with pytest.raises(ValueError, match="not an I/O node"):
        scene.simulate_sequences(programs, scenario="ghost_down", max_duration=20.0)
    # Twice is refused, and the diagnosis of a stall lists the opened lanes.
    scene.add_scenario("twice", faults=[bt.io.open("beam"), bt.io.node_down("RIO1")])
    with pytest.raises(ValueError, match="forced twice"):
        scene.simulate_sequences(programs, scenario="twice", max_duration=20.0)
    scene.unbind_input("estop_ok")
    scene.bind_input("estop_ok", "RIO1", "DI0")                # NO wiring: open = not healthy
    with pytest.raises(ValueError, match="pick/check — forced: beam=false, estop_ok=false"):   # binding order
        scene.simulate_sequences(programs, scenario="rio_down", max_duration=20.0)
    code = scene.generate_python()
    assert 'scene.add_scenario("rio_down", faults=[bt.io.node_down("RIO1")])' in code


def test_address_dialects() -> None:
    assert [c["address"] for c in bt.io.di16(base="%IX0.0")][6:10] == ["%IX0.6", "%IX0.7", "%IX1.0", "%IX1.1"]
    x = [c["address"] for c in bt.io.melsec("di", 20, "X10")]
    assert x[:2] == ["X10", "X11"] and x[14:18] == ["X1E", "X1F", "X20", "X21"]     # hex (Q / iQ-R)
    y = [c["address"] for c in bt.io.melsec("do", 10, "Y000", octal=True)]
    assert y[6:10] == ["Y006", "Y007", "Y010", "Y011"]                               # octal (FX), width kept
    assert [c["address"] for c in bt.io.siemens("do", 10)][6:10] == ["Q0.6", "Q0.7", "Q1.0", "Q1.1"]
    lx = [c["address"] for c in bt.io.logix("di", 20)]
    assert lx[7:9] == ["Local:1:I.Data.7", "Local:1:I.Data.8"] and lx[16] == "Local:1:I.Data.16"   # flat
    assert bt.io.address("K7", 3) == "K10" and bt.io.address("X0F", 1, radix=16) == "X10"
    assert bt.io.melsec("di", 4)[0]["address"] == "X0" and bt.io.melsec("do", 4)[0]["address"] == "Y0"
    # A node built from a dialect binds and lists like any other.
    scene = two_arm_cell()
    author_two_arm(scene)
    scene.add_io_node("Q03", kind="plc", programs=["pick", "belt"], channels=bt.io.melsec("di", 16, "X20") + bt.io.melsec("do", 16, "Y20"))
    scene.auto_assign_io()
    rows = {l.split(",")[0]: l.split(",")[8] for l in scene.io_list("csv").splitlines()[1:] if not l.startswith("#")}
    assert rows["always_on"] == "Y20" and rows["belt"] == "Y21" and rows["beam"] == "X20"   # table order


def test_io_sheet_diff(tmp_path: Path) -> None:
    demo, scene = wired_pick_cell()
    sheet = tmp_path / "pick_cell_io.csv"
    scene.export_io_list(sheet)
    d = bt.io.diff(scene, sheet)
    assert d.ok and not d and str(d) == "I/O sheet matches the cell"
    # The electrical designer's copy drifted: the beam moved to DI4, the
    # vacuum row is gone, a spare appeared.
    rows = bt.io.read_io_list(sheet)
    for r in rows:
        if r["name"] == "part_at_pick":
            r["channel"] = "DI4"
    rows = [r for r in rows if r["name"] != "vacuum"] + [
        {"name": "spare_1", "aspect": "", "direction": "input", "node": "UR", "channel": "DI5"}
    ]
    d = bt.io.diff(scene, rows)
    assert d and not d.ok
    assert [r["name"] for r in d.added] == ["vacuum"]
    assert [r["name"] for r in d.removed] == ["spare_1"]
    assert d.changed == [(("part_at_pick", "", "input", "UR"), {"channel": ("DI2", "DI4")})]
    text = str(d)
    assert "+ vacuum.output" in text and "- spare_1.input" in text and "channel: 'DI2' → 'DI4'" in text
    # A partial sheet compares only the columns it has (no host → keyed
    # without it), and the JSON export round-trips as a sheet too.
    d2 = bt.io.diff(scene, [{"name": "conv", "direction": "output", "channel": "DO0"},
                            {"name": "part_at_pick", "direction": "input", "channel": "DI2"}])
    assert d2.columns == ("channel",) and {r["name"] for r in d2.added} == {"spec_ok", "vacuum"} and not d2.changed
    scene.export_io_list(tmp_path / "pick.json")
    assert bt.io.diff(scene, tmp_path / "pick.json").ok
    with pytest.raises(ValueError, match="needs a `name` column"):
        (tmp_path / "bad.csv").write_text("a,b\n1,2\n", encoding="utf-8")
        bt.io.read_io_list(tmp_path / "bad.csv")

