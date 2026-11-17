"""`bt.nc` — NC part programs on a machine tool's axes (N1 of the NC
machine plan).

A machining centre built with its axes is a mechanism whose joints are
the machine coordinates, declared as an NC machine. What these tests pin:
the mechanism stands where the figures say and is not an arm on the bill;
a work offset is derived from where the part stands, whichever way the
machine faces; a part program is read against the tables, timed by the
axes' rates, and run in a bake with whatever rides the table; and what the
control cannot read is refused by line — at the dry run, when a tending
template is authored, and in the bake.

And what the program cuts (N2): a tool is a body whose length the table's
entry is taken from; the dry run and the baked cycle carve the stock that
rides the table and book it by source line; a setup stood wrong — a
table entry that is not the tool, a work zero off the part — comes out as
a line number and a volume; and the setup sheet says all of it.

And the tool changer (N6): where the machine's picture draws one, it is
two more joints of the mechanism that the control works at each tool
change, and a tool that is not in the spindle rides its pocket.
"""

import math
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

# A Haas VF-2 by its drawing: the enclosure, the lifting window, and the
# axes — 762 x 406 x 508 mm to the negative side of home, the table's
# centre 381 mm left and 211.5 mm ahead of the spindle there.
AXES = bt.parts.MachineAxes(
    travel=(0.762, 0.406, 0.508), home_table=(-0.381, -0.2115), rapid=25.4 / 60, max_feed=16.5 / 60,
    acceleration=4.9, tool_change_s=4.2, spindle_max_rpm=8100,
)
VF2 = {
    "size": (2.316, 1.984, 2.724), "enclosure_height": 1.818, "chamber": 1.139,
    "aperture": (0.676, 0.749, 0.819), "door": "air", "door_slide": "up", "door_travel": 0.595,
    "door_offset": 0.050, "front_door": (0.939, 0.954, 0.816), "table": (0.914, 0.368, 0.853),
    "head_clearance": 0.610, "head_offset": 0.101,
}
EXCHANGE = (0.381, 0.050)
PART = (0.04, 0.04, 0.06)
# Face one pass and drill two holes, then park for the exchange.
PROGRAM = """%
O00010 (FACE AND DRILL)
N10 G90 G94 G17 G21
N20 G53 G0 Z0.
N30 T1 M6
N40 S6000 M3
N50 G54 G0 X-30. Y0.
N60 G43 Z10. H1
N70 G1 Z-0.5 F300.
N80 X30. F900.
N90 G0 Z10.
N100 M5
N110 T2 M6
N120 S4000 M3
N130 G54 G0 X-10. Y0.
N140 G43 Z10. H2
N150 G98 G81 X-10. Y0. Z-6. R2. F150.
N160 X10.
N170 G80
N180 M5
N190 G53 G0 Z0.
N200 G53 G0 X-762. Y-160.5
N210 M30
%
"""


def scene_() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"), base_position=(4.0, 0.0, 0.0))


def machine(scene, name="vf2", **overrides):
    return bt.parts.machine_tool(scene, name, **{**VF2, "exchange": EXCHANGE, "axes": AXES, **overrides})


def cell(**options):
    """The machine with a part standing on its table (in place of a vise),
    the work zero on the part's top face, two tools and the program."""
    scene = scene_()
    vf2 = machine(scene, **options)
    (tx, ty, tz), _ = scene.frame("vf2/table")
    scene.add_box("part", size=PART, position=(tx + 0.33, ty, tz + PART[2] / 2))
    bt.nc.set_work_offset(scene, vf2, "G54", at=(tx + 0.33, ty, tz + PART[2]))
    bt.nc.set_tool(scene, vf2, 1, length=0.110, diameter=0.010)
    bt.nc.set_tool(scene, vf2, 2, length=0.135, diameter=0.005)
    bt.nc.program(scene, "op10", PROGRAM)
    return scene, vf2


def rows(scene) -> dict:
    return {row["names"][0]: row for row in scene.bom().rows}


# The two tools as they stand in the spindle: a collet chuck 75 mm from
# gauge line to nose, the cutter so far out of it — 110 and 135 mm long.
HOLDER = bt.nc.Holder(diameter=0.040, length=0.075)
MILL = bt.nc.end_mill(0.010, flutes=0.022, stickout=0.035, holder=HOLDER)
DRILL = bt.nc.drill(0.005, flutes=0.045, stickout=0.060, holder=HOLDER)
# What the program takes off the 40 mm block: a 10 mm pass half a
# millimetre deep along X, and two 5 mm holes from that floor to 6 mm
# under the top.
PASS = 0.040 * 0.010 * 0.0005
HOLE = math.pi * 0.0025 ** 2 * 0.0055


def cutting_cell(shape=None, **entered):
    """`cell` with its tools as bodies; `entered` puts another length in
    a tool's table entry (`t2=0.115`), `shape` draws the machine as a
    model package."""
    scene, vf2 = cell(**({} if shape is None else {"shape": shape}))
    bt.nc.set_tool(scene, vf2, 1, MILL, length=entered.get("t1"))
    bt.nc.set_tool(scene, vf2, 2, DRILL, length=entered.get("t2"))
    return scene, vf2


def test_the_axes_are_a_mechanism_whose_joints_are_the_machine_coordinates() -> None:
    scene = scene_()
    vf2 = machine(scene)
    assert (vf2.axes, vf2.table_link, vf2.head_link) == ("vf2/axes", "table", "head")
    # Built standing at the exchange position: the far end of X, 160.5 mm
    # into Y, the head at the top.
    assert vf2.park == pytest.approx((-0.762, -0.1605, 0.0))
    q = dict(zip(scene.robot_of(vf2.axes).joint_names, scene.joint_positions_of(vf2.axes)))
    assert (q["x"], q["y"], q["z"]) == pytest.approx(vf2.park)
    # The table's frame is the centre of its top, the head's the spindle
    # nose: 610 mm over the table at home, 101 mm behind the chamber's centre.
    table, _ = scene.link_pose("table", robot=vf2.axes)
    nose, _ = scene.link_pose("head", robot=vf2.axes)
    assert table == pytest.approx(scene.frame("vf2/table")[0])
    assert nose[2] - table[2] == pytest.approx(0.610) and nose[0] == pytest.approx(0.0)
    # The static boxes they replace are gone, and nothing touches at rest.
    assert not {"vf2/table", "vf2/saddle", "vf2/head"} & set(scene.obstacle_names)
    assert scene.check_collisions() == []
    # The control: travels to the negative side of home, the rapid rate
    # as each joint's velocity limit, the rates the model does not carry.
    control = scene.nc_machine(vf2.axes)
    assert control["axes"] == ("x", "y", "z")
    assert [v for axis in control["travel"] for v in axis] == pytest.approx([-0.762, 0.0, -0.406, 0.0, -0.508, 0.0])
    assert control["rapid"] == pytest.approx((25.4 / 60,) * 3) and control["max_feed"] == pytest.approx(0.275)
    assert (control["tool_change_s"], control["spindle_max_rpm"]) == (4.2, 8100)
    assert control["work_offsets"] == {} and control["tools"] == {}
    assert scene.nc_machine_names == ["vf2/axes"]
    # A machine's axes are not an arm: one line for the machine, none for
    # the mechanism, and no controller box of its own.
    by = rows(scene)
    assert "vf2" in by and not any(name.startswith("vf2/axes") for name in by)
    # Moving the axes moves the links the way the machine does: X- carries
    # the table right, Y- carries it back, Z- brings the head down.
    names = scene.robot_of(vf2.axes).joint_names
    home = [0.0, 0.0, 0.0]
    there = [{"x": -0.100, "y": -0.050, "z": -0.200}[n] for n in names]
    t0, _ = scene.link_pose_at("table", home, robot=vf2.axes)
    t1, _ = scene.link_pose_at("table", there, robot=vf2.axes)
    n0, _ = scene.link_pose_at("head", home, robot=vf2.axes)
    n1, _ = scene.link_pose_at("head", there, robot=vf2.axes)
    assert tuple(b - a for a, b in zip(t0, t1)) == pytest.approx((0.100, 0.050, 0.0))
    assert tuple(b - a for a, b in zip(n0, n1)) == pytest.approx((0.0, 0.0, -0.200))
    # At home the spindle is 76 mm in from the table's right end — the
    # drawing's 75 — and 211.5 mm behind its centre.
    assert n0[0] - t0[0] == pytest.approx(0.381) and n0[1] - t0[1] == pytest.approx(0.2115)


def test_the_axes_are_refused_where_the_figures_do_not_fit() -> None:
    scene = scene_()
    # The exchange offset is a table position, so it has to be one the
    # axes reach: 400 mm right of centre is 19 mm past the end of X.
    with pytest.raises(ValueError, match=r"X-781\.0 in machine coordinates, outside the X travel \(-762 to 0 mm\)"):
        machine(scene, exchange=(0.400, 0.050))
    with pytest.raises(ValueError, match="takes the spindle nose through the table"):
        machine(scene, axes=bt.parts.MachineAxes(travel=(0.762, 0.406, 0.700), home_table=(-0.381, -0.2115)))
    with pytest.raises(ValueError, match="axes=True takes a catalog pack's axes"):
        bt.parts.machine_tool(scene, "vmc", axes=True)
    assert scene.obstacle_names == [] and scene.nc_machine_names == []
    # Without axes it is the envelope it was: the table a box, standing.
    plain = bt.parts.machine_tool(scene, "vmc", **VF2, exchange=EXCHANGE)
    assert plain.axes is None and plain.park is None and "vmc/table" in scene.obstacle_names
    # The spindle's offset moves the head either way.
    assert scene.obstacle_pose("vmc/head")[0][1] == pytest.approx(scene.frame("vmc/table")[0][1] - 0.050 + 0.101)
    # Under a roof the head stands up to it at home — a declared contact.
    roofed = scene_()
    vmc = bt.parts.machine_tool(roofed, "vmc", exchange=(0.25, 0.0),
                                axes=bt.parts.MachineAxes(travel=(0.5, 0.4, 0.33), home_table=(-0.25, -0.2)))
    assert vmc.park == pytest.approx((-0.5, -0.2, 0.0)) and roofed.check_collisions() == []


def test_a_work_offset_is_derived_from_where_the_part_stands() -> None:
    scene, vf2 = cell()
    # The part is 330 mm right of the table's centre: 51 mm inside the X
    # home end, on the table's centreline, its top 60 mm over the table.
    offset = scene.nc_machine(vf2.axes)["work_offsets"]["G54"]
    assert offset == pytest.approx((-0.051, -0.2115, 0.060 - 0.610))
    (tx, ty, tz), _ = scene.frame("vf2/table")
    top = (tx + 0.33, ty, tz + PART[2])
    assert bt.nc.machine_coordinates(scene, vf2, top) == pytest.approx(offset)
    # It is the same figure wherever the axes happen to stand — the point
    # rides the table — and whichever way the machine faces in the cell.
    names = scene.robot_of(vf2.axes).joint_names
    moved = {"x": -0.300, "y": -0.020, "z": -0.100}
    scene.set_joint_positions([moved[n] for n in names], robot=vf2.axes)
    # (X and Y went positive by 462 and 140.5 mm, so the table went left and forward.)
    carried = (top[0] - (0.762 - 0.300), top[1] - (0.1605 - 0.020), top[2])
    assert bt.nc.machine_coordinates(scene, vf2, carried) == pytest.approx(offset)
    turned = scene_()
    yaw, at = 2.0, (1.5, -0.7)
    other = machine(turned, yaw=yaw, position=at)
    (ox, oy, oz), _ = turned.frame("vf2/table")
    c, s = math.cos(yaw), math.sin(yaw)
    point = (ox + 0.33 * c, oy + 0.33 * s, oz + PART[2])
    assert bt.nc.machine_coordinates(turned, other, point) == pytest.approx(offset)
    # Or the figures off a real machine's offset page, entered as they are.
    assert bt.nc.set_work_offset(scene, vf2, "G154 P2", xyz=(-0.4, -0.2, -0.3)) == (-0.4, -0.2, -0.3)
    assert set(scene.nc_machine(vf2.axes)["work_offsets"]) == {"G54", "G154 P2"}
    with pytest.raises(ValueError, match="give the work zero as `at`"):
        bt.nc.set_work_offset(scene, vf2, "G55")
    with pytest.raises(ValueError, match="`G60` is not a work offset"):
        bt.nc.set_work_offset(scene, vf2, "G60", xyz=(0.0, 0.0, 0.0))
    with pytest.raises(ValueError, match="was built without its axes"):
        bt.nc.set_tool(scene, bt.parts.machine_tool(scene_(), "plain"), 1, length=0.1)


def test_a_program_is_read_against_the_tables_and_timed_by_the_axes() -> None:
    scene, vf2 = cell()
    run = bt.nc.check(scene, vf2, "op10")
    assert (run.machine, run.program, run.tool_changes, run.tools()) == ("vf2/axes", "op10", 2, [1, 2])
    assert run.duration == pytest.approx(run.cutting_time + run.rapid_time + run.pause_time)
    assert run.pause_time == pytest.approx(2 * 4.2) and run.warnings == ()
    # It ends parked, and never leaves the travels.
    assert run.end == pytest.approx(vf2.park)
    assert run.lower[0] >= -0.762 - 1e-9 and run.upper[2] <= 1e-9
    # The facing pass: 60 mm at 900 mm/min is 4 s and a little for the
    # ramps; line 80 is what runs then, with the tip 0.5 mm into the part.
    start, end = next((a, b) for a, b, line in run.spans if line == 10)
    assert end - start == pytest.approx(4.0, abs=0.05) and run.line_at((start + end) / 2) == 10
    assert run.cutting_distance == pytest.approx(0.0105 + 0.060 + 2 * 0.008, abs=1e-6)
    assert "op10 on vf2/axes" in run.summary() and "2 tool changes" in run.summary()
    spindle = [(line, value) for _, line, what, value in run.events if what == "spindle"]
    assert (6, 6000.0) in spindle and (14, 4000.0) in spindle and spindle[-1][1] == 0.0
    # From home the first run to the part is the best part of the X travel
    # shorter than from the exchange position — the same program, a
    # different cycle.
    near = bt.nc.check(scene, vf2, "op10", start=(0.0, 0.0, 0.0))
    assert near.rapid_distance < run.rapid_distance - 0.4
    assert near.duration < run.duration and near.end == pytest.approx(vf2.park)
    assert scene.joint_positions_of(vf2.axes) == pytest.approx(
        [dict(zip("xyz", vf2.park))[n] for n in scene.robot_of(vf2.axes).joint_names])   # nothing moved

    # In a bake: the step takes as long as the program, and what is
    # attached to the table rides it.
    scene.attach("part", link=vf2.table_link, robot=vf2.axes)
    sq = scene.sequence("cut")
    sq.step("machining", actions=[bt.seq.run_program(vf2.axes, "op10")])
    tl = scene.simulate_sequence("cut", max_duration=300.0)
    assert tl.duration == pytest.approx(run.duration, abs=0.02)
    mid = (start + end) / 2
    q = dict(zip(scene.robot_of(vf2.axes).joint_names, tl.sample(mid, vf2.axes)))
    # Halfway along the pass the spindle is over the part's centre, the
    # tip (110 mm of tool) half a millimetre into its top.
    assert q["x"] == pytest.approx(-0.051, abs=2e-3) and q["z"] == pytest.approx(-0.550 - 0.0005 + 0.110)
    (px, py, pz), _ = tl.object_pose("part", mid)
    nose, _ = scene.link_pose_at("head", tl.sample(mid, vf2.axes), robot=vf2.axes)
    assert (px, py) == pytest.approx((nose[0], nose[1]), abs=2e-3)
    assert nose[2] - (pz + PART[2] / 2) == pytest.approx(0.110 - 0.0005, abs=1e-6)
    assert tl.object_pose("part", tl.duration)[0] == pytest.approx(tl.object_pose("part", 0.0)[0])


def test_what_the_control_cannot_read_is_refused_by_line() -> None:
    scene, vf2 = cell()
    for text, message in (
        ("G54 G90 G0 X0. Y0.\nT3 M6\nG43 Z10. H3\n", r"line 3: tool length H3 is not in the tool table \(it holds: H1, H2\)"),
        ("G54 G90 G0 X0. Y0.\nG0 X60.\n", r"line 2: X 9\.000 mm is past the X travel"),
        ("G54 G90 G0 X0. Y0.\nG41 D1 X5.\n", "line 2: G41: cutter radius compensation is not supported"),
        ("G55 G90 G0 X0. Y0.\n", r"line 1: work offset G55 is not set \(the machine holds: G54\)"),
        ("G54 G90 G0 X0. Y0.\nS9000 M3\n", "line 2: S9000 is over the spindle's 8100 rev/min"),
    ):
        bt.nc.program(scene, "bad", text)
        with pytest.raises(ValueError, match=message):
            bt.nc.check(scene, vf2, "bad")
        # ...and in a bake, with the step it happened in.
        sq = scene.sequence("cut")
        sq.step("machining", actions=[bt.seq.run_program(vf2.axes, "bad")])
        with pytest.raises(ValueError, match="machining.*NC program `bad` on `vf2/axes`: " + message):
            scene.simulate_sequence("cut")
        scene.remove_sequence("cut")
    with pytest.raises(ValueError, match="unknown NC program `op99`"):
        bt.nc.check(scene, vf2, "op99")
    with pytest.raises(ValueError, match="`simple_arm` is not an NC machine"):
        bt.nc.check(scene, "simple_arm", "op10")
    assert scene.nc_program_names == ["op10", "bad"] and scene.nc_program("op10") == PROGRAM
    scene.remove_nc_program("bad")
    assert scene.nc_program_names == ["op10"]


def test_a_tending_template_runs_the_program_in_place_of_its_timer() -> None:
    scene, vf2 = cell()
    scene.add_box("blank", size=PART, position=(3.0, 1.0, 0.5))
    run = bt.nc.check(scene, vf2, "op10")
    hs = bt.tending.haas_autodoor(scene, vf2, part_program="op10", parts=("part", "blank"), clamp_s=0.2)
    # The cell's side, answering without moving.
    sq = scene.sequence("tend")
    sq.step("wait_done", transition=bt.seq.signal(hs.signal("part_done")))
    sq.step("safe", actions=[bt.seq.set_signal(hs.signal("cell_safe"))], transition=bt.seq.signal(hs.signal("door_open")))
    sq.step("ask", actions=[bt.seq.set_signal(hs.signal("clamp_req"))], transition=bt.seq.signal(hs.signal("clamp")))
    sq.step("start", actions=[bt.seq.set_signal(hs.signal("clamp_req"), False), bt.seq.set_signal(hs.signal("start_req"))],
            transition=bt.seq.signal(hs.signal("door_closed")))
    tl = scene.simulate_sequences(["tend", "vf2"], max_duration=300.0)
    # Machining is as long as the program; the part done comes after it.
    span = tl.step_span("vf2/machining")
    assert span.end - span.start == pytest.approx(run.duration, abs=0.02)
    assert tl.signal("vf2/running").falling_edges()[0] == pytest.approx(run.duration, abs=0.02)
    assert tl.signal("vf2/part_done").rising_edges()[0] > run.duration
    # The clamped part rides the table through the cycle and is back at
    # the exchange position when the window opens; the loaded one is
    # taken as the window closes.
    start, end = next((a, b) for a, b, line in run.spans if line == 10)
    there, _ = tl.object_pose("part", (start + end) / 2)
    back, _ = tl.object_pose("part", tl.signal("vf2/side_door/open").rising_edges()[0])
    home, _ = tl.object_pose("part", 0.0)
    assert home[0] - there[0] == pytest.approx(0.762 - 0.051, abs=2e-3) and back == pytest.approx(home)
    rows_ = {(r["program"], r["step"], r["output"]) for r in scene.interlocks(["vf2"]).rows}
    assert any(step == "machining" and "op10" in output for _, step, output in rows_)
    # The axes are driven by the machine's own control, where its program
    # runs: the I/O list grows no start / done wire for them.
    assert not any(p.name == "vf2/axes" for p in scene.io_points())

    # Authoring reads the program once: what the control cannot read is
    # refused there by line, and so is a program that does not come back
    # to where the cell exchanges parts.
    other, vmc = cell()
    bt.nc.program(other, "stray", PROGRAM.replace("N200 G53 G0 X-762. Y-160.5", "N200 G53 G0 X-400. Y-160.5"))
    with pytest.raises(ValueError, match=r"leaves the axes at X-400\.0 Y-160\.5 Z0\.0 .* is X-762\.0 Y-160\.5 Z0\.0"):
        bt.tending.haas_autodoor(other, vmc, part_program="stray")
    bt.nc.program(other, "bad", "G54 G90 G0 X0. Y0.\nG43 Z5. H9\n")
    with pytest.raises(ValueError, match="line 2: tool length H9 is not in the tool table"):
        bt.tending.generic(other, vmc, part_program="bad")
    with pytest.raises(ValueError, match="fanuc_ri2: a part program is not supported yet"):
        bt.tending.fanuc_ri2(other, vmc, part_program="op10")
    # A part program runs on axes, and a clamped part rides a table that moves.
    plain_scene = scene_()
    plain = bt.parts.machine_tool(plain_scene, "vmc", **VF2, exchange=EXCHANGE)
    with pytest.raises(ValueError, match="was built without its axes — a part program runs on"):
        bt.tending.haas_autodoor(plain_scene, plain, part_program="op10")
    with pytest.raises(ValueError, match="its table stands still"):
        bt.tending.haas_autodoor(plain_scene, plain, parts=("a", "b"))


def test_the_machine_and_its_program_survive_a_project_file(tmp_path: Path) -> None:
    scene, vf2 = cell()
    sq = scene.sequence("cut")
    sq.step("machining", actions=[bt.seq.run_program(vf2.axes, "op10")])
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    loaded = bt.Scene.load_project(path)
    assert loaded.nc_machine_names == ["vf2/axes"] and loaded.nc_program("op10") == PROGRAM
    assert loaded.nc_machine("vf2/axes") == scene.nc_machine("vf2/axes")
    assert loaded.simulate_sequence("cut", max_duration=300.0).duration == pytest.approx(
        scene.simulate_sequence("cut", max_duration=300.0).duration)
    script = scene.generate_python()
    assert 'scene.define_nc_machine("vf2/axes", axes=("x", "y", "z")' in script
    assert 'scene.set_nc_tool("vf2/axes", 2, length=0.135, diameter=0.005)' in script
    assert 'bt.seq.run_program("vf2/axes", "op10")' in script


# ------------------------------------------------------------ what it cuts
def test_a_tool_is_a_body_and_its_table_entry_is_taken_from_it(tmp_path: Path) -> None:
    assert (MILL.length, DRILL.length) == pytest.approx((0.110, 0.135))
    assert (MILL.describe(), DRILL.describe()) == ("10 mm flat end mill", "5 mm drill")
    assert bt.nc.bull_nose(0.012, 0.002, flutes=0.02, length=0.1).describe() == "12 mm bull-nose end mill R2"
    with pytest.raises(ValueError, match="needs its `stickout`"):
        bt.nc.end_mill(0.010, flutes=0.02, holder=HOLDER)
    with pytest.raises(ValueError, match="not both"):
        bt.nc.end_mill(0.010, flutes=0.02, stickout=0.03, holder=HOLDER, length=0.1)
    with pytest.raises(ValueError, match="22 mm of flutes do not fit the 20 mm"):
        bt.nc.end_mill(0.010, flutes=0.022, stickout=0.020, holder=HOLDER)
    with pytest.raises(ValueError, match="or its `holder` and `stickout`"):
        bt.nc.drill(0.005, flutes=0.04)

    scene, vf2 = cutting_cell()
    tools = scene.nc_machine(vf2.axes)["tools"]
    assert (tools[1]["length"], tools[1]["diameter"]) == (0.110, 0.010)
    assert tools[1]["body"] == {"cutter": "flat", "diameter": 0.010, "flute_length": 0.022, "length": 0.110,
                                "holder": [0.040, 0.075]}
    assert tools[2]["body"]["cutter"] == "drill"
    # The program runs as it did on the entered lengths: same lengths.
    assert bt.nc.check(scene, vf2, "op10").duration == pytest.approx(bt.nc.check(cell()[0], "vf2/axes", "op10").duration)
    # Another length in the table leaves the tool as long as it is.
    wrong, _ = cutting_cell(t2=0.115)
    entry = wrong.nc_machine("vf2/axes")["tools"][2]
    assert (entry["length"], entry["body"]["length"]) == (0.115, 0.135)
    # A body that is not a tool is refused where it is entered.
    with pytest.raises(ValueError, match="tool 3: .* do not fit 50.0 mm from gauge line to tip"):
        scene.set_nc_tool(vf2.axes, 3, length=0.05,
                          body='{"cutter": "flat", "diameter": 0.01, "flute_length": 0.06, "length": 0.05}')
    with pytest.raises(ValueError, match="give the tool, or the length"):
        bt.nc.set_tool(scene, vf2, 4)
    # The bodies survive a project file and the generated script.
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    assert bt.Scene.load_project(path).nc_machine("vf2/axes") == scene.nc_machine("vf2/axes")
    assert 'scene.set_nc_tool("vf2/axes", 1, length=0.11, diameter=0.01, body="{' in scene.generate_python()


def test_a_dry_run_cuts_the_stock_and_books_it_by_line(tmp_path: Path) -> None:
    scene, vf2 = cutting_cell()
    before = (scene.joint_positions_of(vf2.axes), list(scene.obstacle_names), scene.obstacle_pose("part"))
    run = bt.nc.check(scene, vf2, "op10", stock="part")
    cut = run.machining
    # The pass on the line that feeds across, a hole on each line that drills.
    assert [(program, line, tool) for program, line, tool, _ in cut.lines] == [
        ("op10", 10, 1), ("op10", 17, 2), ("op10", 18, 2)]
    assert [volume for *_, volume in cut.lines] == pytest.approx([PASS, HOLE, HOLE], rel=0.01)
    assert cut.removed_volume == pytest.approx(PASS + 2 * HOLE, rel=0.01)
    assert cut.by_tool() == pytest.approx({1: PASS, 2: 2 * HOLE}, rel=0.01)
    assert cut.initial_volume == pytest.approx(0.04 * 0.04 * 0.06)
    assert cut.initial_volume - cut.removed_volume == pytest.approx(cut.remaining_volume)
    assert cut.ok and not cut.issues and cut.unbodied == () and cut.tolerance is None
    # Tool 1 from its change to the next, tool 2 from there to the end;
    # everything cut falls inside.
    (one, a, b), (two, c, d) = cut.tools
    assert (one, two) == (1, 2) and a < b == c < d == pytest.approx(run.duration)
    first, last = cut.cut_span
    assert a < first < b < last < d
    assert "0.42 cm³ removed of 96.00 (T1 0.20, T2 0.22)" in cut.summary()
    # A dry run moves nothing and adds nothing.
    assert (scene.joint_positions_of(vf2.axes), list(scene.obstacle_names), scene.obstacle_pose("part")) == before
    # The part is a mesh in the block's own frame.
    cut.part.save_obj(tmp_path / "part.obj")
    heights = [float(line.split()[3]) for line in (tmp_path / "part.obj").read_text().splitlines()
               if line.startswith("v ")]
    assert (min(heights), max(heights)) == pytest.approx((-0.030, 0.030))
    # … with the pass's floor half a millimetre under the top and the
    # holes' 6 mm under it.
    levels = {round(z, 6) for z in heights}
    assert {0.0295, 0.024} <= levels and all(z <= -0.03 + 1e-9 or z >= 0.024 - 1e-9 for z in levels)

    # Lengths in a table are not tools: the same run cuts nothing, and says so.
    bare, _ = cell()
    idle = bt.nc.check(bare, "vf2/axes", "op10", stock="part").machining
    assert (idle.unbodied, idle.removed_volume, idle.ok) == ((1, 2), 0.0, False)
    assert "no body in the tool table for T1, T2" in idle.summary()
    with pytest.raises(ValueError, match="are checked against a `stock`"):
        bt.nc.check(scene, vf2, "op10", fixtures=["part"])


def test_a_setup_stood_wrong_comes_out_as_a_line_and_a_volume(tmp_path: Path) -> None:
    """The four inputs meet at the cutter: the program, the work offset,
    the tool table, and the tool."""
    # The drill is 135 mm long and entered as 115: "10 mm over the part"
    # is 10 mm into it, at rapid, on the line that calls the length.
    scene, vf2 = cutting_cell(t2=0.115)
    cut = bt.nc.check(scene, vf2, "op10", stock="part").machining
    crash = cut.issues[0]
    assert (crash.kind, crash.program, crash.line, crash.tool, crash.what) == (
        "rapid_into_stock", "op10", 16, 2, "part")
    assert crash.volume == pytest.approx(math.pi * 0.0025 ** 2 * 0.0095, rel=0.01)
    assert str(crash).startswith("op10 line 16: the cutter meets `part` at rapid (T2, 186.")
    assert not cut.ok and all(issue.kind == "rapid_into_stock" for issue in cut.issues)
    assert cut.removed_volume > 3 * (PASS + 2 * HOLE)

    # The work zero entered 2 mm above the block: the pass is in the air
    # and the holes come up 2 mm short. Nothing crashes — the first part
    # is wrong, which only a comparison says.
    scene, vf2 = cutting_cell()
    x, y, z = scene.nc_machine(vf2.axes)["work_offsets"]["G54"]
    bt.nc.set_work_offset(scene, vf2, "G54", xyz=(x, y, z + 0.002))
    cut = bt.nc.check(scene, vf2, "op10", stock="part").machining
    assert not cut.issues and cut.by_tool() == pytest.approx({2: 2 * math.pi * 0.0025 ** 2 * 0.004}, rel=0.01)

    # Against a design: the uncut block, written as a mesh. Everything
    # the good setup removes is then missing from it …
    block = tmp_path / "block.obj"
    corners = [(sx * 0.02, sy * 0.02, sz * 0.03) for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
    faces = [(1, 3, 2), (2, 3, 4), (5, 6, 7), (6, 8, 7), (1, 2, 5), (2, 6, 5),
             (3, 7, 4), (4, 7, 8), (1, 5, 3), (3, 5, 7), (2, 4, 6), (4, 8, 6)]
    block.write_text("".join(f"v {x} {y} {z}\n" for x, y, z in corners)
                     + "".join(f"f {a} {b} {c}\n" for a, b, c in faces))
    scene, vf2 = cutting_cell()
    cut = bt.nc.check(scene, vf2, "op10", stock="part", design=block, tolerance=0.0).machining
    assert cut.gouge_volume == pytest.approx(cut.removed_volume, rel=1e-6) and cut.excess_volume == 0.0
    assert not cut.ok and cut.gouge_bounds[1][2] == pytest.approx(0.030) and cut.excess_bounds is None
    assert "against the design (±0 mm): 0.0 mm³ left over, 41" in cut.summary()
    # … and within a millimetre, the half-millimetre pass is no deviation
    # while the holes still are, from a millimetre down.
    cut = bt.nc.check(scene, vf2, "op10", stock="part", design=block, tolerance=0.001).machining
    assert cut.gouge_volume == pytest.approx(2 * math.pi * 0.0025 ** 2 * 0.005, rel=0.02)


def test_a_baked_cycle_cuts_the_part_where_the_bake_had_it_and_shows_it() -> None:
    scene, vf2 = cutting_cell()
    scene.add_box("blank", size=PART, position=(3.0, 1.0, 0.5))
    dry = bt.nc.check(scene, vf2, "op10", stock="part").machining
    hs = bt.tending.haas_autodoor(scene, vf2, part_program="op10", parts=("part", "blank"), clamp_s=0.2)
    # The cell's side, answering without moving.
    sq = scene.sequence("tend")
    sq.step("wait_done", transition=bt.seq.signal(hs.signal("part_done")))
    sq.step("safe", actions=[bt.seq.set_signal(hs.signal("cell_safe"))], transition=bt.seq.signal(hs.signal("door_open")))
    sq.step("ask", actions=[bt.seq.set_signal(hs.signal("clamp_req"))], transition=bt.seq.signal(hs.signal("clamp")))
    sq.step("start", actions=[bt.seq.set_signal(hs.signal("clamp_req"), False), bt.seq.set_signal(hs.signal("start_req"))],
            transition=bt.seq.signal(hs.signal("door_closed")))
    tl = scene.simulate_sequences(["tend", "vf2"], max_duration=300.0)
    # The bake cut what the dry run said it would: the same program, the
    # same block, clamped to the same table.
    cut = bt.nc.cut(tl, vf2, "part")
    assert cut.removed_volume == pytest.approx(dry.removed_volume, rel=1e-9)
    assert [(line, tool) for _, line, tool, _ in cut.lines] == [(10, 1), (17, 2), (18, 2)]
    assert cut.ok and scene.obstacle_visible("part")
    names = list(scene.obstacle_names)

    # With the picture: the block is shown as it is cut, by display meshes
    # that ride its track; the block itself is still what is collided.
    shown, pictured = bt.nc.animate(scene, tl, vf2, "part", stage_period=1.0)
    assert pictured.removed_volume == pytest.approx(cut.removed_volume, rel=1e-9)
    stages = [name for name in scene.obstacle_names if name.startswith("part_cut/")]
    assert stages[0] == "part_cut/raw" and len(stages) > 3 and stages[1] == "part_cut/s000"
    tools = [name for name in scene.obstacle_names if name.startswith("vf2/axes/tool/")]
    assert tools == [f"vf2/axes/tool/T{n}/{piece}" for n in (1, 2) for piece in ("cutter", "shank", "holder")]
    assert set(scene.obstacle_names) == {*names, *stages, *tools}
    assert not scene.obstacle_visible("part") and scene.obstacle_enabled("part")
    assert not any(scene.obstacle_enabled(name) for name in stages + tools)
    first, last = pictured.cut_span
    # Before the first cut the raw block shows; after the last, the part —
    # and each is where the block is, on the table as it moves.
    assert shown.object_visible("part_cut/raw", first - 0.05) and not shown.object_visible("part_cut/raw", first + 0.05)
    assert shown.object_visible(stages[-1], last + 0.05) and shown.object_visible(stages[-1], shown.duration)
    assert not shown.object_visible(stages[-1], first)
    for name, t in (("part_cut/raw", first - 0.05), (stages[-1], last + 0.05), (stages[-1], shown.duration)):
        assert shown.object_pose(name, t)[0] == pytest.approx(shown.object_pose("part", t)[0], abs=1e-9)
    assert shown.object_pose("part", (first + last) / 2)[0] != pytest.approx(shown.object_pose("part", 0.0)[0], abs=1e-3)
    # Each tool stands in the spindle while it is the one in it: the
    # holder's middle half its length under the nose.
    (_, a, b), (_, c, _) = pictured.tools
    for holder, t in (("vf2/axes/tool/T1/holder", (a + b) / 2), ("vf2/axes/tool/T2/holder", c + 1.0)):
        nose, _ = scene.link_pose_at("head", shown.sample(t, "vf2/axes"), robot="vf2/axes")
        assert shown.object_visible(holder, t)
        assert shown.object_pose(holder, t)[0] == pytest.approx((nose[0], nose[1], nose[2] - 0.0375), abs=1e-9)
    assert not shown.object_visible("vf2/axes/tool/T1/holder", c + 1.0)
    assert not shown.object_visible("vf2/axes/tool/T2/holder", a + 1.0)
    # Nothing the cell verifies has changed.
    assert shown.duration == tl.duration and float(shown.min_clearance()) == pytest.approx(float(tl.min_clearance()))


# A machine's picture that draws an umbrella tool changer, in small: the
# carousel 500 mm left of the spindle line at the height of the nose at
# home, on a shuttle with 230 mm of travel — so a pocket 270 mm out is
# under the spindle with the shuttle in — and four pockets.
CHANGER_MANIFEST = """
schema_version: '0.1'
id: acme/vmc/changer/r1
category: machine_tool.vmc
name: A machine with a tool changer
specs:
  tool_capacity: 4
assets:
  usd: usd/model.usda
"""

CHANGER_USD = """#usda 1.0
(
    defaultPrim = "Machine"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Machine" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "body" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "visual_0" { double size = 0.4 }
    }

    def Xform "head" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        double3 xformOp:translate = (0, -0.2615, 1.463)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cube "visual_0" { double size = 0.2 }
    }

    def Xform "tool_shuttle" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        double3 xformOp:translate = (-0.5, -0.2615, 1.463)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cube "visual_0" { double size = 0.1 }
    }

    def Xform "tool_carousel" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        double3 xformOp:translate = (-0.5, -0.2615, 1.463)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cube "visual_0" { double size = 0.3 }
    }

    def Scope "joints"
    {
        def PhysicsPrismaticJoint "z"
        {
            rel physics:body0 = </Machine/body>
            rel physics:body1 = </Machine/head>
            uniform token physics:axis = "Z"
            point3f physics:localPos0 = (0, -0.2615, 1.463)
            float physics:lowerLimit = -0.508
            float physics:upperLimit = 0
        }

        def PhysicsPrismaticJoint "tool_shuttle"
        {
            rel physics:body0 = </Machine/body>
            rel physics:body1 = </Machine/tool_shuttle>
            uniform token physics:axis = "X"
            point3f physics:localPos0 = (-0.5, -0.2615, 1.463)
            float physics:lowerLimit = 0
            float physics:upperLimit = 0.23
        }

        def PhysicsRevoluteJoint "tool_carousel"
        {
            rel physics:body0 = </Machine/tool_shuttle>
            rel physics:body1 = </Machine/tool_carousel>
            uniform token physics:axis = "Z"
        }
    }
}
"""


def changer_pack(directory: Path, usd: str = CHANGER_USD, manifest: str = CHANGER_MANIFEST) -> Path:
    (directory / "usd").mkdir(parents=True)
    (directory / "manifest.yaml").write_text(manifest)
    (directory / "usd" / "model.usda").write_text(usd)
    return directory


def test_a_tool_changer_the_picture_draws_is_worked_at_each_tool_change(tmp_path: Path) -> None:
    scene, vf2 = cutting_cell(shape=changer_pack(tmp_path / "changer"))
    plain, _ = cutting_cell()
    # The changer is two more joints of the machine's mechanism, and the
    # control's to work: the shuttle between its two ends, the carousel of
    # as many pockets as the picture says.
    assert vf2.changer == ("tool_shuttle", "tool_carousel")
    names = scene.robot_of(vf2.axes).joint_names
    assert sorted(names) == ["tool_carousel", "tool_shuttle", "x", "y", "z"]
    control = scene.nc_machine(vf2.axes)["changer"]
    assert (control["shuttle"], control["carousel"], control["pockets"]) == ("tool_shuttle", "tool_carousel", 4)
    assert control["reach"] == pytest.approx((0.0, 0.23))
    travel = control["reach"][1]              # (the file's own figure: single precision)
    assert plain.nc_machine("vf2/axes")["changer"] is None and sorted(plain.robot_of("vf2/axes").joint_names) == ["x", "y", "z"]
    # It takes no time of its own: the program is as long, and its axes
    # are where they are, as on the machine drawn without one.
    run, bare = bt.nc.check(scene, vf2, "op10"), bt.nc.check(plain, "vf2/axes", "op10")
    assert run.duration == pytest.approx(bare.duration, abs=1e-12) and run.positions == bare.positions

    def baked(cell):
        # (The part is clamped to the table, so the program cuts it.)
        cell.attach("part", link="table", robot="vf2/axes")
        sq = cell.sequence("run")
        sq.step("cut", actions=[bt.seq.run_program("vf2/axes", "op10")], transition=bt.seq.done())
        return cell.simulate_sequence("run")

    tl = baked(scene)
    cut = bt.nc.cut(tl, vf2, "part")
    assert cut.removed_volume == pytest.approx(PASS + 2 * HOLE, rel=0.02)
    (first, a1, b1), (second, a2, b2) = cut.changes
    assert (first, second) == (1, 2) and (b1 - a1, b2 - a2) == pytest.approx((4.2, 4.2))
    assert [(tool, start) for tool, start, _ in cut.tools] == [(1, pytest.approx(b1)), (2, pytest.approx(b2))]
    run_in = 0.3 * 4.2

    def joints(t: float) -> dict:
        return dict(zip(names, tl.sample(t, vf2.axes)))

    # Tool 1 lives in pocket 1, which stands at the spindle's side: the
    # shuttle runs in and out. For tool 2 the carousel turns its pocket to
    # the spindle in between — a quarter turn back, of four pockets — and
    # the head stands at the top through all of it.
    for t, reach, turn in ((a1, 0.0, 0.0), (a1 + run_in / 2, travel / 2, 0.0), (a1 + run_in, travel, 0.0),
                           (b1 - run_in, travel, 0.0), (b1, 0.0, 0.0), ((b1 + a2) / 2, 0.0, 0.0),
                           (a2 + run_in, travel, 0.0), ((a2 + b2) / 2, travel, -math.pi / 4),
                           (b2 - run_in, travel, -math.pi / 2), (b2, 0.0, -math.pi / 2),
                           (tl.duration, 0.0, -math.pi / 2)):
        at = joints(t)
        assert (at["tool_shuttle"], at["tool_carousel"]) == pytest.approx((reach, turn), abs=1e-9), t
    assert joints((a2 + b2) / 2)["z"] == pytest.approx(0.0, abs=1e-12)
    # Its pictures go with it: the carousel's to the spindle and round,
    # the shuttle's to the spindle.
    (cx, cy, cz), level = scene.obstacle_pose("vf2/shape/tool_carousel")
    assert (cx, cy, cz) == pytest.approx((-0.5, -0.2615, 1.463)) and level == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert tl.object_pose("vf2/shape/tool_carousel", a2 + run_in)[0] == pytest.approx((cx + travel, cy, cz), abs=1e-9)
    at, turned = tl.object_pose("vf2/shape/tool_carousel", b2)
    assert at == pytest.approx((cx, cy, cz), abs=1e-9)
    assert turned == pytest.approx((0.0, 0.0, -math.sin(math.pi / 4), math.cos(math.pi / 4)), abs=1e-9)
    assert tl.object_pose("vf2/shape/tool_shuttle", a2 + run_in)[0] == pytest.approx((cx + travel, cy, cz), abs=1e-9)
    assert not scene.obstacle_enabled("vf2/shape/tool_carousel")

    # With the tools drawn, a tool that is not in the spindle is in its
    # pocket — seen, where on a machine without a changer it is put away.
    shown, _ = bt.nc.animate(scene, tl, vf2, "part", stage_period=1.0)

    def nose(t: float):
        return scene.link_pose_at("head", shown.sample(t, vf2.axes), robot=vf2.axes)[0]

    def holder(tool: int, t: float):
        assert shown.object_visible(f"vf2/axes/tool/T{tool}/holder", t), (tool, t)
        return shown.object_pose(f"vf2/axes/tool/T{tool}/holder", t)[0]

    # Pocket 1 is where the nose is at the top, less the shuttle's travel;
    # pocket 2 a quarter turn on round the carousel's axis.
    nx, ny, nz = nose(0.0)
    under = nz - 0.0375                       # the holder's middle, half its length under the nose
    rx, ry = nx - travel - cx, ny - cy
    assert (rx, ry) == pytest.approx((0.27, 0.0), abs=1e-7)
    assert holder(1, 0.0) == pytest.approx((cx + rx, cy + ry, under), abs=1e-9)
    assert holder(2, 0.0) == pytest.approx((cx - ry, cy + rx, under), abs=1e-9)
    # Each goes between its pocket and the spindle without a jump: the
    # pocket is under the spindle when the shuttle is in.
    for tool, t in ((1, b1 - run_in), (1, a2 + run_in), (2, b2 - run_in)):
        assert holder(tool, t - 1e-3) == pytest.approx(holder(tool, t + 1e-3), abs=1e-3)
        assert holder(tool, t) == pytest.approx((nx, ny, under), abs=1e-9)
    # In the spindle it goes where the head goes, the other one waiting …
    cutting = (b1 + a2) / 2
    hx, hy, hz = nose(cutting)
    assert hz < nz - 0.1 and holder(1, cutting) == pytest.approx((hx, hy, hz - 0.0375), abs=1e-9)
    assert holder(2, cutting) == pytest.approx((cx - ry, cy + rx, under), abs=1e-9)
    # … and once tool 2 is in the spindle, tool 1 is in pocket 1, which
    # has gone a quarter turn back with the carousel.
    end = shown.duration
    assert holder(2, end) == pytest.approx((nose(end)[0], nose(end)[1], nose(end)[2] - 0.0375), abs=1e-9)
    assert holder(1, end) == pytest.approx((cx + ry, cy - rx, under), abs=1e-9)
    # Nothing the cell verifies has changed, here or against the machine
    # drawn without a changer.
    without = baked(plain)
    assert shown.duration == tl.duration == pytest.approx(without.duration, abs=1e-12)
    assert bt.nc.cut(without, "vf2/axes", "part").removed_volume == pytest.approx(cut.removed_volume, rel=1e-9)
    assert [v for change in bt.nc.cut(without, "vf2/axes", "part").changes for v in change] == pytest.approx(
        [v for change in cut.changes for v in change])

    # The changer survives a project file and the generated script.
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    assert bt.Scene.load_project(path).nc_machine("vf2/axes")["changer"] == scene.nc_machine("vf2/axes")["changer"]
    assert 'changer=("tool_shuttle", "tool_carousel", 4)' in scene.generate_python()


def test_a_tool_changer_the_control_cannot_work_is_refused_by_what_is_wrong(tmp_path: Path) -> None:
    # A carousel with stops cannot go the short way round to every pocket.
    stops = CHANGER_USD.replace(
        'rel physics:body1 = </Machine/tool_carousel>\n            uniform token physics:axis = "Z"',
        'rel physics:body1 = </Machine/tool_carousel>\n            uniform token physics:axis = "Z"\n'
        '            float physics:lowerLimit = -180\n            float physics:upperLimit = 180')
    assert stops != CHANGER_USD
    with pytest.raises(ValueError, match="draws a tool changer the control cannot work .* turns freely"):
        machine(scene_(), shape=changer_pack(tmp_path / "stops", usd=stops))
    # A carousel that is not on the shuttle does not go to the spindle.
    loose = CHANGER_USD.replace("rel physics:body0 = </Machine/tool_shuttle>", "rel physics:body0 = </Machine/body>")
    assert loose != CHANGER_USD
    with pytest.raises(ValueError, match="draws a tool changer the control cannot work .* carried by the shuttle"):
        machine(scene_(), shape=changer_pack(tmp_path / "loose", usd=loose))
    # And the picture has to say how many pockets there are.
    silent = CHANGER_MANIFEST.replace("specs:\n  tool_capacity: 4\n", "")
    assert silent != CHANGER_MANIFEST
    with pytest.raises(ValueError, match="does not say how many pockets"):
        machine(scene_(), shape=changer_pack(tmp_path / "silent", manifest=silent))
    # A picture with no changer in it is a machine without one, as before.
    bare = CHANGER_USD.replace('def Xform "tool_carousel"', 'def Xform "magazine"').replace(
        "</Machine/tool_carousel>", "</Machine/magazine>")
    scene = scene_()
    vf2 = machine(scene, shape=changer_pack(tmp_path / "bare", usd=bare))
    assert vf2.changer is None and scene.nc_machine(vf2.axes)["changer"] is None
    assert sorted(scene.robot_of(vf2.axes).joint_names) == ["x", "y", "z"]
    assert scene.obstacle_pose("vf2/shape/magazine")[0] == pytest.approx((-0.5, -0.2615, 1.463))


def test_the_setup_sheet_is_the_cell_written_down() -> None:
    scene, vf2 = cutting_cell()
    sheet = bt.nc.setup_sheet(scene, vf2, "op10", stock="part", fixtures=["vf2/bed"], title="WP-40 OP10")
    assert sheet.startswith("# WP-40 OP10\n")
    run = bt.nc.check(scene, vf2, "op10")
    assert f"- Cycle: {run.duration:.1f} s — cutting" in sheet and "2 tool changes" in sheet
    # The block on the table, the zero on the middle of its top.
    assert "- `part`: 40.0 x 40.0 x 60.0 mm" in sheet
    assert "X+330.000 Y+0.000 Z+30.000 mm (top face 60.0 mm above the table)" in sheet
    assert "| G54 | X-51.000 Y-211.500 Z-550.000 | X+0.000 Y+0.000 Z+30.000 |" in sheet
    assert "| `vf2/bed` |" in sheet
    # The tools: what is in the pocket, and what the table says.
    assert "| T1 | 10 mm flat end mill | 22 | 40 x 75 | 110.000 | 110.000 |  |" in sheet
    assert "| T2 | 5 mm drill | 45 | 40 x 75 | 135.000 | 135.000 |  |" in sheet
    assert "Spindle, as programmed: T1 S6000; T2 S4000." in sheet
    assert "- Removed: 0.42 cm³ of 96.00 — T1 0.20 cm³, T2 0.22 cm³" in sheet
    assert "- Nothing met anything it should not have." in sheet and "design" not in sheet
    # Stood wrong, the sheet says where: the entry that is not the tool,
    # and the line it shows on.
    wrong, vmc = cutting_cell(t2=0.115)
    sheet = bt.nc.setup_sheet(wrong, vmc, "op10", stock="part")
    assert "| T2 | 5 mm drill | 45 | 40 x 75 | 135.000 | 115.000 | **entry -20.000 mm from the tool** |" in sheet
    assert "- **op10 line 16: the cutter meets `part` at rapid (T2, " in sheet
    # Without a stock it is the tables and the timing.
    assert "- Not cut: no stock was named." in bt.nc.setup_sheet(scene, "vf2/axes", "op10")



# ------------------------------------------------- a program that runs the cell
# The same part, and then the cell: the vise open, the window open, a relay
# that calls whoever tends the machine and waits for the answer, the vise
# shut, the window shut — and over again.
CELL_PROGRAM = PROGRAM.replace(
    "N210 M30\n", "N300 M71\nN310 M180\nN320 M21\nN330 M70\nN340 M181\nN350 M99\n")


def tended(text: str = CELL_PROGRAM, **options):
    """`cutting_cell` run by its part program: the Haas table on the
    machine's window (`M180`/`M181`), its vise and one relay."""
    scene, vf2 = cutting_cell()
    bt.nc.program(scene, "cell", text)
    hs = bt.tending.haas_mfin(scene, vf2, "cell", **{"parts": ["part"], "door_codes": (180, 181), **options})
    return scene, vf2, hs


def answering(scene, hs, *, safe: bool = True, watch: float = 8.0) -> None:
    """Whoever tends the machine, as a program `cell`: says the cell is
    safe, takes a second over the relay's call, answers on levels, and
    watches the machine get back to work."""
    S = bt.seq
    sq = scene.sequence("cell")
    sq.step("safe", actions=[S.set_signal(hs.signal("cell_safe"))] if safe else [],
            transition=S.signal(hs.signal("m21")))
    sq.step("work", transition=S.elapsed(1.0))
    sq.step("fin", actions=[S.set_signal(hs.signal("m_fin"))], transition=S.signal(hs.signal("m21"), False))
    sq.step("clear", actions=[S.set_signal(hs.signal("m_fin"), False)],
            transition=S.signal(hs.signal("door_closed")))
    sq.step("watch", transition=S.elapsed(watch))


def test_a_part_program_that_runs_the_cell_is_the_machines_program(tmp_path: Path) -> None:
    scene, vf2, hs = tended()
    assert (hs.template, hs.program, hs.door) == ("haas_mfin", "vf2", "vf2/side_door")
    # The machine's M-code table: what the control hands over, in words.
    assert scene.nc_machine(vf2.axes)["m_codes"] == {
        21: "call the cell", 70: "vise clamp", 71: "vise unclamp", 180: "door open", 181: "door closed"}
    # Read whole, the calls are events that take no time: the dry run is
    # the control's own share of the cycle.
    dry = bt.nc.check(scene, vf2, "cell", stock="part")
    lines = CELL_PROGRAM.splitlines()
    assert dry.functions() == [(1 + lines.index(f"N{n} M{code}"), code)
                               for n, code in ((300, 71), (310, 180), (320, 21), (330, 70), (340, 181))]
    assert "5 machine functions (M71, M180, M21, M70, M181) not timed" in dry.summary()
    assert dry.duration == pytest.approx(bt.nc.check(scene, vf2, "op10").duration, abs=1e-9)
    assert dry.machining.removed_volume == pytest.approx(PASS + 2 * HOLE, rel=0.01)

    answering(scene, hs)
    tl = scene.simulate_sequences(["cell", hs.program], max_duration=120.0)
    # The steps are the program's: its blocks by their lines, its M codes
    # where they stand — and after the M99, the first block again.
    steps = [name.removeprefix("vf2/") for name, _, _ in tl.step_spans if name.startswith("vf2/")]
    assert steps == [
        "cycle start", "N30-N200?", "N30-N200",
        "N300 M71 vise unclamp", "N310 M180 door open?", "N310 M180 door open",
        "N320 M21 call the cell", "N320 M21 call the cell — done", "N330 M70 vise clamp",
        "N340 M181 door closed?", "N340 M181 door closed",
        "cycle start", "N30-N200?", "N30-N200",
    ]
    cut, again = (span for span in tl.step_spans if span[0] == "vf2/N30-N200")
    assert cut[2] - cut[1] == pytest.approx(dry.duration, abs=0.02)
    assert [name for name, *_ in tl.moves(vf2.axes)] == ["cell#0", "cell#0"]
    # The window opens after the cut and shuts before the next, the relay
    # is up for as long as its answer takes and down before the M-FIN goes.
    opened, closed = tl.signal(hs.signal("door_open")), tl.signal(hs.signal("door_closed"))
    relay, fin = tl.signal(hs.signal("m21")), tl.signal(hs.signal("m_fin"))
    assert cut[2] < opened.rising_edges()[0] <= relay.rising_edges()[0]
    assert relay.falling_edges()[0] == pytest.approx(fin.rising_edges()[0], abs=0.011)
    assert fin.falling_edges()[0] >= relay.falling_edges()[0]
    assert opened.falling_edges()[0] > fin.falling_edges()[0] and closed.rising_edges()[-1] <= again[1]

    # The vise holds what stands in it: clamped as the cell starts, the
    # part rides the table through the cut; it is free while the vise is
    # open, and the M70 has it again for the next cut.
    clamp = tl.signal(hs.signal("clamp"))
    assert clamp.high_spans()[0] == pytest.approx((0.0, cut[2]), abs=1e-9)
    names = scene.robot_of(vf2.axes).joint_names
    start, end = next((a, b) for a, b, line in dry.spans if line == 1 + lines.index("N80 X30. F900."))
    mid = (start + end) / 2
    here, there = tl.object_pose("part", 0.0)[0], tl.object_pose("part", mid)[0]
    q0, q1 = (dict(zip(names, tl.sample(t, vf2.axes))) for t in (0.0, mid))
    assert (there[0] - here[0], there[1] - here[1]) == pytest.approx(
        (-(q1["x"] - q0["x"]), -(q1["y"] - q0["y"])), abs=1e-9)
    assert abs(there[0] - here[0]) > 0.3
    assert tl.object_pose("part", opened.rising_edges()[0])[0] == pytest.approx(here, abs=1e-9)
    assert tl.object_pose("part", tl.duration)[0][0] < here[0] - 0.3
    # The control's lanes are there unasked: the spindle between its M3
    # and its M5, the tool in it, the line it runs — none while it waits
    # for the relay's answer.
    spindle = tl.signal(hs.signal("spindle"))
    on = [t for t, _, what, value in dry.events if what == "spindle" and value]
    off = [t for t, _, what, value in dry.events if what == "spindle" and not value and t > on[0]]
    assert spindle.high_spans()[0] == pytest.approx((on[0], off[0]), abs=1e-9)
    assert not tl.signal(hs.signal("coolant")).high_spans()
    tool, line, rpm = (tl.word(f"vf2/axes.{name}") for name in ("tool", "line", "rpm"))
    assert [tool.value_at(t) for t in (0.0, on[0], on[1])] == [0, 1, 2]
    assert (rpm.value_at(on[0] + 0.1), rpm.value_at(off[0] + 0.1)) == (6000, 0)
    assert line.value_at(mid) == 1 + lines.index("N80 X30. F900.")
    assert line.value_at(relay.rising_edges()[0] + 0.5) == 0

    # The interlock table is read off the same program: the M180's row
    # carries its guard, and the alarm that is its other arm.
    table = scene.interlocks([hs.program]).to_markdown()
    for row in (
        "| N310 M180 door open | device `vf2/side_door → open` | `(vf2/cell_safe AND vf2/front_door/closed)` |",
        ("| N310 M180 door open ALARM — the cell is not safe or the front door is open | signal `vf2/alarm := TRUE` | "
         "`NOT (vf2/cell_safe AND vf2/front_door/closed)` |"),
        ("| N30-N200 | program `program cell#0 (vf2/axes)` | "
         "`(vf2/side_door/closed AND vf2/front_door/closed AND NOT vf2/panel/estop)` |"),
        ("| N320 M21 call the cell | signal `vf2/m21 := TRUE` | `(INPOS(vf2/side_door) AND vf2/side_door/open)` "
         "| N310 M180 door open |"),
    ):
        assert row in table, table
    assert "NOT TRUE" not in table
    # … and so are the setup sheet's calls, the PLCopen export, and the
    # project file with the script that writes it again.
    sheet = bt.nc.setup_sheet(scene, vf2, "cell", stock="part")
    assert "- Machine functions, in order (the cell's time, not in the cycle above): M71 vise unclamp (line" in sheet
    scene.export_plcopen(tmp_path / "cell.xml", name="cell")
    assert "cell#0" in (tmp_path / "cell.xml").read_text()
    scene.save_project(tmp_path / "cell.botrail")
    loaded = bt.Scene.load_project(tmp_path / "cell.botrail")
    assert loaded.nc_machine("vf2/axes") == scene.nc_machine("vf2/axes")
    again_tl = loaded.simulate_sequences(["cell", hs.program], max_duration=120.0)
    assert [(n, round(a, 9), round(b, 9)) for n, a, b in again_tl.step_spans] == [
        (n, round(a, 9), round(b, 9)) for n, a, b in tl.step_spans]
    script = scene.generate_python()
    for needle in ('scene.set_nc_m_code("vf2/axes", 180, "door open")',
                   'bt.seq.run_program("vf2/axes", "cell", block=0)',
                   'scene.bind_grasp("vf2/clamp"'):
        assert needle in script, needle


def test_a_cell_that_is_not_safe_alarms_at_the_door_and_stays_there() -> None:
    # Nobody says the cell is safe: the program alarms at its M180 — the
    # door never commanded — and the cell waits for a call that never
    # comes. The bake says where both stand.
    scene, _, hs = tended()
    answering(scene, hs, safe=False)
    with pytest.raises(ValueError) as stall:
        scene.simulate_sequences(["cell", hs.program], max_duration=60.0)
    assert "cell/safe" in str(stall.value)
    assert "vf2/N310 M180 door open ALARM — the cell is not safe or the front door is open" in str(stall.value)

    # Safe too late is not safe: a control that finds the input low
    # alarms, it does not wait for it. The cell raises it once it sees
    # the alarm — and nothing moves.
    scene, _, hs = tended()
    S = bt.seq
    sq = scene.sequence("late")
    sq.step("wait", transition=S.signal(hs.signal("alarm")))
    sq.step("safe", actions=[S.set_signal(hs.signal("cell_safe"))], transition=S.elapsed(5.0))
    tl = scene.simulate_sequences(["late", hs.program], max_duration=60.0)
    assert not tl.signal(hs.signal("door_open")).high_spans()
    assert tl.signal(hs.signal("alarm")).value_at(tl.duration) and not tl.signal(hs.signal("running")).value_at(tl.duration)
    stood = [name for name, _, _ in tl.step_spans if name.startswith("vf2/")][-1]
    assert stood == "vf2/N310 M180 door open ALARM — the cell is not safe or the front door is open"
    assert tl.signal(hs.signal("door_closed")).high_spans() == [(0.0, tl.duration)]


def test_a_program_that_cuts_without_clamping_leaves_its_part_behind() -> None:
    # No M70 after the exchange: the vise stays open, and the table goes
    # back under the spindle without the part.
    scene, vf2, hs = tended(CELL_PROGRAM.replace("N330 M70\n", ""))
    answering(scene, hs)
    tl = scene.simulate_sequences(["cell", hs.program], max_duration=120.0)
    names = scene.robot_of(vf2.axes).joint_names
    x = [dict(zip(names, tl.sample(t, vf2.axes)))["x"] for t in (0.0, tl.duration)]
    assert x[1] > x[0] + 0.3
    assert tl.object_pose("part", tl.duration)[0] == pytest.approx(tl.object_pose("part", 0.0)[0], abs=1e-9)
    assert len(tl.signal(hs.signal("clamp")).high_spans()) == 1


def test_what_a_cell_program_asks_that_nobody_answers_is_refused_by_line() -> None:
    lines = CELL_PROGRAM.splitlines()
    # An M code in nobody's table — here the auto door's, on a machine
    # whose table has the window's.
    scene, vf2 = cutting_cell()
    bt.nc.program(scene, "cell", CELL_PROGRAM.replace("N310 M180", "N310 M80"))
    with pytest.raises(ValueError, match=rf"line {1 + lines.index('N310 M180')}: M80 is not supported") as err:
        bt.tending.haas_mfin(scene, vf2, "cell", parts=["part"], door_codes=(180, 181))
    assert "(M21, M70, M71, M180, M181)" in str(err.value)
    # A relay called with the table away from where the arm was taught.
    with pytest.raises(ValueError, match=r"M21 \(call the cell\) is called with the axes at X-381.0") as err:
        tended(CELL_PROGRAM.replace("N200 G53 G0 X-762. Y-160.5", "N200 G53 G0 X-381. Y-160.5"))
    assert f"line {1 + lines.index('N320 M21')}" in str(err.value) and "X-762.0 Y-160.5 Z0.0" in str(err.value)
    # Run whole — one step, as a program with no calls runs — the control
    # would wait at the first of them for ever: refused, with the line.
    scene, vf2, _ = tended()
    sq = scene.sequence("whole")
    sq.step("run", actions=[bt.seq.run_program(vf2.axes, "cell")])
    with pytest.raises(ValueError, match=rf"line {1 + lines.index('N300 M71')}: M71 is a machine function"):
        scene.simulate_sequence("whole")
    # The control's own codes are not the machine's to take, a relay that
    # waits for M-FIN is one of M21 to M28, and a vise holds something.
    with pytest.raises(ValueError, match="M6 is the control's own"):
        scene.set_nc_m_code(vf2.axes, 6, "tool change")
    with pytest.raises(ValueError, match="M21 to M28"):
        tended(relays=(51,))
    with pytest.raises(ValueError, match="name the parts the vise holds"):
        tended(parts=[])


# ----------------------------------------------------- an automatic front door
# The same cell worked through the front: the program's M80 parts the two
# leaves of the front door, its M81 brings them together again.
FRONT_PROGRAM = PROGRAM.replace(
    "N210 M30\n", "N300 M71\nN310 M80\nN320 M21\nN330 M70\nN340 M81\nN350 M99\n")


def front_cell(text: str = FRONT_PROGRAM, **options):
    """`cutting_cell` on a machine whose front door is driven, run by `text`."""
    scene = scene_()
    vf2 = machine(scene, front_drive="air", **options)
    (tx, ty, tz), _ = scene.frame("vf2/table")
    scene.add_box("part", size=PART, position=(tx + 0.33, ty, tz + PART[2] / 2))
    bt.nc.set_work_offset(scene, vf2, "G54", at=(tx + 0.33, ty, tz + PART[2]))
    bt.nc.set_tool(scene, vf2, 1, MILL)
    bt.nc.set_tool(scene, vf2, 2, DRILL)
    bt.nc.program(scene, "cell", text)
    return scene, vf2, bt.tending.haas_mfin(scene, vf2, "cell", parts=["part"])


def test_an_automatic_front_door_parts_on_the_programs_m80() -> None:
    scene, vf2, hs = front_cell()
    # Two leaves on two axes, each with its own ends — and the front door
    # is shut when both are.
    assert vf2.front_doors == ("vf2/front_door/left", "vf2/front_door/right")
    assert vf2.front_closed == ("vf2/front_door/left/closed", "vf2/front_door/right/closed")
    assert vf2.front_open == ("vf2/front_door/left/open", "vf2/front_door/right/open")
    assert vf2.front_door_lane == vf2.front_closed[0] and vf2.front_door_travel == pytest.approx(0.939 / 2)
    assert "vf2/front_door/closed" not in vf2.sensors and "vf2/front_door/leaf" not in scene.obstacle_names
    # Where an arm waits at it: the middle of the opening, 150 mm out.
    (ex, ey, ez), _ = scene.frame("vf2/front_entry")
    assert (ex, ez) == pytest.approx((0.0, 0.816 + 0.954 / 2)) and ey < -1.984 / 2 - 0.15
    # The front door takes the auto door's codes, the side door the auto
    # window's: each its own row of the machine's table.
    assert scene.nc_machine(vf2.axes)["m_codes"] == {
        21: "call the cell", 70: "vise clamp", 71: "vise unclamp",
        80: "front door open", 81: "front door closed", 180: "side door open", 181: "side door closed"}
    doors = {row["names"][0]: row for row in scene.bom().rows if row["category"] == "machine_tool.door"}
    assert set(doors) == {"vf2/side_door", "vf2/front_door/left", "vf2/front_door/right"}
    assert doors["vf2/front_door/left"]["attributes"]["stroke_mm"] == pytest.approx(469.5)

    S = bt.seq
    sq = scene.sequence("cell")
    sq.step("safe", actions=[S.set_signal(hs.signal("cell_safe"))], transition=S.signal(hs.signal("m21")))
    sq.step("work", transition=S.elapsed(1.0))
    sq.step("fin", actions=[S.set_signal(hs.signal("m_fin"))], transition=S.signal(hs.signal("m21"), False))
    sq.step("clear", actions=[S.set_signal(hs.signal("m_fin"), False)],
            transition=S.all_of(*(S.signal(lane) for lane in vf2.front_closed)))
    sq.step("watch", transition=S.elapsed(3.0))
    tl = scene.simulate_sequences(["cell", hs.program], max_duration=120.0)
    steps = [name.removeprefix("vf2/") for name, _, _ in tl.step_spans if name.startswith("vf2/N3")]
    assert steps[:8] == [
        "N30-N200?", "N30-N200", "N300 M71 vise unclamp", "N310 M80 front door open?", "N310 M80 front door open",
        "N320 M21 call the cell", "N320 M21 call the cell — done", "N330 M70 vise clamp"]
    # Both leaves leave their closed ends together and stand open together
    # — 469.5 mm apart from the middle, each its own way — before the
    # relay calls anybody; the side door never moves.
    opened = [tl.signal(lane).rising_edges()[0] for lane in vf2.front_open]
    assert opened[0] == pytest.approx(opened[1], abs=1e-9)
    assert opened[0] <= tl.signal(hs.signal("m21")).rising_edges()[0]
    for leaf, sign in (("vf2/front_door/leaf_left", -1.0), ("vf2/front_door/leaf_right", 1.0)):
        shut, apart = tl.object_pose(leaf, 0.0)[0], tl.object_pose(leaf, opened[0])[0]
        assert [b - a for a, b in zip(shut, apart)] == pytest.approx([sign * 0.4695, 0.0, 0.0], abs=1e-9)
        assert tl.object_pose(leaf, tl.duration)[0] == pytest.approx(shut, abs=1e-9)
    assert tl.signal(hs.signal("door_closed")).high_spans() == [(0.0, tl.duration)]
    # The cut starts on both doors: every closed lane of the front door is
    # in its guard, and in the side door's.
    table = scene.interlocks([hs.program]).to_markdown()
    assert ("| N30-N200 | program `program cell#0 (vf2/axes)` | `(vf2/side_door/closed AND "
            "vf2/front_door/left/closed AND vf2/front_door/right/closed AND NOT vf2/panel/estop)` |") in table
    assert ("| N310 M80 front door open | device `vf2/front_door/left → open` | "
            "`(vf2/cell_safe AND vf2/side_door/closed)` |") in table


def test_the_two_doors_are_never_open_together() -> None:
    # A program that opens the window with the front door still open: the
    # control alarms at that line, and the window stays shut.
    scene, vf2, hs = front_cell(FRONT_PROGRAM.replace("N320 M21\n", "N315 M180\nN320 M21\n"))
    S = bt.seq
    sq = scene.sequence("cell")
    sq.step("safe", actions=[S.set_signal(hs.signal("cell_safe"))], transition=S.signal(hs.signal("alarm")))
    sq.step("look", transition=S.elapsed(2.0))
    tl = scene.simulate_sequences(["cell", hs.program], max_duration=120.0)
    stood = [name for name, _, _ in tl.step_spans if name.startswith("vf2/")][-1]
    assert stood == "vf2/N315 M180 side door open ALARM — the cell is not safe or the front door is open"
    assert not tl.signal(hs.signal("door_open")).high_spans()
    assert tl.signal(vf2.front_open[0]).value_at(tl.duration)

    # What a front door cannot be.
    with pytest.raises(ValueError, match="front_drive is 'air', 'servo' or None"):
        machine(scene_(), front_drive="hydraulic")
    with pytest.raises(ValueError, match="front_drive needs a front door"):
        machine(scene_(), front_drive="air", front_door=None)
    with pytest.raises(ValueError, match="part off the 2316 mm front"):
        machine(scene_(), front_drive="air", front_door=(1.30, 0.954, 0.816))
    # The same codes for two doors is one door too many.
    scene, vf2, _ = front_cell()
    with pytest.raises(ValueError, match="M80/M81 are the side door's codes here"):
        bt.tending.haas_mfin(scene, vf2, "cell", parts=["part"], door_codes=(80, 81))
