"""`examples/machining/vf2_tending_demo.py` — a cobot tending a Haas VF-2
through its side window (N1 of the NC machine plan: the machine's axes
are a mechanism, and machining is a part program run on them; N2: the
program cuts the part, and the setup is something the cell checks; N3:
the part program runs the cell).

Four things the cell is for. The window: it lifts, less than the opening
is tall, so the arm goes in under the raised leaf's lower edge — and the
machine closes it again once the arm has answered its last call, which
must come after the arm is out. The program: one part program that keeps
running cuts the part, opens the vise and the window, calls the arm
through its relays and waits for the answers, and starts over; the
machine's side is read off its text, so the cycle is as long as the
control and the cell take, the vise and the clamped part ride the table
under the spindle, and the table stands where the arm was taught when the
arm is called. The part: the block is carved on the table by the tools as
they are, comes out as the design, and a setup stood wrong is named by
line and by volume. And the cell's own guard: without the cell-safe input
the program alarms at its `M180`, which the FAT sheet says.

The machine's pack is read from the published catalog, or from a local
catalog build named by `BOTRAIL_CATALOG_BUILD` — and so is its picture,
the reference model `haas/vf/vf-2-reference`. Where neither has that, the
cell is drawn as its envelopes; everything above is checked on the same
envelopes either way.
"""

import math
import re
import sys
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "machining"))

import vf2_tending_demo as demo


def _or_skip(build):
    try:
        return build()
    except Exception as err:
        if "catalog" in str(err).lower() or "fetch" in str(err).lower() or "resolve" in str(err).lower():
            pytest.skip(f"catalog unavailable: {err}")
        raise


@pytest.fixture(scope="module")
def cell():
    return _or_skip(demo.bake)


def test_the_part_program_runs_the_cell_and_starts_over(cell) -> None:
    scene, hs, tl = cell
    assert (hs.template, hs.door) == ("haas_mfin", "vf2/side_door") and tl.sequences == ["tend", "vf2"]
    # The machine's steps are the program's own: the cut by its lines,
    # every M code where it stands — and after the M99, the cut again, on
    # the blank the arm has just loaded.
    steps = [name.removeprefix("vf2/") for name, _, _ in tl.step_spans if name.startswith("vf2/")]
    assert [name for name in steps if not name.endswith("?")] == [
        "cycle start", "N25-N220",
        "N300 M71 vise unclamp", "N305 M180 door open",
        "N310 M21 arm: part out, blank in", "N310 M21 arm: part out, blank in — done",
        "N315 M70 vise clamp",
        "N320 M22 arm: let go and leave", "N320 M22 arm: let go and leave — done",
        "N325 M181 door closed",
        "cycle start", "N25-N220",
    ]
    assert [name for name, *_ in tl.moves("vf2/axes")] == ["op10#0", "op10#0"]
    assert scene.nc_machine("vf2/axes")["m_codes"] == {
        21: "arm: part out, blank in", 22: "arm: let go and leave", 70: "vise clamp", 71: "vise unclamp",
        180: "door open", 181: "door closed"}
    closed, opened = tl.signal(hs.signal("door_closed")), tl.signal(hs.signal("door_open"))
    call, leave, fin = (tl.signal(hs.signal(role)) for role in ("m21", "m22", "m_fin"))
    cut = tl.step_span("vf2/N25-N220")
    # Shut through the cut; lifted by the program's M180 — the cell safe
    # all along — and before the arm, which waits for the call, goes in.
    assert closed.value_at(0.0) and not opened.value_at(0.0)
    assert tl.signal(hs.signal("cell_safe")).high_spans() == [(0.0, tl.duration)]
    assert not tl.signal(hs.signal("alarm")).high_spans()
    assert cut.end < opened.rising_edges()[0] <= call.rising_edges()[0] <= tl.step_span("tend/enter").start
    assert tl.duration < 120.0
    # The leaf rises its 595 mm — up, not along the wall.
    inside = tl.step_span("tend/down").start
    (x0, y0, z0), _ = tl.object_pose("vf2/side_door/leaf", 0.0)
    (x1, y1, z1), _ = tl.object_pose("vf2/side_door/leaf", inside)
    assert (x1 - x0, y1 - y0, z1 - z0) == pytest.approx((0.0, 0.0, 0.595), abs=1e-3)
    # The vise opens before the window does, so the part is the arm's to
    # take. The first call is answered with the blank in the jaws and
    # still in the hand; the vise closes on it (M70), and the hand lets go
    # only on the second call — after the jaws' stroke.
    clamp = tl.signal(hs.signal("clamp"))
    assert clamp.falling_edges()[0] == pytest.approx(cut.end, abs=1e-9) and clamp.value_at(0.0)
    assert fin.rising_edges()[0] == pytest.approx(tl.step_span("tend/load").end, abs=0.011)
    assert call.falling_edges()[0] <= clamp.rising_edges()[-1]
    assert tl.step_span("tend/release").start >= clamp.rising_edges()[-1] + demo.CLAMP_S - 1e-6
    assert leave.rising_edges()[0] <= tl.step_span("tend/release").start
    # The second answer is given with the arm out, and is what lets the
    # window close; the next cut starts on the closed confirmation.
    out = fin.rising_edges()[1]
    assert out >= tl.step_span("tend/exit_2").end - 1e-6
    again = [span for span in tl.step_spans if span[0] == "vf2/N25-N220"][1]
    assert out < closed.rising_edges()[-1] <= again[1] and tl.signal(hs.signal("running")).value_at(tl.duration)
    # The part went to the stocker's out slot, the blank into the vise —
    # the arm's until it let go, and the vise's from the next tick.
    (bx, by, _), _ = scene.frame("stocker/out")
    p, _ = tl.object_pose("part", tl.duration)
    assert (p[0], p[1]) == pytest.approx((bx, by), abs=1e-3)
    (jx, jy, _), _ = scene.frame("vise/jaw")
    p, _ = tl.object_pose("blank", tl.duration)
    assert (p[0], p[1]) == pytest.approx((jx, jy), abs=1e-3)
    # The control's lanes: the spindle turns in the cut and not while the
    # window is open, and the two tools are in it in turn.
    spindle = tl.signal(hs.signal("spindle"))
    assert len(spindle.high_spans()) == 2 and spindle.high_spans()[-1][1] < cut.end
    tool = tl.word("vf2/axes.tool")
    assert (tool.value_at(10.0), tool.value_at(cut.end)) == (1, 2)


def test_machining_is_the_part_program_run_on_the_axes(cell) -> None:
    scene, hs, tl = cell
    run = bt.nc.check(scene, "vf2/axes", "op10")
    # The cycle time comes out of the program: two tools, five facing
    # passes, a pocket and four pecked holes — the control's share, which
    # is the dry run; what the cell takes over the M codes is the bake's.
    span = tl.step_span("vf2/N25-N220")
    assert span.end - span.start == pytest.approx(run.duration, abs=0.02) and 40.0 < run.duration < 90.0
    assert (run.tool_changes, run.tools(), run.warnings) == (2, [1, 2], ())
    assert [code for _, code in run.functions()] == [71, 180, demo.CALL, 70, demo.LEAVE, 181]
    assert tl.signal(hs.signal(f"m{demo.CALL}")).rising_edges()[0] > run.duration
    # G54 is where the part stands in the vise — 51 mm inside the X home
    # end, on the table's centreline, 125 mm over the table — derived, and
    # the program's last move is to the exchange position the cell was built at.
    control = scene.nc_machine("vf2/axes")
    assert control["work_offsets"]["G54"] == pytest.approx((-0.051, -0.2115, 0.125 - 0.610))
    assert set(control["tools"]) == {1, 2}
    assert run.end == pytest.approx((-0.762, -0.1605, 0.0))
    names = scene.robot_of("vf2/axes").joint_names
    for t in (0.0, span.end, tl.step_span("tend/exit_2").end):
        q = dict(zip(names, tl.sample(t, "vf2/axes")))
        assert (q["x"], q["y"], q["z"]) == pytest.approx(run.end, abs=1e-6)
    # While it cuts, the vise and the part ride the table under the
    # spindle: over the facing pass the nose is the tool's 110 mm less the
    # half millimetre of cut above the part's top.
    start, end = next((a, b) for a, b, line in run.spans if line == 18)
    mid = (start + end) / 2
    nose, _ = scene.link_pose_at("head", tl.sample(mid, "vf2/axes"), robot="vf2/axes")
    (px, py, pz), _ = tl.object_pose("part", mid)
    assert abs(px - nose[0]) < 0.028 and abs(py - nose[1]) < 0.017
    assert nose[2] - (pz + demo.PART[2] / 2) == pytest.approx(0.110 - 0.0005, abs=1e-6)
    (vx, vy, _), _ = tl.object_pose("vise/body", mid)
    (v0x, v0y, _), _ = tl.object_pose("vise/body", 0.0)
    (p0x, p0y, _), _ = tl.object_pose("part", 0.0)
    assert (vx - v0x, vy - v0y) == pytest.approx((px - p0x, py - p0y), abs=1e-9)
    # The machine's axes are not an arm: one line for the machine.
    names_on_bill = [name for row in scene.bom().rows for name in row["names"]]
    assert "vf2" in names_on_bill and not any(n.startswith("vf2/axes") for n in names_on_bill)


def test_the_arm_passes_under_the_raised_leaf_with_room(cell) -> None:
    scene, _hs, tl = cell
    # The envelope the cell is checked against: an enclosure open above
    # 1818 mm, a window whose raised leaf leaves 595 of its 749 mm clear.
    assert "vf2/shell/top" not in scene.obstacle_names
    assert scene.obstacle_bounds("vf2/shell/far")[1][2] == pytest.approx(1.818)
    sill = scene.obstacle_bounds("vf2/side_door/leaf")[0][2]
    assert (sill, sill + 0.595) == pytest.approx((0.819, 1.414))
    # Tightest by design: the blank in the jaws (2 mm a side) and a part
    # set down 5 mm proud. Everything else keeps its distance — the leaf
    # by more than 25 mm over the whole cycle, the wall's piers by more.
    assert float(tl.min_clearance()) > 0.0015
    assert float(tl.min_clearance(to=["vf2/side_door/leaf"])) > 0.025
    wall = [n for n in scene.obstacle_names if n.startswith("vf2/shell/")]
    assert float(tl.min_clearance(to=wall)) > 0.05


def test_the_model_is_the_picture_and_changes_nothing_that_is_checked(
    cell, monkeypatch: pytest.MonkeyPatch
) -> None:
    if demo.machine_shape() is None:
        pytest.skip(f"{demo.MACHINE_SHAPE} is neither built here (BOTRAIL_CATALOG_BUILD) nor published")
    scene, _, tl = cell
    pictures = [name for name in scene.obstacle_names if name.startswith("vf2/shape/")]

    def of(link: str) -> list[str]:
        return [name for name in pictures if re.sub(r"_\d+$", "", name) == f"vf2/shape/{link}"]

    # A picture a material of each link of the model — `body`, `body_1`, …
    # — the changer's shuttle and carousel among them, and the front doors,
    # which nothing drives here, standing shut.
    assert sorted({re.sub(r"_\d+$", "", name.removeprefix("vf2/shape/")) for name in pictures}) == [
        "body", "front_door_left", "front_door_right", "head", "saddle", "side_door_left", "side_door_right",
        "table", "tool_carousel", "tool_shuttle"]
    assert not any(scene.obstacle_enabled(name) for name in pictures)
    # What is seen of the machine is the model: its pictures, and the
    # window's leaf, which the first of the window's is bound to.
    assert "vf2/shape/side_door_right" not in pictures and len(of("side_door_right")) > 1
    seen = [name for name in scene.obstacle_names if name.startswith("vf2/") and scene.obstacle_visible(name)]
    assert sorted(seen) == sorted(["vf2/side_door/leaf", *pictures])

    # The table's picture goes where the vise bolted to the table goes,
    # the head's down with the spindle, through the program …
    def moved(name: str, t: float) -> tuple:
        (x0, y0, z0), _ = tl.object_pose(name, 0.0)
        (x1, y1, z1), _ = tl.object_pose(name, t)
        return (x1 - x0, y1 - y0, z1 - z0)

    drilling = 47.0
    names = scene.robot_of("vf2/axes").joint_names
    at = dict(zip(names, tl.sample(drilling, "vf2/axes")))
    park = dict(zip(names, tl.sample(0.0, "vf2/axes")))
    assert moved("vf2/shape/table", drilling) == pytest.approx(moved("vise/body", drilling), abs=1e-9)
    assert all(moved(name, drilling) == pytest.approx(moved("vf2/shape/table", drilling), abs=1e-9)
               for name in of("table"))
    # (the table goes the other way from its axes' commands)
    assert moved("vf2/shape/table", drilling)[:2] == pytest.approx(
        (-(at["x"] - park["x"]), -(at["y"] - park["y"])), abs=1e-6)
    assert moved("vf2/shape/saddle", drilling) == pytest.approx(
        (0.0, moved("vise/body", drilling)[1], 0.0), abs=1e-9)
    assert moved("vf2/shape/head", drilling) == pytest.approx((0.0, 0.0, at["z"]), abs=1e-6) and at["z"] < -0.3
    # … and the window's rises with the leaf, glass and handle with it. The
    # far window stands shut — where the near one's leaf stood, mirrored —
    # and nothing moves it.
    inside = tl.step_span("tend/down").start
    assert moved("vf2/side_door/leaf", inside)[2] == pytest.approx(0.595, abs=1e-3)
    assert all(moved(name, inside) == pytest.approx(moved("vf2/side_door/leaf", inside), abs=1e-9)
               for name in of("side_door_right"))
    (nx, ny, nz), _ = tl.object_pose("vf2/side_door/leaf", 0.0)
    assert scene.obstacle_pose("vf2/shape/side_door_left")[0] == pytest.approx((-nx, ny, nz), abs=1e-3)
    with pytest.raises(ValueError, match="not tracked"):
        tl.object_pose("vf2/shape/side_door_left", inside)

    # The model draws the umbrella tool changer, so the mechanism has it
    # and the control works it: twenty pockets, the carousel 522 mm from
    # the spindle on a shuttle that brings a pocket under it.
    changer = scene.nc_machine("vf2/axes")["changer"]
    assert (changer["shuttle"], changer["carousel"], changer["pockets"]) == ("tool_shuttle", "tool_carousel", 20)
    assert changer["reach"] == pytest.approx((0.0, 0.2295), abs=1e-7)
    (sx, sy, _), _ = scene.link_pose_at("head", tl.sample(0.0, "vf2/axes"), robot="vf2/axes")
    (cx, cy, cz), _ = scene.obstacle_pose("vf2/shape/tool_carousel")
    assert (sx - cx, sy - cy) == pytest.approx((0.522, 0.0), abs=1e-6)
    # The second tool change of the program — the end mill out, the drill
    # in — as the cell's clock has it: 4.2 s, the shuttle in for the
    # middle of it, the carousel a pocket round, every picture of both
    # with them; and the head at the top.
    start = tl.step_span("vf2/N25-N220").start
    first, second = [t for t, _, what, _ in demo.dry_run(scene).events if what == "tool"][:2]
    assert second - first > 4.2
    middle = start + second - 2.1
    at = dict(zip(names, tl.sample(middle, "vf2/axes")))
    assert (at["tool_shuttle"], at["tool_carousel"], at["z"]) == pytest.approx(
        (0.2295, -math.pi / 20, 0.0), abs=1e-7)
    after = dict(zip(names, tl.sample(start + second, "vf2/axes")))
    assert (after["tool_shuttle"], after["tool_carousel"]) == pytest.approx((0.0, -math.pi / 10), abs=1e-9)
    for name in of("tool_carousel") + of("tool_shuttle"):
        assert moved(name, middle)[0] == pytest.approx(0.2295, abs=1e-7), name
        assert moved(name, start + second) == pytest.approx(moved("vf2/shape/tool_carousel", start + second), abs=1e-9)
    assert tl.object_pose("vf2/shape/tool_carousel", start + second)[0] == pytest.approx((cx, cy, cz), abs=1e-9)

    # The same cell without the picture: the same cycle, to the tick, and
    # the same closest approach.
    monkeypatch.setattr(demo, "machine_shape", lambda catalog_root=None: None)
    plain, _, bare = demo.bake()
    assert not any(name.startswith("vf2/shape/") for name in plain.obstacle_names)
    assert [(name, round(a, 9), round(b, 9)) for name, a, b in bare.step_spans] == [
        (name, round(a, 9), round(b, 9)) for name, a, b in tl.step_spans]
    here, there = tl.min_clearance(), bare.min_clearance()
    assert (float(here), here.t) == pytest.approx((float(there), there.t), abs=1e-9)


def test_the_model_is_read_out_of_the_published_catalog_by_its_id() -> None:
    """`shape=` takes a catalog id. The package is then read as the
    catalog's: its files stand in the download cache as links, which a
    local package directory may not be — so this is a road of its own,
    and one a local build never takes."""
    def published() -> str:
        return bt.catalog_package(demo.MACHINE_SHAPE)

    cached = _or_skip(published)        # no catalog, or one from before the model: nothing to read
    # (From here on nothing is skipped: the model is there, and reading it
    # must not fail.)
    for shape in (demo.MACHINE_SHAPE, demo.MACHINE_SHAPE.rsplit("/", 1)[0], cached, Path(cached)):
        scene = bt.Scene(demo.tool(), name=demo.ROBOT, base_position=(3.0, 1.5, 0.0))
        vf2 = bt.parts.machine_tool(scene, "vf2", catalog=demo.MACHINE, exchange=demo.EXCHANGE,
                                    axes=True, shape=shape)
        pictures = [name for name in scene.obstacle_names if name.startswith("vf2/shape/")]
        assert {"vf2/shape/body", "vf2/shape/table", "vf2/shape/head", "vf2/shape/tool_carousel"} <= set(pictures)
        assert vf2.changer == ("tool_shuttle", "tool_carousel")
        assert scene.nc_machine(vf2.axes)["changer"]["pockets"] == 20


def test_the_block_is_cut_on_the_table_and_leaves_as_the_design() -> None:
    # Its own bake: the picture adds display objects to the scene.
    scene, _, tl = _or_skip(demo.bake)
    names = list(scene.obstacle_names)
    shown, cut = demo.cut(scene, tl)
    # Faced by half a millimetre and pocketed by the end mill, drilled
    # four times by the drill — and the result is the part as designed.
    faced = demo.PART[0] * demo.PART[1] * demo.FACE
    pocket = math.pi * (demo.POCKET[0] / 2) ** 2 * demo.POCKET[1]
    holes = 4 * math.pi * (demo.HOLES[0] / 2) ** 2 * demo.HOLES[1]
    assert cut.by_tool() == pytest.approx({1: faced + pocket, 2: holes}, rel=0.01)
    assert (cut.tolerance, cut.excess_volume, cut.gouge_volume) == (demo.TOLERANCE, 0.0, 0.0)
    assert cut.ok and not cut.issues, cut.summary()
    # The bake cut what the dry run of the same setup says, on the lines
    # that feed: nothing at rapid, nothing in the vise.
    dry = demo.dry_run(demo.build()[0]).machining
    assert dry.ok and cut.removed_volume == pytest.approx(dry.removed_volume, rel=1e-6)
    assert [line for _, line, _, _ in cut.lines] == [line for _, line, _, _ in dry.lines]
    span = tl.step_span("vf2/N25-N220")
    first, last = cut.cut_span
    assert span.start < first < last < span.end
    # The block is cut where it is: on the table under the spindle, and
    # the part leaves through the window in the gripper for the stocker.
    stages = [name for name in scene.obstacle_names if name.startswith("part_cut/")]
    assert stages[0] == "part_cut/raw" and len(stages) > 10
    assert not scene.obstacle_visible("part") and scene.obstacle_enabled("part")
    part = stages[-1]              # the block as the program leaves it
    carried = (tl.step_span("tend/exit").start + tl.step_span("tend/exit").end) / 2
    for t in (span.end + 0.1, carried, shown.duration):
        assert shown.object_visible(part, t)
        assert shown.object_pose(part, t)[0] == pytest.approx(tl.object_pose("part", t)[0], abs=1e-9)
    # (the block itself keeps its own track: what is gripped and checked)
    middle = (first + last) / 2
    assert shown.object_pose("part", middle) == tl.object_pose("part", middle) and not shown.object_visible(part, middle)
    (bx, by, _), _ = scene.frame("stocker/out")
    assert shown.object_pose(part, shown.duration)[0][:2] == pytest.approx((bx, by), abs=1e-3)
    assert shown.object_visible("part_cut/raw", first - 0.1) and not shown.object_visible("part_cut/raw", last)
    # The end mill, then the drill, stand in the spindle …
    assert [tool for tool, _, _ in cut.tools] == [1, 2]
    assert shown.object_visible("vf2/axes/tool/T1/cutter", first + 1.0)
    assert shown.object_visible("vf2/axes/tool/T2/cutter", last)

    def under_the_nose(tool: int, t: float) -> bool:
        (nx, ny, _), _ = scene.link_pose_at("head", shown.sample(t, "vf2/axes"), robot="vf2/axes")
        (x, y, _), _ = shown.object_pose(f"vf2/axes/tool/T{tool}/cutter", t)
        return math.hypot(x - nx, y - ny) < 1e-6

    assert under_the_nose(1, first + 1.0) and under_the_nose(2, last)
    # … and the one that is not: out of sight, or — where the machine is
    # drawn with its tool changer — in its pocket on the carousel, a
    # pocket circle's radius off the carousel's axis.
    if scene.nc_machine("vf2/axes")["changer"] is None:
        assert not shown.object_visible("vf2/axes/tool/T1/cutter", last)
    else:
        assert shown.object_visible("vf2/axes/tool/T1/cutter", last) and not under_the_nose(1, last)
        (cx, cy, _), _ = shown.object_pose("vf2/shape/tool_carousel", last)
        for tool, t in ((1, last), (2, first + 1.0)):
            (x, y, _), _ = shown.object_pose(f"vf2/axes/tool/T{tool}/cutter", t)
            assert math.hypot(x - cx, y - cy) == pytest.approx(0.2925, abs=1e-6)
    # All of it is display: the cycle and its clearances are the cell's.
    added = [name for name in scene.obstacle_names if name not in names]
    assert added and not any(scene.obstacle_enabled(name) for name in added)
    assert shown.duration == tl.duration
    assert float(shown.min_clearance()) == pytest.approx(float(tl.min_clearance()), abs=1e-12)


def test_a_setup_stood_wrong_is_named_by_line_and_by_volume() -> None:
    """The dry run — the program on the machine alone — is where a setup
    is proven, before the cell moves."""
    # The work zero entered 2 mm above the block. Nothing crashes: the
    # face is cut in the air and everything else comes up 2 mm short, so
    # the first part has material left all over its top.
    scene, _ = _or_skip(lambda: demo.build(setup="g54_high"))
    cut = demo.dry_run(scene).machining
    assert not cut.issues and not cut.ok
    assert cut.gouge_volume == 0.0 and 1.2e-6 < cut.excess_volume < 1.4e-6
    low, high = cut.excess_bounds
    assert high[2] == pytest.approx(demo.PART[2] / 2) and low[2] > 0.020
    assert "mm³ left over, 0.0 mm³ missing" in cut.summary()

    # The drill entered 20 mm shorter than it is: "15 mm over the part"
    # is 5 mm into it, at rapid, on the line that calls its length.
    scene, _ = demo.build(setup="t2_short")
    run = demo.dry_run(scene)
    cut = run.machining
    line = 1 + scene.nc_program("op10").splitlines().index("N165 G43 Z15. H2")
    crash = cut.issues[0]
    assert (crash.kind, crash.line, crash.tool, crash.what) == ("rapid_into_stock", line, 2, "part")
    assert crash.volume == pytest.approx(math.pi * 0.0025 ** 2 * (0.020 - 0.015 - demo.FACE), rel=0.02)
    assert all(issue.tool == 2 and issue.kind == "rapid_into_stock" for issue in cut.issues)
    assert cut.gouge_volume > 3e-6 and cut.excess_volume == 0.0
    # The end mill's work is untouched, and the sheet says which entry is off.
    good = demo.dry_run(demo.build()[0]).machining
    assert cut.by_tool()[1] == pytest.approx(good.by_tool()[1], rel=1e-9)
    sheet = bt.nc.setup_sheet(scene, "vf2/axes", "op10", stock="part", fixtures=demo.fixtures(scene))
    assert "| T2 | 5 mm drill | 45 | 40 x 75 | 135.000 | 115.000 | **entry -20.000 mm from the tool** |" in sheet
    assert f"- **op10 line {line}: the cutter meets `part` at rapid" in sheet
    assert "- Machine: Haas Automation VF-2 (`vf2/axes`)" in sheet
    assert "| `vise/jaw_fixed` | botrail VQ-125 |" in sheet
    # The vise bolted 30 mm nearer the window: the far side of the part
    # is past the X home, and the program is refused as the cell is
    # built, at the line that would go there.
    line = 1 + demo.PROGRAM.splitlines().index("N65 X27. F900.")
    with pytest.raises(ValueError, match=rf"line {line}: X 6.000 mm is past the X travel"):
        demo.build(setup="vise_out")
    with pytest.raises(ValueError, match="setup is one of"):
        demo.build(setup="upside_down")


def test_an_answer_given_with_the_arm_inside_is_refused_at_the_window() -> None:
    scene, hs = _or_skip(demo.build)
    S = bt.seq
    call, leave, fin = hs.signal(f"m{demo.CALL}"), hs.signal(f"m{demo.LEAVE}"), hs.signal("m_fin")
    # The same cell, a careless program: both calls are answered while
    # the arm is still over the vise.
    sq = scene.sequence("careless")
    sq.step("approach", actions=[S.set_signal(hs.signal("cell_safe")), S.motion("approach")])
    sq.step("wait_call", transition=S.signal(call))
    sq.step("enter", actions=[S.motion("enter")])
    sq.step("loaded", actions=[S.set_signal(fin)], transition=S.signal(call, False))
    sq.step("wait_clamp", actions=[S.set_signal(fin, False)], transition=S.signal(leave))
    sq.step("out", actions=[S.set_signal(fin)], transition=S.signal(leave, False))
    sq.step("clear", actions=[S.set_signal(fin, False)], transition=S.signal(hs.signal("door_closed")))
    # The program's M181 brings the leaf down on the arm, and the bake
    # stops there by name.
    with pytest.raises(ValueError, match="`vf2/side_door` moves `vf2/side_door/leaf` into robot `arm`"):
        scene.simulate_sequences(["careless", hs.program], max_duration=120.0)


def test_a_cell_that_is_not_safe_is_on_the_fat_sheet_with_the_documents(cell, tmp_path: Path) -> None:
    scene, hs, tl = cell
    sheet = tmp_path / "setup.md"
    sheet.write_text(bt.nc.setup_sheet(scene, "vf2/axes", "op10", stock="part", fixtures=demo.fixtures(scene)))
    report, runs = demo.deliver(scene, hs, tl, tmp_path / "set", sheet)
    # The broken wire, as the matrix has it: the program alarms at its
    # M180 and the arm waits for a call that never comes.
    assert runs.errors.get("baseline") is None
    refusal = runs.errors["no_cell_safe"]
    assert f"vf2/{demo.NOT_SAFE}" in refusal and "tend/wait_call" in refusal and "vf2/cell_safe=false" in refusal
    # … and as the FAT has it, with the cycle and the rows read off the
    # interlock table — none of them failed.
    fat = next(s for s in report.sections if s["title"] == "VF-2 tending FAT")["json"]
    verdicts = {row["name"]: row["verdict"] for row in fat["cases"]}
    assert (verdicts["nominal cycle"], verdicts["cell not safe"]) == ("pass", "pass")
    assert not [name for name, verdict in verdicts.items() if verdict in ("fail", "error")]
    door = next(name for name in verdicts if name.startswith("IL-") and "N305 M180 door open: vf2/side_door" in name)
    assert verdicts[door] == "pass"
    # The documents are the program's: the interlock table carries the
    # M180's guard, the handshake the relays and who answers them.
    files = {path.name for path in (tmp_path / "set").iterdir()}
    assert {"vf2_tending.botrail", "vf2_tending_op10.nc", "vf2_tending_bom.csv", "vf2_tending_io.csv",
            "vf2_tending_handshake.md", "vf2_tending.plcopen.xml", "vf2_tending_interlocks.md",
            "vf2_tending_layout.svg", "vf2_tending_fat.md", "vf2_tending_report.md"} <= files
    table = (tmp_path / "set" / "vf2_tending_interlocks.md").read_text()
    assert ("| N305 M180 door open | device `vf2/side_door → open` | "
            "`(vf2/cell_safe AND vf2/front_door/closed)` |") in table
    handshake = (tmp_path / "set" / "vf2_tending_handshake.md").read_text()
    assert "| `vf2/m21` | signal | vf2/cnc | <arm> |" in handshake
    assert "| `vf2/m_fin` | signal | <arm> | vf2/cnc |" in handshake
    assert "- Machine functions, in order (the cell's time, not in the cycle above): M71 vise unclamp" in sheet.read_text()


def test_the_vf2s_front_door_can_be_its_auto_door() -> None:
    def stand():
        scene = bt.Scene(demo.tool(), name=demo.ROBOT, base_position=(3.0, 1.5, 0.0))
        vf2 = bt.parts.machine_tool(scene, "vf2", catalog=demo.machine_pack(), exchange=demo.EXCHANGE,
                                    axes=True, shape=demo.machine_shape(), front_drive="air")
        return scene, vf2

    scene, vf2 = _or_skip(stand)
    # The drawing's 939 mm front opening, parted in the middle.
    assert vf2.front_doors == ("vf2/front_door/left", "vf2/front_door/right")
    assert vf2.front_door_travel == pytest.approx(0.939 / 2)
    (tx, ty, tz), _ = scene.frame("vf2/table")
    scene.add_box("part", size=demo.PART, position=(tx, ty, tz + demo.PART[2] / 2))
    # A program that only works the doors moves no axis, and is a program.
    bt.nc.program(scene, "doors", "M80\nG4 P1.\nM81\nM30\n")
    hs = bt.tending.haas_mfin(scene, vf2, "doors", parts=["part"])
    # The pack names the codes: the auto door's for the front door, the
    # auto window's for the side one.
    codes = scene.nc_machine(vf2.axes)["m_codes"]
    assert (codes[80], codes[81], codes[180], codes[181]) == (
        "front door open", "front door closed", "side door open", "side door closed")
    S = bt.seq
    sq = scene.sequence("cell")
    sq.step("safe", actions=[S.set_signal(hs.signal("cell_safe"))], transition=S.signal(vf2.front_open[0]))
    sq.step("look", transition=S.all_of(*(S.signal(lane) for lane in vf2.front_closed)))
    tl = scene.simulate_sequences(["cell", hs.program], max_duration=60.0)
    opened = tl.signal(vf2.front_open[1]).rising_edges()[0]
    for leaf, sign in (("vf2/front_door/leaf_left", -1.0), ("vf2/front_door/leaf_right", 1.0)):
        shut, apart = tl.object_pose(leaf, 0.0)[0], tl.object_pose(leaf, opened)[0]
        assert [b - a for a, b in zip(shut, apart)] == pytest.approx([sign * 0.4695, 0.0, 0.0], abs=1e-9)
    steps = [name.removeprefix("vf2/") for name, _, _ in tl.step_spans if name.startswith("vf2/")]
    assert "L1 M80 front door open" in steps and "L3 M81 front door closed" in steps
    # Where the model is at hand, each leaf is drawn as its door — the
    # first of the door's pictures is the leaf itself, the glass and the
    # handle ride it apart — and the envelope's own wall is unseen.
    if demo.machine_shape() is not None:
        assert scene.obstacle_visible("vf2/front_door/leaf_left") and scene.obstacle_visible("vf2/front_door/leaf_right")
        assert not scene.obstacle_visible("vf2/shell/front_sill")
        for tag, sign in (("left", -1.0), ("right", 1.0)):
            rides = [name for name in scene.obstacle_names if name.startswith(f"vf2/shape/front_door_{tag}_")]
            assert rides and f"vf2/shape/front_door_{tag}" not in scene.obstacle_names
            for name in rides:
                shut, apart = tl.object_pose(name, 0.0)[0], tl.object_pose(name, opened)[0]
                assert [b - a for a, b in zip(shut, apart)] == pytest.approx([sign * 0.4695, 0.0, 0.0], abs=1e-9)
