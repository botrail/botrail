"""`bt.parts` — standard structures generated from parameters (D2 of
design/design-cell-engineering.md).

A generator is only a composition of the ordinary scene API: boxes under a
name prefix, a frame where the next thing mounts, a device or a sensor
where one belongs, and the parts on the groups with their counts. These
tests pin what each generator makes, that the counts on the BOM follow the
parameters, that the layout sheet labels the assembly once, and that a
`Built` takes itself down cleanly.
"""

import json
import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


def scene_() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def rows(scene) -> dict:
    return {row["names"][0]: row for row in scene.bom().rows}


def test_fence_counts_panels_and_posts_and_follows_the_pitch() -> None:
    scene = scene_()
    built = bt.parts.fence(
        scene, "fence", path=[(-2, -2), (2, -2), (2, 2), (-2, 2)],
        height=2.0, panel_pitch=1.0, door=(0, 2), door_model="D1", model="ST20", manufacturer="TROAX", mass_kg=12,
    )
    # 4 corners + 3 intermediate posts per 4 m edge; 4 panels per edge, one
    # of them the door.
    assert built.name == "fence" and len(built.obstacles) == 4 + 12 + 16
    by = rows(scene)
    fence, posts, door = by["fence"], by["fence/posts"], by["fence/door"]
    assert (fence["category"], fence["qty"], fence["model"], fence["manufacturer"]) == ("structure.fence", 15, "ST20", "TROAX")
    assert (posts["category"], posts["qty"]) == ("structure.fence.post", 16)
    assert (door["category"], door["model"]) == ("structure.door", "D1")
    assert scene.bom().total("mass_kg") == pytest.approx(15 * 12)
    # Every panel is a real box: a 4 m edge at 1 m pitch → panels 0.94 m
    # wide (the pitch minus a post), 2 m tall, standing on the floor.
    lo, hi = scene.obstacle_bounds("fence/panels/e0_0")
    assert hi[0] - lo[0] == pytest.approx(0.94) and hi[2] == pytest.approx(2.0) and lo[2] == pytest.approx(0.0)
    # The east edge runs along y: its panels are turned.
    lo, hi = scene.obstacle_bounds("fence/panels/e1_0")
    assert hi[1] - lo[1] == pytest.approx(0.94, abs=1e-6)
    # Half the pitch → twice the panels and more posts; the sheet changes.
    sheet_1m = scene.layout("json")
    built.remove(scene)
    assert scene.obstacle_names == [] and scene.parts() == []
    bt.parts.fence(scene, "fence", path=[(-2, -2), (2, -2), (2, 2), (-2, 2)], panel_pitch=0.5, model="ST20")
    by = rows(scene)
    assert by["fence"]["qty"] == 32 and by["fence/posts"]["qty"] == 4 + 7 * 4
    assert scene.layout("json") != sheet_1m
    # An open run of two corners is one edge; a degenerate path is refused.
    scene2 = scene_()
    bt.parts.fence(scene2, "run", path=[(0, 0), (3, 0)], closed=False, panel_pitch=1.0)
    assert rows(scene2)["run"]["qty"] == 3 and rows(scene2)["run/posts"]["qty"] == 4
    with pytest.raises(ValueError, match="at least two corners"):
        bt.parts.fence(scene2, "bad", path=[(0, 0)])
    with pytest.raises(ValueError, match="positive"):
        bt.parts.fence(scene2, "bad", path=[(0, 0), (1, 0)], panel_pitch=0)


def test_fence_labels_once_on_the_sheet() -> None:
    scene = scene_()
    bt.parts.fence(scene, "fence", path=[(-2, -2), (2, -2), (2, 2), (-2, 2)], model="ST20", door=(0, 1))
    labels = [
        i["shape"]["text"] for i in json.loads(scene.layout("json"))["items"] if i["shape"]["shape"] == "text"
    ]
    assert labels.count("fence (ST20)") == 1
    assert not any(l.startswith("posts") or l.startswith("door") for l in labels), labels


def test_table_pedestal_and_pallet_stand_on_the_floor_and_offer_a_frame() -> None:
    scene = scene_()
    t = bt.parts.table(scene, "table", size=(1.2, 0.8, 0.75), position=(1.0, 0.0), model="HFS8", mass_kg=20)
    assert t.frames == ["table/top"] and len(t.obstacles) == 5
    pos, _ = scene.frame("table/top")
    assert pos == pytest.approx((1.0, 0.0, 0.75))
    lo, hi = scene.obstacle_bounds("table/top")
    assert hi[2] == pytest.approx(0.75) and (hi[0] - lo[0], hi[1] - lo[1]) == pytest.approx((1.2, 0.8))
    lo, hi = scene.obstacle_bounds("table/leg0")
    assert lo[2] == pytest.approx(0.0)

    p = bt.parts.pedestal(scene, "ped", height=0.5, position=(0.0, 0.0), model="PD-500")
    assert p.frames == ["ped/mount"]
    pos, _ = scene.frame("ped/mount")
    assert pos == pytest.approx((0.0, 0.0, 0.5))
    lo, hi = scene.obstacle_bounds("ped/top")
    assert hi[2] == pytest.approx(0.5)
    lo, hi = scene.obstacle_bounds("ped/base")
    assert lo[2] == pytest.approx(0.0)
    # The robot mounts on it: the frame is the base pose.
    scene.set_robot_base_pose(*scene.frame("ped/mount"))
    assert scene.robot_base_pose[0] == pytest.approx((0.0, 0.0, 0.5))

    pl = bt.parts.pallet(scene, "pallet", position=(-1.5, 0.0), yaw=math.pi / 2)
    assert pl.frames == ["pallet/top"]
    pos, _ = scene.frame("pallet/top")
    assert pos == pytest.approx((-1.5, 0.0, 0.144))
    # Turned a quarter: the long side now runs along y.
    lo, hi = scene.obstacle_bounds("pallet/deck0")
    assert hi[1] - lo[1] == pytest.approx(1.2)

    by = rows(scene)
    assert (by["table"]["category"], by["table"]["qty"], by["table"]["model"]) == ("structure.table", 1, "HFS8")
    assert (by["ped"]["category"], by["ped"]["model"]) == ("structure.pedestal", "PD-500")
    assert (by["pallet"]["category"], by["pallet"]["model"]) == ("pallet", "EPAL 1")
    assert scene.bom().total("mass_kg") == 20


def test_conveyor_makes_a_body_a_device_and_end_frames() -> None:
    scene = scene_()
    c = bt.parts.conveyor(
        scene, "conv", length=2.0, width=0.4, position=(0.0, 1.0, 0.7), direction=(0, 1), speed=0.25,
        model="GVL-2000", manufacturer="MISUMI",
    )
    assert c.devices == ["conv"] and c.frames == ["conv/infeed", "conv/outfeed"]
    assert "conv" in scene.device_names
    # Along +y: the infeed is at y = 0, the outfeed at y = 2, on the belt.
    assert scene.frame("conv/infeed")[0] == pytest.approx((0.0, 0.0, 0.7))
    assert scene.frame("conv/outfeed")[0] == pytest.approx((0.0, 2.0, 0.7))
    lo, hi = scene.obstacle_bounds("conv/belt")
    assert hi[2] == pytest.approx(0.7) and hi[1] - lo[1] == pytest.approx(2.0) and hi[0] - lo[0] == pytest.approx(0.4)
    assert any(n.startswith("conv/leg_") for n in c.obstacles)
    # The identity sits on the device: one BOM line, not a body line.
    by = rows(scene)
    assert (by["conv"]["category"], by["conv"]["model"], by["conv"]["manufacturer"]) == ("conveyor", "GVL-2000", "MISUMI")
    assert not any(name.startswith("conv/") for name in by)
    # The zone rides the belt and carries a part placed on it.
    scene.add_box("crate", size=(0.1, 0.1, 0.1), position=(0.0, 0.2, 0.75))
    sq = scene.sequence("run")
    sq.step("go", actions=[bt.seq.start("conv")], transition=bt.seq.elapsed(2.0))
    tl = scene.simulate_sequence("run")
    assert tl.object_pose("crate", 2.0)[0][1] == pytest.approx(0.2 + 0.25 * 2.0, abs=0.02)
    # The sheet labels the conveyor once, by the device.
    labels = [
        i["shape"]["text"] for i in json.loads(scene.layout("json"))["items"] if i["shape"]["shape"] == "text"
    ]
    assert labels.count("conv") == 1
    with pytest.raises(ValueError, match="non-zero"):
        bt.parts.conveyor(scene, "bad", length=1.0, width=0.3, position=(0, 0, 0.7), direction=(0, 0))


def test_light_curtain_is_a_beam_between_two_columns() -> None:
    scene = scene_()
    lc = bt.parts.light_curtain(scene, "lc", frm=(-1.0, -1.0), to=(1.0, -1.0), height=1.2, model="SL-V", manufacturer="KEYENCE")
    assert lc.sensors == ["lc"] and lc.obstacles == ["lc/column_a", "lc/column_b"]
    assert "lc" in scene.sensor_names
    lo, hi = scene.obstacle_bounds("lc/column_a")
    assert hi[2] == pytest.approx(1.2)
    by = rows(scene)
    assert (by["lc"]["category"], by["lc"]["model"]) == ("sensor.light_curtain", "SL-V")
    # It trips on the robot: the sequence sees the input.
    sq = scene.sequence("watch")
    sq.step("wait", transition=bt.seq.elapsed(0.1))
    tl = scene.simulate_sequence("watch")
    assert "lc" in [name for name, _ in tl.signals]
    lc.remove(scene)
    assert scene.sensor_names == [] and scene.obstacle_names == []


def test_generated_structures_round_trip_through_the_project(tmp_path: Path) -> None:
    scene = scene_()
    bt.parts.fence(scene, "fence", path=[(-1, -1), (1, -1), (1, 1), (-1, 1)], model="ST20")
    bt.parts.pedestal(scene, "ped", height=0.4, position=(0, 0))
    bt.parts.conveyor(scene, "conv", length=1.5, width=0.3, position=(0, 0.8, 0.6), model="GVL")
    # A machine tool brings a device, sensors with turned zones, and a
    # panel of pinned buttons — the whole of it comes back.
    vmc = bt.parts.machine_tool(scene, "vmc", position=(4.0, 0.0), yaw=0.3, model="VMC")
    bt.tending.fanuc_ri2(scene, vmc, cycle_s=1.0)
    scene.save_project(tmp_path / "cell.botrail")
    again = bt.Scene.load_project(tmp_path / "cell.botrail")
    assert again.bom().rows == scene.bom().rows
    assert again.device_names == scene.device_names and again.sensor_names == scene.sensor_names
    assert again.sequence_names == scene.sequence_names
    # Poses round-trip through JSON with float noise, so compare the
    # rendered sheet (rounded to a tenth of a pixel) and the extents.
    assert again.layout("svg") == scene.layout("svg")
    assert again.footprint() == pytest.approx(scene.footprint())
    assert set(again.frames) == set(scene.frames)


def test_wall_leaves_a_doorway_and_spans_it() -> None:
    """A partition is piers and heads: what is left of the run once the
    openings are taken out of it, plus the wall over each one. The pier
    still collides, so a machine driven at it fails; the opening is the
    only way through, and it comes with the frame a route is authored on."""
    scene = scene_()
    built = bt.parts.wall(
        scene, "corridor", path=[(0.0, 2.4), (9.0, 2.4)], height=2.7,
        thickness=0.12, openings=[(0, 3.0, 0.9), (0, 6.0, 1.2, 2.7)],
        model="PT-2700",
    )
    piers = sorted(n for n in built.obstacles if n.startswith("corridor/e0_"))
    assert len(piers) == 3          # 0..2.55, 3.45..5.4, 6.6..9.0
    lo, hi = scene.obstacle_bounds("corridor/e0_1")
    assert (lo[0], hi[0]) == pytest.approx((3.45, 5.40))
    assert hi[2] == pytest.approx(2.7)
    # A doorway shorter than the wall is spanned; one as tall as it is a gap.
    heads = sorted(n for n in built.obstacles if "/head/" in n)
    assert heads == ["corridor/head/e0_0"]
    lo, hi = scene.obstacle_bounds(heads[0])
    assert (lo[2], hi[2]) == pytest.approx((2.1, 2.7))
    assert built.frames == ["corridor/opening0_0", "corridor/opening0_1"]
    assert scene.frame("corridor/opening0_1")[0] == pytest.approx((6.0, 2.4, 0.0))
    row = rows(scene)["corridor"]
    assert (row["category"], row["model"], row["qty"]) == ("structure.wall", "PT-2700", 1)
    assert row["attributes"]["length_mm"] == "9000"
    built.remove(scene)
    assert scene.obstacle_names == [] and scene.frames == {}


def test_wall_closes_a_room_and_refuses_a_plan_that_does_not() -> None:
    """Four corners closed is a room, and each shared corner gets a column
    so two runs meet square. An opening that runs off its wall — or into
    the one beside it — is a floor plan that does not close, and is
    refused rather than quietly clipped."""
    scene = scene_()
    built = bt.parts.wall(scene, "room", path=[(0, 0), (4, 0), (4, 3), (0, 3)],
                          closed=True, height=2.5, openings=[(0, 2.0, 0.9)])
    assert sum("/corner" in n for n in built.obstacles) == 4
    piers = [n for n in built.obstacles if n.startswith("room/e")]
    assert len(piers) == 5  # two piers on the edge with the door, three whole runs
    assert sum("/head/" in n for n in built.obstacles) == 1
    with pytest.raises(ValueError, match="runs off edge 0"):
        bt.parts.wall(scene, "bad", path=[(0, 0), (2, 0)], openings=[(0, 1.8, 0.9)])
    with pytest.raises(ValueError, match="overlap"):
        bt.parts.wall(scene, "bad", path=[(0, 0), (4, 0)],
                      openings=[(0, 1.0, 1.0), (0, 1.8, 1.0)])


def test_rack_stacks_shelves_and_puts_a_frame_on_each() -> None:
    """A bay of shelves on four uprights: `levels` boards, evenly spaced with
    the top one at the bay height, and a frame on each deck where the parts
    sit — the target a pick aims at."""
    scene = scene_()
    built = bt.parts.rack(scene, "rack", (1.2, 0.6, 1.8), (1.0, 0.5), levels=3, model="MS-1260")
    assert [round(scene.frame(f)[0][2], 3) for f in built.frames] == [0.6, 1.2, 1.8]
    assert sum("/uprights/" in n for n in built.obstacles) == 4
    lo, hi = scene.obstacle_bounds("rack/shelves/l2")
    assert hi[2] == pytest.approx(1.8) and hi[0] - lo[0] == pytest.approx(1.2 - 2 * 0.04)
    assert len(built.frames) == 3
    row = rows(scene)["rack"]
    assert (row["category"], row["model"], row["qty"]) == ("structure.rack", "MS-1260", 1)
    # levels is what the rack is, not a quantity — no totals column for it.
    assert row["attributes"]["levels"] == "3"
    assert scene.bom().total("mass_kg") is None
    built.remove(scene)
    assert scene.obstacle_names == [] and scene.frames == {} and scene.parts() == []


# ------------------------------------------------------------ machine tool


def test_machine_tool_stands_the_envelopes_a_tending_cell_verifies_against() -> None:
    scene = scene_()
    scene.set_robot_base_pose((4.0, 0.0, 0.0))
    vmc = bt.parts.machine_tool(scene, "vmc", model="α-D21MiB5 Plus", manufacturer="FANUC", mass_kg=2000)
    # The ROBODRILL figures, as boxes: the side opening 705 wide from the
    # 827 sill, the piers around it, the table top at 0.90 shifted 250 mm
    # to the door, the spindle head from nose-to-table 580 up to the roof.
    lo, hi = scene.obstacle_bounds("vmc/shell/near_front")
    assert hi[0] - lo[0] == pytest.approx(0.06) and (lo[2], hi[2]) == pytest.approx((0.827, 1.696))
    lo, hi = scene.obstacle_bounds("vmc/shell/near_rear")
    front = scene.obstacle_bounds("vmc/shell/near_front")[1][1]
    assert lo[1] - front == pytest.approx(0.705)
    assert scene.obstacle_bounds("vmc/shell/near_head")[0][2] == pytest.approx(1.696)
    lo, hi = scene.obstacle_bounds("vmc/table")
    assert (hi[2], hi[0] - lo[0], hi[1] - lo[1]) == pytest.approx((0.90, 0.65, 0.40))
    assert (lo[0] + hi[0]) / 2 == pytest.approx(0.25)
    assert scene.obstacle_bounds("vmc/head")[0][2] == pytest.approx(0.90 + 0.58)
    (tx, _ty, tz), _ = scene.frame("vmc/table")
    assert (tx, tz) == pytest.approx((0.25, 0.90))
    # The leaf stands proud of the wall, over the opening; the handle
    # frame aims +Z into it; `entry` waits 150 mm outside the leaf.
    lo, hi = scene.obstacle_bounds("vmc/side_door/leaf")
    assert lo[0] > scene.obstacle_bounds("vmc/shell/near_sill")[1][0]
    assert (hi[1] - lo[1], hi[2] - lo[2]) == pytest.approx((0.805, 0.969))
    (hx, _, _), hq = scene.frame("vmc/door/side/handle")
    assert hx > hi[0]
    assert bt.parts._rotate(hq, (0.0, 0.0, 1.0)) == pytest.approx((-1.0, 0.0, 0.0))
    assert scene.frame("vmc/entry")[0][0] == pytest.approx(hi[0] + 0.15)
    # A servo door: a linear axis over the leaf and its trim, 760 mm of
    # stroke at the published 0.8 m / 0.8 s, with `closed` and `open` as
    # named stops — their lanes are the limit switches, not sensors.
    assert vmc.door == "vmc/side_door" and vmc.door in scene.device_names
    assert vmc.door_travel == pytest.approx(0.76) and vmc.door_axis == pytest.approx((0.0, 1.0, 0.0))
    assert vmc.door_objects[0] == "vmc/side_door/leaf" and "vmc/side_door/handle" in vmc.door_objects
    assert vmc.door_lanes == ("vmc/side_door/closed", "vmc/side_door/open")
    assert vmc.panel is not None and vmc.buttons == [f"vmc/panel/{b}" for b in ("cycle_start", "feed_hold", "reset", "estop")]
    # The front door's closed switch and the E-stop: the lanes the
    # machine's program is guarded by.
    assert vmc.front_door_lane == "vmc/front_door/closed" and vmc.estop == "vmc/panel/estop"
    assert vmc.sensors == [vmc.front_door_lane, *vmc.buttons]
    assert rows(scene)["vmc/front_door/closed"]["category"] == "sensor.limit_switch"
    sq = scene.sequence("door")
    sq.step("open", actions=[bt.seq.move_to(vmc.door, "open")], transition=bt.seq.device_done(vmc.door))
    sq.step("hold", transition=bt.seq.elapsed(0.1))
    tl = scene.simulate_sequence("door")
    assert tl.signal("vmc/side_door/closed").value_at(0.0) and not tl.signal("vmc/side_door/open").value_at(0.0)
    assert tl.signal("vmc/side_door/open").value_at(tl.duration) and not tl.signal("vmc/side_door/closed").value_at(tl.duration)
    assert tl.signal("vmc/side_door/open").rising_edges()[0] == pytest.approx(0.76, abs=0.02)
    assert tl.object_pose("vmc/side_door/handle", tl.duration)[0][1] - scene.frame("vmc/door/side/handle")[0][1] == pytest.approx(0.76)
    # The bill: the machine, the door as the axis it is driven by, the
    # panel, its buttons with the 22 mm figures, the limit switches.
    by = rows(scene)
    assert (by["vmc"]["category"], by["vmc"]["model"], by["vmc"]["attributes"]["table_mm"]) == ("machine_tool.vmc", "α-D21MiB5 Plus", "650x400")
    door = by["vmc/side_door"]
    assert (door["category"], door["attributes"]["drive"], door["attributes"]["stroke_mm"], door["attributes"]["open_s"]) == ("machine_tool.door", "servo", 760, 0.76)
    assert by["vmc/panel"]["category"] == "hmi.panel"
    start, estop = by["vmc/panel/cycle_start"], by["vmc/panel/estop"]
    assert (start["category"], start["attributes"]["travel_mm"], start["attributes"]["force_n"]) == ("hmi.button", 2.6, 3.8)
    assert (estop["attributes"]["head_mm"], estop["attributes"]["force_n"], estop["attributes"]["actuator"]) == (40, 44, "mushroom")
    assert "vmc/side_door/closed" not in by   # the stops are the axis's, not articles
    vmc.remove(scene)
    assert scene.obstacle_names == [] and scene.device_names == [] and scene.sensor_names == []
    assert scene.frames == {} and scene.parts() == []


def test_machine_tool_variants_manual_door_and_no_door() -> None:
    scene = scene_()
    scene.set_robot_base_pose((4.0, 0.0, 0.0))
    # A manual door: the leaf is loose (no axis), so two zone sensors read
    # it at its ends — the limit switches, on the bill.
    vmc = bt.parts.machine_tool(scene, "vmc", door="manual", panel="door", buttons=("cycle_start", "clamp"))
    assert vmc.door is None and scene.device_names == [] and vmc.door_travel == pytest.approx(0.76)
    assert rows(scene)["vmc/side_door"]["attributes"]["drive"] == "manual"
    assert vmc.door_lanes == ("vmc/side_door/closed", "vmc/side_door/open")
    assert set(vmc.door_lanes) <= set(vmc.sensors)
    assert rows(scene)["vmc/side_door/closed"]["category"] == "sensor.limit_switch"
    # The panel beside the door faces the way the door does (+X).
    (_, _, _), q = scene.frame("vmc/panel/cycle_start/press")
    assert bt.parts._rotate(q, (0.0, 0.0, 1.0)) == pytest.approx((-1.0, 0.0, 0.0), abs=1e-9)
    for name in vmc.door_objects:
        (x, y, z), qq = scene.obstacle_pose(name)
        scene.set_obstacle_pose(name, (x, y + vmc.door_travel, z), qq)
    sq = scene.sequence("look")
    sq.step("hold", transition=bt.seq.elapsed(0.05))
    tl = scene.simulate_sequence("look")
    assert tl.signal("vmc/side_door/open").value_at(0.0) and not tl.signal("vmc/side_door/closed").value_at(0.0)
    # The front door stands shut: its switch reads made. No E-stop on this
    # panel, so no E-stop lane.
    assert tl.signal("vmc/front_door/closed").value_at(0.0) and vmc.estop is None
    vmc.remove(scene)
    # No side door at all: a solid wall, no lanes, no handle, and the left
    # side — the front door's switch is the only sensor left.
    vmc = bt.parts.machine_tool(scene, "vmc", door=None, panel=None)
    assert vmc.door_lanes is None and vmc.panel is None and "vmc/shell/near" in vmc.obstacles
    assert "vmc/entry" not in vmc.frames and scene.sensor_names == ["vmc/front_door/closed"]
    vmc.remove(scene)
    assert scene.sensor_names == []
    # Without a front door there is no switch either.
    vmc = bt.parts.machine_tool(scene, "vmc", door=None, panel=None, front_door=None)
    assert vmc.front_door_lane is None and scene.sensor_names == []
    vmc.remove(scene)
    vmc = bt.parts.machine_tool(scene, "vmc", door="air", door_side="left")
    _lo, hi = scene.obstacle_bounds("vmc/side_door/leaf")
    assert hi[0] < scene.obstacle_bounds("vmc/shell/far")[0][0]
    assert rows(scene)["vmc/side_door"]["attributes"]["open_s"] == pytest.approx(0.76 / 0.4)


def test_machine_tool_refuses_what_does_not_fit() -> None:
    scene = scene_()
    with pytest.raises(ValueError, match="does not fit"):
        bt.parts.machine_tool(scene, "vmc", aperture=(1.4, 0.869, 0.827))
    with pytest.raises(ValueError, match="through a"):
        bt.parts.machine_tool(scene, "vmc", aperture=(0.705, 1.5, 0.827))
    with pytest.raises(ValueError, match="off the"):
        bt.parts.machine_tool(scene, "vmc", door_travel=1.2)
    with pytest.raises(ValueError, match="into the enclosure"):
        bt.parts.machine_tool(scene, "vmc", exchange=(0.6, 0.0))
    with pytest.raises(ValueError, match="roof"):
        bt.parts.machine_tool(scene, "vmc", head_clearance=1.5)
    with pytest.raises(ValueError, match="panel='door'"):
        bt.parts.machine_tool(scene, "vmc", door=None, panel="door")
    with pytest.raises(ValueError, match="door must be"):
        bt.parts.machine_tool(scene, "vmc", door="hydraulic")
    assert scene.obstacle_names == []


def test_operator_panel_button_reads_a_press_and_only_that_button() -> None:
    scene = scene_()
    # Three buttons in a row facing the arm; the arm's tool tip is a
    # 30 x 80 x 40 box, 40 mm past its TCP.
    panel = bt.parts.operator_panel(scene, "hmi", (0.0, 0.42, 0.55), buttons=("a", "b", "c"), columns=3,
                                    model="XALK", manufacturer="ACME", button_model="XB4BA31")
    assert panel.sensors == ["hmi/a", "hmi/b", "hmi/c"]
    assert set(panel.frames) == {"hmi", "hmi/a", "hmi/a/press", "hmi/b", "hmi/b/press", "hmi/c", "hmi/c/press"}
    (cx, cy, cz), q = scene.frame("hmi/b")
    (px, py, pz), _ = scene.frame("hmi/b/press")
    # +Z into the panel; the press frame 2.6 mm deeper than the cap face.
    assert bt.parts._rotate(q, (0.0, 0.0, 1.0)) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9)
    assert (px - cx, py - cy, pz - cz) == pytest.approx((0.0, 0.0026, 0.0))
    by = rows(scene)
    assert (by["hmi"]["category"], by["hmi"]["model"]) == ("hmi.panel", "XALK")
    assert (by["hmi/a"]["category"], by["hmi/a"]["model"], by["hmi/a"]["qty"]) == ("hmi.button", "XB4BA31", 3)

    def held_at(depth: float) -> dict:
        target = (px, py - 0.04 - depth, pz)
        ik = scene.set_tcp_target(target, q)
        assert ik.converged, ik
        sq = scene.sequence("hold")
        sq.step("hold", transition=bt.seq.elapsed(0.05))
        tl = scene.simulate_sequence("hold")
        return {b: tl.signal(f"hmi/{b}").value_at(0.0) for b in "abc"}

    # Touching the cap reads nothing; pushed in to the press frame, `b` is
    # on and its neighbours are not.
    assert held_at(0.0026) == {"a": False, "b": False, "c": False}
    assert held_at(0.0) == {"a": False, "b": True, "c": False}
    panel.remove(scene)
    assert scene.sensor_names == [] and scene.obstacle_names == [] and scene.frames == {}


def test_vise_frames_the_jaw_floor_and_refuses_a_wide_opening() -> None:
    scene = scene_()
    vise = bt.parts.vise(scene, "vise", (1.0, 0.5, 0.9), opening=0.054, model="VQ-125", manufacturer="ACME")
    assert vise.obstacles == ["vise/body", "vise/jaw_fixed", "vise/jaw_moving"] and vise.frames == ["vise/jaw"]
    (x, y, z), _ = scene.frame("vise/jaw")
    assert (x, y, z) == pytest.approx((1.0, 0.5, 0.96))
    fixed, moving = scene.obstacle_bounds("vise/jaw_fixed"), scene.obstacle_bounds("vise/jaw_moving")
    assert fixed[0][1] - moving[1][1] == pytest.approx(0.054)
    assert fixed[0][2] == pytest.approx(0.96) and fixed[1][2] == pytest.approx(1.0)
    assert scene.obstacle_bounds("vise/body")[0][2] == pytest.approx(0.9)
    by = rows(scene)
    assert (by["vise"]["category"], by["vise"]["model"], by["vise"]["attributes"]["opening_mm"]) == ("fixture.vise", "VQ-125", 54)
    with pytest.raises(ValueError, match="opens 150 mm at most"):
        bt.parts.vise(scene, "wide", (0.0, 0.0), opening=0.2)
    vise.remove(scene)
    assert scene.obstacle_names == []


def test_lathe_and_chuck_stand_the_turning_envelopes() -> None:
    scene = scene_()
    scene.set_robot_base_pose((4.0, 0.0, 0.0))
    lathe = bt.parts.lathe(scene, "lathe", model="ST-10", manufacturer="Haas", mass_kg=3585)
    # The ST-10 figures as boxes: a 3.20 m body, the front opening 900 wide
    # from the 800 sill, the spindle nose 550 mm left of centre at 1.05 m,
    # half a metre behind the front wall.
    lo, hi = scene.obstacle_bounds("lathe/rear")
    assert hi[0] - lo[0] == pytest.approx(3.20) and hi[2] == pytest.approx(2.06)
    lo, hi = scene.obstacle_bounds("lathe/front_door/leaf")
    assert hi[0] - lo[0] == pytest.approx(0.90 + 0.10) and (lo[2], hi[2]) == pytest.approx((0.75, 1.55))
    (sp, sq), _ = scene.frame("lathe/spindle"), None
    assert sp == pytest.approx((-0.55, -0.89 + 0.06 + 0.50, 1.05), abs=1e-6)
    assert bt.parts._rotate(sq, (0.0, 0.0, 1.0)) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    # A manual front door: loose leaf, two limit switches, the handle at
    # the tailstock-side edge with +Z into the leaf, the entry in front.
    assert lathe.door is None and lathe.door_lanes == ("lathe/front_door/closed", "lathe/front_door/open")
    assert lathe.door_axis == pytest.approx((1.0, 0.0, 0.0)) and lathe.door_travel == pytest.approx(0.955)
    assert lathe.front_door_lane is None and lathe.estop == "lathe/panel/estop"
    (hp, hq) = scene.frame("lathe/door/front/handle")
    assert bt.parts._rotate(hq, (0.0, 0.0, 1.0)) == pytest.approx((0.0, 1.0, 0.0), abs=1e-9) and hp[1] < lo[1]
    (ep, _), _ = scene.frame("lathe/entry"), None
    assert ep[0] == pytest.approx(-0.55) and ep[1] < hp[1]
    # The chuck on the spindle: its body behind the face, three jaws proud
    # of it around a 50 mm part, its face frame +Z along the spindle.
    chuck = bt.parts.chuck(scene, "chuck", *scene.frame("lathe/spindle"), opening=0.050,
                           model="HO-6", manufacturer="Kitagawa", mass_kg=22)
    assert chuck.obstacles == ["chuck/body", "chuck/jaw0", "chuck/jaw1", "chuck/jaw2"] and chuck.frames == ["chuck/face"]
    lo, hi = scene.obstacle_bounds("chuck/body")
    assert hi[0] == pytest.approx(-0.55, abs=1e-6) and lo[0] == pytest.approx(-0.635, abs=1e-6)
    assert hi[2] - lo[2] == pytest.approx(0.165, abs=1e-6)
    for k in range(3):
        lo, hi = scene.obstacle_bounds(f"chuck/jaw{k}")
        assert lo[0] == pytest.approx(-0.55, abs=1e-6) and hi[0] == pytest.approx(-0.52, abs=1e-6)
        r = math.hypot((lo[1] + hi[1]) / 2 - sp[1], (lo[2] + hi[2]) / 2 - sp[2])
        assert r == pytest.approx(0.025 + 0.0125, abs=1e-6)
    (fp, fq) = scene.frame("chuck/face")
    assert fp == pytest.approx(sp) and bt.parts._rotate(fq, (0.0, 0.0, 1.0)) == pytest.approx((1.0, 0.0, 0.0), abs=1e-9)
    # The bill: the lathe, its door with the stroke, the chuck with its
    # diameter and opening; the machine program templates take it as is.
    by = rows(scene)
    assert by["lathe"]["category"] == "machine_tool.lathe" and by["lathe"]["model"] == "ST-10"
    assert by["lathe/front_door"]["attributes"]["drive"] == "manual"
    assert by["lathe/front_door"]["attributes"]["stroke_mm"] == pytest.approx(955.0)
    assert (by["chuck"]["category"], by["chuck"]["attributes"]["diameter_mm"], by["chuck"]["attributes"]["opening_mm"]) == (
        "fixture.chuck", 165.0, 50.0)
    hs = bt.tending.manual(scene, lathe, cycle_s=1.0, buttons=("cycle_start", "feed_hold", "reset"))
    assert hs.signal("door_closed") == "lathe/front_door/closed" and "front_door_closed" not in hs.signals
    # Refused: an opening that runs past the front, a stroke off the body,
    # a spindle outside the chamber, a chuck opening its jaws cannot span.
    with pytest.raises(ValueError, match="does not fit the"):
        bt.parts.lathe(scene, "l2", aperture=(2.5, 0.7, 0.8))
    with pytest.raises(ValueError, match="runs the leaf off"):
        bt.parts.lathe(scene, "l3", door_travel=2.0)
    with pytest.raises(ValueError, match="outside the chamber"):
        bt.parts.lathe(scene, "l4", spindle=(-0.55, 1.5, 1.05))
    with pytest.raises(ValueError, match="does not fit a"):
        bt.parts.chuck(scene, "c2", (0.0, 2.0, 1.0), opening=0.20)
    # A servo door is an axis with two stops; no door is a solid front.
    other = scene_()
    other.set_robot_base_pose((4.0, 0.0, 0.0))
    driven = bt.parts.lathe(other, "lathe", door="servo", panel=None)
    assert driven.door == "lathe/front_door" and other.device_names == ["lathe/front_door"]
    assert driven.door_lanes == ("lathe/front_door/closed", "lathe/front_door/open")
    assert rows(other)["lathe/front_door"]["attributes"]["open_s"] == pytest.approx(0.955)
    driven.remove(other)
    solid = bt.parts.lathe(other, "lathe", door=None, panel=None)
    assert solid.door_lanes is None and "lathe/shell/front" in solid.obstacles and other.sensor_names == []
