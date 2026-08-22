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
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


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
    scene.save_project(tmp_path / "cell.botrail")
    again = bt.Scene.load_project(tmp_path / "cell.botrail")
    assert again.bom().rows == scene.bom().rows
    # Poses round-trip through JSON with float noise, so compare the
    # rendered sheet (rounded to a tenth of a pixel) and the extents.
    assert again.layout("svg") == scene.layout("svg")
    assert again.footprint() == pytest.approx(scene.footprint())
    assert set(again.frames) == set(scene.frames)


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
