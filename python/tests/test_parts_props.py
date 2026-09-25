"""`bt.parts` props — the generic things a cell is full of and nobody orders
by part number: the tray a part waits in, the stage under a camera, the
carton, the stock on a pallet, the paint on the floor, the person a
scenario stands in the gate, the portal a camera hangs from. Dimension-
driven like every generator; these tests pin what each makes, what
collides and what is a picture, the frame a set-down aims at, the BOM row
(or its absence), and that a `Built` takes itself down."""

import json
import math

import botrail as bt
import pytest


def objects(scene) -> dict:
    return {o["name"]: o for o in json.loads(scene._project_json())["obstacles"]}


def rows(scene) -> dict:
    return {row["names"][0]: row for row in scene.bom().rows}


def test_tray_collides_as_its_box_seats_on_its_insert_and_takes_itself_down():
    scene = bt.Scene()
    built = bt.parts.tray(scene, "tray", (0.26, 0.26, 0.12), (1.0, 2.0, 0.74), yaw=-math.pi / 2, seat=0.018,
                          model="部品トレイ", description="the machined parts")
    by = objects(scene)
    assert sorted(built.obstacles) == ["tray", "tray/insert", "tray/trim/grip0", "tray/trim/grip1"]
    assert by["tray"]["enabled"] and by["tray"]["visual_asset"]["prim_path"] == "/Shapes/tray"
    assert not by["tray/insert"]["enabled"] and by["tray/insert"]["visual_asset"]["color_override"] is True
    assert all(not by[g]["enabled"] and by[g]["visual_asset"]["prim_path"] == "/Shapes/handle"
               for g in ("tray/trim/grip0", "tray/trim/grip1"))
    lo, hi = scene.obstacle_bounds("tray")
    assert (lo[2], hi[2]) == pytest.approx((0.74, 0.86))
    (x, y, z), _ = scene.frame("tray/seat")
    assert (x, y, z) == pytest.approx((1.0, 2.0, 0.86 + 0.018))
    row = rows(scene)["tray"]
    assert (row["category"], row["model"], row["description"]) == ("tray", "部品トレイ", "the machined parts")
    built.remove(scene)
    assert scene.obstacle_names == [] and not scene.frames and scene.parts() == []
    plain = bt.parts.tray(scene, "plain", (0.3, 0.2, 0.1), (0, 0), detail="plain", grips=False)
    assert plain.obstacles == ["plain", "plain/insert"] and "visual_asset" not in objects(scene)["plain"]
    with pytest.raises(ValueError):
        bt.parts.tray(scene, "bad", (0.3, 0.0, 0.1), (0, 0))


def test_stage_hides_its_block_behind_a_plate_on_legs():
    scene = bt.Scene()
    built = bt.parts.stage(scene, "stage", (0.26, 0.26, 0.12), (3.0, 2.0, 0.74), yaw=math.pi / 2, model="検査ステージ")
    by = objects(scene)
    assert by["stage"]["enabled"] and not by["stage"]["visible"]
    legs = [n for n in built.obstacles if n.startswith("stage/trim/leg")]
    assert len(legs) == 4 and all(not by[n]["enabled"] for n in legs)
    assert by["stage/trim/top"]["visual_asset"]["prim_path"] == "/Shapes/panel"
    lo, hi = scene.obstacle_bounds("stage/trim/leg0")
    assert (lo[2], hi[2]) == pytest.approx((0.74, 0.74 + 0.12 - 0.016))
    assert scene.frame("stage/seat")[0][2] == pytest.approx(0.86 + 0.018)
    assert rows(scene)["stage"]["category"] == "fixture"
    with pytest.raises(ValueError):
        bt.parts.stage(scene, "bad", (0.26, 0.26, 0.01), (0, 0), plate=0.016)


def test_carton_is_one_identified_resident_with_its_mass():
    scene = bt.Scene()
    name = bt.parts.carton(scene, "case", (0.36, 0.28, 0.24), (0.5, 0.5, 0.4), yaw=0.3, mass_kg=5.0)
    assert name == "case" and scene.obstacle_names == ["case"]
    by = objects(scene)
    assert by["case"]["enabled"] and by["case"]["visual_asset"]["prim_path"] == "/Shapes/carton"
    lo, hi = scene.obstacle_bounds("case")
    assert (lo[2], hi[2]) == pytest.approx((0.4, 0.64))
    row = rows(scene)["case"]
    assert (row["category"], row["model"], row["attributes"]["mass_kg"]) == ("workpiece", "RSC-360x280x240", 5.0)
    bt.parts.carton(scene, "other", (0.3, 0.3, 0.3), (2, 2), model="K-30", mass_kg=1.5, detail="plain")
    assert scene.bom().total("mass_kg") == pytest.approx(6.5)
    assert "visual_asset" not in objects(scene)["other"] and rows(scene)["other"]["model"] == "K-30"


def test_bin_is_five_boxes_pinned_as_one_unit_with_a_floor_to_set_down_on():
    """A KLT: four walls and a floor that all collide (a thing set down
    inside lands on the floor and stops at a wall), one part on the group
    so a physics bake carries the five as one rigid unit (design-rl-tabletop.md
    G13), the sleeve a picture round them."""
    scene = bt.Scene()
    built = bt.parts.bin(scene, "bin", (0.3, 0.2, 0.147), (1.0, 2.0, 0.75), yaw=math.pi / 2, mass_kg=0.6,
                         model="R-KLT 3215")
    by = objects(scene)
    assert sorted(built.obstacles) == ["bin/floor", "bin/trim/sleeve", "bin/wall0", "bin/wall1", "bin/wall2", "bin/wall3"]
    assert all(by[n]["enabled"] for n in built.obstacles if not n.startswith("bin/trim/"))
    sleeve = by["bin/trim/sleeve"]
    assert not sleeve["enabled"] and sleeve["visual_asset"]["prim_path"] == "/Shapes/tote"
    assert sleeve["visual_asset"]["color_override"] is True
    assert scene.obstacle_finish("bin/wall0") == "plastic" and scene.obstacle_finish("bin/floor") == "plastic"
    # Turned a quarter, the length lies along y. The floor, 12 mm thick,
    # sits between 12 mm walls that stand the full height.
    lo, hi = scene.obstacle_bounds("bin/floor")
    assert (hi[0] - lo[0], hi[1] - lo[1]) == pytest.approx((0.2 - 0.024, 0.3 - 0.024))
    assert (lo[2], hi[2]) == pytest.approx((0.75, 0.762))
    lo, hi = scene.obstacle_bounds("bin/wall0")
    assert (hi[0] - lo[0], hi[1] - lo[1], lo[2], hi[2]) == pytest.approx((0.2, 0.012, 0.75, 0.897))
    assert scene.frame("bin/floor")[0] == pytest.approx((1.0, 2.0, 0.762))
    row = rows(scene)["bin"]
    assert (row["category"], row["model"], row["attributes"]["mass_kg"]) == ("bin", "R-KLT 3215", 0.6)
    # One rigid unit under the world scope, weighing what the row says.
    plan = scene.physics_plan(physics=bt.Physics(world=True))
    unit = next(r for r in plan.rows if r["name"] == "bin")
    assert unit["kind"] == "dynamic" and unit["mass_kg"] == pytest.approx(0.6)
    assert sorted(unit["members"]) == sorted(built.obstacles)
    assert plan.dynamic() == ["bin"]
    built.remove(scene)
    assert scene.obstacle_names == [] and not scene.frames and scene.parts() == []
    # Plain detail is the five boxes; walls may differ across and along.
    plain = bt.parts.bin(scene, "plain", (0.4, 0.3, 0.147), (0, 0), detail="plain", wall=(0.027, 0.0175), floor=0.038)
    assert plain.obstacles == ["plain/floor", "plain/wall0", "plain/wall1", "plain/wall2", "plain/wall3"]
    lo, hi = scene.obstacle_bounds("plain/floor")
    assert (hi[0] - lo[0], hi[1] - lo[1], hi[2]) == pytest.approx((0.346, 0.265, 0.038))
    assert rows(scene)["plain"]["model"] == "BIN-400x300x147"
    with pytest.raises(ValueError):
        bt.parts.bin(scene, "bad", (0.3, 0.2, 0.147), (0, 0), wall=0.2)
    with pytest.raises(ValueError):
        bt.parts.bin(scene, "bad", None, (0, 0))


def test_unit_load_collides_as_two_hidden_envelopes_and_stacks_its_cartons_flush():
    scene = bt.Scene()
    built = bt.parts.unit_load(scene, "stock", (2, 3, 1.3), yaw=0.0, pallet=(0.8, 1.2, 0.144), height=0.9)
    by = objects(scene)
    colliding = sorted(n for n in built.obstacles if by[n]["enabled"])
    assert colliding == ["stock/load", "stock/pallet"] and all(not by[n]["visible"] for n in colliding)
    assert any(n.startswith("stock/visual/timber/") for n in built.obstacles)
    support = 1.3 + 0.144
    for course in range(round(0.9 / 0.28)):
        lo, hi = scene.obstacle_bounds(f"stock/visual/case{course}00")
        assert lo[2] == pytest.approx(support, abs=1e-9)
        support = hi[2]
    assert support == pytest.approx(1.3 + 0.144 + 0.9, abs=0.003)
    assert scene.bom().rows == [] and not scene.frames
    # The same name draws the same stock; another name may draw another lot.
    twin = bt.Scene()
    bt.parts.unit_load(twin, "stock", (2, 3, 1.3), yaw=0.0, pallet=(0.8, 1.2, 0.144), height=0.9)
    assert [o for o in json.loads(twin._project_json())["obstacles"]] == list(by.values())
    upper = bt.parts.unit_load(scene, "high", (5, 3, 3.0), height=0.6, collide=False)
    assert not any(objects(scene)[n]["enabled"] for n in upper.obstacles)
    built.remove(scene)
    upper.remove(scene)
    assert scene.obstacle_names == []


def test_marking_paints_the_ground_layer_and_never_collides():
    scene = bt.Scene()
    area = bt.parts.marking(scene, "marking/recv", rect=(1.0, 10.5, 8.0, 21.5), color=bt.parts.LINE_GREEN, floor=0.003)
    assert area.obstacles == [f"marking/recv/{i}" for i in range(4)]
    bounds = [scene.obstacle_bounds(n) for n in area.obstacles]
    assert (min(lo[0] for lo, _ in bounds), max(hi[0] for _, hi in bounds)) == pytest.approx((1.0 - 0.04, 8.0 + 0.04))
    assert (min(lo[2] for lo, _ in bounds), max(hi[2] for _, hi in bounds)) == pytest.approx((0.003, 0.006))
    lane = bt.parts.marking(scene, "marking/lane", line=((2.0, 7.0), (37.35, 7.0)), dash=(0.85, 0.65), width=0.06)
    assert len(lane.obstacles) == 24
    lo, hi = scene.obstacle_bounds("marking/lane/23")
    assert (lo[0], hi[0]) == pytest.approx((36.5, 37.35))
    stripe = bt.parts.marking(scene, "marking/branch", line=((3.0, 1.0), (3.0, 4.0)), width=0.06)
    assert stripe.obstacles == ["marking/branch"]
    by = objects(scene)
    assert not any(by[n]["enabled"] for n in area.obstacles + lane.obstacles + stripe.obstacles)
    items = json.loads(scene.layout("json"))["items"]
    layers = {item["layer"] for item in items if item["name"].startswith("marking/")}
    assert layers == {"ground"} and scene.bom().rows == []
    with pytest.raises(ValueError):
        bt.parts.marking(scene, "both", rect=(0, 0, 1, 1), line=((0, 0), (1, 1)))


def test_person_stands_in_collision_and_gantry_frames_its_beam():
    scene = bt.Scene()
    assert bt.parts.person(scene, "visitor", (-7.7, 2.1)) == "visitor"
    lo, hi = scene.obstacle_bounds("visitor")
    assert (hi[2] - lo[2], hi[0] - lo[0]) == pytest.approx((1.7, 0.4)) and objects(scene)["visitor"]["enabled"]
    built = bt.parts.gantry(scene, "inspector", 0.84, 2.05, (3.0, 2.0), yaw=0.0, category="machine.inspection",
                            manufacturer="柳下技研", model="画像検査ステーション")
    assert sorted(built.obstacles) == ["inspector/beam", "inspector/post_l", "inspector/post_r"]
    lo, hi = scene.obstacle_bounds("inspector/post_l")
    assert ((lo[0] + hi[0]) / 2, hi[2]) == pytest.approx((3.0 - 0.42, 2.05))
    lo, hi = scene.obstacle_bounds("inspector/beam")
    assert (hi[0] - lo[0], lo[2], hi[2]) == pytest.approx((0.90, 2.025, 2.075))
    assert scene.frame("inspector/beam")[0] == pytest.approx((3.0, 2.0, 2.025))
    row = rows(scene)["inspector"]
    assert (row["category"], row["manufacturer"], row["qty"]) == ("machine.inspection", "柳下技研", 1)
    assert scene.check_collisions() == []
    built.remove(scene)
    assert scene.obstacle_names == ["visitor"] and not scene.frames
