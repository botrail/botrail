"""Scenery joints — doors, drawers and racks posed on a hinge or a slide
(`.internal/design-scenery-joints.md`).

A joint names the obstacles that move together, the axis and the range;
setting it rewrites their poses. These tests pin the Python surface
(`add_hinge`, `add_slide`, `set_scenery_joint`, `scenery_joint`), that a
fence's door hangs on one, that a prop imported as scenery keeps the
joints its file authored, and that a saved project carries them.
"""

import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


def scene_() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def at(scene, name: str) -> tuple:
    return scene.obstacle_pose(name)[0]


def door_() -> bt.Scene:
    scene = scene_()
    scene.add_box("gate/post", size=(0.06, 0.06, 2.0), position=(2.0, 0.0, 1.0))
    scene.add_box("gate/leaf", size=(0.8, 0.03, 1.9), position=(2.4, 0.0, 1.0))
    scene.add_box("gate/handle", size=(0.03, 0.06, 0.12), position=(2.75, -0.04, 1.0))
    scene.add_hinge("gate", ["gate/leaf", "gate/handle"], origin=(2.0, 0.0, 0.0),
                    range=(0.0, math.radians(110)), base="gate/post")
    return scene


def test_a_hinge_swings_its_objects_and_holds_at_its_range() -> None:
    scene = door_()
    assert scene.scenery_joints == ["gate"]
    joint = scene.scenery_joint("gate")
    assert joint["kind"] == "hinge" and joint["position"] == 0.0 and joint["base"] == "gate/post"
    assert joint["objects"] == ["gate/leaf", "gate/handle"]
    assert joint["origin"] == pytest.approx((2.0, 0.0, 0.0)) and joint["axis"] == pytest.approx((0.0, 0.0, 1.0))

    assert scene.set_scenery_joint("gate", math.pi / 2) == pytest.approx(math.pi / 2)
    assert at(scene, "gate/leaf") == pytest.approx((2.0, 0.4, 1.0))
    assert at(scene, "gate/handle") == pytest.approx((2.04, 0.75, 1.0))
    assert at(scene, "gate/post") == pytest.approx((2.0, 0.0, 1.0)), "what it hangs on stays"
    # Past the stop it goes as far as it can; shut is back where it began.
    assert scene.set_scenery_joint("gate", 9.0) == pytest.approx(math.radians(110))
    assert scene.set_scenery_joint("gate", -1.0) == 0.0
    assert at(scene, "gate/leaf") == pytest.approx((2.4, 0.0, 1.0))
    assert scene.scenery_joint("gate")["position"] == 0.0


def test_a_slide_draws_its_objects_along_the_axis() -> None:
    scene = scene_()
    scene.add_box("unit/body", size=(0.6, 0.6, 0.8), position=(3.0, 0.0, 0.4))
    scene.add_box("unit/rack", size=(0.5, 0.5, 0.1), position=(3.0, 0.0, 0.3))
    scene.add_slide("unit/rack", ["unit/rack"], axis=(-1.0, 0.0, 0.0), range=(0.0, 0.45), base="unit/body")
    joint = scene.scenery_joint("unit/rack")
    assert joint["kind"] == "slide" and joint["origin"] == pytest.approx((3.0, 0.0, 0.3))
    assert scene.set_scenery_joint("unit/rack", 0.3) == pytest.approx(0.3)
    assert at(scene, "unit/rack") == pytest.approx((2.7, 0.0, 0.3))
    assert scene.set_scenery_joint("unit/rack", 1.0) == pytest.approx(0.45)
    assert at(scene, "unit/rack") == pytest.approx((2.55, 0.0, 0.3))


def test_the_joint_goes_where_its_unit_is_carried() -> None:
    scene = door_()
    # The whole gate three metres along Y: the hinge is on the post.
    for name in ("gate/post", "gate/leaf", "gate/handle"):
        (x, y, z), quat = scene.obstacle_pose(name)
        scene.set_obstacle_pose(name, (x, y + 3.0, z), quat)
    assert scene.scenery_joint("gate")["origin"] == pytest.approx((2.0, 3.0, 0.0))
    scene.set_scenery_joint("gate", math.pi / 2)
    assert at(scene, "gate/leaf") == pytest.approx((2.0, 3.4, 1.0))


def test_what_is_not_a_joint_is_refused_and_a_removed_one_is_gone() -> None:
    scene = door_()
    with pytest.raises(ValueError, match="unknown scenery joint"):
        scene.set_scenery_joint("door", 0.1)
    with pytest.raises(ValueError, match="unknown scenery joint"):
        scene.scenery_joint("door")
    with pytest.raises(ValueError, match="unknown obstacle"):
        scene.add_hinge("bad", ["gate/ghost"], origin=(0, 0, 0))
    with pytest.raises(ValueError, match="no objects"):
        scene.add_hinge("bad", [], origin=(0, 0, 0))
    with pytest.raises(ValueError, match="moves with it"):
        scene.add_hinge("bad", ["gate/leaf"], origin=(0, 0, 0), base="gate/leaf")
    with pytest.raises(ValueError, match="not an interval"):
        scene.add_slide("bad", ["gate/leaf"], axis=(1, 0, 0), range=(1.0, 0.0))
    assert scene.scenery_joints == ["gate"]
    # Its objects stay as they stand when the joint is forgotten; the
    # joint goes with the last thing it moved.
    scene.set_scenery_joint("gate", 0.5)
    leaf = at(scene, "gate/leaf")
    scene.remove_scenery_joint("gate")
    assert scene.scenery_joints == [] and at(scene, "gate/leaf") == leaf
    scene.add_hinge("gate", ["gate/handle"], origin=(2.0, 0.0, 0.0))
    scene.remove_obstacle("gate/handle")
    assert scene.scenery_joints == []


def test_a_fence_door_hangs_on_a_hinge() -> None:
    scene = scene_()
    bt.parts.fence(scene, "fence", path=[(-2, -2), (2, -2), (2, 2), (-2, 2)],
                   height=2.0, panel_pitch=1.0, door=(0, 2))
    assert scene.scenery_joints == ["fence/door"]
    joint = scene.scenery_joint("fence/door")
    assert joint["kind"] == "hinge" and "fence/door" in joint["objects"]
    assert joint["base"] == "fence/posts/c0"
    lower, upper = joint["range"]
    assert lower == pytest.approx(-upper) and upper > math.pi / 2, "it swings either way"
    # The hinge is at the end of the leaf, on the run: the leaf's centre
    # swings a half width out of the line of the fence.
    x, y, z = at(scene, "fence/door")
    hx, hy, _ = joint["origin"]
    half = math.hypot(x - hx, y - hy)
    assert hy == pytest.approx(-2.0) and half > 0.3
    scene.set_scenery_joint("fence/door", math.pi / 2)
    assert at(scene, "fence/door") == pytest.approx((hx, -2.0 + half, z))
    # A fence without a door has no joint.
    plain = scene_()
    bt.parts.fence(plain, "fence", path=[(-2, -2), (2, -2)], height=2.0, panel_pitch=1.0)
    assert plain.scenery_joints == []


CABINET = """#usda 1.0
(
    defaultPrim = "Model"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Model" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "Carcass" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "box"
        {
            double size = 0.6
        }
    }

    def Xform "Door" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        double3 xformOp:translate = (0.32, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]

        def Cube "panel"
        {
            double size = 0.02
        }
    }

    def Xform "Drawer" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "tray"
        {
            double size = 0.2
        }
    }

    def PhysicsRevoluteJoint "door"
    {
        uniform token physics:axis = "Z"
        rel physics:body0 = </Model/Carcass>
        rel physics:body1 = </Model/Door>
        point3f physics:localPos0 = (0.32, 0.3, 0)
        float physics:lowerLimit = 0
        float physics:upperLimit = 110
    }

    def PhysicsPrismaticJoint "drawer"
    {
        uniform token physics:axis = "X"
        rel physics:body0 = </Model/Carcass>
        rel physics:body1 = </Model/Drawer>
        float physics:lowerLimit = 0
        float physics:upperLimit = 0.4
    }
}
"""


def test_a_prop_imported_as_scenery_keeps_its_doors_and_drawers(tmp_path: Path) -> None:
    stage = tmp_path / "cabinet.usda"
    stage.write_text(CABINET)
    scene = scene_()
    names = scene.load_usd(stage, prefix="cab", articulated="scenery")
    assert sorted(names) == ["cab/Model/Carcass/box", "cab/Model/Door/panel", "cab/Model/Drawer/tray"]
    assert sorted(scene.scenery_joints) == ["cab/Model/door", "cab/Model/drawer"]
    door = scene.scenery_joint("cab/Model/door")
    assert door["kind"] == "hinge" and door["objects"] == ["cab/Model/Door/panel"]
    assert door["base"] == "cab/Model/Carcass/box"
    assert door["range"] == pytest.approx((0.0, math.radians(110)), abs=1e-6)
    assert door["origin"] == pytest.approx((0.32, 0.3, 0.0), abs=1e-6)
    drawer = scene.scenery_joint("cab/Model/drawer")
    assert drawer["kind"] == "slide" and drawer["range"] == pytest.approx((0.0, 0.4), abs=1e-6)
    # The door swings about its hinge, the drawer comes out along X.
    scene.set_scenery_joint("cab/Model/door", math.pi / 2)
    assert at(scene, "cab/Model/Door/panel") == pytest.approx((0.62, 0.3, 0.0), abs=1e-6)
    scene.set_scenery_joint("cab/Model/drawer", 0.25)
    assert at(scene, "cab/Model/Drawer/tray") == pytest.approx((0.25, 0.0, 0.0), abs=1e-6)


def test_a_saved_project_carries_the_joints(tmp_path: Path) -> None:
    scene = door_()
    scene.set_scenery_joint("gate", 0.5)
    path = tmp_path / "gate.botrail"
    scene.save_project(path)
    again = bt.Scene.load_project(path)
    assert again.scenery_joints == ["gate"]
    saved, loaded = scene.scenery_joint("gate"), again.scenery_joint("gate")
    assert loaded.keys() == saved.keys()
    for key in ("kind", "objects", "base"):
        assert loaded[key] == saved[key], key
    for key in ("position", "range", "origin", "axis"):
        assert loaded[key] == pytest.approx(saved[key]), key
    again.set_scenery_joint("gate", 0.0)
    assert at(again, "gate/leaf") == pytest.approx((2.4, 0.0, 1.0))
    # The generated script declares it again.
    script = scene.generate_python()
    assert 'scene.add_hinge("gate", ["gate/leaf", "gate/handle"]' in script
    assert 'base="gate/post")' in script
