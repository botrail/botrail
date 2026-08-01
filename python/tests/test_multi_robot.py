"""Multi-robot scenes (R1): add_robot, robot= addressing, project round-trip."""

import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def robot() -> bt.Robot:
    return bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf")


@pytest.fixture()
def duo(robot: bt.Robot) -> bt.Scene:
    scene = bt.Scene(robot)
    scene.add_robot(robot, name="arm_b", base_position=(1.5, 0.0, 0.0))
    return scene


def test_add_robot_names_and_listing(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    assert scene.robots == ["simple_arm"]
    # Same model again: the default name is uniquified.
    assert scene.add_robot(robot) == "simple_arm_2"
    assert scene.add_robot(robot, name="arm_c") == "arm_c"
    assert scene.robots == ["simple_arm", "simple_arm_2", "arm_c"]
    assert scene.robot_of("arm_c").dof == 6
    with pytest.raises(ValueError, match="unknown robot"):
        scene.robot_of("nope")


def test_scene_constructor_names_the_first_robot(robot: bt.Robot) -> None:
    scene = bt.Scene(robot, name="left")
    assert scene.robots == ["left"]


def test_robot_kwarg_addresses_instances(duo: bt.Scene) -> None:
    q = [0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
    duo.set_joint_positions(q, robot="arm_b")
    assert duo.joint_positions_of("arm_b") == pytest.approx(q)
    # The first robot is untouched.
    assert duo.joint_positions_of("simple_arm") == pytest.approx([0.0] * 6)

    duo.set_robot_base_pose((2.0, 1.0, 0.0), robot="arm_b")
    position, _ = duo.robot_base_pose_of("arm_b")
    assert position == pytest.approx((2.0, 1.0, 0.0))
    # link_pose resolves within the addressed robot despite identical names.
    (x, y, _z), _ = duo.link_pose("base_link", robot="arm_b")
    assert (x, y) == pytest.approx((2.0, 1.0))


def test_omitted_robot_is_ambiguous_with_two_robots(duo: bt.Scene) -> None:
    with pytest.raises(ValueError, match="pass robot="):
        duo.set_joint_positions([0.0] * 6)
    with pytest.raises(ValueError, match="pass robot="):
        duo.plan([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_omitted_robot_still_works_with_one_robot(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    assert scene.joint_positions == pytest.approx([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])


def test_plan_avoids_the_other_robot(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    # Lean toward +x; the goal is the same lean panned to -x. The naive
    # sweep passes +y, where a second upright arm stands.
    lean = [0.0, 1.2, 0.0, 0.0, 0.0, 0.0]
    scene.set_joint_positions(lean)
    scene.add_robot(robot, name="blocker", base_position=(0.0, 0.55, 0.0))
    goal = [math.pi, 1.2, 0.0, 0.0, 0.0, 0.0]
    traj = scene.plan(goal, robot="simple_arm", broadcast=False)
    assert traj.sample(traj.duration) == pytest.approx(goal, abs=1e-6)
    # The planner had to detour: joint names stay the planning robot's.
    assert traj.joint_names[0] == "shoulder_pan"


def test_attach_to_second_robot(duo: bt.Scene) -> None:
    duo.add_box("box", size=(0.04, 0.04, 0.04), position=(1.5, 0.1, 0.9))
    duo.attach("box", link="tool0", robot="arm_b")
    assert ("box", "tool0") in duo.attachments
    # Rotating arm_b's base joint carries the box; the first arm does not.
    duo.set_joint_positions([math.pi / 2, 0.0, 0.0, 0.0, 0.0, 0.0], robot="arm_b")
    (x, y, _z), _ = duo.obstacle_pose("box")
    assert (x, y) == pytest.approx((1.4, 0.0), abs=1e-6)


def test_watch_robots_sensor(duo: bt.Scene) -> None:
    duo.add_zone_sensor(
        "zone_a",
        position=(0.0, 0.0, 0.5),
        size=(0.4, 0.4, 1.0),
        watch_robots=["simple_arm"],
    )
    assert "zone_a" in duo.sensor_names
    # Round-trips through the project/codegen path below.


def test_project_round_trip(tmp_path: Path, duo: bt.Scene) -> None:
    duo.set_joint_positions([0.2, 0.4, -0.3, 0.0, 0.1, 0.0], robot="arm_b")
    duo.add_box("crate", size=(0.1, 0.1, 0.1), position=(1.5, 0.2, 0.9))
    duo.attach("crate", link="tool0", robot="arm_b")
    duo.add_segment("b_home", goal=[0.0] * 6, robot="arm_b")
    duo.add_zone_sensor(
        "zone_a",
        position=(0.0, 0.0, 0.5),
        size=(0.4, 0.4, 1.0),
        watch_robots=["arm_b"],
    )

    path = tmp_path / "cell.botrail"
    duo.save_project(path)
    loaded = bt.Scene.load_project(path)
    assert loaded.robots == ["simple_arm", "arm_b"]
    position, _ = loaded.robot_base_pose_of("arm_b")
    assert position == pytest.approx((1.5, 0.0, 0.0))
    assert loaded.joint_positions_of("arm_b") == pytest.approx(
        [0.2, 0.4, -0.3, 0.0, 0.1, 0.0]
    )
    assert ("crate", "tool0") in loaded.attachments
    assert "b_home" in loaded.motion_names
    # The reloaded motion still plans on arm_b (its goal DOF checks against
    # the owner), and the grasped crate still rides arm_b's joints.
    loaded.set_joint_positions([math.pi / 2, 0.4, -0.3, 0.0, 0.1, 0.0], robot="arm_b")
    (bx, _by, _bz), _ = loaded.obstacle_pose("crate")
    assert bx < 1.5


def test_generated_python_rebuilds_two_robots(tmp_path: Path, duo: bt.Scene) -> None:
    duo.set_joint_positions([0.2, 0.0, 0.0, 0.0, 0.0, 0.0], robot="arm_b")
    code = duo.generate_python()
    assert 'scene.add_robot(robot_2, name="arm_b"' in code
    assert 'robot="arm_b"' in code
    # The script must actually run and rebuild the same cell.
    namespace: dict = {}
    exec(compile(code.replace("bt.studio(scene)", ""), "<generated>", "exec"), namespace)
    rebuilt = namespace["scene"]
    assert rebuilt.robots == duo.robots
    assert rebuilt.joint_positions_of("arm_b") == pytest.approx(
        [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    position, _ = rebuilt.robot_base_pose_of("arm_b")
    assert position == pytest.approx((1.5, 0.0, 0.0))
