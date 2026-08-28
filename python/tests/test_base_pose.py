"""Robot base pose: world-centric scene behavior (Phase 1)."""

import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def robot() -> bt.Robot:
    return bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")


def quat_z(angle: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))


def test_default_base_is_identity(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    position, quaternion = scene.robot_base_pose
    assert position == pytest.approx((0.0, 0.0, 0.0))
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_base_pose_shifts_link_poses(robot: bt.Robot) -> None:
    scene = bt.Scene(robot, base_position=(1.0, 2.0, 0.5))
    (x, y, z), _ = scene.link_pose("tool0")
    # tool0 sits at (0, 0, 0.85) for the identity base.
    assert (x, y, z) == pytest.approx((1.0, 2.0, 1.35))

    scene.set_robot_base_pose((0.0, 0.0, 0.0))
    (x, y, z), _ = scene.link_pose("tool0")
    assert (x, y, z) == pytest.approx((0.0, 0.0, 0.85))


def test_rotated_base_rotates_the_workspace(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.0, 1.5708, 0.0, 0.0, 0.0, 0.0])
    (x0, y0, z0), _ = scene.link_pose("tool0")
    assert x0 > 0.3  # folded horizontally toward +x

    # Rotate the base 90 deg about z: the arm now reaches toward +y.
    scene.set_robot_base_pose((0.0, 0.0, 0.0), quat_z(math.pi / 2))
    (x1, y1, z1), _ = scene.link_pose("tool0")
    assert y1 == pytest.approx(x0, abs=1e-6)
    assert x1 == pytest.approx(0.0, abs=1e-6)
    assert z1 == pytest.approx(z0, abs=1e-6)


def test_obstacles_stay_in_world_frame(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    # A box engulfing the arm at the origin collides.
    scene.add_box("blocker", size=(0.4, 0.4, 0.4), position=(0.0, 0.0, 0.4))
    assert scene.in_collision()
    # Moving the robot away clears it; the obstacle does not follow.
    scene.set_robot_base_pose((2.0, 0.0, 0.0))
    assert not scene.in_collision()


def test_world_frame_ik_and_planning_with_moved_base(robot: bt.Robot) -> None:
    scene = bt.Scene(robot, base_position=(0.8, -0.3, 0.2), base_quaternion=quat_z(0.7))

    scene.set_joint_positions([0.3, 0.9, -1.2, 0.4, 0.2, 0.0])
    (tx, ty, tz), _ = scene.link_pose("tool0")

    result = scene.set_tcp_target((tx, ty, tz - 0.05))
    assert result.converged
    (rx, ry, rz), _ = scene.link_pose("tool0")
    assert (rx, ry, rz) == pytest.approx((tx, ty, tz - 0.05), abs=1e-3)

    # plan_to_pose accepts the same world-frame target (moving back to a
    # different start first, so there is an actual path to plan).
    scene.set_joint_positions([0.3, 0.9, -1.2, 0.4, 0.2, 0.0])
    traj = scene.plan_to_pose((tx, ty, tz - 0.05), broadcast=False)
    assert traj.duration > 0.0
    end = traj.sample(traj.duration)
    scene.set_joint_positions(end)
    (px, py, pz), _ = scene.link_pose("tool0")
    assert (px, py, pz) == pytest.approx((tx, ty, tz - 0.05), abs=1e-3)


def test_project_roundtrip_preserves_base_pose(robot: bt.Robot, tmp_path: Path) -> None:
    scene = bt.Scene(robot, base_position=(0.5, 0.1, 0.0), base_quaternion=quat_z(0.4))
    path = tmp_path / "cell.botrail"
    scene.save_project(str(path))

    reloaded = bt.Scene.load_project(str(path))
    position, quaternion = reloaded.robot_base_pose
    assert position == pytest.approx((0.5, 0.1, 0.0))
    assert quaternion == pytest.approx(quat_z(0.4))

    code = reloaded.generate_python()
    assert "base_position=(0.500000, 0.100000, 0.000000)" in code
