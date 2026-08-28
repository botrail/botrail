import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def robot() -> bt.Robot:
    return bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")


def test_default_tcp_link(robot: bt.Robot) -> None:
    assert robot.tcp_link == "tool0"


def test_ik_roundtrip_through_fk(robot: bt.Robot) -> None:
    # Take a pose the arm can certainly reach: FK of a known configuration.
    scene = bt.Scene(robot)
    q_true = [0.4, -0.9, 1.2, 0.3, 0.8, -0.5]
    scene.set_joint_positions(q_true)
    position, quaternion = scene.link_pose("tool0")

    result = robot.ik(position, quaternion)
    assert result.converged, (result.pos_error, result.rot_error)

    # The found configuration must realize the same pose.
    scene.set_joint_positions(result.q)
    reached_pos, _ = scene.link_pose("tool0")
    assert reached_pos == pytest.approx(position, abs=1e-4)


def test_ik_position_only(robot: bt.Robot) -> None:
    result = robot.ik((0.3, 0.2, 0.4))
    assert result.converged
    scene = bt.Scene(robot)
    scene.set_joint_positions(result.q)
    (x, y, z), _ = scene.link_pose("tool0")
    assert (x, y, z) == pytest.approx((0.3, 0.2, 0.4), abs=1e-4)


def test_ik_unreachable_is_best_effort(robot: bt.Robot) -> None:
    result = robot.ik((2.0, 0.0, 0.2))
    assert not result.converged
    assert result.pos_error > 0.9


def test_ik_respects_limits(robot: bt.Robot) -> None:
    result = robot.ik((0.2, -0.3, 0.6))
    for qi, limits in zip(result.q, robot.joint_limits):
        if limits is not None:
            lower, upper = limits
            assert lower - 1e-9 <= qi <= upper + 1e-9


def test_ik_rejects_unknown_link(robot: bt.Robot) -> None:
    with pytest.raises(ValueError):
        robot.ik((0.1, 0.1, 0.1), link="no_such_link")


def test_scene_set_tcp_target_moves_scene(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    before = scene.joint_positions
    result = scene.set_tcp_target((0.3, 0.1, 0.5))
    assert result.converged
    assert scene.joint_positions == result.q
    assert scene.joint_positions != before

    (x, y, z), _ = scene.link_pose("tool0")
    assert (x, y, z) == pytest.approx((0.3, 0.1, 0.5), abs=1e-4)


def test_ik_seed_continuity(robot: bt.Robot) -> None:
    # A warm seed near the solution should converge in very few iterations.
    scene = bt.Scene(robot)
    q0 = [0.4, -0.9, 1.2, 0.3, 0.8, -0.5]
    scene.set_joint_positions(q0)
    position, quaternion = scene.link_pose("tool0")
    nudged = (position[0] + 0.005, position[1], position[2])
    result = robot.ik(nudged, quaternion, seed=q0)
    assert result.converged
    assert result.iters <= 10
    assert result.q == pytest.approx(q0, abs=0.1)
