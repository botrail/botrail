import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


def _dist(a, b) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def _held_position(scene: bt.Scene):
    """A grasp spot just above the tool0 visual (clear of all link geometry)."""
    tcp, _ = scene.link_pose(scene.robot.tcp_link)
    return (tcp[0], tcp[1], tcp[2] + 0.06)


def test_attach_follows_joints_and_detach_freezes(scene: bt.Scene) -> None:
    held_at = _held_position(scene)
    scene.add_box("held", (0.04, 0.04, 0.04), held_at)
    scene.attach("held")
    assert scene.attachments == [("held", scene.robot.tcp_link)]
    assert not scene.in_collision()

    # Moving the arm carries the box: the TCP-to-box distance is rigid.
    tcp0, _ = scene.link_pose(scene.robot.tcp_link)
    grip = _dist(tcp0, held_at)
    scene.set_joint_positions([0.6, -0.4, 0.5, 0.0, 0.3, 0.0])
    tcp1, _ = scene.link_pose(scene.robot.tcp_link)
    assert _dist(tcp0, tcp1) > 0.05  # the arm actually moved
    pos1, _ = scene.obstacle_pose("held")
    assert abs(_dist(tcp1, pos1) - grip) < 1e-9

    # Detach freezes the pose at the release point.
    scene.detach("held")
    assert scene.attachments == []
    scene.set_joint_positions([0.0] * 6)
    frozen, _ = scene.obstacle_pose("held")
    assert _dist(frozen, pos1) < 1e-12


def test_attached_object_collides_with_environment(scene: bt.Scene) -> None:
    held_at = _held_position(scene)
    scene.add_box("held", (0.04, 0.04, 0.04), held_at)
    scene.attach("held")
    assert not scene.in_collision()

    # A wall overlapping the held box is a collision (obstacle x obstacle)
    # even though no robot link touches it. Placed straight above the box so
    # every link stays farther away than the box itself.
    scene.add_box("wall", (0.04, 0.04, 0.04), (held_at[0], held_at[1], held_at[2] + 0.03))
    assert scene.in_collision()
    names = {n for pair in scene.check_collisions() for _, n in pair}
    assert {"held", "wall"} <= names

    # Clearance measures the held box once the wall backs away.
    scene.set_obstacle_pose("wall", (held_at[0], held_at[1], held_at[2] + 0.14))
    assert not scene.in_collision()
    d = scene.min_obstacle_distance()
    assert d is not None and abs(d - 0.10) < 1e-6


def test_attach_errors(scene: bt.Scene) -> None:
    with pytest.raises(ValueError):
        scene.attach("ghost")
    scene.add_box("held", (0.02, 0.02, 0.02), _held_position(scene))
    with pytest.raises(ValueError):
        scene.attach("held", link="not_a_link")
    scene.attach("held", touch_links=["tool0", "wrist_3_link"])
    with pytest.raises(ValueError):
        scene.attach("held")
    with pytest.raises(ValueError):
        scene.detach("nope")
    scene.detach("held")
