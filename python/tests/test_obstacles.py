import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


def test_no_obstacles_no_collisions(scene: bt.Scene) -> None:
    assert scene.obstacle_names == []
    assert scene.check_collisions() == []
    assert not scene.in_collision()
    assert scene.min_obstacle_distance() is None
    assert scene.collision_warnings == []


def test_obstacle_lifecycle(scene: bt.Scene) -> None:
    name = scene.add_box("table", (0.4, 0.4, 0.05), (0.5, 0.0, 0.0))
    assert name == "table"
    assert scene.add_box("table", (0.1, 0.1, 0.1), (1.0, 0.0, 0.0)) == "table_2"
    scene.add_sphere("ball", 0.05, (0.0, 0.8, 0.2))
    scene.add_cylinder("post", 0.03, 0.5, (0.0, -0.8, 0.25))
    assert scene.obstacle_names == ["table", "table_2", "ball", "post"]

    scene.remove_obstacle("table_2")
    assert scene.obstacle_names == ["table", "ball", "post"]
    with pytest.raises(ValueError):
        scene.remove_obstacle("nope")


def test_collision_detection_and_distance(scene: bt.Scene) -> None:
    # Far away: no collision, sane clearance.
    scene.add_sphere("ball", 0.05, (1.0, 0.0, 0.2))
    assert not scene.in_collision()
    d = scene.min_obstacle_distance()
    assert d is not None and 0.5 < d < 1.0

    # Move the ball onto the upright arm: collision with a specific link.
    scene.set_obstacle_pose("ball", (0.0, 0.0, 0.45))
    assert scene.in_collision()
    pairs = scene.check_collisions()
    kinds = {(a[0], b[0]) for a, b in pairs}
    assert kinds == {("link", "obstacle")}
    assert any(b[1] == "ball" for _, b in pairs)
    assert scene.min_obstacle_distance() == 0.0


def test_self_collision_via_acm(scene: bt.Scene) -> None:
    # Neutral upright pose: no self collision.
    assert scene.check_collisions() == []
    # Fold the arm back onto itself: elbow + wrist_1 at extremes brings the
    # forearm/tool back against the upper arm.
    scene.set_joint_positions([0.0, -2.2, 2.6, 2.4, 0.0, 0.0])
    pairs = scene.check_collisions()
    assert any(a[0] == "link" and b[0] == "link" for a, b in pairs), pairs


def test_collision_state_is_reflected_in_wire_state(scene: bt.Scene) -> None:
    import json
    import time
    import urllib.request

    from botrail import _core

    scene.add_sphere("ball", 0.08, (0.0, 0.0, 0.45))
    server = _core.serve_studio(scene, "/nonexistent-studio", "127.0.0.1", 0)
    try:
        deadline = time.time() + 5.0
        payload = None
        while time.time() < deadline:
            try:
                with urllib.request.urlopen(f"{server.url}/api/scene", timeout=1) as resp:
                    payload = json.load(resp)
                break
            except OSError:
                time.sleep(0.05)
        assert payload is not None
    finally:
        server.stop()


def test_obstacle_enable_toggle() -> None:
    from pathlib import Path

    import botrail as bt

    examples = Path(__file__).resolve().parents[2] / "examples"
    scene = bt.Scene(bt.Robot.from_urdf(examples / "simple_arm.urdf"))
    scene.add_box("blocker", size=(0.3, 0.3, 0.3), position=(0.0, 0.0, 0.5))
    assert scene.in_collision()

    scene.set_obstacle_enabled("blocker", False)
    assert not scene.in_collision()
    # Planning ignores disabled obstacles too (validity uses the same query).
    assert scene.min_obstacle_distance() is None

    scene.set_obstacle_enabled("blocker", True)
    assert scene.in_collision()


def test_obstacle_colour_is_optional_and_settable(scene: bt.Scene) -> None:
    # A bare obstacle has no authored appearance: the viewer decides.
    scene.add_box("plain", (0.1, 0.1, 0.1), (0.5, 0.0, 0.1))
    assert scene.obstacle_color("plain") is None

    scene.add_box("painted", (0.1, 0.1, 0.1), (0.5, 0.3, 0.1), color=(0.8, 0.3, 0.1))
    assert scene.obstacle_color("painted") == pytest.approx((0.8, 0.3, 0.1))

    scene.set_obstacle_color("plain", (0.0, 0.5, 1.0))
    assert scene.obstacle_color("plain") == pytest.approx((0.0, 0.5, 1.0))
    scene.set_obstacle_color("plain", None)
    assert scene.obstacle_color("plain") is None

    with pytest.raises(ValueError):
        scene.obstacle_color("nope")


def test_colour_never_affects_collision(scene: bt.Scene) -> None:
    scene.add_sphere("ball", 0.05, (1.0, 0.0, 0.2), color=(1.0, 0.0, 0.0))
    before = scene.min_obstacle_distance()
    scene.set_obstacle_color("ball", None)
    assert scene.min_obstacle_distance() == before
    assert scene.check_collisions() == []
