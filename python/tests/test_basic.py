import json
import math
import time
import urllib.request
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def robot() -> bt.Robot:
    return bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")


def test_robot_properties(robot: bt.Robot) -> None:
    assert robot.name == "simple_arm"
    assert robot.dof == 6
    assert robot.joint_names[0] == "shoulder_pan"
    lower, upper = robot.joint_limits[1]
    assert (lower, upper) == (-2.2, 2.2)
    assert "tool0" in robot.link_names


def test_parse_error_is_value_error() -> None:
    with pytest.raises(ValueError):
        bt.Robot.from_urdf_string("<robot name='broken'>")


def test_scene_fk_and_validation(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    assert scene.joint_positions == [0.0] * 6

    (x, y, z), _ = scene.link_pose("tool0")
    assert (x, y, z) == pytest.approx((0.0, 0.0, 0.85))

    # 90 deg shoulder_lift folds the arm horizontally (about +y axis -> +x).
    scene.set_joint_positions([0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0])
    (x, y, z), _ = scene.link_pose("tool0")
    assert (x, y, z) == pytest.approx((0.71, 0.0, 0.14))

    with pytest.raises(ValueError):
        scene.set_joint_positions([0.0])
    with pytest.raises(ValueError):
        scene.link_pose("no_such_link")


def test_server_serves_scene_api(robot: bt.Robot) -> None:
    from botrail import _core

    scene = bt.Scene(robot)
    # A studio build is not required for the JSON API.
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
        assert payload is not None, "server did not come up within 5s"
        assert payload["type"] == "scene_init"
        assert payload["scene"]["robots"][0]["name"] == "simple_arm"
        assert len(payload["scene"]["robots"][0]["links"]) == len(robot.link_names)
    finally:
        server.stop()
