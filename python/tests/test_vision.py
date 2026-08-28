"""Vision sensors: a camera's frustum as a presence input.

The rollout regressions the design asks for — in view / out of the
detection band / hidden behind another body — plus the wrist-camera
anchor, the sequence gate, and the things that must come for free
(I/O derivation) or must not happen (a BOM line).
"""

import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def rotate(q, v):
    x, y, z, w = q
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


def bake_hold(scene: bt.Scene, name: str = "look"):
    sq = scene.sequence(name)
    sq.step("hold", transition=bt.seq.elapsed(0.1))
    return sq.simulate()


def test_vision_in_view_range_and_occlusion(scene) -> None:
    # A part 1 m in front of a fixture camera aimed straight at it.
    scene.add_box("part", (0.1, 0.1, 0.1), (0.5, 0.0, 0.3))
    scene.add_camera(
        "cam",
        position=(1.5, 0.0, 0.3),
        look_at=(0.0, 0.0, 0.3),
        fov=60,
        resolution=(640, 480),
        far=5.0,
    )
    scene.add_vision_sensor("seen", camera="cam", watch=["part"])
    scene.add_vision_sensor(
        "near_only", camera="cam", watch=["part"], detect_range=(0.05, 0.4)
    )
    tl = bake_hold(scene)
    assert tl.signal("seen").value_at(0.05) is True
    # The part sits 1 m out — outside a 0.4 m detection band.
    assert tl.signal("near_only").value_at(0.05) is False

    # A wall between camera and part: occlusion blocks the default sensor;
    # a sensor with occlusion off still trips on the frustum overlap.
    scene.add_box("wall", (0.05, 0.6, 0.6), (1.0, 0.0, 0.3))
    scene.add_vision_sensor("xray", camera="cam", watch=["part"], occlusion=False)
    tl = bake_hold(scene, "look2")
    assert tl.signal("seen").value_at(0.05) is False
    assert tl.signal("xray").value_at(0.05) is True


def test_wrist_camera_vision_follows_the_arm(scene) -> None:
    # Place the part 0.4 m down the wrist camera's own view axis, so it is
    # seen wherever the arm happens to stand — then swing the base a
    # quarter turn and the narrow frustum leaves it behind.
    tcp = scene.robot.tcp_link
    joints = [0.4, -0.5, 0.8, 0.0, 0.6, 0.0]
    scene.set_joint_positions(joints)
    p, q = scene.link_pose_at(tcp, joints)
    d = rotate(q, (0.0, 0.0, -1.0))
    at = tuple(p[i] + 0.4 * d[i] for i in range(3))
    scene.add_box("part", (0.15, 0.15, 0.15), at)
    scene.add_camera(
        "wrist", robot=scene.robot.name, link=tcp, fov=25, resolution=(640, 480), far=2.0
    )
    scene.add_vision_sensor("in_sight", camera="wrist", watch=["part"])
    tl = bake_hold(scene)
    assert tl.signal("in_sight").value_at(0.05) is True

    swung = [joints[0] + math.pi / 2, *joints[1:]]
    scene.set_joint_positions(swung)
    tl = bake_hold(scene, "look2")
    assert tl.signal("in_sight").value_at(0.05) is False


def test_vision_gates_a_sequence_on_arrival(scene) -> None:
    # The design demo: a belt carries the part into the camera's view and
    # the sequence steps forward the moment it is seen.
    scene.add_box("crate", (0.1, 0.1, 0.1), (-1.0, 0.6, 0.3))
    scene.add_conveyor(
        "belt",
        zone_position=(-0.2, 0.6, 0.3),
        zone_size=(2.4, 0.3, 0.3),
        velocity=(0.25, 0.0, 0.0),
        running=False,
    )
    scene.add_camera(
        "gate",
        position=(0.0, 1.4, 0.5),
        look_at=(0.0, 0.6, 0.3),
        fov=30,
        resolution=(640, 480),
        far=2.0,
    )
    scene.add_vision_sensor("crate_seen", camera="gate", watch=["crate"])
    sq = scene.sequence("feed")
    sq.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("crate_seen"))
    sq.step("stop", actions=[bt.seq.stop("belt")], transition=bt.seq.elapsed(0.1))
    tl = sq.simulate()
    # ~1 m of travel at 0.25 m/s before the crate enters the 30° cone
    # around (0, 0.6): the gate fires while the crate crosses the view,
    # not at t=0 and not never.
    fed = tl.step_spans[0][2]
    assert 2.5 < fed < 5.0, fed
    # The transition fires on the very tick the lane rises.
    assert tl.signal("crate_seen").value_at(fed + 0.005) is True


def test_vision_derives_io_but_no_bom_line(scene) -> None:
    scene.add_box("part", (0.1, 0.1, 0.1), (0.5, 0.0, 0.3))
    scene.add_camera("cam", position=(1.5, 0, 0.3), look_at=(0, 0, 0.3))
    scene.add_vision_sensor("seen", camera="cam", watch=["part"])
    # Rule ① is kind-agnostic: the lane derives a DI input contact.
    points = {p.name: p for p in scene.io_points()}
    assert "seen" in points
    assert points["seen"].direction == "input"
    # The purchasable article is the camera, not the judgement: no BOM row.
    assert not any(row.get("target") == "seen" for row in scene.bom().rows)


def test_vision_round_trip_and_codegen(scene, tmp_path) -> None:
    scene.add_camera("cam", position=(1.5, 0, 0.3), look_at=(0, 0, 0.3))
    scene.add_vision_sensor(
        "seen", camera="cam", watch=["ghost"], detect_range=(0.1, 2.0), occlusion=False
    )
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert "seen" in reloaded.sensor_names
    code = scene.generate_python()
    assert 'scene.add_vision_sensor("seen", camera="cam"' in code
    assert "detect_range=(0.1, 2)" in code and "occlusion=False" in code, code


def test_vision_validation(scene) -> None:
    with pytest.raises(ValueError):
        scene.add_vision_sensor("bad", camera="nope")
    scene.add_camera("cam", position=(1, 0, 0.3))
    with pytest.raises(ValueError):
        scene.add_vision_sensor("bad", camera="cam", detect_range=(0.5, 0.1))
    scene.add_vision_sensor("seen", camera="cam")
    # The camera a sensor looks through cannot be removed out from under it.
    with pytest.raises(ValueError, match="watched by vision sensor"):
        scene.remove_camera("cam")
    scene.remove_sensor("seen")
    scene.remove_camera("cam")
