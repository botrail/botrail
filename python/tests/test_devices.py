import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


def test_conveyor_feed_sensor_stop_cycle(scene: bt.Scene) -> None:
    # A box rides a conveyor along +x (well away from the arm) until it
    # trips a beam; the sequence then stops the belt.
    scene.add_box("crate", (0.04, 0.04, 0.04), (-0.5, 0.6, 0.3))
    scene.add_conveyor(
        "belt",
        zone_position=(-0.2, 0.6, 0.3),
        zone_size=(1.2, 0.3, 0.3),
        velocity=(0.25, 0.0, 0.0),
        running=False,
    )
    scene.add_beam_sensor("eye", frm=(0.0, 0.4, 0.3), to=(0.0, 0.8, 0.3))
    assert scene.sensor_names == ["eye"]
    assert scene.device_names == ["belt"]

    sq = scene.sequence("feed")
    sq.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("eye"))
    sq.step("stop", actions=[bt.seq.stop("belt")], transition=bt.seq.elapsed(0.1))
    tl = sq.simulate()

    # Analytic trip time: 0.475 m at 0.25 m/s.
    feed_end = tl.step_spans[0][2]
    assert abs(feed_end - 1.9) <= 0.011
    lanes = dict(tl.signals)
    assert [v for _, v in lanes["eye"]] == [False, True]
    assert [v for _, v in lanes["belt"]] == [False, True, False]
    # The crate travelled with the belt and settles once stopped.
    p_end, _ = tl.object_pose("crate", tl.duration)
    assert abs((p_end[0] - (-0.5)) - 0.25 * feed_end) < 1e-9
    # The live scene is untouched.
    pos, _ = scene.obstacle_pose("crate")
    assert pos[0] == -0.5


def test_linear_axis_and_project_roundtrip(scene: bt.Scene, tmp_path: Path) -> None:
    scene.add_box("door", (0.1, 0.1, 0.1), (0.6, 0.0, 0.2))
    scene.add_linear_axis(
        "lift", objects=["door"], axis=(0, 0, 1), speed=0.5, range=(0.0, 0.4)
    )
    sq = scene.sequence("open")
    sq.step(
        "raise",
        actions=[bt.seq.move_to("lift", 0.3)],
        transition=bt.seq.device_done("lift"),
    )
    tl = sq.simulate()
    assert abs(tl.step_spans[0][2] - 0.6) <= 0.011
    p_end, _ = tl.object_pose("door", tl.duration)
    assert abs(p_end[2] - 0.5) < 1e-9

    # Sensors/devices round-trip through the project file and codegen.
    scene.add_zone_sensor("mat", position=(0.5, 0.0, 0.1), size=(0.4, 0.4, 0.2))
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert reloaded.sensor_names == ["mat"]
    assert reloaded.device_names == ["lift"]
    code = scene.generate_python()
    for needle in (
        'scene.add_zone_sensor("mat"',
        'scene.add_linear_axis("lift"',
        'bt.seq.move_to("lift", 0.3)',
        'bt.seq.device_done("lift")',
    ):
        assert needle in code, f"missing {needle}:\n{code}"


def test_tracking_pick_follows_a_moving_part(scene: bt.Scene) -> None:
    # Conveyor tracking: the taught grasp is met while the belt keeps
    # running, and grasping the part freezes the offset so the lift after it
    # does not drag the part back to where it was taught.
    scene.set_joint_positions([0.0, -0.4, 0.9, 0.0, 0.9, 0.0])
    tcp = scene.robot.tcp_link
    start, _ = scene.link_pose(tcp)
    scene.add_box("crate", (0.04, 0.04, 0.04), (start[0], start[1], start[2] - 0.1))
    scene.add_conveyor(
        "belt",
        zone_position=(start[0] + 0.5, start[1], start[2] - 0.1),
        zone_size=(1.4, 0.3, 0.3),
        velocity=(0.1, 0.0, 0.0),
        running=True,
    )

    sq = scene.sequence("track")
    sq.step("latch", actions=[bt.seq.track("crate")])
    sq.step("follow", transition=bt.seq.elapsed(0.5))
    sq.step("grasp", actions=[bt.seq.attach("crate", link=tcp)])
    sq.step("hold", transition=bt.seq.elapsed(0.4))
    sq.step("release", actions=[bt.seq.untrack()])
    tl = sq.simulate()

    # The tool held station over the part for the whole tracked stretch.
    for t in (0.0, 0.25, 0.5):
        scene.set_joint_positions(tl.sample(t))
        tool, _ = scene.link_pose(tcp)
        part, _ = tl.object_pose("crate", t)
        assert abs((tool[0] - part[0])) < 1e-4, t
        assert abs(part[0] - (start[0] + 0.1 * t)) < 1e-9, t

    # Grasping the tracked part freezes the sync: the belt is still running,
    # but the part is the robot's now and the arm stops chasing it.
    caught, _ = tl.object_pose("crate", 0.5)
    end, _ = tl.object_pose("crate", tl.duration)
    assert tl.duration >= 0.9
    assert max(abs(a - b) for a, b in zip(end, caught)) < 1e-9
    scene.set_joint_positions(tl.sample(tl.duration))
    tool, _ = scene.link_pose(tcp)
    assert abs(tool[0] - end[0]) < 1e-4  # still in the gripper


def test_tracking_rules_are_reported(scene: bt.Scene) -> None:
    scene.add_box("crate", (0.04, 0.04, 0.04), (0.4, 0.0, 0.2))
    scene.add_segment("go", goal=[0.2, -0.4, 0.9, 0.0, 0.9, 0.0])
    sq = scene.sequence("bad")
    sq.step("latch", actions=[bt.seq.track("crate")])
    sq.step("move", actions=[bt.seq.motion("go")])
    with pytest.raises(ValueError, match="release the track first"):
        sq.simulate()
