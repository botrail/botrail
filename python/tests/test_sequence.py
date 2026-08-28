import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    tcp, _ = scene.link_pose(scene.robot.tcp_link)
    scene.add_box("held", (0.04, 0.04, 0.04), (tcp[0], tcp[1], tcp[2] + 0.06))
    return scene


def _teach_motion(scene: bt.Scene, name: str, goal: list) -> None:
    scene.add_segment(name, goal=goal)


def test_pick_cycle_rollout(scene: bt.Scene) -> None:
    start = list(scene.joint_positions)
    _teach_motion(scene, "swing", [1.2, -0.5, 0.6, 0.0, 0.3, 0.0])
    _teach_motion(scene, "home", start)
    scene.define_signal("carrying")

    sq = scene.sequence("cycle")
    sq.step("grasp", actions=[bt.seq.attach("held"), bt.seq.set_signal("carrying")])
    sq.step("swing", actions=[bt.seq.motion("swing")])
    sq.step("drop", actions=[bt.seq.detach("held"), bt.seq.set_signal("carrying", False)])
    sq.step("settle", transition=bt.seq.elapsed(0.3))
    sq.step("home", actions=[bt.seq.motion("home")])
    assert scene.sequence_names == ["cycle"]

    tl = sq.simulate()
    names = [name for name, _, _ in tl.step_spans]
    assert names == ["grasp", "swing", "drop", "settle", "home"]
    # The settle wait is a timer step (quantized up to one scan tick).
    settle = tl.step_spans[3]
    assert 0.3 - 1e-9 <= settle[2] - settle[1] <= 0.31 + 1e-9
    # The held box rides the arm during the swing and freezes at release.
    _, swing_start, swing_end = tl.step_spans[1]
    p0, _ = tl.object_pose("held", swing_start)
    p1, _ = tl.object_pose("held", swing_end)
    assert math.dist(p0, p1) > 0.05
    p_final, _ = tl.object_pose("held", tl.duration)
    assert math.dist(p1, p_final) < 1e-9
    # The arm comes home; the cycle time covers everything.
    assert all(abs(a - b) < 1e-6 for a, b in zip(tl.sample(tl.duration), start))
    assert tl.duration > swing_end + 0.3
    # The carrying signal traces grasp -> drop.
    signals = dict(tl.signals)
    edges = signals["carrying"]
    assert [v for _, v in edges] == [False, True, False]
    # The live scene is untouched by the rollout.
    assert scene.attachments == []
    assert list(scene.joint_positions) == start


def test_sequence_timeline_export_usd(scene: bt.Scene, tmp_path: Path) -> None:
    _teach_motion(scene, "swing", [0.5, 0.0, 0.0, 0.0, 0.0, 0.0])
    sq = scene.sequence("mini")
    sq.step("grasp", actions=[bt.seq.attach("held")])
    sq.step("swing", actions=[bt.seq.motion("swing")])
    tl = sq.simulate()

    out = tmp_path / "cycle.usda"
    warnings = tl.export_usd(out, fps=30.0)
    assert warnings == []
    text = out.read_text()
    assert text.startswith("#usda")
    assert "timeSamples" in text and "held" in text

    # The trajectory view exposes the joint track with step boundaries.
    traj = tl.trajectory
    assert traj.duration == tl.duration
    assert len(traj.segment_ends) == 2


def test_sequence_validation_errors(scene: bt.Scene) -> None:
    sq = scene.sequence("bad")
    sq.step("x", actions=[bt.seq.motion("nope")])
    with pytest.raises(ValueError, match="unknown motion"):
        sq.simulate()
    with pytest.raises(ValueError, match="unknown sequence"):
        scene.simulate_sequence("ghost")
    # Unknown signals are caught at simulate time too.
    sq2 = scene.sequence("sig")
    sq2.step("wait", transition=bt.seq.signal("undeclared"))
    with pytest.raises(ValueError, match="unknown signal"):
        sq2.simulate()
