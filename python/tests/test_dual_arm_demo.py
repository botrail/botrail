"""The dual-arm kitting example, asserted the way its owner would.

`examples/multi_robot/dual_arm_demo.py` is the shipped demonstration of one
robot with two arms: two programs share one robot an arm each, contend for
a bin under a zone interlock, hand a part across, and — on the catalog rig
— carry the tray with both hands. This pins what makes that cell *right*:
both arms busy at once, a bin that is never entered by both, a collision the moment the interlock goes, one 6-axis program per arm, and
the deliverables naming the arms.

The offline rig (two primitive arms from the checkout) runs the kitting;
the hand-off and the two-handed carry, which its arms cannot reach, are
checked on the UR5e rig when the catalog is cached.
"""

import os
import sys
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "multi_robot"))

HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
CATALOG_CACHED = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))


@pytest.fixture(scope="module")
def kitting():
    import dual_arm_demo as demo

    scene, rig, programs = demo.build("simple")
    return demo, scene, rig, demo.simulate(scene, programs)


def spans(timeline) -> dict:
    return {step: (start, end) for step, start, end in timeline.step_spans}


def occupied(timeline, signal: str) -> list[tuple[float, float]]:
    out, start = [], None
    for t, on in dict(timeline.signals)[signal]:
        if on and start is None:
            start = t
        elif not on and start is not None:
            out.append((start, t))
            start = None
    if start is not None:
        out.append((start, timeline.duration))
    return out


def test_one_robot_two_arms_bakes_one_deterministic_cycle(kitting) -> None:
    demo, scene, rig, timeline = kitting
    robot = scene.robots[0]
    assert scene.robots == [robot] and rig.robot.groups == ["left", "right"]
    assert timeline.robots == [robot]
    # Both arms drive: each has moves of its own, and they overlap in time
    # — what the second arm bought.
    for arm in ("left", "right"):
        assert timeline.moves(robot, group=arm)
        assert 0 < timeline.utilization(robot, group=arm) < 1
    assert demo.overlap_seconds(timeline, robot) > 2.0
    # The robot's own busy time merges both arms.
    assert timeline.busy_seconds(robot) >= max(
        timeline.busy_seconds(robot, group=arm) for arm in ("left", "right")
    )
    # Determinism is the premise: same cell, same numbers.
    scene2, _, programs2 = demo.build("simple")
    again = demo.simulate(scene2, programs2)
    assert again.duration == timeline.duration
    assert again.robot_trajectory(robot).positions == timeline.robot_trajectory(robot).positions


def test_every_part_lands_in_the_bin(kitting) -> None:
    demo, _scene, rig, timeline = kitting
    for part in demo.PARTS:
        assert demo.in_bin(rig, timeline.object_pose(part, timeline.duration)[0]), part
    # The bin fills from both sides, the left arm's parts and the right's.
    at = spans(timeline)
    assert at["left/left release L1"][0] < at["left/left release L2"][0]
    assert at["right/right release R1"][0] < at["right/right release R2"][0]


def test_the_bin_is_never_entered_by_both_arms(kitting) -> None:
    _demo, _scene, _rig, timeline = kitting
    left, right = occupied(timeline, "zone_left"), occupied(timeline, "zone_right")
    assert left and right, "both arms use the bin"
    shared = sum(
        max(0.0, min(a1, b1) - max(a0, b0)) for a0, a1 in left for b0, b1 in right
    )
    assert shared == 0.0


def test_without_the_interlock_the_arms_collide() -> None:
    import dual_arm_demo as demo

    scene, _rig, programs = demo.build("simple", clash=True)
    with pytest.raises(ValueError, match="collide at t = ") as caught:
        demo.simulate(scene, programs)
    message = str(caught.value)
    assert "`left`" in message and "`right`" in message


def test_each_arm_exports_its_own_controller_program(kitting, tmp_path: Path) -> None:
    demo, _scene, rig, timeline = kitting
    for arm, io in (("left", demo.LEFT_IO), ("right", demo.RIGHT_IO)):
        path = tmp_path / f"{arm}.script"
        timeline.export_script(path, sequence=arm, group=arm, **io)
        code = path.read_text()
        assert code.startswith(f"def {arm}():")
        joints = rig.robot.group(arm).joints
        assert f"# joints: {', '.join(joints)}" in code
        moves = [line for line in code.splitlines() if "movej(" in line]
        assert moves and all(line.count(",") >= 5 for line in moves)
        # The other arm's moves are not in this program: only this arm's
        # joints appear, six of them per move.
        first = moves[0][moves[0].index("[") + 1 : moves[0].index("]")]
        assert len(first.split(",")) == 6
    with pytest.raises(ValueError, match="several arms"):
        timeline.to_script(sequence="left", **demo.LEFT_IO)


def test_the_deliverables_name_the_arms(kitting) -> None:
    _demo, scene, _rig, timeline = kitting
    robot = scene.robots[0]
    report = scene.cell_report(timeline, clearance_dt=None)
    md = report.to_markdown()
    assert f"| {robot}/left |" in md and f"| {robot}/right |" in md
    rows = report.to_dict()["cycles"][0]["robots"] if hasattr(report, "to_dict") else None
    if rows is not None:
        assert [r.get("group") for r in rows] == [None, "left", "right"]
    interlocks = scene.interlocks().to_markdown()
    assert f"({robot}/left)" in interlocks and f"({robot}/right)" in interlocks
    assert "IDLE(" not in interlocks  # two programs: they wait on zones and flags, not on each other
    req = bt.select.requirements(scene)
    by_line = {row.target: {r.key: r for r in row.requirements} for row in req.rows}
    for arm in ("left", "right"):
        line = by_line[f"{robot}/{arm}"]
        assert "reach_mm" in line
        assert f"{arm} arm's base" in line["reach_mm"].basis
    # The layout draws (nothing, here: the primitive arms quote no reach)
    # without tripping over the two arms.
    assert "<svg" in scene.layout("svg")


@pytest.mark.skipif(not CATALOG_CACHED, reason="the UR5e rig needs the botrail catalog cached")
def test_the_ur5e_rig_carries_the_tray_with_both_hands() -> None:
    import dual_arm_demo as demo

    scene, rig, programs = demo.build("ur5e", carry=True)
    if rig.kind != "ur5e":
        pytest.skip("the catalog UR5e could not be fetched")
    timeline = demo.simulate(scene, programs)
    at = spans(timeline)
    # One part changes hands: the left arm sets L2 down at the hand-off spot,
    # the right arm takes it from there — in between it rests, held by nobody.
    released = at["left/left release L2"][1]
    taken = at["right/right grasp H"][0]
    assert released < taken
    x, y, z = timeline.object_pose("L2", (released + taken) / 2)[0]
    assert (x, y) == pytest.approx(rig.handoff, abs=0.01)
    top = demo.TRAY_TOP + rig.handoff_rise
    assert top < z < top + demo.PART
    assert demo.in_bin(rig, timeline.object_pose("L2", timeline.duration)[0])
    # The tray ends up where the left arm carried it…
    (x, y, _), _ = timeline.object_pose("tray", timeline.duration)
    assert x == pytest.approx(rig.tray[0] + rig.carry, abs=0.01)
    assert y == pytest.approx(rig.tray[1], abs=0.01)
    # …and the right hand kept its hold on the way: its distance to the
    # tray is constant while the tray moves.
    start, end = at["left/left carry"]
    tip = rig.robot.group("right").tip

    def gap(t: float) -> float:
        scene.set_joint_positions(timeline.sample(t))
        hand = scene.link_pose(tip)[0]
        tray = timeline.object_pose("tray", t)[0]
        return sum((a - b) ** 2 for a, b in zip(hand, tray)) ** 0.5

    gap0 = gap(start)
    assert all(abs(gap(start + (end - start) * k / 6) - gap0) < 0.003 for k in range(1, 7))
    # The layout draws a reach circle per arm, from the UR5e's catalog spec.
    svg = scene.layout("svg")
    reach = svg.split('class="reach"')[1].split("</g>")[0]
    assert reach.count("<circle") == 2
