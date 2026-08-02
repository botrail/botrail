"""The two-arm example, asserted the way a cell owner would assert it.

`examples/dual_cell_demo.py` is the shipped demonstration of a multi-robot
cell: two Frankas sharing one infeed, arbitrated by a zone interlock. This
pins the properties that make it a *correct* cell rather than merely one
that runs — cartons taken off a moving belt, landed on the right pallet, the
interlock actually gating the second arm, and the two failure modes that
appear when the interlock is dropped.

Skipped unless the Isaac Franka has been fetched (the example downloads it
on first run; see `examples/demo.py`).
"""

import os
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

CACHE = Path(os.environ.get("BOTRAIL_CACHE_DIR") or Path.home() / ".cache" / "botrail")
pytestmark = pytest.mark.skipif(
    not (CACHE / "assets" / "franka" / "franka.usd").exists(),
    reason="Isaac Franka not in the botrail cache (run examples/demo.py once)",
)

# Baked on the pinned dependency set. The tolerance absorbs libm-level drift
# between machines, not behaviour changes — a replan that adds a detour
# shifts the cycle by far more than this.
GOLDEN_CYCLE = 42.82
CYCLE_BUDGET = 1.0


@pytest.fixture(scope="module")
def baked():
    import dual_cell_demo as demo

    scene = demo.build_cell()
    name = demo.build_cycle(scene)
    return demo, scene, scene.simulate_sequence(name)


def spans(timeline) -> dict:
    return {step: (start, end) for step, start, end in timeline.step_spans}


def test_two_arms_bake_one_deterministic_cycle(baked) -> None:
    demo, scene, timeline = baked
    assert timeline.robots == [demo.NEAR, demo.FAR]
    assert timeline.duration == pytest.approx(GOLDEN_CYCLE, abs=CYCLE_BUDGET)

    # Determinism is the whole premise: same scene, same numbers.
    again = scene.simulate_sequence("dual_pick")
    assert again.duration == timeline.duration
    for robot in timeline.robots:
        assert (
            again.robot_trajectory(robot).positions
            == timeline.robot_trajectory(robot).positions
        )


def test_each_arm_picks_off_a_moving_belt(baked) -> None:
    """The belt never stops: both arms track their carton in. If tracking
    regressed to a stop-and-pick, the travel would be zero."""
    demo, _scene, timeline = baked
    at = spans(timeline)
    for robot, box in ((demo.FAR, demo.BOX_FAR), (demo.NEAR, demo.BOX_NEAR)):
        latch = at[f"{robot}_latch"][0]
        grasp = at[f"{robot}_grasp"][0]
        travel = (
            timeline.object_pose(box, grasp)[0][0]
            - timeline.object_pose(box, latch)[0][0]
        )
        assert travel > 0.02, f"{robot} did not pick off a moving belt"


def test_each_carton_lands_on_its_own_pallet(baked) -> None:
    """The end state, not just the absence of errors. Getting this wrong is
    quiet: an arm that latches before its carton reaches the station grasps
    it off-centre and misses the pallet by that much."""
    demo, scene, timeline = baked
    targets = {
        demo.BOX_FAR: scene.frame("/World/PalletFar/PlaceFrame")[0],
        demo.BOX_NEAR: scene.frame("/World/Pallet/PlaceFrame")[0],
    }
    for box, want in targets.items():
        got = timeline.object_pose(box, timeline.duration)[0]
        assert got == pytest.approx(want, abs=0.005), box


def test_the_interlock_is_what_releases_the_second_arm(baked) -> None:
    demo, _scene, timeline = baked
    at = spans(timeline)
    edges = dict(timeline.signals)["zone_far"]
    cleared = [t for t, occupied in edges if not occupied and t > 0.0]
    assert cleared, "the far arm never left the contested airspace"
    # The near arm moves in on that edge, not on a timer.
    assert at["near_approach"][0] == pytest.approx(cleared[0], abs=0.05)


def test_both_arms_are_in_motion_at_once(baked) -> None:
    """The reason for the second arm. If the cycle serialised, this is the
    assertion that would notice."""
    _demo, _scene, timeline = baked
    moves = {r: timeline.moves(r) for r in timeline.robots}
    overlap, t, step = 0.0, 0.0, 0.02
    while t < timeline.duration:
        if all(any(a <= t <= b for _, a, b in moves[r]) for r in timeline.robots):
            overlap += step
        t += step
    assert overlap > 3.0, f"only {overlap:.2f}s of concurrency"


def test_without_the_interlock_the_cell_is_rejected(baked) -> None:
    """Two independent guards, and the demo shows both: planning refuses a
    station the other arm occupies, and the rollout catches two individually
    valid plans converging."""
    import dual_cell_demo as demo

    scene = demo.build_cell()
    name = demo.build_cycle(scene, interlocked=False)

    with pytest.raises(ValueError, match="planning failed"):
        scene.simulate_sequence(name)

    with pytest.raises(ValueError, match=r"collide at t = .*interlock"):
        scene.simulate_sequence("clash")


def test_exports_usd_with_both_arms(baked, tmp_path: Path) -> None:
    _demo, _scene, timeline = baked
    out = tmp_path / "dual.usda"
    assert timeline.export_usd(out, fps=30.0) == []
    text = out.read_text()
    assert 'upAxis = "Z"' in text
    # One animated prim per instance, named after it.
    for name in ("near", "far"):
        assert f'def Xform "{name}"' in text, name
    assert "timeSamples" in text
