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
HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
# The cell is the Isaac Franka on the factory layout, equipped from the model
# catalog — both have to be cached before this can run.
pytestmark = pytest.mark.skipif(
    not ((CACHE / "assets" / "franka" / "franka.usd").exists()
         and any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))),
    reason="the demo cell needs the Isaac Franka and the botrail catalog "
           "(run examples/demo.py once)",
)

# Baked on the pinned dependency set. The tolerance absorbs libm-level drift
# between machines, not behaviour changes — a replan that adds a detour
# shifts the cycle by far more than this. 2026-08-06: 90.76 → 83.71 when IK
# gained null-space joint centering; the taught 7-DOF configurations moved
# off their limits and the transfer plans shortened. 2026-08-22: 83.71 →
# 84.16 when both pallets moved 160 mm clear of the pedestal they were
# standing inside — every transfer reaches that much further.
GOLDEN_CYCLE = 84.16
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
    for layer in range(demo.CYCLES):
        tag = f"_{layer + 1}"
        for robot, box in (
            (demo.FAR, demo.CARTON[2 * layer]),
            (demo.NEAR, demo.CARTON[2 * layer + 1]),
        ):
            latch = at[f"{robot}_latch{tag}"][0]
            grasp = at[f"{robot}_grasp{tag}"][0]
            travel = (
                timeline.object_pose(box, grasp)[0][0]
                - timeline.object_pose(box, latch)[0][0]
            )
            assert travel > 0.02, f"{robot}{tag} did not pick off a moving belt"


def test_each_carton_lands_on_its_own_course(baked) -> None:
    """The end state, not just the absence of errors. Two things are quiet
    when wrong: an arm that latches before its carton reaches the station
    grasps it off-centre, and a magazine fed on a timer rather than on
    demand hands the arm a carton it was not expecting — both land the box
    a pitch away from the pallet."""
    demo, scene, timeline = baked
    for robot, frame in (
        (demo.FAR, "/World/PalletFar/PlaceFrame"),
        (demo.NEAR, "/World/Pallet/PlaceFrame"),
    ):
        base = scene.frame(frame)[0]
        for layer in range(demo.CYCLES):
            box = demo.CARTON[2 * layer + (0 if robot == demo.FAR else 1)]
            want = (base[0], base[1], base[2] + demo.BOX_SIZE * layer)
            got = timeline.object_pose(box, timeline.duration)[0]
            assert got == pytest.approx(want, abs=0.008), f"{box} (course {layer})"


def test_the_belt_recirculates_its_cleats(baked) -> None:
    """The cleats are what make the belt read as moving. They ride the same
    source/sink loop as the cartons, with collision off so the arms reach
    straight through where they pass."""
    demo, scene, timeline = baked
    for name in demo.CLEAT:
        xs = [
            timeline.object_pose(name, i * 0.2)[0][0]
            for i in range(int(timeline.duration / 0.2))
        ]
        laps = sum(1 for a, b in zip(xs, xs[1:]) if b < a - 0.5)
        assert laps >= 1, f"{name} never went round"
    assert all(scene.obstacle_color(n) is not None for n in demo.CLEAT)


def test_the_magazine_only_feeds_what_is_asked_for(baked) -> None:
    """An indexing feeder, not a timer: the spares stay in the magazine.
    If they leaked onto the belt, the carton each step names would drift."""
    demo, _scene, timeline = baked
    for name in demo.CARTON[2 * demo.CYCLES :]:
        assert not any(
            timeline.object_visible(name, i * timeline.duration / 50)
            for i in range(51)
        ), f"{name} left the magazine"


def test_cartons_enter_the_cell_by_travelling(baked) -> None:
    """The artifact this exists to kill, in its second form: a carton must
    never *appear* inside the cell. The belt runs out through an opening in
    the west guard and carriers are released beyond it, so the first frame
    any carton is drawn on, it is still outside."""
    demo, _scene, timeline = baked
    guard_x = -2.0
    for name in demo.CARTON:
        first = None
        for i in range(int(timeline.duration / 0.05)):
            t = i * 0.05
            if timeline.object_visible(name, t):
                first = timeline.object_pose(name, t)[0][0]
                break
        if first is None:
            continue  # never called for; stays in the magazine
        assert first < guard_x, f"{name} appeared inside the cell at x={first:.2f}"


def test_stock_is_never_drawn_before_it_is_fed(baked) -> None:
    """The artifact this exists to kill: a run must not open on a pile of
    stock that then teleports onto the belt. Every carrier is stowed at
    t = 0 and only appears once the feeder calls for it."""
    demo, _scene, timeline = baked
    for name in demo.CARTON:
        assert not timeline.object_visible(name, 0.0), f"{name} visible at t=0"
    # The cleats are the exception, and deliberately so: they start spread
    # along the belt because a belt is not empty of its own slats.
    assert all(timeline.object_visible(c, 0.0) for c in demo.CLEAT)
    # ...and the ones that are used do become visible.
    for layer in range(demo.CYCLES):
        for box in (demo.CARTON[2 * layer], demo.CARTON[2 * layer + 1]):
            assert timeline.object_visible(box, timeline.duration), box


def test_the_interlock_is_what_releases_the_second_arm(baked) -> None:
    demo, _scene, timeline = baked
    at = spans(timeline)
    edges = dict(timeline.signals)["zone_far"]
    cleared = [t for t, occupied in edges if not occupied and t > 0.0]
    assert len(cleared) >= demo.CYCLES, "the far arm never left the airspace"
    # Every cycle, the near arm moves in on that edge — not on a timer.
    for layer in range(demo.CYCLES):
        assert at[f"near_approach_{layer + 1}"][0] == pytest.approx(
            cleared[layer], abs=0.05
        )


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


def test_the_interlock_keeps_the_arms_out_of_each_others_airspace(baked) -> None:
    """The property that matters, stated directly: never both over the
    station at once.

    Dropping the interlock does not reliably crash the cell — with this
    timing the two transfers happen to miss — so asserting "it collides"
    would be asserting a coincidence. Asserting the *separation* is the
    real invariant, and it is the one that fails the moment the interlock
    goes.

    The unguarded run is the control: it has to violate the property, or
    the guarded zero would prove nothing. *How much* it violates it by is
    the coincidence (0.13 s as the cell stands, seconds when the transfers
    line up differently, and it moves whenever the layout does), so what is
    asserted is only that the overlap is real — several rollout ticks, not
    a boundary artefact."""
    demo, _scene, timeline = baked
    assert demo.shared_airspace(timeline) == 0.0

    scene = demo.build_cell()
    unguarded = scene.simulate_sequence(demo.build_cycle(scene, interlocked=False))
    assert demo.shared_airspace(unguarded) > 0.05


def test_converging_arms_are_caught_by_the_rollout(baked) -> None:
    """The guard that does not depend on timing: give the two arms one
    reason to converge and the tick check names the moment and the links."""
    import dual_cell_demo as demo

    scene = demo.build_cell()
    demo.build_cycle(scene)
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
