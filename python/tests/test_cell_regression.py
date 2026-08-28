"""The cell regression test — the operating form of "決定的に焼ける".

This file is the pattern a botrail user copies into their own CI: author
the cell once, bake it, and assert the numbers that matter (cycle time,
step deadlines, signal handshakes, clearance). A layout edit that changes
the cycle then fails a test instead of surprising the shop floor — the
workflow DESIGN.md §8 names as the success condition at cell granularity,
run here against botrail's own repository.

The cell: a crate rides a conveyor into a photoelectric beam, the belt
stops, the arm approaches, works, and comes home. Everything is primitive
geometry from the repo checkout — no downloads.
"""

from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# Baked on the pinned dependency set; the tolerance absorbs libm-level
# drift between machines, not behavior changes (a replan that adds a
# detour shifts the cycle by far more than 0.25 s).
GOLDEN_CYCLE = 7.45
CYCLE_BUDGET = 8.0


def build_cell(beam_x: float = 0.0) -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("crate", (0.04, 0.04, 0.04), (-0.5, 0.6, 0.3))
    scene.add_conveyor(
        "belt",
        zone_position=(-0.2, 0.6, 0.3),
        zone_size=(1.2, 0.3, 0.3),
        velocity=(0.25, 0.0, 0.0),
        running=False,
    )
    scene.add_beam_sensor("eye", frm=(beam_x, 0.4, 0.3), to=(beam_x, 0.8, 0.3))
    scene.define_signal("present")
    scene.add_segment("approach", goal=[0.6, -0.5, 0.8, 0.0, 0.4, 0.0])
    scene.add_segment("home", goal=HOME)

    sq = scene.sequence("cycle")
    sq.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("eye"))
    sq.step("stop", actions=[bt.seq.stop("belt"), bt.seq.set_signal("present")])
    sq.step("approach", actions=[bt.seq.motion("approach")])
    sq.step("work", transition=bt.seq.elapsed(0.5))
    sq.step(
        "home",
        actions=[bt.seq.motion("home"), bt.seq.set_signal("present", False)],
    )
    return scene


def test_cell_cycle_regression() -> None:
    tl = build_cell().simulate_sequence("cycle")

    # The cycle and its budget.
    assert tl.duration == pytest.approx(GOLDEN_CYCLE, abs=0.25)
    assert tl.duration <= CYCLE_BUDGET

    # The process happened in order.
    assert [name for name, _, _ in tl.step_spans] == [
        "feed",
        "stop",
        "approach",
        "work",
        "home",
    ]

    # The beam trips at the analytic time: 0.475 m of travel at 0.25 m/s,
    # quantized up to one 10 ms scan tick.
    feed = tl.step_span("feed")
    assert tl.signal("eye").rising_edges() == [feed.end]
    assert feed.end == pytest.approx(1.9, abs=0.011)

    # Handshakes: the belt runs exactly through feed; `present` covers
    # stop → work and is clear again by the end of the cycle.
    assert tl.signal("belt").high_spans() == [(0.0, feed.end)]
    assert tl.signal("present").high_spans() == [
        (tl.step_span("stop").start, tl.step_span("home").start)
    ]

    # The work dwell holds its spec.
    assert tl.step_span("work").duration == pytest.approx(0.5, abs=0.011)

    # The swing keeps its distance from the stopped crate for the whole
    # cycle — a measure the rollout itself never takes.
    clearance = tl.min_clearance()
    assert clearance > 0.3
    assert clearance.pair is None

    # The cycle ends back home.
    assert tl.sample(tl.duration) == pytest.approx(HOME, abs=1e-6)


def test_bake_is_deterministic_in_process() -> None:
    scene = build_cell()
    a = scene.simulate_sequence("cycle")
    b = scene.simulate_sequence("cycle")
    # Bit-identical, not approximately equal.
    assert a.duration == b.duration
    assert a.step_spans == b.step_spans
    assert a.signals == b.signals
    for t in (0.0, 1.5, 3.0, a.duration):
        assert a.sample(t) == b.sample(t)


def test_layout_change_shifts_the_cycle_deterministically() -> None:
    # Move the beam 0.25 m downstream: the crate needs exactly one more
    # second at 0.25 m/s, and nothing else about the cell changes. This is
    # the §8 workflow in miniature — a layout edit shows up as a cycle-time
    # diff a test can catch.
    base = build_cell().simulate_sequence("cycle")
    moved = build_cell(beam_x=0.25).simulate_sequence("cycle")
    assert moved.duration - base.duration == pytest.approx(1.0, abs=0.021)
    assert moved.step_span("feed").end - base.step_span("feed").end == pytest.approx(
        1.0, abs=0.021
    )
