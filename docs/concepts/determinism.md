# Determinism

`simulate_sequence()` is deterministic: the same scene and sequence produce a
**bit-identical** timeline every run. Not statistically similar — equal, down
to every sampled joint value. This page is about what exactly is promised,
why it holds, and what it buys.

## The promise, precisely

Within one environment (same botrail build, same machine):

```python
a = scene.simulate_sequence("cycle")
b = scene.simulate_sequence("cycle")

assert a.duration == b.duration          # ==, not approx
assert a.step_spans == b.step_spans
assert a.signals == b.signals
assert a.sample(1.5) == b.sample(1.5)
```

botrail's own test suite asserts exactly this, and planning has the same
property: an unseeded `plan()` twice from the same state returns the same
trajectory (`seed` selects a *different* deterministic exploration, not
reproducibility — that is already there).

**Across machines**, the promise weakens by exactly one thing: low-level math
libraries differ, so golden values in a shared CI carry a small tolerance —

```python
assert tl.duration == pytest.approx(GOLDEN_CYCLE, abs=0.25)
```

— sized to absorb libm-level drift, **not** behavior changes. A replan that
takes a detour moves the cycle by far more than the tolerance, so the test
still catches everything you care about.

## Why it holds

Determinism is not an implementation nicety; it is bought with three design
decisions, each visible elsewhere in the docs:

1. **No physics engine.** Geometry, kinematics, and a discrete scan — no
   contact solver, no integrator with its stability noise. This is the
   headline [non-goal](why-botrail.md#what-botrail-does-not-claim): botrail
   answers *reach, clearance, seconds*, and in exchange every answer is
   exact.
2. **The PLC scan.** Sensors, devices, transitions, and tracking all advance
   on a fixed tick (`dt=0.01`). Time is a grid, not a race; the timeline is
   quantized, and one tick (`abs=0.011`) is the natural tolerance for any
   step-timing assertion.
3. **A deterministically seeded planner.** RRT-Connect is sampling-based, but
   the samples come from a deterministic sequence — and motions plan at their
   step, against a defined snapshot of the world, so the planner's input is
   as reproducible as its randomness.

## What it buys

Determinism is the load-bearing property under every workflow this
documentation teaches:

* **Numbers you can assert.** `tl.duration <= 8.0` is a real test only if
  re-running cannot flake. The whole
  [assertion vocabulary](../guides/timeline-assertions.md) rests here.
* **Diffs that mean something.** Change the layout, re-bake: every difference
  in the timeline is *caused by your edit*, not by simulation noise. That is
  what makes a [cycle-time regression test](../tutorials/verify-in-ci.md)
  reviewable — the failing assertion is the cost of the change.
* **Sweeps that are studies.** A [parameter table](../tutorials/parameter-sweep.md)
  re-prints digit for digit; rows are facts about the cell, not samples from
  a distribution.
* **Recordings that are evidence.** The exported USD replays the exact
  timeline that passed the tests — what you watched is what you verified.

The one-sentence version: *because the bake cannot disagree with itself, the
cell's numbers can be treated like code.*
