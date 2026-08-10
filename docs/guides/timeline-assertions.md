# Timeline assertions

A [`SequenceTimeline`][botrail.SequenceTimeline] is everything one cycle did,
queryable. Because the bake is [deterministic](../concepts/determinism.md),
every number it returns can be asserted in CI — this guide is the vocabulary
for doing that.

## The timing chart

```python
tl.duration                     # cycle time, seconds
tl.step_spans                   # [(name, start, end), ...] in execution order

span = tl.step_span("feed")     # one step, assertion-friendly
span.start, span.end, span.duration
```

The one-step view reads the way a spec does:

```python
assert tl.step_span("feed").end <= 2.0          # the part arrives on time
assert tl.step_span("work").duration == pytest.approx(0.5, abs=0.011)
```

(`0.011` = one 10 ms scan tick plus change — the natural tolerance for
anything quantized by the scan.)

## Signal lanes

Every internal signal, sensor, and device running-state is a waveform:

```python
lane = tl.signal("eye")
lane.edges              # [(t, value), ...] starting with (0, initial)
lane.rising_edges()     # times it turns ON (the initial level is not an edge)
lane.falling_edges()
lane.high_spans()       # [(start, end), ...]; an open interval closes at duration
lane.high_total()       # total ON time
lane.value_at(t)
```

Handshakes become one-liners:

```python
assert tl.signal("eye").rising_edges() == [tl.step_span("feed").end]
assert tl.signal("belt").high_spans() == [(0.0, tl.step_span("feed").end)]
assert tl.signal("carrying").high_total() < 10.0
```

## Clearance

```python
clr = tl.min_clearance(dt=0.01)
```

The tightest robot-to-environment approach over the whole cycle, sampled every
`dt` seconds — carried and conveyed objects replay their baked motion.
[`Clearance`][botrail.Clearance] compares and converts like its distance, so:

```python
assert tl.min_clearance() > 0.05
```

and when it fails, the repr names the time and the touching pair. `clr.t` is
when the minimum first happens; `clr.pair` names the touching
`(robot side, obstacle)` **only while in contact**, so `clr.pair is None` is
the "and nothing ever touched" half of a safety check. Robot-*robot* contact
never appears here — it is already a hard error during the bake itself.

## Robot tracks and object motion

```python
tl.sample(t, robot="far")            # joint positions at t
tl.moves("far")                      # [(label, start, end)] — what drove it when
tl.robot_trajectory("far")           # the cycle as a Trajectory (CSV/JSON export;
                                     #   step boundaries land in segment_ends)
tl.object_pose("crate", t)           # where a carried/conveyed part was
tl.object_visible("crate", t)        # False only while stowed in a magazine
```

`object_pose` is how the tracking tutorial measured its 150 mm of belt
travel between latch and grasp.

## Utilization: the line-balancing number

```python
tl.utilization("st1_lh")     # 0..1 — fraction of the cycle it moved
tl.busy_seconds("st1_lh")    # the same in seconds (overlaps merged)
tl.utilizations()            # {robot: utilization} for the whole cell
```

On a line this is the number that decides where work should go: the
bottleneck station is the one whose arms sit highest, and moving a spot off
it is the edit whose effect on takt you can then measure rather than
estimate. `examples/line_balance_sweep.py` does exactly that — bakes the
real line once per weld-schedule split and prints takt, per-station cycle,
and utilization — and `python/tests/test_line_balance.py` pins the result,
which is what makes "changing the layout" a regression test. The studio
shows the same figure beside each robot lane on the timing chart.

A useful invariant to assert alongside the takt: on an indexed line, the
takt is the transfer plus the slowest station. If that stops holding, the
cycle time has stopped meaning what you think it means.

## Golden values vs budgets

Two different assertions, both worth having:

```python
assert tl.duration == pytest.approx(7.45, abs=0.25)   # golden: catches change
assert tl.duration <= 8.0                             # budget: catches regression
```

Per machine, a re-bake is bit-identical — the tolerance on a golden absorbs
libm-level drift *between* machines, not behavior. Size tolerances to what
they must absorb: one scan tick for step timing, a quarter second for a
full-cycle golden. The
[Verify the cell in CI](../tutorials/verify-in-ci.md) tutorial walks a
complete suite built this way.
