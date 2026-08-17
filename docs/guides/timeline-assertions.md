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

## Handshakes, response times and faults

A handshake between two controllers is a signal one program writes and
another waits on; the bake has both ends, so its timing is a plain
assertion. The pattern is "the reader moves within *n* scans of the edge":

```python
tl = scene.simulate_sequences(["st1", "st2", "transfer"])
done = tl.signal("st1_done").rising_edges()
gate = tl.step_span("transfer/p2_gate")
assert done[0] <= gate.end <= done[0] + 0.02        # released within two scans
```

For a robot the controller side has no lane; the timeline synthesizes it:

```python
busy = tl.robot_busy("st1_lh")                       # [(start, end), ...] merged moves
starts = [t for _, t, _ in tl.moves("st1_lh")]
assert busy[0][0] == starts[0]                       # busy rises with the first start
assert all(b - a > 0 for a, b in busy)
```

Two contacts that must never be on together (an interlock) is an
intersection test on their high spans:

```python
def overlap(a, b):
    return [(max(s1, s2), min(e1, e2)) for s1, e1 in a for s2, e2 in b if max(s1, s2) < min(e1, e2)]

assert overlap(tl.signal("near_in_zone").high_spans(), tl.signal("far_in_zone").high_spans()) == []
```

And a fault scenario (see [the I/O map](io-map.md#faults-the-fourth-delta))
turns "what if the wire breaks" into a row of the same table:

```python
scene.add_scenario("beam_open", faults=[bt.io.open("body_at_head")])
runs = scene.simulate_scenarios(["st1", "st2", "transfer"])
assert "forced: body_at_head=false" in runs.errors["beam_open"]     # it stops, and says why
assert "transfer/p1_load" in runs.errors["beam_open"]                # ... at the step that reads it
```

The safe-side assertion is the one *without* an error: an inverted E-stop
wire that still runs is a wiring finding, and the run shows it —
`assert "estop_open" in runs.errors` is the check that the healthy contact
is wired to fail low. `tl.export_handshake_spec(path)` writes the whole
interface as a Markdown sheet, per scenario.

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
