# Timeline

The baked result of a sequence: what every robot did, when every step ran, how
every signal moved, and where every object was — for one cycle.

```python
tl = scene.simulate_sequence("cycle")

tl.duration                      # cycle time, seconds
tl.step_span("feed").duration    # one step's baked interval
tl.signal("eye").rising_edges()  # when the beam broke
tl.min_clearance()               # tightest approach over the whole cycle
tl.export_usd("cycle.usda", fps=60)
```

The bake is deterministic — the same scene produces a bit-identical timeline
every run — which is what makes these values usable as regression assertions.

::: botrail.SequenceTimeline

## Span

Returned by [`SequenceTimeline.step_span`][botrail.SequenceTimeline.step_span]:
one step's interval, in a form that reads well in an assertion
(`assert tl.step_span("feed").end <= 2.0`).

::: botrail.Span

## SignalTrack

Returned by [`SequenceTimeline.signal`][botrail.SequenceTimeline.signal]. One
boolean waveform lane — an internal signal, a sensor, or a device's running
state — with edge and duty queries on top of it.

::: botrail.SignalTrack

## Clearance

Returned by
[`SequenceTimeline.min_clearance`][botrail.SequenceTimeline.min_clearance]: the
tightest robot-to-environment approach over the cycle, with the time and the
pair it happened at. It compares against plain floats, so
`assert tl.min_clearance() > 0.05` works directly.

::: botrail.Clearance
