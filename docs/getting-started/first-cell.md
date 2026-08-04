# Your first cell

The [Quickstart](quickstart.md) ended with a robot that can plan a motion. A
*cell* is more than that: the environment moves on its own, sensors fire, and
the process is a sequence of steps. This page builds a small one, bakes it into
a timeline, and turns the result into a test.

It uses the same `arm.urdf` from the Quickstart and takes about ten minutes.

## The cell

A crate rides a conveyor toward the robot. A photoelectric beam across the belt
detects it, the belt stops, the robot moves in, works for half a second, and
goes home.

```python title="cell.py"
import botrail as bt


def build_cell(velocity: float = 0.25, lane_y: float = 0.6) -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf("arm.urdf"))

    # The part, and the belt that carries it.
    scene.add_box("crate", size=(0.04, 0.04, 0.04), position=(-0.5, lane_y, 0.3))
    scene.add_conveyor(
        "belt",
        zone_position=(-0.2, lane_y, 0.3),
        zone_size=(1.2, 0.3, 0.3),
        velocity=(velocity, 0.0, 0.0),
        running=False,
    )

    # A beam across the lane at x = 0: ON while something interrupts it.
    scene.add_beam_sensor(
        "eye", frm=(0.0, lane_y - 0.2, 0.3), to=(0.0, lane_y + 0.2, 0.3)
    )

    # Two motions, each a named list of waypoint segments.
    scene.add_segment("approach", goal=[0.6, -0.5, 0.8, 0.4])
    scene.add_segment("home", goal=[0.0, 0.0, 0.0, 0.0])

    # The process, written the way a PLC writes one.
    sq = scene.sequence("cycle")
    sq.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("eye"))
    sq.step("stop", actions=[bt.seq.stop("belt")])
    sq.step("approach", actions=[bt.seq.motion("approach")])
    sq.step("work", transition=bt.seq.elapsed(0.5))
    sq.step("home", actions=[bt.seq.motion("home")])

    return scene
```

Three things are worth naming.

**The conveyor is a zone, not a belt mesh.** Any unattached obstacle whose
origin is inside `zone_size` is carried at `velocity` while the device runs. So
the crate moves because it is *in* the zone, not because it was parented to
anything.

**Steps are entry actions plus a transition condition**, evaluated on a fixed
scan cycle — the PLC model. `bt.seq.signal("eye")` holds the `feed` step until
the beam goes high. Omit `transition` and botrail supplies the obvious one: if
the step starts a motion, it waits for that motion (`done()`); if it starts
nothing, it falls through immediately. That is why `stop` below takes zero
time.

**Motions are planned at their step**, against the world as it is at that
moment — with whatever the robot happens to be carrying.

## Bake it

`simulate_sequence` rolls the sequence out and returns a
[`SequenceTimeline`][botrail.SequenceTimeline]:

```python
tl = build_cell().simulate_sequence("cycle")

print(f"cycle time: {tl.duration:.2f} s")
for name, start, end in tl.step_spans:
    print(f"  {name:9s} {start:5.2f} -> {end:5.2f}")
```

```text
cycle time: 7.44 s
  feed       0.00 ->  1.90
  stop       1.90 ->  1.90
  approach   1.90 ->  4.42
  work       4.42 ->  4.92
  home       4.92 ->  7.44
```

Run it again and you get **the same numbers** — not approximately, exactly. The
bake is deterministic: same scene in, bit-identical timeline out. That is the
property everything below rests on.

## Read the cycle

A timeline is queryable. Three accessors are built for assertions:

```python
tl.step_span("feed")      # .start .end .duration of one step
tl.signal("eye")          # the waveform lane for a sensor, signal, or device
tl.min_clearance()        # tightest robot-to-environment approach over the cycle
```

```python
feed = tl.step_span("feed")
print(feed.duration)              # 1.9  — how long the crate took to arrive

print(tl.signal("eye").rising_edges())   # [1.9] — when the beam broke

clr = tl.min_clearance()
print(f"{clr.distance:.3f} m at t={clr.t:.2f}")   # 0.530 m at t=7.41
```

`min_clearance()` compares against `float` directly, so
`assert tl.min_clearance() > 0.05` reads the way you'd say it.

## Change the layout, read the number

The cell is a function of its parameters, so a layout study is a loop:

```python
for v in (0.15, 0.25, 0.35):
    tl = build_cell(velocity=v).simulate_sequence("cycle")
    print(f"{v:.2f} m/s -> cycle {tl.duration:.2f} s"
          f" (feed {tl.step_span('feed').duration:.2f} s)")
```

```text
0.15 m/s -> cycle 8.71 s (feed 3.17 s)
0.25 m/s -> cycle 7.44 s (feed 1.90 s)
0.35 m/s -> cycle 6.90 s (feed 1.36 s)
```

Only the feed wait moves; the motion part of the cycle is fixed. Nothing was
re-taught between rows — the poses are planned, so the cell absorbed the change.

## Make it a test

Deterministic numbers are assertable numbers. This is the workflow botrail
exists for:

```python title="test_cell.py"
from cell import build_cell


def test_cycle_budget():
    tl = build_cell().simulate_sequence("cycle")

    assert tl.duration <= 8.0                    # cycle-time budget
    assert tl.step_span("feed").end <= 2.0       # the crate arrives on time
    assert tl.signal("eye").rising_edges()       # the handshake happened
    assert tl.min_clearance() > 0.05             # safety margin, meters
```

```bash
pytest test_cell.py
```

Now move the beam sensor a little further downstream, or slow the belt, and the
test fails with the new cycle time. A layout edit becomes a red build instead of
a shop-floor surprise.

!!! tip "This repository does exactly that"

    botrail runs a cell like this in its own CI —
    [`python/tests/test_cell_regression.py`](https://github.com/botrail/botrail/blob/main/python/tests/test_cell_regression.py)
    — and [`examples/sweep_demo.py`](https://github.com/botrail/botrail/blob/main/examples/sweep_demo.py)
    runs the parameter study as a script.

## Ship it

The same bake exports as a USD animation — every robot and every obstacle, with
grasped objects riding along exactly as simulated:

```python
tl.export_usd("cycle.usda", fps=60)
```

The result plays in usdview, Omniverse, or Blender with no botrail involved. To
watch it in the studio instead:

```python
scene = build_cell()
scene.play_usd_animation("cycle.usda")
bt.studio(scene)
```

## Next

* The [tutorials](../tutorials/index.md) build on exactly this: a tracking
  pick where the belt never stops
  ([Pick from a moving belt](../tutorials/sequence-cell.md)), the full CI
  workflow ([Verify the cell in CI](../tutorials/verify-in-ci.md)), and two
  arms sharing one infeed behind a zone interlock
  ([Two arms, one belt](../tutorials/two-robots.md)).
* Look up what else a scene can hold — sources and sinks, linear axes, zone
  sensors, attachments — in the [Scene reference](../reference/api/scene.md).
* Browse the sequence vocabulary in the [`bt.seq` reference](../reference/api/seq.md).
