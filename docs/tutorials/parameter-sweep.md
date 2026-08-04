# Parameter sweeps

*Walks through [`examples/sweep_demo.py`](https://github.com/botrail/botrail/blob/main/examples/sweep_demo.py)
— the cell authored once as a function of its parameters, baked at every
variant, compared by the numbers that matter.*

Because motions are planned rather than taught point by point, a layout change
does not invalidate the cell — it just changes the numbers. That makes layout
studies a loop: author the cell as a function, sweep the parameter, read the
table. No re-teaching between rows. This example runs from a checkout with no
downloads (primitive-geometry arm):

```bash
python examples/sweep_demo.py
```

## The whole script

Short enough to read in one sitting:

```python title="examples/sweep_demo.py"
--8<-- "examples/sweep_demo.py"
```

Everything hangs off the signature `build_cell(velocity, lane_y)`: belt speed,
and how close the conveyor lane runs to the robot. `bake()` reduces a variant
to the three numbers under study — cycle time, feed duration, minimum
clearance.

## The output

```text
== belt speed sweep (lane_y = 0.60 m) ==
 belt m/s |  cycle s |  feed s | clearance m
     0.10 |    10.30 |    4.76 |       0.530
     0.15 |     8.71 |    3.17 |       0.530
     0.20 |     7.92 |    2.38 |       0.530
     0.25 |     7.44 |    1.90 |       0.530
     0.30 |     7.13 |    1.59 |       0.530
     0.35 |     6.90 |    1.36 |       0.530
-> only the feed wait moves; the motion part of the cycle is fixed

== conveyor lane sweep (belt = 0.25 m/s) ==
 lane_y m |  cycle s |  feed s | clearance m
     0.70 |     7.44 |    1.90 |       0.630
     0.60 |     7.44 |    1.90 |       0.530
     0.50 |     7.45 |    1.91 |       0.430
     0.40 |     7.45 |    1.91 |       0.330
     0.35 |     7.45 |    1.91 |       0.280
-> the cycle barely moves, the safety margin is what shrinks
```

## Reading the tables

The two sweeps fail in opposite ways, which is the lesson:

* **Belt speed moves the cycle.** The whole difference between 10.30 s and
  6.90 s is the feed wait — the planned motions are untouched. If the cell
  misses takt, this column says whether a faster belt buys it back.
* **Lane position eats the clearance.** The cycle barely moves (the approach
  is a hair longer), but the safety margin drops linearly — at `lane_y = 0.35`
  the closest approach over the whole cycle is down to 0.28 m. Nothing failed
  yet, which is exactly why it is worth a number: this is the regression a
  visual check misses.

Every row is a deterministic bake — re-running the script prints the same
table, digit for digit.

## From sweep to test

A sweep tells you where the cliff is; a test keeps you off it. The two
assertions this study feeds, in the vocabulary of
[Verify the cell in CI](verify-in-ci.md):

```python
def test_takt_at_nominal_speed():
    tl = build_cell(velocity=0.25).simulate_sequence("cycle")
    assert tl.duration <= 8.0            # from the speed table

def test_lane_keeps_its_margin():
    tl = build_cell(lane_y=0.60).simulate_sequence("cycle")
    assert tl.min_clearance() > 0.5      # 0.530 nominal, with headroom
```

Move the lane 100 mm closer in a layout revision and the second test fails
with the new clearance in the message — the sweep row, delivered as a red
build.

## Scaling it up

The pattern generalizes past two loops — it is ordinary Python around a
deterministic function:

```python
import csv, itertools

with open("study.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["velocity", "lane_y", "cycle", "feed", "clearance"])
    for v, y in itertools.product((0.15, 0.25, 0.35), (0.4, 0.5, 0.6, 0.7)):
        cycle, feed, clearance = bake(velocity=v, lane_y=y)
        w.writerow([v, y, round(cycle, 2), round(feed, 2), round(clearance, 3)])
```

Twelve baked variants, one CSV for whichever plotting tool the layout meeting
uses.

## Next

The same discipline holds with two robots in the cell — and the numbers get
more interesting: [Two arms, one belt](two-robots.md).
