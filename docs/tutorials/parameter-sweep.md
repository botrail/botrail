# Parameter sweeps

*Walks through [`examples/basics/sweep_demo.py`](https://github.com/botrail/botrail/blob/main/examples/basics/sweep_demo.py)
— the cell authored once as a function of its parameters, baked at every
variant with `bt.sweep`, compared by the numbers that matter, and searched
with `bt.optimize`.*

Because motions are planned rather than taught point by point, a layout change
does not invalidate the cell — it just changes the numbers. That makes layout
studies a loop: author the cell as a function, sweep the parameter, read the
table. No re-teaching between rows. This example runs from a checkout with no
downloads (primitive-geometry arm):

```bash
python examples/basics/sweep_demo.py
```

## The whole script

Short enough to read in one sitting:

```python title="examples/basics/sweep_demo.py"
--8<-- "examples/basics/sweep_demo.py"
```

Everything hangs off the signature `build_cell(velocity, lane_y)`: belt speed,
and how close the conveyor lane runs to the robot. `metrics(tl)` reduces one
bake to the three numbers under study — cycle time, feed duration, minimum
clearance — and [`bt.sweep`][botrail.study.sweep] does the rest: it calls
`build_cell` at every point of the grid, bakes the named sequence, applies
`metrics`, and hands back a table ([`Sweep`][botrail.study.Sweep]) whose rows
are the parameters plus the numbers, in grid order. A variant the planner
cannot solve is a row with `ok=False` and the reason, not an exception —
the table says where the cliff is.

## The output

```text
== belt speed sweep (lane_y = 0.60 m) ==
| velocity | lane_y | cycle | feed | clearance |
|---|---|---|---|---|
| 0.1 | 0.6 | 10.30 | 4.76 | 0.530 |
| 0.15 | 0.6 | 8.71 | 3.17 | 0.530 |
| 0.2 | 0.6 | 7.92 | 2.38 | 0.530 |
| 0.25 | 0.6 | 7.44 | 1.90 | 0.530 |
| 0.3 | 0.6 | 7.13 | 1.59 | 0.530 |
| 0.35 | 0.6 | 6.90 | 1.36 | 0.530 |

-> only the feed wait moves; the motion part of the cycle is fixed

== conveyor lane sweep (belt = 0.25 m/s) ==
| velocity | lane_y | cycle | feed | clearance |
|---|---|---|---|---|
| 0.25 | 0.7 | 7.44 | 1.90 | 0.630 |
| 0.25 | 0.6 | 7.44 | 1.90 | 0.530 |
| 0.25 | 0.5 | 7.45 | 1.91 | 0.430 |
| 0.25 | 0.4 | 7.45 | 1.91 | 0.330 |
| 0.25 | 0.35 | 7.45 | 1.91 | 0.280 |

-> the cycle barely moves, the safety margin is what shrinks

== both at once: cycle time over the grid ==
| lane_y \ velocity | 0.15 | 0.25 | 0.35 |
|---|---|---|---|
| 0.7 | 8.71 | 7.44 | 6.90 |
| 0.5 | 8.71 | 7.45 | 6.90 |
| 0.35 | 8.71 | 7.45 | 6.90 |

(clearance over the same grid)
| lane_y \ velocity | 0.15 | 0.25 | 0.35 |
|---|---|---|---|
| 0.7 | 0.630 | 0.630 | 0.630 |
| 0.5 | 0.430 | 0.430 | 0.430 |
| 0.35 | 0.280 | 0.280 | 0.280 |

== the question a layout meeting asks: fastest cycle with 0.4 m of clearance ==
{'velocity': 0.4, 'lane_y': 0.5} -> cycle 6.73 s, clearance 0.43 m (13 bakes, coordinate descent; the full grid is 63)
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

## Two axes at once, and the search

The third block of the output is the same study over both parameters:
`Sweep.pivot(rows, cols, metric)` folds a two-axis grid into one table per
metric, and the two tables say the whole story at a glance — velocity moves
the cycle, lane moves the clearance, and neither touches the other.

The last block is the question a layout meeting actually asks: *the
fastest cycle that still keeps 0.4 m of clearance*. [`bt.optimize`][botrail.study.optimize]
searches the space for it — as a full grid, or, as here, by coordinate
descent on the grid (from the middle of the space, one step of each
parameter at a time, taking the best feasible improvement until nothing
improves): 13 bakes instead of 63, the same answer, and every bake it made
is in `best.evaluated` as a table. Both methods are deterministic — there is
no random number anywhere in a study — so the optimum is as assertable as a
single cell:

```python
best = bt.optimize(build_cell, space={"velocity": (0.10, 0.40, 0.05), "lane_y": (0.30, 0.70, 0.05)},
                   objective="cycle", constraints={"clearance": (">=", 0.4)},
                   metrics=metrics, sequence="cycle", method="descent")
assert best.params == {"velocity": 0.4, "lane_y": 0.5}
```

## Scaling it up

A study is a table, and the table saves: `result.save("study.csv")` (or
`.md`, `.json`) for whichever plotting tool the layout meeting uses,
`result.best("cycle", where=lambda r: r["clearance"] >= 0.4)` for the row
that matters, `result.pareto(minimize=["cycle"], maximize=["clearance"])`
for the trade-off front. Bigger grids bake in parallel — `workers=4` runs
the variants in separate processes and still returns the rows in grid order
(`build` and `metrics` then have to be importable, module-level functions,
as they are in this file).

## Next

The same discipline holds with two robots in the cell — and the numbers get
more interesting: [Two arms, one belt](two-robots.md).
