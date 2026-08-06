# Pick from a moving belt

*Walks through [`examples/sequence_demo.py`](https://github.com/botrail/botrail/blob/main/examples/sequence_demo.py)
— a tracking pick on the factory cell, baked into one deterministic timeline.*

The conveyor feeds a box down the belt until it interrupts a photoelectric beam
at the pick point — **and then keeps running**. The sequence latches onto the
box, so every pose taught at the station rides along with the part: the robot
dives onto the moving box, closes on it in motion, and only lets go of the belt
sync once the box is its own.

```bash
python examples/sequence_demo.py
```

```text
cycle time: 15.56s
    0.00 –   6.01  feed
    6.01 –   6.01  latch
    6.01 –   6.61  descend
    6.61 –   7.01  close
    7.01 –   7.01  grasp
    7.01 –   7.61  lift
    7.61 –  11.35  carry
   11.35 –  12.15  lower
   12.15 –  12.15  release
   12.15 –  12.55  open
   12.55 –  13.35  retreat
   13.35 –  13.85  settle
   13.85 –  15.56  home
tracked pick: caught the box 150 mm downstream, belt still running
exported to cell_seq.usda — view with: usdview cell_seq.usda
```

Thirteen steps, and the number in the last line is the point: between the latch
and the grasp the belt carried the box 150 mm. Without tracking, that is how
far the pick would have missed by.

## The infeed

The demo builds on the [Pose and plan](pose-and-plan.md) scene and gives it
behavior — a conveyor and a beam:

```python
--8<-- "examples/sequence_demo.py:48:72"
```

Two details are doing real work here. The conveyor's transport zone floats
*above* the belt slab, so the advection carries the goods and not the
conveyor's own structure. And the beam is not placed at the pick frame — it is
placed half a box-width plus a beam-radius downstream, because a beam trips
when the box's *leading face* reaches it. Placed that way, the latch fires at
the exact moment the box's center crosses the taught grasp.

## Teaching, hover-first

```python
--8<-- "examples/sequence_demo.py:74:85"
```

Each station is solved hover-first so the grasp warm-starts from the pose right
above it and stays in the same posture family. Between the stations the robot
returns to the ready pose first: the pallet is a 150° base swing from the
conveyor, and warm-starting IK across that walks the solver into a local
minimum. These two habits — hover-first, and re-seeding across big swings —
carry to every cell you'll teach.

The finger stroke is chosen with the same care:

```python
OPEN, CLOSED = 0.039, 0.029
TOUCH = ["/panda/panda_leftfinger", "/panda/panda_rightfinger"]
```

Closed is a millimetre a side *into* the 60 mm box — the cycle's only
by-design contact, which is why the finger pads are the only links allowed to
touch the carried box (`touch_links`). Open has to swallow the few millimetres
a joint-space ramp bows sideways on its way down; 0.04 is the joint limit,
which the planner excludes.

## The sequence

```python
--8<-- "examples/sequence_demo.py:100:136"
```

Read it the way a PLC programmer would:

* **`feed`** starts the belt and the pre-position motion *together*, and its
  transition is `all_of(signal, done)` — series contacts. The step ends when
  the part has arrived **and** the arm is there to meet it.
* **`latch`** is [`bt.seq.track`][botrail.seq.track]: from here, every
  commanded pose is carried by the box's motion since this instant. Note what
  is *not* here — no `stop("conv")`. The belt keeps running.
* **`descend`/`close`** are [`ramps`][botrail.seq.ramp] — guarded,
  fixed-duration joint moves, the right tool for driving *through* contact
  where a collision-checked planner would refuse.
* **`grasp`** attaches the box. Grasping the tracked part freezes the sync
  offset, so the following `lift` goes straight up from wherever the box was
  caught — not from where it was taught.
* **`carry`** unlatches and runs a *planned* transfer. Planned motions cannot
  run while tracking (they bake all their waypoints up front, and the target
  would run away from them); ramps can. Untrack first, then plan.
* **`settle`** is a timer (`elapsed(0.5)`), and the internal `carrying` signal
  brackets the transfer — both of which become assertable lanes in the
  timeline.

## Ask the timeline

The step table above is `timeline.step_spans`. The 150 mm is two
[`object_pose`][botrail.SequenceTimeline.object_pose] queries:

```python
--8<-- "examples/sequence_demo.py:151:155"
```

Anything the bake computed is queryable afterwards — that is what the
[next tutorial](verify-in-ci.md) turns into a test suite.

## The complete script

??? example "examples/sequence_demo.py"

    ```python
    --8<-- "examples/sequence_demo.py"
    ```

## Next

* Turn a bake like this into CI assertions: [Verify the cell in CI](verify-in-ci.md).
* Add a second arm to the same belt: [Two arms, one belt](two-robots.md).
* The exported `cell_seq.usda` plays in usdview as-is, or back in the studio:
  [Export and replay USD](replay-usd.md).
