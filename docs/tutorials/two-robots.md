# Two arms, one belt

*Walks through [`examples/dual_cell_demo.py`](https://github.com/botrail/botrail/blob/main/examples/dual_cell_demo.py)
— two Frankas sharing one infeed, arbitrated by a zone interlock.*

The cell has a single pick point on the belt and two arms facing each other
across it, each palletising to its own pallet. Doubling the arms pays off
because the *transfer* dominates the cycle: while one arm carries a carton to
its pallet, the other is already tracking the next one down the belt.

That is also what makes the arms dangerous to each other. Over the pick point
their envelopes coincide — at both grasps the two hands are in the same place —
so the cell needs an interlock, and it is written the way a PLC writes one: a
zone around the contested airspace, one sensor per arm, and a step that will
not proceed while the other arm's zone signal is on.

```bash
python examples/dual_cell_demo.py
```

```text
cycle time: 83.71s
  near  moving 24.02s of 83.71s
  far   moving 33.17s of 83.71s
both arms in motion for 11.5s of it
stacked 2 course(s) on each pallet from a pool of 6
exported to cell_dual.usda — view with: usdview cell_dual.usda
```

The third line is what the second arm bought: 11.5 s in which both arms were in
motion at once — picks overlapped with transfers instead of queueing behind
them.

## Adding the second arm

```python
--8<-- "examples/dual_cell_demo.py:95:106"
```

The same `Robot` model is added again — a scene holds instances, not one
robot — with a name and a base pose of its own (`(0, 0, 1, 0)` is a 180° yaw:
facing back across the belt). From here every API you know takes a `robot=`
argument: `teach_grasp(..., robot=FAR)`, `add_segment(..., robot=robot)`,
`bt.seq.ramp(..., robot=robot)`, `joint_positions_of(name)`. Motions belong to
an instance; steps say whose hand they move.

## What planning means with two arms

Nothing here plans the two arms *together*. Each motion is planned for its own
arm with the other frozen as an obstacle, at the moment its step starts — and
the rollout then re-checks arm-against-arm **every tick**. The division of
labor is deliberate:

* The planner answers: "can this arm get there, given where everything is
  right now?"
* The rollout answers: "did the two of them, executing their own plans,
  ever meet?" — and a meeting is not a warning. It is a hard error with a
  timestamp and the touching link pair, and the bake fails.

Keeping the arms apart is therefore *authored*, the way a PLC author does it —
which is the next section — and the tick check is the guard that catches you
when the authoring is wrong.

## The interlock

```python
--8<-- "examples/dual_cell_demo.py:239:249"
```

One zone volume, two sensors. A zone sensor reports "somebody is inside", not
*who* — so a single zone watching both arms would be tripped by the very arm
waiting on it. Give each arm its own sensor over the same volume and the gate
reads naturally:

```python
bt.seq.signal("zone_far", False)   # proceed when the far arm's zone is clear
```

And the far arm's pick is gated by the *upstream* photo-eye, so the loop wires
the station's handback:

```python
--8<-- "examples/dual_cell_demo.py:376:387"
```

`--clash` drops exactly one thing — the near arm's `zone_far` condition — and
is the difference between a cell and an accident report.

## Splitting the cycle so it overlaps

The overlap does not happen by itself; the sequence is *shaped* for it. Each
arm's pick half ends the moment the transfer is started, not finished:

```python
--8<-- "examples/dual_cell_demo.py:315:321"
```

`transition=bt.seq.immediately()` releases the sequence while the motion runs —
that is what lets the other arm move in. The place half then re-synchronizes
with [`robot_done`][botrail.seq.robot_done], the inter-robot handshake:

```python
sq.step(f"{robot}_landed{tag}", transition=bt.seq.robot_done(robot))
```

Start a motion, hand the station over, wait for your own hand to land before
using it again — the same discipline a two-station PLC program has.

## The line itself

Worth reading in the full source, briefly noted here:

* **The magazine.** "Endless supply" is a finite pool plus a sink that hands
  carriers back to the source ([`add_source`][botrail.Scene.add_source] /
  [`add_sink`][botrail.Scene.add_sink]) — which is also what a real
  accumulation line is. The source feeds *on demand* (one carton per `start`),
  because the steps name the carton each arm takes: pool order has to be
  arrival order, and pre-loading the belt inverts it.
* **Two photo-eyes.** One upstream (`beam_ahead`) calls an arm over while the
  carton is still travelling; one on the station (`beam_pick`) is what the
  tracking latch keys off. `track` records where the carton is *at that
  instant* — latching on the upstream eye alone put the grip 120 mm off, and
  the carton landed 120 mm off the pallet.
* **Cleats.** Slats riding the belt with collision off: scenery that happens
  to move. The conveyor advects any unattached obstacle in its zone and does
  not care whether collision is on, so the belt reads as *moving* for free.

## What happens without the interlock

```bash
python examples/dual_cell_demo.py --clash
```

```text
the unarbitrated cell happens to run (83.71s), but both arms are over the
station together for 1.86s.
   Nothing separated them — the transfers merely missed each other.

asked to enter together, they are caught:
   robots `near` and `far` collide at t = 4.990s (/panda/panda_link5 ×
   /panda/panda_link6); add an interlock (zone sensor / robot_done) so one
   waits for the other
```

Two lessons in one run. Dropping the interlock does **not** necessarily crash
the cell: the unarbitrated bake can succeed, with the zones reporting how long
both arms were over the station together — nothing separated them, the
transfers merely missed each other, and "it worked when we tried it" is not a
safety argument. Then the `clash` sequence gives them one reason to converge,
and the rollout catches it at the tick it happens, naming the links. That
guard does not depend on timing.

## The complete script

??? example "examples/dual_cell_demo.py"

    ```python
    --8<-- "examples/dual_cell_demo.py"
    ```

## Next

The 48 MB recording this bake exported plays in usdview as-is — or back in the
studio, which is where [Export and replay USD](replay-usd.md) picks up.
