# Attach and tracking

Two mechanisms carry parts through a cycle. **Attach** makes an obstacle part
of the robot — a grasp. **Track** makes the robot's commands follow a moving
part — conveyor tracking. They compose: latch onto a moving box, grasp it,
and the belt never has to stop.

## Attach — the grasp

```python
scene.attach("crate", link="/panda/panda_hand",
             touch_links=["/panda/panda_leftfinger", "/panda/panda_rightfinger"])
...
scene.detach("crate")
scene.attachments        # [(object, link), ...]
```

Attach glues the obstacle to a link **at its current relative pose** — pose
the grasp first, then attach. From that moment the object follows the link
live, in planning, and in playback, and **collides as part of the robot**:
checked against the environment, excused against its `touch_links`.

* `link=None` uses the TCP link.
* `touch_links=None` allows contact with the link's whole subtree — the
  gripper. List the finger pads explicitly when the grasp squeezes *into* the
  part and only the pads should be allowed that contact.
* `detach` freezes the object's pose where the robot holds it — release over
  a surface, not in the air.

In sequences the same pair exists as actions: `bt.seq.attach(...)` /
`bt.seq.detach(...)`.

## Track — conveyor tracking

```python
sq.step("latch",   actions=[bt.seq.track("crate")])
sq.step("descend", actions=[bt.seq.ramp(grasp_targets, 0.6)])
sq.step("close",   actions=[bt.seq.ramp(finger_targets, 0.4)])
sq.step("grasp",   actions=[bt.seq.attach("crate", ...)])
sq.step("lift",    actions=[bt.seq.ramp(hover_targets, 0.6)])
sq.step("carry",   actions=[bt.seq.untrack(), bt.seq.motion("to_pallet")])
```

`track` latches onto the part **at that instant**: from then on, every
commanded pose is carried by the part's motion since the latch. Poses taught
at the station keep meeting the part while it travels — the robot dives onto
the moving box and closes on it in motion.

The rules that make it work:

* **Latch where you taught.** The offset is measured from the part's pose at
  the latch instant; latching an upstream sensor early means grasping
  off-center by however far the part still had to travel. Latch on the
  station's own sensor.
* **Ramps while tracking, plans not.** A planned motion bakes all its
  waypoints up front, and the target would run away from them — authoring one
  while tracking is an error. `ramp` re-evaluates every tick, so approach and
  close are ramps.
* **Grasping freezes the offset.** `attach` on the tracked part stops the
  chase; the following lift goes straight up from wherever the part was
  caught.
* **`untrack` doesn't jump.** The robot holds where it stands; untrack first,
  then run the planned transfer.

## Ramps — the guarded move

`bt.seq.ramp(targets, duration)` drives named joints to targets over a fixed
duration, **without collision checking**. That is not a loophole; it is the
tool for the two motions a collision-checked planner rightly refuses: driving
*through* contact (closing on a part) and moving in sync with something the
planner would treat as an obstacle (descending onto a tracked box). Keep ramps
short and taught; let planned motions do the traveling.

The [Pick from a moving belt](../tutorials/sequence-cell.md) tutorial runs
this entire pattern against a real cell, and measures what tracking bought:
the box was caught 150 mm downstream of where it was taught.
