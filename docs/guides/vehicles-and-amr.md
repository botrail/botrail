# Vehicles and AMRs

An AGV is not a special kind of robot in botrail — it is a
[device](sensors-and-devices.md), the same standing as a conveyor or a
linear axis. That is how a cell PLC sees one: you send it a destination and
wait for a completion signal. Everything else follows from that.

```python
scene.add_vehicle(
    "agv",
    body=["/World/AGV"],                       # obstacles carried rigidly
    path=[(-2.6, -2.9), (0.0, -2.9), (0.0, -1.15)],
    stations={"warehouse": 0, "gate": 1, "dock": 2},
    speed=0.8, turn_speed=math.radians(45),
    start="warehouse",
)

sq.step("call", actions=[bt.seq.goto("agv", "dock")],
        transition=bt.seq.device_done("agv"))
```

`goto` is the dispatch order and `device_done` is "in position" — the same
pair a linear axis uses. The vehicle's moving state is an output lane on the
timing chart, next to the conveyors.

![A vehicle docked in the cell, mid-handover](../assets/studio/vehicle.png)

The studio draws the guide path on the floor with a marker at each station,
and the timing chart carries the vehicle's lane alongside the sensors it
handshakes with — here `dock_occupied`, `gate_zone` and `tray_loaded`.

## Travel is authored, not planned

The `path` is the model of the tape on the floor. botrail does not plan
routes, avoid dynamic obstacles or run SLAM: it drives the path you taught
it, exactly, every time. That is the same bargain the rest of the tool makes
— [determinism](../concepts/determinism.md) in exchange for scope.

Between waypoints the vehicle drives straight at `speed`; where the path
turns it pivots in place at `turn_speed`. Both bake to closed-form spans, so
any resample rate is exact. The **arrival heading is the last leg's
direction**, which means the waypoint before a station is what decides how
the machine docks.

`body` names the obstacles carried rigidly. Entries match an obstacle
exactly or as a subtree prefix, so `body=["/World/AGV"]` picks up everything
under it.

### Two things a vehicle forces that a conveyor never did

**Ground clearance is real.** The aisle check (below) tests the body against
everything it does not carry — including the 3 mm painted lines on the
floor. Model the chassis with the ground clearance a real machine has, and
give the parts that genuinely touch the floor `set_obstacle_enabled(name,
False)` so they are scenery rather than body.

**A turn sweeps wider than the body.** A pivot swings the body's
half-diagonal, so the clearance that decides whether a dock works is the one
*around the turn*, not along the straight. If a dead end has no room for
that, pass `allow_reverse=True` and the vehicle backs out instead of turning
around in it — which is what a differential-drive machine does anyway.

## The aisle check

While a vehicle is travelling, every scan tick tests its body and its load
against everything else. A contact is a hard, timestamped failure:

```
vehicle `agv` collides with `/World/Pallet/DeckBoard_1` at t = 9.320s
(body part `agv/base`); widen the aisle or re-teach the path
```

This is the mobile counterpart of the arm's collision checking, and it is
deliberately not something the tool tries to solve for you: the fix is a
wider aisle or a different path, both of which are layout decisions.

The check runs **only while the vehicle moves**, so a parked machine resting
against its dock guide is legitimate authoring rather than a false alarm.

## Carrying things: the tray

A `tray` is a zone in the vehicle's own frame. Anything unattached whose
origin is inside it rides along, rotation included:

```python
scene.add_vehicle(..., tray_position=(0.0, 0.0, 0.37), tray_size=(0.84, 0.62, 0.12))
```

There is no load or unload action, and that is the point — it is the
[conveyor's zone rule](sensors-and-devices.md#conveyors) moved onto a moving
frame. A part the arm sets down on the deck simply becomes cargo on the next
tick; one the arm picks back up stops being cargo, because a grasp wins over
a deck.

!!! note "Rest the load on the collision surface"
    If the vehicle body is a mesh, collision runs on its convex
    decomposition, and the convex hull of a dished top fills the dish. The
    deck the checker sees can sit a few millimetres above the one you can
    see, and placing a part on the drawn surface is then rejected as a
    collision. Raise the place pose until it clears; the gap is invisible.

## Sensors that travel

A sensor can ride a vehicle by naming it in `mount`. Its geometry is then
read in that vehicle's frame and re-resolved every tick:

```python
scene.add_zone_sensor("tray_loaded",
                      position=(0.0, 0.0, 0.37), size=(0.84, 0.62, 0.12),
                      watch=[CARTON], mount="agv")
```

The difference is not cosmetic. A floor-mounted zone reports the carton for
the moment it is set down and loses it the instant the machine pulls away; a
mounted one still answers "loaded" out on the aisle, which is what a
departure permit has to be able to ask.

## Mounting an arm: the AMR

Put a robot on a vehicle and its base stops being a scene constant:

```python
scene.mount_robot("amr", offset_position=(0.0, 0.0, 0.31))
```

From here the base is *derived* from the vehicle's frame every tick, and
everything downstream — FK, collision, sensors, grasped objects, the studio,
the USD export — follows without any special handling, because all that
changed is where the links are.

Two rules come with it:

* **A planned motion cannot start while the vehicle is driving.** A plan is
  baked in world coordinates when it starts, so a base moving underneath it
  invalidates every waypoint. The rollout rejects it by name rather than
  producing nonsense. Wait for `device_done` first.
* **A ramp can.** Ramps are re-evaluated every tick, so the arm can fold
  itself into its stow *while* the machine travels — which is what a real
  AMR does between stations, and why the stow costs no cycle time.

One taught pose often serves several stations, incidentally: the base is the
deck, so a stand that sits 0.65 m to the machine's left is in the same place
at every station it visits. That is the whole idea of a mobile manipulator,
and it falls out of the mounting rather than needing anything extra.

## What this does not model

Worth stating plainly, because the vocabulary invites bigger expectations:

* No navigation — no SLAM, no dynamic obstacle avoidance, no route planning.
* No fleet dispatch or traffic management. One vehicle's deterministic cycle
  is in scope; deciding which of twenty vehicles goes where is not.
* No docking error, wheel slip, protective-field slowdowns or battery.
* No acceleration model: vehicles move at constant speed. If your machine's
  ramp-up distance is a large fraction of a leg, derate the speed you give
  it rather than using the spec maximum.

For arrival variation, sweep it rather than sampling it — see
[parameter sweeps](../tutorials/parameter-sweep.md) and
`examples/agv_sweep_demo.py`, which prints how late a dispatch may be before
the cell starts waiting on it.

## Examples

* `examples/agv_cell_demo.py` — an AGV serving the factory cell: called
  while the arm picks, held outside the gate by an interlock, loaded on the
  deck, released once its own load sensor says it has the part.
* `examples/amr_demo.py` — the arm riding the vehicle across two stations,
  stowing itself on the move.
* `examples/agv_sweep_demo.py` — dispatch delay and dock depth as
  deterministic response curves.
