# Legged robots

A quadruped or a humanoid is, to the cell, a [vehicle](vehicles-and-amr.md)
with legs. It is sent with `goto`, it arrives with `device_done`, it carries
what sits on its back or in its hands, and it has to fit through the gate
like any AGV. What the vehicle vocabulary lacks is the legs — and that is
all `bt.Gait` adds: which links are the feet, how the machine stands, and
the rhythm it walks to.

```python
dog = bt.Robot.from_urdf("go2.urdf")
gait = bt.Gait(
    legs={"FL": "FL_foot", "FR": "FR_foot", "RL": "RL_foot", "RR": "RR_foot"},
    stance={"FL_hip_joint": 0.0, "FL_thigh_joint": 0.8, "FL_calf_joint": -1.5, ...},
    pattern="trot", period=0.45, lift=0.07, max_stride=0.45, foot_radius=0.022,
)
scene.add_robot(dog, name="go2")
scene.add_vehicle("walker", body=["go2/footprint"], path=[YARD, DOCK, BAY],
                  stations={"yard": 0, "dock": 1, "bay": 2}, speed=0.6, turn_speed=1.0)
scene.mount_robot("walker", robot="go2", gait=gait)   # stands it on the floor

sq.step("to dock", actions=[bt.seq.goto("walker", "dock")],
        transition=bt.seq.device_done("walker"))
```

The `goto` is what makes it walk. There is no walk action to author, any
more than there is a "spin the wheels" action for an AMR: the legs are a
property of the mount, the way a wheel's spin is a property of its axle.

![A quadruped carrying a part through the cell](../assets/studio/legged.png)

## What is simulated, and what is not

Nothing physical. The body rides the vehicle's closed-form motion — a
straight at constant speed, a pivot at constant rate — exactly as an AMR's
base does. The moment the vehicle is dispatched, every footfall of the
drive is planned from that motion: a foot lands where its stance will be
centred under the body at mid-stance (a half-stance ahead on a straight, a
step around the origin on a pivot). From then on a planted foot never moves
in the world, and each leg is solved by IK every scan tick to keep it
there. After the vehicle stops the footfalls converge on the parked
positions by themselves, so the machine settles into its stance within a
cycle or so of arriving.

The one thing that makes a walk look wrong is a foot that slides. That is
the one thing this guarantees — and it is checked, not hoped for: the feet
on a baked timeline stay within a micrometre of their landings.

What it does *not* answer is whether the machine could actually walk:

* No balance, no contact forces, no slip, no falling over.
* Floors follow the vehicle's path: a ramp is walkable when its waypoints
  climb (declare `max_grade` — see
  [vehicles](vehicles-and-amr.md#travel-is-authored-not-planned)), and
  steps are walkable where a *walkable surface* stands (a stair tread, a
  mezzanine slab — see [Stairs and steps](#stairs-and-steps)). What the
  feet do on them is placement, not physics: no balance there either.
* No acceleration — vehicles move at constant speed, legged or not.
* A leg's reach is checked (a stride the legs cannot take is refused by
  name, before or during the bake), its torque is not.

What it *does* answer is what it answers for any vehicle — does it fit,
does it clash, how long is the cycle — with legs that move like legs.

## The gait

| Field | Meaning |
|---|---|
| `legs` | Leg name → foot link, in the order the pattern's phase table is read: `FL, FR, RL, RR` for the quadruped patterns, `L, R` for `biped`. A value may be `(foot_link, contact)` to give one leg its own contact. |
| `stance` | Joint → value of the standing pose. Must name every leg joint; other joints keep their mount-time value. The feet must stand level (on one plane within 5 mm). |
| `pattern` | `walk` (duty 0.75, lateral sequence), `trot` (duty 0.5, diagonal pairs), `biped` (duty 0.6), or `custom` with `duty` and `phases`. |
| `period`, `lift` | Cycle time in seconds; swing apex above the floor in metres. |
| `max_stride` | The longest step a leg may take between two landings. `speed × period` (and the outer foot's arc in a pivot) must stay under it, or the vehicle's rates are refused by name. |
| `foot_radius` | How far the foot link's origin stands above the floor: a ball foot's radius, or an ankle frame's height over its sole. |
| `contact` | `point` (a ball: position only) or `sole` (flat on the floor, pointing where it landed). A sole's foot link must point +Z up in the stance; a 6-DOF leg keeps it flat *and* pointed, a 5-DOF leg keeps it flat. |
| `arm_swing` | Joint → amplitude swung in step with the first leg — a biped's arms. Left alone while the robot holds something or a ramp is driving them. |
| `bob`, `lateral` | Body sway while walking: up and down twice a cycle, over the planted leg once a cycle. The legs absorb it; the feet do not move. |

Mounting with a gait stands the robot on the vehicle plane (the offset is
derived from the stance; pass `offset_position` to override) and puts it
into its stance. Standing, the legs are yours to ramp — a crouch is a ramp
like any other — but a leg joint cannot be ramped mid-walk, and a walk
cannot start while a ramp is still moving a leg. Planned motions cannot
start while the vehicle drives, for the same reason as on an AMR: a plan
is baked in world coordinates.

## The body, the aisle, and the load

The vehicle's `body` is the aisle check. A legged robot's links are a
robot, not a body, so give the vehicle an invisible **footprint** box —
`set_obstacle_visible(name, False)` — sized to the machine; a gate too
narrow for it fails with `VehicleCollision` naming the panel. Mounting with
a gait marks the robot's links as allowed to touch that footprint (it
stands inside it), so it never counts as a collision.

The legs themselves are checked too, while the vehicle moves: a link or a
held part that meets the environment fails with `RiderCollision`, naming
the link and the obstacle. This is the same check an AMR's arm gets while
its carrier drives.

Anything set down inside the vehicle's `tray` rides along — a part placed
on a quadruped's back by an arm is cargo the moment it is released, with no
load action. A humanoid carries in its hands: `attach` the part to a hand
link, and it rides still through the walk (the arms do not swing with full
hands).

**A walking machine's deck is its body.** On a wheeled vehicle the tray is
rigid with the route, and the two are the same thing; on a legged one they
are not — the body tilts onto a grade and rides up the steps while the
route stays on the guide line between the waypoints. So the zone is placed
on the body and what it holds is bound to the body link, which is what a
strap does: on a flight the load climbs and tilts with the back it sits
on, instead of floating a ride's worth above it. Everything downstream —
the studio, the USD bake, the collision checks — reads that pose, so no
authoring changes.

## Humanoids

Two legs, soles instead of balls, arms that swing, a body that bobs:

```python
g1 = bt.Robot.from_urdf("g1_29dof_rev_1_0.urdf")
gait = bt.Gait(
    legs={"L": "left_ankle_roll_link", "R": "right_ankle_roll_link"}, contact="sole",
    stance={...}, pattern="biped", period=0.85, lift=0.05, max_stride=0.5,
    foot_radius=0.035,                     # the ankle frame sits 35 mm over the sole
    arm_swing={"left_shoulder_pitch_joint": -0.25, "right_shoulder_pitch_joint": 0.25},
    bob=0.015, lateral=0.02,
)
```

Two things worth knowing before you teach one:

* **Bend the knees.** A leg stood nearly straight has no reach left for
  the foot that stays planted while the body moves off — the first
  half-period of every walk stretches it by half a stride. A stance with
  the knees at 0.8 rad walks where one at 0.5 rad fails by name.
* **Arm ramps are not planned.** A ramp from "arms at the sides" to "arms
  forward" swings the forearms through a bench at hip height, even though
  both end poses are clear. Teach an intermediate pose (out and up, then
  in), and check every ramp the way `examples/legged/humanoid_carry_demo.py`
  does: sample it and ask `check_collisions()` — the demo refuses to bake
  a ramp that sweeps through the cell.

## From the catalog

A legged machine in the catalog is a `vehicle.legged` package, and its
manifest carries the gait: which links are the feet, the stance it
stands in, the rhythm it was validated to walk with (the catalog builder
stands every such package on its stance and walks it through a pivot
before it ships). `Gait.from_catalog` reads that block, so a cell copies
no joint name out of a URDF:

```python
dog = bt.Robot.from_catalog("unitree/go2/go2")
gait = bt.Gait.from_catalog("unitree/go2/go2", period=0.5)   # keywords override
scene.add_robot(dog, name="go2")
scene.mount_robot("walker", robot="go2", gait=gait)
```

A package directory on disk works the same way — `bt.Gait.from_catalog(
"build/unitree/go2/go2/r1")` reads the `manifest.yaml` a local `bcb build`
wrote — and a package that is not legged is refused by name. Beside the
gait, the manifest's `specs` say what the cell needs to size the body the
gate sees (`footprint_mm`, `height_mm`) and cap the vehicle's speed
(`max_speed_mps`); `examples/legged/legged_patrol_demo.py --compare` bakes one
cell on every package named and tables which fit and how long they take.

## Stairs and steps

A staircase is authored as geometry, not as a special move: mark an
upright box's top face *walkable* (`scene.set_obstacle_walkable(name)`) and
footfalls snap onto it. `bt.parts.stairs` builds a whole flight that way —
every tread a walkable box, stringers, legs and the handrail ordinary
obstacles — with the frames `<name>/foot` and `<name>/top` to author the
guide path's z between (the path climbs as a slope; the *feet* land on the
treads it interpolates past). `catalog=` orders the flight instead of
drawing it, and the sizes then come from what the maker sells. The swing
clears the higher tread by the authored `lift` automatically.

**The body rides the steps, not the guide line.** Two things happen to it
on a flight, and both are about reach rather than looks:

* it **pitches** to the grade of the path under it (blended over one body
  length so nothing snaps, level again when it parks) — a level body asks
  its downhill legs for half a machine-length of slope *below* the hips,
  which is more than a real leg has;
* it **rises** with the feet — the height it holds follows where its feet
  actually are (a swinging foot counts along its arc, so the body glides
  rather than steps), instead of holding one height over the straight line
  the route draws. Without it a single stance has to serve both extremes
  at once: a leg reaching down to a low tread while another folds up under
  a high one.

Together they roughly double the riser a given machine can take. Wheeled
vehicles are unchanged — their bodies stay level.

**Take the stair posture.** A machine walks a flight lower than it stands,
and picks its feet up less. Those two numbers decide the riser it can take
at all, they belong to the machine, and a package that has been measured on
a flight carries them:

```python
gait = bt.Gait.from_catalog("unitree/go2/go2", posture="stairs")
```

`bt.Gait.postures(package)` says whether there is one — `("stairs",)` or
`()` — so a cell that can do either asks first rather than handling a
refusal. Asking for a posture a package does not carry is refused by name;
the cell then states the stance and lift itself
(`bt.Gait.from_catalog(..., stance=…, lift=…)`). For a Go2 on a 150 mm
riser that is roughly 0.25 m of stance depth (against 0.311 standing) and
a 15 mm swing. Get it wrong and the bake says so, with the distance the
leg was short by.

Stating it yourself, what is worth writing down is the *depth*, not the
joint angles: which fold puts a foot 0.25 m under the body depends on the
legs, and a standing stance is rarely a clean one to scale (the Go2's is
thigh 0.8 / calf −1.5, its foot 0.178 m ahead of the hip). That is why the
angles live in the package, which knows the legs.
`examples/legged/stairs_delivery_demo.py` does both: it asks the package for the
posture, and where there is none it solves the fold by standing a scratch
copy of the machine and reading a foot — so the same cell posture-fits
whatever walks it, and the primitive `quad_test.urdf`, having shorter legs,
is told to take a lower flight.

Walkable excuses exactly one thing: the machine walking on it. Its own
aisle check and its rider check treat treads the way no one
collision-checks a floor against the machine standing on it — while an
AGV driven into the flight, or an arm sweeping through it, still collides
by name.

Two checks come with the terrain, both at dispatch, both naming the
offender:

* **Step height** — `bt.Gait(max_step=...)` declares the tallest rise one
  leg may take (a catalog package fills it from `max_step_height_mm`).
  A flight over the ability is refused with the leg and both numbers;
  without a declaration the IK (`GaitReach`) is the backstop.
* **Tread edges** — a foothold closer to a tread's edge than the foot
  radius is refused with the tread's name and the margin, before anything
  walks half-on, half-off a step.

## Reading the bake

`timeline.footfalls(robot)` lists every step as `(leg, lift, land,
(x, y, z))`; `timeline.base_pose(t, robot)` carries the body's sway. The
cycle time is the vehicle's: walking costs no time over driving, which is
to say a legged robot is budgeted like any other vehicle, by its path and
its speed.

Script export (`to_script`, `export_script`) refuses a walking robot — its
joints are its legs, and no controller takes a gait as a program. The cycle
belongs to the PLC side (`export_plcopen`), where the vehicle is a device
like any other.

## Examples

* `examples/legged/legged_patrol_demo.py` — a Unitree Go2 (the catalog package
  `unitree/go2/go2` when it is reachable, else Unitree's URDF with the
  meshes converted on first run) walks in through a gate, docks, has a
  part placed on its back by an arm, and carries it to the bay. `--robot
  <dir>` runs it on a package the catalog builder wrote, `--robot quad` on
  the primitive quadruped in `examples/assets/quad_test.urdf` with no download;
  `--narrow` shows the gate check failing; `--compare <dir> ...` tables
  every candidate.
* `examples/legged/humanoid_carry_demo.py` — a Unitree G1 picks a tote off a
  bench, carries it to another, sets it down and walks back. `--robot
  biped` runs it on `examples/assets/biped_test.urdf`.
* `examples/legged/stairs_delivery_demo.py` — a quadruped carries a tote up a
  `bt.parts.stairs` flight to a mezzanine: footfalls on the treads, the
  body on the slope. `--tall` raises the risers over the gait's
  `max_step` and shows the refusal, named.
