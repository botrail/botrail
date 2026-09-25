# Physics

A cell is authored kinematically: the plan says where every joint is, the
devices carry what their zones hold, a grasp is a rigid attachment. That
is the right world for teaching, planning and sequencing, and it is
deterministic to the bit. Physics is the second opinion — the same cell
handed to a rigid-body engine, so what the authoring *implied* gets
checked: does the workpiece really sit in its tray, does the carton
someone left in mid-air fall, does the arm hold its pose, does the part
ride the belt into the beam, is the grasp where the fingers are.

There are two ways to ask for it.

## The declared scope

Mark what the engine should own and bake with `physics=True`:

```python
scene.set_physics("carton", dynamic=True, mass=0.4)   # a part the engine drops, pushes, carries
scene.set_robot_physics("ur")                        # the arm as an articulated body
timeline = scene.simulate_sequence("cycle", physics=True)
```

Everything else stays what the authoring says — a *kinematic mirror* the
engine sees at its planned pose every tick, pushing with infinite mass
and never pushed back. This is the scope the [RL guide](rl.md) uses: a
few parts and a robot are physical, the cell around them is scenery.

## The world scope

`bt.Physics(world=True)` hands the engine the whole cell and nothing is
marked:

```python
physics = bt.Physics(world=True)
print(scene.physics_plan(physics=physics).to_markdown())   # what the bake will do
timeline = scene.simulate_physics(4.0, physics=physics)    # no program: the cell under gravity
timeline = scene.simulate_sequence("cycle", physics=physics)   # the programs on the physical cell
```

Every obstacle folds into a rigid *unit* by its name hierarchy and part
identity — a pallet with its timber, a workpiece with its display shell —
and a unit stays put only when authoring or identity says it is bolted
down: an equipment part pin (`structure.*`, a conveyor, a rack), a device
that moves it by name, a walkable floor, an explicit
`set_physics(name, dynamic=False)`. Everything else is the engine's, a
pallet on the floor included — it rests because the ground holds it, not
because a rule froze it. A ground plane at z = 0 catches what falls
(`ground=` moves it, `anchored=False` unbolts the equipment too). What a
program picks up *on its own* — a case off a stack — is a unit of its
own; what a program takes *together* in one step (every board and case
of a pallet, attached to a lift at once) stays one rigid unit and is
carried whole.

The programs' time and the world's are two things. A bake ends when
every program has ended, or at `max_duration` — the programs' cap, which
turns a step that waits for a signal that never comes into a *timed out*
diagnosis instead of a hang. A part still in the air when the last
program ends is frozen there unless the world is given a tail:
`bt.Physics(settle=3.0)` keeps the engine running after the programs
until everything it owns has been at rest for a quarter second, or the
three seconds run out. The cap does not count the tail. A bake with no
program at all (`simulate_physics`, the studio's physics toggle with
nothing baked) has no cap: it runs for the seconds asked, or in the
studio until it is switched off.

Every robot is an articulated body: each link a rigid body weighing what
its model states, each joint a force-capped servo. Whether the servos are
on is the bake's business (`powered=`): with no program at all the
machine is unpowered and folds at its joints under gravity; under
programs, a robot is powered where a program drives it — a motion, a ramp,
a track, a grasp of its — and unpowered where none does, so the idle
machine of a cell folds while its neighbour works. `powered=True` switches
every servo on, `powered=False` is a power cut mid-program. A walking
machine's base floats (it stands on its feet, or falls); an arm's stays
bolted to its stand.

`scene.physics_plan()` is the contract: one row per unit and per robot,
what it is (ground, fixed, dynamic, robot), why, its mass, and the
warnings — a unit authored overlapping another will be pushed out, a
model that states no plausible effort limit gets a default cap. Read it
before the first bake; `set_physics` overrides any row.

## What runs the same, and what physics changes

The programs run as authored: the scan loop, the sensors, the step
transitions, the device commands are the same code path. What the engine
takes over:

* **Parts** move by contact. A part on a belt rides it by friction (the
  zone's surface velocity), a bumped part is pushed, a released part falls
  with the hand's velocity. A tray, lift or sink zone does not *capture* a
  physical part the way it captures a kinematic one — a tote on an AMR
  deck rides by friction instead.
* **Robots** follow their plans through servos, so the baked robot lane
  is the physical joint state read back every tick: a lag of a degree
  during a fast move, a payload that makes the arm trail. Tracking a
  moving part works (the per-tick solve is what the motors are told).
* **Grasps** mean what they say: without a gripper drive declared, the
  part rides the hand — a mirror the FK places between the fingers, which
  it does not fight — until `detach` hands it back. Declare a gripper
  drive (`set_gripper_drive`) and a grasp is a hold *declaration*: the
  fingers' force and friction carry the part or drop it, and
  `timeline.grasp_report()` measures the slip.
* **The timeline** says which engine ran it (`timeline.physics`), which
  scope (`timeline.physics_scope`), every touch as a contact episode
  (`timeline.contacts`), and when each part came to rest
  (`timeline.settled_at(name)`).

The flagship pick cycle (`examples/basics/sequence_demo.py`) runs both
ways: kinematically in 16.69 s, under the world scope in 16.78 s, the
same thirteen steps, the box on the pallet either way. The warehouse
cycle (`examples/vehicles/warehouse_demo.py` — two pallet AMRs, a picker,
a traffic interlock, 5,900 bodies) runs its four programs to the end under
the world scope too, the lifts taking their pallets up by contact.

## The servo

A dynamic joint's servo is the cascade of an industrial drive: the
command's rate fed forward, a position loop on the error, a bounded
integral trim for the last milliradians under load, and the engine's
motor as the velocity loop under the force cap. `max_force` is the cap
(the model's effort limit, or a default flagged in the plan when the
model states none — or a "no limit" figure a hundred times the robot's
weight, which some vendor USD files do), the joint's velocity limit is its
rated speed, `armature` the drive's reflected inertia. A stated inertia
tensor implausibly small for a link's mass and shape is replaced by the
shape's; the impulse solver needs a chain whose inertias are within an
order of each other, and a microgram-scale hand between kilogram links
rings.

The knobs are on the declaration, per robot:

```python
scene.set_robot_physics("panda", max_force=87.0, armature=0.05, mass_floor=0.2)
```

## The studio, and a hand on the world

The **⚛ physics** toggle bakes the last request again under the host's
physics and streams it as it grows; see [the studio](studio.md).
`bt.studio(scene, physics=bt.Physics(world=True, powered=False))` decides
what "on" means for that session. With no program the world streams
live, and a body in the viewport can be taken in hand and pulled — the
mouse pick of a physics viewer: a critically damped spring between the
grabbed point and the mouse, capped at a few g so nothing gets launched.

The same hand is a Python call on a live rollout — the world with no
program is `open_rollout([], physics=...)`:

```python
live = scene.open_rollout([], physics=bt.Physics(world=True))
live.drag("crate", point=(0, 0, 0.15), target=(0.5, 0.0, 0.4))   # hold the crate's top, pull it here
live.tick(120)
live.release()
```

`drag` returns whether the body is the engine's to move: bolted equipment
and a part a program holds are not. A robot link is named `robot/link`.

## Another engine

The world `physics_plan()` tabulates can leave botrail as a UsdPhysics
stage — robots as articulations, dynamic units as rigid bodies, what a
device moves as kinematic bodies, belts with a surface velocity — for Isaac
Sim or Isaac Lab to own:

```python
scene.export_usd("cell.usdc", physics=bt.Physics(world=True))
```

See [USD for a physics engine](export.md#usd-for-a-physics-engine-isaac-sim-isaac-lab).

## Not simulated

A part in a dynamic robot's hand rides as a mirror: it does not load the
arm with its weight (a welded part at bake start does). Deformables,
fluids, and a robot's self-collision are outside the engine's world. The
declared scope's semantics are unchanged by the world scope; a project
that never asks for physics is unchanged to the bit.
