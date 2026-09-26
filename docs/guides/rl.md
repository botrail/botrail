# Reinforcement learning

A botrail cell is already a simulator with a clock: the sequencer scans,
the drives move, physics settles the parts, sensors read. `botrail.rl`
opens that same rollout one tick at a time and lets a policy drive one
robot between scans — so an environment is *derived* from the cell you
would hand over, not modelled beside it. What the policy learned in then
runs back inside the cell as an ordinary sequence step.

```bash
pip install "botrail[rl]"     # numpy + gymnasium (gymnasium is optional)
```

## An environment from a cell

Three things make one: a function that builds the scene, a `Task`, and
`rl.make`.

```python
import botrail as bt
import numpy as np
from botrail import rl

def build() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf("simple_arm.urdf"))
    scene.add_box("table", size=(0.6, 0.6, 0.1), position=(0.55, 0.0, 0.05))
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(0.55, 0.0, 0.135))
    scene.set_physics("part", dynamic=True, mass=0.2)
    scene.add_box("goal", size=(0.02, 0.02, 0.02), position=(0.45, 0.1, 0.5))
    scene.set_obstacle_enabled("goal", False)          # a picture, not a thing
    scene.sequence("run").step("wait", transition=bt.seq.elapsed(30.0))
    return scene

def randomize(scene: bt.Scene, rng: np.random.Generator) -> None:
    scene.set_obstacle_pose("goal", (0.4 + rng.uniform(-0.05, 0.05), rng.uniform(-0.15, 0.15), 0.5))

task = rl.Task(
    control=rl.TcpDelta(max_step_m=0.02, hz=20, frame="tcp"),
    observe=[rl.Joints(), rl.TcpPose(), rl.Relative("tcp", "goal"), rl.Collision()],
    reset=randomize,
    reward=rl.rewards.combine(rl.rewards.reach("tcp→goal"), rl.rewards.collision_penalty(1.0)),
    done=rl.rewards.within("tcp→goal", 0.02),
    horizon_s=5.0,
)

env = rl.make(build, task, seed=0)
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
```

`build` runs once (pass `rebuild=True` to build every episode); `reset`
re-randomises the built scene with the seeded generator — move things,
recolour them, re-aim a camera — and the episode opens a fresh rollout from
it. The cell's sequences run underneath: a belt keeps feeding, a sensor
still trips, and `Collision()` reads what the scan-tick safety check found.
An episode truncates at `horizon_s` or when every sequence has run to its
end; the last one closes into a timeline (`env.timeline()`) that plays in
the studio and exports as USD like any bake.

`Task.settle_s` runs the world for that long after each reset, with the
drive holding the robot where it stands and no action taken, before the
first observation: what the randomisation set down comes to rest (a
part dropped onto a table, a pool of things tipped into a bin) instead
of being observed mid-fall. The episode's `horizon_s` counts from that
first observation; `info["t"]` is the rollout's clock, settling included.

`rl.make(..., num_envs=N)` returns a vectorised environment: `N` worlds
stepped together in Rust with the GIL released, one thread each. A single
environment and a world of the vector are **bit-identical** under the same
seed — see [determinism](../concepts/determinism.md) — so a policy
trained on the vector replays exactly in the single one.

## Controls

The action is what the policy commands every control period (`hz`); the
drive's own rate limit (`max_velocity`, the robot's limits by default)
turns it into a servo move.

| Control | Action | Notes |
| --- | --- | --- |
| `JointDelta(max_step, hz)` | joint increments in `[-1, 1]` × `max_step` | the simplest, every robot |
| `JointTarget(hz)` | absolute joint targets, normalised to the limits | |
| `TcpDelta(max_step_m, max_step_rad, frame, hz)` | a Cartesian step of the TCP (`frame="tcp"` or `"world"`), `rotate=True` adds a rotation | IK solved in Rust per step; `info["ik_failed"]` marks a step the workspace refused, the setpoint holds |

`Task.robot` / `Task.group` pick which robot (and which group of a
dual-arm) the policy drives; the others follow their programs.

## Observations

State channels read the world between ticks; each has a `key` in the
observation dict (`rl.make(..., flatten=False)`) and a slice of the flat
vector otherwise.

| Channel | Reads |
| --- | --- |
| `Joints()`, `TcpPose()`, `LinkPose(link)` | the driven robot |
| `ObjectPose(name)`, `ObjectVel(name)` | a dynamic part |
| `Relative(a, b)` | `b` seen from `a` (TCP frame when `a` is `"tcp"`) |
| `Contacts(a, b)`, `Clearance()`, `Collision()` | the physics and the safety check |
| `Signal(name)` | a sequencer signal |

### Sensors

The same cell's sensors, read on the live world and packed in Rust:

```python
observe=[
    rl.Lidar("front", stride=2, noise=0.01),                 # ranges, or points=True for xyz
    rl.Depth("wrist", size=(64, 64), noise_z2=0.002, baseline=0.05, dropout=0.02),
    rl.Segmentation("wrist", size=(64, 64)),
    rl.PointCloud("wrist", size=(32, 32)),
    rl.Rgb("wrist", size=(64, 64)),
]
```

* **`Lidar`** casts the scanner's beams against the *collision* shapes
  (what the cell can hit), every `stride`-th azimuth of the chosen
  `rings`; a beam with no return reads the scanner's maximum range.
  `points=True` gives the hit points `(beams, 3)` in the scanner frame
  instead. Gaussian range `noise` (1σ m) comes from the world's seed and
  the tick — the same seed, the same draw.
* **`Depth`** is a pinhole picture from a scene camera rendered by the
  rollout's own rasteriser: Z along the optical axis, `0` for no return or
  outside `[near, far]` — the studio's depth capture, pixel for pixel. Its
  sensor model is a stereo camera's: noise `noise + noise_z2·z²` (the
  error grows with the square of the range), disparity quantisation for a
  `baseline` at the picture's focal length (the depth steps a real pair
  shows on far surfaces) and a `dropout` share of pixels lost. Leave them
  all unset for the clean render.
* **`Segmentation`** is the body id per pixel (`rl.segmentation_ids(scene)`
  names them); **`PointCloud`** the camera-frame point of every pixel.
* **`Rgb`** is the flat-shaded colour picture: displayColor and shape under
  one directional light plus ambient, hard shadows on request, no
  textures. It is a training signal, not a rendering — the studio and the
  USD export carry the photoreal look.

`geometry="collision"` draws the collision shapes instead of the visual
meshes. Pictures share one render per camera and size per step, so a
depth, segmentation and point cloud of the same camera cost one raster.

### Domain randomisation

`Task.reset` moves and recolours things. `Task.render` sets how the
pictures are drawn, per episode — a dict, or a function of the world's
generator:

```python
def render(rng: np.random.Generator) -> dict:
    return {
        "light": tuple(rng.normal(size=3)),   # a direction toward the light
        "ambient": float(rng.uniform(0.2, 0.5)),
        "shadows": True,                       # a shadow map per picture
        "decimate": 0.01,                      # visual meshes cut to 1 cm cells
    }

task = rl.Task(..., render=render)
```

`shadows` fits an orthographic shadow map to what the picture shows and
costs about a second picture; `decimate` clusters a dense CAD mesh's
vertices into cells of that size — a policy reading 64 pixels a side does
not miss the detail. Left unsaid, a task with a picture channel decimates
to half a pixel's footprint at one metre of its finest picture
(`tan(fov/2) / width`, about 1 cm for 64 pixels over 70°): a catalogue
arm's CAD-dense visual meshes otherwise cost a 64-pixel depth picture ten
times the state observation. `render={"decimate": None}` keeps every
triangle. `scene.set_camera_pose(name, position, quaternion)` in `reset`
jitters a world camera; a wrist camera rides its link.

`rl.randomize` holds the draws a tabletop cell makes every episode, over
the same setters, on one prefix of a tiled scene or none:

```python
TOP = 0.75                                     # the table's top face

def randomize(scene, rng, prefix="", origin=(0.0, 0.0)):
    ox, oy = origin
    drawn = rl.randomize.pool(scene, rng, OTHERS, 4, prefix=prefix)       # 4 of the pool join the can, the rest parked
    region = ((ox + 0.24, ox + 0.52), (oy - 0.28, oy + 0.28))
    rl.randomize.scatter(scene, rng, ["can", *drawn], region, TOP, prefix=prefix)   # footprints 2 cm apart
    rl.randomize.friction(scene, rng, THINGS, 0.4, 0.9, prefix=prefix)
```

`pool` draws `k` of a pool of things and parks the rest — disabled,
hidden, out of the physics and the pictures until a later draw brings
them back (Isaac Lab's `MultiAssetSpawnerCfg(random_choice=True)`, on a
cell whose residents are all built once); given spots and the face they
are on, it also sets the drawn things down on them. `scatter` sets things
down in a region with their footprints kept `gap` apart, biggest first,
starting over when a layout boxes the next thing in — so the physics
plan's overlap audit stays empty. Both read each thing's footprint and
underside off its bounds, not its origin: a primitive and a scanned
object whose frame sits anywhere are set down alike. `friction` draws a
coefficient per thing.

## A cell, tiled

The picture of an environment being learned — a grid of copies of one
cell, a policy running in every one — is the same cell written once as
a function of a scene, a name prefix and an origin:

```python
def cell(scene: bt.Scene, prefix: str, origin: tuple[float, float]) -> str:
    ox, oy = origin
    bt.parts.table(scene, prefix + "table", size=(0.9, 0.7, 0.75), position=(ox + 0.55, oy))
    bt.parts.table(scene, prefix + "stand", size=(0.6, 0.5, 0.75), position=(ox - 0.2, oy))
    name = scene.add_robot(ROBOT, name=prefix.replace("/", "_") + "fr3", base_position=(ox - 0.2, oy, 0.752))
    scene.set_robot_physics(name)
    scene.add_box(prefix + "can", size=(0.06, 0.06, 0.1), position=(ox + 0.5, oy, 0.802))
    scene.set_physics(prefix + "can", dynamic=True, mass=0.35)
    return name

env = rl.make(rl.single(cell), task, num_envs=16)       # learning: 16 independent worlds
tiled = rl.tile(cell, rows=2, cols=3, spacing=(1.9, 1.6))   # watching: six copies in one scene
tiled.randomize(randomize, seed=0)                       # the same draws, per copy
timeline = rl.play(tiled, task, model, duration_s=6.0)  # the policy in every copy at once
timeline.export_usd("tabletop.usda")
```

`single(cell)` is a `build` for `make`: the cell at the origin with no
prefix, on a floor slab, under a wait program. `tile` lays copies out on
a grid, each named `c{i}/` with the robot the cell returned, and `play`
opens the grid's rollout, binds the task's control to every robot,
relocates the task's channels into every copy (`ObjectPose("can")` reads
`c3/can` in copy 3, `Contacts("fr3/finger", "can")` that copy's finger)
and asks the model for each copy's action every control period until
`duration_s`, the task's `done` in every copy, or the programs' end. The
grid is one physical world on one thread — for learning, `make(...,
num_envs=N)` steps `N` worlds in parallel; for the picture and the USD,
the grid — and the grid's USD is a few megabytes, not six times one
cell's: every mesh is written once beside the layer and referenced
([export](export.md)). The robot's base goes a couple of millimetres above the face
it stands on: seated exactly on it, the base link and the top touch at
zero distance and every scan flags a collision. `examples/rl/tabletop_env.py`
is the whole of it — a Franka on a stand with a tread-plate top, a timber
work table, a KLT (`bt.parts.bin`, the catalog's `botrail/bin/klt-vda4500`
in the real-object cell: five boxes pinned as one part, so the world scope
keeps them one rigid unit), a tray, a pool of things drawn every
episode — with `--tile 2x3`. The wrist camera is the catalog's RealSense
D405 (`add_camera(from_catalog=...)`: the gripper camera of the family,
its optics and its BOM line), seated 30° toward the fingertips on a
printed clip that hugs the hand — hooked under the coupling ring, flat
on the broad face, hooked under the bottom edge. The clip is a shape
boxes cannot draw, so it is authored in Node + three against the hand's
measured surfaces (botrail-assets `franka-hand-d405-clip`), vendored as
`examples/assets/franka_hand_d405_clip.usda` and drawn on a resident that
does not collide (`set_obstacle_visual_asset`); what collides is two
hidden boxes and the camera's own mesh. All of it is attached to the hand
with `scene.attach` and pinned as one `adapter` part, so the planner and
the engine keep the camera out of the bin's walls, a physics bake welds
it to the hand, and Isaac Lab gets it as a body on a fixed joint.

### Real objects on the table

The pool is the YCB Object and Model Set (CC BY 4.0) from the catalog:
the scanned, textured models under `ycb/objects/*` — a tomato soup can, a
mustard bottle, a cracker box, a mug, fruit, clamps, 30 of them — at the
masses the set measured. `bt.parts.prop` places one as a single dynamic
obstacle:

```python
bt.parts.prop(scene, prefix + "can", (ox + 0.3, oy, 0.752), catalog="ycb/objects/005-tomato-soup-can")
```

It is drawn with its texture and collides as the convex decomposition of
the scan; the catalog package has moved the scan's origin to the middle
of its footprint on its underside, so the can's observed pose
(`ObjectPose("can")`) is the middle of the can on the table. The colour
pictures draw it flat in the pack's mean albedo (`specs.albedo_rgb`) —
the texture is the studio's. A package is 8–18 MB (the 4096² scan
texture is kept as the source; what botrail draws carries a 1024² copy),
downloaded once; `--shapes` builds the cell from primitives of the same
sizes and masses instead.

### The cell in Isaac Lab

The same cell goes to Isaac Lab as a simulation stage
([export](export.md#usd-for-a-physics-engine-isaac-sim-isaac-lab)):
`examples/export/isaaclab_tabletop.py export` writes it with
`scene.export_usd(path, physics=bt.Physics(world=True))` — the arm an
articulation with the servo botrail authored (limits, drive gains and force
caps, the hand's mimic finger, the exported joint positions), every thing
on the table and the KLT rigid bodies at their masses and friction, the
bench and the stand static colliders, every mesh once beside the layer —
and `... run --headless --num_envs 16` opens it in Isaac Lab, clones it
with `env_spacing` and steps it on the GPU (checked on Isaac Lab 2.3 /
Isaac Sim 5.1: the arm starts in its exported pose and follows joint
targets, the hand closes with its follower, the things settle where they
were set down, sixteen environments agree). What crosses over is the
cell; the task — observation, reward, termination, the episode's
randomisation — is stated again on the other side in Isaac Lab's own
environment configuration, and the pictures are the engines' own (the
finishes are the studio's, the scans' textures ride with the catalog).

## The policy back in the cell

A trained model runs inside a sequence as a `policy` step — the robot's
drive belongs to the step while it runs, the belt and sensors keep going,
and the bake reports cycle time, contacts and collisions as for a taught
motion:

```python
sq = scene.sequence("cycle")
sq.step("feed", actions=[bt.seq.set("belt_run", True)], transition=bt.seq.signal("at_stop"))
sq.step("reach", actions=[bt.seq.policy("reach", hz=20, max_duration=6.0)], transition=bt.seq.done())

policy = rl.load("reach.onnx", scene, task)      # .onnx, an SB3 .zip, or a callable
timeline = scene.simulate_sequence("cycle", physics=True, policies={"reach": policy})
```

A collision under a policy step is a bake error, the step exports to
PLCopen XML as `FB_StartPolicy`, and the studio labels it in the sequence
chart. See `examples/rl/` for a reach task, a wrist-camera pick and the
loop back into the cell.

## Dynamic robots and torque control

By default the robot is a *kinematic* body under physics: it goes exactly
where the plan says and pushes with infinite mass. Declare it dynamic and
the bake simulates it as an articulated body:

```python
scene.set_robot_physics("ur")                 # or robot=None for the only robot
scene.set_robot_physics("ur", max_force=80.0, armature=0.2)
```

Every link with geometry becomes a rigid body weighing what its model
states (URDF `<inertial>`, or the catalog package's `PhysicsMassAPI`; a
link without either gets its shape at the default density, at least
`mass_floor` kg; a stated inertia tensor implausibly small for the link's
mass and shape — some vendor USD files state microgram-scale tensors on
kilogram links — is replaced by the shape's, which the impulse solver
needs to keep the chain conditioned). Every joint becomes a force-capped
servo — `max_force` is the cap (default the URDF effort limit, unless it
is a "no limit" figure a hundred times the robot's weight), the joint's
velocity limit its rated speed, `damping` the velocity loop's gain —
following the planned motion, and the baked robot lane is the physical
joint state read back every scan: a heavy payload makes the arm trail and
droop, a collision stops it. `armature` is the reflected drive inertia
per joint (a geared motor's rotor through the gear ratio squared; default
0.1 kg·m² about a revolute joint, added to the link's tensor, and 1 kg
along a prismatic one, added to its mass), which also keeps a light wrist
from drooping under the engine's impulse-solved motor. The base assembly
stays on its stand or vehicle. Without a physics backend the declaration
is inert, and a robot never declared is unchanged to the bit.

`Task.physics` takes the bake's argument: `True` is the declared scope
(the parts and robots declared physical, the rest kinematic scenery),
`bt.Physics(world=True)` the whole cell under physics with a ground
plane (see [physics](physics.md)) — in a `VecEnv` as in a single one.
Under the world scope a robot no program drives is unpowered and folds;
the environment's drive is a driver, so the task robot's servos switch
on when the episode opens (where the arm stands, fallen or not). Other
machines in the cell stay unpowered unless `bt.Physics(world=True,
powered=True)` powers them all.

The position controls (`JointDelta`, `JointTarget`, `TcpDelta`) work on a
dynamic robot as they do on a kinematic one — the drive rate-limits the
command and the servos follow it. `Torque` hands the policy the joint
torques themselves:

```python
task = rl.Task(
    control=rl.Torque(hz=50),                 # action = torque / max_torque per joint
    observe=[rl.Joints()],                    # the physical q and q̇
    reward=lambda obs, info: -abs(obs["ur/joints"][:6] - target).sum(),
)
```

While a torque stands the joint's servo is off, so gravity is the policy's
to hold; a position command (or `undrive`) hands the joint back to its
servo, which brakes it at the cap and returns at the rated speed.

Gravity compensation is a property of a robot's controller, not of the
physics, so the simulator does not apply it on its own. The servo is the
cascade of an industrial drive — the command's rate fed forward, a
position loop on the error, a bounded integral trim for the last
milliradians under load, the engine's motor as the velocity loop — and
holds a load with no model knowledge; a torque interface whose firmware
compensates is modelled by
`Torque(gravity_compensation=True)`, which adds the model's gravity
torque on top of the action every tick (never past the joint's cap), and
the model's gravity torques are readable as an observation
(`GravityTorque()`) or from a controller (`live.gravity_torques()`) for a
gravity-only or hand-guiding mode of your own. Grasped parts are not the
robot's mass and are not included. A high-gear-ratio industrial arm is
best described with a larger `armature` (1 to 5 kg·m²); a backdrivable
collaborative arm with a small one, which is where torque control means
something.

A walking machine can be dynamic too: its legs stay the gait's kinematic
mirrors (the walk is planned, not simulated — see [legged
robots](legged.md)), and everything else the declaration covers — a head,
a waist, the arms — is a servoed body riding the walk, with the gait's arm
swing as its command. Torques reach those joints, never a leg.

A dynamic robot runs the same programs as a kinematic one. It tracks a
moving part (the per-tick solve is what its motors are told), and a
grasp means what it says: without a gripper drive declared, the part
rides the hand — a mirror the FK places between the fingers, which it
does not fight — until `detach` hands it back to physics with the hand's
velocity. Declare a gripper drive (`set_gripper_drive`) and a grasp is a
hold *declaration* instead: the part stays physics' own, the fingers'
force and friction carry it or drop it, and `grasp_report` measures the
slip; a motion planned during the hold sees the part as carried either
way.

## Cost

Measured on one machine, 32 worlds, a six-axis arm with a dynamic part,
20 Hz control over a 5 ms scan (`scripts/bench_vec_env.py`):

| Observation | env-steps / s |
| --- | --- |
| state only, `TcpDelta` | 11 k |
| + 541-beam LiDAR | 6 k |
| + 64 × 64 depth | 8 k |
| + 64 × 64 RGB | 7.5 k |
| + 64 × 64 RGB with shadows | 5 k |
| dynamic robot, `TcpDelta` | 8 k |
| dynamic robot, `Torque` | 7 k |

The tabletop cell of `examples/rl/tabletop_env.py` — a catalogue FR3
with its hand, eight dynamic things, 16 worlds on the same machine —
runs at about 2.2 k env-steps a second under position control and
3.3 k under torque control, 2.6 k with the arm kinematic: the mirrored
links and the things' contacts are the cost, the servos a sixth of it.
With the YCB scans instead of primitives (convex decompositions touching
each other and the table) the joint-step control runs at about 2.9 k
against 5.2 k for the primitives.

The rasteriser is CPU code — no GPU, nothing to install, deterministic —
and it is the right tool up to a few hundred pixels a side. Photoreal
observations (textures, materials, global light) are out of its scope: for
those, export the cell to USD and render it elsewhere.
