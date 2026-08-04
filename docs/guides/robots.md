# Robots

A [`Robot`][botrail.Robot] is a parsed kinematic model — tree, joints, limits,
geometry. It is immutable and holds no pose: joint values live in the
[`Scene`](scene-and-obstacles.md) that owns it, which is also why the same
model can be added to a scene more than once.

## Three ways in

```python
robot = bt.Robot.from_urdf("arm.urdf")
robot = bt.Robot.from_xacro("arm.urdf.xacro")
robot = bt.Robot.from_usd("franka.usd")
```

**URDF** — mesh paths resolve relative to the file, and `package://` URIs are
resolved heuristically, so most real-world URDFs load without a workspace.
There is also `from_urdf_string(xml)` for generated descriptions.

**Xacro** — expanded without ROS: properties, macros, includes, and
conditionals all work. Most real robot descriptions are xacro, and needing a
ROS installation just to expand them is the usual dead end for ROS-free tools;
botrail ships the expander.

**USD** — a `UsdPhysics` articulation, e.g. an Isaac Sim asset. Link and joint
names are the prim paths (`/panda/panda_hand`), revolute limits are converted
from degrees, distances from the stage's `metersPerUnit`, and Y-up stages are
re-modeled as Z-up. `articulation_root` defaults to the first prim carrying
`PhysicsArticulationRootAPI`; `search_paths` resolves external
(`omniverse://`) references against local directories. Anything skipped during
import is printed, not swallowed.

## What a model knows

```python
robot.dof            # actuated joints
robot.joint_names    # in q-vector order — every `positions` list uses this order
robot.joint_limits   # (lower, upper) per joint, None for continuous
robot.link_names
robot.tcp_link       # deepest leaf; the default end-effector link
```

## Mimic joints

Joints that follow another joint — URDF `<mimic>`, USD `PhysxMimicJointAPI` —
never appear in `joint_names` or in a position vector. A two-finger gripper
with a mimicked second finger costs **one** DOF, not two:

```python
robot.mimic_joints          # {joint: (source joint, multiplier, offset)}
robot.joint_values(q)       # every joint's value, mimics resolved, fixed = 0
```

`joint_values` is the bridge to consumers that want *all* joints (a firmware
interface, an animation rig): it expands a DOF vector into a per-joint map with
the mimic relations applied.

!!! note "When a mimic is authored wrong"

    The Isaac Franka authors its finger mimic about a different axis than the
    joint actually moves on. botrail refuses to guess: it prints
    `mimic joint authored on `rotX` but the joint moves about `transX`; ignored`
    and keeps both fingers as independent DOF. If your vector is one longer
    than you expected, read the import notices.

## IK without a scene

The model solves IK on its own — useful for reachability studies before any
cell exists:

```python
ik = robot.ik((0.4, 0.1, 0.5))            # position only
ik = robot.ik((0.4, 0.1, 0.5), quaternion=(0, 1, 0, 0), link="/panda/panda_hand")

ik.converged     # always check — the solver returns its best effort
ik.q             # best configuration found, always within limits
ik.pos_error     # m
ik.rot_error     # rad
```

`link` defaults to the TCP link and `seed` to the neutral configuration.
Inside a scene, [`set_tcp_target`][botrail.Scene.set_tcp_target] is the same
solve seeded from the current pose — and applied.

## Several robots in one cell

```python
scene = bt.Scene(robot, name="near")
scene.add_robot(robot, name="far",
                base_position=(1.2, 0.0, 0.4),
                base_quaternion=(0.0, 0.0, 1.0, 0.0))   # facing back

scene.robots               # ['near', 'far'] — instance names, insertion order
scene.robot_of("far")      # the model behind an instance
scene.joint_positions_of("far")
```

Instances are what everything else addresses: methods take `robot=` (defaulting
to the first robot), motions belong to an instance, and USD exports place each
one under `/World/<instance name>`. `rename_robot` renames safely — sequence
actions, `robot_done` conditions, and sensor watch lists follow the new name.

Planning with several robots is per-robot with the others frozen as obstacles;
the sequence rollout then re-checks robot-against-robot every tick. The
[Two arms, one belt](../tutorials/two-robots.md) tutorial shows the full
pattern, including the interlocks.
