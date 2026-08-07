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
Meshes load from STL and OBJ; an OBJ that names an `mtllib` keeps its
material colors, in the studio and in exported USD alike. There is also
`from_urdf_string(xml)` for generated descriptions.

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

## The model catalog

Instead of hunting down URDFs, load released packages straight from the
[botrail catalog](https://huggingface.co/datasets/botrail/botrail-catalog)
(needs the optional extra: `pip install botrail[catalog]`):

```python
robot = bt.Robot.from_catalog("2f-85")                       # newest revision
robot = bt.Robot.from_catalog("robotiq/2f/2f-85/r1",
                              revision="<dataset commit sha>")  # pinned
```

Ids resolve exactly or by any unambiguous shorthand (`2f-85`,
`robotiq/2f-85`). An id ends in a revision (`.../r1`), and when a shorthand
matches several revisions of one product the newest wins — a revision is the
same machine re-cut from a better source, so short names follow it forward
instead of breaking. Different *products* stay ambiguous and raise, listing
what matched. Name a revision outright to pin it.

Every load resolves to a concrete dataset commit and records the resolved id
in the robot's source, so a saved project — and the script the studio
exports — replays the *same bytes* later, on the revision it resolved to;
that is the [determinism story](../concepts/determinism.md) extended to model
acquisition.
Downloads land in the standard Hugging Face cache. Packages whose meshes
cannot be redistributed are `recipe_only`: `from_catalog` raises and points at
building them locally with botrail-catalog-builder.

Not every package is a robot. A `workpiece` — a body-in-white, a casting, a
fixture — is a pile of meshes a cell loads as obstacles, and
[`catalog_package`][botrail.catalog_package] hands back its directory so the
cell can reach them without a hand-written cache path that quietly stops
matching when the dataset moves:

```python
package = Path(bt.catalog_package("botrail/body/biw-sedan"))
for piece in sorted((package / "collision").glob("*.stl")):
    scene.add_mesh(f"body/{piece.stem}", str(piece), (0, 0, 0))
```

The frames a package manifest declares come along: `frames.tcp_default`
becomes the model's `tcp_link` (the grasp center, not a fingertip), and
`flange_frame` / `mount_frame` surface as `robot.flange_link` /
`robot.mount_link` — which is what lets catalog parts
[mount without naming a single frame](#mounting-a-tool).

## What a model knows

```python
robot.dof            # actuated joints
robot.joint_names    # in q-vector order — every `positions` list uses this order
robot.joint_limits   # (lower, upper) per joint, None for continuous
robot.link_names
robot.tcp_link       # declared TCP if any (catalog, attach_tool), else deepest leaf
```

## Mimic joints

Joints that follow another joint — URDF `<mimic>`, USD
`PhysxMimicJointAPI`, or the `botrail:mimic` customData that URDF-to-USD
converters author — never appear in `joint_names` or in a position vector. A two-finger gripper
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

## Mounting a tool

`attach_tool` welds an end-effector onto a flange and returns the composite —
one kinematic tree whose DOF vector is the arm's joints followed by the
tool's, mimic joints included. Neither input changes; robots are immutable.

```python
arm = bt.Robot.from_catalog("ur5e")
coupling = bt.Robot.from_catalog("gripper-coupling")
gripper = bt.Robot.from_catalog("2f-85")

robot = arm.attach_tool(coupling).attach_tool(gripper)   # frames from the manifests
robot.dof        # 6 + 1
robot.tcp_link   # the gripper's declared TCP — IK now targets the grasp center
```

With catalog parts nothing needs naming: `flange` defaults to the robot's
declared `flange_link`, `mount` to the tool's declared `mount_link` (else its
root), and a coupling's *outward* face becomes the composite's flange, so the
next `attach_tool` in the stack keeps chaining. Models without declared frames
spell them out:

```python
robot = arm.attach_tool(
    gripper,
    flange="flange",                       # arm-side link (ISO 9409-1 face)
    mount="robotiq_arg2f_base_link",       # tool-side link — its root
    offset_position=(0, 0, 0.0139),        # e.g. the coupling's thickness
)
```

The composite's TCP comes from `tcp=` if you pass it, else from a TCP the tool
declares (catalog manifests do), else the deepest-leaf heuristic — which on a
merged model would pick an arbitrary fingertip, exactly the case the explicit
TCP exists for. The weld is a fixed joint, so the flange/mount pair is treated
like any adjacent pair in collision checking. If the two models share a link
or joint name, pass `prefix="g_"` to namespace the tool's names. Saved
projects and exported scripts carry the attachment and rebuild it on load.

`mount` must be the tool's *root* link; welding a tool by a mid-chain link
would need re-rooting its tree, which botrail refuses rather than guesses.

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
When a seed does not converge — a robot whose limits exclude zero starts
clamped against them, the FR3 famously so — the solver retries from
deterministically generated seeds (limits midpoint, then fixed-seed samples
within the limits), so the same call returns the same answer every time;
`restarts=0` disables this. The studio's drag-to-pose solver never restarts:
a per-frame solve must stay on its solution branch.
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
