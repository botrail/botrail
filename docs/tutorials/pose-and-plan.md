# Pose and plan

*Walks through [`examples/demo.py`](https://github.com/botrail/botrail/blob/main/examples/demo.py)
— a Franka Panda in a small USD factory cell, live in the studio.*

![The demo cell in the studio](../assets/botrail_demo.png)

```bash
python examples/demo.py
```

The first run downloads NVIDIA's official Isaac Sim Franka asset (~10 MB) into
the botrail cache; after that it starts instantly. Then the studio opens with
the arm standing on its pedestal, and everything below is live.

## A USD robot

The robot is not a URDF conversion — it is the Isaac stage itself, loaded as an
articulation:

```python
robot = bt.Robot.from_usd(fetch_franka())   # UsdPhysics joints + rigid bodies
```

`fetch_franka()` is nothing magical: it downloads `franka.usd` and the sublayers
it references into `~/.cache/botrail/` (or `BOTRAIL_CACHE_DIR`), once. Link and
joint names are the prim paths (`/panda/panda_hand`, …), revolute limits are
converted from degrees, and Y-up stages are re-modeled as Z-up.

Two notices print during the import, and they are expected:

```text
botrail: usd robot import: /panda/panda_hand/panda_finger_joint2: mimic joint
    authored on `rotX` but the joint moves about `transX`; ignored
botrail: usd robot import: /panda/rootJoint: unsupported joint type `PhysicsJoint`; skipped
```

botrail says what it skipped rather than silently dropping it. The asset
authors its finger mimic about the wrong axis, so the two fingers stay
independent joints here — which is why the demo's ready pose has nine values,
seven arm joints plus both fingers:

```python
scene.set_joint_positions([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.035, 0.035])
```

## The cell, and mounting the robot on it

The environment is one hand-authored USD layer, and it comes in with one call:

```python
scene = bt.Scene(robot)
scene.load_usd(Path(__file__).parent / "assets" / "factory.usda")
scene.set_robot_base_pose(*scene.frame("/World/MountFrame"))
```

`load_usd` turns the stage's geometry into obstacles and its leaf Xform prims
into **named frames**. `/World/MountFrame` is the top of the pedestal, authored
into the cell exactly so that the robot can be placed with one line — move the
pedestal in the USD, and the robot moves with it.

The same cell also authors `/World/Conveyor/PickFrame` and
`/World/Pallet/PlaceFrame`: the two *taught stations*. Keeping teach points in
the layout file, not in the program, is what makes the next part work.

## Teaching grasps by IK

The teach frames are *grasp* poses — the point between the fingertips, tool
axis along the approach. IK, though, solves for a link. So a taught pose is
backed off along the tool axis to the hand frame first:

```python
--8<-- "examples/demo.py:68:95"
```

`teach_grasp` is the scripted form of dragging the studio's TCP gizmo: solve
toward the pose, fail loudly if IK missed (rather than teaching a bad pose),
and return the joint vector. Passing `standoff=0.15` gives the hover pose
150 mm above the same grasp. The later tutorials call this constantly — every
pick, place, and pallet course is taught this way, which is why moving a frame
in the USD re-teaches the cell for free.

## Things to try in the studio

The scene in your Python process and the scene in the browser are the same
object, so poke it from both sides:

* **Drag the TCP gizmo** — IK follows live, with the clearance readout and
  collision highlighting updating as you go.
* **Plan from the panel** — set a goal, plan, and scrub the trajectory. The
  same thing from Python:

    ```python
    traj = scene.plan_to_pose((0.4, 0.1, 0.5))
    ```

* **Pose from the REPL** — with `bt.studio(scene, block=False)` you keep the
  prompt; `scene.set_tcp_target(...)` moves the browser's robot.
* **Save what you made** — `scene.save_project("cell.botrail")`, or
  `scene.generate_python()` for a script that rebuilds the scene (the studio's
  "Export Python" button does the same).

## The complete script

??? example "examples/demo.py"

    ```python
    --8<-- "examples/demo.py"
    ```

## Next

The cell is still scenery — nothing moves on its own. [Pick from a moving
belt](sequence-cell.md) gives it a conveyor, a photo-eye, and a sequence, and
picks the box without stopping the line.
