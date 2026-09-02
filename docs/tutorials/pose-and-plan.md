# Pose and plan

*Walks through [`examples/basics/demo.py`](https://github.com/botrail/botrail/blob/main/examples/basics/demo.py)
— a Franka Panda in a small USD factory cell, live in the studio.*

![The demo cell in the studio](../assets/botrail_demo.png)

```bash
python examples/basics/demo.py
```

The first run downloads NVIDIA's official Isaac Sim Franka asset (~10 MB) into
the botrail cache, and the cell's equipment from the model catalog (a few
hundred kB — `pip install botrail[catalog]` if you have not already); after
that it starts instantly. Then the studio opens with the arm standing on its
pedestal, and everything below is live.

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
scene.load_usd(Path(__file__).parents[1] / "assets" / "factory.usda")
scene.set_robot_base_pose(*scene.frame("/World/MountFrame"))
```

`load_usd` turns the stage's geometry into obstacles and its leaf Xform prims
into **named frames**. `/World/MountFrame` is the top of the pedestal, authored
into the cell exactly so that the robot can be placed with one line — move the
pedestal in the USD, and the robot moves with it.

The same cell also authors `/World/Conveyor/PickFrame` and
`/World/Pallet/PlaceFrame`: the two *taught stations*. Keeping teach points in
the layout file, not in the program, is what makes the next part work.

## The equipment, ordered

What that layer does *not* model is the belt, the rack, the guarding, the
control cabinet or the light curtain over the vehicle gate. Those are
standard products bought to size, so they come from the
[model catalog](../guides/robots.md#the-model-catalog) instead — five
products, each ordered by a [generator](../guides/standard-parts.md) handed a
catalog id (the guard takes two calls, because two things cross the perimeter
and each opening breaks the run; the curtain then watches the gate opening
the fence cannot close):

```python
--8<-- "examples/basics/demo.py:94:152"
```

Each one is checked against what the package actually sells: ask the fence for
1.8 m and it answers with the heights it has and stops. The panel widths are
never written down — each edge of a `GUARD` run is filled with the fewest
panels that reach the next corner — and the belt arrives as a **device** as
well as a body, so the [next tutorial](sequence-cell.md) can start it without
declaring anything. What comes out the other end is a bill of materials with
part numbers on it:

```text
| conveyor.belt       | botrail | BCU-400-3800     | 1  | 44.5 kg |
| structure.rack      | botrail | MR-900x450x1800  | 1  | 18.4 kg |
| structure.fence     | botrail | MG-2000x1500     | 5  | 16.0 kg |
| structure.fence.post| botrail | MGP-2000         | 18 |  7.0 kg |
| structure.door      | botrail | MGD-2000x800     | 1  | 15.6 kg |
```

[Parts and the BOM](../guides/parts-and-bom.md) picks that up.

## Teaching grasps by IK

The teach frames are *grasp* poses — the point between the fingertips, tool
axis along the approach. IK, though, solves for a link. So a taught pose is
backed off along the tool axis to the hand frame first:

```python
--8<-- "examples/basics/demo.py:176:203"
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
* **Teach and plan a motion** — in the Motion tab, pose the robot, **+
  Joint**, **Plan motion**, and scrub the preview in the timeline dock. The
  one-shot form from Python (the preview lands in the same dock):

    ```python
    traj = scene.plan_to_pose((0.4, 0.1, 0.5))
    ```

* **Pose from the REPL** — with `bt.studio(scene, block=False)` you keep the
  prompt; `scene.set_tcp_target(...)` moves the browser's robot.
* **Save what you made** — `scene.save_project("cell.botrail")`, or
  `scene.generate_python()` for a script that rebuilds the scene (the
  header's **Export .py** button does the same).

## The complete script

??? example "examples/basics/demo.py"

    ```python
    --8<-- "examples/basics/demo.py"
    ```

## Next

The cell is still scenery — nothing moves on its own. [Pick from a moving
belt](sequence-cell.md) gives it a conveyor, a photo-eye, and a sequence, and
picks the box without stopping the line.
