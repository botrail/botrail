# Export and replay USD

*Walks through [`examples/export/export_animation.py`](https://github.com/botrail/botrail/blob/main/examples/export/export_animation.py)
and [`examples/export/play_record.py`](https://github.com/botrail/botrail/blob/main/examples/export/play_record.py)
— getting animation out of botrail, and back in.*

A bake is only useful if it leaves the building. botrail's animation format is
USD: the exported layer references the robot from its original stage at full
visual fidelity, includes every obstacle, and plays in usdview, Omniverse, or
Blender with no botrail installed. The same pipeline runs in reverse — a
recording plays back into the studio, whether botrail baked it or Isaac Sim
did.

## Two exporters, one format

You have met both already:

```python
scene.export_usd("motion.usda", traj, fps=60)   # one planned trajectory
tl.export_usd("cycle.usda", fps=60)             # a whole baked cycle
```

[`Scene.export_usd`][botrail.Scene.export_usd] bakes a single
[`Trajectory`][botrail.Trajectory]; the timeline version bakes everything the
cycle did — every robot, every obstacle, grasped objects riding, releasing,
resting exactly as simulated. Called without a trajectory,
`scene.export_usd("cell.usda")` writes the *static* cell instead — robots at
their current pose, every visible obstacle — the layer a layout is handed
around as.

## A carry motion, exported

`export_animation.py` is the single-trajectory case with the one wrinkle worth
a tutorial: **the box has to ride the gripper**, so the grasp happens *before*
the plan.

```python
--8<-- "examples/export/export_animation.py:28:46"
```

The order matters. Close the fingers into the box, `attach` it, lift so the
held box clears the belt — and only then plan. While attached, the box is part
of the robot: it follows the hand in the plan, collides as the robot, and rides
along in the export.

```bash
python examples/export/export_animation.py
```

```text
exported a 6.04s carry motion to cell_anim.usda
view it with: usdview cell_anim.usda
```

Open it in usdview and press play: the Franka at full Isaac fidelity, the box
leaving the belt and landing over the pallet, the whole cell around it.

## Playing a recording back

The studio plays USD recordings through
[`Scene.play_usd_animation`][botrail.Scene.play_usd_animation]:

```python
scene = build_scene()                     # the cell the recording was baked from
scene.play_usd_animation("cell_seq.usda")
bt.studio(scene)
```

`play_record.py` wraps this with one important idea: **a recording is joint
tracks addressed to robot instances, not to "the robot"**. botrail exports each
robot under `/World/<instance name>`, and playback looks the scene's robots up
by that path. Play the two-arm recording onto the single-arm cell and you get
an error, not a degraded picture:

```text
recording import failed: cannot locate robot `near` in the recording
(no `/World/near`); pass robot_roots with its prim path
```

That error message is also the escape hatch: `robot_roots` maps instance names
to prim paths, which is how recordings from *outside* botrail — an Isaac Sim
capture, say — play through the same pipeline:

```python
scene.play_usd_animation("isaac_capture.usda", robot_roots={"panda": "/World/Franka"})
```

The demo script picks the right cell automatically by sniffing which instance
prims the recording animates:

```python
--8<-- "examples/export/play_record.py:41:54"
```

A recording baked before a layout change still plays — the new scenery just
stays static, and the import says so once per prim rather than flooding the
console. When the warnings pile up, re-bake.

How the robot plays depends on where it came from. A USD-sourced robot shares
a stage with its recording, so playback recovers q(t) from the recorded joint
states (`joint_state` mode). A robot built from URDF or by
[`attach_tool`][botrail.Robot.attach_tool] — the welding cell's arm-plus-gun,
say — has no single stage behind it, so its export bakes per-link world poses
instead, and playback follows those directly (`transforms` mode). Both kinds
of recording also stand alone: open one in `usdview` and it plays with no
botrail installed. What playing it into the live cell adds is the studio
around it — the timeline dock, scrubbing, and the scene's own obstacles
following their recorded tracks.

```bash
python examples/basics/sequence_demo.py          # bake cell_seq.usda
python examples/export/play_record.py            # …and watch it in the studio
```

## The complete scripts

??? example "examples/export/export_animation.py"

    ```python
    --8<-- "examples/export/export_animation.py"
    ```

??? example "examples/export/play_record.py"

    ```python
    --8<-- "examples/export/play_record.py"
    ```

## Where this leaves you

Everything in this series composes: a cell taught by IK
([Pose and plan](pose-and-plan.md)), sequenced like a PLC
([Pick from a moving belt](sequence-cell.md)), verified in CI
([Verify the cell](verify-in-ci.md)), studied as a function
([Parameter sweeps](parameter-sweep.md)), scaled to two arms
([Two arms, one belt](two-robots.md)) — and shipped as a USD file anyone can
open.
