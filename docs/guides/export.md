# Export

Deliverables leave botrail in open formats: USD for anyone with a viewer,
CSV/JSON for your own pipeline, vendor robot programs for the controller, and
Python for the next author. Nothing round-trips through a proprietary project
file. The engineering documents a cell hands over come out of the same
script — the [I/O list](io-map.md), the [bill of materials](parts-and-bom.md),
the [layout sheet and the cell report](layout-and-report.md) are *derived*
from the scene, so they cannot disagree with it or with each other. The
[Hand over the cell](../tutorials/hand-over.md) tutorial writes the whole set.

## USD animation

```python
scene.export_usd("cell.usda")                   # the cell as it stands, no motion
scene.export_usd("motion.usda", traj, fps=60)   # one planned trajectory
tl.export_usd("cycle.usda", fps=60)             # a whole baked cycle
tl.export_usd("cycle.usdc", fps=60)             # same layer, binary crate file
tl.export_usd("takt.usdc", fps=24, start=62.3, end=86.9)   # one takt of a line
```

Without a trajectory the export is the **static cell**: robots at their
current joint positions, every visible obstacle at its pose, toolpaths and
cameras — the layer a layout is handed around as, and what the browser demo
is served. Equipment ordered [from the catalog](parts-and-bom.md) comes out
as ordinary prims, so a cell whose belt and guarding are generated at run
time still ships as one self-contained USD file.

`start`/`end` clip the export to a window, and on a line that is the
difference between shippable and not: a full run is mostly repetition, so
one steady-state takt carries the whole story. The two-station weld line
measures 45.9 MB as a whole run at 30 fps and **16.4 MB as one takt at 24
fps** — see `scripts/export_line_recording.py`, which picks the window
where the pipeline is fullest. (A binary recording keeps its prim names
out of reach of text sniffing, so `examples/export/play_record.py` takes an
explicit `--cell` for those.)

Both bake to a USD layer that plays in usdview, Omniverse, or Blender with no
botrail installed: link motion as timeSamples, every obstacle as prims,
grasped objects riding, releasing, resting exactly as simulated. USD-sourced
robots **reference their original stage** at full visual fidelity (assets are
copied to a sibling `<stem>_assets/` directory); URDF robots are authored from
the model's visuals, and a visual whose OBJ names an `mtllib` keeps its
authored colors — one `displayColor` per face, so a catalog arm looks like
the machine rather than like a palette. Every triangle mesh — a link's
visual, a scanned object, a collision shape — is written **once**, as a
binary layer under `<stem>_assets/meshes/<name>.usdc` named after the first
link or obstacle that drew it, and every prim that draws it references that
layer with its own pose and colour: six arms in a grid cost the file one
arm's meshes, and a cell that was 29 MB of text is a few MB. The studio's
own download is the one exception — a single file, its meshes inline. A sole robot exports under the
historical `Robot` prim;
with several, each lands at `/World/<sanitized instance name>` — the
convention playback relies on. Exporters return their warnings as a list.

Finishes go out as `UsdPreviewSurface` metallic/roughness inputs, apart
from the physics materials. Anything that came in from USD — a static
import, a USD tool on the arm — goes out with its own normals, UVs and
material subsets, and its material networks and images are copied under
`<stem>_assets/`: ship that directory with the layer. [USD import](usd-import.md)
lists what the appearance path carries.

The extension picks the serialization: `.usda` writes text (diffable, but
large — timeSamples dominate), `.usdc` or `.usd` writes the binary crate
format at roughly half the size, byte-for-byte the same composed result.
`.usdz` is rejected on purpose: it is an asset *package*, not a layer, and
packaging the referenced robot assets is a different operation.

[Cameras](sensors-and-devices.md#cameras) in the scene export as
`UsdGeomCamera` prims under `/World/Cameras`, their world pose sampled
frame by frame through their mount (a wrist camera rides the arm, a
vehicle camera its machine) and their optics carried as
focal-length/aperture — so *View → Camera* in usdview or Omniverse frames
exactly what the studio's picture-in-picture shows. The pixel size, which
no `UsdGeomCamera` attribute holds, rides along as a custom
`botrail:resolution`, so the same stage [loads back](usd-import.md#cameras-come-along-too)
with the cameras it left with.

The reverse direction — playing recordings back into the studio, including
Isaac Sim captures — is
[`play_usd_animation`][botrail.Scene.play_usd_animation]; the
[Export and replay USD](../tutorials/replay-usd.md) tutorial covers both ways.

## USD for a physics engine (Isaac Sim, Isaac Lab)

An animation layer is a recording: robots are pictures of their links, and
nothing collides. `physics=` writes the static cell as a **simulation
stage** instead — the world an engine can own, in UsdPhysics:

```python
physics = bt.Physics(world=True)               # the whole cell; True = what was declared
print(scene.physics_plan(physics).to_markdown())   # what becomes what
scene.export_usd("cell.usdc", physics=physics)
```

| In the cell | In the stage |
|---|---|
| a URDF, catalog or tool-composed robot | `PhysicsArticulationRootAPI`: every link a rigid body with its mass and colliders, every joint with limits, a drive (`maxForce` from the effort limit), its velocity cap, mimic couplings (`PhysxMimicJointAPI`), and the pose it stands in |
| a USD-sourced robot (an Isaac asset) | referenced, as ever — its own physics comes with it |
| a dynamic unit of [`physics_plan()`](physics.md#the-world-scope) | one rigid body (`/World/Env/<unit>`), its members beneath it as colliders |
| what a device moves (a door on an axis, a lift's car, a vehicle nobody rides) | one kinematic body per device (`/World/Env/amr1` for `amr1/base_link`, `amr1/visual/...`): the engine never pushes it, your controller poses it |
| a robot riding a vehicle | one fixed-base articulation with its vehicle: six virtual joints (`base_x`, `base_y`, `base_z`, `base_yaw`, `base_pitch`, `base_roll`) state the vehicle's pose, the vehicle's body is a link, the robot is bolted to it. Drive the joints and both move. A USD-sourced robot rides the same way, its own stage's anchoring to the world deactivated |
| a second robot on the same vehicle | joins the first one's articulation, bolted to the vehicle's body: two arms on one cart are one machine, addressed through the first rider's prim, its links and joints named apart (`left_shoulder_pan`, `right_shoulder_pan`) |
| a walker or an aircraft (a floating base) | no joint to anything, rooted at its base link; the envelope its vehicle draws around it stays a picture |
| a conveyor | the bodies its zone reaches become kinematic, with `PhysxSurfaceVelocityAPI` |
| everything else that collides | a static collider |
| the ground of the world scope | a `Plane` collider |

Collision follows `enabled` and the picture follows `visible`, the way they
do in botrail: an invisible collision proxy is authored as a `guide`, a
display shell that never collides stays a picture. A part drawn as a mesh
but colliding as a box (the parts library's tray) gets the box beside the
picture, so the engine sees the shape botrail's own bake sees.

The stage has no timeSamples — an animation and a simulation would fight
over the same prims — so `physics=` does not combine with a trajectory. A
body named under another body (a crate on a pallet, `pallet/crate`) moves
out to be its sibling (`/World/Env/pallet_crate`): UsdPhysics takes every
prim below a rigid body as part of that body. Prefer `.usdc` for robots
with meshes; the file is stamped crate 0.8, which Isaac Sim 5 (USD 24)
reads.

In Isaac Lab the cell is one reference per environment, and its residents
are addressed where the stage put them:

```python
cell = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Cell", spawn=sim_utils.UsdFileCfg(usd_path="cell.usdc"))
robot = ArticulationCfg(prim_path="{ENV_REGEX_NS}/Cell/Robot", spawn=None, actuators={...})
carton = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/cartons/c3", spawn=None)
```

`examples/export/isaaclab_cell.py` runs this end to end (checked on Isaac
Lab 2.3 / Isaac Sim 5.1): the arm starts in its exported pose and follows
joint targets, loose parts fall and rest on the bench and on each other,
the belt carries its carton, and cloned environments agree. Three things
are the engine's, not the stage's:

* **A belt needs CPU dynamics.** A surface velocity is a contact
  modification; under GPU dynamics PhysX lets a part fall through a running
  belt, and the physics replicator does not clone it. Run with
  `device="cpu"` and `replicate_physics=False`, or switch
  `physxSurfaceVelocity:surfaceVelocityEnabled` off (the export warns when
  the cell has a belt).
* **Isaac Lab checks `init_state.joint_pos` against the limits.** It
  defaults to zero; a robot with a joint whose range excludes zero (the
  Franka's fourth) needs its defaults stated — `scene.joint_positions` is
  the pose the stage was written in.
* **Drive gains are a starting point.** The authored position drives hold
  the exported pose when the stage is simply played (`powered=False`
  leaves only the passive drag); Isaac Lab's actuator configs overwrite
  them.
* **A vehicle that carries a robot is driven through its base joints.**
  botrail's vehicles go where their program says, and the stage says the
  same in the form an engine takes on every pipeline — the way Isaac Lab's
  own mobile manipulators are built, with all six degrees of freedom, so a
  lift and a ramp are joint targets like a floor is. The joint positions
  are the vehicle's pose in the cell's frame (metres, and radians once
  Isaac Lab reads them): `base_x = 10.0, base_y = 3.44, base_yaw = 1.57` is
  a vehicle parked at that station, heading +y.

  ```python
  amr = scene["amr1_lift"]                       # the robot *and* its vehicle
  x = amr.joint_names.index("base_x")
  target = amr.data.joint_pos.clone()
  target[:, x] += 0.5                            # half a metre along the cell's x
  amr.set_joint_position_target(target)
  ```

  Checked on Isaac Lab's defaults (GPU dynamics, replicated environments):
  commanded 1 m along, 0.5 m up and 0.2 rad nose-up, the base joints
  tracked within 0.005 and a six-axis arm on the deck kept its pose within
  0.0004 rad — a URDF arm, the Isaac Franka, and an arm from a
  centimetre / Y-up stage alike. The articulation is a fixed base like any
  bolted-down arm's, so Jacobians and mass matrices keep their usual
  shape. The pose is *commanded*: a machine whose pose the ground decides
  — tracks over rough terrain — is not something a botrail vehicle is, in
  the stage or in botrail's own physics.

  Two robots on one vehicle are one articulation: `ArticulationCfg` names
  the first rider's prim, and the joint regexes tell the arms apart by
  their prefixes (`left_.*`, `right_.*`). A USD-sourced robot keeps its
  asset's own names, so two of the same asset on one cart come out with
  PhysX's suffixes (`panda_joint1`, `panda_joint1_0`). Arms sharing an
  articulation do not collide with each other (its self-collision is off
  as a whole), where botrail's own physics would have them collide.

Not written yet: an object held at export time is a free body (it is not
welded to the hand), a USD-sourced robot whose stage roots the
articulation on its base body itself stays anchored to the world (the
export warns), a device nobody rides is a kinematic body rather than a
driven joint, and sensors, signals and programs stay in botrail.

## Camera video

```python
scene.add_camera("overview", position=(2.0, -1.6, 1.4), look_at=(0, 0, 0.4))
scene.simulate_sequence("cycle")               # the bake to film

from botrail import capture
capture.record_camera(scene, "overview", "cycle.mp4")          # or .webm / .gif
```

```console
$ botrail capture cell.py --camera overview --out cycle.mp4 --fps 30
```

Films the baked cycle through a [camera](sensors-and-devices.md#cameras) and
writes a video — the shareable artifact for people who will not open a USD
stage. Under the hood it is the studio's own **⤓ cam** exporter driven by a
headless browser: the cycle is re-walked on a fixed fps grid rather than
captured in real time, so no frame is ever dropped and the same bake always
produces the same file, pixel for pixel — on CI's software renderer too.
`.webm` (VP9) comes straight from the browser; `.mp4` and `.gif` are
converted with ffmpeg (system, or `pip install imageio-ffmpeg`). Needs
playwright with a fetched Chromium (`pip install playwright && python -m
playwright install chromium`); both are checked when called, not at import.

## CSV and JSON

```python
traj.export_csv("motion.csv", dt=0.008)   # resampled uniformly; dt=None writes
                                          # the internal waypoints
traj.export_json("motion.json")           # {joint_names, times, positions, velocities}
```

```text
t,shoulder_pan,shoulder_lift,elbow,wrist_1,wrist_2,wrist_3
0.000000,0.000000,0.000000,0.000000,0.000000,0.000000,0.000000
0.008000,0.002194,-0.001280,0.001646,0.000000,0.000549,0.000000
```

A whole cycle exports the same way: `tl.robot_trajectory("far")` is a
[`Trajectory`][botrail.Trajectory], with step boundaries in `segment_ends`.

## Robot programs

```python
traj.export_script("pick.script", dialect="urscript")
print(traj.to_script())                     # same thing, as a string
```

```text
def pick():
  # Generated by botrail (units: rad, m, s)
  # joints: shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3
  movej([0, 0, 0, 0, 0, 0], a=4, v=2, r=0)
  movej([1.2, -0.7, 0.9, 0, 0.3, 0], a=4, v=2, r=0)
end
```

The script replays the **sparse planned waypoints** (`traj.segments`) as
vendor move commands — one `movej`/`movel` per waypoint, Cartesian-line
segments as linear moves — and leaves time parameterization to the robot
controller, which is where it belongs on a real machine. Speeds derive from
the joint limits scaled by `speed_scale`; linear-move speed is `tcp_speed`.

Two honest limits: the current dialect list is `"urscript"` (a 6-axis
format — exporting a 4-DOF arm is a clean error), and `blend_radius` defaults
to 0 for a reason — overlapping blends abort some controllers, so raise it
only after verifying on yours. botrail's own CI harness can replay exported
scripts against a URSim controller simulator.

The control logic itself goes to the PLC IDE as PLCopen XML
(`scene.export_plcopen("cell.plcopen.xml")` — see
[Offline commissioning](offline-commissioning.md)), and the controller's
log comes back through `tl.diff(trace)`.

A baked *sequence* exports too (`tl.to_script()`), with the sensor
contacts and coils it uses on numbered digital I/O. The ports come from
the [I/O map](io-map.md): bind the points on a `robot_controller` node
(`bt.io.ur_standard()` gives a UR its DI0-7 / DO0-7) and the script picks
them up; `inputs=` / `outputs=` dicts still override per key, `io=` projects
a newer assignment onto an older bake.

A dual-arm robot exports one controller program per arm:
`tl.export_script("left.script", sequence="left", group="left")` carries the
left arm's six joints and its own moves; where the program waits on the other
arm (`robot_done(..., group="right")`), the script reads that controller's
idle contact on an input keyed `<robot>/<arm>`. Leaving `group=` off a
dual-arm robot is an error, not a 12-axis program.

## Python

```python
print(scene.generate_python())
```

A script that rebuilds the scene through the botrail API — the same content as
the studio's **Export .py** button. This is the studio-to-code exit: build
interactively, export, and the cell becomes reviewable text. For a bundled,
loadable artifact instead, see [Projects](projects.md).
