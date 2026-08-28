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
the machine rather than like a palette. A sole robot exports under the
historical `Robot` prim;
with several, each lands at `/World/<sanitized instance name>` — the
convention playback relies on. Exporters return their warnings as a list.

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
exactly what the studio's picture-in-picture shows.

The reverse direction — playing recordings back into the studio, including
Isaac Sim captures — is
[`play_usd_animation`][botrail.Scene.play_usd_animation]; the
[Export and replay USD](../tutorials/replay-usd.md) tutorial covers both ways.

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

## Python

```python
print(scene.generate_python())
```

A script that rebuilds the scene through the botrail API — the same content as
the studio's **Export .py** button. This is the studio-to-code exit: build
interactively, export, and the cell becomes reviewable text. For a bundled,
loadable artifact instead, see [Projects](projects.md).
