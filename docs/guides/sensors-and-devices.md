# Sensors and devices

This is what makes a botrail environment *behave* rather than sit there:
sensors that read the world into signals, and devices that move parts of it
— belts, axes, magazines and vehicles.
Both live on the same scan clock as the [sequencer](sequences.md), and both
show up as waveform lanes on the baked timeline.

## Zone sensors

A box-shaped presence sensor. Its name becomes a read-only input signal, ON
while a watched body overlaps the zone:

```python
scene.add_zone_sensor("station_busy",
                      position=(0.0, 0.6, 0.8), size=(0.5, 0.5, 0.6),
                      watch_robots=["near"])
```

What it watches is explicit:

* `watch=[...]` — a list of obstacle names; default is **every obstacle**.
* `watch_robot=True` / `watch_robots=[...]` — sense robot links too.
* `watch=[]` with `watch_robot=True` — a robot-only light curtain.

A zone says "somebody is inside", not *who* — so an interlock between two arms
needs one zone **per arm** over the same volume; a single zone watching both
would be tripped by the very arm waiting on it.

## Beam sensors

A photoelectric beam between two world points, ON while interrupted:

```python
scene.add_beam_sensor("eye", frm=(0.0, 0.4, 0.3), to=(0.0, 0.8, 0.3),
                      radius=0.005, watch=["crate"])
```

Two things bite in practice. The beam trips when a part's **leading face**
reaches it, so a station sensor belongs half a part-width downstream of the
taught pose. And a beam over a running belt is a **momentary** signal — the
part crosses and the lane drops again. A transition that isn't currently
waiting on it will miss it; latch it into an internal signal, or gate the step
so it is already waiting when the part arrives.

## Conveyors

```python
scene.add_conveyor("belt",
                   zone_position=(-0.2, 0.6, 0.3), zone_size=(1.2, 0.3, 0.3),
                   velocity=(0.25, 0.0, 0.0), running=False)
```

A conveyor is a transport **zone**, not a mesh: while running, any unattached
obstacle whose origin lies inside the zone is carried at `velocity`. Put the
zone above the belt's slab so it carries the goods and not the structure.
Drive it from steps with `bt.seq.start/stop/set_speed`; its running state is a
signal lane (`tl.signal("belt")`), which is how "the belt ran exactly through
feed" becomes an assertion.

Collision-disabled obstacles still ride — that is the trick behind moving
scenery like belt cleats.

For an indexing line, command the pitch as a distance instead of driving
start/stop by timer: `bt.seq.advance("belt", 5.2)` runs a *stopped* belt for
exactly 5.2 m and stops, and `bt.seq.device_done("belt")` is the await. The
final scan tick moves exactly the remainder, so the pitch is exact no matter
how the scan period divides it — see
[Indexed transfer](sequences.md#indexed-transfer).

## Sources and sinks: endless supply, finite pool

A baked timeline holds a fixed set of named object tracks, so "endless supply"
is authored as a **finite pool plus a return loop**, which is also what a real
accumulation line is:

```python
scene.add_source("cartons", pool=[f"box_{i}" for i in range(6)],
                 park=(-1.75, 0.62, -0.45),          # the magazine
                 pitch=(0.0, 0.0, -0.07),            # member i parks at park + pitch*i
                 position=(-2.25, 0.62, 0.66),       # where fed members appear
                 interval=0.0, running=False)
scene.add_sink("line_end", zone_position=(1.3, 0.62, 0.66),
               zone_size=(0.12, 0.4, 0.05), source="cartons")
```

`interval=0.0` makes an **indexing feeder** — one member per `bt.seq.start` —
which is how you guarantee pool order is arrival order. A periodic feeder
(`interval=2.0`) feeds on the clock instead. Members reaching the sink go back
to the source's magazine, free to be fed again. A member that does not start
on its park slot starts out on the line — an already-loaded belt.

## Linear axes

A door, a lifter, an indexing table — one axis, position-commanded:

```python
scene.add_linear_axis("door", objects=["door_panel"],
                      axis=(0.0, 0.0, 1.0), speed=0.4,
                      range=(0.0, 0.6), position=0.0)
```

```python
sq.step("open_door", actions=[bt.seq.move_to("door", 0.6)],
        transition=bt.seq.device_done("door"))
```

The axis moves its listed obstacles along `axis` at `speed`, clamped to
`range`; `device_done` is the in-position condition.

## Vehicles

The fifth device is a guided transport vehicle: it drives an authored path
station to station, carries its body and whatever is on its deck, and is
commanded with `goto` / awaited with `device_done` — the same pair a linear
axis uses.

```python
scene.add_vehicle("agv", body=["/World/AGV"],
                  path=[(-2.6, -2.9), (0.0, -2.9)],
                  stations={"warehouse": 0, "dock": 1},
                  speed=0.8, start="warehouse")
```

A robot can ride one, which makes it an AMR. Vehicles have enough of their
own rules — the aisle check, trays, mounted sensors, what happens to planned
motions while driving — to get their own page:
[Vehicles and AMRs](vehicles-and-amr.md).

## Lifts

The sixth device is an elevator: a car of ordinary obstacles moved along an
axis between named stops, carrying **whatever its capture zone holds when
the ride is commanded** — loose parts by origin, and vehicles *whole*, the
chassis, the deck load and any mounted robot riding one rigid motion.

```python
scene.add_lift("lift", car=["lift"],                # obstacles, prefix ok
               zone_position=(3.25, 0.0, 1.0), zone_size=(1.3, 1.3, 2.0),
               stops={"1F": 0.0, "2F": 2.2}, speed=0.6)
```

Command it with `bt.seq.move_to("lift", "2F")` and await `device_done`.
The cargo is fixed the moment the command fires — an elevator moves after
the doors close — so a vehicle still driving refuses the ride, a vehicle
half out of the zone refuses to board by name, and nothing joins mid-ride.

The vertical hop in a vehicle's path (two waypoints stacked at the car) is
a *lift edge*: validation accepts it only where a lift's zone covers both
ends at its stops, and `goto` never walks across it — drive to the near
side, ride, continue. A stop between floors leaves the vehicle off its
path, and the next `goto` says so.

Doors are not part of the device, and need no special vocabulary: a panel
on a `add_linear_axis` physically blocks the path while closed — boarding
through it simply fails the aisle check — and the open/close steps are
ordinary sequence lanes on the timing chart. `examples/vehicles/lift_demo.py` runs
the whole interlock chain: call → door open → board → door close → ride →
alight.

## Cameras

A named viewpoint with pinhole optics, drawn in the studio as a body plus a
wireframe frustum whose aspect follows `resolution` and whose angle follows
`fov` (horizontal, degrees). A camera is presentation only: it publishes no
signal and never affects planning or the cycle — it answers "what does this
camera see from here", the layout question you get asked before commissioning.

```python
# A fixture, aimed at a world point (-Z looks, +Y is image-up):
scene.add_camera("overview", position=(1.6, -1.4, 1.3), look_at=(0, 0, 0.3),
                 fov=60, resolution=(1280, 720))
# A wrist camera, offset in the link frame:
scene.add_camera("eye_in_hand", robot="ur", link="tool0",
                 position=(0, 0.05, 0.03), fov=50, resolution=(1920, 1080))
# Riding a vehicle deck:
scene.add_camera("agv_front", mount="agv", position=(0.3, 0, 0.4))
```

Deselected, the frustum draws as a compact aim gizmo; selecting the camera
(scene tree or click) extends it to the far clip for coverage checks, and a
world-mounted camera gets the move/rotate gizmo. Mounted cameras ride their
machine during playback like mounted sensors do.

A real camera comes straight from the catalog:

```python
scene.add_camera("inspect", from_catalog="realsense/d400/d435",
                 robot="ur", link="tool0", position=(0, 0.06, 0.02))
```

The package's flat specs become the optics (fov, resolution, and the
near/far band from its rated range — explicit arguments still win), the
given pose places its *mount face* while the optical axis follows the
package's own calibration (`frames.camera_frames`), and the identity lands
on the BOM as a `sensor.camera` line, pinned to the catalog revision.
From there the selection loop closes like for any other equipment:
`scene.requirements()` derives what the cell asks of the camera — the
authored framing always, a working-distance band when a vision sensor
judges through it — and `scene.check()` answers ok / `spec_short` /
`spec_unknown` against the part's stated specs. (The camera is the
purchasable article; vision sensors add requirements to its line, never a
line of their own.)

Selecting a camera also opens its **picture-in-picture** at the viewport's
top-right: the live view through that camera — scenery, robots and process
light, with the authoring aids (grid, gizmos, sensor volumes, overlays)
hidden. The header switches between cameras and toggles the size; closing
the panel stops the second render pass entirely. It coexists with the
SFC/ladder/I/O overlays, and the picture follows playback, so a wrist
camera shows the approach as the arm moves.

### Vision sensors

A camera becomes an *input* by putting a vision sensor behind it: the
sensor's name becomes a read-only signal, ON while a watched body overlaps
the camera's view frustum.

```python
scene.add_vision_sensor("part_seen", camera="eye_in_hand", watch=["workpiece"],
                        detect_range=(0.3, 2.0))   # default: the camera's near/far
```

The camera is the optics — pose, mount, field of view all come from it, so
a wrist camera's sensor sweeps with the arm — and the sensor is the
judgement. It is a *geometric* judgement: frustum overlap plus (by
default) a single occlusion ray from the camera to the body's origin, so a
part hidden behind a wall does not trip it. No pixels are rendered or
interpreted — it answers "was it in view", not "would the vision system
have detected it". Robot links, when watched, trip on overlap alone and
never occlude. Like every sensor, the lane shows in the timing chart, the
SFC/ladder views, and derives an input contact in the [I/O map](io-map.md);
the BOM line stays on the camera (`sensor.camera`) — the sensor is logic,
not hardware.

With a bake (or a motion preview) on the timeline dock, **⤓ cam** records
the PiP camera's view as a WebM video, right in the browser: the baked
tracks are re-walked on a fixed 30 fps grid — not captured in real time —
so the export never drops a frame and the same bake always produces the
same file. Needs WebCodecs (Chrome, Edge, or a recent Firefox); the button
says so when it can't run. The same export runs headless from Python and
CI — [`botrail.capture.record_camera`](export.md#camera-video), or
`botrail capture` on the command line.

## Housekeeping

```python
scene.sensor_names;  scene.remove_sensor("eye")
scene.device_names;  scene.remove_device("belt")
scene.camera_names;  scene.remove_camera("overview")
```

Sensors and devices are saved in [projects](projects.md), appear in the
studio's scene tree, and their lanes are queryable on every bake —
[Timeline assertions](timeline-assertions.md) shows how to test against them.
