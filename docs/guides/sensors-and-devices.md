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

## Housekeeping

```python
scene.sensor_names;  scene.remove_sensor("eye")
scene.device_names;  scene.remove_device("belt")
```

Sensors and devices are saved in [projects](projects.md), appear in the
studio's scene tree, and their lanes are queryable on every bake —
[Timeline assertions](timeline-assertions.md) shows how to test against them.
