# Cable playback after a baked cycle

`bt.rope.animate` simulates a cable against a completed botrail cycle and
returns a new timeline. Robots follow their baked motion; rope reactions are
confined to an independent Rapier 0.36 f64 world. Units are metres, kg and
seconds, with botrail's right-handed Z-up coordinates.

```python
import botrail as bt

# scene contains a robot, a named "grip" signal and a baked cycle.
cycle = scene.simulate_sequence("carry")
tool, _ = scene.link_pose(scene.robot.tcp_link)
cycle = bt.rope.animate(
    scene, cycle,
    name="cable",
    points=[tool, (tool[0] + 0.5, tool[1], tool[2])],
    grippers=[bt.rope.gripper(
        "grip", robot=scene.robot.name, link=scene.robot.tcp_link,
        location="Start",
    )],
    spacing_m=0.5 / 32,
    radius_m=0.005,
    density_kg_m=0.1,
    step_s=1 / 240,
)
record = bt.rope.track(cycle, "cable")
print(record["coupling"], record["events"])
```

The call broadcasts the augmented timeline to Studio. Seek, play and pause
display the robot, centerline, held points and dynamic connectors on one clock.
Each signal edge becomes a physical step boundary, even a pulse shorter than
`step_s`; `source_time_s` and `applied_time_s` record the exact adopted time.
The exact final sample includes final-time grasp/release events.

The physics steps every `step_s`; the playback track keeps every event (each
signal edge, the first and the final sample), at most 60 frames a second
otherwise, and only the two ends of a hold — a stretch where no point moves
more than 20 µm from the last kept frame. Linear blending between kept frames
(Studio, the USD export) stays within that of the simulated rope.

Grippers attach explicit rope material points without teleporting. They do
not establish contact/friction grasps or clamp cross-section orientation.
Locations are `"Start"`, `"End"`, or
`{"ArcLength": {"arc_length_m": 0.2, "tolerance_m": 0.001}}`.
The returned acquisition event identifies the sampled particle and link-local
anchor. Pin declarations are `(location, world_point)` pairs.

Include environment geometry with `obstacles=["table"]` and robot geometry
with `collision_links=[{"robot": scene.robot.name, "link": "tool"}]`.
Only these selected shapes collide. A gripper binding alone creates a frame
proxy without adding its collision geometry. Unsupported geometry is rejected
with a reason. The spring presets (`axial_hz=500`, `bending_hz=20`) are native
frequency settings and have no calibrated material-modulus claim: a higher
`bending_hz` is a stiffer cable, up to what the solver's step can hold.
`friction` is the rope collider's one coefficient; `color` its display colour
(linear RGB, like an obstacle's), carried into Studio and the USD export.

## Ends held in a part

A cable's end does not hang in the air: it is crimped into a connector
housing, held in a clip, clamped in a gland. `bt.rope.anchor` holds a span of
the rope on an obstacle (or a robot's link) for the whole replay. The obstacle
may move — one a robot attaches, carries and sets down — and the rope follows
its baked pose without pulling back:

```python
cycle = bt.rope.animate(
    scene, cycle, name="hv1",
    points=[crimp_a, crimp_b],                      # where it lies at time zero
    anchors=[bt.rope.anchor("hv1/plug_a", location="Start", length_m=0.045),
             bt.rope.anchor("hv1/plug_b", location="End", length_m=0.045)],
    obstacles=["pack/module_1", "pack/module_2"],
    spacing_m=0.01, radius_m=0.006, density_kg_m=0.4,
    color=(0.92, 0.17, 0.012),
)
```

`length_m` of material is held: inward from `"Start"`/`"End"`, centred on any
other location. Zero holds one sample, which stays free to turn; a span holds
two or more and with them the rope's direction — a cable leaves its housing
straight. Each held sample keeps the offset it has at time zero, so lay the
reference points where the housing holds them. The anchoring obstacle is a
frame only, and may be one the cell does not collision-check; it collides with
the rope only when it is also listed in `obstacles` (then it must be enabled).
`examples/assembly/ev_battery_harness_demo.py` plugs five HV module connectors
this way, two robots carrying each by its two housings.

Dynamic connectors use `connectors=[dict(name="plug", location="End",
shape={"Ball": 0.01}, pose=dict(position=[0.5,0,0.3], quaternion=[0,0,0,1]),
mass_kg=0.01)]`. A cuboid uses `shape={"Cuboid": [hx,hy,hz]}`. The constraint
captures the current particle's connector-local anchor; an input connector
pose does not silently snap the rope end onto the connector center.

`timeline.export_usd` writes each rope as a tube mesh under `/World/Ropes`
(eight sides at the rope's radius, capped) with its points time-sampled on
the export's frames; a hold costs its two ends.

Uses `rapier-rope` 0.1.0 from crates.io with f64 enabled. The botrail adapter remains unpublished. Rust users can configure the full
`RopeSpec` through `botrail_rope::pass::RopePass`. Run the primitive-only
`cargo run -p botrail-rope --example rope_replay` for a standalone recording.
Project persistence of ropes is not supported.
