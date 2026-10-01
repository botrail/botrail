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
with a reason. The spring presets (500 Hz axial / 20 Hz bending) are native
frequency settings and have no calibrated material-modulus claim.

Dynamic connectors use `connectors=[dict(name="plug", location="End",
shape={"Ball": 0.01}, pose=dict(position=[0.5,0,0.3], quaternion=[0,0,0,1]),
mass_kg=0.01)]`. A cuboid uses `shape={"Cuboid": [hx,hy,hz]}`. The constraint
captures the current particle's connector-local anchor; an input connector
pose does not silently snap the rope end onto the connector center.

Uses `rapier-rope` 0.1.0 from crates.io with f64 enabled. The botrail adapter remains unpublished. Rust users can configure the full
`RopeSpec` through `botrail_rope::pass::RopePass`. Run the primitive-only
`cargo run -p botrail-rope --example rope_replay` for a standalone recording.
Rope USD export and project persistence are not supported.
