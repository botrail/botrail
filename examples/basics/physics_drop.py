"""Physics drop demo: parts fall, bounce, stack and settle (design-physics.md P1).

The first physics bake. Three cartons are authored a metre above a table
and marked *dynamic* (`scene.set_physics`); everything else — the table,
the tray they land in — is exactly the static scenery it always was. The
sequence is a plain timer wait: physics needs no choreography, gravity is
the program. `simulate_sequence(..., physics=True)` steps the scene under
Rapier at 400 Hz inside the ordinary 100 Hz PLC scan loop, and the result
is a normal `SequenceTimeline` — the cartons' tumble is just another
object track, so studio playback, USD export and pose queries all work
unchanged. Without `physics=True` the same scene bakes kinematically and
the cartons hang in the air: the properties are authoring data, inert
until an engine is asked for.

The bake is deterministic per machine and build (same scene → the same
timeline, bit for bit) — run it twice and diff the USD if you doubt it.

Run with:  python examples/basics/physics_drop.py [out.usda]
"""

import sys

import botrail as bt

scene = bt.Scene()

# The static world: a table, and a shallow tray for the parts to land in.
scene.add_box("table", size=(1.2, 0.8, 0.72), position=(0.0, 0.0, 0.36),
              color=(0.58, 0.44, 0.31))
scene.add_box("tray_floor", size=(0.5, 0.4, 0.02), position=(0.1, 0.0, 0.73),
              color=(0.25, 0.28, 0.32))
for name, dx, dy, sx, sy in [
    ("tray_north", 0.0, 0.21, 0.54, 0.02),
    ("tray_south", 0.0, -0.21, 0.54, 0.02),
    ("tray_east", 0.26, 0.0, 0.02, 0.4),
    ("tray_west", -0.26, 0.0, 0.02, 0.4),
]:
    scene.add_box(name, size=(sx, sy, 0.06), position=(0.1 + dx, dy, 0.75),
                  color=(0.25, 0.28, 0.32))

# The dynamic parts: three cartons queued in the air, dropped in one go.
# Slightly offset so they land on each other and have to sort it out.
CARTONS = [
    ("carton_a", (0.10, 0.02, 1.60), (0.85, 0.33, 0.20)),
    ("carton_b", (0.13, -0.03, 1.85), (0.90, 0.62, 0.18)),
    ("carton_c", (0.07, 0.00, 2.10), (0.30, 0.55, 0.80)),
]
for name, position, color in CARTONS:
    scene.add_box(name, size=(0.12, 0.08, 0.06), position=position, color=color)
    scene.set_physics(name, dynamic=True, mass=0.4, friction=0.6, restitution=0.2)

# Physics needs no choreography: the program is one timer step.
sq = scene.sequence("drop")
sq.step("settle", transition=bt.seq.elapsed(3.0))

# First, the same scene with no engine: the marked cartons are plain
# static obstacles — nothing moves, nothing is tracked. (This comparison
# runs *before* the physics bake on purpose: the studio and the ⤓usd
# download replay the session's most recent bake, so the one worth
# watching has to be the last one made.)
kinematic = scene.simulate_sequence("drop")
assert kinematic.physics is None
print("kinematic bake leaves the cartons untouched (physics props are inert)")

timeline = scene.simulate_sequence("drop", physics=True)
print(f"baked {timeline.duration:.2f}s under physics={timeline.physics!r}")
for name, _, _ in CARTONS:
    (x, y, z), _ = timeline.object_pose(name, timeline.duration)
    print(f"  {name} settled at ({x:+.3f}, {y:+.3f}, {z:.3f})")

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "physics_drop.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

# Watch it: press play in the studio, or open the USD in usdview.
if "--studio" in sys.argv:
    bt.studio(scene)
