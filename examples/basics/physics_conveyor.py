"""Physics conveyor demo: friction transport, a stopper queue, and the
sensor → PLC chain fed by contact (design-physics.md P2).

The belt is authored exactly like every botrail conveyor — a zone box and
a velocity (`add_conveyor`) — but the cartons on it are marked *dynamic*,
so instead of being advected they are **carried by friction**: the running
belt drives the contact points under them (rapier's conveyor mechanism),
they accelerate to belt speed, ride, and one by one press into the stopper
at the far end, queueing up behind each other. A presence sensor at the
stopper sees the first carton arrive; the program runs the belt on for a
couple of seconds so the queue seats, then cuts it — a physics event
driving a PLC transition through the ordinary sensor → SFC chain.

The same cell with `dynamic=False` cartons advects kinematically, exactly
as before: one belt, two transport modes.

Run with:  python examples/basics/physics_conveyor.py [out.usda] [--studio]
"""

import sys

import botrail as bt

BELT_TOP = 0.70
CARTON = (0.12, 0.09, 0.07)

scene = bt.Scene()

# The belt bed (what the cartons actually rest on), a return-side skirt,
# and the stopper across the far end.
scene.add_box("bed", size=(2.6, 0.32, 0.08), position=(0.0, 0.0, BELT_TOP - 0.04),
              color=(0.30, 0.33, 0.38))
scene.add_box("leg_a", size=(0.06, 0.28, 0.62), position=(-1.1, 0.0, 0.31),
              color=(0.45, 0.47, 0.50))
scene.add_box("leg_b", size=(0.06, 0.28, 0.62), position=(1.1, 0.0, 0.31),
              color=(0.45, 0.47, 0.50))
scene.add_box("stopper", size=(0.04, 0.32, 0.12), position=(1.1, 0.0, BELT_TOP + 0.06),
              color=(0.80, 0.65, 0.15))

# Three cartons queued down the belt, dropped in place when the bake
# starts. Dynamic: the belt reaches them through friction, not advection.
for name, x, color in [
    ("carton_a", -1.05, (0.85, 0.33, 0.20)),
    ("carton_b", -0.60, (0.90, 0.62, 0.18)),
    ("carton_c", -0.15, (0.30, 0.55, 0.80)),
]:
    scene.add_box(name, size=CARTON, position=(x, 0.0, BELT_TOP + 0.06), color=color)
    scene.set_physics(name, dynamic=True, mass=0.5, friction=0.6)

# The conveyor: the authored zone + velocity, unchanged vocabulary. The
# zone hugs the carry volume and leaves the bed/leg/stopper origins out —
# the advection captures origins indiscriminately, and the physics mirror
# faithfully moves whatever the zone swallows.
scene.add_conveyor("conv", zone_position=(-0.15, 0.0, BELT_TOP + 0.135),
                   zone_size=(2.3, 0.32, 0.27), velocity=(0.35, 0.0, 0.0),
                   running=False)

# Presence sensor just short of the stopper — whichever carton arrives
# first trips it (carton_c leads: it starts nearest the stopper).
scene.add_zone_sensor("at_stop", position=(0.95, 0.0, BELT_TOP + 0.08),
                      size=(0.15, 0.32, 0.14))

# The program: start the belt, wait for arrival, run on so the queue
# seats against the stopper, then cut the belt and let everything sleep.
sq = scene.sequence("run")
sq.step("feed", actions=[bt.seq.start("conv")],
        transition=bt.seq.signal("at_stop", True))
sq.step("seat", transition=bt.seq.elapsed(3.5))
sq.step("halt", actions=[bt.seq.stop("conv")],
        transition=bt.seq.elapsed(2.5))

timeline = scene.simulate_sequence("run", physics=True, max_duration=30.0)
arrival = next(t for t, v in timeline.signal("at_stop").edges if v)
x = lambda name, t: timeline.object_pose(name, t)[0][0]
cruise = (x("carton_a", 2.5) - x("carton_a", 1.5)) / 1.0
print(f"baked {timeline.duration:.2f}s under physics={timeline.physics!r}")
print(f"  cruise speed {cruise:.3f} m/s (belt commands 0.35)")
print(f"  sensor 'at_stop' rose at t={arrival:.2f}s; belt cut at t={arrival + 3.5:.2f}s")
print("  queue against the stopper (centers, 0.12 m pitch):")
for name in ["carton_a", "carton_b", "carton_c"]:
    print(f"    {name}: x = {x(name, timeline.duration):+.3f}")

# --- what happened, as data (design-physics.md P4) ----------------------
# Touch episodes tell the queue's story: each carton lands on the bed,
# the leader presses the stopper, and the followers couple up one by one.
print("  contact episodes:")
for c in timeline.contacts:
    print(f"    {c['a']} × {c['b']}: t={c['start']:.2f}s, peak {c['peak_force']:.0f} N")
print("  stalls (belt driving under an arrested carton):")
for s_ in timeline.conveyor_stalls():
    print(f"    {s_['object']}: [{s_['start']:.2f} → {s_['end']:.2f}] on {s_['device']}")
for name in ["carton_a", "carton_b", "carton_c"]:
    print(f"  {name} asleep from t={timeline.settled_at(name):.2f}s")

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "physics_conveyor.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

# Watch it: press play in the studio, or open the USD in usdview.
if "--studio" in sys.argv:
    bt.studio(scene)
