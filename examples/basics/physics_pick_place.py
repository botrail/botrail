"""Physics pick & place demo: the release is real (design-physics.md P3).

Two handoffs, one arm. Part A is picked off its pedestal, carried over,
and released **5 mm above** the target plate — the gap every placement
authored with the "float the load a little" rule always had — and now it
visibly drops that 5 mm, seats, and sleeps: physics owns the part the
moment the grasp opens, seeded with the carrier's velocity. Part B makes
that seeding unmissable: the arm releases it **mid-swing**, and instead of
freezing where it was let go (the kinematic behavior), it flies on with
the wrist's tangential velocity and lands in the catch bin down range.

The arm itself is in the physics world too — every link is a kinematic
mirror body — so a deliberate contact (a guarded ramp through a part)
would shove it aside rather than pass through. Planned motions still
treat dynamic parts as obstacles at plan time; contact is authored with
ramps, exactly like weld approaches.

Run with:  python examples/basics/physics_pick_place.py [out.usda] [--studio]
"""

import math
import sys
from pathlib import Path

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[1]
scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
names = scene.robot.joint_names

# Taught configurations (no IK needed: the world is built around the FK).
HOME = [0.0, 0.6, 1.4, -0.6, 0.0, 0.0]
PICK = [0.0, 1.1, 0.9, 0.0, 0.0, 0.0]  # TCP at (0.64, 0, 0.105)
PLACE = list(PICK)
PLACE[0] = -1.22  # same radius/height, swung 70 deg clockwise
scene.set_joint_positions(PICK)
(px, py, pz), _ = scene.link_pose(scene.robot.tcp_link)
# The part hangs 5 cm *below* the wrist (suction-gripper style): the
# approach then comes from above and never pokes the part off its
# pedestal before the grasp — the arm's links are physical now, and a
# careless approach really does shove things (the rust push test makes
# that a feature; here it would be a fumble).
held = (px, py, pz - 0.05)
place_held = (
    math.cos(PLACE[0]) * math.hypot(px, py),
    math.sin(PLACE[0]) * math.hypot(px, py),
    pz - 0.05,
)
scene.set_joint_positions(HOME)

# The world around those poses: floor, a pedestal under the pick (its top
# exactly at the part's bottom), and a target plate 5 mm lower — that gap
# is the drop the release makes real.
# Floor top 5 mm below the robot base so the live scene shows no
# base-plate contact (obstacle-vs-robot touching flags as collision).
scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.055),
              color=(0.35, 0.37, 0.40))
part_bottom = held[2] - 0.025
scene.add_box("pedestal", size=(0.14, 0.14, part_bottom),
              position=(held[0], held[1], part_bottom / 2),
              color=(0.45, 0.47, 0.50))
scene.add_box("plate", size=(0.2, 0.2, part_bottom - 0.005),
              position=(place_held[0], place_held[1], (part_bottom - 0.005) / 2),
              color=(0.80, 0.65, 0.15))
scene.add_box("part_a", size=(0.05, 0.05, 0.05), position=held,
              color=(0.85, 0.33, 0.20))
scene.set_physics("part_a", dynamic=True, mass=0.2, friction=0.6)

# Part B waits on the same pedestal footprint, opposite swing side; its
# catch bin sits where the mid-swing release drops it.
scene.set_joint_positions([0.96, *PICK[1:]])
(bx, by, bz), _ = scene.link_pose(scene.robot.tcp_link)
scene.set_joint_positions(HOME)
scene.add_box("pedestal_b", size=(0.14, 0.14, part_bottom),
              position=(bx, by, part_bottom / 2), color=(0.45, 0.47, 0.50))
scene.add_box("part_b", size=(0.05, 0.05, 0.05), position=(bx, by, held[2]),
              color=(0.30, 0.55, 0.80))
scene.set_physics("part_b", dynamic=True, mass=0.2, friction=0.6)
# The catch bin sits where the free flight lands, with walls *below* the
# release height — the part has to arc down into it, not be fenced off.
BIN = (-0.28, 0.61)
scene.add_box("bin_floor", size=(0.6, 0.6, 0.02), position=(*BIN, 0.01),
              color=(0.25, 0.28, 0.32))
for name, dx, dy, sx, sy in [
    ("bin_n", 0.0, 0.30, 0.64, 0.02), ("bin_s", 0.0, -0.30, 0.64, 0.02),
    ("bin_e", 0.30, 0.0, 0.02, 0.6), ("bin_w", -0.30, 0.0, 0.02, 0.6),
]:
    scene.add_box(name, size=(sx, sy, 0.05), position=(BIN[0] + dx, BIN[1] + dy, 0.025),
                  color=(0.25, 0.28, 0.32))

ramp = lambda q, s=0.8: bt.seq.ramp(dict(zip(names, q)), s)  # noqa: E731

sq = scene.sequence("cycle")
# --- part A: pick, carry, and a 5 mm seating drop -----------------------
sq.step("approach_a", actions=[ramp(PICK)])
sq.step("grab_a", actions=[bt.seq.attach("part_a")])
sq.step("carry_a", actions=[ramp(PLACE, 1.2)])
sq.step("release_a", actions=[bt.seq.detach("part_a")],
        transition=bt.seq.elapsed(1.0))
# --- part B: released mid-swing, flies into the bin ---------------------
sq.step("approach_b", actions=[ramp([0.96, *PICK[1:]], 1.2)])
sq.step("grab_b", actions=[bt.seq.attach("part_b")])
sq.step("throw_b", actions=[ramp([2.27, *PICK[1:]], 1.0)],
        transition=bt.seq.elapsed(0.5))
sq.step("release_b", actions=[bt.seq.detach("part_b")],
        transition=bt.seq.elapsed(1.2))
sq.step("park", actions=[ramp(HOME)], transition=bt.seq.all_of(
    bt.seq.done(), bt.seq.elapsed(2.5)))

timeline = scene.simulate_sequence("cycle", physics=True, max_duration=30.0)
print(f"baked {timeline.duration:.2f}s under physics={timeline.physics!r}")

rel_a = timeline.step_span("release_a").start
za0 = timeline.object_pose("part_a", rel_a)[0][2]
(ax, ay, az), _ = timeline.object_pose("part_a", timeline.duration)
print(f"  part_a released at z={za0:.3f}, seated at z={az:.3f} "
      f"(dropped {1000 * (za0 - az):.1f} mm onto the plate)")

rel_b = timeline.step_span("release_b").start
(rx, ry, rz), _ = timeline.object_pose("part_b", rel_b)
(ebx, eby, ebz), _ = timeline.object_pose("part_b", timeline.duration)
flew = math.hypot(ebx - rx, eby - ry)
in_bin = abs(ebx - BIN[0]) < 0.3 and abs(eby - BIN[1]) < 0.3
print(f"  part_b released mid-swing at ({rx:+.2f}, {ry:+.2f}), flew "
      f"{flew:.2f} m, {'landed in the bin' if in_bin else f'landed at ({ebx:+.2f}, {eby:+.2f})'}")

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "physics_pick_place.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

if "--studio" in sys.argv:
    bt.studio(scene)
