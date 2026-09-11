"""A hand-written controller closing the loop on a live rollout.

`Scene.open_rollout` opens the very rollout `simulate_sequence` bakes, but
stopped at t = 0, to be advanced one scan tick at a time. Between ticks
Python reads the world and drives the arm: every control period (20 Hz,
five 10 ms scans) the TCP error to a goal is turned into a joint target
through IK, and the drive's rate limit — the model's joint velocity
limits — turns that into a servo move. A dynamic block on the table is
pushed across it when the goal runs through it, and the whole thing
finishes into an ordinary timeline: `--studio` plays it, an output path
writes USD.

This is the loop a learned policy closes in design-rl.md R1; here a
P-controller stands in for the policy.
"""

from __future__ import annotations

import sys
from pathlib import Path

import botrail as bt

ASSETS = Path(__file__).resolve().parents[1] / "assets"
TABLE_TOP = 0.10
HZ = 20  # control periods per second (the scan runs at 100 Hz)

scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06), color=(0.35, 0.35, 0.38))
scene.add_box("table", size=(0.6, 0.6, TABLE_TOP), position=(0.55, 0.0, TABLE_TOP / 2),
              color=(0.6, 0.55, 0.45))
# A block tall enough to be pushed near its middle with the tool clear of
# the table, and slippery enough (μ 0.4) to slide rather than tip.
scene.add_box("part", size=(0.06, 0.06, 0.12), position=(0.55, 0.0, TABLE_TOP + 0.06 + 0.005),
              color=(0.85, 0.33, 0.20))
scene.set_physics("part", dynamic=True, mass=0.2, friction=0.4)

# The PLC side is one timer step: the program is "keep the cell alive
# for eight seconds". A real cell would run its belts and sensors here.
sq = scene.sequence("hold")
sq.step("run", transition=bt.seq.elapsed(8.0))

# Goals for the TCP, in order: hover above the block, come down beside
# it, push it 12 cm across the table, and lift away.
part = (0.55, 0.0, TABLE_TOP + 0.06)
PUSH_Z = part[2]  # at the block's middle: the tool's ~6 cm of geometry clears the table
GOALS = [
    (part[0] - 0.10, part[1], part[2] + 0.25),
    (part[0] - 0.10, part[1], PUSH_Z),
    (part[0] + 0.02, part[1], PUSH_Z),
    (part[0] + 0.02, part[1], part[2] + 0.25),
]
REACHED = 0.01  # m

live = scene.open_rollout("hold", physics=True)
live.drive(max_velocity=1.0)
scans_per_period = round(1.0 / (HZ * live.dt))
goal_index = 0
pushed = False
while not live.finished and goal_index < len(GOALS):
    goal = GOALS[goal_index]
    (tx, ty, tz), _ = live.tcp_pose()
    error = ((goal[0] - tx) ** 2 + (goal[1] - ty) ** 2 + (goal[2] - tz) ** 2) ** 0.5
    if error < REACHED:
        goal_index += 1
        continue
    # A proportional step toward the goal, solved to joints from where
    # the arm stands (no restarts: the solution must stay on this branch).
    gain = 0.5
    step = (tx + gain * (goal[0] - tx), ty + gain * (goal[1] - ty), tz + gain * (goal[2] - tz))
    ik = scene.robot.ik(step, seed=live.joint_positions(), restarts=0)
    live.command(ik.q)
    live.tick(scans_per_period)
    touching = [(a, b) for a, b, _ in live.contacts() if "part" in (a, b) and "table" not in (a, b)]
    if touching and not pushed:
        pushed = True
        print(f"t = {live.t:.2f}s: the tool touches the part ({touching[0][0]} × {touching[0][1]})")
    if live.collisions():
        print(f"t = {live.t:.2f}s: collision {live.collisions()}")

(x0, y0, _) = part
(x, y, z), _ = live.object_pose("part")
print(f"goals reached: {goal_index}/{len(GOALS)} by t = {live.t:.2f}s")
print(f"the part moved ({x - x0:+.3f}, {y - y0:+.3f}) m and rests at z = {z:.3f}")

timeline = live.finish()
print(f"baked {timeline.duration:.2f}s under physics={timeline.physics!r}, "
      f"{len(timeline.contacts)} contact episodes")

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "reach_control.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

if "--studio" in sys.argv:
    bt.studio(scene)
