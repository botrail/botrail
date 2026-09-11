"""A policy as one step of a cell — the loop back from learning.

The cell is a small feeder: a belt carries a block to a stop, a zone
sensor trips, and the arm has to reach the block. That reach is not a
taught motion but a `Policy` step (`bt.seq.policy`): a controller
registered at bake time — here the scripted P-controller of
`reach_env.py`, or a trained model passed with `--policy file.onnx|.zip`
— asked for a joint target 20 times a second until it declares itself
done. Everything around it is the ordinary cell: the belt runs under
physics, the sensor gates the step, a planned ramp brings the arm home,
and the bake is deterministic. What comes out is the same timeline as
ever: the cycle time, the contacts, the robot lane with a `policy reach`
stretch, USD, and the studio (`--studio`).

    python examples/rl/policy_cell_demo.py [out.usda] [--policy model.onnx] [--studio]
"""

from __future__ import annotations

import sys
from pathlib import Path

import botrail as bt
import numpy as np
from botrail import rl

ASSETS = Path(__file__).resolve().parents[1] / "assets"
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]
BELT_TOP = 0.10

scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
scene.set_joint_positions(READY)
scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06), color=(0.35, 0.35, 0.38))
# The belt bed, its zone, and a stop at the arm's end of it.
scene.add_box("bed", size=(1.2, 0.3, BELT_TOP), position=(0.6, 0.35, BELT_TOP / 2), color=(0.25, 0.28, 0.32))
scene.add_conveyor("belt", zone_position=(0.6, 0.35, BELT_TOP + 0.08), zone_size=(1.2, 0.3, 0.16), velocity=(-0.25, 0.0, 0.0))
scene.add_box("stop", size=(0.02, 0.3, 0.08), position=(0.19, 0.35, BELT_TOP + 0.04), color=(0.6, 0.6, 0.62))
scene.add_box("part", size=(0.06, 0.06, 0.06), position=(1.05, 0.35, BELT_TOP + 0.03 + 0.005), color=(0.85, 0.33, 0.20))
scene.set_physics("part", dynamic=True, mass=0.2, friction=0.5)
scene.add_zone_sensor("at_stop", position=(0.27, 0.35, BELT_TOP + 0.06), size=(0.12, 0.3, 0.12), watch=["part"])

names = scene.robot.joint_names
sq = scene.sequence("cycle")
sq.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("at_stop", True))
sq.step("seat", transition=bt.seq.elapsed(0.6))
sq.step("halt", actions=[bt.seq.stop("belt")], transition=bt.seq.elapsed(0.2))
# The learned (or scripted) part of the cycle: reach the block where it sits.
sq.step("reach", actions=[bt.seq.policy("reach", hz=20, max_duration=6.0)], transition=bt.seq.done())
sq.step("home", actions=[bt.seq.ramp(dict(zip(names, READY)), 1.0)], transition=bt.seq.done())

# The policy's task: what it observes and how it acts — the same spec a
# training run would use (reach_env.py), the goal now being a hover point
# 10 cm above the block wherever the belt left it.
HOVER = np.array([0.0, 0.0, 0.10])


def hover_error(obs) -> np.ndarray:
    """World-frame vector from the TCP to the hover point (channel
    layout: joints 12, tcp 7, part pose 7)."""
    return obs[19:22] + HOVER - obs[12:15]


task = rl.Task(
    control=rl.TcpDelta(max_step_m=0.02, hz=20),
    observe=[rl.Joints(), rl.TcpPose(), rl.ObjectPose("part"), rl.Contacts("simple_arm/tool0", "part")],
    done=lambda obs, info: float(np.linalg.norm(
        obs["part/pose"][:3] + HOVER - obs["simple_arm/tcp"][:3]
    )) < 0.02,
)


def scripted(obs: np.ndarray) -> np.ndarray:
    """A P-controller in the world frame the control acts in."""
    return np.clip(hover_error(obs) / 0.02, -1.0, 1.0)


args = sys.argv[1:]
if "--policy" in args:
    policy = rl.load(args[args.index("--policy") + 1], scene, task)
else:
    policy = rl.Policy(scene, task, scripted)

timeline = scene.simulate_sequence("cycle", physics=True, max_duration=30.0, policies={"reach": policy})
print(f"cycle time {timeline.duration:.2f}s under physics={timeline.physics!r}")
for name, start, end in timeline.step_spans:
    print(f"  {name:6s} {start:6.2f} → {end:6.2f}s")
for name, start, end in timeline.moves():
    print(f"  robot lane: {name} [{start:.2f}, {end:.2f}]")
print(f"  {len(timeline.contacts)} contact episodes, min clearance {float(timeline.min_clearance()):.3f} m")

out = next((a for a in args if not a.startswith("--") and a != (args[args.index("--policy") + 1] if "--policy" in args else None)), "policy_cell.usda")
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

if "--studio" in args:
    bt.studio(scene)
