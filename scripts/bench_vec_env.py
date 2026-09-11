"""Throughput of botrail.rl environments: env-steps per second for one
world and for N worlds stepped together (design-rl.md R2 acceptance).

    python scripts/bench_vec_env.py [--envs 1,4,16,32] [--steps 200]

The cell is the reach task of examples/rl/reach_env.py with four dynamic
parts on the table (the "pick scene" of the design's target); the control
is JointDelta (no IK on the Python side) and, second, TcpDelta (one IK
per world per step in Python). Reset cost is reported separately.
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import botrail as bt
import numpy as np
from botrail import rl

ASSETS = Path(__file__).resolve().parents[1] / "examples" / "assets"
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]


def build() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
    scene.set_joint_positions(READY)
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06))
    scene.add_box("table", size=(0.6, 0.6, 0.1), position=(0.55, 0.0, 0.05))
    for i in range(4):
        scene.add_box(f"part{i}", size=(0.05, 0.05, 0.05), position=(0.45 + 0.07 * i, 0.2, 0.135))
        scene.set_physics(f"part{i}", dynamic=True, mass=0.2)
    scene.add_box("goal", size=(0.02, 0.02, 0.02), position=(0.45, -0.1, 0.5))
    scene.set_obstacle_enabled("goal", False)
    # A planar 270°/0.5° scanner (541 beams) beside the cell, for --lidar.
    scene.add_lidar("front", position=(0.0, -0.6, 0.3), fov=270.0, range=(0.05, 8.0), resolution=0.5)
    # A wrist camera looking down the tool, for --depth (64×64 by default).
    scene.add_camera("wrist", position=(0.0, 0.0, 0.05), quaternion=(1.0, 0.0, 0.0, 0.0), fov=70.0, resolution=(64, 64), near=0.05, far=3.0, robot="simple_arm", link="tool0")
    sq = scene.sequence("run")
    sq.step("wait", transition=bt.seq.elapsed(30.0))
    return scene


def randomize(scene: bt.Scene, rng: np.random.Generator) -> None:
    scene.set_obstacle_pose("goal", (0.40 + rng.uniform(-0.05, 0.05), -0.1 + rng.uniform(-0.1, 0.1), 0.45 + rng.uniform(0.0, 0.1)))


LIDAR = "--lidar" in sys.argv
DEPTH = "--depth" in sys.argv
RGB = "--rgb" in sys.argv
SHADOWS = "--shadows" in sys.argv
DEPTH_SIZE = int(sys.argv[sys.argv.index("--size") + 1]) if "--size" in sys.argv else 64


def task(control) -> rl.Task:
    observe = [rl.Joints(), rl.TcpPose(), rl.Relative("tcp", "goal"), rl.ObjectPose("part0"), rl.Contacts("simple_arm/tool0", "part0"), rl.Collision()]
    if LIDAR:
        observe.append(rl.Lidar("front", noise=0.01))
    if DEPTH:
        observe.append(rl.Depth("wrist", size=(DEPTH_SIZE, DEPTH_SIZE)))
    if RGB:
        observe.append(rl.Rgb("wrist", size=(DEPTH_SIZE, DEPTH_SIZE)))
    return rl.Task(
        control=control,
        observe=observe,
        reset=randomize,
        reward=rl.rewards.reach("tcp→goal"),
        horizon_s=3.0,
        render={"shadows": True} if SHADOWS else None,
    )


def bench(control_name: str, control, num_envs: int, steps: int) -> None:
    env = rl.make(build, task(control), seed=0, num_envs=num_envs)
    t0 = time.perf_counter()
    env.reset(seed=0)
    reset_ms = (time.perf_counter() - t0) * 1000.0
    rng = np.random.default_rng(0)
    shape = (num_envs, control.dim) if num_envs > 1 else (control.dim,)
    t0 = time.perf_counter()
    for _ in range(steps):
        out = env.step(rng.uniform(-1.0, 1.0, size=shape))
        if num_envs == 1 and (out[2] or out[3]):
            env.reset()
    elapsed = time.perf_counter() - t0
    rate = steps * num_envs / elapsed
    tag = (" +lidar" if LIDAR else "") + (f" +depth{DEPTH_SIZE}" if DEPTH else "") + (f" +rgb{DEPTH_SIZE}" if RGB else "") + (" +shadows" if SHADOWS else "")
    print(f"{control_name:11s}{tag} N={num_envs:3d}: {rate:9.0f} env-steps/s  ({elapsed / steps * 1000:.2f} ms/step, reset {reset_ms:.1f} ms)")


if __name__ == "__main__":
    args = sys.argv[1:]
    sizes = [1, 4, 16, 32]
    steps = 200
    if "--envs" in args:
        sizes = [int(x) for x in args[args.index("--envs") + 1].split(",")]
    if "--steps" in args:
        steps = int(args[args.index("--steps") + 1])
    for n in sizes:
        bench("JointDelta", rl.JointDelta(max_step=0.02, hz=20), n, steps)
    for n in sizes:
        bench("TcpDelta", rl.TcpDelta(max_step_m=0.01, hz=20, frame="tcp"), n, steps)
