"""A reach task as a gymnasium environment, derived from a cell.

`botrail.rl` turns a cell and a `Task` into an environment: the cell's own
rollout opened tick by tick, the arm driven by the action every control
period, the PLC programs and physics running underneath. Here the task is
to bring the TCP to a goal marker that moves every episode; a dynamic
block sits on the table so the physics is live too. A scripted
controller (the goal vector, observed in the TCP frame, is the action)
plays a few episodes and shows the numbers; `--train` fits a PPO policy
with stable-baselines3 when it is installed. Either way the last episode
closes into an ordinary timeline: `--studio` plays it, an output path
writes USD.

    python examples/rl/reach_env.py [out.usda] [--train] [--studio]
"""

from __future__ import annotations

import sys
from pathlib import Path

import botrail as bt
import numpy as np
from botrail import rl

ASSETS = Path(__file__).resolve().parents[1] / "assets"
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]


def build() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
    scene.set_joint_positions(READY)
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06), color=(0.35, 0.35, 0.38))
    scene.add_box("table", size=(0.6, 0.6, 0.1), position=(0.55, 0.0, 0.05), color=(0.6, 0.55, 0.45))
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(0.55, 0.0, 0.135), color=(0.85, 0.33, 0.20))
    scene.set_physics("part", dynamic=True, mass=0.2, friction=0.5)
    # The goal is a picture, not a thing: drawn, never collided with.
    scene.add_box("goal", size=(0.02, 0.02, 0.02), position=(0.45, 0.1, 0.5), color=(0.2, 0.8, 0.3))
    scene.set_obstacle_enabled("goal", False)
    sq = scene.sequence("run")
    sq.step("wait", transition=bt.seq.elapsed(30.0))
    return scene


def randomize(scene: bt.Scene, rng: np.random.Generator) -> None:
    scene.set_obstacle_pose("goal", (0.40 + rng.uniform(-0.05, 0.05), rng.uniform(-0.15, 0.15), 0.45 + rng.uniform(0.0, 0.1)))
    scene.set_obstacle_pose("part", (0.55 + rng.uniform(-0.03, 0.03), rng.uniform(-0.05, 0.05), 0.135))


task = rl.Task(
    control=rl.TcpDelta(max_step_m=0.02, hz=20, frame="tcp"),
    observe=[
        rl.Joints(),
        rl.TcpPose(),
        rl.Relative("tcp", "goal"),
        rl.ObjectPose("part"),
        rl.Contacts("simple_arm/tool0", "part"),
        rl.Collision(),
    ],
    reset=randomize,
    reward=rl.rewards.combine(
        rl.rewards.reach("tcp→goal"),
        rl.rewards.bonus("tcp→goal", 0.02, 1.0),
        rl.rewards.collision_penalty(1.0),
    ),
    done=rl.rewards.within("tcp→goal", 0.02),
    horizon_s=4.0,
)
env = rl.make(build, task, seed=0)
print(f"observation {env.observation_space.shape}, action {env.action_space.shape}, "
      f"{env.scans_per_step} scans per step")


def scripted(channels) -> np.ndarray:
    """The goal vector in the TCP frame, clipped to one step: a P-controller."""
    return np.clip(channels["tcp→goal"][:3] / task.control.max_step_m, -1.0, 1.0)


def play(policy, episodes: int, label: str) -> None:
    for episode in range(episodes):
        obs, info = env.reset()
        total, steps = 0.0, 0
        while True:
            obs, reward, terminated, truncated, info = env.step(policy(obs, info))
            total += reward
            steps += 1
            if terminated or truncated:
                break
        outcome = "reached" if terminated and "error" not in info else "ran out"
        print(f"  {label} episode {episode}: {outcome} after {steps} steps ({info['t']:.2f}s), return {total:+.2f}")


print("scripted controller:")
play(lambda obs, info: scripted(info["channels"]), episodes=3, label="scripted")

if "--train" in sys.argv:
    try:
        from stable_baselines3 import PPO
    except ImportError:
        sys.exit("--train needs stable-baselines3: pip install stable-baselines3")
    model = PPO("MlpPolicy", env, n_steps=256, batch_size=64, verbose=0, seed=0)
    model.learn(total_timesteps=int(next((a for a in sys.argv[1:] if a.isdigit()), "20000")))
    print("learned policy:")
    play(lambda obs, info: model.predict(obs, deterministic=True)[0], episodes=3, label="ppo")

# The last episode, as the timeline every other tool reads.
timeline = env.timeline(publish=True)
print(f"last episode: {timeline.duration:.2f}s, min clearance {float(timeline.min_clearance()):.3f} m, "
      f"{len(timeline.contacts)} contact episodes")
args = [a for a in sys.argv[1:] if not a.startswith("--") and not a.isdigit()]
out = args[0] if args else "reach_env.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

if "--studio" in sys.argv:
    bt.studio(env.scene)
