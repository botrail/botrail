"""Torque control on a dynamic robot (design-rl-dynamics.md).

`scene.set_robot_physics()` turns the arm into an articulated body under a
physics bake: links weigh what the model says (here: their shapes), joints
are force-capped servos, and the baked lane is the physical joint state.
`rl.Torque` then hands a policy the joint torques themselves — the servos
are off, gravity is the policy's to hold. A scripted PD controller in
torque space holds READY against gravity and shows the numbers; `--train`
fits PPO with stable-baselines3. The last episode closes into an ordinary
timeline: `--studio` plays it, an output path writes USD.

    python examples/rl/torque_env.py [out.usda] [--train] [--studio]
"""

from __future__ import annotations

import sys
from pathlib import Path

import botrail as bt
import numpy as np
from botrail import rl

ASSETS = Path(__file__).resolve().parents[1] / "assets"
READY = np.array([0.0, 0.6, 0.8, 0.0, 0.5, 0.0])


def build() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
    scene.set_joint_positions(READY.tolist())
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06), color=(0.35, 0.35, 0.38))
    scene.add_box("table", size=(0.6, 0.6, 0.1), position=(0.55, 0.0, 0.05), color=(0.6, 0.55, 0.45))
    # The arm is a body: shape-derived link masses, servos capped at the
    # URDF effort limits, a small reflected drive inertia per joint.
    scene.set_robot_physics()
    sq = scene.sequence("run")
    sq.step("wait", transition=bt.seq.elapsed(30.0))
    return scene


def randomize(scene: bt.Scene, rng: np.random.Generator) -> None:
    # Start a little off READY every episode: the policy has to bring the
    # arm back and hold it.
    scene.set_joint_positions((READY + rng.uniform(-0.15, 0.15, size=6)).tolist())


def hold_reward(obs, info) -> float:
    q = obs["simple_arm/joints"][:6]
    qd = obs["simple_arm/joints"][6:]
    return float(-np.abs(q - READY).sum() - 0.01 * np.abs(qd).sum())


task = rl.Task(
    control=rl.Torque(hz=50),
    observe=[rl.Joints()],
    reset=randomize,
    reward=hold_reward,
    horizon_s=3.0,
)
env = rl.make(build, task, seed=0, flatten=False)
print(f"action {env.action_space.shape} (torques scaled by the effort limits), {env.scans_per_step} scans per step")

CAPS = np.array([50.0, 50.0, 30.0, 10.0, 10.0, 10.0])  # the arm's effort limits, N·m


class Scripted:
    """A PID controller in torque space: the integral term is what holds
    the arm against gravity, the way a servo drive's does."""

    def __init__(self) -> None:
        self.integral = np.zeros(6)

    def reset(self) -> None:
        self.integral[:] = 0.0

    def __call__(self, obs) -> np.ndarray:
        q = obs["simple_arm/joints"][:6]
        qd = obs["simple_arm/joints"][6:]
        error = READY - q
        self.integral = np.clip(self.integral + error / task.control.hz, -0.5, 0.5)
        tau = 120.0 * error + 200.0 * self.integral - 6.0 * qd
        return np.clip(tau / CAPS, -1.0, 1.0)


def play(policy, episodes: int, label: str) -> None:
    for episode in range(episodes):
        obs, info = env.reset()
        if hasattr(policy, "reset"):
            policy.reset()
        total, steps = 0.0, 0
        while True:
            obs, reward, terminated, truncated, info = env.step(policy(obs))
            total += reward
            steps += 1
            if terminated or truncated:
                break
        err = np.abs(obs["simple_arm/joints"][:6] - READY).max()
        print(f"  {label} episode {episode}: {steps} steps, return {total:+.2f}, final error {err:.4f} rad")


print("scripted PID in torque space:")
play(Scripted(), episodes=3, label="pid")

if "--train" in sys.argv:
    try:
        from stable_baselines3 import PPO
    except ImportError:
        sys.exit("--train needs stable-baselines3: pip install stable-baselines3")
    flat = rl.make(build, task, seed=0)
    model = PPO("MlpPolicy", flat, n_steps=256, batch_size=64, verbose=0, seed=0)
    model.learn(total_timesteps=int(next((a for a in sys.argv[1:] if a.isdigit()), "20000")))
    print("learned policy:")
    play(lambda obs: model.predict(np.concatenate([obs["simple_arm/joints"]]), deterministic=True)[0], episodes=3, label="ppo")

timeline = env.timeline(publish=True)
print(f"last episode: {timeline.duration:.2f}s of physical joint state on the robot lane")
args = [a for a in sys.argv[1:] if not a.startswith("--") and not a.isdigit()]
out = args[0] if args else "torque_env.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

if "--studio" in sys.argv:
    bt.studio(env.scene)
