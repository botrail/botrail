"""A visual environment: a wrist camera's depth and colour pictures as
the observation, randomised looks per episode.

The arm has to reach a part it can only see: the part's pose is not in
the state, only in the wrist camera's depth and colour pictures (and the
segmentation, if a policy wants the label). Every episode the part is
moved and re-coloured, the light re-aimed, and the camera jittered on its
mount — the domain-randomisation hooks of `botrail.rl` (`Task.reset`,
`Task.render`, `Scene.set_camera_pose`). The pictures come from the
rollout's own rasteriser: the camera the studio evaluates the policy
with, at the intrinsics `capture_depth` writes out.

The reward reads a privileged channel — the part's pose relative to the
TCP, `privileged/tcp→part` — the way an asymmetric actor-critic does: the
critic may see it, a deployed policy reads the pictures only (drop it
from the actor's input). A scripted controller that cheats the same way
plays a few episodes and saves the first frames as PPM; `--train` fits a
PPO with stable-baselines3's `MultiInputPolicy`.

    python examples/rl/depth_pick_env.py [--train [steps]] [--out DIR]
"""

from __future__ import annotations

import sys
from pathlib import Path

import botrail as bt
import numpy as np
from botrail import rl

ASSETS = Path(__file__).resolve().parents[1] / "assets"
READY = [-0.042, 0.496, 0.947, 0.037, 0.438, 1.67]  # tool tilted 25° off straight down, 0.4 m above the table (straight down sits on the wrist singularity)
SIZE = (48, 48)


def build() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
    scene.set_joint_positions(READY)
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06), color=(0.35, 0.35, 0.38))
    scene.add_box("table", size=(0.7, 0.7, 0.1), position=(0.55, 0.0, 0.05), color=(0.6, 0.55, 0.45))
    scene.add_box("part", size=(0.05, 0.05, 0.05), position=(0.5, 0.0, 0.125), color=(0.85, 0.33, 0.20))
    scene.set_physics("part", dynamic=True, mass=0.2, friction=0.5)
    # A wrist camera looking straight down at the ready pose: its mount
    # quaternion undoes the tool's tilt (camera axes = world axes there).
    scene.add_camera("wrist", position=(0.0, 0.0, 0.05), quaternion=(0.0001, -0.9763, -0.2163, 0.0004), fov=70.0, resolution=SIZE, near=0.05, far=2.0, robot="simple_arm", link="tool0")
    sq = scene.sequence("run")
    sq.step("wait", transition=bt.seq.elapsed(30.0))
    return scene


def randomize(scene: bt.Scene, rng: np.random.Generator) -> None:
    scene.set_obstacle_pose("part", (0.45 + rng.uniform(0.0, 0.2), rng.uniform(-0.12, 0.12), 0.125))
    scene.set_obstacle_color("part", tuple(rng.uniform(0.2, 1.0, size=3)))
    scene.set_obstacle_color("table", tuple(rng.uniform(0.3, 0.8, size=3)))
    # The camera jitters on its mount: a millimetre or two, a degree or so.
    scene.set_camera_pose("wrist", (rng.normal(0.0, 0.002), rng.normal(0.0, 0.002), 0.05 + rng.normal(0.0, 0.002)))


def lighting(rng: np.random.Generator) -> dict:
    d = np.array([rng.uniform(-0.6, 0.6), rng.uniform(-0.6, 0.6), rng.uniform(0.5, 1.0)])
    return {"light": tuple(d), "ambient": float(rng.uniform(0.25, 0.5))}


HOVER = 0.10  # the reach target: this far above the part's centre

task = rl.Task(
    control=rl.TcpDelta(max_step_m=0.02, hz=20),
    observe=[
        rl.Joints(),
        rl.TcpPose(),
        rl.Depth("wrist", size=SIZE),
        rl.Rgb("wrist", size=SIZE),
        rl.Segmentation("wrist", size=SIZE),
        rl.Relative("tcp", "part", name="privileged/tcp→part"),
    ],
    reset=randomize,
    render=lighting,
    reward=rl.rewards.reach("privileged/tcp→part"),
    done=rl.rewards.within("privileged/tcp→part", HOVER + 0.02),
    horizon_s=4.0,
)


def target_error(env) -> np.ndarray:
    """What the scripted controller cheats with: the world-frame vector
    from the TCP to the hover point above the part."""
    (px, py, pz), _ = env.live.object_pose("part")
    (tx, ty, tz), _ = env.live.tcp_pose()
    return np.array([px - tx, py - ty, pz + HOVER - tz])


def scripted(env) -> np.ndarray:
    return np.clip(target_error(env) / 0.02, -1.0, 1.0)


def ppm(path: Path, rgb: np.ndarray) -> None:
    h, w = rgb.shape[:2]
    with open(path, "wb") as f:
        f.write(b"P6\n%d %d\n255\n" % (w, h))
        f.write(np.ascontiguousarray(rgb.astype(np.uint8)).tobytes())


args = sys.argv[1:]
out = Path(args[args.index("--out") + 1]) if "--out" in args else Path(".")
env = rl.make(build, task, seed=0, flatten=False)
print("observation:", {c.key: c.shape or (c.dim,) for c in env.channels})
ids = rl.segmentation_ids(env.scene)
print(f"segmentation id of the part: {ids['part']}")

for episode in range(3):
    obs, info = env.reset()
    if episode == 0:
        ppm(out / "wrist_rgb.ppm", obs["wrist/rgb"])
        depth = obs["wrist/depth"]
        ppm(out / "wrist_depth.ppm", np.repeat((np.clip(depth / 1.0, 0, 1) * 255)[..., None], 3, axis=-1))
        np.save(out / "wrist_depth.npy", depth)
        print(f"saved wrist_rgb.ppm / wrist_depth.ppm / wrist_depth.npy to {out.resolve()}")
    steps = 0
    seen = 0
    while True:
        obs, reward, terminated, truncated, info = env.step(scripted(env))
        steps += 1
        seen += int((obs["wrist/segmentation"] == ids["part"]).any())
        if float(np.linalg.norm(target_error(env))) < 0.02 or terminated or truncated:
            break
    print(f"  episode {episode}: {steps} steps ({info['t']:.2f}s), the part was in the picture on {seen} of them, "
          f"hover error {float(np.linalg.norm(target_error(env))):.3f} m")

if "--train" in args:
    try:
        from stable_baselines3 import PPO
    except ImportError:
        sys.exit("--train needs stable-baselines3: pip install stable-baselines3")
    steps = int(next((a for a in args[args.index("--train") + 1:] if a.isdigit()), "20000"))
    model = PPO("MultiInputPolicy", env, n_steps=256, batch_size=64, verbose=0, seed=0)
    model.learn(total_timesteps=steps)
    print(f"trained {steps} steps (the actor sees the privileged channel too; mask it for a pictures-only policy)")
