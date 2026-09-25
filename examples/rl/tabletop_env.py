"""A tabletop cell, and a grid of them — the Isaac Lab picture as a
botrail cell (design-rl-tabletop.md T0).

One cell is a Franka FR3 with its hand from the catalog on a steel stand,
a wooden work table beside it, a KLT-style bin, a parts tray and a pool
of eight household things — the YCB objects (`ycb/objects/*` in the
catalog: the scanned, textured models at their measured masses, placed
with `bt.parts.prop`; `--shapes` draws primitives of the same sizes
instead and downloads nothing). It is written once, as
`cell(scene, prefix, origin)`, and that one function is

- the learning environment: `rl.make(rl.single(cell), task, num_envs=N)`
  steps N independent copies on N threads;
- the picture: `rl.tile(cell, 2, 3)` puts six copies in one scene and
  `rl.play` runs the same policy in every copy at once, into one
  timeline the studio plays and USD carries.

The arm is a dynamic body by default (`scene.set_robot_physics`): links
weigh what the catalog states, joints are force-capped servos, and a hand
pressed into the table stops on it instead of pushing the part through
(`--kinematic` for the mirror). Every episode draws which things are on
the table (`rl.randomize.pool`) and where, kept apart by their real
footprints (`rl.randomize.scatter`), their friction, and lets them settle
(`Task.settle_s`) before the first observation. A scripted
controller brings the hand over the can, above everything on the table
— the hover pose solved by IK and stepped toward in joint space
(`rl.JointDelta`); `--torque` swaps the position control for raw joint
torques (`rl.Torque`) and a PD hold; `--train` fits PPO with
stable-baselines3.

    python examples/rl/tabletop_env.py [out.usda] [--studio] [--kinematic] [--torque] [--tile ROWSxCOLS]
        [--shapes] [--catalog-root DIR] [--train [steps]]

`--catalog-root` (or `BOTRAIL_CATALOG_ROOT`) reads the YCB packages and the KLT from a local catalog build
(botrail-catalog-builder's `build/`) instead of the published catalog.
"""

from __future__ import annotations

import functools
import os
import sys
from pathlib import Path

import botrail as bt
import numpy as np
from botrail import rl

TOP = 0.75                      # both tables' top faces
SEAT = 0.002                    # the base sits this far above the stand: seated on the face itself, the base link and the top touch at zero distance and every scan flags a collision
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04]
HOVER_Z = TOP + 0.25            # the goal: the TCP over the can at this height — above everything on the table (the cracker box stands 0.21 tall), so the open hand clears the neighbours the pool draw put around the can
APPROACH = 0.10                 # the scripted controller travels this much higher until it is over the can, then descends
WOOD = (0.62, 0.45, 0.28)
STEEL = (0.72, 0.73, 0.75)
# The bin: the VDA 4500 small-load container spec pack (R-KLT 3215 here).
KLT = "botrail/bin/klt-vda4500"
TRAY_LILAC = (0.70, 0.55, 0.75)

# The pool: the YCB objects by the names the task and the randomiser use.
YCB = {
    "can": "ycb/objects/005-tomato-soup-can",
    "mustard": "ycb/objects/006-mustard-bottle",
    "banana": "ycb/objects/011-banana",
    "apple": "ycb/objects/013-apple",
    "mug": "ycb/objects/025-mug",
    "cracker": "ycb/objects/003-cracker-box",
    "block": "ycb/objects/036-wood-block",
    "sugar": "ycb/objects/004-sugar-box",
}
# `--shapes`: primitives of the YCB objects' sizes and masses —
# name -> (shape, size, mass kg, colour).
SHAPES = {
    "can": ("cyl", (0.034, 0.10), 0.35, (0.85, 0.20, 0.20)),
    "mustard": ("box", (0.097, 0.067, 0.19), 0.60, (0.90, 0.75, 0.10)),
    "banana": ("box", (0.04, 0.178, 0.037), 0.07, (0.95, 0.85, 0.20)),
    "apple": ("sph", (0.037,), 0.07, (0.80, 0.10, 0.10)),
    "mug": ("cyl", (0.045, 0.08), 0.12, (0.80, 0.15, 0.15)),
    "cracker": ("box", (0.072, 0.164, 0.213), 0.41, (0.80, 0.20, 0.15)),
    "block": ("box", (0.10, 0.10, 0.20), 0.73, (0.75, 0.60, 0.40)),
    "sugar": ("box", (0.049, 0.094, 0.176), 0.51, (0.90, 0.90, 0.85)),
}
NAMES = list(YCB)
TARGET = "can"
OTHERS = [name for name in NAMES if name != TARGET]
# Where the cell is built with every thing on the table, cell-relative
# footprint centres: two rows the objects' real sizes fit without touching
# (the physics plan's overlap audit stays empty — two things born inside
# each other are kicked apart by the engine at t = 0).
LAYOUT = {
    "mustard": (0.30, -0.26), "sugar": (0.30, -0.13), "can": (0.30, 0.0),
    "apple": (0.30, 0.12), "banana": (0.30, 0.26),
    "cracker": (0.46, -0.22), "mug": (0.46, 0.0), "block": (0.46, 0.20),
}
# Where an episode scatters the things it draws: half a metre and more
# from the arm's base (a top-down approach to a thing at the base's feet
# folds the arm through odd postures), within its reach, short of the bin
# and the tray.
REGION = ((0.24, 0.52), (-0.28, 0.28))


@functools.cache
def robot() -> bt.Robot:
    """FR3 + Franka Hand from the catalog, fetched once — a `build` per
    world would fetch it again for every world."""
    arm = bt.Robot.from_catalog("franka/fr/fr3")
    hand = bt.Robot.from_catalog("franka/hand/franka-hand")
    return arm.attach_tool(hand)


def rest_z(name: str) -> float:
    """The z a `--shapes` stand-in's centre stands at on the table."""
    kind, size, _, _ = SHAPES[name]
    half = size[2] / 2 if kind == "box" else (size[1] / 2 if kind == "cyl" else size[0])
    return TOP + half + SEAT


def robot_name(prefix: str) -> str:
    return prefix.replace("/", "_") + "fr3"


def catalog_ref(product: str, root) -> str:
    """A catalog id — or, with a local build root, the package directory of
    its newest revision built there."""
    if not root:
        return product
    path = Path(root) / product
    revisions = sorted((p for p in path.glob("r*") if p.name[1:].isdigit()), key=lambda p: int(p.name[1:]))
    if not revisions:
        raise FileNotFoundError(f"{product} is not built under {root}")
    return str(revisions[-1])


def make_cell(*, dynamic: bool = True, objects: str = "ycb", catalog_root=None):
    """The cell as `rl.single` / `rl.tile` want it: `cell(scene, prefix,
    origin) -> robot name`, every resident named `prefix + ...`. `objects`
    is `"ycb"` (the catalog's scanned objects, from `catalog_root` when
    given) or `"shapes"` (primitives of their sizes)."""
    if objects not in ("ycb", "shapes"):
        raise ValueError(f"objects must be 'ycb' or 'shapes', got {objects!r}")

    def cell(scene: bt.Scene, prefix: str, origin: tuple[float, float]) -> str:
        ox, oy = origin
        name = robot_name(prefix)
        bt.parts.table(scene, prefix + "stand", size=(0.6, 0.5, TOP), position=(ox - 0.2, oy), color=STEEL, top_thickness=0.03)
        bt.parts.table(scene, prefix + "table", size=(0.9, 0.7, TOP), position=(ox + 0.55, oy), color=WOOD, top_thickness=0.03)
        scene.add_robot(robot(), name=name, base_position=(ox - 0.2, oy, TOP + SEAT))
        scene.set_joint_positions(READY, robot=name)
        if dynamic:
            scene.set_robot_physics(name)
        scene.set_gripper_drive(max_force=70.0, robot=name)
        # The board is timber and the stand's top is tread plate: finishes
        # the studio draws over the colour (a rollout's pictures keep the
        # flat colour).
        scene.set_obstacle_material(prefix + "table/top", finish="wood")
        scene.set_obstacle_material(prefix + "stand/top", finish="checker_plate")
        # An R-KLT 3215 (300 × 200 × 147): five boxes pinned as one part, so
        # the world scope keeps it one rigid unit. With the real objects it
        # is the catalog's container (the type number, mass and inside
        # dimensions of VDA 4500); the primitive cell draws the same size
        # without ordering it.
        if objects == "ycb":
            bt.parts.bin(scene, prefix + "bin", size=(0.30, 0.20, 0.147), position=(ox + 0.75, oy + 0.18, TOP),
                         catalog=catalog_ref(KLT, catalog_root))
        else:
            bt.parts.bin(scene, prefix + "bin", size=(0.30, 0.20, 0.147), position=(ox + 0.75, oy + 0.18, TOP),
                         model="R-KLT 3215 (stand-in)")
        bt.parts.tray(scene, prefix + "tray", size=(0.30, 0.22, 0.04), position=(ox + 0.75, oy - 0.2, TOP), color=TRAY_LILAC)
        for n in NAMES:
            x, y = ox + LAYOUT[n][0], oy + LAYOUT[n][1]
            if objects == "ycb":
                bt.parts.prop(scene, prefix + n, (x, y, TOP + SEAT), catalog=catalog_ref(YCB[n], catalog_root), friction=0.6)
                continue
            kind, size, mass, color = SHAPES[n]
            if kind == "box":
                scene.add_box(prefix + n, size=size, position=(x, y, rest_z(n)), color=color)
            elif kind == "cyl":
                scene.add_cylinder(prefix + n, radius=size[0], length=size[1], position=(x, y, rest_z(n)), color=color)
            else:
                scene.add_sphere(prefix + n, radius=size[0], position=(x, y, rest_z(n)), color=color)
            scene.set_physics(prefix + n, dynamic=True, mass=mass, friction=0.6)
        # A wrist camera looking down the hand, for the picture channels.
        scene.add_camera(prefix + "wrist", position=(0.05, 0.0, 0.0), quaternion=(1.0, 0.0, 0.0, 0.0), fov=70.0,
                         resolution=(64, 64), near=0.05, far=2.0, robot=name, link="fr3_hand")
        return name

    return cell


def randomize(scene: bt.Scene, rng: np.random.Generator, prefix: str = "", origin: tuple[float, float] = (0.0, 0.0)) -> None:
    """The episode's draw: four of the seven other things join the can (the
    target) on the table, the rest parked out of the world; the five are
    scattered over the reachable part of the table with their footprints
    2 cm apart; every thing's friction drawn anew."""
    ox, oy = origin
    drawn = rl.randomize.pool(scene, rng, OTHERS, 4, prefix=prefix, park=(ox + 3.0, oy + 3.0, 0.5))
    (x0, x1), (y0, y1) = REGION
    rl.randomize.scatter(scene, rng, [TARGET, *drawn], ((ox + x0, ox + x1), (oy + y0, oy + y1)), TOP,
                         prefix=prefix, gap=0.02, lift=SEAT)
    rl.randomize.friction(scene, rng, NAMES, 0.4, 0.9, prefix=prefix)


# The observation, in channel order: joints (16 = 8 q + 8 q̇), TCP pose
# (7: position, quaternion xyzw), the can's pose (7), the finger–can
# contact (2), the collision flag (1). The scripted controllers read the
# joints, the TCP and the can by these slices.
Q, TCP, TCP_QUAT, CAN = slice(0, 8), slice(16, 19), slice(19, 23), slice(23, 26)
MAX_STEP = 0.1                  # rad per control step of the joint-delta action


def hover_point(can_xyz: np.ndarray) -> np.ndarray:
    """The goal for a can at `can_xyz`: straight above it at `HOVER_Z`."""
    return np.array([can_xyz[0], can_xyz[1], HOVER_Z])


def hover_error(flat: np.ndarray) -> np.ndarray:
    """World-frame vector from the TCP to the hover point above the can."""
    return hover_point(flat[CAN]) - flat[TCP]


def hover_reward(obs, info) -> float:
    return -float(np.linalg.norm(hover_point(obs["can/pose"][:3]) - obs["fr3/tcp"][:3]))


def hover_done(obs, info) -> bool:
    return bool(np.linalg.norm(hover_point(obs["can/pose"][:3]) - obs["fr3/tcp"][:3]) < 0.02)


def make_task(control: str = "joint", *, horizon_s: float = 4.0) -> rl.Task:
    """The task under one of three controls: `"joint"` (a step per joint,
    what the scripted controller drives), `"tcp"` (a Cartesian step of the
    TCP plus the hand) or `"torque"` (raw joint torques)."""
    if control == "torque":
        ctrl = rl.Torque(hz=20)
    elif control == "tcp":
        ctrl = rl.TcpDelta(max_step_m=0.02, hz=20, gripper=["fr3_finger_joint1"])
    else:
        ctrl = rl.JointDelta(max_step=MAX_STEP, hz=20)
    return rl.Task(
        control=ctrl,
        observe=[rl.Joints(), rl.TcpPose(), rl.ObjectPose(TARGET), rl.Contacts("fr3/fr3_leftfinger", TARGET), rl.Collision()],
        reset=lambda scene, rng: randomize(scene, rng, "", (0.0, 0.0)),
        reward=hover_reward,
        done=hover_done,
        horizon_s=horizon_s,
        settle_s=0.3,
    )


@functools.cache
def _probe() -> bt.Scene:
    """The arm alone at the origin: the scripted controller's forward
    kinematics and IK table, in the robot's own frame."""
    return bt.Scene(robot())


STRIDE = 0.03                   # metres the scripted controller asks of the TCP per step


def step_toward(flat: np.ndarray, target: np.ndarray, *, hold_orientation: bool = True) -> np.ndarray:
    """A joint-delta action moving the TCP up to `STRIDE` toward the
    world point `target` with the hand's orientation held (or free, with
    `hold_orientation=False`): the next waypoint is solved by IK from
    where the arm stands, so the path is straight in the world and the
    seven-axis arm's elbow cannot drift far from step to step. The
    world-frame error is the robot-frame error too (the cell mounts the
    arm without turning it), so the waypoint is the robot-frame TCP plus
    the clamped error — a controller that works in any copy of a tiled
    cell. A step the IK cannot solve holds."""
    q = [float(v) for v in flat[Q]]
    error = np.asarray(target, dtype=float) - flat[TCP]
    distance = float(np.linalg.norm(error))
    if distance < 1e-6:
        return np.zeros(8)
    probe = _probe()
    (tcp_r, _) = probe.link_pose_at(probe.robot.tcp_link, q)
    quat = tuple(float(v) for v in flat[TCP_QUAT]) if hold_orientation else None
    # A few centimetres should not turn any joint far: a solution that
    # does has jumped to another branch of the redundant arm — try a
    # shorter stride, else hold.
    for stride in (STRIDE, STRIDE / 3):
        step = error * min(1.0, stride / distance)
        waypoint = tuple(float(v) for v in np.asarray(tcp_r) + step)
        sol = probe.robot.ik(waypoint, quaternion=quat, seed=q, restarts=0)
        if sol.converged and np.abs(np.asarray(sol.q) - flat[Q]).max() < 0.5:
            action = np.clip((np.asarray(sol.q) - flat[Q]) / MAX_STEP, -1.0, 1.0)
            action[7] = 0.0
            return action
    return np.zeros(8)


_pose_cache: dict[tuple[float, float, float], np.ndarray] = {}


def toward_pose(flat: np.ndarray, target: np.ndarray) -> np.ndarray:
    """A joint-delta action toward the joint pose that puts the TCP at
    the world point `target` (orientation held), solved once per target
    from wherever the arm first aimed at it and kept — a monotonic
    joint-space approach for the long travel, where solving afresh every
    step lets the seven-axis arm wander between elbow branches."""
    key = tuple(float(v) for v in np.round(target, 3))
    if key not in _pose_cache:
        q = [float(v) for v in flat[Q]]
        probe = _probe()
        (tcp_r, _) = probe.link_pose_at(probe.robot.tcp_link, q)
        point = tuple(float(v) for v in np.asarray(tcp_r) + (np.asarray(target, dtype=float) - flat[TCP]))
        quat = tuple(float(v) for v in flat[TCP_QUAT])
        sol = probe.robot.ik(point, quaternion=quat, seed=q, restarts=0)
        if not sol.converged:
            return step_toward(flat, target)
        _pose_cache[key] = np.asarray(sol.q)
    action = np.clip((_pose_cache[key] - flat[Q]) / MAX_STEP, -1.0, 1.0)
    action[7] = 0.0
    return action


def scripted(flat: np.ndarray) -> np.ndarray:
    """Brings the hand over the can: travels to a point `APPROACH` above
    the hover point in one joint-space move, then descends onto the hover
    point in short straight steps (a joint-space path down would swing the
    hand through the can). A proportional step of the TCP through
    `rl.TcpDelta` would wind that control's own setpoint up past the arm's
    lag; these controllers work from the TCP they observe. It reaches the
    hover point in most draws; a can at the far corners of the table, where
    the joint-space travel arcs low over the neighbours, is where a learned
    policy earns its keep."""
    goal = hover_point(flat[CAN])
    if np.linalg.norm(goal[:2] - flat[TCP][:2]) > 0.04:
        return toward_pose(flat, goal + [0.0, 0.0, APPROACH])
    return step_toward(flat, goal)


def scripted_torque(flat: np.ndarray) -> np.ndarray:
    """A PD hold at READY in torque space (gravity is the policy's to
    carry): the arm stays put while the pool draw settles around it."""
    q, qd = flat[Q], flat[8:16]
    return np.clip((-40.0 * (q - READY) - 4.0 * qd) / 50.0, -1.0, 1.0)


def main() -> None:
    args = sys.argv[1:]
    torque = "--torque" in args
    root = args[args.index("--catalog-root") + 1] if "--catalog-root" in args else os.environ.get("BOTRAIL_CATALOG_ROOT")
    objects = "shapes" if "--shapes" in args else "ycb"
    cell = make_cell(dynamic="--kinematic" not in args, objects=objects, catalog_root=root)
    task = make_task("torque" if torque else "joint")
    model = scripted_torque if torque else scripted
    try:
        env = rl.make(rl.single(cell), task, seed=0)
    except Exception as error:
        if objects == "ycb" and ("ycb/objects" in str(error) or KLT in str(error)):
            sys.exit(f"{error}\nThe YCB objects or the KLT ({KLT}) are not in the catalog you reach — pass "
                     "--catalog-root <botrail-catalog-builder>/build (or set BOTRAIL_CATALOG_ROOT) for a local "
                     "build, or --shapes.")
        raise
    print(f"observation {env.observation_space.shape}, action {env.action_space.shape}, "
          f"{'dynamic' if '--kinematic' not in args else 'kinematic'} arm, {'torque' if torque else 'position'} control, "
          f"{'YCB objects' if objects == 'ycb' else 'primitive stand-ins'}")

    def play(policy, episodes: int, label: str) -> None:
        for episode in range(episodes):
            obs, info = env.reset()
            total, steps = 0.0, 0
            while True:
                obs, reward, terminated, truncated, info = env.step(policy(obs))
                total += reward
                steps += 1
                if terminated or truncated:
                    break
            if torque:
                # The torque controller only holds READY against gravity.
                outcome = f"held READY within {np.abs(obs[Q] - READY).max():.3f} rad"
            else:
                outcome = "hovering" if terminated and "error" not in info else "ran out"
            print(f"  {label} episode {episode}: {outcome} after {steps} steps ({info['t']:.2f}s), return {total:+.2f}")

    print("scripted controller:")
    play(model, episodes=3, label="scripted")

    if "--train" in args:
        try:
            from stable_baselines3 import PPO
        except ImportError:
            sys.exit("--train needs stable-baselines3: pip install stable-baselines3")
        ppo = PPO("MlpPolicy", env, n_steps=256, batch_size=64, verbose=0, seed=0)
        ppo.learn(total_timesteps=int(next((a for a in args if a.isdigit()), "20000")))
        print("learned policy:")
        play(lambda obs: ppo.predict(obs, deterministic=True)[0], episodes=3, label="ppo")
        def model(obs):
            return ppo.predict(obs, deterministic=True)[0]

    timeline = env.timeline(publish=True)
    print(f"last episode: {timeline.duration:.2f}s, min clearance {float(timeline.min_clearance()):.3f} m, "
          f"{len(timeline.contacts)} contact episodes")
    grid = args[args.index("--tile") + 1] if "--tile" in args else None
    positional = [a for a in args if not a.startswith("--") and not a.isdigit() and a not in (grid, root)]
    out = positional[0] if positional else "tabletop_env.usda"
    warnings = timeline.export_usd(out, fps=30.0)
    print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

    tiled = None
    if "--tile" in args:
        rows, cols = (int(v) for v in grid.lower().split("x"))
        tiled = rl.tile(cell, rows, cols, spacing=(1.9, 1.6))
        tiled.randomize(randomize, seed=0)
        played = rl.play(tiled, task, model, duration_s=4.0)
        tiled_out = str(Path(out).with_name(Path(out).stem + "_tiled" + Path(out).suffix))
        played.export_usd(tiled_out, fps=30.0)
        print(f"{rows}×{cols} cells played for {played.duration:.2f}s: wrote {tiled_out}")

    if "--studio" in args:
        if tiled is not None:
            tiled.scene.show_timeline(played)
            bt.studio(tiled.scene)
        else:
            bt.studio(env.scene)


if __name__ == "__main__":
    main()
