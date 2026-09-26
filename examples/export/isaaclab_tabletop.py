"""The tabletop RL cell in Isaac Lab: export the cell of `rl/tabletop_env.py`
as a *simulation stage* (`scene.export_usd(path, physics=...)`), then open
it in Isaac Lab, clone it into a grid of environments and step it on the
GPU — the FR3 an articulation with the servo botrail authored (limits,
drive gains and force caps, the hand's mimic finger, the exported joint
positions), the household things and the KLT rigid bodies, the bench and
the stand static colliders (design-rl-tabletop.md §10.5, T-I).

Two halves, two Python environments:

    # 1. botrail's environment: write the stage (the catalog's YCB scans,
    #    or --shapes for primitives of the same sizes)
    python examples/export/isaaclab_tabletop.py export [tabletop.usda] [--shapes]

    # 2. Isaac Lab's environment (isaaclab 2.x on Isaac Sim 5): run it
    python examples/export/isaaclab_tabletop.py run [tabletop.usda] --headless --num_envs 16
    #    --shot grid.png renders the grid from above (needs --enable_cameras)

What carries over is the cell: the articulation, the bodies, their masses
and friction, the ground. What stays in botrail is the task — observation,
reward, termination, the episode's randomisation (`rl.randomize`) — which
Isaac Lab states in its own `ManagerBasedRLEnvCfg`; and the pictures: the
finishes are the studio's, the scans' textures ride with the catalog, not
the stage. What Isaac Lab addresses, per environment:

    {ENV}/Cell/Robot        the arm (ArticulationCfg, spawn=None)
    {ENV}/Cell/Env/can      a thing on the table (RigidObjectCfg, spawn=None)
    {ENV}/Cell/Env/bin      the KLT: five boxes, one body
    {ENV}/Cell/Env/cam      the wrist camera's bracket: one body on a fixed
                            joint to the hand (`.../cam/weld`)

`scene.physics_plan(physics)` is the table of what became what. No belt
here, so the run keeps Isaac Lab's defaults: GPU dynamics and the physics
replicator (`replicate_physics=True`).

The checks print before the app shuts down; on some installs Isaac Sim's
`app.close()` takes its time, and interrupting it then is harmless.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "rl"))

# The arm's joints and the pose the cell is exported in — tabletop_env.READY
# (seven arm joints and the finger opening), restated here because the
# Isaac Lab half runs where botrail is not installed.
ARM = [f"fr3_joint{i}" for i in range(1, 8)]
FINGERS = ["fr3_finger_joint1", "fr3_finger_joint2"]
READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.04]
OBJECTS = ["can", "mustard", "banana", "apple", "mug", "cracker", "block", "sugar"]
TOP = 0.75  # the table top (tabletop_env.TOP)


def export(out: Path, objects: str) -> None:
    import botrail as bt
    from botrail import rl

    import tabletop_env as demo

    assert list(demo.READY) == READY, "the run half restates tabletop_env.READY"
    scene = rl.single(demo.make_cell(objects=objects))()
    physics = bt.Physics(world=True)
    print(scene.physics_plan(physics).to_markdown())
    for warning in scene.export_usd(out, physics=physics):
        print("warning:", warning)
    print(f"wrote {out}")


def look_at(eye: tuple[float, float, float], target: tuple[float, float, float]) -> tuple[float, float, float, float]:
    """A world-convention camera rotation (forward +X, up +Z) as (w, x, y, z)."""
    f = [t - e for t, e in zip(target, eye)]
    n = math.sqrt(sum(v * v for v in f))
    f = [v / n for v in f]
    up = (0.0, 0.0, 1.0)
    r = [f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0]]
    n = math.sqrt(sum(v * v for v in r))
    r = [v / n for v in r]
    u = [r[1] * f[2] - r[2] * f[1], r[2] * f[0] - r[0] * f[2], r[0] * f[1] - r[1] * f[0]]
    # Columns: x = forward, y = left (-right), z = up.
    m = [[f[0], -r[0], u[0]], [f[1], -r[1], u[1]], [f[2], -r[2], u[2]]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return (0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s)
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2
        return ((m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s)
    if m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2
        return ((m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s)
    s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2
    return ((m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s)


def run(usd: Path, argv: list[str]) -> None:
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs", type=int, default=16)
    parser.add_argument("--seconds", type=float, default=3.0)
    parser.add_argument("--shot", type=Path, default=None, help="render the grid to this PNG")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args(argv)
    if args.shot is not None:
        args.enable_cameras = True
    app = AppLauncher(args).app

    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass

    ready = dict(zip(ARM, READY[:7]))
    ready.update({name: READY[7] for name in FINGERS})

    @configclass
    class TabletopSceneCfg(InteractiveSceneCfg):
        light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2500.0))
        # The whole cell, one reference per environment: ground, stand,
        # table, the arm, the things on the table.
        cell = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Cell", spawn=sim_utils.UsdFileCfg(usd_path=str(usd)))
        # The arm, addressed where the stage put it. Gains left `None`
        # take the DriveAPI botrail authored — the servo's P gain, damping
        # and force cap — and the initial state is the exported pose
        # (Isaac Lab checks it against the limits, and the FR3's fourth
        # joint excludes zero).
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Cell/Robot",
            spawn=None,
            init_state=ArticulationCfg.InitialStateCfg(joint_pos=ready),
            actuators={
                "arm": ImplicitActuatorCfg(joint_names_expr=["fr3_joint.*"], stiffness=None, damping=None),
                "hand": ImplicitActuatorCfg(joint_names_expr=["fr3_finger_joint.*"], stiffness=None, damping=None),
            },
        )
        can = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/can", spawn=None)
        mustard = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/mustard", spawn=None)
        banana = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/banana", spawn=None)
        apple = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/apple", spawn=None)
        mug = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/mug", spawn=None)
        cracker = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/cracker", spawn=None)
        block = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/block", spawn=None)
        sugar = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/sugar", spawn=None)
        bin = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/bin", spawn=None)
        tray = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/tray", spawn=None)
        bracket = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/cam", spawn=None)

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device=args.device))
    scene = InteractiveScene(TabletopSceneCfg(num_envs=args.num_envs, env_spacing=2.5))
    camera = None
    if args.shot is not None:
        from isaaclab.sensors import Camera, CameraCfg

        side = math.ceil(math.sqrt(args.num_envs)) * 2.5
        eye = (-0.9 * side, -0.9 * side, 0.55 * side + 1.5)
        camera = Camera(CameraCfg(
            prim_path="/World/Overview",
            offset=CameraCfg.OffsetCfg(pos=eye, rot=look_at(eye, (0.0, 0.0, 0.6)), convention="world"),
            spawn=sim_utils.PinholeCameraCfg(focal_length=22.0, clipping_range=(0.1, 200.0)),
            width=1600,
            height=1000,
        ))
    sim.reset()

    robot = scene["robot"]
    names = robot.joint_names
    print("joints:", names)
    print("bodies:", len(robot.body_names))
    index = {name: names.index(name) for name in ARM + FINGERS}
    things = OBJECTS + ["bin", "tray"]
    start = {name: scene[name].data.root_pos_w.clone() - scene.env_origins for name in things}
    # Where the bracket stands in the hand's own frame: the swing turns the
    # hand, so a world-frame offset would turn with it.
    from isaaclab.utils.math import quat_apply_inverse

    hand = robot.body_names.index("fr3_hand")
    bracket_offset = lambda: quat_apply_inverse(  # noqa: E731
        robot.data.body_quat_w[:, hand], scene["bracket"].data.root_pos_w - robot.data.body_pos_w[:, hand]
    )
    bracket_start = bracket_offset().clone()
    q0 = robot.data.joint_pos.clone()
    target = q0.clone()
    target[:, index["fr3_joint1"]] += 0.5  # swing the base half a radian
    target[:, index["fr3_joint3"]] -= 0.3
    for name in FINGERS:
        target[:, index[name]] = 0.0  # and close the hand

    dt = sim.get_physics_dt()
    steps = int(args.seconds / dt)
    t0 = time.perf_counter()
    for _ in range(steps):
        robot.set_joint_position_target(target)
        scene.write_data_to_sim()
        sim.step(render=False)
        scene.update(dt)
    rate = steps * args.num_envs / (time.perf_counter() - t0)

    end = {name: scene[name].data.root_pos_w - scene.env_origins for name in things}
    moved = {name: (end[name] - start[name])[0].tolist() for name in things}
    q = robot.data.joint_pos
    arm_error = max((q[:, index[n]] - target[:, index[n]]).abs().max().item() for n in ARM)
    finger = q[:, index["fr3_finger_joint1"]]
    follower = q[:, index["fr3_finger_joint2"]]
    posed = max((q0[:, index[n]] - ready[n]).abs().max().item() for n in ARM + FINGERS)
    print(f"{args.num_envs} environments, {steps} steps of {dt:.4f} s: {rate:,.0f} env-steps/s on {args.device}")
    print(f"joint target error after {args.seconds:.1f} s: {arm_error:.4f} rad; fingers {finger[0].item():.4f} / {follower[0].item():.4f} m")
    for name, d in moved.items():
        print(f"  {name:<8} moved ({d[0]:+.4f}, {d[1]:+.4f}, {d[2]:+.4f}) m, now z = {end[name][0][2].item():.4f}")
    spread = max((end[name] - end[name][0]).abs().max().item() for name in end)
    print(f"  environments agree to {spread:.4f} m")
    bracket_drift = (bracket_offset() - bracket_start).norm(dim=-1).max().item()
    print(f"  the camera bracket moved {bracket_drift:.4f} m relative to the hand through the swing")

    resting = {name: -0.010 < moved[name][2] <= 0.001 and abs(moved[name][0]) < 0.01 and abs(moved[name][1]) < 0.01 for name in OBJECTS}
    checks = {
        "the arm starts in the pose it was exported in": posed < 0.01,
        "the arm follows its joint targets": arm_error < 0.02,
        "the hand closes and the mimic finger follows": finger.max().item() < 0.005 and (finger - follower).abs().max().item() < 0.003,
        "every thing settles onto the table where it was set down": all(resting.values()),
        "the KLT and the tray stay put": all(max(abs(v) for v in moved[name]) < 0.005 for name in ("bin", "tray")),
        "the wrist camera's bracket rides the hand": bracket_drift < 0.002,
        "cloned environments behave alike": spread < 0.01,
    }
    for what, ok in checks.items():
        print(("PASS  " if ok else "FAIL  ") + what, flush=True)
    for name, ok in resting.items():
        if not ok:
            print(f"      {name} moved {moved[name]}", flush=True)

    if camera is not None:
        # RTX needs a few frames before the sensor carries a picture.
        for _ in range(12):
            sim.step(render=True)
            camera.update(dt)
        rgb = camera.data.output["rgb"][0].cpu().numpy()
        from PIL import Image

        Image.fromarray(rgb[..., :3]).save(args.shot)
        print(f"wrote {args.shot}", flush=True)
    app.close()
    if not all(checks.values()):
        raise SystemExit(1)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "export"
    rest = sys.argv[2:]
    path = Path(rest.pop(0)) if rest and not rest[0].startswith("-") else Path("isaaclab_tabletop.usda")
    if mode == "export":
        export(path.resolve(), "shapes" if "--shapes" in rest else "ycb")
    elif mode == "run":
        run(path.resolve(), rest)
    else:
        raise SystemExit(__doc__)
