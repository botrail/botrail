"""A botrail cell in Isaac Lab: export the cell as a *simulation stage*
(`scene.export_usd(path, physics=...)`), then open it in Isaac Lab, clone
it into a grid of environments and step it — the robot an articulation
driven by joint targets, the loose parts rigid bodies, the bench and the
stand static colliders, the conveyor a belt that carries what is on it.

Two halves, two Python environments:

    # 1. botrail's environment: write the stage
    python examples/export/isaaclab_cell.py export [cell.usda]

    # 2. Isaac Lab's environment (isaaclab 2.x on Isaac Sim 5): run it
    python examples/export/isaaclab_cell.py run [cell.usda] --headless

The stage is a static snapshot: no timeSamples, because an animation and
a simulation would fight over the same prims. What Isaac Lab addresses:

    {ENV}/Cell/Robot            the arm (ArticulationCfg, spawn=None)
    {ENV}/Cell/Env/cartons/c3   a dynamic unit (RigidObjectCfg, spawn=None)
    {ENV}/Cell/Env/line/bed     the belt: a kinematic body with
                                PhysxSurfaceVelocityAPI

`scene.physics_plan(physics)` is the table of what became what.

The run defaults to PhysX's CPU pipeline, because of the belt: a surface
velocity is a contact modification, and under GPU dynamics PhysX lets a
part fall straight through a running belt (measured on Isaac Sim 5.1;
Isaac Sim's own conveyor test pins CPU dynamics too). With `--device
cuda:0` this script switches the belt off first — it then holds its
carton like any kinematic body, and the rest of the cell runs the same.
For the same reason the scene is built with `replicate_physics=False`:
the physics replicator clones the bodies but not the belt's velocity.

Isaac Lab validates `init_state.joint_pos` (all zeros unless you say
otherwise) against the joint limits; a robot with a joint whose range
excludes zero (the Franka's fourth) needs its defaults stated.

The checks print before the app shuts down; on some installs Isaac Sim's
`app.close()` takes its time, and interrupting it then is harmless.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent

BELT_TOP = 0.70
BELT_SPEED = 0.25
LINE_Y = -1.6
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]  # the pose the arm is exported in


def export(out: Path) -> None:
    import botrail as bt

    sys.path.insert(0, str(HERE.parent / "basics"))
    from physics_world_demo import build_cell  # the bench, arm, tray, pallet cell

    scene = build_cell()
    scene.set_joint_positions(READY)
    # A conveyor beside the cell, authored as every botrail conveyor is —
    # a zone box and a velocity — with one carton riding it.
    scene.add_box("line/bed", size=(2.6, 0.4, 0.08), position=(0.0, LINE_Y, BELT_TOP - 0.04), color=(0.2, 0.2, 0.22))
    for k, x in enumerate((-1.1, 1.1)):
        scene.add_box(f"line/leg{k}", size=(0.06, 0.34, 0.62), position=(x, LINE_Y, 0.31))
    scene.set_part("line", category="conveyor", model="belt conveyor 2600 (shape example)")
    scene.add_conveyor(
        "conv",
        zone_position=(0.0, LINE_Y, BELT_TOP + 0.135),
        zone_size=(2.6, 0.4, 0.27),
        velocity=(BELT_SPEED, 0.0, 0.0),
    )
    bt.parts.carton(scene, "riding", (0.2, 0.2, 0.15), (-1.0, LINE_Y, BELT_TOP + 0.005), mass_kg=1.0)

    physics = bt.Physics(world=True)
    print(scene.physics_plan(physics).to_markdown())
    for warning in scene.export_usd(out, physics=physics):
        print("warning:", warning)
    print(f"wrote {out}")


def run(usd: Path, argv: list[str]) -> None:
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser()
    parser.add_argument("--num_envs", type=int, default=4)
    parser.add_argument("--seconds", type=float, default=3.0)
    AppLauncher.add_app_launcher_args(parser)
    parser.set_defaults(device="cpu")  # the belt needs CPU dynamics, see above
    args = parser.parse_args(argv)
    app = AppLauncher(args).app

    import torch
    import isaaclab.sim as sim_utils
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
    from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
    from isaaclab.utils import configclass

    @configclass
    class CellSceneCfg(InteractiveSceneCfg):
        light = AssetBaseCfg(prim_path="/World/light", spawn=sim_utils.DomeLightCfg(intensity=2000.0))
        # The whole cell, one reference per environment. The stage brings
        # its own ground plane, colliders and bodies.
        cell = AssetBaseCfg(prim_path="{ENV_REGEX_NS}/Cell", spawn=sim_utils.UsdFileCfg(usd_path=str(usd)))
        # Residents of the cell, addressed where the stage put them.
        robot = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Cell/Robot",
            spawn=None,
            actuators={"arm": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=2000.0, damping=200.0)},
        )
        hovering = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/cartons/c3", spawn=None)
        orphan = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/orphan", spawn=None)
        part = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/part", spawn=None)
        riding = RigidObjectCfg(prim_path="{ENV_REGEX_NS}/Cell/Env/riding", spawn=None)

    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device=args.device))
    scene = InteractiveScene(CellSceneCfg(num_envs=args.num_envs, env_spacing=6.0, replicate_physics=False))
    belt_runs = str(args.device).startswith("cpu")
    if not belt_runs:
        for prim in sim_utils.find_matching_prims("/World/envs/env_.*/Cell/Env/line/bed"):
            prim.GetAttribute("physxSurfaceVelocity:surfaceVelocityEnabled").Set(False)
        print("GPU dynamics: the belt's surface velocity is switched off")
    sim.reset()

    robot = scene["robot"]
    print("joints:", robot.joint_names)
    print("bodies:", robot.body_names)
    start = {name: scene[name].data.root_pos_w.clone() - scene.env_origins for name in ("hovering", "orphan", "part", "riding")}
    q0 = robot.data.joint_pos.clone()
    target = q0.clone()
    target[:, 0] += 0.5  # swing the first axis half a radian
    target[:, 2] -= 0.3

    dt = sim.get_physics_dt()
    for _ in range(int(args.seconds / dt)):
        robot.set_joint_position_target(target)
        scene.write_data_to_sim()
        sim.step(render=False)
        scene.update(dt)

    end = {name: scene[name].data.root_pos_w - scene.env_origins for name in start}
    moved = {name: (end[name] - start[name])[0].tolist() for name in start}
    error = (robot.data.joint_pos - target).abs().max().item()
    print(f"joint target error after {args.seconds:.1f} s: {error:.4f} rad (moved {(robot.data.joint_pos - q0)[0].tolist()})")
    for name, d in moved.items():
        print(f"  {name:<9} moved ({d[0]:+.3f}, {d[1]:+.3f}, {d[2]:+.3f}) m")
    spread = max((end[name] - end[name][0]).abs().max().item() for name in end)
    print(f"  environments agree to {spread:.4f} m")

    posed = (q0[0] - torch.tensor(READY, device=q0.device)).abs().max().item()
    checks = {
        "the arm starts in the pose it was exported in": posed < 0.01,
        "the arm follows its joint targets": error < 0.02,
        "the hovering carton dropped onto the stack": -0.17 < moved["hovering"][2] < -0.12,
        "the orphan box fell to the ground": moved["orphan"][2] < -0.8,
        # Seated 5 mm above a foam insert that does not collide: it
        # settles onto the tray itself, 23 mm down — botrail's own bake
        # says 26.
        "the part settled onto its tray on the bench": -0.035 < moved["part"][2] < -0.015,
        "the belt carried the carton" if belt_runs else "the stopped belt held the carton": (
            moved["riding"][0] > 0.8 * BELT_SPEED * args.seconds
            if belt_runs
            else abs(moved["riding"][0]) < 0.01
        )
        and abs(moved["riding"][2]) < 0.02,
        "cloned environments behave alike": spread < 0.05,
    }
    for what, ok in checks.items():
        print(("PASS  " if ok else "FAIL  ") + what, flush=True)
    app.close()
    if not all(checks.values()):
        raise SystemExit(1)


if __name__ == "__main__":
    mode = sys.argv[1] if len(sys.argv) > 1 else "export"
    rest = sys.argv[2:]
    path = Path(rest.pop(0)) if rest and not rest[0].startswith("-") else Path("isaaclab_cell.usda")
    if mode == "export":
        export(path.resolve())
    elif mode == "run":
        run(path.resolve(), rest)
    else:
        raise SystemExit(__doc__)
