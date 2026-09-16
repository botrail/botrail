"""World physics demo (design-world-physics.md W0/W1): the whole cell under
gravity, with nothing marked and nothing driven.

Every other physics demo marks the bodies it wants the engine to own
(`scene.set_physics(name, dynamic=True)`, `scene.set_robot_physics()`).
This one marks nothing. The cell is built the way any demo is — a bench
and a stand from `bt.parts`, an arm on the stand, a tray on the bench, a
workpiece seated in the tray, a pallet on the floor with cartons stacked
on it — and then handed to the engine whole:

    scene.simulate_physics(4.0)

`simulate_physics` bakes a stretch of time with no program at all. Under
the *world* scope (`bt.Physics(world=True)`, the default here) every
obstacle folds into a rigid unit by its name hierarchy and part identity,
and a unit stays put only when authoring or identity says it is bolted
down — an equipment part pin (`structure.table`, `structure.pedestal`),
a device that moves it by name, a walkable floor, an explicit
`set_physics(dynamic=False)`. Everything else is the engine's, a pallet
on the floor included: it rests because the ground holds it, not because
a rule froze it. There is a ground plane at z = 0, so a cell authored
without a floor slab still has a floor.

Every robot is an articulated body too. With no program running nothing
powers it, so the arm folds at its joints under gravity and comes to rest
on its stand (`--powered` switches the servos on instead: the same world,
the arm holding its pose). A walking machine floats — its base is a free
body — and collapses to the floor (`--scene wash` shows the humanoid).

`scene.physics_plan()` says what the bake will do before it runs — the
table printed first. The bake then shows what the cell's authoring
implied: the workpiece seated 5 mm above its tray drops those 5 mm (and
through the foam insert, which is out of collision), the carton left
hovering above the stack lands on it, and a box someone authored in
mid-air — the mistake this bake exists to find — falls to the floor. The
bench, the stand, the pallet and the tray do not move.

Run with:  python examples/basics/physics_world_demo.py [out.usda] [--studio]
                 [--powered] [--scene demo|warehouse|wash]

`--scene` bakes one of the larger example cells instead (they need the
catalog: `pip install botrail[catalog]`), reporting the plan's counts and
the bake time — the world scope on a real cell.
"""

from __future__ import annotations

import argparse
import importlib
import sys
import time
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent

CARTON = (0.35, 0.30, 0.22)
PART = (0.06, 0.06, 0.04)
SEAT_LIFT = 0.005  # the "float a placed part 5 mm" authoring convention
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]


def build_cell() -> bt.Scene:
    # The arm: a plain six-axis URDF (no catalog needed), standing on the
    # pedestal in a ready pose. Under the world scope it is an articulated
    # body like everything else; nothing here declares it dynamic.
    scene = bt.Scene(bt.Robot.from_urdf(HERE.parent / "assets" / "simple_arm.urdf"))
    scene.set_joint_positions(READY)

    # Equipment: a bench and a robot stand from the parts library. Both are
    # pinned as structure — bolted down, so the world scope keeps them fixed.
    bench = bt.parts.table(scene, "bench", size=(1.2, 0.8, 0.72), position=(0.0, 0.0))
    top = max(scene.obstacle_bounds(name)[1][2] for name in bench.obstacles)
    bt.parts.pedestal(scene, "stand", height=0.5, position=(-1.4, 0.0))
    scene.set_robot_base_pose(*scene.frame("stand/mount"))

    # A tray on the bench (a carrier: loose, it simply rests) with a
    # workpiece seated 5 mm above its foam insert, the way placed parts are
    # authored so the planner never starts in contact.
    bt.parts.tray(scene, "tray", (0.4, 0.3, 0.06), (0.0, 0.0, top))
    seat = scene.frame("tray/seat")[0][2]
    scene.add_box("part", PART, (0.0, 0.0, seat + PART[2] / 2 + SEAT_LIFT), color=(0.78, 0.80, 0.83))
    scene.set_part("part", category="workpiece", model="加工ワーク 60x60x40 (形状例)", mass_kg=0.39)

    # A pallet on the floor with three cartons stacked on it, and a fourth
    # left hovering a hand above the stack.
    pallet = bt.parts.pallet(scene, "pallet", (1.6, 0.0))
    pallet_top = max(scene.obstacle_bounds(name)[1][2] for name in pallet.obstacles)
    for k in range(3):
        bt.parts.carton(scene, f"cartons/c{k}", CARTON, (1.6 + 0.01 * k, 0.0, pallet_top + CARTON[2] * k), mass_kg=4.0)
    bt.parts.carton(scene, "cartons/c3", CARTON, (1.6, 0.0, pallet_top + CARTON[2] * 3 + 0.15), mass_kg=4.0)

    # The mistake: a box authored in mid-air beside the stand.
    scene.add_box("orphan", (0.1, 0.1, 0.1), (-1.4, 0.6, 0.9), color=(0.85, 0.33, 0.20))
    return scene


def describe(scene: bt.Scene, tl: bt.SequenceTimeline, name: str) -> str:
    """One line on what the bake did to obstacle `name`."""
    try:
        (x, y, z), _ = tl.object_pose(name, tl.duration)
    except ValueError:
        return f"  {name:<16} did not move"
    (x0, y0, z0), _ = scene.obstacle_pose(name)
    settled = tl.settled_at(name)
    when = f"settled at t={settled:.2f} s" if settled is not None else "still moving at the end"
    return f"  {name:<16} moved ({x - x0:+.3f}, {y - y0:+.3f}, {z - z0:+.3f}) m, {when}"


def report(scene: bt.Scene, tl: bt.SequenceTimeline, watched: list[str]) -> None:
    print(f"baked {tl.duration:.1f} s under physics={tl.physics!r} ({tl.physics_scope} scope)")
    for name in watched:
        print(describe(scene, tl, name))
    for robot in tl.robots:
        q0 = tl.sample(0.0, robot=robot)
        q1 = tl.sample(tl.duration, robot=robot)
        moved = max(abs(a - b) for a, b in zip(q0, q1)) if q0 else 0.0
        base = ""
        b0, b1 = tl.base_pose(0.0, robot=robot), tl.base_pose(tl.duration, robot=robot)
        if b0 is not None and b1 is not None:
            base = f", base z {b0[0][2]:+.3f} → {b1[0][2]:+.3f} m"
        print(f"  {robot:<16} joints moved up to {moved:.2f} rad{base}")
    print(f"  {len(tl.contacts)} contact episodes")


def large_scene(which: str) -> bt.Scene:
    sys.path.insert(0, str(HERE.parent))
    if which == "demo":
        return importlib.import_module("basics.demo").build_scene()
    if which == "warehouse":
        return importlib.import_module("vehicles.warehouse_demo").build()[0]
    if which == "wash":
        return importlib.import_module("legged.wash_inspect_ship_demo").build_scene()
    raise SystemExit(f"unknown scene {which!r}")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("out", nargs="?", default="physics_world.usda")
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--scene", choices=["demo", "warehouse", "wash"])
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--powered", action="store_true", help="motors on: the servos hold every robot's pose")
    args = parser.parse_args()
    physics = bt.Physics(world=True, powered=True if args.powered else None)

    if args.scene:
        scene = large_scene(args.scene)
        plan = scene.physics_plan()
        # A unit's track is its frame obstacle's (a pinned group is named
        # by its pin, which is no obstacle).
        frames = [row["members"][0] for row in plan.rows if row["kind"] == "dynamic"]
        print(f"{args.scene}: {len(scene.obstacle_names)} obstacles → {len(plan)} bodies, {len(frames)} dynamic")
        for unit, other in plan.overlaps[:10]:
            print(f"  starts inside something: {unit} × {other}")
        for row in plan.rows:
            if row["kind"] == "robot":
                print(f"  {row['name']}: {row['reason']}")
        t0 = time.perf_counter()
        tl = scene.simulate_physics(args.seconds, physics=physics)
        wall = time.perf_counter() - t0
        print(f"baked {tl.duration:.1f} s in {wall:.2f} s ({tl.duration / max(wall, 1e-9):.1f}× real time)")
        moved = [name for name in frames if _moved(tl, name)]
        print(f"{len(moved)} of {len(frames)} dynamic units moved:")
        for name in moved[:25]:
            print(describe(scene, tl, name))
        report(scene, tl, [])
    else:
        scene = build_cell()
        print(scene.physics_plan(physics=physics).to_markdown())
        tl = scene.simulate_physics(args.seconds, physics=physics)
        report(scene, tl, ["part", "cartons/c3", "cartons/c2", "orphan", "tray", "pallet/top", "bench/top"])

    warnings = tl.export_usd(args.out, fps=30.0)
    print(f"wrote {args.out}" + (f" ({len(warnings)} warnings)" if warnings else ""))
    if args.studio:
        bt.studio(scene)


def _moved(tl: bt.SequenceTimeline, name: str) -> bool:
    try:
        tl.object_pose(name, tl.duration)
    except ValueError:
        return False
    return True


if __name__ == "__main__":
    main()
