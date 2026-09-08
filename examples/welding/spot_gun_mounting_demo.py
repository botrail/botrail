"""Move a Kawasaki BX250L with the configured NIMAK 95.020.516/P3U gun.

    uv run python examples/welding/spot_gun_mounting_demo.py --studio

Prebuilt reference models require no local catalog build. The arm moves through
three inspection poses; the gun remains closed because its opening range is not
established. This example checks mounting and motion, not a welding process.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import botrail as bt

ARM = "kawasaki/bx/bx250l-b001/r2"
GUN = "nimak/multiframegun/95-020-516-p3u/r3"
MOTION = "mounting_inspection"


def build_cell(catalog_root: Path | None = None) -> bt.Scene:
    def load(pid: str) -> bt.Robot:
        if catalog_root is not None:
            return bt.Robot.from_package(catalog_root / pid)
        return bt.Robot.from_catalog(pid)

    arm, gun = load(ARM), load(GUN)
    scene = bt.Scene(arm)
    bt.mounting.preview(scene, arm.attach_tool(gun, prefix="gun_")).apply()
    for pose in (
        [0.2, -0.15, 0.1, 0.1, -0.15, 0.1],
        [-0.2, 0.1, -0.1, -0.1, 0.15, -0.1],
        [0.0] * 6,
    ):
        scene.add_segment(MOTION, pose)
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--studio", action="store_true")
    parser.add_argument(
        "--output", type=Path, help="Save the assembly and inspection motion"
    )
    parser.add_argument(
        "--catalog-root", type=Path, help="Local packages for catalog development"
    )
    args = parser.parse_args()
    scene = build_cell(args.catalog_root)
    report = bt.mounting.report(scene)
    trajectory = scene.plan_motion(MOTION, seed=7, broadcast=False)
    print("Mounting can be used in simulation:", report.simulation["ready"])
    print("Detailed mounting checks complete:", report.ready)
    print(
        f"Arm: {scene.robot.dof} axes; inspection motion: {trajectory.duration:.2f} s"
    )
    print(
        "Gun stays closed. Shape and collision envelopes are approximate; TCP is a CAD reference."
    )
    print(
        "Separate mounting hardware: MISUMI CB10-25 x 10 (already shown in the model)."
    )
    if args.output:
        scene.save_project(args.output)
        print("Saved:", args.output)
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
