"""Move a Kawasaki BX250L with the configured NIMAK 95.020.516/P3U gun.

    uv run python examples/welding/spot_gun_mounting_demo.py --studio

Prebuilt reference models require no local catalog build. The arm moves through
inspection poses while the gun's right jaw opens and closes. The 0..20 degree
window and 0.2 rad/s jaw rate are simulation settings, not manufacturer limits.
This example checks mounting and motion, not a welding process.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import botrail as bt

ARM = "kawasaki/bx/bx250l-b001/r2"
GUN = "nimak/multiframegun/95-020-516-p3u/r4"
MOTION = "mounting_inspection"
TIP_RADIUS_MM = 700.0
MAX_OPENING_MM = TIP_RADIUS_MM * math.sin(math.radians(20))


def opening_angle(opening_mm: float) -> float:
    """Projected contact-point separation, not a parallel-jaw workpiece gap."""
    if not math.isfinite(opening_mm) or not 0 <= opening_mm <= MAX_OPENING_MM:
        raise ValueError(f"opening must be between 0 and {MAX_OPENING_MM:.3f} mm")
    return math.asin(opening_mm / TIP_RADIUS_MM)


def build_cell(
    catalog_root: Path | None = None, *, opening_mm: float = 120.0
) -> bt.Scene:
    angle = opening_angle(opening_mm)

    def load(pid: str) -> bt.Robot:
        if catalog_root is not None:
            return bt.Robot.from_package(catalog_root / pid)
        return bt.Robot.from_catalog(pid)

    arm, gun = load(ARM), load(GUN)
    # URDF and USD serialize angular limits at different precision. Honour the
    # loaded limit when a request lands exactly on the 20-degree endpoint.
    jaw_limits = gun.joint_limits[0]
    if jaw_limits is not None:
        angle = min(angle, jaw_limits[1])
    scene = bt.Scene(arm)
    bt.mounting.preview(scene, arm.attach_tool(gun, prefix="gun_")).apply()
    # Resolve names so the same example works with a prefixed tool joint tree.
    jaw_index = next(
        i
        for i, name in enumerate(scene.robot.joint_names)
        if name.endswith("jaw_opening")
    )
    arm_indices = [i for i in range(scene.robot.dof) if i != jaw_index]

    def pose(arm_q: list[float], jaw_q: float) -> list[float]:
        q = [0.0] * scene.robot.dof
        for i, value in zip(arm_indices, arm_q, strict=True):
            q[i] = value
        q[jaw_index] = jaw_q
        return q

    scene.add_segment(MOTION, pose([0.0] * 6, angle))
    for target in (
        pose([0.2, -0.15, 0.1, 0.1, -0.15, 0.1], angle),
        pose([-0.2, 0.1, -0.1, -0.1, 0.15, -0.1], angle),
        pose([0.0] * 6, 0.0),
    ):
        scene.add_segment(MOTION, target)
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--studio", action="store_true")
    parser.add_argument(
        "--opening-mm",
        type=float,
        default=120.0,
        help=f"Projected tip opening, 0..{MAX_OPENING_MM:.3f} mm (default: 120)",
    )
    parser.add_argument(
        "--output", type=Path, help="Save the assembly and inspection motion"
    )
    parser.add_argument(
        "--catalog-root", type=Path, help="Local packages for catalog development"
    )
    args = parser.parse_args()
    try:
        scene = build_cell(args.catalog_root, opening_mm=args.opening_mm)
    except ValueError as exc:
        parser.error(str(exc))
    report = bt.mounting.report(scene)
    trajectory = scene.plan_motion(MOTION, seed=7, broadcast=False)
    print("Mounting can be used in simulation:", report.simulation["ready"])
    print("Detailed mounting checks complete:", report.ready)
    print(f"Arm: 6 axes + gun: 1 axis; inspection motion: {trajectory.duration:.2f} s")
    print(
        f"Gun opening: {args.opening_mm:.1f} mm projected tip separation; right jaw rotates."
    )
    print("Jaw window 0..20 deg and speed 0.2 rad/s are simulation settings.")
    print(
        "Approximate shape/collisions; internal actuator omitted; TCP is a CAD reference."
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
