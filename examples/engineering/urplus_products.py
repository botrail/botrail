"""Load public UR+ product configurations and review their mounting.

Each configuration is a host arm and a gripper from the public catalog,
welded together with `attach_tool`. `bt.mounting.report` repeats what the
loaded products say about that assembly — the coupling a kit requires
between gripper and flange, whether the kit is documented for this host —
and the bill of materials lists what was ordered. All gripper axes are
kinematic simulation axes; no controller or URCap is run.

Run with:  python examples/engineering/urplus_products.py
                 [--configuration NAME] [--studio] [--catalog-root DIR]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import botrail as bt

UR = "universal_robots/ur/ur5e/r2"
QC = "onrobot/quick-changer/109498/r1"  # OnRobot grippers mount through the quick changer
CONFIGURATIONS = {
    "hand-e": (UR, "robotiq/hand-e/hand-e-ur-es-077-kit/r1"),
    "2f85-es077": (UR, "robotiq/2f/2f-85-ur-es-077-kit/r1"),
    "zimmer-ur": (UR, "zimmer/hrc/hrc-03-118506/r1"),
    "zimmer-ur-old": (UR, "zimmer/hrc/hrc-03-118505/r1"),
    "zimmer-crx": ("fanuc/crx/crx20ia_l/r1", "zimmer/hrc/hrc-03-116787/r1"),
    "zimmer-doosan": ("doosan/m/m1509/r1", "zimmer/hrc/hrc-03-126895/r1"),
    "rg6": (UR, "onrobot/rg/rg6/r3"),
    "vgc10": (UR, "onrobot/vgc/vgc10/r1"),
}
QUICK_CHANGED = ("rg6", "vgc10")


def build(configuration: str = "hand-e", catalog_root: Path | None = None) -> bt.Scene:
    """The host arm with the product on its flange, in a display posture."""
    host_id, product_id = CONFIGURATIONS[configuration]

    def load(pid: str) -> bt.Robot:
        if catalog_root is not None:
            return bt.Robot.from_package(catalog_root / pid, catalog_root=catalog_root)
        return bt.Robot.from_catalog(pid)

    host = load(host_id)
    if configuration in QUICK_CHANGED:
        host = host.attach_tool(load(QC), prefix="qc_")
    scene = bt.Scene(host.attach_tool(load(product_id), prefix="kit_"))
    # The arm bent over the table, the fingers slightly open.
    q = [0, -1.57, 1.57, -1.57, -1.57, 0] if host_id == UR else [0, 0.3, -0.5, 0, 0.4, 0]
    q += [0] * (scene.robot.dof - 6)
    if configuration == "hand-e":
        q[-1] = 0.025
    if configuration.startswith("zimmer-"):
        q[-1] = 0.01
    scene.set_joint_positions(q)
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--configuration", choices=CONFIGURATIONS, default="hand-e")
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--catalog-root", type=Path, help="local packages for catalog development")
    args = parser.parse_args()

    scene = build(args.configuration, args.catalog_root)
    print(scene.bom().to_markdown())
    print(bt.mounting.report(scene).to_markdown())
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
