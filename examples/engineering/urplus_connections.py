"""UR5e + documented ES-062 kit: recorded settings and separate checks.

Build the r3 kit and stage its component packages as described in
docs/guides/connections.md. The files are local so the same example can run
offline after preparation. No controller or URCap is installed or contacted.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import botrail as bt

ARM = "universal_robots/ur/ur5e/r2"
KIT = "robotiq/2f/2f-85-ur-es-062-kit/r3"


def build(catalog_root: Path, *, connector="m8_8pin_male", software_version=None,
          format="urdf") -> bt.Scene:
    arm = bt.Robot.from_package(catalog_root / ARM, format=format)
    kit = bt.Robot.from_package(catalog_root / KIT, catalog_root=catalog_root, format=format)
    robot = arm.attach_tool(kit, prefix="kit_")
    scene = bt.Scene()
    scene.add_robot(robot, name="robot")
    scene.set_joint_positions([0, -1.57, 1.57, -1.57, -1.57, 0] + [0] * (robot.dof - 6))
    bt.connections.configure(scene, "robot/tool", "ur5e-es062-polyscope5", values={
        "wrist_connector": connector,
        "voltage_v": 24,
        "protocol": "modbus_rtu_rs485",
        "controller": "ur_e_series",
        "software_family": "polyscope_5",
        "software_version": software_version,
        "plugin": "robotiq_grippers_urcap",
    }, reference="Example declarations; inspect the installed hardware and software before filling missing values")
    return scene


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog-root", type=Path, required=True)
    parser.add_argument("--connector", choices=["m8_8pin_male", "m8_8pin_female"], default="m8_8pin_male")
    parser.add_argument("--software-version")
    parser.add_argument("--format", choices=["urdf", "usd"], default="urdf")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()
    scene = build(args.catalog_root, connector=args.connector, software_version=args.software_version, format=args.format)
    print(bt.connections.report(scene).to_markdown())
    if args.out:
        args.out.mkdir(parents=True, exist_ok=True)
        scene.save_project(args.out / "urplus-connection.botrail")
        (args.out / "connections.json").write_text(bt.connections.report(scene).to_json())
        (args.out / "connections.md").write_text(bt.connections.report(scene).to_markdown())
        (args.out / "mounting.json").write_text(bt.mounting.report(scene).to_json())
    if args.studio:
        bt.studio(scene)
