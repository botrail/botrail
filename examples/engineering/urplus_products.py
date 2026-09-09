"""UR+ product configurations with separate purchase, mounting and connection checks.

Load public packages, or pass --catalog-root for an already staged catalog.
All gripper axes are kinematic simulation axes. No controller or URCap is run.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import botrail as bt

UR = "universal_robots/ur/ur5e/r2"
QC = "onrobot/quick-changer/109498/r1"
CONFIGURATIONS = {
    "hand-e": (
        UR,
        "robotiq/hand-e/hand-e-ur-es-077-kit/r1",
        "ur5e-es077-polyscope5",
        "female",
        None,
    ),
    "2f85-es077": (
        UR,
        "robotiq/2f/2f-85-ur-es-077-kit/r1",
        "ur5e-es077-polyscope5",
        "female",
        None,
    ),
    "zimmer-ur": (
        UR,
        "zimmer/hrc/hrc-03-118506/r1",
        "hrc03-118506-ur5e-io",
        "female",
        "npn",
    ),
    "zimmer-ur-old": (
        UR,
        "zimmer/hrc/hrc-03-118505/r1",
        "hrc03-118505-ur5e-io",
        "male",
        "npn",
    ),
    "zimmer-crx": (
        "fanuc/crx/crx20ia_l/r1",
        "zimmer/hrc/hrc-03-116787/r1",
        "hrc03-116787-crx20ia_l-io",
        "male",
        "pnp",
    ),
    "zimmer-doosan": (
        "doosan/m/m1509/r1",
        "zimmer/hrc/hrc-03-126895/r1",
        "hrc03-126895-m1509-io",
        "female",
        "pnp",
    ),
    "rg6": (UR, "onrobot/rg/rg6/r3", "ur5e-qcr-v3-female-tool", "female", None),
    "vgc10": (UR, "onrobot/vgc/vgc10/r1", "ur5e-qcr-v3-female-tool", "female", None),
}


def build(configuration="hand-e", *, catalog_root=None, format="urdf", revision=None):
    host, product, profile, gender, logic = CONFIGURATIONS[configuration]

    def load(pid):
        if catalog_root is not None:
            root = Path(catalog_root)
            return bt.Robot.from_package(root / pid, catalog_root=root, format=format)
        return bt.Robot.from_catalog(pid, revision=revision, format=format)

    robot = load(host)
    if configuration in ("rg6", "vgc10"):
        robot = robot.attach_tool(load(QC), prefix="qc_")
    robot = robot.attach_tool(load(product), prefix="eoat_")
    scene = bt.Scene()
    scene.add_robot(robot, name="robot")
    q = [0, -1.57, 1.57, -1.57, -1.57, 0] if host == UR else [0, 0.3, -0.5, 0, 0.4, 0]
    q += [0] * (robot.dof - 6)
    if configuration == "hand-e":
        q[-1] = 0.025
    if configuration.startswith("zimmer-"):
        q[-1] = 0.01
    scene.set_joint_positions(q)
    mounted = bt.mounting.report(scene)
    record = next(
        p for p in [*mounted.kits, *mounted.products] if p["catalog"] == product
    )
    # Only example settings are recorded; undocumented values remain unset.
    values = {"wrist_connector": "m8_8pin_" + gender, "voltage_v": 24}
    if logic:
        values.update(protocol="digital_io", signal_logic=logic)
    elif configuration in ("hand-e", "2f85-es077"):
        values.update(
            controller="ur_e_series",
            software_family="polyscope_5",
            protocol="modbus_rtu_rs485",
            plugin="robotiq_grippers_urcap",
        )
    else:
        values.update(controller="ur_e_series", plugin="onrobot_urcap")
    bt.connections.configure(
        scene,
        record["target"],
        profile,
        values=values,
        reference="Example declarations; verify installed wiring, supply and controller releases",
    )
    return scene


def export(scene, output):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    scene.save_project(output / "product.botrail")
    restored = bt.Scene.load_project(output / "product.botrail")
    (output / "replay.py").write_text(restored.generate_python(embed_catalog=True))
    (output / "bom.json").write_text(scene.bom().to_json())
    (output / "connections.json").write_text(bt.connections.report(scene).to_json())
    (output / "connections.md").write_text(bt.connections.report(scene).to_markdown())
    (output / "mounting.json").write_text(bt.mounting.report(scene).to_json())
    (output / "tool-loads.json").write_text(
        json.dumps(bt.select.tool_loads(scene), indent=2)
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configuration", choices=CONFIGURATIONS, default="hand-e")
    parser.add_argument("--catalog-root", type=Path)
    parser.add_argument("--revision", help="Pin the public HF catalog commit")
    parser.add_argument("--format", choices=["urdf", "usd"], default="urdf")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()
    scene = build(
        args.configuration,
        catalog_root=args.catalog_root,
        format=args.format,
        revision=args.revision,
    )
    print(bt.connections.report(scene).to_markdown())
    if args.out:
        export(scene, args.out)
    if args.studio:
        bt.studio(scene)
