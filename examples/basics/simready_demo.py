"""botrail demo: SimReady Central props in a cell.

NVIDIA's SimReady Central (https://huggingface.co/spaces/nvidia/simready-central)
is an open library of simulation-ready USD assets — thousands of props and a
few vendor robots — each package in its publisher's Hugging Face dataset,
pinned to a dataset commit by the library's index. botrail reads them as
ordinary stages: here three of NVIDIA's warehouse props (CC-BY-4.0) stand
in the Franka cell of `demo.py` — two cartons on a pallet, a crate on the
table in front of the arm, a drum in the corner. Each one lands on the bill
of materials with the URL it came from, at its commit, and its license.

What the cell *verifies* — the table height the arm reaches down to, the
pallet the cartons stack on — is still generated from parameters
(`bt.parts`); the SimReady assets are the goods and the scenery, placed
where they stand and weighed as their packages say.

Run with:  python examples/basics/simready_demo.py [out.usda] [--studio]

Needs `pip install botrail[catalog]` (huggingface_hub) for the packages; the
first run downloads three of them (~25 MB) into the Hugging Face cache.
"""

import sys

import botrail as bt
from demo import fetch_franka

# The library's index pins every package to a dataset commit: these are the
# links the SimReady Central search hands out, kept as they came.
NVIDIA = (
    "https://huggingface.co/datasets/nvidia/simready-assets/blob/"
    "653d91e182011f357cf1644013cc94ddf194d349/{asset}/simready_usd/sm_{asset}_01.usd"
)
CARTON = NVIDIA.format(asset="box_cardboard_a01")
CRATE = NVIDIA.format(asset="crate_blue_a01")
DRUM = NVIDIA.format(asset="drum_plastic_blue_a01")


def build_scene() -> bt.Scene:
    robot = bt.Robot.from_usd(fetch_franka())
    scene = bt.Scene(robot)
    ped = bt.parts.pedestal(scene, "pedestal", height=0.45, position=(0.0, 0.0), model="PD-450")
    scene.set_robot_base_pose(*scene.frame(ped.frames[0]))
    scene.set_joint_positions([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.035, 0.035])

    # The table is a dimension the arm's reach is checked against: generated.
    table = bt.parts.table(scene, "table", size=(1.2, 0.8, 0.75), position=(0.9, 0.0), model="HFS8-1200")
    (tx, ty, tz), _ = scene.frame(table.frames[0])
    # The goods are SimReady packages, standing on the table's top face.
    bt.parts.simready(scene, "crate", (tx - 0.1, ty + 0.15, tz), CRATE, yaw=0.2, dynamic=True,
                      category="workpiece")

    bt.parts.pallet(scene, "pallet", position=(-0.3, 1.4))
    (px, py, pz), _ = scene.frame("pallet/top")
    first = bt.parts.simready(scene, "carton_a", (px - 0.05, py, pz), CARTON, yaw=0.05, dynamic=True,
                              category="workpiece")
    top = max(scene.obstacle_bounds(n)[1][2] for n in scene.obstacle_names if n.startswith(first + "/"))
    bt.parts.simready(scene, "carton_b", (px + 0.05, py + 0.02, top), CARTON, yaw=-0.08, dynamic=True,
                      category="workpiece")

    # Scenery: a drum in the corner, static, counted on the BOM all the same.
    bt.parts.simready(scene, "drum", (-1.4, -1.2), DRUM, yaw=0.6)
    return scene


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = args[0] if args else "simready_cell.usda"
    scene = build_scene()
    print(scene.bom().to_markdown())
    # Nothing is baked here: the file holds the cell as it stands, every
    # SimReady package copied in next to the layer with its textures.
    warnings = scene.export_usd(out)
    print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))
    if "--studio" in sys.argv:
        bt.studio(scene)
