"""The scenery, ordered from the catalog.

A cell is mostly things nobody designs: a fence around it, a conveyor
through it, a rack beside it. They are standard products bought to size —
you pick a panel width, a belt length, a number of shelf levels — and until
now the numbers behind them were typed into the scene by hand, which is how
a bill of materials ends up quoting a panel that was never built.

Here every piece of that scenery comes from the model catalog. Each
generator is handed a catalog id instead of a model string, and from the
package it gets:

  * the sizes that are actually sold — ask for a 1.8 m fence and it refuses,
    with the heights it does have;
  * the layout rules — how many panels fit an edge, how far apart the
    conveyor's stands may stand, how close the shelves may be;
  * the part numbers and the mass, so the BOM is something you can send to a
    supplier rather than a description of the drawing;
  * and the drawing itself: each package ships a parametric primitive file,
    expanded to the size at hand, so a mesh panel is drawn as a frame with
    wire in it rather than a slab. None of that detail collides — the
    massing under it does, and it is the same either way.

Run with:  python examples/equipment_cell_demo.py [out_dir]

Needs the catalog: `pip install botrail[catalog]` (the packages are fetched
from the Hugging Face dataset botrail/botrail-catalog and cached).
"""

from __future__ import annotations

import sys
from pathlib import Path

import botrail as bt

EXAMPLES = Path(__file__).parent

FENCE = "botrail/fence/mesh-guard"
CONVEYOR = "botrail/conveyor/belt-unit"
RACK = "botrail/rack/medium-shelf"

# The fence runs around the cell; the door is the second bay of the south
# edge. Panels are not placed here — the catalog's widths are, and the
# generator fills each edge with the fewest of them that reach the corner.
CELL = [(-1.6, -1.6), (1.6, -1.6), (1.6, 1.6), (-1.6, 1.6)]


def build() -> bt.Scene:
    """The cell: an arm on a pedestal, and everything around it ordered."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    scene.set_part("simple_arm", manufacturer="ACME", model="SA-6", mass_kg=28)

    # The pedestal has no catalog package yet, so its identity is typed in.
    # The line it puts on the bill looks the same — but nothing checked that
    # a PD-400 is a thing you can buy at 0.4 m, and that is the whole
    # difference between this line and the ones below it.
    pedestal = bt.parts.pedestal(
        scene, "pedestal", height=0.4, position=(0.0, 0.0),
        model="PD-400", manufacturer="ACME", mass_kg=24,
    )
    scene.set_robot_base_pose(*scene.frame(pedestal.frames[0]))

    # A 2 m belt at 0.75 m, running east. Omitted dimensions come from the
    # catalog, so (x, y) is enough to place it — the stand height is a
    # dimension the product is sold in.
    bt.parts.conveyor(scene, "conv", catalog=CONVEYOR, position=(0.0, 1.0), length=2.0, speed=0.25)

    # Four levels of 1.2 x 0.6 m, and a frame on every deck: those are what a
    # pick aims at.
    bt.parts.rack(scene, "rack", catalog=RACK, position=(-1.0, -1.0), levels=4)

    bt.parts.fence(scene, "fence", path=CELL, catalog=FENCE, height=2.0, door=(0, 1))
    return scene


def deliver(scene: bt.Scene, out: Path) -> None:
    """The two documents this cell can already answer for: what it is made
    of, and what it takes up on the floor."""
    out.mkdir(parents=True, exist_ok=True)
    scene.export_bom(out / "equipment_bom.md")
    scene.export_bom(out / "equipment_bom.csv")
    scene.export_layout(out / "equipment_layout.svg", scale=120, title="equipment cell")


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("deliverables")
    scene = build()
    deliver(scene, out)

    bom = scene.bom()
    print(bom.to_markdown())
    footprint = scene.footprint()
    drawn = sum(1 for name in scene.obstacle_names if "/trim/" in name)
    print(
        f"\n{len(bom)} line(s), {bom.total('mass_kg'):.1f} kg, "
        f"{footprint['width']:.2f} x {footprint['depth']:.2f} m on the floor"
    )
    print(
        f"{len(scene.obstacle_names)} obstacles, of which {drawn} are drawn detail "
        "that never collides"
    )
    print(f"wrote the bill and the layout to {out}/")


if __name__ == "__main__":
    main()
