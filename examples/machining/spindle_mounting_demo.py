"""Inspect the manufacturer-documented ATI RCV-250 kit on a FANUC CRX-10iA.

The public catalog provides prebuilt reference models; the kit is attached
to the arm with `attach_tool` and `bt.mounting.report` repeats what the
loaded products say about the assembly: whether ATI documents the kit for
this host and whether it is still composed the way the part number says.
Nothing is measured from geometry, and no cutting tip or machining
operation is configured — the kit's TCP is a mounting reference.

Run with:  python examples/machining/spindle_mounting_demo.py [--studio]
                 [--output cell.botrail] [--catalog-root DIR]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import botrail as bt

ARM = "fanuc/crx/crx10ia/r1"
KIT = "ati/rcv/rcv-250-crx10-kit/r1"


def build(catalog_root: Path | None = None) -> bt.Scene:
    """The arm with the kit welded on its flange."""

    def load(pid: str) -> bt.Robot:
        if catalog_root is not None:
            return bt.Robot.from_package(catalog_root / pid, catalog_root=catalog_root)
        return bt.Robot.from_catalog(pid)

    arm, kit = load(ARM), load(KIT)
    return bt.Scene(arm.attach_tool(kit, prefix="kit_"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--studio", action="store_true", help="open the assembly in Studio")
    parser.add_argument("--output", type=Path, help="save a self-contained .botrail project")
    parser.add_argument("--catalog-root", type=Path, help="local packages for catalog development")
    args = parser.parse_args()

    scene = build(args.catalog_root)
    report = bt.mounting.report(scene)
    kit = report.kits[0]
    print("Manufacturer mounting support:", kit["manufacturer_support"]["status"])
    print("Kit composition:", kit["composition"]["status"])
    print(report.to_markdown())
    print("TCP is a mounting reference; a cutting tool needs its own calibrated tip.")
    if args.output:
        scene.save_project(args.output)
        print("Saved:", args.output)
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
