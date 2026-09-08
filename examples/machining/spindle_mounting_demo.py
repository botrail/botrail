"""Inspect the manufacturer-documented ATI RCV-250 kit on FANUC CRX-10iA.

The public catalog provides prebuilt reference models. This example checks the
mounting assembly; it does not configure a cutting tip or machining operation.

    uv run python examples/machining/spindle_mounting_demo.py --studio
"""
from __future__ import annotations

import argparse
from pathlib import Path

import botrail as bt

ARM = "fanuc/crx/crx10ia/r1"
KIT = "ati/rcv/rcv-250-crx10-kit/r1"


def build_cell(catalog_root: Path | None = None) -> bt.Scene:
    def load(pid: str) -> bt.Robot:
        if catalog_root is not None:
            return bt.Robot.from_package(catalog_root / pid, catalog_root=catalog_root)
        return bt.Robot.from_catalog(pid)

    arm, kit = load(ARM), load(KIT)
    return bt.Scene(arm.attach_tool(kit, prefix="kit_"))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--studio", action="store_true", help="Open the mounting assembly in Studio")
    parser.add_argument("--output", type=Path, help="Save a self-contained .botrail project")
    parser.add_argument("--catalog-root", type=Path, help="Use local packages for catalog development")
    args = parser.parse_args()
    scene = build_cell(args.catalog_root)
    report = bt.mounting.report(scene)
    for kit in report.kits:
        print("Manufacturer mounting support:", kit["manufacturer_support"]["status"])
        print("Kit composition:", kit["composition"]["status"])
        print("Detailed fit:", kit["detailed_fit"]["status"])
    print("TCP is a mounting reference; the selected cutting tool needs its own calibrated tip.")
    if args.output:
        scene.save_project(args.output)
        print("Saved:", args.output)
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
