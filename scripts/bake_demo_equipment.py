"""Regenerates ``examples/assets/factory_equipment.usda`` — the demo cell's
catalog equipment (the belt, the rack, the guarding, the control cabinet,
the gate's light curtain and the stock) baked to a static USD layer.

The hand-authored layout layer ``factory.usda`` deliberately carries no
standard products: ``examples/basics/demo.py`` orders them from the model
catalog at run time. The browser demo (``studio/dist-wasm``, deployed at
``/demo/``) cannot do that — no Python, no catalog — so the same equipment
is baked here once and shipped next to the layout layer. Re-run after
changing ``equip_cell`` and commit the result:

    .venv/bin/python scripts/bake_demo_equipment.py

Needs ``botrail[catalog]`` (the packages come from the Hugging Face
dataset ``botrail/botrail-catalog`` and are cached).
"""

import sys
from pathlib import Path

import botrail as bt

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "examples" / "basics"))
from demo import equip_cell  # noqa: E402

OUT = REPO / "examples" / "assets" / "factory_equipment.usda"


def main() -> None:
    scene = bt.Scene()
    equip_cell(scene)
    warnings = scene.export_usd(OUT)
    for w in warnings:
        print(f"warning: {w}")
    if warnings:
        raise SystemExit("the equipment layer must bake clean")
    n = len(scene.obstacle_names)
    print(f"baked {n} prims to {OUT.relative_to(REPO)} ({OUT.stat().st_size // 1024} KiB)")


if __name__ == "__main__":
    main()
