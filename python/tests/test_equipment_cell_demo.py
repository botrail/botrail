"""`examples/equipment_cell_demo.py` — a cell whose scenery is ordered.

The fence, the conveyor and the rack come from the model catalog, so what
this pins is the part the catalog is responsible for: every line of the bill
names a product and the revision it came from, the counts follow the layout
rules the packages carry, and the detail those packages are drawn with stays
out of collision.

Skipped unless the catalog is already in the Hugging Face cache (run the
demo once), the same way the weld station demo's test is.
"""

import os
import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

HF_HUB = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
pytestmark = pytest.mark.skipif(
    not (HF_HUB / "datasets--botrail--botrail-catalog").exists(),
    reason="botrail catalog not in the HF cache (run examples/equipment_cell_demo.py once)",
)


@pytest.fixture(scope="module")
def scene() -> bt.Scene:
    import equipment_cell_demo as demo

    return demo.build()


def rows(scene: bt.Scene) -> dict:
    return {row["names"][0]: row for row in scene.bom().rows}


def test_every_catalog_line_is_something_you_could_order(scene: bt.Scene) -> None:
    catalogued = [row for row in scene.bom().rows if row["catalog"]]
    assert len(catalogued) >= 8
    for row in catalogued:
        # A part number, a maker, and the revision it was read from.
        assert row["model"] and row["manufacturer"]
        assert "@" in row["catalog"]
        assert row["qty"] >= 1
    # The three packages, and nothing else, are what the scenery came from.
    assert {row["catalog"].split("/r1@")[0] for row in catalogued} == {
        "botrail/fence/mesh-guard",
        "botrail/conveyor/belt-unit",
        "botrail/rack/medium-shelf",
    }


def test_the_counts_come_from_the_rules_the_packages_carry(scene: bt.Scene) -> None:
    by = rows(scene)
    # A 3.2 m square at panel widths of 200..1500: 12 posts, and panels whose
    # widths all exist in the catalog.
    assert by["fence/posts"]["qty"] == 12
    widths = {
        int(name.rsplit("w", 1)[1]) for name in by if name.startswith("fence/panels/w")
    }
    assert widths <= {200, 300, 400, 600, 800, 1000, 1200, 1500}
    assert by["fence/door"]["model"].startswith("MGD-2000x")
    # 2 m of belt at a 1.5 m maximum span -> three stands; four levels -> four
    # shelves.
    assert by["conv/stands"]["qty"] == 3
    assert by["rack/shelves"]["qty"] == 4
    # The mass is the sum of the parts, and it is not zero by accident.
    assert scene.bom().total("mass_kg") > 300


def test_the_drawing_is_detail_and_the_massing_is_what_collides(scene: bt.Scene) -> None:
    drawn = [name for name in scene.obstacle_names if "/trim/" in name]
    assert len(drawn) > len(scene.obstacle_names) - len(drawn)
    # Panels are drawn as a frame with wire in it; the slab underneath is
    # what a robot can hit.
    assert any(name.endswith("/wire_v1") for name in drawn)
    assert any("/panels/" in name for name in scene.obstacle_names if "/trim/" not in name)


def test_it_writes_the_bill_and_the_sheet(scene: bt.Scene, tmp_path: Path) -> None:
    import equipment_cell_demo as demo

    demo.deliver(scene, tmp_path)
    bom = (tmp_path / "equipment_bom.md").read_text()
    assert "MGP-2000" in bom and "BCU-300-2000" in bom and "MR-SH-1200x600" in bom
    sheet = (tmp_path / "equipment_layout.svg").read_text()
    assert sheet.startswith("<svg") and "Mesh Safety Guard" in sheet
