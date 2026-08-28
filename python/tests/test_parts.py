"""Part identity and the derived bill of materials (D1 of
design/design-cell-engineering.md).

`scene.set_part()` pins *what a resident is* — catalog reference, maker,
model, category, quantity, free attributes — to a robot, device, sensor,
I/O node, obstacle or obstacle group by name; `scene.bom()` derives the
parts list from those pins plus the catalog identity robots and tools
already carry. These tests pin the resolution rules, the merge rule, the
totals, the three table formats, and the round trips through `.botrail`
and `generate_python()`.
"""

import json
import os
import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "basics"))

CACHE = Path(os.environ.get("BOTRAIL_CACHE_DIR") or Path.home() / ".cache" / "botrail")
HAS_FRANKA = (CACHE / "assets" / "franka" / "franka.usd").exists()
HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
HAS_CATALOG = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))


def cell() -> bt.Scene:
    """A small cell with one of everything the BOM lists: a robot, two
    identical tables, a fenced group, a conveyor with a belt slab of the
    same name, a beam sensor, a PLC node with a model, and a source/sink
    pair (which must *not* appear — they model an endless line)."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("table_a", size=(1.2, 0.8, 0.05), position=(0.6, 0.0, 0.7))
    scene.add_box("table_b", size=(1.2, 0.8, 0.05), position=(-0.6, 0.0, 0.7))
    for i in range(3):
        scene.add_box(f"fence/panel_{i}", size=(1.0, 0.05, 2.0), position=(2.0 + i, 0.0, 1.0))
    scene.add_box("belt", size=(2.0, 0.4, 0.05), position=(0.0, 1.0, 0.45))
    scene.add_conveyor(
        "belt",
        zone_position=(0.0, 1.0, 0.55),
        zone_size=(2.0, 0.4, 0.15),
        velocity=(0.2, 0.0, 0.0),
        running=False,
    )
    scene.add_beam_sensor("eye", frm=(0.0, 0.8, 0.5), to=(0.0, 1.2, 0.5))
    scene.add_box("crate_0", size=(0.1, 0.1, 0.1), position=(-3.0, 1.0, 0.6))
    scene.add_source(
        "magazine", pool=["crate_0"], park=(-3.0, 1.0, 0.6), position=(-1.0, 1.0, 0.6), interval=5.0
    )
    scene.add_sink("chute", zone_position=(3.0, 1.0, 0.6), zone_size=(0.5, 0.5, 0.5), source="magazine")
    scene.add_io_node("PLC1", kind="plc", channels=bt.io.di8(base="%IX0.0"), model="R04CPU")
    return scene


def rows_by_names(bom: bt.Bom) -> dict:
    return {tuple(row["names"]): row for row in bom.rows}


# ------------------------------------------------------------ resolution


def test_bare_cell_lists_equipment_unidentified_and_no_geometry() -> None:
    bom = cell().bom()
    # Robot, conveyor, sensor, PLC — the equipment; no obstacle, no
    # source/sink.
    assert [row["category"] for row in bom.rows] == [
        "robot",
        "conveyor",
        "sensor.photoelectric",
        "plc",
    ]
    assert [row["names"] for row in bom.rows] == [["simple_arm"], ["belt"], ["eye"], ["PLC1"]]
    # The PLC's own `model=` column already identifies it; the rest are
    # the purchasing to-do list.
    assert [row["names"][0] for row in bom.unidentified()] == ["simple_arm", "belt", "eye"]
    plc = rows_by_names(bom)[("PLC1",)]
    assert plc["model"] == "R04CPU"
    assert repr(bom) == "Bom(4 rows, 3 unidentified)"
    assert len(bom) == 4


def test_set_part_resolves_the_kind_and_needs_one_when_ambiguous() -> None:
    scene = cell()
    assert scene.set_part("simple_arm", manufacturer="FANUC", model="M-20iD/25") == "robot"
    assert scene.set_part("eye", catalog="keyence/pz-g61n", model="PZ-G61N") == "sensor"
    assert scene.set_part("PLC1", manufacturer="Mitsubishi Electric") == "io_node"
    assert scene.set_part("table_a", model="HFS8-1200") == "obstacle"
    assert scene.set_part("fence", model="FP-2000", qty=3) == "group"
    # `belt` is both a device and an obstacle.
    with pytest.raises(ValueError, match="names several things.*pass kind="):
        scene.set_part("belt", model="GVL-1200")
    assert scene.set_part("belt", kind="device", manufacturer="MISUMI", model="GVL-1200") == "device"
    assert scene.set_part("belt", kind="obstacle", model="belt slab") == "obstacle"
    with pytest.raises(ValueError, match="not a robot, device"):
        scene.set_part("nothing", model="x")
    with pytest.raises(ValueError, match="no sensor named"):
        scene.set_part("table_a", kind="sensor", model="x")
    with pytest.raises(ValueError, match="unknown kind"):
        scene.set_part("table_a", kind="thing", model="x")
    with pytest.raises(ValueError, match="qty must be at least 1"):
        scene.set_part("table_a", qty=0)
    with pytest.raises(ValueError, match="is a bool"):
        scene.set_part("table_a", certified=True)
    with pytest.raises(ValueError, match="number or a string"):
        scene.set_part("table_a", stops=[1, 2])

    entry = scene.part("belt")
    assert entry["kind"] == "device" and entry["model"] == "GVL-1200"
    assert scene.part("nothing") is None
    assert [p["target"] for p in scene.parts()] == [
        "simple_arm",
        "eye",
        "PLC1",
        "table_a",
        "fence",
        "belt",
        "belt",
    ]


def test_bom_merges_identical_products_and_overlays_pins() -> None:
    scene = cell()
    scene.set_part("table_a", model="HFS8-1200", mass_kg=30)
    scene.set_part("table_b", model="HFS8-1200", mass_kg=30)
    scene.set_part("fence", catalog="botrail/fence-panel@r1", category="structure.fence", qty=3, mass_kg=8)
    scene.set_part("belt", kind="device", manufacturer="MISUMI", model="GVL-1200", mass_kg=45)
    scene.set_part("PLC1", manufacturer="Mitsubishi Electric", description="main PLC")
    scene.set_part("simple_arm", manufacturer="FANUC", model="M-20iD/25", mass_kg=250, finish="RAL 1021")
    bom = scene.bom()
    rows = rows_by_names(bom)
    # Two tables → one row, qty 2, both names.
    tables = rows[("table_a", "table_b")]
    assert tables["qty"] == 2 and tables["category"] == "part"
    # The pinned group is one row for three, in its own category.
    fence = rows[("fence",)]
    assert fence["qty"] == 3 and fence["category"] == "structure.fence"
    assert fence["catalog"] == "botrail/fence-panel@r1"
    # The PLC keeps its `model=` and gains the maker and description.
    plc = rows[("PLC1",)]
    assert (plc["manufacturer"], plc["model"], plc["description"]) == (
        "Mitsubishi Electric",
        "R04CPU",
        "main PLC",
    )
    # Totals are Σ qty × value; text attributes ride along; a key nobody
    # states is None, not 0.
    assert bom.total("mass_kg") == pytest.approx(250 + 45 + 2 * 30 + 3 * 8)
    assert rows[("simple_arm",)]["attributes"] == {"mass_kg": 250.0, "finish": "RAL 1021"}
    assert bom.total("price") is None
    assert bom.attribute_keys() == ["finish", "mass_kg"]
    # Only the sensor is still without a maker or model.
    assert [r["names"] for r in bom.unidentified()] == [["eye"]]
    # Removing a resident drops its pin — and its line, when it was
    # geometry.
    scene.remove_obstacle("table_a")
    assert rows_by_names(scene.bom())[("table_b",)]["qty"] == 1
    scene.remove_device("belt")
    assert [p["target"] for p in scene.parts() if p["kind"] == "device"] == []
    assert ("belt",) not in rows_by_names(scene.bom())
    scene.remove_part("fence")
    assert ("fence",) not in rows_by_names(scene.bom())
    with pytest.raises(ValueError, match="no part pinned"):
        scene.remove_part("fence")


def test_rename_robot_follows_the_pin() -> None:
    scene = cell()
    scene.set_part("simple_arm", model="M-20iD/25")
    scene.rename_robot("simple_arm", "r1")
    assert scene.part("r1")["model"] == "M-20iD/25"
    assert scene.bom().rows[0]["names"] == ["r1"]


# --------------------------------------------------------------- tables


def test_tables_and_files(tmp_path: Path) -> None:
    scene = cell()
    scene.set_part("simple_arm", manufacturer="FANUC", model="M-20iD/25", mass_kg=250)
    scene.set_part("table_a", model='HFS8, "1200"', note="a|b")
    bom = scene.bom()
    csv = bom.to_csv()
    assert csv.splitlines()[0] == "category,manufacturer,model,catalog,qty,description,names,mass_kg,note"
    assert 'part,,"HFS8, ""1200""",,1,,table_a,,a|b' in csv
    md = bom.to_markdown()
    assert md.startswith("| # | category | manufacturer | model | catalog | qty | description | names | mass_kg | note |\n")
    assert "a\\|b" in md and "Totals: mass_kg = 250" in md
    doc = json.loads(bom.to_json())
    assert doc["totals"] == {"mass_kg": 250.0}
    assert doc["rows"][0]["names"] == ["simple_arm"]
    assert doc["rows"][0]["attributes"] == {"mass_kg": 250.0}

    scene.export_bom(tmp_path / "bom.csv")
    scene.export_bom(tmp_path / "bom.md")
    scene.export_bom(tmp_path / "bom.json")
    scene.export_bom(tmp_path / "bom.txt", format="md")
    assert (tmp_path / "bom.csv").read_text() == csv
    assert (tmp_path / "bom.md").read_text() == md
    assert (tmp_path / "bom.txt").read_text() == md
    assert json.loads((tmp_path / "bom.json").read_text()) == doc
    with pytest.raises(ValueError, match="unknown extension"):
        scene.export_bom(tmp_path / "bom.xlsx")
    bom.save(tmp_path / "again.csv")
    assert (tmp_path / "again.csv").read_text() == csv


# ---------------------------------------------------------- round trips


def test_parts_round_trip_through_project_and_generated_python(tmp_path: Path) -> None:
    scene = cell()
    scene.set_part("simple_arm", manufacturer="FANUC", model="M-20iD/25", mass_kg=250, finish="RAL 1021")
    scene.set_part("fence", catalog=("botrail/fence-panel", "abc123"), category="structure.fence", qty=3)
    scene.set_part("belt", kind="device", manufacturer="MISUMI", model="GVL-1200")
    scene.set_part("table_a", model="HFS8-1200", attributes={"model number": "x"})
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert reloaded.parts() == scene.parts()
    assert reloaded.bom().rows == scene.bom().rows

    code = reloaded.generate_python()
    assert (
        'scene.set_part("simple_arm", kind="robot", manufacturer="FANUC", model="M-20iD/25", '
        'finish="RAL 1021", mass_kg=250)' in code
    )
    assert (
        'scene.set_part("fence", kind="group", catalog="botrail/fence-panel@abc123", '
        'category="structure.fence", qty=3)' in code
    )
    assert 'scene.set_part("belt", kind="device", manufacturer="MISUMI", model="GVL-1200")' in code
    # A key that is not an identifier goes through `attributes=`.
    assert 'scene.set_part("table_a", kind="obstacle", model="HFS8-1200", attributes={"model number": "x"})' in code
    # The generated script rebuilds the same BOM.
    namespace: dict = {}
    exec(code.replace("bt.studio(scene)", ""), namespace)  # noqa: S102 — our own generated code
    assert namespace["scene"].bom().rows == scene.bom().rows

    # Older files without the field load with no pins.
    doc = json.loads(path.read_text())
    doc.pop("parts")
    (tmp_path / "old.botrail").write_text(json.dumps(doc))
    assert bt.Scene.load_project(tmp_path / "old.botrail").parts() == []


# ------------------------------------------------------------ the demo


@pytest.mark.skipif(not (HAS_FRANKA and HAS_CATALOG),
                    reason="the demo cell needs the Isaac Franka and the botrail catalog")
def test_sequence_demo_bom_is_complete() -> None:
    """The flagship demo types four lines of identity; the rest of the bill
    comes off the catalog packages the cell was ordered from. Nothing is
    left unidentified, the pedestal subtree and the pallet are still single
    parts, and every catalog line carries the part number it would be
    ordered by and the revision it was resolved at."""
    import sequence_demo as sd
    from demo import build_scene

    scene = build_scene()
    sd.build_cycle(scene)
    sd.identify_parts(scene)
    bom = scene.bom()
    assert bom.unidentified() == []

    # The four typed lines — the robot, the photo-eye, and the two whole
    # USD subtrees pinned as one part each.
    typed = [row for row in bom.rows if row["catalog"] is None]
    assert [row["names"] for row in typed] == [
        ["panda"],
        ["beam_pick"],
        ["/World/Pedestal"],
        ["/World/Pallet"],
    ]
    assert [row["category"] for row in typed] == [
        "robot",
        "sensor.photoelectric",
        "structure.pedestal",
        "pallet",
    ]

    # The catalog writes the rest: the belt and its stands, the rack and its
    # shelves, and the guarding down to a line per panel width. The guard is
    # two runs of one product, so its groups share the rows — a merged row
    # carries both names.
    ordered = {
        name: row
        for row in bom.rows
        if row["catalog"] is not None
        for name in row["names"]
    }
    assert set(ordered) == {
        "conv", "conv/stands", "rack", "rack/shelves",
        "fence/east", "fence/east/posts", "fence/west", "fence/west/posts",
        "fence/west/door",
        *(f"fence/east/panels/w{mm}" for mm in (1500, 1000, 800, 300, 200)),
        *(f"fence/west/panels/w{mm}" for mm in (1500, 400, 300, 200)),
    }
    # The part numbers spell out what was ordered: 3.8 m x 400 mm of belt,
    # a 900 x 450 x 1800 bay, an 800 mm door in a 2 m guard.
    assert ordered["conv"]["model"] == "BCU-400-3800"
    assert ordered["rack"]["model"] == "MR-900x450x1800"
    assert ordered["fence/west/door"]["model"] == "MGD-2000x800"
    assert all("@" in row["catalog"] for row in ordered.values())  # id@revision
    assert ordered["conv"]["qty"] == 1
    assert ordered["fence/east/posts"]["qty"] == 18  # both runs, one row

    # Typed masses plus the ones the packages computed from the sizes.
    assert bom.total("mass_kg") == pytest.approx(18 + 120 + 25 + 378.26)

