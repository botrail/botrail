"""`bt.catalog.search` — real products to choose from, filtered the way a
requirement reads. The index is a synthetic one here (no network): the
semantics under test are the filters, the ordering, the requirement-row
shortcut, writing a pick back onto the cell, and how the index is found
(a path, the hub, or the newest cached copy when the hub is away).
"""

from __future__ import annotations

import json
import sys
import types
from pathlib import Path

import pytest

import botrail as bt
from botrail import catalog

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

INDEX = {
    "schema_version": "0.1",
    "generated_at": "2026-08-20T00:00:00Z",
    "products": [
        {
            "id": "robotiq/2f/2f-85/r1", "category": "gripper.parallel", "name": "2F-85",
            "manufacturer": "Robotiq", "kind": "model", "validation_level": "V3", "distribution": "public",
            "specs": {"stroke_mm": 85, "payload_kg": 5.0, "mass_kg": 0.925, "ip_rating": "IP40"},
        },
        {
            "id": "robotiq/2f/2f-140/r1", "category": "gripper.parallel", "name": "2F-140",
            "manufacturer": "Robotiq", "kind": "model", "validation_level": "V2", "distribution": "public",
            "specs": {"stroke_mm": 140, "payload_kg": 2.5, "mass_kg": 1.025},
        },
        {
            "id": "onrobot/rg6/r1", "category": "gripper.parallel", "name": "RG6",
            "manufacturer": "OnRobot", "kind": "model", "validation_level": "V2", "distribution": "public",
            "specs": {"stroke_mm": 160, "payload_kg": 6.0, "mass_kg": 1.25},
        },
        {
            "id": "acme/vac/v1/r1", "category": "gripper.vacuum", "name": "V1",
            "manufacturer": "ACME", "kind": "model", "validation_level": "V1", "distribution": "public",
            "specs": {"payload_kg": 4.0},
        },
        {
            "id": "omron/e3z/e3z-t61/r1", "category": "sensor.photoelectric", "name": "E3Z-T61",
            "manufacturer": {"name": "OMRON"}, "kind": "spec", "validation_level": "V3", "distribution": "public",
            "specs": {"sensing_range_mm": 15000, "response_ms": 1, "output": "NPN"},
            "mechanical": {"footprint_mm": [11, 31], "height_mm": 20, "mass_kg": 0.065},
        },
        {
            "id": "omron/e3z/e3z-d62/r1", "category": "sensor.photoelectric", "name": "E3Z-D62",
            "manufacturer": {"name": "OMRON"}, "kind": "spec", "validation_level": "V2", "distribution": "public",
            "specs": {"sensing_range_mm": 1000, "output": "PNP"},
        },
        {
            "id": "universal_robots/ur/ur5e/r1", "category": "manipulator", "name": "UR5e",
            "manufacturer": "Universal Robots", "validation_level": "V2", "distribution": "public",
            "specs": {"dof": 6, "payload_kg": 5.0, "reach_mm": 850},
        },
        {
            "id": "universal_robots/ur/ur5e/r2", "category": "manipulator", "name": "UR5e",
            "manufacturer": "Universal Robots", "validation_level": "V3", "distribution": "public",
            "specs": {"dof": 6, "payload_kg": 5.0, "reach_mm": 850},
        },
        {
            "id": "universal_robots/ur/ur10e/r1", "category": "manipulator", "name": "UR10e",
            "manufacturer": "Universal Robots", "validation_level": "V2", "distribution": "recipe_only",
            "specs": {"dof": 6, "payload_kg": 12.5, "reach_mm": 1300},
        },
    ],
}


@pytest.fixture()
def index() -> catalog.Index:
    return catalog.Index.from_dict(INDEX, revision="abc123")


def ids(products) -> list[str]:
    return [p.id for p in products]


def test_filters_read_like_requirements(index: catalog.Index) -> None:
    # Category is a prefix match on dotted names (level, then id, with no
    # minimums to measure closeness against); minimums are `>=`.
    assert ids(index.search("gripper")) == [
        "robotiq/2f/2f-85/r1", "onrobot/rg6/r1", "robotiq/2f/2f-140/r1", "acme/vac/v1/r1",
    ]
    assert ids(index.search("gripper.parallel", stroke_mm=150, payload_kg=2.3)) == ["onrobot/rg6/r1"]
    # Unknown is not a pass: the vacuum gripper states no stroke.
    assert set(ids(index.search("gripper", stroke_mm=1))) == set(ids(index.search("gripper.parallel")))
    # `__max`, string specs, maker and level.
    assert ids(index.search("gripper.parallel", mass_kg__max=1.0)) == ["robotiq/2f/2f-85/r1"]
    assert ids(index.search(ip_rating="IP40")) == ["robotiq/2f/2f-85/r1"]
    assert ids(index.search("sensor.photoelectric", output="pnp")) == ["omron/e3z/e3z-d62/r1"]
    assert ids(index.search(manufacturer="onrobot")) == ["onrobot/rg6/r1"]
    assert ids(index.search("manipulator", level="V3")) == ["universal_robots/ur/ur5e/r2"]
    assert ids(index.search(kind="spec")) == ["omron/e3z/e3z-t61/r1", "omron/e3z/e3z-d62/r1"]
    assert ids(index.search(text="ur10")) == ["universal_robots/ur/ur10e/r1"]
    assert ids(index.search("manipulator", limit=1)) == ["universal_robots/ur/ur5e/r2"]
    with pytest.raises(TypeError):
        index.search(stroke_mm=True)


def test_order_is_level_then_closeness_then_id(index: catalog.Index) -> None:
    # Both parallel grippers with stroke >= 100 are V2; the closer fit
    # (less headroom over the minimum) comes first.
    assert ids(index.search("gripper.parallel", stroke_mm=100)) == ["robotiq/2f/2f-140/r1", "onrobot/rg6/r1"]
    assert ids(index.search("gripper.parallel", stroke_mm=100, payload_kg=3)) == ["onrobot/rg6/r1"]
    # A higher validation level outranks closeness.
    assert ids(index.search("sensor.photoelectric", sensing_range_mm=500)) == [
        "omron/e3z/e3z-t61/r1", "omron/e3z/e3z-d62/r1",
    ]
    # Same query, same list.
    assert ids(index.search("gripper")) == ids(index.search("gripper"))


def test_get_resolves_ids_like_from_catalog(index: catalog.Index) -> None:
    assert index.get("onrobot/rg6/r1").name == "RG6"
    assert index.get("universal_robots/ur5e").id == "universal_robots/ur/ur5e/r2"  # newest revision
    assert index.get("rg6").manufacturer == "OnRobot"
    with pytest.raises(KeyError):
        index.get("omron/e3z")  # two products
    with pytest.raises(KeyError):
        index.get("nope")
    assert index.categories() == ["gripper.parallel", "gripper.vacuum", "manipulator", "sensor.photoelectric"]
    assert len(index) == 9


def test_product_attributes_and_repr(index: catalog.Index) -> None:
    p = index.get("omron/e3z/e3z-t61/r1")
    # Numeric specs and `mechanical` flatten into one attribute dict; a
    # footprint pair becomes two sides; strings stay out.
    assert p.attributes() == {
        "sensing_range_mm": 15000, "response_ms": 1, "footprint_x_mm": 11, "footprint_y_mm": 31,
        "height_mm": 20, "mass_kg": 0.065,
    }
    assert p.value("range_mm") == 15000  # the requirement alias reads it
    assert p.text("output") == "NPN" and p.manufacturer == "OMRON"
    assert p.catalog_ref == ("omron/e3z/e3z-t61/r1", "abc123") and p.level == 3
    assert repr(p).startswith("Product('omron/e3z/e3z-t61/r1', sensor.photoelectric, V3, ")
    assert p.to_dict()["attributes"]["mass_kg"] == 0.065


def test_search_for_a_requirement_row_and_identify(index: catalog.Index) -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    scene.add_beam_sensor("eye", frm=(0.25, 0.25, 0.03), to=(0.25, 2.25, 0.03))  # 2 m span
    req = scene.requirements()
    row = req["eye"]
    assert row.status == "unidentified" and row.minimum == {"sensing_range_mm": 2000.0}
    cands = catalog.search_for(row, index=index)
    assert ids(cands) == ["omron/e3z/e3z-t61/r1"]  # D62's 1 m is short
    # `extra` filters ride along; explicit options too.
    assert catalog.search_for(row, index=index, output="PNP") == []
    assert ids(catalog.search_for(row, index=index, level="V2", sensing_range_mm=500)) == [
        "omron/e3z/e3z-t61/r1", "omron/e3z/e3z-d62/r1",
    ]
    # Writing the pick onto the cell: the identity and the numbers come
    # along, so the requirement check now reads them.
    cands[0].identify(scene, "eye")
    part = scene.part("eye")
    assert part["catalog"] == "omron/e3z/e3z-t61/r1@abc123" and part["model"] == "E3Z-T61"
    assert part["manufacturer"] == "OMRON" and part["category"] == "sensor.photoelectric"
    assert part["attributes"]["sensing_range_mm"] == 15000
    after = scene.requirements()["eye"]
    assert after.status == "ok" and after.requirements[0].provided == 15000
    # Overrides add or replace attributes.
    index.get("rg6").identify(scene, "eye", mass_kg=9.9)
    assert scene.part("eye")["attributes"]["mass_kg"] == 9.9


def test_module_search_takes_an_index_or_a_path(tmp_path: Path, index: catalog.Index, monkeypatch) -> None:
    path = tmp_path / "index.json"
    path.write_text(json.dumps(INDEX))
    assert ids(catalog.search("gripper.vacuum", index=index)) == ["acme/vac/v1/r1"]
    assert ids(catalog.search("gripper.vacuum", index=path)) == ["acme/vac/v1/r1"]
    assert catalog.index(path=path).revision is None and catalog.index(path=path).source == str(path)
    monkeypatch.setenv("BOTRAIL_CATALOG_INDEX", str(path))
    assert ids(catalog.search("gripper.vacuum")) == ["acme/vac/v1/r1"]


def test_index_comes_from_the_hub_or_the_cache(tmp_path: Path, monkeypatch) -> None:
    monkeypatch.delenv("BOTRAIL_CATALOG_INDEX", raising=False)
    monkeypatch.setenv("HF_HOME", str(tmp_path / "hf"))
    monkeypatch.delenv("HF_HUB_CACHE", raising=False)
    catalog._CACHE.clear()
    hub_dir = tmp_path / "hf" / "hub" / "datasets--botrail--botrail-catalog" / "snapshots"
    # Two cached snapshots; the newer index (by generated_at) wins, not the
    # later directory name.
    old = dict(INDEX, generated_at="2026-08-01T00:00:00Z", products=INDEX["products"][:1])
    new = dict(INDEX, generated_at="2026-08-20T00:00:00Z")
    (hub_dir / "ffff").mkdir(parents=True)
    (hub_dir / "ffff" / "index.json").write_text(json.dumps(old))
    (hub_dir / "0000").mkdir(parents=True)
    (hub_dir / "0000" / "index.json").write_text(json.dumps(new))

    # The hub answers: pinned to the commit it names.
    fake = types.ModuleType("huggingface_hub")
    calls: dict = {}

    def dataset_info(repo_id, *, revision=None):
        assert repo_id == "botrail/botrail-catalog"
        calls["revision"] = revision
        return types.SimpleNamespace(sha="c0ffee")

    def hf_hub_download(repo_id, filename=None, repo_type=None, revision=None):
        assert (filename, repo_type, revision) == ("index.json", "dataset", "c0ffee")
        return str(hub_dir / "0000" / "index.json")

    fake.dataset_info = dataset_info
    fake.hf_hub_download = hf_hub_download
    monkeypatch.setitem(sys.modules, "huggingface_hub", fake)
    idx = catalog.index()
    assert idx.revision == "c0ffee" and calls["revision"] is None and len(idx) == 9
    assert catalog.search("gripper.vacuum")[0].revision == "c0ffee"
    assert catalog.index() is idx  # cached per process
    assert catalog.index(revision="c0ffee", refresh=True).revision == "c0ffee"

    # The hub is away: the newest cached copy, named by its snapshot.
    def offline(repo_id, *, revision=None):
        raise OSError("offline")

    fake.dataset_info = offline
    catalog._CACHE.clear()
    idx = catalog.index()
    assert idx.revision == "0000" and len(idx) == 9
    assert catalog.index(offline=True, refresh=True).revision == "0000"
    assert catalog.index(revision="ffff", refresh=True).revision == "ffff"
    with pytest.raises(ValueError, match="cannot fetch the catalog index"):
        catalog.index(offline=False, refresh=True)
    # No hub module at all still reads the cache.
    monkeypatch.setitem(sys.modules, "huggingface_hub", None)
    catalog._CACHE.clear()
    assert catalog.index().revision == "0000"
    # Nothing cached and no hub: a clear error.
    monkeypatch.setenv("HF_HOME", str(tmp_path / "empty"))
    catalog._CACHE.clear()
    with pytest.raises(ValueError, match="no cached copy"):
        catalog.index()
    with pytest.raises(ValueError, match="no catalog index in the Hugging Face cache"):
        catalog.index(offline=True)
    catalog._CACHE.clear()
