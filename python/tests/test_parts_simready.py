"""`bt.simready` / `bt.parts.simready` — a SimReady Central package placed as
scenery (design-simready.md S2). A hand-written package stands in for the
library: the packaging manifest, the `.metadata` root list and a root
layer whose body sits off its footprint and above the floor, the way a
modelled prop's origin does, with an empty `Grasp` scope the way NVIDIA's
props carry one."""

import json
import math
from pathlib import Path

import botrail as bt
import pytest

ROOT_LAYER = """#usda 1.0
(
    defaultPrim = "RootNode"
    metersPerUnit = 1
    upAxis = "Z"
)

def Scope "PhysicsMaterials"
{
    def Material "cardboard" (prepend apiSchemas = ["PhysicsMaterialAPI"])
    {
        float physics:dynamicFriction = 0.7
        float physics:staticFriction = 0.8
        float physics:restitution = 0.4
    }
}

def Xform "RootNode" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
{
    def Xform "Geometry"
    {
        def Cube "body" (prepend apiSchemas = ["PhysicsMassAPI", "PhysicsCollisionAPI", "MaterialBindingAPI"])
        {
            double size = 2
            float physics:mass = 2.5
            rel material:binding:physics = </PhysicsMaterials/cardboard>
            double3 xformOp:translate = (0.1, 0, 0.2)
            double3 xformOp:scale = (0.3, 0.2, 0.15)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }
    }

    def Scope "Grasp"
    {
    }
}
"""

BOX_URL = (
    "https://huggingface.co/datasets/nvidia/simready-assets/blob/"
    "653d91e182011f357cf1644013cc94ddf194d349/box_cardboard_a01/simready_usd/sm_box_cardboard_a01_01.usd"
)


@pytest.fixture
def package(tmp_path: Path) -> Path:
    directory = tmp_path / "crate_demo"
    (directory / "simready_usd").mkdir(parents=True)
    (directory / ".metadata").mkdir()
    (directory / "simready_usd" / "sm_crate_demo.usda").write_text(ROOT_LAYER)
    (directory / ".metadata" / "com.nvidia.simready.root_usds.json").write_text(
        json.dumps({"format_version": "1.0", "entries": ["simready_usd/sm_crate_demo.usda"]})
    )
    (directory / "com.nvidia.simready.packaging.json").write_text(
        json.dumps({
            "format_version": "1.0",
            "package_id": "simready.hf.acme.simready-assets.crate_demo.2026.10.23_00",
            "license": "CC-BY-4.0",
        })
    )
    return directory


def test_fetch_reads_a_local_package(package: Path):
    pkg = bt.simready.fetch(package)
    assert pkg.root == package / "simready_usd" / "sm_crate_demo.usda"
    assert pkg.directory == package
    assert pkg.asset == "crate_demo"
    assert pkg.license == "CC-BY-4.0"
    assert pkg.publisher == "acme"
    assert pkg.repo is None and pkg.revision is None
    assert pkg.source == str(pkg.root)
    # The root layer itself names the same package.
    assert bt.simready.fetch(pkg.root).directory == package


def test_parse_url_takes_the_library_links():
    repo, revision, asset, path = bt.simready.parse_url(BOX_URL)
    assert repo == "nvidia/simready-assets"
    assert revision == "653d91e182011f357cf1644013cc94ddf194d349"
    assert asset == "box_cardboard_a01"
    assert path == "simready_usd/sm_box_cardboard_a01_01.usd"
    assert bt.simready.parse_url(BOX_URL.replace("/blob/", "/resolve/"))[2] == "box_cardboard_a01"
    assert bt.simready.parse_url(
        "https://huggingface.co/datasets/Robotiq-Official/simready-assets/tree/main/Robotiq_2F_85"
    ) == ("Robotiq-Official/simready-assets", "main", "Robotiq_2F_85", None)
    with pytest.raises(ValueError, match="not a SimReady package URL"):
        bt.simready.parse_url("https://example.com/box.usd")


def test_load_usd_hands_on_the_authored_mass(package: Path):
    scene = bt.Scene()
    (name,) = scene.load_usd(package / "simready_usd" / "sm_crate_demo.usda", prefix="crate")
    physics = scene.obstacle_physics(name)
    assert physics["mass"] == pytest.approx(2.5)
    assert physics["dynamic"] is False
    # The bound physics material comes along: sliding friction and restitution.
    assert physics["friction"] == pytest.approx(0.7)
    assert physics["restitution"] == pytest.approx(0.4)
    assert scene.obstacle_physics("nothing") is None


def test_simready_stands_on_the_floor_and_pins_its_source(package: Path):
    scene = bt.Scene()
    name = bt.parts.simready(scene, "crate", (1.0, -0.5), package, yaw=math.pi / 2, dynamic=True)
    assert name == "crate"
    obstacles = [n for n in scene.obstacle_names if n.startswith("crate/")]
    assert obstacles == ["crate/RootNode/Geometry/body"]
    lo, hi = scene.obstacle_bounds(obstacles[0])
    # 0.6 × 0.4 × 0.3 m, turned a quarter: the long side now runs along y;
    # the footprint is centred on (1.0, -0.5) and the underside is on the floor.
    assert (lo[0] + hi[0]) / 2 == pytest.approx(1.0, abs=1e-6)
    assert (lo[1] + hi[1]) / 2 == pytest.approx(-0.5, abs=1e-6)
    assert hi[0] - lo[0] == pytest.approx(0.4, abs=1e-6)
    assert hi[1] - lo[1] == pytest.approx(0.6, abs=1e-6)
    assert lo[2] == pytest.approx(0.0, abs=1e-6)
    assert hi[2] == pytest.approx(0.3, abs=1e-6)
    # The package's frame rode along: its origin was 0.1 m before the
    # footprint's middle and 0.05 m below the underside.
    pos, _ = scene.frame("crate/RootNode/Grasp")
    assert pos == pytest.approx((1.0, -0.6, -0.05), abs=1e-6)
    physics = scene.obstacle_physics(obstacles[0])
    assert physics["dynamic"] is True
    assert physics["mass"] == pytest.approx(2.5)

    rows = [r for r in scene.bom().rows if r["category"] == "prop"]
    assert len(rows) == 1
    row = rows[0]
    assert row["model"] == "crate_demo"
    assert row["manufacturer"] == "acme"
    assert row["qty"] == 1
    assert row["attributes"]["license"] == "CC-BY-4.0"
    assert row["attributes"]["source"] == str(package / "simready_usd" / "sm_crate_demo.usda")
    assert row["attributes"]["mass_kg"] == pytest.approx(2.5)


def test_simready_takes_a_mass_and_a_category_of_its_own(package: Path):
    scene = bt.Scene()
    bt.parts.simready(scene, "crate", (0, 0), package, mass_kg=4.0, friction=0.9, category="workpiece", lot="A1")
    (made,) = [n for n in scene.obstacle_names if n.startswith("crate/")]
    physics = scene.obstacle_physics(made)
    assert physics["dynamic"] is False
    assert physics["mass"] == pytest.approx(4.0)
    assert physics["friction"] == pytest.approx(0.9)
    (row,) = [r for r in scene.bom().rows if r["category"] == "workpiece"]
    assert row["attributes"]["mass_kg"] == pytest.approx(4.0)
    assert row["attributes"]["lot"] == "A1"


def test_fetch_the_library_box_and_place_it():
    """Live: the cardboard box NVIDIA publishes, at the commit the library
    pins — skipped where the hub cannot be reached."""
    pytest.importorskip("huggingface_hub")
    try:
        pkg = bt.simready.fetch(BOX_URL)
    except ValueError as err:
        if "cannot reach" not in str(err):
            raise
        pytest.skip(f"SimReady dataset unavailable: {err}")
    assert pkg.repo == "nvidia/simready-assets"
    assert pkg.revision == "653d91e182011f357cf1644013cc94ddf194d349"
    assert pkg.license == "CC-BY-4.0"
    assert pkg.publisher == "nvidia"
    assert pkg.source == BOX_URL
    # The source copies stay behind; the MDL modules the materials import come along.
    source_assets = pkg.directory / "simready_usd" / "source_assets"
    assert not list(source_assets.glob("*.png"))
    assert (source_assets / "OmniPBR.mdl").is_file()

    scene = bt.Scene()
    bt.parts.simready(scene, "box", (0.5, 0.5), pkg, dynamic=True)
    (made,) = [n for n in scene.obstacle_names if n.startswith("box/")]
    lo, hi = scene.obstacle_bounds(made)
    assert hi[0] - lo[0] == pytest.approx(0.699, abs=0.01)
    assert hi[1] - lo[1] == pytest.approx(0.521, abs=0.01)
    assert hi[2] - lo[2] == pytest.approx(0.510, abs=0.01)
    assert lo[2] == pytest.approx(0.0, abs=1e-6)
    assert scene.obstacle_physics(made)["mass"] == pytest.approx(1.0, abs=0.01)
    (row,) = [r for r in scene.bom().rows if r["category"] == "prop"]
    assert row["attributes"]["source"] == BOX_URL
    assert row["attributes"]["license"] == "CC-BY-4.0"


# ------------------------------------------------------------------ search

CRATE_URL = (
    "https://huggingface.co/datasets/nvidia/simready-assets/blob/"
    "653d91e182011f357cf1644013cc94ddf194d349/crate_blue_a01/simready_usd/sm_crate_blue_a01_01.usd"
)

RESULT = [
    {
        "url": CRATE_URL,
        "score": 1.1566758,
        "root_prims": [
            {
                "usd_path": "/RootNode",
                "scaled_bbox_dimension_x": 0.6051332354545593,
                "scaled_bbox_dimension_y": 0.40513327717781067,
                "scaled_bbox_dimension_z": 0.25321948567871005,
                "properties": {
                    "simready_metadata_asset_license": "CC-BY 4.0",
                    "simready_metadata_asset_name": "sm_crate_blue_a01_01",
                    "simready_metadata_asset_type": "prop",
                    "simready_metadata_author": "NVIDIA",
                    "simready_metadata_mass": 2.836737632751465,
                    "simready_metadata_qcode": "Q37123129",
                    "simready_metadata_validation__profile": "Robotics-Prop",
                    "simready_metadata_validation__validated_features__2026-10-08__FET_003_PHYSX__passed": "True",
                    "semantic:wikidata_class:params:semanticData": "crate",
                },
            }
        ],
        "vision_generated_metadata": {
            "plugin_thumbnail_to_vision_metadata_metadata_vlm_generated": [
                {"name": "caption", "value_text": ["Blue open-top plastic crate with diamond lattice sides"]},
                {"name": "object_type", "value_text": ["plastic crate"]},
            ]
        },
        "metadata": {"modified": "2026-10-15T22:03:30.933429", "size": 23781.0},
    },
    {"url": "https://example.com/not-a-package.usd", "score": 0.5},
]


def test_search_asks_the_service_and_reads_the_hits(monkeypatch):
    asked = []

    def fake_get_json(url, timeout):
        asked.append(url)
        return RESULT

    monkeypatch.setattr(bt.simready, "_get_json", fake_get_json)
    hits = bt.simready.search(
        "blue plastic crate", limit=5, max_size=(0.7, 0.5, None), min_size=(None, None, 0.1),
        max_mass_kg=5, license="CC-BY", kind="prop", profile="Robotics-Prop", publisher="nvidia",
    )
    from urllib.parse import parse_qs, urlsplit

    (url,) = asked
    assert url.startswith(bt.simready.SEARCH_URL + "/search?")
    query = {k: v[0] for k, v in parse_qs(urlsplit(url).query).items()}
    assert query["description"] == "blue plastic crate"
    assert query["limit"] == "5"
    assert query["return_root_prims"] == "true"
    assert query["max_bbox_x"] == "0.7" and query["max_bbox_y"] == "0.5" and "max_bbox_z" not in query
    assert query["min_bbox_z"] == "0.1" and "min_bbox_x" not in query
    assert query["filter_by_properties"] == (
        "simready_metadata_asset_license=~*CC-BY*,simready_metadata_asset_type=prop,"
        "simready_metadata_validation__profile=Robotics-Prop"
    )
    assert query["filter_by_properties_numeric"] == "simready_metadata_mass<=5.0"
    assert query["filter_url_regexp"] == ".*/datasets/nvidia/.*"

    # The hit that names no package is dropped; the crate is read in full.
    (hit,) = hits
    assert hit.url == CRATE_URL and hit.source == CRATE_URL
    assert (hit.repo, hit.revision, hit.asset) == (
        "nvidia/simready-assets", "653d91e182011f357cf1644013cc94ddf194d349", "crate_blue_a01",
    )
    assert hit.path == "simready_usd/sm_crate_blue_a01_01.usd"
    assert hit.name == "sm_crate_blue_a01_01"
    assert hit.score == pytest.approx(1.1566758)
    assert hit.size_m == pytest.approx((0.6051, 0.4051, 0.2532), abs=1e-4)
    assert hit.mass_kg == pytest.approx(2.8367, abs=1e-4)
    assert hit.license == "CC-BY 4.0" and hit.publisher == "nvidia" and hit.author == "NVIDIA"
    assert hit.kind == "prop" and hit.profile == "Robotics-Prop"
    assert hit.label == "crate" and hit.qcode == "Q37123129"
    assert hit.caption == "Blue open-top plastic crate with diamond lattice sides"
    assert "simready_metadata_mass" in hit.properties
    assert not any("validated_features" in k for k in hit.properties)


def test_search_env_url_and_fetch_of_a_hit(monkeypatch):
    monkeypatch.setenv("BOTRAIL_SIMREADY_URL", "http://search.test/")
    asked = []
    monkeypatch.setattr(bt.simready, "_get_json", lambda url, timeout: asked.append(url) or RESULT)
    (hit,) = bt.simready.search("crate")
    assert asked[0].startswith("http://search.test/search?")
    with pytest.raises(ValueError, match="words"):
        bt.simready.search("   ")
    with pytest.raises(ValueError, match="positive"):
        bt.simready.search("crate", max_size=(0, None, None))

    downloaded = []
    monkeypatch.setattr(
        bt.simready, "_download",
        lambda repo, asset, revision, rest, cache_dir: downloaded.append((repo, asset, revision, rest)) or "pkg",
    )
    assert bt.simready.fetch(hit) == "pkg"
    assert downloaded == [(
        "nvidia/simready-assets", "crate_blue_a01", "653d91e182011f357cf1644013cc94ddf194d349",
        "simready_usd/sm_crate_blue_a01_01.usd",
    )]


def test_a_hit_pins_what_the_library_knows(package: Path, monkeypatch):
    monkeypatch.setattr(bt.simready, "_get_json", lambda url, timeout: RESULT)
    (hit,) = bt.simready.search("crate")
    local = bt.simready.fetch(package)
    monkeypatch.setattr(bt.simready, "_download", lambda *args: local)
    scene = bt.Scene()
    bt.parts.simready(scene, "crate", (0, 0), hit)
    (row,) = [r for r in scene.bom().rows if r["category"] == "prop"]
    assert row["attributes"]["label"] == "crate"
    assert row["attributes"]["qcode"] == "Q37123129"
    assert row["attributes"]["caption"].startswith("Blue open-top plastic crate")
    assert row["attributes"]["source"] == local.source


def test_search_the_library_for_a_cardboard_box():
    """Live: the library's search service — skipped where it cannot be reached."""
    try:
        hits = bt.simready.search("cardboard box", limit=3, license="CC-BY", kind="prop", max_size=(1.0, 1.0, 1.0))
    except ValueError as err:
        if "cannot reach" not in str(err):
            raise
        pytest.skip(f"SimReady Central unavailable: {err}")
    assert hits, "no hits"
    for hit in hits:
        assert hit.kind == "prop"
        assert hit.license and "CC-BY" in hit.license.upper()
        assert hit.size_m is not None and all(side <= 1.0 + 1e-6 for side in hit.size_m)
        assert hit.revision and hit.asset
