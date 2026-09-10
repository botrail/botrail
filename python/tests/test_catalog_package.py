"""Local builder outputs retain identity without a fabricated Hub revision."""

# ruff: noqa: S102

import json
import shutil
import sys
import types
from pathlib import Path
from xml.etree import ElementTree
from zipfile import ZipFile

import botrail as bt
import pytest
import yaml
from test_catalog import COUPLING_USD
from test_mounting import EVIDENCE, SOURCE, face

PID = "acme/coupling/local/r1"


@pytest.fixture
def package(tmp_path, monkeypatch):
    def no_network(*args, **kwargs):
        raise AssertionError("local package replay must not contact the Hub")

    monkeypatch.setitem(sys.modules, "huggingface_hub", types.SimpleNamespace(
        dataset_info=no_network, hf_hub_download=no_network, snapshot_download=no_network))
    root = tmp_path / "package"
    root.mkdir()
    (root / "shape.obj").write_text("mtllib shape.mtl\nv 0 0 0\nv 0.02 0 0\nv 0 0.02 0\nv 0 0 0.02\nf 1 3 2\nf 1 2 4\nf 2 3 4\nf 3 1 4\n")
    (root / "shape.mtl").write_text("newmtl fixture\nKd 1 0 0\nmap_Kd -s 1 1 1 textures/color.png\n")
    (root / "textures").mkdir()
    (root / "textures/color.png").write_bytes(b"texture fixture bytes")
    (root / "model.urdf").write_text('<robot name="plate"><link name="body"><visual><geometry><mesh filename="shape.obj"/></geometry></visual></link></robot>')
    (root / "model.usda").write_text(COUPLING_USD)
    manifest = {"id": PID, "kind": "model", "name": "Local fixture", "category": "adapter",
        "manufacturer": {"name": "ACME"}, "distribution": "recipe_only",
        "assets": {"urdf": "model.urdf", "usd": "model.usda"},
        "frames": {"mount_frame": "body", "flange_frame": "body", "tcp_default": "body"},
        "order": {"includes": [{"name": "fixture screw", "qty": 4}]},
        "sources": [SOURCE],
        "mounting": {"interfaces": [face("body", "mount", "fixture-face", geometry={
            "normal_z": -1, "frame_verified": True, "complete": False,
            "holes": [{"id": "m5", "position_mm": [0, 0], "kind": "threaded",
                "thread": {"diameter_mm": 5, "pitch_mm": 0.8}, "thread_start_mm": {"min": 2.9, "max": 3}}],
            "evidence": EVIDENCE})]}}
    (root / "manifest.yaml").write_text(yaml.safe_dump(manifest))
    return root


@pytest.mark.parametrize("format", [None, "urdf", "usd"])
def test_local_identity_frames_and_offline_portable_replay(package, tmp_path, format):
    robot = bt.Robot.from_package(package, format=format)
    assert robot.mount_link == ("/Plate/body" if format == "usd" else "body")
    assert robot.flange_link == robot.mount_link == robot.tcp_link
    scene = bt.Scene(robot)
    before = bt.mounting.report(scene).to_dict()
    bom = scene.bom().rows
    assert bom[0]["catalog"].startswith(PID + "@local-sha256:")
    archive = tmp_path / "portable.botrail"
    scene.save_project(archive)
    with ZipFile(archive) as zipped:
        if format != "usd":
            assert any(p.endswith("/shape.obj") for p in zipped.namelist())
            assert any(p.endswith("/shape.mtl") for p in zipped.namelist())
            assert any(p.endswith("/textures/color.png") for p in zipped.namelist())
    shutil.rmtree(package)
    restored = bt.Scene.load_project(archive)
    assert bt.mounting.report(restored).to_dict() == before
    assert restored.bom().rows == bom
    if format != "usd":
        source = json.loads(restored._project_json())["robots"][0]["source"]
        mesh_path = Path(ElementTree.fromstring(source["inner"]["xml"]).find(".//mesh").attrib["filename"])
        assert mesh_path.is_file()
        assert mesh_path.with_suffix(".mtl").is_file()
        assert (mesh_path.parent / "textures/color.png").read_bytes() == b"texture fixture bytes"
    code = restored.generate_python()
    assert "from_catalog(" not in code and "from_package(" not in code
    ns = {}
    exec(code.replace("bt.studio(scene)", ""), ns)
    assert bt.mounting.report(ns["scene"]).to_dict() == before
    assert ns["scene"].bom().rows == bom


def test_fingerprint_tracks_metadata_and_geometry_but_not_directory(package, tmp_path):
    original = bt.Scene(bt.Robot.from_package(package))
    before = bt.mounting.report(original).to_dict()
    copied = tmp_path / "copy"
    shutil.copytree(package, copied)
    assert bt.Scene(bt.Robot.from_package(copied)).bom().rows == original.bom().rows
    with (copied / "shape.obj").open("a") as stream:
        stream.write("# changed asset\n")
    assert bt.Scene(bt.Robot.from_package(copied)).bom().rows != original.bom().rows
    assert bt.mounting.report(original).to_dict() == before


def test_obstacle_materials_survive_offline_project_replay(package, tmp_path):
    scene = bt.Scene(bt.Robot.from_package(package))
    scene.add_mesh("fixture", package / "shape.obj", position=(1, 0, 0))
    archive = tmp_path / "obstacle.botrail"
    scene.save_project(archive)
    with ZipFile(archive) as zipped:
        # The robot and fixture share the mesh and its material/texture tree.
        assert len([p for p in zipped.namelist() if p.endswith("/shape.obj")]) == 1
        assert any(p.endswith("/shape.mtl") for p in zipped.namelist())
        assert any(p.endswith("/textures/color.png") for p in zipped.namelist())
    shutil.rmtree(package)
    restored = bt.Scene.load_project(archive)
    project = json.loads(restored._project_json())
    mesh = Path(next(o for o in project["obstacles"] if o["name"] == "fixture")["geometry"]["url"])
    assert mesh.is_file()
    assert "mtllib shape.mtl" in mesh.read_text()
    assert "textures/color.png" in mesh.with_suffix(".mtl").read_text()
    assert (mesh.parent / "textures/color.png").read_bytes() == b"texture fixture bytes"
    assert restored.obstacle_bounds("fixture") == scene.obstacle_bounds("fixture")


@pytest.mark.parametrize("change,match", [
    ({"assets": {}}, "ships no"),
    ({"id": "local"}, "full catalog product id"),
    ({"kind": "material"}, "model package"),
    ({"assets": {"urdf": "../outside.urdf"}}, "inside the package"),
    ({"assets": {"urdf": "/tmp/outside.urdf"}}, "package-relative"),
    ({"mounting": {"interfaces": [face("missing", "mount", "fixture-face")]}}, "mounting frame"),
])
def test_local_manifest_errors_are_not_silently_ignored(package, change, match):
    path = package / "manifest.yaml"
    manifest = yaml.safe_load(path.read_text())
    manifest.update(change)
    path.write_text(yaml.safe_dump(manifest))
    with pytest.raises(ValueError, match=match):
        bt.Robot.from_package(package)


def test_reject_symlinks_and_bad_replay_mounting(package):
    (package / "shortcut.obj").symlink_to(package / "shape.obj")
    with pytest.raises(ValueError, match="symlinks"):
        bt.Robot.from_package(package)
    source = {"kind": "catalog", "id": PID, "revision": "local-sha256:test",
        "mounting": {"interfaces": [face("missing", "mount", "fixture-face")]},
        "sources": [SOURCE], "inner": {"kind": "urdf", "xml": '<robot name="r"><link name="body"/></robot>'}}
    with pytest.raises(ValueError, match="mounting frame"):
        bt.Robot._from_source_json(json.dumps(source))


def test_missing_urdf_asset_prevents_false_portable_save(package, tmp_path):
    scene = bt.Scene(bt.Robot.from_package(package))
    (package / "shape.obj").unlink()
    with pytest.raises(ValueError, match="shape.obj"):
        scene.save_project(tmp_path / "missing.botrail")
