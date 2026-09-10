"""Synthetic catalog kits test purchase/assembly semantics, not hardware fit."""
# ruff: noqa: S102
import copy
import json
import shutil
import sys
import types

import botrail as bt
import pytest
import yaml
from test_catalog import COUPLING_USD
from test_mounting import EVIDENCE, SOURCE, face

ARM = "acme/arm/host/r1"
BASE = "acme/adapter/coupling/r1"
TOOL = "acme/gripper/hand/r1"
KIT = "acme/gripper/host-kit/r1"
SHA = "1234567890abcdef"


def write_manifest(root, pid, data):
    p = root / pid
    p.mkdir(parents=True, exist_ok=True)
    (p / "manifest.yaml").write_text(yaml.safe_dump(data))
    return p


@pytest.fixture
def packages(tmp_path):
    root = tmp_path / "catalog"
    for pid, role, interface in [(ARM, "flange", "robot-face"), (BASE, "mount", "robot-face"), (TOOL, "mount", "tool-face")]:
        interfaces = [face("body", role, interface)]
        if pid == BASE:
            interfaces.append(face("body", "flange", "tool-face"))
        data = {"id": pid, "kind": "model", "name": pid.split("/")[-2], "category": "adapter" if pid == BASE else "manipulator" if pid == ARM else "gripper.parallel",
                    "manufacturer": {"name": "ACME"}, "sources": [SOURCE], "assets": {"urdf": "model.urdf", "usd": "model.usda"},
                    "frames": {"mount_frame": "body", "flange_frame": "body", "tcp_default": "body"}, "mounting": {"interfaces": interfaces}}
        p = write_manifest(root, pid, data)
        (p / "model.urdf").write_text('<robot name="robot"><link name="body"><visual><geometry><box size=".01 .01 .01"/></geometry></visual></link></robot>')
        (p / "model.usda").write_text(COUPLING_USD)
    data = {"id": KIT, "kind": "kit", "name": "Host kit", "category": "gripper.parallel", "manufacturer": {"name": "ACME"}, "sources": [SOURCE], "assets": {},
                "frames": {"mount_frame": "body", "tcp_default": "gripper/body"},
                "order": {"unit": "kit", "part_number": "HOST-KIT", "includes": [
                    {"name": "Coupling", "catalog": BASE, "qty": 1}, {"name": "Hand", "catalog": TOOL, "qty": 1}, {"name": "Screw set", "qty": 1}]},
                "kit": {"base": BASE, "attachments": [{"package": TOOL, "alias": "gripper", "parent_frame": "body", "mount_frame": "body"}],
                     "manufacturer_support": [{"host": ARM, "scope": "mechanical_mounting", "evidence": copy.deepcopy(EVIDENCE)}],
                     "representation": {"status": "reference", "note": "Synthetic geometry is not a verified product model."}}}
    write_manifest(root, KIT, data)
    return root, data


def load(root, pid=KIT, **kwargs):
    return bt.Robot.from_package(root / pid, catalog_root=root, **kwargs)


def assessment(robot):
    return bt.mounting.report(robot).kits[0]


def test_kit_purchase_unit_preserves_components_and_scoped_results(packages):
    root, _ = packages
    robot = load(root, ARM).attach_tool(load(root), prefix="kit_")
    scene = bt.Scene(robot)
    bom = scene.bom().rows
    # The arm, the kit, and the controller the arm needs.
    assert len(bom) == 3 and bom[1]["qty"] == 1
    assert bom[2]["names"] == ["robot/controller"]
    assert bom[1]["order"]["unit"] == "kit"
    assert [i.get("catalog") for i in bom[1]["order"]["includes"]] == [BASE, TOOL, None]
    report = bt.mounting.report(scene)
    kit = report.kits[0]
    assert kit["manufacturer_support"]["status"] == "pass"
    assert kit["composition"]["status"] == "pass"
    assert report.ready
    assert len(report.assemblies) == 2  # The internal weld is still checked.
    two = bt.Scene()
    two.add_robot(robot, name="one")
    two.add_robot(robot, name="two")
    row = next(r for r in two.bom().rows if r["catalog"].startswith(KIT + "@"))
    assert row["qty"] == 2 and row["order"]["includes"][0]["qty"] == 1  # per kit


def test_wrong_host_is_unknown_not_incompatible(packages):
    root, _ = packages
    kit = assessment(load(root, BASE).attach_tool(load(root), prefix="kit_"))
    assert kit["manufacturer_support"]["status"] == "unknown"
    assert kit["composition"]["status"] == "pass"


@pytest.mark.parametrize("mutation", ["component", "offset"])
def test_changed_assembly_does_not_pass(packages, mutation):
    root, _ = packages
    scene = bt.Scene(load(root, ARM).attach_tool(load(root), prefix="kit_"))
    source = json.loads(scene._project_json())["robots"][0]["source"]
    inside = source["tool"]["inner"]
    if mutation == "component":
        inside["tool"]["id"] = BASE
    else:
        inside["offset"]["position"][2] = .123
    scene = bt.Scene(bt.Robot._from_source_json(json.dumps(source)))
    report = bt.mounting.report(scene)
    assert report.kits[0]["composition"]["status"] == "fail"
    assert report.kits[0]["manufacturer_support"]["status"] == "unknown"
    assert not report.ready


def test_kit_revision_includes_component_bytes_and_is_relocatable(packages, tmp_path):
    root, _ = packages
    before = bt.Scene(load(root)).bom().rows
    copied = tmp_path / "copy"
    shutil.copytree(root, copied)
    assert bt.Scene(load(copied)).bom().rows == before
    p = copied / TOOL / "model.urdf"
    p.write_text(p.read_text().replace('.01 .01 .01', '.02 .01 .01'))
    assert bt.Scene(load(copied)).bom().rows != before


@pytest.mark.parametrize("format", [None, "usd"])
def test_offline_kit_project_and_script_replay(packages, tmp_path, monkeypatch, format):
    root, _ = packages
    scene = bt.Scene(load(root, ARM, format=format).attach_tool(load(root, format=format), prefix="kit_"))
    before, bom = bt.mounting.report(scene).to_dict(), scene.bom().rows
    assert before["kits"][0]["composition"]["status"] == "pass"
    assert scene.robot.tcp_link.endswith("body")
    project = tmp_path / "kit.botrail"
    scene.save_project(project)
    shutil.rmtree(root)
    monkeypatch.setitem(sys.modules, "huggingface_hub", types.SimpleNamespace())
    restored = bt.Scene.load_project(project)
    assert bt.mounting.report(restored).to_dict() == before
    assert restored.bom().rows == bom
    ns = {}
    exec(restored.generate_python().replace("bt.studio(scene)", ""), ns)
    assert bt.mounting.report(ns["scene"]).to_dict() == before
    assert ns["scene"].bom().rows == bom


@pytest.mark.parametrize("change,match", [
    (lambda d: d.update(mounting={"interfaces": [face("body", "mount", "fake")]}), "components"),
    (lambda d: d["order"]["includes"].pop(0), "quantities"),
    (lambda d: d["order"].update(part_number=""), "part_number"),
    (lambda d: d["kit"]["manufacturer_support"][0].update(evidence=[]), "evidence"),
    (lambda d: d["kit"]["manufacturer_support"][0]["evidence"][0].update(source=99), "evidence"),
    (lambda d: d["kit"]["attachments"][0].update(package="../../bad/r1"), "attachment"),
    (lambda d: d["kit"]["attachments"][0].update(offset={"xyz": [float("nan"),0,0]}), "kit"),
])
def test_invalid_kit_contracts_rejected(packages, change, match):
    root, data = packages
    change(data)
    write_manifest(root, KIT, data)
    with pytest.raises(ValueError, match=match):
        load(root)


def test_missing_and_mislabeled_dependencies(packages):
    root, _ = packages
    p = root / TOOL / "manifest.yaml"
    data = yaml.safe_load(p.read_text())
    data["id"] = BASE
    p.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="identity mismatch"):
        load(root)
    p.unlink()
    with pytest.raises((ValueError, FileNotFoundError)):
        load(root)


def test_recursive_kit_rejected(packages):
    root, data = packages
    data["kit"]["base"] = KIT
    data["order"]["includes"][0]["catalog"] = KIT
    write_manifest(root, KIT, data)
    with pytest.raises(ValueError, match="cyclic"):
        load(root)


@pytest.mark.parametrize("tool_public", [True, False])
def test_hub_kit_dependencies_use_one_revision(packages, monkeypatch, tool_public):
    root, _ = packages
    entries = []
    for pid in [KIT, BASE, TOOL]:
        data = yaml.safe_load((root / pid / "manifest.yaml").read_text())
        entries.append({"id": pid, "distribution": "public", "kind": data["kind"], "assets": {k: f"{pid}/{v}" for k,v in data["assets"].items()}})
    if not tool_public:
        # A public replacement must not change the kit's exact component ID.
        entries[-1]["distribution"] = "recipe_only"
        entries.append(dict(entries[-1], id=TOOL.replace("/r1", "/r2"), distribution="public"))
    (root / "index.json").write_text(json.dumps({"products": entries}))
    revisions = []
    def info(*args, revision=None, **kw):
        revisions.append(revision)
        return types.SimpleNamespace(sha=SHA)
    def snapshot(*args, revision=None, **kw):
        assert revision == SHA
        return str(root)
    monkeypatch.setitem(sys.modules, "huggingface_hub", types.SimpleNamespace(
        dataset_info=info, hf_hub_download=lambda *a, **kw: str(root / "index.json"), snapshot_download=snapshot))
    if not tool_public:
        with pytest.raises(ValueError, match=f"{TOOL}.*recipe_only"):
            bt.Robot.from_catalog(KIT)
        return
    kit = bt.Robot.from_catalog(KIT)
    assert revisions == [None, SHA, SHA]
    assert assessment(kit)["composition"]["status"] == "pass"
    assert bt.Scene(kit).bom().rows[0]["catalog"] == f"{KIT}@{SHA}"


def test_search_preserves_kit_purchase_metadata(packages):
    _, data = packages
    index = bt.catalog.Index.from_dict({'products': [data]})
    product, = index.search('gripper', kind='kit')
    assert product.order['part_number'] == 'HOST-KIT'
    assert product.kit['base'] == BASE
    assert product.to_dict()['order'] == data['order']


def test_a_kit_attaches_under_kit_by_default(packages):
    """A kit's coupling exposes `flange` like the arm; without a prefix the
    kit lands under `kit_`, the same names the explicit form gives."""
    root, _ = packages
    implicit = load(root, ARM).attach_tool(load(root))
    explicit = load(root, ARM).attach_tool(load(root), prefix="kit_")
    assert implicit.link_names == explicit.link_names
    assert implicit.tcp_link == explicit.tcp_link and implicit.tcp_link.startswith("kit_")
    scene = bt.Scene(implicit)
    assert bt.mounting.report(scene).to_dict() == bt.mounting.report(bt.Scene(explicit)).to_dict()
    assert "prefix=\"kit_\"" in scene.generate_python()
