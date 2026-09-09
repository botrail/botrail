"""Synthetic complete conditions exercise the public APIs, not hardware fit."""
# ruff: noqa: F811

import json
import shutil

import botrail as bt
import pytest
import yaml
from test_catalog_kits import ARM, BASE, KIT, load, packages  # noqa: F401

VALUES = {
    "wrist_connector": "m8_8pin_male", "pinout": "drawing-A", "voltage_v": 24,
    "current_a": 2, "peak_current_a": 3, "protocol": "modbus_rtu_rs485",
    "controller": "controller-A", "software_family": "polyscope_5",
    "software_version": "5.1.0", "plugin": "grippers", "plugin_version": "1.3.1",
}


@pytest.fixture
def connected(packages):
    root, data = packages
    evidence = [{"source": 0, "section": "Synthetic configuration table"}]
    conditions = []
    for field, value in VALUES.items():
        c = {"field": field, "evidence": evidence}
        if isinstance(value, str):
            c["accepted"] = [value]
        else:
            c["minimum"] = 22 if field == "voltage_v" else 1
            if field == "voltage_v":
                c["maximum"] = 26
        conditions.append(c)
    data["compatibility"] = {"mounts": ["old-hint"], "model_fidelity": {"status": "reference"},
        "connections": [{"id": "host-a", "host": ARM, "part_number": "HOST-KIT", "support": "supported",
                         "evidence": evidence, "conditions": conditions, "note": "Synthetic fixture"}]}
    data["electrical"] = {"bus": ["modbus_rtu_rs485"], "connector": {"pinout": "drawing-A"}}
    (root / KIT / "manifest.yaml").write_text(yaml.safe_dump(data))
    return root, data


def scene_from(root, **kw):
    scene = bt.Scene()
    scene.add_robot(load(root, ARM, **kw).attach_tool(load(root, **kw), prefix="kit_"), name="robot")
    return scene


def selection(scene, values=None):
    bt.connections.configure(scene, "robot/tool", "host-a", values=VALUES if values is None else values,
                             reference="Commissioning drawing A")
    return bt.mounting.report(scene).kits[0]["connection"]


def test_conditions_are_independent_of_mechanical_and_simulation_results(connected):
    root, data = connected
    scene = scene_from(root)
    before = bt.mounting.report(scene)
    assert before.kits[0]["connection"]["configuration"]["status"] == "unknown"
    result = selection(scene)
    assert [result[k]["status"] for k in ("configuration", "electrical", "communication", "software")] == ["pass"] * 4
    after = bt.mounting.report(scene)
    assert after.simulation == before.simulation
    assert after.kits[0]["detailed_fit"] == before.kits[0]["detailed_fit"]
    assert after.kits[0]["model_correspondence"]["status"] == "unknown"
    direct = bt.connections.evaluate(data, ARM, profile="host-a", values=VALUES)
    assert direct["electrical"] == result["electrical"]
    assert "Declared connection conditions" in after.to_markdown()
    report = bt.connections.report(scene)
    assert report.configurations[0]["software"]["status"] == "pass"
    assert "software_version" in report.to_markdown()


@pytest.mark.parametrize("field,value,scope,status", [
    ("wrist_connector", "m8_8pin_female", "electrical", "fail"),
    ("pinout", "drawing-B", "electrical", "fail"),
    ("voltage_v", 27, "electrical", "fail"),
    ("current_a", .5, "electrical", "fail"),
    ("peak_current_a", .5, "electrical", "fail"),
    ("protocol", "modbus_tcp", "communication", "fail"),
    ("software_family", "polyscope_x", "software", "fail"),
    ("software_version", None, "software", "unknown"),
    ("software_version", "5.1.1", "software", "unknown"),
    ("plugin_version", "2.0.0", "software", "unknown"),
    ("plugin_version", None, "software", "unknown"),
])
def test_exact_conditions_and_missing_values(connected, field, value, scope, status):
    root, _ = connected
    result = selection(scene_from(root), {**VALUES, field: value})
    assert result[scope]["status"] == status
    assert result["configuration"]["status"] == "pass"


def test_missing_documentary_condition_cannot_be_filled_in_by_user(connected):
    _, data = connected
    p = data["compatibility"]["connections"][0]
    p["conditions"] = [c for c in p["conditions"] if c["field"] != "current_a"]
    p["conditions"][-1]["accepted"] = []
    result = bt.connections.evaluate(data, ARM, profile="host-a", values=VALUES)
    assert result["electrical"]["status"] == result["software"]["status"] == "unknown"


def test_wrong_host_absence_and_explicit_rejection_differ(connected):
    _, data = connected
    result = bt.connections.evaluate(data, BASE, profile="host-a", values=VALUES)
    assert result["configuration"]["status"] == "fail"
    assert result["electrical"]["status"] == "not_applicable"
    result = bt.connections.evaluate(data, BASE)
    assert result["configuration"]["status"] == "unknown"
    data["compatibility"]["connections"][0]["support"] = "unsupported"
    result = bt.connections.evaluate(data, ARM, profile="host-a", values=VALUES)
    assert result["configuration"]["status"] == "fail"
    assert result["manufacturer_support"] == "unsupported"


@pytest.mark.parametrize("format", ["urdf", "usd"])
def test_embedded_metadata_selection_offline_save_and_python_replay(connected, tmp_path, format):
    root, data = connected
    scene = scene_from(root, format=format)
    before = selection(scene)
    path = tmp_path / "connection.botrail"
    scene.save_project(path)
    shutil.rmtree(root)
    restored = bt.Scene.load_project(path)
    assert bt.mounting.report(restored).kits[0]["connection"] == before
    source = json.loads(restored._project_json())["robots"][0]["source"]["tool"]
    assert source["compatibility"]["model_fidelity"] == {"status": "reference"}
    assert source["compatibility"]["mounts"] == ["old-hint"]
    assert source["electrical"] == data["electrical"]
    namespace = {}
    exec(restored.generate_python().replace("bt.studio(scene)", ""), namespace)  # noqa: S102
    assert bt.mounting.report(namespace["scene"]).kits[0]["connection"] == before


def test_stale_identity_selection_and_deleted_kit_remain_visible(connected):
    root, _ = connected
    scene = scene_from(root)
    selection(scene)
    plan = json.loads(scene._connection_plan_json())
    plan["configurations"][0]["host"] = BASE
    bt.connections.restore(scene, plan)
    assert bt.mounting.report(scene).kits[0]["connection"]["configuration"]["status"] == "fail"
    plan["configurations"][0]["target"] = "deleted/tool"
    bt.connections.restore(scene, plan)
    assert any(c["status"] == "fail" and c["target"] == "deleted/tool" for c in bt.connections.report(scene).checks)


def test_old_project_defaults_to_unknown_and_retains_old_schema(packages):
    root, _ = packages
    scene = scene_from(root)
    source = json.loads(scene._project_json())["robots"][0]["source"]
    def legacy(node):
        node.pop("compatibility", None)
        node.pop("electrical", None)
        for key in ("base", "tool", "inner"):
            if key in node:
                legacy(node[key])
    legacy(source)
    restored = bt.Scene(bt.Robot._from_source_json(json.dumps(source)))
    assert bt.mounting.report(restored).kits[0]["connection"]["electrical"]["status"] == "unknown"
    assert bt.mounting.report(restored).kits[0]["manufacturer_support"]["status"] == "pass"


@pytest.mark.parametrize("value", [True, -1, float("nan"), "24"])
def test_invalid_capacity_is_rejected(connected, value):
    root, _ = connected
    with pytest.raises(ValueError):
        selection(scene_from(root), {**VALUES, "current_a": value})


def test_invalid_catalog_evidence_rejected_on_import_and_evaluation(connected):
    root, data = connected
    data["compatibility"]["connections"][0]["conditions"][0]["evidence"][0]["source"] = 100
    (root / KIT / "manifest.yaml").write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError, match="evidence"):
        load(root)
    with pytest.raises(ValueError, match="evidence"):
        bt.connections.evaluate(data, ARM, profile="host-a", values=VALUES)


def test_catalog_search_keeps_unknown_entries_and_finds_host_sku_program():
    from botrail.catalog import Index

    index = Index.from_dict({"products": [
        {"id": KIT, "name": "Kit", "compatibility": {
            "programs": [{"name": "ur_plus", "status": "listed"}],
            "connections": [{"id": "host-a", "host": ARM, "part_number": "HRC-03-118505", "support": "supported"}]}},
        {"id": BASE, "name": "Legacy"},
    ]})
    assert len(index.search()) == 2
    for query in (ARM, "118505", "ur_plus"):
        assert [p.id for p in index.search(text=query)] == [KIT]
    assert index.products[0].to_dict()["compatibility"]["connections"][0]["host"] == ARM


def test_renaming_preserves_selection_and_bom_pin_invalidates_it(connected):
    root, _ = connected
    scene = scene_from(root)
    selection(scene)
    scene.rename_robot("robot", "arm")
    assert bt.mounting.report(scene).kits[0]["connection"]["configuration"]["status"] == "pass"
    scene.set_part("arm", catalog=BASE)
    assert bt.mounting.report(scene).kits[0]["connection"]["configuration"]["status"] == "fail"


def test_public_source_with_only_legacy_hints_replays_its_snapshot_offline(packages, monkeypatch):
    import sys
    import types

    root, _ = packages
    scene = bt.Scene(load(root, ARM))
    source = json.loads(scene._project_json())["robots"][0]["source"]
    source["revision"] = "a" * 40
    source["compatibility"] = {"mounts": ["legacy-hint"], "vendor_extension": {"status": "unconfirmed"}}
    scene = bt.Scene(bt.Robot._from_source_json(json.dumps(source)))
    monkeypatch.setitem(sys.modules, "huggingface_hub", types.SimpleNamespace())
    namespace = {}
    exec(scene.generate_python().replace("bt.studio(scene)", ""), namespace)  # noqa: S102
    restored = json.loads(namespace["scene"]._project_json())["robots"][0]["source"]
    assert restored["compatibility"]["vendor_extension"] == {"status": "unconfirmed"}
    assert restored["compatibility"]["mounts"] == ["legacy-hint"]
