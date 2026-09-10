"""Mounting contract tests. Geometry and ACME declarations are synthetic.

Real-product metadata cases are separate fixtures; these tests do not certify
the physical fit of the represented hardware.
"""

# Generated-script execution is the contract under test; all inputs are local fixtures.
# ruff: noqa: S102

import copy
import json
import sys
import types
from pathlib import Path

import botrail as bt
import pytest
import yaml

SHA = "0123abcd0123abcd0123abcd0123abcd0123abcd"
ARM = "acme/arm/test/r1"
ADAPTER = "acme/adapter/test/r1"
TOOL = "acme/tool/test/r1"
SOURCE = {"kind": "manufacturer_datasheet", "url": "https://example.com/test-drawing",
          "ref": "fixture-1", "fetched_at": "2026-09-06"}
EVIDENCE = [{"source": 0, "section": "Synthetic test drawing A"}]
IDENTITY = {"position": [0, 0, 0], "quaternion": [0, 0, 0, 1]}
REQUIREMENT = {"id": "adapter", "frame": "tool_root", "order_requires": 0,
               "evidence": EVIDENCE, "note": "The specified adapter is required"}
ORDER = {"requires": [{"catalog": ADAPTER, "category": "adapter", "qty": 1}]}


def face(frame, role, interface_id, **kwargs):
    return {"frame": frame, "role": role, "interface_id": interface_id,
            "evidence": copy.deepcopy(EVIDENCE), **kwargs}


@pytest.fixture
def catalog(tmp_path, monkeypatch):
    """A local Hub transport exercising the real catalog loader, not a mock Robot."""
    root = tmp_path / "catalog"
    root.mkdir()
    products = []

    def add(pid, *, category="tool.spindle", mount="mount", flange=None, mounting=None,
            order=None, sources=None, extra_frames=(), raw=None):
        data = {"id": pid, "category": category, "name": pid.split("/")[-2],
                "manufacturer": {"name": "ACME"}, "frames": {"mount_frame": mount,
                "flange_frame": flange, "tcp_default": flange or mount},
                "mounting": mounting, "order": order,
                "sources": copy.deepcopy([SOURCE] if sources is None else sources)}
        if raw is not None:
            data = copy.deepcopy(raw)
            mount = data["frames"].get("mount_frame") or "base"
            flange = data["frames"].get("flange_frame")
        names = list(dict.fromkeys([mount, *filter(None, [flange, data["frames"].get("tcp_default")]),
                                   *extra_frames]))
        xml = '<robot name="robot">' + "".join(f'<link name="{n}"/>' for n in names)
        xml += "".join(f'<joint name="fixed{i}" type="fixed"><parent link="{mount}"/>'
                       f'<child link="{n}"/><origin xyz="0 0 0.05"/></joint>'
                       for i, n in enumerate(names[1:])) + "</robot>"
        package = root / pid
        package.mkdir(parents=True, exist_ok=True)
        (package / "robot.urdf").write_text(xml)
        (package / "manifest.yaml").write_text(yaml.safe_dump(data))
        if not any(p["id"] == pid for p in products):
            products.append({"id": pid, "distribution": "public", "assets": {"urdf": f"{pid}/robot.urdf"}})
        (root / "index.json").write_text(json.dumps({"products": products}))
        return package

    fake = types.ModuleType("huggingface_hub")
    fake.dataset_info = lambda *a, **kw: types.SimpleNamespace(sha=SHA)
    fake.hf_hub_download = lambda *a, filename=None, **kw: str(root / filename)
    fake.snapshot_download = lambda *a, **kw: str(root)
    monkeypatch.setitem(sys.modules, "huggingface_hub", fake)
    add(ARM, category="manipulator", mount="base", flange="left", extra_frames=["right"],
        mounting={"interfaces": [face("left", "flange", "robot-face"), face("right", "flange", "robot-face")]})
    add(ADAPTER, category="adapter", flange="out", mounting={"interfaces": [
        face("mount", "mount", "robot-face", allowed_poses=[IDENTITY]), face("out", "flange", "tool-face")]})
    add(TOOL, mount="tool_root", mounting={"interfaces": [
        face("tool_root", "mount", "tool-face", allowed_poses=[IDENTITY])],
        "requirements": [copy.deepcopy(REQUIREMENT)]}, order=copy.deepcopy(ORDER))
    return add


def load(pid):
    return bt.Robot.from_catalog(pid, revision=SHA)


def item(report, suffix, target=None):
    return next(i for i in report.items if i.id.endswith(":" + suffix) and (target is None or i.target == target))


def complete():
    return load(ARM).attach_tool(load(ADAPTER), prefix="a_").attach_tool(load(TOOL))


@pytest.mark.parametrize("offset", [(0, 0, 0), (0, 0, 0.0139)])
def test_missing_adapter_cannot_be_replaced_with_an_offset(catalog, offset):
    robot = load(ARM).attach_tool(load(TOOL), offset_position=offset)
    report = bt.mounting.report(robot)
    requirement = item(report, "required:adapter")
    assert requirement.status == "fail"
    assert "not on the attachment path" in requirement.message
    assert "The specified adapter is required" in requirement.message
    assert not report.ready and report.blockers() == [requirement]
    # The static check repeats a missing required part as a warning, never an error.
    static = bt.Scene(robot).check()
    assert static.ok
    assert [(f.code, f.target) for f in static.findings if f.code == "required_part_missing"] == [("required_part_missing", "robot/tool")]


def test_actual_adapter_path_passes(catalog):
    report = bt.mounting.report(complete())
    assert [i.key for i in report.items] == ["required:adapter"]
    assert item(report, "required:adapter").status == "pass"
    assert report.ready
    assert report.assemblies[-1]["upstream_parts"] == ["robot/tool", "robot"]
    assert "Result: ready" in report.to_markdown()


def test_adapter_on_another_branch_or_another_robot_does_not_count(catalog):
    robot = load(ARM).attach_tool(load(ADAPTER), flange="right", prefix="other_")
    robot = robot.attach_tool(load(TOOL), flange="left")
    report = bt.mounting.report(robot)
    assert item(report, "required:adapter").status == "fail"
    assert report.assemblies[-1]["upstream_parts"] == ["robot"]
    scene = bt.Scene(load(ARM).attach_tool(load(TOOL)))
    scene.add_robot(load(ARM).attach_tool(load(ADAPTER), prefix="spare_"), name="spare")
    assert item(bt.mounting.report(scene), "required:adapter").status == "fail"


def test_preassembled_stack_preserves_original_frames_and_bom_names(catalog):
    stack = load(ADAPTER).attach_tool(load(TOOL), prefix="g_")
    robot = load(ARM).attach_tool(stack, prefix="stack_")
    scene = bt.Scene(robot)
    report = bt.mounting.report(scene)
    requirement = item(report, "required:adapter")
    assert requirement.status == "pass"
    names = [n for row in scene.bom().rows for n in row["names"]]
    assert requirement.target in names
    assert requirement.evidence["found"] == ["robot/tool"]


@pytest.mark.parametrize("sources", [[], [{**SOURCE, "kind": "community"}]])
def test_missing_or_community_evidence_leaves_the_requirement_unknown(catalog, sources):
    declaration = face("tool_root", "mount", "tool-face")
    requirement = copy.deepcopy(REQUIREMENT)
    if not sources:
        declaration["evidence"] = []
        requirement["evidence"] = []
    catalog(TOOL, mount="tool_root", mounting={"interfaces": [declaration], "requirements": [requirement]},
            order=copy.deepcopy(ORDER), sources=sources)
    report = bt.mounting.report(complete())
    assert item(report, "required:adapter").status == "unknown"
    assert "could not be established" in item(report, "required:adapter").message
    assert not report.ready


def test_saved_project_and_generated_python_preserve_snapshot(catalog, tmp_path):
    # Directional marks copied from PDF manuals must survive Python quoting.
    package = catalog("acme/mark/fixture/r1", sources=[{**SOURCE, "ref": "資料‎版"}])
    source_path = package.parents[3] / TOOL / "manifest.yaml"
    source = yaml.safe_load(source_path.read_text())
    source["sources"][0]["ref"] = "資料‎版"
    source_path.write_text(yaml.safe_dump(source, allow_unicode=True))
    scene = bt.Scene(complete())
    before = bt.mounting.report(scene).to_dict()
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert bt.mounting.report(reloaded).to_dict() == before
    # Even if a transport incorrectly serves changed metadata at the same SHA,
    # the saved script restores its recorded mounting inputs after the fetch.
    catalog(TOOL, mount="tool_root")
    namespace = {}
    exec("\n".join(l for l in reloaded.generate_python().splitlines() if l != "bt.studio(scene)"), namespace)
    assert bt.mounting.report(namespace["scene"]).to_dict() == before
    assert bt.mounting.report(scene._snapshot()).to_dict() == before
    report = bt.mounting.report(reloaded)
    report.save(tmp_path / "mounting.json")
    report.save(tmp_path / "mounting.md")
    assert json.loads((tmp_path / "mounting.json").read_text()) == before
    assert "| required:adapter | pass |" in (tmp_path / "mounting.md").read_text()


def test_part_relabel_cannot_satisfy_a_requirement(catalog):
    scene = bt.Scene(load(ARM).attach_tool(load(TOOL)))
    scene.set_part("robot", catalog=ADAPTER)
    assert item(bt.mounting.report(scene), "required:adapter").status == "fail"


def test_products_without_declarations_have_nothing_to_report(catalog):
    catalog(TOOL, mount="tool_root")
    report = bt.mounting.report(complete())
    assert report.items == [] and report.ready
    assert len(report.assemblies) == 2
    empty = bt.mounting.report(bt.Scene())
    assert empty.items == [] and empty.assemblies == [] and empty.ready
    assert "No tool attachments" in empty.to_markdown()
    with pytest.raises(TypeError):
        bt.mounting.report("robot")


@pytest.mark.parametrize("mutate, message", [
    (lambda m: m["interfaces"][0]["evidence"][0].update(source=5), "sources index"),
    (lambda m: m["interfaces"][0].update(frame="absent"), "does not exist"),
    (lambda m: m["interfaces"][0].update(allowed_poses=[{**IDENTITY, "quaternion": [0, 0, 0, 2]}]), "unit XYZW"),
    (lambda m: m.update(requirements=[{"id": "r", "frame": "tool_root", "order_requires": 9, "note": "needed"}]), "order_requires"),
])
def test_malformed_declarations_are_errors_not_silently_dropped(catalog, mutate, message):
    spec = {"interfaces": [face("tool_root", "mount", "tool-face")]}
    mutate(spec)
    catalog(TOOL, mount="tool_root", mounting=spec)
    with pytest.raises(ValueError, match=message):
        load(TOOL)


@pytest.mark.parametrize("arm_recipe, tool_recipe, expected", [
    ("universal_robots/ur5e-rosi.yaml", "robotiq/2f-85.yaml", "fail"),
    ("mitsubishi_electric/rv-5as-d.yaml", "botrail/spindle-emsf3060.yaml", "unknown"),
    ("fanuc/r2000ic-165f-official.yaml", "botrail/weld-gun-x1-r4.yaml", "unknown"),
])
def test_three_product_families_report_their_required_parts_and_preserve_them(
    catalog, tmp_path, arm_recipe, tool_recipe, expected,
):
    data = json.loads((Path(__file__).parent / "data/mounting/products.json").read_text())
    products = {p["recipe"]: p["manifest"] for p in data["products"]}
    for raw in products.values():
        catalog(raw["id"], raw=raw)
    arm, tool = load(products[arm_recipe]["id"]), load(products[tool_recipe]["id"])
    scene = bt.Scene(arm.attach_tool(tool, prefix="ee_"))
    report = bt.mounting.report(scene)
    required = [i for i in report.items if ":required:" in i.id]
    assert len(required) == 1 and required[0].status == expected
    assert required[0].target == "robot/tool"
    assert required[0].next_action and required[0].message
    assert not report.ready
    path = tmp_path / "product.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert bt.mounting.report(reloaded).to_dict() == report.to_dict()
    namespace = {}
    exec("\n".join(l for l in reloaded.generate_python().splitlines() if l != "bt.studio(scene)"), namespace)
    assert bt.mounting.report(namespace["scene"]).to_dict() == report.to_dict()
    if tool_recipe == "robotiq/2f-85.yaml":
        coupling = load(products["robotiq/gripper-coupling.yaml"]["id"])
        assembled = bt.mounting.report(arm.attach_tool(coupling, prefix="cpl_").attach_tool(tool, prefix="g_"))
        assert item(assembled, "required:gripper-coupling").status == "pass"
        assert assembled.ready
        iso50 = load(products["robotiq/agc-cpl-062-002.yaml"]["id"])
        candidate = bt.mounting.report(arm.attach_tool(iso50, prefix="cpl_").attach_tool(tool, prefix="g_"))
        assert item(candidate, "required:gripper-coupling").status == "pass"


def test_weld_gun_bracket_requirement_names_the_missing_flange_option(catalog):
    data = json.loads((Path(__file__).parent / "data/mounting/products.json").read_text())
    products = {p["recipe"]: p["manifest"] for p in data["products"]}
    arm = products["fanuc/r2000ic-165f-official.yaml"]
    gun = products["botrail/weld-gun-x1-r4.yaml"]
    for raw in (arm, gun):
        catalog(raw["id"], raw=raw)
    robot = load(arm["id"]).attach_tool(load(gun["id"]), prefix="ee_", offset_quaternion=(0, 0, 1, 0))
    report = bt.mounting.report(robot)
    assert "robot flange option" in item(report, "required:robot-bracket").message
    assert not report.kits and not report.ready


def test_old_project_does_not_acquire_missing_mounting_data_on_script_replay(catalog, tmp_path):
    data = json.loads(bt.Scene(complete())._project_json())

    def strip(value):
        if isinstance(value, dict):
            if value.get("kind") == "catalog":
                for key in ("mounting", "order", "sources"):
                    value.pop(key, None)
            for child in value.values():
                strip(child)
        elif isinstance(value, list):
            for child in value:
                strip(child)

    strip(data)
    path = tmp_path / "old.botrail"
    path.write_text(json.dumps(data))
    old = bt.Scene.load_project(path)
    before = bt.mounting.report(old).to_dict()
    assert before["items"] == []
    namespace = {}
    exec("\n".join(l for l in old.generate_python().splitlines() if l != "bt.studio(scene)"), namespace)
    assert bt.mounting.report(namespace["scene"]).to_dict() == before


def test_usd_leaf_frames_are_resolved_and_preserved(catalog, tmp_path):
    from test_catalog import COUPLING_USD

    package = catalog(ADAPTER, category="adapter", mount="body", flange="body", mounting={"interfaces": [
        face("body", "mount", "robot-face"), face("body", "flange", "tool-face")]})
    (package / "model.usda").write_text(COUPLING_USD)
    index_path = package.parents[3] / "index.json"
    index = json.loads(index_path.read_text())
    next(p for p in index["products"] if p["id"] == ADAPTER)["assets"] = {"usd": f"{ADAPTER}/model.usda"}
    index_path.write_text(json.dumps(index))
    scene = bt.Scene(complete())
    report = bt.mounting.report(scene)
    assert item(report, "required:adapter").status == "pass"
    assert report.assemblies[0]["mount"].endswith("/body")
    path = tmp_path / "usd.botrail"
    scene.save_project(path)
    assert bt.mounting.report(bt.Scene.load_project(path)).to_dict() == report.to_dict()


def test_dual_arm_groups_keep_required_parts_on_the_selected_arm(catalog):
    package = catalog(ARM, category="manipulator", mount="base", flange="left", mounting={
        "interfaces": [face("left", "flange", "robot-face")]})
    path = package / "robot.urdf"
    path.write_text(path.read_text().replace('type="fixed"', 'type="revolute"', 1).replace(
        "</joint>", '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint>', 1))
    dual = bt.Robot.dual_arm(load(ARM), load(ARM))
    with_adapter = dual.attach_tool(load(ADAPTER), group="left", prefix="adapter_")
    wrong = with_adapter.attach_tool(load(TOOL), group="right", prefix="tool_")
    assert item(bt.mounting.report(wrong), "required:adapter").status == "fail"
    right = with_adapter.attach_tool(load(TOOL), group="left", prefix="tool_")
    report = bt.mounting.report(right)
    assert item(report, "required:adapter").status == "pass"
    names = [n for row in bt.Scene(right).bom().rows for n in row["names"]]
    assert item(report, "required:adapter").target in names
