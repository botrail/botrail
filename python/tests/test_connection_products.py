"""Actual catalog attachment topology gates declared product connection routes."""

# ruff: noqa: F811
import json
import shutil
from copy import deepcopy

import botrail as bt
import pytest
import yaml
from test_catalog_kits import ARM, BASE, TOOL, load, packages  # noqa: F401
from test_connection_profiles import VALUES, connected  # noqa: F401


@pytest.fixture
def product(connected):
    root, kit = connected
    path = root / TOOL / "manifest.yaml"
    data = yaml.safe_load(path.read_text())
    data["order"] = {"unit": "each", "part_number": "HAND"}
    data["compatibility"] = deepcopy(kit["compatibility"])
    profile = data["compatibility"]["connections"][0]
    profile["part_number"] = "HAND"
    profile["via"] = [BASE]
    path.write_text(yaml.safe_dump(data))
    return root


def scene(root, *, adapter=True, offset=None, format="urdf"):
    arm = load(root, ARM, format=format)
    if adapter:
        arm = arm.attach_tool(load(root, BASE, format=format), prefix="coupling_")
    robot = arm.attach_tool(
        load(root, TOOL, format=format),
        prefix="hand_",
        **({"offset": offset} if offset else {}),
    )
    s = bt.Scene()
    s.add_robot(robot, name="robot")
    return s


def configure(s):
    p = bt.mounting.report(s).products[0]
    bt.connections.configure(s, p["target"], "host-a", values=VALUES)
    return bt.mounting.report(s).products[0]


def test_actual_ordered_adapter_route_and_host_are_reported(product):
    s = scene(product)
    p = configure(s)
    assert p["host"] == ARM and p["via"] == [BASE]
    assert p["connection"]["configuration"]["status"] == "pass"
    assert p["connection"]["electrical"]["status"] == "pass"
    assert len(bt.connections.report(s).configurations) == 1
    assert len(s.bom().rows) == 3  # individual purchases are not a fictional kit


@pytest.mark.parametrize(
    "change",
    [
        "missing_adapter",
        "extra_adapter",
        "adapter_identity",
        "host_identity",
        "tool_identity",
        "mount_pose",
    ],
)
def test_changed_route_cannot_inherit_the_product_profile(product, change):
    s = scene(product, adapter=change != "missing_adapter")
    if change == "extra_adapter":
        robot = (
            load(product, ARM)
            .attach_tool(load(product, BASE), prefix="a_")
            .attach_tool(load(product, BASE), prefix="b_")
            .attach_tool(load(product, TOOL), prefix="hand_")
        )
        s = bt.Scene()
        s.add_robot(robot, name="robot")
    elif change.endswith("identity"):
        target = {
            "adapter_identity": "robot/tool",
            "host_identity": "robot",
            "tool_identity": "robot/tool2",
        }[change]
        s.set_part(target, catalog=TOOL if target != "robot/tool2" else BASE)
    elif change == "mount_pose":
        source = json.loads(s._project_json())["robots"][0]["source"]
        source["offset"]["position"][2] = 0.01
        s = bt.Scene()
        s.add_robot(bt.Robot._from_source_json(json.dumps(source)), name="robot")
    p = configure(s)
    assert p["connection"]["configuration"]["status"] == "fail"
    assert p["connection"]["electrical"]["status"] == "not_applicable"


@pytest.mark.parametrize("format", ["urdf", "usd"])
def test_product_route_survives_offline_project_and_generated_python(
    product, tmp_path, format
):
    s = scene(product, format=format)
    configure(s)
    before = bt.mounting.report(s).to_dict()
    path = tmp_path / "product.botrail"
    s.save_project(path)
    shutil.rmtree(product)
    restored = bt.Scene.load_project(path)
    assert bt.mounting.report(restored).to_dict() == before
    ns = {}
    exec(restored.generate_python().replace("bt.studio(scene)", ""), ns)  # noqa: S102 - test generated replay
    assert bt.mounting.report(ns["scene"]).to_dict() == before
    restored.rename_robot("robot", "other")
    assert (
        bt.mounting.report(restored).products[0]["connection"]["configuration"][
            "status"
        ]
        == "pass"
    )


def test_unknown_or_absent_profile_does_not_pass(product):
    s = scene(product)
    p = bt.mounting.report(s).products[0]
    assert p["connection"]["configuration"]["status"] == "unknown"
    bt.connections.configure(s, p["target"], "absent", values=VALUES)
    assert (
        bt.mounting.report(s).products[0]["connection"]["configuration"]["status"]
        == "fail"
    )


def test_tool_mass_budget_keeps_unknown_required_hardware_and_scopes_instances(product):
    for pid, mass in [(BASE, 0.06), (TOOL, 0.8)]:
        path = product / pid / "manifest.yaml"
        d = yaml.safe_load(path.read_text())
        d["specs"] = {"mass_kg": mass}
        if pid == TOOL:
            d["order"]["requires"] = [
                {"catalog": BASE, "qty": 1},
                {"part_number": "CABLE", "qty": 1},
            ]
        path.write_text(yaml.safe_dump(d))
    s = scene(product)
    s.add_robot(s.robot, name="other")
    budgets = bt.select.tool_loads(s)
    assert len(budgets) == 2
    for b in budgets:
        assert b["known_mass_kg"] == pytest.approx(
            0.86
        )  # not the two-robot BOM quantity
        assert b["total_mass_kg"] is None
        assert [r.get("part_number") for r in b["unresolved_requirements"]] == ["CABLE"]
        assert b["center_of_gravity"] is None and b["inertia"] is None
        assert not b["workpiece_included"]
    path = product / TOOL / "manifest.yaml"
    d = yaml.safe_load(path.read_text())
    d["order"]["requires"] = [{"catalog": BASE}]
    path.write_text(yaml.safe_dump(d))
    b = bt.select.tool_loads(scene(product))[0]
    assert b["total_mass_kg"] == pytest.approx(0.86)
    assert b["dynamics_status"] == "not_evaluated"


def test_required_hardware_cannot_be_counted_twice_or_match_a_different_sku(product):
    second = "test/tool/second/r1"
    shutil.copytree(product / TOOL, product / second)
    for pid in (TOOL, second):
        path = product / pid / "manifest.yaml"
        data = yaml.safe_load(path.read_text())
        data["id"] = pid
        data["specs"] = {"mass_kg": 0.8}
        data["order"]["requires"] = [{"catalog": BASE, "part_number": "COUPLING"}]
        path.write_text(yaml.safe_dump(data))
    path = product / BASE / "manifest.yaml"
    data = yaml.safe_load(path.read_text())
    data["specs"] = {"mass_kg": 0.06}
    data["order"] = {"part_number": "COUPLING", "unit": "each"}
    path.write_text(yaml.safe_dump(data))
    arm = (
        load(product, ARM)
        .attach_tool(load(product, BASE), prefix="adapter_")
        .attach_tool(load(product, TOOL), prefix="a_")
        .attach_tool(load(product, second), prefix="b_")
    )
    budget = bt.select.tool_loads(bt.Scene(arm))[0]
    assert budget["total_mass_kg"] is None
    assert sum(r["missing_qty"] for r in budget["unresolved_requirements"]) == 1
    data["order"]["part_number"] = "DIFFERENT-SKU"
    path.write_text(yaml.safe_dump(data))
    budget = bt.select.tool_loads(scene(product))[0]
    assert budget["unresolved_requirements"][0]["missing_qty"] == 1
