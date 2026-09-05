"""Physical declarations stay separate from behaviour and unknown demand stays unknown."""

import csv
import io
import json
from pathlib import Path

import botrail as bt
import pytest
from botrail import _cli

c = bt.connections


def equipment(scene, name, **attrs):
    scene.add_box(name, size=(.1, .1, .1), position=(len(scene.parts()), 0, 0))
    scene.set_part(name, model=name, **attrs)
    return name


def power_cell():
    s = bt.Scene()
    equipment(s, "psu", category="power_supply", output_v=24, output_a=2)
    equipment(s, "eye", voltage_v=24, current_a=.1)
    c.port(s, "24V", "psu", "power", "supply")
    c.port(s, "eye.power", "eye", "power", "load")
    c.connect(s, "24V", "eye.power", name="W1", cable="CBL-001", reference="E-001")
    return s


def check(result, id):
    return next(row for row in result.checks if row["id"] == id)


def test_separate_voltage_systems_and_unconnected_loads():
    s = power_cell()
    equipment(s, "psu48", category="power_supply", output_v=48, output_a=10)
    equipment(s, "drive", voltage_v=48, current_a=5)
    equipment(s, "spare", voltage_v=24, current_a=99)
    c.port(s, "48V", "psu48", "power", "supply")
    c.port(s, "drive.power", "drive", "power", "load")
    c.connect(s, "48V", "drive.power")
    r = c.report(s)
    assert [(p["capacity"], p["known_subtotal"], p["status"]) for p in r.supplies] == [(2, .1, "pass"), (10, 5, "pass")]
    assert not r.ready  # The spare's demand is unconnected, never added to either source.
    assert check(r, "requirements:obstacle:spare:power:load")["status"] == "unknown"
    assert not any(req.key == "output_a" for row in bt.select.requirements(s) for req in row.requirements)
    assert s.check().ok
    assert any(i.id == "specifications:obstacle:psu:supply" and i.status == "pass" for i in bt.review(s).items)


def test_missing_consumption_known_overload_zero_and_updated_part():
    s = power_cell()
    equipment(s, "unknown", voltage_v=24)
    c.port(s, "missing", "unknown", "power", "load")
    c.connect(s, "24V", "missing")
    r = c.report(s)
    assert r.supplies[0]["known_subtotal"] == .1
    assert r.supplies[0]["missing_loads"] == ["missing"]
    assert r.supplies[0]["status"] == "unknown"
    s.set_part("eye", model="eye", voltage_v=24, current_a=3)
    r = c.report(s)
    assert r.supplies[0]["status"] == "fail"  # Known overload wins over incomplete data.
    assert not s.check().ok
    assert any(i.id == "connections:physical:supply:24V:capacity" and i.status == "fail" for i in bt.review(s).items)
    s.set_part("eye", model="eye", voltage_v=24, current_a=0)
    s.set_part("unknown", model="unknown", voltage_v=24, current_a=0)
    r = c.report(s)
    assert r.ready and r.supplies[0]["known_subtotal"] == 0
    assert r.supplies[0]["known_loads"] == 2
    assert r.supplies[0]["loads"][0]["origin"]["source"] == "part"


@pytest.mark.parametrize("source,target,status", [
    ({"voltage_v": 24}, {"voltage_v": 24.1}, "fail"),
    ({"voltage_min_v": 23, "voltage_max_v": 25}, {"voltage_min_v": 22, "voltage_max_v": 26}, "pass"),
    ({"voltage_min_v": 23, "voltage_max_v": 27}, {"voltage_min_v": 22, "voltage_max_v": 26}, "fail"),
    ({"voltage_v": 24}, {"voltage_min_v": 20}, "unknown"),
    ({}, {"voltage_v": 24}, "unknown"),
])
def test_entire_voltage_range_must_fit(source, target, status):
    s = bt.Scene()
    equipment(s, "s")
    equipment(s, "l")
    c.port(s, "out", "s", "power", "supply", capacity_a=1, **source)
    c.port(s, "in", "l", "power", "load", current_a=.2, **target)
    c.connect(s, "out", "in", name="wire")
    assert check(c.report(s), "link:wire:voltage")["status"] == status


def test_quantity_and_multiple_feeds_do_not_duplicate_part_ratings():
    s = power_cell()
    s.set_part("eye", model="eye", qty=3, voltage_v=24, current_a=.1)
    assert c.report(s).supplies[0]["known_subtotal"] is None
    c.port(s, "eye.power", "eye", "power", "load", voltage_v=24, current_a=.3)
    assert c.report(s).ready
    c.port(s, "eye.aux", "eye", "power", "load", voltage_v=24, current_a=.2)
    c.connect(s, "24V", "eye.aux")
    assert c.report(s).supplies[0]["known_subtotal"] == .5
    c.port(s, "24V.aux", "psu", "power", "supply", voltage_v=24, capacity_a=2)
    r = c.report(s)
    assert r.supplies[0]["capacity"] is None
    assert check(r, "port:24V.aux:shared_capacity")["status"] == "unknown"


def test_orphan_optional_multiple_and_dangling_references(tmp_path):
    s = power_cell()
    c.disconnect(s, "W1")
    assert check(c.report(s), "port:eye.power:connection")["status"] == "unknown"
    c.port(s, "eye.power", "eye", "power", "load", required=False)
    assert check(c.report(s), "port:eye.power:connection")["status"] == "not_applicable"
    assert c.report(s).supplies[0]["status"] == c.report(s).ports[1]["status"] == "not_applicable"
    assert any(i.id == "specifications:obstacle:psu:supply" and i.status == "not_applicable" for i in bt.review(s).items)
    c.connect(s, "24V", "eye.power", name="W1")
    c.connect(s, "24V", "eye.power", name="W2")
    r = c.report(s)
    assert check(r, "port:eye.power:multiple")["status"] == "fail"
    assert r.supplies[0]["known_subtotal"] == .1  # Duplicate cable cannot double-count demand.
    c.disconnect(s, "W2")
    s.remove_obstacle("eye")
    r = c.report(s)
    assert check(r, "port:eye.power:target")["status"] == "fail"
    assert r.connections[0]["status"] == r.supplies[0]["status"] == "fail"
    s.save_project(tmp_path / "dangling.botrail")
    assert c.report(bt.Scene.load_project(tmp_path / "dangling.botrail")).to_dict() == r.to_dict()
    c.remove_port(s, "eye.power")
    assert check(c.report(s), "link:W1:reference")["status"] == "fail"


@pytest.mark.parametrize("source,target", [("eye.power", "24V"), ("24V", "24V"), ("network", "eye.power")])
def test_wrong_medium_direction_or_self_is_failure(source, target):
    s = power_cell()
    c.port(s, "network", "psu", "network", "peer", required=False, protocol="EtherCAT")
    c.connect(s, source, target, name="bad")
    assert check(c.report(s), "link:bad:type")["status"] == "fail"


def signal_cell():
    s = bt.Scene()
    s.add_beam_sensor("eye", frm=(0, 0, .1), to=(1, 0, .1))
    s.add_io_node("PLC", channels=bt.io.di16(voltage=24, logic="pnp"))
    s.declare_io("eye", role="input", kind="di")
    s.bind_input("eye", "PLC", "DI0")
    c.port(s, "field", "eye", "signal", "output", signal_type="digital", voltage_v=24, logic="pnp")
    c.port(s, "controller", "PLC", "signal", "input", io={"point": "eye", "direction": "input", "node": "PLC"})
    c.connect(s, "field", "controller", name="signal")
    return s


def test_io_assignment_remains_authoritative_and_follows_reassignment():
    s = signal_cell()
    r = c.report(s)
    assert r.ready
    controller = r.ports[1]
    assert controller["resolved"]["signal_type"]["source"] == "io_channel"
    assert controller["io_assignment"] == ["PLC", "DI0"]
    s.bind_input("eye", "PLC", "DI1")
    assert c.report(s).ports[1]["io_assignment"] == ["PLC", "DI1"]
    c.port(s, "controller", "PLC", "signal", "input", logic="npn",
           io={"point": "eye", "direction": "input", "node": "PLC"})
    assert check(c.report(s), "port:controller:io:logic")["status"] == "fail"
    c.port(s, "alias", "PLC", "signal", "input", io={"point": "eye", "direction": "input", "node": "PLC"})
    assert check(c.report(s), "port:alias:duplicate_io")["status"] == "fail"


def test_io_required_signal_mismatch_and_missing_binding():
    s = signal_cell()
    s.add_beam_sensor("other", frm=(0, 1, .1), to=(1, 1, .1))
    c.port(s, "field", "other", "signal", "output", signal_type="safe_digital", voltage_v=24, logic="npn")
    r = c.report(s)
    assert check(r, "link:signal:signal_type")["status"] == "fail"
    assert check(r, "link:signal:logic")["status"] == "fail"
    assert check(r, "link:signal:field")["status"] == "fail"
    s.remove_io_node("PLC")
    assert check(c.report(s), "port:controller:io")["status"] == "fail"
    s.add_io_node("PLC", channels=bt.io.di16(voltage=24, logic="pnp"))
    assert check(c.report(s), "port:controller:io")["status"] == "unknown"


def test_air_flow_needs_equal_reference_conditions():
    s = bt.Scene()
    equipment(s, "air")
    equipment(s, "valve")
    c.port(s, "air.out", "air", "pneumatic", "supply", pressure_bar=6, capacity_l_min=100, flow_reference="ANR")
    c.port(s, "air.in", "valve", "pneumatic", "load", pressure_min_bar=4, pressure_max_bar=7, flow_l_min=20)
    c.connect(s, "air.out", "air.in", name="hose")
    r = c.report(s)
    assert check(r, "link:hose:pressure")["status"] == "pass"
    assert r.supplies[0]["known_subtotal"] is None and not r.ready
    c.port(s, "air.in", "valve", "pneumatic", "load", pressure_min_bar=4, pressure_max_bar=7,
           flow_l_min=20, flow_reference="ANR")
    assert c.report(s).ready
    c.port(s, "air.out", "air", "pneumatic", "supply", pressure_bar=8, capacity_l_min=10, flow_reference="ANR")
    r = c.report(s)
    assert check(r, "link:hose:pressure")["status"] == check(r, "supply:air.out:capacity")["status"] == "fail"


def test_network_uplink_does_not_supply_missing_capabilities():
    s = bt.Scene()
    s.add_io_node("PLC")
    s.add_io_node("remote", kind="remote_io", uplink=("PLC", "EtherCAT"))
    assert check(c.report(s), "requirements:uplink:remote")["status"] == "unknown"
    c.port(s, "plc.net", "PLC", "network", "peer", protocol="EtherCAT")
    c.port(s, "remote.net", "remote", "network", "peer", protocol="PROFINET")
    c.connect(s, "plc.net", "remote.net", name="bus")
    assert check(c.report(s), "link:bus:protocol")["status"] == "fail"
    c.port(s, "remote.net", "remote", "network", "peer", protocol="ethercat")
    assert c.report(s).ready


@pytest.mark.parametrize("specs", [
    {"voltage_v": -1}, {"current_a": float("nan")}, {"current_a": float("inf")},
    {"current_a": True}, {"current_a": "2"}, {"current": 2}, {"protocol": "abc"},
    {"voltage_min_v": 25, "voltage_max_v": 20}, {"capacity_a": 2},
])
def test_invalid_specification_rejected_without_mutation(specs):
    s = power_cell()
    before = s._connection_plan_json()
    with pytest.raises(ValueError):
        c.port(s, "eye.power", "eye", "power", "load", **specs)
    assert s._connection_plan_json() == before


def test_schema_roundtrip_python_snapshot_and_no_behaviour_changes(tmp_path, monkeypatch):
    import jsonschema

    s = signal_cell()
    equipment(s, "supply", category="power_supply", output_v=24, output_a=2)
    before_io, before_bom = s.io_map().to_json(), s.bom().to_json()
    c.port(s, "supply.out", "supply", "power", "supply", terminal="X1:1", reference="E-020")
    c.port(s, "eye.power", "eye", "power", "load", voltage_v=24, current_a=.1)
    c.connect(s, "supply.out", "eye.power", cable="W2")
    assert s.io_map().to_json() == before_io and s.bom().to_json() == before_bom
    s.save_project(tmp_path / "cell.botrail")
    jsonschema.validate(json.loads((tmp_path / "cell.botrail").read_text()), json.loads(bt.project_schema()))
    (tmp_path / "cell.py").write_text(s.generate_python())
    monkeypatch.setattr(bt, "studio", lambda *args, **kwargs: None)
    for again in (s._snapshot(), bt.Scene.load_project(tmp_path / "cell.botrail"), _cli.load_cell(str(tmp_path / "cell.py"))):
        assert c.report(again).to_dict() == c.report(s).to_dict()
    clone = s._snapshot()
    c.disconnect(s, "supply.out -> eye.power")
    assert c.report(clone).ready and not c.report(s).ready


def test_robot_rename_follows_equipment_reference():
    s = bt.Scene(bt.Robot.from_urdf(Path(__file__).resolve().parents[2] / "examples/assets/simple_arm.urdf"))
    c.port(s, "robot.power", "simple_arm", "power", "load", voltage_v=48, current_a=2)
    s.rename_robot("simple_arm", "arm")
    assert c.report(s).ports[0]["target"] == "arm"
    assert check(c.report(s), "port:robot.power:target")["status"] == "pass"


def test_local_power_supply_keeps_declared_rating():
    s = bt.Scene()
    bt.parts.power_supply(s, "PSU", (0, 0, 0), size=(.1, .1, .1), output_a=3, output_v=24)
    assert s.part("PSU")["attributes"]["output_a"] == 3


def test_cli_tables_and_export_revision(tmp_path, capsys):
    s = power_cell()
    path = tmp_path / "cell.botrail"
    s.save_project(path)
    assert _cli.main(["connections", str(path), "--csv", str(tmp_path / "ports.csv"),
                      "--power", str(tmp_path / "power.csv"), "--report", str(tmp_path / "report.md")]) == 0
    assert json.loads(capsys.readouterr().out)["ready"]
    assert len(list(csv.DictReader((tmp_path / "ports.csv").open()))) == 2
    row = next(csv.DictReader(io.StringIO((tmp_path / "power.csv").read_text())))
    assert row["capacity"] == "2.0" and row["known_subtotal"] == "0.1"
    assert "CBL-001" in (tmp_path / "report.md").read_text()
    assert _cli.main(["connections", str(path), "--csv", str(tmp_path / "bad.txt")]) == 2
    assert "extension" in json.loads(capsys.readouterr().out)["error"]
    manifest = bt.export_cell(s, tmp_path / "package", exports=["connections", "report"])
    assert bt.verify_export(manifest, scene=s)["same_revision"]
    data = json.loads((manifest.parent / "cell_report.json").read_text())
    physical = json.loads((manifest.parent / "cell_connections.json").read_text())
    assert physical == data["connections"] == c.report(s).to_dict()
    assert next(csv.DictReader(io.StringIO((manifest.parent / "cell_power.csv").read_text())))["known_subtotal"] == "0.1"
    c.disconnect(s, "W1")
    assert not bt.verify_export(manifest, scene=s)["same_revision"]
    s.save_project(path)
    assert _cli.main(["connections", str(path)]) == 1
    assert not json.loads(capsys.readouterr().out)["ready"]


def test_empty_declarations_are_not_a_completed_review():
    assert not c.report(bt.Scene()).ready
    assert c.report(bt.Scene()).checks[0]["status"] == "not_run"


def test_restore_validates_structure_and_unknown_capacity():
    s = power_cell()
    plan = json.loads(s._connection_plan_json())
    plan["ports"].append(plan["ports"][0])
    with pytest.raises(ValueError, match="duplicate"):
        c.restore(s, plan)
    plan = json.loads(s._connection_plan_json())
    plan["ports"][0]["specs"]["capacity_a"] = "2 A"
    with pytest.raises(ValueError):
        c.restore(s, plan)
    s.set_part("psu", category="power_supply", model="psu", output_v=24)
    r = c.report(s)
    assert r.supplies[0]["known_subtotal"] == .1 and r.supplies[0]["capacity"] is None
    assert check(r, "supply:24V:capacity")["status"] == "unknown"


def test_example_known_and_missing_load():
    import runpy

    build = runpy.run_path(str(Path(__file__).resolve().parents[2] / "examples/engineering/cell_connections_demo.py"))["build"]
    r = c.report(build())
    assert r.ready
    assert [s["known_subtotal"] for s in r.supplies] == pytest.approx([.45, 2, 60])
    assert "0.45 A" in r.to_markdown()
    r = c.report(build(unknown_valve_current=True))
    assert not r.ready and r.supplies[0]["known_subtotal"] == .1
    assert r.supplies[0]["missing_loads"] == ["field/valve.power"]
    assert r.supplies[1]["status"] == "pass" and r.supplies[1]["known_subtotal"] == 2
