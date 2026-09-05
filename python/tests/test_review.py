"""Review readiness must not mistake incomplete inputs for a valid design."""

import json
from pathlib import Path

import botrail as bt
import pytest
from botrail import _cli


def electrical_cell(*, identified=True, voltage=24, logic="pnp", field=True):
    scene = bt.Scene()
    scene.add_beam_sensor("eye", frm=(0, 0, 0.5), to=(0, 0.2, 0.5))
    scene.add_io_node("PLC", kind="plc", channels=bt.io.di8(voltage=24, logic="pnp"))
    scene.bind_input("eye", "PLC", "DI0", **({"voltage": voltage, "logic": logic} if field else {}))
    if identified:
        scene.set_part("eye", model="EYE", sensing_range_mm=200, mass_kg=0.1)
        scene.set_part("PLC", model="PLC", mass_kg=1.)
    return scene


def item(report, id):
    return next(i for i in report.items if i.id == id)


def test_concept_and_design_keep_the_static_check_contract():
    scene = electrical_cell(identified=False, field=False)
    before = scene._project_json()
    static = scene.check().to_json()
    concept = bt.review(scene)
    design = bt.review(scene, stage="design")
    assert concept.check["ok"] and concept.ready
    assert not design.ready
    assert {i.group for i in design.blockers()} >= {"equipment", "specifications", "connections"}
    assert scene._project_json() == before and scene.check().to_json() == static
    # Supplied identities/specifications and both electrical ends resolve the review.
    assert bt.review(electrical_cell(), stage="design").ready


def test_known_electrical_mismatch_blocks_even_a_concept_review():
    scene = electrical_cell(voltage=12, logic="npn")
    assert scene.check().ok
    result = bt.review(scene)
    assert not result.ready
    voltage = item(result, "connections:(unhosted):input:eye:voltage")
    assert voltage.status == "fail" and voltage.evidence["field_value"] == 12
    assert item(result, "connections:(unhosted):input:eye:logic").status == "fail"
    with pytest.raises(ValueError, match="explicit failure"):
        bt.review(scene, annotations={voltage.id: {"not_applicable": "ignore it"}})


def test_missing_electrical_data_is_unknown_and_not_implicitly_compatible():
    result = bt.review(electrical_cell(field=False), required=["connections"])
    missing = [i for i in result.blockers() if i.group == "connections"]
    assert len(missing) == 2 and all(i.status == "unknown" for i in missing)
    assert all("field device" in i.message for i in missing)
    assert all(i.evidence["channel_value"] is not None for i in missing)


def test_merged_bom_quantities_and_missing_values_are_visible():
    scene = bt.Scene()
    for n, qty in (("panel_a", 2), ("panel_b", 3)):
        scene.add_box(n, size=(1, 1, 1), position=(0, 0, 0))
        scene.set_part(n, model="PANEL", mass_kg=5, qty=qty)
    scene.add_box("unknown", size=(1, 1, 1), position=(0, 0, 0))
    scene.set_part("unknown", model="OTHER", qty=4)
    original = scene.bom().total("mass_kg")
    result = bt.review(scene, required=["totals"])
    total = result.totals["mass_kg"]
    assert original == total["known_subtotal"] == 25
    assert total["known_qty"] == 5 and total["target_qty"] == 9
    assert total["missing"] == [{"names": ["unknown"], "qty": 4, "reason": "missing"}]
    assert not result.ready and item(result, "totals:mass_kg").status == "unknown"
    assert scene.bom().total("mass_kg") == original
    with pytest.raises(ValueError, match="every name"):
        bt.review(scene, totals={"mass_kg": ["panel_a"]})
    selected = bt.review(scene, totals={"mass_kg": ["panel_a", "panel_b"]})
    assert selected.totals["mass_kg"]["target_qty"] == 5
    assert not selected.totals["mass_kg"]["missing"]


def test_zero_is_known_and_unknown_totals_are_not_zero():
    scene = bt.Scene()
    scene.add_box("load", size=(1, 1, 1), position=(0, 0, 0))
    scene.set_part("load", model="L", current_a=0)
    result = bt.review(scene, totals={"current_a": ["load"], "power_w": ["load"]})
    assert result.totals["current_a"]["known_subtotal"] == 0
    assert result.totals["current_a"]["known_qty"] == 1
    assert result.totals["power_w"]["known_subtotal"] is None
    assert result.totals["power_w"]["known_qty"] == 0


def test_incomplete_grasp_input_and_supply_budget_do_not_pass():
    robot = bt.Robot.from_urdf(Path(__file__).resolve().parents[2] / "examples/assets/simple_arm.urdf")
    scene = bt.Scene(robot)
    scene.set_part("simple_arm", model="ARM", payload_kg=5)
    scene.add_box("part", size=(.05, .05, .05), position=(1, 0, 0))
    scene.sequence("pick").step("grasp", actions=[bt.seq.attach("part")])
    result = bt.review(scene)
    assert any(i.group == "specifications" and i.status == "unknown" and "no mass for part" in i.message
               for i in result.items)
    # The old supply requirement is an unscoped whole-BOM sum, not capacity proof.
    for name, attrs in (("supply", {"category": "power_supply", "output_a": 10}), ("load", {"current_a": 2})):
        scene.add_box(name, size=(.1, .1, .1), position=(2, 0, 0))
        scene.set_part(name, model=name, **attrs)
    result = bt.review(scene)
    supply = item(result, "specifications:obstacle:supply:supply")
    assert supply.status == "unknown" and "connected" in supply.basis


def test_absent_specs_are_not_claimed_to_have_been_checked():
    scene = bt.Scene()
    scene.add_box("bracket", size=(.1, .1, .1), position=(0, 0, 0))
    scene.set_part("bracket", model="BRACKET")
    result = bt.review(scene, stage="design")
    check = item(result, "specifications:obstacle:bracket")
    assert check.status == "not_run" and check.blocking
    result = bt.review(scene, stage="design", annotations={check.id: {
        "not_applicable": "Bracket strength is reviewed in the mechanical calculation package",
        "source_kind": "user_input", "reference": "MECH-001", "owner": "mechanical design",
    }})
    assert result.ready
    assert item(result, check.id).evidence["observed_status"] == "not_run"


def test_simulation_evidence_must_cover_the_selected_programs():
    scene = bt.Scene()
    for name in ("one", "two"):
        scene.sequence(name).step("wait", transition=bt.seq.elapsed(.03))
    report = scene.cell_report(scene.simulate_sequence("one"))
    result = bt.review(scene, report=report, required=["simulation"])
    assert not result.ready
    assert item(result, "simulation:one").status == "pass"
    assert item(result, "simulation:one:clearance").status == "not_applicable"  # no robots
    assert item(result, "simulation:two").status == "not_run"
    assert bt.review(scene, report=report, sequences=["one"], required=["simulation"]).ready


def test_skipped_clearance_is_distinct_from_a_completed_bake():
    robot = bt.Robot.from_urdf(Path(__file__).resolve().parents[2] / "examples/assets/simple_arm.urdf")
    scene = bt.Scene(robot)
    scene.add_box("wall", size=(1, 1, 1), position=(3, 0, 1))
    scene.sequence("idle").step("wait", transition=bt.seq.elapsed(.03))
    report = scene.cell_report(scene.simulate_sequence("idle"), clearance_dt=None)
    result = bt.review(scene, report=report, required=["simulation"])
    assert item(result, "simulation:idle").status == "pass"
    assert item(result, "simulation:idle:clearance").status == "not_run"
    assert not result.ready


def test_scenario_completion_is_not_expected_result_acceptance(tmp_path):
    scene = bt.Scene()
    scene.define_signal("go", initial=True)
    scene.declare_io("go", role="internal")
    scene.sequence("cycle").step("wait", transition=bt.seq.signal("go"))
    scene.add_scenario("broken", faults=[bt.io.stuck("go", False)])
    runs = scene.simulate_scenarios(["cycle"], max_duration=.05)
    path = tmp_path / "io.csv"
    scene.export_io_list(path)
    report = scene.cell_report(scenarios=runs, deliverables=[path])
    result = bt.review(scene, report=report, required=["scenarios", "deliverables"])
    assert not result.ready
    fault = item(result, "scenarios:broken")
    assert fault.status == "unknown" and fault.evidence["observation"]["ok"] is False
    assert item(result, f"deliverables:{path}").status == "unknown"
    absent = bt.review(scene)
    assert item(absent, "scenarios:broken").status == "not_run"


def test_annotations_preserve_uncertainty_and_formats(tmp_path):
    scene = electrical_cell(field=False)
    id = "connections:(unhosted):input:eye:voltage"
    note = {"source_kind": "manufacturer", "reference": "vendor.pdf p.3", "assumptions": "revision unconfirmed",
            "owner": "electrical design", "due": "2026-09-10", "next_action": "Request the voltage range"}
    result = bt.review(scene, annotations={id: note})
    assert item(result, id).status == "unknown"  # a citation supplies no missing value
    assert item(result, id).annotation == note
    assert "Request the voltage range" in result.to_markdown()
    for fmt in ("json", "md"):
        result.save(tmp_path / f"review.{fmt}")
    assert json.loads((tmp_path / "review.json").read_text()) == result.to_dict()
    assert (tmp_path / "review.md").read_text() == result.to_markdown()
    with pytest.raises(ValueError, match="reference"):
        bt.review(scene, annotations={id: {"source_kind": "measured"}})
    with pytest.raises(ValueError, match="unknown review annotation"):
        bt.review(scene, annotations={"missing": note})
    with pytest.raises(ValueError, match="unknown required"):
        bt.review(scene, required=["typo"])


def test_cli_review_and_check_have_separate_exit_contracts(capsys, tmp_path):
    scene = electrical_cell(identified=False, field=False)
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    assert _cli.main(["check", str(path)]) == 0
    assert json.loads(capsys.readouterr().out)["ok"]
    assert _cli.main(["review", str(path)]) == 0
    assert json.loads(capsys.readouterr().out)["ready"]
    assert _cli.main(["review", str(path), "--stage", "design", "--report", str(tmp_path / "review.md")]) == 1
    assert not json.loads(capsys.readouterr().out)["ready"]
    assert "unresolved" in (tmp_path / "review.md").read_text()
    assert _cli.main(["review", str(path), "--require", "simulation:missing"]) == 2
    assert "unknown required" in json.loads(capsys.readouterr().out)["error"]


def test_cli_baking_and_invalid_config(capsys, tmp_path):
    scene = bt.Scene()
    scene.sequence("idle").step("wait", transition=bt.seq.elapsed(.02))
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    assert _cli.main(["review", str(path), "--stage", "design"]) == 1
    capsys.readouterr()
    assert _cli.main(["review", str(path), "--stage", "design", "--simulate"]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["cell_report"]["cycles"][0]["duration"] == pytest.approx(.02)
    config = tmp_path / "config.json"
    config.write_text(json.dumps({"totals": []}))
    assert _cli.main(["review", str(path), "--config", str(config)]) == 2
    assert "totals must map" in json.loads(capsys.readouterr().out)["error"]
    config.write_text(json.dumps({"required": ["scenarios"]}))
    assert _cli.main(["review", str(path), "--config", str(config), "--markdown"]) == 1
    assert "No fault scenarios declared" in capsys.readouterr().out
