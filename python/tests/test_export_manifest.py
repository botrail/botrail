"""One document set from one snapshot: program scope, scripts, issues, re-runs."""

import json
import sys
from pathlib import Path

import botrail as bt
import pytest
from botrail import _cli

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "export"))


def pick_cell():
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene)
    pick.wire_cell(scene)
    return scene


def doc(path):
    return json.loads(Path(path).read_text())


def test_program_scope_reaches_every_export_and_issues_are_retained(tmp_path):
    scene = pick_cell()
    scene.define_signal("unused_signal", initial=False)
    scene.sequence("unselected").step("unused", transition=bt.seq.signal("unused_signal"))
    manifest = bt.export_cell(scene, tmp_path / "package", sequences=["pick"], scenarios=True, clearance_dt=None)
    data = doc(manifest)
    report = doc(manifest.parent / "cell_report.json")
    assert data["conditions"]["sequences"] == ["pick"] == report["io"]["sequences"]
    assert all(c["sequences"] == ["pick"] for c in report["cycles"])
    for filename in ("cell_io.csv", "cell_topology.mmd", "cell_interlocks.csv", "cell.plcopen.xml"):
        assert "unselected" not in (manifest.parent / filename).read_text()
    assert {i["code"] for i in data["issues"]} >= {"plcopen_stubs"}
    assert report["issues"] == data["issues"]
    assert "plcopen_stubs" in (manifest.parent / "cell_report.md").read_text()
    # The report hashes what was written before it; the manifest lists everything.
    documents = [r["path"] for r in data["files"] if r["kind"] not in ("report_markdown", "report_json")]
    assert [r["path"] for r in report["deliverables"]] == documents
    assert all(r["sha256"] and r["bytes"] for r in data["files"])
    assert data["botrail_version"] == bt.__version__
    # The full project retains the authored unselected program; the manifest
    # records the selected scope separately, rather than silently deleting it.
    assert "unselected" in bt.Scene.load_project(manifest.parent / "cell.botrail").sequence_names


def test_re_export_replaces_the_set_in_place(tmp_path):
    scene = pick_cell()
    options = {"exports": ["io", "script", "report"], "scenarios": True, "clearance_dt": None}
    first = bt.export_cell(scene, tmp_path / "rev", **options)
    assert "get_standard_digital_in(2)" in (first.parent / "cell.script").read_text()
    scene.bind_input("part_at_pick", "UR", "DI5")
    second = bt.export_cell(scene, tmp_path / "rev", **options)
    assert second == first
    assert "get_standard_digital_in(5)" in (second.parent / "cell.script").read_text()
    assert "DI5" in (second.parent / "cell_io.csv").read_text()


def test_usd_auxiliary_assets_are_listed(tmp_path):
    from test_usd_robot import ARM

    stage = tmp_path / "arm.usda"
    stage.write_text(ARM)
    scene = bt.Scene(bt.Robot.from_usd(stage))
    scene.sequence("idle").step("wait", transition=bt.seq.elapsed(.03))
    manifest = bt.export_cell(scene, tmp_path / "package", exports=["usd", "report"], clearance_dt=None)
    assets = [r for r in doc(manifest)["files"] if r["kind"] == "usd_asset"]
    assert assets and all(r["parent"].endswith(".usda") and r["sha256"] for r in assets)


def test_each_concurrent_robot_program_gets_a_script(tmp_path):
    scene = bt.Scene()
    robot = bt.Robot.from_urdf(EXAMPLES / "assets/simple_arm.urdf")
    for index, name in enumerate(("a", "b")):
        scene.add_robot(robot, name=name, base_position=(3 * index, 0, 0))
        scene.add_segment("move_" + name, goal=[.1, 0, 0, 0, 0, 0], robot=name)
        scene.sequence(name).step("move", actions=[bt.seq.motion("move_" + name)])
    manifest = bt.export_cell(scene, tmp_path / "package", exports=["script", "report"], clearance_dt=None)
    data = doc(manifest)
    scripts = [r for r in data["files"] if r["kind"] == "script"]
    assert {(r["path"], r["sequence"]) for r in scripts} == {("cell_a.script", "a"), ("cell_b.script", "b")}
    assert not data["issues"]


def test_lowering_warning_and_failed_script_are_not_silently_lost(tmp_path):
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets/simple_arm.urdf"))
    scene.add_segment("move", goal=[.1, 0, 0, 0, 0, 0])
    scene.sequence("cycle").step("timed move", actions=[bt.seq.motion("move")], transition=bt.seq.elapsed(2.))
    manifest = bt.export_cell(scene, tmp_path / "warning", exports=["script", "report"], clearance_dt=None)
    issues = doc(manifest)["issues"]
    assert any(i["code"] == "export_warning" and "timer" in i["message"] for i in issues)
    assert doc(manifest.parent / "cell_report.json")["issues"] == issues
    # A single bake has no planned trajectory for the pick cell's other arm.
    failed = bt.export_cell(pick_cell(), tmp_path / "unsupported", exports=["script"])
    assert any(i["code"] == "script_not_exported" for i in doc(failed)["issues"])


def test_existing_directory_is_reused_and_invalid_inputs_are_not_modified(tmp_path):
    scene = bt.Scene()
    out = tmp_path / "existing"
    out.mkdir()
    (out / "notes.txt").write_text("user work")
    (out / "cell_bom.csv").write_text("stale")
    # Re-exporting into a directory overwrites the set's own files and
    # leaves everything else alone — the document set is meant to be re-run.
    bt.export_cell(scene, out, exports=["bom"])
    assert (out / "notes.txt").read_text() == "user work"
    assert (out / "cell_bom.csv").read_text() != "stale"
    assert bt.export_cell(scene, out, exports=["bom"], manifest=False) == out
    for options in ({"name": "../escape"}, {"dt": float("nan")}, {"sequences": ["missing"]}):
        with pytest.raises(ValueError):
            bt.export_cell(scene, tmp_path / "invalid", **options)
    assert not (tmp_path / "invalid").exists()


def test_cli_export_lists_files_and_keeps_the_manifest_on_request(tmp_path, capsys):
    scene = bt.Scene()
    cell = tmp_path / "cell.botrail"
    scene.save_project(cell)
    out = tmp_path / "package"
    assert _cli.main(["export", str(cell), "--out", str(out), "--bom", "--report", "--manifest"]) == 0
    exported = json.loads(capsys.readouterr().out)
    manifest = doc(exported["manifest"])
    assert {Path(p).name for p in exported["files"]} == {r["path"] for r in manifest["files"]}
    assert manifest["botrail_version"] == bt.__version__ and manifest["conditions"]["sequences"] == []
