"""One input revision across engineering documents, and honest attachments."""

import json
import shutil
import sys
from pathlib import Path

import botrail as bt
import pytest
from botrail import _cli, deliverables

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "export"))


def pick_cell():
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene)
    pick.wire_cell(scene)
    return scene


def doc(path):
    return json.loads(path.read_text())


def test_di_reassignment_cannot_mix_old_script_with_new_io(tmp_path):
    scene = pick_cell()
    options = {"exports": ["io", "script", "report"], "scenarios": True, "clearance_dt": None}
    first = bt.export_cell(scene, tmp_path / "rev1", **options)
    old_script = (first.parent / "cell.script").read_text()
    assert "get_standard_digital_in(2)" in old_script
    assert bt.verify_export(first, scene=scene)["same_revision"]

    scene.bind_input("part_at_pick", "UR", "DI5")
    assert not bt.verify_export(first, scene=scene)["ok"]
    second = bt.export_cell(scene, tmp_path / "rev2", **options)
    new_script = (second.parent / "cell.script").read_text()
    assert "get_standard_digital_in(5)" in new_script
    assert "DI5" in (second.parent / "cell_io.csv").read_text()
    assert bt.verify_export(second, scene=scene)["same_revision"]
    assert doc(first)["input_sha256"] != doc(second)["input_sha256"]
    # A real file replacement, retaining the new package's manifest.
    shutil.copyfile(first.parent / "cell.script", second.parent / "cell.script")
    checked = bt.verify_export(second)
    assert not checked["same_revision"]
    assert any("cell.script: file digest" in e for e in checked["errors"])
    reviewed = bt.review(scene, manifest=second)
    assert not reviewed.ready
    assert any(i.id == "deliverables:revision" and i.status == "fail" for i in reviewed.items)


def test_external_attachments_are_not_promoted_by_hashing(tmp_path):
    scene = bt.Scene()
    attachment = tmp_path / "old.script"
    attachment.write_text("get_standard_digital_in(2)\n")
    manifest = bt.export_cell(scene, tmp_path / "package", exports=["report"], attachments=[attachment])
    checked = bt.verify_export(manifest)
    assert checked["ok"] and not checked["same_revision"]
    external = checked["files"][0]
    assert external["origin"] == "external_attachment" and "input_sha256" not in external
    result = bt.review(scene, manifest=manifest, required=["deliverables"])
    assert not result.ready
    assert any(i.target.endswith("old.script") and i.status == "unknown" for i in result.items)
    bare = scene.cell_report(deliverables=[attachment])
    assert bare.deliverables[0]["origin"] == "external_attachment"
    assert "External attachments" in bare.to_markdown()


def test_program_scope_reaches_every_export_and_warnings_are_retained(tmp_path):
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
    assert bt.verify_export(manifest, scene=scene)["same_revision"]
    # The full project retains the authored unselected program; the manifest
    # records the selected scope separately, rather than silently deleting it.
    assert "unselected" in bt.Scene.load_project(manifest.parent / "cell.botrail").sequence_names
    review = bt.review(scene, manifest=manifest, required=["deliverables"])
    assert not review.ready  # lowering/stub issues, despite matching revisions
    assert any(i.id == "simulation:pick" and i.status == "pass" for i in review.items)
    assert not any(i.id == "simulation:unselected" for i in review.items)


def test_manifest_binds_conditions_and_implementation_even_without_a_bake(tmp_path):
    scene = bt.Scene()
    scene.add_box("cabinet", size=(1, 1, 1), position=(0, 0, 0))
    scene.set_part("cabinet", catalog="cabinet/r1@commit123")
    first = bt.export_cell(scene, tmp_path / "a", exports=["bom"])
    second = bt.export_cell(scene, tmp_path / "b", exports=["bom"], dt=.02)
    a, b = doc(first), doc(second)
    assert a["input_sha256"] == b["input_sha256"] and a["run_sha256"] != b["run_sha256"]
    assert a["input"]["catalog"] == [{"target": "cabinet", "id": "cabinet/r1", "revision": "commit123"}]
    assert a["generator"]["core_sha256"] and "deliverables.py" in a["generator"]["python_sha256"]
    assert a["conditions"]["physics"] == "kinematic" and not a["conditions"]["simulation_performed"]
    assert a["conditions"]["scenarios"] == []
    a["conditions"]["dt"] = .2
    first.write_text(json.dumps(a))
    checked = bt.verify_export(first)
    assert not checked["ok"] and "run fingerprint mismatch" in checked["errors"]


def test_snapshot_isolates_caller_edits_and_assets_are_hashed(tmp_path, monkeypatch):
    mesh = tmp_path / "box.obj"
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    scene = bt.Scene()
    scene.add_mesh("mesh", mesh, position=(0, 0, 0))
    scene.set_part("mesh", model="old")
    original_input = deliverables._input

    def change_caller(snapshot):
        scene.set_part("mesh", model="new")
        return original_input(snapshot)

    monkeypatch.setattr(deliverables, "_input", change_caller)
    manifest = bt.export_cell(scene, tmp_path / "package", exports=["bom"])
    assert "old" in (manifest.parent / "cell_bom.csv").read_text()
    assert scene.part("mesh")["model"] == "new"
    assert doc(manifest)["input"]["assets"] == [{"path": str(mesh), **deliverables._file(mesh)}]
    assert bt.verify_export(manifest)["same_revision"]
    monkeypatch.setattr(deliverables, "_input", original_input)
    scene.set_part("mesh", model="old")
    assert bt.verify_export(manifest, scene=scene)["ok"]
    mesh.write_text(mesh.read_text() + "# changed\n")
    assert not bt.verify_export(manifest, scene=scene)["ok"]


def test_changed_assets_abort_publication(tmp_path, monkeypatch):
    scene = bt.Scene()
    mesh = tmp_path / "triangle.obj"
    mesh.write_text("v 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n")
    scene.add_mesh("mesh", mesh, position=(0, 0, 0))
    original_generator = deliverables._generator

    def changed():
        mesh.write_text(mesh.read_text() + "# edit during export\n")
        return original_generator()

    monkeypatch.setattr(deliverables, "_generator", changed)
    out = tmp_path / "package"
    with pytest.raises(ValueError, match="assets changed"):
        bt.export_cell(scene, out, exports=["bom"])
    assert not out.exists()


def test_usd_auxiliary_assets_have_provenance(tmp_path):
    from test_usd_robot import ARM

    stage = tmp_path / "arm.usda"
    stage.write_text(ARM)
    scene = bt.Scene(bt.Robot.from_usd(stage))
    scene.sequence("idle").step("wait", transition=bt.seq.elapsed(.03))
    manifest = bt.export_cell(scene, tmp_path / "package", exports=["usd", "report"], clearance_dt=None)
    checked = bt.verify_export(manifest)
    assert checked["same_revision"], checked["errors"]
    assets = [r for r in checked["files"] if r["kind"] == "usd_asset"]
    assert assets and all(r["origin"] == "generated" and r["input_sha256"] == checked["input_sha256"] for r in assets)
    (manifest.parent / assets[0]["path"]).write_text("changed geometry")
    assert not bt.verify_export(manifest)["ok"]


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
    assert bt.verify_export(manifest)["same_revision"] and not data["issues"]
    wrong_scope = bt.review(scene, manifest=manifest, sequences=["a"])
    assert not wrong_scope.ready


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
    assert not bt.verify_export(failed)["same_revision"]


def test_missing_extra_unsafe_and_malformed_files_are_rejected(tmp_path):
    scene = bt.Scene()
    manifest = bt.export_cell(scene, tmp_path / "package", exports=["bom"])
    data = doc(manifest)
    csv = manifest.parent / "cell_bom.csv"
    original = csv.read_bytes()
    csv.unlink()
    assert not bt.verify_export(manifest)["ok"]
    csv.write_bytes(original)
    extra = manifest.parent / "old.script"
    extra.write_text("obsolete")
    assert "unlisted files" in bt.verify_export(manifest)["errors"][0]
    extra.unlink()
    data["files"][0]["path"] = "../outside.csv"
    manifest.write_text(json.dumps(data))
    assert any("escapes package" in e for e in bt.verify_export(manifest)["errors"])
    manifest.write_text("[]")
    assert not bt.verify_export(manifest)["ok"]


def test_existing_directory_and_invalid_inputs_are_not_modified(tmp_path):
    scene = bt.Scene()
    out = tmp_path / "existing"
    out.mkdir()
    (out / "notes.txt").write_text("user work")
    with pytest.raises(ValueError, match="must be empty"):
        bt.export_cell(scene, out)
    assert list(out.iterdir()) == [out / "notes.txt"]
    for options in ({"name": "../escape"}, {"dt": float("nan")}, {"sequences": ["missing"]}):
        with pytest.raises(ValueError):
            bt.export_cell(scene, tmp_path / "invalid", **options)
    assert not (tmp_path / "invalid").exists()


def test_cli_verification_and_review_use_the_manifest(tmp_path, capsys):
    scene = bt.Scene()
    cell = tmp_path / "cell.botrail"
    scene.save_project(cell)
    out = tmp_path / "package"
    assert _cli.main(["export", str(cell), "--out", str(out), "--bom", "--report"]) == 0
    exported = json.loads(capsys.readouterr().out)
    assert exported["same_revision"]
    manifest = exported["manifest"]
    assert _cli.main(["verify-export", manifest, "--cell", str(cell)]) == 0
    assert json.loads(capsys.readouterr().out)["same_revision"]
    assert _cli.main(["review", str(cell), "--manifest", manifest, "--require", "deliverables"]) == 0
    assert json.loads(capsys.readouterr().out)["ready"]
    (out / "cell_bom.csv").write_text("changed")
    assert _cli.main(["verify-export", manifest]) == 1
    assert not json.loads(capsys.readouterr().out)["ok"]
