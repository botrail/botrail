"""The `botrail` command and the `.botrail` JSON Schema (D4 of
design/design-cell-engineering.md — agent-ready).

`botrail check | simulate | export | schema` drive a cell without Python:
load a `.botrail` project or a Python cell file, lint it, bake it, write the
document set, and print JSON that whatever wrote the cell can read back.
The schema is generated from the Rust types the loader reads, so a project
that validates is a project that loads.
"""

import json
import sys
from pathlib import Path

import pytest

import botrail as bt
from botrail import _cli

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
DEMO = EXAMPLES / "cell_deliverables_demo.py"
sys.path.insert(0, str(EXAMPLES))


def run(capsys, *argv) -> tuple[int, dict]:
    code = _cli.main(list(argv))
    out = capsys.readouterr().out
    return code, json.loads(out)


# --------------------------------------------------------------- schema


def test_project_schema_is_the_loaders_contract(tmp_path: Path) -> None:
    schema = json.loads(bt.project_schema())
    assert schema["$schema"].endswith("2020-12/schema")
    assert schema["title"] == "botrail project (.botrail)"
    assert {"version", "robots", "obstacles", "sequences", "io", "parts"} <= set(schema["properties"])
    # Doc comments are the descriptions — the part of a schema an agent reads.
    assert "identity and count" in schema["$defs"]["Part"]["description"]
    # The checked-in copy is current (regenerate with:
    #   python -c "import botrail as bt; open('docs/assets/project.schema.json','w').write(bt.project_schema())").
    checked_in = Path(__file__).resolve().parents[2] / "docs" / "assets" / "project.schema.json"
    assert json.loads(checked_in.read_text()) == schema, "docs/assets/project.schema.json is stale"

    # A real project validates; a broken one does not.
    jsonschema = pytest.importorskip("jsonschema")
    import cell_deliverables_demo as demo

    scene = demo.build()
    scene.save_project(tmp_path / "cell.botrail")
    doc = json.loads((tmp_path / "cell.botrail").read_text())
    validator = jsonschema.Draft202012Validator(schema)
    assert list(validator.iter_errors(doc)) == []
    doc["parts"] = [{"target": 1}]
    assert validator.iter_errors(doc)


# ------------------------------------------------------------------ CLI


def test_check_reads_python_cells_and_projects(capsys, tmp_path: Path) -> None:
    code, out = run(capsys, "check", str(DEMO))
    assert code == 0 and out["ok"] and out["robots"] == ["simple_arm"]
    assert out["counts"]["sequences"] == 1 and out["counts"]["parts"] == 8
    # The demo's hand-typed parts carry no specs: what the cell asks of them
    # is a warning each, never an error.
    assert {f["severity"] for f in out["findings"]} <= {"warning", "info"}
    assert "spec_unknown" in {f["code"] for f in out["findings"]}
    assert out["requirements"]["lines"] == out["counts"]["bom_rows"] and out["requirements"]["short"] == 0
    # The same cell as a project file.
    import cell_deliverables_demo as demo

    demo.build().save_project(tmp_path / "cell.botrail")
    code, out2 = run(capsys, "check", str(tmp_path / "cell.botrail"))
    assert code == 0 and out2["counts"] == out["counts"]
    # A cell with a problem: a sequence that starts a motion nobody taught.
    broken = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    broken.add_beam_sensor("eye", frm=(0, 0, 0.5), to=(0, 1, 0.5))
    broken.set_part("eye", model="PZ")
    sq = broken.sequence("go")
    sq.step("move", actions=[bt.seq.motion("nowhere")])
    broken.save_project(tmp_path / "broken.botrail")
    code, out3 = run(capsys, "check", str(tmp_path / "broken.botrail"))
    assert code == 1 and not out3["ok"]
    assert any(f["severity"] == "error" for f in out3["findings"]), out3
    # An unidentified equipment line is an info finding, not a failure.
    plain = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    plain.save_project(tmp_path / "plain.botrail")
    code, out4 = run(capsys, "check", str(tmp_path / "plain.botrail"))
    assert code == 0 and [f["code"] for f in out4["findings"]] == ["unidentified_part"]


def test_check_reports_load_failures_as_json_with_exit_2(capsys, tmp_path: Path) -> None:
    code, out = run(capsys, "check", str(tmp_path / "nope.botrail"))
    assert code == 2 and not out["ok"] and "no such file" in out["error"]
    (tmp_path / "empty.py").write_text("x = 1\n")
    code, out = run(capsys, "check", str(tmp_path / "empty.py"))
    assert code == 2 and "define a top-level `scene`" in out["error"]
    (tmp_path / "boom.py").write_text("raise RuntimeError('no cell here')\n")
    code, out = run(capsys, "check", str(tmp_path / "boom.py"))
    assert code == 2 and "RuntimeError: no cell here" in out["error"]
    (tmp_path / "cell.txt").write_text("")
    code, out = run(capsys, "check", str(tmp_path / "cell.txt"))
    assert code == 2 and "unknown cell type" in out["error"]
    # A script that exposes `scene` at top level works too.
    (tmp_path / "top.py").write_text(
        "import botrail as bt\n"
        f"scene = bt.Scene(bt.Robot.from_urdf({str(EXAMPLES / 'simple_arm.urdf')!r}))\n"
    )
    code, out = run(capsys, "check", str(tmp_path / "top.py"))
    assert code == 0 and out["robots"] == ["simple_arm"]


def test_simulate_prints_the_report_and_writes_files(capsys, tmp_path: Path) -> None:
    code, report = run(capsys, "simulate", str(DEMO), "--report", str(tmp_path / "r.md"), "--usd", str(tmp_path / "c.usda"))
    assert code == 0
    assert [c["name"] for c in report["cycles"]] == ["pick"]
    assert report["cycles"][0]["duration"] == pytest.approx(11.75, abs=0.05)
    assert report["cycles"][0]["clearance"]["distance"] > 0
    assert (tmp_path / "r.md").read_text().startswith("# ")
    assert (tmp_path / "c.usda").exists()
    # The scenario matrix: the stuck beam stalls, so the exit code says look.
    code, report = run(capsys, "simulate", str(DEMO), "--scenarios", "--no-clearance", "--title", "pick")
    assert code == 1 and report["title"] == "pick"
    assert [s["ok"] for s in report["scenarios"]] == [True, True, False]
    assert [c["name"] for c in report["cycles"]] == ["baseline", "ng_part"]
    assert report["cycles"][0]["clearance"] is None
    # Markdown for people.
    code = _cli.main(["simulate", str(DEMO), "--markdown", "--no-clearance"])
    assert code == 0 and capsys.readouterr().out.startswith("# simple_arm cell — cell report")


def test_export_writes_the_document_set(capsys, tmp_path: Path) -> None:
    code, out = run(capsys, "export", str(DEMO), "--out", str(tmp_path / "docs"), "--all", "--scenarios", "--name", "pick")
    assert code == 0 and out["ok"]
    names = sorted(Path(p).name for p in out["files"])
    assert names == sorted(
        [
            "pick.botrail",
            "pick.py",
            "pick_bom.csv",
            "pick_bom.md",
            "pick_io.csv",
            "pick_topology.mmd",
            "pick.plcopen.xml",
            "pick_layout.svg",
            "pick_layout.dxf",
            "pick_baseline.usda",
            "pick_ng_part.usda",
            "pick.script",
            "pick_report.md",
            "pick_report.json",
        ]
    )
    report = json.loads((tmp_path / "docs" / "pick_report.json").read_text())
    # The report hashes what was written before it.
    assert len(report["deliverables"]) == 12
    assert all(d["sha256"] for d in report["deliverables"])
    # A subset, no bake needed.
    code, out = run(capsys, "export", str(DEMO), "--out", str(tmp_path / "some"), "--bom", "--layout")
    assert code == 0 and sorted(Path(p).name for p in out["files"]) == [
        "cell_deliverables_demo_bom.csv",
        "cell_deliverables_demo_bom.md",
        "cell_deliverables_demo_layout.dxf",
        "cell_deliverables_demo_layout.svg",
    ]


def test_schema_command(capsys, tmp_path: Path) -> None:
    code, out = run(capsys, "schema")
    assert code == 0 and out["title"] == "botrail project (.botrail)"
    code, out = run(capsys, "schema", "--out", str(tmp_path / "s.json"))
    assert code == 0 and json.loads((tmp_path / "s.json").read_text())["title"] == "botrail project (.botrail)"
