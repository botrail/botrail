"""`scene.interlocks()` — the interlock table (E4 of
design/design-cell-engineering.md): every output a step switches against
the condition that admits the step, the inputs classified and a signal
traced to its writers. Derived from the sequences, so it agrees with the
PLCopen SFC and the bake by construction."""

import json
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


def cell() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("door", (0.1, 0.1, 0.1), (0.6, 0.0, 0.2))
    scene.add_linear_axis("gate", objects=["door"], axis=(0, 0, 1), speed=0.5, range=(0.0, 0.4),
                          stops={"closed": 0.0, "open": 0.4})
    scene.add_zone_sensor("mat", position=(0.5, 0.0, 0.1), size=(0.4, 0.4, 0.2))
    scene.define_signal("go")
    scene.add_io_node("PLC1", kind="plc", programs=["cell"])
    plc = scene.sequence("cell")
    plc.step("wait", transition=bt.seq.all_of(bt.seq.signal("mat", False), bt.seq.signal("gate/closed")))
    plc.step("open", actions=[bt.seq.move_to("gate", "open")], transition=bt.seq.device_done("gate"))
    plc.step("release", actions=[bt.seq.set_signal("go")], transition=bt.seq.elapsed(0.5))
    other = scene.sequence("robot")
    other.step("idle", transition=bt.seq.signal("go"))
    other.step("report", actions=[bt.seq.set_signal("go", False)], transition=bt.seq.elapsed(0.2))
    return scene


def test_rows_read_the_guards_and_trace_the_writers(tmp_path: Path) -> None:
    scene = cell()
    table = scene.interlocks()
    assert table.sequences == ["cell", "robot"] and len(table) == 3 and table.io_error is None
    rows = {(r["program"], r["step"]): r for r in table.rows}
    # The gate opens only with the mat clear and the gate confirmed closed:
    # a sensor and a device lane, the lane traced to the axis's commands.
    gate = rows[("cell", "open")]
    assert (gate["kind"], gate["output"], gate["host"]) == ("device", "gate → open", "PLC1")
    assert gate["condition"] == "(NOT mat AND gate/closed)" and gate["after"] == ["wait"]
    assert [(i["name"], i["kind"]) for i in gate["inputs"]] == [("mat", "sensor"), ("gate/closed", "device_lane")]
    assert gate["inputs"][1]["written_by"] == ["cell/open"]
    # `go` goes up once the gate is in position; the robot's `report`
    # follows `go`, which both programs write.
    go = rows[("cell", "release")]
    assert go["condition"] == "INPOS(gate)" and go["inputs"][0]["kind"] == "device"
    report = rows[("robot", "report")]
    assert report["condition"] == "go" and report["output"] == "go := FALSE"
    assert report["inputs"][0]["written_by"] == ["cell/release", "robot/report"]
    # Renderings.
    md = table.to_markdown()
    assert "## `cell` on PLC1" in md
    assert "| open | device `gate → open` | `(NOT mat AND gate/closed)` | wait |" in md
    lines = table.to_csv().splitlines()
    assert lines[0] == "program,host,step,kind,target,output,condition,after,inputs" and len(lines) == 4
    for ext in ("md", "csv", "json"):
        table.save(tmp_path / f"interlocks.{ext}")
    scene.export_interlocks(tmp_path / "x.json", ["cell"])
    doc = json.loads((tmp_path / "x.json").read_text())
    assert doc["sequences"] == ["cell"] and [r["step"] for r in doc["rows"]] == ["open", "release"]
    with pytest.raises(ValueError, match="unknown sequence"):
        scene.interlocks(["nope"])
    with pytest.raises(ValueError, match="unknown format"):
        table.save(tmp_path / "interlocks.txt")


def test_the_cli_writes_the_table_with_the_document_set(tmp_path: Path) -> None:
    from botrail._cli import main

    scene = cell()
    scene.save_project(tmp_path / "cell.botrail")
    assert main(["export", str(tmp_path / "cell.botrail"), "--out", str(tmp_path / "out"), "--interlocks"]) == 0
    md = (tmp_path / "out" / "cell_interlocks.md").read_text()
    assert md.startswith("# Interlock table — cell, robot") and "gate → open" in md
    assert (tmp_path / "out" / "cell_interlocks.csv").read_text().count("\n") == 4
