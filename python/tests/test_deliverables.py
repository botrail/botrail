"""The layout sheet and the cell report (D3 of
design/design-cell-engineering.md), and the deliverable set as one thing.

`scene.layout()` / `export_layout()` project the scene onto the floor —
SVG for the reviewer, R12 DXF for the 2D CAD, JSON for a front end;
`scene.footprint()` is the extent that projection measures;
`scene.cell_report()` gathers the numbers (cycles, clearance, I/O, scenario
matrix, BOM totals, footprint) plus the SHA-256 of every file written from
the same scene. These tests pin what is drawn, what is measured, and — the
point of it all — that a layout edit shows up in exactly the deliverables
it touches, by name.
"""

import hashlib
import json
import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path[:0] = [str(EXAMPLES / d) for d in ("engineering", "export")]


def small_cell() -> bt.Scene:
    """A floor slab (ground), a table, a fenced group, a conveyor with a
    beam, a frame — one of everything the sheet draws."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("floor", size=(10.0, 10.0, 0.05), position=(0.0, 0.0, -0.025))
    scene.add_box("table", size=(1.0, 0.5, 0.7), position=(1.5, 0.0, 0.35))
    scene.set_part("table", model="T-1000")
    for i in range(3):
        scene.add_box(f"fence/p{i}", size=(1.0, 0.04, 2.0), position=(-1.0 + i, 1.5, 1.0))
    scene.set_part("fence", category="structure.fence", model="ST20", qty=3)
    scene.add_conveyor(
        "belt", zone_position=(0.0, -1.0, 0.5), zone_size=(2.0, 0.4, 0.2), velocity=(0.2, 0.0, 0.0), running=False
    )
    scene.add_beam_sensor("eye", frm=(0.5, -1.3, 0.5), to=(0.5, -0.7, 0.5))
    scene.add_frame("env/World/mount", position=(0.0, 0.0, 0.0))
    return scene


# --------------------------------------------------------------- layout


def test_layout_json_draws_the_scene_and_keeps_ground_out_of_the_footprint() -> None:
    scene = small_cell()
    sheet = json.loads(scene.layout("json"))
    by_name = {}
    for item in sheet["items"]:
        by_name.setdefault(item["name"], []).append(item)
    # The floor is ground: drawn, but faint and outside the extents.
    assert by_name["floor"][0]["layer"] == "ground"
    fp = sheet["footprint"]
    assert fp["min"] == pytest.approx([-1.5, -1.2]) and fp["max"] == pytest.approx([2.0, 1.52])
    assert fp["height"] == pytest.approx(2.0)
    # Every kind of thing is on its layer.
    assert by_name["table"][0]["layer"] == "equipment"
    assert by_name["belt"][0]["layer"] == "device" and by_name["belt"][0]["dashed"]
    assert any(i["shape"]["shape"] == "arrow" for i in by_name["belt"])
    assert by_name["eye"][0]["layer"] == "sensor"
    assert by_name["env/World/mount"][0]["layer"] == "frame"
    assert by_name["simple_arm"][0]["layer"] == "robot"
    # Labels: the pinned table and fence read with their models, the frame
    # by its last segment; the floor is not labelled.
    texts = {i["shape"]["text"] for i in sheet["items"] if i["shape"]["shape"] == "text"}
    assert {"table (T-1000)", "fence (ST20)", "belt", "eye", "mount", "simple_arm"} <= texts
    assert "floor" not in texts
    # The footprint query says the same thing.
    assert scene.footprint() == pytest.approx(
        {"min": [-1.5, -1.2], "max": [2.0, 1.52], "width": 3.5, "depth": 2.72, "area": 3.5 * 2.72, "height": 2.0}
    )
    # A higher ground threshold swallows the table.
    assert scene.footprint(ground_z=1.0)["height"] == pytest.approx(2.0)
    assert scene.footprint(ground_z=1.0)["min"] == pytest.approx([-1.5, -1.2])
    without_labels = json.loads(scene.layout("json", labels=False, frames=False, grid=None))
    assert not any(i["layer"] in ("label", "frame", "grid") for i in without_labels["items"])


def test_layout_svg_and_dxf_are_well_formed_documents(tmp_path: Path) -> None:
    scene = small_cell()
    svg = scene.layout("svg", scale=50, title="small cell")
    assert svg.startswith("<svg ") and svg.rstrip().endswith("</svg>")
    assert "small cell — plan view, 3.50 × 2.72 m" in svg
    assert "<title>table</title>" in svg and 'class="dashed"' in svg
    dxf = scene.layout("dxf")
    assert dxf.startswith("0\nSECTION\n2\nHEADER\n") and dxf.rstrip().endswith("0\nEOF")
    for layer in ("EQUIPMENT", "GROUND", "ROBOT", "DEVICE", "SENSOR", "FRAME", "LABEL", "DIM", "GRID"):
        assert f"\n8\n{layer}\n" in dxf
    # Millimetres by default: the table's far edge at x = 2 m reads 2000.0.
    assert "\n2000.0\n" in dxf
    assert "\n2.0000\n" in scene.layout("dxf", units="m")
    with pytest.raises(ValueError, match="units must be"):
        scene.layout("dxf", units="in")
    with pytest.raises(ValueError, match="unknown format"):
        scene.layout("pdf")

    scene.export_layout(tmp_path / "cell.svg", scale=50, title="small cell")
    scene.export_layout(tmp_path / "cell.dxf")
    scene.export_layout(tmp_path / "cell.json")
    scene.export_layout(tmp_path / "cell.txt", format="svg", scale=50, title="small cell")
    assert (tmp_path / "cell.svg").read_text() == svg
    assert (tmp_path / "cell.dxf").read_text() == dxf
    assert (tmp_path / "cell.txt").read_text() == svg
    assert json.loads((tmp_path / "cell.json").read_text())["footprint"]["height"] == pytest.approx(2.0)
    with pytest.raises(ValueError, match="unknown extension"):
        scene.export_layout(tmp_path / "cell.pdf")

    # A real DXF reader (when installed) opens it clean, entities on their
    # layers.
    ezdxf = pytest.importorskip("ezdxf")
    from ezdxf import recover

    doc, auditor = recover.readfile(str(tmp_path / "cell.dxf"))
    assert not auditor.has_errors, auditor.errors
    layers = {e.dxf.layer for e in doc.modelspace()}
    assert {"EQUIPMENT", "GROUND", "ROBOT", "DEVICE", "SENSOR", "FRAME", "LABEL"} <= layers
    kinds = {e.dxftype() for e in doc.modelspace()}
    assert {"POLYLINE", "CIRCLE", "LINE", "TEXT"} <= kinds
    assert ezdxf.__version__  # keeps the import honest


# --------------------------------------------------------------- report


def pick_cell():
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene)
    pick.wire_cell(scene)
    scene.set_part("conv", manufacturer="MISUMI", model="GVL-900", mass_kg=32)
    scene.add_scenario("beam_stuck", faults=[bt.io.stuck("part_at_pick", False)])
    return scene


def test_cell_report_gathers_cycles_io_scenarios_bom_and_hashes(tmp_path: Path) -> None:
    scene = pick_cell()
    runs = scene.simulate_scenarios(["pick"], max_duration=30.0)
    baseline = runs["baseline"]
    bom_path = tmp_path / "bom.csv"
    scene.export_bom(bom_path)
    report = scene.cell_report(
        {"baseline": baseline, "ng": runs["ng_part"]},
        scenarios=runs,
        deliverables=[bom_path],
        title="pick",
    )
    assert report.title == "pick"
    assert repr(report).startswith('CellReport("pick", 2 cycle(s)')
    # Cycles: the bake's numbers, step spans, utilization, branch taken,
    # and the clearance re-scanned against the snapshot.
    assert report.cycle_time("baseline") == pytest.approx(baseline.duration)
    assert report.cycle_time() == pytest.approx(baseline.duration)
    assert report.cycle_time("nope") is None
    cycles = {c["name"]: c for c in report.cycles}
    assert cycles["baseline"]["sequences"] == ["pick"]
    assert cycles["ng"]["scenario"] == "ng_part"
    assert [s["name"] for s in cycles["baseline"]["steps"]][:3] == ["feed", "await part", "halt"]
    assert cycles["baseline"]["branches"] == [["pick", "judge", 0]]
    assert cycles["ng"]["branches"] == [["pick", "judge", 1]]
    assert 0 < cycles["baseline"]["robots"][0]["utilization"] < 1
    assert cycles["baseline"]["clearance"]["distance"] == pytest.approx(float(baseline.min_clearance()))
    assert report.min_clearance() == pytest.approx(
        min(float(baseline.min_clearance()), float(runs["ng_part"].min_clearance()))
    )
    # I/O: four points on the UR node, all bound, no findings.
    assert report.io["points"] == 4 and report.io["unbound"] == 0 and report.io["findings"] == []
    assert report.io["by_kind"] == {"DI": 2, "DO": 2}
    assert report.io["nodes"] == [{"name": "UR", "kind": "robot_controller", "bound": 4, "channels": 16}]
    assert report.io_error is None
    # Scenarios: two runs and the stuck beam's stall.
    rows = {s["name"]: s for s in report.scenarios}
    assert rows["baseline"]["ok"] and rows["ng_part"]["ok"] and not rows["beam_stuck"]["ok"]
    assert "timed out" in rows["beam_stuck"]["error"]
    # BOM totals and the footprint.
    assert report.bom["rows"] == 4 and report.bom["totals"] == {"mass_kg": 32.0}
    assert report.bom["by_category"] == {"conveyor": 1, "robot": 1, "robot_controller": 1, "sensor.photoelectric": 1}
    assert report.bom["unidentified"] == 3  # the arm, the eye and the controller still need numbers
    assert report.footprint["area"] > 0 and report.footprint == scene.footprint()
    # Deliverables: the file, its size, its digest.
    (d,) = report.deliverables
    assert d["path"] == str(bom_path)
    assert d["bytes"] == bom_path.stat().st_size
    assert d["sha256"] == hashlib.sha256(bom_path.read_bytes()).hexdigest()
    # Rendered forms agree with the sections.
    md = report.to_markdown()
    assert md.startswith("# pick — cell report\n")
    assert "| Scenarios | 2/3 passed |" in md and "| Deliverables | 1 files hashed |" in md
    assert "## Cycle `ng` (scenario `ng_part`)" in md
    doc = json.loads(report.to_json())
    assert doc["io"]["points"] == 4 and doc["footprint"] == report.footprint
    report.save(tmp_path / "report.md")
    report.save(tmp_path / "report.json")
    assert (tmp_path / "report.md").read_text() == md
    assert json.loads((tmp_path / "report.json").read_text()) == doc
    with pytest.raises(ValueError, match="unknown extension"):
        report.save(tmp_path / "report.html")


def test_cell_report_accepts_the_other_timeline_shapes_and_no_bake() -> None:
    scene = pick_cell()
    runs = scene.simulate_scenarios(["pick"], max_duration=30.0)
    # A bare timeline is named after its programs; a list after program
    # and scenario; a ScenarioRuns alone supplies both cycles and rows.
    one = scene.cell_report(runs["baseline"], clearance_dt=None)
    assert [c["name"] for c in one.cycles] == ["pick"]
    assert one.cycles[0]["clearance"] is None and one.min_clearance() is None
    many = scene.cell_report([runs["baseline"], runs["ng_part"]], clearance_dt=None)
    assert [c["name"] for c in many.cycles] == ["pick", "pick (ng_part)"]
    from_runs = scene.cell_report(scenarios=runs, clearance_dt=None)
    assert [c["name"] for c in from_runs.cycles] == ["baseline", "ng_part"]
    assert len(from_runs.scenarios) == 3
    # No bake at all still reports the static side.
    static = scene.cell_report()
    assert static.cycles == [] and static.cycle_time() is None
    assert "| Cycle time | — (no bake supplied) |" in static.to_markdown()
    assert static.title == "simple_arm cell"
    with pytest.raises(ValueError, match="timelines must be"):
        scene.cell_report("pick")
    with pytest.raises(ValueError, match="clearance_dt must be positive"):
        scene.cell_report(runs["baseline"], clearance_dt=0.0)


# --------------------------------------------- the deliverable set as one


def deliverable_set(scene: bt.Scene, out: Path) -> dict[str, str]:
    """Writes the static deliverables and returns `{name: sha256}`."""
    out.mkdir(exist_ok=True)
    scene.export_bom(out / "bom.csv")
    scene.export_io_list(out / "io.csv")
    scene.export_layout(out / "layout.svg")
    scene.export_layout(out / "layout.dxf")
    (out / "cell.py").write_text(scene.generate_python())
    report = scene.cell_report(deliverables=sorted(out.iterdir()))
    return {Path(d["path"]).name: d["sha256"] for d in report.deliverables}


def test_a_layout_edit_changes_exactly_the_deliverables_it_touches(tmp_path: Path) -> None:
    """The regression the document set makes possible: after moving the
    photo-eye, the layout sheet and the generated script change and the
    BOM and the I/O list do not — named, not guessed. Adding a fence panel
    changes the BOM too."""
    scene = pick_cell()
    before = deliverable_set(scene, tmp_path / "before")

    moved = pick_cell()
    moved.remove_sensor("part_at_pick")
    moved.add_beam_sensor("part_at_pick", frm=(0.35, 0.25, 0.03), to=(0.35, 0.45, 0.03), watch=["part"])
    after = deliverable_set(moved, tmp_path / "after")
    changed = sorted(name for name in before if before[name] != after[name])
    assert changed == ["cell.py", "layout.dxf", "layout.svg"]

    fenced = pick_cell()
    fenced.add_box("fence/p0", size=(1.0, 0.04, 1.8), position=(0.0, 1.0, 0.9))
    fenced.set_part("fence", category="structure.fence", model="ST20", qty=1)
    after_fence = deliverable_set(fenced, tmp_path / "fenced")
    changed = sorted(name for name in before if before[name] != after_fence[name])
    assert changed == ["bom.csv", "cell.py", "layout.dxf", "layout.svg"]
    # And the same source, twice, is byte-identical.
    again = deliverable_set(pick_cell(), tmp_path / "again")
    assert again == before


# ----------------------------------------------------------------- demo


def test_deliverables_demo_writes_the_document_set(tmp_path: Path) -> None:
    import cell_deliverables_demo as demo

    scene = demo.build()
    report = demo.deliver(scene, tmp_path)
    names = sorted(p.name for p in tmp_path.iterdir())
    assert names == [
        "cell.botrail",
        "cell.plcopen.xml",
        "cell.py",
        "cell_bom.csv",
        "cell_bom.md",
        "cell_cycle.usda",
        "cell_io.csv",
        "cell_layout.dxf",
        "cell_layout.svg",
        "cell_report.json",
        "cell_report.md",
        "cell_topology.mmd",
        "pick_cell.script",
    ]
    assert len(report.deliverables) == 11
    assert report.bom["unidentified"] == 0
    # 2.4 + 1.6 + 2.4 + 1.6 m of fence at 1 m pitch: 2+2+2+2 panels, one of
    # them the door.
    assert report.bom["by_category"]["structure.fence"] == 7
    assert report.bom["by_category"]["structure.door"] == 1
    assert report.bom["by_category"]["structure.fence.post"] == 8
    assert [s["ok"] for s in report.scenarios] == [True, True, False]
    assert report.io["unbound"] == 0
    # The fence pitch drives the BOM and the sheet together.
    coarse = demo.pick.build_cell()
    demo.pick.author_sequence(coarse)
    demo.pick.wire_cell(coarse)
    demo.furnish(coarse, fence_pitch=2.0)
    fence_rows = {row["category"]: row["qty"] for row in coarse.bom().rows}
    assert fence_rows["structure.fence"] < 7 and fence_rows["structure.fence.post"] < 8
    assert coarse.layout("json") != scene.layout("json")
