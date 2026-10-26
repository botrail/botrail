"""Virtual commissioning, VC4 (design-virtual-commissioning.md): tests as a
table — `TestCase`, the expectations, `run_fat` offline (loopback) and
online (a link), the rows derived from the interlock table and the safety
inputs, the cases as data, and the sheet in the report, the document set
and the command line.
"""

import json
import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
for sub in ("export", "machining"):
    sys.path.insert(0, str(EXAMPLES / sub))
sys.path.insert(0, str(Path(__file__).resolve().parent))

vc = bt.vc


# ------------------------------------------------------------------ cells


def pick_cell() -> bt.Scene:
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene)
    pick.wire_cell(scene)
    return scene


def gate_cell(broken: bool = False, safety: bool = False) -> bt.Scene:
    """A belt feeding a part to a beam, gated by a door switch: every
    transition reads a lane, so every interlock row but the first can be
    pinned false. `broken` drops the beam's gate (the belt stops on
    nothing); `safety` declares the door switch a safety input."""
    arm = bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")
    scene = bt.Scene(arm, name="arm")
    scene.add_box("part", size=(0.05, 0.05, 0.05), position=(0.5, 0.3, 0.03))
    scene.add_conveyor("belt", zone_position=(0.5, 0.3, 0.05), zone_size=(1.0, 0.2, 0.1),
                       velocity=(0.1, 0.0, 0.0), running=False)
    scene.add_beam_sensor("beam", frm=(0.7, 0.2, 0.03), to=(0.7, 0.4, 0.03), watch=["part"])
    scene.define_signal("door_closed", initial=True)
    for name in ("start", "gripped", "released"):
        scene.define_signal(name)
    if safety:
        scene.declare_io("door_closed", role="input", safety=True)
    sq = scene.sequence("line")
    sq.step("arm_ready", actions=[bt.seq.set_signal("start")], transition=bt.seq.signal("door_closed"))
    sq.step("feed", actions=[bt.seq.start("belt")],
            transition=bt.seq.immediately() if broken else bt.seq.rising("beam"))
    sq.step("halt", actions=[bt.seq.stop("belt"), bt.seq.set_signal("gripped")],
            transition=bt.seq.all_of(bt.seq.signal("gripped"), bt.seq.elapsed(0.2)))
    sq.step("clear", actions=[bt.seq.set_signal("released")], transition=bt.seq.signal("released"))
    sq.step("done", actions=[bt.seq.set_signal("start", False)])
    return scene


def verdicts(fat) -> dict[str, str]:
    return {c.name: c.verdict for c in fat}


# ----------------------------------------------------------- expectations


def test_expectations_read_the_window_after_the_moment() -> None:
    scene = pick_cell()
    cases = [
        vc.TestCase("cycle", expect=[vc.cycle_within(12.0), vc.step_reached("pick", "place"), vc.clearance(0.0),
                                     vc.rises("vacuum", within=7.0), vc.falls("vacuum")]),
        vc.TestCase("beam open", faults=[bt.io.open("part_at_pick")], at=vc.step("pick", "await part"),
                    expect=[vc.never("vacuum", during=5.0), vc.stalls("pick", at="await part"), vc.stalls()]),
        vc.TestCase("ng part", faults=[bt.io.stuck("spec_ok", False)],
                    expect=[vc.step_reached("pick", "to reject"), vc.never(vc.step("pick", "place"))]),
        vc.TestCase("held", at=vc.rising("vacuum"), for_=1.0,
                    expect=[vc.held("vacuum", True, for_=0.5), vc.no_motion("simple_arm", during=0.2)]),
        vc.TestCase("wrong", expect=[vc.never("vacuum"), vc.cycle_within(1.0), vc.stalls(), vc.held("vacuum", False),
                                     vc.no_motion(), vc.clearance(10.0), vc.rises("vacuum", within=1.0),
                                     vc.step_reached("pick", "to reject")]),
    ]
    fat = vc.run_fat(scene, cases, ["pick"])
    assert verdicts(fat) == {"cycle": "pass", "beam open": "pass", "ng part": "pass", "held": "pass", "wrong": "fail"}
    assert not fat.ok and [c.name for c in fat.failed] == ["wrong"]
    assert fat.counts == {"pass": 4, "fail": 1, "n/a": 0, "error": 0}
    # The horizon is the design's cycle with room: 1.5 × 11.35 s + 1 s.
    bake = scene.simulate_sequence("pick")
    assert fat.horizon == pytest.approx(1.5 * bake.duration + 1.0, abs=1e-3)
    assert fat.mode == "offline (loopback over UR)"
    # The window starts at the moment: the beam opens when the program
    # reaches `await part`, the checks count from there.
    beam = fat["beam open"]
    assert beam.t0 == pytest.approx(1.19, abs=0.02) and beam.t1 == pytest.approx(beam.t0 + fat.horizon, abs=0.02)
    assert [c.ok for c in beam.checks] == [True, True, True]
    assert "await part" in beam.checks[1].detail
    # Every wrong expectation says what it saw.
    wrong = {c.expectation: c for c in fat["wrong"].checks}
    assert all(c.ok is False for c in wrong.values())
    assert "high at 5.85 s" in wrong["never vacuum"].detail
    assert "done at 11.35 s" in wrong["the cycle completes within 1.00 s"].detail
    assert "completed" in wrong["the cycle stalls"].detail
    assert "moved" in wrong["no robot moves"].detail
    assert "mm" in wrong["clearance ≥ 10000 mm"].detail
    assert "no rising edge" in wrong["vacuum rises within 1.00 s"].detail
    assert "not entered" in wrong["pick/to reject reached"].detail
    # The run behind a row is kept: the evidence of a red row.
    assert fat.timelines["ng part"].branches == [("pick", "judge", 1)]
    assert fat.timelines["cycle"].duration == bake.duration


def test_rows_that_cannot_be_judged_are_not_applicable() -> None:
    scene = pick_cell()
    cases = [
        # The moment never comes: the gauge never rises in the nominal cycle.
        vc.TestCase("never comes", at=vc.rising("spec_ok"), expect=[vc.rises("vacuum")]),
        # A precondition that does not take: the reject arm is not on the
        # nominal path, so the row is n/a, not failed.
        vc.TestCase("precondition", given=[vc.step_reached("pick", "to reject")], expect=[vc.never("vacuum")]),
        # A row the author skipped carries its reason.
        vc.TestCase("skipped", skip="no fixture for this one yet", expect=[vc.rises("vacuum")]),
        # A lane the cell does not have is an error, not a pass.
        vc.TestCase("typo", expect=[vc.rises("vaccum")]),
        vc.TestCase("bad fault", faults=[bt.io.stuck("nothing", False)], expect=[vc.rises("vacuum")]),
    ]
    fat = vc.run_fat(scene, cases, ["pick"], horizon=12.0)
    assert verdicts(fat) == {"never comes": "n/a", "precondition": "n/a", "skipped": "n/a", "typo": "fail",
                             "bad fault": "error"}
    assert "did not come" in fat["never comes"].detail
    assert fat["precondition"].detail.startswith("precondition not met")
    assert fat["skipped"].detail == "no fixture for this one yet"
    assert "vaccum" in fat["typo"].checks[0].detail
    assert "nothing" in fat["bad fault"].detail
    # n/a rows do not fail the sheet; the fail and the error do.
    assert not fat.ok and {c.name for c in fat.failed} == {"typo", "bad fault"}
    assert fat.horizon == 12.0


def test_bad_cases_are_refused_when_written() -> None:
    with pytest.raises(TypeError, match="expectations"):
        vc.TestCase("x", expect=[lambda tl: True])
    with pytest.raises(TypeError, match="predicate"):
        vc.TestCase("x", at={"rising": "beam"})
    with pytest.raises(ValueError, match="positive"):
        vc.TestCase("x", for_=0.0)
    with pytest.raises(ValueError, match="name"):
        vc.TestCase("")
    with pytest.raises(TypeError, match="lane name or bt.vc.step"):
        vc.never(3)
    with pytest.raises(ValueError, match="positive"):
        vc.cycle_within(0.0)
    with pytest.raises(ValueError, match="horizon"):
        vc.run_fat(pick_cell(), [vc.TestCase("x")], ["pick"], horizon=-1.0)


# ------------------------------------------------------- derived rows


def test_interlock_rows_become_tests_and_a_dropped_transition_turns_its_row_red() -> None:
    design = gate_cell()
    table = design.interlocks(["line"])
    cases = vc.interlock_tests(design, "line")
    # `halt` switches two outputs: two rows, one gate.
    assert len(cases) == len(table.rows) == 6
    # The table itself names the pins: a level test on high pinned low, a
    # rising edge pinned low; the first step is entered at start regardless.
    by_step = {r["step"]: r for r in table.rows}
    assert by_step["arm_ready"]["undeniable"] and by_step["arm_ready"]["denial"] == []
    assert by_step["feed"]["denial"] == [["door_closed", False]]
    assert by_step["halt"]["denial"] == [["beam", False]]
    assert by_step["clear"]["denial"] == [["gripped", False]]
    assert by_step["done"]["denial"] == [["released", False]]
    assert by_step["feed"]["inputs"][0]["pin"] is False
    skipped = [c for c in cases if c.skip]
    assert len(skipped) == 1 and "first step" in skipped[0].skip
    assert all(c.source == "interlock" for c in cases)

    # The design holds every one of its own interlocks.
    fat = vc.run_fat(design, cases, ["line"])
    assert fat.ok and fat.counts == {"pass": 5, "fail": 0, "n/a": 1, "error": 0}
    halt = fat["IL-03 halt: belt: stop"]
    assert [c.ok for c in halt.checks] == [True, True]
    assert halt.checks[0].expectation == "line/feed reached" and halt.checks[1].expectation == "never step line/halt"

    # The same rows against an implementation that dropped the beam's
    # gate: the belt stops on nothing, and only that gate's rows go red.
    broken = vc.run_fat(gate_cell(broken=True), cases, ["line"])
    assert not broken.ok
    assert [c.name for c in broken.failed] == ["IL-03 halt: belt: stop", "IL-04 halt: gripped := TRUE"]
    red = broken["IL-03 halt: belt: stop"]
    assert red.checks[0].ok is True and red.checks[1].ok is False
    assert "entered at" in red.checks[1].detail
    assert "RISING(beam)" in red.notes


def test_safety_inputs_derive_their_own_rows() -> None:
    scene = gate_cell(safety=True)
    cases = vc.safety_tests(scene, ["line"])
    assert [c.name for c in cases] == ["SF-01 door_closed dropped"]
    case = cases[0]
    # The program tests the switch high, so dropping it means pinning it low.
    assert case.faults == [bt.io.stuck("door_closed", False)] and case.source == "safety"
    fat = vc.run_fat(scene, cases, ["line"])
    assert fat.ok and fat["SF-01 door_closed dropped"].checks[0].detail.startswith("waiting at line/arm_ready")
    # Without the declaration there is no safety input to derive from.
    assert vc.safety_tests(gate_cell(), ["line"]) == []


def test_the_machine_tending_fat_rows_as_test_cases_give_the_scenario_verdicts() -> None:
    import machine_tending_demo as demo

    scene, hs = demo.build()
    names = ["tend", hs.program]
    tl = scene.simulate_sequences(names, max_duration=240.0)
    runs = scene.simulate_scenarios(names, max_duration=tl.duration + 20.0)
    cases = [vc.TestCase("nominal cycle", expect=[vc.cycle_within(tl.duration + 1.0)]), *demo.fat_cases(scene, hs)]
    fat = vc.run_fat(scene, cases, names)
    assert fat.ok and fat.counts["pass"] == 4
    # The sheet's verdict is the matrix's: each fault refused at the step
    # the scenario run stalled in.
    stalled = {
        "door switch stuck": runs.errors["door_switch_stuck"],
        "clamp button open": runs.errors["clamp_button_open"],
        "E-stop pressed": runs.errors["estop_pressed"],
    }
    for name, error in stalled.items():
        row = fat[name]
        assert row.verdict == "pass"
        waiting = row.checks[0].detail  # "waiting at vmc/wait_start at 86.2 s"
        step = waiting.split("waiting at ")[1].split(" at ")[0]
        assert step in error, (step, error)
    assert fat["nominal cycle"].checks[0].detail.startswith(f"done at {tl.duration:.2f} s")


def test_the_openplc_demo_rows_hold_offline() -> None:
    import importlib.util

    path = EXAMPLES / "commissioning" / "openplc_pick_cell" / "run_demo.py"
    spec = importlib.util.spec_from_file_location("openplc_pick_cell_demo", path)
    demo = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(demo)
    scene = demo.build()
    ref = demo.reference(scene)
    cases = demo.fat_cases(ref.duration, 0.32)
    fat = vc.run_fat(scene, cases, ["pick"], horizon=ref.duration + demo.TIMEOUT_S + 10.0)
    assert fat.ok and fat.counts == {"pass": 3, "fail": 0, "n/a": 0, "error": 0}, fat.to_markdown()
    # The rows say what they saw: the fault lamp after the timer, the
    # program parked at `ready` with nothing pressed.
    dark = {c.expectation: c.detail for c in fat["TC-02 no part at the beam"].checks}
    assert dark[f"fault rises within {demo.TIMEOUT_S + 2.0 + 0.32:.2f} s"].startswith("at ")
    assert fat["TC-03 no start, no cycle"].checks[0].detail.startswith("waiting at pick/ready")
    assert fat["TC-03 no start, no cycle"].t1 == pytest.approx(5.0, abs=0.02)


# ----------------------------------------------------------- cases as data


def test_cases_round_trip_through_data_and_files(tmp_path: Path) -> None:
    cases = [
        vc.TestCase("TC-01 cycle", expect=[vc.cycle_within(12.0), vc.clearance(0.02, to=["part"])]),
        vc.TestCase("TC-07 beam open", faults=[bt.io.open("part_at_pick"), bt.io.stuck("spec_ok", True)],
                    forces={"vacuum": False}, press=["panel/start"], at=vc.step("pick", "await part"),
                    until=vc.done("pick"), for_=5.0, scenario="ng_part", notes="the beam's wire",
                    given=[vc.step_reached("pick", ["await part", "halt"])],
                    expect=[vc.never("vacuum", during=10.0), vc.never(vc.step("pick", "grip")),
                            vc.rises("conv", within=0.5), vc.falls("conv"), vc.held("spec_ok", True, for_=2.0),
                            vc.stalls("pick", at="await part"), vc.no_motion("simple_arm", during=3.0),
                            vc.no_motion()]),
        vc.TestCase("skipped", skip="later", source="interlock"),
    ]
    data = vc.cases_to_data(cases)
    back = vc.cases_from_data(data)
    assert vc.cases_to_data(back) == data
    assert [c.name for c in back] == [c.name for c in cases]
    assert back[1].setup == cases[1].setup
    assert [e.describe() for e in back[1].expect] == [e.describe() for e in cases[1].expect]
    assert back[1].for_ == 5.0 and back[1].scenario == "ng_part" and back[2].skip == "later"
    # Files: JSON needs nothing, YAML needs PyYAML.
    (tmp_path / "fat.json").write_text(json.dumps(data), encoding="utf-8")
    assert vc.cases_to_data(vc.load_cases(tmp_path / "fat.json")) == data
    yaml = pytest.importorskip("yaml")
    (tmp_path / "fat.yaml").write_text(yaml.safe_dump(data), encoding="utf-8")
    assert vc.cases_to_data(vc.load_cases(tmp_path / "fat.yaml")) == data
    # The hand-written shapes: a bare lane with sibling options, a step
    # as a list, derived rows spliced in.
    scene = gate_cell(safety=True)
    text = """
cases:
  - name: feed
    at: {rising: start}
    expect:
      - never: beam
        during: 0.5
      - held: {lane: door_closed, value: true, for: 0.5}
      - step_reached: [line, feed]
        within: 0.1
      - stalls: {program: line}
      - no_motion: true
  - interlock_tests: line
  - safety_tests: true
"""
    (tmp_path / "hand.yaml").write_text(text, encoding="utf-8")
    loaded = vc.load_cases(tmp_path / "hand.yaml", scene)
    assert [c.name for c in loaded][:2] == ["feed", "IL-01 arm_ready: start := TRUE"]
    assert loaded[-1].name == "SF-01 door_closed dropped"
    assert [e.describe() for e in loaded[0].expect] == [
        "never beam for 0.50 s", "door_closed high for 0.50 s", "line/feed reached within 0.10 s",
        "line stalls", "no robot moves"]
    with pytest.raises(ValueError, match="derived rows need the scene"):
        vc.cases_from_data({"cases": [{"interlock_tests": "line"}]})
    with pytest.raises(ValueError, match="names one of"):
        vc.cases_from_data({"cases": [{"name": "x", "expect": [{"rise": "a"}]}]})
    with pytest.raises(ValueError, match="stuck / open / node_down"):
        vc.cases_from_data({"cases": [{"name": "x", "faults": [{"broken": "a"}]}]})


# --------------------------------------------------------------- online


def test_the_same_sheet_runs_against_a_controller_on_a_link() -> None:
    from test_vc import two_controller_cell

    scene, names = two_controller_cell()
    with vc.loopback(scene, names, host="PLC1", max_duration=60.0) as s:
        s.run()
        rec = s.recording()
    cases = [vc.TestCase("cycle", expect=[vc.rises("cycle_done"), vc.rises("carrying", within=20.0),
                                          vc.cycle_within(30.0, "line"), vc.cycle_within(30.0, "watch")])]
    opened = []
    fat = vc.run_fat(scene, cases, names, links=lambda: {"PLC1": vc.replay(rec)}, horizon=30.0,
                     pace="sync", on_open=lambda case, session: opened.append(case.name))
    assert opened == ["cycle"]
    assert fat.mode.startswith("online (PLC1: replay")
    row = fat["cycle"]
    assert row.verdict == "pass"
    oks = {c.expectation: c.ok for c in row.checks}
    # The lanes the controller writes are visible; its own program's
    # steps are not (no monitor word), so that check is n/a — the plant's
    # program `watch` is judged as always.
    assert oks["cycle_done rises"] is True and oks["carrying rises within 20.00 s"] is True
    assert oks["line completes within 30.00 s"] is None and oks["watch completes within 30.00 s"] is True
    assert "monitor=True" in next(c.detail for c in row.checks if c.ok is None)


# ---------------------------------------------- the report, the set, the CLI


def test_the_sheet_lands_in_the_report_and_the_document_set(tmp_path: Path) -> None:
    scene = gate_cell(safety=True)
    cases = vc.interlock_tests(scene, "line") + vc.safety_tests(scene, ["line"])
    fat = vc.run_fat(scene, cases, ["line"], title="gate FAT")
    section = fat.section()
    assert section["title"] == "gate FAT" and section["json"]["ok"] is True
    report = scene.cell_report(scene.simulate_sequence("line"), sections=[section])
    assert "## gate FAT" in report.to_markdown() and "SF-01 door_closed dropped" in report.to_markdown()
    assert json.loads(report.to_json())["sections"][0]["json"]["counts"]["pass"] == 6
    # The three file shapes.
    for ext in ("md", "csv", "json"):
        fat.save(tmp_path / f"fat.{ext}")
    assert (tmp_path / "fat.md").read_text().startswith("# gate FAT — line")
    assert (tmp_path / "fat.csv").read_text().splitlines()[0].startswith("case,source,setup,verdict")
    assert json.loads((tmp_path / "fat.json").read_text())["counts"] == fat.counts
    with pytest.raises(ValueError, match="unknown format"):
        fat.save(tmp_path / "fat.txt")

    # The document set runs the derived rows and hashes the sheet in.
    manifest = bt.export_cell(gate_cell(broken=True, safety=True), tmp_path / "set", name="gate",
                              exports=["report"], clearance_dt=None, fat=cases)
    record = json.loads(manifest.read_text())
    files = {row["path"]: row for row in record["files"]}
    assert {"gate_fat.md", "gate_fat.json", "gate_report.md"} <= set(files)
    assert record["conditions"]["fat"]["cases"] == 7 and record["conditions"]["fat"]["counts"]["fail"] == 2
    assert [i["code"] for i in record["issues"]] == ["fat_case_failed", "fat_case_failed"]
    assert "IL-03 halt" in record["issues"][0]["message"]
    text = (tmp_path / "set" / "gate_report.md").read_text()
    assert "## FAT" in text.split("## Deliverables")[0] and "IL-03 halt: belt: stop" in text
    # `fat=True` derives the rows itself.
    manifest = bt.export_cell(scene, tmp_path / "derived", name="gate", exports=["report"], clearance_dt=None,
                              fat=True)
    assert json.loads(manifest.read_text())["conditions"]["fat"]["cases"] == 7
    with pytest.raises(ValueError, match="fat must be"):
        bt.export_cell(scene, tmp_path / "bad", exports=["bom"], fat=[1, 2])


def test_the_command_line_runs_the_sheet(tmp_path: Path, capsys) -> None:
    from botrail._cli import main

    scene = gate_cell(safety=True)
    cell = tmp_path / "gate.botrail"
    scene.save_project(cell)
    # Derived rows by default; JSON out; exit 0 when green.
    assert main(["fat", str(cell), "--out", str(tmp_path / "fat.md")]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["ok"] is True and out["counts"] == {"pass": 6, "fail": 0, "n/a": 1, "error": 0}
    assert out["mode"].startswith("offline") and (tmp_path / "fat.md").exists()
    # Authored cases from a file, Markdown out, exit 1 when a row fails.
    cases = {"cases": [{"name": "belt never stops", "expect": [{"never": {"step": ["line", "halt"]}}]}]}
    (tmp_path / "cases.json").write_text(json.dumps(cases), encoding="utf-8")
    assert main(["fat", str(cell), "--cases", str(tmp_path / "cases.json"), "--markdown", "--horizon", "10"]) == 1
    text = capsys.readouterr().out
    assert text.startswith("# FAT — line") and "**FAIL**" in text and "belt never stops" in text
    # Nothing to run is a usage error.
    plain = tmp_path / "plain.botrail"
    gate_cell().save_project(plain)
    assert main(["fat", str(plain), "--safety"]) == 2
    assert "no test cases" in json.loads(capsys.readouterr().out)["error"]
    # `export --fat` puts the sheet in the set.
    assert main(["export", str(cell), "--out", str(tmp_path / "set"), "--report", "--no-clearance", "--fat"]) == 0
    assert (tmp_path / "set" / "gate_fat.md").exists()
