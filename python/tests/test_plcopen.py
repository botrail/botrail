"""PLCopen XML export and the trace diff — offline commissioning (D5 of
design/design-cell-engineering.md).

`scene.plcopen()` hands the sequences to a PLC IDE as SFC programs whose
variables are the I/O map's points; `tl.diff(trace)` compares the bake with
what the controller actually did, edge by edge. Neither talks to a PLC —
the file goes one way, the log comes back — and both are deterministic.
"""

import os
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "export"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

NS = {"p": "http://www.plcopen.org/xml/tc6_0201", "x": "http://www.w3.org/1999/xhtml"}


def pick_cell():
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene)
    pick.wire_cell(scene)
    return scene


def sfc_walk(xml: str, program: str) -> list[str]:
    """`step name`, `-> condition`, `action …`, `jump target` lines of one
    program's SFC, in body order."""
    doc = ET.fromstring(xml)
    out = []
    for pou in doc.findall(".//p:pou", NS):
        if pou.get("name") != program:
            continue
        for el in pou.find("p:body/p:SFC", NS):
            tag = el.tag.split("}")[1]
            if tag == "step":
                out.append(f"step {el.get('name')}{' *' if el.get('initialStep') == 'true' else ''}")
            elif tag == "transition":
                st = el.find(".//x:p", NS)
                out.append(f"-> {(st.text or '').strip()}")
            elif tag == "actionBlock":
                for a in el.findall("p:action", NS):
                    ref = a.find("p:reference", NS)
                    st = a.find(".//x:p", NS)
                    body = ref.get("name") if ref is not None else (st.text or "").strip().replace("\n", " ")
                    out.append(f"action {a.get('qualifier')} {body}")
            elif tag == "jumpStep":
                out.append(f"jump {el.get('targetName')}")
            else:
                out.append(tag)
    return out


# ------------------------------------------------------------- PLCopen


def test_plcopen_renders_the_pick_cell_as_one_sfc_program(tmp_path: Path) -> None:
    scene = pick_cell()
    xml = scene.plcopen(name="pick cell")
    assert xml.startswith('<?xml version="1.0" encoding="utf-8"?>')
    doc = ET.fromstring(xml)  # well-formed
    assert doc.tag == "{http://www.plcopen.org/xml/tc6_0201}project"
    programs = [p.get("name") for p in doc.findall(".//p:pou", NS) if p.get("pouType") == "program"]
    assert programs == ["pick"]
    walk = sfc_walk(xml, "pick")
    # The steps in order, the initial one marked, the cycle jump at the end.
    steps = [w for w in walk if w.startswith("step ")]
    assert steps == [
        "step feed *", "step await_part", "step halt", "step grip", "step hold", "step judge",
        "step place", "step release", "step to_chute", "step drop", "step return_",
    ]
    assert walk[-1] == "jump feed"
    # Actions: coils, stub FB calls, signal writes; conditions: the FB's
    # done, the R_TRIG edge, the timer, the level test.
    assert "action N conv_run := TRUE; simple_arm_motion(robot := 'simple_arm', name := 'to_pick', start := TRUE);" in walk
    assert "-> simple_arm_motion.done" in walk
    assert "action N part_at_pick_rise(CLK := part_at_pick);" in walk
    assert "-> part_at_pick_rise.Q" in walk
    assert "-> grip.T >= T#300ms" in walk
    assert "action N vacuum := TRUE;" in walk
    # The branch: divergence, the two arm conditions (otherwise = NOT the
    # other), the convergence.
    i = walk.index("selectionDivergence")
    assert walk[i + 1] == "-> spec_ok"
    assert "-> NOT (spec_ok)" in walk
    assert "selectionConvergence" in walk
    # Globals are the derived points, typed; the signal carries its initial
    # value; UR-bound points get no PLC address.
    globals_ = {v.get("name"): v for v in doc.findall(".//p:globalVars/p:variable", NS)}
    assert set(globals_) == {"conv_run", "part_at_pick", "spec_ok", "vacuum"}
    assert globals_["spec_ok"].find("p:initialValue/p:simpleValue", NS).get("value") == "TRUE"
    assert globals_["conv_run"].get("address") is None
    # Stubs used are function-block POUs; unused ones are not emitted.
    fbs = {p.get("name") for p in doc.findall(".//p:pou", NS) if p.get("pouType") == "functionBlock"}
    assert fbs == {"FB_StartMotion", "FB_Attach", "FB_Detach"}
    # Deterministic bytes; the file writer agrees with the string.
    assert xml == scene.plcopen(name="pick cell")
    scene.export_plcopen(tmp_path / "pick.plcopen.xml", name="pick cell")
    assert (tmp_path / "pick.plcopen.xml").read_text() == xml
    # A parked ending and a program subset.
    parked = scene.plcopen(["pick"], cycle=False)
    assert "jump feed" not in sfc_walk(parked, "pick") and "step end_of_cycle" in sfc_walk(parked, "pick")
    with pytest.raises(ValueError, match="unknown sequence"):
        scene.plcopen(["nope"])
    with pytest.raises(ValueError, match="no sequences"):
        bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")).plcopen()


def test_plcopen_uses_the_handshake_where_the_plc_drives_the_robot() -> None:
    """The PLC-master view: a program hosted on a PLC node drives the arms
    through start / done (and a program word), the belt through a coil
    with its address, while a program on the arm's own controller calls
    the stub — decided per program, from the I/O map."""
    from test_io_map import author_two_arm, two_arm_cell

    scene = two_arm_cell()
    author_two_arm(scene)
    scene.add_io_node("PLC1", kind="plc", programs=["pick", "belt"],
                      channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    scene.auto_assign_io()
    xml = scene.plcopen(name="two arm")
    pick = sfc_walk(xml, "pick")
    assert "action N near_start" in pick and "action N far_start" in pick
    assert "action N far_program := 1; (* far_go *)" in pick
    assert "-> near_done AND far_done" in pick
    watch = sfc_walk(xml, "watch")
    assert any(w.startswith("action N near_motion(robot := 'near', name := 'near_go'") for w in watch)
    assert "-> far_done" in watch
    belt = sfc_walk(xml, "belt")
    assert any("belt_run := TRUE;" in w and "(* magazine: feeder model, no I/O *)" in w for w in belt)
    doc = ET.fromstring(xml)
    globals_ = {v.get("name"): v.get("address") for v in doc.findall(".//p:globalVars/p:variable", NS)}
    assert globals_["belt_run"] == "%QX0.1" and globals_["beam"] == "%IX0.0"
    assert globals_["near_start"] is not None and globals_["far_program"] is None
    tasks = doc.findall(".//p:task/p:pouInstance", NS)
    assert [t.get("typeName") for t in tasks] == ["pick", "belt", "watch"]


def test_plcopen_validates_against_the_tc6_schema_when_available() -> None:
    """With the TC6 XSD at hand (`BOTRAIL_PLCOPEN_XSD`, or the OpenPLC
    Editor checkout next door) and lxml installed, the document validates."""
    lxml = pytest.importorskip("lxml")
    from lxml import etree

    candidates = [os.environ.get("BOTRAIL_PLCOPEN_XSD"),
                  str(Path.home() / "projects" / "OpenPLC_Editor" / "editor" / "plcopen" / "tc6_xml_v201.xsd")]
    xsd_path = next((c for c in candidates if c and Path(c).exists()), None)
    if xsd_path is None:
        pytest.skip("no TC6 XSD available (set BOTRAIL_PLCOPEN_XSD)")
    schema = etree.XMLSchema(etree.parse(xsd_path))
    for scene in (pick_cell(),):
        doc = etree.fromstring(scene.plcopen(name="pick").encode())
        assert schema.validate(doc), [e.message for e in schema.error_log][:5]
    from test_io_map import author_two_arm, two_arm_cell

    scene = two_arm_cell()
    author_two_arm(scene)
    scene.add_io_node("PLC1", kind="plc", programs=["pick", "belt"],
                      channels=bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0"))
    doc = etree.fromstring(scene.plcopen().encode())
    assert schema.validate(doc), [e.message for e in schema.error_log][:5]
    assert lxml.__version__


# ---------------------------------------------------------------- trace


def test_trace_diff_matches_the_bake_and_names_every_deviation(tmp_path: Path) -> None:
    scene = pick_cell()
    tl = scene.simulate_sequence("pick")
    perfect = bt.trace.from_timeline(tl)
    d = tl.diff(perfect)
    assert d.ok and d.max_offset == 0.0
    assert {s.name for s in d.signals} == {n for n, _ in tl.signals}
    # Through CSV, and through the tags of the I/O map.
    csv_text = bt.trace.to_csv(perfect)
    (tmp_path / "log.csv").write_text(csv_text.replace("part_at_pick", "BEAM1").replace("vacuum", "YV1"))
    tagged = bt.trace.load(tmp_path / "log.csv")
    assert "BEAM1" in tagged.signals and "part_at_pick" not in tagged.signals
    resolved = bt.trace.load(tmp_path / "log.csv", io=scene.io_map())
    assert "part_at_pick" in resolved.signals and "vacuum" in resolved.signals
    assert tl.diff(tmp_path / "log.csv", io=scene.io_map()).ok
    # A late edge, a missing edge, an extra pulse — each named, with times.
    sig = dict(perfect.signals)
    sig["part_at_pick"] = [(t + (0.3 if v else 0.0), v) for t, v in sig["part_at_pick"]]
    sig["vacuum"] = [e for e in sig["vacuum"] if e[1]]
    sig["conv"] = sig["conv"] + [(9.0, True), (9.1, False)]
    d2 = tl.diff(sig, tolerance=0.05)
    assert not d2.ok
    rows = {s.name: s for s in d2.signals}
    assert rows["part_at_pick"].missing == [(pytest.approx(5.55, abs=0.01), "rose")]
    assert rows["part_at_pick"].extra == [(pytest.approx(5.85, abs=0.01), "rose")]
    assert [k for _, k in rows["vacuum"].missing] == ["rose", "fell"]
    assert [k for _, k in rows["conv"].extra] == ["rose", "fell"]
    codes = {f["code"] for f in d2.findings()}
    assert codes == {"missing_edge", "extra_edge"}
    md = d2.to_markdown()
    assert md.startswith("# Trace diff — MISMATCH") and "the trace never did" in md
    import json

    doc = json.loads(d2.to_json())
    assert doc["ok"] is False and doc["signals"][0]["name"] == d2.signals[0].name
    # A tolerance that swallows the 0.3 s makes it a match with an offset.
    d3 = tl.diff({"part_at_pick": sig["part_at_pick"]}, tolerance=0.5, signals=["part_at_pick"])
    assert d3.ok and d3.max_offset == pytest.approx(0.3, abs=1e-6)
    # A log on the controller's clock aligns on a reference edge.
    late = perfect.shifted(100.0)
    d4 = tl.diff(late, align_on="part_at_pick")
    assert d4.ok and d4.shift == pytest.approx(-100.0)
    with pytest.raises(ValueError, match="rising edge"):
        tl.diff(late, align_on="spec_ok")  # constant, never rises
    # Signals only one side carries are listed, not judged.
    partial = bt.trace.load({"conv": perfect.signals["conv"], "unknown": [(0.0, False), (1.0, True)]})
    d5 = tl.diff(partial)
    assert d5.ok and set(d5.only_in_bake) >= {"part_at_pick", "vacuum"} and d5.only_in_trace == ["unknown"]
    with pytest.raises(ValueError, match="not on both sides"):
        tl.diff(partial, signals=["vacuum"])
    # CSV column aliases and value spellings.
    loose = bt.trace.load("time,tag,state\n0,x,off\n1.5,x,on\n2,x,LOW\n")
    assert loose.edges("x") == ([1.5], [2.0])
    with pytest.raises(ValueError, match="needs `t`, `name` and `value`"):
        bt.trace.load("a,b\n1,2\n")
