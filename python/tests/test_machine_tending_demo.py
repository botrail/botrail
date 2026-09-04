"""`examples/machining/machine_tending_demo.py` — a cobot tending a
machining centre by hand, asserted the way a cell owner would (T0 of
design/design-machine-tending.md).

The cell is built from catalog products (a MELFA ASSISTA, a 2F-85, the
MPH-3 hand, a robot stand), so these tests need the catalog — cached
locally or fetched once — and skip where it is unreachable, like the
machining demo's. What they pin:

* the cycle's order on the chart: the door slid open before the arm goes
  in, the buttons read one press each and never their neighbours, a start
  pressed only with the door shut — and the parts where the cycle leaves
  them;
* the hand: three tools on one wrist — the fork carries the leaf, the pin
  presses with the fingers never shut, the gripper stays the TCP — and
  the bracket lands on the bill as the catalog product it is;
* the wiring: the buttons and the door's switches are the machine's
  inputs, its `running` the robot's; nothing the machine drives.
"""

import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "machining"))

import machine_tending_demo as demo

PRESSED = ("unclamp", "clamp", "cycle_start")


def _bake_or_skip(**kwargs):
    try:
        return demo.bake(**kwargs)
    except Exception as err:
        if "catalog" in str(err).lower() or "fetch" in str(err).lower() or "resolve" in str(err).lower():
            pytest.skip(f"catalog unavailable: {err}")
        raise


@pytest.fixture(scope="module")
def cell():
    return _bake_or_skip()


def test_the_cycle_runs_in_order_and_the_parts_change_places(cell) -> None:
    scene, hs, tl = cell
    assert hs.template == "manual" and hs.door is None and tl.sequences == ["tend", "vmc"]
    presses = {b: tl.signal(f"vmc/panel/{b}").high_spans() for b in demo.BUTTONS}
    # One press each, inside its own press/hold step; the E-stop never.
    for button in PRESSED:
        (t0, t1), = presses[button]
        assert tl.step_span(f"tend/press_{button}").start <= t0 < t1 <= tl.step_span(f"tend/back_{button}").end
    assert presses["estop"] == []
    order = [presses[b][0][0] for b in PRESSED]
    assert order == sorted(order)
    # The door: shut at first, open before the arm goes in, shut again
    # before the start — and the machine runs on that start.
    closed, opened = tl.signal(hs.signal("door_closed")), tl.signal(hs.signal("door_open"))
    assert closed.value_at(0.0) and not opened.value_at(0.0)
    assert opened.rising_edges()[0] <= tl.step_span("tend/enter").start
    assert closed.rising_edges()[-1] <= order[2] and closed.value_at(order[2])
    assert tl.signal(hs.signal("running")).value_at(tl.duration)
    # The finished part went to the stocker's out slot, the blank into the vise.
    (bx, by, _bz), _ = scene.frame("stocker/out")
    p, _ = tl.object_pose("finished", tl.duration)
    assert (p[0], p[1]) == pytest.approx((bx, by), abs=1e-3)
    (jx, jy, _), _ = scene.frame("vise/jaw")
    p, _ = tl.object_pose("blank", tl.duration)
    assert (p[0], p[1]) == pytest.approx((jx, jy), abs=1e-3)
    assert float(tl.min_clearance()) > 0.0015
    assert tl.duration < 180.0


def test_the_hand_is_three_tools_on_one_wrist(cell) -> None:
    scene, _hs, tl = cell
    robot = scene.robot_of("arm")
    assert robot.tcp_link == "tcp" and {demo.PIN_TIP, demo.FORK_TIP, demo.FORK} <= set(robot.link_names)
    # The bracket is a catalog product: its row on the bill carries the id.
    by = {row["names"][0]: row for row in scene.bom().rows}
    assert by["arm/tool"]["category"] == "tool.multi" and by["arm/tool"]["catalog"].startswith("botrail/hand/mph3/")
    # The pin presses with the fingers open — no grasp doubles as a push.
    for button in PRESSED:
        (t0, _t1), = tl.signal(f"vmc/panel/{button}").high_spans()
        assert tl.sample(t0, robot="arm")[-1] == pytest.approx(demo.OPEN, abs=1e-6)
    # The leaf rides the fork: out by the stroke while the arm works
    # inside, back where it began at the end.
    p0, _ = tl.object_pose("vmc/side_door/leaf", 0.0)
    mid, _ = tl.object_pose("vmc/side_door/leaf", tl.step_span("tend/approach").start)
    stroke = by["vmc/side_door"]["attributes"]["stroke_mm"] / 1e3
    assert mid[1] - p0[1] == pytest.approx(stroke, abs=1e-3)
    p1, _ = tl.object_pose("vmc/side_door/leaf", tl.duration)
    assert p1 == pytest.approx(p0, abs=1e-3)


def test_the_buttons_are_the_machines_inputs(cell) -> None:
    scene, hs, _ = cell
    hosts = {(p.name, p.direction, p.host) for p in scene.io_points()}
    # The machine's program lives on the CNC: the buttons and the door's
    # switches are its inputs; its `running` is what the arm's program
    # waits on, a DO on one and a DI on the other.
    for button in PRESSED:
        assert (f"vmc/panel/{button}", "input", hs.node) in hosts
    assert (hs.signal("door_closed"), "input", hs.node) in hosts
    assert (hs.signal("running"), "output", hs.node) in hosts
    assert (hs.signal("running"), "input", "<arm>") in hosts
    by = {row["names"][0]: row for row in scene.bom().rows}
    assert by["vmc"]["model"] == "α-D21MiB5 Plus"
    assert by["vmc/side_door"]["attributes"]["drive"] == "manual"
    assert by["stand"]["catalog"].startswith("sus/zf/robostand-crx")


def test_the_cell_can_be_ordered_from_the_catalog() -> None:
    """The same cell with the machine and the vise as catalog products —
    skipped until the packs are published (a fetch that cannot resolve
    the id is the catalog's business, not this cell's)."""
    try:
        scene, hs, tl = demo.bake(catalog=True)
    except Exception as err:  # an unresolvable pack skips
        if any(word in str(err).lower() for word in ("catalog", "fetch", "resolve", "not found", "404")):
            pytest.skip(f"catalog packs unavailable: {err}")
        raise
    by = {row["names"][0]: row for row in scene.bom().rows}
    assert by["vmc"]["catalog"].startswith(demo.MACHINE_CATALOG)
    assert by["vise"]["catalog"].startswith(demo.VISE_CATALOG)
    assert hs.template == "manual" and tl.signal(hs.signal("running")).value_at(tl.duration)


def test_the_document_set_hands_the_machine_over(cell, tmp_path: Path) -> None:
    scene, _hs, tl = cell
    report, runs = demo.deliver(scene, tl, tmp_path)
    names = {p.name for p in tmp_path.iterdir()}
    assert {"machine_tending.plcopen.xml", "machine_tending_interlocks.md", "machine_tending_interlocks.csv",
            "machine_tending_handshake.md", "machine_tending_layout.svg", "machine_tending_report.md"} <= names
    assert {d["path"].rsplit("/", 1)[-1] for d in report.deliverables} >= {"machine_tending_interlocks.md"}
    # The machine's program is a POU on the CNC's own resource, the arm's
    # on its controller.
    xml = (tmp_path / "machine_tending.plcopen.xml").read_text()
    assert '<pou name="vmc" pouType="program">' in xml and '<pou name="tend" pouType="program">' in xml
    assert '<resource name="vmc_cnc">' in xml and '<resource name="arm">' in xml
    # The interlock table: the start takes only with both doors confirmed
    # shut and no E-stop; every input is a switch on the CNC.
    rows = {(r["program"], r["step"], r["target"]): r for r in scene.interlocks().rows}
    start = rows[("vmc", "cycle_start", "vmc/running")]
    assert start["condition"] == ("(RISING(vmc/panel/cycle_start) AND vmc/side_door/closed AND "
                                  "vmc/front_door/closed AND NOT vmc/panel/estop)")
    assert {i["name"]: i["kind"] for i in start["inputs"]} == {
        "vmc/panel/cycle_start": "sensor", "vmc/side_door/closed": "sensor",
        "vmc/front_door/closed": "sensor", "vmc/panel/estop": "sensor"}
    assert start["host"] == "vmc/cnc" and start["after"] == ["wait_start"]
    # The arm's first move waits on `running` dropping — and names the
    # machine's steps that write it: the handshake, read across the table.
    first = rows[("tend", "to_unclamp", "to_unclamp")]
    assert first["condition"] == "NOT vmc/running" and first["host"] == "<arm>"
    assert first["inputs"][0]["written_by"] == ["vmc/machining", "vmc/done", "vmc/cycle_start"]
    # The report: the machine, its loose leaf and switches, the buttons,
    # the CNC hosting its program — and the FAT rows' verdicts.
    (m,) = report.machines
    assert (m["name"], m["model"], m["controller"], m["programs"]) == ("vmc", "α-D21MiB5 Plus", "vmc/cnc", ["vmc"])
    assert (m["door"]["drive"], m["door"]["driven"], m["door"]["stroke_mm"]) == ("manual", False, 760.0)
    assert m["door"]["lanes"] == ["vmc/side_door/closed", "vmc/side_door/open"]
    assert [b.rsplit("/", 1)[-1] for b in m["buttons"]] == list(demo.BUTTONS)
    matrix = {s["name"]: s["ok"] for s in report.scenarios}
    assert matrix == {"baseline": True, "door_switch_stuck": False, "clamp_button_open": False, "estop_pressed": False}
    for name in ("door_switch_stuck", "estop_pressed"):
        assert "wait_start" in runs.errors[name]
    assert "wait_clamp" in runs.errors["clamp_button_open"]
    assert "## Machines" in report.to_markdown() and "vmc/side_door (loose leaf)" in report.to_markdown()
