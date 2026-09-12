"""`examples/assembly/cover_bolting_demo.py` — a cobot fits a gearbox cover
and screws it down, asserted the way a cell owner would (A0 of
design/design-assembly.md).

The cell is built from catalog products (a UR5e, OnRobot RG6 and
a robot stand), so these tests need the catalog — cached
locally or fetched once — and skip where it is unreachable, like the
tending demo's. What they pin:

* the cycle's order: the cover seated before any screw, the screws in
  the joint's star order, each picked from the presenter, driven with
  the driver's OK and left in its hole;
* the fastening report: every screw engaged on its hole's axis, run down
  its length in the thread's time, the engagement and the torque inside
  the joint's figures — and a screw the joint cannot use refused at build;
* the FAT rows: a NOK retried and passed, a second NOK halting the cell
  with the screw released, the E-stop admitting no run;
* the bill and the wiring: six screws as one line, the driver's handshake
  as wires between two controllers.
"""

import sys
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "assembly"))

import cover_bolting_demo as demo

A = bt.assembly


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


def test_the_cover_is_fitted_then_the_screws_go_in_star_order(cell) -> None:
    _scene, tl, joint, driver, feeder, placement, fastening = cell
    assert tl.sequences == ["assemble", driver.program]
    assert joint.order[:2] in (["h4", "h5"], ["h5", "h4"]) and [h for _s, h in fastening.pairs] == joint.order
    release = tl.step_span(f"assemble/{placement.steps['release']}")
    firsts = [tl.step_span(f"assemble/{names['feed']}").start for names in fastening.steps]
    assert release.end <= firsts[0] and firsts == sorted(firsts)
    # The cover landed on its seat.
    fit = A.fit_report(tl, placement)
    assert fit["checks"] == {"position": "pass", "tilt": "pass", "seated": "pass"}
    assert abs(fit["offset_mm"]) < 0.5 and abs(fit["height_mm"]) < 0.5
    # Every screw: picked at the presenter (its presence lane drops when
    # the bit lifts it), driven on the driver's OK, seated in its hole.
    present = tl.signal(feeder.present).high_spans()
    assert len(present) == len(fastening.pairs)
    oks = tl.signal(driver.signal("ok")).rising_edges()
    assert len(oks) == len(fastening.pairs) and tl.signal(driver.signal("nok")).rising_edges() == []
    for (screw, hole), names, (t_on, t_off) in zip(fastening.pairs, fastening.steps, present):
        take = tl.step_span(f"assemble/{names['take']}")
        assert t_on <= take.start <= t_off
        (fx, fy, fz), _ = joint.hole_pose(hole)
        (x, y, z), _ = tl.object_pose(screw, tl.duration)
        assert (x, y) == pytest.approx((fx, fy), abs=1e-3)
        assert z == pytest.approx(fz - joint.fastener.length_m, abs=1e-3)
    # A screw in its hole and the cover on its housing are meant contacts,
    # not clearance: what the cycle comes closest to is something else.
    clearance = tl.min_clearance()
    assert float(clearance) > 0.0005, clearance
    assert tl.duration < 200.0


def test_the_fastening_report_reads_the_joints_figures_off_the_bake(cell) -> None:
    _scene, tl, joint, driver, _feeder, _placement, fastening = cell
    rows = A.fastening_report(tl, fastening)
    assert [r["hole"] for r in rows] == joint.order and [r["order"] for r in rows] == list(range(1, 7))
    for r in rows:
        assert r["result"] == "ok" and r["attempts"] == 1
        assert r["offset_mm"] < 0.2 and r["tilt_deg"] < 0.5
        assert r["seated_mm"] == pytest.approx(20.0, abs=1.0)
        assert r["drive_s"] == pytest.approx(driver.t_cycle_s - driver.t_find_s, abs=0.05)
        assert (r["engagement_mm"], r["tip_mm"], r["torque_nm"]) == (10.0, 10.0, 4.5)
        assert r["checks"] == {"align": "pass", "tilt": "pass", "seated": "pass", "engagement": "pass",
                               "depth": "pass", "torque": "pass"}
    # 17 mm of M5×0.8 at 340 rpm: the engage move put the tip 3 mm in.
    assert driver.t_run_s == pytest.approx(0.017 / (0.0008 * 340 / 60))
    assert A.check(joint, driver) == []
    with pytest.raises(ValueError, match="engages 6.0 mm"):
        demo.build(length_mm=16)


def test_rg6_grasps_the_boss_sides_below_its_top(cell):
    """A successful attach must not hide a grip that only grazes the top."""
    scene, tl, joint, _driver, _feeder, placement, _fastening = cell
    t = tl.step_span(f"assemble/{placement.steps['hold']}").end
    q = tl.sample(t, robot="arm")
    cover_z = tl.object_pose(joint.b, t)[0][2]
    boss_top = cover_z + demo.COVER[2] + demo.BOSS[1]
    tips = [scene.link_pose_at(name, q, robot="arm")[0]
            for name in demo.tooling.pads(scene.robot_of("arm")) if name.endswith("finger_tip")]
    assert len(tips) == 2
    for p in tips:
        # Contact band in the upper half of the 30 mm boss, with at least
        # 5 mm below its top; not the first grazing contact of closing jaws.
        assert 0.005 <= boss_top - p[2] <= demo.BOSS[1] / 2


def test_the_fat_rows_refuse_what_they_should(cell) -> None:
    scene, tl, _joint, driver, _feeder, _placement, fastening = cell
    runs = scene.simulate_scenarios(["assemble", driver.program], max_duration=tl.duration + 30.0)
    assert runs.errors.get("baseline") is None and runs.errors.get("nok_once") is None
    once = runs["nok_once"]
    rows = A.fastening_report(once, fastening)
    assert rows[0]["result"] == "retry" and rows[0]["attempts"] == 2 and all(r["result"] == "ok" for r in rows[1:])
    assert once.duration > tl.duration
    # Two NOKs: the program halts at the first screw's halt step and the
    # cycle is refused (a refused run has no timeline to read; the step it
    # stalls in is the verdict).
    assert "assemble/halt0" in runs.errors["nok_twice"] and "nok_twice" not in runs.names
    # The E-stop in: the arm never raises a start — it waits at the engage
    # move, whose completion the lane guards.
    assert "engage0" in runs.errors["estop_pressed"] and "forced: panel/estop=true" in runs.errors["estop_pressed"]
    # The presenter empty: no screw comes, the arm waits for one.
    assert "feed0" in runs.errors["feeder_empty"]
    # The START wire open: the arm raises it, the driver never hears it.
    assert "run0" in runs.errors["start_wire_open"] and "driver/idle0" in runs.errors["start_wire_open"]
    assert set(runs.errors) == {"nok_twice", "estop_pressed", "feeder_empty", "start_wire_open"}


def test_a_screw_taught_off_its_hole_is_refused_by_name() -> None:
    """The engage move is collision-checked: 2 mm off the hole's axis the
    screw meets the cover outside its window, and the bake says which."""
    try:
        scene, _joint, driver, *_rest = demo.build(misalign_mm=2.0)
    except Exception as err:
        if "catalog" in str(err).lower() or "fetch" in str(err).lower() or "resolve" in str(err).lower():
            pytest.skip(f"catalog unavailable: {err}")
        raise
    with pytest.raises(ValueError) as refused:
        scene.simulate_sequences(["assemble", driver.program], max_duration=240.0)
    message = str(refused.value)
    assert "set/cover" in message and "screw5" in message, message
    # The spin effect is bound to the driver's run lane.
    import json

    flashes = json.loads(scene._project_json())["flashes"]
    assert any(f["kind"] == "spin" and f["spin_link"] == demo.BIT for f in flashes)


def test_the_bill_and_the_wiring(cell) -> None:
    scene, _tl, _joint, driver, feeder, _placement, fastening = cell
    by = {row["names"][0]: row for row in scene.bom().rows}
    screws = by[fastening.pairs[0][0]]
    assert screws["qty"] == 6 and screws["category"] == "fastener" and screws["model"] == "ISO 4762 M5x20-8.8"
    assert by["feeder"]["category"] == "feeder.screw"
    drivers = [row for row in scene.bom().rows if row["category"] == "tool.screwdriver"]
    assert len(drivers) == 1 and drivers[0]["attributes"]["torque_max_nm"] == 5
    assert by["arm/tool"]["manufacturer"] == "OnRobot"
    assert by["arm/tool"]["attributes"]["part_number"] == "109878"
    assert drivers[0]["attributes"]["part_number"] == "103961"
    assert by["arm/tool/tool3"]["catalog"].startswith(demo.tooling.GRIPPER_CATALOG)
    assert drivers[0]["attributes"]["required_extender"].startswith("109301:")
    assert drivers[0]["attributes"]["required_bit_kit"].startswith("105121:")
    assert by[driver.node]["attributes"]["part_number"] == "113761"
    assert "Compute Box" in by["arm/tool"]["attributes"]["electrical_route"]
    assert "not robot wrist power" in by["arm/tool"]["attributes"]["electrical_route"]
    # The screws carry their joint's torque, and the driver's requirement
    # is derived from it: the joint's 5 N·m top against the 5 delivered.
    req = scene.requirements()
    row = next(r for r in req.rows if r.category == "tool.screwdriver")
    wants = {r.key: (r.value, r.status) for r in row.requirements}
    assert wants["torque_nm"] == (5.0, "ok") and wants["screw_length_mm"] == (20.0, "ok")
    assert wants["thread_max_mm"] == (5.0, "ok") and wants["thread_min_mm"][1] == "ok"
    assert by["arm"]["catalog"].startswith("universal_robots/ur/ur5e") and by["stand"]["catalog"].startswith("sus/zf/robostand-crx")
    assert (by["bench"]["manufacturer"], by["bench"]["model"]) == ("TRUSCO", "AE-1500")
    assert by["panel"]["model"] == "XALK178F"
    bench_lo, bench_hi = scene.obstacle_bounds("bench/top")
    assert bench_lo[0] > scene.obstacle_bounds("stand/top")[1][0]
    feeder_lo, feeder_hi = scene.obstacle_bounds("feeder/body")
    assert all(bench_lo[i] <= feeder_lo[i] < feeder_hi[i] <= bench_hi[i] for i in (0, 1))
    # The arm's control box is ordered from its pack and hosts the arm's
    # program; the handshake runs between it and the driver's controller,
    # every wire with an address on its node.
    assert by["controller"]["catalog"].startswith("universal_robots/control-box/e-series")
    hosts = {(p.name, p.direction, p.host) for p in scene.io_points()}
    assert (driver.signal("start"), "output", "controller") in hosts and (driver.signal("start"), "input", driver.node) in hosts
    assert (driver.signal("ok"), "output", driver.node) in hosts and (driver.signal("ok"), "input", "controller") in hosts
    assert (feeder.present, "input", "controller") in hosts
    statuses = [p.status for p in scene.io_points()]
    assert "unbound" not in statuses and statuses.count("bound") == 14
    # The gripper stays the TCP; the bit and the shank are addressable.
    robot = scene.robot_of("arm")
    assert robot.tcp_link.startswith("gripper_rg6_") and {demo.BIT, demo.TIP} <= set(robot.link_names)
    assert demo.SHANK in robot.joint_names


def test_commercial_tooling_uses_side_support_and_independent_feed_axis():
    """The maker's side mount must not regress to a spindle on a long boom.

    Check manufacturer drawing datums independently of the cell's IK and
    workpiece: 120° mounting normals, 81 mm driver axis offset, +50 mm
    extender and 55 mm feed along the screw axis.
    """
    import math

    tooling = demo.tooling
    driver = tooling.screwdriver(stroke=0.055)
    probe = bt.Scene(driver)
    p0, q0 = probe.link_pose("tip")
    assert p0 == pytest.approx((0.153 - 0.017 + 0.050, 0, 0.081))
    probe.set_joint_positions([0.055])
    assert probe.link_pose("tip")[0] == pytest.approx((p0[0] + 0.055, 0, 0.081))
    # Tip +Z points back towards the body; bit +Z is its spin/feed axis.
    def z_axis(q):
        x, y, z, w = q
        return (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
    assert z_axis(q0) == pytest.approx((-1, 0, 0), abs=1e-9)
    assert z_axis(probe.link_pose("bit")[1]) == pytest.approx((1, 0, 0), abs=1e-9)
    changer = tooling.dual_changer()
    stack = changer.attach_tool(driver, flange="hand_driver", prefix="drv_")
    probe = bt.Scene(stack)
    a = z_axis(probe.link_pose("hand_driver")[1])
    b = z_axis(probe.link_pose("hand_gripper")[1])
    assert sum(x * y for x, y in zip(a, b)) == pytest.approx(math.cos(math.radians(120)))
    x, y, _ = probe.link_pose("drv_tip")[0]
    assert math.hypot(x, y) == pytest.approx(0.208898, abs=1e-6)
    assert not any("boom" in name for name in stack.link_names)


def test_the_hand_over_set_carries_the_joint(cell, tmp_path) -> None:
    """The interlock table shows the two guards a reviewer looks for, the
    report carries an assembly section, and the tightening sheet lists
    every screw with its baked result."""
    import json

    scene, tl, joint, _driver, feeder, placement, fastening = cell
    table = scene.interlocks(["assemble"]).rows
    # A row's condition is what admits the step: the first pick waits for
    # the cover seated and a screw present, the later picks only for a screw.
    pick = next(r for r in table if r["step"] == fastening.steps[0]["to_pick"])
    assert placement.seated == "assemble/cover_seated" and placement.seated in pick["condition"]
    assert feeder.present in pick["condition"]
    assert placement.seated not in next(r for r in table if r["step"] == fastening.steps[1]["to_pick"])["condition"]
    start = next(r for r in table if r["step"] == fastening.steps[0]["start"])
    assert start["output"] == "driver/start := TRUE" and "NOT panel/estop" in start["condition"]
    rows = A.fastening_report(tl, fastening)
    fit = A.fit_report(tl, placement)
    section = A.report_section(fastening, rows, fit)
    assert section["title"] == "Assembly" and "order h5, h4, h0, h2, h1, h3" in section["markdown"]
    assert section["json"]["joint"]["holes"]["h0"]["engagement_mm"] == 10.0 and len(section["json"]["screws"]) == 6
    report = scene.cell_report({"baseline": tl}, title="cover bolting", sections=[section])
    assert report.io["unbound"] == 0 and report.io["bound"] == 14
    md = report.to_markdown()
    assert "## Assembly" in md and "6/6 screws driven" in md and md.count("all pass") == 6
    data = json.loads(report.to_json())
    assert data["sections"][0]["json"]["driver"]["torque_nm"] == 4.5
    assert report.sections[0]["json"]["joint"]["order"] == joint.order
    sheet = tmp_path / "sheet.md"
    A.export_sheet(sheet, fastening, rows=rows)
    text = sheet.read_text()
    assert "Baked result: 6/6 screws driven." in text and "| 1 | h5 |" in text and "| run0 |" in text
    A.export_sheet(tmp_path / "sheet.csv", fastening, rows=rows)
    assert (tmp_path / "sheet.csv").read_text().splitlines()[0].endswith(",drive_s,attempts,result")


def test_the_cell_can_be_ordered_from_the_catalog() -> None:
    """The same cell with the presenter, the screws and
    the workpiece set as catalog products, the joint read from the set's
    `mounting` — skipped until the packs are published (or point
    `BOTRAIL_CATALOG_ROOT` at a local `bcb build` directory)."""
    import os

    root = os.environ.get("BOTRAIL_CATALOG_ROOT")
    try:
        scene, tl, joint, _driver, _feeder, _placement, fastening = demo.bake(catalog=True, catalog_root=root)
    except Exception as err:
        text = str(err).lower()
        if "not in the catalog" in text or "catalog" in text or "fetch" in text or "no manifest" in text:
            pytest.skip(f"assembly packs unavailable: {err}")
        raise
    assert joint.catalog is not None and joint.catalog[0] == demo.WORKPIECE_CATALOG
    assert joint.fastener.catalog[0] == demo.SCREW_CATALOG and joint.torque_nm == (4.0, 5.0)
    by = {row["names"][0]: row for row in scene.bom().rows}
    assert by["feeder"]["catalog"].startswith(demo.FEEDER_CATALOG)
    assert by[fastening.pairs[0][0]]["qty"] == 6 and by[fastening.pairs[0][0]]["catalog"].startswith(demo.SCREW_CATALOG)
    assert by["set/housing"]["catalog"].startswith(demo.WORKPIECE_CATALOG) and by["set/dowel/p0"]["qty"] == 2
    drivers = [row for row in scene.bom().rows if row["category"] == "tool.screwdriver"]
    assert len(drivers) == 1 and drivers[0]["model"] == demo.DRIVER_MODEL
    assert not drivers[0].get("catalog")  # runtime product reference in both modes
    rows = A.fastening_report(tl, fastening)
    assert all(r["result"] == "ok" and r["checks"]["engagement"] == "pass" for r in rows)
    req = scene.requirements()
    row = next(r for r in req.rows if r.category == "tool.screwdriver")
    assert {r.key: r.status for r in row.requirements}["torque_nm"] == "ok"
