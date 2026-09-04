"""`scene.requirements()` / `scene.check()` — what the cell asks of every BOM
line, compared with what the chosen part says. botrail derives and compares;
it never chooses: a part that falls short is an error, a part that does not
say is a warning, a line nobody has identified carries its requirements
into the `unidentified_part` note.
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "drone"))

HF_CACHE = Path(os.environ.get("HF_HOME") or Path.home() / ".cache" / "huggingface") / "hub"
HAS_CATALOG = any(HF_CACHE.glob("datasets--botrail--botrail-catalog*"))

PICK = [0.95, 0.85, -1.1, 0.25, 0.0, 0.0]


def cell(*, reach_mm: float = 1000.0, part_mass: float | None = 0.8) -> bt.Scene:
    """A hand-identified pick cell: every number the derivations read is
    authored here, so the expected requirements follow by hand."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.set_part("simple_arm", manufacturer="ACME", model="SA-6", payload_kg=3.0, reach_mm=reach_mm, mass_kg=28)
    stand = bt.parts.pedestal(scene, "stand", height=0.4, position=(0, 0))
    scene.set_robot_base_pose(*scene.frame(stand.frames[0]))
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(0.25, 0.35, 0.43))
    if part_mass is not None:
        scene.set_part("part", category="workpiece", mass_kg=part_mass)
    scene.add_conveyor("belt", zone_position=(-0.1, 0.35, 0.45), zone_size=(0.9, 0.2, 0.14), velocity=(0.12, 0, 0))
    scene.set_part("belt", manufacturer="MISUMI", model="GVL-900-200", length_mm=900, width_mm=200, max_speed_mps=0.2, load_kg=10)
    scene.add_beam_sensor("eye", frm=(0.25, 0.25, 0.43), to=(0.25, 0.45, 0.43))
    scene.add_zone_sensor("area", position=(0, 0, 0.9), size=(1.0, 0.6, 0.4))
    scene.set_part("area", model="OS32C", range_mm=1000)
    scene.add_io_node("UR", kind="robot_controller", robots=["simple_arm"], channels=bt.io.ur_standard())
    scene.set_part("UR", manufacturer="Universal Robots", model="CB3")
    scene.add_segment("to_pick", goal=PICK)
    scene.add_segment("home", goal=[0.0] * 6)
    seq = scene.sequence("pick")
    seq.step("feed", actions=[bt.seq.start("belt"), bt.seq.motion("to_pick")], transition=bt.seq.done())
    seq.step("await", transition=bt.seq.rising("eye"))
    seq.step("grip", actions=[bt.seq.attach("part")], transition=bt.seq.immediately())
    seq.step("home", actions=[bt.seq.motion("home")], transition=bt.seq.done())
    scene.auto_assign_io()  # the I/O lint is clean: what remains is the requirement comparison
    return scene


def by_key(row) -> dict[str, bt.select.Requirement]:
    return {r.key: r for r in row.requirements}


def test_requirements_follow_from_the_cell() -> None:
    scene = cell()
    req = scene.requirements()
    assert [r.target for r in req] == [r["names"][0] for r in scene.bom().rows]

    robot = by_key(req["simple_arm"])
    # Payload: no tool, the heaviest grasped part.
    assert robot["payload_kg"].value == pytest.approx(0.8) and "grasps part" in robot["payload_kg"].basis
    assert robot["payload_kg"].status == "ok" and robot["payload_kg"].provided == 3.0
    # Reach: the farthest taught goal measured at the TCP (a plain URDF has no
    # flange), from the base, plus the 10 % margin.
    base = scene.robot_base_pose[0]
    tcp = scene.robot.tcp_link
    farthest = max(
        math.dist(scene.link_pose_at(tcp, q)[0], base) for q in (PICK, [0.0] * 6)
    )
    assert robot["reach_mm"].value == pytest.approx(farthest * 1000 * 1.1, rel=1e-6)
    assert "(TCP)" in robot["reach_mm"].basis and robot["reach_mm"].status == "ok"

    belt = by_key(req["belt"])
    assert belt["length_mm"].value == 900 and belt["width_mm"].value == 200
    assert belt["speed_mps"].value == pytest.approx(0.12) and belt["speed_mps"].provided == 0.2
    # The part starts inside the transport zone: it is the belt's load.
    assert belt["load_kg"].value == pytest.approx(0.8) and belt["load_kg"].status == "ok"
    assert req["belt"].status == "ok"

    # A beam's span, an area sensor's half-diagonal.
    assert by_key(req["eye"])["sensing_range_mm"].value == pytest.approx(200)
    assert req["eye"].status == "unidentified" and req["eye"] in req.unidentified()
    area = by_key(req["area"])["range_mm"]
    assert area.value == pytest.approx(0.5 * math.hypot(1.0, 0.6) * 1000, abs=0.05) and area.status == "ok"

    # I/O nodes count the points assigned to them; the I/O report owns their
    # capacity findings, so they raise none here.
    node = by_key(req["UR"])
    assert node["di"].value == 2 and node["do"].value == 1  # eye + area / belt run
    # ... and its "provided" is what it declares: 8 DI / 8 DO here.
    assert node["di"].provided == 8 and node["di"].status == "ok" and req["UR"].status == "ok"
    assert not [f for f in req.findings() if f.target == "UR"]

    # The pedestal carries the robot standing on it.
    stand = by_key(req["stand"])["load_kg"]
    assert stand.value == pytest.approx(28) and "simple_arm standing" in stand.basis
    assert req["stand"].status == "unidentified"

    # Lines the cell asks nothing of.
    assert req["part"].requirements == [] and req["part"].status == "unidentified"
    assert req["simple_arm"].minimum == {"payload_kg": 0.8, "reach_mm": robot["reach_mm"].value}
    assert "stand" in req and "nope" not in req
    with pytest.raises(KeyError):
        req["nope"]


def test_short_unknown_and_incomplete_are_named() -> None:
    scene = cell(reach_mm=300.0, part_mass=None)
    scene.set_part("belt", manufacturer="MISUMI", model="GVL-900-200", length_mm=900)  # no width / speed / load
    req = scene.requirements()
    codes = [(f.code, f.target) for f in req.findings()]
    assert ("spec_short", "simple_arm") in codes
    assert ("spec_unknown", "belt") in codes
    # No mass on the grasped part: the payload is not guessed, the note says why.
    assert ("requirement_incomplete", "simple_arm") in codes
    assert "payload_kg" not in by_key(req["simple_arm"])
    assert any("no mass for part" in n for n in req["simple_arm"].notes)
    short = [f for f in req.findings() if f.code == "spec_short"]
    assert "reach_mm 300 < required" in short[0].message
    assert req.short() == [req["simple_arm"]] and not req.ok
    unknown = {r.key for r in req["belt"].requirements if r.status == "unknown"}
    assert unknown == {"width_mm", "speed_mps"}  # no part on the belt -> no load requirement
    assert req["belt"].status == "unknown"


def test_check_aggregates_every_static_check() -> None:
    scene = cell(reach_mm=300.0)
    report = scene.check()
    codes = [f.code for f in report.findings]
    assert not report.ok and "spec_short" in codes
    note = next(f for f in report.findings if f.code == "unidentified_part" and f.target == "eye")
    assert "needs sensing_range_mm >= 200" in note.message
    d = json.loads(report.to_json())
    assert d["ok"] is False and d["requirements"]["lines"] == len(scene.bom())
    assert d["requirements"]["short"] == 1
    assert report.to_markdown().startswith("FAIL")
    assert len(report.errors()) == 1 and report.errors()[0].target == "simple_arm"
    # Fix the robot: the check passes (unknowns and notes are not failures).
    scene.set_part("simple_arm", reach_mm=2000)
    assert scene.check().ok


def test_formats_and_sequence_filter(tmp_path: Path) -> None:
    scene = cell()
    req = scene.requirements()
    md = req.to_markdown()
    assert "| simple_arm | robot | payload_kg >= 0.8 |" in md and "Notes:" not in md
    rows = json.loads(req.to_json())["rows"]
    assert rows[0]["requirements"][0] == {
        "key": "payload_kg", "op": ">=", "value": 0.8, "basis": "grasps part 0.8 kg",
        "provided": 3.0, "provided_key": "payload_kg", "status": "ok",
    }
    csv_text = req.to_csv()
    assert csv_text.splitlines()[0].startswith("line,category,qty,identified,requirement")
    req.save(tmp_path / "req.md")
    req.save(tmp_path / "req.json")
    assert (tmp_path / "req.md").read_text() == md
    # A second program that never grasps: counting only it drops the payload.
    other = scene.sequence("idle")
    other.step("wait", transition=bt.seq.elapsed(1.0))
    assert "payload_kg" in by_key(scene.requirements(sequences=["pick"])["simple_arm"])
    assert "payload_kg" not in by_key(scene.requirements(sequences=["idle"])["simple_arm"])
    assert scene.requirements().sequences == ["pick", "idle"]
    with pytest.raises(ValueError):
        scene.requirements(sequences=["nope"])
    # The margin is a parameter, not a constant.
    r0 = by_key(scene.requirements(margin=0.0)["simple_arm"])["reach_mm"].value
    r2 = by_key(scene.requirements(margin=0.2)["simple_arm"])["reach_mm"].value
    assert r2 == pytest.approx(r0 * 1.2, rel=1e-6)
    with pytest.raises(ValueError):
        scene.requirements(margin=-1)


def test_cli_check_carries_requirements(capsys) -> None:
    from botrail._cli import main

    scene = cell()
    path = Path(os.environ.get("TMPDIR", "/tmp")) / "botrail_select_cli.botrail"
    scene.save_project(path)
    assert main(["check", str(path)]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["requirements"] == {"lines": len(scene.bom()), "short": 0, "unknown": 0, "unidentified": 3}
    assert [f["target"] for f in out["findings"] if f["code"] == "unidentified_part"] == ["eye", "stand", "part"]


def test_vehicle_lines_ask_drive_rates_and_deck_load() -> None:
    """A cart's deck load is counted at the parked frame — heading included
    — the body never counts itself, and a part without mass is a note,
    never a guess."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("cart", size=(0.6, 0.4, 0.3), position=(2.0, 0.0, 0.15))
    # The path leaves the start northward, so the parked heading is +90°:
    # the deck, authored at +x in the vehicle frame, sits at +y in the
    # world. Everything below stands there — an unrotated frame would find
    # none of it.
    scene.add_box("cart/deck", size=(0.28, 0.28, 0.02), position=(2.0, 0.4, 0.31))
    scene.add_box("tote", size=(0.2, 0.2, 0.15), position=(2.0, 0.4, 0.4))
    scene.set_part("tote", category="workpiece", mass_kg=2.5)
    scene.add_box("loose", size=(0.05, 0.05, 0.05), position=(2.0, 0.45, 0.36))
    scene.add_vehicle(
        "cart", body=["cart", "cart/deck"],
        path=[(2.0, 0.0), (2.0, 2.0)], stations={"a": 0, "b": 1},
        speed=0.7, start="a",
        tray_position=(0.4, 0.0, 0.35), tray_size=(0.3, 0.3, 0.24),
    )
    req = scene.requirements()
    row = req["cart"]
    assert row.category == "vehicle"  # ground: the aisle stays the author's call
    r = by_key(row)
    assert r["max_speed_mps"].value == pytest.approx(0.7)
    # The tote and `loose` ride (the deck plate is the body, not cargo);
    # only the tote's 2.5 kg counts — `loose` has no mass and is a note.
    assert r["payload_kg"].value == pytest.approx(2.5)
    assert "2 part(s) on the deck at start" in r["payload_kg"].basis
    assert row.notes == ["deck load counts no mass for loose — give them mass_kg on set_part"]
    assert "max_climb_mps" not in r


def test_an_aerial_machine_is_shopped_in_the_uav_aisle() -> None:
    """An aerial vehicle asks its climb and descent rates too, and a line
    nobody identified narrows from the derived `vehicle` category into the
    only aisle that flies."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("bee", size=(0.3, 0.3, 0.1), position=(3.0, 0.0, 0.05))
    scene.add_vehicle(
        "bee", body=["bee"],
        path=[(3.0, 0.0, 0.0), (3.0, 0.0, 1.2), (4.5, 0.0, 1.2)],
        stations={"pad": 0, "hover": 2}, speed=1.1, start="pad",
        drive="aerial", climb_speed=0.5, descent_speed=0.8,
    )
    req = scene.requirements()
    row = req["bee"]
    assert row.category == "vehicle.uav" and not row.identified
    r = by_key(row)
    assert r["max_speed_mps"].value == pytest.approx(1.1)
    assert r["max_climb_mps"].value == pytest.approx(0.5) and r["max_climb_mps"].basis == "climb rate"
    assert r["max_descent_mps"].value == pytest.approx(0.8) and r["max_descent_mps"].basis == "descent rate"
    assert row.minimum == {"max_speed_mps": 1.1, "max_climb_mps": 0.5, "max_descent_mps": 0.8}
    # Identified with a climb rating it cannot honour: short, by name.
    scene.set_part("bee", kind="device", manufacturer="ACME", model="AF-1",
                   max_speed_mps=2.0, max_climb_mps=0.4, max_descent_mps=1.0)
    r = by_key(scene.requirements()["bee"])
    assert r["max_climb_mps"].status == "short"
    assert r["max_speed_mps"].status == "ok" and r["max_descent_mps"].status == "ok"


def test_the_merged_machine_carries_its_vehicle_requirements() -> None:
    """A vehicle whose machine *is* the robot (a UAV: a rigid mount on a
    bodiless vehicle) lands its requirements on the robot's own BOM line —
    where its catalog specs answer them. An AMR stays two lines and the
    arm absorbs nothing."""
    scene = bt.Scene()
    scene.add_robot(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"), name="uav")
    scene.add_vehicle(
        "uav", body=[],
        path=[(0.0, 0.0, 0.0), (0.0, 0.0, 1.0)], stations={"pad": 0, "up": 1},
        speed=0.9, start="pad",
        drive="aerial", climb_speed=0.6, descent_speed=0.7,
    )
    scene.mount_robot("uav", robot="uav")
    req = scene.requirements()
    row = req["uav"]
    assert row.kind == "robot" and row.category == "vehicle.uav"
    r = by_key(row)
    assert r["max_speed_mps"].value == pytest.approx(0.9)
    assert r["max_climb_mps"].value == pytest.approx(0.6)
    assert len(req.rows) == len(scene.bom().rows)  # one machine, one line

    amr = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    amr.add_box("chassis", size=(0.7, 0.5, 0.3), position=(0.0, 0.0, 0.15))
    amr.add_vehicle("mule", body=["chassis"], path=[(0.0, 0.0), (2.0, 0.0)],
                    stations={"a": 0, "b": 1}, speed=0.5, start="a")
    amr.mount_robot("mule", robot="simple_arm", offset_position=(0.0, 0.0, 0.3))
    req = amr.requirements()
    assert "max_speed_mps" not in by_key(req["simple_arm"])
    assert by_key(req["mule"])["max_speed_mps"].value == pytest.approx(0.5)


def test_flight_time_is_a_cycle_fact_and_pad_waits_are_free() -> None:
    """`flight_time_min` needs the baked cycle: airborne seconds are read
    exactly off the vehicle's closed-form track — climbs, hovers and the
    descent count, waiting *on* the pad does not — and the declared hover
    endurance answers, short by name when the survey outgrows it."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("bee", size=(0.3, 0.3, 0.1), position=(3.0, 0.0, 0.05))
    scene.add_vehicle(
        "bee", body=["bee"],
        path=[(3.0, 0.0, 0.0), (3.0, 0.0, 1.0)], stations={"pad": 0, "up": 1},
        speed=1.0, start="pad",
        drive="aerial", climb_speed=0.5, descent_speed=1.0,
    )
    seq = scene.sequence("survey")
    seq.step("t1", actions=[bt.seq.goto("bee", "up")], transition=bt.seq.device_done("bee"))
    seq.step("hover", transition=bt.seq.elapsed(1.5))
    seq.step("l1", actions=[bt.seq.goto("bee", "pad")], transition=bt.seq.device_done("bee"))
    seq.step("recharge", transition=bt.seq.elapsed(2.0))  # parked: free
    seq.step("t2", actions=[bt.seq.goto("bee", "up")], transition=bt.seq.device_done("bee"))
    seq.step("l2", actions=[bt.seq.goto("bee", "pad")], transition=bt.seq.device_done("bee"))
    tl = scene.simulate_sequences(["survey"], max_duration=60.0)
    # Climb 2 s (1 m at 0.5), hover 1.5 s, descend 1 s — twice the trip,
    # once the hover, and never the 2 s on the pad.
    assert tl.vehicle_airborne("bee") == pytest.approx(2.0 + 1.5 + 1.0 + 2.0 + 1.0, abs=0.05)
    with pytest.raises(ValueError):
        tl.vehicle_airborne("nope")

    # Without the timeline the comparison is a note, never a guess.
    row = scene.requirements()["bee"]
    assert "flight_time_min" not in by_key(row)
    assert any("bake the cycle" in n for n in row.notes)
    # With it: airborne x 1.1 / 60, answered by the declared endurance.
    airborne = tl.vehicle_airborne("bee")
    scene.set_part("bee", kind="device", manufacturer="ACME", model="AF-1",
                   max_speed_mps=2.0, max_climb_mps=1.0, max_descent_mps=1.5,
                   flight_time_min=18.0)
    r = by_key(scene.requirements(timeline=tl)["bee"])
    assert r["flight_time_min"].value == pytest.approx(airborne * 1.1 / 60.0, abs=0.01)
    assert "airborne" in r["flight_time_min"].basis and r["flight_time_min"].status == "ok"
    # A machine rated under the survey: the check goes red, by name.
    scene.set_part("bee", kind="device", manufacturer="ACME", model="AF-1",
                   max_speed_mps=2.0, max_climb_mps=1.0, max_descent_mps=1.5,
                   flight_time_min=0.1)
    report = scene.check(timeline=tl)
    assert not report.ok
    assert any(f.code == "spec_short" and "flight_time_min" in f.message for f in report.errors())


X500_DIST = Path.home() / "projects" / "botrail-catalog-builder" / "dist"


@pytest.mark.skipif(not (X500_DIST / "px4" / "x500" / "x500" / "r1").exists(),
                    reason="x500 package not built locally")
def test_aerial_requirements_meet_the_x500_manifest() -> None:
    """The drone demo against the real airframe package: the authored
    indoor rates are asked, the manifest's PX4 limits answer them, and
    `search_for` finds the machine in the `vehicle.uav` aisle."""
    import drone_survey_demo as demo

    scene, tl = demo.bake(pack=X500_DIST / "px4" / "x500" / "x500" / "r1")
    req = scene.requirements(timeline=tl)
    row = req["drone"]
    assert row.kind == "robot" and row.category == "vehicle.uav" and row.identified
    r = by_key(row)
    assert r["max_speed_mps"].status == "ok" and r["max_speed_mps"].provided == 12.0
    assert r["max_climb_mps"].status == "ok" and r["max_climb_mps"].provided == 3.0
    assert r["max_descent_mps"].status == "ok" and r["max_descent_mps"].provided == 1.5
    # The whole survey flies inside the declared 18 min hover endurance.
    assert r["flight_time_min"].provided == 18.0 and r["flight_time_min"].status == "ok"
    assert r["flight_time_min"].value == pytest.approx(tl.vehicle_airborne("drone") * 1.1 / 60.0, abs=0.01)
    assert not [f for f in req.findings() if f.target == "drone"]
    cands = bt.catalog.search_for(row, index=X500_DIST / "index.json")
    assert any(p.id.startswith("px4/x500/x500/") for p in cands)


@pytest.mark.skipif(not HAS_CATALOG, reason="botrail catalog not in the HF cache")
def test_catalog_robot_and_tool_rows() -> None:
    """A catalog cobot and gripper: specs come with the identity, the tool
    line gets the grasp's stroke and payload, reach is measured at the
    flange the catalog declares."""
    robot = bt.Robot.from_catalog("universal_robots/ur5e")
    tool = bt.Robot.from_catalog("robotiq/2f-85")
    scene = bt.Scene(robot.attach_tool(tool), name="ur5e")
    scene.add_box("carton", size=(0.25, 0.18, 0.15), position=(0.6, 0.0, 0.5))
    scene.set_part("carton", category="workpiece", mass_kg=2.3)
    scene.add_segment("to_pick", goal=[0.0, -1.2, 1.4, -1.8, -1.57, 0.0, 0.0])
    seq = scene.sequence("pick")
    seq.step("go", actions=[bt.seq.motion("to_pick")], transition=bt.seq.done())
    seq.step("grip", actions=[bt.seq.attach("carton")], transition=bt.seq.immediately())
    req = scene.requirements()
    arm, grip = by_key(req["ur5e"]), by_key(req["ur5e/tool"])
    assert arm["payload_kg"].value == pytest.approx(0.925 + 2.3) and arm["payload_kg"].status == "ok"
    assert "(flange)" in arm["reach_mm"].basis and arm["reach_mm"].provided == 850
    assert grip["payload_kg"].value == pytest.approx(2.3) and grip["payload_kg"].provided == 5.0
    # 85 mm of stroke cannot open past the carton's smallest side.
    assert grip["stroke_mm"].value == 150 and grip["stroke_mm"].status == "short"
    # Holding force: m·g × SF 2 / (μ 0.5 × 2 surfaces) = 2.3 × 19.62 N.
    assert grip["grip_force_n"].value == pytest.approx(45.1)
    assert req["ur5e/tool"].minimum == {
        "payload_kg": 2.3,
        "stroke_mm": 150.0,
        "grip_force_n": 45.1,
    }
    # spec_short: the 150 mm carton beats the 85 mm stroke. No
    # spec_unknown: the published 2F-85 row carries the grip-force flat
    # mirrors (packed at build time, republished 2026-09-02), so the
    # grip_force_n requirement finds its number.
    assert [f.code for f in req.findings() if f.target == "ur5e/tool"] == [
        "spec_short",
    ]


def test_a_machine_tool_asks_the_arm_for_reach_before_anything_is_taught() -> None:
    """The opening decides the arm: a machine's table, through its side
    door, is a reach requirement on the robot beside it — no taught pose
    needed — and a machine across the hall is not this robot's."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"), base_position=(1.4, -0.35, 0.0))
    bt.parts.machine_tool(scene, "vmc", model="VMC-1", manufacturer="ACME")
    bt.parts.machine_tool(scene, "far", position=(9.0, 0.0), model="VMC-2", manufacturer="ACME")
    line = scene.requirements()["simple_arm"]
    reach = [r for r in line.requirements if r.key == "reach_mm"]
    assert len(reach) == 1 and "the table of `vmc`" in reach[0].basis
    (tx, ty, tz), _ = scene.frame("vmc/table")
    distance = ((tx - 1.4) ** 2 + (ty + 0.35) ** 2 + tz ** 2) ** 0.5
    assert reach[0].value == pytest.approx(distance * 1000.0 * 1.1, abs=0.2)


def test_a_lathe_asks_for_reach_to_its_spindle() -> None:
    # The turning counterpart: the spindle nose, through the front door.
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"), base_position=(-0.55, -1.7, 0.0))
    bt.parts.lathe(scene, "lathe", model="ST-10", manufacturer="Haas")
    line = scene.requirements()["simple_arm"]
    reach = [r for r in line.requirements if r.key == "reach_mm"]
    assert len(reach) == 1 and "the spindle of `lathe`" in reach[0].basis and "front opening" in reach[0].basis
    (sx, sy, sz), _ = scene.frame("lathe/spindle")
    distance = ((sx + 0.55) ** 2 + (sy + 1.7) ** 2 + sz ** 2) ** 0.5
    assert reach[0].value == pytest.approx(distance * 1000.0 * 1.1, abs=0.2)
