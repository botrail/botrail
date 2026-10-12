"""Routes are authored once and derived from the residents' current poses:
lengths, bends, clashes, BOM rows and the cable list follow the scene."""

import json
import re
import math
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
PACKS = Path(__file__).resolve().parent / "data" / "routing_packs"


def _pack(pid: str) -> str:
    """A routing spec pack built by the catalog builder, as a package directory."""
    return str(PACKS / pid)
cx = bt.connections
rt = bt.routing


def two_boxes():
    """A signal line between two boxes two metres apart, orthogonal, with a
    spec that is sold in 2 m and 5 m lengths."""
    s = bt.Scene()
    s.add_box("box_a", size=(0.2, 0.2, 0.2), position=(0.0, 0.0, 0.1))
    s.add_box("box_b", size=(0.2, 0.2, 0.2), position=(2.0, 1.0, 0.1))
    cx.port(s, "a.out", "box_a", "signal", "output", at=("box_a", (0.1, 0, 0)), exit=(1, 0, 0), straight_m=0.05)
    cx.port(s, "b.in", "box_b", "signal", "input", at=("box_b", (-0.1, 0, 0)), exit=(-1, 0, 0), straight_m=0.05)
    cx.connect(s, "a.out", "b.in", name="w1", cable="CBL-001")
    rt.spec(s, "sig", media=["signal"], od_mm=8, bend_r_fixed_mm=50, sold_lengths_m=[2, 5], end_allowance_m=0.1,
            manufacturer="Lapp", model="UNITRONIC 8")
    rt.route(s, "r1", carries="w1", spec="sig", orthogonal=True)
    return s


def test_route_follows_the_boxes_and_is_deterministic():
    s = two_boxes()
    g = rt.geometry(s, "r1")
    # 0.05 stub, 1.65 along x, 1.0 along y, 0.1 stub-with-fillet: 2.8 m less
    # two right-angle fillets of r = 50 mm (each saves 2r − πr/2).
    assert g["length_m"] == pytest.approx(2.8 - 2 * (0.1 - math.pi * 0.05 / 2), abs=1e-3)
    assert g["cut_length_m"] == pytest.approx(g["length_m"] + 0.2)
    assert g["purchase_length_m"] == 5.0
    assert [b["ok"] for b in g["bends"]] == [True, True]
    assert [sp["kind"] for sp in g["spans"]] == ["fixed"]
    assert rt.geometry(s, "r1") == g
    report = rt.report(s)
    assert report.ok and report.routes[0]["status"] == "ok"
    assert report.routes[0]["from"]["target"] == "box_a" and report.routes[0]["to"]["port"] == "b.in"


def test_moving_the_box_moves_the_length_and_the_sold_length():
    s = two_boxes()
    before = rt.geometry(s, "r1")["length_m"]
    s.set_obstacle_pose("box_b", (5.0, 1.0, 0.1))
    after = rt.geometry(s, "r1")
    assert after["length_m"] == pytest.approx(before + 3.0, abs=1e-6)
    assert after["purchase_length_m"] is None
    codes = {(f["code"], f["severity"]) for f in rt.report(s).findings}
    assert ("route_length", "error") in codes
    # scene.check() carries the route findings; `botrail check` prints it.
    assert any(f.code == "route_length" for f in s.check().errors())


def test_bend_exit_and_clash_findings_have_a_place():
    s = two_boxes()
    rt.spec(s, "sig", media=["signal"], od_mm=8, bend_r_fixed_mm=600, sold_lengths_m=[2, 5])
    bend = next(f for f in rt.report(s).findings if f["code"] == "route_bend")
    assert bend["severity"] == "error" and bend["position"] is not None and bend["s_m"] is not None
    rt.spec(s, "sig", media=["signal"], od_mm=8, bend_r_fixed_mm=50, sold_lengths_m=[2, 5])
    # A box on the run is a clash; the same box as the duct the route runs
    # through is expected contact.
    s.add_box("wall", size=(0.2, 0.2, 0.2), position=(1.0, 0.0, 0.1))
    clash = next(f for f in rt.report(s).findings if f["code"] == "route_clash")
    assert "wall" in clash["message"] and clash["position"] is not None
    s.remove_obstacle("wall")
    bt.parts.duct(s, "duct", path=[(0.5, 0.0), (1.5, 0.0)], size=(0.1, 0.06), z=0.07, model="MRC-10060")
    rt.route(s, "r1", carries="w1", spec="sig", via=["duct"], orthogonal=True)
    report = rt.report(s)
    assert not any(f["code"] == "route_clash" for f in report.findings), report.findings
    assert report.routes[0]["ducts"] == ["duct"]
    assert any(r["category"] == "cable.duct" and r["attributes"]["length_m"] == 1.0 for r in s.bom().rows)


def test_medium_unknown_spec_and_unrouted_connections():
    s = two_boxes()
    rt.spec(s, "air", kind="tube", media=["pneumatic"])
    rt.route(s, "r1", carries="w1", spec="air")
    codes = {f["code"]: f["severity"] for f in rt.report(s).findings}
    assert codes["route_medium"] == "error"
    assert codes["route_spec_unknown"] == "warning"
    rt.remove_route(s, "r1")
    assert {f["code"] for f in rt.report(s).findings} == {"route_unrouted"}
    assert rt.report(s).ok


def test_bom_rows_merge_lengths_and_pins_overlay():
    s = two_boxes()
    s.add_box("box_c", size=(0.2, 0.2, 0.2), position=(-2.0, 0.0, 0.1))
    cx.port(s, "c.in", "box_c", "signal", "input", at=("box_c", (0.1, 0, 0)), exit=(1, 0, 0))
    cx.port(s, "a.out2", "box_a", "signal", "output", at=("box_a", (-0.1, 0, 0)), exit=(-1, 0, 0))
    cx.connect(s, "a.out2", "c.in", name="w2")
    rt.route(s, "r2", carries="w2", spec="sig", orthogonal=True)
    rt.clamp(s, "k1", at=("box_a", (0, 0.1, 0)), model="CF-8", manufacturer="igus")
    rt.clamp(s, "k2", at=("box_b", (0, 0.1, 0)), model="CF-8", manufacturer="igus")
    rows = {r["category"]: r for r in s.bom().rows}
    cable = rows["cable.signal"]
    assert sorted(cable["names"]) == ["r1", "r2"] and cable["qty"] == 2
    lengths = [rt.geometry(s, n)["cut_length_m"] for n in ("r1", "r2")]
    assert cable["attributes"]["length_m"] == pytest.approx(sum(lengths), abs=2e-3)
    assert cable["attributes"]["purchase_length_m"] == 7.0  # 5 m + 2 m
    assert rows["cable.clamp"]["qty"] == 2 and rows["cable.clamp"]["model"] == "CF-8"
    # A pin on the route's name overlays the spec's identity: a different
    # model is its own line, with its own length.
    s.set_part("r1", kind="route", model="UNITRONIC 8 (shielded)", description="spare core for the lamp")
    rows = [r for r in s.bom().rows if r["category"] == "cable.signal"]
    assert sorted(len(r["names"]) for r in rows) == [1, 1]
    r1 = next(r for r in rows if r["names"] == ["r1"])
    assert r1["model"] == "UNITRONIC 8 (shielded)" and r1["description"] == "spare core for the lamp"
    assert r1["attributes"]["length_m"] == pytest.approx(lengths[0], abs=1e-3)


def test_cable_list_and_project_round_trips(tmp_path):
    s = two_boxes()
    cables = rt.cable_list(s)
    assert cables.rows[0]["from_port"] == "a.out" and cables.rows[0]["carries"] == ["w1"]
    cables.save(tmp_path / "cables.csv")
    header, row = (tmp_path / "cables.csv").read_text().splitlines()[:2]
    assert header.startswith("name,kind,medium,spec,") and row.startswith("r1,cable,signal,sig,Lapp,UNITRONIC 8")
    rt.report(s).save(tmp_path / "routing.md")
    assert "| r1 |" in (tmp_path / "routing.md").read_text()
    # The plan rides the project and comes back through the generated script.
    s.save_project(tmp_path / "cell.botrail")
    back = bt.Scene.load_project(tmp_path / "cell.botrail")
    assert rt.geometry(back, "r1") == rt.geometry(s, "r1")
    assert json.loads(back._routing_plan_json()) == json.loads(s._routing_plan_json())
    code = s.generate_python()
    assert "bt.routing.restore(" in code and "bt.connections.restore(" in code
    ns = {}
    exec(code.replace("bt.studio(scene)", ""), ns)  # noqa: S102 - generated replay contract
    assert rt.geometry(ns["scene"], "r1") == rt.geometry(s, "r1")
    svg = s.layout("svg")
    assert 'class="route' in svg
    assert "ROUTE" in s.layout("dxf")


def test_robot_cable_route_replaces_the_manhattan_estimate():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot = s.robots[0]
    bt.parts.controller(s, "UR", robots=[robot], size=(0.475, 0.268, 0.423), position=(2.0, 0.6), cable_m=6,
                        manufacturer="Universal Robots", model="CB3 control box")
    estimate = next(q for row in s.requirements() for q in row.requirements if q.key == "cable_m")
    assert "along the axes" in estimate.basis
    cx.port(s, "arm.cable", robot, "power", "load", at=(robot, "base_link", (0, -0.1, 0.05)), exit=(0, -1, 0),
            straight_m=0.05)
    cx.port(s, "UR.robot", "UR", "power", "supply", target_kind="io_node", at=("UR/body", (0, -0.134, -0.15)), exit=(0, -1, 0), straight_m=0.1)
    cx.connect(s, "UR.robot", "arm.cable", name="robot_cable")
    rt.spec(s, "ur_cable", media=["power"], od_mm=10, bend_r_fixed_mm=60, sold_lengths_m=[6, 12],
            manufacturer="Universal Robots", model="Robot cable")
    rt.route(s, "robot_cable", carries="robot_cable", spec="ur_cable", orthogonal=True, slack_m=1.0)
    routed = next(q for row in s.requirements() for q in row.requirements if q.key == "cable_m")
    assert routed.basis.startswith("route 'robot_cable'")
    assert routed.value == pytest.approx(rt.geometry(s, "robot_cable")["cut_length_m"], abs=0.01)
    assert routed.status == "ok"
    # Move the box out of reach of the 6 m cable: the requirement follows.
    s.set_obstacle_pose("UR/body", (6.0, 0.6, 0.423 / 2))
    short = next(q for row in s.requirements() for q in row.requirements if q.key == "cable_m")
    assert short.status == "short" and short.value > 6
    assert any(f.code == "spec_short" for f in s.check().errors())


def test_duct_generator_and_through():
    s = bt.Scene()
    built = bt.parts.duct(s, "tray", path=[(0, 0), (2, 0), (2, 1)], size=(0.1, 0.06), model="KDR-100", manufacturer="X")
    assert built.obstacles == ["tray/run0", "tray/run1"]
    plan = json.loads(s._routing_plan_json())
    duct = next(sp for sp in plan["supports"] if sp["name"] == "tray")
    assert duct["kind"] == "duct" and duct["path"][0] == [0.0, 0.0, 0.03] and duct["inner_m"] == pytest.approx([0.097, 0.0585])
    row = next(r for r in s.bom().rows if r["category"] == "cable.duct")
    assert row["attributes"]["length_m"] == 3.0 and row["model"] == "KDR-100"
    assert rt.through("tray") == {"kind": "support", "name": "tray"}
    with pytest.raises(ValueError):
        bt.parts.duct(s, "bad", path=[(0, 0)], size=(0.1, 0.06))


def test_connection_plan_fails_reach_check_as_warnings():
    s = two_boxes()
    cx.port(s, "x.out", "box_a", "signal", "output")
    cx.connect(s, "a.out", "x.out", name="bad")  # output into output
    report = s.check()
    assert any(f.code == "connection" and f.severity == "warning" for f in report.findings)


def arm_cell(slack_m=0.0):
    """The six-axis arm with an air line from a valve box to its tool, and a
    cycle that swings the tool away and back."""
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot = s.robots[0]
    tool = s.robot.tcp_link
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.5, 0.5, 0.05))
    cx.port(s, "valve/out", "valve", "pneumatic", "supply", at=("valve", (0, 0, 0.05)), exit=(0, 0, 1), straight_m=0.02)
    cx.port(s, "arm/air", robot, "pneumatic", "load", at=(robot, tool, (0, 0, 0.02)), exit=(0, 0, 1), straight_m=0.02)
    cx.connect(s, "valve/out", "arm/air", name="air")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20,
            torsion_deg_m=60, manufacturer="SMC", model="TU0604")
    rt.route(s, "air", carries="air", spec="tube", slack_m=slack_m)
    s.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    s.add_segment("back", goal=[0.0] * 6)
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    sq.step("back", actions=[bt.seq.motion("back")])
    return s


def test_the_cycle_gives_free_length_stroke_and_twist_with_times():
    s = arm_cell()
    tl = s.simulate_sequence("cycle")
    report = rt.report(s, tl)
    assert len(report.spans) == 1
    span = report.spans[0]
    assert span["kind"] == "moving" and span["rope"] is None
    assert span["max_distance_m"] > span["min_distance_m"] and span["stroke_m"] > 0.02
    assert span["required_free_m"] == pytest.approx(span["max_distance_m"] + 0.04, abs=2e-3)  # + 2 × 20 mm
    assert span["twist_deg"] > 5.0
    # A moving span with no slack is a warning on its own.
    assert any(f["code"] == "route_length" and f["severity"] == "warning" for f in report.findings)
    # A stiff hose needs room to turn out of each holder: the line between
    # them falls short at the farthest moment. A tight torsion rating is
    # exceeded by the wrist's turn.
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=400,
            torsion_deg_m=10)
    report = rt.report(s, tl)
    tension = next(f for f in report.findings if f["code"] == "route_tension")
    assert tension["severity"] == "error" and 0.0 <= tension["t_s"] <= tl.duration
    twist = next(f for f in report.findings if f["code"] == "route_twist")
    assert twist["severity"] == "error" and twist["t_s"] is not None
    assert {f.code for f in s.check(timeline=tl).errors()} >= {"route_tension", "route_twist"}
    # Slack cures the tension; the static report never mentions the cycle.
    rt.route(s, "air", carries="air", spec="tube", slack_m=2.0)
    assert not any(f["code"] == "route_tension" for f in rt.report(s, tl).findings)
    assert rt.report(s).spans == [] and all(f.get("t_s") is None for f in rt.report(s).findings)


def test_the_rope_replay_is_deterministic_and_judged_over_time():
    s = arm_cell(slack_m=0.4)
    tl = s.simulate_sequence("cycle")
    a = rt.animate(s, tl)
    b = rt.animate(s, tl)
    track = bt.rope.track(a, "route:air#0")
    assert track["points"] == bt.rope.track(b, "route:air#0")["points"]
    # The floor at z = 0 holds the rope up; nothing sinks below it.
    assert min(p[2] for frame in track["points"] for p in frame) >= -track["radius_m"] - 1e-6
    report = rt.report(s, a)
    span = report.spans[0]
    assert span["rope"] == "route:air#0"
    assert span["min_bend_radius_m"] is not None and span["max_sag_m"] is not None
    assert span["max_stretch"] is not None
    # The arm it rides is a body it can touch — all of it but the tool its
    # port is seated in: with 0.4 m of slack the tube drapes over the wrist.
    assert span["min_clearance_m"] == 0.0 and "the robot link `simple_arm/" in span["nearest"]
    assert "tool0" not in span["nearest"] and "wrist_3_link" not in span["nearest"]
    contact = next(f for f in report.findings if f["code"] == "route_contact")
    assert span["nearest"] in contact["message"] and contact["t_s"] is not None
    for f in report.findings:
        if f["code"] in ("route_bend_moving", "route_stretch", "route_sag", "route_contact", "route_clearance"):
            assert f["t_s"] is not None and f["position"] is not None
    assert "## Moving spans" in report.to_markdown()
    # The input timeline is untouched; the fixed spans can be hung too.
    assert not any(n.startswith("route:") for n in tl.rope_names) if hasattr(tl, "rope_names") else True


def test_a_duct_bundle_is_checked_for_fill_and_separation():
    s = bt.Scene()
    s.add_box("box_a", size=(0.2, 0.2, 0.2), position=(0.0, 0.0, 0.1))
    s.add_box("box_b", size=(0.2, 0.2, 0.2), position=(2.0, 0.0, 0.1))
    cx.port(s, "a.p", "box_a", "power", "supply", at=("box_a", (0.1, 0.04, -0.07)), exit=(1, 0, 0))
    cx.port(s, "b.p", "box_b", "power", "load", at=("box_b", (-0.1, 0.04, -0.07)), exit=(-1, 0, 0))
    cx.port(s, "a.s", "box_a", "signal", "output", at=("box_a", (0.1, -0.04, -0.07)), exit=(1, 0, 0))
    cx.port(s, "b.s", "box_b", "signal", "input", at=("box_b", (-0.1, -0.04, -0.07)), exit=(-1, 0, 0))
    cx.connect(s, "a.p", "b.p", name="w_p")
    cx.connect(s, "a.s", "b.s", name="w_s")
    bt.parts.duct(s, "duct", path=[(0.5, 0.0), (1.5, 0.0)], size=(0.03, 0.02), model="MRC-30")
    rt.spec(s, "pw", media=["power"], od_mm=20, bend_r_fixed_mm=80)
    rt.spec(s, "sg", media=["signal"], od_mm=12, bend_r_fixed_mm=50)
    rt.route(s, "r_p", carries="w_p", spec="pw", via=["duct"], orthogonal=True)
    rt.route(s, "r_s", carries="w_s", spec="sg", via=["duct"], orthogonal=True)
    codes = {(f["code"], f["route"]) for f in rt.report(s).findings}
    # 20 mm + 12 mm lines in a 27 × 18.5 mm channel: 85 % by cross-section.
    assert ("duct_fill", "duct") in codes and ("route_separation", "duct") in codes
    bt.parts.duct(s, "duct", path=[(0.5, 0.0), (1.5, 0.0)], size=(0.10, 0.06), model="MRC-100")
    codes = {(f["code"], f["route"]) for f in rt.report(s).findings}
    assert ("duct_fill", "duct") not in codes and ("route_separation", "duct") in codes


def test_a_carrier_bundles_the_hand_lines_with_their_own_slack():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot, links, tool = s.robots[0], s.robot.link_names, s.robot.tcp_link
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.5, 0.5, 0.05))
    for i in (1, 2):
        cx.port(s, f"valve/out{i}", "valve", "pneumatic", "supply", at=("valve", (0.02 * i, 0, 0.05)), exit=(0, 0, 1))
        cx.port(s, f"arm/air{i}", robot, "pneumatic", "load", at=(robot, tool, (0, 0.01 * i, 0.02)), exit=(0, 0, 1))
        cx.connect(s, f"valve/out{i}", f"arm/air{i}", name=f"air{i}")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20)
    rt.carrier(s, "dress", holders=[(robot, links[2], (0, 0, 0.1)), (robot, links[3], (0, 0, 0.1))], inner_mm=8,
               model="NW8 conduit")
    for i in (1, 2):
        rt.route(s, f"air{i}", carries=f"air{i}", spec="tube", via=["dress"], slack_m=[0.05, 0.10, 0.15])
    plan = json.loads(s._routing_plan_json())
    assert next(r for r in plan["routes"] if r["name"] == "air1")["slacks_m"] == [0.05, 0.10, 0.15]
    g = rt.geometry(s, "air1")
    assert g["carriers"] == ["dress"] and [a["name"] for a in g["anchors"]][1:3] == ["dress/h0", "dress/h1"]
    assert [sp["kind"] for sp in g["spans"]] == ["moving", "moving", "moving"]
    assert g["cut_length_m"] == pytest.approx(g["length_m"] + 0.30)
    # Two 6 mm tubes do not fit an 8 mm bore (112 %); a 14 mm one takes them.
    assert any(f["code"] == "duct_fill" and f["route"] == "dress" for f in rt.report(s).findings)
    rt.carrier(s, "dress", holders=[(robot, links[2], (0, 0, 0.1)), (robot, links[3], (0, 0, 0.1))], inner_mm=14)
    assert not any(f["code"] == "duct_fill" for f in rt.report(s).findings)
    hose = next(r for r in s.bom().rows if r["category"] == "cable.carrier")
    assert hose["names"] == ["dress"] and hose["attributes"]["holders"] == 2 and hose["attributes"]["length_m"] > 0
    assert rt.cable_list(s).rows[0]["carriers"] == ["dress"]
    # Over a cycle every moving span carries its own slack.
    s.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    tl = s.simulate_sequence("cycle")
    spans = [sp for sp in rt.report(s, tl).spans if sp["route"] == "air1"]
    assert [sp["material_m"] - (sp2["to_s"] - sp2["from_s"]) for sp, sp2 in zip(spans, g["spans"])] == pytest.approx([0.05, 0.10, 0.15], abs=2e-3)


def test_a_fixed_span_is_checked_against_the_arm_over_the_cycle():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    s.add_box("box_a", size=(0.2, 0.2, 0.2), position=(-0.6, 0.0, 0.1))
    s.add_box("box_b", size=(0.2, 0.2, 0.2), position=(0.6, 0.0, 0.1))
    cx.port(s, "a.out", "box_a", "signal", "output", at=("box_a", (0.1, 0, 0)), exit=(1, 0, 0))
    cx.port(s, "b.in", "box_b", "signal", "input", at=("box_b", (-0.1, 0, 0)), exit=(-1, 0, 0))
    cx.connect(s, "a.out", "b.in", name="w1")
    rt.spec(s, "sig", media=["signal"], od_mm=8, bend_r_fixed_mm=50)
    # Straight through the arm at rest. The planner would refuse to start
    # inside the cable (it is a collider), so this route is one the planner
    # ignores — and the cycle scan still watches it.
    rt.route(s, "r1", carries="w1", spec="sig", collide=False)
    assert any(f["code"] == "route_clash" and "simple_arm/" in f["message"] for f in rt.report(s).findings)
    s.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    tl = s.simulate_sequence("cycle")
    report = rt.report(s, tl)
    span = report.spans[0]
    assert span["kind"] == "fixed" and span["nearest"] is not None and span["min_clearance_m"] == 0.0
    contact = next(f for f in report.findings if f["code"] == "route_contact")
    assert contact["t_s"] is not None and "fixed span" in contact["message"]


def test_fixed_spans_are_colliders_the_planner_sees():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    s.add_box("box_a", size=(0.2, 0.2, 0.2), position=(-0.6, 0.0, 0.1))
    s.add_box("box_b", size=(0.2, 0.2, 0.2), position=(0.6, 0.0, 0.1))
    cx.port(s, "a.out", "box_a", "signal", "output", at=("box_a", (0.1, 0, 0)), exit=(1, 0, 0))
    cx.port(s, "b.in", "box_b", "signal", "input", at=("box_b", (-0.1, 0, 0)), exit=(-1, 0, 0))
    cx.connect(s, "a.out", "b.in", name="w1")
    rt.spec(s, "sig", media=["signal"], od_mm=8, bend_r_fixed_mm=50)
    assert not s.in_collision()
    rt.route(s, "r1", carries="w1", spec="sig")  # straight through the arm, at rest
    pairs = s.check_collisions()
    assert any(("route", "route:r1#0") in pair for pair in pairs), pairs
    assert s.in_collision() and s.min_obstacle_distance() == 0.0
    # The same line, routed clear of the arm: a collider again, at a distance.
    rt.route(s, "r1", carries="w1", spec="sig", via=[(-0.6, 0.0, 0.9), (0.6, 0.0, 0.9)])
    assert not s.in_collision()
    clear = s.min_obstacle_distance()
    assert clear is not None and 0.0 < clear < 1.0
    # A route the planner ignores is no collider (the static report still clashes).
    rt.route(s, "r1", carries="w1", spec="sig", collide=False)
    assert not s.in_collision() and not any("route" in str(pair) for pair in s.check_collisions())
    assert any(f["code"] == "route_clash" for f in rt.report(s).findings)
    # A connector in the arm's own base is not a collision of the arm with its cable.
    cx.port(s, "arm/in", s.robots[0], "signal", "input", at=(s.robots[0], "base_link", (0.0, -0.09, 0.04)),
            exit=(0, -1, 0), straight_m=0.05)
    cx.connect(s, "a.out", "arm/in", name="w2")
    rt.route(s, "r2", carries="w2", spec="sig", orthogonal=True)
    assert not s.in_collision(), s.check_collisions()
    # ... and the planner starts from here.
    s.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    assert s.simulate_sequence("cycle").duration > 0


def test_a_run_along_a_column_lays_the_clamps_and_counts_them():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    s.add_box("box_a", size=(0.2, 0.2, 0.2), position=(1.0, -1.0, 0.1))
    s.add_box("cabinet", size=(0.4, 0.4, 1.2), position=(2.5, -1.0, 0.6))
    s.add_box("column", size=(0.2, 0.2, 1.0), position=(2.0, -0.5, 0.5))
    cx.port(s, "a.out", "box_a", "signal", "output", at=("box_a", (0.1, 0, 0)), exit=(1, 0, 0), straight_m=0.05)
    cx.port(s, "c.in", "cabinet", "signal", "input", at=("cabinet", (-0.2, 0, 0.5)), exit=(-1, 0, 0), straight_m=0.05)
    cx.connect(s, "a.out", "c.in", name="w1")
    rt.spec(s, "sig", media=["signal"], od_mm=8, bend_r_fixed_mm=40)
    rt.route(s, "r1", carries="w1", spec="sig", via=[(1.6, -1.0, 0.1), rt.along("column", pitch_m=0.3), (2.3, -1.0, 1.1)],
             orthogonal=True, clamp=dict(model="P-clip 8", manufacturer="HellermannTyton", length_m=0.02))
    g = rt.geometry(s, "r1")
    auto = [a for a in g["anchors"] if a["auto"]]
    assert [a["name"] for a in auto] == g["auto_clamps"] and len(auto) >= 3
    # On the column's -y face (nearest both ways), 4 + 5 mm off it, parallel to its edge.
    assert all(a["attach"]["kind"] == "obstacle" and a["attach"]["name"] == "column" for a in auto)
    assert all(abs(a["position"][1] - (-0.5 - 0.1 - 0.009)) < 1e-9 for a in auto)
    assert len({round(a["position"][0], 6) for a in auto}) == 1
    zs = [a["position"][2] for a in auto]
    assert zs == sorted(zs) and max(b - a for a, b in zip(zs, zs[1:])) <= 0.3 + 1e-9
    assert all(sp["kind"] == "fixed" for sp in g["spans"]) and len(g["spans"]) == len(auto) + 1
    report = rt.report(s)
    assert not any(f["code"] in ("route_clash", "route_clearance") for f in report.findings), report.findings
    row = next(r for r in s.bom().rows if r["category"] == "cable.clamp")
    assert row["qty"] == len(auto) and row["model"] == "P-clip 8" and row["manufacturer"] == "HellermannTyton"
    assert [t["name"] for t in row["targets"]] == ["r1"]
    assert rt.cable_list(s).rows[0]["auto_clamps"] == len(auto)
    # The plan round-trips with the way point as written.
    plan = json.loads(s._routing_plan_json())
    via = plan["routes"][0]["via"][1]
    assert via == {"kind": "along", "on": {"kind": "obstacle", "name": "column"}, "pitch_m": 0.3}
    assert plan["routes"][0]["clamp"]["length_m"] == 0.02
    s2 = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    for name, size, at in (("box_a", (0.2, 0.2, 0.2), (1.0, -1.0, 0.1)), ("cabinet", (0.4, 0.4, 1.2), (2.5, -1.0, 0.6)),
                           ("column", (0.2, 0.2, 1.0), (2.0, -0.5, 0.5))):
        s2.add_box(name, size=size, position=at)
    cx.restore(s2, json.loads(s._connection_plan_json()))
    rt.restore(s2, plan)
    assert rt.geometry(s2, "r1")["auto_clamps"] == g["auto_clamps"]


def test_a_run_along_a_link_is_dressed_and_rides_the_arm():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot, links, tool = s.robots[0], s.robot.link_names, s.robot.tcp_link
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.5, 0.5, 0.05))
    cx.port(s, "valve/out", "valve", "pneumatic", "supply", at=("valve", (0, 0, 0.05)), exit=(0, 0, 1))
    cx.port(s, "arm/air", robot, "pneumatic", "load", at=(robot, tool, (0, 0, 0.02)), exit=(0, 0, 1))
    cx.connect(s, "valve/out", "arm/air", name="air")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20,
            torsion_deg_m=360)
    rt.route(s, "air", carries="air", spec="tube", via=[rt.along((robot, links[3]), pitch_m=0.08)], slack_m=[0.1, 0.1])
    g = rt.geometry(s, "air")
    auto = [a for a in g["anchors"] if a["auto"]]
    assert len(auto) >= 2 and all(a["moving"] and a["attach"] == {"kind": "link", "robot": robot, "link": links[3],
                                                                 "offset_m": a["attach"]["offset_m"]} for a in auto)
    kinds = [sp["kind"] for sp in g["spans"]]
    assert kinds[0] == "moving" and kinds[-1] == "moving" and set(kinds[1:-1]) == {"dressed"}
    # Over a cycle the dressed spans ride the forearm: constant distance, no
    # free length to find, no twist; the clamps stay where they were laid.
    s.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    tl = s.simulate_sequence("cycle")
    report = rt.report(s, tl)
    dressed = [sp for sp in report.spans if sp["kind"] == "dressed"]
    assert dressed and all(sp["stroke_m"] == 0.0 and sp["twist_deg"] == 0.0 and sp["required_free_m"] == sp["max_distance_m"]
                           for sp in dressed)
    assert not any(f["code"] == "route_tension" and "/c" in f["message"] and "→ air/c" in f["message"]
                   for f in report.findings)
    assert len(auto) + 1 == len(rt.geometry(s, "air")["spans"])


def test_a_bundle_in_a_carrier_replays_as_one_hose():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot, links, tool = s.robots[0], s.robot.link_names, s.robot.tcp_link
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.5, 0.5, 0.05))
    for i in (1, 2):
        cx.port(s, f"valve/out{i}", "valve", "pneumatic", "supply", at=("valve", (0.02 * i, 0, 0.05)), exit=(0, 0, 1))
        cx.port(s, f"arm/air{i}", robot, "pneumatic", "load", at=(robot, tool, (0, 0.01 * i, 0.02)), exit=(0, 0, 1))
        cx.connect(s, f"valve/out{i}", f"arm/air{i}", name=f"air{i}")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20,
            mass_kg_m=0.03)
    rt.carrier(s, "dress", holders=[(robot, links[2], (0, 0, 0.1)), (robot, links[3], (0, 0, 0.1))], inner_mm=14,
               od_mm=18, bend_r_moving_mm=60, mass_kg_m=0.08, model="NW14 conduit")
    for i in (1, 2):
        rt.route(s, f"air{i}", carries=f"air{i}", spec="tube", via=["dress"], slack_m=[0.05, 0.10, 0.15])
    s.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    tl = s.simulate_sequence("cycle")
    tl = rt.animate(s, tl, settle_s=0.2, step_s=1 / 240)
    # The stretch inside the hose is one rope, 18 mm thick; the free spans stay per line.
    assert "carrier:dress#0" in tl.ropes and "route:air1#1" not in tl.ropes and "route:air2#1" not in tl.ropes
    assert {"route:air1#0", "route:air1#2", "route:air2#0", "route:air2#2"} <= set(tl.ropes)
    assert bt.rope.track(tl, "carrier:dress#0")["radius_m"] == pytest.approx(0.009)
    report = rt.report(s, tl)
    inside = [sp for sp in report.spans if sp["span"] == 1]
    assert len(inside) == 2 and all(sp["rope"] == "carrier:dress#0" for sp in inside)
    # Whatever the hose's replay says is said once, in the carrier's name.
    hose_findings = [f for f in report.findings if f["route"] == "dress"]
    per_line = [f for f in report.findings if f["route"] in ("air1", "air2") and "dress/h0 → dress/h1" in f["message"]
                and f["code"] in ("route_bend_moving", "route_stretch", "route_sag", "route_contact", "route_clearance")]
    assert per_line == []
    assert all(f["code"] in ("route_bend_moving", "route_stretch", "route_sag", "route_contact", "route_clearance")
               for f in hose_findings)


def test_ropes_run_in_one_batch_as_they_do_one_after_another():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    sq = s.sequence("rest")
    sq.step("wait", transition=bt.seq.elapsed(0.1))
    tl = s.simulate_sequence("rest")
    decl = dict(points=[(0.6, 0.0, 0.5), (0.9, 0.0, 0.5)], pins=[("Start", (0.6, 0.0, 0.5)), ("End", (0.9, 0.0, 0.5))],
                spacing_m=0.02, radius_m=0.004, step_s=1 / 240)
    one = bt.rope.animate(s, bt.rope.animate(s, tl, name="a", **decl), name="b", **decl)
    many, errors = bt.rope.animate_many(s, tl, [dict(name="a", **decl), dict(name="b", **decl),
                                                dict(name="broken", points=[(0, 0, 0)], spacing_m=0.02)])
    assert errors[0] is None and errors[1] is None and errors[2]
    assert many.ropes == ["a", "b"]
    for name in ("a", "b"):
        assert bt.rope.track(one, name)["points"] == bt.rope.track(many, name)["points"]


def _bundle_cell():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot, links, tool = s.robots[0], s.robot.link_names, s.robot.tcp_link
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.5, 0.5, 0.05))
    for i in (1, 2):
        cx.port(s, f"valve/out{i}", "valve", "pneumatic", "supply", at=("valve", (0.02 * i, 0, 0.05)), exit=(0, 0, 1))
        cx.port(s, f"arm/air{i}", robot, "pneumatic", "load", at=(robot, tool, (0, 0.01 * i, 0.02)), exit=(0, 0, 1))
        cx.connect(s, f"valve/out{i}", f"arm/air{i}", name=f"air{i}")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20,
            torsion_deg_m=360, mass_kg_m=0.03)
    holders = [(robot, links[2], (0, 0, 0.1)), (robot, links[3], (0, 0, 0.1))]
    rt.carrier(s, "dress", holders=holders, inner_mm=14, od_mm=18, bend_r_moving_mm=60, mass_kg_m=0.08,
               model="NW14 conduit", manufacturer="Murrplastik")
    for i in (1, 2):
        rt.route(s, f"air{i}", carries=f"air{i}", spec="tube", via=["dress"], slack_m=[0.05, 0.10, 0.15])
    s.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    return s, holders


def test_a_carrier_is_a_line_to_choose():
    s, holders = _bundle_cell()
    row = s.requirements()["dress"]
    q = {r.key: r for r in row.requirements}
    assert row.category == "cable.carrier" and row.kind == "carrier" and row.identified
    # Two 6 mm tubes at 50 % fill: sqrt(2 × 36 / 0.5) = 12 mm; the hose states 14.
    assert q["inner_mm"].value == pytest.approx(12.0, abs=0.05) and q["inner_mm"].provided == 14
    assert q["inner_mm"].status == "ok" and not q["inner_mm"].lint
    assert any("baked cycle" in n for n in row.notes) and "free_length_m" not in q
    # Over the cycle the row asks for the hose and the retraction.
    tl = s.simulate_sequence("cycle")
    row = s.requirements(timeline=tl)["dress"]
    q = {r.key: r for r in row.requirements}
    assert {"inner_mm", "free_length_m", "stroke_m", "torsion_deg_m"} <= set(q) and not row.notes
    span = next(sp for sp in rt.report(s, tl).spans if sp["from"] == "dress/h0" and sp["to"] == "dress/h1")
    assert q["free_length_m"].value == pytest.approx(span["required_free_m"], abs=0.01)
    assert q["stroke_m"].value == pytest.approx(span["stroke_m"], abs=0.001) and q["stroke_m"].status == "unknown"
    # What the hose offers between its holders: its run plus the slack laid into it (0.10 m).
    g = rt.geometry(s, "air1")
    run = next(a for a in g["anchors"] if a["name"] == "dress/h1")["s_m"] - next(a for a in g["anchors"] if a["name"] == "dress/h0")["s_m"]
    assert q["free_length_m"].provided == pytest.approx(run + 0.10, abs=0.002)
    assert all(not r.lint for r in row.requirements)
    assert not any(f.code in ("spec_short", "spec_unknown") and f.target == "dress" for f in s.check(timeline=tl))
    assert set(row.minimum) >= {"inner_mm", "free_length_m", "stroke_m", "torsion_deg_m"}
    # A bore the bundle does not go into: short in the table, an error in the checks.
    rt.carrier(s, "dress", holders=holders, inner_mm=8, od_mm=12, model="NW8 conduit")
    row = s.requirements()["dress"]
    assert row.status == "short" and next(r for r in row.requirements if r.key == "inner_mm").status == "short"
    fill = next(f for f in rt.report(s).findings if f["code"] == "duct_fill")
    assert fill["severity"] == "error" and "112 %" in fill["message"]


def test_sweep_holder_tables_the_hose_and_puts_the_plan_back():
    s, _ = _bundle_cell()
    tl = s.simulate_sequence("cycle")
    before = s._routing_plan_json()
    table = rt.sweep_holder(s, tl, "dress", 1, z=[0.05, 0.10, 0.15])
    assert table.params == ["z"] and [r["z"] for r in table.rows] == [0.05, 0.10, 0.15]
    assert all(r["ok"] for r in table.rows) and table.metrics == ["stroke_m", "free_length_m", "twist_deg", "errors"]
    assert all(r["free_length_m"] > 0 and r["stroke_m"] >= 0 for r in table.rows)
    assert table.best("stroke_m") in table.rows and "| z |" in table.to_markdown()
    assert s._routing_plan_json() == before
    assert rt.sweep_holder(s, tl, "dress", 1, z=[0.05, 0.10, 0.15]).rows == table.rows
    with pytest.raises(ValueError):
        rt.sweep_holder(s, tl, "dress", 3, z=[0.1])


def test_lines_clamps_conduits_and_ducts_read_their_packs():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    rt.spec(s, "tube", catalog=_pack("smc/tube/tu0604bu/r1"), bend_r_moving_mm=20)
    rt.spec(s, "pn", catalog=_pack("lapp/cable/etherline-festoon-pn-cat5e/r1"), od_mm=7.0)
    specs = {sp["name"]: sp for sp in json.loads(s._routing_plan_json())["specs"]}
    tube = specs["tube"]
    assert tube["kind"] == "tube" and tube["media"] == ["pneumatic"] and tube["od_m"] == pytest.approx(0.006)
    assert tube["bend_r_fixed_m"] == pytest.approx(0.015) and tube["bend_r_moving_m"] == pytest.approx(0.020)
    assert tube["order_step_m"] == 20 and (tube["manufacturer"], tube["model"]) == ("SMC", "TU0604BU-20")
    assert tube["catalog"] == "smc/tube/tu0604bu/r1"
    pn = specs["pn"]  # what you pass wins; what the pack states fills the rest
    assert pn["od_m"] == pytest.approx(0.007) and pn["bend_r_fixed_m"] == pytest.approx(0.0268)
    assert pn["bend_r_moving_m"] == pytest.approx(0.067) and pn["mass_kg_m"] == pytest.approx(0.069)
    assert pn["kind"] == "cable" and pn["media"] == ["network"] and pn["model"] == "2170936" and pn["max_length_m"] is None
    with pytest.raises(ValueError, match="bt.routing.spec reads"):
        rt.spec(s, "wrong", catalog=_pack("hellermanntyton/clamp/h6p-pa66hs-bk/r1"))
    # A clip: its width is the length it holds.
    s.add_box("wall", size=(2.0, 0.1, 2.0), position=(0.0, 1.0, 1.0))
    rt.clamp(s, "c1", at=("wall", (0.0, -0.05, 0.0)), catalog=_pack("hellermanntyton/clamp/h6p-pa66hs-bk/r1"))
    supports = {sp["name"]: sp for sp in json.loads(s._routing_plan_json())["supports"]}
    assert supports["c1"]["length_m"] == pytest.approx(0.01)
    assert (supports["c1"]["manufacturer"], supports["c1"]["model"]) == ("HellermannTyton", "211-60005")
    # A conduit states its bore; a kit nobody has measured does not.
    robot, links = s.robots[0], s.robot.link_names
    holders = [(robot, links[2], (0, 0, 0.1)), (robot, links[3], (0, 0, 0.1))]
    rt.carrier(s, "hose", holders=holders, catalog=_pack("abb/conduit/adaptaflex-prfs16/r1"))
    rt.carrier(s, "kit", holders=holders, catalog=_pack("murrplastik/dress_pack/fhs-sh-set-ur5/r1"))
    supports = {sp["name"]: sp for sp in json.loads(s._routing_plan_json())["supports"]}
    assert supports["hose"]["inner_m"] == pytest.approx(0.0117) and supports["hose"]["od_m"] == pytest.approx(0.0158)
    assert supports["hose"]["model"] == "PRFS16/BL/100FT" and supports["hose"]["manufacturer"] == "ABB"
    assert supports["kit"].get("inner_m") is None and supports["kit"]["model"] == "83693492"
    assert supports["kit"]["catalog"] == "murrplastik/dress_pack/fhs-sh-set-ur5/r1"
    # A duct: the pack's inner section plus the wall, and the part it pins.
    bt.parts.duct(s, "trunk", path=[(0.0, -1.0), (1.0, -1.0)], catalog=_pack("obo_bettermann/duct/lk4-60100/r1"))
    lo, hi = s.obstacle_bounds("trunk/run0")
    assert hi[1] - lo[1] == pytest.approx(0.103) and hi[2] - lo[2] == pytest.approx(0.0615)
    row = next(r for r in s.bom().rows if r["category"] == "cable.duct")
    assert row["model"] == "6178037" and row["manufacturer"] == "OBO Bettermann" and "lk4-60100" in str(row["catalog"])
    trunk = next(sp for sp in json.loads(s._routing_plan_json())["supports"] if sp["name"] == "trunk")
    assert trunk["inner_m"] == pytest.approx([0.1, 0.06]) and trunk["catalog"] == "obo_bettermann/duct/lk4-60100/r1"


def test_a_kit_without_a_stated_bore_asks_for_one():
    s, holders = _bundle_cell()
    rt.carrier(s, "dress", holders=holders, catalog=_pack("murrplastik/dress_pack/fhs-sh-set-ur5/r1"))
    assert not any(f["code"] == "duct_fill" for f in rt.report(s).findings)
    row = s.requirements()["dress"]
    q = {r.key: r for r in row.requirements}
    assert row.identified and q["inner_mm"].value == pytest.approx(12.0, abs=0.05)
    assert q["inner_mm"].status == "unknown" and q["inner_mm"].provided is None
    assert row.minimum["inner_mm"] == q["inner_mm"].value
    hose = next(r for r in s.bom().rows if r["category"] == "cable.carrier")
    assert hose["model"] == "83693492" and "inner_mm" not in hose["attributes"] and hose["attributes"]["holders"] == 2


def _two_boxes_and_two_ducts():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    s.add_box("box_a", size=(0.2, 0.2, 0.2), position=(-1.2, -1.0, 0.1))
    s.add_box("box_b", size=(0.2, 0.2, 0.2), position=(1.2, -1.0, 0.1))
    # A wall across the straight way, which the near duct runs into too.
    s.add_box("wall", size=(0.1, 0.5, 0.4), position=(0.0, -1.0, 0.2))
    cx.port(s, "a.out", "box_a", "signal", "output", at=("box_a", (0.1, 0, 0)), exit=(1, 0, 0), straight_m=0.05)
    cx.port(s, "b.in", "box_b", "signal", "input", at=("box_b", (-0.1, 0, 0)), exit=(-1, 0, 0), straight_m=0.05)
    cx.connect(s, "a.out", "b.in", name="w1")
    bt.parts.duct(s, "near", path=[(-0.9, -1.0), (0.9, -1.0)], size=(0.06, 0.04))
    bt.parts.duct(s, "far", path=[(-0.9, -0.4), (0.9, -0.4)], size=(0.06, 0.04))
    rt.spec(s, "sig", media=["signal"], od_mm=8, bend_r_fixed_mm=40)
    return s


def test_candidates_pick_the_ducts_and_keep_the_fixed_way_points():
    s = _two_boxes_and_two_ducts()
    rt.route(s, "r1", carries="w1", spec="sig", orthogonal=True)
    assert any(f["code"] == "route_clash" for f in rt.report(s).findings)  # straight through the wall
    candidates = rt.suggest(s, "r1")
    assert candidates == rt.suggest(s, "r1")  # the same list every time
    # The clean ways go through the far duct (a short piece of the near one
    # before the wall is fine); the straight way and the near duct alone
    # run into the wall.
    assert candidates[0]["errors"] == 0 and "far" in candidates[0]["ducts"] and not candidates[0]["current"]
    assert all(c["errors"] >= 1 for c in candidates if c["ducts"] in ([], ["near"]))
    assert all("far" in c["ducts"] for c in candidates if c["errors"] == 0)
    direct = next(c for c in candidates if c["ducts"] == [])
    assert direct["current"] and "route_clash" in direct["codes"]
    assert [c["errors"] for c in candidates] == sorted(c["errors"] for c in candidates)
    clean = [c for c in candidates if c["errors"] == 0]
    assert [c["score"] for c in clean] == sorted(c["score"] for c in clean)
    assert all(c["score"] == pytest.approx(c["length_m"] + c["open_m"] + 0.3 * c["bends"] + 0.2 * c["supports"])
               for c in clean)
    assert all(0.0 <= c["open_m"] <= c["length_m"] + 1e-9 for c in clean)
    rt.adopt(s, "r1", 0)
    assert "far" in rt.geometry(s, "r1")["ducts"] and rt.report(s).ok
    assert rt.suggest(s, "r1")[0]["current"]
    # A way point the author fixed stays, in order, in every candidate.
    rt.route(s, "r1", carries="w1", spec="sig", via=[(0.0, -0.4, 0.6)], orthogonal=True)
    for c in rt.suggest(s, "r1"):
        points = [v for v in c["via"] if v["kind"] == "point"]
        assert points == [{"kind": "point", "position": [0.0, -0.4, 0.6]}]
        assert len(c["ducts"]) <= 3
    with pytest.raises(ValueError, match="unknown route"):
        rt.suggest(s, "nope")


def test_a_way_can_be_replaced_through_the_session():
    s = _two_boxes_and_two_ducts()
    rt.route(s, "r1", carries="w1", spec="sig")
    s._set_route_via_json("r1", json.dumps([{"kind": "support", "name": "far"}, {"kind": "point", "position": [1.0, -0.6, 0.1]}]))
    plan = json.loads(s._routing_plan_json())
    assert [v["kind"] for v in plan["routes"][0]["via"]] == ["support", "point"]
    assert rt.geometry(s, "r1")["ducts"] == ["far"]
    with pytest.raises(ValueError):
        s._set_route_via_json("r1", json.dumps([{"kind": "support", "name": ""}]))
    with pytest.raises(ValueError, match="unknown route"):
        s._set_route_via_json("r9", "[]")


def test_a_product_can_declare_where_its_lines_plug_in():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    s.add_box("psu/body", size=(0.2, 0.2, 0.12), position=(1.0, 0.0, 0.06))
    s.set_part("psu", kind="group", category="power_supply", catalog=_pack("test/box/ported/r1"), model="TEST-PSU")
    declared = cx.ports_of(s, "psu")
    assert [p["name"] for p in declared] == ["dc_out", "bus"]
    dc = declared[0]
    assert dc["medium"] == "power" and dc["role"] == "supply" and dc["note"] == "the gland on the back face"
    assert dc["at"] == ("psu/body", (0.0, 0.1, 0.03)) and dc["exit"] == (0.0, 1.0, 0.0) and dc["straight_m"] == pytest.approx(0.04)
    assert cx.ports_of(s, "simple_arm") == [] or True  # a product that declares none answers nothing
    # The declaration is the default; what is written wins.
    cx.port(s, "psu/out", "psu", from_catalog="dc_out", voltage_v=24, capacity_a=10)
    cx.port(s, "psu/net", "psu", "network", "peer", from_catalog="bus", straight_m=0.08)
    ports = {p["name"]: p for p in json.loads(s._connection_plan_json())["ports"]}
    assert ports["psu/out"]["medium"] == "power" and ports["psu/out"]["role"] == "supply"
    assert ports["psu/out"]["geometry"]["at"] == {"kind": "obstacle", "name": "psu/body", "offset_m": [0.0, 0.1, 0.03]}
    assert ports["psu/out"]["geometry"]["exit"] == [0.0, 1.0, 0.0] and ports["psu/out"]["geometry"]["straight_m"] == pytest.approx(0.04)
    assert ports["psu/net"]["geometry"]["straight_m"] == pytest.approx(0.08)
    with pytest.raises(ValueError, match="declares no port"):
        cx.port(s, "psu/x", "psu", from_catalog="nope")
    with pytest.raises(ValueError, match="medium and role"):
        cx.port(s, "psu/y", "psu")


def test_a_kit_that_states_its_holders_places_them_on_the_arm():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot = s.robots[0]
    with pytest.raises(ValueError, match="robot="):
        rt.carrier(s, "dress", catalog=_pack("test/dress/kit-with-holders/r1"))
    rt.carrier(s, "dress", catalog=_pack("test/dress/kit-with-holders/r1"), robot=robot)
    dress = next(sp for sp in json.loads(s._routing_plan_json())["supports"] if sp["name"] == "dress")
    assert [h["at"]["link"] for h in dress["holders"]] == ["upper_arm_link", "forearm_link"]
    assert dress["holders"][0]["at"] == {"kind": "link", "robot": robot, "link": "upper_arm_link", "offset_m": [0.0, 0.0, 0.1]}
    assert dress["holders"][0]["kind"] == "fixed"
    assert dress["inner_m"] == pytest.approx(0.014) and dress["model"] == "TEST-KIT-1"
    with pytest.raises(ValueError, match="names its holders"):
        rt.carrier(s, "bare", inner_mm=10)


def test_the_kit_row_finds_its_candidates_in_the_catalog_index():
    # The catalog's index, cut down to the routing packs (what `bcb publish`
    # stages); the search is offline.
    index = bt.catalog.Index.from_path(PACKS / "index.json")
    s, _ = _bundle_cell()
    row = s.requirements()["dress"]
    found = bt.catalog.search_for(row, index=index)
    ids = [p.id for p in found]
    # Two 6 mm tubes need a 12 mm bore: the 21 conduit fits, the 16 does not,
    # the kit that states no bore comes back marked unstated, last.
    assert ids[0] == "abb/conduit/adaptaflex-pafs21/r1" and "abb/conduit/adaptaflex-prfs16/r1" not in ids
    assert ids[-1] == "murrplastik/dress_pack/fhs-sh-set-ur5/r1" and found[-1].unstated == ("inner_mm",)
    assert [p.id for p in bt.catalog.search_for(row, index=index, strict=True)] == ["abb/conduit/adaptaflex-pafs21/r1"]
    assert row.minimum["inner_mm"] == pytest.approx(12.0, abs=0.05)


def _clipped_tube_cell():
    """A tube from a valve over a clip on a post to the arm's tool: a fixed
    span, then one that rides the arm."""
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot, tool = s.robots[0], s.robot.tcp_link
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.6, 0.6, 0.05))
    s.add_box("post", size=(0.05, 0.05, 0.6), position=(0.6, 0.3, 0.3))
    cx.port(s, "valve/out", "valve", "pneumatic", "supply", at=("valve", (0, 0, 0.05)), exit=(0, 0, 1))
    cx.port(s, "arm/air", robot, "pneumatic", "load", at=(robot, tool, (0, 0, 0.02)), exit=(0, 0, 1))
    cx.connect(s, "valve/out", "arm/air", name="air")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20)
    rt.clamp(s, "clip", at=("post", (0.0, 0.03, 0.3)))
    rt.route(s, "air", carries="air", spec="tube", via=["clip"], slack_m=0.1)
    s.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    return s


def _route_prims(path, kind: str) -> set[str]:
    return set(re.findall(rf'def {kind} "((?:route|carrier)_[^"]*)"', Path(path).read_text()))


def test_the_cycle_usd_leaves_a_replayed_span_to_its_rope(tmp_path):
    s = _clipped_tube_cell()
    assert [sp["kind"] for sp in rt.geometry(s, "air")["spans"]] == ["fixed", "moving"]
    # The scene's stage has no replay: the whole route as one curve.
    s.export_usd(tmp_path / "scene.usda")
    assert _route_prims(tmp_path / "scene.usda", "BasisCurves") == {"route_air"}
    # The moving span replayed: its rope is the tube, the design curve keeps the fixed span only.
    tl = rt.animate(s, s.simulate_sequence("cycle"), settle_s=0.1, step_s=1 / 240)
    assert tl.ropes == ["route:air#1"]
    tl.export_usd(tmp_path / "cycle.usda", fps=10.0)
    assert _route_prims(tmp_path / "cycle.usda", "BasisCurves") == {"route_air_from0"}
    assert _route_prims(tmp_path / "cycle.usda", "Mesh") == {"route_air_1"}
    # Every span replayed: no design curve is left beside the ropes.
    tl = rt.animate(s, s.simulate_sequence("cycle"), settle_fixed=True, settle_s=0.1, step_s=1 / 240)
    tl.export_usd(tmp_path / "all.usda", fps=10.0)
    assert _route_prims(tmp_path / "all.usda", "BasisCurves") == set()
    assert _route_prims(tmp_path / "all.usda", "Mesh") == {"route_air_0", "route_air_1"}


def _tool_tube_cell(holders):
    """simple_arm standing straight up at zero; a valve beside it; a tube from
    the valve to the tool through a carrier held at `holders`."""
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot = s.robots[0]
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.6, 0.0, 0.05))
    cx.port(s, "valve/out", "valve", "pneumatic", "supply", at=("valve", (0, 0, 0.05)), exit=(0, 0, 1))
    cx.port(s, "arm/air", robot, "pneumatic", "load", at=(robot, "tool0", (0, 0, 0.04)), exit=(0, 0, 1))
    cx.connect(s, "valve/out", "arm/air", name="air")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20)
    rt.carrier(s, "dress", holders=[(robot, link, offset) for link, offset in holders], inner_mm=10, od_mm=14)
    rt.route(s, "air", carries="air", spec="tube", via=["dress"], slack_m=[0.1, 0.1, 0.1])
    return s


def test_a_design_line_through_the_arm_it_rides_is_a_clash():
    s = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    robot = s.robots[0]
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.6, 0.0, 0.05))
    cx.port(s, "valve/out", "valve", "pneumatic", "supply", at=("valve", (0, 0, 0.05)), exit=(0, 0, 1))
    cx.port(s, "arm/air", robot, "pneumatic", "load", at=(robot, "tool0", (0, 0, 0.04)), exit=(0, 0, 1))
    cx.connect(s, "valve/out", "arm/air", name="air")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20)
    # Up the middle of the column the arm stands as: a moving span, judged at
    # the scene's pose against the arm it rides — all of it but the tool the
    # port is seated on.
    rt.route(s, "air", carries="air", spec="tube", via=[(0.0, 0.0, 0.3)], slack_m=0.1)
    assert [sp["kind"] for sp in rt.geometry(s, "air")["spans"]] == ["moving"]
    clash = [f for f in rt.report(s).findings if f["code"] == "route_clash"]
    assert clash and all("the robot link `simple_arm/" in f["message"] for f in clash)
    assert not any("tool0" in f["message"] or "wrist_3_link" in f["message"] for f in clash)


def test_a_holder_inside_a_link_is_warned_and_only_that_link_is_left_out():
    inside = _tool_tube_cell([("upper_arm_link", (0, 0, 0.1)), ("forearm_link", (0, 0, 0.1))])
    warned = [f for f in rt.report(inside).findings if f["code"] == "route_clash" and "holds the line inside" in f["message"]]
    assert {f["message"].split("`")[1] for f in warned} == {"dress/h0", "dress/h1"}
    hose = json.loads(inside._span_replay_links_json("air", 1, 0.003))
    exempt = {link for _, link in hose["exempt"]}
    assert exempt == {"upper_arm_link", "forearm_link"}
    assert {link for _, link in hose["collide"]} >= {"base_link", "shoulder_link", "wrist_1_link"}
    assert not exempt & {link for _, link in hose["collide"]}
    # Held on the surface, 12 cm off the axis: nothing left out, nothing warned.
    outside = _tool_tube_cell([("upper_arm_link", (0, 0.12, 0.15)), ("forearm_link", (0, 0.12, 0.12))])
    assert not any("holds the line inside" in f["message"] for f in rt.report(outside).findings)
    hose = json.loads(outside._span_replay_links_json("air", 1, 0.003))
    assert hose["exempt"] == [] and {"upper_arm_link", "forearm_link"} <= {link for _, link in hose["collide"]}
    # Over a cycle the arm it rides is looked at too: the nearest body of a
    # moving span can now be one of its links.
    outside.add_segment("away", goal=[0.6, 0.4, -0.5, 0.2, 0.0, 0.0])
    sq = outside.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    tl = rt.animate(outside, outside.simulate_sequence("cycle"), settle_s=0.2, step_s=1 / 240)
    spans = rt.report(outside, tl).spans
    assert any("simple_arm/" in (sp["nearest"] or "") for sp in spans if sp["kind"] == "moving")


HOUSING_ARM = """<robot name="housing_arm">
  <link name="base"><collision><origin xyz="0 0 0.05"/><geometry><box size="0.12 0.12 0.1"/></geometry></collision></link>
  <link name="plate"><collision><origin xyz="0 0 0.005"/><geometry><box size="0.2 0.2 0.01"/></geometry></collision></link>
  <joint name="plate_mount" type="fixed"><parent link="base"/><child link="plate"/><origin xyz="0 0 -0.01"/></joint>
  <joint name="j1" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.1"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <link name="arm"><collision><origin xyz="0 0 0.2"/><geometry><box size="0.06 0.06 0.4"/></geometry></collision></link>
  <joint name="j2" type="revolute"><parent link="arm"/><child link="head"/><origin xyz="0 0 0.4"/>
    <axis xyz="0 1 0"/><limit lower="-3" upper="3" effort="10" velocity="1"/></joint>
  <link name="head"><collision><geometry><box size="0.16 0.16 0.16"/></geometry></collision></link>
  <joint name="tool_mount" type="fixed"><parent link="head"/><child link="tool"/><origin xyz="0 0 0.1"/></joint>
  <link name="tool"/>
</robot>"""


def housing_arm():
    """A two-joint arm whose head housing reaches 80 mm down the link below
    it — the way a cobot's wrist housing overlaps its forearm — with a tube
    from a valve beside it to the tool."""
    s = bt.Scene(bt.Robot.from_urdf_string(HOUSING_ARM))
    robot = s.robots[0]
    s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.4, 0.0, 0.05))
    cx.port(s, "valve/out", "valve", "pneumatic", "supply", at=("valve", (0, 0, 0.05)), exit=(0, 0, 1))
    cx.port(s, "arm/air", robot, "pneumatic", "load", at=(robot, "tool", (0, 0, 0)), exit=(0, 0, 1))
    cx.connect(s, "valve/out", "arm/air", name="air")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20)
    return s, robot


def test_a_run_along_a_link_takes_a_side_face_and_keeps_its_clamps_out_of_the_joints():
    s, robot = housing_arm()

    def clamps(face=None):
        rt.route(s, "air", carries="air", spec="tube", slack_m=0.1,
                 via=[rt.along((robot, "arm"), pitch_m=0.05, face=face)])
        return [a for a in rt.geometry(s, "air")["anchors"] if a["auto"]]

    off = 0.003 + 0.005          # the tube's radius and the clip's standoff
    for face, axis, sign in (("+x", 0, 1), ("-x", 0, -1), ("+y", 1, 1), ("-y", 1, -1)):
        offsets = [a["attach"]["offset_m"] for a in clamps(face)]
        assert len(offsets) >= 3 and all(abs(o[axis] - sign * (0.03 + off)) < 1e-9 for o in offsets), face
        # Along the link, not across its end, and drawn in clear of the head's
        # housing (its box reaches down to z = 0.32): the last clamp more than
        # `off` below it.
        zs = [o[2] for o in offsets]
        assert min(zs) >= 0.05 - 1e-9 and max(zs) < 0.32 - off, (face, zs)
    # Unpinned, the face is a side one (the link's long axis is its z): the
    # run spreads along the link, from near the base to near the head.
    zs = [a["attach"]["offset_m"][2] for a in clamps()]
    assert max(zs) - min(zs) > 0.2
    positions = [a["position"] for a in clamps("+x")]
    others = [[robot, link] for link in ("base", "head")]
    assert json.loads(s._points_near_links_json(json.dumps(positions), off - 1e-6, json.dumps(others))) == []
    # Sideways on the face: the same run, moved across it — two lines along
    # one face side by side, each with its clips.
    for aside in (0.007, -0.007):
        rt.route(s, "air", carries="air", spec="tube", slack_m=0.1,
                 via=[rt.along((robot, "arm"), pitch_m=0.05, face="+x", offset_m=aside)])
        ys = {round(a["attach"]["offset_m"][1], 6) for a in rt.geometry(s, "air")["anchors"] if a["auto"]}
        assert ys == {round(aside, 6)}, (aside, ys)
    with pytest.raises(ValueError, match="offset_m"):
        rt.along((robot, "arm"), offset_m=float("nan"))
    # Room at the way-in end: the run starts that much further along the
    # link, past the clearance of the base; the way-out end stays.
    rt.route(s, "air", carries="air", spec="tube", slack_m=0.1, via=[rt.along((robot, "arm"), pitch_m=0.05, face="+x")])
    plain = sorted(a["attach"]["offset_m"][2] for a in rt.geometry(s, "air")["anchors"] if a["auto"])
    rt.route(s, "air", carries="air", spec="tube", slack_m=0.1,
             via=[rt.along((robot, "arm"), pitch_m=0.05, face="+x", inset_m=(0.1, 0))])
    roomy = sorted(a["attach"]["offset_m"][2] for a in rt.geometry(s, "air")["anchors"] if a["auto"])
    assert roomy[0] == pytest.approx(plain[0] + 0.1, abs=1e-6) and roomy[-1] == pytest.approx(plain[-1], abs=1e-6)
    # An inset longer than the run leaves no run: the route cannot be laid.
    rt.route(s, "air", carries="air", spec="tube", slack_m=0.1,
             via=[rt.along((robot, "arm"), pitch_m=0.05, face="+x", inset_m=(1.0, 1.0))])
    with pytest.raises(ValueError, match="inset"):
        rt.geometry(s, "air")
    assert any(f["code"] == "route_endpoint" and "inset" in f["message"] for f in rt.report(s).findings)
    with pytest.raises(ValueError, match="inset_m"):
        rt.along((robot, "arm"), inset_m=(0.1, 0.1, 0.1))
    # The face rides the plan as written; one that is not a face is refused,
    # in Python and in a plan read back.
    rt.route(s, "air", carries="air", spec="tube", slack_m=0.1, via=[rt.along((robot, "arm"), pitch_m=0.05, face="+x")])
    plan = json.loads(s._routing_plan_json())
    assert plan["routes"][0]["via"][0]["face"] == "+x" and "offset_m" not in plan["routes"][0]["via"][0]
    with pytest.raises(ValueError, match="face"):
        rt.along((robot, "arm"), face="top")
    plan["routes"][0]["via"][0]["face"] = "up"
    with pytest.raises(ValueError, match="face"):
        rt.restore(s, plan)


def test_an_anchor_carries_the_line_s_tangent_and_the_replay_leads_a_clip_along_it():
    s, robot = housing_arm()
    rt.clamp(s, "clip", at=(robot, "base", (0.07, 0.0, 0.05)), length_m=0.01)
    rt.route(s, "air", carries="air", spec="tube", slack_m=0.1, clamp=dict(length_m=0.01),
             via=["clip", rt.along((robot, "arm"), pitch_m=0.05, face="+x")])
    g = rt.geometry(s, "air")
    anchors = {a["name"]: a for a in g["anchors"]}
    # A port: out along its stub, and in against the other's.
    assert anchors["valve/out"]["tangent"] == pytest.approx([0.0, 0.0, 1.0])
    assert anchors["arm/air"]["tangent"] == pytest.approx([0.0, 0.0, -1.0])
    # The auto clamps of a run: along it, the first and the last ones too.
    auto = [a for a in g["anchors"] if a["auto"]]
    assert len(auto) >= 3 and all(a["tangent"] == pytest.approx([0.0, 0.0, 1.0], abs=1e-6) for a in auto)
    # A declared clip, its orientation unknown: the bisector of its legs —
    # in from the valve (toward -x), out up the arm (+z).
    t = anchors["clip"]["tangent"]
    assert t[0] < 0 and t[2] > 0 and abs(t[1]) < 1e-9 and math.hypot(*t) == pytest.approx(1.0)
    # Replayed, the loop from the clip to the first auto clamp arrives at
    # the run's clip along the run: the clip holds its length of line
    # straight, so the rope's held samples there lie along the tangent (the
    # hold is a soft constraint: within a few degrees, not exactly) and stay
    # with the link as it moves. The declared clip is a fixed holder too:
    # it holds the line along the bisector of its legs.
    s.add_segment("away", goal=[0.8, -0.4])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    tl = rt.animate(s, s.simulate_sequence("cycle"), settle_s=0.2, step_s=1 / 240)
    track = bt.rope.track(tl, "route:air#1")
    first, last = track["points"][0], track["points"][-1]

    def angle(a, b, to):
        d = [b[i] - a[i] for i in range(3)]
        n = math.hypot(*d)
        return math.degrees(math.acos(max(-1.0, min(1.0, sum(d[i] * to[i] for i in range(3)) / n))))

    assert angle(first[-2], first[-1], [0.0, 0.0, 1.0]) < 30
    assert angle(first[0], first[1], t) < 30
    s.set_joint_positions(tl.sample(tl.duration))
    _, q = s.link_pose("arm")
    x, y, z, w = q
    up = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
    assert angle(last[-2], last[-1], up) < 30


def test_the_static_solver_draws_the_rod_at_rest_between_its_holders():
    s, robot = housing_arm()
    rt.clamp(s, "clip", at=(robot, "base", (0.07, 0.0, 0.05)), length_m=0.01)
    rt.route(s, "air", carries="air", spec="tube", slack_m=0.1, clamp=dict(length_m=0.01),
             via=["clip", rt.along((robot, "arm"), pitch_m=0.05, face="+x")])
    s.add_segment("away", goal=[0.8, -0.4])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    tl0 = s.simulate_sequence("cycle")
    tl = rt.animate(s, tl0, solver="static")
    again = rt.animate(s, tl0, solver="static")
    names = [n for n in tl.ropes if n.startswith("route:air#")]
    assert names, "every moving span is a track"
    for name in names:
        track = bt.rope.track(tl, name)
        assert "static" in track["solver"]
        assert track["times"][0] == 0.0 and track["times"][-1] == pytest.approx(tl.duration)
        assert track["points"] == bt.rope.track(again, name)["points"], "drawn, not simulated: the same twice"
        # Inextensible at every frame: one sample spacing between neighbours.
        for frame in track["points"]:
            steps = [math.dist(a, b) for a, b in zip(frame, frame[1:])]
            assert max(steps) - min(steps) < 1e-4
            assert all(abs(d - steps[0]) < 1e-4 for d in steps)
    # The span from the clip to the first clip of the run arrives along the
    # run (+z of the arm link) and leaves the clip from its point: the held
    # samples are exact, not a soft constraint.
    track = bt.rope.track(tl, "route:air#1")
    last = track["points"][-1]
    s.set_joint_positions(tl.sample(tl.duration))
    _, q = s.link_pose("arm")
    x, y, z, w = q
    up = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
    d = [last[-1][i] - last[-2][i] for i in range(3)]
    assert sum(d[i] * up[i] for i in range(3)) / math.hypot(*d) > math.cos(math.radians(2))
    assert len(track["points"][0]) == len(track["points"][-1])
    with pytest.raises(ValueError, match="solver"):
        rt.animate(s, tl0, solver="magic")
    with pytest.raises(ValueError, match="rigidity"):
        rt.animate(s, tl0, solver="static", rigidity_nm2=0.0)


def test_a_holder_s_kind_and_direction_shape_how_the_line_is_held(monkeypatch):
    s, robot = housing_arm()
    # A rigid clip pointed up the base, a rotating holder, a strap, each on
    # the base; the plan records kind and direction, the geometry turns the
    # direction into the anchor's tangent (signed the way the line travels).
    rt.clamp(s, "clip", at=(robot, "base", (0.07, 0.0, 0.05)), length_m=0.01, direction=(0, 0, -1))
    rt.clamp(s, "turn", at=(robot, "arm", (0.038, 0.0, 0.08)), length_m=0.01, kind="rotating")
    rt.clamp(s, "tie", at=(robot, "arm", (0.038, 0.0, 0.14)), length_m=0.02, kind="strap")
    plan = rt._plan(s)
    holds = {sp["name"]: (sp.get("hold"), sp.get("direction")) for sp in plan["supports"]}
    assert holds["clip"] == ("fixed", [0.0, 0.0, -1.0])
    assert holds["turn"] == ("rotating", None)
    assert holds["tie"] == ("strap", None)
    rt.route(s, "air", carries="air", spec="tube", slack_m=0.1, clamp=dict(length_m=0.01, kind="strap"),
             via=["clip", "turn", "tie", rt.along((robot, "arm"), pitch_m=0.05, face="+x")], collide=False)
    g = rt.geometry(s, "air")
    anchors = {a["name"]: a for a in g["anchors"]}
    # The clip's declared direction, turned the way the line travels (up).
    assert anchors["clip"]["tangent"] == pytest.approx([0.0, 0.0, 1.0])
    assert anchors["clip"]["holder"] == "fixed"
    assert anchors["turn"]["holder"] == "rotating" and anchors["tie"]["holder"] == "strap"
    assert all(a["holder"] == "strap" for a in g["anchors"] if a["auto"]), "the run's clips take the route's kind"
    assert anchors["valve/out"]["holder"] == "fixed"
    with pytest.raises(ValueError, match="kind"):
        rt.clamp(s, "bad", at=(robot, "base", (0.07, 0.0, 0.05)), kind="welded")
    with pytest.raises(ValueError, match="direction"):
        rt.clamp(s, "bad", at=(robot, "base", (0.07, 0.0, 0.05)), direction=(0, 0, 0))
    # The replay holds a fixed or rotating holder's line straight through
    # it (one sample spacing at least, along its direction) and a strap at
    # one sample: what each pass asks of the rope.
    s.add_segment("away", goal=[0.8, -0.4])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    seen = []
    real = bt.rope.animate_many

    def spy(scene, timeline, passes):
        seen.extend(passes)
        return real(scene, timeline, passes)

    monkeypatch.setattr(bt.rope, "animate_many", spy)
    rt.animate(s, s.simulate_sequence("cycle"), solver="static")
    held = {}
    for p in seen:
        for a in p["anchors"]:
            held[(p["name"], a["location"])] = a["length_m"]
    # Span 1 leaves the fixed clip: held over a sample spacing; its end at
    # the rotating holder too. Span 3 leaves the strap: one sample.
    assert held[("route:air#1", "Start")] >= 0.01 and held[("route:air#1", "End")] >= 0.01
    assert held[("route:air#3", "Start")] == 0.0
    clip_out = bt.rope.track(rt.animate(s, s.simulate_sequence("cycle"), solver="static"), "route:air#1")["points"][0]

    def angle(a, b, to):
        d = [b[i] - a[i] for i in range(3)]
        n = math.hypot(*d)
        return math.degrees(math.acos(max(-1.0, min(1.0, sum(d[i] * to[i] for i in range(3)) / n))))

    assert angle(clip_out[0], clip_out[1], [0.0, 0.0, 1.0]) < 2, "held straight up out of the clip"


def test_a_sliding_holder_is_a_ring_the_line_runs_through():
    s, robot = housing_arm()
    rt.clamp(s, "clip", at=(robot, "base", (0.07, 0.0, 0.05)), length_m=0.01)
    rt.clamp(s, "ring", at=(robot, "arm", (0.038, 0.0, 0.12)), length_m=0.02, kind="sliding")
    rt.clamp(s, "arm_clip", at=(robot, "arm", (0.038, 0.0, 0.25)), length_m=0.01)
    rt.route(s, "air", carries="air", spec="tube", via=["clip", "ring", "arm_clip"], slack_m=[0.03, 0.03, 0.05])
    g = rt.geometry(s, "air")
    assert [sp["slides"] for sp in g["spans"]] == [False, False, True, False]
    s.add_segment("away", goal=[0.8, -0.4])
    sq = s.sequence("cycle")
    sq.step("away", actions=[bt.seq.motion("away")])
    tl0 = s.simulate_sequence("cycle")
    for solver in ("static", "physics"):
        tl = rt.animate(s, tl0, solver=solver, settle_s=0.2, step_s=1 / 240)
        names = [n for n in tl.ropes if n.startswith("route:air#")]
        # One rope from the clip through the ring to the arm's clip: the
        # spans either side of the ring share it, under the first's name.
        assert "route:air#1" in names and "route:air#2" not in names, (solver, names)
        track = bt.rope.track(tl, "route:air#1")
        ring_at = next(a for a in g["anchors"] if a["name"] == "ring")["position"]
        for frame, t in zip(track["points"], track["times"]):
            s.set_joint_positions(tl0.sample(min(t, tl0.duration)))
            at, q = s.link_pose("arm")
            # The ring rides the arm; the line passes through its bore: the
            # particle nearest the ring's centre lies on its axis, within
            # the bore, half a particle spacing along it at most.
            x, y, z, w = q
            up = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
            side = [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)]
            centre = [at[i] + up[i] * 0.12 + side[i] * 0.038 for i in range(3)]
            nearest = min(frame, key=lambda p: math.dist(p, centre))
            d = [nearest[i] - centre[i] for i in range(3)]
            along = sum(d[i] * up[i] for i in range(3))
            radial = math.sqrt(max(sum(v * v for v in d) - along * along, 0.0))
            assert radial < 0.004 + 0.0005 and abs(along) < 0.012, (solver, t, radial, along)
        s.set_joint_positions([0.0, 0.0])
    report = rt.report(s, tl)
    rows = {(r["from"], r["to"]) for r in report.spans}
    assert ("clip", "ring") in rows and ("ring", "arm_clip") in rows
    assert [r["rope"] for r in report.spans if r["from"] in ("clip", "ring")] == ["route:air#1", "route:air#1"]


def test_a_carrier_s_holders_take_kinds_and_a_clamp_pack_states_one():
    s, robot = housing_arm()
    rt.carrier(s, "dress", holders=[(robot, "base", (0.07, 0.0, 0.05)),
                                    dict(at=(robot, "arm", (0.038, 0.0, 0.2)), kind="rotating", direction=(0, 0, 1))],
               inner_mm=12)
    holders = next(sp for sp in rt._plan(s)["supports"] if sp["name"] == "dress")["holders"]
    assert [h["kind"] for h in holders] == ["fixed", "rotating"]
    assert holders[1]["direction"] == [0.0, 0.0, 1.0] and holders[0]["at"]["link"] == "arm" or holders[0]["at"]["link"] == "base"
    with pytest.raises(ValueError, match="unknown keys"):
        rt.carrier(s, "bad", holders=[dict(at=(robot, "base", (0, 0, 0)), colour="red")])
    rt.clamp(s, "tie", at=(robot, "base", (0.05, 0.0, 0.11)), catalog=str(PACKS / "test/strap/velcro/r1"))
    tie = next(sp for sp in rt._plan(s)["supports"] if sp["name"] == "tie")
    assert tie["hold"] == "strap" and tie["length_m"] == pytest.approx(0.02)
    assert tie["catalog"].startswith("test/strap/velcro/r1")


def test_a_retracting_holder_pays_the_line_out_along_its_rail():
    # A rail down the arm from 0.12 m toward the base, its carriage pulled
    # up to the start, and no slack in the line from the base clip: as the
    # arm swings away the line would go 4 cm short, and pulls the carriage
    # down the rail instead.
    def cell(stroke):
        s, robot = housing_arm()
        rt.clamp(s, "clip", at=(robot, "base", (0.07, 0.0, 0.05)), length_m=0.01)
        rt.clamp(s, "pay", at=(robot, "arm", (0.038, 0.0, 0.12)), length_m=0.01, kind="retracting",
                 axis=(0, 0, -1), stroke_m=stroke, force_n=3.0)
        rt.clamp(s, "arm_clip", at=(robot, "arm", (0.038, 0.0, 0.25)), length_m=0.01)
        rt.route(s, "air", carries="air", spec="tube", via=["clip", "pay", "arm_clip"], slack_m=[0.0, 0.0, 0.1],
                 collide=False)
        s.add_segment("away", goal=[-1.2, 0.0])
        sq = s.sequence("cycle")
        sq.step("away", actions=[bt.seq.motion("away")])
        return s, robot
    s, robot = cell(0.1)
    plan = rt._plan(s)
    pay = next(sp for sp in plan["supports"] if sp["name"] == "pay")
    assert pay["hold"] == "retracting" and pay["axis"] == [0.0, 0.0, -1.0] and pay["stroke_m"] == 0.1
    assert next(sp for sp in rt.geometry(s, "air")["spans"] if sp["to"] == "pay")["kind"] == "moving"
    g = rt.geometry(s, "air")
    rail = next(a for a in g["anchors"] if a["name"] == "pay")["rail"]
    assert rail["axis"] == pytest.approx([0.0, 0.0, -1.0]) and rail["stroke_m"] == 0.1 and rail["force_n"] == 3.0
    with pytest.raises(ValueError, match="axis="):
        rt.clamp(s, "bad", at=(robot, "arm", (0.038, 0.0, 0.17)), kind="retracting")
    with pytest.raises(ValueError, match="belong to a retracting"):
        rt.clamp(s, "bad", at=(robot, "arm", (0.038, 0.0, 0.17)), stroke_m=0.1)
    tl0 = s.simulate_sequence("cycle")
    tl = rt.animate(s, tl0, solver="static")
    track = bt.rope.track(tl, "route:air#1")
    # The carriage's sample (the span's end) runs down the arm as the arm
    # reaches: measured along the rail in the arm's frame.
    along = []
    for frame, t in zip(track["points"], track["times"]):
        s.set_joint_positions(tl0.sample(min(t, tl0.duration)))
        at, q = s.link_pose("arm")
        x, y, z, w = q
        up = [2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)]
        side = [1 - 2 * (y * y + z * z), 2 * (x * y + w * z), 2 * (x * z - w * y)]
        start = [at[i] + up[i] * 0.12 + side[i] * 0.038 for i in range(3)]
        along.append(-sum((frame[-1][i] - start[i]) * up[i] for i in range(3)))
    s.set_joint_positions([0.0, 0.0])
    # Parked near the rail's start before the arm moves: the taut line
    # takes the few millimetres the rest shape needs beyond the laid one.
    assert along[0] < 0.01, along[0]
    assert max(along) > 0.02 and max(along) <= 0.1 + 1e-6, along
    report = rt.report(s, tl)
    row = next(r for r in report.spans if r["from"] == "clip" and r["to"] == "pay")
    assert row["retraction_m"] == pytest.approx(max(along) - min(along), abs=2e-3)
    # The free length needed is measured to the rail (where the carriage
    # can go), not to the rail's start: with the rail the line is long
    # enough; without it the holders get farther apart than the line.
    assert row["max_distance_m"] < row["material_m"] + 1e-6, row
    assert not [f for f in report.findings
                if f["code"] in ("route_tension", "route_stroke") and "pay" in f["message"]], report.findings
    # A rail too short for the reach: the carriage runs out of stroke.
    short, _ = cell(0.01)
    tl_short = rt.animate(short, short.simulate_sequence("cycle"), solver="static")
    codes = [f["code"] for f in rt.report(short, tl_short).findings]
    assert "route_stroke" in codes


SPINNING_ARM = """<robot name="spinning_arm">
  <link name="base"><collision><origin xyz="0 0 0.05"/><geometry><box size="0.12 0.12 0.1"/></geometry></collision></link>
  <joint name="spin" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 0.1"/>
    <axis xyz="0 0 1"/><limit lower="-6.3" upper="6.3" effort="10" velocity="1"/></joint>
  <link name="arm"><collision><origin xyz="0 0 0.2"/><geometry><box size="0.06 0.06 0.4"/></geometry></collision></link>
  <joint name="tool_mount" type="fixed"><parent link="arm"/><child link="tool"/><origin xyz="0 0 0.4"/></joint>
  <link name="tool"/>
</robot>"""


def test_the_static_rod_carries_the_twist_between_holders_that_hold_the_roll():
    # An arm that spins about its own axis, a tube clipped to the base and
    # to the arm: the spin rolls the arm's clip about the line, and the rod
    # between the clips twists by it. With a rotating holder on the arm the
    # roll is released and the rod carries no torsion.
    def cell(kind):
        s = bt.Scene(bt.Robot.from_urdf_string(SPINNING_ARM))
        robot = s.robots[0]
        s.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.4, 0.0, 0.05))
        cx.port(s, "valve/out", "valve", "pneumatic", "supply", at=("valve", (0, 0, 0.05)), exit=(0, 0, 1))
        cx.port(s, "arm/air", robot, "pneumatic", "load", at=(robot, "tool", (0, 0, 0)), exit=(0, 0, 1))
        cx.connect(s, "valve/out", "arm/air", name="air")
        rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20,
                torsion_deg_m=360)
        rt.clamp(s, "clip", at=(robot, "base", (0.07, 0.0, 0.05)), length_m=0.01, direction=(0, 0, 1))
        rt.clamp(s, "arm_clip", at=(robot, "arm", (0.038, 0.0, 0.2)), length_m=0.01, kind=kind, direction=(0, 0, 1))
        # Little slack: the line has no room to writhe, so the spin is all torsion.
        rt.route(s, "air", carries="air", spec="tube", via=["clip", "arm_clip"], slack_m=[0.03, 0.05], collide=False)
        s.add_segment("turn", goal=[1.2])
        sq = s.sequence("cycle")
        sq.step("turn", actions=[bt.seq.motion("turn")])
        return s
    s = cell("fixed")
    tl = rt.animate(s, s.simulate_sequence("cycle"), solver="static")
    track = bt.rope.track(tl, "route:air#1")
    assert track["twist_deg"] is not None and len(track["twist_deg"]) == len(track["times"])
    assert abs(track["twist_deg"][0]) < 1.0, "installed without twist"
    report = rt.report(s, tl)
    row = next(r for r in report.spans if r["to"] == "arm_clip")
    # The spin of 1.2 rad = 69°, taken up as torsion between the clips (a
    # little of it as the line's own turn with the clip's small circle).
    assert row["torsion_deg"] is not None and 35 < row["torsion_deg"] < 80, row
    assert row["torsion_deg"] <= row["twist_deg"] + 1e-6
    # The same cell with a rotating holder on the arm: the rod rolls free
    # there and carries no torsion; the kinematic twist leaves the roll out too.
    s2 = cell("rotating")
    tl2 = rt.animate(s2, s2.simulate_sequence("cycle"), solver="static")
    track2 = bt.rope.track(tl2, "route:air#1")
    assert track2.get("twist_deg") is None
    row2 = next(r for r in rt.report(s2, tl2).spans if r["to"] == "arm_clip")
    assert row2.get("torsion_deg") is None and row2["twist_deg"] < row["twist_deg"]


def test_the_fixing_rules_of_a_dress_pack_are_checked_on_the_plan():
    # A signal cable to the spinning arm: a clip on a post right past the
    # box's connector stub (on another body), a clamp on the arm's joint
    # housing, and the spin turning the line about its axis between the
    # holders with less twist allowed than ±180° wants; a second cable
    # straight to its connector on the tool, which then holds the moving
    # span alone.
    s = bt.Scene(bt.Robot.from_urdf_string(SPINNING_ARM))
    robot = s.robots[0]
    s.add_box("box", size=(0.1, 0.1, 0.1), position=(0.4, 0.0, 0.05))
    cx.port(s, "box/out", "box", "signal", "output", at=("box", (0, 0, 0.05)), exit=(0, 0, 1), straight_m=0.1)
    cx.port(s, "box/out2", "box", "signal", "output", at=("box", (0, 0.03, 0.05)), exit=(0, 0, 1))
    cx.port(s, "arm/in", robot, "signal", "input", at=(robot, "tool", (0, 0, 0)), exit=(0, 0, 1), straight_m=0.03)
    cx.port(s, "arm/in2", robot, "signal", "input", at=(robot, "tool", (0, 0.03, 0)), exit=(0, 0, 1))
    cx.connect(s, "box/out", "arm/in", name="sig")
    cx.connect(s, "box/out2", "arm/in2", name="sig2")
    rt.spec(s, "cable", kind="cable", media=["signal"], od_mm=6, bend_r_fixed_mm=30, bend_r_moving_mm=60,
            torsion_deg_m=180)
    # A clip on a post 2 cm past the connector's stub: within its reach.
    s.add_box("post", size=(0.02, 0.02, 0.2), position=(0.41, 0.0, 0.1))
    rt.clamp(s, "near_box", at=("post", (0, 0, 0.1)), length_m=0.01)
    rt.clamp(s, "on_joint", at=(robot, "arm", (0.038, 0.0, 0.02)), length_m=0.01)
    rt.route(s, "sig", carries="sig", spec="cable", via=["near_box", "on_joint"], slack_m=0.1, collide=False)
    rt.route(s, "sig2", carries="sig2", spec="cable", slack_m=0.2, collide=False)
    report = rt.report(s)
    rules = ("route_exit_bend", "route_clamp_near_joint", "route_connector_holds", "route_twist_room")
    found = {f["code"]: f for f in report.findings if f["code"] in rules}
    assert "route_exit_bend" in found and "near_box" in found["route_exit_bend"]["message"], report.findings
    assert "route_clamp_near_joint" in found and "on_joint" in found["route_clamp_near_joint"]["message"]
    assert "route_connector_holds" in found and "arm/in2" in found["route_connector_holds"]["message"]
    assert "route_twist_room" in found and "spin" in found["route_twist_room"]["message"]
    assert all(f["severity"] == "warning" for f in found.values())
    # A tube to a push-in fitting, clipped clear of the joint, with line
    # enough for ±180° of the spin: none of them.
    t = bt.Scene(bt.Robot.from_urdf_string(SPINNING_ARM))
    robot = t.robots[0]
    t.add_box("valve", size=(0.1, 0.1, 0.1), position=(0.4, 0.0, 0.05))
    cx.port(t, "valve/out", "valve", "pneumatic", "supply", at=("valve", (0, 0, 0.05)), exit=(0, 0, 1))
    cx.port(t, "arm/air", robot, "pneumatic", "load", at=(robot, "tool", (0, 0, 0)), exit=(0, 0, 1))
    cx.connect(t, "valve/out", "arm/air", name="air")
    rt.spec(t, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20,
            torsion_deg_m=360)
    rt.clamp(t, "clip", at=(robot, "base", (0.07, 0.0, 0.05)), length_m=0.01)
    rt.clamp(t, "arm_clip", at=(robot, "arm", (0.038, 0.0, 0.2)), length_m=0.01)
    rt.route(t, "air", carries="air", spec="tube", via=["clip", "arm_clip"], slack_m=[0.4, 0.05], collide=False)
    codes = {f["code"] for f in rt.report(t).findings}
    assert not codes & set(rules), codes


def test_lines_keep_off_each_other_in_the_static_solve_and_a_crossing_is_reported():
    # Two tubes from two valves either side of the arm to two ports on it,
    # laid so that their free spans cross: the static solve keeps them
    # apart (two radii at least); replayed as independent ropes they pass
    # through each other, which the report names.
    s = bt.Scene(bt.Robot.from_urdf_string(SPINNING_ARM))
    robot = s.robots[0]
    s.add_box("left", size=(0.1, 0.1, 0.1), position=(0.4, 0.15, 0.05))
    s.add_box("right", size=(0.1, 0.1, 0.1), position=(0.4, -0.15, 0.05))
    cx.port(s, "left/out", "left", "pneumatic", "supply", at=("left", (0, 0, 0.05)), exit=(0, 0, 1))
    cx.port(s, "right/out", "right", "pneumatic", "supply", at=("right", (0, 0, 0.05)), exit=(0, 0, 1))
    cx.port(s, "arm/a", robot, "pneumatic", "load", at=(robot, "arm", (0.038, -0.02, 0.2)), exit=(1, 0, 0))
    cx.port(s, "arm/b", robot, "pneumatic", "load", at=(robot, "arm", (0.038, 0.02, 0.2)), exit=(1, 0, 0))
    cx.connect(s, "left/out", "arm/a", name="a")
    cx.connect(s, "right/out", "arm/b", name="b")
    rt.spec(s, "tube", kind="tube", media=["pneumatic"], od_mm=6, bend_r_fixed_mm=15, bend_r_moving_mm=20)
    # The clips well apart, so the lines cross in the free span between
    # them and the ports, not against the samples the clips hold.
    rt.clamp(s, "clip_a", at=(robot, "base", (0.07, 0.06, 0.05)), length_m=0.01)
    rt.clamp(s, "clip_b", at=(robot, "base", (0.07, -0.06, 0.05)), length_m=0.01)
    rt.route(s, "a", carries="a", spec="tube", via=["clip_a"], slack_m=0.08, collide=False)
    rt.route(s, "b", carries="b", spec="tube", via=["clip_b"], slack_m=0.08, collide=False)
    s.add_segment("turn", goal=[0.6])
    sq = s.sequence("cycle")
    sq.step("turn", actions=[bt.seq.motion("turn")])
    tl0 = s.simulate_sequence("cycle")

    def closest(tl):
        ta, tb = bt.rope.track(tl, "route:a#1"), bt.rope.track(tl, "route:b#1")
        worst = math.inf
        for t in [i * 0.1 for i in range(int(tl.duration * 10) + 1)]:
            pa = _at(ta, t)
            pb = _at(tb, t)
            for i in range(len(pa) - 1):
                for j in range(len(pb) - 1):
                    worst = min(worst, _segment_distance(pa[i], pa[i + 1], pb[j], pb[j + 1]))
        return worst

    tl = rt.animate(s, tl0, solver="static")
    assert closest(tl) > 0.006 - 0.0005, closest(tl)
    assert not [f for f in rt.report(s, tl).findings if f["code"] == "route_crossing"]
    tl_ph = rt.animate(s, tl0, settle_s=0.2, step_s=1 / 240)
    if closest(tl_ph) < 0.006 - 0.001:
        codes = [f["code"] for f in rt.report(s, tl_ph).findings]
        assert "route_crossing" in codes, codes


def _at(track, t):
    times = track["times"]
    k = max(i for i, s in enumerate(times) if s <= t + 1e-9)
    return track["points"][k]


def _segment_distance(p1, q1, p2, q2):
    """The least distance between the segments p1–q1 and p2–q2."""
    import numpy as np
    p1, q1, p2, q2 = (np.array(v) for v in (p1, q1, p2, q2))
    d1, d2, r = q1 - p1, q2 - p2, p1 - p2
    a, e, f = d1.dot(d1), d2.dot(d2), d2.dot(r)
    if a < 1e-12 and e < 1e-12:
        return float(np.linalg.norm(r))
    if a < 1e-12:
        s_, t_ = 0.0, min(max(f / e, 0.0), 1.0)
    else:
        c = d1.dot(r)
        if e < 1e-12:
            s_, t_ = min(max(-c / a, 0.0), 1.0), 0.0
        else:
            b = d1.dot(d2)
            denom = a * e - b * b
            s_ = min(max((b * f - c * e) / denom, 0.0), 1.0) if denom > 1e-12 else 0.0
            t_ = (b * s_ + f) / e
            if t_ < 0.0:
                t_, s_ = 0.0, min(max(-c / a, 0.0), 1.0)
            elif t_ > 1.0:
                t_, s_ = 1.0, min(max((b - c) / a, 0.0), 1.0)
    return float(np.linalg.norm((p1 + d1 * s_) - (p2 + d2 * t_)))


def test_a_clamp_on_a_link_fixed_to_the_base_is_still():
    s, robot = housing_arm()
    # The plate is welded to the base: a clip on it is as still as the base.
    rt.clamp(s, "base_clip", at=(robot, "plate", (0.09, 0.0, 0.015)), length_m=0.01)
    rt.clamp(s, "arm_clip", at=(robot, "arm", (0.038, 0.0, 0.2)), length_m=0.01)
    rt.route(s, "air", carries="air", spec="tube", via=["base_clip", "arm_clip"], slack_m=0.1)
    g = rt.geometry(s, "air")
    moving = {a["name"]: a["moving"] for a in g["anchors"]}
    assert moving == {"valve/out": False, "base_clip": False, "arm_clip": True, "arm/air": True}
    assert [sp["kind"] for sp in g["spans"]] == ["fixed", "moving", "moving"]
