"""`bt.tending` — the machine's side of a tending handshake, authored as
a program of its own (design/design-machine-tending.md §4).

The templates put signals and a sequence in the scene and nothing else;
what these tests pin is the *order* the handshake produces on the chart
(door open before SERVICE REQUEST, clamp after the request, door closed
after the exchange is reported done), the I/O the CNC node grows, and the
manual machine's refusal of a start with the door open."""

from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


def cell(*, door: str = "servo", buttons=("cycle_start", "clamp", "unclamp", "estop")):
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"), base_position=(4.0, 0.0, 0.0))
    # A wide button pitch: the test arm's tool tip is an 80 mm box, and a
    # press must not read on the neighbour (that check is a test of its
    # own, in the demo's).
    vmc = bt.parts.machine_tool(scene, "vmc", door=door, panel="door", buttons=buttons, panel_pitch=0.10)
    return scene, vmc


def test_fanuc_ri2_orders_the_handshake_on_the_chart() -> None:
    scene, vmc = cell()
    hs = bt.tending.fanuc_ri2(scene, vmc, cycle_s=3.0, notice_s=1.0, clamp_s=0.2)
    assert (hs.machine, hs.template, hs.program, hs.door, hs.node) == ("vmc", "fanuc_ri2", "vmc", "vmc/side_door", "vmc/cnc")
    assert hs.signal("door_open") == "vmc/side_door/open"
    with pytest.raises(KeyError, match="no signal for the role"):
        hs.signal("coolant")
    assert "vmc" in scene.sequence_names
    # A robot program that answers the handshake without moving.
    sq = scene.sequence("tend")
    sq.step("wait", transition=bt.seq.signal(hs.signal("service_req")))
    sq.step("ask", actions=[bt.seq.set_signal(hs.signal("clamp_req"))], transition=bt.seq.signal(hs.signal("clamp")))
    sq.step("leave", actions=[bt.seq.set_signal(hs.signal("clamp_req"), False)], transition=bt.seq.elapsed(0.5))
    sq.step("ok", actions=[bt.seq.set_signal(hs.signal("service_ok"))], transition=bt.seq.signal(hs.signal("door_closed")))
    sq.step("home", actions=[bt.seq.set_signal(hs.signal("service_ok"), False)])
    tl = scene.simulate_sequences(["tend", "vmc"])
    notice = tl.signal("vmc/notice")
    assert notice.rising_edges() == pytest.approx([2.0]) and notice.falling_edges() == pytest.approx([3.0])
    door_open = tl.signal("vmc/side_door/open").rising_edges()[0]
    request = tl.signal("vmc/service_req").rising_edges()[0]
    clamp = tl.signal("vmc/clamp").rising_edges()[0]
    closed = tl.signal("vmc/side_door/closed").rising_edges()[-1]
    assert 3.0 < door_open <= request < clamp < tl.step_span("tend/ok").start < closed
    assert tl.signal("vmc/clamp").value_at(3.1) is False           # unclamped before the door opens
    assert tl.signal("vmc/service_req").value_at(tl.duration) is False
    assert tl.signal("vmc/running").value_at(tl.duration) is True   # the next cycle started
    assert [name for name, _, _ in tl.step_spans if name.startswith("vmc/")][:4] == [
        "vmc/machining", "vmc/notice", "vmc/finish", "vmc/open_door"]
    # The I/O the CNC grows: the handshake as wires between two hosts,
    # the door as a command word and an in-position input.
    hosts = {(p.name, p.host) for p in scene.io_points()}
    assert ("vmc/service_req", "vmc/cnc") in hosts and ("vmc/clamp_req", "vmc/cnc") in hosts
    assert ("vmc/side_door", "vmc/cnc") in hosts
    assert scene.plcopen(["tend", "vmc"]).count("<pou ") >= 2


def test_fanuc_ri2_without_an_automatic_door_skips_the_door_steps() -> None:
    scene, vmc = cell(door="manual")
    hs = bt.tending.fanuc_ri2(scene, vmc, cycle_s=1.0, notice_s=0.2, node=False)
    assert hs.door is None and hs.node is None and scene.io_points() == [] or all(p.host != "vmc/cnc" for p in scene.io_points())
    sq = scene.sequence("tend")
    sq.step("wait", transition=bt.seq.signal(hs.signal("service_req")))
    sq.step("ask", actions=[bt.seq.set_signal(hs.signal("clamp_req"))], transition=bt.seq.signal(hs.signal("clamp")))
    sq.step("ok", actions=[bt.seq.set_signal(hs.signal("service_ok"))], transition=bt.seq.elapsed(0.1))
    tl = scene.simulate_sequences(["tend", "vmc"])
    names = [name for name, _, _ in tl.step_spans if name.startswith("vmc/")]
    assert "vmc/open_door" not in names and "vmc/close_door" not in names and "vmc/resume" in names
    with pytest.raises(ValueError, match="cycle_s must be positive"):
        bt.tending.fanuc_ri2(scene, vmc, cycle_s=0.0)


def test_manual_machine_ignores_a_start_with_the_door_open() -> None:
    scene, vmc = cell(door="manual")
    hs = bt.tending.manual(scene, vmc, cycle_s=0.5, clamp_s=0.1, buttons=("unclamp", "clamp", "cycle_start"))
    assert hs.template == "manual" and hs.signal("start_button") == "vmc/panel/cycle_start"
    assert hs.signal("clamp") == "vmc/clamp" and dict(scene.signals)["vmc/clamp"] is True
    with pytest.raises(ValueError, match="has no button 'go'"):
        bt.tending.manual(scene, vmc, buttons=("unclamp", "clamp", "go"))
    # Stand the arm's tool in each button in turn: the presses are what
    # the machine program waits for. The door stays shut, so the start
    # is honoured.
    def press_pose(button: str) -> list:
        (x, y, z), q = scene.frame(f"vmc/panel/{button}/press")
        # +Z of the frame runs into the panel (-X); the tool tip is 40 mm
        # past the TCP.
        ik = scene.set_tcp_target((x + 0.04, y, z), q)
        assert ik.converged, (button, ik)
        return list(scene.joint_positions)

    scene.set_robot_base_pose((1.40, -0.89, 0.85))
    away = [0.0] * 6
    poses = {b: press_pose(b) for b in ("unclamp", "clamp", "cycle_start")}
    scene.set_joint_positions(away)
    for b, pose in poses.items():
        scene.add_segment(f"to_{b}", goal=pose)
    scene.add_segment("away", goal=away)
    sq = scene.sequence("operator")
    sq.step("wait", transition=bt.seq.signal(hs.signal("running"), False))
    for b in ("unclamp", "clamp", "cycle_start"):
        sq.step(f"to_{b}", actions=[bt.seq.motion(f"to_{b}")])
        sq.step(f"hold_{b}", transition=bt.seq.elapsed(0.2))
        sq.step(f"back_{b}", actions=[bt.seq.motion("away")])
    tl = scene.simulate_sequences(["operator", "vmc"])
    order = [tl.signal(f"vmc/panel/{b}").rising_edges()[0] for b in ("unclamp", "clamp", "cycle_start")]
    assert order == sorted(order) and tl.signal("vmc/panel/estop").high_spans() == []
    assert tl.signal("vmc/clamp").falling_edges()[0] >= order[0]
    assert tl.signal("vmc/clamp").rising_edges()[-1] >= order[1]
    assert tl.signal("vmc/running").rising_edges()[-1] >= order[2]
    # The same presses with the door open: the start is ignored, and the
    # bake says where the machine is stuck.
    for name in vmc.door_objects:
        (x, y, z), qq = scene.obstacle_pose(name)
        scene.set_obstacle_pose(name, (x, y + vmc.door_travel, z), qq)
    with pytest.raises(ValueError, match="vmc/wait_start"):
        scene.simulate_sequences(["operator", "vmc"], max_duration=20.0)


def test_haas_autodoor_waits_for_cell_safe_and_closes_on_start() -> None:
    scene, vmc = cell()
    hs = bt.tending.haas_autodoor(scene, vmc, cycle_s=2.0, clamp_s=0.2)
    assert (hs.template, hs.door, hs.node) == ("haas_autodoor", "vmc/side_door", "vmc/cnc")
    # The guards ride on the roles: the front door's switch, the E-stop.
    assert hs.signal("front_door_closed") == "vmc/front_door/closed" and hs.signal("estop") == "vmc/panel/estop"
    # The cell's side: make the cell safe once the part is done, ask for
    # the clamp, report out with a remote cycle start.
    sq = scene.sequence("tend")
    sq.step("wait_done", transition=bt.seq.signal(hs.signal("part_done")))
    sq.step("safe", actions=[bt.seq.set_signal(hs.signal("cell_safe"))], transition=bt.seq.signal(hs.signal("door_open")))
    sq.step("ask", actions=[bt.seq.set_signal(hs.signal("clamp_req"))], transition=bt.seq.signal(hs.signal("clamp")))
    sq.step("out", actions=[bt.seq.set_signal(hs.signal("clamp_req"), False)], transition=bt.seq.elapsed(0.3))
    sq.step("start", actions=[bt.seq.set_signal(hs.signal("start_req"))], transition=bt.seq.signal(hs.signal("door_closed")))
    sq.step("home", actions=[bt.seq.set_signal(hs.signal("start_req"), False),
                             bt.seq.set_signal(hs.signal("cell_safe"), False)])
    tl = scene.simulate_sequences(["tend", "vmc"])
    done = tl.signal("vmc/part_done").rising_edges()[0]
    safe = tl.signal("vmc/cell_safe").rising_edges()[0]
    opened = tl.signal("vmc/side_door/open").rising_edges()[0]
    start = tl.signal("vmc/start_req").rising_edges()[0]
    closed = tl.signal("vmc/side_door/closed").rising_edges()[-1]
    # M80 only once the cell is safe; M81 on the start; the next cycle on
    # the closed confirmation.
    assert done <= safe < opened < start < closed
    assert tl.signal("vmc/running").value_at(tl.duration) is True
    assert tl.signal("vmc/part_done").value_at(tl.duration) is False
    # M80/M81 need a driven door.
    alone, plain = cell(door="manual")
    with pytest.raises(ValueError, match="no driven side door"):
        bt.tending.haas_autodoor(alone, plain)


def test_generic_renames_the_roles_and_runs_the_exchange() -> None:
    scene, vmc = cell(door="air")
    hs = bt.tending.generic(scene, vmc, cycle_s=2.0, clamp_s=0.2,
                            signals={"ready": "vmc/M_FIN", "exchange_done": "vmc/ROBOT_OUT"})
    assert hs.template == "generic" and hs.signal("ready") == "vmc/M_FIN"
    assert "vmc/ready" not in hs.signals.values() and hs.signal("clamp") == "vmc/clamp"
    sq = scene.sequence("tend")
    sq.step("wait", transition=bt.seq.signal("vmc/M_FIN"))
    sq.step("ask", actions=[bt.seq.set_signal(hs.signal("clamp_req"))], transition=bt.seq.signal(hs.signal("clamp")))
    sq.step("out", actions=[bt.seq.set_signal(hs.signal("clamp_req"), False), bt.seq.set_signal("vmc/ROBOT_OUT")],
            transition=bt.seq.signal(hs.signal("door_closed")))
    sq.step("home", actions=[bt.seq.set_signal("vmc/ROBOT_OUT", False)])
    tl = scene.simulate_sequences(["tend", "vmc"])
    ready = tl.signal("vmc/M_FIN").rising_edges()[0]
    assert tl.signal("vmc/side_door/open").rising_edges()[0] <= ready < tl.signal("vmc/clamp").rising_edges()[0]
    assert tl.signal("vmc/side_door/closed").rising_edges()[-1] > tl.step_span("tend/out").start
    assert tl.signal("vmc/running").value_at(tl.duration) is True
    alone, plain = cell()
    with pytest.raises(ValueError, match="no role 'coolant'"):
        bt.tending.generic(alone, plain, signals={"coolant": "vmc/M08"})


def test_the_guards_refuse_a_start_with_the_estop_in_or_the_front_door_open() -> None:
    scene, vmc = cell()
    hs = bt.tending.fanuc_ri2(scene, vmc, cycle_s=2.0, notice_s=0.5, clamp_s=0.2)
    sq = scene.sequence("tend")
    sq.step("wait", transition=bt.seq.signal(hs.signal("service_req")))
    sq.step("ask", actions=[bt.seq.set_signal(hs.signal("clamp_req"))], transition=bt.seq.signal(hs.signal("clamp")))
    sq.step("leave", actions=[bt.seq.set_signal(hs.signal("clamp_req"), False)], transition=bt.seq.elapsed(0.3))
    sq.step("ok", actions=[bt.seq.set_signal(hs.signal("service_ok"))], transition=bt.seq.signal(hs.signal("door_closed")))
    sq.step("home", actions=[bt.seq.set_signal(hs.signal("service_ok"), False)])
    # The faults a FAT sheet lists, each a scenario: the E-stop in, the
    # front door's switch not made (the door open, or the switch broken).
    scene.add_scenario("estop_in", faults=[bt.io.stuck(hs.signal("estop"), True)])
    scene.add_scenario("front_open", faults=[bt.io.stuck(hs.signal("front_door_closed"), False)])
    runs = scene.simulate_scenarios(["tend", "vmc"], max_duration=10.0)
    assert "baseline" in runs and set(runs.errors) == {"estop_in", "front_open"}
    # The E-stop holds the machine at the closed door: no next cycle.
    assert "vmc/close_door" in runs.errors["estop_in"] or "close_door" in runs.errors["estop_in"]
    # With the front door open the side door never opens: the machine
    # waits at `finish`, the arm at its `wait`.
    assert "finish" in runs.errors["front_open"]
    # The interlock table reads the same guards as rows.
    rows = {(r["program"], r["step"]): r for r in scene.interlocks().rows}
    assert rows[("vmc", "open_door")]["condition"] == "(T >= 0.2 s AND vmc/front_door/closed)"
    assert rows[("vmc", "cycle_start")]["condition"] == "(INPOS(vmc/side_door) AND vmc/side_door/closed AND NOT vmc/panel/estop)"


def test_a_lathe_is_worked_through_its_one_door() -> None:
    # The templates take a lathe as they take a machining centre: its door
    # is the front door, so no door-exclusivity guard applies — the start
    # is guarded by that door's closed switch and the E-stop alone.
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"), base_position=(4.0, 0.0, 0.0))
    lathe = bt.parts.lathe(scene, "lathe", buttons=("cycle_start", "clamp", "unclamp", "estop"), panel_pitch=0.10)
    hs = bt.tending.manual(scene, lathe, cycle_s=2.0, clamp_s=0.2)
    assert hs.signal("door_closed") == "lathe/front_door/closed" and hs.signal("estop") == "lathe/panel/estop"
    rows = {r["step"]: r for r in scene.interlocks(["lathe"]).rows}
    assert rows["cycle_start"]["condition"] == (
        "(RISING(lathe/panel/cycle_start) AND lathe/front_door/closed AND NOT lathe/panel/estop)")
    assert hs.mtconnect_items()["DoorState"] == ("lathe/front_door/closed", "lathe/front_door/open")
