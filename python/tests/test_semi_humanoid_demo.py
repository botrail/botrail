"""The shelf-picking cell of `examples/vehicles/semi_humanoid_demo.py`, on the
primitive semi-humanoid: no catalog, no download. What the cell promises —
one machine takes the low and the top board, the head camera gates each
pick, the torso folds while the wheels roll, the machine is one purchase,
and the requirements carry its working heights."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import botrail as bt
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples" / "vehicles"))

import semi_humanoid_demo as demo

ROBOT = demo.ROBOT


@pytest.fixture(scope="module")
def baked():
    return demo.bake("semi")


def test_one_machine_takes_the_low_and_the_top_board(baked) -> None:
    scene, machine, poses, tl = baked
    assert 40.0 < tl.duration < 70.0
    # Both cartons end on the stand, side by side, where the hands set them.
    top = machine.stand_top + machine.carton[2] / 2
    ahead = -(machine.dock_standoff + machine.stand_inset)
    for tag, y in (("low", machine.span), ("top", -machine.span)):   # facing -x, the right hand is at +y
        position, _ = tl.object_pose(f"carton_{tag}", tl.duration)
        assert position == pytest.approx((ahead, y, top + 0.002), abs=2e-3)
    # The torso made the difference: a lift for each board, chosen by the teaching.
    assert poses["torso_low"] == {"lift_joint": 0.0} and poses["torso_top"] == {"lift_joint": 0.40}
    names = scene.robot_of(ROBOT).joint_names
    lift = names.index("lift_joint")
    assert tl.sample(tl.step_span("pick low").end, robot=ROBOT)[lift] == pytest.approx(0.0, abs=1e-9)
    assert tl.sample(tl.step_span("pick top").end, robot=ROBOT)[lift] == pytest.approx(0.40, abs=1e-9)
    # ...and the same cell bakes the same bits.
    again = demo.bake("semi")[3]
    assert again.duration == tl.duration
    assert again.sample(30.0, robot=ROBOT) == tl.sample(30.0, robot=ROBOT)


def test_the_head_camera_gates_each_pick(baked) -> None:
    _, _, _, tl = baked
    signals = dict(tl.signals)

    def rises(name: str) -> list[float]:
        return [t for t, value in signals[name] if value]

    # Neither carton is in view on arrival: the glance down finds the low one...
    look = tl.step_span("look low")
    assert any(look.start < t <= look.end for t in rises("sees_low"))
    # ...and the top one stands on a board over the machine's eyes until the
    # torso is on its way up.
    up = tl.step_span("torso up")
    seen = [t for t in rises("sees_top") if t > tl.step_span("lift low").end]
    assert seen and up.start < seen[0] <= tl.step_span("look top").end

    # A bay that was not replenished stalls the cycle at the glance, by name.
    scene = demo.build_scene(demo.MACHINES["semi"])
    poses = demo.teach(scene, demo.MACHINES["semi"])
    name = demo.build_cycle(scene, demo.MACHINES["semi"], poses)
    position, _ = scene.obstacle_pose("carton_top")
    scene.set_obstacle_pose("carton_top", (position[0], position[1] + 5.0, position[2]))
    with pytest.raises(ValueError, match="look top"):
        scene.simulate_sequence(name, max_duration=60.0)


def test_the_torso_folds_while_the_wheels_roll(baked) -> None:
    scene, machine, _, tl = baked
    names = scene.robot_of(ROBOT).joint_names
    left, right, lift = (names.index(j) for j in ("left_wheel_joint", "right_wheel_joint", "lift_joint"))
    # To the bay: 2.4 m backwards out of the dock, a quarter turn to the
    # right, 0.35 m up to the bay — r = 0.10 wheels on a 0.50 m track.
    arc = 0.25 * (math.pi / 2) / 0.10
    q = tl.sample(tl.step_span("to bay").end, robot=ROBOT)
    assert q[left] == pytest.approx(-24.0 + arc + 3.5, abs=1e-6)
    assert q[right] == pytest.approx(24.0 + arc - 3.5, abs=1e-6)
    # On the way home the column comes down from the top board's height on
    # the move: one sample has the wheels turning and the lift part way.
    home = tl.step_span("to dock")
    t = home.start + 1.0
    before, mid = tl.sample(t - 0.1, robot=ROBOT), tl.sample(t, robot=ROBOT)
    assert abs(mid[left] - before[left]) > 0.1
    assert 0.05 < mid[lift] < 0.40 and mid[lift] < before[lift]
    assert tl.sample(home.end, robot=ROBOT)[lift] == pytest.approx(machine.travel["lift_joint"], abs=1e-9)
    # Out and back cancel: the way home unwinds the wheels.
    end = tl.sample(tl.duration, robot=ROBOT)
    assert end[left] == pytest.approx(0.0, abs=1e-6) and end[right] == pytest.approx(0.0, abs=1e-6)


def test_the_machine_is_one_purchase_with_working_heights(baked) -> None:
    scene, machine, _, tl = baked
    rows = {tuple(r["names"]): r["category"] for r in scene.bom().rows}
    assert rows[(ROBOT,)] == "robot"
    assert not any(demo.BASE in names or f"{ROBOT}/controller" in names for names in rows)

    line = bt.select.requirements(scene, timeline=tl)[ROBOT]
    assert line.category == "vehicle.mobile_manipulator"          # the aisle to shop in
    asked = {r.key: r for r in line.requirements}
    low = machine.boards[0] + machine.carton[2] / 2 + 0.002
    top = machine.boards[1] + machine.carton[2] / 2 + 0.002 + 0.03   # lifted off the board
    assert asked["vertical_reach_min_mm"].value == pytest.approx(low * 1000, abs=1.0)
    assert asked["vertical_reach_min_mm"].op == "<=" and "pick_low" in asked["vertical_reach_min_mm"].basis
    assert asked["vertical_reach_max_mm"].value == pytest.approx(top * 1000, abs=1.0)
    assert asked["arm_count"].value == 2.0 and asked["arm_count"].basis == "arms taught: left, right"
    assert asked["max_speed_mps"].value == machine.speed
    # Reach is not asked: the machine takes its arms' bases to the work, so
    # how far a hand is from its shoulder is this machine's, not the cell's.
    assert "reach_mm" not in asked and any(note.startswith("reach_mm is not asked") for note in line.notes)
    assert asked["payload_kg"].value == pytest.approx(0.35)
    # Nothing the cell asks is answered short, so the cell's check holds.
    assert scene.check(timeline=tl).ok


def test_a_narrow_aisle_is_refused_by_name() -> None:
    machine = demo.MACHINES["semi"]
    scene = demo.build_scene(machine, aisle=0.95)
    poses = demo.teach(scene, machine)
    name = demo.build_cycle(scene, machine, poses)
    with pytest.raises(ValueError, match=r"collides with `row/"):
        scene.simulate_sequence(name, max_duration=60.0)


def test_a_torso_sweep_through_the_bay_is_caught_before_the_bake() -> None:
    # Ramps are the author's: the engine does not check a parked robot's ramp
    # against the cell, so the demo samples each torso sweep first.
    machine = demo.MACHINES["semi"]
    scene = demo.build_scene(machine)
    poses = demo.teach(scene, machine)
    reach_in = poses["top_grip"]
    dropped = demo.joints(scene, reach_in, {"lift_joint": 0.0})
    hits = demo.ramp_contacts(scene, "bay", reach_in, dropped)
    assert any("bay/board" in pair[1] or "bay/board" in pair[0] for pair in hits), hits
    assert demo.ramp_contacts(scene, "bay", poses["travel"], demo.joints(scene, poses["travel"], {"lift_joint": 0.4})) == []


def test_a_carry_below_the_board_backs_out_over_its_edge_first() -> None:
    # A low board at 0.95 m stands over where this machine carries: straight
    # from over the carton down into the carry would cut the board's front
    # edge, so teaching adds the pose that backs the hand out first...
    scene, machine, poses, tl = demo.bake("semi", boards=(0.95, 1.50))
    assert "low_out" in poses and "low_out" not in demo.bake("semi")[2]
    top = machine.stand_top + machine.carton[2] / 2 + 0.002
    for tag in ("low", "top"):
        assert tl.object_pose(f"carton_{tag}", tl.duration)[0][2] == pytest.approx(top, abs=2e-3)
    # ...and without it the bake says what the line would have hit.
    scene = demo.build_scene(machine)
    poses = demo.teach(scene, machine)
    del poses["low_out"]
    with pytest.raises(ValueError, match=r"lift low.*carton_low x bay/board_low"):
        scene.simulate_sequence(demo.build_cycle(scene, machine, poses), max_duration=180.0)


def test_one_bay_tells_the_machines_apart(baked, capsys) -> None:
    # `--compare` on what runs offline: the primitive machine in front of its
    # own bay, and in front of the G1-D's — whose low board, 0.25 m over the
    # floor, a lift column does not get a level hand down to.
    own = demo.compare(demo.MACHINES["semi"].boards, machines=["semi"])
    assert own == [("semi", "lift_joint 0", "lift_joint 0.4", baked[3].duration, "ok")]
    (key, low, top, cycle, verdict), = demo.compare(demo.MACHINES["g1d"].boards, machines=["semi"])
    assert (key, low, top, cycle) == ("semi", "—", "—", None)
    assert verdict.startswith("no torso posture reaches `low` — the closest is ") and verdict.endswith(" mm short")
    assert 200 < float(verdict.split()[-3]) < 330
    printed = capsys.readouterr().out
    assert "bay: boards at 0.25 m and 1.45 m; aisle 1.40 m" in printed and "verdict" in printed
    # The machines the table is for come from the catalog; the ones whose base
    # does not turn say so, and get the cell laid out for them.
    assert list(demo.MACHINES) == ["g1d", "rby1", "rby1m", "ffw", "galbot", "r1pro", "semi"]
    assert [key for key, m in demo.MACHINES.items() if m.holonomic] == ["rby1m", "ffw", "galbot", "r1pro"]
    assert demo.MACHINES["rby1"].package.endswith("/r2") and demo.MACHINES["semi"].package == ""


def test_the_glance_is_taught_gentlest_first() -> None:
    # A glance is chosen like a torso posture: the first of the machine's
    # candidates that frames the carton (its corners — the sensor asks for
    # overlap) and gets there without sweeping the cell.
    from dataclasses import replace

    machine = demo.MACHINES["semi"]
    given = machine.look["low"]
    away = {"head_pan_joint": 1.2, "head_tilt_joint": -0.5}
    scene = demo.build_scene(machine)
    assert demo.teach(scene, machine)["look_low"] == given
    picky = replace(machine, look={**machine.look, "low": [away, given]})
    assert demo.teach(demo.build_scene(picky), picky)["look_low"] == given
    blind = replace(machine, look={**machine.look, "low": [away]})
    with pytest.raises(RuntimeError, match=r"no glance shows the low carton:\n.*deg out of view"):
        demo.teach(demo.build_scene(blind), blind)


def test_a_base_that_does_not_turn_gets_a_cell_it_can_serve(monkeypatch) -> None:
    # A holonomic machine docks facing whatever it faced when parked: in the
    # turning machines' cell it would arrive side-on to the bay. Its cell puts
    # the stand on the bay's side of the aisle and it crabs between the two.
    # Offline: the primitive machine on omni wheels.
    from dataclasses import replace

    crab = replace(demo.MACHINES["semi"], key="crab", holonomic=True)
    turning = demo.load

    def load(machine):
        robot, wheels = turning(machine)
        return robot, (replace(wheels, drive="omni") if machine.key == "crab" else wheels)

    monkeypatch.setattr(demo, "load", load)
    scene = demo.build_scene(crab)
    (dock, heading) = demo.stations(True)["dock"]
    position, attitude = scene.robot_base_pose_of(ROBOT)
    assert position[:2] == pytest.approx(dock, abs=1e-6)                    # parked up its own spur,
    assert attitude == pytest.approx(demo.yaw_quat(heading), abs=1e-6)      # nose to the racks
    poses = demo.teach(scene, crab)
    tl = scene.simulate_sequence(demo.build_cycle(scene, crab, poses), max_duration=180.0)
    # No pivots: the drive is its three legs' length over its speed.
    legs = 2 * demo.SPUR + demo.CORNER_X
    assert tl.step_span("to bay").end == pytest.approx(legs / crab.speed, abs=0.02)
    # Both cartons end on the stand ahead of the dock — ahead is +y here.
    ahead = dock[1] + crab.dock_standoff + crab.stand_inset
    top = crab.stand_top + crab.carton[2] / 2 + 0.002
    for tag, x in (("low", crab.span), ("top", -crab.span)):                # facing +y, the right hand is at +x
        assert tl.object_pose(f"carton_{tag}", tl.duration)[0] == pytest.approx((x, ahead, top), abs=2e-3)
    # The cell is laid out by what the machine description says: it has to agree with the wheels.
    monkeypatch.setattr(demo, "load", turning)
    with pytest.raises(ValueError, match="the cell is laid out by that"):
        demo.build_scene(crab)


def test_teaching_asks_the_planner_and_leaves_nothing_behind(baked) -> None:
    # Whether a hand gets back into the carry in a straight line is asked of
    # the planner, on a motion authored for the question — and removed: it is
    # not in the motion list, the project's script or the hand-over set.
    scene, machine, poses, _ = baked
    kinds = {name: kind for name, kind in poses.items() if name.endswith("_carry_kind")}
    assert set(kinds) == {"low_right_carry_kind", "top_left_carry_kind",
                          "stand_right_carry_kind", "stand_left_carry_kind"}
    assert set(kinds.values()) == {demo.LINE}              # this arm's poses are one posture, moved
    assert demo.PROBE not in scene.motion_names and f'"{demo.PROBE}"' not in scene.generate_python()
    # Straight after teaching, before the cycle is written, the scene has no
    # motion at all: every question was removed once it was answered. (The
    # answer "no line" — a planned return — comes up on the catalog's 7-axis
    # machines, which the offline tests do not load.)
    fresh = demo.build_scene(machine)
    demo.teach(fresh, machine)
    assert fresh.motion_names == []


def test_the_tutorial_embeds_the_lines_it_talks_about() -> None:
    # The tutorial includes the demo by line range; a line moved in the demo
    # silently moves the tutorial's code. Each range must still start and
    # end where the prose expects.
    root = Path(__file__).resolve().parents[2]
    source = (root / "examples" / "vehicles" / "semi_humanoid_demo.py").read_text().splitlines()
    doc = (root / "docs" / "tutorials" / "semi-humanoid.md").read_text()
    expected = {
        "# A base that turns backs out of the dock": "scene.mount_robot(BASE, robot=ROBOT, wheels=wheels)",
        "def load(machine: Machine):": "return bt.Robot.from_catalog(machine.package), bt.Wheels.from_catalog(machine.package)",
        "def task(kind: str, heading: float, targets: dict, leaves: dict) -> None:": '+ "\\n  ".join(text for _, text in refused))',
        "def line_exists(arm: str, start: list, goal: list) -> bool:": 'return q, "joint"',
        "def ramp_contacts(": "return sorted(hits)",
        'scene.add_camera("head_cam"': "return scene",
        'step("look low"': 'step("look low"',
        "# The fold rides the drive": 'step("to dock"',
        'robot = bt.Robot.from_urdf(ASSETS / "semi_humanoid_test.urdf")': "return robot, wheels",
    }
    ranges = [line.split('"')[1].split(":")[1:] for line in doc.splitlines()
              if line.startswith('--8<-- "examples/vehicles/semi_humanoid_demo.py:')]
    assert len(ranges) == len(expected)
    for (first, last), (start, end) in zip(expected.items(), ranges):
        a, b = source[int(start) - 1].strip(), source[int(end) - 1].strip()
        assert a.startswith(first), f"lines {start}:{end} start with {a!r}"
        assert b.startswith(last), f"lines {start}:{end} end with {b!r}"
