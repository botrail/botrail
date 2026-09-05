"""Planning groups: a dual-arm robot's arms, addressed by `group=`.

The fixture is a fixed torso with two 3-DOF arms and a finger each
(`examples/assets/dual_arm_test.urdf`), so nothing here needs the network.
"""

from pathlib import Path

import pytest

import botrail as bt

ASSETS = Path(__file__).resolve().parents[2] / "examples" / "assets"


@pytest.fixture
def dual() -> bt.Robot:
    return bt.Robot.from_urdf(ASSETS / "dual_arm_test.urdf")


@pytest.fixture
def arm() -> bt.Robot:
    return bt.Robot.from_urdf(ASSETS / "simple_arm.urdf")


def test_groups_are_derived_per_arm(dual: bt.Robot, arm: bt.Robot) -> None:
    assert dual.groups == ["left", "right"]
    left = dual.group("left")
    assert left.joints == ["left_shoulder", "left_elbow", "left_wrist", "left_finger"]
    assert left.tip == "left_hand"          # the hand, not a fingertip
    assert left.base == "left_base"
    assert left.derived
    # A single arm is one group of everything, tipped at its TCP.
    assert arm.groups == ["arm"]
    assert arm.group("arm").tip == arm.tcp_link
    assert arm.group("arm").joints == arm.joint_names
    with pytest.raises(ValueError, match="unknown group"):
        dual.group("torso")


def test_ik_and_plan_in_one_group_hold_the_other_arm(dual: bt.Robot) -> None:
    scene = bt.Scene(dual)
    names = dual.joint_names
    q0 = scene.joint_positions
    # A reachable left-hand target: FK of a bent left arm.
    bent = list(q0)
    bent[names.index("left_shoulder")] = 1.0
    bent[names.index("left_elbow")] = -0.8
    scene.set_joint_positions(bent)
    p, quat = scene.link_pose("left_hand")
    scene.set_joint_positions(q0)

    res = scene.set_tcp_target(p, quat, group="left")
    assert res.converged
    moved = [n for n, a, b in zip(names, scene.joint_positions, q0) if abs(a - b) > 1e-9]
    assert moved and all(n.startswith("left_") for n in moved)

    scene.set_joint_positions(q0)
    # A post in the straight line makes the planner sample.
    scene.add_box("post", (0.05, 0.05, 0.10), (-0.17, 0.25, 0.10))
    traj = scene.plan_to_pose(p, quat, group="left", seed=1)
    right = [i for i, n in enumerate(names) if n.startswith("right_")]
    for q in traj.positions:
        for i in right:
            assert q[i] == q0[i], f"{names[i]} moved during a left-arm plan"
    # The derived group is the whole branch, finger included (the
    # single-arm rule); an arm declared by its hand leaves the finger out.
    finger = names.index("left_finger")
    assert any(q[finger] != q0[finger] for q in traj.positions)
    armed = bt.Scene(dual.define_group("left", tip="left_hand"))
    armed.add_box("post", (0.05, 0.05, 0.10), (-0.17, 0.25, 0.10))
    traj = armed.plan_to_pose(p, quat, group="left", seed=1)
    assert all(q[i] == q0[i] for q in traj.positions for i in right + [finger])
    # A link alone names its arm.
    traj = scene.plan_to_pose(p, quat, link="left_hand", seed=1)
    assert all(q[i] == q0[i] for q in traj.positions for i in right)
    # A goal in the group's own joint order is accepted.
    short = [bent[names.index(j)] for j in dual.group("left").joints]
    traj = scene.plan(short, group="left", seed=1)
    assert len(traj.positions[0]) == dual.dof


def test_attach_defaults_to_the_groups_tip(dual: bt.Robot) -> None:
    scene = bt.Scene(dual)
    p, _ = scene.link_pose("right_hand")
    scene.add_box("part", (0.03, 0.03, 0.03), (p[0], p[1], p[2] - 0.08))
    scene.attach("part", group="right")
    assert scene.attachments == [("part", "right_hand")]


def test_motions_carry_their_group(dual: bt.Robot) -> None:
    scene = bt.Scene(dual)
    names = dual.joint_names
    goal = list(scene.joint_positions)
    goal[names.index("left_elbow")] = 1.0
    goal[names.index("right_elbow")] = 1.0
    scene.add_segment("r", goal=goal, group="right")
    traj = scene.plan_motion("r", broadcast=False)
    end = traj.positions[-1]
    assert abs(end[names.index("right_elbow")] - 1.0) < 1e-9
    assert end[names.index("left_elbow")] == 0.0     # outside the group: ignored
    with pytest.raises(ValueError, match="unknown group"):
        scene.add_segment("x", goal=goal, group="torso")


def test_define_group_declares_and_then_demands_a_name(dual: bt.Robot) -> None:
    declared = dual.define_group("l", tip="left_hand").define_group("r", tip="right_hand")
    assert declared.groups == ["l", "r"]
    assert declared.group("l").joints == ["left_shoulder", "left_elbow", "left_wrist"]
    assert not declared.group("l").derived
    with pytest.raises(ValueError, match="several arms"):
        _ = declared.tcp_link
    scene = bt.Scene(declared)
    with pytest.raises(ValueError, match="pass group="):
        scene.plan_to_pose((0.1, 0.2, 0.3))
    # The derived robot keeps the legacy answer: no group means every joint.
    assert bt.Scene(dual).plan([0.0] * dual.dof).positions


def test_dual_arm_composes_two_arms(arm: bt.Robot) -> None:
    pair = bt.Robot.dual_arm(
        arm, arm,
        left_position=(0.0, 0.3, 0.8), left_quaternion=(0.0, 0.0, 0.0, 1.0),
        right_position=(0.0, -0.3, 0.8), right_quaternion=(0.0, 0.0, 0.0, 1.0),
    )
    assert pair.dof == 12
    assert pair.groups == ["left", "right"]
    assert pair.group("left").tip == "left_tool0"
    assert pair.group("right").tip == "right_tool0"
    assert set(pair.group("left").joints).isdisjoint(pair.group("right").joints)
    scene = bt.Scene(pair)
    p, quat = scene.link_pose("right_tool0")
    target = (p[0] + 0.1, p[1], p[2] - 0.1)
    traj = scene.plan_to_pose(target, group="right", seed=1)
    names = pair.joint_names
    left = [i for i, n in enumerate(names) if n.startswith("left_")]
    assert all(q[i] == 0.0 for q in traj.positions for i in left)
    # The composite round-trips through a project and a generated script.
    scene.add_segment("reach", goal=traj.positions[-1], group="right")
    src = scene.generate_python()
    assert ".mount(" in src and 'group="right"' in src
    ns: dict = {}
    # The generated script ends by opening the studio; run everything above it.
    exec(compile(src.replace("bt.studio(scene)", ""), "<generated>", "exec"), ns)
    rebuilt: bt.Scene = ns["scene"]
    assert rebuilt.robot.groups == ["left", "right"]
    assert rebuilt.motion_segments("reach")


def test_declared_groups_round_trip_through_a_project(dual: bt.Robot, tmp_path: Path) -> None:
    declared = dual.define_group("l", tip="left_hand").define_group("r", tip="right_hand")
    scene = bt.Scene(declared)
    scene.add_segment("reach", goal=scene.joint_positions, group="r")
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    loaded = bt.Scene.load_project(path)
    assert loaded.robot.groups == ["l", "r"]
    src = scene.generate_python()
    assert 'define_group("l"' in src and 'group="r"' in src
    ns: dict = {}
    exec(compile(src.replace("bt.studio(scene)", ""), "<generated>", "exec"), ns)
    assert ns["scene"].robot.groups == ["l", "r"]


# ------------------------------------------------------------ A1: the bake


def _reach(scene: bt.Scene, name: str, group: str, targets: dict) -> None:
    names = scene.robot.joint_names
    goal = list(scene.joint_positions)
    for joint, value in targets.items():
        goal[names.index(joint)] = value
    scene.add_segment(name, goal=goal, group=group)


def test_two_arms_drive_at_once(dual: bt.Robot) -> None:
    scene = bt.Scene(dual)
    robot = scene.robots[0]
    _reach(scene, "left_reach", "left", {"left_shoulder": 1.2, "left_elbow": -1.0})
    sq = scene.sequence("both")
    sq.step("left", actions=[bt.seq.motion("left_reach")], transition=bt.seq.immediately())
    sq.step("right", actions=[bt.seq.ramp({"right_elbow": 1.0}, 0.5)],
            transition=bt.seq.robot_done(robot, group="right"))
    sq.step("wait left", transition=bt.seq.robot_done(robot, group="left"))
    tl = sq.simulate()
    names = dual.joint_names
    end = tl.sample(tl.duration)
    assert abs(end[names.index("left_elbow")] + 1.0) < 1e-9
    assert abs(end[names.index("right_elbow")] - 1.0) < 1e-9
    mid = tl.sample(0.25)
    assert abs(mid[names.index("right_elbow")] - 0.5) < 1e-6      # the ramp, halfway
    assert mid[names.index("left_shoulder")] > 0.05                # the motion, under way
    assert tl.moves(group="right") == [("ramp", 0.0, 0.5)]
    assert [m[0] for m in tl.moves(group="left")] == ["left_reach"]
    assert abs(tl.busy_seconds(group="right") - 0.5) < 1e-9
    assert tl.utilization(group="left") > tl.utilization(group="right")
    right = tl.robot_trajectory(group="right")
    assert right.joint_names == dual.group("right").joints
    assert len(right.positions[0]) == 4
    with pytest.raises(ValueError, match="unknown group"):
        tl.moves(group="torso")


def test_a_busy_arm_refuses_a_second_driver(dual: bt.Robot) -> None:
    scene = bt.Scene(dual)
    _reach(scene, "left_reach", "left", {"left_shoulder": 1.2})
    sq = scene.sequence("clash")
    sq.step("left", actions=[bt.seq.motion("left_reach")], transition=bt.seq.immediately())
    sq.step("fingers", actions=[bt.seq.ramp({"left_finger": 0.5}, 0.2)], transition=bt.seq.done())
    with pytest.raises(ValueError, match="driven by `left_reach`"):
        sq.simulate()


def test_arms_meeting_is_a_group_collision(dual: bt.Robot) -> None:
    scene = bt.Scene(dual)
    scene.add_box("plate", (0.06, 1.0, 0.02), (-0.33, 0.25, 0.20))
    _reach(scene, "right_reach", "right", {"right_shoulder": 0.3, "right_wrist": 2.5})
    sq = scene.sequence("clash")
    sq.step("left out", actions=[bt.seq.ramp({"left_shoulder": 1.2}, 2.0)], transition=bt.seq.done())
    sq.step("grip", actions=[bt.seq.attach("plate", group="left")], transition=bt.seq.immediately())
    sq.step("right in", actions=[bt.seq.motion("right_reach")], transition=bt.seq.immediately())
    sq.step("left back", actions=[bt.seq.ramp({"left_shoulder": 0.6}, 1.0)], transition=bt.seq.done())
    with pytest.raises(ValueError, match="collide at t = 2") as caught:
        sq.simulate()
    assert "`left`" in str(caught.value) and "`right`" in str(caught.value) and "plate" in str(caught.value)


def test_a_hand_follows_what_the_other_holds(dual: bt.Robot) -> None:
    scene = bt.Scene(dual)
    scene.add_box("tray", (0.04, 0.04, 0.04), (0.0, 0.25, -0.02))
    _reach(scene, "carry", "left", {"left_shoulder": 0.8, "left_elbow": -0.6})
    sq = scene.sequence("hold")
    sq.step("hold", actions=[bt.seq.attach("tray", group="left"), bt.seq.track("tray", group="right")],
            transition=bt.seq.immediately())
    sq.step("carry", actions=[bt.seq.motion("carry")], transition=bt.seq.done())
    sq.step("let go", actions=[bt.seq.untrack(group="right")], transition=bt.seq.immediately())
    tl = sq.simulate()

    def gap(t: float) -> float:
        q = tl.sample(t)
        scene.set_joint_positions(q)
        hand, _ = scene.link_pose("right_hand")
        tray, _ = tl.object_pose("tray", t)
        return sum((a - b) ** 2 for a, b in zip(hand, tray)) ** 0.5

    gap0 = gap(0.0)
    assert all(abs(gap(tl.duration * k / 8) - gap0) < 2e-3 for k in range(1, 9))
    # The generated script carries the arm-addressed actions.
    src = scene.generate_python()
    assert 'bt.seq.track("tray", group="right")' in src
    assert 'bt.seq.untrack(group="right")' in src
    assert 'group="left"' in src


def test_a_zone_can_watch_one_arm(dual: bt.Robot) -> None:
    scene = bt.Scene(dual)
    robot = scene.robots[0]
    for name, arm in (("zone_left", "left"), ("zone_right", "right")):
        scene.add_zone_sensor(name, position=(-0.3, 0.25, 0.3), size=(0.3, 0.3, 0.3),
                              watch_groups=[(robot, arm)])
    _reach(scene, "left_reach", "left", {"left_shoulder": 1.2})
    sq = scene.sequence("swing")
    sq.step("left", actions=[bt.seq.motion("left_reach")], transition=bt.seq.done())
    tl = sq.simulate()
    signals = dict(tl.signals)
    assert signals["zone_left"][-1][1] is True
    assert all(not v for _, v in signals["zone_right"])
    src = scene.generate_python()
    assert f'watch_groups=[("{robot}", "left")]' in src
