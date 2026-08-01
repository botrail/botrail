"""R3: multi-actor sequences — the §1 interlock scenario via the Python API."""

import botrail as bt
import pytest

# Two 1-DOF sliders facing each other across a shared band (the Rust
# multi_actor_tests fixture, driven end to end through Python).
SLIDER = """
<robot name="slider">
  <link name="base"/>
  <link name="rod">
    <visual>
      <origin xyz="0 0.25 0"/>
      <geometry><box size="0.08 0.5 0.08"/></geometry>
    </visual>
  </link>
  <joint name="s" type="prismatic">
    <parent link="base"/><child link="rod"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.6" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def dual_cell() -> bt.Scene:
    robot = bt.Robot.from_urdf_string(SLIDER)
    scene = bt.Scene(robot, base_position=(0.0, -0.75, 0.0), name="a")
    scene.add_robot(
        robot, name="b", base_position=(0.0, 0.75, 0.0), base_quaternion=(0, 0, 1, 0)
    )
    scene.add_segment("a_in", [0.45], robot="a")
    scene.add_segment("a_out", [0.0], robot="a")
    scene.add_segment("b_in", [0.45], robot="b")
    scene.add_segment("b_out", [0.0], robot="b")
    scene.add_zone_sensor(
        "zone", position=(0.0, 0.0, 0.3), size=(0.4, 0.4, 0.4), watch_robots=["a"]
    )
    return scene


def interlocked(scene: bt.Scene) -> None:
    sq = scene.sequence("cell")
    sq.step("A enter", actions=[bt.seq.motion("a_in")])
    sq.step("A retreat", actions=[bt.seq.motion("a_out")], transition=bt.seq.immediately())
    sq.step("B interlock", transition=bt.seq.signal("zone", False))
    sq.step("B enter", actions=[bt.seq.motion("b_in")])
    sq.step("B retreat", actions=[bt.seq.motion("b_out")], transition=bt.seq.immediately())
    sq.step(
        "cycle end",
        transition=bt.seq.all_of(bt.seq.robot_done("a"), bt.seq.robot_done("b")),
    )


def test_interlocked_dual_cell_bakes_deterministically() -> None:
    scene = dual_cell()
    interlocked(scene)
    tl = scene.simulate_sequence("cell")

    assert tl.robots == ["a", "b"]
    a_moves = tl.moves("a")
    b_moves = tl.moves("b")
    assert [m[0] for m in a_moves] == ["a_in", "a_out"]
    assert [m[0] for m in b_moves] == ["b_in", "b_out"]
    # Concurrency: B entered while A was still retreating...
    assert b_moves[0][1] < a_moves[1][2]
    # ...and per-robot tracks sample independently.
    mid_a_in = a_moves[0][2] / 2.0
    assert tl.sample(mid_a_in, robot="a")[0] > 0.01
    assert tl.sample(mid_a_in, robot="b")[0] == 0.0
    # With two robots an unaddressed sample is ambiguous.
    with pytest.raises(ValueError, match="pass robot="):
        tl.sample(0.0)

    again = scene.simulate_sequence("cell")
    assert again.duration == tl.duration
    for name in ("a", "b"):
        assert tl.robot_trajectory(name).positions == again.robot_trajectory(name).positions


def test_dropping_the_interlock_reports_the_collision() -> None:
    scene = dual_cell()
    sq = scene.sequence("clash")
    sq.step("both enter", actions=[bt.seq.motion("a_in"), bt.seq.motion("b_in")])
    with pytest.raises(ValueError, match="collide at t = .*interlock"):
        scene.simulate_sequence("clash")


def test_handover_and_robot_scoped_actions() -> None:
    scene = dual_cell()
    scene.add_box("box", (0.04, 0.04, 0.04), (0.0, 0.0, 0.3))
    sq = scene.sequence("pass")
    sq.step("A grasp", actions=[bt.seq.attach("box", robot="a")], transition=bt.seq.elapsed(0.2))
    sq.step("A place", actions=[bt.seq.detach("box")], transition=bt.seq.elapsed(0.2))
    sq.step("B grasp", actions=[bt.seq.attach("box", robot="b")], transition=bt.seq.elapsed(0.2))
    tl = scene.simulate_sequence("pass")
    assert tl.duration == pytest.approx(0.6, abs=0.05)
    # The object rides whoever holds it; the pose stays queryable throughout.
    assert tl.object_pose("box", 0.1) is not None
    assert tl.object_pose("box", tl.duration) is not None


def test_ambiguous_robot_in_actions_is_rejected() -> None:
    scene = dual_cell()
    sq = scene.sequence("bad")
    sq.step("ramp", actions=[bt.seq.ramp({"s": 0.1}, 0.2)])
    with pytest.raises(ValueError, match="give the action a robot"):
        scene.simulate_sequence("bad")
