"""A scene with no robot in it: a cell of devices, vehicles and obstacles.

`bt.Scene()` opens empty — the shape a conveyor line, an AGV loop or a
drone cell with a box airframe wants, where before an unrelated arm had to
stand somewhere to satisfy the constructor. Robots stay first-class:
`add_robot` works as ever, and everything robot-implicit answers by name
instead of assuming a first robot.
"""

from __future__ import annotations

import botrail as bt
import pytest


def _cell() -> bt.Scene:
    scene = bt.Scene()
    scene.add_box("agv/chassis", size=(0.6, 0.4, 0.25), position=(0.0, 0.0, 0.125))
    scene.add_box("tote", size=(0.2, 0.2, 0.15), position=(0.0, 0.0, 0.325))
    scene.add_vehicle("agv", body=["agv"], path=[(0.0, 0.0), (3.0, 0.0), (3.0, 2.0)],
                      stations={"a": 0, "b": 2}, speed=0.5, turn_speed=1.2, start="a",
                      tray_position=(0.0, 0.0, 0.3), tray_size=(0.5, 0.4, 0.2))
    seq = scene.sequence("haul")
    seq.step("go", actions=[bt.seq.goto("agv", "b")],
             transition=bt.seq.device_done("agv"))
    return scene


def test_a_robotless_cell_bakes_and_exports(tmp_path) -> None:
    scene = _cell()
    assert scene.robots == []
    tl = scene.simulate_sequence("haul", max_duration=60.0)
    assert tl.duration > 0.0
    p, _ = tl.object_pose("tote", tl.duration)
    assert p[0] == pytest.approx(3.0, abs=1e-3) and p[1] == pytest.approx(2.0, abs=2e-3)
    tl.export_usd(tmp_path / "agv.usdc", fps=30)
    assert (tmp_path / "agv.usdc").stat().st_size > 0


def test_robot_implicit_surfaces_answer_by_name() -> None:
    scene = _cell()
    with pytest.raises(ValueError, match="no robot"):
        scene.robot  # noqa: B018 - the getter itself is the test
    with pytest.raises(ValueError, match="no robot"):
        scene.set_joint_positions([0.0])
    # The robot kwargs describe the robot — without one they are refused
    # rather than silently dropped.
    with pytest.raises(ValueError, match="describe the robot"):
        bt.Scene(name="ghost")


def test_the_project_round_trips_without_robots(tmp_path) -> None:
    scene = _cell()
    scene.save_project(tmp_path / "cell.btproj")
    back = bt.Scene.load_project(tmp_path / "cell.btproj")
    assert back.robots == []
    assert set(back.obstacle_names) == set(scene.obstacle_names)
    code = back.generate_python()
    assert "scene = bt.Scene()" in code
    assert 'scene.add_vehicle("agv"' in code


def test_a_robot_added_later_is_first_class() -> None:
    from pathlib import Path

    scene = _cell()
    arm = bt.Robot.from_urdf(Path(__file__).resolve().parents[2] / "examples" / "assets" / "simple_arm.urdf")
    scene.add_robot(arm, name="arm", base_position=(5.0, 5.0, 0.0))
    assert scene.robots == ["arm"]
    scene.set_joint_positions([0.2] * arm.dof, robot="arm")
    assert scene.joint_positions_of("arm") == pytest.approx([0.2] * arm.dof)
