from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


def test_urscript_from_motion(scene: bt.Scene) -> None:
    g1 = [0.6, 0.4, -0.5, 0.2, 0.0, 0.0]
    g2 = [-0.4, 0.8, -1.0, 0.0, 0.3, 0.0]
    scene.add_segment("main", goal=g1)
    scene.add_segment("main", goal=g2)
    traj = scene.plan_motion("main", broadcast=False)

    code = traj.to_script(dialect="urscript", name="pick")
    lines = code.splitlines()
    assert lines[0] == "def pick():"
    assert lines[-1] == "end"

    # One movej per waypoint, shared segment boundaries deduplicated,
    # plus the initial move-to-start.
    total = sum(len(wps) for _, wps in traj.segments)
    assert code.count("movej(") == total - 1
    # Speeds from the URDF limits: min velocity 2.0, acceleration 4.0.
    assert "a=4, v=2" in code
    # Segment goals stop exactly.
    assert code.rstrip().splitlines()[-2].endswith("r=0)")


def test_urscript_speed_scale_and_start(scene: bt.Scene) -> None:
    scene.add_segment("main", goal=[0.5, 0.4, -0.5, 0.2, 0.0, 0.0])
    traj = scene.plan_motion("main", broadcast=False)

    code = traj.to_script(speed_scale=0.5)
    assert "a=2, v=1" in code

    with_start = traj.to_script()
    without_start = traj.to_script(move_to_start=False)
    assert with_start.count("movej(") == without_start.count("movej(") + 1


def test_urscript_cartesian_becomes_movel(scene: bt.Scene) -> None:
    start = [0.0, 1.1, -0.6, -0.5, 0.0, 0.0]
    scene.set_joint_positions(start)
    (x, y, z), quat = scene.link_pose("tool0")
    goal_ik = scene.robot.ik((x, y, z - 0.08), quat, seed=start)
    assert goal_ik.converged

    scene.add_segment("descend", goal=goal_ik.q, kind="cartesian_line")
    traj = scene.plan_motion("descend", broadcast=False)

    code = traj.to_script(name="descend", tcp_speed=0.1)
    # The whole IK follow path collapses to one linear move; the only
    # movej is the initial move-to-start.
    assert code.count("movel(") == 1
    assert code.count("movej(") == 1
    assert "v=0.1" in code


def test_export_script_names_program_after_file(scene: bt.Scene, tmp_path: Path) -> None:
    scene.add_segment("main", goal=[0.5, 0.4, -0.5, 0.2, 0.0, 0.0])
    traj = scene.plan_motion("main", broadcast=False)

    path = tmp_path / "pick_and_place.script"
    traj.export_script(path)
    code = path.read_text()
    assert code.startswith("def pick_and_place():")


def test_single_plan_exports_too(scene: bt.Scene) -> None:
    traj = scene.plan([1.2, 0.9, -1.5, 0.5, 0.0, 0.0], broadcast=False)
    code = traj.to_script()
    assert code.count("movej(") == len(traj.segments[0][1])


def test_unknown_dialect_is_rejected(scene: bt.Scene) -> None:
    traj = scene.plan([0.5, 0.0, 0.0, 0.0, 0.0, 0.0], broadcast=False)
    with pytest.raises(ValueError, match="urscript"):
        traj.to_script(dialect="klingon")
