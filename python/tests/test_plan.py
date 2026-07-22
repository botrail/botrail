import json
import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


def goal_folded() -> list[float]:
    return [1.2, 0.9, -1.5, 0.5, 0.0, 0.0]


def test_plan_free_space(scene: bt.Scene) -> None:
    traj = scene.plan(goal_folded(), broadcast=False)
    assert traj.duration > 0.0
    assert traj.sample(0.0) == pytest.approx(scene.joint_positions, abs=1e-9)
    assert traj.sample(traj.duration) == pytest.approx(goal_folded(), abs=1e-9)
    # Times strictly increasing.
    assert all(b > a for a, b in zip(traj.times, traj.times[1:]))
    assert traj.joint_names[0] == "shoulder_pan"
    # A single plan carries its sparse path as one joint segment.
    [(kind, waypoints)] = traj.segments
    assert kind == "joint"
    assert waypoints[0] == pytest.approx(scene.joint_positions, abs=1e-9)
    assert waypoints[-1] == pytest.approx(goal_folded(), abs=1e-9)
    assert len(waypoints) < len(traj.times)


def test_plan_avoids_obstacle(scene: bt.Scene) -> None:
    # A wall between the upright start and a goal folded straight through
    # it: the direct joint-space interpolation must be blocked (so the test
    # exercises a real detour), and every sampled state of the planned
    # trajectory must stay collision-free.
    scene.add_box("wall", (0.05, 0.8, 0.5), (0.28, 0.0, 0.45))
    start = scene.joint_positions
    goal = [0.0, 1.57, 0.0, 0.0, 0.0, 0.0]

    def blocked(a: list[float], b: list[float]) -> bool:
        for k in range(51):
            u = k / 50
            scene.set_joint_positions([x + (y - x) * u for x, y in zip(a, b)])
            if scene.in_collision():
                return True
        return False

    assert blocked(start, goal), "scenario is trivial: straight line is free"
    scene.set_joint_positions(start)

    traj = scene.plan(goal, seed=7, broadcast=False)
    t = 0.0
    while t <= traj.duration:
        q = traj.sample(t)
        scene.set_joint_positions(q)
        assert not scene.in_collision(), f"collision at t={t:.3f}"
        t += 0.05
    scene.set_joint_positions(goal)
    assert not scene.in_collision()


def test_plan_reports_failures(scene: bt.Scene) -> None:
    # Blob sits where the horizontally-folded goal pose reaches, but clear
    # of the upright start.
    scene.add_sphere("blob", 0.15, (0.5, 0.0, 0.14))
    goal = [0.0, math.pi / 2, 0.0, 0.0, 0.0, 0.0]
    with pytest.raises(ValueError, match="goal"):
        scene.plan(goal, broadcast=False)

    with pytest.raises(ValueError, match="joint"):
        scene.plan([0.0, 0.0], broadcast=False)


def test_plan_respects_velocity_limits(scene: bt.Scene) -> None:
    traj = scene.plan(goal_folded(), broadcast=False)
    # Velocity limits from examples/simple_arm.urdf, in q order.
    vmax = {name: v for name, v in zip(traj.joint_names, [2.0, 2.0, 2.5, 3.0, 3.0, 3.0])}
    times, positions = traj.times, traj.positions
    for (t0, q0), (t1, q1) in zip(zip(times, positions), zip(times[1:], positions[1:])):
        for name, a, b in zip(traj.joint_names, q0, q1):
            v = abs(b - a) / (t1 - t0)
            assert v <= vmax[name] * 1.001, f"{name} velocity {v}"


def test_trajectory_exports(scene: bt.Scene, tmp_path: Path) -> None:
    traj = scene.plan(goal_folded(), broadcast=False)

    json_path = tmp_path / "traj.json"
    traj.export_json(json_path)
    payload = json.loads(json_path.read_text())
    assert payload["joint_names"] == traj.joint_names
    assert len(payload["times"]) == len(payload["positions"])

    csv_path = tmp_path / "traj.csv"
    traj.export_csv(csv_path, dt=0.05)
    lines = csv_path.read_text().strip().splitlines()
    assert lines[0] == "t," + ",".join(traj.joint_names)
    assert len(lines) >= int(traj.duration / 0.05)
    last = [float(x) for x in lines[-1].split(",")]
    assert last[1:] == pytest.approx(goal_folded(), abs=1e-4)


def test_plan_to_pose(scene: bt.Scene) -> None:
    traj = scene.plan_to_pose((0.3, 0.1, 0.5), broadcast=False)
    scene.set_joint_positions(traj.sample(traj.duration))
    (x, y, z), _ = scene.link_pose("tool0")
    assert (x, y, z) == pytest.approx((0.3, 0.1, 0.5), abs=1e-3)
