import json
import math
import urllib.request
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def test_motion_editing(scene: bt.Scene) -> None:
    assert scene.motion_names == []
    scene.add_segment("main", goal=[0.5, 0.4, -0.5, 0.2, 0.0, 0.0])
    scene.add_segment("main")  # captures current (upright) pose
    assert scene.motion_names == ["main"]
    segments = scene.motion_segments("main")
    assert len(segments) == 2
    assert segments[0][0] == "joint"
    assert segments[1][1] == scene.joint_positions

    scene.remove_segment("main", 1)
    assert len(scene.motion_segments("main")) == 1
    with pytest.raises(ValueError):
        scene.remove_segment("main", 5)
    scene.clear_motion("main")
    assert scene.motion_segments("main") == []
    with pytest.raises(ValueError):
        scene.add_segment("main", goal=[0.0], kind="joint")
    with pytest.raises(ValueError):
        scene.add_segment("main", kind="teleport")


def test_plan_motion_passes_through_waypoints(scene: bt.Scene) -> None:
    g1 = [0.6, 0.4, -0.5, 0.2, 0.0, 0.0]
    g2 = [-0.4, 0.8, -1.0, 0.0, 0.3, 0.0]
    scene.add_segment("main", goal=g1)
    scene.add_segment("main", goal=g2)
    traj = scene.plan_motion("main", broadcast=False)

    assert len(traj.segment_ends) == 2
    assert traj.segment_ends[1] == pytest.approx(traj.duration)
    assert traj.sample(traj.segment_ends[0]) == pytest.approx(g1, abs=1e-6)
    assert traj.sample(traj.duration) == pytest.approx(g2, abs=1e-6)


def test_plan_motion_exposes_sparse_segments(scene: bt.Scene) -> None:
    g1 = [0.6, 0.4, -0.5, 0.2, 0.0, 0.0]
    g2 = [-0.4, 0.8, -1.0, 0.0, 0.3, 0.0]
    scene.add_segment("main", goal=g1)
    scene.add_segment("main", goal=g2)
    traj = scene.plan_motion("main", broadcast=False)

    assert len(traj.segments) == 2
    kind, waypoints = traj.segments[0]
    assert kind == "joint"
    # Chained endpoints: start config -> g1 -> g2, exactly.
    assert waypoints[0] == pytest.approx(scene.joint_positions)
    assert waypoints[-1] == pytest.approx(g1)
    assert traj.segments[1][1][0] == pytest.approx(g1)
    assert traj.segments[1][1][-1] == pytest.approx(g2)
    # Sparse: fewer waypoints than the densified trajectory samples.
    total = sum(len(wps) for _, wps in traj.segments)
    assert total < len(traj.times)


def test_cartesian_segment_moves_tcp_on_a_line(scene: bt.Scene) -> None:
    # Fold horizontally, then a straight 8cm descent as a cartesian segment.
    start = [0.0, 1.1, -0.6, -0.5, 0.0, 0.0]
    scene.set_joint_positions(start)
    (x, y, z), quat = scene.link_pose("tool0")
    goal_ik = scene.robot.ik((x, y, z - 0.08), quat, seed=start)
    assert goal_ik.converged

    scene.add_segment("descend", goal=goal_ik.q, kind="cartesian_line")
    traj = scene.plan_motion("descend", broadcast=False)

    kind, waypoints = traj.segments[0]
    assert kind == "cartesian_line"
    assert waypoints[0] == pytest.approx(start)

    t = 0.0
    while t <= traj.duration:
        scene.set_joint_positions(traj.sample(t))
        (px, py, pz), _ = scene.link_pose("tool0")
        # x/y stay on the vertical line within tolerance.
        assert (px, py) == pytest.approx((x, y), abs=2e-3)
        assert z - 0.081 <= pz <= z + 1e-3
        t += 0.05


def test_constraint_sugar_roundtrip(scene: bt.Scene) -> None:
    scene.set_joint_positions([0.0, 1.5708, 0.0, 0.0, 0.0, 0.0])
    scene.add_segment(
        "constrained",
        goal=[1.0, 1.5708, 0.0, 0.0, 0.0, 0.0],
        orientation_cone=((0, 0, 1), (1, 1, 0), 1.0),
    )
    traj = scene.plan_motion("constrained", seed=3, broadcast=False)
    assert traj.duration > 0


def test_project_save_load_roundtrip(scene: bt.Scene, tmp_path: Path) -> None:
    scene.add_box("wall", (0.05, 0.8, 0.5), (0.28, 0.0, 0.45))
    scene.add_segment("main", goal=[0.5, 0.4, -0.5, 0.2, 0.0, 0.0])
    scene.set_joint_positions([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])

    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    payload = json.loads(path.read_text())
    assert payload["version"] == 2
    assert len(payload["robots"]) == 1
    assert payload["robots"][0]["source"]["kind"] == "urdf"
    assert "<robot" in payload["robots"][0]["source"]["xml"]

    loaded = bt.Scene.load_project(path)
    assert loaded.robot.name == "simple_arm"
    assert loaded.joint_positions == pytest.approx(scene.joint_positions)
    assert loaded.obstacle_names == ["wall"]
    assert loaded.motion_names == ["main"]
    # The reloaded scene plans the same motion.
    traj = loaded.plan_motion("main", broadcast=False)
    assert traj.duration > 0


def test_generated_python_compiles(scene: bt.Scene) -> None:
    scene.add_sphere("ball", 0.05, (0.6, 0.0, 0.2))
    scene.add_segment("main", goal=[0.3, 0.5, -0.4, 0.0, 0.0, 0.0])
    code = scene.generate_python()
    compile(code, "generated_scene.py", "exec")  # syntax must be valid
    for needle in ("bt.Robot.from_urdf_string", "scene.add_sphere", "scene.plan_motion"):
        assert needle in code


def test_http_project_endpoints(scene: bt.Scene) -> None:
    import time

    from botrail import _core

    scene.add_box("wall", (0.1, 0.1, 0.1), (0.6, 0.0, 0.2))
    scene.add_segment("main", goal=[0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    server = _core.serve_studio(scene, "/nonexistent-studio", "127.0.0.1", 0)
    try:
        deadline = time.time() + 5
        project = None
        while time.time() < deadline:
            try:
                with urllib.request.urlopen(f"{server.url}/api/project", timeout=1) as resp:
                    project = json.load(resp)
                break
            except OSError:
                time.sleep(0.05)
        assert project is not None and project["version"] == 2
        assert [o["name"] for o in project["obstacles"]] == ["wall"]
        assert [m["name"] for m in project["motions"]] == ["main"]

        # Round-trip the project back through POST (rename the obstacle).
        project["obstacles"][0]["name"] = "renamed"
        req = urllib.request.Request(
            f"{server.url}/api/project",
            data=json.dumps(project).encode(),
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=2) as resp:
            assert resp.status == 200
        assert scene.obstacle_names == ["renamed"]

        with urllib.request.urlopen(f"{server.url}/api/export.py", timeout=2) as resp:
            code = resp.read().decode()
        assert code.startswith('"""Generated by botrail studio')
    finally:
        server.stop()
