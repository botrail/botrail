from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def test_export_usd_bakes_robot_and_grasped_object(scene: bt.Scene, tmp_path: Path) -> None:
    tcp, _ = scene.link_pose(scene.robot.tcp_link)
    scene.add_box("held", (0.04, 0.04, 0.04), (tcp[0], tcp[1], tcp[2] + 0.06))
    scene.add_box("/World/Shelf/Board", (0.3, 0.3, 0.02), (0.6, 0.0, 0.4))
    scene.attach("held")

    traj = scene.plan([0.4, -0.3, 0.3, 0.0, 0.2, 0.0])
    out = tmp_path / "anim.usda"
    warnings = scene.export_usd(out, traj, fps=30.0)
    assert warnings == []

    text = out.read_text()
    assert text.startswith("#usda")
    assert 'upAxis = "Z"' in text
    assert "timeSamples" in text
    # URDF robots are authored self-contained: link prims + no asset dir.
    assert "wrist_3_link" in text
    assert not (tmp_path / "anim_assets").exists()
    # The grasped box gets a sampled track; the static shelf a nested prim.
    assert "held" in text
    assert "Shelf" in text


def test_export_usd_rejects_bad_fps(scene: bt.Scene, tmp_path: Path) -> None:
    traj = scene.plan([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        scene.export_usd(tmp_path / "x.usda", traj, fps=0.0)


def test_export_usd_static_writes_the_cell(scene: bt.Scene, tmp_path: Path) -> None:
    scene.add_box("/World/Shelf/Board", (0.3, 0.3, 0.02), (0.6, 0.0, 0.4), color=(0.2, 0.4, 0.6))
    scene.add_box("proxy", (0.1, 0.1, 0.1), (0.9, 0.0, 0.1))
    scene.set_obstacle_visible("proxy", False)

    out = tmp_path / "cell.usda"
    assert scene.export_usd(out) == []
    text = out.read_text()
    assert text.startswith("#usda")
    # The cell as it stands: the robot at its pose, the visible obstacle
    # as a prim — and hidden means hidden.
    assert "wrist_3_link" in text
    assert "Shelf" in text
    assert "proxy" not in text


def test_export_usd_static_round_trips_obstacles(tmp_path: Path) -> None:
    # A robot-less scene — a layout — writes a static layer that reads back.
    src = bt.Scene()
    src.add_box("bench/top", (0.8, 0.4, 0.03), (0.5, 0.0, 0.7), color=(0.5, 0.3, 0.1))
    src.add_cylinder("bench/leg", radius=0.03, length=0.7, position=(0.5, 0.0, 0.35))
    out = tmp_path / "layout.usda"
    assert src.export_usd(out) == []

    back = bt.Scene()
    back.load_usd(out)
    assert sorted(back.obstacle_names) == [
        "/World/Env/bench/leg",
        "/World/Env/bench/top",
    ]
    pos, _ = back.obstacle_pose("/World/Env/bench/top")
    assert pos == pytest.approx((0.5, 0.0, 0.7))


def test_export_usd_static_rejects_robot_selector(scene: bt.Scene, tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="trajectory"):
        scene.export_usd(tmp_path / "x.usda", robot="arm")
