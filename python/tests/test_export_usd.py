from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


def test_export_usd_bakes_robot_and_grasped_object(scene: bt.Scene, tmp_path: Path) -> None:
    tcp, _ = scene.link_pose(scene.robot.tcp_link)
    scene.add_box("held", (0.04, 0.04, 0.04), (tcp[0], tcp[1], tcp[2] + 0.06))
    scene.add_box("/World/Shelf/Board", (0.3, 0.3, 0.02), (0.6, 0.0, 0.4))
    scene.attach("held")

    traj = scene.plan([0.4, -0.3, 0.3, 0.0, 0.2, 0.0])
    out = tmp_path / "anim.usda"
    warnings = scene.export_usd(traj, out, fps=30.0)
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
        scene.export_usd(traj, tmp_path / "x.usda", fps=0.0)
