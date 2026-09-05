"""CLI dispatch without catalog downloads or a blocking browser server."""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "welding"))


@pytest.fixture
def cli(monkeypatch):
    import weld_station_demo as demo

    scene = Mock()
    timeline = scene.simulate_sequence.return_value
    timeline.duration = 182.50
    timeline.robots = demo.ARMS
    timeline.moves.return_value = []
    timeline.export_usd.return_value = []
    station = SimpleNamespace(length=4.1, height=1.5, seam_y=0.9, seam_z=1.6)
    monkeypatch.setattr(demo, "build_cell", Mock(return_value=(scene, station, [])))
    monkeypatch.setattr(demo, "teach", Mock(return_value={}))
    monkeypatch.setattr(demo, "build_sequence", Mock(return_value="weld_station"))
    monkeypatch.setattr(demo, "build_clash", Mock(return_value="clash"))
    monkeypatch.setattr(demo, "zone_overlap", Mock(return_value=0.0))
    monkeypatch.setattr(demo.bt, "studio", Mock())
    return demo, scene, timeline


@pytest.mark.parametrize("args, output, studio", [
    ([], "cell_weld.usda", False),
    (["custom.usdc"], "custom.usdc", False),
    (["--studio"], "cell_weld.usda", True),
    (["custom.usdc", "--studio"], "custom.usdc", True),
    (["--studio", "custom.usdc"], "custom.usdc", True),
])
def test_export_and_optional_studio(cli, monkeypatch, args, output, studio):
    demo, scene, timeline = cli
    monkeypatch.setattr(sys, "argv", ["weld_station_demo.py", *args])

    def open_studio(opened_scene):
        assert opened_scene is scene
        timeline.export_usd.assert_called_once_with(Path(output), fps=60.0)

    demo.bt.studio.side_effect = open_studio
    demo.main()

    scene.simulate_sequence.assert_called_once_with("weld_station", max_duration=400.0)
    timeline.export_usd.assert_called_once_with(Path(output), fps=60.0)
    if studio:
        demo.bt.studio.assert_called_once_with(scene)
    else:
        demo.bt.studio.assert_not_called()


@pytest.mark.parametrize("collision", [True, False])
def test_clash_does_not_launch_studio(cli, monkeypatch, collision):
    demo, scene, timeline = cli
    monkeypatch.setattr(sys, "argv", ["weld_station_demo.py", "--clash", "--studio"])
    if collision:
        scene.simulate_sequence.side_effect = ValueError("robot-robot collision")
        demo.main()
    else:
        with pytest.raises(SystemExit, match="expected a robot-robot collision"):
            demo.main()

    scene.simulate_sequence.assert_called_once_with("clash", max_duration=60.0)
    timeline.export_usd.assert_not_called()
    demo.bt.studio.assert_not_called()
