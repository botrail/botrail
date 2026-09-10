"""AMR CLI dispatch without catalog downloads or a blocking Studio server."""

import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "examples/vehicles"))


@pytest.fixture
def cli(monkeypatch):
    import amr_demo as demo

    machine = SimpleNamespace(
        product="AMR", maker="Generic", deck=0.69, proud=0.0,
        mount=(0.0, 0.0, 0.705), tray_size=(0.4, 0.5),
        length=1.0, width=0.8, swing=0.64, infeed=(-2.0, 0.0),
        specs={"max_speed_mps": 1.5}, cruise=lambda _: 0.5,
    )
    scene, timeline = Mock(), Mock()
    timeline.duration = 30.24
    timeline.step_spans = []
    timeline.signals = [(name, []) for name in ("amr", "tray_loaded", "overhang", "outfeed")]
    timeline.step_span.return_value = SimpleNamespace(start=10.0, end=17.0)
    timeline.base_pose.return_value = ((0.0, 0.0, 0.7), (0.0, 0.0, 0.0, 1.0))
    timeline.object_pose.return_value = ((1.34, 0.7, 0.65), (0.0, 0.0, 0.0, 1.0))
    timeline.moves.return_value = []
    monkeypatch.setattr(demo, "Carrier", Mock(return_value=machine))
    monkeypatch.setattr(demo, "manifest", Mock(return_value={"name": "Catalog product"}))
    monkeypatch.setattr(demo, "load_chain", Mock(return_value=[]))
    monkeypatch.setattr(demo, "pivot_at", Mock(return_value=15.0))
    monkeypatch.setattr(demo, "bake", Mock(return_value=(scene, timeline)))
    monkeypatch.setattr(demo, "compare", Mock())
    monkeypatch.setattr(demo.bt, "studio", Mock())
    return demo, scene, timeline


@pytest.mark.parametrize("args, output, carrier, holonomic, studio", [
    ([], "cell_amr.usda", "rb-kairos", False, False),
    (["--studio"], "cell_amr.usda", "rb-kairos", False, True),
    (["custom.usdc", "--studio"], "custom.usdc", "rb-kairos", False, True),
    (["--studio", "--carrier", "rb-theron", "custom.usdc", "--holonomic"],
     "custom.usdc", "rb-theron", True, True),
])
def test_export_before_optional_studio(cli, monkeypatch, args, output, carrier, holonomic, studio):
    demo, scene, timeline = cli
    monkeypatch.setattr(sys, "argv", ["amr_demo.py", *args])

    def open_studio(opened_scene):
        assert opened_scene is scene
        timeline.export_usd.assert_called_once_with(output, fps=60)

    demo.bt.studio.side_effect = open_studio
    demo.main()
    demo.bake.assert_called_once_with(carrier, False, holonomic=holonomic)
    timeline.export_usd.assert_called_once_with(output, fps=60)
    if studio:
        demo.bt.studio.assert_called_once_with(scene)
    else:
        demo.bt.studio.assert_not_called()


def test_comparison_does_not_open_studio(cli, monkeypatch):
    demo, _, timeline = cli
    monkeypatch.setattr(sys, "argv", ["amr_demo.py", "--compare", "--studio"])
    demo.main()
    demo.compare.assert_called_once_with()
    demo.bake.assert_not_called()
    timeline.export_usd.assert_not_called()
    demo.bt.studio.assert_not_called()


def test_unknown_load_does_not_prevent_cli_export(cli, monkeypatch, capsys):
    demo, _, timeline = cli
    demo.load_chain.return_value = [("gripper", None, demo.PART_MASS), ("arm", 16, None)]
    monkeypatch.setattr(sys, "argv", ["amr_demo.py"])
    demo.main()
    output = capsys.readouterr().out
    assert "rated unknown" in output
    assert "unknown (component mass not declared)" in output
    timeline.export_usd.assert_called_once_with("cell_amr.usda", fps=60)


@pytest.mark.parametrize("failure", ["bake", "export"])
def test_failed_cycle_or_export_does_not_open_studio(cli, monkeypatch, failure):
    demo, _, timeline = cli
    monkeypatch.setattr(sys, "argv", ["amr_demo.py", "--studio", "--drive-and-plan"])
    if failure == "bake":
        demo.bake.side_effect = ValueError("cannot plan while driving")
        with pytest.raises(SystemExit) as error:
            demo.main()
        assert error.value.code == 1
        timeline.export_usd.assert_not_called()
    else:
        timeline.export_usd.side_effect = OSError("export failed")
        with pytest.raises(OSError, match="export failed"):
            demo.main()
    demo.bt.studio.assert_not_called()
