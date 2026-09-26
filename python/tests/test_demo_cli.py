"""Every cell demo runs the same way (examples/README.md):

    python examples/<group>/<name>.py [out.usd] [--studio]

bake, print, write the USD, exit — and with `--studio` open the studio on
that bake *afterwards* (a studio that connects late replays the last
bake, so the order is the contract). Checked at the source level for the
whole set, so no demo is baked here, and once for real on the smallest
one with its cell mocked out, the way `test_weld_station_cli.py` does.
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, Mock

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

# The demos that build a cell and simulate it. Sweeps, document sets,
# mounting reports, the exporters under export/ and the RL trainer keep
# their own command lines and are not listed.
CELL_DEMOS = [
    "assembly/cover_bolting_demo.py",
    "basics/demo.py",
    "basics/friction_grasp_demo.py",
    "basics/gripper_pick_demo.py",
    "basics/hand_grasp_demo.py",
    "basics/physics_conveyor.py",
    "basics/physics_drop.py",
    "basics/physics_pick_place.py",
    "basics/physics_world_demo.py",
    "basics/sequence_demo.py",
    "basics/sfc_chart_demo.py",
    "drone/drone_survey_demo.py",
    "legged/building_delivery_demo.py",
    "legged/humanoid_carry_demo.py",
    "legged/legged_patrol_demo.py",
    "legged/stairs_delivery_demo.py",
    "legged/wash_inspect_ship_demo.py",
    "machining/ati_deburring_demo.py",
    "machining/machine_tending_demo.py",
    "machining/machining_demo.py",
    "machining/two_machine_cell_demo.py",
    "multi_robot/dual_arm_demo.py",
    "multi_robot/dual_cell_demo.py",
    "painting/painting_demo.py",
    "painting/painting_hood_demo.py",
    "palletizing/palletizing_line_demo.py",
    "rl/policy_cell_demo.py",
    "rl/reach_control_demo.py",
    "rl/reach_env.py",
    "rl/torque_env.py",
    "rl/tabletop_env.py",
    "vehicles/agv_cell_demo.py",
    "vehicles/amr_demo.py",
    "vehicles/lift_demo.py",
    "vehicles/semi_humanoid_demo.py",
    "vehicles/warehouse_demo.py",
    "welding/nimak_spot_welding_demo.py",
    "welding/weld_line_demo.py",
    "welding/weld_station_demo.py",
]


@pytest.mark.parametrize("demo", CELL_DEMOS)
def test_writes_usd_then_opens_studio(demo):
    source = (EXAMPLES / demo).read_text()
    assert '"--studio"' in source, f"{demo} has no --studio option"
    # The entry point: main() where there is one, the module itself otherwise.
    entry = source[source.index("def main"):] if "def main" in source else source
    assert "export_usd(" in entry, f"{demo} does not write a USD"
    assert "bt.studio(" in entry, f"{demo} never opens the studio"
    assert entry.rfind("export_usd(") < entry.rfind("bt.studio("), (
        f"{demo} opens the studio before the USD is written"
    )


@pytest.fixture
def sequence_cli(monkeypatch):
    sys.path.insert(0, str(EXAMPLES / "basics"))
    import sequence_demo as demo

    scene = MagicMock()
    timeline = scene.simulate_sequence.return_value
    timeline.duration = 24.5
    timeline.step_spans = [("latch", 3.0, 3.1), ("grasp", 4.0, 4.4)]
    timeline.object_pose.return_value = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    timeline.export_usd.return_value = []
    monkeypatch.setattr(demo, "build_scene", Mock(return_value=scene))
    monkeypatch.setattr(demo, "build_cycle", Mock(return_value="cycle"))
    monkeypatch.setattr(demo, "identify_parts", Mock())
    monkeypatch.setattr(demo.bt, "studio", Mock())
    return demo, scene, timeline


@pytest.mark.parametrize("args, output, studio", [
    ([], "cell_seq.usda", False),
    (["custom.usdc"], "custom.usdc", False),
    (["--studio"], "cell_seq.usda", True),
    (["custom.usdc", "--studio"], "custom.usdc", True),
    (["--studio", "custom.usdc"], "custom.usdc", True),
])
def test_sequence_demo_export_and_optional_studio(sequence_cli, monkeypatch, tmp_path, args, output, studio):
    demo, scene, timeline = sequence_cli
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(sys, "argv", ["sequence_demo.py", *args])

    def open_studio(opened_scene):
        assert opened_scene is scene
        timeline.export_usd.assert_called_once_with(Path(output), fps=60.0)

    demo.bt.studio.side_effect = open_studio
    demo.main()

    scene.simulate_sequence.assert_called_once_with("cycle")
    timeline.export_usd.assert_called_once_with(Path(output), fps=60.0)
    scene.bom.return_value.save.assert_called_once_with(Path(output).with_name(Path(output).stem + "_bom.csv"))
    if studio:
        demo.bt.studio.assert_called_once_with(scene)
    else:
        demo.bt.studio.assert_not_called()
