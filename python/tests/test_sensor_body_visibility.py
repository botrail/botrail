"""Replacing a housing must preserve sensing and survive project/script export."""
import json
from pathlib import Path

import botrail as bt
import pytest

ARM = Path(__file__).resolve().parents[2] / "examples/assets/simple_arm.urdf"


@pytest.mark.parametrize("kind", ["camera", "lidar"])
def test_housing_visibility_round_trip_and_legacy_default(kind, tmp_path):
    scene = bt.Scene(bt.Robot.from_urdf(ARM))
    add = getattr(scene, f"add_{kind}")
    add("existing_model", robot=scene.robot.name, link=scene.robot.tcp_link, body_visible=False)
    add("standalone", position=(3, 2, 1))
    key = f"{kind}s"
    original = json.loads(scene._project_json())[key]
    assert [s["body_visible"] for s in original] == [False, True]

    project = tmp_path / "sensors.botrail"
    scene.save_project(project)
    restored = bt.Scene.load_project(project)
    assert json.loads(restored._project_json())[key] == original
    namespace = {}
    exec(restored.generate_python().replace("bt.studio(scene)", ""), namespace)  # noqa: S102 — exercise trusted generated output
    assert json.loads(namespace["scene"]._project_json())[key] == original

    legacy = json.loads(scene._project_json())
    for sensor in legacy[key]:
        del sensor["body_visible"]
    path = tmp_path / "legacy.json"
    path.write_text(json.dumps(legacy))
    restored = bt.Scene.load_project(path)
    sensors = json.loads(restored._project_json())[key]
    assert all(s["body_visible"] for s in sensors)
    for actual, expected in zip(sensors, original):
        assert {k: v for k, v in actual.items() if k != "body_visible"} == {
            k: v for k, v in expected.items() if k != "body_visible"
        }
