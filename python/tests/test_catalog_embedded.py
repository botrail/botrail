"""Saved public catalog sources replay without the dataset or Hub dependency."""

# ruff: noqa: F811
import shutil
import sys

import botrail as bt
import pytest
from test_catalog import ARM_ID, COUPLING_ID, DUAL_ID, catalog  # noqa: F401


def frames(robot):
    tips = (
        [(name, robot.group(name).tip) for name in robot.groups]
        if len(robot.groups) > 1
        else robot.tcp_link
    )
    return robot.mount_link, robot.flange_link, tips


@pytest.mark.parametrize("model", [ARM_ID, COUPLING_ID, DUAL_ID, "composite"])
def test_embedded_public_script_replays_without_dataset(
    catalog, tmp_path, monkeypatch, model
):
    robot = (
        bt.Robot.from_catalog(ARM_ID).attach_tool(bt.Robot.from_catalog(COUPLING_ID))
        if model == "composite"
        else bt.Robot.from_catalog(model)
    )
    scene = bt.Scene(robot)
    if robot.dof:
        scene.set_joint_positions([0.1] * robot.dof)
    bom = scene.bom().rows
    before_frames = frames(robot)
    report = bt.mounting.report(scene).to_dict()
    project = tmp_path / "public.botrail"
    scene.save_project(project)
    shutil.rmtree(catalog["repo"])
    monkeypatch.setitem(sys.modules, "huggingface_hub", None)
    restored = bt.Scene.load_project(project)
    code = restored.generate_python(embed_catalog=True)
    ns = {}
    exec(code.replace("bt.studio(scene)", ""), ns)  # noqa: S102 - generated replay
    replay = ns["scene"]
    assert replay.bom().rows == bom
    assert replay.joint_positions == pytest.approx(scene.joint_positions)
    assert frames(replay.robot) == before_frames
    assert replay.robot.groups == robot.groups
    assert bt.mounting.report(replay).to_dict() == report
