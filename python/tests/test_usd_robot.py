"""USD articulation import (Phase 4): Robot.from_usd end to end."""

import math
from pathlib import Path

import pytest

import botrail as bt

# 2-DOF arm articulation (meters, Z-up): fixed anchor, revolute Z with a
# localPose1 offset, revolute Y. Mirrors the Rust golden fixture.
ARM = """#usda 1.0
(
    defaultPrim = "Robot"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Robot" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom" { double size = 0.1 }
    }

    def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom" { double size = 0.1 }
    }

    def Xform "link2" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom" { double size = 0.1 }
    }

    def Scope "joints"
    {
        def PhysicsFixedJoint "anchor"
        {
            rel physics:body1 = </Robot/base>
        }

        def PhysicsRevoluteJoint "j1"
        {
            rel physics:body0 = </Robot/base>
            rel physics:body1 = </Robot/link1>
            uniform token physics:axis = "Z"
            point3f physics:localPos0 = (0, 0, 0.5)
            point3f physics:localPos1 = (0, 0, -0.2)
            float physics:lowerLimit = -90
            float physics:upperLimit = 90
            custom float physxJoint:maxJointVelocity = 120
        }

        def PhysicsRevoluteJoint "j2"
        {
            rel physics:body0 = </Robot/link1>
            rel physics:body1 = </Robot/link2>
            uniform token physics:axis = "Y"
            point3f physics:localPos0 = (0, 0, 0.2)
            float physics:lowerLimit = -120
            float physics:upperLimit = 120
        }
    }
}
"""


@pytest.fixture()
def robot_usd(tmp_path: Path) -> Path:
    path = tmp_path / "arm.usda"
    path.write_text(ARM)
    return path


def test_from_usd_builds_model(robot_usd: Path) -> None:
    robot = bt.Robot.from_usd(robot_usd)
    assert robot.name == "Robot"
    assert robot.dof == 2
    # Naming contract: prim paths.
    assert robot.joint_names == ["/Robot/joints/j1", "/Robot/joints/j2"]
    assert "/Robot/base" in robot.link_names
    # Degrees -> radians.
    lower, upper = robot.joint_limits[0]
    assert lower == pytest.approx(-math.pi / 2, abs=1e-6)
    assert upper == pytest.approx(math.pi / 2, abs=1e-6)


def test_usd_robot_plans_in_a_scene(robot_usd: Path) -> None:
    scene = bt.Scene(bt.Robot.from_usd(robot_usd))
    # link2 frame: j1 at z=0.5 then j2 at z=0.4 (localPose1 folded in).
    (x, y, z), _ = scene.link_pose("/Robot/link2")
    assert (x, y, z) == pytest.approx((0.0, 0.0, 0.9), abs=1e-6)

    scene.add_box("wall", size=(0.05, 0.4, 0.4), position=(0.25, 0.0, 0.9))
    traj = scene.plan([1.0, 0.8], broadcast=False)
    assert traj.duration > 0.0
    t = 0.0
    while t <= traj.duration:
        scene.set_joint_positions(traj.sample(t))
        assert not scene.in_collision()
        t += 0.05


def test_project_roundtrip_reimports_usd_robot(robot_usd: Path, tmp_path: Path) -> None:
    scene = bt.Scene(bt.Robot.from_usd(robot_usd), base_position=(0.5, 0.0, 0.0))
    scene.set_joint_positions([0.3, -0.4])
    project = tmp_path / "cell.botrail"
    scene.save_project(project)

    reloaded = bt.Scene.load_project(project)
    assert reloaded.robot.name == "Robot"
    assert reloaded.joint_positions == pytest.approx([0.3, -0.4])
    position, _ = reloaded.robot_base_pose
    assert position == pytest.approx((0.5, 0.0, 0.0))

    code = reloaded.generate_python()
    assert "bt.Robot.from_usd(" in code
    assert "articulation_root=\"/Robot\"" in code


def test_studio_serves_usd_asset(robot_usd: Path) -> None:
    import json
    import time
    import urllib.request

    from botrail import _core

    scene = bt.Scene(bt.Robot.from_usd(robot_usd))
    server = _core.serve_studio(scene, "/nonexistent-studio", "127.0.0.1", 0)
    try:
        deadline = time.time() + 5
        data = None
        while time.time() < deadline:
            try:
                with urllib.request.urlopen(f"{server.url}/api/scene", timeout=1) as resp:
                    data = json.load(resp)
                break
            except OSError:
                time.sleep(0.05)
        assert data is not None
        asset = data["scene"]["usd_asset"]
        assert asset == {"url": "/assets/arm.usda", "articulation_root": "/Robot"}

        # The referenced stage is served from the robot's directory.
        with urllib.request.urlopen(f"{server.url}{asset['url']}", timeout=1) as resp:
            body = resp.read().decode()
        assert "PhysicsRevoluteJoint" in body

        # Path traversal is rejected.
        try:
            urllib.request.urlopen(f"{server.url}/assets/foo/../arm.usda", timeout=1)
            raised = False
        except urllib.error.HTTPError as e:
            raised = e.code == 404
        assert raised
    finally:
        server.stop()
