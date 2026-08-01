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
        asset = data["scene"]["robots"][0]["usd_asset"]
        assert asset == {"url": "/usd-assets/0/arm.usda", "articulation_root": "/Robot"}

        # The referenced stage is served from the robot's directory.
        with urllib.request.urlopen(f"{server.url}{asset['url']}", timeout=1) as resp:
            body = resp.read().decode()
        assert "PhysicsRevoluteJoint" in body

        # Path traversal is rejected.
        try:
            urllib.request.urlopen(f"{server.url}/usd-assets/0/foo/../arm.usda", timeout=1)
            raised = False
        except urllib.error.HTTPError as e:
            raised = e.code == 404
        assert raised
    finally:
        server.stop()


def test_usd_robot_project_bundles_the_stage(robot_usd: Path, tmp_path: Path) -> None:
    """The stage layers travel inside the archive: the original can vanish."""
    scene = bt.Scene(bt.Robot.from_usd(robot_usd))
    scene.set_joint_positions([0.5, -0.3])
    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    assert project.read_bytes()[:2] == b"PK"  # USD robot forces the archive

    robot_usd.unlink()  # original stage deleted...
    reloaded = bt.Scene.load_project(project)  # ...reload uses the bundle
    assert reloaded.robot.name == "Robot"
    assert reloaded.robot.dof == 2
    assert reloaded.joint_positions == pytest.approx([0.5, -0.3])


# Golden checks against official Isaac Sim assets (Franka, UR10). Opt-in:
# point BOTRAIL_ISAAC_DIR at a directory containing franka.usd / ur10.usd
# (e.g. from the public mirror:
# https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.2/Isaac/Robots/...).
import math
import os

ISAAC_DIR = os.environ.get("BOTRAIL_ISAAC_DIR")
isaac = pytest.mark.skipif(
    not ISAAC_DIR, reason="BOTRAIL_ISAAC_DIR not set (Isaac assets not available)"
)


@isaac
def test_franka_golden() -> None:
    robot = bt.Robot.from_usd(Path(ISAAC_DIR) / "franka.usd")
    assert robot.dof == 9  # 7 arm + 2 fingers
    names = [j.rsplit("/", 1)[-1] for j in robot.joint_names]
    assert names == [f"panda_joint{i}" for i in range(1, 8)] + [
        "panda_finger_joint1",
        "panda_finger_joint2",
    ]
    # Published Panda position limits (rad), degrees->radians conversion.
    expected = [
        (-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973),
        (-3.0718, -0.0698), (-2.8973, 2.8973), (-0.0175, 3.7525),
        (-2.8973, 2.8973),
    ]
    for (lo, hi), (elo, ehi) in zip(robot.joint_limits[:7], expected):
        assert lo == pytest.approx(elo, abs=1e-3)
        assert hi == pytest.approx(ehi, abs=1e-3)
    # Prismatic fingers: 0..0.04 m (metersPerUnit applied).
    for lo, hi in robot.joint_limits[7:]:
        assert lo == pytest.approx(0.0, abs=1e-6)
        assert hi == pytest.approx(0.04, abs=1e-6)
    # The imported model plans out of the box.
    scene = bt.Scene(robot)
    goal = [0.3, -0.5, 0.2, -1.8, 0.1, 1.5, 0.4, 0.02, 0.02]
    traj = scene.plan(goal, broadcast=False)
    assert traj.duration > 0.0


@isaac
def test_ur10_golden() -> None:
    robot = bt.Robot.from_usd(Path(ISAAC_DIR) / "ur10.usd")
    assert robot.dof == 6
    names = [j.rsplit("/", 1)[-1] for j in robot.joint_names]
    assert names == [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]
    # UR10: every joint +-360 deg -> +-2*pi rad.
    for lo, hi in robot.joint_limits:
        assert lo == pytest.approx(-2 * math.pi, abs=1e-4)
        assert hi == pytest.approx(2 * math.pi, abs=1e-4)
