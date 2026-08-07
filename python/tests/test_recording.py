"""USD recording playback: export → play_usd_animation roundtrip."""

from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

# 2-DOF arm articulation (meters, Z-up) — mirrors the Rust golden fixture.
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
def scene(tmp_path: Path) -> bt.Scene:
    robot = tmp_path / "arm.usda"
    robot.write_text(ARM)
    return bt.Scene(bt.Robot.from_usd(robot))


def test_play_recording_joint_and_transform_modes(scene: bt.Scene, tmp_path: Path) -> None:
    scene.add_box("crate", (0.1, 0.1, 0.1), (0.8, 0.0, 0.05))
    traj = scene.plan([0.8, -0.5])
    out = tmp_path / "rec.usda"
    assert scene.export_usd(traj, out, fps=30.0) == []

    # Botrail exports carry JointState for every joint → joint playback.
    res = scene.play_usd_animation(out)
    assert res["mode"] == "joint_state"
    assert res["warnings"] == []
    assert res["duration"] == pytest.approx(traj.times[-1], abs=1 / 30 + 1e-6)

    # The same layer replays as raw transforms on demand.
    forced = scene.play_usd_animation(out, force_transforms=True)
    assert forced["mode"] == "transforms"
    assert forced["duration"] == pytest.approx(res["duration"])


def test_urdf_robots_replay_as_transforms(tmp_path: Path) -> None:
    """A URDF (or `attach_tool` composite) robot has no stage to reference,
    so its export bakes per-link world poses — and that replays: the
    importer resolves the writer's flat link naming and plays the
    transform tier. There are no joint tracks to recover, and none are
    needed."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    traj = scene.plan([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    out = tmp_path / "rec.usda"
    assert scene.export_usd(traj, out, fps=30.0) == []
    res = scene.play_usd_animation(out)
    assert res["mode"] == "transforms"
    assert res["warnings"] == []
    assert res["duration"] == pytest.approx(traj.times[-1], abs=1 / 30 + 1e-6)


def test_play_recording_missing_file(scene: bt.Scene, tmp_path: Path) -> None:
    with pytest.raises(ValueError):
        scene.play_usd_animation(tmp_path / "nope.usda")
