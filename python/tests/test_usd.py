"""USD scene import (Phase 3): obstacles, frames, normalization, planning."""

import re
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

# Centimeter-unit, Y-up cell: a referenced table (mesh top + cube leg), a
# ball, and a mount frame. USD (100, 0, 50)cm Y-up -> (1.0, -0.5, 0)m Z-up.
CELL = """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "World"
{
    def Xform "Table" (prepend references = @./table.usda@</Table>)
    {
        double3 xformOp:translate = (50, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def Sphere "Ball"
    {
        double radius = 8
        double3 xformOp:translate = (0, 60, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def Xform "MountFrame"
    {
        double3 xformOp:translate = (10, 40, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""

TABLE = """#usda 1.0
(
    defaultPrim = "Table"
)

def Xform "Table"
{
    def Mesh "Top"
    {
        point3f[] points = [(-30, 40, -20), (30, 40, -20), (30, 40, 20), (-30, 40, 20), (-30, 45, -20), (30, 45, -20), (30, 45, 20), (-30, 45, 20)]
        int[] faceVertexCounts = [4, 4, 4, 4, 4, 4]
        int[] faceVertexIndices = [0, 1, 2, 3, 4, 7, 6, 5, 0, 4, 5, 1, 1, 5, 6, 2, 2, 6, 7, 3, 3, 7, 4, 0]
    }

    def Cube "Leg"
    {
        double size = 10
        double3 xformOp:translate = (0, 20, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""


@pytest.fixture()
def cell(tmp_path: Path) -> Path:
    (tmp_path / "table.usda").write_text(TABLE)
    path = tmp_path / "cell.usda"
    path.write_text(CELL)
    return path


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def test_load_usd_imports_obstacles_and_frames(scene: bt.Scene, cell: Path) -> None:
    names = scene.load_usd(cell, prefix="env")
    assert "env/World/Table/Top" in names
    assert "env/World/Table/Leg" in names
    assert "env/World/Ball" in names
    assert set(names) <= set(scene.obstacle_names)

    # cm -> m and Y-up -> Z-up: leg at USD (30, 20, 0) -> (0.3, 0, 0.2).
    frames = scene.frames
    (fx, fy, fz), _ = scene.frame("env/World/MountFrame")
    assert (fx, fy, fz) == pytest.approx((0.1, 0.0, 0.4))
    assert "env/World/MountFrame" in frames

    # The ball (USD (0, 60, 0), r=8cm) lands at world (0, 0, 0.6) — right
    # on the upright arm: collision proves the geometry landed where the
    # unit/up-axis conversion says it should.
    assert scene.in_collision()
    pairs = scene.check_collisions()
    assert any("env/World/Ball" in (a[1], b[1]) for a, b in pairs)


def test_robot_placement_on_imported_frame(scene: bt.Scene, cell: Path) -> None:
    scene.load_usd(cell)
    scene.set_robot_base_pose(*scene.frame("/World/MountFrame"))
    position, _ = scene.robot_base_pose
    assert position == pytest.approx((0.1, 0.0, 0.4))
    (x, y, z), _ = scene.link_pose("tool0")
    assert (x, y, z) == pytest.approx((0.1, 0.0, 1.25))  # 0.85 above base


def test_plan_avoids_imported_geometry(scene: bt.Scene, cell: Path) -> None:
    scene.load_usd(cell)
    scene.remove_obstacle("/World/Ball")  # collides at start; keep the table
    traj = scene.plan([1.2, 0.9, -1.5, 0.5, 0.0, 0.0], broadcast=False)
    t = 0.0
    while t <= traj.duration:
        scene.set_joint_positions(traj.sample(t))
        assert not scene.in_collision(), f"collision at t={t:.2f}"
        t += 0.05


def test_project_roundtrip_preserves_frames(scene: bt.Scene, cell: Path, tmp_path: Path) -> None:
    scene.load_usd(cell)
    project = tmp_path / "cell.botrail"
    scene.save_project(project)

    reloaded = bt.Scene.load_project(project)
    assert set(reloaded.obstacle_names) == set(scene.obstacle_names)
    (fx, fy, fz), _ = reloaded.frame("/World/MountFrame")
    assert (fx, fy, fz) == pytest.approx((0.1, 0.0, 0.4))
    assert "scene.add_frame(\"/World/MountFrame\"" in reloaded.generate_python()


def test_load_usd_carries_display_colour(tmp_path: Path) -> None:
    """A group's constant displayColor paints its subtree; a prim can
    override it, and a prim with none is left to the viewer."""
    stage = tmp_path / "painted.usda"
    stage.write_text(
        """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "World" {
    def Xform "Rig" {
        color3f[] primvars:displayColor = [(0.2, 0.4, 0.6)]

        def Cube "Frame" { double size = 0.2 }

        def Cube "Guard" {
            double size = 0.2
            color3f[] primvars:displayColor = [(0.9, 0.7, 0.1)]
            double3 xformOp:translate = (0.5, 0, 0)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }
    }

    def Cube "Plain" {
        double size = 0.2
        double3 xformOp:translate = (0, 0.5, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""
    )
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.load_usd(stage)

    assert scene.obstacle_color("/World/Rig/Frame") == pytest.approx((0.2, 0.4, 0.6))
    assert scene.obstacle_color("/World/Rig/Guard") == pytest.approx((0.9, 0.7, 0.1))
    assert scene.obstacle_color("/World/Plain") is None


def test_load_usd_imports_camera_prims(tmp_path: Path) -> None:
    """A stage's `Camera` prims become world-fixture cameras with the
    authored optics: the film back is the horizontal fov, the aperture
    aspect the image aspect, `clippingRange` the near/far — and a camera
    Omniverse hid with `visibility = "invisible"` still counts."""
    stage = tmp_path / "cams.usda"
    stage.write_text(
        """#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1
    upAxis = "Z"
)
def Xform "World" {
    def Cube "Bench" { double size = 0.5 }
    def Camera "Overview" {
        float focalLength = 18.147562
        float horizontalAperture = 20.955
        float verticalAperture = 11.787
        float2 clippingRange = (0.05, 30)
        token visibility = "invisible"
        double3 xformOp:translate = (1.5, -1.5, 1.2)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
"""
    )
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    names = scene.load_usd(stage, prefix="env")
    assert names == ["env/World/Bench"]
    assert scene.camera_names == ["env/World/Overview"]

    code = scene.generate_python()
    m = re.search(
        r'scene\.add_camera\("env/World/Overview", position=\(1\.5, -1\.5, 1\.2\).*'
        r"fov=([\d.]+), resolution=\((\d+), (\d+)\), near=([\d.]+), far=([\d.]+)",
        code,
    )
    assert m, code
    # 18.15 mm on a 20.955 mm film back is a 60° view; the aperture aspect
    # (16:9) sizes the default 1280-wide image; USD's float attributes
    # come back through f32.
    assert float(m.group(1)) == pytest.approx(60.0, abs=1e-5)
    assert (int(m.group(2)), int(m.group(3))) == (1280, 720)
    assert float(m.group(4)) == pytest.approx(0.05, abs=1e-7)
    assert float(m.group(5)) == pytest.approx(30.0, abs=1e-5)
    # A camera is scene state like any other: it goes, and stays gone.
    scene.remove_camera("env/World/Overview")
    assert scene.camera_names == []
