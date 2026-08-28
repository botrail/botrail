"""Mesh collision (Phase 2): VHACD obstacles, mesh robot links, projects."""

import struct
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


def write_box_stl(path: Path, size: tuple[float, float, float]) -> None:
    hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
    v = [
        (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
        (-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz),
    ]
    tris = [
        (0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
        (0, 1, 5), (0, 5, 4), (2, 3, 7), (2, 7, 6),
        (1, 2, 6), (1, 6, 5), (3, 0, 4), (3, 4, 7),
    ]
    with path.open("wb") as f:
        f.write(b"\0" * 80)
        f.write(struct.pack("<I", len(tris)))
        for a, b, c in tris:
            f.write(struct.pack("<3f", 0.0, 0.0, 0.0))
            for i in (a, b, c):
                f.write(struct.pack("<3f", *v[i]))
            f.write(struct.pack("<H", 0))


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def test_mesh_obstacle_collides_and_clears(scene: bt.Scene, tmp_path: Path) -> None:
    stl = tmp_path / "box.stl"
    write_box_stl(stl, (0.3, 0.3, 0.3))

    name = scene.add_mesh("crate", stl, position=(0.0, 0.0, 0.5))
    assert name == "crate"
    assert "crate" in scene.obstacle_names
    assert scene.in_collision()

    scene.set_obstacle_pose("crate", (2.0, 0.0, 0.0))
    assert not scene.in_collision()
    assert scene.min_obstacle_distance() > 1.0


def test_mesh_scale_is_applied(scene: bt.Scene, tmp_path: Path) -> None:
    stl = tmp_path / "unit.stl"
    write_box_stl(stl, (0.1, 0.1, 0.1))

    # Unscaled: a 10cm box 30cm to the side does not reach the arm.
    scene.add_mesh("small", stl, position=(0.3, 0.0, 0.4))
    assert not scene.in_collision()
    scene.remove_obstacle("small")

    # Scaled 6x, the same mesh does.
    scene.add_mesh("big", stl, position=(0.3, 0.0, 0.4), scale=(6.0, 6.0, 6.0))
    assert scene.in_collision()


def test_plan_avoids_mesh_obstacle(scene: bt.Scene, tmp_path: Path) -> None:
    stl = tmp_path / "wall.stl"
    write_box_stl(stl, (0.05, 0.8, 0.5))
    scene.add_mesh("wall", stl, position=(0.28, 0.0, 0.45))

    traj = scene.plan([1.2, 0.9, -1.5, 0.5, 0.0, 0.0], broadcast=False)
    assert traj.duration > 0.0
    # Sampled states stay collision-free.
    t = 0.0
    while t <= traj.duration:
        scene.set_joint_positions(traj.sample(t))
        assert not scene.in_collision(), f"collision at t={t:.2f}"
        t += 0.05


def test_mesh_robot_links_get_collision_shapes(tmp_path: Path) -> None:
    stl = tmp_path / "link.stl"
    write_box_stl(stl, (0.1, 0.1, 0.4))
    urdf = f"""<robot name="meshbot">
      <link name="base">
        <visual><origin xyz="0 0 0"/><geometry><mesh filename="{stl}"/></geometry></visual>
      </link>
      <link name="arm">
        <visual><origin xyz="0 0 0.2"/><geometry><mesh filename="{stl}"/></geometry></visual>
      </link>
      <joint name="j" type="revolute">
        <parent link="base"/><child link="arm"/>
        <origin xyz="0 0 0.4"/><axis xyz="0 1 0"/>
        <limit lower="-3" upper="3" effort="1" velocity="1"/>
      </joint>
    </robot>"""
    scene = bt.Scene(bt.Robot.from_urdf_string(urdf))
    # Mesh links now carry real collision shapes: no skip warnings.
    assert scene.collision_warnings == []

    # A box crossing the arm link's mesh (z 0.4..0.8 at q=0) collides.
    scene.add_box("hit", size=(0.2, 0.2, 0.2), position=(0.0, 0.0, 0.6))
    assert scene.in_collision()
    scene.set_obstacle_pose("hit", (1.0, 0.0, 0.6))
    assert not scene.in_collision()


def test_project_roundtrip_with_mesh_obstacle(scene: bt.Scene, tmp_path: Path) -> None:
    stl = tmp_path / "fixture.stl"
    write_box_stl(stl, (0.2, 0.2, 0.2))
    scene.add_mesh("fixture", stl, position=(0.0, 0.0, 0.5))
    assert scene.in_collision()

    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    reloaded = bt.Scene.load_project(project)
    assert reloaded.obstacle_names == ["fixture"]
    assert reloaded.in_collision()

    code = reloaded.generate_python()
    assert "scene.add_mesh(\"fixture\"" in code
    assert "fixture.stl" in code


def test_zip_project_is_portable(scene: bt.Scene, tmp_path: Path) -> None:
    """Bundled mesh assets survive deletion of the original file."""
    stl = tmp_path / "crate.stl"
    write_box_stl(stl, (0.3, 0.3, 0.3))
    scene.add_mesh("crate", stl, position=(0.0, 0.0, 0.5))

    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    # Mesh projects are zip archives now.
    assert project.read_bytes()[:2] == b"PK"

    stl.unlink()  # the original mesh is gone...
    reloaded = bt.Scene.load_project(project)  # ...but the bundle has a copy
    assert reloaded.obstacle_names == ["crate"]
    assert reloaded.in_collision()
