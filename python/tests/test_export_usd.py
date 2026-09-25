from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def test_export_usd_bakes_robot_and_grasped_object(scene: bt.Scene, tmp_path: Path) -> None:
    tcp, _ = scene.link_pose(scene.robot.tcp_link)
    scene.add_box("held", (0.04, 0.04, 0.04), (tcp[0], tcp[1], tcp[2] + 0.06))
    scene.add_box("/World/Shelf/Board", (0.3, 0.3, 0.02), (0.6, 0.0, 0.4))
    scene.attach("held")

    traj = scene.plan([0.4, -0.3, 0.3, 0.0, 0.2, 0.0])
    out = tmp_path / "anim.usda"
    warnings = scene.export_usd(out, traj, fps=30.0)
    assert warnings == []

    text = out.read_text()
    assert text.startswith("#usda")
    assert 'upAxis = "Z"' in text
    assert "timeSamples" in text
    # URDF robots are authored self-contained: link prims + no asset dir.
    assert "wrist_3_link" in text
    assert not (tmp_path / "anim_assets").exists()
    # The grasped box gets a sampled track; the static shelf a nested prim.
    assert "held" in text
    assert "Shelf" in text


def test_export_usd_rejects_bad_fps(scene: bt.Scene, tmp_path: Path) -> None:
    traj = scene.plan([0.2, 0.0, 0.0, 0.0, 0.0, 0.0])
    with pytest.raises(ValueError):
        scene.export_usd(tmp_path / "x.usda", traj, fps=0.0)


def test_export_usd_static_writes_the_cell(scene: bt.Scene, tmp_path: Path) -> None:
    scene.add_box("/World/Shelf/Board", (0.3, 0.3, 0.02), (0.6, 0.0, 0.4), color=(0.2, 0.4, 0.6))
    scene.add_box("proxy", (0.1, 0.1, 0.1), (0.9, 0.0, 0.1))
    scene.set_obstacle_visible("proxy", False)

    out = tmp_path / "cell.usda"
    assert scene.export_usd(out) == []
    text = out.read_text()
    assert text.startswith("#usda")
    # The cell as it stands: the robot at its pose, the visible obstacle
    # as a prim — and hidden means hidden.
    assert "wrist_3_link" in text
    assert "Shelf" in text
    assert "proxy" not in text


def test_export_usd_static_round_trips_obstacles(tmp_path: Path) -> None:
    # A robot-less scene — a layout — writes a static layer that reads back.
    src = bt.Scene()
    src.add_box("bench/top", (0.8, 0.4, 0.03), (0.5, 0.0, 0.7), color=(0.5, 0.3, 0.1))
    src.add_cylinder("bench/leg", radius=0.03, length=0.7, position=(0.5, 0.0, 0.35))
    out = tmp_path / "layout.usda"
    assert src.export_usd(out) == []

    back = bt.Scene()
    back.load_usd(out)
    assert sorted(back.obstacle_names) == [
        "/World/Env/bench/leg",
        "/World/Env/bench/top",
    ]
    pos, _ = back.obstacle_pose("/World/Env/bench/top")
    assert pos == pytest.approx((0.5, 0.0, 0.7))


def box_obj(path: Path) -> None:
    """A 0.1 m cube as an OBJ, with an mtllib painting its faces two colours."""
    (path.with_suffix(".mtl")).write_text("newmtl red\nKd 0.9 0.1 0.1\nnewmtl blue\nKd 0.1 0.1 0.9\n")
    corners = [(x, y, z) for z in (0.0, 0.1) for y in (-0.05, 0.05) for x in (-0.05, 0.05)]
    faces = [(1, 3, 2), (2, 3, 4), (5, 6, 7), (6, 8, 7), (1, 2, 5), (2, 6, 5),
             (3, 7, 4), (4, 7, 8), (1, 5, 3), (3, 5, 7), (2, 4, 6), (4, 8, 6)]
    lines = [f"mtllib {path.with_suffix('.mtl').name}"] + [f"v {x} {y} {z}" for x, y, z in corners]
    lines += ["usemtl red"] + [f"f {a} {b} {c}" for a, b, c in faces[:6]]
    lines += ["usemtl blue"] + [f"f {a} {b} {c}" for a, b, c in faces[6:]]
    path.write_text("\n".join(lines) + "\n")


def test_meshes_are_written_once_beside_the_layer_and_compose(tmp_path: Path) -> None:
    """A mesh file drawn by several obstacles goes out once, as a binary
    layer under `<stem>_assets/meshes/`, referenced from each prim with its
    own pose and colour (design-rl-tabletop.md G6): the layer stays small,
    a USD reader composes the triangles, and the export reads back."""
    obj = tmp_path / "part.obj"
    box_obj(obj)
    src = bt.Scene()
    for i, color in enumerate([(0.8, 0.2, 0.1), None, (0.1, 0.6, 0.2)]):
        src.add_mesh(f"stock/part{i}", obj, position=(0.3 * i, 0.0, 0.7), color=color)
    out = tmp_path / "cell.usda"
    assert src.export_usd(out) == []
    layers = sorted(p.name for p in (tmp_path / "cell_assets" / "meshes").iterdir())
    assert layers == ["part0.usdc"]
    text = out.read_text()
    assert text.count("./cell_assets/meshes/part0.usdc") == 3
    assert "faceVertexIndices" not in text
    assert out.stat().st_size < 20_000

    back = bt.Scene()
    back.load_usd(out)
    for i in range(3):
        lo, hi = back.obstacle_bounds(f"/World/Env/stock/part{i}")
        assert (lo[0], hi[0], lo[2], hi[2]) == pytest.approx((0.3 * i - 0.05, 0.3 * i + 0.05, 0.7, 0.8), abs=1e-6)

    Usd = pytest.importorskip("pxr.Usd")
    from pxr import UsdGeom

    stage = Usd.Stage.Open(str(out))
    painted = UsdGeom.Mesh(stage.GetPrimAtPath("/World/Env/stock/part0"))
    assert len(painted.GetPointsAttr().Get()) == 8
    assert len(painted.GetFaceVertexIndicesAttr().Get()) == 36
    # The scene's colour is one constant over the file's own faces; the
    # obstacle that asked for none shows the mtllib's two.
    assert list(painted.GetDisplayColorAttr().Get()) == pytest.approx([(0.8, 0.2, 0.1)])
    assert painted.GetDisplayColorPrimvar().GetInterpolation() == "constant"
    own = UsdGeom.Mesh(stage.GetPrimAtPath("/World/Env/stock/part1"))
    assert len(own.GetDisplayColorAttr().Get()) == 12
    assert own.GetDisplayColorPrimvar().GetInterpolation() == "uniform"
    try:
        from pxr import UsdValidation
    except ImportError:
        return
    context = UsdValidation.ValidationContext(UsdValidation.ValidationRegistry().GetOrLoadAllValidators())
    assert [e.GetMessage() for e in context.Validate(stage)] == []


def test_export_usd_static_rejects_robot_selector(scene: bt.Scene, tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="trajectory"):
        scene.export_usd(tmp_path / "x.usda", robot="arm")
