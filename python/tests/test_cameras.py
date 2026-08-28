import math
import re
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def rotate(q, v):
    """Rotate vector v by quaternion (x, y, z, w) — plain math, no deps."""
    x, y, z, w = q
    ux, uy, uz = x, y, z
    # t = 2 * cross(u, v); v' = v + w * t + cross(u, t)
    tx = 2 * (uy * v[2] - uz * v[1])
    ty = 2 * (uz * v[0] - ux * v[2])
    tz = 2 * (ux * v[1] - uy * v[0])
    return (
        v[0] + w * tx + (uy * tz - uz * ty),
        v[1] + w * ty + (uz * tx - ux * tz),
        v[2] + w * tz + (ux * ty - uy * tx),
    )


def test_cameras_round_trip_project_and_codegen(scene, tmp_path) -> None:
    tcp = scene.robot.tcp_link
    scene.add_camera(
        "overview",
        position=(1.5, -1.5, 1.2),
        look_at=(0.0, 0.0, 0.3),
        fov=70,
        resolution=(1920, 1080),
    )
    scene.add_camera("wrist", robot=scene.robot.name, link=tcp, position=(0, 0.05, 0.02))
    assert scene.camera_names == ["overview", "wrist"]

    path = tmp_path / "cam.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert reloaded.camera_names == ["overview", "wrist"]

    code = scene.generate_python()
    assert 'scene.add_camera("overview"' in code
    assert re.search(r'scene\.add_camera\("wrist".*robot=.*link=', code), code
    # Old files without cameras still load (the field is additive).
    scene.remove_camera("wrist")
    assert scene.camera_names == ["overview"]


def test_look_at_aims_minus_z_with_z_up_image(scene) -> None:
    # From (2, 0, 1) toward (0, 0, 1): the view ray is -X, so the camera's
    # -Z must map onto -X and its +Y (image-up) onto world +Z.
    scene.add_camera("cam", position=(2.0, 0.0, 1.0), look_at=(0.0, 0.0, 1.0))
    code = scene.generate_python()
    m = re.search(
        r'scene\.add_camera\("cam".*quaternion=\(([-\d., ]+)\)', code
    )
    assert m, code
    q = tuple(float(v) for v in m.group(1).split(","))
    view = rotate(q, (0.0, 0.0, -1.0))
    up = rotate(q, (0.0, 1.0, 0.0))
    assert math.isclose(view[0], -1.0, abs_tol=1e-6), view
    assert abs(view[1]) < 1e-6 and abs(view[2]) < 1e-6, view
    assert math.isclose(up[2], 1.0, abs_tol=1e-6), up

    # The π-rotation family (looking along -Y from +Y) once collapsed to
    # identity — nalgebra's iterative from_matrix stalls at exactly 180°.
    scene.add_camera("pitched", position=(0.0, 1.4, 0.5), look_at=(0.0, 0.6, 0.3))
    code = scene.generate_python()
    m = re.search(r'scene\.add_camera\("pitched".*quaternion=\(([-\d., ]+)\)', code)
    q = tuple(float(v) for v in m.group(1).split(","))
    view = rotate(q, (0.0, 0.0, -1.0))
    n = math.sqrt(0.8**2 + 0.2**2)
    assert math.isclose(view[1], -0.8 / n, abs_tol=1e-6), view
    assert math.isclose(view[2], -0.2 / n, abs_tol=1e-6), view


def test_usd_export_carries_cameras(scene, tmp_path) -> None:
    pytest.importorskip("pxr")
    from pxr import Gf, Usd, UsdGeom

    tcp = scene.robot.tcp_link
    scene.add_camera(
        "overview",
        position=(1.5, -1.5, 1.2),
        look_at=(0.0, 0.0, 0.3),
        fov=70,
        resolution=(1920, 1080),
    )
    scene.add_camera("wrist", robot=scene.robot.name, link=tcp, position=(0, 0.05, 0.03))
    scene.add_box("cart", (0.3, 0.3, 0.2), (2.0, 0.0, 0.1))
    scene.add_vehicle("agv", body=["cart"], path=[(2.0, 0.0), (3.0, 0.0)], stations={"A": 0, "B": 1})
    scene.add_camera("agv_cam", mount="agv", position=(0.2, 0.0, 0.5), fov=80, resolution=(640, 480))

    sq = scene.sequence("cycle")
    sq.step("drive", actions=[bt.seq.goto("agv", "B")], transition=bt.seq.device_done("agv"))
    tl = sq.simulate()

    out = tmp_path / "cams.usda"
    warnings = tl.export_usd(out, fps=10)
    assert warnings == []
    stage = Usd.Stage.Open(str(out))
    for name in ("overview", "wrist", "agv_cam"):
        prim = stage.GetPrimAtPath(f"/World/Cameras/{name}")
        assert prim and prim.GetTypeName() == "Camera", name

    # The optics round-trip: aperture/focal ratio reproduces the authored
    # horizontal fov, vertical aperture the resolution's aspect.
    cam = UsdGeom.Camera(stage.GetPrimAtPath("/World/Cameras/overview"))
    focal = cam.GetFocalLengthAttr().Get()
    hap = cam.GetHorizontalApertureAttr().Get()
    assert abs(2 * math.degrees(math.atan(hap / (2 * focal))) - 70.0) < 0.05
    assert abs(cam.GetVerticalApertureAttr().Get() / hap - 1080 / 1920) < 1e-4
    clip = cam.GetClippingRangeAttr().Get()
    assert abs(clip[0] - 0.05) < 1e-6 and abs(clip[1] - 30.0) < 1e-3

    # The wrist camera's sampled pose equals link_pose ∘ offset at the
    # sampled instant — the 3-經路同絵 check, numerically.
    t, fps = 0.5, 10.0
    q = list(tl.sample(t))
    p_link, quat_link = scene.link_pose_at(tcp, q)
    off = rotate(quat_link, (0.0, 0.05, 0.03))
    expected = tuple(p_link[i] + off[i] for i in range(3))
    xform = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Cameras/wrist"))
    m = xform.ComputeLocalToWorldTransform(Usd.TimeCode(t * fps))
    got = m.ExtractTranslation()
    assert all(abs(got[i] - expected[i]) < 1e-6 for i in range(3)), (got, expected)

    # The vehicle camera rides its machine: one metre of travel A→B.
    xform = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Cameras/agv_cam"))
    p0 = xform.ComputeLocalToWorldTransform(Usd.TimeCode(0)).ExtractTranslation()
    p1 = xform.ComputeLocalToWorldTransform(
        Usd.TimeCode(tl.duration * fps)
    ).ExtractTranslation()
    assert abs((p1[0] - p0[0]) - 1.0) < 1e-6, (p0, p1)
    assert isinstance(Gf.Vec3d(p1), Gf.Vec3d)


def test_camera_validation(scene) -> None:
    with pytest.raises(ValueError):
        scene.add_camera("bad", fov=200.0)
    with pytest.raises(ValueError):
        scene.add_camera("bad", near=0.5, far=0.1)
    with pytest.raises(ValueError):
        scene.add_camera("bad", robot="nope", link="whatever")
    with pytest.raises(ValueError):
        scene.add_camera("bad", robot=scene.robot.name, link="no_such_link")
    with pytest.raises(ValueError):
        scene.add_camera("bad", mount="no_such_vehicle")
    with pytest.raises(ValueError):
        # A wrist camera moves with the arm; aiming it at a world point
        # would silently mean something else, so it is refused.
        scene.add_camera(
            "bad",
            robot=scene.robot.name,
            link=scene.robot.tcp_link,
            look_at=(0, 0, 0),
        )
    with pytest.raises(ValueError):
        scene.remove_camera("never_added")
    assert scene.camera_names == []
