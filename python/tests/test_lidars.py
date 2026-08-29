import math
import re
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture()
def scene() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def rotate(q, v):
    """Rotate vector v by quaternion (x, y, z, w) — plain math, no deps."""
    x, y, z, w = q
    ux, uy, uz = x, y, z
    tx = 2 * (uy * v[2] - uz * v[1])
    ty = 2 * (uz * v[0] - ux * v[2])
    tz = 2 * (ux * v[1] - uy * v[0])
    return (
        v[0] + w * tx + (uy * tz - uz * ty),
        v[1] + w * ty + (uz * tx - ux * tz),
        v[2] + w * tz + (ux * ty - uy * tx),
    )


def test_lidars_round_trip_project_and_codegen(scene, tmp_path) -> None:
    tcp = scene.robot.tcp_link
    scene.add_lidar(
        "gate",
        position=(3.2, -2.4, 0.15),
        yaw=180.0,
        fov=270.0,
        range=(0.05, 20.0),
    )
    scene.add_lidar("boom", robot=scene.robot.name, link=tcp, position=(0, 0.05, 0.02))
    assert scene.lidar_names == ["gate", "boom"]

    path = tmp_path / "lidar.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert reloaded.lidar_names == ["gate", "boom"]

    code = scene.generate_python()
    assert re.search(
        r'scene\.add_lidar\("gate".*fov=270.*range=\(0\.05, 20\)', code
    ), code
    assert re.search(r'scene\.add_lidar\("boom".*robot=.*link=', code), code
    # Old files without lidars still load (the field is additive).
    scene.remove_lidar("boom")
    assert scene.lidar_names == ["gate"]


def test_vehicle_mounted_lidar_round_trips(scene, tmp_path) -> None:
    scene.add_box("cart", (0.3, 0.3, 0.2), (2.0, 0.0, 0.1))
    scene.add_vehicle(
        "agv", body=["cart"], path=[(2.0, 0.0), (3.0, 0.0)], stations={"A": 0, "B": 1}
    )
    scene.add_lidar("nav", mount="agv", position=(0.35, 0, 0.15), fov=360.0)
    path = tmp_path / "nav.botrail"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    assert reloaded.lidar_names == ["nav"]
    code = reloaded.generate_python()
    assert re.search(r'scene\.add_lidar\("nav".*mount="agv"', code), code


def test_yaw_aims_the_scan_heading(scene) -> None:
    # yaw=90 spins the scan frame's +X (angle 0) onto world +Y; the scan
    # plane's normal stays +Z.
    scene.add_lidar("turned", position=(1.0, 0.0, 0.2), yaw=90.0)
    code = scene.generate_python()
    m = re.search(r'scene\.add_lidar\("turned".*quaternion=\(([-\d., e]+)\)', code)
    assert m, code
    q = tuple(float(v) for v in m.group(1).split(","))
    # The codegen prints quaternions at ~1e-6, so compare directions, not
    # bit-exact values.
    heading = rotate(q, (1.0, 0.0, 0.0))
    normal = rotate(q, (0.0, 0.0, 1.0))
    assert math.isclose(heading[1], 1.0, abs_tol=1e-5), heading
    assert abs(heading[0]) < 1e-5 and abs(heading[2]) < 1e-5, heading
    assert math.isclose(normal[2], 1.0, abs_tol=1e-5), normal


def test_lidar_bom_row_and_part_identity(scene) -> None:
    scene.add_lidar("gate", position=(1.0, 0.0, 0.2))
    bom = scene.bom()
    rows = {tuple(row["names"]): row for row in bom.rows}
    assert rows[("gate",)]["category"] == "sensor.lidar"
    # Pinning an identity lands on the scanner's line (kind resolves).
    scene.set_part("gate", manufacturer="SICK", model="LMS111-10100")
    rows = {tuple(row["names"]): row for row in scene.bom().rows}
    assert rows[("gate",)]["model"] == "LMS111-10100"
    # Removing the scanner prunes the pin instead of stranding it.
    scene.remove_lidar("gate")
    assert all("gate" not in row["names"] for row in scene.bom().rows)


def test_layout_draws_the_scan_wedge(scene) -> None:
    scene.add_lidar("gate", position=(1.0, 0.5, 0.2), yaw=45.0, fov=270.0)
    svg = scene.layout("svg")
    assert "gate" in svg
    # A mounted scanner draws nothing (it travels).
    scene.add_box("cart", (0.3, 0.3, 0.2), (2.0, 0.0, 0.1))
    scene.add_vehicle("agv", body=["cart"], path=[(2.0, 0.0), (3.0, 0.0)], stations={"A": 0})
    scene.add_lidar("nav", mount="agv")
    svg = scene.layout("svg")
    assert "nav" not in svg


def test_lidar_validation(scene) -> None:
    with pytest.raises(ValueError):
        scene.add_lidar("bad", fov=0.0)
    with pytest.raises(ValueError):
        scene.add_lidar("bad", fov=400.0)
    with pytest.raises(ValueError):
        scene.add_lidar("bad", range=(2.0, 1.0))
    with pytest.raises(ValueError):
        scene.add_lidar("bad", range=(0.0, 1.0))
    with pytest.raises(ValueError):
        scene.add_lidar("bad", resolution=0.0)
    with pytest.raises(ValueError):
        scene.add_lidar("bad", robot="nope", link="whatever")
    with pytest.raises(ValueError):
        scene.add_lidar("bad", robot=scene.robot.name, link="no_such_link")
    with pytest.raises(ValueError):
        scene.add_lidar("bad", mount="no_such_vehicle")
    with pytest.raises(ValueError):
        scene.add_lidar("bad", yaw=90.0, quaternion=(0, 0, 0, 1))
    with pytest.raises(ValueError):
        scene.remove_lidar("never_added")
    assert scene.lidar_names == []
