"""Simulated laser sweeps: the collider truth, headless and deterministic.

The acceptance numbers the design asks for — a facing wall at 1.500 m
±1 mm, the angle convention told apart by pillars on the axes, bit-equal
repeat calls, the measuring band's invalid returns — plus the shadow
window behind a pallet, the mount resolutions with their own-body
exclusions, and the PLY round-trip.
"""

import math
import struct
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


def read_ply(path: Path) -> list[tuple[float, float, float]]:
    data = path.read_bytes()
    header, _, body = data.partition(b"end_header\n")
    assert b"binary_little_endian" in header
    count = int(next(line.split()[-1] for line in header.split(b"\n") if line.startswith(b"element vertex")))
    points = [struct.unpack_from("<3f", body, i * 12) for i in range(count)]
    assert len(body) == count * 12
    return points


def test_facing_wall_range_and_angle_convention() -> None:
    scene = bt.Scene()
    # A wall facing the scanner down +X, pillars on +X / +Y / -Y.
    scene.add_box("wall", (0.1, 4.0, 1.0), (3.05, 0.0, 0.5))
    scene.add_box("post_ahead", (0.1, 0.1, 1.0), (1.0, 0.0, 0.5))
    scene.add_box("post_left", (0.1, 0.1, 1.0), (0.0, 1.0, 0.5))
    scene.add_box("post_right", (0.1, 0.1, 1.0), (0.0, -1.0, 0.5))
    scene.add_lidar("scan", position=(0.0, 0.0, 0.2), fov=270.0, range=(0.05, 5.0))
    frame = scene.lidar_scan("scan")

    beams = {a: (r, h) for a, r, h in zip(frame.angles, frame.ranges, frame.hits)}
    # Angle 0 = +X: the near post face at 0.95 m, ±1 mm.
    r, h = beams[0.0]
    assert h == "post_ahead" and abs(r - 0.95) < 1e-3, (r, h)
    # CCW toward +Y: +90° sees the left post, -90° the right one.
    assert beams[90.0][1] == "post_left" and abs(beams[90.0][0] - 0.95) < 1e-3
    assert beams[-90.0][1] == "post_right" and abs(beams[-90.0][0] - 0.95) < 1e-3
    # A beam past the posts reaches the wall: 3.0 m / cos(20°).
    r, h = beams[20.0]
    assert h == "wall" and abs(r - 3.0 / math.cos(math.radians(20.0))) < 1e-3, (r, h)
    # The sweep spans exactly [-135, 135] at 0.5°.
    assert frame.angles[0] == -135.0 and frame.angles[-1] == 135.0
    assert len(frame.angles) == 541

    # Deterministic: a second sweep is bit-identical.
    again = scene.lidar_scan("scan")
    assert again.ranges == frame.ranges and again.hits == frame.hits


def test_measuring_band_invalidates_returns() -> None:
    scene = bt.Scene()
    # A plate hugging the scanner (inside min range) ahead, open space
    # behind (beyond max range there is nothing at all).
    scene.add_box("plate", (0.02, 0.5, 0.5), (0.03, 0.0, 0.2))
    scene.add_lidar("scan", position=(0.0, 0.0, 0.2), fov=270.0, range=(0.05, 2.0))
    frame = scene.lidar_scan("scan")
    beams = {a: (r, h) for a, r, h in zip(frame.angles, frame.ranges, frame.hits)}
    # The plate face sits at 0.02 m — inside the blind ring: no return.
    assert beams[0.0] == (0.0, None)
    # Nothing within 2 m the other way either.
    assert beams[130.0] == (0.0, None)


def test_shadow_window_behind_a_pallet() -> None:
    # The design demo: the pallet carves its shadow out of the wall.
    scene = bt.Scene()
    scene.add_box("wall", (0.1, 4.0, 1.0), (3.05, 0.0, 0.5))
    scene.add_box("pallet", (0.4, 0.4, 0.4), (1.5, 0.0, 0.2))
    scene.add_lidar("gate", position=(0.0, 0.0, 0.2), fov=90.0, range=(0.05, 5.0))
    frame = scene.lidar_scan("gate")
    beams = list(zip(frame.angles, frame.ranges, frame.hits))
    # Head-on: the pallet face at 1.3 m, not the wall.
    center = next((r, h) for a, r, h in beams if a == 0.0)
    assert center[1] == "pallet" and abs(center[0] - 1.3) < 1e-3
    # The shadow's angular half-width is atan(0.2 / 1.3) ≈ 8.75°: every
    # pallet return lies inside it, and the beams just outside see the
    # wall again.
    half = math.degrees(math.atan2(0.2, 1.3))
    pallet_angles = [a for a, _, h in beams if h == "pallet"]
    assert pallet_angles and max(abs(a) for a in pallet_angles) <= half + 0.5
    # (Restricted to |a| ≤ 30°: past atan(2/3) ≈ 33.7° the beams leave
    # the 4 m wall's edge entirely.)
    assert all(h == "wall" for a, _, h in beams if half + 0.5 < abs(a) <= 30.0), beams[:5]


def test_robot_links_are_seen() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_lidar("watch", position=(1.2, 0.0, 0.5), yaw=180.0, fov=90.0, range=(0.05, 3.0))
    frame = scene.lidar_scan("watch")
    hits = {h for h in frame.hits if h}
    assert any(h.startswith("simple_arm/") for h in hits), hits


def test_vehicle_scan_parks_and_ignores_own_body() -> None:
    scene = bt.Scene()
    scene.add_box("chassis", (0.5, 0.35, 0.25), (0.0, 2.0, 0.125))
    scene.add_vehicle(
        "agv", body=["chassis"], path=[(0.0, 2.0), (2.0, 2.0)], stations={"A": 0, "B": 1}
    )
    scene.add_box("post", (0.1, 0.1, 0.6), (1.0, 2.0, 0.3))
    scene.add_lidar("nav", mount="agv", position=(0.3, 0.0, 0.2), fov=360.0, range=(0.05, 1.2))
    frame = scene.lidar_scan("nav")
    # Parked at A heading +X: the scanner sweeps from (0.3, 2.0, 0.2).
    assert all(abs(p - e) < 1e-9 for p, e in zip(frame.position, (0.3, 2.0, 0.2)))
    beams = {a: (r, h) for a, r, h in zip(frame.angles, frame.ranges, frame.hits)}
    r, h = beams[0.0]
    assert h == "post" and abs(r - 0.65) < 1e-3, (r, h)
    # The massing chassis crosses the sweep from inside — excluded, so
    # the rear beams see nothing rather than their own machine.
    assert "chassis" not in {h for h in frame.hits if h}
    # A full circle has no duplicate closing beam.
    assert len(frame.angles) == 720
    assert frame.angles[0] == -180.0 and frame.angles[-1] == 179.5


def test_link_scan_rides_the_arm() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    joints = [0.4, -0.5, 0.8, 0.0, 0.6, 0.0]
    scene.set_joint_positions(joints)
    tcp = scene.robot.tcp_link
    scene.add_lidar("wrist", robot=scene.robot.name, link=tcp, position=(0, 0, 0.05), fov=180.0)
    frame = scene.lidar_scan("wrist")
    p, q = scene.link_pose_at(tcp, joints)
    # The sweep origin is the link pose ∘ offset (offset is along link z).
    x, y, z, w = q
    off = (
        2 * (x * z + w * y) * 0.05,
        2 * (y * z - w * x) * 0.05,
        (1 - 2 * (x * x + y * y)) * 0.05,
    )
    expected = tuple(p[i] + off[i] for i in range(3))
    assert all(abs(a - b) < 1e-9 for a, b in zip(frame.position, expected))
    # The mount link never blinds the sweep from inside: every return is
    # a real distance, none pinned at zero-by-contact.
    assert all(r == 0.0 or r >= 0.05 for r in frame.ranges)


def test_points_and_ply_round_trip(tmp_path) -> None:
    scene = bt.Scene()
    scene.add_box("wall", (0.1, 4.0, 1.0), (3.05, 1.0, 0.5))
    scene.add_lidar("scan", position=(1.5, 1.0, 0.2), fov=90.0, range=(0.05, 5.0))
    frame = scene.lidar_scan("scan")
    points = frame.points()
    valid = sum(1 for r in frame.ranges if r > 0)
    assert len(points) == valid > 0
    # Every return lies on the wall face plane x = 3.0, in the scan plane
    # z = 0.2 — the world transform, the angle grid and the range all
    # have to agree for this to hold.
    assert all(abs(x - 3.0) < 1e-6 and abs(z - 0.2) < 1e-9 for x, _, z in points)
    out = tmp_path / "scan.ply"
    frame.save_ply(out)
    parsed = read_ply(out)
    assert len(parsed) == valid
    assert all(abs(px - qx) < 1e-4 for (px, _, _), (qx, _, _) in zip(parsed, points))
    # Scan-frame points put the wall dead ahead instead.
    local = frame.points(world=False)
    assert all(abs(x - 1.5) < 1e-6 and abs(z) < 1e-9 for x, _, z in local)


def test_scan_validation() -> None:
    scene = bt.Scene()
    with pytest.raises(ValueError):
        scene.lidar_scan("nope")


# ------------------------------------------------- timeline scans (L3)


def corridor_cell(walls: bool = False) -> bt.Scene:
    """An AGV with a nav scanner driving 2 m toward an end wall."""
    scene = bt.Scene()
    scene.add_box("chassis", (0.5, 0.35, 0.25), (0.0, 2.0, 0.125))
    scene.add_vehicle(
        "agv", body=["chassis"], path=[(0.0, 2.0), (2.0, 2.0)], stations={"A": 0, "B": 1}
    )
    scene.add_box("wall", (0.1, 0.8, 0.6), (2.8, 2.0, 0.3))
    if walls:
        scene.add_box("left", (3.0, 0.1, 0.5), (1.0, 2.7, 0.25))
        scene.add_box("right", (3.0, 0.1, 0.5), (1.0, 1.3, 0.25))
    scene.add_lidar("nav", mount="agv", position=(0.3, 0.0, 0.2), fov=360.0, range=(0.05, 3.0))
    sq = scene.sequence("drive")
    sq.step("go", actions=[bt.seq.goto("agv", "B")], transition=bt.seq.device_done("agv"))
    return scene


def test_timeline_scan_follows_the_drive() -> None:
    scene = corridor_cell()
    tl = scene.simulate_sequence("drive")
    # Parked: the wall face (x = 2.75) is 2.45 m from the scanner (0.3).
    ahead = lambda f: dict(zip(f.angles, f.ranges))[0.0]
    start = scene.lidar_scan("nav", t=0.0)
    assert abs(ahead(start) - 2.45) < 2e-3 and start.t == 0.0
    # Docked at B: 0.45 m.
    end = scene.lidar_scan("nav", t=tl.duration)
    assert abs(ahead(end) - 0.45) < 2e-3
    # A single straight leg drives at constant speed: mid-time ≈ mid-way.
    mid = scene.lidar_scan("nav", t=tl.duration / 2)
    assert abs(ahead(mid) - 1.45) < 0.05, ahead(mid)
    # t past the end clamps to the docked state.
    clamped = scene.lidar_scan("nav", t=1e9)
    assert clamped.ranges == end.ranges and clamped.t == tl.duration


def test_timeline_scan_requires_a_bake() -> None:
    scene = corridor_cell()
    with pytest.raises(ValueError, match="simulate"):
        scene.lidar_scan("nav", t=1.0)
    # Without t the authored scene needs no bake.
    assert scene.lidar_scan("nav").t is None


def test_timeline_scan_sees_conveyed_objects() -> None:
    # A crate rides a belt toward a fixture scanner: the beam shortens by
    # exactly the belt travel.
    scene = bt.Scene()
    scene.add_box("crate", (0.1, 0.1, 0.1), (-1.0, 0.6, 0.3))
    scene.add_conveyor(
        "belt",
        zone_position=(-0.2, 0.6, 0.3),
        zone_size=(2.4, 0.3, 0.3),
        velocity=(0.25, 0.0, 0.0),
        running=False,
    )
    scene.add_lidar("gate", position=(1.5, 0.6, 0.3), yaw=180.0, fov=90.0, range=(0.05, 5.0))
    sq = scene.sequence("feed")
    sq.step("run", actions=[bt.seq.start("belt")], transition=bt.seq.elapsed(4.0))
    sq.simulate()
    ahead = lambda f: dict(zip(f.angles, f.ranges))[0.0]
    assert abs(ahead(scene.lidar_scan("gate", t=0.0)) - 2.45) < 2e-3
    assert abs(ahead(scene.lidar_scan("gate", t=4.0)) - 1.45) < 2e-3


def test_timeline_scan_skips_stowed_stock() -> None:
    # A pool crate waiting in its magazine is stowed on the timeline: the
    # authored scene shows it, the baked instants do not.
    scene = bt.Scene()
    scene.add_box("stock", (0.1, 0.1, 0.1), (2.0, 0.0, 0.1))
    scene.add_source(
        "magazine", pool=["stock"], park=(2.0, 0.0, 0.1), position=(0.3, 0.3, 0.1)
    )
    scene.add_lidar("watch", position=(1.0, 0.0, 0.1), fov=90.0, range=(0.05, 5.0))
    sq = scene.sequence("idle")
    sq.step("hold", transition=bt.seq.elapsed(0.5))
    sq.simulate()
    assert "stock" in {h for h in scene.lidar_scan("watch").hits if h}
    assert "stock" not in {h for h in scene.lidar_scan("watch", t=0.2).hits if h}


def test_timeline_scan_rides_the_ramping_arm() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    tcp = scene.robot.tcp_link
    # Joint 0 is the base yaw — the zero-posed tcp sits on its axis, so
    # ramp the shoulder instead: the wrist actually travels.
    joint = scene.robot.joint_names[1]
    scene.add_lidar("wrist", robot=scene.robot.name, link=tcp, position=(0, 0, 0.05), fov=180.0)
    sq = scene.sequence("swing")
    sq.step("turn", [bt.seq.ramp({joint: 1.2}, 1.5)], transition=bt.seq.done())
    tl = sq.simulate()
    for t in (0.0, tl.duration):
        frame = scene.lidar_scan("wrist", t=t)
        p, q = scene.link_pose_at(tcp, list(tl.sample(t)))
        x, y, z, w = q
        off = (
            2 * (x * z + w * y) * 0.05,
            2 * (y * z - w * x) * 0.05,
            (1 - 2 * (x * x + y * y)) * 0.05,
        )
        expected = tuple(p[i] + off[i] for i in range(3))
        assert all(abs(a - b) < 1e-9 for a, b in zip(frame.position, expected)), t
    p0 = scene.lidar_scan("wrist", t=0.0).position
    p1 = scene.lidar_scan("wrist", t=tl.duration).position
    assert max(abs(a - b) for a, b in zip(p0, p1)) > 0.05


def test_scan_sweep_merges_the_corridor() -> None:
    scene = corridor_cell(walls=True)
    tl = scene.simulate_sequence("drive")
    frames = scene.scan_sweep("nav", fps=5.0)
    # The export grid: 1/fps steps plus the final instant.
    assert frames[0].t == 0.0 and frames[-1].t == pytest.approx(tl.duration)
    assert len(frames) == math.floor((tl.duration - 1e-9) * 5.0) + 2
    # The scanner advanced 2 m over the sweep.
    assert abs(frames[-1].position[0] - frames[0].position[0] - 2.0) < 1e-6
    # Merged, the drive surveys the corridor: the left wall's returns
    # (y ≈ 2.65) span far more of the aisle than one parked sweep sees.
    points = [p for f in frames for p in f.points()]
    left = [x for x, y, _ in points if abs(y - 2.65) < 1e-6]
    assert left and max(left) - min(left) > 2.5, (len(left), min(left, default=0), max(left, default=0))


def test_cli_scan_at_and_sweep(capsys, tmp_path) -> None:
    import json

    from botrail import _cli

    cell = tmp_path / "corridor.py"
    cell.write_text(
        "import botrail as bt\n"
        "scene = bt.Scene()\n"
        'scene.add_box("chassis", (0.5, 0.35, 0.25), (0.0, 2.0, 0.125))\n'
        'scene.add_vehicle("agv", body=["chassis"], path=[(0.0, 2.0), (2.0, 2.0)],'
        ' stations={"A": 0, "B": 1})\n'
        'scene.add_box("wall", (0.1, 0.8, 0.6), (2.8, 2.0, 0.3))\n'
        'scene.add_lidar("nav", mount="agv", position=(0.3, 0, 0.2), fov=360.0,'
        " range=(0.05, 3.0))\n"
        'sq = scene.sequence("drive")\n'
        'sq.step("go", actions=[bt.seq.goto("agv", "B")],'
        " transition=bt.seq.device_done(\"agv\"))\n"
    )
    # --at bakes and sweeps one instant.
    csv = tmp_path / "docked.csv"
    code = _cli.main(["scan", str(cell), "--at", "1e9", "--out", str(csv)])
    assert code == 0 and json.loads(capsys.readouterr().out)["ok"]
    row = next(line for line in csv.read_text().splitlines() if line.startswith("0.0000,"))
    assert abs(float(row.split(",")[1]) - 0.45) < 2e-3, row
    # --sweep merges every frame's cloud.
    ply = tmp_path / "sweep.ply"
    code = _cli.main(["scan", str(cell), "--sweep", "2", "--out", str(ply)])
    report = json.loads(capsys.readouterr().out)
    assert code == 0 and report["ok"] and report["frames"] > 3
    assert len(read_ply(ply)) == report["points"] > 0


def test_cli_scan_writes_ply_and_csv(capsys, tmp_path) -> None:
    import json

    from botrail import _cli

    cell = tmp_path / "cell.py"
    cell.write_text(
        "import botrail as bt\n"
        "scene = bt.Scene()\n"
        'scene.add_box("wall", (0.1, 4.0, 1.0), (3.05, 0.0, 0.5))\n'
        'scene.add_lidar("gate", position=(0, 0, 0.2), fov=90.0, range=(0.05, 5.0))\n'
    )
    ply = tmp_path / "scan.ply"
    code = _cli.main(["scan", str(cell), "--out", str(ply)])
    report = json.loads(capsys.readouterr().out)
    assert code == 0 and report["ok"] and report["lidar"] == "gate"
    assert report["returns"] > 0
    assert len(read_ply(ply)) == report["returns"]

    csv = tmp_path / "scan.csv"
    code = _cli.main(["scan", str(cell), "--out", str(csv)])
    capsys.readouterr()
    assert code == 0
    lines = csv.read_text().strip().splitlines()
    assert lines[0] == "angle_deg,range_m,hit"
    assert len(lines) - 1 == report["beams"]
    assert any(line.endswith(",wall") for line in lines[1:])
