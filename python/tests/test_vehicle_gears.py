"""A differential drive's gears (design-forklift.md F0): the gear a station
is entered in, the gear a right-angle corner takes, and the speed a
reversed leg runs at. A forklift's forks are its +X, so `arrive="forward"`
puts the forks into the pallet and `prefer="reverse"` drives the aisle
drive-unit first."""

from __future__ import annotations

import math

import pytest

import botrail as bt


def cell(**kwargs) -> bt.Scene:
    """A chassis on an L: 2 m along +x, then 1 m along +y, at 0.5 m/s with
    a 90°/s pivot — 4 + 1 + 2 = 7 s driven forward throughout."""
    scene = bt.Scene()
    scene.add_box("chassis", (0.2, 0.2, 0.2), (0.0, 0.0, 0.1))
    scene.add_vehicle("agv", body=["chassis"], path=[(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)],
                      stations={"a": 0, "c": 2}, speed=0.5, turn_speed=math.pi / 2, start="a", **kwargs)
    return scene


def drive(scene: bt.Scene, *stations: str) -> bt.SequenceTimeline:
    sq = scene.sequence("drive")
    for station in stations:
        sq.step(f"to_{station}", actions=[bt.seq.goto("agv", station)], transition=bt.seq.device_done("agv"))
    return sq.simulate()


def yaw_at(tl, t: float) -> float:
    _, (qx, qy, qz, qw) = tl.object_pose("chassis", t)
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))


def test_a_preferred_reverse_gear_backs_down_a_right_angle_at_its_speed() -> None:
    tl = drive(cell(allow_reverse=True, prefer="reverse", reverse_speed=1.0), "c")
    # 2 m forward at 0.5, a quarter turn, 1 m backed at 1.0.
    assert tl.duration == pytest.approx(6.0, abs=0.02)
    # Parked facing -y: it backed up the +y leg.
    assert yaw_at(tl, tl.duration) == pytest.approx(-math.pi / 2, abs=1e-6)


def test_the_default_still_drives_every_leg_forward() -> None:
    tl = drive(cell(allow_reverse=True), "c")
    assert tl.duration == pytest.approx(7.0, abs=0.02)
    assert yaw_at(tl, tl.duration) == pytest.approx(math.pi / 2, abs=1e-6)


def test_a_station_arrival_gear_wins_over_the_corner_rule() -> None:
    # Forks first into `c` although the machine prefers backing.
    tl = drive(cell(allow_reverse=True, prefer="reverse", reverse_speed=1.0, arrive={"c": "forward"}), "c")
    assert tl.duration == pytest.approx(7.0, abs=0.02)
    assert yaw_at(tl, tl.duration) == pytest.approx(math.pi / 2, abs=1e-6)
    # Backing onto `c` although the machine never reverses on its own.
    tl = drive(cell(arrive={"c": "reverse"}), "c")
    assert yaw_at(tl, tl.duration) == pytest.approx(-math.pi / 2, abs=1e-6)


def test_gears_are_validated_and_belong_to_a_differential_drive() -> None:
    with pytest.raises(ValueError, match="not a station"):
        cell(arrive={"dock": "forward"})
    with pytest.raises(ValueError, match='"forward" or "reverse"'):
        cell(prefer="sideways")
    with pytest.raises(ValueError, match='"forward" or "reverse"'):
        cell(arrive={"c": "backwards"})
    with pytest.raises(ValueError, match="reverse_speed must be positive"):
        cell(reverse_speed=0.0)
    with pytest.raises(ValueError, match="differential-drive ideas"):
        cell(drive="holonomic", prefer="reverse")
    with pytest.raises(ValueError, match="differential-drive ideas"):
        cell(drive="holonomic", arrive={"c": "forward"})


def test_gears_round_trip_through_the_project_and_generated_python(tmp_path) -> None:
    scene = cell(allow_reverse=True, prefer="reverse", reverse_speed=1.0, arrive={"c": "forward", "a": "reverse"})
    code = scene.generate_python()
    line = next(l for l in code.splitlines() if "add_vehicle(" in l)
    assert "allow_reverse=True, reverse_speed=1, prefer=\"reverse\"" in line
    assert 'arrive={"a": "reverse", "c": "forward"}' in line
    path = tmp_path / "gears.botrail.json"
    scene.save_project(path)
    back = bt.Scene.load_project(path)
    tl = drive(back, "c")
    assert tl.duration == pytest.approx(7.0, abs=0.02)   # the arrival gear survived
    assert yaw_at(tl, tl.duration) == pytest.approx(math.pi / 2, abs=1e-6)
    # And the plain machine writes none of it.
    plain = cell().generate_python()
    assert "prefer" not in plain and "arrive" not in plain and "reverse_speed" not in plain


def test_wheels_accept_a_tricycle_and_drive_it_like_a_differential() -> None:
    wheels = bt.Wheels({"drive_wheel": 0.115, "left": 0.0425}, steer={"drive_wheel": "steer"}, drive="tricycle")
    assert wheels.vehicle_drive == "differential"


# ---------------------------------------------------------------- steered
def test_a_steered_drive_rounds_the_corner_and_stops_only_on_a_line(tmp_path) -> None:
    # The L with a 0.5 m radius: 1.5 m, a quarter arc at v / R = 1 rad/s, 0.5 m.
    tl = drive(cell(drive="steered", turn_radius=0.5), "c")
    assert tl.duration == pytest.approx(1.5 / 0.5 + math.pi / 2 + 0.5 / 0.5, abs=0.02)
    assert yaw_at(tl, tl.duration) == pytest.approx(math.pi / 2, abs=1e-6)
    # Half way round the arc it faces 45°: no pivot anywhere.
    assert yaw_at(tl, 3.0 + math.pi / 4) == pytest.approx(math.pi / 4, abs=1e-3)
    with pytest.raises(ValueError, match="turn_radius belongs"):
        cell(turn_radius=0.5)
    with pytest.raises(ValueError, match="needs turn_radius"):
        cell(drive="steered")
    with pytest.raises(ValueError, match="on the level"):
        cell(drive="steered", turn_radius=0.5, max_grade=0.1)
    # A station on the corner is refused by name: the machine leaves a
    # station along the line it arrived by.
    scene = bt.Scene()
    scene.add_box("chassis", (0.2, 0.2, 0.2), (0.0, 0.0, 0.1))
    scene.add_vehicle("agv", body=["chassis"], path=[(0.0, 0.0), (2.0, 0.0), (2.0, 1.0)],
                      stations={"a": 0, "b": 1, "c": 2}, speed=0.5, turn_speed=math.pi / 2, start="a",
                      drive="steered", turn_radius=0.5)
    with pytest.raises(ValueError, match="along one line"):
        drive(scene, "c")
    # And it survives the project file and the generated Python.
    scene = cell(drive="steered", turn_radius=0.5, allow_reverse=True, reverse_speed=1.0)
    line = next(l for l in scene.generate_python().splitlines() if "add_vehicle(" in l)
    assert 'drive="steered", turn_radius=0.5' in line and "reverse_speed=1" in line
    path = tmp_path / "steered.botrail.json"
    scene.save_project(path)
    back = bt.Scene.load_project(path)
    assert drive(back, "c").duration == pytest.approx(tl.duration, abs=0.02)


def test_the_sweep_of_a_steered_body_reproduces_the_sheets_turning_radius() -> None:
    """Linde L-MATIC AC 1.6 t (Series 1170 type sheet): the fixed axle sits
    1740 mm behind the fork face, the rear end 325 mm behind that, 790 mm
    wide, turning radius Wa 2033 mm. Wa is the outer front corner's circle
    about the turn centre, so the vehicle frame's radius is
    sqrt(2033² − 1740²) − 395 = 656 mm — and driven round a corner at that
    radius, the body's corner sweeps the sheet's figure."""
    rear_axle_to_face, overhang, width = 1.740, 0.325, 0.790
    wa = 2.033
    radius = math.sqrt(wa ** 2 - rear_axle_to_face ** 2) - width / 2
    assert radius == pytest.approx(0.656, abs=0.001)
    scene = bt.Scene()
    length = rear_axle_to_face + overhang
    chassis = scene.add_box("truck/chassis", (length, width, 1.0), (length / 2 - overhang, 0.0, 0.5))
    scene.add_vehicle("truck", body=["truck"], path=[(0.0, 0.0), (6.0, 0.0), (6.0, 6.0)],
                      stations={"a": 0, "c": 2}, speed=1.0, turn_speed=10.0, start="a",
                      drive="steered", turn_radius=radius)
    sq = scene.sequence("drive")
    sq.step("go", actions=[bt.seq.goto("truck", "c")], transition=bt.seq.device_done("truck"))
    tl = sq.simulate()
    # The arc starts R short of the corner at t = 5.344 s and takes π/2 · R s.
    center = (6.0 - radius, radius)
    for frac in (0.1, 0.5, 0.9):
        t = (6.0 - radius) / 1.0 + frac * (math.pi / 2) * radius
        (x, y, _), q = tl.object_pose(chassis, t)
        yaw = yaw_of(q)
        # The outer (right-hand) front corner, from the box centre.
        lx, ly = length / 2, -width / 2
        cx = x + math.cos(yaw) * lx - math.sin(yaw) * ly
        cy = y + math.sin(yaw) * lx + math.cos(yaw) * ly
        assert math.hypot(cx - center[0], cy - center[1]) == pytest.approx(wa, rel=0.003)


def yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
