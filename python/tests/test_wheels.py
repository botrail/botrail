"""`scene.mount_robot(wheels=...)`: a robot that is its vehicle's running gear
— a semi-humanoid modelled whole — and `bt.Wheels.from_catalog`. No catalog
fetch: the machine is the primitive fixture, the packages are directories."""

from __future__ import annotations

import math
from pathlib import Path

import botrail as bt
import pytest
import yaml

S = bt.seq
FIXTURE = Path(__file__).resolve().parents[2] / "examples" / "assets" / "semi_humanoid_test.urdf"
# Wheels r = 0.10 on a 0.50 m track; a 90 degree pivot runs each 0.25 m arm.
ARC = 0.25 * (math.pi / 2) / 0.10


def wheels(**overrides) -> bt.Wheels:
    fields = {
        "wheels": {"left_wheel_joint": 0.10, "right_wheel_joint": 0.10},
        "base_frame": "base_footprint",
    }
    return bt.Wheels(**(fields | overrides))


def machine(*, gear: bt.Wheels | None = None, path=None, stations=None, start="a") -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(str(FIXTURE)), name="semi")
    scene.add_vehicle(
        "base", body=[], path=path or [(0, 0), (2.0, 0), (2.0, 1.0)],
        stations=stations or {"a": 0, "c": 2}, speed=0.5, turn_speed=math.pi / 2, start=start,
    )
    scene.mount_robot("base", robot="semi", wheels=gear or wheels())
    return scene


def drive(scene: bt.Scene, *extra) -> None:
    sq = scene.sequence("drive")
    sq.step("go", actions=[S.goto("base", "c"), *extra], transition=S.device_done("base"))


def test_the_wheels_turn_by_what_the_vehicle_drove() -> None:
    scene = machine()
    drive(scene)
    tl = scene.simulate_sequence("drive")
    names = list(scene.robot.joint_names)
    left, right = names.index("left_wheel_joint"), names.index("right_wheel_joint")
    assert tl.duration == pytest.approx(7.0, abs=0.011)
    # Straight: distance / radius, the mirrored right axle counting down.
    assert tl.sample(2.0)[left] == pytest.approx(10.0, abs=1e-9)
    assert tl.sample(2.0)[right] == pytest.approx(-10.0, abs=1e-9)
    # A left pivot backs the left wheel up and runs the right one on.
    assert tl.sample(4.5)[left] == pytest.approx(20.0 - ARC / 2, abs=1e-9)
    assert tl.sample(4.5)[right] == pytest.approx(-20.0 - ARC / 2, abs=1e-9)
    assert tl.sample(tl.duration)[left] == pytest.approx(30.0 - ARC, abs=1e-9)
    assert tl.sample(tl.duration)[right] == pytest.approx(-30.0 - ARC, abs=1e-9)
    # The base frame is what rides the vehicle: the root stands on the floor.
    assert tl.base_pose(0.0, "semi")[0] == pytest.approx((0.0, 0.0, 0.0))
    assert tl.base_pose(tl.duration, "semi")[0] == pytest.approx((2.0, 1.0, 0.0))


def test_a_ramp_bakes_into_the_same_track_as_the_rolling() -> None:
    scene = machine()
    drive(scene, S.ramp({"lift_joint": 0.30}, duration=3.0))
    tl = scene.simulate_sequence("drive")
    names = list(scene.robot.joint_names)
    q = tl.sample(1.5)
    assert q[names.index("left_wheel_joint")] == pytest.approx(7.5, abs=1e-6)
    assert 0.05 < q[names.index("lift_joint")] < 0.25
    assert tl.sample(tl.duration)[names.index("lift_joint")] == pytest.approx(0.30)


def test_the_mount_puts_the_machine_in_its_posture() -> None:
    scene = machine(gear=wheels(posture={"lift_joint": 0.12, "left_elbow": 0.9}))
    q = dict(zip(scene.robot.joint_names, scene.joint_positions))
    assert q["lift_joint"] == 0.12 and q["left_elbow"] == 0.9 and q["right_elbow"] == 0.0
    with pytest.raises(ValueError, match="posture joint `nope`"):
        machine(gear=wheels(posture={"nope": 1.0}))


def test_the_declaration_is_checked_by_name() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(str(FIXTURE)), name="semi")
    scene.add_vehicle("base", body=[], path=[(0, 0), (1, 0)], stations={"a": 0, "b": 1}, start="a")
    for gear, message in [
        (wheels(wheels={"nope": 0.1}), "wheel joint `nope` is not a joint"),
        (wheels(wheels={"lift_joint": 0.1}), "must be continuous"),
        (wheels(wheels={"left_wheel_joint": 0.0}), "radius"),
        (wheels(base_frame="left_flange"), "hangs under joint"),
    ]:
        with pytest.raises(ValueError, match=message):
            scene.mount_robot("base", robot="semi", wheels=gear)
    with pytest.raises(ValueError, match="steer names wheels that are not declared"):
        scene.mount_robot("base", robot="semi", wheels=wheels(steer={"nope": "waist_yaw_joint"}))
    with pytest.raises(ValueError, match="drive must be one of"):
        scene.mount_robot("base", robot="semi", wheels=wheels(drive="tracked"))
    gait = bt.Gait(legs={"L": "left_wheel", "R": "right_wheel"}, stance={})
    with pytest.raises(ValueError, match="walks .* or rolls"):
        scene.mount_robot("base", robot="semi", gait=gait, wheels=wheels())


def test_no_move_may_drive_a_wheel() -> None:
    scene = machine()
    sq = scene.sequence("spin")
    sq.step("spin", actions=[S.ramp({"left_wheel_joint": 3.0}, duration=1.0)], transition=S.done())
    with pytest.raises(Exception, match="is a wheel of its mount"):
        scene.simulate_sequence("spin")


def lift_cell(walkable: bool) -> bt.Scene:
    """Into a lift car, up a floor, out onto a mezzanine: every floor's top
    face is exactly where the wheels stand."""
    top, car_x = 2.2, 3.25
    scene = machine(
        path=[(1.0, 0.0, 0.0), (car_x, 0.0, 0.0), (car_x, 0.0, top), (4.6, 0.0, top)],
        stations={"lobby": 0, "car": 1, "dock": 3}, start="lobby",
    )
    scene.add_box("slab", (5.0, 3.0, 0.1), (1.0, 0.0, -0.05))
    scene.add_box("lift/floor", (1.4, 1.4, 0.04), (car_x, 0.0, -0.02))
    scene.add_lift("lift", car=["lift"], zone_position=(car_x, 0.0, 1.0), zone_size=(1.3, 1.3, 2.0),
                   stops={"1F": 0.0, "2F": top}, speed=0.6)
    scene.add_box("mezz/deck", (2.0, 2.5, 0.06), (4.95, 0.0, top - 0.03))
    if walkable:
        for name in ("slab", "lift/floor", "mezz/deck"):
            scene.set_obstacle_walkable(name, True)
    sq = scene.sequence("ride")
    sq.step("board", actions=[S.goto("base", "car")], transition=S.device_done("base"))
    sq.step("ride", actions=[S.move_to("lift", "2F")], transition=S.device_done("lift"))
    sq.step("alight", actions=[S.goto("base", "dock")], transition=S.device_done("base"))
    return scene


def test_a_walkable_floor_is_rolled_on_not_hit() -> None:
    with pytest.raises(Exception, match="collides with `slab`"):
        lift_cell(walkable=False).simulate_sequence("ride", max_duration=60.0)
    scene = lift_cell(walkable=True)
    tl = scene.simulate_sequence("ride", max_duration=60.0)
    names = list(scene.robot.joint_names)
    left = names.index("left_wheel_joint")
    board, ride, alight = (tl.step_span(n) for n in ("board", "ride", "alight"))
    # 2.25 m into the car, nothing while the car climbs, 1.35 m out.
    assert tl.sample(board.end)[left] == pytest.approx(22.5, abs=1e-6)
    assert tl.sample(ride.end)[left] == tl.sample(ride.start)[left]
    assert tl.sample(alight.end)[left] == pytest.approx(36.0, abs=1e-6)
    assert tl.base_pose(tl.duration, "semi")[0] == pytest.approx((4.6, 0.0, 2.2))


def test_the_machine_is_one_purchase() -> None:
    rows = machine().bom().rows
    assert [(r["category"], r["names"]) for r in rows] == [("robot", ["semi"])]
    # The same robot bolted on as cargo is two: the vehicle and its rider.
    scene = bt.Scene(bt.Robot.from_urdf(str(FIXTURE)), name="semi")
    scene.add_box("cart/deck", (0.6, 0.5, 0.1), (0, 0, 0.05))
    scene.add_vehicle("base", body=["cart"], path=[(0, 0), (1, 0)], stations={"a": 0, "b": 1}, start="a")
    scene.mount_robot("base", robot="semi", offset_position=(0, 0, 0.1))
    assert len(scene.bom().rows) > 1
    # ...and declared to roll, it is one again, footprint box and all.
    scene.mount_robot("base", robot="semi", wheels=wheels())
    assert ("robot", ["semi"]) in [(r["category"], r["names"]) for r in scene.bom().rows]
    assert not [r for r in scene.bom().rows if "base" in r["names"] or "semi/controller" in r["names"]]


def test_a_project_and_its_script_keep_the_wheels(tmp_path: Path, monkeypatch) -> None:
    scene = machine(gear=wheels(posture={"lift_joint": 0.2}))
    drive(scene)
    path = tmp_path / "semi.botrail"
    scene.save_project(path)
    loaded = bt.Scene.load_project(path)
    code = loaded.generate_python()
    assert 'wheels=bt.Wheels({"left_wheel_joint": 0.1, "right_wheel_joint": 0.1}, ' \
           'base_frame="base_footprint")' in code
    namespace: dict = {}
    monkeypatch.setattr(bt, "studio", lambda scene: None)
    exec(code, namespace)  # noqa: S102 - our own generated script
    baseline = scene.simulate_sequence("drive")
    for rebuilt in (loaded, namespace["scene"]):
        tl = rebuilt.simulate_sequence("drive")
        assert tl.duration == baseline.duration
        for t in (0.0, 1.3, 4.4, tl.duration):
            assert tl.sample(t) == baseline.sample(t)


def test_under_world_physics_the_wheels_stay_the_vehicles() -> None:
    # The machine becomes an articulated body whose joints are servoed — all
    # but the wheels: they are the mount's, kinematic, and still exact.
    scene = machine()
    drive(scene, S.ramp({"lift_joint": 0.30}, duration=3.0))
    physics = bt.Physics(world=True)
    row = next(r for r in scene.physics_plan(physics).rows if r["name"] == "semi")
    assert row["reason"].startswith("14 joints servo")
    assert "2 wheel joint(s) turned by the vehicle" in row["reason"]
    tl = scene.simulate_sequence("drive", physics=physics, max_duration=30.0)
    names = list(scene.robot.joint_names)
    q = tl.sample(tl.duration)
    assert q[names.index("left_wheel_joint")] == pytest.approx(30.0 - ARC, abs=1e-9)
    assert q[names.index("right_wheel_joint")] == pytest.approx(-30.0 - ARC, abs=1e-9)
    assert q[names.index("lift_joint")] == pytest.approx(0.30, abs=5e-3)   # a servo, not a mirror


def test_the_usd_animation_turns_the_wheels(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom

    scene = machine()
    drive(scene)
    tl = scene.simulate_sequence("drive")
    out = tmp_path / "semi.usdc"
    assert tl.export_usd(out, fps=20) == []
    stage = Usd.Stage.Open(str(out))
    prim = next(p for p in stage.Traverse() if p.GetName() == "left_wheel")
    xf = UsdGeom.Xformable(prim)

    def heading(t: float) -> float:
        """How far the wheel has turned about its axle, read off its +X."""
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode(t * 20))
        x = m.TransformDir((1, 0, 0))
        return math.atan2(-x[2], x[0])

    # 1 s of the first straight: 0.5 m / 0.10 m = 5 rad about +Y.
    assert math.remainder(heading(1.0) - 5.0, math.tau) == pytest.approx(0.0, abs=1e-4)
    assert math.remainder(heading(0.5) - 2.5, math.tau) == pytest.approx(0.0, abs=1e-4)


# --- bt.Wheels.from_catalog ------------------------------------------------

LOCOMOTION = {
    "kind": "wheeled",
    "drive": "differential",
    "wheels": [
        {"joint": "left_wheel_joint", "radius_m": 0.085},
        {"joint": "right_wheel_joint", "radius_m": 0.085},
    ],
    "postures": {"travel": {"lift_joint": 0.05}, "reach": {"lift_joint": 0.35}},
}


def _package(tmp_path: Path, **manifest) -> Path:
    data = {
        "id": "acme/semi/wheeled/r1",
        "category": "vehicle.mobile_manipulator",
        "name": "Semi",
        "frames": {"base_frame": "base_footprint"},
    }
    data.update(manifest)
    (tmp_path / "manifest.yaml").write_text(yaml.safe_dump(data), encoding="utf-8")
    return tmp_path


def test_a_package_directory_yields_the_declared_wheels(tmp_path: Path) -> None:
    package = _package(tmp_path, locomotion=LOCOMOTION)
    gear = bt.Wheels.from_catalog(package)
    assert gear.wheels == {"left_wheel_joint": (0.085, 0.0), "right_wheel_joint": (0.085, 0.0)}
    assert gear.base_frame == "base_footprint" and gear.steer == {}
    assert (gear.drive, gear.vehicle_drive) == ("differential", "differential")
    assert gear.posture == {"lift_joint": 0.05}
    assert sorted(bt.Wheels.postures(package)) == ["reach", "travel"]
    assert bt.Wheels.from_catalog(package, posture="reach").posture == {"lift_joint": 0.35}
    assert bt.Wheels.from_catalog(package, posture=None).posture == {}
    with pytest.raises(ValueError, match=r"no `locomotion.postures.stow` \(it has: reach, travel\)"):
        bt.Wheels.from_catalog(package, posture="stow")
    # ...and it is a Wheels the extension mounts: the posture is taken up.
    scene = machine(gear=gear)
    assert dict(zip(scene.robot.joint_names, scene.joint_positions))["lift_joint"] == 0.05
    spec = bt.Wheels.from_catalog(package / "manifest.yaml", base_frame=None)._spec()
    assert spec["base_frame"] is None and spec["wheels"][0] == ("left_wheel_joint", 0.085, 0.0, None)


def test_mecanum_and_swerve_blocks_say_how_the_vehicle_drives(tmp_path: Path) -> None:
    mecanum = {"kind": "wheeled", "drive": "mecanum", "wheels": [
        {"joint": "fl", "radius_m": 0.076, "lateral": -1}, {"joint": "fr", "radius_m": 0.076, "lateral": 1}]}
    gear = bt.Wheels.from_catalog(_package(tmp_path, locomotion=mecanum))
    assert gear.wheels == {"fl": (0.076, -1.0), "fr": (0.076, 1.0)} and gear.vehicle_drive == "holonomic"
    assert gear.posture == {}  # no `travel` stated: mounted as it stands
    swerve = {"kind": "wheeled", "drive": "swerve", "wheels": [
        {"joint": "w1", "radius_m": 0.06, "steer": "s1"}, {"joint": "w2", "radius_m": 0.06, "steer": "s2"}]}
    gear = bt.Wheels.from_catalog(_package(tmp_path, locomotion=swerve))
    assert gear.steer == {"w1": "s1", "w2": "s2"} and gear.vehicle_drive == "holonomic"
    assert gear._spec()["wheels"][1] == ("w2", 0.06, 0.0, "s2")
    # A chassis mesh with the wheels welded in still rolls.
    bare = bt.Wheels.from_catalog(_package(tmp_path, locomotion={"kind": "wheeled", "drive": "omni"}))
    assert bare.wheels == {} and bare._spec()["wheels"] == []


def test_walking_and_rolling_packages_are_told_apart(tmp_path: Path) -> None:
    legged = {"kind": "biped", "legs": [], "stance": {}, "gait": {}}
    with pytest.raises(ValueError, match="it walks, see bt.Gait.from_catalog"):
        bt.Wheels.from_catalog(_package(tmp_path, locomotion=legged))
    with pytest.raises(ValueError, match="has no `locomotion` block"):
        bt.Wheels.from_catalog(_package(tmp_path))
    with pytest.raises(ValueError, match=r"this machine rolls .*bt\.Wheels\.from_catalog"):
        bt.Gait.from_catalog(_package(tmp_path, locomotion=LOCOMOTION))
