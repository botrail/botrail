"""The legged example, asserted the way a cell owner would assert it.

`examples/legged/legged_patrol_demo.py` puts a quadruped into a cell as a vehicle
with legs: dispatched with `goto`, docked by a zone, loaded over its back,
let out on a departure permit. What these tests pin is the part a picture
cannot show and the kinematics tests in Rust do not cover — the cell-level
consequences:

* the dog walks (footfalls are taken) and the gait costs no cycle time over
  the same vehicle carried rigidly,
* a planted foot stays where it landed, read back off the baked timeline,
* the handover really is a handshake: the arm waits for the dock, the dog
  waits for the load and the arm's retreat, and the part rides out,
* a gate too narrow for the footprint fails by name.

They run on the primitive quadruped (`--robot quad`), which needs no
download; the Go2 is the same cell with a different URDF.
"""

import math
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "legged"))

import legged_patrol_demo as demo  # noqa: E402

LEGS = ("FL", "FR", "RL", "RR")


@pytest.fixture(scope="module")
def baked():
    return demo.bake("quad")


def _rotate(q, v):
    x, y, z, w = q
    # q * v * q^-1 for a unit quaternion (x, y, z, w)
    t = (2 * (y * v[2] - z * v[1]), 2 * (z * v[0] - x * v[2]), 2 * (x * v[1] - y * v[0]))
    return (
        v[0] + w * t[0] + (y * t[2] - z * t[1]),
        v[1] + w * t[1] + (z * t[0] - x * t[2]),
        v[2] + w * t[2] + (x * t[1] - y * t[0]),
    )


def _foot_world(scene, tl, leg, t):
    """The foot's world position at `t`, from the timeline's base track and
    the scene's FK in the robot frame."""
    q = tl.sample(t, robot="dog")
    base_p, base_q = tl.base_pose(t, robot="dog")
    # Link pose relative to the (parked) scene base, then re-based.
    home_p, _ = scene.robot_base_pose_of("dog")
    link_p, _ = scene.link_pose_at(f"{leg}_foot", q, robot="dog")
    local = tuple(link_p[i] - home_p[i] for i in range(3))  # the scene base has no yaw
    rel = _rotate(base_q, local)
    return tuple(base_p[i] + rel[i] for i in range(3))


def test_the_dog_walks_and_the_part_rides_out(baked):
    _, tl = baked
    steps = tl.footfalls("dog")
    assert len(steps) > 40
    assert {leg for leg, *_ in steps} == set(LEGS)
    # Every landing is on the floor, ball radius up.
    assert all(abs(pos[2] - demo.QUAD_GAIT["foot_radius"]) < 1e-9 for *_, pos in steps)
    # The part left the bench on the dog's back and came back to the yard.
    end = tl.object_pose("part", tl.duration)[0]
    assert abs(end[0] - demo.YARD[0]) < 0.2 and abs(end[1] - demo.YARD[1]) < 0.1, end
    assert end[2] > 0.3, "the part is not on the dog's back"


def test_a_planted_foot_does_not_move(baked):
    scene, tl = baked
    steps = [s for s in tl.footfalls("dog") if s[0] == "FL"]
    checked = 0
    for (_, _, land, pos), (_, lift, _, _) in zip(steps, steps[1:]):
        for k in range(1, 4):
            t = land + (lift - land) * k / 4
            foot = _foot_world(scene, tl, "FL", t)
            assert math.dist(foot, pos) < 2e-3, f"FL slipped {math.dist(foot, pos):.4f} m at {t:.2f}s"
            checked += 1
    assert checked > 30


def test_the_handover_is_a_handshake(baked):
    _, tl = baked
    lanes = dict(tl.signals)
    spans = {name: (start, end) for name, start, end in tl.step_spans}
    docked_at = next(t for t, v in lanes["dog_docked"] if v)
    loaded_at = next(t for t, v in lanes["tray_loaded"] if v)
    arm_off = [t for t, v in lanes["arm_over_dock"] if not v]
    # The arm reaches over the dock only after the dog is there, the part
    # is aboard before the dog leaves, and it leaves only once the arm is
    # back out of the way.
    assert spans["load/to dog"][0] >= docked_at - 1e-9
    assert loaded_at < spans["patrol/to bay"][0] + 1e-9
    assert any(abs(t - spans["patrol/to bay"][0]) < 0.02 for t in arm_off) or max(arm_off) <= spans["patrol/to bay"][0] + 1e-9
    # The walk is most of the cycle.
    assert tl.signal("walker").high_total() > 0.5 * tl.duration


def test_walking_costs_no_cycle_time(baked):
    _, walked = baked
    scene = demo.build_scene("quad")
    # The same vehicle, carried rigidly: mount without a gait at the same
    # height the gait stands the robot at.
    p, _ = scene.robot_base_pose_of("dog")
    scene.mount_robot("walker", offset_position=(0.0, 0.0, p[2]), robot="dog")
    names = demo.build_cycle(scene)
    carried = scene.simulate_sequences(names, max_duration=90.0)
    assert abs(carried.duration - walked.duration) < 1e-9
    assert carried.footfalls("dog") == []


def test_a_narrow_gate_fails_by_name():
    scene = demo.build_scene("quad", narrow=True)
    names = demo.build_cycle(scene)
    with pytest.raises(ValueError, match="VehicleCollision|collides"):
        scene.simulate_sequences(names, max_duration=90.0)


def test_a_stride_the_legs_cannot_take_is_refused():
    scene = demo.build_scene("quad")
    gait = demo.bt.Gait(**{**demo.QUAD_GAIT, "period": 1.2})   # 0.6 m strides on 0.4 m legs
    scene.mount_robot("walker", robot="dog", gait=gait)
    names = demo.build_cycle(scene)
    with pytest.raises(ValueError, match="max_stride"):
        scene.simulate_sequences(names, max_duration=90.0)


def test_the_mount_and_its_gait_survive_a_project_round_trip(baked, tmp_path):
    scene, walked = baked
    path = tmp_path / "legged.botrail"
    scene.save_project(path)
    again = demo.bt.Scene.load_project(path)
    # The reloaded cell stands the dog where it stood and walks the same
    # steps in the same time.
    assert again.robot_base_pose_of("dog") == scene.robot_base_pose_of("dog")
    tl = again.simulate_sequences(["patrol", "load"], max_duration=90.0)
    assert abs(tl.duration - walked.duration) < 1e-9
    assert tl.footfalls("dog") == walked.footfalls("dog")
    # ...and the generated script re-authors the mount with its gait.
    code = scene.generate_python()
    assert 'scene.mount_robot("walker"' in code and "gait=bt.Gait(" in code, code


def test_the_gate_width_that_fits_is_the_footprint_plus_clearance():
    """The aisle check as a sweep: the gate closes on the dog somewhere
    between its footprint (0.42 m wide, plus the posts either side) and a
    comfortable 0.6 m."""
    fits = {}
    for half in (0.14, 0.30, 0.40):
        demo.NARROW_HALF = half
        try:
            scene = demo.build_scene("quad", narrow=True)
            names = demo.build_cycle(scene)
            scene.simulate_sequences(names, max_duration=90.0)
            fits[half] = True
        except ValueError as err:
            assert "collides" in str(err), err
            fits[half] = False
    demo.NARROW_HALF = 0.14
    assert fits == {0.14: False, 0.30: True, 0.40: True}, fits


def test_a_catalog_package_directory_is_a_walker(tmp_path):
    """A `vehicle.legged` package the catalog builder wrote — `urdf/model.urdf`
    beside a manifest whose `locomotion` block is the gait — runs the cell
    with nothing copied out of it: the gait, the body the gate sees and the
    rates all come from the manifest."""
    import shutil

    import yaml

    package = tmp_path / "test" / "quad" / "quad" / "r1"
    (package / "urdf").mkdir(parents=True)
    shutil.copy(demo.ASSETS / "quad_test.urdf", package / "urdf" / "model.urdf")
    gait = demo.QUAD_GAIT
    manifest = {
        "id": "test/quad/quad/r1",
        "category": "vehicle.legged",
        "name": "Quad",
        "specs": {"dof": 12, "locomotion": "quadruped", "footprint_mm": [640, 420],
                  "height_mm": 360, "max_speed_mps": 0.5},
        "locomotion": {
            "kind": "quadruped",
            "legs": [{"name": n, "foot": f, "contact": "point"} for n, f in gait["legs"].items()],
            "stance": gait["stance"],
            "foot_radius_m": gait.get("foot_radius", 0.0),
            "gait": {"pattern": gait["pattern"], "period_s": gait["period"],
                     "lift_m": gait["lift"], "max_stride_m": gait["max_stride"]},
        },
    }
    (package / "manifest.yaml").write_text(yaml.safe_dump(manifest), encoding="utf-8")

    _, walk, footprint, speed, _ = demo.dog_of(str(package))
    assert footprint == (0.64, 0.42, 0.36)
    assert walk.legs["FL"] == (gait["legs"]["FL"], "point") and walk.period == gait["period"]
    assert speed == min(0.5, round(0.6 * gait["max_stride"] / gait["period"], 3))

    _, tl = demo.bake(str(package))
    assert tl.footfalls("dog")
    carried = tl.object_pose("part", tl.duration)[0]
    assert math.dist(carried[:2], demo.YARD) < 0.5  # the part rode out on the dog's back


# ---- the same dog on wheels (design-wheel-legged.md WL0) ------------------
# `--robot quadw` is the primitive quadruped with a continuous wheel on every
# calf, mounted with a gait *and* wheels. The offline stand-in for the Go2-W.


def _wheel_slots(scene):
    names = scene.robot_of("dog").joint_names
    return [names.index(f"{leg}_wheel_joint") for leg in LEGS]


def test_a_wheel_legged_dog_rolls_the_walkway_and_takes_no_step(baked):
    _, walked = baked
    scene, tl = demo.bake("quadw")
    # No footfalls: the legs held their posture and the wheels did the work,
    # which is what the cycle time says too.
    assert tl.footfalls("dog") == []
    assert tl.duration < walked.duration - 10.0, (tl.duration, walked.duration)
    # The first leg is the walkway, yard to dock, straight along +x: 5.6 m
    # at 1.0 m/s, and every wheel (all axles +y) turned by exactly that
    # over its 50 mm radius.
    to_dock = next(end for name, start, end in tl.step_spans if name == "patrol/to dock")
    q0, q1 = tl.sample(0.0, robot="dog"), tl.sample(to_dock, robot="dog")
    for slot in _wheel_slots(scene):
        assert abs((q1[slot] - q0[slot]) - 5.6 / 0.05) < 1e-6, q1[slot] - q0[slot]
    # The part still rides out on its back.
    end = tl.object_pose("part", tl.duration)[0]
    assert abs(end[0] - demo.YARD[0]) < 0.2 and end[2] > 0.3, end


def test_walking_on_its_wheels_is_the_walk_of_the_legged_dog(baked):
    _, walked = baked
    scene, tl = demo.bake("quadw", mode="walk")
    # The wheels are locked: not one turned, from the first tick to the last.
    slots = _wheel_slots(scene)
    for t in (0.0, 3.0, 10.0, 25.0, tl.duration):
        q = tl.sample(t, robot="dog")
        assert all(q[slot] == 0.0 for slot in slots), (t, [q[s] for s in slots])
    # The very walk the wheel-less dog takes — as many steps, in the same
    # time, on the same spots, a wheel radius (50 mm) instead of a ball
    # radius (20 mm) over the floor.
    assert abs(tl.duration - walked.duration) < 1e-9
    steps, plain = tl.footfalls("dog"), walked.footfalls("dog")
    assert len(steps) == len(plain) > 40
    for (leg, lift, land, pos), (leg2, lift2, land2, pos2) in zip(steps, plain):
        assert (leg, lift, land) == (leg2, lift2, land2)
        assert math.dist(pos[:2], pos2[:2]) < 1e-9
        assert abs(pos[2] - pos2[2] - 0.03) < 1e-9
    # ...and it round-trips through a project with both gears on the mount.
    import tempfile
    from pathlib import Path as _Path

    with tempfile.TemporaryDirectory() as tmp:
        path = _Path(tmp) / "wheel_legs.botrail"
        scene.save_project(path)
        again = demo.bt.Scene.load_project(path)
        assert again.robot_base_pose_of("dog") == scene.robot_base_pose_of("dog")
        back = again.simulate_sequences(["patrol", "load"], max_duration=90.0)
        assert back.footfalls("dog") == steps
    code = scene.generate_python()
    assert "gait=bt.Gait(" in code and 'wheels=bt.Wheels(' in code and 'mode="walk"' in code, code


def test_a_catalog_package_with_wheels_rolls_and_walks_from_its_manifest(tmp_path):
    """A `vehicle.legged` package whose `locomotion` block carries `wheels`
    (a Go2-W) runs the cell with nothing copied out of it: the gait — its
    foothold and walking pace included — and the wheels both come off the
    manifest, the vehicle cruises at a derated top speed, and on a flat
    walkway it rolls the whole way."""
    import shutil

    import yaml

    package = tmp_path / "test" / "quadw" / "quadw" / "r1"
    (package / "urdf").mkdir(parents=True)
    shutil.copy(demo.ASSETS / "quad_wheel_test.urdf", package / "urdf" / "model.urdf")
    gait = demo.QUADW_GAIT
    manifest = {
        "id": "test/quadw/quadw/r1",
        "category": "vehicle.legged",
        "name": "QuadW",
        "specs": {"dof": 16, "locomotion": "quadruped", "drive": "skid",
                  "footprint_mm": [640, 420], "height_mm": 390, "max_speed_mps": 2.0},
        "locomotion": {
            "kind": "quadruped",
            "legs": [{"name": n, "foot": f, "contact": "point"} for n, f in gait["legs"].items()],
            "stance": gait["stance"],
            "foot_radius_m": gait["foot_radius"],
            "foothold_m": gait["foothold"],
            "gait": {"pattern": gait["pattern"], "period_s": gait["period"],
                     "lift_m": gait["lift"], "max_stride_m": gait["max_stride"],
                     "speed_mps": 0.45},
            "wheels": {"drive": "skid", "max_step_mm": 30.0,
                       "joints": [{"joint": j, "radius_m": r, "axle": f"{j[:2]}_axle"}
                                  for j, r in demo.QUADW_WHEELS.items()]},
        },
    }
    (package / "manifest.yaml").write_text(yaml.safe_dump(manifest), encoding="utf-8")

    assert demo.bt.Wheels.rolls(package)
    wheels = demo.bt.Wheels.from_catalog(package)
    assert wheels.wheels == dict(demo.QUADW_WHEELS)
    assert wheels.drive == "skid" and wheels.mode == "auto" and wheels.max_step == 0.03
    walk = demo.bt.Gait.from_catalog(package)
    assert walk.foothold == gait["foothold"] and walk.speed == 0.45
    _, walk, footprint, speed, _ = demo.dog_of(str(package))
    assert footprint == (0.64, 0.42, 0.39)
    assert speed == round(demo.ROLL_DERATE * 2.0, 3)
    assert walk.speed == 0.45
    assert demo.wheels_of(str(package)).mode == "auto"

    _, tl = demo.bake(str(package))
    assert tl.footfalls("dog") == []
    assert {mode for *_, mode, _ in tl.locomotion("dog")} == {"roll"}
    carried = tl.object_pose("part", tl.duration)[0]
    assert math.dist(carried[:2], demo.YARD) < 0.5
    # ...and walked whole on request, at the walking pace
    _, tl = demo.bake(str(package), mode="walk")
    assert tl.footfalls("dog") and {mode for *_, mode, _ in tl.locomotion("dog")} == {"walk"}
