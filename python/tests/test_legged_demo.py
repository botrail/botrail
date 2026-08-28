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
