"""Crouching (`bt.seq.crouch`): a parked walker lowers its body while it
stands — feet planted, legs re-solved every tick — and stands back up.

On the primitive biped (`examples/assets/biped_test.urdf`), no download. The
planted feet, the settle after a walk and the tick checks are pinned in Rust
(`rollout::biped_tests`); these are the binding and the cell-level rules:
the action as authored, the static posture a pose is taught on, a goto
refused while crouched, the project round trip.
"""

from pathlib import Path

import pytest

import botrail as bt

ASSETS = Path(__file__).resolve().parents[2] / "examples" / "assets"
GAIT = {
    "legs": {"L": "L_foot", "R": "R_foot"},
    "contact": "sole",
    "stance": {f"{s}_{j}": v for s in ("L", "R")
               for j, v in (("hip_yaw_joint", 0.0), ("hip_roll_joint", 0.0), ("hip_pitch_joint", -0.4),
                            ("knee_joint", 0.8), ("ankle_pitch_joint", -0.4), ("ankle_roll_joint", 0.0))},
    "pattern": "biped", "period": 0.8, "lift": 0.05, "max_stride": 0.5, "foot_radius": 0.05,
}


def biped_scene():
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "biped_test.urdf"), name="walker")
    scene.add_vehicle("legs", body=[], path=[(0.0, 0.0), (1.5, 0.0)], stations={"a": 0, "b": 1},
                      speed=0.3, start="a")
    scene.mount_robot("legs", robot="walker", gait=bt.Gait(**GAIT))
    return scene


def test_the_action_as_authored():
    assert bt.seq.crouch("walker", 0.1) == {"type": "crouch", "depth": 0.1, "robot": "walker"}
    assert bt.seq.crouch("walker", 0.25, lean=0.3, duration=1.5) == {
        "type": "crouch", "depth": 0.25, "robot": "walker", "lean": 0.3, "duration": 1.5}
    assert bt.seq.crouch(None, 0.0) == {"type": "crouch", "depth": 0.0}


def test_a_crouch_goes_down_and_stands_back_up():
    scene = biped_scene()
    z0 = scene.robot_base_pose_of("walker")[0][2]
    sq = scene.sequence("work")
    # No transition: a crouch is a move, so the step waits for it.
    sq.step("down", actions=[bt.seq.crouch("walker", 0.1, lean=0.15, duration=1.0)])
    sq.step("hold", transition=bt.seq.elapsed(0.5))
    sq.step("up", actions=[bt.seq.crouch("walker", 0.0, duration=1.0)])
    sq.step("go", actions=[bt.seq.goto("legs", "b")], transition=bt.seq.device_done("legs"))
    tl = sq.simulate()
    z = lambda t: tl.base_pose(t, robot="walker")[0][2]  # noqa: E731
    assert z(1.0) == pytest.approx(z0 - 0.1, abs=1e-9)
    assert z(1.5) == pytest.approx(z0 - 0.1, abs=1e-9)
    # Standing again — to the leg solve's tolerance: the planted feet are
    # where the IK put them, a fraction of a micrometre off.
    assert z(2.5) == pytest.approx(z0, abs=1e-6)
    # ... and walked off afterwards, upright again.
    end = tl.base_pose(tl.duration, robot="walker")[0]
    assert end[0] == pytest.approx(1.5, abs=1e-6)
    assert end[2] == pytest.approx(z0, abs=1e-6)


def test_a_goto_while_crouched_is_refused_by_name():
    scene = biped_scene()
    sq = scene.sequence("work")
    sq.step("down", actions=[bt.seq.crouch("walker", 0.1)])
    sq.step("go", actions=[bt.seq.goto("legs", "b")], transition=bt.seq.device_done("legs"))
    with pytest.raises(Exception, match=r"stand it up first: bt\.seq\.crouch\(\"walker\", 0\)"):
        sq.simulate()


def test_a_crouch_too_deep_is_refused_by_name():
    scene = biped_scene()
    sq = scene.sequence("work")
    sq.step("down", actions=[bt.seq.crouch("walker", 0.45)])
    with pytest.raises(Exception, match=r"cannot crouch to 0\.450 m"):
        sq.simulate()
    with pytest.raises(ValueError, match=r"cannot crouch to 0\.450 m"):
        scene.crouch_pose(0.45)


def test_crouch_pose_is_the_posture_the_bake_reaches():
    scene = biped_scene()
    standing = scene.robot_base_pose_of("walker")
    (p, q), joints = scene.crouch_pose(0.1, lean=0.2)
    # The scene is untouched; the posture is lowered from where it stands.
    assert scene.robot_base_pose_of("walker") == standing
    assert standing[0][2] - p[2] == pytest.approx(0.1, abs=1e-12)
    sq = scene.sequence("work")
    sq.step("down", actions=[bt.seq.crouch("walker", 0.1, lean=0.2, duration=1.0)])
    tl = sq.simulate()
    bp, bq = tl.base_pose(tl.duration, robot="walker")
    assert bp == pytest.approx(p, abs=1e-9)
    assert abs(sum(a * b for a, b in zip(bq, q))) == pytest.approx(1.0, abs=1e-9)
    assert tl.sample(tl.duration, robot="walker") == pytest.approx(joints, abs=1e-5)


def test_a_crouch_round_trips_through_the_project(tmp_path):
    scene = biped_scene()
    sq = scene.sequence("work")
    sq.step("down", actions=[bt.seq.crouch("walker", 0.1, lean=0.2, duration=1.5)])
    sq.step("up", actions=[bt.seq.crouch("walker", 0.0)])
    script = scene.generate_python()
    assert 'bt.seq.crouch("walker", 0.1, lean=0.2, duration=1.5)' in script
    assert 'bt.seq.crouch("walker", 0.0)' in script
    path = tmp_path / "cell.botrail"
    scene.save_project(path)
    again = bt.Scene.load_project(path)
    assert again.generate_python() == script
