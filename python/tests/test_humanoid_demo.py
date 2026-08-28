"""The humanoid example, asserted the way a cell owner would assert it.

`examples/legged/humanoid_carry_demo.py` walks a biped between two benches with a
tote in its hands. The legs are a gait on the mount, so what the cell-level
test pins is what a picture cannot and the Rust gait tests do not cover:

* the tote is carried — it leaves bench A in the hands and ends on bench B,
* the arms hold still through the carry (no swing with full hands) and
  swing on the way back (empty hands),
* the body bobs and leans while walking, read back off the base track,
  and rides rigidly once it stands,
* the soles land flat, at sole height.

It runs on the primitive biped (`--robot biped`), which needs no download;
the G1 is the same cell with Unitree's URDF.
"""

import math
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "legged"))

import humanoid_carry_demo as demo  # noqa: E402


@pytest.fixture(scope="module")
def baked():
    return demo.bake("biped")


def _span(tl, name):
    s = tl.step_span(name)
    return s.start, s.end


def test_the_tote_rides_in_the_hands_to_bench_b(baked):
    _, tl = baked
    start = tl.object_pose("tote", 0.0)[0]
    end = tl.object_pose("tote", tl.duration)[0]
    # From bench A (ahead of station a) to bench B (ahead of station b),
    # both faced +x: the tote moved the way the robot walked.
    assert abs(start[1] - demo.BENCH_A[1]) < 0.05 and start[0] > demo.BENCH_A[0] + 0.1
    assert abs(end[1] - demo.BENCH_B[1]) < 0.05 and end[0] > demo.BENCH_B[0] + 0.1, end
    # It stayed at hand height the whole way and was not dropped.
    to_b = _span(tl, "to b")
    for k in range(10):
        t = to_b[0] + (to_b[1] - to_b[0]) * k / 10
        z = tl.object_pose("tote", t)[0][2]
        assert abs(z - start[2]) < 0.08, f"tote at {z} at {t}"


def test_full_hands_do_not_swing_and_empty_ones_do(baked):
    scene, tl = baked
    names = scene.robot.joint_names
    shoulder = names.index("L_shoulder_pitch_joint")
    to_b, back = _span(tl, "to b"), _span(tl, "return")
    carried = [tl.sample(to_b[0] + k * 0.1, robot="walker")[shoulder] for k in range(int((to_b[1] - to_b[0]) / 0.1))]
    assert max(carried) - min(carried) < 1e-9, "the arm moved while carrying"
    swung = [tl.sample(back[0] + k * 0.1, robot="walker")[shoulder] for k in range(int((back[1] - back[0]) / 0.1))]
    assert max(swung) - min(swung) > 0.5, "the arm did not swing on the way back"
    assert len(tl.footfalls("walker")) > 20


def test_the_body_sways_while_walking_and_rides_rigid_standing(baked):
    _, tl = baked
    standing = tl.base_pose(0.0, robot="walker")[0][2]
    to_b = _span(tl, "to b")
    heights = [tl.base_pose(to_b[0] + k * 0.02, robot="walker")[0][2]
               for k in range(int((to_b[1] - to_b[0]) / 0.02))]
    bob = demo.BIPED_GAIT["bob"]
    assert max(heights) > standing + 0.8 * bob and min(heights) < standing - 0.8 * bob
    # Standing at bench B (set down / release / rest): rigid again.
    rest = _span(tl, "rest")
    for k in range(5):
        z = tl.base_pose(rest[0] + (rest[1] - rest[0]) * k / 5, robot="walker")[0][2]
        assert abs(z - standing) < 1e-9


def test_the_arm_ramps_clear_the_benches(baked):
    scene, _ = baked
    stance, q_tuck, q_carry, q_lift = demo.build_scene("biped")[1]
    for a, b in ((stance, q_tuck), (q_tuck, q_carry), (q_carry, q_lift), (q_carry, q_tuck), (q_tuck, stance)):
        assert demo.ramp_contacts(scene, a, b) == []
    # ...and a straight reach from the hip would not: that is why there
    # are two ramps.
    assert any("bench" in pair[1] for pair in demo.ramp_contacts(scene, stance, q_carry))


def test_soles_land_flat_at_sole_height(baked):
    _, tl = baked
    sole = demo.BIPED_GAIT["foot_radius"]
    steps = tl.footfalls("walker")
    assert {leg for leg, *_ in steps} == {"L", "R"}
    assert all(abs(pos[2] - sole) < 1e-9 for *_, pos in steps)
    # The walk to B backs away from bench A, turns, and arrives nose first:
    # the body heads +x again at the last landings before B.
    to_b = _span(tl, "to b")
    last = [s for s in steps if s[2] <= to_b[1] + 1.5][-2:]
    for leg, lift, land, pos in last:
        _, base_q = tl.base_pose(land + 0.05, robot="walker")
        x, y, z, w = base_q   # yaw from the quaternion (x, y, z, w)
        yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        assert abs(yaw) < 1e-6, f"{leg} landed with the body at {yaw}"
    # ...and the first ones stepped backwards, away from the bench.
    first = [s for s in steps if s[2] > to_b[0]][:4]
    assert all(pos[0] < 0.05 for *_, pos in first), first


def test_a_walking_robot_has_no_controller_script(baked):
    _, tl = baked
    # The carry sequence drives the walker (its arms ramp), but its legs
    # are a gait: there is no robot program in it to export.
    with pytest.raises(ValueError, match="gait"):
        tl.to_script(sequence="carry", dialect="urscript")
