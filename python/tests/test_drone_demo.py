"""The drone survey (`examples/drone_survey_demo.py`): aerial legs at the
closed-form axis max, the onboard scanner blipping per rack, and the low
corridor through the parked arm refused by name.
"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

import drone_survey_demo as demo  # noqa: E402


def test_the_survey_flies_at_the_axis_max() -> None:
    import math

    # `pack=None` draws the box airframe: the flight is the same either way
    # (the machine's geometry is what it clears, not how long it takes), and
    # this keeps the timing test offline and exact.
    scene, tl = demo.bake(pack=None)
    # Every leg's clock is the slower axis, every course change a pivot at
    # turn_speed, and the gate wait ends when the arm's program says so —
    # the whole cycle in closed form.
    turn = math.pi / 2
    d_out = math.hypot(demo.RACKS[0] - demo.CLIMB_AT[0], 1.1)
    course = math.atan2(1.1, demo.RACKS[0] - demo.CLIMB_AT[0])
    arm_clear = demo.ARM_TEND + demo.ARM_FOLD
    expect = (
        demo.CORRIDOR / demo.CLIMB                       # up to the gate
        + (arm_clear - demo.CORRIDOR / demo.CLIMB)       # the interlock wait
        + (demo.CLIMB_AT[0] - demo.PAD[0]) / demo.SPEED  # the low corridor
        + (demo.CRUISE - demo.CORRIDOR) / demo.CLIMB     # climb to cruise
        + course / turn + d_out / demo.SPEED             # out to r1
        + course / turn                                  # turn onto the row
        + 2 * 1.2 / demo.SPEED                           # r1 -> r2 -> r3
        + 3 * 0.8                                        # scans
        + math.pi / turn                                 # about-face
        + 2 * 1.2 / demo.SPEED                           # r3 -> r1
        + course / turn + d_out / demo.SPEED             # back to the climb point
        + (demo.CRUISE - demo.CORRIDOR) / demo.DESCENT   # descend to the corridor
        + course / turn                                  # turn for home
        + (demo.CLIMB_AT[0] - demo.PAD[0]) / demo.SPEED  # the corridor again
        + demo.CORRIDOR / demo.DESCENT                   # landing
    )
    assert tl.duration == pytest.approx(expect, abs=0.15)
    # The scanner blipped over every rack it crossed — three out, two on
    # the retrace — riding the airframe.
    edges = dict(tl.signals)["scan"]
    rises = [t for t, v in edges if v]
    assert len(rises) == 5, edges
    # …and the drone entered the corridor only after the arm said clear.
    clear = [t for t, v in dict(tl.signals)["arm_clear"] if v]
    assert clear and clear[0] == pytest.approx(arm_clear, abs=0.05)
    # The cell regenerates with its aerial drive.
    code = scene.generate_python()
    assert 'drive="aerial"' in code and "climb_speed=0.6" in code


def test_the_gate_is_what_permits_the_shared_corridor() -> None:
    """Same paths, same volumes, same machines — only the clock differs.
    With the gate the cycle bakes green while the arm works under the
    corridor; without it the drone flies in while the tool is still up,
    and the refusal names the instant. Geometry cannot tell these two
    cells apart; the cross-robot check prices the pair of clocks."""
    _scene, tl = demo.bake(pack=None)
    assert tl.duration > 0.0

    with pytest.raises(ValueError) as refusal:
        demo.bake(interlock=False, pack=None)
    said = str(refusal.value)
    assert "hits robot" in said or "collide" in said, said
    assert "survey" in said, said


def test_a_low_corridor_through_the_stowed_arm_is_refused() -> None:
    """No timing fixes a corridor that is simply too low: even with the
    interlock honoured and the arm folded, 0.45 m threads the machine
    through the arm's folded wrist."""
    with pytest.raises(ValueError) as refusal:
        demo.bake(corridor=0.45, pack=None)
    said = str(refusal.value)
    assert "hits robot" in said or "collide" in said, said


PACKAGE = Path.home() / "projects/botrail-catalog-builder/build/px4/x500/x500/r1"
needs_x500 = pytest.mark.skipif(
    not (PACKAGE / "urdf" / "model.urdf").is_file(), reason="px4/x500 is not built locally"
)


@needs_x500
def test_the_catalog_airframe_is_a_robot_riding_its_vehicle() -> None:
    """A UAV is a robot rigid-mounted on an aerial vehicle — the exact
    symmetry of a quadruped with a gait on a differential one. Being a
    robot is what buys it interference computation: link-level, live, and
    the cross-robot check every tick."""
    scene = demo.build(pack=PACKAGE)
    assert "drone" in scene.robots
    # One machine, one line: a rigid mount on a bodiless vehicle IS the
    # machine, so the vehicle folds into the robot's row.
    rows = [r for r in scene.bom().rows if r["names"][0] == "drone"]
    assert len(rows) == 1, rows
    assert (rows[0]["model"], rows[0]["category"]) == ("X500", "vehicle.uav")
    assert rows[0]["attributes"]["mass_kg"] == 2.0

    # It stands on its landing gear on the pad, props uppermost.
    base, _ = scene.link_pose("base_footprint", robot="drone")
    assert base[2] == pytest.approx(0.0, abs=1e-6), "the gear plane is the pad"

    # Dropping the gate meets the working arm's raised wrist — the refusal
    # names the instant and both links, one from each machine.
    with pytest.raises(ValueError, match="collide") as refusal:
        demo.bake(interlock=False, pack=PACKAGE)
    said = str(refusal.value)
    assert "wrist" in said and "drone" in said, said

    # With the gate, the same two machines share the corridor in time.
    _scene, tl = demo.bake(pack=PACKAGE)
    assert tl.duration > 0.0


def test_the_drive_kwargs_guard_each_other() -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    scene.add_box("body", size=(0.3, 0.3, 0.1), position=(2.0, 2.0, 0.05))
    with pytest.raises(ValueError, match="needs climb_speed"):
        scene.add_vehicle("d", body=["body"], path=[(2.0, 2.0), (3.0, 2.0)],
                          stations={"a": 0, "b": 1}, drive="aerial")
    with pytest.raises(ValueError, match="belong to drive"):
        scene.add_vehicle("d", body=["body"], path=[(2.0, 2.0), (3.0, 2.0)],
                          stations={"a": 0, "b": 1}, climb_speed=0.5)
    with pytest.raises(ValueError, match="ground drive"):
        scene.add_vehicle("d", body=["body"], path=[(2.0, 2.0), (3.0, 2.0)],
                          stations={"a": 0, "b": 1}, drive="aerial",
                          climb_speed=0.5, descent_speed=0.5, max_grade=0.1)
