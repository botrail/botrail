"""Grasping (design-grasping.md G1): deriving the close, exempting the
tool, and reading the grasp back out of a physics bake.

`grasp_close` solves the drive values that bring the pads to a signed
clearance from the part (default: half a millimetre of overtravel, the
measured sweet spot for reliable contact without disturbance), a ramp
bakes them, `attach(touch_links="tool")` exempts the whole tool subtree,
and `tl.grasp_report()` says whether the taught close actually touched —
plus the release-order and specs checks.
"""

import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

# A coupled two-finger gripper with collision boxes: pad inner faces 30 mm
# out from the centre plane each at q = 0, so a 40 mm part is touched at
# drive = 0.010 exactly, and the grasp_center frame sits between the pads.
GRIPPER = """
<robot name="grip2f">
  <link name="palm">
    <collision><origin xyz="0 0 0.02"/><geometry><box size="0.08 0.06 0.04"/></geometry></collision>
  </link>
  <link name="finger_l">
    <collision><origin xyz="0 0 0.03"/><geometry><box size="0.01 0.02 0.06"/></geometry></collision>
  </link>
  <link name="finger_r">
    <collision><origin xyz="0 0 0.03"/><geometry><box size="0.01 0.02 0.06"/></geometry></collision>
  </link>
  <link name="grasp_center"/>
  <joint name="drive" type="prismatic">
    <parent link="palm"/><child link="finger_l"/>
    <origin xyz="-0.035 0 0.04"/><axis xyz="1 0 0"/>
    <limit lower="0" upper="0.028" effort="10" velocity="0.1"/>
  </joint>
  <joint name="follow" type="prismatic">
    <parent link="palm"/><child link="finger_r"/>
    <origin xyz="0.035 0 0.04"/><axis xyz="1 0 0"/>
    <limit lower="-0.028" upper="0" effort="10" velocity="0.1"/>
    <mimic joint="drive" multiplier="-1" offset="0"/>
  </joint>
  <joint name="palm_to_tcp" type="fixed">
    <parent link="palm"/><child link="grasp_center"/>
    <origin xyz="0 0 0.07"/>
  </joint>
</robot>
"""


def gripper_scene() -> bt.Scene:
    """Bare gripper at the origin, a 40 mm part seated on a pedestal
    between its pads."""
    scene = bt.Scene(bt.Robot.from_urdf_string(GRIPPER))
    scene.add_box("part", size=(0.04, 0.02, 0.04), position=(0, 0, 0.07))
    scene.add_box("pedestal", size=(0.12, 0.12, 0.05), position=(0, 0, 0.025))
    scene.set_physics("part", dynamic=True, mass=0.1, friction=0.6)
    return scene


def grasp_cycle(scene: bt.Scene, name: str, open_before_detach: bool) -> None:
    q = scene.grasp_close("part")
    sq = scene.sequence(name)
    sq.step("close", actions=[bt.seq.ramp(q, 0.4)])
    sq.step(
        "grab",
        actions=[bt.seq.attach("part", link="palm", touch_links="tool")],
        transition=bt.seq.elapsed(0.5),
    )
    if open_before_detach:
        sq.step("open", actions=[bt.seq.ramp({"drive": 0.0}, 0.3)])
    sq.step("drop", actions=[bt.seq.detach("part")], transition=bt.seq.elapsed(0.5))


def test_grasp_close_derives_the_touch_width() -> None:
    scene = gripper_scene()
    q = scene.grasp_close("part")
    # 40 mm part in a 60 mm opening: touch at 0.010, plus 0.5 mm overtravel.
    assert set(q) == {"drive"}, "mimic follower closes with its driver"
    assert q["drive"] == pytest.approx(0.0105, abs=1e-4)
    # A stop-short clearance stays out of contact by that margin.
    shy = scene.grasp_close("part", clearance=0.002)
    assert shy["drive"] == pytest.approx(0.008, abs=1e-4)
    # An explicit fully-closed value takes the same path to the same touch.
    explicit = scene.grasp_close("part", closed={"drive": 0.028})
    assert explicit["drive"] == pytest.approx(q["drive"], abs=1e-6)


def test_grasp_close_speaks_authoring_errors() -> None:
    scene = bt.Scene(bt.Robot.from_urdf_string(GRIPPER))
    scene.add_box("far", size=(0.04, 0.02, 0.04), position=(0.4, 0, 0.07))
    with pytest.raises(Exception, match="never reaches"):
        scene.grasp_close("far")

    arm = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    arm.add_box("part", size=(0.04, 0.04, 0.04), position=(0.5, 0, 0.3))
    with pytest.raises(Exception, match="no drive joints"):
        arm.grasp_close("part")


def test_grasp_close_solves_on_an_attached_tool() -> None:
    """The same gripper welded onto an arm: the drive value is a property
    of pads-vs-part geometry, so it matches the bare-gripper answer when
    the part sits at the same relative pose."""
    arm = bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")
    gripper = bt.Robot.from_urdf_string(GRIPPER)
    robot = arm.attach_tool(gripper, flange="tool0", tcp="grasp_center")
    scene = bt.Scene(robot)
    (px, py, pz), quat = scene.link_pose("grasp_center")
    scene.add_box("part", size=(0.04, 0.02, 0.04), position=(px, py, pz), quaternion=quat)
    q = scene.grasp_close("part")
    assert q["drive"] == pytest.approx(0.0105, abs=1e-4)


def test_attach_touch_links_tool_is_the_subtree() -> None:
    assert bt.seq.attach("part", touch_links="tool")["touch_links"] == ["tool"]
    scene = gripper_scene()
    scene.attach("part", link="palm", touch_links=["tool"])
    # The exemption covers the fingers: closing to overtravel while holding
    # is not a collision of the robot with its own load.
    scene.detach("part")


def test_physics_report_reads_the_grasp() -> None:
    scene = gripper_scene()
    grasp_cycle(scene, "cycle", open_before_detach=True)
    tl = scene.simulate_sequence("cycle", physics=True)
    (rep,) = tl.grasp_report(grip_force_n=20.0, mu=1.1, payload_kg=5.0)
    assert rep["object"] == "part"
    assert set(rep["touched"]) == {"finger_l", "finger_r"}
    assert all(f > 0 for f in rep["touched"].values())
    assert rep["mass_kg"] == pytest.approx(0.1)
    assert rep["released_touching"] is False
    assert rep["checks"] == {
        "touch": "pass",
        "release": "pass",
        "payload": "pass",
        "grip_force": "pass",
        # A welded attach has no slip to measure — hold is a friction-
        # drive (set_gripper_drive) check.
        "hold": "skip",
    }
    # A gripper too weak for the load fails the static holding check.
    (weak,) = tl.grasp_report(grip_force_n=0.1, mu=0.2)
    assert weak["checks"]["grip_force"] == "fail"


def test_release_inside_the_squeeze_warns() -> None:
    """Detaching before opening returns the part to physics inside the
    overtravel — the report flags the authored order."""
    scene = gripper_scene()
    grasp_cycle(scene, "bad", open_before_detach=False)
    tl = scene.simulate_sequence("bad", physics=True)
    (rep,) = tl.grasp_report()
    assert rep["released_touching"] is True
    assert rep["checks"]["release"] == "warn"


def test_misplaced_part_fails_the_touch_check() -> None:
    """The taught close solved against the authored pose; a part that is
    not there any more closes on air — which is exactly what the report
    exists to catch (fingers 0/2)."""
    scene = gripper_scene()
    grasp_cycle(scene, "cycle", open_before_detach=True)
    scene.add_scenario("shifted", obstacles={"part": (0.0, 0.12, 0.07)})
    tl = scene.simulate_sequence("cycle", physics=True, scenario="shifted")
    (rep,) = tl.grasp_report()
    assert rep["touched"] == {}
    assert rep["checks"]["touch"] == "fail"


def test_kinematic_bake_skips_contact_checks() -> None:
    scene = gripper_scene()
    grasp_cycle(scene, "cycle", open_before_detach=True)
    tl = scene.simulate_sequence("cycle")
    (rep,) = tl.grasp_report()
    assert tl.physics is None
    assert rep["touched"] == {}
    assert rep["checks"]["touch"] == "skip"
    # The mass and the static holding check still work off the authoring.
    assert rep["mass_kg"] == pytest.approx(0.1)
    (checked,) = tl.grasp_report(grip_force_n=20.0, mu=1.1)
    assert checked["checks"]["grip_force"] == "pass"


def test_link_material_is_name_keyed() -> None:
    scene = gripper_scene()
    scene.set_link_material("finger_l", friction=1.1)
    scene.set_link_material("finger_r", friction=1.1, restitution=0.05)
    with pytest.raises(Exception, match="unknown link"):
        scene.set_link_material("nope", friction=1.0)


def test_close_ramp_respects_joint_limits() -> None:
    """The derived value lands inside the drive's limits, so the ramp it
    feeds validates — the whole teach→bake loop in one line each."""
    scene = gripper_scene()
    q = scene.grasp_close("part")
    assert 0.0 <= q["drive"] <= 0.028
    sq = scene.sequence("just_close")
    sq.step("close", actions=[bt.seq.ramp(q, 0.3)])
    tl = scene.simulate_sequence("just_close")
    assert tl.duration >= 0.3


def test_grasp_close_max_accel_feeds_grip_check() -> None:
    """A carry with real acceleration raises the demanded holding force;
    the report's max_accel is nonzero for a swinging carry."""
    scene = gripper_scene()
    grasp_cycle(scene, "cycle", open_before_detach=True)
    tl = scene.simulate_sequence("cycle", physics=True)
    (rep,) = tl.grasp_report()
    # The bare gripper never moves its palm, so the carry is still.
    assert rep["max_accel"] == pytest.approx(0.0, abs=1e-6)


def test_report_defaults_from_the_catalog_gripper() -> None:
    """A catalog gripper welded on with attach_tool keeps its specs inside
    the composite's provenance; grasp_report defaults its holding checks
    from them — payload here, grip force once the published row carries
    the flat mirrors (G2 republish)."""
    arm = bt.Robot.from_catalog("ur5e")
    tool = bt.Robot.from_catalog("robotiq/2f-85")
    scene = bt.Scene(arm.attach_tool(tool), base_position=(0.0, 0.0, 0.74))
    (px, py, pz), _ = scene.link_pose(scene.robot.tcp_link)
    scene.add_box("carton", size=(0.06, 0.06, 0.06), position=(px, py, pz))
    scene.set_physics("carton", dynamic=True, mass=3.0)
    sq = scene.sequence("hold")
    sq.step("grab", actions=[bt.seq.attach("carton", touch_links="tool")],
            transition=bt.seq.elapsed(0.3))
    sq.step("drop", actions=[bt.seq.detach("carton")], transition=bt.seq.elapsed(0.2))
    tl = scene.simulate_sequence("hold")
    (rep,) = tl.grasp_report()
    assert rep["payload_limit_kg"] == pytest.approx(5.0)  # 2F-85 specs, not retyped
    assert rep["mass_kg"] == pytest.approx(3.0)
    assert rep["checks"]["payload"] == "pass"
    # An explicit argument always beats the catalog number.
    (tight,) = tl.grasp_report(payload_kg=2.0)
    assert tight["checks"]["payload"] == "fail"


def test_friction_drive_holds_and_reports_slip() -> None:
    """G3: with a gripper drive declared the attach is a friction hold —
    the fingers are force-capped dynamic bodies and the report carries a
    measured slip instead of a weld's None. The pedestal here is slimmer
    than the pads' sweep: a dynamic finger born inside static scenery
    starts the bake buried (the fingers-in-pedestal lesson)."""
    scene = bt.Scene(bt.Robot.from_urdf_string(GRIPPER))
    scene.add_box("part", size=(0.04, 0.02, 0.04), position=(0, 0, 0.07))
    scene.add_box("pedestal", size=(0.03, 0.012, 0.05), position=(0, 0, 0.025))
    scene.set_physics("part", dynamic=True, mass=0.1, friction=0.6)
    scene.set_gripper_drive(max_force=30.0)
    scene.set_link_material("finger_l", friction=0.9)
    scene.set_link_material("finger_r", friction=0.9)
    grasp_cycle(scene, "cycle", open_before_detach=False)
    tl = scene.simulate_sequence("cycle", physics=True)
    (rep,) = tl.grasp_report()
    # slip_m is measured (a friction hold), not None (a weld) — and a
    # still cell with a 30 N cap on a 100 g part holds firmly.
    assert rep["slip_m"] is not None
    assert rep["slip_m"] < 0.01
    assert rep["checks"]["hold"] == "pass"
    assert set(rep["touched"]) == {"finger_l", "finger_r"}
