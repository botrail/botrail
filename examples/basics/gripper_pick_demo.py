"""Physics pick & place with the gripper doing the gripping
(design-grasping.md G1).

`physics_pick_place.py` carries its parts suction-style, hung under the
wrist, because fingers were a hazard: a hand-tuned close either stopped
short (no contact) or overtravelled (shoved the part). This demo is the
same story told with a real gripper — a catalog UR5e + Robotiq 2F-85 —
and the three hand-tunings that cell needed are now derived:

* the close width comes from `scene.grasp_close("part")` — solved from
  the pad/part geometry at teach time, half a millimetre of overtravel so
  the physics bake reliably *records* the touch (the hand-tuned `SHUT =
  0.33` in `sfc_chart_demo.py` is this number, found by eye);
* the touch exemption is `attach(..., touch_links="tool")` — the whole
  tool subtree, not a hand-enumerated pad list;
* whether the grasp actually happened is read back with
  `tl.grasp_report()`: which pads touched at the attach (with contact
  forces), whether the release was clean (open, *then* detach), and the
  static holding checks against the gripper's numbers.

The second bake runs a scenario where the part never arrived at the
station. The taught cycle closes on air and carries nothing — attach is a
weld, it cannot notice — but the report can: `touch: fail, 0 pads`.

Run with:  python examples/basics/gripper_pick_demo.py [out.usda] [--studio]
"""

import sys

import botrail as bt

# --- cell dimensions (metres; z = 0 is the shop floor) ------------------
BASE_Z = 0.74  # robot mounting plane, on top of its pedestal
PART = 0.06  # part edge length: 60 mm cube, well inside the 85 mm stroke
STAND_XY = (0.45, 0.28)
STAND_TOP = 0.40
PLATE_XY = (0.45, -0.34)
PLATE_TOP = STAND_TOP - 0.005  # 5 mm lower: the release visibly seats
HOVER = 0.12
SEAT_GAP = 0.005  # release height above the seat — the drop physics makes real

DOWN = (1.0, 0.0, 0.0, 0.0)  # tool +Z at the floor
OPEN = 0.0
READY = [0.0, -1.9, 1.8, -1.5, -1.57, 0.0, OPEN]

arm = bt.Robot.from_catalog("ur5e")
coupling = bt.Robot.from_catalog("gripper-coupling")
gripper = bt.Robot.from_catalog("2f-85")
scene = bt.Scene(arm.attach_tool(coupling, prefix="cpl_").attach_tool(gripper),
                 base_position=(0.0, 0.0, BASE_Z))
scene.set_joint_positions(READY)

# The world: floor, the robot's pedestal (scenery), a stand the part
# waits on, and the plate it is seated onto.
scene.add_box("floor", size=(2.4, 2.4, 0.05), position=(0.2, 0.0, -0.025),
              color=(0.35, 0.37, 0.40))
scene.add_cylinder("pedestal", 0.09, BASE_Z, (0.0, 0.0, BASE_Z / 2),
                   color=(0.34, 0.36, 0.40))
scene.set_obstacle_enabled("pedestal", False)
scene.add_box("stand", size=(0.16, 0.16, STAND_TOP), position=(*STAND_XY, STAND_TOP / 2),
              color=(0.45, 0.47, 0.50))
scene.add_box("plate", size=(0.22, 0.22, PLATE_TOP), position=(*PLATE_XY, PLATE_TOP / 2),
              color=(0.80, 0.65, 0.15))
scene.add_box("part", size=(PART, PART, PART),
              position=(*STAND_XY, STAND_TOP + PART / 2), color=(0.85, 0.33, 0.20))
scene.set_physics("part", dynamic=True, mass=0.35, friction=0.6)
# The pads are rubber — say so, and the contact record (and any future
# friction grasp) sees rubber, not the 0.5 default.
for side in ("left", "right"):
    scene.set_link_material(f"{side}_inner_finger_pad", friction=1.1)

# --- teach --------------------------------------------------------------
# IK poses with the finger value carried in every goal (a pose taught
# with the gripper open would re-open it mid-carry). The 2F-85's catalog
# TCP is the pad midpoint; it grips the part's upper flank — deep enough
# for the pads, high enough that the knuckles clear the part's top.
GRIP_DEPTH = 0.018  # how far below the part's top the pad midpoint sits
GRIP = (STAND_XY[0], STAND_XY[1], STAND_TOP + PART - GRIP_DEPTH)
HANG = GRIP[2] - STAND_TOP  # the part hangs this far below the TCP
SEAT = (PLATE_XY[0], PLATE_XY[1], PLATE_TOP + HANG + SEAT_GAP)


def at(position, finger):
    scene.set_joint_positions(READY)
    if not scene.set_tcp_target(position, DOWN).converged:
        raise SystemExit(f"cell layout unreachable at {position}")
    return [*scene.joint_positions[:6], finger]


# Pose the grasp, then let the geometry answer what "closed" is for this
# part. (sfc_chart_demo hand-tuned this as SHUT = 0.33; here it is
# derived — and prints as roughly that same number.)
scene.set_joint_positions(at(GRIP, OPEN))
q_close = scene.grasp_close("part")
(finger_joint, close_value), = q_close.items()
print(f"derived close: {finger_joint} = {close_value:.3f} rad "
      f"(60 mm part in an 85 mm stroke)")

names = scene.robot.joint_names
ramp = lambda goal, s: bt.seq.ramp(dict(zip(names, goal)), s)  # noqa: E731

approach = at((GRIP[0], GRIP[1], GRIP[2] + HOVER), OPEN)
descend = at(GRIP, OPEN)
carry = [*approach[:6], close_value]
over_seat = at((SEAT[0], SEAT[1], SEAT[2] + HOVER), close_value)
seat = at(SEAT, close_value)
clear = at((SEAT[0], SEAT[1], SEAT[2] + HOVER), OPEN)
scene.set_joint_positions(READY)

# The whole cycle is authored as ramps, physics_pick_place-style: after
# the grasp the part is robot's load resting a solver-slop hair into its
# stand, and a collision-checked planner rightly refuses to start there.
# Deliberate contact — and the moves right around it — is ramp territory.
sq = scene.sequence("cycle")
sq.step("approach", actions=[ramp(approach, 1.2)])
sq.step("descend", actions=[ramp(descend, 0.8)])
# A settle beat between "closed" and "grabbed", the way a real cell
# confirms grip before moving — it also gives the contact record a few
# ticks of steady squeeze to measure a force from.
sq.step("close", actions=[bt.seq.ramp(q_close, 0.5)],
        transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(0.65)))
sq.step("grab", actions=[bt.seq.attach("part", touch_links="tool")])
sq.step("lift", actions=[ramp(carry, 0.9)])
sq.step("place", actions=[ramp(over_seat, 1.4)])
sq.step("seat", actions=[ramp(seat, 0.8)])
# Open *then* detach: a part returned to physics inside the squeeze gets
# kicked (the report warns if the order is wrong).
sq.step("open", actions=[bt.seq.ramp({finger_joint: OPEN}, 0.4)])
sq.step("release", actions=[bt.seq.detach("part")], transition=bt.seq.elapsed(0.6))
sq.step("retreat", actions=[ramp(clear, 0.8)])
sq.step("home", actions=[ramp(READY, 1.2)],
        transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(1.0)))

timeline = scene.simulate_sequence("cycle", physics=True, max_duration=60.0)
print(f"baked {timeline.duration:.2f}s under physics={timeline.physics!r}")


def show(report):
    for rep in report:
        # Forces can legitimately read 0.0: a half-millimetre overtravel
        # sits inside the solver's slop, so a gentle grip is "touching,
        # negligible force" — the touch itself is the verified fact.
        pads = ", ".join(
            link + (f" ({force:.1f} N)" if force > 0.05 else "")
            for link, force in rep["touched"].items())
        print(f"  grasp of {rep['object']} at t={rep['start']:.2f}s: "
              f"{len(rep['touched'])} tool links touching" + (f" — {pads}" if pads else ""))
        print(f"    checks: {rep['checks']}  (mass {rep['mass_kg']:.2f} kg, "
              f"peak carry accel {rep['max_accel']:.2f} m/s²)")


# The gripper's own numbers: 2F-85 grip force is 20–235 N — check the
# *weakest* setting against this part and carry.
show(timeline.grasp_report(grip_force_n=20.0, mu=0.6, payload_kg=5.0))
seated = timeline.settled_at("part")
if seated is not None:
    (_, _, z), _ = timeline.object_pose("part", timeline.duration)
    print(f"  part seated at z={z:.3f} (settled at t={seated:.2f}s, "
          f"dropped the authored {1000 * SEAT_GAP:.0f} mm gap)")

# --- the run where the part never arrived -------------------------------
# Same taught cycle, but the part is still in the warehouse. The weld
# cannot notice; the contact record can.
scene.add_scenario("part_missing", obstacles={"part": (1.05, -0.8, PART / 2)})
missing = scene.simulate_sequence("cycle", physics=True, scenario="part_missing",
                                  max_duration=60.0)
print("scenario part_missing:")
show(missing.grasp_report())

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "gripper_pick.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

if "--studio" in sys.argv:
    bt.studio(scene)
