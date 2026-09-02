"""A multi-finger hand wraps a can (design-grasping.md G2).

The tool is a catalog Unitree Dex3-1 — three articulated fingers, 7 DOF,
`gripper.multifinger` — welded onto a UR5e. Everything the grasp needs
comes from the package: the mount face and grasp-centre TCP
(`frames.mount_frame` / `tcp_default`), the fingertip frames
(`robot.grasp_frames`), and the holding checks' numbers (`payload_kg`
0.5 — Unitree publishes no fingertip force in newtons, so the report's
grip-force check honestly skips).

The close is derived per finger: `grasp_close` sweeps each finger chain
and stops at its *first touch* of the can — index and middle solve from
their joint limits alone, and the 3-DOF thumb gets its fully-closed pose
from a small scan (the `closed=` escape hatch: a wrapping thumb's closed
pose is authoring, the touch along the way is derivation).

Three cell-authoring lessons this demo bakes in, each caught during its
own authoring by `tl.grasp_report()`, the contact record, or a live
collision scan of the baked timeline:

* IK solves the FULL DOF vector, fingers included — teach poses as
  `[*ik[:6], fingers...]` or the "open" hand arrives pre-curled;
* travel at hover height and change height only in short vertical hops:
  one big joint-space swing to the seat SAGS mid-arc (measured: the
  carried can 46 mm through the plate, the lower finger with it), and
  physics says nothing — hand and held part are kinematic, and
  kinematic-vs-static contact is neither solved nor recorded;
* retreat before homing — the return sweep of an open hand passes
  exactly where the can was just seated (an earlier bake of this file
  knocked it to the floor at t=6.4 s).

Requires the published catalog (`unitree/dex3/dex3-1-right/r1`).

Run with:  python examples/basics/hand_grasp_demo.py [out.usda] [--studio]
"""

import math
import sys

import botrail as bt

BASE_Z = 0.74
STAND_TOP = 0.55
CAN_R, CAN_H = 0.03, 0.12
CAN = (0.55, 0.0, STAND_TOP + CAN_H / 2)
# Aim the grasp centre a shade behind/beside the can axis so the OPEN
# (flat) fingers clear the can surface — a wrap taught dead-centre sweeps
# the fingers through the can it just seated when they straighten.
AIM = (CAN[0] - 0.015, CAN[1] - 0.012, CAN[2])
PLACE = (0.42, -0.35, CAN[2] + 0.003)
UP = (0.0, 0.0, 0.0, 1.0)  # palm axes world-aligned: fingers +X at the can

arm = bt.Robot.from_catalog("ur5e")
hand = bt.Robot.from_catalog("dex3-1-right")
robot = arm.attach_tool(hand)  # mount face and grasp-centre TCP from the manifest
print(f"fingertips: {robot.grasp_frames}")
scene = bt.Scene(robot, base_position=(0.0, 0.0, BASE_Z))
names = robot.joint_names
FINGERS = len(names) - 6
READY = [0.0, -1.9, 1.8, -1.5, -1.57, 0.0] + [0.0] * FINGERS
scene.set_joint_positions(READY)

scene.add_box("floor", size=(2.4, 2.4, 0.05), position=(0.2, 0, -0.025),
              color=(0.35, 0.37, 0.40))
scene.add_box("stand", size=(0.16, 0.16, STAND_TOP), position=(CAN[0], CAN[1], STAND_TOP / 2),
              color=(0.45, 0.47, 0.50))
scene.add_box("plate", size=(0.2, 0.2, STAND_TOP - 0.003),
              position=(PLACE[0], PLACE[1], (STAND_TOP - 0.003) / 2), color=(0.80, 0.65, 0.15))
scene.add_cylinder("can", CAN_R, CAN_H, CAN, color=(0.30, 0.55, 0.80))
scene.set_physics("can", dynamic=True, mass=0.4, friction=0.6)


def ik(target):
    # IK works the FULL DOF vector — finger joints included — and its
    # restarts leave them wherever a seed put them. Keep the arm's six and
    # pin the fingers back open.
    scene.set_joint_positions(READY)
    if not scene.set_tcp_target(target, UP).converged:
        raise SystemExit(f"cell layout unreachable at {target}")
    return [*list(scene.joint_positions)[:6], *([0.0] * FINGERS)]


HOVER = 0.14
PLACE_AIM = (PLACE[0] - 0.015, PLACE[1] - 0.012, PLACE[2])
q_grip = ik(AIM)
q_over_grip = ik((AIM[0], AIM[1], AIM[2] + HOVER))
q_place = ik(PLACE_AIM)
q_over_place = ik((PLACE_AIM[0], PLACE_AIM[1], PLACE_AIM[2] + HOVER))

# The thumb's fully-closed pose: scanned so its tip sweep passes the can
# surface. Index and middle need nothing — their limit ends already sweep
# through the can, and grasp_close stops each at its first touch.
qi = {n: i for i, n in enumerate(names)}
scene.set_joint_positions(q_grip)
best = None
for t0 in (-0.5, 0.0, 0.5):
    for k1 in range(9):
        for k2 in range(9):
            t1 = -0.92 + (0.72 + 0.92) * k1 / 8
            t2 = -1.74 * k2 / 8
            q = list(q_grip)
            q[qi["right_hand_thumb_0_joint"]] = t0
            q[qi["right_hand_thumb_1_joint"]] = t1
            q[qi["right_hand_thumb_2_joint"]] = t2
            scene.set_joint_positions(q)
            (x, y, z), _ = scene.link_pose("thumb_tip")
            radial = math.hypot(x - CAN[0], y - CAN[1])
            cost = abs(radial - 0.028) + max(0.0, abs(z - CAN[2]) - 0.05) * 3
            if best is None or cost < best[0]:
                best = (cost, t0, t1, t2)
scene.set_joint_positions(q_grip)
q_close = scene.grasp_close("can", closed={
    "right_hand_thumb_0_joint": best[1],
    "right_hand_thumb_1_joint": best[2],
    "right_hand_thumb_2_joint": best[3],
})
print("derived close:", {k.removeprefix("right_hand_"): round(v, 3) for k, v in q_close.items()})
scene.set_joint_positions(READY)

ramp = lambda q, s: bt.seq.ramp(dict(zip(names, q)), s)  # noqa: E731
fingers = lambda base: [*base[:6], *[q_close.get(n, 0.0) for n in names[6:]]]  # noqa: E731
open_pose = {k: 0.0 for k in q_close}

# Every travelling move runs at hover height and every height change is a
# short vertical hop between two nearby IK solutions. A single big
# joint-space swing straight to the seat SAGS below the target mid-arc —
# the first bake of this file drove the carried can 46 mm *through* the
# plate (and the lower finger with it), and physics says nothing: an
# attached part and the hand are kinematic, and kinematic-vs-static
# contact is neither solved nor recorded.
sq = scene.sequence("cycle")
sq.step("approach", actions=[ramp(q_over_grip, 1.2)])
sq.step("reach", actions=[ramp(q_grip, 0.8)])
sq.step("close", actions=[bt.seq.ramp(q_close, 0.6)],
        transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(0.75)))
sq.step("grab", actions=[bt.seq.attach("can", touch_links="tool")])
sq.step("lift", actions=[ramp(fingers(q_over_grip), 0.9)])
sq.step("place", actions=[ramp(fingers(q_over_place), 1.4)])
sq.step("seat", actions=[ramp(fingers(q_place), 0.8)])
sq.step("open", actions=[bt.seq.ramp(open_pose, 0.5)])
sq.step("drop", actions=[bt.seq.detach("can")], transition=bt.seq.elapsed(0.8))
# Retreat straight up before homing — the return sweep of an open hand
# passes exactly where the can now stands.
sq.step("retreat", actions=[ramp(q_over_place, 0.8)])
sq.step("home", actions=[ramp(READY, 1.2)],
        transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(1.0)))

timeline = scene.simulate_sequence("cycle", physics=True, max_duration=60.0)
print(f"baked {timeline.duration:.2f}s under physics={timeline.physics!r}")
for rep in timeline.grasp_report():
    tips = ", ".join(sorted(link.removeprefix("right_hand_") for link in rep["touched"]))
    print(f"  grasp of {rep['object']}: {len(rep['touched'])} finger links touching — {tips}")
    print(f"    checks: {rep['checks']}  (payload limit {rep['payload_limit_kg']} kg "
          f"from the catalog specs; grip force unpublished → skip)")
(x, y, z), _ = timeline.object_pose("can", timeline.duration)
print(f"  can seated at ({x:+.3f}, {y:+.3f}, {z:.3f}) — target plate ({PLACE[0]}, {PLACE[1]})")

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "hand_grasp.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

if "--studio" in sys.argv:
    bt.studio(scene)
