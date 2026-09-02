"""Friction grasp: the hold is physics, not a weld (design-grasping.md G3).

`gripper_pick_demo.py` teaches this same cell with attach-as-weld: the
part rides the TCP because the bake glues it there, and the report reads
contact facts around that weld. Here the weld is gone:

* `scene.set_gripper_drive()` turns the 2F-85's finger links into dynamic
  bodies moved by force-capped position motors (each with a real moving
  mass — contact stiffness scales with the pair's masses);
* the close is taught 2 mm PAST touch (`clearance=-0.002`): a
  half-millimetre kiss reads as "touching" but develops well under a
  newton of clamp;
* `attach` becomes a hold *declaration*: the part's track stays
  physics-sampled end to end, so what friction cannot carry drops for
  real, and `tl.grasp_report()` grows a measured `slip_m` and a `hold`
  check.

Two bakes of the same taught cycle: a feeble 0.18 N*m cap closes the
hand — it even carries the fingers' own weight, just barely — but the
squeeze it buys is a couple of newtons, and the lift loses the part
back onto its stand. The stock drive (the knuckle's own effort limit)
carries it to the plate. Same authoring — the physics decides. The
stock bake runs LAST on purpose: the studio replays the most recent
bake, so `--studio` opens on the cycle that works.

Teaching notes, both measured on the catalog 2F-85:

* grip the upper flank SHALLOW (pad midpoint ~10 mm below the part's
  top). The URDF approximates the four-bar with mimic joints, so the
  inner-knuckle bars are rigid where the real linkage pivots away — a
  deeper grip lands the BARS on the part's corners and props the pads
  2 mm open (bars 9.5 N, pads 3 N, and the carry fails);
* half a millimetre of overtravel is right for *recording* a touch under
  a weld (G1); holding by friction needs ~2 mm.

Run with:  python examples/basics/friction_grasp_demo.py [out.usda] [--studio]
"""

import sys

import botrail as bt

# --- the gripper_pick_demo cell, unchanged --------------------------------
BASE_Z = 0.74
PART = 0.06
STAND_XY = (0.45, 0.28)
STAND_TOP = 0.40
PLATE_XY = (0.45, -0.34)
PLATE_TOP = STAND_TOP - 0.005
HOVER = 0.12
SEAT_GAP = 0.005

DOWN = (1.0, 0.0, 0.0, 0.0)
OPEN = 0.0
READY = [0.0, -1.9, 1.8, -1.5, -1.57, 0.0, OPEN]

arm = bt.Robot.from_catalog("ur5e")
coupling = bt.Robot.from_catalog("gripper-coupling")
gripper = bt.Robot.from_catalog("2f-85")
scene = bt.Scene(arm.attach_tool(coupling, prefix="cpl_").attach_tool(gripper),
                 base_position=(0.0, 0.0, BASE_Z))
scene.set_joint_positions(READY)

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
for side in ("left", "right"):
    scene.set_link_material(f"{side}_inner_finger_pad", friction=1.1)

# --- the one line that changes the physics of holding ---------------------
# Fingers become force-limited dynamic bodies; the cap defaults to the
# knuckle's URDF effort limit, stiffness/damping/finger_mass to measured
# defaults. Everything taught below is ordinary G1 authoring.
scene.set_gripper_drive()

# --- teach (gripper_pick_demo's poses, G3-adjusted) ------------------------
GRIP_DEPTH = 0.010  # shallow: keeps the mimic-rigid knuckle bars off the part
GRIP = (STAND_XY[0], STAND_XY[1], STAND_TOP + PART - GRIP_DEPTH)
HANG = GRIP[2] - STAND_TOP
SEAT = (PLATE_XY[0], PLATE_XY[1], PLATE_TOP + HANG + SEAT_GAP)


def at(position, finger):
    scene.set_joint_positions(READY)
    if not scene.set_tcp_target(position, DOWN).converged:
        raise SystemExit(f"cell layout unreachable at {position}")
    return [*scene.joint_positions[:6], finger]


scene.set_joint_positions(at(GRIP, OPEN))
q_close = scene.grasp_close("part", clearance=-0.002)
(finger_joint, close_value), = q_close.items()
print(f"taught close: {finger_joint} = {close_value:.3f} rad (2 mm overtravel)")

names = scene.robot.joint_names
ramp = lambda goal, s: bt.seq.ramp(dict(zip(names, goal)), s)  # noqa: E731

approach = at((GRIP[0], GRIP[1], GRIP[2] + HOVER), OPEN)
descend = at(GRIP, OPEN)
carry = [*approach[:6], close_value]
over_seat = at((SEAT[0], SEAT[1], SEAT[2] + HOVER), close_value)
seat = at(SEAT, close_value)
clear = at((SEAT[0], SEAT[1], SEAT[2] + HOVER), OPEN)
scene.set_joint_positions(READY)

sq = scene.sequence("cycle")
sq.step("approach", actions=[ramp(approach, 1.2)])
sq.step("descend", actions=[ramp(descend, 0.8)])
sq.step("close", actions=[bt.seq.ramp(q_close, 0.5)],
        transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(0.65)))
sq.step("grab", actions=[bt.seq.attach("part", touch_links="tool")])
sq.step("lift", actions=[ramp(carry, 0.9)])
sq.step("place", actions=[ramp(over_seat, 1.4)])
sq.step("seat", actions=[ramp(seat, 0.8)])
sq.step("open", actions=[bt.seq.ramp({finger_joint: OPEN}, 0.4)])
sq.step("release", actions=[bt.seq.detach("part")], transition=bt.seq.elapsed(0.6))
sq.step("retreat", actions=[ramp(clear, 0.8)])
sq.step("home", actions=[ramp(READY, 1.2)],
        transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(1.0)))


def show(tl, label):
    (rep,) = tl.grasp_report(grip_force_n=20.0, mu=0.6, payload_kg=5.0)
    pads = ", ".join(f"{link} ({force:.1f} N)"
                     for link, force in rep["touched"].items() if force > 0.5)
    print(f"{label}:")
    print(f"  touching: {pads or 'nothing above 0.5 N'}")
    print(f"  slip {1000 * rep['slip_m']:.1f} mm -> hold: {rep['checks']['hold']}")
    seated = tl.settled_at("part")
    (x, y, z), _ = tl.object_pose("part", tl.duration)
    # The stand and plate seats differ by 5 mm in z — tell them apart by
    # where the part is, not how high.
    on_plate = (abs(x - PLATE_XY[0]) < 0.11 and abs(y - PLATE_XY[1]) < 0.11
                and abs(z - (PLATE_TOP + PART / 2)) < 0.02)
    near_stand = abs(x - STAND_XY[0]) < 0.12 and abs(y - STAND_XY[1]) < 0.12
    where = ("on the plate" if on_plate
             else "back at the stand" if near_stand
             else f"at ({x:.2f}, {y:.2f}, z={z:.3f})")
    print(f"  part ends {where}" + (f", settled at t={seated:.2f}s" if seated else ""))
    rep["_on_plate"] = on_plate
    return rep


# The feeble hand first: redeclaring the drive replaces it, the taught
# sequence is untouched. 0.18 N*m at the knuckle still closes the
# fingers (below ~0.15 they cannot even hold their own weight and the
# hand visibly dangles — that reads as a broken robot, not a weak grip),
# but the squeeze it buys is a couple of newtons, under what 0.35 kg
# needs at this carry's 2 m/s2 peak.
scene.set_gripper_drive(max_force=0.18)
feeble = scene.simulate_sequence("cycle", physics=True, max_duration=60.0)
rep = show(feeble, "feeble drive (max_force = 0.18 N*m)")
assert rep["checks"]["hold"] == "fail" and not rep["_on_plate"]

# The stock drive, baked LAST: the studio replays the latest bake, so a
# connected (or --studio) viewer sees the cycle that works, and the USD
# below exports it.
scene.set_gripper_drive()
timeline = scene.simulate_sequence("cycle", physics=True, max_duration=60.0)
rep = show(timeline, "stock drive (cap = knuckle effort limit)")
assert rep["checks"]["hold"] == "pass" and rep["_on_plate"]

args = [a for a in sys.argv[1:] if not a.startswith("--")]
out = args[0] if args else "friction_grasp.usda"
warnings = timeline.export_usd(out, fps=30.0)
print(f"wrote {out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

if "--studio" in sys.argv:
    bt.studio(scene)
