"""A semi-humanoid taking the bottom and the top shelf of a bay.

A humanoid upper body on a wheeled base is bought for one thing a cobot on
a cart cannot do: it works from the floor to over its head, because its
torso moves its arms' bases. To a cell it is nothing new — a vehicle with
no body whose running gear is the robot itself (`mount_robot(wheels=)`),
and a robot whose joints fall into groups: two arms, a torso, a head. This
cell asks the four questions a cell asks of any mobile machine, and the one
this kind adds:

  * **通れるか** — the aisle is `--aisle` wide; the machine drives it in its
    travel posture, pivots at the bay, and the check is its own links
    against the racks (there is no footprint box to get wrong).
  * **届くか、高さ方向に** — the low board and the top board of one bay.
    Neither is inside the arms' reach from where the torso travels: the
    torso goes down for one and up for the other, and *then* the arm is
    planned. Teaching tries the machine's torso postures gentlest first
    and keeps the first the hand reaches from. The derived requirement is
    `vertical_reach_min_mm` / `vertical_reach_max_mm`: the lowest and the
    highest taught hand position over the floor.
  * **干渉しないか** — every arm motion is planned against the bay, the
    other arm and the machine's own base; the torso ramps are the author's
    and are checked along their sweep before the bake (`ramp_contacts` —
    it is what says a bowed torso must straighten *before* the column
    rises, or the head comes up under the top board).
  * **何秒か** — drive + torso + arms, one number.

The cycle, one trip for two cartons — one per hand:

  * `to bay` — backwards out of the dock, down the aisle, a quarter turn,
    nose first up to the bay.
  * `look low` / `look top` — the head camera is an input: the step waits
    for the vision sensor to see the carton it is about to take. The low
    one needs a glance down; the top one stands on a board over the
    machine's eyes and comes into view only as the torso rises. A bay that
    was not replenished stalls the cycle here, by name.
  * `torso down` → `pick low` (right hand, down onto the carton — the low
    board is open above) → `straighten` → `torso up` → `pick top` (left
    hand, straight in over the board and straight out). An arm is planned
    without the torso — the other arm's base must not move under it — so
    the torso is ramped first.
  * `to dock` — the vehicle drives and the torso folds to its travel
    height *on the move*: ramps bake alongside the rolling wheels, so the
    fold costs no cycle time.
  * `set down` — both arms at once: the cartons onto the hand-off stand,
    hands open, arms back.

`--robot g1d` (the default) is a Unitree G1-D from the catalog: a
two-stage column, a waist that bows and turns, two 7-axis arms with
three-finger hands, a differential base — fetched once into the botrail
cache. It reaches down by bowing 115 degrees into the bay. `--robot rby1`
is a Rainbow Robotics RB-Y1 A: its torso is a six-joint leg, and it makes
height by squatting. `--robot semi` runs the same cell on the primitive
machine in `examples/assets/semi_humanoid_test.urdf` with no download (what
the tests use): a lift column and 4-axis arms. The bay is each machine's
own — its boards stand where that machine's torso has to move to reach them.

`--robot rby1m` (the RB-Y1 on mecanum wheels), `--robot ffw` (a ROBOTIS AI
Worker FFW-SG2: a lift column on three swerve modules), `--robot galbot` (a
Galbot G1: a five-joint torso on four omni wheels) and `--robot r1pro` (a
Galaxea R1 Pro: a four-joint torso on three swerve modules) have bases that do
not turn — a holonomic machine docks facing whatever it faced when parked. Their
cell is laid out for that: the hand-off stand is on the bay's side of the
aisle, the machine parks nose to the racks and crabs between the two.

`--compare` is the buyer's question: **one** bay — `--robot`'s — and every
machine in front of it. One row each: the torso posture teaching chose for
either board, the cycle time, and for the ones that do not make it the
reason — a board no torso posture puts the hand on, and how far off the best
one is.

Run with:  python examples/vehicles/semi_humanoid_demo.py [out.usdc]
               [--robot g1d|rby1|rby1m|ffw|galbot|r1pro|semi] [--aisle 1.4] [--compare]
               [--deliverables DIR] [--studio]

`--deliverables DIR` writes the hand-over set from the same bake — project,
generated script, BOM, I/O list, PLCopen, interlocks, layout sheet, USD and
the cell report (`bt.export_cell`).
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, replace
from pathlib import Path

import botrail as bt

S = bt.seq
ASSETS = Path(__file__).resolve().parents[1] / "assets"
ROBOT = "machine"
BASE = "base"
LINE = "cartesian_line"
PROBE = "line?"            # a motion teaching authors to ask the planner a question, and removes

STEEL = (0.07, 0.16, 0.23)
CARTON = (0.72, 0.55, 0.34)
DECK = (0.56, 0.63, 0.66)
BEAM = (0.84, 0.32, 0.07)
TEAL = (0.07, 0.43, 0.43)
YELLOW = (0.92, 0.65, 0.16)


# ------------------------------------------------------------ the machines
@dataclass(frozen=True)
class Machine:
    """What the cell has to know about one machine that the model does not
    say: which arm takes which board, how the torso may stand for each,
    where the head looks, how the hands open."""

    key: str
    title: str
    #: Which arm takes the bottom board, which the top one.
    arms: dict
    #: The torso at rest, and — per task (`low`, `top`, `stand`) — the
    #: postures it may take, gentlest first: teaching takes the first one the
    #: arm reaches its targets from.
    travel: dict
    torso: dict
    #: Joints -> value that point the head camera at the low / top carton —
    #: one glance or, gentlest first, the ones teaching may choose from — and
    #: back (`ahead`). A head's joints, or — for a fixed head — the torso's.
    look: dict
    #: Arm -> joints -> value: the hand open, and closed on a carton.
    hands_open: dict
    hands_shut: dict
    #: Arm -> the links that touch a held carton.
    touch: dict
    #: Arm -> the link a carton is held at (the arm's tip when `None`).
    grasp: dict
    #: That link's orientation at every taught pose, in the machine's own
    #: frame — the hand level, pointing straight ahead.
    approach: tuple
    #: Arm -> where the carry holds that link, in the machine's own frame,
    #: over the travel torso; and arm seeds the IK starts from.
    carry: dict
    seeds: dict
    #: The head camera: link, offset, orientation (-Z looks), horizontal fov.
    camera: tuple
    #: The bay this machine serves: the heights of its low and its top board.
    boards: tuple
    #: (across, along the approach, height) of a carton.
    carton: tuple
    #: Station -> bay front, metres ahead of the machine's centre; and the
    #: same to the stand's front at the dock — further, because the machine
    #: arrives there with a carton in each hand, ahead of its chest.
    standoff: float
    dock_standoff: float
    #: How far inside the front edge a carton stands, and how far a hand
    #: backs straight out of the bay before it turns away: the carton it
    #: holds has to clear the board's front edge, or the planner is left to
    #: find a way around it.
    inset: float
    pull: float
    #: How far inside the stand's front edge a carton is set down.
    stand_inset: float
    #: Half the distance between the two cartons (the shoulders' half width).
    span: float
    stand_top: float
    speed: float
    turn: float
    #: Seconds a torso move / a glance / a hand takes.
    paces: tuple = (2.5, 0.8, 0.4)
    #: The catalog package the machine comes from; none for the offline one.
    package: str = ""
    #: A base that does not turn (mecanum, swerve): it docks facing whatever
    #: it faced when parked, so its cell puts the hand-off stand on the bay's
    #: side of the aisle and the machine crabs between the two, nose to the racks.
    holonomic: bool = False


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def yaw_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (y * vz - z * vy), 2 * (z * vx - x * vz), 2 * (x * vy - y * vx)
    return (vx + w * tx + y * tz - z * ty, vy + w * ty + z * tx - x * tz, vz + w * tz + x * ty - y * tx)


# Tool +Z ahead (+x), tool +Y to the machine's right: the primitive
# machine's hands reach this with the arm in its own plane, fingers closing
# side to side.
AHEAD = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
LEVEL = (0.0, 0.0, 0.0, 1.0)
# A camera looks along its -Z with +Y up; on a link whose +X is ahead and
# +Z is up that is this offset.
CAMERA_AHEAD = (0.5, -0.5, -0.5, 0.5)
SIDES = ("left", "right")

SEMI = Machine(
    key="semi",
    title="primitive semi-humanoid (examples/assets/semi_humanoid_test.urdf)",
    arms={"low": "right", "top": "left"},
    travel={"lift_joint": 0.05, "waist_yaw_joint": 0.0},
    torso={"low": [{"lift_joint": 0.0}], "top": [{"lift_joint": 0.40}], "stand": [{"lift_joint": 0.10}]},
    look={"low": {"head_pan_joint": -0.45, "head_tilt_joint": 0.75},
          "top": {"head_pan_joint": 0.45, "head_tilt_joint": -0.35},
          "ahead": {"head_pan_joint": 0.0, "head_tilt_joint": 0.0}},
    hands_open={side: {f"{side}_finger": 0.03} for side in SIDES},
    hands_shut={side: {f"{side}_finger": 0.022} for side in SIDES},
    touch={side: [f"{side}_hand", f"{side}_finger_a", f"{side}_finger_b"] for side in SIDES},
    grasp={side: None for side in SIDES},
    approach=AHEAD,
    carry={"left": (0.30, 0.22, 0.89), "right": (0.30, -0.22, 0.89)},
    seeds={side: [{f"{side}_shoulder_pitch": -0.35, f"{side}_elbow": -1.75, f"{side}_wrist": 0.53},
                  {f"{side}_shoulder_pitch": -1.0, f"{side}_elbow": -1.2, f"{side}_wrist": 0.6}]
           for side in SIDES},
    camera=("head_camera", (0.0, 0.0, 0.0), CAMERA_AHEAD, 70.0),
    boards=(0.75, 1.50),
    carton=(0.06, 0.06, 0.10),
    standoff=0.34,
    dock_standoff=0.42,
    inset=0.10,
    pull=0.15,
    stand_inset=0.10,
    span=0.22,
    stand_top=0.85,
    speed=0.5,
    turn=math.pi / 2,
)

# r2: the revision that states the head camera frame the cell mounts its camera on.
G1D_ID = "unitree/g1/g1-d/r2"
G1D = Machine(
    key="g1d",
    title=f"Unitree G1-D (catalog `{G1D_ID}`)",
    arms={"low": "right", "top": "left"},
    travel={"LZ_mt_Joint": 0.0, "LZ_it_Joint": 0.0, "Yaw_Joint": 0.0, "torso_Joint": 0.0},
    # `Yaw_Joint` is the waist's *pitch* (the vendor's name): this machine
    # reaches down by bowing, and up by its two-stage column. The bows run
    # from none at all: in front of another machine's bay (`--compare`) the
    # low board may be one it reaches standing up.
    torso={"low": [{"Yaw_Joint": bow} for bow in (0.0, 0.4, 0.8, 1.2, 1.4, 1.6, 1.8, 2.0, 2.2)],
           "top": [{"LZ_mt_Joint": 0.21, "LZ_it_Joint": 0.21}],
           "stand": [{}, {"Yaw_Joint": 0.3}, {"Yaw_Joint": 0.6}]},
    # The head is fixed: the torso aims it — a bow to look down at the low
    # board; the top one comes into view as the column rises.
    look={"low": [{"Yaw_Joint": bow / 10} for bow in range(3, 12)], "top": {}, "ahead": {}},
    hands_open={side: {} for side in SIDES},
    hands_shut={
        "right": {"right_hand_index_0_joint": 0.5, "right_hand_index_1_joint": 0.4,
                  "right_hand_middle_0_joint": 0.5, "right_hand_middle_1_joint": 0.4,
                  "right_hand_thumb_1_joint": -0.3, "right_hand_thumb_2_joint": -0.6},
        "left": {"left_hand_index_0_joint": -0.5, "left_hand_index_1_joint": -0.4,
                 "left_hand_middle_0_joint": -0.5, "left_hand_middle_1_joint": -0.4,
                 "left_hand_thumb_1_joint": 0.3, "left_hand_thumb_2_joint": 0.6},
    },
    touch={side: [f"{side}_hand_palm_link"]
           + [f"{side}_hand_{finger}_link" for finger in
              ("thumb_0", "thumb_1", "thumb_2", "index_0", "index_1", "middle_0", "middle_1")]
           for side in SIDES},
    # The package's grasp frames: the fingertips' centre, beside the fingers
    # on the thumb's side — where a carton sits in an open hand.
    grasp={side: f"{side}_grasp_center" for side in SIDES},
    approach=LEVEL,
    carry={"left": (0.30, 0.20, 0.98), "right": (0.30, -0.20, 0.98)},
    seeds={side: [{f"{side}_shoulder_pitch_joint": -0.5, f"{side}_elbow_joint": 0.6},
                  {f"{side}_shoulder_pitch_joint": -1.2, f"{side}_elbow_joint": 0.3},
                  {f"{side}_shoulder_pitch_joint": 0.3, f"{side}_elbow_joint": 1.0},
                  {f"{side}_shoulder_pitch_joint": -2.0, f"{side}_elbow_joint": 0.2},
                  {f"{side}_shoulder_pitch_joint": -1.0, f"{side}_elbow_joint": 1.5}]
           for side in SIDES},
    # The package's head camera frame (from r2 on): the vendor's frame of the
    # legged G1's head camera, carried over on the head part the two machines
    # share — it looks 47.6 degrees down. The vendor does not say which camera
    # the G1-D carries, so the field of view is the cell's guess.
    camera=("head_camera", (0.0, 0.0, 0.0), CAMERA_AHEAD, 90.0),
    boards=(0.25, 1.45),
    carton=(0.04, 0.06, 0.10),
    standoff=0.40,
    dock_standoff=0.40,
    inset=0.12,
    pull=0.17,
    stand_inset=0.05,
    span=0.20,
    stand_top=0.85,
    speed=0.8,
    turn=1.2,
    package=G1D_ID,
)


def squat(depth: float, bow: float = 0.0) -> dict:
    """RB-Y1's torso is a leg: ankle, knee and hip pitch. Ankle and hip the
    same way and the knee twice the other lowers the chest and keeps it
    upright and over the wheels; `bow` leans it forward at the hip."""
    return {"torso_1": depth, "torso_2": -2 * depth, "torso_3": depth + bow}


# The exact revision, not the newest: r1 checks its wrist with the vendor's
# capsule, which wraps the whole hand and whatever it holds (radius 75 mm, to
# 30 mm past the TCP) — a hand that cannot come within 75 mm of a board takes
# nothing off one. r2 gives the wrist its own shape.
RBY1_ID = "rainbow_robotics/rb-y1/rb-y1-a/r2"
RBY1 = Machine(
    key="rby1",
    title=f"Rainbow Robotics RB-Y1 A (catalog `{RBY1_ID}`)",
    arms={"low": "right", "top": "left"},
    travel=squat(0.0),
    # Down is a squat, deeper and deeper, and at the bottom a bow as well.
    torso={"low": [squat(depth) for depth in (0.0, 0.3, 0.6, 0.9, 1.2)] + [squat(1.2, bow) for bow in (0.2, 0.37)],
           "top": [squat(0.0)],
           "stand": [squat(depth) for depth in (0.0, 0.3, 0.6, 0.8, 1.0)]},
    look={"low": [{"head_1": tilt} for tilt in (0.3, 0.5, 0.7, 0.9)],
          "top": [{}, {"head_1": -0.3}, {"head_1": -0.6}],
          "ahead": {"head_0": 0.0, "head_1": 0.0}},
    # Two fingers, a joint each, shut at zero: one slides to -50 mm, its twin to +50 mm.
    hands_open={side: {f"gripper_finger_{side[0]}1": -0.05, f"gripper_finger_{side[0]}2": 0.05} for side in SIDES},
    hands_shut={side: {f"gripper_finger_{side[0]}1": -0.034, f"gripper_finger_{side[0]}2": 0.034} for side in SIDES},
    touch={side: [f"ee_{side}", f"ee_finger_{side[0]}1", f"ee_finger_{side[0]}2"] for side in SIDES},
    grasp={side: None for side in SIDES},
    # Tool +Z ahead, tool +X — the fingers' stroke — to the left: they close
    # on a carton's sides, not over and under it.
    approach=(0.5, 0.5, 0.5, 0.5),
    # High, for an arm this long: a straight line from the top board down to
    # a carry at 1.0 m runs out of this arm's posture part way.
    carry={"left": (0.30, 0.22, 1.20), "right": (0.30, -0.22, 1.20)},
    seeds={side: [{f"{side}_arm_0": -0.4, f"{side}_arm_3": -1.6, f"{side}_arm_6": 1.57},
                  {f"{side}_arm_0": -0.4, f"{side}_arm_3": -1.6, f"{side}_arm_6": -1.57},
                  {f"{side}_arm_0": -1.2, f"{side}_arm_3": -1.0, f"{side}_arm_6": 1.57},
                  {f"{side}_arm_0": 0.3, f"{side}_arm_3": -2.2, f"{side}_arm_6": 1.57}]
           for side in SIDES},
    # The package states no camera frame: this cell's reading of the head.
    camera=("link_head_2", (0.05, 0.0, 0.0), CAMERA_AHEAD, 90.0),
    boards=(0.75, 1.50),
    carton=(0.06, 0.06, 0.10),
    # The wheels are under the machine's nose — the column stands 0.23 m
    # behind them — so everything ahead is that much further from the shoulder.
    standoff=0.40,
    dock_standoff=0.40,
    inset=0.10,
    pull=0.17,
    stand_inset=0.06,
    span=0.22,
    stand_top=0.85,
    speed=0.8,
    turn=1.2,
    package=RBY1_ID,
)

# The same upper body on four mecanum wheels. V1.3's hand is another one: its
# fingers stroke along the tool's Y, and its column stands over the wheels'
# centre, not behind it.
RBY1M_ID = "rainbow_robotics/rb-y1/rb-y1-m/r2"
RBY1M = replace(
    RBY1,
    key="rby1m",
    title=f"Rainbow Robotics RB-Y1 M (catalog `{RBY1M_ID}`)",
    approach=AHEAD,
    # ...and its finger joints count the other way round (…1: 0..+50 mm, …2: -50..0).
    hands_open={side: {f"gripper_finger_{side[0]}1": 0.05, f"gripper_finger_{side[0]}2": -0.05} for side in SIDES},
    hands_shut={side: {f"gripper_finger_{side[0]}1": 0.034, f"gripper_finger_{side[0]}2": -0.034} for side in SIDES},
    standoff=0.50,
    dock_standoff=0.50,
    carry={"left": (0.40, 0.22, 1.20), "right": (0.40, -0.22, 1.20)},
    package=RBY1M_ID,
    holonomic=True,
)

FFW_ID = "robotis/ai-worker/ffw-sg2"
FFW = Machine(
    key="ffw",
    title=f"ROBOTIS AI Worker FFW-SG2 (catalog `{FFW_ID}`)",
    arms={"low": "right", "top": "left"},
    # A lift column whose zero is its top: down is negative.
    travel={"lift_joint": 0.0},
    torso={"low": [{"lift_joint": -drop} for drop in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)],
           "top": [{"lift_joint": 0.0}],
           "stand": [{"lift_joint": -drop} for drop in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5)]},
    look={"low": [{"head_joint1": tilt} for tilt in (0.2, 0.4, 0.6, 0.69)],
          "top": [{}, {"head_joint1": -0.2}],
          "ahead": {"head_joint1": 0.0, "head_joint2": 0.0}},
    # RH-P12-RN: one driven joint a hand, open at zero; its three twins are mimics.
    hands_open={side: {f"gripper_{side[0]}_joint1": 0.0} for side in SIDES},
    hands_shut={side: {f"gripper_{side[0]}_joint1": 0.45} for side in SIDES},
    touch={side: [f"gripper_{side[0]}_rh_p12_rn_{part}" for part in ("base", "r1", "r2", "l1", "l2")] for side in SIDES},
    grasp={side: None for side in SIDES},
    approach=AHEAD,
    carry={"left": (0.32, 0.23, 1.10), "right": (0.32, -0.23, 1.10)},
    seeds={side: [{f"arm_{side[0]}_joint1": -0.3, f"arm_{side[0]}_joint4": -1.6},
                  {f"arm_{side[0]}_joint1": -1.0, f"arm_{side[0]}_joint4": -1.2},
                  {f"arm_{side[0]}_joint1": 0.3, f"arm_{side[0]}_joint4": -2.2}]
           for side in SIDES},
    # The ZED Mini's left eye: a body frame of the vendor's, +X ahead, +Z up.
    camera=("zed_left_camera_frame", (0.0, 0.0, 0.0), CAMERA_AHEAD, 102.0),
    boards=(0.75, 1.50),
    carton=(0.06, 0.06, 0.10),
    standoff=0.45,
    dock_standoff=0.45,
    inset=0.10,
    pull=0.17,
    stand_inset=0.08,
    span=0.23,
    stand_top=0.85,
    speed=0.8,
    turn=1.2,
    package=FFW_ID,
    holonomic=True,
)

def upright(leg1: float, leg3: float) -> dict:
    """Galbot G1's torso is a leg of three pitches (and a yaw and a roll on
    top): the middle one the sum of the other two keeps the chest upright,
    and these pairs keep it over the wheels."""
    return {"leg_joint1": leg1, "leg_joint2": leg1 + leg3, "leg_joint3": leg3}


# Shoulder heights of 1.34 m down to 0.73 m over the floor, a step of about 0.1 m.
GALBOT_LADDER = [(0.8, 1.2), (0.6, 0.9), (0.4, 0.7), (0.3, 0.5), (0.2, 0.3), (0.2, 0.1), (0.0, 0.0)]
GALBOT_ID = "galbot/g1/g1"
GALBOT = Machine(
    key="galbot",
    title=f"Galbot G1 (catalog `{GALBOT_ID}`)",
    arms={"low": "right", "top": "left"},
    travel=upright(0.9, 1.3),
    torso={"low": [upright(0.9, 1.3)] + [upright(*step) for step in GALBOT_LADDER],
           "top": [upright(0.9, 1.4), upright(0.9, 1.3)] + [upright(*step) for step in GALBOT_LADDER[:3]],
           "stand": [upright(0.9, 1.3)] + [upright(*step) for step in GALBOT_LADDER]},
    look={"low": [{"head_joint2": tilt} for tilt in (0.2, 0.35, 0.49)],
          "top": [{}, {"head_joint2": -0.2}],
          "ahead": {"head_joint1": 0.0, "head_joint2": 0.0}},
    # One driven joint a hand, open at zero; the rest of the linkage follows it.
    hands_open={side: {f"{side}_gripper_joint": 0.0} for side in SIDES},
    hands_shut={side: {f"{side}_gripper_joint": 0.9} for side in SIDES},
    touch={side: [f"{side}_gripper_{part}" for part in
                  ("base_link", "l_finger_link", "r_finger_link", "l_knuckle_link", "r_knuckle_link",
                   "l_inner_knuckle_link", "r_inner_knuckle_link")] for side in SIDES},
    grasp={side: None for side in SIDES},
    approach=AHEAD,
    carry={"left": (0.45, 0.21, 1.05), "right": (0.45, -0.21, 1.05)},
    # Its zero is a T-pose: the seeds start from the arm hanging at the side, elbow bent.
    seeds={"right": [{"right_arm_joint1": -1.07, "right_arm_joint2": 1.57, "right_arm_joint4": -bend} for bend in (1.2, 0.6, 1.8)],
           "left": [{"left_arm_joint1": 1.07, "left_arm_joint2": -1.57, "left_arm_joint4": bend} for bend in (1.2, 0.6, 1.8)]},
    # The package has no camera link: the vendor's head mount frame (+X ahead,
    # +Y down) is where this cell puts one. The field of view is the cell's guess.
    camera=("head_end_effector_mount_link", (0.0, 0.0, 0.0), (math.sqrt(0.5), 0.0, -math.sqrt(0.5), 0.0), 90.0),
    boards=(0.75, 1.50),
    carton=(0.06, 0.06, 0.10),
    standoff=0.50,
    dock_standoff=0.50,
    inset=0.10,
    pull=0.17,
    stand_inset=0.08,
    span=0.21,
    stand_top=0.85,
    speed=0.8,
    turn=1.2,
    package=GALBOT_ID,
    holonomic=True,
)

def knees(joint1: float, joint2: float) -> dict:
    """Galaxea R1 Pro's torso is three pitches and a yaw. The third pitch
    turns the other way, so making it the sum of the first two keeps the chest
    upright; these pairs keep it over the base, the knee forward or back."""
    return {"torso_joint1": joint1, "torso_joint2": joint2, "torso_joint3": joint1 + joint2}


# Shoulder heights of 1.34 m down to 0.93 m over the floor (it travels at 1.45 m).
R1PRO_LADDER = [(-0.4, 1.1), (-0.6, 1.6), (-0.7, 2.0), (0.7, -2.3), (0.9, -2.7)]
# A camera on a frame that already is an optical one: +Z looks, +X right, +Y down.
CAMERA_OPTICAL = (1.0, 0.0, 0.0, 0.0)
R1PRO_ID = "galaxea/r1/r1-pro"
# In this cell the R1 Pro does not finish: the procedure holds the torso still
# while an arm works and pulls a carton 0.15 m straight towards the body, and
# this arm's elbow (100 degrees of fold) keeps its wrist more than 0.35 m from
# its shoulder — the reach that is left between too near and too far is a few
# millimetres wide. It stays in the table, where that is the answer.
R1PRO = Machine(
    key="r1pro",
    title=f"Galaxea R1 Pro (catalog `{R1PRO_ID}`)",
    arms={"low": "right", "top": "left"},
    travel=knees(0.0, 0.0),
    torso={"low": [knees(0.0, 0.0)] + [knees(*step) for step in R1PRO_LADDER],
           "top": [knees(0.0, 0.0)] + [knees(*step) for step in R1PRO_LADDER[:2]],
           "stand": [knees(0.0, 0.0)] + [knees(*step) for step in R1PRO_LADDER]},
    # The head is fixed and its camera looks 20 degrees down: a glance further
    # down is a bow at the hip (the third pitch, which counts backwards).
    look={"low": [{}] + [{"torso_joint3": -bow} for bow in (0.15, 0.3, 0.45)], "top": [{}], "ahead": {}},
    # Two fingers, a joint each, shut at zero: one slides to +50 mm, its twin to -50 mm.
    hands_open={side: {f"{side}_gripper_finger_joint1": 0.05, f"{side}_gripper_finger_joint2": -0.05} for side in SIDES},
    hands_shut={side: {f"{side}_gripper_finger_joint1": 0.034, f"{side}_gripper_finger_joint2": -0.034} for side in SIDES},
    touch={side: [f"{side}_gripper_link", f"{side}_gripper_finger_link1", f"{side}_gripper_finger_link2"] for side in SIDES},
    grasp={side: None for side in SIDES},
    approach=AHEAD,
    carry={"left": (0.30, 0.17, 1.10), "right": (0.30, -0.17, 1.10)},
    seeds={side: [{f"{side}_arm_joint1": lift, f"{side}_arm_joint4": -bend}
                  for lift, bend in ((-0.4, 1.4), (-1.0, 1.0), (0.4, 1.4), (-1.6, 0.6))] for side in SIDES},
    # The package's head camera frame — the vendor's, an optical one.
    camera=("zed_link", (0.0, 0.0, 0.0), CAMERA_OPTICAL, 90.0),
    boards=(0.75, 1.50),
    carton=(0.06, 0.06, 0.10),
    # Its elbow folds to 100 degrees and no further: nearer than 0.37 m to the
    # shoulder the wrist does not go, so the machine stands well back and pulls
    # a carton out no further than clears the board's edge.
    standoff=0.50,
    dock_standoff=0.50,
    inset=0.10,
    pull=0.15,
    stand_inset=0.08,
    span=0.17,
    stand_top=0.85,
    speed=0.8,
    turn=1.2,
    package=R1PRO_ID,
    holonomic=True,
)

MACHINES = {m.key: m for m in (G1D, RBY1, RBY1M, FFW, GALBOT, R1PRO, SEMI)}


def load(machine: Machine):
    """The robot with its groups declared, and its wheels."""
    if machine.package:
        # Arms, torso, hands and the torso-with-arm composites come declared
        # in the package; so do the wheels, the base frame and the posture
        # the machine travels in.
        return bt.Robot.from_catalog(machine.package), bt.Wheels.from_catalog(machine.package)
    robot = bt.Robot.from_urdf(ASSETS / "semi_humanoid_test.urdf")
    arm = ("shoulder_pitch", "shoulder_roll", "elbow", "wrist")
    for side in SIDES:
        robot = robot.define_group(side, tip=f"{side}_tcp", flange=f"{side}_flange",
                                   joints=[f"{side}_{j}" for j in arm])
    robot = robot.define_group("torso", tip="chest", joints=["lift_joint", "waist_yaw_joint"])
    robot = robot.define_group("head", tip="head_camera", joints=["head_pan_joint", "head_tilt_joint"])
    wheels = bt.Wheels({"left_wheel_joint": 0.10, "right_wheel_joint": 0.10},
                       base_frame="base_footprint", posture=machine.travel)
    return robot, wheels


# ---------------------------------------------------------------- the cell
# The dock is the path's *end*: a parked vehicle faces the leg it arrived
# by, so the machine starts nose to the hand-off stand, backs out of the
# dock, and turns once — at the bay — to arrive there nose first too.
CORNER_X = 2.4
SPUR = 0.35
BAY = (1.20, 0.40)         # width, depth
BOARD = 0.03
POST = 0.04
STAND = (0.50, 0.90)
# How far over the low carton a hand comes down from.
DROP = 0.12
RESOLUTION = (1280, 720)
STATIONS = {"bay": ((CORNER_X, SPUR), math.pi / 2), "dock": ((0.0, 0.0), math.pi)}
# A holonomic machine never turns: it parks nose to the racks — up a spur of
# its own, because a parked vehicle faces the leg it arrived by — and the
# stand is ahead of it there, as the bay is at the other end of the aisle.
CRAB_STATIONS = {"bay": STATIONS["bay"], "dock": ((0.0, SPUR), math.pi / 2)}


def stations(holonomic: bool = False) -> dict:
    """Station -> ((x, y), heading) for a base that turns, or one that does not."""
    return CRAB_STATIONS if holonomic else STATIONS


def trim(scene: bt.Scene, name: str, size: tuple, position: tuple, color: tuple,
         quaternion=None, metalness: float = 0.0, roughness: float = 0.65) -> None:
    """Static visual detail; only the original posts, decks and cartons collide."""
    scene.add_box(name, size, position, quaternion=quaternion, color=color)
    scene.set_obstacle_enabled(name, False)
    scene.set_obstacle_material(name, metalness=metalness, roughness=roughness)


def add_bay(scene: bt.Scene, name: str, boards: tuple, position: tuple) -> None:
    """An open bay: four posts, a low board and a top board, nothing in
    between — what a machine that bows into a bay needs over the low one.
    `bt.parts.rack` spaces its boards evenly; this cell verifies two
    heights, so the bay is built here, from boxes, under one group."""
    (w, d), (x, y) = BAY, position
    top = boards[-1]
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (-1, 1), (1, 1))):
        px, py = x + sx * (w - POST) / 2, y + sy * (d - POST) / 2
        scene.add_box(f"{name}/post{i}", (POST, POST, top),
                      (px, py, top / 2), color=STEEL)
        scene.set_obstacle_material(f"{name}/post{i}", metalness=0.45, roughness=0.4)
        trim(scene, f"{name}/trim/foot{i}", (0.085, 0.085, 0.012), (px, py, 0.006), STEEL)
        # The perforated faces make the height-adjustable uprights legible.
        for slot in range(1, int(top / 0.09)):
            trim(scene, f"{name}/trim/slot{i}_{slot}", (0.010, 0.001, 0.022),
                 (px, py - POST / 2 - 0.0006, slot * 0.09), (0.025, 0.035, 0.045))
    for tag, height in zip(("low", "top"), boards):
        scene.add_box(f"{name}/board_{tag}", (w - 2 * POST, d, BOARD), (x, y, height - BOARD / 2), color=DECK)
        scene.set_obstacle_material(f"{name}/board_{tag}", metalness=0.35, roughness=0.48)
        for side in (-1, 1):
            trim(scene, f"{name}/trim/{tag}_beam{side}", (w - 2 * POST, 0.025, 0.045),
                 (x, y + side * (d / 2 - 0.015), height - BOARD - 0.0225), BEAM, metalness=0.25)
        # Shelf-edge label holders sit below the deck, outside the grasp.
        trim(scene, f"{name}/trim/{tag}_label", (0.18, 0.003, 0.033),
             (x, y - d / 2 - 0.001, height - BOARD - 0.0225), (0.88, 0.89, 0.84))
        trim(scene, f"{name}/trim/{tag}_swatch", (0.04, 0.001, 0.028),
             (x - 0.064, y - d / 2 - 0.003, height - BOARD - 0.0225), TEAL if tag == "low" else YELLOW)
    # Bracing stays at the sides; the front remains open for the bowed torso.
    rise, run = top - 0.10, d - POST
    angle = math.atan2(rise, run)
    for side in (-1, 1):
        trim(scene, f"{name}/trim/brace{side}", (0.013, math.hypot(run, rise), 0.018),
             (x + side * (w - POST) / 2, y, top / 2), DECK,
             quaternion=(math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2)), metalness=0.6)
    scene.set_part(name, kind="group", category="structure.rack", manufacturer="Generic",
                   model=f"open bay {w * 1e3:.0f} x {d * 1e3:.0f} x {top * 1e3:.0f}")


def dress_cell(scene: bt.Scene, machine: Machine, aisle: float) -> None:
    """Floor paint and stocked shelving, derived from the working cell's dimensions."""
    front = SPUR + machine.standoff
    row_y = front - aisle - BAY[1] / 2
    stand_lo, stand_hi = scene.obstacle_bounds("stand/top")
    xmin = min(-0.65, stand_lo[0]) - 0.35
    xmax = CORNER_X + 1.35
    ymin = min(row_y - BAY[1] / 2, stand_lo[1]) - 0.35
    ymax = max(front + BAY[1], stand_hi[1]) + 0.35
    trim(scene, "floor/slab", (xmax - xmin, ymax - ymin, 0.06),
         ((xmin + xmax) / 2, (ymin + ymax) / 2, -0.029), (0.29, 0.33, 0.35), roughness=0.95)
    # Paint overlays are above the slab and each other to avoid coplanar flicker.
    trim(scene, "floor/aisle", (CORNER_X + 1.15, aisle, 0.001),
         ((CORNER_X - 0.05) / 2, front - aisle / 2, 0.0015), (0.16, 0.23, 0.25), roughness=0.9)
    for edge, y in (("bay", front), ("row", front - aisle)):
        bt.parts.marking(scene, f"floor/edge_{edge}", line=((-0.6, y), (CORNER_X + 0.55, y)),
                         width=0.025, color=YELLOW, floor=0.003, thickness=0.001)
    for i in range(5):
        x = 0.3 + i * 0.40
        # Small chevrons follow the actual straight leg of the vehicle route.
        for side in (-1, 1):
            bt.parts.marking(scene, f"floor/route/{i}_{side}",
                             line=((x - 0.07, side * 0.055), (x, 0.0)), width=0.018,
                             color=(0.53, 0.67, 0.66), floor=0.003, thickness=0.001)
    # A teal pad identifies the receiving stand in either dock layout.
    pad = (stand_lo[0] - 0.12, stand_lo[1] - 0.12, stand_hi[0] + 0.12, stand_hi[1] + 0.12)
    trim(scene, "floor/handover", (pad[2] - pad[0], pad[3] - pad[1], 0.001),
         ((pad[0] + pad[2]) / 2, (pad[1] + pad[3]) / 2, 0.0045), (0.055, 0.26, 0.28), roughness=0.9)
    bt.parts.marking(scene, "floor/handover_edge", rect=pad, width=0.025,
                     color=TEAL, floor=0.005, thickness=0.001)
    scene.set_obstacle_color("stand/top", DECK)
    scene.set_obstacle_material("stand/top", metalness=0.45, roughness=0.4)
    # Two thin landing marks directly under the taught set-down targets.
    for tag, side in (("low", -1.0 if machine.arms["low"] == "right" else 1.0),
                      ("top", -1.0 if machine.arms["top"] == "right" else 1.0)):
        px, py, _ = in_world("dock", (machine.dock_standoff + machine.stand_inset,
                                      side * machine.span, 0.0), machine.holonomic)
        w, d, _ = machine.carton
        if not machine.holonomic:
            w, d = d, w
        bt.parts.marking(scene, f"stand/landing_{tag}",
                         rect=(px - w / 2 - 0.012, py - d / 2 - 0.012,
                               px + w / 2 + 0.012, py + d / 2 + 0.012),
                         width=0.008, color=TEAL if tag == "low" else YELLOW,
                         floor=machine.stand_top + 0.0002, thickness=0.0006)
    for level in range(3):
        scene.set_obstacle_color(f"row/shelves/l{level}", DECK)
        scene.set_obstacle_material(f"row/shelves/l{level}", metalness=0.35, roughness=0.48)
        for edge in ("f", "b"):
            scene.set_obstacle_color(f"row/trim/beam{level}{edge}", BEAM)
        # Background stock is deliberately modest; the two picked cartons
        # stay the only moving workpieces and the only stock on the pick bay.
        z = machine.boards[-1] * (level + 1) / 3
        for index in range(4):
            name = f"row/stock/{level}_{index}"
            bt.parts.carton(scene, name, (0.30 + 0.025 * (index % 2), 0.25, 0.16 + 0.035 * ((index + level) % 3)),
                            (CORNER_X - 1.65 + index * 0.62, row_y, z + 0.001), detail="full")
            scene.set_obstacle_enabled(name, False)
            scene.remove_part(name)


def build_scene(machine: Machine, aisle: float = 1.4) -> bt.Scene:
    """The aisle, the bay with a carton on its low and its top board, the
    hand-off stand, and the machine parked at the dock."""
    robot, wheels = load(machine)
    scene = bt.Scene(robot, name=ROBOT)
    if (wheels.vehicle_drive == "holonomic") != machine.holonomic:
        raise ValueError(f"{machine.key}: the wheels drive `{wheels.vehicle_drive}` but the machine description "
                         f"says holonomic={machine.holonomic} — the cell is laid out by that")
    where = stations(machine.holonomic)
    (bay_xy, _), (dock_xy, _) = where["bay"], where["dock"]
    # A base that turns backs out of the dock and pivots once, at the bay; one
    # that does not crabs down the aisle, nose to the racks the whole way.
    route = ({"path": [bay_xy, (CORNER_X, 0.0), (0.0, 0.0), dock_xy], "stations": {"bay": 0, "dock": 3}}
             if machine.holonomic else
             {"path": [bay_xy, (CORNER_X, 0.0), dock_xy], "stations": {"bay": 0, "dock": 2},
              "turn_speed": machine.turn, "allow_reverse": True})
    scene.add_vehicle(BASE, body=[], start="dock", speed=machine.speed, drive=wheels.vehicle_drive, **route)
    scene.mount_robot(BASE, robot=ROBOT, wheels=wheels)

    front = SPUR + machine.standoff                      # the bay's front edge
    add_bay(scene, "bay", machine.boards, (CORNER_X, front + BAY[1] / 2))
    # Across the aisle, the row the machine must not brush while it pivots.
    bt.parts.rack(scene, "row", size=(3.0, BAY[1], machine.boards[-1]),
                  position=(CORNER_X - 0.6, front - aisle - BAY[1] / 2), levels=3,
                  model="SR-3000", manufacturer="Generic", color=STEEL, detail="full")
    w, d, h = machine.carton
    for tag, board in zip(("low", "top"), machine.boards):
        # Facing the bay (+y) the machine's right hand is at +x.
        side = 1.0 if machine.arms[tag] == "right" else -1.0
        scene.add_box(f"carton_{tag}", (w, d, h),
                      (CORNER_X + side * machine.span, front + machine.inset, board + h / 2 + 0.002),
                      color=CARTON)
        scene.set_part(f"carton_{tag}", category="workpiece", model="carton 100", mass_kg=0.35)
        # One visual asset follows the held obstacle through the whole cycle.
        bt.parts.appearance(scene, f"carton_{tag}", "carton", (w, d, h))
    if machine.holonomic:
        bt.parts.table(scene, "stand", size=(STAND[1], STAND[0], machine.stand_top),
                       position=(dock_xy[0], dock_xy[1] + machine.dock_standoff + STAND[0] / 2),
                       model="HFS8-500", manufacturer="Generic", color=STEEL, detail="full")
    else:
        bt.parts.table(scene, "stand", size=(STAND[0], STAND[1], machine.stand_top),
                       position=(-(machine.dock_standoff + STAND[0] / 2), 0.0),
                       model="HFS8-500", manufacturer="Generic", color=STEEL, detail="full")

    dress_cell(scene, machine, aisle)
    # Built into the head: draw the optical frustum without an extra housing.
    link, offset, orientation, fov = machine.camera
    scene.add_camera("head_cam", robot=ROBOT, link=link, position=offset, quaternion=orientation,
                     fov=fov, resolution=RESOLUTION, near=0.1, far=3.0, body_visible=False)
    for tag in ("low", "top"):
        scene.add_vision_sensor(f"sees_{tag}", camera="head_cam", watch=[f"carton_{tag}"])
    return scene


def joints(scene: bt.Scene, base: list, *targets: dict) -> list:
    names = scene.robot_of(ROBOT).joint_names
    q = list(base)
    for target in targets:
        for joint, value in target.items():
            q[names.index(joint)] = value
    return q


class Stand:
    """Stands the machine at a station for teaching and checking: the base
    goes where the vehicle will carry it (the mount's offset and all), so
    what is solved is solved against the real bay. Restores on exit."""

    def __init__(self, scene: bt.Scene, holonomic: bool = False):
        self.scene = scene
        self.stations = stations(holonomic)
        self.base = scene.robot_base_pose_of(ROBOT)
        self.q = list(scene.joint_positions_of(ROBOT))
        (dock, heading) = self.stations["dock"]
        # The mount's offset, read back in the machine's own frame.
        world = (self.base[0][0] - dock[0], self.base[0][1] - dock[1], self.base[0][2])
        self.offset = rotate(yaw_quat(-heading), world)

    def at(self, station: str) -> float:
        (x, y), heading = self.stations[station]
        dx, dy, dz = rotate(yaw_quat(heading), self.offset)
        self.scene.set_robot_base_pose((x + dx, y + dy, dz), yaw_quat(heading), robot=ROBOT)
        return heading

    def __enter__(self):
        return self

    def __exit__(self, *exc) -> None:
        self.scene.set_robot_base_pose(*self.base, robot=ROBOT)
        self.scene.set_joint_positions(self.q, robot=ROBOT)


def in_world(station: str, point: tuple, holonomic: bool = False) -> tuple:
    """A point given in the machine's own frame (x ahead, y left, z over
    the floor), where the machine stands at `station`."""
    (x, y), heading = stations(holonomic)[station]
    dx, dy, dz = rotate(yaw_quat(heading), point)
    return (x + dx, y + dy, dz)


def out_of_view(scene: bt.Scene, machine: Machine, point: tuple) -> float:
    """How many degrees outside the head camera's frustum `point` is, as the
    machine stands (<= 0: framed). The geometric half of what the vision
    sensor judges — its occlusion ray is left to the bake, which stalls at
    the glance by name. A camera looks along its -Z, +Y up; `fov` is the
    horizontal angle and the aspect follows the resolution."""
    link, offset, orientation, fov = machine.camera
    at, turned = scene.link_pose(link, robot=ROBOT)
    eye = tuple(a + o for a, o in zip(at, rotate(turned, offset)))
    attitude = quat_mul(turned, orientation)
    x, y, z = rotate((-attitude[0], -attitude[1], -attitude[2], attitude[3]),
                     tuple(p - e for p, e in zip(point, eye)))
    if z >= 0.0:
        return 180.0
    half = math.radians(fov) / 2
    half_up = math.atan(math.tan(half) * RESOLUTION[1] / RESOLUTION[0])
    return math.degrees(max(abs(math.atan2(x, -z)) - half, abs(math.atan2(y, -z)) - half_up))


def teach(scene: bt.Scene, machine: Machine) -> dict:
    """Every taught configuration, by name — and, per task, the torso
    posture chosen for it (`torso_low`, ...) and how the hand returns to the
    carry (`*_carry_kind`). Solved with the machine standing where it will
    work and its torso where the ramp will have put it: an arm-only motion
    holds the torso wherever it is, so the two must agree."""
    robot = scene.robot_of(ROBOT)
    _, _, h = machine.carton
    poses: dict = {}

    def held_at(arm: str) -> str:
        return machine.grasp[arm] or robot.group(arm).tip

    def solve(arm: str, torso: dict, position: tuple, heading: float, seed: list) -> list:
        """The arm at `position`, hand level and square to the approach,
        free of collision — or a `LookupError` that says what stopped it."""
        orientation = quat_mul(yaw_quat(heading), machine.approach)
        short, touching = None, set()
        for start in [seed] + [joints(scene, seed, s) for s in machine.seeds[arm]]:
            scene.set_joint_positions(joints(scene, start, torso, machine.hands_open[arm]), robot=ROBOT)
            result = scene.set_tcp_target(position, orientation, link=held_at(arm), robot=ROBOT, group=arm)
            if not result.converged:
                short = result.pos_error
                continue
            hits = scene.check_collisions()
            if not hits:
                return list(scene.joint_positions_of(ROBOT))
            touching.update(f"{a[1]} × {b[1]}" for a, b in hits)
        where = ", ".join(f"{v:.2f}" for v in position)
        err = LookupError(f"the {arm} hand cannot take ({where})"
                          + (f", {short * 1e3:.0f} mm short" if short is not None else "")
                          + (f"; it touches {', '.join(sorted(touching)[:3])}" if touching else ""))
        err.short = None if touching else short          # it got there, touching: not a matter of reach
        raise err

    def level_hand(attitude: tuple, heading: float) -> bool:
        """Whether a hand in this attitude is level and square to the approach."""
        level = quat_mul(yaw_quat(heading), machine.approach)
        return abs(sum(a * b for a, b in zip(attitude, level))) > math.cos(0.01)

    def line_exists(arm: str, start: list, goal: list) -> bool:
        """Whether the hand gets from `start` to `goal` in a straight line.
        Two taught poses of a 7-axis arm may be different postures of the
        same hand, and then no line joins them — it dies part way with
        `configuration jump (IK branch change)`. The planner is asked, on a
        motion that exists only for the question."""
        scene.set_joint_positions(start, robot=ROBOT)
        scene.add_segment(PROBE, goal=goal, kind=LINE, group=arm, robot=ROBOT)
        try:
            scene.plan_motion(PROBE, broadcast=False)
            return True
        except (ValueError, RuntimeError):
            return False
        finally:
            scene.remove_motion(PROBE)

    def back_to_carry(arm: str, torso: dict, heading: float, start: list) -> tuple:
        """The carry under this torso, and how a hand gets back into it from
        `start`. A torso that only lifts leaves the carried hand level: the
        pose is solved like the others and the way in is a straight line,
        the hand level all the way (out from under a board, not through it)
        — when there is one; a planned move when there is not. A torso that
        bows takes the hand's attitude with it: the way in is planned."""
        q = joints(scene, poses["travel"], torso)
        position, attitude = scene.link_pose_at(held_at(arm), q, robot=ROBOT)
        if level_hand(attitude, heading):
            carry = solve(arm, torso, position, heading, q)
            return carry, LINE if line_exists(arm, start, carry) else "joint"
        return q, "joint"

    def task(kind: str, heading: float, targets: dict, leaves: dict) -> None:
        """One torso posture for all of `targets` (`name -> (arm, point)`):
        the gentlest of the machine's candidates that reaches them.
        `leaves` names, per arm, the target its hand leaves for the carry from."""
        refused = []
        for torso in machine.torso[kind]:
            solved, seed = {}, poses["travel"]
            try:
                for name, (arm, point) in targets.items():
                    solved[name] = seed = solve(arm, torso, point, heading, seed)
            except LookupError as err:
                refused.append((err.short, f"{torso}: {err}"))
                continue
            poses.update(solved)
            poses[f"torso_{kind}"] = torso
            for arm, last in leaves.items():
                poses[f"{kind}_{arm}_carry"], poses[f"{kind}_{arm}_carry_kind"] = back_to_carry(
                    arm, torso, heading, solved[last])
            return
        # The first line is the verdict a table wants: how far off the best posture is.
        shorts = [short for short, _ in refused]
        verdict = ("the hand gets there only touching something" if None in shorts
                   else f"the closest is {min(shorts) * 1e3:.0f} mm short")
        raise RuntimeError(f"{machine.key}: no torso posture reaches `{kind}` — {verdict}:\n  "
                           + "\n  ".join(text for _, text in refused))

    with Stand(scene, machine.holonomic) as stand:
        # The carry first — the cycle starts and travels in it: each hand
        # level ahead of the chest, solved like every other pose.
        heading = stand.at("dock")
        poses["travel"] = joints(scene, list(scene.joint_positions_of(ROBOT)), machine.travel)
        for arm in SIDES:
            try:
                poses["travel"] = solve(arm, machine.travel, in_world("dock", machine.carry[arm], machine.holonomic), heading,
                                        poses["travel"])
            except LookupError as err:
                raise RuntimeError(f"{machine.key}: the carry: {err}") from err
        for arm in SIDES:
            poses["travel"] = joints(scene, poses["travel"], {j: 0.0 for j in machine.hands_shut[arm]})

        # The low board is open above: the hand comes down onto its
        # carton and goes back up. The top one is under nothing the hand
        # can use — it goes straight in over the board and straight out.
        heading = stand.at("bay")
        grip, _ = scene.obstacle_pose("carton_low")
        # The grip first, its neighbours seeded from it: a 7-axis arm has a
        # posture for every target, and a straight line between two taught
        # poses only exists if they are the same posture, moved.
        back = rotate(yaw_quat(heading), (-machine.pull, 0.0, 0.0))
        arm = machine.arms["low"]
        task("low", heading, {
            "low_grip": (arm, grip),
            "low_over": (arm, (grip[0], grip[1], grip[2] + DROP)),
        }, leaves={arm: "low_over"})
        # A torso that squats carries its hand lower than it picks: straight
        # from over the carton into a carry below the board cuts the board's
        # front edge. A level hand backs out over the edge first, as the top
        # one does — and how it goes on from there is asked again.
        carry = poses[f"low_{arm}_carry"]
        carried, attitude = scene.link_pose_at(held_at(arm), carry, robot=ROBOT)
        if level_hand(attitude, heading) and carried[2] - h / 2 < machine.boards[0] + 0.01:
            try:
                poses["low_out"] = solve(arm, poses["torso_low"], (grip[0] + back[0], grip[1] + back[1], grip[2] + DROP),
                                         heading, poses["low_over"])
            except LookupError:
                # An arm whose elbow folds no further cannot bring the hand
                # that near its shoulder: the way back is planned from over
                # the carton, and the planner keeps the carton off the edge.
                poses[f"low_{arm}_carry_kind"] = "joint"
            else:
                poses[f"low_{arm}_carry_kind"] = LINE if line_exists(arm, poses["low_out"], carry) else "joint"
        grip, _ = scene.obstacle_pose("carton_top")
        out = tuple(g + b for g, b in zip(grip, back))
        task("top", heading, {
            "top_grip": (machine.arms["top"], grip),
            "top_up": (machine.arms["top"], (grip[0], grip[1], grip[2] + 0.03)),
            "top_out": (machine.arms["top"], (out[0], out[1], out[2] + 0.03)),
            "top_over": (machine.arms["top"], out),
        }, leaves={machine.arms["top"]: "top_out"})

        # The glances, gentlest first: one that frames the carton and gets
        # there without sweeping the cell — the low one from where the torso
        # travels, the top one from under the torso that reaches its board.
        for kind, under in (("low", poses["travel"]), ("top", joints(scene, poses["travel"], poses["torso_top"]))):
            (cx, cy, cz), refused = scene.obstacle_pose(f"carton_{kind}")[0], []
            # The sensor asks whether the carton *overlaps* the frustum: its corners, not its centre.
            corners = [(cx + sx * machine.carton[0] / 2, cy + sy * machine.carton[1] / 2, cz + sz * h / 2)
                       for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
            candidates = machine.look[kind] if isinstance(machine.look[kind], list) else [machine.look[kind]]
            for glance in candidates:
                scene.set_joint_positions(joints(scene, under, glance), robot=ROBOT)
                off = min(out_of_view(scene, machine, corner) for corner in corners)
                hits = (ramp_contacts(scene, "bay", under, joints(scene, under, glance), holonomic=machine.holonomic)
                        if off <= 0.0 else [])
                if off <= 0.0 and not hits:
                    poses[f"look_{kind}"] = glance
                    break
                refused.append(f"{glance}: " + (f"the carton is {off:.0f} deg out of view" if off > 0.0
                                                else f"the glance sweeps through the cell: {hits[:2]}"))
            else:
                raise RuntimeError(f"{machine.key}: no glance shows the {kind} carton:\n  " + "\n  ".join(refused))

        # One torso posture for both hands at the stand: they set down together.
        heading = stand.at("dock")
        ahead = machine.dock_standoff + machine.stand_inset
        seats = {}
        for tag in ("low", "top"):
            arm = machine.arms[tag]
            side = 1.0 if arm == "left" else -1.0
            seat = in_world("dock", (ahead, side * machine.span, machine.stand_top + h / 2 + 0.002), machine.holonomic)
            seats[f"{tag}_seat"] = (arm, seat)
            seats[f"{tag}_above"] = (arm, (seat[0], seat[1], seat[2] + 0.06))
        task("stand", heading, seats, leaves={machine.arms[tag]: f"{tag}_above" for tag in ("low", "top")})
    scene.set_joint_positions(poses["travel"], robot=ROBOT)
    return poses


def ramp_contacts(scene: bt.Scene, station: str, a: list, b: list, samples: int = 24,
                  holonomic: bool = False) -> list:
    """Link/obstacle pairs a ramp from `a` to `b` would touch with the
    machine parked at `station`, sampled along its cubic: a ramp is not
    planned, so its sweep is the author's to check before the bake."""
    hits = set()
    with Stand(scene, holonomic) as stand:
        stand.at(station)
        for k in range(samples + 1):
            u = k / samples
            ease = u * u * (3 - 2 * u)
            scene.set_joint_positions([x + (y - x) * ease for x, y in zip(a, b)], robot=ROBOT)
            hits.update((p[1], q[1]) for p, q in scene.check_collisions())
    return sorted(hits)


def build_cycle(scene: bt.Scene, machine: Machine, poses: dict) -> str:
    """Writes the arms' motions and the cycle; returns the sequence's name."""
    travel = poses["travel"]
    look = {**machine.look, "low": poses["look_low"], "top": poses["look_top"]}
    torso = {kind: poses[f"torso_{kind}"] for kind in ("low", "top", "stand")}
    torso_s, look_s, hand_s = machine.paces

    # The torso's sweeps, arms in the carry: down and up at the bay, to the
    # stand's height at the dock, and the glances in between.
    low, top = joints(scene, travel, torso["low"]), joints(scene, travel, torso["top"])
    for label, station, a, b in (
        ("look low", "bay", travel, joints(scene, travel, look["low"])),
        ("torso down", "bay", joints(scene, travel, look["low"]), low),
        ("straighten", "bay", low, travel),
        ("torso up", "bay", travel, top),
        ("look top", "bay", top, joints(scene, top, look["top"])),
        ("torso stand", "dock", travel, joints(scene, travel, torso["stand"])),
    ):
        hits = ramp_contacts(scene, station, a, b, holonomic=machine.holonomic)
        if hits:
            raise RuntimeError(f"{machine.key}: the `{label}` ramp sweeps through the cell: {hits[:3]}")

    for tag in ("low", "top"):
        arm = machine.arms[tag]

        def seg(motion: str, pose: str, kind: str = "joint", arm: str = arm) -> None:
            scene.add_segment(motion, goal=poses[pose], kind=kind, group=arm, robot=ROBOT)

        seg(f"pick_{tag}", f"{tag}_over")              # planned: from the carry to the bay's mouth
        seg(f"pick_{tag}", f"{tag}_grip", LINE)        # straight onto the carton
        if tag == "low":
            seg("lift_low", "low_over", LINE)          # straight up,
            if "low_out" in poses:
                seg("lift_low", "low_out", LINE)       # back over the edge, when the carry is below the board,
        else:
            seg("lift_top", "top_up", LINE)            # off the board,
            seg("lift_top", "top_out", LINE)           # straight out from under nothing, over the edge,
        seg(f"lift_{tag}", f"{tag}_{arm}_carry", poses[f"{tag}_{arm}_carry_kind"])    # and into the carry
        seg(f"place_{tag}", f"{tag}_above")
        seg(f"place_{tag}", f"{tag}_seat", LINE)       # straight down onto the stand
        seg(f"clear_{tag}", f"{tag}_above", LINE)
        seg(f"clear_{tag}", f"stand_{arm}_carry", poses[f"stand_{arm}_carry_kind"])

    sq = scene.sequence("pick")

    def ramp(duration: float, *targets: dict) -> list:
        """One ramp over the joints of `targets` (later ones win) — none at
        all when there is nothing to move (a fixed head has no glance)."""
        merged = {joint: value for target in targets for joint, value in target.items()}
        return [S.ramp(merged, duration, robot=ROBOT)] if merged else []

    def hand(arm: str, shut: bool):
        """The fingers' ramp: closed on the carton, or back to open."""
        closed = machine.hands_shut[arm]
        opened = {**{joint: 0.0 for joint in closed}, **machine.hands_open[arm]}
        return ramp(hand_s, closed if shut else opened)

    def step(name: str, actions: list, *also) -> None:
        """A step that waits for what it started (nothing, when it starts
        nothing — a fixed head has no glance) and for `also`."""
        moving = [S.done()] if any(a["type"] in ("start_ramp", "start_motion") for a in actions) else []
        waits = [*moving, *also] or [S.immediately()]
        sq.step(name, actions=actions, transition=waits[0] if len(waits) == 1 else S.all_of(*waits))

    def grasp(tag: str) -> list:
        arm = machine.arms[tag]
        return [S.attach(f"carton_{tag}", robot=ROBOT, group=arm, touch_links=machine.touch[arm]),
                *hand(arm, True)]

    low_arm, top_arm = machine.arms["low"], machine.arms["top"]
    sq.step("to bay", actions=[S.goto(BASE, "bay")], transition=S.device_done(BASE))
    # The camera is an input: no carton in view, no pick. The low one is
    # looked down on from where the torso travels; the top one stands on a
    # board over the machine's eyes until the torso is up.
    step("look low", ramp(look_s, look["low"]), S.signal("sees_low"))
    step("torso down", [*ramp(torso_s, look["ahead"], machine.travel, torso["low"]), *hand(low_arm, False)])
    step("pick low", [S.motion("pick_low")])
    step("grasp low", grasp("low"))
    step("lift low", [S.motion("lift_low")])
    # Up is two moves, not one: a torso that bowed into the bay straightens
    # first — raised as it stands, its head would come up under the top board.
    step("straighten", ramp(torso_s, machine.travel))
    step("torso up", [*ramp(torso_s, torso["top"]), *hand(top_arm, False)])
    step("look top", ramp(look_s, look["top"]), S.signal("sees_top"))
    step("pick top", [S.motion("pick_top")])
    step("grasp top", grasp("top"))
    step("lift top", [S.motion("lift_top")])
    # The fold rides the drive: the torso comes down while the wheels roll.
    step("to dock", [S.goto(BASE, "dock"), *ramp(torso_s, look["ahead"], machine.travel)], S.device_done(BASE))
    step("torso stand", ramp(torso_s, torso["stand"]))
    step("set down", [S.motion("place_low"), S.motion("place_top")])
    step("release", [S.detach("carton_low"), S.detach("carton_top"), *hand(low_arm, False), *hand(top_arm, False)])
    step("clear", [S.motion("clear_low"), S.motion("clear_top")])
    step("fold", ramp(torso_s, machine.travel))
    return "pick"


def bake(robot: str = "g1d", aisle: float = 1.4, boards: tuple | None = None):
    """The cell, taught and baked: `(scene, machine, poses, timeline)`.
    `boards` puts the machine in front of another bay than its own."""
    machine = MACHINES[robot] if boards is None else replace(MACHINES[robot], boards=tuple(boards))
    scene = build_scene(machine, aisle)
    poses = teach(scene, machine)
    name = build_cycle(scene, machine, poses)
    return scene, machine, poses, scene.simulate_sequence(name, max_duration=180.0)


def compare(boards: tuple, aisle: float = 1.4, machines=None) -> list:
    """One bay, every machine: `(key, low torso, top torso, cycle, verdict)`
    per machine, printed as the table a buyer wants.

    The bay's two heights and the aisle are the cell's; the teaching is one
    procedure. What differs is the machine — how its torso makes height
    (a column, a bow, a squat), how long its arms are, where it has to stand
    — and each keeps what is its hand's: the carton width it grips, how far
    from the bay it stops. A verdict comes from one of three places: the
    *package* (not to be had), the *teaching* (no torso posture of the ones
    the machine description lists puts the hand on the board — with how far
    off the best one is), the *bake* (a body that brushes the aisle, a sweep
    through the bay). None is an opinion about the machine: they are this
    bay's requirements, met or not."""

    def posture(machine: Machine, torso: dict) -> str:
        moved = ", ".join(f"{joint} {value:g}" for joint, value in torso.items()
                          if value != machine.travel.get(joint, 0.0))
        return moved or "as it travels"

    def first_line(err: Exception) -> str:
        """The one line of a failure worth putting in a table."""
        text = " ".join(str(err).split("\n")[0].split()).rstrip(":")
        return text if len(text) < 110 else text[:107] + "..."

    print(f"bay: boards at {boards[0]:.2f} m and {boards[1]:.2f} m; aisle {aisle:.2f} m")
    print(f"{'machine':<8} {'torso, low board':<52} {'torso, top board':<52} {'cycle':>7}   verdict")
    rows = []
    for key in machines or MACHINES:
        low = top = "—"
        cycle = None
        machine = replace(MACHINES[key], boards=tuple(boards))
        try:
            try:
                scene = build_scene(machine, aisle)
            except (ValueError, RuntimeError, OSError) as err:
                raise RuntimeError(f"{key}: the package is not to be had: {first_line(err)}") from err
            poses = teach(scene, machine)
            low, top = posture(machine, poses["torso_low"]), posture(machine, poses["torso_top"])
            cycle = scene.simulate_sequence(build_cycle(scene, machine, poses), max_duration=180.0).duration
            verdict = "ok"
        except (ValueError, RuntimeError) as err:
            verdict = first_line(err).removeprefix(f"{key}: ")
        rows.append((key, low, top, cycle, verdict))
        print(f"{key:<8} {low:<52} {top:<52} {'—' if cycle is None else f'{cycle:6.2f}s':>7}   {verdict}")
    print("\nOne bay, one aisle, one way of teaching it. What changed is the machine:")
    print("how its torso makes height, how far its arms go, where it stands to work.")
    return rows


def main() -> None:
    args = sys.argv[1:]

    def option(flag: str, default=None):
        return args[args.index(flag) + 1] if flag in args else default

    robot, aisle, deliverables = option("--robot", "g1d"), float(option("--aisle", 1.4)), option("--deliverables")
    values = {option(flag) for flag in ("--robot", "--aisle", "--deliverables")}
    out = next((a for a in args if not a.startswith("--") and a not in values), "cell_semi_humanoid.usdc")
    if robot not in MACHINES:
        sys.exit(f"unknown --robot `{robot}` ({' | '.join(MACHINES)})")
    machine = MACHINES[robot]
    if "--compare" in args:
        # Every machine in front of `--robot`'s bay.
        compare(machine.boards, aisle)
        return
    print(f"{machine.title}; aisle {aisle:.2f} m")

    try:
        scene = build_scene(machine, aisle)
    except (ValueError, RuntimeError, OSError) as err:
        sys.exit(f"the machine is not to be had: {err}")
    try:
        poses = teach(scene, machine)
        name = build_cycle(scene, machine, poses)
    except RuntimeError as err:
        sys.exit(f"teaching failed: {err}")
    if "--studio" in args:
        bt.studio(scene, view=((5.9, 4.7, 4.0), (1.25, 0.0, 0.65)))
        return
    try:
        tl = scene.simulate_sequence(name, max_duration=180.0)
    except ValueError as err:
        sys.exit(f"cycle failed: {err}")

    print(f"cycle time: {tl.duration:.2f}s")
    for step, start, end in tl.step_spans:
        print(f"  {step:<12} {start:6.2f} – {end:6.2f}s")
    # The wheels are joints like any other: read off the same track. (Out
    # and back cancel — the way home unwinds them — so ask at the bay.)
    names = scene.robot_of(ROBOT).joint_names
    arrived = tl.step_span("to bay").end
    turned = {j: tl.sample(arrived, robot=ROBOT)[names.index(j)] for j in names if "wheel" in j.lower()}
    print(f"at the bay ({arrived:.2f}s) the wheels have turned: "
          + ", ".join(f"{j} {a:+.1f} rad" for j, a in turned.items()))
    print(f"torso: low {poses['torso_low']}, top {poses['torso_top']}, stand {poses['torso_stand'] or 'as it travels'}")
    for tag in ("low", "top"):
        p = tl.object_pose(f"carton_{tag}", tl.duration)[0]
        print(f"carton_{tag} ends at ({p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}) — on the stand")
    print()
    print(scene.bom().to_markdown())
    print(bt.select.requirements(scene, timeline=tl).to_markdown())

    tl.export_usd(out, fps=60)
    print(f"wrote {out}")
    if deliverables:
        manifest = bt.export_cell(scene, deliverables, name="semi_humanoid", max_duration=180.0)
        print(f"wrote the hand-over set: {manifest}")


if __name__ == "__main__":
    main()
