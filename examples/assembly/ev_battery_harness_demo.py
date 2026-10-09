"""EV battery pack: two robots plug the HV module connectors.

The station is the one Liebherr-Verzahntechnik, KOSTAL and KUKA showed for
pluggable battery-module connectors, set into a pack line: two robots in
one cell, each holding one end of a flexible module connector, take it
from its blister, find the slot with the 2D vision sensor in the gripper,
push both ends home at once and lock the CPA. What the cell is built from:

  * the pack on its workpiece carrier on a roller line (`_ev_battery_pack`):
    six modules either side of a central channel, the KOSTAL KS 22 tab
    headers at their inner ends, the coolant manifold on the channel's
    floor and the BMS daisy-chain harness in its carrier above it, the BDU
    in its bay; the five module connectors (35 mm², a KS 22 receptacle at
    each end) in a black ESD blister on the same carrier, the next carrier
    queued upstream;
  * two FANUC CRX-10iA on pedestals either side of the line (the catalog's;
    the KUKA KR CYBERTECH nano of the original is not in it), each with a
    SCHUNK MPG-plus 25 on a float unit, fingers for the 15 mm housing and a
    2D smart camera (SensoPart VISOR Robotic class) on the gripper's outer
    side; a 30° bracket leans each wrist outboard of its plumb gripper, or
    two wrists would meet over a 300 mm cable;
  * an aluminium-frame guard with clear panels, the line passing through,
    a light curtain at each opening, the R-30iB Mini Plus controllers and
    the cabinet behind, the operator's station in front.

The cycle, per connector: both arms over the blister → each camera finds
its housing → down, grip → out of the blister and lifted, the ends brought
4 cm together so the cable hangs in a loop rather than being pulled
straight → carried over the pack in one plane above the module tops, both
arms on one clock (one step, one duration — KUKA.RoboTeam's job), the long
moves through via poses on their straight lines → each camera finds its
header → approach → pushed home slowly → CPA locked → let go → clear. Five
connectors in about 67 s, L1 to R3 in a zig-zag across the channel that
never crosses itself.

The cable is `bt.rope`: a cable simulated after the bake against the
finished cycle. Its two ends are crimped inside the receptacle housings —
45 mm held by each housing (`bt.rope.anchor`) — and the housings are
ordinary parts the robots attach, carry and set down on their headers, so
the cable goes where they go and stays where they are plugged. The robots
never feel it (one-way); what it shows is what a rigid simulation cannot:
whether the two arms pull it taut (how much longer it gets than it lay in
the blister), whether it brushes the modules on the way over, how far it
droops into the channel once plugged and how close it comes to the LV
harness and the coolant manifold there. `--slack 0.06` fits connectors
60 mm too long, and they lie on the LV harness's carrier; `--slack -0.07`
fits them 70 mm too short, and the arms pull them 6 % longer pushing them
home: the run refuses both by name. The bend radius is printed but not
judged — the rope's bending stiffness is the solver's native setting, not
the cable's EI. About 40 s of cable simulation for the five.

The run prints the cycle, the closest approach to the guard, the pack and
the blister, and a line per connector; the recording carries the cables
as tubes (`/World/Ropes`).

Run with:  python examples/assembly/ev_battery_harness_demo.py [out.usdc] [--studio]
               [--slack 0.0] [--catalog-root DIR]
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

import botrail as bt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import _ev_battery_pack as P

ARM = "fanuc/crx/crx10ia/r1"
GRIPPER = "schunk/mpg-plus/mpg-plus-25/r1"
PEDESTAL = "sus/zf/robostand-crx/r1"
CONTROLLER = "fanuc/r-30ib-plus/cabinet/r1"
CURTAIN = "keyence/gl-r/series/r1"
FRAME = "misumi/hfs/6-series/r3"
CABINET = "rittal/vx25/base/r1"

ROBOTS = ("L", "R")                   # L stands at +y, R at -y
BASE_X, BASE_Y, BASE_Z = 0.15, 0.92, 0.70
SIDE = {"L": 1.0, "R": -1.0}
# Tool down, the fingers closing along the line (x), the camera outboard.
DOWN = {"L": (0.0, 1.0, 0.0, 0.0), "R": (1.0, 0.0, 0.0, 0.0)}
OPEN, SHUT = 0.003, 0.0               # MPG-plus 25 finger joint: 3 mm a jaw, closed on the housing

# ---------------------------------------------------------------- the hand
ADAPTER = 0.012                       # ISO 9409 plate on the CRX flange
TILT = math.radians(-30.0)            # the bracket leans the wrist outboard of the plumb gripper
BRACKET = 0.045                       # the bracket block, along the gripper's axis
FLOAT = 0.035                         # float unit: takes up the last tenths at the header
JAW_TIP = 0.027                       # MPG-plus 25: jaw tips above its mount
GRASP = JAW_TIP + 0.030               # the housing's centre between the fingers
FINGER = (0.004, 0.020, 0.034)        # each finger: x thickness, y depth, length below the jaw tips
CAMERA = (0.040, 0.034, 0.070)        # 2D smart camera body
CAMERA_Y = 0.050                      # its axis off the fingers', outboard
CAMERA_TILT = math.radians(20.0)      # leaning in to look at the fingers' work
ALUMINIUM = (0.80, 0.81, 0.83)
DARK = (0.03, 0.032, 0.035)

# ---------------------------------------------------------------- the moves
JOINT_SPEED = [math.radians(v) for v in (120, 120, 180, 180, 180, 180)]   # CRX-10iA axis speeds
FREE = 0.8                            # m/s, TCP, nothing in the hand
CARRY = 0.30                          # m/s, TCP, a cable between the hands
SEAT = 0.040                          # m/s, the last 15 mm onto the header
TRANSFER_Z = 0.22                     # the plane every carry and return runs in, above the module tops:
                                      # a cable hanging in its loop clears the housings already plugged
NEAR = 0.04                           # where the last, slow move onto a housing starts
APPROACH = 0.015                      # where the push home starts
GATHER = 0.040                        # how far each end comes in as it is lifted
VIAS = 2                              # via poses on the straight line of a long move
LOOK = 0.30                           # s: a vision sensor's job
CPA = 0.40                            # s: the locking pin presses the CPA down

# ---------------------------------------------------------------- the checks
STRETCH_MAX = 0.010                   # pulled taut: 1 % longer than it lay in the blister
LV_CLEAR_MIN = 0.010                  # HV clear of the LV harness, m
BEND_MIN = 4 * P.CABLE_OD             # static bend radius rule of thumb, 4 x OD — indicative only
BOOT = 0.015                          # beyond a housing's exit: the sim holds the housing's span rigid
                                      # and the next sample free, so the corner at the exit is the
                                      # model's, not the cable's


def loader(catalog_root: Path | None):
    def load(pid: str) -> bt.Robot:
        if catalog_root is not None:
            return bt.Robot.from_package(catalog_root / pid, catalog_root=catalog_root)
        return bt.Robot.from_catalog(pid)
    return load


def _material(color) -> str:
    return "c_" + "_".join(f"{round(v * 1000):04d}" for v in color)


def _box_xml(size, xyz=(0, 0, 0), color=DARK, collide=True, rpy=(0, 0, 0)) -> str:
    s = " ".join(f"{v:.5f}" for v in size)
    o = (f'<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" '
         f'rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>')
    c = " ".join(f"{v:.4f}" for v in color)
    out = (f'<visual>{o}<geometry><box size="{s}"/></geometry>'
           f'<material name="{_material(color)}"><color rgba="{c} 1"/></material></visual>')
    if collide:
        out += f'<collision>{o}<geometry><box size="{s}"/></geometry></collision>'
    return out


def _cyl_xml(radius, length, xyz=(0, 0, 0), color=DARK, collide=True) -> str:
    o = f'<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}"/>'
    c = " ".join(f"{v:.4f}" for v in color)
    g = f'<geometry><cylinder radius="{radius:.5f}" length="{length:.5f}"/></geometry>'
    out = f'<visual>{o}{g}<material name="{_material(color)}"><color rgba="{c} 1"/></material></visual>'
    if collide:
        out += f"<collision>{o}{g}</collision>"
    return out


def _part(name: str, visual: str, extra: str = "") -> bt.Robot:
    return bt.Robot.from_urdf_string(f'<robot name="{name}"><link name="{name}">{visual}</link>{extra}</robot>')


def hand(load) -> bt.Robot:
    """CRX-10iA → angled bracket → float unit (the camera on its outboard
    side) → MPG-plus 25 → fingers for the KS 22 housing. The bracket leans
    the wrist `TILT` outboard while the gripper hangs plumb: two CRX wrists
    side by side need more room than the two ends of a 300 mm cable leave
    them. TCP `grasp_tcp` at the housing's centre between the fingers."""
    arm = load(ARM)
    # The bracket: the ISO plate on the flange, a block cut at TILT, its
    # face square to the gripper's axis.
    axis = (0.0, -math.sin(TILT), math.cos(TILT))
    reach = BRACKET
    mid = [ADAPTER * (k == 2) + axis[k] * reach / 2 for k in range(3)]
    end = [ADAPTER * (k == 2) + axis[k] * reach for k in range(3)]
    visual = _cyl_xml(0.0315, ADAPTER, (0, 0, ADAPTER / 2), ALUMINIUM)
    visual += _box_xml((0.056, 0.056, reach), tuple(mid), ALUMINIUM, rpy=(TILT, 0, 0))
    bracket = _part("bracket", visual,
                    f'<link name="bracket_tcp"/><joint name="bracket_tcp_joint" type="fixed"><parent link="bracket"/>'
                    f'<child link="bracket_tcp"/><origin xyz="{end[0]:.5f} {end[1]:.5f} {end[2]:.5f}" '
                    f'rpy="{TILT:.6f} 0 0"/></joint>')
    # The float unit, the camera on an L-bracket off its outboard side, its
    # window at the bottom.
    visual = _box_xml((0.050, 0.050, FLOAT), (0, 0, FLOAT / 2), (0.60, 0.61, 0.63))
    visual += _box_xml((0.056, 0.010, 0.012), (0, 0.025, 0.010), (0.12, 0.13, 0.14), collide=False)
    cz = 0.004 + CAMERA[2] / 2
    visual += _box_xml((0.040, CAMERA_Y - 0.025 - CAMERA[1] / 2 + 0.002, 0.006),
                       (0, (0.025 + CAMERA_Y - CAMERA[1] / 2) / 2, 0.006), ALUMINIUM)
    visual += _box_xml(CAMERA, (0, CAMERA_Y, cz), (0.035, 0.036, 0.040))
    visual += _box_xml((CAMERA[0] - 0.006, CAMERA[1] - 0.006, 0.006), (0, CAMERA_Y, cz + CAMERA[2] / 2 + 0.003),
                       (0.55, 0.56, 0.58))
    lens = cz + CAMERA[2] / 2 + 0.006
    head = _part("float", visual,
                 f'<link name="float_tcp"/><joint name="float_tcp_joint" type="fixed"><parent link="float"/>'
                 f'<child link="float_tcp"/><origin xyz="0 0 {FLOAT}"/></joint>'
                 f'<link name="visor"/><joint name="visor_joint" type="fixed"><parent link="float"/>'
                 f'<child link="visor"/><origin xyz="0 {CAMERA_Y} {lens}"/></joint>')
    grip = load(GRIPPER)
    for side, sign in (("left", 1), ("right", -1)):
        finger = _box_xml(FINGER, (sign * (P.RECEPT[0] / 2 + FINGER[0] / 2), 0, JAW_TIP - 0.002 + FINGER[2] / 2),
                          (0.10, 0.10, 0.11))
        grip = grip.mount(_part(f"{side}_finger", finger), at=f"{side}_jaw")
    grip = grip.mount(_part("grasp", "", f'<link name="grasp_tcp"/><joint name="grasp_tcp_joint" type="fixed">'
                                         f'<parent link="grasp"/><child link="grasp_tcp"/>'
                                         f'<origin xyz="0 0 {GRASP}"/></joint>'), at="mount")
    arm = arm.attach_tool(bracket, tcp="bracket_tcp")
    arm = arm.attach_tool(head, flange="bracket_tcp", tcp="float_tcp")
    return arm.attach_tool(grip, flange="float_tcp", tcp="grasp_tcp")


@dataclass
class Cell:
    housings: dict = field(default_factory=dict)     # cable -> {"l": obstacle, "r": obstacle}
    headers: dict = field(default_factory=dict)      # terminal -> header obstacle
    cables: tuple = ()
    finds: dict = field(default_factory=dict)        # (cable, robot, "kit" | "slot") -> vision sensor


def build_cell(load, slack: float = 0.0) -> tuple[bt.Scene, Cell]:
    """The line, the pack, the kit, the two arms and the guard round them."""
    tool = hand(load)
    quarter = math.sin(math.pi / 4)
    scene = bt.Scene(tool, name="L", base_position=(BASE_X, BASE_Y, BASE_Z),
                     base_quaternion=(0.0, 0.0, -quarter, quarter))
    scene.add_robot(tool, name="R", base_position=(BASE_X, -BASE_Y, BASE_Z),
                    base_quaternion=(0.0, 0.0, quarter, quarter))
    cell = Cell()
    cell.cables = tuple(P.Cable(c.name, c.left, c.right, round(c.length + slack, 4), c.slot,
                                f"HV-MC-35-{round((c.length + slack) * 1000)}") for c in P.CABLES)
    floor = scene.add_box("floor", size=(8.0, 7.0, 0.05), position=(0.2, -0.3, -0.025), color=(0.30, 0.31, 0.32))
    scene.set_obstacle_material(floor, metalness=0.0, roughness=0.85)
    P.build_line(scene, length=6.4)
    cell.headers = P.build_pack(scene)
    cell.housings = P.build_kit(scene, cell.cables)
    for robot in ROBOTS:
        s = SIDE[robot]
        stand = bt.parts.pedestal(scene, f"{robot}/pedestal", BASE_Z, (BASE_X, s * BASE_Y), catalog=PEDESTAL)
        for name in stand.obstacles:
            scene.allow_link_obstacle_contact("base_link", name, robot=robot)
        scene.set_joint_positions([0.0, 0.0, 0.0, 0.0, -1.57, 0.0, OPEN], robot=robot)
        bt.parts.controller(scene, f"{robot}_ctrl", [robot], (BASE_X - 0.55 + (0.0 if s > 0 else 0.42), 1.86),
                            catalog=CONTROLLER, variant="mini", yaw=math.pi, cable_m=7.0)
        scene.define_signal(f"{robot}/grip")
        scene.add_camera(f"{robot}/visor", robot=robot, link="visor", position=(0.0, 0.0, 0.0),
                         quaternion=(math.sin((math.pi + CAMERA_TILT) / 2), 0.0, 0.0,
                                     math.cos((math.pi + CAMERA_TILT) / 2)),
                         fov=36.0, resolution=(1280, 960), near=0.02, far=0.6, body_visible=False)
        scene.set_part(f"{robot}/visor", manufacturer="SensoPart", model="VISOR Robotic (2D vision sensor)",
                       category="sensor.camera", description="Narrow field of view, polarising filter; "
                       "hand-eye calibrated, reports X/Y/angle of the slot")
        scene.set_part(f"{robot}/tool", model="angled bracket 30°, ISO 50 plate", category="adapter",
                       description="Customer design: leans the wrist outboard of the plumb gripper")
        scene.set_part(f"{robot}/tool2", model="float unit with vision-sensor bracket", category="adapter",
                       description="Customer design: takes up the last tenths at the header")
    scene.define_signal("cpa_lock")
    for c in cell.cables:
        for robot, side in (("L", "l"), ("R", "r")):
            terminal = c.left if side == "l" else c.right
            housing = cell.housings[c.name][side]
            kit = f"{robot}/find_{c.name}"
            slot = f"{robot}/slot_{c.name}"
            scene.add_vision_sensor(kit, camera=f"{robot}/visor", watch=[housing], detect_range=(0.05, 0.40))
            scene.add_vision_sensor(slot, camera=f"{robot}/visor", watch=[cell.headers[terminal]],
                                    detect_range=(0.05, 0.40))
            cell.finds[(c.name, robot, "kit")] = kit
            cell.finds[(c.name, robot, "slot")] = slot
            # A housing may meet its own header — on the header's axis, upright —
            # and the blister floor it lies on, over its own place.
            x, y = P.terminal_xy(terminal)
            scene.allow_object_obstacle_contact(housing, cell.headers[terminal],
                                                window=((x, y, P.HEADER_TOP), (0.0, 0.0, 1.0), 0.002,
                                                        math.radians(2.0)))
            scene.allow_object_obstacle_contact(housing, "kit/blister",
                                                window=(c.kit(side), (0.0, 0.0, 1.0), 0.002, math.radians(2.0)))
            # The fingers set a housing down and open off it: they touch it.
            for link in ("left_finger", "right_finger"):
                scene.allow_link_obstacle_contact(link, housing, robot=robot)
    guard(scene)
    return scene, cell


GUARD = (2.70, 3.10, 2.10)            # the guard's frame: along the line, across, tall
GUARD_X = 0.15


def guard(scene: bt.Scene) -> None:
    """An aluminium-frame guard with clear panels round both arms, the line
    passing through openings in its ends with a light curtain across each,
    a door in its front; outside it the cabinet and the two controllers at
    the back, the operator's station and an operator at the front, the
    walkway marked on the floor, the next carrier waiting upstream."""
    w, d, h = GUARD
    bt.parts.frame_unit(scene, "guard", GUARD, (GUARD_X, 0.0, 0.0), catalog=FRAME, template="enclosure",
                        section="3030", legs="3060", panels=["front", "back", "left", "right"],
                        openings={"left": ((-0.70, 0.70), (None, 1.20)), "right": ((-0.70, 0.70), (None, 1.20)),
                                  "front": ((0.20, 1.10), (None, 1.95))},
                        door="front", feet=True)
    for end, x in (("in", GUARD_X - w / 2 - 0.06), ("out", GUARD_X + w / 2 + 0.06)):
        bt.parts.light_curtain(scene, f"curtain_{end}", (x, -0.68), (x, 0.68), height=1.165, resolution=45,
                               catalog=CURTAIN, watch_robot=False, watch=[])
    bt.parts.cabinet(scene, "cabinet", (0.80, 0.50, 2.00), (1.05, d / 2 + 0.55), catalog=CABINET, yaw=math.pi)
    # The operator's station by the door: a post with the panel tilted up.
    sx, sy = GUARD_X - 0.65, -d / 2 - 0.30
    post = scene.add_box("station/post", size=(0.08, 0.08, 1.05), position=(sx, sy, 0.525), color=(0.62, 0.63, 0.65))
    scene.set_obstacle_material(post, metalness=0.6, roughness=0.4)
    scene.add_box("station/foot", size=(0.40, 0.40, 0.02), position=(sx, sy, 0.01), color=(0.25, 0.26, 0.28))
    bt.parts.operator_panel(scene, "station/panel", (sx, sy - 0.02, 1.16), tilt=math.radians(30), size=(0.36, 0.24),
                            buttons=("cycle_start", "cycle_stop", "reset", "estop"), columns=4)
    bt.parts.person(scene, "operator", (sx - 0.05, sy - 0.62), yaw=math.pi / 2)
    # The walkway: the keep-out line round the guard, the aisle along its front.
    m = 0.45
    bt.parts.marking(scene, "floor/keep_out", rect=(GUARD_X - w / 2 - m, -d / 2 - m, GUARD_X + w / 2 + m, d / 2 + m),
                     dash=(0.30, 0.15))
    bt.parts.marking(scene, "floor/aisle", line=((-3.1, -d / 2 - 1.35), (3.5, -d / 2 - 1.35)), width=0.10,
                     color=(0.08, 0.32, 0.12))
    P.waiting_carrier(scene, -2.30)


# ================================================================ teaching
READY_AT = (BASE_X, 0.62, 1.22)       # each arm's wait: high over its own side of the line
GRIPPING = ["left_finger", "right_finger", "left_jaw", "right_jaw", "mount", "grasp"]


@dataclass
class Taught:
    q: dict = field(default_factory=dict)       # (cable | "ready", robot, key) -> joint positions (arm only)
    at: dict = field(default_factory=dict)      # (cable, robot, key) -> TCP position


def _closest(q, seed, limits):
    """`q` with each revolute joint wrapped by whole turns to the value
    nearest the seed that its limits allow — per-point IK can land a turn
    away, and a ramp between two such solutions spins the wrist."""
    out = []
    for v, s, (lo, hi) in zip(q, seed, limits):
        best = v
        for k in (-1, 1):
            w = v + k * 2 * math.pi
            if lo <= w <= hi and abs(w - s) < abs(best - s):
                best = w
        out.append(best)
    return out


def teach(scene: bt.Scene, cell: Cell) -> Taught:
    """Every pose of both arms by IK, tool down, each warm-started from the
    one before it in the cycle; the two arms checked together at each. The
    long moves (the carry, the way back to the blister) get two via poses on
    the straight line between their ends: a joint ramp swings the tool on an
    arc round the base, and two arms reaching for the same channel would
    swing their wrists into each other."""
    taught = Taught()
    limits = scene.robot.joint_limits
    seeds = {}
    for robot in ROBOTS:
        s = SIDE[robot]
        seed = [0.0, 0.3, -0.3, 0.0, -1.3, 0.0, OPEN]
        q = _solve(scene, robot, (READY_AT[0], s * READY_AT[1], READY_AT[2]), seed, limits)
        taught.q[("ready", robot, "ready")] = q
        taught.at[("ready", robot, "ready")] = (READY_AT[0], s * READY_AT[1], READY_AT[2])
        seeds[robot] = q
    last = {robot: taught.at[("ready", robot, "ready")] for robot in ROBOTS}

    def pose(cable, robot, key, position, vias=0):
        start = last[robot]
        for i in range(1, vias + 1):
            u = i / (vias + 1)
            via = tuple(a + (b - a) * u for a, b in zip(start, position))
            q = _solve(scene, robot, via, seeds[robot], limits)
            seeds[robot] = q
            taught.q[(cable, robot, f"{key}~{i}")] = q
            taught.at[(cable, robot, f"{key}~{i}")] = via
        q = _solve(scene, robot, position, seeds[robot], limits)
        seeds[robot] = q
        taught.q[(cable, robot, key)] = q
        taught.at[(cable, robot, key)] = position
        last[robot] = position

    for c in cell.cables:
        for robot, side in (("L", "l"), ("R", "r")):
            s = SIDE[robot]
            kx, ky, kz = c.kit(side)
            px, py, pz = c.plugged(side)
            top = P.MT + TRANSFER_Z
            pose(c.name, robot, "over_kit", (kx, ky, top), vias=VIAS)
            pose(c.name, robot, "near_kit", (kx, ky, kz + NEAR))
            pose(c.name, robot, "kit", (kx, ky, kz))
            pose(c.name, robot, "out", (kx, ky - s * GATHER / 2, kz + NEAR))
            pose(c.name, robot, "lifted", (kx, ky - s * GATHER, top))
            pose(c.name, robot, "over", (px, py, top), vias=VIAS)
            pose(c.name, robot, "approach", (px, py, pz + APPROACH))
            pose(c.name, robot, "plugged", (px, py, pz))
            seeds[robot] = taught.q[(c.name, robot, "over")]
            last[robot] = taught.at[(c.name, robot, "over")]
        for key in ("over_kit", "near_kit", "kit", "lifted", "over~1", "over~2", "over", "plugged"):
            for robot in ROBOTS:
                scene.set_joint_positions([*taught.q[(c.name, robot, key)], OPEN], robot=robot)
            hits = [pair for pair in scene.check_collisions()
                    if not any(h in (pair[0][1], pair[1][1]) for h in cell.housings[c.name].values())]
            if hits:
                raise SystemExit(f"{c.name} {key}: the arms meet the cell at {hits[:3]}")
    for robot in ROBOTS:
        pose("home", robot, "ready", taught.at[("ready", robot, "ready")], vias=VIAS)
        scene.set_joint_positions([*taught.q[("ready", robot, "ready")], OPEN], robot=robot)
    return taught


def _solve(scene, robot, position, seed, limits):
    scene.set_joint_positions(list(seed[:6]) + [OPEN], robot=robot)
    result = scene.set_tcp_target(position, DOWN[robot], robot=robot)
    if not result.converged:
        raise SystemExit(f"{robot} cannot reach {tuple(round(v, 3) for v in position)} "
                         f"({result.pos_error * 1000:.1f} mm short)")
    q = list(scene.joint_positions_of(robot))[:6]
    return _closest(q, seed[:6], limits[:6])


# ================================================================ the program
def _seconds(q0, q1, path: float, speed: float, least: float = 0.3) -> float:
    """A ramp's duration: the slowest joint at its rated speed, or the tool
    at `speed` over `path` — a smoothstep peaks at 1.5 x its mean rate."""
    joint = max(abs(b - a) / w for a, b, w in zip(q0, q1, JOINT_SPEED))
    return round(max(1.5 * joint, 1.5 * path / speed, least), 2)


def program(scene: bt.Scene, cell: Cell, taught: Taught) -> str:
    """One program drives both arms, the way a team controller does: every
    move is one step holding a ramp for each arm with one duration, so
    both ends of the cable set off and arrive together."""
    names = scene.robot.joint_names[:6]
    sq = scene.sequence("connect")
    where = {robot: taught.q[("ready", robot, "ready")] for robot in ROBOTS}
    at = {robot: None for robot in ROBOTS}

    def move(name, cable, key, speed):
        keys = [k for k in (f"{key}~{i}" for i in range(1, VIAS + 1)) if (cable, "L", k) in taught.q] + [key]
        for i, k in enumerate(keys):
            targets = {r: taught.q[(cable, r, k)] for r in ROBOTS}
            dt = max(_seconds(where[r], targets[r],
                              0.0 if at[r] is None else math.dist(at[r], taught.at[(cable, r, k)]), speed)
                     for r in ROBOTS)
            label = name if len(keys) == 1 else f"{name} {i + 1}/{len(keys)}"
            sq.step(label, actions=[bt.seq.ramp(dict(zip(names, targets[r])), dt, robot=r, check=True)
                                    for r in ROBOTS])
            for r in ROBOTS:
                where[r], at[r] = targets[r], taught.at[(cable, r, k)]

    def look(name, cable, what):
        sq.step(name, transition=bt.seq.all_of(*(bt.seq.signal(cell.finds[(cable, r, what)]) for r in ROBOTS),
                                               bt.seq.elapsed(LOOK)))

    sq.step("pack in place", transition=bt.seq.elapsed(0.5))
    for r in ROBOTS:
        at[r] = taught.at[("ready", r, "ready")]
    for c in cell.cables:
        plugs = {r: cell.housings[c.name][s] for r, s in (("L", "l"), ("R", "r"))}
        move(f"{c.name}: over the kit", c.name, "over_kit", FREE)
        look(f"{c.name}: find the housings", c.name, "kit")
        move(f"{c.name}: down to the housings", c.name, "near_kit", FREE)
        move(f"{c.name}: onto the housings", c.name, "kit", 0.10)
        sq.step(f"{c.name}: grip",
                actions=[bt.seq.ramp({"finger_joint": SHUT}, 0.25, robot=r) for r in ROBOTS]
                + [bt.seq.attach(plugs[r], robot=r, touch_links=GRIPPING) for r in ROBOTS]
                + [bt.seq.set_signal(f"{r}/grip") for r in ROBOTS],
                transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(0.3)))
        move(f"{c.name}: out of the blister", c.name, "out", 0.08)
        move(f"{c.name}: lift", c.name, "lifted", CARRY)
        move(f"{c.name}: carry", c.name, "over", CARRY)
        look(f"{c.name}: find the slots", c.name, "slot")
        move(f"{c.name}: approach", c.name, "approach", CARRY)
        move(f"{c.name}: push home", c.name, "plugged", SEAT)
        sq.step(f"{c.name}: lock the CPA", actions=[bt.seq.set_signal("cpa_lock")],
                transition=bt.seq.elapsed(CPA))
        sq.step(f"{c.name}: let go",
                actions=[bt.seq.ramp({"finger_joint": OPEN}, 0.25, robot=r) for r in ROBOTS]
                + [bt.seq.detach(plugs[r]) for r in ROBOTS]
                + [bt.seq.set_signal(f"{r}/grip", False) for r in ROBOTS] + [bt.seq.set_signal("cpa_lock", False)],
                transition=bt.seq.all_of(bt.seq.done(), bt.seq.elapsed(0.3)))
        move(f"{c.name}: clear", c.name, "over", FREE)
    move("home", "home", "ready", FREE)
    return "connect"


# ================================================================ the cables
BENDING_HZ = 200.0                    # native bend-spring frequency: a stiff 35 mm² jacket, not a calibrated EI
SPACING = 0.010


def cables(scene: bt.Scene, timeline, cell: Cell):
    """Each connector as a rope against the finished cycle: its ends held in
    the two housings, lying on whatever it meets."""
    for c in cell.cables:
        housings = cell.housings[c.name]
        timeline = bt.rope.animate(
            scene, timeline, name=c.name, points=c.reference(),
            anchors=[bt.rope.anchor(housings["l"], location="Start", length_m=P.HELD),
                     bt.rope.anchor(housings["r"], location="End", length_m=P.HELD)],
            obstacles=P.rope_obstacles(scene, c, cell.housings),
            spacing_m=SPACING, radius_m=P.CABLE_R, density_kg_m=P.CABLE_KG_M,
            bending_hz=BENDING_HZ, friction=0.6, color=P.HV_ORANGE)
    return timeline


def bake(slack: float = 0.0, catalog_root: Path | None = None):
    """Build, teach, bake the cycle, then the cables: `(scene, timeline, info)`."""
    load = loader(catalog_root)
    scene, cell = build_cell(load, slack)
    taught = teach(scene, cell)
    name = program(scene, cell, taught)
    started = time.perf_counter()
    timeline = scene.simulate_sequence(name, max_duration=300.0)
    baked = time.perf_counter() - started
    started = time.perf_counter()
    timeline = cables(scene, timeline, cell)
    return scene, timeline, dict(cell=cell, taught=taught, bake_s=baked, rope_s=time.perf_counter() - started)


# ================================================================ the checks
def _span(timeline, step: str) -> tuple[float, float]:
    for name, a, b in timeline.step_spans:
        if name == step or name.endswith(f"/{step}"):
            return a, b
    raise KeyError(step)


def _box_gap(points: np.ndarray, centre, size) -> np.ndarray:
    """Distance from each point to an axis-aligned box (0 inside)."""
    d = np.abs(points - np.asarray(centre)) - np.asarray(size) / 2
    return np.linalg.norm(np.maximum(d, 0.0), axis=-1)


def _lv_gap(points: np.ndarray) -> np.ndarray:
    """Distance from each point to the BMS harness: its carrier (a box) and
    the branches up the modules' faces (vertical cylinders)."""
    x0, x1 = P.CHANNEL_X
    carrier = _box_gap(points, ((x0 + x1) / 2, 0.0, P.CARRIER_Z + P.CARRIER_H / 2),
                       (x1 - x0, P.CARRIER_W, P.CARRIER_H))
    gaps = [carrier]
    for side in (1.0, -1.0):
        for x in P.ROWS:
            dz = np.maximum(0.0, np.maximum(P.LV_Z - points[..., 2], points[..., 2] - P.MT))
            radial = np.hypot(points[..., 0] - x, points[..., 1] - side * P.BRANCH_Y) - P.BRANCH_R
            gaps.append(np.hypot(np.maximum(radial, 0.0), dz))
    return np.min(gaps, axis=0)


def _pipe_gap(points: np.ndarray) -> np.ndarray:
    """Distance from each point to the coolant pipes' surfaces."""
    return np.min([np.hypot(points[..., 1] - s * P.PIPE_Y, points[..., 2] - P.PIPE_Z) - P.PIPE_R
                   for s in (1.0, -1.0)], axis=0)


def _bend_radius(points: np.ndarray, stride: int = 2) -> float:
    """Smallest circumradius of samples `stride` apart along the centreline."""
    a, b, c = points[:-2 * stride], points[stride:-stride], points[2 * stride:]
    ab, bc, ca = (np.linalg.norm(b - a, axis=1), np.linalg.norm(c - b, axis=1), np.linalg.norm(a - c, axis=1))
    area2 = np.linalg.norm(np.cross(b - a, c - a), axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        r = np.where(area2 > 1e-12, ab * bc * ca / (2 * area2), np.inf)
    return float(np.min(r))


def cable_report(timeline, cell: Cell) -> list[dict]:
    """Per connector: how much longer than it lay in the blister it gets
    while the hands have it (pulled taut), how close it comes to the
    modules on the way over, and once plugged — how far it droops into the
    channel, its gap to the LV harness and to the pack floor, its tightest
    bend beyond the housings' exits."""
    rows = []
    modules = [((x, s * P.MODULE_Y, P.MODULE_Z + P.MODULE[2] / 2), P.MODULE) for s in (1, -1) for x in P.ROWS]
    for c in cell.cables:
        track = bt.rope.track(timeline, c.name)
        times = np.asarray(track["times"])
        points = np.asarray(track["points"])
        length = np.linalg.norm(np.diff(points, axis=1), axis=2).sum(axis=1)
        grip = _span(timeline, f"{c.name}: grip")[0]
        go = _span(timeline, f"{c.name}: let go")[1]
        lift = _span(timeline, f"{c.name}: lift")[0]
        approach = _span(timeline, f"{c.name}: approach")[0]
        rest = length[times < grip][-1]
        held = (times >= grip) & (times <= go)
        flying = (times >= lift) & (times < approach)
        # Along the reference: the free span is outside the 45 mm each housing holds.
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(points[0], axis=0), axis=1))])
        free = (s > P.HELD + 1e-6) & (s < c.length - P.HELD - 1e-6)
        clear_of_boots = (s > P.HELD + BOOT) & (s < c.length - P.HELD - BOOT)
        final = points[-1]
        exit_z = final[~free][:, 2].mean()
        rows.append(dict(
            cable=c.name, part=c.part_number, length=c.length,
            stretch=float(length[held].max() / rest - 1.0),
            flying_module_gap=min(float(_box_gap(points[flying][:, free], ctr, size).min())
                                  for ctr, size in modules) - P.CABLE_R,
            droop=float(exit_z - final[free][:, 2].min()),
            lv_gap=float(_lv_gap(final[free]).min()) - P.CABLE_R,
            pipe_gap=float(_pipe_gap(final[free]).min()) - P.CABLE_R,
            bend=_bend_radius(final[clear_of_boots]),
        ))
    return rows


def verdict(rows: list[dict]) -> list[str]:
    """What the cables refuse, by name. The bend radius is printed, not
    judged: the rope's bending stiffness is the solver's, not the cable's."""
    out = []
    for r in rows:
        mm = round(r["length"] * 1000)
        if r["stretch"] > STRETCH_MAX:
            out.append(f"{r['cable']} is pulled {100 * r['stretch']:.1f} % longer between the hands "
                       f"(over {100 * STRETCH_MAX:.0f} %): {mm} mm is too short")
        for key, what in (("lv_gap", "the LV harness"), ("pipe_gap", "the coolant manifold")):
            if r[key] < LV_CLEAR_MIN:
                near = (f"lies on {what}" if r[key] <= 0.0 else
                        f"comes {1000 * r[key]:.0f} mm from {what} (at least {1000 * LV_CLEAR_MIN:.0f} mm)")
                out.append(f"{r['cable']} {near}: {mm} mm droops too far")
        if r["flying_module_gap"] < 0.0:
            out.append(f"{r['cable']} drags across a module on the way over")
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default=str(HERE / "ev_battery_harness.usdc"))
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--slack", type=float, default=0.0,
                        help="metres added to every connector's length (negative: shorter)")
    parser.add_argument("--catalog-root", type=Path, default=None)
    args = parser.parse_args()

    scene, timeline, info = bake(args.slack, args.catalog_root)
    cell = info["cell"]
    n = len(cell.cables)
    print(f"{n} module connectors in {timeline.duration:.1f} s ({timeline.duration / n:.1f} s each); "
          f"cycle baked in {info['bake_s']:.1f} s, the cables in {info['rope_s']:.1f} s")
    for label, groups in (("the guard", ["guard"]), ("the pack", ["pack"]), ("the blister", ["kit"])):
        gap = timeline.min_clearance(0.02, to=groups)
        where = f" ({' / '.join(gap.pair)})" if gap.pair else ""
        print(f"  closest to {label}: {1000 * gap.distance:.1f} mm at {gap.t:.1f} s{where}")
    rows = cable_report(timeline, cell)
    print("  cable  part           while carried:  taut   flying over modules   plugged: droop  LV gap"
          "  pipe gap  bend R*")
    for r in rows:
        print(f"  {r['cable']:5s}  {r['part']:13s}  {100 * r['stretch']:+18.2f} %  {1000 * r['flying_module_gap']:13.0f} mm"
              f"   {1000 * r['droop']:12.0f} mm {1000 * r['lv_gap']:5.0f} mm {1000 * r['pipe_gap']:6.0f} mm"
              f" {1000 * r['bend']:6.0f} mm")
    print(f"  * beyond {1000 * BOOT:.0f} mm from the housings' exits, on the solver's bending stiffness "
          f"(indicative; the rule of thumb is {1000 * BEND_MIN:.0f} mm, 4 x OD)")
    refused = verdict(rows)
    for line in refused:
        print(f"  REFUSED: {line}")
    if not refused:
        print(f"  every connector: under {100 * STRETCH_MAX:.0f} % pulled, clear of the modules in flight, "
              f"{1000 * LV_CLEAR_MIN:.0f} mm or more off the LV harness and the coolant manifold")

    warnings = timeline.export_usd(args.out, fps=30.0)
    print(f"wrote {args.out}" + (f" ({len(warnings)} warnings)" if warnings else ""))

    if args.studio:
        bt.studio(scene, view=((-1.9, -2.6, 2.2), (0.05, 0.0, 0.80)))
    if refused:
        sys.exit(1)


if __name__ == "__main__":
    main()
