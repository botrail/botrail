"""A collaborative arm tends a machining centre: side door, vise, buttons.

The cell in the photograph every machine-tool builder prints: a compact
vertical machining centre with a robot at its side door, swapping the
finished part in the vise for a blank between cycles. Here it is built
from products and public figures rather than a CAD file — a MELFA ASSISTA
RV-5AS-D on a catalog robot stand, in front of a `bt.parts.machine_tool`
that stands the FANUC ROBODRILL α-D21MiB5 Plus of the public catalogue
as the *envelopes* a tending cell verifies against
(design/design-machine-tending.md §3): the side-door opening and its
sill, the table at its exchange position, the spindle head retracted over
it, the walls, the door leaf, the operator panel and its buttons.

The hand is the end-effector of the other photograph: one bracket
carrying a 2F-85 gripper for the workpiece, a pin for the buttons and a
fork for the door handle, and the robot *switches* between them by
turning its wrist so the right one faces the job. The bracket is a
catalog product (`botrail/hand/mph3`, whose URDF is generated from a
`bt.tools` layout), ordered like the arm and the gripper. Each tool is a
tip frame the teach aims at (`link=`): the pin presses square into a cap
with the fingers left open, the fork takes the handle bar between its
prongs and the door goes wherever the fork goes.

The machine has no robot interface — the retrofit. The robot presses
UNCLAMP with the pin, hooks the handle with the fork and *slides the door
open* (the leaf rides the fork through a straight-line move), unloads and
loads with the gripper, presses CLAMP, slides the door shut and presses
CYCLE START — each press a 2.6 mm push into a 22 mm button whose zone
sensor is the machine's input. The machine's side is a *program of its
own* (`bt.tending.manual`), scanned beside the robot's: it runs its cycle
on the start button and ignores a start pressed with the door open, the
way a guard interlock does. Two programs, scanned together.

What the bake is for: the door leaf, the clamp and every button are lanes
on the chart, in the order the cycle puts them; the press of each button
is checked against its neighbours — a lane that should stay flat stays
flat — and the pads, the pin and the prongs against everything else,
every tick.

What is handed over (`deliver`, written next to the USD): the layout
sheet with the door's stroke, the bill, the I/O list and the handshake
spec between the arm's controller and the CNC, the PLCopen file with the
machine's program on a resource of its own, the **interlock table** —
every output of both programs against the condition that admits it (the
start only with both doors confirmed shut and no E-stop) — and the cell
report with the machine's section and the **FAT rows**: three faults a
control designer would test for (the door's closed switch stuck, an open
wire on the CLAMP button, the E-stop in), each a scenario whose run must
refuse the cycle rather than let it through.

Run with:  python examples/machining/machine_tending_demo.py [out.usdc]
                 [--catalog] [--studio]

`--catalog` orders the machine and the vise from their catalog packs: the
ROBODRILL's openings, table, head and door come from
`fanuc/robodrill/alpha-d21mib5-plus`, the vise's jaws from
`botrail/fixture/vise-125` — the same cell, with every figure traceable
to a page and every line of the bill carrying a number.

Needs the catalog (`pip install botrail[catalog]`; the packages are
fetched from the Hugging Face dataset botrail/botrail-catalog and
cached).
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent

# --------------------------------------------------------------- products
ARM = "rv-5as-d"                 # Mitsubishi Electric MELFA ASSISTA, 5 kg / 910 mm
COUPLING = "gripper-coupling"
GRIPPER = "2f-85"
STAND = "sus/zf/robostand-crx"   # a robot stand, bought to the height the door wants
ROBOT = "arm"
# The machine and the vise as catalog products (`--catalog`): the
# ROBODRILL's envelope pack — openings, table, head, the options it sells
# — and a generic machine vise.
MACHINE_CATALOG = "fanuc/robodrill/alpha-d21mib5-plus"
VISE_CATALOG = "botrail/fixture/vise-125"
# The multi-purpose hand: the MPH-3 of the catalog (its URDF is generated
# from a `bt.tools` layout, so its links are named the way `bt.tools`
# names them). The gripper is the TCP; the pin and the fork are tips.
HAND = "mph3"
HAND_CATALOG = "botrail/hand/mph3"
PIN_TIP = bt.tools.tip(HAND, "pusher")
FORK_TIP = bt.tools.tip(HAND, "fork")
FORK = f"{HAND}_fork"

# ------------------------------------------------------------------ cell
# The machine stands at the origin, door on its right (+X). The stand is
# at the door, its top at the height the ROBODRILL robot package puts the
# pedestal (770 mm), the arm's base 5 mm proud of it (a mounted base is
# checked as part of the robot, and resting reads as touching).
STAND_H = 0.77
STAND_OUT = 0.25                 # the stand centre, past the `entry` frame
STOCKER_OUT = 0.75               # the stocker, on the far side of the stand
STOCKER = (0.50, 0.70, 0.80)     # a bench with the blank and the finished part
SLOT = 0.18                      # the two slots, either side of the bench centre

PART = (0.05, 0.05, 0.06)        # the workpiece: a 50 x 50 x 60 block
PART_MASS = 0.9
SEAT = 0.005                     # a carried part is set down proud of the surface
HOVER = 0.10                     # the straight-line approach above a grasp
GRIP = 0.005                     # the pads' centre below the part's top face
OPEN, SHUT = 0.0, 0.40           # finger joint: 92 mm across the pads, and 50
PRESS_STANDOFF = 0.03            # where a press starts, off the cap
HOOK_STANDOFF = 0.06             # where the fork comes at the handle from (the bar is 16 mm; the seat 50)

CYCLE_S = 20.0                   # a short part program, so the bake stays short
CLAMP_S = 0.8

BUTTONS = ("cycle_start", "clamp", "unclamp", "estop")
# The links a grip legitimately rests on.
PADS = [f"{side}_inner_{part}" for side in ("left", "right")
        for part in ("finger", "finger_pad", "knuckle")]

# Where the arm parks between cycles: turned to the machine's rear,
# folded over its own stand, tool down — a jog, not a work point, so it
# is typed in joints and checked at build. The seeds IK starts from.
READY = [-math.pi / 2, -0.6, 2.2, 0.0, 1.55, 0.0, OPEN]
SEEDS = (READY, [0.0, -0.3, 1.9, 0.0, 1.5, 0.0, OPEN], [0.0] * 6 + [OPEN])
# The branches a 6-axis arm can take to one pose — shoulder forward or
# back, elbow more or less bent, wrist down or level — as seeds tried
# after the ones a teach names, so a pose is refused only when *every*
# branch fouls or falls short.
BRANCHES = [[0.0, j2, j3, 0.0, j5, 0.0, OPEN]
            for j2 in (-0.6, 0.0, 0.6) for j3 in (1.0, 1.6, 2.2) for j5 in (0.8, 1.5)]
# A finer sweep behind the coarse one — tried only when the coarse seeds
# all fall short, so a pose the coarse ones reach keeps its branch.
BRANCHES += [[0.0, j2, j3, 0.0, j5, 0.0, OPEN]
             for j2 in (-0.6, -0.3, 0.0, 0.3, 0.6) for j3 in (0.6, 1.0, 1.4, 1.8, 2.2)
             for j5 in (0.4, 0.8, 1.2, 1.5, 1.9)
             if [0.0, j2, j3, 0.0, j5, 0.0, OPEN] not in BRANCHES]

# Linear-RGB colours.
BLANK = (0.55, 0.56, 0.58)
FINISHED = (0.70, 0.72, 0.75)


# --------------------------------------------------------------- helpers
def q_mul(a, b):
    """The Hamilton product a ⊗ b, quaternions as (x, y, z, w): the
    rotation `b` followed by `a`."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def rotate(q, v):
    """`v` turned by `q`."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (qy * vz - qz * vy), 2 * (qz * vx - qx * vz), 2 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty),
            vy + qw * ty + (qz * tx - qx * tz),
            vz + qw * tz + (qx * ty - qy * tx))


def down(spin: float) -> tuple:
    """Tool +Z at the floor, the finger pads closing along world X turned
    by `spin` — the 2F-85's pads part along its own Y."""
    return q_mul((1.0, 0.0, 0.0, 0.0), (0.0, 0.0, math.sin(spin / 2), math.cos(spin / 2)))


def quat_from_axes(x, y, z) -> tuple:
    """The quaternion of the frame whose columns are the unit axes."""
    m00, m10, m20 = x
    m01, m11, m21 = y
    m02, m12, m22 = z
    t = m00 + m11 + m22
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        return ((m21 - m12) / s, (m02 - m20) / s, (m10 - m01) / s, 0.25 * s)
    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2
        return (0.25 * s, (m01 + m10) / s, (m02 + m20) / s, (m21 - m12) / s)
    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2
        return ((m01 + m10) / s, 0.25 * s, (m12 + m21) / s, (m02 - m20) / s)
    s = math.sqrt(1.0 + m22 - m00 - m11) * 2
    return ((m02 + m20) / s, (m12 + m21) / s, 0.25 * s, (m10 - m01) / s)


def aimed(z, y) -> tuple:
    """A tool frame with +Z along `z` and +Y as near `y` as `z` allows —
    the tip's approach axis, and which way the rest of the hand hangs."""
    zx, zy, zz = z
    n = math.sqrt(zx * zx + zy * zy + zz * zz)
    z = (zx / n, zy / n, zz / n)
    d = sum(a * b for a, b in zip(y, z))
    y = tuple(a - d * b for a, b in zip(y, z))
    n = math.sqrt(sum(a * a for a in y))
    y = tuple(a / n for a in y)
    x = (y[1] * z[2] - y[2] * z[1], y[2] * z[0] - y[0] * z[2], y[0] * z[1] - y[1] * z[0])
    return quat_from_axes(x, y, z)


def along(pose, distance: float):
    """`pose` = (position, quaternion) moved `distance` along its own +Z."""
    (x, y, z), q = pose
    dx, dy, dz = rotate(q, (0.0, 0.0, distance))
    return (x + dx, y + dy, z + dz), q


# ------------------------------------------------------------------ build
def tool() -> bt.Robot:
    """The arm with its hand: the 2F-85 on a bracket that also carries a
    pin and a fork — three tools, one wrist, switched by turning it."""
    arm = bt.Robot.from_catalog(ARM)
    coupling, gripper = bt.Robot.from_catalog(COUPLING), bt.Robot.from_catalog(GRIPPER)
    # The gripper down the plate's axis (the hand's declared flange); the
    # pin out one side of the plate and the fork out the other, both in
    # the plane the pads open in — so with the pads across a part, pin
    # and fork lie along the side opening, and the hand passes it end-on.
    bracket = bt.Robot.from_catalog(HAND_CATALOG)
    stack = bracket.attach_tool(coupling, prefix="cpl_").attach_tool(gripper)
    return arm.attach_tool(stack)


def build(*, catalog: bool = False) -> tuple[bt.Scene, bt.tending.Handshake]:
    """The cell, taught and programmed. `catalog` orders the machine and
    the vise from their packs instead of typing their figures in."""
    scene = bt.Scene(tool(), name=ROBOT)

    # -- the machine, the vise, the parts ----------------------------------
    if catalog:
        # The pack carries the envelope and the options; what is written
        # here is only which options and where the panel goes. A manual
        # door is not one the pack sells — the leaf is the pack's, loose.
        vmc = bt.parts.machine_tool(
            scene, "vmc", catalog=MACHINE_CATALOG, door="manual", door_side="right",
            panel="door", buttons=BUTTONS,
        )
    else:
        vmc = bt.parts.machine_tool(
            scene, "vmc", door="manual", door_side="right",
            panel="door", buttons=BUTTONS, detail="full",
            model="α-D21MiB5 Plus", manufacturer="FANUC", mass_kg=2000,
        )
    (tx, ty, tz), _ = scene.frame("vmc/table")
    # The vise on the door side of the table: the table traverses to its
    # exchange position, the vise sits where the arm can reach across the
    # sill. The jaws take the part's 50 mm plus 2 mm a side.
    if catalog:
        vise = bt.parts.vise(scene, "vise", (tx + 0.25, ty, tz), opening=PART[1] + 0.004,
                             catalog=VISE_CATALOG, jaw_width=0.125)
    else:
        vise = bt.parts.vise(scene, "vise", (tx + 0.25, ty, tz), opening=PART[1] + 0.004,
                             model="VQ-125", manufacturer="ACME", mass_kg=12)
    (jx, jy, jz), _ = scene.frame(vise.frames[0])
    seat_z = jz + SEAT + PART[2] / 2
    scene.add_box("finished", size=PART, position=(jx, jy, seat_z), color=FINISHED)
    scene.set_part("finished", kind="obstacle", category="workpiece", model="WP-50", mass_kg=PART_MASS)

    # -- the stand and the stocker, at the door ----------------------------
    (ex, ey, _), _ = scene.frame("vmc/entry")
    stand_xy = (ex + STAND_OUT, ey)
    stand = bt.parts.pedestal(scene, "stand", catalog=STAND, height=STAND_H, position=stand_xy,
                              yaw=math.pi)
    (mx, my, mz), mq = scene.frame(stand.frames[0])
    scene.set_robot_base_pose((mx, my, mz + 0.005), mq, robot=ROBOT)
    # Bolted down: the base and the plate it stands on are one contact
    # the clearance measure must not report.
    scene.allow_link_obstacle_contact(scene.robot_of(ROBOT).link_names[0], "stand/top", robot=ROBOT)
    bench = bt.parts.table(scene, "stocker", size=STOCKER, position=(ex + STOCKER_OUT, ey),
                           model="WB-500", manufacturer="ACME", mass_kg=30)
    (bx, by, bz), _ = scene.frame(bench.frames[0])
    scene.add_box("blank", size=PART, position=(bx, by - SLOT, bz + SEAT + PART[2] / 2), color=BLANK)
    scene.set_part("blank", kind="obstacle", category="workpiece", model="WP-50-raw", mass_kg=PART_MASS)
    # The two slots as frames: where the finished part is set down, where
    # the blank is picked up — what the teach aims at.
    scene.add_frame("stocker/out", position=(bx, by + SLOT, bz))
    scene.add_frame("stocker/blank", position=(bx, by - SLOT, bz))
    # The pads touch the part they close on — that is what a grasp is, so
    # it is declared rather than found by the clearance measure.
    for part in ("finished", "blank"):
        for link in PADS:
            scene.allow_link_obstacle_contact(link, part, robot=ROBOT)

    # -- the machine's program: its cycle on the start button, the clamp
    # on its button, and no start with the door open ----------------------
    hs = bt.tending.manual(scene, vmc, cycle_s=CYCLE_S, clamp_s=CLAMP_S,
                           buttons=("unclamp", "clamp", "cycle_start"))

    teach(scene, vmc)
    program(scene, vmc, hs)

    # -- the FAT rows: each fault a scenario, each run must refuse the cycle
    # rather than let it through -----------------------------------------------
    scene.add_scenario("door_switch_stuck", faults=[bt.io.stuck(hs.signal("door_closed"), False)])
    scene.add_scenario("clamp_button_open", faults=[bt.io.open("vmc/panel/clamp")])
    scene.add_scenario("estop_pressed", faults=[bt.io.stuck(hs.signal("estop"), True)])
    return scene, hs


# ------------------------------------------------------------------ teach
DOOR_HOME: dict = {}


def slide_door(scene: bt.Scene, vmc: bt.parts.MachineTool, fraction: float) -> None:
    """Puts the leaf (and what rides on it) at `fraction` of its stroke —
    open for teaching the poses inside, shut again before the bake."""
    ax, ay, az = vmc.door_axis
    for name in vmc.door_objects:
        (x, y, z), q = scene.obstacle_pose(name)
        x0, y0, z0 = DOOR_HOME.setdefault(name, (x, y, z))
        d = fraction * vmc.door_travel
        scene.set_obstacle_pose(name, (x0 + ax * d, y0 + ay * d, z0 + az * d), q)


def teach(scene: bt.Scene, vmc: bt.parts.MachineTool, *, vise: str = "vise", stocker: str = "stocker",
          prefix: str = "", home: bool = True, press_standoff: float = PRESS_STANDOFF) -> None:
    """Every pose the arm works from, solved against the machine's own
    kinematics from the cell's frames — nothing typed in joints. The
    poses inside are taught with the door open: an arm through a closed
    leaf is a collision, not a pose.

    `vise` and `stocker` name the fixture in this machine and the bench
    its parts come and go from; `prefix` goes on every motion name, so
    one arm can be taught two machines (`a_enter`, `b_enter`); `home`
    teaches the shared park motion (once); `press_standoff` is where a
    press starts, off the cap — wider where the swing to the panel passes
    close to its plate."""
    limits = scene.robot_of(ROBOT).joint_limits
    name = vmc.name
    # Which way this machine lies from the base, as the first joint's
    # angle: every seed below faces it, so an arm between two machines is
    # taught the far one as readily as the near one.
    base_p, base_q = scene.robot_base_pose_of(ROBOT)
    (ex0, ey0, _), _ = scene.frame(f"{name}/entry")
    fx, fy, _ = rotate(base_q, (1.0, 0.0, 0.0))
    dx, dy = ex0 - base_p[0], ey0 - base_p[1]
    bearing = math.atan2(fx * dy - fy * dx, fx * dx + fy * dy)
    # A machine straight behind the base is at J1 = ±π: the same place,
    # but the IK converges from one winding and not the other, so every
    # seed is tried at each winding within the joint's range.
    lo1, hi1 = limits[0] or (-math.inf, math.inf)
    windings = [b1 for b1 in (bearing, bearing + 2 * math.pi, bearing - 2 * math.pi) if lo1 <= b1 <= hi1]
    branches = [[b1, *b[1:]] for b in BRANCHES for b1 in windings]
    scene.set_joint_positions(SEEDS[0], robot=ROBOT)
    DOOR_HOME.clear()

    def unwind(q: list, seed: list) -> list:
        """Each wrist the short way round from `seed` — a 6-axis IK hands
        back an angle, not a winding, and the ±200° wrists of this arm
        offer a full turn to lose between two taught points."""
        out = []
        for value, want, limit in zip(q, seed, limits):
            lo, hi = limit or (-math.inf, math.inf)
            best = value
            for turn in (-1, 1):
                other = value + turn * 2 * math.pi
                if lo - 1e-9 <= other <= hi + 1e-9 and abs(other - want) < abs(best - want):
                    best = other
            out.append(best)
        return out

    def solve(target, quat, *seeds: list, fingers: float = OPEN, link: str | None = None,
              strict: bool = False) -> list:
        """`link` names the tool tip the pose is taught for — the gripper's
        TCP by default, the pin's or the fork's. `strict` tries the given
        seeds only — for a pose that must stay on its predecessor's branch."""
        short, fouled = None, []
        for seed in (seeds if strict else (*seeds, *branches)):
            seed = list(seed[:6]) + [fingers]
            scene.set_joint_positions(seed, robot=ROBOT)
            ik = scene.set_tcp_target(target, quat, link=link, robot=ROBOT)
            if not ik.converged:
                short = ik.pos_error
                continue
            q = unwind(list(scene.joint_positions_of(ROBOT)), seed)
            q[-1] = fingers
            scene.set_joint_positions(q, robot=ROBOT)
            hits = [f"{a[1]} x {b[1]}" for a, b in scene.check_collisions()]
            if not hits:
                return q
            fouled = hits
        where = tuple(round(v, 3) for v in target)
        if short is not None and not fouled:
            raise RuntimeError(f"{ARM} cannot reach {where}: {short * 1e3:.0f} mm short")
        raise RuntimeError(f"every branch at {where} fouls: {', '.join(fouled)}")

    def joint(motion: str, q: list) -> list:
        scene.add_segment(prefix + motion, goal=q, robot=ROBOT)
        return q

    def line(motion: str, q: list) -> list:
        scene.add_segment(prefix + motion, goal=q, kind="cartesian_line", robot=ROBOT)
        return q

    # Parked over the stand, and waiting at the door — both with the
    # door shut, which is how the arm meets them.
    q_part = down(math.pi / 2)
    ready = list(READY)
    scene.set_joint_positions(ready, robot=ROBOT)
    if hits := [f"{a[1]} x {b[1]}" for a, b in scene.check_collisions()]:
        raise RuntimeError(f"the park pose fouls: {', '.join(hits)}")
    if home:
        scene.add_segment("home", goal=ready, robot=ROBOT)
    (jx, jy, jz), _ = scene.frame(f"{vise}/jaw")
    grasp_z = jz + SEAT + PART[2] - GRIP
    (ex, ey, _ez), _ = scene.frame(f"{name}/entry")
    wait = solve((ex, ey, grasp_z + HOVER), q_part, ready, *SEEDS)
    joint("approach", wait)
    # The straight run through the side opening, door open: over the
    # finished part, pads across the part's free faces (the jaws hold the
    # other two), then down onto it.
    slide_door(scene, vmc, 1.0)
    over = solve((jx, jy, grasp_z + HOVER), q_part, wait, ready)
    grasp = solve((jx, jy, grasp_z), q_part, over)
    line("enter", over)
    line("down", grasp)
    line("up", over)
    line("exit", wait)
    slide_door(scene, vmc, 0.0)

    # The stocker: the finished part's slot and the blank's, each a frame
    # on the bench top.
    for tag in ("out", "blank"):
        (sx, sy, sz), _ = scene.frame(f"{stocker}/{tag}")
        set_z = sz + SEAT + PART[2] - GRIP
        hi = solve((sx, sy, set_z + HOVER), q_part, ready, wait)
        lo = solve((sx, sy, set_z), q_part, hi)
        joint(f"to_{tag}", hi)
        line(f"set_{tag}" if tag == "out" else "down_blank", lo)
        line(f"clear_{tag}" if tag == "out" else "up_blank", hi)

    # The door, by the fork: its seat driven onto the handle bar along the
    # wall's normal, at either end of the stroke, with the gripper hanging
    # down out of the way. The fork tip frame has +Z along the prongs and
    # +Y up the plate's -Z, so +Y toward the ceiling hangs the gripper
    # toward the floor.
    (hx, hy, hz), handle_q = scene.frame(f"{name}/door/side/handle")
    into = rotate(handle_q, (0.0, 0.0, 1.0))         # into the leaf
    fq = aimed(into, (0.0, 0.0, 1.0))
    ax, ay, az = vmc.door_axis
    travel = vmc.door_travel
    ends = {"closed": (hx, hy, hz), "open": (hx + ax * travel, hy + ay * travel, hz + az * travel)}
    seed = [windings[0], -0.3, 1.9, 0.0, 1.5, 0.0, OPEN]
    # The four handle poses are solved on one IK branch — each end's
    # standoff from its handle pose, the open end from the closed one — so
    # the straight lines between them (take, slide, leave) never have to
    # change branch midway, which a cartesian line refuses.
    # A chain: the open end's standoff is solved first (the pose with the
    # least room), and each pose after it only from the one before, so
    # the whole door path — take, slide, leave, at both ends — stays on
    # one branch. A first seed whose chain breaks is dropped for the next.
    # The fingers are tucked (shut) for the door: an empty gripper hanging
    # open beside a leaf is what grazes it on the way in.
    offs = {tag: along((ends[tag], fq), -HOOK_STANDOFF)[0] for tag in ends}
    path = [("open", offs["open"], 1.0), ("open", ends["open"], 1.0),
            ("closed", ends["closed"], 0.0), ("closed", offs["closed"], 0.0)]
    poses: dict[tuple[str, str], list] = {}
    failure = None
    for first in (seed, ready, wait, *branches):
        prev, chain = first, {}
        try:
            for i, (tag, target, fraction) in enumerate(path):
                slide_door(scene, vmc, fraction)
                prev = solve(target, fq, prev, fingers=SHUT, link=FORK_TIP, strict=True)
                chain[(tag, "near" if i in (1, 2) else "off")] = prev
        except RuntimeError as err:
            failure = err
            continue
        poses = chain
        break
    if not poses:
        raise RuntimeError(f"no branch carries the fork along {name}'s door: {failure}")
    for tag in ("closed", "open"):
        joint(f"to_handle_{tag}", poses[(tag, "off")])
        line(f"take_handle_{tag}", poses[(tag, "near")])
        line(f"leave_handle_{tag}", poses[(tag, "off")])
    # The slide itself, with the leaf attached: one straight-line move
    # along the wall between the two handle poses.
    line("slide_open", poses[("open", "near")])
    line("slide_close", poses[("closed", "near")])
    slide_door(scene, vmc, 0.0)

    # The buttons, by the pin: its tip driven the button's travel into
    # the cap along the press frame's axis, from a standoff off the cap.
    # The pin tip frame has +Z along the pin and +Y up the plate's +Z, so
    # +Y at the floor hangs the gripper down.
    for button in ("unclamp", "clamp", "cycle_start"):
        (px, py, pz), frame_q = scene.frame(f"{name}/panel/{button}/press")
        pq = aimed(rotate(frame_q, (0.0, 0.0, 1.0)), (0.0, 0.0, -1.0))
        off = solve(along(((px, py, pz), pq), -press_standoff)[0], pq, seed, ready, wait, link=PIN_TIP)
        near = solve((px, py, pz), pq, off, link=PIN_TIP)
        joint(f"to_{button}", off)
        line(f"press_{button}", near)
        line(f"back_{button}", off)
    scene.set_joint_positions(ready, robot=ROBOT)


# ---------------------------------------------------------------- program
def program(scene: bt.Scene, vmc: bt.parts.MachineTool, hs: bt.tending.Handshake, *,
            sq=None, prefix: str = "", parts: tuple[str, str] = ("finished", "blank"),
            home: bool = True) -> None:
    """The robot's program for one machine, appended to `sq` (the `tend`
    sequence by default) with `prefix` on every step and motion — the
    same prefix `teach` used — swapping `parts = (finished, blank)`;
    `home` ends with the park motion."""
    S = bt.seq
    finger = scene.robot_of(ROBOT).joint_names[-1]
    sq = sq if sq is not None else scene.sequence("tend")
    finished, blank = parts

    def step(name: str, **kwargs) -> None:
        sq.step(prefix + name, **kwargs)

    def motion(name: str):
        return S.motion(prefix + name)

    def grip(value: float):
        return S.ramp({finger: value}, 0.4, robot=ROBOT)

    def unload_load() -> None:
        """Through the door: finished part out, blank in — the pads open
        on the blank once it sits in the jaws; the clamp is a button."""
        step("enter", actions=[motion("enter")])
        step("down", actions=[motion("down")])
        step("grip", actions=[grip(SHUT)])
        step("hold", actions=[S.attach(finished, touch_links="tool", robot=ROBOT)])
        step("up", actions=[motion("up")])
        step("exit", actions=[motion("exit")])
        step("to_out", actions=[motion("to_out")])
        step("set_out", actions=[motion("set_out")])
        step("release_out", actions=[S.detach(finished), grip(OPEN)])
        step("clear_out", actions=[motion("clear_out")])
        step("to_blank", actions=[motion("to_blank")])
        step("down_blank", actions=[motion("down_blank")])
        step("grip_blank", actions=[grip(SHUT)])
        step("hold_blank", actions=[S.attach(blank, touch_links="tool", robot=ROBOT)])
        step("up_blank", actions=[motion("up_blank")])
        step("approach_2", actions=[motion("approach")])
        step("enter_2", actions=[motion("enter")])
        step("load", actions=[motion("down")])
        step("release", actions=[S.detach(blank), grip(OPEN)])
        step("up_2", actions=[motion("up")])
        step("exit_2", actions=[motion("exit")])

    # -- the door on the fork (fingers tucked), the buttons under the pin
    # (fingers open) ---------------------------------------------------------
    def press(button: str) -> None:
        step(f"to_{button}", actions=[motion(f"to_{button}")])
        step(f"press_{button}", actions=[motion(f"press_{button}")])
        step(f"hold_{button}", transition=S.elapsed(0.2))
        step(f"back_{button}", actions=[motion(f"back_{button}")])

    def slide(tag: str, move: str) -> None:
        # The fork takes the handle bar between its prongs, and the leaf
        # (with what rides on it) is carried by the fork for the slide.
        # The empty gripper is tucked shut for the door and opened again
        # after it — hanging open, its pads are what grazes the leaf.
        step(f"tuck_{tag}", actions=[grip(SHUT)])
        step(f"to_handle_{tag}", actions=[motion(f"to_handle_{tag}")])
        step(f"take_handle_{tag}", actions=[motion(f"take_handle_{tag}")])
        step(f"hold_door_{tag}", actions=[S.attach(o, link=FORK, robot=ROBOT) for o in vmc.door_objects])
        step(move, actions=[motion(move)])
        step(f"let_go_{tag}", actions=[S.detach(o) for o in vmc.door_objects])
        end = "open" if tag == "closed" else "closed"
        step(f"leave_handle_{tag}", actions=[motion(f"leave_handle_{end}")])
        step(f"untuck_{tag}", actions=[grip(OPEN)])

    step("wait_done", transition=S.signal(hs.signal("running"), False))
    press("unclamp")
    slide("closed", "slide_open")
    step("approach", actions=[motion("approach")])
    unload_load()
    press("clamp")
    slide("open", "slide_close")
    press("cycle_start")
    if home:
        sq.step("home", actions=[S.motion("home")])


# ------------------------------------------------------------------- bake
def bake(*, catalog: bool = False):
    scene, hs = build(catalog=catalog)
    tl = scene.simulate_sequences(["tend", hs.program], max_duration=240.0)
    return scene, hs, tl


# ------------------------------------------------------------- hand-over
def deliver(scene: bt.Scene, tl: bt.SequenceTimeline, out: Path):
    """The document set, written into `out` from the one source, and the
    report that hashes it: the drawing, the bill, the I/O and the
    handshake, the two programs as PLCopen, the interlock table, and the
    FAT scenarios' verdicts. Returns `(report, runs)`."""
    out.mkdir(parents=True, exist_ok=True)
    files: list[Path] = []

    def write(name: str, fn) -> Path:
        path = out / name
        fn(path)
        files.append(path)
        return path

    write("machine_tending.botrail", scene.save_project)
    write("machine_tending.py", lambda p: p.write_text(scene.generate_python()))
    write("machine_tending_bom.csv", scene.export_bom)
    write("machine_tending_bom.md", scene.export_bom)
    write("machine_tending_io.csv", scene.export_io_list)
    write("machine_tending_topology.mmd", scene.export_topology)
    write("machine_tending_handshake.md", tl.export_handshake_spec)
    write("machine_tending.plcopen.xml", lambda p: scene.export_plcopen(p, name="machine tending"))
    write("machine_tending_interlocks.md", scene.export_interlocks)
    write("machine_tending_interlocks.csv", scene.export_interlocks)
    write("machine_tending_layout.svg", lambda p: scene.export_layout(p, scale=120, title="machine tending"))
    write("machine_tending_layout.dxf", lambda p: scene.export_layout(p, title="machine tending"))
    # The scenario matrix: the baseline and the three faults. A refused
    # cycle never completes, so the faults run to the cap and stall.
    runs = scene.simulate_scenarios(["tend", "vmc"], max_duration=tl.duration + 20.0)
    report = scene.cell_report({"baseline": tl}, scenarios=runs, deliverables=files,
                               title="machine tending")
    report.save(out / "machine_tending_report.md")
    report.save(out / "machine_tending_report.json")
    return report, runs


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default=str(HERE / "machine_tending.usdc"))
    parser.add_argument("--catalog", action="store_true",
                        help=f"order the machine ({MACHINE_CATALOG}) and the vise ({VISE_CATALOG}) from the catalog")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    scene, hs, tl = bake(catalog=args.catalog)
    print(f"{hs.template}{' / catalog' if args.catalog else ''}: cycle {tl.duration:.2f}s")
    for name, t0, t1 in tl.step_spans:
        print(f"  {name:<28} {t0:7.2f} - {t1:7.2f}s")
    lanes = [hs.signal("door_closed"), hs.signal("door_open"), hs.signal("clamp"), hs.signal("running"),
             *(f"vmc/panel/{button}" for button in BUTTONS)]
    for lane in lanes:
        spans = ", ".join(f"{a:.2f}-{b:.2f}" for a, b in tl.signal(lane).high_spans())
        print(f"  {lane:<28} on: {spans or '-'}")
    clearance = tl.min_clearance()
    pair = f" ({clearance.pair[0]} x {clearance.pair[1]})" if clearance.pair else ""
    print(f"min clearance over the cycle: {float(clearance) * 1e3:.1f} mm at {clearance.t:.2f}s{pair}")

    warnings = tl.export_usd(args.out, fps=60)
    print(f"wrote {args.out}" + (f" ({warnings})" if warnings else ""))
    print(scene.bom().to_markdown())
    out_dir = Path(args.out).with_name("machine_tending_deliverables")
    report, runs = deliver(scene, tl, out_dir)
    print(scene.interlocks([hs.program]).to_markdown())
    for name in ("baseline", *scene.scenario_names):
        verdict = runs.errors.get(name)
        print(f"  scenario {name:<20} {'ok' if verdict is None else 'refused — ' + verdict}")
    print(f"wrote the document set to {out_dir}/ ({len(report.deliverables)} files hashed in the report)")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
