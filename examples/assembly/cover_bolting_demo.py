"""A cobot fits a gearbox cover over its dowels and screws it down.

The cell every screwdriving-tool vendor shows: a collaborative arm at a
bench, a screw presenter at its elbow, a housing on a fixture. Here it
is built from products and public figures (design/design-assembly.md): a
UR5e on a catalog robot stand, an OnRobot RG6 and **Screwdriver 103961 on one wrist**
(on a Dual Quick Changer v3, with the driver supported at its side
and a Type A 50 mm bit extender for access past the cover boss), a screw presenter
(`bt.parts.screw_feeder`, an indexing source with a presence sensor over
its pick point) and a **bolted joint** (`bt.assembly.joint`): the
housing's six M5 threads and two dowels, the cover's clearance holes,
the screw — ISO 4762 M5×20, 8.8 — and the torque and the thread
engagement the joint is designed for.

The cycle: the gripper takes the cover by its bearing boss (a close
derived from the mesh, `grasp_close`), carries it over the housing and
sets it on the dowels; then, screw by screw in the **star order** the
joint derives — the middle pair, then the diagonals — the bit picks a
screw from the presenter, carries it over its hole, the driver's
program (`bt.assembly.driver`, a program of its own scanned beside the
robot's) finds the drive, runs the screw down at 340 rpm for exactly
the time M5×0.8 takes, pre-tightens, final-tightens at 40 rpm and
answers OK — while the tool's own 55 mm stroke ramps the screw into the
hole. A NOK backs the bit out and runs once more; a second NOK releases
the screw where it stands and halts the cell.

What the bake says (`bt.assembly.fastening_report`): for every screw,
how far off its hole's axis it engaged and at what tilt, how far it went
down, how long the rundown took, how many attempts, OK or NOK — and the
figures no bake changes: 10 mm of thread engaged against the 10 mm the
joint needs, the tip 10 mm into a 13 mm hole, 4.5 N·m inside the
joint's 4.0–5.0 and the driver's 0.15–5. Run it with `--length 16` and
the joint is refused before anything moves: an M5×16 through a 10 mm
cover engages 6 mm.

What is handed over (`deliver`, next to the USD): the layout sheet, the
bill (six screws as one line with a count), the I/O list and the
handshake between the arm's controller and the driver's, PLCopen with
the driver's program on a resource of its own, the **interlock table**
(the first screw picked only once the cover is seated, no start raised
with the E-stop in), the **tightening sheet** (`bt.assembly.export_sheet`)
and the cell report with an assembly section and the **FAT rows** — a
NOK retried and passed, a NOK twice (halts, the screw released where it
stands), the E-stop in (no run admitted), the presenter empty (no screw
comes), the START wire open (the driver never hears the arm) — each a
scenario whose run must refuse the cycle rather than let it through.

Run with:  python examples/assembly/cover_bolting_demo.py [out.usdc]
                 [--length 20] [--rpm 340] [--catalog [--catalog-root DIR]] [--studio]

Appearance and product references: cover_bolting_demo.md. The TRUSCO AE-1500
bench, Schneider XALK178F E-stop enclosure, custom nests and equipment detail
are built in Python; the workpiece display meshes follow the joint dimensions.

Needs the catalog (`pip install botrail[catalog]`; the UR5e, RG6 and
stand are fetched from botrail/botrail-catalog and cached). `--catalog`
also orders the presenter (`botrail/feeder/screw-presenter-m2-m6`), screws
(`botrail/fastener/iso4762-m5`) and housing-and-cover set
(`botrail/workpiece/gear-cover-set`), whose mounting declares the joint.
`--catalog-root DIR` reads those assembly packs from a local build.
The OnRobot changer and screwdriver use the same dimensioned reference
geometry in both modes; the former generic SD5-340 pack is not used.
See cover_bolting_demo.md for the purchased stack, Compute Box wiring
route and limits of the reference geometry and simulated handshake.

A fitted part meets what it is fitted into, and the check knows it is
meant to: the cover is allowed to meet the housing and its dowels (they
are collided like everything else), and each screw is allowed inside the
cover and the housing only within its hole's window — its tip on the
hole's axis within the hole's tolerance. The engage move, which is
checked, puts the tip 3 mm into the hole; a screw taught 2 mm off its
hole (`--misalign 2`) is refused there, by name. The rundown itself is
the tool's own stroke, a ramp, which is not collision-checked, as a
gripper's close is not.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import _cover_bolting_cell as appearance
import _cover_bolting_tooling as tooling
import botrail as bt

HERE = Path(__file__).resolve().parent
A = bt.assembly

# --------------------------------------------------------------- products
ARM = "universal_robots/ur/ur5e/r2"  # 5 kg / 850 mm
STAND = "sus/zf/robostand-crx"    # a robot stand
# Optional assembly packs; the commercial end-effector stack is the same
# in both modes.
FEEDER_CATALOG = "botrail/feeder/screw-presenter-m2-m6/r1"
SCREW_CATALOG = "botrail/fastener/iso4762-m5/r1"
WORKPIECE_CATALOG = "botrail/workpiece/gear-cover-set/r1"
ROBOT = "arm"
BIT = "drv_bit"
SHANK = "drv_shank"
TIP = "drv_tip"
# OnRobot Screwdriver 103961, side-mounted reference geometry:
# 0.15–5 N·m, M1.6–M6 to 50 mm, 340 rpm, 55 mm shank stroke,
# 2.5 kg, 308 × 86 × 114 mm. Not vendor CAD.
DRIVER_MODEL = tooling.DRIVER_MODEL
DRIVER_MASS = tooling.DRIVER_MASS
STROKE = 0.055
TOOL = {"torque_nm": (0.15, 5.0), "thread_mm": (1.6, 6.0), "screw_length_mm": 50, "bit_mm": 4}
RPM_RUN = 340.0

# The joint: an aluminium gear housing 160 × 120 × 60 with six M5 threads
# and two ø6 dowels, a 10 mm cover with a ø45 × 30 bearing boss, ISO 4762
# M5×20 class 8.8 tightened to 4.0–5.0 N·m (the housing is aluminium:
# the class's 6.15 N·m ceiling does not apply) with 2×d engaged.
HOUSING = (0.160, 0.120, 0.060)
COVER = (0.160, 0.120, 0.010)
BOSS = (0.045, 0.030)             # diameter, height — the bearing boss the gripper takes
HOLES_MM = ((-60, -45), (60, -45), (60, 45), (-60, 45), (0, 45), (0, -45))
DOWELS_MM = ((-60, 0), (60, 0))
DOWEL = (0.006, 0.008)            # diameter, height
THREAD_DEPTH_MM = 13.0
MIN_ENGAGEMENT_MM = 10.0
TORQUE_NM = (4.0, 5.0)
TORQUE_SET = 4.5
SCREW_LENGTH_MM = 20
HOUSING_MASS = 3.1
COVER_MASS = 0.6

# ------------------------------------------------------------------ cell
STAND_H = 0.77
BENCH = appearance.BENCH
BENCH_XY = (0.95, 0.0)           # leaves the catalog stand clear of the table
HOUSING_XY = (0.55, 0.02)
STOCK_XY = (0.55, -0.30)          # where the cover waits
FEEDER_XY = (0.34, 0.28)
REJECT_XY = (1.90, 0.20)  # a tray beyond the bench's end: where the screws are when the presenter is empty
PANEL_AT = (1.25, 0.28, 0.91)
CONTROLLER_CATALOG = "universal_robots/control-box/e-series"
CONTROLLER_AT = (-0.60, -0.08)    # beside the stand, door facing the operator
HOVER = 0.05                      # straight-line approach above a pick
GRIP = 0.008                      # the pads' centre below the boss's top
CLEAR = 0.06                      # above the engaged screw, between holes
RETRY = 1

# Where the arm parks: turned to the bench, tools up and clear — a jog,
# typed in joints and checked at build. The seed IK starts from.
READY = [0.0, -1.2, 1.0, -1.0, -1.57, 0.0]
BRANCHES = [[0.0, j2, j3, j4, j5, 0.0]
            for j2 in (-0.6, -1.0, -1.4, -1.8) for j3 in (0.6, 1.0, 1.4, 1.8, 2.2)
            for j4 in (-2.5, -1.57, -0.8, 0.0) for j5 in (-1.57, 0.0, 1.57)]

# Linear-RGB colours.
ALUMINIUM_CAST = (0.50, 0.51, 0.53)
ALUMINIUM_COVER = (0.62, 0.63, 0.66)


# --------------------------------------------------------------- helpers
def q_mul(a, b):
    """The Hamilton product a ⊗ b, quaternions as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def zrot(angle: float) -> tuple:
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def down(spin: float) -> tuple:
    """Tool +Z at the floor, the pads closing along world X turned by `spin`."""
    return q_mul((1.0, 0.0, 0.0, 0.0), zrot(spin))


# ------------------------------------------------------------------ build
def catalog_ref(product: str, root):
    """A pack by id — or, with a local build root, its package directory."""
    return str(Path(root) / product) if root else product


def hand() -> bt.Robot:
    """Catalog RG6 and side-mounted 103961 on a commercial Dual QC v3.

    The driver and changer are runtime dimensioned reference models in both
    modes; --catalog selects the feeder, fasteners and workpiece packs.
    """
    arm = bt.Robot.from_catalog(ARM)
    stack = tooling.dual_changer().attach_tool(
        tooling.screwdriver(stroke=STROKE), flange="hand_driver", mount="mount", prefix="drv_")
    gripper = bt.Robot.from_catalog(tooling.GRIPPER_CATALOG)
    stack = stack.attach_tool(gripper, flange="hand_gripper", prefix="gripper_")
    robot = arm.attach_tool(stack)
    # The arm's six joints are the group IK solves with: the shank and the
    # fingers keep their values whichever tip a pose is taught for.
    return robot.define_group("arm", tip=robot.tcp_link, joints=robot.joint_names[:6])


def build(*, length_mm: int = SCREW_LENGTH_MM, rpm: float = RPM_RUN, misalign_mm: float = 0.0,
          catalog: bool = False, catalog_root=None):
    """The cell, taught and programmed: the scene, the joint, the driver,
    the feeder, the placement and the fastening. `length_mm` is the
    screw's; a length the joint cannot use is refused here. `misalign_mm`
    teaches the last screw that far off its hole — the bake then refuses
    its engage move, the screw meeting the cover outside its window.
    `catalog` orders the presenter, screws and workpiece set from packs
    (`catalog_root`: a local build)."""
    scene = bt.Scene(hand(), name=ROBOT)
    robot = scene.robot_of(ROBOT)
    scene.set_part(f"{ROBOT}/tool", manufacturer="OnRobot", model=tooling.CHANGER_MODEL,
                   category="tool.multi", description="two integral angled QC faces; dimensioned reference",
                   mass_kg=tooling.CHANGER_MASS, source_url=tooling.CHANGER_SOURCE,
                   part_number="109878", electrical_route="Compute Box; not robot wrist power")
    scene.set_part(f"{ROBOT}/tool/tool2", manufacturer="OnRobot", model=DRIVER_MODEL,
                   category="tool.screwdriver", description="side-mounted 103961; dimensioned reference",
                   mass_kg=DRIVER_MASS, part_number="103961", source_url=tooling.DRIVER_SOURCE,
                   torque_min_nm=TOOL["torque_nm"][0], torque_max_nm=TOOL["torque_nm"][1],
                   thread_min_mm=TOOL["thread_mm"][0], thread_max_mm=TOOL["thread_mm"][1],
                   screw_length_mm=TOOL["screw_length_mm"], bit_mm=TOOL["bit_mm"], rpm=rpm,
                   stroke_mm=STROKE * 1e3, current_max_a=4.5,
                   required_extender="109301: Bit Extender A 50 mm; included in bit geometry",
                   required_bit_kit="105121: Metric Kit; HEX 4 mm, Type A, M5 carrier and screw fix",
                   accessory_source="https://b2b.onrobot.com/accessories2/",
                   accessory_mass="unknown; excluded from 2.5 kg driver mass")

    scene.set_part(f"{ROBOT}/tool/tool3", part_number="102021",
                   electrical_route="Dual QC 109878 / Compute Box, not a single-QC wrist-power kit")
    # -- the stand, the bench ---------------------------------------------
    stand = bt.parts.pedestal(scene, "stand", catalog=STAND, height=STAND_H, position=(0.0, 0.0))
    (mx, my, mz), mq = scene.frame(stand.frames[0])
    scene.set_robot_base_pose((mx, my, mz + 0.005), mq, robot=ROBOT)
    scene.allow_link_obstacle_contact(robot.link_names[0], "stand/top", robot=ROBOT)
    appearance.box(scene, "stand/adapter", (0.19, 0.19, 0.005), (mx, my, mz + 0.0025),
                   appearance.METAL, metal=0.85)
    scene.set_part("stand/adapter", manufacturer="botrail", model="UR5e-to-ZF adapter (custom)",
                   category="adapter", description="5 mm demo plate; mounting pattern requires engineering")
    bench = appearance.bench(scene, BENCH_XY)
    (_bx, _by, bz), _ = scene.frame(bench.frames[0])

    # -- the housing on its fixture, the cover at its stock ------------------
    hx, hy = HOUSING_XY
    top = bz + HOUSING[2]
    sx, sy = STOCK_XY
    fx, fy = FEEDER_XY
    if catalog:
        # The set from its pack: the housing with its dowels where the pack's
        # flange face puts them, the cover at its stock; the joint read from
        # the same pack — its holes, its screw, its torque, its engagement.
        work = bt.parts.workpiece(scene, "set", (hx, hy, bz), catalog=catalog_ref(WORKPIECE_CATALOG, catalog_root),
                                  cover_at=(sx, sy, bz))
        dowels = list(work.dowels)
        scene.set_obstacle_material(work.housing, metalness=1.0, roughness=0.7)
        screw = A.Fastener.from_catalog(catalog_ref(SCREW_CATALOG, catalog_root), length_mm=length_mm)
        joint = A.joint(scene, "cover_joint", a=work.housing, b=work.cover, seat=work.seat,
                        catalog=catalog_ref(WORKPIECE_CATALOG, catalog_root), fastener=screw)
    else:
        scene.add_box("set/housing", size=HOUSING, position=(hx, hy, bz + HOUSING[2] / 2), color=ALUMINIUM_CAST)
        scene.set_obstacle_material("set/housing", metalness=1.0, roughness=0.7)
        dowels = []
        for i, (dx, dy) in enumerate(DOWELS_MM):
            name = f"set/housing/dowel{i}"
            scene.add_cylinder(name, DOWEL[0] / 2, DOWEL[1], (hx + dx / 1e3, hy + dy / 1e3, top + DOWEL[1] / 2),
                               color=(0.30, 0.30, 0.32))
            dowels.append(name)
        scene.set_part("set/housing", kind="obstacle", category="workpiece", model="GH-160 housing",
                       manufacturer="botrail", mass_kg=HOUSING_MASS)
        bt.parts.compound(scene, "set/cover", [
            bt.parts.Box(COVER, at=(0.0, 0.0, COVER[2] / 2)),
            bt.parts.Cylinder(BOSS[0] / 2, BOSS[1], at=(0.0, 0.0, COVER[2])),
        ], position=(sx, sy, bz), color=ALUMINIUM_COVER, finish=(0.0, 0.45))
        scene.set_part("set/cover", kind="obstacle", category="workpiece", model="GH-160 cover",
                       manufacturer="botrail", mass_kg=COVER_MASS)
        # -- the joint: what the drawing states ----------------------------
        screw = A.iso4762(5, length_mm)
        threads = A.BoltPattern(
            [A.Hole(f"h{i}", xy, "threaded", diameter_mm=5.0, depth_mm=THREAD_DEPTH_MM) for i, xy in enumerate(HOLES_MM)],
            [A.Locator(f"p{i}", xy, "pin", DOWEL[0] * 1e3, DOWEL[1] * 1e3) for i, xy in enumerate(DOWELS_MM)],
        )
        clearances = A.BoltPattern(
            [A.Hole(f"h{i}", xy, "clearance", diameter_mm=5.5, grip_mm=COVER[2] * 1e3) for i, xy in enumerate(HOLES_MM)],
            [A.Locator(f"p{i}", xy, "hole", DOWEL[0] * 1e3 + 0.1, 10.0) for i, xy in enumerate(DOWELS_MM)],
        )
        joint = A.joint(scene, "cover_joint", a="set/housing", b="set/cover", pattern_a=threads, pattern_b=clearances,
                        fastener=screw, seat=((hx, hy, top), (0.0, 0.0, 0.0, 1.0)), thickness=COVER[2],
                        torque_nm=TORQUE_NM, min_engagement_mm=MIN_ENGAGEMENT_MM)
    for link in tooling.pads(robot):
        scene.allow_link_obstacle_contact(link, joint.b, robot=ROBOT)

    appearance.fixtures(scene, housing_xy=HOUSING_XY, stock_xy=STOCK_XY, top=bz,
                        housing_size=HOUSING, cover_size=COVER)
    appearance.workpiece_detail(scene, joint, housing_size=HOUSING, cover_size=COVER, boss=BOSS)

    # -- the presenter with its magazine -----------------------------------
    if catalog:
        feeder = bt.parts.screw_feeder(scene, "feeder", (fx, fy, bz), screws=len(joint.order), fastener=screw,
                                       catalog=catalog_ref(FEEDER_CATALOG, catalog_root))
    else:
        screws = [screw.place(scene, f"screw{i}", (fx, fy, bz), manufacturer="botrail") for i in range(len(joint.order))]
        feeder = bt.parts.screw_feeder(scene, "feeder", (fx, fy, bz), screws=screws,
                                       model="SP-M5 (reference)", manufacturer="botrail", present_s=1.5)
    for name in feeder.screws:
        scene.allow_link_obstacle_contact(BIT, name, robot=ROBOT)
    appearance.feeder_detail(scene, feeder)

    # -- the panel: an E-stop the driver is guarded by -----------------------
    panel = appearance.panel(scene, PANEL_AT, ROBOT)
    estop = panel.sensors[0]

    # -- the two controllers: the arm's box from its pack; the driver's
    # handshake uses abstract simulation channels, not a Compute Box pinout.
    # See the product notes for the physical connection route. ----------------
    bt.parts.controller(scene, "controller", robots=[ROBOT], position=CONTROLLER_AT,
                        catalog=CONTROLLER_CATALOG, programs=["assemble"], cable_m=6)
    driver = A.driver(scene, "driver", robot=ROBOT, bit=BIT, shank=SHANK, stroke=STROKE, fastener=joint.fastener,
                      torque_nm=TORQUE_SET, cycles=len(joint.order) + RETRY, rpm_run=rpm, tool=TOOL,
                      estop=estop, channels=bt.io.channels("di", 8, "DI") + bt.io.channels("do", 8, "DO"))
    appearance.compute_box(scene, driver.node, (1.38, 0.22, bz))
    scene.add_spin("driver/spin", driver.signal("run"), ROBOT, link=BIT)
    problems = [f for f in A.check(joint, driver) if f.severity == "error"]
    if problems:
        raise ValueError("; ".join(f.message for f in problems))

    close = teach(scene, joint, feeder, driver, misalign_mm=misalign_mm)
    placement, fastening = program(scene, joint, driver, feeder, close,
                                   touch=[joint.a, *dowels, "bench/top"])
    wiring = scene.auto_assign_io(["assemble", driver.program])
    if wiring.errors():
        raise ValueError("wiring: " + "; ".join(f.message for f in wiring.errors()))

    # -- the FAT rows: each fault a scenario, each run must refuse the cycle
    # rather than let it through ---------------------------------------------
    scene.add_scenario("nok_once", signals={driver.signal("fault_first"): True})
    scene.add_scenario("nok_twice", signals={driver.signal("fault_first"): True, driver.signal("fault_retry"): True})
    scene.add_scenario("estop_pressed", faults=[bt.io.stuck(estop, True)])
    # The presenter's magazine empty — its screws are in the reject tray —
    # so a request presents nothing.
    scene.add_scenario("feeder_empty", obstacles={
        s: (REJECT_XY[0], REJECT_XY[1] + 0.02 * i, bz) for i, s in enumerate(feeder.screws)})
    # The START wire to the driver broken: the arm raises it, the driver
    # never hears it.
    scene.add_scenario("start_wire_open", faults=[bt.io.open(driver.signal("start"))])
    return scene, joint, driver, feeder, placement, fastening


# ------------------------------------------------------------------ teach
def teach(scene: bt.Scene, joint: A.Joint, feeder: bt.parts.ScrewFeeder, driver: A.Driver, *,
          misalign_mm: float = 0.0) -> dict:
    """Every pose, solved from the cell's frames against the arm's own
    kinematics — nothing typed in joints but the park. Returns the
    gripper's close on the cover's boss. `misalign_mm` shifts the last
    screw's taught poses off their hole along +X."""
    robot = scene.robot_of(ROBOT)
    limits = robot.joint_limits
    names = robot.joint_names
    extra = robot.dof - 6
    finger = names[-1]

    def full(q6, fingers: float = 0.0) -> list:
        q = list(q6[:6]) + [0.0] * extra
        q[-1] = fingers
        return q

    def hits() -> list[str]:
        return [f"{a[1]} x {b[1]}" for a, b in scene.check_collisions()]

    def unwind(q: list, seed: list) -> list:
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

    def solve(target, quat, *seeds, link=None, fingers: float = 0.0, strict: bool = False) -> list:
        short, fouled = None, []
        for seed in (seeds if strict else (*seeds, *(full(b) for b in BRANCHES))):
            seed = full(seed, fingers)
            scene.set_joint_positions(seed, robot=ROBOT)
            ik = scene.set_tcp_target(target, quat, link=link, robot=ROBOT)
            if not ik.converged:
                short = ik.pos_error
                continue
            q = unwind(list(scene.joint_positions_of(ROBOT)), seed)
            q = full(q, fingers)
            scene.set_joint_positions(q, robot=ROBOT)
            found = hits()
            if not found:
                return q
            fouled = found
        where = tuple(round(v, 3) for v in target)
        if short is not None and not fouled:
            raise RuntimeError(f"{ARM} cannot reach {where}: {short * 1e3:.0f} mm short")
        raise RuntimeError(f"every branch at {where} fouls: {', '.join(fouled)}")

    def joint_move(motion: str, q: list) -> list:
        scene.add_segment(motion, goal=q, robot=ROBOT)
        return q

    def line(motion: str, q: list) -> list:
        scene.add_segment(motion, goal=q, kind="cartesian_line", robot=ROBOT)
        return q

    ready = full(READY)
    scene.set_joint_positions(ready, robot=ROBOT)
    if found := hits():
        raise RuntimeError(f"the park pose fouls: {', '.join(found)}")
    scene.add_segment("home", goal=ready, robot=ROBOT)

    # -- the cover, by the gripper: pads on the boss, gripper down ------------
    (cx, cy, cz), _ = scene.obstacle_pose(joint.b)
    # RG6 fingertips travel in an arc. Place their actual contact band on
    # the boss, compensating the catalog's fixed TCP at this jaw width.
    grip_z = cz + COVER[2] + BOSS[1] - GRIP + tooling.grip_tcp_offset(robot, BOSS[0])
    (jx, jy, jz), _ = joint.seat
    grasp = None
    failure = None
    for spin in (math.pi / 2, -math.pi / 2, 0.0, math.pi):
        try:
            q_grip = down(spin)
            approach = solve((cx, cy, grip_z + HOVER), q_grip, ready)
            grasp = solve((cx, cy, grip_z), q_grip, approach, strict=True)
            carry = solve((jx, jy, grip_z - cz + jz + HOVER), q_grip, approach, ready)
            seat = solve((jx, jy, grip_z - cz + jz), q_grip, carry, strict=True)
            break
        except RuntimeError as err:
            failure = err
            grasp = None
    if grasp is None:
        raise RuntimeError(f"no wrist spin carries the cover from its stock to the housing: {failure}")
    scene.set_joint_positions(grasp, robot=ROBOT)
    close = scene.grasp_close(joint.b, robot=ROBOT, joints=[finger])
    joint_move("cover_approach", approach)
    line("cover_down", grasp)
    line("cover_up", approach)
    joint_move("cover_carry", carry)
    line("cover_seat", seat)
    line("cover_clear", carry)

    # -- the screws, by the driver: the tip's +Z up the hole's normal ---------
    (px, py, pz), _ = scene.frame(feeder.pick)
    head = joint.fastener.length_m + joint.fastener.head_m - 0.001   # the bit a millimetre into the socket
    solved = False
    failure = None
    for spin in (math.pi / 2, math.pi, 0.0, -math.pi / 2):
        up = zrot(spin)
        try:
            hover = solve((px, py, pz + head + HOVER), up, ready, link=TIP)
            pick = solve((px, py, pz + head), up, hover, link=TIP, strict=True)
            poses = {}
            for hole in joint.order:
                (fx, fy, fz), _ = joint.hole_pose(hole)
                if hole == joint.order[-1]:
                    fx += misalign_mm / 1e3
                # The engage move ends with the tip in the hole, by the
                # driver's engage depth; the stroke runs the rest.
                engage_z = fz - driver.engage_m
                over = solve((fx, fy, engage_z + head + CLEAR), up, hover, ready, link=TIP)
                engage = solve((fx, fy, engage_z + head), up, over, link=TIP, strict=True)
                poses[hole] = (over, engage)
            solved = True
            break
        except RuntimeError as err:
            failure = err
    if not solved:
        raise RuntimeError(f"no wrist spin drives every screw: {failure}")
    joint_move("screw_to_pick", hover)
    line("screw_pick", pick)
    line("screw_lift", hover)
    for hole, (over, engage) in poses.items():
        joint_move(f"screw_over_{hole}", over)
        line(f"screw_engage_{hole}", engage)
        line(f"screw_clear_{hole}", over)
    scene.set_joint_positions(ready, robot=ROBOT)
    return close


# ---------------------------------------------------------------- program
def program(scene: bt.Scene, joint: A.Joint, driver: A.Driver, feeder: bt.parts.ScrewFeeder, close: dict, *,
            touch: list[str]):
    """The robot's program: the cover fitted (allowed to meet `touch` —
    the housing and its dowels), then the screws in the joint's order,
    then home — with the driver's `finish` raised at the end so its
    program ends with the cell's."""
    S = bt.seq
    finger = scene.robot_of(ROBOT).joint_names[-1]
    sq = scene.sequence("assemble")
    placement = A.place(sq, joint.b, motions={
        "approach": "cover_approach", "down": "cover_down", "up": "cover_up",
        "carry": "cover_carry", "seat": "cover_seat", "clear": "cover_clear",
    }, close=close, open={finger: 0.0}, expected=joint.seat, touch=touch, robot=ROBOT)
    fastening = A.fasten(sq, joint, driver, feeder, motions={
        "to_pick": "screw_to_pick", "pick": "screw_pick", "lift": "screw_lift",
        "over": "screw_over_{hole}", "engage": "screw_engage_{hole}", "clear": "screw_clear_{hole}",
    }, retry=RETRY, robot=ROBOT, after=[placement.seated])
    sq.step("home", actions=[S.motion("home"), S.set_signal(driver.signal("finish"))])
    return placement, fastening


# ------------------------------------------------------------------- bake
def bake(*, length_mm: int = SCREW_LENGTH_MM, rpm: float = RPM_RUN, misalign_mm: float = 0.0,
         catalog: bool = False, catalog_root=None):
    scene, joint, driver, feeder, placement, fastening = build(
        length_mm=length_mm, rpm=rpm, misalign_mm=misalign_mm, catalog=catalog, catalog_root=catalog_root)
    tl = scene.simulate_sequences(["assemble", driver.program], max_duration=240.0)
    return scene, tl, joint, driver, feeder, placement, fastening


# ------------------------------------------------------------- hand-over
def deliver(scene: bt.Scene, tl: bt.SequenceTimeline, fastening: A.Fastening, placement: A.Placement,
            out: Path):
    """The document set, written into `out` from the one source, and the
    report that hashes it. Returns `(report, runs, rows)`."""
    out.mkdir(parents=True, exist_ok=True)
    files: list[Path] = []

    def write(name: str, fn) -> Path:
        path = out / name
        fn(path)
        files.append(path)
        return path

    rows = A.fastening_report(tl, fastening)
    fit = A.fit_report(tl, placement)
    write("cover_bolting.botrail", scene.save_project)
    write("cover_bolting.py", lambda p: p.write_text(scene.generate_python()))
    write("cover_bolting_bom.csv", scene.export_bom)
    write("cover_bolting_bom.md", scene.export_bom)
    write("cover_bolting_io.csv", scene.export_io_list)
    write("cover_bolting_topology.mmd", scene.export_topology)
    write("cover_bolting_handshake.md", tl.export_handshake_spec)
    write("cover_bolting.plcopen.xml", lambda p: scene.export_plcopen(p, name="cover bolting"))
    write("cover_bolting_interlocks.md", scene.export_interlocks)
    write("cover_bolting_layout.svg", lambda p: scene.export_layout(p, scale=120, title="cover bolting"))
    write("cover_bolting_layout.dxf", lambda p: scene.export_layout(p, title="cover bolting"))
    write("cover_bolting_tightening.md", lambda p: A.export_sheet(p, fastening, rows=rows, title="cover bolting"))
    write("cover_bolting_tightening.csv", lambda p: A.export_sheet(p, fastening, rows=rows))
    write("cover_bolting_fastening.json", lambda p: p.write_text(json.dumps({"fit": fit, "screws": rows}, indent=2)))
    runs = scene.simulate_scenarios(["assemble", fastening.driver.program], max_duration=tl.duration + 30.0)
    report = scene.cell_report({"baseline": tl}, scenarios=runs, deliverables=files, title="cover bolting",
                               sections=[A.report_section(fastening, rows, fit)])
    report.save(out / "cover_bolting_report.md")
    report.save(out / "cover_bolting_report.json")
    return report, runs, rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default=str(HERE / "cover_bolting.usdc"))
    parser.add_argument("--length", type=int, default=SCREW_LENGTH_MM, help="screw length under the head, mm")
    parser.add_argument("--rpm", type=float, default=RPM_RUN, help="the driver's rundown speed")
    parser.add_argument("--misalign", type=float, default=0.0,
                        help="teach the last screw this many mm off its hole (the bake refuses it)")
    parser.add_argument("--catalog", action="store_true",
                        help=f"order the presenter, screws and workpiece set from their packs "
                             f"({FEEDER_CATALOG}, {SCREW_CATALOG}, {WORKPIECE_CATALOG})")
    parser.add_argument("--catalog-root", default=None, help="read the packs from a local build: DIR/<id>")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    try:
        scene, tl, joint, driver, feeder, placement, fastening = bake(
            length_mm=args.length, rpm=args.rpm, misalign_mm=args.misalign,
            catalog=args.catalog or bool(args.catalog_root), catalog_root=args.catalog_root)
    except ValueError as err:
        raise SystemExit(f"refused: {err}")
    print(f"cover bolting{' / catalog' if args.catalog or args.catalog_root else ''}: "
          f"cycle {tl.duration:.2f}s, {len(fastening.pairs)} screws in the order {joint.order}")
    for name, t0, t1 in tl.step_spans:
        if name.startswith("assemble/") and not name.startswith("assemble/judge"):
            print(f"  {name:<32} {t0:7.2f} - {t1:7.2f}s")
    for lane in (feeder.present, driver.signal("run"), driver.signal("ok"), driver.signal("nok")):
        spans = ", ".join(f"{a:.2f}-{b:.2f}" for a, b in tl.signal(lane).high_spans())
        print(f"  {lane:<28} on: {spans or '-'}")
    rows = A.fastening_report(tl, fastening)
    print("fastening report:")
    for r in rows:
        checks = " ".join(f"{k}={v}" for k, v in r["checks"].items())
        print(f"  #{r['order']} {r['hole']} {r['screw']}: off {r['offset_mm']:.2f} mm, tilt {r['tilt_deg']:.2f}°, "
              f"down {r['seated_mm']:.1f} mm in {r['drive_s']:.2f}s, {r['result']} ({r['attempts']}) — {checks}")
    fit = A.fit_report(tl, placement)
    print(f"cover fit: off {fit['offset_mm']:.2f} mm, {fit['height_mm']:+.2f} mm high, tilt {fit['tilt_deg']:.2f}° — "
          + " ".join(f"{k}={v}" for k, v in fit["checks"].items()))
    per_screw = (tl.step_span("assemble/clear5").end - tl.step_span("assemble/feed0").start) / len(rows)
    print(f"per screw: {per_screw:.1f}s (driver {driver.t_cycle_s:.2f}s of it at {driver.rpm_run:g} rpm)")
    clearance = tl.min_clearance()
    pair = f" ({clearance.pair[0]} x {clearance.pair[1]})" if clearance.pair else ""
    print(f"min clearance over the cycle: {float(clearance) * 1e3:.1f} mm at {clearance.t:.2f}s{pair}")

    warnings = tl.export_usd(args.out, fps=60)
    print(f"wrote {args.out}" + (f" ({warnings})" if warnings else ""))
    print(scene.bom().to_markdown())
    out_dir = Path(args.out).with_name("cover_bolting_deliverables")
    report, runs, _rows = deliver(scene, tl, fastening, placement, out_dir)
    print(scene.requirements().to_markdown())
    table = scene.interlocks([driver.program])
    print(f"interlock table: {len(table)} rows for `{driver.program}` (in the document set)")
    for name in ("baseline", *scene.scenario_names):
        verdict = runs.errors.get(name)
        print(f"  scenario {name:<16} {'ok' if verdict is None else 'refused — ' + verdict}")
    print(f"wrote the document set to {out_dir}/ ({len(report.deliverables)} files hashed in the report)")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
