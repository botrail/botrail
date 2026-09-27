"""A small-parts assembly line on a linear-motor loop, rebuilt from a
picture of a commercial simulator's product page.

The reference picture (500 x 285, kept outside the repository) looks along
one line from its right-hand end: a long transport loop runs across the
bottom of the frame, red carriers on its top run and one coming back
underneath; two yellow six-axis robots hang from a light-grey portal over
it, three yellow SCARAs with black bases and black hose arches stand on
white pedestals behind it, jig pallets wait on conveyors at the right, and
another line fills the background. The camera and the layout are read off
the picture (two vanishing points give the lens and the heading, the
robots' own sizes the scale — see `.internal/design-shuttle-line.md`).
Commercial equipment uses these products; fixtures and adapters are SI designs:

  * an ATS SuperTrak GEN3 set up over-under — seven 1000 mm straight
    sections between two 180-degree 800 mm sections, 12 shuttles, each
    with an integrator's red carrier plate and nest; a shuttle is a robot
    of two joints (along the straight, round an end) so it runs the whole
    loop, upside down along the bottom;
  * two FANUC LR Mate 200iD hanging from the portal, each with an SMC
    MHZ2-20D and SI-built fingers — S1 loads housings out of a jig pallet, S5 takes the finished
    modules off into a tray (catalog r3: FANUC's official ROS 2 model,
    with detailed meshes, surface normals and part materials);
  * three FANUC SR-3iA (the catalog's reference model) on pedestals, each
    with station-specific tooling — S2 sets a PCB with PFYN 6 cups, S3 a
    cover with ZP3 cups, S4 a connector with an MPG-plus 25, each
    off a presenter that calls up the next part as one is taken;
  * Makitech Type34-S1 belts for the jig pallets, FANUC R-30iB Mate Plus /
    Compact Plus controllers, the SuperTrak control panel.

The shuttles run the loop under zone control, the way the SuperTrak
controller does it: a shuttle moves into the next stop only once the
block ahead reads clear, stops at a station until the station says done,
and carries the module on from there. What runs is one lap from the
picture's moment — every station busy, the loop as the picture has it —
round to the same places: twelve jobs at each station, each module on its
shuttle from S1 to S5, and every handshake a signal.

Run with:  python examples/assembly/shuttle_line_demo.py [out.usdc] [--studio]
                        [--catalog-root DIR] [--handler CATALOG_ID]

`--catalog-root` points at a catalog builder's `build/` directory instead
of the published catalog; what the local build lacks still comes from the
published one.
FANUC's official ROS 2 model (r3) is the default. Use
`--handler fanuc/lrmate200id/lrmate200id/r2` to select the previous model.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import _shuttle_line_equipment as eq
import _shuttle_line_tooling as tooling
import botrail as bt

# ---- what is ordered -------------------------------------------------
HANDLER = "fanuc/lrmate200id/lrmate200id/r3"   # FANUC LR Mate 200iD, official ROS 2 model, ceiling mounted
GRIPPER = tooling.HOUSING_GRIPPER
SCARA = "fanuc/sr3ia/sr-3ia/r1"             # FANUC SR-3iA

BELT = "makitech/belgotch/type34-s1/r2"     # the jig pallet lanes
CONTROLLER = "fanuc/r-30ib-plus/cabinet/r1"  # R-30iB Mate Plus (LR Mate), Compact Plus (SR-3iA)
CATALOG_ROOT: Path | None = None

# ---- the layout, read off the picture (metres) ------------------------
TOP = 1.10                                  # the upper sections' outer face
SECTIONS = 7                                # 7 x 1000 mm straight sections
MOUNT_Z = 2.28                              # the handlers' mounting face, under the frame
R1_XY, R2_XY = (-0.30, 0.22), (-2.93, 0.12)
SCARA_Y, SCARA_TOP = 0.30, 1.36             # SR-3iA pedestals behind the loop
SCARA_X = {"s2": -1.29, "s3": -1.85, "s4": -2.45}
DECK_TOP = 1.00
NEXT_Y = 3.60                               # the next line's loop, behind this one
LANE_TOP = 1.30                             # jig pallet lanes: belt top (Type34-S1, H 1300)
LANES = {"in": -0.02, "out": 0.44}          # x of the two lanes, running along y
LANE_Y = (0.28, 2.78)
FEED_TOP = 1.24                             # the SCARAs' presenters: nest floor
OUT_XY = (-3.20, 0.41)                      # the outfeed tray behind R2, on the side away from S4
OUT_TOP = 1.32                              # its stand
FRAME = {"x_right": 0.55, "x_left": -7.65, "bays": [-1.20, -2.95, -4.70, -6.45],
         "y_front": -0.60, "y_back": 0.95, "height": MOUNT_Z + 0.02 + 0.15}
# The top stops (distance from the right end along the top): the entry
# over the right end, S1 under the loader, S2..S4 at the SCARAs, S5 under
# the unloader, buffers between, the empty run to the left end.
STOPS = {"entry": 0.0, "s1": 0.27, "b1": 0.76, "s2": 1.24, "s3": 1.66, "b2": 2.10, "s4": 2.54, "b3": 2.88,
         "s5": 3.22, "t1": 4.18, "t2": 5.22, "t3": 6.26, "t4": 6.62}
BOTTOM = [6.20, 5.45, 4.70, 3.80, 2.90, 2.00, 1.10, 0.30]    # the lower run's stops (s)
# where the picture has them: 8 on top, and 4 underneath
AT_START = ["entry", "s1", "s2", "s3", "s4", "s5", "t1", "t3", "r0", "r2", "r4", "r6"]
STATIONS = ["s1", "s2", "s3", "s4", "s5"]
GRIP_DOWN = (0.70710678, 0.70710678, 0.0, 0.0)   # tool +Z down, the fingers across the line (y)
OPEN, CLOSED = tooling.OPEN, tooling.CLOSED  # custom fingers: 50 mm open, 40 mm closed
HOVER = 0.10
SHUTTLE_SPEED = 1.5                         # m/s peak, between stops (SuperTrak: 4 m/s max)

# The picture's camera: vanishing points -> f = 329 px on 500 x 285
# (46.8 deg vertical), the eye 1.77 m up (see .internal/shuttle-line/cam.py).
PIC_EYE, PIC_LOOK, PIC_FOV = (0.695, -2.092, 1.765), (-2.381, 1.829, 1.362), 46.84
# The studio's own lens is 45 deg on a narrower canvas: the same view from
# half a metre further back, so the right-hand end stays in the frame.
STUDIO_VIEW = ((1.25, -2.80, 1.84), (-2.381, 1.829, 1.362))



def load(pid: str) -> bt.Robot:
    """From a local catalog build when one is given and has the product,
    else from the published catalog."""
    if CATALOG_ROOT and (CATALOG_ROOT / pid).is_dir():
        return bt.Robot.from_package(CATALOG_ROOT / pid, catalog_root=CATALOG_ROOT)
    return bt.Robot.from_catalog(pid)


def scara(station: str) -> bt.Robot:
    """An SR-3iA with the tool selected for this station."""
    arm = load(SCARA)
    if station == "s4":
        return tooling.parallel_hand(arm, load, connector=True)
    return tooling.vacuum_hand(arm, load, station)


def handler() -> bt.Robot:
    """An LR Mate with a pneumatic gripper and housing-specific fingers."""
    return tooling.parallel_hand(load(HANDLER), load)


def building(scene: bt.Scene) -> None:
    """The hall: a blue epoxy floor and the white walls the picture's
    background fades into. (Kept within a few bays of the line: the
    studio sizes its key light to the cell, and far walls would flatten
    it onto the floor.)"""
    slab = scene.add_box("floor/slab", (34.0, 20.0, 0.05), (-3.0, 3.5, -0.025), color=(0.030, 0.062, 0.110))
    scene.set_obstacle_enabled(slab, False)
    scene.set_obstacle_material(slab, metalness=0.0, roughness=0.55)
    for tag, size, pos in (("wall_back", (34.0, 0.3, 6.0), (-3.0, 8.5, 3.0)),
                           ("wall_left", (0.3, 20.0, 6.0), (-12.0, 3.5, 3.0))):
        wall = scene.add_box(f"hall/{tag}", size, pos, color=(1.0, 1.0, 1.0))
        scene.set_obstacle_enabled(wall, False)
        scene.set_obstacle_material(wall, metalness=0.0, roughness=0.95)


def supply_side(scene: bt.Scene, loop) -> dict:
    """The jig pallet lanes behind S1 (full pallets in, empty ones out), the
    SCARAs' presenters, the outfeed tray behind S5, and the signal tower."""
    length = LANE_Y[1] - LANE_Y[0]
    for tag, x in LANES.items():
        bt.parts.conveyor(scene, f"lane_{tag}", catalog=ref(BELT), length=length, width=0.30,
                          position=(x, (LANE_Y[0] + LANE_Y[1]) / 2, LANE_TOP),
                          direction=(0.0, -1.0) if tag == "in" else (0.0, 1.0), speed=0.2)
    # 3 x 4 nests: the pallet at the pick end holds the next eleven housings
    # (the twelfth has just gone into the nest at S1), one full pallet
    # queues behind it, an empty one goes out
    seats, _ = eq.jig_pallet(scene, "jig/pick", (LANES["in"], 0.50, LANE_TOP), nests=(3, 4), pitch=(0.105, 0.062))
    queued, _ = eq.jig_pallet(scene, "jig/queued", (LANES["in"], 0.98, LANE_TOP), nests=(3, 4), pitch=(0.105, 0.062))
    for k, (x, y, z) in enumerate(queued):
        eq.part(scene, f"parts/queued{k}", "housing", (x, y, z + eq.GAP))
    eq.jig_pallet(scene, "jig/empty", (LANES["out"], 0.75, LANE_TOP), nests=(3, 4), pitch=(0.105, 0.062))
    feeders = {}
    for tag, x in SCARA_X.items():
        kind = {"s2": "pcb", "s3": "cover", "s4": "connector"}[tag]
        pick = eq.presenter(scene, f"feeder_{tag}", (x + 0.28, 0.40, DECK_TOP), size=(0.12, 0.26), top=FEED_TOP)
        feeders[tag] = (kind, pick)
    bt.parts.table(scene, "outfeed_stand", (0.44, 0.34, OUT_TOP - DECK_TOP), (*OUT_XY, DECK_TOP))
    out = bt.parts.tray(scene, "outfeed", (0.40, 0.30, 0.03), (*OUT_XY, OUT_TOP))
    eq.signal_tower(scene, "tower", (-1.20, FRAME["y_back"], FRAME["height"]))
    # the robot controllers: the handlers' Mate cabinets on the floor behind the line, the
    # SCARAs' Compact Plus units on the deck behind each pedestal
    for robot, x in (("r1", -0.35), ("r2", -3.10)):
        bt.parts.controller(scene, f"{robot}_ctrl", [robot], (x, FRAME["y_back"] + 0.40, 0.0), catalog=ref(CONTROLLER),
                            variant="mate", mount="floor", yaw=math.pi)
    for tag, x in SCARA_X.items():
        bt.parts.controller(scene, f"sr_{tag}_ctrl", [f"sr_{tag}"], (x, 0.72, DECK_TOP), catalog=ref(CONTROLLER),
                            variant="compact", mount="cabinet", yaw=math.pi)
    return {"seats": seats, "feeders": feeders, "outfeed": out}


def ref(pid: str):
    return CATALOG_ROOT / pid if CATALOG_ROOT and (CATALOG_ROOT / pid).is_dir() else pid


def build() -> tuple[bt.Scene, dict]:
    # each handler's base turned toward its work: the nest at its stop and the pallet or tray behind
    yaw = {"r1": work_yaw(R1_XY, [(-STOPS["s1"], 0.0), (LANES["in"] - 0.11, 0.40), (LANES["in"] + 0.11, 0.40),
                                  (LANES["in"] - 0.11, 0.60), (LANES["in"] + 0.11, 0.60)]),
           "r2": work_yaw(R2_XY, [(-STOPS["s5"], 0.0), (OUT_XY[0] - 0.11, OUT_XY[1] - 0.1),
                                  (OUT_XY[0] + 0.11, OUT_XY[1] + 0.1)])}
    scene = bt.Scene(handler(), name="r1", base_position=(*R1_XY, MOUNT_Z), base_quaternion=hanging(yaw["r1"]))
    scene.add_robot(handler(), name="r2", base_position=(*R2_XY, MOUNT_Z), base_quaternion=hanging(yaw["r2"]))
    facing = (0.0, 0.0, -math.sin(math.pi / 4), math.cos(math.pi / 4))   # an SR-3iA's +X toward the loop
    for tag, x in SCARA_X.items():
        scene.add_robot(scara(tag), name=f"sr_{tag}", base_position=(x, SCARA_Y, SCARA_TOP), base_quaternion=facing)
    building(scene)
    loop = eq.supertrak(scene, "trak", SECTIONS, (0.0, 0.0), top=TOP)
    eq.add_shuttles(scene, loop, len(AT_START), panel=(0.45, FRAME["y_back"] + 0.55, 0.0))
    stops = eq.loop_stops(loop, STOPS, BOTTOM)
    index = {st.tag: i for i, st in enumerate(stops)}
    for k, tag in enumerate(AT_START):
        scene.set_joint_positions(list(eq.path_q(loop, stops[index[tag]].p)), robot=loop.shuttles[k])
    blocks = eq.loop_sensors(scene, loop, stops)
    for tag in STATIONS:
        eq.stop_sensor(scene, loop, f"at_{tag}", STOPS[tag])
    frame = eq.portal_frame(scene, "frame", **FRAME)
    for tag, (x, y) in (("r1", R1_XY), ("r2", R2_XY)):
        eq.mount_beam(scene, f"frame/mount_{tag}", x, y, underside=MOUNT_Z + 0.02, length=1.9)
    eq.station_deck(scene, "deck", -0.30, -7.3, eq.ST_DEPTH / 2 + 0.05, FRAME["y_back"] - 0.10, DECK_TOP)
    for tag, x in SCARA_X.items():
        eq.scara_pedestal(scene, f"pedestal_{tag}", x, SCARA_Y + 0.03, floor=DECK_TOP, top=SCARA_TOP)
    eq.floor_frame(scene, "floor_frame", 0.45, -7.55, -0.40, 0.90)
    supply = supply_side(scene, loop)
    tooling.vacuum_services(scene, load, SCARA_X, SCARA_TOP, DECK_TOP)
    # the next line over, standing still, as the picture's background has it
    eq.neighbour_line(scene, "line2", SECTIONS, (0.0, NEXT_Y), top=TOP,
                      frame={**FRAME, "y_front": NEXT_Y - 0.60, "y_back": NEXT_Y + 0.95},
                      carriers=[0.0, 0.27, 1.24, 1.66, 2.54, 3.22, 4.18, 6.26])
    return scene, {"loop": loop, "frame": frame, "supply": supply, "stops": stops, "index": index,
                   "blocks": blocks, "yaw": yaw}


# ================================================================ the schedule
def schedule(info: dict) -> dict:
    """Who carries what, when: every shuttle goes once round (as many stops
    as the loop has, so the line ends as it began); each station serves the
    shuttles in the order they reach it, the one standing there first. A
    shuttle's product is the one S1 loaded into it; the four on their way
    at the start are -1 (at S2) .. -4 (at S5)."""
    stops, index = info["stops"], info["index"]
    n = len(stops)
    start = {k: index[tag] for k, tag in enumerate(AT_START)}
    order = {}
    for st in STATIONS:
        i_st = index[st]
        # arrivals within the lap: distance (in stops) from each shuttle's start
        dist = {k: (i_st - i0) % n for k, i0 in start.items()}
        order[st] = sorted(dist, key=lambda k: dist[k])
    carried = {k: None for k in start}
    for j, tag in enumerate(("s2", "s3", "s4", "s5")):
        carried[AT_START.index(tag)] = -(j + 1)
    loads = {k: i for i, k in enumerate(order["s1"])}          # product = S1 job index
    jobs = {st: [] for st in STATIONS}
    for st in STATIONS:
        for k in order[st]:
            before_s1 = st != "s1" and ((index[st] - start[k]) % n) < ((index["s1"] - start[k]) % n or n)
            jobs[st].append(carried[k] if (before_s1 and carried[k] is not None) else loads[k])
    return {"start": start, "order": order, "loads": loads, "carried": carried, "jobs": jobs}


def pname(kind: str, product: int) -> str:
    return f"parts/{kind}_{'m' if product < 0 else ''}{abs(product)}"


ADDS = {"s1": ["housing"], "s2": ["pcb"], "s3": ["cover"], "s4": ["connector"]}


def payload(product, station: str) -> list:
    """What a shuttle carries away from `station` with `product` on it."""
    if product is None or station == "s5":
        return []
    upto = STATIONS.index(station)
    return [pname(kind, product) for st in STATIONS[:upto + 1] for kind in ADDS.get(st, [])]


def shuttle_programs(scene: bt.Scene, info: dict, sch: dict) -> list:
    """One program per shuttle, once round: into the next stop when its
    block reads clear; at a station, let go of what it carries, wait for
    the station's `done_<stop>`, take up the product as it now is."""
    loop, stops, blocks = info["loop"], info["stops"], info["blocks"]
    n = len(stops)
    exists = set(scene.obstacle_names)
    for st in STATIONS:
        scene.define_signal(f"done_{st}", initial=False)
    seqs = []
    for k, name in enumerate(loop.shuttles):
        prog = f"shuttle{k:02d}"
        sq = scene.sequence(prog)
        i0 = sch["start"][k]
        p = stops[i0].p
        product = sch["carried"][k]
        held = []

        def serve(tag: str, step: str, product, held, *, k=k, sq=sq, name=name):
            if tag == "s1":
                product = sch["loads"][k]
            if held:
                sq.step(f"{step}_let_go", actions=[bt.seq.detach(o) for o in held])
            sq.step(f"{step}_wait", transition=bt.seq.signal(f"done_{tag}"))
            held = [o for o in payload(product, tag) if o in exists]
            if held:
                sq.step(f"{step}_take", actions=[bt.seq.attach(o, link="body", robot=name) for o in held])
            return product, held

        if stops[i0].tag in STATIONS:
            product, held = serve(stops[i0].tag, "a0", product, [])
        for a in range(1, n + 1):
            i = (i0 + a) % n
            p_next = p + ((stops[i].p - p) % loop.lap)
            sq.step(f"a{a}_clear", transition=bt.seq.all_of(*[bt.seq.signal(b, False) for b in blocks[i]]))
            for m, ramp in enumerate(eq.path_moves(loop, name, p, p_next, SHUTTLE_SPEED)):
                sq.step(f"a{a}_{stops[i].tag}_{m}", actions=[ramp])
            p = p_next
            if a < n and stops[i].tag in STATIONS:
                product, held = serve(stops[i].tag, f"a{a}", product, held)
        seqs.append(prog)
    return seqs


# ================================================================ the goods
def nest_seat(loop, s: float) -> tuple:
    return (loop.x(s), loop.y0, TOP + eq.NEST_TOP)


def stack(scene: bt.Scene, product: int, seat, upto: str) -> list:
    """Product `product` as it stands after station `upto`, on `seat`
    (x, y, z of the surface under it). Returns its parts, bottom first."""
    x, y, z = seat
    names = []
    for st in STATIONS[:STATIONS.index(upto) + 1]:
        for kind in ADDS.get(st, []):
            z += eq.GAP
            names.append(eq.part(scene, pname(kind, product), kind, (x, y, z)))
            z += {"housing": eq.HOUSING, "pcb": eq.PCB, "cover": eq.COVER_PART, "connector": eq.CONNECTOR}[kind][2]
    return names


SIZES = {"housing": eq.HOUSING, "pcb": eq.PCB, "cover": eq.COVER_PART, "connector": eq.CONNECTOR}
KIND = {"s2": "pcb", "s3": "cover", "s4": "connector"}


def goods(scene: bt.Scene, info: dict, sch: dict) -> dict:
    """The products at the picture's moment. Every station has just set its
    part down on the shuttle in front of it (product 0's housing at S1,
    -1 .. -3 at S2 .. S4 as far as they have got), the finished -4 waits at
    S5; housings 1 .. 11 sit in the jig pallet; each presenter shows its
    next part and holds the rest in its magazine (a source)."""
    loop = info["loop"]
    for st in ("s1", "s2", "s3", "s4"):
        stack(scene, sch["jobs"][st][0], nest_seat(loop, STOPS[st]), st)
    stack(scene, sch["jobs"]["s5"][0], nest_seat(loop, STOPS["s5"]), "s4")
    seats = info["supply"]["seats"]
    for n in range(1, 12):
        x, y, z = seats[n]
        eq.part(scene, pname("housing", n), "housing", (x, y, z + eq.GAP))
    picks = {}
    for st, kind in KIND.items():
        _, (fx, fy, fz) = info["supply"]["feeders"][st]
        h = SIZES[kind][2]
        jobs = sch["jobs"][st]
        eq.part(scene, pname(kind, jobs[1]), kind, (fx, fy, fz + eq.GAP))
        park = (fx, fy + 0.3, -0.5)
        pool = []
        for i, product in enumerate(jobs[2:]):
            made = eq.part(scene, pname(kind, product), kind, (park[0], park[1], park[2] - 0.05 * i - h / 2))
            pool.append(made)
        scene.add_source(f"src_{st}", pool=pool, park=park, pitch=(0.0, 0.0, -0.05),
                         position=(fx, fy, fz + eq.GAP + h / 2), interval=0.0, running=False)
        picks[st] = (fx, fy, fz + eq.GAP + h)
    return {"picks": picks}


# ================================================================ the handlers
LR_SPEEDS = [math.radians(v) for v in (450, 380, 520, 550, 545, 1000)]   # LR Mate 200iD axis speeds
SWING = 0.45                                # the swings between hover poses, at this share of them


def grip_point(bottom_z: float, x: float, y: float) -> tuple:
    """Custom-finger TCP at the middle of the housing side faces."""
    return (x, y, bottom_z + eq.HOUSING[2] / 2)


def work_yaw(axis, points) -> float:
    """The heading a hanging handler's base is turned to so the work it
    reaches lies either side of J1 = 0: the middle of the smallest arc (seen
    from its axis) holding every point. A handler set the other way round
    would reach half its work across J1's stop and swing the long way."""
    ang = sorted(math.atan2(y - axis[1], x - axis[0]) for x, y, *_ in points)
    gaps = [((ang[(i + 1) % len(ang)] - a) % (2 * math.pi), i) for i, a in enumerate(ang)]
    gap, i = max(gaps)
    return math.atan2(math.sin(ang[(i + 1) % len(ang)] + (2 * math.pi - gap) / 2),
                      math.cos(ang[(i + 1) % len(ang)] + (2 * math.pi - gap) / 2))


def hanging(yaw: float) -> tuple:
    """Base orientation of a ceiling-mounted handler: half a turn about x,
    then `yaw` about the vertical."""
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)


def teach(scene: bt.Scene, robot: str, poses: dict, yaw: float) -> dict:
    """Every pose a handler works from, solved against its own kinematics,
    each from a seed pointing the arm at it (J1 = heading - azimuth) with the
    wrist on the same side every time. The parallel hand is the same turned half
    round its axis, so J6 then takes whichever of +- k*180 deg lies nearest
    zero: a swing never spins the hand for nothing."""
    taught = {}
    lo, hi = scene.robot_of(robot).joint_limits[5]
    axis = scene.robot_base_pose_of(robot)[0]
    for name, xyz in poses.items():
        j1 = math.remainder(yaw - math.atan2(xyz[1] - axis[1], xyz[0] - axis[0]), 2 * math.pi)
        seed = [j1, math.radians(10), math.radians(-20), 0.0, math.radians(100), 0.0, OPEN]
        scene.set_joint_positions(seed, robot=robot)
        ik = scene.set_tcp_target(xyz, GRIP_DOWN, robot=robot)
        if not ik.converged:
            raise RuntimeError(f"{robot} cannot reach {name} at {tuple(round(v, 3) for v in xyz)}: "
                               f"{ik.pos_error * 1e3:.0f} mm short")
        q = list(scene.joint_positions_of(robot))
        q[5] = min((q[5] + k * math.pi for k in range(-4, 5) if lo <= q[5] + k * math.pi <= hi), key=abs)
        taught[name] = q
    return taught


def swept(scene: bt.Scene, robot: str, frm: list, to: list, samples: int = 16) -> None:
    """A swing is a joint ramp, and the rollout checks a ramp against the
    other machines only: check this one against the scenery here, pose by
    pose along the way, before anything runs."""
    for i in range(samples + 1):
        u = i / samples
        scene.set_joint_positions([a + (b - a) * u for a, b in zip(frm, to)], robot=robot)
        hits = [pair for pair in scene.check_collisions() if robot in (pair[0][0], pair[1][0])]
        if hits:
            raise RuntimeError(f"{robot}: the swing hits {hits[0]} at {u:.2f} of the way")
    scene.set_joint_positions(frm, robot=robot)


def straight(scene: bt.Scene, robot: str, frm: list, to: list, share: float = 0.5) -> dict:
    """A joint ramp at `share` of the axis speeds: the short vertical
    approach or retreat between a hover pose and the grip below it, and
    (checked by `swept`) the swing between two hover poses."""
    t = max(abs(b - a) / (share * v) for a, b, v in zip(frm[:6], to[:6], LR_SPEEDS)) + 0.15
    names = scene.robot_of(robot).joint_names[:6]
    return bt.seq.ramp(dict(zip(names, to[:6])), round(t, 3), robot=robot)


def fingers(robot: str, value: float, t: float = 0.3) -> dict:
    return bt.seq.ramp({"finger_joint": value}, t, robot=robot)


def loader(scene: bt.Scene, info: dict, sch: dict) -> str:
    """R1 at S1: product 0 has just gone into the nest at the start; then
    each next housing out of the jig pallet while the next shuttle comes,
    into the nest once it stands at S1."""
    loop = info["loop"]
    seats = info["supply"]["seats"]
    x, y, z = nest_seat(loop, STOPS["s1"])
    lo = grip_point(z + eq.GAP, x, y)
    poses = {"s1_lo": lo, "s1_hi": (lo[0], lo[1], lo[2] + HOVER), "rest": (lo[0], lo[1], lo[2] + HOVER + 0.15)}
    for n in range(1, 12):
        jx, jy, jz = seats[n]
        g = grip_point(jz + eq.GAP, jx, jy)
        poses[f"jig{n}_lo"], poses[f"jig{n}_hi"] = g, (g[0], g[1], g[2] + HOVER)
    q = teach(scene, "r1", poses, info["yaw"]["r1"])
    for n in range(1, 12):
        swept(scene, "r1", q["s1_hi"], q[f"jig{n}_hi"])
    swept(scene, "r1", q["s1_hi"], q["rest"])
    # at the picture's moment: product 0 just set down in the nest at S1
    scene.set_joint_positions(q["s1_lo"][:6] + [CLOSED], robot="r1")
    sq = scene.sequence("r1")
    for j, product in enumerate(sch["jobs"]["s1"]):
        h = pname("housing", product)
        if j > 0:
            sq.step(f"j{j}_to_jig", actions=[straight(scene, "r1", q["s1_hi"], q[f"jig{product}_hi"], SWING)])
            sq.step(f"j{j}_down", actions=[straight(scene, "r1", q[f"jig{product}_hi"], q[f"jig{product}_lo"])])
            sq.step(f"j{j}_close", actions=[fingers("r1", CLOSED)])
            sq.step(f"j{j}_grip", actions=[bt.seq.attach(h, robot="r1", touch_links=tooling.FINGER_CONTACTS)])
            sq.step(f"j{j}_lift", actions=[straight(scene, "r1", q[f"jig{product}_lo"], q[f"jig{product}_hi"])])
            sq.step(f"j{j}_await", transition=bt.seq.signal("at_s1"))
            sq.step(f"j{j}_over", actions=[straight(scene, "r1", q[f"jig{product}_hi"], q["s1_hi"], SWING)])
            sq.step(f"j{j}_down_nest", actions=[straight(scene, "r1", q["s1_hi"], q["s1_lo"])])
        sq.step(f"j{j}_release", actions=([bt.seq.detach(h)] if j > 0 else []) + [fingers("r1", OPEN)])
        sq.step(f"j{j}_clear", actions=[straight(scene, "r1", q["s1_lo"], q["s1_hi"])])
        sq.step(f"j{j}_done", actions=[bt.seq.set_signal("done_s1")], transition=bt.seq.signal("at_s1", False))
        sq.step(f"j{j}_reset", actions=[bt.seq.set_signal("done_s1", False)])
    sq.step("park", actions=[straight(scene, "r1", q["s1_hi"], q["rest"], SWING)])
    return "r1"


def unloader(scene: bt.Scene, info: dict, sch: dict, *, kinds=("housing", "pcb", "cover", "connector")) -> str:
    """R2 at S5: each finished module off its shuttle — the shuttle may go
    as soon as it is lifted clear — and into the outfeed tray, row by row."""
    loop = info["loop"]
    x, y, z = nest_seat(loop, STOPS["s5"])
    lo = grip_point(z + eq.GAP, x, y)
    poses = {"s5_lo": lo, "s5_hi": (lo[0], lo[1], lo[2] + HOVER), "rest": (lo[0], lo[1], lo[2] + HOVER + 0.15)}
    sx, sy, sz = scene.frame("outfeed/seat")[0]
    for j in range(len(sch["jobs"]["s5"])):
        r, c = divmod(j, 3)
        px, py = sx + (c - 1) * 0.105, sy + (r - 1.5) * 0.062
        g = grip_point(sz + eq.GAP, px, py)
        poses[f"out{j}_lo"], poses[f"out{j}_hi"] = g, (g[0], g[1], g[2] + HOVER)
    q = teach(scene, "r2", poses, info["yaw"]["r2"])
    for j in range(len(sch["jobs"]["s5"])):
        swept(scene, "r2", q["s5_hi"], q[f"out{j}_hi"])
    swept(scene, "r2", q["s5_hi"], q["rest"])
    scene.set_joint_positions(q["s5_hi"][:6] + [OPEN], robot="r2")
    sq = scene.sequence("r2")
    for j, product in enumerate(sch["jobs"]["s5"]):
        parts = [o for o in (pname(k, product) for k in kinds) if o in set(scene.obstacle_names)]
        sq.step(f"j{j}_await", transition=bt.seq.signal("at_s5"))
        if j > 0:
            sq.step(f"j{j}_over", actions=[straight(scene, "r2", q[f"out{j - 1}_hi"], q["s5_hi"], SWING)])
        sq.step(f"j{j}_down", actions=[straight(scene, "r2", q["s5_hi"], q["s5_lo"])])
        sq.step(f"j{j}_close", actions=[fingers("r2", CLOSED)])
        sq.step(f"j{j}_grip", actions=[bt.seq.attach(o, robot="r2", touch_links=tooling.FINGER_CONTACTS)
                                                                  for o in parts])
        sq.step(f"j{j}_lift", actions=[straight(scene, "r2", q["s5_lo"], q["s5_hi"])])
        # the shuttle may go now it is lifted clear; the next one must find
        # the handshake down again, so it drops the moment this one leaves
        sq.step(f"j{j}_done", actions=[bt.seq.set_signal("done_s5")], transition=bt.seq.signal("at_s5", False))
        sq.step(f"j{j}_reset", actions=[bt.seq.set_signal("done_s5", False)])
        sq.step(f"j{j}_to_out", actions=[straight(scene, "r2", q["s5_hi"], q[f"out{j}_hi"], SWING)])
        sq.step(f"j{j}_put", actions=[straight(scene, "r2", q[f"out{j}_hi"], q[f"out{j}_lo"])])
        sq.step(f"j{j}_release", actions=[bt.seq.detach(o) for o in parts] + [fingers("r2", OPEN)])
        sq.step(f"j{j}_back", actions=[straight(scene, "r2", q[f"out{j}_lo"], q[f"out{j}_hi"])])
    sq.step("park", actions=[straight(scene, "r2", q[f"out{len(sch['jobs']['s5']) - 1}_hi"], q["rest"], SWING)])
    return "r2"


SR_SPEEDS = [math.radians(720), math.radians(780), 1.8, math.radians(3000)]   # SR-3iA axis speeds
SR_DOWN = (1.0, 0.0, 0.0, 0.0)              # the cups face down, their rows along x


def assembler(scene: bt.Scene, info: dict, sch: dict, st: str) -> str:
    """An SR-3iA at S2 .. S4: its part off the presenter, onto the module on
    the shuttle in front of it, the next part called up as this one goes."""
    loop = info["loop"]
    robot = f"sr_{st}"
    kind = KIND[st]
    h = SIZES[kind][2]
    jobs = sch["jobs"][st]
    x, y, _ = nest_seat(loop, STOPS[st])
    # what the part is set on: the module as the station before left it
    below = sum(SIZES[k][2] + eq.GAP for s_ in STATIONS[:STATIONS.index(st)] for k in ADDS.get(s_, []))
    place = (x, y, TOP + eq.NEST_TOP + below + eq.GAP + h)
    pick = info["goods"]["picks"][st]
    mechanical = st == "s4"
    if mechanical:
        place = (place[0], place[1], place[2] - h / 2)
        pick = (pick[0], pick[1], pick[2] - h / 2)
    seed = [0.0, 0.0, 0.1, 0.0] + ([tooling.CONNECTOR_OPEN] if mechanical else [])
    q = {}
    for name, xyz in (("place", place), ("pick", pick)):
        scene.set_joint_positions(seed, robot=robot)
        ik = scene.set_tcp_target(xyz, SR_DOWN, robot=robot)
        if not ik.converged:
            raise RuntimeError(f"{robot} cannot reach its {name} at {tuple(round(v, 3) for v in xyz)}")
        q[name] = list(scene.joint_positions_of(robot))
    for name in ("place", "pick"):          # up: the shaft drawn in, clear of every module passing below
        q[f"{name}_hi"] = q[name][:2] + [0.0] + q[name][3:]

    def ramp(frm, to, share=0.5):
        t = max(abs(b - a) / (share * v) for a, b, v in zip(frm, to, SR_SPEEDS)) + 0.12
        return bt.seq.ramp(dict(zip(("J1", "J2", "J3", "J4"), to)), round(t, 3), robot=robot)

    initial = q["place"][:4] + ([CLOSED] if mechanical else [])
    scene.set_joint_positions(initial, robot=robot)
    sq = scene.sequence(robot)
    for j, product in enumerate(jobs):
        part_ = pname(kind, product)
        if j > 0:
            sq.step(f"j{j}_to_pick", actions=[ramp(q["place_hi"], q["pick_hi"])])
            sq.step(f"j{j}_down", actions=[ramp(q["pick_hi"], q["pick"])])
            if mechanical:
                sq.step(f"j{j}_close", actions=[fingers(robot, CLOSED, 0.12)])
            sq.step(f"j{j}_grip" if mechanical else f"j{j}_suck",
                    actions=[bt.seq.attach(part_, robot=robot,
                                           touch_links=tooling.FINGER_CONTACTS if mechanical else tooling.CUP_CONTACTS)],
                    transition=bt.seq.elapsed(0.05 if mechanical else 0.15))
            sq.step(f"j{j}_up", actions=[ramp(q["pick"], q["pick_hi"])]
                    + ([bt.seq.start(f"src_{st}")] if j + 1 < len(jobs) else []))
            sq.step(f"j{j}_await", transition=bt.seq.signal(f"at_{st}"))
            sq.step(f"j{j}_over", actions=[ramp(q["pick_hi"], q["place_hi"])])
            sq.step(f"j{j}_set", actions=[ramp(q["place_hi"], q["place"])])
            sq.step(f"j{j}_release" if mechanical else f"j{j}_blow",
                    actions=[bt.seq.detach(part_)], transition=bt.seq.elapsed(0.12))
        if mechanical:
            sq.step(f"j{j}_open", actions=[fingers(robot, tooling.CONNECTOR_OPEN, 0.12)])
        sq.step(f"j{j}_lift", actions=[ramp(q["place"], q["place_hi"])])
        sq.step(f"j{j}_done", actions=[bt.seq.set_signal(f"done_{st}")], transition=bt.seq.signal(f"at_{st}", False))
        sq.step(f"j{j}_reset", actions=[bt.seq.set_signal(f"done_{st}", False)])
    return robot


def programs(scene: bt.Scene, info: dict) -> list:
    """The line's programs: one per shuttle (the SuperTrak controller's
    moves), one per station robot."""
    sch = schedule(info)
    info["schedule"] = sch
    info["goods"] = goods(scene, info, sch)
    # These faces deliberately touch their own workpieces, including the taught
    # start pose before the first detach. Housings, adapters and other parts
    # retain normal collision checks.
    for robot, station, kind, links in (
        ("r1", "s1", "housing", tooling.FINGER_CONTACTS),
        ("r2", "s5", "housing", tooling.FINGER_CONTACTS),
        ("sr_s2", "s2", "pcb", tooling.CUP_CONTACTS),
        ("sr_s3", "s3", "cover", tooling.CUP_CONTACTS),
        ("sr_s4", "s4", "connector", tooling.FINGER_CONTACTS),
    ):
        for product in sch["jobs"][station]:
            for link in links:
                scene.allow_link_obstacle_contact(link, pname(kind, product), robot=robot)
    seqs = shuttle_programs(scene, info, sch)
    seqs.append(loader(scene, info, sch))
    for st in KIND:
        seqs.append(assembler(scene, info, sch, st))
    seqs.append(unloader(scene, info, sch))
    return seqs


def bake(max_duration: float = 150.0):
    scene, info = build()
    seqs = programs(scene, info)
    tl = scene.simulate_sequences(seqs, max_duration=max_duration)
    return scene, tl, info


def report(scene: bt.Scene, tl, info: dict) -> None:
    """What the run did, station by station, and what the line is made of."""
    sch = info["schedule"]
    util = tl.utilizations()
    spans = {name: (a, b) for name, a, b in tl.step_spans}
    print(f"shuttle line: {tl.duration:.1f} s, {len(info['loop'].shuttles)} shuttles once round the loop")
    labels = {"r1": "S1 loads housings", "sr_s2": "S2 sets PCBs", "sr_s3": "S3 sets covers",
              "sr_s4": "S4 sets connectors", "r2": "S5 unloads modules"}
    for robot, st in (("r1", "s1"), ("sr_s2", "s2"), ("sr_s3", "s3"), ("sr_s4", "s4"), ("r2", "s5")):
        done = sorted(a for n, (a, b) in spans.items() if n.startswith(f"{robot}/j") and n.endswith("_done"))
        takt = (done[-1] - done[0]) / (len(done) - 1) if len(done) > 1 else float("nan")
        print(f"  {labels[robot]:20s} {len(done):2d} jobs (products {sch['jobs'][st][0]} .. {sch['jobs'][st][-1]}), "
              f"every {takt:.2f} s, moving {100 * util.get(robot, 0):.0f} % of the time")
    moving = [100 * util.get(name, 0) for name in info["loop"].shuttles]
    print(f"  shuttles            moving {min(moving):.0f} - {max(moving):.0f} % of the time")
    bom = scene.bom()
    print(f"  bill of materials: {len(bom)} lines, {len(bom.unidentified())} unidentified")
    for row in bom.rows:
        if row.get("manufacturer"):
            print(f"    {row.get('qty', 1):>3} x {row['manufacturer']} {row.get('model', '')}"
                  f"  ({row.get('category', '')})")


def main() -> None:
    global CATALOG_ROOT, HANDLER
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", nargs="?", default=str(HERE / "shuttle_line.usdc"))
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--catalog-root", type=Path, default=None)
    parser.add_argument("--handler", default=HANDLER,
                        help="Catalog ID for the two LR Mate handlers (default: %(default)s)")
    args = parser.parse_args()
    CATALOG_ROOT = args.catalog_root
    HANDLER = args.handler
    scene, tl, info = bake()
    report(scene, tl, info)
    tl.export_usd(args.out, fps=30)
    print(f"wrote {args.out}")
    if args.studio:
        bt.studio(scene, view=STUDIO_VIEW)


if __name__ == "__main__":
    main()
