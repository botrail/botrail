"""An end-of-line palletizing line, rebuilt from a screenshot of a commercial
simulator's web viewer.

The reference picture (the viewer at 9.5 s of a 112 s run; kept outside the
repository) shows two yellow 4-axis palletizers working a pallet line:
cases come in along a front roller line and are turned into two lanes beside
the front robot; the back robot takes cases from belts on either side; empty
pallets drop out of a dispenser at the right, loaded ones run left through a
ring stretch wrapper to a turntable with a label printer. The camera, the
layout and the sizes are read off the picture (its three vanishing points
give the camera; a 0.5 m pallet-conveyor height gives the scale — see
`.internal/design-palletizing-line.md`), and every machine is a real one:

  * FANUC M-410iC/185 palletizers (185 kg, 3143 mm, 4 axes) from the catalog,
    each on a welded riser with an EOAT of two Schmalz FA-Xc 442 area bars;
  * Interroll MCP case conveyors — RM 8310 rollers, RM 8731 90-degree
    transfers, BM 8420 belts — and Interroll MPP pallet conveyors (PM 9710
    rollers, PM 9720 chains, PM 9735 turntable);
  * a Robopac Genesis Futura 40 rotary-ring wrapper with its top press,
    a PALOMAT Inline dispenser, a Logopak Series 700 labeller;
  * EPAL 2 pallets (1200 x 1000) and 400 x 300 x 250 cases.

What runs is one shift's worth of the picture: robot A builds two pallets
from its two lanes (the front line sends every other case into the second
lane); robot B finishes the pallet on its station from the belts either side;
the wrapper finishes the pallet in it and lets it go to the turntable, where
it is labelled, turned and sent out; the line then indexes — the pallet
waiting at B's first station into the wrapper, B's finished one after it —
and the dispenser drops an empty pallet on B's station for the next load.
Everything is interlocked by signals, the cases by photo-eyes, the pallets by
beams at each station, and the two arms are checked against each other on
every tick.

Run with:  python examples/palletizing/palletizing_line_demo.py [out.usdc] [--studio]
                        [--catalog-root DIR]

`--catalog-root` points at a catalog builder's `build/` directory instead
of the published catalog.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import botrail as bt  # noqa: E402
import _palletizing_equipment as eq  # noqa: E402

# ---- what is ordered -------------------------------------------------
PALLETIZER = "fanuc/m410ic/m410ic-185/r1"
CATALOG_ROOT: Path | None = None

# ---- the goods -------------------------------------------------------
PALLET = (1.2, 1.0, 0.162)            # EPAL 2
CASE = (0.40, 0.30, 0.25)             # RSC 400 x 300 x 250
CASE_KG = 9.0
SEAT = 0.004                          # what is set down stands this proud of what it stands on
LAYER = 9                             # 3 x 3 cases a layer
# between cases in a layer: a case stops on its photo-eye within a scan
# tick (5 mm at 0.5 m/s) of the taught pick, and is set down that far off
GAP = 0.015

# ---- the layout, read off the picture (metres) ------------------------
CASE_TOP = 0.75                       # case conveyors: roller tops
PAL_TOP = 0.50                        # pallet conveyors: roller tops
FRONT_Y = 0.0                         # the case infeed, flowing +x
F1 = (4.60, 7.44)                     # roller run: hood -> transfer 1
T1_X = 7.80
F2 = (8.16, 10.74)
T2_X = 11.10
LANE0 = 0.345                         # lanes: from the transfers' far edge ...
LANE_SPLIT = 1.75                     # ... a feed zone to here, then the pick zone ...
LANE_END = 2.35                       # ... to the pick end, flowing +y
A_XY, A_RISER = (9.45, 2.65), 1.10
B_XY, B_RISER = (10.35, 8.55), 1.00
MAGAZINE_XY = (7.40, 3.45)
STATION_Y = 4.68                      # robot A's pallet stations
SA1_X, SA2_X = 8.10, 10.90
LINE_Y = 6.30                         # the pallet line, flowing -x
B_BELT_Y = 9.25
B_LEFT_END, B_RIGHT_END = 9.10, 11.60  # where B's belts stop, either side of it
TURN_X = 0.05
WRAP_X = 4.40
SB1_X, SB2_X = 9.60, 11.10
DISP_X = 14.30
COLUMNS = ((0.6, 0.3), (0.6, 10.1), (14.9, 10.1), (14.9, 0.3))
COLUMN = 0.45
# The pallet line, right to left: sections abut so a pallet crossing from one
# zone to the next is always carried; a gap bridges the turntable's swing.
SECTIONS = (("chain_in", 15.20, 17.20), ("dispense", 13.40, 15.20), ("buffer", 11.85, 13.40),
            ("sb2", 10.35, 11.85), ("sb1", 8.85, 10.35), ("run", 5.50, 8.85), ("wrap", 3.30, 5.50),
            ("to_turn", 0.995, 3.30))
TURN_DECK = 0.69                      # half the turntable's conveyor length (1380)
TURN_MODULE = 0.945                   # half its module length (1890)

# ---- motion ------------------------------------------------------------
CASE_SPEED = 0.5                      # m/s, the case conveyors
PALLET_SPEED = 0.4                    # m/s, the pallet line (PM 9710: 0.1-0.5)
HOVER = 0.30                          # approach and retract above a pick or a place, at least ...
A_TRAVEL, B_TRAVEL = 1.90, 2.00       # ... this high: a case carried over the stacks beside the station
A_PICKS = 16                          # robot A: alternately lane 1 -> station A1, lane 2 -> A2
B_FINISH = 4                          # robot B: the cases that finish the pallet on B's station ...
B_NEW = 8                             # ... and the first ones on the next pallet

# The picture's camera: 3 vanishing points -> f = 433.7 px on a 797 x 496
# canvas (59.5 deg vertical), the eye 5.7 m up.
VC_EYE, VC_LOOK, VC_FOV = (13.0, -4.0, 5.7), (9.10, 3.60, 0.51), 59.5
# The studio's own lens is 45 deg: the same view from further back.
STUDIO_VIEW = ((14.7, -7.3, 7.9), (8.72, 4.35, 0.0))


def load(pid: str) -> bt.Robot:
    if CATALOG_ROOT:
        return bt.Robot.from_package(CATALOG_ROOT / pid, catalog_root=CATALOG_ROOT)
    return bt.Robot.from_catalog(pid)


def palletizer() -> bt.Robot:
    """The M-410iC/185 with its case gripper welded on the flange."""
    gripper = bt.Robot.from_urdf_string(eq.eoat_urdf("eoat"))
    return load(PALLETIZER).attach_tool(gripper, tcp="eoat_tcp")


def names_under(scene: bt.Scene, prefix: str) -> list:
    return [n for n in scene.obstacle_names if n.startswith(prefix + "/")]


# ================================================================ the building
def building(scene: bt.Scene) -> None:
    """The floor as the picture paints it — a light grey hall floor, the white
    zone the line stands in, a grey walkway band behind — and the four
    building columns of the bay."""
    def paint(name, x0, y0, x1, y1, color, z):
        made = scene.add_box(name, (x1 - x0, y1 - y0, 0.002), ((x0 + x1) / 2, (y0 + y1) / 2, z), color=color)
        scene.set_obstacle_enabled(made, False)
        scene.set_obstacle_material(made, metalness=0.0, roughness=0.9)

    # the hall floor runs out past the picture's top edge (1.3 deg below the horizon)
    slab = scene.add_box("floor/slab", (800.0, 800.0, 0.05), (8.0, 6.0, -0.025), color=(0.80, 0.80, 0.79))
    scene.set_obstacle_enabled(slab, False)
    scene.set_obstacle_material(slab, metalness=0.0, roughness=0.9)
    paint("floor/zone_line", -7.0, 4.0, 22.0, 11.2, (1.0, 1.0, 1.0), 0.001)
    paint("floor/walkway", -12.0, 11.2, 26.0, 14.2, (0.62, 0.62, 0.61), 0.001)
    paint("floor/zone_back", -12.0, 14.2, 26.0, 17.5, (1.0, 1.0, 1.0), 0.001)
    paint("floor/zone_front", 11.8, -3.5, 17.5, 4.0, (0.88, 0.88, 0.87), 0.0012)
    for i, (x, y) in enumerate(COLUMNS):
        scene.add_box(f"column/{i}", (COLUMN, COLUMN, 9.0), (x, y, 4.5), color=(0.55, 0.56, 0.57))
        scene.add_box(f"column/{i}/base", (0.70, 0.70, 0.03), (x, y, 0.015), color=(0.40, 0.41, 0.42))
    # the building is not a purchase: nothing is pinned on it


# ================================================================ the goods
def slot_offset(n: int) -> tuple[float, float, float]:
    """Where case `n` of a pallet goes, from the pallet's deck centre: a 3 x 3
    column stack, 400 along x and 300 along y, layers upward."""
    layer, slot = divmod(n, LAYER)
    i, j = divmod(slot, 3)
    return (i - 1) * (CASE[0] + GAP), (j - 1) * (CASE[1] + GAP), layer * (CASE[2] + SEAT)


def deck_z(top: float) -> float:
    """The deck of a pallet standing on a conveyor whose roller tops are `top`."""
    return top + 0.002 + PALLET[2]


def pallet_with_cases(scene: bt.Scene, name: str, xy, top: float, cases: int) -> list:
    """An EPAL 2 at (x, y) on rollers at `top`, with its first `cases` cases.
    Returns every name the load is made of — what a machine attaches to carry
    it — the boards first."""
    x, y = xy
    bt.parts.pallet(scene, name, (x, y, top + 0.002), size=PALLET, model="EPAL 2")
    scene.remove_part(name)
    names = names_under(scene, name)
    for n in range(cases):
        dx, dy, dz = slot_offset(n)
        names.append(case(scene, f"{name}/case{n}", (x + dx, y + dy, deck_z(top) + SEAT + dz)))
    return names


def case(scene: bt.Scene, name: str, bottom) -> str:
    return bt.parts.carton(scene, name, CASE, bottom, mass_kg=CASE_KG)


# ================================================================ the case lines
def case_lines(scene: bt.Scene) -> dict:
    """The front infeed with its two 90-degree transfers into robot A's
    lanes, and robot B's belts from the left and from the right — the
    conveyors, the photo-eyes at the pick ends and at the transfers, and
    the cases already on them."""
    black = (0.03, 0.03, 0.035)
    eq.case_roller(scene, "front/f1", F1[1] - F1[0], ((F1[0] + F1[1]) / 2, FRONT_Y), top=CASE_TOP,
                   roller_color=black, speed=CASE_SPEED, running=True)
    t1 = eq.transfer(scene, "front/t1", (T1_X, FRONT_Y), cross=(0.0, 1.0), top=CASE_TOP, speed=CASE_SPEED)
    eq.case_roller(scene, "front/f2", F2[1] - F2[0], ((F2[0] + F2[1]) / 2, FRONT_Y), top=CASE_TOP,
                   roller_color=black, speed=CASE_SPEED, running=True)
    t2 = eq.transfer(scene, "front/t2", (T2_X, FRONT_Y), cross=(0.0, 1.0), top=CASE_TOP, speed=CASE_SPEED)
    # Each lane is two zero-pressure zones, the way an MDR lane accumulates:
    # a feed zone that holds a case at its end until the pick zone is free,
    # and the pick zone the robot takes from.
    for tag, x in (("lane1", T1_X), ("lane2", T2_X)):
        eq.case_roller(scene, f"front/{tag}", LANE_SPLIT - LANE0, (x, (LANE0 + LANE_SPLIT) / 2), direction=(0.0, 1.0),
                       top=CASE_TOP, speed=CASE_SPEED)
        eq.case_roller(scene, f"front/{tag}_pick", LANE_END - LANE_SPLIT, (x, (LANE_SPLIT + LANE_END) / 2),
                       direction=(0.0, 1.0), top=CASE_TOP, speed=CASE_SPEED)
        for side, s in (("l", -1), ("r", 1)):     # the black guards either side of the pick end
            g = scene.add_box(f"front/{tag}/guard_{side}", (0.006, 0.55, 0.30),
                              (x + s * (eq.MCP_BF / 2 + 0.06), LANE_END - 0.30, CASE_TOP + 0.15), color=eq.DARK)
            scene.set_obstacle_enabled(g, False)
    hood = scene.add_box("front/hood", (0.45, 0.80, 1.05), (F1[0] - 0.2, FRONT_Y, 0.525), color=(0.45, 0.33, 0.21))
    scene.set_obstacle_material(hood, metalness=0.0, roughness=0.8)

    # robot B's infeeds: a belt each side of it, fed by roller runs from off the picture
    eq.case_roller(scene, "back/left_run", 3.4, (B_LEFT_END - 2.6 - 1.7, B_BELT_Y), top=CASE_TOP, speed=CASE_SPEED,
                   running=True)
    eq.case_belt(scene, "back/left_belt", 2.6, (B_LEFT_END - 1.3, B_BELT_Y), top=CASE_TOP, speed=CASE_SPEED)
    eq.case_belt(scene, "back/right_belt", 2.6, (B_RIGHT_END + 1.3, B_BELT_Y), direction=(-1.0, 0.0),
                 top=CASE_TOP, speed=CASE_SPEED)
    eq.case_roller(scene, "back/right_run", 4.2, (B_RIGHT_END + 2.6 + 2.1, B_BELT_Y), direction=(-1.0, 0.0),
                   top=CASE_TOP, speed=CASE_SPEED, running=True)

    # ---- the cases, in the order they will be picked --------------------
    zc = CASE_TOP + SEAT                       # a case's bottom on the rollers
    lane = {1: [], 2: []}
    # A lane holds three at most — one at the pick end, one at the end of the
    # feed zone, one just turned in — so it starts with one, and the front
    # line brings the rest.
    for i, x in ((1, T1_X), (2, T2_X)):
        lane[i].append(case(scene, f"cases/lane{i}_0", (x, LANE_END - 0.175, zc)))
    arriving = []                                                       # on the front line, lead first
    arriving_f2 = case(scene, "cases/f2_0", (F2[0] + 1.4, FRONT_Y, zc))
    for k, x in enumerate((F1[1] - 0.45, F1[1] - 1.35, F1[1] - 2.25)):
        arriving.append(case(scene, f"cases/f1_{k}", (x, FRONT_Y, zc)))
    # the source: the rest arrive through the hood, one per case robot A takes
    need_l1 = A_PICKS // 2 - len(lane[1])
    need_l2 = A_PICKS - A_PICKS // 2 - len(lane[2]) - 1
    through_t1 = max(2 * need_l1 - 1, 2 * need_l2)        # even arrivals to lane 1, odd to lane 2
    # A source's members wait on their parking slots (park + pitch * i, the
    # box's centre): a member anywhere else counts as already out on the line.
    pool = [case(scene, f"cases/front{k}", (F1[0] - 0.2, FRONT_Y, -1.0 - 0.3 * k - CASE[2] / 2))
            for k in range(through_t1 - len(arriving))]
    scene.add_source("src_front", pool=pool, park=(F1[0] - 0.2, FRONT_Y, -1.0), pitch=(0.0, 0.0, -0.3),
                     position=(F1[0] + 0.25, FRONT_Y, zc + CASE[2] / 2), interval=0.0, running=False)
    at_t1 = arriving + pool
    for k, name in enumerate(at_t1):
        (lane[1] if k % 2 == 0 else lane[2]).append(name)
    lane[2].insert(len([n for n in lane[2] if n.startswith("cases/lane")]), arriving_f2)

    b_left = [case(scene, f"cases/bl{k}", (B_LEFT_END - 0.22 - 0.55 * k, B_BELT_Y, zc)) for k in range(3)]
    b_right = [case(scene, f"cases/br{k}", (B_RIGHT_END + 0.22 + 0.55 * k, B_BELT_Y, zc)) for k in range(3)]
    per_side = (B_FINISH + B_NEW + 1) // 2
    pool_l = [case(scene, f"cases/left{k}", (B_LEFT_END - 5.9, B_BELT_Y, -1.0 - 0.3 * k - CASE[2] / 2))
              for k in range(per_side - 3)]
    pool_r = [case(scene, f"cases/right{k}", (B_RIGHT_END + 6.7, B_BELT_Y, -1.0 - 0.3 * k - CASE[2] / 2))
              for k in range(per_side - 3)]
    scene.add_source("src_left", pool=pool_l, park=(B_LEFT_END - 5.9, B_BELT_Y, -1.0), pitch=(0.0, 0.0, -0.3),
                     position=(B_LEFT_END - 5.9, B_BELT_Y, zc + CASE[2] / 2), interval=0.0, running=False)
    scene.add_source("src_right", pool=pool_r, park=(B_RIGHT_END + 6.7, B_BELT_Y, -1.0), pitch=(0.0, 0.0, -0.3),
                     position=(B_RIGHT_END + 6.7, B_BELT_Y, zc + CASE[2] / 2), interval=0.0, running=False)

    # ---- photo-eyes: across the case, at mid-height ---------------------
    zb = zc + CASE[2] / 2
    everything = list(lane[1]) + list(lane[2]) + b_left + pool_l + b_right + pool_r
    front = [n for n in everything if n.startswith("cases/")]
    for tag, x in (("t1", T1_X), ("t2", T2_X)):        # a case centred on the transfer
        scene.add_beam_sensor(f"eye_{tag}", frm=(x + CASE[0] / 2 + 0.006, FRONT_Y - 0.25, zb),
                              to=(x + CASE[0] / 2 + 0.006, FRONT_Y + 0.25, zb), watch=front)
    for i, x in ((1, T1_X), (2, T2_X)):                # a case at each zone's end
        scene.add_beam_sensor(f"eye_lane{i}", frm=(x - 0.4, LANE_END - 0.02, zb), to=(x + 0.4, LANE_END - 0.02, zb),
                              watch=front)
        scene.add_beam_sensor(f"eye_feed{i}", frm=(x - 0.4, LANE_SPLIT - 0.02, zb),
                              to=(x + 0.4, LANE_SPLIT - 0.02, zb), watch=front)
    scene.add_beam_sensor("eye_left", frm=(B_LEFT_END - 0.02, B_BELT_Y - 0.4, zb),
                          to=(B_LEFT_END - 0.02, B_BELT_Y + 0.4, zb), watch=front)
    scene.add_beam_sensor("eye_right", frm=(B_RIGHT_END + 0.02, B_BELT_Y - 0.4, zb),
                          to=(B_RIGHT_END + 0.02, B_BELT_Y + 0.4, zb), watch=front)
    for n in ("eye_t1", "eye_t2", "eye_lane1", "eye_lane2", "eye_feed1", "eye_feed2", "eye_left", "eye_right"):
        scene.set_part(n, manufacturer="SICK", model="WL12G-3B2531", category="sensor.photoelectric")
    lefts = b_left + pool_l
    rights = b_right + pool_r
    return {"t1": t1, "t2": t2, "lane": lane, "at_t1": at_t1, "left": lefts, "right": rights}


# ================================================================ the pallet line
def pallet_line(scene: bt.Scene) -> dict:
    """The pallet line, right to left: the chain infeed at the far end, the
    dispenser section, a buffer, robot B's two stations, the run to the
    wrapper, the wrapper's own section, the run to the turntable, the
    turntable, and the exit toward the back. Robot A's two stations stand in
    front of it."""
    parts = {}
    for name, x0, x1 in SECTIONS:
        pos = ((x0 + x1) / 2, LINE_Y)
        if name == "chain_in":
            eq.pallet_chain(scene, f"line/{name}", x1 - x0, pos, top=PAL_TOP, speed=PALLET_SPEED)
        else:
            zone = PALLET[2] - 0.007 if name == "dispense" else 1.9   # under the stack: the bottom pallet only
            eq.pallet_roller(scene, f"line/{name}", x1 - x0, pos, top=PAL_TOP, zone_height=zone, speed=PALLET_SPEED)
    parts["turntable"] = eq.turntable(scene, "turntable", (TURN_X, LINE_Y), top=PAL_TOP, speed=PALLET_SPEED)
    # the swing clearance either side of the deck: carried across all the same
    zone_h = 1.9
    scene.add_conveyor("line/gap_a", zone_position=((TURN_X + TURN_DECK + 0.995) / 2, LINE_Y, PAL_TOP + zone_h / 2),
                       zone_size=(0.995 - TURN_X - TURN_DECK, eq.MPP_CW, zone_h),
                       velocity=(-PALLET_SPEED, 0.0, 0.0), running=False)
    exit0 = LINE_Y + TURN_MODULE
    scene.add_conveyor("line/gap_b", zone_position=(TURN_X, (LINE_Y + TURN_DECK + exit0) / 2, PAL_TOP + zone_h / 2),
                       zone_size=(eq.MPP_CW, exit0 - LINE_Y - TURN_DECK, zone_h),
                       velocity=(0.0, PALLET_SPEED, 0.0), running=False)
    eq.pallet_chain(scene, "line/exit", 2.4, (TURN_X, exit0 + 1.2), direction=(0.0, 1.0), top=PAL_TOP,
                    speed=PALLET_SPEED)
    for n in ("line/gap_a", "line/gap_b"):   # the swing clearance is part of the turntable module
        scene.set_part(n, manufacturer="Interroll", model="PM 9735 module length (included)", category="conveyor.zone")
    # robot A's stations: modules in front of the line, pallet guides round them
    for tag, x in (("sa1", SA1_X), ("sa2", SA2_X)):
        eq.pallet_roller(scene, f"station/{tag}", 1.5, (x, STATION_Y), direction=(1.0, 0.0), top=PAL_TOP)
        for side, s in (("f", -1), ("b", 1)):
            g = scene.add_box(f"station/{tag}/guide_{side}", (1.36, 0.05, 0.12),
                              (x, STATION_Y + s * (PALLET[1] / 2 + 0.06), PAL_TOP + 0.14), color=eq.RAL_1023)
            scene.set_obstacle_enabled(g, False)
        for side, s in (("l", -1), ("r", 1)):
            g = scene.add_box(f"station/{tag}/stop_{side}", (0.05, 1.10, 0.12),
                              (x + s * (PALLET[0] / 2 + 0.08), STATION_Y, PAL_TOP + 0.14), color=eq.RAL_1023)
            scene.set_obstacle_enabled(g, False)
    parts["wrapper"] = eq.wrapper(scene, "wrapper", (WRAP_X, LINE_Y), top=deck_z(PAL_TOP), bands=5, pool=2,
                                  load_top=deck_z(PAL_TOP) + 4 * (CASE[2] + SEAT) + SEAT)
    parts["dispenser"] = eq.dispenser(scene, "dispenser", (DISP_X, LINE_Y), top=PAL_TOP, pallets=7, pallet=PALLET)
    parts["labeller"] = eq.labeller(scene, "labeller", (TURN_X, LINE_Y - 1.30), facing=(0.0, 1.0), stroke=0.42)
    parts["magazine"] = eq.sheet_magazine(scene, "magazine", MAGAZINE_XY, pallet=PALLET)
    # the label the labeller puts on a pallet's face: a source of one
    label = scene.add_box("labels/l0", (0.21, 0.002, 0.15), (TURN_X, LINE_Y, -2.0), color=(0.92, 0.92, 0.90))
    scene.set_obstacle_enabled(label, False)
    scene.add_source("src_label", pool=[label], park=(TURN_X, LINE_Y, -2.0),
                     position=(TURN_X, LINE_Y - PALLET[1] / 2 - 0.018, deck_z(PAL_TOP) + 0.7), interval=0.0,
                     running=False)
    parts["label"] = label

    parts["exit0"] = exit0
    return parts


def goods(scene: bt.Scene, parts: dict) -> dict:
    """The line at the picture's moment: a full pallet in the wrapper, a
    finished one waiting at B's first station and one nearly finished at its
    second, a part-built pallet at each of robot A's stations, a stack in the
    dispenser, a full pallet on the far chain."""
    loads = {
        "wrap": pallet_with_cases(scene, "load/wrap", (WRAP_X, LINE_Y), PAL_TOP, 4 * LAYER),
        "sb1": pallet_with_cases(scene, "load/sb1", (SB1_X, LINE_Y), PAL_TOP, 3 * LAYER),
        "sb2": pallet_with_cases(scene, "load/sb2", (SB2_X, LINE_Y), PAL_TOP, 4 * LAYER - B_FINISH),
        "sa1": pallet_with_cases(scene, "load/sa1", (SA1_X, STATION_Y), PAL_TOP, 2 * LAYER + 6),
        "sa2": pallet_with_cases(scene, "load/sa2", (SA2_X, STATION_Y), PAL_TOP, LAYER + 4),
    }
    bt.parts.unit_load(scene, "load/outbound", (16.25, LINE_Y, PAL_TOP), pallet=PALLET, height=1.30, collide=False)
    # ---- beams across the pallet line, through the deck boards: they run
    # the pallet's length unbroken (at block height a beam would blink
    # through the fork openings). What they see: every pallet that travels.
    boards = [n for key in ("wrap", "sb1", "sb2") for n in loads[key] if "/case" not in n]
    for pallet in parts["dispenser"].stack:
        boards += names_under(scene, pallet)
    zb = PAL_TOP + 0.002 + PALLET[2] - 0.011
    y0, y1 = LINE_Y - 0.45, LINE_Y + 0.45
    for name, x in (("eye_wrap", WRAP_X - PALLET[0] / 2 - 0.01), ("eye_turn", TURN_X - PALLET[0] / 2 - 0.01),
                    ("eye_sb1", SB1_X - PALLET[0] / 2 - 0.01), ("eye_sb2", SB2_X - PALLET[0] / 2 - 0.01)):
        scene.add_beam_sensor(name, frm=(x, y0, zb), to=(x, y1, zb), watch=boards)
    ye = parts["exit0"] + 2.4 - 0.05
    scene.add_beam_sensor("eye_exit", frm=(TURN_X - 0.45, ye, zb), to=(TURN_X + 0.45, ye, zb), watch=boards)
    for name in ("eye_wrap", "eye_turn", "eye_sb1", "eye_sb2", "eye_exit"):
        scene.set_part(name, manufacturer="SICK", model="WL12G-3B2531", category="sensor.photoelectric")
    return loads


# ================================================================ the robots
def teach(scene: bt.Scene, robot: str, poses: dict, seed: list) -> dict:
    """Every pose a palletizer works from, solved against its own
    kinematics: the gripper square to the case (tool +Z down, its bars along
    x), `HOVER` above for the approach. Each becomes a planned motion; the
    joint values are returned for the straight moves in and out."""
    down = (1.0, 0.0, 0.0, 0.0)
    lo, hi = scene.robot_of(robot).joint_limits[3]
    taught = {}
    for name, xyz in poses.items():
        scene.set_joint_positions(seed, robot=robot)
        ik = scene.set_tcp_target(xyz, down, robot=robot)
        if not ik.converged:
            raise RuntimeError(f"{robot} cannot reach {name} at {tuple(round(v, 3) for v in xyz)}: "
                               f"{ik.pos_error * 1e3:.0f} mm short")
        q = list(scene.joint_positions_of(robot))
        # The pad is square to the case either way round: take the wrist
        # angle a half turn from IK's when that is the shorter swing.
        q[3] = min((q[3] + n * math.pi for n in range(-4, 5) if lo <= q[3] + n * math.pi <= hi),
                   key=lambda v: abs(v - seed[3]))
        scene.add_segment(f"{robot}/{name}", goal=q, robot=robot)
        taught[name] = q
    scene.set_joint_positions(seed, robot=robot)
    return taught


SPEEDS = [math.radians(v) for v in (140, 140, 140, 305)]   # M-410iC/185 axis speeds


def straight(robot: str, frm: list, to: list, share: float = 0.7) -> dict:
    """The approach or retreat between a hover pose and the pick or place
    below it: a joint ramp at `share` of the axis speeds, 0.2 s to settle.
    Short and vertical, it needs no plan (and is not checked against the
    scenery it reaches into — that is what the plan above it is for)."""
    t = max(abs(b - a) / (share * v) for a, b, v in zip(frm, to, SPEEDS)) + 0.2
    return bt.seq.ramp(dict(zip(("J1", "J2", "J3", "J4"), to)), round(t, 3), robot=robot)


# Each robot is set on its riser turned to face its pallets, so its work
# fits J1's +-180 deg without crossing the stop: A faces +y (its lanes at
# about -106 / +106 deg, its stations at -36 / +34), B faces -y.
A_YAW, B_YAW = math.pi / 2, -math.pi / 2
A_READY = [0.0, math.radians(10), math.radians(-25), 0.0]
B_READY = [0.0, math.radians(10), math.radians(-25), 0.0]


def yaw_quat(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def robots(scene: bt.Scene, lines: dict) -> dict:
    """Risers under both robots, every pose they are taught, and the plan of
    which case each pick takes and where it goes."""
    eq.riser(scene, "riser_a", A_XY, A_RISER, floor_plate=2.0)
    eq.riser(scene, "riser_b", B_XY, B_RISER, color=(0.10, 0.10, 0.11), plate_color=(0.05, 0.05, 0.05),
             floor_plate=1.9)
    for robot, riser in (("robot_a", "riser_a"), ("robot_b", "riser_b")):
        # each robot stands on its riser: base plate on top plate, by design
        scene.allow_link_obstacle_contact("base_link", f"{riser}/top", robot=robot)
    scene.set_joint_positions(A_READY, robot="robot_a")
    scene.set_joint_positions(B_READY, robot="robot_b")
    grip = CASE_TOP + SEAT + CASE[2]                   # the case's top on the rollers

    poses_a = {}
    for i, x in ((1, T1_X), (2, T2_X)):
        poses_a[f"pick{i}_lo"] = (x, LANE_END - 0.17, grip)
        poses_a[f"pick{i}_hi"] = (x, LANE_END - 0.17, max(grip + HOVER, A_TRAVEL))
    plan_a = []
    count = {1: 2 * LAYER + 6, 2: LAYER + 4}
    for k in range(A_PICKS):
        i = 1 + k % 2
        sx = SA1_X if i == 1 else SA2_X
        n = count[i]
        count[i] += 1
        dx, dy, dz = slot_offset(n)
        top = deck_z(PAL_TOP) + SEAT + dz + CASE[2]
        poses_a[f"set{k}_lo"] = (sx + dx, STATION_Y + dy, top)
        poses_a[f"set{k}_hi"] = (sx + dx, STATION_Y + dy, max(top + HOVER, A_TRAVEL))
        plan_a.append((i, lines["lane"][i][k // 2], f"set{k}"))
    q_a = teach(scene, "robot_a", poses_a, A_READY)

    b_hi = max(grip + HOVER, B_TRAVEL)
    poses_b = {"pickL_lo": (B_LEFT_END - 0.22, B_BELT_Y, grip), "pickL_hi": (B_LEFT_END - 0.22, B_BELT_Y, b_hi),
               "pickR_lo": (B_RIGHT_END + 0.22, B_BELT_Y, grip), "pickR_hi": (B_RIGHT_END + 0.22, B_BELT_Y, b_hi)}
    plan_b = []
    for k in range(B_FINISH + B_NEW):
        side = "L" if k % 2 == 0 else "R"
        n = 4 * LAYER - B_FINISH + k if k < B_FINISH else k - B_FINISH
        dx, dy, dz = slot_offset(n)
        top = deck_z(PAL_TOP) + SEAT + dz + CASE[2]
        poses_b[f"set{k}_lo"] = (SB2_X + dx, LINE_Y + dy, top)
        poses_b[f"set{k}_hi"] = (SB2_X + dx, LINE_Y + dy, max(top + HOVER, B_TRAVEL))
        feed = lines["left"] if side == "L" else lines["right"]
        plan_b.append((side, feed[k // 2], f"set{k}"))
    q_b = teach(scene, "robot_b", poses_b, B_READY)
    for robot, ready in (("robot_a", A_READY), ("robot_b", B_READY)):
        scene.add_segment(f"{robot}/ready", goal=ready, robot=robot)
        # the gripper: two Schmalz FA-Xc bars on the integrator's frame
        scene.set_part(f"{robot}/tool", manufacturer="Schmalz", model="FA-Xc SVK 442 3R18 O20", qty=2,
                       category="gripper.vacuum", description="10.01.38.10029, on an integrator-built EOAT frame")
    return {"a": plan_a, "b": plan_b, "q_a": q_a, "q_b": q_b}


# ================================================================ the programs
def programs(scene: bt.Scene, lines: dict, parts: dict, loads: dict, plans: dict) -> list:
    """The PLC side of the line — one program per machine, talking through
    signals — and the two robot programs."""
    for sig in ("vac_a", "vac_b", "a_tick", "wrap_done", "wrap_go", "index_go", "at_turn", "turn_clear",
                "b_full", "b_pallet"):
        scene.define_signal(sig, initial=False)
    seqs = []
    t1, t2 = lines["t1"], lines["t2"]
    touch = ["eoat_mount"]

    # ---- the front line: every other case into lane 1, the rest to lane 2
    # A case turned into a lane: stop the rollers under it, lift the belts
    # until it is off the eye and clear of the deck, drop them, roll on.
    def turn(sq, tr, tag, k):
        sq.step(f"c{k}_turn", actions=[bt.seq.stop(tr.main), bt.seq.start(tr.cross)],
                transition=bt.seq.signal(f"eye_{tag}", False))
        sq.step(f"c{k}_off", transition=bt.seq.elapsed(0.3))
        sq.step(f"c{k}_drop", actions=[bt.seq.stop(tr.cross), bt.seq.start(tr.main)])

    tr = scene.sequence("transfer1")
    for k, name in enumerate(lines["at_t1"]):
        tr.step(f"c{k}_at", transition=bt.seq.signal("eye_t1"))
        if k % 2 == 0:
            turn(tr, t1, "t1", k)
        else:
            tr.step(f"c{k}_pass", transition=bt.seq.signal("eye_t1", False))
    seqs.append("transfer1")
    tr2 = scene.sequence("transfer2")
    for k in range(sum(1 for n in lines["lane"][2] if not n.startswith("cases/lane"))):
        tr2.step(f"c{k}_at", transition=bt.seq.signal("eye_t2"))
        turn(tr2, t2, "t2", k)
    seqs.append("transfer2")
    # the lanes' two zones: the pick zone runs until a case is at its end and
    # waits for the robot; the feed zone holds its lead case until the pick
    # zone's end is free, then lets it go
    for i in (1, 2):
        pk = scene.sequence(f"lane{i}")
        for k in range(len(lines["lane"][i])):
            pk.step(f"run{k}", actions=[bt.seq.start(f"front/lane{i}_pick")], transition=bt.seq.signal(f"eye_lane{i}"))
            pk.step(f"hold{k}", actions=[bt.seq.stop(f"front/lane{i}_pick")],
                    transition=bt.seq.signal(f"eye_lane{i}", False))
        seqs.append(f"lane{i}")
        fz = scene.sequence(f"feed{i}")
        for k in range(len(lines["lane"][i]) - 1):
            fz.step(f"run{k}", actions=[bt.seq.start(f"front/lane{i}")], transition=bt.seq.signal(f"eye_feed{i}"))
            fz.step(f"gate{k}", actions=[bt.seq.stop(f"front/lane{i}")],
                    transition=bt.seq.signal(f"eye_lane{i}", False))
            fz.step(f"leave{k}", actions=[bt.seq.start(f"front/lane{i}")],
                    transition=bt.seq.signal(f"eye_feed{i}", False))
        seqs.append(f"feed{i}")
    # the hood feeds one case for every one robot A takes
    pool = [n for n in lines["at_t1"] if n.startswith("cases/front")]
    fd = scene.sequence("hood") if pool else None
    for k in range(len(pool)):
        want = k % 2 == 0
        fd.step(f"wait{k}", transition=bt.seq.signal("a_tick", want))
        fd.step(f"feed{k}", actions=[bt.seq.start("src_front")])
    if pool:
        seqs.append("hood")
    # robot B's belts, the same way, with a source each
    for side, belt, eye, src in (("L", "back/left_belt", "eye_left", "src_left"),
                                 ("R", "back/right_belt", "eye_right", "src_right")):
        bl = scene.sequence(f"belt{side}")
        feed = lines["left"] if side == "L" else lines["right"]
        for k in range(len(feed)):
            bl.step(f"feed{k}", actions=[bt.seq.start(belt)], transition=bt.seq.signal(eye))
            bl.step(f"hold{k}", actions=[bt.seq.stop(belt)], transition=bt.seq.signal(eye, False))
            if k + 3 < len(feed):
                bl.step(f"more{k}", actions=[bt.seq.start(src)])
        seqs.append(f"belt{side}")

    # ---- robot A: lane 1 -> station A1, lane 2 -> station A2 -------------
    ra = scene.sequence("robot_a")
    qa = plans["q_a"]
    for k, (i, name, place) in enumerate(plans["a"]):
        ra.step(f"wait{k}", transition=bt.seq.signal(f"eye_lane{i}"))
        ra.step(f"over{k}", actions=[bt.seq.motion(f"robot_a/pick{i}_hi")])
        ra.step(f"down{k}", actions=[straight("robot_a", qa[f"pick{i}_hi"], qa[f"pick{i}_lo"])])
        ra.step(f"grip{k}", actions=[bt.seq.attach(name, touch_links=touch, robot="robot_a"),
                                     bt.seq.set_signal("vac_a")], transition=bt.seq.elapsed(0.25))
        ra.step(f"lift{k}", actions=[straight("robot_a", qa[f"pick{i}_lo"], qa[f"pick{i}_hi"]),
                                     bt.seq.set_signal("a_tick", k % 2 == 0)])
        ra.step(f"swing{k}", actions=[bt.seq.motion(f"robot_a/{place}_hi")])
        ra.step(f"place{k}", actions=[straight("robot_a", qa[f"{place}_hi"], qa[f"{place}_lo"])])
        ra.step(f"drop{k}", actions=[bt.seq.detach(name), bt.seq.set_signal("vac_a", False)],
                transition=bt.seq.elapsed(0.25))
        ra.step(f"clear{k}", actions=[straight("robot_a", qa[f"{place}_lo"], qa[f"{place}_hi"])])
    ra.step("park", actions=[bt.seq.motion("robot_a/ready")])
    seqs.append("robot_a")

    # ---- robot B: finish the pallet, wait for the next one, start it -----
    rb = scene.sequence("robot_b")
    qb = plans["q_b"]
    for k, (side, name, place) in enumerate(plans["b"]):
        if k == B_FINISH:
            rb.step("full", actions=[bt.seq.set_signal("b_full"), bt.seq.motion("robot_b/ready")])
            rb.step("await_pallet", transition=bt.seq.signal("b_pallet"))
        eye = "eye_left" if side == "L" else "eye_right"
        rb.step(f"wait{k}", transition=bt.seq.signal(eye))
        rb.step(f"over{k}", actions=[bt.seq.motion(f"robot_b/pick{side}_hi")])
        rb.step(f"down{k}", actions=[straight("robot_b", qb[f"pick{side}_hi"], qb[f"pick{side}_lo"])])
        rb.step(f"grip{k}", actions=[bt.seq.attach(name, touch_links=touch, robot="robot_b"),
                                     bt.seq.set_signal("vac_b")], transition=bt.seq.elapsed(0.25))
        rb.step(f"lift{k}", actions=[straight("robot_b", qb[f"pick{side}_lo"], qb[f"pick{side}_hi"])])
        rb.step(f"swing{k}", actions=[bt.seq.motion(f"robot_b/{place}_hi")])
        rb.step(f"place{k}", actions=[straight("robot_b", qb[f"{place}_hi"], qb[f"{place}_lo"])])
        rb.step(f"drop{k}", actions=[bt.seq.detach(name), bt.seq.set_signal("vac_b", False)],
                transition=bt.seq.elapsed(0.25))
        rb.step(f"clear{k}", actions=[straight("robot_b", qb[f"{place}_lo"], qb[f"{place}_hi"])])
    rb.step("park", actions=[bt.seq.motion("robot_b/ready")])
    seqs.append("robot_b")

    # ---- the wrapper: two loads -------------------------------------------
    w = parts["wrapper"]
    wr = scene.sequence("wrapper")
    load_top = deck_z(PAL_TOP) + 4 * (CASE[2] + SEAT)
    press_down = w.press_top - load_top - 0.03
    turn = 0.0
    for cycle in range(2):
        if cycle == 1:
            wr.step("await", transition=bt.seq.signal("wrap_go"))
            wr.step("busy", actions=[bt.seq.set_signal("wrap_done", False)])
        wr.step(f"press{cycle}", actions=[bt.seq.ramp({w.press: press_down}, 2.0, robot=w.name)])
        wr.step(f"lower{cycle}", actions=[bt.seq.ramp({w.lift: w.top - (deck_z(PAL_TOP) - 0.05)}, 2.5,
                                                      robot=w.name)])
        for k, (src, zc) in enumerate(w.film):
            turn += 2.2 * 2 * math.pi
            wr.step(f"band{cycle}_{k}", actions=[bt.seq.ramp({w.lift: w.top - zc, w.ring: turn}, 3.3, robot=w.name)])
            wr.step(f"film{cycle}_{k}", actions=[bt.seq.start(src)])
        turn += 2 * math.pi
        wr.step(f"up{cycle}", actions=[bt.seq.ramp({w.lift: 0.0, w.ring: turn}, 2.0, robot=w.name)])
        wr.step(f"release{cycle}", actions=[bt.seq.ramp({w.press: 0.0}, 1.5, robot=w.name)])
        wr.step(f"done{cycle}", actions=[bt.seq.set_signal("wrap_done", True)])
    seqs.append("wrapper")

    # ---- the pallet line, in two programs that own disjoint sections ----
    # `wrapline` owns the run to the wrapper, the wrapper's section and the
    # way to the turntable: the wrapped pallet goes out, the next comes in.
    tt = parts["turntable"]
    out = ["line/to_turn", "line/gap_a", tt.zone_a]
    wl = scene.sequence("wrapline")
    wl.step("await", transition=bt.seq.all_of(bt.seq.signal("wrap_done"), bt.seq.signal("b_full")))
    wl.step("index", actions=[bt.seq.start(n) for n in ["line/run", "line/wrap"] + out]
            + [bt.seq.set_signal("index_go")], transition=bt.seq.signal("eye_turn"))
    wl.step("at_turn", actions=[bt.seq.stop(n) for n in out] + [bt.seq.set_signal("at_turn")],
            transition=bt.seq.signal("eye_wrap", False))           # the wrapped pallet's tail is long gone
    wl.step("to_wrap", transition=bt.seq.signal("eye_wrap"))        # the next one's head
    wl.step("in_wrap", actions=[bt.seq.stop("line/run"), bt.seq.stop("line/wrap"), bt.seq.set_signal("wrap_go")])
    seqs.append("wrapline")
    # `stations` owns robot B's two stations, the buffer and the dispenser:
    # the pallet at the first station leaves (onto `wrapline`'s run), the
    # finished one moves up behind it, and an empty one drops in.
    d = parts["dispenser"]
    st = scene.sequence("stations")
    st.step("await", transition=bt.seq.signal("index_go"))
    st.step("index", actions=[bt.seq.start("line/sb1"), bt.seq.start("line/sb2")], transition=bt.seq.signal("eye_sb1"))
    st.step("sb1_leaving", transition=bt.seq.signal("eye_sb1", False))    # its pallet's tail passes the eye
    st.step("to_sb1", transition=bt.seq.signal("eye_sb1"))                 # the next one's head reaches it
    st.step("at_sb1", actions=[bt.seq.stop("line/sb1"), bt.seq.stop("line/sb2"), bt.seq.move_to(d.lift, "raised")],
            transition=bt.seq.device_done(d.lift))
    st.step("dispense", actions=[bt.seq.start("line/dispense"), bt.seq.start("line/buffer"), bt.seq.start("line/sb2")],
            transition=bt.seq.signal("eye_sb2"))
    st.step("at_sb2", actions=[bt.seq.stop("line/dispense"), bt.seq.stop("line/buffer"), bt.seq.stop("line/sb2"),
                               bt.seq.set_signal("b_pallet"), bt.seq.move_to(d.lift, "hold")],
            transition=bt.seq.device_done(d.lift))
    seqs.append("stations")

    # ---- the turntable: label, turn, send out, turn back -----------------
    carried = loads["wrap"] + [f"wrapper/film/b{k}_0" for k in range(len(w.film))] + [parts["label"]]
    tn = scene.sequence("turntable")
    tn.step("await", transition=bt.seq.signal("at_turn"))
    tn.step("apply", actions=[bt.seq.move_to(parts["labeller"], "out")],
            transition=bt.seq.device_done(parts["labeller"]))
    tn.step("label", actions=[bt.seq.start("src_label")], transition=bt.seq.elapsed(0.4))
    tn.step("retract", actions=[bt.seq.move_to(parts["labeller"], "home")],
            transition=bt.seq.device_done(parts["labeller"]))
    tn.step("hold", actions=[bt.seq.attach(n, link=tt.deck, robot=tt.name) for n in carried])
    tn.step("turn", actions=[bt.seq.ramp({tt.joint: math.pi / 2}, 4.0, robot=tt.name)])
    tn.step("let_go", actions=[bt.seq.detach(n) for n in carried])
    tn.step("send", actions=[bt.seq.start(tt.zone_b), bt.seq.start("line/gap_b"), bt.seq.start("line/exit")],
            transition=bt.seq.signal("eye_exit"))
    tn.step("out", actions=[bt.seq.stop(tt.zone_b), bt.seq.stop("line/gap_b"), bt.seq.stop("line/exit"),
                            bt.seq.set_signal("turn_clear")])
    tn.step("back", actions=[bt.seq.ramp({tt.joint: 0.0}, 4.0, robot=tt.name)])
    seqs.append("turntable")
    return seqs


# ================================================================ assembly
def build() -> tuple[bt.Scene, dict]:
    scene = bt.Scene(palletizer(), name="robot_a", base_position=(*A_XY, A_RISER), base_quaternion=yaw_quat(A_YAW))
    scene.add_robot(palletizer(), name="robot_b", base_position=(*B_XY, B_RISER), base_quaternion=yaw_quat(B_YAW))
    building(scene)
    lines = case_lines(scene)
    parts = pallet_line(scene)
    loads = goods(scene, parts)
    plans = robots(scene, lines)
    seqs = programs(scene, lines, parts, loads, plans)
    # The gripper's frame is wider than a case: set down beside its
    # neighbours, it rests on their tops as it lets go. That touch is the
    # job; the arm and the case it carries stay checked.
    cases = [n for n in scene.obstacle_names if n.startswith("cases/") or ("/case" in n and "/visual/" not in n)]
    for robot in ("robot_a", "robot_b"):
        for n in cases:
            scene.allow_link_obstacle_contact("eoat_mount", n, robot=robot)
    return scene, {"lines": lines, "parts": parts, "loads": loads, "plans": plans, "seqs": seqs}


def bake(max_duration: float = 180.0):
    scene, info = build()
    tl = scene.simulate_sequences(info["seqs"], max_duration=max_duration)
    return scene, tl, info


def report(scene: bt.Scene, tl, info: dict) -> None:
    """What the run did, machine by machine, and what the line is made of."""
    spans = {name: (a, b) for name, a, b in tl.step_spans}
    util = tl.utilizations()
    print(f"palletizing line: {tl.duration:.1f} s")
    print(f"  robot A  {A_PICKS} cases, lane 1 -> station A1 and lane 2 -> A2 alternately, "
          f"moving {100 * util.get('robot_a', 0):.0f} % of the time")
    print(f"  robot B  {B_FINISH} cases finish its pallet (full at {spans['robot_b/full'][0]:.1f} s), "
          f"{B_NEW} start the next, moving {100 * util.get('robot_b', 0):.0f} % of the time")
    for cycle in range(2):
        a, b = spans[f"wrapper/press{cycle}"][0], spans[f"wrapper/done{cycle}"][0]
        print(f"  wrapper  pallet {cycle + 1} wrapped {a:5.1f} - {b:5.1f} s ({b - a:.1f} s, 5 bands at 40 rpm)")
    print(f"  line     indexed at {spans['wrapline/index'][0]:.1f} s; the next pallet in the wrapper at "
          f"{spans['wrapline/in_wrap'][0]:.1f} s")
    print(f"  turntable labelled at {spans['turntable/label'][0]:.1f} s, turned, out on the exit at "
          f"{spans['turntable/out'][0]:.1f} s")
    print(f"  dispenser dropped a pallet onto B's station at {spans['stations/at_sb2'][0]:.1f} s")
    bom = scene.bom()
    print(f"  bill of materials: {len(bom)} lines, {len(bom.unidentified())} unidentified")
    for row in bom.rows:
        if row.get("manufacturer"):
            print(f"    {row.get('qty', 1):>3} x {row['manufacturer']} {row.get('model', '')}"
                  f"  ({row.get('category', '')})")


def main() -> None:
    global CATALOG_ROOT
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("out", nargs="?", default=str(HERE / "palletizing_line.usdc"))
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--catalog-root", type=Path, default=None)
    args = parser.parse_args()
    CATALOG_ROOT = args.catalog_root
    scene, tl, info = bake()
    report(scene, tl, info)
    # 20 fps: every one of the ~900 things that moves at all is sampled for
    # the whole run, and at 30 fps that is a 110 MB recording
    tl.export_usd(args.out, fps=20)
    print(f"wrote {args.out}")
    if args.studio:
        bt.studio(scene, view=STUDIO_VIEW)


if __name__ == "__main__":
    main()
