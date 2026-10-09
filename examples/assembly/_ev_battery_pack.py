"""The product side of the EV module-connection cell: the battery pack on its
workpiece carrier, the module connectors in their blister and the roller
line under them. Metres, z up, x along the line (the pack travels +x).

A six-module pack segment in an aluminium housing — two columns of modules
either side of a central channel, the HV terminals (KOSTAL KS 22 tab
headers, 37.9 x 15 mm, on insulating pads) at each module's inner end; in
the channel the coolant manifold on the floor and the BMS daisy-chain
harness in its carrier on brackets above it, a branch up to each module's
CMC; the battery disconnect unit (BDU) and the BMS master in the bay at
the upstream end. The modules run in series in a zig-zag across the
channel:

    L1(-) = pack -   L1+ -> R1-   R1+ -> L2-   L2+ -> R2-   R2+ -> L3-
    L3+ -> R3-       R3(+) = pack +

so the five module connectors never cross in plan. Each one is a 35 mm²
cable (12 mm over the jacket, 0.40 kg/m) with a KS 22 receptacle housing
(49.65 x 21.23 x 15 mm, its CPA on top) crimped at each end, the cable
leaving the housing's inner end; two lengths, 320 and 340 mm crimp to
crimp. They come in a black ESD blister on the carrier beside the pack,
lying straight in their grooves in the order they are fitted. The pack's
own terminals (L1-, R3+) keep their yellow transport caps: the BDU leads
are the next station's.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import botrail as bt

# ---------------------------------------------------------------- colours
# Linear RGB, the way botrail takes an obstacle's colour.
HV_ORANGE = (0.92, 0.17, 0.012)       # the cable jacket, RAL 2003-ish
KS22_ORANGE = (0.80, 0.19, 0.016)     # the connector housings
MODULE_GREY = (0.62, 0.63, 0.645)     # painted aluminium side plates
COVER_BLACK = (0.018, 0.019, 0.021)   # the cell-contacting cover on top
END_PLATE = (0.78, 0.79, 0.81)        # aluminium end plates and straps
HOUSING = (0.70, 0.71, 0.73)          # the pack housing's extrusions
HOUSING_FLOOR = (0.36, 0.37, 0.39)
PAD_WHITE = (0.80, 0.80, 0.78)        # the headers' insulating pads, labels
INK = (0.03, 0.03, 0.035)
LV_BLACK = (0.022, 0.022, 0.024)
CAP_YELLOW = (0.85, 0.58, 0.02)
CARRIER = (0.07, 0.085, 0.11)         # the workpiece carrier, painted steel
WEAR_STRIP = (0.62, 0.63, 0.64)
BLISTER = (0.03, 0.032, 0.036)        # black ESD thermoform

# ---------------------------------------------------------------- the line
CONV_Z = 0.650                        # roller tops
CARRIER_SIZE = (1.62, 1.10, 0.040)    # the workpiece carrier: x, y, thickness
CARRIER_X = 0.05                      # its centre on the stop
CARRIER_TOP = CONV_Z + CARRIER_SIZE[2]

# ---------------------------------------------------------------- the pack
PACK_X = (-0.640, 0.420)              # housing outside, along the line
PACK_Y = 0.500                        # half width
PACK_Z = CARRIER_TOP + 0.030          # on the carrier's locating blocks
FLOOR = 0.012                         # housing floor plate
COOLING = 0.008                       # the cooling plate the modules sit on
WALL = 0.025
WALL_TOP = 0.885
MODULE = (0.220, 0.400, 0.140)        # x (width), y (length), z
MODULE_Z = PACK_Z + FLOOR + COOLING   # module bottom
MT = MODULE_Z + MODULE[2]             # module top
ROWS = (-0.250, 0.0, 0.250)           # module centres along x
CHANNEL = 0.060                       # half width of the central channel
MODULE_Y = CHANNEL + MODULE[1] / 2    # module centre, |y|
TERM_DX = 0.055                       # a terminal's offset from its module's centre, x
TERM_Y = 0.095                        # the terminals' |y| (centre of the header)
PAD = (0.032, 0.052, 0.004)
HEADER = (0.015, 0.038, 0.024)        # the part of the tab header above the pad
HEADER_TOP = MT + PAD[2] + HEADER[2]
ENGAGE = 0.006                        # the receptacle's skirt over the header
RECEPT = (0.015, 0.0497, 0.0212)      # KS 22 receptacle: W, L, H
PLUG_Z = HEADER_TOP - ENGAGE + RECEPT[2] / 2   # a plugged receptacle's centre
CRIMP_IN = 0.005                      # the crimp sits this far inside the outer end
HELD = RECEPT[1] - CRIMP_IN           # cable held by the housing: crimp to exit
BAY_X = (-0.615, -0.400)              # the BDU bay inside the housing
# The channel's contents: the coolant manifold (supply and return) on the
# floor, the BMS daisy-chain harness in a carrier on brackets bridging it, a
# branch from the carrier up each module's inner face to its CMC.
PIPE_R = 0.011
PIPE_Y = 0.019                        # either side of the channel's middle
PIPE_Z = PACK_Z + FLOOR + 0.006 + PIPE_R
CARRIER_W = 0.036                     # the harness carrier: a U-profile
CARRIER_H = 0.018
CARRIER_Z = PIPE_Z + PIPE_R + 0.031   # the carrier's base, on brackets bridging the pipes
LV_R = 0.0065
LV_Z = CARRIER_Z + 0.003 + LV_R       # the trunk's axis, in the carrier
BRANCH_R = 0.0045
BRANCH_Y = CHANNEL - 0.008            # the branches run up the modules' inner faces
CHANNEL_X = (-0.385, 0.375)           # the run of the manifold and the carrier along the channel

# ---------------------------------------------------------------- the cable
CABLE_OD = 0.012
CABLE_R = CABLE_OD / 2
CABLE_KG_M = 0.40                     # 35 mm² copper and its jacket

# ---------------------------------------------------------------- the blister
KIT_X0, KIT_PITCH = 0.500, 0.045      # groove centres along x
KIT_Y_MIN = 0.210                     # half length, at least
KIT_FLOOR = CARRIER_TOP + 0.030       # the grooves' floor (on four stand-offs)
KIT_RIB = 0.012                       # rib height above the groove floor
GROOVE = 0.032                        # groove width: the housing and two fingers
KIT_PLUG_Z = KIT_FLOOR + RECEPT[2] / 2


@dataclass(frozen=True)
class Cable:
    """One module connector: which terminals it joins, how long it is
    (crimp to crimp), its groove in the blister."""

    name: str
    left: str        # terminal on the +y column (robot L's end)
    right: str       # terminal on the -y column (robot R's end)
    length: float
    slot: int
    part_number: str

    @property
    def kit_x(self) -> float:
        return KIT_X0 + KIT_PITCH * self.slot

    def kit(self, side: str) -> tuple[float, float, float]:
        """A housing's centre as the cable lies in the blister: `side` "l" at +y."""
        y = self.length / 2 + CRIMP_IN - RECEPT[1] / 2
        return (self.kit_x, y if side == "l" else -y, KIT_PLUG_Z)

    def plugged(self, side: str) -> tuple[float, float, float]:
        x, y = terminal_xy(self.left if side == "l" else self.right)
        return (x, y, PLUG_Z)

    def reference(self) -> list[tuple[float, float, float]]:
        """The rope's reference centreline: straight along y in its groove, at
        the housings' axis, from the crimp at the +y end to the one at -y."""
        half = self.length / 2
        return [(self.kit_x, half, KIT_PLUG_Z), (self.kit_x, -half, KIT_PLUG_Z)]


CABLES = (
    Cable("c1", "L1+", "R1-", 0.320, 0, "HV-MC-35-320"),
    Cable("c2", "L2-", "R1+", 0.340, 1, "HV-MC-35-340"),
    Cable("c3", "L2+", "R2-", 0.320, 2, "HV-MC-35-320"),
    Cable("c4", "L3-", "R2+", 0.340, 3, "HV-MC-35-340"),
    Cable("c5", "L3+", "R3-", 0.320, 4, "HV-MC-35-320"),
)
PACK_TERMINALS = ("L1-", "R3+")       # capped: the BDU leads are the next station's


def terminal_xy(terminal: str) -> tuple[float, float]:
    """`L2-`: the left column's second module, its negative terminal."""
    side, row, sign = terminal[0], int(terminal[1]) - 1, terminal[2]
    x = ROWS[row] + (TERM_DX if sign == "+" else -TERM_DX)
    return (x, TERM_Y if side == "L" else -TERM_Y)


def module_name(side: str, row: int) -> str:
    return f"pack/module_{side}{row + 1}"


def _box(scene, name, size, position, color, *, collide=True, metal=0.0, rough=0.6):
    made = scene.add_box(name, size=size, position=position, color=color)
    if not collide:
        scene.set_obstacle_enabled(made, False)
    scene.set_obstacle_material(made, metalness=metal, roughness=rough)
    return made


def _cyl(scene, name, radius, length, position, color, *, quaternion=None, collide=True,
         metal=0.0, rough=0.6):
    made = scene.add_cylinder(name, radius, length, position, quaternion=quaternion, color=color)
    if not collide:
        scene.set_obstacle_enabled(made, False)
    scene.set_obstacle_material(made, metalness=metal, roughness=rough)
    return made


def _along(axis: str) -> tuple[float, float, float, float]:
    """A cylinder (which stands along z) laid along x or y."""
    h = math.sqrt(0.5)
    return (0.0, h, 0.0, h) if axis == "x" else (h, 0.0, 0.0, h)


# ================================================================== the line
def build_line(scene, length: float = 4.4) -> None:
    """The roller line through the station and the workpiece carrier on its
    stop: a steel plate with wear strips, locating bushes and the pack's
    four locating blocks."""
    bt.parts.conveyor(scene, "line", length=length, width=1.16, position=(0.0, 0.0, CONV_Z),
                      rollers=(0.10, 0.060), stand_span=1.2, color=(0.30, 0.31, 0.33),
                      model="pallet roller line 1160, stop and lift-and-locate at the station")
    x, (lx, ly, t) = CARRIER_X, CARRIER_SIZE
    _box(scene, "carrier/plate", (lx, ly, t), (x, 0.0, CONV_Z + t / 2), CARRIER, metal=0.4, rough=0.5)
    for s in (-1, 1):
        _box(scene, f"carrier/strip_{'rl'[s > 0]}", (lx - 0.06, 0.06, 0.004),
             (x, s * (ly / 2 - 0.05), CARRIER_TOP + 0.002), WEAR_STRIP, collide=False, metal=0.6, rough=0.4)
        for e in (-1, 1):
            _cyl(scene, f"carrier/bush_{'rl'[s > 0]}{'ab'[e > 0]}", 0.018, 0.006,
                 (x + e * (lx / 2 - 0.06), s * (ly / 2 - 0.12), CARRIER_TOP), (0.5, 0.5, 0.52),
                 collide=False, metal=0.8, rough=0.3)
    for px in PACK_X:
        for s in (-1, 1):
            _box(scene, f"carrier/block_{px:+.2f}{'rl'[s > 0]}", (0.08, 0.10, PACK_Z - CARRIER_TOP),
                 (px + (0.05 if px < 0 else -0.05), s * 0.38, (CARRIER_TOP + PACK_Z) / 2),
                 (0.50, 0.51, 0.53), metal=0.5, rough=0.45)
    scene.set_part("carrier", kind="group", category="fixture",
                   model="workpiece carrier 1620 x 1100", description="Steel pallet with the pack's locating blocks and the kit's stand-offs")


# ================================================================== the pack
def _module(scene, side: str, row: int) -> None:
    sy = 1.0 if side == "L" else -1.0
    name = module_name(side, row)
    x, y = ROWS[row], sy * MODULE_Y
    w, l, h = MODULE
    z = MODULE_Z + h / 2
    # The module is what collides; the rest is how it looks.
    _box(scene, name, (w, l, h), (x, y, z), MODULE_GREY, metal=0.35, rough=0.5)
    # The cell-contacting cover over the outer part; the inner end, where the
    # terminals are, is the module's own top.
    c0, c1 = CHANNEL + 0.075, CHANNEL + l - 0.017
    _box(scene, f"{name}/cover", (w - 0.012, c1 - c0, 0.004), (x, sy * (c0 + c1) / 2, MT + 0.002), COVER_BLACK,
         collide=False, rough=0.45)
    for e in (-1, 1):   # end plates, and the steel straps that hold the stack
        _box(scene, f"{name}/end_{'io'[e * sy > 0]}", (w + 0.002, 0.016, h - 0.004),
             (x, y + e * (l / 2 - 0.008), z - 0.002), END_PLATE, collide=False, metal=0.75, rough=0.35)
    for s in (-1, 1):
        for zz in (0.035, h - 0.035):
            _box(scene, f"{name}/strap_{'ab'[s > 0]}{round(zz * 1000)}", (0.003, l - 0.04, 0.012),
                 (x + s * (w / 2 + 0.0015), y, MODULE_Z + zz), END_PLATE, collide=False, metal=0.8, rough=0.3)
    # The label, the CMC's LV connector at the inner end, between the HV terminals.
    _box(scene, f"{name}/label", (0.070, 0.090, 0.0008), (x - 0.04, y + sy * 0.06, MT + 0.0044), PAD_WHITE,
         collide=False, rough=0.8)
    for k in range(9):
        _box(scene, f"{name}/barcode_{k}", (0.050, 0.0016 + 0.0012 * (k % 3), 0.0010),
             (x - 0.04, y + sy * (0.03 + 0.006 * k), MT + 0.0049), INK, collide=False)
    _box(scene, f"{name}/lv", (0.028, 0.022, 0.016), (x, sy * (CHANNEL + 0.030), MT + 0.008), LV_BLACK,
         rough=0.5)
    _box(scene, f"{name}/lv_latch", (0.010, 0.012, 0.004), (x, sy * (CHANNEL + 0.030), MT + 0.018),
         (0.55, 0.56, 0.57), collide=False)
    scene.set_part(name, manufacturer=None, model="prismatic-cell module 12S, 400 x 220 x 140",
                   category="workpiece", description="Battery module (customer part); CMC with LV connector at the inner end")


def _terminal(scene, terminal: str, capped: bool) -> str:
    x, y = terminal_xy(terminal)
    name = f"pack/hv_{terminal.replace('+', 'p').replace('-', 'n')}"
    _box(scene, f"{name}/pad", PAD, (x, y, MT + PAD[2] / 2), PAD_WHITE, collide=False, rough=0.7)
    for s in (-1, 1):
        _cyl(scene, f"{name}/screw_{'ab'[s > 0]}", 0.0035, 0.004, (x, y + s * 0.021, MT + PAD[2]), INK,
             collide=False, metal=0.6, rough=0.4)
    header = _box(scene, f"{name}/header", HEADER, (x, y, MT + PAD[2] + HEADER[2] / 2), KS22_ORANGE, rough=0.5)
    scene.set_part(header, manufacturer="KOSTAL Kontakt Systeme", model="KS 22 tab header (1-way)",
                   category="connector", description="HV class 4 battery module connector header, 37.9 x 31.3 x 15 mm")
    if capped:
        _box(scene, f"{name}/cap", (0.019, 0.044, 0.016), (x, y, HEADER_TOP - 0.004), CAP_YELLOW, collide=False,
             rough=0.55)
    return header


def build_pack(scene) -> dict[str, str]:
    """The housing, six modules, their HV terminals and LV connectors, the
    daisy-chain harness in the channel and the BDU bay. Returns the header
    obstacle of every terminal, by terminal name."""
    x0, x1 = PACK_X
    cx, lx = (x0 + x1) / 2, x1 - x0
    _box(scene, "pack/floor", (lx, 2 * PACK_Y, FLOOR), (cx, 0.0, PACK_Z + FLOOR / 2), HOUSING_FLOOR,
         metal=0.55, rough=0.45)
    _box(scene, "pack/cooling", (ROWS[-1] - ROWS[0] + MODULE[0], 2 * PACK_Y - 2 * WALL - 0.02, COOLING),
         (0.0, 0.0, PACK_Z + FLOOR + COOLING / 2), (0.52, 0.53, 0.55), collide=False, metal=0.6, rough=0.4)
    h = WALL_TOP - PACK_Z - FLOOR
    zc = PACK_Z + FLOOR + h / 2
    for s in (-1, 1):
        _box(scene, f"pack/wall_{'rl'[s > 0]}", (lx, WALL, h), (cx, s * (PACK_Y - WALL / 2), zc), HOUSING,
             metal=0.7, rough=0.4)
        _box(scene, f"pack/flange_{'rl'[s > 0]}", (lx + 0.03, 0.04, 0.008), (cx, s * (PACK_Y + 0.005), WALL_TOP - 0.004),
             HOUSING, collide=False, metal=0.7, rough=0.4)
    for e, px in ((-1, x0), (1, x1)):
        _box(scene, f"pack/wall_{'ud'[e > 0]}", (WALL, 2 * PACK_Y, h), (px - e * WALL / 2, 0.0, zc), HOUSING,
             metal=0.7, rough=0.4)
    for k in range(len(ROWS) - 1):   # the cross members between the rows
        xm = (ROWS[k] + ROWS[k + 1]) / 2
        for s in (-1, 1):
            _box(scene, f"pack/cross_{k + 1}{'rl'[s > 0]}", (0.026, MODULE[1], 0.040),
                 (xm, s * MODULE_Y, PACK_Z + FLOOR + 0.020), HOUSING, metal=0.7, rough=0.4)
    _box(scene, "pack/cross_bay", (0.026, 2 * PACK_Y - 2 * WALL, 0.10), (BAY_X[1] + 0.013, 0.0, PACK_Z + FLOOR + 0.05),
         HOUSING, metal=0.7, rough=0.4)
    scene.set_part("pack", kind="group", category="workpiece", model="HV battery pack segment, 6 modules",
                   description="Customer product: aluminium housing, 6 x 12S modules in series, BDU and BMS master")
    for side in "LR":
        for row in range(len(ROWS)):
            _module(scene, side, row)
    headers = {}
    for side in "LR":
        for row in range(len(ROWS)):
            for sign in "-+":
                t = f"{side}{row + 1}{sign}"
                headers[t] = _terminal(scene, t, capped=t in PACK_TERMINALS)
    # The coolant manifold on the channel floor, on clamps; the BMS daisy
    # chain in its carrier on top; a branch up each module's inner face.
    x0, x1 = CHANNEL_X
    run, mid = x1 - x0, (x0 + x1) / 2
    for s in (-1, 1):
        _cyl(scene, f"pack/coolant/pipe_{'rl'[s > 0]}", PIPE_R, run, (mid, s * PIPE_Y, PIPE_Z), (0.72, 0.73, 0.75),
             quaternion=_along("x"), metal=0.8, rough=0.35)
        _cyl(scene, f"pack/coolant/cap_{'rl'[s > 0]}", PIPE_R + 0.003, 0.03, (x1 - 0.015, s * PIPE_Y, PIPE_Z),
             (0.05, 0.16, 0.55), quaternion=_along("x"), collide=False, rough=0.5)
    for k, x in enumerate(np.linspace(x0 + 0.06, x1 - 0.06, 4)):
        _box(scene, f"pack/coolant/clamp_{k}", (0.016, 2 * PIPE_Y + 2 * PIPE_R + 0.01, 0.006),
             (x, 0.0, PACK_Z + FLOOR + 0.003), (0.20, 0.21, 0.22), collide=False)
        # The carrier's bracket: a bridge over the pipes from the clamp.
        _box(scene, f"pack/lv_harness/bracket_{k}", (0.014, 0.006, CARRIER_Z - PACK_Z - FLOOR - 0.006),
             (x, 0.0, (PACK_Z + FLOOR + 0.006 + CARRIER_Z) / 2), (0.12, 0.12, 0.13), collide=False)
    scene.set_part("pack/coolant", kind="group", category="fluid", model="coolant manifold, supply and return",
                   description="Feeds the cooling plate under the modules; the HV connectors keep clear of it")
    cz = CARRIER_Z
    _box(scene, "pack/lv_harness/carrier", (run, CARRIER_W, 0.003), (mid, 0.0, cz + 0.0015), LV_BLACK, rough=0.7)
    for s in (-1, 1):
        _box(scene, f"pack/lv_harness/carrier_{'rl'[s > 0]}", (run, 0.003, CARRIER_H),
             (mid, s * (CARRIER_W / 2 - 0.0015), cz + CARRIER_H / 2), LV_BLACK, rough=0.7)
    _cyl(scene, "pack/lv_harness/trunk", LV_R, run - 0.02, (mid, 0.0, LV_Z), (0.05, 0.05, 0.055),
         quaternion=_along("x"), collide=False, rough=0.75)
    for side, sy in (("L", 1.0), ("R", -1.0)):
        for row, x in enumerate(ROWS):
            out = BRANCH_Y - CARRIER_W / 2
            _cyl(scene, f"pack/lv_harness/out_{side}{row + 1}", BRANCH_R, out,
                 (x, sy * (CARRIER_W / 2 + out / 2), LV_Z), LV_BLACK, quaternion=_along("y"), rough=0.75)
            _cyl(scene, f"pack/lv_harness/up_{side}{row + 1}", BRANCH_R, MT - LV_Z,
                 (x, sy * BRANCH_Y, (LV_Z + MT) / 2), LV_BLACK, rough=0.75)
            _box(scene, f"pack/lv_harness/clip_{side}{row + 1}", (0.012, 0.010, 0.016),
                 (x, sy * (CHANNEL - 0.006), MT - 0.03), (0.10, 0.10, 0.11), collide=False)
    scene.set_part("pack/lv_harness", kind="group", category="harness", model="BMS daisy-chain harness (LV)",
                   description="Pre-installed in its carrier; the HV module connectors keep clear of it")
    # The BDU and the BMS master in their bay.
    bx = (BAY_X[0] + BAY_X[1]) / 2
    _box(scene, "pack/bdu", (0.17, 0.42, 0.115), (bx, -0.18, PACK_Z + FLOOR + 0.0575), (0.20, 0.21, 0.23),
         metal=0.3, rough=0.5)
    _box(scene, "pack/bdu/label", (0.09, 0.0008, 0.05), (bx, -0.18 + 0.2104, PACK_Z + FLOOR + 0.07), KS22_ORANGE,
         collide=False)
    for k, yy in enumerate((-0.30, -0.06)):
        _box(scene, f"pack/bdu/stud_{k}", (0.030, 0.040, 0.025), (bx + 0.03, yy, PACK_Z + FLOOR + 0.1275),
             KS22_ORANGE, collide=False)
    scene.set_part("pack/bdu", manufacturer=None, model="battery disconnect unit", category="electrical",
                   description="Main contactors, pre-charge, fuse and current sensor (customer part)")
    _box(scene, "pack/bms", (0.14, 0.22, 0.040), (bx, 0.25, PACK_Z + FLOOR + 0.02), (0.05, 0.12, 0.07),
         metal=0.2, rough=0.5)
    scene.set_part("pack/bms", manufacturer=None, model="BMS master", category="electrical",
                   description="Battery management master; heads the daisy chain")
    return headers


# ================================================================== the kit
def _receptacle(scene, name: str, centre, cable: Cable) -> str:
    """A KS 22 receptacle housing with its CPA on top (one obstacle, its
    long side along y, the cable leaving its inner end)."""
    w, l, hh = RECEPT
    solids = [bt.parts.Box(size=(w, l, hh)),
              bt.parts.Box(size=(0.009, 0.014, 0.003), at=(0.0, 0.0, hh / 2 + 0.0015))]
    made = bt.parts.compound(scene, name, solids, centre, color=KS22_ORANGE)
    scene.set_obstacle_material(made, metalness=0.0, roughness=0.5)
    scene.set_part(made, manufacturer="KOSTAL Kontakt Systeme", model="KS 22 receptacle housing with CPA (1-way)",
                   category="connector", description=f"On {cable.part_number}; 49.65 x 21.23 x 15 mm")
    return made


def build_kit(scene, cables=CABLES) -> dict[str, dict[str, str]]:
    """The blister on its stand-offs beside the pack, the cables' housings
    in their grooves (the cables are ropes, added after the bake) — a
    blister as long as its longest cable needs. Returns `{cable: {"l":
    housing, "r": housing}}`."""
    xs = [c.kit_x for c in cables]
    KIT_Y = max(KIT_Y_MIN, max(c.length for c in cables) / 2 + CRIMP_IN + 0.030)
    x0, x1 = xs[0] - GROOVE / 2 - 0.016, xs[-1] + GROOVE / 2 + 0.016
    cx = (x0 + x1) / 2
    for k, (px, py) in enumerate(((x0 + 0.02, -KIT_Y + 0.03), (x0 + 0.02, KIT_Y - 0.03),
                                  (x1 - 0.02, -KIT_Y + 0.03), (x1 - 0.02, KIT_Y - 0.03))):
        tall = KIT_FLOOR - 0.004 - CARRIER_TOP
        _cyl(scene, f"kit/standoff_{k}", 0.010, tall, (px, py, CARRIER_TOP + tall / 2),
             (0.55, 0.56, 0.58), collide=False, metal=0.8, rough=0.3)
    _box(scene, "kit/blister", (x1 - x0, 2 * KIT_Y, 0.004), (cx, 0.0, KIT_FLOOR - 0.002), BLISTER, rough=0.7)
    edges = [x0] + [x + GROOVE / 2 for x in xs]
    starts = [x - GROOVE / 2 for x in xs] + [x1]
    for k, (a, b) in enumerate(zip(edges, starts)):
        if b - a > 1e-4:
            _box(scene, f"kit/rib_{k}", (b - a, 2 * KIT_Y, KIT_RIB), ((a + b) / 2, 0.0, KIT_FLOOR + KIT_RIB / 2),
                 BLISTER, rough=0.7)
    for s in (-1, 1):
        _box(scene, f"kit/end_{'rl'[s > 0]}", (x1 - x0, 0.008, KIT_RIB), (cx, s * (KIT_Y - 0.004), KIT_FLOOR + KIT_RIB / 2),
             BLISTER, rough=0.7)
    _box(scene, "kit/label", (0.10, 0.0008, 0.008), (cx, -KIT_Y - 0.0004, KIT_FLOOR + 0.006), PAD_WHITE,
         collide=False)
    scene.set_part("kit", kind="group", category="packaging", model="module connector kit, 5 cables",
                   description="ESD blister of the pack's module connectors, carried on the workpiece carrier")
    housings = {}
    for c in cables:
        housings[c.name] = {side: _receptacle(scene, f"{c.name}/plug_{side}", c.kit(side), c) for side in "lr"}
        scene.set_part(c.name, kind="group", category="harness", model=f"HV module connector {c.part_number}",
                       manufacturer=None, description=f"35 mm², {round(c.length * 1000)} mm crimp to crimp, "
                       "KS 22 receptacles both ends")
    return housings


def rope_obstacles(scene, cable: Cable, housings: dict) -> list[str]:
    """What a cable can lie on or brush against: the pack's modules, walls,
    floor and harness, the headers, the blister, and the other cables'
    housings (where they are at each moment)."""
    names = set(scene.obstacle_names)
    out = [module_name(s, r) for s in "LR" for r in range(len(ROWS))]
    out += ["pack/floor", "pack/coolant/pipe_l", "pack/coolant/pipe_r", "pack/lv_harness/carrier",
            "pack/lv_harness/carrier_l", "pack/lv_harness/carrier_r"]
    out += [f"pack/lv_harness/{p}_{s}{r + 1}" for p in ("out", "up") for s in "LR" for r in range(len(ROWS))]
    out += [f"pack/{n}" for n in ("cross_1l", "cross_1r", "cross_2l", "cross_2r")]
    out += [n for n in names if n.startswith("kit/rib_") or n.startswith("kit/end_")] + ["kit/blister"]
    out += [h for c in CABLES if c.name != cable.name for h in housings[c.name].values()]
    return [n for n in out if n in names]


def waiting_carrier(scene, x: float) -> None:
    """The next carrier, queued upstream on the line: its pack and kit as
    they will arrive — the picture of the line, out of collision."""
    lx, ly, t = CARRIER_SIZE
    dx = x - CARRIER_X
    name = "queue"
    _box(scene, f"{name}/plate", (lx, ly, t), (x, 0.0, CONV_Z + t / 2), CARRIER, collide=False, metal=0.4, rough=0.5)
    x0, x1 = PACK_X[0] + dx, PACK_X[1] + dx
    h = WALL_TOP - PACK_Z
    _box(scene, f"{name}/floor", (x1 - x0, 2 * PACK_Y, FLOOR), ((x0 + x1) / 2, 0.0, PACK_Z + FLOOR / 2), HOUSING_FLOOR,
         collide=False, metal=0.55, rough=0.45)
    for s in (-1, 1):
        _box(scene, f"{name}/wall_{'rl'[s > 0]}", (x1 - x0, WALL, h), ((x0 + x1) / 2, s * (PACK_Y - WALL / 2), PACK_Z + h / 2),
             HOUSING, collide=False, metal=0.7, rough=0.4)
    for e, px in ((-1, x0), (1, x1)):
        _box(scene, f"{name}/wall_{'ud'[e > 0]}", (WALL, 2 * PACK_Y, h), (px - e * WALL / 2, 0.0, PACK_Z + h / 2), HOUSING,
             collide=False, metal=0.7, rough=0.4)
    for side, sy in (("L", 1.0), ("R", -1.0)):
        for row, rx in enumerate(ROWS):
            mx = rx + dx
            _box(scene, f"{name}/module_{side}{row + 1}", MODULE, (mx, sy * MODULE_Y, MODULE_Z + MODULE[2] / 2),
                 MODULE_GREY, collide=False, metal=0.35, rough=0.5)
            c0, c1 = CHANNEL + 0.075, CHANNEL + MODULE[1] - 0.017
            _box(scene, f"{name}/cover_{side}{row + 1}", (MODULE[0] - 0.012, c1 - c0, 0.004),
                 (mx, sy * (c0 + c1) / 2, MT + 0.002), COVER_BLACK, collide=False, rough=0.45)
            for sign in (-1, 1):
                _box(scene, f"{name}/header_{side}{row + 1}{'np'[sign > 0]}", HEADER,
                     (mx + sign * TERM_DX, sy * TERM_Y, MT + PAD[2] + HEADER[2] / 2), KS22_ORANGE, collide=False)
    kx = (KIT_X0 + KIT_X0 + KIT_PITCH * (len(CABLES) - 1)) / 2 + dx
    _box(scene, f"{name}/kit", (0.24, 2 * KIT_Y_MIN, KIT_RIB + 0.004), (kx, 0.0, KIT_FLOOR + KIT_RIB / 2), BLISTER,
         collide=False, rough=0.7)
    for c in CABLES:
        for side in "lr":
            cx, cy, cz = c.kit(side)
            _box(scene, f"{name}/plug_{c.name}{side}", RECEPT, (cx + dx, cy, cz), KS22_ORANGE, collide=False)
        _cyl(scene, f"{name}/cable_{c.name}", CABLE_R, c.length - 2 * HELD,
             (c.kit_x + dx, 0.0, KIT_FLOOR + CABLE_R), HV_ORANGE, quaternion=_along("y"), collide=False, rough=0.5)
    scene.set_part(name, kind="group", category="workpiece", model="next carrier (pack and kit), queued",
                   description="Waits at the upstream stop for this one to leave")
