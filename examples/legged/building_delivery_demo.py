"""ビル内配送 — a quadruped carries the post up six storeys, on the stairs.

A small office building: 荷受け in the basement, 受付 on the ground floor,
執務エリア above. One stair, one lift. A delivery dog collects at B1F and
calls at a handover point on every floor — **and never uses the lift**. The
lift is in the cell (a device with a stop at each level, its own line on the
BOM); the sequence simply never commands it. That is the premise the drawing
states, and this is what it costs.

What the bake answers is what botrail always answers, asked of a building
instead of a cell:

* **昇れるか** — the flights are ordered from the catalog
  (`botrail/stairs/steel-flight`) and the dog from its own
  (`unitree/go2/go2`, rated `max_step_height_mm = 160`). Order the flight a
  *person's* building is built to — 175 rise on a 300 tread, which the pack
  sells and a stair-builder would fit (`--code`) — and the bake refuses it
  by name before the dog takes a step. **This machine cannot climb a code
  stair.** The building is drawn round the machine instead: 150 on a 400,
  and the storey follows the stair rather than the other way round.
* **通れるか** — the corridor, the stair hall and the shaft are walls, and
  both the dog's footprint and its links are checked against them tick by
  tick. Leave a cleaning cart across the corridor (`--cart`) and the bake
  names the piece it hits, the part of the machine that hit it, and when.
* **何分か** — 5 deliveries, 10 flights, 30 m of climb, and the per-storey
  arrival times come out of the bake. The pace is set by the *flight*, not
  the corridor: swept, this machine takes the building at 0.40 m/s and no
  faster — at 0.45 the leading leg runs out of fold on the first tread and
  the bake says so instead of letting it through.

Nothing about the walk is authored. The treads are walkable, so the
footfalls land on them rather than on the ramp the guide path interpolates;
the body pitches onto the flight and rides up with the steps; the legs do
the rest. What the cell states is the stair *posture* — lower, with a
shorter swing — because the catalog carries only how a machine stands on
the floor.

    python examples/legged/building_delivery_demo.py             # bake + USD
    python examples/legged/building_delivery_demo.py --studio    # watch it climb
    python examples/legged/building_delivery_demo.py --code      # the refusal, named
    python examples/legged/building_delivery_demo.py --cart      # the corridor, blocked
    python examples/legged/building_delivery_demo.py --floors 1  # one storey, quickly

The handover is a dwell under a zone sensor, not an animation: this machine
has no arm, so a person takes the parcel off its back. The route, the
clearances and the clock are what the cell can honestly answer for.

Two things about how it is drawn. **B1F の床が z = 0**: the studio draws its
floor at z = 0 and hides everything under it, so the building is built up
from the basement slab rather than down from grade. And botrail has no
transparency, so the glazing is drawn by what holds it — spandrel, mullion,
transom — while the pane itself is an obstacle that is never rendered.
Which is what glass is, to a machine.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import legged_patrol_demo as patrol  # noqa: E402  (the Go2, and its fallbacks)
import stairs_delivery_demo as flight_demo  # noqa: E402  (the stair posture)

# --------------------------------------------------------------- the flight
FLIGHT = "botrail/stairs/steel-flight"
SHELF = "erecta/basic/wire-shelf"
RISE, TREAD, WIDTH, STEPS = 0.15, 0.40, 0.90, 10
# The flight a *person's* building is built to. The pack sells it (2R + T =
# 650 mm, a comfortable pace); it is the dog that cannot take it.
CODE_RISE, CODE_TREAD = 0.175, 0.30
SLAB = 0.20                    # structural slab
PLENUM = 0.16                  # slab soffit to the suspended ceiling

# -------------------------------------------------------------- the building
LEVELS = ("B1F", "1F", "2F", "3F", "4F", "5F")
X0, X1 = 0.0, 18.00            # the plot, west to east
Y0, Y1 = 0.0, 6.00             # south (the glazed face) to north
GLAZE_Y, GLAZE_T = 0.10, 0.20  # the curtain wall, on the slab edge
SPANDREL = 0.95                # how much of it is solid — see `_facade`
COR_Y0, COR_Y1 = 0.20, 2.60    # the corridor: 2.40 m clear
COR_MID = 0.5 * (COR_Y0 + COR_Y1)
SCREEN_Y, SCREEN_T = 2.65, 0.10   # the office front, glazed over a 1.1 m dado
DADO = 1.10
OFF_Y1, NORTH_T = 5.85, 0.15   # the office band, and the back wall
WELL_X = 4.90                  # the corridor slab starts here; west is the well
EV_X0, EV_X1 = 16.20, 17.85    # the lift shaft
LANE_A, LANE_B = 0.75, 2.05    # the two flights, side by side in the well
FOOT_X = 5.55                  # where a flight is footed on its floor
STAIR_X = 6.80                 # where the dog lines up on the flight
HO_X = 15.30                   # the handover point, in front of the lift
PICK_X = 8.60                  # B1F, at the receiving racks
# A flight is entered on a **ramp, not a step**: the last stretch of floor
# before it climbs at about half the flight's grade over roughly a body
# length, so the body is already pitching when the leading foot reaches the
# first tread. Walk in level instead and the front legs fold past their
# limit on tread 1 and the bake refuses the walk — which is the honest
# answer, and not the one this cell wants. `SEAM` is how far a landing runs
# under the top tread's nosing: without that overlap a foot straddles the
# joint, and the tread-edge check refuses *that* instead.
LEAD, EXIT, SEAM = 0.45, 0.80, 0.30

# ------------------------------------------------------------------- surfaces
# Linear RGB, like the rest of botrail. Colour says what a surface is; the
# (metalness, roughness) pair beside it says how it takes light — an obstacle
# given neither is drawn translucent, so every solid here carries both.
#
# These read *light*. A storey under a slab gets no key light and no sky, so
# what an interior looks like in the studio is almost entirely its albedo:
# take real paint down to a "sensible" mid grey and the building goes black.
WALL = (0.80, 0.78, 0.74)           # painted plasterboard
SOFFIT = (0.86, 0.86, 0.84)         # suspended ceiling tile
VINYL = (0.62, 0.62, 0.60)          # corridor sheet vinyl
CARPET = (0.30, 0.31, 0.34)         # office carpet tile
SEALED = (0.46, 0.46, 0.44)         # B1F, sealed concrete
FASCIA = (0.55, 0.55, 0.53)         # the slab edge, seen from outside
MULLION = (0.52, 0.54, 0.56)        # anodised aluminium
DESK = (0.66, 0.59, 0.48)           # laminate
LIFT_DOOR = (0.56, 0.58, 0.60)      # hairline stainless
LAMP = (0.95, 0.94, 0.90)
SIGN = (0.06, 0.26, 0.42)
PARCEL = (0.50, 0.36, 0.20)
PLANT = (0.07, 0.22, 0.06)

MAT_PAINT = (0.05, 0.66)            # painted plasterboard, sheet metal
MAT_CONCRETE = (0.02, 0.92)
MAT_STEEL = (0.80, 0.32)
MAT_FLOOR = (0.05, 0.42)            # sheet vinyl takes a highlight
MAT_SOFT = (0.00, 0.97)             # carpet, upholstery
MAT_LAMP = (0.00, 0.24)

# ------------------------------------------------------------------ the walk
WALK_SPEED = 0.40
MAX_GRADE = 0.7
DWELL = 3.0                         # the handover
CART = ((11.80, 1.55), (0.72, 1.90, 0.98))   # `--cart`: left across it

_TOLD: set = set()


def storey_of(rise: float) -> float:
    """**The stair sets the storey**, not the other way round: a switchback
    of two `STEPS` flights, and the floor lands where they leave it."""
    return 2.0 * STEPS * rise


# --------------------------------------------------------------------- floor
def _slab(scene, name, x, y, z, *, color, material=MAT_CONCRETE, thickness=SLAB):
    """A floor: walkable, so footfalls land on it, and **not** an obstacle —
    a machine standing on a floor is not a collision, and the aisle check
    would otherwise report every storey the dog stands on."""
    made = scene.add_box(
        name, size=(x[1] - x[0], y[1] - y[0], thickness),
        position=(0.5 * (x[0] + x[1]), 0.5 * (y[0] + y[1]), z - thickness / 2.0),
        color=color,
    )
    scene.set_obstacle_walkable(made, True)
    scene.set_obstacle_enabled(made, False)
    scene.set_obstacle_material(made, *material)
    return made


def _solid(scene, name, size, position, *, color, material, quaternion=None,
           collides=True):
    made = scene.add_box(name, size=size, position=position, quaternion=quaternion,
                         color=color)
    if not collides:
        scene.set_obstacle_enabled(made, False)
    scene.set_obstacle_material(made, *material)
    return made


def _paint(scene, built, material):
    """The pair every drawn surface needs, over a whole generated group."""
    for name in built.obstacles:
        scene.set_obstacle_material(name, *material)
    return built


# -------------------------------------------------------------------- facade
def _facade(scene, level: str, z: float, clear: float) -> None:
    """The south face, one storey of it.

    The spandrel is the *real* obstacle: 0.95 m of solid wall on the slab
    edge, and the dog is 0.40 m tall, so the corridor is bounded where the
    machine actually is. The glass over it collides and is not drawn, which
    closes the storey over the dog's head and still lets a camera see in.
    The mullions and transoms are decoration on the same plane.
    """
    span = (X0 + 0.10, EV_X1)
    _paint(scene, bt.parts.wall(
        scene, f"{level}/facade", path=[(span[0], GLAZE_Y), (span[1], GLAZE_Y)],
        height=SPANDREL, thickness=GLAZE_T, base_z=z, color=FASCIA,
        model="CW-SP950", manufacturer="botrail",
    ), MAT_PAINT)
    bays = max(1, int(round((span[1] - span[0]) / 1.8)))
    for i in range(bays + 1):
        x = span[0] + (span[1] - span[0]) * i / bays
        _solid(scene, f"{level}/facade/trim/mullion{i:02d}", (0.05, GLAZE_T + 0.01, clear),
               (x, GLAZE_Y, z + clear / 2.0), color=MULLION,
               material=MAT_STEEL, collides=False)
    for tag, zc, h in (("head", z + clear - 0.05, 0.10),
                       ("sill", z + SPANDREL + 0.03, 0.06)):
        _solid(scene, f"{level}/facade/trim/{tag}", (span[1] - span[0], GLAZE_T + 0.02, h),
               (0.5 * (span[0] + span[1]), GLAZE_Y, zc), color=MULLION,
               material=MAT_STEEL, collides=False)
    pane = scene.add_box(
        f"{level}/facade/glass", size=(span[1] - span[0], 0.02, clear - SPANDREL),
        position=(0.5 * (span[0] + span[1]), GLAZE_Y, z + (clear + SPANDREL) / 2.0))
    scene.set_obstacle_visible(pane, False)


# --------------------------------------------------------------------- stair
def _flight(scene, name, position, yaw, rise, tread, pack):
    """One flight, from the catalog where it can be reached.

    A building stair is slung between two landings, so it carries no leg
    under its high end, and its handrail is not the safety orange a
    mezzanine flight is painted."""
    args = dict(
        steps=STEPS, rise=rise, tread=tread, width=WIDTH, position=position,
        yaw=yaw, legs=False, rail_color=(0.46, 0.48, 0.50),
        tread_color=(0.58, 0.59, 0.60), color=(0.50, 0.52, 0.54),
    )
    if pack is not None:
        try:
            from botrail._spec import Spec

            Spec.load(pack)
        except Exception as err:  # noqa: BLE001 — unreachable catalog, not a bad order
            if "flight" not in _TOLD:
                _TOLD.add("flight")
                print(f"catalog {pack} unavailable ({patrol.first_line(err)}); "
                      "drawing the flights from their dimensions")
            pack = None
    if pack is not None:
        return bt.parts.stairs(scene, name, catalog=pack, **args)
    return bt.parts.stairs(
        scene, name, detail="full", manufacturer="botrail",
        model=f"SF-{rise * 1000:.0f}x{tread * 1000:.0f}x{WIDTH * 1000:.0f}-{STEPS}",
        **args,
    )


def _stair_geometry(rise: float, tread: float):
    """Where the switchback's pieces land, for the flight that is ordered.

    The half landing runs `SEAM` under the down-flight's top tread, and the
    up-flight is placed so *its* top tread runs the same distance under the
    floor above — the two joints a walking machine actually crosses."""
    run, climb = STEPS * tread, STEPS * rise
    land_x1 = FOOT_X - run + SEAM
    return run, climb, (land_x1 - 1.70, land_x1), WELL_X + SEAM - run


def _stair_run(scene, i: int, z: float, rise: float, tread: float, pack) -> None:
    """The switchback taking level `i` up to level `i + 1`: up the south
    lane, round the half landing, up the north lane."""
    _run, climb, (land_x0, land_x1), foot2 = _stair_geometry(rise, tread)
    lower, upper = LEVELS[i], LEVELS[i + 1]
    _flight(scene, f"stair/{lower}/up", (FOOT_X, LANE_A, z), math.pi, rise, tread, pack)
    _slab(scene, f"stair/{lower}/landing", (land_x0, land_x1), (COR_Y0, COR_Y1),
          z + climb, color=VINYL, material=MAT_FLOOR, thickness=0.12)
    _flight(scene, f"stair/{upper}/arrive", (foot2, LANE_B, z + climb), 0.0,
            rise, tread, pack)


# ---------------------------------------------------------------------- lift
def _lift(scene, h: float, clear: float, roof: float) -> None:
    """The lift. It is in the cell — a car, doors at every landing, a stop at
    every floor — and the sequence never commands it. What the cell is for is
    to say what that costs."""
    for tag, y in (("s", COR_Y0 - 0.08), ("n", COR_Y1 + 0.08)):
        _solid(scene, f"lift/shaft/{tag}", (EV_X1 - EV_X0, 0.16, roof),
               (0.5 * (EV_X0 + EV_X1), y, roof / 2.0),
               color=FASCIA, material=MAT_CONCRETE)
    _solid(scene, "lift/shaft/e", (0.15, COR_Y1 - COR_Y0 + 0.32, roof),
           (EV_X1 + 0.075, COR_MID, roof / 2.0), color=FASCIA, material=MAT_CONCRETE)
    car_w = 1.50
    _solid(scene, "lift/car/floor", (1.45, car_w, 0.06), (EV_X0 + 0.85, COR_MID, h - 0.03),
           color=LIFT_DOOR, material=MAT_STEEL)
    for tag, y in (("s", COR_MID - car_w / 2), ("n", COR_MID + car_w / 2)):
        _solid(scene, f"lift/car/{tag}", (1.45, 0.04, 2.10),
               (EV_X0 + 0.85, y, h + 1.05), color=LIFT_DOOR, material=MAT_STEEL)
    _solid(scene, "lift/car/back", (0.04, car_w, 2.10), (EV_X0 + 1.55, COR_MID, h + 1.05),
           color=LIFT_DOOR, material=MAT_STEEL)
    scene.add_lift(
        "lift", car=["lift/car"],
        zone_position=(EV_X0 + 0.85, COR_MID, h + 1.05),
        zone_size=(1.40, car_w - 0.10, 2.05),
        stops={name: i * h for i, name in enumerate(LEVELS)},
        speed=1.0, start="1F",
    )
    scene.set_part("lift", kind="device", category="vehicle.lift", qty=1,
                   manufacturer="Generic", model="P-6-CO-600",
                   description="passenger lift, 6 stops", rated_kg="600")
    # Landing doors: two leaves and a jamb per floor, closed. They are real —
    # the corridor ends at them.
    for i, level in enumerate(LEVELS):
        z = i * h
        for tag, sign in (("l", -1.0), ("r", 1.0)):
            _solid(scene, f"{level}/lift_door/{tag}", (0.06, 0.55, 2.10),
                   (EV_X0 + 0.03, COR_MID + sign * 0.29, z + 1.05),
                   color=LIFT_DOOR, material=MAT_STEEL)
        _solid(scene, f"{level}/lift_door/head", (0.10, 1.50, clear - 2.10),
               (EV_X0 + 0.05, COR_MID, z + (clear + 2.10) / 2.0),
               color=WALL, material=MAT_PAINT)
        for tag, sign in (("l", -1.0), ("r", 1.0)):
            _solid(scene, f"{level}/lift_door/jamb_{tag}",
                   (0.10, (COR_Y1 - COR_Y0 - 1.50) / 2.0, 2.10),
                   (EV_X0 + 0.05, COR_MID + sign * (COR_Y1 - COR_Y0 + 1.50) / 4.0, z + 1.05),
                   color=WALL, material=MAT_PAINT)
        _solid(scene, f"{level}/lift_door/trim/lantern", (0.04, 0.24, 0.11),
               (EV_X0 - 0.02, COR_MID + 0.92, z + 2.32), color=LAMP,
               material=MAT_LAMP, collides=False)
        _solid(scene, f"{level}/lift_door/trim/panel", (0.04, 0.11, 0.28),
               (EV_X0 - 0.02, COR_MID + 0.94, z + 1.15), color=(0.30, 0.31, 0.33),
               material=MAT_STEEL, collides=False)


# --------------------------------------------------------------- the storeys
def _handover(scene, level: str, z: float) -> None:
    """The 受け渡し場所: a delivery locker against the office front, and the
    zone that says the dog is at it. The dwell waits on the sensor, not on
    the path index — the cell reads the world, as everywhere else."""
    _solid(scene, f"{level}/handover/locker", (1.10, 0.45, 1.40),
           (HO_X, COR_Y1 - 0.24, z + 0.70), color=(0.42, 0.45, 0.48), material=MAT_PAINT)
    for k in range(3):
        _solid(scene, f"{level}/handover/trim/door{k}", (1.04, 0.03, 0.40),
               (HO_X, COR_Y1 - 0.47, z + 0.28 + 0.44 * k),
               color=(0.55, 0.58, 0.60), material=MAT_PAINT, collides=False)
    _solid(scene, f"{level}/handover/trim/mark", (0.86, 0.86, 0.004),
           (HO_X, COR_MID, z + 0.004), color=(0.68, 0.20, 0.13),
           material=MAT_SOFT, collides=False)
    scene.add_zone_sensor(f"at_{level}", position=(HO_X, COR_MID, z + 0.30),
                          size=(1.00, 1.00, 0.90), watch_robots=["dog"])


def _office(scene, level: str, z: float, clear: float) -> None:
    """執務エリア behind a glazed screen: the dado is the wall the corridor
    check sees, the mullions and the desks beyond it are drawn only."""
    doors = [(0, x - WELL_X, 0.95) for x in (8.4, 12.6)]
    _paint(scene, bt.parts.wall(
        scene, f"{level}/screen", path=[(WELL_X, SCREEN_Y), (EV_X0, SCREEN_Y)],
        height=DADO, thickness=SCREEN_T, base_z=z, openings=doors, head=DADO,
        color=WALL, model="PT-1100", manufacturer="botrail",
    ), MAT_PAINT)
    span = EV_X0 - WELL_X
    posts = max(1, int(round(span / 1.2)))
    for i in range(posts + 1):
        x = WELL_X + span * i / posts
        _solid(scene, f"{level}/screen/trim/post{i:02d}", (0.05, SCREEN_T, clear - PLENUM - DADO),
               (x, SCREEN_Y, z + (clear - PLENUM + DADO) / 2.0), color=MULLION,
               material=MAT_STEEL, collides=False)
    _solid(scene, f"{level}/screen/trim/head", (span, SCREEN_T + 0.02, 0.06),
           (WELL_X + span / 2.0, SCREEN_Y, z + clear - PLENUM - 0.03),
           color=MULLION, material=MAT_STEEL, collides=False)

    # Desks in pairs, back to back, seen over the dado from the corridor.
    for i, x in enumerate((7.0, 8.6, 10.6, 12.2, 14.2)):
        for j, y in enumerate((3.35, 4.05)):
            _solid(scene, f"{level}/desk{i}{j}/top", (1.40, 0.68, 0.03),
                   (x, y, z + 0.72), color=DESK, material=MAT_PAINT, collides=False)
            _solid(scene, f"{level}/desk{i}{j}/ped", (0.40, 0.58, 0.60),
                   (x + 0.45, y, z + 0.30), color=(0.46, 0.47, 0.49),
                   material=MAT_PAINT, collides=False)
            _solid(scene, f"{level}/desk{i}{j}/screen", (0.05, 0.68, 0.42),
                   (x - 0.62, y, z + 0.94), color=(0.34, 0.26, 0.22),
                   material=MAT_SOFT, collides=False)
            _solid(scene, f"{level}/desk{i}{j}/chair", (0.46, 0.46, 0.46),
                   (x, y + (0.62 if j else -0.62), z + 0.30), color=(0.12, 0.12, 0.13),
                   material=MAT_SOFT, collides=False)
    for i, x in enumerate((6.6, 11.4, 15.4)):
        _solid(scene, f"{level}/cabinet{i}", (0.90, 0.45, 1.10),
               (x, OFF_Y1 - 0.30, z + 0.55), color=(0.60, 0.61, 0.62),
               material=MAT_PAINT, collides=False)


def _lobby(scene, z: float, clear: float) -> None:
    """1F エントランス(受付): the counter, the seats, and the entrance."""
    _solid(scene, "1F/reception/counter", (2.60, 0.62, 1.05), (11.0, 3.30, z + 0.525),
           color=DESK, material=MAT_PAINT)
    _solid(scene, "1F/reception/trim/top", (2.76, 0.74, 0.05), (11.0, 3.30, z + 1.075),
           color=(0.74, 0.70, 0.62), material=MAT_PAINT, collides=False)
    _solid(scene, "1F/reception/trim/back", (2.20, 0.10, 2.30), (11.0, 4.30, z + 1.15),
           color=(0.50, 0.52, 0.54), material=MAT_PAINT, collides=False)
    _solid(scene, "1F/reception/trim/sign", (1.30, 0.03, 0.30), (11.0, 4.24, z + 2.05),
           color=SIGN, material=MAT_LAMP, collides=False)
    for i, x in enumerate((7.2, 8.4)):
        _solid(scene, f"1F/sofa{i}/seat", (1.00, 0.68, 0.14), (x, 4.10, z + 0.38),
               color=(0.22, 0.26, 0.32), material=MAT_SOFT, collides=False)
        _solid(scene, f"1F/sofa{i}/back", (1.00, 0.14, 0.42), (x, 4.41, z + 0.66),
               color=(0.22, 0.26, 0.32), material=MAT_SOFT, collides=False)
    for i, x in enumerate((6.4, 13.8)):
        _solid(scene, f"1F/planter{i}/pot", (0.44, 0.44, 0.42), (x, 3.10, z + 0.21),
               color=(0.34, 0.33, 0.30), material=MAT_PAINT, collides=False)
        _solid(scene, f"1F/planter{i}/plant", (0.60, 0.60, 1.05), (x, 3.10, z + 0.95),
               color=PLANT, material=MAT_SOFT, collides=False)
    # The entrance: a pair of sliding leaves in the curtain wall, drawn open.
    # Nothing in the cell passes through it.
    for tag, sign in (("l", -1.0), ("r", 1.0)):
        _solid(scene, f"1F/entrance/{tag}", (0.90, 0.06, 2.30),
               (9.6 + sign * 1.35, GLAZE_Y, z + 1.15), color=MULLION,
               material=MAT_STEEL, collides=False)
    _solid(scene, "1F/entrance/trim/head", (3.60, GLAZE_T + 0.04, 0.22),
           (9.6, GLAZE_Y, z + 2.41), color=MULLION, material=MAT_STEEL, collides=False)


def _receiving(scene, z: float, racks) -> None:
    """B1F 荷受けエリア(倉庫): what the dog collects from."""
    for i, x in enumerate((6.6, 8.6, 10.6)):
        made = None
        if racks is not None:
            try:
                made = bt.parts.rack(scene, f"B1F/rack{i}", size=(1.20, 0.45, 1.90),
                                     position=(x, OFF_Y1 - 0.40), catalog=racks, levels=4)
            except Exception as err:  # noqa: BLE001 — unreachable catalog
                if "rack" not in _TOLD:
                    _TOLD.add("rack")
                    print(f"catalog {racks} unavailable ({patrol.first_line(err)}); "
                          "drawing the racks from their dimensions")
                racks = None
        if made is None:
            made = bt.parts.rack(scene, f"B1F/rack{i}", size=(1.20, 0.45, 1.90),
                                 position=(x, OFF_Y1 - 0.40), levels=4,
                                 model="MR-1200x450x1900", manufacturer="botrail")
        _paint(scene, made, MAT_STEEL)
        for level in range(4):
            (fx, fy, fz), _ = scene.frame(f"B1F/rack{i}/level{level}")
            for k, across in enumerate((-0.30, 0.30)):
                _solid(scene, f"B1F/rack{i}/stock/l{level}_{k}", (0.34, 0.30, 0.26),
                       (fx, fy + across, fz + 0.13), color=PARCEL, material=MAT_SOFT,
                       collides=False)
    for i, x in enumerate((13.2, 14.6)):
        bt.parts.pallet(scene, f"B1F/pallet{i}", position=(x, 4.20))
        stack = 0.62 if i == 0 else 0.40
        _solid(scene, f"B1F/stack{i}", (1.00, 0.80, stack), (x, 4.20, z + 0.144 + stack / 2.0),
               color=PARCEL, material=MAT_SOFT)
    # The collection point, and the trolley the post arrives on.
    _solid(scene, "B1F/trolley/deck", (0.80, 0.52, 0.05), (PICK_X, OFF_Y1 - 1.40, z + 0.78),
           color=LIFT_DOOR, material=MAT_STEEL)
    _solid(scene, "B1F/trolley/post", (0.05, 0.52, 0.95), (PICK_X - 0.38, OFF_Y1 - 1.40, z + 0.50),
           color=LIFT_DOOR, material=MAT_STEEL)
    # The goods shutter the post comes in through — which is why the main
    # entrance is a storey up: this is the lower ground floor.
    _solid(scene, "B1F/shutter/door", (3.20, 0.06, 2.60), (12.6, GLAZE_Y - 0.06, z + 1.30),
           color=(0.46, 0.44, 0.40), material=MAT_PAINT, collides=False)
    for k in range(11):
        _solid(scene, f"B1F/shutter/trim/slat{k}", (3.16, 0.03, 0.03),
               (12.6, GLAZE_Y - 0.11, z + 0.14 + 0.24 * k), color=(0.34, 0.33, 0.31),
               material=MAT_PAINT, collides=False)


def _storey(scene, i: int, level: str, z: float, clear: float) -> None:
    """One floor plate: the slabs, the walls round them, the ceiling, the
    light."""
    finish = (SEALED, MAT_CONCRETE) if i == 0 else (
        (VINYL, MAT_FLOOR) if i == 1 else (CARPET, MAT_SOFT))
    _slab(scene, f"{level}/slab/corridor", (X0 if i == 0 else WELL_X, EV_X0),
          (Y0, SCREEN_Y), z, color=SEALED if i == 0 else VINYL, material=MAT_FLOOR)
    _slab(scene, f"{level}/slab/office", (X0, EV_X1), (COR_Y1, Y1), z,
          color=finish[0], material=finish[1])
    for tag, path in (
        ("back", [(X0, Y1 - NORTH_T / 2), (EV_X1, Y1 - NORTH_T / 2)]),
        ("east", [(EV_X1 - NORTH_T / 2, COR_Y1), (EV_X1 - NORTH_T / 2, Y1)]),
    ):
        _paint(scene, bt.parts.wall(scene, f"{level}/wall/{tag}", path=path,
                                    height=clear, thickness=NORTH_T, base_z=z,
                                    color=WALL), MAT_PAINT)
    _facade(scene, level, z, clear)
    # A suspended ceiling is what makes a storey read as a room rather than a
    # shelf: it closes the plenum, and it is the brightest surface in there.
    # None over the well — that is a shaft, open its full height.
    for tag, x, y in (("cor", (WELL_X, EV_X0), (COR_Y0, SCREEN_Y)),
                      ("off", (X0, EV_X1), (SCREEN_Y, Y1 - NORTH_T))):
        _solid(scene, f"{level}/ceiling/{tag}", (x[1] - x[0], y[1] - y[0], 0.02),
               (0.5 * (x[0] + x[1]), 0.5 * (y[0] + y[1]), z + clear - PLENUM),
               color=SOFFIT, material=MAT_PAINT, collides=False)
    for k in range(6):
        x = 6.0 + k * 1.9
        for tag, y in (("cor", COR_MID), ("off", 4.10)):
            _solid(scene, f"{level}/light/{tag}{k}", (1.40, 0.20, 0.05),
                   (x, y, z + clear - PLENUM - 0.025), color=LAMP,
                   material=MAT_LAMP, collides=False)
    _solid(scene, f"{level}/sign", (0.05, 0.36, 0.18), (WELL_X + 0.20, COR_Y1 - 0.10, z + 2.15),
           color=SIGN, material=MAT_LAMP, collides=False)


# --------------------------------------------------------------------- route
def _route(floors: int, rise: float, tread: float, h: float):
    """The path, and the stations on it — one polyline from the basement to
    the top floor, out to each handover point and back to the stair."""
    run, climb, (land_x0, _land_x1), foot2 = _stair_geometry(rise, tread)
    path: list[tuple[float, float, float]] = []
    stations: dict[str, int] = {}
    turn = foot2 - LEAD           # where the walk crosses the half landing
    if turn <= land_x0:
        raise ValueError("the half landing is too short for the turn")

    def go(p, station: str | None = None) -> None:
        path.append((round(p[0], 4), round(p[1], 4), round(p[2], 4)))
        if station is not None:
            stations[station] = len(path) - 1

    go((PICK_X, OFF_Y1 - 2.30, 0.0), "pickup")
    go((PICK_X, COR_MID, 0.0))
    go((STAIR_X, LANE_A, 0.0), "stair_B1F")
    for i in range(floors):
        z, up = i * h, (i + 1) * h
        upper = LEVELS[i + 1]
        # up the south lane ...
        go((FOOT_X + LEAD, LANE_A, z))
        go((FOOT_X, LANE_A, z + rise / 2))
        go((FOOT_X - run, LANE_A, z + climb + rise / 2))
        # ... round the half landing ...
        go((turn, LANE_A, z + climb))
        go((turn, LANE_B, z + climb))
        # ... and up the north lane.
        go((foot2, LANE_B, z + climb + rise / 2))
        go((foot2 + run, LANE_B, up + rise / 2))
        go((foot2 + run + EXIT, LANE_B, up), f"floor_{upper}")
        # out along the corridor to the handover point, and back
        go((STAIR_X + 0.60, COR_MID, up))
        go((HO_X, COR_MID, up), f"ho_{upper}")
        if i + 1 < floors:
            go((STAIR_X + 0.60, COR_MID, up))
            go((STAIR_X, LANE_A, up), f"stair_{upper}")
    return path, stations


# --------------------------------------------------------------------- build
def build(*, robot: str = "go2", floors: int = 5, rise: float = RISE,
          tread: float = TREAD, pack: str | Path | None = FLIGHT,
          cart: bool = False, walk: float = WALK_SPEED,
          racks: str | None = SHELF) -> bt.Scene:
    floors = max(1, min(floors, len(LEVELS) - 1))
    h = storey_of(rise)
    clear, roof = h - SLAB, len(LEVELS) * h

    # Standing first: `specs.height_mm` is measured in that pose, and what
    # the dog's back needs is its shape *above the root*, which does not fold
    # with the legs. Then the package's own stair posture.
    model, standing, footprint, speed, turn = patrol.dog_of(robot)
    over_root = footprint[2] - (flight_demo.stance_depth(model, standing) + standing.foot_radius)
    _m, gait, *_rest = patrol.dog_of(robot, posture="stairs")
    gait = flight_demo.rate_step(robot, flight_demo.stair_gait(gait, model))
    back = flight_demo.stance_depth(model, gait) + gait.foot_radius + over_root

    scene = bt.Scene(model, name="dog")
    for i, level in enumerate(LEVELS):
        _storey(scene, i, level, i * h, clear)
    # The stair core's own walls run the full height: the well goes through
    # every storey, and the office beside it does not.
    _paint(scene, bt.parts.wall(scene, "core/wall", path=[(X0, SCREEN_Y), (WELL_X, SCREEN_Y)],
                                height=roof, thickness=SCREEN_T + 0.05, base_z=0.0,
                                color=WALL), MAT_PAINT)
    _paint(scene, bt.parts.wall(scene, "core/west", path=[(X0 + 0.075, Y0), (X0 + 0.075, Y1)],
                                height=roof, thickness=NORTH_T, base_z=0.0,
                                color=FASCIA), MAT_CONCRETE)
    for i in range(floors):
        _stair_run(scene, i, i * h, rise, tread, pack)
    _lift(scene, h, clear, roof)
    _slab(scene, "roof", (X0, EV_X1), (Y0, Y1), roof, color=FASCIA)
    _paint(scene, bt.parts.wall(scene, "roof/parapet",
                                path=[(X0 + 0.1, GLAZE_Y), (EV_X1 - 0.1, GLAZE_Y),
                                      (EV_X1 - 0.1, Y1 - 0.1), (X0 + 0.1, Y1 - 0.1)],
                                closed=True, height=0.70, thickness=0.18, base_z=roof,
                                color=FASCIA), MAT_CONCRETE)
    # The machine room over the shaft, and the tank beside it: what a lift
    # puts on a skyline, and the reason a building's roof is never flat.
    _solid(scene, "roof/plant/machine_room", (2.60, 3.00, 2.40),
           (EV_X0 + 0.70, COR_MID + 0.30, roof + 1.20), color=FASCIA,
           material=MAT_CONCRETE, collides=False)
    _solid(scene, "roof/plant/tank", (1.80, 1.40, 1.10), (12.4, 4.10, roof + 1.15),
           color=(0.62, 0.63, 0.64), material=MAT_STEEL, collides=False)
    for i, x in enumerate((11.7, 13.1)):
        for j, y in enumerate((3.6, 4.6)):
            _solid(scene, f"roof/plant/leg{i}{j}", (0.09, 0.09, 0.60),
                   (x, y, roof + 0.30), color=(0.42, 0.43, 0.44),
                   material=MAT_STEEL, collides=False)

    _receiving(scene, 0.0, racks)
    _lobby(scene, h, clear)
    for i in range(2, len(LEVELS)):
        _office(scene, LEVELS[i], i * h, clear)
    for i in range(1, floors + 1):
        _handover(scene, LEVELS[i], i * h)

    if cart:
        # A cleaning cart left across the corridor on 2F — the thing a
        # building actually does to a robot's route.
        (cx, cy), (sx, sy, sz) = CART
        z = 2 * h
        _solid(scene, "2F/cart/body", (sx, sy, sz), (cx, cy, z + sz / 2.0),
               color=(0.62, 0.50, 0.14), material=MAT_PAINT)
        _solid(scene, "2F/cart/trim/bag", (sx - 0.10, sy - 0.60, 0.70),
               (cx, cy - 0.55, z + 0.35), color=(0.22, 0.24, 0.26),
               material=MAT_SOFT, collides=False)

    # ---- the dog: a vehicle with legs, and the post on its back ----------
    path, stations = _route(floors, rise, tread, h)
    scene.add_box("dog/footprint", footprint,
                  (path[0][0], path[0][1], footprint[2] / 2.0))
    scene.set_obstacle_visible("dog/footprint", False)
    case = (0.34, 0.24, 0.20)   # the size of the machine's own back
    _solid(scene, "post/case", case, (path[0][0], path[0][1], back + case[2] / 2.0),
           color=(0.30, 0.38, 0.50), material=MAT_PAINT, collides=False)
    _solid(scene, "post/trim/lid", (case[0] - 0.03, case[1] - 0.03, 0.025),
           (path[0][0], path[0][1], back + case[2]), color=(0.46, 0.54, 0.62),
           material=MAT_PAINT, collides=False)
    # The load has a mass, so the deck load is something the cell can weigh
    # against what the machine is rated to carry (the Go2's 8 kg).
    scene.set_part("post/case", kind="obstacle", category="workpiece", qty=1,
                   description="post, one round", mass_kg=5.0)
    scene.add_vehicle(
        "dog", body=["dog/footprint"], path=path, stations=stations,
        speed=min(speed, walk), turn_speed=turn, start="pickup", max_grade=MAX_GRADE,
        tray_position=(0.0, 0.0, back + case[2] / 2.0 + 0.02),
        tray_size=(case[0] + 0.14, case[1] + 0.14, case[2] + 0.10),
    )
    scene.mount_robot("dog", gait=gait)
    scene.set_part("dog", kind="device", category="vehicle.legged", qty=1)

    # ---- the cycle: to the flight, up it, out to the handover, dwell -----
    seq = scene.sequence("deliver")
    for i in range(floors):
        lower, upper = LEVELS[i], LEVELS[i + 1]
        seq.step(f"to_stair_{lower}", actions=[bt.seq.goto("dog", f"stair_{lower}")],
                 transition=bt.seq.device_done("dog"))
        seq.step(f"climb_{upper}", actions=[bt.seq.goto("dog", f"floor_{upper}")],
                 transition=bt.seq.device_done("dog"))
        seq.step(f"deliver_{upper}", actions=[bt.seq.goto("dog", f"ho_{upper}")],
                 transition=bt.seq.all_of(bt.seq.device_done("dog"),
                                          bt.seq.signal(f"at_{upper}")))
        seq.step(f"handover_{upper}", transition=bt.seq.elapsed(DWELL))
    return scene


def bake(*, robot: str = "go2", floors: int = 5, rise: float = RISE,
         tread: float = TREAD, pack: str | Path | None = FLIGHT, cart: bool = False,
         walk: float = WALK_SPEED):
    scene = build(robot=robot, floors=floors, rise=rise, tread=tread, pack=pack,
                  cart=cart, walk=walk)
    return scene, scene.simulate_sequence("deliver", max_duration=900.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", nargs="?", default=str(HERE / "building_cell.usdc"))
    parser.add_argument("--robot", default="go2",
                        help="go2 (the catalog dog), quad, or a package directory")
    parser.add_argument("--flight", default=FLIGHT,
                        help="the stair pack: a catalog id or a package directory")
    parser.add_argument("--floors", type=int, default=len(LEVELS) - 1,
                        help="how many storeys to deliver to (1..5)")
    parser.add_argument("--code", action="store_true",
                        help=f"order the flight a person's building is built to "
                             f"({CODE_RISE * 1000:.0f} x {CODE_TREAD * 1000:.0f})")
    parser.add_argument("--cart", action="store_true",
                        help="leave a cleaning cart across the 2F corridor")
    parser.add_argument("--speed", type=float, default=WALK_SPEED, metavar="MPS",
                        help="how fast the dog walks (the flight is what caps it)")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    if args.code:
        try:
            bake(robot=args.robot, floors=1, rise=CODE_RISE, tread=CODE_TREAD,
                 pack=args.flight, walk=args.speed)
        except ValueError as err:
            print(f"the storey would be {storey_of(CODE_RISE):.2f} m and the flight "
                  f"a normal one. Refused, as it should be:")
            print(f"  {patrol.first_line(err)}")
            return
        raise SystemExit(f"a {CODE_RISE * 1000:.0f} mm riser against a 160 mm step "
                         "should have been refused")

    if args.cart:
        try:
            bake(robot=args.robot, floors=args.floors, pack=args.flight, cart=True,
                 walk=args.speed)
        except ValueError as err:
            print("refused, as it should be:")
            print(f"  {patrol.first_line(err)}")
            return
        raise SystemExit("a cart across a 2.40 m corridor should have been hit")

    scene, tl = bake(robot=args.robot, floors=args.floors, pack=args.flight,
                     walk=args.speed)
    for row in scene.bom().rows:
        mass = row["attributes"].get("mass_kg")
        print(f"  {row['names'][0]:<22} {row['category']:<22} x{row['qty']:<3} "
              f"{(row.get('model') or '')[:22]:<22} {f'{mass} kg' if mass else ''}")
    steps = tl.footfalls("dog")
    print(f"\ncycle {tl.duration:.1f}s over {args.floors} deliveries at "
          f"{min(patrol.dog_of(args.robot)[3], args.speed):.2f} m/s, "
          f"{len(steps)} footfalls, top foothold z = {max(f[3][2] for f in steps):.2f} m")
    spans = {name: (t0, t1) for name, t0, t1 in tl.step_spans}
    previous = 0.0
    for i in range(1, args.floors + 1):
        at = spans.get(f"handover_{LEVELS[i]}")
        if at is None:
            continue
        print(f"  {LEVELS[i]:<4} handover at {at[0]:6.1f}s   "
              f"(+{at[0] - previous:5.1f}s for the storey)")
        previous = at[1]
    tl.export_usd(args.out, fps=args.fps)
    print(f"wrote {args.out}")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
