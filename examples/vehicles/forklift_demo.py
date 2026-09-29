"""An automated forklift putting pallets into racking, ordered from the catalog.

A Toyota Autopilot SAE160 — a 1.6 t support-arm stacker with a triplex
mast — serves a run of TRUSCO pallet racking across a 3.0 m aisle from a
second run of racking. One shift: a received pallet on the floor is taken
forks-first, carried drive-unit-first down the aisle, and put away on the
upper beam level; a pallet waiting on the lower level is taken out and set
down on the shipping spot; the truck backs onto its charger. Nothing is
drawn from a picture of the truck: the mast, the forks, the support arms,
the wheels and every figure the cell checks against come out of the
package the catalog builder validated against the maker's type sheet.

What the bake answers:

  * **Does it turn in the aisle?** A stacker pivots about its support-arm
    axle with the drive unit swinging 1.78 m behind it. Every tick it
    moves, the truck and what it carries are checked against both racks;
    `--aisle 2.5` is refused at a named time against a named upright, the
    sheet's own Ast (2.696 m for this pallet) passes.
  * **Does it reach the level?** The sheet limits an automode load station
    to h23 − 200 mm. `--mast dx` (the 2.35 m duplex mast) is refused for
    level 2 by that rule and serves level 1 with `--level 1`.
  * **Do the forks fit the pallet?** The pallet is timber, EPAL 1 layout:
    the forks and the support arms enter between the bottom boards and
    under the stringers, and the withdrawal after a set-down passes the
    fork blade between the beam and the pallet's stringers — all of it
    collision-checked as the truck drives.
  * **What does the shift cost?** Forks-first travel is the sheet's 1.1
    km/h, drive-unit-first 8.0 km/h; the truck turns round when that pays
    (a corner takes the gear that has the leg done soonest) and arrives
    forks-first where a station demands it. The lift runs at the sheet's
    laden and unladen speeds.
  * **What if it could not pivot?** A tiller stacker turns on its
    support-arm axle. `--turn-radius 0.6` drives the same route as a
    counterbalance truck would — every corner an arc of that radius, a
    gear change an overshoot past the corner and a swing back — and the
    same aisle check says what that costs in width (a counterbalance
    stacker's sheet quotes 3.3 m and more).

Run with:  python examples/vehicles/forklift_demo.py [recording.usdc] [--studio]
                        [--out DIR] [--aisle M] [--mast tx|dx] [--level N]
                        [--turn-radius M] [--sweep] [--catalog-root DIR]

`--out DIR` writes the document set (layout SVG/DXF, BOM, I/O list,
PLCopen, report, USD recording). `--sweep` bakes the shift at several aisle
widths and prints which pass. `--catalog-root` points at a catalog
builder's `build/` directory instead of the published dataset.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import botrail as bt
import yaml

HERE = Path(__file__).resolve().parent

# ---- what is ordered ---------------------------------------------------
TRUCKS = {
    "tx": "toyota_material_handling/autopilot/sae160/r1",     # TX Hi-Lo triplex mast, 4.7 m
    "dx": "toyota_material_handling/autopilot/sae160-dx/r1",  # DX Tele duplex mast, 2.35 m
}
RACK = "trusco/pallet-rack/1d-2500x1100/r1"                   # 1-ton racking, 2500 clear bays

CATALOG_ROOT: Path | None = None   # set by --catalog-root: a builder's build/ directory


def ref(pid: str):
    """A spec pack reference: the id, or the built package directory."""
    return CATALOG_ROOT / pid if CATALOG_ROOT else pid


def load(pid: str) -> bt.Robot:
    if CATALOG_ROOT:
        return bt.Robot.from_package(CATALOG_ROOT / pid, catalog_root=CATALOG_ROOT)
    return bt.Robot.from_catalog(pid)


def package_dir(pid: str) -> Path:
    return CATALOG_ROOT / pid if CATALOG_ROOT else Path(bt.catalog_package(pid))


def manifest(pid: str) -> dict:
    return yaml.safe_load((package_dir(pid) / "manifest.yaml").read_text(encoding="utf-8")) or {}


# ---- the layout (m): x east, y north -----------------------------------
AISLE = 3.0                 # rack face to rack face; the sheet's Ast for this pallet is 2.696
RACK_X, RACK_Y = 8.5, 9.0   # the served run's centre; its aisle face is south
RACK_HEIGHT, RACK_LEVELS = 3.5, 2
SEATS = (1.5, 3.0)          # beam tops: a 0.9 m load fits under the next beam
PIVOT_OFF = 0.80            # the truck pivots this far off the served rack's face
RECV_X, TURN_X = 1.0, 3.6   # the received pallet, and where the truck turns round for it (its rear
                            # clears the pallet, its sweep clears the rack's first upright)
SHIP_X = 13.2               # the shipping spot's stub off the lane
PULL_X, CHARGER_X = 13.2, 15.2  # where the truck turns after pulling off the charger, and the charger
PRE = 1.5                   # the stop before a bay's corner, where the forks are set for the level
PALLET = (1.2, 0.8, 0.144)  # EUR pallet, the long side along the forks
LOAD = (1.15, 0.75, 0.9)
LOAD_KG = 600.0
PALLET_KG = 25.0            # an EPAL 1, about
TURN = math.radians(30)     # pivot rate: not on the sheet
UNLADEN_UP, UNLADEN_DOWN = 0.34, 0.41   # the sheet's unladen lift / lowering speeds
TRAVEL_TOP = 0.30           # fork top while carrying a pallet
ENTRY_CLEAR = 0.10          # the pallet over the beam on the way in
RELEASE_DROP = 0.02         # the forks dropped under the stringers before withdrawing
STRINGER = 0.099            # the pallet's stringer underside over its bottom (EPAL layout, bt.parts.pallet)
PALLET_GAP = 0.02           # the pallet stands this far off the fork face (the shanks never touch its stringers)
STUDIO_VIEW = ((16.5, -5.5, 7.5), (7.5, 6.0, 1.2))

WOOD = (0.62, 0.47, 0.30)
CASES = (0.80, 0.68, 0.50)
STEEL = (0.55, 0.57, 0.60)
LINE = (0.95, 0.80, 0.15)


def yaw_quat(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def group_of(scene: bt.Scene, prefix: str) -> list[str]:
    return [n for n in scene.obstacle_names if n == prefix or n.startswith(prefix + "/")]


# ================================================================== the truck
class Truck:
    """The truck as its package states it: the model, its wheels, the mast's
    joints, and the sheet's figures the cell checks against. Nothing below
    knows it is a Toyota."""

    def __init__(self, pid: str):
        spec = manifest(pid)
        self.id, self.product = spec["id"], spec["name"]
        self.maker = spec["manufacturer"]["name"]
        self.specs = spec["specs"]
        self.model = load(pid)
        self.wheels = bt.Wheels.from_catalog(package_dir(pid) if CATALOG_ROOT else pid)
        mast = next(g for g in spec["frames"]["groups"] if g["name"] == "mast")
        self.mast = list(mast["joints"])                    # floor first: free lift, then the stages
        names = list(self.model.joint_names)
        limits = self.model.joint_limits
        self.travel = {j: (limits[names.index(j)][1] - limits[names.index(j)][0]) for j in self.mast}
        self.h13 = self.specs["fork_height_lowered_mm"] / 1000.0   # fork top, lowered
        self.h23 = self.specs["lift_height_mm"] / 1000.0           # fork top, raised
        self.max_station = self.h23 - 0.200                        # the sheet: automode load stations end here
        self.x = self.specs["load_distance_mm"] / 1000.0           # axle to fork face
        self.fork_length = self.specs["fork_length_mm"] / 1000.0
        self.width = self.specs["footprint_mm"][1] / 1000.0
        self.rear = self.specs["footprint_mm"][0] / 1000.0 - self.fork_length + self.x   # axle to the rear end
        self.wa = self.specs["turning_radius_mm"] / 1000.0
        self.ast = self.specs["aisle_width_mm"] / 1000.0
        self.speed_forks = self.specs["max_speed_fork_first_mps"]
        self.speed_drive = self.specs["max_speed_mps"]
        self.lift_laden = self.specs["lift_speed_mps"]
        self.lower_laden = self.specs["lowering_speed_mps"]
        self.top = self.h13   # the fork top the program has left the forks at

    @property
    def swing(self) -> float:
        """What a pivot sweeps: the rear corner about the axle."""
        return math.hypot(self.rear, self.width / 2)

    def pose(self, top: float) -> dict[str, float]:
        """Mast joint values that put the fork top `top` over the floor,
        the joints used floor first."""
        left = top - self.h13
        if left < -1e-9 or left > sum(self.travel.values()) + 1e-9:
            raise ValueError(f"{self.product}: a fork top of {top:.3f} m is outside the mast's "
                             f"{self.h13:.3f}–{self.h23:.3f} m")
        pose = {}
        for joint in self.mast:
            pose[joint] = min(max(left, 0.0), self.travel[joint])
            left -= pose[joint]
        return pose

    def lift(self, sq, name: str, top: float, laden: bool) -> None:
        """A ramp to the fork top `top` at the sheet's speed for the load."""
        rising = top > self.top
        speed = (self.lift_laden if laden else UNLADEN_UP) if rising else (self.lower_laden if laden else UNLADEN_DOWN)
        seconds = max(abs(top - self.top) / speed, 0.05)
        sq.step(name, actions=[bt.seq.ramp(self.pose(top), seconds, robot="agf")])
        self.top = top


# ================================================================== the cell
def stock(scene: bt.Scene, name: str, seat: tuple, yaw: float, *, collide: bool) -> None:
    """A pallet of cases standing in the racking — scenery, not the shift's."""
    bt.parts.unit_load(scene, name, seat, yaw=yaw, pallet=PALLET, height=LOAD[2] - 0.15, collide=collide, detail="full")


def pallet_of_cases(scene: bt.Scene, name: str, at: tuple, yaw: float, tag: str) -> list[str]:
    """A timber pallet with a stack of cases on it — the load the truck
    moves, board by board, so its pockets are what the forks enter."""
    x, y, z = at
    built = bt.parts.pallet(scene, name, (x, y, z), size=PALLET, yaw=yaw, mass_kg=PALLET_KG)
    cases = scene.add_box(f"{name}/load", LOAD, (x, y, z + PALLET[2] + LOAD[2] / 2), quaternion=yaw_quat(yaw), color=CASES)
    scene.set_part(cases, category="workpiece", model=f"cases {tag}", mass_kg=LOAD_KG)
    return [*built.obstacles, cases]


def charger(scene: bt.Scene, at: tuple, yaw: float) -> None:
    """A floor-standing charging station: a housing and the contact head at
    the truck's plate height, facing local -Y (the truck's right flank
    parks beside it)."""
    x, y = at
    q = yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)

    def place(lx, ly):
        return (x + c * lx - s * ly, y + s * lx + c * ly)

    housing = scene.add_box("charger/housing", (0.50, 0.30, 1.20), (*place(0.0, 0.0), 0.60), quaternion=q, color=STEEL)
    head = scene.add_box("charger/head", (0.25, 0.05, 0.15), (*place(0.0, -0.175), 0.359), quaternion=q, color=(0.66, 0.53, 0.18))
    scene.set_obstacle_material(housing, metalness=0.4, roughness=0.5)
    scene.set_obstacle_material(head, metalness=0.8, roughness=0.35)
    scene.set_part("charger", kind="group", category="vehicle.charger", model="Autopilot charging station",
                   manufacturer="Toyota Material Handling")


def build(*, aisle: float = AISLE, mast: str = "tx", level: int = 2, turn_radius: float = 0.0) -> tuple[bt.Scene, Truck, dict]:
    """The cell: two runs of racking across the aisle, the received pallet,
    the pallet waiting in the rack, the shipping spot, the charger — and
    the truck on its taped route."""
    truck = Truck(TRUCKS[mast])
    seat_put = SEATS[level - 1]
    need = seat_put + STRINGER + ENTRY_CLEAR
    if need > truck.max_station + 1e-9:
        raise ValueError(
            f"{truck.product}: level {level} seats a pallet at {seat_put:.2f} m, so the forks must reach "
            f"{need:.2f} m to enter — over the sheet's automode load station limit h23 − 200 mm = "
            f"{truck.max_station:.2f} m; use --level {max(i + 1 for i, s in enumerate(SEATS) if s + STRINGER + ENTRY_CLEAR <= truck.max_station)}"
        )
    scene = bt.Scene(truck.model, name="agf")

    # ---- the racking: the served run, and the stock run across the aisle
    depth = 1.1
    face = RACK_Y - depth / 2
    bt.parts.pallet_rack(scene, "rack/A", (RACK_X, RACK_Y), bays=2, height=RACK_HEIGHT, levels=RACK_LEVELS,
                         beam_heights=SEATS, catalog=ref(RACK))
    rack_b_y = face - aisle - depth / 2
    bt.parts.pallet_rack(scene, "rack/B", (RACK_X, rack_b_y), bays=2, height=RACK_HEIGHT, levels=RACK_LEVELS,
                         beam_heights=SEATS, catalog=ref(RACK))
    seats = {}
    for run in ("A", "B"):
        for bay in range(2):
            for lv in range(RACK_LEVELS + 1):
                seats[(run, bay, lv)] = scene.frame(f"rack/{run}/bay{bay}/level{lv}")[0]
    for bay in range(2):
        for lv in range(RACK_LEVELS + 1):
            if (bay + lv) % 3 != 1:
                stock(scene, f"rack/B/stock{bay}{lv}", seats[("B", bay, lv)], math.pi / 2, collide=lv == 0)
    # The served run: stock above the pick and below the put-away, and the
    # floor level of both served bays empty — the support arms roll in there.
    stock(scene, "rack/A/stock02", seats[("A", 0, 2)], math.pi / 2, collide=True)
    stock(scene, f"rack/A/stock1{3 - level}", seats[("A", 1, 3 - level)], math.pi / 2, collide=True)

    # ---- the lane: a pivoting truck turns on the spot PIVOT_OFF off the rack
    # face; one that rounds its corners must have its fork tips clear of a
    # pallet before the arc begins, so its lane keeps tips + radius off the
    # face, and its stubs run that much longer too.
    tips = truck.fork_length - truck.x          # the fork tips ahead of the axle
    lane = face - (PIVOT_OFF if turn_radius <= 0 else tips + turn_radius + 0.15)
    stub = turn_radius + tips + PALLET[0] / 2 + PALLET_GAP + 0.25   # 1.48 m for a pivoting truck
    # ---- the pallets the shift moves: A received on the floor, B waiting on level 1 of bay 0
    bay_x = [seats[("A", 0, 0)][0], seats[("A", 1, 0)][0]]
    recv = (RECV_X, lane)                       # the truck's axle at the pickup, forks west
    ship = (SHIP_X, lane - stub)                # the truck's axle at the set-down, forks south
    # A pallet against the forks is centred 0.6 m ahead of the fork face, less the gap.
    seat_off = PALLET[0] / 2 - truck.x + PALLET_GAP           # pallet centre from the axle, along the forks
    pallet_a = pallet_of_cases(scene, "pallet/A", (recv[0] - seat_off, recv[1], 0.0), 0.0, "A")
    pallet_b = pallet_of_cases(scene, "pallet/B", seats[("A", 0, 1)], math.pi / 2, "B")
    bt.parts.marking(scene, "marking/recv", rect=(recv[0] - 0.70, recv[1] - 0.50, recv[0] + 0.70, recv[1] + 0.50), color=LINE)
    bt.parts.marking(scene, "marking/ship", rect=(ship[0] - 0.50, ship[1] - 0.70, ship[0] + 0.50, ship[1] + 0.70), color=LINE)
    bt.parts.marking(scene, "marking/lane", line=((RECV_X - 0.5, lane), (CHARGER_X + 1.0, lane)), dash=(0.6, 0.4), width=0.05, color=LINE)

    # ---- the charger, beside the truck's right flank when it has backed home
    home = (CHARGER_X, lane)
    # The contact head stands 15 mm off the truck's flank (its plate is 1.104 m
    # behind the axle, at 359 mm), the housing behind the head.
    charger(scene, (home[0] + 1.104, home[1] + truck.width / 2 + 0.015 + 0.025 + 0.175), 0.0)

    # ---- the route: the shift's itinerary, taped (stations at their
    # waypoints). The stops where the forks are set for a level sit on the
    # lane a little before each bay's corner, so a truck that cannot pivot
    # can round the corner from them; a tiller truck pivots there instead.
    y_bay = RACK_Y - seat_off                   # the axle's stop that centres the pallet on the seat
    path = [
        home, (PULL_X, lane), (TURN_X, lane), recv,                                  # 0–3
        (bay_x[1] - PRE, lane), (bay_x[1], lane), (bay_x[1], y_bay), (bay_x[1], lane), (bay_x[1] - PRE, lane),   # 4–8 bay 1
        (bay_x[0] - PRE, lane), (bay_x[0], lane), (bay_x[0], y_bay), (bay_x[0], lane), (bay_x[0] + PRE, lane),   # 9–13 bay 0
        (SHIP_X, lane), ship, (SHIP_X, lane), home,                                  # 14–17 ship, home
    ]
    stations = {"charger": 0, "recv": 3, "bay1_pre": 4, "bay1": 6, "bay1_out": 8,
                "bay0_pre": 9, "bay0": 11, "bay0_out": 13, "ship": 15, "home": 17}
    arrive = {"recv": "forward", "bay1": "forward", "bay0": "forward", "ship": "forward", "home": "reverse"}
    drive = {"drive": "steered", "turn_radius": turn_radius} if turn_radius > 0 else {}
    scene.add_vehicle("agf_base", body=[], path=path, stations=stations, start="charger",
                      speed=truck.speed_forks, reverse_speed=truck.speed_drive, turn_speed=TURN,
                      allow_reverse=True, prefer="reverse", arrive=arrive, **drive)
    scene.mount_robot("agf_base", robot="agf", wheels=truck.wheels)

    # ---- what the PLC sees: a photo-eye across each served bay, the spot sensors, the charger
    for bay, lv, tag in ((1, level, "bay1_full"), (0, 1, "bay0_full")):
        sx, sy, sz = seats[("A", bay, lv)]
        scene.add_beam_sensor(tag, (sx - 1.15, sy, sz + 0.45), (sx + 1.15, sy, sz + 0.45),
                              watch=["pallet/A/load", "pallet/B/load"])
    # A zone sees an object by its origin: a deck board's is 13 cm up, so a
    # low zone on the spot sees the pallet and stays out of the picture.
    scene.add_zone_sensor("ship_full", position=(ship[0], ship[1], 0.15), size=(1.0, 1.4, 0.3),
                          watch=["pallet/B/deck2"])
    scene.add_zone_sensor("at_charger", position=(home[0] + 0.8, home[1], 0.03), size=(2.4, 1.2, 0.06),
                          watch_robots=["agf"])

    # ---- the shift
    seat_pick = SEATS[0]
    sq = scene.sequence("shift")

    def drive(name: str, station: str) -> None:
        sq.step(name, actions=[bt.seq.goto("agf_base", station)], transition=bt.seq.device_done("agf_base"))

    def hold(name: str, pieces: list[str]) -> None:
        sq.step(name, actions=[bt.seq.attach(p, link="forks", robot="agf") for p in pieces], transition=bt.seq.immediately())

    def release(name: str, pieces: list[str], then: str) -> None:
        sq.step(name, actions=[bt.seq.detach(p) for p in pieces], transition=bt.seq.signal(then))

    # ① the received pallet, forks-first off the floor, up to level `level`
    drive("to_recv", "recv")
    truck.lift(sq, "seat_a", STRINGER, laden=False)                 # the fork top meets the stringers
    hold("hold_a", pallet_a)
    truck.lift(sq, "lift_a", TRAVEL_TOP, laden=True)
    drive("to_bay1_pre", "bay1_pre")
    truck.lift(sq, "raise_a", seat_put + STRINGER + ENTRY_CLEAR, laden=True)
    drive("to_bay1", "bay1")
    truck.lift(sq, "land_a", seat_put + STRINGER, laden=True)       # the pallet rests on the beams
    release("release_a", pallet_a, "bay1_full")
    truck.lift(sq, "clear_a", seat_put + STRINGER - RELEASE_DROP, laden=False)
    drive("to_bay1_out", "bay1_out")
    # ② the pallet on level 1 of bay 0, out to the shipping spot
    truck.lift(sq, "lower_a", seat_pick + STRINGER - RELEASE_DROP, laden=False)
    drive("to_bay0_pre", "bay0_pre")
    sq.step("bay0_ready", actions=[], transition=bt.seq.signal("bay0_full"))
    drive("to_bay0", "bay0")
    truck.lift(sq, "seat_b", seat_pick + STRINGER, laden=False)
    hold("hold_b", pallet_b)
    truck.lift(sq, "lift_b", seat_pick + STRINGER + ENTRY_CLEAR, laden=True)
    drive("to_bay0_out", "bay0_out")
    truck.lift(sq, "lower_b", TRAVEL_TOP, laden=True)
    drive("to_ship", "ship")
    truck.lift(sq, "land_b", STRINGER, laden=True)
    release("release_b", pallet_b, "ship_full")
    truck.lift(sq, "clear_b", truck.h13, laden=False)
    # ③ home: out of the shipping spot, round the corner and backing onto the charger
    drive("to_home", "home")
    sq.step("charging", actions=[], transition=bt.seq.signal("at_charger"))

    info = {"aisle": aisle, "level": level, "seat_put": seat_put, "seat_pick": seat_pick, "lane": lane,
            "turn_radius": turn_radius,
            "face": face, "rack_b_face": rack_b_y + depth / 2, "pallet_a": pallet_a, "pallet_b": pallet_b,
            "seats": seats, "ship": ship, "recv": recv, "home": home}
    return scene, truck, info


def bake(*, aisle: float = AISLE, mast: str = "tx", level: int = 2, turn_radius: float = 0.0):
    scene, truck, info = build(aisle=aisle, mast=mast, level=level, turn_radius=turn_radius)
    return scene, truck, info, scene.simulate_sequence("shift", max_duration=900.0)


# ================================================================== reports
def _high_spans(edges, end: float) -> list[tuple[float, float]]:
    spans, start = [], None
    for t, v in edges:
        if v and start is None:
            start = t
        elif not v and start is not None:
            spans.append((start, t))
            start = None
    if start is not None:
        spans.append((start, end))
    return spans


def flows(tl) -> dict[str, tuple[float, float]]:
    spans = {name.rsplit("/", 1)[-1]: (t0, t1) for name, t0, t1 in tl.step_spans}
    return {
        "① 入荷パレットの棚入れ": (spans["to_recv"][0], spans["clear_a"][1]),
        "② 棚出し・出荷": (spans["lower_a"][0], spans["clear_b"][1]),
        "③ 帰還・充電": (spans["to_home"][0], spans["charging"][1]),
    }


def tightest_pass(scene: bt.Scene, tl, truck: Truck, prefix: str, dt: float = 0.05) -> tuple[float, float, str]:
    """How close the driving truck — body, mast, forks and what it carries —
    comes to the standing obstacles under `prefix` (the racking across the
    aisle, the charger): axis-aligned bounds of the truck's envelope, grown
    to a pallet, against each of them. A floor on the true clearance the
    rollout proved; the served rack is not asked, since the truck drives
    into it on purpose."""
    static = [(n, *scene.obstacle_bounds(n)) for n in scene.obstacle_names if n.startswith(prefix)]
    half = (max(truck.rear, truck.fork_length - truck.x + 0.05), max(truck.width, PALLET[1]) / 2)
    best = (math.inf, 0.0, "")
    for t0, t1 in _high_spans(dict(tl.signals)["agf_base"], tl.duration):
        t = t0
        while t <= t1:
            (x, y, _), q = tl.base_pose(t, robot="agf")
            yaw = yaw_of(q)
            c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
            hx, hy = half[0] * c + half[1] * s, half[0] * s + half[1] * c
            lo, hi = (x - hx, y - hy, 0.017), (x + hx, y + hy, truck.h23 + 0.7)
            for other, olo, ohi in static:
                gap = max(0.0, olo[0] - hi[0], lo[0] - ohi[0], olo[1] - hi[1], lo[1] - ohi[1], olo[2] - hi[2], lo[2] - ohi[2])
                if gap < best[0]:
                    best = (gap, t, other)
            t += dt
    return best


def report(scene: bt.Scene, truck: Truck, info: dict, tl) -> None:
    turning = (f"rounds its corners at {info['turn_radius']:.2f} m (as a counterbalance truck would)"
               if info["turn_radius"] > 0 else
               f"pivots about its support-arm axle sweeping {truck.swing:.2f} m (sheet Wa {truck.wa:.3f} m)")
    print(f"{truck.product} ({truck.maker}): forks {truck.fork_length:.2f} m, lifts to {truck.h23:.2f} m "
          f"(automode stations to {truck.max_station:.2f} m), {turning}; {truck.speed_drive:.2f} m/s drive-unit first, "
          f"{truck.speed_forks:.2f} m/s forks first")
    print(f"racking: seats at {' / '.join(f'{s:.2f}' for s in SEATS)} m, aisle {info['aisle']:.2f} m "
          f"(sheet Ast {truck.ast:.3f} m); put-away to level {info['level']} ({info['seat_put']:.2f} m), "
          f"retrieval from level 1 ({info['seat_pick']:.2f} m)")
    # What the cell asks of the truck, against what its package says.
    asked = scene.requirements()["agf"]
    print("the cell asks the truck: " + "; ".join(
        f"{r.key} {r.op} {r.value:g} ({r.status}, sheet {r.provided:g})" for r in asked.requirements))
    print(f"shift {tl.duration:.1f} s")
    for name, (t0, t1) in flows(tl).items():
        print(f"  {name}: {t0:6.1f} – {t1:6.1f} s  ({t1 - t0:.1f} s)")
    lanes = dict(tl.signals)
    driving = sum(t1 - t0 for t0, t1 in _high_spans(lanes["agf_base"], tl.duration))
    print(f"  driving {driving:.0f} s of {tl.duration:.0f} ({driving / tl.duration:.0%}); "
          f"lifting {sum(t1 - t0 for n, t0, t1 in tl.step_spans if any(k in n for k in ('seat_', 'lift_', 'raise_', 'land_', 'clear_', 'lower_'))):.0f} s")
    gap, at, other = tightest_pass(scene, tl, truck, "rack/B")
    print(f"  tightest pass to the racking across the aisle: {gap * 1e3:.0f} mm at {at:.1f} s ({other})")
    gap, at, other = tightest_pass(scene, tl, truck, "charger")
    print(f"  tightest pass to the charger: {gap * 1e3:.0f} mm at {at:.1f} s ({other})")
    # The pockets: what the geometry leaves the forks, in millimetres.
    print(f"  fork in the pocket: {(STRINGER - truck.h13) * 1e3:.1f} mm under the stringers lowered, "
          f"{(RELEASE_DROP) * 1e3:.0f} mm under them after a set-down with "
          f"{(STRINGER - RELEASE_DROP - truck.specs['fork_thickness_mm'] / 1000.0) * 1e3:.0f} mm over the beam")
    # (A link-to-obstacle minimum over the shift reads 0 mm: the fork top
    # under a pallet's stringers is contact by design. The driving margins
    # above are the machine's; the pocket margins are the geometry's.)
    end = tl.duration
    (ax, ay, az), _ = tl.object_pose("pallet/A/deck2", end)
    (bx, by, bz), _ = tl.object_pose("pallet/B/deck2", end)
    seat = info["seats"][("A", 1, info["level"])]
    print(f"  pallet A ends on bay 1 level {info['level']}: deck at z = {az - PALLET[2] + 0.011:.3f} m over the seat {seat[2]:.2f} m; "
          f"pallet B on the shipping spot at ({bx:.2f}, {by:.2f}), z = {bz - PALLET[2] + 0.011:.3f} m")


def deliver(scene: bt.Scene, tl, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    files = []

    def write(name, fn):
        path = out / name
        fn(path)
        files.append(path)

    write("forklift_layout.svg", lambda p: scene.export_layout(p, scale=40, title="無人フォークリフト 棚入れセル"))
    write("forklift_layout.dxf", lambda p: scene.export_layout(p, title="forklift"))
    write("forklift_bom.csv", scene.export_bom)
    write("forklift_bom.md", scene.export_bom)
    write("forklift_io.csv", scene.export_io_list)
    write("forklift.plcopen.xml", lambda p: scene.export_plcopen(p, name="forklift"))
    write("forklift_cell.usdc", lambda p: tl.export_usd(p, fps=15.0))
    rep = scene.cell_report(tl, deliverables=files, title="無人フォークリフト 棚入れセル")
    rep.save(out / "forklift_report.md")
    rep.save(out / "forklift_report.json")
    print(f"wrote the document set to {out}/ ({len(files) + 2} files)")


def main() -> None:
    global CATALOG_ROOT
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", nargs="?", default="forklift_cell.usdc", help="write the baked shift as USD here")
    parser.add_argument("--aisle", type=float, default=AISLE, help="rack face to rack face, metres (the sheet's Ast is 2.696)")
    parser.add_argument("--mast", choices=sorted(TRUCKS), default="tx", help="the truck's mast: tx (4.7 m) or dx (2.35 m)")
    parser.add_argument("--level", type=int, choices=range(1, RACK_LEVELS + 1), default=2, help="the beam level the received pallet goes to")
    parser.add_argument("--turn-radius", type=float, default=0.0, dest="turn_radius",
                        help="round every corner with an arc of this radius instead of pivoting (a counterbalance truck); 0 pivots")
    parser.add_argument("--sweep", action="store_true", help="bake the shift at several aisle widths and say which pass")
    parser.add_argument("--out", type=Path, default=None, help="write the document set here")
    parser.add_argument("--catalog-root", type=Path, default=None, help="a catalog builder's build/ directory")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()
    if args.catalog_root:
        CATALOG_ROOT = args.catalog_root.resolve()

    if args.sweep:
        truck = Truck(TRUCKS[args.mast])
        print(f"the sheet's Ast for a 1200 x 800 pallet, short side handling: {truck.ast:.3f} m")
        for aisle in ((2.4, 2.6, 2.8, 3.0) if args.turn_radius <= 0 else (2.6, 2.8, 3.0, 3.3)):
            try:
                _, _, _, tl = bake(aisle=aisle, mast=args.mast, level=args.level, turn_radius=args.turn_radius)
            except ValueError as err:
                print(f"  {aisle:.1f} m: refused — {str(err).splitlines()[0]}")
            else:
                print(f"  {aisle:.1f} m: passes ({tl.duration:.1f} s)")
        return

    try:
        scene, truck, info, tl = bake(aisle=args.aisle, mast=args.mast, level=args.level, turn_radius=args.turn_radius)
    except ValueError as err:
        print("refused, as it should be:")
        print(f"  {err}")
        return
    report(scene, truck, info, tl)
    tl.export_usd(args.recording, fps=15)
    print(f"wrote {args.recording}")
    if args.out:
        deliver(scene, tl, args.out)
    if args.studio:
        bt.studio(scene, view=STUDIO_VIEW)


if __name__ == "__main__":
    main()
