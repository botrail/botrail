"""A distribution warehouse from a consultation sketch: three AMR flows, one source.

The sketch is `warehouse-automation-layout.png` — the drawing a logistics
customer brings to a first automation meeting: a 40 m shed with inbound
docks on one side and outbound docks on the other, receiving inspection and
pallet racking, a case-picking area with two workbenches, packing and
weighing, shipping staging, an office, a charging corner. Three transport
flows are marked as the ones to automate, with two constraints written next
to them: *call-based* supply, and *aisles no narrower than 3.0 m*.

  * **① 入荷後の棚前搬送** — a received pallet, from the inspection area to
    the front of rack A.
  * **② 棚出し・作業台供給** — the picking station calls for the next pallet:
    the empty one goes to the pallet store, a picked one comes from rack B.
  * **③ 梱包後の出荷搬送** — the finished pallet, from packing to the shipping
    staging area.

Everything the flows need is ordered from the catalog rather than drawn:
two MiR1350 carriers wearing the MiR EU Pallet Lift, the Nord pallet racks
every hand-over happens on (the AMR drives under, lifts 60 mm, backs out),
TRUSCO 1-ton pallet racking, a MiR Charge 48V for each machine, a UR20 with
an OnRobot VGC10 doing the case picking beside a Makitech belt, TRUSCO
benches. The layout itself is derived from the sketch (see
`.internal/design-warehouse-amr.md` for the px → m reading and the one
deliberate departure: four single-deep rack rows with 3.0 m aisles, since
back-to-back pairs would contradict the customer's own aisle figure).

What the bake answers, in the order the customer asked:

  * **Does it fit the 3.0 m aisle?** The aisle check runs each carrier's
    body against every column, stand and wall on every tick it moves, and a
    pivot sweeps the body's half-diagonal. `--aisle 1.5` narrows the main
    aisle and the bake refuses by name — a column, at a time.
  * **Can two machines share it?** A vehicle that meets another while both
    drive is a hard error, so the fleet manager's traffic control has to be
    written down: each carrier *requests* the aisle and drives on a *grant*;
    a small arbiter program hands the aisle to one machine at a time. Run
    `--no-interlock` and the two meet head-on at a named time and place.
  * **What does call-based supply cost?** The picking station raises `call`
    when its pallet is empty; the chart shows how long it waits for the swap.
  * **Does it clear the existing equipment?** The finished pallet is taken
    out from between the packing bench and the belt: the clearance scan
    reports the tightest pass of the cycle.

Run with:  python examples/vehicles/warehouse_demo.py [--studio] [--out DIR]
                        [--aisle M] [--no-interlock] [--catalog-root DIR]

`--out DIR` writes the document set (layout SVG/DXF, BOM, I/O list,
interlock table, PLCopen, report, USD recording). `--catalog-root` points at
a catalog builder's `build/` directory instead of the published dataset.
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

import botrail as bt

HERE = Path(__file__).resolve().parent

# ---- what is ordered -------------------------------------------------
CARRIER = "mobile_industrial_robots/mir/mir1350/r1"          # 1350 kg pallet-class AMR
LIFT = "mobile_industrial_robots/mir/eu-pallet-lift-1350/r1"  # its EUR pallet lift, 60 mm
STAND = "nord_modules/pallet-rack/eu/r1"                     # the hand-over stand it docks under
CHARGER = "mobile_industrial_robots/mir/charge-48v/r1"
RACK = "trusco/pallet-rack/1d-2500x1100/r1"                  # 1-ton racking, 2500 clear bays
ARM = "universal_robots/ur/ur20/r2"                          # 20 kg at 1.75 m: a full EUR pallet
QC = "onrobot/quick-changer/109498/r1"                       # OnRobot tools mount through it
CUP = "onrobot/vgc/vgc10/r1"
BELT = "makitech/belgotch/type34-s1/r2"
BENCH = "trusco/ae/ae-1500/r1"
CONTROL_BOX_PACK = "universal_robots/control-box/e-series/r1"
SHELF = "botrail/rack/medium-shelf/r1"

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


# ---- the building, read off the sketch (37.5 px/m, origin at the SW corner)
BUILDING = (40.0, 22.5)
WALL_H, WALL_T = 7.0, 0.25
CUTAWAY = 1.0                    # the south and east walls are drawn this tall, so the shed reads from outside
COLUMN = 0.5
COLUMNS = ((9.0, 21.5), (23.4, 21.5), (31.3, 21.5), (23.4, 5.4))
INBOUND_DOCKS = (19.3, 15.85, 12.35)      # west wall, door centres (y)
OUTBOUND_DOCKS = (18.3, 14.8, 11.3)       # east wall
DOCK_DOOR = (3.0, 3.5)                    # width, clear height
FIRE_SHUTTER = (23.55, 3.3, 4.0)          # south wall: centre x, width, head
EMERGENCY_EXIT = (6.6, 1.0, 2.1)          # east wall: centre y, width, head
WORKER_ENTRANCE = (2.5, 1.0, 2.1)         # south wall, into the office

# The main aisle runs east-west between the office / charging / store fronts
# (y = AISLE_S) and the rack fronts. Its width is the customer's own figure;
# `--aisle` is what narrows it.
AISLE_S = 5.1
AISLE = 3.75
LANE_Y0 = 0.5                    # the aisle zone the arbiter guards
STAND_LEN, STAND_W = 1.30, 1.178   # Nord Pallet Rack EU envelope (the pack's numbers)
STAND_SUPPORT = 0.348
STAND_INNER = 1.0

RACK_X = (10.9, 15.0, 19.1, 23.2)   # four single-deep rows, 3.0 m aisles between
RACK_BAYS, RACK_H, RACK_LEVELS = 4, 4.0, 3
RECV = (1.0, 8.0, 10.5, 21.5)       # 入荷検品・仮置き: x0, x1, y0, y1
RECV_STAND = (4.5, 11.3)            # its outbound position: a stand at the south edge
PICKING = (24.3, 29.6, 10.5, 21.5)  # ケースピッキング
PICK_STAND = (27.4, 19.0)           # the supply position the cobot picks from
PICK_GATE_Y = None                  # filled from the lane
PICKER_BASE = (28.45, 19.3)
CONTROL_BOX = (29.2, 18.2)          # the UR control box, beside the riser
RISER = 0.75
BENCH2 = (25.0, 13.0)               # 作業台 2, manual
PACKING = (32.0, 38.9, 16.2, 21.5)  # 梱包・計量
BELT_X, BELT_Y, BELT_TOP, BELT_LEN, BELT_W = 29.4, 19.9, 0.75, 7.0, 0.4
PACKOUT_STAND = (33.6, 17.6)        # the finished pallet waits here, open to the west
PACK_BENCH = (37.2, 17.4)
SHIPPING = (32.0, 38.9, 10.0, 15.5) # 出荷仮置き, 4 x 4
SHIP_STAND = (33.4, 10.7)           # its AMR position, open to the west
SHIP_GRID_X = (34.8, 36.2, 37.6)
SHIP_GRID_Y = (10.7, 12.1, 13.5, 14.9)
OFFICE = (1.0, 7.4, 5.0)            # x0, x1, y1 (the perimeter wall closes the south)
STANDBY = (8.3, 15.3, 1.3, 5.1)     # AMR 待機・充電
CHARGER_XS = (10.0, 13.0)
CHARGER_Y = 2.4
MATERIALS = (30.3, 38.9, 1.3, 5.1)  # 資材・空パレット
EMPTY_STAND = (33.0, 3.6)           # empty pallets go here, open to the north
LANE1_X = 31.0                      # AMR-1's way up into the packing / shipping block

# ---- loads ----------------------------------------------------------
PALLET = (0.8, 1.2, 0.144)          # EUR pallet as it sits on a stand: 800 along the drive, 1200 across
CASE = (0.36, 0.28, 0.24)
CASE_KG = 5.0
SEAT = 0.005                        # everything set down stands proud of what it stands on
CASE_SLOTS = ((-0.19, -0.15), (-0.19, 0.15), (0.19, -0.15), (0.19, 0.15))  # in the pallet's frame
PITCH = 0.50                        # belt index per case
HOVER = 0.25
SPEED = 1.0                         # cruise, under the 1.2 m/s data sheet: a loaded machine among people
TURN = math.radians(45.0)
LIFT_UP, LIFT_DOWN = 4.0, 3.2       # the lift's own figures
READY = [0.0, -1.9, 1.9, -1.6, -math.pi / 2, 0.0]
CUPS = ["cup_a1", "cup_a2", "cup_b1", "cup_b2", "tcp"]

# Linear-RGB colours.
CONCRETE = (0.42, 0.42, 0.41)
LINE_YELLOW = (0.85, 0.62, 0.05)
LINE_GREEN = (0.10, 0.45, 0.20)
CARTON = (0.62, 0.45, 0.26)
WRAP = (0.78, 0.80, 0.84)
DOOR = (0.55, 0.57, 0.58)
STEEL_DARK = (0.22, 0.24, 0.27)


def yaw_quat(yaw: float) -> tuple:
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def down(yaw: float) -> tuple:
    """Tool +Z at the floor, jaws square to `yaw` — aimed along the reach so
    the wrist stays unwound."""
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)


def yaw_of(q) -> float:
    x, y, z, w = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def rotate(dx: float, dy: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return c * dx - s * dy, s * dx + c * dy


def mul_quat(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


# ================================================================== the machine
class PalletAmr:
    """A carrier and its pallet lift, measured from their packages.

    The cell asks four things of the machine and reads all four out of the
    catalog: the body that has to clear the aisle (the carrier's collision
    boxes), the plane the lift bolts to (`frames.flange_frame`), how fast it
    may run (`specs`), and what the lift's seat is (`pallet`). Nothing below
    knows it is a MiR.
    """

    def __init__(self, carrier: str = CARRIER, lift: str = LIFT):
        self.spec = manifest(carrier)
        self.lift_spec = manifest(lift)
        self.id, self.product = self.spec["id"], self.spec["name"]
        self.maker = self.spec["manufacturer"]["name"]
        self.specs = self.spec["specs"]
        self.model = load(carrier)
        self.lift = load(lift)
        root = package_dir(carrier)
        self.visual_urdf = root / self.spec["assets"]["urdf"]
        if self.model.flange_link is None:
            raise ValueError(f"{self.id} declares no mount frame to bolt a top module to")
        probe = bt.Scene(self.model)
        self.deck = probe.link_pose(self.model.flange_link)[0][2]
        # The body: every collision primitive of every link, posed by FK.
        # The top cover is a link of its own and comes off with the top
        # module, exactly as on the machine.
        self.boxes: list[tuple[str, tuple, tuple, tuple]] = []
        for link in ET.parse(self.visual_urdf).getroot().findall("link"):
            name = link.get("name")
            if name == "top_cover":
                continue
            for collision in link.findall("collision"):
                box = collision.find("geometry/box")
                if box is None:
                    continue
                origin = collision.find("origin")
                xyz = tuple(float(v) for v in (origin.get("xyz", "0 0 0") if origin is not None else "0 0 0").split())
                size = tuple(float(v) for v in box.get("size").split())
                (px, py, pz), q = probe.link_pose(name)
                self.boxes.append((name, size, (px + xyz[0], py + xyz[1], pz + xyz[2]), q))
        if not self.boxes:
            raise ValueError(f"{self.id}: no collision boxes to drive the aisle with")
        self.length, self.width = (v / 1000.0 for v in self.specs["footprint_mm"])
        lift_probe = bt.Scene(self.lift)
        self.seat = lift_probe.link_pose(self.lift.flange_link)[0][2]   # rail top over the mount plane
        self.stroke = float(self.lift_spec["specs"]["stroke_mm"]) / 1000.0
        self.lift_joint = self.lift.joint_names[0]

    @property
    def swing(self) -> float:
        """What a pivot sweeps: the body's half-diagonal."""
        return math.hypot(self.length, self.width) / 2

    @property
    def lowered(self) -> float:
        """Rail top over the floor with the lift down."""
        return self.deck + self.seat

    def add(self, scene: bt.Scene, name: str, at: tuple[float, float], heading: float) -> str:
        """Body, visuals and the lift, parked at `at` facing `heading`.
        Returns the lift robot's name."""
        x, y = at
        q = yaw_quat(heading)
        for link, size, (px, py, pz), lq in self.boxes:
            rx, ry = rotate(px, py, heading)
            made = scene.add_box(f"{name}/{link}", size, (x + rx, y + ry, pz), quaternion=mul_quat(q, lq),
                                 color=STEEL_DARK)
            scene.set_obstacle_visible(made, False)
        visuals = scene.load_urdf(self.visual_urdf, prefix=f"{name}/visual", position=(x, y, 0.0),
                                  quaternion=q, frames=False)
        for piece in visuals:
            scene.set_obstacle_enabled(piece, False)
            if "/top_cover" in piece:
                scene.set_obstacle_visible(piece, False)   # it comes off for the lift
        lift = f"{name}_lift"
        scene.add_robot(self.lift, name=lift)
        return lift

    def mount(self, scene: bt.Scene, name: str, lift: str) -> None:
        scene.mount_robot(name, robot=lift, offset_position=(0.0, 0.0, self.deck))
        # The module bolts onto the chassis it stands on: declared, so the
        # contact never reads as the lift touching its own carrier.
        for link, _, _, _ in self.boxes:
            scene.allow_link_obstacle_contact(self.lift.link_names[1], f"{name}/{link}", robot=lift)
        for wheel, radius in (("left_wheel_link", self.specs["wheel_diameter_mm"] / 2000.0),
                              ("right_wheel_link", self.specs["wheel_diameter_mm"] / 2000.0)):
            scene.set_vehicle_wheel(name, f"{name}/visual/{wheel}", radius=radius, axis=(0.0, 1.0, 0.0))
        for tag in ("fl", "fr", "bl", "br"):
            scene.set_vehicle_wheel(name, f"{name}/visual/{tag}_caster_wheel_link",
                                    radius=self.specs["caster_diameter_mm"] / 2000.0, axis=(0.0, 1.0, 0.0))
        scene.set_part(name, kind="device", catalog=self.id, manufacturer=self.maker,
                       model=self.product, category="vehicle.amr",
                       payload_kg=self.specs["payload_kg"], max_speed_mps=self.specs["max_speed_mps"])


# ================================================================== the shed
def _slab(scene: bt.Scene) -> None:
    w, d = BUILDING
    made = scene.add_box("floor/slab", (w, d, 0.05), (w / 2, d / 2, -0.022), color=CONCRETE)
    scene.set_obstacle_enabled(made, False)
    scene.set_obstacle_material(made, metalness=0.0, roughness=0.9)


def _mark(scene: bt.Scene, name: str, x0: float, y0: float, x1: float, y1: float,
          color=LINE_YELLOW, width: float = 0.08) -> None:
    """A painted rectangle: four strips under `ground_z`, never colliding."""
    z = 0.0045
    for i, (size, pos) in enumerate((
        ((x1 - x0, width, 0.003), ((x0 + x1) / 2, y0, z)),
        ((x1 - x0, width, 0.003), ((x0 + x1) / 2, y1, z)),
        ((width, y1 - y0, 0.003), (x0, (y0 + y1) / 2, z)),
        ((width, y1 - y0, 0.003), (x1, (y0 + y1) / 2, z)),
    )):
        made = scene.add_box(f"marking/{name}/{i}", size, pos, color=color)
        scene.set_obstacle_enabled(made, False)


def _line(scene: bt.Scene, name: str, a: tuple, b: tuple, color=LINE_YELLOW, width: float = 0.08) -> None:
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    yaw = math.atan2(b[1] - a[1], b[0] - a[0])
    made = scene.add_box(f"marking/{name}", (length, width, 0.003),
                         ((a[0] + b[0]) / 2, (a[1] + b[1]) / 2, 0.0045), quaternion=yaw_quat(yaw), color=color)
    scene.set_obstacle_enabled(made, False)


def _cutaway(scene: bt.Scene, name: str, path, openings=()) -> None:
    """A perimeter wall drawn low so the shed reads from outside: the real
    7 m wall collides but is hidden; a 1 m curb shows where it stands."""
    full = bt.parts.wall(scene, name, path=path, height=WALL_H, thickness=WALL_T, openings=list(openings))
    for piece in full.obstacles:
        scene.set_obstacle_visible(piece, False)
    curb = bt.parts.wall(scene, f"{name}_curb", path=path, height=CUTAWAY, thickness=WALL_T,
                         openings=[(e, c, w) for e, c, w, *_ in openings])
    for piece in curb.obstacles:
        scene.set_obstacle_enabled(piece, False)


def building(scene: bt.Scene) -> None:
    w, d = BUILDING
    _slab(scene)
    # West wall with the three inbound docks, north wall, and the two walls
    # nearest the camera cut away.
    bt.parts.wall(scene, "wall/west", path=[(0.0, 0.0), (0.0, d)], height=WALL_H, thickness=WALL_T,
                  openings=[(0, y, DOCK_DOOR[0], DOCK_DOOR[1]) for y in INBOUND_DOCKS])
    bt.parts.wall(scene, "wall/north", path=[(0.0, d), (w, d)], height=WALL_H, thickness=WALL_T)
    _cutaway(scene, "wall/east", [(w, d), (w, 0.0)],
             [(0, d - y, DOCK_DOOR[0], DOCK_DOOR[1]) for y in OUTBOUND_DOCKS]
             + [(0, d - EMERGENCY_EXIT[0], EMERGENCY_EXIT[1], EMERGENCY_EXIT[2])])
    _cutaway(scene, "wall/south", [(w, 0.0), (0.0, 0.0)],
             [(0, w - FIRE_SHUTTER[0], FIRE_SHUTTER[1], FIRE_SHUTTER[2]),
              (0, w - WORKER_ENTRANCE[0], WORKER_ENTRANCE[1], WORKER_ENTRANCE[2])])
    # Dock doors, closed: a leaf in each opening.
    for i, y in enumerate(INBOUND_DOCKS):
        scene.add_box(f"dock/in{i + 1}/door", (0.06, DOCK_DOOR[0], DOCK_DOOR[1]),
                      (0.0, y, DOCK_DOOR[1] / 2), color=DOOR)
    for i, y in enumerate(OUTBOUND_DOCKS):
        scene.add_box(f"dock/out{i + 1}/door", (0.06, DOCK_DOOR[0], DOCK_DOOR[1]),
                      (w, y, DOCK_DOOR[1] / 2), color=DOOR)
    scene.set_part("dock", kind="group", category="structure.door", qty=6, model="Sectional dock door 3000x3500",
                   manufacturer="Generic")
    # The fire shutter rolled up in its box over the opening; the emergency exit leaf.
    scene.add_box("shutter/box", (FIRE_SHUTTER[1] + 0.3, 0.45, 0.45), (FIRE_SHUTTER[0], WALL_T / 2 + 0.225, FIRE_SHUTTER[2] + 0.225),
                  color=STEEL_DARK)
    scene.set_part("shutter", kind="group", category="structure.door", qty=1, model="Fire shutter 3300x4000",
                   manufacturer="Generic")
    scene.add_box("exit/door", (0.05, EMERGENCY_EXIT[1], EMERGENCY_EXIT[2]),
                  (w, EMERGENCY_EXIT[0], EMERGENCY_EXIT[2] / 2), color=(0.55, 0.12, 0.10))
    for i, (x, y) in enumerate(COLUMNS):
        scene.add_box(f"column/{i}", (COLUMN, COLUMN, WALL_H), (x, y, WALL_H / 2), color=CONCRETE)
    scene.set_part("column", kind="group", category="structure.column", qty=len(COLUMNS),
                   model="RC column 500x500", manufacturer="Building")
    # The office: three partition walls closed by the perimeter, a door to the floor.
    x0, x1, y1 = OFFICE
    bt.parts.wall(scene, "office", path=[(x0, WALL_T / 2), (x0, y1), (x1, y1), (x1, WALL_T / 2)],
                  height=3.0, thickness=0.12, openings=[(1, x1 - x0 - 1.0, 0.9)], detail="full")


def markings(scene: bt.Scene, lane: float, stand_y: float) -> None:
    _mark(scene, "recv", *RECV, color=LINE_GREEN)
    _mark(scene, "picking", *PICKING, color=LINE_GREEN)
    _mark(scene, "packing", *PACKING, color=LINE_GREEN)
    _mark(scene, "shipping", *SHIPPING, color=LINE_GREEN)
    _mark(scene, "standby", *STANDBY, color=LINE_GREEN)
    _mark(scene, "materials", *MATERIALS)
    # The receiving floor grid: 4 x 5 pallet positions, the sketch's "最大 20PL".
    x0, x1, y0, y1 = RECV
    for i in range(1, 4):
        _line(scene, f"recv_v{i}", (x0 + (x1 - x0) * i / 4, y0), (x0 + (x1 - x0) * i / 4, y1), width=0.05)
    for j in range(1, 5):
        _line(scene, f"recv_h{j}", (x0, y0 + (y1 - y0) * j / 5), (x1, y0 + (y1 - y0) * j / 5), width=0.05)
    # The AMR lane down the main aisle and its branches, the sketch's dashed line.
    _line(scene, "lane", (2.0, lane), (38.0, lane), width=0.06)
    for x in (RECV_STAND[0], *RACK_X, PICK_STAND[0], LANE1_X, EMPTY_STAND[0], *CHARGER_XS):
        top = stand_y if x in RACK_X else lane
        _line(scene, f"branch_{x:.1f}", (x, min(lane, top)), (x, max(lane, top) + 0.01), width=0.06)


# ================================================================== the loads
def unit_load(scene: bt.Scene, name: str, at: tuple, yaw: float, height: float = 0.9,
              z0: float = 0.0, collide: bool = True) -> None:
    """A stored pallet, as scenery: a pallet slab and a wrapped stack on it —
    two boxes, because a warehouse has hundreds of these and the aisle check
    visits every enabled one."""
    x, y = at
    q = yaw_quat(yaw)
    slab = scene.add_box(f"{name}/pallet", (PALLET[0], PALLET[1], PALLET[2]), (x, y, z0 + PALLET[2] / 2),
                         quaternion=q, color=bt.parts.WOOD)
    stack = scene.add_box(f"{name}/load", (PALLET[0] - 0.04, PALLET[1] - 0.04, height),
                          (x, y, z0 + PALLET[2] + height / 2), quaternion=q, color=WRAP)
    for piece in (slab, stack):
        scene.set_obstacle_enabled(piece, collide)


def pallet_with_cases(scene: bt.Scene, name: str, at: tuple, yaw: float, courses: int,
                      z0: float, slots=CASE_SLOTS) -> list[str]:
    """An EUR pallet with `courses` layers of four cases, floating `SEAT`
    above what they stand on. Returns the case names, bottom course first."""
    x, y = at
    bt.parts.pallet(scene, name, (x, y, z0), size=PALLET, yaw=yaw)
    cases = []
    q = yaw_quat(yaw)
    for course in range(courses):
        for k, (lx, ly) in enumerate(slots):
            dx, dy = rotate(lx, ly, yaw)
            case = f"{name}/case{course}{k}"
            scene.add_box(case, CASE, (x + dx, y + dy, z0 + PALLET[2] + SEAT + course * (CASE[2] + SEAT) + CASE[2] / 2),
                          quaternion=q, color=CARTON)
            scene.set_part(case, category="workpiece", model="RSC-360x280x240", mass_kg=CASE_KG)
            cases.append(case)
    return cases


def stand(scene: bt.Scene, name: str, at: tuple, yaw: float) -> str:
    """A Nord pallet rack at `at`, its open end facing `yaw`."""
    bt.parts.pallet_stand(scene, name, at, yaw=yaw, catalog=ref(STAND))
    return name


# ================================================================== the areas
def receiving(scene: bt.Scene) -> list[str]:
    x0, x1, y0, y1 = RECV
    xs = [x0 + (x1 - x0) * (i + 0.5) / 4 for i in range(4)]
    ys = [y0 + (y1 - y0) * (j + 0.5) / 5 for j in range(5)]
    # Received pallets on the floor grid, minus the row the stand takes.
    filled = ((0, 1), (1, 1), (2, 1), (0, 2), (2, 2), (3, 2), (1, 3), (3, 3), (0, 4), (2, 4))
    for i, j in filled:
        unit_load(scene, f"recv/pl{i}{j}", (xs[i], ys[j]), 0.0, height=0.7 + 0.15 * ((i + j) % 3))
    # The outbound position: a stand, open to the aisle, with today's pallet on it.
    stand(scene, "stand/recv", RECV_STAND, -math.pi / 2)
    return pallet_with_cases(scene, "p1", RECV_STAND, -math.pi / 2, courses=1, z0=STAND_SUPPORT)


def racking(scene: bt.Scene, stand_y: float, rack_y0: float) -> None:
    run = RACK_BAYS * (2.5 + 0.09) + 0.09
    for row, x in enumerate(RACK_X):
        tag = "ABCD"[row]
        bt.parts.pallet_rack(scene, f"rack/{tag}", (x, rack_y0 + run / 2), yaw=math.pi / 2, bays=RACK_BAYS,
                             height=RACK_H, levels=RACK_LEVELS, catalog=ref(RACK),
                             detail="full" if row == 0 else "plain")
        # Stock: two EUR pallets a bay, 1200 to the aisle. Elevated loads are
        # drawn only — nothing drives up there.
        for bay in range(RACK_BAYS):
            for level in range(RACK_LEVELS + 1):
                seat, _ = scene.frame(f"rack/{tag}/bay{bay}/level{level}")
                for pos, off in enumerate((-0.64, 0.64)):
                    if (row + 2 * bay + 3 * level + pos) % 3 == 0:
                        continue
                    unit_load(scene, f"rack/{tag}/b{bay}l{level}p{pos}", (seat[0], seat[1] + off), math.pi / 2,
                              height=0.55 + 0.1 * ((bay + level + pos) % 4), z0=seat[2], collide=level == 0)
        # The rack-front P&D position: a stand on the row's axis, open to the aisle.
        stand(scene, f"stand/pd{tag}", (x, stand_y), -math.pi / 2)


def picking_station(scene: bt.Scene) -> tuple[bt.Robot, list[str]]:
    """The cobot cell at 作業台 1: the supply stand, the arm on its riser, the
    belt it feeds. 作業台 2 stays a bench for people."""
    stand(scene, "stand/pick", PICK_STAND, -math.pi / 2)
    riser = bt.parts.pedestal(scene, "riser", height=RISER, position=PICKER_BASE, model="PD-350",
                              manufacturer="Generic", mass_kg=38)
    scene.set_obstacle_enabled(f"{riser.name}/column", False)
    arm = load(ARM).attach_tool(load(QC), prefix="qc_").attach_tool(load(CUP))
    scene.add_robot(arm, name="picker", base_position=(*PICKER_BASE, RISER + SEAT))
    scene.set_joint_positions(READY, robot="picker")
    # The arm's control box, on the floor beside the riser — the line the
    # bill derives for every arm, placed so it can be reached and wired.
    bt.parts.controller(scene, "picker_box", robots=["picker"], position=CONTROL_BOX, yaw=math.pi,
                        catalog=ref(CONTROL_BOX_PACK))
    bt.parts.conveyor(scene, "pack_line", catalog=ref(BELT), length=BELT_LEN, width=BELT_W,
                      position=(BELT_X + BELT_LEN / 2, BELT_Y, BELT_TOP), direction=(1.0, 0.0), speed=0.3)
    bt.parts.table(scene, "bench2", catalog=ref(BENCH), position=BENCH2, yaw=math.pi / 2)
    cases = pallet_with_cases(scene, "p2", PICK_STAND, -math.pi / 2, courses=1, z0=STAND_SUPPORT)
    return arm, cases


def packing(scene: bt.Scene) -> list[str]:
    """梱包・計量: the belt ends here; the scale and the labeler sit on it,
    the finished pallet waits on its stand for flow ③."""
    scene.add_box("scale/platform", (0.6, BELT_W + 0.2, 0.12), (BELT_X + 3.0, BELT_Y, 0.06), color=STEEL_DARK)
    scene.add_box("scale/indicator", (0.25, 0.08, 0.16), (BELT_X + 3.0, BELT_Y + BELT_W / 2 + 0.2, BELT_TOP + 0.4),
                  color=(0.75, 0.75, 0.72))
    scene.add_box("scale/post", (0.04, 0.04, BELT_TOP + 0.32), (BELT_X + 3.0, BELT_Y + BELT_W / 2 + 0.2, (BELT_TOP + 0.32) / 2),
                  color=STEEL_DARK)
    scene.set_part("scale", kind="group", category="instrument.scale", qty=1, model="In-line checkweigher 60 kg",
                   manufacturer="Generic")
    scene.add_box("labeler/stand", (0.5, 0.4, 0.9), (BELT_X + 4.5, BELT_Y + BELT_W / 2 + 0.45, 0.45), color=STEEL_DARK)
    scene.add_box("labeler/head", (0.45, 0.35, 0.35), (BELT_X + 4.5, BELT_Y + BELT_W / 2 + 0.45, 1.08), color=(0.72, 0.72, 0.70))
    scene.add_box("labeler/applicator", (0.3, 0.30, 0.08), (BELT_X + 4.5, BELT_Y, BELT_TOP + 0.55), color=(0.72, 0.72, 0.70))
    scene.set_part("labeler", kind="group", category="labeler.print_apply", qty=1, model="Print & apply labeler",
                   manufacturer="Generic")
    scene.add_zone_sensor("weighing", position=(BELT_X + 3.0, BELT_Y, BELT_TOP + 0.2), size=(0.5, BELT_W, 0.4))
    bt.parts.table(scene, "pack_bench", catalog=ref(BENCH), position=PACK_BENCH)
    stand(scene, "stand/packout", PACKOUT_STAND, math.pi)
    slots = ((-0.19, -0.15), (-0.19, 0.15), (0.19, -0.15), (0.19, 0.15))
    cases = pallet_with_cases(scene, "p4", PACKOUT_STAND, math.pi, courses=1, z0=STAND_SUPPORT, slots=slots)
    # a second, partial course — the last cases packed
    for k, (lx, ly) in enumerate(((-0.19, -0.15), (-0.19, 0.15))):
        dx, dy = rotate(lx, ly, math.pi)
        case = f"p4/case1{k}"
        scene.add_box(case, CASE, (PACKOUT_STAND[0] + dx, PACKOUT_STAND[1] + dy,
                                   STAND_SUPPORT + PALLET[2] + SEAT + (CASE[2] + SEAT) + CASE[2] / 2),
                      quaternion=yaw_quat(math.pi), color=CARTON)
        scene.set_part(case, category="workpiece", model="RSC-360x280x240", mass_kg=CASE_KG)
        cases.append(case)
    return cases


def shipping(scene: bt.Scene) -> None:
    stand(scene, "stand/ship", SHIP_STAND, math.pi)
    for i, x in enumerate(SHIP_GRID_X):
        for j, y in enumerate(SHIP_GRID_Y):
            if (i + j) % 4 == 3:
                continue
            unit_load(scene, f"ship/pl{i}{j}", (x, y), math.pi, height=0.9 + 0.12 * ((i + j) % 3))


def standby(scene: bt.Scene) -> list[tuple[float, float]]:
    """Two chargers, plate to the aisle: the machines back onto them. Returns
    where each machine's body centre parks."""
    parks = []
    for i, x in enumerate(CHARGER_XS):
        built = bt.parts.charging_station(scene, f"charger/{i + 1}", (x, CHARGER_Y), yaw=math.pi / 2,
                                          catalog=ref(CHARGER))
        dock, _ = scene.frame(built.frames[0])
        parks.append((dock[0], dock[1]))
    return parks


def materials(scene: bt.Scene) -> None:
    stand(scene, "stand/empty", EMPTY_STAND, math.pi / 2)
    for i in range(2):
        bt.parts.rack(scene, f"materials/shelf{i}", position=(36.6 + 1.3 * i, 2.0), catalog=ref(SHELF), levels=4,
                      width_mm=1200, depth_mm=600, height_mm=2100)
    # Empty pallets, stacked five high.
    for k in range(5):
        scene.add_box(f"materials/empties/{k}", PALLET, (35.2, 3.6, PALLET[2] * (k + 0.5)),
                      quaternion=yaw_quat(math.pi / 2), color=bt.parts.WOOD)
    scene.set_part("materials/empties", kind="group", category="pallet", qty=5, model="EUR pallet",
                   manufacturer="EPAL")


# ================================================================== the routes
def routes(scene: bt.Scene, machine: PalletAmr, parks, lane: float, stand_y: float) -> dict:
    """Both machines' paths, stations and the handshake sensors."""
    pick = PICK_STAND
    charger1, charger2 = ((x, y + machine.length / 2) for x, y in parks)   # contacts at the back, plate to the south
    path1 = [
        charger1, (charger1[0], lane), (RECV_STAND[0], lane), RECV_STAND,                    # 0-3  to the receiving stand
        (RECV_STAND[0], lane), (RACK_X[0], lane), (RACK_X[0], stand_y),                       # 4-6  rack A front
        (RACK_X[0], lane), (LANE1_X, lane), (LANE1_X, PACKOUT_STAND[1]), PACKOUT_STAND,     # 7-10 the finished pallet
        (LANE1_X, PACKOUT_STAND[1]), (LANE1_X, SHIP_STAND[1]), SHIP_STAND,                   # 11-13 shipping staging
        (LANE1_X, SHIP_STAND[1]), (LANE1_X, lane), (charger1[0], lane), (charger1[0], lane + 0.6), charger1,  # 14-18 home, backing on
    ]
    stations1 = {"charger": 0, "recv": 3, "pdA": 6, "packout": 10, "ship": 13, "home": 18}
    path2 = [
        charger2, (charger2[0], lane), (pick[0], lane), pick,                                 # 0-3  the picking station
        (pick[0], lane), (EMPTY_STAND[0], lane), EMPTY_STAND,                                 # 4-6  the empty pallet store
        (EMPTY_STAND[0], lane), (RACK_X[1], lane), (RACK_X[1], stand_y),                      # 7-9  rack B front
        (RACK_X[1], lane), (pick[0], lane), pick,                                             # 10-12 back to picking
        (pick[0], lane), (charger2[0], lane), (charger2[0], lane + 0.6), charger2,            # 13-16 home
    ]
    stations2 = {"charger": 0, "pick": 3, "empty": 6, "pdB": 9, "pick2": 12, "home": 16}
    lifts = {}
    for name, path, stations, park in (("amr1", path1, stations1, charger1), ("amr2", path2, stations2, charger2)):
        lifts[name] = machine.add(scene, name, park, math.pi / 2)
        scene.add_vehicle(name, body=[name], path=path, stations=stations, speed=SPEED, turn_speed=TURN,
                          start="charger", allow_reverse=True)
        machine.mount(scene, name, lifts[name])
    # The aisle the arbiter guards, and who is in it.
    w = BUILDING[0]
    for name in ("amr1", "amr2"):
        # A strip at floor level: it only has to overlap the chassis's
        # underside to see the machine, and drawn low it stays out of the
        # picture (the studio draws every zone as a translucent box).
        scene.add_zone_sensor(f"aisle_{name}", position=(w / 2, AISLE_S + AISLE / 2, 0.03),
                              size=(w - 2 * LANE_Y0, AISLE, 0.06), watch=[f"{name}/base_link"])
    # The picking station's handshake: the machine in the stand, the arm over it.
    scene.add_zone_sensor("pick_amr", position=(pick[0], pick[1], 0.03), size=(2.2, 2.6, 0.06), watch=["amr2/base_link"])
    # The arm's exclusion volume starts above a docked machine and its
    # load: a wrist over the stand is what the machine must wait for.
    scene.add_zone_sensor("pick_arm", position=(pick[0], pick[1], 1.7), size=(1.7, 2.6, 1.8), watch_robots=["picker"])
    return lifts


# ================================================================== teaching
def teach(scene: bt.Scene, cases: list[str]) -> None:
    """The picker's poses, solved from the machine: a hover and a seat over
    every case slot, the belt drop, and a parked pose out of the stand."""
    limits = scene.robot_of("picker").joint_limits

    def unwind(q, seed):
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

    def seed_for(target, lift: float, elbow: float) -> list:
        """An elbow-up guess aimed at the target: the pan toward it, the
        upper arm raised by `lift`, the forearm folded down by `elbow`, and
        the wrist bent so the tool already points at the floor. Numerical IK
        started from the ready pose wanders into the elbow-down branch
        here, which sweeps the forearm through the pallet; started from
        this it lands where a palletizer's arm actually is."""
        pan = math.atan2(target[1] - PICKER_BASE[1], target[0] - PICKER_BASE[0])
        return [pan, lift, elbow, -(lift + elbow) - math.pi / 2, -math.pi / 2, 0.0]

    def pose(name, target, *seeds):
        reach = math.atan2(target[1] - PICKER_BASE[1], target[0] - PICKER_BASE[0])
        seeds = (*seeds, seed_for(target, -1.2, 1.6), seed_for(target, -0.8, 1.9), READY)
        short, fouled = None, []
        for seed in seeds:
            scene.set_joint_positions(seed, robot="picker")
            ik = scene.set_tcp_target(target, down(reach), robot="picker")
            if not ik.converged:
                short = ik.pos_error
                continue
            q = unwind(list(scene.joint_positions_of("picker")), seed)
            scene.set_joint_positions(q, robot="picker")
            hits = [f"{a[1]} x {b[1]}" for a, b in scene.check_collisions()]
            if not hits:
                scene.add_segment(name, goal=q, robot="picker")
                return q
            fouled = hits
        if short is not None and not fouled:
            raise RuntimeError(f"{ARM} cannot reach {tuple(round(v, 2) for v in target)} from "
                               f"{PICKER_BASE} on a {RISER:.2f} m riser — {short * 1e3:.0f} mm short")
        raise RuntimeError(f"every branch taught for `{name}` fouls: {', '.join(fouled)}")

    drop = (BELT_X + 0.3, BELT_Y, BELT_TOP + SEAT + CASE[2])
    belt_hi = pose("belt_hi", (drop[0], drop[1], drop[2] + HOVER))
    pose("belt_lo", drop, belt_hi)
    for k, case in enumerate(cases):
        lo, hi = scene.obstacle_bounds(case)
        top = ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, hi[2])
        over = pose(f"pick{k}_hi", (top[0], top[1], top[2] + HOVER))
        pose(f"pick{k}_lo", top, over)
    pose("parked", (PICKER_BASE[0] + 0.55, PICKER_BASE[1] + 0.9, RISER + 1.1), belt_hi)
    scene.add_segment("ready", goal=READY, robot="picker")
    scene.set_joint_positions(READY, robot="picker")


# ================================================================== programs
def group_of(scene: bt.Scene, prefix: str) -> list[str]:
    return [n for n in scene.obstacle_names if n == prefix or n.startswith(prefix + "/")]


def programs(scene: bt.Scene, machine: PalletAmr, lifts: dict, loads: dict, interlock: bool) -> list[str]:
    """The three flows, the picker, and the traffic control between them."""
    for sig in ("req_amr1", "grant_amr1", "req_amr2", "grant_amr2", "call", "supply_done", "vacuum"):
        scene.define_signal(sig, initial=(not interlock) and sig.startswith("grant"))

    def lift_steps(sq, name: str, pallet: str, up: bool, cases: bool = True) -> None:
        """Take a pallet: attach it — and the cases on it — to the lift and
        raise; or set it down: lower and let go. A ramp is legal while
        parked or driving, which is why the lift needs no plan. `cases=False`
        takes the bare pallet: what is on the belt by then is not cargo."""
        lift, joint = lifts[name], machine.lift_joint
        pieces = [n for n in group_of(scene, pallet) if cases or "/case" not in n]
        if up:
            sq.step(f"hold_{pallet}", actions=[bt.seq.attach(p, link="pallet", robot=lift) for p in pieces],
                    transition=bt.seq.immediately())
            sq.step(f"lift_{pallet}", actions=[bt.seq.ramp({joint: machine.stroke}, LIFT_UP, robot=lift)])
        else:
            sq.step(f"lower_{pallet}", actions=[bt.seq.ramp({joint: 0.0}, LIFT_DOWN, robot=lift)])
            sq.step(f"release_{pallet}", actions=[bt.seq.detach(p) for p in pieces], transition=bt.seq.immediately())

    aisle_legs: list[str] = []

    def drive(sq, name: str, station: str, aisle: bool) -> None:
        """A leg. Through the main aisle it is requested from the traffic
        control and driven on the grant; the request drops on arrival."""
        if aisle:
            aisle_legs.append(name)
            sq.step(f"request_{station}", actions=[bt.seq.set_signal(f"req_{name}")],
                    transition=bt.seq.signal(f"grant_{name}"))
        sq.step(f"to_{station}", actions=[bt.seq.goto(name, station)], transition=bt.seq.device_done(name))
        if aisle:
            sq.step(f"at_{station}", actions=[bt.seq.set_signal(f"req_{name}", False)],
                    transition=bt.seq.elapsed(0.05))

    # ---- ① and ③: AMR-1 -------------------------------------------------
    rx = scene.sequence("receiving")
    drive(rx, "amr1", "recv", aisle=True)
    lift_steps(rx, "amr1", "p1", up=True)
    drive(rx, "amr1", "pdA", aisle=True)
    lift_steps(rx, "amr1", "p1", up=False)
    drive(rx, "amr1", "packout", aisle=True)
    lift_steps(rx, "amr1", "p4", up=True)
    drive(rx, "amr1", "ship", aisle=False)
    lift_steps(rx, "amr1", "p4", up=False)
    drive(rx, "amr1", "home", aisle=True)

    # ---- the picker: four cases, the call, four more ------------------
    pk = scene.sequence("picking")

    def pick(k: int, case: str) -> None:
        pk.step(f"clear{k}", transition=bt.seq.signal("pick_amr", False))
        pk.step(f"reach{k}", actions=[bt.seq.motion(f"pick{k}_hi")])
        pk.step(f"descend{k}", actions=[bt.seq.motion(f"pick{k}_lo")])
        pk.step(f"grip{k}", actions=[bt.seq.attach(case, link="tcp", touch_links=CUPS, robot="picker"),
                                     bt.seq.set_signal("vacuum", True)], transition=bt.seq.elapsed(0.3))
        pk.step(f"lift{k}", actions=[bt.seq.motion(f"pick{k}_hi")])
        pk.step(f"swing{k}", actions=[bt.seq.motion("belt_hi")])
        pk.step(f"place{k}", actions=[bt.seq.motion("belt_lo")])
        pk.step(f"release{k}", actions=[bt.seq.detach(case), bt.seq.set_signal("vacuum", False)],
                transition=bt.seq.elapsed(0.3))
        pk.step(f"clear_belt{k}", actions=[bt.seq.motion("belt_hi")])
        pk.step(f"index{k}", actions=[bt.seq.advance("pack_line", PITCH)],
                transition=bt.seq.device_done("pack_line"))

    for k, case in enumerate(loads["p2"]):
        pick(k, case)
    # The pallet is empty: call for the next one and get out of the way.
    pk.step("call", actions=[bt.seq.set_signal("call")], transition=bt.seq.immediately())
    pk.step("park", actions=[bt.seq.motion("parked")])
    pk.step("await_supply", transition=bt.seq.signal("supply_done"))
    for k, case in enumerate(loads["p3"]):
        pick(k, case)
    pk.step("home", actions=[bt.seq.motion("ready")])

    # ---- ②: AMR-2, on call ------------------------------------------------
    sp = scene.sequence("supply")
    sp.step("standby", transition=bt.seq.signal("call"))
    # Dispatched only once the arm has parked clear of the stand: a machine
    # that waited for it *in* the aisle would hold the aisle for nothing.
    sp.step("station_ready", transition=bt.seq.signal("pick_arm", False))
    drive(sp, "amr2", "pick", aisle=True)
    lift_steps(sp, "amr2", "p2", up=True, cases=False)   # the picker emptied it
    drive(sp, "amr2", "empty", aisle=True)
    lift_steps(sp, "amr2", "p2", up=False, cases=False)
    drive(sp, "amr2", "pdB", aisle=True)
    lift_steps(sp, "amr2", "p3", up=True)
    drive(sp, "amr2", "pick2", aisle=True)
    lift_steps(sp, "amr2", "p3", up=False)
    sp.step("supplied", actions=[bt.seq.set_signal("supply_done")], transition=bt.seq.immediately())
    drive(sp, "amr2", "home", aisle=True)

    names = ["receiving", "picking", "supply"]
    if interlock:
        # ---- the traffic control: one machine in the aisle at a time ------
        # Each round waits for a request and grants it — AMR-1 first when
        # both ask in the same scan — then holds until the request drops.
        # One round per aisle leg authored above, so the arbiter ends with
        # the shift instead of waiting for a call that never comes.
        tc = scene.sequence("traffic")
        for k in range(len(aisle_legs)):
            sel = tc.select(f"round{k}")
            for name in ("amr1", "amr2"):
                arm = sel.when(bt.seq.signal(f"req_{name}"))
                arm.step(f"grant{k}_{name}", actions=[bt.seq.set_signal(f"grant_{name}")],
                         transition=bt.seq.signal(f"req_{name}", False))
                arm.step(f"release{k}_{name}", actions=[bt.seq.set_signal(f"grant_{name}", False)],
                         transition=bt.seq.immediately())
        names.append("traffic")
    return names


# ================================================================== the cell
def build(*, aisle: float = AISLE, interlock: bool = True) -> tuple[bt.Scene, PalletAmr]:
    lane = AISLE_S + aisle / 2
    stand_y = AISLE_S + aisle + STAND_LEN / 2
    rack_y0 = AISLE_S + aisle + STAND_LEN + 0.35
    machine = PalletAmr()

    scene = bt.Scene()
    building(scene)
    markings(scene, lane, stand_y)
    loads = {"p1": receiving(scene)}
    racking(scene, stand_y, rack_y0)
    _, loads["p2"] = picking_station(scene)
    loads["p4"] = packing(scene)
    shipping(scene)
    parks = standby(scene)
    materials(scene)
    # The picked pallet waiting at rack B's front for flow ②.
    loads["p3"] = pallet_with_cases(scene, "p3", (RACK_X[1], stand_y), -math.pi / 2, courses=1, z0=STAND_SUPPORT)

    lifts = routes(scene, machine, parks, lane, stand_y)
    # The cup touches the case it lifts: declared, not discovered.
    for case in loads["p2"] + loads["p3"]:
        for link in CUPS:
            scene.allow_link_obstacle_contact(link, case, robot="picker")
    teach(scene, loads["p2"])
    programs(scene, machine, lifts, loads, interlock)
    scene.set_obstacle_material("floor/slab", metalness=0.0, roughness=0.9)
    return scene, machine


def bake(*, aisle: float = AISLE, interlock: bool = True):
    scene, machine = build(aisle=aisle, interlock=interlock)
    names = ["receiving", "picking", "supply"] + (["traffic"] if interlock else [])
    return scene, machine, scene.simulate_sequences(names, max_duration=900.0)


# ================================================================== reports
def flows(tl) -> dict[str, tuple[float, float]]:
    """Each flow as the span from its first drive to its last release."""
    spans = {name: (t0, t1) for name, t0, t1 in tl.step_spans}
    def between(a: str, b: str):
        return (spans[a][0], spans[b][1])
    return {
        "① 入荷後の棚前搬送": between("receiving/request_recv", "receiving/release_p1"),
        "③ 梱包後の出荷搬送": between("receiving/request_packout", "receiving/release_p4"),
        "② 棚出し・作業台供給": between("supply/request_pick", "supply/supplied"),
    }


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


def tightest_pass(scene: bt.Scene, tl, machine: PalletAmr, dt: float = 0.1) -> tuple[float, float, tuple[str, str]]:
    """How close a driving machine — body, lift and the pallet it carries —
    comes to the equipment standing in the shed, over the whole shift.

    The rollout already proves there is no contact; this is the margin
    that proof leaves, measured between axis-aligned bounds (the loaded
    machine's box, grown to the pallet it carries, against every standing
    obstacle), so it is a floor on the true clearance, never above it."""
    # What a machine is *meant* to drive over or into stays out of the
    # scan: its loads, the stands it docks inside (45 mm a side by the
    # pack's inner width) and the charging plates it parks over.
    skip = ("amr1", "amr2", "p1/", "p2/", "p3/", "p4/", "marking/", "floor/", "stand/", "charger/")
    static = [(n, *scene.obstacle_bounds(n)) for n in scene.obstacle_names
              if not n.startswith(skip) and "/trim" not in n and "_curb" not in n]
    half = (max(machine.length, PALLET[1]) / 2, max(machine.width, PALLET[1]) / 2)   # a pivot could point either way
    lanes = dict(tl.signals)
    best = (math.inf, 0.0, ("", ""))
    for name in ("amr1", "amr2"):
        for t0, t1 in _high_spans(lanes[name], tl.duration):
            t = t0
            while t <= t1:
                (x, y, _), q = tl.object_pose(f"{name}/base_link", t)
                yaw = yaw_of(q)
                c, s_ = abs(math.cos(yaw)), abs(math.sin(yaw))
                hx = half[0] * c + half[1] * s_
                hy = half[0] * s_ + half[1] * c
                lo = (x - hx, y - hy, machine.specs["ground_clearance_mm"] / 1000.0)
                hi = (x + hx, y + hy, machine.lowered + machine.stroke + PALLET[2] + 1.0)
                for other, olo, ohi in static:
                    gap = max(0.0, olo[0] - hi[0], lo[0] - ohi[0], olo[1] - hi[1], lo[1] - ohi[1],
                              olo[2] - hi[2], lo[2] - ohi[2])
                    if gap < best[0]:
                        best = (gap, t, (name, other))
                t += dt
    return best


def deliver(scene: bt.Scene, tl, out: Path) -> None:
    out.mkdir(parents=True, exist_ok=True)
    files = []

    def write(name, fn):
        path = out / name
        fn(path)
        files.append(path)

    write("warehouse_layout.svg", lambda p: scene.export_layout(p, scale=40, title="物流倉庫 AMR 自動化 (案)"))
    write("warehouse_layout.dxf", lambda p: scene.export_layout(p, title="warehouse"))
    write("warehouse_bom.csv", scene.export_bom)
    write("warehouse_bom.md", scene.export_bom)
    write("warehouse_io.csv", scene.export_io_list)
    write("warehouse_interlocks.md", scene.export_interlocks)
    write("warehouse.plcopen.xml", lambda p: scene.export_plcopen(p, name="warehouse"))
    write("warehouse_cell.usdc", lambda p: tl.export_usd(p, fps=15.0))
    report = scene.cell_report(tl, deliverables=files, title="物流倉庫 AMR 自動化 (案)")
    report.save(out / "warehouse_report.md")
    report.save(out / "warehouse_report.json")
    print(f"wrote the document set to {out}/ ({len(files) + 2} files)")


def main() -> None:
    global CATALOG_ROOT
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("recording", nargs="?", default=None, help="write the baked shift as USD here")
    parser.add_argument("--aisle", type=float, default=AISLE, help="main aisle width in metres (the sketch says 3.0 minimum)")
    parser.add_argument("--no-interlock", dest="no_interlock", action="store_true",
                        help="drop the traffic control: both machines drive on their own clock, refused")
    parser.add_argument("--out", type=Path, default=None, help="write the document set here")
    parser.add_argument("--catalog-root", type=Path, default=None, help="a catalog builder's build/ directory")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()
    if args.catalog_root:
        CATALOG_ROOT = args.catalog_root.resolve()

    if args.no_interlock or args.aisle != AISLE:
        try:
            bake(aisle=args.aisle, interlock=not args.no_interlock)
        except ValueError as err:
            print("refused, as it should be:")
            print(f"  {err}")
            return
        if args.no_interlock:
            raise SystemExit("two machines on one aisle without traffic control should have been refused")
        print(f"a {args.aisle:.2f} m aisle passes")
        return

    scene, machine, tl = bake()
    print(f"{machine.product} ({machine.maker}) x 2 with the {machine.lift_spec['name']}: "
          f"body {machine.length:.2f} x {machine.width:.2f} m, pivot sweeps {machine.swing:.2f} m, "
          f"rail top {machine.lowered * 1e3:.0f} mm under a {STAND_SUPPORT * 1e3:.0f} mm stand, "
          f"lift {machine.stroke * 1e3:.0f} mm")
    print(f"shift {tl.duration:.1f} s")
    for name, (t0, t1) in flows(tl).items():
        print(f"  {name}: {t0:6.1f} – {t1:6.1f} s  ({t1 - t0:.1f} s)")
    lanes = dict(tl.signals)
    call = next(t for t, v in lanes["call"] if v)
    arrived = tl.step_span("supply/to_pick").end
    done = next(t for t, v in lanes["supply_done"] if v)
    print(f"  呼出し {call:.1f} s → 到着 {arrived:.1f} s → 供給完了 {done:.1f} s: "
          f"the picker waited {done - call:.1f} s")
    waits = [(name, tl.step_span(name).duration) for name, _, _ in tl.step_spans if "/request_" in name]
    print("  aisle grants: " + ", ".join(f"{n.split('/')[1][8:]}={d:.1f}s" for n, d in waits))
    gap, at, pair = tightest_pass(scene, tl, machine)
    print(f"  tightest pass of a loaded machine to the standing equipment: {gap * 1e3:.0f} mm at {at:.1f} s "
          f"({pair[0]} x {pair[1]}); inside a stand it has "
          f"{(STAND_INNER - machine.width) * 500:.0f} mm a side")
    for name in ("amr1", "amr2"):
        driving = sum(t1 - t0 for t0, t1 in _high_spans(lanes[name], tl.duration))
        print(f"  {name} driving {driving:.0f} s of {tl.duration:.0f} ({driving / tl.duration:.0%})")
    print(f"  picker busy {tl.utilizations()['picker']:.0%}")
    if args.recording:
        tl.export_usd(args.recording, fps=15)
        print(f"wrote {args.recording}")
    if args.out:
        deliver(scene, tl, args.out)
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
