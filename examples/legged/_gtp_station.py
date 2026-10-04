"""The goods-to-person shipping station of `gtp_station_demo.py`, rebuilt
from one frame of a video of a GTP (shelf-to-person AGV) system, captioned
"QR読み取り〜カゴ台車積み込み" (read the QR code, load the roll cage).

What the frame shows, and what is built here (world: x across the fence into
the AGV area, y along the fence toward the frame's left, z up, metres; the
origin on the floor at the foot of the centre window's right post):

  * the AGV area behind a black welded-mesh fence (built as Axelent
    X-Guard from the catalog, 2.5 m), a white line painted along its foot on
    the people's side, and three station windows in it — left, centre and
    (cut by the frame's right edge) right;
  * a field of yellow inventory pods behind it, in blocks with aisles, each
    on four silver legs with the pocket a drive unit lifts it by;
  * at the left window a pod, a picker reaching into its bins, and the long
    orange table with grey folding containers (オリコン); the next pod
    waiting behind the panel;
  * at the centre window a dark carton rack with its cartons, the worker who
    reads their QR codes, the orange table with the papers and the bin under
    it, and the roll cage the cartons go into;
  * at the right window the same: a carton rack, the table, its worker and
    the roll cage at the frame's edge.

Sizes the frame cannot give are standard ones or read off it against the
people (1.75 m). What can be bought is ordered from the catalog in the size
nearest the frame — the fence (X-Guard Classic), the roll cages (マキテック
MRC-S5, 1100 x 800 x 1700), the containers (三甲 オリコン30B) and the drive
units under the racks (日立 Racrew); the people (`bt.parts.person`) and the
racks a drive unit carries (`bt.parts.mobile_rack`, 1.2 m square: what a
Racrew drives under) are botrail's. The camera is fitted to the frame's
vanishing points (`photo_camera()`): pitched 27.5 degrees down, f = 839 px,
the eye 3.25 m up.

The tables are `bt.parts.table`, the cartons `bt.parts.carton`, the white
line `bt.parts.marking`, the waste bin and the high-bay lights shapes of
botrail's library. What botrail has no picture for — the concrete floor
and the papers on the tables — is textured unit-box USD layers (pxr + PIL)
in `examples/assets/gtp_station/textured/`, each the picture of a box this
file owns.
"""

from __future__ import annotations

import functools
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

import botrail as bt
import yaml

ASSETS = Path(__file__).resolve().parents[1] / "assets" / "gtp_station"
TEX_DIR = ASSETS / "textured"


def _manifest(path: Path) -> dict:
    return json.loads(path.read_text()) if path.exists() else {}


TEXTURED = _manifest(TEX_DIR / "textured.json")


def srgb(r: float, g: float, b: float) -> tuple:
    """0-255 sRGB (a swatch, the frame) to the linear RGB botrail takes."""
    def lin(v):
        v /= 255.0
        return v / 12.92 if v <= 0.04045 else ((v + 0.055) / 1.055) ** 2.4
    return (round(lin(r), 4), round(lin(g), 4), round(lin(b), 4))


# ---- finishes (read off the frame, then pushed toward the real thing) --------
FLOOR = srgb(92, 92, 90)             # dark grey concrete
LINE_WHITE = srgb(232, 232, 226)
FENCE_BLACK = srgb(24, 24, 26)       # black powder coat
DARK_STEEL = srgb(48, 49, 53)
TABLE_ORANGE = srgb(226, 142, 42)
PAPER = srgb(240, 240, 236)
TOTE_GREY = srgb(150, 156, 160)
SHIRT = srgb(200, 190, 170)

# ---- the fence --------------------------------------------------------------
# Axelent X-Guard Classic, graphite black: 2400 panels on the pack's 100 mm
# floor gap, the posts to 2.5 m (the frame reads the top rail at 2.41 m), 50 mm
# posts. A run between two windows is whole panels with nothing left over, so
# the right window's left post and the left window's right post stand 3 and
# 4 cm off the frame's; the centre window is the frame's
FENCE = "axelent/fence/x-guard-classic"
FENCE_H = 2.4                        # the panel height ordered
LINE_X, LINE_W = -0.24, 0.10         # the white line's centre and width
WINDOWS = {"left": (4.06, 4.99), "centre": (0.02, 1.06), "right": (-3.89, -2.88)}   # y between the posts' centres
FENCE_RUNS = {                       # y from, to — laid from the window, so the wall end takes no filler
    "right": (-3.89, -16.84),        # 8 x 1500 + 500, to the south wall
    "right_centre": (-2.88, 0.02),   # 2 x 1400
    "centre_left": (1.06, 4.06),     # 1500 + 1400
    "left": (4.99, 24.34),           # 12 x 1500 + 700, to the north wall
}
FENCE_Y = (-16.84, 24.34)

# ---- the station furniture ---------------------------------------------------
TABLE = (1.0, 1.0, 0.75)             # station table: depth (x) x width (y) x height
TABLES = {"centre": (-0.57, 1.65), "right": (-0.58, -2.49)}
LONG_TABLE = ((-2.45, -0.10), (4.92, 5.82), 0.75)   # the picking table, its length across the fence
TABLE_LEGS = srgb(26, 26, 28)        # the tables' black frame under the orange top
# the frame's large two-sided roll cage (it reads 1160 x 820 x 1800 against a
# 1.75 m worker) is the common large size: マキテック MRC-S5
CAGE_PACK = "makitech/roll-box-pallet/mrc-s"
CAGE = (1.10, 0.80, 1.70)            # width (the open sides run along it) x depth x height
CAGE_DECK_Z = 0.243                  # its deck (the pack's)
CAGES = {"centre": ((-2.04, 0.55), -math.pi / 2), "right": ((-1.85, -3.42), -math.pi / 2)}   # open to the fence
# the grey order totes on the picking table: 三甲 オリコン30B (530 x 366 x 205),
# grey as the frame draws them
TOTE_PACK = "sanko/oricon/oricon"
TOTE_H = 0.205
PERSON_H = 1.75
WORKERS = {"left": ((-0.56, 4.42), 0.35, "pick"), "centre": ((-0.48, 0.44), 0.0, "reach"),
           "right": ((-2.02, -2.42), 2.0, "stand")}

# ---- the AGV area ------------------------------------------------------------
# The drive units are 日立 Racrew (catalog): 990 x 916, 380 to the turntable.
# A rack's underside stands 2 cm over it; a carried rack is lifted 3 cm off
# the floor — the turntable raised 5 cm (the stroke is not published)
AGV_PACK = "hitachi_industrial_products/racrew/racrew"
SHELF_CLEARANCE = 0.40
SHELF_LIFT = 0.03
POD = (1.2, 1.2, 2.25)
POD_GAP = 0.06
BLOCK = (2, 2)                       # pods per block (x, y)
AISLE = (1.3, 1.3)
FIELD = ((5.5, 24.0), (-15.13, 22.6))  # the first row's front at x = 5.5, a block on y [-1.36, 1.18]
RACK = (1.2, 1.2)                    # the carton racks at the windows (a Racrew drives under them)
RACK_DECK_T = 0.085                  # `mobile_rack`'s open deck: frame, plate and tray
RACK_SEAT = SHELF_CLEARANCE + RACK_DECK_T + SHELF_LIFT   # the tray's top, carried: what a carton stands on
RACK_H = 1.45                        # its top rail
RACK_FRONT = 0.06                    # the AGV stops a rack this far behind the fence plane
CARTON = (0.40, 0.30, 0.25)          # the medium carton the robot handles: length x width x height (label on an end)
CARTON_COLUMNS = 1                   # on a rack for the robot: one column (the hands go either side), three layers
CARTON_LAYERS = 3
BIG_CARTON = (0.60, 0.45, 0.40)      # as imaged: the big cartons, two by two, long side to the window


# ---- small geometry helpers ----------------------------------------------------
def qz(a: float) -> tuple:
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def rotate(q, v) -> tuple:
    x, y, z, w = q
    vx, vy, vz = v
    cx, cy, cz = y * vz - z * vy, z * vx - x * vz, x * vy - y * vx
    ccx, ccy, ccz = y * cz - z * cy, z * cx - x * cz, x * cy - y * cx
    return (vx + 2 * (w * cx + ccx), vy + 2 * (w * cy + ccy), vz + 2 * (w * cz + ccz))


def deco(scene, name, size, at, color, q=None, **material) -> str:
    """A box that is only a picture."""
    made = scene.add_box(name, size, at, quaternion=q, color=color)
    scene.set_obstacle_enabled(made, False)
    if material:
        scene.set_obstacle_material(made, **material)
    return made


def solid(scene, name, size, at, color, q=None, **material) -> str:
    """A box the robot and the checks see."""
    made = scene.add_box(name, size, at, quaternion=q, color=color)
    if material:
        scene.set_obstacle_material(made, **material)
    return made


def span(scene, name, x, y, z, color, *, enabled=True, **material) -> str:
    size = (x[1] - x[0], y[1] - y[0], z[1] - z[0])
    at = ((x[0] + x[1]) / 2, (y[0] + y[1]) / 2, (z[0] + z[1]) / 2)
    return (solid if enabled else deco)(scene, name, size, at, color, **material)


def _prop(prop: str):
    """(layer, design entry) of a textured prop, or None."""
    if prop in TEXTURED:
        return TEX_DIR / f"{prop}.usda", TEXTURED[prop]
    return None


def has(prop: str) -> bool:
    return _prop(prop) is not None


def dress(scene, obstacle: str, prop: str, origin=(0.0, 0.0, 0.0), scale=(1.0, 1.0, 1.0)) -> str:
    """Draws `obstacle` as the prop `prop` (a unit-box USD layer) at `scale`
    times its design size, its design origin at `origin` in the obstacle's
    own frame. The box stays what collides; the prop is the picture."""
    layer, entry = _prop(prop)
    (sx, sy, sz), (cx, cy, cz) = entry["size"], entry["center"]
    kx, ky, kz = scale
    ox, oy, oz = origin[0] + cx * kx, origin[1] + cy * ky, origin[2] + cz * kz
    scene.set_obstacle_visual_asset(obstacle, layer, f"/Shapes/{prop}",
                                    (sx * kx, 0, 0, 0, 0, sy * ky, 0, 0, 0, 0, sz * kz, 0, ox, oy, oz, 1))
    scene.set_obstacle_material(obstacle)
    return obstacle


def prop_box(prop: str, scale=(1.0, 1.0, 1.0)) -> tuple:
    return tuple(s * k for s, k in zip(_prop(prop)[1]["size"], scale))


def place_prop(scene, name: str, prop: str, base, *, yaw: float = 0.0, scale=(1.0, 1.0, 1.0),
               enabled: bool = False, color=(0.6, 0.6, 0.6)) -> str:
    """A prop standing with its design origin at `base`, turned by `yaw`: one
    box the size of the prop's bounds, drawn as the prop."""
    size = prop_box(prop, scale)
    (cx, cy, cz) = (c * k for c, k in zip(_prop(prop)[1]["center"], scale))
    off = rotate(qz(yaw), (cx, cy, cz))
    at = (base[0] + off[0], base[1] + off[1], base[2] + off[2])
    made = scene.add_box(name, size, at, quaternion=qz(yaw), color=color)
    if not enabled:
        scene.set_obstacle_enabled(made, False)
    dress(scene, made, prop, origin=(-cx, -cy, -cz), scale=scale)
    return made


def fence(scene) -> None:
    """The fence along x = 0: an X-Guard run (FENCE_RUNS) between each two
    windows and from the outer windows to the walls — a post either side of
    every window. The panels' slabs and the posts collide; the pack's panels
    and posts draw them."""
    for run, (a, b) in FENCE_RUNS.items():
        bt.parts.fence(scene, f"fence/{run}", [(0.0, a), (0.0, b)], catalog=FENCE, height=FENCE_H, closed=False)


# ============================================================ the floor
def floor(scene) -> None:
    """The concrete floor and the white line along the fence's foot."""
    if has("floor_concrete"):
        entry = TEXTURED["floor_concrete"]
        (sx, sy, _sz), (cx, cy, _cz) = entry["size"], entry["center"]
        made = deco(scene, "floor", (sx, sy, 0.01), (cx, cy, -0.005), FLOOR)
        scene.set_obstacle_visual_asset(made, TEX_DIR / "floor_concrete.usda", "/Shapes/floor_concrete",
                                        (1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, -cx, -cy, 0.005, 1))
        scene.set_obstacle_material(made)
    else:
        deco(scene, "floor", (60.0, 40.0, 0.01), (10.0, 0.0, -0.005), FLOOR, roughness=0.75, metalness=0.0)
    # the hall's floor beyond the textured patch (what the far corners of a view see)
    deco(scene, "floor/hall", (140.0, 140.0, 0.01), (10.0, 0.0, -0.0062), srgb(86, 85, 81), roughness=0.8, metalness=0.0)
    y0, y1 = FENCE_Y
    bt.parts.marking(scene, "floor/line", line=((LINE_X, y0), (LINE_X, y1)), width=LINE_W, color=LINE_WHITE, thickness=0.002)


# ============================================================ the hall
HALL_X = (-11.0, 26.5)
HALL_Y = (-17.0, 24.5)
HALL_EAVE = 9.0
WALL_LOW = srgb(140, 140, 135)       # painted block to 2.4 m
WALL_SIDING = srgb(158, 161, 160)    # ribbed steel siding above
ROOF_STEEL = srgb(92, 94, 98)


def hall(scene, *, ceiling: bool = True) -> None:
    """The building around the station (outside the frame's view, which ends
    in the pod field): block walls with steel siding, columns at 10 m along
    them, and — `ceiling` — the roof, its trusses and the high-bay lights. The
    roof is a hair short of opaque so it casts no shadow (the studio's key
    light comes from above)."""
    (x0, x1), (y0, y1), h = HALL_X, HALL_Y, HALL_EAVE
    walls = (("n", (x1 - x0, 0.2), ((x0 + x1) / 2, y1)), ("s", (x1 - x0, 0.2), ((x0 + x1) / 2, y0)),
             ("e", (0.2, y1 - y0), (x1, (y0 + y1) / 2)), ("w", (0.2, y1 - y0), (x0, (y0 + y1) / 2)))
    for tag, (sx, sy), (cx, cy) in walls:
        deco(scene, f"hall/wall_{tag}", (sx, sy, 2.4), (cx, cy, 1.2), WALL_LOW, roughness=0.85, metalness=0.0)
        deco(scene, f"hall/siding_{tag}", (sx, sy, h - 2.4), (cx, cy, 2.4 + (h - 2.4) / 2), WALL_SIDING, roughness=0.5,
             metalness=0.3, finish="plastic")
        deco(scene, f"hall/kerb_{tag}", (sx + 0.02, sy + 0.02, 0.12), (cx, cy, 0.06), srgb(228, 190, 40), roughness=0.6)
    for i, x in enumerate(range(int(x0) + 1, int(x1), 10)):
        for j, y in enumerate((y0 + 0.25, y1 - 0.25)):
            deco(scene, f"hall/column{i}_{j}", (0.3, 0.3, h), (x, y, h / 2), srgb(120, 124, 130), roughness=0.5, metalness=0.5)
    if not ceiling:
        return
    deco(scene, "hall/roof", (x1 - x0, y1 - y0, 0.15), ((x0 + x1) / 2, (y0 + y1) / 2, h + 1.0), ROOF_STEEL, roughness=0.8,
         opacity=0.995)
    for i, x in enumerate(range(int(x0) + 3, int(x1), 8)):
        deco(scene, f"hall/truss{i}", (0.25, y1 - y0, 0.7), (x, (y0 + y1) / 2, h + 0.55), srgb(110, 112, 116), roughness=0.6,
             metalness=0.4, opacity=0.995)
        for j, y in enumerate(range(int(y0) + 4, int(y1), 7)):
            made = bt.parts.shaped_box(scene, f"hall/light{i}_{j}", "highbay", (0.44, 0.44, 0.164), (x + 4.0, y, h - 0.4 + 0.082))
            scene.set_obstacle_material(made, opacity=0.995)
            deco(scene, f"hall/light{i}_{j}/hanger", (0.02, 0.02, 1.0), (x + 4.0, y, h + 0.24), srgb(60, 60, 60), opacity=0.995)


# ============================================================ the AGV area
def pod_at(scene, name: str, x: float, y: float, *, yaw: float = 0.0, enabled: bool = True, lift: float = 0.0) -> str:
    """A yellow inventory pod standing with its footprint centred on (x, y);
    `lift` = carried, its feet that far off the floor."""
    return bt.parts.mobile_rack(scene, name, POD, (x, y), clearance=SHELF_CLEARANCE, lift=lift, yaw=yaw,
                                collide=enabled, description="existing (as imaged)").name


def pod_field(scene) -> list:
    """The pod field: blocks of BLOCK pods with AISLE between, from FIELD_X0
    back. Returns the pods' names."""
    (x0, x1), (y0, y1) = FIELD
    px, py = POD[0] + POD_GAP, POD[1] + POD_GAP
    bx, by = BLOCK[0] * px + AISLE[0], BLOCK[1] * py + AISLE[1]
    made = []
    k = 0
    x = x0
    while x + BLOCK[0] * px <= x1 + 1e-6:
        y = y0
        while y + BLOCK[1] * py <= y1 + 1e-6:
            for i in range(BLOCK[0]):
                for j in range(BLOCK[1]):
                    made.append(pod_at(scene, f"pods/p{k:03d}", x + (i + 0.5) * px, y + (j + 0.5) * py, enabled=False))
                    k += 1
            y += by
        x += bx
    return made


def rack_seat(y: float, column: int, layer: int, gap: float) -> tuple:
    """A carton's underside centre on the rack centred on `y`: the front row,
    its length along the window and its labelled face 2 cm behind the rack's
    front edge, `layer` from the deck. With more than one column, `column` 0
    is +y (the left as the window sees it) and the columns are `gap` apart."""
    dy = 0.0 if CARTON_COLUMNS == 1 else (CARTON[0] + gap) / 2
    return (RACK_FRONT + 0.02 + CARTON[1] / 2, y + (dy if column == 0 else -dy), RACK_SEAT + CARTON[2] * layer)


def carton(scene, name: str, seat, size=CARTON, yaw: float = -math.pi / 2) -> str:
    """A shipping carton (`bt.parts.carton`) standing on `seat` (its
    underside centre), turned by `yaw`: by default its length along the
    window and its labelled long face (the library carton's -y) toward the
    people (-x). Goods, not equipment: nothing pinned (the demo pins the
    ones it handles)."""
    made = bt.parts.carton(scene, name, size, seat, yaw=yaw)
    scene.remove_part(made)
    return made


def carton_rack(scene, name: str, y: float, *, load: str = "big", gap: float = 0.10, front: float = RACK_FRONT,
                riser: float = 0.0, height: float = RACK_H) -> tuple:
    """The dark carton rack at a window (its front RACK_FRONT behind the fence,
    centred on `y`) with its load: as imaged, big cartons two by two
    (`load="big"`); for the robot, medium cartons in two columns `gap` apart
    and three layers (`"medium"`); or nothing (`None`). Returns (the rack's
    obstacles, {carton: seat}). `front` puts the rack elsewhere along x (a
    rack waiting behind the window). `riser` = a platform on the deck that
    raises the load (`{name}/riser`), `height` = the rack's top ring."""
    x0, x1 = front, front + RACK[0]
    xc = (x0 + x1) / 2
    # the rack (open toward the window), carried: what collides is its
    # uprights, its deck and the rails round the top of the three closed sides
    rack = bt.parts.mobile_rack(scene, name, (*RACK, height), (xc, y), style="open", clearance=SHELF_CLEARANCE,
                                lift=SHELF_LIFT, yaw=math.pi)
    assert abs(scene.frame(f"{name}/deck")[0][2] - RACK_SEAT) < 1e-9
    parts = list(rack.obstacles)
    if riser > 0.0:
        # a steel platform with a grey top, inside the uprights, the load on it
        parts.append(span(scene, f"{name}/riser", (x0 + 0.05, x1 - 0.05), (y - RACK[1] / 2 + 0.05, y + RACK[1] / 2 - 0.05),
                          (RACK_SEAT, RACK_SEAT + riser), srgb(96, 100, 106), roughness=0.55, metalness=0.35))
    parts += drive_unit(scene, f"{name}/agv", xc, y)
    seats = {}
    if load == "medium":
        k = 0
        for layer in range(CARTON_LAYERS):
            for column in range(CARTON_COLUMNS):
                seat = rack_seat(y, column, layer, gap)
                seat = (seat[0] + front - RACK_FRONT, seat[1], seat[2] + riser)
                seats[carton(scene, f"{name}/carton{k}", seat)] = seat
                k += 1
    elif load == "big":
        l, w, h = BIG_CARTON
        for k, (dy, layer) in enumerate(((l / 2 + 0.01, 0), (-l / 2 - 0.01, 0), (l / 2 + 0.01, 1), (-l / 2 - 0.01, 1))):
            seat = (front + 0.05 + w / 2, y + dy, RACK_SEAT + h * layer)   # behind the front uprights
            seats[carton(scene, f"{name}/carton{k}", seat, BIG_CARTON)] = seat
    return parts, seats


@functools.lru_cache(maxsize=None)
def agv_package() -> tuple:
    """The drive unit's catalog package: (its directory, its manifest)."""
    root = Path(bt.catalog_package(AGV_PACK))
    return root, yaml.safe_load((root / "manifest.yaml").read_text(encoding="utf-8"))


def drive_unit(scene, name: str, x: float, y: float, yaw: float = 0.0, *, carrying: bool = True) -> list:
    """A drive unit (日立 Racrew, catalog) at (x, y) facing `yaw`, under the
    rack it lifts (`carrying`: its turntable raised to the rack's underside,
    a dark lift column under it). What collides: the published envelope, one
    box (`name`); the catalog model draws it. Returns the obstacles."""
    root, manifest = agv_package()
    urdf, specs = root / manifest["assets"]["urdf"], manifest["specs"]
    length, width = (v / 1000.0 for v in specs["footprint_mm"])
    h, under = specs["height_mm"] / 1000.0, specs["ground_clearance_mm"] / 1000.0
    body = solid(scene, name, (length, width, h - under), (x, y, under + (h - under) / 2), DARK_STEEL, q=qz(yaw))
    scene.set_obstacle_visible(body, False)
    made = [body]
    for piece in scene.load_urdf(urdf, prefix=f"{name}/look", position=(x, y, 0.0), quaternion=qz(yaw), frames=False):
        scene.set_obstacle_enabled(piece, False)
        made.append(piece)
    rise = SHELF_CLEARANCE + SHELF_LIFT - h
    if carrying and rise > 0:
        (px, py, pz), q = scene.obstacle_pose(f"{name}/look/turntable")
        scene.set_obstacle_pose(f"{name}/look/turntable", (px, py, pz + rise), q)
        made.append(deco(scene, f"{name}/look/lift", (0.5, 0.5, rise), (x, y, h + rise / 2 - 0.004), srgb(40, 42, 46),
                         q=qz(yaw), roughness=0.5, metalness=0.4))
    return made


# ============================================================ the stations
def station_table(scene, name: str, centre, *, papers: bool = True) -> None:
    """The orange station table against the fence (TABLE: depth x width x
    height), papers on it."""
    d, w, h = TABLE
    x, y = centre
    bt.parts.table(scene, name, (d, w, h), (x, y), detail="full", top_thickness=0.05, color=TABLE_LEGS,
                   top_color=TABLE_ORANGE, model=f"作業台 {d * 1000:.0f}x{w * 1000:.0f}(オレンジ天板)",
                   description="existing (as imaged)")
    if papers:
        if has("paper_stack"):
            place_prop(scene, f"{name}/papers", "paper_stack", (x - 0.05, y - 0.05, h + 0.004), yaw=math.pi / 2 + 0.15)
            place_prop(scene, f"{name}/list", "paper_list", (x - 0.25, y + 0.22, h + 0.003), yaw=math.pi / 2 - 0.35)
        else:
            deco(scene, f"{name}/papers", (0.31, 0.22, 0.006), (x - 0.05, y - 0.05, h + 0.003), PAPER, q=qz(0.15), roughness=0.8)
            deco(scene, f"{name}/list", (0.297, 0.21, 0.002), (x - 0.25, y + 0.22, h + 0.001), PAPER, q=qz(-0.35), roughness=0.8)


def roll_cage(scene, name: str, centre, yaw: float) -> list:
    """A two-sided roll cage (カゴ台車, the pack's MRC-S5) standing at
    `centre`, turned by `yaw` (its open sides along its own y). What
    collides: the base up to the deck and the two end frames. Returns the
    obstacles."""
    return bt.parts.roll_container(scene, name, position=centre, catalog=CAGE_PACK, width_mm=round(CAGE[0] * 1000),
                                   depth_mm=round(CAGE[1] * 1000), yaw=yaw).obstacles


def worker(scene, name: str, which: str) -> str:
    """A worker of the frame, standing as WORKERS has them (`bt.parts.person`)."""
    at, yaw, pose = WORKERS[which]
    return bt.parts.person(scene, name, at, yaw=yaw, pose=pose, height=PERSON_H, color=SHIRT)


def left_station(scene, *, people: bool = True) -> None:
    """The picking window: a pod in the window, the long table with grey
    order totes, the picker; the next pod waiting behind the panel."""
    ya, yb = WINDOWS["left"]
    pod_at(scene, "left/pod", 0.66, (ya + yb) / 2, lift=SHELF_LIFT)
    drive_unit(scene, "left/agv", 0.66, (ya + yb) / 2)
    pod_at(scene, "left/next", 2.40, 3.02)
    (x0, x1), (y0, y1), h = LONG_TABLE
    bt.parts.table(scene, "left/table", (x1 - x0, y1 - y0, h), ((x0 + x1) / 2, (y0 + y1) / 2), detail="full",
                   top_thickness=0.05, color=TABLE_LEGS, top_color=TABLE_ORANGE,
                   model=f"作業台 {(x1 - x0) * 1000:.0f}x{(y1 - y0) * 1000:.0f}(オレンジ天板)", description="existing (as imaged)")
    for i, xx in enumerate((-0.42, -1.00, -1.58)):
        bt.parts.bin(scene, f"left/tote{i}", position=(xx, (y0 + y1) / 2, h), catalog=TOTE_PACK,
                     height_mm=round(TOTE_H * 1000), yaw=math.pi / 2 + 0.04 * (i - 1), color=TOTE_GREY)
    if people:
        worker(scene, "left/worker", "left")


def case_station(scene, which: str, *, people: bool = True) -> list:
    """A shipping window (centre or right): the carton rack with its cartons,
    the table with the papers, the roll cage, the worker."""
    ya, yb = WINDOWS[which]
    _rack, cartons = carton_rack(scene, f"{which}/rack", (ya + yb) / 2, load="big")
    station_table(scene, f"{which}/table", TABLES[which])
    if which == "centre":
        x, y = TABLES[which]
        bt.parts.shaped_box(scene, "centre/bin", "waste_bin", (0.35, 0.30, 0.55), (x + 0.15, y - 0.25, 0.275), quaternion=qz(math.pi / 2))
    roll_cage(scene, f"{which}/cage", *CAGES[which])
    if people:
        worker(scene, f"{which}/worker", which)
    return cartons


# ============================================================ assembly
@dataclass
class Station:
    pods: list = field(default_factory=list)
    cartons: dict = field(default_factory=dict)


def build(scene, *, to_be: bool = False, ceiling: bool = True) -> Station:
    """The station as imaged (people at work). With `to_be` the centre window
    is left to the demo: no worker, no table (the robot's QR read is the
    record), no rack (the AGV brings it) and no cage (it stands where the
    robot loads it)."""
    floor(scene)
    hall(scene, ceiling=ceiling)
    fence(scene)
    st = Station()
    st.pods = pod_field(scene)
    left_station(scene)
    if not to_be:
        st.cartons["centre"] = case_station(scene, "centre")
    st.cartons["right"] = case_station(scene, "right")
    return st


# ============================================================ the frame's camera
# Fitted to the frame (1280 x 696) by least squares (0.4 px rms): the fence's
# top rail and the white line meet at (-982, -89); its posts converge below at
# (640, 1958) — a camera pitched 27.5 degrees down, the principal point at the
# frame's centre, f = 839 px; the eye from the centre worker's height (1.75 m),
# which makes the fence 2.41 m and the cage 1.8 m.
PHOTO_SIZE = (1280, 696)
PHOTO_F = 838.8
PHOTO_EYE = (-4.526, -2.439, 3.247)
PHOTO_YAW, PHOTO_PITCH = 30.25, 27.52  # degrees: azimuth from +x toward +y, and down


def photo_camera() -> dict:
    """The frame's camera as eye, look-at, up and vertical field of view."""
    ps, th = math.radians(PHOTO_YAW), math.radians(PHOTO_PITCH)
    F = (math.cos(th) * math.cos(ps), math.cos(th) * math.sin(ps), -math.sin(th))
    _w, h = PHOTO_SIZE
    fov = 2 * math.degrees(math.atan(h / 2 / PHOTO_F))
    look = tuple(e + 5.0 * f for e, f in zip(PHOTO_EYE, F))
    return {"pos": list(PHOTO_EYE), "look": list(look), "up": [0, 0, 1], "fov": fov}


if __name__ == "__main__":
    scene = bt.Scene()
    build(scene)
    print(len(scene.obstacle_names), "obstacles")
    cam = photo_camera()
    bt.studio(scene, view=(tuple(cam["pos"]), tuple(cam["look"])))
