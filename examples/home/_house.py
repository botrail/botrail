"""The flat `home_tidy_demo.py` tidies: a 7.2 x 4.8 m living-dining-kitchen.

The furniture and the appliances are SimReady Central packages — NVIDIA's
sofa and kettle, LG's refrigerator, dishwasher, microwave, OLED television
and air purifier, ForgeCAD's kitchen cabinets, dining chairs, glasses and
bin, Palatial's range hood, faucet, coffee maker, lamp, cabinet, plant and
mugs — fetched at the commits the library's index registered and placed
with `bt.parts.simready`. What the
cell *verifies* is built here from parameters instead (AGENTS.md): the
coffee table and the dining table the robot picks from, the sink it puts
things in and the counter around it, the walls and the doorways — a
dimension the tidy-up depends on is never frozen inside a baked asset.

Coordinates: the inside south-west corner of the room is the origin, x runs
along the south wall (east), y into the room (north), metres.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import botrail as bt

# ------------------------------------------------------------ the packages
# The links SimReady Central's search hands out (2026-10-23), kept as they
# came: every one is pinned to the dataset commit the index registered.
_NV = "https://huggingface.co/datasets/nvidia/simready-assets/blob/653d91e182011f357cf1644013cc94ddf194d349"
_LG = "https://huggingface.co/datasets/LGElectronics/simready-assets/blob"
_FC = "https://huggingface.co/datasets/ForgeCAD/simready-assets/blob/0573053fa2c9174ca2a4678a462ec4dfd3035d59"
_PS = "https://huggingface.co/datasets/PalatialSim/palatial-nv-asset-library/blob/69a87329036cec0440403351c12230970c4e67d8"
ASSETS = {
    "sofa": f"{_NV}/sofa_soft_b02/simready_usd/sm_sofa_soft_b02_01.usd",
    "tv": f"{_LG}/528532c52b7adc2fef1cf7626bad7953791153a1/tv_oled77g6wua/simready_usd/tv_oled77g6wua.usd",
    "air_purifier": f"{_LG}/ddc78d9b3d3d3c09b7ba4c22798192710defcf8c/air-purifier_as25gcbz0/simready_usd/air-purifier_as25gcbz0.usd",
    "refrigerator": f"{_LG}/c071e517c96ef837b3bdd1c2399bfa32d9bdbd18/refrigerator_lrfs28xbs/simready_usd/refrigerator_lrfs28xbs.usd",
    "dishwasher": f"{_LG}/1db222d77da29ce9018a07b19ddfe6b5048dd18c/dishwasher_ldfc2423v/simready_usd/dishwasher_ldfc2423v.usd",
    "microwave": f"{_LG}/088bbc929fd9987d055904e7c92cc07108d2d872/cooking_mser1130b/simready_usd/cooking_mser1130b.usd",
    "kettle": f"{_NV}/kitchenware_kettle_a01/simready_usd/sm_kitchenware_kettle_a01_01.usd",
    "base_cabinet": f"{_FC}/kitchen_base_cabinet/simready_usd/kitchen_base_cabinet.usda",
    "chair": f"{_FC}/dining_chair/simready_usd/dining_chair.usda",
    "glass": f"{_FC}/drinking_glass/simready_usd/drinking_glass.usda",
    "trash": f"{_FC}/pedal_trash_can/simready_usd/pedal_trash_can.usda",
    "hood": f"{_PS}/D00194/simready_usd/D00194.usd",
    "faucet": f"{_PS}/D00300/simready_usd/D00300.usd",
    "coffee_maker": f"{_PS}/D00155/simready_usd/D00155.usd",
    "lamp": f"{_PS}/D00261/simready_usd/D00261.usd",
    "cabinet": f"{_PS}/D00092/simready_usd/D00092.usd",
    "plant": f"{_PS}/D00655/simready_usd/D00655.usd",
    "mug": f"{_PS}/D00160/simready_usd/D00160.usd",
}
# The way each package faces in its own frame: NVIDIA and LG author the
# front along +x, ForgeCAD and Palatial along -y (radians, measured from +x).
FRONT = {key: -math.pi / 2 for key in ("base_cabinet", "chair", "glass", "trash", "hood", "faucet",
                                       "coffee_maker", "lamp", "cabinet", "plant", "mug")}
CATEGORY = {
    "sofa": "furniture.sofa", "tv": "appliance.tv", "air_purifier": "appliance.air_purifier",
    "refrigerator": "appliance.refrigerator", "dishwasher": "appliance.dishwasher",
    "microwave": "appliance.microwave", "kettle": "appliance.kettle", "coffee_maker": "appliance.coffee_maker",
    "base_cabinet": "kitchen.cabinet", "hood": "kitchen.hood", "faucet": "kitchen.faucet", "trash": "kitchen.bin",
    "chair": "furniture.chair", "cabinet": "furniture.cabinet", "lamp": "decor.lamp", "plant": "decor.plant",
    "glass": "tableware", "mug": "tableware",
}

# ---------------------------------------------------------------- the room
W, D = 7.2, 4.8                 # inside the walls
HEIGHT = 2.4                    # floor to ceiling
WALL_T = 0.12
WINDOW = (1.6, 2.2, 2.05)       # the balcony window in the south wall: centre x, width, head
DOOR = (3.3, 0.85, 2.0)         # the door to the hall in the north wall: centre x, width, head

# What the tidy-up verifies — the heights it picks from and puts down at.
COFFEE_TABLE = ((1.6, 2.5), (0.5, 1.0, 0.40))       # centre; x, y, height
DINING_TABLE = ((4.3, 2.8), (1.4, 0.8, 0.72))
COUNTER_TOP = 0.91                                  # the cabinets' worktop (ForgeCAD's height)
COUNTER_DEPTH = 0.62
SINK = (2.8, 0.90)                                  # the sink unit: centre y, width
BASIN = (0.42, 0.80, 0.20)                          # inside: front to back, side to side, depth
BASIN_FRONT = 0.05                                  # worktop in front of the basin
DOCK_X = 4.4

# Linear RGB.
OAK = (0.55, 0.36, 0.17)
WALNUT = (0.22, 0.12, 0.06)
PLASTER = (0.78, 0.76, 0.72)
WHITE = (0.82, 0.82, 0.80)
BLACK = (0.03, 0.03, 0.035)
STAINLESS = (0.55, 0.56, 0.58)
CHARCOAL = (0.08, 0.08, 0.09)
LINEN = (0.62, 0.56, 0.46)
RUG = (0.40, 0.36, 0.31)
GLASS = (0.55, 0.70, 0.78)


@dataclass
class House:
    """What the demo reads back off the built flat."""

    #: Pickable things: name -> the obstacles one grasp carries.
    things: dict = field(default_factory=dict)
    #: Where each thing is set down in the sink: name -> (x, y, z) of its underside.
    sink_spots: dict = field(default_factory=dict)


def place(scene: bt.Scene, name: str, key: str, position, facing: float, **kw) -> str:
    """A SimReady package standing at `position` (x, y[, z]) with its front
    turned to `facing` (radians from +x)."""
    kw.setdefault("category", CATEGORY[key])
    return bt.parts.simready(scene, name, position, ASSETS[key], yaw=facing - FRONT.get(key, 0.0), **kw)


def deco(scene: bt.Scene, name: str, size, position, color, *, finish: str | None = None,
         roughness: float = 0.7, metalness: float = 0.0, opacity: float | None = None) -> str:
    """A box that is drawn and never collides — trim, paint, cloth."""
    made = scene.add_box(name, size, position, color=color)
    scene.set_obstacle_enabled(made, False)
    scene.set_obstacle_material(made, metalness=metalness, roughness=roughness, finish=finish,
                                **({} if opacity is None else {"opacity": opacity}))
    return made


def solid(scene: bt.Scene, name: str, size, position, color, *, finish: str | None = None,
          roughness: float = 0.6, metalness: float = 0.0) -> str:
    """A box that is drawn and collides."""
    made = scene.add_box(name, size, position, color=color)
    scene.set_obstacle_material(made, metalness=metalness, roughness=roughness, finish=finish)
    return made


def obstacles_of(scene: bt.Scene, name: str) -> list[str]:
    return [n for n in scene.obstacle_names if n.startswith(name + "/")]


def top_of(scene: bt.Scene, name: str) -> float:
    return max(scene.obstacle_bounds(n)[1][2] for n in obstacles_of(scene, name))


# ------------------------------------------------------------- the shell
def shell(scene: bt.Scene) -> None:
    t = WALL_T
    path = [(-t / 2, -t / 2), (W + t / 2, -t / 2), (W + t / 2, D + t / 2), (-t / 2, D + t / 2)]
    wx, ww, wh = WINDOW
    dx, dw, dh = DOOR
    # Edge 0 runs east along the south wall, edge 2 west along the north one.
    bt.parts.wall(scene, "house/walls", path, closed=True, height=HEIGHT, thickness=t, detail="full",
                  openings=[(0, wx + t / 2, ww, wh), (2, (W + t / 2) - dx, dw, dh)], color=PLASTER,
                  trim_color=WHITE, model="interior wall 2400")
    # The floor: oak boards, drawn just above the studio's own floor plane.
    deco(scene, "house/floor", (W + t, D + t, 0.02), (W / 2, D / 2, -0.009), OAK, finish="wood", roughness=0.55)

    # The balcony window: two sliding glass leaves in a dark frame, and
    # linen curtains drawn back either side of it.
    x0, x1 = wx - ww / 2, wx + ww / 2
    for i, x in enumerate((wx - ww / 4, wx + ww / 4)):
        deco(scene, f"house/window/glass{i}", (ww / 2 - 0.02, 0.012, wh - 0.06), (x, -0.02 + i * 0.03, wh / 2),
             GLASS, roughness=0.05, metalness=0.1, opacity=0.18)
    for name, size, pos in (("head", (ww, 0.06, 0.05), (wx, 0.0, wh - 0.025)),
                            ("sill", (ww, 0.06, 0.03), (wx, 0.0, 0.015)),
                            ("jamb_w", (0.05, 0.06, wh), (x0 + 0.025, 0.0, wh / 2)),
                            ("jamb_e", (0.05, 0.06, wh), (x1 - 0.025, 0.0, wh / 2)),
                            ("mullion", (0.04, 0.05, wh), (wx, 0.0, wh / 2))):
        deco(scene, f"house/window/{name}", size, pos, CHARCOAL, roughness=0.4, metalness=0.6)
    for side, x in (("w", x0 - 0.12), ("e", x1 + 0.12)):
        deco(scene, f"house/curtain_{side}", (0.34, 0.06, 2.25), (x, 0.10, 1.14), LINEN, roughness=0.95)
    deco(scene, "house/curtain_rail", (ww + 0.9, 0.03, 0.03), (wx, 0.10, 2.28), CHARCOAL, metalness=0.5)

    # The door to the hall, shut, with its lever.
    solid(scene, "house/door/leaf", (dw - 0.05, 0.04, dh - 0.02), (dx, D + 0.02, (dh - 0.02) / 2), WHITE)
    deco(scene, "house/door/lever", (0.12, 0.05, 0.02), (dx + dw / 2 - 0.12, D - 0.02, 1.0), STAINLESS,
         metalness=0.9, roughness=0.25)
    scene.set_part("house/door", kind="group", category="structure.door", manufacturer="Generic",
                   model=f"interior door {dw * 1e3:.0f} x {dh * 1e3:.0f}")


# ------------------------------------------------------------- the living
def living(scene: bt.Scene, house: House) -> None:
    # A low walnut TV board under the wall-hung OLED, and the sofa facing them.
    solid(scene, "living/tv_board", (0.40, 1.80, 0.40), (0.20, 2.5, 0.20), WALNUT, finish="wood", roughness=0.5)
    deco(scene, "living/tv_board_line", (0.004, 1.70, 0.02), (0.402, 2.5, 0.22), BLACK)
    scene.set_part("living/tv_board", category="furniture.tv_board", manufacturer="Generic", model="TV board 1800")
    place(scene, "living/tv", "tv", (0.06, 2.5, 0.78), 0.0)
    place(scene, "living/sofa", "sofa", (2.95, 2.5), math.pi)

    # The coffee table the mugs were left on: oak top, black frame.
    (cx, cy), (lx, ly, h) = COFFEE_TABLE
    bt.parts.table(scene, "living/coffee_table", size=(lx, ly, h), position=(cx, cy), top_thickness=0.03, leg=0.03,
                   color=BLACK, top_color=OAK, detail="full", manufacturer="Generic",
                   model=f"coffee table {ly * 1e3:.0f} x {lx * 1e3:.0f} x {h * 1e3:.0f}")
    scene.set_obstacle_material("living/coffee_table/top", roughness=0.45, finish="wood")
    deco(scene, "living/rug", (2.0, 2.4, 0.006), (cx + 0.15, cy, 0.004), RUG, roughness=1.0)

    # A side table with the lamp at the sofa's north end; the plant and the
    # air purifier in the corners; a tall cabinet on the north wall.
    solid(scene, "living/side_table", (0.40, 0.40, 0.50), (2.95, 3.55, 0.25), WALNUT, finish="wood")
    scene.set_part("living/side_table", category="furniture.table", manufacturer="Generic", model="side table 400")
    place(scene, "living/lamp", "lamp", (2.95, 3.55, 0.50), 0.0)
    place(scene, "living/plant", "plant", (0.32, 4.45), 0.0)
    place(scene, "living/air_purifier", "air_purifier", (0.30, 0.40), math.pi / 4)
    place(scene, "living/cabinet", "cabinet", (1.25, D - 0.20), -math.pi / 2)

    # Two mugs left on the coffee table, handles turned away from the robot.
    for side, dx in (("left", -0.13), ("right", 0.13)):
        name = f"living/mug_{side}"
        place(scene, name, "mug", (cx + dx, cy - ly / 2 + 0.12, h), math.pi / 2 - math.pi / 2)
        house.things[f"mug_{side}"] = obstacles_of(scene, name)


# -------------------------------------------------------------- the dining
def dining(scene: bt.Scene, house: House) -> None:
    (cx, cy), (lx, ly, h) = DINING_TABLE
    bt.parts.table(scene, "dining/table", size=(lx, ly, h), position=(cx, cy), top_thickness=0.03, leg=0.05,
                   color=BLACK, top_color=OAK, detail="full", manufacturer="Generic",
                   model=f"dining table {lx * 1e3:.0f} x {ly * 1e3:.0f} x {h * 1e3:.0f}")
    scene.set_obstacle_material("dining/table/top", roughness=0.4, finish="wood")
    for i, x in enumerate((cx - 0.35, cx + 0.35)):
        place(scene, f"dining/chair_s{i}", "chair", (x, cy - ly / 2 - 0.28), math.pi / 2)
        place(scene, f"dining/chair_n{i}", "chair", (x, cy + ly / 2 + 0.28), -math.pi / 2)
        for row, y in (("s", cy - ly / 2 + 0.18), ("n", cy + ly / 2 - 0.18)):
            deco(scene, f"dining/placemat_{row}{i}", (0.40, 0.28, 0.003), (x, y, h + 0.0015), LINEN, roughness=0.9)
    # Two glasses left at the kitchen end of the table.
    for side, dy in (("left", -0.13), ("right", 0.13)):
        name = f"dining/glass_{side}"
        place(scene, name, "glass", (cx + lx / 2 - 0.12, cy + dy, h), 0.0)
        house.things[f"glass_{side}"] = obstacles_of(scene, name)


# ------------------------------------------------------------- the kitchen
def sink_unit(scene: bt.Scene, house: House) -> None:
    """The sink: a cabinet as deep and as high as the ForgeCAD ones beside
    it, its worktop cut round a stainless basin. The basin is what the
    robot verifies — its floor, its walls, how far back it sits — so it is
    built here, not baked."""
    y, width = SINK
    depth, span, sink_depth = BASIN
    front = W - COUNTER_DEPTH
    top, slab = COUNTER_TOP, 0.04
    x0, x1 = front + BASIN_FRONT, front + BASIN_FRONT + depth          # the hole, front to back
    y0, y1 = y - span / 2, y + span / 2
    floor = top - sink_depth
    g = "kitchen/sink"
    # Carcass: sides, back, base and plinth collide; the doors are drawn.
    for side, sy in (("s", y - width / 2 + 0.009), ("n", y + width / 2 - 0.009)):
        solid(scene, f"{g}/side_{side}", (COUNTER_DEPTH, 0.018, top - slab), (front + COUNTER_DEPTH / 2, sy, (top - slab) / 2), WHITE)
    solid(scene, f"{g}/back", (0.018, width, top - slab), (W - 0.009, y, (top - slab) / 2), WHITE)
    solid(scene, f"{g}/plinth", (0.018, width, 0.10), (front + 0.06, y, 0.05), BLACK)
    solid(scene, f"{g}/base", (COUNTER_DEPTH - 0.06, width, 0.018), (front + 0.03 + COUNTER_DEPTH / 2, y, 0.109), WHITE)
    for i, dy in enumerate((-width / 4, width / 4)):
        deco(scene, f"{g}/door{i}", (0.019, width / 2 - 0.006, top - slab - 0.12), (front + 0.0095, y + dy, 0.10 + (top - slab - 0.10) / 2),
             OAK, finish="wood", roughness=0.5)
        deco(scene, f"{g}/pull{i}", (0.02, 0.012, 0.16), (front - 0.01, y + dy - math.copysign(width / 4 - 0.05, dy), top - slab - 0.14),
             STAINLESS, metalness=0.9, roughness=0.25)
    # Worktop round the hole: in front, behind, either side.
    for name, (sx, sy), (px, py) in (
        ("front", (x0 - front, width), ((front + x0) / 2, y)),
        ("back", (W - x1, width), ((x1 + W) / 2, y)),
        ("south", (depth, y0 - (y - width / 2)), ((x0 + x1) / 2, (y - width / 2 + y0) / 2)),
        ("north", (depth, (y + width / 2) - y1), ((x0 + x1) / 2, (y1 + y + width / 2) / 2)),
    ):
        solid(scene, f"{g}/top_{name}", (sx, sy, slab), (px, py, top - slab / 2), (0.70, 0.70, 0.68), roughness=0.35)
    # The basin: a floor and four walls of stainless sheet under the worktop.
    s = 0.012
    solid(scene, f"{g}/basin/floor", (depth, span, s), ((x0 + x1) / 2, y, floor - s / 2), STAINLESS, metalness=0.85, roughness=0.3)
    wall_h = top - slab - floor + s
    for name, size, pos in (
        ("front", (s, span, wall_h), (x0 + s / 2, y, floor - s + wall_h / 2)),
        ("back", (s, span, wall_h), (x1 - s / 2, y, floor - s + wall_h / 2)),
        ("south", (depth, s, wall_h), ((x0 + x1) / 2, y0 + s / 2, floor - s + wall_h / 2)),
        ("north", (depth, s, wall_h), ((x0 + x1) / 2, y1 - s / 2, floor - s + wall_h / 2)),
    ):
        solid(scene, f"{g}/basin/{name}", size, pos, STAINLESS, metalness=0.85, roughness=0.3)
    deco(scene, f"{g}/basin/drain", (0.08, 0.08, 0.002), ((x0 + x1) / 2, y, floor + 0.001), CHARCOAL, metalness=0.8)
    scene.set_part(g, kind="group", category="kitchen.sink", manufacturer="Generic",
                   model=f"sink unit {width * 1e3:.0f} with basin {span * 1e3:.0f} x {depth * 1e3:.0f} x {sink_depth * 1e3:.0f}")
    place(scene, "kitchen/faucet", "faucet", ((x1 + W) / 2 + 0.01, y, top), math.pi)
    # Where the dishes go: one row across the front of the basin, the mugs in
    # the middle, the glasses either side of them.
    row = x0 + s + 0.07
    for side, sign in (("left", 1.0), ("right", -1.0)):
        house.sink_spots[f"mug_{side}"] = (row, y + sign * 0.10, floor)
        house.sink_spots[f"glass_{side}"] = (row, y + sign * 0.28, floor)


def kitchen(scene: bt.Scene, house: House) -> None:
    sink_y, sink_w = SINK
    depth = COUNTER_DEPTH
    x = W - depth / 2
    # The run along the east wall, south to north: a cabinet with the hob
    # and the hood, the dishwasher under a strip of worktop, the sink, a
    # cabinet with the microwave, the refrigerator.
    south = sink_y - sink_w / 2 - 0.61 - 0.60
    place(scene, "kitchen/cabinet_s", "base_cabinet", (x, south + 0.30), math.pi)
    deco(scene, "kitchen/hob", (0.52, 0.56, 0.006), (x + 0.02, south + 0.30, COUNTER_TOP + 0.003), BLACK,
         roughness=0.1, metalness=0.2)
    for i, (dx, dy) in enumerate(((-0.12, -0.12), (-0.12, 0.12), (0.12, 0.0))):
        deco(scene, f"kitchen/hob_ring{i}", (0.16, 0.16, 0.001), (x + 0.02 + dx, south + 0.30 + dy, COUNTER_TOP + 0.0065),
             (0.12, 0.12, 0.13), roughness=0.3)
    place(scene, "kitchen/hood", "hood", (W - 0.25, south + 0.30, 1.55), math.pi)
    place(scene, "kitchen/dishwasher", "dishwasher", (W - 0.34, south + 0.60 + 0.305), math.pi)
    solid(scene, "kitchen/worktop_dw", (depth, 0.61, 0.04), (x, south + 0.60 + 0.305, COUNTER_TOP - 0.02),
          (0.70, 0.70, 0.68), roughness=0.35)
    scene.set_part("kitchen/worktop_dw", category="kitchen.worktop", manufacturer="Generic", model="worktop 610")
    place(scene, "kitchen/kettle", "kettle", (W - 0.22, south + 0.60 + 0.20, COUNTER_TOP), math.pi)
    place(scene, "kitchen/coffee_maker", "coffee_maker", (W - 0.20, south + 0.60 + 0.45, COUNTER_TOP), math.pi)
    sink_unit(scene, house)
    north = sink_y + sink_w / 2
    place(scene, "kitchen/cabinet_n", "base_cabinet", (x, north + 0.30), math.pi)
    place(scene, "kitchen/microwave", "microwave", (W - 0.25, north + 0.30, COUNTER_TOP), math.pi)
    place(scene, "kitchen/refrigerator", "refrigerator", (W - 0.47, north + 0.63 + 0.465), math.pi)
    place(scene, "kitchen/trash", "trash", (W - 0.22, south - 0.30), math.pi)

    # Wall cupboards over the dishwasher, the sink and the cabinet; a tiled
    # splashback under them.
    y0, y1 = south + 0.60, north + 0.60
    for i in range(int(round((y1 - y0) / 0.5))):
        yc = y0 + (i + 0.5) * (y1 - y0) / round((y1 - y0) / 0.5)
        solid(scene, f"kitchen/wall_cupboard/{i}", (0.34, (y1 - y0) / round((y1 - y0) / 0.5) - 0.006, 0.70),
              (W - 0.17, yc, 1.85), WHITE, roughness=0.4)
        deco(scene, f"kitchen/wall_cupboard/pull{i}", (0.015, 0.14, 0.012), (W - 0.345, yc, 1.52), STAINLESS,
             metalness=0.9, roughness=0.25)
    scene.set_part("kitchen/wall_cupboard", kind="group", category="kitchen.cabinet", manufacturer="Generic",
                   model="wall cupboard 340 x 700")
    deco(scene, "kitchen/splashback", (0.008, y1 - south, 0.60), (W - 0.004, (south + y1) / 2, COUNTER_TOP + 0.30),
         (0.88, 0.88, 0.86), roughness=0.2)


# ---------------------------------------------------------------- the dock
def dock(scene: bt.Scene) -> None:
    solid(scene, "dock/body", (0.46, 0.20, 0.14), (DOCK_X, 0.10, 0.07), (0.16, 0.17, 0.19), roughness=0.4)
    deco(scene, "dock/led", (0.30, 0.004, 0.012), (DOCK_X, 0.202, 0.11), (0.10, 0.65, 0.95), roughness=0.2)
    deco(scene, "dock/plate", (0.62, 0.42, 0.004), (DOCK_X, 0.32, 0.003), (0.10, 0.10, 0.11), roughness=0.8)
    scene.set_part("dock", kind="group", category="vehicle.charger", manufacturer="Generic", model="charging dock")


def build(scene: bt.Scene) -> House:
    """The whole flat, furnished; returns what the tidy-up reads back."""
    house = House()
    shell(scene)
    living(scene, house)
    dining(scene, house)
    kitchen(scene, house)
    dock(scene)
    return house
