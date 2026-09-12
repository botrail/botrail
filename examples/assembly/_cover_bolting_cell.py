"""Dimensioned equipment and authored detail for the cover-bolting example.

All geometry is built at run time. See cover_bolting_demo.md for product
references and the distinction between purchased equipment and demo designs.
"""

from __future__ import annotations

import hashlib
import math
import tempfile
from pathlib import Path

import botrail as bt

PAINT = (0.10, 0.22, 0.17)
METAL = (0.48, 0.51, 0.55)
DARK = (0.025, 0.032, 0.040)
YELLOW = (0.95, 0.62, 0.015)
BENCH = (1.5, 0.75, 0.74)  # TRUSCO AE-1500, manufacturer dimensions
BENCH_SOURCE = "https://www.orange-book.com/ja/c/products/index.html?itemCd=AE1500++++++++++++++++++++++++8500"
PANEL_SOURCE = "https://www.se.com/us/en/product/XALK178F/"


def box(scene, name, size, at, color=METAL, *, metal=0.0, rough=0.45, collision=False):
    scene.add_box(name, size=size, position=at, color=color)
    scene.set_obstacle_material(name, metalness=metal, roughness=rough)
    scene.set_obstacle_enabled(name, collision)
    return name


def cylinder(scene, name, radius, height, at, color=METAL, *, quaternion=None,
             metal=0.85, rough=0.3, collision=False):
    scene.add_cylinder(name, radius, height, at, quaternion=quaternion, color=color)
    scene.set_obstacle_material(name, metalness=metal, roughness=rough)
    scene.set_obstacle_enabled(name, collision)
    return name


def bench(scene, position):
    """AE-1500's laminate board, inset steel frame and adjustable feet."""
    x, y = position
    width, depth, h = BENCH
    table = bt.parts.table(scene, "bench", size=BENCH, position=position,
                           top_thickness=0.021, leg=0.045, color=PAINT,
                           model="AE-1500", manufacturer="TRUSCO", mass_kg=33.5,
                           source_url=BENCH_SOURCE, geometry="dimensioned reference; frame details approximate")
    scene.set_obstacle_color("bench/top", (0.66, 0.64, 0.57))
    scene.set_obstacle_material("bench/top", metalness=0.0, roughness=0.62)
    # The maker's frame is inset from the board, not four posts at its corners.
    for i, (sx, sy) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        px, py = x + sx * (width / 2 - 0.085), y + sy * (depth / 2 - 0.070)
        scene.remove_obstacle(f"bench/leg{i}")
        box(scene, f"bench/leg{i}", (0.045, 0.045, h - 0.056),
            (px, py, (h - 0.056) / 2 + 0.035), PAINT, collision=True)
        cylinder(scene, f"bench/feet/stem{i}", 0.008, 0.029, (px, py, 0.028))
        cylinder(scene, f"bench/feet/pad{i}", 0.027, 0.014, (px, py, 0.007), DARK, metal=0, rough=0.8)
        for z in (h - 0.060, 0.265):
            cylinder(scene, f"bench/hardware/bolt{i}_{z}", 0.005, 0.003,
                     (px, py + sy * 0.0235, z), quaternion=(math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
    for side in (-1, 1):
        box(scene, f"bench/frame/apron{side}", (width - 0.15, 0.025, 0.07),
            (x, y + side * (depth / 2 - 0.070), h - 0.056), PAINT, collision=True)
        box(scene, f"bench/frame/end{side}", (0.025, depth - 0.12, 0.07),
            (x + side * (width / 2 - 0.085), y, h - 0.056), PAINT, collision=True)
        box(scene, f"bench/frame/lower_end{side}", (0.025, depth - 0.12, 0.06),
            (x + side * (width / 2 - 0.085), y, 0.265), PAINT, collision=True)
    box(scene, "bench/frame/rear_stretcher", (width - 0.15, 0.025, 0.09),
        (x, y + depth / 2 - 0.070, 0.265), PAINT, collision=True)
    for side in (-1, 1):
        box(scene, f"bench/edge/long{side}", (width, 0.002, 0.018),
            (x, y + side * (depth / 2 + 0.001), h - 0.0105), (0.17, 0.16, 0.14))
    return table


def panel(scene, position, robot):
    """XALK178F reference enclosure on an authored bench bracket.

    Its two NC contacts are represented by the demo's single abstract E-stop
    lane; this geometry does not introduce a safety-controller implementation.
    """
    x, y, z = position
    panel = bt.parts.operator_panel(
        scene, "panel", position, buttons=("estop",), size=(0.068, 0.068),
        thickness=0.053, proud=0.0158, watch_robots=[robot],
        model="XALK178F", manufacturer="Schneider Electric", color=(0.16, 0.17, 0.18),
        source_url=PANEL_SOURCE, contacts="2 NC; one abstract E-stop lane in this demo",
    )
    box(scene, "panel/lid", (0.068, 0.003, 0.068), (x, y - 0.027, z), YELLOW)
    q = (math.sqrt(0.5), 0, 0, math.sqrt(0.5))
    for i, (sx, sz) in enumerate(((-1, -1), (1, 1))):
        cylinder(scene, f"panel/lid_screw{i}", 0.0024, 0.001,
                 (x + sx * 0.025, y - 0.029, z + sz * 0.025), quaternion=q)
    box(scene, "panel/bracket/post", (0.025, 0.025, z - BENCH[2]),
        (x, y + 0.038, (z + BENCH[2]) / 2), DARK, collision=True)
    box(scene, "panel/bracket/foot", (0.10, 0.10, 0.006),
        (x, y + 0.005, BENCH[2] + 0.003), DARK, collision=True)
    scene.set_part("panel/bracket", kind="group", manufacturer="botrail", model="CB-PANEL-BRACKET",
                   category="fixture", description="custom bench-mounted E-stop bracket")
    scene.set_part("panel/estop", kind="sensor", manufacturer="Schneider Electric",
                   model="XALK178F actuator (included)", category="hmi.button",
                   description="included in the complete control station, not a separate purchase")
    return panel


def fixtures(scene, *, housing_xy, stock_xy, top, housing_size, cover_size):
    """Side-locating nest and cover stock pocket, clear of the joint faces."""
    hx, hy = housing_xy
    sx, sy = stock_xy
    w, d, _ = housing_size
    # Recessed fixture: the workpiece remains on the original table datum.
    for sign in (-1, 1):
        box(scene, f"fixture/side{sign}", (0.018, d + 0.034, 0.018),
            (hx + sign * (w / 2 + 0.012), hy, top + 0.009), DARK, collision=True)
        for j, dy in enumerate((-d / 2 - 0.010, d / 2 + 0.010)):
            cylinder(scene, f"fixture/bolt{sign}_{j}", 0.004, 0.004,
                     (hx + sign * (w / 2 + 0.012), hy + dy, top + 0.020))
        box(scene, f"fixture/end{sign}", (w + 0.006, 0.012, 0.012),
            (hx, hy + sign * (d / 2 + 0.009), top + 0.006), DARK, collision=True)
    scene.set_part("fixture", kind="group", manufacturer="botrail", model="GH-160 locating nest",
                   category="fixture", description="custom side-locating fixture; table face is Z datum")
    cw, cd, _ = cover_size
    for sign in (-1, 1):
        box(scene, f"stock/side{sign}", (0.010, cd + 0.018, 0.006),
            (sx + sign * (cw / 2 + 0.007), sy, top + 0.003), (0.025, 0.16, 0.21), collision=True)
        box(scene, f"stock/end{sign}", (cw + 0.024, 0.008, 0.006),
            (sx, sy + sign * (cd / 2 + 0.006), top + 0.003), (0.025, 0.16, 0.21), collision=True)
    scene.set_part("stock", kind="group", manufacturer="botrail", model="GH-160 cover nest",
                   category="fixture", description="custom polymer stock pocket")


def feeder_detail(scene, feeder):
    """Sheet-metal seams, hopper lid, vents and pickup rail inside its envelope."""
    name = feeder.name
    (lo, hi) = scene.obstacle_bounds(f"{name}/body")
    x, y = (lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2
    w, d, h = (b - a for a, b in zip(lo, hi))
    z = hi[2]
    scene.set_obstacle_color(f"{name}/body", (0.57, 0.59, 0.58))
    scene.set_obstacle_material(f"{name}/body", metalness=0.15, roughness=0.48)
    box(scene, f"{name}/detail/base", (w + 0.004, d + 0.004, 0.008),
        (x, y, lo[2] + 0.004), DARK)
    box(scene, f"{name}/detail/lid_seam", (w - 0.010, d * 0.43, 0.002),
        (x, y + d * 0.22, z + 0.001), DARK)
    box(scene, f"{name}/detail/hopper_lid", (w - 0.018, d * 0.39, 0.004),
        (x, y + d * 0.22, z + 0.004), (0.21, 0.25, 0.28), metal=0.3)
    box(scene, f"{name}/detail/handle", (0.040, 0.010, 0.006),
        (x, y + d * 0.22, z + 0.009), DARK)
    for i in range(9):
        box(scene, f"{name}/detail/vent{i}", (0.004, 0.001, 0.025),
            (x - 0.070 + i * 0.012, lo[1] - 0.0006, lo[2] + h * 0.40), DARK)
    for i, dx in enumerate((-w / 2 + 0.014, w / 2 - 0.014)):
        for j, dz in enumerate((0.018, h - 0.018)):
            cylinder(scene, f"{name}/detail/fastener{i}_{j}", 0.0028, 0.0015,
                     (x + dx, lo[1] - 0.001, lo[2] + dz),
                     quaternion=(math.sqrt(0.5), 0, 0, math.sqrt(0.5)))
    box(scene, f"{name}/detail/control_bezel", (0.036, 0.002, 0.020),
        (x + w * 0.28, lo[1] - 0.0015, z - 0.038), DARK)
    box(scene, f"{name}/detail/power_switch", (0.012, 0.003, 0.013),
        (x + w * 0.28, lo[1] - 0.004, z - 0.038), (0.06, 0.25, 0.10))


def workpiece_detail(scene, joint, *, housing_size, cover_size, boss):
    """Cast walls/ribs and a machined cover, generated from the cell dimensions.

    A separate display mesh keeps the existing collision hull, grasp and hole
    windows. It is generated in Python at run time, including for catalog mode,
    and bundled by save_project like any other visual asset.
    """
    B, C = bt.parts.Box, bt.parts.Cylinder
    w, d, h = housing_size
    cw, cd, ct = cover_size
    diameter, height = boss

    def rounded_plate(width, depth, thick, z, radius):
        return [B((width - 2 * radius, depth, thick), (0, 0, z + thick / 2)),
                B((width, depth - 2 * radius, thick), (0, 0, z + thick / 2)),
                *[C(radius, thick, (sx * (width / 2 - radius), sy * (depth / 2 - radius), z))
                  for sx in (-1, 1) for sy in (-1, 1)]]

    # Keep the cavity inside the original block. Two substantial machined
    # flanges and narrow ribs read as a casting, rather than a solid billet.
    housing = rounded_plate(w, d, 0.008, -h / 2, 0.008)
    housing += [B((w - 0.018, 0.014, h - 0.016), (0, sy * (d / 2 - 0.015), 0))
                for sy in (-1, 1)]
    housing += [B((0.018, d - 0.016, h - 0.008), (sx * (w / 2 - 0.009), 0, 0.004))
                for sx in (-1, 1)]
    housing += [B((w - 0.036, 0.020, 0.008), (0, sy * (d / 2 - 0.010), h / 2 - 0.004))
                for sy in (-1, 1)]
    for x in (-w * 0.3125, -w * 0.15625, 0, w * 0.15625, w * 0.3125):
        for sign in (-1, 1):
            housing.append(B((0.004, 0.007, h - 0.020), (x, sign * (d / 2 - 0.0035), -0.002)))
    cover = rounded_plate(cw, cd, ct, 0, 0.008)
    cover += [C(diameter / 2 + 0.004, 0.003, (0, 0, ct)),
              C(diameter / 2, height - 0.001, (0, 0, ct)),
              C(diameter / 2 - 0.0008, 0.001, (0, 0, ct + height - 0.001))]
    identity = (1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1.)
    for name, solids, color, rough in ((joint.a, housing, (0.38, 0.41, 0.44), 0.52),
                                        (joint.b, cover, (0.64, 0.67, 0.70), 0.29)):
        key = hashlib.sha256(repr(("v2", solids, color, rough)).encode()).hexdigest()[:20]
        cache = Path(tempfile.gettempdir()) / "botrail-cover-bolting"
        cache.mkdir(exist_ok=True)
        path = cache / f"{key}.usda"
        if not path.exists():
            source = bt.Scene()
            bt.parts.compound(source, "detail", solids, (0, 0, 0), color=color, segments=64)
            source.export_usd(path)
        scene.set_obstacle_visual_asset(name, path, "/World/Env/detail", identity)
        scene.set_obstacle_color(name, color)
        scene.set_obstacle_material(name, metalness=0.85, roughness=rough)
    # Thread entrances are fixed to the stationary housing and disappear
    # beneath the seated cover. Their centres come from the joint, not a
    # second list of hole coordinates.
    for hole in joint.order:
        (x, y, z), _ = joint.hole_pose(hole)
        cylinder(scene, f"{joint.a}/thread/{hole}", joint.fastener.thread_mm / 2000,
                 0.0002, (x, y, z - ct + 0.0001), DARK, metal=0, rough=0.9)


def compute_box(scene, node, position):
    """Robot Kit 113761: Compute Box, supply and supplied cable set.

    Box dimensions from OnRobot's UR Screwdriver manual §8.2.4.2.
    The simulation's driver I/O node remains an abstract controller program.
    """
    x, y, z = position
    box(scene, f"{node}/body", (0.1119, 0.0883, 0.0325),
        (x, y, z + 0.01625), METAL, metal=0.8, collision=True)
    for side in (-1, 1):
        box(scene, f"{node}/end{side}", (0.1119, 0.006, 0.0325),
            (x, y + side * 0.044, z + 0.01625), DARK)
    box(scene, f"{node}/ethernet", (0.016, 0.008, 0.013),
        (x + 0.035, y - 0.049, z + 0.016), DARK)
    box(scene, f"{node}/status", (0.020, 0.012, 0.0008),
        (x, y, z + 0.0329), (0.04, 0.34, 0.60))
    scene.set_part(node, kind="io_node", manufacturer="OnRobot", model="Robot Kit (113761), including Compute Box",
                   category="plc", part_number="113761", source_url="https://b2b.onrobot.com/robot-kit/",
                   description="Compute Box, power supply, 5 m M12-to-M8 and Ethernet cables; abstract simulation I/O",
                   voltage_v=24, cable_m=5, electrical_route="external supply -> Compute Box -> Dual QC -> tools",
                   commissioning="verify current, firmware/URCap and pinout; simulated handshake is not a device driver")
