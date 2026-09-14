"""The cover-bolting example's own designs, built at run time: the panel's
bracket, the locating nest and stock pocket, and the Compute Box. The bench,
the E-stop station, the presenter and the housing-and-cover set are catalog
products and draw themselves. See cover_bolting_demo.md for product
references and the distinction between purchased equipment and demo designs.
"""

from __future__ import annotations


import botrail as bt

METAL = (0.48, 0.51, 0.55)
DARK = (0.025, 0.032, 0.040)
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


def panel(scene, position, robot, *, bench_top, catalog):
    """XALK178F on an authored bench bracket.

    The station comes from its catalog pack — its enclosure drawn by the
    pack's trim, the operator by the generator. Its two NC contacts are
    represented by the demo's single abstract E-stop lane; this geometry
    does not introduce a safety-controller implementation.
    """
    x, y, z = position
    panel = bt.parts.operator_panel(scene, "panel", position, buttons=("estop",), catalog=catalog, watch_robots=[robot],
                                    source_url=PANEL_SOURCE, contacts="2 NC; one abstract E-stop lane in this demo")
    box(scene, "panel/bracket/post", (0.025, 0.025, z - bench_top),
        (x, y + 0.038, (z + bench_top) / 2), DARK, collision=True)
    box(scene, "panel/bracket/foot", (0.10, 0.10, 0.006),
        (x, y + 0.005, bench_top + 0.003), DARK, collision=True)
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
