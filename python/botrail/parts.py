"""Standard structures, generated from parameters: fences, tables, pedestals,
conveyor bodies, pallets, light curtains — the scenery every cell has and
nobody wants to model.

Each generator composes the ordinary scene API — `add_box`, `add_frame`,
`add_conveyor`, `add_beam_sensor`, `set_part` — so what it builds is plain
residents: boxes under a name prefix (`fence/panels/n0`, `table/top`), a
frame where the next thing mounts, a device or a sensor where one belongs,
and a *part* on the group with the quantity, so the BOM counts panels and
posts and the layout sheet labels the assembly once. Change a parameter and
the geometry, the BOM line and the sheet change together.

    bt.parts.fence(scene, "fence", path=[(-2, -2), (2, -2), (2, 2), (-2, 2)],
                   height=2.0, panel_pitch=1.0, door=(1, 1), model="ST20")
    ped = bt.parts.pedestal(scene, "pedestal", height=0.5, position=(0, 0))
    scene.set_robot_base_pose(*scene.frame(ped.frames[0]))

botrail does not model shapes. These are boxes arranged by parameters, and
that is the point: a fence is *panels of a pitch along a path*, a table is
*a top on legs* — the meaning is what the BOM and the sheet need, and the
few centimetres a real profile differs by change nothing a cell verifies.
Anything with a shape of its own comes in from CAD as a mesh (see the
Geometry Provider pattern in the standard-parts guide) and gets its
identity the same way, with `set_part`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]
Color = tuple[float, float, float]

# Muted linear-RGB colours: galvanised fence, dark posts, grey steel, wood.
FENCE_PANEL: Color = (0.55, 0.58, 0.60)
FENCE_POST: Color = (0.16, 0.17, 0.19)
STEEL: Color = (0.42, 0.44, 0.47)
DARK_STEEL: Color = (0.20, 0.21, 0.23)
WOOD: Color = (0.52, 0.36, 0.18)
BELT: Color = (0.10, 0.10, 0.11)


@dataclass
class Built:
    """What a generator put into the scene, by name — the obstacles, and the
    frames, devices and sensors that came with them — so the caller can
    mount on the frame, drive the device, or take the whole thing down."""

    name: str
    obstacles: list[str] = field(default_factory=list)
    frames: list[str] = field(default_factory=list)
    devices: list[str] = field(default_factory=list)
    sensors: list[str] = field(default_factory=list)

    def remove(self, scene) -> None:
        """Takes everything this generator added out of the scene (parts go
        with their residents)."""
        for name in self.sensors:
            scene.remove_sensor(name)
        for name in self.devices:
            scene.remove_device(name)
        for name in self.obstacles:
            scene.remove_obstacle(name)
        for name in self.frames:
            scene.remove_frame(name)


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def _identity(model: Optional[str], manufacturer: Optional[str], attrs: dict) -> dict:
    out = dict(attrs)
    if model is not None:
        out["model"] = model
    if manufacturer is not None:
        out["manufacturer"] = manufacturer
    return out


# --------------------------------------------------------------------- fence


def fence(
    scene,
    name: str,
    path: Sequence[Point2],
    *,
    height: float = 2.0,
    panel_pitch: float = 1.0,
    post: float = 0.06,
    panel_thickness: float = 0.03,
    closed: bool = True,
    door: Optional[tuple[int, int]] = None,
    door_model: Optional[str] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    post_model: Optional[str] = None,
    panel_color: Color = FENCE_PANEL,
    post_color: Color = FENCE_POST,
    **attributes,
) -> Built:
    """A safety fence along `path` (floor corners, metres): each edge is
    split into panels of about `panel_pitch` (the pitch is stretched so an
    edge takes a whole number), with a post at every corner and between
    panels. `closed` joins the last corner back to the first. `door=(edge,
    panel)` makes that panel the door — its own obstacle `<name>/door` and
    its own BOM line. Two parts are pinned: `<name>` (the panels — the fence
    line, `qty` = panels, `model` / `manufacturer` / attributes such as
    `mass_kg` per panel) and `<name>/posts` (`qty` = posts). Returns the
    names it made."""
    pts = [tuple(map(float, p)) for p in path]
    if len(pts) < 2:
        raise ValueError("fence: path needs at least two corners")
    if panel_pitch <= 0 or height <= 0:
        raise ValueError("fence: panel_pitch and height must be positive")
    edges = list(zip(pts, pts[1:] + ([pts[0]] if closed and len(pts) > 2 else [])))
    built = Built(name)
    panels = 0
    posts = 0
    # Posts at every distinct corner (a closed loop shares its ends).
    for i, (x, y) in enumerate(pts):
        pname = scene.add_box(
            f"{name}/posts/c{i}", size=(post, post, height), position=(x, y, height / 2), color=post_color
        )
        built.obstacles.append(pname)
        posts += 1
    for e, ((x0, y0), (x1, y1)) in enumerate(edges):
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-9:
            continue
        count = max(1, round(length / panel_pitch))
        pitch = length / count
        yaw = math.atan2(y1 - y0, x1 - x0)
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        for i in range(count):
            cx = x0 + ux * pitch * (i + 0.5)
            cy = y0 + uy * pitch * (i + 0.5)
            is_door = door is not None and door == (e, i)
            oname = f"{name}/door" if is_door else f"{name}/panels/e{e}_{i}"
            oname = scene.add_box(
                oname,
                size=(pitch - post, panel_thickness, height),
                position=(cx, cy, height / 2),
                quaternion=_yaw_quat(yaw),
                color=panel_color,
            )
            built.obstacles.append(oname)
            if is_door:
                scene.set_part(oname, category="structure.door", model=door_model, qty=1)
            else:
                panels += 1
            # An intermediate post between panels (not at the edge ends —
            # those are corners).
            if i < count - 1:
                px = x0 + ux * pitch * (i + 1)
                py = y0 + uy * pitch * (i + 1)
                pname = scene.add_box(
                    f"{name}/posts/e{e}_{i}", size=(post, post, height), position=(px, py, height / 2), color=post_color
                )
                built.obstacles.append(pname)
                posts += 1
    scene.set_part(
        name, kind="group", category="structure.fence", qty=max(panels, 1),
        **_identity(model, manufacturer, attributes),
    )
    scene.set_part(f"{name}/posts", kind="group", category="structure.fence.post", qty=posts, model=post_model)
    return built


# --------------------------------------------------------------------- table


def table(
    scene,
    name: str,
    size: Point3,
    position: Point2 | Point3,
    *,
    top_thickness: float = 0.03,
    leg: float = 0.04,
    yaw: float = 0.0,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = STEEL,
    **attributes,
) -> Built:
    """A table `size = (length, width, height)` standing on the floor at
    `position` (its centre, x, y[, floor z]): a top of `top_thickness` on
    four legs. Adds the frame `<name>/top` at the centre of the top face —
    where a fixture or a workpiece sits — and pins one part
    (`structure.table`) on the group."""
    lx, wy, h = size
    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    built = Built(name)
    built.obstacles.append(
        scene.add_box(f"{name}/top", size=(lx, wy, top_thickness), position=(x, y, z0 + h - top_thickness / 2),
                      quaternion=q, color=color)
    )
    c, s = math.cos(yaw), math.sin(yaw)
    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        dx, dy = sx * (lx / 2 - leg / 2), sy * (wy / 2 - leg / 2)
        px, py = x + c * dx - s * dy, y + s * dx + c * dy
        built.obstacles.append(
            scene.add_box(f"{name}/leg{i}", size=(leg, leg, h - top_thickness),
                          position=(px, py, z0 + (h - top_thickness) / 2), quaternion=q, color=color)
        )
    scene.add_frame(f"{name}/top", position=(x, y, z0 + h), quaternion=q)
    built.frames.append(f"{name}/top")
    scene.set_part(name, kind="group", category="structure.table", **_identity(model, manufacturer, attributes))
    return built


# ------------------------------------------------------------------ pedestal


def pedestal(
    scene,
    name: str,
    height: float,
    position: Point2 | Point3,
    *,
    top: Point2 = (0.35, 0.35),
    base: Point2 = (0.5, 0.5),
    column: float = 0.2,
    plate: float = 0.02,
    yaw: float = 0.0,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = DARK_STEEL,
    **attributes,
) -> Built:
    """A robot pedestal: base plate, column, top plate, `height` from floor
    to the top face at `position`. Adds the frame `<name>/mount` at the top
    centre — the robot's base pose (`scene.set_robot_base_pose(*scene.frame(
    "<name>/mount"))`) — and pins one part (`structure.pedestal`)."""
    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    built = Built(name)
    built.obstacles.append(
        scene.add_box(f"{name}/base", size=(base[0], base[1], plate), position=(x, y, z0 + plate / 2),
                      quaternion=q, color=color)
    )
    shaft = max(height - 2 * plate, plate)
    built.obstacles.append(
        scene.add_box(f"{name}/column", size=(column, column, shaft), position=(x, y, z0 + plate + shaft / 2),
                      quaternion=q, color=color)
    )
    built.obstacles.append(
        scene.add_box(f"{name}/top", size=(top[0], top[1], plate), position=(x, y, z0 + height - plate / 2),
                      quaternion=q, color=color)
    )
    scene.add_frame(f"{name}/mount", position=(x, y, z0 + height), quaternion=q)
    built.frames.append(f"{name}/mount")
    scene.set_part(name, kind="group", category="structure.pedestal", **_identity(model, manufacturer, attributes))
    return built


# ------------------------------------------------------------------ conveyor


def conveyor(
    scene,
    name: str,
    length: float,
    width: float,
    position: Point3,
    *,
    direction: Point2 = (1.0, 0.0),
    speed: float = 0.2,
    running: bool = False,
    zone_height: float = 0.15,
    belt_thickness: float = 0.05,
    rail: float = 0.03,
    legs: bool = True,
    leg: float = 0.05,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = DARK_STEEL,
    **attributes,
) -> Built:
    """A belt conveyor: `length` along `direction`, `width` across, its
    belt surface centred at `position` (x, y, z of the surface). Builds the
    body — belt slab, two side rails, legs at both ends — as obstacles under
    `<name>/`, and the conveyor *device* `<name>` whose transport zone sits
    on the belt (`zone_height` tall, `speed` along `direction`). The part is
    pinned on the device (`conveyor`): the body is its geometry, not a
    second product. Adds the frames `<name>/infeed` and `<name>/outfeed` at
    the belt ends."""
    dx, dy = float(direction[0]), float(direction[1])
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        raise ValueError("conveyor: direction must be a non-zero xy vector")
    dx, dy = dx / norm, dy / norm
    yaw = math.atan2(dy, dx)
    q = _yaw_quat(yaw)
    x, y, z = (float(v) for v in position)
    built = Built(name)
    built.obstacles.append(
        scene.add_box(f"{name}/belt", size=(length, width, belt_thickness), position=(x, y, z - belt_thickness / 2),
                      quaternion=q, color=BELT)
    )
    nx, ny = -dy, dx  # across the belt
    for side, s in (("rail_l", 1.0), ("rail_r", -1.0)):
        ox, oy = nx * s * (width / 2 + rail / 2), ny * s * (width / 2 + rail / 2)
        built.obstacles.append(
            scene.add_box(f"{name}/{side}", size=(length, rail, belt_thickness + 0.04),
                          position=(x + ox, y + oy, z - belt_thickness / 2 + 0.02), quaternion=q, color=color)
        )
    if legs and z - belt_thickness > leg:
        h = z - belt_thickness
        for end, e in (("in", -1.0), ("out", 1.0)):
            for side, s in (("l", 1.0), ("r", -1.0)):
                ex, ey = dx * e * (length / 2 - leg), dy * e * (length / 2 - leg)
                ox, oy = nx * s * (width / 2 - leg / 2), ny * s * (width / 2 - leg / 2)
                built.obstacles.append(
                    scene.add_box(f"{name}/leg_{end}{side}", size=(leg, leg, h),
                                  position=(x + ex + ox, y + ey + oy, h / 2), quaternion=q, color=color)
                )
    scene.add_conveyor(
        name,
        zone_position=(x, y, z + zone_height / 2),
        zone_size=(length, width, zone_height),
        velocity=(dx * speed, dy * speed, 0.0),
        zone_quaternion=q,
        running=running,
    )
    built.devices.append(name)
    for end, e in (("infeed", -1.0), ("outfeed", 1.0)):
        fname = f"{name}/{end}"
        scene.add_frame(fname, position=(x + dx * e * length / 2, y + dy * e * length / 2, z), quaternion=q)
        built.frames.append(fname)
    scene.set_part(name, kind="device", category="conveyor", **_identity(model, manufacturer, attributes))
    return built


# -------------------------------------------------------------------- pallet


def pallet(
    scene,
    name: str,
    position: Point2 | Point3,
    *,
    size: Point3 = (1.2, 1.0, 0.144),
    deck_boards: int = 5,
    yaw: float = 0.0,
    model: Optional[str] = "EPAL 1",
    manufacturer: Optional[str] = None,
    color: Color = WOOD,
    **attributes,
) -> Built:
    """A wooden pallet `size = (length, width, height)` on the floor at
    `position` (centre): three bottom boards, nine blocks, `deck_boards`
    top boards. Adds the frame `<name>/top` at the centre of the deck and
    pins one part (`pallet`)."""
    lx, wy, h = size
    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)

    def place(local_x: float, local_y: float) -> tuple[float, float]:
        return x + c * local_x - s * local_y, y + s * local_x + c * local_y

    built = Built(name)
    board = 0.022
    block = h - 2 * board
    # Bottom boards run along x at three y positions; blocks sit on them;
    # deck boards run along y? No — EPAL: deck boards along the length,
    # bottom boards across. Kept simple: bottom boards along y at three x
    # positions, deck boards along x at `deck_boards` y positions.
    for i, bx in enumerate((-lx / 2 + 0.05, 0.0, lx / 2 - 0.05)):
        px, py = place(bx, 0.0)
        built.obstacles.append(
            scene.add_box(f"{name}/bottom{i}", size=(0.1, wy, board), position=(px, py, z0 + board / 2),
                          quaternion=q, color=color)
        )
        for j, by in enumerate((-wy / 2 + 0.07, 0.0, wy / 2 - 0.07)):
            px, py = place(bx, by)
            built.obstacles.append(
                scene.add_box(f"{name}/block{i}{j}", size=(0.1, 0.14, block),
                              position=(px, py, z0 + board + block / 2), quaternion=q, color=color)
            )
    n = max(1, deck_boards)
    pitch = wy / n
    for k in range(n):
        by = -wy / 2 + pitch * (k + 0.5)
        px, py = place(0.0, by)
        built.obstacles.append(
            scene.add_box(f"{name}/deck{k}", size=(lx, pitch * 0.8, board), position=(px, py, z0 + h - board / 2),
                          quaternion=q, color=color)
        )
    scene.add_frame(f"{name}/top", position=(x, y, z0 + h), quaternion=q)
    built.frames.append(f"{name}/top")
    scene.set_part(name, kind="group", category="pallet", **_identity(model, manufacturer, attributes))
    return built


# ------------------------------------------------------------- light curtain


def light_curtain(
    scene,
    name: str,
    frm: Point2,
    to: Point2,
    *,
    height: float = 1.2,
    beam_height: Optional[float] = None,
    column: float = 0.04,
    watch_robot: bool = True,
    watch: Optional[list[str]] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = FENCE_POST,
    **attributes,
) -> Built:
    """A light curtain between two floor points: a beam sensor `<name>`
    at `beam_height` (half the `height` by default) that trips on any robot
    link (`watch_robot`) and/or the named objects, and two mounting columns
    `<name>/column_a|b` of `height`. The part (`sensor.light_curtain`) is
    pinned on the sensor; the columns are its mounting geometry."""
    (xa, ya), (xb, yb) = (float(frm[0]), float(frm[1])), (float(to[0]), float(to[1]))
    zb = beam_height if beam_height is not None else height / 2
    built = Built(name)
    for tag, (px, py) in (("a", (xa, ya)), ("b", (xb, yb))):
        built.obstacles.append(
            scene.add_box(f"{name}/column_{tag}", size=(column, column, height), position=(px, py, height / 2), color=color)
        )
    scene.add_beam_sensor(name, frm=(xa, ya, zb), to=(xb, yb, zb), watch=watch, watch_robot=watch_robot)
    built.sensors.append(name)
    scene.set_part(name, kind="sensor", category="sensor.light_curtain", **_identity(model, manufacturer, attributes))
    return built


__all__ = ["Built", "conveyor", "fence", "light_curtain", "pallet", "pedestal", "table"]
