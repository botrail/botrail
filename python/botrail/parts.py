"""Standard structures, generated from parameters: fences, walls, tables,
pedestals, racks, conveyor bodies, pallets, light curtains, stairs, control
cabinets, a machining centre with its door, its panel and a vise — the
scenery every cell has and nobody wants to model.

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

Pass `catalog=` instead of a model string and the parameters come from the
catalog: the height is checked against the ones that are sold, the panels are
laid out in widths that exist, and every line of the BOM carries the part
number you would order it by.

    bt.parts.fence(scene, "fence", path=[...], catalog="botrail/fence/mesh-guard",
                   height=2.0, door=(0, 1))

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
from typing import TYPE_CHECKING, Mapping, Optional, Sequence, Union

if TYPE_CHECKING:
    from ._spec import CatalogRef

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]
Color = tuple[float, float, float]

# Muted linear-RGB colours: galvanised fence, dark posts, grey steel, wood.
FENCE_PANEL: Color = (0.55, 0.58, 0.60)
FENCE_POST: Color = (0.16, 0.17, 0.19)
STEEL: Color = (0.42, 0.44, 0.47)
DARK_STEEL: Color = (0.20, 0.21, 0.23)
WOOD: Color = (0.52, 0.36, 0.18)
# 縞鋼板 — the bright non-slip plate a stair tread is made of, and the
# safety colour its handrail is painted (both linear RGB, like the rest).
CHECKER_PLATE: Color = (0.50, 0.52, 0.53)
# The light beige a control cabinet is painted (Munsell 5Y7/1, the colour
# the big Japanese enclosure series ship in).
CABINET: Color = (0.58, 0.56, 0.49)
SAFETY_ORANGE: Color = (0.91, 0.36, 0.02)
BELT: Color = (0.10, 0.10, 0.11)
# The two surfaces a building is made of: painted plasterboard, and the
# fair-faced slab it is built off.
PLASTER: Color = (0.62, 0.60, 0.56)
CONCRETE: Color = (0.40, 0.40, 0.39)


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
    nodes: list[str] = field(default_factory=list)

    def remove(self, scene) -> None:
        """Takes everything this generator added out of the scene (parts go
        with their residents)."""
        for name in self.nodes:
            scene.remove_io_node(name)
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



# ------------------------------------------------------------------- detail

# `detail="full"` draws a machine the way it looks — a mesh panel as a frame
# and a grid of wire, a conveyor with its drive and pulleys, a rack with its
# beams and braces. Everything it adds is **decoration**: drawn, never
# collided. The massing underneath is unchanged, so how a cell verifies never
# depends on how it looks (the pair of switches in the obstacles guide).
DETAIL_MODES = ("plain", "full")

FENCE_FRAME: Color = (0.38, 0.40, 0.42)
MOTOR: Color = (0.24, 0.26, 0.30)


def _detail(mode: Optional[str], has_catalog: bool) -> str:
    """A catalog knows the real sections, so a catalog part is drawn in full
    unless asked otherwise; a hand-written one stays the plain massing."""
    if mode is None:
        return "full" if has_catalog else "plain"
    if mode not in DETAIL_MODES:
        raise ValueError(f"detail must be one of {DETAIL_MODES}, not {mode!r}")
    return mode


def _load_trim(
    scene, built: Built, spec, role: str, prefix: str, position: Point3,
    quaternion=None, **args,
) -> bool:
    """Draw a part from the file the catalog ships for it, expanded to this
    size. Everything it adds is decoration, the same as `_trim`. Returns
    False when the pack names no file, so the caller draws its own."""
    path = None if spec is None else spec.trim(role)
    if path is None:
        return False
    before = set(scene.frames)
    names = scene.load_urdf(
        str(path), prefix=prefix, position=position, quaternion=quaternion,
        args={key: repr(float(value)) for key, value in args.items()}, frames=False,
    )
    for name in names:
        scene.set_obstacle_enabled(name, False)
        built.obstacles.append(name)
    built.frames.extend(sorted(set(scene.frames) - before))
    return True


def _trim(scene, built: Built, name: str, size, position, quaternion=None, color=None) -> str:
    """Add a decorative box: rendered, out of collision."""
    made = scene.add_box(name, size=size, position=position, quaternion=quaternion, color=color)
    scene.set_obstacle_enabled(made, False)
    built.obstacles.append(made)
    return made


def _bars(span: float, pitch_hint: float, most: int = 5) -> list[float]:
    """Offsets, centred on 0, for the bars drawn across `span`. The drawn
    pitch is never finer than the span allows for `most` bars — a 49 mm mesh
    is read as a grid, not modelled wire by wire."""
    pitch = max(pitch_hint, span / (most + 1))
    count = max(0, int(round(span / pitch)) - 1)
    if count <= 0:
        return []
    step = span / (count + 1)
    return [-span / 2 + step * (i + 1) for i in range(count)]


def _axis_quat(nx: float, ny: float) -> tuple[float, float, float, float]:
    """Turns +Z (a cylinder's axis) onto the horizontal direction (nx, ny)."""
    half = math.sqrt(0.5)
    return (-ny * half, nx * half, 0.0, half)


def _pitch_quat(angle: float) -> tuple[float, float, float, float]:
    """A rotation about X — takes +Y (a bar's long axis) up by `angle`."""
    return (math.sin(angle / 2.0), 0.0, 0.0, math.cos(angle / 2.0))


def _slope_quat(angle: float) -> tuple[float, float, float, float]:
    """A rotation about +Y. `-pitch` lays a box's long +X axis up a
    slope; `pi/2 - pitch` stands a cylinder's +Z axis along one.
    (`_pitch_quat` turns about X, which is what a +Y-long bar needs.)"""
    return (0.0, math.sin(angle / 2.0), 0.0, math.cos(angle / 2.0))


def _mul_quat(a, b):
    """a then b, both (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _trim_cylinder(scene, built: Built, name: str, radius, length, position, quaternion, color) -> str:
    made = scene.add_cylinder(name, radius=radius, length=length, position=position,
                              quaternion=quaternion, color=color)
    scene.set_obstacle_enabled(made, False)
    built.obstacles.append(made)
    return made


def _aperture(mesh: object, default: float = 0.049) -> float:
    """The wire spacing a mesh code names ("49x49" -> 49 mm), in metres."""
    if isinstance(mesh, (int, float)):
        return float(mesh) / 1000.0
    if isinstance(mesh, str):
        head = mesh.split("x")[0].strip()
        try:
            return float(head) / 1000.0
        except ValueError:
            return default
    return default


def _mesh_panel(
    scene, built: Built, prefix: str, centre: Point3, size: Point3, yaw: float, *,
    frame: float, wire: float, aperture: float,
    frame_color: Color = FENCE_FRAME, wire_color: Color = FENCE_PANEL,
) -> None:
    """A mesh panel as it looks: a tube frame with a grid of wire in it. The
    panel's own slab collides; none of this does."""
    cx, cy, cz = centre
    width, thickness, height = size
    c, sn = math.cos(yaw), math.sin(yaw)
    q = _yaw_quat(yaw)

    def at(tag: str, along: float, up: float, box: Point3, color: Color) -> None:
        _trim(scene, built, f"{prefix}/{tag}", box,
              (cx + c * along, cy + sn * along, cz + up), q, color)

    inner_w, inner_h = width - 2 * frame, height - 2 * frame
    at("frame_t", 0.0, height / 2 - frame / 2, (width, thickness, frame), frame_color)
    at("frame_b", 0.0, -height / 2 + frame / 2, (width, thickness, frame), frame_color)
    at("frame_l", -width / 2 + frame / 2, 0.0, (frame, thickness, inner_h), frame_color)
    at("frame_r", width / 2 - frame / 2, 0.0, (frame, thickness, inner_h), frame_color)
    for i, along in enumerate(_bars(inner_w, aperture)):
        at(f"wire_v{i}", along, 0.0, (wire, wire, inner_h), wire_color)
    for i, up in enumerate(_bars(inner_h, aperture)):
        at(f"wire_h{i}", 0.0, up, (inner_w, wire, wire), wire_color)


# ------------------------------------------------------------------ packing


def _reachable(
    limit: int, options: Sequence[int], post_mm: float
) -> tuple[list[int], list[Optional[int]], int]:
    """`best[s]` = fewest panels that fill exactly s mm, `used[s]` = the last
    one placed. Options are walked widest first and a tie never overwrites,
    so reconstruction is stable."""
    unreachable = limit + 1
    best = [unreachable] * (limit + 1)
    best[0] = 0
    used: list[Optional[int]] = [None] * (limit + 1)
    for filled in range(limit + 1):
        if best[filled] == unreachable:
            continue
        for width in options:
            reach = filled + int(round(width + post_mm))
            if reach <= limit and best[reach] > best[filled] + 1:
                best[reach] = best[filled] + 1
                used[reach] = width
    return best, used, unreachable


def buildable_lengths(length_mm: float, widths_mm: Sequence[int], post_mm: float) -> list[int]:
    """The edge lengths nearest `length_mm` that these panels can actually
    make — what to move a corner to when a run does not come out."""
    options = sorted({int(w) for w in widths_mm}, reverse=True)
    if not options:
        return []
    target = int(round(length_mm))
    limit = target + int(round(options[0] + post_mm))
    best, _, unreachable = _reachable(limit, options, post_mm)
    below = next((s for s in range(min(target, limit), 0, -1) if best[s] != unreachable), None)
    above = next((s for s in range(target, limit + 1) if best[s] != unreachable), None)
    return sorted({s for s in (below, above) if s})


def _pack_edge(
    length_mm: float, widths_mm: Sequence[int], post_mm: float, tolerance_mm: float,
    reserve_mm: float = 0.0,
) -> Optional[list[int]]:
    """Fill one edge with panel widths that exist.

    An edge carries a post at each corner and one between every pair of
    panels, and the corner posts are centred on the corner, so the edge is
    exactly `sum(width + post)` long. Returns the widths (widest first) that
    reach within `tolerance_mm` of the edge, or None when the edge cannot be
    built from these widths. `reserve_mm` takes a slot out first — that is
    where a door goes.

    Deterministic, and picked the way a bill is read rather than the way a
    solver finds it first: **fewest panels, then fewest different widths**
    (each one is another line to order and another part to store), then the
    widest bay. So a 2.4 m run comes back as 1000 + 1000 + 200, not
    1500 + 400 + 300.
    """
    target = int(round(length_mm - reserve_mm - (post_mm if reserve_mm else 0.0)))
    if target < 0:
        return None
    slack = int(round(tolerance_mm))
    options = sorted({int(w) for w in widths_mm}, reverse=True)
    # best[s] = fewest panels that fill exactly s mm, and the width that got
    # there. Options are walked widest first and a tie never overwrites, so
    # the reconstruction is stable.
    best, used, unreachable = _reachable(target, options, post_mm)

    def tail(room: int) -> Optional[list[int]]:
        """The fewest panels that come within `slack` of `room`."""
        if room < 0:
            return None
        reachable = [s for s in range(max(0, room - slack), room + 1) if best[s] != unreachable]
        if not reachable:
            return None
        # Fewest panels, and among those the longest run (smallest gap left).
        cursor = min(reachable, key=lambda s: (best[s], -s))
        plan: list[int] = []
        while cursor > 0:
            width = used[cursor]
            assert width is not None
            plan.append(width)
            cursor -= int(round(width + post_mm))
        return plan

    chosen: Optional[list[int]] = None
    key: Optional[tuple[int, int, int]] = None
    for primary in options:
        step = int(round(primary + post_mm))
        for repeats in range(target // step, -1, -1):
            rest = tail(target - repeats * step)
            if rest is None:
                continue
            plan = [primary] * repeats + rest
            if not plan:
                continue
            candidate = (len(plan), len(set(plan)), -primary)
            if key is None or candidate < key:
                key, chosen = candidate, plan
            break  # more of the primary bay never means fewer panels
    return sorted(chosen, reverse=True) if chosen else None


def _plan_edge(
    length_mm: float, widths_mm: Sequence[int], post_mm: float, tolerance_mm: float,
    door_widths_mm: Optional[Sequence[int]] = None,
) -> Optional[tuple[list[int], Optional[int]]]:
    """`_pack_edge`, with a door taking one of the slots when asked for.

    The door is as wide as the edge affords: widths are tried widest first
    and the one needing the fewest panels around it wins.
    """
    if not door_widths_mm:
        plan = _pack_edge(length_mm, widths_mm, post_mm, tolerance_mm)
        return (plan, None) if plan is not None else None
    best: Optional[tuple[list[int], Optional[int]]] = None
    for door_width in sorted({int(w) for w in door_widths_mm}, reverse=True):
        plan = _pack_edge(length_mm, widths_mm, post_mm, tolerance_mm, reserve_mm=door_width)
        if plan is not None and (best is None or len(plan) < len(best[0])):
            best = (plan, door_width)
    return best


# --------------------------------------------------------------------- fence


def fence(
    scene,
    name: str,
    path: Sequence[Point2],
    *,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    height: Optional[float] = None,
    panel_pitch: Optional[float] = None,
    post: Optional[float] = None,
    panel_thickness: Optional[float] = None,
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
    """A safety fence along `path` (floor corners, metres), with a post at
    every corner and between panels. `closed` joins the last corner back to
    the first. `door=(edge, slot)` makes that slot the door — its own
    obstacle `<name>/door` and its own BOM line.

    Without a catalog each edge is split into panels of about `panel_pitch`
    (the pitch is stretched so an edge takes a whole number), and two parts
    are pinned: `<name>` (the panels, `qty` = panels, with the `model` /
    `manufacturer` / `mass_kg` you passed) and `<name>/posts`.

    With `catalog=` — the id of a fence spec pack, or a package directory —
    the fence is built out of panels that exist. `height` is checked against
    the heights that are sold, the widths come from the catalog and each edge
    is filled with the fewest of them that reach its length, and any catalog
    parameter can be set by name (`mesh_mm="20x20"`). The parts pinned are
    then `<name>` (the fence as one product, so the layout sheet still labels
    it once), one group **per panel width** carrying that width's part number
    and count, `<name>/posts` and `<name>/door` — a bill you can order from.

    `detail="full"` (the default with a catalog) draws each panel the way it
    looks — a tube frame with a grid of wire in it, posts of the section the
    catalog sells, a plate under each — as decoration that never collides;
    the panel slab underneath still does, so nothing about the verification
    changes. `detail="plain"` is the bare massing. Returns the names it
    made."""
    pts = [tuple(map(float, p)) for p in path]
    if len(pts) < 2:
        raise ValueError("fence: path needs at least two corners")

    spec = None
    params: dict = {}
    widths_mm: list[int] = []
    door_widths_mm: Optional[list[int]] = None
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("fence")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if height is not None and "height_mm" in spec.params():
            params["height_mm"] = spec.choose("height_mm", round(height * 1000.0, 3))
        height = _sized(params, "height_mm", height)
        post = post if post is not None else _mm(spec.dimension_mm("post", "section_w", 60.0))
        panel_thickness = (
            panel_thickness
            if panel_thickness is not None
            else _mm(spec.dimension_mm("panel", "thickness", 30.0))
        )
        model = model or spec.name
        manufacturer = manufacturer or spec.manufacturer
        widths_mm = _usable_widths(spec, post, panel_pitch)
        if spec.has_component("door"):
            door_widths_mm = spec.widths_mm("door")
        post_depth = _mm(spec.dimension_mm("post", "section_d", None))
        frame_mm = spec.dimension_mm("panel", "frame", 30.0)
        wire_mm = spec.dimension_mm("panel", "wire", 5.0)
        aperture = _aperture(params.get("mesh_mm"))

    mode = _detail(detail, spec is not None)
    height = 2.0 if height is None else height
    post = 0.06 if post is None else post
    panel_thickness = 0.03 if panel_thickness is None else panel_thickness
    if spec is None:
        post_depth, frame_mm, wire_mm, aperture = None, 30.0, 5.0, 0.049
    if height <= 0 or post <= 0:
        raise ValueError("fence: height and post must be positive")
    if spec is None and (panel_pitch is not None and panel_pitch <= 0):
        raise ValueError("fence: panel_pitch must be positive")
    pitch_hint = 1.0 if panel_pitch is None else panel_pitch

    edges = list(zip(pts, pts[1:] + ([pts[0]] if closed and len(pts) > 2 else [])))
    built = Built(name)
    per_width: dict[int, int] = {}
    panels = 0
    posts = 0
    door_width_mm: Optional[int] = None
    # Posts at every distinct corner (a closed loop shares its ends).
    for i, (x, y) in enumerate(pts):
        pname = scene.add_box(
            f"{name}/posts/c{i}", size=(post, post, height), position=(x, y, height / 2), color=post_color
        )
        built.obstacles.append(pname)
        posts += 1
        if mode == "full":
            drawn = _load_trim(
                scene, built, spec, "post", f"{name}/trim/c{i}", (x, y, 0.0), None,
                height=height, section_w=post, section_d=post_depth or post, square=1.0,
            )
            if drawn:
                scene.set_obstacle_visible(pname, False)
            else:
                _trim(scene, built, f"{name}/trim/base_c{i}", (post * 2.4, post * 2.4, post / 4),
                      (x, y, post / 8), None, post_color)
    for e, ((x0, y0), (x1, y1)) in enumerate(edges):
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-9:
            continue
        door_slot = door[1] if door is not None and door[0] == e else None
        bays, joints = _slots(spec, length, widths_mm, post, pitch_hint, door_widths_mm, door_slot)
        yaw = math.atan2(y1 - y0, x1 - x0)
        ux, uy = (x1 - x0) / length, (y1 - y0) / length
        for i, (along, width, is_door) in enumerate(bays):
            if is_door:
                oname = f"{name}/door"
            elif spec is None:
                oname = f"{name}/panels/e{e}_{i}"
            else:
                # A group per width so the BOM carries one line per part number.
                oname = f"{name}/panels/w{round(width * 1000)}/e{e}_{i}"
            oname = scene.add_box(
                oname,
                size=(width, panel_thickness, height),
                position=(x0 + ux * along, y0 + uy * along, height / 2),
                quaternion=_yaw_quat(yaw),
                color=panel_color,
            )
            built.obstacles.append(oname)
            if mode == "full":
                # The slab keeps collision; what you see is the frame and the
                # wire in it — from the pack's own file when it ships one.
                scene.set_obstacle_visible(oname, False)
                prefix = f"{name}/trim/e{e}_{i}"
                px, py = x0 + ux * along, y0 + uy * along
                drawn = _load_trim(
                    scene, built, spec, "door" if is_door else "panel", prefix,
                    (px, py, 0.0), _yaw_quat(yaw),
                    width=width, height=height, thickness=panel_thickness,
                    frame=frame_mm / 1000.0, wire=wire_mm / 1000.0, aperture=aperture,
                    handle=1.0 if is_door else 0.0,
                )
                if not drawn:
                    _mesh_panel(
                        scene, built, prefix, (px, py, height / 2),
                        (width, panel_thickness, height), yaw,
                        frame=frame_mm / 1000.0, wire=wire_mm / 1000.0, aperture=aperture,
                        wire_color=panel_color,
                    )
                    if is_door:  # a handle where it opens
                        grip = width / 2 - frame_mm / 1000.0 * 2
                        _trim(scene, built, f"{prefix}/handle",
                              (frame_mm / 1000.0 * 3, panel_thickness * 2, wire_mm / 500.0),
                              (x0 + ux * (along + grip), y0 + uy * (along + grip), height / 2),
                              _yaw_quat(yaw), FENCE_FRAME)
            if is_door:
                door_width_mm = round(width * 1000)
                if spec is None:  # pinned here so the BOM keeps its order
                    scene.set_part(oname, category="structure.door", model=door_model, qty=1)
            else:
                panels += 1
                per_width[round(width * 1000)] = per_width.get(round(width * 1000), 0) + 1
            # An intermediate post between bays (not at the edge ends — those
            # are corners).
            if i < len(joints):
                px, py = x0 + ux * joints[i], y0 + uy * joints[i]
                # A post between panels stands along the run, so it can be the
                # section the catalog actually sells (60 x 40, not 60 square).
                section = (post, post_depth or post, height)
                pname = scene.add_box(
                    f"{name}/posts/e{e}_{i}", size=section,
                    position=(px, py, height / 2),
                    quaternion=_yaw_quat(yaw) if post_depth else None,
                    color=post_color,
                )
                built.obstacles.append(pname)
                posts += 1
                if mode == "full":
                    drawn = _load_trim(
                        scene, built, spec, "post", f"{name}/trim/p{e}_{i}", (px, py, 0.0),
                        _yaw_quat(yaw), height=height, section_w=post,
                        section_d=post_depth or post, square=0.0,
                    )
                    if drawn:
                        scene.set_obstacle_visible(pname, False)
                    else:
                        _trim(scene, built, f"{name}/trim/base_e{e}_{i}",
                              (post * 2.4, post * 2.4, post / 4), (px, py, post / 8), None,
                              post_color)

    if spec is None:
        scene.set_part(
            name, kind="group", category="structure.fence", qty=max(panels, 1),
            **_identity(model, manufacturer, attributes),
        )
        scene.set_part(f"{name}/posts", kind="group", category="structure.fence.post", qty=posts,
                       model=post_model)
        return built

    # The fence as one product: what the layout sheet labels, and what the
    # BOM carries the configuration on. No mass — the panels have it, and it
    # must not be counted twice. The parameters go on as text: they are a
    # record of what was ordered, and a column of heights has no sum.
    recorded = {key: str(_plain(value)) for key, value in params.items()}
    scene.set_part(
        name, kind="group", category="structure.fence", qty=1, catalog=spec.catalog_ref,
        **_identity(model, manufacturer, {**recorded, **attributes}),
    )
    for width_mm, count in sorted(per_width.items(), reverse=True):
        scene.set_part(
            f"{name}/panels/w{width_mm}", kind="group", category="structure.fence", qty=count,
            catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=spec.part_number("panel", **params, width_mm=width_mm),
            **_kg(spec.mass_kg("panel", **params, width_mm=width_mm)),
        )
    scene.set_part(
        f"{name}/posts", kind="group", category="structure.fence.post", qty=posts,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=post_model or spec.part_number("post", **params),
        **_kg(spec.mass_kg("post", **params)),
    )
    if door_width_mm is not None:
        scene.set_part(
            f"{name}/door", category="structure.door", qty=1, catalog=spec.catalog_ref,
            manufacturer=manufacturer,
            model=door_model or spec.part_number("door", **params, width_mm=door_width_mm),
            **_kg(spec.mass_kg("door", **params, width_mm=door_width_mm)),
        )
    return built


def _mm(value: Optional[float]) -> Optional[float]:
    return None if value is None else float(value) / 1000.0


def _sized(params: dict, key: str, given: Optional[float]) -> Optional[float]:
    """A dimension in metres: what the catalog resolved, else what the caller
    passed. A pack that does not sell a size by that name simply leaves it to
    the caller."""
    value = params.get(key)
    return given if value is None else float(value) / 1000.0


def _sized_box(spec, params: dict, given, keys) -> tuple[float, float, float]:
    """The three sides in metres, from the catalog where it sells them and
    from the caller where it does not. Asking for a size the pack cannot
    resolve either way says which axis is missing."""
    if given is not None:
        for key, value in zip(keys, given):
            if key in spec.params():
                params[key] = spec.choose(key, round(float(value) * 1000.0, 3))
    sides = [_sized(params, key, side) for key, side in zip(keys, given or (None, None, None))]
    missing = [key[: -len("_mm")] for key, side in zip(keys, sides) if side is None]
    if missing:
        raise ValueError(f"{spec.id}: this pack does not size the {', '.join(missing)} — pass size=")
    return (sides[0], sides[1], sides[2])


def _plain(value):
    """2000.0 reads as 2000 — these end up in part numbers and on the BOM."""
    if isinstance(value, float) and abs(value - round(value)) < 1e-6:
        return int(round(value))
    return value


def _kg(mass: Optional[float]) -> dict:
    return {"mass_kg": mass} if mass is not None else {}


def _usable_widths(spec, post: float, panel_pitch: Optional[float]) -> list[int]:
    """The catalog's panel widths, minus the ones this fence cannot use."""
    widths = spec.widths_mm("panel")
    if not widths:
        raise ValueError(f"{spec.id}: the panel component declares no widths_mm")
    pitch_max = spec.rule("post_pitch_max_mm")
    usable = list(widths)
    if pitch_max is not None:
        usable = [w for w in usable if w + post * 1000.0 <= float(pitch_max) + 1e-6]
    if panel_pitch is not None:
        usable = [w for w in usable if w <= panel_pitch * 1000.0 + 1e-6]
    if not usable:
        raise ValueError(
            f"{spec.id}: no panel width fits (widths {widths}, post pitch max {pitch_max}"
            + (f", panel_pitch {panel_pitch * 1000:.0f} mm" if panel_pitch else "")
            + ")"
        )
    return usable


def _slots(
    spec, length: float, widths_mm: Sequence[int], post: float, pitch_hint: float,
    door_widths_mm: Optional[Sequence[int]], door_slot: Optional[int],
) -> tuple[list[tuple[float, float, bool]], list[float]]:
    """One edge as (bays, joints), measured from its start in metres: a bay is
    (centre, width, is_door), a joint is where a post goes between two of them.

    The corner posts are centred on the corners, so the run starts half a post
    in and every joint costs a whole post: `length = sum(width) + bays * post`.
    A uniform pitch is taken straight from the index rather than accumulated,
    so an edge without a catalog lays out to the last bit as it always has.
    """
    if spec is None:
        count = max(1, round(length / pitch_hint))
        pitch = length / count
        bays = [(pitch * (i + 0.5), pitch - post, door_slot == i) for i in range(count)]
        return bays, [pitch * (i + 1) for i in range(count - 1)]
    tolerance = float(spec.rule("tolerance_mm", 25.0))
    plan = _plan_edge(
        length * 1000.0, widths_mm, post * 1000.0, tolerance,
        door_widths_mm if door_slot is not None else None,
    )
    if plan is None:
        near = buildable_lengths(length * 1000.0, widths_mm, post * 1000.0)
        hint = " / ".join(f"{n} mm" for n in near) or "none nearby"
        raise ValueError(
            f"{spec.id}: an edge of {length * 1000:.0f} mm cannot be built from panels of "
            f"{'/'.join(str(w) for w in widths_mm)} mm with {post * 1000:.0f} mm posts "
            f"(within {tolerance:.0f} mm). Nearest buildable: {hint} — move the corner, "
            f"or raise rules.tolerance_mm in the catalog"
        )
    panel_widths, door_width = plan
    widths = [(width / 1000.0, False) for width in panel_widths]
    if door_width is not None:
        widths.insert(min(door_slot or 0, len(widths)), (door_width / 1000.0, True))
    bays: list[tuple[float, float, bool]] = []
    joints: list[float] = []
    cursor = post / 2.0
    for i, (width, is_door) in enumerate(widths):
        bays.append((cursor + width / 2.0, width, is_door))
        cursor += width
        if i < len(widths) - 1:
            joints.append(cursor + post / 2.0)
            cursor += post
    return bays, joints


# --------------------------------------------------------------------- table


def table(
    scene,
    name: str,
    size: Optional[Point3] = None,
    position: Point2 | Point3 = (0.0, 0.0),
    *,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    top_thickness: Optional[float] = None,
    leg: Optional[float] = None,
    yaw: float = 0.0,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    top_model: Optional[str] = None,
    color: Color = STEEL,
    **attributes,
) -> Built:
    """A table `size = (length, width, height)` standing on the floor at
    `position` (its centre, x, y[, floor z]): a top of `top_thickness` on
    four legs. Adds the frame `<name>/top` at the centre of the top face —
    where a fixture or a workpiece sits — and pins one part
    (`structure.table`) on the group.

    With `catalog=` — the id of a table spec pack, or a package directory — a
    stand you can order: the sides are matched against the ones that are sold
    (omit `size` for the pack's defaults, so `position` alone will do), the
    profile section and the board thickness come from the pack, and where the
    maker sells the board separately it lands on the BOM as its own line.

    `detail="full"` (the default with a catalog) adds the rails under the
    board and a pad under each foot — decoration that never collides, so the
    legs and the board stay the only thing a robot can hit."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("table")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        size = _sized_box(spec, params, size, ("width_mm", "depth_mm", "height_mm"))
        top_thickness = (
            top_thickness
            if top_thickness is not None
            else _mm(spec.dimension_mm("top", "thickness", 30.0))
        )
        leg = leg if leg is not None else _mm(spec.dimension_mm("frame", "leg", 40.0))
        manufacturer = manufacturer or spec.manufacturer

    mode = _detail(detail, spec is not None)
    if size is None:
        raise ValueError("table: size is required without a catalog")
    top_thickness = 0.03 if top_thickness is None else top_thickness
    leg = 0.04 if leg is None else leg
    lx, wy, h = (float(v) for v in size)
    if min(lx, wy, h) <= 0:
        raise ValueError("table: size must be positive")

    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)

    def world(dx: float, dy: float) -> tuple[float, float]:
        return x + c * dx - s * dy, y + s * dx + c * dy

    built = Built(name)
    built.obstacles.append(
        scene.add_box(f"{name}/top", size=(lx, wy, top_thickness), position=(x, y, z0 + h - top_thickness / 2),
                      quaternion=q, color=color)
    )
    corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    for i, (sx, sy) in enumerate(corners):
        px, py = world(sx * (lx / 2 - leg / 2), sy * (wy / 2 - leg / 2))
        built.obstacles.append(
            scene.add_box(f"{name}/leg{i}", size=(leg, leg, h - top_thickness),
                          position=(px, py, z0 + (h - top_thickness) / 2), quaternion=q, color=color)
        )
    scene.add_frame(f"{name}/top", position=(x, y, z0 + h), quaternion=q)
    built.frames.append(f"{name}/top")

    if mode == "full":
        drawn = _load_trim(
            scene, built, spec, "frame", f"{name}/trim/frame", (x, y, z0), q,
            width=lx, depth=wy, height=h, leg=leg, top_thickness=top_thickness,
        )
        if drawn:
            for i in range(len(corners)):
                scene.set_obstacle_visible(f"{name}/leg{i}", False)
        else:
            # An aluminium stand is legs plus the rails that tie them together
            # under the board, and a pad under each foot.
            rail = leg * 0.8
            under = z0 + h - top_thickness - rail / 2
            for i, (ex, ey) in enumerate([(0, -1), (0, 1), (-1, 0), (1, 0)]):
                span = (rail * 0.6, wy - 2 * leg, rail) if ex else (lx - 2 * leg, rail * 0.6, rail)
                px, py = world(ex * (lx / 2 - leg / 2), ey * (wy / 2 - leg / 2))
                _trim(scene, built, f"{name}/trim/rail{i}", span, (px, py, under), q, DARK_STEEL)
            for i, (sx, sy) in enumerate(corners):
                px, py = world(sx * (lx / 2 - leg / 2), sy * (wy / 2 - leg / 2))
                _trim(scene, built, f"{name}/trim/foot{i}", (leg * 1.8, leg * 1.8, leg / 4),
                      (px, py, z0 + leg / 8), q, DARK_STEEL)
        if _load_trim(
            scene, built, spec, "top", f"{name}/trim/top", (x, y, z0 + h), q,
            width=lx, depth=wy, thickness=top_thickness,
        ):
            scene.set_obstacle_visible(f"{name}/top", False)

    if spec is None:
        scene.set_part(name, kind="group", category="structure.table", **_identity(model, manufacturer, attributes))
        return built

    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    scene.set_part(
        name, kind="group", category=spec.category("frame", "structure.table"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("frame", **params), description=spec.name,
        **{**recorded, **_kg(spec.mass_kg("frame", **params)), **attributes},
    )
    if spec.has_component("top"):
        # The board is its own article where the maker sells it that way.
        scene.set_part(
            f"{name}/top", category=spec.category("top", "structure.table"), qty=1,
            catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=top_model or spec.part_number("top", **params),
            **_kg(spec.mass_kg("top", **params)),
        )
    return built


# ------------------------------------------------------------------ pedestal


def pedestal(
    scene,
    name: str,
    height: Optional[float] = None,
    position: Point2 | Point3 = (0.0, 0.0),
    *,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    top: Optional[Point2] = None,
    base: Optional[Point2] = None,
    column: Optional[float] = None,
    plate: Optional[float] = None,
    yaw: float = 0.0,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = DARK_STEEL,
    **attributes,
) -> Built:
    """A robot pedestal: base plate, column, top plate, `height` from floor
    to the top face at `position`. Adds the frame `<name>/mount` at the top
    centre — the robot's base pose (`scene.set_robot_base_pose(*scene.frame(
    "<name>/mount"))`) — and pins one part (`structure.pedestal`).

    With `catalog=` — the id of a pedestal spec pack, or a package directory —
    a stand you can order: the height is matched against the ones that are
    sold (omit it for the pack's default) and the column, plates and their
    footprints come from the pack, so the BOM names the stand a robot is
    actually bolted to.

    `detail="full"` (the default with a catalog) adds the gussets between the
    column and the base — decoration that never collides."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("pedestal")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if height is not None and "height_mm" in spec.params():
            params["height_mm"] = spec.choose("height_mm", round(height * 1000.0, 3))
        height = _sized(params, "height_mm", height)
        if height is None:
            raise ValueError(f"{spec.id}: this pack does not size the height — pass height=")
        base = base if base is not None else (
            _mm(spec.dimension_mm("pedestal", "base_w", 500.0)),
            _mm(spec.dimension_mm("pedestal", "base_d", 500.0)),
        )
        top = top if top is not None else (
            _mm(spec.dimension_mm("pedestal", "top_w", 350.0)),
            _mm(spec.dimension_mm("pedestal", "top_d", 350.0)),
        )
        column = column if column is not None else _mm(spec.dimension_mm("pedestal", "column", 200.0))
        plate = plate if plate is not None else _mm(spec.dimension_mm("pedestal", "plate", 20.0))
        manufacturer = manufacturer or spec.manufacturer

    mode = _detail(detail, spec is not None)
    if height is None:
        raise ValueError("pedestal: height is required without a catalog")
    top = (0.35, 0.35) if top is None else top
    base = (0.5, 0.5) if base is None else base
    column = 0.2 if column is None else column
    plate = 0.02 if plate is None else plate
    if height <= 0 or column <= 0 or plate <= 0:
        raise ValueError("pedestal: height, column and plate must be positive")

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

    if mode == "full":
        drawn = _load_trim(
            scene, built, spec, "pedestal", f"{name}/trim/stand", (x, y, z0), q,
            height=height, column=column, plate=plate,
            base_w=base[0], base_d=base[1], top_w=top[0], top_d=top[1],
        )
        if drawn:
            for part in ("base", "column", "top"):
                scene.set_obstacle_visible(f"{name}/{part}", False)
        else:
            # The gussets that take the moment out of a robot into the floor.
            c, s_ = math.cos(yaw), math.sin(yaw)
            reach = max(min(base[0], base[1]) / 2 - column / 2, 0.0) * 0.7
            rise = min(shaft * 0.35, max(reach * 1.4, plate * 3))
            web = max(plate / 2, 0.004)
            if reach > web:
                for i, (ux, uy) in enumerate([(1, 0), (-1, 0), (0, 1), (0, -1)]):
                    dx, dy = ux * (column / 2 + reach / 2), uy * (column / 2 + reach / 2)
                    px, py = x + c * dx - s_ * dy, y + s_ * dx + c * dy
                    _trim(scene, built, f"{name}/trim/gusset{i}",
                          (reach if ux else web, reach if uy else web, rise),
                          (px, py, z0 + plate + rise / 2), q, color)

    if spec is None:
        scene.set_part(name, kind="group", category="structure.pedestal", **_identity(model, manufacturer, attributes))
        return built

    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    scene.set_part(
        name, kind="group", category=spec.category("pedestal", "structure.pedestal"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("pedestal", **params), description=spec.name,
        **{**recorded, **_kg(spec.mass_kg("pedestal", **params)), **attributes},
    )
    return built


# ------------------------------------------------------------------- cabinet


def cabinet(
    scene,
    name: str,
    size: Optional[Point3] = None,
    position: Point2 | Point3 = (0.0, 0.0),
    *,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    base: Optional[bool] = None,
    plate: Optional[bool] = None,
    base_height: Optional[float] = None,
    yaw: float = 0.0,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = CABINET,
    **attributes,
) -> Built:
    """A control cabinet: `size = (width, depth, height)` standing at
    `position` (its centre, x, y[, floor z]), door face on -Y before `yaw`.
    Adds the frame `<name>/front` at the centre of the door face at floor
    level — where an operator stands, and what a maintenance-space check
    will measure from — and pins the enclosure (`structure.cabinet`).

    The panel builder's customisation is what this generator carries: the
    *enclosure* is the article (what is inside it is other people's BOM
    lines), and the plinth base and the mounting plate are articles of their
    own. `base=` stands the body on its plinth (`<name>/base`), `plate=`
    stands the mounting plate inside (`<name>/plate`) — each is one more
    line on the BOM when a catalog names it.

    With `catalog=` — the id of a cabinet spec pack, or a package directory —
    an enclosure you can order: width, depth and height are matched against
    the sizes that are sold, the BOM row carries the article number they
    compose into, and base and plate default to whatever the pack sells
    (pass `base=False` / `plate=False` to leave them out). A combination
    nobody sells is refused by the pack's mass table.

    `detail="full"` (the default with a catalog) draws the door leaves and
    their handles — or the pack's own drawing (`trim:`) — as decoration that
    never collides. The massing stays the body (and its plinth)."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("cabinet")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        size = _sized_box(spec, params, size, ("width_mm", "depth_mm", "height_mm"))
        if base is None:
            base = spec.has_component("base")
        if plate is None:
            plate = spec.has_component("plate")
        if "base_height_mm" in spec.params():
            # The plinth height is an ordering axis of its own (a base is
            # sold 50 or 100 mm tall) — validated like any other dimension.
            if base_height is not None:
                params["base_height_mm"] = spec.choose(
                    "base_height_mm", round(base_height * 1000.0, 3))
            base_height = _sized(params, "base_height_mm", base_height)
        elif base_height is None and spec.has_component("base"):
            base_height = _mm(spec.dimension_mm("base", "height", 100.0))
        manufacturer = manufacturer or spec.manufacturer

    mode = _detail(detail, spec is not None)
    if size is None:
        raise ValueError("cabinet: size is required without a catalog")
    base = False if base is None else base
    plate = False if plate is None else plate
    base_height = 0.1 if base_height is None else base_height
    w, d, h = (float(v) for v in size)
    if min(w, d, h) <= 0:
        raise ValueError("cabinet: size must be positive")
    if base and base_height <= 0:
        raise ValueError("cabinet: base_height must be positive")

    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    built = Built(name)

    def world(dx: float, dy: float) -> tuple[float, float]:
        return x + c * dx - s * dy, y + s * dx + c * dy

    plinth = base_height if base else 0.0
    if base:
        built.obstacles.append(
            scene.add_box(f"{name}/base", size=(w, d, plinth),
                          position=(x, y, z0 + plinth / 2), quaternion=q, color=DARK_STEEL)
        )
    built.obstacles.append(
        scene.add_box(f"{name}/body", size=(w, d, h),
                      position=(x, y, z0 + plinth + h / 2), quaternion=q, color=color)
    )
    if plate:
        # The mounting plate stands against the back wall, inside the
        # enclosure — massing like the body (it is steel you ordered), and
        # enclosed by it, so it never collides with anything the body does
        # not.
        thick = 0.0023 if spec is None else (_mm(spec.dimension_mm("plate", "thickness", 2.3)) or 0.0023)
        pw = None if spec is None else _mm(spec.dimension_mm("plate", "width"))
        ph = None if spec is None else _mm(spec.dimension_mm("plate", "height"))
        pw = pw if pw is not None else max(w - 0.15, w * 0.5)
        ph = ph if ph is not None else max(h - 0.25, h * 0.5)
        px, py = world(0.0, d / 2 - 0.05)
        built.obstacles.append(
            scene.add_box(f"{name}/plate", size=(pw, thick, ph),
                          position=(px, py, z0 + plinth + h / 2), quaternion=q, color=STEEL)
        )

    fx, fy = world(0.0, -d / 2)
    scene.add_frame(f"{name}/front", position=(fx, fy, z0), quaternion=q)
    built.frames.append(f"{name}/front")

    if mode == "full":
        drawn = _load_trim(
            scene, built, spec, "body", f"{name}/trim/shell", (x, y, z0), q,
            width=w, depth=d, height=h, base=plinth,
        )
        if drawn:
            for part in ["body"] + (["base"] if base else []) + (["plate"] if plate else []):
                scene.set_obstacle_visible(f"{name}/{part}", False)
        else:
            # Door leaves a shade proud of the face, and a flat handle on
            # each — one door on a narrow body, a pair from a metre up (the
            # generic look; a pack's own drawing knows its real split).
            doors = 2 if w >= 1.0 else 1
            leaf_t, gap = 0.004, 0.01
            leaf_w = (w - gap * (doors + 1)) / doors
            zc = z0 + plinth + h / 2
            for i in range(doors):
                off = -w / 2 + gap + leaf_w / 2 + i * (leaf_w + gap)
                px, py = world(off, -(d / 2 + leaf_t / 2))
                _trim(scene, built, f"{name}/trim/door{i}", (leaf_w, leaf_t, h - 2 * gap),
                      (px, py, zc), q, color)
                # The handle sits by the leaf's swinging edge: the meeting
                # edge of a pair, the lock side of a single door.
                edge = off + (leaf_w / 2 - 0.05) * (1 if doors == 1 or i == 0 else -1)
                hx, hy = world(edge, -(d / 2 + leaf_t + 0.008))
                _trim(scene, built, f"{name}/trim/handle{i}", (0.025, 0.016, 0.14),
                      (hx, hy, zc), q, DARK_STEEL)

    if spec is None:
        scene.set_part(name, kind="group", category="structure.cabinet",
                       **_identity(model, manufacturer, attributes))
        return built

    if not base:
        # No plinth ordered — its axis has nothing to say on the BOM.
        params.pop("base_height_mm", None)
    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    scene.set_part(
        name, kind="group", category=spec.category("body", "structure.cabinet"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("body", **params), description=spec.name,
        **{**recorded, **_kg(spec.mass_kg("body", **params)), **attributes},
    )
    if base and spec.has_component("base"):
        scene.set_part(
            f"{name}/base", kind="obstacle",
            category=spec.category("base", "structure.cabinet.base"), qty=1,
            catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=spec.part_number("base", **params),
            **_kg(spec.mass_kg("base", **params)),
        )
    if plate and spec.has_component("plate"):
        scene.set_part(
            f"{name}/plate", kind="obstacle",
            category=spec.category("plate", "structure.cabinet.plate"), qty=1,
            catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=spec.part_number("plate", **params),
            **_kg(spec.mass_kg("plate", **params)),
        )
    return built


# ---------------------------------------------------------------------- rack


def rack(
    scene,
    name: str,
    size: Optional[Point3] = None,
    position: Point2 | Point3 = (0.0, 0.0),
    *,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    levels: Optional[int] = None,
    upright: Optional[float] = None,
    shelf_thickness: Optional[float] = None,
    yaw: float = 0.0,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    shelf_model: Optional[str] = None,
    color: Color = STEEL,
    **attributes,
) -> Built:
    """Shelving: `size = (width, depth, height)` standing on the floor at
    `position` (its centre, x, y[, floor z]), with `levels` shelves evenly
    spaced and the top one at `height`, on four corner uprights.

    Adds a frame at the centre of every shelf's top face — `<name>/level0` at
    the bottom, upwards — which is where the parts on that shelf sit and what
    a pick targets. Pins the bay (`structure.rack`) on the group.

    With `catalog=` — the id of a rack spec pack, or a package directory — the
    bay is one you can order: the width, depth, height and number of levels
    are matched against what is sold (omit them for the catalog's defaults),
    the shelves are a line of their own on the BOM with their own part number,
    and a level spacing the catalog does not allow is refused.

    Shelving sold as posts and shelves rather than as a bay works the same
    way: a pack with an `upright` component and no `bay` puts the series on
    the group line and the posts on their own, four of them, counted in the
    packs the maker sells them in (`rules.uprights_per_pack`).

    `detail="full"` (the default with a catalog) adds the beams under each
    deck, the diagonal braces on the sides and the foot plates — decoration
    that never collides, so the uprights and decks stay the only thing a
    robot can hit."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("rack")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if levels is not None and "levels" in spec.params():
            params["levels"] = spec.choose("levels", levels)
        size = _sized_box(spec, params, size, ("width_mm", "depth_mm", "height_mm"))
        if params.get("levels") is not None:
            levels = int(round(float(params["levels"])))
        section = spec.dimension_mm("upright", "section", None)
        if section is None:
            section = spec.dimension_mm("bay", "upright", 40.0)
        upright = upright if upright is not None else _mm(section)
        shelf_thickness = (
            shelf_thickness
            if shelf_thickness is not None
            else _mm(spec.dimension_mm("shelf", "thickness", 30.0))
        )
        beam_mm = spec.dimension_mm("bay", "beam", 40.0)
        pitch_min = spec.rule("level_pitch_min_mm")
        pitch = size[2] * 1000.0 / levels
        if pitch_min is not None and pitch < float(pitch_min) - 1e-6:
            raise ValueError(
                f"{spec.id}: {levels} levels in {size[2] * 1000:.0f} mm leaves {pitch:.0f} mm "
                f"between shelves, under the {float(pitch_min):.0f} mm this rack allows — "
                f"take a level out or a taller bay"
            )
        manufacturer = manufacturer or spec.manufacturer

    mode = _detail(detail, spec is not None)
    if spec is None:
        beam_mm = 40.0
    if size is None:
        raise ValueError("rack: size is required without a catalog")
    levels = 4 if levels is None else levels
    upright = 0.04 if upright is None else upright
    shelf_thickness = 0.03 if shelf_thickness is None else shelf_thickness
    if levels < 1:
        raise ValueError("rack: levels must be at least one")
    lx, wy, h = (float(v) for v in size)
    if min(lx, wy, h) <= 0:
        raise ValueError("rack: size must be positive")

    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    built = Built(name)
    uprights: list[str] = []

    def world(dx: float, dy: float) -> tuple[float, float]:
        return x + c * dx - s * dy, y + s * dx + c * dy

    for i, (sx, sy) in enumerate([(-1, -1), (1, -1), (1, 1), (-1, 1)]):
        px, py = world(sx * (lx / 2 - upright / 2), sy * (wy / 2 - upright / 2))
        built.obstacles.append(
            scene.add_box(f"{name}/uprights/c{i}", size=(upright, upright, h),
                          position=(px, py, z0 + h / 2), quaternion=q, color=color)
        )
        uprights.append(f"{name}/uprights/c{i}")
    bay_drawn = mode == "full" and _load_trim(
        scene, built, spec, "bay", f"{name}/trim/bay", (x, y, z0), q,
        width=lx, depth=wy, height=h, upright=upright, beam=beam_mm / 1000.0,
        shelf_thickness=shelf_thickness, levels=levels,
    )
    if bay_drawn:
        for pname in uprights:
            scene.set_obstacle_visible(pname, False)
    elif mode == "full":
        for i, pname in enumerate(uprights):
            lo, hi = scene.obstacle_bounds(pname)
            _trim(scene, built, f"{name}/trim/foot{i}", (upright * 2, upright * 2, upright / 4),
                  ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, z0 + upright / 8), q, DARK_STEEL)

    # Shelves are evenly spaced with the top one at `height`; each frame sits
    # on the deck, where the parts go.
    for level in range(levels):
        top = z0 + h * (level + 1) / levels
        built.obstacles.append(
            scene.add_box(f"{name}/shelves/l{level}",
                          size=(lx - 2 * upright, wy - 2 * upright, shelf_thickness),
                          position=(x, y, top - shelf_thickness / 2), quaternion=q, color=color)
        )
        fname = f"{name}/level{level}"
        scene.add_frame(fname, position=(x, y, top), quaternion=q)
        built.frames.append(fname)
        if mode == "full":
            drawn = _load_trim(
                scene, built, spec, "shelf", f"{name}/trim/l{level}", (x, y, top), q,
                width=lx, depth=wy, thickness=shelf_thickness, upright=upright,
            )
            if drawn:
                scene.set_obstacle_visible(f"{name}/shelves/l{level}", False)
            if not bay_drawn:
                # The beams the deck rests on, front and back.
                beam = beam_mm / 1000.0
                for edge, side in (("f", -1.0), ("b", 1.0)):
                    px, py = world(0.0, side * (wy / 2 - upright / 2))
                    _trim(scene, built, f"{name}/trim/beam{level}{edge}",
                          (lx - 2 * upright, upright * 0.6, beam),
                          (px, py, top - shelf_thickness - beam / 2), q, DARK_STEEL)

    if mode == "full" and not bay_drawn:
        # A diagonal on each side — what keeps shelving from racking over.
        run, rise = wy - upright, h
        brace = math.hypot(run, rise)
        tilt = _mul_quat(q, _pitch_quat(math.atan2(rise, run)))
        for side, sx in (("l", -1.0), ("r", 1.0)):
            px, py = world(sx * (lx / 2 - upright / 2), 0.0)
            _trim(scene, built, f"{name}/trim/brace_{side}",
                  (upright / 3, brace, upright / 3), (px, py, z0 + h / 2), tilt, DARK_STEEL)

    if spec is None:
        # `levels` as text: it is what the rack is, not a quantity to sum.
        scene.set_part(name, kind="group", category="structure.rack", qty=1,
                       **_identity(model, manufacturer, {"levels": str(levels), **attributes}))
        return built

    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    # Shelving is sold two ways. A bay is one article you order as a unit (the
    # shelves on top of it are extra); a post system has no such article — you
    # buy posts and shelves — so the group line carries the series name and the
    # posts get a line of their own, priced by the pack they come in.
    bay = spec.has_component("bay")
    scene.set_part(
        name, kind="group",
        category=spec.category("bay", "structure.rack") if bay else "structure.rack",
        qty=1, catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or (spec.part_number("bay", **params) if bay else spec.name),
        **({"description": spec.name} if bay else {}),
        **{**recorded, **(_kg(spec.mass_kg("bay", **params)) if bay else {}), **attributes},
    )
    if spec.has_component("upright"):
        per_pack = float(spec.rule("uprights_per_pack", 1) or 1)
        scene.set_part(
            f"{name}/uprights", kind="group",
            category=spec.category("upright", "structure.rack"),
            qty=max(1, math.ceil(4 / per_pack)),
            catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=spec.part_number("upright", **params),
            **_kg(spec.mass_kg("upright", **params)),
        )
    if spec.has_component("shelf"):
        scene.set_part(
            f"{name}/shelves", kind="group", category=spec.category("shelf", "structure.rack.shelf"),
            qty=levels, catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=shelf_model or spec.part_number("shelf", **params),
            **_kg(spec.mass_kg("shelf", **params)),
        )
    return built


# ------------------------------------------------------------------ conveyor


def conveyor(
    scene,
    name: str,
    length: Optional[float] = None,
    width: Optional[float] = None,
    position: Point2 | Point3 = (0.0, 0.0),
    *,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    direction: Point2 = (1.0, 0.0),
    speed: Optional[float] = None,
    running: bool = False,
    zone_height: float = 0.15,
    belt_thickness: Optional[float] = None,
    rail: Optional[float] = None,
    legs: bool = True,
    leg: Optional[float] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = DARK_STEEL,
    **attributes,
) -> Built:
    """A belt conveyor: `length` along `direction`, `width` across, its belt
    surface centred at `position` (x, y, z of the surface). Builds the body —
    belt slab, two side rails, legs — as obstacles under `<name>/`, and the
    conveyor *device* `<name>` whose transport zone sits on the belt
    (`zone_height` tall, `speed` along `direction`). The part is pinned on the
    device (`conveyor`): the body is its geometry, not a second product. Adds
    the frames `<name>/infeed` and `<name>/outfeed` at the belt ends.

    With `catalog=` — the id of a conveyor spec pack, or a package directory —
    a conveyor you can order: the length, belt width and stand height are
    matched against the ones that are sold (omit them and the catalog's
    defaults apply, so `position` may be given as (x, y)), the speed is
    checked against the range the drive covers, and the mass follows the
    length. The stands are spaced by the catalog's maximum span and land on
    the BOM as their own line."""
    spec = None
    params: dict = {}
    stand_span_mm = None
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("conveyor")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if length is not None and "length_mm" in spec.params():
            params["length_mm"] = spec.choose("length_mm", round(length * 1000.0, 3))
        if width is not None and "width_mm" in spec.params():
            params["width_mm"] = spec.choose("width_mm", round(width * 1000.0, 3))
        if len(position) > 2 and "height_mm" in spec.params():
            params["height_mm"] = spec.choose("height_mm", round(float(position[2]) * 1000.0, 3))
        length = _sized(params, "length_mm", length)
        width = _sized(params, "width_mm", width)
        speed = spec.behavior("speed_mps", speed)
        belt_thickness = (
            belt_thickness
            if belt_thickness is not None
            else _mm(spec.dimension_mm("unit", "belt_thickness", 50.0))
        )
        rail = rail if rail is not None else _mm(spec.dimension_mm("unit", "rail", 30.0))
        if spec.has_component("stand"):
            leg = leg if leg is not None else _mm(spec.dimension_mm("stand", "leg", 50.0))
            stand_span_mm = spec.rule("stand_span_max_mm")
        manufacturer = manufacturer or spec.manufacturer

    mode = _detail(detail, spec is not None)
    if length is None or width is None:
        raise ValueError("conveyor: length and width are required without a catalog")
    speed = 0.2 if speed is None else speed
    belt_thickness = 0.05 if belt_thickness is None else belt_thickness
    rail = 0.03 if rail is None else rail
    leg = 0.05 if leg is None else leg

    dx, dy = float(direction[0]), float(direction[1])
    norm = math.hypot(dx, dy)
    if norm < 1e-9:
        raise ValueError("conveyor: direction must be a non-zero xy vector")
    dx, dy = dx / norm, dy / norm
    yaw = math.atan2(dy, dx)
    q = _yaw_quat(yaw)
    x, y = float(position[0]), float(position[1])
    if len(position) > 2:
        z = float(position[2])
    elif params.get("height_mm") is not None:
        z = float(params["height_mm"]) / 1000.0
    else:
        raise ValueError("conveyor: position needs the belt height (x, y, z)")
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
    stands = 0
    if legs and z - belt_thickness > leg:
        h = z - belt_thickness
        # Without a catalog: a pair at each end, as before. With one: pairs
        # spaced by the catalog's maximum span, so a longer run gets more.
        count = 2
        if stand_span_mm:
            count = max(2, math.ceil(length * 1000.0 / float(stand_span_mm)) + 1)
        # Legacy inset: a leg's width in from each end. With a catalog the
        # stands sit flush with the belt ends (half a leg in) and spread
        # evenly, so the outermost pair stays under the pulleys.
        run = length - (2 * leg if spec is None else leg)
        for i in range(count):
            along = -run / 2 + (run * i / (count - 1) if count > 1 else 0.0)
            ex, ey = dx * along, dy * along
            pair = []
            for side, s in (("l", 1.0), ("r", -1.0)):
                ox, oy = nx * s * (width / 2 - leg / 2), ny * s * (width / 2 - leg / 2)
                end = ("in", "out")[i > 0] if spec is None else f"s{i}"
                pname = f"{name}/leg_{end}{side}" if spec is None else f"{name}/stands/{end}_{side}"
                pair.append(
                    scene.add_box(pname, size=(leg, leg, h),
                                  position=(x + ex + ox, y + ey + oy, h / 2), quaternion=q, color=color)
                )
            built.obstacles.extend(pair)
            if mode == "full":
                drawn = _load_trim(
                    scene, built, spec, "stand", f"{name}/trim/s{stands}",
                    (x + ex, y + ey, 0.0), q,
                    height=z, belt_thickness=belt_thickness, width=width, leg=leg,
                )
                if drawn:
                    for pname in pair:
                        scene.set_obstacle_visible(pname, False)
                else:
                    for side, s in (("l", 1.0), ("r", -1.0)):
                        ox, oy = nx * s * (width / 2 - leg / 2), ny * s * (width / 2 - leg / 2)
                        _trim(scene, built, f"{name}/trim/foot_{stands}{side}",
                              (leg * 1.8, leg * 1.8, leg / 4),
                              (x + ex + ox, y + ey + oy, leg / 8), q, DARK_STEEL)
                    _trim(scene, built, f"{name}/trim/tie_{stands}",
                          (leg * 0.7, width - leg, leg * 0.7),
                          (x + ex, y + ey, h * 0.25), q, color)
            stands += 1
    if mode == "full" and _load_trim(
        scene, built, spec, "unit", f"{name}/trim/unit", (x, y, 0.0), q,
        length=length, width=width, height=z, belt_thickness=belt_thickness, rail=rail,
    ):
        # The pack draws the whole body — the massing keeps collision only.
        for part in ("belt", "rail_l", "rail_r"):
            scene.set_obstacle_visible(f"{name}/{part}", False)
    elif mode == "full":
        # The rollers the belt runs on, and the drive under the outfeed end.
        # A nose roller is about as thick as the frame it sits in.
        roller = belt_thickness * 0.45
        for end, e in (("head", 1.0), ("tail", -1.0)):
            _trim_cylinder(
                scene, built, f"{name}/trim/{end}_roller", roller, width,
                (x + dx * e * (length / 2 - roller), y + dy * e * (length / 2 - roller),
                 z - belt_thickness / 2),
                _axis_quat(nx, ny), DARK_STEEL,
            )
        drive_l, drive_w = min(0.22, length / 4), min(0.16, width * 0.6)
        along = length / 2 - drive_l
        _trim(scene, built, f"{name}/trim/drive", (drive_l, drive_w, drive_w),
              (x + dx * along, y + dy * along, z - belt_thickness - drive_w / 2), q, MOTOR)
        motor = drive_w * 0.75
        across = width / 2 + motor / 2
        _trim_cylinder(
            scene, built, f"{name}/trim/motor", motor / 2, motor,
            (x + dx * along + nx * across, y + dy * along + ny * across,
             z - belt_thickness - drive_w / 2),
            _axis_quat(nx, ny), MOTOR,
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

    if spec is None:
        scene.set_part(name, kind="device", category="conveyor", **_identity(model, manufacturer, attributes))
        return built

    # The device row is the unit itself, so it carries the part number you
    # would order it by; the series name goes in the description.
    # The speed that was actually set, not the catalog's default.
    running = {**spec.behaviors(), "speed_mps": speed}
    recorded = {key: str(_plain(value)) for key, value in {**params, **running}.items()}
    scene.set_part(
        name, kind="device", category=spec.category("unit", "conveyor"),
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("unit", **params), description=spec.name,
        **{**recorded, **_kg(spec.mass_kg("unit", **params)), **attributes},
    )
    if stands and spec.has_component("stand"):
        scene.set_part(
            f"{name}/stands", kind="group", category=spec.category("stand", "structure.pedestal"),
            qty=stands, catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=spec.part_number("stand", **params),
            **_kg(spec.mass_kg("stand", **params)),
        )
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
    height: Optional[float] = None,
    beam_height: Optional[float] = None,
    column: float = 0.04,
    watch_robot: bool = True,
    watch: Optional[list[str]] = None,
    catalog: Optional["CatalogRef"] = None,
    resolution: Optional[float] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = FENCE_POST,
    **attributes,
) -> Built:
    """A light curtain between two floor points: two mounting columns
    `<name>/column_a|b` of `height` (1.2 m unless given), and a beam sensor
    `<name>` at `beam_height` (half the `height` by default) spanning the
    gap between their lens faces — pulled in off the column centres, so the
    curtain is not born tripped by its own housings. With the defaults it
    trips on anything that enters the field, robot links and objects alike;
    `watch=[...]` with `watch_robot=False` narrows it to the named objects,
    `watch=[]` to robot links alone. The part (`sensor.light_curtain`) is
    pinned on the sensor; the columns are its mounting geometry.

    With `catalog=` — the id of a light-curtain spec pack, or a package
    directory — a curtain you can order: `height` is the protective height
    and is matched against the ones sold, `resolution` (mm — the smallest
    object it must catch: 14 for a finger, 25 for a hand) picks the type,
    the columns take the maker's section, and the BOM row carries the model
    number of the emitter/receiver pair and its mass. A beam longer than the
    curtain's operating range is refused with the numbers — the same
    `range_mm` a requirement check would ask of it."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("light_curtain")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if height is not None and "protective_height_mm" in params:
            params["protective_height_mm"] = spec.choose(
                "protective_height_mm", round(height * 1000.0, 3))
        if resolution is not None and "resolution_mm" in params:
            params["resolution_mm"] = spec.choose("resolution_mm", resolution)
        height = _sized(params, "protective_height_mm", height)
        manufacturer = manufacturer or spec.manufacturer
    height = 1.2 if height is None else float(height)
    if height <= 0:
        raise ValueError("light_curtain: height must be positive")

    (xa, ya), (xb, yb) = (float(frm[0]), float(frm[1])), (float(to[0]), float(to[1]))
    span = math.hypot(xb - xa, yb - ya)
    reach: dict = {}
    if spec is not None:
        limit = _curtain_range_mm(spec, params)
        _within_range(spec, span, limit, "the curtain's operating range")
        if limit is not None:
            # The range of the type chosen, not the series figure — what a
            # `range_mm` requirement is checked against.
            reach = {"range_mm": limit}
    zb = beam_height if beam_height is not None else height / 2
    # The columns face each other across the beam: the maker's section is
    # `section_w` across it (the lens face) and `section_d` along it.
    section = (column, column)
    if spec is not None:
        across = _mm(spec.dimension_mm("curtain", "section_w"))
        along = _mm(spec.dimension_mm("curtain", "section_d"))
        section = (across or column, along or column)
    q = _yaw_quat(math.atan2(yb - ya, xb - xa))
    built = Built(name)
    for tag, (px, py) in (("a", (xa, ya)), ("b", (xb, yb))):
        built.obstacles.append(
            scene.add_box(f"{name}/column_{tag}", size=(section[1], section[0], height),
                          position=(px, py, height / 2), quaternion=q, color=color)
        )
    # The field spans the gap between the lens faces, not the column
    # centres: the sensor watches anything in the field — its own posts
    # included — and a beam threaded through them would read tripped
    # forever. The beam is a capsule whose cap reaches its radius past the
    # endpoint, so the pull-in is half the housing plus that radius.
    inset = section[1] / 2 + 0.005 + 1e-3
    if span <= 2 * inset:
        raise ValueError(
            f"light_curtain: the columns stand {span * 1e3:.0f} mm apart — inside "
            f"their own {section[1] * 1e3:.0f} mm housings, leaving no field between them"
        )
    ux, uy = (xb - xa) / span, (yb - ya) / span
    scene.add_beam_sensor(
        name, frm=(xa + ux * inset, ya + uy * inset, zb),
        to=(xb - ux * inset, yb - uy * inset, zb), watch=watch, watch_robot=watch_robot,
    )
    built.sensors.append(name)
    if spec is None:
        scene.set_part(name, kind="sensor", category="sensor.light_curtain", **_identity(model, manufacturer, attributes))
        return built
    scene.set_part(
        name, kind="sensor", category=spec.category("curtain", "sensor.light_curtain"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("curtain", **params), description=spec.name,
        **{**_recorded(spec, params), **reach, **_kg(spec.mass_kg("curtain", **params)), **attributes},
    )
    return built


def _curtain_range_mm(spec, params: dict) -> Optional[float]:
    """How far apart the pair may stand. A curtain's range goes with its
    resolution (the finger type reaches less far than the hand type), so a
    pack may carry it per resolution under `rules.range_mm_by_resolution`;
    otherwise the series figure in `specs.range_mm` applies."""
    table = spec.rule("range_mm_by_resolution")
    resolution = params.get("resolution_mm")
    if isinstance(table, dict) and resolution is not None:
        for key, value in table.items():
            try:
                if abs(float(key) - float(resolution)) < 1e-6:
                    return float(value)
            except (TypeError, ValueError):
                continue
    value = spec.specs().get("range_mm")
    return None if value is None or isinstance(value, str) else float(value)


def _within_range(spec, span: float, limit_mm: Optional[float], what: str) -> None:
    if limit_mm is not None and span * 1000.0 > limit_mm + 1e-6:
        raise ValueError(
            f"{spec.id}: the beam spans {span * 1000.0:.0f} mm but {what} is "
            f"{_plain(limit_mm)} mm"
        )


def _recorded(spec, params: dict) -> dict:
    """What the BOM row carries besides the model number: the datasheet
    figures and the axes as chosen — the chosen value wins over the series
    figure of the same name, so a 1 m diffuse sensor does not answer a
    requirement with the 30 m its through-beam sibling reaches. Numbers
    stay numbers, which is what a requirement check compares."""
    out: dict = {}
    for key, value in {**spec.specs(), **params}.items():
        out[key] = float(value) if isinstance(value, (int, float)) and not isinstance(value, bool) else str(value)
    return out


# ------------------------------------------------------------ photoelectric

# The pale acrylic of a corner-cube reflector.
REFLECTOR: Color = (0.86, 0.86, 0.80)


def photoelectric(
    scene,
    name: str,
    frm: Point3,
    to: Point3,
    *,
    body: Optional[Point3] = None,
    watch_robot: bool = False,
    watch: Optional[list[str]] = None,
    catalog: Optional["CatalogRef"] = None,
    sensing: Optional[str] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = DARK_STEEL,
    **attributes,
) -> Built:
    """A photoelectric sensor: a beam `<name>` from the lens at `frm` to
    `to` (both in metres, 3D) that trips on the named objects (`watch` — a
    workpiece arriving on the belt) and/or on any robot link
    (`watch_robot`), and the sensor body `<name>/body` behind the lens —
    `body = (depth, width, height)`, the amplifier-in-head block sold by
    the million (20 x 11 x 31 mm unless given). The part
    (`sensor.photoelectric`) is pinned on the sensor.

    What stands at `to` follows the sensing method: a through-beam pair
    puts the receiver `<name>/receiver` there, a retroreflective sensor its
    reflector `<name>/reflector`, a diffuse one nothing — the beam ends on
    the target itself.

    With `catalog=` — the id of a photoelectric spec pack, or a package
    directory — a sensor you can order: `sensing` picks the method the pack
    sells (`through_beam` / `retroreflective` / `diffuse` / ...), the other
    axes (`sensing_range_mm`, `output`, ...) are chosen by name, the body
    takes the maker's dimensions, the BOM row carries the model number and
    mass, and a reflector the maker sells separately is a line of its own.
    A beam longer than the sensing range is refused with the numbers — the
    same `sensing_range_mm` a requirement check would ask of it."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("photoelectric")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if sensing is not None and "sensing" in params:
            params["sensing"] = spec.choose("sensing", sensing)
        if body is None:
            sides = [spec.dimension_mm("sensor", key) for key in ("depth", "width", "height")]
            if all(side is not None for side in sides):
                body = (sides[0] / 1000.0, sides[1] / 1000.0, sides[2] / 1000.0)
        manufacturer = manufacturer or spec.manufacturer
    method = str(params.get("sensing") or sensing or "diffuse")
    depth, width, height = (0.02, 0.011, 0.031) if body is None else (float(v) for v in body)
    if min(depth, width, height) <= 0:
        raise ValueError("photoelectric: body must be positive")

    (xa, ya, za) = (float(frm[0]), float(frm[1]), float(frm[2]))
    (xb, yb, zb) = (float(to[0]), float(to[1]), float(to[2]))
    span = math.sqrt((xb - xa) ** 2 + (yb - ya) ** 2 + (zb - za) ** 2)
    if span <= 0:
        raise ValueError("photoelectric: frm and to must be different points")
    if spec is not None:
        limit = params.get("sensing_range_mm", spec.specs().get("sensing_range_mm"))
        limit = None if limit is None or isinstance(limit, str) else float(limit)
        _within_range(spec, span, limit, "the sensing range")
    # The body looks along the beam: its lens face is at `frm`, the block
    # behind it. On the floor plane, since a sensor is mounted level.
    yaw = math.atan2(yb - ya, xb - xa)
    q = _yaw_quat(yaw)
    ux, uy = math.cos(yaw), math.sin(yaw)
    built = Built(name)
    built.obstacles.append(
        scene.add_box(f"{name}/body", size=(depth, width, height),
                      position=(xa - ux * depth / 2, ya - uy * depth / 2, za), quaternion=q, color=color)
    )
    if method == "through_beam":
        # The receiver is the same block, looking back at the emitter.
        built.obstacles.append(
            scene.add_box(f"{name}/receiver", size=(depth, width, height),
                          position=(xb + ux * depth / 2, yb + uy * depth / 2, zb), quaternion=q, color=color)
        )
    elif method == "retroreflective":
        plate = [None if spec is None else _mm(spec.dimension_mm("reflector", key))
                 for key in ("thickness", "width", "height")]
        thick, wide, tall = plate[0] or 0.008, plate[1] or 0.06, plate[2] or 0.06
        built.obstacles.append(
            scene.add_box(f"{name}/reflector", size=(thick, wide, tall),
                          position=(xb + ux * thick / 2, yb + uy * thick / 2, zb), quaternion=q, color=REFLECTOR)
        )
    scene.add_beam_sensor(name, frm=(xa, ya, za), to=(xb, yb, zb), watch=watch, watch_robot=watch_robot)
    built.sensors.append(name)
    if spec is None:
        scene.set_part(name, kind="sensor", category="sensor.photoelectric", **_identity(model, manufacturer, attributes))
        return built
    scene.set_part(
        name, kind="sensor", category=spec.category("sensor", "sensor.photoelectric"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("sensor", **params), description=spec.name,
        **{**_recorded(spec, params), **_kg(spec.mass_kg("sensor", **params)), **attributes},
    )
    if method == "retroreflective" and spec.has_component("reflector"):
        # Sold separately and required — a line of its own, like a
        # cabinet's plinth.
        scene.set_part(
            f"{name}/reflector", kind="obstacle",
            category=spec.category("reflector", "sensor.photoelectric"), qty=1,
            catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=spec.part_number("reflector", **params),
            **_kg(spec.mass_kg("reflector", **params)),
        )
    return built


# ---------------------------------------------------------------- proximity


def proximity(
    scene,
    name: str,
    frm: Point3,
    direction: Point3 = (1.0, 0.0, 0.0),
    *,
    sensing_range: Optional[float] = None,
    body: Optional[tuple[float, float]] = None,
    watch: Optional[list[str]] = None,
    watch_robot: bool = False,
    catalog: Optional["CatalogRef"] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = STEEL,
    **attributes,
) -> Built:
    """An inductive proximity switch: a beam `<name>` from the sensing face
    at `frm`, `sensing_range` along `direction` — the few millimetres a
    metal target must come within (4 mm unless given) — and the threaded
    barrel `<name>/body` behind the face, `body = (diameter, length)` in
    metres (an M12 x 47 mm barrel unless given). The beam trips on the
    named objects (`watch`) and/or on any robot link. The part
    (`sensor.proximity`) is pinned on the sensor.

    With `catalog=` — the id of a proximity-switch spec pack, or a package
    directory — a switch you can order: the pack's axes (`size` M8/M12/M18/
    M30, `shield`, `output`, `contact`, `connection` …) are chosen by name,
    the sensing range is the model's (`sensing_range_mm`), the barrel takes
    the size the pack lists for the thread (`rules.body_mm_by_size`), and
    the BOM row carries the model number and mass."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("proximity")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if sensing_range is not None and "sensing_range_mm" in params:
            params["sensing_range_mm"] = spec.choose(
                "sensing_range_mm", round(sensing_range * 1000.0, 3))
        sensing_range = _sized(params, "sensing_range_mm", sensing_range)
        if body is None:
            body = _body_by_size(spec, params.get("size"))
        manufacturer = manufacturer or spec.manufacturer
    sensing_range = 0.004 if sensing_range is None else float(sensing_range)
    diameter, length = (0.012, 0.047) if body is None else (float(body[0]), float(body[1]))
    if sensing_range <= 0 or min(diameter, length) <= 0:
        raise ValueError("proximity: sensing_range and body must be positive")

    dx, dy, dz = (float(direction[0]), float(direction[1]), float(direction[2]))
    norm = math.sqrt(dx * dx + dy * dy + dz * dz)
    if norm <= 0:
        raise ValueError("proximity: direction must not be zero")
    ux, uy, uz = dx / norm, dy / norm, dz / norm
    xa, ya, za = float(frm[0]), float(frm[1]), float(frm[2])
    to = (xa + ux * sensing_range, ya + uy * sensing_range, za + uz * sensing_range)
    built = Built(name)
    centre = (xa - ux * length / 2, ya - uy * length / 2, za - uz * length / 2)
    if abs(uz) > 0.9:
        # Looking up or down: the barrel stands on end.
        size, q = (diameter, diameter, length), None
    else:
        size, q = (length, diameter, diameter), _yaw_quat(math.atan2(uy, ux))
    built.obstacles.append(
        scene.add_box(f"{name}/body", size=size, position=centre, quaternion=q, color=color)
    )
    scene.add_beam_sensor(name, frm=(xa, ya, za), to=to, watch=watch, watch_robot=watch_robot)
    built.sensors.append(name)
    if spec is None:
        scene.set_part(name, kind="sensor", category="sensor.proximity", **_identity(model, manufacturer, attributes))
        return built
    scene.set_part(
        name, kind="sensor", category=spec.category("sensor", "sensor.proximity"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("sensor", **params), description=spec.name,
        **{**_recorded(spec, params), **_kg(spec.mass_kg("sensor", **params)), **attributes},
    )
    return built


def _body_by_size(spec, size) -> Optional[tuple[float, float]]:
    """The barrel a thread size stands for — `rules.body_mm_by_size`
    (`{M12: [12, 47]}`, diameter and length), or the `sensor` component's
    fixed dimensions where a pack sells one size."""
    table = spec.rule("body_mm_by_size")
    if isinstance(table, dict) and size is not None:
        entry = table.get(str(size))
        if isinstance(entry, (list, tuple)) and len(entry) == 2:
            return float(entry[0]) / 1000.0, float(entry[1]) / 1000.0
    diameter = _mm(spec.dimension_mm("sensor", "diameter"))
    length = _mm(spec.dimension_mm("sensor", "length"))
    return (diameter, length) if diameter is not None and length is not None else None


# ------------------------------------------------------------- power supply


def power_supply(
    scene,
    name: str,
    position: Point3,
    *,
    size: Optional[Point3] = None,
    yaw: float = 0.0,
    catalog: Optional["CatalogRef"] = None,
    output_a: Optional[float] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = STEEL,
    **attributes,
) -> Built:
    """A DIN-rail power supply: the box `<name>/body`, `size = (width,
    depth, height)`, standing on `position` (the centre of its foot — on a
    rail inside a cabinet), turned by `yaw`. The part (`power_supply`)
    carries `output_v` / `output_a`, which is what `scene.check()` sums the
    cell's `current_a` against.

    With `catalog=` — the id of a power-supply spec pack, or a package
    directory — a unit you can order: `output_a` is matched against the
    ratings sold, the box takes the size the pack lists for that rating
    (`rules.size_mm_by_output_a`, width / depth / height), and the BOM row
    carries the model number, its mass and its rating."""
    spec = None
    params: dict = {}
    rating: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("power_supply")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if output_a is not None and "output_a" in params:
            params["output_a"] = spec.choose("output_a", output_a)
        if size is None:
            size = _size_by_rating(spec, params.get("output_a"))
        power = (spec.manifest.get("electrical") or {}).get("power") or {}
        for key in ("output_v", "output_a", "output_w"):
            if isinstance(power.get(key), (int, float)):
                rating[key] = float(power[key])
        manufacturer = manufacturer or spec.manufacturer
    if size is None:
        size = (0.04, 0.12, 0.12)
    w, d, h = (float(v) for v in size)
    if min(w, d, h) <= 0:
        raise ValueError("power_supply: size must be positive")
    x, y, z0 = float(position[0]), float(position[1]), float(position[2])
    built = Built(name)
    built.obstacles.append(
        scene.add_box(f"{name}/body", size=(w, d, h), position=(x, y, z0 + h / 2),
                      quaternion=_yaw_quat(yaw), color=color)
    )
    if spec is None:
        scene.set_part(name, kind="group", category="power_supply", **_identity(model, manufacturer, attributes))
        return built
    scene.set_part(
        name, kind="group", category=spec.category("unit", "power_supply"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("unit", **params), description=spec.name,
        **{**rating, **_recorded(spec, params), **_kg(spec.mass_kg("unit", **params)), **attributes},
    )
    return built


def _size_by_rating(spec, output_a) -> Optional[Point3]:
    """The box a rating comes in — `rules.size_mm_by_output_a` (`{10: [38,
    122, 124]}`, width / depth / height), or the `unit` component's fixed
    dimensions where a pack sells one size."""
    table = spec.rule("size_mm_by_output_a")
    if isinstance(table, dict) and output_a is not None:
        for key, entry in table.items():
            try:
                same = abs(float(key) - float(output_a)) < 1e-6
            except (TypeError, ValueError):
                same = False
            if same and isinstance(entry, (list, tuple)) and len(entry) == 3:
                return (float(entry[0]) / 1000.0, float(entry[1]) / 1000.0, float(entry[2]) / 1000.0)
    sides = [_mm(spec.dimension_mm("unit", key)) for key in ("width", "depth", "height")]
    if any(side is None for side in sides):
        return None
    return (sides[0], sides[1], sides[2])  # type: ignore[return-value]


# ---------------------------------------------------------------- remote I/O


def remote_io(
    scene,
    name: str,
    position: Point3,
    *,
    catalog: Optional["CatalogRef"] = None,
    di_units: Optional[int] = None,
    do_units: Optional[int] = None,
    points_per_unit: int = 16,
    uplink=None,
    place: Optional[str] = None,
    yaw: float = 0.0,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = DARK_STEEL,
    **attributes,
) -> Built:
    """A remote I/O station on a DIN rail: the bus coupler `<name>/coupler`
    and its DI / DO terminal units `<name>/di{i}` / `<name>/do{i}` side by
    side from `position` (the centre of the coupler's foot; the units run
    along local +X, turned by `yaw`), and the I/O node `<name>`
    (`kind="remote_io"`, hung off `uplink` the way `add_io_node` takes it)
    with a channel per point — `DI0…` and `DO0…`. The coupler is the part
    (`io.remote`); each unit is a BOM line of its own.

    With `catalog=` — the id of a remote-I/O spec pack, or a package
    directory — a station you can order: `di_units` / `do_units` are matched
    against what the pack sells, `logic` (PNP / NPN) picks the unit models,
    the coupler and the units take the maker's widths and point counts, and
    every line carries its model number and mass."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("remote_io")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        for given, key in ((di_units, "di_units"), (do_units, "do_units")):
            if given is not None and key in params:
                params[key] = spec.choose(key, given)
        di_units = int(params["di_units"]) if "di_units" in params else di_units
        do_units = int(params["do_units"]) if "do_units" in params else do_units
        manufacturer = manufacturer or spec.manufacturer
    di_units = 1 if di_units is None else int(di_units)
    do_units = 1 if do_units is None else int(do_units)
    if di_units < 0 or do_units < 0 or points_per_unit <= 0:
        raise ValueError("remote_io: unit counts must not be negative")

    def dims(role: str, default: tuple[float, float, float]) -> tuple[float, float, float]:
        if spec is None:
            return default
        sides = [_mm(spec.dimension_mm(role, key)) for key in ("width", "depth", "height")]
        return tuple(  # type: ignore[return-value]
            side if side is not None else fallback for side, fallback in zip(sides, default)
        )

    def points(role: str) -> int:
        if spec is None:
            return points_per_unit
        value = spec.dimension_mm(role, "points", points_per_unit)
        return int(value) if value else points_per_unit

    coupler = dims("coupler", (0.046, 0.071, 0.100))
    di_size = dims("di", (0.012, coupler[1], coupler[2]))
    do_size = dims("do", (0.012, coupler[1], coupler[2]))
    di_points, do_points = points("di"), points("do")

    x, y, z0 = float(position[0]), float(position[1]), float(position[2])
    q = _yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    built = Built(name)
    cursor = 0.0  # along local +X from the coupler's left edge

    def place_box(label: str, size: tuple[float, float, float]) -> str:
        nonlocal cursor
        dx = cursor - coupler[0] / 2 + size[0] / 2
        made = scene.add_box(f"{name}/{label}", size=size,
                             position=(x + c * dx, y + s * dx, z0 + size[2] / 2), quaternion=q, color=color)
        built.obstacles.append(made)
        cursor += size[0]
        return made

    place_box("coupler", coupler)
    di_names = [place_box(f"di{i}", di_size) for i in range(di_units)]
    do_names = [place_box(f"do{i}", do_size) for i in range(do_units)]

    from .io import channels as _channels

    logic = str(params["logic"]).lower() if params.get("logic") is not None else None
    chans = (_channels("di", di_units * di_points, "DI", logic=logic)
             + _channels("do", do_units * do_points, "DO", logic=logic))
    coupler_model = model or (spec.part_number("coupler", **params) if spec is not None else None)
    scene.add_io_node(name, kind="remote_io", uplink=uplink, channels=chans, place=place, model=coupler_model)
    built.nodes.append(name)
    counts = {"di": float(di_units * di_points), "do": float(do_units * do_points)}
    if spec is None:
        scene.set_part(name, kind="io_node", category="io.remote",
                       **{**counts, **_identity(model, manufacturer, attributes)})
        return built
    scene.set_part(
        name, kind="io_node", category=spec.category("coupler", "io.remote"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=coupler_model, description=spec.name,
        **{**_recorded(spec, params), **counts, **_kg(spec.mass_kg("coupler", **params)), **attributes},
    )
    for role, names in (("di", di_names), ("do", do_names)):
        if not names or not spec.has_component(role):
            continue
        for unit in names:
            scene.set_part(
                unit, kind="obstacle", category=spec.category(role, "io.remote"), qty=1,
                catalog=spec.catalog_ref, manufacturer=manufacturer,
                model=spec.part_number(role, **params), **_kg(spec.mass_kg(role, **params)),
            )
    return built


__all__ = [
    "Built", "cabinet", "conveyor", "fence", "light_curtain", "pallet",
    "pedestal", "photoelectric", "power_supply", "proximity", "rack",
    "remote_io", "stairs", "table", "wall",
]


# -------------------------------------------------------------------- stairs


def stairs(
    scene,
    name: str,
    *,
    steps: Optional[int] = None,
    rise: Optional[float] = None,
    tread: Optional[float] = None,
    width: Optional[float] = None,
    position: Point2 | Point3 = (0.0, 0.0),
    yaw: float = 0.0,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    nosing: Optional[float] = None,
    rail_height: Optional[float] = None,
    rails: bool = True,
    legs: bool = True,
    model: Optional[str] = None,
    rail_model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = STEEL,
    tread_color: Color = CHECKER_PLATE,
    rail_color: Color = SAFETY_ORANGE,
    **attributes,
) -> Built:
    """A steel stair flight, the kind bolted against a mezzanine: `steps`
    checker-plate treads climbing `rise` per step along local +x from
    `position` (rotated by `yaw`), carried on a plate stringer each side and
    handed by a tubular rail in safety orange.

    Every tread is a *walkable* box, so a legged machine's footfalls snap
    onto it (see the legged guide); everything else —
    stringers, support legs, the handrail — is an ordinary obstacle, so an
    AGV driven into the flight fails its aisle check and an arm sweeping
    through the rail collides. Adds the frames `<name>/foot` (on the floor
    at the bottom) and `<name>/top` (the landing edge) — author the vehicle
    path's z between them — and pins the flight (`structure.stairs`).

    Each tread overhangs the one below by `nosing`, the way a real one does.
    That overlap is what a walking machine needs at the seam: **keep it at
    least twice the foot radius**, or a foothold lands in the gap between
    two treads and the bake refuses it by name.

    With `catalog=` — the id of a stair spec pack, or a package directory —
    the flight is one you can order: the rise, tread, width and number of
    steps are matched against what is sold, the sections come from the pack,
    the handrails are a line of their own on the BOM (one per side), and a
    combination the maker does not sell — too steep, too shallow for the
    walking rule `2 x rise + tread` — is refused with the numbers.

    `rails=False` drops the handrail (a flight against a wall);
    `legs=False` drops the support leg under the high end, which a flight
    slung between two landings — a storey of a building stair — does not
    have."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("stairs")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        for given, key in ((rise, "rise_mm"), (tread, "tread_mm"), (width, "width_mm")):
            if given is not None and key in spec.params():
                params[key] = spec.choose(key, round(float(given) * 1000.0, 3))
        if steps is not None and "steps" in spec.params():
            params["steps"] = spec.choose("steps", steps)
        rise = _sized(params, "rise_mm", rise)
        tread = _sized(params, "tread_mm", tread)
        width = _sized(params, "width_mm", width)
        if params.get("steps") is not None:
            steps = int(round(float(params["steps"])))
        manufacturer = manufacturer or spec.manufacturer

    rise = 0.175 if rise is None else float(rise)
    tread = 0.27 if tread is None else float(tread)
    width = 0.9 if width is None else float(width)
    if steps is None:
        raise ValueError("stairs: steps is required (a catalog pack defaults it)")
    steps = int(steps)
    if steps < 1:
        raise ValueError(f"stairs: steps must be >= 1, got {steps}")
    if rise <= 0 or tread <= 0 or width <= 0:
        raise ValueError("stairs: rise, tread and width must be positive")

    def dim(role: str, key: str, default_mm: float) -> float:
        """A drawn section: the pack's where it carries one, else the
        generator's own."""
        if spec is not None and spec.has_component(role):
            return float(_mm(spec.dimension_mm(role, key, default_mm)))
        return default_mm / 1000.0

    plate = dim("flight", "plate", 9.0)  # stringer plate
    stringer = dim("flight", "stringer", 250.0)  # its depth
    deck = dim("flight", "tread", 32.0)  # checker plate on its angle
    foot_plate = dim("flight", "foot", 90.0)  # levelling foot
    nose = nosing if nosing is not None else dim("flight", "nosing", 60.0)
    nose = min(float(nose), tread / 2.0)
    tube = dim("handrail", "tube", 42.7)
    post = dim("handrail", "post", 48.6)
    rail_h = rail_height if rail_height is not None else dim("handrail", "height", 900.0)
    ret = dim("handrail", "return", 450.0)

    # The rule the trade sizes stairs by — 2 x rise + tread, the pace of a
    # person on them. A pack that states it does not sell what falls outside.
    if spec is not None:
        low, high = spec.rule("walk_rule_min_mm"), spec.rule("walk_rule_max_mm")
        walk = 2.0 * rise * 1000.0 + tread * 1000.0
        if (low is not None and walk < float(low) - 1e-6) or (
            high is not None and walk > float(high) + 1e-6
        ):
            raise ValueError(
                f"{spec.id}: rise {rise * 1000:.0f} with tread {tread * 1000:.0f} gives "
                f"2R + T = {walk:.0f} mm, outside the "
                f"{float(low or 0):.0f}..{float(high or 0):.0f} mm this flight is sold "
                "in — take a deeper tread or a lower rise"
            )

    x0, y0 = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    c, s_ = math.cos(yaw), math.sin(yaw)

    def at(lx: float, ly: float, lz: float) -> tuple:
        return (x0 + c * lx - s_ * ly, y0 + s_ * lx + c * ly, z0 + lz)

    run, height = steps * tread, steps * rise
    pitch = math.atan2(rise, tread)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)

    def pitch_z(x: float) -> float:
        """The nosing line at `x` — what a stringer's top edge follows and
        what a handrail is measured from."""
        return rise * (x + nose) / tread + rise

    mode = _detail(detail, spec is not None)
    built = Built(name)

    # -- treads: checker plate, and the only walkable thing in the flight --
    for i in range(1, steps + 1):
        made = scene.add_box(
            f"{name}/tread{i:02d}",
            size=(tread + nose, width, deck),
            position=at((i - 0.5) * tread - nose / 2.0, 0.0, i * rise - deck / 2.0),
            quaternion=q,
            color=tread_color,
        )
        scene.set_obstacle_walkable(made, True)
        built.obstacles.append(made)

    # -- stringers: a plate each side, top edge on the nosing line ---------
    # Cut where the plate would otherwise run under the floor, as a real
    # flight is cut to meet it.
    start = max(0.0, tread * (stringer * cos_p - rise) / rise - nose)
    span = max(run - start, tread)
    xc = (start + run) / 2.0
    slope_q = _mul_quat(q, _slope_quat(-pitch))
    y_side = (width + plate) / 2.0
    for side, uy in (("l", 1.0), ("r", -1.0)):
        built.obstacles.append(
            scene.add_box(
                f"{name}/stringer_{side}",
                size=(span / cos_p, plate, stringer),
                position=at(
                    xc + (stringer / 2.0) * sin_p,
                    uy * y_side,
                    pitch_z(xc) - (stringer / 2.0) * cos_p,
                ),
                quaternion=slope_q,
                color=color,
            )
        )

    # -- what stands it up: a leg under the high end, levelling feet -------
    leg_x = run - max(0.15, tread / 2.0)
    leg_top = pitch_z(leg_x) - stringer * cos_p
    for side, uy in (("l", 1.0), ("r", -1.0)):
        if legs and leg_top > 0.1:
            built.obstacles.append(
                scene.add_box(
                    f"{name}/leg_{side}",
                    size=(0.06, 0.06, leg_top),
                    position=at(leg_x, uy * y_side, leg_top / 2.0),
                    quaternion=q,
                    color=color,
                )
            )
        if mode == "full":
            for label, lx in (("a", start + 0.05), ("b", leg_x)):
                _trim(
                    scene, built, f"{name}/trim/foot_{side}{label}",
                    (foot_plate, foot_plate, 0.012),
                    at(lx, uy * y_side, 0.006), q, DARK_STEEL,
                )

    # -- the handrail: sloped run, level return over the landing, posts ----
    if rails:
        rail_q = _mul_quat(q, _slope_quat(math.pi / 2.0 - pitch))
        flat_q = _axis_quat(c, s_)
        x_top = max(run - tread - nose, 0.0)  # the last nosing
        x_end = x_top + ret
        y_rail = y_side + tube / 2.0 + 0.02
        for side, uy in (("l", 1.0), ("r", -1.0)):
            ys = uy * y_rail
            built.obstacles.append(
                scene.add_cylinder(
                    f"{name}/handrails/rail_{side}",
                    radius=tube / 2.0,
                    length=max(x_top / cos_p, tube),
                    position=at(
                        x_top / 2.0,
                        ys,
                        (pitch_z(0.0) + pitch_z(x_top)) / 2.0 + rail_h,
                    ),
                    quaternion=rail_q,
                    color=rail_color,
                )
            )
            built.obstacles.append(
                scene.add_cylinder(
                    f"{name}/handrails/return_{side}",
                    radius=tube / 2.0,
                    length=max(x_end - x_top, tube),
                    position=at((x_top + x_end) / 2.0, ys, height + rail_h),
                    quaternion=flat_q,
                    color=rail_color,
                )
            )
            for label, lx, base, top in (
                ("a", 0.0, 0.0, pitch_z(0.0) + rail_h),  # on the floor
                ("b", x_top, pitch_z(x_top) - 0.05, height + rail_h),  # on the flight
                ("c", x_end, height, height + rail_h),  # on the landing
            ):
                built.obstacles.append(
                    scene.add_cylinder(
                        f"{name}/handrails/post_{side}{label}",
                        radius=post / 2.0,
                        length=max(top - base, post),
                        position=at(lx, ys, (base + top) / 2.0),
                        quaternion=q,
                        color=rail_color,
                    )
                )
            if mode == "full":
                _trim(
                    scene, built, f"{name}/handrails/trim/foot_{side}",
                    (foot_plate, foot_plate, 0.012), at(0.0, ys, 0.006), q, DARK_STEEL,
                )

    scene.add_frame(f"{name}/foot", position=at(0.0, 0.0, 0.0), quaternion=q)
    scene.add_frame(f"{name}/top", position=at(run, 0.0, height), quaternion=q)
    built.frames.extend([f"{name}/foot", f"{name}/top"])

    if spec is None:
        scene.set_part(
            name, kind="group", category="structure.stairs", qty=1,
            **_identity(model, manufacturer, {
                "rise_mm": str(round(rise * 1000)),
                "tread_mm": str(round(tread * 1000)),
                "width_mm": str(round(width * 1000)),
                "steps": str(steps),
                **attributes,
            }),
        )
        return built

    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    scene.set_part(
        name, kind="group", category=spec.category("flight", "structure.stairs"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("flight", **params), description=spec.name,
        **{**recorded, **_kg(spec.mass_kg("flight", **params)), **attributes},
    )
    # The rail is bought by the side, the way the flight is bought by the
    # flight — two lines, because that is how the order goes out.
    if rails and spec.has_component("handrail"):
        scene.set_part(
            f"{name}/handrails", kind="group",
            category=spec.category("handrail", "structure.stairs.rail"),
            qty=2, catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=rail_model or spec.part_number("handrail", **params),
            **_kg(spec.mass_kg("handrail", **params)),
        )
    return built


# ---------------------------------------------------------------------- wall


def _wall_openings(
    edge: int, openings: Sequence[Sequence[float]], length: float,
    head: float, height: float,
) -> list[tuple[float, float, float]]:
    """The openings on one edge as `(start, end, head)`, in order along it.

    Refused rather than clipped: an opening that runs off the end of its
    wall, or into the one beside it, is a floor plan that does not close,
    and a silently shortened door is a route that passes a wall that is
    not there."""
    out: list[tuple[float, float, float]] = []
    for spec in openings:
        values = [float(v) for v in spec]
        if len(values) < 3:
            raise ValueError(
                "wall: an opening is (edge, centre, width[, head]), "
                f"got {tuple(spec)!r}"
            )
        if int(values[0]) != edge:
            continue
        centre, width = values[1], values[2]
        clear = values[3] if len(values) > 3 else head
        if width <= 0 or clear <= 0:
            raise ValueError("wall: an opening's width and head must be positive")
        start, end = centre - width / 2.0, centre + width / 2.0
        if start < -1e-9 or end > length + 1e-9:
            raise ValueError(
                f"wall: the opening at {centre:.3f} m is {width:.3f} m wide, which "
                f"runs off edge {edge} ({length:.3f} m long)"
            )
        out.append((start, end, min(clear, height)))
    out.sort()
    for (_a0, a1, _ah), (b0, _b1, _bh) in zip(out, out[1:]):
        if b0 < a1 - 1e-9:
            raise ValueError(
                f"wall: two openings on edge {edge} overlap at {b0:.3f} m — "
                "one opening, or two with a pier between them"
            )
    return out


def wall(
    scene,
    name: str,
    path: Sequence[Point2],
    *,
    height: float = 2.7,
    thickness: float = 0.12,
    base_z: float = 0.0,
    closed: bool = False,
    openings: Sequence[Sequence[float]] = (),
    head: float = 2.1,
    detail: Optional[str] = None,
    color: Color = PLASTER,
    trim_color: Optional[Color] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    **attributes,
) -> Built:
    """A partition along `path` (floor corners, metres): `thickness` thick,
    `height` tall, standing off `base_z`. `closed` joins the last corner back
    to the first, so a four-corner path is a room.

    `openings=[(edge, centre, width), ...]` cuts a doorway `width` wide,
    centred `centre` metres along that edge, and spans the wall over it —
    the pier each side and the head above are ordinary obstacles, so a
    machine driven at the pier fails its aisle check while one sent through
    the opening passes. A fourth element sets that opening's clear height
    (`head` otherwise); at or above the wall's own height it is a gap
    through it, with nothing over. Each one adds the frame
    `<name>/opening{edge}_{i}` on the floor at its centre, facing along the
    wall — that is where a route is authored through it.

    Corners get a `thickness` square column so two runs meet square, and
    `detail="full"` adds a skirting to each face and a lining round each
    opening as decoration (drawn, never collided). Pins one part
    (`structure.wall`) carrying the run's length, height and thickness.

        bt.parts.wall(scene, "corridor/north", path=[(0, 2.4), (18, 2.4)],
                      height=2.7, openings=[(0, 6.0, 0.9), (0, 11.0, 0.9)])
    """
    pts = [(float(p[0]), float(p[1])) for p in path]
    if len(pts) < 2:
        raise ValueError("wall: path needs at least two corners")
    if height <= 0 or thickness <= 0:
        raise ValueError("wall: height and thickness must be positive")

    mode = _detail(detail, False)
    trim = trim_color if trim_color is not None else color
    edges = list(zip(pts, pts[1:] + ([pts[0]] if closed and len(pts) > 2 else [])))
    built = Built(name)
    run = 0.0

    # A column at every corner two runs share. Without it the two boxes
    # meet on a mitre and leave a notch you can see through — and, at an
    # acute corner, one a foot could fall into.
    corners = list(range(len(pts))) if closed and len(pts) > 2 else list(range(1, len(pts) - 1))
    for i in corners:
        x, y = pts[i]
        built.obstacles.append(
            scene.add_box(f"{name}/corner{i}", size=(thickness, thickness, height),
                          position=(x, y, base_z + height / 2.0), color=color)
        )

    for e, ((x0, y0), (x1, y1)) in enumerate(edges):
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-9:
            continue
        run += length
        yaw = math.atan2(y1 - y0, x1 - x0)
        q = _yaw_quat(yaw)
        ux, uy = (x1 - x0) / length, (y1 - y0) / length

        def at(along: float, across: float = 0.0, z: float = 0.0) -> Point3:
            return (x0 + ux * along - uy * across, y0 + uy * along + ux * across, z)

        holes = _wall_openings(e, openings, length, head, height)
        # The solid piers: what is left of the run once the openings are
        # taken out of it.
        piers = []
        cut = 0.0
        for start, end, _clear in holes:
            if start - cut > 1e-6:
                piers.append((cut, start))
            cut = end
        if length - cut > 1e-6:
            piers.append((cut, length))
        for i, (a, b) in enumerate(piers):
            built.obstacles.append(
                scene.add_box(f"{name}/e{e}_{i}", size=(b - a, thickness, height),
                              position=at((a + b) / 2.0, 0.0, base_z + height / 2.0),
                              quaternion=q, color=color)
            )
            if mode == "full":
                for side, across in (("i", (thickness + 0.012) / 2.0),
                                     ("o", -(thickness + 0.012) / 2.0)):
                    _trim(scene, built, f"{name}/trim/skirt_e{e}_{i}{side}",
                          (b - a, 0.012, 0.06), at((a + b) / 2.0, across, base_z + 0.03),
                          q, DARK_STEEL)
        for i, (start, end, clear) in enumerate(holes):
            centre = (start + end) / 2.0
            if height - clear > 1e-6:
                built.obstacles.append(
                    scene.add_box(f"{name}/head/e{e}_{i}", size=(end - start, thickness, height - clear),
                                  position=at(centre, 0.0, base_z + (height + clear) / 2.0),
                                  quaternion=q, color=color)
                )
            frame = f"{name}/opening{e}_{i}"
            scene.add_frame(frame, position=at(centre, 0.0, base_z), quaternion=q)
            built.frames.append(frame)
            if mode == "full":
                lining = thickness + 0.05
                for side, along in (("a", start), ("b", end)):
                    _trim(scene, built, f"{name}/trim/jamb_e{e}_{i}{side}",
                          (0.03, lining, clear), at(along, 0.0, base_z + clear / 2.0), q, trim)
                _trim(scene, built, f"{name}/trim/head_e{e}_{i}",
                      (end - start, lining, 0.03), at(centre, 0.0, base_z + clear),
                      q, trim)

    scene.set_part(
        name, kind="group", category="structure.wall", qty=1,
        **_identity(model, manufacturer, {
            "length_mm": str(round(run * 1000)),
            "height_mm": str(round(height * 1000)),
            "thickness_mm": str(round(thickness * 1000)),
            **attributes,
        }),
    )
    return built


# ------------------------------------------------------------- machine tool

# The two-tone a compact machining centre is painted: light enclosure,
# dark window glass, the maker's accent band, cast-iron bed and table.
MACHINE_SHELL: Color = (0.62, 0.63, 0.62)
MACHINE_WINDOW: Color = (0.05, 0.06, 0.07)
MACHINE_ACCENT: Color = (0.80, 0.55, 0.03)
MACHINE_BED: Color = (0.24, 0.25, 0.27)
TABLE_STEEL: Color = (0.46, 0.47, 0.49)

# 22 mm pushbutton caps in the IEC 60073 colours (linear RGB), and what a
# machine-tool panel's buttons are called.
BUTTON_COLORS: dict[str, Color] = {
    "green": (0.02, 0.35, 0.06),
    "red": (0.55, 0.02, 0.02),
    "yellow": (0.75, 0.55, 0.02),
    "blue": (0.02, 0.10, 0.45),
    "white": (0.80, 0.80, 0.78),
    "black": (0.02, 0.02, 0.02),
}
BUTTON_BY_NAME = {
    "cycle_start": "green", "start": "green",
    "feed_hold": "red", "stop": "red", "estop": "red",
    "reset": "blue", "clamp": "yellow", "unclamp": "yellow", "door": "white",
}
# The ISO 22 mm pushbutton: cap over the bezel, operating travel, actuating
# force — and the ø40 mushroom head of an emergency stop.
BUTTON_CAP = 0.0285
BUTTON_TRAVEL = 0.0026
BUTTON_FORCE_N = 3.8
ESTOP_CAP = 0.040
ESTOP_FORCE_N = 44.0

# A CNC lathe as the envelopes a tending cell verifies against: the Haas
# ST-10 of the public spec pages is the default (machinetoolindex.com and
# dealers' listings, 2026-09: 6.5 in / 165 mm chuck, 44 mm bar, 419 mm
# swing, 200 x 406 mm travels, 3585 kg; overall 126 x 70 x 81 in — 3.20 x
# 1.78 x 2.06 m). What no public page prints — the front opening, the
# spindle's height and its depth behind the door — are design values
# (`LATHE_APERTURE`, `LATHE_SPINDLE`), the first to replace from a drawing.
LATHE_SIZE = (3.20, 1.78, 2.06)
LATHE_APERTURE = (0.90, 0.70, 0.80)     # the front door opening: width, height, sill
LATHE_SPINDLE = (-0.55, 0.50, 1.05)     # chuck face: x from the body centre, depth behind the front wall, height
LATHE_CHAMBER = 1.00                    # the work area's depth
LATHE_CHUCK = 0.165                     # 6.5 in
LATHE_TURRET = (0.40, 0.40, 0.45)       # the turret's envelope, right of the chuck

# The α-D21MiB5 Plus (FANUC ROBODRILL) figures the generator defaults to,
# transcribed from the public catalogue (design-machine-tending.md §3.2):
# body, the side auto-door opening of the X500 machine with its sill and
# stroke, table, the spindle nose to table at Z max, and the wide front
# door. The table height above the floor is not published — 0.90 m is an
# assumption, and the one figure to replace when the maker's drawing is
# at hand.
VMC_SIZE: Point3 = (1.615, 2.108, 2.137)
VMC_APERTURE: Point3 = (0.705, 0.869, 0.827)
VMC_FRONT_DOOR: Point3 = (0.730, 0.869, 0.827)
VMC_TABLE: Point3 = (0.650, 0.400, 0.900)
VMC_EXCHANGE: Point2 = (0.250, 0.0)
VMC_HEAD_CLEARANCE = 0.580
VMC_CHAMBER = 1.30
# A servo door runs an 800 mm stroke in 0.8 s where an air cylinder takes
# 2 s (FANUC's published comparison) — the speeds a drive defaults to.
DOOR_SPEED = {"servo": 1.0, "air": 0.4}
DOOR_DRIVES = ("manual", "air", "servo")
# "Not given": the generator's (or the pack's) own figure applies.
_DEFAULT = object()


def _rotate(q, v):
    """`v` turned by the quaternion `q` (x, y, z, w)."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # t = 2 q × v ; v' = v + w t + q × t
    tx, ty, tz = 2 * (qy * vz - qz * vy), 2 * (qz * vx - qx * vz), 2 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


@dataclass
class MachineTool(Built):
    """What `machine_tool` built, plus the names a tending program
    addresses: the side door's axis (`door`, `None` for a manual door or
    none), what rides on that door (`door_objects` — the leaf and its
    trim, for a robot that slides it by hand), its end-of-travel lanes
    (`door_lanes` = closed, open — the axis's stop lanes, or two zone
    sensors on a loose leaf), the stroke and the world direction it opens
    along (`door_travel`, `door_axis`), the front door's closed switch and
    the E-stop lane the machine's program is guarded by (`front_door_lane`,
    `estop`), the operator panel's `Built` and the button sensors on it."""

    door: Optional[str] = None
    door_travel: float = 0.0
    door_axis: Point3 = (0.0, 1.0, 0.0)
    door_objects: list[str] = field(default_factory=list)
    door_lanes: Optional[tuple[str, str]] = None
    #: The front door's closed switch (`<name>/front_door/closed`), `None`
    #: without a front door — the lane the side door's opening is guarded
    #: by (the two doors are never open together).
    front_door_lane: Optional[str] = None
    #: The panel's E-stop lane (`<name>/panel/estop`), `None` without one —
    #: what every start is guarded by.
    estop: Optional[str] = None
    panel: Optional[Built] = None
    buttons: list[str] = field(default_factory=list)
    #: The control interface the catalog pack states (`template` and a
    #: signal table) — what `bt.tending` checks its template against.
    interface: Optional[dict] = None


def operator_panel(
    scene,
    name: str,
    position: Point3,
    *,
    yaw: float = 0.0,
    tilt: float = 0.0,
    size: Point2 = (0.30, 0.22),
    thickness: float = 0.03,
    buttons: Sequence[str] = ("cycle_start", "feed_hold", "reset", "estop"),
    columns: Optional[int] = None,
    pitch: float = 0.045,
    cap: float = BUTTON_CAP,
    travel: float = BUTTON_TRAVEL,
    proud: float = 0.010,
    watch_robots: Optional[list[str]] = None,
    catalog: Optional["CatalogRef"] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    button_model: Union[str, Mapping[str, str], None] = None,
    color: Color = DARK_STEEL,
    **attributes,
) -> Built:
    """An operator panel: a plate `size = (width, height)` centred at
    `position`, its face toward -Y before `yaw`, tilted up toward the
    operator by `tilt`, with a grid of 22 mm pushbuttons on it.

    A button is three things. A cap (decoration — drawn, never collided),
    a **zone sensor** `<name>/<button>` the size of the cap and as deep as
    the button's operating travel, sitting *inside* the cap face — so a
    tool that touches the cap reads nothing and one that pushes it in the
    2.6 mm a 22 mm actuator travels turns the input on, for as long as it
    is held — and two frames: `<name>/<button>` on the cap face and
    `<name>/<button>/press` the travel below it, both with +Z pointing
    into the panel, which is where a pressing tool aims its approach axis.
    Nothing moves: the stroke is a depth, and the input is the meaning.
    A neighbouring button's zone is the check that a wide tool did not
    press two.

    Cap colours follow the name (`cycle_start` green, `feed_hold` red,
    `reset` blue, …) and `estop` is drawn as the ø40 mushroom head with
    its collar; each button's sensor is pinned as an `hmi.button` with the
    head size, travel and actuating force, the panel itself as an
    `hmi.panel`. By default any robot link trips a button;
    `watch_robots=[...]` narrows it to the arms named.

    With `catalog=` — the id of a pushbutton-box spec pack, or a package
    directory — a box you can order: the number of buttons is matched
    against the sizes sold (the box's face follows), the pitch, the cap
    and the travel come from the pack, and the box, its buttons and the
    E-stop land on the bill with their article numbers."""
    names = [str(b) for b in buttons]
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("operator_panel")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if "positions" in params:
            params["positions"] = spec.choose("positions", len(names))
            faces = spec.rule("size_mm_by_positions") or {}
            face = faces.get(str(params["positions"])) or faces.get(params["positions"])
            if face:
                size = (float(face[0]) / 1000.0, float(face[1]) / 1000.0)
        thickness = _mm(spec.dimension_mm("box", "thickness", thickness * 1000.0)) or thickness
        pitch = _mm(spec.dimension_mm("box", "pitch", pitch * 1000.0)) or pitch
        proud = _mm(spec.dimension_mm("box", "proud", proud * 1000.0)) or proud
        cap = _mm(spec.dimension_mm("button", "cap", cap * 1000.0)) or cap
        travel = _mm(spec.dimension_mm("button", "travel", travel * 1000.0)) or travel
        manufacturer = manufacturer or spec.manufacturer
    if len(set(names)) != len(names):
        raise ValueError("operator_panel: button names must be distinct")
    if not names:
        raise ValueError("operator_panel: at least one button")
    w, h = float(size[0]), float(size[1])
    if min(w, h, thickness, pitch, cap, travel, proud) <= 0:
        raise ValueError("operator_panel: sizes must be positive")
    cols = int(columns) if columns is not None else min(len(names), 4)
    rows_ = -(-len(names) // cols)
    if (cols - 1) * pitch + cap > w + 1e-9 or (rows_ - 1) * pitch + cap > h + 1e-9:
        raise ValueError(
            f"operator_panel: {len(names)} buttons at {pitch * 1e3:.0f} mm pitch need a face "
            f"{((cols - 1) * pitch + cap) * 1e3:.0f} x {((rows_ - 1) * pitch + cap) * 1e3:.0f} mm; "
            f"the panel is {w * 1e3:.0f} x {h * 1e3:.0f}"
        )

    x, y, z = (float(v) for v in position)
    # Local: the face is the -Y side of the plate; tilt turns it upward
    # about the plate's own X (`_mul_quat(outer, inner)` — the inner
    # rotation is about the outer's rotated axes).
    q = _mul_quat(_yaw_quat(yaw), _pitch_quat(-float(tilt)))
    q_press = _mul_quat(q, _pitch_quat(-math.pi / 2))   # +Z into the panel
    q_cap = _mul_quat(q, _pitch_quat(math.pi / 2))      # a cylinder's +Z out of it

    def world(lx: float, ly: float, lz: float) -> Point3:
        dx, dy, dz = _rotate(q, (lx, ly, lz))
        return (x + dx, y + dy, z + dz)

    built = Built(name)
    built.obstacles.append(
        scene.add_box(f"{name}/plate", size=(w, thickness, h), position=(x, y, z), quaternion=q, color=color)
    )
    face = -thickness / 2
    scene.add_frame(name, position=world(0.0, face, 0.0), quaternion=q_press)
    built.frames.append(name)

    if watch_robots is None:
        watch: dict = {"watch": [], "watch_robot": True}
    else:
        watch = {"watch": [], "watch_robots": list(watch_robots)}
    models = (
        {b: button_model for b in names} if isinstance(button_model, str)
        else dict(button_model or {})
    )
    if spec is not None:
        # The pack's articles: one for the buttons of the box, one for the
        # E-stop — so identical buttons merge into a line with a count.
        for button in names:
            role = "estop" if button == "estop" else "button"
            if spec.has_component(role):
                models.setdefault(button, spec.part_number(role, **params))
    for i, button in enumerate(names):
        col, row = i % cols, i // cols
        u = (col - (cols - 1) / 2) * pitch
        v = ((rows_ - 1) / 2 - row) * pitch
        estop = button == "estop"
        radius = (ESTOP_CAP if estop else cap) / 2
        height = proud * 2.5 if estop else proud
        colour = BUTTON_COLORS[BUTTON_BY_NAME.get(button, "black")]
        if estop:
            _trim_cylinder(scene, built, f"{name}/{button}/collar", 0.030, 0.003,
                           world(u, face - 0.0015, v), q_cap, BUTTON_COLORS["yellow"])
        _trim_cylinder(scene, built, f"{name}/{button}/cap", radius, height,
                       world(u, face - height / 2, v), q_cap, colour)
        # The zone sits behind the cap face, as deep as the stroke.
        zone = f"{name}/{button}"
        scene.add_zone_sensor(
            zone, position=world(u, face - height + travel / 2, v),
            size=(2 * radius, travel, 2 * radius), quaternion=q, **watch,
        )
        built.sensors.append(zone)
        scene.add_frame(zone, position=world(u, face - height, v), quaternion=q_press)
        scene.add_frame(f"{zone}/press", position=world(u, face - height + travel, v), quaternion=q_press)
        built.frames.extend([zone, f"{zone}/press"])
        figures = {
            "head_mm": 40.0 if estop else 22.0,
            "cap_mm": round(2 * radius * 1e3, 1),
            "travel_mm": round(travel * 1e3, 2),
            "force_n": ESTOP_FORCE_N if estop else BUTTON_FORCE_N,
            "actuator": "mushroom" if estop else "flush",
            "color": BUTTON_BY_NAME.get(button, "black"),
        }
        role = "estop" if estop else "button"
        if spec is not None and spec.has_component(role):
            scene.set_part(
                zone, kind="sensor", category=spec.category(role, "hmi.button"), qty=1,
                catalog=spec.catalog_ref, manufacturer=manufacturer, model=models.get(button),
                **{**figures, **_kg(spec.mass_kg(role, positions=1))},
            )
        else:
            scene.set_part(
                zone, kind="sensor", category="hmi.button",
                **_identity(models.get(button), manufacturer if models.get(button) else None, figures),
            )
    if spec is None or not spec.has_component("box"):
        scene.set_part(
            name, kind="group", category="hmi.panel",
            **_identity(model, manufacturer, {"buttons": float(len(names)), **attributes}),
        )
        return built
    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    scene.set_part(
        name, kind="group", category=spec.category("box", "hmi.panel"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("box", **params), description=spec.name,
        **{**recorded, "buttons": float(len(names)), **_kg(spec.mass_kg("box", **params)), **attributes},
    )
    return built


def vise(
    scene,
    name: str,
    position: Point2 | Point3,
    *,
    yaw: float = 0.0,
    jaw_width: float = 0.125,
    opening: float = 0.060,
    max_opening: float = 0.150,
    jaw_height: float = 0.040,
    jaw_thickness: float = 0.030,
    body_height: float = 0.060,
    body_length: float = 0.360,
    catalog: Optional["CatalogRef"] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = TABLE_STEEL,
    **attributes,
) -> Built:
    """A machine vise standing on a table top at `position` (x, y[, the
    table's top z]): the body, a fixed jaw and a moving jaw set `opening`
    apart, the jaws clamping along local Y (the fixed jaw on +Y, the
    screw end trailing off to -Y) before `yaw`. Adds the frame
    `<name>/jaw` at the centre of the jaw floor between the jaws — where
    the workpiece sits, `jaw_width` wide along X and `opening` across — and
    pins the vise (`fixture.vise`).

    Clamping is a signal, not a motion: the jaws stand where the part
    goes and the cell's program says when it is held (a machine-tending
    handshake's `clamp` — see `bt.tending`). An `opening` beyond
    `max_opening` is refused with the numbers, the way a size nobody
    sells is.

    With `catalog=` — the id of a vise spec pack, or a package directory —
    a vise you can order: `jaw_width` is matched against the ones sold,
    the jaw and body figures and the maximum opening come from the pack,
    and the BOM row carries its article number and mass."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("vise")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if "jaw_width_mm" in params:
            params["jaw_width_mm"] = spec.choose("jaw_width_mm", round(jaw_width * 1000.0, 3))
            jaw_width = params["jaw_width_mm"] / 1000.0
            limits = spec.rule("max_opening_mm_by_jaw") or {}
            limit = limits.get(str(_plain(params["jaw_width_mm"]))) or limits.get(params["jaw_width_mm"])
            if limit is not None:
                max_opening = float(limit) / 1000.0
        jaw_height = _mm(spec.dimension_mm("vise", "jaw_height", jaw_height * 1000.0)) or jaw_height
        jaw_thickness = _mm(spec.dimension_mm("vise", "jaw_thickness", jaw_thickness * 1000.0)) or jaw_thickness
        body_height = _mm(spec.dimension_mm("vise", "body_height", body_height * 1000.0)) or body_height
        body_length = _mm(spec.dimension_mm("vise", "body_length", body_length * 1000.0)) or body_length
        manufacturer = manufacturer or spec.manufacturer
    if not 0 < opening <= max_opening + 1e-9:
        raise ValueError(
            f"vise: a {jaw_width * 1e3:.0f} mm vise opens {max_opening * 1e3:.0f} mm at most, "
            f"not {opening * 1e3:.0f}"
        )
    if min(jaw_width, jaw_height, jaw_thickness, body_height, body_length) <= 0:
        raise ValueError("vise: sizes must be positive")
    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)

    def world(dx: float, dy: float) -> tuple[float, float]:
        return x + c * dx - s * dy, y + s * dx + c * dy

    built = Built(name)
    back = opening / 2 + jaw_thickness + 0.02        # the body's +Y end
    bx, by = world(0.0, back - body_length / 2)
    built.obstacles.append(
        scene.add_box(f"{name}/body", size=(jaw_width + 0.02, body_length, body_height),
                      position=(bx, by, z0 + body_height / 2), quaternion=q, color=color)
    )
    for tag, sign in (("fixed", 1.0), ("moving", -1.0)):
        jx, jy = world(0.0, sign * (opening / 2 + jaw_thickness / 2))
        built.obstacles.append(
            scene.add_box(f"{name}/jaw_{tag}", size=(jaw_width, jaw_thickness, jaw_height),
                          position=(jx, jy, z0 + body_height + jaw_height / 2), quaternion=q,
                          color=DARK_STEEL)
        )
    scene.add_frame(f"{name}/jaw", position=(x, y, z0 + body_height), quaternion=q)
    built.frames.append(f"{name}/jaw")
    figures = {
        "jaw_width_mm": round(jaw_width * 1e3, 1),
        "opening_mm": round(opening * 1e3, 1),
        "max_opening_mm": round(max_opening * 1e3, 1),
    }
    if spec is None:
        scene.set_part(
            name, kind="group", category="fixture.vise",
            **_identity(model, manufacturer, {**figures, **attributes}),
        )
        return built
    scene.set_part(
        name, kind="group", category=spec.category("vise", "fixture.vise"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("vise", **params), description=spec.name,
        **{**_recorded(spec, params), **figures, **_kg(spec.mass_kg("vise", **params)), **attributes},
    )
    return built



def chuck(
    scene,
    name: str,
    position: Point3,
    quaternion: Optional[tuple[float, float, float, float]] = None,
    *,
    diameter: float = LATHE_CHUCK,
    length: float = 0.085,
    jaws: int = 3,
    jaw_height: float = 0.030,
    jaw_width: float = 0.025,
    opening: float = 0.050,
    max_opening: Optional[float] = None,
    catalog: Optional["CatalogRef"] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = DARK_STEEL,
    **attributes,
) -> Built:
    """A lathe chuck: a `diameter` body `length` long with its face at
    `position`, its axis the +Z of `quaternion` (pass a lathe's
    `<name>/spindle` frame — `bt.parts.chuck(scene, "chuck",
    *scene.frame("lathe/spindle"))`), and `jaws` jaw blocks standing
    `jaw_height` off the face around a part of `opening` diameter — the
    gripping diameter, so a robot loading a part along the axis meets
    the jaws where they are. Frame `<name>/face`: the face centre, +Z out
    along the spindle axis (a load comes in along -Z). Part: `fixture.chuck`
    with the diameter, the opening and the jaw count; with `catalog=` the
    diameter is matched against the ones sold and the maximum opening
    comes from the pack."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("chuck")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if "diameter_mm" in params:
            params["diameter_mm"] = spec.choose("diameter_mm", round(diameter * 1000.0, 3))
            diameter = float(params["diameter_mm"]) / 1000.0
        by_diameter = spec.rule("max_opening_mm_by_diameter") or {}
        if max_opening is None and by_diameter:
            key = str(_plain(params.get("diameter_mm", round(diameter * 1000.0, 3))))
            if key in by_diameter:
                max_opening = float(by_diameter[key]) / 1000.0
        manufacturer = manufacturer or spec.manufacturer
    if min(diameter, length, jaw_height, jaw_width, opening) <= 0 or jaws < 2:
        raise ValueError("chuck: sizes must be positive and jaws at least 2")
    if opening + 2 * jaw_width > diameter + 1e-9:
        raise ValueError(
            f"chuck: a {opening * 1e3:.0f} mm opening with {jaw_width * 1e3:.0f} mm jaws does not fit a "
            f"{diameter * 1e3:.0f} mm chuck"
        )
    if max_opening is not None and opening > max_opening + 1e-9:
        raise ValueError(f"chuck: {opening * 1e3:.0f} mm is past the {max_opening * 1e3:.0f} mm maximum opening")
    q = quaternion or (0.0, 0.0, 0.0, 1.0)
    px, py, pz = (float(v) for v in position)
    axis = _rotate(q, (0.0, 0.0, 1.0))
    built = Built(name)
    # The body, its face at `position`, behind it along the axis.
    bx, by, bz = (px - axis[i] * length / 2 for i in range(3))
    body = scene.add_cylinder(f"{name}/body", radius=diameter / 2, length=length, position=(bx, by, bz),
                              quaternion=q, color=color)
    built.obstacles.append(body)
    # The jaws around the opening, `jaw_height` proud of the face.
    r = opening / 2 + jaw_width / 2
    for k in range(jaws):
        theta = math.pi / 2 + 2 * math.pi * k / jaws
        local = (r * math.cos(theta), r * math.sin(theta), jaw_height / 2)
        wx, wy, wz = _rotate(q, local)
        spin = (0.0, 0.0, math.sin(theta / 2), math.cos(theta / 2))
        jaw = scene.add_box(f"{name}/jaw{k}", size=(jaw_width, jaw_width, jaw_height),
                            position=(px + wx, py + wy, pz + wz), quaternion=_mul_quat(q, spin), color=color)
        built.obstacles.append(jaw)
    scene.add_frame(f"{name}/face", position=(px, py, pz), quaternion=q)
    built.frames.append(f"{name}/face")
    figures = {"diameter_mm": round(diameter * 1000.0, 1), "opening_mm": round(opening * 1000.0, 1),
               "jaws": float(jaws)}
    if max_opening is not None:
        figures["max_opening_mm"] = round(max_opening * 1000.0, 1)
    if spec is None:
        scene.set_part(name, kind="group", category="fixture.chuck", qty=1,
                       **_identity(model, manufacturer, {**figures, **attributes}))
        return built
    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    scene.set_part(
        name, kind="group", category=spec.category("body", "fixture.chuck"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("body", **params), description=spec.name,
        **{**recorded, **figures, **_kg(spec.mass_kg("body", **params)), **attributes},
    )
    return built

def machine_tool(
    scene,
    name: str,
    size: Optional[Point3] = None,
    position: Point2 | Point3 = (0.0, 0.0),
    *,
    yaw: float = 0.0,
    aperture: Optional[Point3] = None,
    door: Union[str, None, object] = _DEFAULT,
    door_side: Union[str, object] = _DEFAULT,
    door_travel: Optional[float] = None,
    door_speed: Optional[float] = None,
    front_door: Union[Point3, None, object] = _DEFAULT,
    chamber: Optional[float] = None,
    table: Optional[Point3] = None,
    exchange: Optional[Point2] = None,
    head_clearance: Optional[float] = None,
    panel: Optional[str] = "front",
    buttons: Optional[Sequence[str]] = None,
    panel_pitch: float = 0.045,
    wall: float = 0.06,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = MACHINE_SHELL,
    **attributes,
) -> MachineTool:
    """A vertical machining centre as the envelopes a tending cell
    verifies against — not its shape. `size = (width, depth, height)`
    stands at `position` (its centre, x, y[, floor z]), front face on -Y
    before `yaw`. Without arguments it is the FANUC ROBODRILL
    α-D21MiB5 Plus of the public catalogue (`VMC_*` above): change any
    figure and the envelopes, the frames and the BOM line change together.

    What it puts in the scene, all of it collision-checked:

    * the enclosure — bed, side walls, roof, the rear column block (the
      last `depth - chamber` of the body), and a front wall around the
      front door opening (`front_door = (width, height, sill)`), with its
      leaf standing closed;
    * the **table** `table = (width, depth, top height)` at the exchange
      position (`exchange = (x, y)` offset from the chamber centre, x
      toward the door side — a table that traverses to the door is what
      a tending robot reaches), and the spindle head above it from
      `head_clearance` (nose to table at Z max) to the roof;
    * the **side door**: an opening `aperture = (width, height, sill)` in
      the `door_side` wall and a leaf that slides toward the rear by
      `door_travel`. `door="servo"` / `"air"` make it a linear axis
      `<name>/side_door` with the stops `closed` and `open`
      (`bt.seq.move_to(door, "open")` opens it, `move_to(door, "closed")`
      closes; the speed comes from the drive — `door_speed` overrides),
      and the rollout checks the leaf against every robot each tick: a
      door closing on an arm is a `DeviceCollision` by name.
      `door="manual"` leaves the leaf loose, for a robot that takes the
      handle (`bt.seq.attach` the `door_objects` and run a
      `cartesian_line`); `door=None` builds a plain wall. Either way the
      lanes `<name>/side_door/closed` and `/open` read the leaf at its
      ends of travel — the axis's stop lanes, or two zone sensors on a
      loose leaf — the limit switches a door interlock is written from;
    * an **operator panel** (`operator_panel`) with `buttons` at
      `panel_pitch`, on the front face (`panel="front"`) or on the
      door-side wall ahead of the opening (`panel="door"`, where a robot
      at the door reaches it); `panel=None` leaves it off.

    Frames: `<name>/table` (centre of the table top), `<name>/entry` (the
    side opening's centre, 150 mm outside the door leaf — where a robot
    waits), `<name>/door/side/handle` (the leaf's handle, +Z into the
    leaf), and the panel's `<name>/panel/<button>[/press]`.

    Refused rather than clipped, like a wall plan that does not close: an
    opening that does not fit its wall, a leaf whose stroke runs off the
    body, a spindle head that would stand through the roof.

    `detail="full"` adds the windows, the door rails, the accent band and
    the stack light — drawn, never collided. The part is pinned on the
    group (`machine_tool.vmc`), the side door as `<name>/side_door`
    (`machine_tool.door`, with its drive and stroke), the panel and its
    buttons by `operator_panel`.

    With `catalog=` — the id of a machine-tool spec pack, or a package
    directory — a machine you can order: the body, the openings, the
    table and the head come from the pack's `mechanical.envelope`, the
    options it sells (`column_mm`, `side_door`, `door_side`) are chosen
    by name and refused when nobody sells them, the door's speed follows
    the drive's published time, every article lands on the bill with its
    number, and the pack's `interface` (its handshake template and
    signal table) rides on the returned `MachineTool` for `bt.tending`."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("machine_tool")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        # The door and its side are options the pack sells: the caller's
        # choice is matched against them, the pack's default stands otherwise.
        if "side_door" in params:
            sold = [str(_plain(v)) for v in (spec.params()["side_door"].get("values") or [])]
            if door is None:
                # No side door at all — outside what the pack sells, so
                # the option is simply not ordered.
                params.pop("side_door")
            elif door == "manual" and "manual" not in sold:
                # The leaf without a drive: the pack's opening, sill and
                # stroke stand, and no door drive is ordered — a robot
                # that takes the handle is the drive.
                params.pop("side_door")
            else:
                if door is not _DEFAULT:
                    params["side_door"] = spec.choose("side_door", str(door))
                chosen = params["side_door"]
                door = None if chosen in (None, "none") else str(chosen)
        if "door_side" in params:
            if door_side is not _DEFAULT:
                params["door_side"] = spec.choose("door_side", door_side)
            door_side = str(params["door_side"])
        column = float(params.get("column_mm") or 0.0) / 1000.0
        mech = spec.mechanical
        if size is None and mech.get("footprint_mm") and mech.get("height_mm"):
            fp = mech["footprint_mm"]
            size = (float(fp[0]) / 1000.0, float(fp[1]) / 1000.0, float(mech["height_mm"]) / 1000.0 + column)
        side = spec.envelope("doors", "side")
        if aperture is None and isinstance(side, dict):
            aperture = (side["width_mm"] / 1000.0, side["height_mm"] / 1000.0, side["sill_mm"] / 1000.0)
            if door_travel is None and side.get("travel_mm") is not None:
                door_travel = float(side["travel_mm"]) / 1000.0
        front = spec.envelope("doors", "front")
        if front_door is _DEFAULT and isinstance(front, dict):
            front_door = (front["width_mm"] / 1000.0, front["height_mm"] / 1000.0, front["sill_mm"] / 1000.0)
        table_env = spec.envelope("table")
        if table is None and isinstance(table_env, dict) and table_env.get("size_mm"):
            tw_mm, td_mm = table_env["size_mm"]
            table = (float(tw_mm) / 1000.0, float(td_mm) / 1000.0,
                     float(table_env.get("height_mm") or VMC_TABLE[2] * 1000.0) / 1000.0)
        nose = spec.envelope("head", "nose_to_table_mm")
        if head_clearance is None and isinstance(nose, (list, tuple)) and len(nose) == 2:
            head_clearance = float(nose[1]) / 1000.0
        if chamber is None and spec.envelope("chamber_mm") is not None:
            chamber = float(spec.envelope("chamber_mm")) / 1000.0
        if door_speed is None and door in ("air", "servo"):
            open_s = spec.behavior(f"door_open_s_{door}")
            if isinstance(open_s, (int, float)) and open_s > 0 and door_travel is not None:
                door_speed = float(door_travel) / float(open_s)
        manufacturer = manufacturer or spec.manufacturer
    if front_door is _DEFAULT:
        front_door = VMC_FRONT_DOOR
    if door is _DEFAULT:
        door = "servo"
    if door_side is _DEFAULT:
        door_side = "right"
    if door is not None and door not in DOOR_DRIVES:
        raise ValueError(f"machine_tool: door must be one of {DOOR_DRIVES} or None, not {door!r}")
    if door_side not in ("left", "right"):
        raise ValueError(f"machine_tool: door_side is 'left' or 'right', not {door_side!r}")
    if panel not in (None, "front", "door"):
        raise ValueError(f"machine_tool: panel is 'front', 'door' or None, not {panel!r}")
    mode = _detail(detail, spec is not None)
    w, d, h = (float(v) for v in (size or VMC_SIZE))
    aw, ah, sill = (float(v) for v in (aperture or VMC_APERTURE))
    tw, td, th = (float(v) for v in (table or VMC_TABLE))
    ex, ey = (float(v) for v in (exchange or VMC_EXCHANGE))
    hc = VMC_HEAD_CLEARANCE if head_clearance is None else float(head_clearance)
    cd = VMC_CHAMBER if chamber is None else float(chamber)
    t = float(wall)
    if door is not None and aw < min(tw, td) - 1e-9:
        raise ValueError(
            f"machine_tool: a {aw * 1e3:.0f} mm side opening is narrower than the table's "
            f"{min(tw, td) * 1e3:.0f} mm short side — nothing on the table passes it"
        )
    if min(w, d, h, aw, ah, sill, tw, td, th, hc, cd, t) <= 0:
        raise ValueError("machine_tool: sizes must be positive")
    if cd + 2 * t > d:
        raise ValueError(
            f"machine_tool: a {cd:.2f} m chamber does not fit a {d:.2f} m deep body"
        )
    travel = aw + 0.055 if door_travel is None else float(door_travel)
    speed = door_speed if door_speed is not None else DOOR_SPEED.get(door or "", 0.0)

    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    sx = 1.0 if door_side == "right" else -1.0
    y0 = -d / 2                       # the front face
    y_r = y0 + t + cd                 # where the rear column block begins
    ls = t + cd                       # the side walls' length
    yc_wall = (y0 + y_r) / 2
    y_c = y0 + t + cd / 2             # the chamber's centre
    inner = w / 2 - t
    leaf_t, over = 0.04, 0.05
    half = aw / 2 + over
    plate = 0.05

    # Refused before anything is placed: a plan that does not fit is not
    # a smaller machine.
    if door is not None:
        if sill + ah > h - t + 1e-9:
            raise ValueError(
                f"machine_tool: the side opening tops out at {(sill + ah) * 1e3:.0f} mm, "
                f"through a {h * 1e3:.0f} mm body"
            )
        if (y_c - aw / 2) - y0 < t or y_r - (y_c + aw / 2) < 0:
            raise ValueError(
                f"machine_tool: a {aw * 1e3:.0f} mm side opening does not fit the "
                f"{cd * 1e3:.0f} mm chamber"
            )
        if y_c + half + travel > d / 2 + 1e-9:
            raise ValueError(
                f"machine_tool: a {travel * 1e3:.0f} mm door stroke runs the leaf off the "
                f"{d * 1e3:.0f} mm body"
            )
        if door != "manual" and speed <= 0:
            raise ValueError("machine_tool: door_speed must be positive")
    elif panel == "door":
        raise ValueError("machine_tool: panel='door' needs a side door to stand beside")
    if front_door is not None:
        fw, fh, fs = (float(v) for v in front_door)
        if fw > w - 2 * t + 1e-9 or fs + fh > h - t + 1e-9:
            raise ValueError(
                f"machine_tool: a {fw * 1e3:.0f} x {fh * 1e3:.0f} mm front opening on a "
                f"{fs * 1e3:.0f} mm sill does not fit a {w * 1e3:.0f} x {h * 1e3:.0f} mm front"
            )
    tx_, ty_ = sx * ex, y_c + ey
    if abs(tx_) + tw / 2 > inner + 1e-9 or ty_ - td / 2 < y0 + t - 1e-9 or ty_ + td / 2 > y_r + 1e-9:
        raise ValueError(
            f"machine_tool: the table ({tw * 1e3:.0f} x {td * 1e3:.0f} mm) at the exchange "
            f"offset ({ex * 1e3:.0f}, {ey * 1e3:.0f}) stands into the enclosure"
        )
    if th + hc > h - t - 1e-9:
        raise ValueError(
            f"machine_tool: the spindle head retracted to {(th + hc) * 1e3:.0f} mm stands "
            f"through a {h * 1e3:.0f} mm roof"
        )

    def world(dx: float, dy: float) -> tuple[float, float]:
        return x + c * dx - s * dy, y + s * dx + c * dy

    def box(tag: str, sz: Point3, at: Point3, colour: Color = color, group: str = "shell") -> str:
        px, py = world(at[0], at[1])
        made = scene.add_box(f"{name}/{group}/{tag}" if group else f"{name}/{tag}",
                             size=sz, position=(px, py, z0 + at[2]), quaternion=q, color=colour)
        built.obstacles.append(made)
        return made

    built = MachineTool(name)

    # ---- the enclosure --------------------------------------------------
    bed_h = min(0.60, th - 0.10)
    box("bed", (w - 2 * t, cd, bed_h), (0.0, y_c, bed_h / 2), MACHINE_BED, group="")
    box("column", (w, d - ls, h), (0.0, (y_r + d / 2) / 2, h / 2), color, group="")
    box("top", (w, ls, t), (0.0, yc_wall, h - t / 2))
    box("far", (t, ls, h), (-sx * (w / 2 - t / 2), yc_wall, h / 2))
    # The door-side wall, around the opening (or solid, without a door).
    if door is None:
        box("near", (t, ls, h), (sx * (w / 2 - t / 2), yc_wall, h / 2))
    else:
        front_pier = (y_c - aw / 2) - y0
        rear_pier = y_r - (y_c + aw / 2)
        xw = sx * (w / 2 - t / 2)
        box("near_sill", (t, ls, sill), (xw, yc_wall, sill / 2))
        box("near_head", (t, ls, h - sill - ah), (xw, yc_wall, (sill + ah + h) / 2))
        box("near_front", (t, front_pier, ah), (xw, y0 + front_pier / 2, sill + ah / 2))
        if rear_pier > 1e-9:
            box("near_rear", (t, rear_pier, ah), (xw, y_r - rear_pier / 2, sill + ah / 2))
    # The front wall, around the front door opening (or solid).
    if front_door is None:
        box("front", (w, t, h), (0.0, y0 + t / 2, h / 2))
    else:
        yf = y0 + t / 2
        box("front_sill", (w, t, fs), (0.0, yf, fs / 2))
        box("front_head", (w, t, h - fs - fh), (0.0, yf, (fs + fh + h) / 2))
        pier = (w - fw) / 2
        for tag, sign in (("front_left", -1.0), ("front_right", 1.0)):
            box(tag, (pier, t, fh), (sign * (w / 2 - pier / 2), yf, fs + fh / 2))
        front_leaf = box("leaf", (fw + 0.10, leaf_t, fh + 0.10), (0.0, y0 - 0.01 - leaf_t / 2, fs + fh / 2),
                         color, group="front_door")
        # The front door's closed switch: a sliver the shut leaf overlaps
        # by 5 mm at its left edge — the confirmation the machine's program
        # reads before it opens the side door (never both at once).
        lx, ly = world(-(fw + 0.10) / 2 - 0.005, y0 - 0.01 - leaf_t / 2)
        lane = f"{name}/front_door/closed"
        scene.add_zone_sensor(lane, position=(lx, ly, z0 + fs + 0.10), size=(0.02, leaf_t + 0.02, 0.20),
                              quaternion=q, watch=[front_leaf])
        built.sensors.append(lane)
        built.front_door_lane = lane
        scene.set_part(lane, kind="sensor", category="sensor.limit_switch", end="closed")
        if mode == "full":
            wx, wy = world(0.0, y0 - 0.01 - leaf_t - 0.0025)
            _trim(scene, built, f"{name}/front_door/window", (fw - 0.10, 0.005, fh - 0.30),
                  (wx, wy, z0 + fs + fh / 2 + 0.05), q, MACHINE_WINDOW)

    # ---- the table at the exchange position, and the head over it -------
    box("saddle", (min(tw * 0.7, 0.45), min(td * 1.4, 0.55), th - plate - bed_h),
        (tx_, ty_, (bed_h + th - plate) / 2), MACHINE_BED, group="")
    box("table", (tw, td, plate), (tx_, ty_, th - plate / 2), TABLE_STEEL, group="")
    box("head", (0.32, 0.42, h - t - th - hc), (0.0, y_c, (th + hc + h - t) / 2), MACHINE_BED, group="")
    tx, ty = world(tx_, ty_)
    scene.add_frame(f"{name}/table", position=(tx, ty, z0 + th), quaternion=q)
    built.frames.append(f"{name}/table")

    # ---- the side door ----------------------------------------------------
    if door is not None:
        xl = sx * (w / 2 + 0.01 + leaf_t / 2)
        zl = sill + ah / 2
        leaf = box("leaf", (leaf_t, 2 * half, ah + 2 * over), (xl, y_c, zl), color, group="side_door")
        built.door_objects.append(leaf)
        # The handle: a vertical bar standing 90 mm off the leaf's outer
        # face at its front edge, 200 mm above the sill — decoration a
        # gripper closes on (never collided), and far enough out for one
        # to close on it with its knuckles clear of the leaf. The frame
        # is the bar's centre, +Z into the leaf.
        xh = xl + sx * (leaf_t / 2 + 0.090)
        yh = y_c - aw / 2
        zh = sill + 0.20
        hx, hy = world(xh, yh)
        built.door_objects.append(
            _trim(scene, built, f"{name}/side_door/handle", (0.02, 0.02, 0.14), (hx, hy, z0 + zh), q, DARK_STEEL)
        )
        for i, dz in enumerate((-0.05, 0.05)):
            px, py = world(xl + sx * (leaf_t / 2 + 0.045), yh)
            built.door_objects.append(
                _trim(scene, built, f"{name}/side_door/stub{i}", (0.090, 0.016, 0.016),
                      (px, py, z0 + zh + dz), q, DARK_STEEL)
            )
        q_handle = _mul_quat(q, _slope_quat(-sx * math.pi / 2))
        scene.add_frame(f"{name}/door/side/handle", position=(hx, hy, z0 + zh), quaternion=q_handle)
        built.frames.append(f"{name}/door/side/handle")
        if mode == "full":
            wx, wy = world(xl + sx * (leaf_t / 2 + 0.0025), y_c)
            built.door_objects.append(
                _trim(scene, built, f"{name}/side_door/window", (0.005, aw - 0.10, ah - 0.30),
                      (wx, wy, z0 + zl + 0.05), q, MACHINE_WINDOW)
            )
            for tag, dz in (("rail_top", ah / 2 + over + 0.02), ("rail_bottom", -(ah / 2 + over + 0.02))):
                rx, ry = world(sx * (w / 2 + 0.01 + leaf_t / 2), y_c + travel / 2)
                _trim(scene, built, f"{name}/side_door/{tag}", (0.015, 2 * half + travel, 0.03),
                      (rx, ry, z0 + zl + dz), q, DARK_STEEL)
        built.door_travel = travel
        built.door_axis = (-s, c, 0.0)
        if door != "manual":
            # A driven door is an axis with two named stops: their lanes
            # `<axis>/closed` and `<axis>/open` are the limit switches an
            # interlock waits on, and what the axis drives is checked
            # against every robot each tick.
            axis = f"{name}/side_door"
            scene.add_linear_axis(axis, objects=list(built.door_objects), axis=(-s, c, 0.0),
                                  speed=float(speed), range=(0.0, travel), position=0.0,
                                  stops={"closed": 0.0, "open": travel})
            built.devices.append(axis)
            built.door = axis
            built.door_lanes = (f"{axis}/closed", f"{axis}/open")
        else:
            # A loose leaf has no axis to read: two zone sensors, a sliver
            # the leaf overlaps by 5 mm at each end of its stroke, are the
            # closed and open limit switches.
            lanes = []
            for tag, yz in (("closed", y_c - half - 0.005), ("open", y_c + travel + half + 0.005)):
                zx, zy = world(xl, yz)
                lane = f"{name}/side_door/{tag}"
                scene.add_zone_sensor(lane, position=(zx, zy, z0 + zl), size=(leaf_t + 0.02, 0.02, 0.20),
                                      quaternion=q, watch=[leaf])
                built.sensors.append(lane)
                lanes.append(lane)
                scene.set_part(lane, kind="sensor", category="sensor.limit_switch", end=tag)
            built.door_lanes = (lanes[0], lanes[1])
        ex_, ey_ = world(sx * (w / 2 + 0.01 + leaf_t + 0.15), y_c)
        scene.add_frame(f"{name}/entry", position=(ex_, ey_, z0 + zl), quaternion=q)
        built.frames.append(f"{name}/entry")

    # ---- the operator panel -------------------------------------------------
    if panel is not None:
        keys = tuple(buttons) if buttons is not None else ("cycle_start", "feed_hold", "reset", "estop")
        # A pack that names its panel's switches names the buttons too.
        button_models = {}
        if spec is not None:
            for button in keys:
                role = "estop" if button == "estop" else "button"
                if spec.has_component(role):
                    button_models[button] = spec.part_number(role, **params)
        if panel == "front":
            px, py = world(sx * (w / 2 - 0.35), y0 - 0.015 - 0.005)
            made = operator_panel(scene, f"{name}/panel", (px, py, z0 + 1.35), yaw=yaw, tilt=0.35,
                                  buttons=keys, pitch=panel_pitch, button_model=button_models or None,
                                  manufacturer=manufacturer if button_models else None)
        else:
            gap = (y_c - aw / 2) - y0
            width = min(0.24, gap - 0.04)
            px, py = world(sx * (w / 2 + 0.015 + 0.005), y0 + gap / 2)
            made = operator_panel(scene, f"{name}/panel", (px, py, z0 + 1.30), yaw=yaw + sx * math.pi / 2,
                                  size=(width, 0.22), buttons=keys, columns=2, pitch=panel_pitch,
                                  button_model=button_models or None,
                                  manufacturer=manufacturer if button_models else None)
        built.panel = made
        built.buttons = list(made.sensors)
        if f"{name}/panel/estop" in built.buttons:
            built.estop = f"{name}/panel/estop"
        built.obstacles.extend(made.obstacles)
        built.frames.extend(made.frames)
        built.sensors.extend(made.sensors)
        if spec is not None and button_models:
            # The switches are the machine's own articles: their rows
            # link to the machine's pack, like the panel's.
            for button in made.sensors:
                part = scene.part(button) or {}
                scene.set_part(
                    button, kind="sensor", category=part.get("category") or "hmi.button",
                    catalog=spec.catalog_ref, manufacturer=part.get("manufacturer"),
                    model=part.get("model"), qty=int(part.get("qty") or 1),
                    attributes=dict(part.get("attributes") or {}),
                )

    if mode == "full":
        bx, by = world(0.0, y0 - 0.005)
        _trim(scene, built, f"{name}/trim/band", (w, 0.01, 0.06), (bx, by, z0 + h - 0.10), q, MACHINE_ACCENT)
        mx, my = world(sx * (w / 2 - 0.15), y0 + 0.15)
        _trim_cylinder(scene, built, f"{name}/trim/mast", 0.012, 0.10, (mx, my, z0 + h + 0.05), q, DARK_STEEL)
        for i, (tag, colour) in enumerate((("green", (0.023, 0.332, 0.061)), ("amber", (0.686, 0.323, 0.005)),
                                           ("red", (0.578, 0.018, 0.012)))):
            _trim_cylinder(scene, built, f"{name}/trim/light_{tag}", 0.03, 0.06,
                           (mx, my, z0 + h + 0.13 + 0.06 * i), q, colour)

    # ---- the identity ---------------------------------------------------------
    figures = {
        "footprint_mm": f"{w * 1e3:.0f}x{d * 1e3:.0f}",
        "height_mm": round(h * 1e3, 1),
        "table_mm": f"{tw * 1e3:.0f}x{td * 1e3:.0f}",
    }
    door_figures = {
        "drive": door,
        "opening_mm": f"{aw * 1e3:.0f}x{ah * 1e3:.0f}",
        "stroke_mm": round(travel * 1e3, 1),
        **({"open_s": round(travel / speed, 3)} if built.door else {}),
    }
    if spec is None:
        scene.set_part(
            name, kind="group", category="machine_tool.vmc",
            **_identity(model, manufacturer, {**figures, **attributes}),
        )
        if door is not None:
            # The door is one article — the axis where it is driven (so
            # the device row *is* the door on the bill), the leaf group
            # when it is pulled by hand.
            scene.set_part(
                built.door or f"{name}/side_door",
                kind="device" if built.door else "group", category="machine_tool.door",
                **door_figures,
            )
        return built
    built.interface = spec.interface
    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    scene.set_part(
        name, kind="group", category=spec.category("body", "machine_tool.vmc"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("body", **params), description=spec.name,
        **{**recorded, **figures, **_kg(spec.mass_kg("body", **params)), **attributes},
    )
    if door is not None and spec.has_component("side_door"):
        if "side_door" in params:
            ordered = dict(model=spec.part_number("side_door", **params),
                           **_kg(spec.mass_kg("side_door", **params)))
        else:
            ordered = {}  # the leaf, loose: no drive on the bill
        scene.set_part(
            built.door or f"{name}/side_door",
            kind="device" if built.door else "group",
            category=spec.category("side_door", "machine_tool.door"), qty=1,
            catalog=spec.catalog_ref, manufacturer=manufacturer,
            **{**door_figures, **ordered},
        )
    if built.panel is not None and spec.has_component("panel"):
        scene.set_part(
            f"{name}/panel", kind="group", category=spec.category("panel", "hmi.panel"), qty=1,
            catalog=spec.catalog_ref, manufacturer=manufacturer,
            model=spec.part_number("panel", **params), buttons=float(len(built.buttons)),
        )
    return built


def lathe(
    scene,
    name: str,
    size: Optional[Point3] = None,
    position: Point2 | Point3 = (0.0, 0.0),
    *,
    yaw: float = 0.0,
    aperture: Optional[Point3] = None,
    door: Union[str, None, object] = _DEFAULT,
    door_travel: Optional[float] = None,
    door_speed: Optional[float] = None,
    spindle: Optional[Point3] = None,
    chamber: Optional[float] = None,
    turret: Optional[Point3] = None,
    tailstock: bool = False,
    panel: Optional[str] = "front",
    buttons: Optional[Sequence[str]] = None,
    panel_pitch: float = 0.045,
    wall: float = 0.06,
    catalog: Optional["CatalogRef"] = None,
    detail: Optional[str] = None,
    model: Optional[str] = None,
    manufacturer: Optional[str] = None,
    color: Color = MACHINE_SHELL,
    **attributes,
) -> MachineTool:
    """A CNC lathe as the envelopes a tending cell verifies against —
    the turning counterpart of `machine_tool`. `size = (length, depth,
    height)` stands at `position`, front face on -Y before `yaw`, the
    spindle axis along the length (+X toward the tailstock). Without
    arguments it is the Haas ST-10 of the public spec pages (`LATHE_*`
    above); the front opening, the spindle's height and depth are design
    values, the first to replace when the drawing is at hand.

    What it puts in the scene, all of it collision-checked:

    * the enclosure — bed, the rear block behind the `chamber`, roof, end
      walls, and the **front wall around the door opening**
      `aperture = (width, height, sill)`, centred on the spindle;
    * the **headstock** and the **spindle nose** at
      `spindle = (x from the body centre, depth behind the front wall's
      inner face, height)`, the **turret** envelope `turret = (x, y, z
      size)` right of it at spindle height, and a tailstock block at the
      far end when `tailstock=True` — the chuck is a part of its own
      (`bt.parts.chuck(scene, "chuck", *scene.frame("<name>/spindle"))`);
    * the **front door**: a leaf that slides toward the tailstock end by
      `door_travel`. `door="servo"` / `"air"` make it a linear axis
      `<name>/front_door` with the stops `closed` and `open`, checked
      against every robot each tick; `door="manual"` (the default) leaves
      the leaf loose for a robot that takes the handle; `door=None`
      builds a solid front. Either way the lanes `<name>/front_door/closed`
      and `/open` read the leaf at its ends of travel;
    * an **operator panel** on the front face, right of the opening
      (`panel="front"`; `None` leaves it off).

    Frames: `<name>/spindle` (the spindle nose centre, +Z out along the
    axis toward the tailstock — a load comes in along -Z),
    `<name>/entry` (the opening's centre, 150 mm outside the leaf),
    `<name>/door/front/handle` (+Z into the leaf), the panel's.

    The returned `MachineTool` names the door axis, its lanes, the stroke
    and its world direction, the panel and its buttons, the E-stop lane —
    what `bt.tending` and a teach read. A lathe has the one door, so its
    `front_door_lane` is `None` and no door-exclusivity guard applies.
    Refused rather than clipped: an opening that does not fit the front,
    a stroke that runs the leaf off the body, a spindle outside the
    chamber.

    With `catalog=` a lathe spec pack's `mechanical.envelope`
    (`doors.front`, `spindle`, `turret`, `chamber_depth_mm`) and the door
    drive it sells (`front_door`) stand in for the figures, its articles
    land on the bill, and its `interface` rides on the result."""
    spec = None
    params: dict = {}
    if catalog is not None:
        from ._spec import Spec

        spec = Spec.load(catalog)
        spec.expect_generator("lathe")
        params = {key: spec.default(key) for key in spec.params()}
        for key in [key for key in attributes if key in params]:
            params[key] = spec.choose(key, attributes.pop(key))
        if "front_door" in params:
            sold = [str(_plain(v)) for v in (spec.params()["front_door"].get("values") or [])]
            if door is None:
                params.pop("front_door")
            elif door == "manual" and "manual" not in sold:
                params.pop("front_door")
            else:
                if door is not _DEFAULT:
                    params["front_door"] = spec.choose("front_door", str(door))
                chosen = params["front_door"]
                door = None if chosen in (None, "none") else str(chosen)
        mech = spec.mechanical
        if size is None and mech.get("footprint_mm") and mech.get("height_mm"):
            fp = mech["footprint_mm"]
            size = (float(fp[0]) / 1000.0, float(fp[1]) / 1000.0, float(mech["height_mm"]) / 1000.0)
        front = spec.envelope("doors", "front")
        if aperture is None and isinstance(front, dict):
            aperture = (front["width_mm"] / 1000.0, front["height_mm"] / 1000.0, front["sill_mm"] / 1000.0)
        if door_travel is None and isinstance(front, dict) and front.get("travel_mm"):
            door_travel = float(front["travel_mm"]) / 1000.0
        sp = spec.envelope("spindle")
        if spindle is None and isinstance(sp, dict):
            spindle = (sp["x_mm"] / 1000.0, sp["depth_mm"] / 1000.0, sp["height_mm"] / 1000.0)
        tr = spec.envelope("turret")
        if turret is None and isinstance(tr, dict):
            turret = (tr["x_mm"] / 1000.0, tr["y_mm"] / 1000.0, tr["z_mm"] / 1000.0)
        if chamber is None and spec.envelope("chamber_depth_mm") is not None:
            chamber = float(spec.envelope("chamber_depth_mm")) / 1000.0
        if door not in (None, _DEFAULT, "manual") and door_speed is None:
            open_s = spec.behavior(f"door_open_s_{door}")
            if isinstance(open_s, (int, float)) and open_s > 0 and door_travel is not None:
                door_speed = float(door_travel) / float(open_s)
        manufacturer = manufacturer or spec.manufacturer
    if door is _DEFAULT:
        door = "manual"
    if door is not None and door not in DOOR_DRIVES:
        raise ValueError(f"lathe: door must be one of {DOOR_DRIVES} or None, not {door!r}")
    if panel not in (None, "front"):
        raise ValueError(f"lathe: panel is 'front' or None, not {panel!r}")
    mode = _detail(detail, spec is not None)
    w, d, h = (float(v) for v in (size or LATHE_SIZE))
    aw, ah, sill = (float(v) for v in (aperture or LATHE_APERTURE))
    sx_, sd, sz_ = (float(v) for v in (spindle or LATHE_SPINDLE))
    tw, td, th = (float(v) for v in (turret or LATHE_TURRET))
    cd = LATHE_CHAMBER if chamber is None else float(chamber)
    t = float(wall)
    if min(w, d, h, aw, ah, sill, sd, sz_, tw, td, th, cd, t) <= 0:
        raise ValueError("lathe: sizes must be positive")
    if cd + 2 * t > d:
        raise ValueError(f"lathe: a {cd:.2f} m chamber does not fit a {d:.2f} m deep body")
    travel = aw + 0.055 if door_travel is None else float(door_travel)
    speed = door_speed if door_speed is not None else DOOR_SPEED.get(door or "", 0.0)

    x, y = float(position[0]), float(position[1])
    z0 = float(position[2]) if len(position) > 2 else 0.0
    q = _yaw_quat(yaw)
    c, s = math.cos(yaw), math.sin(yaw)
    y0 = -d / 2                       # the front face
    y_r = y0 + t + cd                 # where the rear block begins
    y_c = y0 + t + cd / 2             # the chamber's centre (depth)
    y_s = y0 + t + sd                 # the spindle nose's depth
    leaf_t, over = 0.04, 0.05
    half = aw / 2 + over
    x_c = sx_                         # the opening is centred on the spindle

    # Refused before anything is placed: a plan that does not fit is not
    # a smaller machine.
    if door is not None:
        if sill + ah > h - t + 1e-9:
            raise ValueError(
                f"lathe: the front opening tops out at {(sill + ah) * 1e3:.0f} mm, through a {h * 1e3:.0f} mm body"
            )
        if x_c - aw / 2 < -w / 2 + t - 1e-9 or x_c + aw / 2 > w / 2 - t + 1e-9:
            raise ValueError(
                f"lathe: a {aw * 1e3:.0f} mm front opening centred at x = {x_c * 1e3:.0f} mm does not fit "
                f"the {w * 1e3:.0f} mm front"
            )
        if x_c + half + travel > w / 2 + 1e-9:
            raise ValueError(
                f"lathe: a {travel * 1e3:.0f} mm door stroke runs the leaf off the {w * 1e3:.0f} mm body"
            )
        if door != "manual" and speed <= 0:
            raise ValueError("lathe: door_speed must be positive")
    if abs(sx_) > w / 2 - t - 1e-9 or sd > cd - 1e-9 or sz_ > h - t - 1e-9:
        raise ValueError(
            f"lathe: the spindle at ({sx_ * 1e3:.0f}, {sd * 1e3:.0f}, {sz_ * 1e3:.0f}) mm stands outside the chamber"
        )

    def world(dx: float, dy: float) -> tuple[float, float]:
        return x + c * dx - s * dy, y + s * dx + c * dy

    def box(tag: str, sz: Point3, at: Point3, colour: Color = color, group: str = "shell") -> str:
        px, py = world(at[0], at[1])
        made = scene.add_box(f"{name}/{group}/{tag}" if group else f"{name}/{tag}",
                             size=sz, position=(px, py, z0 + at[2]), quaternion=q, color=colour)
        built.obstacles.append(made)
        return made

    built = MachineTool(name)

    # ---- the enclosure ----------------------------------------------------
    bed_h = min(0.60, sz_ - 0.25)
    box("bed", (w - 2 * t, cd, bed_h), (0.0, y_c, bed_h / 2), MACHINE_BED, group="")
    box("rear", (w, d - t - cd, h), (0.0, (y_r + d / 2) / 2, h / 2), color, group="")
    box("top", (w, t + cd, t), (0.0, (y0 + y_r) / 2, h - t / 2))
    for tag, sign in (("left", -1.0), ("right", 1.0)):
        box(tag, (t, t + cd, h), (sign * (w / 2 - t / 2), (y0 + y_r) / 2, h / 2))
    # The front wall, around the door opening (or solid).
    if door is None:
        box("front", (w, t, h), (0.0, y0 + t / 2, h / 2))
    else:
        yf = y0 + t / 2
        box("front_sill", (w, t, sill), (0.0, yf, sill / 2))
        box("front_head", (w, t, h - sill - ah), (0.0, yf, (sill + ah + h) / 2))
        left_pier = (x_c - aw / 2) + w / 2
        right_pier = w / 2 - (x_c + aw / 2)
        if left_pier > 1e-9:
            box("front_left", (left_pier, t, ah), (-w / 2 + left_pier / 2, yf, sill + ah / 2))
        if right_pier > 1e-9:
            box("front_right", (right_pier, t, ah), (w / 2 - right_pier / 2, yf, sill + ah / 2))

    # ---- the spindle, the turret, the tailstock ----------------------------
    head_w = min(0.45, (w / 2 + x_c) - t - 0.05)
    box("headstock", (head_w, cd * 0.6, sz_ + 0.30 - bed_h),
        (x_c - head_w / 2 - 0.02, y_s + cd * 0.1, (bed_h + sz_ + 0.30) / 2), MACHINE_BED, group="")
    box("turret", (tw, td, th), (x_c + 0.20 + tw / 2, y_s + 0.15, sz_), MACHINE_BED, group="")
    if tailstock:
        box("tailstock", (0.25, 0.30, 0.35), (w / 2 - t - 0.25, y_s, sz_ - 0.05), MACHINE_BED, group="")
    spx, spy = world(x_c, y_s)
    # +Z of the spindle frame along the axis toward the tailstock (+X
    # before yaw): a quarter turn about Y, then the machine's yaw.
    spin_q = _mul_quat(q, (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4)))
    scene.add_frame(f"{name}/spindle", position=(spx, spy, z0 + sz_), quaternion=spin_q)
    built.frames.append(f"{name}/spindle")

    # ---- the front door -----------------------------------------------------
    if door is not None:
        yl = y0 - 0.01 - leaf_t / 2
        zl = sill + ah / 2
        leaf = box("leaf", (2 * half, leaf_t, ah + 2 * over), (x_c, yl, zl), color, group="front_door")
        built.door_objects.append(leaf)
        # The handle: a bar 90 mm off the leaf at its tailstock-side edge,
        # sill + 200 — where a hand (or a fork) takes it. +Z into the leaf
        # is +Y before yaw: a quarter turn about X the other way.
        hx_, hy_ = x_c + half - 0.10, yl - leaf_t / 2 - 0.09
        handle_q = _mul_quat(q, (-math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4)))
        wx, wy = world(hx_, hy_)
        scene.add_frame(f"{name}/door/front/handle", position=(wx, wy, z0 + sill + 0.20), quaternion=handle_q)
        built.frames.append(f"{name}/door/front/handle")
        if mode == "full":
            _trim(scene, built, f"{name}/front_door/handle", (0.02, 0.02, 0.14), (wx, wy, z0 + sill + 0.20), q, DARK_STEEL)
            for i, dz in enumerate((-0.06, 0.06)):
                sx2, sy2 = world(hx_, yl - leaf_t / 2 - 0.045)
                _trim(scene, built, f"{name}/front_door/stub{i}", (0.016, 0.090, 0.016),
                      (sx2, sy2, z0 + sill + 0.20 + dz), q, DARK_STEEL)
            wx2, wy2 = world(x_c, yl - leaf_t / 2 - 0.0025)
            _trim(scene, built, f"{name}/front_door/window", (aw - 0.10, 0.005, ah - 0.30),
                  (wx2, wy2, z0 + zl + 0.05), q, MACHINE_WINDOW)
            for tag, dz in (("rail_top", ah / 2 + over + 0.02), ("rail_bottom", -(ah / 2 + over + 0.02))):
                rx, ry = world(x_c + travel / 2, yl)
                _trim(scene, built, f"{name}/front_door/{tag}", (2 * half + travel, 0.015, 0.03),
                      (rx, ry, z0 + zl + dz), q, DARK_STEEL)
        built.door_travel = travel
        built.door_axis = (c, s, 0.0)
        if door != "manual":
            axis = f"{name}/front_door"
            scene.add_linear_axis(axis, objects=list(built.door_objects), axis=(c, s, 0.0),
                                  speed=float(speed), range=(0.0, travel), position=0.0,
                                  stops={"closed": 0.0, "open": travel})
            built.devices.append(axis)
            built.door = axis
            built.door_lanes = (f"{axis}/closed", f"{axis}/open")
        else:
            lanes = []
            for tag, xz in (("closed", x_c - half - 0.005), ("open", x_c + travel + half + 0.005)):
                zx, zy = world(xz, yl)
                lane = f"{name}/front_door/{tag}"
                scene.add_zone_sensor(lane, position=(zx, zy, z0 + zl), size=(0.02, leaf_t + 0.02, 0.20),
                                      quaternion=q, watch=[leaf])
                built.sensors.append(lane)
                lanes.append(lane)
                scene.set_part(lane, kind="sensor", category="sensor.limit_switch", end=tag)
            built.door_lanes = (lanes[0], lanes[1])
        ex_, ey_ = world(x_c, y0 - 0.01 - leaf_t - 0.15)
        scene.add_frame(f"{name}/entry", position=(ex_, ey_, z0 + zl), quaternion=q)
        built.frames.append(f"{name}/entry")

    # ---- the operator panel ------------------------------------------------
    if panel is not None:
        keys = tuple(buttons) if buttons is not None else ("cycle_start", "feed_hold", "reset", "estop")
        button_models = {}
        if spec is not None:
            for button in keys:
                role = "estop" if button == "estop" else "button"
                if spec.has_component(role):
                    button_models[button] = spec.part_number(role, **params)
        # Right of the opening, clear of the leaf's travel — the pendant's
        # place on a slant-bed lathe.
        clear = x_c + half + (travel if door is not None else 0.0) + 0.25
        px_ = min(w / 2 - 0.25, clear)
        px, py = world(px_, y0 - 0.015 - 0.005)
        made = operator_panel(scene, f"{name}/panel", (px, py, z0 + 1.35), yaw=yaw, tilt=0.35,
                              buttons=keys, columns=2 if len(keys) >= 4 else None, pitch=panel_pitch,
                              button_model=button_models or None,
                              manufacturer=manufacturer if button_models else None)
        built.panel = made
        built.buttons = list(made.sensors)
        if f"{name}/panel/estop" in built.buttons:
            built.estop = f"{name}/panel/estop"
        built.obstacles.extend(made.obstacles)
        built.frames.extend(made.frames)
        built.sensors.extend(made.sensors)
        if spec is not None and button_models:
            for button in made.sensors:
                part = scene.part(button) or {}
                scene.set_part(
                    button, kind="sensor", category=part.get("category") or "hmi.button",
                    catalog=spec.catalog_ref, manufacturer=part.get("manufacturer"),
                    model=part.get("model"), qty=int(part.get("qty") or 1),
                    attributes=dict(part.get("attributes") or {}),
                )

    if mode == "full":
        # The accent band along the roof edge, as on the machining centre.
        bx_, by_ = world(0.0, y0 - 0.0025)
        _trim(scene, built, f"{name}/trim/band", (w, 0.005, 0.06), (bx_, by_, z0 + h - 0.10), q, MACHINE_ACCENT)

    figures = {
        "length_mm": round(w * 1000.0, 1), "depth_mm": round(d * 1000.0, 1), "height_mm": round(h * 1000.0, 1),
        "opening_w_mm": round(aw * 1000.0, 1), "opening_h_mm": round(ah * 1000.0, 1),
        "spindle_height_mm": round(sz_ * 1000.0, 1),
    }
    door_figures = {"drive": door or "none", "stroke_mm": round(travel * 1000.0, 1)}
    if door not in (None, "manual"):
        door_figures["open_s"] = round(travel / speed, 3)
    if spec is None:
        scene.set_part(
            name, kind="group", category="machine_tool.lathe", qty=1,
            **_identity(model, manufacturer, {**figures, **attributes}),
        )
        if door is not None:
            scene.set_part(built.door or f"{name}/front_door", kind="device" if built.door else "group",
                           category="machine_tool.door", **door_figures)
        return built
    built.interface = spec.interface
    recorded = {key: str(_plain(value)) for key, value in {**params, **spec.specs()}.items()}
    scene.set_part(
        name, kind="group", category=spec.category("body", "machine_tool.lathe"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("body", **params), description=spec.name,
        **{**recorded, **figures, **_kg(spec.mass_kg("body", **params)), **attributes},
    )
    if door is not None and spec.has_component("front_door"):
        ordered = (dict(model=spec.part_number("front_door", **params), **_kg(spec.mass_kg("front_door", **params)))
                   if "front_door" in params else {})
        scene.set_part(built.door or f"{name}/front_door", kind="device" if built.door else "group",
                       category=spec.category("front_door", "machine_tool.door"), qty=1,
                       catalog=spec.catalog_ref, manufacturer=manufacturer, **{**door_figures, **ordered})
    if built.panel is not None and spec.has_component("panel"):
        scene.set_part(f"{name}/panel", kind="group", category=spec.category("panel", "hmi.panel"), qty=1,
                       catalog=spec.catalog_ref, manufacturer=manufacturer,
                       model=spec.part_number("panel", **params), buttons=float(len(built.buttons)))
    return built
