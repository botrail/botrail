"""Standard structures, generated from parameters: fences, tables, pedestals,
racks, conveyor bodies, pallets, light curtains — the scenery every cell has
and nobody wants to model.

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
from typing import TYPE_CHECKING, Optional, Sequence

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
        upright = upright if upright is not None else _mm(spec.dimension_mm("bay", "upright", 40.0))
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
    scene.set_part(
        name, kind="group", category=spec.category("bay", "structure.rack"), qty=1,
        catalog=spec.catalog_ref, manufacturer=manufacturer,
        model=model or spec.part_number("bay", **params), description=spec.name,
        **{**recorded, **_kg(spec.mass_kg("bay", **params)), **attributes},
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
