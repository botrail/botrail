"""Cable and tube routes: ``bt.routing``.

A route is the physical run of a connection — which ports it joins (the
connection plan, ``bt.connections``), which line it is made of (a *spec*:
outer diameter, bend radii, sold lengths), which way points, ducts and
clamps it passes. Everything else is derived every time it is read, from
the residents' current poses: the centerline with its bends, the cut and
purchase length, the BOM rows, the cable list and the checks. Move the
cabinet and the cable follows, and so does its length.

Numbers are declared, never guessed: a spec that states no bend radius
leaves the bend checks unknown (``route_spec_unknown``), it does not get a
default. A span between a clamp on a robot link and the next anchor is a
*moving* span: drawn at its time-zero shape here, verified by the rope
replay.
"""

from __future__ import annotations

import csv
import io
import json
import math
import warnings
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

__all__ = [
    "CableList", "RoutingReport", "adopt", "along", "animate", "attach", "cable_list", "carrier", "clamp", "duct",
    "geometry", "remove_route", "remove_spec", "remove_support", "report", "restore", "route", "spec",
    "suggest", "sweep_holder", "through",
]


#: The share of a duct's or a carrier's cross-section a bundle may take
#: before it is reported (`duct_fill`), and the fill a kit's bore is sized
#: for in `scene.requirements()`.
FILL_LIMIT = 0.5


def _plan(scene) -> dict:
    return json.loads(scene._routing_plan_json())


def _pack(catalog, categories: tuple[str, ...], what: str):
    """A routing spec pack from the catalog (an id, or a package directory):
    the loaded :class:`Spec`, its ``specs`` and the reference to record
    (``id@revision``). Refuses a pack of another aisle."""
    from ._spec import Spec

    pack = Spec.load(catalog)
    category = str(pack.manifest.get("category") or "")
    if not any(category == c or (c.endswith(".") and category.startswith(c)) for c in categories):
        raise ValueError(f"{pack.id} is a {category or 'geometry'} package — {what} reads {' / '.join(categories)}")
    specs = pack.manifest.get("specs") or {}
    ident, revision = pack.catalog_ref
    return pack, dict(specs), (f"{ident}@{revision}" if revision else ident)


def _pack_model(pack) -> Optional[str]:
    """What to call the article: the order part number, else the first
    component's part number at the pack's defaults, else the pack's name."""
    order = pack.manifest.get("order") or {}
    if order.get("part_number"):
        return str(order["part_number"])
    components = pack.components()
    if components:
        try:
            return pack.part_number(components[0]["role"], **{k: pack.default(k) for k in pack.params()})
        except (KeyError, ValueError, TypeError):
            pass
    return str(pack.name or "") or None


def restore(scene, plan: dict) -> None:
    """Restore a serialized routing plan (specs, supports, routes)."""
    scene._set_routing_plan_json(json.dumps(plan, allow_nan=False))


def _finite(value, key: str, *, positive: bool = False) -> float:
    try:
        number = float(value)
    except (TypeError, ValueError):
        raise ValueError(f"{key} must be a number, not {value!r}") from None
    if not math.isfinite(number) or number < 0 or (positive and number == 0):
        raise ValueError(f"{key} must be finite and {'positive' if positive else 'nonnegative'}, not {value!r}")
    return number


def _mm(value, key: str) -> Optional[float]:
    return None if value is None else _finite(value, key, positive=True) / 1000.0


def _xyz(value, key: str) -> list[float]:
    try:
        xyz = [float(v) for v in value]
    except (TypeError, ValueError):
        raise ValueError(f"{key} must be three numbers, not {value!r}") from None
    if len(xyz) != 3 or not all(math.isfinite(v) for v in xyz):
        raise ValueError(f"{key} must be three finite numbers, not {value!r}")
    return xyz


def _color(value):
    if value is None:
        return None
    color = [float(c) for c in value]
    if len(color) != 3 or not all(math.isfinite(c) and c >= 0 for c in color):
        raise ValueError(f"color must be three nonnegative linear RGB values, not {value}")
    return color


def attach(scene, at) -> dict:
    """Where something sits, as the plan stores it.

    ``at`` is ``(obstacle_or_frame, (x, y, z))`` — the name of an obstacle
    (it follows ``set_obstacle_pose``) or, failing that, of a frame — or
    ``(robot, link, (x, y, z))`` for a point on a robot link (it follows the
    joints), the offset in the resident's own frame in metres. A dict with
    ``kind`` (``obstacle`` / ``link`` / ``frame``) is taken as is.
    """
    if isinstance(at, dict):
        if at.get("kind") not in ("obstacle", "link", "frame"):
            raise ValueError(f"attachment kind must be obstacle, link or frame, not {at.get('kind')!r}")
        return at
    try:
        parts = list(at)
    except TypeError:
        raise ValueError(f"at must be (name, xyz) or (robot, link, xyz), not {at!r}") from None
    if len(parts) == 2:
        name, offset = parts
        name = str(name)
        offset = _xyz(offset, "at offset")
        if name in scene.obstacle_names:
            return {"kind": "obstacle", "name": name, "offset_m": offset}
        if name in scene.frames:
            return {"kind": "frame", "name": name, "offset_m": offset}
        raise ValueError(f"at: {name!r} is neither an obstacle nor a frame of this scene")
    if len(parts) == 3:
        robot, link, offset = parts
        robot, link = str(robot), str(link)
        if robot not in scene.robots:
            raise ValueError(f"at: unknown robot {robot!r}")
        return {"kind": "link", "robot": robot, "link": link, "offset_m": _xyz(offset, "at offset")}
    raise ValueError(f"at must be (name, xyz) or (robot, link, xyz), not {at!r}")


def _upsert(items: list, key: str, value: dict) -> None:
    index = next((i for i, item in enumerate(items) if item[key] == value[key]), None)
    if index is None:
        items.append(value)
    else:
        items[index] = value


def spec(scene, name: str, *, kind: Optional[str] = None, media=None, od_mm=None, bend_r_fixed_mm=None,
         bend_r_moving_mm=None, mass_kg_m=None, max_length_m=None, torsion_deg_m=None,
         end_allowance_m: float = 0.0, sold_lengths_m=None, order_step_m=None,
         manufacturer: Optional[str] = None, model: Optional[str] = None, catalog: Optional[str] = None,
         color=None) -> str:
    """Declare (or replace) a line — the rule every route made of it follows.

    ``kind`` is ``cable``, ``tube`` or ``hose``; ``media`` the media it may
    carry (``power`` / ``pneumatic`` / ``signal`` / ``network``; unstated =
    unchecked). Dimensions are in millimetres: ``od_mm`` the outer diameter,
    ``bend_r_fixed_mm`` the minimum bend radius laid fixed (the fillet at
    every corner), ``bend_r_moving_mm`` the one in continuous motion.
    ``sold_lengths_m`` (6 and 12 m robot cables) or ``order_step_m`` (20 m
    reels) decide the purchase length; ``end_allowance_m`` is added at each
    end for stripping and the connector; ``max_length_m`` is a product or
    standard limit on one run; ``torsion_deg_m`` the twist the line takes
    per metre of free length (a moving span's holders turning about each
    other). Identity (``manufacturer`` / ``model`` /
    ``catalog``) goes onto the BOM row of every route made of this line.
    A value you do not know you leave out: the checks say *unknown*.

    With ``catalog=`` — the id of a line's spec pack (``cable.*`` /
    ``pneumatic.tube``), or a package directory — the figures the pack
    states (``od_mm``, the bend radii, ``mass_kg_m``, ``media``,
    ``max_length_m``, ``sold_lengths_m`` / ``order_step_m``) are the
    defaults, its maker and part number the identity, and the reference is
    recorded; what you pass explicitly wins, and what the pack does not
    state stays unknown.
    """
    if catalog is not None:
        pack, specs, catalog = _pack(catalog, ("cable.power", "cable.signal", "cable.network", "pneumatic.tube"),
                                     "bt.routing.spec")
        if kind is None:
            kind = specs.get("line_kind") or ("tube" if pack.manifest.get("category") == "pneumatic.tube" else "cable")
        media = media or specs.get("media")
        od_mm = specs.get("od_mm") if od_mm is None else od_mm
        bend_r_fixed_mm = specs.get("bend_r_fixed_mm") if bend_r_fixed_mm is None else bend_r_fixed_mm
        bend_r_moving_mm = specs.get("bend_r_moving_mm") if bend_r_moving_mm is None else bend_r_moving_mm
        mass_kg_m = specs.get("mass_kg_m") if mass_kg_m is None else mass_kg_m
        max_length_m = specs.get("max_length_m") if max_length_m is None else max_length_m
        torsion_deg_m = specs.get("torsion_deg_m") if torsion_deg_m is None else torsion_deg_m
        sold_lengths_m = specs.get("sold_lengths_m") if sold_lengths_m is None else sold_lengths_m
        order_step_m = specs.get("order_step_m") if order_step_m is None else order_step_m
        manufacturer = manufacturer or pack.manufacturer
        model = model or _pack_model(pack)
    kind = kind or "cable"
    if kind not in ("cable", "tube", "hose"):
        raise ValueError(f"kind must be cable, tube or hose, not {kind!r}")
    media = [str(m) for m in (media or [])]
    for m in media:
        if m not in ("power", "pneumatic", "signal", "network"):
            raise ValueError(f"media must be power, pneumatic, signal or network, not {m!r}")
    value = {
        "name": name, "kind": kind, "media": media,
        "od_m": _mm(od_mm, "od_mm"),
        "bend_r_fixed_m": _mm(bend_r_fixed_mm, "bend_r_fixed_mm"),
        "bend_r_moving_m": _mm(bend_r_moving_mm, "bend_r_moving_mm"),
        "mass_kg_m": None if mass_kg_m is None else _finite(mass_kg_m, "mass_kg_m", positive=True),
        "max_length_m": None if max_length_m is None else _finite(max_length_m, "max_length_m", positive=True),
        "torsion_deg_m": None if torsion_deg_m is None else _finite(torsion_deg_m, "torsion_deg_m", positive=True),
        "end_allowance_m": _finite(end_allowance_m, "end_allowance_m"),
        "sold_lengths_m": [_finite(v, "sold_lengths_m", positive=True) for v in (sold_lengths_m or [])],
        "order_step_m": None if order_step_m is None else _finite(order_step_m, "order_step_m", positive=True),
        "manufacturer": manufacturer, "model": model, "catalog": catalog, "color": _color(color),
    }
    plan = _plan(scene)
    _upsert(plan["specs"], "name", value)
    restore(scene, plan)
    return name


HOLDER_KINDS = ("fixed", "rotating", "sliding", "strap", "retracting")


def _holder_kind(kind) -> str:
    if kind not in HOLDER_KINDS:
        raise ValueError(f"a holder's kind is one of {', '.join(HOLDER_KINDS)}, not {kind!r}")
    return str(kind)


def _direction(value, what: str) -> list[float]:
    d = _xyz(value, what)
    if math.hypot(*d) < 1e-9:
        raise ValueError(f"{what}: direction must not be zero")
    return d


def _rail(value: dict, kind: str, axis, stroke_m, force_n, what: str) -> None:
    """A retracting holder's rail onto its plan entry: the axis it runs
    along (the body's frame), its stroke and the pull toward its start."""
    if kind == "retracting":
        if axis is None or stroke_m is None or force_n is None:
            raise ValueError(f"{what}: a retracting holder names its axis=, stroke_m= and force_n=")
        value["axis"] = _direction(axis, what)
        value["stroke_m"] = _finite(stroke_m, f"{what} stroke_m")
        value["force_n"] = _finite(force_n, f"{what} force_n")
    elif axis is not None or stroke_m is not None or force_n is not None:
        raise ValueError(f"{what}: axis, stroke_m and force_n belong to a retracting holder")


def _clamp_article(catalog, *, manufacturer=None, model=None, length_m=None, standoff_m=None, kind=None) -> dict:
    """A clamp article from its spec pack (``cable.clamp``): maker, part
    number, the holding length (the clip's width) and what it leaves the
    line free to do (``kind``) — what is passed wins."""
    pack, specs, ref = _pack(catalog, ("cable.clamp",), "a clamp")
    if length_m is None and specs.get("width_mm") is not None:
        length_m = float(specs["width_mm"]) / 1000.0
    if kind is None and specs.get("kind") is not None:
        kind = _holder_kind(specs["kind"])
    return {"manufacturer": manufacturer or pack.manufacturer, "model": model or _pack_model(pack), "catalog": ref,
            "length_m": 0.0 if length_m is None else float(length_m), "standoff_m": standoff_m,
            "kind": "fixed" if kind is None else kind}


def clamp(scene, name: str, *, at, length_m: Optional[float] = None, kind: Optional[str] = None, direction=None,
          axis=None, stroke_m: Optional[float] = None, force_n: Optional[float] = None,
          manufacturer: Optional[str] = None, model: Optional[str] = None, catalog: Optional[str] = None) -> str:
    """A clamp: a holder the line is held in (``at`` as :func:`attach`).

    On a robot link it is where a moving span starts or ends; ``length_m``
    is the material the holder keeps straight. ``kind`` is what the holder
    leaves the line free to do — ``"fixed"`` (a rigid clip or bracket: the
    place, the direction, no roll, no slide; the default), ``"rotating"``
    (a rotating holder: the line rolls about its own axis), ``"sliding"``
    (a guide the line slides through, and rolls in), ``"strap"`` (a
    strap: the point, and the line turns and rolls) or ``"retracting"``
    (a carriage on a rail, holding like a fixed clamp, pulled toward the
    rail's start: a retraction unit, with ``axis`` the rail's direction in
    the body's frame, ``stroke_m`` how far it runs and ``force_n`` the
    pull) — and ``direction`` the way the line runs through it, in the
    body's frame (the bisector of its two legs when unset). Every clamp is a line of the BOM
    (``cable.clamp``); identical articles merge. With ``catalog=`` (a
    ``cable.clamp`` spec pack) the article's maker, part number, width (as
    the holding length) and kind come from the pack.
    """
    if catalog is not None:
        article = _clamp_article(catalog, manufacturer=manufacturer, model=model, length_m=length_m, kind=kind)
        manufacturer, model, catalog, length_m, kind = (article["manufacturer"], article["model"],
                                                        article["catalog"], article["length_m"], article["kind"])
    value = {"kind": "clamp", "name": name, "at": attach(scene, at),
             "length_m": _finite(0.0 if length_m is None else length_m, "length_m"),
             "hold": _holder_kind("fixed" if kind is None else kind),
             "manufacturer": manufacturer, "model": model, "catalog": catalog}
    if direction is not None:
        value["direction"] = _direction(direction, f"clamp {name!r}")
    _rail(value, value["hold"], axis, stroke_m, force_n, f"clamp {name!r}")
    plan = _plan(scene)
    _upsert(plan["supports"], "name", value)
    restore(scene, plan)
    return name


def duct(scene, name: str, *, path, inner_m, manufacturer: Optional[str] = None,
         model: Optional[str] = None, catalog: Optional[str] = None) -> str:
    """A duct or tray as a support only: the centerline a route follows end
    to end (``path``, world metres, two or more points) and its inner
    cross-section ``inner_m = (width, height)``. ``bt.parts.duct`` builds
    the boxes on the floor and declares this in one call; call this
    directly for a duct whose body is already in the scene (imported
    geometry) or needs none.
    """
    points = [_xyz(p, "path") for p in path]
    if len(points) < 2:
        raise ValueError("a duct path needs two or more points")
    inner = [_finite(v, "inner_m", positive=True) for v in inner_m]
    if len(inner) != 2:
        raise ValueError("inner_m is (width, height)")
    value = {"kind": "duct", "name": name, "path": points, "inner_m": inner,
             "manufacturer": manufacturer, "model": model, "catalog": catalog}
    plan = _plan(scene)
    _upsert(plan["supports"], "name", value)
    restore(scene, plan)
    return name


def carrier(scene, name: str, *, holders=None, inner_mm=None, od_mm=None, bend_r_moving_mm=None, torsion_deg_m=None,
            mass_kg_m: Optional[float] = None, stroke_m: Optional[float] = None, manufacturer: Optional[str] = None,
            model: Optional[str] = None, catalog: Optional[str] = None, robot: Optional[str] = None) -> str:
    """A protective hose or 3D energy chain dressed along an arm: the dress
    pack the lines to the hand run in together.

    ``holders`` are where it is fixed, in order (each as :func:`attach`,
    typically ``(robot, link, offset)`` on the upper arm, the forearm, the
    wrist — or ``dict(at=..., kind=..., direction=..., axis=..., stroke_m=..., force_n=...)``
    with the holder's kind, direction and rail as for :func:`clamp`: a
    rotating holder lets the hose roll, a sliding one lets it slide
    through, a retracting one is the kit's retraction unit); ``inner_mm``
    its bore. A route passes it as one way point,
    ``via=[name]``, and takes the holders as its anchors; every route
    through it is one bundle, whose cross-section is checked against the
    bore (``duct_fill``) and whose media are kept apart (``route_separation``
    for power with signal). The hose is a line of the BOM
    (``cable.carrier``) as long as the longest route dressed in it. Over a
    cycle, :func:`animate` replays the stretch between two holders as one
    hose — ``od_mm`` thick, ``mass_kg_m`` of its own plus the lines
    inside — and the lines read that replay (``carrier:<name>#<i>``).
    ``stroke_m`` is what the kit's retraction takes up, when it has one.
    With ``catalog=`` (a ``cable.carrier`` spec pack: a conduit, or a
    robot's dress-pack kit) the bore, the diameter, the bend radius, the
    torsion, the mass and the stroke the pack states are the defaults and
    its maker and part number the identity; a bore nobody states leaves
    the bundle's fill unchecked and the requirement row saying so. A kit
    that states where its holders go (``holders_at``: a link and an offset
    in mm per holder) places them on ``robot`` when you pass no
    ``holders``.

    The carrier is a line of ``scene.requirements()``: the cell asks it for
    a bore (the bundle at 50 % fill) and, over a baked cycle, the hose
    length between the holders, the stroke, the twist per metre and the
    bend radius the replay showed — compared with what is declared here
    (or, through ``catalog=``, what a kit states), and the figures
    ``bt.catalog.search_for`` looks for.
    """
    if catalog is not None:
        pack, specs, catalog = _pack(catalog, ("cable.carrier",), "bt.routing.carrier")
        inner_mm = specs.get("inner_mm") if inner_mm is None else inner_mm
        od_mm = specs.get("od_mm") if od_mm is None else od_mm
        bend_r_moving_mm = specs.get("bend_r_moving_mm") if bend_r_moving_mm is None else bend_r_moving_mm
        torsion_deg_m = specs.get("torsion_deg_m") if torsion_deg_m is None else torsion_deg_m
        mass_kg_m = specs.get("mass_kg_m") if mass_kg_m is None else mass_kg_m
        stroke_m = specs.get("stroke_m") if stroke_m is None else stroke_m
        manufacturer = manufacturer or pack.manufacturer
        model = model or _pack_model(pack)
        if holders is None and specs.get("holders_at"):
            if robot is None:
                raise ValueError(f"carrier: {pack.id} states where its holders go — pass robot= to place them")
            holders = [(robot, str(h["link"]), tuple(float(v) / 1000.0 for v in h["xyz_mm"]))
                       for h in specs["holders_at"]]
    if holders is None:
        raise ValueError("a carrier names its holders (or a kit that states holders_at, with robot=)")
    hs = []
    for i, h in enumerate(holders):
        if isinstance(h, dict) and "at" in h:
            unknown = set(h) - {"at", "kind", "direction", "axis", "stroke_m", "force_n"}
            if unknown:
                raise ValueError(f"carrier holder {i}: unknown keys {sorted(unknown)}")
            holder = {"at": attach(scene, h["at"]), "kind": _holder_kind(h.get("kind") or "fixed")}
            if h.get("direction") is not None:
                holder["direction"] = _direction(h["direction"], f"carrier holder {i}")
            _rail(holder, holder["kind"], h.get("axis"), h.get("stroke_m"), h.get("force_n"), f"carrier holder {i}")
        else:
            holder = {"at": attach(scene, h), "kind": "fixed"}
        hs.append(holder)
    if not hs:
        raise ValueError("a carrier has at least one holder")
    value = {"kind": "carrier", "name": name, "holders": hs, "inner_m": _mm(inner_mm, "inner_mm"),
             "od_m": _mm(od_mm, "od_mm"), "bend_r_moving_m": _mm(bend_r_moving_mm, "bend_r_moving_mm"),
             "torsion_deg_m": None if torsion_deg_m is None else _finite(torsion_deg_m, "torsion_deg_m", positive=True),
             "manufacturer": manufacturer, "model": model, "catalog": catalog}
    if mass_kg_m is not None:
        value["mass_kg_m"] = _finite(mass_kg_m, "mass_kg_m", positive=True)
    if stroke_m is not None:
        value["stroke_m"] = _finite(stroke_m, "stroke_m")
    plan = _plan(scene)
    _upsert(plan["supports"], "name", value)
    restore(scene, plan)
    return name


def through(name: str) -> dict:
    """A way point meaning "run through the support ``name``" — the same as
    passing the support's name as a string in ``via``."""
    return {"kind": "support", "name": name}


def along(on, *, pitch_m: Optional[float] = None, face: Optional[str] = None,
          offset_m: Optional[float] = None, inset_m=None) -> dict:
    """A way point meaning "run along the body ``on``": an obstacle's name,
    or ``(robot, link)`` for a robot link.

    The route is laid on a face of the body's collision bounds — ``face``
    (``"+x"`` … ``"-z"``, in the body's frame) when given, else the side
    face, one along the body's long axis, nearest the way in and the way
    out — the line's radius plus the clamp's standoff off the surface, and
    held by auto clamps every ``pitch_m`` (the route's
    ``clamp_pitch_m``, 0.3 m, when unset) — ``<route>/c0``, ``c1``, … in the
    route's anchors, counted in the BOM under the route's ``clamp``
    article. On a robot link the run rides the link: its spans are
    ``dressed``, replayed with the arm but not sized for free length, and
    its ends are drawn in until they clear the robot's other links (the
    joint housings at either end of an arm link). A
    lateral offset between the way in and the way out smaller than a pitch
    is taken up by the way in, so the run itself is parallel to the body's
    edge; the entry and the exit are the projections of the neighbouring
    way points, so the run covers what lies between them. ``offset_m``
    moves the run sideways on its face, along the body's axis across the
    run (toward that axis's positive end) — two lines tied along one face
    side by side, each with its own clips, are ``offset_m=0.007`` and
    ``offset_m=-0.007``. ``inset_m`` stops the run short at its ends, past
    the clearance of the neighbouring links: one figure for both ends or
    ``(way_in, way_out)`` — ``inset_m=(0.12, 0)`` leaves the first 12 cm of
    an arm link free for the loop that climbs to it.
    """
    if isinstance(on, str):
        body = {"kind": "obstacle", "name": on}
    else:
        try:
            robot, link = on
        except (TypeError, ValueError):
            raise ValueError(f"along: on is an obstacle name or (robot, link), not {on!r}") from None
        body = {"kind": "link", "robot": str(robot), "link": str(link)}
    value = {"kind": "along", "on": body}
    if pitch_m is not None:
        value["pitch_m"] = _finite(pitch_m, "pitch_m", positive=True)
    if face is not None:
        if face not in ("+x", "-x", "+y", "-y", "+z", "-z"):
            raise ValueError(f"along: face is one of +x, -x, +y, -y, +z, -z, not {face!r}")
        value["face"] = face
    if offset_m is not None:
        if not math.isfinite(float(offset_m)):
            raise ValueError(f"along: offset_m must be finite, not {offset_m!r}")
        value["offset_m"] = float(offset_m)
    if inset_m is not None:
        pair = (inset_m, inset_m) if isinstance(inset_m, (int, float)) else tuple(inset_m)
        if len(pair) != 2:
            raise ValueError(f"along: inset_m is one figure or (way_in, way_out), not {inset_m!r}")
        value["inset_m"] = [_finite(v, "inset_m") for v in pair]
    return value


def _via(item) -> dict:
    if isinstance(item, str):
        return {"kind": "support", "name": item}
    if isinstance(item, dict):
        if item.get("kind") == "support":
            return {"kind": "support", "name": str(item["name"])}
        if item.get("kind") == "point":
            return {"kind": "point", "position": _xyz(item["position"], "via")}
        if item.get("kind") == "along":
            return item
        raise ValueError(f"via entries are support names, points, through(...) or along(...), not {item!r}")
    return {"kind": "point", "position": _xyz(item, "via")}


def route(scene, name: str, *, carries, spec: str, via=(), orthogonal: bool = False,
          slack_m=0.0, collide: bool = True, color=None, clamp_pitch_m: Optional[float] = None,
          clamp=None) -> str:
    """Declare (or replace) a route.

    ``carries`` names the connections (``bt.connections``) this line runs —
    one, or several for a multi-core cable; the first decides the two end
    ports, which must have geometry (``at=`` / ``exit=`` on the port).
    ``spec`` names the line. ``via`` is the ordered way: points ``(x, y,
    z)``, clamp names, duct names (:func:`through`), bodies to run along
    (:func:`along`). ``orthogonal`` inserts axis-aligned elbows between way
    points (z, then x, then y), the way a line runs along a wall and a
    floor. ``slack_m`` is material beyond the measured run — the service
    loop a moving span needs: one figure shared by the route's moving
    spans, or a list with one figure per moving span in route order.
    ``collide`` makes the fixed spans colliders for the planner, the tick
    verification and the clearance readout (named ``route:<name>#<span>``);
    the static clash check runs either way. ``clamp_pitch_m`` is the clamp
    pitch of the runs along a body, ``clamp`` the article those clamps are
    (with ``kind`` as for :func:`clamp`: straps along a cobot's arm are
    ``clamp=dict(kind="strap", ...)``) — the article those clamps are
    — a dict with ``manufacturer``, ``model``, ``catalog``, ``length_m``
    (material held straight in each) and ``standoff_m`` (the saddle's
    height, 5 mm when unset).
    """
    carries = [str(c) for c in ([carries] if isinstance(carries, str) else carries)]
    if not carries:
        raise ValueError("a route carries at least one connection")
    if isinstance(slack_m, (list, tuple)):
        slacks = [_finite(v, "slack_m") for v in slack_m]
        slack, slacks_m = math.fsum(slacks), slacks
    else:
        slack, slacks_m = _finite(slack_m, "slack_m"), []
    value = {"name": name, "carries": carries, "spec": spec, "via": [_via(v) for v in via],
             "orthogonal": bool(orthogonal), "slack_m": slack, "slacks_m": slacks_m, "collide": bool(collide),
             "color": _color(color)}
    if clamp_pitch_m is not None:
        value["clamp_pitch_m"] = _finite(clamp_pitch_m, "clamp_pitch_m", positive=True)
    if clamp is not None:
        if isinstance(clamp, str):
            clamp = {"model": clamp}
        unknown = set(clamp) - {"manufacturer", "model", "catalog", "length_m", "standoff_m", "kind"}
        if unknown:
            raise ValueError(f"clamp: unknown keys {sorted(unknown)}")
        if clamp.get("catalog"):
            # The article from its spec pack; what is written here wins.
            clamp = {**_clamp_article(clamp["catalog"], manufacturer=clamp.get("manufacturer"),
                                      model=clamp.get("model"), length_m=clamp.get("length_m"),
                                      standoff_m=clamp.get("standoff_m"), kind=clamp.get("kind"))}
        article = {"manufacturer": clamp.get("manufacturer"), "model": clamp.get("model"),
                   "catalog": clamp.get("catalog"), "length_m": _finite(clamp.get("length_m") or 0.0, "clamp length_m"),
                   "kind": _holder_kind(clamp.get("kind") or "fixed")}
        if clamp.get("standoff_m") is not None:
            article["standoff_m"] = _finite(clamp["standoff_m"], "clamp standoff_m")
        value["clamp"] = article
    plan = _plan(scene)
    _upsert(plan["routes"], "name", value)
    restore(scene, plan)
    return name


def _remove(scene, key: str, name: str, what: str) -> None:
    plan = _plan(scene)
    if not any(item["name"] == name for item in plan[key]):
        raise ValueError(f"unknown {what}: {name}")
    plan[key] = [item for item in plan[key] if item["name"] != name]
    restore(scene, plan)


def remove_route(scene, name: str) -> None:
    _remove(scene, "routes", name, "route")


def remove_support(scene, name: str) -> None:
    """Remove a clamp or duct; routes that pass it report ``route_endpoint``."""
    _remove(scene, "supports", name, "support")


def remove_spec(scene, name: str) -> None:
    """Remove a spec; routes made of it report ``route_endpoint``."""
    _remove(scene, "specs", name, "spec")


def suggest(scene, route: str) -> list[dict]:
    """The ways ``route`` could take through the ducts the cell has, best
    first. The way points you fixed — points, clamps, carriers, runs along
    a body — stay where they are, in order; every ordered choice of up to
    three ducts is tried between them, each candidate resolved and checked
    at the current poses. A candidate carries its ``via`` (what
    :func:`adopt` or :func:`route` takes), the ``ducts`` it passes,
    ``length_m``, ``cut_length_m``, ``bends``, ``supports``, the static
    ``errors`` / ``warnings`` with their ``codes`` (the ducts' bundle
    findings included), its ``open_m`` (the run outside any duct), a
    ``score`` (length + the open run again + 0.3 per bend + 0.2 per
    support: a metre in a duct beats a metre on the floor) and ``current``
    for the way the route takes now. The order is
    by errors, then score, then warnings — the same every time for the
    same cell. Free space is not searched: a candidate is a choice of
    ducts, not a path around an obstacle."""
    return json.loads(scene._route_candidates_json(route))


def adopt(scene, route: str, candidate) -> None:
    """Give ``route`` the way of ``candidate`` — a row of :func:`suggest`,
    or its index in that list."""
    if isinstance(candidate, int):
        candidate = suggest(scene, route)[candidate]
    scene._set_route_via_json(route, json.dumps(candidate["via"]))


def geometry(scene, name: str) -> dict:
    """One route's derived shape at the current poses: ``centerline``,
    ``s`` (arc length at each point), ``anchors`` (each with its
    ``position``, ``s_m``, ``moving`` and the ``tangent`` the line runs
    through it along: a port's stub, an auto clamp's run, a declared
    clamp's bisector), ``spans``, ``bends``, ``length_m``,
    ``cut_length_m``, ``purchase_length_m``."""
    return json.loads(scene._route_geometry_json(name))


def _cell(value) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return format(value, ".8g")
    if isinstance(value, (list, tuple)):
        return ", ".join(_cell(v) for v in value)
    if isinstance(value, dict):
        return _cell(value.get("port"))
    return str(value).replace("|", "\\|").replace("\n", "<br>")


@dataclass
class RoutingReport:
    """The checks: ``findings`` (``severity`` / ``code`` / ``route`` /
    ``message`` and, when located, ``position``, ``s_m`` and — on a cycle —
    ``t_s``) over the ``routes`` of the cable list, and, when a timeline was
    given, the ``spans`` that move: what the kinematics say of each (the
    holders' distance and twist over the cycle, hence the free length and
    the stroke) and what the rope replay says (bend, stretch, sag, nearest
    body). ``ok`` is "no error"; warnings and unknowns stay listed."""
    routes: list[dict]
    findings: list[dict]
    spans: list[dict] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not any(f["severity"] == "error" for f in self.findings)

    def errors(self) -> list[dict]:
        return [f for f in self.findings if f["severity"] == "error"]

    def warnings(self) -> list[dict]:
        return [f for f in self.findings if f["severity"] == "warning"]

    def to_dict(self) -> dict:
        return {"ok": self.ok, "routes": self.routes, "spans": self.spans, "findings": self.findings}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False, allow_nan=False)

    def to_markdown(self) -> str:
        lines = ["# Cable and tube routes\n", "Checks: " + ("no errors" if self.ok else "errors"),
                 "\nLengths follow the residents' current poses. A moving span's motion is judged on the baked cycle: the kinematics give its free length, stroke and twist; the rope replay its shape, clearance and slack — its stiffness is the solver's, so tension, torsion and life are not figures here. Nothing sizes a conductor or a tube.",
                 "\n## Routes\n", "| route | spec | medium | from | to | run m | cut m | purchase m | ducts | clamps | status |",
                 "|---|---|---|---|---|---|---|---|---|---|---|"]
        for r in self.routes:
            lines.append("| " + " | ".join(_cell(r.get(k)) for k in (
                "name", "spec", "medium", "from", "to", "length_m", "cut_length_m", "purchase_length_m",
                "ducts", "clamps", "status")) + " |")
        if self.spans:
            lines.extend(["\n## Moving spans\n",
                          "| route | span | kind | line m | holders min–max m | stroke m | free needed m | twist ° | min bend r m | stretch | sag m | clearance m | nearest | status |",
                          "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|"])
            for sp in self.spans:
                lines.append("| " + " | ".join(_cell(v) for v in (
                    sp["route"], f"{sp['from']} → {sp['to']}", sp["kind"], sp["material_m"],
                    f"{_cell(sp['min_distance_m'])}–{_cell(sp['max_distance_m'])}", sp["stroke_m"], sp["required_free_m"],
                    sp["twist_deg"], sp.get("min_bend_radius_m"), sp.get("max_stretch"), sp.get("max_sag_m"),
                    sp.get("min_clearance_m"), sp.get("nearest"), sp["status"])) + " |")
        lines.extend(["\n## Findings\n", "| severity | code | route | message | at (m) | t (s) |", "|---|---|---|---|---|---|"])
        for f in self.findings:
            lines.append("| " + " | ".join(_cell(f.get(k)) for k in ("severity", "code", "route", "message", "s_m", "t_s")) + " |")
        return "\n".join(lines) + "\n"

    def save(self, path) -> None:
        path = Path(path)
        if path.suffix == ".json":
            path.write_text(self.to_json(), encoding="utf-8")
        elif path.suffix == ".md":
            path.write_text(self.to_markdown(), encoding="utf-8")
        else:
            raise ValueError("routing report extension must be .json or .md")


def report(scene, timeline=None, *, dt: float = 0.05, sag_limit_m: float = 0.15) -> RoutingReport:
    """Resolve every route at the current poses and check it: bends against
    the spec's fixed radius, the exit out of each connector, the cut length
    against the sold lengths and the maximum run, fixed spans against the
    enabled obstacles and the robots at their current pose, routes against
    each other, and the connections placed but not yet routed.

    With a baked ``timeline`` the moving spans are judged over the cycle
    too, every ``dt`` seconds: the distance between a span's holders gives
    the free length it needs (``route_tension`` when the line between them
    is shorter) and the stroke a retraction takes up; their whole relative
    rotation the twist (``route_twist`` against the spec's
    ``torsion_deg_m`` — bend and torsion together, the worst case); and,
    where :func:`animate` replayed the span as a
    rope, its tightest bend (``route_bend_moving``), stretch
    (``route_stretch``), sag over ``sag_limit_m`` (``route_sag``) and the
    nearest body (``route_contact`` / ``route_clearance``) — each finding
    with its time.
    """
    data = json.loads(scene._routing_report_json())
    spans: list[dict] = []
    findings = list(data["findings"])
    if timeline is not None:
        motion = json.loads(scene._routing_timeline_report_json(timeline, dt, sag_limit_m))
        spans = motion["spans"]
        findings.extend(motion["findings"])
    return RoutingReport(data["routes"], findings, spans)


def _span_points(g: dict, span: dict) -> list[list[float]]:
    return [p for p, s in zip(g["centerline"], g["s"]) if span["from_s"] - 1e-9 <= s <= span["to_s"] + 1e-9]


def _with_slack(points: list[list[float]], slack: float, room=None) -> list[list[float]]:
    """The span's polyline with ``slack`` metres more line in it: a dip at
    the middle of its longest run, so the rope starts with the material
    it has rather than being born taut. The dip hangs down, unless another
    side, turned about the run in 45° steps, has more room by
    ``room(points)`` (how far that shape keeps from the bodies around it;
    negative: it starts inside one) — a stiff line keeps the loop it is
    laid in, so it is laid where there is space for it."""
    if slack <= 0 or len(points) < 2:
        return points
    longest, at = 0.0, 0
    for i in range(len(points) - 1):
        run = math.dist(points[i], points[i + 1])
        if run > longest:
            longest, at = run, i
    half = (longest + slack) / 2
    drop = math.sqrt(max(half * half - (longest / 2) ** 2, 0.0))
    a, b = points[at], points[at + 1]
    m = [(a[i] + b[i]) / 2 for i in range(3)]
    u = [(b[i] - a[i]) / longest for i in range(3)] if longest > 1e-12 else [1.0, 0.0, 0.0]
    # Down, square to the run (a vertical run: any square direction).
    down = [0.0 - u[2] * -u[0], 0.0 - u[2] * -u[1], -1.0 - u[2] * -u[2]]
    if math.hypot(*down) < 1e-6:
        down = [u[1], -u[0], 0.0] if abs(u[2]) < 0.9 else [0.0, u[2], -u[1]]
    n = math.hypot(*down)
    d0 = [v / n for v in down]
    d1 = [u[1] * d0[2] - u[2] * d0[1], u[2] * d0[0] - u[0] * d0[2], u[0] * d0[1] - u[1] * d0[0]]
    best, best_room = None, -math.inf
    for deg in (0, 45, -45, 90, -90, 135, -135, 180):
        c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        d = [c * d0[i] + s * d1[i] for i in range(3)]
        shaped = points[: at + 1] + [[m[i] + drop * d[i] for i in range(3)]] + points[at + 1:]
        if room is None:
            return shaped
        space = room(shaped)
        if space > best_room:
            best, best_room = shaped, space
    return best


def _led(points: list[list[float]], end: str, tangent, held: float) -> list[list[float]]:
    """The span's polyline with its first (``end="Start"``) or last
    (``"End"``) ``held`` metres laid straight along ``tangent``, the way
    the holder there runs the line — the material a clip holds, which the
    polyline turns at."""
    if end == "Start":
        p = points[0]
        lead = [p[i] + tangent[i] * held for i in range(3)]
        rest = [q for q in points[1:] if math.dist(q, p) > held + 1e-9]
        return [p, lead] + rest
    p = points[-1]
    tail = [p[i] - tangent[i] * held for i in range(3)]
    rest = [q for q in points[:-1] if math.dist(q, p) > held + 1e-9]
    return rest + [tail, p]


def _densify(points: list[list[float]], step: float) -> list[list[float]]:
    """A polyline with a point at least every ``step`` along it."""
    out = [list(points[0])]
    for a, b in zip(points, points[1:]):
        n = max(1, math.ceil(math.dist(a, b) / step))
        out += [[a[i] + (b[i] - a[i]) * k / n for i in range(3)] for k in range(1, n + 1)]
    return out


def _interior(points: list[list[float]], keep_off: float) -> list[list[float]]:
    """The points of a polyline more than ``keep_off`` along it from either
    end — what a starting shape is judged by (its ends are the clips)."""
    s = [0.0]
    for a, b in zip(points, points[1:]):
        s.append(s[-1] + math.dist(a, b))
    return [p for p, sv in zip(points, s) if keep_off < sv < s[-1] - keep_off]


def _anchor_of(scene, plan: dict, anchor: dict, route: Optional[dict] = None) -> tuple[dict, float]:
    """The attachment behind a route anchor and the length it holds."""
    if anchor.get("auto") and anchor.get("attach"):
        clamp = (route or {}).get("clamp") or {}
        return anchor["attach"], float(clamp.get("length_m") or 0.0) / 2
    if anchor["kind"] == "port":
        ports = json.loads(scene._connection_plan_json())["ports"]
        port = next(p for p in ports if p["name"] == anchor["name"])
        return port["geometry"]["at"], float(port["geometry"].get("straight_m") or 0.0)
    support = next((sp for sp in plan["supports"] if sp["name"] == anchor["name"]), None)
    if support is not None:
        return support["at"], float(support.get("length_m") or 0.0) / 2
    # A carrier's holder: `<carrier>/h<k>`.
    carrier_name, k = anchor["name"].rsplit("/h", 1)
    carrier_ = next(sp for sp in plan["supports"] if sp["name"] == carrier_name)
    return carrier_["holders"][int(k)]["at"], 0.0


def _duct_channels(plan: dict, ducts, wall: float = 0.01) -> list:
    """The floor and the two walls of each duct the route lies in, as static
    shapes for the replay: the cell models a duct as one box (which the
    line could not lie inside), the replay needs the channel."""
    shapes = []
    for name in ducts:
        duct_ = next((sp for sp in plan["supports"] if sp["name"] == name and sp["kind"] == "duct"), None)
        if duct_ is None:
            continue
        w, h = duct_["inner_m"]
        for a, b in zip(duct_["path"], duct_["path"][1:]):
            dx, dy = b[0] - a[0], b[1] - a[1]
            run = math.hypot(dx, dy)
            if run <= 1e-9:
                continue
            yaw = math.atan2(dy, dx)
            q = [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]
            cx, cy, cz = (a[0] + b[0]) / 2, (a[1] + b[1]) / 2, (a[2] + b[2]) / 2
            nx, ny = -math.sin(yaw), math.cos(yaw)  # lateral
            shapes.append(({"position": [cx, cy, cz - h / 2 - wall / 2], "quaternion": q},
                           {"Cuboid": [run / 2 + w / 2, w / 2 + wall, wall / 2]}))
            for side in (-1, 1):
                off = side * (w / 2 + wall / 2)
                shapes.append(({"position": [cx + nx * off, cy + ny * off, cz], "quaternion": q},
                               {"Cuboid": [run / 2 + w / 2, wall / 2, h / 2]}))
    return shapes


def _carrier_span(span: dict) -> Optional[tuple[str, int]]:
    """``(carrier, i)`` for a span from holder ``i`` to holder ``i + 1`` of
    one carrier — the stretch that is one hose."""
    try:
        c0, i = span["from"].rsplit("/h", 1)
        c1, j = span["to"].rsplit("/h", 1)
        if c0 == c1 and int(j) == int(i) + 1:
            return c0, int(i)
    except ValueError:
        pass
    return None


def _hose_figures(spans: list[dict], carrier: str) -> Optional[dict]:
    """What the hose of ``carrier`` is asked for, from the rows of a cycle
    report: the free length its spans need in total (each span: the worst
    line's), the largest stroke, the largest twist per metre of line and
    the tightest replayed bend — each with the span it came from.
    ``None`` when no span of the report runs inside it."""
    by_index: dict[int, list[dict]] = {}
    for sp in spans:
        hose = _carrier_span(sp)
        if hose is not None and hose[0] == carrier:
            by_index.setdefault(hose[1], []).append(sp)
    if not by_index:
        return None
    out: dict = {"free_length_m": 0.0, "free_basis": [], "stroke_m": 0.0, "stroke_span": None,
                 "twist_deg_m": 0.0, "twist_at": None, "bend_r_m": None, "bend_span": None}
    for i in sorted(by_index):
        rows = by_index[i]
        worst = max(rows, key=lambda sp: sp["required_free_m"])
        out["free_length_m"] += worst["required_free_m"]
        out["free_basis"].append(f"h{i} → h{i + 1} {worst['max_distance_m']:.2f} m apart at most")
        stroke = max(sp["stroke_m"] for sp in rows)
        if out["stroke_span"] is None or stroke > out["stroke_m"]:
            out["stroke_m"], out["stroke_span"] = stroke, i
        for sp in rows:
            if sp["material_m"] > 0:
                per_m = sp["twist_deg"] / sp["material_m"]
                if out["twist_at"] is None or per_m > out["twist_deg_m"]:
                    out["twist_deg_m"], out["twist_at"] = per_m, (i, sp["twist_deg"], sp["material_m"])
            r = sp.get("min_bend_radius_m")
            if r is not None and (out["bend_r_m"] is None or r < out["bend_r_m"]):
                out["bend_r_m"], out["bend_span"] = r, i
    return out


def sweep_holder(scene, timeline, carrier: str, holder: int, *, x=None, y=None, z=None, dt: float = 0.05):
    """Try places for one holder of a carrier and table what the hose then
    goes through over the baked cycle.

    ``holder`` is the holder's index in the carrier's list; ``x`` / ``y`` /
    ``z`` are the candidate coordinates in the holder's own frame (the
    link's), each a list — the grid is their product, an axis not given
    keeps the holder's current value. Nothing is re-baked: the cycle is the
    same, only the plan's holder moves, and the kinematic report is read
    again at every point. Each row carries the holder's coordinates and
    ``stroke_m`` (the largest over the spans the holder ends),
    ``free_length_m`` (what the hose needs between its holders, in total),
    ``twist_deg`` (the largest over those spans) and ``errors`` (the route
    findings, static and over the cycle, that are errors on the routes
    dressed in the carrier). The replay is not run: contact is not among
    the figures. Returns a :class:`botrail.study.Sweep` — ``best("stroke_m")``
    is the place a retraction has least to do — and leaves the plan as it
    was.
    """
    import itertools

    from . import study

    plan = _plan(scene)
    support = next((s for s in plan["supports"] if s["kind"] == "carrier" and s["name"] == carrier), None)
    if support is None:
        raise ValueError(f"unknown carrier: {carrier}")
    holders = support["holders"]
    if not 0 <= holder < len(holders):
        raise ValueError(f"carrier {carrier!r} has holders 0..{len(holders) - 1}, not {holder}")
    base = [float(v) for v in holders[holder]["at"]["offset_m"]]
    axes = []
    for key, i, values in (("x", 0, x), ("y", 1, y), ("z", 2, z)):
        if values is None:
            continue
        coords = [float(v) for v in values]
        if not coords or not all(math.isfinite(v) for v in coords):
            raise ValueError(f"sweep_holder: {key}= is a nonempty list of finite coordinates")
        axes.append((key, i, coords))
    if not axes:
        raise ValueError("sweep_holder: give x=, y= or z= (lists of coordinates in the holder's frame)")
    names = [key for key, _, _ in axes]
    points = list(itertools.product(*[values for _, _, values in axes]))
    routes = [r["name"] for r in plan["routes"]
              if any(v.get("kind") == "support" and v.get("name") == carrier for v in r["via"])]
    original = json.loads(json.dumps(plan))
    rows: list[dict] = []
    try:
        for point in points:
            offset = list(base)
            for (_, i, _), v in zip(axes, point):
                offset[i] = float(v)
            trial = json.loads(json.dumps(original))
            next(s for s in trial["supports"] if s["name"] == carrier)["holders"][holder]["at"]["offset_m"] = offset
            row: dict = dict(zip(names, (float(v) for v in point)))
            try:
                restore(scene, trial)
                static = json.loads(scene._routing_report_json())
                motion = json.loads(scene._routing_timeline_report_json(timeline, dt, 0.15))
            except ValueError as e:
                rows.append({**row, "ok": False, "error": str(e)})
                continue
            mine = [sp for sp in motion["spans"] if sp["route"] in routes]
            ended = [sp for sp in mine if f"{carrier}/h{holder}" in (sp["from"], sp["to"])]
            figures = _hose_figures(mine, carrier) or {}
            errors = sum(1 for f in static["findings"] + motion["findings"]
                         if f["severity"] == "error" and f["route"] in routes + [carrier])
            rows.append({**row, "ok": True, "error": None,
                         "stroke_m": round(max((sp["stroke_m"] for sp in ended), default=0.0), 3),
                         "free_length_m": round(float(figures.get("free_length_m") or 0.0), 2),
                         "twist_deg": round(max((sp["twist_deg"] for sp in ended), default=0.0), 1),
                         "errors": errors})
    finally:
        restore(scene, original)
    return study.Sweep(params=names, metrics=["stroke_m", "free_length_m", "twist_deg", "errors"], rows=rows)


def animate(scene, timeline, *, routes=None, settle_fixed: bool = False, ground_z=0.0,
            step_s: float = 1 / 480, spacing_m=None, axial_hz: float = 500.0, bending_hz: float = 120.0,
            friction: float = 0.5, margin_m: float = 0.5, arm_links: bool = True, settle_s: float = 1.0,
            rest: str = "straight", rod_hz: float = 120.0, solver: str = "physics", rigidity_nm2=None):
    """Replay the routes' moving spans as ropes against a baked cycle.

    Every moving span (between a clamp on a robot link, or a port on one,
    and its neighbour) becomes a ``bt.rope`` track named
    ``route:<route>#<span>``: held at both ends by what holds it in the
    plan (a port's ``straight_m``, half a clamp's ``length_m``; a fixed or
    rotating holder holds the line's direction too — one sample spacing at
    least, led along its ``direction``, the bisector of its legs, or the
    run of an ``along`` clip — so the line leaves the holder the way it
    points it; a strap holds a point; a port on a moving holder is a
    swivel fitting and holds a point). The spans the line slides through
    a holder between are one rope, through a ring on the holder's body
    at each sliding holder, named after the first of them, with their
    slack together. Each starts from its time-zero polyline with its share of the route's ``slack_m``
    hung in as a dip, at the line's radius, colliding with the enabled
    obstacles within ``margin_m`` of it (not the ducts it runs in, not the
    residents its holders sit on, not a body its starting polyline already
    passes through — the replay would blow apart on it; the static report
    names that clash) and with the floor at ``ground_z`` (``None`` for
    none). A span dressed along a link (both ends on it) is replayed the
    same way; the stretch between two holders of a carrier is replayed
    once, as the hose — ``carrier:<name>#<i>``, the hose's diameter, its
    mass plus the lines inside, the largest slack any of them brings — and
    every line in it reads that track. A span held on a robot collides
    with that robot's links — it rests on the arm instead of falling
    through it — except a holding link whose shape holds the held point
    (a clip seated in the casting, which would push its own held particle
    away). The slack's dip is hung on the side with the most room (a stiff
    line keeps the loop it is laid in), and a link the starting shape still
    runs through is left out with a warning (the report then names the
    contact). ``arm_links=False`` leaves the arm out
    altogether. ``settle_fixed=True`` replays the
    fixed spans too, so they hang under gravity instead of standing on
    their polyline. Every rope first settles for ``settle_s`` with its
    holders parked at time zero — it was laid along a design polyline, not
    a hanging shape — before the cycle runs. The ropes are computed side
    by side (they do not interact).

    The scene's current poses are time zero, as for ``bt.rope.animate``.
    Returns the new timeline; :func:`report` with it judges the spans. A
    line rests straight (``rest="straight"``: a tube or a cable off the
    reel, bent only by what holds it and by its weight — not the dip it is
    laid in) and is a rod, not a string (``rod_hz=120``: windows of a few
    particles each drawn toward their straight rest shape, so a bend
    anywhere is resisted — a tube holds a round loop between its holders
    instead of hanging in a V; ``bending_hz=120`` is the fold-resisting
    spring beside it). The fixed spans, settled only to hang, stay ropes.
    The stiffness is the solver's native setting, not the line's, and it
    is bounded by the step — past about twice this the replay tears:
    shape, clearance and slack are judged, tension is not.

    ``solver="static"`` draws the spans instead of simulating them (see
    :func:`botrail.rope.animate`): at every frame each span is the rod at
    rest between its holders — of least bending and gravitational energy,
    its length what the plan gives it, held by every holder as the replay
    holds it (a fixed port's stub and a run's clip in direction too; a
    moving port, a swivel fitting, and a declared clamp at their points),
    clear of every link of the arm and of the obstacles around — a harness
    configurator's picture, with no dynamics, settling or spring settings. ``rigidity_nm2`` is the flexural rigidity that balances the
    bend against the weight (``None``: a polyurethane tube's for each
    line's diameter). ``step_s``, ``settle_s``, ``rest``, ``rod_hz`` and the
    spring frequencies do not apply to it; the report judges its frames as
    it does the replay's.
    """
    if solver not in ("physics", "static"):
        raise ValueError(f'solver is "physics" or "static", not {solver!r}')
    from . import rope as _rope

    plan = _plan(scene)
    names = [r["name"] for r in plan["routes"]] if routes is None else list(routes)
    enabled = [n for n in scene.obstacle_names if scene.obstacle_enabled(n)]
    specs = {sp["name"]: sp for sp in plan["specs"]}
    supports = {sp["name"]: sp for sp in plan["supports"]}
    passes: list[dict] = []
    hoses: dict[str, dict] = {}
    for name in names:
        route = next((r for r in plan["routes"] if r["name"] == name), None)
        if route is None:
            raise ValueError(f"unknown route: {name}")
        g = geometry(scene, name)
        spec = specs.get(route["spec"]) or {}
        od = float(spec.get("od_m") or 0.008)
        mass = float(spec.get("mass_kg_m") or 0.1)
        moving = [sp for sp in g["spans"] if sp["kind"] == "moving"]
        share = float(route["slack_m"]) / len(moving) if moving else 0.0
        slacks = list(route.get("slacks_m") or [])
        ducts = set(g["ducts"])
        channels = _duct_channels(plan, g["ducts"])
        moving_seen = 0
        # The spans the line slides through a holder between are one
        # stretch: one rope, through a ring at each sliding holder, named
        # after the first of them, with their slack together.
        stretches: list[list[int]] = []
        for k, span in enumerate(g["spans"]):
            if span.get("slides") and stretches:
                stretches[-1].append(k)
            else:
                stretches.append([k])
        for members in stretches:
            k = members[0]
            span = g["spans"][k]
            slack = 0.0
            kinds = {g["spans"][m]["kind"] for m in members}
            for m in members:
                if g["spans"][m]["kind"] == "moving":
                    slack += slacks[moving_seen] if moving_seen < len(slacks) else (0.0 if slacks else share)
                    moving_seen += 1
            if kinds == {"fixed"} and not settle_fixed:
                continue
            if "moving" not in kinds and "dressed" not in kinds and not settle_fixed:
                continue
            stretch = {"from": span["from"], "to": g["spans"][members[-1]]["to"],
                       "from_s": span["from_s"], "to_s": g["spans"][members[-1]]["to_s"],
                       "kind": "moving" if "moving" in kinds else span["kind"]}
            points = _span_points(g, stretch)
            spacing = spacing_m or max(od, 0.01)
            if len(points) < 2 or (stretch["to_s"] - stretch["from_s"]) + slack < 2 * spacing:
                # Two clamps a hand's width apart hold the line as it is.
                continue
            span = stretch
            hose = _carrier_span(span)
            if hose is not None and f"{hose[0]}#{hose[1]}" in hoses:
                # Another line in the same stretch of hose: its mass and the
                # larger slack go to the one rope.
                entry = hoses[f"{hose[0]}#{hose[1]}"]
                entry["density_kg_m"] += mass
                entry["slack"] = max(entry["slack"], slack)
                entry["members"].append(name)
                continue
            anchors, pins, guides, skip, linked = [], [], [], set(ducts), set()
            # The sliding holders inside the stretch: rings on their
            # bodies the line runs through, as long as the holder, a
            # millimetre wider than the line.
            for m in members[1:]:
                anchor = next(a for a in g["anchors"] if a["name"] == g["spans"][m]["from"])
                at, held = _anchor_of(scene, plan, anchor, route)
                ring = dict(position=anchor["position"], axis=anchor.get("tangent") or [0.0, 0.0, 1.0],
                            length_m=max(2 * held, 0.01), inner_m=od + 0.002)
                if at["kind"] == "obstacle":
                    guides.append(dict(obstacle=at["name"], **ring))
                    skip.add(at["name"])
                elif at["kind"] == "link":
                    guides.append(dict(robot=at["robot"], link=at["link"], **ring))
                    linked.add((at["robot"], at["link"]))
            for end, anchor_name in (("Start", span["from"]), ("End", span["to"])):
                anchor = next(a for a in g["anchors"] if a["name"] == anchor_name)
                at, held = _anchor_of(scene, plan, anchor, route)
                holder = anchor.get("holder", "fixed")
                if anchor["kind"] == "port" and anchor["moving"]:
                    # A fitting on a hand that turns is a swivel elbow: it
                    # holds the tube at a point and turns with it. Held
                    # straight, a fitting swinging round the hand's axis
                    # drags the loop after it: the rope folds at the
                    # fitting, and the static rod lays the loop over the
                    # wrist it must turn back round.
                    held = 0.0
                elif anchor["kind"] == "clamp" and holder == "strap":
                    # A strap holds the point: the line turns and rolls in it.
                    held = 0.0
                elif anchor["kind"] == "clamp" and anchor.get("tangent"):
                    # A fixed or rotating holder holds the line's direction
                    # too (a declared one along its `direction`, else the
                    # bisector of its legs; a clip of a run along the run):
                    # the rope says so over one sample spacing at least,
                    # led that way. The roll a rotating holder leaves free
                    # is the static rod's to take (the rope has none).
                    held = max(held, spacing)
                    points = _led(points, end, anchor["tangent"], held)
                rail = anchor.get("rail")
                # The holder holds the line's roll too: a rigid clamp, a
                # retraction's carriage, a connector on a still body (a
                # fitting on a hand that turns is the swivel above).
                roll = held > 0.0 and (
                    (anchor["kind"] == "port" and not anchor["moving"])
                    or (anchor["kind"] == "clamp" and holder in ("fixed", "retracting")))
                if at["kind"] == "obstacle":
                    anchors.append(_rope.anchor(at["name"], location=end, length_m=held, rail=rail, roll=roll))
                    skip.add(at["name"])
                elif at["kind"] == "link":
                    anchors.append(_rope.anchor(robot=at["robot"], link=at["link"], location=end, length_m=held,
                                                rail=rail, roll=roll))
                    linked.add((at["robot"], at["link"]))
                else:
                    pins.append((end, points[0] if end == "Start" else points[-1]))
            lo = [min(p[i] for p in points) - margin_m for i in range(3)]
            hi = [max(p[i] for p in points) + margin_m for i in range(3)]
            obstacles = []
            for n in enabled:
                if n in skip or any(n.startswith(d + "/") for d in ducts):
                    continue
                bounds = scene.obstacle_bounds(n)
                if bounds is None:
                    continue
                (olo, ohi) = bounds
                if not all(ohi[i] >= lo[i] and olo[i] <= hi[i] for i in range(3)):
                    continue
                # A body the starting line already runs through would throw
                # the rope apart: leave it out, and say so.
                r = od / 2
                if any(all(olo[i] - r <= q[i] <= ohi[i] + r for i in range(3)) for q in points):
                    warnings.warn(f"route {name!r} span {k} starts inside `{n}`: the replay does not collide with it "
                                  "(the static report names the clash)", stacklevel=2)
                    continue
                obstacles.append(n)
            entry = dict(label=f"route {name!r} span {k}", name=f"route:{name}#{k}", points=points, slack=slack,
                         anchors=anchors, pins=pins, guides=guides, obstacles=obstacles, links=[],
                         fixed=span["kind"] == "fixed",
                         spacing_m=spacing, radius_m=od / 2, density_kg_m=mass,
                         color=g["color"], channels=channels, members=[name])
            if hose is not None:
                carrier = supports.get(hose[0]) or {}
                od_hose = float(carrier.get("od_m") or ((float(carrier["inner_m"]) + 0.004) if carrier.get("inner_m")
                                                        else 2 * od))
                entry.update(label=f"carrier {hose[0]!r} span {hose[1]}", name=f"carrier:{hose[0]}#{hose[1]}",
                             spacing_m=spacing_m or max(od_hose, 0.01), radius_m=od_hose / 2,
                             density_kg_m=mass + float(carrier.get("mass_kg_m") or 0.0))
                hoses[f"{hose[0]}#{hose[1]}"] = entry
            # The robot the span is held on: every link but a holding one
            # whose shape holds the held point (it would push its own held
            # particle away).
            if arm_links and linked:
                setup = json.loads(scene._span_replay_links_json(name, k, entry["radius_m"]))
                # The static solve holds its held samples fast, so a holding
                # link's shape cannot push them: it keeps every link.
                links_ = setup["collide"] + (setup["exempt"] if solver == "static" else [])
                entry["links"] = [{"robot": r, "link": l} for r, l in links_]
            passes.append(entry)

    # The starting shapes: the slack hung on a side clear of the arm and of
    # the obstacles around; a link the start still runs through is left out
    # (the rope would be thrown out of it), and the report names the contact.
    for entry in passes:
        r = entry["radius_m"]
        names_ = [[l["robot"], l["link"]] for l in entry["links"]]
        bounds = [b for b in (scene.obstacle_bounds(n) for n in entry["obstacles"]) if b is not None]

        def judged(points, r=r):
            return _interior(_densify(points, max(r, 0.01)), 2 * r + 0.01)

        def room(points, names_=names_, bounds=bounds, r=r):
            """How far the shape keeps from the arm and the obstacles around
            (the nearest of a few steps); below zero it starts inside one."""
            inner = judged(points)
            if not inner:
                return math.inf
            for margin in (0.08, 0.05, 0.03, 0.02, 0.01, 0.0):
                reach = r + margin
                if names_ and json.loads(scene._points_near_links_json(json.dumps(inner), reach, json.dumps(names_))):
                    continue
                if any(all(lo[i] - reach <= q[i] <= hi[i] + reach for i in range(3)) for (lo, hi) in bounds for q in inner):
                    continue
                return margin
            return -1.0

        entry["start"] = _with_slack(entry["points"], entry["slack"], room=room)
        if names_:
            inside = json.loads(scene._points_near_links_json(json.dumps(judged(entry["start"])), r, json.dumps(names_)))
            if inside:
                warnings.warn(f"{entry['label']} starts inside {', '.join(f'`{a}/{b}`' for a, b in inside)}: the replay "
                              "does not collide with it (the report names the contact)", stacklevel=2)
                entry["links"] = [l for l in entry["links"] if [l["robot"], l["link"]] not in inside]

    def payload(entry: dict, obstacles, links) -> dict:
        return dict(name=entry["name"], points=entry["start"], anchors=entry["anchors"], guides=entry["guides"],
                    pins=entry["pins"], obstacles=obstacles, collision_links=links, spacing_m=entry["spacing_m"],
                    radius_m=entry["radius_m"], density_kg_m=entry["density_kg_m"], step_s=step_s, axial_hz=axial_hz,
                    bending_hz=bending_hz, friction=friction, color=entry["color"], ground_z=ground_z,
                    preroll_s=settle_s, extra_colliders=entry["channels"], rest=rest,
                    rod_hz=0.0 if entry["fixed"] else rod_hz, solver=solver, rigidity_nm2=rigidity_nm2)

    # Geometry the replay cannot take (a degenerate hull piece) is dropped
    # rather than failing the route: first the arm's links, then the
    # obstacles, with a warning naming what the span no longer collides
    # with. Everything that can run runs in one batch; only what failed
    # goes round again.
    attempts = [lambda e: (e["obstacles"], e["links"]), lambda e: (e["obstacles"], []), lambda e: ([], [])]
    stage = [0] * len(passes)
    pending = list(range(len(passes)))
    out = timeline
    while pending:
        out, errors = _rope.animate_many(scene, out, [payload(passes[i], *attempts[stage[i]](passes[i])) for i in pending])
        again = []
        for i, err in zip(pending, errors):
            if err is None:
                continue
            stage[i] += 1
            if stage[i] >= len(attempts):
                raise ValueError(err)
            dropped = "the arm's links" if stage[i] == 1 else "the obstacles around it"
            warnings.warn(f"{passes[i]['label']}: replayed without {dropped} — {err}", stacklevel=2)
            again.append(i)
        pending = again
    return out


_COLUMNS = ["name", "kind", "medium", "spec", "manufacturer", "model", "catalog", "carries", "from_port",
            "from_target", "to_port", "to_target", "od_mm", "length_m", "fixed_m", "moving_m", "cut_length_m",
            "purchase_length_m", "ducts", "clamps", "auto_clamps", "carriers", "bends", "status", "findings"]


@dataclass
class CableList:
    """One row per route — the cable list a wiring design, purchasing or an
    ECAD import starts from. Renders as CSV, Markdown or JSON."""
    rows: list[dict]

    def to_dict(self) -> dict:
        return {"rows": self.rows}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False, allow_nan=False)

    def total(self, key: str = "cut_length_m") -> Optional[float]:
        values = [r[key] for r in self.rows if isinstance(r.get(key), (int, float))]
        return math.fsum(values) if values else None

    def to_csv(self) -> str:
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=_COLUMNS, lineterminator="\n")
        writer.writeheader()
        for row in self.rows:
            writer.writerow({k: ";".join(row[k]) if isinstance(row.get(k), list) else row.get(k) for k in _COLUMNS})
        return stream.getvalue()

    def to_markdown(self) -> str:
        lines = ["# Cable list\n", "| # | route | kind | medium | spec / model | from | to | cut m | purchase m | status |",
                 "|---|---|---|---|---|---|---|---|---|---|"]
        for i, r in enumerate(self.rows, 1):
            model = " / ".join(filter(None, (r.get("spec"), r.get("model"))))
            lines.append("| " + " | ".join(_cell(v) for v in (
                i, r["name"], r["kind"], r.get("medium"), model,
                f"{r.get('from_port') or ''} ({r.get('from_target') or ''})",
                f"{r.get('to_port') or ''} ({r.get('to_target') or ''})",
                r.get("cut_length_m"), r.get("purchase_length_m"), r["status"])) + " |")
        total = self.total()
        if total is not None:
            lines.append(f"\nTotal cut length: {total:.2f} m")
        return "\n".join(lines) + "\n"

    def save(self, path) -> None:
        path = Path(path)
        if path.suffix == ".csv":
            path.write_text(self.to_csv(), encoding="utf-8")
        elif path.suffix == ".md":
            path.write_text(self.to_markdown(), encoding="utf-8")
        elif path.suffix == ".json":
            path.write_text(self.to_json(), encoding="utf-8")
        else:
            raise ValueError("cable list extension must be .csv, .md or .json")


def cable_list(scene) -> CableList:
    """The cable list: every route with its line, ends, lengths and status."""
    rows = []
    for r in report(scene).routes:
        row = {k: r.get(k) for k in _COLUMNS if k in r}
        row["kind"] = r.get("kind")
        for end in ("from", "to"):
            endpoint = r.get(end) or {}
            row[f"{end}_port"] = endpoint.get("port")
            row[f"{end}_target"] = endpoint.get("target")
        rows.append({k: row.get(k) for k in _COLUMNS})
    return CableList(rows)
