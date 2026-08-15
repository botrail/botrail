"""Spray applicators: the footprint a gun lays down, and how much paint
it delivers.

An applicator is a plain dict describing a *calibrated footprint* — the
film profile measured on a plane at a known standoff — plus the flow that
fills it. ``timeline.spray_coat(target, applicator)`` projects that
footprint onto the target along the spray axis and integrates it over the
baked cycle; see ``design/design-painting.md``.

This is geometry, not fluid dynamics. There is no air flow and no
electrostatic field here, so the wrap an ESTA bell gets around an edge is
**not** modeled and absolute microns are only as good as the pattern you
feed in. :func:`from_profile` is the honest entry point: it takes the
static-pattern coupon a shop already has and derives both the shape and
the delivery rate from it. :func:`fan` and :func:`bell` are the
literature's analytic fits, useful before anyone has sprayed anything.

Units are SI throughout, like the rest of botrail: meters, m^3/s. A
pattern sheet in mm and microns needs converting on the way in.

The tool-frame convention matches the toolpath solver's: the TCP's local
``+Z`` runs from the nozzle tip toward the gun body, so paint travels
along ``-Z``. A fan's width lies along the TCP's local ``+X``.
"""

from __future__ import annotations

import math
import os
from typing import Iterable, Sequence

from . import toolpath as _toolpath

__all__ = [
    "Pattern",
    "applicator",
    "bell",
    "fan",
    "from_profile",
    "strokes",
    "wrap_strokes",
]


class Pattern(dict):
    """A footprint shape. A plain dict (so it serializes and diffs like
    one), plus what a measurement knows that an analytic fit does not:
    the standoff it was taken at and the volume rate it implies."""

    def __init__(
        self,
        data: dict,
        *,
        standoff: float | None = None,
        deposition_rate: float | None = None,
    ):
        super().__init__(data)
        #: Reference standoff the pattern is valid at [m], if known.
        self.standoff = standoff
        #: Paint reaching the reference plane [m^3/s], if known.
        self.deposition_rate = deposition_rate


def fan(
    width: float,
    height: float,
    *,
    beta_across: float = 2.0,
    beta_along: float = 2.0,
) -> Pattern:
    """Elliptic dual-beta — the standard fit for a flat-fan air gun.

    ``width`` is the full fan across (along the TCP's ``+X``), ``height``
    the full extent along it at mid-fan; both in meters. The footprint is
    an ellipse: the along-extent narrows toward the ends of the fan. Beta
    shapes each axis — ``1`` is a flat top hat, ``2`` parabolic, larger
    more peaked. Below ``1`` is rejected (the profile would diverge at the
    footprint edge, which no measured pattern does).

    A fan is *not* rotationally symmetric, so the spin about the tool axis
    matters: the fan must run across the direction of travel. Authored
    paths must pin the spin; the 5-DOF solver's free spin is for
    :func:`bell`.
    """
    if not (width > 0.0 and height > 0.0):
        raise ValueError(f"fan width and height must be positive, got {width}x{height}")
    _check_beta(beta_across, "beta_across")
    _check_beta(beta_along, "beta_along")
    return Pattern(
        {
            "kind": "dual_beta",
            "width": float(width),
            "height": float(height),
            "beta_across": float(beta_across),
            "beta_along": float(beta_along),
        }
    )


def bell(diameter: float, *, beta: float = 2.0) -> Pattern:
    """Axisymmetric cone — a rotary atomizing bell or a round nozzle.

    ``diameter`` is the full footprint on the reference plane [m]; ``beta``
    shapes it (``1`` a flat disc, ``2`` parabolic). Being rotationally
    symmetric, this is the best case for the solver: the spin about the
    tool axis is genuinely free, so ``IkMode::Axis`` gets its full 5-DOF
    room. It is also the automotive mainstream, which is a pleasant
    coincidence.
    """
    if not (diameter > 0.0):
        raise ValueError(f"pattern diameter must be positive, got {diameter}")
    _check_beta(beta, "beta")
    return Pattern({"kind": "round", "diameter": float(diameter), "beta": float(beta)})


def _check_beta(value: float, name: str) -> None:
    """Below 1 the profile diverges at the footprint edge, which no
    measured pattern does — and it would make the normalization integral
    depend on the quadrature resolution rather than on the gun."""
    if not (value >= 1.0):
        raise ValueError(f"{name} must be >= 1 (1 is flat), got {value}")


def from_profile(
    source: str | os.PathLike | Iterable[Sequence[float]],
    *,
    standoff: float,
    seconds: float,
) -> Pattern:
    """A measured static-pattern profile: the coupon a shop sprays to
    characterize a gun.

    ``source`` is a two-column CSV (``radius, film``) or any iterable of
    ``(radius, film)`` pairs — radii ascending from the pattern center,
    **both in meters**, being the film left after spraying ``seconds`` at
    ``standoff``. A header row is skipped if the first row will not parse
    as numbers.

    The measurement carries both things the model needs: the shape, and —
    by integrating it — the volume rate the gun actually delivers to the
    plane. :func:`applicator` picks both up, so a calibrated gun needs no
    flow figure guessed at.
    """
    if not (seconds > 0.0):
        raise ValueError(f"seconds must be positive, got {seconds}")
    if not (standoff > 0.0):
        raise ValueError(f"standoff must be positive, got {standoff}")

    rows = _rows(source)
    radii = [r for r, _ in rows]
    weight = [f for _, f in rows]
    if len(rows) < 2:
        raise ValueError("a measured profile needs at least 2 samples")
    if any(b <= a for a, b in zip(radii, radii[1:])) or radii[0] < 0.0:
        raise ValueError("profile radii must be non-negative and strictly ascending")
    if any(f < 0.0 for f in weight) or not any(f > 0.0 for f in weight):
        raise ValueError("profile film values must be non-negative and not all zero")

    # Volume under the axisymmetric profile, trapezoid on r*f(r): what the
    # gun put on the plane, divided by how long it took. Any radius the
    # profile does not reach is outside the footprint and contributes
    # nothing, which is the same convention the integrator uses.
    volume = 0.0
    prev_r, prev_f = 0.0, weight[0]
    for r, f in rows:
        volume += math.pi * (r - prev_r) * (prev_r * prev_f + r * f)
        prev_r, prev_f = r, f
    return Pattern(
        {"kind": "measured", "radii": radii, "weight": weight},
        standoff=standoff,
        deposition_rate=volume / seconds,
    )


def applicator(
    pattern: Pattern | dict,
    *,
    standoff: float | None = None,
    flow: float | None = None,
    transfer_efficiency: float = 0.65,
    max_range: float | None = None,
) -> dict:
    """Assembles an applicator for ``timeline.spray_coat``.

    ``standoff`` is the distance the pattern is valid at [m] — the whole
    model is a projection of that plane, so the authored gun distance
    should match it. ``flow`` is delivery at the nozzle [m^3/s] and
    ``transfer_efficiency`` the fraction of it that reaches the plane
    (roughly 0.3-0.5 for air spray, 0.8-0.95 for an electrostatic bell);
    their product is what lands. A :func:`from_profile` pattern supplies
    both ``standoff`` and the landed rate, so ``flow`` can be left out and
    is then back-computed from ``transfer_efficiency``.

    ``max_range`` is where the model gives up [m]; it defaults to two and
    a half times the standoff. Nothing lands past it, and inside a fifth
    of the standoff nothing is deposited either — that stretch is reported
    as ``too_close_time`` rather than silently trusted, because the
    inverse square there is well outside what the coupon measured.
    """
    standoff = standoff if standoff is not None else getattr(pattern, "standoff", None)
    if standoff is None:
        raise ValueError(
            "standoff is required (or use a from_profile() pattern, which carries it)"
        )
    if not (0.0 < transfer_efficiency <= 1.0):
        raise ValueError(
            f"transfer_efficiency must be in (0, 1], got {transfer_efficiency}"
        )
    if flow is None:
        rate = getattr(pattern, "deposition_rate", None)
        if rate is None:
            raise ValueError(
                "flow is required for an analytic pattern; a from_profile() pattern "
                "derives it from the measurement"
            )
        flow = rate / transfer_efficiency
    return {
        "standoff": float(standoff),
        "pattern": dict(pattern),
        "flow": float(flow),
        "transfer_efficiency": float(transfer_efficiency),
        "max_range": float(max_range if max_range is not None else standoff * 2.5),
    }


def _rows(source) -> list[tuple[float, float]]:
    """Reads (radius, film) pairs from a CSV path, CSV text, or an
    iterable of pairs."""
    if isinstance(source, (str, os.PathLike)):
        text = os.fspath(source)
        if "\n" not in text and os.path.exists(text):
            with open(text, "r", encoding="utf-8") as f:
                text = f.read()
        lines = [ln.strip() for ln in text.splitlines()]
        pairs = []
        for line in lines:
            if not line or line.startswith("#"):
                continue
            parts = [p for p in line.replace(",", " ").split() if p]
            if len(parts) < 2:
                raise ValueError(f"profile row needs two columns, got {line!r}")
            try:
                pairs.append((float(parts[0]), float(parts[1])))
            except ValueError:
                if pairs:
                    raise ValueError(f"unparsable profile row {line!r}") from None
                continue  # a header row, before any data
        return pairs
    return [(float(r), float(f)) for r, f in source]


# ---------------------------------------------------------------- strokes


def strokes(
    size: Sequence[float],
    *,
    standoff: float,
    pattern_width: float,
    overlap: float,
    speed: float,
    overtravel: float,
    margin: float | None = None,
    center: Sequence[float] = (0.0, 0.0),
    height: float = 0.0,
    direction: str = "x",
    frame: str | None = None,
    spin: float | str | None = None,
    brush: str | None = None,
    trigger: str = "stroke",
) -> _toolpath.Toolpath:
    """A serpentine raster over a flat area: laps at ``pattern_width x
    (1 - overlap)`` pitch, each running the area's length plus
    ``overtravel`` at both ends, one standoff above it.

    Painting has no CAM: there is no G-code to import, so the path is
    generated from the surface and the shop's rules — pattern width, lap
    overlap, gun speed, standoff — the way an OLP package's paint module
    does. This is the plane version; :func:`wrap_strokes` is the cylinder
    one. Free-form surfaces are out of scope (design/design-painting.md
    §5): that is CAM.

    ``size`` is ``(length_x, length_y)`` of the area in the part frame,
    ``center`` its middle and ``height`` its ``z``; laps run along
    ``direction`` (``"x"`` or ``"y"``) and step across the other axis.
    ``margin`` is how far the lap *set* reaches past the area's edge across
    the laps (default half a pattern width, so the edge lap's centre sits
    on the edge). Whole laps at exactly the requested pitch, centred on the
    area: covering a little extra beats rounding the pitch, which would
    blur exactly what an overlap comparison is about.

    Overtravel matters more than it looks: the path rests only at its two
    ends, but the gun still turns around between laps and slows doing so,
    laying on extra paint. Push the turnaround a full pattern radius past
    the area and that build-up lands off the part.

    ``spin`` pins the rotation about the tool axis. Leave it ``None`` for a
    bell (rotationally symmetric, so the 5-DOF solver keeps the spin free);
    pass ``"fan"`` for a flat-fan gun to run the fan across the direction
    of travel, or an angle in radians to set it yourself.

    ``brush`` names the process setting the laps run with (a
    ``scene.define_brush`` name) and makes the raster trigger per stroke:
    with ``trigger="stroke"`` (the default) the laps carry the brush and
    the side-steps between them run at speed with the gun off, so the
    turnarounds stop spraying the floor; ``trigger="continuous"`` keeps
    the gun open through the side-steps too. Without a brush the raster
    is one continuous feed move sprayed with whatever applicator
    ``spray_coat`` is handed — the shape of a program with no per-stroke
    trigger.
    """
    lx, ly = float(size[0]), float(size[1])
    if not (lx > 0.0 and ly > 0.0):
        raise ValueError(f"size must be two positive lengths, got {size!r}")
    pitch = _pitch(pattern_width, overlap)
    _check_positive(standoff, "standoff")
    _check_positive(speed, "speed")
    if overtravel < 0.0:
        raise ValueError(f"overtravel must be >= 0, got {overtravel}")
    margin = pattern_width / 2.0 if margin is None else float(margin)
    if direction not in ("x", "y"):
        raise ValueError(f'direction must be "x" or "y", got {direction!r}')

    along, across = (lx, ly) if direction == "x" else (ly, lx)
    cx, cy = float(center[0]), float(center[1])
    half_along = along / 2.0 + overtravel
    half_across = across / 2.0 + margin
    gaps = max(1, math.ceil(2.0 * half_across / pitch))
    offsets = [-(gaps * pitch) / 2.0 + pitch * k for k in range(gaps + 1)]

    if direction == "x":
        travel = (1.0, 0.0, 0.0)
        point = lambda a, c: (cx + a, cy + c, height)  # noqa: E731
    else:
        travel = (0.0, 1.0, 0.0)
        point = lambda a, c: (cx + c, cy + a, height)  # noqa: E731
    up = (0.0, 0.0, 1.0)
    spin_value = _resolve_spin(spin, up, travel)

    stroke_brush, step_brush = _trigger_brushes(brush, trigger)
    tp = _toolpath.builder(frame=frame)
    tp.rapid_to(_lift(point(-half_along, offsets[0]), up, standoff), spin=spin_value)
    for i, c in enumerate(offsets):
        a0, a1 = (-half_along, half_along) if i % 2 == 0 else (half_along, -half_along)
        # The lap: gun on (with the brush, if any).
        tp.feed(speed, brush=stroke_brush)
        tp.line_to(_lift(point(a0, c), up, standoff), spin=spin_value)
        tp.line_to(_lift(point(a1, c), up, standoff), spin=spin_value)
        # The side-step to the next lap: gun as the trigger mode says.
        if i + 1 < len(offsets):
            tp.feed(speed, brush=step_brush)
            tp.line_to(_lift(point(a1, offsets[i + 1]), up, standoff), spin=spin_value)
    return tp.build()


def wrap_strokes(
    radius: float,
    length: float,
    *,
    standoff: float,
    pattern_width: float,
    overlap: float,
    speed: float,
    overtravel: float,
    arc: Sequence[float] = (-math.pi / 2, math.pi / 2),
    margin: float | None = None,
    center: Sequence[float] = (0.0, 0.0, 0.0),
    axis: str = "y",
    frame: str | None = None,
    spin: float | str | None = None,
    brush: str | None = None,
    trigger: str = "stroke",
) -> _toolpath.Toolpath:
    """A serpentine raster wrapped onto a cylinder: laps run along the
    cylinder's axis, one standoff off its surface, and step around it in
    angle so the pitch measured *on the surface* is ``pattern_width x
    (1 - overlap)``. The tool axis is radial at every point — the gun
    stays square on and at constant standoff over the whole arc, which is
    what a flat raster over a curved part cannot do (its edges drift far
    and oblique; :meth:`Scene.check_paint` will say so).

    ``radius`` and ``length`` describe the surface; ``center`` is the
    cylinder's centre in the part frame and ``axis`` (``"x"`` or ``"y"``)
    its direction. ``arc`` is the angular sector to cover, in radians
    measured in the plane across the axis from ``+z`` toward the third
    axis (so ``0`` is the top of a horizontal cylinder and ``(-a, a)`` a
    cap of half-angle ``a`` around it). ``margin`` extends the lap set past
    the sector, along the surface (default half a pattern width). The
    other arguments — ``spin``, ``brush``, ``trigger`` — are as in
    :func:`strokes`.
    """
    _check_positive(radius, "radius")
    _check_positive(length, "length")
    _check_positive(standoff, "standoff")
    _check_positive(speed, "speed")
    if overtravel < 0.0:
        raise ValueError(f"overtravel must be >= 0, got {overtravel}")
    pitch = _pitch(pattern_width, overlap)
    margin = pattern_width / 2.0 if margin is None else float(margin)
    if axis not in ("x", "y"):
        raise ValueError(f'axis must be "x" or "y", got {axis!r}')
    a0, a1 = float(arc[0]), float(arc[1])
    if not (a1 > a0):
        raise ValueError(f"arc must be (start, end) with start < end, got {arc!r}")

    # Angular pitch for the surface pitch, and whole laps centred on the
    # sector — the same rounding rule as the flat raster.
    dtheta = pitch / radius
    span = (a1 - a0) + 2.0 * margin / radius
    gaps = max(1, math.ceil(span / dtheta))
    mid = (a0 + a1) / 2.0
    angles = [mid - (gaps * dtheta) / 2.0 + dtheta * k for k in range(gaps + 1)]
    half_along = length / 2.0 + overtravel
    cx, cy, cz = (float(v) for v in center)

    def radial(theta: float) -> tuple[float, float, float]:
        # Across the axis, from +z toward the third axis.
        s, c = math.sin(theta), math.cos(theta)
        return (s, 0.0, c) if axis == "y" else (0.0, s, c)

    travel = (0.0, 1.0, 0.0) if axis == "y" else (1.0, 0.0, 0.0)

    def point(along: float, theta: float) -> tuple[float, float, float]:
        n = radial(theta)
        r = radius + standoff
        if axis == "y":
            return (cx + r * n[0], cy + along, cz + r * n[2])
        return (cx + along, cy + r * n[1], cz + r * n[2])

    stroke_brush, step_brush = _trigger_brushes(brush, trigger)
    tp = _toolpath.builder(frame=frame)
    n0 = radial(angles[0])
    tp.rapid_to(point(-half_along, angles[0]), axis=n0, spin=_resolve_spin(spin, n0, travel))
    for i, theta in enumerate(angles):
        n = radial(theta)
        sp = _resolve_spin(spin, n, travel)
        s0, s1 = (-half_along, half_along) if i % 2 == 0 else (half_along, -half_along)
        tp.feed(speed, brush=stroke_brush)
        tp.line_to(point(s0, theta), axis=n, spin=sp)
        tp.line_to(point(s1, theta), axis=n, spin=sp)
        if i + 1 < len(angles):
            n1 = radial(angles[i + 1])
            tp.feed(speed, brush=step_brush)
            tp.line_to(point(s1, angles[i + 1]), axis=n1, spin=_resolve_spin(spin, n1, travel))
    return tp.build()


def _trigger_brushes(brush: str | None, trigger: str) -> tuple[str | None, str | None]:
    """The brush a lap carries and the brush a side-step carries."""
    if trigger not in ("stroke", "continuous"):
        raise ValueError(f'trigger must be "stroke" or "continuous", got {trigger!r}')
    if brush is None:
        if trigger != "stroke":
            raise ValueError("trigger needs a brush: pass brush=... to trigger per stroke")
        return None, None
    return brush, (brush if trigger == "continuous" else None)


def _pitch(pattern_width: float, overlap: float) -> float:
    _check_positive(pattern_width, "pattern_width")
    if not (0.0 <= overlap < 1.0):
        raise ValueError(f"overlap must be in [0, 1), got {overlap}")
    return pattern_width * (1.0 - overlap)


def _check_positive(value: float, name: str) -> None:
    if not (value > 0.0):
        raise ValueError(f"{name} must be positive, got {value}")


def _lift(p: Sequence[float], axis: Sequence[float], standoff: float) -> tuple[float, float, float]:
    return (p[0] + axis[0] * standoff, p[1] + axis[1] * standoff, p[2] + axis[2] * standoff)


def _resolve_spin(spin, axis: Sequence[float], travel: Sequence[float]) -> float | None:
    """``None`` stays free; ``"fan"`` puts the TCP's local ``+X`` (the fan's
    width) across the direction of travel; a number is used as is."""
    if spin is None:
        return None
    if isinstance(spin, str):
        if spin != "fan":
            raise ValueError(f'spin must be None, "fan", or an angle, got {spin!r}')
        # The fan lies along local +X; across travel means +X ⟂ travel and
        # ⟂ the axis, i.e. along axis × travel. A fan is the same fan
        # turned half a turn, so fold the answer into (-90°, 90°] — the
        # smaller wrist motion of two equivalent ones.
        want = _cross(axis, travel)
        angle = _spin_from_reference(axis, want)
        while angle > math.pi / 2:
            angle -= math.pi
        while angle <= -math.pi / 2:
            angle += math.pi
        return angle
    return float(spin)


def _spin_from_reference(axis: Sequence[float], want: Sequence[float]) -> float:
    """The spin angle that puts local ``+X`` along ``want`` — the signed
    angle from the solver's spin reference (the world basis vector least
    aligned with the axis, projected onto the plane across it) to ``want``,
    about ``axis``. Mirrors ``toolpath::spin_reference`` in the engine."""
    a = _normalize(axis)
    pick = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
    ref = _normalize(_reject(pick, a))
    w = _normalize(_reject(want, a))
    cos = max(-1.0, min(1.0, _dot(ref, w)))
    sin = _dot(_cross(ref, w), a)
    return math.atan2(sin, cos)


def _dot(u, v) -> float:
    return u[0] * v[0] + u[1] * v[1] + u[2] * v[2]


def _cross(u, v) -> tuple[float, float, float]:
    return (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])


def _reject(v, a) -> tuple[float, float, float]:
    d = _dot(v, a)
    return (v[0] - a[0] * d, v[1] - a[1] * d, v[2] - a[2] * d)


def _normalize(v) -> tuple[float, float, float]:
    n = math.sqrt(_dot(v, v))
    if n < 1e-12:
        raise ValueError("degenerate direction")
    return (v[0] / n, v[1] / n, v[2] / n)
