"""Toolpath authoring: a builder for programmatic paths and a G-code
importer. A toolpath is a continuous Cartesian process path (milling,
trimming, deburring) followed at a commanded feed rate — see
``design/design-machining.md``.

Every position is in the part frame (meters); the tool axis points from
the cutter tip toward the tool body (``+Z`` for 3-axis work). Feed rates
are m/s. Register the result with ``scene.add_toolpath(name, tp)``, then
``scene.check_toolpath(name)`` for a face diagnosis or
``scene.plan_toolpath(name)`` to bake the trajectory.
"""

from __future__ import annotations

import json as _json
import math
import os
from typing import Sequence

from . import _core

__all__ = ["Toolpath", "ToolpathBuilder", "builder", "from_apt", "from_gcode"]

_Vec = Sequence[float]


class Toolpath(dict):
    """An authored toolpath: ``{"frame": ..., "moves": [...]}``.

    A plain dict (so it serializes and diffs like one) plus the parser
    warnings that produced it, if any.
    """

    def __init__(self, data: dict, warnings: list[str] | None = None):
        super().__init__(data)
        self.warnings: list[str] = list(warnings or [])

    @property
    def target_count(self) -> int:
        return sum(len(m["targets"]) for m in self["moves"])


def _target(position: _Vec, axis: _Vec | None, spin: float | None) -> dict:
    if len(position) != 3:
        raise ValueError(f"position must have 3 components, got {position!r}")
    t: dict = {"position": [float(v) for v in position]}
    if axis is not None:
        if len(axis) != 3 or all(abs(v) < 1e-12 for v in axis):
            raise ValueError(f"axis must be a non-zero 3-vector, got {axis!r}")
        t["tool_axis"] = [float(v) for v in axis]
    if spin is not None:
        t["spin"] = float(spin)
    return t


class ToolpathBuilder:
    """Accumulates rapid/feed targets into a toolpath.

    Consecutive targets of the same kind (and feed) merge into one move;
    a rapid, a feed change, or ``feed()`` after rapids starts a new one.
    """

    def __init__(self, frame: str | None = None):
        self._frame = frame
        self._moves: list[dict] = []
        self._feed: float | None = None
        self._brush: str | None = None

    def feed(self, feed: float, brush: str | None = None) -> "ToolpathBuilder":
        """Sets the process feed rate (m/s) for subsequent ``line_to`` /
        ``arc_to`` calls, and — for a spray program — the *brush* they run
        with: a named applicator + flow + trigger timing declared with
        ``scene.define_brush``. Brushes are the program's own trigger, per
        stroke: once any move in the path names one, a feed move without
        one runs with the gun off (a turnaround at speed), so pass the
        brush again on every ``feed()`` that sprays. A path that names no
        brush sprays every feed move with the applicator handed to
        ``spray_coat``."""
        if not (feed > 0.0):
            raise ValueError(f"feed must be positive, got {feed}")
        self._feed = float(feed)
        self._brush = None if brush is None else str(brush)
        return self

    def rapid_to(
        self, position: _Vec, axis: _Vec | None = None, spin: float | None = None
    ) -> "ToolpathBuilder":
        """Non-cutting reposition (timed by joint limits)."""
        self._push("rapid", None, [_target(position, axis, spin)])
        return self

    def line_to(
        self, position: _Vec, axis: _Vec | None = None, spin: float | None = None
    ) -> "ToolpathBuilder":
        """Straight cutting move at the current feed."""
        if self._feed is None:
            raise ValueError("call feed() before line_to()")
        self._push("feed", self._feed, [_target(position, axis, spin)], self._brush)
        return self

    def arc_to(
        self,
        position: _Vec,
        center: _Vec,
        normal: _Vec = (0.0, 0.0, 1.0),
        cw: bool = False,
        chord_tol: float = 1e-4,
    ) -> "ToolpathBuilder":
        """Cutting arc from the previous target to ``position`` about
        ``center``, swept in the plane perpendicular to ``normal`` (CCW
        about it; ``cw=True`` for the other way). An off-plane component
        between the endpoints interpolates linearly (helix). Tessellated
        at ``chord_tol`` into line targets."""
        if self._feed is None:
            raise ValueError("call feed() before arc_to()")
        start = self._last_position()
        if start is None:
            raise ValueError("arc_to() needs a previous target as the arc start")
        end = [float(v) for v in position]
        c = [float(v) for v in center]
        n = [float(v) for v in normal]
        nn = math.sqrt(sum(v * v for v in n))
        if nn < 1e-12:
            raise ValueError("normal must be non-zero")
        n = [v / nn for v in n]
        if cw:
            n = [-v for v in n]
        # In-plane basis (u toward the start point).
        rs = [s - cc for s, cc in zip(start, c)]
        re = [e - cc for e, cc in zip(end, c)]
        axial_s = sum(a * b for a, b in zip(rs, n))
        axial_e = sum(a * b for a, b in zip(re, n))
        us = [a - axial_s * b for a, b in zip(rs, n)]
        ue = [a - axial_e * b for a, b in zip(re, n)]
        r0 = math.sqrt(sum(v * v for v in us))
        r1 = math.sqrt(sum(v * v for v in ue))
        if r0 < 1e-9 or r1 < 1e-9:
            raise ValueError("arc endpoints must be off the rotation axis")
        u = [v / r0 for v in us]
        v = [
            n[1] * u[2] - n[2] * u[1],
            n[2] * u[0] - n[0] * u[2],
            n[0] * u[1] - n[1] * u[0],
        ]
        sweep = math.atan2(
            sum(a * b for a, b in zip(ue, v)), sum(a * b for a, b in zip(ue, u))
        ) % (2.0 * math.pi)
        if sweep < 1e-9:
            raise ValueError(
                "zero-sweep arc (full circles: author two half arcs)"
            )
        radius = max(r0, r1)
        alpha = (
            2.0 * math.acos(max(-1.0, 1.0 - chord_tol / radius))
            if radius > chord_tol
            else math.pi / 2
        )
        steps = max(1, math.ceil(sweep / min(alpha, 0.5)))
        targets = []
        for k in range(1, steps + 1):
            f = k / steps
            ang = sweep * f
            r = r0 + (r1 - r0) * f
            axial = axial_s + (axial_e - axial_s) * f
            p = [
                c[i] + r * (math.cos(ang) * u[i] + math.sin(ang) * v[i]) + axial * n[i]
                for i in range(3)
            ]
            if k == steps:
                p = end
            targets.append(_target(p, None, None))
        self._push("feed", self._feed, targets, self._brush)
        return self

    def build(self) -> Toolpath:
        if not self._moves:
            raise ValueError("empty toolpath: add rapid_to()/line_to() targets first")
        data: dict = {"moves": self._moves}
        if self._frame is not None:
            data["frame"] = self._frame
        return Toolpath(data)

    def _push(
        self,
        kind: str,
        feed: float | None,
        targets: list[dict],
        brush: str | None = None,
    ) -> None:
        last = self._moves[-1] if self._moves else None
        if (
            last is not None
            and last["type"] == kind
            and last.get("feed") == feed
            and last.get("brush") == brush
        ):
            last["targets"].extend(targets)
            return
        move: dict = {"type": kind, "targets": targets}
        if feed is not None:
            move["feed"] = feed
        if brush is not None:
            move["brush"] = brush
        self._moves.append(move)

    def _last_position(self) -> list[float] | None:
        for move in reversed(self._moves):
            if move["targets"]:
                return move["targets"][-1]["position"]
        return None


def builder(frame: str | None = None) -> ToolpathBuilder:
    """A fresh :class:`ToolpathBuilder`, optionally bound to a part frame
    (a ``scene.add_frame`` name; targets are then relative to it and the
    path re-solves when the frame moves)."""
    return ToolpathBuilder(frame)


def from_gcode(
    source: str | os.PathLike,
    *,
    frame: str | None = None,
    chord_tol: float = 1e-4,
) -> Toolpath:
    """Parses a G-code subset (G0/1/2/3, G17-19, G20/21, G90/91, F/G94)
    into a toolpath. ``source`` is a path to an ``.nc`` file, or the
    program text itself. Coordinates come out in meters in the part
    frame; the tool axis is the frame's ``+Z`` (3-axis semantics).

    Harmless spindle/coolant words land in ``Toolpath.warnings``; codes
    that would change the path's meaning (G41/42, T/M6, G95, G4, canned
    cycles) raise ``ValueError`` with the line number.
    """
    text = os.fspath(source)
    if "\n" not in text and os.path.exists(text):
        with open(text, "r", encoding="utf-8") as f:
            text = f.read()
    parsed = _json.loads(_core._parse_gcode_json(text, chord_tol))
    data: dict = {"moves": parsed["moves"]}
    if frame is not None:
        data["frame"] = frame
    return Toolpath(data, parsed["warnings"])


def from_apt(
    source: str | os.PathLike,
    *,
    frame: str | None = None,
) -> Toolpath:
    """Parses an APT/CL subset — the 5-axis entry format: ``GOTO`` records
    carry the tool axis as a plain ``i,j,k`` vector, machine-independent
    where 5-axis G-code would need the machine's kinematics. ``source`` is
    a path to a ``.apt``/``.cls`` file, or the program text itself.

    Supported: ``GOTO``/``FROM`` (3 or 6 values), ``RAPID`` (arms the next
    GOTO), ``FEDRAT`` (per-minute), ``UNITS``, ``$`` continuations, ``$$``
    comments, ``FINI``. ``SPINDL``/``COOLNT``/first ``LOADTL`` land in
    ``Toolpath.warnings``; ``CUTCOM``, ``CIRCLE``, ``GOHOME``, and a
    second ``LOADTL`` raise ``ValueError`` with the line number.
    """
    text = os.fspath(source)
    if "\n" not in text and os.path.exists(text):
        with open(text, "r", encoding="utf-8") as f:
            text = f.read()
    parsed = _json.loads(_core._parse_apt_json(text))
    data: dict = {"moves": parsed["moves"]}
    if frame is not None:
        data["frame"] = frame
    return Toolpath(data, parsed["warnings"])
