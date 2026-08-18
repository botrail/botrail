"""Traces from the real controller, and the diff against the bake — offline
commissioning without an online link.

botrail never closes a loop with a running PLC (that would cost the
determinism everything else stands on). What it does instead: the cell
hands its logic and its I/O list to the controller, the controller runs,
its I/O log comes back as a trace, and the trace is *compared* with the
timeline the cell baked — edge by edge, by name. Where they disagree is
where the design and the machine differ: a sensor that never fired, a
handshake that came late, a coil that switched twice.

    trace = bt.trace.load("plc_log.csv", io=scene.io_map())   # tags → point names
    d = tl.diff(trace, tolerance=0.05, align_on="beam_pick")
    print(d.to_markdown()); assert d.ok

A trace is `{signal name: [(t, value), ...]}` — a CSV with `t,name,value`
columns (`time`/`signal`/`tag`/`state` are accepted too; values `1/0`,
`true/false`, `on/off`, `high/low`) or a dict built any other way. Only
signals present on both sides are compared; the rest are listed, not
judged.
"""

from __future__ import annotations

import csv
import io as _io
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable, Optional, Union

Edge = tuple[float, bool]

_TRUE = {"1", "true", "on", "high", "t", "yes"}
_FALSE = {"0", "false", "off", "low", "f", "no"}


def _parse_value(text: str) -> bool:
    v = text.strip().lower()
    if v in _TRUE:
        return True
    if v in _FALSE:
        return False
    try:
        return float(v) != 0.0
    except ValueError as e:
        raise ValueError(f"trace: cannot read {text!r} as a boolean value") from e


@dataclass
class Trace:
    """Recorded signal levels by name: `(t, value)` samples in time order.
    Consecutive equal values are harmless — edges are what the diff reads."""

    signals: dict[str, list[Edge]] = field(default_factory=dict)

    @property
    def names(self) -> list[str]:
        return sorted(self.signals)

    @property
    def duration(self) -> float:
        return max((s[-1][0] for s in self.signals.values() if s), default=0.0)

    def edges(self, name: str) -> tuple[list[float], list[float]]:
        """`(rising, falling)` edge times of `name` — the first sample is
        the initial level, not an edge."""
        rising: list[float] = []
        falling: list[float] = []
        samples = sorted(self.signals.get(name, []))
        for (t0, v0), (t1, v1) in zip(samples, samples[1:]):
            if v1 and not v0:
                rising.append(t1)
            elif v0 and not v1:
                falling.append(t1)
        return rising, falling

    def shifted(self, dt: float) -> "Trace":
        """The same trace with every time moved by `dt`."""
        return Trace({k: [(t + dt, v) for t, v in s] for k, s in self.signals.items()})

    def renamed(self, mapping: dict[str, str]) -> "Trace":
        """Signals renamed through `mapping` (tag → point name); names not
        in the mapping stay."""
        return Trace({mapping.get(k, k): s for k, s in self.signals.items()})


def _tag_map(io) -> dict[str, str]:
    """`{tag or field name: point label}` from an IoMap (its JSON carries
    both) — a controller log keys its columns by the symbolic tag or by the
    field device on the drawing (`BEAM1`), and either resolves."""
    try:
        doc = json.loads(io.to_json())
    except (AttributeError, ValueError):
        return {}
    out: dict[str, str] = {}
    for b in doc.get("bindings", []):
        point = b.get("point", {})
        name = point.get("name")
        aspect = point.get("aspect")
        if not name:
            continue
        label = f"{name}.{aspect}" if aspect else name
        for key in (b.get("tag"), b.get("field")):
            if key and key not in out:
                out[key] = label
    return out


def load(source: Union[str, Path, dict, "Trace"], *, io=None, t0: Optional[float] = None) -> Trace:
    """A trace from a CSV file / CSV text / dict. `io=` (an `IoMap`)
    renames binding tags and field-device names to point names, so a log
    written with the electrical drawing's names reads against the bake's.
    `t0=` subtracts a start time (a log that begins at the controller's
    clock)."""
    if isinstance(source, Trace):
        trace = Trace(dict(source.signals))
    elif isinstance(source, dict):
        trace = Trace({str(k): [(float(t), bool(v)) for t, v in vals] for k, vals in source.items()})
    else:
        text = source if (isinstance(source, str) and "\n" in source) else Path(source).read_text()
        trace = _from_csv(text)
    if io is not None:
        trace = trace.renamed(_tag_map(io))
    if t0 is not None:
        trace = trace.shifted(-t0)
    for name in trace.signals:
        trace.signals[name].sort()
    return trace


def _from_csv(text: str) -> Trace:
    reader = csv.reader(_io.StringIO(text))
    rows = [r for r in reader if r and any(c.strip() for c in r)]
    if not rows:
        return Trace()
    header = [h.strip().lower() for h in rows[0]]
    def col(*names: str) -> Optional[int]:
        for n in names:
            if n in header:
                return header.index(n)
        return None
    ti, ni, vi = col("t", "time", "timestamp"), col("name", "signal", "tag", "point"), col("value", "state", "level", "v")
    if ti is None or ni is None or vi is None:
        raise ValueError(
            "trace: the CSV needs `t`, `name` and `value` columns (or time/signal|tag/state); "
            f"got {rows[0]}"
        )
    signals: dict[str, list[Edge]] = {}
    for r in rows[1:]:
        if len(r) <= max(ti, ni, vi):
            continue
        signals.setdefault(r[ni].strip(), []).append((float(r[ti]), _parse_value(r[vi])))
    return Trace(signals)


# ------------------------------------------------------------------ diff


@dataclass
class SignalDiff:
    """One signal, edge by edge: `matched` pairs `(bake t, trace t, kind)`
    within the tolerance, `missing` bake edges the trace never showed,
    `extra` trace edges the bake never predicted, and the largest offset
    among the matches."""

    name: str
    matched: list[tuple[float, float, str]] = field(default_factory=list)
    missing: list[tuple[float, str]] = field(default_factory=list)
    extra: list[tuple[float, str]] = field(default_factory=list)
    max_offset: float = 0.0

    @property
    def ok(self) -> bool:
        return not self.missing and not self.extra


@dataclass
class TraceDiff:
    """The design-versus-reality diff of one baked timeline."""

    tolerance: float
    shift: float
    signals: list[SignalDiff]
    # Baked signals the trace does not carry, and trace signals the bake
    # does not carry — listed, not judged.
    only_in_bake: list[str] = field(default_factory=list)
    only_in_trace: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return all(s.ok for s in self.signals)

    @property
    def max_offset(self) -> float:
        return max((s.max_offset for s in self.signals), default=0.0)

    def findings(self) -> list[dict]:
        out = []
        for s in self.signals:
            for t, kind in s.missing:
                out.append({"severity": "error", "code": "missing_edge", "signal": s.name,
                            "message": f"{s.name}: the bake {kind} at {t:.3f} s, the trace never did"})
            for t, kind in s.extra:
                out.append({"severity": "error", "code": "extra_edge", "signal": s.name,
                            "message": f"{s.name}: the trace {kind} at {t:.3f} s, the bake never did"})
        for name in self.only_in_bake:
            out.append({"severity": "info", "code": "not_in_trace", "signal": name,
                        "message": f"{name}: baked, not in the trace"})
        for name in self.only_in_trace:
            out.append({"severity": "info", "code": "not_in_bake", "signal": name,
                        "message": f"{name}: in the trace, not baked"})
        return out

    def to_json(self) -> str:
        return json.dumps(
            {
                "ok": self.ok,
                "tolerance": self.tolerance,
                "shift": self.shift,
                "max_offset": self.max_offset,
                "signals": [
                    {
                        "name": s.name,
                        "ok": s.ok,
                        "matched": [{"bake": a, "trace": b, "kind": k} for a, b, k in s.matched],
                        "missing": [{"t": t, "kind": k} for t, k in s.missing],
                        "extra": [{"t": t, "kind": k} for t, k in s.extra],
                        "max_offset": s.max_offset,
                    }
                    for s in self.signals
                ],
                "only_in_bake": self.only_in_bake,
                "only_in_trace": self.only_in_trace,
                "findings": self.findings(),
            },
            indent=2,
        )

    def to_markdown(self) -> str:
        lines = [
            f"# Trace diff — {'match' if self.ok else 'MISMATCH'}",
            "",
            f"tolerance {self.tolerance:.3f} s, alignment shift {self.shift:+.3f} s, "
            f"largest matched offset {self.max_offset:.3f} s.",
            "",
            "| signal | matched | missing | extra | max offset (s) |",
            "|---|---|---|---|---|",
        ]
        for s in self.signals:
            lines.append(
                f"| {s.name} | {len(s.matched)} | {len(s.missing)} | {len(s.extra)} | {s.max_offset:.3f} |"
            )
        problems = [f for f in self.findings() if f["severity"] == "error"]
        if problems:
            lines += ["", "Findings:", ""]
            lines += [f"- {f['message']}" for f in problems]
        if self.only_in_bake or self.only_in_trace:
            lines += [""]
            if self.only_in_bake:
                lines.append(f"Baked, not in the trace: {', '.join(self.only_in_bake)}.")
            if self.only_in_trace:
                lines.append(f"In the trace, not baked: {', '.join(self.only_in_trace)}.")
        return "\n".join(lines) + "\n"


def _match(bake: list[float], real: list[float], tolerance: float, kind: str, out: SignalDiff) -> None:
    """Greedy in-order matching of two sorted edge lists."""
    i = j = 0
    while i < len(bake) and j < len(real):
        d = real[j] - bake[i]
        if abs(d) <= tolerance:
            out.matched.append((bake[i], real[j], kind))
            out.max_offset = max(out.max_offset, abs(d))
            i += 1
            j += 1
        elif d < 0:
            out.extra.append((real[j], kind))
            j += 1
        else:
            out.missing.append((bake[i], kind))
            i += 1
    out.missing.extend((t, kind) for t in bake[i:])
    out.extra.extend((t, kind) for t in real[j:])


def diff(
    timeline,
    trace: Union[Trace, dict, str, Path],
    *,
    tolerance: float = 0.05,
    signals: Optional[Iterable[str]] = None,
    align_on: Optional[str] = None,
    io=None,
) -> TraceDiff:
    """Compares a baked `SequenceTimeline` with a trace. `signals=` picks
    the names to judge (default: every name both sides carry); `align_on=`
    names a signal whose first rising edge sets the trace's clock against
    the bake's (a controller log starts whenever it starts); `io=` renames
    tags as in `load`."""
    trace = load(trace, io=io)
    baked_names = [name for name, _ in timeline.signals]
    shift = 0.0
    if align_on is not None:
        bake_r = timeline.signal(align_on).rising_edges()
        real_r, _ = trace.edges(align_on)
        if not bake_r or not real_r:
            raise ValueError(f"align_on={align_on!r}: both the bake and the trace need a rising edge of it")
        shift = bake_r[0] - real_r[0]
        trace = trace.shifted(shift)
    names = list(signals) if signals is not None else [n for n in baked_names if n in trace.signals]
    rows: list[SignalDiff] = []
    for name in names:
        if name not in trace.signals or name not in baked_names:
            raise ValueError(f"signal {name!r} is not on both sides (bake: {name in baked_names}, trace: {name in trace.signals})")
        track = timeline.signal(name)
        real_r, real_f = trace.edges(name)
        row = SignalDiff(name)
        _match(track.rising_edges(), real_r, tolerance, "rose", row)
        _match(track.falling_edges(), real_f, tolerance, "fell", row)
        row.missing.sort()
        row.extra.sort()
        row.matched.sort()
        rows.append(row)
    judged = set(names)
    return TraceDiff(
        tolerance=tolerance,
        shift=shift,
        signals=rows,
        only_in_bake=[n for n in baked_names if n not in trace.signals and n not in judged],
        only_in_trace=[n for n in trace.names if n not in baked_names],
    )


def from_timeline(timeline, signals: Optional[Iterable[str]] = None) -> Trace:
    """A trace *of the bake itself* — the perfect log, for tests and for
    writing the expected trace out next to the program."""
    names = list(signals) if signals is not None else [n for n, _ in timeline.signals]
    return Trace({name: list(timeline.signal(name).edges) for name in names})


def to_csv(trace: Trace) -> str:
    """The trace as `t,name,value` CSV."""
    out = _io.StringIO()
    w = csv.writer(out, lineterminator="\n")
    w.writerow(["t", "name", "value"])
    for name in trace.names:
        for t, v in trace.signals[name]:
            w.writerow([f"{t:.6f}", name, 1 if v else 0])
    return out.getvalue()


__all__ = ["Trace", "TraceDiff", "SignalDiff", "load", "diff", "from_timeline", "to_csv"]
