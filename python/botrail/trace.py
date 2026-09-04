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

A machine tool logs in MTConnect rather than as a PLC trend:
`read_mtconnect` reads an `MTConnectStreams` document's events
(`Execution`, `DoorState`, `ChuckState`, `EmergencyStop`, …) as levels on
the bake's lanes, and `to_mtconnect` writes the bake back out in the same
vocabulary — the expected stream, for the agent's operator to compare.
"""

from __future__ import annotations

import csv
import io as _io
import json
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone
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


# ------------------------------------------------------------- MTConnect

#: How an MTConnect event's value reads as a level, by data item type
#: (MTConnect Part 3, Observation Information Model). Types not listed
#: read `ACTIVE` / `ON` / `TRUE` / `CLOSED` / `TRIGGERED` / `1` as high.
MTCONNECT_LEVELS: dict[str, dict[str, bool]] = {
    "Execution": {"ACTIVE": True},
    "EmergencyStop": {"TRIGGERED": True, "ARMED": False},
    "ChuckState": {"CLOSED": True, "OPEN": False, "UNLATCHED": False},
    "PowerState": {"ON": True, "OFF": False},
    "ControllerMode": {"AUTOMATIC": True},
    "Availability": {"AVAILABLE": True},
    "PartDetect": {"PRESENT": True, "NOT_PRESENT": False},
}
_HIGH = {"ACTIVE", "ON", "TRUE", "CLOSED", "TRIGGERED", "1", "HIGH", "PRESENT"}
#: `DoorState` is three-valued: OPEN, CLOSED, or UNLATCHED — neither end
#: confirmed. It reads onto two lanes, the closed switch and the open one.
DOOR_STATES = {"OPEN": (False, True), "CLOSED": (True, False), "UNLATCHED": (False, False)}

Items = dict[str, Union[str, tuple[str, str]]]


def _local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _mtc_time(text: str) -> datetime:
    text = text.strip()
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    stamp = datetime.fromisoformat(text)
    return stamp if stamp.tzinfo else stamp.replace(tzinfo=timezone.utc)


def _seconds(t0: Union[str, float, datetime, None], stamp: datetime, first: datetime) -> float:
    if t0 is None:
        origin = first
    elif isinstance(t0, datetime):
        origin = t0 if t0.tzinfo else t0.replace(tzinfo=timezone.utc)
    elif isinstance(t0, str):
        origin = _mtc_time(t0)
    else:
        return (stamp - first).total_seconds() + float(t0)
    return (stamp - origin).total_seconds()


def read_mtconnect(source: Union[str, Path], items: Items, *, t0: Union[str, float, datetime, None] = None) -> Trace:
    """A trace from an MTConnect `MTConnectStreams` document (a file path
    or the XML text — the agent's `/current` or `/sample` response).

    `items` says which observations are which lanes: each key is a data
    item's `dataItemId`, its `name`, or its type (`Execution`,
    `EmergencyStop`, …) and each value the bake's lane name — or, for
    `DoorState`, a `(closed_lane, open_lane)` pair, since a door reports
    OPEN, CLOSED or UNLATCHED (neither end confirmed). Levels follow the
    standard's vocabulary (`MTCONNECT_LEVELS`): `Execution ACTIVE` is the
    machine running, `EmergencyStop TRIGGERED` the E-stop in, `ChuckState
    CLOSED` the clamp made. `UNAVAILABLE` observations are skipped.

    Times are seconds from the first matched observation, or from `t0`
    (an ISO 8601 stamp, a `datetime`, or a number of seconds the first
    observation sits at). Samples and conditions are not read — the diff
    compares levels."""
    text = source if (isinstance(source, str) and source.lstrip().startswith("<")) else Path(source).read_text()
    root = ET.fromstring(text)
    found: list[tuple[datetime, str, str]] = []  # (stamp, key, value)
    for el in root.iter():
        tag = _local(el.tag)
        key = None
        for candidate in (el.get("dataItemId"), el.get("name"), tag):
            if candidate is not None and candidate in items:
                key = candidate
                break
        if key is None or el.get("timestamp") is None:
            continue
        value = (el.text or "").strip()
        if not value or value.upper() == "UNAVAILABLE":
            continue
        found.append((_mtc_time(el.get("timestamp")), key, value))
    if not found:
        return Trace()
    found.sort(key=lambda f: f[0])
    first = found[0][0]
    signals: dict[str, list[Edge]] = {}
    for stamp, key, value in found:
        t = _seconds(t0, stamp, first)
        lane = items[key]
        state = value.upper()
        if isinstance(lane, tuple):
            closed, opened = DOOR_STATES.get(state, (False, False))
            signals.setdefault(lane[0], []).append((t, closed))
            signals.setdefault(lane[1], []).append((t, opened))
            continue
        table = MTCONNECT_LEVELS.get(_mtc_type(root, key))
        level = table.get(state, False) if table is not None else state in _HIGH
        signals.setdefault(lane, []).append((t, level))
    for samples in signals.values():
        samples.sort()
    return Trace(signals)


def _mtc_type(root: ET.Element, key: str) -> str:
    """The element type an `items` key stands for — the key itself when it
    is a type, else the tag of the element carrying it as id or name."""
    if key in MTCONNECT_LEVELS or key == "DoorState":
        return key
    for el in root.iter():
        if el.get("dataItemId") == key or el.get("name") == key:
            return _local(el.tag)
    return key


def to_mtconnect(trace: Union[Trace, object], items: Items, *, start: str = "2000-01-01T00:00:00Z",
                 device: str = "machine") -> str:
    """A minimal `MTConnectStreams` document of `trace` (a `Trace`, or a
    `SequenceTimeline` — the bake's lanes) in the standard's vocabulary,
    with `items` read the other way round: the lane under each key is
    written as that data item — `Execution` as ACTIVE / READY, `DoorState`
    as OPEN / CLOSED / UNLATCHED from its two lanes, `EmergencyStop` as
    TRIGGERED / ARMED, `ChuckState` as CLOSED / OPEN, anything else as
    ON / OFF. Stamps run from `start` at the trace's seconds. The expected
    stream, to lay beside the machine's own."""
    if not isinstance(trace, Trace):
        trace = from_timeline(trace)
    origin = _mtc_time(start)
    events: list[tuple[float, str, str]] = []  # (t, key, value)
    for key, lane in items.items():
        if isinstance(lane, tuple):
            times = sorted({t for name in lane for t, _ in trace.signals.get(name, [])})
            for t in times:
                closed, opened = (_level_at(trace, lane[0], t), _level_at(trace, lane[1], t))
                state = "CLOSED" if closed and not opened else "OPEN" if opened and not closed else "UNLATCHED"
                events.append((t, key, state))
            continue
        for t, level in trace.signals.get(lane, []):
            events.append((t, key, _mtc_word(key, level)))
    events.sort(key=lambda e: (e[0], e[1]))
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<MTConnectStreams xmlns="urn:mtconnect.org:MTConnectStreams:2.0">',
        f'<Header creationTime="{start}" sender="botrail" instanceId="1" version="2.0.0" bufferSize="131072" '
        f'nextSequence="{len(events) + 1}" firstSequence="1" lastSequence="{max(len(events), 1)}"/>',
        f'<Streams><DeviceStream name="{device}" uuid="{device}"><ComponentStream component="Controller" '
        f'name="controller" componentId="cont"><Events>',
    ]
    for n, (t, key, value) in enumerate(events, start=1):
        stamp = (origin + timedelta(seconds=t)).isoformat().replace("+00:00", "Z")
        # A key that is a standard type is written as that element; any
        # other key is a data item id on a generic event.
        tag = key if key in MTCONNECT_LEVELS or key == "DoorState" else "Event"
        lines.append(f'<{tag} dataItemId="{key}" timestamp="{stamp}" sequence="{n}">{value}</{tag}>')
    lines.append("</Events></ComponentStream></DeviceStream></Streams></MTConnectStreams>")
    return "\n".join(lines) + "\n"


def _level_at(trace: Trace, name: str, t: float) -> bool:
    level = False
    for t1, v in sorted(trace.signals.get(name, [])):
        if t1 <= t + 1e-9:
            level = v
        else:
            break
    return level


def _mtc_word(kind: str, level: bool) -> str:
    words = {
        "Execution": ("ACTIVE", "READY"),
        "EmergencyStop": ("TRIGGERED", "ARMED"),
        "ChuckState": ("CLOSED", "OPEN"),
        "PowerState": ("ON", "OFF"),
        "ControllerMode": ("AUTOMATIC", "MANUAL"),
        "Availability": ("AVAILABLE", "UNAVAILABLE"),
        "PartDetect": ("PRESENT", "NOT_PRESENT"),
    }
    high, low = words.get(kind, ("ON", "OFF"))
    return high if level else low
