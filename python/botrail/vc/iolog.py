"""A controller's I/O over time, as a machine's data logger writes it —
read against the cell's exchange.

A PLC's logging function, a SCADA trend or a gateway exports the I/O image
as a table: a time column and one column per device, headed by its address
(`X0`, `%IX100.5`) or its tag. `IoLog` reads such a table against the
exchange of the controller it came from, so every column lands on a point
of the cell — the coils and command words the controller *wrote* (which
drive the model when the log is replayed) and the contacts and words it
*read* (which are compared with what the model computes)::

    xc = bt.vc.exchange(scene, "PLC1")
    log = bt.vc.IoLog.load("line_io.csv", xc)        # columns by address, tag, variable or name
    print(log.unmatched)                             # columns that are no point of the cell

A wide table (`t, X0, X1, Y0, D100` — one row per sample, an empty cell is
"unchanged") and a long one (`t, name, value`) are both read. Values are
what the controller holds: `1/0`, `true/false`, `on/off` on a bit, the
register's number on a word.

`bt.vc.tap(link, xc)` writes the same table from a plant session — the I/O
of a commissioning run, to replay or to compare later.
"""

from __future__ import annotations

import csv
import io as _io
from datetime import datetime
from pathlib import Path
from statistics import median
from typing import Optional, Union

Value = Union[bool, int, float]
Sample = tuple[float, Value]

__all__ = ["IoLog", "IoTap", "tap"]

_TRUE = {"1", "true", "on", "high", "t", "yes"}
_FALSE = {"0", "false", "off", "low", "f", "no"}
_TIME = {"t", "time", "timestamp", "時刻", "時間", "datetime", "date"}
_NAME = {"name", "signal", "tag", "address", "label", "point", "device"}
_VALUE = {"value", "state", "val"}


def _points(exchange) -> dict[str, object]:
    return {p.label: p for p in exchange.points}


def _keys(exchange, names: Optional[dict[str, str]]) -> dict[str, str]:
    """`{column heading (lower case): point label}`: a point answers to its
    label, its PLC tag, its variable and its address (`%IX100.5`,
    `IX100.5`, `X0`), and to whatever `names` adds."""
    out: dict[str, str] = {}
    points = list(exchange.points)
    for attr in ("label", "tag", "var", "address"):
        for p in points:
            key = getattr(p, attr, None)
            if not key:
                continue
            key = str(key).strip().lower()
            out.setdefault(key, p.label)
            if attr == "address" and key.startswith("%"):
                out.setdefault(key[1:], p.label)
    known = {p.label for p in points}
    for key, label in (names or {}).items():
        if label not in known:
            raise ValueError(f"io log: `{key}` is mapped to `{label}`, which is no point of {exchange.host}'s exchange")
        out[str(key).strip().lower()] = label
    return out


def _value(text: str, ty: str, where: str) -> Value:
    v = text.strip().lower()
    if ty == "BOOL":
        if v in _TRUE:
            return True
        if v in _FALSE:
            return False
        try:
            return float(v) != 0.0
        except ValueError as e:
            raise ValueError(f"io log: {where}: cannot read {text!r} as a bit") from e
    try:
        number = float(v)
    except ValueError as e:
        raise ValueError(f"io log: {where}: cannot read {text!r} as a number") from e
    return int(round(number)) if ty == "INT" else number


def _seconds(text: str, where: str) -> float:
    """A time cell: seconds, or a date and time (`2026-11-10 08:15:02.250`,
    `2026/11/10 08:15:02`, `08:15:02.250`)."""
    v = text.strip()
    try:
        return float(v)
    except ValueError:
        pass
    for fmt in ("%Y-%m-%d %H:%M:%S.%f", "%Y-%m-%d %H:%M:%S", "%Y/%m/%d %H:%M:%S.%f", "%Y/%m/%d %H:%M:%S",
                "%Y-%m-%dT%H:%M:%S.%f", "%Y-%m-%dT%H:%M:%S", "%H:%M:%S.%f", "%H:%M:%S"):
        try:
            return datetime.strptime(v, fmt).timestamp()
        except ValueError:
            continue
    raise ValueError(f"io log: {where}: cannot read {text!r} as a time")


class IoLog:
    """The I/O image of one controller over time. `series[label]` is that
    point's samples `(t, value)`, the first being its first reading and
    the rest its changes; `written` are the points the controller writes
    (coils, commands), `read` the ones it reads (contacts, feedback)."""

    def __init__(self, exchange, series: Optional[dict[str, list[Sample]]] = None, *, source: str = "log",
                 unmatched: Optional[list[str]] = None, period: float = 0.0):
        self.exchange = exchange
        self.host: str = exchange.host
        self._points = _points(exchange)
        self.series: dict[str, list[Sample]] = {}
        for label, samples in (series or {}).items():
            if label not in self._points:
                raise ValueError(f"io log: `{label}` is no point of {self.host}'s exchange")
            self.series[label] = _changes(sorted(samples, key=lambda s: s[0]))
        #: Where the log came from, as the studio names it.
        self.source = source
        #: Columns of the file that matched no point of the exchange.
        self.unmatched: list[str] = list(unmatched or [])
        #: The logger's sampling period (the median row spacing of a wide
        #: table), or 0 when the rows are changes.
        self.period = period

    # -- reading ------------------------------------------------------------

    @property
    def labels(self) -> list[str]:
        return list(self.series)

    @property
    def written(self) -> list[str]:
        """The logged points the controller writes — what a replay feeds the model."""
        return [k for k in self.series if self._points[k].side == "input"]

    @property
    def read(self) -> list[str]:
        """The logged points the controller reads — what a replay compares."""
        return [k for k in self.series if self._points[k].side == "output"]

    @property
    def duration(self) -> float:
        return max((s[-1][0] for s in self.series.values() if s), default=0.0)

    @property
    def missing(self) -> list[str]:
        """Points of the exchange the log does not carry."""
        return [k for k in self._points if k not in self.series]

    def value(self, label: str, t: float) -> Optional[Value]:
        """The point's value at `t` — `None` before its first reading."""
        out: Optional[Value] = None
        for at, v in self.series.get(label, ()):
            if at > t + 1e-9:
                break
            out = v
        return out

    def image(self, t: float) -> dict[str, Value]:
        """Every logged point's value at `t`."""
        out = {}
        for label in self.series:
            v = self.value(label, t)
            if v is not None:
                out[label] = v
        return out

    # -- deriving -----------------------------------------------------------

    def recording(self):
        """The controller's writes as a `bt.vc.Recording` — what
        `bt.vc.replay` feeds a plant session."""
        from . import Recording

        records = [(t, self.host, label, v) for label in self.written for t, v in self.series[label]]
        records.sort(key=lambda r: r[0])
        return Recording(records)

    def delayed(self, label: str, seconds: float) -> "IoLog":
        """The same log with one point's changes moved by `seconds` (its
        first reading stays) — a sensor that answers late, to see what a
        comparison makes of it."""
        if label not in self.series:
            raise ValueError(f"io log: `{label}` is not in the log (logged: {', '.join(self.series)})")
        series = {k: list(v) for k, v in self.series.items()}
        first, rest = series[label][0], series[label][1:]
        series[label] = [first] + [(t + seconds, v) for t, v in rest]
        return IoLog(self.exchange, series, source=self.source, unmatched=self.unmatched, period=self.period)

    # -- files --------------------------------------------------------------

    @classmethod
    def from_csv(cls, text: str, exchange, *, names: Optional[dict[str, str]] = None, t0: Optional[float] = None,
                 source: str = "log") -> "IoLog":
        """Reads a logger's CSV against `exchange`. `names` maps headings
        the exchange does not know (`{"X0": "s1"}`); `t0` is the time that
        becomes zero (the first row's by default)."""
        rows = [r for r in csv.reader(_io.StringIO(text)) if r and any(c.strip() for c in r)]
        if not rows:
            return cls(exchange, source=source)
        header = [h.strip() for h in rows[0]]
        lower = [h.lower() for h in header]
        keys = _keys(exchange, names)
        points = _points(exchange)
        tcol = next((i for i, h in enumerate(lower) if h in _TIME), 0)
        ncol = next((i for i, h in enumerate(lower) if h in _NAME and i != tcol), None)
        vcol = next((i for i, h in enumerate(lower) if h in _VALUE), None)
        series: dict[str, list[Sample]] = {}
        unmatched: list[str] = []
        times: list[float] = []
        if ncol is not None and vcol is not None:  # long: t, name, value
            for n, row in enumerate(rows[1:], start=2):
                t = _seconds(row[tcol], f"row {n}")
                name = row[ncol].strip()
                label = keys.get(name.lower())
                if label is None:
                    if name not in unmatched:
                        unmatched.append(name)
                    continue
                times.append(t)
                series.setdefault(label, []).append(
                    (t, _value(row[vcol], points[label].value_type, f"row {n}, {name}")))
            period = 0.0
        else:  # wide: t, <point>, <point>, …
            columns: list[tuple[int, str]] = []
            for i, h in enumerate(header):
                if i == tcol or not h:
                    continue
                label = keys.get(h.lower())
                if label is None:
                    unmatched.append(h)
                else:
                    columns.append((i, label))
            for n, row in enumerate(rows[1:], start=2):
                t = _seconds(row[tcol], f"row {n}")
                times.append(t)
                for i, label in columns:
                    if i < len(row) and row[i].strip():
                        series.setdefault(label, []).append(
                            (t, _value(row[i], points[label].value_type, f"row {n}, {header[i]}")))
            steps = [b - a for a, b in zip(times, times[1:]) if b > a]
            period = median(steps) if steps else 0.0
            # A table written on change has no period worth the name.
            if steps and max(steps) > 3.0 * period + 1e-9:
                period = 0.0
        if not series:
            raise ValueError(
                f"io log: no column is a point of {exchange.host}'s exchange (headings: {', '.join(header)}) — "
                "head the columns with the points' addresses, tags or names, or pass names={heading: point}"
            )
        zero = min(times) if t0 is None else t0
        series = {k: [(t - zero, v) for t, v in s] for k, s in series.items()}
        return cls(exchange, series, source=source, unmatched=unmatched, period=period)

    @classmethod
    def load(cls, path: Union[str, Path], exchange, **kw) -> "IoLog":
        path = Path(path)
        kw.setdefault("source", f"log {path.name}")
        return cls.from_csv(path.read_text(encoding="utf-8-sig"), exchange, **kw)

    def to_csv(self, by: str = "address") -> str:
        """The wide table: a row at every time something changes, every
        point's value in it. Columns are headed by the point's `address`
        (its name when unbound), `tag`, `var` or `label`."""
        if by not in ("address", "tag", "var", "label"):
            raise ValueError(f"io log: by must be 'address', 'tag', 'var' or 'label', not {by!r}")
        labels = sorted(self.series, key=lambda k: (self._points[k].side != "output", k))
        heads = [str(getattr(self._points[k], by, None) or k) for k in labels]
        times = sorted({round(t, 6) for s in self.series.values() for t, _ in s})
        cursors = {k: 0 for k in labels}
        current: dict[str, Optional[Value]] = {k: None for k in labels}
        out = _io.StringIO()
        w = csv.writer(out, lineterminator="\n")
        w.writerow(["t"] + heads)
        for t in times:
            for k in labels:
                s = self.series[k]
                i = cursors[k]
                while i < len(s) and s[i][0] <= t + 5e-7:
                    current[k] = s[i][1]
                    i += 1
                cursors[k] = i
            w.writerow([f"{t:.3f}"] + [_cell(current[k]) for k in labels])
        return out.getvalue()

    def save(self, path: Union[str, Path], by: str = "address") -> None:
        Path(path).write_text(self.to_csv(by), encoding="utf-8")

    def __repr__(self) -> str:
        return (f"IoLog(host={self.host!r}, points={len(self.series)}, duration={self.duration:.2f} s, "
                f"source={self.source!r})")


def _cell(value: Optional[Value]) -> str:
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int) or float(value).is_integer():
        return str(int(value))
    return repr(float(value))


def _changes(samples: list[Sample]) -> list[Sample]:
    out: list[Sample] = []
    for t, v in samples:
        if not out or out[-1][1] != v:
            out.append((t, v))
    return out


class IoTap:
    """A link with a logger on it: every image that crosses `link` — the
    plant's outputs going to the controller, the controller's writes
    coming back — is kept as an `IoLog` (`tap.log`), the table a machine's
    data logger would have written of the run. The link itself is
    untouched: `Session(links={"PLC1": bt.vc.tap(link, xc)})`."""

    def __init__(self, link, exchange):
        self._link = link
        self._series: dict[str, list[Sample]] = {}
        self._xc = exchange
        self._types = {p.label: p.value_type for p in exchange.points}
        self._last_t = 0.0

    def _note(self, label: str, t: float, value: Value) -> None:
        if label not in self._types:
            return
        s = self._series.setdefault(label, [])
        if not s or s[-1][1] != value:
            s.append((t, value))

    def exchange(self, host: str, t: float, outputs: dict[str, Value]) -> dict[str, Value]:
        # The outputs handed over for scan `t` are the plant as the scan
        # before left it: that is their time. The writes land at `t`.
        for label, value in outputs.items():
            self._note(label, self._last_t, value)
        self._last_t = t
        writes = self._link.exchange(host, t, outputs)
        for label, value in (getattr(writes, "ordered", None) or (writes or {}).items()):
            self._note(label, t, value)
        return writes

    @property
    def log(self) -> IoLog:
        """The I/O logged so far."""
        return IoLog(self._xc, {k: list(v) for k, v in self._series.items()}, source="tap")

    def __getattr__(self, name: str):
        # start / stop / describe / polls / robots / joints: the link's own.
        return getattr(self._link, name)


def tap(link, exchange) -> IoTap:
    """`link` with a logger on it (see `IoTap`)."""
    return IoTap(link, exchange)
