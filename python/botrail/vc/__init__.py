"""Virtual commissioning: the cell as the **plant** of a controller outside
it (design-virtual-commissioning.md, VC1).

The bake is the trajectory the cell follows if its controller behaves as
designed. A plant session lets a controller *actually* drive the cell: the
programs on the controller's host are not scanned — the controller writes
the host's inputs (coils, device commands, robot start and program words)
and reads its outputs (sensors, limit switches, in-position and done
contacts, handshake wires) through the host's **exchange**, the half of the
I/O map that host sees. Everything else — the other programs, devices,
sensors, physics — runs as in the bake, so the result is the same
`SequenceTimeline`: studio playback, USD, the clearance report and
`tl.diff` work on it unchanged.

Nothing here is authored. The exchange is derived from the I/O map — the
same points, variables, types and addresses the PLCopen export declares::

    xc = bt.vc.exchange(scene, "plc1")          # what the controller writes and reads
    print(xc.to_markdown())

Two links come with VC1, both deterministic (no wall clock):

* **loopback** — botrail runs the host's own programs, but every action
  they fire is encoded onto the exchange and decoded back. The reference
  run: it reproduces the batch bake bit for bit, which is what proves the
  exchange lossless and the I/O derivation complete::

      with bt.vc.loopback(scene, ["pick"], host="UR") as s:
          s.run()                                  # to the end of the programs
          tl2 = s.timeline()                       # == scene.simulate_sequence("pick")
          rec = s.recording()                      # every input written, with its time

* **replay** — the recorded inputs fed back, with the host external::

      with bt.vc.Session(scene, ["pick"], links={"UR": bt.vc.replay(rec)}) as s:
          s.run(for_=tl2.duration)

A session can also pin lanes like a PLC force (`force` / `unforce`) and
inject faults (`inject(bt.io.stuck("beam", False), at=bt.vc.rising("x"))`),
and `tl.diff(bt.trace.from_timeline(tl2))` then names where the plant and
the design parted.

With a controller on the wire the session runs in wall-clock time
(`pace="realtime"`), the plant answering as a field device — a Modbus TCP
server the PLC polls, an OPC UA client of the controller's server, or the
RTDE client of a UR controller that runs the robot's own program and
streams its joints into the plant (`bt.vc.ursim`; the link's `robots` make
the session mirror them) — and the programs the plant scans itself cycle
like the controller's (`cycle=True`)::

    xc = bt.vc.exchange(scene, "PLC1")
    link = bt.vc.modbus_server(xc, "0.0.0.0:5020", allow_remote=True)   # the PLC polls this
    with bt.vc.Session(scene, ["pick"], links={"PLC1": link}, pace="realtime", cycle=True) as s:
        s.run(for_=60.0)
        print(s.stats())                            # scans, lag, overruns
        d = bake.diff(s.trace(), tolerance=0.1, align_on="conv")

The session is the hand on the plant while it runs — `pause` / `resume` /
`step`, `speed`, `press` (a panel button), `break_on` (a breakpoint on a
lane's edge), `record` / `stop_record` (a clip of the run as a timeline of
its own) — and with a studio open on the scene (`bt.studio(scene,
block=False)`) the run streams there live, the dock's plant bar and the I/O
table's force column send the same commands back, and a stopped recording
lands on the dock playable and exportable (design-virtual-commissioning.md
VC3).
"""

from __future__ import annotations

import csv
import io as _io
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Optional, Protocol, Union

from ..trace import Trace
from . import interfaces
from .links import ModbusServerLink, OpcUaClientLink, UrDashboard, UrRtdeLink, modbus_server, opcua_client, ursim

Value = Union[bool, int, float]
Record = tuple[float, str, str, Value]  # (t, host, label, value)

__all__ = [
    "Exchange",
    "Link",
    "ModbusServerLink",
    "OpcUaClientLink",
    "UrDashboard",
    "UrRtdeLink",
    "ursim",
    "interfaces",
    "Recording",
    "ReplayLink",
    "Session",
    "exchange",
    "loopback",
    "modbus_server",
    "opcua_client",
    "replay",
    "rising",
    "falling",
    "level",
    "done",
    "step",
    "elapsed",
    # the FAT (fat.py)
    "TestCase",
    "Expectation",
    "Check",
    "CaseResult",
    "FatReport",
    "run_fat",
    "interlock_tests",
    "safety_tests",
    "load_cases",
    "cases_from_data",
    "cases_to_data",
    "rises",
    "falls",
    "never",
    "held",
    "step_reached",
    "stalls",
    "no_motion",
    "clearance",
    "cycle_within",
    "check",
]


# ------------------------------------------------------------------ exchange


class Exchange:
    """The exchange of one controller host — `scene.exchange(host)` with
    the renderings: `to_markdown()`, `to_csv()`, `to_json()`, `save(path)`.
    Points are the plant's inputs (the controller writes) and outputs (the
    controller reads); `entries` are the numbered robot commands;
    `parameters` the device constants a bus cannot carry live."""

    def __init__(self, core):
        self._core = core

    @property
    def host(self) -> str:
        return self._core.host

    @property
    def programs(self) -> list[str]:
        return list(self._core.programs)

    @property
    def points(self) -> list:
        return list(self._core.points)

    @property
    def inputs(self) -> list:
        """Points the controller writes."""
        return [p for p in self._core.points if p.side == "input"]

    @property
    def outputs(self) -> list:
        """Points the controller reads."""
        return [p for p in self._core.points if p.side == "output"]

    @property
    def entries(self) -> list[tuple[str, int, str, str]]:
        return list(self._core.entries)

    @property
    def parameters(self) -> list[tuple[str, str, float]]:
        return list(self._core.parameters)

    @property
    def notes(self) -> list[str]:
        return list(self._core.notes)

    def to_json(self) -> str:
        return self._core.to_json()

    def rows(self) -> list[dict]:
        """One dict per point, the table's columns."""
        return [
            {
                "label": p.label,
                "var": p.var,
                "side": p.side,
                "type": p.value_type,
                "kind": p.kind,
                "source": p.source,
                "node": p.node or "",
                "channel": p.channel or "",
                "address": p.address or "",
                "tag": p.tag or "",
                "invert": "nc" if p.invert else "",
                "unit": p.unit or "",
                "initial": p.initial,
                "note": p.note or "",
            }
            for p in self._core.points
        ]

    def to_csv(self) -> str:
        out = _io.StringIO()
        rows = self.rows()
        columns = ["label", "var", "side", "type", "kind", "source", "node", "channel",
                   "address", "tag", "invert", "unit", "initial", "note"]
        w = csv.DictWriter(out, fieldnames=columns, lineterminator="\n")
        w.writeheader()
        for r in rows:
            w.writerow(r)
        return out.getvalue()

    def to_markdown(self) -> str:
        lines = [f"# Exchange — host `{self.host}`", ""]
        if self.programs:
            lines.append("Programs on this host: " + ", ".join(f"`{p}`" for p in self.programs))
            lines.append("")
        for side, title in (("input", "Inputs — the controller writes"),
                            ("output", "Outputs — the controller reads")):
            pts = [p for p in self._core.points if p.side == side]
            lines.append(f"## {title} ({len(pts)})")
            lines.append("")
            lines.append("| point | variable | type | source | address | tag | wire | initial |")
            lines.append("|---|---|---|---|---|---|---|---|")
            for p in pts:
                where = p.address or (f"{p.node}.{p.channel}" if p.node else "")
                ty = f"{p.value_type} ({p.unit})" if p.unit else p.value_type
                lines.append(
                    f"| `{p.label}` | `{p.var}` | {ty} | {p.source} | {where} | "
                    f"{p.tag or ''} | {'NC' if p.invert else ''} | {p.initial} |"
                )
            lines.append("")
            # What the plant does with a point beyond the obvious (a coil a
            # grasp follows).
            for p in pts:
                if p.note:
                    lines.append(f"- `{p.label}`: {p.note}")
            if any(p.note for p in pts):
                lines.append("")
        if self.entries:
            lines.append("## Robot commands (program word → command)")
            lines.append("")
            lines.append("| robot | number | command |")
            lines.append("|---|---|---|")
            for robot, number, key, label in self.entries:
                lines.append(f"| `{robot}` | {number} | {label} |")
            lines.append("")
        if self.parameters:
            lines.append("## Parameters")
            lines.append("")
            lines.append("| device | parameter | value |")
            lines.append("|---|---|---|")
            for device, name, value in self.parameters:
                lines.append(f"| `{device}` | {name} | {value} |")
            lines.append("")
        if self._core.internal:
            lines.append("Internal relays (never exchanged): " + ", ".join(f"`{n}`" for n in self._core.internal))
            lines.append("")
        for note in self.notes:
            lines.append(f"- note: {note}")
        return "\n".join(lines).rstrip() + "\n"

    def save(self, path: Union[str, Path]) -> None:
        """Writes the table; the format follows the extension (`.md`, `.csv`, `.json`)."""
        path = Path(path)
        text = {".md": self.to_markdown, ".markdown": self.to_markdown,
                ".csv": self.to_csv, ".json": self.to_json}.get(path.suffix.lower())
        if text is None:
            raise ValueError(f"exchange.save: unknown extension {path.suffix!r} — use .md, .csv or .json")
        path.write_text(text(), encoding="utf-8")

    def __repr__(self) -> str:
        return repr(self._core)


def exchange(scene, host: str, sequences: Optional[list[str]] = None) -> Exchange:
    """The exchange of controller `host` over `sequences` (every sequence
    by default) — derived from the I/O map, never authored."""
    return Exchange(scene.exchange(host, sequences))


# ------------------------------------------------------------------ links


class Link(Protocol):
    """What a session asks a controller each scan: given the host, the time
    of the scan about to run and the plant's outputs after the last one,
    the inputs to write (`{label: value}`, changed ones suffice)."""

    def exchange(self, host: str, t: float, outputs: dict[str, Value]) -> dict[str, Value]: ...


@dataclass
class Recording:
    """Every input written on the plant during a session: `(t, host,
    label, value)` in order — what `replay` feeds back, bit for bit."""

    records: list[Record] = field(default_factory=list)

    @property
    def hosts(self) -> list[str]:
        seen: list[str] = []
        for _, host, _, _ in self.records:
            if host not in seen:
                seen.append(host)
        return seen

    @property
    def duration(self) -> float:
        return max((t for t, _, _, _ in self.records), default=0.0)

    def trace(self, host: Optional[str] = None) -> Trace:
        """The boolean inputs as a `bt.trace.Trace` (words and reals are
        left out) — what `tl.diff` reads."""
        signals: dict[str, list[tuple[float, bool]]] = {}
        for t, h, label, value in self.records:
            if host is not None and h != host:
                continue
            if isinstance(value, bool):
                signals.setdefault(label, []).append((t, value))
        return Trace(signals)

    def to_csv(self) -> str:
        out = _io.StringIO()
        w = csv.writer(out, lineterminator="\n")
        w.writerow(["t", "host", "label", "value"])
        for t, host, label, value in self.records:
            w.writerow([f"{t:.6f}", host, label, json.dumps(value)])
        return out.getvalue()

    @classmethod
    def from_csv(cls, text: str) -> "Recording":
        rows = list(csv.reader(_io.StringIO(text)))
        if not rows:
            return cls()
        if [c.strip().lower() for c in rows[0]] == ["t", "host", "label", "value"]:
            rows = rows[1:]
        records: list[Record] = []
        for row in rows:
            if len(row) != 4:
                raise ValueError(f"recording: expected t,host,label,value — got {row!r}")
            t, host, label, value = row
            records.append((float(t), host, label, json.loads(value)))
        return cls(records)

    def to_json(self) -> str:
        return json.dumps([{"t": t, "host": h, "label": l, "value": v} for t, h, l, v in self.records])

    @classmethod
    def from_json(cls, text: str) -> "Recording":
        return cls([(r["t"], r["host"], r["label"], r["value"]) for r in json.loads(text)])

    def save(self, path: Union[str, Path]) -> None:
        path = Path(path)
        if path.suffix.lower() == ".json":
            path.write_text(self.to_json(), encoding="utf-8")
        else:
            path.write_text(self.to_csv(), encoding="utf-8")

    @classmethod
    def load(cls, path: Union[str, Path]) -> "Recording":
        path = Path(path)
        text = path.read_text(encoding="utf-8")
        return cls.from_json(text) if path.suffix.lower() == ".json" else cls.from_csv(text)


class ReplayLink:
    """A recorded input stream played back: at each scan the writes whose
    time is that scan's. Deterministic — the plant replays the same
    timeline the recording came from."""

    def __init__(self, recording: Recording, tolerance: float = 1e-6):
        self.recording = recording
        self.tolerance = tolerance
        self._by_host: dict[str, list[Record]] = {}
        for rec in recording.records:
            self._by_host.setdefault(rec[1], []).append(rec)
        self._cursor: dict[str, int] = {h: 0 for h in self._by_host}

    def describe(self) -> str:
        return "replay"

    @property
    def exchanges(self) -> int:
        """Records played back so far."""
        return sum(self._cursor.values())

    def exchange(self, host: str, t: float, outputs: dict[str, Value]) -> dict[str, Value]:
        records = self._by_host.get(host, [])
        i = self._cursor.get(host, 0)
        writes: dict[str, Value] = {}
        ordered: list[tuple[str, Value]] = []
        while i < len(records) and records[i][0] <= t + self.tolerance:
            _, _, label, value = records[i]
            ordered.append((label, value))
            i += 1
        self._cursor[host] = i
        # Several writes of one label in one scan (a pulse dropped and
        # raised) must all land, in order: a dict keeps the last, so the
        # session applies the ordered list when there is one.
        for label, value in ordered:
            writes[label] = value
        if len(ordered) != len(writes):
            return _Ordered(ordered)  # type: ignore[return-value]
        return writes


class _Ordered(dict):
    """Writes that must be applied in order, repeats included."""

    def __init__(self, ordered: list[tuple[str, Value]]):
        super().__init__()
        self.ordered = ordered
        for label, value in ordered:
            self[label] = value


def replay(recording: Union[Recording, str, Path]) -> ReplayLink:
    """A link that plays a recording (or a saved one) back."""
    if not isinstance(recording, Recording):
        recording = Recording.load(recording)
    return ReplayLink(recording)


# -------------------------------------------------------------- predicates


class _Predicate:
    """`until=` conditions and `at=` anchors share these objects."""

    anchor: Optional[dict] = None

    def __call__(self, session: "Session") -> bool:  # pragma: no cover - abstract
        raise NotImplementedError


class rising(_Predicate):
    """The lane rose since the last check (as an `until`), or its next
    rising edge (as an `at`)."""

    def __init__(self, name: str):
        self.name = name
        self.anchor = {"rising": name}
        self._prev: Optional[bool] = None

    def __call__(self, session: "Session") -> bool:
        now = session.signal(self.name)
        rose = self._prev is False and now
        self._prev = now
        return rose


class falling(_Predicate):
    def __init__(self, name: str):
        self.name = name
        self._prev: Optional[bool] = None

    def __call__(self, session: "Session") -> bool:
        now = session.signal(self.name)
        fell = self._prev is True and not now
        self._prev = now
        return fell


class level(_Predicate):
    """The lane stands at `value`."""

    def __init__(self, name: str, value: bool = True):
        self.name = name
        self.value = value

    def __call__(self, session: "Session") -> bool:
        return session.signal(self.name) == self.value


class done(_Predicate):
    """A program the plant scans has run to its end (every program when
    `program` is None)."""

    def __init__(self, program: Optional[str] = None):
        self.program = program

    def __call__(self, session: "Session") -> bool:
        if self.program is None:
            return session.finished
        return any(p == self.program and s is None for p, s in session.steps())


class step(_Predicate):
    """A program the plant scans stands in step `name` (as an `until`), or
    enters it (as an `at`)."""

    def __init__(self, program: str, name: str):
        self.program = program
        self.name = name
        self.anchor = {"program": program, "step": name}

    def __call__(self, session: "Session") -> bool:
        return any(p == self.program and s == self.name for p, s in session.steps())


class elapsed(_Predicate):
    """`seconds` of plant time since the predicate was first checked."""

    def __init__(self, seconds: float):
        self.seconds = seconds
        self._t0: Optional[float] = None

    def __call__(self, session: "Session") -> bool:
        if self._t0 is None:
            self._t0 = session.t
        return session.t - self._t0 >= self.seconds - 1e-9


# ----------------------------------------------------------------- session


class Session:
    """A plant session: the cell opened as the plant of the controllers in
    `links` (and of `hosts` run with no link — a loopback host, or one
    whose writes come through `apply`). Each `tick` asks every link for
    its writes, hands them to the plant, and scans; `run` ticks until a
    predicate holds, a duration passes, or — with neither — the plant's
    own programs end. Deterministic: no wall clock, and every input is
    recorded (`recording()`).

    The session is also the hand on the plant while it runs: `pause` /
    `resume` / `step` hold and release the scan loop (the controller keeps
    polling a frozen world), `speed` scales a realtime run, `press` pushes
    a panel button (its lane high for `PRESS_HOLD_S`), `break_on` pauses
    at a lane's edge, `record` / `stop_record` cut a clip of the run as a
    timeline of its own. With a studio connected to the scene the run
    streams there live, the dock's plant controls and the I/O table's
    force buttons send the same commands back, a stopped recording lands
    on the dock as a playable, exportable clip, and `finish` publishes the
    whole run. `studio=False` keeps a session off the wire to the studio."""

    #: How long a pressed button stays high, in plant seconds.
    PRESS_HOLD_S = 0.3
    _WINDOW_S = 0.1
    _STATUS_S = 0.5

    def __init__(
        self,
        scene,
        sequences: Union[str, list[str]],
        *,
        links: Optional[dict[str, Link]] = None,
        hosts: Optional[Iterable[str]] = None,
        loopback: bool = False,
        cycle: bool = False,
        pace: str = "sync",
        speed: float = 1.0,
        dt: float = 0.01,
        max_duration: float = 120.0,
        scenario: Optional[str] = None,
        physics=None,
        studio: bool = True,
    ):
        if pace not in ("sync", "free", "realtime"):
            raise ValueError(f"pace must be 'sync', 'free' or 'realtime', not {pace!r}")
        if not (speed > 0.0):
            raise ValueError(f"speed must be positive, got {speed}")
        self.pace = pace
        self._speed = float(speed)
        self._clock: Optional[float] = None
        self._lags: list[float] = []
        self._overruns = 0
        self._links: dict[str, Link] = dict(links or {})
        external = list(self._links)
        for h in hosts or ():
            if h not in external:
                external.append(h)
        if not external:
            raise ValueError("a session needs an external host: links={host: link} or hosts=[...]")
        self.scene = scene
        self.sequences = [sequences] if isinstance(sequences, str) else list(sequences)
        self.dt = dt
        # A link that streams a robot's joints (`robots`) makes the plant
        # mirror that robot instead of driving it.
        self._streams: list[tuple[str, str, Link]] = [
            (host, robot, link) for host, link in self._links.items() for robot in getattr(link, "robots", ())
        ]
        self._live = scene.open_plant(
            self.sequences, external, loopback=loopback, cycle=cycle, dt=dt,
            max_duration=max_duration, scenario=scenario, physics=physics,
            joints=[robot for _, robot, _ in self._streams] or None,
        )
        self._ticks = 0
        self._finished = None
        # The hand on the plant.
        self._paused = False
        self._step_budget = 0
        self._halted: Optional[str] = None
        self._breaks: list[tuple[str, bool]] = []
        self._presses: dict[str, float] = {}
        self._forced: dict[str, bool] = {}
        self._recording_from: Optional[float] = None
        self._clips: list[tuple[str, object]] = []
        # The studio side.
        self._studio = studio
        self._following = True
        self._sent = 0.0
        self._last_window_wall = 0.0
        self._last_status_wall = 0.0
        self._status_dirty = True
        self._viewed = False
        self._buttons = [
            p["target"] for p in scene.parts()
            if p.get("kind") == "sensor" and str(p.get("category") or "").startswith("hmi.button")
        ]
        # A link that listens or connects comes up now, before the controller
        # on the other end is told to start.
        for link in self._links.values():
            start = getattr(link, "start", None)
            if callable(start):
                start()

    # -- reading ------------------------------------------------------------

    @property
    def t(self) -> float:
        """The plant clock after the last scan."""
        return self._live.t

    @property
    def hosts(self) -> list[str]:
        return list(self._live.hosts)

    @property
    def finished(self) -> bool:
        """Every program the plant scans has ended (never true while an
        external controller's program is still its own)."""
        return self._live.finished

    def exchange(self, host: str) -> Exchange:
        return Exchange(self._live.exchange(host))

    def outputs(self, host: str) -> dict[str, Value]:
        return dict(self._live.outputs(host))

    def signal(self, name: str) -> bool:
        return bool(self._live.signal(name))

    def word(self, name: str) -> float:
        """A word lane's current value — an encoder (`feed.encoder`), an
        identification reader (`reader.id`)."""
        return float(self._live.word(name))

    def steps(self) -> list[tuple[str, Optional[str]]]:
        return list(self._live.steps())

    # -- driving ------------------------------------------------------------

    def tick(self, n: int = 1) -> float:
        """`n` scans: each asks the links for their writes for the scan
        about to run, applies them, and scans. In `realtime` pace a scan
        waits for its wall-clock moment (`dt / speed` apart); the lag it
        started with is recorded (`stats()`). A direct `tick` always scans,
        paused or not — it is the step."""
        import time

        for _ in range(n):
            self._service()
            next_t = 0.0 if self._ticks == 0 else self._live.t + self.dt
            if self.pace == "realtime":
                if self._clock is None:
                    self._clock = time.monotonic() - next_t / self._speed
                due = self._clock + next_t / self._speed
                now = time.monotonic()
                if due > now:
                    time.sleep(due - now)
                    lag = 0.0
                else:
                    lag = now - due
                    if lag > self.dt / self._speed:
                        self._overruns += 1
                self._lags.append(lag)
            for host, link in self._links.items():
                writes = link.exchange(host, next_t, self._live.outputs(host))
                if not writes:
                    continue
                ordered = getattr(writes, "ordered", None)
                if ordered is not None:
                    for label, value in ordered:
                        self._live.apply(host, {label: value})
                else:
                    self._live.apply(host, dict(writes))
            # The controller's joints and its word on being busy.
            for _, robot, link in self._streams:
                q = link.joints()
                if q is not None:
                    self._live.command(q, robot)
                idle = getattr(link, "idle", None)
                if idle is not None:
                    self._live.report_idle(robot, bool(idle))
            watched = {lane: self._live.signal(lane) for lane, _ in self._breaks}
            self._live.tick()
            self._ticks += 1
            self._after_tick(watched)
        return self._live.t

    def apply(self, host: str, inputs: dict[str, Value]) -> None:
        """Writes a host's inputs by hand (a controller without a link),
        for the next scan."""
        self._live.apply(host, inputs)

    def stats(self) -> dict:
        """How the session kept time: scans run, and in `realtime` pace the
        mean and worst lag behind the wall clock (s) and the scans that
        started more than one period late."""
        lags = self._lags
        return {
            "pace": self.pace,
            "speed": self._speed,
            "scans": self._ticks,
            "t": self._live.t,
            "lag_mean_s": (sum(lags) / len(lags)) if lags else 0.0,
            "lag_max_s": max(lags) if lags else 0.0,
            "overruns": self._overruns,
        }

    def run(
        self,
        until: Optional[Callable[["Session"], bool]] = None,
        *,
        for_: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> float:
        """Ticks until `until(session)` holds, or `for_` seconds of plant
        time pass, or — with neither — every program the plant scans has
        ended. `timeout` (plant seconds) raises `TimeoutError` instead of
        waiting forever on a controller that never answers. While paused
        (`pause`, a breakpoint, the studio's hold) the loop idles — the
        plant stands, the controller polls it as it stands — and goes on
        when resumed or stepped. Returns the clock."""
        t0 = self._live.t
        first = True
        while True:
            # `until` is judged before the hold: `run(until=lambda s: s.paused)`
            # returns the moment a breakpoint lands.
            if until is not None and until(self):
                return self._live.t
            if self._paused and self._step_budget == 0:
                self._idle()
                continue
            if for_ is not None and not first and self._live.t - t0 >= for_ - 1e-9:
                return self._live.t
            if until is None and for_ is None and self._ticks > 0 and self._live.finished:
                return self._live.t
            if timeout is not None and self._live.t - t0 > timeout + 1e-9:
                raise TimeoutError(
                    f"the plant ran {timeout} s past t = {t0:.2f} s without `until` holding "
                    f"(steps: {self.steps()})"
                )
            self.tick()
            first = False
            if self._step_budget > 0:
                self._step_budget -= 1
                if self._step_budget == 0:
                    self._status_dirty = True

    # -- the transport --------------------------------------------------------

    @property
    def paused(self) -> bool:
        """Whether the plant is held (not while `step` scans are owed)."""
        return self._paused and self._step_budget == 0

    @property
    def halted(self) -> Optional[str]:
        """Why the session paused on its own — the breakpoint it hit
        (`↑part_at_pick at 12.34 s`) — or `None`."""
        return self._halted

    def pause(self) -> None:
        """Holds the plant where it stands: `run` idles, the controller
        keeps polling a frozen world and its writes wait for the next scan."""
        self._paused = True
        self._step_budget = 0
        self._status_dirty = True

    def resume(self) -> None:
        self._paused = False
        self._step_budget = 0
        self._halted = None
        self._rebase_clock()
        self._status_dirty = True

    def step(self, n: int = 1) -> None:
        """`n` more scans, then hold again (from a running loop: run `n`
        scans past now and pause)."""
        if n < 1:
            raise ValueError("step takes at least one scan")
        self._paused = True
        self._halted = None
        self._step_budget = int(n)
        self._rebase_clock()
        self._status_dirty = True

    @property
    def speed(self) -> float:
        """The realtime speed factor (plant seconds per wall second)."""
        return self._speed

    @speed.setter
    def speed(self, value: float) -> None:
        if not (value > 0.0):
            raise ValueError(f"speed must be positive, got {value}")
        self._speed = float(value)
        self._rebase_clock()
        self._status_dirty = True

    def _rebase_clock(self) -> None:
        """The next scan is due now: the wall clock does not count while
        the plant is held, and a new speed starts from here."""
        import time

        if self.pace == "realtime" and self._clock is not None:
            next_t = 0.0 if self._ticks == 0 else self._live.t + self.dt
            self._clock = time.monotonic() - next_t / self._speed

    def _idle(self) -> None:
        import time

        self._service()
        self._maybe_status()
        time.sleep(0.02)
        self._rebase_clock()

    # -- the hand on the wire -----------------------------------------------

    def force(self, name: str, value: bool) -> None:
        """Pins a lane (a PLC force) until `unforce`."""
        self._live.force(name, value)
        self._forced[name] = bool(value)
        self._status_dirty = True

    def unforce(self, name: str) -> None:
        self._live.unforce(name)
        self._forced.pop(name, None)
        self._presses.pop(name, None)
        self._status_dirty = True

    release = unforce

    @property
    def forced(self) -> dict[str, bool]:
        """The lanes pinned now, with their level."""
        return dict(self._forced)

    def press(self, lane: str, hold_s: Optional[float] = None) -> None:
        """Presses a push button: its lane (a panel button's zone sensor,
        or any lane) is forced high now and released after `hold_s` plant
        seconds (`PRESS_HOLD_S` by default) — long enough for a polled
        controller to see the edge."""
        hold = self.PRESS_HOLD_S if hold_s is None else float(hold_s)
        if not (hold > 0.0):
            raise ValueError(f"hold_s must be positive, got {hold}")
        self.force(lane, True)
        self._presses[lane] = self._live.t + hold

    @property
    def buttons(self) -> list[str]:
        """The lanes a click presses: the cell's panel buttons (sensors
        pinned as `hmi.button`)."""
        return list(self._buttons)

    def break_on(self, lane: str, rising: bool = True) -> None:
        """Pauses the session the scan `lane` takes its rising (or
        falling) edge — a breakpoint on a signal."""
        self._live.signal(lane)  # an unknown lane is refused by name
        key = (lane, bool(rising))
        if key not in self._breaks:
            self._breaks.append(key)
        self._status_dirty = True

    def clear_break(self, lane: str, rising: Optional[bool] = None) -> None:
        self._breaks = [
            b for b in self._breaks if not (b[0] == lane and (rising is None or b[1] == rising))
        ]
        self._status_dirty = True

    @property
    def breaks(self) -> list[tuple[str, bool]]:
        return list(self._breaks)

    def inject(self, fault: dict, at: Optional[Union[_Predicate, dict]] = None) -> None:
        """Injects `bt.io.stuck / open / node_down` now, or at a `rising` /
        `step` anchor."""
        anchor = at.anchor if isinstance(at, _Predicate) else at
        if isinstance(at, _Predicate) and anchor is None:
            raise ValueError(f"{type(at).__name__} is not an anchor — use rising(...) or step(...)")
        self._live.inject(fault, anchor)

    # -- recording clips -------------------------------------------------------

    def record(self) -> None:
        """Starts a clip at the current scan (a running one goes on)."""
        if self._recording_from is None:
            self._recording_from = self._live.t
            self._status_dirty = True

    @property
    def recording_from(self) -> Optional[float]:
        return self._recording_from

    def stop_record(self, publish: Optional[bool] = None):
        """Ends the clip and returns it: the stretch since `record` as a
        `SequenceTimeline` starting at zero (`clips` keeps them all). With
        a studio watching (or `publish=True`) the clip lands on its dock —
        playable, exportable as USD — and the dock stops following the
        live plant until `follow`. `None` without a recording running."""
        start = self._recording_from
        if start is None:
            return None
        self._recording_from = None
        end = self._live.t
        clip = self._live.clip(start, end)
        label = f"clip {start:.2f}–{end:.2f} s"
        self._clips.append((label, clip))
        if publish is None:
            publish = self._studio and self._live.has_viewers()
        if publish:
            self._live.publish(clip, label)
            self._following = False
        self._status_dirty = True
        return clip

    @property
    def clips(self) -> list:
        """Every clip cut so far, in order."""
        return [clip for _, clip in self._clips]

    def follow(self) -> None:
        """The studio follows the live plant again (after a clip)."""
        if not self._following:
            self._following = True
            self._sent = max(0.0, self._live.t - 2.0)
            self._last_window_wall = 0.0
        self._status_dirty = True

    @property
    def following(self) -> bool:
        return self._following

    # -- the studio ------------------------------------------------------------

    def status(self) -> dict:
        """The session as the studio's dock shows it: the hosts and their
        links, the clock and the lag, what is paused, forced, broken on,
        recorded."""
        hosts = []
        for name in self.hosts:
            link = self._links.get(name)
            if link is None:
                described, exchanges, error = "loopback", 0, None
            else:
                describe = getattr(link, "describe", None)
                described = describe() if callable(describe) else type(link).__name__
                exchanges = int(getattr(link, "polls", 0) or getattr(link, "exchanges", 0) or 0)
                err = getattr(link, "_error", None)
                error = None if err is None else str(err)
            hosts.append({"name": name, "link": described, "exchanges": exchanges, "error": error})
        lags = self._lags
        return {
            "label": f"{' + '.join(self.sequences)} (plant of {', '.join(self.hosts)})",
            "t": self._live.t,
            "dt": self.dt,
            "pace": self.pace,
            "speed": self._speed,
            "paused": self._paused,
            "halted": self._halted,
            "hosts": hosts,
            "lag_ms": 1000.0 * (sum(lags) / len(lags)) if lags else 0.0,
            "lag_max_ms": 1000.0 * max(lags) if lags else 0.0,
            "overruns": self._overruns,
            "forced": [[lane, value] for lane, value in self._forced.items()],
            "breaks": [{"lane": lane, "rising": rising} for lane, rising in self._breaks],
            "recording_from": self._recording_from,
            "following": self._following,
            "buttons": list(self._buttons),
            "clips": [label for label, _ in self._clips],
        }

    def _service(self) -> None:
        """Applies what the studio asked for since the last scan."""
        if not self._studio:
            return
        for raw in self._live.take_studio_commands():
            try:
                self._apply_command(json.loads(raw))
            except Exception as e:  # noqa: BLE001 - a bad command must not stop the plant
                import warnings

                warnings.warn(f"studio command {raw}: {e}", stacklevel=2)
                self._status_dirty = True

    def _apply_command(self, cmd: dict) -> None:
        kind = cmd.get("command")
        if kind == "pause":
            self.pause()
        elif kind == "resume":
            self.resume()
        elif kind == "step":
            self.step(int(cmd.get("n", 1)))
        elif kind == "speed":
            self.speed = float(cmd["speed"])
        elif kind == "force":
            self.force(cmd["lane"], bool(cmd["value"]))
        elif kind == "unforce":
            self.unforce(cmd["lane"])
        elif kind == "press":
            self.press(cmd["lane"], cmd.get("hold_s"))
        elif kind == "break":
            if cmd.get("on", True):
                self.break_on(cmd["lane"], rising=bool(cmd.get("rising", True)))
            else:
                self.clear_break(cmd["lane"], rising=bool(cmd.get("rising", True)))
        elif kind == "record":
            if cmd.get("on", True):
                self.record()
            else:
                self.stop_record()
        elif kind == "follow":
            self.follow()
        else:
            raise ValueError(f"unknown plant command {kind!r}")

    def _after_tick(self, watched: dict[str, bool]) -> None:
        t = self._live.t
        for lane, until in list(self._presses.items()):
            if t + 1e-9 >= until:
                self.unforce(lane)
        for lane, rising in list(self._breaks):
            before = watched.get(lane)
            now = self._live.signal(lane)
            if before is not None and before != now and now == rising:
                self._paused = True
                self._step_budget = 0
                self._halted = f"{'↑' if rising else '↓'}{lane} at {t:.2f} s"
                self._status_dirty = True
        if self._studio:
            self._maybe_broadcast()

    def _maybe_broadcast(self) -> None:
        import time

        now = time.monotonic()
        if (
            self._following
            and self._live.t - self._sent >= self._WINDOW_S - 1e-9
            and now - self._last_window_wall >= 0.033
        ):
            if self._live.has_viewers():
                self._viewed = True
            self._sent = self._live.broadcast(self._sent)
            self._last_window_wall = now
        self._maybe_status()

    def _maybe_status(self) -> None:
        import time

        now = time.monotonic()
        if self._status_dirty or now - self._last_status_wall >= self._STATUS_S:
            if self._live.has_viewers():
                self._live.publish_status(json.dumps(self.status()))
            self._status_dirty = False
            self._last_status_wall = now

    # -- results ------------------------------------------------------------

    def recording(self) -> Recording:
        """Every input written so far."""
        return Recording(list(self._live.input_log()))

    def trace(self, host: Optional[str] = None) -> Trace:
        """The boolean inputs written so far as a trace."""
        return self.recording().trace(host)

    def timeline(self):
        """The timeline so far, without closing the session."""
        return self._live.timeline()

    def finish(self, publish: Optional[bool] = None):
        """Closes the session into its timeline (the ordinary
        `SequenceTimeline`); `publish` broadcasts it to open studios — by
        default when a studio followed the run."""
        if self._finished is None:
            watching = self._studio and self._live.has_viewers()
            if publish is None:
                publish = self._following and (watching or self._viewed)
            if watching and self._following:
                self._live.broadcast(self._sent, done=True)
            if watching:
                self._live.publish_status(None)
            self._finished = self._live.finish(publish=bool(publish))
        return self._finished

    def __enter__(self) -> "Session":
        return self

    def __exit__(self, *exc) -> None:
        for link in self._links.values():
            stop = getattr(link, "stop", None)
            if callable(stop):
                try:
                    stop()
                except Exception:  # noqa: BLE001 - a link that is already down
                    pass
        if self._finished is None and exc[0] is None:
            self.finish()


def loopback(scene, sequences: Union[str, list[str]], host: Union[str, Iterable[str]], **kw) -> Session:
    """A session whose external host(s) botrail runs itself, through the
    exchange — the reference run that must equal the bake."""
    hosts = [host] if isinstance(host, str) else list(host)
    return Session(scene, sequences, hosts=hosts, loopback=True, **kw)


# The FAT — tests as a table (VC4). Imported last: the module reads the
# session and predicates above.
from .fat import (  # noqa: E402
    CaseResult,
    Check,
    Expectation,
    FatReport,
    TestCase,
    cases_from_data,
    cases_to_data,
    check,
    clearance,
    cycle_within,
    falls,
    held,
    interlock_tests,
    load_cases,
    never,
    no_motion,
    rises,
    run_fat,
    safety_tests,
    stalls,
    step_reached,
)
