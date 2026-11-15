"""Monitoring: the cell as the mirror of a controller that runs a real
machine (design-plc-resolution.md §11, PM0 / PM1).

A plant session (`bt.vc.Session`) *answers* a controller under test: it
writes the controller's inputs. A monitor never answers. It takes the
controller's I/O image as it was read — from a machine's log, or live
through a read-only reader — and does three things with it:

* what the controller **wrote** (coils, device commands, robot starts)
  drives the model, so the 3D cell moves as the machine was told to;
* what the controller **read** and the model computes itself (sensors,
  limit switches, feedback words) is *compared* with the model's lanes —
  never written over them. A disagreement that outlasts the tolerance is a
  deviation: the machine's sensor came late, never came, reads a level the
  model does not;
* what the controller read and the model has no source for (a push
  button, a selector, a setpoint keyed in at the panel) is followed: the
  model's lane takes the read value.

A log, replayed (deterministic, as fast as it computes)::

    xc = bt.vc.exchange(scene, "PLC1", programs)
    log = bt.vc.IoLog.load("line_io.csv", xc)
    with bt.vc.monitor(scene, programs, log) as m:
        m.run()                                   # to the end of the log
        for d in m.deviations():
            print(d.note)                         # `s2` rose 0.50 s later on the machine than in the model
        tl = m.timeline()                         # the model's run, with the readings on it

A running controller, read only (wall-clock time)::

    reader = bt.vc.modbus_reader(xc, "10.0.0.5:502", period=0.2)
    with bt.vc.monitor(scene, programs, reader) as m:
        m.run(for_=600.0)
        m.log().save("shift.csv")                 # what was read: replays later, exactly

The monitor has no `force`, `press`, `write`, `inject` or `apply`, and
refuses those commands from a studio: nothing goes back to the controller,
by the reader's construction (`readers.py`) and by this class's. The model
is an approximation: the 3D shows what the controller sees and how the
model moves under its commands, not a photograph of the machine. Start a
live monitor with the machine in the state the scene is built in — the
model does not know what happened before it began to read.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Callable, Optional, Union

from . import Session
from .iolog import IoLog

Value = Union[bool, int, float]

__all__ = ["Deviation", "Monitor", "monitor"]

#: Sources of a controller input nothing in the model drives.
_FOLLOWED = ("signal:read-only", "declared")
#: How much of the readings a live status carries, seconds.
_STATUS_WINDOW_S = 60.0


@dataclass
class Deviation:
    """Where the machine's reading of a point and the model's lane parted
    for longer than the tolerance: `kind` is `late` / `early` (the
    machine's edge against the model's), `level` (the two stand apart, or
    one pulsed alone), `value` (a word outside its band); `open` while
    they still disagree."""

    lane: str
    start: float
    end: float
    kind: str
    note: str
    open: bool = False

    @property
    def duration(self) -> float:
        return self.end - self.start

    def to_dict(self) -> dict:
        return {"lane": self.lane, "start": self.start, "end": self.end, "kind": self.kind, "note": self.note}


class _Cursor:
    """A log's image at a time that only moves forward."""

    def __init__(self, log: IoLog):
        self._series = log.series
        self._at = {k: 0 for k in log.series}
        self._image: dict[str, Value] = {}

    def image(self, t: float) -> dict[str, Value]:
        for label, samples in self._series.items():
            i = self._at[label]
            while i < len(samples) and samples[i][0] <= t + 1e-6:
                self._image[label] = samples[i][1]
                i += 1
            self._at[label] = i
        return self._image


class _LogSource:
    """An `IoLog` as a source: the image at the scan's time, and — for
    what is compared with the model as the last scan left it — the image
    at that scan's."""

    def __init__(self, log: IoLog):
        self.log = log
        self.period = log.period
        self._now = _Cursor(log)
        self._then = _Cursor(log)

    def describe(self) -> str:
        return self.log.source

    def sample(self, t: float) -> Optional[dict[str, Value]]:
        return self._now.image(t) or None

    def lagged(self, t: float) -> dict[str, Value]:
        return self._then.image(t)


class _Track:
    """One compared point: the read series, and the disagreement under way."""

    def __init__(self, point, lane: str):
        self.point = point
        self.lane = lane
        self.is_bool = point.value_type == "BOOL"
        self.series: list[tuple[float, Value]] = []
        self.model: Optional[Value] = None
        self.read: Optional[Value] = None
        self.since: Optional[float] = None     # the disagreement began here
        self.leader: Optional[str] = None      # "model" | "machine" | None (apart from the first reading)
        self.edge: Optional[bool] = None       # the level the leader went to
        self.worst = 0.0                       # a word's largest gap in the stretch


class _Feed:
    """The session's side of a monitor: each scan it hands the plant what
    the controller wrote, follows what the model cannot know, and compares
    the rest. It sends nothing — the source reads, and that is all."""

    def __init__(self, source, exchange, *, tolerance: float, bands: dict[str, float],
                 follow: Optional[list[str]], buttons: frozenset = frozenset()):
        self.source = source
        self.tolerance = tolerance
        self.bands = bands
        self._live = None
        self._written: dict[str, object] = {}
        self._followed: dict[str, object] = {}
        self._tracks: dict[str, _Track] = {}
        for p in exchange.points:
            if p.side == "input":
                self._written[p.label] = p
            elif (p.label in follow) if follow is not None else (p.source in _FOLLOWED or p.label in buttons):
                self._followed[p.label] = p
            else:
                self._tracks[p.label] = _Track(p, p.label + ".raw" if p.source == "analog" else p.label)
        unknown = [k for k in (follow or ()) if k not in self._followed]
        if unknown:
            raise ValueError(f"monitor: follow={unknown} — not points {exchange.host} reads")
        self._applied: dict[str, Value] = {}
        self._seen: dict[str, list[tuple[float, Value]]] = {}
        self._closed: list[Deviation] = []
        self._last: Optional[float] = None
        self.t = 0.0
        #: Scans that had an image to work with.
        self.exchanges = 0

    # -- the link protocol ----------------------------------------------------

    def start(self) -> None:
        start = getattr(self.source, "start", None)
        if callable(start):
            start()

    def stop(self) -> None:
        stop = getattr(self.source, "stop", None)
        if callable(stop):
            stop()

    def describe(self) -> str:
        return self.source.describe()

    @property
    def _error(self):
        return getattr(self.source, "_error", None)

    @property
    def polls(self) -> int:
        return int(getattr(self.source, "polls", 0) or self.exchanges)

    def exchange(self, host: str, t: float, outputs: dict[str, Value]) -> dict[str, Value]:
        # The outputs handed over for scan `t` are the model as the scan
        # before left it; a reading is compared with them at that time.
        then = t if self._last is None else self._last
        self._last = t
        self.t = then
        image = self.source.sample(t)
        if not image:
            return {}
        self.exchanges += 1
        lagged = getattr(self.source, "lagged", None)
        compared = lagged(then) if callable(lagged) else image
        # What was read, kept as a log would have it: a write at the scan
        # it lands in, a reading at the scan it is compared with.
        for label, value in image.items():
            s = self._seen.setdefault(label, [])
            if not s or s[-1][1] != value:
                s.append((t if label in self._written else then, value))
        writes: dict[str, Value] = {}
        for label in self._written:
            value = image.get(label)
            if value is not None and self._applied.get(label) != value:
                self._applied[label] = value
                writes[label] = value
        # A followed lane changes when it is pinned — the model's clock is
        # still the last scan's, so the reading of that scan lands on it.
        for label, p in self._followed.items():
            value = compared.get(label)
            if value is None or self._applied.get(label) == value:
                continue
            self._applied[label] = value
            if p.value_type == "BOOL":
                self._live.force(label, bool(value) != bool(p.invert))
            else:
                self._live.force_word(label, _si(value, p))
        for label, track in self._tracks.items():
            read = compared.get(label)
            model = outputs.get(label)
            if read is None or model is None:
                continue
            self._judge(track, then, _lane(model, track.point), _lane(read, track.point))
        return writes

    # -- the comparison ---------------------------------------------------------

    def _band(self, track: _Track, model: float, read: float) -> float:
        band = self.bands.get(track.lane, self.bands.get(track.point.label))
        if band is not None:
            return band
        return max(2.0 if track.point.value_type == "INT" and _scale(track.point) == 1.0 else 0.0,
                   0.02 * max(abs(model), abs(read)))

    def _judge(self, track: _Track, t: float, model: Value, read: Value) -> None:
        if not track.series or track.series[-1][1] != read:
            track.series.append((t, read))
        before_model, before_read = track.model, track.read
        track.model, track.read = model, read
        if track.is_bool:
            apart, gap = (model != read), 0.0
        else:
            gap = abs(float(model) - float(read))
            apart = gap > self._band(track, float(model), float(read))
        if apart and track.since is None:
            track.since, track.worst = t, gap
            if before_model is None:
                track.leader, track.edge = None, None
            elif model != before_model and read == before_read:
                track.leader, track.edge = "model", bool(model) if track.is_bool else None
            elif read != before_read and model == before_model:
                track.leader, track.edge = "machine", bool(read) if track.is_bool else None
            else:
                track.leader, track.edge = None, None
        elif apart:
            track.worst = max(track.worst, gap)
        elif track.since is not None:
            if t - track.since > self.tolerance + 1e-9:
                # Who moved to end it: the follower catching up, or the
                # leader going back (a pulse the other side never had).
                caught_up = (track.leader == "model" and read != before_read) or (
                    track.leader == "machine" and model != before_model)
                self._closed.append(self._deviation(track, t, open_=False, caught_up=caught_up))
            track.since, track.leader, track.edge = None, None, None

    def _deviation(self, track: _Track, end: float, *, open_: bool, caught_up: bool = False) -> Deviation:
        lane, start, span = track.lane, track.since, end - track.since
        if not track.is_bool:
            note = (f"`{lane}` reads {_num(track.read)} on the machine, the model has {_num(track.model)} "
                    f"(apart by up to {_num(track.worst)} for {span:.2f} s)")
            if not open_:
                note = f"`{lane}` was apart from the model by up to {_num(track.worst)} for {span:.2f} s"
            return Deviation(lane, start, end, "value", note, open_)
        moved = "rose" if track.edge else "fell"
        if track.leader is None:
            if open_:
                note = (f"`{lane}` reads {_level(track.read)} on the machine, {_level(track.model)} in the model "
                        f"(since {start:.2f} s)")
            else:
                note = f"`{lane}` stood apart from the model for {span:.2f} s (from {start:.2f} s)"
            return Deviation(lane, start, end, "level", note, open_)
        first, second = ("the model", "the machine") if track.leader == "model" else ("the machine", "the model")
        if open_:
            where = "in the model" if track.leader == "model" else "on the machine"
            kind = "late" if track.leader == "model" else "early"
            return Deviation(lane, start, end, kind,
                             f"`{lane}` {moved} {where} at {start:.2f} s; {second} has not followed", open_)
        if not caught_up:
            where = "in the model" if track.leader == "model" else "on the machine"
            return Deviation(lane, start, end, "level",
                             f"`{lane}` was {_level(track.edge)} for {span:.2f} s {where} only (from {start:.2f} s)",
                             open_)
        if track.leader == "model":
            return Deviation(lane, start, end, "late",
                             f"`{lane}` {moved} {span:.2f} s later on the machine than in the model "
                             f"({start:.2f} → {end:.2f} s)", open_)
        return Deviation(lane, start, end, "early",
                         f"`{lane}` {moved} {span:.2f} s earlier on the machine than in the model "
                         f"({start:.2f} → {end:.2f} s)", open_)

    def deviations(self) -> list[Deviation]:
        out = list(self._closed)
        for track in self._tracks.values():
            if track.since is not None and self.t - track.since > self.tolerance + 1e-9:
                out.append(self._deviation(track, self.t, open_=True))
        out.sort(key=lambda d: (d.start, d.lane))
        return out

    def readings(self, host: str, since: Optional[float] = None) -> dict:
        """The compared readings and the deviations as a `ReadingsMsg`."""
        signals, words = [], []
        for track in self._tracks.values():
            series = track.series
            if not series:
                continue
            if since is not None:
                kept = [s for s in series if s[0] >= since]
                before = [s for s in series if s[0] < since]
                series = before[-1:] + kept
            times = [t for t, _ in series]
            if track.is_bool:
                signals.append({"name": track.lane, "times": times, "values": [bool(v) for _, v in series],
                                "kind": "read"})
            else:
                words.append({"name": track.lane, "times": times, "values": [float(v) for _, v in series],
                              "kind": "read",
                              "integer": track.point.value_type == "INT" and _scale(track.point) == 1.0})
        deviations = [d.to_dict() for d in self.deviations() if since is None or d.end >= since]
        return {"host": host, "source": self.describe(), "tolerance": self.tolerance,
                "signals": signals, "words": words, "deviations": deviations}


def _scale(point) -> float:
    scale = float(getattr(point, "scale", 1.0) or 1.0)
    return scale if scale > 0.0 else 1.0


def _si(value: Value, point) -> float:
    return float(value) / _scale(point)


def _lane(value: Value, point) -> Value:
    """A controller-side value as the model's lane holds it: an NC wire
    turned back, a scaled register in its unit."""
    if point.value_type == "BOOL":
        return bool(value) != bool(point.invert)
    return _si(value, point)


def _level(value) -> str:
    return "on" if value else "off"


def _num(value) -> str:
    return f"{float(value):.6g}"


class _Core(Session):
    """The plant session under a monitor: the same scan loop and stream to
    the studio, with the hands taken off."""

    _refused: Optional[str] = None
    _feed: Optional[_Feed] = None

    def _apply_command(self, cmd: dict) -> None:
        kind = cmd.get("command")
        # A clip of the stream and the dock's follow touch neither the
        # controller nor the model.
        if kind in ("record", "follow"):
            super()._apply_command(cmd)
            return
        self._refused = f"{kind}: a monitor only reads — nothing is forced, pressed or written"
        self._status_dirty = True

    def status(self) -> dict:
        status = super().status()
        host = self.hosts[0]
        status["label"] = f"{' + '.join(self.sequences)} (monitor of {', '.join(self.hosts)})"
        status["read_only"] = True
        status["buttons"] = []
        status["forced"] = []
        status["forced_words"] = []
        status["breaks"] = []
        if self._feed is not None:
            status["readings"] = self._feed.readings(host, since=max(0.0, self.t - _STATUS_WINDOW_S))
        if self._refused:
            status["refused"] = self._refused
        return status


class Monitor:
    """The cell mirroring a controller from what was read of it — see the
    module's documentation. `source` is an `IoLog` (replayed in plant
    time) or a reader (`bt.vc.modbus_reader`, `bt.vc.opcua_reader`;
    followed in wall-clock time). `sequences` are the cell's programs as
    for a plant session: the monitored controller's own (not scanned —
    the controller runs them) and whatever the model scans itself (the
    equipment's programs); leave out a script that stands in for the
    operator, whose buttons the controller reports.

    `tolerance` is how long a reading and the model may disagree before it
    counts (0.1 s, or twice the source's period when that is longer — an
    edge read by polling is that much late); `bands={lane: gap}` is how
    far a word may sit from the model's (2 % of its value, and 2 counts on
    an integer, by default); `follow=[...]` names the read points the
    model takes from the controller instead of comparing (by default the
    ones nothing in the model drives, and the panel's push buttons)."""

    def __init__(
        self,
        scene,
        sequences: Union[str, list[str]],
        source,
        *,
        host: Optional[str] = None,
        tolerance: Optional[float] = None,
        bands: Optional[dict[str, float]] = None,
        follow: Optional[list[str]] = None,
        dt: float = 0.01,
        max_duration: Optional[float] = None,
        physics=None,
        studio: bool = True,
    ):
        sequences = [sequences] if isinstance(sequences, str) else list(sequences)
        self._log = source if isinstance(source, IoLog) else None
        feed_source = _LogSource(source) if isinstance(source, IoLog) else source
        if not callable(getattr(feed_source, "sample", None)):
            raise TypeError(
                "monitor: the source is an IoLog or a reader (bt.vc.modbus_reader, bt.vc.opcua_reader) — "
                f"not {type(source).__name__}. A link that writes the controller's inputs is for a controller "
                "under test (bt.vc.Session)."
            )
        host = host or getattr(source, "host", None)
        if not host:
            raise ValueError("monitor: which controller? pass host=")
        xc = scene.exchange(host, sequences)
        period = float(getattr(feed_source, "period", 0.0) or 0.0)
        self.tolerance = float(tolerance) if tolerance is not None else max(0.1, 2.0 * period)
        if not (self.tolerance >= 0.0):
            raise ValueError(f"monitor: tolerance must not be negative, got {tolerance}")
        self.host = host
        self.source = source
        # A panel button is a sensor in the model (a hand in its zone), but
        # on a machine the hand is the operator's: followed, like a relay.
        buttons = frozenset(
            p["target"] for p in scene.parts()
            if p.get("kind") == "sensor" and str(p.get("category") or "").startswith("hmi.button")
        )
        self._feed = _Feed(feed_source, xc, tolerance=self.tolerance, bands=dict(bands or {}), follow=follow,
                           buttons=buttons)
        if max_duration is None:
            max_duration = (self._log.duration + 10.0) if self._log is not None else 3600.0
        self._core = _Core(
            scene, sequences, links={host: self._feed}, pace="sync" if self._log is not None else "realtime",
            dt=dt, max_duration=max_duration, physics=physics, studio=studio,
        )
        self._core._feed = self._feed
        self._feed._live = self._core._live
        self._scene = scene
        self._finished = None
        known = {p.label for p in xc.points}
        carried = set(self._log.series) if self._log is not None else known
        #: Points of the source the cell's exchange does not have (ignored).
        self.ignored: list[str] = sorted(carried - known)
        #: Points of the exchange the source does not carry (not driven, not compared).
        self.missing: list[str] = sorted(known - carried)

    # -- reading ------------------------------------------------------------

    @property
    def t(self) -> float:
        """The model's clock after the last scan."""
        return self._core.t

    @property
    def pace(self) -> str:
        return self._core.pace

    @property
    def compared(self) -> list[str]:
        """The points whose readings are compared with the model's."""
        return list(self._feed._tracks)

    @property
    def followed(self) -> list[str]:
        """The read points the model takes from the controller."""
        return list(self._feed._followed)

    @property
    def driving(self) -> list[str]:
        """The points the controller writes, which move the model."""
        return list(self._feed._written)

    def signal(self, name: str) -> bool:
        return self._core.signal(name)

    def word(self, name: str) -> float:
        return self._core.word(name)

    def steps(self) -> list[tuple[str, Optional[str]]]:
        return self._core.steps()

    def stats(self) -> dict:
        """How the monitor kept time, and how much it read."""
        stats = self._core.stats()
        stats["reads"] = self._feed.polls
        error = self._feed._error
        stats["error"] = None if error is None else str(error)
        return stats

    def status(self) -> dict:
        """The monitor as the studio's dock shows it."""
        return self._core.status()

    # -- running ------------------------------------------------------------

    def tick(self, n: int = 1) -> float:
        return self._core.tick(n)

    def run(
        self,
        until: Optional[Callable[["Monitor"], bool]] = None,
        *,
        for_: Optional[float] = None,
        timeout: Optional[float] = None,
    ) -> float:
        """Scans until `until(monitor)` holds or `for_` seconds pass. With
        neither, a log runs to its end; a live reader has no end, so it
        needs one of the two."""
        if until is None and for_ is None:
            if self._log is None:
                raise ValueError("monitor: a live reader has no end — pass for_= or until=")
            # One scan past the last row: a reading is compared with the
            # model as the scan it belongs to left it.
            end = self._log.duration + self._core.dt
            return self._core.run(until=lambda s: s._ticks > 0 and s.t >= end - 1e-9)
        return self._core.run(until=None if until is None else (lambda s: until(self)), for_=for_, timeout=timeout)

    # -- the stream's own controls (they touch neither controller nor model) ----

    def record(self) -> None:
        self._core.record()

    def stop_record(self, publish: Optional[bool] = None):
        return self._core.stop_record(publish)

    @property
    def clips(self) -> list:
        return self._core.clips

    # -- results ------------------------------------------------------------

    def deviations(self) -> list[Deviation]:
        """Where the readings and the model parted for longer than the
        tolerance, in time order; the ones still apart are `open`."""
        return self._feed.deviations()

    def readings(self) -> dict:
        """The compared readings and the deviations, as the timeline
        carries them (a `ReadingsMsg`)."""
        return self._feed.readings(self.host)

    def log(self) -> IoLog:
        """Everything read so far as an `IoLog` — save it, and the run
        replays exactly (`bt.vc.monitor(scene, programs, log)`)."""
        if self._log is not None:
            return self._log
        xc = self._scene.exchange(self.host, self._core.sequences)
        known = {p.label for p in xc.points}
        from . import Exchange

        return IoLog(Exchange(xc), {k: list(v) for k, v in self._feed._seen.items() if k in known},
                     source=self._feed.describe(), period=float(getattr(self.source, "period", 0.0) or 0.0))

    def timeline(self):
        """The model's run so far, with the readings attached."""
        if self._finished is not None:
            return self._finished
        return self._core.timeline().with_readings(json.dumps(self.readings()))

    def waiting(self) -> list[dict]:
        """What every program waits on now (`SequenceTimeline.waiting`) —
        the monitored controller's too, when its node reports its steps."""
        return self._core.waiting()

    def analog(self) -> list[dict]:
        return self._core.analog()

    def report(self) -> str:
        """The comparison as markdown: what was read, what parted."""
        ds = self.deviations()
        lines = [f"# Monitor — `{self.host}` from {self._feed.describe()}", "",
                 f"- model time: {self.t:.2f} s, tolerance {self.tolerance:.2f} s",
                 f"- drives the model ({len(self.driving)}): " + ", ".join(f"`{k}`" for k in self.driving),
                 f"- compared ({len(self.compared)}): " + ", ".join(f"`{k}`" for k in self.compared),
                 f"- followed ({len(self.followed)}): " + ", ".join(f"`{k}`" for k in self.followed)]
        if self.missing:
            lines.append(f"- not in the source ({len(self.missing)}): " + ", ".join(f"`{k}`" for k in self.missing))
        if self.ignored:
            lines.append(f"- not in the cell ({len(self.ignored)}): " + ", ".join(f"`{k}`" for k in self.ignored))
        lines += ["", f"## Deviations ({len(ds)})", ""]
        if ds:
            lines += ["| lane | kind | from | to | |", "|---|---|---|---|---|"]
            lines += [f"| `{d.lane}` | {d.kind} | {d.start:.2f} | {d.end:.2f}{' …' if d.open else ''} | {d.note} |"
                      for d in ds]
        else:
            lines.append("None: every compared reading agrees with the model within the tolerance.")
        return "\n".join(lines) + "\n"

    def finish(self, publish: Optional[bool] = None):
        """Closes the monitor into its timeline — the model's run with the
        readings and the deviations on it; `publish` shows it in the open
        studios (by default when one followed the run)."""
        if self._finished is None:
            core = self._core
            watching = core._studio and core._live.has_viewers()
            if publish is None:
                publish = core._following and (watching or core._viewed)
            readings = json.dumps(self.readings())
            timeline = core.finish(publish=False).with_readings(readings)
            self._finished = timeline
            if publish:
                self._scene.show_timeline(timeline)
        return self._finished

    def __enter__(self) -> "Monitor":
        return self

    def __exit__(self, *exc) -> None:
        self._feed.stop()
        if self._finished is None and exc[0] is None:
            self.finish()


def monitor(scene, sequences: Union[str, list[str]], source, **kw) -> Monitor:
    """The cell mirroring a controller from a log or a read-only reader
    (see [`Monitor`])."""
    return Monitor(scene, sequences, source, **kw)
