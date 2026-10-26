"""Tests as a table — the FAT (design-virtual-commissioning.md VC4).

A factory acceptance test is a sheet: *get the cell here, do this to it,
and this must follow*. `TestCase` is one such row — a moment (`at`), the
faults, forces and button presses that go in there, and the
**expectations** on what the plant records afterwards — and `run_fat` runs
a list of them and fills the sheet in as a `FatReport`, the same report
whether the cases ran **offline** (botrail's own executor, in loopback
over every host, bit for bit the bake) or **online** (a controller on the
wire through `bt.vc` links)::

    cases = [
        bt.vc.TestCase("TC-01 cycle", expect=[bt.vc.cycle_within(12.0)]),
        bt.vc.TestCase("TC-07 beam open", faults=[bt.io.open("part_at_pick")],
                       at=bt.vc.step("pick", "await part"),
                       expect=[bt.vc.never("vacuum", during=10.0)]),
        *bt.vc.interlock_tests(scene, "pick"),      # one row per interlock-table row
        *bt.vc.safety_tests(scene),                 # every safety input dropped
    ]
    fat = bt.vc.run_fat(scene, cases)               # offline; links={...} for a PLC
    fat.save("cell_fat.md"); assert fat.ok
    report = scene.cell_report({"baseline": tl}, sections=[fat.section()])

Nothing in the derived rows is authored: `interlock_tests` reads the
interlock table — an output of a step and the condition that admits the
step — and for each row pins the condition false (`denial`, the lanes and
levels the table itself names) and expects the step never to be entered
once the program has reached the gate; `safety_tests` reads the I/O map's
safety inputs and expects the cycle refused with each one dropped. The
cases are plain data, so the rows derived from the *design* run against
whatever *implements* it — the same cell with a transition dropped, or
the PLC's own program — and the row whose interlock is missing turns red.

Expectations read the timeline the run recorded, over the window from the
case's anchor to the end of its run: `rises` / `falls` (an edge within a
time), `never` (a lane never high, or a step never entered), `held` (a lane
held at a level), `step_reached`, `stalls` (the cycle refused), `no_motion`,
`clearance`, `cycle_within`, and `check` for a predicate of your own.
pytest keeps the `assert`s; the FAT is the deliverable.
"""

from __future__ import annotations

import copy
import csv
import io as _io
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable, Mapping, Optional, Sequence, Union

import botrail.vc as _vc

__all__ = [
    "Check",
    "Expectation",
    "TestCase",
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

_EPS = 1e-9


def _fmt(t: float) -> str:
    return f"{t:.2f} s"


def _blind(programs: list[str]) -> str:
    names = ", ".join(f"`{p}`" for p in programs)
    return f"no step bands for {names} — an external controller's program needs monitor=True on its node"


# ------------------------------------------------------------ expectations


@dataclass
class Check:
    """One expectation's verdict on one run: `ok` is `True`, `False`, or
    `None` when the run could not tell (the thing is not observable)."""

    expectation: str
    ok: Optional[bool]
    detail: str

    @property
    def mark(self) -> str:
        return "✓" if self.ok else ("—" if self.ok is None else "✗")

    def to_dict(self) -> dict:
        return {"expectation": self.expectation, "ok": self.ok, "detail": self.detail}


class _Run:
    """What an expectation reads: the run's timeline and window."""

    def __init__(self, tl, t0: float, t1: float, steps_at_end, dt: float):
        self.tl = tl
        self.t0 = t0
        self.t1 = t1
        self.steps_at_end = list(steps_at_end)
        self.dt = dt
        self.programs = list(tl.sequences)

    def deadline(self, window: Optional[float]) -> float:
        return self.t1 if window is None else min(self.t1, self.t0 + window)

    def spans_of(self, program: str) -> list[tuple[str, float, float]]:
        """The step bands of `program` — `program/step` in a multi-program
        run, bare names when it ran alone."""
        prefix = program + "/"
        spans = [(n[len(prefix):], a, b) for n, a, b in self.tl.step_spans if n.startswith(prefix)]
        if not spans and self.programs == [program]:
            spans = list(self.tl.step_spans)
        return spans

    def finished_at(self, program: str) -> Optional[float]:
        """When `program` ran to its end — the end of its last band — or
        `None` while it still stands in a step (or is not in the run)."""
        for name, step in self.steps_at_end:
            if name == program:
                if step is not None:
                    return None
                spans = self.spans_of(program)
                return max((b for _, _, b in spans), default=0.0) if spans else 0.0
        return None


class Expectation:
    """Base of the expectations: `window` is the plant time after the
    anchor the check needs (the run is kept going at least that long);
    `check(run)` reads the recorded timeline."""

    kind: str = ""
    window: Optional[float] = None

    def describe(self) -> str:  # pragma: no cover - abstract
        raise NotImplementedError

    def check(self, run: _Run) -> Check:  # pragma: no cover - abstract
        raise NotImplementedError

    def to_dict(self) -> dict:  # pragma: no cover - abstract
        raise NotImplementedError

    def __repr__(self) -> str:
        return f"{type(self).__name__}({self.describe()!r})"


class _Edge(Expectation):
    def __init__(self, kind: str, lane: str, within: Optional[float]):
        self.kind = kind
        self.lane = lane
        self.window = within

    def describe(self) -> str:
        suffix = "" if self.window is None else f" within {_fmt(self.window)}"
        return f"{self.lane} {self.kind}{suffix}"

    def to_dict(self) -> dict:
        d: dict = {self.kind: self.lane}
        if self.window is not None:
            d["within"] = self.window
        return d

    def check(self, run: _Run) -> Check:
        track = run.tl.signal(self.lane)
        edges = track.rising_edges() if self.kind == "rises" else track.falling_edges()
        deadline = run.deadline(self.window)
        hit = [t for t in edges if run.t0 - _EPS <= t <= deadline + _EPS]
        if hit:
            return Check(self.describe(), True, f"at {_fmt(hit[0])} (+{_fmt(hit[0] - run.t0)})")
        ended = ""
        if self.window is not None and run.t1 < run.t0 + self.window - _EPS:
            ended = f"; the run ended at {_fmt(run.t1)}"
        word = "rising" if self.kind == "rises" else "falling"
        return Check(self.describe(), False, f"no {word} edge {_fmt(run.t0)}–{_fmt(deadline)}{ended}")


def rises(lane: str, within: Optional[float] = None) -> Expectation:
    """`lane` takes a rising edge within `within` plant seconds of the
    anchor (by the end of the run when `None`)."""
    return _Edge("rises", lane, within)


def falls(lane: str, within: Optional[float] = None) -> Expectation:
    """`lane` takes a falling edge within `within` seconds of the anchor."""
    return _Edge("falls", lane, within)


class _Never(Expectation):
    kind = "never"

    def __init__(self, what, during: Optional[float]):
        if isinstance(what, _vc.step):
            self.program: Optional[str] = what.program
            self.step: Optional[str] = what.name
            self.lane: Optional[str] = None
        elif isinstance(what, str):
            self.program, self.step, self.lane = None, None, what
        else:
            raise TypeError("never() takes a lane name or bt.vc.step(program, step)")
        self.window = during

    def describe(self) -> str:
        what = self.lane if self.lane is not None else f"step {self.program}/{self.step}"
        suffix = "" if self.window is None else f" for {_fmt(self.window)}"
        return f"never {what}{suffix}"

    def to_dict(self) -> dict:
        d: dict = {"never": self.lane if self.lane is not None else {"step": [self.program, self.step]}}
        if self.window is not None:
            d["during"] = self.window
        return d

    def check(self, run: _Run) -> Check:
        deadline = run.deadline(self.window)
        if self.lane is not None:
            track = run.tl.signal(self.lane)
            if track.value_at(run.t0):
                return Check(self.describe(), False, f"already high at {_fmt(run.t0)}")
            for a, b in track.high_spans():
                if b > run.t0 + _EPS and a < deadline - _EPS:
                    return Check(self.describe(), False, f"high at {_fmt(a)} (+{_fmt(a - run.t0)})")
            return Check(self.describe(), True, f"low {_fmt(run.t0)}–{_fmt(deadline)}")
        spans = run.spans_of(self.program)
        if not spans:
            return Check(self.describe(), None, _blind([self.program]))
        for name, a, b in spans:
            if name == self.step and a >= run.t0 - _EPS and a <= deadline + _EPS:
                return Check(self.describe(), False, f"entered at {_fmt(a)} (+{_fmt(a - run.t0)})")
        return Check(self.describe(), True, f"not entered {_fmt(run.t0)}–{_fmt(deadline)}")


def never(what, during: Optional[float] = None) -> Expectation:
    """A lane is never high — or, with `bt.vc.step(program, step)`, the
    step is never entered — during `during` seconds after the anchor (the
    whole run when `None`)."""
    return _Never(what, during)


class _Held(Expectation):
    kind = "held"

    def __init__(self, lane: str, value: bool, for_: Optional[float]):
        self.lane = lane
        self.value = bool(value)
        self.window = for_

    def describe(self) -> str:
        suffix = "" if self.window is None else f" for {_fmt(self.window)}"
        return f"{self.lane} {'high' if self.value else 'low'}{suffix}"

    def to_dict(self) -> dict:
        d: dict = {"held": {"lane": self.lane, "value": self.value}}
        if self.window is not None:
            d["held"]["for"] = self.window
        return d

    def check(self, run: _Run) -> Check:
        track = run.tl.signal(self.lane)
        deadline = run.deadline(self.window)
        if track.value_at(run.t0) != self.value:
            return Check(self.describe(), False, f"{'low' if self.value else 'high'} at {_fmt(run.t0)}")
        for t, v in track.edges:
            if run.t0 + _EPS < t < deadline - _EPS and v != self.value:
                return Check(self.describe(), False, f"went {'low' if self.value else 'high'} at {_fmt(t)}")
        return Check(self.describe(), True, f"held {_fmt(run.t0)}–{_fmt(deadline)}")


def held(lane: str, value: bool = True, for_: Optional[float] = None) -> Expectation:
    """`lane` holds `value` from the anchor for `for_` seconds (to the end
    of the run when `None`) — no edge in between."""
    return _Held(lane, value, for_)


class _StepReached(Expectation):
    kind = "step_reached"

    def __init__(self, program: str, steps: Sequence[str], within: Optional[float]):
        self.program = program
        self.steps = list(steps)
        self.window = within
        if not self.steps:
            raise ValueError("step_reached needs a step name")

    def describe(self) -> str:
        names = self.steps[0] if len(self.steps) == 1 else "(" + " | ".join(self.steps) + ")"
        suffix = "" if self.window is None else f" within {_fmt(self.window)}"
        return f"{self.program}/{names} reached{suffix}"

    def to_dict(self) -> dict:
        d: dict = {"step_reached": [self.program, self.steps[0] if len(self.steps) == 1 else self.steps]}
        if self.window is not None:
            d["within"] = self.window
        return d

    def check(self, run: _Run) -> Check:
        spans = run.spans_of(self.program)
        if not spans:
            return Check(self.describe(), None, _blind([self.program]))
        deadline = run.deadline(self.window)
        for name, a, _ in spans:
            if name in self.steps and run.t0 - _EPS <= a <= deadline + _EPS:
                return Check(self.describe(), True, f"entered {name} at {_fmt(a)} (+{_fmt(a - run.t0)})")
        stood = next((s for p, s in run.steps_at_end if p == self.program), None)
        where = f"stood at {stood}" if stood is not None else "the program ended"
        return Check(self.describe(), False, f"not entered {_fmt(run.t0)}–{_fmt(deadline)}; {where}")


def step_reached(program: str, step: Union[str, Sequence[str]], within: Optional[float] = None) -> Expectation:
    """`program` enters `step` (any of several) within `within` seconds of
    the anchor. An external controller's steps are seen through its
    monitor word (`monitor=True`)."""
    steps = [step] if isinstance(step, str) else list(step)
    return _StepReached(program, steps, within)


class _Stalls(Expectation):
    kind = "stalls"
    window = None

    def __init__(self, program: Optional[str], at: Optional[str]):
        self.program = program
        self.at = at

    def describe(self) -> str:
        who = "the cycle" if self.program is None else self.program
        where = "" if self.at is None else f" at {self.at}"
        return f"{who} stalls{where}"

    def to_dict(self) -> dict:
        d: dict = {}
        if self.program is not None:
            d["program"] = self.program
        if self.at is not None:
            d["at"] = self.at
        return {"stalls": d or True}

    def check(self, run: _Run) -> Check:
        programs = [self.program] if self.program is not None else run.programs
        blind = [p for p in programs if not run.spans_of(p)]
        if blind:
            return Check(self.describe(), None, _blind(blind))
        waiting = [(p, s) for p, s in run.steps_at_end if p in programs and s is not None]
        if not waiting:
            done = [p for p in programs if run.finished_at(p) is not None]
            when = max((run.finished_at(p) or 0.0) for p in done) if done else run.t1
            return Check(self.describe(), False, f"the cycle completed at {_fmt(when)}")
        if self.at is not None and not any(s == self.at for _, s in waiting):
            where = ", ".join(f"{p}/{s}" for p, s in waiting)
            return Check(self.describe(), False, f"waiting at {where} at {_fmt(run.t1)}, not {self.at}")
        where = ", ".join(f"{p}/{s}" for p, s in waiting)
        return Check(self.describe(), True, f"waiting at {where} at {_fmt(run.t1)}")


def stalls(program: Optional[str] = None, at: Optional[str] = None) -> Expectation:
    """The cycle is refused: `program` (any program when `None`) has not
    run to its end when the run ends — standing in step `at` when given.
    What a stuck guard switch or an E-stop in must do to a start."""
    return _Stalls(program, at)


class _NoMotion(Expectation):
    kind = "no_motion"

    def __init__(self, robot: Optional[str], during: Optional[float]):
        self.robot = robot
        self.window = during

    def describe(self) -> str:
        who = "no robot" if self.robot is None else self.robot
        suffix = "" if self.window is None else f" for {_fmt(self.window)}"
        return f"{who} moves{suffix}" if self.robot is None else f"{who} does not move{suffix}"

    def to_dict(self) -> dict:
        d: dict = {"no_motion": self.robot if self.robot is not None else True}
        if self.window is not None:
            d["during"] = self.window
        return d

    def check(self, run: _Run) -> Check:
        deadline = run.deadline(self.window)
        robots = [self.robot] if self.robot is not None else list(run.tl.robots)
        for robot in robots:
            for a, b in run.tl.robot_busy(robot):
                if b > run.t0 + _EPS and a < deadline - _EPS:
                    return Check(self.describe(), False, f"{robot} moved at {_fmt(max(a, run.t0))}")
        return Check(self.describe(), True, f"still {_fmt(run.t0)}–{_fmt(deadline)}")


def no_motion(robot: Optional[str] = None, during: Optional[float] = None) -> Expectation:
    """No robot (or `robot`) starts or continues a motion during `during`
    seconds after the anchor (the whole run when `None`)."""
    return _NoMotion(robot, during)


class _Clearance(Expectation):
    kind = "clearance"
    window = None

    def __init__(self, min_m: float, to: Optional[list[str]]):
        if not (min_m >= 0.0):
            raise ValueError(f"clearance takes a non-negative distance, got {min_m}")
        self.min_m = float(min_m)
        self.to = list(to) if to else None

    def describe(self) -> str:
        suffix = "" if self.to is None else f" to {', '.join(self.to)}"
        return f"clearance ≥ {self.min_m * 1e3:.0f} mm{suffix}"

    def to_dict(self) -> dict:
        d: dict = {"clearance": self.min_m}
        if self.to is not None:
            d["to"] = self.to
        return d

    def check(self, run: _Run) -> Check:
        window = run.tl if run.t0 <= _EPS and run.t1 >= run.tl.duration - _EPS else run.tl.clip(run.t0, run.t1)
        c = window.min_clearance(to=self.to) if self.to is not None else window.min_clearance()
        d = float(c)
        pair = f" ({c.pair[0]} × {c.pair[1]})" if c.pair else ""
        at = c.t + (0.0 if window is run.tl else run.t0)
        return Check(self.describe(), d >= self.min_m - _EPS, f"min {d * 1e3:.1f} mm at {_fmt(at)}{pair}")


def clearance(min_m: float, to: Optional[list[str]] = None) -> Expectation:
    """The minimum clearance over the window stays at or above `min_m`
    metres (`to` narrows it to named obstacles)."""
    return _Clearance(min_m, to)


class _CycleWithin(Expectation):
    kind = "cycle_within"

    def __init__(self, seconds: float, program: Optional[str]):
        if not (seconds > 0.0):
            raise ValueError(f"cycle_within takes a positive time, got {seconds}")
        self.seconds = float(seconds)
        self.program = program
        self.window = self.seconds

    def describe(self) -> str:
        who = "the cycle" if self.program is None else self.program
        return f"{who} completes within {_fmt(self.seconds)}"

    def to_dict(self) -> dict:
        d: dict = {"cycle_within": self.seconds}
        if self.program is not None:
            d["program"] = self.program
        return d

    def check(self, run: _Run) -> Check:
        programs = [self.program] if self.program is not None else run.programs
        blind = [p for p in programs if not run.spans_of(p)]
        if blind:
            return Check(self.describe(), None, _blind(blind))
        ends = []
        for p in programs:
            end = run.finished_at(p)
            if end is None:
                stood = next((s for q, s in run.steps_at_end if q == p), None)
                return Check(self.describe(), False,
                             f"{p} still at {stood} when the run ended at {_fmt(run.t1)}")
            ends.append(end)
        end = max(ends) if ends else run.t1
        took = end - run.t0
        return Check(self.describe(), took <= self.seconds + _EPS, f"done at {_fmt(end)} (+{_fmt(took)})")


def cycle_within(seconds: float, program: Optional[str] = None) -> Expectation:
    """Every program (or `program`) runs to its end within `seconds` of the
    anchor. An external controller's program ends when its monitor word
    parks."""
    return _CycleWithin(seconds, program)


class _Custom(Expectation):
    kind = "check"
    window = None

    def __init__(self, fn: Callable, label: str):
        self.fn = fn
        self.label = label

    def describe(self) -> str:
        return self.label

    def to_dict(self) -> dict:
        return {"check": self.label}

    def check(self, run: _Run) -> Check:
        result = self.fn(run.tl, run.t0, run.t1)
        if isinstance(result, tuple):
            ok, detail = result
            return Check(self.label, bool(ok), str(detail))
        return Check(self.label, bool(result), "")


def check(fn: Callable[[Any, float, float], Any], label: str) -> Expectation:
    """Your own predicate: `fn(timeline, t0, t1)` returns a truth value, or
    `(ok, detail)`. For what the built-in expectations do not say."""
    return _Custom(fn, label)


# --------------------------------------------------------------- the case


@dataclass
class TestCase:
    """One row of the FAT: the moment (`at`, a `bt.vc` predicate; the
    first scan when `None`), what goes in there — `faults` (`bt.io.stuck /
    open / node_down`), `forces` (lanes pinned), `press` (panel buttons) —
    how long the case runs on (`until` a predicate, `for_` seconds, or
    until the programs end / the horizon), and what must hold over the
    window: `given` are preconditions (a case whose setup never took is
    reported *n/a*, not failed), `expect` the verdict. `scenario` starts
    the cell from a named scenario's deltas; `skip` reports the row n/a
    without running it (a derived row nothing can pin)."""

    name: str
    faults: Sequence[dict] = ()
    forces: Mapping[str, bool] = field(default_factory=dict)
    press: Sequence[str] = ()
    at: Any = None
    until: Any = None
    for_: Optional[float] = None
    expect: Sequence[Expectation] = ()
    given: Sequence[Expectation] = ()
    scenario: Optional[str] = None
    notes: str = ""
    source: str = "authored"
    skip: Optional[str] = None

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError("a test case needs a name")
        self.faults = list(self.faults)
        self.forces = dict(self.forces)
        self.press = list(self.press)
        self.expect = list(self.expect)
        self.given = list(self.given)
        for e in (*self.expect, *self.given):
            if not isinstance(e, Expectation):
                raise TypeError(f"{self.name}: expectations are bt.vc.rises(...) and friends, not {e!r}")
        if self.at is not None and not callable(self.at):
            raise TypeError(f"{self.name}: `at` is a bt.vc predicate (step / rising / elapsed / level)")
        if self.until is not None and not callable(self.until):
            raise TypeError(f"{self.name}: `until` is a bt.vc predicate")
        if self.for_ is not None and not (self.for_ > 0.0):
            raise ValueError(f"{self.name}: for_ must be positive")

    @property
    def setup(self) -> str:
        """The faults, forces and presses as one line."""
        parts = []
        for f in self.faults:
            kind = f.get("kind")
            if kind == "stuck":
                parts.append(f"{f['target']} stuck {'high' if f.get('value') else 'low'}")
            else:
                parts.append(f"{f['target']} {kind}")
        parts += [f"{lane} forced {'high' if v else 'low'}" for lane, v in self.forces.items()]
        parts += [f"press {lane}" for lane in self.press]
        if self.scenario:
            parts.insert(0, f"scenario {self.scenario}")
        when = "" if self.at is None else f" at {_describe_predicate(self.at)}"
        return (", ".join(parts) + when) if parts else ("—" if self.at is None else f"nothing{when}")

    @property
    def baseline(self) -> bool:
        """A row that changes nothing and runs the whole cycle: it shares
        one run with every other such row."""
        return not (self.faults or self.forces or self.press or self.at is not None
                    or self.until is not None or self.for_ is not None)

    def to_dict(self) -> dict:
        """The row as plain data (`cases_to_data`)."""
        d: dict = {"name": self.name}
        if self.faults:
            d["faults"] = [_fault_to_data(f) for f in self.faults]
        if self.forces:
            d["forces"] = dict(self.forces)
        if self.press:
            d["press"] = list(self.press)
        if self.at is not None:
            d["at"] = _predicate_to_data(self.at)
        if self.until is not None:
            d["until"] = _predicate_to_data(self.until)
        if self.for_ is not None:
            d["for"] = self.for_
        if self.given:
            d["given"] = [e.to_dict() for e in self.given]
        if self.expect:
            d["expect"] = [e.to_dict() for e in self.expect]
        if self.scenario:
            d["scenario"] = self.scenario
        if self.notes:
            d["notes"] = self.notes
        if self.source != "authored":
            d["source"] = self.source
        if self.skip:
            d["skip"] = self.skip
        return d


def _describe_predicate(p) -> str:
    if isinstance(p, _vc.step):
        return f"step {p.program}/{p.name}"
    if isinstance(p, _vc.rising):
        return f"↑{p.name}"
    if isinstance(p, _vc.falling):
        return f"↓{p.name}"
    if isinstance(p, _vc.level):
        return f"{p.name} {'high' if p.value else 'low'}"
    if isinstance(p, _vc.elapsed):
        return _fmt(p.seconds)
    if isinstance(p, _vc.done):
        return "done" if p.program is None else f"{p.program} done"
    return getattr(p, "__name__", type(p).__name__)


def _predicate_to_data(p) -> dict:
    if isinstance(p, _vc.step):
        return {"step": [p.program, p.name]}
    if isinstance(p, _vc.rising):
        return {"rising": p.name}
    if isinstance(p, _vc.falling):
        return {"falling": p.name}
    if isinstance(p, _vc.level):
        return {"level": [p.name, p.value]}
    if isinstance(p, _vc.elapsed):
        return {"elapsed": p.seconds}
    if isinstance(p, _vc.done):
        return {"done": p.program if p.program is not None else True}
    raise ValueError(f"a predicate of your own ({p!r}) cannot be written as data")


def _fault_to_data(f: dict) -> dict:
    kind = f["kind"]
    if kind == "stuck":
        return {"stuck": f["target"], "value": bool(f.get("value", False))}
    return {kind: f["target"]}


# ------------------------------------------------------------- the report


@dataclass
class CaseResult:
    """One row filled in: the verdict (`pass`, `fail`, `n/a`, `error`),
    the setup as run, each check, and the window it was judged over."""

    name: str
    verdict: str
    setup: str
    checks: list[Check]
    detail: str = ""
    t0: Optional[float] = None
    t1: Optional[float] = None
    source: str = "authored"
    notes: str = ""

    @property
    def ok(self) -> bool:
        return self.verdict in ("pass", "n/a")

    def to_dict(self) -> dict:
        return {
            "name": self.name, "verdict": self.verdict, "setup": self.setup, "detail": self.detail,
            "t0": self.t0, "t1": self.t1, "source": self.source, "notes": self.notes,
            "checks": [c.to_dict() for c in self.checks],
        }


class FatReport:
    """The FAT sheet: one `CaseResult` per case, the mode it ran in, and
    the timelines of the runs (`timelines[name]` — playable, exportable,
    the evidence behind a red row). `ok` is no row failed or errored;
    rows reported n/a (a precondition that did not take, a derived row
    nothing can pin, a step no monitor word shows) do not fail it."""

    def __init__(self, cases: list[CaseResult], *, mode: str, sequences: list[str],
                 horizon: float, dt: float, title: Optional[str] = None,
                 timelines: Optional[dict] = None):
        self.cases = list(cases)
        self.mode = mode
        self.sequences = list(sequences)
        self.horizon = horizon
        self.dt = dt
        self.title = title or "FAT"
        self.timelines = dict(timelines or {})

    def __len__(self) -> int:
        return len(self.cases)

    def __iter__(self):
        return iter(self.cases)

    def __getitem__(self, name: str) -> CaseResult:
        for c in self.cases:
            if c.name == name:
                return c
        raise KeyError(name)

    @property
    def ok(self) -> bool:
        return all(c.ok for c in self.cases)

    @property
    def counts(self) -> dict[str, int]:
        out = {"pass": 0, "fail": 0, "n/a": 0, "error": 0}
        for c in self.cases:
            out[c.verdict] = out.get(c.verdict, 0) + 1
        return out

    @property
    def failed(self) -> list[CaseResult]:
        return [c for c in self.cases if not c.ok]

    def to_dict(self) -> dict:
        return {
            "title": self.title, "mode": self.mode, "sequences": self.sequences,
            "horizon_s": self.horizon, "dt": self.dt, "ok": self.ok, "counts": self.counts,
            "cases": [c.to_dict() for c in self.cases],
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False) + "\n"

    def to_markdown(self) -> str:
        n = self.counts
        lines = [f"# {self.title} — {', '.join(self.sequences)}", ""]
        lines.append(
            f"{self.mode}; horizon {self.horizon:g} s, scan {self.dt * 1e3:g} ms. "
            f"**{n['pass']} passed, {n['fail']} failed**, {n['n/a']} n/a, {n['error']} error — "
            f"{'ok' if self.ok else 'NOT OK'}."
        )
        lines += ["", "| # | case | setup | checks | verdict |", "|---|---|---|---|---|"]
        for i, c in enumerate(self.cases, 1):
            checks = "<br>".join(f"{k.mark} {_md(k.expectation)} — {_md(k.detail)}" for k in c.checks) or "—"
            verdict = {"pass": "**pass**", "fail": "**FAIL**", "n/a": "n/a", "error": "**ERROR**"}[c.verdict]
            if c.detail and c.verdict in ("n/a", "error"):
                verdict += f" — {_md(c.detail)}"
            name = _md(c.name) + (f"<br><sub>{_md(c.notes)}</sub>" if c.notes else "")
            lines.append(f"| {i} | {name} | {_md(c.setup)} | {checks} | {verdict} |")
        return "\n".join(lines) + "\n"

    def to_csv(self) -> str:
        buf = _io.StringIO()
        w = csv.writer(buf)
        w.writerow(["case", "source", "setup", "verdict", "detail", "t0", "t1", "checks"])
        for c in self.cases:
            checks = "; ".join(f"{k.mark} {k.expectation} — {k.detail}" for k in c.checks)
            w.writerow([c.name, c.source, c.setup, c.verdict, c.detail,
                        "" if c.t0 is None else f"{c.t0:.3f}", "" if c.t1 is None else f"{c.t1:.3f}", checks])
        return buf.getvalue()

    def save(self, path: Union[str, Path], format: Optional[str] = None) -> None:
        """Writes the sheet; the format follows the extension (`.md`,
        `.csv`, `.json`) unless `format` says otherwise."""
        path = Path(path)
        fmt = (format or path.suffix.lstrip(".") or "md").lower()
        text = {"md": self.to_markdown, "markdown": self.to_markdown, "csv": self.to_csv,
                "json": self.to_json}.get(fmt)
        if text is None:
            raise ValueError(f"FatReport.save: unknown format {fmt!r} — use .md, .csv or .json")
        path.write_text(text(), encoding="utf-8")

    def section(self) -> dict:
        """The sheet as a cell-report section: `scene.cell_report(...,
        sections=[fat.section()])`."""
        body = self.to_markdown().split("\n", 2)[2]  # drop the title line
        return {"title": self.title, "markdown": body, "json": self.to_dict()}

    def __repr__(self) -> str:
        n = self.counts
        return f"FatReport({len(self.cases)} cases: {n['pass']} pass, {n['fail']} fail, {n['n/a']} n/a; {self.mode})"


def _md(text: str) -> str:
    return str(text).replace("|", "\\|").replace("\n", " ")


# -------------------------------------------------------------- the runner


def _hosts_of(scene, sequences: list[str]) -> list[str]:
    hosts: list[str] = []
    for _, host in scene.program_hosts(sequences):
        if host not in hosts:
            hosts.append(host)
    return hosts


def run_fat(
    scene,
    cases: Iterable[TestCase],
    sequences: Optional[Sequence[str]] = None,
    *,
    links: Union[Mapping[str, Any], Callable[[], Mapping[str, Any]], None] = None,
    horizon: Optional[float] = None,
    dt: float = 0.01,
    pace: Optional[str] = None,
    cycle: bool = False,
    before: Optional[Callable[[TestCase], None]] = None,
    on_open: Optional[Callable[[TestCase, Any], None]] = None,
    title: Optional[str] = None,
    studio: bool = False,
) -> FatReport:
    """Runs the cases and fills the sheet in. Without `links` every case is
    a loopback session over every host of the program set — botrail's own
    executor through its own wiring, bit for bit the bake — so the FAT is
    an offline document of the design. With `links` (`{host: link}`, or a
    callable returning a fresh dict per case, for a controller that must
    be restarted between rows) each case is a session with the controller
    on the wire, and the sheet has the same shape. `horizon` caps each
    case's plant time after its anchor (default: 1.5 × the baseline cycle
    + 1 s); `before(case)` runs before a case's session opens and
    `on_open(case, session)` once it listens — where a PLC is stopped and
    started again. `studio=True` streams each case's run to a studio open
    on the scene, as a session does."""
    cases = list(cases)
    names = list(sequences) if sequences is not None else list(scene.sequence_names)
    if not names:
        raise ValueError("run_fat: the cell has no programs")
    if dt <= 0.0:
        raise ValueError(f"run_fat: dt must be positive, got {dt}")
    online = links is not None
    hosts = None if online else _hosts_of(scene, names)
    link_factory = links if callable(links) else (lambda: dict(links))  # type: ignore[arg-type]
    pace = pace or ("realtime" if online else "sync")
    if horizon is None:
        # The design's own cycle sets the clock every case gets.
        bake = scene.simulate_sequences(names, dt=dt, max_duration=600.0)
        horizon = round(1.5 * bake.duration + 1.0, 3)
    elif not (horizon > 0.0):
        raise ValueError(f"run_fat: horizon must be positive, got {horizon}")
    cap = float(horizon)

    results: list[CaseResult] = []
    timelines: dict = {}
    shared: dict = {}  # baseline runs by scenario
    described: list[str] = []

    def open_session(case: TestCase, max_duration: float):
        if before is not None:
            before(case)
        if online:
            links_now = dict(link_factory())
            described.clear()
            for host, link in links_now.items():
                describe = getattr(link, "describe", None)
                described.append(f"{host}: {describe() if callable(describe) else type(link).__name__}")
            s = _vc.Session(scene, names, links=links_now, pace=pace, cycle=cycle, dt=dt,
                            max_duration=max_duration, scenario=case.scenario, studio=studio)
        else:
            s = _vc.Session(scene, names, hosts=hosts, loopback=True, pace=pace, dt=dt,
                            max_duration=max_duration, scenario=case.scenario, studio=studio)
        if on_open is not None:
            on_open(case, s)
        return s

    def run_case(case: TestCase):
        """One session: returns (timeline, t0, t1, steps_at_end)."""
        at = copy.deepcopy(case.at) if case.at is not None else None
        until = copy.deepcopy(case.until) if case.until is not None else None
        with open_session(case, 3.0 * cap + 10.0) as s:
            t0 = 0.0
            if at is not None:
                reached = False

                def arrive(sess) -> bool:
                    nonlocal reached
                    if at(sess):
                        reached = True
                        return True
                    return sess.t >= cap - _EPS

                s.run(until=arrive)
                if not reached:
                    raise _NotApplicable(f"the moment ({_describe_predicate(case.at)}) did not come "
                                         f"within {cap:g} s")
                t0 = s.t
            for f in case.faults:
                s.inject(f)
            for lane, value in case.forces.items():
                s.force(lane, value)
            for lane in case.press:
                s.press(lane)
            def stop(sess) -> bool:
                # A finished plant takes no more scans; otherwise the case
                # ends on its own terms (`until`, `for_`) or at the horizon.
                dt_run = sess.t - t0
                if sess.finished or dt_run >= cap - _EPS:
                    return True
                if until is not None:
                    return bool(until(sess))
                if case.for_ is not None:
                    return dt_run >= case.for_ - _EPS
                return False

            s.run(until=stop)
            steps = s.steps()
            t1 = s.t
            tl = s.finish()
            return tl, t0, t1, steps

    for case in cases:
        if case.skip:
            results.append(CaseResult(case.name, "n/a", case.setup, [], detail=case.skip,
                                      source=case.source, notes=case.notes))
            continue
        try:
            if case.baseline and case.scenario in shared:
                tl, t0, t1, steps = shared[case.scenario]
            else:
                tl, t0, t1, steps = run_case(case)
                if case.baseline:
                    shared[case.scenario] = (tl, t0, t1, steps)
        except _NotApplicable as e:
            results.append(CaseResult(case.name, "n/a", case.setup, [], detail=str(e),
                                      source=case.source, notes=case.notes))
            continue
        except (ValueError, RuntimeError, TimeoutError, TypeError, KeyError) as e:
            results.append(CaseResult(case.name, "error", case.setup, [], detail=f"{type(e).__name__}: {e}",
                                      source=case.source, notes=case.notes))
            continue
        timelines[case.name] = tl
        run = _Run(tl, t0, t1, steps, dt)
        checks: list[Check] = []
        verdict = "pass"
        detail = ""
        for e in case.given:
            c = _safe_check(e, run)
            checks.append(c)
            if c.ok is False:
                verdict, detail = "n/a", f"precondition not met: {c.expectation} ({c.detail})"
        if verdict == "pass":
            judged = []
            for e in case.expect:
                c = _safe_check(e, run)
                checks.append(c)
                judged.append(c.ok)
            if any(ok is False for ok in judged):
                verdict = "fail"
            elif judged and all(ok is None for ok in judged):
                verdict, detail = "n/a", "nothing observable"
        results.append(CaseResult(case.name, verdict, case.setup, checks, detail=detail, t0=t0, t1=t1,
                                  source=case.source, notes=case.notes))

    if online:
        mode = "online (" + "; ".join(described) + f"), pace {pace}"
    else:
        mode = f"offline (loopback over {', '.join(hosts or [])})"
    return FatReport(results, mode=mode, sequences=names, horizon=cap, dt=dt,
                     title=title, timelines=timelines)


class _NotApplicable(Exception):
    """The case's moment never came: reported n/a, not failed."""


def _safe_check(e: Expectation, run: _Run) -> Check:
    try:
        return e.check(run)
    except (ValueError, KeyError, TypeError, RuntimeError) as exc:
        return Check(e.describe(), False, f"{type(exc).__name__}: {exc}")


# ------------------------------------------------------- derived test cases


def interlock_tests(scene, program: Optional[str] = None, sequences: Optional[Sequence[str]] = None,
                    prefix: str = "IL") -> list[TestCase]:
    """One case per row of the interlock table (`scene.interlocks`): the
    row's condition pinned false from the first scan — the lanes and
    levels the table names as its `denial` — with the program expected to
    reach the gate (`given`: the step before) and **never** enter the step
    that switches the output. A row whose condition no pin denies (a
    timer, motion completion, device or robot feedback, a first step) is
    carried as n/a with the reason. `program` narrows the rows; the cases
    run over `sequences` (every program by default)."""
    from .. import io as _bt_io

    table = scene.interlocks(list(sequences) if sequences is not None else None)
    rows = [r for r in table.rows if program is None or r["program"] == program]
    cases: list[TestCase] = []
    for i, row in enumerate(rows, 1):
        name = f"{prefix}-{i:02d} {row['step']}: {row['output']}"
        notes = f"`{row['output']}` only after `{row['condition']}`"
        if row.get("undeniable"):
            cases.append(TestCase(name, notes=notes, source="interlock",
                                  skip=f"the condition cannot be pinned false: {row['undeniable']}"))
            continue
        faults = [_bt_io.stuck(lane, bool(value)) for lane, value in row["denial"]]
        given = [step_reached(row["program"], row["after"])] if row["after"] else []
        cases.append(TestCase(
            name, faults=faults, given=given,
            expect=[never(_vc.step(row["program"], row["step"]))],
            notes=notes, source="interlock",
        ))
    return cases


def safety_tests(scene, sequences: Optional[Sequence[str]] = None, prefix: str = "SF") -> list[TestCase]:
    """One case per safety input of the I/O map (`safety=True` on its
    declaration or binding) that a program reads: the input pinned to the
    level that denies the program's test of it, from the first scan, and
    the cycle expected to stall — the E-stop in, the guard switch open,
    the curtain broken, none of them let the cycle start. A safety input
    whose level no program condition fixes is carried as n/a."""
    names = list(sequences) if sequences is not None else None
    points = [p for p in scene.io_points(names)
              if p.safety and p.direction == "input" and p.aspect is None and p.readers]
    if not points:
        return []
    pins: dict[str, bool] = {}
    for row in scene.interlocks(names).rows:
        for inp in row["inputs"]:
            if inp.get("pin") is not None and inp["name"] not in pins:
                pins[inp["name"]] = bool(inp["pin"])
    from .. import io as _bt_io

    cases: list[TestCase] = []
    for i, p in enumerate(points, 1):
        readers = ", ".join(sorted({prog for prog, _, _ in p.readers}))
        name = f"{prefix}-{i:02d} {p.name} dropped"
        if p.name not in pins:
            cases.append(TestCase(name, source="safety", notes=f"read by {readers}",
                                  skip="no program condition fixes the level that denies it"))
            continue
        unsafe = pins[p.name]
        cases.append(TestCase(
            name, faults=[_bt_io.stuck(p.name, unsafe)], expect=[stalls()],
            notes=f"safety input read by {readers}; pinned {'high' if unsafe else 'low'} the cycle must not start",
            source="safety",
        ))
    return cases


# ------------------------------------------------------------ cases as data


def cases_to_data(cases: Iterable[TestCase]) -> dict:
    """The cases as the plain mapping `cases_from_data` reads (`{"cases":
    [...]}`) — what a FAT YAML / JSON file holds."""
    return {"cases": [c.to_dict() for c in cases]}


def cases_from_data(data: Mapping, scene=None) -> list[TestCase]:
    """Test cases from plain data — the FAT file's mapping: `cases` is a
    list of rows (`name`, `faults`, `forces`, `press`, `at`, `until`,
    `for`, `given`, `expect`, `scenario`, `notes`), or of
    `{"interlock_tests": program | true}` / `{"safety_tests": true}` to
    splice the derived rows in (needs `scene`)."""
    rows = data.get("cases") if isinstance(data, Mapping) else data
    if not isinstance(rows, list):
        raise ValueError("FAT data needs a `cases` list")
    out: list[TestCase] = []
    for i, row in enumerate(rows):
        if not isinstance(row, Mapping):
            raise ValueError(f"cases[{i}]: a mapping is expected")
        if "interlock_tests" in row or "safety_tests" in row:
            if scene is None:
                raise ValueError(f"cases[{i}]: derived rows need the scene")
            if "interlock_tests" in row:
                prog = row["interlock_tests"]
                out += interlock_tests(scene, None if prog is True else str(prog),
                                       sequences=row.get("sequences"))
            if row.get("safety_tests"):
                out += safety_tests(scene, sequences=row.get("sequences"))
            continue
        out.append(_case_from_data(row, i))
    return out


def _case_from_data(row: Mapping, i: int) -> TestCase:
    from .. import io as _bt_io

    name = row.get("name")
    if not name:
        raise ValueError(f"cases[{i}]: needs a name")
    faults = []
    for f in row.get("faults", ()) or ():
        if not isinstance(f, Mapping):
            raise ValueError(f"{name}: a fault is a mapping like {{stuck: lane, value: false}}")
        if "stuck" in f:
            faults.append(_bt_io.stuck(str(f["stuck"]), bool(f.get("value", False))))
        elif "open" in f:
            faults.append(_bt_io.open(str(f["open"])))
        elif "node_down" in f:
            faults.append(_bt_io.node_down(str(f["node_down"])))
        else:
            raise ValueError(f"{name}: a fault is stuck / open / node_down, not {sorted(f)}")
    return TestCase(
        str(name),
        faults=faults,
        forces={str(k): bool(v) for k, v in (row.get("forces") or {}).items()},
        press=[str(p) for p in (row.get("press") or ())],
        at=_predicate_from_data(row["at"], name) if row.get("at") is not None else None,
        until=_predicate_from_data(row["until"], name) if row.get("until") is not None else None,
        for_=row.get("for", row.get("for_")),
        given=[_expectation_from_data(e, name) for e in (row.get("given") or ())],
        expect=[_expectation_from_data(e, name) for e in (row.get("expect") or ())],
        scenario=row.get("scenario"),
        notes=str(row.get("notes", "") or ""),
        source=str(row.get("source", "authored") or "authored"),
        skip=row.get("skip"),
    )


def _predicate_from_data(d, case: str):
    if not isinstance(d, Mapping) or len(d) != 1:
        raise ValueError(f"{case}: a moment is one of {{step: [program, step]}}, {{rising: lane}}, "
                         f"{{falling: lane}}, {{level: [lane, value]}}, {{elapsed: seconds}}, {{done: program}}")
    (kind, arg), = d.items()
    if kind == "step":
        return _vc.step(str(arg[0]), str(arg[1]))
    if kind == "rising":
        return _vc.rising(str(arg))
    if kind == "falling":
        return _vc.falling(str(arg))
    if kind == "level":
        lane, value = (arg, True) if isinstance(arg, str) else (arg[0], arg[1])
        return _vc.level(str(lane), bool(value))
    if kind == "elapsed":
        return _vc.elapsed(float(arg))
    if kind == "done":
        return _vc.done(None if arg is True else str(arg))
    raise ValueError(f"{case}: unknown moment {kind!r}")


_EXPECTATIONS = ("rises", "falls", "never", "held", "step_reached", "stalls", "no_motion",
                 "clearance", "cycle_within")


def _expectation_from_data(d, case: str) -> Expectation:
    if not isinstance(d, Mapping):
        raise ValueError(f"{case}: an expectation is a mapping like {{rises: lane, within: 0.5}}")
    kinds = [k for k in d if k in _EXPECTATIONS]
    if len(kinds) != 1:
        raise ValueError(f"{case}: an expectation names one of {_EXPECTATIONS}, got {sorted(d)}")
    kind = kinds[0]
    arg = d[kind]
    opts = {k: v for k, v in d.items() if k != kind}
    if isinstance(arg, Mapping):
        opts = {**arg, **opts}
        arg = None
    if kind in ("rises", "falls"):
        lane = arg if arg is not None else opts.get("lane")
        return (rises if kind == "rises" else falls)(str(lane), opts.get("within"))
    if kind == "never":
        if arg is None and "step" in opts:
            arg = {"step": opts["step"]}
        what = _vc.step(str(arg["step"][0]), str(arg["step"][1])) if isinstance(arg, Mapping) else str(arg or opts.get("lane"))
        return never(what, opts.get("during"))
    if kind == "held":
        lane = arg if isinstance(arg, str) else opts.get("lane")
        return held(str(lane), bool(opts.get("value", True)), opts.get("for", opts.get("for_")))
    if kind == "step_reached":
        if arg is not None:
            program, step = arg[0], arg[1]
        else:
            program, step = opts["program"], opts["step"]
        return step_reached(str(program), step if isinstance(step, list) else str(step), opts.get("within"))
    if kind == "stalls":
        return stalls(opts.get("program"), opts.get("at"))
    if kind == "no_motion":
        robot = None if arg in (None, True) else str(arg)
        return no_motion(opts.get("robot", robot), opts.get("during"))
    if kind == "clearance":
        return clearance(float(arg if arg is not None else opts["min_m"]), opts.get("to"))
    return cycle_within(float(arg if arg is not None else opts["seconds"]), opts.get("program"))


def load_cases(path: Union[str, Path], scene=None) -> list[TestCase]:
    """Test cases from a FAT file — `.yaml` / `.yml` (PyYAML) or `.json` —
    in the shape `cases_from_data` reads. Derived rows
    (`interlock_tests`, `safety_tests`) need `scene`."""
    path = Path(path)
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() in (".yaml", ".yml"):
        try:
            import yaml
        except ImportError as e:  # pragma: no cover - environment
            raise ImportError("reading a FAT .yaml needs PyYAML (pip install pyyaml); .json needs nothing") from e
        data = yaml.safe_load(text)
    else:
        data = json.loads(text)
    return cases_from_data(data, scene)
