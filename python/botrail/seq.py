"""PLC-style sequence authoring: action / transition-condition helpers and
the step builder returned by ``scene.sequence(name)``.

A sequence is a list of steps (工程). Each step fires its entry ``actions``
and completes when its ``transition`` condition holds — the SFC /
step-ladder mental model. When ``transition`` is omitted, a step that
starts a motion, ramp, policy or crouch waits for it (``done()``);
anything else moves on ``immediately()``.

    sq = scene.sequence("pick_place")
    sq.step("approach", actions=[bt.seq.motion("approach")])
    sq.step("close",    actions=[bt.seq.ramp({"finger": 0.008}, 0.4)])
    sq.step("grasp",    actions=[bt.seq.attach("/World/Conveyor/Box_A")])
    sq.step("carry",    actions=[bt.seq.motion("to_pallet")])
    sq.step("release",  actions=[bt.seq.detach("/World/Conveyor/Box_A")])
    timeline = sq.simulate()
"""

import json
from collections.abc import Iterable, Mapping
from typing import Any, Dict, Optional, Union

Action = Dict[str, Any]
Condition = Dict[str, Any]

# ------------------------------------------------------------------ actions


def motion(name: str) -> Action:
    """Start a named motion; await it with ``done()``."""
    return {"type": "start_motion", "motion": name}


def toolpath(name: str, robot: Optional[str] = None) -> Action:
    """Start a toolpath (continuous Cartesian process path — see
    ``bt.toolpath``): an automatic approach to the path start, then the
    feed-floored follow. Await it with ``done()``. ``robot`` names the
    instance (required when the scene has several robots)."""
    action: Action = {"type": "start_toolpath", "toolpath": name}
    if robot is not None:
        action["robot"] = robot
    return action


def run_program(machine: str, program: str, block: Optional[int] = None) -> Action:
    """Run an NC part program on a machine tool's axes; await it with
    ``done()``. ``machine`` names the robot declared as an NC machine (a
    ``bt.parts.machine_tool`` built with ``axes=`` returns it as
    ``.axes``), ``program`` a part program added with
    ``scene.add_nc_program``. The program is read when the step fires —
    against the machine's work offsets and tool table, from where its
    axes stand — and what its control cannot read fails the bake with
    the line number.

    ``block`` runs one block of it: what the control runs on its own
    between two machine functions — the M codes of the machine's table
    (a door, a vise, a relay), where it stops and waits for the machine.
    A program that calls any runs block by block, with a step of the
    machine's own between two blocks; ``bt.nc.sequence`` writes that
    chain from the program. Without ``block`` the program runs whole, and
    one that calls a machine function is refused."""
    action: Action = {"type": "run_program", "robot": machine, "program": program}
    if block is not None:
        action["block"] = int(block)
    return action


def ramp(
    targets: Mapping[str, float],
    duration: float,
    robot: Optional[str] = None,
    check: bool = False,
) -> Action:
    """Ramp joints to targets over ``duration`` s (gripper open/close);
    await it with ``done()``. ``robot`` names the instance (required when
    the scene has several robots). A ramp is not planned: with ``check``,
    its straight joint line is checked against the cell as it stands when
    the ramp starts — as a planned motion's is — and a contact fails the
    bake by name. A walk that has just stopped still sways for a moment:
    each point is then held where the body will be when the arm gets
    there. Leave it off for a gripper closing on its part."""
    action: Action = {
        "type": "start_ramp",
        "targets": [{"joint": j, "value": float(v)} for j, v in targets.items()],
        "duration": float(duration),
    }
    if robot is not None:
        action["robot"] = robot
    if check:
        action["check"] = True
    return action


def policy(
    name: str,
    robot: Optional[str] = None,
    group: Optional[str] = None,
    hz: float = 20.0,
    max_duration: float = 10.0,
) -> Action:
    """Hand the robot (or the arm ``group``) to the registered policy
    ``name`` for this step: a learned or scripted controller passed to
    ``simulate_sequence(policies={name: ...})`` (see ``botrail.rl.Policy``),
    asked for a joint target ``hz`` times a second until it declares
    itself done or ``max_duration`` seconds pass. Its targets move the
    joints under the model's velocity limits; a collision with the scenery
    while it drives fails the bake. Await it with ``done()``."""
    action: Action = {
        "type": "policy",
        "policy": name,
        "hz": float(hz),
        "max_duration": float(max_duration),
    }
    if robot is not None:
        action["robot"] = robot
    if group is not None:
        action["group"] = group
    return action


def crouch(
    robot: Optional[str],
    depth: float,
    *,
    lean: float = 0.0,
    duration: Optional[float] = None,
) -> Action:
    """Lower a parked walker's body while it stands: over ``duration`` s
    its body (the root link) goes from the crouch it is in to ``depth``
    metres below its standing height, pitched nose-down by ``lean`` rad
    about its own lateral axis — the feet stay where they stand, each leg
    re-solved every scan tick by the walk's own leg IK. ``crouch(robot,
    0)`` stands it back up. Await it with ``done()``.

    ``robot`` names the mounted walker (``None``: the scene's sole robot).
    ``duration=None`` takes a pace: 0.3 m/s of depth and 0.6 rad/s of
    lean at the mean (the smoothstep peaks at 1.5 × that), at least 0.6 s.

    The bake prices the whole crouch when the step fires — a leg that
    cannot keep its foot planted fails it by name — and checks the robot
    and what it holds against the cell every tick on the way down. Fired
    while the walk before it is still settling, it begins when the last
    foot lands. While the body is lowered the vehicle does not drive (a
    goto is refused until it stands up), and a planned motion waits for
    the crouch to finish; ramps may run alongside it (with ``check``,
    each point is held where the body will be)."""
    action: Action = {"type": "crouch", "depth": float(depth)}
    if robot is not None:
        action["robot"] = robot
    if lean:
        action["lean"] = float(lean)
    if duration is not None:
        action["duration"] = float(duration)
    return action


def attach(
    obj: str,
    link: Optional[str] = None,
    touch_links: Union[str, Iterable[str], None] = None,
    robot: Optional[str] = None,
    group: Optional[str] = None,
) -> Action:
    """Grasp: rigidly attach an obstacle at its current relative pose.
    ``robot`` names the carrying instance (required with several robots);
    ``group`` names the arm of a dual-arm robot (``link`` then defaults to
    that arm's tip). ``touch_links="tool"`` exempts the whole tool subtree
    — palm and fingers — which a closed gripper needs (the default exempts
    only the anchor link's own chain)."""
    action: Action = {"type": "attach", "object": obj}
    if link is not None:
        action["link"] = link
    if touch_links is not None:
        links = [touch_links] if isinstance(touch_links, str) else list(touch_links)
        action["touch_links"] = links
    if robot is not None:
        action["robot"] = robot
    if group is not None:
        action["group"] = group
    return action


def detach(obj: str) -> Action:
    """Release: the obstacle's pose freezes where the robot holds it."""
    return {"type": "detach", "object": obj}


def track(
    obj: str,
    link: Optional[str] = None,
    robot: Optional[str] = None,
    group: Optional[str] = None,
) -> Action:
    """Conveyor tracking: latch onto a moving part. Until :func:`untrack`,
    every commanded pose is carried by the part's motion since this step, so
    poses taught at the station keep meeting the part while it travels — the
    line never has to stop. Grasping the tracked part freezes the offset, so
    the lift after it goes straight up. Planned motions cannot run while
    tracking; ramps can. ``group`` names the arm that follows on a dual-arm
    robot: only its joints are spent, so the other arm may drive on — or
    carry the very part this one follows (a two-handed hold is one hand
    grasping and the other tracking)."""
    action: Action = {"type": "track", "object": obj}
    if link is not None:
        action["link"] = link
    if robot is not None:
        action["robot"] = robot
    if group is not None:
        action["group"] = group
    return action


def untrack(robot: Optional[str] = None, group: Optional[str] = None) -> Action:
    """Stop following the tracked part; the robot holds where it stands.
    ``group`` names the arm whose track to release when several follow."""
    action: Action = {"type": "untrack"}
    if robot is not None:
        action["robot"] = robot
    if group is not None:
        action["group"] = group
    return action


def set_signal(name: str, value: bool = True) -> Action:
    """Write an internal signal (declare it with ``scene.define_signal``)."""
    return {"type": "set", "signal": name, "value": bool(value)}


def set_word(name: str, value: "WordValue") -> Action:
    """Write a word (declare it with ``scene.define_word``) on the step's
    entry: a number, another word, or one arithmetic instruction over
    two — ``set_word("target", 60)``, ``set_word("n", word("n") + 1)``."""
    return {"type": "set_word", "word": name, "value": _expr(value)}


def start(device: str) -> Action:
    """Start a conveyor."""
    return {"type": "device", "device": device, "command": {"type": "start"}}


def stop(device: str) -> Action:
    """Stop a conveyor."""
    return {"type": "device", "device": device, "command": {"type": "stop"}}


def set_speed(device: str, speed: float) -> Action:
    """Rescale a conveyor's velocity to ``speed`` (m/s, direction kept)."""
    return {
        "type": "device",
        "device": device,
        "command": {"type": "set_speed", "speed": float(speed)},
    }


def move_to(device: str, position: float | str) -> Action:
    """Command a linear axis to ``position`` (metres) — or a lift to a
    named stop (``move_to("lift", "2F")``); await with ``device_done``.
    A lift's cargo (vehicles and loose parts in its capture zone) is
    fixed the moment this fires: the doors are closed."""
    if isinstance(position, str):
        return {
            "type": "device",
            "device": device,
            "command": {"type": "move_to_stop", "stop": position},
        }
    return {
        "type": "device",
        "device": device,
        "command": {"type": "move_to", "position": float(position)},
    }


def goto(device: str, station: str) -> Action:
    """Dispatch a vehicle to a named station (the AGV call); await arrival
    with ``device_done``. Travel is uninterruptible: a second goto while
    the vehicle is still moving is a sequencing error."""
    return {
        "type": "device",
        "device": device,
        "command": {"type": "goto", "station": station},
    }


def advance(device: str, distance: float) -> Action:
    """Indexed transfer: run a *stopped* conveyor for exactly ``distance``
    metres along its velocity direction, then stop; await it with
    ``device_done``. The final scan tick moves exactly the remainder, so
    the pitch is exact no matter how the scan period divides it — no more
    ``elapsed(pitch/v)`` plus one tick of slack."""
    return {
        "type": "device",
        "device": device,
        "command": {"type": "advance", "distance": float(distance)},
    }


# --------------------------------------------------------------- conditions


def immediately() -> Condition:
    """Always true: fire the actions and move on."""
    return {"type": "immediately"}


def done() -> Condition:
    """Every motion/ramp started by this step has finished."""
    return {"type": "done"}


def robot_done(robot: str, group: Optional[str] = None) -> Condition:
    """The named robot has no motion/ramp in flight — whichever step
    started it. The idle test interlocks are built from. ``group`` narrows
    it to one arm of a dual-arm robot: nothing drives any of its joints."""
    if group is not None:
        return {"type": "group_done", "robot": robot, "group": group}
    return {"type": "robot_done", "robot": robot}


def elapsed(seconds: float) -> Condition:
    """On-delay timer (TON) from step entry."""
    return {"type": "elapsed", "seconds": float(seconds)}


def signal(name: str, value: bool = True) -> Condition:
    """Level test of a signal (internal relay or sensor input)."""
    return {"type": "signal", "name": name, "value": bool(value)}


def rising(name: str) -> Condition:
    """Rising edge (``-|P|-``): the signal turned on since this program's
    previous scan — "the *next* part", not one already sitting on the
    beam. Startup state is not an edge."""
    return {"type": "rising", "name": name}


def falling(name: str) -> Condition:
    """Falling edge (``-|N|-``): the signal turned off since this
    program's previous scan."""
    return {"type": "falling", "name": name}


def device_done(device: str) -> Condition:
    """A linear axis has reached its commanded position."""
    return {"type": "device_done", "device": device}


def otherwise() -> Condition:
    """Always-true branch guard — the ``else`` arm of a ``select``. Put it
    last: arms are tried in order, so it catches whatever the guards
    before it did not (and the exported script skips the wait entirely,
    since some arm is always ready)."""
    return {"type": "immediately"}


def all_of(*conditions: Condition) -> Condition:
    """Series contacts (AND)."""
    return {"type": "all", "conditions": list(conditions)}


def any_of(*conditions: Condition) -> Condition:
    """Parallel contacts (OR)."""
    return {"type": "any", "conditions": list(conditions)}


def not_(condition: Condition) -> Condition:
    """The inverse of a condition — a normally-closed contact over a whole
    network (``not_(signal("stop"))``, ``not_(any_of(a, b))``)."""
    return {"type": "not", "condition": condition}


def step_active(sequence: str, step: str) -> Condition:
    """The program ``sequence`` stands in ``step`` — the step relay of a
    step ladder, read as a contact. Output circuits are written from it:
    ``sq.rung(step_active("auto", "feed")).coil("conv")``."""
    return {"type": "step_active", "sequence": sequence, "step": step}


# -------------------------------------------------------------------- words


def _operand(value) -> Union[str, float]:
    if isinstance(value, Word):
        return value.name
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(
            f"a word operand is bt.seq.word(name) or a number, got {value!r}"
        )
    return float(value)


def _expr(value) -> dict:
    if isinstance(value, WordExpr):
        return {"left": value.left, "op": value.op, "right": value.right}
    return {"left": _operand(value)}


class WordExpr:
    """One arithmetic instruction over two operands — what ``word("a") +
    1`` builds. Hand it to ``set_word`` or a rung's ``.move``; a longer
    formula is one rung per instruction, as on a PLC."""

    def __init__(self, left, op: str, right):
        self.left = _operand(left)
        self.op = op
        self.right = _operand(right)

    def _nested(self, *_):
        raise TypeError(
            "a word expression is one instruction (a + b): move it into a word, "
            "then use that word in the next rung"
        )

    __add__ = __radd__ = __sub__ = __rsub__ = _nested
    __mul__ = __rmul__ = __truediv__ = __rtruediv__ = _nested
    __ge__ = __gt__ = __le__ = __lt__ = _nested

    def __repr__(self) -> str:
        return f"WordExpr({self.left!r} {self.op} {self.right!r})"


class Word:
    """A word by name, for comparisons and arithmetic (see ``word``)."""

    __hash__ = None  # == builds a condition

    def __init__(self, name: str):
        self.name = name

    def __ge__(self, other) -> Condition:
        return compare(self, ">=", other)

    def __gt__(self, other) -> Condition:
        return compare(self, ">", other)

    def __le__(self, other) -> Condition:
        return compare(self, "<=", other)

    def __lt__(self, other) -> Condition:
        return compare(self, "<", other)

    def __eq__(self, other) -> Condition:  # type: ignore[override]
        return compare(self, "==", other)

    def __ne__(self, other) -> Condition:  # type: ignore[override]
        return compare(self, "!=", other)

    def __add__(self, other) -> WordExpr:
        return WordExpr(self, "+", other)

    def __radd__(self, other) -> WordExpr:
        return WordExpr(other, "+", self)

    def __sub__(self, other) -> WordExpr:
        return WordExpr(self, "-", other)

    def __rsub__(self, other) -> WordExpr:
        return WordExpr(other, "-", self)

    def __mul__(self, other) -> WordExpr:
        return WordExpr(self, "*", other)

    def __rmul__(self, other) -> WordExpr:
        return WordExpr(other, "*", self)

    def __truediv__(self, other) -> WordExpr:
        return WordExpr(self, "/", other)

    def __rtruediv__(self, other) -> WordExpr:
        return WordExpr(other, "/", self)

    def __repr__(self) -> str:
        return f"word({self.name!r})"


WordValue = Union[Word, WordExpr, int, float]


def word(name: str) -> Word:
    """A word — a number a program compares and computes with: one the
    scene defines (``scene.define_word``), an analog input (``"dp"``, or
    its counts ``"dp.raw"``), a device's ``"<device>.encoder"``, a
    reader's ``"<sensor>.id"``, a counter's ``"<signal>.count"``.

    Comparing it gives a contact, arithmetic a value to move::

        word("dp") >= word("dp_limit")       # a condition
        word("dp.raw") < 0                   # the wire is broken
        rung.move("margin", word("dp_limit") - word("dp"))

    Two whole operands (integer words, whole constants) make integer
    arithmetic — ``/`` truncates, as a PLC's ``INT / INT`` does."""
    return Word(name)


def compare(left: Union[Word, int, float], op: str, right: Union[Word, int, float]) -> Condition:
    """A comparison contact — what ``word("a") >= 5`` builds. ``op`` is
    one of ``>= > <= < == !=``."""
    if op not in (">=", ">", "<=", "<", "==", "!="):
        raise ValueError(f"compare: unknown operator {op!r} (>= > <= < == !=)")
    return {"type": "compare", "left": _operand(left), "op": op, "right": _operand(right)}


def calc(left: Union[Word, int, float], op: str, right: Union[Word, int, float]) -> WordExpr:
    """One arithmetic instruction — what ``word("a") + 1`` builds. ``op``
    is one of ``+ - * /``."""
    if op not in ("+", "-", "*", "/"):
        raise ValueError(f"calc: unknown operator {op!r} (+ - * /)")
    return WordExpr(left, op, right)


_DRIVERS = ("start_motion", "start_ramp", "run_program", "policy", "crouch")


def _step_dict(name: str, actions: Iterable[Action], transition: Optional[Condition]) -> dict:
    """One step's wire dict, with the default-transition sugar: a step
    that starts a motion/ramp awaits it (``done()``); anything else moves
    on ``immediately()``."""
    actions = list(actions)
    if transition is None:
        drives = any(a.get("type") in _DRIVERS for a in actions)
        transition = done() if drives else immediately()
    return {"name": name, "actions": actions, "transition": transition}


class _Steps:
    """Shared step-list editing: the top-level sequence and every branch
    arm append steps (and further branches) the same way; each edit syncs
    the whole sequence through the root builder."""

    _root: "SequenceBuilder"
    _steps: list

    def step(self, name: str, actions: Iterable[Action] = (), transition: Optional[Condition] = None):
        """Appends one step. Without ``transition``, steps that start a
        motion/ramp await it (``done()``); others pass ``immediately()``."""
        self._steps.append(_step_dict(name, actions, transition))
        self._root._sync()
        return self

    def select(self, name: str) -> "SelectBuilder":
        """Appends a branching step (SFC selection divergence) and returns
        its builder: add arms with ``.when(condition)``, steps inside each
        arm, and every arm rejoins at whatever this list appends next.

            sel = sq.select("judge")
            sel.when(bt.seq.signal("part_ok")).step("place", actions=[...])
            sel.when(bt.seq.signal("part_ng")).step("reject", actions=[...])
            sq.step("home", actions=[bt.seq.motion("home")])   # the rejoin
        """
        step = {
            "name": name,
            "actions": [],
            "transition": immediately(),
            "select": [],
        }
        self._steps.append(step)
        self._root._sync()
        return SelectBuilder(self._root, step["select"])


class SelectBuilder:
    """Arms of one branching step (see ``_Steps.select``)."""

    def __init__(self, root: "SequenceBuilder", arms: list):
        self._root = root
        self._arms = arms

    def when(self, condition: Condition) -> "ArmBuilder":
        """Appends an arm guarded by ``condition``. Arms are tried in the
        order added (SFC's left-to-right priority); the first whose
        condition holds runs. An arm left empty skips straight to the
        rejoin."""
        arm = {"condition": condition, "steps": []}
        self._arms.append(arm)
        self._root._sync()
        return ArmBuilder(self._root, arm["steps"])


class ArmBuilder(_Steps):
    """One arm's step list — the same ``step``/``select`` API as the
    sequence itself, so arms nest."""

    def __init__(self, root: "SequenceBuilder", steps: list):
        self._root = root
        self._steps = steps


class RungBuilder:
    """One rung's outputs (see ``SequenceBuilder.rung``); each call adds
    an output and returns the rung, so they chain."""

    def __init__(self, root: "SequenceBuilder", outputs: list):
        self._root = root
        self._outputs = outputs

    def _add(self, output: dict) -> "RungBuilder":
        self._outputs.append(output)
        self._root._sync()
        return self

    def coil(self, target: str) -> "RungBuilder":
        """``-( )-``: the target follows the rung every scan — an internal
        signal, a conveyor's or a source's run coil (the device's name),
        or a cylinder's solenoid coil (``"press/extend"``)."""
        return self._add({"type": "coil", "target": target})

    def set(self, signal: str) -> "RungBuilder":
        """``-(S)-``: latches the signal on while the rung conducts."""
        return self._add({"type": "set", "signal": signal})

    def reset(self, signal: str) -> "RungBuilder":
        """``-(R)-``: latches the signal off while the rung conducts; on a
        counter's signal it also clears the count."""
        return self._add({"type": "reset", "signal": signal})

    def ton(self, signal: str, seconds: float) -> "RungBuilder":
        """On-delay timer: ``signal`` comes on once the rung has conducted
        for ``seconds`` and drops with it. The signal is declared by this
        (a PLC's ``T`` device); read it with ``bt.seq.signal(signal)``."""
        return self._add({"type": "ton", "signal": signal, "seconds": float(seconds)})

    def ctu(self, signal: str, preset: int) -> "RungBuilder":
        """Up-counter: counts the rung's rising edges; ``signal`` is on
        from ``preset`` counts until a ``reset(signal)``. The count is the
        word lane ``<signal>.count`` (``tl.word("made.count")``)."""
        return self._add({"type": "ctu", "signal": signal, "preset": int(preset)})

    def move(self, word: str, value: WordValue) -> "RungBuilder":
        """A transfer or an arithmetic instruction: while the rung
        conducts, ``word`` is written with ``value`` every scan — a number
        or a word (``MOV``), or one instruction over two
        (``word("a") + 1``). Gate the rung with ``rising(...)`` to run it
        once per edge (a count, an accumulation step)."""
        return self._add({"type": "move", "word": word, "value": _expr(value)})


class SequenceBuilder(_Steps):
    """Accumulates one program, mirroring every edit into the scene (and
    any connected studio): its step chain (``step`` / ``select``), its
    rungs (``rung``) and its gates (``gates``). Creating a builder for an
    existing sequence name starts it over from nothing."""

    def __init__(self, scene, name: str):
        self._scene = scene
        self._name = name
        self._root = self
        self._steps = []
        self._logic: dict = {"rungs": []}
        self._sync()

    def rung(self, condition: Condition, comment: str = "") -> RungBuilder:
        """Appends a rung: a contact network (any condition but ``done`` /
        ``elapsed``, which belong to a step) and the outputs chained onto
        the returned builder::

            lad = scene.sequence("common")
            lad.rung(all_of(any_of(signal("pb_start"), signal("run")), not_(signal("pb_stop")))).coil("run")
            lad.rung(signal("run"), comment="belt").coil("conv")
            lad.rung(signal("jam")).ton("t_jam", 1.5)
            lad.rung(signal("t_jam")).set("alarm")

        Rungs are scanned every scan, top to bottom, after the program's
        step transitions; a program may be rungs alone. Programs scan in
        the order they are simulated, so a common circuit goes first and
        an output circuit last. Every signal a rung drives is this
        program's alone (a double coil is refused)."""
        rung = {"comment": comment, "condition": condition, "outputs": []}
        self._logic["rungs"].append(rung)
        self._sync()
        return RungBuilder(self, rung["outputs"])

    def gates(
        self,
        *,
        enable: Optional[Condition] = None,
        reset: Optional[Condition] = None,
        cycle: bool = False,
    ) -> "SequenceBuilder":
        """The gates of the step chain: it advances only while ``enable``
        holds (a step-mode switch, a pause), ``reset`` puts it back in —
        and holds it in — its first step, and with ``cycle`` it starts
        over when it ends (a bake then needs another program that ends,
        an operator's script, to say when to stop)."""
        for key, value in (("enable", enable), ("reset", reset)):
            if value is None:
                self._logic.pop(key, None)
            else:
                self._logic[key] = value
        self._logic["cycle"] = bool(cycle)
        self._sync()
        return self

    def plant(self, on: bool = True) -> "SequenceBuilder":
        """Marks the program as the equipment's own — what a pump, a tank
        or a machine does, not what a controller tells it: a pressure
        that climbs while liquid flows, written as a rung that adds to a
        word. It runs with the plant in every mode (a bake, a live
        session with the PLC outside) and stays out of the I/O map: the
        signals it reads are outputs of whoever writes them (the pump's
        contactor), the ones it writes inputs of whoever reads them (a
        flow switch). No solver — integrators and comparisons in words."""
        self._logic["plant"] = bool(on)
        self._sync()
        return self

    @property
    def name(self) -> str:
        return self._name

    def simulate(self, dt: float = 0.01, max_duration: float = 120.0, scenario: Optional[str] = None):
        """Rolls the sequence out (see ``Scene.simulate_sequence``).
        ``scenario`` runs it under a named initial-state delta
        (``scene.add_scenario``)."""
        return self._scene.simulate_sequence(
            self._name, dt=dt, max_duration=max_duration, scenario=scenario
        )

    def _sync(self) -> None:
        payload = {"name": self._name, "steps": self._steps}
        logic = self._logic
        if (logic["rungs"] or logic.get("enable") or logic.get("reset") or logic.get("cycle")
                or logic.get("plant")):
            payload["logic"] = logic
        self._scene._upsert_sequence_json(json.dumps(payload))
