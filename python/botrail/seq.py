"""PLC-style sequence authoring: action / transition-condition helpers and
the step builder returned by ``scene.sequence(name)``.

A sequence is a list of steps (工程). Each step fires its entry ``actions``
and completes when its ``transition`` condition holds — the SFC /
step-ladder mental model. When ``transition`` is omitted, a step that
starts a motion or ramp waits for it (``done()``); anything else moves on
``immediately()``.

    sq = scene.sequence("pick_place")
    sq.step("approach", actions=[bt.seq.motion("approach")])
    sq.step("close",    actions=[bt.seq.ramp({"finger": 0.008}, 0.4)])
    sq.step("grasp",    actions=[bt.seq.attach("/World/Conveyor/Box_A")])
    sq.step("carry",    actions=[bt.seq.motion("to_pallet")])
    sq.step("release",  actions=[bt.seq.detach("/World/Conveyor/Box_A")])
    timeline = sq.simulate()
"""

import json
from typing import Any, Dict, Iterable, Mapping, Optional

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


def ramp(
    targets: Mapping[str, float],
    duration: float,
    robot: Optional[str] = None,
) -> Action:
    """Ramp joints to targets over ``duration`` s (gripper open/close);
    await it with ``done()``. ``robot`` names the instance (required when
    the scene has several robots)."""
    action: Action = {
        "type": "start_ramp",
        "targets": [{"joint": j, "value": float(v)} for j, v in targets.items()],
        "duration": float(duration),
    }
    if robot is not None:
        action["robot"] = robot
    return action


def attach(
    obj: str,
    link: Optional[str] = None,
    touch_links: Optional[Iterable[str]] = None,
    robot: Optional[str] = None,
) -> Action:
    """Grasp: rigidly attach an obstacle at its current relative pose.
    ``robot`` names the carrying instance (required with several robots)."""
    action: Action = {"type": "attach", "object": obj}
    if link is not None:
        action["link"] = link
    if touch_links is not None:
        action["touch_links"] = list(touch_links)
    if robot is not None:
        action["robot"] = robot
    return action


def detach(obj: str) -> Action:
    """Release: the obstacle's pose freezes where the robot holds it."""
    return {"type": "detach", "object": obj}


def track(obj: str, link: Optional[str] = None, robot: Optional[str] = None) -> Action:
    """Conveyor tracking: latch onto a moving part. Until :func:`untrack`,
    every commanded pose is carried by the part's motion since this step, so
    poses taught at the station keep meeting the part while it travels — the
    line never has to stop. Grasping the tracked part freezes the offset, so
    the lift after it goes straight up. Planned motions cannot run while
    tracking; ramps can."""
    action: Action = {"type": "track", "object": obj}
    if link is not None:
        action["link"] = link
    if robot is not None:
        action["robot"] = robot
    return action


def untrack(robot: Optional[str] = None) -> Action:
    """Stop following the tracked part; the robot holds where it stands."""
    action: Action = {"type": "untrack"}
    if robot is not None:
        action["robot"] = robot
    return action


def set_signal(name: str, value: bool = True) -> Action:
    """Write an internal signal (declare it with ``scene.define_signal``)."""
    return {"type": "set", "signal": name, "value": bool(value)}


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


def move_to(device: str, position: float) -> Action:
    """Command a linear axis to ``position``; await with ``device_done``."""
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


def robot_done(robot: str) -> Condition:
    """The named robot has no motion/ramp in flight — whichever step
    started it. The idle test interlocks are built from."""
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


_DRIVERS = ("start_motion", "start_ramp")


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


class SequenceBuilder(_Steps):
    """Accumulates steps for one sequence, mirroring every edit into the
    scene (and any connected studio). Creating a builder for an existing
    sequence name starts it over from zero steps."""

    def __init__(self, scene, name: str):
        self._scene = scene
        self._name = name
        self._root = self
        self._steps = []
        self._sync()

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
        self._scene._upsert_sequence_json(json.dumps(payload))
