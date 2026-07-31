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


def ramp(targets: Mapping[str, float], duration: float) -> Action:
    """Ramp joints to targets over ``duration`` s (gripper open/close);
    await it with ``done()``."""
    return {
        "type": "start_ramp",
        "targets": [{"joint": j, "value": float(v)} for j, v in targets.items()],
        "duration": float(duration),
    }


def attach(
    obj: str,
    link: Optional[str] = None,
    touch_links: Optional[Iterable[str]] = None,
) -> Action:
    """Grasp: rigidly attach an obstacle at its current relative pose."""
    action: Action = {"type": "attach", "object": obj}
    if link is not None:
        action["link"] = link
    if touch_links is not None:
        action["touch_links"] = list(touch_links)
    return action


def detach(obj: str) -> Action:
    """Release: the obstacle's pose freezes where the robot holds it."""
    return {"type": "detach", "object": obj}


def set_signal(name: str, value: bool = True) -> Action:
    """Write an internal signal (declare it with ``scene.define_signal``)."""
    return {"type": "set", "signal": name, "value": bool(value)}


# --------------------------------------------------------------- conditions


def immediately() -> Condition:
    """Always true: fire the actions and move on."""
    return {"type": "immediately"}


def done() -> Condition:
    """The motion/ramp started by this step has finished."""
    return {"type": "done"}


def elapsed(seconds: float) -> Condition:
    """On-delay timer (TON) from step entry."""
    return {"type": "elapsed", "seconds": float(seconds)}


def signal(name: str, value: bool = True) -> Condition:
    """Level test of a signal."""
    return {"type": "signal", "name": name, "value": bool(value)}


def all_of(*conditions: Condition) -> Condition:
    """Series contacts (AND)."""
    return {"type": "all", "conditions": list(conditions)}


def any_of(*conditions: Condition) -> Condition:
    """Parallel contacts (OR)."""
    return {"type": "any", "conditions": list(conditions)}


_DRIVERS = ("start_motion", "start_ramp")


class SequenceBuilder:
    """Accumulates steps for one sequence, mirroring every edit into the
    scene (and any connected studio). Creating a builder for an existing
    sequence name starts it over from zero steps."""

    def __init__(self, scene, name: str):
        self._scene = scene
        self._name = name
        self._steps: list = []
        self._sync()

    @property
    def name(self) -> str:
        return self._name

    def step(self, name: str, actions: Iterable[Action] = (), transition: Optional[Condition] = None):
        """Appends one step. Without ``transition``, steps that start a
        motion/ramp await it (``done()``); others pass ``immediately()``."""
        actions = list(actions)
        if transition is None:
            drives = any(a.get("type") in _DRIVERS for a in actions)
            transition = done() if drives else immediately()
        self._steps.append({"name": name, "actions": actions, "transition": transition})
        self._sync()
        return self

    def simulate(self, dt: float = 0.01, max_duration: float = 120.0):
        """Rolls the sequence out (see ``Scene.simulate_sequence``)."""
        return self._scene.simulate_sequence(self._name, dt=dt, max_duration=max_duration)

    def _sync(self) -> None:
        payload = {"name": self._name, "steps": self._steps}
        self._scene._upsert_sequence_json(json.dumps(payload))
