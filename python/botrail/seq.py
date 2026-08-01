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


def device_done(device: str) -> Condition:
    """A linear axis has reached its commanded position."""
    return {"type": "device_done", "device": device}


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
