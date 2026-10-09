"""Replay a cable against a baked cycle, in Z-up metres and seconds.

The independent Rapier 0.36 f64 world never applies reactions to the
source bake. Grippers attach explicit material points without teleporting
them; source signal times split the replay clock, preserving short pulses.
Anchors hold a span for the whole replay — a cable's end inside the
connector housing a robot carries and plugs.
"""
import json
import math


def gripper(signal, *, robot, link, location="Start"):
    """Bind a signal to a named robot/link and Start/End/material location.

    ``location`` may also be ``{"ArcLength": {"arc_length_m": ..., "tolerance_m": ...}}``.
    """
    return dict(signal=signal, robot=robot, link=link, location=location)


def anchor(obstacle=None, *, robot=None, link=None, location="Start", length_m=0.0):
    """Hold a span of the rope on an obstacle (or a robot's link) for the
    whole replay: the crimp and boot inside a connector housing, the clip
    on a harness. The obstacle may move — one a robot attaches, carries and
    sets down — and the rope follows its baked pose; it never pulls back.

    ``length_m`` of material is held: inward from ``"Start"``/``"End"``,
    centred on any other location. Zero holds one sample, free to turn;
    a span also holds the rope's direction (a cable leaves its connector
    straight). Each held sample keeps the offset it has at time zero, so
    lay the rope's reference points where the housing holds it.

    The obstacle is a frame only: it collides with the rope only when it
    is also listed in ``obstacles`` (then it must be enabled).
    """
    if obstacle is not None and robot is None and link is None:
        return dict(obstacle=obstacle, location=location, length_m=length_m)
    if obstacle is None and robot is not None and link is not None:
        return dict(robot=robot, link=link, location=location, length_m=length_m)
    raise ValueError("an anchor names an obstacle, or a robot and its link")


def animate(scene, timeline, *, name, points, grippers=(), collision_links=(),
            obstacles=(), pins=(), connectors=(), anchors=(), spacing_m=0.02,
            radius_m=0.005, density_kg_m=0.1, step_s=1 / 240, axial_hz=500.0,
            bending_hz=20.0, friction=0.5, color=None):
    """Return a new timeline containing this rope; the input stays unchanged.

    ``collision_links`` contains ``{"robot": ..., "link": ...}`` entries;
    ``obstacles`` contains enabled scene obstacle names. Only selected
    geometry participates. Unsupported shapes fail with a reason.
    ``anchors`` are `anchor` declarations.
    ``axial_hz`` / ``bending_hz`` are native spring frequencies (defaults
    500 Hz axial / 20 Hz bending), not calibrated EA/EI: a higher bending
    frequency is a stiffer cable. ``friction`` is the rope collider's one
    coefficient. ``color`` is the display colour, linear RGB like an
    obstacle's.
    """
    if color is not None:
        color = [float(c) for c in color]
        if len(color) != 3 or not all(math.isfinite(c) and c >= 0 for c in color):
            raise ValueError(f"color must be three nonnegative linear RGB values, not {color}")
    payload = dict(name=name, points=[list(p) for p in points], grippers=list(grippers),
                   collision_links=list(collision_links), obstacles=list(obstacles),
                   pins=list(pins), connectors=list(connectors), anchors=list(anchors),
                   spacing_m=spacing_m, radius_m=radius_m, density_kg_m=density_kg_m,
                   step_s=step_s, axial_hz=axial_hz, bending_hz=bending_hz,
                   friction=friction, color=color)
    return scene._animate_rope_json(timeline, json.dumps(payload, allow_nan=False))


def track(timeline, name):
    """Playback data, including exact event times and captured local anchors."""
    return json.loads(timeline._rope_track_json(name))
