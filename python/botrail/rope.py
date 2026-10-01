"""Replay a cable against a baked cycle, in Z-up metres and seconds.

The independent Rapier 0.36 f64 world never applies reactions to the
source bake. Grippers attach explicit material points without teleporting
them; source signal times split the replay clock, preserving short pulses.
"""
import json


def gripper(signal, *, robot, link, location="Start"):
    """Bind a signal to a named robot/link and Start/End/material location.

    ``location`` may also be ``{"ArcLength": {"arc_length_m": ..., "tolerance_m": ...}}``.
    """
    return dict(signal=signal, robot=robot, link=link, location=location)


def animate(scene, timeline, *, name, points, grippers=(), collision_links=(),
            obstacles=(), pins=(), connectors=(), spacing_m=0.02,
            radius_m=0.005, density_kg_m=0.1, step_s=1 / 240):
    """Return a new timeline containing this rope; the input stays unchanged.

    ``collision_links`` contains ``{"robot": ..., "link": ...}`` entries;
    ``obstacles`` contains enabled scene obstacle names. Only selected
    geometry participates. Unsupported shapes fail with a reason.
    Native frequency settings are 500 Hz axial / 20 Hz bending, not calibrated EA/EI.
    """
    payload = dict(name=name, points=[list(p) for p in points], grippers=list(grippers),
                   collision_links=list(collision_links), obstacles=list(obstacles),
                   pins=list(pins), connectors=list(connectors), spacing_m=spacing_m,
                   radius_m=radius_m, density_kg_m=density_kg_m, step_s=step_s)
    return scene._animate_rope_json(timeline, json.dumps(payload, allow_nan=False))


def track(timeline, name):
    """Playback data, including exact event times and captured local anchors."""
    return json.loads(timeline._rope_track_json(name))
