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


def anchor(obstacle=None, *, robot=None, link=None, location="Start", length_m=0.0, rail=None, roll=False):
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

    ``rail=dict(axis=(x, y, z), stroke_m=, force_n=)`` makes the anchor a
    carriage on a rail of the body — a retraction unit: the held span
    runs from its place along ``axis`` (world, at time zero) for
    ``stroke_m``, pulled back toward the start with ``force_n``. The static
    shape (``solver="static"``) runs the carriage; the replay holds it at
    the rail's start. ``roll=True`` says the holder holds the rope's roll
    about its own axis as well (a rigid clamp, a connector): the static
    rod then carries the twist between two such holders — it has no roll
    at a holder that leaves it free.
    """
    value = dict(location=location, length_m=length_m, roll=bool(roll))
    if rail is not None:
        value["rail"] = dict(axis=[float(v) for v in rail["axis"]], stroke_m=float(rail["stroke_m"]),
                             force_n=float(rail["force_n"]))
    if obstacle is not None and robot is None and link is None:
        return dict(obstacle=obstacle, **value)
    if obstacle is None and robot is not None and link is not None:
        return dict(robot=robot, link=link, **value)
    raise ValueError("an anchor names an obstacle, or a robot and its link")


def guide(obstacle=None, *, robot=None, link=None, position, axis, length_m, inner_m):
    """A ring on an obstacle (or a robot's link) the rope runs through — a
    sliding holder: a collider on the body, ``length_m`` long along
    ``axis`` with a bore of ``inner_m``, centred at ``position`` (world,
    time zero). The rope slides and rolls in it; nothing holds it there."""
    at = dict(position=[float(v) for v in position], axis=[float(v) for v in axis],
              length_m=float(length_m), inner_m=float(inner_m))
    if obstacle is not None and robot is None and link is None:
        return dict(obstacle=obstacle, **at)
    if obstacle is None and robot is not None and link is not None:
        return dict(robot=robot, link=link, **at)
    raise ValueError("a guide names an obstacle, or a robot and its link")


def animate(scene, timeline, *, name, points, grippers=(), collision_links=(),
            obstacles=(), pins=(), connectors=(), anchors=(), guides=(), spacing_m=0.02,
            radius_m=0.005, density_kg_m=0.1, step_s=1 / 240, axial_hz=500.0,
            bending_hz=20.0, friction=0.5, color=None, ground_z=None, preroll_s=0.0,
            extra_colliders=(), rest="start", rod_hz=0.0, solver="physics", rigidity_nm2=None):
    """Return a new timeline containing this rope; the input stays unchanged.

    ``collision_links`` contains ``{"robot": ..., "link": ...}`` entries;
    ``obstacles`` contains enabled scene obstacle names. Only selected
    geometry participates. Unsupported shapes fail with a reason.
    ``anchors`` are `anchor` declarations; ``guides`` are `guide` rings the
    rope runs through.
    ``axial_hz`` / ``bending_hz`` are native spring frequencies (defaults
    500 Hz axial / 20 Hz bending), not calibrated EA/EI: a higher bending
    frequency is a stiffer cable. The bending springs rest in the shape the
    rope is laid in (``rest="start"``: a pre-formed harness keeps its bends,
    and so does a corner of the laid line) or straight (``rest="straight"``:
    a tube or a cable off the reel, bent only by what holds it and by its
    weight). ``rod_hz`` makes the rope a rod: windows of a few particles
    along it are each drawn toward their own rest shape by a spring of
    that natural frequency, so a bend anywhere is resisted — what makes a
    tube hold a round loop between its holders instead of hanging in a V;
    the bending springs alone resist a fold and little else. Zero (the
    default) for none; a spring setting, not a calibrated flexural
    rigidity. ``friction`` is the rope collider's one
    coefficient. ``color`` is the display colour, linear RGB like an
    obstacle's. ``ground_z`` adds a floor at that height for the rope to
    rest on (the cell has no floor obstacle of its own). ``preroll_s``
    settles the rope under gravity for that long, everything that holds it
    parked at its time-zero pose, before the clock starts — a rope laid
    along a design polyline is not yet hanging. ``extra_colliders`` are
    static shapes of the replay's own, ``(pose, shape)`` pairs like a
    connector's (``dict(position=, quaternion=)``, ``{"Cuboid": [hx, hy,
    hz]}``) — the floor and walls of a duct the cell models as one box.

    ``solver="static"`` draws the shape instead of simulating it: at every
    frame the line is the inextensible rod at rest between what holds it
    — of least bending and gravitational energy, held in position (and in
    direction, where an anchor holds two or more samples), clear of the
    collision links, the obstacles, the extra colliders and the floor —
    with no dynamics, no settling and no spring settings: what resists the
    bend is a flexural rigidity, ``rigidity_nm2`` (N·m²; a polyurethane
    tube's for the line's diameter when ``None`` — 1.5e-3 at 6 mm, with
    the fourth power of the diameter), against the line's weight. The
    shape at a frame does not depend on how fast the cycle ran to it, and
    the laid shape only picks which rest the solve settles to (a loop stays
    on the side it is laid). Grippers, connectors and the rope's spring
    settings do not apply to it; the track records 30 frames a second.
    """
    payload = _payload(name=name, points=points, grippers=grippers, collision_links=collision_links,
                       obstacles=obstacles, pins=pins, connectors=connectors, anchors=anchors, guides=guides,
                       spacing_m=spacing_m, radius_m=radius_m, density_kg_m=density_kg_m, step_s=step_s,
                       axial_hz=axial_hz, bending_hz=bending_hz, friction=friction, color=color,
                       ground_z=ground_z, preroll_s=preroll_s, extra_colliders=extra_colliders, rest=rest,
                       rod_hz=rod_hz, solver=solver, rigidity_nm2=rigidity_nm2)
    return scene._animate_rope_json(timeline, json.dumps(payload, allow_nan=False))


def _payload(*, name, points, grippers=(), collision_links=(), obstacles=(), pins=(), connectors=(),
             anchors=(), guides=(), spacing_m=0.02, radius_m=0.005, density_kg_m=0.1, step_s=1 / 240, axial_hz=500.0,
             bending_hz=20.0, friction=0.5, color=None, ground_z=None, preroll_s=0.0, extra_colliders=(),
             rest="start", rod_hz=0.0, solver="physics", rigidity_nm2=None):
    if rest not in ("start", "straight"):
        raise ValueError(f'rest is "start" or "straight", not {rest!r}')
    if not (math.isfinite(float(rod_hz)) and float(rod_hz) >= 0.0):
        raise ValueError(f"rod_hz must be finite and nonnegative, not {rod_hz!r}")
    if solver not in ("physics", "static"):
        raise ValueError(f'solver is "physics" or "static", not {solver!r}')
    if rigidity_nm2 is not None and not (math.isfinite(float(rigidity_nm2)) and float(rigidity_nm2) > 0.0):
        raise ValueError(f"rigidity_nm2 must be finite and positive, not {rigidity_nm2!r}")
    if color is not None:
        color = [float(c) for c in color]
        if len(color) != 3 or not all(math.isfinite(c) and c >= 0 for c in color):
            raise ValueError(f"color must be three nonnegative linear RGB values, not {color}")
    return dict(name=name, points=[list(p) for p in points], grippers=list(grippers),
                collision_links=list(collision_links), obstacles=list(obstacles),
                pins=list(pins), connectors=list(connectors), anchors=list(anchors), guides=list(guides),
                spacing_m=spacing_m, radius_m=radius_m, density_kg_m=density_kg_m,
                step_s=step_s, axial_hz=axial_hz, bending_hz=bending_hz,
                friction=friction, color=color,
                ground_z=None if ground_z is None else float(ground_z),
                preroll_s=float(preroll_s), extra_colliders=[list(c) for c in extra_colliders], rest=rest,
                rod_hz=float(rod_hz), solver=solver,
                rigidity_nm2=None if rigidity_nm2 is None else float(rigidity_nm2))


def animate_many(scene, timeline, passes):
    """Several ropes against one bake in one call, computed side by side
    (ropes do not interact, so the result is the one :func:`animate` would
    give one after another). ``passes`` are dicts of :func:`animate`'s
    keyword arguments. Returns ``(timeline, errors)``: the timeline with
    every rope that ran, and per pass ``None`` or the reason it did not."""
    payloads = [json.dumps(_payload(**p), allow_nan=False) for p in passes]
    out, errors = scene._animate_ropes_json(timeline, payloads)
    return out, list(errors)


def track(timeline, name):
    """Playback data, including exact event times and captured local anchors."""
    return json.loads(timeline._rope_track_json(name))
