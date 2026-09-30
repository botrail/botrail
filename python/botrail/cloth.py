"""Cloth in a cell: a T-shirt or a sheet handled by the robots' grippers.

The cloth is simulated after a bake, against the finished cycle: each
gripper's tool point follows its robot through the cycle and closes while a
signal is on. The cloth never pushes a robot back, and the robots plan as if
it were not there.

    shirt = bt.cloth.tshirt("shirt", on="table/top", spacing=0.02)
    marks = bt.cloth.landmarks(scene, shirt)   # frames shirt/cuff_left, ...
    # ... teach motions against the landmarks; set a signal where a gripper closes ...
    bt.cloth.add(scene, shirt, grippers=[bt.cloth.gripper("grip", radius=0.04)])
    tl = scene.simulate_sequence("fold")       # the cloth rides every bake
    tl.cloth("shirt").position("cuff_left", 3.0)

A cloth lies flat on a level support surface: the frame named by ``on`` (a
table's ``<name>/top``) gives its centre, the support height and its
heading; ``position`` gives the centre and the height without a frame. At
``yaw=0`` the width runs along ``+x`` and the length — a T-shirt's hem to
its shoulders — along ``-y``.
"""

import json
from collections.abc import Iterable, Sequence
from typing import Any, Dict, Optional, Tuple

Cloth = Dict[str, Any]
Gripper = Dict[str, Any]


def _cloth(
    name: str,
    kind: str,
    on: Optional[str],
    position: Optional[Sequence[float]],
    yaw: float,
    spacing: float,
    shell: Dict[str, Any],
) -> Cloth:
    cloth: Cloth = {"name": name, "kind": kind, "yaw": float(yaw), "spacing": float(spacing)}
    if on is not None:
        cloth["on"] = on
    if position is not None:
        cloth["position"] = [float(v) for v in position]
    cloth.update({key: value for key, value in shell.items() if value is not None})
    return cloth


def tshirt(
    name: str,
    *,
    on: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    yaw: float = 0.0,
    spacing: float = 0.02,
    layers: str = "sewn",
    neck: str = "notch",
    seams: str = "shared",
    body: Optional[Tuple[float, float]] = None,
    sleeve: Optional[Tuple[float, float]] = None,
    thickness: Optional[float] = None,
    band: Optional[float] = None,
    friction: Optional[float] = None,
    density: Optional[float] = None,
    workers: Optional[int] = None,
    max_iterations: Optional[int] = None,
) -> Cloth:
    """A T-shirt lying flat, front up.

    ``layers`` is ``"sewn"`` (a front and a back joined along the outline)
    or ``"single"`` (one panel); ``neck`` ``"notch"`` or ``"round"``;
    ``seams`` ``"shared"`` or ``"stitched"``. ``body`` is ``(width,
    length)`` (0.50 x 0.70 m by default) and ``sleeve`` ``(length, width)``
    (0.22 x 0.20 m). ``spacing`` is the mesh cell: 0.02 m is the full-size
    shirt (about two thousand vertices, minutes to simulate); 0.05 m a
    coarse one that runs in seconds.

    Landmarks: ``cuff_left/right``, ``hem_left/right/center``,
    ``shoulder_left/right``, ``underarm_left/right``, ``neck_front/back``,
    ``chest``.
    """
    cloth = _cloth(
        name,
        "tshirt",
        on,
        position,
        yaw,
        spacing,
        dict(
            thickness=thickness,
            band=band,
            friction=friction,
            density=density,
            workers=workers,
            max_iterations=max_iterations,
        ),
    )
    cloth.update(layers=layers, neck=neck, seams=seams)
    if body is not None:
        cloth["body"] = [float(body[0]), float(body[1])]
    if sleeve is not None:
        cloth["sleeve"] = [float(sleeve[0]), float(sleeve[1])]
    return cloth


def sheet(
    name: str,
    size: Tuple[float, float],
    *,
    on: Optional[str] = None,
    position: Optional[Sequence[float]] = None,
    yaw: float = 0.0,
    spacing: float = 0.02,
    thickness: Optional[float] = None,
    band: Optional[float] = None,
    friction: Optional[float] = None,
    density: Optional[float] = None,
    workers: Optional[int] = None,
    max_iterations: Optional[int] = None,
) -> Cloth:
    """A rectangular sheet ``size = (width, length)`` lying flat.

    Landmarks, as it lies at ``yaw=0`` (north is ``+y``, east ``+x``):
    ``corner_nw/ne/sw/se``, ``edge_n/s/w/e``, ``center``.
    """
    cloth = _cloth(
        name,
        "sheet",
        on,
        position,
        yaw,
        spacing,
        dict(
            thickness=thickness,
            band=band,
            friction=friction,
            density=density,
            workers=workers,
            max_iterations=max_iterations,
        ),
    )
    cloth["size"] = [float(size[0]), float(size[1])]
    return cloth


def gripper(
    signal: str,
    *,
    robot: Optional[str] = None,
    group: Optional[str] = None,
    link: Optional[str] = None,
    radius: float = 0.04,
    compliance: float = 0.0,
    soften: int = 0,
    pad: Optional[Sequence[float]] = None,
    pad_offset: Sequence[float] = (0.0, 0.0, 0.0),
) -> Gripper:
    """A gripper that handles cloth: it holds while ``signal`` is on.

    Its tool point is ``link``'s frame, else the tip of ``group`` (one arm
    of a dual-arm robot), else the robot's TCP. When the signal turns on,
    the gripper takes the cloth vertices within ``radius`` of the tool
    point, through every layer — or takes over the patch another gripper
    holds there, which is a hand-over. When it turns off it lets go.

    ``compliance`` 0 pins the patch; a positive value (m/N) holds it on
    springs, which a flap folded by two grippers needs (0.02 for a
    T-shirt's hem). ``soften`` is the number of cloth steps over which the
    hold weakens before it opens, so a taut flap settles instead of
    snapping free.

    ``pad`` gives the gripper a box (full extents, in the tool frame,
    centred at ``pad_offset``) that pushes cloth it does not hold; without
    one it passes through cloth.
    """
    out: Gripper = {
        "signal": signal,
        "radius": float(radius),
        "compliance": float(compliance),
        "soften": int(soften),
    }
    if robot is not None:
        out["robot"] = robot
    if group is not None:
        out["group"] = group
    if link is not None:
        out["link"] = link
    if pad is not None:
        out["pad"] = {
            "size": [float(v) for v in pad],
            "offset": [float(v) for v in pad_offset],
        }
    return out


def _payload(cloth: Cloth, grippers: Iterable[Gripper], step: float) -> str:
    return json.dumps({"cloth": cloth, "grippers": list(grippers), "step": float(step)})


def landmarks(scene, cloth: Cloth, frames: bool = True) -> Dict[str, Tuple[float, float, float]]:
    """Where the cloth's landmarks lie before anything moves it.

    With ``frames`` (the default) each is also registered as the frame
    ``<cloth name>/<landmark>``, so motions and toolpaths can be taught
    against it. Nothing is simulated.
    """
    marks = {name: tuple(p) for name, p in scene._cloth_landmarks_json(_payload(cloth, (), 0.1))}
    if frames:
        for name, position in marks.items():
            scene.add_frame(f"{cloth['name']}/{name}", position)
    return marks


def add(scene, cloth: Cloth, grippers: Iterable[Gripper] = (), step: float = 0.1) -> None:
    """Puts the cloth in the cell: every bake from here on — a
    ``simulate_sequence`` call or the studio's simulate button — is followed
    by the cloth's simulation against the baked cycle, and its timeline
    carries the track (``timeline.cloth(name)``). Adding a cloth of the
    same name replaces it.

    ``step`` is the cloth's time step (s); 0.1 is what the solver is
    qualified at.
    """
    scene._cloth_add_json(_payload(cloth, grippers, step))


def remove(scene, name: str) -> bool:
    """Takes a cloth out of the cell; False if it was not there."""
    return scene._cloth_remove(name)


def animate(scene, timeline, cloth: Cloth, grippers: Iterable[Gripper] = (), step: float = 0.1):
    """Simulates the cloth against an already baked ``timeline`` and returns
    the timeline with its track, without putting the cloth in the cell —
    for trying cloth settings on one cycle without baking it again."""
    return scene._animate_cloth_json(timeline, _payload(cloth, grippers, step))
