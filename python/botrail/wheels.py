"""Wheels — how a robot mounted on a vehicle rolls it.

A humanoid upper body on a wheeled base — a semi-humanoid — is one machine:
one purchase, one URDF, arms and torso and wheels in one joint tree. To a
cell it is what a legged machine is, a vehicle whose running gear is the
robot itself: it is sent with ``bt.seq.goto``, arrives with
``bt.seq.device_done``, and has to fit the aisle like any AGV. ``Wheels`` is
what the vehicle vocabulary lacks for it — which joints are the wheels, how
big they are, and which frame stands on the floor. Hand it to
``scene.mount_robot(..., wheels=...)`` and the wheels turn whenever the
vehicle drives; there is no "spin the wheels" action to author.

Nothing is simulated physically. The vehicle's checked motion is the only
input: each wheel turns by exactly the distance its hub travelled — forward
on a straight, opposite ways through a pivot, not at all on a lift ride —
and the turning is baked into the joint track alongside whatever the arms
do on the way. What the declaration changes beyond the picture:

- the machine is one row of the BOM (the vehicle is not a second purchase),
- a ``walkable`` surface under it — a floor slab, a lift car's floor — is
  where it rolls, not a collision,
- the wheel joints are the mount's: no motion or ramp may drive them.

    wheels = bt.Wheels(
        {"left_wheel_joint": 0.085, "right_wheel_joint": 0.085},
        base_frame="base_footprint",
    )
    scene.add_vehicle("base", body=[], path=..., stations=..., speed=0.8)
    scene.mount_robot("base", robot="g1d", wheels=wheels)   # stands it on the floor

A catalog package of category ``vehicle.mobile_manipulator`` carries all of
this in its manifest (the ``locomotion`` block the catalog builder validated
the package to drive with), and so does a ``vehicle.legged`` one whose block
has ``wheels`` (``Wheels.rolls(package)`` says which):

    robot = bt.Robot.from_catalog("unitree/g1/g1-d")
    wheels = bt.Wheels.from_catalog("unitree/g1/g1-d")
    scene.add_vehicle("base", body=[], path=..., drive=wheels.vehicle_drive, ...)
    scene.mount_robot("base", robot="g1d", wheels=wheels)

A mecanum wheel adds its rollers' handedness, a swerve module its steering
joint:

    bt.Wheels({"fl_wheel": (0.076, -1), "fr_wheel": (0.076, +1),
               "rl_wheel": (0.076, +1), "rr_wheel": (0.076, -1)}, drive="mecanum")
    bt.Wheels({"wheel_1": 0.06, "wheel_2": 0.06, "wheel_3": 0.06},
              steer={"wheel_1": "steer_1", "wheel_2": "steer_2", "wheel_3": "steer_3"},
              drive="swerve")

A wheel-legged quadruped — a Go2-W, a B2-W: wheels for feet — is mounted
with a ``Gait`` *and* ``Wheels``. The gait's feet are the axle frames the
wheels turn on (a frame fixed to each calf at the wheel joint's origin), its
``foot_radius`` is the wheel radius, and ``mode`` says which gear takes the
route. ``"auto"`` (the default) decides leg by leg of every route from the
floor under it: rolled where the floor is continuous, walked where it
steps — a stair flight, a kerb taller than ``max_step`` — from a wheel's
leading edge before the first step to a hind foot's last landing past the
last one, a turn walked when either straight beside it is, a roll too short
to be worth the change walked with its neighbours; the walked legs go at the
gait's ``speed``, the rest at the vehicle's. ``"roll"`` holds the legs in
the posture the machine was mounted in and turns every wheel by its hub's
travel, refusing a step by name; ``"walk"`` locks the wheels and walks on
them as on feet everywhere, wheel radius for foot radius.

    gait = bt.Gait(legs={"FL": "FL_axle", ...}, foot_radius=0.086, speed=0.4, ...)
    wheels = bt.Wheels({"FL_foot_joint": 0.086, ...})
    scene.mount_robot("dog", robot="go2w", gait=gait, wheels=wheels)
    tl.locomotion("go2w")     # [(t0, t1, "roll" | "walk", metres), ...]
"""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

from .gait import _package_dir, _read_manifest

__all__ = ["Wheels"]

_DRIVES = ("differential", "skid", "mecanum", "omni", "swerve")
_HOLONOMIC = ("mecanum", "omni", "swerve")
_MODES = ("auto", "roll", "walk")


@dataclass
class Wheels:
    """How a mounted robot rolls. Every name is checked against the model
    when the robot is mounted.

    Attributes:
        wheels: Wheel joint -> rolling radius in metres, or ``(radius,
            lateral)`` for a mecanum wheel: ``-1`` / ``+1`` for the two
            diagonals of the usual 45 degree arrangement (front-left and
            rear-right ``-1``). The joints must be continuous with level
            axles; which way an axis points does not matter — mirrored
            wheels both roll forward. May be empty: a model whose wheels
            are part of the chassis mesh still declares that it rolls.
        steer: Wheel joint -> the joint that steers it about the vertical
            (a swerve module). It is aimed along the wheel's travel, by the
            shorter turn.
        base_frame: The link whose origin is the machine's point on the
            floor, under its turn centre — a URDF's ``base_footprint``. It
            is what rides the vehicle frame when ``mount_robot`` is given
            no offset. Without one the wheels' contact plane under their
            centroid is used. Must be fixed to the root link.
        posture: Joint -> value the machine is put in as it is mounted —
            how it travels (torso down, arms tucked). Other joints keep
            the value they had.
        drive: What the base is — ``differential``, ``skid``, ``mecanum``,
            ``omni`` or ``swerve``. The wheels turn the same either way;
            this says how the vehicle should be driven
            (:attr:`vehicle_drive`).
        mode: For a machine mounted with a gait as well (a wheel-legged
            quadruped): ``"auto"`` (the default) rolls or walks each leg of
            a route as the floor under it decides; ``"roll"`` takes every
            route on the wheels, the legs held in the mounted posture, and
            refuses a step by name; ``"walk"`` locks the wheels and walks on
            them everywhere. Read only when a gait is given.
        max_step: The tallest break in the floor the wheels roll over,
            metres — a kerb, a threshold. Zero, the default, walks (or
            refuses) every step; state a figure the wheels are rated for.
    """

    wheels: Mapping[str, Any] = field(default_factory=dict)
    steer: Mapping[str, str] = field(default_factory=dict)
    base_frame: str | None = None
    posture: Mapping[str, float] = field(default_factory=dict)
    drive: str = "differential"
    mode: str = "auto"
    max_step: float = 0.0

    @property
    def vehicle_drive(self) -> str:
        """The ``add_vehicle(drive=...)`` this base wants: ``holonomic`` for
        mecanum / omni / swerve (it translates any way holding its
        heading), ``differential`` otherwise (it turns on the spot and
        drives where it faces)."""
        return "holonomic" if self.drive in _HOLONOMIC else "differential"

    @classmethod
    def from_catalog(
        cls,
        package: str | Path,
        *,
        revision: str | None = None,
        posture: str | None = "travel",
        **overrides: Any,
    ) -> Wheels:
        """The wheels a catalog package declares.

        ``package`` is a catalog id (``"unitree/g1/g1-d"`` — resolved and
        fetched like ``Robot.from_catalog``, pinned with ``revision``) or a
        package directory on disk. The manifest's ``locomotion`` block of
        ``kind: wheeled`` and its ``frames.base_frame`` become the Wheels;
        keyword ``overrides`` replace any field. A legged package whose
        block carries ``wheels`` — a wheel-legged machine, a Go2-W — gives
        its wheels too, to be mounted beside ``bt.Gait.from_catalog`` of the
        same package (``mode`` ``"auto"``, and the wheels' ``max_step_mm``).
        A package that only walks, or carries no block, is refused by name.

        ``posture`` names one of the package's ``locomotion.postures``;
        the default takes ``travel`` when the package states it. Pass
        ``None`` to mount the machine as it stands. A wheel-legged package
        has one posture, ``wheels.posture``, laid over the stance.
        """
        directory = _package_dir(package, revision)
        manifest = _read_manifest(directory)
        ident = manifest.get("id") or str(directory)
        loc = manifest.get("locomotion")
        legged = loc.get("wheels") if loc and loc.get("kind") != "wheeled" else None
        if legged:
            kwargs: dict[str, Any] = {
                "wheels": {str(w["joint"]): float(w["radius_m"]) for w in legged.get("joints") or []},
                "base_frame": (manifest.get("frames") or {}).get("base_frame"),
                "posture": {str(j): float(v) for j, v in (legged.get("posture") or {}).items()},
                "drive": str(legged.get("drive") or "skid"),
                "max_step": float(legged.get("max_step_mm") or 0.0) / 1000.0,
            }
            kwargs.update(overrides)
            return cls(**kwargs)
        if not loc or loc.get("kind") != "wheeled":
            kind = (loc or {}).get("kind")
            hint = (
                f"its locomotion is `{kind}` with no `wheels` — it walks, see bt.Gait.from_catalog"
                if kind
                else f"manifest.yaml has no `locomotion` block (category {manifest.get('category')!r})"
            )
            raise ValueError(
                f"{ident}: not a wheeled machine: {hint}; only a package with "
                f"`locomotion.kind: wheeled`, or a legged one with `locomotion.wheels`, "
                f"carries wheels"
            )
        postures = loc.get("postures") or {}
        if posture is not None and posture not in postures and posture != "travel":
            raise ValueError(
                f"{ident}: manifest.yaml declares no `locomotion.postures.{posture}` "
                f"(it has: {', '.join(sorted(postures)) or 'none'})"
            )
        kwargs: dict[str, Any] = {
            "wheels": {
                str(w["joint"]): (float(w["radius_m"]), float(w.get("lateral") or 0.0))
                for w in loc.get("wheels") or []
            },
            "steer": {
                str(w["joint"]): str(w["steer"]) for w in loc.get("wheels") or [] if w.get("steer")
            },
            "base_frame": (manifest.get("frames") or {}).get("base_frame"),
            "posture": {str(j): float(v) for j, v in (postures.get(posture) or {}).items()},
            "drive": str(loc.get("drive") or "differential"),
        }
        kwargs.update(overrides)
        return cls(**kwargs)

    @staticmethod
    def postures(package: str | Path, *, revision: str | None = None) -> tuple[str, ...]:
        """The postures a package states (``locomotion.postures``) — the
        names :meth:`from_catalog` takes as ``posture=``."""
        loc = _read_manifest(_package_dir(package, revision)).get("locomotion") or {}
        return tuple(loc.get("postures") or {})

    @staticmethod
    def rolls(package: str | Path, *, revision: str | None = None) -> bool:
        """Whether a package carries wheels at all: a ``kind: wheeled`` block,
        or a legged block with ``wheels`` (a wheel-legged machine) — what to
        ask before :meth:`from_catalog` rather than catch its refusal."""
        loc = _read_manifest(_package_dir(package, revision)).get("locomotion") or {}
        return loc.get("kind") == "wheeled" or bool(loc.get("wheels"))

    def _spec(self) -> dict:
        """The plain dict the extension reads (see `wheels_from_py`)."""
        if self.drive not in _DRIVES:
            raise ValueError(f"drive must be one of {_DRIVES}, got {self.drive!r}")
        if self.mode not in _MODES:
            raise ValueError(f"mode must be one of {_MODES}, got {self.mode!r}")
        unknown = [joint for joint in self.steer if joint not in self.wheels]
        if unknown:
            raise ValueError(f"steer names wheels that are not declared: {unknown}")
        wheels = []
        for joint, value in self.wheels.items():
            radius, lateral = value if isinstance(value, (tuple, list)) else (value, 0.0)
            steer = self.steer.get(joint)
            wheels.append((str(joint), float(radius), float(lateral), None if steer is None else str(steer)))
        return {
            "wheels": wheels,
            "base_frame": self.base_frame,
            "posture": [(str(j), float(v)) for j, v in self.posture.items()],
            "mode": self.mode,
            "max_step": float(self.max_step),
        }
