"""Gaits — how a robot mounted on a vehicle walks it.

To a cell, a quadruped or a humanoid is a vehicle with legs: it is sent
with ``bt.seq.goto``, arrives with ``bt.seq.device_done``, carries what sits
on its back, and has to fit the aisle like any AGV. What the vehicle
vocabulary lacks is the legs, and that is all a ``Gait`` adds — which links
are the feet, how the machine stands, and the rhythm it walks to. Hand it
to ``scene.mount_robot(..., gait=...)`` and the legs walk whenever the
vehicle drives; there is no walk action to author, any more than there is
a "spin the wheels" action for an AMR.

Nothing is simulated physically. The body rides the vehicle's closed-form
motion, the footfalls are planned from it the moment the vehicle is
dispatched (a foot lands where its stance will be centred under the body),
a planted foot never moves in the world, and each leg is solved by IK every
scan tick. What the bake answers is what it answers for any vehicle — does
it fit, does it clash, how long does the cycle take — with legs that move
like legs instead of a body that hovers.

    gait = bt.Gait(
        legs={"FL": "FL_foot", "FR": "FR_foot", "RL": "RL_foot", "RR": "RR_foot"},
        stance={"FL_hip_joint": 0.0, "FL_thigh_joint": 0.8, "FL_calf_joint": -1.5, ...},
        pattern="trot", period=0.5, lift=0.06, max_stride=0.4, foot_radius=0.022,
    )
    scene.add_vehicle("dog", body=[], path=..., stations=..., speed=0.6)
    scene.mount_robot("dog", robot="go2", gait=gait)   # stands it on the floor

A catalog package of category ``vehicle.legged`` carries all of this in
its manifest (the ``locomotion`` block the catalog builder validated the
package to walk with), so the cell does not copy joint names out of a URDF:

    dog = bt.Robot.from_catalog("unitree/go2/go2")
    gait = bt.Gait.from_catalog("unitree/go2/go2")   # or a package directory
    scene.mount_robot("dog", robot="go2", gait=gait)

A biped is the same thing with two legs, soles instead of balls, arms
that swing, and a body that bobs:

    gait = bt.Gait(
        legs={"L": "left_ankle_roll_link", "R": "right_ankle_roll_link"}, contact="sole",
        stance={...}, pattern="biped", period=0.9, lift=0.05, max_stride=0.5,
        foot_radius=0.035, arm_swing={"left_shoulder_pitch_joint": -0.25,
                                      "right_shoulder_pitch_joint": 0.25},
        bob=0.02, lateral=0.02,
    )
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

__all__ = ["Gait"]

_CONTACTS = ("point", "sole", "sole_yaw_free")
_PATTERNS = ("walk", "trot", "biped", "custom")


@dataclass
class Gait:
    """How a mounted robot walks. Every name is checked against the model
    when the robot is mounted.

    Attributes:
        legs: Leg name -> foot link, in the order the pattern's phase table
            is read: ``FL, FR, RL, RR`` for the quadruped patterns, ``L, R``
            for ``biped``. A value may also be ``(foot_link, contact)`` to
            give one leg its own contact.
        stance: Joint -> value of the standing pose. Must name every leg
            joint; other joints keep the value they had when mounted.
        pattern: ``walk`` (duty 0.75, lateral sequence), ``trot`` (duty 0.5,
            diagonal pairs), ``biped`` (duty 0.6), or ``custom`` with
            ``duty`` and ``phases``.
        period: Cycle period in seconds.
        lift: Swing apex above the floor, metres.
        max_stride: The longest step a leg may take between two landings:
            ``speed * period`` (and the pivot's outer-foot arc) must stay
            under it, or the bake refuses the vehicle's rates by name.
        foot_radius: How far the foot link's origin stands above the floor:
            a ball foot's radius, or an ankle frame's height over its sole.
            Zero for a frame on the sole itself.
        contact: ``point`` (a ball: position only), ``sole`` (flat on the
            floor, pointing where it landed) or ``sole_yaw_free``, for every
            leg not given its own. A sole's foot link must point +Z up in
            the stance; a 6-DOF leg keeps it flat and pointed, a 5-DOF leg
            keeps it flat.
        bob: Vertical sway of the body while walking, metres — up over each
            planted leg, down through double support, twice a cycle.
        lateral: Lean of the body over the planted leg, metres, once a
            cycle. Both are zero by default (a trotting quadruped's body
            rides nearly rigid); a biped wants a couple of centimetres.
        duty: Stance fraction of the cycle, ``custom`` pattern only.
        phases: Per-leg cycle phase in ``[0, 1)``, ``custom`` pattern only.
        body_link: The link the legs hang from; the root link by default.
        arm_swing: Joint -> amplitude (rad) swung in time with the first
            leg — a biped's arms. Left alone while the robot holds something
            or a ramp is driving them (a carried part rides still).
    """

    legs: Mapping[str, Any] | Sequence[tuple[str, Any]]
    stance: Mapping[str, float]
    pattern: str = "trot"
    period: float = 0.5
    lift: float = 0.06
    max_stride: float = 0.4
    foot_radius: float = 0.0
    contact: str = "point"
    duty: float | None = None
    phases: Sequence[float] | None = None
    body_link: str | None = None
    arm_swing: Mapping[str, float] = field(default_factory=dict)
    bob: float = 0.0
    lateral: float = 0.0

    @classmethod
    def from_catalog(
        cls, package: str | Path, *, revision: str | None = None, **overrides: Any
    ) -> Gait:
        """The gait a catalog package declares.

        ``package`` is a catalog id (``"unitree/go2/go2"`` — resolved and
        fetched like ``Robot.from_catalog``, pinned with ``revision``) or a
        package directory on disk, one holding ``manifest.yaml`` (a local
        build of the catalog builder, say). The manifest's ``locomotion``
        block — the feet, the stance, the rhythm the package was validated
        to walk with — becomes the Gait; keyword ``overrides`` replace any
        field (``period=0.5``). A package without the block (anything but
        ``vehicle.legged``) is refused by name.
        """
        directory = _package_dir(package, revision)
        manifest = _read_manifest(directory)
        ident = manifest.get("id") or str(directory)
        loc = manifest.get("locomotion")
        if not loc:
            raise ValueError(
                f"{ident}: manifest.yaml has no `locomotion` block (category "
                f"{manifest.get('category')!r}) — only a vehicle.legged package carries a gait"
            )
        defaults = loc.get("gait") or {}
        kwargs: dict[str, Any] = {
            "legs": {
                str(leg["name"]): (str(leg["foot"]), str(leg.get("contact") or "point"))
                for leg in loc.get("legs") or []
            },
            "stance": {str(j): float(v) for j, v in (loc.get("stance") or {}).items()},
            "pattern": str(defaults.get("pattern") or "trot"),
            "period": float(defaults.get("period_s", 0.5)),
            "lift": float(defaults.get("lift_m", 0.06)),
            "max_stride": float(defaults.get("max_stride_m", 0.4)),
            "foot_radius": float(loc.get("foot_radius_m") or 0.0),
            "body_link": loc.get("body_frame"),
            "arm_swing": {str(j): float(a) for j, a in (loc.get("arm_swing") or {}).items()},
            "bob": float(defaults.get("bob_m") or 0.0),
            "lateral": float(defaults.get("lateral_m") or 0.0),
        }
        kwargs.update(overrides)
        return cls(**kwargs)

    def _spec(self) -> dict:
        """The plain dict the extension reads (see `gait_from_py`)."""
        if self.pattern not in _PATTERNS:
            raise ValueError(f"pattern must be one of {_PATTERNS}, got {self.pattern!r}")
        if self.contact not in _CONTACTS:
            raise ValueError(f"contact must be one of {_CONTACTS}, got {self.contact!r}")
        if self.pattern == "custom" and (self.duty is None or self.phases is None):
            raise ValueError("a custom pattern needs duty and phases")
        items = self.legs.items() if isinstance(self.legs, Mapping) else list(self.legs)
        legs = []
        for name, foot in items:
            contact = self.contact
            if isinstance(foot, (tuple, list)):
                foot, contact = foot
                if contact not in _CONTACTS:
                    raise ValueError(f"leg {name!r}: contact must be one of {_CONTACTS}, got {contact!r}")
            legs.append((str(name), str(foot), str(contact)))
        spec: dict[str, Any] = {
            "legs": legs,
            "stance": [(str(j), float(v)) for j, v in self.stance.items()],
            "pattern": self.pattern,
            "period": float(self.period),
            "lift": float(self.lift),
            "max_stride": float(self.max_stride),
            "foot_radius": float(self.foot_radius),
            "arm_swing": [(str(j), float(a)) for j, a in self.arm_swing.items()],
            "body_link": self.body_link,
            "bob": float(self.bob),
            "lateral": float(self.lateral),
        }
        if self.pattern == "custom":
            spec["duty"] = float(self.duty)  # type: ignore[arg-type]
            spec["phases"] = [float(p) for p in self.phases]  # type: ignore[union-attr]
        return spec


def _package_dir(package: str | Path, revision: str | None) -> Path:
    """A package directory: the path given if it is one on disk (or its
    manifest.yaml), else the catalog package of that id, fetched."""
    path = Path(package)
    if path.is_dir():
        return path
    if path.is_file() and path.name == "manifest.yaml":
        return path.parent
    from . import _core  # the extension; lazy so tests can stand in for it

    return Path(_core.catalog_package(str(package), revision=revision))


def _read_manifest(directory: Path) -> dict[str, Any]:
    path = directory / "manifest.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"{directory}: no manifest.yaml — not a catalog package")
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover — pulled in by the catalog extra
        raise ImportError(
            "reading manifest.yaml needs PyYAML (installed with `pip install 'botrail[catalog]'`)"
        ) from exc
    manifest = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise TypeError(f"{path}: manifest.yaml is not a mapping")
    return manifest
