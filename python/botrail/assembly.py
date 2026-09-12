"""Assembly: fit and fasten (design/design-assembly.md).

A mechanical assembly cell does two things with a part: it *fits* it —
carries it onto its mate, over pins, into a bore — and *fastens* it, one
screw at a time. botrail says both in the vocabulary it already has. A
fit is an `attach`, a move and a `detach`, plus a report of where the
part landed against where the joint says it goes. A fastening is that
same carry for the screw, then the screwdriver's own stroke running it
down at the thread's feed, while a **driver program** — one more PLC
program scanned beside the robot's, the way a machine tool's is — times
the rundown from the screw's pitch and answers OK or NOK. Threads, torque
curves and cam-out are not simulated: the numbers are data with a source,
the motion is geometry, the result is a signal.

    fastener = bt.assembly.iso4762(5, 20)                   # M5×20, 8.8
    holes = [bt.assembly.Hole(f"h{i}", xy, "threaded", depth_mm=13) ...]
    joint = bt.assembly.joint(scene, "cover", a="housing", b="cover",
                              pattern_a=bt.assembly.BoltPattern(holes), pattern_b=...,
                              fastener=fastener, seat=scene.frame("housing/top"),
                              thickness=0.010, torque_nm=(4.0, 5.0), min_engagement_mm=10)
    driver = bt.assembly.driver(scene, "driver", robot="arm", bit="drv_bit",
                                shank="drv_shank", stroke=0.055, fastener=fastener,
                                torque_nm=4.5, cycles=len(joint.order) + 1)
    sq = scene.sequence("assemble")
    fastening = bt.assembly.fasten(sq, joint, driver, feeder, motions={...})
    tl = scene.simulate_sequences(["assemble", driver.program])
    rows = bt.assembly.fastening_report(tl, fastening)

A fitted part meets what it is fitted into, and the collision check has
to know that is intended: `fasten` and `place` declare it
(`Scene.allow_object_obstacle_contact`) — the cover may meet its housing
and dowels anywhere, a screw may be inside the cover and the housing only
**within its hole's window** (the hole's axis, the hole's tolerance), so
a screw taught off its hole is still a collision the bake refuses.

What a joint knows is what a drawing states: the holes of each part
(threaded or clearance, depth, grip), the locators (pins and their holes),
the screw, the torque the joint is designed for and the engagement it
needs. From that come the hole frames a teach aims at, the **order** the
screws go in (a star pattern, opposite pairs from the centre out), the
**engagement** (length − grip − washer − thread start), the tip's depth
against the hole's bottom, and the checks a driver has to pass — its
torque range, its bit, its stroke. `Fastener` figures for ISO 4762 socket
head cap screws are tabled (`iso4762`), with the ISO 898-1 reference
torque at μ = 0.14 as a ceiling, never as the joint's own value.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Union

from . import parts, seq
from .select import Finding

__all__ = [
    "ISO_4762",
    "TORQUE_REF_NM",
    "BoltPattern",
    "Driver",
    "Fastener",
    "Fastening",
    "Hole",
    "Joint",
    "Locator",
    "Placement",
    "check",
    "driver",
    "export_sheet",
    "fasten",
    "fastening_report",
    "fit_report",
    "iso4762",
    "joint",
    "place",
    "report_section",
    "rundown_s",
    "star_order",
]

Point2 = tuple[float, float]
Point3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
Pose = tuple[Point3, Quat]

#: ISO 4762 hexagon socket head cap screws, coarse thread: thread → (pitch,
#: head diameter dk, head height k, hexagon socket s), millimetres.
ISO_4762: dict[int, tuple[float, float, float, float]] = {
    3: (0.5, 5.5, 3.0, 2.5),
    4: (0.7, 7.0, 4.0, 3.0),
    5: (0.8, 8.5, 5.0, 4.0),
    6: (1.0, 10.0, 6.0, 5.0),
    8: (1.25, 13.0, 8.0, 6.0),
    10: (1.5, 16.0, 10.0, 8.0),
    12: (1.75, 18.0, 12.0, 10.0),
}

#: Reference tightening torque (N·m) for ISO 898-1 property classes at a
#: friction coefficient of 0.14 (TR Fastenings' table of the usual VDI 2230
#: figures) — the *ceiling* a steel joint is tightened to, not a joint's
#: own design value: a tapped aluminium housing is tightened to less.
TORQUE_REF_NM: dict[str, dict[int, float]] = {
    "8.8": {3: 1.37, 4: 3.1, 5: 6.15, 6: 10.5, 8: 26.0, 10: 51.0, 12: 89.0},
    "10.9": {3: 1.92, 4: 4.4, 5: 8.65, 6: 15.0, 8: 36.0, 10: 72.0, 12: 125.0},
}

_STEEL_KG_PER_M3 = 7850.0


# ---------------------------------------------------------------- fastener
@dataclass(frozen=True)
class Fastener:
    """One screw, by its figures (millimetres): the thread and its pitch,
    the length under the head, the head's diameter `head_dk_mm` and
    height `head_k_mm`, the drive's across-flats `drive_s_mm` (the bit it
    takes), the head standard and property class, and — where a table
    gives one — the reference torque `torque_ref_nm` it must not be
    tightened beyond. `model` is the part number it goes on the bill as."""

    thread_mm: float
    pitch_mm: float
    length_mm: float
    head_dk_mm: float
    head_k_mm: float
    drive_s_mm: float
    head_standard: str = "ISO 4762"
    property_class: str = "8.8"
    model: Optional[str] = None
    mass_kg: Optional[float] = None
    torque_ref_nm: Optional[float] = None
    #: The catalog pack it was read from (`from_catalog`), as
    #: `set_part(catalog=...)` records it — `None` for a tabled screw.
    catalog: Optional[tuple] = None
    #: How that pack was named to `from_catalog` — an id, or a package
    #: directory — so `place` can open it again.
    source: Optional[object] = None

    def __post_init__(self) -> None:
        for key in ("thread_mm", "pitch_mm", "length_mm", "head_dk_mm", "head_k_mm", "drive_s_mm"):
            value = getattr(self, key)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"Fastener: {key} must be finite and positive")

    @property
    def model_name(self) -> str:
        """The part number: what was given, else `<standard> M<d>x<L>-<class>`."""
        if self.model:
            return self.model
        d, length = _plain(self.thread_mm), _plain(self.length_mm)
        return f"{self.head_standard} M{d}x{length}-{self.property_class}"

    @property
    def length_m(self) -> float:
        return self.length_mm / 1000.0

    @property
    def head_m(self) -> float:
        return self.head_k_mm / 1000.0

    def place(self, scene, name: str, position: Point3, quaternion: Optional[Quat] = None,
              *, manufacturer: Optional[str] = None, **attributes) -> str:
        """The screw as an obstacle (`bt.parts.bolt`): its tip at `position`,
        +Z up the shank, pinned as a `fastener` with these figures — and to
        its pack, for a screw read `from_catalog`."""
        if self.catalog is not None:
            return parts.bolt(
                scene, name, position=position, quaternion=quaternion,
                catalog=self.source if self.source is not None else self.catalog,
                length=self.length_m, property_class=self.property_class,
                manufacturer=manufacturer, **attributes,
            )
        return parts.bolt(
            scene, name, thread=self.thread_mm / 1000.0, length=self.length_m,
            head_diameter=self.head_dk_mm / 1000.0, head_height=self.head_m,
            position=position, quaternion=quaternion, model=self.model_name,
            manufacturer=manufacturer, standard=self.head_standard,
            property_class=self.property_class, mass_kg=self.mass_kg,
            pitch_mm=self.pitch_mm, drive_s_mm=self.drive_s_mm, **attributes,
        )

    @classmethod
    def from_catalog(cls, ref, *, length_mm: Optional[float] = None,
                     property_class: Optional[str] = None) -> "Fastener":
        """A screw from a fastener spec pack — the id, or a package
        directory: the thread, pitch, head and drive the pack's `screw`
        component states, the `length_mm` and `property_class` it sells
        (its defaults when omitted), the article number, the mass and the
        class's reference torque (`torque_ref_<class>_nm` in its specs)."""
        from ._spec import Spec

        spec = Spec.load(ref)
        spec.expect_generator("bolt")
        params = {key: spec.default(key) for key in spec.params()}
        if length_mm is not None and "length_mm" in params:
            params["length_mm"] = spec.choose("length_mm", float(length_mm))
        if property_class is not None and "property_class" in params:
            params["property_class"] = spec.choose("property_class", property_class)
        figures = {key: spec.dimension_mm("screw", key) for key in ("thread", "pitch", "head_dk", "head_k", "drive_s")}
        missing = [key for key, value in figures.items() if value is None]
        if missing:
            raise ValueError(f"{spec.id}: the screw component needs dimensions_mm {missing}")
        klass = str(params.get("property_class", property_class or "8.8"))
        specs = spec.specs()
        ref_key = f"torque_ref_{klass.replace('.', '')}_nm"
        torque_ref = specs.get(ref_key, specs.get("torque_ref_nm"))
        return cls(
            thread_mm=float(figures["thread"]), pitch_mm=float(figures["pitch"]),
            length_mm=float(params.get("length_mm", length_mm or 0.0)),
            head_dk_mm=float(figures["head_dk"]), head_k_mm=float(figures["head_k"]),
            drive_s_mm=float(figures["drive_s"]),
            head_standard=str(specs.get("head_standard", "ISO 4762")), property_class=klass,
            model=spec.part_number("screw", **params), mass_kg=spec.mass_kg("screw", **params),
            torque_ref_nm=float(torque_ref) if torque_ref is not None else None,
            catalog=spec.catalog_ref, source=ref,
        )


def iso4762(thread_mm: int, length_mm: float, property_class: str = "8.8",
            model: Optional[str] = None) -> Fastener:
    """An ISO 4762 (DIN 912) hexagon socket head cap screw of the tabled
    sizes (M3–M12), `length_mm` under the head. The mass is the steel of
    its envelope; the reference torque is the class's at μ = 0.14 where
    the table has the class."""
    thread = int(thread_mm)
    if thread not in ISO_4762:
        raise ValueError(f"iso4762: M{thread_mm} is not tabled — one of {sorted(ISO_4762)}")
    if not math.isfinite(length_mm) or length_mm <= 0:
        raise ValueError("iso4762: length_mm must be positive")
    pitch, dk, k, s = ISO_4762[thread]
    shank = math.pi * (thread / 2000.0) ** 2 * (length_mm / 1000.0)
    head = math.pi * (dk / 2000.0) ** 2 * (k / 1000.0) * 0.8   # less the socket
    return Fastener(
        thread_mm=float(thread), pitch_mm=pitch, length_mm=float(length_mm),
        head_dk_mm=dk, head_k_mm=k, drive_s_mm=s, head_standard="ISO 4762",
        property_class=str(property_class), model=model,
        mass_kg=round((shank + head) * _STEEL_KG_PER_M3, 5),
        torque_ref_nm=TORQUE_REF_NM.get(str(property_class), {}).get(thread),
    )


# --------------------------------------------------------------- patterns
@dataclass(frozen=True)
class Hole:
    """A hole of a bolt pattern at `xy_mm` on the mating plane: `threaded`
    (with a usable `depth_mm` from the plane and an unthreaded
    `thread_start_mm`) or `clearance` (through a part `grip_mm` thick
    under the head — the part's thickness when omitted).
    `tolerance_mm` is the position a placed screw is judged against."""

    id: str
    xy_mm: Point2
    kind: str = "threaded"
    diameter_mm: Optional[float] = None
    depth_mm: Optional[float] = None
    thread_start_mm: float = 0.0
    grip_mm: Optional[float] = None
    tolerance_mm: float = 0.2

    def __post_init__(self) -> None:
        if self.kind not in ("threaded", "clearance"):
            raise ValueError(f"Hole {self.id!r}: kind must be 'threaded' or 'clearance'")
        if self.kind == "clearance" and self.thread_start_mm:
            raise ValueError(f"Hole {self.id!r}: a clearance hole has no thread start")


@dataclass(frozen=True)
class Locator:
    """A locating feature at `xy_mm` on the mating plane: a `pin` or
    `boss` standing `depth_mm` proud of one part, a `hole` or `recess`
    that deep in the other."""

    id: str
    xy_mm: Point2
    kind: str = "pin"
    diameter_mm: Optional[float] = None
    depth_mm: Optional[float] = None

    def __post_init__(self) -> None:
        if self.kind not in ("pin", "boss", "hole", "recess"):
            raise ValueError(f"Locator {self.id!r}: kind must be pin, boss, hole or recess")


@dataclass
class BoltPattern:
    """The holes and locators one part shows on a mating plane, in the
    plane's frame (millimetres, +Z out of the part)."""

    holes: list[Hole]
    locators: list[Locator] = field(default_factory=list)

    def __post_init__(self) -> None:
        ids = [h.id for h in self.holes] + [l.id for l in self.locators]
        if len(set(ids)) != len(ids):
            raise ValueError("BoltPattern: hole and locator ids must be distinct")
        if not self.holes:
            raise ValueError("BoltPattern: at least one hole")

    def hole(self, hole_id: str) -> Hole:
        for h in self.holes:
            if h.id == hole_id:
                return h
        raise KeyError(f"no hole {hole_id!r} — one of {[h.id for h in self.holes]}")


def star_order(holes: Union[Sequence[Hole], Mapping[str, Point2]]) -> list[str]:
    """The tightening order a flange is brought down flat by: opposite
    pairs through the pattern's centroid, the innermost pair first (the
    middle of a rectangular cover), then the pairs spread around the
    circle rather than walked round it — 1, 5, 3, 7, 2, 6, 4, 8 on a ring
    of eight, the star pattern flange assembly guidance (ASME PCC-1)
    tightens in. Circular passes at rising torque are a matter of
    repeating the sequence."""
    points: dict[str, Point2] = (
        {h.id: h.xy_mm for h in holes} if not isinstance(holes, Mapping) else dict(holes)
    )
    if not points:
        return []
    ids = list(points)
    cx = sum(points[i][0] for i in ids) / len(ids)
    cy = sum(points[i][1] for i in ids) / len(ids)

    def angle(i: str) -> float:
        return math.atan2(points[i][1] - cy, points[i][0] - cx)

    def radius(i: str) -> float:
        return math.hypot(points[i][0] - cx, points[i][1] - cy)

    by_angle = sorted(ids, key=lambda i: (angle(i), i))
    unpaired = list(by_angle)
    pairs: list[tuple[str, ...]] = []
    while unpaired:
        first = unpaired.pop(0)
        if not unpaired:
            pairs.append((first,))
            break
        want = angle(first) + math.pi
        # The most nearly opposite of what is left.
        mate = min(unpaired, key=lambda j: (abs(math.remainder(angle(j) - want, 2 * math.pi)), j))
        unpaired.remove(mate)
        pairs.append((first, mate))
    # Inner pairs first; equal radii keep their angular order, then are
    # spread: 0, mid, quarter, three-quarter … (bit-reversal), so a ring
    # is tightened in a star and a rectangle from the middle out.
    pairs.sort(key=lambda p: (round(sum(radius(i) for i in p) / len(p), 3), angle(p[0])))
    spread: list[tuple[str, ...]] = []
    groups: dict[float, list[tuple[str, ...]]] = {}
    for p in pairs:
        groups.setdefault(round(sum(radius(i) for i in p) / len(p), 3), []).append(p)
    for _r, group in sorted(groups.items()):
        spread.extend(_bit_reversed(group))
    return [i for p in spread for i in p]


def _bit_reversed(items: list) -> list:
    """`items` in van der Corput order: first, middle, the quarters…"""
    n = len(items)
    if n <= 2:
        return list(items)
    bits = max(1, (n - 1).bit_length())
    order = sorted(range(n), key=lambda k: (int(f"{k:0{bits}b}"[::-1], 2), k))
    return [items[k] for k in order]


# ------------------------------------------------------------------ joint
@dataclass
class Joint:
    """A bolted joint the cell makes: part `a` (the threaded side) and part
    `b` (the clearance side) with their patterns, the screw, the joint's
    torque range and required engagement, the pose of the mating plane
    (`seat`: its centre, +Z out of `a`, where `b`'s underside lands),
    `b`'s thickness, and what `joint()` derived — the tightening `order`
    and one frame per hole (`frames[id]` = `<name>/hole/<id>`, on `b`'s
    top face, +Z out, where a screw's head seats)."""

    name: str
    a: str
    b: str
    pattern_a: BoltPattern
    pattern_b: BoltPattern
    fastener: Fastener
    torque_nm: tuple[float, float]
    seat: Pose
    thickness_m: float
    min_engagement_mm: Optional[float] = None
    washer_mm: float = 0.0
    order: list[str] = field(default_factory=list)
    frames: dict[str, str] = field(default_factory=dict)
    poses: dict[str, Pose] = field(default_factory=dict)
    seat_frame: str = ""
    #: The workpiece pack the joint was read from, if any.
    catalog: Optional[tuple] = None

    def hole_frame(self, hole_id: str) -> str:
        return self.frames[hole_id]

    def hole_pose(self, hole_id: str) -> Pose:
        return self.poses[hole_id]

    def grip_mm(self, hole_id: str) -> float:
        """Under the head to the mating plane: the clearance hole's grip,
        else `b`'s thickness."""
        grip = self.pattern_b.hole(hole_id).grip_mm
        return float(grip) if grip is not None else self.thickness_m * 1000.0

    def engagement_mm(self, hole_id: str) -> float:
        """Thread engaged in `a`: length − grip − washer − thread start."""
        return (self.fastener.length_mm - self.grip_mm(hole_id) - self.washer_mm
                - self.pattern_a.hole(hole_id).thread_start_mm)

    def tip_depth_mm(self, hole_id: str) -> float:
        """How deep the tip goes into `a` from the mating plane."""
        return self.fastener.length_mm - self.grip_mm(hole_id) - self.washer_mm

    def check(self, driver: Optional[Driver] = None) -> list[Finding]:
        """What the drawing and the tool say against each other — see
        `check()`."""
        return check(self, driver)


def joint(
    scene,
    name: str,
    *,
    a: str,
    b: str,
    seat: Pose,
    pattern_a: Optional[BoltPattern] = None,
    pattern_b: Optional[BoltPattern] = None,
    fastener: Optional[Fastener] = None,
    thickness: Optional[float] = None,
    torque_nm: Union[float, tuple[float, float], None] = None,
    min_engagement_mm: Optional[float] = None,
    washer_mm: float = 0.0,
    order: Union[str, Sequence[str]] = "star",
    strict: bool = True,
    catalog=None,
) -> Joint:
    """Declares the bolted joint between obstacles `a` (threaded holes)
    and `b` (clearance holes, `thickness` metres thick) and puts its
    frames in the scene: `<name>/seat` at the mating plane's centre
    (`seat` = its world pose, +Z out of `a` — where `b`'s underside
    lands), `<name>/hole/<id>` on `b`'s top face over each hole (+Z out,
    the head's seat — a screw is driven by aiming the tool tip's +Z
    along it) and `<name>/locator/<id>` on the plane. The patterns must
    agree hole for hole (same ids, same positions).

    `torque_nm` is the joint's design value (a range, or one figure);
    `min_engagement_mm` the thread engagement it needs (a drawing's
    figure — 2×d in aluminium is the usual rule of thumb, but the joint
    states it, botrail does not guess). `order` is `"star"` (derived) or
    the hole ids in the order to tighten. With `strict` (the default) a
    joint the screw cannot make — too little engagement, a tip through
    the hole's bottom — is refused here rather than found on the line.

    With `catalog=` — a workpiece spec pack's id or package directory
    (what `bt.parts.workpiece` stood the parts from) — the patterns, the
    screw, the torque range, the engagement and the thickness are read
    from the pack's `mounting`: its `flange` face is `a`'s threaded holes
    and locators, its `mount` face `b`'s clearance holes, its `fasteners`
    the screw and the torque; nothing is typed in. Arguments given
    alongside override what the pack states."""
    if a not in scene.obstacle_names or b not in scene.obstacle_names:
        raise ValueError(f"joint: {a!r} and {b!r} must be obstacles in the scene")
    catalog_ref = None
    if catalog is not None:
        read = _joint_from_pack(catalog)
        pattern_a = pattern_a or read["pattern_a"]
        pattern_b = pattern_b or read["pattern_b"]
        fastener = fastener or read["fastener"]
        thickness = thickness if thickness is not None else read["thickness"]
        torque_nm = torque_nm if torque_nm is not None else read["torque_nm"]
        min_engagement_mm = min_engagement_mm if min_engagement_mm is not None else read["min_engagement_mm"]
        catalog_ref = read["catalog"]
    if pattern_a is None or pattern_b is None or fastener is None or thickness is None or torque_nm is None:
        raise ValueError("joint: pattern_a, pattern_b, fastener, thickness and torque_nm are needed (or a catalog= that states them)")
    if thickness <= 0:
        raise ValueError("joint: thickness must be positive")
    lo, hi = (float(torque_nm), float(torque_nm)) if isinstance(torque_nm, (int, float)) else (
        float(torque_nm[0]), float(torque_nm[1]))
    if not 0 < lo <= hi:
        raise ValueError("joint: torque_nm must be positive, min <= max")
    ids_a = [h.id for h in pattern_a.holes]
    ids_b = [h.id for h in pattern_b.holes]
    if sorted(ids_a) != sorted(ids_b):
        raise ValueError(f"joint: the patterns disagree — {a} has {ids_a}, {b} has {ids_b}")
    if isinstance(order, str):
        if order != "star":
            raise ValueError("joint: order is 'star' or the hole ids in sequence")
        sequence = star_order(pattern_a.holes)
    else:
        sequence = [str(i) for i in order]
        if sorted(sequence) != sorted(ids_a):
            raise ValueError(f"joint: order must name every hole once — {ids_a}")
    (sx, sy, sz), sq = seat
    made = Joint(
        name=name, a=a, b=b, pattern_a=pattern_a, pattern_b=pattern_b, fastener=fastener,
        torque_nm=(lo, hi), seat=((float(sx), float(sy), float(sz)), tuple(float(v) for v in sq)),
        thickness_m=float(thickness), min_engagement_mm=min_engagement_mm, washer_mm=float(washer_mm),
        order=sequence, catalog=catalog_ref,
    )
    findings = check(made)
    errors = [f for f in findings if f.severity == "error"]
    if strict and errors:
        raise ValueError(f"joint {name!r} cannot be made: " + "; ".join(f.message for f in errors))
    scene.add_frame(f"{name}/seat", position=made.seat[0], quaternion=made.seat[1])
    made.seat_frame = f"{name}/seat"
    for hole in pattern_a.holes:
        hx, hy = hole.xy_mm[0] / 1000.0, hole.xy_mm[1] / 1000.0
        dx, dy, dz = _rotate(made.seat[1], (hx, hy, made.thickness_m))
        pose = ((made.seat[0][0] + dx, made.seat[0][1] + dy, made.seat[0][2] + dz), made.seat[1])
        frame = f"{name}/hole/{hole.id}"
        scene.add_frame(frame, position=pose[0], quaternion=pose[1])
        made.frames[hole.id] = frame
        made.poses[hole.id] = pose
    for loc in pattern_a.locators:
        lx, ly = loc.xy_mm[0] / 1000.0, loc.xy_mm[1] / 1000.0
        dx, dy, dz = _rotate(made.seat[1], (lx, ly, 0.0))
        scene.add_frame(f"{name}/locator/{loc.id}",
                        position=(made.seat[0][0] + dx, made.seat[0][1] + dy, made.seat[0][2] + dz),
                        quaternion=made.seat[1])
    return made


def _range(value, pick: str = "max") -> Optional[float]:
    """A pack's `{min, max}` (or a plain number) as one figure."""
    if value is None:
        return None
    if isinstance(value, dict):
        value = value.get(pick, value.get("max", value.get("min")))
    return None if value is None else float(value)


def _joint_from_pack(catalog) -> dict:
    """What a workpiece pack's `mounting` states about its bolted joint, as
    the arguments `joint()` takes: the `flange` face's holes and locators
    (threaded side, `a`), the `mount` face's (clearance side, `b`), the
    first `fasteners` entry as the screw with its torque, the flange
    holes' `fastener_rules.min_engagement_mm`, the mount holes' grip as
    the thickness."""
    from ._spec import Spec

    spec = Spec.load(catalog)
    mounting = spec.mounting() or {}
    faces = {i.get("role"): i for i in mounting.get("interfaces", []) if isinstance(i, dict)}
    if "flange" not in faces or "mount" not in faces:
        raise ValueError(f"{spec.id}: a bolted joint needs a `flange` (threaded) and a `mount` (clearance) face in mounting.interfaces")

    def pattern(face: dict) -> BoltPattern:
        geometry = face.get("geometry") or {}
        holes = []
        for h in geometry.get("holes", []):
            rules = h.get("fastener_rules") or {}
            holes.append(Hole(
                str(h["id"]), (float(h["position_mm"][0]), float(h["position_mm"][1])),
                str(h.get("kind", "threaded")), diameter_mm=_range(h.get("diameter_mm")),
                depth_mm=_range(h.get("depth_mm"), "min"), thread_start_mm=_range(h.get("thread_start_mm")) or 0.0,
                grip_mm=_range(h.get("grip_mm")),
                tolerance_mm=float(h.get("position_tolerance_mm") or 0.2),
            ))
        locators = [
            Locator(str(l["id"]), (float(l["position_mm"][0]), float(l["position_mm"][1])),
                    str(l.get("kind", "pin")), diameter_mm=_range(l.get("diameter_mm"), "min"),
                    depth_mm=_range(l.get("depth_mm"), "min"))
            for l in geometry.get("locators", [])
        ]
        return BoltPattern(holes, locators)

    flange, mount = faces["flange"], faces["mount"]
    pattern_a, pattern_b = pattern(flange), pattern(mount)
    rules = [h.get("fastener_rules") or {} for h in (flange.get("geometry") or {}).get("holes", [])]
    engagements = [float(r["min_engagement_mm"]) for r in rules if r.get("min_engagement_mm") is not None]
    fasteners = mount.get("fasteners") or []
    if not fasteners:
        raise ValueError(f"{spec.id}: the mount face declares no `fasteners` — the screw the joint takes")
    first = fasteners[0]
    thread = first.get("thread") or {}
    diameter = float(thread.get("diameter_mm"))
    length = _range(first.get("length_mm"), "min")
    klass = str(first.get("property_class", "8.8"))
    head = str(first.get("head_standard", "ISO 4762"))
    if "4762" in head or "912" in head:
        fastener = iso4762(int(round(diameter)), length, klass)
    else:
        pitch, dk, k, s = ISO_4762.get(int(round(diameter)), (1.0, 2 * diameter, diameter, diameter * 0.8))
        fastener = Fastener(diameter, float(thread.get("pitch_mm") or pitch), length, dk, k, s,
                            head_standard=head, property_class=klass)
    torque = first.get("torque_nm")
    torque_nm = (float(torque["min"]), float(torque["max"])) if isinstance(torque, dict) else (
        (float(torque), float(torque)) if torque is not None else None)
    grips = [_range(h.get("grip_mm")) for h in (mount.get("geometry") or {}).get("holes", [])]
    grips = [g for g in grips if g is not None]
    return {
        "pattern_a": pattern_a, "pattern_b": pattern_b, "fastener": fastener,
        "torque_nm": torque_nm, "min_engagement_mm": max(engagements) if engagements else None,
        "thickness": max(grips) / 1000.0 if grips else None, "catalog": spec.catalog_ref,
    }


def check(joint: Joint, driver: Optional[Driver] = None) -> list[Finding]:
    """The joint against its screw, and — given the `driver` — against the
    tool: `hole_mismatch` (a hole in one part not over its mate),
    `hole_kinds` (a clearance hole where a thread should be),
    `engagement_short` (thread engaged under what the joint needs),
    `hole_bottom` (the tip through the usable depth), `torque_over_table`
    (the joint's torque above the class's reference), `torque_off_joint`
    (the driver set outside the joint's range), `torque_over_tool` (the
    joint's torque outside what the tool delivers), `bit_mismatch`,
    `screw_too_long` and `stroke_short`. Errors are what makes a joint
    impossible as drawn; warnings are what a reviewer would flag."""
    out: list[Finding] = []
    f = joint.fastener
    for hole in joint.pattern_a.holes:
        try:
            mate = joint.pattern_b.hole(hole.id)
        except KeyError:
            out.append(Finding("error", "hole_mismatch", f"{joint.name}: {joint.b} has no hole {hole.id!r}", joint.name))
            continue
        dist = math.hypot(hole.xy_mm[0] - mate.xy_mm[0], hole.xy_mm[1] - mate.xy_mm[1])
        if dist > 0.5:
            out.append(Finding("error", "hole_mismatch",
                               f"{joint.name}: hole {hole.id} is {dist:.1f} mm off between {joint.a} and {joint.b}", joint.name))
        if hole.kind != "threaded" or mate.kind != "clearance":
            out.append(Finding("warning", "hole_kinds",
                               f"{joint.name}: hole {hole.id} is {hole.kind} in {joint.a} and {mate.kind} in {joint.b}; "
                               "a screw joint threads into a and passes through b", joint.name))
        engagement = joint.engagement_mm(hole.id)
        if joint.min_engagement_mm is not None and engagement < joint.min_engagement_mm - 1e-9:
            out.append(Finding("error", "engagement_short",
                               f"{joint.name}: hole {hole.id} engages {engagement:.1f} mm of thread, "
                               f"under the {joint.min_engagement_mm:.1f} mm the joint needs "
                               f"(M{_plain(f.thread_mm)}x{_plain(f.length_mm)} through {joint.grip_mm(hole.id):.1f} mm)", joint.name))
        if hole.depth_mm is not None and joint.tip_depth_mm(hole.id) > hole.depth_mm + 1e-9:
            out.append(Finding("error", "hole_bottom",
                               f"{joint.name}: hole {hole.id} takes {hole.depth_mm:.1f} mm of tip, "
                               f"the screw puts {joint.tip_depth_mm(hole.id):.1f} mm in", joint.name))
    if f.torque_ref_nm is not None and joint.torque_nm[1] > f.torque_ref_nm + 1e-9:
        out.append(Finding("warning", "torque_over_table",
                           f"{joint.name}: {joint.torque_nm[1]:.2f} N·m is above the {f.torque_ref_nm:.2f} N·m "
                           f"reference for M{_plain(f.thread_mm)} class {f.property_class} (μ = 0.14)", joint.name))
    if driver is not None:
        if not joint.torque_nm[0] - 1e-9 <= driver.torque_nm <= joint.torque_nm[1] + 1e-9:
            out.append(Finding("error", "torque_off_joint",
                               f"{driver.name}: set to {driver.torque_nm:.2f} N·m, the joint {joint.name} takes "
                               f"{joint.torque_nm[0]:.2f}–{joint.torque_nm[1]:.2f}", driver.name))
        tool = driver.tool
        rng = tool.get("torque_nm")
        if rng is not None:
            tlo, thi = float(rng[0]), float(rng[1])
            if joint.torque_nm[0] < tlo - 1e-9 or joint.torque_nm[1] > thi + 1e-9:
                out.append(Finding("error", "torque_over_tool",
                                   f"{driver.name}: delivers {tlo:.2f}–{thi:.2f} N·m, the joint {joint.name} takes "
                                   f"{joint.torque_nm[0]:.2f}–{joint.torque_nm[1]:.2f}", driver.name))
        bit = tool.get("bit_mm")
        if bit is not None and abs(float(bit) - f.drive_s_mm) > 1e-6:
            out.append(Finding("error", "bit_mismatch",
                               f"{driver.name}: a {float(bit):g} mm bit on a screw whose drive is {f.drive_s_mm:g} mm", driver.name))
        longest = tool.get("screw_length_mm")
        if longest is not None and f.length_mm > float(longest) + 1e-9:
            out.append(Finding("error", "screw_too_long",
                               f"{driver.name}: takes screws to {float(longest):g} mm, this one is {_plain(f.length_mm)} mm", driver.name))
        if driver.stroke_m is not None and driver.run_m > driver.stroke_m + 1e-9:
            out.append(Finding("error", "stroke_short",
                               f"{driver.name}: a {driver.stroke_m * 1e3:.0f} mm stroke cannot run a screw {driver.run_m * 1e3:.0f} mm down", driver.name))
    return out


# ----------------------------------------------------------------- driver
def rundown_s(length_m: float, pitch_mm: float, rpm: float) -> float:
    """Seconds to run `length_m` of thread down at `rpm`: the feed is the
    pitch per turn."""
    if length_m < 0 or pitch_mm <= 0 or rpm <= 0:
        raise ValueError("rundown_s: positive length, pitch and rpm")
    return length_m / (pitch_mm / 1000.0 * rpm / 60.0)


@dataclass
class Driver:
    """What `driver` authored: the screwdriver's program (`program`, on the
    I/O node `node`), the links it drives with (`bit`, the `shank` joint
    and its `stroke_m`), the screw and torque it is set up for, its
    phase timing, and its signals by role — `signal("start")` is the
    output the robot's program raises, `signal("ok")` / `signal("nok")`
    the inputs it waits on. `tool` is what the tool's datasheet says
    (`torque_nm` range, `screw_length_mm`, `bit_mm`) for `check`."""

    name: str
    program: str
    robot: str
    bit: str
    fastener: Fastener
    torque_nm: float
    node: Optional[str] = None
    shank: Optional[str] = None
    stroke_m: Optional[float] = None
    engage_m: float = 0.003
    rpm_run: float = 340.0
    rpm_final: float = 40.0
    t_find_s: float = 0.2
    t_pre_s: float = 0.2
    final_turn: float = 1.0 / 3.0
    cycles: int = 1
    signals: dict[str, str] = field(default_factory=dict)
    tool: dict = field(default_factory=dict)

    def signal(self, role: str) -> str:
        if role not in self.signals:
            raise KeyError(f"{self.name}: no signal for the role {role!r} — one of {sorted(self.signals)}")
        return self.signals[role]

    @property
    def run_m(self) -> float:
        """The stroke one screw runs down: its length less the depth the
        engage move already put its tip into the hole."""
        return self.fastener.length_m - self.engage_m

    @property
    def t_run_s(self) -> float:
        return rundown_s(self.run_m, self.fastener.pitch_mm, self.rpm_run)

    @property
    def t_final_s(self) -> float:
        return self.final_turn / (self.rpm_final / 60.0)

    @property
    def t_cycle_s(self) -> float:
        """Find, run, pre-tighten, final: the driver's time per screw."""
        return self.t_find_s + self.t_run_s + self.t_pre_s + self.t_final_s

    def phases(self) -> dict[str, float]:
        return {"find_s": self.t_find_s, "run_s": self.t_run_s, "pre_s": self.t_pre_s, "final_s": self.t_final_s}


def driver(
    scene,
    name: str,
    *,
    robot: str,
    bit: str,
    fastener: Fastener,
    torque_nm: float,
    cycles: int,
    shank: Optional[str] = None,
    stroke: Optional[float] = None,
    engage: float = 0.003,
    rpm_run: float = 340.0,
    rpm_final: float = 40.0,
    t_find_s: float = 0.2,
    t_pre_s: float = 0.2,
    final_turn: float = 1.0 / 3.0,
    program: Optional[str] = None,
    node: bool = True,
    estop: Optional[str] = None,
    finish: Optional[str] = None,
    tool: Optional[Mapping[str, object]] = None,
    channels: Optional[list] = None,
) -> Driver:
    """The screwdriver's controller as a program of its own, scanned
    beside the robot's — the way a tightening controller runs: on
    `<name>/start` it *finds* the drive (`t_find_s`), *runs* the screw
    down at `rpm_run` for as long as the thread takes (the length less
    `engage`, the depth the engage move already put the tip into the hole,
    at `pitch × rpm`), *pre-tightens* (`t_pre_s`), *final-tightens*
    a `final_turn` at `rpm_final`, then answers `<name>/ok` or
    `<name>/nok` and holds `<name>/busy` until the start drops. `<name>/run`
    is on while the bit turns (what a spin effect binds to). The result
    is OK unless a scenario says otherwise: `<name>/fault_first` fails the
    next run, `<name>/fault_retry` the one after — the two faults a
    fastening's retry branch is tested with.

    `cycles` is how many starts the program can serve (screws plus the
    retries allowed): the program is that many rundowns nested so it ends
    on `<name>/finish` (or `finish`, a signal the robot's program raises
    when it is done) from whichever cycle it is waiting in. With `estop`
    (an input lane) no run starts while it is pressed. `shank` and
    `stroke` name the tool's own feed joint and its travel, for the
    ramp `fasten` runs the screw down with; `tool` carries the datasheet
    figures `check` compares against (`torque_nm=(min, max)`,
    `screw_length_mm`, `bit_mm`). `node` (default) hosts the program on
    the I/O node `<name>/controller`, so the handshake lands on the I/O
    list as wires between the two controllers; `channels`
    (`bt.io.channels`) are that controller's terminals, what
    `Scene.auto_assign_io` gives the wires addresses on."""
    if cycles < 1:
        raise ValueError("driver: at least one cycle")
    if torque_nm <= 0 or rpm_run <= 0 or rpm_final <= 0 or engage < 0:
        raise ValueError("driver: positive torque and rpms, a non-negative engage depth")
    if engage >= fastener.length_m:
        raise ValueError("driver: the engage depth must be less than the screw's length")
    if (shank is None) != (stroke is None):
        raise ValueError("driver: give shank and stroke together (the tool's own feed), or neither")
    program = program or name
    roles = {r: f"{name}/{r}" for r in ("start", "run", "ok", "nok", "busy", "fault_first", "fault_retry")}
    roles["finish"] = finish or f"{name}/finish"
    if estop is not None:
        roles["estop"] = estop
    existing = {s for s, _ in scene.signals}
    for role, signal in roles.items():
        if role != "estop" and signal not in existing:
            scene.define_signal(signal)
    made = Driver(
        name=name, program=program, robot=robot, bit=bit, fastener=fastener, torque_nm=float(torque_nm),
        shank=shank, stroke_m=stroke, engage_m=float(engage), rpm_run=float(rpm_run), rpm_final=float(rpm_final),
        t_find_s=float(t_find_s), t_pre_s=float(t_pre_s), final_turn=float(final_turn), cycles=int(cycles),
        signals=roles, tool=dict(tool or {}),
    )
    S = seq
    start, run, ok, nok, busy = (roles[r] for r in ("start", "run", "ok", "nok", "busy"))
    fault_first, fault_retry, fin = roles["fault_first"], roles["fault_retry"], roles["finish"]
    admit = S.signal(start) if estop is None else S.all_of(S.signal(start), S.signal(estop, False))

    def cycle(builder, k: int) -> None:
        builder.step(f"idle{k}", transition=S.any_of(S.signal(start), S.signal(fin)))
        gate = builder.select(f"gate{k}")
        arm = gate.when(admit)
        arm.step(f"find{k}", actions=[S.set_signal(run), S.set_signal(busy)], transition=S.elapsed(made.t_find_s))
        arm.step(f"run{k}", transition=S.elapsed(made.t_run_s))
        arm.step(f"pre{k}", transition=S.elapsed(made.t_pre_s))
        arm.step(f"final{k}", transition=S.elapsed(made.t_final_s))
        judge = arm.select(f"judge{k}")
        judge.when(S.signal(fault_first)).step(
            f"nok_first{k}", actions=[S.set_signal(nok), S.set_signal(fault_first, False)])
        judge.when(S.signal(fault_retry)).step(
            f"nok_retry{k}", actions=[S.set_signal(nok), S.set_signal(fault_retry, False)])
        judge.when(S.otherwise()).step(f"ok{k}", actions=[S.set_signal(ok)])
        # `busy` drops only once the start has: the robot's "wait for not
        # busy" then means the handshake is over, not that the answer is in.
        arm.step(f"done{k}", actions=[S.set_signal(run, False)], transition=S.signal(start, False))
        arm.step(f"clear{k}", actions=[S.set_signal(busy, False), S.set_signal(ok, False), S.set_signal(nok, False)])
        if k + 1 < cycles:
            cycle(arm, k + 1)
        gate.when(S.signal(fin))

    cycle(scene.sequence(program), 0)
    if node:
        made.node = f"{name}/controller"
        scene.add_io_node(made.node, kind="plc", programs=[program], channels=channels,
                          label=f"{name} screwdriver controller")
    return made


# ---------------------------------------------------------------- program
@dataclass
class Placement:
    """What `place` appended: the part carried, its expected seat, the
    step names (`steps["release"]` is where it was let go) and the
    `seated` signal the release raises — what a fastening of the part
    waits on."""

    part: str
    expected: Pose
    steps: dict[str, str] = field(default_factory=dict)
    seated: str = ""


@dataclass
class Fastening:
    """What `fasten` appended: which screw went to which hole, in order
    (`pairs`), the driver and the feeder it used, the joint, the halted
    lane, and the step names per screw (`steps[k]["run"]`…)."""

    joint: Joint
    driver: Driver
    feeder: parts.ScrewFeeder
    pairs: list[tuple[str, str]]
    halted: str
    retry: int
    steps: list[dict[str, str]] = field(default_factory=list)


def _pin(scene, name: str, **more) -> None:
    """Adds attributes to a pinned part without losing the rest of its
    identity (re-pinning replaces)."""
    part = scene.part(name)
    if part is None:
        return
    attributes = dict(part.get("attributes") or {})
    attributes.update(more)
    scene.set_part(
        name, kind=part.get("kind"), category=part.get("category"), catalog=part.get("catalog"),
        manufacturer=part.get("manufacturer"), model=part.get("model"),
        description=part.get("description"), qty=int(part.get("qty") or 1), attributes=attributes,
    )


def _motion(motions: Mapping[str, str], key: str, **fmt) -> str:
    if key not in motions:
        raise ValueError(f"motions needs {key!r} — the taught motion's name (with {{i}} / {{hole}} for the screw)")
    return str(motions[key]).format(**fmt)


def place(
    sq,
    part: str,
    *,
    motions: Mapping[str, str],
    close: Mapping[str, float],
    open: Mapping[str, float],
    expected: Pose,
    touch: Sequence[str] = (),
    grip_s: float = 0.4,
    robot: Optional[str] = None,
    prefix: str = "",
) -> Placement:
    """Appends the fit of `part` to the sequence `sq`: the taught motions
    `approach` (over the part), `down` (onto it), `up`, `carry` (over its
    seat), `seat` (down onto the mate) and `clear`, with the gripper ramped
    to `close` before the grasp and `open` after the release. `expected`
    is the pose the part should stand at when seated — what `fit_report`
    measures the bake against. `touch` names what the carried part is
    meant to meet — its mate, the dowels — and declares that contact
    allowed (`Scene.allow_object_obstacle_contact`), so setting it down
    is not a collision. The release raises the internal signal
    `<sequence>/<part>_seated` (`Placement.seated`): what a fastening of
    the part waits on (`fasten(after=[placement.seated])`) — a row on the
    interlock table, not a convention."""
    S = seq
    scene = sq._scene
    for obstacle in touch:
        scene.allow_object_obstacle_contact(part, obstacle)
    seated = f"{sq.name}/{part.rsplit('/', 1)[-1]}_seated"
    if seated not in {s for s, _ in scene.signals}:
        scene.define_signal(seated)
    steps: dict[str, str] = {}

    def step(key: str, **kwargs) -> None:
        name = f"{prefix}{key}_{part.rsplit('/', 1)[-1]}"
        steps[key] = name
        sq.step(name, **kwargs)

    step("approach", actions=[S.motion(_motion(motions, "approach"))])
    step("down", actions=[S.motion(_motion(motions, "down"))])
    step("grip", actions=[S.ramp(dict(close), grip_s, robot=robot)])
    step("hold", actions=[S.attach(part, touch_links="tool", robot=robot)])
    step("up", actions=[S.motion(_motion(motions, "up"))])
    step("carry", actions=[S.motion(_motion(motions, "carry"))])
    step("seat", actions=[S.motion(_motion(motions, "seat"))])
    step("release", actions=[S.detach(part), S.set_signal(seated), S.ramp(dict(open), grip_s, robot=robot)])
    step("clear", actions=[S.motion(_motion(motions, "clear"))])
    return Placement(part, expected, steps, seated)


def fasten(
    sq,
    joint: Joint,
    driver: Driver,
    feeder: parts.ScrewFeeder,
    *,
    motions: Mapping[str, str],
    retry: int = 1,
    robot: Optional[str] = None,
    prefix: str = "",
    retract_s: float = 0.4,
    tilt_deg: float = 1.0,
    after: Sequence[str] = (),
) -> Fastening:
    """Appends the fastening of `joint` to the sequence `sq`, one screw per
    hole in the joint's order, each from the `feeder`: request a screw
    (`start` the feeder, wait for `present`), the taught `to_pick`,
    `pick` (the bit into the head), attach the screw to the bit, `lift`,
    `over` the hole, `engage` (the tip into the hole by the driver's engage
    depth — motions named with `{i}` for the screw's index and `{hole}` for
    its hole id), then
    raise the driver's `start`; while the driver finds the drive and runs,
    the tool's shank ramps the screw down its length at the same
    thread-derived time, and the step waits for `ok` or `nok`. On OK the
    screw is released and the shank retracted; on NOK the shank backs out
    and the run is repeated up to `retry` times — a last NOK releases the
    screw where it stands, raises `<program>/halted` and stops the
    program there (a refused cycle, the FAT row). A driver without its
    own feed runs the screw down with the taught motion `drive`
    instead.

    Each screw is allowed to meet the joint's two parts only **within its
    hole's window** (`Scene.allow_object_obstacle_contact`): its tip within
    the hole's `tolerance_mm` of the hole's axis, leaning at most
    `tilt_deg` — so the engage move, which is collision-checked, refuses a
    screw taught off its hole, and the clearance measure does not count a
    screw in its hole as contact.

    Two interlocks are written in, rows on the interlock table rather
    than conventions: the first screw is picked only once every signal in
    `after` is on (the part's `seated` from `place`) and a screw is
    present, and no `start` is raised while the driver's E-stop lane is
    pressed (the engage move completes only with it released).

    The driver must have a cycle per possible start: `len(order) ×
    (1 + retry)` at most."""
    scene = sq._scene
    S = seq
    n = len(joint.order)
    if len(feeder.screws) < n:
        raise ValueError(f"fasten: {joint.name} needs {n} screws, the feeder {feeder.name} holds {len(feeder.screws)}")
    if driver.cycles < n:
        raise ValueError(f"fasten: {driver.name} serves {driver.cycles} starts, {joint.name} needs {n} (plus retries)")
    if retry < 0:
        raise ValueError("fasten: retry must be 0 or more")
    if driver.shank is None and "drive" not in motions:
        raise ValueError("fasten: a driver without its own feed needs the taught motion `drive` (the arm running the screw down)")
    halted = f"{sq.name}/halted"
    if halted not in {s for s, _ in scene.signals}:
        scene.define_signal(halted)
    start, ok, nok, busy = (driver.signal(r) for r in ("start", "ok", "nok", "busy"))
    estop = driver.signals.get("estop")
    made = Fastening(joint, driver, feeder, [], halted, retry)
    # The screws now know their joint: its torque goes on their line, and
    # the driver's requirement (`bt.select`) is derived from it.
    for screw in feeder.screws[:n]:
        _pin(scene, screw, torque_nm=joint.torque_nm[1], joint=joint.name)

    def rundown():
        if driver.shank is not None:
            return S.ramp({driver.shank: driver.run_m}, driver.t_run_s, robot=robot)
        return S.motion(_motion(motions, "drive"))

    def back():
        if driver.shank is not None:
            return S.ramp({driver.shank: 0.0}, retract_s, robot=robot)
        return S.motion(_motion(motions, "clear"))

    def one(k: int, hole: str) -> None:
        screw = feeder.screws[k]
        fmt = {"i": k, "hole": hole}
        names: dict[str, str] = {}
        # The screw may be inside the two parts only on its hole's axis.
        (hx, hy, hz), hq = joint.hole_pose(hole)
        window = ((hx, hy, hz), _rotate(hq, (0.0, 0.0, 1.0)),
                  joint.pattern_b.hole(hole).tolerance_mm / 1000.0, math.radians(tilt_deg))
        for part in (joint.b, joint.a):
            scene.allow_object_obstacle_contact(screw, part, window=window)

        def step(builder, key: str, **kwargs) -> None:
            name = f"{prefix}{key}{k}"
            names[key] = name
            builder.step(name, **kwargs)

        ready = [S.signal(feeder.present)] + ([S.signal(s) for s in after] if k == 0 else [])
        step(sq, "feed", actions=[S.start(feeder.device)],
             transition=ready[0] if len(ready) == 1 else S.all_of(*ready))
        step(sq, "to_pick", actions=[S.motion(_motion(motions, "to_pick", **fmt))])
        step(sq, "pick", actions=[S.motion(_motion(motions, "pick", **fmt))])
        step(sq, "take", actions=[S.attach(screw, link=driver.bit, touch_links=[driver.bit], robot=robot)])
        step(sq, "lift", actions=[S.motion(_motion(motions, "lift", **fmt))])
        step(sq, "over", actions=[S.motion(_motion(motions, "over", **fmt))])
        step(sq, "engage", actions=[S.motion(_motion(motions, "engage", **fmt))],
             transition=S.done() if estop is None else S.all_of(S.done(), S.signal(estop, False)))
        step(sq, "start", actions=[S.set_signal(start)], transition=S.elapsed(driver.t_find_s))
        step(sq, "run", actions=[rundown()],
             transition=S.all_of(S.done(), S.any_of(S.signal(ok), S.signal(nok))))

        def outcome(builder, attempt: int) -> None:
            """The judge after a run: OK releases; NOK backs out and runs
            again while a retry is left, else halts."""
            judge = builder.select(f"{prefix}judge{k}" if attempt == 0 else f"{prefix}judge{k}_{attempt}")
            good = judge.when(S.signal(ok))
            tag = "" if attempt == 0 else f"_{attempt}"
            good.step(f"{prefix}release{k}{tag}", actions=[S.detach(screw), S.set_signal(start, False)])
            good.step(f"{prefix}retract{k}{tag}", actions=[back()])
            bad = judge.when(S.signal(nok))
            if attempt < retry:
                bad.step(f"{prefix}back{k}_{attempt + 1}", actions=[S.set_signal(start, False), back()],
                         transition=S.all_of(S.done(), S.signal(busy, False)))
                bad.step(f"{prefix}restart{k}_{attempt + 1}", actions=[S.set_signal(start)],
                         transition=S.elapsed(driver.t_find_s))
                bad.step(f"{prefix}rerun{k}_{attempt + 1}", actions=[rundown()],
                         transition=S.all_of(S.done(), S.any_of(S.signal(ok), S.signal(nok))))
                outcome(bad, attempt + 1)
            else:
                # Released where it stands, and the program stops here: the
                # arms of a branch rejoin holding the same things, and a
                # halted cell holds nothing.
                bad.step(f"{prefix}halt{k}", actions=[S.set_signal(start, False), S.detach(screw), S.set_signal(halted)],
                         transition=S.signal(halted, False))

        outcome(sq, 0)
        names["release"] = f"{prefix}release{k}"
        names["halt"] = f"{prefix}halt{k}"
        step(sq, "clear", actions=[S.motion(_motion(motions, "clear", **fmt))])
        made.pairs.append((screw, hole))
        made.steps.append(names)

    for k, hole in enumerate(joint.order):
        one(k, hole)
    return made


# ---------------------------------------------------------------- reports
def _rotate(q: Quat, v: Point3) -> Point3:
    qx, qy, qz, qw = q
    vx, vy, vz = v
    tx, ty, tz = 2 * (qy * vz - qz * vy), 2 * (qz * vx - qx * vz), 2 * (qx * vy - qy * vx)
    return (vx + qw * tx + (qy * tz - qz * ty), vy + qw * ty + (qz * tx - qx * tz), vz + qw * tz + (qx * ty - qy * tx))


def _plain(value: float):
    return round(value) if abs(value - round(value)) < 1e-6 else value


def _span(tl, sequence: str, step: str):
    """A step's span, or `None` when the bake never reached it. A bake of
    several programs prefixes step names with the program's; a bake of
    one does not."""
    try:
        return tl.step_span(f"{sequence}/{step}" if sequence else step)
    except (KeyError, ValueError):
        return None


def _sequence_of(tl, first: str) -> str:
    """The prefix the bake put on the step `first` — the program's name,
    or nothing for a bake of one program."""
    for name, _t0, _t1 in tl.step_spans:
        seq_name, _, step = name.rpartition("/")
        if step == first:
            return seq_name
    raise ValueError(f"the timeline has no step {first!r}: was this program baked?")


def fastening_report(tl, fastening: Fastening, *, tilt_deg: float = 1.0) -> list[dict]:
    """One row per screw of a baked fastening, facts then checks: `hole`,
    `screw`, `order`, `t_engage` / `t_done`, the screw's `offset_mm` from
    the hole's axis and `tilt_deg` when it engaged, `seated_mm` (how far
    it went down, against the run the driver was set for), `rundown_s`,
    `attempts` and `result` (`ok`, `retry` — OK after a NOK — or `nok`),
    the data-derived `engagement_mm` and `tip_mm`, the `torque_nm`, and
    `checks` — `align`, `tilt`, `seated`, `engagement`, `depth`, `torque`
    — each `pass` / `fail` / `n/a` (`grasp_report`'s form)."""
    joint, driver = fastening.joint, fastening.driver
    program = _sequence_of(tl, fastening.steps[0]["feed"])
    ok_lane, nok_lane = tl.signal(driver.signal("ok")), tl.signal(driver.signal("nok"))
    rows: list[dict] = []
    for k, ((screw, hole), names) in enumerate(zip(fastening.pairs, fastening.steps)):
        (fx, fy, fz), fq = joint.hole_pose(hole)
        normal = _rotate(fq, (0.0, 0.0, 1.0))
        engage = _span(tl, program, names["engage"])
        run = _span(tl, program, names["run"])
        clear = _span(tl, program, names["clear"])
        halt = _span(tl, program, names["halt"])
        row: dict = {"order": k + 1, "hole": hole, "screw": screw, "t_engage": None, "t_done": None,
                     "offset_mm": None, "tilt_deg": None, "seated_mm": None, "drive_s": None,
                     "attempts": 0, "result": "not run",
                     "engagement_mm": round(joint.engagement_mm(hole), 2),
                     "tip_mm": round(joint.tip_depth_mm(hole), 2),
                     "torque_nm": driver.torque_nm}
        checks: dict[str, str] = {}
        if engage is not None:
            t_engage = engage.end
            (px, py, pz), pq = tl.object_pose(screw, t_engage)
            axis = _rotate(pq, (0.0, 0.0, 1.0))
            d = (px - fx, py - fy, pz - fz)
            along = sum(a * b for a, b in zip(d, normal))
            lateral = math.sqrt(max(0.0, sum(a * a for a in d) - along * along))
            cos = max(-1.0, min(1.0, sum(a * b for a, b in zip(axis, normal))))
            row["t_engage"] = round(t_engage, 3)
            row["offset_mm"] = round(lateral * 1e3, 3)
            row["tilt_deg"] = round(math.degrees(math.acos(cos)), 3)
            tolerance = joint.pattern_b.hole(hole).tolerance_mm
            checks["align"] = "pass" if lateral * 1e3 <= tolerance + 1e-9 else "fail"
            checks["tilt"] = "pass" if row["tilt_deg"] <= tilt_deg else "fail"
            end = clear.start if clear is not None else (halt.start if halt is not None else tl.duration)
            row["t_done"] = round(end, 3)
            (qx, qy, qz), _ = tl.object_pose(screw, end)
            seated = -sum(a * b for a, b in zip((qx - fx, qy - fy, qz - fz), normal))
            row["seated_mm"] = round(seated * 1e3, 3)
            checks["seated"] = "pass" if abs(seated - joint.fastener.length_m) <= 0.001 else "fail"
            if run is not None:
                row["drive_s"] = round(run.end - run.start, 3)
            oks = [t for t in ok_lane.rising_edges() if t_engage <= t <= end + 1e-9]
            noks = [t for t in nok_lane.rising_edges() if t_engage <= t <= end + 1e-9]
            row["attempts"] = len(oks) + len(noks)
            row["result"] = "ok" if oks and not noks else "retry" if oks else "nok" if noks else "not run"
        checks["engagement"] = ("pass" if joint.min_engagement_mm is None or row["engagement_mm"] >= joint.min_engagement_mm - 1e-9
                                else "fail")
        depth = joint.pattern_a.hole(hole).depth_mm
        checks["depth"] = "n/a" if depth is None else ("pass" if row["tip_mm"] <= depth + 1e-9 else "fail")
        lo, hi = joint.torque_nm
        checks["torque"] = "pass" if lo - 1e-9 <= driver.torque_nm <= hi + 1e-9 else "fail"
        row["checks"] = checks
        rows.append(row)
    return rows


def fit_report(tl, placement: Placement, *, tolerance_mm: float = 0.5, tilt_deg: float = 0.5) -> dict:
    """Where a placed part landed against where it should (`expected`),
    at the moment it was released: `offset_mm` in the seat's plane,
    `height_mm` along its normal (positive = proud of the seat),
    `tilt_deg`, `t_release`, and `checks` — `position` within
    `tolerance_mm`, `tilt` within `tilt_deg`, `seated` (within the
    tolerance along the normal)."""
    (ex, ey, ez), eq = placement.expected
    normal = _rotate(eq, (0.0, 0.0, 1.0))
    span = _span(tl, _sequence_of(tl, placement.steps["release"]), placement.steps["release"])
    if span is None:
        raise ValueError(f"the timeline has no step {placement.steps['release']!r}: was this placement baked?")
    release = span.start
    (px, py, pz), pq = tl.object_pose(placement.part, release + 1e-6)
    d = (px - ex, py - ey, pz - ez)
    along = sum(a * b for a, b in zip(d, normal))
    lateral = math.sqrt(max(0.0, sum(a * a for a in d) - along * along))
    axis = _rotate(pq, (0.0, 0.0, 1.0))
    cos = max(-1.0, min(1.0, sum(a * b for a, b in zip(axis, normal))))
    tilt = math.degrees(math.acos(cos))
    return {
        "part": placement.part, "t_release": round(release, 3),
        "offset_mm": round(lateral * 1e3, 3), "height_mm": round(along * 1e3, 3), "tilt_deg": round(tilt, 3),
        "checks": {
            "position": "pass" if lateral * 1e3 <= tolerance_mm + 1e-9 else "fail",
            "tilt": "pass" if tilt <= tilt_deg else "fail",
            "seated": "pass" if abs(along) * 1e3 <= tolerance_mm + 1e-9 else "fail",
        },
    }


def export_sheet(path: Union[str, Path], fastening: Fastening, *, rows: Optional[list[dict]] = None,
                 format: Optional[str] = None, title: Optional[str] = None) -> None:
    """The tightening sheet: one line per screw in the order it is driven
    — hole, screw, thread, length, class, head, engagement, torque, the
    driver's rundown time and the step that runs it — as Markdown or CSV
    (by `format` or the file's suffix). With `rows` (a `fastening_report`)
    the baked result columns are added and the Markdown states the
    driver's timing per screw and how many screws were driven."""
    path = Path(path)
    fmt = (format or path.suffix.lstrip(".")).lower()
    if fmt not in ("md", "markdown", "csv"):
        raise ValueError("tightening sheet format must be md or csv")
    joint, driver, f = fastening.joint, fastening.driver, fastening.joint.fastener
    by_hole = {r["hole"]: r for r in rows} if rows else {}
    head = ["order", "hole", "screw", "thread", "length_mm", "class", "head", "engagement_mm",
            "torque_nm", "rundown_s", "step"]
    if rows:
        head += ["offset_mm", "seated_mm", "drive_s", "attempts", "result"]
    lines = []
    for k, (screw, hole) in enumerate(fastening.pairs):
        names = fastening.steps[k] if k < len(fastening.steps) else {}
        line = [str(k + 1), hole, screw, f"M{_plain(f.thread_mm)}x{_plain(f.pitch_mm)}", str(_plain(f.length_mm)),
                f.property_class, f.head_standard, f"{joint.engagement_mm(hole):.1f}",
                f"{driver.torque_nm:.2f}", f"{driver.t_run_s:.2f}", names.get("run", "")]
        if rows:
            r = by_hole.get(hole, {})
            line += ["" if r.get("offset_mm") is None else f"{r['offset_mm']:.2f}",
                     "" if r.get("seated_mm") is None else f"{r['seated_mm']:.1f}",
                     "" if r.get("drive_s") is None else f"{r['drive_s']:.2f}",
                     str(r.get("attempts", "")), str(r.get("result", ""))]
        lines.append(line)
    if fmt == "csv":
        import csv

        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(head)
            writer.writerows(lines)
        return
    lo, hi = joint.torque_nm
    summary = (f"Joint `{joint.name}`: {joint.b} onto {joint.a}, {len(fastening.pairs)} × {f.model_name}, "
               f"{lo:.2f}–{hi:.2f} N·m (driver `{driver.name}` set to {driver.torque_nm:.2f} N·m, "
               f"{driver.rpm_run:g} rpm rundown, {driver.rpm_final:g} rpm final). "
               f"Order: {', '.join(fastening.joint.order)}.")
    text = [f"# Tightening sheet — {title or joint.name}", "", summary]
    if f.torque_ref_nm is not None:
        text.append(f"Reference torque for M{_plain(f.thread_mm)} class {f.property_class} at μ = 0.14: "
                    f"{f.torque_ref_nm:.2f} N·m (ceiling, not the joint's value).")
    text.append(_timing_line(driver))
    if rows:
        text.append(_outcome_line(rows))
    text += ["", "| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    text += ["| " + " | ".join(line) + " |" for line in lines]
    path.write_text("\n".join(text) + "\n", encoding="utf-8")


def _timing_line(driver: Driver) -> str:
    f = driver.fastener
    return (f"Driver time per screw {driver.t_cycle_s:.2f} s: find {driver.t_find_s:.2f} s, "
            f"run {driver.t_run_s:.2f} s ({driver.run_m * 1e3:.0f} mm at {driver.rpm_run:g} rpm, pitch {f.pitch_mm:g}), "
            f"pre-tighten {driver.t_pre_s:.2f} s, final {driver.t_final_s:.2f} s "
            f"({driver.final_turn:.2f} turn at {driver.rpm_final:g} rpm).")


def _outcome_line(rows: list[dict]) -> str:
    driven = sum(1 for r in rows if r.get("result") in ("ok", "retry"))
    retried = sum(1 for r in rows if r.get("result") == "retry")
    return (f"Baked result: {driven}/{len(rows)} screws driven"
            + (f", {retried} after a retry" if retried else "") + ".")


def report_section(fastening: Fastening, rows: Optional[list[dict]] = None,
                   fit: Optional[dict] = None, *, title: str = "Assembly") -> dict:
    """The assembly section of a cell report (`scene.cell_report(...,
    sections=[...])`): the joint — its parts, screw, torque, engagement
    and order — the driver's timing, the fit of the placed part (a
    `fit_report`) and the baked fastening rows (a `fastening_report`),
    as Markdown and as JSON. What a reviewer reads before the interlock
    table and the FAT rows."""
    joint, driver, f = fastening.joint, fastening.driver, fastening.joint.fastener
    lo, hi = joint.torque_nm
    engagement = min(joint.engagement_mm(h) for h in joint.order)
    tool_range = driver.tool.get("torque_nm")
    tool_note = (f" (tool {tool_range[0]:g}–{tool_range[1]:g} N·m)"
                 if isinstance(tool_range, (tuple, list)) and len(tool_range) == 2 else "")
    lines = [
        f"Joint `{joint.name}`: `{joint.b}` onto `{joint.a}`, {len(fastening.pairs)} × {f.model_name}"
        + (f" ({joint.catalog[0]})" if joint.catalog else "") + f", order {', '.join(joint.order)}.",
        f"Torque {lo:.2f}–{hi:.2f} N·m, driver `{driver.name}` set to {driver.torque_nm:.2f} N·m" + tool_note
        + f"; thread engagement {engagement:.1f} mm"
        + (f" against {joint.min_engagement_mm:.1f} mm needed" if joint.min_engagement_mm is not None else "")
        + f"; {fastening.retry} retry allowed, then `{fastening.halted}`.",
        _timing_line(driver),
    ]
    if fit is not None:
        checks = ", ".join(f"{k} {v}" for k, v in fit["checks"].items())
        lines.append(f"Fit of `{fit['part']}`: off {fit['offset_mm']:.2f} mm, {fit['height_mm']:+.2f} mm along the seat, "
                     f"tilt {fit['tilt_deg']:.2f}° — {checks}.")
    if rows:
        lines.append(_outcome_line(rows))
    md = "\n".join(lines) + "\n"
    if rows:
        head = ["#", "hole", "screw", "offset mm", "tilt °", "seated mm", "drive s", "attempts", "result", "checks"]
        md += "\n| " + " | ".join(head) + " |\n|" + "---|" * len(head) + "\n"
        for r in rows:
            fails = [k for k, v in r["checks"].items() if v == "fail"]
            md += "| " + " | ".join([
                str(r["order"]), r["hole"], r["screw"],
                "" if r["offset_mm"] is None else f"{r['offset_mm']:.2f}",
                "" if r["tilt_deg"] is None else f"{r['tilt_deg']:.2f}",
                "" if r["seated_mm"] is None else f"{r['seated_mm']:.1f}",
                "" if r["drive_s"] is None else f"{r['drive_s']:.2f}",
                str(r["attempts"]), str(r["result"]), "all pass" if not fails else "fail: " + ", ".join(fails),
            ]) + " |\n"
    return {
        "title": title,
        "markdown": md,
        "json": {
            "joint": {
                "name": joint.name, "a": joint.a, "b": joint.b, "fastener": f.model_name,
                "catalog": list(joint.catalog) if joint.catalog else None,
                "torque_nm": [lo, hi], "min_engagement_mm": joint.min_engagement_mm,
                "order": list(joint.order),
                "holes": {h: {"engagement_mm": joint.engagement_mm(h), "tip_mm": joint.tip_depth_mm(h)} for h in joint.order},
            },
            "driver": {"name": driver.name, "program": driver.program, "torque_nm": driver.torque_nm,
                       "rpm_run": driver.rpm_run, "rpm_final": driver.rpm_final, "phases_s": driver.phases(),
                       "cycle_s": driver.t_cycle_s, "retry": fastening.retry, "halted": fastening.halted},
            "fit": fit,
            "screws": rows or [],
        },
    }
