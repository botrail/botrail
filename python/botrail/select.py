"""Requirements derived from the cell, and the selection check.

botrail does not choose parts. It derives what every bill-of-materials line
must be able to do — from the cell the part sits in — compares that with what
the chosen part says it can do, and reports where the two disagree or where
it does not know. Choosing stays with the person, the agent or the vendor;
:mod:`botrail.catalog` finds the real candidates to choose from.

    req = scene.requirements()          # one row per BOM line
    print(req.to_markdown())
    req["tool"].minimum                 # {"payload_kg": 2.3, "stroke_mm": 150.0}
    report = scene.check()              # I/O lint + sequences + parts + requirements
    assert report.ok, report.to_markdown()

**Vocabulary.** A requirement names a spec the catalog names too, so a value
read from a catalog package (`Robot.from_catalog`, `bt.parts.*(catalog=...)`,
`bt.catalog.Product.identify`) or typed by hand on `set_part(...)` lands in the
same column. The keys, and the attribute names that answer them
(:data:`ALIASES`):

| requirement            | derived from                                               | answered by                                  |
|------------------------|------------------------------------------------------------|----------------------------------------------|
| `payload_kg`           | tool mass + the heaviest part the robot grasps; parts riding a vehicle's deck at start | `payload_kg`                                 |
| `reach_mm`             | the farthest taught target from the base, plus a margin; the table of a machining centre (or the spindle of a lathe) in the cell, through its opening | `reach_mm`                                   |
| `stroke_mm`            | the smallest side of the grasped parts (parallel gripper)  | `stroke_mm`, `opening_mm`                    |
| `sensing_range_mm`     | a beam sensor's span                                       | `sensing_range_mm`, `range_mm`, `max_range_mm` |
| `range_mm`             | a light curtain's span / an area sensor's half-diagonal    | `range_mm`, `max_range_mm`, `sensing_range_mm` |
| `protective_height_mm` | a light curtain's post height                              | `protective_height_mm`, `height_mm`          |
| `scan_fov_deg`         | a lidar's authored sweep angle                             | `scan_fov_deg`                               |
| `length_mm`, `width_mm`| a conveyor's zone along and across its belt                | `length_mm` / `width_mm`, `belt_width_mm`    |
| `speed_mps`            | a conveyor's belt speed, an axis speed                     | `max_speed_mps`, `speed_max_mps`, `speed_mps`|
| `max_speed_mps`        | a vehicle's travel speed                                   | the same                                     |
| `max_climb_mps`        | an aerial vehicle's climb rate                             | the same                                     |
| `max_descent_mps`      | an aerial vehicle's descent rate                           | the same                                     |
| `flight_time_min`      | an aerial vehicle's airborne time per cycle, from the baked timeline (`requirements(timeline=tl)`) | the same |
| `load_kg`              | parts on a conveyor / an axis; robots standing on a pedestal | `load_kg`, `capacity_kg`, `max_load_kg`, `payload_kg` |
| `torque_nm`            | a screwdriver's tightening torque: the joint torque on the screws it drives (`bt.assembly.fasten`) | `torque_max_nm`, `max_torque_nm`, `torque_nm` |
| `screw_length_mm`, `thread_max_mm` (≥) / `thread_min_mm` (≤) | the longest screw, the largest and smallest thread the screwdriver has to take | `screw_length_mm`, `thread_max_mm`, `thread_min_mm` |
| `di` `do` `ai` `ao` `safe_di` `safe_do` | points assigned to an I/O node                | the node's declared channels                 |

Every requirement is a minimum (`>=`) unless noted. The derivations are
geometric and deterministic — no sizing, no safety evaluation; a value that
cannot be derived (a grasped part without `mass_kg`) is reported as a note,
never guessed.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterator
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Union

__all__ = [
    "ALIASES",
    "CheckReport",
    "Finding",
    "Requirement",
    "Requirements",
    "Row",
    "check",
    "requirements",
]

#: Requirement key -> the part attributes that answer it, in priority order.
ALIASES: dict[str, tuple[str, ...]] = {
    "payload_kg": ("payload_kg",),
    "reach_mm": ("reach_mm",),
    # Arms a dual-arm product must have: the arms the cell's motions use.
    "arm_count": ("arm_count",),
    "stroke_mm": ("stroke_mm", "opening_mm"),
    # A hand's widest opening across its grasp surfaces — the multifinger
    # analogue of a parallel gripper's stroke (design-grasping.md G2).
    "aperture_mm": ("aperture_mm",),
    # Holding force the grasp must apply; answered by the gripper's stated
    # capability (max first; a min-only figure still answers conservatively).
    "grip_force_n": ("grip_force_max_n", "grip_force_min_n"),
    "sensing_range_mm": ("sensing_range_mm", "range_mm", "max_range_mm"),
    "fov_deg": ("fov_h_deg", "hfov_deg", "fov_deg"),
    "resolution_h_px": ("resolution_h_px",),
    "resolution_v_px": ("resolution_v_px",),
    "max_range_mm": ("max_range_mm",),
    "min_range_mm": ("min_range_mm",),
    "scan_fov_deg": ("scan_fov_deg",),
    "range_mm": ("range_mm", "max_range_mm", "sensing_range_mm"),
    "protective_height_mm": ("protective_height_mm", "height_mm"),
    "length_mm": ("length_mm",),
    "width_mm": ("width_mm", "belt_width_mm"),
    "speed_mps": ("max_speed_mps", "speed_max_mps", "speed_mps"),
    "max_speed_mps": ("max_speed_mps", "speed_max_mps", "speed_mps"),
    "max_climb_mps": ("max_climb_mps",),
    "max_descent_mps": ("max_descent_mps",),
    "flight_time_min": ("flight_time_min",),
    "load_kg": ("load_kg", "capacity_kg", "max_load_kg", "payload_kg"),
    # A screwdriver's tightening torque and the screws it takes (the
    # joint's torque lands on the screw lines — `bt.assembly.fasten`).
    "torque_nm": ("torque_max_nm", "max_torque_nm", "torque_nm"),
    "screw_length_mm": ("screw_length_mm", "max_screw_length_mm", "screw_length_max_mm"),
    "thread_max_mm": ("thread_max_mm",),
    "thread_min_mm": ("thread_min_mm",),
    "output_a": ("output_a", "current_a"),
    # The robot cable a placed controller box needs, arm base to box.
    "cable_m": ("cable_m", "cable_length_m"),
    "di": ("di",),
    "do": ("do",),
    "ai": ("ai",),
    "ao": ("ao",),
    "safe_di": ("safe_di",),
    "safe_do": ("safe_do",),
}

_SEVERITY_ORDER = {"error": 0, "warning": 1, "info": 2}
_EPS = 1e-9


# ---------------------------------------------------------------- results


@dataclass
class Requirement:
    """One thing a BOM line must be able to do, and whether its part can."""

    key: str
    value: float
    op: str = ">="
    basis: str = ""
    provided: Optional[float] = None
    provided_key: Optional[str] = None
    #: `ok` | `short` | `unknown` (identified part, no value) | `unidentified`
    status: str = "unknown"
    #: Whether `findings()` reports a shortfall. An I/O node's channel
    #: capacity is the I/O report's business, so those are derived for the
    #: table but not linted twice.
    lint: bool = True

    @property
    def ok(self) -> bool:
        return self.status == "ok"

    def to_dict(self) -> dict[str, Any]:
        return {
            "key": self.key,
            "op": self.op,
            "value": self.value,
            "basis": self.basis,
            "provided": self.provided,
            "provided_key": self.provided_key,
            "status": self.status,
        }

    def __str__(self) -> str:
        return f"{self.key} {self.op} {_fmt(self.value)}"


@dataclass
class Row:
    """One bill-of-materials line with what the cell asks of it."""

    target: str
    names: list[str]
    kind: str
    category: str
    qty: int
    identified: bool
    catalog: Optional[str]
    manufacturer: Optional[str]
    model: Optional[str]
    attributes: dict[str, Any] = field(default_factory=dict)
    requirements: list[Requirement] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

    @property
    def minimum(self) -> dict[str, float]:
        """The `>=` requirements as `{key: value}` — what `bt.catalog.search(**row.minimum)` takes."""
        return {r.key: r.value for r in self.requirements if r.op == ">="}

    @property
    def status(self) -> str:
        """`ok` | `short` | `unknown` | `unidentified` | `none` (nothing derived)."""
        if not self.requirements:
            return "none" if self.identified else "unidentified"
        if not self.identified:
            return "unidentified"
        if any(r.status == "short" for r in self.requirements):
            return "short"
        if any(r.status == "unknown" for r in self.requirements):
            return "unknown"
        return "ok"

    @property
    def ok(self) -> bool:
        return self.status in ("ok", "none")

    def to_dict(self) -> dict[str, Any]:
        return {
            "target": self.target,
            "names": list(self.names),
            "kind": self.kind,
            "category": self.category,
            "qty": self.qty,
            "identified": self.identified,
            "catalog": self.catalog,
            "manufacturer": self.manufacturer,
            "model": self.model,
            "status": self.status,
            "requirements": [r.to_dict() for r in self.requirements],
            "notes": list(self.notes),
        }


@dataclass
class Finding:
    severity: str
    code: str
    message: str
    target: Optional[str] = None

    def to_dict(self) -> dict[str, Any]:
        d: dict[str, Any] = {"severity": self.severity, "code": self.code, "message": self.message}
        if self.target is not None:
            d["target"] = self.target
        return d


class Requirements:
    """What the cell asks of every BOM line — the result of :func:`requirements`."""

    def __init__(self, rows: list[Row], *, margin: float, sequences: list[str]) -> None:
        self.rows = rows
        self.margin = margin
        self.sequences = sequences

    def __iter__(self) -> Iterator[Row]:
        return iter(self.rows)

    def __len__(self) -> int:
        return len(self.rows)

    def __getitem__(self, name: str) -> Row:
        """The row for a BOM line — by its first name or any resident merged into it."""
        for row in self.rows:
            if row.target == name or name in row.names:
                return row
        raise KeyError(f"no BOM line named {name!r} (lines: {[r.target for r in self.rows]})")

    def __contains__(self, name: object) -> bool:
        return any(r.target == name or name in r.names for r in self.rows)

    # ------------------------------------------------------------ views

    def short(self) -> list[Row]:
        return [r for r in self.rows if r.status == "short"]

    def unknown(self) -> list[Row]:
        return [r for r in self.rows if r.status == "unknown"]

    def unidentified(self) -> list[Row]:
        return [r for r in self.rows if not r.identified]

    @property
    def ok(self) -> bool:
        """No line falls short of what the cell asks (unknowns do not count)."""
        return not self.short()

    def findings(self) -> list[Finding]:
        """`spec_short` (error), `spec_unknown` (warning) and
        `requirement_incomplete` (info), in row order. An I/O node's
        channel capacity gets no spec finding — the I/O report already
        lints it — but its robot cable does."""
        out: list[Finding] = []
        for row in self.rows:
            for r in row.requirements:
                if not r.lint:
                    continue
                if r.status == "short":
                    out.append(
                        Finding(
                            "error",
                            "spec_short",
                            f"{row.target}: {r.key} {_fmt(r.provided)} < required {_fmt(r.value)}"
                            f" ({r.basis})",
                            row.target,
                        )
                    )
                elif r.status == "unknown":
                    out.append(
                        Finding(
                            "warning",
                            "spec_unknown",
                            f"{row.target}: needs {r}" + (f" ({r.basis})" if r.basis else "")
                            + f" but the part does not say — add {r.key}= on set_part or pick a catalog item",
                            row.target,
                        )
                    )
            for note in row.notes:
                out.append(Finding("info", "requirement_incomplete", f"{row.target}: {note}", row.target))
        return out

    # ---------------------------------------------------------- formats

    def to_markdown(self) -> str:
        lines = [
            "| line | category | requirement | basis | provided | status |",
            "|---|---|---|---|---|---|",
        ]
        for row in self.rows:
            label = row.target + (f" (x{row.qty})" if row.qty > 1 else "")
            if not row.requirements:
                lines.append(f"| {label} | {row.category} | — | | | {row.status} |")
                continue
            for i, r in enumerate(row.requirements):
                provided = "" if r.provided is None else _fmt(r.provided)
                if r.provided_key and r.provided_key != r.key:
                    provided += f" ({r.provided_key})"
                lines.append(
                    f"| {label if i == 0 else ''} | {row.category if i == 0 else ''} | {r} | {r.basis} |"
                    f" {provided} | {r.status} |"
                )
        notes = [f"- {row.target}: {n}" for row in self.rows for n in row.notes]
        if notes:
            lines += ["", "Notes:", *notes]
        return "\n".join(lines) + "\n"

    def to_json(self) -> str:
        return json.dumps(
            {
                "margin": self.margin,
                "sequences": list(self.sequences),
                "rows": [row.to_dict() for row in self.rows],
            },
            indent=2,
        )

    def to_csv(self) -> str:
        import csv
        import io

        buf = io.StringIO()
        w = csv.writer(buf)
        w.writerow(["line", "category", "qty", "identified", "requirement", "op", "value", "basis", "provided", "status"])
        for row in self.rows:
            if not row.requirements:
                w.writerow([row.target, row.category, row.qty, row.identified, "", "", "", "", "", row.status])
            for r in row.requirements:
                w.writerow(
                    [row.target, row.category, row.qty, row.identified, r.key, r.op, _fmt(r.value), r.basis,
                     "" if r.provided is None else _fmt(r.provided), r.status]
                )
        return buf.getvalue()

    def save(self, path: Union[str, Path], format: Optional[str] = None) -> None:
        path = Path(path)
        fmt = (format or path.suffix.lstrip(".") or "md").lower()
        if fmt in ("md", "markdown"):
            text = self.to_markdown()
        elif fmt == "json":
            text = self.to_json()
        elif fmt == "csv":
            text = self.to_csv()
        else:
            raise ValueError(f"unknown format {fmt!r} — use md, json or csv")
        path.write_text(text, encoding="utf-8")


class CheckReport:
    """Every static check of a cell in one list — what `botrail check` prints."""

    def __init__(self, findings: list[Finding], requirements: Requirements) -> None:
        self.findings = findings
        self.requirements = requirements

    @property
    def ok(self) -> bool:
        return not self.errors()

    def errors(self) -> list[Finding]:
        return [f for f in self.findings if f.severity == "error"]

    def warnings(self) -> list[Finding]:
        return [f for f in self.findings if f.severity == "warning"]

    def infos(self) -> list[Finding]:
        return [f for f in self.findings if f.severity == "info"]

    def __len__(self) -> int:
        return len(self.findings)

    def __iter__(self) -> Iterator[Finding]:
        return iter(self.findings)

    def to_dict(self) -> dict[str, Any]:
        req = self.requirements
        return {
            "ok": self.ok,
            "findings": [f.to_dict() for f in self.findings],
            "requirements": {
                "lines": len(req),
                "short": len(req.short()),
                "unknown": len(req.unknown()),
                "unidentified": len(req.unidentified()),
            },
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2)

    def to_markdown(self) -> str:
        counts = {s: len([f for f in self.findings if f.severity == s]) for s in _SEVERITY_ORDER}
        head = ("ok" if self.ok else "FAIL") + " — " + ", ".join(f"{n} {s}" for s, n in counts.items() if n)
        if not self.findings:
            return head + " — no findings\n"
        lines = [head, "", "| severity | code | message |", "|---|---|---|"]
        lines += [f"| {f.severity} | {f.code} | {f.message} |" for f in self.findings]
        return "\n".join(lines) + "\n"


# ------------------------------------------------------------- derivation


def requirements(
    scene, *, sequences: Optional[list[str]] = None, margin: float = 0.1, timeline=None,
    cable_slack_m: float = 1.0,
) -> Requirements:
    """Derive what every BOM line must be able to do from the cell it is in,
    and compare it with what the chosen part says (its catalog specs or the
    attributes typed on `set_part`).

    `sequences` limits the programs whose grasps and I/O points are counted
    (default: all). `margin` is added to the reach and flight-time
    requirements (0.1 = 10 %). `timeline` is an optional baked
    `simulate_sequences` result: cycle facts only it can supply — an aerial
    vehicle's airborne time — are derived from it, and left as a note when
    it is absent. `cable_slack_m` is what a placed controller's robot cable
    needs beyond the run along the axes from the arm's base to the box (a
    drop, a loop, the terminations). Nothing is chosen and nothing is
    sized: a number the cell cannot supply (a grasped part with no
    `mass_kg`) becomes a note, not a guess.
    """
    if margin < 0:
        raise ValueError("margin must be >= 0")
    if cable_slack_m < 0:
        raise ValueError("cable_slack_m must be >= 0")
    cell = _Cell(scene, sequences, timeline)
    cell.cable_slack_m = float(cable_slack_m)
    rows: list[Row] = []
    for bom_row in scene.bom().rows:
        names = list(bom_row["names"])
        category = bom_row["category"] or ""
        kind = cell.kind_of(names[0], category)
        identified = bool(bom_row.get("catalog") or bom_row.get("model") or bom_row.get("manufacturer"))
        if category in ("", "vehicle", "robot"):
            # The derived default categories. An aerial machine is shopped
            # in one aisle: narrow them so `search_for` looks there. A
            # category an author or a catalog identity stated is never one
            # of these, so it is never overridden.
            category = cell.category_hint(names[0], kind) or category
        reqs: list[Requirement] = []
        notes: list[str] = []
        for name in names:
            r, n = cell.derive(name, kind, category, margin)
            reqs += r
            notes += n
        reqs = _merge(reqs)
        attributes = dict(bom_row.get("attributes") or {})
        if kind == "io_node":
            # A node's "provided" is what it declares: its channels by kind.
            attributes = {**cell.node_capacity(names[0]), **attributes}
        for r in reqs:
            key, value = _provided(attributes, r.key)
            r.provided, r.provided_key = value, key
            if not identified:
                r.status = "unidentified"
            elif value is None:
                r.status = "unknown"
            else:
                r.status = "ok" if _satisfies(r, value) else "short"
        rows.append(
            Row(
                target=names[0],
                names=names,
                kind=kind,
                category=category,
                qty=int(bom_row.get("qty") or 1),
                identified=identified,
                catalog=bom_row.get("catalog"),
                manufacturer=bom_row.get("manufacturer"),
                model=bom_row.get("model"),
                attributes=attributes,
                requirements=reqs,
                notes=_dedupe(notes),
            )
        )
    return Requirements(rows, margin=margin, sequences=cell.sequence_names)


def check(
    scene, *, sequences: Optional[list[str]] = None, timeline=None,
    cable_slack_m: float = 1.0, service_clearance_m: float = 0.9,
) -> CheckReport:
    """Every static check in one report: the I/O lint, each sequence walked
    for dangling references, unidentified equipment lines (with what the
    cell asks of them), the requirement comparison, and the placement of
    the cabinets — a controller nobody has placed (`controller_unplaced`),
    and the service space in front of a control cabinet's or controller's
    door blocked by something (`service_space`; `service_clearance_m`
    deep, or what the part states as `service_clearance_mm`). `timeline`
    (a baked cycle) adds the cycle-fact requirements — an aerial vehicle's
    flight time; `cable_slack_m` is the robot cable allowance beyond the
    run from arm to box. Errors make `ok` false; `botrail check` prints
    exactly this."""
    if service_clearance_m < 0:
        raise ValueError("service_clearance_m must be >= 0")
    findings: list[Finding] = []
    io_error: Optional[str] = None
    try:
        report = scene.io_report(sequences) if sequences is not None else scene.io_report()
        findings += [Finding(f.severity, f.code, f.message) for f in report.findings]
    except ValueError as e:
        io_error = str(e)
        findings.append(Finding("error", "io_derivation", io_error))
    for name in sequences if sequences is not None else scene.sequence_names:
        # A sequence that cannot even be walked is an error, not a surprise
        # at bake time: `io_points` derives per program and validates the
        # references on the way.
        try:
            scene.io_points(sequences=[name])
        except ValueError as e:
            if io_error is None:
                findings.append(Finding("error", "sequence", f"{name}: {e}", name))
    req = requirements(scene, sequences=sequences, timeline=timeline, cable_slack_m=cable_slack_m)
    unidentified = {tuple(r["names"]) for r in scene.bom().unidentified()}
    for row in req.rows:
        if tuple(row.names) in unidentified:
            message = f"{', '.join(row.names)} ({row.category}) has no maker, model or catalog reference"
            if row.requirements:
                message += " — needs " + ", ".join(str(r) for r in row.requirements)
            findings.append(Finding("info", "unidentified_part", message, row.target))
    findings += req.findings()
    findings += _placement_findings(scene, sequences, req, service_clearance_m)
    # A tool whose installation documents require a part that is missing from
    # its attachment path (a bare 2F-85 without its coupling) is a warning:
    # the product says so, and the cell as authored cannot be built that way.
    from .mounting import report as mounting_report

    for item in mounting_report(scene).items:
        if item.status == "fail":
            code = "required_part_missing" if item.key.startswith("required:") else item.key
            findings.append(Finding("warning", code, item.message, item.target))
    return CheckReport(findings, req)


def _placement_findings(scene, sequences, req: "Requirements", clearance_m: float) -> list[Finding]:
    """Where the cabinets stand. A robot controller nobody has placed (the
    derived `<robot>/controller` line, or a declared node without `place`)
    is an info; a control cabinet or controller whose door face
    (`<name>/front`) has something standing in the service space in front
    of it is a warning. The space is the door's width by the box's height,
    `clearance_m` deep — or what the part states as `service_clearance_mm`,
    the maker's own figure from a catalog pack."""
    out: list[Finding] = []
    cell = _Cell(scene, sequences)
    frames = set(scene.frames)
    for row in req.rows:
        name = row.target
        if row.category == "robot_controller":
            node = cell.nodes.get(name)
            if node is None or not node.get("place"):
                out.append(
                    Finding(
                        "info",
                        "controller_unplaced",
                        f"{name}: the controller is not placed — bt.parts.controller(scene, {name!r}, ...) "
                        "puts the box on the floor plan, or place= on add_io_node names where it stands",
                        name,
                    )
                )
                continue
        elif row.category != "structure.cabinet":
            continue
        front = f"{name}/front"
        extent = cell.extent_of(f"{name}/body")
        if front not in frames or extent is None:
            continue
        stated = _number(row.attributes.get("service_clearance_mm"))
        depth = stated / 1000.0 if stated is not None else float(clearance_m)
        if depth <= 0:
            continue
        w, _, h = extent
        (fx, fy, fz), q = scene.frame(front)
        # The door faces the frame's -Y: the space is the box that side of it.
        ox, oy, oz = _rotate(q, (0.0, -depth / 2.0, 0.0))
        centre = (fx + ox, fy + oy, fz + oz + h / 2.0)
        blockers = cell.obstacles_meeting(centre, q, (w, depth, h), exclude=f"{name}/")
        if blockers:
            shown = ", ".join(blockers[:3]) + (f" and {len(blockers) - 3} more" if len(blockers) > 3 else "")
            out.append(
                Finding(
                    "warning",
                    "service_space",
                    f"{name}: {_fmt(depth)} m in front of {front} is blocked by {shown}",
                    name,
                )
            )
    return out


# ----------------------------------------------------------------- the cell


class _Cell:
    """Everything the derivations read, indexed once from the project JSON."""

    def __init__(self, scene, sequences: Optional[list[str]], timeline=None) -> None:
        self.scene = scene
        self.timeline = timeline
        self.project = json.loads(scene._project_json())
        self.robots: list[str] = list(scene.robots)
        self.default_robot = self.robots[0] if self.robots else None
        all_sequences = {s["name"]: s for s in self.project.get("sequences") or []}
        if sequences is None:
            self.sequences = list(all_sequences.values())
        else:
            missing = [s for s in sequences if s not in all_sequences]
            if missing:
                raise ValueError(f"unknown sequence(s) {missing} (have {sorted(all_sequences)})")
            self.sequences = [all_sequences[s] for s in sequences]
        self.sequence_names = [s["name"] for s in self.sequences]
        self.obstacles: dict[str, dict] = {o["name"]: o for o in self.project.get("obstacles") or []}
        self.sensors: dict[str, dict] = {s["name"]: s["kind"] for s in self.project.get("sensors") or []}
        self.cameras: dict[str, dict] = {c["name"]: c for c in self.project.get("cameras") or []}
        self.lidars: dict[str, dict] = {l["name"]: l for l in self.project.get("lidars") or []}
        self.devices: dict[str, dict] = {d["name"]: d["kind"] for d in self.project.get("devices") or []}
        self.mounts: dict[str, dict] = {
            (r.get("name") or self.default_robot): r["mount"]
            for r in self.project.get("robots") or []
            if r.get("mount")
        }
        io = self.project.get("io") or {}
        self.nodes: dict[str, dict] = {n["name"]: n for n in io.get("nodes") or []}
        self.parts: dict[tuple[str, str], dict] = {(p["target"], p["kind"]): p for p in scene.parts()}
        self.bom_rows: list[dict] = list(scene.bom().rows)
        self.points = self._points(sequences)
        self._grasps: dict[str, list[str]] = {}
        self._mass_cache: dict[str, Optional[float]] = {}
        self.cable_slack_m = 1.0

    def _points(self, sequences: Optional[list[str]]) -> list[dict]:
        try:
            text = self.scene.io_list("json", sequences) if sequences is not None else self.scene.io_list("json")
        except ValueError:
            return []
        try:
            return list(json.loads(text).get("points") or [])
        except (ValueError, AttributeError):
            return []

    # ------------------------------------------------------------ naming

    def kind_of(self, name: str, category: str) -> str:
        if name in self.robots:
            return "robot"
        head, _, tail = name.rpartition("/")
        if head in self.robots and tail.startswith("tool"):
            return "tool"
        # A tool on a tool — a gripper on a bracket (`arm/tool/tool2`).
        root, *chain = name.split("/")
        if root in self.robots and chain and all(seg.startswith("tool") for seg in chain):
            return "tool"
        if head in self.robots and tail == "controller":
            # The controller an arm needs before a cabinet is declared for
            # it: a node, sized from the points on the arm's implicit host.
            return "io_node"
        # An arm of a dual-arm robot assembled from catalog arms is its
        # own line, sized like a robot.
        if head in self.robots and tail in self.arms_of(head):
            return "robot"
        kinds = [k for (t, k) in self.parts if t == name]
        if len(kinds) == 1:
            return kinds[0]
        if len(kinds) > 1:
            hint = _kind_hint(category)
            if hint in kinds:
                return hint
            return min(kinds)
        if name in self.devices:
            return "device"
        if name in self.sensors:
            return "sensor"
        if name in self.cameras:
            return "camera"
        if name in self.lidars:
            return "lidar"
        if name in self.nodes:
            return "io_node"
        if name in self.obstacles:
            return "obstacle"
        return "group"

    def vehicle_of(self, robot: str) -> Optional[str]:
        """The vehicle merged into this robot's BOM line — the machine *is*
        the robot: legs (a gait mount), or the whole airframe (a rigid mount
        on a vehicle with no body of its own — a UAV). Mirrors `Scene::bom`,
        including the escape: a part pinned on the device keeps it a line of
        its own, so the robot does not absorb its requirements."""
        mount = self.mounts.get(robot)
        if not mount:
            return None
        device = mount.get("device")
        kind = self.devices.get(device)
        if not kind or kind.get("kind") != "vehicle" or (device, "device") in self.parts:
            return None
        if mount.get("gait") or not kind.get("body"):
            return device
        return None

    def category_hint(self, name: str, kind: str) -> Optional[str]:
        """A shopping aisle for a line the author left with the derived
        default category. Only the aerial machine has exactly one aisle;
        ground vehicles stay unhinted (cart, AGV or AMR is a choice)."""
        if kind == "device":
            device = self.devices.get(name)
        elif kind == "robot":
            ridden = self.vehicle_of(name)
            device = self.devices.get(ridden) if ridden else None
        else:
            return None
        if device and device.get("kind") == "vehicle" and device.get("aerial"):
            return "vehicle.uav"
        return None

    def derive(self, name: str, kind: str, category: str, margin: float) -> tuple[list[Requirement], list[str]]:
        if kind == "robot":
            return self._robot(name, margin)
        if kind == "tool":
            return self._tool(name, category)
        if kind == "sensor":
            return self._sensor(name, category)
        if kind == "camera":
            return self._camera(name)
        if kind == "lidar":
            return self._lidar(name)
        if kind == "device":
            return self._device(name, margin)
        if kind == "io_node":
            reqs, notes = self._node(name)
            cable, cable_notes = self._controller(name)
            return reqs + cable, notes + cable_notes
        return self._structure(name, category)

    def _screwdriver(self) -> list[Requirement]:
        """What the screws in the cell ask of a screwdriver: the joint
        torque on their lines, the longest of them, the thread range."""
        screws = [
            (target, part.get("attributes") or {})
            for (target, kind), part in self.parts.items()
            if kind == "obstacle" and (part.get("category") or "") == "fastener"
        ]
        reqs: list[Requirement] = []
        torques = [(t, _number(a.get("torque_nm"))) for t, a in screws]
        torques = [(t, v) for t, v in torques if v is not None]
        if torques:
            t, v = max(torques, key=lambda tv: tv[1])
            reqs.append(Requirement("torque_nm", _round(v, 3), basis=f"drives {t} at {_fmt(v)} N·m"))
        lengths = [(t, _number(a.get("length_mm"))) for t, a in screws]
        lengths = [(t, v) for t, v in lengths if v is not None]
        if lengths:
            t, v = max(lengths, key=lambda tv: tv[1])
            reqs.append(Requirement("screw_length_mm", _round(v, 1), basis=f"{t} is {_fmt(v)} mm long"))
        threads = [(t, _number(a.get("thread_mm"))) for t, a in screws]
        threads = [(t, v) for t, v in threads if v is not None]
        if threads:
            t, v = max(threads, key=lambda tv: tv[1])
            reqs.append(Requirement("thread_max_mm", _round(v, 2), basis=f"{t} is M{_fmt(v)}"))
            t, v = min(threads, key=lambda tv: tv[1])
            reqs.append(Requirement("thread_min_mm", _round(v, 2), op="<=", basis=f"{t} is M{_fmt(v)}"))
        return reqs

    # ------------------------------------------------------------ lookups

    def mass_of(self, obstacle: str) -> Optional[float]:
        """`mass_kg` of an obstacle's own part, else of the nearest group
        part above it (one unit's mass, not the group total)."""
        if obstacle in self._mass_cache:
            return self._mass_cache[obstacle]
        value: Optional[float] = None
        part = self.parts.get((obstacle, "obstacle"))
        if part is not None:
            value = _number((part.get("attributes") or {}).get("mass_kg"))
        if value is None:
            prefix = obstacle
            while "/" in prefix and value is None:
                prefix = prefix.rsplit("/", 1)[0]
                group = self.parts.get((prefix, "group"))
                if group is not None:
                    value = _number((group.get("attributes") or {}).get("mass_kg"))
        self._mass_cache[obstacle] = value
        return value

    def extent_of(self, obstacle: str) -> Optional[tuple[float, float, float]]:
        """The obstacle's own size (box sides, a cylinder's diameter and
        length, a sphere's diameter; a mesh's world AABB)."""
        entry = self.obstacles.get(obstacle)
        if entry is None:
            return None
        geometry = entry.get("geometry") or {}
        kind = geometry.get("kind")
        if kind == "box":
            size = geometry.get("size")
            return (float(size[0]), float(size[1]), float(size[2]))
        if kind == "cylinder":
            d = 2.0 * float(geometry.get("radius", 0.0))
            return (d, d, float(geometry.get("length", 0.0)))
        if kind == "sphere":
            d = 2.0 * float(geometry.get("radius", 0.0))
            return (d, d, d)
        try:
            lo, hi = self.scene.obstacle_bounds(obstacle)
        except ValueError:
            return None
        return (hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2])

    def bounds_of_group(self, name: str) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float]]]:
        """World AABB over the obstacle `name` and everything under `name/`."""
        lo = [math.inf] * 3
        hi = [-math.inf] * 3
        found = False
        for obstacle in self.obstacles:
            if obstacle == name or obstacle.startswith(name + "/"):
                try:
                    a, b = self.scene.obstacle_bounds(obstacle)
                except ValueError:
                    continue
                found = True
                for i in range(3):
                    lo[i] = min(lo[i], a[i])
                    hi[i] = max(hi[i], b[i])
        if not found:
            return None
        return (lo[0], lo[1], lo[2]), (hi[0], hi[1], hi[2])

    def arms_of(self, robot: str) -> list[str]:
        """The arms (planning groups) of a dual-arm robot; empty for a
        robot with one."""
        try:
            groups = list(self.scene.robot_of(robot).groups)
        except ValueError:
            return []
        return groups if len(groups) > 1 else []

    def robot_and_arm(self, name: str) -> tuple[str, Optional[str]]:
        """A BOM line `robot/arm` (an arm mounted from the catalog) split
        into the robot and the arm; any other line is a whole robot."""
        robot, _, arm = name.rpartition("/")
        if robot and arm in self.arms_of(robot):
            return robot, arm
        return name, None

    def grasped_by(self, robot: str, arm: Optional[str] = None) -> list[str]:
        """Objects the robot holds now or grasps in a counted sequence —
        with `arm`, those that arm grasps."""
        key = robot if arm is None else f"{robot}/{arm}"
        if key in self._grasps:
            return self._grasps[key]
        tip = None
        if arm is not None:
            group = self.scene.robot_of(robot).group(arm)
            tip = group.flange or group.tip
        names: list[str] = []
        for obstacle, entry in self.obstacles.items():
            attached = entry.get("attached_to")
            if (
                attached
                and (attached.get("robot") or self.default_robot) == robot
                and (arm is None or attached.get("link") == tip)
            ):
                names.append(obstacle)
        for sequence in self.sequences:
            for action in _walk_actions(sequence.get("steps") or []):
                if action.get("type") == "attach" and (action.get("robot") or self.default_robot) == robot:
                    if arm is not None and action.get("group") != arm:
                        continue
                    obj = action.get("object")
                    if obj and obj not in names:
                        names.append(obj)
        self._grasps[key] = names
        return names

    def targets_of(
        self, robot: str, arm: Optional[str] = None
    ) -> tuple[list[tuple[float, float, float]], str, list[str]]:
        """Positions of every taught segment goal of the robot's motions,
        measured at the flange when the robot declares one (a reach spec
        is quoted to the flange, not past the tool), else at the TCP. With
        `arm`, the motions of that arm (and whole-robot ones), measured at
        the arm's own flange or tip."""
        notes: list[str] = []
        if arm is not None:
            group = self.scene.robot_of(robot).group(arm)
            link, where = (group.flange, "flange") if group.flange else (group.tip, "TCP")
        else:
            link, where = self._flange_link(robot), "flange"
            if link is None:
                try:
                    link = self.scene.robot_of(robot).tcp_link
                except ValueError:
                    link = None
                where = "TCP"
        targets: list[tuple[float, float, float]] = []
        for motion in self.project.get("motions") or []:
            owner = motion.get("robot") or self.default_robot
            if owner != robot:
                continue
            if arm is not None and motion.get("group") not in (None, arm):
                continue
            for segment in motion.get("segments") or []:
                q = segment.get("goal_positions")
                if not q:
                    continue
                if link is None:
                    notes.append(f"reach not derived — motion `{motion['name']}` is taught but the robot has no TCP link")
                    break
                try:
                    position, _ = self.scene.link_pose_at(link, list(q), robot=robot)
                except ValueError as e:
                    notes.append(f"reach skips motion `{motion['name']}`: {e}")
                    break
                targets.append((float(position[0]), float(position[1]), float(position[2])))
        return targets, where, notes

    def _flange_link(self, robot: str) -> Optional[str]:
        """The flange link a catalog robot declares (`None` for a plain URDF)."""
        for entry in self.project.get("robots") or []:
            if (entry.get("name") or self.default_robot) != robot:
                continue
            source = entry.get("source") or {}
            if source.get("kind") == "composite":
                return source.get("flange") or (source.get("base") or {}).get("flange")
            return source.get("flange")
        return None

    def node_capacity(self, node: str) -> dict[str, float]:
        """Declared channels of an I/O node, counted by kind (`di`, `do`,
        `ai`, `ao`, `safe_di`, `safe_do`)."""
        entry = self.nodes.get(node)
        if not entry:
            return {}
        counts: dict[str, float] = {}
        for channel in entry.get("channels") or []:
            kind = str(channel.get("kind") or "").lower()
            if kind in ("di", "do", "ai", "ao", "safe_di", "safe_do"):
                counts[kind] = counts.get(kind, 0.0) + 1.0
        return counts

    def tool_rows(self, robot: str) -> list[dict]:
        return [r for r in self.bom_rows if any(n.startswith(robot + "/tool") for n in r["names"])]

    def robot_row(self, robot: str) -> Optional[dict]:
        for r in self.bom_rows:
            if robot in r["names"]:
                return r
        return None

    # ------------------------------------------------------------- rules

    def _robot(self, name: str, margin: float) -> tuple[list[Requirement], list[str]]:
        # A BOM line is a whole robot, or one arm of a dual-arm robot
        # assembled from catalog arms (`robot/arm`): that arm's tools,
        # grasps and targets, measured from its own base.
        robot, arm = self.robot_and_arm(name)
        reqs: list[Requirement] = []
        notes: list[str] = []
        tool_mass, tool_known, has_tool = 0.0, True, False
        for row in self.tool_rows(name):
            has_tool = True
            m = _number((row.get("attributes") or {}).get("mass_kg"))
            if m is None:
                tool_known = False
            else:
                tool_mass += m * int(row.get("qty") or 1)
        heaviest, unknown = self._heaviest(self.grasped_by(robot, arm))
        if (has_tool and tool_known) or heaviest is not None:
            basis: list[str] = []
            if has_tool:
                basis.append(f"tool {_fmt(tool_mass)} kg" if tool_known else "tool mass unknown")
            if heaviest is not None:
                basis.append(f"grasps {heaviest[0]} {_fmt(heaviest[1])} kg")
            value = tool_mass + (heaviest[1] if heaviest is not None else 0.0)
            reqs.append(Requirement("payload_kg", _round(value, 3), basis=", ".join(basis)))
        if has_tool and not tool_known:
            notes.append("payload counts no tool mass — the tool has no mass_kg")
        if unknown:
            notes.append(f"payload counts no mass for {', '.join(unknown)} — give them mass_kg on set_part")
        # Reach is per arm: from the arm's own base, at its own tip. A
        # dual-arm product (one line, several arms) asks for the farthest
        # arm's figure; an arm mounted from the catalog is its own line.
        arms = [arm] if arm is not None else (self.arms_of(robot) or [None])
        farthest_arm: Optional[tuple[float, str, Optional[str]]] = None
        for a in arms:
            targets, where, reach_notes = self.targets_of(robot, a)
            notes += reach_notes
            if not targets:
                continue
            base = self._arm_base(robot, a)
            far = max(_dist(t, base) for t in targets)
            if farthest_arm is None or far > farthest_arm[0]:
                farthest_arm = (far, where, a)
        if farthest_arm is not None:
            far, where, a = farthest_arm
            whose = "the base" if a is None else f"the {a} arm's base"
            reqs.append(
                Requirement(
                    "reach_mm",
                    _round(far * 1000.0 * (1.0 + margin), 1),
                    basis=f"farthest taught target {far:.2f} m from {whose} ({where}), +{margin:.0%}",
                )
            )
        if arm is None and self.arms_of(robot):
            used = sorted(
                {
                    str(m.get("group"))
                    for m in self.project.get("motions") or []
                    if (m.get("robot") or self.default_robot) == robot and m.get("group")
                }
            )
            reqs.append(
                Requirement(
                    "arm_count",
                    float(len(used) or len(self.arms_of(robot))),
                    basis=f"arms taught: {', '.join(used)}" if used else "arms of the robot",
                )
            )
        # A machine tool in the cell asks for reach before anything is
        # taught: its table, through its side opening, from this base.
        # Only a machine within an arm's length of the base is this
        # robot's to serve (a second machine across the hall is not).
        base_pose = self._arm_base(robot, arm)
        for row in self.scene.bom().rows:
            category = str(row.get("category") or "")
            if category.startswith("machine_tool.vmc"):
                frame, what, through = "table", "the table", "its side opening"
            elif category.startswith("machine_tool.lathe"):
                frame, what, through = "spindle", "the spindle", "its front opening"
            else:
                continue
            machine = str((row.get("names") or [""])[0])
            target = self.scene.frames.get(f"{machine}/{frame}")
            if target is None:
                continue
            distance = _dist(target[0], base_pose)
            if distance > 3.0:
                continue
            reqs.append(
                Requirement(
                    "reach_mm",
                    _round(distance * 1000.0 * (1.0 + margin), 1),
                    basis=f"{what} of `{machine}` {distance:.2f} m from the base, through {through}, +{margin:.0%}",
                )
            )
        ridden = self.vehicle_of(robot)
        if ridden is not None:
            # The machine is the robot (legs, or a whole airframe): what the
            # cell asks of its vehicle lands on the same line its specs do.
            r, n = self._vehicle(ridden, self.devices[ridden], margin)
            reqs += r
            notes += n
        return reqs, notes

    def _arm_base(self, robot: str, arm: Optional[str]) -> tuple[float, float, float]:
        """Where reach is measured from: the robot's base, or an arm's
        base link (rigid on the body, so the current posture will do)."""
        if arm is None:
            return self.scene.robot_base_pose_of(robot)[0]
        group = self.scene.robot_of(robot).group(arm)
        return self.scene.link_pose(group.base, robot=robot)[0]

    def _tool(self, name: str, category: str) -> tuple[list[Requirement], list[str]]:
        robot = name.split("/")[0]
        grasped = self.grasped_by(robot)
        reqs: list[Requirement] = []
        notes: list[str] = []
        if category.startswith(("tool.screwdriver", "tool.nutrunner")):
            reqs += self._screwdriver()
        heaviest, unknown = self._heaviest(grasped)
        if heaviest is not None:
            reqs.append(Requirement("payload_kg", _round(heaviest[1], 3), basis=f"grasps {heaviest[0]} {_fmt(heaviest[1])} kg"))
        if unknown and heaviest is None:
            notes.append(f"payload not derived — {', '.join(unknown)} have no mass_kg")
        mechanical = category.startswith(("gripper.parallel", "gripper.multifinger"))
        if mechanical:
            # The part must fit in the opening: a parallel gripper's stroke,
            # a hand's aperture — same derivation, category-named key.
            widest: Optional[tuple[str, float]] = None
            for obj in grasped:
                extent = self.extent_of(obj)
                if extent is None:
                    continue
                side = min(extent)
                if widest is None or side > widest[1]:
                    widest = (obj, side)
            if widest is not None:
                key = (
                    "aperture_mm"
                    if category.startswith("gripper.multifinger")
                    else "stroke_mm"
                )
                reqs.append(
                    Requirement(
                        key,
                        _round(widest[1] * 1000.0, 1),
                        basis=f"smallest side of {widest[0]} ({widest[1] * 1000.0:.0f} mm)",
                    )
                )
        if mechanical and heaviest is not None:
            # Static holding force with the assumptions written down:
            # F × μ × surfaces ≥ m g × SF, at μ 0.5, two friction surfaces,
            # safety factor 2 (the baked timeline's grasp_report re-checks
            # with the cell's real μ and carry acceleration).
            required = heaviest[1] * 9.81 * 2.0 / (0.5 * 2.0)
            reqs.append(
                Requirement(
                    "grip_force_n",
                    _round(required, 1),
                    basis=(
                        f"holding {heaviest[0]} {_fmt(heaviest[1])} kg: "
                        "m·g × SF 2 / (μ 0.5 × 2 surfaces)"
                    ),
                )
            )
        return reqs, notes

    def _sensor(self, name: str, category: str) -> tuple[list[Requirement], list[str]]:
        kind = self.sensors.get(name)
        if not kind:
            return [], []
        reqs: list[Requirement] = []
        if kind.get("kind") == "beam":
            span = _dist(kind["from"], kind["to"])
            if category.startswith("sensor.light_curtain"):
                reqs.append(Requirement("range_mm", _round(span * 1000.0, 1), basis=f"beam span {span:.2f} m"))
                bounds = self.bounds_of_group(name)
                if bounds is not None:
                    height = bounds[1][2] - bounds[0][2]
                    reqs.append(
                        Requirement(
                            "protective_height_mm",
                            _round(height * 1000.0, 1),
                            basis=f"post height {height:.2f} m",
                        )
                    )
            else:
                reqs.append(Requirement("sensing_range_mm", _round(span * 1000.0, 1), basis=f"beam span {span:.2f} m"))
        elif kind.get("kind") == "zone":
            size = kind.get("size") or [0, 0, 0]
            half = 0.5 * math.hypot(float(size[0]), float(size[1]))
            reqs.append(
                Requirement(
                    "range_mm",
                    _round(half * 1000.0, 1),
                    basis=f"half-diagonal of the {size[0]:g} x {size[1]:g} m zone",
                )
            )
        return reqs, []

    def _camera(self, name: str) -> tuple[list[Requirement], list[str]]:
        """What the cell asks of a camera: the authored framing (fov,
        resolution) always; a working-distance band only when a vision
        sensor actually judges through it (a presentation-only camera has
        no range requirement — its far clip is a draw distance, not a
        spec). `min_range_mm` is a `<=` requirement, so like the other
        ceiling checks it gates `check` but stays out of `row.minimum`
        (design-camera.md §11 B5)."""
        camera = self.cameras.get(name)
        if not camera:
            return [], []
        reqs: list[Requirement] = [
            Requirement(
                "fov_deg",
                _round(float(camera.get("fov_deg") or 0.0), 2),
                basis="authored field of view",
            )
        ]
        resolution = camera.get("resolution") or [0, 0]
        for key, value in (("resolution_h_px", resolution[0]), ("resolution_v_px", resolution[1])):
            reqs.append(Requirement(key, float(value), basis="authored resolution"))
        bands = []
        for sensor_name, kind in self.sensors.items():
            if kind.get("kind") != "vision" or kind.get("camera") != name:
                continue
            band = kind.get("detect_range") or [camera.get("near"), camera.get("far")]
            bands.append((sensor_name, float(band[0]), float(band[1])))
        if bands:
            far_name, _, far_m = max(bands, key=lambda b: b[2])
            near_name, near_m, _ = min(bands, key=lambda b: b[1])
            reqs.append(
                Requirement(
                    "max_range_mm",
                    _round(far_m * 1000.0, 1),
                    basis=f"vision sensor `{far_name}` judges out to {far_m:g} m",
                )
            )
            reqs.append(
                Requirement(
                    "min_range_mm",
                    _round(near_m * 1000.0, 1),
                    op="<=",
                    basis=f"vision sensor `{near_name}` judges from {near_m:g} m",
                )
            )
        return reqs, []

    def _lidar(self, name: str) -> tuple[list[Requirement], list[str]]:
        """What the cell asks of a lidar: the authored sweep angle always;
        a measuring band — and `field_evaluation`, the product feature the
        judgement runs on — only when field sensors actually judge through
        it (a survey-only scanner has no range requirement — its authored
        max range is a scan reach, not a spec). `min_range_mm` is a `<=`
        requirement — a device with a bigger blind ring than authored
        would miss the close intrusions the sim detected — so it gates
        `check` but stays out of `row.minimum` (design-lidar.md 判断 L12,
        the camera rule applied to sweeps)."""
        lidar = self.lidars.get(name)
        if not lidar:
            return [], []
        reqs: list[Requirement] = [
            Requirement(
                "scan_fov_deg",
                _round(float(lidar.get("fov_deg") or 0.0), 2),
                basis="authored sweep angle",
            )
        ]
        channels = int(lidar.get("channels") or 1)
        if channels > 1:
            # A 3D sweep was authored: fewer rings or a narrower vertical
            # field could not reproduce the scans the sim analyzed.
            reqs.append(
                Requirement("channels", channels, basis="authored scan rings")
            )
            reqs.append(
                Requirement(
                    "vfov_deg",
                    _round(float(lidar.get("vfov_deg") or 0.0), 2),
                    basis="authored vertical field",
                )
            )
        band = lidar.get("range") or [0.0, 0.0]
        radii = []
        for sensor_name, kind in self.sensors.items():
            if kind.get("kind") != "field" or kind.get("lidar") != name:
                continue
            radius = kind.get("range")
            radii.append((sensor_name, float(radius if radius is not None else band[1])))
        if radii:
            far_name, far_m = max(radii, key=lambda r: r[1])
            # Field evaluation is a product feature, not a given: a 3D
            # perception lidar measures the same distances but carries no
            # field engine — a cell judging through fields can only be
            # built from a device that evaluates them (measurement-grade
            # like an LMS1xx, or a safety scanner).
            reqs.append(
                Requirement(
                    "field_evaluation",
                    1,
                    basis="field sensors judge through it — the device must evaluate fields",
                )
            )
            reqs.append(
                Requirement(
                    "max_range_mm",
                    _round(far_m * 1000.0, 1),
                    basis=f"field sensor `{far_name}` sweeps out to {far_m:g} m",
                )
            )
            reqs.append(
                Requirement(
                    "min_range_mm",
                    _round(float(band[0]) * 1000.0, 1),
                    op="<=",
                    basis="authored blind ring — a bigger one would miss close intrusions",
                )
            )
        return reqs, []

    def _device(self, name: str, margin: float) -> tuple[list[Requirement], list[str]]:
        kind = self.devices.get(name)
        if not kind:
            return [], []
        reqs: list[Requirement] = []
        notes: list[str] = []
        k = kind.get("kind")
        if k == "conveyor":
            pose = kind["zone_pose"]
            size = [float(v) for v in kind["zone_size"]]
            velocity = [float(v) for v in kind["velocity"]]
            speed = _norm(velocity)
            direction = _unit(velocity) if speed > _EPS else (1.0, 0.0, 0.0)
            q = pose["quaternion"]
            along = _extent_along(size, _rotate_inverse(q, direction))
            across_dir = _cross((0.0, 0.0, 1.0), direction)
            if _norm(across_dir) < _EPS:
                across_dir = (0.0, 1.0, 0.0)
            across = _extent_along(size, _rotate_inverse(q, _unit(across_dir)))
            reqs.append(Requirement("length_mm", _round(along * 1000.0, 1), basis="zone length along the belt"))
            reqs.append(Requirement("width_mm", _round(across * 1000.0, 1), basis="zone width across the belt"))
            if speed > _EPS:
                reqs.append(Requirement("speed_mps", _round(speed, 3), basis="belt speed"))
            carried = self._objects_in(pose["position"], q, size)
            load, unknown = self._total_mass(carried)
            if len(carried) > len(unknown):
                reqs.append(
                    Requirement("load_kg", _round(load, 3), basis=f"{len(carried)} part(s) on the belt at start")
                )
            if unknown:
                notes.append(f"load counts no mass for {', '.join(unknown)} — give them mass_kg on set_part")
        elif k == "linear_axis":
            lo, hi = kind.get("range") or [0.0, 0.0]
            reqs.append(Requirement("stroke_mm", _round((float(hi) - float(lo)) * 1000.0, 1), basis="axis range"))
            speed = float(kind.get("speed") or 0.0)
            if speed > _EPS:
                reqs.append(Requirement("speed_mps", _round(speed, 3), basis="axis speed"))
            objects = list(kind.get("objects") or [])
            load, unknown = self._total_mass(objects)
            if len(objects) > len(unknown):
                reqs.append(Requirement("load_kg", _round(load, 3), basis=f"carries {', '.join(objects)}"))
            if unknown:
                notes.append(f"load counts no mass for {', '.join(unknown)} — give them mass_kg on set_part")
        elif k == "vehicle":
            r, n = self._vehicle(name, kind, margin)
            reqs += r
            notes += n
        return reqs, notes

    def _vehicle(self, name: str, kind: dict, margin: float) -> tuple[list[Requirement], list[str]]:
        reqs: list[Requirement] = []
        notes: list[str] = []
        speed = float(kind.get("speed") or 0.0)
        if speed > _EPS:
            reqs.append(Requirement("max_speed_mps", _round(speed, 3), basis="travel speed"))
        aerial = kind.get("aerial")
        if aerial:
            reqs.append(Requirement("max_climb_mps", _round(float(aerial["climb_speed"]), 3), basis="climb rate"))
            reqs.append(
                Requirement("max_descent_mps", _round(float(aerial["descent_speed"]), 3), basis="descent rate")
            )
            # Flight time is a cycle fact, so it needs the baked cycle: the
            # airborne seconds are read exactly off the vehicle's closed-form
            # track — every moving span, plus every hover above the starting
            # pad (waiting *on* the pad costs nothing). This is a comparison
            # against the declared hover endurance, not a battery model.
            if self.timeline is None:
                notes.append("flight time not compared — bake the cycle and pass requirements(timeline=tl)")
            else:
                try:
                    airborne = float(self.timeline.vehicle_airborne(name))
                except ValueError as e:
                    airborne = 0.0
                    notes.append(f"flight time not compared — {e}")
                if airborne > _EPS:
                    reqs.append(
                        Requirement(
                            "flight_time_min",
                            _round(airborne * (1.0 + margin) / 60.0, 2),
                            basis=f"airborne {airborne:.1f} s per cycle, +{margin:.0%}",
                        )
                    )
        tray = kind.get("tray")
        frame = self._parked_frame(kind) if tray else None
        if frame is not None:
            position, quaternion = frame
            pose = tray["pose"]
            offset = _rotate(quaternion, pose["position"])
            zone_position = tuple(position[i] + offset[i] for i in range(3))
            zone_quaternion = _quat_mul(quaternion, pose["quaternion"])
            body = set(kind.get("body") or [])
            carried = [o for o in self._objects_in(zone_position, zone_quaternion, tray["size"]) if o not in body]
            load, unknown = self._total_mass(carried)
            if len(carried) > len(unknown):
                reqs.append(
                    Requirement("payload_kg", _round(load, 3), basis=f"{len(carried)} part(s) on the deck at start")
                )
            if unknown:
                notes.append(f"deck load counts no mass for {', '.join(unknown)} — give them mass_kg on set_part")
        return reqs, notes

    def _parked_frame(self, kind: dict) -> Optional[tuple[tuple[float, float, float], tuple[float, float, float, float]]]:
        """World pose of the vehicle parked at its start station — a mirror
        of `VehiclePath::frame_at` (the studio keeps the same mirror in TS):
        the heading faces the leg leaving the waypoint (wrapping on a ring),
        or the leg arriving when nothing leaves."""
        path = kind.get("path") or {}
        waypoints = [(list(w) + [0.0])[:3] for w in path.get("waypoints") or []]
        stations = {s["name"]: int(s["index"]) for s in path.get("stations") or []}
        at = stations.get(kind.get("start"))
        n = len(waypoints)
        if at is None or not 0 <= at < n:
            return None
        ring = bool(path.get("ring"))

        def direction(i: int, j: int) -> Optional[float]:
            dx = waypoints[j][0] - waypoints[i][0]
            dy = waypoints[j][1] - waypoints[i][1]
            # Heading is about +Z: only the horizontal run can set it.
            return math.atan2(dy, dx) if math.hypot(dx, dy) > 1e-9 else None

        heading: Optional[float] = None
        for step in range(1, n):
            if ring:
                j = (at + step) % n
            elif at + step < n:
                j = at + step
            else:
                break
            heading = direction(at, j)
            if heading is not None:
                break
        if heading is None:
            for step in range(1, n):
                if ring:
                    j = (at + n - (step % n)) % n
                elif step <= at:
                    j = at - step
                else:
                    break
                heading = direction(j, at)
                if heading is not None:
                    break
        if heading is None:
            heading = 0.0
        p = waypoints[at]
        q = (0.0, 0.0, math.sin(heading / 2.0), math.cos(heading / 2.0))
        return (float(p[0]), float(p[1]), float(p[2])), q

    def _node(self, name: str) -> tuple[list[Requirement], list[str]]:
        # A derived controller line (`<robot>/controller`) owns the points
        # the derivation put on the arm's implicit host, `<robot>`.
        head, _, tail = name.rpartition("/")
        hosts = {name, f"<{head}>"} if tail == "controller" and head in self.robots else {name}
        counts: dict[str, int] = {}
        for point in self.points:
            status = point.get("status")
            on_node = point.get("node") == name and status == "bound"
            hosted = status != "bound" and point.get("host") in hosts
            if not (on_node or hosted):
                continue
            key = str(point.get("kind") or "").lower()
            if key not in ("di", "do", "ai", "ao"):
                continue
            if point.get("safety"):
                key = "safe_" + key
            counts[key] = counts.get(key, 0) + 1
        reqs = [
            Requirement(key, float(n), basis=f"{n} point(s) assigned to this node", lint=False)
            for key, n in sorted(counts.items())
        ]
        return reqs, []

    def _controller(self, name: str) -> tuple[list[Requirement], list[str]]:
        """The robot cable a placed controller needs: from each arm's base to
        the box along the axes — a cable runs in a duct or a trench, not as
        the crow flies — plus the slack, the farthest arm's figure. A
        controller without a `place` gets no figure (`controller_unplaced`
        says so); a `place` that names nothing in the scene is a note."""
        node = self.nodes.get(name)
        kind = (node or {}).get("kind") or {}
        if not node or kind.get("kind") != "robot_controller":
            return [], []
        place = node.get("place")
        if not place:
            return [], []
        box = self._place_origin(place)
        if box is None:
            return [], [f"cable length needs the box — place {place!r} is not an obstacle or a frame of this scene"]
        farthest: Optional[tuple[float, str]] = None
        for robot in kind.get("robots") or []:
            if robot not in self.robots:
                continue
            base = self.scene.robot_base_pose_of(robot)[0]
            run = sum(abs(float(b) - float(p)) for b, p in zip(base, box))
            if farthest is None or run > farthest[0]:
                farthest = (run, robot)
        if farthest is None:
            return [], []
        run, robot = farthest
        return [
            Requirement(
                "cable_m",
                _round(run + self.cable_slack_m, 2),
                basis=f"{robot} base to {place} {run:.2f} m along the axes + {_fmt(self.cable_slack_m)} m slack",
            )
        ], []

    def _place_origin(self, place: str) -> Optional[tuple[float, float, float]]:
        entry = self.obstacles.get(place)
        if entry is not None:
            p = (entry.get("pose") or {}).get("position")
            if p is not None:
                return (float(p[0]), float(p[1]), float(p[2]))
        if place in self.scene.frames:
            return tuple(self.scene.frame(place)[0])  # type: ignore[return-value]
        return None

    def _structure(self, name: str, category: str) -> tuple[list[Requirement], list[str]]:
        reqs: list[Requirement] = []
        notes: list[str] = []
        if category.startswith(("structure.pedestal", "structure.table")):
            bounds = self.bounds_of_group(name)
            if bounds is not None:
                (x0, y0, _), (x1, y1, top) = bounds
                load = 0.0
                standing: list[str] = []
                for robot in self.robots:
                    base = self.scene.robot_base_pose_of(robot)[0]
                    if x0 - 0.02 <= base[0] <= x1 + 0.02 and y0 - 0.02 <= base[1] <= y1 + 0.02 and abs(base[2] - top) <= 0.05:
                        standing.append(robot)
                        row = self.robot_row(robot)
                        m = _number((row.get("attributes") or {}).get("mass_kg")) if row else None
                        if m is None:
                            notes.append(f"load counts no mass for {robot} — it has no mass_kg")
                        else:
                            load += m
                        for tool in self.tool_rows(robot):
                            tm = _number((tool.get("attributes") or {}).get("mass_kg"))
                            if tm is not None:
                                load += tm * int(tool.get("qty") or 1)
                if standing:
                    reqs.append(Requirement("load_kg", _round(load, 3), basis=f"{', '.join(standing)} standing on it"))
        elif category == "power_supply":
            # Each supply must cover the loads at its own voltage: a part's
            # `voltage_v` against the supply's `output_v` (the I/O lint's
            # 0.5 V tolerance). A load that does not say its voltage counts
            # against every supply, and a supply that does not say its
            # voltage counts every load — conservative, never silent.
            supply_row = next((r for r in self.bom_rows if name in r["names"]), None)
            supply_v = _number(((supply_row or {}).get("attributes") or {}).get("output_v"))
            units = int((supply_row or {}).get("qty") or 1)
            total = 0.0
            lines = 0
            unvoiced: list[str] = []
            peers: list[str] = []
            for row in self.bom_rows:
                if name in row["names"]:
                    continue
                attrs = row.get("attributes") or {}
                if supply_v is not None and str(row.get("category") or "") == "power_supply":
                    peer_v = _number(attrs.get("output_v"))
                    if peer_v is not None and abs(peer_v - supply_v) <= 0.5:
                        peers.extend(row["names"])
                current = _number(attrs.get("current_a"))
                if current is None:
                    continue
                load_v = _number(attrs.get("voltage_v"))
                if supply_v is not None and load_v is not None and abs(load_v - supply_v) > 0.5:
                    continue
                if supply_v is not None and load_v is None:
                    unvoiced.extend(row["names"])
                total += current * int(row.get("qty") or 1)
                lines += 1
            if lines:
                at = f" at {_fmt(supply_v)} V" if supply_v is not None else ""
                reqs.append(Requirement("output_a", _round(total, 3), basis=f"sum of current_a over {lines} line(s){at}"))
            if unvoiced:
                notes.append(f"current counts {', '.join(unvoiced)} with no voltage_v — give them voltage_v on set_part to keep them off the other supplies")
            if units > 1:
                notes.append(f"{units} units — the same loads are counted against each unit")
            if peers:
                notes.append(f"{', '.join(peers)} also supplies {_fmt(supply_v)} V — the same loads are counted against each")
        return reqs, notes

    # ----------------------------------------------------------- helpers

    def _heaviest(self, objects: list[str]) -> tuple[Optional[tuple[str, float]], list[str]]:
        heaviest: Optional[tuple[str, float]] = None
        unknown: list[str] = []
        for obj in objects:
            m = self.mass_of(obj)
            if m is None:
                unknown.append(obj)
            elif heaviest is None or m > heaviest[1]:
                heaviest = (obj, m)
        return heaviest, unknown

    def _total_mass(self, objects: list[str]) -> tuple[float, list[str]]:
        total = 0.0
        unknown: list[str] = []
        for obj in objects:
            m = self.mass_of(obj)
            if m is None:
                unknown.append(obj)
            else:
                total += m
        return total, unknown

    def obstacles_meeting(self, position, quaternion, size, *, exclude: str = "", ground_z: float = 0.02) -> list[str]:
        """Solid obstacles whose bounds overlap a yaw-oriented box: not the
        ones under `exclude` (the cabinet's own parts), not what rides a
        robot, not decoration (`enabled=False`), not the floor."""
        hits: list[str] = []
        for obstacle, entry in self.obstacles.items():
            if (exclude and obstacle.startswith(exclude)) or entry.get("attached_to"):
                continue
            if entry.get("enabled") is False:
                continue
            try:
                lo, hi = self.scene.obstacle_bounds(obstacle)
            except ValueError:
                continue
            if hi[2] <= ground_z:
                continue
            if _box_meets_aabb(position, quaternion, size, lo, hi):
                hits.append(obstacle)
        return hits

    def _objects_in(self, position, quaternion, size) -> list[str]:
        """Free obstacles whose origin lies inside an oriented box."""
        names: list[str] = []
        for obstacle, entry in self.obstacles.items():
            if entry.get("attached_to"):
                continue
            p = entry.get("pose", {}).get("position")
            if p is None:
                continue
            local = _rotate_inverse(quaternion, (p[0] - position[0], p[1] - position[1], p[2] - position[2]))
            if all(abs(local[i]) <= float(size[i]) / 2.0 + 1e-6 for i in range(3)):
                names.append(obstacle)
        return names


# ------------------------------------------------------------------ utils


def _kind_hint(category: str) -> str:
    if category.startswith(("conveyor", "axis", "vehicle", "machine_tool.door", "feeder")):
        return "device"
    if category.startswith("hmi.button"):
        return "sensor"
    if category == "sensor.camera":
        return "camera"
    if category == "sensor.lidar":
        return "lidar"
    if category.startswith("sensor"):
        return "sensor"
    if category in ("plc", "plc.safety", "io.remote", "robot_controller") or category.startswith("io."):
        return "io_node"
    return "group"


def _walk_actions(steps: list[dict]) -> Iterator[dict]:
    for step in steps:
        yield from step.get("actions") or []
        for arm in step.get("select") or []:
            yield from _walk_actions(arm.get("steps") or [])


def _merge(reqs: list[Requirement]) -> list[Requirement]:
    """Same key from several residents of one line -> the strictest value."""
    merged: dict[str, Requirement] = {}
    order: list[str] = []
    for r in reqs:
        if r.key not in merged:
            merged[r.key] = Requirement(r.key, r.value, r.op, r.basis, lint=r.lint)
            order.append(r.key)
            continue
        m = merged[r.key]
        if r.op == ">=" and r.value > m.value:
            m.value, m.basis = r.value, r.basis
        elif r.op == "==" and r.basis not in m.basis:
            m.basis = f"{m.basis} / {r.basis}"
    return [merged[k] for k in order]


def _provided(attributes: dict[str, Any], key: str) -> tuple[Optional[str], Optional[float]]:
    for alias in ALIASES.get(key, (key,)):
        value = _number(attributes.get(alias))
        if value is not None:
            return alias, value
    return None, None


def _satisfies(r: Requirement, provided: float) -> bool:
    if r.op == ">=":
        return provided + _EPS >= r.value
    if r.op == "<=":
        return provided - _EPS <= r.value
    return abs(provided - r.value) <= _EPS


def _number(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _round(value: float, digits: int) -> float:
    return float(round(value, digits))


def _fmt(value: Optional[float]) -> str:
    if value is None:
        return "?"
    if abs(value - round(value)) < 1e-9:
        return str(round(value))
    return f"{value:.3g}" if abs(value) < 1 else f"{value:.4g}"


def _dedupe(items: list[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for item in items:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


def _dist(a, b) -> float:
    return math.sqrt(sum((float(a[i]) - float(b[i])) ** 2 for i in range(3)))


def _norm(v) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in v))


def _unit(v) -> tuple[float, float, float]:
    n = _norm(v)
    return (float(v[0]) / n, float(v[1]) / n, float(v[2]) / n)


def _cross(a, b) -> tuple[float, float, float]:
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _rotate(q, v) -> tuple[float, float, float]:
    """Rotate `v` by the unit quaternion `q = (x, y, z, w)`."""
    x, y, z, w = (float(c) for c in q)
    vx, vy, vz = (float(c) for c in v)
    # t = 2 q_vec x v ; v' = v + w t + q_vec x t
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _rotate_inverse(q, v) -> tuple[float, float, float]:
    x, y, z, w = (float(c) for c in q)
    return _rotate((-x, -y, -z, w), v)


def _box_meets_aabb(centre, q, size, lo, hi) -> bool:
    """Whether a box (`size` = width, depth, height, turned by the yaw of
    `q` about `centre`) overlaps a world-aligned box — the separating-axis
    test on the four plan axes plus the height interval. Faces that merely
    touch do not count."""
    eps = 1e-6
    cx, cy, cz = (float(c) for c in centre)
    w, d, h = (float(c) for c in size)
    if cz - h / 2.0 >= float(hi[2]) - eps or cz + h / 2.0 <= float(lo[2]) + eps:
        return False
    ux = _rotate(q, (1.0, 0.0, 0.0))
    uy = _rotate(q, (0.0, 1.0, 0.0))
    axes = ((ux[0], ux[1]), (uy[0], uy[1]))
    halves = (w / 2.0, d / 2.0)
    corners = [
        (cx + sx * halves[0] * axes[0][0] + sy * halves[1] * axes[1][0],
         cy + sx * halves[0] * axes[0][1] + sy * halves[1] * axes[1][1])
        for sx in (-1.0, 1.0) for sy in (-1.0, 1.0)
    ]
    for i in (0, 1):
        if max(c[i] for c in corners) <= float(lo[i]) + eps or min(c[i] for c in corners) >= float(hi[i]) - eps:
            return False
    aabb = [(float(lo[0]), float(lo[1])), (float(hi[0]), float(lo[1])),
            (float(lo[0]), float(hi[1])), (float(hi[0]), float(hi[1]))]
    for axis, half in zip(axes, halves):
        c0 = cx * axis[0] + cy * axis[1]
        proj = [p[0] * axis[0] + p[1] * axis[1] for p in aabb]
        if max(proj) <= c0 - half + eps or min(proj) >= c0 + half - eps:
            return False
    return True


def _quat_mul(a, b) -> tuple[float, float, float, float]:
    """Compose unit quaternions `a ∘ b`, both `(x, y, z, w)`."""
    ax, ay, az, aw = (float(c) for c in a)
    bx, by, bz, bw = (float(c) for c in b)
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _extent_along(size, local_dir) -> float:
    """Extent of a box of `size` measured along a unit direction given in its own frame."""
    return sum(abs(float(local_dir[i])) * float(size[i]) for i in range(3))
