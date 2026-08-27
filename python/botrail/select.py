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
| `reach_mm`             | the farthest taught target from the base, plus a margin    | `reach_mm`                                   |
| `stroke_mm`            | the smallest side of the grasped parts (parallel gripper)  | `stroke_mm`, `opening_mm`                    |
| `sensing_range_mm`     | a beam sensor's span                                       | `sensing_range_mm`, `range_mm`, `max_range_mm` |
| `range_mm`             | a light curtain's span / an area sensor's half-diagonal    | `range_mm`, `max_range_mm`, `sensing_range_mm` |
| `protective_height_mm` | a light curtain's post height                              | `protective_height_mm`, `height_mm`          |
| `length_mm`, `width_mm`| a conveyor's zone along and across its belt                | `length_mm` / `width_mm`, `belt_width_mm`    |
| `speed_mps`            | a conveyor's belt speed, an axis speed                     | `max_speed_mps`, `speed_max_mps`, `speed_mps`|
| `max_speed_mps`        | a vehicle's travel speed                                   | the same                                     |
| `max_climb_mps`        | an aerial vehicle's climb rate                             | the same                                     |
| `max_descent_mps`      | an aerial vehicle's descent rate                           | the same                                     |
| `flight_time_min`      | an aerial vehicle's airborne time per cycle, from the baked timeline (`requirements(timeline=tl)`) | the same |
| `load_kg`              | parts on a conveyor / an axis; robots standing on a pedestal | `load_kg`, `capacity_kg`, `max_load_kg`, `payload_kg` |
| `output_a`             | the sum of `current_a` over the other lines (power supply) | `output_a`, `current_a`                      |
| `di` `do` `ai` `ao` `safe_di` `safe_do` | points assigned to an I/O node                | the node's declared channels                 |

Every requirement is a minimum (`>=`) unless noted. The derivations are
geometric and deterministic — no sizing, no safety evaluation; a value that
cannot be derived (a grasped part without `mass_kg`) is reported as a note,
never guessed.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator, Optional, Union

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
    "stroke_mm": ("stroke_mm", "opening_mm"),
    "sensing_range_mm": ("sensing_range_mm", "range_mm", "max_range_mm"),
    "fov_deg": ("fov_h_deg", "hfov_deg", "fov_deg"),
    "resolution_h_px": ("resolution_h_px",),
    "resolution_v_px": ("resolution_v_px",),
    "max_range_mm": ("max_range_mm",),
    "min_range_mm": ("min_range_mm",),
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
    "output_a": ("output_a", "current_a"),
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
        `requirement_incomplete` (info), in row order. I/O nodes get no
        spec findings — the I/O report already lints their capacity."""
        out: list[Finding] = []
        for row in self.rows:
            if row.kind != "io_node":
                for r in row.requirements:
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
    scene, *, sequences: Optional[list[str]] = None, margin: float = 0.1, timeline=None
) -> Requirements:
    """Derive what every BOM line must be able to do from the cell it is in,
    and compare it with what the chosen part says (its catalog specs or the
    attributes typed on `set_part`).

    `sequences` limits the programs whose grasps and I/O points are counted
    (default: all). `margin` is added to the reach and flight-time
    requirements (0.1 = 10 %). `timeline` is an optional baked
    `simulate_sequences` result: cycle facts only it can supply — an aerial
    vehicle's airborne time — are derived from it, and left as a note when
    it is absent. Nothing is chosen and nothing is sized: a number the cell
    cannot supply (a grasped part with no `mass_kg`) becomes a note, not a
    guess.
    """
    if margin < 0:
        raise ValueError("margin must be >= 0")
    cell = _Cell(scene, sequences, timeline)
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


def check(scene, *, sequences: Optional[list[str]] = None, timeline=None) -> CheckReport:
    """Every static check in one report: the I/O lint, each sequence walked
    for dangling references, unidentified equipment lines (with what the
    cell asks of them) and the requirement comparison. `timeline` (a baked
    cycle) adds the cycle-fact requirements — an aerial vehicle's flight
    time. Errors make `ok` false; `botrail check` prints exactly this."""
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
    req = requirements(scene, sequences=sequences, timeline=timeline)
    unidentified = {tuple(r["names"]) for r in scene.bom().unidentified()}
    for row in req.rows:
        if tuple(row.names) in unidentified:
            message = f"{', '.join(row.names)} ({row.category}) has no maker, model or catalog reference"
            if row.requirements:
                message += " — needs " + ", ".join(str(r) for r in row.requirements)
            findings.append(Finding("info", "unidentified_part", message, row.target))
    findings += req.findings()
    return CheckReport(findings, req)


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
        if kind == "device":
            return self._device(name, margin)
        if kind == "io_node":
            return self._node(name)
        return self._structure(name, category)

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

    def grasped_by(self, robot: str) -> list[str]:
        """Objects the robot holds now or grasps in a counted sequence."""
        if robot in self._grasps:
            return self._grasps[robot]
        names: list[str] = []
        for obstacle, entry in self.obstacles.items():
            attached = entry.get("attached_to")
            if attached and (attached.get("robot") or self.default_robot) == robot:
                names.append(obstacle)
        for sequence in self.sequences:
            for action in _walk_actions(sequence.get("steps") or []):
                if action.get("type") == "attach" and (action.get("robot") or self.default_robot) == robot:
                    obj = action.get("object")
                    if obj and obj not in names:
                        names.append(obj)
        self._grasps[robot] = names
        return names

    def targets_of(self, robot: str) -> tuple[list[tuple[float, float, float]], str, list[str]]:
        """Positions of every taught segment goal of the robot's motions,
        measured at the flange when the robot declares one (a reach spec
        is quoted to the flange, not past the tool), else at the TCP."""
        notes: list[str] = []
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
        heaviest, unknown = self._heaviest(self.grasped_by(name))
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
        targets, where, reach_notes = self.targets_of(name)
        notes += reach_notes
        if targets:
            base = self.scene.robot_base_pose_of(name)[0]
            farthest = max(_dist(t, base) for t in targets)
            reqs.append(
                Requirement(
                    "reach_mm",
                    _round(farthest * 1000.0 * (1.0 + margin), 1),
                    basis=f"farthest taught target {farthest:.2f} m from the base ({where}), +{margin:.0%}",
                )
            )
        ridden = self.vehicle_of(name)
        if ridden is not None:
            # The machine is the robot (legs, or a whole airframe): what the
            # cell asks of its vehicle lands on the same line its specs do.
            r, n = self._vehicle(ridden, self.devices[ridden], margin)
            reqs += r
            notes += n
        return reqs, notes

    def _tool(self, name: str, category: str) -> tuple[list[Requirement], list[str]]:
        robot = name.rpartition("/")[0]
        grasped = self.grasped_by(robot)
        reqs: list[Requirement] = []
        notes: list[str] = []
        heaviest, unknown = self._heaviest(grasped)
        if heaviest is not None:
            reqs.append(Requirement("payload_kg", _round(heaviest[1], 3), basis=f"grasps {heaviest[0]} {_fmt(heaviest[1])} kg"))
        if unknown and heaviest is None:
            notes.append(f"payload not derived — {', '.join(unknown)} have no mass_kg")
        if category.startswith("gripper.parallel"):
            widest: Optional[tuple[str, float]] = None
            for obj in grasped:
                extent = self.extent_of(obj)
                if extent is None:
                    continue
                side = min(extent)
                if widest is None or side > widest[1]:
                    widest = (obj, side)
            if widest is not None:
                reqs.append(
                    Requirement(
                        "stroke_mm",
                        _round(widest[1] * 1000.0, 1),
                        basis=f"smallest side of {widest[0]} ({widest[1] * 1000.0:.0f} mm)",
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
        counts: dict[str, int] = {}
        for point in self.points:
            status = point.get("status")
            on_node = point.get("node") == name and status == "bound"
            hosted = status != "bound" and point.get("host") == name
            if not (on_node or hosted):
                continue
            key = str(point.get("kind") or "").lower()
            if key not in ("di", "do", "ai", "ao"):
                continue
            if point.get("safety"):
                key = "safe_" + key
            counts[key] = counts.get(key, 0) + 1
        reqs = [
            Requirement(key, float(n), basis=f"{n} point(s) assigned to this node")
            for key, n in sorted(counts.items())
        ]
        return reqs, []

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
            total = 0.0
            lines = 0
            for row in self.bom_rows:
                if name in row["names"]:
                    continue
                current = _number((row.get("attributes") or {}).get("current_a"))
                if current is not None:
                    total += current * int(row.get("qty") or 1)
                    lines += 1
            if lines:
                reqs.append(Requirement("output_a", _round(total, 3), basis=f"sum of current_a over {lines} line(s)"))
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
    if category.startswith(("conveyor", "axis", "vehicle")):
        return "device"
    if category == "sensor.camera":
        return "camera"
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
            merged[r.key] = Requirement(r.key, r.value, r.op, r.basis)
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
