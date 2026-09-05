"""Design information review, using the existing static checks and cell report.

``bt.review(scene)`` does not simulate, infer missing specifications, or change
``scene.check().ok``. It lists what can be judged, what is missing, and what
has not been run. ``ready`` refers only to the displayed review scope.
"""

from __future__ import annotations

import json
import math
from collections.abc import Iterable, Mapping
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any

from . import select

STATUSES = ("pass", "fail", "unknown", "not_run", "not_applicable")
GROUPS = ("checks", "equipment", "specifications", "connections", "totals", "simulation", "scenarios", "deliverables")
STAGES = {
    "concept": ("checks",),
    "design": ("checks", "equipment", "specifications", "connections", "simulation"),
}
SOURCE_KINDS = ("manufacturer", "user_input", "assumption", "derived", "measured", "unknown")


@dataclass
class ReviewItem:
    """One observation; an explicit failure blocks every review stage."""

    id: str
    group: str
    target: str
    status: str
    message: str
    basis: str
    next_action: str = ""
    evidence: dict[str, Any] = field(default_factory=dict)
    required: bool = False
    annotation: dict[str, str] = field(default_factory=dict)

    @property
    def blocking(self) -> bool:
        return self.status == "fail" or (self.required and self.status in ("unknown", "not_run"))

    def to_dict(self) -> dict[str, Any]:
        return {**asdict(self), "blocking": self.blocking}


class ReviewReport:
    """Review observations, known subtotals and unresolved required items.

    The original ``check`` and optional ``cell_report`` are included as JSON
    snapshots. A supplied cell report is evidence supplied by the caller;
    its correspondence to the current scene is not authenticated here.
    """

    def __init__(self, *, stage: str, required: list[str], items: list[ReviewItem],
                 totals: dict[str, dict[str, Any]], check: dict[str, Any], cell_report: dict[str, Any] | None):
        self.stage = stage
        self.required = required
        self.items = items
        self.totals = totals
        self.check = check
        self.cell_report = cell_report

    @property
    def ready(self) -> bool:
        return not self.blockers()

    def blockers(self) -> list[ReviewItem]:
        return [item for item in self.items if item.blocking]

    @property
    def counts(self) -> dict[str, int]:
        return {status: sum(item.status == status for item in self.items) for status in STATUSES}

    def to_dict(self) -> dict[str, Any]:
        return {
            "stage": self.stage,
            "ready": self.ready,
            "required": list(self.required),
            "counts": self.counts,
            "items": [item.to_dict() for item in self.items],
            "totals": self.totals,
            "check": self.check,
            "cell_report": self.cell_report,
        }

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False, allow_nan=False)

    def to_markdown(self) -> str:
        lines = [
            f"# Design information review — {self.stage}", "",
            (f"Review scope: **{'ready' if self.ready else 'unresolved'}**. "
             f"Static check: {'ok' if self.check['ok'] else 'FAIL'}."), "",
            "Required groups / items: " + ", ".join(self.required) + ". Explicit failures always block.",
            "Readiness covers these observations only; it is not approval of the complete cell design.", "",
            ", ".join(f"{status}: {count}" for status, count in self.counts.items()), "",
            "| item | target | result | required | observation / basis | next action |",
            "|---|---|---|---|---|---|",
        ]
        for item in self.items:
            annotation = "; ".join(f"{k}: {v}" for k, v in item.annotation.items())
            observation = f"{item.message} — {item.basis}"
            if annotation:
                observation += f"; {annotation}"
            lines.append("| " + " | ".join(_md(v) for v in (
                item.id, item.target, item.status, "yes" if item.required else "no", observation, item.next_action,
            )) + " |")
        if self.totals:
            lines += ["", "## Known subtotals", "",
                      "| attribute | known subtotal | known / target quantity | missing targets |",
                      "|---|---|---|---|"]
            for key, total in self.totals.items():
                missing = "; ".join(
                    f"{', '.join(row['names'])} (x{row['qty']}: {row['reason']})" for row in total["missing"]
                )
                lines.append("| " + " | ".join(_md(v) for v in (
                    key, "unknown" if total["known_subtotal"] is None else f"{total['known_subtotal']:g}",
                    f"{total['known_qty']} / {total['target_qty']}", missing,
                )) + " |")
        return "\n".join(lines) + "\n"

    def save(self, path: str | Path, format: str | None = None) -> None:
        path = Path(path)
        fmt = (format or path.suffix.lstrip(".")).lower()
        if fmt == "json":
            text = self.to_json()
        elif fmt in ("md", "markdown"):
            text = self.to_markdown()
        else:
            raise ValueError("review format must be md or json")
        path.write_text(text, encoding="utf-8")


def review(
    scene, *, report=None, manifest: str | Path | None = None,
    sequences: list[str] | None = None, stage: str = "concept",
    required: Iterable[str] | None = None,
    totals: Mapping[str, Iterable[str] | None] | None = None,
    annotations: Mapping[str, Mapping[str, str]] | None = None,
) -> ReviewReport:
    """Review the design information available for a cell, without baking it.

    ``report`` is an optional ``Scene.cell_report()`` result. Alternatively,
    ``manifest`` verifies an ``export_cell`` package against this cell and
    uses its report and program scope. Invalid packages block the review;
    external attachments and export warnings remain unresolved.
    ``sequences``
    selects the programs being reviewed (default all). ``stage='concept'``
    permits unresolved information outside static checks; ``'design'`` also
    requires equipment identity, specifications, connections and simulation.
    An explicit failure blocks either stage, even in an optional group.

    ``required`` adds group names or exact item IDs to that stage's scope.
    ``totals`` selects numeric attributes and BOM names to count, e.g.
    ``{'current_a': ['valve', 'eye']}``; ``None`` as a selection means the
    whole BOM. By default only whole-BOM ``mass_kg`` is counted. Supply
    capacity and power consumption are not inferred from these subtotals.

    ``annotations`` is keyed by item ID. It accepts ``source_kind`` (one of
    manufacturer/user_input/assumption/derived/measured/unknown), ``reference``,
    ``assumptions``, ``owner``, ``due``, ``next_action`` and ``not_applicable``
    (a nonempty reason). Manufacturer/measured sources require a reference.
    An annotation documents an input; it never fills a missing value or
    turns a failed comparison into a pass. An explicit failure cannot be
    marked not applicable. Unknown item IDs are rejected.
    """
    if stage not in STAGES:
        raise ValueError(f"unknown review stage {stage!r}; use concept or design")
    if totals is not None and not isinstance(totals, Mapping):
        raise TypeError("totals must map attribute names to BOM target lists")
    if annotations is not None and not isinstance(annotations, Mapping):
        raise TypeError("annotations must map item IDs to annotations")
    verification = None
    package_report = None
    if manifest is not None:
        from .deliverables import verify_export

        if report is not None:
            raise ValueError("supply report or manifest, not both")
        verification = verify_export(manifest, scene=scene)
        if verification["ok"]:
            scope = verification["conditions"]["sequences"]
            if sequences is not None and list(sequences) != scope:
                verification["ok"] = False
                verification["errors"].append("review program scope differs from package scope")
            else:
                sequences = scope
                row = next((r for r in verification["files"] if r["kind"] == "report_json"), None)
                if row:
                    package_report = json.loads((Path(manifest).parent / row["path"]).read_text(encoding="utf-8"))
    required = list(dict.fromkeys([*STAGES[stage], *_strings(required, "required")]))
    check = select.check(scene, sequences=sequences)
    from .connections import report as connection_report

    physical = connection_report(scene)
    req = check.requirements
    items: list[ReviewItem] = []

    def add(id, group, target, status, message, basis, next_action="", **evidence):
        items.append(ReviewItem(id, group, target, status, message, basis, next_action, evidence))

    add("checks:static", "checks", "cell", "pass" if check.ok else "fail",
        "Static checks completed" if check.ok else "Static check errors remain", "Scene.check()",
        "Resolve the error findings" if not check.ok else "", source_kind="derived")
    # Specification findings are represented below with their actual inputs.
    for index, finding in enumerate(check.findings):
        if finding.code in ("spec_short", "spec_unknown", "requirement_incomplete", "unidentified_part") or finding.code.startswith("connection_"):
            continue
        if finding.severity == "info":
            continue
        failed = finding.severity == "error" or finding.code in ("voltage", "polarity", "safety", "safety_pair", "capacity")
        add(f"checks:{finding.code}:{index}", "checks", finding.target or "cell",
            "fail" if failed else "unknown", finding.message, f"Scene.check(): {finding.code}",
            "Resolve the finding against the device/interface specification", finding=finding.to_dict())

    for row in req:
        prefix = f"{row.kind}:{row.target}"
        add(f"equipment:{prefix}", "equipment", row.target, "pass" if row.identified else "unknown",
            "Equipment identity recorded" if row.identified else "Equipment identity missing",
            "BOM identity; identity alone does not establish specification completeness",
            "Identify the equipment with its supplier" if not row.identified else "",
            catalog=row.catalog, model=row.model, manufacturer=row.manufacturer, names=row.names, qty=row.qty,
            source_kind="user_input")
        if row.category == "power_supply":
            sources = [s for s in physical.supplies if s["medium"] == "power" and s["target_kind"] == row.kind and s["target"] in row.names]
            status = "unknown" if not sources else next((s for s in ("fail", "unknown", "not_run")
                        if any(p["status"] == s for p in sources)),
                        "not_applicable" if all(p["status"] == "not_applicable" for p in sources) else "pass")
            add(f"specifications:{prefix}:supply", "specifications", row.target, status,
                "No connected loads; supply capacity not evaluated" if status == "not_applicable" else
                "Supply capacity against connected loads" if sources else "No power supply connections declared",
                "Declared supply ports and connected demand; whole-BOM consumption is not a supply requirement",
                "Declare the supply connections and missing endpoint specifications" if status in ("fail", "unknown", "not_run") else "",
                supplies=sources, source_kind="derived")
        elif not row.requirements and not row.notes:
            add(f"specifications:{prefix}", "specifications", row.target, "not_run",
                "No specification comparison was derived", "Scene.requirements()",
                "Record the applicable requirements, or document why comparison is not applicable")
        for r in row.requirements:
            status = {"ok": "pass", "short": "fail"}.get(r.status, "unknown")
            basis = r.basis
            # Channel counts are independently checked by the I/O lint. A
            # model number is not needed to compare declared channel counts.
            if row.kind == "io_node" and r.provided is not None:
                status = "pass" if select._satisfies(r, r.provided) else "fail"
            provided = "unknown" if r.provided is None else f"{r.provided:g}"
            add(f"specifications:{prefix}:{r.key}", "specifications", row.target, status,
                f"{r}; provided {r.provided_key or r.key} = {provided}", basis,
                "Confirm the required and provided values and their source" if status != "pass" else "",
                comparison=r.to_dict(), catalog=row.catalog, source_kind="derived", provided_source_kind="unknown")
        for index, note in enumerate(row.notes):
            add(f"specifications:{prefix}:incomplete:{index}", "specifications", row.target, "unknown",
                note, "Scene.requirements(): incomplete input or unevaluated requirement",
                "Provide the named input or run the missing evaluation", source_kind="derived")
    if not req.rows:
        for group in ("equipment", "specifications"):
            add(f"{group}:none", group, "cell", "not_applicable", "No BOM equipment",
                "The scene has no BOM rows; this does not check for omitted equipment")

    _connections(scene, sequences, add)
    for entry in physical.checks:
        # No interfaces are inferred for legacy cells. Identifiable utility
        # needs without ports still generate explicit unknown findings.
        if entry["id"] == "requirements:none":
            continue
        add("connections:physical:" + entry["id"], "connections", entry["target"], entry["status"],
            entry["message"], entry["basis"], "Resolve the connection requirement" if entry["status"] in ("fail", "unknown") else "",
            **entry["evidence"])
    subtotals = _totals(scene.bom().rows, {"mass_kg": None} if totals is None else totals)
    for key, value in subtotals.items():
        status = "not_applicable" if value["target_qty"] == 0 else "unknown" if value["missing"] else "pass"
        add(f"totals:{key}", "totals", key, status, "Known subtotal; see target coverage",
            "Sum of qty × finite nonnegative values for explicitly selected BOM rows",
            "Supply values for the listed missing targets" if value["missing"] else "", **value)
    if not subtotals:
        add("totals:none", "totals", "cell", "not_applicable", "No subtotal requested", "totals={} supplied")

    cell_report = json.loads(report.to_json()) if report is not None else package_report
    _execution(scene, req.sequences, cell_report or {}, add, verification)
    ids = {item.id for item in items}
    unknown = set(required) - set(GROUPS) - ids
    if unknown:
        raise ValueError(f"unknown required review groups/items: {sorted(unknown)}")
    _annotate(items, annotations or {})
    for item in items:
        item.required = item.group in required or item.id in required
    return ReviewReport(stage=stage, required=required, items=items, totals=subtotals,
                        check=check.to_dict(), cell_report=cell_report)


def _connections(scene, sequences, add):
    try:
        points = json.loads(scene.io_list("json", sequences=sequences))["points"]
    except ValueError as exc:
        add("connections:derivation", "connections", "cell", "not_run", str(exc),
            "I/O derivation failed", "Resolve the static check errors")
        return
    io = json.loads(scene.io_map().to_json())
    nodes = {node["name"]: node for node in io.get("nodes", [])}
    for p in points:
        name = p["name"] + (f".{p['aspect']}" if p.get("aspect") else "")
        prefix = f"connections:{p.get('host') or '(unhosted)'}:{p['direction']}:{name}"
        if p["status"] in ("internal", "cosmetic"):
            add(prefix, "connections", name, "not_applicable", "No physical channel required",
                f"I/O derivation classifies this point as {p['status']}")
            continue
        binding = next((b for b in io.get("bindings", []) if
                        b["point"]["name"] == p["name"] and b["point"].get("aspect") == p.get("aspect")
                        and b["point"]["direction"] == p["direction"] and b["node"] == p.get("node")), None)
        add(prefix + ":assignment", "connections", name, "pass" if binding else "unknown",
            f"Bound to {binding['node']}.{binding['channel']}" if binding else "No channel assigned",
            "Scene.io_map() / derived I/O point", "Assign the point to a controller channel" if not binding else "",
            point=p, source_kind="derived")
        if not binding:
            continue
        channel = next(c for c in nodes[binding["node"]]["channels"] if c["id"] == binding["channel"])
        if channel["kind"] not in ("di", "do", "safe_di", "safe_do"):
            add(prefix + ":electrical", "connections", name, "not_run",
                "Electrical compatibility is not evaluated for this channel kind", p["kind"],
                "Review the numeric interface with the controller/device supplier")
            continue
        device = binding.get("device") or {}
        terminal = channel.get("electrical") or {}
        for key in ("voltage", "logic"):
            a, b = device.get(key), terminal.get(key)
            missing = [end for end, value in (("field device", a), ("channel", b)) if value is None]
            if missing:
                status = "unknown"
                message = f"Missing {key} on " + " and ".join(missing)
            else:
                matches = abs(a - b) <= 0.5 if key == "voltage" else a == b
                status = "pass" if matches else "fail"
                message = f"{key}: field device {a}, channel {b}"
            add(prefix + ":" + key, "connections", name, status, message,
                ("Declared field/channel voltages; existing I/O lint's 0.5 V tolerance" if key == "voltage"
                 else "Declared field/channel logic types"),
                f"Confirm {key} with the device and controller specifications" if status != "pass" else "",
                field_value=a, channel_value=b, node=binding["node"], channel=binding["channel"], source_kind="user_input")
    if not points:
        add("connections:none", "connections", "cell", "not_applicable", "No I/O points derived",
            "The selected programs and scene declare no I/O; omitted interfaces are not inferred")


def _totals(rows, selections):
    result = {}
    names = {name for row in rows for name in row["names"]}
    for key, selection in selections.items():
        if not isinstance(key, str) or not key:
            raise ValueError("subtotal attribute names must be nonempty strings")
        targets = names if selection is None else set(_strings(selection, "subtotal targets"))
        if targets - names:
            raise ValueError(f"unknown BOM targets for {key}: {sorted(targets - names)}")
        total = {"known_subtotal": None, "target_qty": 0, "known_qty": 0, "missing": [], "targets": []}
        for row in rows:
            if not targets.intersection(row["names"]):
                continue
            if not set(row["names"]) <= targets:
                raise ValueError(f"{key}: select every name in merged BOM row {row['names']} to count its quantity")
            qty = row["qty"]
            total["target_qty"] += qty
            total["targets"].append({"names": row["names"], "qty": qty})
            value = (row.get("attributes") or {}).get(key)
            valid = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0
            if valid:
                subtotal = (total["known_subtotal"] or 0.) + value * qty
                if not math.isfinite(subtotal):
                    raise ValueError(f"{key}: subtotal overflows")
                total["known_subtotal"] = subtotal
                total["known_qty"] += qty
            else:
                total["missing"].append({"names": row["names"], "qty": qty,
                                         "reason": "missing" if value is None else "not a finite nonnegative number"})
        result[key] = total
    return result


def _execution(scene, sequences, report, add, verification=None):
    cycles = report.get("cycles") or []
    for name in sequences:
        observed = [c for c in cycles if name in c["sequences"] and c.get("scenario") in (None, "baseline")]
        add(f"simulation:{name}", "simulation", name, "pass" if observed else "not_run",
            "Completed cycle supplied" if observed else "No completed baseline cycle supplied",
            ("Verified batch report; execution completion, not cycle-budget acceptance"
             if verification is not None and verification["ok"] else
             "Caller-supplied CellReport; execution completion, not cycle-budget acceptance or input-version validation"),
            "Simulate this program and supply its cell report" if not observed else "",
            cycles=[{"name": c["name"], "duration": c["duration"], "sequences": c["sequences"]} for c in observed],
            source_kind="derived")
        clearances = [c["clearance"] for c in observed if c.get("clearance") is not None]
        no_geometry = not scene.robots or not scene.obstacle_names
        clearance_status = "pass" if observed and len(clearances) == len(observed) else "not_run"
        if no_geometry:
            clearance_status = "not_applicable"
        add(f"simulation:{name}:clearance", "simulation", name, clearance_status,
            "Clearance measurement supplied" if clearances else "No clearance measurement supplied",
            "No robots or no environment obstacles" if no_geometry else
            "Sampled robot/environment measurement; no project-specific margin or safety assessment",
            "Supply a cell report with clearance enabled" if clearance_status == "not_run" else "",
            measurements=clearances, source_kind="derived")
    if not sequences:
        add("simulation:none", "simulation", "cell", "not_applicable", "No programs selected",
            "The scene/review scope declares no sequence to simulate")
    scenario_names = list(scene.scenario_names)
    scenario_rows = {r["name"]: r for r in report.get("scenarios") or []}
    for name in scenario_names:
        row = scenario_rows.get(name)
        add(f"scenarios:{name}", "scenarios", name, "unknown" if row else "not_run",
            "Run recorded; expected-result acceptance is not evaluated" if row else "No scenario run supplied",
            "Scenario execution success/stall is not an expected-behaviour verdict",
            "Record and evaluate the expected outputs, timing and recovery conditions", observation=row)
    if not scenario_names:
        add("scenarios:none", "scenarios", "cell", "not_run", "No fault scenarios declared",
            "Scenario coverage is not inferred", "Declare the fault cases required by the project")
    if verification is not None:
        add("deliverables:revision", "deliverables", "package", "pass" if verification["ok"] else "fail",
            "Package and current input revision verified" if verification["ok"] else "; ".join(verification["errors"]),
            "Manifest, file digests, serialized authored input and geometry asset hashes",
            "Regenerate the package from the current cell" if not verification["ok"] else "",
            input_sha256=verification.get("input_sha256"), run_sha256=verification.get("run_sha256"))
        for index, issue in enumerate(verification["issues"]):
            add(f"deliverables:issue:{index}", "deliverables", issue["path"] or "package", "unknown",
                issue["message"], issue["code"], "Resolve or document the exporter limitation")
    rows = verification["files"] if verification is not None else report.get("deliverables") or []
    for row in rows:
        verified = verification is not None and verification["ok"] and row.get("origin") == "generated"
        add(f"deliverables:{row['path']}", "deliverables", row["path"], "pass" if verified else "unknown",
            "Generated file and current input revision verified" if verified else
            "External or unverified file; common design revision is not verified",
            "Export manifest verification" if verification is not None else "CellReport file digest",
            "" if verified else "Confirm the file was generated from the reviewed design revision", **row)
    if not rows:
        add("deliverables:none", "deliverables", "cell", "not_run", "No deliverable records supplied",
            "No files were passed in the cell report", "Generate the documents required for this review")


def _annotate(items, annotations):
    by_id = {item.id: item for item in items}
    if set(annotations) - set(by_id):
        raise ValueError(f"unknown review annotation items: {sorted(set(annotations) - set(by_id))}")
    allowed = {"source_kind", "reference", "assumptions", "owner", "due", "next_action", "not_applicable"}
    for id, note in annotations.items():
        if not isinstance(note, Mapping):
            raise TypeError(f"{id}: annotation must be an object")
        if set(note) - allowed or any(not isinstance(v, str) or not v.strip() for v in note.values()):
            raise ValueError(f"{id}: annotations need known fields and nonempty text values")
        if "source_kind" in note and note["source_kind"] not in SOURCE_KINDS:
            raise ValueError(f"{id}: unknown source_kind")
        if note.get("source_kind") in ("manufacturer", "measured") and not note.get("reference"):
            raise ValueError(f"{id}: manufacturer/measured evidence needs a reference")
        item = by_id[id]
        if "not_applicable" in note:
            if item.status == "fail":
                raise ValueError(f"{id}: an explicit failure cannot be marked not_applicable")
            item.evidence["observed_status"] = item.status
            item.status = "not_applicable"
            item.next_action = ""
        if "next_action" in note:
            item.next_action = note["next_action"]
        item.annotation = dict(note)


def _strings(values, label):
    if values is None:
        return []
    if isinstance(values, str):
        raise TypeError(f"{label} must be a list of names, not a string")
    result = list(values)
    if any(not isinstance(value, str) or not value for value in result):
        raise ValueError(f"{label} must contain nonempty names")
    return result


def _md(value):
    return str(value).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;").replace("|", "&#124;").replace("\n", "<br>")
