"""Evidence-backed declaration checks for mechanically attached products.

The Rust checker reads the actual Robot attachment source. It does not infer
fit from visual meshes, BOM labels, flange-standard hints, or attachment offsets.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ._core import Robot, Scene
    from .review import ReviewItem

__all__ = ["MountingReport", "report"]


class MountingReport:
    """Mechanical assembly observations. ``ready`` covers all listed checks.

    Drawing dimensions, selected fasteners and declared assembly envelopes
    are checked when supplied. Missing inputs remain ``unknown``/``not_run``;
    matching interface names alone cannot make an assembly ready.
    """

    def __init__(self, data: dict):
        from .review import ReviewItem

        self.scope = data["scope"]
        self.validator_version = data["validator_version"]
        self.input_hash = data["input_hash"]
        self.assemblies = data["assemblies"]
        self.kits = data.get("kits", [])
        self.products = data.get("products", [])
        self.simulation = data.get("simulation", {"ready": False, "blockers": [], "connections": []})
        self.items = [ReviewItem(**item) for item in data["items"]]

    @property
    def ready(self) -> bool:
        return not self.blockers()

    def blockers(self) -> list[ReviewItem]:
        return [item for item in self.items if item.blocking]

    def to_dict(self) -> dict:
        return {"scope": self.scope, "validator_version": self.validator_version,
                "input_hash": self.input_hash, "ready": self.ready,
                "assemblies": self.assemblies, "kits": self.kits, "products": self.products, "simulation": self.simulation,
                "items": [item.to_dict() for item in self.items]}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False, allow_nan=False)

    def to_markdown(self) -> str:
        from .review import _md

        lines = ["# Mechanical mounting review", "",
                 f"Scope: {self.scope}. Result: {'ready' if self.ready else 'unresolved'}.",
                 f"Validator: {self.validator_version}. Input: {self.input_hash}.", "",
                 "| target | item | result | observation | next action |",
                 "|---|---|---|---|---|"]
        if self.kits:
            table = lines[-2:]
            lines = lines[:-2] + ["Manufacturer kits (mechanical support and detailed fit are separate):", "",
                "| kit | manufacturer support | composition | model correspondence | detailed fit |",
                "|---|---|---|---|---|"]
            for kit in self.kits:
                lines.append("| " + " | ".join(_md(v) for v in [kit["target"],
                    *(kit[key]["status"] for key in ("manufacturer_support", "composition", "model_correspondence", "detailed_fit"))]) + " |")
                note = kit["manufacturer_support"].get("note")
                if note:
                    lines += ["", _md(note), ""]
            lines += ["", "Declared connection conditions (separate from mechanical results):", "",
                      "| kit | configuration | electrical | communication | software |", "|---|---|---|---|---|"]
            for kit in self.kits:
                connection = kit.get("connection", {})
                lines.append("| " + " | ".join(_md(v) for v in [kit["target"],
                    *(connection.get(key, {}).get("status", "unknown") for key in ("configuration", "electrical", "communication", "software"))]) + " |")
            lines += ["", *table]
        if self.simulation["connections"]:
            table = lines[-2:]
            lines = lines[:-2] + ["Simulation mounting routes (not a detailed-fit approval):", "",
                "| connection | method | evidence basis |", "|---|---|---|"]
            for connection in self.simulation["connections"]:
                lines.append("| " + " | ".join(_md(connection[key]) for key in ("target", "method", "basis")) + " |")
            lines += ["", f"Mounting eligible for simulation: {self.simulation['ready']}.", "", *table]
        for item in self.items:
            lines.append("| " + " | ".join(_md(v) for v in
                         (item.target, item.id, item.status, item.message, item.next_action)) + " |")
            for detail in item.evidence.get("checks", []):
                lines.append("| " + " | ".join(_md(v) for v in
                    (item.target, detail["check"], detail["status"], detail["message"], "")) + " |")
        return "\n".join(lines) + "\n"

    def save(self, path: str | Path, format: str | None = None) -> None:
        path = Path(path)
        fmt = (format or path.suffix.lstrip(".")).lower()
        if fmt not in ("json", "md", "markdown"):
            raise ValueError("mounting report format must be json or markdown")
        path.write_text(self.to_json() if fmt == "json" else self.to_markdown(), encoding="utf-8")


def report(target: Robot | Scene) -> MountingReport:
    """Inspect a ``Robot`` or every robot in a ``Scene`` without simulation.

    A Scene additionally checks whether ``set_part(catalog=...)`` disagrees
    with the loaded model. Required adapters must be on the tool's actual
    upstream path. Missing declarations/evidence stay unknown.
    """
    from ._core import Robot, Scene

    if not isinstance(target, (Robot, Scene)):
        raise TypeError("mounting.report expects a Robot or Scene")
    return MountingReport(json.loads(target._mounting_report_json()))


class MountingProposal:
    """A separate assembly and its comparison against a scene snapshot.

    ``route`` describes mounting evidence, not a detailed-fit approval.
    ``save`` retains unknown/failing proposals in the normal project format;
    ``can_apply`` also accepts supported kits and sourced interface/pose/part
    declarations with incomplete detail. ``report.simulation`` records each
    route and its basis; ``report.ready`` remains the strict result.
    Known mismatches and unresolved scene references still block application.
    """

    def __init__(self, scene: Scene, candidate: Robot, robot: str | None = None):
        self._scene = scene
        self._candidate = candidate
        self._robot = robot
        self.scene, encoded = scene._mounting_preview(candidate, robot)
        self.data = json.loads(encoded)
        self.before = self.data["before"]
        self.after = self.data["after"]
        self.report = MountingReport(self.after["report"])
        self.route = self.data["route"]
        self.can_apply = self.data["can_apply"]
        self.blockers = self.data["blockers"]
        self.mounting_blockers = self.data.get("mounting_blockers", [])
        self.revalidation = self.data["revalidation"]
        self.preserved_joints = self.data["preserved_joints"]
        self.reset_joints = self.data["reset_joints"]

    def save(self, path: str | Path) -> None:
        """Save the separate assembly, bundling assets via Scene.save_project.

        This does not apply the proposal or persist a confirmation flag. A
        loaded assembly is inspected again from its original declarations.
        """
        self.scene.save_project(str(path))

    def apply(self) -> None:
        """Recheck the snapshots and atomically rebuild the scene robot.

        Raises ValueError for stale previews, unresolved mounting or dangling
        scene references. Existing motions must be planned again, sequences
        simulated again and toolpaths checked again after replacement.
        """
        self._scene._mounting_apply(
            self._candidate, self.data["base_revision"],
            self.data["candidate_revision"], self._robot,
        )


def preview(scene: Scene, candidate: Robot, *, robot: str | None = None) -> MountingProposal:
    """Compare a complete candidate Robot without modifying the live scene.

    Compose the candidate with the existing Robot.attach_tool/mount methods,
    or reuse Scene.load_project(path).robot. A kit remains one purchase unit.
    Mass is the declared BOM mass: ``known_kg`` is a subtotal whenever
    ``missing`` is nonempty, and is never an inferred physical mass.
    """
    from ._core import Robot, Scene

    if not isinstance(scene, Scene) or not isinstance(candidate, Robot):
        raise TypeError("mounting.preview expects a Scene and a candidate Robot")
    return MountingProposal(scene, candidate, robot)


def candidates(scene: Scene, robots, *, robot: str | None = None) -> list[MountingProposal]:
    """Inspect supplied candidates, retaining unknowns and known failures.

    Inputs may come from catalog queries, local packages or saved projects.
    This is evidence-based guidance over supplied models, not a claim that
    the catalog contains every available adapter or an automatic CAD search.
    """
    order = {"direct_evidence": 0, "adapter_evidence": 1, "needs_information": 2}
    return sorted((preview(scene, candidate, robot=robot) for candidate in robots),
                  key=lambda proposal: order[proposal.route])


__all__ += ["MountingProposal", "candidates", "preview"]
