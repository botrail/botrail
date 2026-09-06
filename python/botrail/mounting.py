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
        self.items = [ReviewItem(**item) for item in data["items"]]

    @property
    def ready(self) -> bool:
        return not self.blockers()

    def blockers(self) -> list[ReviewItem]:
        return [item for item in self.items if item.blocking]

    def to_dict(self) -> dict:
        return {"scope": self.scope, "validator_version": self.validator_version,
                "input_hash": self.input_hash, "ready": self.ready,
                "assemblies": self.assemblies, "items": [item.to_dict() for item in self.items]}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False, allow_nan=False)

    def to_markdown(self) -> str:
        from .review import _md

        lines = ["# Mechanical mounting review", "",
                 f"Scope: {self.scope}. Result: {'ready' if self.ready else 'unresolved'}.",
                 f"Validator: {self.validator_version}. Input: {self.input_hash}.", "",
                 "| target | item | result | observation | next action |",
                 "|---|---|---|---|---|"]
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
