"""What the loaded products say about how they are mounted.

``report(robot_or_scene)`` walks the assembly ``attach_tool`` recorded and
lists items, each ``pass``, ``fail`` or ``unknown``:

* ``required:<id>`` — a part the product's installation documents require
  between it and the robot flange (a Robotiq 2F-85 needs its coupling), and
  whether that part is actually on the attachment path;
* ``kit_host`` / ``kit_composition`` — whether a purchase kit is documented
  for the robot it is mounted on, and whether it is still assembled the way
  the manufacturer's part number describes.
* ``mount_pose`` — whether a vehicle-mounted robot's mounting face matches
  the recorded carrier frame and reference pose. This measures frame alignment;
  it does not establish mechanical fit or manufacturer approval.
* ``requirements_complete`` / ``declared_pose`` — completeness of the
  installation's required-part list and its explicitly permitted poses.
* ``interface`` — agreement of evidenced bare-interface identifiers;
  this is separate from physical dimensions and manufacturer host approval.
* ``geometry`` / ``fasteners`` / ``clearance`` — drawing dimensions and
  tolerances, selected screws and tightening conditions, and conservative
  envelopes of adjacent parts and tool access.

Fit checks use verified drawing frames and explicit bounds, never mesh
measurements or assumed tolerances. Missing evidence, incomplete coverage
and partially compatible ranges remain ``unknown``. All actual tool/arm
attachments and rigid vehicle mounts are reviewed, including older files
without drawing data. ``ready`` is therefore false for such unverified
assemblies even if the required parts and catalog frames match.
"""

from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ._core import Robot, Scene

__all__ = ["MountingItem", "MountingReport", "report"]


@dataclass
class MountingItem:
    """One observation: ``mounting:<target>:<key>``."""

    id: str
    target: str
    status: str
    message: str
    next_action: str = ""
    evidence: dict = field(default_factory=dict)
    group: str = "mounting"
    basis: str = ""
    required: bool = True

    @property
    def key(self) -> str:
        return self.id.split(":", 2)[-1]

    def to_dict(self) -> dict:
        return asdict(self)


def _md(value) -> str:
    return str(value if value is not None else "").replace("|", "\\|").replace("\n", "<br>")


class MountingReport:
    """The items, the assemblies they were read from, and the kits found."""

    def __init__(self, data: dict):
        self.assemblies: list[dict] = data["assemblies"]
        self.kits: list[dict] = data.get("kits", [])
        self.items = [MountingItem(**item) for item in data["items"]]

    @property
    def ready(self) -> bool:
        """No reported item failed or stayed unknown; not a full hardware qualification."""
        return not self.blockers()

    def blockers(self) -> list[MountingItem]:
        return [item for item in self.items if item.status in ("fail", "unknown")]

    def to_dict(self) -> dict:
        return {"ready": self.ready, "assemblies": self.assemblies, "kits": self.kits,
                "items": [item.to_dict() for item in self.items]}

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), indent=2, ensure_ascii=False, allow_nan=False)

    def to_markdown(self) -> str:
        lines = ["# Mounting", "", "Result: " + ("ready" if self.ready else "unresolved") + ".", ""]
        if self.kits:
            lines += ["| kit | host | manufacturer support | composition |", "|---|---|---|---|"]
            for kit in self.kits:
                lines.append("| " + " | ".join(_md(v) for v in (
                    kit["target"], kit.get("host") or "—",
                    kit["manufacturer_support"]["status"], kit["composition"]["status"])) + " |")
            lines.append("")
        if self.items:
            lines += ["| target | check | result | finding | next action |", "|---|---|---|---|---|"]
            for item in self.items:
                lines.append("| " + " | ".join(_md(v) for v in (
                    item.target, item.key, item.status, item.message, item.next_action)) + " |")
        elif self.assemblies:
            lines.append("The loaded products carry no mounting statements to check.")
        else:
            lines.append("No tool attachments or vehicle mounts to review.")
        return "\n".join(lines) + "\n"

    def save(self, path: str | Path, format: str | None = None) -> None:
        path = Path(path)
        fmt = (format or path.suffix.lstrip(".")).lower()
        if fmt not in ("json", "md", "markdown"):
            raise ValueError("mounting report format must be json or markdown")
        path.write_text(self.to_json() if fmt == "json" else self.to_markdown(), encoding="utf-8")


def report(target: Robot | Scene) -> MountingReport:
    """Review tool attachments, and vehicle mounting frames when given a ``Scene``."""
    from ._core import Robot, Scene

    if not isinstance(target, (Robot, Scene)):
        raise TypeError("mounting.report expects a Robot or Scene")
    return MountingReport(json.loads(target._mounting_report_json()))
