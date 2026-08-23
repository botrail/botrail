"""Spec packs — the catalog's `kind: spec` packages, read here.

Some equipment is standard but bought to size: safety fences, conveyors,
racking. The catalog does not ship geometry for those (there is no one shape
to ship); it ships the *configuration* — the dimensions you can actually
order, the part numbers those compose into, and the mass that comes with
them. The generators in `bt.parts` build the geometry from it, so the boxes
in the scene and the lines on the BOM say the same thing.

    spec = Spec.load("botrail/fence/mesh-guard")
    height_mm = spec.choose("height_mm", 2000)     # rejects 1800 with the list
    spec.part_number("panel", height_mm=height_mm, width_mm=1200)  # MG-2000x1200

`load` takes a catalog id (downloaded through `catalog_package`), an
`(id, revision)` pair, or a path to a package directory — the last one is how
the catalog builder validates a package before it is published.

Nothing here evaluates expressions: a parameter is a list of values or a
stepped range, and mass is a table, a length coefficient or an areal
density (or an areal one plus a coefficient, for a panel whose frame runs
round its edge). A part number is a template over those values — and where a
maker does not write a dimension as it stands, the pack carries a table of
the codes it writes instead, per part. See docs/equipment-catalog.md in
botrail-catalog-builder.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Optional, Union

CatalogRef = Union[str, "Path", tuple[str, Optional[str]]]

_EPS = 1e-6


class Spec:
    """One spec pack: its manifest, and the questions a generator asks of it."""

    def __init__(self, manifest: dict, directory: Path, revision: Optional[str]) -> None:
        self.manifest = manifest
        self.directory = directory
        self.id: str = manifest.get("id", str(directory))
        self.revision = revision
        self.name: Optional[str] = manifest.get("name")
        maker = manifest.get("manufacturer") or {}
        self.manufacturer: Optional[str] = maker.get("name") if isinstance(maker, dict) else None
        config = manifest.get("configuration")
        if not isinstance(config, dict):
            raise ValueError(
                f"{self.id}: not a spec pack — it has no `configuration` "
                "(a geometry package loads with Robot.from_catalog instead)"
            )
        self.config = config

    # ---------------------------------------------------------------- load

    @classmethod
    def load(cls, ref: CatalogRef) -> "Spec":
        id_or_path, revision = ref if isinstance(ref, tuple) else (ref, None)
        path = Path(id_or_path)
        if path.is_dir():
            directory = path
        elif path.is_file():
            directory = path.parent
        else:
            from . import catalog_package

            directory = Path(catalog_package(str(id_or_path), revision=revision))
            revision = revision or _snapshot_revision(directory)
        return cls(_read_manifest(directory), directory, revision)

    @property
    def catalog_ref(self) -> tuple[str, Optional[str]]:
        """What `set_part(catalog=...)` records — the id and the revision it came from."""
        return self.id, self.revision

    # ------------------------------------------------------------ questions

    @property
    def generator(self) -> Optional[str]:
        value = self.config.get("generator")
        return str(value) if value is not None else None

    def expect_generator(self, name: str) -> None:
        if self.generator not in (None, name):
            raise ValueError(
                f"{self.id} is a `{self.generator}` package — bt.parts.{name} cannot build it"
            )

    def params(self) -> dict:
        params = self.config.get("params") or {}
        return params if isinstance(params, dict) else {}

    def default(self, name: str) -> Any:
        param = self.params().get(name)
        return param.get("default") if isinstance(param, dict) else None

    def choose(self, name: str, value: Any) -> Any:
        """Match a requested dimension against what the catalog actually sells."""
        param = self.params().get(name)
        if not isinstance(param, dict):
            known = ", ".join(sorted(self.params())) or "none"
            raise ValueError(f"{self.id}: no parameter {name!r} (it has: {known})")
        if value is None:
            return param.get("default")
        if "values" in param:
            for candidate in param["values"]:
                if _same(candidate, value):
                    return candidate
            allowed = " / ".join(str(_plain(v)) for v in param["values"])
            raise ValueError(
                f"{self.id}: {name}={_plain(value)} is not available — choose from {allowed}"
            )
        low, high, step = param["min"], param["max"], param["step"]
        if not isinstance(value, (int, float)):
            raise ValueError(f"{self.id}: {name}={value!r} is not a number")
        if not (low - _EPS <= value <= high + _EPS):
            raise ValueError(
                f"{self.id}: {name}={_plain(value)} is out of range "
                f"{_plain(low)}..{_plain(high)} (step {_plain(step)})"
            )
        steps = round((value - low) / step)
        snapped = low + steps * step
        if abs(snapped - value) > 1e-4:
            raise ValueError(
                f"{self.id}: {name}={_plain(value)} is off the {_plain(step)} step — "
                f"nearest is {_plain(snapped)}"
            )
        return snapped

    def behavior(self, name: str, value: Any = None) -> Any:
        """A running value — a speed the unit can be driven at, a rating it
        carries. Ranges and choices are matched like a dimension; a plain
        number is a rating and comes back as it stands."""
        entry = (self.config.get("behavior") or {}).get(name)
        if entry is None:
            return value
        if not isinstance(entry, dict):
            return entry
        if value is None:
            return entry.get("default")
        if "values" in entry:
            for candidate in entry["values"]:
                if _same(candidate, value):
                    return candidate
            allowed = " / ".join(str(_plain(v)) for v in entry["values"])
            raise ValueError(
                f"{self.id}: {name}={_plain(value)} is not available — choose from {allowed}"
            )
        low, high = entry["min"], entry["max"]
        if not (low - _EPS <= value <= high + _EPS):
            raise ValueError(
                f"{self.id}: {name}={_plain(value)} is out of range "
                f"{_plain(low)}..{_plain(high)}"
            )
        step = entry.get("step")
        if step is None:
            return value
        steps = round((value - low) / step)
        snapped = low + steps * step
        if abs(snapped - value) > 1e-4:
            raise ValueError(
                f"{self.id}: {name}={_plain(value)} is off the {_plain(step)} step — "
                f"nearest is {_plain(snapped)}"
            )
        return snapped

    def behaviors(self) -> dict:
        """Every running value at its default — what the BOM records."""
        entries = self.config.get("behavior") or {}
        return {name: self.behavior(name) for name in entries}

    def trim(self, role: str) -> Optional[Path]:
        """The file this part is *drawn* from, if the pack ships one — a URDF
        or xacro of primitives, expanded to the size at hand. Without it the
        generator falls back to its own built-in look."""
        if not self.has_component(role):
            return None
        rel = self.component(role).get("trim")
        return None if not rel else self.directory / str(rel)

    def specs(self) -> dict:
        """The datasheet numbers a generator does not use but a bill should
        carry — a rack's load per level, an ingress rating."""
        specs = self.manifest.get("specs") or {}
        return {k: v for k, v in specs.items() if isinstance(v, (int, float, str))}

    def category(self, role: str, default: Optional[str] = None) -> Optional[str]:
        return self.component(role).get("category", default)

    def component(self, role: str) -> dict:
        for component in self.config.get("components") or []:
            if isinstance(component, dict) and component.get("role") == role:
                return component
        raise ValueError(f"{self.id}: no component {role!r}")

    def has_component(self, role: str) -> bool:
        try:
            self.component(role)
        except ValueError:
            return False
        return True

    def widths_mm(self, role: str) -> Optional[list]:
        widths = self.component(role).get("widths_mm")
        return sorted(widths) if widths else None

    def dimension_mm(self, role: str, key: str, default: Optional[float] = None) -> Optional[float]:
        """A fixed dimension the generator draws with. A pack that does not
        carry that part (or that number) leaves the generator its default."""
        if not self.has_component(role):
            return None if default is None else float(default)
        dims = self.component(role).get("dimensions_mm") or {}
        value = dims.get(key, default)
        return float(value) if value is not None else None

    def rule(self, key: str, default: Any = None) -> Any:
        rules = self.config.get("rules") or {}
        return rules.get(key, default)

    def part_number(self, role: str, **values: Any) -> str:
        component = self.component(role)
        template = component.get("part_number")
        if not template:
            return ""
        fields = {k: _plain(v) for k, v in values.items()}
        fields.update(self._order_codes(role, component, values))
        try:
            return str(template).format(**fields)
        except KeyError as exc:
            raise ValueError(f"{self.id}: part number for {role!r} needs {exc}") from exc

    def _order_codes(self, role: str, component: dict, values: dict) -> dict:
        """What a dimension is called in the article number. Some makers do not
        write it as it stands — Axelent codes a 2200 mm panel as 220 — so the
        pack carries the table, per part: the post of that same fence stands
        2300 mm tall and is coded 230."""
        codes = {}
        for name, table in (component.get("codes") or {}).items():
            if name not in values or not isinstance(table, dict):
                continue
            # A stepped axis is coded in bands (any height from 700 to 799 is
            # one article); a list of values has a code each, and a missing
            # one is a hole in the pack rather than a band to fall into.
            param = self.params().get(name)
            banded = isinstance(param, dict) and "values" not in param
            code = _code(table, values[name], band=banded)
            if code is None:
                raise ValueError(
                    f"{self.id}: {role} has no order code for {name}={_plain(values[name])}"
                )
            codes[f"{name}_code"] = code
        return codes

    def mass_kg(self, role: str, **values: Any) -> Optional[float]:
        mass = self.component(role).get("mass")
        if not isinstance(mass, dict):
            return None
        if "table" in mass:
            for row in mass["table"]:
                keys = [k for k in row if k != "kg"]
                if all(k in values and _same(values[k], row[k]) for k in keys):
                    return float(row["kg"])
            wanted = ", ".join(f"{k}={_plain(v)}" for k, v in sorted(values.items()))
            raise ValueError(f"{self.id}: no mass row for {role} at {wanted}")
        total = float(mass.get("base_kg", 0.0))
        if "per_m2_kg" in mass:
            side_a, side_b = (self._dimension(role, values, key) for key in mass["area"])
            total += (side_a / 1000.0) * (side_b / 1000.0) * float(mass["per_m2_kg"])
        for key, coeff in (mass.get("per_mm") or {}).items():
            total += float(coeff) * self._dimension(role, values, key)
        return round(total, 3)

    def _dimension(self, role: str, values: dict, key: str) -> float:
        """A dimension the mass is worked out from. Mass follows what the
        catalog resolved, so a pack that weighs a part by an axis it does not
        sell is broken rather than merely quiet."""
        try:
            return float(values[key])
        except (KeyError, TypeError, ValueError):
            raise ValueError(
                f"{self.id}: the mass of {role!r} needs {key}, which this pack does not size"
            ) from None


def _read_manifest(directory: Path) -> dict:
    path = directory / "manifest.yaml"
    if not path.is_file():
        raise ValueError(f"{directory}: no manifest.yaml — not a catalog package")
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - the catalog extra pulls it in
        raise ValueError(
            "reading a catalog manifest needs PyYAML — install it with "
            "`pip install botrail[catalog]`"
        ) from exc
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"{path}: manifest is not a mapping")
    return data


def _snapshot_revision(directory: Path) -> Optional[str]:
    """The dataset commit a downloaded package came from (its snapshot directory)."""
    parts = directory.parts
    if "snapshots" in parts:
        index = parts.index("snapshots")
        if index + 1 < len(parts):
            return parts[index + 1]
    return None


def _plain(value: Any) -> Any:
    """2000.0 prints as 2000 — these go into part numbers and error messages."""
    if isinstance(value, float) and abs(value - round(value)) < _EPS:
        return int(round(value))
    return value


def _same(a: Any, b: Any) -> bool:
    if isinstance(a, str) or isinstance(b, str):
        return a == b
    return abs(float(a) - float(b)) < _EPS


def _code(table: dict, value: Any, band: bool = False) -> Optional[str]:
    """A code table read back from the manifest has string keys (YAML through
    JSON turns 2200 into "2200"), so match on the number where there is one.

    With `band`, the keys are the low end of a range instead: a stand cut to
    any height between 700 and 799 mm is one article, so the greatest key at
    or below the value names it."""
    for key, code in table.items():
        try:
            if abs(float(key) - float(value)) < _EPS:
                return str(code)
        except (TypeError, ValueError):
            if str(key) == str(value):
                return str(code)
    if not band:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    best: Optional[tuple[float, str]] = None
    for key, code in table.items():
        try:
            low = float(key)
        except (TypeError, ValueError):
            continue
        if low <= number + _EPS and (best is None or low > best[0]):
            best = (low, str(code))
    return None if best is None else best[1]
