"""Search the botrail catalog — real products to choose from.

The catalog (the Hugging Face dataset `botrail/botrail-catalog`) publishes an
`index.json` with every product's category, maker, numeric specs and
validation level. This module reads that index and filters it the way a
requirement reads — minimums on spec keys — so a person or an agent picks
from products that exist, with the numbers that decide the pick in view.
botrail still does not choose: :func:`search` returns candidates, and
:meth:`Product.identify` writes the one *you* picked onto the cell.

    cands = bt.catalog.search("gripper.parallel", stroke_mm=150, payload_kg=2.3)
    cands[0].identify(scene, "ur5e/tool")           # set_part with the catalog id and its specs
    bt.catalog.search_for(scene.requirements()["eye"])   # straight from a requirement row

Spec keys are the requirement vocabulary of :mod:`botrail.select` — the
catalog's `specs` / `mechanical` / `electrical` numbers, read through the
same aliases. A product that does not state a filtered key does not match
(unknown is not a pass). Results are ordered by validation level, then by
closeness to the requested minimums, then by id — the same query gives the
same list.

The index is fetched with `huggingface_hub` (`pip install botrail[catalog]`)
and pinned to a dataset commit; a copy already in the Hugging Face cache is
used when the hub cannot be reached, and `index(path=...)` /
`BOTRAIL_CATALOG_INDEX` read a local file (a builder's `dist/index.json`).
"""

from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterator, Optional, Union

from .select import ALIASES, _number

__all__ = ["REPO_ID", "Index", "Product", "index", "search", "search_for"]

REPO_ID = "botrail/botrail-catalog"
REPO_URL = f"https://huggingface.co/datasets/{REPO_ID}"
LEVELS = ("V0", "V1", "V2", "V3", "V4", "V5")

_CACHE: dict[Optional[str], "Index"] = {}


@dataclass
class Product:
    """One catalog entry as the index lists it."""

    id: str
    category: str
    name: str
    manufacturer: Optional[str] = None
    kind: str = "model"
    specs: dict[str, Any] = field(default_factory=dict)
    mechanical: Optional[dict[str, Any]] = None
    electrical: Optional[dict[str, Any]] = None
    validation_level: Optional[str] = None
    distribution: str = "public"
    assets: dict[str, Any] = field(default_factory=dict)
    configuration: Optional[dict[str, Any]] = None
    #: The dataset commit the index came from (None for a local file).
    revision: Optional[str] = None

    @classmethod
    def from_entry(cls, entry: dict[str, Any], revision: Optional[str] = None) -> "Product":
        maker = entry.get("manufacturer")
        if isinstance(maker, dict):
            maker = maker.get("name")
        return cls(
            id=str(entry["id"]),
            category=str(entry.get("category") or ""),
            name=str(entry.get("name") or entry["id"]),
            manufacturer=maker,
            kind=str(entry.get("kind") or "model"),
            specs=dict(entry.get("specs") or {}),
            mechanical=entry.get("mechanical"),
            electrical=entry.get("electrical"),
            validation_level=entry.get("validation_level"),
            distribution=str(entry.get("distribution") or "public"),
            assets=dict(entry.get("assets") or {}),
            configuration=entry.get("configuration"),
            revision=revision,
        )

    @property
    def catalog_ref(self) -> tuple[str, Optional[str]]:
        """What `set_part(catalog=...)` records: the id and the revision."""
        return self.id, self.revision

    @property
    def level(self) -> int:
        """Validation level as a number (V0 = 0); unknown counts as -1."""
        lvl = self.validation_level
        return LEVELS.index(lvl) if lvl in LEVELS else -1

    def attributes(self) -> dict[str, float]:
        """Every numeric spec the product states, flattened — what lands on
        the part when it is identified, and what :func:`search` filters on."""
        out: dict[str, float] = {}
        for source in (self.specs, self.mechanical or {}, self.electrical or {}):
            for key, value in source.items():
                n = _number(value)
                if n is not None:
                    out.setdefault(str(key), n)
                elif isinstance(value, (list, tuple)) and key == "footprint_mm" and len(value) >= 2:
                    x, y = _number(value[0]), _number(value[1])
                    if x is not None and y is not None:
                        out.setdefault("footprint_x_mm", x)
                        out.setdefault("footprint_y_mm", y)
        return out

    def value(self, key: str) -> Optional[float]:
        """The number answering a requirement key, through the same aliases
        the requirement check uses."""
        attrs = self.attributes()
        for alias in ALIASES.get(key, (key,)):
            if alias in attrs:
                return attrs[alias]
        return None

    def text(self, key: str) -> Optional[str]:
        """A string spec (`ip_rating`, `flange_standard`, ...), if stated."""
        for source in (self.specs, self.mechanical or {}, self.electrical or {}):
            value = source.get(key)
            if isinstance(value, str):
                return value
        return None

    def identify(self, scene, target: str, *, kind: Optional[str] = None, qty: int = 1, **overrides: Any) -> str:
        """Write this product onto a resident: `set_part(target, catalog=(id,
        revision), manufacturer, model, category, attributes)`. The numeric
        specs come along, so the requirement check reads them. `overrides`
        add or replace attributes (`mass_kg=...`). Returns what `set_part`
        returns (the kind the target resolved to)."""
        attributes: dict[str, Any] = dict(self.attributes())
        for key, value in overrides.items():
            attributes[key] = value
        return scene.set_part(
            target,
            kind=kind,
            catalog=self.catalog_ref,
            manufacturer=self.manufacturer,
            model=self.name,
            category=self.category,
            qty=qty,
            attributes=attributes,
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.id,
            "revision": self.revision,
            "kind": self.kind,
            "category": self.category,
            "name": self.name,
            "manufacturer": self.manufacturer,
            "validation_level": self.validation_level,
            "distribution": self.distribution,
            "attributes": self.attributes(),
        }

    def __repr__(self) -> str:
        attrs = ", ".join(f"{k}={v:g}" for k, v in sorted(self.attributes().items()))
        return f"Product({self.id!r}, {self.category}, {self.validation_level or '?'}, {attrs})"


class Index:
    """The catalog index: every product, and the search over them."""

    def __init__(self, products: list[Product], *, revision: Optional[str] = None, source: Optional[str] = None) -> None:
        self.products = products
        self.revision = revision
        self.source = source

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, revision: Optional[str] = None, source: Optional[str] = None) -> "Index":
        products = [Product.from_entry(e, revision) for e in data.get("products") or [] if "id" in e]
        return cls(products, revision=revision, source=source)

    @classmethod
    def from_path(cls, path: Union[str, Path], *, revision: Optional[str] = None) -> "Index":
        path = Path(path)
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        return cls.from_dict(data, revision=revision, source=str(path))

    def __iter__(self) -> Iterator[Product]:
        return iter(self.products)

    def __len__(self) -> int:
        return len(self.products)

    def categories(self) -> list[str]:
        return sorted({p.category for p in self.products})

    def get(self, query: str) -> Product:
        """A product by exact id, or by a unique id whose path segments
        contain the query's segments in order (`universal_robots/ur5e`);
        several revisions of one product resolve to the newest."""
        for p in self.products:
            if p.id == query:
                return p
        parts = [s for s in query.split("/") if s]
        matches = [p for p in self.products if _subsequence(parts, p.id.split("/"))]
        if not matches:
            raise KeyError(f"no catalog product matches {query!r}")
        stems = {p.id.rsplit("/", 1)[0] for p in matches}
        if len(stems) == 1:
            return max(matches, key=lambda p: _revision_number(p.id))
        raise KeyError(f"{query!r} is ambiguous: {sorted(p.id for p in matches)}")

    def search(
        self,
        category: Optional[str] = None,
        *,
        kind: Optional[str] = None,
        manufacturer: Optional[str] = None,
        level: Optional[str] = None,
        text: Optional[str] = None,
        limit: Optional[int] = None,
        **specs: Any,
    ) -> list[Product]:
        """Products matching a category (prefix: `gripper` matches
        `gripper.parallel`), a maker, a minimum validation level, a text
        fragment of id or name, and spec filters: `key=value` means the
        product states `key >= value`; `key__max=value` means `<= value`;
        a string value must equal the product's string spec. Ordered by
        validation level (best first), then closeness to the minimums
        (smallest headroom first), then id."""
        min_level = LEVELS.index(level) if level in LEVELS else None
        filters = [_filter(k, v) for k, v in specs.items()]
        out: list[tuple[tuple, Product]] = []
        for p in self.products:
            if category and not _category_matches(p.category, category):
                continue
            if kind and p.kind != kind:
                continue
            if manufacturer and (p.manufacturer or "").lower() != manufacturer.lower():
                continue
            if min_level is not None and p.level < min_level:
                continue
            if text and text.lower() not in (p.id + " " + p.name).lower():
                continue
            closeness = 0.0
            ok = True
            for key, op, value in filters:
                if isinstance(value, str):
                    have = p.text(key)
                    if have is None or have.strip().lower() != value.strip().lower():
                        ok = False
                        break
                    continue
                have_n = p.value(key)
                if have_n is None:
                    ok = False
                    break
                if op == ">=":
                    if have_n + 1e-9 < value:
                        ok = False
                        break
                    closeness += (have_n - value) / max(abs(value), 1e-9)
                elif op == "<=":
                    if have_n - 1e-9 > value:
                        ok = False
                        break
                    closeness += (value - have_n) / max(abs(value), 1e-9)
                else:
                    if abs(have_n - value) > 1e-9:
                        ok = False
                        break
            if ok:
                out.append(((-p.level, round(closeness, 9), p.id), p))
        out.sort(key=lambda item: item[0])
        products = [p for _, p in out]
        return products[:limit] if limit is not None else products


def index(
    *,
    revision: Optional[str] = None,
    path: Union[str, Path, None] = None,
    offline: Optional[bool] = None,
    refresh: bool = False,
) -> Index:
    """The catalog index. `path` (or `BOTRAIL_CATALOG_INDEX`) reads a local
    `index.json`; otherwise the hub resolves `revision` (default: the newest
    commit) and downloads the index pinned to it. When the hub cannot be
    reached the newest index already in the Hugging Face cache is used —
    `offline=True` goes straight to the cache, `offline=False` never does.
    Hub lookups are cached per process (`refresh=True` asks again)."""
    path = path or os.environ.get("BOTRAIL_CATALOG_INDEX")
    if path:
        return Index.from_path(path)
    if offline:
        cached = _cached_index()
        if cached is None:
            raise ValueError(f"no catalog index in the Hugging Face cache (run online once, or pass path=); {REPO_URL}")
        return cached
    if not refresh and revision in _CACHE:
        return _CACHE[revision]
    try:
        idx = _hub_index(revision)
    except Exception as e:  # any hub failure falls back to the cache
        if offline is False:
            raise ValueError(f"cannot fetch the catalog index from {REPO_URL}: {e}") from e
        cached = _cached_index() if revision is None else _cached_index(revision)
        if cached is None:
            raise ValueError(
                f"cannot fetch the catalog index from {REPO_URL} ({e}) and no cached copy was found"
            ) from e
        idx = cached
    _CACHE[revision] = idx
    return idx


def search(
    category: Optional[str] = None,
    *,
    index: Union[Index, str, Path, None] = None,
    revision: Optional[str] = None,
    kind: Optional[str] = None,
    manufacturer: Optional[str] = None,
    level: Optional[str] = None,
    text: Optional[str] = None,
    limit: Optional[int] = None,
    **specs: Any,
) -> list[Product]:
    """Products in the catalog that match — see :meth:`Index.search`.
    `index` takes an :class:`Index` or a path to one; otherwise the
    published index is used (pinned to `revision` when given).

        bt.catalog.search("gripper.parallel", stroke_mm=150, payload_kg=2.3)
        bt.catalog.search("manipulator", reach_mm=900, payload_kg=6, level="V3")
        bt.catalog.search(kind="spec", category="structure.fence")
    """
    idx = _resolve_index(index, revision)
    return idx.search(category, kind=kind, manufacturer=manufacturer, level=level, text=text, limit=limit, **specs)


def search_for(row, *, category: Optional[str] = None, index: Union[Index, str, Path, None] = None, **extra: Any) -> list[Product]:
    """Candidates for one requirement row (`scene.requirements()["tool"]`):
    its category and every `>=` requirement become the filters; `extra`
    adds or overrides filters (`level="V3"`, `ip_rating="IP54"`)."""
    filters: dict[str, Any] = dict(row.minimum)
    options: dict[str, Any] = {}
    for key in ("kind", "manufacturer", "level", "text", "limit"):
        if key in extra:
            options[key] = extra.pop(key)
    filters.update(extra)
    return search(category or row.category or None, index=index, **options, **filters)


# ---------------------------------------------------------------- internals


def _resolve_index(index_: Union[Index, str, Path, None], revision: Optional[str]) -> Index:
    if isinstance(index_, Index):
        return index_
    if index_ is not None:
        return Index.from_path(index_)
    return index(revision=revision)


def _hub_index(revision: Optional[str]) -> Index:
    try:
        import huggingface_hub as hub
    except ImportError as e:
        raise ImportError(
            "the catalog needs the optional dependency `huggingface_hub` — install it with "
            "`pip install botrail[catalog]`"
        ) from e
    # Pin to a commit first (dataset_info takes no repo_type on hub 1.x), so
    # the index and any package fetched after it come from one snapshot.
    info = hub.dataset_info(REPO_ID, revision=revision)
    sha = str(info.sha)
    path = hub.hf_hub_download(REPO_ID, filename="index.json", repo_type="dataset", revision=sha)
    return Index.from_path(path, revision=sha)


def _hub_cache_dir() -> Path:
    explicit = os.environ.get("HF_HUB_CACHE")
    if explicit:
        return Path(explicit)
    home = os.environ.get("HF_HOME")
    return Path(home) / "hub" if home else Path.home() / ".cache" / "huggingface" / "hub"


def _cached_index(revision: Optional[str] = None) -> Optional[Index]:
    """The newest `index.json` among the dataset's cached snapshots — by its
    `generated_at`, then by commit — or the one at `revision`."""
    snapshots = _hub_cache_dir() / f"datasets--{REPO_ID.replace('/', '--')}" / "snapshots"
    if not snapshots.is_dir():
        return None
    best: Optional[tuple[tuple[str, str], Path]] = None
    for snap in sorted(snapshots.iterdir()):
        if revision is not None and snap.name != revision:
            continue
        file = snap / "index.json"
        if not file.is_file():
            continue
        try:
            with open(file, encoding="utf-8") as f:
                generated = str(json.load(f).get("generated_at") or "")
        except (OSError, ValueError):
            continue
        key = (generated, snap.name)
        if best is None or key > best[0]:
            best = (key, file)
    if best is None:
        return None
    return Index.from_path(best[1], revision=best[1].parent.name)


def _filter(key: str, value: Any) -> tuple[str, str, Any]:
    if key.endswith("__max"):
        return key[: -len("__max")], "<=", float(value)
    if key.endswith("__min"):
        return key[: -len("__min")], ">=", float(value)
    if key.endswith("__eq"):
        base = key[: -len("__eq")]
        return base, "==", value if isinstance(value, str) else float(value)
    if isinstance(value, str):
        return key, "==", value
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"spec filter {key}= must be a number or a string, not {type(value).__name__}")
    return key, ">=", float(value)


def _category_matches(category: str, query: str) -> bool:
    return category == query or category.startswith(query + ".")


def _subsequence(needle: list[str], haystack: list[str]) -> bool:
    i = 0
    for segment in haystack:
        if i < len(needle) and segment == needle[i]:
            i += 1
    return i == len(needle)


def _revision_number(product_id: str) -> int:
    tail = product_id.rsplit("/", 1)[-1]
    if tail.startswith("r") and tail[1:].isdigit():
        return int(tail[1:])
    return -1
