"""SimReady Central assets as botrail scenery.

NVIDIA's SimReady Central is an open library of simulation-ready OpenUSD
assets — thousands of props and a few vendor robots and grippers, each
package hosted in its publisher's Hugging Face dataset and pinned to a
dataset commit by the library's index (the Space
`nvidia/simready-central`). botrail reads them with `scene.load_usd`
unchanged: metres, Z-up, the geometry as obstacles, the mass the package
authored, the textures and MDL look in the studio.

This module finds packages and brings one to disk;
:func:`botrail.parts.simready` stands it in a cell. :func:`search` asks the
library's search service the way a person would — words, a size, a license —
and returns hits that say what they are; :func:`fetch` takes a hit, a
Hugging Face URL as the library hands it out (`.../blob/<commit>/<asset>/...`),
a `(repo, asset, commit)` triple, or a package directory already on disk:

    hits = bt.simready.search("blue plastic crate", max_size=(0.7, 0.5, 0.4), limit=5)
    hits[0].name, hits[0].size_m, hits[0].mass_kg, hits[0].license
    pkg = bt.simready.fetch(hits[0])
    scene.load_usd(pkg.root, prefix="crate")
    pkg.license, pkg.source      # 'CC-BY-4.0', the URL at its commit

Downloads go through `huggingface_hub` (`pip install botrail[catalog]`) into
its cache, pinned to the commit, without the package's thumbnails and the
CAD and texture copies under `source_assets/`. The library's assets are not
botrail catalog products: they carry no part number, no specs and no
mounting, so a cell records them by source URL and license on the part
(`set_part(source=..., license=...)`), and the catalog stays the list of
what is ordered.
"""

from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.parse
import urllib.request
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Optional, Union

__all__ = ["Hit", "PACKAGING", "Package", "SEARCH_URL", "fetch", "parse_url", "search"]

#: The library's search service (the Space's API; `BOTRAIL_SIMREADY_URL` overrides).
SEARCH_URL = "https://nvidia-simready-central.hf.space"
PACKAGING = "com.nvidia.simready.packaging.json"
ROOTS = ".metadata/com.nvidia.simready.root_usds.json"
# Package parts botrail never reads: the thumbnails, and the source CAD and
# texture copies under `source_assets/` — but not its MDL modules, which the
# materials import and the studio draws with.
SKIPPED = (
    "simready_usd/.thumbs/**",
    *(
        f"simready_usd/source_assets/*.{ext}"
        for ext in (
            "png", "jpg", "jpeg", "exr", "tga", "tif", "tiff", "bmp", "hdr", "dds",
            "fbx", "obj", "stl", "step", "stp", "iges", "igs", "glb", "gltf", "blend",
            "usd", "usda", "usdc", "usdz", "zip", "7z",
        )
    ),
)

_HF_URL = re.compile(
    r"^https?://huggingface\.co/datasets/(?P<repo>[^/]+/[^/]+)/(?:blob|resolve|tree)/"
    r"(?P<revision>[^/]+)/(?P<asset>[^/]+)(?:/(?P<rest>.+))?$"
)
_PACKAGE_ID = re.compile(r"^simready\.hf\.(?P<publisher>[^.]+)\.(?P<dataset>[^.]+)\.")

Ref = Union[str, "os.PathLike[str]", tuple[str, str, Optional[str]], "Hit", "Package"]


@dataclass
class Package:
    """A SimReady package on disk — what `scene.load_usd(package.root)` reads."""

    root: Path
    """The root USD layer (`simready_usd/<asset>.usd`)."""
    directory: Path
    """The package folder: the packaging manifest, `.metadata/`, `simready_usd/`."""
    asset: str
    """The package's logical asset name — its folder in the dataset."""
    repo: Optional[str] = None
    """The Hugging Face dataset it came from, `None` for a local folder."""
    revision: Optional[str] = None
    """The dataset commit the files are from."""
    license: Optional[str] = None
    """The SPDX id the packaging manifest declares (`CC-BY-4.0`, ...)."""
    publisher: Optional[str] = None
    """Who published it — the dataset's namespace (`nvidia`, `Robotiq-Official`)."""
    package_id: Optional[str] = None
    """The manifest's `package_id`, which carries the publisher's version."""

    @property
    def source(self) -> str:
        """What a part records: the root layer's URL at its commit, or its path."""
        if self.repo and self.revision:
            rel = self.root.relative_to(self.directory).as_posix()
            return f"https://huggingface.co/datasets/{self.repo}/blob/{self.revision}/{self.asset}/{rel}"
        return str(self.root)


@dataclass
class Hit:
    """One search result: enough to read, to judge, and to fetch and pin."""

    url: str
    """The root layer's URL at the dataset commit the library indexed."""
    repo: str
    revision: str
    asset: str
    path: Optional[str]
    """The root layer's path inside the asset folder."""
    name: str
    """The asset's name (`sm_crate_blue_a01_01`, `Robotiq_2F_85`)."""
    score: float
    size_m: Optional[tuple[float, float, float]] = None
    """The bounding box of the root prim, metres, as the library measured it."""
    mass_kg: Optional[float] = None
    license: Optional[str] = None
    publisher: Optional[str] = None
    """The dataset's namespace (`nvidia`, `Robotiq-Official`)."""
    author: Optional[str] = None
    """Who the package names as its author (`NVIDIA`, `Robotiq`)."""
    kind: Optional[str] = None
    """`prop`, `robot` or `gripper` — the package's `asset_type`."""
    profile: Optional[str] = None
    """The SimReady validation profile (`Robotics-Prop`, `Robot-Body`, `Robot-Gripper`)."""
    label: Optional[str] = None
    """The Wikidata class the package is labelled with (`crate`, `pallet`)."""
    qcode: Optional[str] = None
    caption: Optional[str] = None
    """What the library's vision model saw in the thumbnail."""
    properties: dict[str, Any] = field(default_factory=dict)
    """Every `simready_metadata_*` and semantic property the index holds, as returned."""

    @property
    def source(self) -> str:
        return self.url


def search(
    text: str,
    *,
    limit: int = 10,
    max_size: Optional[tuple[Optional[float], Optional[float], Optional[float]]] = None,
    min_size: Optional[tuple[Optional[float], Optional[float], Optional[float]]] = None,
    max_mass_kg: Optional[float] = None,
    license: Optional[str] = None,
    kind: Optional[str] = None,
    profile: Optional[str] = None,
    publisher: Optional[str] = None,
    url: Optional[str] = None,
    timeout: float = 60.0,
) -> list[Hit]:
    """Packages the library's search service ranks for `text`, best first.

    `text` is what the thing is, in words (`"blue plastic crate"`,
    `"two-finger gripper"`); the service matches it against captions,
    labels and names. The rest narrows: `max_size` / `min_size` bound the
    root prim's bounding box in metres (`None` leaves an axis open —
    `max_size=(0.7, 0.5, None)` caps the footprint only), `max_mass_kg`
    the authored mass, `license` a substring of the declared license
    (`"CC-BY"` takes the library's three spellings), `kind` the package's
    type (`"prop"`, `"robot"`, `"gripper"`), `profile` the validation
    profile (`"Robotics-Prop"`, `"Robot-Body"`, `"Robot-Gripper"`) and
    `publisher` the dataset namespace (`"nvidia"`). The service is the Space
    behind the library (`SEARCH_URL`, or `BOTRAIL_SIMREADY_URL` / `url`);
    when it cannot be reached this raises `ValueError("cannot reach ...")`."""
    if not text.strip():
        raise ValueError("search needs words to look for")
    params: dict[str, str] = {
        "description": text.strip(),
        "limit": str(int(limit)),
        "return_root_prims": "true",
        "return_metadata": "true",
        "return_vision_generated_metadata": "true",
    }
    for key, size in (("max", max_size), ("min", min_size)):
        if size is None:
            continue
        for axis, value in zip("xyz", size):
            if value is not None:
                if value <= 0:
                    raise ValueError(f"{key}_size must be positive, got {value}")
                params[f"{key}_bbox_{axis}"] = repr(float(value))
    filters: list[str] = []
    if license:
        filters.append(f"simready_metadata_asset_license=~*{license}*")
    if kind:
        filters.append(f"simready_metadata_asset_type={kind}")
    if profile:
        filters.append(f"simready_metadata_validation__profile={profile}")
    if filters:
        params["filter_by_properties"] = ",".join(filters)
    if max_mass_kg is not None:
        params["filter_by_properties_numeric"] = f"simready_metadata_mass<={float(max_mass_kg)!r}"
    if publisher:
        params["filter_url_regexp"] = f".*/datasets/{re.escape(publisher)}/.*"
    base = (url or os.environ.get("BOTRAIL_SIMREADY_URL") or SEARCH_URL).rstrip("/")
    data = _get_json(f"{base}/search?{urllib.parse.urlencode(params)}", timeout)
    if not isinstance(data, list):
        raise ValueError(f"unexpected answer from {base}: {str(data)[:200]}")
    return [hit for item in data if (hit := _hit(item)) is not None]


def _get_json(url: str, timeout: float) -> Any:
    request = urllib.request.Request(url, headers={"Accept": "application/json", "User-Agent": "botrail"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:  # noqa: S310 — the library's own service
            return json.loads(response.read().decode("utf-8"))
    except (urllib.error.URLError, TimeoutError, OSError, ValueError) as exc:
        raise ValueError(f"cannot reach SimReady Central at {url.split('?', 1)[0]}: {exc}") from exc


def _hit(item: Any) -> Optional[Hit]:
    """A `Hit` from one search result, `None` for a result that names no package."""
    if not isinstance(item, dict) or not isinstance(item.get("url"), str):
        return None
    try:
        repo, revision, asset, path = parse_url(item["url"])
    except ValueError:
        return None
    prim = next((p for p in item.get("root_prims") or [] if isinstance(p, dict)), {})
    properties = prim.get("properties") if isinstance(prim.get("properties"), dict) else {}
    meta = {k[len("simready_metadata_"):]: v for k, v in properties.items() if k.startswith("simready_metadata_")}
    size = None
    dims = [prim.get(f"scaled_bbox_dimension_{axis}") for axis in "xyz"]
    if all(isinstance(d, (int, float)) for d in dims):
        size = (float(dims[0]), float(dims[1]), float(dims[2]))
    mass = _number(meta.get("mass"))
    caption = None
    vision = item.get("vision_generated_metadata")
    if isinstance(vision, dict):
        for entries in vision.values():
            for entry in entries if isinstance(entries, list) else []:
                if isinstance(entry, dict) and entry.get("name") == "caption":
                    values = entry.get("value_text") or entry.get("value_sayt") or []
                    if values and isinstance(values[0], str):
                        caption = values[0]
    label = properties.get("semantic:wikidata_class:params:semanticData")
    name = meta.get("asset_name") or asset
    return Hit(
        url=item["url"],
        repo=repo,
        revision=revision,
        asset=asset,
        path=path,
        name=str(name),
        score=float(item.get("score") or 0.0),
        size_m=size,
        mass_kg=mass,
        license=_text(meta.get("asset_license")),
        publisher=repo.split("/", 1)[0],
        author=_text(meta.get("author")),
        kind=_text(meta.get("asset_type")),
        profile=_text(meta.get("validation__profile")),
        label=_text(label),
        qcode=_text(meta.get("qcode")),
        caption=caption,
        properties={k: v for k, v in properties.items() if "validated_features" not in k},
    )


def _number(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if number == number else None  # NaN is not a number here


def _text(value: Any) -> Optional[str]:
    return value if isinstance(value, str) and value else None


def parse_url(url: str) -> tuple[str, str, str, Optional[str]]:
    """`(repo, revision, asset, path)` of a dataset URL the library hands out —
    `blob`, `resolve` or `tree`, the path inside the asset when the URL names a file."""
    m = _HF_URL.match(url.strip())
    if m is None:
        raise ValueError(
            "not a SimReady package URL (expected https://huggingface.co/datasets/<ns>/<dataset>/"
            f"blob/<commit>/<asset>/...): {url}"
        )
    return m["repo"], m["revision"], m["asset"], m["rest"]


def fetch(ref: Ref, *, cache_dir: Union[str, "os.PathLike[str]", None] = None) -> Package:
    """The package `ref` names, on disk.

    `ref` is a :class:`Hit` from :func:`search`, a Hugging Face URL of the
    package or of a file in it, a `(repo, asset, revision)` triple
    (`revision` `None` takes the dataset's current commit, which the
    returned package then records), or a path already on disk — the
    package folder, or its root USD. A download needs `huggingface_hub`
    and raises `ValueError("cannot reach ...")` when the hub cannot be
    reached and the cache has no copy."""
    if isinstance(ref, Package):
        return ref
    if isinstance(ref, Hit):
        return _download(ref.repo, ref.asset, ref.revision, ref.path, cache_dir)
    if isinstance(ref, tuple):
        repo, asset, revision = ref
        return _download(repo, asset, revision, None, cache_dir)
    text = os.fspath(ref)
    if text.startswith(("http://", "https://")):
        repo, revision, asset, rest = parse_url(text)
        return _download(repo, asset, revision, rest, cache_dir)
    return _local(Path(text))


def _download(repo: str, asset: str, revision: Optional[str], rest: Optional[str], cache_dir) -> Package:
    try:
        from huggingface_hub import snapshot_download
    except ImportError as exc:
        raise ImportError(
            "fetching a SimReady package needs huggingface_hub: pip install 'botrail[catalog]'"
        ) from exc
    try:
        snapshot = Path(
            snapshot_download(
                repo_id=repo,
                repo_type="dataset",
                revision=revision,
                allow_patterns=[f"{asset}/**"],
                ignore_patterns=[f"{asset}/{pattern}" for pattern in SKIPPED],
                cache_dir=None if cache_dir is None else os.fspath(cache_dir),
            )
        )
    except Exception as exc:  # offline, unknown dataset, gated — one message to skip on
        raise ValueError(f"cannot reach the SimReady dataset {repo}@{revision or 'main'}: {exc}") from exc
    directory = snapshot / asset
    if not directory.is_dir():
        raise ValueError(f"{repo}@{revision or 'main'} has no package folder {asset!r}")
    # The snapshot folder is named by the commit it holds.
    commit = snapshot.name if re.fullmatch(r"[0-9a-f]{40}", snapshot.name) else revision
    package = _local(directory, hint=rest)
    package.repo = repo
    package.revision = commit
    package.publisher = package.publisher or repo.split("/", 1)[0]
    return package


def _local(path: Path, *, hint: Optional[str] = None) -> Package:
    """A package folder (or its root USD) already on disk."""
    path = Path(path)
    if path.is_file():
        directory = _package_dir(path.parent) or path.parent
        root = path
    elif path.is_dir():
        directory = _package_dir(path) or path
        root = _root_layer(directory, hint)
    else:
        raise FileNotFoundError(f"no SimReady package at {path}")
    manifest = _manifest(directory)
    package_id = manifest.get("package_id") if isinstance(manifest.get("package_id"), str) else None
    publisher = None
    if package_id and (m := _PACKAGE_ID.match(package_id)):
        publisher = m["publisher"]
    license = manifest.get("license") if isinstance(manifest.get("license"), str) else None
    return Package(
        root=root,
        directory=directory,
        asset=directory.name,
        license=license,
        publisher=publisher,
        package_id=package_id,
    )


def _package_dir(start: Path) -> Optional[Path]:
    """The nearest folder at or above `start` holding the packaging manifest."""
    for candidate in (start, *start.parents[:3]):
        if (candidate / PACKAGING).is_file():
            return candidate
    return None


def _manifest(directory: Path) -> dict:
    path = directory / PACKAGING
    if not path.is_file():
        return {}
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def _root_layer(directory: Path, hint: Optional[str]) -> Path:
    """The package's root USD: what its `.metadata` names, else the file the
    reference named, else the one layer beside `simready_usd/`'s payloads."""
    roots = directory / ROOTS
    if roots.is_file():
        try:
            entries = json.loads(roots.read_text(encoding="utf-8")).get("entries") or []
        except (OSError, ValueError, AttributeError):
            entries = []
        for entry in entries:
            if isinstance(entry, str) and (directory / entry).is_file():
                return directory / entry
    if hint and (directory / hint).is_file() and hint.lower().endswith((".usd", ".usda", ".usdc", ".usdz")):
        return directory / hint
    layers = sorted(
        p for p in (directory / "simready_usd").glob("*") if p.is_file() and p.suffix.lower() in (".usd", ".usda", ".usdc", ".usdz")
    )
    if len(layers) == 1:
        return layers[0]
    if not layers:
        raise ValueError(f"{directory}: no root USD layer under simready_usd/")
    raise ValueError(f"{directory}: several root layers, name one: {[p.name for p in layers]}")
