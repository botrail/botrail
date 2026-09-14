"""Vendors the generic shape library from botrail-assets into the package.

`python/botrail/_shapes/<name>.usda` is what `bt.parts.appearance` draws
from: one unit-box USD layer per shape, authored in
`botrail-assets/workshop-shapes/` (Node + three-usd-robot) and committed
here so `maturin develop`, the tests and the wheel need neither Node nor
the network. `SOURCES.json` records the assets commit and a digest per
file. Re-run after changing a shape upstream:

    .venv/bin/python scripts/sync_shapes.py [--assets ../botrail-assets]
    .venv/bin/python scripts/sync_shapes.py --check    # vendored == upstream?
"""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
DEST = REPO / "python" / "botrail" / "_shapes"
UPSTREAM = "workshop-shapes/usd"
REPOSITORY = "https://github.com/botrail/botrail-assets"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def git(assets: Path, *args: str) -> str:
    return subprocess.run(["git", "-C", str(assets), *args], check=True, capture_output=True, text=True).stdout.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--assets", default=str(REPO.parent / "botrail-assets"), help="a botrail-assets checkout")
    parser.add_argument("--check", action="store_true", help="verify the vendored copy matches the checkout")
    args = parser.parse_args()
    assets = Path(args.assets).resolve()
    source = assets / UPSTREAM
    layers = sorted(source.glob("*.usda"))
    if not layers:
        print(f"no shape layers under {source}", file=sys.stderr)
        return 2
    upstream = {p.name: digest(p) for p in layers}
    if args.check:
        vendored = {p.name: digest(p) for p in sorted(DEST.glob("*.usda"))}
        listed = json.loads((DEST / "SOURCES.json").read_text(encoding="utf-8"))["files"]
        stale = sorted(n for n in set(upstream) | set(vendored) if upstream.get(n) != vendored.get(n))
        unlisted = sorted(n for n in set(vendored) | set(listed) if vendored.get(n) != listed.get(n))
        for name in stale:
            print(f"stale: {name}", file=sys.stderr)
        for name in unlisted:
            print(f"SOURCES.json disagrees: {name}", file=sys.stderr)
        print(f"{len(vendored)} vendored shape layers {'match' if not (stale or unlisted) else 'DIFFER from'} {source}")
        return 1 if stale or unlisted else 0
    DEST.mkdir(parents=True, exist_ok=True)
    for old in DEST.glob("*.usda"):
        if old.name not in upstream:
            old.unlink()
    for path in layers:
        (DEST / path.name).write_bytes(path.read_bytes())
    dirty = bool(git(assets, "status", "--porcelain", "--", UPSTREAM))
    record = {
        "repository": REPOSITORY,
        "path": UPSTREAM,
        "commit": git(assets, "rev-parse", "HEAD"),
        "dirty": dirty,
        "synced": datetime.now(timezone.utc).date().isoformat(),
        "files": upstream,
    }
    (DEST / "SOURCES.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
    print(f"vendored {len(layers)} shape layers from {source} @ {record['commit'][:12]}{' (dirty)' if dirty else ''}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
