#!/usr/bin/env python3
"""Verify the registry rope dependency and redistributed MIT notices.

The version is whatever `crates/botrail-rope/Cargo.toml` pins: this checks
that it is an exact pin, that the lockfile resolves that same version from
crates.io (no path or git source, no vendor copy), that the crate is still
MIT, and that the notices shipped in the wheel match the crate's.
"""
import json
import re
from pathlib import Path
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[1]
manifest = tomllib.loads((ROOT / "crates/botrail-rope/Cargo.toml").read_text())
dep = manifest["dependencies"]["rapier-rope"]
pin = re.fullmatch(r"=(\d+\.\d+\.\d+)", dep.get("version", ""))
assert pin, f"rapier-rope must be pinned to an exact version (`=X.Y.Z`), not {dep.get('version')!r}"
version = pin.group(1)
assert dep.get("default-features") is False and dep.get("features") == ["f64"], dep
assert not (ROOT / "vendor/rapier-rope").exists()
metadata = json.loads(
    subprocess.check_output(["cargo", "metadata", "--locked", "--format-version", "1"], cwd=ROOT)
)
packages = [p for p in metadata["packages"] if p["name"] == "rapier-rope"]
assert len(packages) == 1, f"one rapier-rope in the lockfile, not {len(packages)}"
package = packages[0]
assert package["version"] == version, f"lockfile has rapier-rope {package['version']}, the manifest pins {version}"
assert (package["source"] or "").startswith("registry+"), package["source"]
source = Path(package["manifest_path"]).parent
assert package["license"] == "MIT", package["license"]
for name in ("LICENSE", "THIRD_PARTY_NOTICES.md"):
    shipped = ROOT / "python/botrail/_licenses/rapier-rope" / name
    assert shipped.read_bytes() == (source / name).read_bytes(), f"{shipped} differs from the crate's {name}"
print(f"PASS: crates.io rapier-rope {version}, f64, no vendor copy; wheel MIT notices match")
