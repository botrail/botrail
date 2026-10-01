#!/usr/bin/env python3
"""Verify the registry rope dependency and redistributed MIT notices."""
import json
from pathlib import Path
import subprocess
import tomllib

ROOT = Path(__file__).resolve().parents[1]
manifest = tomllib.loads((ROOT / "crates/botrail-rope/Cargo.toml").read_text())
dep = manifest["dependencies"]["rapier-rope"]
assert dep == {"version": "=0.1.0", "default-features": False, "features": ["f64"]}
assert not (ROOT / "vendor/rapier-rope").exists()
metadata = json.loads(subprocess.check_output(["cargo", "metadata", "--locked", "--format-version", "1"], cwd=ROOT))
package = next(p for p in metadata["packages"] if p["name"] == "rapier-rope")
assert package["version"] == "0.1.0" and package["source"].startswith("registry+")
source = Path(package["manifest_path"]).parent
assert package["license"] == "MIT"
for name in ("LICENSE", "THIRD_PARTY_NOTICES.md"):
    assert (ROOT / "python/botrail/_licenses/rapier-rope" / name).read_bytes() == (source / name).read_bytes()
print("PASS: crates.io rapier-rope 0.1.0, f64, no vendor copy; wheel MIT notices match")
