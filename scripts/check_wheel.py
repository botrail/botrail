"""Checks that a built wheel ships what `pip install botrail` relies on
besides the extension: the studio bundle (`botrail/_studio`, the output of
scripts/build_studio.sh) and the shape library (`botrail/_shapes/*.usda`).

maturin walks the python package with `.gitignore` applied, so a file an
ignore rule matches is left out of the wheel without a word — the built
studio, or `*.usda` even when git tracks it — while `maturin develop`
(editable) keeps serving the working tree and never notices. release.yml
and ci.yml run this on the wheel they just built:

    python scripts/check_wheel.py dist/*.whl

Each wheel is unpacked into a scratch directory and imported from there by
an isolated interpreter (`-I -S`: no site-packages, no environment), so an
editable install in the current environment cannot stand in for it.
"""

from __future__ import annotations

import glob
import subprocess
import sys
import tempfile
import zipfile
from pathlib import Path

# Runs in the isolated interpreter with the unpacked wheel as argv[1].
PROBE = r"""
import os
import sys
from pathlib import Path

root = Path(sys.argv[1]).resolve()
sys.path.insert(0, str(root))
os.environ.pop("BOTRAIL_STUDIO_DIR", None)  # the bundled copy, not a checkout's

import botrail as bt
from botrail._launcher import _studio_dir

package = Path(bt.__file__).resolve().parent
if package.parent != root:
    sys.exit(f"imported botrail from {package}, not from the wheel")
try:
    studio = _studio_dir()
except FileNotFoundError as error:
    sys.exit(f"studio bundle missing from the wheel: {error}")
missing = [name for name in bt.parts.SHAPES if not bt.parts.shape_path(name).is_file()]
if missing:
    sys.exit(f"shape library incomplete in the wheel: missing {', '.join(missing)}")
assets = [path for path in studio.rglob("*") if path.is_file()]
print(f"botrail {bt.__version__}: studio ({len(assets)} files), {len(bt.parts.SHAPES)} shapes")
"""


def check(wheel: Path) -> bool:
    with tempfile.TemporaryDirectory() as scratch:
        with zipfile.ZipFile(wheel) as archive:
            archive.extractall(scratch)
        result = subprocess.run(
            [sys.executable, "-I", "-S", "-c", PROBE, scratch],
            capture_output=True,
            text=True,
            check=False,  # the verdict is printed below
        )
    if result.returncode == 0:
        print(f"ok  {wheel.name}: {result.stdout.strip()}")
        return True
    output = result.stderr.strip() or result.stdout.strip() or "no output"
    print(f"BAD {wheel.name}: {output.splitlines()[-1]}")
    return False


def main(argv: list[str]) -> int:
    wheels = [Path(path) for pattern in argv for path in sorted(glob.glob(pattern))]
    if not wheels:
        print("usage: check_wheel.py dist/*.whl (no wheel found)", file=sys.stderr)
        return 2
    # Every wheel gets its verdict, not only the first bad one.
    verdicts = [check(wheel) for wheel in wheels]
    return 0 if all(verdicts) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
