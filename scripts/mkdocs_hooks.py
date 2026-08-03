"""Publishes the repository's shared ``assets/`` directory into the built site.

The logo and screenshots are read from the repository root by the README, so
the docs site serves the same files rather than keeping a second copy under
``docs/``. Registered from ``mkdocs.yml`` via ``hooks:``.
"""

from __future__ import annotations

from pathlib import Path

from mkdocs.structure.files import File

ROOT = Path(__file__).resolve().parent.parent
ASSETS = ROOT / "assets"


def on_files(files, config):
    for path in sorted(ASSETS.rglob("*")):
        if not path.is_file():
            continue
        files.append(
            File(
                str(path.relative_to(ROOT)),
                src_dir=str(ROOT),
                dest_dir=config["site_dir"],
                use_directory_urls=config["use_directory_urls"],
            )
        )
    return files
