"""Launches the studio web UI for a scene."""

from __future__ import annotations

import os
import time
import webbrowser
from pathlib import Path
from typing import Optional

from . import _core


def _studio_dir() -> Path:
    env = os.environ.get("BOTRAIL_STUDIO_DIR")
    if env:
        directory = Path(env)
        if not (directory / "index.html").exists():
            raise FileNotFoundError(
                f"BOTRAIL_STUDIO_DIR={env} does not contain a built studio (index.html missing)"
            )
        return directory
    bundled = Path(__file__).resolve().parent / "_studio"
    if (bundled / "index.html").exists():
        return bundled
    raise FileNotFoundError(
        "studio assets not found. In a source checkout, build them with "
        "scripts/build_studio.sh, or point BOTRAIL_STUDIO_DIR at a built "
        "studio dist directory."
    )


def studio(
    scene: _core.Scene,
    *,
    host: str = "127.0.0.1",
    port: int = 0,
    open_browser: bool = True,
    block: bool = True,
    physics=None,
) -> Optional[_core.StudioServer]:
    """Serves the studio UI for ``scene`` and (by default) opens a browser.

    With ``block=True`` (default) this runs until Ctrl-C. With
    ``block=False`` it returns a :class:`StudioServer` handle; the server
    stops when the handle is garbage collected or ``stop()`` is called.

    ``physics`` is what the studio's physics toggle bakes under: ``None``
    (the default) the whole cell — ``bt.Physics(world=True)``, every
    obstacle and robot the engine's, ground at z = 0 —, a ``bt.Physics(...)``
    exactly that (``powered=False`` makes the toggle a power cut), ``False``
    no physics on this host (the toggle reports it).
    """
    server = _core.serve_studio(scene, str(_studio_dir()), host, port, physics)
    print(f"botrail studio running at {server.url}" + (" (Ctrl-C to stop)" if block else ""))
    if open_browser:
        webbrowser.open(server.url)
    if not block:
        return server
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    return None
