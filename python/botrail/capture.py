"""Headless camera video capture.

Drives the studio's in-browser exporter (the ``⤓ cam`` button) from
Python: serves the scene, opens a headless Chromium on a software
rasterizer, waits for the baked cycle to reach the page, starts the
deterministic seek export through the ``window.__STUDIO__`` automation
handle, and saves the resulting WebM — optionally converting to MP4 or
GIF with ffmpeg.

The dependencies (playwright with a fetched Chromium; ffmpeg for
conversions) are checked when :func:`record_camera` is called, not at
import time — the base wheel stays dependency-free.

    scene.add_camera("overview", position=(2.0, -1.6, 1.4), look_at=(0, 0, 0.4))
    scene.simulate_sequence("cycle")           # the bake to film
    from botrail import capture
    capture.record_camera(scene, "overview", "cycle.mp4")
"""

from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path
from typing import Union

__all__ = ["record_camera"]

# The same software-rasterizer flags the doc screenshots use — capture
# must work on CI boxes with no GPU.
CHROMIUM_ARGS = [
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--enable-unsafe-swiftshader",
]

#: Seconds for meshes to stream in after the canvas appears.
_SETTLE = 3.0


def record_camera(
    scene,
    camera: str,
    out: Union[str, Path],
    *,
    fps: int = 30,
    timeout: float = 600.0,
) -> Path:
    """Records ``camera``'s view of the scene's baked cycle into ``out``.

    Bake first (``simulate_sequence`` / ``simulate_sequences``, or play a
    recording) — the browser re-walks that cycle on a fixed ``fps`` grid,
    so the export never drops a frame and the same bake yields the same
    file. ``out`` decides the container: ``.webm`` is native; ``.mp4`` and
    ``.gif`` are converted with ffmpeg. Returns the output path.
    """
    try:
        from playwright.sync_api import sync_playwright
    except ImportError as e:  # dependency error at call time, by design
        raise RuntimeError(
            "botrail.capture needs playwright: pip install playwright && "
            "python -m playwright install chromium"
        ) from e
    import botrail as bt

    out = Path(out)
    if out.suffix not in (".webm", ".mp4", ".gif"):
        raise ValueError(
            f"{out.name}: unsupported container {out.suffix!r} — use .webm, .mp4 or .gif"
        )
    names = scene.camera_names
    if camera not in names:
        have = ", ".join(names) if names else "none — add one with scene.add_camera(...)"
        raise ValueError(f"no camera named {camera!r} in the scene (cameras: {have})")
    ffmpeg = _ffmpeg() if out.suffix in (".mp4", ".gif") else None

    out.parent.mkdir(parents=True, exist_ok=True)
    server = bt.studio(scene, block=False, open_browser=False)
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(args=CHROMIUM_ARGS)
            page = browser.new_page(viewport={"width": 1280, "height": 800})
            page.set_default_timeout(timeout * 1000)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            page.wait_for_timeout(_SETTLE * 1000)
            # The camera list and a baked cycle must both have arrived (a
            # bake from before the page opened reaches it via the
            # handshake replay).
            page.wait_for_function(
                "(name) => { const h = window.__STUDIO__;"
                " const s = h && h.getState();"
                " return !!(s && s.playback && s.cameras.some((c) => c.name === name)); }",
                arg=camera,
                timeout=30_000,
            )
            try:
                with page.expect_download(timeout=timeout * 1000) as dl:
                    page.evaluate(
                        "([name, fps]) => window.__STUDIO__.getState().beginCamExport(name, fps)",
                        [camera, fps],
                    )
                download = dl.value
            except Exception as e:
                raise RuntimeError(
                    "the camera export produced no file — the browser may lack "
                    "WebCodecs, or the export failed (see the page console)"
                ) from e
            if out.suffix == ".webm":
                download.save_as(out)
                webm = out
            else:
                tmp = Path(tempfile.mkdtemp(prefix="botrail-capture-")) / "cam.webm"
                download.save_as(tmp)
                webm = tmp
            browser.close()
    finally:
        server.stop()

    if out.suffix == ".mp4":
        assert ffmpeg is not None
        subprocess.run(
            [ffmpeg, "-y", "-loglevel", "error", "-i", str(webm),
             "-c:v", "libx264", "-pix_fmt", "yuv420p", "-movflags", "+faststart",
             str(out)],
            check=True,
        )
    elif out.suffix == ".gif":
        assert ffmpeg is not None
        subprocess.run(
            [ffmpeg, "-y", "-loglevel", "error", "-i", str(webm),
             "-vf", "split[a][b];[a]palettegen[p];[b][p]paletteuse",
             str(out)],
            check=True,
        )
    return out


def _ffmpeg() -> str:
    """An ffmpeg executable: the system one, else imageio-ffmpeg's."""
    found = shutil.which("ffmpeg")
    if found:
        return found
    try:
        import imageio_ffmpeg

        return imageio_ffmpeg.get_ffmpeg_exe()
    except ImportError as e:
        raise RuntimeError(
            "converting to .mp4/.gif needs ffmpeg — install it, or "
            "pip install imageio-ffmpeg, or write a .webm instead"
        ) from e
