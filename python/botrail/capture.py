"""Headless camera capture: video, and metric depth images.

Drives the studio from Python: serves the scene, opens a headless
Chromium on a software rasterizer, and works through the
``window.__STUDIO__`` automation handle. :func:`record_camera` runs the
in-browser exporter (the ``⤓ cam`` button) and saves the resulting WebM
— optionally converting to MP4 or GIF with ffmpeg. :func:`capture_depth`
takes one metric depth image (design/design-camera.md §12): Z along the
optical axis in meters, RealSense z16 conventions, with the pinhole
intrinsics alongside.

The dependencies (playwright with a fetched Chromium; ffmpeg for video
conversions; numpy for depth) are checked when the functions are called,
not at import time — the base wheel stays dependency-free.

    scene.add_camera("overview", position=(2.0, -1.6, 1.4), look_at=(0, 0, 0.4))
    scene.simulate_sequence("cycle")           # the bake to film
    from botrail import capture
    capture.record_camera(scene, "overview", "cycle.mp4")
    frame = capture.capture_depth(scene, "overview", "cycle_depth.npy", t=2.5)
"""

from __future__ import annotations

import base64
import json
import math
import shutil
import struct
import subprocess
import tempfile
import time
import zlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

__all__ = ["DepthFrame", "capture_depth", "capture_pointcloud", "record_camera"]

# The same software-rasterizer flags the doc screenshots use — capture
# must work on CI boxes with no GPU.
CHROMIUM_ARGS = [
    "--use-gl=angle",
    "--use-angle=swiftshader",
    "--enable-unsafe-swiftshader",
]

#: Seconds for meshes to stream in after the canvas appears.
_SETTLE = 3.0


def _check_quality(quality: str) -> None:
    if quality not in ("performance", "balanced", "high"):
        raise ValueError("quality must be 'performance', 'balanced', or 'high'")


def _page_quality(page, quality: str) -> None:
    # Set before Studio creates its renderer, just like a saved UI preference.
    page.add_init_script(
        "localStorage.setItem('botrail-studio.render-quality', "
        + json.dumps(quality) + ");"
    )


def record_camera(
    scene,
    camera: str,
    out: str | Path,
    *,
    fps: int = 30,
    depth: bool | str | Path = False,
    timeout: float = 600.0,
    quality: str = "balanced",
) -> Path:
    """Records ``camera``'s view of the scene's baked cycle into ``out``.

    Bake first (``simulate_sequence`` / ``simulate_sequences``, or play a
    recording) — the browser re-walks that cycle on a fixed ``fps`` grid,
    so the export never drops a frame and the same bake yields the same
    file. ``out`` decides the container: ``.webm`` is native; ``.mp4`` and
    ``.gif`` are converted with ffmpeg. Returns the output path.

    ``depth`` additionally records the metric depth stream on the same
    frame grid (RGBD; design/design-camera.md §12.4 DEP2) into an
    ``.npz`` — ``depth`` (frames, h, w) float32 meters (z16 conventions:
    0 = no return), ``times``, ``K``, ``near``/``far``/``fps``/
    ``camera``. Pass a path for the ``.npz``, or ``True`` to put it next
    to the video as ``<stem>_depth.npz``. Lossy video codecs never touch
    these values — the stream leaves the browser as raw float32.

    ``quality`` selects ``"performance"``, ``"balanced"`` (default), or
    ``"high"`` for shadows and edge smoothing. The camera's recording
    resolution and metric depth are independent of this choice.
    """
    _check_quality(quality)
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
    depth_out: Path | None = None
    if depth:
        try:
            import numpy  # noqa: F401 — assembles the .npz after the run
        except ImportError as e:
            raise RuntimeError(
                "record_camera(depth=...) needs numpy: pip install numpy"
            ) from e
        depth_out = (
            out.with_name(out.stem + "_depth.npz")
            if isinstance(depth, bool)
            else Path(depth)
        )
        if depth_out.suffix != ".npz":
            raise ValueError(
                f"{depth_out.name}: the depth stream is written as .npz"
            )
    _check_camera(scene, camera)
    ffmpeg = _ffmpeg() if out.suffix in (".mp4", ".gif") else None

    out.parent.mkdir(parents=True, exist_ok=True)
    server = bt.studio(scene, block=False, open_browser=False)
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(args=CHROMIUM_ARGS)
            page = browser.new_page(viewport={"width": 1280, "height": 800})
            _page_quality(page, quality)
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
            # The export finishes with one download (the .webm), or two
            # when the depth stream rides along — collect them all before
            # the browser goes away.
            want = 2 if depth_out is not None else 1
            downloads: list = []
            # (a lambda, not `downloads.append` — playwright's sync
            # wrapper sets attributes on the handler it is given)
            page.on("download", lambda d: downloads.append(d))
            page.evaluate(
                "([name, fps, depthData]) => window.__STUDIO__.getState()"
                ".beginCamExport(name, fps, depthData ? { depthData: true } : undefined)",
                [camera, fps, depth_out is not None],
            )
            deadline = time.monotonic() + timeout
            while len(downloads) < want and time.monotonic() < deadline:
                page.wait_for_timeout(250)
            if len(downloads) < want:
                raise RuntimeError(
                    "the camera export produced no file — the browser may lack "
                    "WebCodecs, or the export failed (see the page console)"
                )
            by_ext = {Path(d.suggested_filename).suffix: d for d in downloads}
            video = by_ext.get(".webm")
            if video is None:
                raise RuntimeError(
                    "the export produced no .webm (see the page console)"
                )
            if out.suffix == ".webm":
                video.save_as(out)
                webm = out
            else:
                tmp = Path(tempfile.mkdtemp(prefix="botrail-capture-")) / "cam.webm"
                video.save_as(tmp)
                webm = tmp
            if depth_out is not None:
                stream = by_ext.get(".bin")
                if stream is None:
                    raise RuntimeError(
                        "the export produced no depth stream (see the page console)"
                    )
                tmp_bin = Path(tempfile.mkdtemp(prefix="botrail-depth-")) / "depth.bin"
                stream.save_as(tmp_bin)
                _depth_npz(tmp_bin, depth_out)
                tmp_bin.unlink()
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


@dataclass(frozen=True)
class DepthFrame:
    """One metric depth image, RealSense z16 conventions.

    ``depth`` is an ``(h, w)`` float32 array of Z along the optical axis
    in meters — not euclidean ray length — with row 0 the top of the
    picture and 0 meaning no return (background, or outside
    ``[near, far]``; a camera from the catalog carries the product's
    measuring range there). ``position``/``quaternion`` (xyzw) are the
    camera's world pose at capture time — the extrinsics; the camera
    frame looks along -Z with +Y the image's up.
    """

    depth: Any  # numpy (h, w) float32
    camera: str
    t: float | None
    near: float
    far: float
    fx: float
    fy: float
    cx: float
    cy: float
    position: tuple[float, float, float]
    quaternion: tuple[float, float, float, float]
    #: The same view's color image, ``(h, w, 3)`` uint8 sRGB — present
    #: only when captured with ``rgb=True`` (depth and color come from
    #: one seek and one pose, so the pair never tears).
    rgb: Any | None = None

    @property
    def width(self) -> int:
        return int(self.depth.shape[1])

    @property
    def height(self) -> int:
        return int(self.depth.shape[0])

    @property
    def K(self) -> Any:
        """The 3x3 pinhole intrinsics matrix."""
        import numpy as np

        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]]
        )

    def meta(self) -> dict:
        """The sidecar dict: intrinsics, extrinsics and the depth
        conventions."""
        return {
            "camera": self.camera,
            "t": self.t,
            "width": self.width,
            "height": self.height,
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "near": self.near,
            "far": self.far,
            "position": list(self.position),
            "quaternion": list(self.quaternion),
            "units": "m",
            "invalid": 0,
            "png_depth_scale": 0.001,
        }

    def points(self, *, world: bool = True, stride: int = 1, color: bool = False) -> Any:
        """Unprojects the depth image into an ``(N, 3)`` float32 point
        cloud in meters; pixels with no return are dropped.

        ``world=True`` (the default) transforms through the camera's
        pose into scene coordinates; ``False`` keeps camera coordinates
        (-Z view, +Y up). ``stride`` keeps every stride-th pixel in both
        directions — a cheap decimation for display-sized clouds.
        ``color=True`` returns ``(N, 6)`` instead, columns 3:6 the
        pixel's sRGB in ``[0, 1]`` — needs a frame captured with
        ``rgb=True``.
        """
        import numpy as np

        if color and self.rgb is None:
            raise ValueError(
                "no color image on this frame — capture with rgb=True "
                "(capture_depth) or color=True (capture_pointcloud)"
            )
        d = np.asarray(self.depth)[::stride, ::stride]
        v, u = np.nonzero(d)
        z = d[v, u].astype(np.float64)
        # Pixel centers: index u covers [u, u+1), so its ray passes
        # through u + 0.5. Rows count down from the top, the camera's
        # +Y is up — hence the sign flip; -Z is the viewing direction.
        uu = u * stride + 0.5 - self.cx
        vv = v * stride + 0.5 - self.cy
        pts = np.stack([uu * z / self.fx, -vv * z / self.fy, -z], axis=1)
        if world:
            pts = pts @ _quat_matrix(self.quaternion).T + np.asarray(self.position)
        pts = pts.astype(np.float32)
        if not color:
            return pts
        rgb = np.asarray(self.rgb)[::stride, ::stride][v, u].astype(np.float32) / 255.0
        return np.hstack([pts, rgb])


def capture_depth(
    scene,
    camera: str,
    out: str | Path | None = None,
    *,
    t: float | None = None,
    rgb: bool = False,
    timeout: float = 120.0,
    quality: str = "balanced",
) -> DepthFrame:
    """Captures ``camera``'s metric depth image from the scene.

    ``t`` seeks a baked cycle to that time — bake first
    (``simulate_sequence``, or play a recording); ``None`` captures the
    scene as authored. ``out`` may be ``.npy`` (float32 meters) or
    ``.png`` (16-bit grayscale, millimeters — RealSense ``depth_scale``
    0.001, clipped at 65.535 m); either gets a ``.json`` sidecar with the
    intrinsics. Returns the :class:`DepthFrame` in every case.

    ``rgb=True`` also grabs the same view's color image onto
    :attr:`DepthFrame.rgb` — one seek, one pose, a true RGBD pair (the
    colors use the same output transform as the PiP). ``quality`` selects
    ``"performance"``, ``"balanced"`` (default), or ``"high"`` for that
    RGB image; metric depth and intrinsics stay the same.
    """
    _check_quality(quality)
    try:
        from playwright.sync_api import sync_playwright
    except ImportError as e:  # dependency error at call time, by design
        raise RuntimeError(
            "botrail.capture needs playwright: pip install playwright && "
            "python -m playwright install chromium"
        ) from e
    try:
        import numpy as np
    except ImportError as e:
        raise RuntimeError(
            "botrail.capture.capture_depth needs numpy: pip install numpy"
        ) from e
    import botrail as bt

    out = Path(out) if out is not None else None
    if out is not None and out.suffix not in (".npy", ".png"):
        raise ValueError(
            f"{out.name}: unsupported format {out.suffix!r} — use .npy "
            "(float32 meters) or .png (16-bit millimeters)"
        )
    _check_camera(scene, camera)

    server = bt.studio(scene, block=False, open_browser=False)
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(args=CHROMIUM_ARGS)
            page = browser.new_page(viewport={"width": 1280, "height": 800})
            _page_quality(page, quality)
            page.set_default_timeout(timeout * 1000)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            page.wait_for_timeout(_SETTLE * 1000)
            try:
                page.wait_for_function(
                    "([name, needsBake]) => { const h = window.__STUDIO__;"
                    " const s = h && h.getState();"
                    " return !!(s && s.cameras.some((c) => c.name === name)"
                    " && (!needsBake || s.playback)); }",
                    arg=[camera, t is not None],
                    timeout=30_000,
                )
            except Exception as e:
                hint = (
                    " — t was given: bake a cycle first (simulate_sequence, "
                    "or play a recording)"
                    if t is not None
                    else ""
                )
                raise RuntimeError(
                    f"the scene never became capturable in the page{hint}"
                ) from e
            r = page.evaluate(
                "([name, t, rgb]) =>"
                " window.__STUDIO__.getState().captureDepth(name, t, rgb)",
                [camera, t, rgb],
            )
            browser.close()
    finally:
        server.stop()

    w, h = int(r["width"]), int(r["height"])
    depth = (
        np.frombuffer(base64.b64decode(r["data"]), dtype="<f4")
        .reshape(h, w)
        .copy()
    )
    # Square pixels: the vertical fov is derived from the horizontal one
    # and the aspect (aimSensorCamera), so fy == fx exactly.
    fx = (w / 2.0) / math.tan(math.radians(float(r["fov_deg"])) / 2.0)
    color = (
        np.frombuffer(base64.b64decode(r["rgb"]), dtype=np.uint8)
        .reshape(h, w, 3)
        .copy()
        if rgb
        else None
    )
    frame = DepthFrame(
        depth=depth,
        camera=camera,
        t=t,
        near=float(r["near"]),
        far=float(r["far"]),
        fx=fx,
        fy=fx,
        cx=w / 2.0,
        cy=h / 2.0,
        position=tuple(float(x) for x in r["position"]),
        quaternion=tuple(float(x) for x in r["quaternion"]),
        rgb=color,
    )
    if out is not None:
        _save_depth(frame, out)
    return frame


def capture_pointcloud(
    scene,
    camera: str,
    out: str | Path | None = None,
    *,
    t: float | None = None,
    stride: int = 1,
    color: bool = False,
    timeout: float = 120.0,
    quality: str = "balanced",
) -> Any:
    """Captures ``camera``'s depth and unprojects it into a world-space
    point cloud (design/design-camera.md §12.4 DEP3).

    Returns an ``(N, 3)`` float32 array of scene-frame points in meters
    — depth pixels with no return are dropped, ``stride`` decimates, and
    ``t`` seeks a baked cycle as in :func:`capture_depth`. ``out``
    writes the cloud as a binary little-endian ``.ply``.

    ``color=True`` colors every point with its pixel's sRGB — the
    return grows to ``(N, 6)`` (rgb in ``[0, 1]``) and the ``.ply``
    carries ``uchar`` red/green/blue the way viewers expect.
    ``quality`` controls the RGB rendering as in :func:`capture_depth`.
    """
    out = Path(out) if out is not None else None
    if out is not None and out.suffix != ".ply":
        raise ValueError(f"{out.name}: point clouds are written as .ply")
    frame = capture_depth(scene, camera, t=t, rgb=color, timeout=timeout, quality=quality)
    pts = frame.points(world=True, stride=stride, color=color)
    if out is not None:
        _write_ply(out, pts)
    return pts


def _save_depth(frame: DepthFrame, out: Path) -> None:
    import numpy as np

    out.parent.mkdir(parents=True, exist_ok=True)
    if out.suffix == ".npy":
        np.save(out, frame.depth)
    else:  # ".png", validated by the caller
        mm = np.clip(np.rint(frame.depth * 1000.0), 0.0, 65535.0).astype(">u2")
        _write_png16(out, mm)
    out.with_suffix(".json").write_text(json.dumps(frame.meta(), indent=2) + "\n")


def _write_png16(path: Path, gray: Any) -> None:
    """Writes an ``(h, w)`` big-endian uint16 array as a 16-bit grayscale
    PNG — stdlib only, so the depth path adds no imaging dependency."""
    h, w = gray.shape

    def chunk(tag: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + tag
            + data
            + struct.pack(">I", zlib.crc32(tag + data) & 0xFFFFFFFF)
        )

    ihdr = struct.pack(">IIBBBBB", w, h, 16, 0, 0, 0, 0)
    raw = b"".join(b"\x00" + gray[i].tobytes() for i in range(h))
    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", ihdr)
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


def _depth_npz(bin_path: Path, npz_path: Path) -> None:
    """Turns the exporter's depth stream (a JSON header line, then the
    frames as raw little-endian float32) into the documented ``.npz``:
    ``depth`` (frames, h, w) meters, ``times`` on the export's fps grid,
    ``K``, ``near``/``far``/``fps``/``camera``."""
    import numpy as np

    raw = bin_path.read_bytes()
    nl = raw.index(b"\n")
    hdr = json.loads(raw[:nl].decode())
    n, h, w = int(hdr["frames"]), int(hdr["height"]), int(hdr["width"])
    payload = len(raw) - nl - 1
    if payload != n * h * w * 4:
        raise RuntimeError(
            f"depth stream is {payload} bytes, expected {n * h * w * 4} "
            f"({n} frames of {w}x{h} float32)"
        )
    frames = np.frombuffer(raw, dtype="<f4", offset=nl + 1).reshape(n, h, w)
    times = np.minimum(
        np.arange(n, dtype=np.float64) / float(hdr["fps"]), float(hdr["duration"])
    )
    fx = (w / 2.0) / math.tan(math.radians(float(hdr["fov_deg"])) / 2.0)
    K = np.array([[fx, 0.0, w / 2.0], [0.0, fx, h / 2.0], [0.0, 0.0, 1.0]])
    npz_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        npz_path,
        depth=frames,
        times=times,
        K=K,
        near=float(hdr["near"]),
        far=float(hdr["far"]),
        fps=float(hdr["fps"]),
        camera=str(hdr["camera"]),
    )


def _quat_matrix(q: tuple[float, float, float, float]) -> Any:
    """A 3x3 rotation matrix from an xyzw quaternion."""
    import numpy as np

    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def _write_ply(path: Path, pts: Any) -> None:
    """Writes an ``(N, 3)`` float32 array — or ``(N, 6)`` with sRGB in
    columns 3:6 — as a binary little-endian PLY (colors as the ``uchar``
    red/green/blue viewers expect). Stdlib only, same policy as the PNG
    writer."""
    import numpy as np

    colored = pts.shape[1] == 6
    header = (
        "ply\n"
        "format binary_little_endian 1.0\n"
        f"element vertex {len(pts)}\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        + ("property uchar red\nproperty uchar green\nproperty uchar blue\n" if colored else "")
        + "end_header\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as f:
        f.write(header.encode("ascii"))
        if colored:
            xyz = np.ascontiguousarray(pts[:, :3].astype("<f4")).view(np.uint8).reshape(len(pts), 12)
            rgb = np.clip(np.rint(pts[:, 3:] * 255.0), 0, 255).astype(np.uint8)
            f.write(np.hstack([xyz, rgb]).tobytes())
        else:
            f.write(np.ascontiguousarray(pts.astype("<f4")).tobytes())


def _check_camera(scene, camera: str) -> None:
    names = scene.camera_names
    if camera not in names:
        have = ", ".join(names) if names else "none — add one with scene.add_camera(...)"
        raise ValueError(f"no camera named {camera!r} in the scene (cameras: {have})")


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
