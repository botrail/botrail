"""CI smoke for botrail.capture: a short cycle, recorded headless.

Needs playwright with a fetched Chromium; skips (not errors) without them —
the base wheel must stay importable and testable with no browser at all.
"""

import json
import shutil
import subprocess
from pathlib import Path

import pytest

import botrail as bt
from botrail import capture

pytest.importorskip("playwright.sync_api")

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"


@pytest.fixture(scope="module")
def chromium():
    from playwright.sync_api import sync_playwright

    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(args=capture.CHROMIUM_ARGS)
            browser.close()
    except Exception as e:  # noqa: BLE001 — any launch failure means "no browser here"
        pytest.skip(f"chromium unavailable: {e}")


def test_record_camera_smoke(chromium, tmp_path) -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_camera(
        "cam",
        position=(1.2, -1.0, 0.9),
        look_at=(0.0, 0.0, 0.3),
        fov=60,
        resolution=(320, 180),
    )
    sq = scene.sequence("cycle")
    sq.step("a", transition=bt.seq.elapsed(1.0))
    sq.step("b", transition=bt.seq.elapsed(1.0))
    tl = sq.simulate()

    out = capture.record_camera(scene, "cam", tmp_path / "cam.webm", fps=8)
    assert out.exists() and out.stat().st_size > 1000

    # The deterministic grid: round(duration * fps) + 1 frames, exactly.
    ffprobe = shutil.which("ffprobe")
    if ffprobe:
        frames = subprocess.run(
            [ffprobe, "-v", "error", "-select_streams", "v", "-count_frames",
             "-show_entries", "stream=nb_read_frames", "-of", "csv=p=0", str(out)],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        expected = max(2, round(tl.duration * 8) + 1)
        assert int(frames) == expected, f"{frames} frames, expected {expected}"


def test_record_camera_rgbd(chromium, tmp_path) -> None:
    """§12.4 DEP2 acceptance: the depth stream walks the video's exact
    frame grid — same count, same times — and carries metric values."""
    np = pytest.importorskip("numpy")

    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    joint = scene.robot.joint_names[0]
    scene.add_camera(
        "cam", position=(1.4, -1.2, 0.9), look_at=(0, 0, 0.4), fov=60,
        resolution=(320, 180),
    )
    sq = scene.sequence("cycle")
    sq.step("swing", [bt.seq.ramp({joint: 1.2}, 1.5)], transition=bt.seq.done())
    tl = sq.simulate()

    out = capture.record_camera(scene, "cam", tmp_path / "rgbd.webm", fps=8, depth=True)
    npz_path = tmp_path / "rgbd_depth.npz"
    assert out.exists() and out.stat().st_size > 1000
    assert npz_path.exists()

    data = np.load(npz_path)
    n = data["depth"].shape[0]
    assert n == max(2, round(tl.duration * 8) + 1)
    assert data["depth"].shape == (n, 180, 320)
    assert data["depth"].dtype == np.float32
    assert np.allclose(
        data["times"], np.minimum(np.arange(n) / 8.0, tl.duration)
    )
    fx = (320 / 2.0) / np.tan(np.radians(60.0) / 2.0)
    assert abs(data["K"][0, 0] - fx) < 1e-9
    assert data["K"][1, 1] == data["K"][0, 0]
    assert float(data["near"]) == 0.05 and float(data["far"]) == 30.0
    # Real returns, and the swing shows up in the stream.
    assert float(data["depth"].max()) > 0.0
    assert not np.array_equal(data["depth"][0], data["depth"][-1])

    # The video walked the same grid: identical frame count.
    ffprobe = shutil.which("ffprobe")
    if ffprobe:
        frames = subprocess.run(
            [ffprobe, "-v", "error", "-select_streams", "v", "-count_frames",
             "-show_entries", "stream=nb_read_frames", "-of", "csv=p=0", str(out)],
            capture_output=True, text=True, check=True,
        ).stdout.strip()
        assert int(frames) == n


def test_record_camera_refusals(tmp_path) -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    with pytest.raises(ValueError, match="no camera named"):
        capture.record_camera(scene, "ghost", tmp_path / "x.webm")
    scene.add_camera("cam")
    with pytest.raises(ValueError, match="unsupported container"):
        capture.record_camera(scene, "cam", tmp_path / "x.avi")


# ---------------------------------------------------------------- depth


def _decode_png16(path: Path):
    """Reads back the 16-bit grayscale PNG the writer produces (own
    format, own reader — the round-trip check needs no imaging lib)."""
    import struct
    import zlib

    import numpy as np

    data = path.read_bytes()
    assert data[:8] == b"\x89PNG\r\n\x1a\n"
    pos, idat, w, h = 8, b"", 0, 0
    while pos < len(data):
        (n,) = struct.unpack(">I", data[pos : pos + 4])
        tag = data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + n]
        if tag == b"IHDR":
            w, h, depth_bits, color = struct.unpack(">IIBB", body[:10])
            assert (depth_bits, color) == (16, 0)
        elif tag == b"IDAT":
            idat += body
        pos += 12 + n
    rows = zlib.decompress(idat)
    stride = 1 + w * 2  # filter byte + big-endian u16 row
    assert all(rows[i * stride] == 0 for i in range(h))
    pix = b"".join(rows[i * stride + 1 : (i + 1) * stride] for i in range(h))
    return np.frombuffer(pix, dtype=">u2").reshape(h, w)


def test_capture_depth_numbers(chromium, tmp_path) -> None:
    """§12.4 DEP1 acceptance: Z-depth semantics, clipping, K, files."""
    np = pytest.importorskip("numpy")

    scene = bt.Scene()
    # A fronto-parallel wall, near face 1.5 m out: Z-depth means every
    # wall pixel reads the same 1.500, edges included.
    scene.add_box("wall", size=(0.1, 4.0, 4.0), position=(1.55, 0.0, 1.0))
    scene.add_camera(
        "wall", position=(0, 0, 1.0), look_at=(2, 0, 1.0), fov=60, resolution=(160, 120)
    )
    # A 0.6 m plate 2 m out in its own lane (y=10), for the reprojection
    # and the clipping checks. Its near face sits at exactly x=2.0.
    scene.add_box("plate", size=(0.1, 0.6, 0.6), position=(2.05, 10.0, 1.0))
    view = {"position": (0, 10.0, 1.0), "look_at": (2, 10.0, 1.0), "fov": 60}
    scene.add_camera("plate", resolution=(160, 120), **view)
    scene.add_camera("clipnear", resolution=(64, 48), near=2.5, **view)
    scene.add_camera("clipfar", resolution=(64, 48), far=1.5, **view)

    # (a) constant Z across the picture, ±1 mm.
    f = capture.capture_depth(scene, "wall", tmp_path / "wall.npy")
    d = f.depth
    h, w = d.shape
    for r, c in [(h // 2, w // 2), (h // 2, 3), (h // 2, w // 4), (h // 4, w // 2)]:
        assert abs(float(d[r, c]) - 1.5) <= 0.001, (r, c, float(d[r, c]))
    # .npy + sidecar round-trip.
    assert np.array_equal(np.load(tmp_path / "wall.npy"), d)
    meta = json.loads((tmp_path / "wall.json").read_text())
    assert meta["width"] == 160 and meta["invalid"] == 0
    assert abs(meta["fx"] - f.fx) < 1e-9 and meta["units"] == "m"

    # (d) the plate's pixel edges land where K says, within 1 px.
    fp = capture.capture_depth(scene, "plate", tmp_path / "plate.png")
    mask = np.abs(fp.depth - 2.0) < 0.01
    cols = np.where(mask.any(axis=0))[0]
    rows = np.where(mask.any(axis=1))[0]
    half = fp.fx * 0.3 / 2.0  # 0.3 m half-extent projected at 2 m
    assert abs(cols.min() - (fp.cx - half)) <= 1.0
    assert abs(cols.max() + 1 - (fp.cx + half)) <= 1.0
    assert abs(rows.min() - (fp.cy - half)) <= 1.0
    assert abs(rows.max() + 1 - (fp.cy + half)) <= 1.0
    # 16-bit PNG round-trip: millimeters, exactly.
    png = _decode_png16(tmp_path / "plate.png")
    assert png.shape == fp.depth.shape
    assert np.array_equal(
        png.astype(np.float64),
        np.clip(np.rint(fp.depth.astype(np.float64) * 1000.0), 0.0, 65535.0),
    )

    # (b) outside [near, far] -> no return: the same plate is invisible
    # both to a camera whose near is beyond it and to one whose far is
    # short of it.
    for name in ("clipnear", "clipfar"):
        fc = capture.capture_depth(scene, name)
        assert float(fc.depth[fc.height // 2, fc.width // 2]) == 0.0, name


def test_capture_depth_seek(chromium) -> None:
    """§12.4 DEP1 acceptance: same bake + same t -> bit identical; the
    seek really moves the scene."""
    np = pytest.importorskip("numpy")

    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    joint = scene.robot.joint_names[0]
    scene.add_camera(
        "cam", position=(1.4, -1.2, 0.9), look_at=(0, 0, 0.4), fov=60,
        resolution=(160, 120),
    )
    sq = scene.sequence("cycle")
    sq.step("swing", [bt.seq.ramp({joint: 1.4}, 1.0)], transition=bt.seq.done())
    sq.simulate()

    start = capture.capture_depth(scene, "cam", t=0.0)
    mid1 = capture.capture_depth(scene, "cam", t=1.0)
    mid2 = capture.capture_depth(scene, "cam", t=1.0)
    assert np.array_equal(mid1.depth, mid2.depth)
    assert not np.array_equal(start.depth, mid1.depth)


def test_capture_depth_refusals(tmp_path) -> None:
    pytest.importorskip("numpy")
    scene = bt.Scene()
    with pytest.raises(ValueError, match="no camera named"):
        capture.capture_depth(scene, "ghost")
    scene.add_camera("cam")
    with pytest.raises(ValueError, match="unsupported format"):
        capture.capture_depth(scene, "cam", tmp_path / "d.exr")
    with pytest.raises(ValueError, match="written as .ply"):
        capture.capture_pointcloud(scene, "cam", tmp_path / "d.xyz")


def test_depth_frame_points_math() -> None:
    """The unprojection: pixel centers through K, -Z view / +Y up, pose
    transform, stride, invalid pixels dropped — no browser involved."""
    np = pytest.importorskip("numpy")

    d = np.full((4, 4), 2.0, dtype=np.float32)
    d[1, 2] = 0.0  # one lost return
    base = {"depth": d, "camera": "c", "t": None, "near": 0.1, "far": 10.0,
            "fx": 2.0, "fy": 2.0, "cx": 2.0, "cy": 2.0}

    f = capture.DepthFrame(**base, position=(0, 0, 0), quaternion=(0, 0, 0, 1))
    pts = f.points()
    assert pts.shape == (15, 3)  # 16 pixels, the invalid one dropped
    # Pixel (0,0): center offset (-1.5, -1.5) px at 2 m with fx=fy=2
    # -> camera coords (-1.5, +1.5, -2) (rows run down, +Y is up).
    assert np.allclose(pts[0], [-1.5, 1.5, -2.0])
    assert np.allclose(f.points(world=False), pts)  # identity pose

    # A translated pose shifts every point.
    ft = capture.DepthFrame(**base, position=(1, 2, 3), quaternion=(0, 0, 0, 1))
    assert np.allclose(ft.points()[0], [-0.5, 3.5, 1.0])

    # 90° about +Z maps camera x to world y.
    s = np.sin(np.pi / 4)
    fr = capture.DepthFrame(**base, position=(0, 0, 0), quaternion=(0, 0, s, s))
    assert np.allclose(fr.points()[0], [-1.5, -1.5, -2.0], atol=1e-12)

    # Stride keeps rows/cols 0 and 2 -> 4 points, all valid here.
    assert f.points(stride=2).shape == (4, 3)


def test_capture_pointcloud(chromium, tmp_path) -> None:
    """§12.4 DEP3: an angled, off-axis camera's cloud lands on the
    authored world planes — depth semantics, K and the pose transform
    must all agree for this to come out — and the PLY round-trips."""
    np = pytest.importorskip("numpy")

    scene = bt.Scene()
    # Wall near face at x = 1.95; the floor plane sits at z ~ 0.
    scene.add_box("wall", size=(0.1, 4.0, 3.0), position=(2.0, 0.0, 1.5))
    scene.add_camera(
        "cam", position=(-0.4, 0.8, 1.6), look_at=(1.95, 0.0, 1.0),
        fov=60, resolution=(160, 120),
    )

    pts = capture.capture_pointcloud(scene, "cam", tmp_path / "cloud.ply")
    assert pts.dtype == np.float32 and pts.ndim == 2 and pts.shape[1] == 3
    assert len(pts) > 3000
    # Every returned point lies on the wall plane or the floor plane.
    off_plane = np.minimum(np.abs(pts[:, 0] - 1.95), np.abs(pts[:, 2]))
    assert float(off_plane.max()) <= 0.005
    # Both planes are actually represented.
    assert (np.abs(pts[:, 0] - 1.95) <= 0.005).sum() > 1000
    assert (np.abs(pts[:, 2]) <= 0.005).sum() > 100

    # PLY round-trip: our header, then the exact float32 triplets.
    raw = (tmp_path / "cloud.ply").read_bytes()
    end = raw.index(b"end_header\n") + len(b"end_header\n")
    header = raw[:end].decode("ascii")
    assert f"element vertex {len(pts)}" in header
    assert "format binary_little_endian 1.0" in header
    back = np.frombuffer(raw[end:], dtype="<f4").reshape(-1, 3)
    assert np.array_equal(back, pts)
