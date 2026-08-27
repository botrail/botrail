"""CI smoke for botrail.capture: a short cycle, recorded headless.

Needs playwright with a fetched Chromium; skips (not errors) without them —
the base wheel must stay importable and testable with no browser at all.
"""

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
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
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


def test_record_camera_refusals(tmp_path) -> None:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
    with pytest.raises(ValueError, match="no camera named"):
        capture.record_camera(scene, "ghost", tmp_path / "x.webm")
    scene.add_camera("cam")
    with pytest.raises(ValueError, match="unsupported container"):
        capture.record_camera(scene, "cam", tmp_path / "x.avi")
