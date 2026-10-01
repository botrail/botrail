#!/usr/bin/env python3
"""Inspect the built Studio's real timeline seek/play/centerline display."""
import functools
import hashlib
import http.server
import json
import threading
from pathlib import Path

from playwright.sync_api import sync_playwright

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "target/rope-viewer"


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    payload = ROOT / "target/rope-replay.json"
    messages = json.loads(payload.read_text())
    track = messages[-1]["timeline"]["ropes"][0]
    handler = functools.partial(http.server.SimpleHTTPRequestHandler, directory=ROOT / "studio/dist")
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    checks = []
    try:
        with sync_playwright() as p:
            browser = p.chromium.launch(executable_path="/usr/bin/google-chrome", headless=True,
                args=["--no-sandbox", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"])
            page = browser.new_page(viewport={"width": 1440, "height": 1000})
            errors = []
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.goto(f"http://127.0.0.1:{server.server_port}")
            page.wait_for_function("!!window.__STUDIO__")
            page.evaluate("messages => {const s=window.__STUDIO__; for(const m of messages) s.getState().applyServerMessage(m); s.getState().setPlaying(false); s.getState().setConnection('connected');}", messages)
            page.wait_for_selector(".timeline-bands")
            for label, time in [("settle", 0.1), ("grasp", 0.3), ("carry", 1.0), ("release", 1.5), ("final", 2.0)]:
                box = page.locator(".timeline-bands").bounding_box()
                page.mouse.click(box["x"] + min(box["width"] - 0.05, box["width"] * time / 2.0), box["y"] + 8)
                # UI mouse coordinates have pixel granularity. Normal playback
                # reaches and samples the exact end rather than the last pixel.
                if label == "final":
                    page.evaluate("() => {const s=window.__STUDIO__.getState(); s.setPlaybackTime(1.98); s.setPlaying(true);}")
                    page.wait_for_function("window.__STUDIO__.getState().playbackTime === 2 && !window.__STUDIO__.getState().playing")
                page.wait_for_timeout(180)
                state = page.evaluate("() => { const s=window.__STUDIO__.getState(); return {t:s.playbackTime, p:Array.from(s.ropeSamples.cable), tool:s.overridePoses.carrier[2].position, count:s.playback.ropes[0].segments.length, radius:s.playback.ropes[0].radius_m}; }")
                t = state["t"]
                k = max(i for i, value in enumerate(track["times"]) if value <= t)
                j = min(k + 1, len(track["times"]) - 1)
                u = 0 if j == k else (t - track["times"][k]) / (track["times"][j] - track["times"][k])
                expected = [a + (b - a) * u for pa, pb in zip(track["points"][k], track["points"][j]) for a, b in zip(pa, pb)]
                assert len(state["p"]) == len(expected) == 99
                assert max(abs(a-b) for a,b in zip(expected,state["p"])) < 1e-6
                assert state["count"] == 32 and state["radius"] == 0.005
                page.screenshot(path=OUT / f"{label}.png")
                checks.append({"phase": label, **state})
            # Run the same imperative path used by normal playback, then pause.
            page.evaluate("() => {const s=window.__STUDIO__.getState(); s.setPlaybackTime(0.3); s.setPlaying(true);}")
            page.wait_for_timeout(450)
            page.evaluate("window.__STUDIO__.getState().setPlaying(false)")
            page.screenshot(path=OUT / "playing.png")
            assert not errors, errors
            report = {"passed": True, "input": "Rust example wire messages injected into the actual built Studio; no live server claim", "browser": browser.version, "payload_sha256": hashlib.sha256(payload.read_bytes()).hexdigest(), "checks": checks, "page_errors": errors}
            (OUT / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps({"passed": True, "phases": len(checks), "screenshots": 6, "browser": browser.version}))
            browser.close()
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2)


if __name__ == "__main__":
    main()
