"""Regenerates the studio screenshots used by the documentation.

Headless: serves the studio for each scene and captures it with Playwright
Chromium on SwiftShader (no GPU needed). Uses the same Franka factory cell as
the examples, so the first run downloads the asset (~10 MB, cached).

    .venv/bin/python scripts/docs_screenshots.py

Writes docs/assets/studio/*.png — commit the results.
"""

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "examples"))

import botrail as bt  # noqa: E402
from demo import build_scene  # noqa: E402
import sfc_chart_demo  # noqa: E402
from playwright.sync_api import sync_playwright  # noqa: E402
from sequence_demo import BOX, TOUCH, build_cycle  # noqa: E402
import agv_cell_demo  # noqa: E402

OUT = ROOT / "docs" / "assets" / "studio"
CHROMIUM_ARGS = ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"]
VIEWPORT = {"width": 1600, "height": 1000}

# Camera override: three.js reports its renderer to `__THREE_DEVTOOLS__`; we
# wrap `render()` and re-aim the camera each frame, which wins over the studio's
# own orbit controls. Aim with `window.__CAM = {pos: [...], look: [...]}`.
CAMERA_HOOK = """
window.__THREE_DEVTOOLS__ = new EventTarget();
window.__THREE_DEVTOOLS__.addEventListener('observe', (e) => {
  const obj = e.detail;
  if (obj && obj.isWebGLRenderer && !obj.__docsWrapped) {
    obj.__docsWrapped = true;
    const orig = obj.render.bind(obj);
    obj.render = (scene, camera) => {
      if (window.__CAM) {
        camera.position.set(...window.__CAM.pos);
        camera.lookAt(...window.__CAM.look);
      }
      orig(scene, camera);
    };
  }
});
"""


def main() -> None:
    OUT.mkdir(parents=True, exist_ok=True)
    with sync_playwright() as p:
        browser = p.chromium.launch(args=CHROMIUM_ARGS)
        page = browser.new_page(viewport=VIEWPORT)
        # SwiftShader takes ~a minute per capture on a loaded machine; the
        # default 30 s screenshot timeout flakes.
        page.set_default_timeout(180_000)
        page.add_init_script(CAMERA_HOOK)

        # ---- 1. the demo cell as it opens: viewport, panels, TCP gizmo -----
        scene = build_scene()
        server = bt.studio(scene, block=False, open_browser=False)
        page.goto(server.url)
        page.wait_for_selector("canvas")
        time.sleep(3.0)  # meshes stream in
        page.screenshot(path=OUT / "overview.png")
        print("wrote overview.png")

        # ---- 2. a planned trajectory, broadcast to the connected studio ----
        page.locator(".tab", has_text="Motion").click()
        scene.plan_to_pose((0.45, -0.35, 0.75))
        time.sleep(1.5)
        page.screenshot(path=OUT / "plan.png")
        print("wrote plan.png")
        server.stop()

        # ---- 3. the sequence cell: baked timeline dock + mid-cycle pose ----
        scene = build_scene()
        name = build_cycle(scene)
        server = bt.studio(scene, block=False, open_browser=False)
        page.goto(server.url)
        page.wait_for_selector("canvas")
        page.locator(".tab", has_text="Sequence").click()
        time.sleep(2.0)
        tl = scene.simulate_sequence(name)  # broadcasts the baked timeline
        page.wait_for_selector(".timeline-bands", timeout=20000)
        # Freeze a telling moment live: the tracked grasp, pads on the box.
        t = tl.step_span("close").end
        scene.set_joint_positions(list(tl.sample(t)))
        scene.set_obstacle_pose(BOX, *tl.object_pose(BOX, t))
        scene.attach(BOX, link="/panda/panda_hand", touch_links=TOUCH)
        page.evaluate("window.__CAM = {pos: [1.5, -0.6, 1.35], look: [-0.1, 0.62, 0.62]}")
        time.sleep(1.5)
        page.screenshot(path=OUT / "sequence.png")
        print("wrote sequence.png")
        server.stop()

        # ---- 4. the SFC chart: the pick cell paused on its edge wait -----
        scene = sfc_chart_demo.build_cell()
        sfc_chart_demo.teach(scene)
        sfc_chart_demo.author_pick(scene)
        sfc_chart_demo.author_lamp(scene)
        server = bt.studio(scene, block=False, open_browser=False)
        page.goto(server.url)
        page.wait_for_selector("canvas")
        page.locator(".tab", has_text="Sequence").click()
        time.sleep(2.0)
        page.locator("button", has_text="SFC chart").click()
        scene.simulate_sequences(["pick", "lamp"], max_duration=120.0)
        page.wait_for_selector(".timeline-bands", timeout=30000)
        # Park the playhead where the chart tells its story: the arm posed
        # over the pick point, the belt feeding, the edge wait live —
        # ↑part_at_pick gray until the part arrives. Clicking the step is
        # the chart's own seek.
        page.locator(".sfc-box", has_text="await part").first.click()
        page.evaluate("window.__CAM = {pos: [1.9, -1.5, 1.75], look: [0.15, 0.25, 0.6]}")
        time.sleep(1.5)
        page.screenshot(path=OUT / "sfc.png")
        print("wrote sfc.png")
        server.stop()

        # ---- 5. the transport cell: guide path, stations, load sensor -----
        scene = agv_cell_demo.build_scene()
        name = agv_cell_demo.build_cycle(scene)
        server = bt.studio(scene, block=False, open_browser=False)
        page.goto(server.url)
        page.wait_for_selector("canvas")
        page.locator(".tab", has_text="Sequence").click()
        time.sleep(2.0)
        tl = scene.simulate_sequence(name, max_duration=90.0)
        page.wait_for_selector(".timeline-bands", timeout=20000)
        page.evaluate("window.__CAM = {pos: [2.2, -4.6, 3.9], look: [-0.1, -1.2, 0.2]}")
        # Let the playhead reach the handover: the arm over the deck, the
        # gate zone lit, all four lanes populated.
        time.sleep(20.5)
        page.screenshot(path=OUT / "vehicle.png")
        print("wrote vehicle.png")
        server.stop()

        browser.close()


if __name__ == "__main__":
    main()
