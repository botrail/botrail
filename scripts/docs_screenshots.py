"""Regenerates the studio screenshots used by the documentation.

Headless: serves the studio for each scene and captures it with Playwright
Chromium on SwiftShader (no GPU needed). Uses the same Franka factory cell as
the examples, so the first run downloads the asset (~10 MB, cached).

    .venv/bin/python scripts/docs_screenshots.py            # everything
    .venv/bin/python scripts/docs_screenshots.py io sfc     # named captures only

Writes docs/assets/studio/*.png — commit the results.
"""

import re
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "examples" / d)
                for d in ("basics", "export", "legged", "machining", "multi_robot", "vehicles", "welding")]

import botrail as bt  # noqa: E402
from demo import build_scene  # noqa: E402
import export_urscript  # noqa: E402
import sfc_chart_demo  # noqa: E402
import weld_line_demo  # noqa: E402
from playwright.sync_api import sync_playwright  # noqa: E402
from sequence_demo import BOX, TOUCH, build_cycle  # noqa: E402
import agv_cell_demo  # noqa: E402
import dual_arm_demo  # noqa: E402
import legged_patrol_demo  # noqa: E402
import machine_tending_demo  # noqa: E402

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
    only = set(sys.argv[1:])

    def want(name: str) -> bool:
        return not only or name in only

    with sync_playwright() as p:
        browser = p.chromium.launch(args=CHROMIUM_ARGS)
        page = browser.new_page(viewport=VIEWPORT)
        # SwiftShader takes ~a minute per capture on a loaded machine; the
        # default 30 s screenshot timeout flakes.
        page.set_default_timeout(180_000)
        page.add_init_script(CAMERA_HOOK)

        # ---- 1. the demo cell as it opens: viewport, panels, TCP gizmo -----
        if want("overview") or want("plan"):
            scene = build_scene()
            server = bt.studio(scene, block=False, open_browser=False)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            time.sleep(3.0)  # meshes stream in
            if want("overview"):
                page.screenshot(path=OUT / "overview.png")
                print("wrote overview.png")

            # ---- 2. a planned trajectory, broadcast to the connected studio
            if want("plan"):
                page.locator(".tab", has_text="Motion").click()
                scene.plan_to_pose((0.45, -0.35, 0.75))
                time.sleep(1.5)
                page.screenshot(path=OUT / "plan.png")
                print("wrote plan.png")
            server.stop()

        if want("sequence"):
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
            # From inside the cell, south-east of the pick and clear of the
            # rack — its top deck stands at 1.35, exactly where this camera
            # used to sit.
            page.evaluate("window.__CAM = {pos: [1.25, -0.95, 1.30], look: [0.0, 0.62, 0.62]}")
            time.sleep(1.5)
            page.screenshot(path=OUT / "sequence.png")
            print("wrote sequence.png")
            server.stop()

        if want("sfc"):
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

        if want("ld"):
            # ---- 4b. the ladder view: the same cell, parked on the routing
            # decision — the latched `reject` verdict conducting through an
            # NC contact is the story told in ladder vocabulary.
            scene = sfc_chart_demo.build_cell()
            sfc_chart_demo.teach(scene)
            sfc_chart_demo.author_pick(scene)
            sfc_chart_demo.author_lamp(scene)
            server = bt.studio(scene, block=False, open_browser=False)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            page.locator(".tab", has_text="Sequence").click()
            time.sleep(2.0)
            page.locator("button", has_text="Ladder").click()
            scene.simulate_sequences(["pick", "lamp"], max_duration=120.0)
            page.wait_for_selector(".timeline-bands", timeout=30000)
            # Seek to the good part's seat: the ◇ judge rung has just fired
            # (its coils flash), the token rides `S12 · to tray`.
            page.locator(".ld-comment", has_text="to tray").first.click()
            page.evaluate("window.__CAM = {pos: [1.9, -1.5, 1.75], look: [0.15, 0.25, 0.6]}")
            time.sleep(1.5)
            page.screenshot(path=OUT / "ld.png")
            print("wrote ld.png")
            server.stop()

        if want("vehicle"):
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
            # gate zone lit, the signal lanes populated.
            time.sleep(20.5)
            page.screenshot(path=OUT / "vehicle.png")
            print("wrote vehicle.png")
            server.stop()

        if want("legged"):
            # ---- 5b. the legged cell: a quadruped carrying a part out ----
            # The Go2 is fetched from Unitree's repository on first run
            # (~20 MB, cached); `--robot quad` would do without, but the
            # picture is the real machine.
            scene = legged_patrol_demo.build_scene("go2")
            names = legged_patrol_demo.build_cycle(scene)
            server = bt.studio(scene, block=False, open_browser=False)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            page.locator(".tab", has_text="Sequence").click()
            time.sleep(2.0)
            scene.simulate_sequences(names, max_duration=90.0)
            page.wait_for_selector(".timeline-bands", timeout=60000)
            page.evaluate("window.__CAM = {pos: [4.6, -2.4, 1.7], look: [2.6, 0.4, 0.35]}")
            # Mid-walk to the bay with the part on its back, ~21 s in: wait
            # on the studio's own clock rather than ours.
            deadline = time.time() + 120
            while time.time() < deadline:
                text = page.locator(".timeline-bands").locator("xpath=..").inner_text()
                times = re.findall(r"(\d+\.\d+)s", text)
                if times and float(times[-1]) >= 21.0:
                    break
                time.sleep(0.1)
            page.screenshot(path=OUT / "legged.png")
            print("wrote legged.png")
            server.stop()

        # ---- 6. the I/O map: the wired pick cell, table over the viewport,
        # channel chips on the lanes, a fault scenario's diagnosis on the dock
        if want("io"):
            scene = export_urscript.build_cell()
            export_urscript.author_sequence(scene)
            export_urscript.wire_cell(scene)
            scene.add_scenario("beam_stuck", faults=[bt.io.stuck("part_at_pick", False)])
            server = bt.studio(scene, block=False, open_browser=False)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            page.locator(".tab", has_text="Sequence").click()
            time.sleep(2.0)
            page.locator("button", has_text="I/O").click()
            scene.simulate_sequences(["pick"], max_duration=30.0)
            page.wait_for_selector(".timeline-bands", timeout=30000)
            # The stall under the stuck beam: pick the world, simulate — the
            # last good bake stays and the dock names the forced point.
            page.locator("select").last.select_option("beam_stuck")
            page.locator("button.plan-go").click()
            page.wait_for_selector(".timeline-diagnosis", timeout=60000)
            page.evaluate("window.__CAM = {pos: [1.9, -1.5, 1.75], look: [0.15, 0.25, 0.6]}")
            time.sleep(1.5)
            page.screenshot(path=OUT / "io.png")
            print("wrote io.png")
            server.stop()

        # ---- 7. the topology: the weld line in its three placements —
        # nothing declared, a PLC master with declared cabinets, and the
        # stations running their own programs (design-electrical.md demo 2)
        if want("topology"):
            scene, line, riders = weld_line_demo.build_line()
            poses = weld_line_demo.teach(scene, line, riders)
            for st in weld_line_demo.STATIONS:
                weld_line_demo.build_station_program(scene, st, poses, bodies=weld_line_demo.BODIES)
            weld_line_demo.build_transfer_program(scene, riders, gated=True)
            server = bt.studio(scene, block=False, open_browser=False)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            page.locator(".tab", has_text="Sequence").click()
            time.sleep(3.0)
            page.locator("button", has_text="Topology").click()
            time.sleep(1.5)
            page.screenshot(path=OUT / "topology_stage1.png", clip={"x": 0, "y": 44, "width": 620, "height": 420})
            print("wrote topology_stage1.png")
            plc = bt.io.di16(base="%IX0.0") + bt.io.do16(base="%QX0.0")
            scene.add_io_node("PLC1", kind="plc", programs=["transfer", "st1", "st2"], channels=plc)
            scene.add_io_node("RIO1", kind="remote_io", uplink=("PLC1", "PROFINET"),
                              channels=bt.io.di8(base="%IX1.0"), model="ET200SP")
            scene.add_io_node("ST1", kind="robot_controller", robots=["st1_lh", "st1_rh"],
                              channels=bt.io.ur_standard())
            scene.add_io_node("ST2", kind="robot_controller", robots=["st2_lh", "st2_rh"],
                              channels=bt.io.ur_standard())
            scene.bind_input("body_at_head", "RIO1", "DI2", tag="BodyAtHead", field="-B1")
            scene.auto_assign_io()
            time.sleep(1.5)
            page.screenshot(path=OUT / "topology_stage2.png", clip={"x": 0, "y": 44, "width": 620, "height": 620})
            print("wrote topology_stage2.png")
            scene.add_io_node("PLC1", kind="plc", programs=["transfer"], channels=plc)
            scene.add_io_node("ST1", kind="robot_controller", robots=["st1_lh", "st1_rh"], programs=["st1"],
                              channels=bt.io.ur_standard())
            scene.add_io_node("ST2", kind="robot_controller", robots=["st2_lh", "st2_rh"], programs=["st2"],
                              channels=bt.io.ur_standard())
            scene.auto_assign_io(reassign=True)   # drop the placement's automatic channels, renumber
            scene.simulate_sequences(["st1", "st2", "transfer"], max_duration=400.0)
            page.wait_for_selector(".timeline-bands", timeout=60000)
            page.evaluate("window.__CAM = {pos: [6.5, -9.0, 4.5], look: [4.0, 0.0, 0.8]}")
            time.sleep(6.0)   # into the first index: moving high, wires lit
            page.screenshot(path=OUT / "topology.png")
            print("wrote topology.png")
            server.stop()

        if want("machine_tending"):
            # ---- the tending cell: the arm through the side door, mid-unload,
            # the door's and the buttons' lanes below ---------------------------
            scene, hs, tl = machine_tending_demo.bake()
            server = bt.studio(scene, block=False, open_browser=False)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            page.locator(".tab", has_text="Sequence").click()
            time.sleep(3.0)
            scene.simulate_sequences(["tend", hs.program], max_duration=240.0)
            page.wait_for_selector(".timeline-bands", timeout=30000)
            # The grasp on the finished part, the door slid to its open end.
            t = tl.step_span("tend/grip").end
            scene.set_joint_positions(list(tl.sample(t, robot="arm")), robot="arm")
            for name in scene.obstacle_names:
                if name.startswith("vmc/side_door/"):
                    try:
                        scene.set_obstacle_pose(name, *tl.object_pose(name, t))
                    except ValueError:
                        pass  # the door's rails stay put; only the leaf and its trim ride the axis
            page.evaluate("window.__CAM = {pos: [3.6, -3.2, 2.6], look: [0.6, -0.4, 1.0]}")
            time.sleep(2.0)
            page.screenshot(path=OUT / "machine_tending.png")
            print("wrote machine_tending.png")
            server.stop()

        if want("machine_tending_hand"):
            # ---- the multi-purpose hand: the fork on the handle, the door
            # half open, the gripper and the pin hanging clear -------------
            scene, hs, tl = machine_tending_demo.bake()
            server = bt.studio(scene, block=False, open_browser=False)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            time.sleep(3.0)
            # The baked cycle arrives with the connection and plays; a pose
            # set live would be overwritten by the next frame, so the studio
            # is paused and its own seek bar put mid-slide — arm and leaf
            # sampled together from the tracks.
            page.wait_for_selector(".timeline-bands", timeout=30000)
            page.evaluate("window.__STUDIO__.getState().setPlaying(false)")
            span = tl.step_span("tend/slide_open")
            t = (span.start + span.end) / 2
            bands = page.locator(".timeline-bands").bounding_box()
            page.mouse.click(bands["x"] + bands["width"] * (t / tl.duration), bands["y"] + bands["height"] / 2)
            page.evaluate("window.__CAM = {pos: [2.4, -1.9, 1.9], look: [0.9, -0.3, 1.1]}")
            time.sleep(2.0)
            page.screenshot(path=OUT / "machine_tending_hand.png")
            print("wrote machine_tending_hand.png")
            server.stop()

        if want("dual_arm"):
            # ---- one robot, two arms: the tray carried with both hands, the
            # timeline dock with a lane per arm ------------------------------
            scene, _rig, programs = dual_arm_demo.build("ur5e", carry=True)
            tl = dual_arm_demo.simulate(scene, programs)
            server = bt.studio(scene, block=False, open_browser=False)
            page.goto(server.url)
            page.wait_for_selector("canvas")
            page.locator(".tab", has_text="Sequence").click()
            time.sleep(3.0)
            page.wait_for_selector(".timeline-bands", timeout=30000)
            page.evaluate("window.__STUDIO__.getState().setPlaying(false)")
            span = tl.step_span("left/left carry")
            t = (span.start + span.end) / 2
            bands = page.locator(".timeline-bands").bounding_box()
            page.mouse.click(bands["x"] + bands["width"] * (t / tl.duration), bands["y"] + bands["height"] / 2)
            page.evaluate("window.__CAM = {pos: [1.7, -1.3, 1.55], look: [0.35, 0.0, 0.85]}")
            time.sleep(2.0)
            page.screenshot(path=OUT / "dual_arm.png")
            print("wrote dual_arm.png")
            server.stop()

        browser.close()


if __name__ == "__main__":
    main()
