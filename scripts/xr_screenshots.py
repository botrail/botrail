"""Headless WebXR check of the studio through the IWER headset emulator.

    .venv/bin/python scripts/xr_screenshots.py                 # the basics demo cell
    .venv/bin/python scripts/xr_screenshots.py --cell weld --gpu
    .venv/bin/python scripts/xr_screenshots.py --cell demo sequence weld palletizing --gpu

Builds the cell, serves it with ``bt.studio(block=False)`` and opens it in
Playwright Chromium with Meta's Immersive Web Emulation Runtime (IWER, the
studio's own devDependency) installed *before any page script* — so the
shipped bundle is unchanged and ``navigator.xr`` answers for a Quest 3 with
its head 1.6 m above the floor. A render hook (three's ``__THREE_DEVTOOLS__``
observer, as ``docs_screenshots.py`` uses) records draw calls and triangles
of every scene render. Then: desktop numbers → the header's VR button → N
XR frames → ``window.__XR__.report()`` (head height in world Z, origin
placement, stance) → a screenshot of the canvas (both eyes) → a JSON report
next to it.

SwiftShader needs seconds per XR frame, so frame *times* mean nothing here;
draw calls and triangles do. ``--gpu`` renders on the NVIDIA GPU instead
(hundreds of times faster). Needs Playwright with Chromium installed.
Writes ``tmp/xr/<cell>.png`` and ``.json`` unless ``--out-dir`` says otherwise.

``--interact`` drives the emulated controllers through the V1 controls
(design-vr.md §3.4 / §3.6) and checks each against the store and the
report: the right ray on a robot hovers it (label), the trigger selects
it, a right-stick flick snap-turns 45° about the head, a left-stick push
throws the arc and letting go lands the head on its hit, X recenters on
the launch stance, and (with ``--play``) A pauses / plays, B doubles the
speed, and a trigger pull on the panel's speed button doubles it again.
The script only moves the emulator (``window.__IWER__``); the page's
``__XR__.aimPose`` turns a world target into a controller pose.

``--grab`` is V2 (design-vr.md §3.5): on the desktop the mouse path's
contract through ``__XR__.tcpGrab / tcpMove / tcpRelease`` (what a
TransformControls drag does to the handle: the arm follows the target,
an unreachable one turns the verdict red, letting go snaps the handle
back to the real TCP); in the session the right grip on the handle,
squeeze, a lift of 8 cm (the robot follows), a pull out of reach
(unreachable), release (snaps back), then left Y and the panel's
``+ point`` each add a waypoint to the motion the MOTION panel shows.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path[:0] = [str(ROOT / "examples" / d) for d in ("basics", "welding", "palletizing")]

import botrail as bt  # noqa: E402
from playwright.sync_api import sync_playwright  # noqa: E402

IWER_BUNDLE = ROOT / "studio" / "node_modules" / "iwer" / "build" / "iwer.min.js"
SWIFTSHADER_ARGS = ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader"]
GPU_ARGS = ["--use-gl=angle", "--use-angle=gl", "--ignore-gpu-blocklist"]
GPU_ENV = {"__NV_PRIME_RENDER_OFFLOAD": "1", "__GLX_VENDOR_LIBRARY_NAME": "nvidia"}

# The emulated headset, installed over headless Chromium's device-less
# navigator.xr; the studio's own XR store never emulates (emulate: false).
# The device object stays reachable for later phases (controller placement).
EMULATOR_HOOK = """
(() => {
  // IWER 2.8 lays a fixed, full-page container (z-index 999) over the
  // document for its session presentation; given our own, it can be made
  // transparent to the pointer so Playwright's click reaches the VR button.
  const container = document.createElement('div');
  const device = new IWER.XRDevice(IWER.metaQuest3, { canvasContainer: container });
  container.style.pointerEvents = 'none';
  device.installRuntime({ forceInstall: true });
  device.stereoEnabled = true;
  device.position.set(0, 1.6, 0);
  device.controllers.left?.position.set(-0.25, 1.25, -0.35);
  device.controllers.right?.position.set(0.25, 1.25, -0.35);
  window.__IWER__ = device;
})();
"""

# Draw calls / triangles of the last scene render. Reading renderer.info a
# frame later would see the OutputPass's single quad on the desktop path
# (every gl.render resets the counters), so the hook samples right after
# each render of a Scene. In XR one render covers both eyes. The counters
# are what the camera saw (after frustum culling, plus the shadow pass), so
# the hook also totals the visible scene itself — meshes, their draw calls
# and triangles — which does not depend on where anyone stands.
STATS_HOOK = """
window.__THREE_DEVTOOLS__ = new EventTarget();
window.__THREE_DEVTOOLS__.addEventListener('observe', (e) => {
  const r = e.detail;
  if (r && r.isWebGLRenderer && !r.__xrStats) {
    r.__xrStats = true;
    const orig = r.render.bind(r);
    const totals = (scene) => {
      let meshes = 0, calls = 0, triangles = 0;
      scene.traverseVisible((o) => {
        if (!(o.isMesh || o.isLine || o.isPoints || o.isSprite)) return;
        const g = o.geometry;
        const groups = Array.isArray(o.material) && g && g.groups.length ? g.groups.length : 1;
        calls += groups;
        if (!o.isMesh || !g) return;
        meshes++;
        const n = g.index ? g.index.count : (g.attributes.position ? g.attributes.position.count : 0);
        triangles += (n / 3) * (o.isInstancedMesh ? o.count : 1);
      });
      return { meshes, calls, triangles: Math.round(triangles) };
    };
    r.render = (scene, camera) => {
      orig(scene, camera);
      if (scene.isScene) {
        const prev = window.__RENDER_STATS__;
        const frames = (prev ? prev.frames : 0) + 1;
        window.__RENDER_STATS__ = {
          calls: r.info.render.calls,
          triangles: r.info.render.triangles,
          presenting: r.xr.isPresenting,
          frames,
          shadowMapType: r.shadowMap.type,
          scene: prev && frames % 30 ? prev.scene : totals(scene),
        };
      }
    };
  }
});
"""


def build(cell: str) -> tuple[bt.Scene, str | None]:
    """The cell, and the name of a sequence to bake for ``--play`` (if any)."""
    if cell == "demo":
        from demo import build_scene

        return build_scene(), None
    if cell == "sequence":
        from demo import build_scene
        from sequence_demo import build_cycle

        scene = build_scene()
        return scene, build_cycle(scene)
    if cell == "weld":
        import weld_line_demo

        scene, line, riders = weld_line_demo.build_line()
        poses = weld_line_demo.teach(scene, line, riders)
        for st in weld_line_demo.STATIONS:
            weld_line_demo.build_station_program(scene, st, poses, bodies=weld_line_demo.BODIES)
        weld_line_demo.build_transfer_program(scene, riders, gated=True)
        return scene, None
    if cell == "palletizing":
        import palletizing_line_demo

        scene, _info = palletizing_line_demo.build()
        return scene, None
    raise SystemExit(f"unknown cell {cell!r}")


# Where the emulator's controllers rest (reference space, Y-up, metres).
HAND_REST = {"left": [-0.25, 1.25, -0.35], "right": [0.25, 1.25, -0.35]}


def wrap_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def interact(page, report: dict, errors: list[str], cell: str, play: bool, run_controls: bool = True) -> dict:
    """The V1 controls, one after another, on the emulated controllers.
    Returns the controller helpers (also what ``--grab`` drives)."""
    checks: dict = {}
    if run_controls:
        report["interact"] = checks

    def xr():
        return page.evaluate("() => window.__XR__.report()")

    def store(expr: str):
        return page.evaluate(f"() => {{ const s = window.__STUDIO__.getState(); return {expr}; }}")

    def frames(n: int) -> None:
        start = xr()["xrFrames"]
        deadline = time.time() + 120
        while xr()["xrFrames"] < start + n:
            if time.time() > deadline:
                raise TimeoutError(f"{cell}: no XR frames while interacting")
            time.sleep(0.05)

    def aim(hand: str, target: list[float], frm: list[float] | None = None) -> None:
        pose = page.evaluate("([t, f]) => window.__XR__.aimPose(t, f)", [target, frm or HAND_REST[hand]])
        if pose is None:
            raise RuntimeError(f"{cell}: no origin to aim from")
        page.evaluate(
            """([h, p]) => { const c = window.__IWER__.controllers[h];
                 c.position.set(...p.position); c.quaternion.set(...p.quaternion); }""",
            [hand, pose],
        )
        frames(3)

    def rest(hand: str) -> None:
        page.evaluate(
            """([h, p]) => { const c = window.__IWER__.controllers[h];
                 c.position.set(...p); c.quaternion.set(0, 0, 0, 1); }""",
            [hand, HAND_REST[hand]],
        )
        frames(2)

    def press(hand: str, button: str) -> None:
        page.evaluate("([h, b]) => window.__IWER__.controllers[h].updateButtonValue(b, 1)", [hand, button])
        frames(3)
        page.evaluate("([h, b]) => window.__IWER__.controllers[h].updateButtonValue(b, 0)", [hand, button])
        frames(3)

    def stick(hand: str, x: float, y: float) -> None:
        page.evaluate("([h, x, y]) => window.__IWER__.controllers[h].updateAxes('thumbstick', x, y)", [hand, x, y])
        frames(3)

    def expect(name: str, ok: bool, detail) -> None:
        checks[name] = {"ok": ok, "detail": detail}
        print(f"[{cell}] {'ok ' if ok else 'FAIL'} {name}: {detail}", flush=True)
        if not ok:
            errors.append(f"interact {name}: {detail}")

    if not run_controls:
        return {"frames": frames, "press": press, "rest": rest, "xr": xr, "aim": aim}

    # 1. Hover and select a robot with the right ray + trigger. The first
    # robot is the default selection, so park the selection on an obstacle
    # first and watch the trigger bring it back.
    robot = store("s.robots[0] ? { name: s.robots[0].desc.name, base: s.robots[0].basePose?.position ?? [0, 0, 0] } : null")
    if robot:
        obstacle = store("s.obstacles[0]?.name ?? null")
        if obstacle:
            page.evaluate("(n) => window.__STUDIO__.getState().selectObstacle(n)", obstacle)
        target = [robot["base"][0], robot["base"][1], robot["base"][2] + 0.35]
        aim("right", target)
        frames(4)
        hover = xr()["hover"]
        expect("hover robot", bool(hover) and hover["kind"] == "robot" and hover["name"] == robot["name"], hover)
        label = xr()["label"]
        expect("hover label", robot["name"] in label.split("\n")[0], label.replace("\n", " | "))
        press("right", "trigger")
        sel = xr()["selection"]
        expect("trigger selects", sel.get("type") == "tcp" and sel.get("robot") == robot["name"], sel)
        rest("right")
        frames(3)
        after = xr()["hover"]
        expect("hover leaves the robot", after is None or after["name"] != robot["name"], after)

    # 2. Snap turn: right stick to the right, 45° clockwise about the head.
    before = xr()
    stick("right", 1, 0)
    stick("right", 0, 0)
    after = xr()
    turned = wrap_angle(after["origin"]["yaw"] - before["origin"]["yaw"])
    moved = math.dist(after["headWorld"][:2], before["headWorld"][:2])
    expect("snap turn 45° right", abs(turned + math.pi / 4) < 1e-3 and moved < 0.02,
           {"turned_deg": round(math.degrees(turned), 2), "head_moved_m": round(moved, 4),
            "headLocal": before["headLocal"], "eyes": before["eyes"], "headWorld": before["headWorld"]})
    stick("right", 1, 0)  # held past the threshold: no second turn until it re-arms
    again = wrap_angle(xr()["origin"]["yaw"] - after["origin"]["yaw"])
    expect("a held stick turns once", abs(again + math.pi / 4) < 1e-3, round(math.degrees(again), 2))
    stick("right", 0, 0)
    for _ in range(2):  # two flicks left undo the two right turns
        stick("right", -1, 0)
        stick("right", 0, 0)
    expect("snap turns back", abs(wrap_angle(xr()["origin"]["yaw"] - before["origin"]["yaw"])) < 1e-3,
           round(math.degrees(wrap_angle(xr()["origin"]["yaw"] - before["origin"]["yaw"])), 2))

    # 3. Teleport: left stick forward throws the arc from the left ray;
    # letting go lands the head over the hit.
    stick("left", 0, -1)
    arc = xr()["teleport"]
    expect("arc shown while the stick is held", arc["hand"] == "left" and arc["hit"] is not None and arc["valid"], arc)
    head_before = xr()["headWorld"]
    yaw_before = xr()["origin"]["yaw"]
    stick("left", 0, 0)
    now = xr()
    landed = math.dist(now["headWorld"][:2], arc["hit"][:2]) if arc["hit"] else None
    expect("teleport lands the head on the hit",
           landed is not None and landed < 0.03 and abs(now["headWorld"][2] - head_before[2]) < 1e-3
           and abs(wrap_angle(now["origin"]["yaw"] - yaw_before)) < 1e-6 and now["teleport"]["hand"] is None,
           {"landed_off_m": None if landed is None else round(landed, 4), "hit": arc["hit"], "head": now["headWorld"]})

    # 4. Recenter (left X): back over the launch stance.
    press("left", "x-button")
    now = xr()
    stance = now["stance"]
    off = math.dist(now["headWorld"][:2], stance["standAt"][:2]) if stance else None
    expect("X recenters on the stance", off is not None and off < 0.03, {"off_m": None if off is None else round(off, 4)})

    if play:
        # 5. Right A pauses and plays, right B doubles the speed.
        playing0 = store("s.playing")
        press("right", "a-button")
        playing1 = store("s.playing")
        press("right", "a-button")
        playing2 = store("s.playing")
        expect("A toggles play", playing1 == (not playing0) and playing2 == playing0, [playing0, playing1, playing2])
        speed0 = store("s.playbackSpeed")
        press("right", "b-button")
        speed1 = store("s.playbackSpeed")
        expect("B doubles the speed", speed1 == (1 if speed0 >= 8 else speed0 * 2), [speed0, speed1])
        # 6. The panel: a trigger pull on its speed button from above.
        panel = xr()["panel"]
        button = next((b for b in (panel or {}).get("buttons", []) if b["action"] == "speed"), None)
        if button and button["world"]:
            above = page.evaluate("(w) => window.__XR__.toReference(w)", button["world"])
            frm = [above[0] + 0.12, above[1] + 0.3, above[2] + 0.25]
            aim("right", button["world"], frm)
            press("right", "trigger")
            speed2 = store("s.playbackSpeed")
            expect("panel speed button", speed2 == (1 if speed1 >= 8 else speed1 * 2), [speed1, speed2])
            rest("right")
        else:
            expect("panel present", False, panel)
        page.evaluate("() => window.__STUDIO__.getState().setPlaybackSpeed(1)")
    return {"frames": frames, "press": press, "rest": rest, "xr": xr, "aim": aim}


def tcp_state(page) -> dict:
    """The focused robot's live TCP pose and solver verdict, from the store."""
    return page.evaluate("""() => {
      const s = window.__STUDIO__.getState();
      const name = s.selection.type === 'tcp' ? s.selection.robot : null;
      const r = s.robots.find((r) => r.desc.name === name) ?? null;
      if (!r || !r.tcpLink) return null;
      const i = r.desc.links.findIndex((l) => l.name === r.tcpLink);
      return { robot: r.desc.name, link: r.tcpLink, usd: !!r.desc.usd_asset,
               pose: r.linkPoses[i] ?? null, ik: r.ikStatus, q: r.jointPositions.slice() };
    }""")


def segment_count(page) -> int:
    return page.evaluate("() => window.__STUDIO__.getState().motions.reduce((n, m) => n + m.segments.length, 0)")


def hold_at(move, target: list[float], rounds: int = 6, pause: float = 0.25) -> None:
    """Hold a target the way a hand does — a hair of jitter around it — so
    the handle keeps emitting and the server keeps solving: its interactive
    preset settles a 7-DOF arm over a few messages, not one."""
    for i in range(rounds):
        wobble = 0.0005 if i % 2 else -0.0005
        move([target[0] + wobble, target[1], target[2]])
        time.sleep(pause)
    move(target)
    time.sleep(pause)


def grab_desktop(page, report: dict, errors: list[str], cell: str, scene) -> None:
    """The mouse path's contract: take the handle, move it, let go."""
    checks: dict = {}
    report["grab_desktop"] = checks

    def expect(name: str, ok: bool, detail) -> None:
        checks[name] = {"ok": ok, "detail": detail}
        print(f"[{cell}] {'ok ' if ok else 'FAIL'} desktop {name}: {detail}", flush=True)
        if not ok:
            errors.append(f"grab desktop {name}: {detail}")

    page.evaluate("() => { const s = window.__STUDIO__.getState(); s.stopPlayback(); s.selectTcp(s.robots[0]?.desc.name); }")
    time.sleep(0.5)
    live = tcp_state(page)
    tcp = page.evaluate("() => window.__XR__.report().tcp")
    if not live or not tcp:
        expect("handle present", False, {"live": live, "tcp": tcp})
        return
    gap = math.dist(tcp["world"], live["pose"]["position"])
    checks["home"] = {"robot": live["robot"], "q": live["q"], "tcp": live["pose"]["position"]}
    expect("handle sits on the TCP", gap < 0.01, {"gap_m": round(gap, 4), "usd": live["usd"], "link": live["link"],
                                                   "tcp": [round(v, 3) for v in live["pose"]["position"]]})
    expect("mouse smoothing off", tcp["smoothing"] == 0, tcp["smoothing"])
    assert page.evaluate("() => window.__XR__.tcpGrab()")
    move = lambda w: page.evaluate("(w) => window.__XR__.tcpMove(w)", w)  # noqa: E731
    target = [live["pose"]["position"][0], live["pose"]["position"][1], live["pose"]["position"][2] + 0.08]
    hold_at(move, target)
    after = tcp_state(page)
    err = math.dist(after["pose"]["position"], target)
    # The arm is the judge: the solver's `converged` is its own tolerance
    # (the Franka reports a 0.2 mm residual as not converged).
    expect("the arm follows an 8 cm lift (IK)", after["ik"] is not None and err < 0.02,
           {"ik": after["ik"], "off_m": round(err, 4)})
    far = [target[0] + 3.0, target[1], target[2]]
    hold_at(move, far, rounds=2)
    unreachable = tcp_state(page)
    status = page.evaluate("() => window.__XR__.report().tcp.status")
    expect("out of reach: unreachable, handle red", unreachable["ik"] is not None and not unreachable["ik"]["converged"]
           and status.get("reachable") is False, {"ik": unreachable["ik"], "status": status})
    # The failed solve left the arm stretched toward the far target — on a
    # 7-DOF arm against its limits a pose the solver does not come back
    # from in one go (the mouse path has always been like this). Put the
    # arm back from Python, as an operator would from the JOINTS panel,
    # then hold the handle where it started: a short, reachable solve.
    scene.set_joint_positions(live["q"], robot=live["robot"])
    time.sleep(0.5)
    hold_at(move, live["pose"]["position"])
    home = tcp_state(page)
    err = math.dist(home["pose"]["position"], live["pose"]["position"])
    expect("back within reach, the arm returns", err < 0.02, {"ik": home["ik"], "off_m": round(err, 4)})
    page.evaluate("() => window.__XR__.tcpRelease()")
    time.sleep(0.5)
    back = page.evaluate("() => window.__XR__.report().tcp")
    live2 = tcp_state(page)
    snap = math.dist(back["world"], live2["pose"]["position"])
    expect("release snaps the handle back to the TCP", not back["grabbed"] and snap < 0.01, {"gap_m": round(snap, 4)})


def grab_xr(page, report: dict, errors: list[str], cell: str, scene, frames, press, rest, xr, aim) -> None:
    """The right grip on the handle, then the taught point."""
    checks: dict = {}
    report["grab_xr"] = checks

    def expect(name: str, ok: bool, detail) -> None:
        checks[name] = {"ok": ok, "detail": detail}
        print(f"[{cell}] {'ok ' if ok else 'FAIL'} {name}: {detail}", flush=True)
        if not ok:
            errors.append(f"grab xr {name}: {detail}")

    def place_right(world: list[float]) -> None:
        ref = page.evaluate("(w) => window.__XR__.toReference(w)", world)
        page.evaluate("""([p]) => { const c = window.__IWER__.controllers.right;
             c.position.set(...p); c.quaternion.set(0, 0, 0, 1); }""", [ref])
        frames(3)

    def hold_right(world: list[float]) -> None:
        # The smoothed handle needs a few frames per step; the solver a few steps.
        for i in range(6):
            wobble = 0.0005 if i % 2 else -0.0005
            place_right([world[0] + wobble, world[1], world[2]])
            frames(8)
        place_right(world)
        frames(12)
        time.sleep(0.3)

    page.evaluate("() => { const s = window.__STUDIO__.getState(); s.stopPlayback(); s.selectTcp(s.robots[0]?.desc.name); }")
    frames(3)
    # Start from the configuration the desktop check started from: a bake
    # (`--play`) leaves the live arm wherever its cycle ended.
    home = report.get("grab_desktop", {}).get("home")
    if home:
        scene.set_joint_positions(home["q"], robot=home["robot"])
        time.sleep(0.5)
        frames(3)
    tcp = xr()["tcp"]
    live = tcp_state(page)
    if not tcp or not live:
        expect("handle present in the session", False, {"tcp": tcp, "live": live})
        return
    checks["start_tcp"] = [round(v, 3) for v in live["pose"]["position"]]
    expect("xr smoothing 0.2", abs(tcp["smoothing"] - 0.2) < 1e-9, tcp["smoothing"])
    # Squeeze away from the handle: nothing happens.
    rest("right")
    press("right", "squeeze")
    expect("a squeeze away from the handle grabs nothing", not xr()["tcp"]["grabbed"], xr()["tcp"]["world"])
    # On the handle: squeeze takes it.
    place_right(tcp["world"])
    page.evaluate("() => window.__IWER__.controllers.right.updateButtonValue('squeeze', 1)")
    frames(3)
    held = xr()["tcp"]
    expect("squeeze on the handle takes it", held["grabbed"], held)
    # Lift 8 cm: the handle follows the hand (smoothed), the arm follows the handle.
    start = tcp_state(page)["pose"]["position"]
    lifted = [start[0], start[1], start[2] + 0.08]
    hold_right(lifted)
    after = tcp_state(page)
    err = math.dist(after["pose"]["position"], lifted)
    expect("the arm follows the hand (IK)", after["ik"] is not None and err < 0.02,
           {"ik": after["ik"], "off_m": round(err, 4), "label": xr()["label"].split("\n")[-1]})
    # Out of reach: the verdict turns.
    place_right([lifted[0] + 3.0, lifted[1], lifted[2]])
    frames(30)
    time.sleep(0.5)
    now = xr()
    expect("out of reach reads unreachable", now["tcp"]["status"].get("reachable") is False and "unreachable" in now["label"],
           {"status": now["tcp"]["status"], "label": now["label"].split("\n")[-1]})
    # Let go: the handle drops back on the real TCP, wherever the arm got to.
    page.evaluate("() => window.__IWER__.controllers.right.updateButtonValue('squeeze', 0)")
    frames(5)
    released = xr()["tcp"]
    live2 = tcp_state(page)
    snap = math.dist(released["world"], live2["pose"]["position"])
    expect("release snaps the handle back", not released["grabbed"] and snap < 0.01, {"gap_m": round(snap, 4)})
    # The arm back home from Python (see grab_desktop), the handle following
    # it; then a second grab lifts it again — a short, reachable solve. (A
    # smoothed handle dragged back from 3 m away would sweep the arm
    # through the same far targets on the way.)
    scene.set_joint_positions(live["q"], robot=live["robot"])
    time.sleep(0.5)
    frames(3)
    tcp2 = xr()["tcp"]
    followed = math.dist(tcp2["world"], live["pose"]["position"])
    expect("the handle follows the arm home", followed < 0.01, {"gap_m": round(followed, 4)})
    place_right(tcp2["world"])
    page.evaluate("() => window.__IWER__.controllers.right.updateButtonValue('squeeze', 1)")
    frames(3)
    hold_right(lifted)
    home = tcp_state(page)
    err = math.dist(home["pose"]["position"], lifted)
    expect("a second grab lifts it again", xr()["tcp"]["grabbed"] and err < 0.02, {"ik": home["ik"], "off_m": round(err, 4)})
    page.evaluate("() => window.__IWER__.controllers.right.updateButtonValue('squeeze', 0)")
    frames(5)
    rest("right")
    # The taught point: left Y, then the panel's button.
    n0 = segment_count(page)
    press("left", "y-button")
    time.sleep(0.5)
    n1 = segment_count(page)
    expect("left Y adds a waypoint", n1 == n0 + 1, {"before": n0, "after": n1,
           "motions": page.evaluate("() => window.__STUDIO__.getState().motions.map((m) => [m.name, m.segments.length])")})
    button = next((b for b in (xr()["panel"] or {}).get("buttons", []) if b["action"] == "waypoint"), None)
    if button and button["world"]:
        above = page.evaluate("(w) => window.__XR__.toReference(w)", button["world"])
        aim("right", button["world"], [above[0] + 0.12, above[1] + 0.3, above[2] + 0.25])
        press("right", "trigger")
        time.sleep(0.5)
        n2 = segment_count(page)
        expect("the panel's + point adds a waypoint", n2 == n1 + 1, {"before": n1, "after": n2})
        rest("right")
    else:
        expect("panel waypoint button", False, button)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cell", nargs="+", default=["demo"], choices=["demo", "sequence", "weld", "palletizing"])
    parser.add_argument("--out-dir", default=str(ROOT / "tmp" / "xr"))
    parser.add_argument("--frames", type=int, default=4, help="XR frames to render before reporting")
    parser.add_argument("--settle", type=float, default=5.0,
                        help="seconds the draw-call count must hold still before measuring (meshes stream in)")
    parser.add_argument("--size", default="1600x1000")
    parser.add_argument("--gpu", action="store_true", help="render on the NVIDIA GPU instead of SwiftShader")
    parser.add_argument("--no-xr", action="store_true", help="desktop numbers only: never enter the session")
    parser.add_argument("--play", action="store_true",
                        help="bake the cell's sequence first and check that playback keeps advancing inside the session")
    parser.add_argument("--interact", action="store_true",
                        help="drive the emulated controllers through the V1 controls (select, turn, teleport, "
                             "recenter; with --play also the A/B buttons and the panel) and check each")
    parser.add_argument("--grab", action="store_true",
                        help="V2: the TCP handle on the desktop (grab / move / release through __XR__) and in the "
                             "session (right grip squeeze, IK follows, unreachable, release), then left Y and the "
                             "panel's + point each add a waypoint")
    parser.add_argument("--no-emulator", action="store_true",
                        help="do not install IWER: checks that the desktop path is untouched "
                             "(no XR support, no VR button, no canvas antialias)")
    parser.add_argument("--timeout", type=float, default=600, help="seconds to wait for the cell and the XR frames")
    args = parser.parse_args()
    width, height = (int(v) for v in args.size.split("x"))
    if not IWER_BUNDLE.exists():
        raise SystemExit(f"{IWER_BUNDLE} missing — run `pnpm install` in studio/")
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.gpu:
        os.environ.update(GPU_ENV)

    failures = 0
    with sync_playwright() as p:
        browser = p.chromium.launch(args=GPU_ARGS if args.gpu else SWIFTSHADER_ARGS)
        for cell in args.cell:
            errors: list[str] = []
            logs: list[str] = []
            page = browser.new_page(viewport={"width": width, "height": height})
            page.set_default_timeout(args.timeout * 1000)
            page.on("pageerror", lambda e: errors.append(str(e)))
            page.on("console", lambda m: logs.append(m.text)
                    if m.type in ("warning", "error") and "GL Driver" not in m.text else None)
            http_errors: list[str] = []
            page.on("response", lambda r: http_errors.append(f"{r.status} {r.url}") if r.status >= 400 else None)
            if not args.no_emulator:
                page.add_init_script(IWER_BUNDLE.read_text() + "\n" + EMULATOR_HOOK)
            page.add_init_script(STATS_HOOK)

            def wait_for(predicate_js: str, what: str, timeout: float = args.timeout) -> None:
                deadline = time.time() + timeout
                while time.time() < deadline:
                    if page.evaluate(predicate_js):
                        return
                    time.sleep(0.25)
                raise TimeoutError(f"{cell}: timed out waiting for {what}")

            def wait_settled() -> dict:
                # Meshes stream in over the websocket and the mesh routes for
                # a while after the connection; the picture is complete once
                # the per-frame draw-call and triangle counts stop moving.
                deadline = time.time() + args.timeout
                last, still = None, 0.0
                while time.time() < deadline:
                    stats = page.evaluate("() => window.__RENDER_STATS__")
                    key = (stats["calls"], stats["triangles"]) if stats else None
                    if key is not None and key == last:
                        still += 1.0
                        if still >= args.settle:
                            return stats
                    else:
                        last, still = key, 0.0
                    time.sleep(1.0)
                raise TimeoutError(f"{cell}: the draw-call count never settled")

            print(f"[{cell}] building…", flush=True)
            scene, cycle = build(cell)
            server = bt.studio(scene, block=False, open_browser=False)
            report: dict = {"cell": cell}
            try:
                page.goto(server.url)
                page.wait_for_selector("canvas")
                wait_for("() => window.__STUDIO__?.getState().connection === 'connected'", "the connection")
                if not args.no_emulator:
                    wait_for("() => window.__STUDIO__?.getState().xr.supported === true", "the XR probe (IWER)")
                    wait_for("() => document.querySelector('button.vr-button') !== null", "the VR button")
                desktop = wait_settled()
                report["robots"] = page.evaluate("() => window.__STUDIO__.getState().robots.length")
                report["desktop"] = desktop
                if args.grab:
                    grab_desktop(page, report, errors, cell, scene)
                if args.play:
                    if not cycle:
                        raise SystemExit(f"{cell}: no sequence to play (use --cell sequence)")
                    tl = scene.simulate_sequence(cycle)  # broadcasts the baked timeline
                    wait_for("() => window.__STUDIO__.getState().playback !== null", "the baked timeline")
                    page.evaluate("() => window.__STUDIO__.getState().setPlaying(true)")
                    report["baked_s"] = tl.duration
                sc = desktop["scene"]
                print(f"[{cell}] scene: {sc['meshes']} meshes, {sc['calls']} draw calls, {sc['triangles'] / 1000:.0f}k tris", flush=True)
                print(f"[{cell}] desktop view: {desktop['calls']} draw calls, {desktop['triangles'] / 1000:.0f}k tris", flush=True)
                if args.no_emulator:
                    # The path every other browser takes: the probe says no,
                    # the header shows no VR button, the canvas has no
                    # antialias of its own (the OutputPass target does it).
                    path = page.evaluate("""() => ({
                      xrSupported: window.__STUDIO__.getState().xr.supported,
                      vrButton: document.querySelector('button.vr-button') !== null,
                      antialias: document.querySelector('canvas').getContext('webgl2').getContextAttributes().antialias,
                    })""")
                    report["desktop_path"] = path
                    ok = not path["xrSupported"] and not path["vrButton"] and not path["antialias"]
                    print(f"[{cell}] desktop path untouched: {ok} {path}", flush=True)
                    if not ok:
                        errors.append(f"desktop path changed: {path}")
                elif not args.no_xr:
                    page.click("button.vr-button")
                    wait_for(f"() => (window.__XR__?.report().xrFrames ?? 0) >= {args.frames}", f"{args.frames} XR frames")
                    stats = wait_settled()
                    xr = page.evaluate("() => window.__XR__.report()")
                    report["xr"] = {**xr, "stats": stats, "store": page.evaluate("() => window.__STUDIO__.getState().xr")}
                    if args.play:
                        # The playback driver runs in the XR frame loop too: the
                        # playhead must keep moving while presenting.
                        clock = "() => { const s = window.__STUDIO__.getState(); return [s.playbackTime, s.playing]; }"
                        t0, playing0 = page.evaluate(clock)
                        time.sleep(2.0)
                        t1, playing1 = page.evaluate(clock)
                        report["xr"]["playback"] = {"t0": t0, "t1": t1, "playing": playing0 and playing1}
                        print(f"[{cell}] playback in XR: {t0:.2f} s → {t1:.2f} s (playing = {playing0 and playing1})", flush=True)
                        if not (t1 > t0 and playing1):
                            errors.append(f"playback did not advance in XR: {t0} → {t1}")
                    print(f"[{cell}] xr view: {stats['calls']} draw calls (both eyes), {stats['triangles'] / 1000:.0f}k tris, "
                          f"head z = {xr['headHeight']:.3f} m, antialias = {xr['antialias']}, "
                          f"presenting = {stats['presenting']}", flush=True)
                    if args.interact or args.grab:
                        helpers = interact(page, report, errors, cell, args.play, run_controls=args.interact)
                        if args.grab:
                            grab_xr(page, report, errors, cell, scene, **helpers)
                    png = out_dir / f"{cell}.png"
                    page.locator("canvas").screenshot(path=png)
                    print(f"[{cell}] wrote {png}", flush=True)
                    # Leave the session: the desktop pass returns.
                    page.click("button.vr-button")
                    wait_for("() => window.__STUDIO__.getState().xr.presenting === false", "the session to end", 60)
                    time.sleep(1.0)
                    report["after_exit"] = page.evaluate("() => window.__RENDER_STATS__")
            finally:
                server.stop()
                report["errors"] = errors
                report["console"] = [line[:200] for line in logs[:12]]
                report["http_errors"] = http_errors[:12]
                (out_dir / f"{cell}.json").write_text(json.dumps(report, indent=1))
                page.close()
            if errors:
                failures += 1
                print(f"[{cell}] page errors: {errors[:3]}", flush=True)
        browser.close()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
