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
it, a right-stick flick snap-turns 45° about the head, a right-stick push
throws the arc and letting go lands the head on its hit, the left stick
glides (up to 2 m/s the way the head faces, analog, sideways too, coasting
to rest when let go — and neither turning nor aiming), X recenters on the
launch stance, and (with ``--play``) A pauses / plays, B doubles the
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

In the basics cell (``demo``, ``sequence``) ``--grab`` then takes things
in hand (design-vr.md §3.12): a squeeze in mid-air takes nothing; a grip
on a crate takes the crate alone, which follows the hand, turns with it
and stays where it is let go; a block of a pallet brings the pallet and a
lamp the stack light (the part the bill knows them as); a selected piece
comes alone; none of it changes the selection; and (with ``--play``) a
part the recording has moved is taken from where it is drawn, which ends
the replay.

Where the cell has doors and drawers (scenery joints: the basics cell's
fence door; ``--cell home``, the SimReady kitchen's refrigerator and
dishwasher) ``--grab`` works them by hand: a grip on a door takes its
hinge, not the door — the hand's sweep about the axis opens it, a pull
past the stop holds at the stop until the hand is back there, it stays as
it is left, and a move from Python shows in the studio.
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
sys.path[:0] = [str(ROOT / "examples" / d) for d in ("basics", "welding", "palletizing", "home")]

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
    if cell == "home":
        import home_tidy_demo

        scene, _house = home_tidy_demo.build()
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
        launch = xr()
        press("right", "trigger")
        sel = xr()["selection"]
        expect("trigger selects", sel.get("type") == "tcp" and sel.get("robot") == robot["name"], sel)
        # The scene re-renders on a selection; the player's heading is not
        # a prop of it. Still facing what the launch stance faces.
        kept = xr()
        stance = kept["stance"]
        facing = [stance["lookAt"][i] - stance["standAt"][i] for i in range(2)]
        heading = kept["glide"]["heading"]
        cos = ((facing[0] * heading[0] + facing[1] * heading[1]) / math.hypot(*facing)) if heading else 0.0
        expect("a selection keeps the heading", abs(wrap_angle(kept["origin"]["yaw"] - launch["origin"]["yaw"])) < 1e-9
               and cos > 0.9999 and math.dist(kept["headWorld"], launch["headWorld"]) < 1e-9,
               {"yaw_deg": round(math.degrees(kept["origin"]["yaw"]), 2), "cos_to_target": round(cos, 6)})
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

    # 3. Teleport: right stick forward throws the arc from the right ray;
    # letting go lands the head over the hit.
    stick("right", 0, -1)
    arc = xr()["teleport"]
    expect("arc shown while the stick is held", arc["hand"] == "right" and arc["hit"] is not None and arc["valid"], arc)
    head_before = xr()["headWorld"]
    yaw_before = xr()["origin"]["yaw"]
    stick("right", 0, 0)
    now = xr()
    landed = math.dist(now["headWorld"][:2], arc["hit"][:2]) if arc["hit"] else None
    expect("teleport lands the head on the hit",
           landed is not None and landed < 0.03 and abs(now["headWorld"][2] - head_before[2]) < 1e-3
           and abs(wrap_angle(now["origin"]["yaw"] - yaw_before)) < 1e-6 and now["teleport"]["hand"] is None,
           {"landed_off_m": None if landed is None else round(landed, 4), "hit": arc["hit"], "head": now["headWorld"]})

    # 4. Glide: the left stick walks the player the way the head faces.
    # The turns above came back to the launch heading and a teleport keeps
    # it, so "the way the head faces" is still the stance's direction.
    def speed(r: dict) -> float:
        return math.hypot(*r["glide"]["velocity"])

    def cosine(a: list[float], b: list[float]) -> float:
        n = math.hypot(*a) * math.hypot(*b)
        return (a[0] * b[0] + a[1] * b[1]) / n if n > 1e-12 else 0.0

    start = xr()
    stance = start["stance"]
    facing = [stance["lookAt"][i] - stance["standAt"][i] for i in range(2)]
    right_of = [facing[1], -facing[0]]
    stick("left", 0, -1)
    frames(60)
    cruise = xr()
    walked = [cruise["headWorld"][i] - start["headWorld"][i] for i in range(2)]
    expect("left stick glides forward at full speed",
           abs(speed(cruise) - 2.0) < 0.05 and cosine(cruise["glide"]["velocity"], facing) > 0.999
           and cosine(walked, facing) > 0.999 and math.hypot(*walked) > 0.5,
           {"speed_mps": round(speed(cruise), 3), "walked_m": round(math.hypot(*walked), 3),
            "cos_to_facing": round(cosine(walked, facing), 5), "facing": facing, "heading": cruise["glide"]["heading"],
            "stance": stance, "yaw": start["origin"]["yaw"], "head_start": start["headWorld"]})
    expect("a glide neither turns, lifts nor aims",
           abs(wrap_angle(cruise["origin"]["yaw"] - start["origin"]["yaw"])) < 1e-9
           and abs(cruise["headWorld"][2] - start["headWorld"][2]) < 1e-3 and cruise["teleport"]["hand"] is None,
           {"yaw": cruise["origin"]["yaw"], "head_z": cruise["headWorld"][2], "teleport": cruise["teleport"]})
    stick("left", 0, -0.575)  # half of the live travel: a quarter of the speed
    frames(60)
    slow = xr()
    expect("the stick is analog", abs(speed(slow) - 0.5) < 0.03 and cosine(slow["glide"]["velocity"], facing) > 0.999,
           {"speed_mps": round(speed(slow), 3)})
    stick("left", 1, 0)
    frames(60)
    side = xr()
    expect("stick right strafes right", abs(speed(side) - 2.0) < 0.05
           and cosine(side["glide"]["velocity"], right_of) > 0.999,
           {"speed_mps": round(speed(side), 3), "cos_to_right": round(cosine(side["glide"]["velocity"], right_of), 5)})
    stick("left", 0, 0)
    frames(60)
    stopped = xr()
    frames(5)
    still = xr()
    drift = math.dist(still["headWorld"], stopped["headWorld"])
    expect("letting go coasts to rest", stopped["glide"]["velocity"] == [0, 0] and drift < 1e-9,
           {"velocity": stopped["glide"]["velocity"], "drift_m": drift})

    # 5. Recenter (left X): back over the launch stance.
    press("left", "x-button")
    now = xr()
    stance = now["stance"]
    off = math.dist(now["headWorld"][:2], stance["standAt"][:2]) if stance else None
    expect("X recenters on the stance", off is not None and off < 0.03, {"off_m": None if off is None else round(off, 4)})

    if play:
        # 6. Right A pauses and plays, right B doubles the speed.
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
        # 7. The panel: a trigger pull on its speed button from above.
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


def q_mul(a: list[float], b: list[float]) -> list[float]:
    """Hamilton product of two quaternions, x y z w."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return [aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz]


def q_rotate(q: list[float], v: list[float]) -> list[float]:
    x, y, z, w = q
    return q_mul(q_mul(q, [*v, 0.0]), [-x, -y, -z, w])[:3]


# The cells whose layout the hand check knows by name (the basics demo's).
HAND_CELLS = ("demo", "sequence")


def hand_xr(page, report: dict, errors: list[str], cell: str, scene, frames, xr, **_) -> None:
    """Things of the cell taken in hand (design-vr.md §3.12)."""
    checks: dict = {}
    report["hand_xr"] = checks

    def expect(name: str, ok: bool, detail) -> None:
        checks[name] = {"ok": ok, "detail": detail}
        print(f"[{cell}] {'ok ' if ok else 'FAIL'} {name}: {detail}", flush=True)
        if not ok:
            errors.append(f"hand xr {name}: {detail}")

    def poses(prefix: str = "") -> dict:
        return page.evaluate(
            """(p) => Object.fromEntries(window.__STUDIO__.getState().obstacles
                 .filter((o) => o.name.replace(/^[/]+/, '').startsWith(p)).map((o) => [o.name, o.pose]))""",
            prefix.lstrip("/"))

    def centre(name: str) -> list[float]:
        lo, hi = scene.obstacle_bounds(name)
        return [(a + b) / 2 for a, b in zip(lo, hi)]

    def place(hand: str, world: list[float], quaternion: tuple = (0, 0, 0, 1)) -> None:
        ref = page.evaluate("(w) => window.__XR__.toReference(w)", world)
        page.evaluate("""([h, p, q]) => { const c = window.__IWER__.controllers[h];
             c.position.set(...p); c.quaternion.set(...q); }""", [hand, ref, list(quaternion)])
        frames(3)

    def squeeze(hand: str, value: int) -> None:
        page.evaluate("([h, v]) => window.__IWER__.controllers[h].updateButtonValue('squeeze', v)", [hand, value])
        frames(3)

    def settle() -> None:
        # The hand's pose goes to the host and comes back as the scene's.
        frames(6)
        time.sleep(0.5)

    def add(a: list[float], b: list[float]) -> list[float]:
        return [x + y for x, y in zip(a, b)]

    def moved(before: dict, after: dict, by: list[float]) -> float:
        """The worst departure of a set of obstacles from `before + by`."""
        return max(math.dist(after[n]["position"], add(before[n]["position"], by)) for n in before)

    robot = page.evaluate("() => window.__STUDIO__.getState().robots[0]?.desc.name")
    selection = "() => { const s = window.__STUDIO__.getState().selection; return [s.type, s.robot ?? s.name ?? s.path]; }"

    # A part the recording has moved is taken from where it is drawn, and
    # the replay ends (first: the rest edits the scene the bake recorded).
    tracks = page.evaluate("() => window.__STUDIO__.getState().playback?.objects?.tracks.map((t) => t.name) ?? []")
    if tracks:
        page.evaluate("() => { const s = window.__STUDIO__.getState(); s.seek(0.6 * s.playback.duration); }")
        frames(3)
        shown = page.evaluate("""() => { const s = window.__STUDIO__.getState();
            return Object.fromEntries(Object.entries(s.overrideObstaclePoses ?? {}).filter(([n]) => !s.stowedObstacles.has(n))); }""")
        scene_poses = poses()
        far = {n: math.dist(p["position"], scene_poses[n]["position"]) for n, p in shown.items()
               if n in scene_poses and scene.obstacle_visible(n)}
        name = max(far, key=far.get) if far else None
        if name is None or far[name] < 0.05:
            expect("a recorded part away from its scene pose", False, far)
        else:
            # The part's centre as drawn: its scene centre carried to the recorded pose.
            was, now = scene_poses[name], shown[name]
            inv = [-was["quaternion"][0], -was["quaternion"][1], -was["quaternion"][2], was["quaternion"][3]]
            local = q_rotate(inv, [c - p for c, p in zip(centre(name), was["position"])])
            at = add(now["position"], q_rotate(now["quaternion"], local))
            place("left", at)
            squeeze("left", 1)
            state = page.evaluate("() => { const s = window.__STUDIO__.getState(); return [s.playing, s.overrideObstaclePoses === null]; }")
            expect("a recorded part is taken from where it is drawn", xr()["held"]["left"] == name,
                   {"held": xr()["held"], "part": name, "from_scene_m": round(far[name], 3)})
            expect("taking it ends the replay", state == [False, True], {"playing": state[0], "scene_shown": state[1]})
            settle()
            kept = math.dist(poses()[name]["position"], now["position"])
            expect("and stays in the hand", kept < 0.002, {"moved_m": round(kept, 4)})
            squeeze("left", 0)
    page.evaluate("(r) => { const s = window.__STUDIO__.getState(); s.stopPlayback(); s.selectTcp(r); }", robot)
    frames(3)

    # Mid-air: nothing.
    place("left", add(centre("/World/PalletFar/Crate_2"), [0, 0, 1.2]))
    before = poses()
    squeeze("left", 1)
    held = xr()["held"]
    settle()
    squeeze("left", 0)
    expect("a squeeze in mid-air takes nothing", held == {"left": None, "right": None} and poses() == before, held)

    # A workpiece: alone, following the hand, turning with it, staying put.
    crate = "/World/PalletFar/Crate_2"
    grip = add(centre(crate), [0.1, 0, 0])
    place("left", grip)
    squeeze("left", 1)
    expect("a grip on a crate takes the crate alone", xr()["held"] == {"left": crate, "right": None}, xr()["held"])
    before = poses()
    lift = [0.3, -0.25, 0.4]
    grip = add(grip, lift)
    place("left", grip)
    settle()
    after = poses()
    rest_of = {n: p for n, p in before.items() if n != crate}
    expect("it follows the hand", moved({crate: before[crate]}, after, lift) < 0.001
           and after[crate]["quaternion"] == before[crate]["quaternion"] and moved(rest_of, after, [0, 0, 0]) == 0,
           {"off_m": round(moved({crate: before[crate]}, after, lift), 5), "others_moved": moved(rest_of, after, [0, 0, 0])})
    # A quarter turn of the hand about the vertical (the reference space's Y).
    quarter = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    place("left", grip, (0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4)))
    settle()
    turned = poses()[crate]
    want = add(grip, q_rotate(quarter, [a - b for a, b in zip(after[crate]["position"], grip)]))
    facing = abs(sum(a * b for a, b in zip(turned["quaternion"], q_mul(quarter, after[crate]["quaternion"]))))
    expect("it turns with the hand", math.dist(turned["position"], want) < 0.001 and facing > 1 - 1e-6,
           {"off_m": round(math.dist(turned["position"], want), 5), "facing": round(facing, 7)})
    squeeze("left", 0)
    place("left", add(grip, [0.5, 0, 0]))
    settle()
    expect("it stays where it is let go", xr()["held"]["left"] is None and poses()[crate] == turned, poses()[crate]["position"])

    # A piece of a part brings the part: a block of the pallet, a lamp of the stack light.
    for hand, piece, unit, by in (("right", "/World/PalletFar/Blocks/Block_3", "/World/PalletFar", [0.2, 0.1, 0.05]),
                                  ("left", "stack_light/green", "/stack_light", [-0.2, 0.0, -0.3])):
        at = centre(piece)
        place(hand, at)
        before = poses(unit)
        squeeze(hand, 1)
        got = xr()["held"][hand]
        place(hand, add(at, by))
        settle()
        squeeze(hand, 0)
        off = moved(before, poses(unit), by)
        expect(f"a piece of {unit} brings it whole", got == unit and len(before) > 1 and off < 0.001,
               {"held": got, "members": len(before), "off_m": round(off, 5)})

    # Selected, the piece comes alone.
    block = "/World/PalletFar/Blocks/Block_3"
    page.evaluate("(n) => window.__STUDIO__.getState().selectObstacle(n)", block)
    frames(3)
    at = centre(block)
    place("left", at)
    before = poses("/World/PalletFar")
    squeeze("left", 1)
    got = xr()["held"]["left"]
    place("left", add(at, [0, 0, 0.3]))
    settle()
    squeeze("left", 0)
    after = poses("/World/PalletFar")
    others = {n: p for n, p in before.items() if n != block}
    expect("a selected piece comes alone", got == block and moved({block: before[block]}, after, [0, 0, 0.3]) < 0.001
           and moved(others, after, [0, 0, 0]) == 0, {"held": got})
    expect("a grab leaves the selection be", page.evaluate(selection) == ["obstacle", block], page.evaluate(selection))
    page.evaluate("(r) => window.__STUDIO__.getState().selectTcp(r)", robot)
    page.evaluate("""() => { for (const h of ['left', 'right']) { const c = window.__IWER__.controllers[h];
         c.quaternion.set(0, 0, 0, 1); } }""")
    frames(3)


def q_axis_angle(axis: list[float], angle: float) -> list[float]:
    s = math.sin(angle / 2)
    return [axis[0] * s, axis[1] * s, axis[2] * s, math.cos(angle / 2)]


# The doors and drawers the joint check works, by cell (scenery joints).
JOINT_CELLS = {
    "demo": ["fence/west/door"],
    "sequence": ["fence/west/door"],
    "home": ["kitchen/refrigerator/World/Joints/LeftDoor_joint",
             "kitchen/dishwasher/World/Joints/LowerRack_joint"],
}


def joints_xr(page, report: dict, errors: list[str], cell: str, scene, frames, xr, **_) -> None:
    """Doors and drawers worked by hand (design-scenery-joints.md)."""
    names = JOINT_CELLS.get(cell, [])
    if not names:
        return
    checks: dict = {}
    report["joints_xr"] = checks

    def expect(name: str, ok: bool, detail) -> None:
        checks[name] = {"ok": ok, "detail": detail}
        print(f"[{cell}] {'ok ' if ok else 'FAIL'} {name}: {detail}", flush=True)
        if not ok:
            errors.append(f"joints xr {name}: {detail}")

    def pose_controller(world: list[float]) -> None:
        ref = page.evaluate("(w) => window.__XR__.toReference(w)", world)
        page.evaluate("""([p]) => { const c = window.__IWER__.controllers.left;
             c.position.set(...p); c.quaternion.set(0, 0, 0, 1); }""", [ref])
        frames(3)

    # The grip sits off the pose the emulator sets; with the controller's
    # orientation held, by a fixed step in the world. A hinge reads the
    # hand's sweep about its axis, so the grip itself goes where asked.
    pose_controller([0.0, 0.0, 1.2])
    offset = [g - c for g, c in zip(xr()["grips"]["left"], [0.0, 0.0, 1.2])]
    checks["grip_offset_m"] = [round(v, 4) for v in offset]

    def place(world: list[float]) -> None:
        pose_controller([w - o for w, o in zip(world, offset)])

    def squeeze(value: int) -> None:
        page.evaluate("(v) => window.__IWER__.controllers.left.updateButtonValue('squeeze', v)", value)
        frames(3)

    def shown(name: str) -> float:
        return page.evaluate(
            "(n) => window.__STUDIO__.getState().sceneryJoints.find((j) => j.name === n)?.position ?? null", name)

    waits: list[float] = []
    checks["echo_s"] = waits

    def settle(name: str) -> None:
        """Until the host has the hand's last word and the studio has the
        host's answer (the send is throttled; the echo is a broadcast)."""
        frames(6)
        start, last, since = time.time(), None, time.time()
        while time.time() - start < 10:
            at = scene.scenery_joint(name)["position"]
            if at != last:
                last, since = at, time.time()
            elif shown(name) == at and time.time() - since > 0.3:
                break
            time.sleep(0.05)
        waits.append(round(time.time() - start, 2))

    def pose(name: str) -> dict:
        return page.evaluate(
            "(n) => window.__STUDIO__.getState().obstacles.find((o) => o.name === n)?.pose ?? null", name)

    known = page.evaluate("() => window.__STUDIO__.getState().sceneryJoints.map((j) => j.name)")
    expect("the studio knows the cell's joints", sorted(known) == sorted(scene.scenery_joints),
           {"studio": len(known), "scene": len(scene.scenery_joints)})
    page.evaluate("() => { const s = window.__STUDIO__.getState(); s.stopPlayback(); s.selectTcp(s.robots[0]?.desc.name); }")
    frames(3)
    for name in names:
        scene.set_scenery_joint(name, 0.0)
        time.sleep(0.5)
        joint = scene.scenery_joint(name)
        hinge = joint["kind"] == "hinge"
        origin, axis, (lower, upper) = joint["origin"], joint["axis"], joint["range"]
        label = name.rsplit("/", 1)[-1]

        def off_axis(point: list[float]) -> list[float]:
            r = [p - o for p, o in zip(point, origin)]
            along = sum(a * b for a, b in zip(r, axis))
            return [a - along * b for a, b in zip(r, axis)]

        def centre(obstacle: str) -> list[float]:
            lo, hi = scene.obstacle_bounds(obstacle)
            return [(a + b) / 2 for a, b in zip(lo, hi)]

        # Take it by the member furthest from the axis: the handle side.
        drawn = [o for o in joint["objects"] if scene.obstacle_visible(o)]
        member = max(drawn, key=lambda o: math.hypot(*off_axis(centre(o))))
        start = centre(member)
        before = pose(member)

        def hand_at(q: float) -> list[float]:
            """Where the hand is when it has asked the joint for `q`."""
            if not hinge:
                return [c + a * q for c, a in zip(start, axis)]
            r = [c - o for c, o in zip(start, origin)]
            return [o + v for o, v in zip(origin, q_rotate(q_axis_angle(axis, q), r))]

        def sweep(a: float, b: float) -> None:
            """The hand from `a` to `b`, in steps a hand would take."""
            step = math.radians(10) if hinge else 0.05
            n = max(1, math.ceil(abs(b - a) / step))
            for i in range(1, n + 1):
                place(hand_at(a + (b - a) * i / n))

        place(start)
        squeeze(1)
        held = xr()["held"]["left"]
        expect(f"{label}: a grip on it takes the joint", held == name, {"held": held, "by": member})
        # Open part of the way.
        part = (math.radians(40) if hinge else 0.2) * (1 if upper > 0 else -1)
        part = max(min(part, upper), lower)
        sweep(0.0, part)
        settle(name)
        at = scene.scenery_joint(name)["position"]
        if hinge:
            want = add(origin, q_rotate(q_axis_angle(axis, part), [p - o for p, o in zip(before["position"], origin)]))
        else:
            want = [p + a * part for p, a in zip(before["position"], axis)]
        off = math.dist(pose(member)["position"], want)
        expect(f"{label}: the hand's move opens it", abs(at - part) < 2e-3 and abs(shown(name) - at) < 1e-9 and off < 2e-3,
               {"asked": round(part, 4), "scene": round(at, 4), "member_off_m": round(off, 5)})
        # Past the stop, back a little (still past it), then back inside.
        stop = upper if part > 0 else lower
        way = 1 if part > 0 else -1
        over = math.radians(30) if hinge else 0.1
        sweep(part, stop + way * over)
        settle(name)
        at_stop = scene.scenery_joint(name)["position"]
        sweep(stop + way * over, stop + way * over / 2)
        settle(name)
        still = scene.scenery_joint(name)["position"]
        sweep(stop + way * over / 2, stop - way * over / 2)
        settle(name)
        back = scene.scenery_joint(name)["position"]
        expect(f"{label}: it holds at its stop until the hand is back there",
               abs(at_stop - stop) < 1e-9 and abs(still - stop) < 1e-9 and abs(back - (stop - way * over / 2)) < 2e-3,
               {"stop": round(stop, 4), "past": round(at_stop, 4), "still": round(still, 4), "back": round(back, 4)})
        squeeze(0)
        place(add(hand_at(stop - way * over / 2), [0.3, 0.3, 0.3]))
        settle(name)
        left_at = scene.scenery_joint(name)["position"]
        expect(f"{label}: it stays as it is left", xr()["held"]["left"] is None and abs(left_at - back) < 1e-9,
               {"position": round(left_at, 4)})
        # Shut from Python: the studio follows.
        scene.set_scenery_joint(name, 0.0)
        settle(name)
        shut = math.dist(pose(member)["position"], before["position"])
        expect(f"{label}: a move from Python shows in the studio", abs(shown(name)) < 1e-9 and shut < 1e-6,
               {"position": shown(name), "member_off_m": round(shut, 7)})


def add(a: list[float], b: list[float]) -> list[float]:
    return [x + y for x, y in zip(a, b)]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cell", nargs="+", default=["demo"],
                        choices=["demo", "sequence", "weld", "palletizing", "home"])
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
                             "panel's + point each add a waypoint; in the basics cell also things taken in hand")
    parser.add_argument("--tags", action="store_true",
                        help="switch the studio's 3D I/O tags on before the page loads (they must show in the headset)")
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
            if args.tags:
                # The 3D I/O tags on whatever the cell wires (or names, when
                # nothing is bound): boards drawn in the scene, so the
                # headset shows them too.
                page.add_init_script("localStorage.setItem('botrail-studio.ioTags', 'on')")

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
                            if cell in HAND_CELLS:
                                hand_xr(page, report, errors, cell, scene, **helpers)
                            joints_xr(page, report, errors, cell, scene, **helpers)
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
