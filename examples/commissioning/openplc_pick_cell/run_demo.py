"""The pick cell as the plant of a real IEC 61131-3 runtime — OpenPLC v3 in
Docker (design-virtual-commissioning.md VC2).

The cell is `examples/export/export_urscript.py`'s with a PLC (`PLC1`)
running the `pick` program and driving the arm from outside its controller,
an operator panel (a start button the cycle waits for, a fault lamp) and a
timeout arm. botrail writes the PLC's program as Structured Text from the
same source as the PLCopen file, hands it to OpenPLC over its web API,
registers itself as the PLC's Modbus slave device (the virtual remote I/O
the sensors and coils live on), starts the PLC, and runs the plant in
wall-clock time: OpenPLC polls botrail, botrail moves the arm, the belt and
the part, and reports what the PLC program actually did against the
reference run (botrail scanning the same program in loopback, the start
button pressed at the same moment).

    docker compose up -d
    python run_demo.py [--seconds 40] [--polling-ms 20] [--out out/] [--check]
    python run_demo.py --studio              # watch it live: the dock's plant bar, force, clips
    python run_demo.py --manual              # … and press the start button in the studio yourself
    python run_demo.py --studio --force-beam # the beam forced low: the PLC's timer runs out
    python run_demo.py --fat [--check]       # the FAT sheet against the PLC: a row per case, the PLC restarted per row

`--check` exits 1 unless the loop ran, every signal of the run matched the
reference within the controller's latency — two polling periods per hop of
the pick chain, 0.32 s at 20 ms polling; the offsets measured are
0.05–0.06 s at the belt and 0.15–0.16 s at the vacuum (the nightly CI job) —
and the part went down within 10 mm of where the reference set it (measured
6–7 mm: the belt halts a poll later, and the part is carried with that
offset). The grasp is the vacuum coil's (`scene.bind_grasp`, within the
suction face's reach): the plant takes the part when the PLC energises the
coil over it and releases it when the coil drops — a real PLC has no
"attach" command, so none crosses the wire. The PLC reports its active
step in a word (`monitor=True` on its node, `pick.step` on the exchange), so
the plant records the PLC's own step bands and the studio's SFC chart shows
which arm it took. `--fat` runs the cell's FAT sheet (`bt.vc.run_fat`)
against the PLC instead of one scripted cycle — the nominal cycle, the
beam dark, no start — the same rows `fat_cases` holds offline; the PLC is
stopped and started for every row. The runtime's web UI is at http://127.0.0.1:8080
(openplc / openplc) while the container runs; the container shares the
host's network so the PLC dials botrail on loopback.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
EXAMPLES = HERE.parents[1]
sys.path.insert(0, str(EXAMPLES / "export"))
sys.path.insert(0, str(HERE.parent))

import botrail as bt  # noqa: E402
from _openplc import OpenPlc, host_ip_for_the_plc  # noqa: E402

OPENPLC = "http://127.0.0.1:8080"
PLC_PORT = 5020
CONTAINER = "botrail-openplc"




# ------------------------------------------------------------------ the cell


def build(monitor: bool = True) -> bt.Scene:
    """The pick cell with a PLC driving the arm: `PLC1` runs `pick`, the UR
    controller is the robot's; the PLC's image starts at %IX100.0 / %QX100.0 /
    %QW100 — where OpenPLC maps a Modbus slave device.

    Over the export example's cell this one has an operator panel — a start
    button the cycle waits for, a fault lamp — and a timeout arm: a part that
    never reaches the beam (the beam forced low from the studio's I/O table,
    say) runs the PLC's timer out, the lamp comes on and the arm goes home.
    With `monitor` the PLC reports its active step in a word the plant records,
    so the studio's SFC chart shows the PLC's own program."""
    import export_urscript as pick

    scene = pick.build_cell()
    scene.define_signal("fault")
    # The operator's station stands in front of the cell, outside the
    # keep-out line and out of the arm's reach: a post with the panel tilted
    # up on it, watching no robot — only a hand (a click) presses its button.
    sx, sy = PANEL_XY
    post = scene.add_box("station/post", size=(0.06, 0.06, 0.98), position=(sx, sy + 0.03, 0.49),
                         color=(0.62, 0.63, 0.65))
    scene.set_obstacle_material(post, metalness=0.6, roughness=0.4)
    scene.add_box("station/foot", size=(0.30, 0.30, 0.02), position=(sx, sy + 0.03, 0.01), color=(0.25, 0.26, 0.28))
    bt.parts.operator_panel(
        scene, "panel", (sx, sy, 1.04), yaw=0.0, tilt=0.5, size=(0.16, 0.14),
        buttons=("cycle_start",), lamps={"fault": "red"}, watch_robots=[],
    )
    author_program(scene)
    # The arm's own controller: its standard I/O takes start and done, and a
    # word takes the program number the PLC selects the motion by (on a real
    # UR, a general-purpose integer register over the fieldbus).
    scene.add_io_node("UR", kind="robot_controller", robots=["simple_arm"],
                      channels=bt.io.ur_standard() + bt.io.word(count=1))
    scene.add_io_node(
        "PLC1", kind="plc", programs=["pick"], monitor=monitor,
        channels=bt.io.di8(base="%IX100.0") + bt.io.do8(base="%QX100.0") + bt.io.word(count=3, base="%QW100"),
    )
    scene.auto_assign_io()
    return scene


def author_program(scene: bt.Scene) -> None:
    """The pick program with a start button and a timeout arm. The grasp is
    the vacuum coil's (`bind_grasp`, within the suction face's reach); the
    program never attaches."""
    import export_urscript as pick

    scene.bind_grasp("vacuum", parts=["part"], reach_m=pick.GRASP_REACH)
    sq = scene.sequence("pick")
    sq.step("ready", actions=[bt.seq.set_signal("fault", False)],
            transition=bt.seq.rising("panel/cycle_start"))
    sq.step("feed", actions=[bt.seq.start("conv"), bt.seq.motion("to_pick")])
    # The part arrives — or it does not, and the timer says so.
    arrival = sq.select("arrival")
    part = arrival.when(bt.seq.rising("part_at_pick"))
    part.step("halt", actions=[bt.seq.stop("conv")])
    part.step("descend", actions=[bt.seq.motion("pick_down")])
    part.step("grip", actions=[bt.seq.set_signal("vacuum")], transition=bt.seq.elapsed(0.3))
    judge = part.select("judge")
    judge.when(bt.seq.signal("spec_ok")).step(
        "place", actions=[bt.seq.motion("place")]
    ).step("release", actions=[bt.seq.set_signal("vacuum", False), bt.seq.motion("clear_tray")])
    reject = judge.when(bt.seq.otherwise())
    reject.step("to reject", actions=[bt.seq.motion("to_reject")])
    reject.step("drop", actions=[bt.seq.set_signal("vacuum", False), bt.seq.motion("clear_bin")])
    none = arrival.when(bt.seq.elapsed(TIMEOUT_S))
    none.step("no part", actions=[bt.seq.stop("conv"), bt.seq.set_signal("fault", True)])
    sq.step("return", actions=[bt.seq.motion("home")])


#: The operator's station: in front of the cell, outside the keep-out line.
PANEL_XY = (-0.55, -0.72)
#: The arrival timeout of the pick program (plant seconds).
TIMEOUT_S = 8.0
#: When the operator presses start in the scripted runs (plant seconds).
PRESS_AT_S = 0.5
#: How far the PLC's run may set the part down from the reference's (mm):
#: the belt's overrun while a polled PLC reads the beam and drops the coil.
SET_DOWN_TOLERANCE_MM = 10.0
#: When the FAT rows press start: a PLC restarted for the row needs a moment
#: before it polls (plant seconds).
FAT_PRESS_AT_S = 2.0


def reference(scene: bt.Scene, force_beam: bool = False):
    """The run as designed: botrail scans `pick` itself through the
    exchange (loopback), the operator presses start at `PRESS_AT_S` — what
    the OpenPLC run is compared with. Deterministic."""
    with bt.vc.loopback(scene, ["pick"], host="PLC1", studio=False) as s:
        if force_beam:
            s.force("part_at_pick", False)
        s.run(for_=PRESS_AT_S)
        s.press("panel/cycle_start")
        s.run()
        return s.timeline()


def fat_cases(ref_duration: float, tolerance: float) -> list:
    """The FAT sheet of the pick cell (`bt.vc.TestCase`): one cycle on the
    start button, the beam dark so the timer runs out, and no start at all.
    The same rows hold offline (loopback) and against the PLC (`--fat`);
    `ref_duration` is the reference run's, `tolerance` the controller's
    latency over the pick chain."""
    vc = bt.vc
    cycle = ref_duration - PRESS_AT_S + 1.0 + tolerance
    return [
        vc.TestCase("TC-01 start → one cycle", press=["panel/cycle_start"], at=vc.elapsed(FAT_PRESS_AT_S),
                    expect=[vc.cycle_within(cycle, "pick"), vc.step_reached("pick", "place"),
                            vc.rises("vacuum", within=cycle), vc.never("fault")],
                    notes="the scripted cycle as a row: picked, placed, no fault"),
        # The arm the PLC took shows in what it switched — the lamp, no
        # vacuum, `grip` never entered — not in `no part` itself: that step
        # is instantaneous (stop the belt, set the lamp, move on), and a
        # polled monitor word never shows a step the PLC left in the same
        # scan it entered it.
        vc.TestCase("TC-02 no part at the beam", forces={"part_at_pick": False}, press=["panel/cycle_start"],
                    at=vc.elapsed(FAT_PRESS_AT_S),
                    expect=[vc.rises("fault", within=TIMEOUT_S + 2.0 + tolerance), vc.never("vacuum"),
                            vc.never(vc.step("pick", "grip")),
                            vc.cycle_within(TIMEOUT_S + 6.0 + tolerance, "pick")],
                    notes="the beam forced low: the timer runs out, the lamp comes on, the arm goes home"),
        vc.TestCase("TC-03 no start, no cycle", for_=5.0,
                    expect=[vc.stalls("pick", at="ready"), vc.never("conv"), vc.no_motion("simple_arm")],
                    notes="nothing pressed: the PLC waits at `ready` and nothing moves"),
    ]




# -------------------------------------------------------------------- demo


def run_fat_against_the_plc(scene, xc, plc: OpenPlc, bind: str, out: Path, *, cases: list, horizon: float,
                            studio: bool = False):
    """The FAT sheet with the PLC driving: every row is a session of its
    own — a fresh Modbus server on the port, the PLC stopped before it and
    started once the server listens, the row's clock starting when the PLC
    polls — and the sheet is the same `FatReport` the offline run gives."""
    allow_remote = bind != "127.0.0.1"

    def links():
        return {"PLC1": bt.vc.modbus_server(xc, f"{bind}:{PLC_PORT}", iec_base=100, allow_remote=allow_remote)}

    def before(case) -> None:
        plc.stop()
        time.sleep(1.0)  # the runtime lets go of its slave connection

    def on_open(case, session) -> None:
        plc.start()
        deadline = time.time() + 30.0
        while time.time() < deadline:
            if session.status()["hosts"][0]["exchanges"] > 0:
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("the PLC did not poll botrail within 30 s of starting")
        print(f"  {case.name}: the PLC polls")

    fat = bt.vc.run_fat(scene, cases, ["pick"], links=links, horizon=horizon, before=before, on_open=on_open,
                        title="pick cell FAT (OpenPLC)", studio=studio)
    plc.stop()
    fat.save(out / "pick_fat.md")
    fat.save(out / "pick_fat.json")
    print(fat.to_markdown())
    print(json.dumps({"fat": fat.counts, "ok": fat.ok, "mode": fat.mode}, indent=2))
    return fat


def part_set_down(ref, tl) -> dict:
    """Where the PLC's run left the part against where the reference set it
    down: the distance between the two final positions, in millimetres. The
    belt halts a poll or two later under a polled PLC, so the part stops a
    few millimetres further on and is carried with that offset; anything
    more is the grasp or the place gone wrong."""
    at = tl.object_pose("part", tl.duration)[0]
    want = ref.object_pose("part", ref.duration)[0]
    offset = sum((a - b) ** 2 for a, b in zip(at, want)) ** 0.5
    return {"at_m": [round(v, 4) for v in at], "reference_m": [round(v, 4) for v in want],
            "offset_mm": round(offset * 1000.0, 1)}


def _signal_matches(diff, name: str, rising_only: bool = False) -> bool:
    """A signal's edges matched within tolerance — no missing or extra. With
    `rising_only`, only its rising edge is judged (a sensor whose fall is the
    grasp that does not cross the wire)."""
    findings = [f for f in diff.findings() if f["signal"] == name]
    if rising_only:
        findings = [f for f in findings if "rose" in f["message"] or "rising" in f.get("code", "")]
    return not findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seconds", type=float, default=None, help="plant seconds to run (default: the reference run + 3 s)")
    parser.add_argument("--polling-ms", type=int, default=20, help="OpenPLC slave polling period (default 20)")
    parser.add_argument("--tolerance", type=float, default=None,
                        help="diff tolerance, seconds (default 2 polling periods per hop of the pick chain, 8 hops)")
    parser.add_argument("--out", default=str(HERE / "out"), help="where the ST, exchange, recording and USD go")
    parser.add_argument("--openplc", default=OPENPLC, help="OpenPLC web UI")
    parser.add_argument("--plc-reaches-us-at", default=None, help="the IP the PLC dials for this machine (default: resolved from the container)")
    parser.add_argument("--studio", action="store_true", help="open the studio on the cell and stream the run there live")
    parser.add_argument("--manual", action="store_true",
                        help="do not press the start button from the script: click it in the studio (implies --studio)")
    parser.add_argument("--force-beam", action="store_true",
                        help="force the beam low before the cycle: the PLC runs its timer out and takes the fault arm")
    parser.add_argument("--fat", action="store_true",
                        help="run the FAT sheet against the PLC instead of one scripted cycle (the PLC is stopped "
                             "and started for every row); --check exits 1 unless every row passes")
    parser.add_argument("--check", action="store_true", help="exit 1 unless the diff is clean")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    studio_wanted = args.studio or args.manual

    scene = build()
    ref = reference(scene, force_beam=args.force_beam)
    xc = bt.vc.exchange(scene, "PLC1")
    st = scene.st("PLC1", cycle=False)  # one cycle, then the program parks
    (out / "pick_PLC1.st").write_text(st, encoding="utf-8")
    xc.save(out / "pick_exchange_PLC1.md")
    scene.export_plcopen(out / "pick.plcopen.xml", name="pick cell")
    print(f"reference: {ref.duration:.2f} s; exchange PLC1: {len(xc.inputs)} inputs, {len(xc.outputs)} outputs")

    server = None
    if studio_wanted:
        server = bt.studio(scene, block=False, open_browser=True)
        print(f"studio: {server.url} — the dock follows the plant once the PLC runs; "
              f"{'click the green button to start the cycle' if args.manual else 'the script presses start'}")

    plc_side_ip = args.plc_reaches_us_at or host_ip_for_the_plc()
    bind = "127.0.0.1" if plc_side_ip == "127.0.0.1" else "0.0.0.0"
    link = bt.vc.modbus_server(xc, f"{bind}:{PLC_PORT}", iec_base=100, allow_remote=bind != "127.0.0.1")
    (out / "pick_modbus_PLC1.md").write_text(link.register_map_markdown(), encoding="utf-8")
    sizes = {t: max((r.address + r.count for r in link.rows if r.table == t), default=0) for t in
             ("discrete_input", "coil", "input_register", "holding_register")}
    link.start()

    plc = OpenPlc(args.openplc)
    plc.wait_ready()
    plc.stop()
    plc.settings(args.polling_ms)
    plc.clear_slave_devices()
    print(f"the PLC reaches this machine at {plc_side_ip}:{PLC_PORT}")
    plc.add_slave_device("botrail", plc_side_ip, PLC_PORT,
                         di=sizes["discrete_input"], coils=sizes["coil"],
                         ir=sizes["input_register"], hr=sizes["holding_register"])
    log = plc.load_program("botrail_pick", st)
    (out / "openplc_compile.log").write_text(log, encoding="utf-8")
    print("OpenPLC compiled the program")

    # Every hop through the controller costs up to a poll in and a poll out,
    # and the offsets add up along the pick chain: the beam read, the belt
    # coil, the descent's command and its done, the vacuum coil, the place's
    # command and its done, the release — eight hops (measured with 20 ms
    # polling: 0.05–0.06 s at the belt, 0.15–0.16 s at the vacuum, about a
    # poll per hop). The tolerance is the worst case, not a typical run.
    hops = 8
    tolerance = args.tolerance if args.tolerance is not None else 2 * hops * args.polling_ms / 1000.0

    if args.fat:
        link.stop()  # every row opens its own server on the port
        fat = run_fat_against_the_plc(
            scene, xc, plc, bind, out, cases=fat_cases(ref.duration, tolerance),
            horizon=ref.duration + TIMEOUT_S + 10.0, studio=studio_wanted,
        )
        if server is not None:
            server.stop()
        return 0 if (fat.ok or not args.check) else 1

    if args.seconds is not None:
        seconds = args.seconds
    elif args.manual:
        seconds = 120.0
    else:
        seconds = ref.duration + 3.0
    with bt.vc.Session(scene, ["pick"], links={"PLC1": link}, pace="realtime", max_duration=seconds + 5.0) as s:
        plc.start()
        if args.force_beam:
            s.force("part_at_pick", False)
        if args.manual:
            s.run(for_=seconds)
        else:
            s.run(for_=PRESS_AT_S)
            s.press("panel/cycle_start")
            s.run(for_=seconds - PRESS_AT_S)
        plc.stop()
        tl = s.timeline()
        rec = s.recording()
        stats = s.stats()
        clips = s.clips
    rec.save(out / "pick_inputs.csv")
    tl.export_usd(out / "pick_emulated.usdc")
    for k, clip in enumerate(clips):
        clip.export_usd(out / f"pick_clip{k + 1}.usdc")
    trace = bt.trace.from_timeline(tl)
    print(json.dumps({"stats": stats, "polls": link.polls, "writes_recorded": len(rec.records),
                      "clips": len(clips)}, indent=2))
    if not trace.edges("conv")[0]:
        # The PLC never switched the belt on: it did not reach the plant.
        print("the PLC wrote nothing the plant could see — its Modbus master never reached botrail "
              "(`docker logs botrail-openplc` shows its connection attempts)")
        if server is not None:
            server.stop()
        return 1
    d = ref.diff(trace, tolerance=tolerance, align_on="conv")
    (out / "pick_diff.md").write_text(d.to_markdown(), encoding="utf-8")
    print(d.to_markdown())
    steps = [(name, round(a, 2), round(b, 2)) for name, a, b in tl.step_spans]
    print("the PLC's steps, as it reported them:", steps)
    # The feed phase is the PLC driving the plant directly: the belt coil it
    # writes, the beam it reads. The pick is the coil-bound grasp: the PLC
    # energises `vacuum`, the plant takes the part, the place motion carries
    # it off the beam, the coil drops and the part is released. Every edge
    # within a few polling periods of the reference is the loop closed end to
    # end; the step word says which arm the PLC took.
    feed_ok = _signal_matches(d, "conv") and _signal_matches(d, "part_at_pick", rising_only=True)
    landed = part_set_down(ref, tl)
    result = {
        "loop": {"polls": link.polls, "writes": len(rec.records), "overruns": stats["overruns"]},
        "feed_phase_reproduces": feed_ok,
        "full_diff_ok": d.ok,
        "part_set_down": landed,
        "plc_steps": [name for name, _, _ in tl.step_spans],
        "note": None if d.ok else (
            "the PLC did not reach the plant or the belt diverged" if not feed_ok
            else "the feed reproduced but the pick diverged — see the findings above"),
    }
    print(json.dumps(result, indent=2))
    if server is not None:
        server.stop()
    # The gate: the loop ran, the whole run matched the reference, and the
    # part went down where the reference put it.
    ran = link.polls > 0 and len(rec.records) > 4
    placed = landed["offset_mm"] <= SET_DOWN_TOLERANCE_MM
    return 0 if ((ran and d.ok and placed) or not args.check) else 1


if __name__ == "__main__":
    sys.exit(main())
