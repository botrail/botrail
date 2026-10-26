"""A fixed-pitch feed line as the plant of OpenPLC — the PLC writes the
axis position, botrail answers in-position (design-virtual-commissioning.md
VC6).

The cell is the vocabulary a PLC drives a cell with and nothing else: a
feed axis that carries a strip under a press in 100 mm indexes, a
double-solenoid press cylinder, an encoder on the feed, a start button and
a signal tower. The PLC's program is botrail's Structured Text from the
`index` sequence: on start it writes the target position (an INT in
millimetres — `bt.io.word(unit="mm")` — into a holding register), waits
for the in-position contact, energises the press's `extend` coil, waits for
the `extended` limit switch, dwells, retracts, waits for `retracted`, and
indexes again; after three parts the strip returns home. botrail is the
PLC's Modbus slave: the axis, the cylinder, the switches and the encoder
word live on it. The run is diffed against the reference (botrail scanning
the same program in loopback, start pressed at the same moment), and the
PLC's position writes are read back from the recording.

    docker compose up -d
    python run_demo.py [--seconds 30] [--polling-ms 20] [--out out/] [--check]
    python run_demo.py --studio               # watch it live: the encoder lane climbs under the press
    python run_demo.py --manual               # … and press the start button in the studio yourself

`--check` exits 1 unless the loop ran, the PLC wrote the four positions
(100, 200, 300, 0 mm) and every signal of the run matched the reference
within the controller's latency. That latency **accumulates**: every hop
through a polled controller — a word written, an in-position read, a coil,
a switch — costs up to a polling period each way, an index is eight such
hops and the cycle three indexes and a return, so the default tolerance is
two polling periods per hop over the whole cycle (1.04 s at 20 ms) and the
run reports the drift index by index (measured: about 0.15 s per index,
one polling period per hop).
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import botrail as bt  # noqa: E402
from _openplc import OpenPlc, host_ip_for_the_plc  # noqa: E402

OPENPLC = "http://127.0.0.1:8080"
PLC_PORT = 5020

#: The index pitch (m) and the number of parts on the strip.
PITCH_M = 0.1
PARTS = 3
#: The press is given this long to reach its switch before the PLC calls a jam.
PRESS_TIMEOUT_S = 2.0
#: When the operator presses start in the scripted runs (plant seconds).
PRESS_AT_S = 0.5
#: The PLC's positions, in the order it writes them (mm).
EXPECTED_POSITIONS = [100, 200, 300, 0]


def build(monitor: bool = True) -> bt.Scene:
    """The feed line: a strip on a linear axis under a press cylinder, a
    start button, a signal tower, and the PLC node whose image starts at
    %IX100.0 / %QX100.0 / %IW100 / %QW100 — where OpenPLC maps a Modbus
    slave device. The word channels carry millimetres: the PLC's registers
    are 16-bit, so the axis position and the encoder cross as scaled INTs."""
    scene = bt.Scene()
    scene.add_box("bed", size=(0.9, 0.2, 0.05), position=(0.35, 0.0, 0.475), color=(0.42, 0.44, 0.47))
    scene.add_box("strip", size=(0.5, 0.08, 0.006), position=(0.05, 0.0, 0.503), color=(0.75, 0.70, 0.45))
    for k in range(PARTS):
        scene.add_box(f"blank{k + 1}", size=(0.03, 0.03, 0.004), position=(-0.1 + 0.1 * k, 0.0, 0.508),
                      color=(0.30, 0.32, 0.35))
    # The strip and the blanks ride the feed axis (100 mm per index).
    scene.add_linear_axis("feed", objects=["strip"] + [f"blank{k + 1}" for k in range(PARTS)],
                          axis=(1.0, 0.0, 0.0), speed=0.25, range=(0.0, 0.4), position=0.0, encoder=True)
    # The press over the third blank's first stop: a double-solenoid cylinder
    # with a 30 mm stroke that touches the strip when extended.
    bt.parts.cylinder(scene, "press", (0.2, 0.0, 0.55), axis=(0.0, 0.0, -1.0), stroke=0.03, time=0.3,
                      solenoid="double", bore=0.032, pad=(0.03, 0.03, 0.01))
    scene.define_signal("running")
    scene.define_signal("fault")
    bt.parts.tower_lamp(scene, "tower", (0.85, 0.25, 0.5), tiers={"green": "running", "red": "fault"}, pole=0.12)
    bt.parts.operator_panel(scene, "panel", (-0.35, -0.3, 0.6), yaw=0.0, tilt=0.35, size=(0.12, 0.10),
                            buttons=("cycle_start",), watch_robots=[])
    author_program(scene)
    scene.add_io_node(
        "PLC1", kind="plc", programs=["index"], monitor=monitor,
        channels=bt.io.di8(base="%IX100.0") + bt.io.do8(base="%QX100.0")
        + bt.io.channels("word", 2, prefix="QW", base="%QW100", unit="mm")
        + bt.io.channels("word", 2, prefix="IW", base="%IW100", unit="mm"),
    )
    # The PLC writes the position (a holding register) and reads the
    # encoder (an input register): bound by hand so the tables come out
    # the way OpenPLC maps a slave device; the rest is assigned in order.
    scene.bind_output("feed.position", "PLC1", "QW0", tag="FeedTarget")
    scene.bind_input("feed.encoder", "PLC1", "IW0", tag="FeedActual")
    scene.auto_assign_io()
    return scene


def author_program(scene: bt.Scene) -> None:
    """The index program: start, then per part feed → press → dwell → lift,
    with a jam arm when the press does not reach its switch; the strip
    goes home at the end."""
    sq = scene.sequence("index")
    sq.step("ready", actions=[bt.seq.set_signal("running", False), bt.seq.set_signal("fault", False)],
            transition=bt.seq.rising("panel/cycle_start"))
    sq.step("run", actions=[bt.seq.set_signal("running")])
    for k in range(1, PARTS + 1):
        sq.step(f"feed{k}", actions=[bt.seq.move_to("feed", k * PITCH_M)], transition=bt.seq.device_done("feed"))
        sq.step(f"press{k}", actions=[bt.seq.move_to("press", "extended")])
        down = sq.select(f"down{k}")
        down.when(bt.seq.device_done("press")).step(f"dwell{k}", transition=bt.seq.elapsed(0.3))
        jam = down.when(bt.seq.elapsed(PRESS_TIMEOUT_S))
        jam.step(f"jam{k}", actions=[bt.seq.set_signal("fault", True)])
        sq.step(f"lift{k}", actions=[bt.seq.move_to("press", "retracted")], transition=bt.seq.device_done("press"))
    sq.step("return", actions=[bt.seq.move_to("feed", 0.0), bt.seq.set_signal("running", False)],
            transition=bt.seq.device_done("feed"))


def reference(scene: bt.Scene):
    """The run as designed: botrail scans `index` itself through the
    exchange (loopback), start pressed at `PRESS_AT_S`. Deterministic."""
    with bt.vc.loopback(scene, ["index"], host="PLC1", studio=False) as s:
        s.run(for_=PRESS_AT_S)
        s.press("panel/cycle_start")
        s.run()
        return s.timeline(), s.recording()


def positions_written(recording) -> list[int]:
    """The PLC's position writes, in order, as the register saw them (mm),
    the initial image dropped."""
    writes = [int(v) for t, host, label, v in recording.records if label == "feed.position" and t > 0.0]
    out: list[int] = []
    for w in writes:
        if not out or out[-1] != w:
            out.append(w)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seconds", type=float, default=None, help="plant seconds to run (default: the reference run + 3 s)")
    parser.add_argument("--polling-ms", type=int, default=20, help="OpenPLC slave polling period (default 20)")
    parser.add_argument("--tolerance", type=float, default=None,
                        help="diff tolerance, seconds (default 2 polling periods per hop over the whole cycle: "
                             "8 hops per index, 3 indexes and the return)")
    parser.add_argument("--out", default=str(HERE / "out"), help="where the ST, exchange, register map, recording and USD go")
    parser.add_argument("--openplc", default=OPENPLC, help="OpenPLC web UI")
    parser.add_argument("--plc-reaches-us-at", default=None, help="the IP the PLC dials for this machine (default: resolved from the container)")
    parser.add_argument("--studio", action="store_true", help="open the studio on the cell and stream the run there live")
    parser.add_argument("--manual", action="store_true",
                        help="do not press the start button from the script: click it in the studio (implies --studio)")
    parser.add_argument("--check", action="store_true", help="exit 1 unless the PLC drove the line as designed")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    studio_wanted = args.studio or args.manual

    scene = build()
    ref, ref_rec = reference(scene)
    xc = bt.vc.exchange(scene, "PLC1")
    st = scene.st("PLC1", cycle=False)
    (out / "feed_PLC1.st").write_text(st, encoding="utf-8")
    xc.save(out / "feed_exchange_PLC1.md")
    scene.export_plcopen(out / "feed.plcopen.xml", name="feed line")
    print(f"reference: {ref.duration:.2f} s; exchange PLC1: {len(xc.inputs)} inputs, {len(xc.outputs)} outputs; "
          f"the reference wrote {positions_written(ref_rec)} mm")

    server = None
    if studio_wanted:
        server = bt.studio(scene, block=False, open_browser=True)
        print(f"studio: {server.url} — the encoder lane follows the feed once the PLC runs; "
              f"{'click the green button to start the cycle' if args.manual else 'the script presses start'}")

    plc_side_ip = args.plc_reaches_us_at or host_ip_for_the_plc()
    bind = "127.0.0.1" if plc_side_ip == "127.0.0.1" else "0.0.0.0"
    link = bt.vc.modbus_server(xc, f"{bind}:{PLC_PORT}", iec_base=100, allow_remote=bind != "127.0.0.1")
    (out / "feed_modbus_PLC1.md").write_text(link.register_map_markdown(), encoding="utf-8")
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
    log = plc.load_program("botrail_feed", st)
    (out / "openplc_compile.log").write_text(log, encoding="utf-8")
    print("OpenPLC compiled the program")

    # Every hop through the controller — a switch read, a coil or a word
    # written — costs up to a poll in and a poll out; an index is eight such
    # hops (position, busy, in-position, extend, extended, retract,
    # retracted, the dwell timer) and the offsets add up along the whole
    # cycle: three indexes and the return home.
    hops = 8 * PARTS + 2
    tolerance = args.tolerance if args.tolerance is not None else 2 * hops * args.polling_ms / 1000.0

    if args.seconds is not None:
        seconds = args.seconds
    elif args.manual:
        seconds = 120.0
    else:
        seconds = ref.duration + 3.0
    with bt.vc.Session(scene, ["index"], links={"PLC1": link}, pace="realtime", max_duration=seconds + 5.0) as s:
        plc.start()
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
    rec.save(out / "feed_inputs.csv")
    tl.export_usd(out / "feed_emulated.usdc")
    trace = bt.trace.from_timeline(tl)
    written = positions_written(rec)
    encoder = tl.word("feed.encoder")
    print(json.dumps({"stats": stats, "polls": link.polls, "writes_recorded": len(rec.records),
                      "positions_written_mm": written, "encoder_max_m": round(encoder.max(), 4),
                      "encoder_final_m": round(encoder.final_value(), 4)}, indent=2))
    if not written:
        print("the PLC wrote no position the plant could see — its Modbus master never reached botrail "
              "(`docker logs botrail-openplc` shows its connection attempts)")
        if server is not None:
            server.stop()
        return 1
    d = ref.diff(trace, tolerance=tolerance, align_on="feed")
    (out / "feed_diff.md").write_text(d.to_markdown(), encoding="utf-8")
    print(d.to_markdown())
    steps = [(name, round(a, 2), round(b, 2)) for name, a, b in tl.step_spans]
    print("the PLC's steps, as it reported them:", steps)
    # The controller's drift, index by index: when the press reached its
    # switch in the bake and in the run (the first edge aligns the clocks).
    bake_press = ref.signal("press/extended").rising_edges()
    run_press = tl.signal("press/extended").rising_edges()
    shift = (run_press[0] - bake_press[0]) if bake_press and run_press else 0.0
    drift = [round(r - b - shift, 3) for b, r in zip(bake_press, run_press)]
    print(f"press reached its switch {len(run_press)} times; drift per index after alignment: {drift} s "
          f"(bake {[round(t, 2) for t in bake_press]}, run {[round(t, 2) for t in run_press]})")
    # The judgement: the PLC wrote the positions in millimetres, the plant
    # moved the axis and answered in-position, the press ran its coils and
    # switches — and every edge sits within the controller's latency of
    # the reference.
    result = {
        "loop": {"polls": link.polls, "writes": len(rec.records), "overruns": stats["overruns"]},
        "positions_as_designed": written == EXPECTED_POSITIONS,
        "full_diff_ok": d.ok,
        "tolerance_s": tolerance,
        "drift_per_index_s": drift,
        "plc_steps": [name for name, _, _ in tl.step_spans],
    }
    print(json.dumps(result, indent=2))
    if server is not None:
        server.stop()
    ran = link.polls > 0 and len(rec.records) > 4
    return 0 if ((ran and d.ok and written == EXPECTED_POSITIONS) or not args.check) else 1


if __name__ == "__main__":
    sys.exit(main())
