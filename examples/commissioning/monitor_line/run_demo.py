"""A machine's I/O log replayed in 3D, and a running PLC watched read-only
(design-plc-resolution.md §11, PM0 / PM1).

The machine is the ladder line of `../ladder_line` — a belt, two stations,
a feeder, a panel — and its PLC. Two ways of looking at a machine that is
not botrail's to drive:

* **A log, replayed.** The line's I/O over a run is a CSV as a PLC's data
  logger writes it: a time column and one column per address (`%IX100.5`,
  `%QX100.0`). `bt.vc.IoLog` reads it against the cell's exchange, and
  `bt.vc.monitor` replays it: the coils the PLC wrote move the model, the
  sensors it read are compared with the model's. Delay one sensor's column
  by half a second and the comparison names it.

      python run_demo.py [out.usd] [--studio]
      python run_demo.py --replay out/equipment_io.csv [--studio]   # any log of the line (the OpenPLC run's)

* **A running PLC, read only.** OpenPLC runs the line (`--equipment`: the
  PLC and a plant session, standing in for the machine), and a second
  botrail process mirrors it through `bt.vc.modbus_reader` — a Modbus
  client that only ever sends read requests — while the model follows in
  3D.

      docker compose up -d                              # the PLC (../ladder_line/compose.yaml)
      python run_demo.py --equipment [--fault s2]       # terminal 1: the machine
      python run_demo.py --monitor [--studio]           # terminal 2: the mirror, read-only
      python run_demo.py --judge [--fault s2] [--check] # both, and the verdict

  `--fault s2` breaks that sensor on the machine mid-run: the mirror shows
  where the machine left the model.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import importlib.util
import threading

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import botrail as bt  # noqa: E402

# The cell and its programs are the ladder line's (../ladder_line/run_demo.py),
# loaded by path: the two files share a name.
_spec = importlib.util.spec_from_file_location("ladder_line_demo", HERE.parent / "ladder_line" / "run_demo.py")
line = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(line)

PLC = "PLC1"
PLC_MODBUS = "127.0.0.1:502"      # OpenPLC's own Modbus server: its I/O image, readable
WORKS = ("work1", "work2", "work3")
DELAYED, DELAY_S = "s2", 0.5


def summary(tl) -> dict:
    return {
        "duration_s": round(tl.duration, 2),
        "works_discharged": len(tl.signal("s_out").rising_edges()),
        "belt_starts": len(tl.signal("conv").rising_edges()),
        "feeds": len(tl.signal("feeder").rising_edges()),
    }


def record(scene: bt.Scene) -> tuple[bt.vc.IoLog, object]:
    """The line's run with a logger on PLC1's wires: the reference run
    (botrail scans PLC1's programs through the exchange), replayed with a
    tap — the table a data logger would have written."""
    xc = bt.vc.exchange(scene, PLC, line.PROGRAMS)
    with bt.vc.loopback(scene, line.PROGRAMS, host=PLC, studio=False) as s:
        s.run()
        recording = s.recording()
    tapped = bt.vc.tap(bt.vc.replay(recording), xc)
    with bt.vc.Session(scene, line.PROGRAMS, links={PLC: tapped}, studio=False) as s:
        s.run()
        tl = s.timeline()
    return tapped.log, tl


def replay(scene: bt.Scene, log: bt.vc.IoLog, studio: bool = False):
    """The log replayed on the model. The operator's script is left out:
    the buttons are in the log, as the PLC read them."""
    with bt.vc.monitor(scene, line.PLC_PROGRAMS, log, studio=studio) as m:
        m.run()
        return m.timeline(), m.deviations(), m.report()


def same_motion(a, b) -> bool:
    end = min(a.duration, b.duration)
    return all(a.object_pose(w, t) == b.object_pose(w, t) for w in WORKS for t in (end * k / 60 for k in range(61)))


def offline(scene: bt.Scene, out: Path):
    """The log written, read back and replayed, then with one sensor's
    column late: `(the judgement holds, the late replay's timeline)`."""
    log, machine = record(scene)
    path = out / "line_io.csv"
    log.save(path)
    xc = bt.vc.exchange(scene, PLC, line.PLC_PROGRAMS)
    read = bt.vc.IoLog.load(path, xc)
    tl, deviations, report = replay(scene, read)
    (out / "replay.md").write_text(report, encoding="utf-8")
    moved = same_motion(machine, tl)
    late = read.delayed(DELAYED, DELAY_S)
    late.save(out / "line_io_late.csv")
    tl_late, found, report_late = replay(scene, late)
    (out / "replay_late.md").write_text(report_late, encoding="utf-8")
    named = bool(found) and all(d.lane == DELAYED and d.kind == "late" and abs(d.duration - DELAY_S) < 0.02
                                for d in found)
    print(json.dumps({
        "log": {"file": str(path), "rows": len(path.read_text().splitlines()) - 1, "points": len(read.labels),
                "not_in_the_cell": read.unmatched},
        "replay": {**summary(tl), "same_motion_as_the_machine": moved, "deviations": len(deviations)},
        f"{DELAYED}_delayed_{DELAY_S}s": [d.note for d in found],
    }, indent=1))
    ok = moved and not deviations and named
    print("the log replays as the machine ran, and the delayed sensor is named" if ok
          else "the replay does not hold")
    return ok, tl_late


def equipment(scene: bt.Scene, args, out: Path) -> int:
    """The machine: OpenPLC runs PLC1's program and a plant session is the
    line it drives. `--fault` breaks a sensor (stuck off) from `--fault-at`."""
    from _openplc import OpenPlc, host_ip_for_the_plc

    xc = bt.vc.exchange(scene, PLC, line.PROGRAMS)
    st = scene.st(PLC, line.PROGRAMS, cycle=False)
    plc_side_ip = host_ip_for_the_plc()
    bind = "127.0.0.1" if plc_side_ip == "127.0.0.1" else "0.0.0.0"
    link = bt.vc.modbus_server(xc, f"{bind}:{line.PLC_PORT}", iec_base=100, allow_remote=bind != "127.0.0.1")
    sizes = {t: max((r.address + r.count for r in link.rows if r.table == t), default=0) for t in
             ("discrete_input", "coil", "input_register", "holding_register")}
    tapped = bt.vc.tap(link, xc)
    link.start()
    plc = OpenPlc(args.openplc_url)
    plc.wait_ready()
    plc.stop()
    plc.settings(args.polling_ms)
    plc.clear_slave_devices()
    plc.add_slave_device("botrail", plc_side_ip, line.PLC_PORT, di=sizes["discrete_input"], coils=sizes["coil"],
                         ir=sizes["input_register"], hr=sizes["holding_register"])
    plc.load_program("botrail_ladder", st)
    print("equipment: OpenPLC compiled the program", flush=True)
    with bt.vc.Session(scene, line.PROGRAMS, links={PLC: tapped}, pace="realtime", max_duration=120.0,
                       studio=False) as s:
        if args.fault:
            s.inject(bt.io.stuck(args.fault, False), at={"program": "operator", "step": args.fault_at})
        plc.start()
        print("equipment: running", flush=True)
        s.run(for_=1.5)
        try:
            s.run(timeout=30.0)             # … until the operator's script ends
        except TimeoutError:
            # A broken sensor: the line waits on it for good.
            print(f"equipment: stalled — {dict(s.steps())['auto']!r} never ends", flush=True)
        plc.stop()
        tl = s.timeline()
    tapped.log.save(out / "equipment_io.csv")
    print("equipment: " + json.dumps({**summary(tl), "polls": link.polls, "fault": args.fault}), flush=True)
    return 0


def mirror(scene: bt.Scene, args, out: Path, until=None) -> tuple[dict, list, object]:
    """The mirror: PLC1's image read from OpenPLC's Modbus server — read
    requests only — moving the model; nothing is written anywhere."""
    xc = bt.vc.exchange(scene, PLC, line.PLC_PROGRAMS)
    reader = bt.vc.modbus_reader(xc, args.monitor or PLC_MODBUS, period=args.period)
    if args.studio:
        server = bt.studio(scene, block=False)
        print(f"studio: {server.url}", flush=True)
    with bt.vc.monitor(scene, line.PLC_PROGRAMS, reader, studio=args.studio) as m:
        m.run(until=until, for_=None if until else args.seconds)
        tl = m.timeline()
        deviations = m.deviations()
        m.log().save(out / "monitor_io.csv")
        (out / "monitor.md").write_text(m.report(), encoding="utf-8")
        stats = m.stats()
    result = {**summary(tl), "reads": stats["reads"], "function_codes_sent": sorted(reader.function_codes),
              "overruns": stats["overruns"], "deviations": [d.note for d in deviations]}
    return result, deviations, tl


def judge(scene: bt.Scene, args, out: Path) -> int:
    """The machine in a second process, the mirror here: the model must
    make the same works, and — with a fault on the machine — name it."""
    command = [sys.executable, str(Path(__file__).resolve()), "--equipment", "--out", str(out),
               "--openplc-url", args.openplc_url, "--polling-ms", str(args.polling_ms)]
    if args.fault:
        command += ["--fault", args.fault, "--fault-at", args.fault_at]
    machine = subprocess.Popen(command, stdout=subprocess.PIPE, text=True)
    lines: list[str] = []

    def pump() -> None:
        for text in machine.stdout:  # type: ignore[union-attr]
            lines.append(text.strip())
            print(text.strip(), flush=True)

    threading.Thread(target=pump, daemon=True).start()
    ended: list[float] = []

    def machine_done(m) -> bool:
        # A few seconds past the machine's end, so the last readings land.
        if machine.poll() is not None and not ended:
            ended.append(time.monotonic())
        return bool(ended) and time.monotonic() - ended[0] > 1.0

    got, deviations, tl = mirror(scene, args, out, until=machine_done)
    machine.wait(timeout=30)
    last = next((text for text in reversed(lines) if text.startswith("equipment: {")), None)
    want = json.loads(last[len("equipment: "):]) if last else {}
    print("mirror: " + json.dumps(got), flush=True)
    tl.export_usd(out / "monitor_mirror.usdc")
    read_only = set(got["function_codes_sent"]) <= {1, 2, 3, 4}
    follows = all(got[k] == want.get(k) for k in ("works_discharged", "belt_starts", "feeds"))
    if args.fault:
        named = any(d.lane == args.fault for d in deviations)
        ok = read_only and got["reads"] > 0 and named
        print(f"the mirror names the machine's `{args.fault}`" if named else f"the mirror missed `{args.fault}`")
    else:
        ok = read_only and got["reads"] > 0 and follows and not deviations
        print("the mirror followed the machine, reading only" if ok else "the mirror did not follow the machine")
    return 0 if (ok or not args.check) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("usd", nargs="?", default=None, help="write the replayed (delayed) log as USD")
    parser.add_argument("--studio", action="store_true", help="open the studio: on the replay, or on the live mirror")
    parser.add_argument("--replay", default=None, metavar="CSV", help="replay this log of the line instead")
    parser.add_argument("--equipment", action="store_true", help="be the machine: OpenPLC and the line it drives")
    parser.add_argument("--monitor", nargs="?", const=PLC_MODBUS, default=None, metavar="HOST:PORT",
                        help=f"mirror a running PLC read-only over Modbus ({PLC_MODBUS})")
    parser.add_argument("--judge", action="store_true", help="run the machine and the mirror, and compare")
    parser.add_argument("--fault", default=None, metavar="SENSOR", help="a sensor that breaks on the machine")
    parser.add_argument("--fault-at", default="cycle stop: press", help="the operator step the fault begins at")
    parser.add_argument("--seconds", type=float, default=40.0, help="how long --monitor watches")
    parser.add_argument("--period", type=float, default=0.02,
                        help="the mirror's poll period, seconds (the feeder's coil is up for one PLC scan: a "
                             "slower poll misses it, and the model never feeds the work)")
    parser.add_argument("--openplc-url", default=line.OPENPLC)
    parser.add_argument("--polling-ms", type=int, default=20)
    parser.add_argument("--out", default=str(HERE / "out"))
    parser.add_argument("--check", action="store_true", help="exit 1 unless the judgement holds")
    args = parser.parse_args()

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    scene = line.build()
    if args.equipment:
        return equipment(scene, args, out)
    if args.judge:
        return judge(scene, args, out)
    if args.monitor:
        got, _, _ = mirror(scene, args, out)
        print("mirror: " + json.dumps(got, indent=1))
        return 0
    if args.replay:
        log = bt.vc.IoLog.load(args.replay, bt.vc.exchange(scene, PLC, line.PLC_PROGRAMS))
        tl, deviations, report = replay(scene, log)
        print(json.dumps({"log": str(args.replay), "points": len(log.labels), **summary(tl),
                          "deviations": [d.note for d in deviations]}, indent=1))
        ok = True
    else:
        ok, tl = offline(scene, out)
    if args.usd:
        tl.export_usd(args.usd)
        print(f"wrote {args.usd}")
    if args.studio:                      # the readings on the lanes, red where they part (the late `s2`)
        scene.show_timeline(tl)
        bt.studio(scene)
    return 0 if (ok or not args.check) else 1


if __name__ == "__main__":
    sys.exit(main())
