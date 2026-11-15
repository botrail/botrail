"""A filter unit run on numbers: setpoints in words, a 4–20 mA transmitter,
comparisons and arithmetic in the rungs (design-plc-resolution.md PF2).

A pump pushes liquid from a tank through a cartridge filter and an outlet
valve. The element clogs as it filters, so the differential pressure across
it climbs; a transmitter (0–100 kPa over 4–20 mA into a 0–4000 count
module) sends it to `PLC1`, whose program is rungs alone:

* the run relay, the valve's solenoid, the pump behind the valve's open
  switch;
* `dp >= dp_warn` lights the "change the element soon" lamp, `dp >=
  dp_limit` trips the clogged alarm and stops the unit — both setpoints
  are words;
* a wire-break rung on the raw counts (`dp.raw < -100`: an open loop reads
  0 mA, a quarter of the span under the bottom count);
* the valve watchdog counts milliseconds in a word while the valve is
  commanded and not confirmed, against the setpoint `valve_time`;
* a peak hold moved into `dp_peak`.

The clogging itself is a rung too, on the plant's side (`process`, marked
`.plant()`: the equipment's own program, on no controller and out of the
I/O map): while liquid flows it adds to `clog` and moves it into the
pressure the transmitter measures. No fluid solver: an integrator in a
word.

The operator's script (`operator`, on the desk) runs the story the bake
tells: start, filter until the alarm, change the element, reset, key a
lower warning setpoint in (`dp_warn` 60 → 40), start again — the warning
now comes 2.5 s sooner — and stop.

    python run_demo.py [out.usd] [--studio]
    python run_demo.py --openplc [--check]     # the same rungs as ST on OpenPLC (docker compose up -d)

Without `--openplc` it bakes the story, checks that the loopback through
`PLC1`'s exchange is the bake bit for bit, then runs three live sessions
in which a hand changes a number while the unit runs: a shorter
`valve_time` (a false valve alarm), a broken transmitter wire, a forced
pressure. With it, the Structured Text botrail writes for `PLC1` — INT
and REAL words, the scaling of the counts, the comparisons — is compiled
and run by OpenPLC over Modbus and the run is diffed against the bake.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import botrail as bt  # noqa: E402
from botrail.seq import all_of, any_of, not_, rising, signal, word  # noqa: E402

PLC_PROGRAMS = ["filter"]
PROGRAMS = PLC_PROGRAMS + ["process", "operator"]

CLOG_NEW = 12.0          # kPa across a fresh element
CLOG_PER_SCAN = 0.08     # kPa a 10 ms scan while liquid flows (8 kPa/s — a clogging on fast-forward)
DP_WARN, DP_LIMIT = 60.0, 80.0
DP_WARN_RETUNED = 40.0
VALVE_TIME_MS = 600      # the valve strokes in 400 ms
OPENPLC = "http://127.0.0.1:8080"
PLC_PORT = 5020

STEEL = (0.62, 0.65, 0.69)
PIPE = (0.45, 0.50, 0.56)
FRAME = (0.26, 0.28, 0.31)


def build(operator: bool = True) -> bt.Scene:
    """The skid: tank, pump, filter housing with its transmitter, outlet
    valve, signal tower — the words, the analog input and the programs."""
    scene = bt.Scene()
    scene.add_box("skid/base", size=(2.0, 0.9, 0.06), position=(0.0, 0.0, 0.03), color=FRAME)
    scene.add_box("skid/tank", size=(0.50, 0.50, 0.70), position=(-0.65, 0.05, 0.41), color=STEEL)
    scene.add_box("skid/tank_lid", size=(0.54, 0.54, 0.03), position=(-0.65, 0.05, 0.775), color=FRAME)
    # The pump: a volute and its motor, with a run indicator on the motor.
    scene.add_box("skid/pump_foot", size=(0.36, 0.20, 0.06), position=(-0.12, -0.05, 0.09), color=FRAME)
    quarter_y = (0.0, 0.7071067811865476, 0.0, 0.7071067811865476)
    scene.add_cylinder("skid/pump_motor", radius=0.075, length=0.22, position=(-0.04, -0.05, 0.20),
                       quaternion=quarter_y, color=(0.18, 0.36, 0.60))
    scene.add_cylinder("skid/pump_volute", radius=0.09, length=0.07, position=(-0.21, -0.05, 0.20),
                       quaternion=quarter_y, color=PIPE)
    scene.add_box("pump/lamp", size=(0.05, 0.05, 0.02), position=(-0.04, -0.05, 0.285), color=(0.20, 0.24, 0.20))
    # The filter housing and the transmitter across it.
    scene.add_cylinder("skid/housing", radius=0.11, length=0.62, position=(0.35, 0.0, 0.40), color=STEEL)
    scene.add_cylinder("skid/housing_cap", radius=0.125, length=0.04, position=(0.35, 0.0, 0.73), color=FRAME)
    scene.add_box("dpt/body", size=(0.09, 0.07, 0.10), position=(0.35, -0.19, 0.50), color=(0.16, 0.42, 0.70))
    for k, z in enumerate((0.30, 0.62)):
        scene.add_box(f"dpt/tap{k}", size=(0.015, 0.09, 0.015), position=(0.35, -0.13, z), color=PIPE)
    scene.add_box("dpt/riser", size=(0.015, 0.015, 0.34), position=(0.35, -0.17, 0.46), color=PIPE)
    # Piping: tank → pump → housing → valve → outlet.
    for name, size, at in (
        ("suction", (0.26, 0.04, 0.04), (-0.32, -0.05, 0.20)),
        ("discharge", (0.04, 0.04, 0.16), (-0.21, -0.05, 0.31)),
        ("to_housing", (0.48, 0.04, 0.04), (0.03, -0.05, 0.38)),
        ("outlet", (0.50, 0.04, 0.04), (0.70, 0.0, 0.62)),
        ("spout", (0.04, 0.04, 0.30), (0.95, 0.0, 0.48)),
    ):
        scene.add_box(f"pipe/{name}", size=size, position=at, color=PIPE)
    scene.add_box("skid/valve_body", size=(0.10, 0.08, 0.08), position=(0.70, 0.0, 0.62), color=(0.55, 0.22, 0.20))
    # The outlet valve's actuator: a spring-return cylinder — energised it
    # opens (400 ms), dropped it closes. Its switches are the open / closed
    # confirmations.
    bt.parts.cylinder(scene, "valve", (0.70, 0.0, 0.74), axis=(0.0, 0.0, 1.0), stroke=0.06, time=0.4,
                      solenoid="single", bore=0.04, pad=(0.05, 0.05, 0.012))

    for name in ("pb_start", "pb_stop", "pb_reset", "new_element", "run", "lamp_run", "pump", "warn", "alarm",
                 "alarm_dp", "alarm_valve", "alarm_wire"):
        scene.define_signal(name)
    scene.add_lamp("pump/run", signal="pump", obstacle="pump/lamp", color=(0.1, 1.0, 0.2))
    bt.parts.tower_lamp(scene, "tower", (0.95, 0.35, 0.06),
                        tiers={"green": "lamp_run", "yellow": "warn", "red": "alarm"}, pole=0.85)

    # The plant's numbers: how clogged the element is and the pressure
    # across it — and the transmitter that measures it.
    scene.define_word("clog", CLOG_NEW)
    scene.define_word("dp_true", 0.0)
    scene.add_analog("dp", source="dp_true", range=(0.0, 100.0), unit="kPa", raw=(0, 4000), place="dpt/body")
    # The controller's numbers: setpoints, a millisecond count, a peak.
    scene.define_word("dp_warn", DP_WARN)
    scene.define_word("dp_limit", DP_LIMIT)
    scene.define_word("valve_time", VALVE_TIME_MS, integer=True)
    scene.define_word("t_valve", 0, integer=True)
    scene.define_word("dp_peak", 0.0)

    author(scene)
    if operator:
        script(scene)
    scene.add_io_node(
        "PLC1", kind="plc", programs=PLC_PROGRAMS,
        channels=bt.io.di8(base="%IX100.0") + bt.io.do8(base="%QX100.0") + bt.io.ai(2, base="%IW100")
        + bt.io.channels("word", 1, prefix="IW", base="%IW102")
        + bt.io.channels("word", 1, prefix="SP", base="%IW103", unit="0.1"),
    )
    # The watchdog's setpoint comes from a panel: an input the plant holds,
    # so a hand can change it while a real PLC runs the program. The
    # warning setpoint is keyed in at the desk (the operator's program
    # writes it), a tenth of a kPa to the count.
    scene.declare_io("valve_time", role="input")
    scene.bind_input("dp", "PLC1", "AI0", tag="DPT1", field="PDT-101")
    scene.bind_input("valve_time", "PLC1", "IW0", tag="ValveTime")
    scene.bind_input("dp_warn", "PLC1", "SP0", tag="DpWarn")
    if operator:
        scene.add_io_node("DESK", kind="plc", programs=["operator"],
                          channels=bt.io.di8(base="%IX0.0") + bt.io.do8(base="%QX0.0")
                          + bt.io.channels("word", 1, prefix="QW", base="%QW0", unit="0.1"))
        scene.bind_output("dp_warn", "DESK", "QW0")
    scene.auto_assign_io()
    scene.add_camera("overview", position=(1.9, -2.3, 1.7), look_at=(0.1, 0.0, 0.45), fov=45.0,
                     resolution=(1280, 720))
    return scene


def author(scene: bt.Scene) -> None:
    opened = signal("valve/extended")
    flowing = all_of(signal("pump"), opened)

    # --- the PLC: rungs alone, top to bottom every 10 ms -------------------
    lad = scene.sequence("filter")
    lad.rung(all_of(any_of(signal("pb_start"), signal("run")), not_(signal("pb_stop")), not_(signal("alarm"))),
             comment="run (self-hold)").coil("run")
    lad.rung(signal("run"), comment="outlet valve").coil("valve/extend")
    lad.rung(all_of(signal("run"), opened), comment="pump: behind the valve's open switch").coil("pump")
    # (Gated by `run`: before a polled controller has read its inputs once
    # they are all zero — a setpoint of 0 would warn at power-up.)
    lad.rung(all_of(signal("run"), word("dp") >= word("dp_warn")), comment="change the element soon").coil("warn")
    lad.rung(word("dp") >= word("dp_limit"), comment="element clogged").set("alarm_dp")
    lad.rung(word("dp.raw") < -100, comment="transmitter wire broken").set("alarm_wire")
    stroking = all_of(signal("run"), not_(opened))
    lad.rung(stroking, comment="valve watchdog: 10 ms a scan").move("t_valve", word("t_valve") + 10)
    lad.rung(not_(stroking), comment="valve watchdog cleared").move("t_valve", 0)
    lad.rung(all_of(stroking, word("t_valve") >= word("valve_time")),
             comment="valve did not open in time").set("alarm_valve")
    lad.rung(any_of(signal("alarm_dp"), signal("alarm_valve"), signal("alarm_wire")), comment="alarm").coil("alarm")
    lad.rung(signal("pb_reset"), comment="alarm reset").reset("alarm_dp").reset("alarm_valve").reset("alarm_wire")
    lad.rung(word("dp") > word("dp_peak"), comment="peak hold").move("dp_peak", word("dp"))
    lad.rung(signal("run"), comment="running lamp").coil("lamp_run")

    # --- the plant: the element clogs while it filters ---------------------
    proc = scene.sequence("process").plant()
    proc.rung(flowing, comment="the element clogs while liquid flows").move("clog", word("clog") + CLOG_PER_SCAN)
    proc.rung(flowing, comment="pressure across the element").move("dp_true", word("clog"))
    proc.rung(not_(flowing), comment="no flow, no pressure").move("dp_true", 0)
    proc.rung(rising("new_element"), comment="a fresh element").move("clog", CLOG_NEW)


def press(sq, name: str, button: str, hold: float = 0.2) -> None:
    sq.step(f"{name}: press", actions=[bt.seq.set_signal(button, True)], transition=bt.seq.elapsed(hold))
    sq.step(f"{name}: release", actions=[bt.seq.set_signal(button, False)])


def script(scene: bt.Scene) -> None:
    """The operator's script: what makes the bake a story, and ends it."""
    op = scene.sequence("operator")
    op.step("idle", transition=bt.seq.elapsed(0.5))
    press(op, "start", "pb_start")
    op.step("filtering until it clogs", transition=signal("alarm"))
    op.step("stopped on the alarm", transition=bt.seq.elapsed(1.2))
    press(op, "change the element", "new_element")
    press(op, "reset", "pb_reset")
    op.step("a lower warning keyed in", actions=[bt.seq.set_word("dp_warn", DP_WARN_RETUNED)],
            transition=bt.seq.elapsed(0.4))
    press(op, "start again", "pb_start")
    op.step("filtering until the warning", transition=signal("warn"))
    op.step("warned", transition=bt.seq.elapsed(1.0))
    press(op, "stop", "pb_stop")
    op.step("the valve closes", transition=all_of(signal("valve/retracted"), bt.seq.elapsed(0.6)))


def bake(scene: bt.Scene):
    return scene.simulate_sequences(PROGRAMS)


def loopback(scene: bt.Scene):
    with bt.vc.loopback(scene, PROGRAMS, host="PLC1", studio=False) as s:
        s.run()
        return s.timeline()


def same(a, b) -> bool:
    return a.signals == b.signals and a.words == b.words and a.step_spans == b.step_spans


def edges(tl, name: str) -> list:
    return [(round(t, 2), v) for t, v in dict(tl.signals)[name]]


def summary(tl) -> dict:
    pump = tl.signal("pump").rising_edges()
    warn = tl.signal("warn").rising_edges()
    return {
        "duration_s": round(tl.duration, 2),
        "run": edges(tl, "lamp_run"),
        "warn": edges(tl, "warn"),
        "alarm": edges(tl, "alarm"),
        # From the pump starting to the warning, each run: the retuned
        # setpoint brings it forward.
        "pump_to_warn_s": [round(w - p, 2) for p, w in zip(pump, warn)],
    }


def live_checks(scene: bt.Scene) -> dict:
    """A hand on the numbers while the unit runs (loopback through PLC1)."""
    out = {}
    # 1. A watchdog shorter than the valve's stroke: a false valve alarm.
    with bt.vc.loopback(scene, PROGRAMS, host="PLC1", studio=False) as s:
        s.write("valve_time", 200)
        s.run(for_=2.0)
        out["valve_time=200"] = {"alarm_valve": s.signal("alarm_valve"), "pump": s.signal("pump"),
                                 "t_valve_ms": s.word("t_valve")}
    # 2. The transmitter's wire breaks while filtering.
    with bt.vc.loopback(scene, PROGRAMS, host="PLC1", studio=False) as s:
        s.run(for_=3.0)
        before = s.analog()[0]
        s.inject(bt.io.open("dp"))
        s.run(for_=0.2)
        after = s.analog()[0]
        out["wire break"] = {"before": {k: before[k] for k in ("current", "raw", "scaled")},
                             "after": {k: after[k] for k in ("current", "raw", "scaled", "open")},
                             "alarm_wire": s.signal("alarm_wire"), "run": s.signal("run")}
    # 3. The pressure forced to 90 kPa at the transmitter.
    with bt.vc.loopback(scene, PROGRAMS, host="PLC1", studio=False) as s:
        s.run(for_=3.0)
        s.force("dp", 90.0)
        s.run(for_=0.2)
        out["dp forced to 90"] = {"raw": s.word("dp.raw"), "alarm_dp": s.signal("alarm_dp"),
                                  "dp_peak": s.word("dp_peak")}
    return out


def run_openplc(scene: bt.Scene, ref, args, out: Path) -> int:
    from _openplc import OpenPlc, host_ip_for_the_plc

    xc = bt.vc.exchange(scene, "PLC1", PROGRAMS)
    st = scene.st("PLC1", PROGRAMS, cycle=False)
    (out / "filter_PLC1.st").write_text(st, encoding="utf-8")
    xc.save(out / "filter_exchange_PLC1.md")
    plc_side_ip = host_ip_for_the_plc()
    bind = "127.0.0.1" if plc_side_ip == "127.0.0.1" else "0.0.0.0"
    link = bt.vc.modbus_server(xc, f"{bind}:{PLC_PORT}", iec_base=100, allow_remote=bind != "127.0.0.1")
    sizes = {t: max((r.address + r.count for r in link.rows if r.table == t), default=0) for t in
             ("discrete_input", "coil", "input_register", "holding_register")}
    link.start()
    plc = OpenPlc(args.openplc_url)
    plc.wait_ready()
    plc.stop()
    plc.settings(args.polling_ms)
    plc.clear_slave_devices()
    plc.add_slave_device("botrail", plc_side_ip, PLC_PORT, di=sizes["discrete_input"], coils=sizes["coil"],
                         ir=sizes["input_register"], hr=sizes["holding_register"])
    log = plc.load_program("botrail_filter", st)
    (out / "openplc_compile.log").write_text(log, encoding="utf-8")
    print("OpenPLC compiled the program (INT / REAL words, the analog scaling, the comparisons)")
    with bt.vc.Session(scene, PROGRAMS, links={"PLC1": link}, pace="realtime",
                       max_duration=ref.duration + 20.0, studio=args.studio) as s:
        plc.start()
        s.run(for_=1.5)                      # the PLC polls before the operator starts
        s.run()                              # … until the operator's script ends
        tl = s.timeline()
        stats = s.stats()
        # Then a hand on the panel while the real PLC runs: a watchdog
        # shorter than the valve's stroke, and a start.
        s.write("valve_time", 200)
        s.force("pb_start", True)
        s.run(for_=1.5)
        false_alarm = s.signal("alarm")
        plc.stop()
    got, want = summary(tl), summary(ref)
    print(json.dumps({"openplc": got, "bake": want, "polls": link.polls, "overruns": stats["overruns"],
                      "valve_time=200 trips the alarm": false_alarm}, default=str))
    tl.export_usd(out / "filter_emulated.usdc")

    def close(a, b) -> bool:
        return len(a) == len(b) and all(va == vb and abs(ta - tb) <= args.tolerance for (ta, va), (tb, vb) in zip(a, b))

    ok = (link.polls > 0 and all(close(got[k], want[k]) for k in ("run", "warn", "alarm"))
          and len(got["pump_to_warn_s"]) == len(want["pump_to_warn_s"])
          and all(abs(a - b) <= args.tolerance for a, b in zip(got["pump_to_warn_s"], want["pump_to_warn_s"]))
          and false_alarm)
    print("the PLC ran the unit as the bake did, and took the new setpoint" if ok
          else "the PLC's run differs from the bake")
    return 0 if (ok or not args.check) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("usd", nargs="?", default=None, help="write the baked story as USD")
    parser.add_argument("--studio", action="store_true", help="open the studio on the baked unit")
    parser.add_argument("--openplc", action="store_true", help="run the program on OpenPLC and compare with the bake")
    parser.add_argument("--openplc-url", default=OPENPLC)
    parser.add_argument("--polling-ms", type=int, default=20)
    parser.add_argument("--tolerance", type=float, default=0.5,
                        help="how far the PLC's edges may sit from the bake's, seconds (polled hops add up)")
    parser.add_argument("--out", default=str(HERE / "out"))
    parser.add_argument("--check", action="store_true", help="exit 1 unless the judgement holds")
    args = parser.parse_args()

    scene = build()
    tl = bake(scene)
    print(json.dumps(summary(tl), default=str))
    # The loop at the moment of the first warning: measured → mA → counts → scaled.
    first_warn = tl.signal("warn").rising_edges()[0]
    print(json.dumps({"loop at the first warning": tl.analog(first_warn)}, default=str))
    looped = loopback(scene)
    identical = same(tl, looped)
    print(f"loopback through PLC1's exchange equals the bake: {identical}")
    live = live_checks(scene)
    print(json.dumps({"live": live}, default=str))
    retuned = summary(tl)["pump_to_warn_s"]
    judged = (identical and len(retuned) == 2 and retuned[1] < retuned[0] - 2.0
              and live["valve_time=200"]["alarm_valve"] and not live["valve_time=200"]["pump"]
              and live["wire break"]["alarm_wire"] and live["wire break"]["after"]["raw"] == -1000
              and live["dp forced to 90"]["alarm_dp"] and live["dp forced to 90"]["raw"] == 3600)
    print("the numbers drive the unit: setpoints, loop and live changes hold" if judged
          else "the judgement does not hold")
    if args.usd:
        tl.export_usd(args.usd)
        print(f"wrote {args.usd}")
    code = 0 if (judged or not args.check) else 1
    if args.openplc:
        out = Path(args.out)
        out.mkdir(parents=True, exist_ok=True)
        code = max(code, run_openplc(scene, tl, args, out))
    elif args.studio:
        scene.show_timeline(tl)
        bt.studio(scene)
    return code


if __name__ == "__main__":
    sys.exit(main())
