"""A transfer line run by a ladder: rungs, a gated step chain, timers and a
counter (design-plc-resolution.md PF1).

A belt carries a work from the infeed past two stations to the discharge.
The control is written the way a PLC program is laid out — three programs
on `PLC1`, scanned in this order every 10 ms:

* `common` — rungs: the self-holding run relay, the cycle-stop request,
  the step-mode advance;
* `auto` — the step chain (feed in → station 1 → dwell → station 2 → dwell →
  discharge), cycling, gated by `advance`, cleared by the reset button. Its
  steps fire no action: they are states;
* `outputs` — rungs: the belt's and the feeder's coils from the step relays
  (and the manual jog), the dwell timers, the lamps, the counter of works
  made.

The operator is a program too (`operator`, off the PLC): it presses the
relay-buttons on a script — start, a cycle stop, a jog, step mode, a reset
— so the bake is one deterministic story, and it is what says when the
bake ends (rungs and a cycling chain never do).

    python run_demo.py [out.usd] [--studio]
    python run_demo.py --openplc [--check]     # the same program as ST on OpenPLC (docker compose up -d)

Without `--openplc` it bakes the line, checks that the loopback through
`PLC1`'s exchange is the bake bit for bit, and writes the USD. With it,
the Structured Text botrail writes for `PLC1` — the three programs, rungs
included — is compiled and run by OpenPLC, which drives the plant over
Modbus while the operator script presses the same buttons; the run is
diffed against the bake.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import botrail as bt  # noqa: E402

PLC_PROGRAMS = ["common", "auto", "outputs"]
PROGRAMS = PLC_PROGRAMS + ["operator"]

BELT_Z = 0.80
SPEED = 0.25
X_IN, X_ST1, X_ST2, X_OUT = -0.60, -0.20, 0.20, 0.62
DWELL_S = 0.8
WORKS = 3
OPENPLC = "http://127.0.0.1:8080"
PLC_PORT = 5020


def build(operator: bool = True) -> bt.Scene:
    """The line: a belt, three works in a magazine, a beam at each station
    and at the discharge, a panel and a signal tower — and its programs."""
    scene = bt.Scene()
    scene.add_box("line/bed", size=(1.5, 0.24, 0.06), position=(0.0, 0.0, BELT_Z - 0.03), color=(0.34, 0.36, 0.40))
    for k, x in enumerate((-0.65, 0.65)):
        scene.add_box(f"line/leg{k}", size=(0.05, 0.2, BELT_Z - 0.06), position=(x, 0.0, (BELT_Z - 0.06) / 2),
                      color=(0.25, 0.26, 0.28))
    for name, x in (("st1", X_ST1), ("st2", X_ST2)):
        scene.add_box(f"line/{name}_gantry", size=(0.04, 0.34, 0.03), position=(x, 0.0, BELT_Z + 0.22),
                      color=(0.80, 0.66, 0.20))
        for side in (-1, 1):
            scene.add_box(f"line/{name}_post{side}", size=(0.03, 0.03, 0.22),
                          position=(x, 0.155 * side, BELT_Z + 0.11), color=(0.80, 0.66, 0.20))
    scene.add_box("line/bin", size=(0.22, 0.30, 0.10), position=(0.88, 0.0, BELT_Z - 0.20), color=(0.55, 0.22, 0.20))
    works = [f"work{k + 1}" for k in range(WORKS)]
    for k, name in enumerate(works):
        scene.add_box(name, size=(0.08, 0.08, 0.06), position=(-0.95, -0.12 + 0.12 * k, BELT_Z - 0.30),
                      color=(0.86, 0.74, 0.18))
    scene.add_conveyor("conv", zone_position=(0.0, 0.0, BELT_Z + 0.05), zone_size=(1.5, 0.24, 0.14),
                       velocity=(SPEED, 0.0, 0.0), running=False)
    # An indexing feeder: one work onto the infeed per start; the works go
    # back to the magazine from the bin end.
    scene.add_source("feeder", works, park=(-0.95, -0.12, BELT_Z - 0.30), pitch=(0.0, 0.12, 0.0),
                     position=(X_IN, 0.0, BELT_Z + 0.03), interval=0.0)
    scene.add_sink("bin", zone_position=(0.80, 0.0, BELT_Z + 0.05), zone_size=(0.10, 0.30, 0.30), source="feeder")
    for name, x in (("s_in", X_IN), ("s1", X_ST1), ("s2", X_ST2), ("s_out", X_OUT)):
        scene.add_beam_sensor(name, (x, -0.14, BELT_Z + 0.03), (x, 0.14, BELT_Z + 0.03), watch=works)

    # The operator's buttons are relays (a script — or a hand in the studio
    # — writes them); the panel's lamps and the tower show the outputs.
    for name in ("pb_start", "pb_stop", "pb_reset", "pb_jog", "pb_step", "sw_step",
                 "run", "stop_req", "stop_done", "advance", "lamp_run", "lamp_lot"):
        scene.define_signal(name)
    bt.parts.tower_lamp(scene, "tower", (0.72, 0.30, 0.0), tiers={"green": "lamp_run", "yellow": "lamp_lot"}, pole=0.95)

    author(scene)
    if operator:
        script(scene)
    scene.add_io_node(
        "PLC1", kind="plc", programs=PLC_PROGRAMS, monitor=True,
        channels=bt.io.di16(base="%IX100.0") + bt.io.do8(base="%QX100.0") + bt.io.word(count=2, base="%QW100"),
    )
    # The operator's desk: the buttons' side of the wires to the PLC.
    if operator:
        scene.add_io_node("DESK", kind="plc", programs=["operator"],
                          channels=bt.io.di8(base="%IX0.0") + bt.io.do8(base="%QX0.0"))
    scene.declare_io("feeder", role="output")  # the feeder's run coil
    scene.auto_assign_io()
    scene.add_camera("overview", position=(1.5, -1.9, 1.9), look_at=(0.0, 0.0, 0.8), fov=50.0, resolution=(1280, 720))
    return scene


def author(scene: bt.Scene) -> None:
    sig, both, either, neg = bt.seq.signal, bt.seq.all_of, bt.seq.any_of, bt.seq.not_
    at = lambda step: bt.seq.step_active("auto", step)  # noqa: E731

    # --- common: run / stop / step mode (scanned first) -------------------
    lad = scene.sequence("common")
    # Rung order is scan order: the stop is taken before the run relay is
    # written, and the request stands until the relay has dropped — in the
    # other order the step chain slips into the next cycle for one scan.
    lad.rung(both(sig("stop_req"), at("wait")), comment="cycle stop: at the end of the cycle").coil("stop_done")
    lad.rung(both(either(sig("pb_start"), sig("run")), neg(sig("stop_done"))), comment="run (self-hold)").coil("run")
    lad.rung(both(sig("pb_stop"), sig("run")), comment="cycle stop asked").set("stop_req")
    lad.rung(neg(sig("run")), comment="stop taken").reset("stop_req")
    lad.rung(both(sig("run"), either(neg(sig("sw_step")), bt.seq.rising("pb_step"))),
             comment="advance (step mode: one step per press)").coil("advance")

    # --- auto: the step chain, gated ---------------------------------------
    sq = scene.sequence("auto")
    sq.step("wait", transition=both(sig("run"), neg(sig("stop_req"))))
    sq.step("feed in", transition=sig("s_in"))   # the feeder's coil is on until the work is seen
    sq.step("to st1", transition=sig("s1"))
    sq.step("dwell 1", transition=sig("t_dwell1"))
    sq.step("to st2", transition=sig("s2"))
    sq.step("dwell 2", transition=sig("t_dwell2"))
    sq.step("discharge", transition=sig("s_out"))
    sq.step("clear", transition=neg(sig("s_out")))
    sq.gates(enable=sig("advance"), reset=sig("pb_reset"), cycle=True)

    # --- outputs: coils from the step relays (scanned last) ----------------
    out = scene.sequence("outputs")
    out.rung(
        either(
            both(sig("run"), either(both(at("to st1"), neg(sig("s1"))), both(at("to st2"), neg(sig("s2"))),
                                    at("discharge"), at("clear"))),
            both(neg(sig("run")), sig("pb_jog")),
        ),
        comment="belt: the travel steps, or the manual jog",
    ).coil("conv")
    out.rung(at("feed in"), comment="feeder: one work per cycle").coil("feeder")
    out.rung(at("dwell 1"), comment="station 1 dwell").ton("t_dwell1", DWELL_S)
    out.rung(at("dwell 2"), comment="station 2 dwell").ton("t_dwell2", DWELL_S)
    out.rung(sig("run"), comment="running lamp").coil("lamp_run")
    out.rung(at("clear"), comment="works made").ctu("made", 2)
    out.rung(sig("made"), comment="lot of two made").coil("lamp_lot")
    out.rung(sig("pb_reset"), comment="lot counter cleared with the steps").reset("made")


def press(sq, name: str, button: str, hold: float = 0.2) -> None:
    sq.step(f"{name}: press", actions=[bt.seq.set_signal(button, True)], transition=bt.seq.elapsed(hold))
    sq.step(f"{name}: release", actions=[bt.seq.set_signal(button, False)])


def script(scene: bt.Scene) -> None:
    """The operator's script: what makes the bake a story, and ends it."""
    op = scene.sequence("operator")
    op.step("idle", transition=bt.seq.elapsed(0.5))
    press(op, "start", "pb_start")
    op.step("two cycles on the way", transition=bt.seq.elapsed(9.0))
    press(op, "cycle stop", "pb_stop")           # mid second cycle: it finishes, then the line stops
    op.step("the cycle ends", transition=bt.seq.all_of(bt.seq.signal("run", False), bt.seq.elapsed(1.0)))
    press(op, "jog", "pb_jog", hold=0.6)         # stopped: the belt jogs by hand
    op.step("step mode", actions=[bt.seq.set_signal("sw_step", True)], transition=bt.seq.elapsed(0.3))
    press(op, "start again", "pb_start")
    for k in range(4):                           # wait → feed in → to st1 → (the work travels) → dwell 1
        op.step(f"step {k + 1}: wait", transition=bt.seq.elapsed(2.0 if k == 3 else 0.4))
        press(op, f"step {k + 1}", "pb_step")
    op.step("in the dwell", transition=bt.seq.elapsed(0.5))
    press(op, "reset", "pb_reset")               # 工程クリア: back to `wait`
    op.step("done", actions=[bt.seq.set_signal("sw_step", False)], transition=bt.seq.elapsed(0.5))


def bake(scene: bt.Scene):
    return scene.simulate_sequences(PROGRAMS)


def loopback(scene: bt.Scene):
    with bt.vc.loopback(scene, PROGRAMS, host="PLC1", studio=False) as s:
        s.run()
        return s.timeline()


def same(a, b) -> bool:
    return a.signals == b.signals and a.words == b.words and a.step_spans == b.step_spans and all(
        a.object_pose(w, t) == b.object_pose(w, t) for w in ("work1", "work2", "work3")
        for t in (a.duration * k / 40 for k in range(41)))


def summary(tl) -> dict:
    chain = [name.split("/")[-1] for name, _, _ in tl.step_spans if name.startswith("auto/")]
    return {
        "duration_s": round(tl.duration, 2),
        "cycles_started": chain.count("feed in"),
        "works_discharged": len(tl.signal("s_out").rising_edges()),
        "belt_starts": len(tl.signal("conv").rising_edges()),
        "run": [(round(t, 2), v) for t, v in dict(tl.signals)["run"]],
        "lot_lamp": [(round(t, 2), v) for t, v in dict(tl.signals)["lamp_lot"]],
    }


def run_openplc(scene: bt.Scene, ref, args, out: Path) -> int:
    from _openplc import OpenPlc, host_ip_for_the_plc

    xc = bt.vc.exchange(scene, "PLC1", PROGRAMS)
    st = scene.st("PLC1", PROGRAMS, cycle=False)
    (out / "ladder_PLC1.st").write_text(st, encoding="utf-8")
    xc.save(out / "ladder_exchange_PLC1.md")
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
    log = plc.load_program("botrail_ladder", st)
    (out / "openplc_compile.log").write_text(log, encoding="utf-8")
    print("OpenPLC compiled the program (rungs, gates and all)")
    with bt.vc.Session(scene, PROGRAMS, links={"PLC1": link}, pace="realtime",
                       max_duration=ref.duration + 20.0, studio=args.studio) as s:
        plc.start()
        s.run(for_=1.5)                      # the PLC polls before the operator starts
        s.run()                              # … until the operator's script ends
        plc.stop()
        tl = s.timeline()
        stats = s.stats()
    got, want = summary(tl), summary(ref)
    print(json.dumps({"openplc": got, "bake": want, "polls": link.polls, "overruns": stats["overruns"]},
                     default=str))
    tl.export_usd(out / "ladder_emulated.usdc")
    # The judgement: the same cycles, works and belt starts, and the run
    # relay and the lot lamp (the PLC's counter reaching its preset — the
    # count itself lives in the PLC) switching at the bake's moments,
    # within the controller's latency.
    def close(a, b) -> bool:
        return len(a) == len(b) and all(va == vb and abs(ta - tb) <= args.tolerance for (ta, va), (tb, vb) in zip(a, b))

    ok = (link.polls > 0 and got["cycles_started"] == want["cycles_started"]
          and got["works_discharged"] == want["works_discharged"]
          and got["belt_starts"] == want["belt_starts"]
          and close(got["run"], want["run"]) and close(got["lot_lamp"], want["lot_lamp"]))
    print("the PLC ran the line as the bake did" if ok else "the PLC's run differs from the bake")
    return 0 if (ok or not args.check) else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("usd", nargs="?", default=None, help="write the baked cycle as USD")
    parser.add_argument("--studio", action="store_true", help="open the studio on the baked line")
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
    print(json.dumps({**summary(tl), "made_count": tl.word("made.count").samples}, default=str))
    looped = loopback(scene)
    identical = same(tl, looped)
    print(f"loopback through PLC1's exchange equals the bake: {identical}")
    if args.usd:
        tl.export_usd(args.usd)
        print(f"wrote {args.usd}")
    code = 0 if (identical or not args.check) else 1
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
