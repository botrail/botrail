"""The pick cell with its robot program running on a Universal Robots
controller — URSim in Docker — and botrail as the plant around it
(design-virtual-commissioning.md VC5: the robot controller in the loop).

In the OpenPLC demo the controller under test is the cell's PLC and botrail
moves the arm. Here the controller is the **robot's own**: botrail exports
the pick program as URScript whose I/O lives in the controller's bit
registers (`rtde=True`), sends it to URSim, and runs the cell as the plant
of the `UR` host over RTDE — the program's coils (belt run, vacuum) come
back from the output registers, the sensors it reads (the beam, the gauge)
go into the input registers, and the controller's `actual_q` streams into
the plant, which mirrors the arm as the controller moves it and keeps
watching collisions, clearance, the belt, the part and the grasp. The
run lands in the same timeline as the bake and is diffed against it; the
arm's timing is the controller's, so the report also says how far the
real controller's cycle is from botrail's bake, move by move.

    docker compose up -d
    python run_demo.py [--seconds 60] [--out out/] [--check]
    python run_demo.py --studio          # watch the mirrored arm live

`--check` exits 1 unless the program ran to its end on the controller,
every signal of the run matched the bake within `--tolerance` (default 3 s
— the controller's own motion timing, not a polling latency: the number
the report prints) and the part went down within 10 mm of where the bake
set it. The cell's poses keep the UR's `wrist_2` a quarter turn off zero:
its straight-line moves are the controller's own `movel`, which flips the
wrist and stops at that singularity. PolyScope is at http://127.0.0.1:6080
(noVNC) while the container runs.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "export"))

URSIM = "127.0.0.1"
#: How far the controller's run may set the part down from the bake's (mm).
SET_DOWN_TOLERANCE_MM = 10.0
ROBOT = "simple_arm"
PROGRAM = "pick"


def build() -> bt.Scene:
    """The export example's pick cell, its grasp the vacuum coil's, with the
    UR controller's bit registers as the robot program's I/O: the beam and
    the gauge on input registers 64 / 65, the belt and the vacuum on output
    registers 64 / 65."""
    import export_urscript as pick

    scene = pick.build_cell()
    pick.author_sequence(scene, coil_grasp=True)
    scene.add_io_node("UR", kind="robot_controller", robots=[ROBOT], channels=bt.io.ur_registers())
    scene.bind_input("part_at_pick", "UR", "IR64", field="BEAM1")
    scene.bind_input("spec_ok", "UR", "IR65", field="GAUGE")
    scene.bind_output("conv", "UR", "OR64", field="VFD1")
    scene.bind_output("vacuum", "UR", "OR65", field="YV1")
    return scene


def reset_script(xc) -> str:
    """A one-shot program that clears the output registers the pick program
    writes — they persist on the controller across programs."""
    lines = ["def botrail_reset():"]
    for p in xc.inputs:
        lines.append(f"  write_output_boolean_register({p.port}, False)")
    lines.append("end")
    return "\n".join(lines) + "\n"


def arrivals(ref, tl, robot: str, tol: float = 0.02) -> list[dict]:
    """When the controller's arm reached each goal the bake's moves end at:
    the bake's end time, the run's arrival time, their difference."""
    rows = []
    t_from = 0.0
    for name, start, end in ref.moves(robot):
        goal = ref.sample(end, robot)
        arrived = None
        t = t_from
        while t <= tl.duration + 1e-9:
            q = tl.sample(t, robot)
            if max(abs(a - b) for a, b in zip(q, goal)) < tol:
                arrived = t
                break
            t += tl_step(tl)
        rows.append({"move": name, "bake_end_s": round(end, 3),
                     "controller_arrival_s": None if arrived is None else round(arrived, 3),
                     "offset_s": None if arrived is None else round(arrived - end, 3)})
        if arrived is not None:
            t_from = arrived
    return rows


def tl_step(tl) -> float:
    return 0.01


def report_markdown(ref, tl, rows: list[dict], diff, stats: dict, link) -> str:
    out = ["# URSim pick cell — the robot controller in the loop", ""]
    out.append(f"Bake {ref.duration:.2f} s; controller run {tl.duration:.2f} s "
               f"({tl.duration - ref.duration:+.2f} s). RTDE {link.packages} packages, {link.exchanges} scans, "
               f"lag max {stats['lag_max_s'] * 1e3:.1f} ms, overruns {stats['overruns']}.")
    out += ["", "| move | bake end | controller arrival | offset |", "|---|---|---|---|"]
    for r in rows:
        arr = "—" if r["controller_arrival_s"] is None else f"{r['controller_arrival_s']:.2f} s"
        off = "—" if r["offset_s"] is None else f"{r['offset_s']:+.2f} s"
        out.append(f"| {r['move']} | {r['bake_end_s']:.2f} s | {arr} | {off} |")
    out += ["", diff.to_markdown()]
    return "\n".join(out) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--host", default=URSIM, help="the controller (URSim) address")
    parser.add_argument("--seconds", type=float, default=None, help="plant seconds the program may take (default: 3 × the bake + 5 s)")
    parser.add_argument("--frequency", type=float, default=125.0, help="RTDE output rate, Hz (default 125)")
    parser.add_argument("--tolerance", type=float, default=3.0, help="diff tolerance, seconds (default 3: the controller's own motion timing)")
    parser.add_argument("--out", default=str(HERE / "out"), help="where the script, exchange, recording, report and USD go")
    parser.add_argument("--studio", action="store_true", help="open the studio on the cell and stream the run there live")
    parser.add_argument("--check", action="store_true", help="exit 1 unless the program ran to its end and the diff is clean")
    args = parser.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    scene = build()
    # The sweep, so the program carries the reject arm the nominal run skips.
    runs = scene.simulate_scenarios([PROGRAM])
    ref = runs["baseline"]
    script = runs.to_script(sequence=PROGRAM, rtde=True, name="botrail_pick")
    (out / "pick_ur.script").write_text(script, encoding="utf-8")
    xc = bt.vc.exchange(scene, "UR")
    xc.save(out / "pick_exchange_UR.md")
    print(f"bake: {ref.duration:.2f} s; exchange UR: {len(xc.inputs)} inputs (the program's coils), "
          f"{len(xc.outputs)} outputs (what it reads)")

    server = None
    if args.studio:
        server = bt.studio(scene, block=False, open_browser=True)
        print(f"studio: {server.url} — the dock follows the mirrored arm once the program runs")

    dash = bt.vc.UrDashboard(args.host)
    dash.wait_ready()
    dash.prepare()
    print(f"URSim: {dash.robotmode()}")
    dash.run_script(reset_script(xc))
    time.sleep(1.0)

    seconds = args.seconds if args.seconds is not None else 3.0 * ref.duration + 5.0
    link = bt.vc.ursim(xc, args.host, robot=ROBOT, frequency=args.frequency,
                       allow_remote=args.host not in ("127.0.0.1", "localhost"))
    with bt.vc.Session(scene, [PROGRAM], links={"UR": link}, pace="realtime", max_duration=seconds + 10.0) as s:
        s.tick()  # the first scan: the plant's outputs land in the input registers
        dash.run_script(script)
        s.run(until=lambda sess: link.runtime_state == "playing", timeout=20.0)
        print("the controller runs the program")
        s.run(until=lambda sess: bool(link.idle), timeout=seconds)
        s.run(for_=0.5)
        tl = s.timeline()
        rec = s.recording()
        stats = s.stats()
    completed = link.runtime_state == "stopped"
    rec.save(out / "pick_inputs.csv")
    tl.export_usd(out / "pick_ursim.usdc")

    trace = bt.trace.from_timeline(tl)
    diff = ref.diff(trace, tolerance=args.tolerance, align_on="conv")
    rows = arrivals(ref, tl, ROBOT)
    # Where the controller's run left the part against where the bake set
    # it down: the arm is the controller's, the grasp the vacuum coil's.
    at, want = tl.object_pose("part", tl.duration)[0], ref.object_pose("part", ref.duration)[0]
    set_down_mm = 1000.0 * sum((a - b) ** 2 for a, b in zip(at, want)) ** 0.5
    (out / "pick_diff.md").write_text(diff.to_markdown(), encoding="utf-8")
    text = report_markdown(ref, tl, rows, diff, stats, link)
    (out / "ursim_report.md").write_text(text, encoding="utf-8")
    print(text)
    result = {
        "completed": completed,
        "bake_s": round(ref.duration, 3),
        "run_s": round(tl.duration, 3),
        "arrivals": rows,
        "diff_ok": diff.ok,
        "max_offset_s": round(diff.max_offset, 3),
        "part_set_down_offset_mm": round(set_down_mm, 1),
        "rtde_packages": link.packages,
        "stats": stats,
    }
    (out / "ursim_result.json").write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))
    if server is not None:
        server.stop()
    placed = set_down_mm <= SET_DOWN_TOLERANCE_MM
    return 0 if ((completed and diff.ok and placed) or not args.check) else 1


if __name__ == "__main__":
    sys.exit(main())
