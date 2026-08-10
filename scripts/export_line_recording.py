#!/usr/bin/env python
"""Bakes the weld line and exports one steady-state takt as a USD recording.

The distributable artifact for the line: a full run is mostly repetition,
so a single takt from the pipelined middle carries the whole story — three
bodies in flight, every station welding, the transfer indexing — at a
quarter of the bytes.

    $ python scripts/export_line_recording.py out/line_takt.usdc [--fps 24]
                                              [--stations 4] [--full]

Play it back into a rebuilt cell with `examples/play_record.py`.
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples"))

import weld_line_demo as line  # noqa: E402


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    out = Path(args[0]) if args else Path("line_takt.usdc")
    fps = 24.0
    if "--fps" in sys.argv:
        fps = float(sys.argv[sys.argv.index("--fps") + 1])
    if "--stations" in sys.argv:
        line.set_stations(int(sys.argv[sys.argv.index("--stations") + 1]))

    scene, ln, riders = line.build_line()
    poses = line.teach(scene, ln, riders)
    programs = [line.build_station_program(scene, st, poses)
                for st in line.STATIONS]
    programs.append(line.build_transfer_program(scene, riders))
    timeline = scene.simulate_sequences(programs, max_duration=400.0)

    at = {s: (a, b) for s, a, b in timeline.step_spans}
    if "--full" in sys.argv:
        start, end = 0.0, timeline.duration
        label = "full run"
    else:
        # The pipeline is fullest between the last two pitches: every
        # station holds a body, so the slice is a complete takt of the
        # line at steady state rather than of its ramp-up.
        start = at[f"transfer/p{line.PITCHES - 2}_landed"][1]
        end = at[f"transfer/p{line.PITCHES - 1}_landed"][1]
        label = "one steady-state takt"

    out.parent.mkdir(parents=True, exist_ok=True)
    warnings = timeline.export_usd(out, fps=fps, start=start, end=end)
    for w in warnings:
        print(f"warning: {w}")
    size = out.stat().st_size / 1e6
    print(
        f"{label}: [{start:.2f}, {end:.2f}]s of {timeline.duration:.2f}s "
        f"at {fps:.0f} fps -> {out} ({size:.1f} MB)"
    )
    print(f"play with: python examples/play_record.py {out}")


if __name__ == "__main__":
    main()
