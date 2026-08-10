"""Line balancing as a deterministic sweep: move a spot, read the takt.

This is the claim botrail exists to make, at line scale. A body-in-white
line's takt is set by its slowest station; which station that is depends on
how the weld schedule is split across them; and that split is *one table*
in `weld_line_demo.SEAM_SPLITS`. So the question a line owner actually asks
— "if I move two spots from station 1 to station 2, what happens to the
takt?" — is a sweep over that table, and every answer is a number, not an
opinion.

    $ python examples/line_balance_sweep.py

One row per split: how many spots each station holds, how busy its arms
are, the steady-state takt, and which station is the bottleneck.

Nothing here is a simulation *estimate*: each row is a full bake of the
real cell — the same teaching, the same collision-checked ramps, the same
`advance`-indexed transfer — so a row that says 24.62s is the same 24.62s
the regression test pins.

Run with:  python examples/line_balance_sweep.py [--csv out.csv]
"""

import sys
from pathlib import Path

import weld_line_demo as line
import weld_station_demo as ws

# Every way to cut a five-spot seam across two stations, keeping each
# station's spots contiguous (a station owns a *stretch* of seam, not a
# scatter) and its gun attitude single-valued: spots left of a station's
# centre take the -x attitude, right of it +x.
SEAM = (-1.2, -0.6, 0.0, 0.6, 1.2)
SPLITS = [1, 2, 3, 4]


def layout_for(front: int) -> dict:
    """`SEAM_SPLITS`-shaped entry putting `front` spots on station 1.

    The gun attitude is not part of the choice — it follows from where a
    spot sits relative to its station's datum (`attitude_of`). A split
    that leaves a station spots on *both* sides of its datum therefore
    costs it a wrist re-orientation mid-row, and that cost shows up in
    this table rather than being asserted."""
    return {2: (SEAM[:front], SEAM[front:])}


def bake(front: int, belt: float = 0.40) -> dict:
    """One full bake of the two-station line with this split and belt speed."""
    line.SEAM_SPLITS.update(layout_for(front))
    line.set_stations(2)
    ws.BELT_V = belt
    scene, ln, riders = line.build_line()
    poses = line.teach(scene, ln, riders)
    programs = [line.build_station_program(scene, st, poses)
                for st in line.STATIONS]
    programs.append(line.build_transfer_program(scene, riders))
    timeline = scene.simulate_sequences(programs, max_duration=400.0)

    at = {s: (a, b) for s, a, b in timeline.step_spans}
    takt = (at[f"transfer/p{line.PITCHES - 1}_landed"][1]
            - at[f"transfer/p{line.PITCHES - 2}_landed"][1])
    # Per station: the busiest of its two arms is what gates that station,
    # and its cycle is the span from sliding in to reporting done.
    stations = {}
    for st in line.STATIONS:
        arms = [f"{st}_{side}" for side in line.SIDES]
        stations[st] = {
            "spots": len(line.SPOTS[st]),
            "util": max(timeline.utilization(a) for a in arms),
            "cycle": at[f"{st}/b2_report"][0] - at[f"{st}/b2_slide_in"][0],
        }
    index = at[f"transfer/p{line.PITCHES - 1}_index"]
    return {
        "front": front,
        "belt": belt,
        "total": timeline.duration,
        "takt": takt,
        "transfer": index[1] - index[0],
        "stations": stations,
    }


def main() -> None:
    out = None
    if "--csv" in sys.argv:
        out = Path(sys.argv[sys.argv.index("--csv") + 1])

    print(f"two stations, {len(SEAM)} spots a side, "
          f"{line.BODIES} bodies, pitch {line.PITCH} m\n")
    header = f"{'split':>7}  {'st1':>12}  {'st2':>12}  {'takt':>8}  bottleneck"
    print(header)
    print("-" * len(header))
    rows = []
    for front in SPLITS:
        row = bake(front)
        st1, st2 = row["stations"]["st1"], row["stations"]["st2"]
        slowest = max(row["stations"], key=lambda st: row["stations"][st]["cycle"])
        print(
            f"{front:>2} / {len(SEAM) - front:<2}  "
            f"{st1['spots']} pt {st1['util'] * 100:3.0f}%  "
            f"{st2['spots']} pt {st2['util'] * 100:3.0f}%  "
            f"{row['takt']:7.2f}s  {slowest} "
            f"({st1['cycle']:.1f}s / {st2['cycle']:.1f}s)"
        )
        rows.append(row)

    best = min(rows, key=lambda r: r["takt"])
    worst = max(rows, key=lambda r: r["takt"])
    print(
        f"\nbest {best['front']}/{len(SEAM) - best['front']} at "
        f"{best['takt']:.2f}s, worst {worst['front']}/{len(SEAM) - worst['front']} "
        f"at {worst['takt']:.2f}s — {worst['takt'] - best['takt']:.2f}s of takt "
        f"({(worst['takt'] / best['takt'] - 1) * 100:.0f}%) rides on where the "
        "spots sit"
    )
    shift = 8 * 3600
    print(
        f"an eight-hour shift: {shift / best['takt']:.0f} bodies at the best "
        f"split, {shift / worst['takt']:.0f} at the worst"
    )

    # And the honest follow-up the first table provokes. A takt of
    # 24.62 s that contains a 13 s transfer is not a station-balancing
    # problem at all — so measure the other knob rather than assert it.
    print(
        f"\nbut at the best split the transfer alone is "
        f"{best['transfer']:.2f}s of the {best['takt']:.2f}s takt, and the "
        f"slowest station only "
        f"{max(s['cycle'] for s in best['stations'].values()):.2f}s. "
        "The belt, not the split, is the line's constraint:\n"
    )
    header2 = f"{'belt':>7}  {'transfer':>9}  {'slowest st':>11}  {'takt':>8}"
    print(header2)
    print("-" * len(header2))
    for belt in (0.40, 0.65, 0.90):
        row = bake(best["front"], belt)
        slowest = max(s["cycle"] for s in row["stations"].values())
        print(f"{belt:6.2f}m/s  {row['transfer']:8.2f}s  {slowest:10.2f}s  "
              f"{row['takt']:7.2f}s")
        rows.append(row)

    if out is not None:
        lines = ["front_spots,rear_spots,belt_mps,takt_s,total_s,transfer_s,"
                 "st1_util,st2_util,st1_cycle_s,st2_cycle_s"]
        for row in rows:
            st1, st2 = row["stations"]["st1"], row["stations"]["st2"]
            lines.append(
                f"{row['front']},{len(SEAM) - row['front']},{row['belt']:.2f},"
                f"{row['takt']:.2f},{row['total']:.2f},{row['transfer']:.2f},"
                f"{st1['util']:.4f},{st2['util']:.4f},"
                f"{st1['cycle']:.2f},{st2['cycle']:.2f}"
            )
        out.write_text("\n".join(lines) + "\n")
        print(f"\nwrote {out}")


if __name__ == "__main__":
    main()
