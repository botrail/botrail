"""Parameter sweep over a transport cell — arrival variation without dice.

A real AGV does not arrive at the same instant every tact. The tempting
answer is a random arrival time, but that trades away the one thing this
tool has: bake the same cell twice and get the same numbers. So variation
is swept, not sampled — you get the whole response curve instead of a
distribution, and every row of it is assertable in CI.

Two axes, both of which a cell designer actually argues about:

  * **How late the vehicle is called.** The cell absorbs a late call for
    free up to a point, and then pays for it second for second. Where that
    knee sits is the useful number: it is the schedule slack.
  * **How deep the dock sits.** Further in is a shorter reach for the arm
    and a longer drive; too far in and the machine hits the pallet. The
    sweep prints the whole range including where it stops being feasible.

Runs the MiR250 cell from `agv_cell_demo` (first run downloads the Franka
and the vehicle meshes, ~12 MB, cached).

    python examples/agv_sweep_demo.py
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import agv_cell_demo as cell  # noqa: E402


def bake(call_delay: float = 0.0, dock_y: float = None):
    """Bakes one variant; returns `(cycle, gate_wait, transfer_start)` or a
    reason string if the cell does not stand up."""
    original = cell.DOCK
    if dock_y is not None:
        cell.DOCK = (original[0], dock_y)
    try:
        scene = cell.build_scene()
        name = cell.build_cycle(scene, call_delay=call_delay)
        tl = scene.simulate_sequence(name, max_duration=120.0)
    except (ValueError, RuntimeError) as err:
        return str(err)
    finally:
        cell.DOCK = original
    entry = tl.step_span("入構")
    return tl.duration, entry.end - entry.start, tl.step_span("移載").start


def main() -> None:
    print(f"== dispatch delay sweep (dock at y = {cell.DOCK[1]:.2f}) ==")
    print(f"{'late s':>7} | {'cycle s':>8} | {'entry s':>8} | {'transfer @ s':>12}")
    base = None
    for delay in (0.0, 1.0, 2.0, 2.5, 3.0, 4.0, 6.0):
        row = bake(call_delay=delay)
        if isinstance(row, str):
            print(f"{delay:7.1f} | {row}")
            continue
        cycle, entry, transfer = row
        base = cycle if base is None else base
        print(f"{delay:7.1f} | {cycle:8.2f} | {entry:8.2f} | {transfer:12.2f}"
              f"   (+{cycle - base:.2f})")
    print("-> free until the knee, then paid second for second. The knee is")
    print("   the schedule slack: how late dispatch may be before the cell")
    print("   waits on it rather than the other way round.\n")

    print("== dock depth sweep (call on time) ==")
    print(f"{'dock y':>7} | {'cycle s':>8} | {'entry s':>8} | {'transfer @ s':>12}")
    for y in (-1.10, -1.12, -1.13, -1.14, -1.15):
        row = bake(dock_y=y)
        if isinstance(row, str):
            reason = "vehicle hits the pallet" if "collides" in row else "arm cannot reach"
            print(f"{y:7.2f} | infeasible — {reason}")
            continue
        cycle, entry, transfer = row
        print(f"{y:7.2f} | {cycle:8.2f} | {entry:8.2f} | {transfer:12.2f}")
    print("-> 30 mm of band, and it is the *layout* that sets it: the pallet")
    print("   keeps the vehicle out, the arm's reach pulls it in, and the")
    print("   carton has to land on the deck rather than over its edge")

    print("\nEvery row is a deterministic bake: re-running prints the same")
    print("numbers, which is what makes an arrival-variation study something")
    print("you can assert in CI rather than a distribution you sample.")


if __name__ == "__main__":
    main()
