"""Parameter sweep over a cell — "環境の自由度" made operational.

The cell is authored once as a function of its parameters; every variant
is then baked deterministically and compared by the numbers that matter
(cycle time, sensor timing, clearance). This is the loop behind layout
studies and cycle-time regression: change the environment, re-simulate,
read the diff — no re-teaching. `bt.sweep` runs the grid and tables it;
`bt.optimize` searches it for the best feasible point — deterministically,
without a random number anywhere.

Runs from a checkout with no downloads (primitive-geometry arm):

    python examples/basics/sweep_demo.py
"""

from pathlib import Path

import botrail as bt

ASSETS = Path(__file__).resolve().parents[1] / "assets"


def build_cell(velocity: float = 0.25, lane_y: float = 0.6) -> bt.Scene:
    """Conveyor feed → beam stop → approach → work → home, parameterized
    by belt speed and by how close the conveyor lane runs to the robot."""
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
    scene.add_box("crate", (0.04, 0.04, 0.04), (-0.5, lane_y, 0.3))
    scene.add_conveyor(
        "belt",
        zone_position=(-0.2, lane_y, 0.3),
        zone_size=(1.2, 0.3, 0.3),
        velocity=(velocity, 0.0, 0.0),
        running=False,
    )
    scene.add_beam_sensor(
        "eye", frm=(0.0, lane_y - 0.2, 0.3), to=(0.0, lane_y + 0.2, 0.3)
    )
    scene.add_segment("approach", goal=[0.6, -0.5, 0.8, 0.0, 0.4, 0.0])
    scene.add_segment("home", goal=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    sq = scene.sequence("cycle")
    sq.step("feed", actions=[bt.seq.start("belt")], transition=bt.seq.signal("eye"))
    sq.step("stop", actions=[bt.seq.stop("belt")])
    sq.step("approach", actions=[bt.seq.motion("approach")])
    sq.step("work", transition=bt.seq.elapsed(0.5))
    sq.step("home", actions=[bt.seq.motion("home")])
    return scene


def metrics(tl: bt.SequenceTimeline) -> dict:
    """The three numbers under study, read off one bake."""
    return {
        "cycle": tl.duration,
        "feed": tl.step_span("feed").duration,
        "clearance": float(tl.min_clearance()),
    }


def bake(velocity: float = 0.25, lane_y: float = 0.6):
    """One variant as `(cycle, feed, clearance)` — the same numbers a
    sweep row holds, for a test that wants them by hand."""
    m = metrics(build_cell(velocity, lane_y).simulate_sequence("cycle"))
    return m["cycle"], m["feed"], m["clearance"]


def main() -> None:
    print("== belt speed sweep (lane_y = 0.60 m) ==")
    speed = bt.sweep(
        build_cell,
        grid={"velocity": [0.10, 0.15, 0.20, 0.25, 0.30, 0.35], "lane_y": [0.6]},
        metrics=metrics,
        sequence="cycle",
    )
    print(speed.to_markdown())
    print("-> only the feed wait moves; the motion part of the cycle is fixed\n")

    print("== conveyor lane sweep (belt = 0.25 m/s) ==")
    lane = bt.sweep(
        build_cell,
        grid={"velocity": [0.25], "lane_y": [0.70, 0.60, 0.50, 0.40, 0.35]},
        metrics=metrics,
        sequence="cycle",
    )
    print(lane.to_markdown())
    print("-> the cycle barely moves, the safety margin is what shrinks\n")

    print("== both at once: cycle time over the grid ==")
    both = bt.sweep(
        build_cell,
        grid={"velocity": [0.15, 0.25, 0.35], "lane_y": [0.7, 0.5, 0.35]},
        metrics=metrics,
        sequence="cycle",
    )
    print(both.pivot("lane_y", "velocity", "cycle"))
    print("(clearance over the same grid)")
    print(both.pivot("lane_y", "velocity", "clearance"))

    print("== the question a layout meeting asks: fastest cycle with 0.4 m of clearance ==")
    best = bt.optimize(
        build_cell,
        space={"velocity": (0.10, 0.40, 0.05), "lane_y": (0.30, 0.70, 0.05)},
        objective="cycle",
        constraints={"clearance": (">=", 0.4)},
        metrics=metrics,
        sequence="cycle",
        method="descent",
    )
    print(f"{best.params} -> cycle {best.row['cycle']:.2f} s, clearance {best.row['clearance']:.2f} m "
          f"({len(best.evaluated)} bakes, coordinate descent; the full grid is 63)")

    print("\nEvery row above is a deterministic bake: re-running this script")
    print("prints the same numbers, which is what makes them assertable in CI")
    print("(see python/tests/test_cell_regression.py).")


if __name__ == "__main__":
    main()
