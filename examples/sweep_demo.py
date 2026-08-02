"""Parameter sweep over a cell — "環境の自由度" made operational.

The cell is authored once as a function of its parameters; every variant
is then baked deterministically and compared by the numbers that matter
(cycle time, sensor timing, clearance). This is the loop behind layout
studies and cycle-time regression: change the environment, re-simulate,
read the diff — no re-teaching.

Runs from a checkout with no downloads (primitive-geometry arm):

    python examples/sweep_demo.py
"""

from pathlib import Path

import botrail as bt

EXAMPLES = Path(__file__).resolve().parent


def build_cell(velocity: float = 0.25, lane_y: float = 0.6) -> bt.Scene:
    """Conveyor feed → beam stop → approach → work → home, parameterized
    by belt speed and by how close the conveyor lane runs to the robot."""
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))
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


def bake(velocity: float = 0.25, lane_y: float = 0.6):
    tl = build_cell(velocity, lane_y).simulate_sequence("cycle")
    return tl.duration, tl.step_span("feed").duration, float(tl.min_clearance())


def main() -> None:
    print("== belt speed sweep (lane_y = 0.60 m) ==")
    print(f"{'belt m/s':>9} | {'cycle s':>8} | {'feed s':>7} | {'clearance m':>11}")
    base_cycle = None
    for v in (0.10, 0.15, 0.20, 0.25, 0.30, 0.35):
        cycle, feed, clearance = bake(velocity=v)
        base_cycle = base_cycle if base_cycle is not None else cycle
        print(f"{v:9.2f} | {cycle:8.2f} | {feed:7.2f} | {clearance:11.3f}")
    print("-> only the feed wait moves; the motion part of the cycle is fixed\n")

    print("== conveyor lane sweep (belt = 0.25 m/s) ==")
    print(f"{'lane_y m':>9} | {'cycle s':>8} | {'feed s':>7} | {'clearance m':>11}")
    for y in (0.70, 0.60, 0.50, 0.40, 0.35):
        cycle, feed, clearance = bake(lane_y=y)
        print(f"{y:9.2f} | {cycle:8.2f} | {feed:7.2f} | {clearance:11.3f}")
    print("-> the cycle barely moves, the safety margin is what shrinks")
    print("\nEvery row above is a deterministic bake: re-running this script")
    print("prints the same numbers, which is what makes them assertable in CI")
    print("(see python/tests/test_cell_regression.py).")


if __name__ == "__main__":
    main()
