"""One source, two outputs: the sequence that drives the simulation also
compiles to a robot controller program with real I/O.

A conveyor feeds a part over a photoelectric beam; the six-axis arm meets
it, grips with a vacuum coil, and carries it home. `simulate()` bakes the
deterministic timeline (cycle time, timing chart, USD export); the *same*
sequence then lowers to URScript — moves from the rollout's own planned
paths, the beam wait as a digital-input spin, the conveyor and vacuum
coils as digital-output writes, the settle timer as a sleep. The name →
port wiring is the only thing the controller needs on top.

Run with:  python examples/export_urscript.py [pick_cell.script]
"""

from __future__ import annotations

import sys
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent

# The cell's I/O list, as it would appear on the electrical drawing:
# inputs are contacts the program waits on, outputs are coils it drives.
INPUTS = {"part_at_pick": 2}  # beam sensor → DI2
OUTPUTS = {"conv": 0, "vacuum": 1}  # conveyor run → DO0, vacuum valve → DO1


def build_cell() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(HERE / "simple_arm.urdf"))

    # A 6 cm part starts upstream on a belt running +x at 0.2 m/s.
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(-0.45, 0.35, 0.03))
    scene.add_conveyor(
        "conv",
        zone_position=(-0.1, 0.35, 0.05),
        zone_size=(0.9, 0.2, 0.14),
        velocity=(0.2, 0.0, 0.0),
        running=False,
    )
    # The beam crosses the belt at the pick point.
    scene.add_beam_sensor(
        "part_at_pick",
        frm=(0.25, 0.25, 0.03),
        to=(0.25, 0.45, 0.03),
        watch=["part"],
    )
    scene.define_signal("vacuum")

    scene.add_segment("to_pick", goal=[0.95, 0.85, -1.1, 0.25, 0.0, 0.0])
    scene.add_segment("home", goal=[0.0] * 6)
    return scene


def author_sequence(scene: bt.Scene) -> bt.seq.SequenceBuilder:
    sq = scene.sequence("pick")
    # Belt on and pre-position in parallel; the step ends when the part
    # has arrived *and* the arm is there (series contacts).
    sq.step(
        "feed",
        actions=[bt.seq.start("conv"), bt.seq.motion("to_pick")],
        transition=bt.seq.all_of(bt.seq.signal("part_at_pick"), bt.seq.done()),
    )
    sq.step("halt", actions=[bt.seq.stop("conv")])
    sq.step("grip", actions=[bt.seq.set_signal("vacuum")], transition=bt.seq.elapsed(0.3))
    sq.step("hold", actions=[bt.seq.attach("part")])
    sq.step("return", actions=[bt.seq.motion("home")])
    return sq


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE.parent / "pick_cell.script"

    scene = build_cell()
    sq = author_sequence(scene)

    tl = sq.simulate()
    print(f"simulated cycle: {tl.duration:.2f}s, steps:")
    for name, start, end in tl.step_spans:
        print(f"  {name:<8} {start:6.2f} – {end:6.2f}s")

    tl.export_script(out, inputs=INPUTS, outputs=OUTPUTS)
    print(f"\nwrote {out} — the same steps, as a controller program:\n")
    print(tl.to_script(inputs=INPUTS, outputs=OUTPUTS))


if __name__ == "__main__":
    main()
