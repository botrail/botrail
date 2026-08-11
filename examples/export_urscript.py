"""One source, two outputs: the sequence that drives the simulation also
compiles to a robot controller program with real I/O.

A conveyor feeds a part over a photoelectric beam; the six-axis arm meets
it, grips with a vacuum coil, judges it against an in-spec zone, and
either places it or purges it. `simulate()` bakes the deterministic
timeline (cycle time, timing chart, USD export); the *same* sequence then
lowers to URScript — moves from the rollout's own planned paths, the
part-arrival *edge* as a two-stage digital-input wait, the SFC branch as
a wait-any plus `if/elif` (the arm this bake skipped is in the program
too — the controller decides at runtime what the bake decided by state),
coils as digital-output writes, timers as sleeps. The name → port wiring
is the only thing the controller needs on top.

Run with:  python examples/export_urscript.py [pick_cell.script]
"""

from __future__ import annotations

import sys
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent

# The cell's I/O list, as it would appear on the electrical drawing:
# inputs are contacts the program waits on, outputs are coils it drives.
INPUTS = {"part_at_pick": 2, "in_spec": 3}  # beam → DI2, spec gauge → DI3
OUTPUTS = {"conv": 0, "vacuum": 1}  # conveyor run → DO0, vacuum valve → DO1


def build_cell() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(HERE / "simple_arm.urdf"))

    # A 6 cm part starts upstream on a belt running +x at 0.12 m/s —
    # slow enough that the arm is in position well before the part
    # arrives, which is what makes waiting on the *edge* sound.
    scene.add_box("part", size=(0.06, 0.06, 0.06), position=(-0.45, 0.35, 0.03))
    scene.add_conveyor(
        "conv",
        zone_position=(-0.1, 0.35, 0.05),
        zone_size=(0.9, 0.2, 0.14),
        velocity=(0.12, 0.0, 0.0),
        running=False,
    )
    # The beam crosses the belt at the pick point; the spec zone says
    # whether the picked part sits where a good one should.
    scene.add_beam_sensor(
        "part_at_pick",
        frm=(0.25, 0.25, 0.03),
        to=(0.25, 0.45, 0.03),
        watch=["part"],
    )
    scene.add_zone_sensor(
        "in_spec",
        position=(0.25, 0.35, 0.05),
        size=(0.2, 0.2, 0.2),
        watch=["part"],
    )
    scene.define_signal("vacuum")

    scene.add_segment("to_pick", goal=[0.95, 0.85, -1.1, 0.25, 0.0, 0.0])
    scene.add_segment("place", goal=[-0.9, 0.85, -1.1, 0.25, 0.0, 0.0])
    scene.add_segment("home", goal=[0.0] * 6)
    return scene


def author_sequence(scene: bt.Scene) -> bt.seq.SequenceBuilder:
    sq = scene.sequence("pick")
    # Belt on and pre-position in parallel; then wait for the part to
    # *arrive* — a rising edge. A part already sitting on the beam from a
    # broken cycle is a level, not an arrival, and an edge that fires
    # while the arm is still travelling would be missed — which is why
    # the wait is its own step, entered once the arm is ready.
    sq.step("feed", actions=[bt.seq.start("conv"), bt.seq.motion("to_pick")])
    sq.step("await part", transition=bt.seq.rising("part_at_pick"))
    sq.step("halt", actions=[bt.seq.stop("conv")])
    sq.step("grip", actions=[bt.seq.set_signal("vacuum")], transition=bt.seq.elapsed(0.3))
    sq.step("hold", actions=[bt.seq.attach("part")])
    # SFC selection: the spec gauge decides. This bake takes the good
    # path; the reject arm still compiles into the controller program.
    judge = sq.select("judge")
    judge.when(bt.seq.signal("in_spec")).step(
        "place", actions=[bt.seq.motion("place")]
    ).step("release", actions=[bt.seq.set_signal("vacuum", False), bt.seq.detach("part")])
    reject = judge.when(bt.seq.otherwise())
    reject.step(
        "purge",
        actions=[bt.seq.set_signal("vacuum", False), bt.seq.detach("part")],
        transition=bt.seq.elapsed(0.3),
    )
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
