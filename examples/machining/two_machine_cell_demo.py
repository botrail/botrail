"""One arm, two machines — the cell every tending payback is figured on.

Two ROBODRILL-sized machining centres face each other across an aisle,
their side doors toward the arm on its stand between them, a bench per
machine beside the aisle with the blank going in and the finished part
coming out. The arm is the MELFA ASSISTA with the three-tool hand of
`machine_tending_demo.py`, and everything it does at one machine —
UNCLAMP by the pin, the door by the fork, the swap by the gripper, CLAMP,
the door shut, CYCLE START — it does at the other, taught the same way
(`machine_tending_demo.teach` / `program`, prefixed `a_` and `b_`). Both
machines are worked by hand, each running its own program
(`bt.tending.manual`), so the cell scans three programs together.

What the bake is for: with a part program longer than the swap, one arm
keeps two spindles cutting. The timeline's utilization says how much of
the cycle each machine runs and how much the arm works; the interlock
table carries three programs and the handshake spec two CNCs; and the
same guards hold at both doors.

Run with:  python examples/machining/two_machine_cell_demo.py [out.usdc] [--studio]
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import botrail as bt
import machine_tending_demo as one

ROBOT = one.ROBOT
AISLE = 0.30            # the stand's centre to each machine's `entry` frame
BENCH_OUT = 0.55        # the benches' centres, either side of the aisle
BENCH = (0.70, 0.50, 0.80)   # a bench turned along the aisle
CYCLE_S = 90.0          # a part program longer than the swap: the arm, not the spindle, sets the pace
PRESS_STANDOFF = 0.06   # the swing from the far door to the panel passes its plate's corner: start wider
MACHINE = {"door": "manual", "door_side": "right", "panel": "door", "buttons": one.BUTTONS, "detail": "full",
           "model": "α-D21MiB5 Plus", "manufacturer": "FANUC", "mass_kg": 2000}


def build() -> tuple[bt.Scene, dict[str, bt.tending.Handshake]]:
    scene = bt.Scene(one.tool(), name=ROBOT)
    a = bt.parts.machine_tool(scene, "vmc_a", **MACHINE)
    (ex, ey, _), _ = scene.frame("vmc_a/entry")
    # B faces A across the aisle: a half turn puts its right-hand door
    # toward the arm, and the placement lines its entry frame up with A's
    # (the openings sit forward of the body's centre, so the mirror image
    # lands at twice A's offset), `2 * AISLE` away.
    b = bt.parts.machine_tool(scene, "vmc_b", position=(2 * ex + 2 * AISLE, 2 * ey), yaw=math.pi, **MACHINE)
    stand_xy = (ex + AISLE, ey)
    stand = bt.parts.pedestal(scene, "stand", catalog=one.STAND, height=one.STAND_H, position=stand_xy,
                              yaw=math.pi)
    (mx, my, mz), mq = scene.frame(stand.frames[0])
    scene.set_robot_base_pose((mx, my, mz + 0.005), mq, robot=ROBOT)
    scene.allow_link_obstacle_contact(scene.robot_of(ROBOT).link_names[0], "stand/top", robot=ROBOT)

    handshakes: dict[str, bt.tending.Handshake] = {}
    for tag, vmc, side in (("a", a, -1.0), ("b", b, 1.0)):
        # The vise on the door side of the table, the finished part in it.
        (tx, ty, tz), _ = scene.frame(f"{vmc.name}/table")
        (dx, _dy, _dz), _ = scene.frame(f"{vmc.name}/entry")
        toward = 1.0 if dx > tx else -1.0
        # 3 mm a side around the part: two IK residuals (pick and place)
        # add up on the far machine, and 2 mm read as a graze.
        vise = bt.parts.vise(scene, f"vise_{tag}", (tx + toward * 0.25, ty, tz), opening=one.PART[1] + 0.006,
                             model="VQ-125", manufacturer="ACME", mass_kg=12)
        (jx, jy, jz), _ = scene.frame(vise.frames[0])
        scene.add_box(f"finished_{tag}", size=one.PART, position=(jx, jy, jz + one.SEAT + one.PART[2] / 2),
                      color=one.FINISHED)
        scene.set_part(f"finished_{tag}", kind="obstacle", category="workpiece", model="WP-50",
                       mass_kg=one.PART_MASS)
        # This machine's bench, beside the aisle: the blank nearer the
        # machine, the out slot nearer the other one.
        bench = bt.parts.table(scene, f"stocker_{tag}", size=BENCH, position=(stand_xy[0], ey + side * BENCH_OUT),
                               model="WB-700", manufacturer="ACME", mass_kg=36)
        (bx, by, bz), _ = scene.frame(bench.frames[0])
        blank_x, out_x = bx - toward * one.SLOT, bx + toward * one.SLOT
        scene.add_frame(f"stocker_{tag}/blank", position=(blank_x, by, bz))
        scene.add_frame(f"stocker_{tag}/out", position=(out_x, by, bz))
        scene.add_box(f"blank_{tag}", size=one.PART, position=(blank_x, by, bz + one.SEAT + one.PART[2] / 2),
                      color=one.BLANK)
        scene.set_part(f"blank_{tag}", kind="obstacle", category="workpiece", model="WP-50-raw",
                       mass_kg=one.PART_MASS)
        for part in (f"finished_{tag}", f"blank_{tag}"):
            for link in one.PADS:
                scene.allow_link_obstacle_contact(link, part, robot=ROBOT)
        handshakes[tag] = bt.tending.manual(scene, vmc, cycle_s=CYCLE_S, clamp_s=one.CLAMP_S,
                                            buttons=("unclamp", "clamp", "cycle_start"))

    # Taught once per machine with its own prefix; the park motion once.
    one.teach(scene, a, vise="vise_a", stocker="stocker_a", prefix="a_", home=True, press_standoff=PRESS_STANDOFF)
    one.teach(scene, b, vise="vise_b", stocker="stocker_b", prefix="b_", home=False, press_standoff=PRESS_STANDOFF)
    sq = scene.sequence("tend")
    one.program(scene, a, handshakes["a"], sq=sq, prefix="a_", parts=("finished_a", "blank_a"), home=False)
    one.program(scene, b, handshakes["b"], sq=sq, prefix="b_", parts=("finished_b", "blank_b"), home=True)
    return scene, handshakes


def bake():
    scene, handshakes = build()
    tl = scene.simulate_sequences(["tend", "vmc_a", "vmc_b"], max_duration=600.0)
    return scene, handshakes, tl


def running_fraction(tl: bt.SequenceTimeline, lane: str) -> float:
    """How much of the cycle a machine's `running` lane is high."""
    return sum(b - a for a, b in tl.signal(lane).high_spans()) / tl.duration


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("out", nargs="?", default=str(HERE / "two_machine_cell.usdc"))
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    scene, handshakes, tl = bake()
    print(f"one arm, two machines: cycle {tl.duration:.2f}s, arm busy {tl.utilization(ROBOT):.0%}")
    for hs in handshakes.values():
        print(f"  {hs.machine:<6} cutting {running_fraction(tl, hs.signal('running')):.0%} of the cycle")
    clearance = tl.min_clearance()
    pair = f" ({clearance.pair[0]} x {clearance.pair[1]})" if clearance.pair else ""
    print(f"min clearance over the cycle: {float(clearance) * 1e3:.1f} mm at {clearance.t:.2f}s{pair}")
    warnings = tl.export_usd(args.out, fps=60)
    print(f"wrote {args.out}" + (f" ({warnings})" if warnings else ""))
    out = Path(args.out).with_name("two_machine_cell_deliverables")
    out.mkdir(parents=True, exist_ok=True)
    scene.export_interlocks(out / "two_machine_cell_interlocks.md")
    scene.export_plcopen(out / "two_machine_cell.plcopen.xml", name="two machine cell")
    tl.export_handshake_spec(out / "two_machine_cell_handshake.md")
    scene.export_layout(out / "two_machine_cell_layout.svg", scale=120, title="two machine cell")
    scene.cell_report({"cycle": tl}, title="two machine cell").save(out / "two_machine_cell_report.md")
    print(f"wrote the interlock table, the PLCopen file, the handshake spec, the layout and the report to {out}/")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
