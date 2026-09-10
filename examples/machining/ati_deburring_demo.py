"""ATI RCV-250 / CRX-10iA: nominal aluminum edge-deburring cycle.

The prebuilt CRX kit carries the compliant spindle; a separately ordered
9150-RC-B-24645 aluminum-cut bur is added as a cylindrical envelope
(`bt.tools.rotary_bur`) on the collet nose. The cycle enables
compliance, spins up, runs a tangential lead-in, a 100 mm edge and a
lead-out at 8 mm/s with 0.2 mm radial engagement, then stops. Only the
`bur_cutter` link may touch the stock; housing, shank and every rapid
stay collision checked. Pneumatic compliance, cutting forces, flutes and
material removal are not simulated. The cycle starts only on its
permissives (air, clamp — simulated inputs, not sensors from the kit);
the `air_missing` scenario takes the inhibited branch instead.

Run with:  python examples/machining/ati_deburring_demo.py [--studio]
                 [--output DIR]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import botrail as bt

ARM = "fanuc/crx/crx10ia/r1"
KIT = "ati/rcv/rcv-250-crx10-kit/r1"
WORKPIECE = "fixture/stock"
CYCLE = "deburr_cycle"

# --- the bur: ATI RCV-250 manual 9610-50-1043 (aluminum-cut 24645) --------
BIT_MODEL = "9150-RC-B-24645"
SHANK_MM = 6.35
BUR_DIAMETER_MM = 9.525
BUR_CUTTING_LENGTH_MM = 15.875
EXPOSED_SHANK_MM = 10.0  # demo mounting choice; insertion depth needs measurement
# Collet nose in the kit's spindle mount frame: ATI_MR_CRX.zip / Models /
# RCV250 reference-model transform (nominal CAD, not a measured TCP).
COLLET_NOSE_M = (0.164522, 0.0, 0.03646)

# --- the cut (demo choices) ------------------------------------------------
MATERIAL = "Aluminum, grade unspecified (demo)"
FEED_MPS = 0.008
RADIAL_ENGAGEMENT_MM = 0.2


def build() -> bt.Scene:
    """The arm with the kit and the bur, the coupon on its fixture, and the cycle."""
    diameter = BUR_DIAMETER_MM / 1000
    head = BUR_CUTTING_LENGTH_MM / 1000
    bur = bt.tools.rotary_bur(
        diameter=diameter,
        cutting_length=head,
        shank_diameter=SHANK_MM / 1000,
        exposed_shank=EXPOSED_SHANK_MM / 1000,
    )
    arm = bt.Robot.from_catalog(ARM)
    kit = bt.Robot.from_catalog(KIT)
    robot = arm.attach_tool(kit, prefix="kit_").attach_tool(
        bur,
        flange="kit_spindle/mount",
        mount="mount",
        tcp="tcp",
        prefix="bur_",
        offset_position=COLLET_NOSE_M,
        offset_quaternion=(0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4)),
    )
    scene = bt.Scene(robot)
    scene.set_part(
        "crx10ia/tool2",
        manufacturer="ATI Industrial Automation",
        model=BIT_MODEL,
        category="cutting_tool",
        description="Separately ordered aluminum-cut bur; cylindrical reference envelope",
    )
    # A taught vertical spindle posture, with the bur outside the -X edge.
    ik = robot.ik(
        (0.635, 0, 0.6),
        (0, 0, 0, 1),
        link="bur_tcp",
        seed=[0.12, 0.80, 0.93, 0.75, 2.97, 2.31],
        max_iters=200,
    )
    if not ik.converged:
        raise ValueError("Selected projection cannot reach the taught approach pose")
    scene.set_joint_positions(ik.q)

    # -- the coupon and its fixture (live dimensions, no baked geometry) ----
    edge_y = -diameter / 2 + RADIAL_ENGAGEMENT_MM / 1000
    scene.add_frame("edge", (0.7, 0, 0.6))
    scene.add_box(
        WORKPIECE,
        size=(0.10, 0.04, 0.01),
        position=(0.7, edge_y - 0.02, 0.608),
        color=(0.72, 0.75, 0.78),
    )
    scene.set_part(
        WORKPIECE,
        model="Aluminum coupon 100 x 40 x 10 mm",
        category="workpiece",
        attributes={"material": MATERIAL},
    )
    scene.add_box(
        "fixture/bench",
        (0.12, 0.14, 0.57),
        (0.95, -0.075, 0.285),
        color=(0.25, 0.31, 0.38),
    )
    scene.add_box(
        "fixture/support",
        (0.31, 0.02, 0.033),
        (0.805, edge_y - 0.03, 0.5865),
        color=(0.32, 0.36, 0.40),
    )
    scene.add_box(
        "fixture/clamp",
        (0.025, 0.022, 0.018),
        (0.7, edge_y - 0.037, 0.612),
        color=(0.85, 0.50, 0.10),
    )
    scene.allow_link_obstacle_contact("bur_cutter", WORKPIECE)

    # -- the toolpath: tangential lead-in and lead-out, no axial plunge ------
    path = bt.toolpath.builder(frame="edge")
    path.rapid_to((-0.065, 0, 0), axis=(0, 0, 1))
    path.feed(FEED_MPS).line_to((0.065, 0, 0), axis=(0, 0, 1))
    scene.add_toolpath("edge_deburr", path.build())

    # -- the program: start permissives, then the cut ----------------------
    for name, initial in (
        ("sim_air_ok", True),
        ("sim_work_clamped", True),
        ("motor_run", False),
        ("compliance_on", False),
        ("cycle_inhibited", False),
    ):
        scene.define_signal(name, initial=initial)
    scene.add_scenario("air_missing", signals={"sim_air_ok": False})
    sq = scene.sequence(CYCLE)
    branch = sq.select("start_permissives")
    valid = branch.when(
        bt.seq.all_of(bt.seq.signal("sim_air_ok"), bt.seq.signal("sim_work_clamped"))
    )
    valid.step(
        "enable_compliance",
        actions=[bt.seq.set_signal("compliance_on")],
        transition=bt.seq.elapsed(0.3),
    )
    valid.step(
        "spin_up_simulated",
        actions=[bt.seq.set_signal("motor_run")],
        transition=bt.seq.elapsed(1.5),
    )
    valid.step(
        "deburr_edge",
        actions=[bt.seq.toolpath("edge_deburr")],
        transition=bt.seq.done(),
    )
    valid.step(
        "stop_motor",
        actions=[bt.seq.set_signal("motor_run", False)],
        transition=bt.seq.elapsed(1),
    )
    valid.step("release_compliance", actions=[bt.seq.set_signal("compliance_on", False)])
    branch.when(bt.seq.otherwise()).step(
        "inhibited", actions=[bt.seq.set_signal("cycle_inhibited")]
    )
    scene.add_cut_trace("deburring", "motor_run", scene.robots[0], spin_link="bur_cutter")
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--output", type=Path, help="directory for the project and the BOM")
    args = parser.parse_args()

    scene = build()
    timeline = scene.simulate_sequence(CYCLE, toolpath_spin="optimize")
    print(f"Nominal rigid-tool cycle: {timeline.duration:.2f} s")
    for name, t0, t1 in timeline.step_spans:
        print(f"  {name:<20} {t0:6.2f} - {t1:6.2f} s")
    print(bt.mounting.report(scene).to_markdown())
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        scene.save_project(args.output / "cell.botrail")
        scene.export_bom(args.output / "bom.csv")
        print("Saved:", args.output)
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
