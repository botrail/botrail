"""ATI RCV-250 / CRX-10iA: nominal aluminum edge-deburring cycle.

    uv run python examples/machining/ati_deburring_demo.py --studio

Uses prebuilt catalog products and a separately ordered 9150-RC-B-24645
aluminum-cut bur. The rigid cutter envelope can touch stock; housing and
shank remain collision checked. This does not simulate pneumatic compliance
or predict removal/finish. See examples/process/README.md for measured inputs.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import botrail as bt

DEFAULT_SETUP = Path(__file__).parents[1] / "process/ati_rcv250.json"
WORKPIECE = "fixture/stock"
CYCLE = "deburr_cycle"


def build_cell(setup_path: Path = DEFAULT_SETUP) -> bt.Scene:
    profile = json.loads(setup_path.read_text())
    settings, facts = profile["settings"], profile["facts"]
    if settings["bit_model"] != "9150-RC-B-24645":
        raise ValueError(
            "This example models the 9150-RC-B-24645; change its geometry for another bit"
        )
    diameter = facts["bur_diameter_mm"]["value"] / 1000
    head = facts["bur_cutting_length_mm"]["value"] / 1000
    exposed = settings["exposed_shank_mm"] / 1000
    bur = bt.tools.rotary_bur(
        diameter=diameter,
        cutting_length=head,
        shank_diameter=facts["shank_mm"]["value"] / 1000,
        exposed_shank=exposed,
    )
    arm = bt.Robot.from_catalog("fanuc/crx/crx10ia/r1")
    kit = bt.Robot.from_catalog("ati/rcv/rcv-250-crx10-kit/r1")
    nose = facts["collet_nose_m"]["value"]
    robot = arm.attach_tool(kit, prefix="kit_").attach_tool(
        bur,
        flange="kit_spindle/mount",
        mount="mount",
        tcp="tcp",
        prefix="bur_",
        offset_position=nose,
        offset_quaternion=(0, math.sin(math.pi / 4), 0, math.cos(math.pi / 4)),
    )
    scene = bt.Scene(robot)
    scene.set_part(
        "crx10ia/tool2",
        manufacturer="ATI Industrial Automation",
        model=settings["bit_model"],
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
    engagement = settings["radial_engagement_mm"] / 1000
    edge_y = -diameter / 2 + engagement
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
        attributes={"material": settings["material"]},
    )
    # Live fixture dimensions; no baked environment geometry.
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
    path = bt.toolpath.builder(frame="edge")
    path.rapid_to((-0.065, 0, 0), axis=(0, 0, 1))
    # Tangential lead-in and lead-out; no axial plunge into the stock.
    path.feed(settings["feed_mps"]).line_to((0.065, 0, 0), axis=(0, 0, 1))
    scene.add_toolpath("edge_deburr", path.build())
    # Keep the selected offset in the saved setup: report compares it with
    # the live tool, so an edited/calibrated frame can be checked independently.
    profile["tcp_offset_m"] = [nose[0] + exposed + head, nose[1], nose[2]]
    bt.process.configure(scene, WORKPIECE, profile)
    setup_ok = not any(
        c["status"] == "fail" for c in bt.process.report(scene, WORKPIECE).checks
    )
    # Physical requirements on the real kit. Supply outlets, valve drivers and
    # terminal mapping remain unconnected until the cell has selected them.
    bt.connections.port(
        scene,
        "ati.motor_air",
        "crx10ia/tool",
        "pneumatic",
        "load",
        pressure_bar=6.2,
        flow_l_min=852,
        reference=facts["motor_pressure_bar"]["source"],
    )
    bt.connections.port(
        scene,
        "ati.compliance_air",
        "crx10ia/tool",
        "pneumatic",
        "load",
        pressure_min_bar=1,
        pressure_max_bar=4.1,
        reference=facts["compliance_pressure_min_bar"]["source"],
    )
    for name, valve in (
        ("motor_run", "9005-50-6203"),
        ("compliance_on", "9005-50-6202"),
    ):
        bt.connections.port(
            scene,
            "ati." + name,
            "crx10ia/tool",
            "signal",
            "input",
            voltage_v=24,
            signal_type="digital",
            reference=f"9610-50-1049-03; {valve}",
        )
    for name, initial in (
        ("sim_air_ok", True),
        ("sim_work_clamped", True),
        ("sim_setup_ok", setup_ok),
        ("motor_run", False),
        ("compliance_on", False),
        ("cycle_inhibited", False),
    ):
        scene.define_signal(name, initial=initial)
    scene.add_scenario("air_missing", signals={"sim_air_ok": False})
    sq = scene.sequence(CYCLE)
    branch = sq.select("start_permissives")
    valid = branch.when(
        bt.seq.all_of(
            bt.seq.signal("sim_setup_ok"),
            bt.seq.signal("sim_air_ok"),
            bt.seq.signal("sim_work_clamped"),
        )
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
    valid.step(
        "release_compliance", actions=[bt.seq.set_signal("compliance_on", False)]
    )
    branch.when(bt.seq.otherwise()).step(
        "inhibited", actions=[bt.seq.set_signal("cycle_inhibited")]
    )
    scene.add_cut_trace(
        "deburring", "motor_run", scene.robots[0], spin_link="bur_cutter"
    )
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--setup", type=Path, default=DEFAULT_SETUP)
    parser.add_argument(
        "--output", type=Path, help="Directory for project, setup, reports and BOM"
    )
    args = parser.parse_args()
    scene = build_cell(args.setup)
    timeline = scene.simulate_sequence(CYCLE, toolpath_spin="optimize")
    report = bt.process.report(scene, WORKPIECE)
    print(f"Nominal rigid-tool cycle: {timeline.duration:.2f} s")
    print(report.to_markdown())
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        scene.save_project(args.output / "cell.botrail")
        report.save(args.output / "process.json")
        report.save(args.output / "process.md")
        bt.connections.report(scene).save(args.output / "connections.json")
        (args.output / "setup.json").write_text(
            json.dumps(bt.process.setup(scene, WORKPIECE), indent=2) + "\n"
        )
        scene.export_bom(args.output / "bom.csv")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
