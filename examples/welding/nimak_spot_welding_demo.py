"""NIMAK 95.020.516/P3U: two-position spot-welding dry cycle on BX250L.

    uv run python examples/welding/nimak_spot_welding_demo.py --studio

The plates retain a positive electrode clearance: force, electric current and
weld formation are not simulated. The jaw and arm move through the real catalog
kinematic tree. All gun/workpiece collisions remain enabled.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import botrail as bt

DEFAULT_SETUP = Path(__file__).parents[1] / "process/nimak_95_020_516.json"
WORKPIECE = "fixture/lower_sheet"
CYCLE = "spot_weld_dry_cycle"


def build_cell(setup_path: Path = DEFAULT_SETUP) -> bt.Scene:
    profile = json.loads(setup_path.read_text())
    settings = profile["settings"]
    thickness = settings["sheet_thickness_mm"] / 1000
    clearance = settings["dry_clearance_mm"] / 1000
    if (
        not math.isfinite(thickness)
        or not math.isfinite(clearance)
        or thickness <= 0
        or clearance <= 0
    ):
        raise ValueError(
            "Sheet thickness and dry electrode clearance must be finite and positive"
        )
    if settings["sheet_count"] != 2:
        raise ValueError("This fixture holds two overlapping sheets")
    gap = 2 * thickness + 2 * clearance
    open_gap = settings["open_gap_mm"] / 1000
    if not gap < open_gap <= 0.7 * math.sin(math.radians(20)):
        raise ValueError(
            "Open gap must exceed the dry sheet stack and stay in the model's 20-degree window"
        )
    closed_q, open_q = math.asin(gap / 0.7), math.asin(open_gap / 0.7)
    arm = bt.Robot.from_catalog("kawasaki/bx/bx250l-b001/r2")
    gun = bt.Robot.from_catalog("nimak/multiframegun/95-020-516-p3u/r4")
    if gun.joint_limits[0] is not None:
        open_q = min(open_q, gun.joint_limits[0][1])
    scene = bt.Scene(arm)
    bt.mounting.preview(scene, arm.attach_tool(gun, prefix="gun_")).apply()
    robot = scene.robot
    jaw = robot.joint_names.index("gun_jaw_opening")
    quat = scene.link_pose("gun_cad_tip")[1]
    seed = [0, -0.266, -0.215, 0, 0.215, 0, open_q]

    def pose(x: float, y: float) -> list[float]:
        ik = robot.ik((x, y, 1.7), quat, link="gun_cad_tip", seed=seed, max_iters=200)
        if not ik.converged:
            raise ValueError(f"Cannot reach the electrode target at {x}, {y}")
        q = list(ik.q)
        q[jaw] = open_q
        return q

    approach = pose(-0.025, 3.0)
    scene.set_joint_positions(approach)
    for n, x in enumerate((-0.025, 0.025), 1):
        scene.add_segment(f"spot_{n}", pose(x, 3.1), kind="cartesian_line")
    scene.add_segment("withdraw", approach, kind="cartesian_line")
    for index, name in enumerate((WORKPIECE, "fixture/upper_sheet")):
        scene.add_box(
            name,
            (0.32, 0.07, thickness),
            (0, 3.1, 1.7 + clearance + (index + 0.5) * thickness),
            color=(0.58 + index * 0.12, 0.62 + index * 0.12, 0.67 + index * 0.12),
        )
        scene.set_part(
            name,
            model="Steel overlap coupon",
            category="workpiece",
            attributes={
                "material": settings["material"],
                "thickness_mm": settings["sheet_thickness_mm"],
            },
        )
    for x, label in ((-0.14, "left"), (0.14, "right")):
        height = 1.7 + clearance
        scene.add_box(
            f"fixture/{label}_support",
            (0.025, 0.06, height),
            (x, 3.1, height / 2),
            color=(0.26, 0.33, 0.39),
        )
        scene.add_box(
            f"fixture/{label}_clamp",
            (0.035, 0.025, 0.016),
            (x, 3.1, height + 2 * thickness + 0.008),
            color=(0.86, 0.50, 0.10),
        )
    profile["robot"] = scene.robots[0]
    bt.process.configure(scene, WORKPIECE, profile)
    setup_ok = not any(
        c["status"] == "fail" for c in bt.process.report(scene, WORKPIECE).checks
    )
    for name, initial in (
        ("sim_cooling_ok", True),
        ("sim_drive_ready", True),
        ("sim_setup_ok", setup_ok),
        ("sim_work_clamped", True),
        ("weld_request_simulated", False),
        ("cycle_inhibited", False),
    ):
        scene.define_signal(name, initial=initial)
    scene.add_scenario("cooling_missing", signals={"sim_cooling_ok": False})
    sq = scene.sequence(CYCLE)
    branch = sq.select("start_permissives")
    valid = branch.when(
        bt.seq.all_of(
            *(
                bt.seq.signal(n)
                for n in (
                    "sim_setup_ok",
                    "sim_cooling_ok",
                    "sim_drive_ready",
                    "sim_work_clamped",
                )
            )
        )
    )
    for n in (1, 2):
        valid.step(f"approach_spot_{n}", actions=[bt.seq.motion(f"spot_{n}")])
        valid.step(
            f"close_dry_{n}",
            actions=[
                bt.seq.ramp({"gun_jaw_opening": closed_q}, (open_q - closed_q) / 0.2)
            ],
        )
        valid.step(f"squeeze_dwell_{n}", transition=bt.seq.elapsed(0.2))
        valid.step(
            f"pulse_simulated_{n}",
            actions=[bt.seq.set_signal("weld_request_simulated")],
            transition=bt.seq.elapsed(settings["weld_time_s"]),
        )
        valid.step(
            f"hold_{n}",
            actions=[bt.seq.set_signal("weld_request_simulated", False)],
            transition=bt.seq.elapsed(settings["hold_time_s"]),
        )
        valid.step(
            f"open_{n}",
            actions=[
                bt.seq.ramp({"gun_jaw_opening": open_q}, (open_q - closed_q) / 0.2)
            ],
        )
    valid.step("withdraw", actions=[bt.seq.motion("withdraw")])
    branch.when(bt.seq.otherwise()).step(
        "inhibited", actions=[bt.seq.set_signal("cycle_inhibited")]
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
    timeline = scene.simulate_sequence(CYCLE)
    report = bt.process.report(scene, WORKPIECE)
    print(
        f"Two-position dry cycle: {timeline.duration:.2f} s; no welding current applied"
    )
    print(report.to_markdown())
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        scene.save_project(args.output / "cell.botrail")
        report.save(args.output / "process.json")
        report.save(args.output / "process.md")
        (args.output / "setup.json").write_text(
            json.dumps(bt.process.setup(scene, WORKPIECE), indent=2) + "\n"
        )
        scene.export_bom(args.output / "bom.csv")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
