"""NIMAK 95.020.516/P3U: two-position spot-welding dry cycle on a BX250L.

Two overlapping steel sheets on a fixture; the gun approaches each spot,
closes to a positive electrode clearance on either side of the stack,
raises a simulated weld request for the weld time, holds, opens and
withdraws. Force, current and weld formation are not simulated; the jaw
and the arm move through the real catalog kinematic tree, and every
gun/workpiece collision check stays enabled. The cycle starts only on
its permissives (cooling, drive, clamp — simulated inputs, not verified
hardware feedback); the `cooling_missing` scenario takes the inhibited
branch instead.

Run with:  python examples/welding/nimak_spot_welding_demo.py [--studio]
                 [--output DIR]
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import botrail as bt

ARM = "kawasaki/bx/bx250l-b001/r2"
GUN = "nimak/multiframegun/95-020-516-p3u/r4"
WORKPIECE = "fixture/lower_sheet"
CYCLE = "spot_weld_dry_cycle"

# --- the coupon and the dry cycle (demo choices, not a qualified WPS) ----
MATERIAL = "Mild steel, grade unspecified (demo)"
SHEET_THICKNESS_MM = 1.0  # two overlapping 1 mm sheets
DRY_CLEARANCE_MM = 1.0  # electrode-to-sheet gap kept on each side while "closed"
OPEN_GAP_MM = 80.0  # projected tip separation when the jaw is open
WELD_TIME_S = 0.2  # simulated request pulse; no current is applied
HOLD_TIME_S = 0.2
SQUEEZE_DWELL_S = 0.2

# --- the gun model's jaw (simulation settings, not manufacturer limits) --
TIP_RADIUS_M = 0.7  # jaw pivot to electrode tip in the reference model
JAW_RATE = 0.2  # rad/s
WORK_Z = 1.7  # electrode contact height of the fixture, metres


def jaw_angle(gap_m: float) -> float:
    """Jaw rotation that projects to a tip separation of `gap_m`."""
    return math.asin(gap_m / TIP_RADIUS_M)


def build() -> bt.Scene:
    """The arm with the gun, the coupon on its fixture, and the dry cycle."""
    thickness = SHEET_THICKNESS_MM / 1000
    clearance = DRY_CLEARANCE_MM / 1000
    closed_q = jaw_angle(2 * thickness + 2 * clearance)
    open_q = jaw_angle(OPEN_GAP_MM / 1000)

    arm = bt.Robot.from_catalog(ARM)
    gun = bt.Robot.from_catalog(GUN)
    if gun.joint_limits[0] is not None:
        open_q = min(open_q, gun.joint_limits[0][1])
    scene = bt.Scene(arm.attach_tool(gun, prefix="gun_"))
    robot = scene.robot
    jaw = robot.joint_names.index("gun_jaw_opening")
    quat = scene.link_pose("gun_cad_tip")[1]
    seed = [0, -0.266, -0.215, 0, 0.215, 0, open_q]

    def pose(x: float, y: float) -> list[float]:
        ik = robot.ik((x, y, WORK_Z), quat, link="gun_cad_tip", seed=seed, max_iters=200)
        if not ik.converged:
            raise ValueError(f"Cannot reach the electrode target at {x}, {y}")
        q = list(ik.q)
        q[jaw] = open_q
        return q

    # -- motions: approach each spot with the jaw open, then withdraw ------
    approach = pose(-0.025, 3.0)
    scene.set_joint_positions(approach)
    for n, x in enumerate((-0.025, 0.025), 1):
        scene.add_segment(f"spot_{n}", pose(x, 3.1), kind="cartesian_line")
    scene.add_segment("withdraw", approach, kind="cartesian_line")

    # -- the coupon: two sheets stacked between the electrodes -------------
    for index, name in enumerate((WORKPIECE, "fixture/upper_sheet")):
        scene.add_box(
            name,
            (0.32, 0.07, thickness),
            (0, 3.1, WORK_Z + clearance + (index + 0.5) * thickness),
            color=(0.58 + index * 0.12, 0.62 + index * 0.12, 0.67 + index * 0.12),
        )
        scene.set_part(
            name,
            model="Steel overlap coupon",
            category="workpiece",
            attributes={"material": MATERIAL, "thickness_mm": SHEET_THICKNESS_MM},
        )
    for x, label in ((-0.14, "left"), (0.14, "right")):
        height = WORK_Z + clearance
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

    # -- the program: start permissives, then two dry spots ----------------
    for name, initial in (
        ("sim_cooling_ok", True),
        ("sim_drive_ready", True),
        ("sim_work_clamped", True),
        ("weld_request_simulated", False),
        ("cycle_inhibited", False),
    ):
        scene.define_signal(name, initial=initial)
    scene.add_scenario("cooling_missing", signals={"sim_cooling_ok": False})
    close_time = (open_q - closed_q) / JAW_RATE
    sq = scene.sequence(CYCLE)
    branch = sq.select("start_permissives")
    valid = branch.when(
        bt.seq.all_of(
            bt.seq.signal("sim_cooling_ok"),
            bt.seq.signal("sim_drive_ready"),
            bt.seq.signal("sim_work_clamped"),
        )
    )
    for n in (1, 2):
        valid.step(f"approach_spot_{n}", actions=[bt.seq.motion(f"spot_{n}")])
        valid.step(
            f"close_dry_{n}",
            actions=[bt.seq.ramp({"gun_jaw_opening": closed_q}, close_time)],
        )
        valid.step(f"squeeze_dwell_{n}", transition=bt.seq.elapsed(SQUEEZE_DWELL_S))
        valid.step(
            f"pulse_simulated_{n}",
            actions=[bt.seq.set_signal("weld_request_simulated")],
            transition=bt.seq.elapsed(WELD_TIME_S),
        )
        valid.step(
            f"hold_{n}",
            actions=[bt.seq.set_signal("weld_request_simulated", False)],
            transition=bt.seq.elapsed(HOLD_TIME_S),
        )
        valid.step(
            f"open_{n}",
            actions=[bt.seq.ramp({"gun_jaw_opening": open_q}, close_time)],
        )
    valid.step("withdraw", actions=[bt.seq.motion("withdraw")])
    branch.when(bt.seq.otherwise()).step(
        "inhibited", actions=[bt.seq.set_signal("cycle_inhibited")]
    )
    return scene


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--output", type=Path, help="directory for the project and the BOM")
    args = parser.parse_args()

    scene = build()
    timeline = scene.simulate_sequence(CYCLE)
    print(f"Two-position dry cycle: {timeline.duration:.2f} s; no welding current applied")
    for name, t0, t1 in timeline.step_spans:
        print(f"  {name:<24} {t0:6.2f} - {t1:6.2f} s")
    if args.output:
        args.output.mkdir(parents=True, exist_ok=True)
        scene.save_project(args.output / "cell.botrail")
        scene.export_bom(args.output / "bom.csv")
        print("Saved:", args.output)
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
