"""An AMR working two stations: pick here, drive there, place.

The arm is bolted to the MiR250's deck instead of a pedestal, which changes
one thing and one thing only — its base is no longer a scene constant. From
that follow the two rules this demo is really about:

  * **A planned motion cannot start while the machine is driving.** Plans
    are baked in world coordinates when they start, so a base that moves
    underneath one invalidates every waypoint. The rollout rejects it by
    name rather than quietly producing nonsense (`--drive-and-plan` shows
    the error).
  * **A ramp can.** Ramps are re-evaluated every scan tick, so the arm can
    fold itself away *while* the vehicle travels — which is exactly what a
    real AMR does between stations, and why the stow costs no cycle time.

Everything else — the tray, the aisle check, the interlock vocabulary — is
the same as the AGV cell; the arm just happens to be riding.

Run with:  python examples/amr_demo.py [out.usda] [--drive-and-plan]
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import botrail as bt  # noqa: E402
from agv_cell_demo import (  # noqa: E402
    BLACK, BODY_DX, DECK_TOP, LASER_DX, LASER_DY, LASER_DZ, MIR_WHITE,
    SPEED, TURN, WHEEL_DY, WHEEL_R, fetch_mir250, yaw_flip,
)
from demo import fetch_franka  # noqa: E402

# Two stations on the open floor south of the cell, on the painted walkway:
# the AMR is a machine that brings its own arm, so it needs no cell at all.
PICK = (-2.60, -2.90)
PLACE = (0.90, -2.90)

# The arm sits on the deck, facing the machine's left so its work envelope
# hangs over the side rather than over its own chassis.
ARM_MOUNT = (BODY_DX, 0.0, DECK_TOP)
READY = [0.0, -0.6, 0.0, -2.2, 0.0, 1.8, 0.785, 0.035, 0.035]
# Folded down over the deck: what the arm does between stations.
STOWED = [0.0, -1.4, 0.0, -2.7, 0.0, 1.5, 0.785, 0.035, 0.035]

BOX_SIZE = 0.06
OPEN, CLOSED = 0.039, 0.029
HAND = "/panda/panda_hand"
PADS = ["/panda/panda_leftfinger", "/panda/panda_rightfinger"]
HOVER = 0.12
STAND_TOP = 0.40  # the two work stands the parts sit on
# How far off the lane the stands sit — within the arm's reach from the deck.
STAND_OFFSET = 0.65
CARTON = "carton"


def build_scene() -> bt.Scene:
    """The AMR — a MiR250 carrying a Franka — plus the two work stands.

    Shared with `play_record.py`, which rebuilds the cell a recording was
    baked from."""
    meshes = fetch_mir250()
    robot = bt.Robot.from_usd(fetch_franka())
    scene = bt.Scene(robot, name="panda")

    # The vehicle, parked at the pick station facing the place station.
    x, y = PICK
    scene.add_mesh("agv/base", meshes / "mir_250_base.stl",
                   (x + BODY_DX, y, 0.0), color=MIR_WHITE)
    lie = (math.sin(math.pi / 4), 0.0, 0.0, math.cos(math.pi / 4))
    for side, name in ((1, "l"), (-1, "r")):
        wheel = f"agv/wheel_{name}"
        scene.add_cylinder(wheel, WHEEL_R, 0.038, (x, y + side * WHEEL_DY, WHEEL_R),
                           quaternion=lie, color=BLACK)
        scene.set_obstacle_enabled(wheel, False)
    for dx, dy, yaw, name in ((LASER_DX, LASER_DY, 0.25 * math.pi, "front"),
                              (-LASER_DX, -LASER_DY, -0.75 * math.pi, "back")):
        laser = f"agv/scanner_{name}"
        scene.add_mesh(laser, meshes / "sick_lms-100.stl",
                       (x + dx + BODY_DX, y + dy, LASER_DZ),
                       quaternion=yaw_flip(yaw), color=BLACK)
        scene.set_obstacle_enabled(laser, False)

    scene.add_vehicle(
        "amr",
        body=["agv"],
        path=[PICK, PLACE],
        stations={"pick": 0, "place": 1},
        speed=SPEED,
        turn_speed=TURN,
        start="pick",
        allow_reverse=True,
    )
    # The arm rides the deck: from here its base is derived, not placed.
    scene.mount_robot("amr", offset_position=ARM_MOUNT)
    scene.set_joint_positions(READY)

    # A work stand at each station, and the part waiting on the first one.
    # They stand *beside* the lane, not in it: the machine drives along +x
    # and the arm works over its own side, which is how a mobile
    # manipulator serves a row of stations without leaving the aisle.
    for (sx, sy), name in ((PICK, "stand_pick"), (PLACE, "stand_place")):
        scene.add_box(name, (0.30, 0.30, STAND_TOP),
                      (sx, sy + STAND_OFFSET, STAND_TOP / 2), color=(0.10, 0.10, 0.11))
    scene.add_box(CARTON, (BOX_SIZE,) * 3,
                  (PICK[0], PICK[1] + STAND_OFFSET, STAND_TOP + BOX_SIZE / 2),
                  color=(0.35, 0.16, 0.05))
    return scene


def build_cycle(scene: bt.Scene, drive_and_plan: bool = False) -> str:
    """Teaches the poses and writes the two-station cycle."""
    names = scene.robot.joint_names
    fingers = [n for n in names if "panda_finger_joint" in n]
    down = (1.0, 0.0, 0.0, 0.0)  # tool +Z straight down

    def teach(position, standoff: float = 0.0) -> list:
        target = (position[0], position[1], position[2] + 0.10 + standoff)
        ik = scene.set_tcp_target(target, down, link=HAND)
        if not ik.converged:
            raise RuntimeError(
                f"IK missed {tuple(round(v, 3) for v in target)}: "
                f"{ik.pos_error * 1e3:.1f} mm short"
            )
        return list(scene.joint_positions)

    def with_fingers(q: list, width: float) -> list:
        q = list(q)
        for f in fingers:
            q[names.index(f)] = width
        return q

    # Both stands are taught at the *pick* station — the arm's base is the
    # deck, so the place stand is at the same spot in the machine's own
    # frame once it has driven there. That is the whole point of a mobile
    # manipulator, and it is why one taught pose serves both stations.
    part = (PICK[0], PICK[1] + STAND_OFFSET, STAND_TOP + BOX_SIZE / 2)
    hover_q = teach(part, HOVER)
    grasp_q = teach(part)
    scene.set_joint_positions(READY)

    scene.add_segment("to_part", goal=with_fingers(hover_q, OPEN))
    scene.add_segment("to_part", goal=with_fingers(grasp_q, OPEN))
    scene.add_segment("lift", goal=with_fingers(hover_q, CLOSED))
    scene.add_segment("to_stand", goal=with_fingers(hover_q, CLOSED))
    scene.add_segment("to_stand", goal=with_fingers(grasp_q, CLOSED))
    scene.add_segment("clear", goal=with_fingers(hover_q, OPEN))

    sq = scene.sequence("amr_transfer")
    sq.step("接近", actions=[bt.seq.motion("to_part")])
    sq.step("グリッパ閉", actions=[bt.seq.ramp({f: CLOSED for f in fingers}, 0.4)])
    sq.step("把持", actions=[bt.seq.attach(CARTON, link=HAND, touch_links=PADS)])
    sq.step("持上げ", actions=[bt.seq.motion("lift")])
    if drive_and_plan:
        # The error this demo exists to show: a plan cannot be baked against
        # a base that is about to move.
        sq.step("走行(誤)", actions=[bt.seq.goto("amr", "place"),
                                      bt.seq.motion("to_stand")])
    else:
        # The right way round: start driving, and *ramp* the arm into its
        # stow while it travels. The fold costs no cycle time at all.
        sq.step(
            "走行",
            actions=[bt.seq.goto("amr", "place"),
                     bt.seq.ramp(dict(zip(names, STOWED)), 1.2)],
            transition=bt.seq.device_done("amr"),
        )
        sq.step("載置", actions=[bt.seq.motion("to_stand")])
        sq.step("開放", actions=[bt.seq.ramp({f: OPEN for f in fingers}, 0.4)])
        sq.step("離脱", actions=[bt.seq.detach(CARTON)])
        sq.step("退避", actions=[bt.seq.motion("clear")])
    return sq.name


def main() -> None:
    drive_and_plan = "--drive-and-plan" in sys.argv[1:]
    out = next((a for a in sys.argv[1:] if not a.startswith("--")), "cell_amr.usda")

    scene = build_scene()
    name = build_cycle(scene, drive_and_plan)
    try:
        tl = scene.simulate_sequence(name, max_duration=90.0)
    except ValueError as err:
        print(f"cycle failed: {err}")
        sys.exit(1)

    print(f"cycle time: {tl.duration:.2f}s")
    for step, start, end in tl.step_spans:
        print(f"  {step:<8} {start:6.2f} – {end:6.2f}s")
    # The base is a track now, not a constant.
    for t in (0.0, tl.duration):
        p, _ = tl.base_pose(t)
        print(f"  arm base at {t:5.2f}s: {tuple(round(v, 3) for v in p)}")
    moved = tl.object_pose(CARTON, tl.duration)[0]
    print(f"carton ends at {tuple(round(v, 3) for v in moved)} "
          f"(picked at {tuple(round(v, 2) for v in (PICK[0], PICK[1] + STAND_OFFSET))})")

    tl.export_usd(out, fps=60)
    print(f"wrote {out}")
    print(f"  replay it with:  python examples/play_record.py {out}")


if __name__ == "__main__":
    main()
