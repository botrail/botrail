"""Two Frankas sharing one infeed, arbitrated by a zone interlock.

The cell has a single pick point on the belt and two arms facing each other
across it, each palletising to its own pallet. Doubling the arms only pays
off because the *transfer* dominates the cycle: while one arm is carrying a
carton to its pallet, the other is already tracking the next one down the
belt.

That is also what makes the arms dangerous to each other. Over the pick
point their envelopes coincide — at both grasps the two hands are in the
same place — so the cell needs an interlock, and it is written the way a PLC
writes one: a zone around the contested airspace, one sensor per arm, and a
step that will not proceed while the other arm's zone signal is on.

Nothing here plans the two arms together. Each is planned on its own with
the other frozen as an obstacle, and the rollout re-checks arm-against-arm
every tick — so a missing interlock is not a silent near-miss, it is a hard
`RobotCollision` with a timestamp. Run with `--clash` to see exactly that.

Run with:  python examples/dual_cell_demo.py [out.usda] [--clash]
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import botrail as bt  # noqa: E402
from demo import build_scene, teach_grasp  # noqa: E402

NEAR, FAR = "near", "far"
BOX_NEAR = "/World/Conveyor/Box_A"
BOX_FAR = "/World/Conveyor/Box_B"

# Finger stroke and hover height, as in the single-arm demo.
OPEN, CLOSED = 0.039, 0.029
HOVER = 0.15
PADS = ["/panda/panda_leftfinger", "/panda/panda_rightfinger"]
BEAM_RADIUS = 0.005
BOX_SIZE = 0.06

READY = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.035, 0.035]

# Line pacing. The photo-eye sits *upstream* of the pick point, the way a
# real cell does it: tripping it is the arm's cue to come over, and the
# carton needs to still be arriving when the arm gets there. The approach is
# a planned move from the pallet side and takes ~5.7 s, so at 0.05 m/s the
# eye goes 300 mm before the station.
#
# The carton spacing is the other half of the pacing. A beam is a momentary
# signal: if the second carton crosses it while the first arm is still over
# the pick point, the arm waiting on `all_of(beam, other arm clear)` never
# sees both at once and the cell deadlocks. So the cartons are spaced by
# more than one arm's pick-and-clear.
BELT_SPEED = 0.05
BEAM_LEAD = 0.30
FAR_START, NEAR_START = -0.75, -1.30


def build_cell() -> bt.Scene:
    """The factory cell with both arms standing on their pedestals."""
    scene = build_scene(name=NEAR)  # near arm on /World/MountFrame
    scene.add_robot(
        scene.robot,
        name=FAR,
        base_position=scene.frame("/World/MountFrameFar")[0],
        # Facing back across the belt.
        base_quaternion=(0.0, 0.0, 1.0, 0.0),
    )
    scene.set_joint_positions(READY, robot=FAR)
    return scene


def teach(scene: bt.Scene, robot: str, pick, place) -> dict:
    """The four poses each arm works between, solved hover-first so the
    grasp warm-starts from the pose right above it."""
    poses = {}
    scene.set_joint_positions(READY, robot=robot)
    poses["hover"] = teach_grasp(scene, pick, standoff=HOVER, robot=robot)
    poses["grasp"] = teach_grasp(scene, pick, robot=robot)
    # Back to ready before the pallet: it is a wide base swing from the belt,
    # and warm-starting across it walks the solver into a local minimum.
    scene.set_joint_positions(READY, robot=robot)
    poses["drop"] = teach_grasp(scene, place, standoff=HOVER, robot=robot)
    poses["place"] = teach_grasp(scene, place, robot=robot)
    scene.set_joint_positions(READY, robot=robot)
    return poses


def build_cycle(scene: bt.Scene, interlocked: bool = True) -> str:
    names = scene.robot.joint_names
    fingers = [n for n in names if "panda_finger_joint" in n]
    pick = scene.frame("/World/Conveyor/PickFrame")

    poses = {
        FAR: teach(scene, FAR, pick, scene.frame("/World/PalletFar/PlaceFrame")),
        NEAR: teach(scene, NEAR, pick, scene.frame("/World/Pallet/PlaceFrame")),
    }

    # ---- the line: two cartons, the far arm's one leading ----------------
    scene.set_obstacle_pose(BOX_FAR, (FAR_START, pick[0][1], pick[0][2]))
    scene.set_obstacle_pose(BOX_NEAR, (NEAR_START, pick[0][1], pick[0][2]))
    scene.add_conveyor(
        "conv",
        zone_position=(-0.6, 0.62, 0.60),
        zone_size=(1.8, 0.4, 0.14),
        velocity=(BELT_SPEED, 0.0, 0.0),
        running=False,
    )
    # Two photo-eyes, as a real line has: one upstream to call an arm over
    # while the carton is still travelling, one on the station itself. The
    # second is what the tracking latch keys off — `track` records where the
    # carton is *at that instant* and rides the taught poses off it, so
    # latching early means grasping the carton off-centre by however far it
    # still had to come. (Latching on the upstream eye alone put the near
    # arm's grip 120 mm off, and the carton landed 120 mm off the pallet.)
    for name, x in (
        ("beam_ahead", pick[0][0] - BEAM_LEAD),
        ("beam_pick", pick[0][0]),
    ):
        trip_x = x + BOX_SIZE / 2 + BEAM_RADIUS
        scene.add_beam_sensor(
            name,
            frm=(trip_x, 0.42, pick[0][2]),
            to=(trip_x, 0.82, pick[0][2]),
            radius=BEAM_RADIUS,
            watch=[BOX_NEAR, BOX_FAR],
        )

    # ---- the interlock: one zone per arm over the contested airspace -----
    # A zone reports "somebody is inside", not who, so each arm needs its
    # own sensor over the same volume — one zone watching both would be
    # tripped by the very arm waiting on it.
    for robot in (NEAR, FAR):
        scene.add_zone_sensor(
            f"zone_{robot}",
            position=(pick[0][0], pick[0][1], pick[0][2] + 0.17),
            size=(0.5, 0.5, 0.6),
            watch_robots=[robot],
        )

    # ---- motions --------------------------------------------------------
    def with_fingers(q: list, width: float) -> list:
        q = list(q)
        for f in fingers:
            q[names.index(f)] = width
        return q

    for robot in (NEAR, FAR):
        scene.add_segment(
            f"{robot}_to_pick", goal=with_fingers(poses[robot]["hover"], OPEN), robot=robot
        )
        scene.add_segment(
            f"{robot}_to_pallet", goal=with_fingers(poses[robot]["drop"], CLOSED), robot=robot
        )
        scene.add_segment(f"{robot}_home", goal=READY, robot=robot)

    scene.define_signal("carrying_near")
    scene.define_signal("carrying_far")
    ramp_to = lambda q: dict(zip(names, q))  # noqa: E731

    sq = scene.sequence("dual_pick")

    def pick_cycle(robot: str, box: str, gate) -> None:
        """One arm's half: take a carton off the moving belt and start the
        transfer. Ends the moment the transfer is *started*, not finished —
        that is what lets the other arm move in."""
        sq.step(f"{robot}_wait", transition=gate)
        sq.step(f"{robot}_approach", actions=[bt.seq.motion(f"{robot}_to_pick")])
        # Hold over the station until the carton is actually under the
        # gripper; only then does the taught grasp mean what it says.
        sq.step(f"{robot}_arrive", transition=bt.seq.signal("beam_pick"))
        # From here the taught poses ride the carton down the belt.
        sq.step(f"{robot}_latch", actions=[bt.seq.track(box, robot=robot)])
        sq.step(
            f"{robot}_descend",
            actions=[
                bt.seq.ramp(ramp_to(with_fingers(poses[robot]["grasp"], OPEN)), 0.6, robot=robot)
            ],
        )
        sq.step(
            f"{robot}_close",
            actions=[bt.seq.ramp({f: CLOSED for f in fingers}, 0.4, robot=robot)],
        )
        sq.step(
            f"{robot}_grasp",
            actions=[
                # Grasping freezes the belt-sync offset, so the lift goes
                # straight up from wherever the carton was caught.
                bt.seq.attach(box, link="/panda/panda_hand", touch_links=PADS, robot=robot),
                bt.seq.set_signal(f"carrying_{robot}"),
            ],
        )
        sq.step(
            f"{robot}_lift",
            actions=[
                bt.seq.ramp(ramp_to(with_fingers(poses[robot]["hover"], CLOSED)), 0.6, robot=robot)
            ],
        )
        sq.step(
            f"{robot}_carry",
            actions=[bt.seq.untrack(robot=robot), bt.seq.motion(f"{robot}_to_pallet")],
            # Do not wait for the transfer: releasing the sequence here is
            # the whole point — the other arm picks while this one drives.
            transition=bt.seq.immediately(),
        )

    def place_cycle(robot: str) -> None:
        """The other half, run once the transfer has landed."""
        sq.step(f"{robot}_landed", transition=bt.seq.robot_done(robot))
        sq.step(
            f"{robot}_lower",
            actions=[
                bt.seq.ramp(ramp_to(with_fingers(poses[robot]["place"], CLOSED)), 0.8, robot=robot)
            ],
        )
        sq.step(
            f"{robot}_release",
            actions=[bt.seq.detach(box_of(robot)), bt.seq.set_signal(f"carrying_{robot}", False)],
        )
        sq.step(
            f"{robot}_open",
            actions=[bt.seq.ramp({f: OPEN for f in fingers}, 0.4, robot=robot)],
        )
        sq.step(
            f"{robot}_retreat",
            actions=[
                bt.seq.ramp(ramp_to(with_fingers(poses[robot]["drop"], OPEN)), 0.8, robot=robot)
            ],
        )

    def box_of(robot: str) -> str:
        return BOX_FAR if robot == FAR else BOX_NEAR

    # With nothing arbitrating, both arms are free to go at once. Their
    # plans are each made with the other frozen where it stands, so both
    # succeed — and the rollout catches the two of them converging.
    clash = scene.sequence("clash")
    clash.step(
        "both_go",
        actions=[bt.seq.motion("near_to_pick"), bt.seq.motion("far_to_pick")],
        transition=bt.seq.all_of(bt.seq.robot_done(NEAR), bt.seq.robot_done(FAR)),
    )

    sq.step("feed", actions=[bt.seq.start("conv")])
    # The eye starts the cell: the leading carton arriving is what sends the
    # far arm over. From there the two arms hand the station back and forth,
    # and the *only* thing keeping the near arm out is the far arm's zone —
    # which is why `--clash` (which drops exactly that condition) collides.
    pick_cycle(FAR, BOX_FAR, bt.seq.signal("beam_ahead"))
    pick_cycle(
        NEAR,
        BOX_NEAR,
        bt.seq.signal("zone_far", False) if interlocked else bt.seq.immediately(),
    )
    place_cycle(FAR)
    place_cycle(NEAR)
    sq.step(
        "home",
        actions=[bt.seq.motion("near_home"), bt.seq.motion("far_home")],
        transition=bt.seq.all_of(bt.seq.robot_done(NEAR), bt.seq.robot_done(FAR)),
    )
    return sq.name


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    clash = "--clash" in sys.argv
    out = Path(args[0]) if args else Path("cell_dual.usda")

    scene = build_cell()
    name = build_cycle(scene, interlocked=not clash)

    if clash:
        # Two independent guards catch an unarbitrated cell, and it is worth
        # seeing both. Planning is the first: an arm cannot plan into a pose
        # the other one is standing in, because the other is frozen as an
        # obstacle while the plan is made.
        try:
            scene.simulate_sequence(name)
            raise SystemExit("expected the unarbitrated cell to fail")
        except ValueError as e:
            print(f"1. the planner refuses to enter an occupied station:\n   {e}\n")

        # The rollout is the second, and the one that matters for arms that
        # each planned successfully: it re-checks arm against arm every
        # tick, so converging on the same place is a timestamped failure
        # rather than a near-miss nobody notices.
        try:
            scene.simulate_sequence("clash")
            raise SystemExit("expected a robot-robot collision")
        except ValueError as e:
            print(f"2. and the rollout catches two valid plans converging:\n   {e}")
        return

    timeline = scene.simulate_sequence(name)
    print(f"cycle time: {timeline.duration:.2f}s")
    for robot in timeline.robots:
        busy = sum(end - start for _, start, end in timeline.moves(robot))
        print(f"  {robot:5s} moving {busy:5.2f}s of {timeline.duration:.2f}s")

    # What the second arm bought: the stretch where both are in motion.
    spans = {step: (start, end) for step, start, end in timeline.step_spans}
    far_carry = spans["far_carry"][0]
    near_lift = spans["near_lift"][1]
    print(f"both arms in motion from {far_carry:.2f}s to {near_lift:.2f}s")

    warnings = timeline.export_usd(out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported to {out} — view with: usdview {out}")


if __name__ == "__main__":
    main()
