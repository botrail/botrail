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

# The line runs on a magazine, not on hand-placed boxes. A baked timeline
# holds a fixed set of named object tracks, so "endless supply" is a finite
# pool plus a sink that hands carriers back to the source — which is also
# what a real accumulation line is. The cell ships with four cartons on the
# belt; the demo tops the pool up to `CARTONS`.
CARTONS = 6
CARTON = [f"/World/Conveyor/Box_{c}" for c in "ABCD"] + [
    f"/World/Conveyor/Carton_{i}" for i in (4, 5)
]
# Cleats on the belt surface, on the same loop. They are what makes the belt
# read as *moving* rather than as a static slab with two boxes on it, and
# they cost nothing new: the conveyor advects any unattached obstacle in its
# zone and does not care whether collision is on, so they ride with
# collision off and the arms never see them.
CLEATS = 14
CLEAT = [f"/World/Conveyor/Cleat_{i}" for i in range(CLEATS)]

# Cartons stacked per pallet. The pallet is what makes the cycle finite:
# supply is endless, stack height is not.
CYCLES = 2

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
BELT_SPEED = 0.15
# The upstream eye has to sit at least one approach-move ahead of the
# station: the arm takes ~5.7 s to come over, and a carton that reaches the
# station first has already gone by the time the arm looks for it — the
# `beam_pick` wait then never fires. At 0.15 m/s that is 0.86 m, so 1.0 m
# with margin.
BEAM_LEAD = 1.0
# Carton spacing on the belt, and where the line begins and ends. The belt
# now runs out through the opening in the west guard, so `BELT_IN` is
# *outside* the cell: a called-for carton travels in rather than appearing
# on the belt in front of you, which is the one thing a magazine gives
# itself away by.
CARTON_PITCH = 0.60
BELT_IN, BELT_OUT = -2.25, 1.30
# The magazine is under the floor. Stowed carriers are not drawn during
# playback, so where they wait does not matter to the recording — but the
# *live* scene has no timeline behind it, and a stack of stock hovering
# beside the belt is exactly the thing that gives the trick away.
MAGAZINE = (-1.75, 0.62, -0.45)


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


def magazine_slot(i: int):
    """Where pool member `i` waits — stacked, so the queue reads as a stack
    of cartons rather than one carton in N places."""
    return (MAGAZINE[0], MAGAZINE[1], MAGAZINE[2] - (BOX_SIZE + 0.01) * i)


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

    places = {
        FAR: scene.frame("/World/PalletFar/PlaceFrame"),
        NEAR: scene.frame("/World/Pallet/PlaceFrame"),
    }
    poses = {r: teach(scene, r, pick, places[r]) for r in (FAR, NEAR)}

    # One taught pose per course of the stack. Solved from the arm's own
    # hover-over-the-pallet pose so every course stays in one posture
    # family, the same reason the four base poses are solved hover-first.
    def course(robot: str, layer: int) -> list:
        (x, y, z), quat = places[robot]
        scene.set_joint_positions(poses[robot]["drop"], robot=robot)
        return teach_grasp(scene, ((x, y, z + BOX_SIZE * layer), quat), robot=robot)

    stack = {r: [course(r, layer) for layer in range(CYCLES)] for r in (FAR, NEAR)}
    for r in (FAR, NEAR):
        scene.set_joint_positions(READY, robot=r)

    # ---- the line: a carton magazine and a cleated belt, both on loops ---
    belt_y, belt_z = pick[0][1], pick[0][2]
    # Every carton starts in the magazine and is called for one at a time.
    # That is not decoration: the steps name the carton each arm takes, so
    # *pool order has to be arrival order*, and the only way to guarantee
    # that is to feed on demand. Pre-loading the belt inverts the order
    # outright — whichever carton is nearest the station arrives first,
    # whatever its index — and a timer feed drifts out of it the first time
    # a carton passes while the cell is busy elsewhere.
    for i, name in enumerate(CARTON):
        if name not in scene.obstacle_names:
            scene.add_box(name, (BOX_SIZE,) * 3, (0, 0, 0), color=(0.527, 0.254, 0.076))
        scene.set_obstacle_pose(name, magazine_slot(i))
    for i, name in enumerate(CLEAT):
        # Thin slats across the belt, sitting on its surface.
        scene.add_box(name, (0.02, 0.40, 0.012), (0, 0, 0), color=(0.09, 0.102, 0.122))
        # Collision off: they are scenery that happens to move, and the
        # gripper reaches right through where they pass.
        scene.set_obstacle_enabled(name, False)
        scene.set_obstacle_pose(
            name, (BELT_IN + (BELT_OUT - BELT_IN) * i / CLEATS, belt_y, belt_z - 0.024)
        )

    # The transport zone has to span the whole run now: it is what carries
    # carriers into the sink at the far end, not just past the stations.
    scene.add_conveyor(
        "conv",
        zone_position=((BELT_IN + BELT_OUT) / 2, belt_y, 0.60),
        zone_size=(BELT_OUT - BELT_IN + 0.2, 0.4, 0.14),
        velocity=(BELT_SPEED, 0.0, 0.0),
        running=False,
    )
    # One sink for the whole line end; each source gets its own so a carton
    # never comes back as a cleat.
    scene.add_source(
        "cartons",
        pool=CARTON,
        park=MAGAZINE,
        pitch=(0.0, 0.0, -(BOX_SIZE + 0.01)),
        # Outside the guard: the carton is only ever seen travelling.
        position=(BELT_IN, belt_y, belt_z),
        # An indexing feeder: one carton per `start`, so pool order is
        # arrival order no matter how the cell is paced.
        interval=0.0,
        running=False,
    )
    scene.add_source(
        "cleats",
        pool=CLEAT,
        park=(MAGAZINE[0], MAGAZINE[1], MAGAZINE[2]),
        pitch=(0.0, 0.0, 0.0),
        position=(BELT_IN, belt_y, belt_z - 0.024),
        interval=(BELT_OUT - BELT_IN) / CLEATS / BELT_SPEED,
        running=False,
    )
    for name, source, z in (
        ("carton_out", "cartons", belt_z),
        ("cleat_out", "cleats", belt_z - 0.024),
    ):
        scene.add_sink(
            name,
            zone_position=(BELT_OUT, belt_y, z),
            zone_size=(0.12, 0.4, 0.05),
            source=source,
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
            watch=list(CARTON),
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

    def pick_cycle(robot: str, box: str, gate, tag: str = "") -> None:
        """One arm's half: take a carton off the moving belt and start the
        transfer. Ends the moment the transfer is *started*, not finished —
        that is what lets the other arm move in."""
        # Call for the next carton as this half begins: it travels while
        # the arm gets itself over the station.
        sq.step(
            f"{robot}_wait{tag}",
            actions=[bt.seq.start("cartons")],
            transition=gate,
        )
        sq.step(f"{robot}_approach{tag}", actions=[bt.seq.motion(f"{robot}_to_pick")])
        # Hold over the station until the carton is actually under the
        # gripper; only then does the taught grasp mean what it says.
        sq.step(f"{robot}_arrive{tag}", transition=bt.seq.signal("beam_pick"))
        # From here the taught poses ride the carton down the belt.
        sq.step(f"{robot}_latch{tag}", actions=[bt.seq.track(box, robot=robot)])
        sq.step(
            f"{robot}_descend{tag}",
            actions=[
                bt.seq.ramp(ramp_to(with_fingers(poses[robot]["grasp"], OPEN)), 0.6, robot=robot)
            ],
        )
        sq.step(
            f"{robot}_close{tag}",
            actions=[bt.seq.ramp({f: CLOSED for f in fingers}, 0.4, robot=robot)],
        )
        sq.step(
            f"{robot}_grasp{tag}",
            actions=[
                # Grasping freezes the belt-sync offset, so the lift goes
                # straight up from wherever the carton was caught.
                bt.seq.attach(box, link="/panda/panda_hand", touch_links=PADS, robot=robot),
                bt.seq.set_signal(f"carrying_{robot}"),
            ],
        )
        sq.step(
            f"{robot}_lift{tag}",
            actions=[
                bt.seq.ramp(ramp_to(with_fingers(poses[robot]["hover"], CLOSED)), 0.6, robot=robot)
            ],
        )
        sq.step(
            f"{robot}_carry{tag}",
            actions=[bt.seq.untrack(robot=robot), bt.seq.motion(f"{robot}_to_pallet")],
            # Do not wait for the transfer: releasing the sequence here is
            # the whole point — the other arm picks while this one drives.
            transition=bt.seq.immediately(),
        )

    def place_cycle(robot: str, box: str, layer: int, tag: str) -> None:
        """The other half, run once the transfer has landed. `layer` is the
        course on the pallet — each cycle sets its carton a box-height
        higher, which is what makes a finite number of cycles the natural
        end of the run."""
        sq.step(f"{robot}_landed{tag}", transition=bt.seq.robot_done(robot))
        sq.step(
            f"{robot}_lower{tag}",
            actions=[
                bt.seq.ramp(
                    ramp_to(with_fingers(stack[robot][layer], CLOSED)), 0.8, robot=robot
                )
            ],
        )
        sq.step(
            f"{robot}_release{tag}",
            actions=[bt.seq.detach(box), bt.seq.set_signal(f"carrying_{robot}", False)],
        )
        sq.step(
            f"{robot}_open{tag}",
            actions=[bt.seq.ramp({f: OPEN for f in fingers}, 0.4, robot=robot)],
        )
        sq.step(
            f"{robot}_retreat{tag}",
            actions=[
                bt.seq.ramp(ramp_to(with_fingers(poses[robot]["drop"], OPEN)), 0.8, robot=robot)
            ],
        )

    # With nothing arbitrating, both arms are free to go at once. Their
    # plans are each made with the other frozen where it stands, so both
    # succeed — and the rollout catches the two of them converging.
    clash = scene.sequence("clash")
    clash.step(
        "both_go",
        actions=[bt.seq.motion("near_to_pick"), bt.seq.motion("far_to_pick")],
        transition=bt.seq.all_of(bt.seq.robot_done(NEAR), bt.seq.robot_done(FAR)),
    )

    sq.step(
        "feed",
        actions=[
            bt.seq.start("conv"),
            bt.seq.start("cleats"),
        ],
    )
    # The eye starts each pick: the next carton arriving is what sends the
    # far arm over. From there the two arms hand the station back and forth,
    # and the *only* thing keeping the near arm out is the far arm's zone —
    # which is why `--clash` (which drops exactly that condition) collides.
    #
    # Supply is endless; the pallet is not. `CYCLES` courses and the run is
    # over — with the belt still running and the magazine still feeding.
    for layer in range(CYCLES):
        tag = f"_{layer + 1}"
        far_box, near_box = CARTON[2 * layer], CARTON[2 * layer + 1]
        pick_cycle(FAR, far_box, bt.seq.signal("beam_ahead"), tag)
        pick_cycle(
            NEAR,
            near_box,
            bt.seq.signal("zone_far", False) if interlocked else bt.seq.immediately(),
            tag,
        )
        place_cycle(FAR, far_box, layer, tag)
        place_cycle(NEAR, near_box, layer, tag)
    sq.step(
        "home",
        actions=[bt.seq.motion("near_home"), bt.seq.motion("far_home")],
        transition=bt.seq.all_of(bt.seq.robot_done(NEAR), bt.seq.robot_done(FAR)),
    )
    return sq.name


def shared_airspace(timeline) -> float:
    """Seconds both arms are inside the contested zone at once — zero in a
    cell whose interlock is doing its job."""
    edges = dict(timeline.signals)
    occupied = {}
    for robot in (NEAR, FAR):
        spans, start = [], None
        for t, on in edges[f"zone_{robot}"]:
            if on and start is None:
                start = t
            elif not on and start is not None:
                spans.append((start, t))
                start = None
        if start is not None:
            spans.append((start, timeline.duration))
        occupied[robot] = spans
    return sum(
        max(0.0, min(a1, b1) - max(a0, b0))
        for a0, a1 in occupied[NEAR]
        for b0, b1 in occupied[FAR]
    )


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    clash = "--clash" in sys.argv
    out = Path(args[0]) if args else Path("cell_dual.usda")

    scene = build_cell()
    name = build_cycle(scene, interlocked=not clash)

    if clash:
        # Dropping the interlock does not necessarily crash the cell — and
        # that is the part worth seeing. The near arm comes over while the
        # far arm is still leaving; whether they touch is down to how the
        # two transfers happen to line up, which is not a safety argument.
        # The zones say it plainly: both arms in the contested airspace at
        # once.
        try:
            timeline = scene.simulate_sequence(name)
        except ValueError as e:
            print(f"the unarbitrated cell fails outright:\n   {e}\n")
        else:
            overlap = shared_airspace(timeline)
            print(
                f"the unarbitrated cell happens to run ({timeline.duration:.2f}s), "
                f"but both arms are over the station together for {overlap:.2f}s.\n"
                "   Nothing separated them — the transfers merely missed each "
                "other.\n"
            )

        # Give them one reason to converge and the rollout says so at the
        # tick it happens, with the link pair. That is the guard that does
        # not depend on timing.
        try:
            scene.simulate_sequence("clash")
            raise SystemExit("expected a robot-robot collision")
        except ValueError as e:
            print(f"asked to enter together, they are caught:\n   {e}")
        return

    timeline = scene.simulate_sequence(name)
    print(f"cycle time: {timeline.duration:.2f}s")
    for robot in timeline.robots:
        busy = sum(end - start for _, start, end in timeline.moves(robot))
        print(f"  {robot:5s} moving {busy:5.2f}s of {timeline.duration:.2f}s")

    # What the second arm bought: how long both were in motion at once.
    moves = {r: timeline.moves(r) for r in timeline.robots}
    overlap, t, step = 0.0, 0.0, 0.02
    while t < timeline.duration:
        if all(any(a <= t <= b for _, a, b in moves[r]) for r in timeline.robots):
            overlap += step
        t += step
    print(f"both arms in motion for {overlap:.1f}s of it")
    print(f"stacked {CYCLES} course(s) on each pallet from a pool of {CARTONS}")

    warnings = timeline.export_usd(out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported to {out} — view with: usdview {out}")


if __name__ == "__main__":
    main()
