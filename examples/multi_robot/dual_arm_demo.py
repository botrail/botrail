"""Two arms, one robot: a dual-arm kitting cell.

Two UR5e arms stand on one torso and are *one* robot with two planning
groups, `left` and `right` — the way a dual-arm product (OpenArm, YuMi)
or a pair of arms on one controller is modelled. Every API you know takes
`group=`: IK and planning move one arm and leave the other where it is,
grasps hang off an arm's own tip, and the rollout drives the two arms'
joints independently — a motion on one arm and a ramp on the other bake
side by side, and a second driver on a joint that is already moving is a
hard error rather than a silent overwrite.

The cell is a kitting station. A tray on the bench holds four parts, two
per side; each arm picks its own column and drops the parts into one bin
between them. The bin's airspace is contested — both arms drop at the
same point — so the two programs interlock the way a PLC would: a zone
per arm over the bin, and a step that will not enter while the other
arm's zone reads occupied (the right arm has priority, announced ahead of
its pick). One part changes hands: the left arm sets it on the tray's
centre, the right arm takes it from there. Nothing plans the two arms
together; each motion is planned for its arm with the other frozen as an
obstacle, and the rollout re-checks arm-against-arm every tick — a
meeting is a `GroupCollision` with a timestamp. `--clash` drops the
interlock to show exactly that.

`--carry` finishes by moving the tray with both hands: the left arm
grasps it (`attach`), the right arm latches onto it (`track`) and
follows the lift wherever the left arm's planned motion takes it — a
leader/follower carry, no closed-loop planning.

The same cell is exported per arm: `export_script(group="left")` lowers
the left program to a 6-axis URScript, the right arm being the partner
controller its waits read on inputs. `--robot simple` runs the offline
rig — two primitive arms from the checkout — with the same code.

Run with:  python examples/multi_robot/dual_arm_demo.py [out.usda]
               [--robot ur5e|simple] [--carry] [--clash] [--studio]
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import botrail as bt

ASSETS = Path(__file__).resolve().parents[1] / "assets"

# --- cell dimensions (metres; z = 0 is the shop floor) ------------------
BENCH_TOP = 0.72
TRAY_THICK = 0.012
TRAY_TOP = BENCH_TOP + TRAY_THICK
PART = 0.05  # part edge length
BIN_SIZE = 0.16  # the kit bin, outer
BIN_WALL = 0.006
BIN_HEIGHT = 0.10
BIN_FLOOR = BENCH_TOP + BIN_WALL
DOWN = (1.0, 0.0, 0.0, 0.0)  # tool +Z at the floor
GAP = 0.003  # a released part is let go this far above where it lands
# The bin's contested airspace: a narrow column over its mouth, tall enough
# that "clear" means the other hand has left it, not merely risen in it.
ZONE = (0.16, 0.14, 0.50)
PARTS = ("L1", "L2", "R1", "R2")
LEFT, RIGHT = "left", "right"


@dataclass(frozen=True)
class Rig:
    """A dual-arm robot and the cell geometry its reach allows."""

    kind: str
    robot: bt.Robot
    ready: tuple[float, ...]  # one arm's home configuration
    seeds: tuple[tuple[float, ...], ...]  # other postures IK may start from
    torso_top: float  # the arms' mounting plane, on top of the torso
    standoff: float  # the flange's height above a face it holds (the cup)
    hover: float  # approach height above a grasp
    lift: float  # how high the tray is lifted for the carry
    carries: bool  # whether the rig reaches the two-handed carry's poses
    hands_over: bool  # whether the rig reaches the hand-off spot between the arms
    drop_clear: float  # how high above the bin's walls a part is let go
    span: float  # the arm bases sit at y = ±span
    bin: tuple[float, float]  # the kit bin's centre (x, y)
    tray: tuple[float, float]  # the tray's centre
    tray_width: float  # the tray's extent in y
    handoff: tuple[float, float]  # where the left arm sets the part down for the right
    handoff_rise: float  # the hand-off fixture's height above the tray (0: the tray itself)
    part_x: tuple[float, float]  # the two columns' x
    part_y: float  # the columns sit at y = ±part_y
    edge: float  # the hands take the tray at y = ±edge
    carry: float  # how far in +x the tray is carried


def build_rig(kind: str = "ur5e") -> Rig:
    """`ur5e`: two catalog UR5e on one torso. `simple`: the offline rig,
    two primitive arms from the checkout, on a cell shrunk to their
    reach. The UR5e rig falls back to the offline one when the catalog
    cannot be fetched, so the demo always runs."""
    if kind == "ur5e":
        try:
            arm = bt.Robot.from_catalog("ur5e")
        except Exception as e:  # noqa: BLE001 — offline: the primitive arms stand in
            print(f"catalog UR5e unavailable ({e}); running the offline rig")
            return build_rig("simple")
        ready = (0.0, -1.9, 1.8, -1.5, -1.57, 0.0)
        seeds = ((0.0, -1.4, 1.6, -1.8, -1.57, 0.0), (0.0, -2.2, 2.2, -1.6, -1.57, 0.0))
        yaw = (0.0, 0.0, 0.0, 1.0)
        geometry = {
            "torso_top": 0.75, "standoff": 0.008, "hover": 0.10, "lift": 0.10,
            "carries": True, "hands_over": True, "drop_clear": 0.10,
            "span": 0.30, "bin": (0.26, 0.0), "tray": (0.55, 0.0), "tray_width": 0.60,
            "handoff": (0.55, 0.0), "handoff_rise": 0.0,
            "part_x": (0.48, 0.62), "part_y": 0.21, "edge": 0.27, "carry": 0.12,
        }
    elif kind == "simple":
        arm = bt.Robot.from_urdf(ASSETS / "simple_arm.urdf")
        ready = (0.0, -1.2, 1.4, -1.77, -1.57, 0.0)
        seeds = (
            (0.0, -0.8, 1.6, -2.4, -1.57, 0.0),
            (0.0, -0.5, 1.2, -2.3, -1.57, 0.0),
            (0.0, -1.0, 2.0, -2.6, -1.57, 0.0),
            (0.0, -1.5, 2.2, -2.3, -1.57, 0.0),
            (0.0, -0.3, 0.8, -2.1, -1.57, 0.0),
            (0.0, -1.8, 2.4, -2.2, -1.57, 0.0),
            (0.0, 0.0, 0.5, -2.1, -1.57, 0.0),
        )
        yaw = (0.0, 0.0, 1.0, 0.0)  # its ready pose reaches to -x: face it round
        # The primitive arm hangs a 40 mm tool block under its flange and
        # points a tool down without folding into itself only in a narrow
        # band — 0–15 cm below its base, 25–45 cm out, straight ahead. It
        # holds parts by that block, stands higher, and works a tighter,
        # lower cell; the centre line (the hand-off) and the tray's edges
        # (the carry) fold it, so those are the real arms' to show.
        geometry = {
            "torso_top": 0.98, "standoff": 0.045, "hover": 0.06, "lift": 0.03,
            "carries": False, "hands_over": False, "drop_clear": 0.04,
            "span": 0.28, "bin": (0.25, 0.0), "tray": (0.49, 0.0), "tray_width": 0.50,
            "handoff": (0.38, 0.0), "handoff_rise": 0.0,
            "part_x": (0.37, 0.45), "part_y": 0.16, "edge": 0.19, "carry": 0.04,
        }
    else:
        raise SystemExit(f"unknown rig {kind!r} — use ur5e or simple")
    # One robot, two arms: the composite is a single 12-DOF model whose
    # planning groups are the two arms, named by the mount.
    pair = bt.Robot.dual_arm(
        arm, arm,
        left_position=(0.0, geometry["span"], 0.0), left_quaternion=yaw,
        right_position=(0.0, -geometry["span"], 0.0), right_quaternion=yaw,
    )
    return Rig(kind, pair, ready, seeds, **geometry)


def parts_of(rig: Rig) -> dict[str, tuple[float, float]]:
    """Where each part waits on the tray: the left column is the left
    arm's, the right column the right arm's."""
    x0, x1 = rig.part_x
    return {
        "L1": (x0, rig.part_y), "L2": (x1, rig.part_y),
        "R1": (x0, -rig.part_y), "R2": (x1, -rig.part_y),
    }


def home(rig: Rig) -> list[float]:
    """Both arms at the rig's ready pose, in the composite's joint order."""
    names = rig.robot.joint_names
    q = [0.0] * rig.robot.dof
    for arm in (LEFT, RIGHT):
        for joint, value in zip(rig.robot.group(arm).joints, rig.ready):
            q[names.index(joint)] = value
    return q


def build_cell(rig: Rig) -> bt.Scene:
    scene = bt.Scene(rig.robot, base_position=(0.0, 0.0, rig.torso_top))
    scene.set_joint_positions(home(rig))
    robot = scene.robots[0]

    # The world: floor, the torso the arms stand on (scenery), the bench,
    # the tray with its four parts, and the kit bin between the arms.
    scene.add_box("floor", size=(2.4, 2.4, 0.05), position=(0.4, 0.0, -0.025),
                  color=(0.35, 0.37, 0.40))
    scene.add_box("torso", size=(0.30, 0.90, rig.torso_top),
                  position=(0.0, 0.0, rig.torso_top / 2), color=(0.34, 0.36, 0.40))
    scene.set_obstacle_enabled("torso", False)
    scene.add_box("bench", size=(0.80, 1.40, 0.04), position=(0.55, 0.0, BENCH_TOP - 0.02),
                  color=(0.55, 0.50, 0.42))
    scene.add_box("tray", size=(0.30, rig.tray_width, TRAY_THICK),
                  position=(*rig.tray, BENCH_TOP + TRAY_THICK / 2), color=(0.25, 0.28, 0.33))
    for name, (x, y) in parts_of(rig).items():
        scene.add_box(name, size=(PART, PART, PART), position=(x, y, TRAY_TOP + PART / 2),
                      color=(0.85, 0.33, 0.20) if name[0] == "L" else (0.20, 0.55, 0.85))
        # Released parts fall into the bin under physics; until then they
        # rest on the tray.
        scene.set_physics(name, dynamic=True, mass=0.2, friction=0.6)
    if rig.handoff_rise > 0:
        # A post on the tray to hand the part over on, where the arms
        # reach it best.
        scene.add_box("fixture", size=(0.07, 0.07, rig.handoff_rise),
                      position=(*rig.handoff, TRAY_TOP + rig.handoff_rise / 2), color=(0.45, 0.47, 0.50))
    bx, by = rig.bin
    half, wall = BIN_SIZE / 2, BIN_WALL
    scene.add_box("bin/floor", size=(BIN_SIZE, BIN_SIZE, wall),
                  position=(bx, by, BENCH_TOP + wall / 2), color=(0.30, 0.30, 0.32))
    for side, (dx, dy, sx, sy) in {
        "west": (-half + wall / 2, 0.0, wall, BIN_SIZE),
        "east": (half - wall / 2, 0.0, wall, BIN_SIZE),
        "south": (0.0, -half + wall / 2, BIN_SIZE, wall),
        "north": (0.0, half - wall / 2, BIN_SIZE, wall),
    }.items():
        scene.add_box(f"bin/{side}", size=(sx, sy, BIN_HEIGHT),
                      position=(bx + dx, by + dy, BENCH_TOP + BIN_HEIGHT / 2),
                      color=(0.30, 0.30, 0.32))

    # The interlock's eyes: one zone per arm over the bin. A zone reports
    # "somebody is inside", not who, so each arm needs its own — one zone
    # watching both would be tripped by the very arm waiting on it.
    # The zone is the airspace over the bin's mouth — where a hand goes
    # to drop — not the arm's whole way there: an arm reaching past the
    # bin for its tray column must not read as inside.
    for arm in (LEFT, RIGHT):
        scene.add_zone_sensor(
            f"zone_{arm}",
            position=(bx, by, BENCH_TOP + BIN_HEIGHT + ZONE[2] / 2), size=ZONE,
            watch_groups=[(robot, arm)],
        )
    # The programs' handshake flags.
    for signal in ("right_needs_bin", "handoff_ready", "right_done",
                   "tray_up", "follower_on", "carried"):
        scene.define_signal(signal)
    return scene


# Where each part is let go over the bin: four spots a part's width apart
# at the bin's mouth, so the kit lands side by side rather than in a stack.
# All four are in the bin's airspace, a hand's height above its walls — one
# small column both arms drop into, which is what makes the bin contested.
SLOTS = {"L1": (-0.03, 0.03), "L2": (0.03, 0.03), "R1": (-0.03, -0.03), "R2": (0.03, -0.03)}


def drop_point(rig: Rig, part: str) -> tuple[float, float, float]:
    dx, dy = SLOTS[part]
    return (rig.bin[0] + dx, rig.bin[1] + dy,
            BENCH_TOP + BIN_HEIGHT + rig.drop_clear + PART + rig.standoff)


def teach(scene: bt.Scene, rig: Rig, carry: bool) -> None:
    """Every motion of both arms, taught by IK in each arm's own group.
    A pick is an approach plus a straight descent; a place lifts, moves
    over the destination and lowers; a clear lifts out and goes home."""
    q_home = home(rig)
    line = "cartesian_line"  # approaches and lifts: straight TCP lines
    hover, lift = rig.hover, rig.lift
    names = rig.robot.joint_names

    def posed(arm: str, joints: tuple[float, ...]) -> list[float]:
        """Home, with one arm at the given posture."""
        q = list(q_home)
        for joint, value in zip(rig.robot.group(arm).joints, joints):
            q[names.index(joint)] = value
        return q

    def at(arm: str, position, seed=None) -> list[float]:
        """The arm at `position`, tool down, in a configuration free of
        collision — IK is seeded from the pose it continues, then from the
        rig's alternative postures, the way a teacher jogs an arm out of a
        fold before saving the point."""
        seeds = [seed or q_home] + [posed(arm, s) for s in rig.seeds]
        short, touching = None, set()
        for q_seed in seeds:
            scene.set_joint_positions(q_seed)
            result = scene.set_tcp_target(position, DOWN, group=arm)
            if not result.converged:
                short = result.pos_error
                continue
            pairs = scene.check_collisions()
            if not pairs:
                return list(scene.joint_positions)
            touching.update(f"{a[1]} × {b[1]}" for a, b in pairs)
        raise SystemExit(
            f"{rig.kind} rig: the {arm} arm cannot reach {position} collision-free"
            + (f" ({short * 1000:.1f} mm short)" if short is not None else "")
            + (f"; it touches {', '.join(sorted(touching))}" if touching else "")
        )

    def pick_place_clear(arm: str, tag: str, part_xy, part_top: float, dest, *,
                         set_down: bool) -> None:
        """Pick: approach, then a straight descent. Place: a straight
        lift, then either a set-down (over the spot, straight down — the
        hand-off) or a drop from above (the bin: nothing descends into a
        bin that is filling up). Clear: back home."""
        x, y = part_xy
        grip_z = part_top + rig.standoff
        above = at(arm, (x, y, grip_z + hover))
        grip = at(arm, (x, y, grip_z), seed=above)
        scene.add_segment(f"{arm}_pick_{tag}", goal=above, group=arm)
        scene.add_segment(f"{arm}_pick_{tag}", goal=grip, kind=line, group=arm)
        scene.add_segment(f"{arm}_place_{tag}", goal=above, kind=line, group=arm)
        if set_down:
            over = at(arm, (dest[0], dest[1], dest[2] + hover))
            release = at(arm, dest, seed=over)
            scene.add_segment(f"{arm}_place_{tag}", goal=over, group=arm)
            scene.add_segment(f"{arm}_place_{tag}", goal=release, kind=line, group=arm)
            scene.add_segment(f"{arm}_clear_{tag}", goal=over, kind=line, group=arm)
        else:
            release = at(arm, dest)
            scene.add_segment(f"{arm}_place_{tag}", goal=release, group=arm)
        scene.add_segment(f"{arm}_clear_{tag}", goal=q_home, group=arm)

    parts = parts_of(rig)
    tray_top = TRAY_TOP + PART  # a part waiting on the tray
    handoff = rig.handoff
    handoff_top = TRAY_TOP + rig.handoff_rise
    # The left arm: its own two parts — one into the bin, one onto the
    # hand-off spot for the right arm (into the bin too, on a rig that
    # cannot reach the spot).
    pick_place_clear(LEFT, "L1", parts["L1"], tray_top, drop_point(rig, "L1"), set_down=False)
    if rig.hands_over:
        pick_place_clear(LEFT, "L2", parts["L2"], tray_top,
                         (handoff[0], handoff[1], handoff_top + GAP + PART + rig.standoff),
                         set_down=True)
    else:
        pick_place_clear(LEFT, "L2", parts["L2"], tray_top, drop_point(rig, "L2"), set_down=False)
    # The right arm: its own two parts and the handed-over one, all into
    # the bin.
    pick_place_clear(RIGHT, "R1", parts["R1"], tray_top, drop_point(rig, "R1"), set_down=False)
    pick_place_clear(RIGHT, "R2", parts["R2"], tray_top, drop_point(rig, "R2"), set_down=False)
    if rig.hands_over:
        pick_place_clear(RIGHT, "H", handoff, handoff_top + GAP + PART, drop_point(rig, "L2"),
                         set_down=False)

    if carry:
        # Two hands on the tray. The leader lifts it clear first; the
        # follower then takes the far edge of the *raised* tray and
        # latches on, so the leader's carry is planned against a
        # follower that is beside the load, not in its way.
        tx, ty = rig.tray
        hold_z = TRAY_TOP + rig.standoff
        above = at(LEFT, (tx, ty + rig.edge, hold_z + hover))
        hold = at(LEFT, (tx, ty + rig.edge, hold_z), seed=above)
        up = at(LEFT, (tx, ty + rig.edge, hold_z + lift), seed=hold)
        far = at(LEFT, (tx + rig.carry, ty + rig.edge, hold_z + lift), seed=up)
        down = at(LEFT, (tx + rig.carry, ty + rig.edge, hold_z + GAP), seed=far)
        scene.add_segment("left_tray", goal=above, group=LEFT)
        scene.add_segment("left_tray", goal=hold, kind=line, group=LEFT)
        scene.add_segment("left_lift", goal=up, kind=line, group=LEFT)
        scene.add_segment("left_carry", goal=far, kind=line, group=LEFT)
        scene.add_segment("left_carry", goal=down, kind=line, group=LEFT)
        scene.add_segment("left_home", goal=far, kind=line, group=LEFT)
        scene.add_segment("left_home", goal=q_home, group=LEFT)
        f_hover = at(RIGHT, (tx, ty - rig.edge, hold_z + lift + hover))
        f_hold = at(RIGHT, (tx, ty - rig.edge, hold_z + lift), seed=f_hover)
        scene.add_segment("right_tray", goal=f_hover, group=RIGHT)
        scene.add_segment("right_tray", goal=f_hold, kind=line, group=RIGHT)
        scene.add_segment("right_home", goal=q_home, group=RIGHT)
    scene.set_joint_positions(q_home)


def build_programs(scene: bt.Scene, rig: Rig, *, interlocked: bool = True,
                   carry: bool = False) -> list[str]:
    """One program per arm, PLC style. They share the robot and the bin,
    and talk through signals: the bin's turn, the handover, the carry."""
    robot = scene.robots[0]
    del robot  # the programs address arms by group, not the robot by name
    seq = bt.seq

    def cycle(sq, arm: str, tag: str, part: str, *, into_bin: bool, wait=None,
              after=(), cleared=()) -> None:
        sq.step(f"{arm} pick {tag}", actions=[seq.motion(f"{arm}_pick_{tag}")], transition=seq.done())
        sq.step(f"{arm} grasp {tag}", actions=[seq.attach(part, group=arm)], transition=seq.immediately())
        if into_bin:
            sq.step(f"{arm} wait bin {tag}", transition=wait if interlocked else seq.immediately())
        sq.step(f"{arm} place {tag}", actions=[seq.motion(f"{arm}_place_{tag}")], transition=seq.done())
        sq.step(f"{arm} release {tag}", actions=[seq.detach(part), *after], transition=seq.immediately())
        sq.step(f"{arm} clear {tag}", actions=[seq.motion(f"{arm}_clear_{tag}")], transition=seq.done())
        if cleared:
            # Announced once the arm is out of the way, not at the release:
            # a flag the other arm moves in on.
            sq.step(f"{arm} cleared {tag}", actions=list(cleared), transition=seq.immediately())

    # The left program yields the bin to the right arm: it enters only
    # while the right arm neither is inside nor has announced a drop.
    left_turn = seq.all_of(seq.signal("right_needs_bin", False), seq.signal("zone_right", False))
    right_turn = seq.signal("zone_left", False)

    left = scene.sequence(LEFT)
    cycle(left, LEFT, "L1", "L1", into_bin=True, wait=left_turn)
    if rig.hands_over:
        cycle(left, LEFT, "L2", "L2", into_bin=False, cleared=[seq.set_signal("handoff_ready")])
    else:
        cycle(left, LEFT, "L2", "L2", into_bin=True, wait=left_turn)

    right = scene.sequence(RIGHT)
    rounds = [("R1", "R1"), ("R2", "R2")] + ([("H", "L2")] if rig.hands_over else [])
    for tag, part in rounds:
        if tag == "H":
            right.step("right await handoff", transition=seq.signal("handoff_ready"))
        right.step(f"right claim {tag}", actions=[seq.set_signal("right_needs_bin")],
                   transition=seq.immediately())
        last = tag == rounds[-1][0]
        cycle(right, RIGHT, tag, part, into_bin=True, wait=right_turn,
              after=[seq.set_signal("right_needs_bin", False)],
              cleared=[seq.set_signal("right_done")] if last else ())

    if carry:
        left.step("left await right", transition=seq.signal("right_done"))
        left.step("left to tray", actions=[seq.motion("left_tray")], transition=seq.done())
        left.step("left grasp tray", actions=[seq.attach("tray", group=LEFT)], transition=seq.immediately())
        left.step("left lift tray", actions=[seq.motion("left_lift"), seq.set_signal("tray_up")],
                  transition=seq.all_of(seq.done(), seq.signal("follower_on")))
        left.step("left carry", actions=[seq.motion("left_carry")], transition=seq.done())
        left.step("left set down", actions=[seq.detach("tray"), seq.set_signal("carried")],
                  transition=seq.immediately())
        left.step("left home", actions=[seq.motion("left_home")], transition=seq.done())
        right.step("right await tray", transition=seq.signal("tray_up"))
        right.step("right to tray", actions=[seq.motion("right_tray")], transition=seq.done())
        right.step("right hold tray", actions=[seq.track("tray", group=RIGHT), seq.set_signal("follower_on")],
                   transition=seq.signal("carried"))
        right.step("right let go", actions=[seq.untrack(group=RIGHT)], transition=seq.immediately())
        right.step("right home", actions=[seq.motion("right_home")], transition=seq.done())
    return [LEFT, RIGHT]


def build(kind: str = "ur5e", *, carry: bool = False, clash: bool = False) -> tuple[bt.Scene, Rig, list[str]]:
    rig = build_rig(kind)
    if carry and not rig.carries:
        print(f"the {rig.kind} rig cannot hold the tray with both hands — kitting only")
        carry = False
    scene = build_cell(rig)
    teach(scene, rig, carry)
    programs = build_programs(scene, rig, interlocked=not clash, carry=carry)
    return scene, rig, programs


def simulate(scene: bt.Scene, programs: list[str]):
    # Physics: a released part falls into the bin instead of hanging in
    # the air where the arm let go of it.
    return scene.simulate_sequences(programs, physics=True, max_duration=120.0)


def overlap_seconds(timeline, robot: str) -> float:
    """Seconds both arms were in motion at once — what the second arm
    bought."""
    moves = {arm: timeline.moves(robot, group=arm) for arm in (LEFT, RIGHT)}
    total, t, step = 0.0, 0.0, 0.02
    while t < timeline.duration:
        if all(any(a <= t <= b for _, a, b in moves[arm]) for arm in moves):
            total += step
        t += step
    return total


def in_bin(rig: Rig, position) -> bool:
    """Inside the bin's footprint and resting in it — on its floor or on
    another part."""
    x, y, z = position
    half = BIN_SIZE / 2
    return (
        abs(x - rig.bin[0]) < half
        and abs(y - rig.bin[1]) < half
        and BENCH_TOP < z < BENCH_TOP + len(PARTS) * PART
    )


# The I/O each arm's controller sees: the other arm's zone and flags are
# inputs, its own flags are outputs. Signals both programs share are one
# wire, so a flag one program raises is the other program's input.
LEFT_IO = {"inputs": {"zone_right": 0, "right_needs_bin": 1, "right_done": 2},
           "outputs": {"handoff_ready": 0}}
RIGHT_IO = {"inputs": {"zone_left": 0, "handoff_ready": 1},
            "outputs": {"right_needs_bin": 0, "right_done": 1}}


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    kind = "ur5e"
    if "--robot" in sys.argv:
        kind = sys.argv[sys.argv.index("--robot") + 1]
        args = [a for a in args if a != kind]
    carry = "--carry" in sys.argv
    clash = "--clash" in sys.argv
    out = Path(args[0]) if args else Path("dual_arm_kitting.usda")

    scene, rig, programs = build(kind, carry=carry, clash=clash)
    robot = scene.robots[0]
    print(f"{robot}: {rig.robot.dof} DOF, arms {rig.robot.groups} ({rig.kind} rig)")

    if clash:
        # Without the interlock both arms head for the same drop point.
        # The rollout does not average that away: the tick they meet is
        # the answer, with the arms and the links named.
        try:
            simulate(scene, programs)
            raise SystemExit("expected the arms to collide")
        except ValueError as e:
            print(f"unarbitrated, the arms are caught:\n   {e}")
        return

    timeline = simulate(scene, programs)
    print(f"cycle time: {timeline.duration:.2f}s")
    for arm in (LEFT, RIGHT):
        busy = timeline.busy_seconds(robot, group=arm)
        print(f"  {arm:5s} moving {busy:5.2f}s of {timeline.duration:.2f}s "
              f"({timeline.utilization(robot, group=arm):.0%})")
    print(f"both arms in motion for {overlap_seconds(timeline, robot):.1f}s of it")
    kitted = [p for p in PARTS if in_bin(rig, timeline.object_pose(p, timeline.duration)[0])]
    print(f"kitted {len(kitted)} of {len(PARTS)} parts: {', '.join(kitted)}"
          + (" (L2 handed over)" if rig.hands_over and "L2" in kitted else ""))
    if carry and rig.carries:
        (x, y, _), _ = timeline.object_pose("tray", timeline.duration)
        print(f"tray carried {x - rig.tray[0]:.2f} m to ({x:.2f}, {y:.2f}) with both hands")

    warnings = timeline.export_usd(out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported to {out} — view with: usdview {out}")

    if not (carry and rig.carries):
        # One controller program per arm: the left program's waits on the
        # right arm read the right controller's flags on inputs.
        for arm, io in ((LEFT, LEFT_IO), (RIGHT, RIGHT_IO)):
            path = out.with_name(f"{out.stem}_{arm}.script")
            timeline.export_script(path, sequence=arm, group=arm, **io)
            print(f"wrote {path} ({arm} arm, 6 axes)")

    if "--studio" in sys.argv:
        bt.studio(scene)


if __name__ == "__main__":
    main()
