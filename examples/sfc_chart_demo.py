"""A pick cell you can watch: the studio's SFC chart over a real cell.

The cell is a UR5e with a Robotiq 2F-85 on a coupling (catalog models —
the first run downloads them), picking parts off a belt. The belt is real
geometry, not just a transport zone; the gripper really closes on the
part before the attach; and each seat is approached the way a cell does
— over the target, straight down, release, lift clear.

**Two parts run through in order, and the cell decides for itself which
is which.** They differ only in height, and an over-height beam strung
across the station is the gauge: the 60 mm part passes under it, the
80 mm one breaks it. Nothing else marks them — no flag, no scenario.

Each part's block runs the same process: get in position, run the belt,
wait for the part to *arrive* (a rising edge — a part already sitting on
the beam is not an arrival), stop, read the gauge, grip, and route. The
gauge only reads while the part stands on the station, so its verdict is
latched into a relay and acted on after the pick — the oldest habit in
ladder logic, and the reason the chart has two ◇ diamonds per part: one
that measures, one that routes. A second program (`lamp`) watches the
gripper through edge conditions, so two columns scan side by side.

    .venv/bin/python examples/sfc_chart_demo.py

Then, in the browser:

1. Sequence tab -> **SFC chart** — the authored program, neutral.
2. **Simulate (2 programs)** — the path each part took stays solid and
   the arms it did not take dash out. The first block marks *ok* and
   seats its part on the tray; the second marks *ng* and puts its part
   back on the belt to be sent downstream.
3. Press play in the dock: a token rides each column, and the live
   condition beside it colors green as it becomes true (`0.50s` counts
   up, edge atoms underline while the signal is high, and the condition
   that released a step glows for a beat after the token hops).
4. Click any solid step box — the playhead jumps to when it began.
5. Watch the `too_tall` lane in the dock: a long high while the tall part
   stands on the station (that is the reading the `reject` relay latches)
   and a blip each time a part is lifted up through the beam afterwards —
   which is exactly why the verdict is captured rather than re-read.
"""

import botrail as bt

# --- cell dimensions (metres; z = 0 is the shop floor) ------------------
# Everything stands *above* z = 0: the studio draws the floor there, and
# an obstacle under it is simply behind the floor — invisible.
FLOOR_TOP = 0.0
BASE_Z = 0.74  # the robot's mounting plane, on top of the pedestal
BELT_Y = 0.45
BELT_TOP = BASE_Z - 0.10
# The belt runs on past the pick point: that downstream stretch is where
# rejects go, so the cell needs no reject furniture at all.
BELT_X0, BELT_X1 = -0.80, 0.78
PART = 0.06
PICK_X = 0.25
# The beam trips on the part's leading edge, so it sits half a part
# upstream of the pick point: when the belt halts, the part is centred
# where the gripper is taught to descend.
BEAM_X = PICK_X + PART / 2
TRAY_TOP = BASE_Z - 0.12
PICK_Z = BELT_TOP + PART / 2  # beam height: the middle of a good part
# One taught pick has to serve parts of both heights, so it grips well
# down the side of a good part but not so deep that the gripper's body
# would land on a tall one. Either way the part hangs `HANG` below the
# tool, so one taught seat height puts either of them down.
GRIP_Z = BELT_TOP + 0.05
HANG = GRIP_Z - BELT_TOP
HOVER = 0.14  # how far above a seat the arm hovers before coming down
# A carried part is checked as part of the robot, so "resting on" a
# surface reads as a collision. Seat it a few millimetres clear — the
# same margin a real release leaves, and invisible on screen.
SEAT_GAP = 0.004

TRAY_XY = (0.30, -0.42)
# Rejects are set back down on the belt downstream of the beam and sent
# away — the way a line actually purges a bad part.
PURGE_X = 0.52

# Two parts run through, in order. They differ in height, and that is
# what the gauge measures: an over-height beam strung across the station
# above a good part's top and below a bad one's.
NG_HEIGHT = 0.08
GAUGE_Z = BELT_TOP + (PART + NG_HEIGHT) / 2
PARTS = [  # name, height, colour, where it starts on the belt
    ("part_ok", PART, (0.80, 0.58, 0.24), -0.35),
    ("part_ng", NG_HEIGHT, (0.72, 0.30, 0.24), -0.65),
]

DOWN = (1.0, 0.0, 0.0, 0.0)  # tool +Z pointing at the floor
OPEN, SHUT = 0.0, 0.33  # 2F-85 finger joint: 92 mm and 60 mm across the pads
READY = [0.0, -1.9, 1.8, -1.5, -1.57, 0.0, OPEN]
# The links a grip legitimately rests on: without them the pads read as a
# collision the moment the part becomes part of the robot.
PADS = [
    f"{side}_inner_{part}"
    for side in ("left", "right")
    for part in ("finger", "finger_pad", "knuckle")
]

BELT_MID = (BELT_X0 + BELT_X1) / 2
BELT_LEN = BELT_X1 - BELT_X0

# name -> (size, position, colour, metalness, roughness)
SCENERY = [
    ("belt/slab", (BELT_LEN, 0.24, 0.08), (BELT_MID, BELT_Y, BELT_TOP - 0.04),
     (0.13, 0.14, 0.16), 0.1, 0.85),
    ("belt/rail-l", (BELT_LEN, 0.02, 0.05), (BELT_MID, BELT_Y - 0.13, BELT_TOP + 0.02),
     (0.62, 0.64, 0.67), 0.9, 0.35),
    ("belt/rail-r", (BELT_LEN, 0.02, 0.05), (BELT_MID, BELT_Y + 0.13, BELT_TOP + 0.02),
     (0.62, 0.64, 0.67), 0.9, 0.35),
    ("cell/tray-stand", (0.18, 0.18, TRAY_TOP), (*TRAY_XY, TRAY_TOP / 2),
     (0.30, 0.32, 0.36), 0.6, 0.5),
    ("cell/tray", (0.34, 0.34, 0.02), (*TRAY_XY, TRAY_TOP + 0.01),
     (0.20, 0.42, 0.52), 0.4, 0.45),
]


def build_cell() -> bt.Scene:
    """The cell: arm with gripper, belt structure, fixtures, the part, and
    the two field devices (transport zone + through-beam sensor)."""
    arm = bt.Robot.from_catalog("ur5e")
    coupling = bt.Robot.from_catalog("gripper-coupling")
    gripper = bt.Robot.from_catalog("2f-85")
    robot = arm.attach_tool(coupling, prefix="cpl_").attach_tool(gripper)
    scene = bt.Scene(robot, base_position=(0.0, 0.0, BASE_Z))
    scene.set_joint_positions(READY)

    for name, size, position, color, metalness, roughness in SCENERY:
        scene.add_box(name, size=size, position=position, color=color)
        scene.set_obstacle_material(name, metalness=metalness, roughness=roughness)
    # The pedestal the arm stands on: scenery, not an obstacle to itself.
    pedestal = BASE_Z - FLOOR_TOP
    scene.add_cylinder("cell/pedestal", 0.09, pedestal, (0.0, 0.0, FLOOR_TOP + pedestal / 2),
                       color=(0.34, 0.36, 0.40))
    scene.set_obstacle_material("cell/pedestal", metalness=0.8, roughness=0.4)
    scene.set_obstacle_enabled("cell/pedestal", False)
    legs = BELT_TOP - 0.08 - FLOOR_TOP
    for x in (BELT_X0 + 0.15, BELT_X1 - 0.15):
        for side in (-1, 1):
            leg = f"belt/leg{x:+.2f}{side:+d}"
            scene.add_cylinder(leg, 0.02, legs, (x, BELT_Y + side * 0.10, FLOOR_TOP + legs / 2),
                               color=(0.50, 0.52, 0.56))
            scene.set_obstacle_material(leg, metalness=0.8, roughness=0.4)
    # The sensors' own hardware, so the beams come out of something: the
    # arrival beam at part height, the gauge one part-step higher.
    for x, z, tag in ((BEAM_X, PICK_Z, "arrive"), (PICK_X, GAUGE_Z, "gauge")):
        for side, end in ((-1, "emitter"), (1, "receiver")):
            name = f"cell/{tag}-{end}"
            scene.add_box(name, size=(0.04, 0.03, 0.07),
                          position=(x, BELT_Y + side * 0.155, z), color=(0.85, 0.55, 0.15))
            scene.set_obstacle_material(name, metalness=0.3, roughness=0.5)

    # The two workpieces, waiting their turn on the belt. They are gripped
    # at the same height, so the taller one is simply held lower down —
    # one taught pick serves both.
    for name, height, color, start in PARTS:
        scene.add_box(name, size=(PART, PART, height),
                      position=(start, BELT_Y, BELT_TOP + height / 2), color=color)
        scene.set_obstacle_material(name, metalness=0.2, roughness=0.6)

    # A conveyor is a transport zone: it sits above the slab, so it carries
    # the goods and not the structure (the slab's origin is below it).
    # The zone stops short of the slab's ends, so a part carried to the end
    # of the run comes to rest *on* the belt rather than half off it.
    scene.add_conveyor(
        "conv",
        zone_position=(BELT_MID, BELT_Y, BELT_TOP + 0.04),
        zone_size=(BELT_LEN - 0.16, 0.22, 0.10),
        velocity=(0.12, 0.0, 0.0),
        running=False,
    )
    names = [name for name, _, _, _ in PARTS]
    scene.add_beam_sensor(
        "part_at_pick",
        frm=(BEAM_X, BELT_Y - 0.14, PICK_Z),
        to=(BEAM_X, BELT_Y + 0.14, PICK_Z),
        watch=names,
    )
    # The gauge: a beam over the station, above a good part and below a
    # bad one. It only watches the workpieces, so the arm reaching in to
    # pick does not trip it. Nothing decides quality but this height.
    scene.add_beam_sensor(
        "too_tall",
        frm=(PICK_X, BELT_Y - 0.14, GAUGE_Z),
        to=(PICK_X, BELT_Y + 0.14, GAUGE_Z),
        watch=names,
    )
    # The gauge only reads while the part is standing on the station, so
    # the verdict is latched into a relay and acted on after the pick.
    scene.define_signal("reject")
    scene.define_signal("gripped")
    return scene


def teach(scene: bt.Scene) -> None:
    """Teach the cycle's motions by IK, so the tool really lands on the
    part. Each goal carries the finger joint too — a motion taught with
    the gripper open would re-open it mid-carry.

    Every seat is taught as a *pair*: a hover above it and the seat
    itself. Coming in sideways at working height and letting go is what
    makes a simulated cell look simulated; real ones arrive over the
    target, come straight down, release, and lift clear."""

    def at(position, finger):
        scene.set_joint_positions(READY)
        result = scene.set_tcp_target(position, DOWN)
        if not result.converged:
            raise SystemExit(f"cell layout unreachable at {position}")
        return [*scene.joint_positions[:6], finger]

    def seat(name, position, carried=SHUT):
        """`over_x` / `to_x` / `clear_x`: hover, come down, lift away
        empty-handed."""
        x, y, z = position
        scene.add_segment(f"over_{name}", goal=at((x, y, z + HOVER), carried))
        scene.add_segment(f"to_{name}", goal=at(position, carried))
        scene.add_segment(f"clear_{name}", goal=at((x, y, z + HOVER), OPEN))

    approach = at((PICK_X, BELT_Y, GRIP_Z + HOVER), OPEN)
    scene.add_segment("approach", goal=approach)
    scene.add_segment("descend", goal=at((PICK_X, BELT_Y, GRIP_Z), OPEN))
    scene.add_segment("lift", goal=[*approach[:6], SHUT])
    seat("tray", (*TRAY_XY, TRAY_TOP + 0.02 + HANG + SEAT_GAP))
    seat("purge", (PURGE_X, BELT_Y, BELT_TOP + HANG + SEAT_GAP))
    scene.add_segment("home", goal=READY)
    scene.set_joint_positions(READY)


def author_pick(scene: bt.Scene) -> None:
    """The process: one block per part, run back to back. Steps are the
    chart's boxes; transitions its bars."""
    sq = scene.sequence("pick")
    for name, _, _, _ in PARTS:
        one_part(sq, name)


def one_part(sq, part: str) -> None:
    """Feed, gauge, pick, and route one part."""
    # Get in position *first*, then run the belt. Feeding while the arm is
    # still travelling is the classic way to lose an arrival: the part can
    # cross the beam before the step that watches for it is even active.
    sq.step("ready", actions=[bt.seq.motion("approach")])
    sq.step("feed", actions=[bt.seq.start("conv")])
    sq.step("await part", transition=bt.seq.rising("part_at_pick"))
    sq.step("halt", actions=[bt.seq.stop("conv")])

    # The gauge reads only while the part stands on the station, and by the
    # time the routing decision matters the part is in the gripper — so the
    # verdict is latched here, into a relay, and read back later. Capturing
    # a measurement while you can is the oldest habit in ladder logic.
    gauge = sq.select("gauge")
    gauge.when(bt.seq.signal("too_tall")).step(
        "mark ng", actions=[bt.seq.set_signal("reject")]
    )
    gauge.when(bt.seq.otherwise()).step(
        "mark ok", actions=[bt.seq.set_signal("reject", False)]
    )

    sq.step("descend", actions=[bt.seq.motion("descend")])
    sq.step(
        "grip",
        actions=[bt.seq.ramp({"finger_joint": SHUT}, 0.5), bt.seq.set_signal("gripped")],
    )
    sq.step("hold", actions=[bt.seq.attach(part, touch_links=PADS)])
    sq.step("lift", actions=[bt.seq.motion("lift")])

    # SFC selection on the latched verdict: good parts are seated on the
    # tray, bad ones go back on the belt and are sent downstream.
    judge = sq.select("judge")
    seat(judge.when(bt.seq.signal("reject", False)), "tray", "to tray", part)
    reject = judge.when(bt.seq.otherwise())
    seat(reject, "purge", "to belt", part)
    reject.step("purge", actions=[bt.seq.start("conv")])

    sq.step("return", actions=[bt.seq.motion("home")])


def seat(arm, motion: str, label: str, part: str) -> None:
    """Put the carried part down like a cell does: over the target, down
    onto it, open, and lift clear before going anywhere else."""
    arm.step(label, actions=[bt.seq.motion(f"over_{motion}")])
    arm.step("lower", actions=[bt.seq.motion(f"to_{motion}")])
    arm.step(
        "release",
        actions=[
            bt.seq.ramp({"finger_joint": OPEN}, 0.4),
            bt.seq.detach(part),
            bt.seq.set_signal("gripped", False),
        ],
    )
    arm.step("clear", actions=[bt.seq.motion(f"clear_{motion}")])


def author_lamp(scene: bt.Scene) -> None:
    """A busy-lamp program driven purely by edges: lit while the gripper
    holds a part. Watching its token wait on the rising edge while the
    `pick` column works is the two-programs-one-scan story at a glance."""
    scene.define_signal("busy")
    lamp = scene.sequence("lamp")
    lamp.step("dark", transition=bt.seq.rising("gripped"))
    lamp.step("lit", actions=[bt.seq.set_signal("busy")], transition=bt.seq.falling("gripped"))
    lamp.step("off", actions=[bt.seq.set_signal("busy", False)])


def main() -> None:
    scene = build_cell()
    teach(scene)
    author_pick(scene)
    author_lamp(scene)
    print(__doc__)
    bt.studio(scene)


if __name__ == "__main__":
    main()
