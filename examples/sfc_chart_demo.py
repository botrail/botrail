"""A pick cell you can watch: the studio's SFC chart over a real cell.

The cell is a UR5e with a Robotiq 2F-85 on a coupling (catalog models —
the first run downloads them), picking 60 mm parts off a running belt.
The belt is real geometry here, not just a transport zone; the gripper
really closes on the part before the attach; and both ends of the cycle
approach the way a cell does — over the target, straight down, release,
lift clear — so what the viewport shows is what the cycle does.

The process is the one the chart is for: feed the belt and pre-position
in parallel, wait for the part to *arrive* (a rising edge on the beam,
not a level — a part already sitting on the beam is not an arrival),
grip, then a ◇ select decides on the spec gauge: a good part is seated on
the tray, a bad one goes back on the belt and is sent downstream out of
the cell. A second program (`lamp`) watches the gripper through edge
conditions, so two columns scan side by side.

    .venv/bin/python examples/sfc_chart_demo.py

Then, in the browser:

1. Sequence tab -> **SFC chart** — the authored programs, neutral.
2. **Simulate (2 programs)** — the taken path stays solid, the reject arm
   dashes out, the winning guard `spec_ok=1` reads green.
3. Press play in the dock: a token rides each column, and the live
   condition beside it colors green as it becomes true (`0.50s` counts
   up, edge atoms underline while the signal is high, and the condition
   that released a step glows for a beat after the token hops).
4. Click any solid step box — the playhead jumps to when it began.
5. Pick the `ng_part` world in RUN and Simulate again: the other arm
   lights up, and the part rides away down the belt instead.
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
PICK_Z = BELT_TOP + PART / 2
HOVER = 0.14  # how far above a seat the arm hovers before coming down
# A carried part is checked as part of the robot, so "resting on" a
# surface reads as a collision. Seat it a few millimetres clear — the
# same margin a real release leaves, and invisible on screen.
SEAT_GAP = 0.004

TRAY_XY = (0.30, -0.42)
# Rejects are set back down on the belt downstream of the beam and sent
# away — the way a line actually purges a bad part.
PURGE_X = 0.52

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
    # The sensor's own hardware, so the beam has something to come out of.
    for side, name in ((-1, "cell/emitter"), (1, "cell/receiver")):
        scene.add_box(name, size=(0.04, 0.03, 0.07),
                      position=(BEAM_X, BELT_Y + side * 0.155, PICK_Z), color=(0.85, 0.55, 0.15))
        scene.set_obstacle_material(name, metalness=0.3, roughness=0.5)

    scene.add_box("part", size=(PART, PART, PART),
                  position=(BELT_X0 + 0.10, BELT_Y, PICK_Z), color=(0.80, 0.58, 0.24))
    scene.set_obstacle_material("part", metalness=0.2, roughness=0.6)

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
    scene.add_beam_sensor(
        "part_at_pick",
        frm=(BEAM_X, BELT_Y - 0.14, PICK_Z),
        to=(BEAM_X, BELT_Y + 0.14, PICK_Z),
        watch=["part"],
    )
    # The spec gauge's verdict is an input contact: good parts by default,
    # and the `ng_part` scenario is the world where it reads low.
    scene.define_signal("spec_ok", initial=True)
    scene.define_signal("gripped")
    scene.add_scenario("ng_part", signals={"spec_ok": False})
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

    approach = at((PICK_X, BELT_Y, PICK_Z + HOVER), OPEN)
    scene.add_segment("approach", goal=approach)
    scene.add_segment("descend", goal=at((PICK_X, BELT_Y, PICK_Z), OPEN))
    scene.add_segment("lift", goal=[*approach[:6], SHUT])
    seat("tray", (*TRAY_XY, TRAY_TOP + 0.02 + PART / 2 + SEAT_GAP))
    seat("purge", (PURGE_X, BELT_Y, PICK_Z + SEAT_GAP))
    scene.add_segment("home", goal=READY)
    scene.set_joint_positions(READY)


def author_pick(scene: bt.Scene) -> None:
    """The process. Steps are the chart's boxes; transitions its bars."""
    sq = scene.sequence("pick")
    sq.step("feed", actions=[bt.seq.start("conv"), bt.seq.motion("approach")])
    sq.step("await part", transition=bt.seq.rising("part_at_pick"))
    sq.step("halt", actions=[bt.seq.stop("conv")])
    sq.step("descend", actions=[bt.seq.motion("descend")])
    sq.step(
        "grip",
        actions=[bt.seq.ramp({"finger_joint": SHUT}, 0.5), bt.seq.set_signal("gripped")],
    )
    sq.step("hold", actions=[bt.seq.attach("part", touch_links=PADS)])
    sq.step("lift", actions=[bt.seq.motion("lift")])

    # SFC selection: the spec gauge decides. Both arms move the robot — a
    # bake takes one of them, and the `ng_part` scenario covers the other.
    # Each seats the part the same way, over different furniture.
    judge = sq.select("judge")
    seat(judge.when(bt.seq.signal("spec_ok")), "tray", "to tray")
    reject = judge.when(bt.seq.otherwise())
    seat(reject, "purge", "to belt")
    # …and send the bad one downstream, out of the cell.
    reject.step("purge", actions=[bt.seq.start("conv")])

    sq.step("return", actions=[bt.seq.motion("home")])


def seat(arm, motion: str, label: str) -> None:
    """Put the carried part down like a cell does: over the target, down
    onto it, open, and lift clear before going anywhere else."""
    arm.step(label, actions=[bt.seq.motion(f"over_{motion}")])
    arm.step("lower", actions=[bt.seq.motion(f"to_{motion}")])
    arm.step(
        "release",
        actions=[
            bt.seq.ramp({"finger_joint": OPEN}, 0.4),
            bt.seq.detach("part"),
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
