"""One source, two outputs: the sequence that drives the simulation also
compiles to a robot controller program with real I/O.

A belt feeds a part to a through-beam; the six-axis arm on its pedestal
comes down onto it, takes it with a vacuum coil, and an SFC branch on the
spec gauge either sets it on the tray to its right or lowers it into the
reject bin to its left. Every station is taught by IK as a hover and a
set-down, so the suction face lands on the part's top face and the part
lands on the tray's insert. `simulate()` bakes the
deterministic timeline (cycle time, timing chart, USD export); the *same*
sequence lowers to URScript — moves from the rollout's own planned paths,
the part-arrival *edge* as a two-stage digital-input wait, the branch as
a wait-any plus `if/elif`, coils as digital-output writes, timers as
sleeps. The name → port wiring is the only thing the controller needs on
top — and it lives on the scene as the cell's I/O map: a robot-controller
node with its standard I/O, and one binding per point the derived I/O
list (`scene.io_points()`) says the cell needs. The same map writes the
I/O list for the electrical drawing (`scene.export_io_list`).

Both branch arms move the robot, which is where *scenarios* come in: one
deterministic bake takes one arm, so exporting it alone is refused (the
other arm was never planned). The scenario sweep bakes the NG world too,
proves every arm ran (`uncovered_arms() == []`), and merges the runs into
one program whose arms each carry their own bake's moves.

Run with:  python examples/export/export_urscript.py [pick_cell.script]
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import botrail as bt

ASSETS = Path(__file__).resolve().parents[1] / "assets"

# The cell's I/O list, as it would appear on the electrical drawing:
# inputs are contacts the program waits on, outputs are coils it drives.
# `wire_cell` puts it on the scene (the form the project keeps and the
# script export reads); the dicts are the same wiring as `to_script`'s
# explicit `inputs=` / `outputs=` arguments, kept here for comparison.
INPUTS = {"part_at_pick": 2, "spec_ok": 3}  # beam → DI2, spec gauge → DI3
OUTPUTS = {"conv": 0, "vacuum": 1}  # conveyor run → DO0, vacuum valve → DO1


def wire_cell(scene: bt.Scene) -> None:
    """The robot controller is the cell master: its standard digital I/O
    takes the beam, the spec gauge, the conveyor drive and the vacuum
    valve. `scene.io_points()` derives which points exist; these lines
    say which channel each one lands on."""
    scene.add_io_node("UR", kind="robot_controller", robots=["simple_arm"],
                      channels=bt.io.ur_standard())
    scene.bind_input("part_at_pick", "UR", "DI2", field="BEAM1")   # photoelectric beam
    scene.bind_input("spec_ok", "UR", "DI3", field="GAUGE")        # spec gauge verdict
    scene.bind_output("conv", "UR", "DO0", field="VFD1")           # conveyor drive run
    scene.bind_output("vacuum", "UR", "DO1", field="YV1")           # vacuum valve


# --- the cell (metres; z = 0 is the shop floor) ------------------------
BASE_Z = 0.72  # the pedestal's top: the arm's mounting plane
BELT_TOP = 0.80  # the conveying height
BELT_Y = 0.42  # the belt's centre line, behind the arm
BELT_X0, BELT_X1 = -1.05, 0.35
BELT_WIDTH = 0.20
PART = 0.06  # the part: a 60 mm block
PART_START_X = -0.62
PICK_X = 0.0  # where a part halts: straight in front of the arm
# The beam trips on the part's leading edge, so it sits half a part
# downstream of the pick: a halted part stands centred under the tool.
BEAM_X = PICK_X + PART / 2
TABLE_TOP = 0.70
TRAY_XY = (0.44, -0.02)  # the good-parts tray, right of the arm
TRAY = (0.30, 0.22, 0.03)
BIN_XY = (-0.43, 0.04)  # the reject bin, left of the arm
BIN = (0.36, 0.28, 0.18)
# The bin stands high, its rim a little under the belt: a part carried over
# from the pick clears the belt's near rail on its way, and the arm reaches
# over the rim without folding its wrist into its forearm.
BIN_STAND = 0.56
TOOL_FACE = 0.04  # the suction face, this far beyond `tool0`
HOVER = 0.10  # how far above a station the arm comes over before going down
# The suction face stops this far above the part's top: inside the grasp's
# reach (`GRASP_REACH`), never touching the part before the vacuum has it.
PICK_GAP = 0.002
GRASP_REACH = 0.01
# A carried part is checked as part of the robot, so "resting on" a
# surface reads as a collision: it is set down this far clear — the margin
# a real release leaves, invisible on screen.
SEAT_GAP = 0.004
# The wrist's roll joint stands a quarter turn round in every taught pose.
# At zero it would line the last two axes up: harmless on this arm, but the
# wrist singularity of the UR arm the exported program runs on, where a
# straight-line move would make the controller flip its wrist and stop.
WRIST = math.pi / 2
#: Where the cycle starts and ends: folded over the front, tool down.
READY = [0.0, -0.3, 2.0, math.pi - 1.7, WRIST, 0.0]
# Colours are linear (the studio and USD both read them so): a mid grey is
# about 0.2, not 0.5.
PART_COLOR = (0.12, 0.40, 0.68)
TABLE_FRAME = (0.10, 0.11, 0.12)
TABLE_TOP_COLOR = (0.035, 0.04, 0.045)
REJECT_RED = (0.52, 0.05, 0.04)


def build_cell() -> bt.Scene:
    """The cell: the arm on its pedestal, the belt behind it with the
    through-beam at the pick, the good-parts tray on a table to its right,
    the reject bin on a stand to its left — and the motions taught by IK
    against them (`teach`)."""
    # The arm stands on the pedestal's mount, turned to face the belt (+y)
    # at J1 = 0, so its working range — tray right, belt ahead, bin left —
    # is centred in the base joint's. Its base plate sits on the top plate.
    scene = bt.Scene(bt.Robot.from_urdf(ASSETS / "simple_arm.urdf"))
    bt.parts.pedestal(scene, "pedestal", height=BASE_Z, position=(0.0, 0.0), top=(0.24, 0.24), base=(0.40, 0.40),
                      column=0.14, yaw=math.pi / 2)
    scene.set_robot_base_pose(*scene.frame("pedestal/mount"))
    scene.allow_link_obstacle_contact("base_link", "pedestal/top")

    # A belt at the conveying height running +x at 0.12 m/s — slow enough
    # that the arm is in position well before the part arrives, which is
    # what makes waiting on the *edge* sound. The part is a 60 mm block
    # waiting upstream.
    bt.parts.conveyor(scene, "conv", length=BELT_X1 - BELT_X0, width=BELT_WIDTH,
                      position=((BELT_X0 + BELT_X1) / 2, BELT_Y, BELT_TOP), speed=0.12, running=False)
    scene.add_box("part", size=(PART, PART, PART), position=(PART_START_X, BELT_Y, BELT_TOP + PART / 2 + 1e-4),
                  color=PART_COLOR)
    scene.set_obstacle_material("part", metalness=0.55, roughness=0.35)
    # The beam crosses the belt half a part downstream of the pick point: it
    # trips on the leading edge, so a halted part stands centred under the
    # taught pick. The spec gauge's verdict arrives as an input contact
    # (`spec_ok`, DI3) — good parts by default; the `ng_part` scenario is
    # the world where it reads low.
    bt.parts.photoelectric(scene, "part_at_pick",
                           frm=(BEAM_X, BELT_Y + 0.17, BELT_TOP + PART / 2),
                           to=(BEAM_X, BELT_Y - 0.17, BELT_TOP + PART / 2), watch=["part"])
    # Good parts go on a tray on the table to the arm's right, rejects into
    # the red bin on the stand to its left.
    bt.parts.table(scene, "out_table", size=(0.36, 0.32, TABLE_TOP), position=TRAY_XY, color=TABLE_FRAME,
                   top_color=TABLE_TOP_COLOR)
    bt.parts.tray(scene, "tray", size=TRAY, position=(*TRAY_XY, TABLE_TOP))
    bt.parts.table(scene, "reject_stand", size=(0.38, 0.32, BIN_STAND), position=BIN_XY, color=TABLE_FRAME,
                   top_color=TABLE_TOP_COLOR)
    bt.parts.bin(scene, "reject_bin", size=BIN, position=(*BIN_XY, BIN_STAND), color=REJECT_RED)
    # The floor: the arm's keep-out line.
    bt.parts.marking(scene, "floor/keep_out", rect=(-0.85, -0.42, 0.85, 0.62), dash=(0.20, 0.10), width=0.05)

    scene.define_signal("spec_ok", initial=True)
    scene.define_signal("vacuum")
    teach(scene)

    # The test-case matrix: the row FAT would call "NG 品を流す".
    scene.add_scenario("ng_part", signals={"spec_ok": False})
    return scene


def _down(x: float, y: float) -> tuple[float, float, float, float]:
    """The tool pointing at the floor over (x, y), turned so the arm reaches
    it with its wrist roll at `WRIST`."""
    half = (math.atan2(y, x) + math.pi - WRIST) / 2
    return (math.cos(half), math.sin(half), 0.0, 0.0)


def teach(scene: bt.Scene) -> None:
    """Teach the motions by IK, so the suction face really lands on the
    part. Every station is a pair — a hover `HOVER` above it and the
    station itself — and the arm comes over, straight down, and back up
    the same line: a part is never dragged sideways at working height.

    `to_pick` / `pick_down` take the arm over the halted part and onto its
    top face (`PICK_GAP` clear: within the grasp's reach, never touching
    before the vacuum takes it). `place` and `to_reject` lift the part
    clear of the belt, carry it over the tray or the bin and lower it in;
    `clear_tray` / `clear_bin` lift the empty tool back out, and `home`
    returns to `READY`, where the cycle starts."""

    def at(x: float, y: float, face_z: float) -> list[float]:
        # Solve from the arm turned toward the station, elbow up, so every
        # pose is the same branch and the moves between them stay short.
        scene.set_joint_positions([math.atan2(y, x) - math.pi / 2, 0.5, 1.2, math.pi - 1.7, WRIST, 0.0])
        ik = scene.set_tcp_target((x, y, face_z + TOOL_FACE), _down(x, y))
        if not ik.converged:
            raise RuntimeError(f"the cell layout is out of reach at ({x:.3f}, {y:.3f}, {face_z:.3f})")
        return list(scene.joint_positions)

    pick_z = BELT_TOP + PART + PICK_GAP  # the suction face over the halted part
    over_pick = at(PICK_X, BELT_Y, pick_z + HOVER)
    # The set-downs come from the furniture: the tray's insert and the
    # bin's floor are frames its generator put there.
    tray_z = scene.frame("tray/seat")[0][2] + PART + SEAT_GAP
    bin_z = scene.frame("reject_bin/floor")[0][2] + PART + SEAT_GAP
    over_tray = at(*TRAY_XY, tray_z + HOVER)
    over_bin = at(*BIN_XY, BIN_STAND + BIN[2] + PART + HOVER)  # the part's underside clears the rim

    scene.add_segment("to_pick", goal=over_pick)
    scene.add_segment("pick_down", goal=at(PICK_X, BELT_Y, pick_z), kind="cartesian_line")
    for motion, over, down in (("place", over_tray, at(*TRAY_XY, tray_z)), ("to_reject", over_bin, at(*BIN_XY, bin_z))):
        scene.add_segment(motion, goal=over_pick, kind="cartesian_line")  # lift clear of the belt
        scene.add_segment(motion, goal=over)
        scene.add_segment(motion, goal=down, kind="cartesian_line")
    scene.add_segment("clear_tray", goal=over_tray, kind="cartesian_line")
    scene.add_segment("clear_bin", goal=over_bin, kind="cartesian_line")
    scene.add_segment("home", goal=READY)
    scene.set_joint_positions(READY)


def author_sequence(scene: bt.Scene, coil_grasp: bool = False) -> bt.seq.SequenceBuilder:
    """The pick program. With ``coil_grasp`` the grasp is the vacuum coil's:
    the plant takes ``part`` when ``vacuum`` rises with the suction face
    within ``GRASP_REACH`` of it, and releases it when the coil falls — the
    program never ``attach``es, since a PLC has no attach command; this is
    the authoring a controller under test can run (`bt.vc`,
    design-virtual-commissioning.md VC2)."""
    if coil_grasp:
        scene.bind_grasp("vacuum", parts=["part"], reach_m=GRASP_REACH)
    sq = scene.sequence("pick")
    # Belt on and pre-position in parallel; then wait for the part to
    # *arrive* — a rising edge. A part already sitting on the beam from a
    # broken cycle is a level, not an arrival, and an edge that fires
    # while the arm is still travelling would be missed — which is why
    # the wait is its own step, entered once the arm is ready.
    sq.step("feed", actions=[bt.seq.start("conv"), bt.seq.motion("to_pick")])
    sq.step("await part", transition=bt.seq.rising("part_at_pick"))
    sq.step("halt", actions=[bt.seq.stop("conv")])
    sq.step("descend", actions=[bt.seq.motion("pick_down")])
    sq.step("grip", actions=[bt.seq.set_signal("vacuum")], transition=bt.seq.elapsed(0.3))
    if not coil_grasp:
        sq.step("hold", actions=[bt.seq.attach("part")])
    release = [bt.seq.set_signal("vacuum", False)] + ([] if coil_grasp else [bt.seq.detach("part")])
    # SFC selection: the spec gauge decides. Both arms move the robot —
    # a bake takes one of them, and the scenario sweep covers the other.
    # Each lets go where it set the part down and lifts the empty tool out.
    judge = sq.select("judge")
    judge.when(bt.seq.signal("spec_ok")).step(
        "place", actions=[bt.seq.motion("place")]
    ).step("release", actions=[*release, bt.seq.motion("clear_tray")])
    reject = judge.when(bt.seq.otherwise())
    reject.step("to reject", actions=[bt.seq.motion("to_reject")])
    reject.step("drop", actions=[*release, bt.seq.motion("clear_bin")])
    sq.step("return", actions=[bt.seq.motion("home")])
    return sq


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else ASSETS.parents[1] / "pick_cell.script"

    scene = build_cell()
    author_sequence(scene)
    wire_cell(scene)
    # Every derived point has a channel, nothing clashes: the I/O list is
    # ready for the drawing.
    report = scene.io_report()
    assert report.ok, report
    io_csv = out.with_name("pick_cell_io.csv")
    scene.export_io_list(io_csv)
    print(f"wrote {io_csv} — the I/O list:\n{scene.io_list('md')}")

    # The row FAT calls "beam broken" is a scenario too: a stuck sensor
    # is a forced input, not a different program. That row cannot
    # complete — the sweep collects the stall, with the forced point in
    # the diagnosis, next to the rows that did.
    scene.add_scenario("beam_stuck", faults=[bt.io.stuck("part_at_pick", False)])

    # The whole test-case matrix, one deterministic bake per world.
    runs = scene.simulate_scenarios(["pick"], max_duration=30.0)
    for name, tl in runs.items():
        path = " → ".join(step for step, _, _ in tl.step_spans if "/" not in step)
        print(f"{name:<10} cycle {tl.duration:6.2f}s  ({path})")
    for name, error in runs.errors.items():
        print(f"{name:<10} FAILED: {error}")
    assert set(runs.errors) == {"beam_stuck"}, runs.errors
    assert runs.uncovered_arms() == [], runs.uncovered_arms()
    print("branch coverage: every arm exercised\n")

    # One bake alone cannot compile the branch it skipped:
    try:
        runs["baseline"].to_script()
    except ValueError as e:
        print(f"baseline alone: {e}\n")

    # The sweep can — each arm's moves come from the bake that took it.
    # The ports come from the bindings on the UR node; passing the dicts
    # explicitly (`inputs=INPUTS, outputs=OUTPUTS`) gives the same script.
    runs.export_script(out)
    print(f"wrote {out} — the whole matrix, as one controller program:\n")
    print(runs.to_script())


if __name__ == "__main__":
    main()
