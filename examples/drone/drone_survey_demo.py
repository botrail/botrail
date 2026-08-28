"""A cycle-count drone and a case palletizer share one warehouse aisle.

Both machines are ordered, not drawn. The palletizer is a UR12e — 12.5 kg
at 1.3 m, the cobot class sold for case palletizing — wearing a Schmalz
ECBPi vacuum gripper on the ISO 9409-1 adapter plate the gripper's own
manifest says it needs. The drone is the PX4 X500 airframe, a *robot* rigid-mounted
on an aerial vehicle (the exact symmetry of a quadruped with a gait on a
differential one), so its interference is computed link by link and its
propellers check as their swept discs. The racks and the case infeed are
catalog products too, sized to what those packages actually sell.

The work is the work a warehouse does. Cases index up the infeed one pitch
at a time; the TM20 picks each off the belt, swings it across to the
staging pallet at the mouth of the aisle and sets it down, two to a course.
The drone flies an inventory count: out of its dock, down the aisle, and up
and down the rack faces in a serpentine — bay 1 bottom to top, bay 2 top to
bottom, bay 3 bottom to top — reading one location per stop with a
side-looking scanner that keeps facing the racks because the aerial drive
holds a fixed yaw. Nine locations are expected. Eight answer. **Finding the
ninth missing is what a cycle count is for**, and the empty shelf is in the
cell so the count has something to find.

The two machines cannot both be at the mouth of the aisle. The dock sits
past the palletizing cell, so every flight in and out crosses the airspace
the arm swings cases through — geometry alone forbids this cell. What
permits it is a zone handshake, the one a real WMS runs: the drone asks
from its pad (`count_request`) rather than burning battery holding in the
air; the palletizer finishes the case in its cup — it does not drop one
because a drone asked — parks over its own infeed and answers
(`aisle_clear`); the drone launches, counts and comes home; landing raises
`count_done` and the palletizer picks up where it left off. The timing
chart shows the whole conversation, and the shift prices it: the count
costs the palletizer a measurable block of standing by, which is the
number a planner actually wants before agreeing to fly one.

Run `--no-interlock` and nothing about the paths, the volumes or the
machines changes. Only the clock does: the drone crosses the mouth while a
case is still in the air, and the bake refuses at the instant they meet,
naming a link of each. That is what the cross-robot check is *for* — not
proving a flight clears parked scenery, but pricing two machines' motions
against each other in time.

    python examples/drone/drone_survey_demo.py                 # bake + USD
    python examples/drone/drone_survey_demo.py --no-interlock  # right paths, wrong clock
    python examples/drone/drone_survey_demo.py --low           # a crossing under the stack
    python examples/drone/drone_survey_demo.py --studio        # watch the shift

`--airframe <dir>` flies a package the catalog builder wrote instead of the
published one.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import botrail as bt

HERE = Path(__file__).resolve().parent

# ---- what is ordered -------------------------------------------------
ARM_PACK = "universal_robots/ur12e"                  # 12.5 kg at 1.3 m — the palletizing cobot class
ADAPTER = "botrail/adapter/flange-plate-ecbpi"       # the plate its manifest requires
CUP = "botrail/vacuum/vacuum-gripper-ecbpi"          # Schmalz ECBPi, 10 kg
AIRFRAME = "px4/x500/x500"                           # the inventory drone
RACK_PACK = "botrail/rack/medium-shelf"
BELT_PACK = "botrail/conveyor/belt-unit"

# ---- the palletizing cell, at the mouth of the aisle -----------------
RISER = 0.30                     # a low plinth: the work sits above the shoulder
ARM_XY = (1.5, -0.85)
BELT_MID, BELT_Y = 1.0, -1.60    # the case infeed, running towards the pick
BELT_LEN, BELT_W, BELT_TOP = 3.0, 0.4, 0.75
PITCH = 0.45                     # one index brings the next case to the stop
CASE = (0.36, 0.28, 0.24)        # a shipper carton
CASE_KG = 6.0
CASES = 4
PALLET_XY = (1.5, 0.12)          # the staging pallet, in the aisle mouth
DECK = 0.144                     # EPAL deck height
SLOTS = (-0.145, 0.145)          # two cases to a course
SEAT = 0.005                     # everything this cell sets down stands proud of
                                 # what it stands on: a case resting *on* the belt
                                 # is a carried part touching an obstacle the moment
                                 # the cup takes it
HOVER = 0.25                     # approach and retract standoff

# ---- the aisle -------------------------------------------------------
BAYS = (3.5, 5.5, 7.5)           # bay centres down the aisle
BAY_Y = 1.5                      # rack centreline; 0.6 deep, so the face is at 1.2
RACK = (1.8, 0.6, 1.8)           # width, depth, height — all sizes the pack sells
LEVELS = 3
SHELF_Z = tuple(RACK[2] * (i + 1) / LEVELS for i in range(LEVELS))   # 0.6 / 1.2 / 1.8
SCAN_Z = tuple(z + 0.15 for z in SHELF_Z)                            # label height
TOTE = (0.6, 0.4, 0.3)
EMPTY = (1, 2)                   # bay 2, top level: the location that will not answer

# ---- the flight ------------------------------------------------------
DOCK = (0.0, 0.0)
PAD = 0.10                       # the dock pad the gear stands on
LANE_Z = 1.2                     # the crossing altitude at the aisle mouth
OVER = 2.5                       # transit home, above the racks
HOLD_X = 0.6                     # the hold point, outside the arm's envelope
# Indoor rates. The airframe is rated far faster (PX4's multicopter limits,
# `specs.max_*`); a rack aisle is not the place to use them, and the cell is
# what says so — the catalog caps, the cell chooses.
SPEED, CLIMB, DESCENT = 0.8, 0.6, 0.9
SPIN = 25.0                      # propeller rate, rad/s (see `spin=` below)
SCAN_DWELL = 1.5                 # a location is read, not glanced at
BELT_SPEED = 0.25

READY = [0.0, -1.9, 1.9, -1.6, -math.pi / 2, 0.0]   # arm up, cup down


def down(yaw: float) -> tuple:
    """Tool +Z at the floor, the cup square to `yaw`. Aiming along the
    reach keeps the last joint near zero — a wound wrist costs seconds
    later, unwinding."""
    return (math.cos(yaw / 2), math.sin(yaw / 2), 0.0, 0.0)


def slot_of(case: int, pallet_y: float = PALLET_XY[1]) -> tuple[float, float]:
    """Where case `case` lands: two to a course, courses upwards. The
    build order is the order a palletizer actually stacks in."""
    course, place = divmod(case, len(SLOTS))
    return pallet_y + SLOTS[place], DECK + (course + 1) * (CASE[2] + SEAT)


def palletizer():
    """The TM20 with its plate and cup, as one machine.

    The vacuum gripper's manifest names the adapter it needs
    (`ISO 9409-1-50-4-M6 -> flange-plate-ecbpi`); composing the three in
    that order is what makes the tool centre point land where the datasheet
    says it does, and what puts three lines on the bill instead of one."""
    arm = bt.Robot.from_catalog(ARM_PACK)
    return (arm
            .attach_tool(bt.Robot.from_catalog(ADAPTER), prefix="adp_")
            .attach_tool(bt.Robot.from_catalog(CUP)))


def airframe(pack=AIRFRAME):
    """The drone as a `Robot`, with its manifest.

    A UAV is a robot mounted on an aerial vehicle. Being a robot is what
    buys it interference computation: its links against the racks while it
    flies (`RiderCollision`), its links against the palletizer's links
    every tick (`RobotCollision`), and the live distance readout while you
    author."""
    directory = Path(pack)
    if directory.is_dir():
        # A package the catalog builder wrote: the URDF carries the meshes,
        # the manifest the identity and the rates.
        return bt.Robot.from_urdf(directory / "urdf" / "model.urdf"), _manifest(directory)
    return bt.Robot.from_catalog(str(pack)), _manifest(Path(bt.catalog_package(str(pack))))


def _manifest(directory: Path) -> dict:
    import yaml

    return yaml.safe_load((directory / "manifest.yaml").read_text(encoding="utf-8")) or {}


def build(*, interlock: bool = True, lane: float = LANE_Z,
          pallet_y: float = PALLET_XY[1], pack=AIRFRAME) -> bt.Scene:
    scene = bt.Scene(palletizer(), name="palletizer",
                     base_position=(*ARM_XY, RISER + 0.005))
    scene.set_joint_positions(READY)

    # -- the palletizing cell ------------------------------------------
    riser = bt.parts.pedestal(scene, "riser", height=RISER, position=ARM_XY, diameter=0.34)
    scene.set_obstacle_enabled(f"{riser.name}/column", False)   # the arm stands on it
    bt.parts.conveyor(scene, "infeed", BELT_LEN, BELT_W,
                             (BELT_MID, BELT_Y, BELT_TOP),
                             catalog=BELT_PACK, speed=BELT_SPEED)
    bt.parts.pallet(scene, "staging", (PALLET_XY[0], pallet_y), model="EPAL 1")

    # Four cases waiting their turn on the infeed, one index apart. The
    # belt's transport zone carries them: an index is a distance, not a
    # timer, so the stop is where the case stops.
    for i in range(CASES):
        scene.add_box(f"case{i}", size=CASE,
                      position=(ARM_XY[0] - (i + 1) * PITCH, BELT_Y, BELT_TOP + SEAT + CASE[2] / 2),
                      color=(0.72, 0.56, 0.36))
        scene.set_part(f"case{i}", category="workpiece", model="RSC-360x280x240",
                       mass_kg=CASE_KG)
    # Case-in-position: the photo-eye across the belt at the index stop.
    scene.add_beam_sensor("case_eye", frm=(ARM_XY[0], BELT_Y - 0.3, BELT_TOP + SEAT + CASE[2] / 2),
                          to=(ARM_XY[0], BELT_Y + 0.3, BELT_TOP + SEAT + CASE[2] / 2))

    # -- the aisle the drone counts ------------------------------------
    for bay, x in enumerate(BAYS, start=1):
        bt.parts.rack(scene, f"bay{bay}", RACK, (x, BAY_Y), catalog=RACK_PACK, levels=LEVELS)
        for level, shelf in enumerate(SHELF_Z):
            if (bay - 1, level) == EMPTY:
                continue      # the location the count will fail to find
            scene.add_box(f"bay{bay}/tote{level}", size=TOTE,
                          position=(x, BAY_Y, shelf + TOTE[2] / 2),
                          color=(0.20, 0.34, 0.46))
            scene.set_part(f"bay{bay}/tote{level}", category="workpiece",
                           model="TOTE-600x400", mass_kg=11.0)

    # -- the drone ------------------------------------------------------
    scene.add_box("dock", size=(0.9, 0.9, PAD), position=(*DOCK, PAD / 2),
                  color=(0.22, 0.24, 0.27))
    model, spec = airframe(pack)
    scene.add_robot(model, name="drone")
    lane_y = 0.0
    scene.add_vehicle(
        "drone", body=[],
        path=[(*DOCK, PAD + 0.005),                     # 0  on the pad
              (*DOCK, lane),                            # 1  climb out
              (HOLD_X, lane_y, lane),                   # 2  hold, clear of the arm
              (2.5, lane_y, lane),                      # 3  across the mouth
              (2.5, lane_y, SCAN_Z[0])]                 # 4  down to the first face
        + [(x, lane_y, SCAN_Z[level])                   # 5..13 the serpentine
           for i, x in enumerate(BAYS)
           for level in (range(LEVELS) if i % 2 == 0 else reversed(range(LEVELS)))]
        + [(BAYS[-1], lane_y, OVER),                    # 14 climb over the racks
           (2.5, lane_y, OVER),                         # 15 transit home
           (2.5, lane_y, lane)],                        # 16 down to the mouth again
        stations={"dock": 0, "hold": 2,
                  **{f"b{i + 1}l{level}": 5 + 3 * i + n
                     for i, _ in enumerate(BAYS)
                     for n, level in enumerate(range(LEVELS) if i % 2 == 0
                                               else reversed(range(LEVELS)))}},
        ring=True,          # 16 closes back to the dock: one diagonal home
        speed=SPEED, start="dock",
        drive="aerial", climb_speed=CLIMB, descent_speed=DESCENT,
        fixed_yaw=0.0,      # the scanner must keep facing the racks
    )
    # The machine rides its vehicle rigidly, gear on the pad. The props turn
    # while it flies — presentation, not physics: the checks read the swept
    # discs whatever the phase, so the rate is free to be readable.
    rotors = sorted(j for j in model.joint_names if "rotor" in j)
    scene.mount_robot("drone", robot="drone",
                      spin={name: SPIN if i < len(rotors) / 2 else -SPIN
                            for i, name in enumerate(rotors)})
    numeric = {k: float(v) for k, v in (spec.get("specs") or {}).items()
               if isinstance(v, (int, float)) and not isinstance(v, bool)}
    if Path(pack).is_dir():
        scene.set_part("drone", kind="robot", catalog=spec.get("id"),
                       manufacturer=(spec.get("manufacturer") or {}).get("name"),
                       model=spec.get("name"), category=spec.get("category"), **numeric)
    # The scanner: a side-looking read window riding the airframe, deep
    # enough to reach the far side of a tote and no taller than the label
    # it reads — a window as deep as a shelf pitch would answer two
    # locations at once and count neither.
    scene.add_zone_sensor("scan", position=(0.0, 0.9, 0.0), size=(0.5, 1.4, 0.12),
                          watch=[f"bay{b}/tote{n}" for b in (1, 2, 3) for n in range(LEVELS)],
                          mount="drone")

    # The cup touches the case it lifts — that is what picking is, so it
    # is declared rather than discovered at bake time.
    for i in range(CASES):
        for link in ("tcp", "body"):
            scene.allow_link_obstacle_contact(link, f"case{i}", robot="palletizer")

    teach(scene, pallet_y)
    programs(scene, interlock=interlock)
    return scene


def teach(scene: bt.Scene, pallet_y: float) -> None:
    """Every pose the palletizer works from, solved against the machine's
    own kinematics rather than typed in — swap the arm and the cell
    re-teaches itself, or refuses by name."""
    limits = scene.robot_of("palletizer").joint_limits

    def unwind(q: list, seed: list) -> list:
        """The same pose, with each wrist turned the short way.

        A 6-axis IK hands back an angle, not a winding: two solutions a
        full turn apart put the cup in exactly the same place, and taking
        the wrong one spends a silent 360 deg unwinding between the pick
        and the place. Nothing in the picture says so — the arm just
        takes longer and swings where it should not."""
        out = []
        for value, want, limit in zip(q, seed, limits):
            lo, hi = limit or (-math.inf, math.inf)
            best = value
            for turn in (-1, 1):
                other = value + turn * 2 * math.pi
                if lo - 1e-9 <= other <= hi + 1e-9 and abs(other - want) < abs(best - want):
                    best = other
            out.append(best)
        return out

    def pose(name: str, target, yaw: float, *seeds: list) -> list:
        """Solve, unwind, and refuse a pose that fouls — in that order.

        A 6-axis IK has several branches, and which one it hands back is
        decided by where it started. Seeding from the pose the arm
        actually arrives in keeps the cycle in one branch; falling back to
        the ready pose is what a programmer does at the teach pendant when
        the arm folds onto itself."""
        short, fouled = None, []
        for seed in seeds:
            scene.set_joint_positions(seed, robot="palletizer")
            ik = scene.set_tcp_target(target, down(yaw), robot="palletizer")
            if not ik.converged:
                short = ik.pos_error
                continue
            q = unwind(list(scene.joint_positions_of("palletizer")), seed)
            scene.set_joint_positions(q, robot="palletizer")
            hits = [f"{a[1]} x {b[1]}" for a, b in scene.check_collisions()]
            if not hits:
                scene.add_segment(name, goal=q, robot="palletizer")
                return q
            fouled = hits
        if short is not None and not fouled:
            raise RuntimeError(
                f"{ARM_PACK} cannot reach {tuple(round(v, 2) for v in target)} "
                f"from a {RISER:.2f} m plinth at {ARM_XY} — {short * 1e3:.0f} mm "
                f"short of it. Move the plinth, or order the longer arm.")
        raise RuntimeError(f"every branch taught for `{name}` fouls: {', '.join(fouled)}")

    # The index stop, approached from above the way every case pick is.
    pick_hi = pose("pick_hi", (ARM_XY[0], BELT_Y, BELT_TOP + SEAT + CASE[2] + HOVER),
                   -math.pi / 2, READY)
    pose("pick_lo", (ARM_XY[0], BELT_Y, BELT_TOP + SEAT + CASE[2]), -math.pi / 2,
         pick_hi, READY)
    # Where each case is set down, and the clearance over it. Each is
    # seeded from the pose the arm actually arrives in, so the cycle stays
    # in one branch of the arm's kinematics from the first case to the last.
    for case in range(CASES):
        y, top = slot_of(case, pallet_y)
        hi = pose(f"set{case}_hi", (PALLET_XY[0], y, top + HOVER), math.pi / 2, pick_hi, READY)
        pose(f"set{case}_lo", (PALLET_XY[0], y, top), math.pi / 2, hi, READY)
    # Parked over its own infeed: out of the aisle, and out of the way of
    # anything the drone does at the mouth.
    pose("parked", (ARM_XY[0], BELT_Y + 0.35, BELT_TOP + 0.55), -math.pi / 2, READY, pick_hi)
    scene.add_segment("ready", goal=READY, robot="palletizer")


def programs(scene: bt.Scene, *, interlock: bool) -> None:
    scene.define_signal("vacuum", initial=False)
    if interlock:
        scene.define_signal("count_request", initial=False)
        scene.define_signal("aisle_clear", initial=False)
        scene.define_signal("count_done", initial=False)

    pal = scene.sequence("palletize")

    def case_cycle(case: int) -> None:
        pal.step(f"index{case}", actions=[bt.seq.advance("infeed", PITCH)],
                 transition=bt.seq.all_of(bt.seq.device_done("infeed"),
                                          bt.seq.signal("case_eye")))
        pal.step(f"reach{case}", actions=[bt.seq.motion("pick_hi")], transition=bt.seq.done())
        pal.step(f"descend{case}", actions=[bt.seq.motion("pick_lo")], transition=bt.seq.done())
        pal.step(f"grip{case}", actions=[bt.seq.attach(f"case{case}", touch_links=["tcp", "body"], robot="palletizer"),
                                         bt.seq.set_signal("vacuum", True)],
                 transition=bt.seq.elapsed(0.3))     # the cup pulls down
        pal.step(f"lift{case}", actions=[bt.seq.motion("pick_hi")], transition=bt.seq.done())
        pal.step(f"swing{case}", actions=[bt.seq.motion(f"set{case}_hi")],
                 transition=bt.seq.done())
        pal.step(f"place{case}", actions=[bt.seq.motion(f"set{case}_lo")],
                 transition=bt.seq.done())
        pal.step(f"release{case}", actions=[bt.seq.detach(f"case{case}"),
                                            bt.seq.set_signal("vacuum", False)],
                 transition=bt.seq.elapsed(0.3))     # and lets go
        pal.step(f"clear{case}", actions=[bt.seq.motion(f"set{case}_hi")],
                 transition=bt.seq.done())

    case_cycle(0)
    case_cycle(1)
    if interlock:
        # The zone handshake, between cases: a palletizer does not drop the
        # one in its cup because a drone asked.
        pal.step("hand_over", transition=bt.seq.signal("count_request"))
        pal.step("park", actions=[bt.seq.motion("parked")], transition=bt.seq.done())
        pal.step("grant", actions=[bt.seq.set_signal("aisle_clear", True)],
                 transition=bt.seq.immediately())
        pal.step("stand_by", transition=bt.seq.signal("count_done"))
        pal.step("take_back", actions=[bt.seq.set_signal("aisle_clear", False),
                                       bt.seq.motion("ready")], transition=bt.seq.done())
    case_cycle(2)
    case_cycle(3)
    pal.step("home", actions=[bt.seq.motion("ready")], transition=bt.seq.done())

    count = scene.sequence("count")
    if interlock:
        # Asked for from the pad, not from a hover: a drone that waits in
        # the air for its window spends the battery it came to use.
        count.step("request", actions=[bt.seq.set_signal("count_request", True)],
                   transition=bt.seq.immediately())
        count.step("permit", transition=bt.seq.signal("aisle_clear"))
    count.step("launch", actions=[bt.seq.goto("drone", "hold")],
               transition=bt.seq.device_done("drone"))
    for bay, _ in enumerate(BAYS, start=1):
        for level in (range(LEVELS) if bay % 2 else reversed(range(LEVELS))):
            count.step(f"fly_b{bay}l{level}", actions=[bt.seq.goto("drone", f"b{bay}l{level}")],
                       transition=bt.seq.device_done("drone"))
            count.step(f"read_b{bay}l{level}", transition=bt.seq.elapsed(SCAN_DWELL))
    count.step("home", actions=[bt.seq.goto("drone", "dock")],
               transition=bt.seq.device_done("drone"))
    if interlock:
        count.step("report", actions=[bt.seq.set_signal("count_done", True)],
                   transition=bt.seq.immediately())


def bake(*, interlock: bool = True, lane: float = LANE_Z,
         pallet_y: float = PALLET_XY[1], pack=AIRFRAME):
    scene = build(interlock=interlock, lane=lane, pallet_y=pallet_y, pack=pack)
    return scene, scene.simulate_sequences(["palletize", "count"], max_duration=300.0)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("out", nargs="?", default=str(HERE / "drone_cell.usdc"))
    parser.add_argument("--no-interlock", dest="no_interlock", action="store_true",
                        help="drop the zone handshake: same geometry, wrong clock, refused")
    parser.add_argument("--low", action="store_true",
                        help="cross the mouth below the staging stack, refused by name")
    parser.add_argument("--airframe", default=AIRFRAME,
                        help="the UAV pack: a catalog id or a package directory")
    parser.add_argument("--studio", action="store_true")
    args = parser.parse_args()

    if args.no_interlock:
        try:
            bake(interlock=False, pack=args.airframe)
        except ValueError as err:
            print("refused, as it should be:")
            print(f"  {err}")
            return
        raise SystemExit("crossing the mouth with a case in the air should have been refused")

    if args.low:
        try:
            bake(lane=0.5, pack=args.airframe)
        except ValueError as err:
            print("refused, as it should be:")
            print(f"  {err}")
            return
        raise SystemExit("a crossing under the staging stack should have been refused")

    scene, tl = bake(pack=args.airframe)
    lanes = dict(tl.signals)
    read = sum(1 for _, v in lanes["scan"] if v)
    expected = len(BAYS) * LEVELS
    # What the handshake actually cost: not "time not moving" — a
    # palletizer is not moving while the belt indexes either — but the
    # block it stood by with the aisle granted away.
    stood_by = sum(t1 - t0 for name, t0, t1 in tl.step_spans
                   if name in ("palletize/park", "palletize/stand_by"))
    airborne = tl.vehicle_airborne("drone")
    rated = scene.requirements(timeline=tl)["drone"].attributes.get("flight_time_min")
    print(f"shift {tl.duration:.1f} s — {CASES} cases palletized, "
          f"{read} of {expected} locations answered")
    print(f"  bay {EMPTY[0] + 1} level {EMPTY[1] + 1} did not answer: that is the count "
          f"doing its job, and the reason to fly one")
    print(f"  the aisle handshake cost the palletizer {stood_by:.1f} s parked and standing by")
    print(f"  {airborne / 60:.1f} min airborne of the {rated:.0f} min the airframe is rated for")
    tl.export_usd(args.out, fps=30)
    print(f"wrote {args.out}")
    if args.studio:
        bt.studio(scene)


if __name__ == "__main__":
    main()
