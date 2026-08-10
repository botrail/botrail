"""A two-station body-in-white line: three programs, one takt.

This is the W1 slice of design/design-weld-line.md — the smallest build
that is a *line* rather than a station, and the demo the parallel-sequence
engine work exists for:

* **One program per station, plus a transfer program.** Station 1 (geo)
  puts three spots a side on the front half of the flank seam; station 2
  (respot) adds two a side on the rear half; the transfer program owns the
  conveyor and the body sources. Each program is a plain serial SFC — the
  same authoring as every earlier demo — and `simulate_sequences` scans all
  three every tick. Nothing here could be written as one sequence without
  hand-weaving the stations into a single total order.

* **Signals are the interlock.** Stations report `st1_done` / `st2_done`;
  the transfer waits for the stations that actually hold a body, indexes
  the line one pitch with `advance` (exact to the millimetre — no
  `elapsed(pitch/v)` arithmetic), and drops `moving`, which is what
  releases the stations onto the next body. Run with `--clash` to see why
  the gate is load-bearing: advancing without waiting slides the body out
  from between the electrodes mid-cycle, and the spots land on air metres
  past the datum. The rollout reports nothing — every taught pose is
  still collision-free, the process is simply not happening — which is
  exactly why the datum assertion, not the collision check, is the test
  that owns this failure.

* **The line takt is the slowest station plus the transfer.** While
  station 1 welds body 2, station 2 is welding body 1 — the overlap is the
  whole argument for parallel programs, and `main` prints it.

Where the seam is cut between stations is the line-balancing knob, and
`SEAM_SPLITS` is the whole of it. The gun attitude is not part of the
choice: a spot ahead of a station's datum is reached leaning one way and
one behind it the other (design-weld-line.md §4.3), so `attitude_of`
reads it off the offset. A split that leaves a station spots on both
sides of its datum costs that station a wrist re-orientation mid-row —
which `examples/line_balance_sweep.py` measures rather than assumes.

Run with:  python examples/weld_line_demo.py [out.usda] [--clash]
"""

import sys
from pathlib import Path

import botrail as bt
import weld_station_demo as ws

# Line geometry. The pitch clears a 4.11 m body with a metre of daylight;
# stations sit one pitch apart, the head one pitch upstream, the sink two
# pitches (plus its own reach) downstream.
PITCH = 5.2
HEAD = -PITCH
SIDES = ("lh", "rh")
BODIES = 3

# Spot-mark obstacle names, filled by `build_line`.
MARKS: list = []

# Which stretch of the seam each station owns — the line-balancing knob.
# The default cuts front/rear so each station stays on one side of its own
# datum, and therefore in one gun attitude for its whole row.
SEAM_SPLITS = {
    2: ((-1.2, -0.6, 0.0), (0.6, 1.2)),
    4: ((-1.2, -0.6), (0.0,), (0.6,), (1.2,)),
}


def attitude_of(x: float) -> str:
    """Which gun attitude a spot at body-x `x` is welded in.

    It is not a preference: a station's arm stands at that station's
    datum, so a spot ahead of the datum is reached leaning one way and a
    spot behind it the other (design-weld-line.md §4.3). The sign of the
    offset picks the attitude, which is why a station holding a
    contiguous stretch on one side of its datum needs only one.
    """
    return "dn" if x <= 0.0 else "up"


def _layout(count: int) -> tuple:
    split = SEAM_SPLITS[count]
    stations = tuple(f"st{k + 1}" for k in range(count))
    return (
        stations,
        {st: k * PITCH for k, st in enumerate(stations)},
        {st: split[k] for k, st in enumerate(stations)},
    )


STATIONS, ST_X, SPOTS = _layout(2)
TAIL = len(STATIONS) * PITCH
ARMS = [f"{st}_{side}" for st in STATIONS for side in SIDES]
PITCHES = BODIES + len(STATIONS)


def set_stations(count: int) -> None:
    """Reshapes the module for an N-station line (2 is the default and the
    golden). The pipeline machinery — lead pitches, transfer gates, sink
    placement, scenery spans — all derives from these tables, so nothing
    else has to change."""
    global STATIONS, ST_X, SPOTS, TAIL, ARMS, PITCHES
    if count not in SEAM_SPLITS:
        raise SystemExit(f"--stations takes one of {sorted(SEAM_SPLITS)}")
    STATIONS, ST_X, SPOTS = _layout(count)
    TAIL = len(STATIONS) * PITCH
    ARMS = [f"{st}_{side}" for st in STATIONS for side in SIDES]
    PITCHES = BODIES + len(STATIONS)


def side_of(arm: str) -> float:
    return ws.SIDE[arm.split("_")[1]]


def station_of(arm: str) -> str:
    return arm.split("_")[0]


class Line:
    """Rider offsets (relative to a body's origin) and station datums."""

    def __init__(self, station: ws.Station):
        self.station = station
        # (suffix, offset from the body origin) for everything that rides.
        self.offsets = [("skid", (0.0, 0.0, ws.SKID_TOP - 0.03))]
        for st in STATIONS:
            for j, x in enumerate(SPOTS[st]):
                sign = ws.ROLE_SIGN[attitude_of(x)]
                for side in SIDES:
                    self.offsets.append((
                        f"tab_{st}_{side}{j + 1}",
                        (x + sign * ws.TAB_STANDOFF,
                         ws.SIDE[side] * (station.flank + ws.TAB_OUT / 2),
                         station.seam_z),
                    ))

    def spot(self, arm: str, index: int) -> tuple:
        st = station_of(arm)
        return (ST_X[st] + SPOTS[st][index],
                side_of(arm) * self.station.seam_y,
                self.station.seam_z)

    def withdrawn(self, arm: str, index: int) -> tuple:
        x, y, z = self.spot(arm, index)
        return (x, y, z - ws.CLEAR)


def build_line() -> tuple:
    arm = bt.Robot.from_catalog(ws.ARM)
    gun = bt.Robot.from_catalog(ws.GUN_ID)
    robot = arm.attach_tool(gun)

    scene = bt.Scene(robot)
    scene.rename_robot(scene.robots[0], ARMS[0])
    for name in ARMS:
        x = ST_X[station_of(name)]
        y = side_of(name) * ws.BASE_Y
        facing = (0.0, 0.0, -ws.S2, ws.S2) if y > 0 else (0.0, 0.0, ws.S2, ws.S2)
        if name == ARMS[0]:
            scene.set_robot_base_pose((x, y, 0.0), facing, robot=name)
        else:
            scene.add_robot(robot, name=name, base_position=(x, y, 0.0),
                            base_quaternion=facing)
        scene.set_joint_positions(ws.READY, robot=name)

    # Line furniture, spanning head to sink. Same materials, same rules as
    # the station demo: collision on for whatever an arm could reach.
    bed_from, bed_to = HEAD - 2.6, TAIL + 2.8
    guard_y = ws.BASE_Y + 0.95
    scenery = [
        ("Bed", (bed_to - bed_from, 0.86, ws.BED_H),
         ((bed_from + bed_to) / 2, 0.0, ws.BED_TOP - ws.BED_H / 2),
         (0.16, 0.17, 0.19), True, ws.MAT_MACHINE),
    ]
    legs = 6
    for i in range(legs):
        x = bed_from + 1.0 + (bed_to - bed_from - 2.0) * i / (legs - 1)
        scenery.append((f"BedLeg_{i}", (0.16, 0.70, 0.72), (x, 0.0, 0.36),
                        (0.20, 0.21, 0.23), True, ws.MAT_MACHINE))
    for name in ARMS:
        x, y = ST_X[station_of(name)], side_of(name) * ws.BASE_Y
        scenery.append((f"Plate_{name}", (1.10, 1.10, 0.06), (x, y, -0.05),
                        (0.22, 0.23, 0.25), True, ws.MAT_MACHINE))
    for st in STATIONS:
        for side in SIDES:
            s = ws.SIDE[side]
            scenery.append((
                f"WeldCtrl_{st}_{side}", (0.70, 0.60, 1.60),
                (ST_X[st] - 1.6, s * (ws.BASE_Y + 0.55), 0.80),
                (0.24, 0.26, 0.30), True, ws.MAT_PAINT,
            ))
    post = (0.08, 0.08, ws.GUARD_H)
    guard_from, guard_to = bed_from - 1.0, bed_to + 1.0
    span = guard_to - guard_from
    posts = 8
    for side, y in (("lh", guard_y), ("rh", -guard_y)):
        for i in range(posts):
            x = guard_from + span * i / (posts - 1)
            scenery.append((f"Post_{side}_{i}", post, (x, y, ws.GUARD_H / 2),
                            ws.FENCE, True, ws.MAT_PAINT))
        for tag, z in (("top", ws.GUARD_H - 0.06), ("knee", 0.55)):
            scenery.append((f"Rail_{side}_{tag}", (span, 0.05, 0.08),
                            ((guard_from + guard_to) / 2, y, z),
                            ws.FENCE, True, ws.MAT_PAINT))
    rail_half = (guard_y - ws.OPENING) / 2
    for end, x in (("in", guard_from), ("out", guard_to)):
        for side, sign in (("lh", 1.0), ("rh", -1.0)):
            scenery.append((f"Post_{end}_{side}", post,
                            (x, sign * ws.OPENING, ws.GUARD_H / 2),
                            ws.FENCE, True, ws.MAT_PAINT))
            for tag, z in (("top", ws.GUARD_H - 0.06), ("knee", 0.55)):
                scenery.append((
                    f"Rail_{end}_{side}_{tag}", (0.05, 2 * rail_half, 0.08),
                    (x, sign * (ws.OPENING + rail_half), z),
                    ws.FENCE, True, ws.MAT_PAINT,
                ))
    scenery.append(("Mark_lh", (span, 0.10, 0.004), (2.6, 1.45, 0.002),
                    ws.FLOOR_MARK, False, ws.MAT_MATTE))
    scenery.append(("Mark_rh", (span, 0.10, 0.004), (2.6, -1.45, 0.002),
                    ws.FLOOR_MARK, False, ws.MAT_MATTE))
    for name, size, position, color, collides, material in scenery:
        prim = f"/World/Cell/{name}"
        scene.add_box(prim, size, position, color=color)
        scene.set_obstacle_material(prim, *material)
        if not collides:
            scene.set_obstacle_enabled(prim, False)

    # One body, measured; then the fleet. Three bodies are in flight at
    # steady state (head, station 1, station 2), so the magazine holds
    # three — every rider slot is one source whose pool is that slot in
    # each body, emitted at the head in load order.
    shell, meshes = ws.body_meshes()
    riders = {b: [] for b in range(1, BODIES + 1)}
    for b in range(1, BODIES + 1):
        for name, path in meshes:
            prim = f"/World/Line/b{b}/{name.rsplit('/', 1)[-1]}"
            scene.add_mesh(prim, str(path), ws.PARK, color=ws.STEEL)
            scene.set_obstacle_material(prim, *ws.MAT_STEEL)
            scene.set_obstacle_visible(prim, False)
            riders[b].append((prim, (0.0, 0.0, 0.0)))
    lo, hi = [1e9] * 3, [-1e9] * 3
    for prim, _ in riders[1]:
        scene.set_obstacle_pose(prim, (0.0, 0.0, 0.0))
        a, c = scene.obstacle_bounds(prim)
        lo = [min(v, w) for v, w in zip(lo, a)]
        hi = [max(v, w) for v, w in zip(hi, c)]
        scene.set_obstacle_pose(prim, ws.PARK)
    station = ws.Station(lo, hi)
    line = Line(station)

    # Re-anchor the mesh riders at the body origin (their vertices carry
    # the shape), then the display shell and the authored pieces.
    for b in range(1, BODIES + 1):
        riders[b] = [(prim, (0.0, 0.0, station.lift)) for prim, _ in riders[b]]
        if shell is not None:
            prim = f"/World/Line/b{b}/shell"
            scene.add_mesh(prim, str(shell), ws.PARK, color=ws.STEEL)
            scene.set_obstacle_material(prim, *ws.MAT_STEEL)
            scene.set_obstacle_enabled(prim, False)
            riders[b].append((prim, (0.0, 0.0, station.lift)))
        for suffix, offset in line.offsets:
            prim = f"/World/Line/b{b}/{suffix}"
            if suffix == "skid":
                scene.add_box(prim, (4.30, 0.70, 0.06), ws.PARK, color=ws.SKID_COLOR)
                scene.set_obstacle_material(prim, *ws.MAT_SKID)
            else:
                scene.add_box(prim, (2 * ws.SHEET, ws.TAB_OUT, ws.TAB_H),
                              ws.PARK, color=ws.STEEL)
                scene.set_obstacle_material(prim, *ws.MAT_STEEL)
            riders[b].append((prim, offset))

    # Sources (one per rider slot, pool = that slot in each body) and the
    # sink past the last station. The sink's reach has to cover every
    # rider's *origin* at the exit datum, and nothing at station 2.
    for slot, (suffix, offset) in enumerate(
        [(p.rsplit("/", 1)[-1], o) for p, o in riders[1]]
    ):
        pool = [riders[b][slot][0] for b in range(1, BODIES + 1)]
        scene.add_source(
            f"src_{suffix}",
            pool=pool,
            park=ws.PARK,
            pitch=(0.0, 0.0, 0.0),
            position=(HEAD + offset[0], offset[1], offset[2]),
            interval=0.0,
            running=False,
        )
        scene.add_sink(
            f"snk_{suffix}",
            zone_position=(TAIL, 0.0, 1.2),
            zone_size=(5.0, 2.4, 2.0),
            source=f"src_{suffix}",
        )

    # The transfer zone, sized off the riders, and the same authoring-time
    # guard as the station demo: anything static whose origin falls inside
    # would silently ride the belt — reject it now, not in a replay.
    ride_lo = [min(o[i] for _, o in riders[1]) for i in range(3)]
    ride_hi = [max(o[i] for _, o in riders[1]) for i in range(3)]
    x_from = HEAD + ride_lo[0] - 0.3
    x_to = TAIL + ride_hi[0] + 2.6
    y_half = max(abs(ride_lo[1]), ride_hi[1]) + 0.15
    z_from = (ws.BED_TOP - ws.BED_H / 2 + min(ride_lo[2], station.lift)) / 2
    z_to = max(ride_hi[2], station.seam_z) + 0.10
    riding = {prim for b in riders for prim, _ in riders[b]}
    inside = [
        name for name in scene.obstacle_names
        if name not in riding
        and all(a <= v <= c for v, a, c in zip(scene.obstacle_pose(name)[0],
                                               (x_from, -y_half, z_from),
                                               (x_to, y_half, z_to)))
    ]
    if inside:
        raise SystemExit(
            "these would ride the transfer with the bodies: " + ", ".join(inside)
        )
    scene.add_conveyor(
        "line",
        zone_position=((x_from + x_to) / 2, 0.0, (z_from + z_to) / 2),
        zone_size=(x_to - x_from, 2 * y_half, z_to - z_from),
        velocity=(ws.BELT_V, 0.0, 0.0),
        running=False,
    )

    # Part-present at the line head: the photo-eye the transfer gates on.
    # It also closes the scan-order race a same-tick load+advance has (the
    # source emits after the belt has advected, costing the body its first
    # scan of travel — 4 mm, forever, at every station downstream).
    scene.add_beam_sensor(
        "body_at_head",
        frm=(HEAD, -1.2, ws.SKID_TOP + 0.35),
        to=(HEAD, 1.2, ws.SKID_TOP + 0.35),
        radius=0.03,
    )

    scene.define_signal("moving", False)
    for st in STATIONS:
        scene.define_signal(f"{st}_done", False)

    # Process presentation, PLC style: each station's weld-current signal
    # is raised by its weld steps (a weld controller's "current on"), each
    # arm's flash binds to its own station's signal, and the spot marks —
    # the nuggets — are fed onto the tabs by the release steps through the
    # ordinary source machinery, so they blink into usdview exactly as
    # they do in the studio and recirculate with everything else.
    MARKS.clear()
    for st in STATIONS:
        scene.define_signal(f"{st}_arc", False)
    for name in ARMS:
        scene.add_weld_flash(f"flash_{name}",
                             signal=f"{station_of(name)}_arc", robot=name)
    for arm in ARMS:
        st, s_ = station_of(arm), side_of(arm)
        for index, x in enumerate(SPOTS[st]):
            sign = ws.ROLE_SIGN[attitude_of(x)]
            pose = (ST_X[st] + x + sign * ws.TAB_STANDOFF,
                    s_ * (station.flank + ws.TAB_OUT / 2), station.seam_z)
            # One mark per spot is the whole magazine: it rides out with
            # its body and the tail sink hands it back for the next one
            # (the same recirculation as every other rider).
            prim = f"/World/Line/mark_{arm}_s{index + 1}"
            scene.add_box(prim, (2 * ws.SHEET + 0.004, 0.055, 0.055),
                          ws.PARK, color=(0.09, 0.07, 0.06))
            scene.set_obstacle_material(prim, 0.55, 0.65)
            scene.set_obstacle_enabled(prim, False)
            pool = [prim]
            MARKS.append(prim)
            scene.add_source(
                f"src_mark_{arm}_s{index + 1}",
                pool=pool,
                park=ws.PARK,
                pitch=(0.0, 0.0, 0.0),
                position=pose,
                interval=0.0,
                running=False,
            )
            scene.add_sink(
                f"snk_mark_{arm}_s{index + 1}",
                zone_position=(TAIL, 0.0, 1.2),
                zone_size=(5.0, 2.4, 2.0),
                source=f"src_mark_{arm}_s{index + 1}",
            )
    return scene, line, riders


# The arms live at seam height, half a metre outboard of it, and enter by
# *sliding in* — the tab comes through the open side of the throat and
# lands in the electrode gap, W0's stage-in move on a new seam. Everything
# else was measured and rejected: a direct READY-to-seam ramp cuts the
# corner over the roof and drags the gun through the ditch flange; a
# vertical drop from a hover is blocked at every x by the gun's own bulk
# shaving the pillar band; a diagonal hover slides the tab sideways into
# the C-frame. The one big posture change left — READY down to the park —
# swings the electrode arm through the roof band, so it happens exactly
# once, at program start, when the line is still provably empty.
PARK_OUT = 0.50


def teach(scene: bt.Scene, line: Line, riders: dict) -> dict:
    """Each station's poses, taught with body 1 standing at that station —
    where a body is whenever that station's guns are near one.

    Everything the cycle runs is a taught ramp; nothing plans. Each arm
    lives beside its stretch of seam and slides in and out of it, so the
    whole cycle is teach-and-verify: the frame sweep is the proof, and
    the bake costs no planning time at all."""
    poses = {}
    for st in STATIONS:
        for prim, offset in riders[1]:
            scene.set_obstacle_pose(
                prim, (ST_X[st] + offset[0], offset[1], offset[2]))
        try:
            for side in SIDES:
                arm = f"{st}_{side}"
                spots = []
                for index in range(len(SPOTS[st])):
                    quat = ws.Q_ROLE[attitude_of(SPOTS[st][index])]
                    for name in ARMS:
                        scene.set_joint_positions(ws.READY, robot=name)
                    if spots:
                        scene.set_joint_positions(spots[-1][1], robot=arm)
                    out = ws.solve(scene, arm, line.withdrawn(arm, index), quat,
                                   f"withdrawn from spot {index + 1}")
                    scene.set_joint_positions(out, robot=arm)
                    at = ws.solve(scene, arm, line.spot(arm, index), quat,
                                  f"spot {index + 1}")
                    spots.append((out, at))
                    scene.set_joint_positions(ws.READY, robot=arm)

                def park_beside(index: int, seed: list) -> list:
                    x, y, z = line.withdrawn(arm, index)
                    for name in ARMS:
                        scene.set_joint_positions(ws.READY, robot=name)
                    scene.set_joint_positions(seed, robot=arm)
                    target = (x, y + side_of(arm) * PARK_OUT, z)
                    return ws.solve(scene, arm,
                                    target, ws.Q_ROLE[attitude_of(SPOTS[st][index])],
                                    f"park beside spot {index + 1}")

                park_in = park_beside(0, spots[0][0])
                park_out = park_beside(len(spots) - 1, spots[-1][0])
                # Retake full turns the short way, in the order the cycle
                # actually runs the chain.
                reference = ws.READY
                chain = [park_in]
                for out, at in spots:
                    chain += [out, at]
                chain += [park_out]
                limits = scene.robot.joint_limits
                for q in chain:
                    q[:] = ws.unwind(q, reference, limits)
                    reference = q
                poses[arm] = {"spots": spots, "park_in": park_in,
                              "park_out": park_out}
        finally:
            for prim, _ in riders[1]:
                scene.set_obstacle_pose(prim, ws.PARK)
    for name in ARMS:
        scene.set_joint_positions(ws.READY, robot=name)
    return poses


def build_station_program(scene: bt.Scene, st: str, poses: dict,
                          bodies: int = BODIES) -> str:
    """One station's SFC: for each body that reaches it — wait out the
    pitches, weld the row (both sides in lockstep), report done.

    Every move is a taught ramp (READY → hover → seam and back): the arms
    never plan, so the bake spends its time verifying, not searching."""
    names = scene.robot.joint_names
    arms = [f"{st}_{side}" for side in SIDES]

    def arm_to(q: list) -> dict:
        return dict(zip(names, q))

    sq = scene.sequence(st)
    # The one big posture change — READY down to the seam-side park —
    # swings the electrode arm through the roof band, so it runs before
    # the first pitch, against a line that is still provably empty.
    sq.step("deploy", actions=[
        bt.seq.ramp(arm_to(poses[arm]["park_in"]), 2.0, robot=arm)
        for arm in arms
    ])
    lead_pitches = list(STATIONS).index(st) + 1
    for body in range(1, bodies + 1):
        tag = f"b{body}"
        # Wait out the transfer(s) that carry this body to the station.
        # The first body needs `lead_pitches` of them; after that, exactly
        # one per body. `moving` is the transfer program's output: a rising
        # edge says the line is indexing, a falling edge says it has landed
        # (to the millimetre, courtesy of `advance`).
        pitches = lead_pitches if body == 1 else 1
        for k in range(pitches):
            sq.step(f"{tag}_p{k + 1}_start",
                    transition=bt.seq.signal("moving", True))
            sq.step(f"{tag}_p{k + 1}_landed",
                    transition=bt.seq.signal("moving", False))
        sq.step(f"{tag}_slide_in", actions=[
            bt.seq.ramp(arm_to(poses[arm]["spots"][0][0]), 1.0, robot=arm)
            for arm in arms
        ])
        for index in range(len(SPOTS[st])):
            spot = f"{tag}_s{index + 1}"
            for phase, which, duration in (("travel", 0, ws.TRAVEL_T),
                                           ("engage", 1, ws.LIFT_T)):
                sq.step(f"{spot}_{phase}", actions=[
                    bt.seq.ramp(arm_to(poses[arm]["spots"][index][which]),
                                duration, robot=arm)
                    for arm in arms
                ])
            sq.step(f"{spot}_squeeze", actions=[
                bt.seq.ramp({ws.GUN: ws.GUN_SQUEEZE}, ws.SQUEEZE_T, robot=arm)
                for arm in arms
            ])
            sq.step(f"{spot}_weld",
                    actions=[bt.seq.set_signal(f"{st}_arc", True)],
                    transition=bt.seq.elapsed(ws.WELD_T))
            sq.step(f"{spot}_release", actions=[
                bt.seq.ramp({ws.GUN: ws.GUN_OPEN}, ws.SQUEEZE_T, robot=arm)
                for arm in arms
            ] + [bt.seq.set_signal(f"{st}_arc", False)]
              + [bt.seq.start(f"src_mark_{arm}_s{index + 1}")
                 for arm in arms])
            sq.step(f"{spot}_withdraw", actions=[
                bt.seq.ramp(arm_to(poses[arm]["spots"][index][0]), ws.LIFT_T,
                            robot=arm)
                for arm in arms
            ])
        sq.step(f"{tag}_slide_out", actions=[
            bt.seq.ramp(arm_to(poses[arm]["park_out"]), 1.0, robot=arm)
            for arm in arms
        ])
        # Back along the outboard lane to sit beside the first spot,
        # ready for the next body (and clear of everything the transfer
        # sweeps past this station).
        sq.step(f"{tag}_traverse", actions=[
            bt.seq.ramp(arm_to(poses[arm]["park_in"]), 1.2, robot=arm)
            for arm in arms
        ])
        sq.step(f"{tag}_report",
                actions=[bt.seq.set_signal(f"{st}_done", True)])
        if body < bodies:
            # The next transfer consumes the report; drop it once the line
            # is moving again so the next cycle's report is a fresh edge.
            sq.step(f"{tag}_handoff", transition=bt.seq.signal("moving", True))
            sq.step(f"{tag}_reset",
                    actions=[bt.seq.set_signal(f"{st}_done", False)])
    return sq.name


def build_transfer_program(scene: bt.Scene, riders: dict,
                           gated: bool = True) -> str:
    """The transfer POU: load a body while the stations work, wait for
    every station that holds one, index the whole line a pitch."""
    sq = scene.sequence("transfer")
    for pitch in range(1, PITCHES + 1):
        tag = f"p{pitch}"
        if pitch <= BODIES:
            sq.step(f"{tag}_load", actions=[
                bt.seq.start(f"src_{prim.rsplit('/', 1)[-1]}")
                for prim, _ in riders[1]
            ], transition=bt.seq.signal("body_at_head", True))
        # Which stations hold a body during the window before this pitch:
        # station k (1-based) welds body b in window w = b + k - 1.
        holding = [
            st for k, st in enumerate(STATIONS, start=1)
            if 1 <= pitch - k <= BODIES
        ]
        if holding and gated:
            sq.step(f"{tag}_gate", transition=bt.seq.all_of(*[
                bt.seq.signal(f"{st}_done", True) for st in holding
            ]))
        sq.step(f"{tag}_index",
                actions=[bt.seq.set_signal("moving", True),
                         bt.seq.advance("line", PITCH)],
                transition=bt.seq.device_done("line"))
        sq.step(f"{tag}_landed", actions=[bt.seq.set_signal("moving", False)])
    return sq.name


def busy_windows(timeline, arms) -> list:
    """Merged [start, end] intervals where any of `arms` is moving."""
    spans = sorted(
        (start, end)
        for arm in arms
        for _, start, end in timeline.moves(arm)
    )
    merged = []
    for start, end in spans:
        if merged and start <= merged[-1][1] + 1e-9:
            merged[-1][1] = max(merged[-1][1], end)
        else:
            merged.append([start, end])
    return merged


def overlap(a: list, b: list) -> float:
    return sum(
        max(0.0, min(a1, b1) - max(a0, b0))
        for a0, a1 in a
        for b0, b1 in b
    )


def sweep_for_contact(scene: bt.Scene, riders: dict, timeline,
                      stop_early: bool = False) -> dict:
    """Frame-by-frame gun-vs-body check over the baked line (the rollout
    reports only robot-robot pairs, so this is where a belt yanking a body
    out of a closed gun shows up)."""
    pieces = [
        prim for b in riders for prim, _ in riders[b]
        if not prim.endswith("/shell")
    ]
    shells = [prim for b in riders for prim, _ in riders[b]
              if prim.endswith("/shell")]
    for prim in shells:
        scene.set_obstacle_enabled(prim, False)
    trajectories = {r: timeline.robot_trajectory(r) for r in timeline.robots}
    # Re-posing every rider every frame dominates the sweep, and bodies
    # only move while the belt runs — so track one witness per body and
    # re-pose a body's set only when its witness has actually moved.
    witness = {
        b: next(prim for prim, _ in riders[b] if not prim.endswith("/shell"))
        for b in riders
    }
    placed = {b: None for b in riders}
    offences = {}
    t, step = 0.0, 0.05
    while t <= timeline.duration:
        for robot, trajectory in trajectories.items():
            scene.set_joint_positions(list(trajectory.sample(t)), robot=robot)
        for b in riders:
            state = (timeline.object_pose(witness[b], t),
                     timeline.object_visible(witness[b], t))
            if state == placed[b]:
                continue
            placed[b] = state
            for prim, _ in riders[b]:
                if prim.endswith("/shell"):
                    continue
                position, quaternion = timeline.object_pose(prim, t)
                scene.set_obstacle_pose(prim, position, quaternion)
                scene.set_obstacle_enabled(
                    prim, timeline.object_visible(prim, t))
        for a, b in scene.check_collisions():
            if {a[0], b[0]} == {"link", "obstacle"}:
                offences.setdefault((a[1], b[1]), []).append(round(t, 2))
                if stop_early:
                    return offences
        t += step
    return offences


def main() -> None:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    clash = "--clash" in sys.argv
    for k, flag in enumerate(sys.argv[1:], start=1):
        if flag.startswith("--stations"):
            value = flag.split("=", 1)[1] if "=" in flag else sys.argv[k + 1]
            set_stations(int(value))
    out = Path(args[0]) if args else Path("cell_line.usda")

    scene, line, riders = build_line()
    poses = teach(scene, line, riders)
    # The ungated variant deadlock-proofs itself by welding one body per
    # station: the free-running transfer finishes long before the stations
    # would wait on a `moving` edge it will never raise again — and one
    # body is plenty to show the welds landing on air.
    programs = [build_station_program(scene, st, poses,
                                      bodies=1 if clash else BODIES)
                for st in STATIONS]
    programs.append(build_transfer_program(scene, riders, gated=not clash))

    timeline = scene.simulate_sequences(programs, max_duration=400.0)

    if clash:
        # Ungated, the transfer indexes the moment it has loaded — under
        # the welding. Nothing collides (the squeeze is taught 5 mm off
        # the sheet, so the tab slides out of the gun untouched) and the
        # rollout bakes it without complaint: the failure is that the
        # welds land on air. The datum tells the story.
        at = {s_: (a, b) for s_, a, b in timeline.step_spans}
        start, end = at["st1/b1_s1_weld"]
        mid = (start + end) / 2
        pose = timeline.object_pose(f"/World/Line/b1/{riders[1][0][0].rsplit('/', 1)[-1]}", mid)[0]
        print("unarbitrated transfer: the weld fired with body 1 at "
              f"x = {pose[0]:+.3f} m — {abs(pose[0] - ST_X['st1']):.3f} m past "
              "the station datum, electrodes on air")
        return

    spots = sum(len(SPOTS[st]) for st in STATIONS) * 2
    print(f"line: {len(STATIONS)} stations, {BODIES} bodies, pitch {PITCH} m, "
          f"{spots} spots per body")
    print(f"total {timeline.duration:.2f}s for {BODIES} bodies through "
          f"{PITCHES} pitches")
    windows = {st: busy_windows(timeline, [f"{st}_{s}" for s in SIDES])
               for st in STATIONS}
    both = overlap(windows["st1"], windows["st2"])
    print(f"stations welding concurrently: {both:.2f}s "
          "(what a single serial sequence cannot do)")
    for st in STATIONS:
        busy = sum(end - start for start, end in windows[st])
        print(f"  {st} busy {busy:6.2f}s")
    at = {s: (a, b) for s, a, b in timeline.step_spans}
    takt = (at[f"transfer/p{PITCHES - 1}_landed"][1]
            - at[f"transfer/p{PITCHES - 2}_landed"][1])
    print(f"steady-state takt (pitch {PITCHES - 2} -> {PITCHES - 1}): {takt:.2f}s")

    offences = sweep_for_contact(scene, riders, timeline)
    if offences:
        details = "; ".join(
            f"{a} x {b} at {t[0]}s ({len(t)} frames)"
            for (a, b), t in offences.items()
        )
        raise SystemExit(f"gun through the body: {details}")
    print("frame sweep: no gun ever touches a body")

    warnings = timeline.export_usd(out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported to {out} — view with: usdview {out}")


if __name__ == "__main__":
    main()
