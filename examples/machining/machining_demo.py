"""Robot machining on catalog hardware: MELFA ASSISTA + motor spindle.

A trimming cell with a 5-axis rim chamfer, built from products rather
than stand-ins — a Mitsubishi Electric RV-5AS-D (`manipulator`) carrying
a NAKANISHI-class motor spindle (`tool.spindle`), both pulled straight
from the catalog. The spindle ships the frame convention the 5-DOF solver
wants (`tip` +Z runs tip -> body) and its cutter as its own link, so the
contact exemption binds to a real part of the tool.

What the cell shows, following design/design-machining.md:

* **A 5-axis path from APT.** The rim chamfer arrives as CL text
  (`GOTO/x,y,z,i,j,k` — the machine-independent 5-axis format), its tool
  axis leaning 30 deg outward all the way around the plate, hopping the
  toe clamps. The lean direction precesses a full turn per lap.

* **Why the cycle bakes with the global spin pass.** The cutter is
  rotationally symmetric, so the spin of a taught stance is arbitrary —
  nobody jogs to a *particular* one. Teach this cell's stance half a turn
  round and per-sample greedy resolution breaks at the very first sample;
  `spin="optimize"` (a Descartes-style Viterbi pass over spin candidates)
  is untouched. On this arm's roomy wrist the two agree wherever greedy
  survives, so the optimizer costs a fraction of a second and buys the
  cell its independence from how it was taught.

* **Reach, diagnosed rather than guessed.** Pull the fixture 180 mm in
  toward the base and the chamfer stops solving — `check_toolpath` names
  the first failing sample and why (the RV-5AS-D's J5 is +-120 deg, so a
  leaning tool close in runs out of wrist). Push it back out and the same
  program solves.

* **The feed report.** Floors make `length / feed` a hard lower bound,
  so joints can only slow a cut: `feed_report` says how much was lost
  (hold ratio), where (time spans), and which axis owned each slowdown.

The rest is the cell: clamp -> spin-up -> cuts -> spin-down -> unclamp,
progressive material removal, USD with BasisCurves overlays, URScript
with movep chains and digital I/O. Move the fixture and re-simulate:
nothing was taught in joints.

Run with:  python examples/machining/machining_demo.py [--studio]
"""

import math
from pathlib import Path
import sys

import botrail as bt

# The two catalog products this cell is built from.
CATALOG_ARM = "mitsubishi_electric/assista/rv-5as-d/r1"
CATALOG_SPINDLE = "botrail/spindle/spindle-emsf3060/r1"

# Fixture geometry, all in the part frame (meters). The plate top is z=0.
# Sized to the RV-5AS-D (910 mm reach): the spindle-vertical band covers
# the whole bench, so the job is a 180 x 120 panel — big enough to fill
# the arm's stroke, small enough that the cycle stays inside the rollout's
# default 120 s budget.
PLATE = (0.18, 0.12, 0.012)
CONTOUR_HALF = (0.08, 0.05)  # trim line, 10 mm inside the plate edge
CORNER_R = 0.015
CUT_DEPTH = -0.002
RAPID_Z = 0.008
CUT_FEED = 0.015     # 15 mm/s trim
PLUNGE_FEED = 0.004  # 4 mm/s entry
RAPID_CAP = 0.25     # m/s, movel-style rapid cap

# The taught stance: spindle vertical, tip 480 mm out and 288 mm above the
# base, elbow up. J6 (the spin about the tool axis) is taught at 0 — the
# tool is symmetric, so the value is arbitrary, but teaching it at a limit
# (+-200 deg here) leaves greedy resolution a half turn to unwind at the
# first sample. `main` shows exactly that.
REF_Q = [0.0, 0.010563, 1.873486, 0.0, 1.257542, 0.0]

FIXTURE_BODIES = ("plate", "bench", "clamp_front", "clamp_back")


def build_scene() -> tuple[bt.Scene, str]:
    arm = bt.Robot.from_catalog(CATALOG_ARM)
    spindle = bt.Robot.from_catalog(CATALOG_SPINDLE)
    # The spindle's root *is* its mounting face and its tcp is the cutter
    # tip, so the flange name is the only thing to say.
    robot = arm.attach_tool(spindle, flange=arm.flange_link)
    scene = bt.Scene(robot)
    scene.set_joint_positions(REF_Q)

    # Teach the fixture datum: the plate top sits RAPID_Z under the tip's
    # reference pose, so the taught configuration *is* the rapid plane.
    tip, _ = scene.link_pose(robot.tcp_link)
    origin = (tip[0], tip[1], tip[2] - RAPID_Z)
    scene.add_frame("fixture", position=origin)

    def at(dx: float, dy: float, dz: float) -> tuple[float, float, float]:
        return (origin[0] + dx, origin[1] + dy, origin[2] + dz)

    # Stock: a live collision body. Only the cutter may touch it — and
    # only while cutting (rapids suspend the exemption).
    scene.add_box("plate", size=PLATE, position=at(0, 0, -PLATE[2] / 2))
    scene.set_obstacle_color("plate", (0.75, 0.77, 0.80))
    scene.allow_link_obstacle_contact("cutter", "plate")
    # The bench the plate is clamped to, standing on the floor, and the
    # toe clamps: ordinary collision. The clamps grip the long edges 3 mm
    # onto the plate, leaving 7 mm to the trim line.
    bench_h = origin[2] - PLATE[2]
    scene.add_box(
        "bench",
        size=(PLATE[0] + 0.16, PLATE[1] + 0.16, bench_h),
        position=(origin[0], origin[1], bench_h / 2),
    )
    scene.set_obstacle_color("bench", (0.35, 0.33, 0.30))
    for side, sy in (("front", -1.0), ("back", 1.0)):
        scene.add_box(
            f"clamp_{side}",
            size=(0.03, 0.03, 0.018),
            position=at(0.0, sy * (PLATE[1] / 2 + 0.012), 0.009),
        )
        scene.set_obstacle_color(f"clamp_{side}", (0.85, 0.55, 0.10))
    # The cell's I/O: clamp valve and spindle run, written by the sequence.
    scene.define_signal("clamped")
    scene.define_signal("spindle_run")
    return scene, robot.tcp_link


def contour_toolpath() -> bt.toolpath.Toolpath:
    """Rounded-rectangle trim, one climb pass: plunge at the mid -x edge,
    run CCW, retract. Corners are quarter arcs from the builder."""
    hx, hy = CONTOUR_HALF
    r = CORNER_R
    tp = bt.toolpath.builder(frame="fixture")
    tp.rapid_to((-hx, 0.0, RAPID_Z))
    tp.feed(PLUNGE_FEED).line_to((-hx, 0.0, CUT_DEPTH))
    tp.feed(CUT_FEED)
    tp.line_to((-hx, hy - r, CUT_DEPTH))
    tp.arc_to((-hx + r, hy, CUT_DEPTH), center=(-hx + r, hy - r, CUT_DEPTH), cw=True)
    tp.line_to((hx - r, hy, CUT_DEPTH))
    tp.arc_to((hx, hy - r, CUT_DEPTH), center=(hx - r, hy - r, CUT_DEPTH), cw=True)
    tp.line_to((hx, -hy + r, CUT_DEPTH))
    tp.arc_to((hx - r, -hy, CUT_DEPTH), center=(hx - r, -hy + r, CUT_DEPTH), cw=True)
    tp.line_to((-hx + r, -hy, CUT_DEPTH))
    tp.arc_to((-hx, -hy + r, CUT_DEPTH), center=(-hx + r, -hy + r, CUT_DEPTH), cw=True)
    tp.line_to((-hx, 0.0, CUT_DEPTH))
    tp.rapid_to((-hx, 0.0, RAPID_Z))
    return tp.build()


RIM_TILT_DEG = 30.0
RIM_HOP_Z = 0.030  # over the toe clamps (top at +18 mm)


def rim_apt(tilt_deg: float = RIM_TILT_DEG, feed_mmpm: float = 900.0,
            step: float = 0.008) -> str:
    """The rim chamfer as APT CL text — the 5-axis entry format: each
    ``GOTO`` carries the tool axis as ``i,j,k``, tilted outward by
    ``tilt_deg`` so the lean precesses a full turn around the plate. The
    clamp stretches are hopped with RAPID moves above the clamp tops."""
    hx, hy = CONTOUR_HALF
    t = math.radians(tilt_deg)
    lines = [
        "PARTNO / RIM CHAMFER 30DEG",
        "UNITS / MM",
        "MULTAX / ON",
        "LOADTL / 1",
        "SPINDL / RPM, 18000, CLW",
        f"FEDRAT / MMPM, {feed_mmpm:.0f}",
    ]

    def goto(x: float, y: float, z: float, nx: float, ny: float,
             rapid: bool = False) -> None:
        i, j, k = nx * math.sin(t), ny * math.sin(t), math.cos(t)
        if rapid:
            lines.append("RAPID")
        lines.append(
            f"GOTO / {x*1e3:.3f}, {y*1e3:.3f}, {z*1e3:.3f}, "
            f"{i:.6f}, {j:.6f}, {k:.6f}"
        )

    def feed_line(p0, p1, n) -> None:
        k = max(1, round(math.dist(p0, p1) / step))
        for s in range(1, k + 1):
            goto(p0[0] + (p1[0] - p0[0]) * s / k,
                 p0[1] + (p1[1] - p0[1]) * s / k, 0.0, *n)

    goto(hx, -hy, RAPID_Z, 1, 0, rapid=True)
    goto(hx, -hy, 0.0, 1, 0)
    feed_line((hx, -hy), (hx, hy), (1, 0))
    feed_line((hx, hy), (0.035, hy), (0, 1))
    goto(0.035, hy, RIM_HOP_Z, 0, 1, rapid=True)
    goto(-0.035, hy, RIM_HOP_Z, 0, 1, rapid=True)
    goto(-0.035, hy, 0.0, 0, 1)
    feed_line((-0.035, hy), (-hx, hy), (0, 1))
    feed_line((-hx, hy), (-hx, -hy), (-1, 0))
    feed_line((-hx, -hy), (-0.035, -hy), (0, -1))
    goto(-0.035, -hy, RIM_HOP_Z, 0, -1, rapid=True)
    goto(0.035, -hy, RIM_HOP_Z, 0, -1, rapid=True)
    goto(0.035, -hy, 0.0, 0, -1)
    feed_line((0.035, -hy), (hx, -hy), (0, -1))
    goto(hx, -hy, RAPID_Z, 0, -1, rapid=True)
    lines.append("FINI")
    return "\n".join(lines) + "\n"


def pocket_gcode() -> str:
    """A 40 x 24 mm zigzag pocket in the plate's center, 1.5 mm deep,
    6 mm stepover — the kind of file a CAM post writes. Millimeters,
    absolute, XY plane; the spindle words become parser warnings."""
    lines = [
        "( pocket 40x24, zigzag, 8mm end mill )",
        "G21 G90 G17 G94",
        "S18000 M3",
        "G0 X-20 Y-12 Z5",
        "G1 Z-1.5 F240",
    ]
    y = -12.0
    x = 20.0
    while True:
        lines.append(f"G1 X{x:.1f} F540")
        y += 6.0
        if y > 12.0:
            break
        lines.append(f"G1 Y{y:.1f}")
        x = -x
    lines += ["G0 Z8", "M5", "M30"]
    return "\n".join(lines) + "\n"


def build_cell() -> tuple[bt.Scene, str]:
    """The full cell: fixture scene, all three toolpaths, and the
    machining sequence — what `play_record.py` rebuilds to replay
    `cell_machining.usdc` onto. Bake it with ``toolpath_spin="optimize"``
    so the cell does not depend on the spin the stance was taught at."""
    scene, tcp_link = build_scene()
    scene.add_toolpath("contour", contour_toolpath())
    scene.add_toolpath("pocket", bt.toolpath.from_gcode(pocket_gcode(), frame="fixture"))
    scene.add_toolpath("rim", bt.toolpath.from_apt(rim_apt(), frame="fixture"))

    # The machining cycle, in PLC vocabulary: clamp (valve + actuation
    # time), spindle up (run contact + at-speed delay), the three cuts,
    # spindle down, unclamp.
    sq = scene.sequence("cycle")
    sq.step("clamp", actions=[bt.seq.set_signal("clamped")],
            transition=bt.seq.elapsed(0.5))
    sq.step("spin_up", actions=[bt.seq.set_signal("spindle_run")],
            transition=bt.seq.elapsed(1.5))
    sq.step("trim", actions=[bt.seq.toolpath("contour")],
            transition=bt.seq.done())
    sq.step("pocket", actions=[bt.seq.toolpath("pocket")],
            transition=bt.seq.done())
    sq.step("chamfer", actions=[bt.seq.toolpath("rim")],
            transition=bt.seq.done())
    sq.step("spin_down", actions=[bt.seq.set_signal("spindle_run", False)],
            transition=bt.seq.elapsed(1.0))
    sq.step("unclamp", actions=[bt.seq.set_signal("clamped", False)],
            transition=bt.seq.elapsed(0.5))
    # Presentation, zero effect on the cycle: while spindle_run is on the
    # studio draws the TCP's trail (the cut so far) and strobes the
    # cutter.
    scene.add_cut_trace("cutting", "spindle_run", scene.robots[0],
                        spin_link="cutter")
    return scene, tcp_link


def build_replay_cell() -> bt.Scene:
    """The cell plus the carve-stage obstacles a recording references.

    `main()` exports `cell_machining.usdc` with the progressive-removal
    prims (`plate_cut/NNN`) animated by visibility; replaying it needs the
    same obstacles in the scene or those prims stay unmatched. Re-baking
    and re-carving is deterministic, so the names line up exactly."""
    scene, _ = build_cell()
    tl = scene.simulate_sequence("cycle", toolpath_spin="optimize")
    scene.animate_carve(tl, "plate")
    return scene


def cut_seconds(tp: bt.toolpath.Toolpath) -> float:
    """Commanded cutting time: chord length / feed over the feed moves —
    what the cycle costs if the robot holds the feed everywhere."""
    total = 0.0
    prev = None
    for move in tp["moves"]:
        for target in move["targets"]:
            p = target["position"]
            if prev is not None and move["type"] == "feed":
                d = sum((a - b) ** 2 for a, b in zip(p, prev)) ** 0.5
                total += d / move["feed"]
            prev = p
    return total


def shift_fixture(scene: bt.Scene, dx: float) -> None:
    """Moves the whole fixture — datum frame, stock, table, clamps — by
    `dx` along x, the way a real fixture moves: as one thing."""
    home, _ = scene.frame("fixture")
    scene.add_frame("fixture", position=(home[0] + dx, home[1], home[2]))
    for name in FIXTURE_BODIES:
        (px, py, pz), quat = scene.obstacle_pose(name)
        scene.set_obstacle_pose(name, (px + dx, py, pz), quat)


def main() -> None:
    scene, tcp_link = build_cell()

    # Face diagnosis first: every sample attempted, failures located.
    # This arm's wrist has the room for all three paths as taught.
    for name in ("contour", "pocket", "rim"):
        report = scene.check_toolpath(name)
        print(f"check {name}: {report!r}")
        if not report.ok:
            sys.exit(f"unexpected: {report.issues[:3]}")

    # What the global spin pass is for. The cutter is symmetric, so the
    # spin of the taught stance is arbitrary — teach the same pose half a
    # turn round (J6 at its limit rather than at 0) and greedy has to
    # unwind it at the first sample, which is a configuration jump, which
    # is a fault. The optimizer picks the spin for the whole path and
    # never sees the seed's.
    scene.set_joint_positions(REF_Q[:5] + [REF_Q[5] + math.pi])
    twisted = scene.check_toolpath("contour")
    twisted_opt = scene.check_toolpath("contour", spin="optimize")
    print(f"stance taught a half turn round: greedy {len(twisted.issues)}/"
          f"{twisted.total_samples} fail ({twisted.issues[0]['detail']}) "
          f"| optimize {'ok' if twisted_opt.ok else 'FAIL'}")
    if twisted.ok or not twisted_opt.ok:
        sys.exit("unexpected: the taught spin no longer decides greedy")
    scene.set_joint_positions(REF_Q)

    # Reach, diagnosed: 180 mm closer to the base the leaning chamfer runs
    # out of wrist (J5 is +-120 deg on this arm), and check_toolpath says
    # so with a sample index instead of a shrug.
    shift_fixture(scene, -0.18)
    near = scene.check_toolpath("rim", spin="optimize")
    kinds = sorted({issue["kind"] for issue in near.issues})
    print(f"fixture 180mm closer: rim {len(near.issues)}/{near.total_samples} "
          f"fail {kinds} (first at sample {near.issues[0]['sample']})")
    shift_fixture(scene, 0.18)
    if near.ok:
        sys.exit("unexpected: the near fixture no longer stresses the wrist")

    # The cycle: clamp -> spin up -> trim -> pocket -> chamfer -> spin
    # down -> unclamp, baked with the spin pass for the reason above.
    tl = scene.simulate_sequence("cycle", toolpath_spin="optimize")
    print(f"cycle {tl.duration:.2f}s (spindle on "
          f"{tl.signal('spindle_run').high_total():.2f}s)")
    for name, t0, t1 in tl.step_spans:
        print(f"  {name:<10} {t0:7.2f} - {t1:7.2f}s")
    # The feed report: floors are lower bounds, so the joints can only
    # slow a cut — this says how much, where, and which axis owned it.
    for name in ("contour", "pocket", "rim"):
        report = tl.feed_report(name)
        print(f"feed {name}: {report!r}")
        for span in report.slow_spans[:2]:
            print(f"   slow {span['start']:6.2f}-{span['end']:6.2f}s: "
                  f"{span['achieved_feed']*1e3:.1f}mm/s of "
                  f"{span['commanded_feed']*1e3:.0f} ({span['limiting_joint']})")
    clearance = tl.min_clearance()
    print(f"min clearance over the cycle: {float(clearance) * 1000:.1f}mm"
          + (f" ({clearance.pair[0]} - {clearance.pair[1]})" if clearance.pair else ""))

    # The fixture moves 30 mm — frame, stock, and clamps as one thing —
    # and the same cell re-solves; nothing was taught in joints.
    shift_fixture(scene, 0.03)
    shifted = scene.simulate_sequence("cycle", toolpath_spin="optimize")
    print(f"fixture +30mm: cycle {shifted.duration:.2f}s "
          f"(was {tl.duration:.2f}s, rim hold "
          f"{shifted.feed_report('rim').hold_ratio*100:.1f}% vs "
          f"{tl.feed_report('rim').hold_ratio*100:.1f}%)")
    shift_fixture(scene, -0.03)

    # The machined part: a voxel subtraction of the cutter's sweep from
    # the stock — the picture and the numbers, never verification (in a
    # kinematic world the cut cannot contradict the plan).
    carve = tl.carve_stock("plate")
    print(f"carve: {carve!r}")
    # OBJ + MTL carries the surface classing as face colors: surviving
    # skin in the stock grey, cutter-made surfaces in a bright machined
    # finish — that contrast is what makes the removal readable.
    cut_obj = Path("machining_plate_cut.obj").resolve()
    carve.save_obj(cut_obj)
    # Progressive removal: re-cut the same sweep in stages and swap the
    # visible mesh along the timeline — the pristine plate holds until
    # the first real cut, then each snapshot shows for its window. Pure
    # presentation: collision and planning still see the original stock,
    # and playback, USD export, and recordings replay the same tracks.
    tl = scene.animate_carve(tl, "plate")

    # Hand-offs: USD with the toolpaths as BasisCurves, and the whole
    # cycle as URScript — movep chains at the feed, signals as digital
    # outputs.
    # Binary: this arm ships real CAD meshes, and their ASCII float
    # expansion is most of a `.usda` (117 MB against 26 MB here). The
    # crate file carries the same stage — `play_record` recognises it by
    # name, since a binary keeps its prim names out of reach.
    recording = Path("cell_machining.usdc")
    warnings = tl.export_usd(recording)
    print(f"USD: {recording} ({recording.stat().st_size / 1e6:.0f} MB, "
          f"{'no warnings' if not warnings else warnings})")
    script = tl.to_script(
        outputs={"clamped": 0, "spindle_run": 1},
        blend_radius=0.002,
    )
    Path("machining_cell.script").write_text(script)
    moveps = sum(1 for line in script.splitlines() if "movep(" in line)
    print(f"URScript: machining_cell.script ({moveps} movep, "
          f"{script.count('set_standard_digital_out')} digital writes)")

    if "--studio" in sys.argv:
        print(
            "\nstudio:\n"
            "  - press the play button in the timeline dock to run the cycle\n"
            "    (the cutting trail accumulates while spindle_run is on)\n"
            "  - the plate is cut away as the cycle plays: bright faces are\n"
            "    the surfaces the cutter made, grey is the surviving skin\n"
            "  - scrub the timeline to any point to see the material state\n"
            "    at that moment (t=0 is the pristine stock)"
        )
        bt.studio(scene)


if __name__ == "__main__":
    main()
