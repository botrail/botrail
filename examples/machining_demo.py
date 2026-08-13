"""Robot machining, C1+C2: a trimming cell with a 5-axis rim chamfer.

The C0 slice of design/design-machining.md proved a toolpath can be baked
at commanded feed; C1 grew it into a cell (live stock through an allowed
cutter-plate contact, cuts as sequence steps, URScript movep hand-off).
C2 adds the parts that make it *practice*:

* **A 5-axis path from APT.** The rim chamfer arrives as CL text
  (`GOTO/x,y,z,i,j,k` — the machine-independent 5-axis format), its tool
  axis leaning 30 deg outward all the way around the plate, hopping the
  toe clamps. The lean direction precesses a full turn per lap.

* **Global spin optimization where greedy dies.** Followed greedily, the
  wrist walks into a stretch it cannot pass (44 located failures on the
  base-facing edge). `spin="optimize"` runs a Descartes-style Viterbi
  pass over spin candidates around the natural solution and solves the
  same path whole — spending spin early to stay solvable late. The cell's
  cycle therefore bakes with `toolpath_spin="optimize"`.

* **The feed report.** Floors make `length / feed` a hard lower bound,
  so joints can only slow a cut: `feed_report` says how much was lost
  (hold ratio), where (time spans), and which axis owned each slowdown.

Everything else is C1: clamp -> spin-up -> cuts -> spin-down -> unclamp,
USD with BasisCurves overlays, URScript with movep chains and digital
I/O. Move the fixture and re-simulate: nothing was taught in joints.

Run with:  python examples/machining_demo.py [--studio]
"""

from pathlib import Path
import sys

import botrail as bt

ASSETS = Path(__file__).resolve().parent / "assets"

# Fixture geometry, all in the part frame (meters). The plate top is z=0.
# Sized to the little demo arm (a ~0.85 m 6-axis): the valid band with the
# spindle vertical runs x 0.33-0.45 from the base, so the job is a
# 140 x 100 plate rather than a door panel.
PLATE = (0.14, 0.10, 0.012)
CONTOUR_HALF = (0.06, 0.04)  # trim line, 10 mm inside the plate edge
CORNER_R = 0.015
CUT_DEPTH = -0.002
RAPID_Z = 0.008
CUT_FEED = 0.015     # 15 mm/s trim
PLUNGE_FEED = 0.004  # 4 mm/s entry
RAPID_CAP = 0.25     # m/s, movel-style rapid cap

# A working configuration with the flange facing the floor (the pitch
# joints sum to pi), elbow dipped *under* the shoulder line — folding the
# wrist over the top instead pushes its links into the forearm with the
# spindle mounted. Found by scanning flange-down configurations for the
# widest collision-free reach band.
REF_Q = [0.0, 0.5, 0.9, 1.7415926535897932, 0.0, 0.0]

FIXTURE_BODIES = ("plate", "table", "clamp_front", "clamp_back")


def build_scene() -> tuple[bt.Scene, str]:
    arm = bt.Robot.from_urdf(str(Path(__file__).resolve().parent / "simple_arm.urdf"))
    spindle = bt.Robot.from_urdf(str(ASSETS / "spindle.urdf"))
    robot = arm.attach_tool(spindle, flange="tool0")
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
    # Table and toe clamps: ordinary collision. The clamps grip the long
    # edges 3 mm onto the plate, leaving 7 mm to the trim line.
    scene.add_box("table", size=(0.30, 0.24, 0.06), position=at(0, 0, -PLATE[2] - 0.03))
    scene.set_obstacle_color("table", (0.35, 0.33, 0.30))
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
    import math

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
    `cell_machining.usda` onto. The rim chamfer's spin needs the global
    optimizer, so simulate with ``toolpath_spin="optimize"``."""
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

    `main()` exports `cell_machining.usda` with the progressive-removal
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

    # Face diagnosis first: every sample attempted, failures located. The
    # rim is the C2 case — greedy walks the wrist into the base-facing
    # edge; the global spin pass solves the same path whole.
    for name in ("contour", "pocket"):
        report = scene.check_toolpath(name)
        print(f"check {name}: {report!r}")
        if not report.ok:
            sys.exit(f"unexpected: {report.issues[:3]}")
    rim_greedy = scene.check_toolpath("rim")
    rim_opt = scene.check_toolpath("rim", spin="optimize")
    print(f"check rim:  greedy {len(rim_greedy.issues)}/{rim_greedy.total_samples} fail "
          f"(first: {rim_greedy.issues[0]['kind']} at sample {rim_greedy.issues[0]['sample']}) "
          f"| optimize {'ok' if rim_opt.ok else 'FAIL'}")
    if not rim_opt.ok:
        sys.exit(f"unexpected: {rim_opt.issues[:3]}")

    # The cycle: clamp -> spin up -> trim -> pocket -> chamfer -> spin
    # down -> unclamp. The chamfer needs the optimizer, so the whole bake
    # runs with it.
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
    warnings = tl.export_usd("cell_machining.usda")
    print(f"USD: cell_machining.usda ({'no warnings' if not warnings else warnings})")
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
