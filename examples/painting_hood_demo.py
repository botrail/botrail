"""Robot painting as a verifier: a curved hood panel, checked before it is
sprayed.

The K1 cell of design/design-painting.md. The same RV-5AS-D + bell as
`painting_demo.py`, now over a *curved* part — a hood-like cylindrical
shell (R 0.5 m, 360 x 240 mm) generated here for want of a catalog
workpiece — and the questions a paint engineer asks before the first
drop: is the gun the right distance away, square enough on, and did the
program actually point at the part?

What the cell shows:

* **Two programs from the surface, no CAM.** `bt.paint.strokes` rasters
  the part flat, the way a first attempt does; `bt.paint.wrap_strokes`
  wraps the same raster onto the cylinder so the gun stays radial. Both
  are generated from pattern width, overlap, speed and standoff — the
  shop's rules — not taught point by point.

* **The check that runs before anything is baked.** `scene.check_paint`
  looks along the spray axis at every spraying sample and reports standoff
  and incidence against the rules (here +-20 mm and 10 degrees). The flat
  raster is fine over the crown and breaks both rules over the shoulders,
  where the surface has curved away; the wrapped raster is clean to the
  millimetre. Pure geometry — no robot involved, so it is the same answer
  whichever arm ends up carrying the gun — and the studio draws the
  findings on the path.

* **The film is the second gate, not the same one.** Baked and integrated
  with the surface named (`facing=(0,0,1)`, so the rims never muddy the
  statistics), *both* rasters make spec: paint is conserved, so on a
  gentle curve the flat raster's longer, obliquer shoulders come out only
  a few percent thinner (23.3 vs 24.6 um). That is the honest shape of
  the model — what the standoff window protects (transfer efficiency,
  dry spray, sags, the electrostatic wrap) is precisely the physics a
  geometric film does not carry, so the check enforces the window and
  the film reports the coverage. Neither replaces the other.

* **A fixture error, caught before it is sprayed.** Shim the panel 30 mm
  proud of where the program was taught and `check_paint` says
  "too close" at every sample. Bake it anyway and the film's mean barely
  moves (conservation again) but its ripple triples: the footprint
  shrank while the pitch did not, and the laps stopped overlapping
  enough — striping, the classic too-close defect. Move the *whole
  fixture* — frame, panel, bench — and the cell simply re-solves: nothing
  was taught in joints.

* **Where the arm slows, and why it does not matter here.** At 150 mm/s
  this cobot holds 92% of the commanded speed on the wrapped raster; the
  slow stretches are all turnarounds, which the overtravel keeps off the
  part, so the film does not see them. That is what overtravel is for.
  (The K0 panel demo shows what happens when the slowdowns land *on* the
  part.)

The K2 half (design §4.3) is about the trigger and the paint bill:

* **Brushes trigger per stroke.** `scene.define_brush` names an
  applicator, a flow and a lead/lag; `wrap_strokes(brush=...)` puts the
  brush on the laps and leaves the side-steps dry. Same film, a fifth
  less paint: the turnarounds were spraying the bench.

* **Overtravel against trigger timing, in numbers.** Drop the overtravel
  and the cycle is 13 s shorter — but with the gun open through the
  turnaround its dwell lands on the part (thick ends, 61% in spec), and
  with it closed at the edge the ends starve (14 um, 80%). A quarter
  second of lead and lag buys most of it back (93%); half a second
  overshoots. That is the knob a paint programmer turns all day, and the
  film map shows each setting.

* **Two brushes, one cycle.** A primer pass at 0.6 flow and a topcoat at
  1.0, sprayed in one sequence and accounted per brush.

* **Where the paint went.** `overspray()` names every physical obstacle
  that took paint — the bench under the hood catches half of what the
  continuous raster sprays — and a masking strip laid across the hood
  both shadows the film beneath it and shows up in the bill: a masked
  fixture that took paint is a mask that leaked.

Run with:  python examples/painting_hood_demo.py [--studio]
"""

import math
from pathlib import Path
import sys

import botrail as bt

CATALOG_ARM = "mitsubishi_electric/assista/rv-5as-d/r1"
GUN_URDF = Path(__file__).parent / "assets" / "spray_gun.urdf"

# The part: a cylindrical shell curving across y (its axis runs along x,
# the direction the arm reaches), crown at the part frame's origin. A
# hood section, near enough.
HOOD_R = 0.50
HOOD_LEN = 0.24     # along the axis (x)
HOOD_CHORD = 0.36   # across the arc (y)
HOOD_THICK = 0.006
HALF_ANGLE = math.asin(HOOD_CHORD / 2 / HOOD_R)   # ~21 degrees each way

# The process, and the rules it is checked against.
STANDOFF = 0.25
PATTERN = 0.16
GUN_SPEED = 0.15
OVERTRAVEL = 0.10
LAP_MARGIN = PATTERN / 4
OVERLAP = 0.6
RULES = {"standoff": (0.23, 0.27), "max_incidence": math.radians(10)}
TARGET_FILM = 25e-6
SPEC = (20e-6, 30e-6)
TRANSFER_EFFICIENCY = 0.85
PATCH = 0.004

# The taught stance: bell vertical over the crown, 420 mm out and up.
REF_Q = [0.0, -0.18645, 1.630698, -0.0, 1.697344, 0.0]

FIXTURE = ("hood", "bench")


def hood_mesh(path: Path, around: int = 36, along: int = 12) -> Path:
    """Writes the hood as an OBJ: outer and inner cylindrical faces plus
    four rims, outward-wound, in the part frame (crown at the origin, the
    cylinder's axis along x through `(0, 0, -R)`)."""
    verts: list[tuple[float, float, float]] = []
    faces: list[tuple[int, int, int]] = []

    def ring(r: float) -> int:
        base = len(verts)
        for i in range(around + 1):
            th = -HALF_ANGLE + 2 * HALF_ANGLE * i / around
            for j in range(along + 1):
                x = -HOOD_LEN / 2 + HOOD_LEN * j / along
                verts.append((x, r * math.sin(th), -HOOD_R + r * math.cos(th)))
        return base

    outer, inner = ring(HOOD_R), ring(HOOD_R - HOOD_THICK)

    def at(base: int, i: int, j: int) -> int:
        return base + i * (along + 1) + j

    # Wound so every face points out of the shell: with the axis along x,
    # (angle, x) is a left-handed pair on the outer face, hence the order.
    for i in range(around):
        for j in range(along):
            a, b, c, d = at(outer, i, j), at(outer, i + 1, j), at(outer, i + 1, j + 1), at(outer, i, j + 1)
            faces += [(a, c, b), (a, d, c)]
            a, b, c, d = at(inner, i, j), at(inner, i + 1, j), at(inner, i + 1, j + 1), at(inner, i, j + 1)
            faces += [(a, b, c), (a, c, d)]
    for j in range(along):
        a, b, c, d = at(outer, 0, j), at(outer, 0, j + 1), at(inner, 0, j + 1), at(inner, 0, j)
        faces += [(a, c, b), (a, d, c)]
        a, b, c, d = at(outer, around, j), at(outer, around, j + 1), at(inner, around, j + 1), at(inner, around, j)
        faces += [(a, b, c), (a, c, d)]
    for i in range(around):
        a, b, c, d = at(outer, i, 0), at(outer, i + 1, 0), at(inner, i + 1, 0), at(inner, i, 0)
        faces += [(a, b, c), (a, c, d)]
        a, b, c, d = at(outer, i, along), at(outer, i + 1, along), at(inner, i + 1, along), at(inner, i, along)
        faces += [(a, c, b), (a, d, c)]

    with open(path, "w", encoding="utf-8") as f:
        f.write("# hood section: cylindrical shell, axis along x, crown at the origin\n")
        for v in verts:
            f.write(f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
        for t in faces:
            f.write(f"f {t[0] + 1} {t[1] + 1} {t[2] + 1}\n")
    return path


def build_scene(mesh_dir: Path | None = None) -> tuple[bt.Scene, str]:
    """Arm + bell over the hood on a bench, the part frame taught from the
    stance so the whole job is authored relative to it."""
    arm = bt.Robot.from_catalog(CATALOG_ARM)
    gun = bt.Robot.from_urdf(GUN_URDF)
    robot = arm.attach_tool(gun, flange=arm.flange_link)
    scene = bt.Scene(robot)
    scene.set_joint_positions(REF_Q)

    tip, _ = scene.link_pose(robot.tcp_link)
    origin = (tip[0], tip[1], tip[2] - STANDOFF)
    scene.add_frame("part", position=origin)

    mesh_dir = mesh_dir or Path(__file__).parent / "assets"
    hood = hood_mesh(mesh_dir / "hood_section.obj")
    scene.add_mesh("hood", hood, position=origin)
    scene.set_obstacle_color("hood", (0.62, 0.63, 0.66))
    scene.set_obstacle_material("hood", metalness=0.1, roughness=0.5)
    # The bench under it, standing on the floor. The hood's shoulders drop
    # to z = -R(1 - cos(half angle)) below the crown; leave that clear.
    sag = HOOD_R * (1 - math.cos(HALF_ANGLE))
    bench_h = origin[2] - sag - 0.02
    scene.add_box(
        "bench",
        size=(HOOD_LEN + 0.20, HOOD_CHORD + 0.20, bench_h),
        position=(origin[0], origin[1], bench_h / 2),
    )
    scene.set_obstacle_color("bench", (0.30, 0.31, 0.33))
    # The cell's I/O: purge (colour change / clean) and the gun itself —
    # plus `spraying`, which nothing in the sequence writes: it is the
    # effective trigger (enable AND program) a baked timeline fills in,
    # for the timing chart and the spray-cone effect to follow.
    scene.define_signal("purge")
    scene.define_signal("gun_on")
    scene.define_signal("spraying")
    # The jet: a spray cone the size of the standoff and pattern, bound to
    # the effective trigger. Declared with the cell — a timeline exports
    # the scene it was baked from, effects included, so this is not
    # something to add after the bake. Presentation only.
    scene.add_spray_cone("jet", "spraying", scene.robots[0],
                         length=STANDOFF, radius=PATTERN / 2)
    return scene, robot.tcp_link


def flat_raster(overlap: float = OVERLAP) -> bt.toolpath.Toolpath:
    """The first attempt: raster the hood as if it were flat, laps along
    x, at the crown's height everywhere."""
    return bt.paint.strokes(
        (HOOD_LEN, HOOD_CHORD),
        standoff=STANDOFF,
        pattern_width=PATTERN,
        overlap=overlap,
        speed=GUN_SPEED,
        overtravel=OVERTRAVEL,
        margin=LAP_MARGIN,
        direction="x",
        frame="part",
    )


def wrapped_raster(overlap: float = OVERLAP) -> bt.toolpath.Toolpath:
    """The same raster wrapped onto the cylinder: laps along the axis,
    stepped in angle, the gun radial at every point."""
    return bt.paint.wrap_strokes(
        HOOD_R,
        HOOD_LEN,
        standoff=STANDOFF,
        pattern_width=PATTERN,
        overlap=overlap,
        speed=GUN_SPEED,
        overtravel=OVERTRAVEL,
        arc=(-HALF_ANGLE, HALF_ANGLE),
        margin=LAP_MARGIN,
        center=(0.0, 0.0, -HOOD_R),
        axis="x",
        frame="part",
    )


def applicator() -> dict:
    """A bell whose flow lands `TARGET_FILM` at the raster's pitch and
    speed (film goes as flow / (speed x pitch))."""
    pitch = PATTERN * (1.0 - OVERLAP)
    return bt.paint.applicator(
        bt.paint.bell(PATTERN),
        standoff=STANDOFF,
        flow=TARGET_FILM * GUN_SPEED * pitch / TRANSFER_EFFICIENCY,
        transfer_efficiency=TRANSFER_EFFICIENCY,
    )


# --- K2: brushes, trigger, overspray -----------------------------------

PRIMER_FLOW = 0.6
LEAD_LAG = 0.25   # seconds; the setting the matrix below arrives at


def define_process(scene: bt.Scene, lead: float = 0.0, lag: float = 0.0) -> None:
    """The bell as a scene resident, and two brushes on it: a primer at
    reduced flow and the topcoat at full, both with the given trigger
    timing. Brushes are the program's own trigger — the PLC's `gun_on`
    still has to agree."""
    scene.define_applicator("bell", applicator())
    scene.define_brush("primer", applicator="bell", flow=PRIMER_FLOW, lead=lead, lag=lag)
    scene.define_brush("top", applicator="bell", flow=1.0, lead=lead, lag=lag)


def raster(
    overtravel: float = OVERTRAVEL,
    brush: str | None = None,
    trigger: str = "stroke",
    overlap: float = OVERLAP,
) -> bt.toolpath.Toolpath:
    """The wrapped raster with the trigger spelled out: `brush=None` is
    the K1 program (one continuous feed move, sprayed with whatever
    applicator `spray_coat` is handed); a brush makes the laps spray with
    it and the side-steps run dry (`trigger="stroke"`) or wet
    (`"continuous"`)."""
    return bt.paint.wrap_strokes(
        HOOD_R,
        HOOD_LEN,
        standoff=STANDOFF,
        pattern_width=PATTERN,
        overlap=overlap,
        speed=GUN_SPEED,
        overtravel=overtravel,
        arc=(-HALF_ANGLE, HALF_ANGLE),
        margin=LAP_MARGIN,
        center=(0.0, 0.0, -HOOD_R),
        axis="x",
        frame="part",
        brush=brush,
        trigger=trigger,
    )


def coat_programs(
    scene: bt.Scene,
    names: list,
    applicator_dict: dict | None = None,
    spec: tuple = SPEC,
) -> tuple:
    """Bakes one cycle that runs `names` in order with the gun enabled
    throughout, and integrates the film. `applicator_dict` is only needed
    when a program names no brush."""
    sq = scene.sequence("cycle")
    sq.step("purge", actions=[bt.seq.set_signal("purge")],
            transition=bt.seq.elapsed(2.0))
    sq.step("ready", actions=[bt.seq.set_signal("purge", False),
                              bt.seq.set_signal("gun_on")],
            transition=bt.seq.elapsed(0.2))
    for i, name in enumerate(names):
        sq.step(f"spray_{i + 1}", actions=[bt.seq.toolpath(name)],
                transition=bt.seq.done())
    sq.step("close", actions=[bt.seq.set_signal("gun_on", False)],
            transition=bt.seq.elapsed(0.2))
    timeline = sq.simulate()
    film = timeline.spray_coat(
        "hood",
        applicator_dict,
        gate="gun_on",
        patch_size=PATCH,
        spec=spec,
        facing=(0.0, 0.0, 1.0),
        facing_tolerance=HALF_ANGLE + math.radians(5),
    )
    return timeline, film


# A two-coat build: primer at PRIMER_FLOW plus the topcoat, so the spec is
# the single-coat one scaled by (1 + PRIMER_FLOW).
TWO_COAT_SPEC = (SPEC[0] * (1 + PRIMER_FLOW), SPEC[1] * (1 + PRIMER_FLOW))

# A strip of masking tape over the crown, 30 x 100 mm, a few millimetres
# above the skin (the hood is within 3 mm of flat there, so the strip's
# shadow is its footprint).
MASK = ("mask", (0.03, 0.10, 0.003))


def add_mask(scene: bt.Scene) -> None:
    """A masking strip laid over the hood near one end — where trim would
    go. A real, enabled obstacle: it shadows the film beneath it and
    catches paint."""
    fp, _ = scene.frame("part")
    name, size = MASK
    scene.add_box(name, size=size, position=(fp[0] - 0.09, fp[1], fp[2] + 0.004))
    scene.set_obstacle_color(name, (0.85, 0.72, 0.30))


def check(scene: bt.Scene, name: str) -> "bt.PaintReport":
    """The pre-bake check against the shop's rules; the findings stay on
    the path in the studio."""
    return scene.check_paint(name, "hood", **RULES)


def coat(scene: bt.Scene, name: str) -> tuple:
    """Bakes the coating cycle for toolpath `name` and integrates the film
    it left on the hood's outer face."""
    # PLC vocabulary: purge the gun (colour change / clean), then open it
    # for the raster, then close it. The gun follows `gun_on`; the film
    # integrator gates on the same signal.
    sq = scene.sequence("cycle")
    sq.step("purge", actions=[bt.seq.set_signal("purge")],
            transition=bt.seq.elapsed(2.0))
    sq.step("ready", actions=[bt.seq.set_signal("purge", False)],
            transition=bt.seq.elapsed(0.2))
    sq.step("spray", actions=[bt.seq.set_signal("gun_on"), bt.seq.toolpath(name)],
            transition=bt.seq.done())
    sq.step("close", actions=[bt.seq.set_signal("gun_on", False)],
            transition=bt.seq.elapsed(0.2))
    timeline = sq.simulate()
    # `facing` names the job: the outer face, whose normals lie within the
    # half angle (plus a little) of +z. Without it the rims would swing
    # in and out of the statistics as the gun turns around past them.
    film = timeline.spray_coat(
        "hood",
        applicator(),
        gate="gun_on",
        patch_size=PATCH,
        spec=SPEC,
        facing=(0.0, 0.0, 1.0),
        facing_tolerance=HALF_ANGLE + math.radians(5),
    )
    return timeline, film


def shift_fixture(scene: bt.Scene, dx: float) -> None:
    """Moves the whole fixture — part frame, hood, bench — along x. The
    rasters are authored in the part frame, so they follow and re-solve."""
    fp, fq = scene.frame("part")
    scene.add_frame("part", position=(fp[0] + dx, fp[1], fp[2]), quaternion=fq)
    for name in FIXTURE:
        p, q = scene.obstacle_pose(name)
        scene.set_obstacle_pose(name, position=(p[0] + dx, p[1], p[2]), quaternion=q)


def raise_panel(scene: bt.Scene, dz: float) -> None:
    """A fixture error: the hood alone sits `dz` proud of where the
    program was taught. The frame — and so the program — stays put."""
    p, q = scene.obstacle_pose("hood")
    scene.set_obstacle_pose("hood", position=(p[0], p[1], p[2] + dz), quaternion=q)


def main() -> None:
    scene, _ = build_scene()
    scene.add_toolpath("flat", flat_raster())
    scene.add_toolpath("wrapped", wrapped_raster())

    # --- the check, before anything is baked -----------------------------
    print(f"rules: standoff {RULES['standoff'][0] * 1e3:.0f}-{RULES['standoff'][1] * 1e3:.0f} mm, "
          f"incidence <= {math.degrees(RULES['max_incidence']):.0f} deg\n")
    reports = {}
    for name in ("flat", "wrapped"):
        rep = check(scene, name)
        reports[name] = rep
        print(f"{name:8s} {rep!r}")
        print(f"         on target {rep.on_target_ratio:.0%} of the spraying; where it was, "
              f"{rep.in_band_ratio:.0%} in band")
        for kind in ("too_far", "oblique", "too_close"):
            spans = rep.spans(kind)
            if spans:
                where = ", ".join(f"{a:.2f}-{b:.2f} m" for a, b in spans[:4])
                more = f" (+{len(spans) - 4} more)" if len(spans) > 4 else ""
                print(f"         {kind:9s} along the path at {where}{more}")
        reach = scene.check_toolpath(name)
        print(f"         reach: {'every sample solves' if reach.ok else reach!r}")
    print("  ^ the flat raster is fine over the crown and breaks both rules over\n"
          "    the shoulders; wrapped onto the cylinder the gun stays radial.\n")

    # --- the film: the second gate ---------------------------------------
    films = {}
    for name in ("flat", "wrapped"):
        timeline, film = coat(scene, name)
        films[name] = (timeline, film)
        baked = timeline.paint_report("hood", gate="gun_on", **RULES)
        held = timeline.feed_report(name).hold_ratio
        print(f"{name:8s} cycle {timeline.duration:5.1f}s  held {held:5.1%}  "
              f"film {film.mean * 1e6:4.1f} um ({film.min * 1e6:.1f}-{film.max * 1e6:.1f}, "
              f"spread {film.sigma / film.mean:.1%})  in spec {film.in_spec_ratio:6.1%}  |  "
              f"baked check {'ok' if baked.ok else 'flagged'}, "
              f"{baked.on_target_ratio:.0%} on target")
    print("  ^ both make spec — paint is conserved, so on a gentle curve the flat\n"
          "    raster's longer, obliquer shoulders are only a few percent thinner.\n"
          "    The check enforces the process window (transfer efficiency, dry\n"
          "    spray, sags, wrap — physics the geometric film does not carry);\n"
          "    the film reports the coverage. Two gates. The arm's slow stretches\n"
          "    are all turnarounds, which the overtravel keeps off the part.\n")

    # --- the fixture -------------------------------------------------------
    base = films["wrapped"][1]
    shift_fixture(scene, -0.03)
    moved_tl, moved = coat(scene, "wrapped")
    print(f"fixture -30 mm (frame + hood + bench): cycle {moved_tl.duration:.1f}s, "
          f"film {moved.mean * 1e6:.1f} um, in spec {moved.in_spec_ratio:.1%} "
          f"(was {base.in_spec_ratio:.1%}) — re-solved, nothing taught in joints")
    shift_fixture(scene, 0.03)

    raise_panel(scene, 0.03)
    proud = check(scene, "wrapped")
    print(f"hood 30 mm proud (a fixture error): {proud!r}")
    proud_tl, proud_film = coat(scene, "wrapped")
    print(f"   sprayed anyway: film {proud_film.mean * 1e6:.1f} um "
          f"({proud_film.min * 1e6:.1f}-{proud_film.max * 1e6:.1f}), spread "
          f"{proud_film.sigma / proud_film.mean:.1%} (was {base.sigma / base.mean:.1%}) — "
          f"the footprint shrank, the pitch did not: striping. The check said so\n"
          f"   before the bake.")
    raise_panel(scene, -0.03)
    # Restore the marks of the taught cell for the studio.
    check(scene, "flat")
    check(scene, "wrapped")

    # === K2: the trigger, the brushes, the bill ============================
    print("\n--- brushes and the trigger ---")
    define_process(scene)
    bell = applicator()
    scene.add_toolpath("continuous", raster(OVERTRAVEL, None))
    scene.add_toolpath("stroke", raster(OVERTRAVEL, "top"))
    scene.add_toolpath("continuous_0", raster(0.0, None))
    scene.add_toolpath("stroke_0", raster(0.0, "top"))
    print("  program                        cycle  gun on   film        spread  in spec  sprayed  on hood   bench   transfer")
    matrix = {}

    def row(label: str, names: list, app: dict | None) -> None:
        tl, f = coat_programs(scene, names, app)
        matrix[label] = (tl, f)
        bench = f.overspray().get("bench", 0.0)
        print(f"  {label:30s} {tl.duration:5.1f}s  {f.gun_on_time:5.1f}s  "
              f"{f.mean * 1e6:4.1f} um ({f.min * 1e6:4.1f}-{f.max * 1e6:4.1f})  "
              f"{f.sigma / f.mean:5.1%}  {f.in_spec_ratio:6.1%}  {f.sprayed_volume * 1e6:5.1f} cc  "
              f"{f.deposited_volume * 1e6:4.1f} cc  {bench * 1e6:4.1f} cc  "
              f"{f.effective_transfer_efficiency:5.1%}")

    row("continuous, overtravel 100", ["continuous"], bell)
    row("per stroke, overtravel 100", ["stroke"], None)
    row("continuous, overtravel 0", ["continuous_0"], bell)
    row("per stroke, overtravel 0", ["stroke_0"], None)
    define_process(scene, lead=LEAD_LAG, lag=LEAD_LAG)
    row(f"per stroke, ot 0, +-{LEAD_LAG:.2f}s", ["stroke_0"], None)
    define_process(scene, lead=2 * LEAD_LAG, lag=2 * LEAD_LAG)
    row(f"per stroke, ot 0, +-{2 * LEAD_LAG:.2f}s", ["stroke_0"], None)
    define_process(scene)
    print(
        "  ^ per-stroke triggering: same film, a fifth less paint — the side-steps\n"
        "    were spraying the bench. Drop the overtravel and the cycle is 13 s\n"
        "    shorter, but the turnaround dwell lands on the part with the gun open\n"
        "    (thick ends) and the ends starve with it closed at the edge; a quarter\n"
        "    second of lead and lag buys most of it back, half a second overshoots.\n"
        "    That knob is what a paint programmer turns all day."
    )

    print("\n--- two brushes, one cycle ---")
    scene.add_toolpath("primer", raster(OVERTRAVEL, "primer"))
    two_tl, two = coat_programs(scene, ["primer", "stroke"], None, spec=TWO_COAT_SPEC)
    by_sprayed = two.sprayed_by_brush()
    by_dep = two.deposited_by_brush()
    print(f"  primer @ {PRIMER_FLOW:.1f} + topcoat @ 1.0: cycle {two_tl.duration:.1f}s, film "
          f"{two.mean * 1e6:.1f} um, in spec {two.in_spec_ratio:.1%} of a "
          f"{TWO_COAT_SPEC[0] * 1e6:.0f}-{TWO_COAT_SPEC[1] * 1e6:.0f} um two-coat build")
    for name in ("primer", "top"):
        print(f"    {name:7s} sprayed {by_sprayed[name] * 1e6:4.1f} cc, on the hood "
              f"{by_dep[name] * 1e6:4.1f} cc")

    print("\n--- where the paint went ---")
    add_mask(scene)
    masked_tl, masked = coat_programs(scene, ["stroke"], None)
    _, unmasked = matrix["per stroke, overtravel 100"]
    print(f"  masking strip over the crown: addressed area "
          f"{unmasked.total_area * 1e4:.0f} -> {masked.total_area * 1e4:.0f} cm2 "
          f"(the shadow leaves the statistics; its penumbra, "
          f"{masked.uncoated_area * 1e4:.0f} cm2 under the strip's edge, stays a holiday); "
          f"paint on the hood {unmasked.deposited_volume * 1e6:.2f} -> "
          f"{masked.deposited_volume * 1e6:.2f} cc")
    for name, v in sorted(masked.overspray().items(), key=lambda kv: -kv[1]):
        print(f"    {name:6s} took {v * 1e6:4.1f} cc")
    print(f"    lost   {masked.lost_volume * 1e6:4.1f} cc past everything (incl. "
          f"{(1 - TRANSFER_EFFICIENCY):.0%} atomization loss)")
    print("  ^ a masked fixture that took paint is a mask that leaked; the bench\n"
          "    is where the overtravel goes.")
    scene.remove_obstacle(MASK[0])
    for name in ("continuous", "continuous_0", "stroke_0", "primer"):
        scene.remove_toolpath(name)

    # --- the film map, and the film building up ----------------------------
    timeline, film = matrix["per stroke, overtravel 100"]
    film_obj = Path("painting_hood_film.obj").resolve()
    film.save_obj(film_obj)
    print(f"\nfilm map: {film_obj.name} (+ .mtl) — {film!r}")

    # Progressive build-up: re-walk the coat in stages and swap the visible
    # mesh along the timeline — the hood's own grey holds until the first
    # stroke, then each snapshot shows for its window. The effective
    # trigger (enable AND program, lead/lag included) is written as the
    # `spraying` lane, and a spray cone follows it. Pure presentation:
    # collision and planning still see the original hood, and playback,
    # USD export, and recordings replay the same tracks.
    # Coloured against spec (the verdict) rather than by amount, because
    # this is the verifier's demo: neutral is on target, blue thin, red
    # thick — the panel demo shows the other reading, the paint's colour
    # going on.
    timeline = scene.animate_paint(
        timeline, "hood", gate="gun_on", spec=SPEC, style="spec",
        facing=(0.0, 0.0, 1.0), facing_tolerance=HALF_ANGLE + math.radians(5),
        trigger_signal="spraying",
    )
    spraying = timeline.signal("spraying").high_total()
    print(f"animate_paint: {len([n for n in scene.obstacle_names if n.startswith('hood_film/')])} "
          f"stages, spraying {spraying:.1f}s of the {timeline.duration:.1f}s cycle "
          f"(gun_on {timeline.signal('gun_on').high_total():.1f}s)")

    # Hand-off: USD with the strokes as BasisCurves, the film building up
    # as visibility-switched stages, and the jet as a beam.
    recording = Path("cell_painting.usdc")
    warnings = timeline.export_usd(recording)
    print(f"USD: {recording} ({recording.stat().st_size / 1e6:.1f} MB, "
          f"{'no warnings' if not warnings else warnings})")

    if "--studio" in sys.argv:
        print(
            "\nstudio:\n"
            "  - press play: the film builds up on the hood stroke by stroke\n"
            "    (the key at the top right reads it against spec: neutral is on\n"
            "    target, blue thin, red thick) and the jet shows while spraying\n"
            "  - the `spraying` lane in the dock is the effective trigger —\n"
            "    enable AND program — not the enable alone\n"
            "  - the coloured points on the flat raster are check_paint's\n"
            "    findings: blue too far, yellow oblique, grey off target"
        )
        bt.studio(scene)


if __name__ == "__main__":
    main()
