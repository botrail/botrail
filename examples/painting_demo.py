"""Robot painting: a panel coated by a rotary bell, and the film it left.

The K0 cell of design/design-painting.md. A Mitsubishi Electric RV-5AS-D
(`manipulator`, catalog) carries a hand-authored bell applicator
(`examples/assets/spray_gun.urdf`, standing in for a catalog product) over
a flat panel in a serpentine raster, and `spray_coat` walks the baked
cycle to report what the paint actually did.

What the cell shows:

* **A calibrated footprint, not a guess.** The applicator's pattern is the
  static-pattern coupon a shop sprays to characterize a gun — a radial
  film profile at a known standoff and time. `bt.paint.from_profile`
  recovers both the shape and the delivery rate from it, and the demo
  checks that against the analytic bell it was generated from. This is
  *calibrated geometry*: no air flow, no electrostatics, so an ESTA
  bell's wrap around an edge is not modeled and the absolute microns are
  only as good as the coupon.

* **Lap overlap, decided by the numbers.** Sweep the overlap holding the
  target film constant (change the pitch, change the flow to match — the
  shop's own procedure) and the uniformity is what moves: 20% overlap
  leaves 29% of the panel in spec and a 25% spread; from 30% up the panel
  makes spec and the spread falls to 2-3%. Note that 40% (2.9%) beats
  50% (8.4%): laps do not sum monotonically — the lap sum beats against
  the pattern width — which is exactly the sort of thing a rule of thumb
  gets wrong and a simulation does not.

* **Why the film has to ride the baked trajectory.** Film goes as
  `flow / (speed x pitch)`, so a gun that cannot hold its commanded speed
  lays on more paint. Command 0.30 m/s — ordinary for a paint robot — on
  this cobot and it holds 22% of it, so the panel comes out at 109 um
  against a 20-30 um spec: nothing in spec, a cycle that got *slower*,
  and a defect a constant-speed simulation would never show. That
  coupling of `feed_report` to film is the thing botrail can say that a
  kinematics-free film calculator cannot.

`spray_coat` reports over the surface the gun *addressed* — in range and
within `max_incidence` of square on. A part's back face is not a holiday,
and neither is the rim of a panel sprayed from above.

Two triggers decide when paint flows: the PLC's enable (`gun_on`, set here
in the same step the raster starts) and the program's own — the feed
strokes. The approach the rollout plans in from the taught stance, and the
rapids, never spray the part however the enable was authored.

Run with:  python examples/painting_demo.py [--studio]
"""

import math
from pathlib import Path
import sys

import botrail as bt

CATALOG_ARM = "mitsubishi_electric/assista/rv-5as-d/r1"
GUN_URDF = Path(__file__).parent / "assets" / "spray_gun.urdf"

# The job, all in the part frame (meters). The panel top is z=0.
PANEL = (0.24, 0.18, 0.006)
STANDOFF = 0.25       # bell face to panel; the pattern's reference plane
PATTERN = 0.16        # footprint diameter there
GUN_SPEED = 0.15      # m/s. A paint robot runs 3-5x this; see the docstring
OVERTRAVEL = 0.10     # past the panel edge, so the turnaround cone misses it
LAP_MARGIN = PATTERN / 2   # how far the lap set reaches past the panel edge
TARGET_FILM = 25e-6   # 25 um, the middle of spec
SPEC = (20e-6, 30e-6)
TRANSFER_EFFICIENCY = 0.85   # an electrostatic bell's, roughly
PATCH = 0.004         # film-map resolution; a twentieth of the pattern
PAINT_COLOR = (0.72, 0.10, 0.06)   # the paint, linear RGB — the studio's build-up ramp

# The taught stance: bell vertical, tip 420 mm out and 420 mm up, which
# puts the panel on the bench at a comfortable standoff. Solved once with
# `robot.ik` and pinned here, the way a stance is taught.
REF_Q = [0.0, -0.18645, 1.630698, -0.0, 1.697344, 0.0]


def build_scene() -> tuple[bt.Scene, str]:
    """Arm + bell over a panel on a bench, with the part frame taught from
    the stance so the whole job is authored relative to it."""
    arm = bt.Robot.from_catalog(CATALOG_ARM)
    gun = bt.Robot.from_urdf(GUN_URDF)
    # The gun's root is its mounting face and its tcp is the bell face, so
    # the flange name is the only thing to say.
    robot = arm.attach_tool(gun, flange=arm.flange_link)
    scene = bt.Scene(robot)
    scene.set_joint_positions(REF_Q)

    # Teach the part datum: the panel top sits one standoff below the bell
    # face, so the taught stance *is* the spray plane.
    tip, _ = scene.link_pose(robot.tcp_link)
    origin = (tip[0], tip[1], tip[2] - STANDOFF)
    scene.add_frame("part", position=origin)

    scene.add_box(
        "panel",
        size=PANEL,
        position=(origin[0], origin[1], origin[2] - PANEL[2] / 2),
    )
    scene.set_obstacle_color("panel", (0.62, 0.63, 0.66))
    bench_h = origin[2] - PANEL[2]
    scene.add_box(
        "bench",
        size=(PANEL[0] + 0.20, PANEL[1] + 0.20, bench_h),
        position=(origin[0], origin[1], bench_h / 2),
    )
    scene.set_obstacle_color("bench", (0.30, 0.31, 0.33))
    # The cell's I/O. Painting is non-contact, so unlike the machining
    # cell there is no contact exemption to arrange — the gun and the arm
    # are checked against the panel and the bench the ordinary way.
    scene.define_signal("gun_on")
    # `spraying` is the effective trigger (enable AND program) a baked
    # timeline fills in; the jet — a cone the standoff long and the
    # pattern wide, with its footprint ring on the panel — follows it.
    # Declared with the cell: a timeline exports the scene it was baked
    # from, effects included. Presentation only.
    scene.define_signal("spraying")
    scene.add_spray_cone("jet", "spraying", scene.robots[0],
                         length=STANDOFF, radius=PATTERN / 2)
    return scene, robot.tcp_link


def coupon_profile(applicator: dict, seconds: float = 3.0, samples: int = 13) -> str:
    """The static-pattern coupon this applicator would leave: spray it at
    its reference standoff for `seconds` without moving, and measure the
    film across the disc.

    Generated here from the analytic bell so the demo has one to read —
    the real article comes off a panel and a film gauge. Two columns,
    radius and film, both meters, which is what `bt.paint.from_profile`
    reads.
    """
    radius = applicator["pattern"]["diameter"] / 2.0
    rate = applicator["flow"] * applicator["transfer_efficiency"]
    # The normalized round-beta shape: (1 - (r/R)^2)^(beta-1), scaled so it
    # integrates to one over the disc. For beta=2 that integral is
    # pi*R^2/2.
    beta = applicator["pattern"]["beta"]
    norm = math.pi * radius**2 / 2.0
    rows = ["# radius[m], film[m] — static pattern, %.0fs at %.0f mm" % (
        seconds, applicator["standoff"] * 1e3)]
    for i in range(samples):
        r = radius * i / (samples - 1)
        shape = max(0.0, 1.0 - (r / radius) ** 2) ** (beta - 1.0)
        rows.append(f"{r:.5f}, {rate * seconds * shape / norm:.9f}")
    return "\n".join(rows) + "\n"


def applicator_for(pitch: float, speed: float = GUN_SPEED) -> dict:
    """A bell whose flow lands `TARGET_FILM` at this pitch and speed.

    Film goes as `flow / (speed x pitch)`, so changing the lap pitch means
    changing the flow to keep the target — which is what a paint engineer
    does at the gun, and what makes an overlap sweep a comparison of
    *uniformity* rather than of averages.
    """
    return bt.paint.applicator(
        bt.paint.bell(PATTERN),
        standoff=STANDOFF,
        flow=TARGET_FILM * speed * pitch / TRANSFER_EFFICIENCY,
        transfer_efficiency=TRANSFER_EFFICIENCY,
    )


def strokes(overlap: float, speed: float = GUN_SPEED) -> tuple:
    """A serpentine raster over the panel: laps at `pattern x (1-overlap)`,
    each running the panel's length plus overtravel at both ends.

    The overtravel matters more than it looks. The path rests only at its
    two ends, but the gun still has to turn around, and it slows doing so
    — laying on extra paint. Push the turnaround a full pattern radius
    past the panel and that build-up lands on the floor of the booth
    instead of on the part.
    """
    pitch = PATTERN * (1.0 - overlap)
    half_x = PANEL[0] / 2 + OVERTRAVEL
    half_y = PANEL[1] / 2 + LAP_MARGIN
    # Whole laps at exactly the requested pitch, centered on the panel:
    # covering a little extra beats rounding the pitch, which would blur
    # the very thing the sweep compares.
    gaps = max(1, math.ceil(2 * half_y / pitch))
    ys = [-(gaps * pitch) / 2 + pitch * k for k in range(gaps + 1)]

    tp = bt.toolpath.builder(frame="part")
    tp.rapid_to((-half_x, ys[0], STANDOFF))
    tp.feed(speed)
    for i, y in enumerate(ys):
        x0, x1 = (-half_x, half_x) if i % 2 == 0 else (half_x, -half_x)
        tp.line_to((x0, y, STANDOFF))
        tp.line_to((x1, y, STANDOFF))
    return tp.build(), pitch, len(ys)


def coat(scene: bt.Scene, overlap: float, speed: float = GUN_SPEED) -> tuple:
    """Bakes one coating cycle and integrates the film it left."""
    toolpath, pitch, laps = strokes(overlap, speed)
    scene.add_toolpath("coat", toolpath)

    # The cycle in PLC vocabulary: open the gun, run the raster, close it.
    # The gun follows a signal rather than the path's own structure, which
    # is the PLC-correct way to say "spraying" — per-stroke triggering
    # (and its lead/lag) is K2.
    sq = scene.sequence("cycle")
    sq.step(
        "spray",
        actions=[bt.seq.set_signal("gun_on"), bt.seq.toolpath("coat")],
        transition=bt.seq.done(),
    )
    sq.step(
        "close",
        actions=[bt.seq.set_signal("gun_on", False)],
        transition=bt.seq.elapsed(0.2),
    )
    timeline = sq.simulate()
    film = timeline.spray_coat(
        "panel",
        applicator_for(pitch, speed),
        gate="gun_on",
        patch_size=PATCH,
        spec=SPEC,
    )
    return timeline, film, pitch, laps


def main() -> None:
    scene, _ = build_scene()

    # --- the applicator, calibrated ------------------------------------
    analytic = applicator_for(PATTERN * 0.4)
    coupon = coupon_profile(analytic)
    measured = bt.paint.from_profile(coupon, standoff=STANDOFF, seconds=3.0)
    delivered = analytic["flow"] * analytic["transfer_efficiency"]
    print(
        f"coupon: {len(coupon.splitlines()) - 1} samples over "
        f"{PATTERN * 500:.0f} mm; recovered "
        f"{measured.deposition_rate * 1e6 * 60:.2f} cc/min to the plane "
        f"(bell delivers {delivered * 1e6 * 60:.2f})"
    )

    # --- overlap: the film's uniformity, at a constant target ----------
    print(f"\nlap overlap at {GUN_SPEED * 1e3:.0f} mm/s "
          f"(flow trimmed to hold {TARGET_FILM * 1e6:.0f} um):")
    print("  overlap   pitch  laps   cycle     mean   spread  in spec  holidays")
    best = None
    for overlap in (0.2, 0.3, 0.4, 0.5, 0.6, 0.7):
        timeline, film, pitch, laps = coat(scene, overlap)
        print(
            f"  {overlap:6.0%}  {pitch * 1e3:4.0f} mm  {laps:4d}  "
            f"{timeline.duration:6.1f}s  {film.mean * 1e6:4.1f} um  "
            f"{film.sigma / film.mean:6.1%}  {film.in_spec_ratio:7.1%}  "
            f"{film.uncoated_area * 1e4:5.1f} cm2"
        )
        # Best = most in spec, then tightest spread.
        key = (film.in_spec_ratio, -film.sigma / film.mean)
        if best is None or key > best[3]:
            best = (timeline, film, overlap, key)

    # --- speed: what the robot can actually hold -----------------------
    print("\ncommanded gun speed (60% overlap), against what the arm holds:")
    print("     speed   cycle  feed held     mean  in spec")
    for speed in (0.10, 0.15, 0.20, 0.30):
        timeline, film, _, _ = coat(scene, 0.6, speed)
        held = timeline.feed_report("coat").hold_ratio
        print(
            f"  {speed * 1e3:3.0f} mm/s  {timeline.duration:6.1f}s  "
            f"{held:8.1%}  {film.mean * 1e6:5.1f} um  {film.in_spec_ratio:7.1%}"
        )
    print(
        "  ^ the film rides the *baked* trajectory: at 300 mm/s this arm\n"
        "    holds a fifth of the commanded speed, so the panel comes out\n"
        "    four times over spec — and the cycle gets slower, not faster."
    )

    # --- the film map ---------------------------------------------------
    timeline, film, overlap, _ = best
    print(f"\nbest of the sweep: {overlap:.0%} overlap")
    print(f"  {film!r}")
    print(
        f"  paint {film.sprayed_volume * 1e6:.1f} cc sprayed, "
        f"{film.deposited_volume * 1e6:.1f} cc on the panel "
        f"(effective transfer {film.effective_transfer_efficiency:.1%} — most of\n"
        f"  it goes past a panel this small next to a {PATTERN * 1e3:.0f} mm pattern; "
        f"per-stroke triggering is what recovers it)"
    )
    print(
        f"  addressed {film.total_area * 1e4:.1f} cm2 of "
        f"{film.surface_area * 1e4:.1f} cm2 of skin, at "
        f"{film.patch_size * 1e3:.0f} mm patches ({film.patch_count} of them)"
    )
    if film.too_close_time > 0.0:
        print(f"  WARNING: {film.too_close_time:.2f}s inside the model's "
              f"validity floor — check the standoff")

    # OBJ + MTL carries the film as face colors, banded onto a sequential
    # ramp: pale where it is thin, dark where it is heavy, and the bare
    # substrate in a dark neutral so a holiday cannot read as "a bit
    # thin". The studio and the USD export both read colors back from
    # this format.
    film_obj = Path("painting_panel_film.obj").resolve()
    film.save_obj(film_obj)
    print(f"  film map: {film_obj.name} (+ .mtl)")

    if "--studio" in sys.argv:
        # Presentation only: the panel takes the paint's colour as the film
        # builds up, stroke by stroke, and the jet's cone and footprint
        # ring show where the pattern lands. Collision and planning still
        # see the original panel; the numbers above came from `spray_coat`.
        pitch = PATTERN * (1.0 - overlap)
        timeline = scene.animate_paint(
            timeline, "panel", applicator_for(pitch), gate="gun_on", spec=SPEC,
            trigger_signal="spraying", paint_color=PAINT_COLOR,
        )
        print(
            "\nstudio:\n"
            "  - press play: the panel takes the paint's colour as the film builds\n"
            "    up, lap by lap; the key at the top right reads it in microns\n"
            "  - the cone at the bell is the jet, its ring on the panel the\n"
            "    pattern's footprint at the calibrated standoff\n"
            "  - the `spraying` lane in the dock is the effective trigger —\n"
            "    enable AND program — which here means the raster's feed moves"
        )
        bt.studio(scene)


if __name__ == "__main__":
    main()
