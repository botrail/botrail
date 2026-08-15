"""The painting example, asserted the way a paint engineer would assert it.

`examples/painting_demo.py` is the K0 cell of design/design-painting.md: a
flat panel coated by a rotary bell in a serpentine raster, with the film
integrated off the baked cycle. This pins the properties that make it
*painting* rather than motion:

* paint is conserved — what the gun delivers to its reference plane is
  what turns up as film, so the microns mean something,
* a coupon measured off the gun recovers the same applicator as the
  analytic one it came from, which is what makes calibration the honest
  entry point rather than a decoration,
* the film is deterministic, and the gate signal is what decides when the
  gun is spraying,
* statistics run over the surface the gun addressed — the panel's back
  face and rim are not holidays,
* lap overlap buys uniformity, and it does not buy it monotonically,
* the film rides the *baked* trajectory: commanding a speed the arm
  cannot hold puts the panel out of spec, which is the coupling a
  constant-speed film calculator cannot show.

The cell is built from a catalog product (a Mitsubishi Electric RV-5AS-D)
carrying a hand-authored bell, so these tests need the catalog package —
cached locally or fetched once. Where it is unreachable they skip rather
than fail; the deposition engine's own coverage lives in the Rust suite
(`crates/botrail-scene/src/coat.rs`).
"""

import math
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

import painting_demo as demo  # noqa: E402

import botrail as bt  # noqa: E402


@pytest.fixture(scope="module")
def scene():
    try:
        built, _ = demo.build_scene()
    except Exception as err:  # noqa: BLE001 - any fetch/parse failure skips
        pytest.skip(f"catalog unavailable: {err}")
    return built


@pytest.fixture(scope="module")
def coated(scene):
    """One reference cycle at the demo's own settings."""
    return demo.coat(scene, 0.6)


def test_every_lap_of_the_raster_solves(scene):
    """A bell is rotationally symmetric, so the spin about the tool axis
    is free and the 5-DOF solver gets its full room — the raster solves
    greedily, with no global spin pass needed."""
    demo.strokes(0.6)
    scene.add_toolpath("coat", demo.strokes(0.6)[0])
    report = scene.check_toolpath("coat")
    assert report.ok, f"{report!r} {report.issues[:3]}"


def test_the_paint_is_conserved(coated):
    """The invariant the whole model rests on: film is the applicator's
    delivery, projected. What lands on the panel plus what missed it has
    to add up to what the gun sprayed, and the panel's share cannot
    exceed the transfer efficiency."""
    _, film, _, _ = coated
    assert film.deposited_volume > 0.0
    assert film.deposited_volume < film.sprayed_volume * demo.TRANSFER_EFFICIENCY
    assert film.effective_transfer_efficiency == pytest.approx(
        film.deposited_volume / film.sprayed_volume
    )
    # Mean film is over the *addressed* surface, while the deposited
    # volume counts every drop that landed on the target — including the
    # trickle on the rim, which the incidence mask keeps out of the
    # statistics. So the volume is the larger of the two, slightly.
    spread_over_addressed = film.deposited_volume / film.total_area
    assert film.mean <= spread_over_addressed
    assert film.mean == pytest.approx(spread_over_addressed, rel=0.02)


def test_a_coupon_recovers_the_gun_it_came_from(scene):
    """Calibration is the honest entry point: the shape *and* the delivery
    rate come off the measurement, so nobody has to guess a flow. Round-
    tripping the analytic bell through a coupon has to land back on it —
    within the trapezoid's error over thirteen samples."""
    analytic = demo.applicator_for(demo.PATTERN * 0.4)
    coupon = demo.coupon_profile(analytic)
    measured = bt.paint.from_profile(coupon, standoff=demo.STANDOFF, seconds=3.0)
    delivered = analytic["flow"] * analytic["transfer_efficiency"]
    assert measured.deposition_rate == pytest.approx(delivered, rel=0.02)
    assert measured.standoff == demo.STANDOFF
    # And the applicator built from it needs no flow figure supplied.
    from_coupon = bt.paint.applicator(
        measured, transfer_efficiency=demo.TRANSFER_EFFICIENCY
    )
    assert from_coupon["flow"] == pytest.approx(analytic["flow"], rel=0.02)


def test_the_measured_pattern_paints_like_the_analytic_one(scene):
    """The two entry points are the same gun: a coupon generated from the
    analytic bell must lay down the same film as the bell did."""
    analytic = demo.applicator_for(demo.PATTERN * 0.4)
    measured = bt.paint.from_profile(
        demo.coupon_profile(analytic), standoff=demo.STANDOFF, seconds=3.0
    )
    scene.add_toolpath("coat", demo.strokes(0.6)[0])
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

    def film_of(app):
        return timeline.spray_coat(
            "panel", app, gate="gun_on", patch_size=demo.PATCH, spec=demo.SPEC
        )

    a = film_of(analytic)
    b = film_of(
        bt.paint.applicator(measured, transfer_efficiency=demo.TRANSFER_EFFICIENCY)
    )
    assert b.mean == pytest.approx(a.mean, rel=0.03)
    assert b.max == pytest.approx(a.max, rel=0.03)


def test_the_film_is_deterministic(coated, scene):
    """Same cell, same film, bit for bit — the property that lets a film
    map be a regression test at all."""
    timeline, film, pitch, _ = coated
    again = timeline.spray_coat(
        "panel",
        demo.applicator_for(pitch),
        gate="gun_on",
        patch_size=demo.PATCH,
        spec=demo.SPEC,
    )
    assert again.thickness == film.thickness


def test_the_gate_signal_decides_when_the_gun_sprays(coated):
    """Two triggers, both needed. The PLC's enable (`gun_on`) is set in
    the same step the raster starts, so it is high through the approach
    the rollout plans in from the taught stance and through the rapids;
    the program's own trigger — the feed strokes — is not. Paint is
    charged for the intersection: less than the signal's high time by
    exactly the approach and the rapids, so the way in never sprays the
    part. Ungated, the enable is taken as always on, but the feed strokes
    still decide."""
    timeline, film, pitch, _ = coated
    high = timeline.signal("gun_on").high_total()
    feed = sum(b - a for a, b, _ in timeline.process_spans())
    assert film.gun_on_time == pytest.approx(feed, abs=0.05)
    assert film.gun_on_time < high < timeline.duration
    ungated = timeline.spray_coat(
        "panel", demo.applicator_for(pitch), patch_size=demo.PATCH
    )
    assert ungated.gun_on_time == pytest.approx(film.gun_on_time, abs=0.05)
    assert ungated.sprayed_volume == pytest.approx(film.sprayed_volume)
    assert ungated.deposited_volume == pytest.approx(film.deposited_volume)


def test_the_back_and_rim_of_the_panel_are_not_holidays(coated):
    """Statistics run over the surface the gun addressed. A panel sprayed
    from above is a solid: its underside never faces the gun, and its rim
    only ever does at a grazing angle the coupon says nothing about.
    Averaging either in would drown the film map."""
    _, film, _, _ = coated
    top = demo.PANEL[0] * demo.PANEL[1]
    assert film.total_area == pytest.approx(top, rel=1e-6)
    assert film.surface_area > 2 * top
    assert film.uncoated_area == 0.0


def test_lap_overlap_buys_uniformity_and_not_monotonically(scene):
    """The headline of the sweep. Holding the target film constant (pitch
    changes, flow follows) isolates uniformity, and it improves with
    overlap — but 40% edges out 50%, because laps do not sum monotonically
    at every pitch. That is the sort of thing a rule of thumb gets wrong.
    """
    spread = {}
    for overlap in (0.2, 0.4, 0.5, 0.7):
        _, film, _, _ = demo.coat(scene, overlap)
        spread[overlap] = film.sigma / film.mean
        # The flow trim holds the average put, so the sweep really is
        # comparing uniformity and not level.
        assert film.mean == pytest.approx(demo.TARGET_FILM, rel=0.25)
    assert spread[0.7] < spread[0.4] < spread[0.2]
    assert spread[0.4] < spread[0.5]


def test_the_film_rides_the_baked_trajectory(scene):
    """The claim a constant-speed film calculator cannot make. Command a
    paint robot's 300 mm/s on this cobot: it holds a fraction of it, the
    cycle gets *slower*, and the extra dwell puts the panel far over
    spec."""
    slow_tl, slow_film, _, _ = demo.coat(scene, 0.6, 0.10)
    fast_tl, fast_film, _, _ = demo.coat(scene, 0.6, 0.30)
    slow_held = slow_tl.feed_report("coat").hold_ratio
    fast_held = fast_tl.feed_report("coat").hold_ratio
    assert slow_held > 0.95
    assert fast_held < 0.35
    # Commanding three times the speed made the cycle longer, not shorter.
    assert fast_tl.duration > slow_tl.duration
    # And the film went with the speed actually achieved, not the
    # commanded one: over spec, by roughly the ratio of the two.
    assert slow_film.in_spec_ratio > 0.95
    assert fast_film.in_spec_ratio == 0.0
    assert fast_film.mean > 3 * slow_film.mean


def test_the_film_map_writes_colors_that_survive_the_round_trip(coated, tmp_path):
    """OBJ + MTL is the one format the studio loader and the USD exporter
    both read face colors back from, so the film map has to land there —
    banded, with the bare substrate its own material rather than a step of
    the ramp."""
    _, film, _, _ = coated
    obj = tmp_path / "film.obj"
    film.save_obj(obj)
    mtl = obj.with_suffix(".mtl")
    assert obj.exists() and mtl.exists()
    text = obj.read_text()
    assert "mtllib film.mtl" in text
    assert text.count("usemtl") >= 2
    # Banded onto the ramp, not one material per patch.
    materials = mtl.read_text().count("newmtl")
    assert 2 <= materials <= 14, f"{materials} materials"
    assert len(film.thickness) == film.patch_count


def test_a_crowded_gun_is_reported_not_swallowed(scene):
    """Inside a fifth of the reference standoff the inverse square is
    outside what the coupon measured, so nothing is deposited there. That
    has to show up as a number: a silent zero would read as "this stretch
    laid down nothing"."""
    scene.add_toolpath("coat", demo.strokes(0.6)[0])
    sq = scene.sequence("cycle")
    sq.step(
        "spray",
        actions=[bt.seq.set_signal("gun_on"), bt.seq.toolpath("coat")],
        transition=bt.seq.done(),
    )
    timeline = sq.simulate()
    # An applicator calibrated for a much longer standoff than the cell
    # was taught at: the gun is now crowding the panel.
    crowding = bt.paint.applicator(
        bt.paint.bell(demo.PATTERN),
        standoff=demo.STANDOFF * 8,
        flow=1e-6,
        transfer_efficiency=demo.TRANSFER_EFFICIENCY,
    )
    film = timeline.spray_coat("panel", crowding, gate="gun_on", patch_size=0.01)
    assert film.too_close_time > 0.0


def test_a_bad_applicator_is_refused_with_a_reason():
    with pytest.raises(ValueError, match="transfer_efficiency"):
        bt.paint.applicator(bt.paint.bell(0.2), standoff=0.25, flow=1e-6,
                            transfer_efficiency=1.5)
    with pytest.raises(ValueError, match="standoff"):
        bt.paint.applicator(bt.paint.bell(0.2), flow=1e-6)
    with pytest.raises(ValueError, match="flow"):
        bt.paint.applicator(bt.paint.bell(0.2), standoff=0.25)
    with pytest.raises(ValueError, match="ascending"):
        bt.paint.from_profile([(0.0, 1.0), (0.0, 2.0)], standoff=0.25, seconds=1.0)


def test_the_target_and_gate_must_exist(coated):
    timeline, _, pitch, _ = coated
    app = demo.applicator_for(pitch)
    with pytest.raises(ValueError, match="unknown obstacle"):
        timeline.spray_coat("nope", app)
    with pytest.raises(ValueError, match="no signal"):
        timeline.spray_coat("panel", app, gate="nope")
    with pytest.raises(ValueError, match="spec"):
        timeline.spray_coat("panel", app, spec=(30e-6, 20e-6))


def test_the_raster_covers_the_panel_with_whole_laps(scene):
    """The lap set keeps the requested pitch exactly and covers a little
    extra, rather than rounding the pitch to fit — rounding would blur the
    very thing the overlap sweep compares."""
    for overlap in (0.3, 0.6):
        _, pitch, laps = demo.strokes(overlap)
        assert pitch == pytest.approx(demo.PATTERN * (1 - overlap))
        span = (laps - 1) * pitch
        assert span >= demo.PANEL[1] + 2 * demo.LAP_MARGIN - 1e-9
        assert span < demo.PANEL[1] + 2 * demo.LAP_MARGIN + pitch
    # And the turnarounds sit far enough past the panel that the extra
    # paint from slowing down lands off the part.
    assert demo.OVERTRAVEL > demo.PATTERN / 2


def test_the_jet_points_from_the_tip_at_the_part(scene, coated):
    """The spray effect's direction convention, pinned where it can be
    measured: paint travels along the TCP's -Z, so the exported beam —
    a cylinder centred on its own origin — sits exactly half its length
    *toward the part*, never back up into the gun. (The studio draws the
    same convention as a cone with its apex at the tip; the screenshot is
    its only check, which is why the exported twin is asserted here.)"""
    import re

    timeline, _, pitch, _ = coated
    timeline = scene.animate_paint(
        timeline, "panel", demo.applicator_for(pitch), gate="gun_on",
        spec=demo.SPEC, trigger_signal="spraying", stages=4, patch_size=0.02,
    )
    out = Path(__file__).parent / "_jet.usda"
    try:
        assert timeline.export_usd(str(out)) == []
        text = out.read_text()
        body = text[text.index('def Cylinder "jet"'):]
        samples = re.findall(
            r"([\d.]+): \(([-\d.e]+), ([-\d.e]+), ([-\d.e]+)\)",
            re.search(r"xformOp:translate\.timeSamples = \{(.*?)\}", body, re.S).group(1),
        )
    finally:
        out.unlink(missing_ok=True)
    assert samples
    lane = timeline.signal("spraying")
    fps = 60.0
    frame = next(int(float(f)) for f, *_ in samples if lane.value_at(int(float(f)) / fps))
    beam = next([float(v) for v in xyz] for f, *xyz in samples if int(float(f)) == frame)
    # The gun is vertical over the panel in this cell, so "toward the
    # part" is straight down by half the cone's length.
    scene.set_joint_positions(timeline.robot_trajectory().sample(frame / fps))
    tip, _ = scene.link_pose(scene.robot.tcp_link)
    assert beam[0] == pytest.approx(tip[0], abs=1e-6)
    assert beam[1] == pytest.approx(tip[1], abs=1e-6)
    assert tip[2] - beam[2] == pytest.approx(demo.STANDOFF / 2, abs=1e-6)


def test_bad_beta_is_refused():
    with pytest.raises(ValueError, match="beta"):
        bt.paint.bell(0.2, beta=0.5)
    with pytest.raises(ValueError, match="beta"):
        bt.paint.fan(0.3, 0.08, beta_across=0.5)
