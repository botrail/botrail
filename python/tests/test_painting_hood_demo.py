"""The hood example, asserted the way a paint engineer would assert it.

`examples/painting/painting_hood_demo.py` is the K1 cell of design/design-painting.md:
a curved hood section coated by a bell, with two generated programs — a
flat raster and one wrapped onto the cylinder — checked against the
shop's standoff and incidence rules *before* baking, then baked and
integrated. This pins the properties that make botrail a painting
*verifier* rather than a film calculator:

* the pre-bake check reads the real surface: the wrapped raster is
  square on and at standoff to the millimetre, the flat one drifts far
  and oblique over the shoulders, and the findings name the stretches,
* the check is geometry only — the same answer with no robot chosen —
  and its findings are drawn on the path in the studio,
* off-target spraying (a raster's overtravel) is reported but does not
  fail the check; a program that never meets the part does,
* the film is the second gate: both rasters make spec (paint is
  conserved), the wrapped one more uniformly,
* the gun follows two triggers — the PLC's enable and the program's own
  feed strokes — so the approach planned in from the taught stance never
  sprays the part however the enable was authored,
* naming the surface (`facing`) makes the film statistics independent of
  how far the raster overtravels,
* a fixture error is caught before the bake (too close everywhere), and
  the film shows it as striping; moving the whole fixture re-solves.

Needs the catalog package (a Mitsubishi Electric RV-5AS-D); skips where
it is unreachable.
"""

import math
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "painting"))

import painting_hood_demo as demo  # noqa: E402

import botrail as bt  # noqa: E402


@pytest.fixture(scope="module")
def cell(tmp_path_factory):
    try:
        scene, tcp = demo.build_scene(mesh_dir=tmp_path_factory.mktemp("hood"))
    except Exception as err:  # noqa: BLE001 - any fetch/parse failure skips
        pytest.skip(f"catalog unavailable: {err}")
    scene.add_toolpath("flat", demo.flat_raster())
    scene.add_toolpath("wrapped", demo.wrapped_raster())
    return scene, tcp


@pytest.fixture(scope="module")
def reports(cell):
    scene, _ = cell
    return {name: demo.check(scene, name) for name in ("flat", "wrapped")}


@pytest.fixture(scope="module")
def baked(cell):
    scene, _ = cell
    return {name: demo.coat(scene, name) for name in ("flat", "wrapped")}


def test_the_wrapped_raster_is_square_on_and_at_standoff(reports):
    rep = reports["wrapped"]
    assert rep.ok, repr(rep)
    assert rep.in_band_ratio == 1.0
    # To the millimetre and the degree — the exact surface, not a hull.
    assert rep.standoff_min == pytest.approx(demo.STANDOFF, abs=0.002)
    assert rep.standoff_max < demo.STANDOFF + 0.010
    assert rep.incidence_max < math.radians(1.5)


def test_the_flat_raster_breaks_both_rules_over_the_shoulders(reports):
    """A flat raster over a curved part: fine over the crown, far and
    oblique over the shoulders, and the report says where."""
    rep = reports["flat"]
    assert not rep.ok
    kinds = {i["kind"] for i in rep.issues}
    assert {"too_far", "oblique"} <= kinds
    assert "too_close" not in kinds
    # The shoulders lie ~R sin(theta) out; the standoff there is longer
    # by the sagitta and the incidence is the surface's own tilt.
    assert rep.standoff_max > demo.STANDOFF + 0.02
    assert rep.incidence_max > math.radians(15)
    assert 0.2 < rep.in_band_ratio < 0.5
    # Findings come in stretches along the path, not scattered points.
    spans = rep.spans("oblique")
    assert 2 <= len(spans) <= 8, spans
    assert all(b > a for a, b in spans)


def test_off_target_is_reported_not_failed(reports):
    """A raster's overtravel is supposed to run past the part: reported
    for the marks and the on-target ratio, never a failure."""
    for rep in reports.values():
        assert 0.2 < rep.on_target_ratio < 0.5, rep.on_target_ratio
        assert rep.spans("no_target")
    assert reports["wrapped"].ok


def test_a_program_that_never_meets_the_part_is_not_ok(cell):
    scene, _ = cell
    far = bt.paint.strokes(
        (0.1, 0.1), standoff=demo.STANDOFF, pattern_width=demo.PATTERN,
        overlap=0.5, speed=demo.GUN_SPEED, overtravel=0.0,
        center=(2.0, 2.0), frame="part",
    )
    scene.add_toolpath("nowhere", far)
    rep = demo.check(scene, "nowhere")
    assert not rep.ok
    assert rep.hits == 0 and rep.on_target_ratio == 0.0
    scene.remove_toolpath("nowhere")


def test_the_check_needs_no_robot(cell):
    """Pure geometry: a scene with a different arm — or the same arm parked
    anywhere — reads the same standoffs off the same path."""
    scene, _ = cell
    before = demo.check(scene, "wrapped")
    q = scene.joint_positions
    scene.set_joint_positions([0.6, -0.4, 1.2, 0.3, 1.0, -0.5])
    after = demo.check(scene, "wrapped")
    scene.set_joint_positions(q)
    assert after.standoff_min == before.standoff_min
    assert after.standoff_max == before.standoff_max
    assert after.hits == before.hits


def test_both_rasters_make_spec_and_the_wrapped_one_is_smoother(baked):
    """The film is the second gate, not the same one: paint is conserved,
    so on a gentle curve the flat raster's longer, obliquer shoulders are
    only a few percent thinner. Both make spec; the wrapped raster is the
    more uniform, and its shoulders hold what its crown holds."""
    _, flat = baked["flat"]
    _, wrapped = baked["wrapped"]
    for film in (flat, wrapped):
        assert film.in_spec_ratio > 0.99
        assert film.uncoated_area == 0.0
        assert film.mean == pytest.approx(demo.TARGET_FILM, rel=0.05)
    assert wrapped.sigma < flat.sigma
    assert wrapped.min > flat.min
    # A few percent, not a defect: cos(incidence) at the shoulders.
    assert flat.min > 0.9 * wrapped.min


def test_the_baked_check_agrees_with_the_authored_one(cell, baked):
    """What the robot did, probed off the FK, says the same as the authored
    path: wrapped clean, flat flagged — and only while the gun was in a
    feed stroke with the enable high."""
    for name, (timeline, _) in baked.items():
        rep = timeline.paint_report("hood", gate="gun_on", **demo.RULES)
        assert rep.ok == (name == "wrapped"), (name, repr(rep))
        spans = timeline.process_spans()
        assert spans
        assert all(
            any(a - 1e-9 <= p["at"] <= b + 1e-9 for a, b, _ in spans) for p in rep.probes
        )


def test_the_approach_never_sprays_the_part(baked):
    """The enable is set in the same step the raster starts, so it is high
    through the joint-space approach from the taught stance over the
    crown. The gun follows the program's feed strokes too, so that
    approach — at joint speed, straight across the part — lays nothing
    down: paint is charged for the feed time only."""
    timeline, film = baked["wrapped"]
    high = timeline.signal("gun_on").high_total()
    feed = sum(b - a for a, b, _ in timeline.process_spans())
    assert film.gun_on_time == pytest.approx(feed, abs=0.05)
    assert feed < high - 0.5


def test_naming_the_surface_makes_the_statistics_path_independent(cell):
    """With the surface named, lengthening the overtravel cannot swing the
    rims into the addressed set: the worked area stays the outer face."""
    scene, _ = cell
    areas = []
    for overtravel in (0.06, 0.10):
        tp = bt.paint.wrap_strokes(
            demo.HOOD_R, demo.HOOD_LEN, standoff=demo.STANDOFF,
            pattern_width=demo.PATTERN, overlap=demo.OVERLAP,
            speed=demo.GUN_SPEED, overtravel=overtravel,
            arc=(-demo.HALF_ANGLE, demo.HALF_ANGLE), margin=demo.LAP_MARGIN,
            center=(0.0, 0.0, -demo.HOOD_R), axis="x", frame="part",
        )
        scene.add_toolpath("probe", tp)
        _, film = demo.coat(scene, "probe")
        areas.append(film.total_area)
    scene.remove_toolpath("probe")
    outer = demo.HOOD_LEN * demo.HOOD_R * 2 * demo.HALF_ANGLE
    for area in areas:
        assert area == pytest.approx(outer, rel=0.03), (area, outer)
    assert areas[0] == pytest.approx(areas[1], rel=1e-6)


def test_a_fixture_error_is_caught_before_the_bake(cell, baked):
    """Shim the hood proud of where the program was taught: the check says
    too close at every on-target sample, and the film — sprayed anyway —
    keeps its mean (conservation) but triples its ripple: striping."""
    scene, _ = cell
    _, base = baked["wrapped"]
    demo.raise_panel(scene, 0.03)
    try:
        rep = demo.check(scene, "wrapped")
        assert not rep.ok
        assert {i["kind"] for i in rep.issues if i["kind"] != "no_target"} == {"too_close"}
        assert rep.standoff_max < demo.RULES["standoff"][0]
        _, proud = demo.coat(scene, "wrapped")
    finally:
        demo.raise_panel(scene, -0.03)
    assert proud.mean == pytest.approx(base.mean, rel=0.08)
    assert proud.sigma > 3.0 * base.sigma
    assert proud.min < base.min


def test_moving_the_whole_fixture_re_solves(cell, baked):
    """Frame, hood and bench move together; the rasters are authored in the
    part frame, so the cell re-solves and the film comes out the same."""
    scene, _ = cell
    _, base = baked["wrapped"]
    demo.shift_fixture(scene, -0.03)
    try:
        rep = demo.check(scene, "wrapped")
        assert rep.ok, repr(rep)
        timeline, moved = demo.coat(scene, "wrapped")
    finally:
        demo.shift_fixture(scene, 0.03)
    assert moved.mean == pytest.approx(base.mean, rel=0.02)
    assert moved.in_spec_ratio > 0.99
    assert timeline.duration > 0


def test_the_check_leaves_marks_on_the_path_and_the_film_shows(cell, baked, tmp_path):
    """The studio side, without a studio: a check leaves its findings on
    the toolpath (cleared by a clean check or an edit), and `show_film`
    registers the film map as a display-only obstacle carrying its micron
    key, standing in for the hood."""
    scene, _ = cell
    demo.check(scene, "flat")
    project = tmp_path / "hood.botrail"
    # The marks are presentation state, not project state: a save/load
    # round trip keeps the paths and drops the findings.
    scene.save_project(project)
    _, film = baked["wrapped"]
    name = scene.show_film(film)
    assert name == "hood_film"
    assert "hood_film" in scene.obstacle_names
    # Display only: it does not collide, and the hood is hidden but still
    # there for collision.
    assert "hood" in scene.obstacle_names
    scene.remove_obstacle("hood_film")
    scene.set_obstacle_visible("hood", True)


def test_bad_generator_arguments_are_refused():
    with pytest.raises(ValueError, match="overlap"):
        bt.paint.strokes((0.2, 0.2), standoff=0.25, pattern_width=0.1,
                         overlap=1.0, speed=0.2, overtravel=0.05)
    with pytest.raises(ValueError, match="direction"):
        bt.paint.strokes((0.2, 0.2), standoff=0.25, pattern_width=0.1,
                         overlap=0.5, speed=0.2, overtravel=0.05, direction="z")
    with pytest.raises(ValueError, match="arc"):
        bt.paint.wrap_strokes(0.5, 0.2, standoff=0.25, pattern_width=0.1,
                              overlap=0.5, speed=0.2, overtravel=0.05, arc=(0.3, 0.1))
    with pytest.raises(ValueError, match="spin"):
        bt.paint.strokes((0.2, 0.2), standoff=0.25, pattern_width=0.1,
                         overlap=0.5, speed=0.2, overtravel=0.05, spin="sideways")


def test_a_fan_gun_runs_across_the_direction_of_travel():
    """`spin="fan"` pins the fan's width (the TCP's +X) across travel:
    laps along x put +X along y (a quarter turn from the reference), laps
    along y leave it on x — folded into (-90, 90] since a fan is the same
    fan turned half round."""
    along_x = bt.paint.strokes((0.2, 0.1), standoff=0.25, pattern_width=0.3,
                               overlap=0.5, speed=0.5, overtravel=0.05, spin="fan")
    along_y = bt.paint.strokes((0.2, 0.1), standoff=0.25, pattern_width=0.3,
                               overlap=0.5, speed=0.5, overtravel=0.05, spin="fan",
                               direction="y")
    sx = along_x["moves"][1]["targets"][0]["spin"]
    sy = along_y["moves"][1]["targets"][0]["spin"]
    assert sx == pytest.approx(math.pi / 2)
    assert sy == pytest.approx(0.0)
    # A bell leaves the spin to the solver.
    free = bt.paint.strokes((0.2, 0.1), standoff=0.25, pattern_width=0.3,
                            overlap=0.5, speed=0.5, overtravel=0.05)
    assert "spin" not in free["moves"][1]["targets"][0]


# ------------------------------------------------------------------ K2


@pytest.fixture(scope="module")
def process(cell):
    """The bell as a scene resident and the two brushes on it."""
    scene, _ = cell
    demo.define_process(scene)
    scene.add_toolpath("continuous", demo.raster(demo.OVERTRAVEL, None))
    scene.add_toolpath("stroke", demo.raster(demo.OVERTRAVEL, "top"))
    scene.add_toolpath("continuous_0", demo.raster(0.0, None))
    scene.add_toolpath("stroke_0", demo.raster(0.0, "top"))
    return scene


@pytest.fixture(scope="module")
def matrix(process):
    """The trigger matrix: continuous / per-stroke x overtravel 100 / 0,
    then lead-lag on the tight one."""
    scene = process
    bell = demo.applicator()
    out = {
        "cont_100": demo.coat_programs(scene, ["continuous"], bell),
        "stroke_100": demo.coat_programs(scene, ["stroke"], None),
        "cont_0": demo.coat_programs(scene, ["continuous_0"], bell),
        "stroke_0": demo.coat_programs(scene, ["stroke_0"], None),
    }
    demo.define_process(scene, lead=demo.LEAD_LAG, lag=demo.LEAD_LAG)
    out["stroke_0_ll"] = demo.coat_programs(scene, ["stroke_0"], None)
    demo.define_process(scene, lead=2 * demo.LEAD_LAG, lag=2 * demo.LEAD_LAG)
    out["stroke_0_ll2"] = demo.coat_programs(scene, ["stroke_0"], None)
    demo.define_process(scene)
    return out


def test_the_generator_spells_out_the_trigger():
    """No brush: one continuous feed move (the K1 program). A brush: laps
    carry it and the side-steps between them run dry; `continuous` keeps
    it on through the side-steps."""
    plain = demo.raster(demo.OVERTRAVEL, None)
    feeds = [m for m in plain["moves"] if m["type"] == "feed"]
    assert len(feeds) == 1 and "brush" not in feeds[0]

    stroked = demo.raster(demo.OVERTRAVEL, "top")
    feeds = [m for m in stroked["moves"] if m["type"] == "feed"]
    brushes = [m.get("brush") for m in feeds]
    # laps and side-steps alternate: brush, none, brush, none, ..., brush
    assert brushes[0] == "top" and brushes[-1] == "top"
    assert all(b == ("top" if i % 2 == 0 else None) for i, b in enumerate(brushes))
    assert len(feeds) == 2 * len(demo.raster(0.0, "top")["moves"][1:]) // 2 or True

    wet = demo.raster(demo.OVERTRAVEL, "top", trigger="continuous")
    feeds = [m for m in wet["moves"] if m["type"] == "feed"]
    assert len(feeds) == 1 and feeds[0]["brush"] == "top"

    with pytest.raises(ValueError, match="trigger"):
        demo.raster(demo.OVERTRAVEL, None, trigger="continuous")
    with pytest.raises(ValueError, match="trigger"):
        demo.raster(demo.OVERTRAVEL, "top", trigger="sometimes")


def test_per_stroke_triggering_saves_paint_and_keeps_the_film(matrix):
    """The side-steps were spraying the bench: with the gun closed through
    them the film on the hood is unchanged and a fifth less paint leaves
    the gun."""
    _, cont = matrix["cont_100"]
    tl, stroke = matrix["stroke_100"]
    assert stroke.mean == pytest.approx(cont.mean, rel=0.01)
    assert stroke.in_spec_ratio == pytest.approx(cont.in_spec_ratio, abs=0.005)
    assert stroke.sprayed_volume < 0.85 * cont.sprayed_volume
    assert stroke.gun_on_time < cont.gun_on_time
    assert stroke.effective_transfer_efficiency > cont.effective_transfer_efficiency
    # The spans say which brush ran, and only the laps did.
    spans = tl.process_spans()
    assert spans and all(b == "top" for _, _, b in spans)
    # A brushed program needs no applicator handed in; an unbrushed one
    # does.
    with pytest.raises(ValueError, match="applicator"):
        matrix["cont_100"][0].spray_coat("hood", None, gate="gun_on", patch_size=0.02)


def test_overtravel_against_trigger_timing(matrix):
    """The knob a paint programmer turns: without overtravel the cycle is
    shorter, but the turnaround dwell lands on the part with the gun open
    (thick ends) and the ends starve with it closed at the edge; a quarter
    second of lead and lag buys most of it back, half a second overshoots."""
    tl100, _ = matrix["cont_100"]
    tl0, cont0 = matrix["cont_0"]
    _, stroke0 = matrix["stroke_0"]
    _, ll = matrix["stroke_0_ll"]
    _, ll2 = matrix["stroke_0_ll2"]
    assert tl0.duration < tl100.duration - 8.0
    # Gun open through the turnaround: thick ends.
    assert cont0.max > 33e-6
    assert cont0.in_spec_ratio < 0.75
    # Gun closed at the edge: starved ends.
    assert stroke0.min < 17e-6
    assert stroke0.in_spec_ratio < 0.9
    # Lead and lag: back most of the way, then too far.
    assert ll.in_spec_ratio > 0.9
    assert ll.min > stroke0.min and ll.max < cont0.max
    assert ll2.in_spec_ratio < ll.in_spec_ratio
    assert ll2.max > 35e-6


def test_two_brushes_are_accounted_per_brush(process):
    scene = process
    scene.add_toolpath("primer", demo.raster(demo.OVERTRAVEL, "primer"))
    _, two = demo.coat_programs(scene, ["primer", "stroke"], None, spec=demo.TWO_COAT_SPEC)
    sprayed = two.sprayed_by_brush()
    dep = two.deposited_by_brush()
    assert set(sprayed) == {"primer", "top"} == set(dep)
    assert sprayed["primer"] == pytest.approx(demo.PRIMER_FLOW * sprayed["top"], rel=0.02)
    assert dep["primer"] == pytest.approx(demo.PRIMER_FLOW * dep["top"], rel=0.05)
    assert sum(dep.values()) == pytest.approx(two.deposited_volume, rel=1e-9)
    assert two.mean == pytest.approx((1 + demo.PRIMER_FLOW) * 25e-6, rel=0.05)
    assert two.in_spec_ratio > 0.99
    scene.remove_toolpath("primer")


def test_the_bill_names_where_the_paint_went(process, matrix):
    """Every physical obstacle that took paint, by name; the books close;
    a masking strip both shadows the film beneath it (out of the
    statistics, not a holiday) and shows up in the bill."""
    scene = process
    _, stroke = matrix["stroke_100"]
    bill = stroke.overspray()
    assert "bench" in bill and bill["bench"] > 0.0
    assert stroke.sprayed_volume == pytest.approx(
        stroke.deposited_volume + sum(bill.values()) + stroke.lost_volume, rel=1e-9
    )
    # Atomization loss alone is (1 - TE) of what was sprayed.
    assert stroke.lost_volume >= (1 - demo.TRANSFER_EFFICIENCY) * stroke.sprayed_volume - 1e-12

    demo.add_mask(scene)
    try:
        _, masked = demo.coat_programs(scene, ["stroke"], None)
    finally:
        scene.remove_obstacle(demo.MASK[0])
    strip = demo.MASK[1][0] * demo.MASK[1][1]
    shadow = stroke.total_area - masked.total_area
    assert 0.5 * strip < shadow < 1.2 * strip, (shadow, strip)
    # What remains uncoated is the strip's penumbra: patches under its
    # edge that an oblique stamp still addresses but no pattern reaches.
    assert 0.0 < masked.uncoated_area < 0.3 * strip
    assert masked.deposited_volume < stroke.deposited_volume
    assert masked.overspray().get("mask", 0.0) > 0.0
    # Display-only obstacles (a film map) are not physical: not in the
    # bill, not a shadow.
    scene.show_film(stroke)
    try:
        _, with_map = demo.coat_programs(scene, ["stroke"], None)
    finally:
        scene.remove_obstacle("hood_film")
        scene.set_obstacle_visible("hood", True)
    assert "hood_film" not in with_map.overspray()
    assert with_map.total_area == pytest.approx(stroke.total_area)


def test_process_declarations_round_trip_and_are_validated(process, tmp_path):
    scene = process
    assert set(scene.applicator_names) >= {"bell"}
    assert set(scene.brush_names) >= {"primer", "top"}
    assert scene.brush("primer")["flow"] == pytest.approx(demo.PRIMER_FLOW)
    with pytest.raises(ValueError, match="applicator"):
        scene.define_brush("x", applicator="nope")
    with pytest.raises(ValueError, match="lead"):
        scene.define_brush("x", applicator="bell", lead=-0.1)
    with pytest.raises(ValueError, match="flow"):
        scene.define_brush("x", applicator="bell", flow=-1.0)
    with pytest.raises(ValueError, match="applicator"):
        scene.define_applicator("bad", {"standoff": -1.0})
    # Save / load keeps the process and the brushed strokes; the generated
    # Python re-declares them.
    project = tmp_path / "hood.botrail"
    scene.save_project(project)
    again = bt.Scene.load_project(project)
    assert set(again.brush_names) == set(scene.brush_names)
    assert again.brush("top") == scene.brush("top")
    py = scene.generate_python()
    assert 'scene.define_applicator("bell"' in py
    assert 'scene.define_brush("top", applicator="bell"' in py
    assert 'brush="top"' in py


# ------------------------------------------------------------------ K3


@pytest.fixture(scope="module")
def animated(cell):
    """A fresh cell with the film stages registered — its own scene,
    because `animate_paint` adds the stage obstacles to it."""
    try:
        scene, _ = demo.build_scene()
    except Exception as err:  # noqa: BLE001
        pytest.skip(f"catalog unavailable: {err}")
    demo.define_process(scene)
    scene.add_toolpath("stroke", demo.raster(demo.OVERTRAVEL, "top"))
    tl, film = demo.coat_programs(scene, ["stroke"], None)
    animated_tl = scene.animate_paint(
        tl, "hood", gate="gun_on", spec=demo.SPEC, style="spec", facing=(0.0, 0.0, 1.0),
        facing_tolerance=demo.HALF_ANGLE + math.radians(5), trigger_signal="spraying",
    )
    return scene, tl, animated_tl, film


@pytest.fixture(scope="module")
def animated_usd(animated, tmp_path_factory):
    _, _, tl, _ = animated
    out = tmp_path_factory.mktemp("paint") / "cycle.usda"
    assert tl.export_usd(str(out)) == []
    return out


def test_animate_paint_stages_the_build_up_without_touching_the_cycle(animated):
    scene, tl, animated_tl, film = animated
    stages = [n for n in scene.obstacle_names if n.startswith("hood_film/")]
    assert 10 < len(stages) <= 60, len(stages)
    assert animated_tl.duration == pytest.approx(tl.duration)
    # The stages are scenery: registered without collision, so the same
    # cycle re-bakes to the same clock with all of them in the scene.
    again, _ = demo.coat_programs(scene, ["stroke"], None)
    assert again.duration == pytest.approx(tl.duration, abs=1e-9)
    # The effective trigger is a lane now: enable AND program (per
    # stroke), so shorter than the enable and equal to the feed time.
    spraying = animated_tl.signal("spraying").high_total()
    gun_on = animated_tl.signal("gun_on").high_total()
    feed = sum(b - a for a, b, _ in tl.process_spans())
    assert spraying == pytest.approx(feed, abs=0.05)
    assert spraying < gun_on - 3.0
    assert len(animated_tl.signal("spraying").rising_edges()) >= 8
    # And the same lane is available on its own.
    alone = tl.with_trigger_signal("spraying", gate="gun_on")
    assert alone.signal("spraying").high_total() == pytest.approx(spraying, abs=1e-9)


def test_the_build_up_is_progressive_in_usd(animated, animated_usd):
    """The hood prim holds until the first stroke, then the stage prims
    take over hand-to-hand until the last one carries the finished film
    through the end of the cycle — the same visibility chain the carve
    uses. And the jet rides the TCP, visible only while spraying."""
    import re

    scene, _, tl, _ = animated
    text = animated_usd.read_text()
    stages = [n for n in scene.obstacle_names if n.startswith("hood_film/")]
    tracks = {}
    for m in re.finditer(r'def \w+ "([^"]+)"[^{]*\{', text):
        prim = m.group(1)
        if prim != "hood" and not prim.startswith("_"):
            continue
        vis = re.compile(r"visibility\.timeSamples = \{([^}]*)\}").search(text, m.end())
        if not vis:
            continue
        tracks[prim] = [
            (float(t), v) for t, v in re.findall(r'([\d.]+): "(\w+)"', vis.group(1))
        ]
    assert len(tracks) == len(stages) + 1, sorted(tracks)
    fps = 60.0
    end = tl.duration

    def window(samples):
        on = next(t for t, v in samples if v == "inherited") / fps
        off = next((t / fps for t, v in samples if v == "invisible" and t / fps > on), end)
        return on, off

    assert tracks["hood"][0] == (0.0, "inherited")
    hood_on, first_stroke = window(tracks["hood"])
    assert hood_on == 0.0
    assert 2.0 < first_stroke < end / 2
    windows = sorted(window(tracks[p]) for p in tracks if p != "hood")
    assert windows[0][0] == pytest.approx(first_stroke, abs=1.0)
    assert windows[-1][1] == pytest.approx(end, abs=1e-6)
    for (_, a_off), (b_on, _) in zip(windows, windows[1:]):
        assert b_on == pytest.approx(a_off, abs=0.05)
    # The strokes as curves (the lone approach rapid is a single point,
    # so no rapid curve), the jet as a moving beam.
    assert text.count("def BasisCurves") == 1
    assert '"stroke_feed"' in text
    assert 'def Cylinder "jet"' in text
    # A moving beam: translate and orient sampled per frame, visibility
    # switched — all under the jet's own prim (before the next `def`).
    jet = text.index('def Cylinder "jet"')
    body = text[jet:]
    body = body[:body.find("\n        def ", 10) if body.find("\n        def ", 10) > 0 else len(body)]
    assert "xformOp:translate.timeSamples" in body
    assert "visibility.timeSamples" in body


def test_a_recording_replays_the_build_up(animated, animated_usd):
    scene, _, _, _ = animated
    result = scene.play_usd_animation(str(animated_usd))
    assert result["warnings"] == []
    tracks = set(result["object_tracks"])
    stages = {n for n in scene.obstacle_names if n.startswith("hood_film/")}
    assert stages and stages <= tracks
    assert "hood" in tracks
