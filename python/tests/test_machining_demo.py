"""The machining example, asserted the way a cell owner would assert it.

`examples/machining_demo.py` is the C0/C1 cell of
design/design-machining.md: a live stock plate trimmed (builder toolpath:
lines + corner arcs) and pocketed (G-code import) at commanded feed,
inside a clamp -> spindle -> cut -> unclamp sequence. This pins the
properties that make it *machining* rather than motion:

* the whole path is reachable and stays on one IK branch (face check ok),
* the commanded feed owns the clock — cutting time equals path length
  over feed, joints permitting — and the bake says so deterministically,
* the stock is a real collision body: only the allowed cutter-plate pair
  may touch, only while cutting — a rapid through the stock fails,
* the toolpath is authored against the fixture frame, so moving the
  fixture re-solves the same program instead of breaking taught points,
* dangerous G-codes are refused with line numbers, harmless spindle words
  are warnings,
* the artifacts round-trip: project save/load keeps the toolpaths and the
  contact exemption, USD carries the paths as BasisCurves, URScript
  renders the cuts as movep chains at the feed.

Self-contained (simple_arm + an authored spindle URDF): no catalog, no
network.
"""

import math
import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES))

import machining_demo as demo  # noqa: E402

import botrail as bt  # noqa: E402


@pytest.fixture(scope="module")
def cell():
    scene, tcp_link = demo.build_cell()
    contour = demo.contour_toolpath()
    pocket = bt.toolpath.from_gcode(demo.pocket_gcode(), frame="fixture")
    return scene, tcp_link, contour, pocket


@pytest.fixture(scope="module")
def baked(cell):
    scene, _, _, _ = cell
    return {
        name: scene.plan_toolpath(name, rapid_speed=demo.RAPID_CAP)
        for name in ("contour", "pocket")
    }


def test_every_sample_of_both_paths_solves(cell):
    scene, _, _, _ = cell
    for name in ("contour", "pocket"):
        report = scene.check_toolpath(name)
        assert report.ok, f"{name}: {report!r} {report.issues[:3]}"
        assert bool(report)
        assert report.issues == []


def test_the_commanded_feed_owns_the_clock(cell, baked):
    """Cutting time is the process's number. The feed floors make
    `path length / feed` a hard lower bound on the feed moves' spans;
    the few percent above it is the acceleration limit slowing corners
    (plunge -> rim is a right angle) — not the joints stealing the feed."""
    _, _, contour, pocket = cell
    for name, tp in (("contour", contour), ("pocket", pocket)):
        traj = baked[name]
        commanded = demo.cut_seconds(tp)
        ends = [0.0] + list(traj.segment_ends)
        feed_time = sum(
            ends[i + 1] - ends[i]
            for i, move in enumerate(tp["moves"])
            if move["type"] == "feed"
        )
        assert commanded <= feed_time <= commanded * 1.05, (
            f"{name}: feed moves took {feed_time:.2f}s, commanded {commanded:.2f}s"
        )


# Baked on the pinned dependency set. The tolerance absorbs libm-level
# drift between machines, not behaviour changes — a re-taught reference
# pose or a solver change moves these by far more.
GOLDEN = {"contour": 28.21, "pocket": 27.25}


def test_cycle_times_hold_their_golden(baked):
    for name, expected in GOLDEN.items():
        assert baked[name].duration == pytest.approx(expected, abs=0.25), name


def test_baking_is_deterministic(cell):
    scene, _, _, _ = cell
    a = scene.plan_toolpath("contour", rapid_speed=demo.RAPID_CAP)
    b = scene.plan_toolpath("contour", rapid_speed=demo.RAPID_CAP)
    assert a.times == b.times
    assert a.positions == b.positions


def test_the_tool_stays_vertical_and_on_depth_mid_cut(cell, baked):
    """Mid-trim the cutter tip must sit on the programmed depth with its
    axis vertical — the 5-DOF task holds even though no full pose was
    ever authored."""
    scene, tcp_link, _, _ = cell
    traj = baked["contour"]
    fixture_z = scene.frame("fixture")[0][2]
    ends = traj.segment_ends
    # Sample inside the long rim cut (after the plunge, before retract).
    for f in (0.35, 0.5, 0.65):
        t = ends[1] + (ends[-2] - ends[1]) * f
        scene.set_joint_positions(traj.sample(t))
        (x, y, z), quat = scene.link_pose(tcp_link)
        assert z - fixture_z == pytest.approx(demo.CUT_DEPTH, abs=5e-4)
        # Tool axis = TCP +Z rotated by quat; must align with world +Z.
        qx, qy, qz, qw = quat
        axis_z = 1.0 - 2.0 * (qx * qx + qy * qy)
        assert axis_z > math.cos(2e-3), f"axis tilted at t={t:.2f}s"
    scene.set_joint_positions(demo.REF_Q)


def test_moving_the_fixture_re_solves_the_same_program(cell, baked):
    scene, _, _, _ = cell
    try:
        demo.shift_fixture(scene, 0.03)
        report = scene.check_toolpath("contour")
        assert report.ok, report.issues[:3]
        shifted = scene.plan_toolpath("contour", rapid_speed=demo.RAPID_CAP)
        assert shifted.duration == pytest.approx(
            baked["contour"].duration, abs=0.5
        )
    finally:
        demo.shift_fixture(scene, -0.03)


def test_an_impossible_fixture_is_diagnosed_not_crashed(cell):
    """Half a meter out, the far half of the contour leaves the arm's
    reach: the face check reports located failures and keeps walking; the
    bake refuses with the failing sample in the message."""
    scene, _, _, _ = cell
    home = scene.frame("fixture")
    try:
        scene.add_frame(
            "fixture", position=(home[0][0] + 0.5, home[0][1], home[0][2])
        )
        report = scene.check_toolpath("contour")
        assert not report.ok
        assert 0 < len(report.issues)
        assert any(i["kind"] == "unreachable" for i in report.issues)
        with pytest.raises(ValueError, match="sample"):
            scene.plan_toolpath("contour")
    finally:
        scene.add_frame("fixture", position=home[0])


def test_dangerous_gcode_is_refused_with_line_numbers():
    for src, needle in (
        ("G1 X5 F100\nG41 D1\n", "G41"),
        ("T2 M6\n", "tool"),
        ("G95\n", "G95"),
        ("G4 P1\n", "dwell"),
    ):
        with pytest.raises(ValueError, match="line") as err:
            bt.toolpath.from_gcode(src)
        assert needle.lower() in str(err.value).lower()


def test_spindle_words_are_warnings_not_errors(cell):
    _, _, _, pocket = cell
    assert len(pocket.warnings) == 3
    assert any("S" in w for w in pocket.warnings)
    assert any("M3" in w for w in pocket.warnings)


def test_toolpaths_survive_the_project_round_trip(cell, tmp_path):
    scene, _, _, _ = cell
    path = tmp_path / "machining.botrail"
    scene.save_project(str(path))
    reloaded = bt.Scene.load_project(str(path))
    assert sorted(reloaded.toolpath_names) == ["contour", "pocket", "rim"]
    # The reloaded check passes only if the cutter-plate exemption
    # survived the round trip — the stock is a live collision body.
    assert reloaded.check_toolpath("contour").ok
    code = reloaded.generate_python()
    assert "bt.toolpath.builder(frame=\"fixture\")" in code
    assert "scene.add_toolpath(\"contour\", _tp.build())" in code
    assert 'scene.allow_link_obstacle_contact("cutter", "plate")' in code
    assert "bt.seq.toolpath(\"contour\")" in code


# ------------------------------------------------------------ C1: the cell

# Baked on the pinned dependency set: clamp 0.5 + spin-up 1.5 + trim
# (approach + cut) + pocket + the 5-axis rim chamfer (the C2 path — its
# spin schedule needs the global optimizer) + spin-down 1.0 + unclamp
# 0.5.
CYCLE_GOLDEN = 108.03


@pytest.fixture(scope="module")
def cycle(cell):
    scene, _, _, _ = cell
    return scene.simulate_sequence("cycle", toolpath_spin="optimize")


def test_the_cycle_holds_its_golden_and_order(cycle):
    assert cycle.duration == pytest.approx(CYCLE_GOLDEN, abs=0.25)
    assert [s[0] for s in cycle.step_spans] == [
        "clamp", "spin_up", "trim", "pocket", "chamfer", "spin_down", "unclamp",
    ]


def test_the_spindle_covers_every_cut(cycle):
    """The spindle-run output must be high before the first cut starts and
    stay high past the last cut's end — the interlock a machining cell is
    built around."""
    spans = {name: (t0, t1) for name, t0, t1 in cycle.step_spans}
    on_spans = cycle.signal("spindle_run").high_spans()
    assert len(on_spans) == 1
    on0, on1 = on_spans[0]
    assert on0 <= spans["trim"][0] and on1 >= spans["chamfer"][1]


def test_cutting_needs_the_contact_exemption(cell):
    """The stock is a live collision body: revoke the cutter-plate
    allowance and the plunge is a located collision again."""
    scene, _, _, _ = cell
    scene.disallow_link_obstacle_contact("cutter", "plate")
    try:
        report = scene.check_toolpath("contour")
        assert not report.ok
        assert any(i["kind"] == "collision" for i in report.issues)
    finally:
        scene.allow_link_obstacle_contact("cutter", "plate")


def test_a_rapid_through_the_stock_is_refused(cell):
    """Exemptions apply only while cutting: the same allowed pair fails
    the bake the moment the contact happens during a rapid."""
    scene, _, _, _ = cell
    dragged = (
        bt.toolpath.builder(frame="fixture")
        .rapid_to((-0.05, 0.0, demo.CUT_DEPTH))
        .rapid_to((0.05, 0.0, demo.CUT_DEPTH))
        .build()
    )
    scene.add_toolpath("dragged", dragged)
    try:
        report = scene.check_toolpath("dragged")
        assert not report.ok
        assert any(
            i["kind"] == "collision" and "rapid" in i["detail"]
            for i in report.issues
        )
    finally:
        scene.remove_toolpath("dragged")


def test_urscript_renders_the_cuts_as_movep_at_feed(cycle):
    script = cycle.to_script(
        outputs={"clamped": 0, "spindle_run": 1}, blend_radius=0.002
    )
    assert "movep(" in script
    assert "v=0.015" in script  # trim feed
    assert "v=0.004" in script  # plunge feed
    assert "r=0.002" in script
    assert "set_standard_digital_out(1, True)" in script
    # Each blended chain still comes to rest at its end.
    assert "movel(" in script


def test_timeline_clearance_ignores_the_cutting_contact(cycle):
    """min_clearance over the whole cycle stays a meaningful number — the
    allowed cutter-plate pair is excluded, so the floor is the cutter
    passing the toe clamps, not zero."""
    assert float(cycle.min_clearance()) > 0.002


# ------------------------------------------------- C2: practice-grade parts


def test_greedy_fails_the_rim_and_the_optimizer_solves_it(cell):
    """The C2 acceptance: the rim chamfer's outward lean precesses a full
    turn, and followed greedily the wrist walks into the base-facing edge
    (dozens of located failures). The Descartes-style spin pass solves
    the same path whole — it spends spin early to stay solvable late,
    which no local rule can do."""
    scene, _, _, _ = cell
    greedy = scene.check_toolpath("rim")
    assert not greedy.ok
    assert len(greedy.issues) > 20
    optimized = scene.check_toolpath("rim", spin="optimize")
    assert optimized.ok, optimized.issues[:3]
    # And without the optimizer the *cycle* refuses at the chamfer step,
    # with the failing sample in the message.
    with pytest.raises(ValueError, match="chamfer.*sample"):
        scene.simulate_sequence("cycle")


def test_the_optimized_cycle_is_deterministic(cell, cycle):
    scene, _, _, _ = cell
    again = scene.simulate_sequence("cycle", toolpath_spin="optimize")
    assert cycle.robot_trajectory().times == again.robot_trajectory().times
    assert cycle.robot_trajectory().positions == again.robot_trajectory().positions


def test_feed_reports_name_their_limiting_axis(cycle):
    """The report says where the joints stole feed and which axis owned
    it. The flat cuts hold well; the tilted rim honestly loses a third of
    its feed to the corners."""
    contour = cycle.feed_report("contour")
    rim = cycle.feed_report("rim")
    assert contour.hold_ratio > 0.85
    assert 0.5 < rim.hold_ratio < 0.85
    assert rim.hold_ratio < contour.hold_ratio
    assert len(rim.slow_spans) >= 3
    joints = {span["limiting_joint"] for span in rim.slow_spans}
    assert joints & {"shoulder_pan", "shoulder_lift", "elbow",
                     "wrist_1", "wrist_2", "wrist_3"}
    for span in rim.slow_spans:
        assert span["achieved_feed"] < span["commanded_feed"]
    with pytest.raises(ValueError, match="no toolpath named"):
        cycle.feed_report("nope")


def test_apt_import_carries_the_axis_and_refuses_the_dangerous(cell):
    _, _, _, _ = cell
    rim = bt.toolpath.from_apt(demo.rim_apt(), frame="fixture")
    assert rim.target_count > 30
    # The 5-axis part: tool axes lean off vertical.
    tilted = [
        t["tool_axis"]
        for m in rim["moves"]
        for t in m["targets"]
        if "tool_axis" in t
    ]
    assert tilted and any(abs(a[0]) > 0.4 or abs(a[1]) > 0.4 for a in tilted)
    assert any("SPINDL" in w for w in rim.warnings)
    assert any("LOADTL" in w for w in rim.warnings)
    for src, needle in (
        ("CUTCOM/LEFT\n", "CUTCOM"),
        ("CIRCLE/0,0,0,0,0,1,10\n", "CIRCLE"),
        ("FEDRAT/300\nGOTO/0,0,0\nLOADTL/2\n", "LOADTL"),
    ):
        with pytest.raises(ValueError, match="line") as err:
            bt.toolpath.from_apt("LOADTL/1\n" + src)
        assert needle in str(err.value)


def test_the_fixture_shift_moves_the_feed_report_not_the_program(cell, cycle):
    """The design promise in numbers: shift the fixture and re-simulate —
    the same authored cell re-solves, and what changes is measured (the
    rim's hold ratio), not broken."""
    scene, _, _, _ = cell
    try:
        demo.shift_fixture(scene, 0.03)
        shifted = scene.simulate_sequence("cycle", toolpath_spin="optimize")
        assert shifted.duration == pytest.approx(cycle.duration, abs=1.0)
        assert shifted.feed_report("rim").hold_ratio == pytest.approx(
            cycle.feed_report("rim").hold_ratio, abs=0.1
        )
    finally:
        demo.shift_fixture(scene, -0.03)


# --------------------------------------------------------- C3: the picture


def test_the_carve_matches_the_cut(cycle):
    """The machined part, as numbers: the plate is 140x100x12 = 168 cm3
    exactly, and the three cuts take ~9 cm3 out of it. Conservation is
    exact by construction (counted voxels)."""
    carve = cycle.carve_stock("plate")
    assert carve.initial_volume == pytest.approx(168e-6, rel=0.02)
    assert carve.removed_volume == pytest.approx(9.2e-6, abs=1.0e-6)
    assert carve.initial_volume == pytest.approx(
        carve.removed_volume + carve.remaining_volume, abs=1e-12
    )
    # Greedy meshing keeps the machined part light enough to display.
    assert 0 < carve.triangle_count < 10_000


def test_carving_is_deterministic_and_writes_stl(cycle, tmp_path):
    a = cycle.carve_stock("plate")
    b = cycle.carve_stock("plate")
    assert a.removed_volume == b.removed_volume
    assert a.triangle_count == b.triangle_count
    out = tmp_path / "cut.stl"
    a.save_stl(out)
    # Binary STL: 84-byte header + 50 bytes per triangle.
    assert out.stat().st_size == 84 + 50 * a.triangle_count


def test_the_machined_surfaces_are_classed_in_the_obj(cycle, tmp_path):
    """What makes the removal *readable*: faces the cutter made carry the
    bright machined color, the surviving skin keeps the stock color —
    two materials in the MTL, and both actually used."""
    carve = cycle.carve_stock("plate")
    out = tmp_path / "cut.obj"
    carve.save_obj(out)
    mtl = (tmp_path / "cut.mtl").read_text()
    assert mtl.count("newmtl") == 2, mtl
    assert "Kd 0.92 0.94 0.98" in mtl  # machined finish
    assert "Kd 0.58 0.6 0.63" in mtl  # surviving skin
    obj = out.read_text()
    assert "mtllib cut.mtl" in obj
    # Both classes appear as material runs, and every face is present.
    assert obj.count("usemtl") >= 2
    assert obj.count("\nf ") == carve.triangle_count
    # The file round-trips through the scene loader with its colors.
    scene2 = bt.Scene(bt.Robot.from_urdf(str(EXAMPLES / "simple_arm.urdf")))
    scene2.add_mesh("part", out, position=(0.5, 0.0, 0.2))


def test_the_cut_trace_survives_the_project_round_trip(cell, tmp_path):
    scene, _, _, _ = cell
    code = scene.generate_python()
    assert (
        'scene.add_cut_trace("cutting", signal="spindle_run", '
        'robot="simple_arm", spin_link="cutter")' in code
    ), code
    path = tmp_path / "c3.botrail"
    scene.save_project(str(path))
    reloaded = bt.Scene.load_project(str(path))
    assert "add_cut_trace" in reloaded.generate_python()


def test_traces_do_not_leak_spheres_into_usd(cycle, tmp_path):
    """A weld flash bakes blinking spheres named after it; a cut trace
    must not — the toolpath BasisCurves already carry the picture in
    USD. (The robot's own sphere visuals are not the subject here.)"""
    out = tmp_path / "c3.usda"
    cycle.export_usd(str(out))
    assert "cutting" not in out.read_text()


def test_usd_export_carries_the_toolpaths_as_curves(cell, baked, tmp_path):
    scene, _, _, _ = cell
    timeline = scene.timeline_from_trajectory(baked["contour"], label="contour")
    assert timeline.duration == pytest.approx(baked["contour"].duration)
    out = tmp_path / "cell.usda"
    warnings = timeline.export_usd(str(out))
    assert warnings == []
    text = out.read_text()
    # Feed and rapid overlays for every registered toolpath.
    assert text.count("def BasisCurves") == 6
    for prim in (
        "contour_feed", "contour_rapid", "pocket_feed", "pocket_rapid",
        "rim_feed", "rim_rapid",
    ):
        assert f'"{prim}"' in text, prim
    # Nothing ever touches: the clearance floor over the whole cut is the
    # cutter passing the toe clamps.
    assert float(timeline.min_clearance()) > 0.002


# ---------------------------------------------------------------------------
# Progressive material removal: the plate is cut away *during* playback.


@pytest.fixture(scope="module")
def animated():
    """A fresh cell with the carve stages registered — its own scene,
    because `animate_carve` adds the stage obstacles to it."""
    scene, _ = demo.build_cell()
    tl = scene.simulate_sequence("cycle", toolpath_spin="optimize")
    return scene, scene.animate_carve(tl, "plate")


@pytest.fixture(scope="module")
def animated_usd(animated, tmp_path_factory):
    _, tl = animated
    out = tmp_path_factory.mktemp("carve") / "cycle.usda"
    assert tl.export_usd(str(out)) == []
    return out


def test_animate_carve_stages_the_removal_without_touching_the_cycle(animated):
    scene, tl = animated
    stages = [n for n in scene.obstacle_names if n.startswith("plate_cut/")]
    assert len(stages) > 5, stages
    assert tl.duration == pytest.approx(CYCLE_GOLDEN, abs=0.25)
    # The stages are scenery: registered without collision, so the same
    # cycle re-bakes to the same clock with all of them in the scene.
    again = scene.simulate_sequence("cycle", toolpath_spin="optimize")
    assert again.duration == pytest.approx(tl.duration, abs=1e-9)


def test_the_removal_is_progressive_in_usd(animated, animated_usd):
    """The plate prim holds until the first real cut, then the stage
    prims take over hand-to-hand until the last one carries the final
    part through the end of the cycle. Visibility samples are sparse —
    the first frame plus transitions, held in between (USD semantics) —
    so windows are reconstructed from the change points."""
    import re

    scene, tl = animated
    text = animated_usd.read_text()
    stages = [n for n in scene.obstacle_names if n.startswith("plate_cut/")]
    tracks = {}
    for m in re.finditer(r'def \w+ "([^"]+)"[^{]*\{', text):
        prim = m.group(1)
        if prim != "plate" and not prim.startswith("_"):
            continue
        vis = re.compile(r"visibility\.timeSamples = \{([^}]*)\}").search(text, m.end())
        tracks[prim] = [
            (float(t), v) for t, v in re.findall(r'([\d.]+): "(\w+)"', vis.group(1))
        ]
    assert len(tracks) == len(stages) + 1, sorted(tracks)
    fps = 60.0
    end = tl.duration

    def window(samples):
        """[on, off) in seconds under held interpolation."""
        on = next(t for t, v in samples if v == "inherited") / fps
        off = next((t / fps for t, v in samples if v == "invisible" and t / fps > on), end)
        return on, off

    # The pristine plate shows from frame 0 until the first real cut.
    assert tracks["plate"][0] == (0.0, "inherited")
    plate_on, first_cut = window(tracks["plate"])
    assert plate_on == 0.0
    assert 2.0 < first_cut < end / 2
    # Stages chain contiguously from the handover to the final frame.
    windows = sorted(window(tracks[p]) for p in tracks if p != "plate")
    assert windows[0][0] == pytest.approx(first_cut, abs=1.0)
    assert windows[-1][1] == pytest.approx(end, abs=1e-6)
    for (_, a_off), (b_on, _) in zip(windows, windows[1:]):
        assert b_on == pytest.approx(a_off, abs=0.05)


def test_a_recording_replays_the_removal(animated, animated_usd):
    """`play_record` route: the exported cycle replayed onto a cell that
    re-carved the same stages — every stage prim finds its obstacle and
    comes back as a visibility track, with nothing left unmatched."""
    scene, _ = animated
    result = scene.play_usd_animation(str(animated_usd))
    assert result["warnings"] == []
    tracks = set(result["object_tracks"])
    stages = {n for n in scene.obstacle_names if n.startswith("plate_cut/")}
    assert stages and stages <= tracks
    assert "plate" in tracks
