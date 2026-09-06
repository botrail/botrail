"""The AMR example, asserted the way a cell owner would assert it.

`examples/vehicles/amr_demo.py` is a mobile manipulator assembled from three
catalog packages — a carrier, an arm, a gripper — running one transfer
between a bench in an aisle and a conveyor in a machining bay. What these
tests pin is the part that is easy to break silently: that the machine's
own numbers are *derived* from its package rather than typed in, so that
swapping the carrier moves everything that depends on it.

* the deck the arm bolts to, the body that drives the aisle and the
  speed the legs are driven at all come from the carrier's package,
* the machine stops where its own arm is level with the work, and noses
  into the bay as deep as its own body allows,
* the part really is carried on the deck: the load sensor rides with it,
  and the departure permit waits on it,
* the stow is a ramp inside the drive (free), and its path is checked
  even though a ramp is not planned,
* a *planned* motion in that same step is refused, by name.

The cell is built from catalog products, so these tests need the catalog
packages — cached locally or fetched once. Where the catalog is
unreachable they skip rather than fail; the engine's own coverage lives
in the Rust suites.
"""

import json
import math
import sys
import zipfile
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "vehicles"))

import amr_demo as demo  # noqa: E402

ALT = "rb-theron"  # a second carrier: half the deck height of the default


def _or_skip(build):
    """Run `build`, or skip when the catalog cannot be reached — an
    offline machine (and CI, which fakes the catalog out) has no business
    failing on a download."""
    try:
        return build()
    except pytest.skip.Exception:
        raise
    except Exception as err:  # noqa: BLE001 - any fetch/parse failure skips
        pytest.skip(f"catalog unavailable: {err}")


@pytest.fixture(scope="module")
def carrier():
    return _or_skip(lambda: demo.Carrier(demo.CARRIER))


@pytest.fixture(scope="module")
def baked():
    return _or_skip(lambda: demo.bake(demo.CARRIER))


def test_the_machine_is_measured_not_typed(carrier):
    """Deck, body and speed come out of the package, and agree with the
    data sheet the same package publishes."""
    specs = carrier.specs
    assert carrier.deck == pytest.approx(specs["deck_height_mm"] / 1e3, abs=0.01)
    # The arm bolts a plate's thickness above the surface it stands on,
    # and that surface is the chassis, not the frame's nominal height.
    assert carrier.mount[2] == pytest.approx(carrier.surface + demo.PLATE)
    assert carrier.surface >= carrier.deck
    # The body is the collision geometry, so its footprint is the real
    # machine's, and the pivot sweeps its half-diagonal.
    footprint = [v / 1e3 for v in specs["footprint_mm"]]
    assert carrier.length <= footprint[0] + 0.05
    assert carrier.swing > carrier.width / 2
    # Speed is derated off the data sheet, never above it.
    assert carrier.cruise(3.0) <= specs["max_speed_mps"]
    assert carrier.cruise(0.5) < carrier.cruise(3.0)


def test_the_body_draws_complete_visuals_with_separate_collisions(carrier, tmp_path):
    """The decimated collision STL must never replace the full appearance."""
    scene = demo.bt.Scene()
    carrier.add_body(scene, "amr")
    project = tmp_path / "body.botrail"
    scene.save_project(project)
    with zipfile.ZipFile(project) as archive:
        body = {o["name"]: o for o in json.loads(archive.read("project.json"))["obstacles"]}
        collisions = {f"amr/{p.stem}" for p in carrier.pieces}
        assert {n for n, o in body.items() if o["enabled"]} == collisions
        for stl in carrier.pieces:
            obstacle = body[f"amr/{stl.stem}"]
            assert not obstacle["visible"]
            assert archive.read(obstacle["geometry"]["url"]) == stl.read_bytes()

        visuals = {n for n, o in body.items() if o["visible"]}
        assert visuals == set(body) - collisions
        assert {
            f"amr/visual/{link}" for link in (
                "base_link", "top_cover", "front_right_wheel_link",
                "front_left_wheel_link", "back_right_wheel_link", "back_left_wheel_link",
            )
        } <= visuals
        chassis = body["amr/visual/base_link"]
        assert archive.read(chassis["geometry"]["url"]) == (
            carrier.visual_urdf.parent / "../sources/meshes/bases/rbkairos/rbkairos_chassis.stl"
        ).read_bytes()
        assert chassis["color"] == [0.0, 0.0, 0.0]  # catalog black, not a grey override
        assert body["amr/visual/top_cover"]["color"] == [0.5, 0.5, 0.5]
    assert scene.frames == {}  # fixed world frames would not ride with the body


def test_the_complete_appearance_rides_and_turns_with_the_body(baked):
    _, timeline = baked
    base0, q0 = timeline.base_pose(0.0)
    base1, q1 = timeline.base_pose(timeline.duration)
    yaw = demo.yaw_of(q1) - demo.yaw_of(q0)
    c, s = math.cos(yaw), math.sin(yaw)
    for name in ("amr/base_link", "amr/visual/base_link", "amr/visual/top_cover",
                 "amr/visual/front_right_wheel_link", "amr/visual/back_left_wheel_link"):
        start, _ = timeline.object_pose(name, 0.0)
        end, _ = timeline.object_pose(name, timeline.duration)
        dx, dy, dz = (a - b for a, b in zip(start, base0))
        assert end == pytest.approx((base1[0] + c * dx - s * dy,
                                     base1[1] + s * dx + c * dy, base1[2] + dz))


def test_catalog_wheels_rotate_during_travel_and_stop_at_the_dock(baked, tmp_path):
    scene, timeline = baked
    project = tmp_path / "amr.botrail"
    scene.save_project(project)
    with zipfile.ZipFile(project) as archive:
        devices = json.loads(archive.read("project.json"))["devices"]
    wheels = next(d["kind"]["wheels"] for d in devices if d["name"] == "amr")
    assert len(wheels) == 4
    assert {w["radius"] for w in wheels} == {0.127}
    drive = timeline.step_span("走行")
    for wheel in wheels:
        assert wheel["axis"] == [0, 1, 0]
        name = wheel["object"]
        p0, q0 = timeline.object_pose(name, drive.start)
        p1, q1 = timeline.object_pose(name, drive.start + 0.1)
        distance = math.dist(p0, p1)
        assert distance > 0
        dot = abs(sum(a * b for a, b in zip(q0, q1)))
        assert 2 * math.acos(min(dot, 1)) == pytest.approx(distance / wheel["radius"], abs=1e-8)
        assert timeline.object_pose(name, 0)[1] == timeline.object_pose(name, drive.start / 2)[1]
        assert timeline.object_pose(name, drive.end)[1] == timeline.object_pose(name, timeline.duration)[1]


def test_staging_pallets_clear_tables_each_other_and_the_aisle(baked):
    scene, _ = baked

    def bounds(group):
        boxes = [scene.obstacle_bounds(n) for n in scene.obstacle_names
                 if n.startswith(f"{group}/")]
        return (tuple(min(lo[i] for lo, _ in boxes) for i in range(3)),
                tuple(max(hi[i] for _, hi in boxes) for i in range(3)))

    pallets = [bounds(f"pallet{i}") for i in range(2)]
    assert pallets[0][0][0] - bounds("lathe")[1][0] == pytest.approx(demo.PALLET_GAP)
    assert pallets[1][0][0] - pallets[0][1][0] == pytest.approx(demo.PALLET_GAP)
    for lo, hi in pallets:
        assert hi[1] <= demo.SERVED_FACE + 1e-9
        for table in ("bench", "lathe", "cnc"):
            tlo, thi = bounds(table)
            assert any(hi[i] <= tlo[i] or thi[i] <= lo[i] for i in (0, 1)), table


def test_a_deck_that_needs_a_riser_is_refused(carrier):
    """RB-SUMMIT carries its own structure where an arm would bolt on;
    the package says so, before anything is built. (The `carrier` fixture
    is here to skip the test when the catalog is unreachable — the raise
    below is the assertion, so it must not go through `_or_skip`.)"""
    with pytest.raises(ValueError, match="riser"):
        demo.Carrier("rb-summit")


def test_the_machine_stands_where_its_own_arm_reaches(carrier):
    """Both stations are derived: the infeed puts the arm level with the
    part, the outfeed noses in until the body is `NOSE` short of the
    machine tool."""
    assert carrier.infeed[0] + carrier.mount[0] == pytest.approx(demo.PART_X)
    assert carrier.outfeed[1] + carrier.hi[0] == pytest.approx(demo.CNC_FACE - demo.NOSE)


def test_swapping_the_carrier_moves_everything_that_depends_on_it(carrier):
    other = _or_skip(lambda: demo.Carrier(ALT))
    assert other.deck < carrier.deck - 0.2  # a much lower machine
    assert other.mount[2] < carrier.mount[2]
    assert other.infeed[0] != carrier.infeed[0]  # it stops somewhere else
    assert other.outfeed[1] != carrier.outfeed[1]  # and noses in differently


def test_the_part_rides_the_deck_and_lands_on_the_belt(baked):
    _, tl = baked
    loaded = tl.signal("tray_loaded")
    drive = tl.step_span("走行")
    # Cargo before dispatch, still cargo out on the aisle: a floor-mounted
    # zone would have lost it the moment the machine pulled away.
    assert loaded.value_at(drive.start)
    assert loaded.value_at((drive.start + drive.end) / 2)
    landed = tl.object_pose(demo.TOTE, tl.duration)[0]
    assert landed[0] == pytest.approx(demo.BELT_X, abs=0.05)
    assert landed[2] == pytest.approx(demo.BELT_TOP + demo.PART / 2, abs=0.02)
    # It rode the belt to the end of the run rather than sitting where it
    # was placed: the conveyor started when the arm let go.
    assert landed[1] == pytest.approx(demo.BELT_RUN[0], abs=0.05)


def test_nothing_overhangs_when_the_machine_pulls_out(baked):
    _, tl = baked
    drive = tl.step_span("走行")
    assert not tl.signal("overhang").value_at(drive.start)
    # …and the arm is back over the side at the bay, which is what the
    # envelope is for: it is the working position that trips it.
    assert tl.signal("overhang").high_total() > 0.0


def test_the_stow_is_a_ramp_inside_the_drive(baked):
    """The fold costs no cycle time: it runs while the machine travels,
    and the drive is what the step waits on."""
    _, tl = baked
    drive = tl.step_span("走行")
    ramps = [m for m in tl.moves() if m[0] == "ramp"
             and drive.start <= m[1] < drive.end]
    assert ramps, "no ramp inside the drive step"
    assert ramps[0][2] <= drive.end + 1e-6


def test_the_base_is_a_track_not_a_constant(baked):
    _, tl = baked
    start = tl.base_pose(0.0)
    end = tl.base_pose(tl.duration)
    assert start is not None and end is not None
    assert start[0] != end[0]
    # It arrives facing the bay: the last leg's direction is the heading.
    assert demo.yaw_of(end[1]) == pytest.approx(1.5708, abs=0.02)


def test_a_planned_motion_while_driving_is_refused(baked):
    with pytest.raises(ValueError, match="cannot start while `amr` is driving"):
        demo.bake(demo.CARRIER, drive_and_plan=True)


def test_the_carrier_is_on_the_bill_of_materials(baked):
    scene, _ = baked
    part = scene.part("amr")
    assert part is not None
    assert part["catalog"].startswith("robotnik/rb-kairos")


def test_the_holonomic_variant_docks_unrotated() -> None:
    # Mecanum wheels on the same route: the machine translates the corner
    # without pivoting and docks facing what it faced when parked.
    import amr_demo as demo

    scene, tl = _or_skip(lambda: demo.bake(demo.CARRIER, holonomic=True))
    _p0, q0 = tl.base_pose(0.0)
    _p1, q1 = tl.base_pose(tl.duration)
    assert max(abs(a - b) for a, b in zip(q0, q1)) < 1e-9
    assert 'drive="holonomic"' in scene.generate_python()
