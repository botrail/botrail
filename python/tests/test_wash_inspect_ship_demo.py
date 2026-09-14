"""The humanoid finishing line, asserted the way its owner would assert it.

`examples/legged/wash_inspect_ship_demo.py` walks a Unitree G1 round four
stations with one taught set of reaches. What the cell-level test pins:

* the part travels the whole loop and ends in the shipping tray, seated on
  its insert — the same joint goals worked at four stations;
* the START button was actually pressed (its lane rises while the arm is
  at the bath) and the E-stop beside it never was;
* the verdict was read off the camera (`insp_ok`) and latched;
* an NG part takes the other branch and ends in the reject tray;
* a visitor in the gate stops the cycle before the first walk, and an open
  wire on START stops it at the bath.

Needs the catalog (the G1 and the scenery are products); skipped when it
cannot be fetched.
"""

import sys
from pathlib import Path

import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "legged"))

import wash_inspect_ship_demo as demo


@pytest.fixture(scope="module")
def baked():
    try:
        # Exercise the actual wash programme within Studio's default cap.
        return demo.bake(max_duration=120.0)
    except Exception as err:
        if any(word in str(err).lower() for word in ("catalog", "fetch", "resolve", "not found", "404")):
            pytest.skip(f"catalog unavailable: {err}")
        raise


def _at(tl, name):
    return tl.object_pose(name, tl.duration)[0]


def _rotate(q, v):
    x, y, z, w = q
    a, b, c = v
    uv = (y*c-z*b, z*a-x*c, x*b-y*a)
    uuv = (y*uv[2]-z*uv[1], z*uv[0]-x*uv[2], x*uv[1]-y*uv[0])
    return tuple(v[i] + 2 * (w * uv[i] + uuv[i]) for i in range(3))


def test_the_workpiece_stays_inside_the_downward_facing_palm(baked):
    scene, _, tl = baked
    original_base = scene.robot_base_pose_of(demo.ROBOT)
    try:
        for step in ("lift", "carry 1", "to bath", "lift 2", "carry 2", "to inspection",
                     "lift 3", "carry 3", "to shipping"):
            span = tl.step_span(f"finish/{step}")
            for fraction in (.25, .5, .75):
                t = span.start + (span.end - span.start) * fraction
                scene.set_robot_base_pose(*tl.base_pose(t, robot=demo.ROBOT), robot=demo.ROBOT)
                hand, q = scene.link_pose_at(demo.HAND, tl.sample(t, robot=demo.ROBOT), robot=demo.ROBOT)
                part, part_q = tl.object_pose("part", t)
                in_hand = _rotate((-q[0], -q[1], -q[2], q[3]), tuple(p-h for p, h in zip(part, hand)))
                # The G1 hand's useful palm/finger region, in its own frame.
                # The old grip was (60, 0, -70) mm, below the little finger.
                assert .070 < in_hand[0] < .080, (step, t, in_hand)
                assert .034 < in_hand[1] < .042, (step, t, in_hand)
                assert abs(in_hand[2]) < .002, (step, t, in_hand)
                # Local +Y is the palm face; it must face the work's top.
                assert _rotate(q, (0, 1, 0)) == pytest.approx(_rotate(part_q, (0, 0, -1)), abs=.002)
    finally:
        scene.set_robot_base_pose(*original_base, robot=demo.ROBOT)


def test_releases_keep_the_workpiece_seated_on_each_support(baked):
    _, _, tl = baked
    for step, station in (("release 1", "w"), ("release 2", "i"), ("release 3", "s")):
        p, q = tl.object_pose("part", tl.step_span(f"finish/{step}").end)
        expected = demo.local(station, *demo.WORK, demo.RISER_TOP + demo.SEAT + demo.PART[2] / 2)
        assert p == pytest.approx(expected, abs=.001), step
        assert _rotate(q, (0, 0, 1)) == pytest.approx((0, 0, 1), abs=.002), step


def test_the_part_ends_in_the_shipping_tray(baked):
    _scene, _poses, tl = baked
    x, y, z = _at(tl, "part")
    ex, ey, ez = demo.local("s", demo.WORK[0], demo.WORK[1], demo.RISER_TOP + demo.SEAT + demo.PART[2] / 2)
    assert abs(x - ex) < 0.01 and abs(y - ey) < 0.01 and abs(z - ez) < 0.005, (x, y, z)
    assert tl.duration < 120.0


def test_cartons_rest_on_the_actual_shelf_decks(baked):
    scene, _, _ = baked
    for carton, level in enumerate((0, 0, 1, 2)):
        lo, hi = scene.obstacle_bounds(f"cartons/c{carton}")
        shelf_lo, shelf_hi = scene.obstacle_bounds(f"shelf/shelves/l{level}")
        assert lo[2] == pytest.approx(shelf_hi[2], abs=1e-9)
        assert min(hi[0], shelf_hi[0]) > max(lo[0], shelf_lo[0])
        assert min(hi[1], shelf_hi[1]) > max(lo[1], shelf_lo[1])


def test_finished_project_reloads_with_the_workpiece_visual(baked, tmp_path):
    import json

    import botrail as bt

    scene, _, _ = baked
    path = tmp_path / "finishing.botrail"
    scene.save_project(path)
    loaded = bt.Scene.load_project(path)
    objects = {o["name"]: o for o in json.loads(loaded._project_json())["obstacles"]}
    assert objects["part"]["visual_asset"]["prim_path"] == "/Shapes/workpiece"
    assert not any(name.startswith("teach/") for name in objects)


def test_head_assemblies_follow_declared_datums_and_single_bom_lines(baked):
    import json

    scene, _, _ = baked
    for device, host in (("head_camera_device_camera_depth_frame", "d435_link"),
                         ("head_radar_device_livox_frame", "mid360_link")):
        actual = scene.link_pose(device, robot=demo.ROBOT)
        expected = scene.link_pose(host, robot=demo.ROBOT)
        offset = _rotate(expected[1], (0, 0, demo.LIDAR_SEAT if host == "mid360_link" else 0))
        assert actual[0] == pytest.approx(tuple(a+b for a, b in zip(expected[0], offset)), abs=1e-8)
        assert abs(sum(a*b for a, b in zip(actual[1], expected[1]))) == pytest.approx(1, abs=1e-8)
    # The viewer camera's -Z forward/+Y up maps to ROS optical +Z/-Y.
    live = scene.open_rollout(demo.PROGRAMS)
    pos, q = live.camera_pose("g1_eyes")
    optical_pos, optical_q = scene.link_pose("head_camera_device_camera_depth_optical_frame", robot=demo.ROBOT)
    assert pos == pytest.approx(optical_pos, abs=1e-8)
    assert _rotate(q, (0, 0, -1)) == pytest.approx(_rotate(optical_q, (0, 0, 1)), abs=1e-8)
    data = json.loads(scene._project_json())
    for key, name in (("cameras", "g1_eyes"), ("lidars", "g1_lidar")):
        sensor = next(s for s in data[key] if s["name"] == name)
        assert sensor["body_visible"] is False
    catalog_rows = [row for row in scene.bom().rows if row.get("catalog")]
    for product in (demo.CAMERA_PLATE, demo.RADAR_PLATE, demo.HEAD_CAMERA, demo.HEAD_LIDAR):
        rows = [row for row in catalog_rows if row["catalog"].split("@", 1)[0] == product]
        assert len(rows) == 1 and rows[0]["qty"] == 1


def test_the_button_was_pressed_and_the_estop_was_not(baked):
    _scene, _poses, tl = baked
    press = tl.step_span("finish/press start")
    spans = tl.signal("washer_panel/start").high_spans()
    assert spans and press.start <= spans[0][0] <= press.end + 1e-9, spans
    assert not tl.signal("washer_panel/estop").high_spans()
    assert tl.signal("wash_done").high_spans()


def test_the_verdict_was_read_off_the_camera_and_latched(baked):
    _scene, _poses, tl = baked
    ok = tl.signal("insp_ok").high_spans()
    assert ok and not tl.signal("insp_ng").high_spans()
    latched = tl.signal("ok_latched").high_spans()
    assert latched and latched[0][0] >= ok[0][0]
    assert tl.step_span("finish/to shipping").end < tl.duration


def test_one_reach_serves_every_station(baked):
    _scene, poses, _tl = baked
    # The taught set: one grip, reused at the tray, the bath, the stage and
    # the shipping tray — and the fingertips' reach is a measurement.
    assert 0.10 < poses["finger_reach"] < 0.16
    assert set(poses) >= {"above", "grip", "carry", "home", "press", "pre_press", "ng_above", "ng_place"}


def test_the_fat_rows(baked):
    scene, _poses, tl = baked
    runs = scene.simulate_scenarios(demo.PROGRAMS, max_duration=tl.duration + 30.0)
    # An NG part completes by the other branch: it ends in the reject tray.
    assert "ng_part" not in runs.errors, runs.errors.get("ng_part")
    x, y, z = _at(runs["ng_part"], "part")
    ex, ey, _ = demo.local("i", demo.NG_WORK[0], demo.NG_WORK[1], 0.0)
    assert abs(x - ex) < 0.01 and abs(y - ey) < 0.01, (x, y)
    assert z == pytest.approx(demo.RISER_TOP + demo.SEAT + demo.PART[2] / 2, abs=.001)
    assert runs["ng_part"].signal("insp_ng").high_spans()
    # The rest must refuse to go on, each where its guard stands.
    for name in ("visitor_in_gate", "start_wire_open", "tray_empty"):
        assert name in runs.errors, f"{name} completed"
    assert "gate clear" in runs.errors["visitor_in_gate"]
    assert "washing" in runs.errors["start_wire_open"] or "press" in runs.errors["start_wire_open"]
    assert "wait part" in runs.errors["tray_empty"]
