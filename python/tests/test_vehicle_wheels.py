"""Wheel appearance follows checked travel; these tests need no catalog."""
import json
import math

import botrail as bt
import pytest

WHEELS = {"fl": (0.2, 0.3, -1), "fr": (0.2, -0.3, 1),
          "rl": (-0.2, 0.3, 1), "rr": (-0.2, -0.3, -1)}


def cell(*, animate=True, holonomic=False, reverse=False):
    scene = bt.Scene()
    scene.add_box("cart/body", (0.5, 0.4, 0.2), (0, 0, 0.25))
    for name, (x, y, lateral) in WHEELS.items():
        # A non-circular visual makes the orientation observable.
        scene.add_box(f"cart/{name}", (0.12, 0.06, 0.2), (x, y, 0.1))
        scene.set_obstacle_enabled(f"cart/{name}", False)
    path = [(0, 0), (1, 0)] if reverse else [(0, 0), (1, 0), (1, 1)]
    scene.add_vehicle("cart", body=["cart"], path=path,
                      stations={"a": 0, "b": len(path) - 1}, speed=0.5,
                      turn_speed=math.pi / 2, allow_reverse=reverse,
                      drive="holonomic" if holonomic else "differential")
    if animate:
        for name, (_, _, lateral) in WHEELS.items():
            scene.set_vehicle_wheel("cart", f"cart/{name}", radius=0.1,
                                    lateral_ratio=lateral)
    sq = scene.sequence("haul")
    sq.step("park", transition=bt.seq.elapsed(0.5))
    sq.step("out", actions=[bt.seq.goto("cart", "b")],
            transition=bt.seq.device_done("cart"))
    if reverse:
        sq.step("back", actions=[bt.seq.goto("cart", "a")],
                transition=bt.seq.device_done("cart"))
    sq.step("dwell", transition=bt.seq.elapsed(0.5))
    return scene


def assert_spin(timeline, name, t, angle):
    """Expected world quaternion = body yaw * local axle rotation."""
    _, (x, y, z, w) = timeline.object_pose("cart/body", t)
    s, c = math.sin(angle / 2), math.cos(angle / 2)
    expected = (x * c - z * s, w * s + y * c, x * s + z * c, w * c - y * s)
    _, actual = timeline.object_pose(f"cart/{name}", t)
    assert abs(sum(a * b for a, b in zip(actual, expected))) == pytest.approx(1, abs=1e-9)


@pytest.mark.parametrize("holonomic", [False, True])
def test_distance_turning_stops_and_seeking(holonomic):
    tl = cell(holonomic=holonomic).simulate_sequence("haul")
    baseline = cell(animate=False, holonomic=holonomic).simulate_sequence("haul")
    start = tl.step_span("out").start
    end = tl.step_span("out").end
    assert tl.duration == baseline.duration
    # Check backwards seeks too. Rotations are derived from time, never
    # accumulated by the viewer's rendering frame rate.
    for name, (_, y, lateral) in WHEELS.items():
        for elapsed in (1.7, 0.2, 1.0):
            assert_spin(tl, name, start + elapsed, 0.5 * elapsed / 0.1)
        if holonomic:
            assert_spin(tl, name, start + 3, 10 + lateral * 5)
            final_angle = 10 + lateral * 10
        else:
            turn = (-1 if y > 0 else 1) * 5 * math.pi / 2
            assert_spin(tl, name, start + 2.5, 10 + turn / 2)
            final_angle = 20 + turn
        assert_spin(tl, name, end, final_angle)
        assert_spin(tl, name, tl.duration, final_angle)
        assert_spin(tl, name, start / 2, 0)
        for t in (0, start + 1, start + 2.5, end, tl.duration):
            assert tl.object_pose(f"cart/{name}", t)[0] == pytest.approx(
                baseline.object_pose(f"cart/{name}", t)[0])
            assert tl.object_pose("cart/body", t) == baseline.object_pose("cart/body", t)


def test_reversing_unwinds_the_wheels():
    tl = cell(reverse=True).simulate_sequence("haul")
    back = tl.step_span("back").start
    for name in WHEELS:
        assert_spin(tl, name, back + 1, 5)
        assert_spin(tl, name, tl.duration, 0)


def test_project_and_generated_python_keep_wheel_bindings(tmp_path, monkeypatch):
    scene = cell(holonomic=True)
    # Re-registering replaces this wheel, rather than applying two spins.
    scene.set_vehicle_wheel("cart", "cart/fl", radius=0.1, lateral_ratio=-1)
    path = tmp_path / "cart.botrail"
    scene.save_project(path)
    # Primitive-only projects are plain JSON (no external assets to pack).
    project = json.loads(path.read_text())
    wheels = project["devices"][0]["kind"]["wheels"]
    assert len(wheels) == 4
    assert wheels[0]["radius"] == 0.1
    loaded = bt.Scene.load_project(path)
    namespace = {}
    monkeypatch.setattr(bt, "studio", lambda scene: None)
    exec(loaded.generate_python(), namespace)  # noqa: S102 - execute our own generated fixture
    baseline = scene.simulate_sequence("haul")
    for rebuilt in (loaded, namespace["scene"]):
        tl = rebuilt.simulate_sequence("haul")
        for name in WHEELS:
            for t in (0, 1.2, 3.2, tl.duration):
                got = tl.object_pose(f"cart/{name}", t)
                want = baseline.object_pose(f"cart/{name}", t)
                assert got[0] == pytest.approx(want[0])
                assert got[1] == pytest.approx(want[1])
    # Existing projects have no wheels field and retain their rigid visuals.
    del project["devices"][0]["kind"]["wheels"]
    old = tmp_path / "old.botrail"
    old.write_text(json.dumps(project))
    assert "set_vehicle_wheel" not in bt.Scene.load_project(old).generate_python()


@pytest.mark.parametrize("settings", [
    {"radius": 0}, {"radius": -1}, {"radius": math.nan},
    {"axis": (0, 0, 0)}, {"axis": (0, 0, 1)}, {"axis": (1e308, 1e308, 0)},
    {"pivot": (math.inf, 0, 0)}, {"lateral_ratio": math.nan},
])
def test_invalid_wheel_geometry_is_refused(settings):
    with pytest.raises(ValueError, match="wheel"):
        cell().set_vehicle_wheel("cart", "cart/fl", **({"radius": 0.1} | settings))


def test_only_separate_vehicle_visuals_can_spin():
    scene = cell()
    with pytest.raises(ValueError, match="disabled"):
        scene.set_vehicle_wheel("cart", "cart/body", radius=0.1)
    with pytest.raises(ValueError, match="belong"):
        scene.set_vehicle_wheel("cart", "outside", radius=0.1)
    with pytest.raises(ValueError, match="unknown vehicle"):
        scene.set_vehicle_wheel("missing", "cart/fl", radius=0.1)


def test_usd_animation_contains_wheel_spin(tmp_path):
    pytest.importorskip("pxr")
    from pxr import Usd, UsdGeom

    tl = cell().simulate_sequence("haul")
    path = tmp_path / "cart.usdc"
    assert tl.export_usd(path, fps=20) == []
    stage = Usd.Stage.Open(str(path))
    prim = next(p for p in stage.Traverse() if p.GetName() == "fl")
    xf = UsdGeom.Xformable(prim)
    for t in (0, 0.25, 1, 3, 5.75):
        m = xf.ComputeLocalToWorldTransform(Usd.TimeCode(t * 20))
        p, q = tl.object_pose("cart/fl", t)
        assert tuple(m.ExtractTranslation()) == pytest.approx(p, abs=1e-6)
        rotation = m.RemoveScaleShear().ExtractRotationQuat()
        actual = (*rotation.GetImaginary(), rotation.GetReal())
        assert abs(sum(a * b for a, b in zip(actual, q))) == pytest.approx(1, abs=1e-6)
