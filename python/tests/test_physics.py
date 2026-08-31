"""Physics bakes (design-physics.md P1): a dynamic part falls and settles
under Rapier inside the ordinary scan-loop bake; the properties are inert
without an engine; the bake is deterministic per machine and build."""

import pytest

import botrail as bt

TABLE_TOP = 0.72
PART_HALF = 0.015
REST_Z = TABLE_TOP + PART_HALF


@pytest.fixture()
def scene() -> bt.Scene:
    scene = bt.Scene()
    scene.add_box("table", size=(1.2, 0.8, TABLE_TOP), position=(0, 0, TABLE_TOP / 2))
    scene.add_box("part", size=(0.1, 0.05, 2 * PART_HALF), position=(0.1, 0.05, 1.5))
    scene.set_physics("part", dynamic=True, mass=0.2, friction=0.5)
    sq = scene.sequence("settle")
    sq.step("wait", transition=bt.seq.elapsed(2.0))
    return scene


def test_dynamic_part_falls_and_settles(scene: bt.Scene) -> None:
    tl = scene.simulate_sequence("settle", physics=True)
    assert tl.physics == "rapier"
    # Mid-fall: below the start, above the table.
    (_, _, z), _ = tl.object_pose("part", 0.2)
    assert REST_Z + 0.05 < z < 1.5
    # Settled on the table top (contact slop allowed), not tipped over.
    (x, y, z), _ = tl.object_pose("part", tl.duration)
    assert z == pytest.approx(REST_Z, abs=3e-3)
    assert abs(x - 0.1) < 0.05 and abs(y - 0.05) < 0.05


def test_physics_props_are_inert_without_an_engine(scene: bt.Scene) -> None:
    tl = scene.simulate_sequence("settle")
    assert tl.physics is None
    # Nothing moved, so nothing is tracked — today's kinematic bake.
    with pytest.raises(ValueError):
        tl.object_pose("part", 0.5)


def test_physics_bake_is_deterministic(scene: bt.Scene) -> None:
    a = scene.simulate_sequence("settle", physics=True)
    b = scene.simulate_sequence("settle", physics=True)
    for k in range(21):
        t = 0.1 * k
        assert a.object_pose("part", t) == b.object_pose("part", t)


def test_unknown_engine_is_rejected(scene: bt.Scene) -> None:
    with pytest.raises(ValueError, match="unknown physics engine"):
        scene.simulate_sequence("settle", physics="mujoco")


def test_physics_props_roundtrip_through_a_project(scene: bt.Scene, tmp_path) -> None:
    path = tmp_path / "cell.json"
    scene.save_project(path)
    reloaded = bt.Scene.load_project(path)
    tl = reloaded.simulate_sequence("settle", physics=True)
    (_, _, z), _ = tl.object_pose("part", tl.duration)
    assert z == pytest.approx(REST_Z, abs=3e-3)


def test_usd_export_carries_the_fall(scene: bt.Scene, tmp_path) -> None:
    tl = scene.simulate_sequence("settle", physics=True)
    out = tmp_path / "drop.usda"
    assert tl.export_usd(str(out), fps=30.0) == []
    src = out.read_text()
    assert '"part"' in src and "timeSamples" in src


def _belt_scene() -> bt.Scene:
    scene = bt.Scene()
    scene.add_box("bed", size=(2.2, 0.3, 0.1), position=(0, 0, 0.65))
    scene.add_box("stopper", size=(0.04, 0.3, 0.1), position=(0.9, 0, 0.75))
    scene.add_box("part", size=(0.1, 0.08, 0.06), position=(-0.8, 0, 0.76))
    scene.set_physics("part", dynamic=True, mass=0.3, friction=0.6)
    # Zone shaped so the bed and stopper origins stay out of it: the
    # advection captures origins indiscriminately, and the physics mirror
    # faithfully moves whatever it captures.
    scene.add_conveyor("conv", zone_position=(-0.125, 0, 0.815), zone_size=(1.95, 0.3, 0.27),
                       velocity=(0.3, 0, 0), running=False)
    scene.add_zone_sensor("at_stop", position=(0.8, 0, 0.78), size=(0.12, 0.3, 0.12),
                          watch=["part"])
    sq = scene.sequence("run")
    sq.step("feed", actions=[bt.seq.start("conv")],
            transition=bt.seq.signal("at_stop", True))
    sq.step("seat", transition=bt.seq.elapsed(2.0))
    sq.step("hold", actions=[bt.seq.stop("conv")],
            transition=bt.seq.elapsed(2.5))
    return scene


def test_a_belt_conveys_a_part_into_the_sensor_and_the_stopper() -> None:
    scene = _belt_scene()
    tl = scene.simulate_sequence("run", physics=True)
    assert tl.physics == "rapier"
    # Cruise at belt speed while the run is still open.
    x = lambda t: tl.object_pose("part", t)[0][0]
    assert (x(4.0) - x(2.0)) / 2.0 == pytest.approx(0.3, abs=0.03)
    # The presence sensor rose, and the program advanced on it.
    times, values = zip(*tl.signal("at_stop").edges)
    assert True in values
    # Seated against the stopper face at x = 0.88 (part half 0.05).
    assert x(tl.duration) == pytest.approx(0.88 - 0.05, abs=0.01)
    # And the bake can say what happened: the press is a touch episode,
    # the arrest under the running belt is a stall, and the part sleeps.
    pairs = {frozenset((c["a"], c["b"])) for c in tl.contacts}
    assert frozenset(("part", "stopper")) in pairs
    assert any(s["object"] == "part" and s["device"] == "conv"
               for s in tl.conveyor_stalls())
    assert tl.settled_at("part") is not None


def test_the_kinematic_belt_still_advects_the_same_cell() -> None:
    # Same authoring, no physics marks: the belt advects the part exactly
    # as before — one belt, two transport modes.
    scene = _belt_scene()
    scene.set_physics("part", dynamic=False)
    tl = scene.simulate_sequence("run")
    assert tl.physics is None
    x = lambda t: tl.object_pose("part", t)[0][0]
    assert (x(3.0) - x(1.0)) / 2.0 == pytest.approx(0.3, abs=0.01)


def test_detach_hands_the_carrier_velocity_to_the_engine() -> None:
    # A part released mid-swing flies on with the arm's velocity and lands
    # down range; the ride itself is the ordinary rigid attach.
    from pathlib import Path

    examples = Path(__file__).resolve().parents[2] / "examples"
    scene = bt.Scene(bt.Robot.from_urdf(examples / "assets" / "simple_arm.urdf"))
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.05))
    names = scene.robot.joint_names
    bent = [0.0, 1.1, 0.9, 0.0, 0.0, 0.0]
    scene.set_joint_positions(bent)
    (tx, ty, tz), _ = scene.link_pose(scene.robot.tcp_link)
    scene.add_box("part", size=(0.05, 0.05, 0.05), position=(tx, ty, tz + 0.05))
    scene.set_physics("part", dynamic=True, mass=0.2, friction=0.6)

    swung = list(bent)
    swung[0] += 1.5
    sq = scene.sequence("throw")
    sq.step("grab", actions=[bt.seq.attach("part")])
    sq.step("swing", actions=[bt.seq.ramp(dict(zip(names, swung)), 1.0)],
            transition=bt.seq.elapsed(0.5))
    sq.step("release", actions=[bt.seq.detach("part")],
            transition=bt.seq.elapsed(2.5))

    tl = scene.simulate_sequence("throw", physics=True)
    release = tl.step_span("release").start
    (rx, ry, _), _ = tl.object_pose("part", release)
    (ex, ey, ez), _ = tl.object_pose("part", tl.duration)
    carried = ((ex - rx) ** 2 + (ey - ry) ** 2) ** 0.5
    assert carried > 0.3, f"flew only {carried} m"
    assert ez == pytest.approx(0.025, abs=0.01)  # on the floor, not in it


def _identity_scene() -> bt.Scene:
    # No explicit mass= — the part identity is the only mass source (an
    # explicit mass always wins over the identity, which is why the
    # shared fixture with its mass=0.2 is no use here).
    scene = bt.Scene()
    scene.add_box("table", size=(1.2, 0.8, TABLE_TOP), position=(0, 0, TABLE_TOP / 2))
    scene.add_box("part", size=(0.1, 0.05, 2 * PART_HALF), position=(0.1, 0.05, 1.5))
    scene.set_physics("part", dynamic=True)
    sq = scene.sequence("settle")
    sq.step("wait", transition=bt.seq.elapsed(2.0))
    return scene


def test_part_identity_mass_flows_into_bake_and_usd(tmp_path) -> None:
    # A part identity stating mass_kg is the mass default for a dynamic
    # body with no explicit mass= — in the bake (a 2.5 kg part lands ~17×
    # harder than the 0.15 kg density default) and in the USD export.
    light = _identity_scene().simulate_sequence("settle", physics=True)
    scene = _identity_scene()
    scene.set_part("part", category="part", attributes={"mass_kg": 2.5})
    heavy = scene.simulate_sequence("settle", physics=True)
    peak = lambda tl: max(c["peak_force"] for c in tl.contacts)
    assert peak(heavy) > 5 * peak(light)

    pytest.importorskip("pxr")
    from pxr import Usd, UsdPhysics

    out = tmp_path / "phys.usda"
    assert heavy.export_usd(str(out), fps=30.0) == []
    stage = Usd.Stage.Open(str(out))
    part = stage.GetPrimAtPath("/World/Env/part")
    assert part.HasAPI(UsdPhysics.RigidBodyAPI)
    assert part.HasAPI(UsdPhysics.CollisionAPI)
    assert UsdPhysics.MassAPI(part).GetMassAttr().Get() == pytest.approx(2.5)
    # Friction rides a bound physics material; un-annotated scenery stays
    # visual-only (the table here has no physics props → no collider).
    targets = part.GetRelationship("material:binding:physics").GetTargets()
    material = UsdPhysics.MaterialAPI(stage.GetPrimAtPath(str(targets[0])))
    assert material.GetStaticFrictionAttr().Get() == pytest.approx(0.5)
    table = stage.GetPrimAtPath("/World/Env/table")
    assert not table.HasAPI(UsdPhysics.CollisionAPI)
