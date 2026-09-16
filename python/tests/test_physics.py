"""Physics bakes (design-physics.md P1): a dynamic part falls and settles
under Rapier inside the ordinary scan-loop bake; the properties are inert
without an engine; the bake is deterministic per machine and build."""

from pathlib import Path

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


# ---------------- world scope (design-world-physics.md W0) ----------------


def _cell() -> bt.Scene:
    """A cell with nothing marked: a pinned rack (equipment), a carton
    hovering above its shelf, a pallet on the floor, a floor slab under
    the ground plane, and a box declared static in the air."""
    scene = bt.Scene()
    scene.add_box("floor", size=(4.0, 4.0, 0.1), position=(0, 0, -0.06))
    scene.add_box("rack/post", size=(0.05, 0.05, 1.5), position=(1.0, 0.0, 0.75))
    scene.add_box("rack/shelf", size=(0.8, 0.4, 0.02), position=(1.0, 0.0, 1.0))
    scene.set_part("rack", kind="group", category="structure.rack")
    scene.add_box("carton", size=(0.2, 0.15, 0.1), position=(1.0, 0.0, 1.3))
    scene.set_part("carton", category="workpiece", mass_kg=2.0)
    scene.add_box("pallet/slab", size=(1.2, 0.8, 0.14), position=(-1.0, 0.0, 0.07))
    scene.add_box("pallet/visual/board", size=(1.2, 0.1, 0.02), position=(-1.0, 0.3, 0.05))
    scene.set_obstacle_enabled("pallet/visual/board", False)
    scene.set_part("pallet", kind="group", category="pallet", mass_kg=18.0)
    scene.add_box("anvil", size=(0.1, 0.1, 0.1), position=(0.0, 1.0, 0.8))
    scene.set_physics("anvil", dynamic=False)
    return scene


def test_physics_plan_reads_the_world_before_it_runs() -> None:
    plan = _cell().physics_plan()
    by_name = {row["name"]: row for row in plan.rows}
    assert by_name["ground"]["kind"] == "ground"
    assert by_name["rack"]["kind"] == "fixed" and by_name["rack"]["members"] == ["rack/post", "rack/shelf"]
    assert by_name["carton"]["kind"] == "dynamic" and by_name["carton"]["mass_kg"] == 2.0
    assert by_name["pallet"]["kind"] == "dynamic"
    assert by_name["pallet"]["members"] == ["pallet/slab", "pallet/visual/board"]
    assert by_name["pallet"]["mass_kg"] == 18.0
    assert by_name["floor"]["kind"] == "fixed" and "below the ground" in by_name["floor"]["reason"]
    assert by_name["anvil"]["kind"] == "fixed" and by_name["anvil"]["reason"] == "declared static"
    assert set(plan.dynamic()) == {"carton", "pallet"}
    assert plan.mirrors == 0
    assert "| carton | dynamic |" in plan.to_markdown()
    # The declared scope lists nothing but the marked bodies.
    declared = _cell().physics_plan(physics=True)
    assert [row["name"] for row in declared.rows] == []
    assert declared.mirrors > 0


def test_simulate_physics_drops_the_unsupported_and_keeps_the_rest() -> None:
    scene = _cell()
    tl = scene.simulate_physics(3.0)
    assert tl.physics == "rapier" and tl.physics_scope == "world"
    assert tl.sequences == [] and tl.duration == pytest.approx(3.0)
    # The carton fell 0.25 m onto the shelf and rests on it.
    (_, _, z), _ = tl.object_pose("carton", tl.duration)
    assert z == pytest.approx(1.01 + 0.05, abs=3e-3)
    assert tl.settled_at("carton") is not None
    # The pallet is loose but rests on the ground where it was put — and
    # its visual board rode along.
    (x, _, z), _ = tl.object_pose("pallet/slab", tl.duration)
    assert abs(x + 1.0) < 3e-3 and z == pytest.approx(0.07, abs=3e-3)
    (_, y, _), _ = tl.object_pose("pallet/visual/board", tl.duration)
    assert y == pytest.approx(0.3, abs=3e-3)
    # The rack, the floor and the declared-static anvil never moved.
    for name in ("rack/post", "floor", "anvil"):
        with pytest.raises(ValueError):
            tl.object_pose(name, 1.0)
    # The same cell under the declared scope has nothing to move.
    plain = scene.simulate_physics(1.0, physics=True)
    assert plain.physics_scope == "declared"
    with pytest.raises(ValueError):
        plain.object_pose("carton", 0.5)


def test_physics_options_object_drives_a_sequence_bake(scene: bt.Scene) -> None:
    # `bt.Physics(world=True)` on an ordinary sequence bake: the declared
    # part still falls, and the cell's table is a loose unit resting on
    # the ground plane; `physics=True` stays the declared scope.
    tl = scene.simulate_sequence("settle", physics=bt.Physics(world=True))
    assert tl.physics_scope == "world"
    (_, _, z), _ = tl.object_pose("part", tl.duration)
    assert z == pytest.approx(REST_Z, abs=3e-3)
    declared = scene.simulate_sequence("settle", physics=bt.Physics())
    assert declared.physics_scope == "declared"
    with pytest.raises(ValueError, match="unknown physics engine"):
        bt.Physics(engine="mujoco")
    with pytest.raises(ValueError):
        scene.simulate_physics(0.0)
    with pytest.raises(ValueError):
        scene.simulate_physics(1.0, physics=False)
    assert repr(bt.Physics(world=True)).startswith("Physics(engine='rapier', world=True")


# ------------- world scope robots (design-world-physics.md W1) -------------

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]


def _arm_cell() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.set_joint_positions(READY)
    scene.add_box("floor", size=(3.0, 3.0, 0.1), position=(0, 0, -0.06))
    sq = scene.sequence("hold")
    sq.step("wait", transition=bt.seq.elapsed(1.0))
    lift = scene.sequence("lift")
    lift.step("up", actions=[bt.seq.ramp({"shoulder_lift": 1.0}, 1.0)], transition=bt.seq.done())
    lift.step("settle", transition=bt.seq.elapsed(0.5))
    return scene


def test_an_undeclared_arm_collapses_unpowered_and_holds_powered() -> None:
    scene = _arm_cell()
    plan = scene.physics_plan()
    (row,) = [r for r in plan.rows if r["kind"] == "robot"]
    assert row["name"] == "simple_arm" and "base on its stand" in row["reason"]
    tl = scene.simulate_physics(3.0)
    q = tl.sample(tl.duration)
    assert abs(q[1] - READY[1]) > 0.5, q
    # Motors on: the servo holds READY to a few milliradians.
    held = scene.simulate_physics(1.0, physics=bt.Physics(world=True, powered=True))
    assert max(abs(a - b) for a, b in zip(held.sample(held.duration), READY)) < 0.02
    # A program powers the robot it drives, by default; one that never
    # moves it leaves it unpowered, and `powered=False` cuts it outright.
    ran = scene.simulate_sequence("lift", physics=bt.Physics(world=True))
    assert abs(ran.sample(ran.duration)[1] - 1.0) < 0.02
    idle = scene.simulate_sequence("hold", physics=bt.Physics(world=True))
    assert abs(idle.sample(idle.duration)[1] - READY[1]) > 0.3
    cut = scene.simulate_sequence("lift", physics=bt.Physics(world=True, powered=False))
    assert abs(cut.sample(cut.duration)[1] - READY[1]) > 0.3
    # The declared scope leaves an undeclared arm kinematic.
    plain = scene.simulate_physics(0.5, physics=True)
    assert plain.sample(plain.duration) == READY


def test_a_live_rollout_snapshots_its_timeline_as_it_goes() -> None:
    scene = _arm_cell()
    live = scene.open_rollout("lift", physics=bt.Physics(world=True))
    live.tick(60)
    snap = live.timeline()
    assert not live.finished
    assert abs(snap.duration - 0.6) < 1e-6
    # The open step's band runs to the clock; the robot lane is the
    # servo's read-back so far.
    assert [(n, round(a, 2), round(b, 2)) for n, a, b in snap.step_spans] == [("up", 0.0, 0.6)]
    assert abs(snap.sample(snap.duration)[1] - (0.6 + 0.4 * 0.6)) < 0.05
    while not live.finished:
        live.tick()
    whole = live.finish(publish=False)
    assert abs(whole.duration - 1.5) < 0.011
    assert whole.step_spans[0][0] == "up" and len(whole.step_spans) == 2


def test_a_floating_declaration_round_trips_and_floats(tmp_path) -> None:
    scene = _arm_cell()
    scene.set_robot_base_pose((0.0, 0.0, 0.5))
    scene.set_robot_physics(floating=True)
    path = tmp_path / "cell.json"
    scene.save_project(path)
    loaded = bt.Scene.load_project(path)
    assert loaded.robot_physics()
    row = [r for r in loaded.physics_plan(physics=True).rows if r["kind"] == "robot"][0]
    assert "base floating" in row["reason"], row["reason"]
    tl = loaded.simulate_physics(2.0, physics=bt.Physics(ground=0.0))
    (_, _, z), _ = tl.base_pose(tl.duration)
    assert z < 0.3, z


def test_the_studio_host_takes_the_physics_toggle_meaning() -> None:
    # What the studio's physics toggle bakes under is the host's: the
    # whole cell by default, a `bt.Physics(...)` verbatim, or no physics.
    from botrail import _core

    scene = _cell()
    for physics in (None, True, False, bt.Physics(world=True, powered=False)):
        server = _core.serve_studio(scene, "/nonexistent-studio", "127.0.0.1", 0, physics)
        server.stop()
    with pytest.raises(ValueError, match="bt.Physics"):
        _core.serve_studio(scene, "/nonexistent-studio", "127.0.0.1", 0, "nope")
