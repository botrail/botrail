"""`scene.export_usd(path, physics=...)`: the cell as a simulation stage
(design-world-physics.md W4-U) — the world of `physics_plan()` in UsdPhysics,
for an engine to own (Isaac Sim / Isaac Lab). Read back here with pxr; the
run in Isaac Lab itself is `examples/export/isaaclab_cell.py`.
"""

import math
import os
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
READY = [0.0, 0.6, 0.8, 0.0, 0.5, 0.0]
FRANKA = Path(os.environ.get("BOTRAIL_CACHE") or Path.home() / ".cache" / "botrail") / "assets" / "franka" / "franka.usd"

# A small two-link arm as a USD stage in USD's own defaults — centimetres,
# Y-up — the way a non-Isaac asset comes: what a referenced robot riding a
# vehicle has to compose through.
YUP_ARM = """#usda 1.0
(
    defaultPrim = "Arm"
    metersPerUnit = 0.01
    upAxis = "Y"
)

def Xform "Arm" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
    {
        float physics:mass = 4
        def Cube "geom" (prepend apiSchemas = ["PhysicsCollisionAPI"])
        {
            double size = 1
            double3 xformOp:scale = (20, 10, 20)
            double3 xformOp:translate = (0, 5, 0)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }
    }
    def Xform "link1" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
    {
        float physics:mass = 2
        double3 xformOp:translate = (0, 10, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        def Cube "geom" (prepend apiSchemas = ["PhysicsCollisionAPI"])
        {
            double size = 1
            double3 xformOp:scale = (5, 40, 5)
            double3 xformOp:translate = (0, 20, 0)
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
        }
    }
    def Scope "joints"
    {
        def PhysicsFixedJoint "anchor"
        {
            rel physics:body1 = </Arm/base>
        }
        def PhysicsRevoluteJoint "j1" (prepend apiSchemas = ["PhysicsDriveAPI:angular"])
        {
            rel physics:body0 = </Arm/base>
            rel physics:body1 = </Arm/link1>
            uniform token physics:axis = "Z"
            point3f physics:localPos0 = (0, 10, 0)
            float physics:lowerLimit = -120
            float physics:upperLimit = 120
            uniform token drive:angular:physics:type = "force"
            float drive:angular:physics:stiffness = 1000
            float drive:angular:physics:damping = 100
        }
    }
}
"""


def _cell() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.set_joint_positions(READY)
    # Equipment: bolted down by its part pin, so it only collides.
    scene.add_box("bench/top", (1.2, 0.8, 0.04), (0.8, 0.0, 0.70))
    scene.add_box("bench/leg", (0.1, 0.1, 0.68), (0.8, 0.0, 0.34))
    scene.set_part("bench", category="structure.table", model="bench (shape example)")
    # A pallet (a carrier: loose, one body of three pieces) with a crate on
    # it that is a thing of its own.
    scene.add_box("pallet/deck", (1.0, 0.8, 0.02), (2.5, 0.0, 0.13))
    scene.add_box("pallet/block0", (0.1, 0.1, 0.12), (2.1, 0.0, 0.06))
    scene.add_box("pallet/block1", (0.1, 0.1, 0.12), (2.9, 0.0, 0.06))
    scene.set_part("pallet", category="pallet", model="pallet (shape example)", mass_kg=20.0)
    scene.add_box("pallet/crate", (0.3, 0.3, 0.2), (2.5, 0.0, 0.245))
    scene.set_part("pallet/crate", category="workpiece", model="crate (shape example)", mass_kg=3.0)
    # A conveyor: the zone sits on the bed, its legs stay below it.
    scene.add_box("line/bed", (2.0, 0.4, 0.08), (0.0, -1.5, 0.66))
    scene.add_box("line/leg", (0.06, 0.3, 0.62), (0.0, -1.5, 0.31))
    scene.set_part("line", category="conveyor", model="belt conveyor (shape example)")
    scene.add_conveyor("conv", zone_position=(0.0, -1.5, 0.835), zone_size=(2.0, 0.4, 0.27),
                       velocity=(0.2, 0.0, 0.0))
    scene.add_box("riding", (0.2, 0.2, 0.1), (-0.5, -1.5, 0.755))
    return scene


def _stage(path: Path):
    pytest.importorskip("pxr")
    from pxr import Usd

    return Usd.Stage.Open(str(path))


def test_a_simulation_stage_is_the_physics_plan_in_usdphysics(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdGeom, UsdPhysics

    scene = _cell()
    physics = bt.Physics(world=True)
    out = tmp_path / "cell.usda"
    warnings = scene.export_usd(out, physics=physics)
    # The one thing about the stage that fails silently downstream.
    assert len(warnings) == 1 and "line/bed" in warnings[0] and "CPU" in warnings[0]

    stage = _stage(out)
    assert stage.GetMetadata("kilogramsPerUnit") == 1.0
    # Values, not a one-frame animation: a consumer that writes poses back
    # must not find them shadowed by timeSamples.
    assert not any(
        attr.GetNumTimeSamples() for prim in stage.Traverse() for attr in prim.GetAttributes()
    )

    bodies = {
        str(p.GetPath())
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith("/World/Env")
    }
    # One rigid body per dynamic unit of the plan — the crate named under
    # the pallet moved out to be its sibling, since UsdPhysics would take a
    # prim below a body as part of that body — plus the belt.
    assert set(scene.physics_plan(physics).dynamic()) == {"pallet", "pallet/crate", "riding"}
    assert bodies == {
        "/World/Env/pallet",
        "/World/Env/pallet_crate",
        "/World/Env/riding",
        "/World/Env/line/bed",
    }
    pallet = stage.GetPrimAtPath("/World/Env/pallet")
    assert UsdPhysics.MassAPI(pallet).GetMassAttr().Get() == pytest.approx(20.0)
    colliders = [p for p in pallet.GetChildren() if p.HasAPI(UsdPhysics.CollisionAPI)]
    assert {p.GetName() for p in colliders} == {"deck", "block0", "block1"}
    # Members stand where the scene put them, through the body's frame.
    deck = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Env/pallet/deck"))
    assert tuple(deck.ComputeLocalToWorldTransform(0).ExtractTranslation()) == pytest.approx(
        (2.5, 0.0, 0.13)
    )

    # Equipment collides without a body.
    top = stage.GetPrimAtPath("/World/Env/bench/top")
    assert top.HasAPI(UsdPhysics.CollisionAPI) and not top.HasAPI(UsdPhysics.RigidBodyAPI)
    assert not stage.GetPrimAtPath("/World/Env/bench").HasAPI(UsdPhysics.RigidBodyAPI)

    # The belt: what the conveyor's zone reaches, as a kinematic body that
    # carries along its own frame; the leg under it stays scenery.
    bed = stage.GetPrimAtPath("/World/Env/line/bed")
    assert bed.GetAttribute("physics:kinematicEnabled").Get() is True
    # (Stock USD does not know PhysX's schemas, so it lists them only in
    # the raw metadata — and carries them through untouched.)
    assert "PhysxSurfaceVelocityAPI" in bed.GetMetadata("apiSchemas").GetAddedOrExplicitItems()
    assert tuple(bed.GetAttribute("physxSurfaceVelocity:surfaceVelocity").Get()) == pytest.approx(
        (0.2, 0.0, 0.0)
    )
    assert not stage.GetPrimAtPath("/World/Env/line/leg").HasAPI(UsdPhysics.RigidBodyAPI)

    # The ground of the world scope.
    ground = stage.GetPrimAtPath("/World/Ground")
    assert ground.GetTypeName() == "Plane" and ground.HasAPI(UsdPhysics.CollisionAPI)


def test_a_urdf_robot_is_an_articulation_in_its_pose(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdPhysics

    scene = _cell()
    out = tmp_path / "cell.usda"
    scene.export_usd(out, physics=bt.Physics(world=True))
    stage = _stage(out)

    robot = stage.GetPrimAtPath("/World/Robot")
    assert robot.HasAPI(UsdPhysics.ArticulationRootAPI)
    assert robot.GetAttribute("physxArticulation:enabledSelfCollisions").Get() is False
    links = [p for p in robot.GetChildren() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    assert len(links) == len(scene.robot.link_names)
    # This URDF declares no collision geometry: a link collides as it looks.
    assert any(
        c.HasAPI(UsdPhysics.CollisionAPI) for link in links for c in link.GetChildren()
    )

    joints = {p.GetName(): p for p in stage.GetPrimAtPath("/World/Robot/joints").GetChildren()}
    assert joints["root_joint"].GetTypeName() == "PhysicsFixedJoint"  # bolted to its stand
    names = scene.robot.joint_names
    for name, q in zip(names, READY):
        joint = joints[name]
        assert joint.GetTypeName() == "PhysicsRevoluteJoint"
        assert joint.HasAPI(UsdPhysics.DriveAPI, "angular")
        # PhysX starts an articulation from the stated joint positions, not
        # from where the links stand: both say the same pose.
        state = joint.GetAttribute("state:angular:physics:position").Get()
        target = joint.GetAttribute("drive:angular:physics:targetPosition").Get()
        assert state == pytest.approx(math.degrees(q), abs=1e-3)
        assert target == pytest.approx(math.degrees(q), abs=1e-3)
        assert joint.GetAttribute("drive:angular:physics:stiffness").Get() > 0.0

    # Unpowered: the drives keep only the passive drag.
    limp = tmp_path / "limp.usda"
    scene.export_usd(limp, physics=bt.Physics(world=True, powered=False))
    limp_stage = _stage(limp)  # held: a prim does not keep its stage alive
    joint = limp_stage.GetPrimAtPath(f"/World/Robot/joints/{names[0]}")
    assert joint.GetAttribute("drive:angular:physics:stiffness").Get() == 0.0
    assert joint.GetAttribute("drive:angular:physics:damping").Get() > 0.0

    # A free base (a walker, an aircraft) has no joint to the world.
    scene.set_robot_physics(floating=True)
    free = tmp_path / "free.usda"
    scene.export_usd(free, physics=bt.Physics(world=True))
    free_stage = _stage(free)
    assert not free_stage.GetPrimAtPath("/World/Robot/joints/root_joint").IsValid()
    assert free_stage.GetPrimAtPath(f"/World/Robot/joints/{names[0]}").IsValid()
    scene.set_robot_physics(floating=False)

    # And it reads back as the same machine, standing where it stood.
    back = bt.Scene(bt.Robot.from_usd(out, articulation_root="/World/Robot"))
    assert back.robot.dof == scene.robot.dof
    for q in (READY, [0.4, -0.2, 0.3, 0.5, -0.4, 0.2]):
        expect, _ = scene.link_pose_at(scene.robot.tcp_link, q)
        got, _ = back.link_pose_at(back.robot.tcp_link, q)
        base, _ = scene.link_pose_at(scene.robot.link_names[0], q)
        back_base, _ = back.link_pose_at(back.robot.link_names[0], q)
        relative = lambda p, o: tuple(a - b for a, b in zip(p, o))
        assert relative(got, back_base) == pytest.approx(relative(expect, base), abs=1e-5)


def test_the_declared_scope_exports_what_was_declared(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdPhysics

    scene = _cell()
    scene.set_physics("riding", dynamic=True, mass=0.7, friction=0.8)
    out = tmp_path / "declared.usda"
    scene.export_usd(out, physics=True)
    stage = _stage(out)
    bodies = {
        str(p.GetPath())
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.RigidBodyAPI) and str(p.GetPath()).startswith("/World/Env")
    }
    # Only what `set_physics` marked is the engine's (and the belt, which a
    # device drives); the pallet nobody declared is scenery that collides.
    assert bodies == {"/World/Env/riding", "/World/Env/line/bed"}
    assert UsdPhysics.MassAPI(stage.GetPrimAtPath("/World/Env/riding")).GetMassAttr().Get() == (
        pytest.approx(0.7)
    )
    deck = stage.GetPrimAtPath("/World/Env/pallet/deck")
    assert deck.HasAPI(UsdPhysics.CollisionAPI)
    assert stage.GetPrimAtPath("/World/Ground").IsValid() is False  # no ground declared
    assert stage.GetPrimAtPath("/World/Robot").HasAPI(UsdPhysics.ArticulationRootAPI)


def test_what_a_device_moves_is_one_kinematic_body(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdGeom, UsdPhysics

    # A machine door on a linear axis and an AGV: botrail moves both by
    # name. To the engine each is *one* kinematic body — it never pushes
    # them, the consumer's own controller poses them (one call per device,
    # not one per cover and wheel) — under either scope.
    for k, physics in enumerate((True, bt.Physics(world=True))):
        scene = bt.Scene()
        scene.add_box("machine/door", (0.05, 0.8, 1.0), (1.0, 0.0, 0.5))
        scene.add_box("machine/door/handle", (0.04, 0.1, 0.04), (0.96, 0.0, 0.6))
        scene.add_box("machine/frame", (0.1, 1.0, 1.2), (1.2, 0.0, 0.6))
        # Equipment by its pin: bolted down under the world scope too (an
        # unpinned box on the floor would be a loose body there).
        scene.set_part("machine/frame", category="structure.frame", model="machine frame (shape example)")
        scene.add_linear_axis("door", ["machine/door", "machine/door/handle"], (0.0, 1.0, 0.0), 0.4, (0.0, 0.8))
        scene.add_box("agv/chassis", (0.8, 0.5, 0.25), (-1.0, 0.0, 0.15))
        scene.add_box("agv/cover", (0.7, 0.4, 0.02), (-1.0, 0.0, 0.29))
        scene.set_obstacle_enabled("agv/cover", False)  # a picture that rides along
        scene.add_vehicle("agv", ["agv/chassis", "agv/cover"], [(-1.0, 0.0), (-1.0, 2.0)], {"home": 0, "dock": 1})
        out = tmp_path / f"devices_{k}.usda"  # pxr caches a layer by its path
        assert scene.export_usd(out, physics=physics) == []
        stage = _stage(out)
        kinematic = {
            str(p.GetPath())
            for p in stage.Traverse()
            if p.HasAPI(UsdPhysics.RigidBodyAPI)
            and p.GetAttribute("physics:kinematicEnabled").Get()
        }
        # Named by what the pieces' names share: the door *is* its body,
        # the AGV's two pieces meet under `agv`.
        assert kinematic == {"/World/Env/machine/door", "/World/Env/agv"}
        assert stage.GetPrimAtPath("/World/Env/machine/door/geom").HasAPI(UsdPhysics.CollisionAPI)
        assert stage.GetPrimAtPath("/World/Env/machine/door/handle").HasAPI(UsdPhysics.CollisionAPI)
        assert stage.GetPrimAtPath("/World/Env/agv/chassis").HasAPI(UsdPhysics.CollisionAPI)
        cover = stage.GetPrimAtPath("/World/Env/agv/cover")
        assert cover.IsValid() and not cover.HasAPI(UsdPhysics.CollisionAPI)
        # The machine's frame is nobody's: scenery, where it always was.
        frame = stage.GetPrimAtPath("/World/Env/machine/frame")
        assert frame.HasAPI(UsdPhysics.CollisionAPI) and not frame.HasAPI(UsdPhysics.RigidBodyAPI)
        handle = UsdGeom.Xformable(stage.GetPrimAtPath("/World/Env/machine/door/handle"))
        assert tuple(handle.ComputeLocalToWorldTransform(0).ExtractTranslation()) == pytest.approx(
            (0.96, 0.0, 0.6)
        )


def _mobile_arm() -> bt.Scene:
    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))
    scene.add_box("agv/base", (0.8, 0.6, 0.3), (0.0, 2.0, 0.15))
    scene.add_box("agv/cover", (0.7, 0.5, 0.02), (0.0, 2.0, 0.31))
    scene.add_vehicle("amr", body=["agv"], path=[(0.0, 2.0), (2.0, 2.0)], stations={"a": 0, "b": 1})
    scene.mount_robot("amr", offset_position=(0.1, 0.0, 0.32))
    scene.set_joint_positions(READY)
    return scene


def test_a_robot_riding_a_vehicle_takes_it_as_its_base(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import Gf, UsdGeom, UsdPhysics

    scene = _mobile_arm()
    out = tmp_path / "mobile.usda"
    assert scene.export_usd(out, physics=bt.Physics(world=True)) == []
    stage = _stage(out)
    joints = "/World/Robot/joints"
    targets = lambda prim, rel: [
        str(t) for t in stage.GetPrimAtPath(prim).GetRelationship(rel).GetTargets()
    ]

    # The vehicle goes where it is told, and the stage says so in the one
    # form an engine takes on every pipeline: its body is a link of the
    # arm's articulation, behind six virtual joints whose positions *are*
    # the vehicle's pose — here, parked at its first station, heading +x.
    # (Bolted to the world the arm stayed behind when the vehicle was
    # posed; bolted to a kinematic body it followed under CPU dynamics
    # only. Both measured in Isaac Lab.)
    expect = {"base_x": 0.0, "base_y": 2.0, "base_z": 0.0, "base_yaw": 0.0, "base_pitch": 0.0, "base_roll": 0.0}
    for name, value in expect.items():
        joint = stage.GetPrimAtPath(f"{joints}/{name}")
        kind = "linear" if name in ("base_x", "base_y", "base_z") else "angular"
        assert joint.GetTypeName() == ("PhysicsPrismaticJoint" if kind == "linear" else "PhysicsRevoluteJoint")
        assert joint.GetAttribute(f"state:{kind}:physics:position").Get() == pytest.approx(value, abs=1e-5)
        assert joint.GetAttribute(f"drive:{kind}:physics:targetPosition").Get() == pytest.approx(value, abs=1e-5)
        assert joint.GetAttribute(f"drive:{kind}:physics:stiffness").Get() > 0.0
    # A fixed base, anchored to the world before the first virtual joint.
    assert stage.GetPrimAtPath("/World/Robot").HasAPI(UsdPhysics.ArticulationRootAPI)
    assert targets(f"{joints}/root_joint", "physics:body0") == []
    assert targets(f"{joints}/root_joint", "physics:body1") == ["/World/Robot/carrier_anchor"]
    # The chain ends on the vehicle's body, a link the solver moves...
    body = stage.GetPrimAtPath("/World/Env/agv")
    assert targets(f"{joints}/base_roll", "physics:body1") == ["/World/Env/agv"]
    assert body.HasAPI(UsdPhysics.RigidBodyAPI)
    assert not body.GetAttribute("physics:kinematicEnabled").Get()
    # ...and the arm is bolted to that body where it sits on the deck.
    mount = stage.GetPrimAtPath(f"{joints}/mount_joint")
    assert targets(f"{joints}/mount_joint", "physics:body0") == ["/World/Env/agv"]
    assert targets(f"{joints}/mount_joint", "physics:body1") == ["/World/Robot/base_link"]
    body_world = UsdGeom.Xformable(body).ComputeLocalToWorldTransform(0)
    local = Gf.Vec3d(*mount.GetAttribute("physics:localPos0").Get())
    base, _ = scene.robot_base_pose
    assert tuple(body_world.Transform(local)) == pytest.approx(base, abs=1e-6)
    text = out.read_text()
    assert "excludeFromArticulation" not in text and "filteredPairs" not in text

    # A vehicle nobody rides stays a kinematic body, posed from outside.
    scene.add_box("cart/deck", (0.6, 0.4, 0.2), (3.0, -2.0, 0.1))
    scene.add_vehicle("cart", body=["cart"], path=[(3.0, -2.0), (5.0, -2.0)], stations={"a": 0, "b": 1})
    fleet = tmp_path / "fleet.usda"  # (pxr caches a layer by its path: a new file, not a rewrite)
    scene.export_usd(fleet, physics=bt.Physics(world=True))
    fleet_stage = _stage(fleet)
    cart = fleet_stage.GetPrimAtPath("/World/Env/cart/deck")
    assert cart.GetAttribute("physics:kinematicEnabled").Get() is True

    # A machine whose base floats (a walker, an aircraft) has no joint to
    # anything, and its vehicle — the envelope planning draws around it —
    # is not matter it could stand in: a picture, never a collider.
    scene.set_robot_physics(floating=True)
    free = tmp_path / "free.usda"
    scene.export_usd(free, physics=bt.Physics(world=True))
    free_stage = _stage(free)
    assert not free_stage.GetPrimAtPath("/World/Robot/joints/root_joint").IsValid()
    assert not free_stage.GetPrimAtPath("/World/Robot/joints/base_x").IsValid()
    assert free_stage.GetPrimAtPath("/World/Robot/base_link").HasAPI(UsdPhysics.ArticulationRootAPI)
    envelope = free_stage.GetPrimAtPath("/World/Env/agv/base")
    assert envelope.IsValid() and not envelope.HasAPI(UsdPhysics.CollisionAPI)
    assert not free_stage.GetPrimAtPath("/World/Env/agv").HasAPI(UsdPhysics.RigidBodyAPI)


def test_a_second_robot_on_the_vehicle_joins_the_articulation(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdPhysics

    scene = bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"), name="left")
    scene.add_robot(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"), name="right")
    scene.add_box("agv/base", (1.0, 0.8, 0.3), (0.0, 2.0, 0.15))
    scene.add_vehicle("amr", body=["agv"], path=[(0.0, 2.0), (2.0, 2.0)], stations={"a": 0, "b": 1})
    scene.mount_robot("amr", robot="left", offset_position=(0.1, 0.25, 0.3))
    scene.mount_robot("amr", robot="right", offset_position=(0.1, -0.25, 0.3))
    for name in ("left", "right"):
        scene.set_joint_positions(READY, robot=name)
    out = tmp_path / "crew.usda"
    assert scene.export_usd(out, physics=bt.Physics(world=True)) == []
    stage = _stage(out)

    # Two arms on one cart are one machine to the engine: one articulation,
    # rooted at the first rider, whose chain carries the vehicle; the
    # second is bolted to the vehicle's body by a mount joint alone.
    roots = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    assert roots == ["/World/left"]
    assert stage.GetPrimAtPath("/World/left/joints/base_x").IsValid()
    assert not stage.GetPrimAtPath("/World/right/joints/base_x").IsValid()
    assert not stage.GetPrimAtPath("/World/right/joints/root_joint").IsValid()
    mount = stage.GetPrimAtPath("/World/right/joints/right_mount_joint")
    assert [str(t) for t in mount.GetRelationship("physics:body0").GetTargets()] == ["/World/Env/agv/base"]
    assert [str(t) for t in mount.GetRelationship("physics:body1").GetTargets()] == ["/World/right/right_base_link"]
    # Named apart — two arms of one model would otherwise clash, and PhysX
    # would rename the second's joints `shoulder_pan_0`.
    for robot in ("left", "right"):
        joints = [c.GetName() for c in stage.GetPrimAtPath(f"/World/{robot}/joints").GetChildren()]
        assert f"{robot}_shoulder_pan" in joints, joints
        assert stage.GetPrimAtPath(f"/World/{robot}/{robot}_tool0").IsValid()


@pytest.mark.skipif(not FRANKA.exists(), reason="needs the cached Isaac Franka asset")
def test_a_usd_sourced_robot_rides_its_vehicle(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdPhysics

    scene = bt.Scene(bt.Robot.from_usd(FRANKA))
    scene.add_box("agv/base", (0.8, 0.6, 0.3), (0.0, 2.0, 0.15))
    scene.add_vehicle("amr", body=["agv"], path=[(0.0, 2.0), (2.0, 2.0)], stations={"a": 0, "b": 1})
    scene.mount_robot("amr", offset_position=(0.1, 0.0, 0.3))
    scene.set_joint_positions([0.0, -0.6, 0.0, -2.2, 0.0, 1.8, 0.8, 0.03, 0.03])
    out = tmp_path / "franka_agv.usda"
    assert scene.export_usd(out, physics=bt.Physics(world=True)) == []
    stage = _stage(out)

    # The asset brings its own physics, its anchoring to the world included:
    # that joint is deactivated by an `over`, and the chain hangs under the
    # referenced prim, whose articulation root the asset applies.
    assert not stage.GetPrimAtPath("/World/Robot/rootJoint").IsActive()  # deactivated: not simulated
    assert stage.GetPrimAtPath("/World/Robot").HasAPI(UsdPhysics.ArticulationRootAPI)
    joints = stage.GetPrimAtPath("/World/Robot/joints")
    assert [c.GetName() for c in joints.GetChildren()] == [
        "root_joint", "base_x", "base_y", "base_z", "base_yaw", "base_pitch", "base_roll", "mount_joint",
    ]
    mount = joints.GetChild("mount_joint")
    assert [str(t) for t in mount.GetRelationship("physics:body1").GetTargets()] == ["/World/Robot/panda_link0"]
    assert tuple(mount.GetAttribute("physics:localPos0").Get()) == pytest.approx((0.1, 0.0, 0.15), abs=1e-6)
    assert joints.GetChild("base_y").GetAttribute("state:linear:physics:position").Get() == pytest.approx(2.0)


def test_a_referenced_robot_in_other_units_rides_beside_its_prim(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdPhysics

    asset = tmp_path / "arm.usda"
    asset.write_text(YUP_ARM)
    scene = bt.Scene(bt.Robot.from_usd(asset))
    scene.add_box("agv/base", (0.8, 0.6, 0.3), (0.0, 2.0, 0.15))
    scene.add_vehicle("amr", body=["agv"], path=[(0.0, 2.0), (2.0, 2.0)], stations={"a": 0, "b": 1})
    scene.mount_robot("amr", offset_position=(0.1, 0.0, 0.3))
    scene.set_joint_positions([0.4])
    out = tmp_path / "yup_ride.usda"
    assert scene.export_usd(out, physics=bt.Physics(world=True)) == []
    stage = _stage(out)

    # The stage composes through a unit correction (centimetres, Y-up), so
    # the chain — world-frame poses — stands beside the robot prim, not
    # under it; membership is by connection, so it is the same articulation.
    assert not stage.GetPrimAtPath("/World/Robot/joints/anchor").IsActive()
    chain = stage.GetPrimAtPath("/World/Robot_carrier/joints")
    assert chain.IsValid() and chain.GetChild("base_roll").IsValid()
    mount = chain.GetChild("mount_joint")
    assert [str(t) for t in mount.GetRelationship("physics:body1").GetTargets()] == ["/World/Robot/base"]
    assert tuple(mount.GetAttribute("physics:localPos0").Get()) == pytest.approx((0.1, 0.0, 0.15), abs=1e-5)
    assert stage.GetPrimAtPath("/World/Robot").HasAPI(UsdPhysics.ArticulationRootAPI)


def test_collision_follows_enabled_and_the_picture_follows_visible(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdGeom, UsdPhysics

    scene = bt.Scene()
    scene.add_box("shell", (0.5, 0.5, 0.5), (0.0, 0.0, 0.25))
    scene.set_obstacle_enabled("shell", False)  # drawn, never collides
    scene.add_box("proxy", (0.5, 0.5, 0.5), (0.0, 0.0, 0.25))
    scene.set_obstacle_visible("proxy", False)  # collides, never drawn
    scene.set_part("shell", category="structure.stand", model="stand (shape example)")
    scene.set_part("proxy", category="structure.stand", model="stand (shape example)")

    plain = tmp_path / "plain.usda"
    scene.export_usd(plain)
    assert not _stage(plain).GetPrimAtPath("/World/Env/proxy").IsValid()  # the picture only

    out = tmp_path / "sim.usda"
    scene.export_usd(out, physics=bt.Physics(world=True))
    stage = _stage(out)
    shell = stage.GetPrimAtPath("/World/Env/shell")
    proxy = stage.GetPrimAtPath("/World/Env/proxy")
    assert shell.IsValid() and not shell.HasAPI(UsdPhysics.CollisionAPI)
    assert proxy.HasAPI(UsdPhysics.CollisionAPI)
    assert UsdGeom.Imageable(proxy).GetPurposeAttr().Get() == "guide"


def test_a_mesh_drawn_part_collides_as_its_primitive(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    from pxr import UsdPhysics

    scene = bt.Scene()
    scene.add_box("floor", (2.0, 2.0, 0.1), (0.0, 0.0, -0.05))
    bt.parts.tray(scene, "tray", (0.4, 0.3, 0.06), (0.0, 0.0, 0.0))
    out = tmp_path / "tray.usda"
    scene.export_usd(out, physics=bt.Physics(world=True))
    stage = _stage(out)
    # The tray is a box to the planner and to the bake, a pocketed tray to
    # the eye. Colliding as the picture, a part seated on it fell into the
    # pocket (measured in Isaac Sim); the box rides beside the picture.
    body = stage.GetPrimAtPath("/World/Env/tray")
    assert body.HasAPI(UsdPhysics.RigidBodyAPI)
    picture = stage.GetPrimAtPath("/World/Env/tray/geom")
    collider = stage.GetPrimAtPath("/World/Env/tray/collider")
    assert picture.GetTypeName() == "Mesh" and not picture.HasAPI(UsdPhysics.CollisionAPI)
    assert collider.GetTypeName() == "Cube" and collider.HasAPI(UsdPhysics.CollisionAPI)
    assert collider.GetAttribute("purpose").Get() == "guide"


def test_a_simulation_stage_has_no_animation(tmp_path: Path) -> None:
    scene = _cell()
    traj = scene.plan([0.2, 0.5, 0.7, 0.0, 0.4, 0.0])
    with pytest.raises(ValueError, match="simulation stage"):
        scene.export_usd(tmp_path / "both.usda", traj, physics=True)
    # `physics=False` is today's export, unchanged.
    assert scene.export_usd(tmp_path / "plain.usda", physics=False) == []
    assert "PhysicsArticulationRootAPI" not in (tmp_path / "plain.usda").read_text()


def test_the_plain_stage_of_a_plain_cell_validates(tmp_path: Path) -> None:
    pytest.importorskip("pxr")
    try:
        from pxr import UsdValidation
    except ImportError:
        pytest.skip("this pxr has no UsdValidation")

    out = tmp_path / "cell.usda"
    _cell().export_usd(out, physics=bt.Physics(world=True))
    context = UsdValidation.ValidationContext(
        UsdValidation.ValidationRegistry().GetOrLoadAllValidators()
    )
    errors = context.Validate(_stage(out))
    assert [e.GetMessage() for e in errors] == []


def test_a_crate_file_is_stamped_for_older_readers(tmp_path: Path) -> None:
    # A reader refuses a crate newer than itself whatever is inside; Isaac
    # Sim 5 (USD 24) stops at 0.11. Nothing botrail writes needs more than
    # 0.8, so that is what the file says.
    out = tmp_path / "cell.usdc"
    _cell().export_usd(out, physics=bt.Physics(world=True))
    head = out.read_bytes()[:11]
    assert head[:8] == b"PXR-USDC" and tuple(head[8:]) == (0, 8, 0)
    stage = _stage(out)
    assert stage.GetPrimAtPath("/World/Robot").IsValid()
