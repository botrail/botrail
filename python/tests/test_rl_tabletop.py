"""The tabletop cell and its helpers (design-rl-tabletop.md T0): one cell
function as the learning environment and as a grid; settling ticks; the
pictures' default decimation; a `VecEnv` under `bt.Physics` (G11); an
external drive powering a world-scope robot (G12); a dynamic arm that
stops on what it presses; the randomisation helpers."""

import json
import math
import sys
from pathlib import Path

import botrail as bt
import numpy as np
import pytest
from botrail import rl

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
sys.path.insert(0, str(EXAMPLES / "rl"))

import tabletop_env as demo
from test_rl import READY as ARM_READY
from test_rl import build as arm_build

# ------------------------------------------------------------ the cell


def test_single_and_vector_worlds_match_with_a_dynamic_arm() -> None:
    """The cell as `rl.single` builds it, the arm dynamic: a vector world
    and the single environment agree to the bit under the same seed —
    physics, servos, settling ticks and the pool draw included."""
    cell = demo.make_cell(objects="shapes")
    task = demo.make_task()
    single = rl.make(rl.single(cell), task, seed=0)
    venv = rl.make(rl.single(cell), task, seed=0, num_envs=2)
    assert single.scene.robot_physics("fr3")
    assert "can" in single.scene.obstacle_names and "floor" in single.scene.obstacle_names
    # Nothing is born inside anything else: the pool's spots are spaced
    # for the widest footprint (the engine kicks overlapping things apart).
    assert single.scene.physics_plan(physics=bt.Physics(world=True)).overlaps == []
    sobs, sinfo = single.reset(seed=3)
    vobs, vinfo = venv.reset(seed=3)
    assert sinfo["t"] == pytest.approx(0.3) and vinfo["envs"][0]["t"] == pytest.approx(0.3)
    assert np.array_equal(sobs, vobs[0])
    for _ in range(4):
        a = demo.scripted(sobs)
        sobs, sr, st, stc, sinfo = single.step(a)
        vobs, vr, vt, vtc, vinfo = venv.step(np.stack([a, demo.scripted(vobs[1])]))
        assert np.array_equal(sobs, vobs[0])
        assert sr == vr[0] and st == vt[0] and stc == vtc[0]
    # The other world drew a different pool: not every thing is on the table.
    enabled = [n for n in demo.OTHERS if venv.scenes[1].obstacle_enabled(n)]
    assert len(enabled) == 4


def test_the_dynamic_arm_stops_on_the_mug_it_presses() -> None:
    """The acceptance of T0 (design-rl-tabletop.md §10.3): commanded 30 cm
    through a mug on the table, a kinematic mirror pushes the mug through
    the table top and reaches the target; the dynamic arm stops on it
    and the mug stays where it stood."""
    task = demo.make_task(horizon_s=5.0)
    env = rl.make(rl.single(demo.make_cell(objects="shapes")), task, seed=0)
    obs, _ = env.reset()
    (bx, by, bz), _ = env.live.object_pose("mug")
    top = bz + 0.04
    assert bz == pytest.approx(demo.rest_z("mug"), abs=0.01)
    for step in range(90):
        # Hover over the mug, then command the hand down to its centre.
        target = np.array([bx, by, top + 0.12 if step < 40 else bz])
        obs, *_ = env.step(demo.step_toward(obs, target))
    (_, _, cz), _ = env.live.object_pose("mug")
    (_, _, tz), _ = env.live.tcp_pose()
    # The mug may squirt out sideways from under the open fingers (its rim
    # is wider than their gap) — what it cannot do is go into the table.
    assert abs(cz - bz) < 0.01, f"the mug was pressed from z={bz:.3f} to z={cz:.3f}"
    assert tz > demo.TOP - 0.005, f"the hand went through the table top (tcp z={tz:.3f})"


def test_pictures_default_to_a_decimated_mesh() -> None:
    """A picture channel decimates a catalogue arm's visual meshes to
    half a pixel's footprint at a metre unless `render` says otherwise —
    `None` keeps every triangle."""
    cell = demo.make_cell(dynamic=False, objects="shapes")
    observe = [rl.Joints(), rl.Depth("wrist", size=(64, 64))]
    auto = rl.make(rl.single(cell), rl.Task(observe=observe, horizon_s=1.0), seed=0)
    assert auto._auto_cell == pytest.approx(np.tan(np.radians(auto.scene.camera_fov("wrist")) / 2) / 64)
    full = rl.make(rl.single(cell), rl.Task(observe=observe, horizon_s=1.0, render={"decimate": None}), seed=0)
    coarse = rl.make(rl.single(cell), rl.Task(observe=observe, horizon_s=1.0, render={"decimate": 0.05}), seed=0)
    auto.reset()
    full.reset()
    coarse.reset()
    n_auto, n_full, n_coarse = (env.live.render_triangles() for env in (auto, full, coarse))
    assert n_coarse < n_auto < n_full, (n_coarse, n_auto, n_full)
    with pytest.raises(ValueError):
        rl.make(rl.single(cell), rl.Task(observe=observe, render={"decimate": -1.0}), seed=0).reset()


def test_a_vector_env_takes_physics_options() -> None:
    """G11: `Task.physics` may be a `bt.Physics` — the world scope, powered
    — in a `VecEnv` as in the single one, and the two still agree."""
    cell = demo.make_cell(objects="shapes")
    task = demo.make_task()
    task.physics = bt.Physics(world=True, powered=True)
    single = rl.make(rl.single(cell), task, seed=0)
    venv = rl.make(rl.single(cell), task, seed=0, num_envs=2)
    sobs, _ = single.reset(seed=1)
    vobs, _ = venv.reset(seed=1)
    assert np.array_equal(sobs, vobs[0])
    for _ in range(3):
        a = demo.scripted(sobs)
        sobs, *_ = single.step(a)
        vobs, *_ = venv.step(np.stack([a, demo.scripted(vobs[1])]))
        assert np.array_equal(sobs, vobs[0])
    # The powered arm went where the controller sent it (three small steps
    # from READY) — an unpowered one would have folded at the elbow.
    q = sobs[:8]
    assert abs(q[3] - demo.READY[3]) < 0.5, q


def test_tile_builds_the_grid_and_play_runs_the_policy_in_every_copy() -> None:
    """`rl.tile` names each copy's residents by its prefix and its robot
    by the cell's rule; `rl.play` binds one control per robot, relocates
    the task's channels into every copy, and closes one timeline in
    which every arm moved toward its own can."""
    cell = demo.make_cell(objects="shapes")
    tiled = rl.tile(cell, 1, 2, spacing=(1.9, 1.6))
    assert tiled.robots == ["c0_fr3", "c1_fr3"]
    assert tiled.cells == [("c0/", "c0_fr3"), ("c1/", "c1_fr3")]
    assert tiled.origins == [(0.0, 0.0), (1.9, 0.0)]
    names = tiled.scene.obstacle_names
    assert "c0/can" in names and "c1/can" in names and "floor" in names
    (x1, _, _), _ = tiled.scene.obstacle_pose("c1/can")
    (x0, _, _), _ = tiled.scene.obstacle_pose("c0/can")
    assert x1 - x0 == pytest.approx(1.9)
    tiled.randomize(demo.randomize, seed=0)
    task = demo.make_task()
    tl = rl.play(tiled, task, demo.scripted, duration_s=1.0)
    # Up to a second past the settling ticks — less once every copy is done.
    assert task.settle_s + 0.1 < tl.duration <= 1.0 + task.settle_s + 1e-6
    assert set(tl.robots) == {"c0_fr3", "c1_fr3"}
    for robot in tiled.robots:
        q0 = np.array(tl.sample(task.settle_s, robot=robot))
        q1 = np.array(tl.sample(tl.duration, robot=robot))
        assert np.abs(q1 - q0).max() > 0.05, f"{robot} did not move"
    with pytest.raises(ValueError):
        rl.tile(lambda scene, prefix, origin: None, 1, 1)


def test_relocation_moves_every_name_into_the_copy() -> None:
    spec = rl.Relative("tcp", "can").lower("fr3")
    assert rl._relocate(spec, "c2/", "fr3", "c2_fr3") == {"kind": "relative", "a": "c2_fr3/tcp", "b": "c2/can"}
    spec = rl.Contacts("fr3/fr3_leftfinger", "can").lower("fr3")
    assert rl._relocate(spec, "c2/", "fr3", "c2_fr3") == {"kind": "contacts", "a": "c2_fr3/fr3_leftfinger", "b": "c2/can"}
    assert rl._relocate(rl.Joints().lower("fr3"), "c2/", "fr3", "c2_fr3")["robot"] == "c2_fr3"
    assert rl._relocate(rl.Depth("wrist").lower("fr3"), "c2/", "fr3", "c2_fr3")["camera"] == "c2/wrist"


# ------------------------------------------------------------ settle, power


def dropped_part_build() -> bt.Scene:
    scene = arm_build()
    scene.set_obstacle_pose("part", (0.55, 0.0, 0.30))  # 16 cm above the table
    return scene


def test_settle_ticks_run_before_the_first_observation() -> None:
    """`settle_s` is ticked off after the reset with no action: the part
    the randomisation dropped is on the table in the first observation,
    the clock reads the settling time, and the horizon counts from there."""
    observe = [rl.Joints(), rl.ObjectPose("part")]
    still = rl.make(dropped_part_build, rl.Task(observe=observe, horizon_s=0.5), seed=0)
    settled = rl.make(dropped_part_build, rl.Task(observe=observe, horizon_s=0.5, settle_s=0.6), seed=0)
    obs0, info0 = still.reset()
    obs1, info1 = settled.reset()
    assert info0["t"] == 0.0 and info1["t"] == pytest.approx(0.6)
    assert obs0[12 + 2] == pytest.approx(0.30, abs=1e-6)
    assert obs1[12 + 2] < 0.16, f"the part did not settle: z={obs1[14]:.3f}"
    n = 0
    while True:
        _, _, terminated, truncated, info = settled.step(np.zeros(settled.action_space.shape))
        n += 1
        if terminated or truncated:
            break
    assert truncated and n == 10 and info["t"] == pytest.approx(1.1)
    venv = rl.make(dropped_part_build, rl.Task(observe=observe, horizon_s=0.5, settle_s=0.6), seed=0, num_envs=2)
    vobs, vinfo = venv.reset()
    assert np.array_equal(vobs[0], obs1)
    assert vinfo["envs"][1]["t"] == pytest.approx(0.6)
    with pytest.raises(ValueError):
        rl.make(dropped_part_build, rl.Task(observe=observe, settle_s=-1.0), seed=0).reset()


def test_an_external_drive_powers_a_world_scope_robot() -> None:
    """G12: under the world scope a robot no program drives is unpowered
    and folds; the environment's drive is a driver, so its arm holds —
    unless the task says `powered=False` outright."""

    def build() -> bt.Scene:
        scene = arm_build()
        scene.set_robot_physics()
        return scene

    # A zero `JointDelta` step holds the drive's target where the arm
    # stands: the servos, if on, keep READY against gravity for a second.
    def held(physics) -> np.ndarray:
        task = rl.Task(control=rl.JointDelta(max_step=0.0, hz=20), observe=[rl.Joints()], horizon_s=1.1, physics=physics)
        env = rl.make(build, task, seed=0)
        obs, _ = env.reset()
        for _ in range(20):
            obs, *_ = env.step(np.zeros(6))
        return obs[:6]

    world = held(bt.Physics(world=True))
    assert np.abs(world - ARM_READY).max() < 0.05, f"the driven arm folded: {world}"
    off = held(bt.Physics(world=True, powered=False))
    assert np.abs(off - ARM_READY).max() > 0.1, f"the unpowered arm held: {off}"
    declared = held(True)
    assert np.abs(declared - ARM_READY).max() < 0.05


# ------------------------------------------------------------ randomisation helpers


def things_build() -> bt.Scene:
    scene = bt.Scene()
    scene.add_box("table", size=(1.0, 1.0, 0.1), position=(0, 0, 0.05))
    for name in ("a", "b", "c", "d"):
        scene.add_box(name, size=(0.05, 0.05, 0.05), position=(0.0, 0.0, 0.125))
        scene.set_physics(name, dynamic=True, mass=0.1)
    return scene


def test_pool_draws_k_things_onto_spots_and_parks_the_rest() -> None:
    scene = things_build()
    spots = [(-0.3, -0.3), (0.3, -0.3), (-0.3, 0.3), (0.3, 0.3)]
    chosen = rl.randomize.pool(scene, np.random.default_rng(1), ["a", "b", "c", "d"], 2, spots, 0.1, jitter=0.0)
    assert len(chosen) == 2 and len(set(chosen)) == 2
    for name in "abcd":
        (x, y, z), _ = scene.obstacle_pose(name)
        if name in chosen:
            # Footprint middle on the spot, underside 2 mm above the face.
            assert scene.obstacle_enabled(name) and (x, y) in spots and z == pytest.approx(0.1 + 0.002 + 0.025)
        else:
            assert not scene.obstacle_enabled(name) and not scene.obstacle_visible(name) and x >= 3.0
    again = things_build()
    assert rl.randomize.pool(again, np.random.default_rng(1), ["a", "b", "c", "d"], 2, spots, 0.1, jitter=0.0) == chosen
    with pytest.raises(ValueError):
        rl.randomize.pool(scene, np.random.default_rng(1), ["a", "b"], 3, spots, 0.1)
    with pytest.raises(ValueError, match="face"):
        rl.randomize.pool(scene, np.random.default_rng(1), ["a", "b"], 1, spots)
    # Without spots the draw only chooses: the drawn stay put, the rest park.
    before = {n: scene.obstacle_pose(n)[0] for n in "abcd"}
    kept = rl.randomize.pool(scene, np.random.default_rng(2), ["a", "b", "c", "d"], 3)
    assert all(scene.obstacle_pose(n)[0] == before[n] and scene.obstacle_enabled(n) for n in kept)


def test_things_are_set_down_by_their_footprint_not_their_origin(tmp_path) -> None:
    """A thing whose frame sits off its footprint (a scan, a mesh modelled
    anywhere) lands with its footprint on the spot and its underside on the
    face — the pool and the scatter read its bounds, not its origin."""
    obj = tmp_path / "off.obj"
    obj.write_text(
        "".join(f"v {x} {y} {z}\n" for z in (0.03, 0.13) for y in (0.10, 0.16) for x in (0.20, 0.24))
        + "f 1 3 2\nf 2 3 4\nf 5 6 7\nf 6 8 7\nf 1 2 5\nf 2 6 5\nf 3 7 4\nf 4 7 8\nf 1 5 3\nf 3 5 7\nf 2 4 6\nf 4 8 6\n"
    )
    scene = things_build()
    scene.add_mesh("off", str(obj), position=(0.0, 0.0, 0.0))
    rl.randomize.pool(scene, np.random.default_rng(0), ["off"], 1, [(0.3, -0.3)], 0.1, jitter=0.0, gap=0.0)
    lo, hi = scene.obstacle_bounds("off")
    assert ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]) == pytest.approx((0.3, -0.3, 0.1), abs=1e-9)
    rl.randomize.scatter(scene, np.random.default_rng(0), ["off"], ((-0.4, -0.2), (0.2, 0.4)), 0.1, lift=0.0)
    lo, hi = scene.obstacle_bounds("off")
    assert -0.4 <= lo[0] and hi[0] <= -0.2 and 0.2 <= lo[1] and hi[1] <= 0.4 and lo[2] == pytest.approx(0.1)


def test_scatter_keeps_things_apart() -> None:
    scene = things_build()
    placed = rl.randomize.scatter(scene, np.random.default_rng(0), list("abcd"), ((-0.2, 0.2), (-0.2, 0.2)), 0.1, gap=0.02)
    assert set(placed) == set("abcd")
    names = list(placed)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            (ax, ay), (bx, by) = placed[a], placed[b]
            assert abs(ax - bx) >= 0.07 - 1e-9 or abs(ay - by) >= 0.07 - 1e-9, (a, b)
        lo, hi = scene.obstacle_bounds(a)
        assert ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2) == pytest.approx(placed[a])
        assert lo[2] == pytest.approx(0.1 + 0.002)
    assert scene.physics_plan(physics=bt.Physics(world=True)).overlaps == []
    with pytest.raises(ValueError, match="does not fit"):
        rl.randomize.scatter(scene, np.random.default_rng(0), ["a"], ((-0.02, 0.02), (-0.02, 0.02)), 0.1)
    with pytest.raises(ValueError, match="do not fit the region"):
        rl.randomize.scatter(scene, np.random.default_rng(0), list("abcd"), ((-0.06, 0.06), (-0.06, 0.06)), 0.1)
    rl.randomize.friction(scene, np.random.default_rng(0), list("abcd"), 0.4, 0.9)


# ------------------------------------------------------------ the YCB objects


def ycb_root():
    """Where the YCB packages and the KLT pack come from: a local catalog
    build (`BOTRAIL_CATALOG_ROOT`), or the published catalog — `None` to skip."""
    import os

    root = os.environ.get("BOTRAIL_CATALOG_ROOT")
    if root:
        built = Path(root)
        return root if (built / "ycb" / "objects").is_dir() and (built / demo.KLT).is_dir() else None
    try:
        index = bt.catalog.index()
        if index.get("ycb/objects/005-tomato-soup-can") is None or index.get(demo.KLT) is None:
            return None
    except Exception:
        return None
    return ""


def tree_size(layer: Path) -> int:
    """The layer plus everything under its sibling `<stem>_assets/`."""
    assets = layer.with_name(layer.stem + "_assets")
    return layer.stat().st_size + sum(p.stat().st_size for p in assets.rglob("*") if p.is_file())


def test_the_tiled_export_writes_each_mesh_once_and_stays_small(tmp_path) -> None:
    """Six copies of the arm and its things cost the file one set of
    meshes (design-rl-tabletop.md G6 / T3): the grid's USD is a few
    megabytes rather than a hundred and seventy, and a USD reader composes
    every link's triangles from the layers beside it."""
    cell = demo.make_cell(objects="shapes")
    single = rl.single(cell)()
    single.sequence("run").step("wait", transition=bt.seq.elapsed(0.2))
    one = tmp_path / "single.usda"
    single.simulate_sequence("run").export_usd(one)
    tiled = rl.tile(cell, 2, 3).scene
    tiled.sequence("run").step("wait", transition=bt.seq.elapsed(0.2))
    grid = tmp_path / "grid.usda"
    tiled.simulate_sequence("run").export_usd(grid)
    layers = {p.name for p in (tmp_path / "grid_assets" / "meshes").iterdir()}
    assert layers == {p.name for p in (tmp_path / "single_assets" / "meshes").iterdir()}
    assert "fr3_link0.usdc" in layers and "tray.usdc" not in layers  # the tray is a library shape, referenced already
    assert tree_size(one) < 10_000_000
    assert tree_size(grid) < 40_000_000
    # The physics stage shares the mechanism: one arm's meshes, not six.
    physics = tmp_path / "physics.usda"
    single.export_usd(physics, physics=bt.Physics(world=True))
    assert tree_size(physics) < 10_000_000

    Usd = pytest.importorskip("pxr.Usd")
    from pxr import UsdGeom

    stage = Usd.Stage.Open(str(grid))
    meshes = [UsdGeom.Mesh(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh) and "fr3" in str(p.GetPath())]
    assert len(meshes) >= 6 * 8
    assert all(len(m.GetPointsAttr().Get()) > 0 for m in meshes)


def test_the_wrist_camera_is_a_catalog_d405_on_a_clip_the_hand_carries() -> None:
    """The picture channels look through a RealSense D405 from the catalog
    — its optics, its body, its BOM line — seated on a printed clip that
    hugs the hand (botrail-assets franka-hand-d405-clip, vendored): drawn
    from the clip's layer on a resident that does not collide, colliding
    as two hidden boxes and the camera's own mesh, all attached to the
    hand, one part on the BOM, clear of the hand, the wrist, the fingers
    and of each other by the planner's clearance."""
    scene = rl.single(demo.make_cell(objects="shapes"))()
    assert scene.attachments == [(f"cam/{p}", "fr3_hand") for p in ("clip", "claw", "plate", "body")]
    by = {o["name"]: o for o in json.loads(scene._project_json())["obstacles"]}
    assert by["cam/clip"]["visual_asset"]["prim_path"] == "/Clip/clip" and not by["cam/clip"]["enabled"]
    assert by["cam/clip"]["visual_asset"]["url"].endswith("franka_hand_d405_clip.usda")
    assert all(by[n]["enabled"] and not by[n]["visible"] for n in ("cam/claw", "cam/plate"))
    rows = {r["names"][0]: r for r in scene.bom().rows}
    assert rows["wrist"]["category"] == "sensor.camera" and rows["wrist"]["model"] == "RealSense D405"
    assert rows["wrist"]["catalog"].startswith("realsense/d400/d405/r1")
    assert rows["cam"]["category"] == "adapter" and rows["cam"]["attributes"]["mass_kg"] == demo.CLIP_MASS
    assert scene.camera_fov("wrist") == pytest.approx(87.0)
    assert scene.check_collisions() == []
    plan = {r["name"]: r for r in scene.physics_plan(physics=bt.Physics(world=True)).rows}
    assert plan["cam"]["kind"] == "dynamic" and plan["cam"]["reason"] == "carried by fr3/fr3_hand"


def test_the_physics_stage_carries_the_cell_for_isaac_lab(tmp_path) -> None:
    """What `examples/export/isaaclab_tabletop.py` writes (design-rl-tabletop.md
    §10.5, T-I): one articulation rooted at the arm with its exported pose
    in `PhysicsJointStateAPI`, a rigid body per thing on the table and one
    for the KLT's five boxes, the bench and the stand as static colliders,
    every mesh once beside the layer — and nothing pxr's validators object
    to."""
    Usd = pytest.importorskip("pxr.Usd")
    from pxr import UsdPhysics

    scene = rl.single(demo.make_cell(objects="shapes"))()
    physics = bt.Physics(world=True)
    plan = {r["name"]: r["kind"] for r in scene.physics_plan(physics).rows}
    assert plan["fr3"] == "robot" and plan["bin"] == "dynamic" and plan["table"] == "fixed"
    out = tmp_path / "tabletop.usda"
    assert scene.export_usd(out, physics=physics) == []
    assert tree_size(out) < 10_000_000
    stage = Usd.Stage.Open(str(out))
    assert stage.GetPrimAtPath("/World/Robot").HasAPI(UsdPhysics.ArticulationRootAPI)
    posed = {}
    for prim in stage.Traverse():
        state = prim.GetAttribute("state:angular:physics:position") or prim.GetAttribute("state:linear:physics:position")
        if state and state.HasAuthoredValue():
            posed[prim.GetName()] = state.Get()
    assert posed["fr3_joint4"] == pytest.approx(math.degrees(demo.READY[3]), abs=0.01)
    assert posed["fr3_finger_joint1"] == pytest.approx(demo.READY[7], abs=1e-6)
    assert posed["fr3_finger_joint2"] == pytest.approx(demo.READY[7], abs=1e-6)  # the mimic follower, derived
    bodies = {str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI) and "/Env/" in str(p.GetPath())}
    assert {f"/World/Env/{n}" for n in demo.NAMES} | {"/World/Env/bin", "/World/Env/tray", "/World/Env/cam"} <= bodies
    # The camera bracket rides the hand: welded to its link.
    weld = UsdPhysics.FixedJoint(stage.GetPrimAtPath("/World/Env/cam/weld"))
    assert weld and [str(p) for p in weld.GetBody0Rel().GetTargets()] == ["/World/Robot/fr3_hand"]
    assert not any(p.startswith("/World/Env/table") or p.startswith("/World/Env/stand") for p in bodies)
    assert stage.GetPrimAtPath("/World/Env/table/top").HasAPI(UsdPhysics.CollisionAPI)
    try:
        from pxr import UsdValidation
    except ImportError:
        return
    context = UsdValidation.ValidationContext(UsdValidation.ValidationRegistry().GetOrLoadAllValidators())
    assert [e.GetMessage() for e in context.Validate(stage)] == []


def test_the_cell_stands_the_ycb_objects_on_the_table() -> None:
    """The catalog's scans placed with `bt.parts.prop`: re-centred on their
    footprints, so the can's observed pose is the middle of the can on the
    table; every object rests upright, identified on the BOM; an episode's
    scatter keeps their real footprints apart; the world bakes."""
    root = ycb_root()
    if root is None:
        pytest.skip("the YCB packages and the KLT pack are neither built locally (BOTRAIL_CATALOG_ROOT) nor published")
    cell = demo.make_cell(objects="ycb", catalog_root=root or None)
    env = rl.make(rl.single(cell), demo.make_task(), seed=0)
    scene = env.scene
    assert scene.physics_plan(physics=bt.Physics(world=True)).overlaps == []
    rows = {r["names"][0]: r for r in scene.bom().rows}
    assert rows["can"]["model"] == "Tomato Soup Can (YCB 005)" and rows["can"]["attributes"]["mass_kg"] == 0.349
    for name in demo.NAMES:
        (x, y, z), _ = scene.obstacle_pose(name)
        lo, hi = scene.obstacle_bounds(name)
        # The package frame is the footprint's middle on the underside.
        assert ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]) == pytest.approx((x, y, z), abs=1e-6), name
    hovering = []
    for seed in range(3):
        obs, info = env.reset(seed=seed)
        assert scene.physics_plan(physics=bt.Physics(world=True)).overlaps == []
        can = obs[demo.CAN]
        assert abs(can[2] - demo.TOP) < 0.005  # its underside on the table after settling (contact slop)
        for _ in range(80):
            obs, _, terminated, truncated, _ = env.step(demo.scripted(obs))
            if terminated or truncated:
                break
        hovering.append(bool(terminated))
    # The scripted controller reaches the hover over the scanned can.
    assert sum(hovering) >= 2, hovering
    tl = env.timeline()
    assert tl.duration > 0.3
