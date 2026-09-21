"""Robot.from_catalog: loading packages from the model catalog.

The network layer is huggingface_hub, faked out here with a local fixture
tree so the tests pin the orchestration: revision pinning, id resolution,
distribution gating, manifest TCP, and project/script determinism.
"""

import json
import sys
import types
from pathlib import Path

import pytest

import botrail as bt

ARM_ID = "acme/arm/mini/r1"
ARM_R2 = "acme/arm/mini/r2"
MAXI_ID = "acme/arm/maxi/r1"
COUPLING_ID = "acme/coupling/plate/r1"
LOCKED_ID = "acme/arm/locked/r1"
DUAL_ID = "acme/pair/dual/r1"
DUAL_URDF = (Path(__file__).resolve().parents[2] / "examples" / "assets" / "dual_arm_test.urdf").read_text()
SEMI_ID = "acme/semi/wheeled/r1"
SEMI_URDF = (Path(__file__).resolve().parents[2] / "examples" / "assets" / "semi_humanoid_test.urdf").read_text()

SHA = "0123abcd0123abcd0123abcd0123abcd0123abcd"

ARM_URDF = """
<robot name="mini">
  <link name="base_link"/>
  <link name="link1"/>
  <link name="tool_tip"/>
  <joint name="j1" type="revolute">
    <parent link="base_link"/><child link="link1"/>
    <origin xyz="0 0 0.2"/><axis xyz="0 0 1"/>
    <limit lower="-3.14" upper="3.14" effort="10" velocity="1"/>
  </joint>
  <joint name="tip" type="fixed">
    <parent link="link1"/><child link="tool_tip"/>
    <origin xyz="0 0 0.1"/>
  </joint>
</robot>
"""

COUPLING_USD = """#usda 1.0
(
    defaultPrim = "Plate"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Plate" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "body" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom" { double size = 0.05 }
    }
}
"""


def _manifest(
    pid: str,
    tcp: str | None = None,
    mount: str | None = None,
    distribution: str = "public",
    identity: bool = False,
) -> str:
    frames = f"  tcp_default: {tcp}\n" if tcp else ""
    if mount:
        frames += f"  mount_frame: {mount}\n"
    # The identity block a real manifest carries (maker, product name,
    # category, specs) — what the BOM names the machine by. Mixed spec
    # types on purpose: only numbers become BOM attributes.
    ident = (
        "name: Mini Arm\nmanufacturer:\n  name: ACME Robotics\n  country: JP\n"
        "category: manipulator\nspecs:\n  dof: 1\n  payload_kg: 3.5\n  reach_mm: 300\n"
        "  controller: [MC-1]\n  ip_rating: IP54\n"
        if identity
        else ""
    )
    return (
        f"schema_version: '0.1'\nid: {pid}\ndistribution: {distribution}\n{ident}"
        f"frames:\n  flange_frame: link1\n{frames}"
    )


@pytest.fixture()
def catalog(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> dict:
    repo = tmp_path / "dataset"
    products = []

    def add(pid: str, distribution: str, files: dict[str, str], assets: dict) -> None:
        pkg = repo / pid
        for rel, text in files.items():
            path = pkg / rel
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(text)
        products.append(
            {
                "id": pid,
                "category": "manipulator",
                "name": pid,
                "manufacturer": "ACME",
                "specs": {},
                "validation_level": "V2",
                "distribution": distribution,
                "assets": {k: (f"{pid}/{v}" if v else None) for k, v in assets.items()},
            }
        )

    add(
        ARM_ID,
        "public",
        {
            "manifest.yaml": _manifest(ARM_ID, tcp="tool_tip", identity=True),
            "urdf/model.urdf": ARM_URDF,
        },
        {"urdf": "urdf/model.urdf", "usd": None},
    )
    # Another public revision of the same product.
    add(
        ARM_R2,
        "public",
        {"manifest.yaml": _manifest(ARM_R2, tcp="tool_tip"), "urdf/model.urdf": ARM_URDF},
        {"urdf": "urdf/model.urdf", "usd": None},
    )
    add(
        MAXI_ID,
        "public",
        {"manifest.yaml": _manifest(MAXI_ID), "urdf/model.urdf": ARM_URDF},
        {"urdf": "urdf/model.urdf", "usd": None},
    )
    add(
        COUPLING_ID,
        "public",
        {
            "manifest.yaml": _manifest(COUPLING_ID, mount="body"),
            "usd/model.usda": COUPLING_USD,
        },
        {"urdf": None, "usd": "usd/model.usda"},
    )
    add(
        LOCKED_ID,
        "recipe_only",
        {"manifest.yaml": _manifest(LOCKED_ID, distribution="recipe_only")},
        {"urdf": None, "usd": None},
    )
    # A dual-arm product: the manifest names its arms, botrail reads them
    # as planning groups.
    add(
        DUAL_ID,
        "public",
        {
            "manifest.yaml": (
                f"schema_version: '0.1'\nid: {DUAL_ID}\ndistribution: public\n"
                "name: Pair\nmanufacturer:\n  name: ACME Robotics\ncategory: manipulator.dual_arm\n"
                "specs:\n  dof: 8\n  arm_count: 2\n  payload_kg: 2.0\n"
                "frames:\n  base_frame: body\n  arms:\n"
                "    - name: left\n      base_frame: left_base\n      flange_frame: left_hand\n"
                "      tcp_default: left_hand\n"
                "      joints: [left_shoulder, left_elbow, left_wrist, left_finger]\n"
                "    - name: right\n      base_frame: right_base\n      flange_frame: right_hand\n"
                "      tcp_default: right_hand\n"
                "      joints: [right_shoulder, right_elbow, right_wrist, right_finger]\n"
            ),
            "urdf/model.urdf": DUAL_URDF,
        },
        {"urdf": "urdf/model.urdf", "usd": None},
    )
    # A wheeled semi-humanoid: arms as `frames.arms[]`, and the rest of the
    # body — torso, head, an arm planned with the torso — as `frames.groups[]`.
    arm_joints = "shoulder_pitch shoulder_roll elbow wrist finger".split()
    arms = "".join(
        f"    - name: {side}\n      base_frame: {side}_base\n      flange_frame: {side}_flange\n"
        f"      tcp_default: {side}_tcp\n      joints: [{', '.join(f'{side}_{j}' for j in arm_joints)}]\n"
        for side in ("left", "right")
    )
    torso = "lift_joint, waist_yaw_joint"
    add(
        SEMI_ID,
        "public",
        {
            "manifest.yaml": (
                f"schema_version: '0.1'\nid: {SEMI_ID}\ndistribution: public\n"
                "name: Wheeled\nmanufacturer:\n  name: ACME Robotics\ncategory: vehicle.mobile_manipulator\n"
                "specs:\n  arm_count: 2\n  reach_mm: 650\n  max_speed_mps: 1.0\n"
                f"frames:\n  base_frame: base_footprint\n  arms:\n{arms}"
                "  groups:\n"
                f"    - {{name: torso, tip: chest, joints: [{torso}]}}\n"
                "    - {name: head, tip: head_camera, joints: [head_pan_joint, head_tilt_joint]}\n"
                f"    - {{name: right_with_torso, tip: right_tcp, joints: [{torso}, "
                f"{', '.join(f'right_{j}' for j in arm_joints)}]}}\n"
                "self_collision:\n  basis: the maker's collision matrix\n  allowed_pairs:\n"
                "    - [column, neck]\n    - [chest, left_upper]\n"
                "locomotion:\n  kind: wheeled\n  drive: differential\n  wheels:\n"
                "    - {joint: left_wheel_joint, radius_m: 0.1}\n"
                "    - {joint: right_wheel_joint, radius_m: 0.1}\n"
                "  postures:\n    travel: {lift_joint: 0.05}\n"
            ),
            "urdf/model.urdf": SEMI_URDF,
        },
        {"urdf": "urdf/model.urdf", "usd": None},
    )
    (repo / "index.json").write_text(
        json.dumps({"schema_version": "0.1", "generated_at": "2026-08-05", "products": products})
    )

    calls: dict = {"repo": repo}
    fake = types.ModuleType("huggingface_hub")

    # huggingface_hub 1.x signature: keyword-only, and no `repo_type`
    # (that argument was 0.x-only here) — a strict fake so passing it again
    # fails this suite before it fails users.
    def dataset_info(repo_id, *, revision=None, timeout=None, files_metadata=False, token=None):
        assert repo_id == "botrail/botrail-catalog"
        calls["revision_requested"] = revision
        return types.SimpleNamespace(sha=SHA)

    def hf_hub_download(repo_id, filename=None, repo_type=None, revision=None):
        assert revision == SHA
        return str(repo / filename)

    def snapshot_download(repo_id, repo_type=None, revision=None, allow_patterns=None):
        assert revision == SHA
        calls["allow_patterns"] = allow_patterns
        return str(repo)

    fake.dataset_info = dataset_info
    fake.hf_hub_download = hf_hub_download
    fake.snapshot_download = snapshot_download
    monkeypatch.setitem(sys.modules, "huggingface_hub", fake)
    return calls


def test_from_catalog_loads_and_pins_the_revision(catalog: dict) -> None:
    robot = bt.Robot.from_catalog(ARM_ID)
    assert robot.dof == 1
    assert robot.joint_names == ["j1"]
    # The manifest's declared TCP, not the deepest-leaf guess.
    assert robot.tcp_link == "tool_tip"
    # Only the package directory is fetched.
    assert catalog["allow_patterns"] == [f"{ARM_ID}/*"]
    # No revision passed -> newest resolved, but downloads pinned to the SHA.
    assert catalog["revision_requested"] is None
    robot_pinned = bt.Robot.from_catalog(ARM_ID, revision=SHA)
    assert catalog["revision_requested"] == SHA
    assert robot_pinned.dof == 1


def test_short_ids_resolve_by_segment_subsequence(catalog: dict) -> None:
    assert bt.Robot.from_catalog("mini").name == "mini"
    assert bt.Robot.from_catalog("acme/mini").name == "mini"
    with pytest.raises(ValueError, match="ambiguous.*mini.*maxi|ambiguous"):
        bt.Robot.from_catalog("acme/arm")
    with pytest.raises(ValueError, match="not in the catalog"):
        bt.Robot.from_catalog("nope")


def test_a_short_name_takes_the_newest_revision(catalog: dict, tmp_path: Path) -> None:
    """Short names follow public revisions; replay records the resolved ID.
    Selecting a revision does not assert equivalent geometry or motion."""
    scene = bt.Scene(bt.Robot.from_catalog("mini"))
    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    code = bt.Scene.load_project(project).generate_python()
    assert f'from_catalog("{ARM_R2}"' in code
    assert ARM_ID not in code

    # Naming a revision outright still pins it.
    pinned = bt.Scene(bt.Robot.from_catalog(ARM_ID))
    pinned.save_project(project)
    assert f'from_catalog("{ARM_ID}"' in bt.Scene.load_project(project).generate_python()


def _set_distribution(catalog: dict, ids: list[str], distribution: str) -> None:
    path = catalog["repo"] / "index.json"
    data = json.loads(path.read_text())
    for product in data["products"]:
        if product["id"] in ids:
            product["distribution"] = distribution
    path.write_text(json.dumps(data))


@pytest.mark.parametrize("query", ["mini", "acme/mini"])
def test_new_metadata_revision_preserves_public_loading(catalog, tmp_path, query):
    _set_distribution(catalog, [ARM_R2], "recipe_only")
    assert bt.catalog.Index.from_path(catalog["repo"] / "index.json").get(query).id == ARM_ID
    assert Path(bt.catalog_package(query)) == catalog["repo"] / ARM_ID
    robot = bt.Robot.from_catalog(query)
    assert catalog["allow_patterns"] == [f"{ARM_ID}/*"]

    # Replaying the selected public model does not re-resolve its short name.
    scene = bt.Scene(robot)
    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    _set_distribution(catalog, [ARM_R2], "public")
    assert f'from_catalog("{ARM_ID}", revision="{SHA}")' in bt.Scene.load_project(project).generate_python()


@pytest.mark.parametrize("query", [ARM_R2, "mini/r2"])
def test_explicit_recipe_revision_is_not_substituted(catalog, query):
    _set_distribution(catalog, [ARM_R2], "recipe_only")
    with pytest.raises(ValueError, match=f"{ARM_R2}.*recipe_only.*locally"):
        bt.Robot.from_catalog(query)
    assert "allow_patterns" not in catalog


def test_no_public_revision_reports_latest_recipe(catalog):
    _set_distribution(catalog, [ARM_ID, ARM_R2], "recipe_only")
    assert bt.catalog.Index.from_path(catalog["repo"] / "index.json").get("mini").id == ARM_R2
    with pytest.raises(ValueError, match=f"{ARM_R2}.*recipe_only.*locally"):
        bt.Robot.from_catalog("mini")
    assert "allow_patterns" not in catalog


@pytest.mark.parametrize("failure", ["missing_model", "format", "network"])
def test_public_revision_failure_does_not_retry_older_model(catalog, monkeypatch, failure):
    kwargs = {}
    if failure == "missing_model":
        (catalog["repo"] / ARM_R2 / "urdf/model.urdf").unlink()
        match = "model.urdf"
    elif failure == "format":
        kwargs["format"] = "usd"
        match = f"{ARM_R2}.*ships no usd model"
    else:
        def unavailable(*args, allow_patterns=None, **kwargs):
            catalog["allow_patterns"] = allow_patterns
            raise OSError("download unavailable")
        monkeypatch.setattr(sys.modules["huggingface_hub"], "snapshot_download", unavailable)
        match = "download unavailable"
    with pytest.raises(ValueError, match=match):
        bt.Robot.from_catalog("mini", **kwargs)
    assert catalog["allow_patterns"] == [f"{ARM_R2}/*"]


def test_distinct_products_stay_ambiguous(catalog: dict) -> None:
    """The rule is narrow on purpose: only a differing trailing revision
    collapses. `mini` and `maxi` are different machines, and picking one
    for the caller would be a guess."""
    _set_distribution(catalog, [MAXI_ID, LOCKED_ID], "recipe_only")
    with pytest.raises(KeyError, match="ambiguous"):
        bt.catalog.Index.from_path(catalog["repo"] / "index.json").get("acme/arm")
    with pytest.raises(ValueError, match="ambiguous"):
        bt.Robot.from_catalog("acme/arm")


def test_usd_only_packages_load_via_the_importer(catalog: dict) -> None:
    robot = bt.Robot.from_catalog("plate")
    assert robot.dof == 0
    assert robot.link_names == ["/Plate/body"]


def test_recipe_only_raises_with_a_pointer_to_local_builds(catalog: dict) -> None:
    with pytest.raises(ValueError, match="recipe_only.*locally"):
        bt.Robot.from_catalog("locked")


def test_missing_dependency_names_the_extra(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setitem(sys.modules, "huggingface_hub", None)
    with pytest.raises(ValueError, match=r"botrail\[catalog\]"):
        bt.Robot.from_catalog("anything")


def test_projects_and_scripts_replay_the_pinned_catalog(
    catalog: dict, tmp_path: Path
) -> None:
    # By full id: this is about replaying a pinned catalog, not about how
    # short names resolve (that is its own test).
    robot = bt.Robot.from_catalog(ARM_ID)
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.4])
    project = tmp_path / "cell.botrail"
    scene.save_project(project)

    # Loading needs no network: the fetched URDF is embedded in the project.
    del sys.modules["huggingface_hub"]
    reloaded = bt.Scene.load_project(project)
    assert reloaded.robot.dof == 1
    assert reloaded.robot.tcp_link == "tool_tip"
    assert reloaded.joint_positions == pytest.approx([0.4])

    # The generated script re-fetches by id at the pinned SHA.
    code = reloaded.generate_python()
    assert f'bt.Robot.from_catalog("{ARM_ID}", revision="{SHA}")' in code


def test_catalog_tool_mounts_on_a_catalog_robot(catalog: dict) -> None:
    arm = bt.Robot.from_catalog("mini")
    plate = bt.Robot.from_catalog("plate")
    combined = arm.attach_tool(plate, flange="tool_tip", mount="/Plate/body")
    assert combined.dof == 1
    assert "/Plate/body" in combined.link_names


def test_manifest_frames_enable_argument_free_mounting(
    catalog: dict, tmp_path: Path
) -> None:
    arm = bt.Robot.from_catalog("mini")
    plate = bt.Robot.from_catalog("plate")
    # The manifests declared the faces, so nobody has to name them.
    assert arm.flange_link == "link1"
    assert plate.mount_link == "/Plate/body"
    combined = arm.attach_tool(plate)
    assert "/Plate/body" in combined.link_names
    # The plate declares no onward flange: the stack ends here.
    assert combined.flange_link is None
    # Without any declaration, botrail refuses to guess a flange.
    bare = bt.Robot.from_urdf_string(ARM_URDF)
    with pytest.raises(ValueError, match="flange"):
        bare.attach_tool(plate)

    # The nested source — a composite of two catalog parts, one of them
    # USD — survives a project roundtrip without network access.
    scene = bt.Scene(combined)
    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    del sys.modules["huggingface_hub"]
    reloaded = bt.Scene.load_project(project)
    assert reloaded.robot.dof == 1
    assert "/Plate/body" in reloaded.robot.link_names


def test_catalog_identity_reaches_the_bom_and_survives_the_project(
    catalog: dict, tmp_path: Path
) -> None:
    """A catalog robot is an identified BOM line without any authoring:
    maker, product, category, catalog id@revision and the numeric specs
    come off the manifest. A tool welded on is its own line. The identity
    is persisted, so a reloaded project (no network) still names it."""
    arm = bt.Robot.from_catalog(ARM_ID)
    coupling = bt.Robot.from_catalog(COUPLING_ID)
    scene = bt.Scene(arm.attach_tool(coupling))
    bom = scene.bom()
    assert len(bom) == 3
    robot, tool, controller = bom.rows
    assert robot["names"] == ["mini"]
    assert robot["manufacturer"] == "ACME Robotics"
    assert robot["model"] == "Mini Arm"
    assert robot["category"] == "manipulator"
    assert robot["catalog"] == f"{ARM_ID}@{SHA}"
    # Numeric specs only — the controller list and IP rating are not
    # attributes.
    assert robot["attributes"] == {"dof": 1.0, "payload_kg": 3.5, "reach_mm": 300.0}
    # The coupling's manifest carries no identity: the line still shows
    # the package, unidentified only in the maker/model sense.
    assert tool["names"] == ["mini/tool"]
    assert tool["category"] == "tool"
    assert tool["catalog"] == f"{COUPLING_ID}@{SHA}"
    # The controller the arm needs, named by the one the manifest lists
    # (`specs.controller`) — the maker's, not a purchase from the catalog.
    assert controller["names"] == ["mini/controller"]
    assert controller["category"] == "robot_controller"
    assert (controller["manufacturer"], controller["model"], controller["catalog"]) == ("ACME Robotics", "MC-1", None)
    assert controller["description"] == "controller for mini"
    assert bom.unidentified() == []
    assert bom.total("payload_kg") == 3.5

    # A pinned part on the robot overlays the derived identity.
    scene.set_part("mini", description="handling arm", price=1_500_000)
    row = scene.bom().rows[0]
    assert row["manufacturer"] == "ACME Robotics"
    assert row["description"] == "handling arm"
    assert scene.bom().total("price") == 1_500_000

    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    reloaded = bt.Scene.load_project(project)
    assert reloaded.bom().rows == scene.bom().rows
    assert 'scene.set_part("mini", kind="robot", description="handling arm", price=1500000)' in (
        reloaded.generate_python()
    )


def test_a_dual_arm_package_loads_with_its_arms_as_groups(
    catalog: dict, tmp_path: Path
) -> None:
    """`frames.arms[]` becomes the robot's planning groups — declared, not
    derived, so the names are the manifest's and the finger rides with its
    arm — and they survive the project and the generated script."""
    pair = bt.Robot.from_catalog("pair")
    assert pair.groups == ["left", "right"]
    assert pair.group("left").joints == ["left_shoulder", "left_elbow", "left_wrist", "left_finger"]
    assert pair.group("left").tip == "left_hand"
    assert pair.group("right").flange == "right_hand"
    assert not pair.group("left").derived
    with pytest.raises(ValueError, match="several arms"):
        _ = pair.tcp_link

    scene = bt.Scene(pair)
    scene.add_segment("reach", goal=scene.joint_positions, group="right")
    path = tmp_path / "pair.botrail"
    scene.save_project(path)
    loaded = bt.Scene.load_project(path)
    assert loaded.robot.groups == ["left", "right"]
    assert loaded.robot.group("right").joints == pair.group("right").joints
    src = scene.generate_python()
    assert 'from_catalog("acme/pair/dual/r1"' in src and 'group="right"' in src
    ns: dict = {}
    exec(compile(src.replace("bt.studio(scene)", ""), "<generated>", "exec"), ns)
    assert ns["scene"].robot.groups == ["left", "right"]
    # The BOM line carries the arm count the manifest quoted.
    row = next(r for r in scene.bom().rows if DUAL_ID in str(r.get("catalog", "")) or "pair" in r["names"][0])
    assert row["attributes"].get("arm_count") == 2


def test_a_whole_body_package_loads_its_torso_and_head_as_groups(
    catalog: dict, tmp_path: Path
) -> None:
    """`frames.groups[]` — a whole-body machine's torso, head and the
    composites an arm is planned with the torso by — are declared after the
    arms, so the wheels are nobody's planning joints, an arm plans without
    the lift, and the composite plans with it."""
    robot = bt.Robot.from_catalog("wheeled")
    assert robot.groups == ["left", "right", "torso", "head", "right_with_torso"]
    assert robot.group("torso").joints == ["lift_joint", "waist_yaw_joint"]
    assert robot.group("torso").tip == "chest" and robot.group("torso").flange is None
    assert robot.group("right").flange == "right_flange"
    assert len(robot.group("right_with_torso").joints) == 7
    grouped = {j for g in robot.groups for j in robot.group(g).joints}
    assert {"left_wheel_joint", "right_wheel_joint"}.isdisjoint(grouped)
    # The pairs the package declares may touch ride in on the model (YAML
    # lists, not tuples — the loader once read none of them)...
    assert robot.allowed_collisions == [("column", "neck"), ("chest", "left_upper")]
    more = robot.allow_collisions([("neck", "column"), ("head", "chest")])
    assert more.allowed_collisions == [*robot.allowed_collisions, ("chest", "head")]
    with pytest.raises(ValueError, match="link `nowhere` does not exist"):
        robot.allow_collisions([("chest", "nowhere")])

    scene = bt.Scene(robot)
    names = robot.joint_names
    start = list(scene.joint_positions)
    position, quaternion = scene.link_pose("right_tcp")
    # 0.35 m lower is out of the arm's reach from where the lift stands ...
    low = (position[0] + 0.15, position[1], position[2] - 0.35)
    # ... so the arm alone moves only itself, and the composite brings the torso.
    scene.set_tcp_target(low, quaternion, group="right")
    alone = list(scene.joint_positions)
    assert alone[names.index("lift_joint")] == start[names.index("lift_joint")] == 0.0
    scene.set_joint_positions(start)
    # An unnamed plan on a machine with several groups is refused by name.
    with pytest.raises(ValueError, match="group"):
        scene.plan(start)
    scene.add_segment("reach", goal=start, group="right_with_torso")

    path = tmp_path / "wheeled.botrail"
    scene.save_project(path)
    loaded = bt.Scene.load_project(path)
    assert loaded.robot.groups == robot.groups
    # ...and come back with the project, as the package's own.
    assert loaded.robot.allowed_collisions == robot.allowed_collisions
    assert "allow_collisions" not in loaded.generate_python()
    assert loaded.robot.group("head").joints == ["head_pan_joint", "head_tilt_joint"]
    # A reach circle is an arm's: the layout draws two, not five.
    svg = scene.layout()
    reach = svg.split('class="reach"')[1].split("</g>")[0]
    assert reach.count("<circle") == 2


def test_a_whole_body_package_rolls_on_its_declared_wheels(catalog: dict) -> None:
    """`bt.Wheels.from_catalog` by catalog id, and what the requirements make
    of the machine: two arms (not five groups), taught through a composite,
    with the vehicle's speed on the robot's own line."""
    robot = bt.Robot.from_catalog("wheeled")
    gear = bt.Wheels.from_catalog("wheeled")
    assert gear.wheels == {"left_wheel_joint": (0.1, 0.0), "right_wheel_joint": (0.1, 0.0)}
    assert gear.base_frame == "base_footprint" and gear.posture == {"lift_joint": 0.05}
    scene = bt.Scene(robot, name="semi")
    scene.add_vehicle("base", body=[], path=[(0, 0), (2, 0)], stations={"a": 0, "b": 1},
                      speed=0.5, start="a", drive=gear.vehicle_drive)
    scene.mount_robot("base", robot="semi", wheels=gear)
    assert dict(zip(robot.joint_names, scene.joint_positions))["lift_joint"] == 0.05
    assert [r["names"] for r in scene.bom().rows] == [["semi"]]

    def asked() -> dict:
        return {r.key: (r.value, r.basis) for r in bt.select.requirements(scene)["semi"].requirements}

    assert asked()["arm_count"] == (2.0, "arms of the robot")
    assert asked()["max_speed_mps"][0] == 0.5
    # A composite's motion teaches the arm inside it — one arm, not a third —
    # and its target is a working height over the floor.
    names = robot.joint_names
    q = list(scene.joint_positions)
    for joint, value in (("lift_joint", 0.30), ("right_shoulder_pitch", -1.2), ("right_elbow", -0.9)):
        q[names.index(joint)] = value
    scene.add_segment("reach", goal=q, group="right_with_torso")
    got = asked()
    assert got["arm_count"] == (1.0, "arms taught: right")
    # Reach is not asked of a machine that carries its arms' bases on its own
    # torso: that distance is this machine's way of standing at the work, and
    # as a requirement it would error against a vendor's figure and filter the
    # catalog by it. The line says so instead.
    assert "reach_mm" not in got
    assert any(note.startswith("reach_mm is not asked") for note in bt.select.requirements(scene)["semi"].notes)
    # The same robot on a pedestal is an arm on a body that stands still: its
    # reach is asked — from the arm's first joint (its shoulder, not the body
    # link the shoulder is bolted to), *as taught*, the lift raised with it.
    fixed = bt.Scene(robot, name="semi")
    fixed.add_segment("reach", goal=q, group="right_with_torso")
    on_pedestal = {r.key: (r.value, r.basis) for r in bt.select.requirements(fixed)["semi"].requirements}
    tip, _ = fixed.link_pose_at("right_flange", q)
    assert robot.group("right").base == "right_base" and robot.group("right").first_link == "right_shoulder"
    base, _ = fixed.link_pose_at("right_shoulder", q)
    span = sum((a - b) ** 2 for a, b in zip(tip, base)) ** 0.5
    assert on_pedestal["reach_mm"][0] == pytest.approx(span * 1100.0, abs=0.1)
    assert "the right arm's first joint (flange)" in on_pedestal["reach_mm"][1]
    assert "vertical_reach_max_mm" not in on_pedestal
    height = scene.link_pose_at("right_tcp", q)[0][2]
    assert got["vertical_reach_min_mm"][0] == got["vertical_reach_max_mm"][0] == pytest.approx(height * 1000, abs=0.1)
    assert "taught hand position" in got["vertical_reach_max_mm"][1] and "`reach`" in got["vertical_reach_max_mm"][1]
    line = bt.select.requirements(scene)["semi"]
    assert {r.key: r.op for r in line.requirements}["vertical_reach_min_mm"] == "<="
    # The package states no working heights: asked for, not answered.
    assert {r.key: r.status for r in line.requirements}["vertical_reach_max_mm"] == "unknown"
    # A rigid mount of the same machine asks for no working height.
    rigid = bt.Scene(robot, name="semi")
    rigid.add_vehicle("base", body=[], path=[(0, 0), (2, 0)], stations={"a": 0, "b": 1}, start="a")
    rigid.mount_robot("base", robot="semi")
    rigid.add_segment("reach", goal=q, group="right_with_torso")
    keys = {r.key for r in bt.select.requirements(rigid)["semi"].requirements}
    assert "vertical_reach_max_mm" not in keys and "reach_mm" in keys
