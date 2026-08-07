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
) -> str:
    frames = f"  tcp_default: {tcp}\n" if tcp else ""
    if mount:
        frames += f"  mount_frame: {mount}\n"
    return (
        f"schema_version: '0.1'\nid: {pid}\ndistribution: {distribution}\n"
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
        {"manifest.yaml": _manifest(ARM_ID, tcp="tool_tip"), "urdf/model.urdf": ARM_URDF},
        {"urdf": "urdf/model.urdf", "usd": None},
    )
    # A second cut of the same product — a better source, same machine.
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
    """Revisions are the same product re-cut from a better source, so a
    short name follows them forward instead of turning every catalog
    revision into a breaking change. The *resolved* id is what gets
    recorded, so a replay stays on the revision it resolved to."""
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


def test_distinct_products_stay_ambiguous(catalog: dict) -> None:
    """The rule is narrow on purpose: only a differing trailing revision
    collapses. `mini` and `maxi` are different machines, and picking one
    for the caller would be a guess."""
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
