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


def _manifest(pid: str, tcp: str | None = None, distribution: str = "public") -> str:
    frames = f"  tcp_default: {tcp}\n" if tcp else ""
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
    add(
        MAXI_ID,
        "public",
        {"manifest.yaml": _manifest(MAXI_ID), "urdf/model.urdf": ARM_URDF},
        {"urdf": "urdf/model.urdf", "usd": None},
    )
    add(
        COUPLING_ID,
        "public",
        {"manifest.yaml": _manifest(COUPLING_ID), "usd/model.usda": COUPLING_USD},
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

    def dataset_info(repo_id, repo_type=None, revision=None):
        assert repo_id == "botrail/botrail-catalog"
        assert repo_type == "dataset"
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
    robot = bt.Robot.from_catalog("mini")
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
