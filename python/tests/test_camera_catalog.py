"""Camera x catalog: `add_camera(from_catalog=)` and the selection loop.

The design demo (design-camera.md §7 CAM6): a package's flat specs become
the optics, its optical calibration poses the axis, its identity lands on
the BOM — and `requirements()`/`check()` answer ok / spec_short /
spec_unknown for cameras like for every other line.
"""

import json
import math
import re
import sys
import types
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
CAM_ID = "acme/cam/eye/r1"
SHA = "feed0123feed0123feed0123feed0123feed0123"

# ROS convention: the body looks along mount +X; the optical frame
# (+Z forward, +Y down) hangs off it with the usual rpy.
CAM_URDF = """
<robot name="eye">
  <link name="mount"/>
  <link name="body"/>
  <link name="optical"/>
  <joint name="b" type="fixed">
    <parent link="mount"/><child link="body"/><origin xyz="0 0 0.0125"/>
  </joint>
  <joint name="o" type="fixed">
    <parent link="body"/><child link="optical"/>
    <origin xyz="0.01 0 0" rpy="-1.5707963267948966 0 -1.5707963267948966"/>
  </joint>
</robot>
"""

CAM_MANIFEST = """schema_version: '0.1'
id: acme/cam/eye/r1
distribution: public
name: Eye 100
manufacturer:
  name: ACME Vision
category: sensor.camera
specs:
  fov_h_deg: 87.0
  fov_v_deg: 58.0
  resolution_h_px: 1280
  resolution_v_px: 720
  min_range_mm: 280.0
  max_range_mm: 10000.0
  frame_rate_hz: 90.0
frames:
  mount_frame: mount
  camera_frames: [optical]
"""

CAM_SPECS = {
    "fov_h_deg": 87.0,
    "fov_v_deg": 58.0,
    "resolution_h_px": 1280,
    "resolution_v_px": 720,
    "min_range_mm": 280.0,
    "max_range_mm": 10000.0,
    "frame_rate_hz": 90.0,
}


@pytest.fixture()
def catalog(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    repo = tmp_path / "dataset"
    pkg = repo / CAM_ID
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "manifest.yaml").write_text(CAM_MANIFEST)
    (pkg / "urdf" / "model.urdf").write_text(CAM_URDF)
    index = {
        "schema_version": "0.1",
        "generated_at": "2026-08-28",
        "products": [
            {
                "id": CAM_ID,
                "category": "sensor.camera",
                "name": "Eye 100",
                "manufacturer": "ACME Vision",
                "specs": dict(CAM_SPECS),
                "validation_level": "V2",
                "distribution": "public",
                "assets": {"urdf": f"{CAM_ID}/urdf/model.urdf", "usd": None},
            }
        ],
    }
    (repo / "index.json").write_text(json.dumps(index))

    fake = types.ModuleType("huggingface_hub")

    def dataset_info(repo_id, *, revision=None, timeout=None, files_metadata=False, token=None):
        return types.SimpleNamespace(sha=SHA)

    def hf_hub_download(repo_id, filename=None, repo_type=None, revision=None):
        return str(repo / filename)

    def snapshot_download(repo_id, repo_type=None, revision=None, allow_patterns=None):
        return str(repo)

    fake.dataset_info = dataset_info
    fake.hf_hub_download = hf_hub_download
    fake.snapshot_download = snapshot_download
    monkeypatch.setitem(sys.modules, "huggingface_hub", fake)
    return repo


def rotate(q, v):
    x, y, z, w = q
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (
        v[0] + w * tx + (y * tz - z * ty),
        v[1] + w * ty + (z * tx - x * tz),
        v[2] + w * tz + (x * ty - y * tx),
    )


@pytest.fixture()
def scene(catalog) -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf"))


def test_from_catalog_fills_optics_axis_and_bom(scene) -> None:
    scene.add_camera("eye", from_catalog="acme/cam/eye", position=(1.0, 0.0, 0.5))
    code = scene.generate_python()
    line = next(l for l in code.splitlines() if 'add_camera("eye"' in l)
    # Optics from the package's flat specs.
    assert "fov=87" in line and "resolution=(1280, 720)" in line, line
    assert "near=0.28" in line and "far=10" in line, line
    # The pose places the mount face; the optical axis follows the
    # package's calibration: the body looks along mount +X.
    m = re.search(r"position=\(([-\d., ]+)\), quaternion=\(([-\d., ]+)\)", line)
    pos = tuple(float(v) for v in m.group(1).split(","))
    q = tuple(float(v) for v in m.group(2).split(","))
    assert math.isclose(pos[0], 1.01, abs_tol=1e-6) and math.isclose(pos[2], 0.5125, abs_tol=1e-6)
    view = rotate(q, (0.0, 0.0, -1.0))
    assert math.isclose(view[0], 1.0, abs_tol=1e-6), view
    up = rotate(q, (0.0, 1.0, 0.0))
    assert math.isclose(up[2], 1.0, abs_tol=1e-6), up
    # The identity landed on the BOM, specs and all.
    row = next(r for r in scene.bom().rows if "eye" in (r.get("names") or []))
    assert row["category"] == "sensor.camera"
    assert row.get("manufacturer") == "ACME Vision" and row.get("model") == "Eye 100"
    assert f"{CAM_ID}@{SHA}" in (row.get("catalog") or "")
    assert row["attributes"]["fov_h_deg"] == 87.0
    # Explicit arguments still win over the package.
    scene.add_camera("tight", from_catalog="acme/cam/eye", fov=25.0)
    assert 'scene.add_camera("tight"' in scene.generate_python()
    assert "fov=25" in next(
        l for l in scene.generate_python().splitlines() if '"tight"' in l
    )


def test_requirements_check_and_search_close_the_loop(scene, catalog) -> None:
    scene.add_box("part", (0.1, 0.1, 0.1), (0.5, 0.0, 0.3))
    scene.add_camera(
        "inspect",
        from_catalog="acme/cam/eye",
        position=(1.5, 0.0, 0.3),
        quaternion=(0.0, 0.0, 1.0, 0.0),  # mount +X toward -X: at the part
        fov=70.0,
    )
    scene.add_vision_sensor("part_seen", camera="inspect", watch=["part"], detect_range=(0.3, 2.4))

    rows = {r.target: r for r in scene.requirements().rows}
    row = rows["inspect"]
    assert row.kind == "camera" and row.category == "sensor.camera"
    reqs = {r.key: r for r in row.requirements}
    # The design demo's numbers: 87 >= 70 ok, 10000 >= 2400 ok — and the
    # `<=` floor gates check but stays out of `row.minimum`.
    assert reqs["fov_deg"].status == "ok" and reqs["fov_deg"].provided == 87.0
    assert reqs["max_range_mm"].value == 2400.0 and reqs["max_range_mm"].status == "ok"
    assert reqs["min_range_mm"].op == "<=" and reqs["min_range_mm"].status == "ok"
    assert "min_range_mm" not in row.minimum and "max_range_mm" in row.minimum
    assert not [f for f in scene.check().findings if f.target == "inspect"]

    # search_for closes the loop against the (local) index.
    hits = bt.catalog.search_for(row, index=catalog / "index.json")
    assert [p.id for p in hits] == [CAM_ID]

    # Too greedy a framing: the part answers, and falls short. (A second
    # `from_catalog` of the same product would merge into `inspect`'s BOM
    # line — identical products are one line — so this one is a distinct
    # hand-identified model.)
    scene.add_camera("wide", position=(0.0, 2.0, 1.0), fov=100.0)
    scene.set_part(
        "wide", kind="camera", model="W-1", attributes={"fov_h_deg": 87.0}
    )
    findings = [f for f in scene.check().findings if f.target == "wide"]
    assert any(f.code == "spec_short" and "fov_deg" in f.message for f in findings), [
        (f.target, f.code, f.message) for f in scene.check().findings
    ]

    # An identified part that does not state the axis: unknown, not short.
    scene.add_camera("mystery", position=(0, 2, 1))
    scene.set_part("mystery", kind="camera", model="CAM-X")
    findings = [f for f in scene.check().findings if f.target == "mystery"]
    assert any(f.code == "spec_unknown" for f in findings)
