"""Lidar x catalog: `add_lidar(from_catalog=)` and the selection loop.

The design demo (design-lidar.md §7 L4): a package's flat specs become
the sweep, its scan frame poses the origin (ROS laser convention — no
rotation fix), its identity lands on the BOM — and `requirements()`/
`check()` answer ok / spec_short / spec_unknown for lidars like for
every other line.
"""

import json
import math
import re
import sys
import types
from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"
LID_ID = "acme/scan/ray/r1"
SHA = "beef0123beef0123beef0123beef0123beef0123"

# The scan frame hangs off the mount with a lift and a quarter turn: the
# package's own calibration says the sweep heading is mount +Y.
LID_URDF = """
<robot name="ray">
  <link name="mount"/>
  <link name="body"/>
  <link name="laser"/>
  <joint name="b" type="fixed">
    <parent link="mount"/><child link="body"/><origin xyz="0 0 0.05"/>
  </joint>
  <joint name="l" type="fixed">
    <parent link="body"/><child link="laser"/>
    <origin xyz="0 0 0.052" rpy="0 0 1.5707963267948966"/>
  </joint>
</robot>
"""

LID_SPECS = {
    "scan_fov_deg": 270.0,
    "angular_resolution_deg": 0.5,
    "scan_rate_hz": 50.0,
    "min_range_mm": 500.0,
    "max_range_mm": 20000.0,
    "safety_rated": 0,
}

LID_MANIFEST = f"""schema_version: '0.1'
id: {LID_ID}
distribution: public
name: Ray 270
manufacturer:
  name: ACME Sensing
category: sensor.lidar
specs:
{chr(10).join(f"  {k}: {v}" for k, v in LID_SPECS.items())}
frames:
  mount_frame: mount
  lidar_frames: [laser]
"""


@pytest.fixture()
def catalog(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    repo = tmp_path / "dataset"
    pkg = repo / LID_ID
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "manifest.yaml").write_text(LID_MANIFEST)
    (pkg / "urdf" / "model.urdf").write_text(LID_URDF)
    index = {
        "schema_version": "0.1",
        "generated_at": "2026-08-29",
        "products": [
            {
                "id": LID_ID,
                "category": "sensor.lidar",
                "name": "Ray 270",
                "manufacturer": "ACME Sensing",
                "specs": dict(LID_SPECS),
                "validation_level": "V2",
                "distribution": "public",
                "assets": {"urdf": f"{LID_ID}/urdf/model.urdf", "usd": None},
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
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def test_from_catalog_fills_sweep_scan_frame_and_bom(scene) -> None:
    scene.add_lidar("gate", from_catalog="acme/scan/ray", position=(1.0, 0.0, 0.5))
    code = scene.generate_python()
    line = next(l for l in code.splitlines() if 'add_lidar("gate"' in l)
    # The sweep from the package's flat specs (range in meters).
    assert "fov=270" in line and "range=(0.5, 20)" in line and "resolution=0.5" in line, line
    # The pose places the mount face; the scan origin follows the
    # package's calibration: lifted 0.102 and swept toward mount +Y.
    m = re.search(r"position=\(([-\d., ]+)\), quaternion=\(([-\d., e]+)\)", line)
    pos = tuple(float(v) for v in m.group(1).split(","))
    q = tuple(float(v) for v in m.group(2).split(","))
    assert math.isclose(pos[2], 0.602, abs_tol=1e-6), pos
    heading = rotate(q, (1.0, 0.0, 0.0))
    assert math.isclose(heading[1], 1.0, abs_tol=1e-5), heading
    # The identity landed on the BOM, specs and all.
    row = next(r for r in scene.bom().rows if "gate" in (r.get("names") or []))
    assert row["category"] == "sensor.lidar"
    assert row.get("manufacturer") == "ACME Sensing" and row.get("model") == "Ray 270"
    assert f"{LID_ID}@{SHA}" in (row.get("catalog") or "")
    assert row["attributes"]["scan_fov_deg"] == 270.0
    # Explicit arguments still win over the package.
    scene.add_lidar("narrow", from_catalog="acme/scan/ray", fov=180.0)
    assert "fov=180" in next(
        l for l in scene.generate_python().splitlines() if '"narrow"' in l
    )
    # A scan through the catalog scanner sweeps from the calibrated
    # frame, heading +Y: the wall face at y = 1.5 answers on beam 0.
    scene.add_box("side_wall", (2.0, 0.1, 1.0), (1.0, 1.55, 0.5))
    frame = scene.lidar_scan("gate")
    beams = dict(zip(frame.angles, frame.ranges))
    assert abs(beams[0.0] - 1.5) < 1e-3, beams[0.0]


def test_requirements_check_and_search_close_the_loop(scene, catalog) -> None:
    scene.add_lidar(
        "gate", from_catalog="acme/scan/ray", position=(2.7, 0.0, 0.15), yaw=-90.0
    )
    scene.add_field_sensor("gate_warn", lidar="gate", range=2.5)

    rows = {r.target: r for r in scene.requirements().rows}
    row = rows["gate"]
    assert row.kind == "lidar" and row.category == "sensor.lidar"
    reqs = {r.key: r for r in row.requirements}
    # The design demo's numbers: 270 >= 270 ok, 20000 >= 2500 ok — and
    # the `<=` blind-ring floor gates check but stays out of `row.minimum`.
    assert reqs["scan_fov_deg"].status == "ok" and reqs["scan_fov_deg"].provided == 270.0
    assert reqs["max_range_mm"].value == 2500.0 and reqs["max_range_mm"].status == "ok"
    assert reqs["min_range_mm"].op == "<=" and reqs["min_range_mm"].status == "ok"
    assert "min_range_mm" not in row.minimum and "max_range_mm" in row.minimum
    assert not [f for f in scene.check().findings if f.target == "gate"]

    # search_for closes the loop against the (local) index.
    hits = bt.catalog.search_for(row, index=catalog / "index.json")
    assert [p.id for p in hits] == [LID_ID]

    # Too wide a sweep for the part: the requirement answers, and falls
    # short. (Hand-identified — a second from_catalog of the same product
    # would merge into `gate`'s line.)
    scene.add_lidar("dome", position=(0.0, 2.0, 0.2), fov=360.0)
    scene.set_part("dome", kind="lidar", model="R-1", attributes={"scan_fov_deg": 270.0})
    findings = [f for f in scene.check().findings if f.target == "dome"]
    assert any(f.code == "spec_short" and "scan_fov_deg" in f.message for f in findings), [
        (f.target, f.code, f.message) for f in scene.check().findings
    ]

    # An identified part that does not state the axis: unknown, not short.
    scene.add_lidar("mystery", position=(0, 3, 0.2))
    scene.set_part("mystery", kind="lidar", model="LX-9")
    findings = [f for f in scene.check().findings if f.target == "mystery"]
    assert any(f.code == "spec_unknown" for f in findings)


def test_survey_only_scanner_asks_no_range(scene) -> None:
    # No field judges through it: the authored max range is a scan reach,
    # not a spec — only the sweep angle is required (判断 L12).
    scene.add_lidar("survey", from_catalog="acme/scan/ray", position=(0, 0, 0.2))
    rows = {r.target: r for r in scene.requirements().rows}
    keys = {r.key for r in rows["survey"].requirements}
    assert keys == {"scan_fov_deg"}, keys
