"""`bt.assembly` and its parts against catalog spec packs — screws, a
presenter and a workpiece set ordered by id (design-assembly.md A2).

The packs here are written the way the catalog builder publishes them
(a `manifest.yaml` in a package directory), so nothing is fetched. What
these pin: a screw read from its pack carries the pack's figures, article
number and mass; the presenter sizes itself from its pack and makes its
own magazine; the workpiece set stands its housing, dowels and cover
from the pack and the joint reads the same `mounting` for its holes,
screw, torque and engagement — no coordinate typed in; and a screwdriver
line on the bill is checked against the torque the screws ask for."""

from pathlib import Path

import botrail as bt
import pytest

A = bt.assembly
EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

SCREW = """
schema_version: '0.1'
id: acme/fastener/iso4762-m5/r1
kind: spec
category: fastener
name: ISO 4762 M5 hexagon socket head cap screw
manufacturer:
  name: ACME Fasteners
distribution: public
specs:
  thread_mm: 5
  pitch_mm: 0.8
  head_standard: ISO 4762
  torque_ref_88_nm: 6.15
  torque_ref_109_nm: 8.65
configuration:
  generator: bolt
  params:
    length_mm:
      values: [12, 16, 20, 25, 30]
      default: 20
    property_class:
      values: ["8.8", "10.9"]
      default: "8.8"
  components:
    - role: screw
      category: fastener
      part_number: "ISO 4762 M5x{length_mm}-{property_class}"
      dimensions_mm: {thread: 5, pitch: 0.8, head_dk: 8.5, head_k: 5, drive_s: 4}
      mass:
        table:
          - {length_mm: 12, kg: 0.0036}
          - {length_mm: 16, kg: 0.0043}
          - {length_mm: 20, kg: 0.0049}
          - {length_mm: 25, kg: 0.0056}
          - {length_mm: 30, kg: 0.0064}
"""

FEEDER = """
schema_version: '0.1'
id: acme/feeder/presenter/r1
kind: spec
category: feeder.screw
name: Screw Presenter
manufacturer:
  name: ACME Feeding
distribution: public
mechanical:
  footprint_mm: [220, 160]
  height_mm: 150
  mass_kg: 4.0
  mount: table
configuration:
  generator: screw_feeder
  params:
    thread_mm:
      values: [3, 4, 5, 6]
      default: 5
  components:
    - role: feeder
      category: feeder.screw
      variants:
        - {thread_mm: 3, part_number: "SP-M3", kg: 4.0}
        - {thread_mm: 4, part_number: "SP-M4", kg: 4.0}
        - {thread_mm: 5, part_number: "SP-M5", kg: 4.0}
        - {thread_mm: 6, part_number: "SP-M6", kg: 4.0}
      dimensions_mm: {length: 240, width: 180, height: 140, pick_x: 20, pick_y: -60}
  behavior:
    present_s: 1.5
"""

WORKPIECE = """
schema_version: '0.1'
id: acme/workpiece/cover-set/r1
kind: spec
category: workpiece
name: Cover Set
manufacturer:
  name: ACME Gears
distribution: public
mechanical:
  footprint_mm: [160, 120]
  height_mm: 100
  mass_kg: 3.7
  mount: table
configuration:
  generator: workpiece
  params:
    variant:
      values: [standard]
      default: standard
  components:
    - role: housing
      category: workpiece
      part_number: "GH-160-H"
      dimensions_mm: {length: 160, width: 120, height: 60}
      mass: {base_kg: 3.1}
    - role: cover
      category: workpiece
      part_number: "GH-160-C"
      dimensions_mm: {length: 160, width: 120, thickness: 10, boss_diameter: 45, boss_height: 30}
      mass: {base_kg: 0.6}
    - role: dowel
      category: fastener
      part_number: "ISO 8734 6m6x16-A"
      dimensions_mm: {diameter: 6, proud: 8}
      mass: {base_kg: 0.0035}
mounting:
  interfaces:
    - frame: housing/top
      role: flange
      geometry:
        normal_z: 1
        holes:
          - {id: h0, position_mm: [-60, -45], kind: threaded, thread: {diameter_mm: 5, pitch_mm: 0.8}, depth_mm: {min: 13, max: 13}, fastener_rules: {min_engagement_mm: 10, torque_nm: {min: 4.0, max: 5.0}}}
          - {id: h1, position_mm: [60, -45], kind: threaded, thread: {diameter_mm: 5, pitch_mm: 0.8}, depth_mm: {min: 13, max: 13}, fastener_rules: {min_engagement_mm: 10, torque_nm: {min: 4.0, max: 5.0}}}
          - {id: h2, position_mm: [60, 45], kind: threaded, thread: {diameter_mm: 5, pitch_mm: 0.8}, depth_mm: {min: 13, max: 13}, fastener_rules: {min_engagement_mm: 10, torque_nm: {min: 4.0, max: 5.0}}}
          - {id: h3, position_mm: [-60, 45], kind: threaded, thread: {diameter_mm: 5, pitch_mm: 0.8}, depth_mm: {min: 13, max: 13}, fastener_rules: {min_engagement_mm: 10, torque_nm: {min: 4.0, max: 5.0}}}
        locators:
          - {id: p0, kind: pin, position_mm: [-60, 0], diameter_mm: {min: 6, max: 6}, depth_mm: {min: 8, max: 8}}
          - {id: p1, kind: pin, position_mm: [60, 0], diameter_mm: {min: 6, max: 6}, depth_mm: {min: 8, max: 8}}
    - frame: cover/bottom
      role: mount
      geometry:
        normal_z: -1
        holes:
          - {id: h0, position_mm: [-60, -45], kind: clearance, diameter_mm: {min: 5.5, max: 5.5}, grip_mm: {min: 10, max: 10}}
          - {id: h1, position_mm: [60, -45], kind: clearance, diameter_mm: {min: 5.5, max: 5.5}, grip_mm: {min: 10, max: 10}}
          - {id: h2, position_mm: [60, 45], kind: clearance, diameter_mm: {min: 5.5, max: 5.5}, grip_mm: {min: 10, max: 10}}
          - {id: h3, position_mm: [-60, 45], kind: clearance, diameter_mm: {min: 5.5, max: 5.5}, grip_mm: {min: 10, max: 10}}
        locators:
          - {id: p0, kind: hole, position_mm: [-60, 0], diameter_mm: {min: 6.1, max: 6.1}, depth_mm: {min: 10, max: 10}}
          - {id: p1, kind: hole, position_mm: [60, 0], diameter_mm: {min: 6.1, max: 6.1}, depth_mm: {min: 10, max: 10}}
      fasteners:
        - holes: [h0, h1, h2, h3]
          thread: {diameter_mm: 5, pitch_mm: 0.8}
          head_standard: ISO 4762
          property_class: "8.8"
          length_mm: {min: 20, max: 20}
          washer_mm: {min: 0, max: 0}
          torque_nm: {min: 4.0, max: 5.0}
"""


def scene_() -> bt.Scene:
    return bt.Scene(bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf"))


def pack(tmp_path: Path, name: str, text: str) -> Path:
    directory = tmp_path / name
    directory.mkdir()
    (directory / "manifest.yaml").write_text(text)
    return directory


def rows(scene) -> dict:
    return {row["names"][0]: row for row in scene.bom().rows}


def test_a_screw_from_its_pack_carries_the_packs_figures(tmp_path: Path) -> None:
    directory = pack(tmp_path, "screw", SCREW)
    m5 = A.Fastener.from_catalog(directory, length_mm=25, property_class="10.9")
    assert (m5.thread_mm, m5.pitch_mm, m5.length_mm, m5.head_dk_mm, m5.head_k_mm, m5.drive_s_mm) == (5, 0.8, 25, 8.5, 5, 4)
    assert m5.model_name == "ISO 4762 M5x25-10.9" and m5.mass_kg == pytest.approx(0.0056)
    assert m5.torque_ref_nm == 8.65 and m5.catalog[0] == "acme/fastener/iso4762-m5/r1"
    with pytest.raises(ValueError, match="not available"):
        A.Fastener.from_catalog(directory, length_mm=22)
    scene = scene_()
    names = [m5.place(scene, f"screw{i}", (1.0, 0.0, 0.0)) for i in range(3)]
    by = rows(scene)
    line = by[names[0]]
    assert line["qty"] == 3 and line["model"] == "ISO 4762 M5x25-10.9" and line["catalog"].startswith("acme/fastener/iso4762-m5")
    assert line["attributes"]["thread_mm"] == 5.0 and line["attributes"]["length_mm"] == 25.0
    assert line["attributes"]["mass_kg"] == pytest.approx(0.0056) and line["attributes"]["property_class"] == "10.9"
    # The generator alone, at the pack's defaults.
    default = bt.parts.bolt(scene, "one", position=(2.0, 0.0, 0.0), catalog=directory)
    assert rows(scene)[default]["model"] == "ISO 4762 M5x20-8.8"


def test_the_presenter_sizes_itself_from_its_pack_and_fills_its_magazine(tmp_path: Path) -> None:
    scene = scene_()
    feeder = bt.parts.screw_feeder(scene, "feeder", (2.0, 0.5), screws=4,
                                   fastener=A.iso4762(4, 16), catalog=pack(tmp_path, "feeder", FEEDER))
    assert feeder.screws == [f"feeder/screw{i}" for i in range(4)]
    by = rows(scene)
    assert by["feeder"]["model"] == "SP-M4" and by["feeder"]["catalog"].startswith("acme/feeder/presenter")
    assert by["feeder"]["attributes"]["present_s"] == 1.5 and by["feeder"]["attributes"]["length_mm"] == 240.0
    assert by["feeder/present"]["model"] == "SP-M4 presence sensor"
    # The body took the pack's size; the pick point its offset on the top.
    lo, hi = scene.obstacle_bounds("feeder/body")
    assert hi[0] - lo[0] == pytest.approx(0.24) and hi[2] == pytest.approx(0.14)
    (px, py, pz), _ = scene.frame(feeder.pick)
    assert (px, py, pz) == pytest.approx((2.02, 0.44, 0.141))
    with pytest.raises(ValueError, match="not available"):
        bt.parts.screw_feeder(scene, "f2", (3.0, 0.5), screws=2, fastener=A.iso4762(8, 30),
                              catalog=pack(tmp_path, "feeder2", FEEDER))


def test_the_workpiece_set_and_its_joint_come_from_one_pack(tmp_path: Path) -> None:
    scene = scene_()
    directory = pack(tmp_path, "set", WORKPIECE)
    work = bt.parts.workpiece(scene, "set", (1.0, 0.0, 0.5), catalog=directory, cover_at=(1.0, 0.5, 0.5))
    assert (work.housing, work.cover) == ("set/housing", "set/cover") and work.dowels == ["set/dowel/p0", "set/dowel/p1"]
    assert work.seat == ((1.0, 0.0, 0.56), (0.0, 0.0, 0.0, 1.0)) and set(work.frames) == {"set/seat", "set/stock"}
    by = rows(scene)
    assert by["set/housing"]["model"] == "GH-160-H" and by["set/cover"]["model"] == "GH-160-C"
    assert by["set/dowel/p0"]["model"] == "ISO 8734 6m6x16-A" and by["set/dowel/p0"]["qty"] == 2
    lo, hi = scene.obstacle_bounds("set/dowel/p0")
    assert (lo[0] + hi[0]) / 2 == pytest.approx(0.94) and hi[2] == pytest.approx(0.568)
    lo, hi = scene.obstacle_bounds("set/cover")
    assert hi[2] == pytest.approx(0.54, abs=1e-6)  # 10 mm plate and a 30 mm boss at its stock
    joint = A.joint(scene, "cover_joint", a=work.housing, b=work.cover, seat=work.seat, catalog=directory)
    assert [h.id for h in joint.pattern_a.holes] == ["h0", "h1", "h2", "h3"]
    assert joint.fastener.model_name == "ISO 4762 M5x20-8.8" and joint.torque_nm == (4.0, 5.0)
    assert joint.min_engagement_mm == 10 and joint.thickness_m == pytest.approx(0.010)
    assert joint.engagement_mm("h0") == 10.0 and joint.tip_depth_mm("h0") == 10.0
    assert joint.catalog[0] == "acme/workpiece/cover-set/r1"
    (x, y, z), _ = scene.frame(joint.hole_frame("h2"))
    assert (x, y, z) == pytest.approx((1.06, 0.045, 0.57))
    assert A.check(joint) == []
    # A screw the pack does not sell for this joint is still refused.
    with pytest.raises(ValueError, match="engages 6.0 mm"):
        A.joint(scene, "short", a=work.housing, b=work.cover, seat=work.seat, catalog=directory,
                fastener=A.iso4762(5, 16))


def test_a_screwdriver_line_is_checked_against_the_screws_it_drives() -> None:
    arm = bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")
    scene = bt.Scene(arm.attach_tool(bt.tools.screwdriver(), flange="tool0", mount="mount", prefix="drv_"))
    robot = scene.robots[0]
    scene.set_part(f"{robot}/tool", manufacturer="ACME", model="SD-5", category="tool.screwdriver",
                   torque_max_nm=5.0, thread_min_mm=1.6, thread_max_mm=6.0, screw_length_mm=50)
    for i, (thread, length) in enumerate(((5, 20), (6, 30))):
        A.iso4762(thread, length).place(scene, f"screw{i}", (1.0 + 0.1 * i, 0.0, 0.0))
    # Screws know their joint's torque once a fastening declares it — here
    # pinned directly: an M6 joint at its table torque (10.5 N·m).
    scene.set_part("screw1", kind="obstacle", category="fastener", model="ISO 4762 M6x30-8.8",
                   thread_mm=6.0, length_mm=30.0, torque_nm=10.5)
    scene.set_part("screw0", kind="obstacle", category="fastener", model="ISO 4762 M5x20-8.8",
                   thread_mm=5.0, length_mm=20.0, torque_nm=4.5)
    req = scene.requirements()
    row = next(r for r in req.rows if r.category == "tool.screwdriver")
    wants = {r.key: r for r in row.requirements}
    assert wants["torque_nm"].value == 10.5 and wants["torque_nm"].status == "short"
    assert wants["screw_length_mm"].value == 30.0 and wants["screw_length_mm"].status == "ok"
    assert wants["thread_max_mm"].value == 6.0 and wants["thread_max_mm"].status == "ok"
    assert wants["thread_min_mm"].op == "<=" and wants["thread_min_mm"].status == "ok"
    assert "torque_nm" in " ".join(str(f) for f in req.findings())
