"""Product looks that come with their packs (design-shared-props.md P2): a
table whose frame trim draws the board and whose legs stand in from the
edge, an operator panel whose pack draws the enclosure and ships the
operator with it, a screw presenter whose pack draws the body. Massing
collides, trims never do; `detail="plain"` keeps the massing alone."""

import json
from pathlib import Path

import botrail as bt
import pytest

TABLE = """
id: acme/bench/b-1500/r2
kind: spec
category: structure.table
name: B-1500 bench
manufacturer: {name: ACME}
configuration:
  generator: table
  params:
    width_mm: {values: [1500], default: 1500}
    depth_mm: {values: [750], default: 750}
    height_mm: {values: [740], default: 740}
  components:
    - role: frame
      category: structure.table
      part_number: B-1500
      trim: visual/frame.urdf.xacro
      dimensions_mm: {leg: 45, top_thickness: 21, inset_length: 85, inset_width: 70}
      mass: {base_kg: 33.5}
"""
FRAME = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="frame">
  <xacro:arg name="width" default="1"/><xacro:arg name="depth" default="1"/>
  <xacro:arg name="height" default="1"/><xacro:arg name="top_thickness" default="0.03"/>
  <xacro:arg name="inset_length" default="0.02"/><xacro:arg name="inset_width" default="0.02"/>
  <material name="m"><color rgba="0.5 0.5 0.5 1"/></material>
  <link name="root"/>
  <link name="top"><visual><origin xyz="0 0 ${$(arg height) - $(arg top_thickness)/2}"/>
    <geometry><box size="$(arg width) $(arg depth) $(arg top_thickness)"/></geometry><material name="m"/></visual></link>
  <joint name="j_top" type="fixed"><parent link="root"/><child link="top"/></joint>
  <link name="leg"><visual><origin xyz="${$(arg width)/2 - $(arg inset_length)} ${$(arg depth)/2 - $(arg inset_width)} 0.35"/>
    <geometry><box size="0.045 0.045 0.7"/></geometry><material name="m"/></visual></link>
  <joint name="j_leg" type="fixed"><parent link="root"/><child link="leg"/></joint>
</robot>
"""
PANEL = """
id: acme/hmi/station/r2
kind: spec
category: hmi.panel
name: E-stop station
manufacturer: {name: ACME}
configuration:
  generator: operator_panel
  params:
    positions: {values: [1], default: 1}
  components:
    - role: box
      category: hmi.panel
      part_number: ST-1
      trim: visual/box.urdf.xacro
      dimensions_mm: {thickness: 53, pitch: 45, proud: 15.8}
  rules:
    size_mm_by_positions: {"1": [68, 68]}
"""
BOX = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="box">
  <xacro:arg name="width" default="0.1"/><xacro:arg name="height" default="0.1"/><xacro:arg name="thickness" default="0.05"/>
  <material name="lid"><color rgba="0.95 0.62 0.015 1"/></material>
  <link name="root"/>
  <link name="lid"><visual><origin xyz="0 ${-$(arg thickness)/2 - 0.001} 0"/>
    <geometry><box size="$(arg width) 0.002 $(arg height)"/></geometry><material name="lid"/></visual></link>
  <joint name="j_lid" type="fixed"><parent link="root"/><child link="lid"/></joint>
</robot>
"""
FEEDER = """
id: acme/feeder/presenter/r2
kind: spec
category: feeder.screw
name: Presenter
manufacturer: {name: ACME}
configuration:
  generator: screw_feeder
  params:
    thread_mm: {values: [5], default: 5}
  components:
    - role: feeder
      category: feeder.screw
      part_number: SP-M{thread_mm}
      trim: visual/body.urdf.xacro
      dimensions_mm: {length: 220, width: 160, height: 150, pick_x: 0, pick_y: -50}
      mass: {base_kg: 4.0}
  behavior:
    present_s: 1.5
"""
BODY = """<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="body">
  <xacro:arg name="length" default="0.2"/><xacro:arg name="width" default="0.1"/><xacro:arg name="height" default="0.1"/>
  <material name="m"><color rgba="0.57 0.59 0.58 1"/></material>
  <link name="root"/>
  <link name="shell"><visual><origin xyz="0 0 ${$(arg height)/2}"/>
    <geometry><box size="$(arg length) $(arg width) $(arg height)"/></geometry><material name="m"/></visual></link>
  <joint name="j_shell" type="fixed"><parent link="root"/><child link="shell"/></joint>
  <link name="lid"><visual><origin xyz="0 0.03 ${$(arg height) + 0.004}"/>
    <geometry><box size="0.2 0.06 0.004"/></geometry><material name="m"/></visual></link>
  <joint name="j_lid" type="fixed"><parent link="root"/><child link="lid"/></joint>
</robot>
"""


def pack(tmp_path: Path, name: str, manifest: str, trim: str, trim_text: str) -> str:
    directory = tmp_path / name
    (directory / "visual").mkdir(parents=True)
    (directory / "manifest.yaml").write_text(manifest)
    (directory / "visual" / trim).write_text(trim_text)
    return str(directory)


def objects(scene) -> dict:
    return {o["name"]: o for o in json.loads(scene._project_json())["obstacles"]}


def rows(scene) -> dict:
    return {row["names"][0]: row for row in scene.bom().rows}


def test_a_table_pack_puts_the_legs_in_and_draws_the_board_from_its_frame(tmp_path):
    scene = bt.Scene()
    built = bt.parts.table(scene, "bench", position=(1.0, 2.0), catalog=pack(tmp_path, "bench", TABLE, "frame.urdf.xacro", FRAME))
    by = objects(scene)
    # The board that comes with the frame: 21 mm, hidden behind the trim's own `top`.
    assert by["bench/top"]["geometry"]["size"] == [1.5, 0.75, 0.021]
    assert by["bench/top"]["enabled"] and not by["bench/top"]["visible"]
    assert "bench/trim/frame/top" in built.obstacles and not by["bench/trim/frame/top"]["enabled"]
    # The legs stand 85 / 70 mm in from the edge, collide, and hide behind the frame.
    x, y, _ = by["bench/leg2"]["pose"]["position"]
    assert (x, y) == pytest.approx((1.0 + 0.75 - 0.085, 2.0 + 0.375 - 0.070))
    assert by["bench/leg2"]["enabled"] and not by["bench/leg2"]["visible"]
    assert rows(scene)["bench"]["model"] == "B-1500" and scene.bom().total("mass_kg") == pytest.approx(33.5)
    # Plain detail: the massing alone, legs where the pack puts them.
    plain = bt.Scene()
    built = bt.parts.table(plain, "bench", position=(0, 0), catalog=pack(tmp_path, "bench2", TABLE, "frame.urdf.xacro", FRAME),
                           detail="plain")
    assert not any("/trim/" in n for n in built.obstacles) and objects(plain)["bench/top"]["visible"]


def test_a_station_pack_draws_its_enclosure_and_ships_the_operator(tmp_path):
    scene = bt.Scene()
    built = bt.parts.operator_panel(scene, "panel", (0.5, 0.2, 0.9), buttons=("estop",),
                                    catalog=pack(tmp_path, "station", PANEL, "box.urdf.xacro", BOX))
    by = objects(scene)
    assert by["panel/plate"]["geometry"]["size"] == pytest.approx([0.068, 0.053, 0.068])
    assert by["panel/plate"]["enabled"] and not by["panel/plate"]["visible"]
    assert "panel/trim/box/lid" in built.obstacles and not by["panel/trim/box/lid"]["enabled"]
    assert not any(n.startswith("panel/trim/vertical") for n in built.obstacles)   # the pack's look, not the built-in bezel
    assert "panel/estop" in scene.sensor_names and "panel/estop/cap" in built.obstacles
    by_row = rows(scene)
    assert (by_row["panel"]["model"], by_row["panel"]["catalog"]) == ("ST-1", "acme/hmi/station/r2")
    assert by_row["panel/estop"]["model"] == "ST-1 operator (included)" and by_row["panel/estop"]["catalog"] == "acme/hmi/station/r2"
    with pytest.raises(ValueError):
        bt.parts.operator_panel(scene, "two", (0.5, 0.2, 0.9), buttons=("reset", "estop"),
                                catalog=pack(tmp_path, "station2", PANEL, "box.urdf.xacro", BOX))


def test_a_presenter_pack_draws_its_body(tmp_path):
    scene = bt.Scene()
    built = bt.parts.screw_feeder(scene, "feeder", (0.3, 0.3, 0.75), screws=4,
                                  catalog=pack(tmp_path, "presenter", FEEDER, "body.urdf.xacro", BODY))
    by = objects(scene)
    assert by["feeder/body"]["enabled"] and not by["feeder/body"]["visible"]
    assert "feeder/trim/body/shell" in built.obstacles and not by["feeder/trim/body/shell"]["enabled"]
    assert "feeder/rail" in built.obstacles and "feeder/nest" in built.obstacles
    assert rows(scene)["feeder"]["model"] == "SP-M5"
    plain = bt.Scene()
    built = bt.parts.screw_feeder(plain, "feeder", (0.3, 0.3, 0.75), screws=4, detail="plain",
                                  catalog=pack(tmp_path, "presenter2", FEEDER, "body.urdf.xacro", BODY))
    assert not any("/trim/" in n for n in built.obstacles) and objects(plain)["feeder/body"]["visible"]


WORKPIECE = """
id: acme/workpiece/set/r2
kind: spec
category: workpiece
name: Housing and cover
manufacturer: {name: ACME}
configuration:
  generator: workpiece
  params:
    variant: {values: [standard], default: standard}
  components:
    - role: housing
      category: workpiece
      part_number: H-1
      visual: usd/set.usda#/World/Env/housing
      dimensions_mm: {length: 160, width: 120, height: 60}
    - role: cover
      category: workpiece
      part_number: C-1
      visual: usd/set.usda#/World/Env/cover
      dimensions_mm: {length: 160, width: 120, thickness: 10, boss_diameter: 45, boss_height: 30}
"""


def test_a_workpiece_pack_draws_its_parts_as_the_prims_it_ships(tmp_path):
    directory = tmp_path / "set"
    (directory / "usd").mkdir(parents=True)
    (directory / "manifest.yaml").write_text(WORKPIECE)
    source = bt.Scene()
    source.add_box("housing", (0.16, 0.12, 0.06), (0, 0, 0))
    source.add_box("cover", (0.16, 0.12, 0.01), (0, 0, 0.005))
    assert source.export_usd(directory / "usd" / "set.usda") == []
    scene = bt.Scene()
    built = bt.parts.workpiece(scene, "set", (0.5, 0.2, 0.74), catalog=str(directory), cover_at=(0.5, -0.3, 0.74))
    by = objects(scene)
    for name, prim in ((built.housing, "/World/Env/housing"), (built.cover, "/World/Env/cover")):
        asset = by[name]["visual_asset"]
        assert asset["prim_path"] == prim and asset["url"].endswith("set.usda")
        assert asset["transform"] == [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        assert by[name]["enabled"] and by[name].get("material") is None   # the layer's finishes, not an override
    assert rows(scene)["set/housing"]["model"] == "H-1"
    plain = bt.Scene()
    bt.parts.workpiece(plain, "set", (0.5, 0.2, 0.74), catalog=str(directory), detail="plain")
    assert all("visual_asset" not in objects(plain)[n] for n in ("set/housing", "set/cover"))
    # A malformed reference is refused, not silently ignored.
    (directory / "manifest.yaml").write_text(WORKPIECE.replace("usd/set.usda#/World/Env/cover", "usd/set.usda"))
    with pytest.raises(ValueError, match="visual must be"):
        bt.parts.workpiece(bt.Scene(), "set", (0, 0), catalog=str(directory))
