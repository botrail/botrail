"""`bt.parts.prop` — a catalog object (a scanned YCB mug, a cracker box)
placed as one dynamic obstacle (design-rl-tabletop.md T1). A synthetic
package stands in for the catalog: an OBJ whose origin sits off its
footprint and below its underside, the way a scan's does, with an MTL and
a texture, under a URDF whose root is a separate `base` frame."""

import json
import math
import struct
import zlib
from pathlib import Path

import botrail as bt
import pytest

LO = (0.02, -0.05, -0.003)  # the box's corners in the mesh frame: off-centre, like a scan
HI = (0.08, 0.01, 0.12)


def png(path: Path, rgb: tuple[int, int, int]) -> None:
    """A 2 × 2 single-colour PNG, stdlib only."""
    raw = b"".join(b"\x00" + bytes(rgb) * 2 for _ in range(2))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    path.write_bytes(
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", 2, 2, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw))
        + chunk(b"IEND", b"")
    )


def box_obj(path: Path) -> None:
    (x0, y0, z0), (x1, y1, z1) = LO, HI
    corners = [(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)]
    faces = [(1, 3, 2), (2, 3, 4), (5, 6, 7), (6, 8, 7), (1, 2, 5), (2, 6, 5),
             (3, 7, 4), (4, 7, 8), (1, 5, 3), (3, 5, 7), (2, 4, 6), (4, 8, 6)]
    lines = ["mtllib thing.mtl"]
    lines += [f"v {x} {y} {z}" for x, y, z in corners]
    lines += ["vt 0 0", "vt 1 0", "vt 1 1"]
    lines += ["usemtl skin"] + [f"f {a}/1 {b}/2 {c}/3" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n")


def make_package(root: Path, *, joint: str = "fixed", origin: str = "", visuals: int = 1) -> Path:
    directory = root / "demo" / "objects" / "thing" / "r1"
    (directory / "urdf").mkdir(parents=True)
    (directory / "visual").mkdir()
    box_obj(directory / "visual" / "thing.obj")
    (directory / "visual" / "thing.mtl").write_text("newmtl skin\nKd 1 1 1\nmap_Kd skin.png\n")
    png(directory / "visual" / "skin.png", (200, 40, 30))
    visual = '<visual><geometry><mesh filename="../visual/thing.obj"/></geometry></visual>' * visuals
    (directory / "urdf" / "model.urdf").write_text(
        f'<robot name="thing"><link name="thing">{visual}</link><link name="base"/>'
        f'<joint name="base_joint" type="{joint}"><parent link="base"/><child link="thing"/>{origin}'
        '<axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/></joint></robot>'
    )
    (directory / "manifest.yaml").write_text(
        "id: demo/objects/thing/r1\nkind: model\ncategory: workpiece\nname: Thing (demo 001)\n"
        "manufacturer:\n  name: Demo Set\n"
        "specs:\n  mass_kg: 0.25\n  albedo_rgb: [0.5, 0.1, 0.05]\n"
        "assets:\n  urdf: urdf/model.urdf\n"
    )
    return directory


def centre(scene, name):
    lo, hi = scene.obstacle_bounds(name)
    return ((lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2), lo[2], hi[2]


def test_a_prop_stands_its_footprint_on_the_point_and_is_identified(tmp_path: Path) -> None:
    package = make_package(tmp_path)
    scene = bt.Scene()
    name = bt.parts.prop(scene, "thing", (0.4, 0.2, 0.75), catalog=str(package))
    assert name == "thing" and scene.obstacle_names == ["thing"]
    (cx, cy), bottom, top = centre(scene, name)
    assert (cx, cy, bottom) == pytest.approx((0.4, 0.2, 0.75), abs=1e-9)
    assert top - bottom == pytest.approx(HI[2] - LO[2])
    # The flat colour for colour-only consumers is the pack's albedo.
    assert scene.obstacle_color(name) == pytest.approx((0.5, 0.1, 0.05))
    # One BOM row: the catalog id, the pack's name, maker, category, mass.
    row = next(r for r in scene.bom().rows if r["names"] == ["thing"])
    assert row["model"] == "Thing (demo 001)"
    assert row["manufacturer"] == "Demo Set" and row["category"] == "workpiece"
    assert row["attributes"]["mass_kg"] == 0.25
    obstacle = next(o for o in json.loads(scene._project_json())["obstacles"] if o["name"] == "thing")
    assert obstacle["physics"]["dynamic"] and obstacle["physics"]["mass"] == 0.25


def test_yaw_turns_the_prop_about_its_footprint_and_overrides_apply(tmp_path: Path) -> None:
    package = make_package(tmp_path)
    scene = bt.Scene()
    name = bt.parts.prop(scene, "thing", (1.0, -1.0), catalog=str(package), yaw=math.pi / 2,
                         mass_kg=0.4, color=(0.2, 0.2, 0.2), dynamic=False)
    lo, hi = scene.obstacle_bounds(name)
    # A quarter turn swaps the footprint's sides about the same centre.
    assert (hi[0] - lo[0], hi[1] - lo[1]) == pytest.approx((HI[1] - LO[1], HI[0] - LO[0]))
    (cx, cy), bottom, _ = centre(scene, name)
    assert (cx, cy, bottom) == pytest.approx((1.0, -1.0, 0.0), abs=1e-9)
    assert scene.obstacle_color(name) == pytest.approx((0.2, 0.2, 0.2))
    row = next(r for r in scene.bom().rows if r["names"] == ["thing"])
    assert row["attributes"]["mass_kg"] == 0.4


def test_the_models_own_frames_are_composed(tmp_path: Path) -> None:
    """A model whose mesh hangs off its root (a fixed joint turning it a
    quarter about x — a Y-up mesh stood up) still lands on its footprint."""
    package = make_package(tmp_path, origin='<origin xyz="0.3 0 0" rpy="1.5707963267948966 0 0"/>')
    scene = bt.Scene()
    name = bt.parts.prop(scene, "thing", (0.0, 0.0, 0.5), catalog=str(package))
    lo, hi = scene.obstacle_bounds(name)
    # Turned about x, the box's z extent lies along y and its y extent along z.
    assert (hi[1] - lo[1], hi[2] - lo[2]) == pytest.approx((HI[2] - LO[2], HI[1] - LO[1]))
    (cx, cy), bottom, _ = centre(scene, name)
    assert (cx, cy, bottom) == pytest.approx((0.0, 0.0, 0.5), abs=1e-9)


def test_a_prop_is_one_rigid_mesh(tmp_path: Path) -> None:
    scene = bt.Scene()
    with pytest.raises(ValueError, match="one rigid body"):
        bt.parts.prop(scene, "a", (0, 0), catalog=str(make_package(tmp_path / "a", joint="revolute")))
    with pytest.raises(ValueError, match="2 visual meshes"):
        bt.parts.prop(scene, "b", (0, 0), catalog=str(make_package(tmp_path / "b", visuals=2)))
    bare = make_package(tmp_path / "c")
    (bare / "manifest.yaml").write_text("id: demo/objects/thing/r1\nkind: model\n")
    with pytest.raises(ValueError, match="ships no model"):
        bt.parts.prop(scene, "c", (0, 0), catalog=str(bare))


def test_a_prop_rests_where_it_was_set_down_under_physics(tmp_path: Path) -> None:
    package = make_package(tmp_path)
    scene = bt.Scene()
    scene.add_box("table", size=(1.0, 1.0, 0.05), position=(0.4, 0.2, 0.725))
    name = bt.parts.prop(scene, "thing", (0.4, 0.2, 0.752), catalog=str(package))
    scene.sequence("run").step("wait", transition=bt.seq.elapsed(1.5))
    tl = scene.simulate_sequence("run", physics=True)
    start, _ = scene.obstacle_pose(name)
    end, quat = tl.object_pose(name, tl.duration)
    assert end[2] == pytest.approx(start[2] - 0.002, abs=0.003)  # down the 2 mm gap, onto the table
    assert (end[0], end[1]) == pytest.approx((start[0], start[1]), abs=1e-3)
    assert abs(quat[3]) == pytest.approx(1.0, abs=1e-3)  # upright


def test_a_saved_project_carries_the_mesh_its_material_and_texture(tmp_path: Path) -> None:
    package = make_package(tmp_path)
    scene = bt.Scene()
    bt.parts.prop(scene, "thing", (0.4, 0.2, 0.75), catalog=str(package))
    saved = tmp_path / "cell.botrail"
    scene.save_project(saved)
    moved = tmp_path / "elsewhere"
    saved.rename(moved.with_suffix(".botrail"))
    loaded = bt.Scene.load_project(moved.with_suffix(".botrail"))
    (cx, cy), bottom, _ = centre(loaded, "thing")
    assert (cx, cy, bottom) == pytest.approx((0.4, 0.2, 0.75), abs=1e-9)
