"""The shape library: unit-box USD layers vendored from botrail-assets
(`workshop-shapes/`), drawn onto obstacles by `bt.parts.appearance` so the
picture travels with its resident while collision stays the box."""

import json
import shutil
from pathlib import Path

import botrail as bt
import pytest

LIBRARY = Path(bt.parts.__file__).resolve().parent / "_shapes"


def test_the_library_lists_what_it_ships():
    shipped = sorted(p.stem for p in LIBRARY.glob("*.usda"))
    assert shipped == list(bt.parts.SHAPES)
    record = json.loads((LIBRARY / "SOURCES.json").read_text(encoding="utf-8"))
    assert sorted(Path(name).stem for name in record["files"]) == shipped
    assert record["repository"].endswith("/botrail-assets") and record["path"] == "workshop-shapes/usd"
    assert bt.parts.shape_path("carton") == LIBRARY / "carton.usda"
    with pytest.raises(ValueError, match="unknown shape"):
        bt.parts.shape_path("bucket")


def test_every_layer_is_one_unit_box_mesh_with_real_holes():
    Usd = pytest.importorskip("pxr.Usd")
    UsdGeom = pytest.importorskip("pxr.UsdGeom")
    Gf = pytest.importorskip("pxr.Gf")
    triangles = []
    for name in bt.parts.SHAPES:
        stage = Usd.Stage.Open(str(bt.parts.shape_path(name)))
        prim = stage.GetPrimAtPath(f"/Shapes/{name}")
        assert prim.IsA(UsdGeom.Mesh), name
        assert [c.GetTypeName() for c in prim.GetChildren()] and all(c.IsA(UsdGeom.Subset) for c in prim.GetChildren()), name
        bounds = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(prim).ComputeAlignedRange()
        assert tuple(bounds.GetMin()) == pytest.approx((-.5, -.5, -.5), abs=1e-6), name
        assert tuple(bounds.GetMax()) == pytest.approx((.5, .5, .5), abs=1e-6), name
        if name == "workpiece":
            mesh = UsdGeom.Mesh(prim)
            transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
            points = [transform.Transform(Gf.Vec3d(p)) for p in mesh.GetPointsAttr().Get()]
            indices = list(mesh.GetFaceVertexIndicesAttr().Get())
            assert all(n == 3 for n in mesh.GetFaceVertexCountsAttr().Get())
            triangles.extend(tuple(points[j] for j in indices[i:i + 3]) for i in range(0, len(indices), 3))

    def hits_vertical_ray(x, y):
        for a, b, c in triangles:
            det = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
            if abs(det) < 1e-12:
                continue
            u = ((b[1] - c[1]) * (x - c[0]) + (c[0] - b[0]) * (y - c[1])) / det
            v = ((c[1] - a[1]) * (x - c[0]) + (a[0] - c[0]) * (y - c[1])) / det
            if min(u, v, 1 - u - v) >= -1e-8:
                return True
        return False

    assert not hits_vertical_ray(0, 0)  # the central bore goes all the way through
    assert not hits_vertical_ray(.34, .34)  # a mounting hole, not a dark decal
    assert hits_vertical_ray(.38, 0)  # material beside the bores remains


def test_the_appearance_stays_with_one_portable_resident(tmp_path, monkeypatch):
    library = tmp_path / "shapes"
    shutil.copytree(LIBRARY, library)
    monkeypatch.setattr(bt.parts, "_SHAPES_DIR", library)
    scene = bt.Scene()
    scene.add_box("part", (.06, .06, .04), (0, 0, .9))
    assert bt.parts.appearance(scene, "part", "workpiece", (.06, .06, .04)) == "part"
    scene.set_part("part", category="workpiece", mass_kg=.39)
    scene.save_project(tmp_path / "cell.botrail")
    shutil.rmtree(library)
    loaded = bt.Scene.load_project(tmp_path / "cell.botrail")
    assert loaded.obstacle_names == ["part"]  # no independently moving trim pieces
    loaded.set_obstacle_pose("part", (2, 3, 1.2))
    obj = json.loads(loaded._project_json())["obstacles"][0]
    assert obj["pose"]["position"] == [2, 3, 1.2]
    assert obj["geometry"]["size"] == [.06, .06, .04]
    assert obj["visual_asset"]["prim_path"] == "/Shapes/workpiece"
    assert obj["visual_asset"].get("color_override", False) is False
    assert Path(obj["visual_asset"]["url"]).is_file()
    assert loaded.export_usd(tmp_path / "relocated.usda") == []


def test_shaped_box_is_decoration_and_tint_follows_the_colour():
    scene = bt.Scene()
    grip = bt.parts.shaped_box(scene, "tray/grip", "handle", (.09, .012, .045), (.4, .3, .8),
                               quaternion=(0, 0, .7071068, .7071068))
    plain = bt.parts.shaped_box(scene, "bench/foot", "adjuster", (.06, .06, .04), (0, 0, .02), color=(.2, .2, .2))
    by = {o["name"]: o for o in json.loads(scene._project_json())["obstacles"]}
    assert (by[grip]["enabled"], by[grip]["visual_asset"]["prim_path"], by[grip]["visual_asset"].get("color_override", False)) == (False, "/Shapes/handle", False)
    assert (by[plain]["enabled"], by[plain]["visual_asset"]["color_override"]) == (False, True)
    assert by[grip]["visual_asset"]["transform"][:3] == [.09, 0, 0]
    assert scene.check_collisions() == []
    with pytest.raises(ValueError, match="positive sides"):
        bt.parts.appearance(scene, grip, "handle", (.09, 0, .045))
