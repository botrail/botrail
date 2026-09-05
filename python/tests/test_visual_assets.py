"""P1 appearance: source → scene → portable project / USD artifact."""
import json
from pathlib import Path
import shutil
import struct
import zipfile
import zlib

import pytest
import botrail as bt


def _png(path: Path, normal=False):
    """Small, deterministic test texture; no optional imaging dependency."""
    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
    rows = b"".join(b"\0" + b"".join(bytes((128, 128, 255) if normal else ((240, 180, 50) if (x + y) % 2 else (40, 90, 220))) for x in range(4)) for y in range(4))
    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", 4, 4, 8, 2, 0, 0, 0)) + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))


def write_visual_assets(root: Path):
    (root / "layers").mkdir(parents=True)
    (root / "textures").mkdir()
    _png(root / "textures/color.png")
    _png(root / "textures/normal.png", True)
    mesh = '''def Mesh "Sample" (prepend apiSchemas = ["MaterialBindingAPI", "PhysicsCollisionAPI"])
    {
        point3f[] points = [(-10,-10,-10),(10,-10,-10),(10,10,-10),(-10,10,-10),(-10,-10,10),(10,-10,10),(10,10,10),(-10,10,10)]
        int[] faceVertexCounts = [4,4,4,4,4,4]
        int[] faceVertexIndices = [0,3,2,1,4,5,6,7,0,1,5,4,3,7,6,2,1,2,6,5,0,4,7,3]
        normal3f[] normals = [(-0.577,-0.577,-0.577),(0.577,-0.577,-0.577),(0.577,0.577,-0.577),(-0.577,0.577,-0.577),(-0.577,-0.577,0.577),(0.577,-0.577,0.577),(0.577,0.577,0.577),(-0.577,0.577,0.577)] (interpolation = "vertex")
        texCoord2f[] primvars:st = [(0,0),(1,0),(1,1),(0,1)] (interpolation = "faceVarying")
        int[] primvars:st:indices = [0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3]
        uniform token subdivisionScheme = "none"
        color3f[] primvars:displayColor = [(0.1,0.7,0.2)]
        rel material:binding = </Library/Looks/Metal>
        uniform token subsetFamily:materialBind:familyType = "nonOverlapping"
        def GeomSubset "Painted" (prepend apiSchemas = ["MaterialBindingAPI"])
        {
            uniform token elementType = "face"
            uniform token familyName = "materialBind"
            int[] indices = [0,1]
            rel material:binding = </Library/Looks/Paint>
        }
    }'''
    looks = '''def Scope "Looks" {
      def Material "Metal" {
        token outputs:surface.connect = </Library/Looks/Metal/Surface.outputs:surface>
        def Shader "Surface" {
          uniform token info:id = "UsdPreviewSurface"
          color3f inputs:diffuseColor.connect = </Library/Looks/Metal/Texture.outputs:rgb>
          normal3f inputs:normal.connect = </Library/Looks/Metal/Normal.outputs:rgb>
          float inputs:metallic = 0.82
          float inputs:roughness = 0.27
          token outputs:surface
        }
        def Shader "UV" {
          uniform token info:id = "UsdPrimvarReader_float2"
          token inputs:varname = "st"
          float2 outputs:result
        }
        def Shader "Texture" {
          uniform token info:id = "UsdUVTexture"
          asset inputs:file = @../textures/color.png@
          token inputs:sourceColorSpace = "sRGB"
          float2 inputs:st.connect = </Library/Looks/Metal/UV.outputs:result>
          float3 outputs:rgb
        }
        def Shader "Normal" {
          uniform token info:id = "UsdUVTexture"
          asset inputs:file = @../textures/normal.png@
          token inputs:sourceColorSpace = "raw"
          float2 inputs:st.connect = </Library/Looks/Metal/UV.outputs:result>
          float3 outputs:rgb
        }
      }
      def Material "Paint" {
        token outputs:surface.connect = </Library/Looks/Paint/Surface.outputs:surface>
        def Shader "Surface" {
          uniform token info:id = "UsdPreviewSurface"
          color3f inputs:diffuseColor = (0.65,0.12,0.04)
          float inputs:metallic = 0
          float inputs:roughness = 0.65
          token outputs:surface
        }
      }
    }'''
    library = root / "layers/library.usda"
    library.write_text('#usda 1.0\n(defaultPrim = "Library")\ndef Xform "Library" {\n' + mesh + '\n' + looks + '\n}')
    cell = root / "cell.usda"
    cell.write_text('''#usda 1.0
(defaultPrim = "World"
 metersPerUnit = 0.01
 upAxis = "Y")
def Xform "World" {
  def Xform "Object" (prepend references = @layers/library.usda@</Library>) {
    double3 xformOp:translate = (80, 30, 20)
    double3 xformOp:scale = (1.5, 1, 0.7)
    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
  }
}
''')
    # An arm with a hand uses the same textured source under different prim paths.
    from test_usd_robot import ARM
    arm = root / "arm.usda"
    arm.write_text(ARM.replace('def Cube "geom" { double size = 0.1 }', 'def Xform "visual" (prepend references = @layers/library.usda@</Library>) {\n double3 xformOp:scale = (0.005,0.005,0.005)\n uniform token[] xformOpOrder = ["xformOp:scale"]\n }'))
    hand = root / "hand.usda"
    hand.write_text('''#usda 1.0
(defaultPrim = "Hand"
 metersPerUnit = 1
 upAxis = "Z")
def Xform "Hand" (prepend apiSchemas = ["PhysicsArticulationRootAPI"]) {
  def Xform "base" (prepend apiSchemas = ["PhysicsRigidBodyAPI"]) {
    def Xform "visual" (prepend references = @layers/library.usda@</Library>) {
      double3 xformOp:scale = (0.003,0.003,0.003)
      uniform token[] xformOpOrder = ["xformOp:scale"]
    }
  }
}
''')
    return cell, arm, hand


def project_json(path):
    if zipfile.is_zipfile(path):
        with zipfile.ZipFile(path) as z:
            return json.loads(z.read("project.json"))
    return json.loads(path.read_text())


def test_static_visual_survives_portable_project_and_python(tmp_path):
    cell, _, _ = write_visual_assets(tmp_path / "source")
    scene = bt.Scene()
    (name,) = scene.load_usd(cell, prefix="cell")
    bounds = scene.obstacle_bounds(name)
    scene.set_obstacle_enabled(name, False)
    scene.set_obstacle_visible(name, False)
    scene.set_part(name, manufacturer="Fixture", model="panel-1")
    project = tmp_path / "cell.botrail"
    scene.save_project(project)
    saved = project_json(project)
    visual = saved["obstacles"][0]["visual_asset"]
    assert visual["prim_path"] == "/World/Object/Sample"
    assert not visual.get("color_override", False)
    with zipfile.ZipFile(project) as z:
        assert sum(n.endswith("textures/color.png") for n in z.namelist()) == 1
        assert sum(n.endswith("textures/normal.png") for n in z.namelist()) == 1
    (tmp_path / "source").rename(tmp_path / "source-unavailable")
    loaded = bt.Scene.load_project(project)
    namespace = {}
    exec(loaded.generate_python().replace("bt.studio(scene)", ""), namespace)
    for candidate in [loaded, namespace["scene"]]:
        assert candidate.obstacle_names == [name]
        for actual, expected in zip(candidate.obstacle_bounds(name), bounds):
            assert actual == pytest.approx(expected, abs=1e-12)
        assert candidate.bom().rows == scene.bom().rows
        candidate.save_project(tmp_path / "again.botrail")
        obstacle = project_json(tmp_path / "again.botrail")["obstacles"][0]
        assert not obstacle["enabled"] and not obstacle["visible"]
        assert obstacle["visual_asset"]["transform"] == pytest.approx(visual["transform"], abs=1e-14)
        assert not obstacle["visual_asset"].get("color_override", False)


def test_composite_visual_sources_keep_link_mapping_after_relocation(tmp_path):
    _, arm, hand = write_visual_assets(tmp_path / "source")
    robot = bt.Robot.from_usd(arm).attach_tool(bt.Robot.from_usd(hand),
        flange="/Robot/link2", mount="/Hand/base", prefix="hand", offset_position=(0.02, 0, 0.1))
    scene = bt.Scene(robot, base_position=(0.3, -0.2, 0.1))
    scene.set_joint_positions([0.5, -0.3])
    original = {name: scene.link_pose(name) for name in robot.link_names}
    project = tmp_path / "composite.botrail"
    scene.save_project(project)
    (tmp_path / "source").rename(tmp_path / "source-unavailable")
    loaded = bt.Scene.load_project(project)
    assert loaded.robot.link_names == robot.link_names
    assert loaded.robot.tcp_link == robot.tcp_link
    assert loaded.joint_positions == pytest.approx(scene.joint_positions)
    for name, pose in original.items():
        p, q = loaded.link_pose(name)
        assert p == pytest.approx(pose[0])
        assert q == pytest.approx(pose[1])
    assert loaded.export_usd(tmp_path / "composite.usda") == []
    text = (tmp_path / "composite.usda").read_text()
    assert "normal3f[] normals" in text and "primvars:st" in text
    assert len(list((tmp_path / "composite_assets").rglob("color.png"))) == 2


def test_usd_artifacts_keep_scalar_and_source_materials(tmp_path):
    Usd = pytest.importorskip("pxr.Usd")
    UsdShade = pytest.importorskip("pxr.UsdShade")
    UsdGeom = pytest.importorskip("pxr.UsdGeom")
    cell, _, _ = write_visual_assets(tmp_path / "source")
    scene = bt.Scene()
    (name,) = scene.load_usd(cell)
    scene.add_box("paint", (0.2,0.2,0.2), (2,0,0), color=(0.1,0.3,0.6))
    scene.set_obstacle_material("paint", metalness=0.0, roughness=0.52)
    scene.set_physics("paint", friction=0.7)
    source = Usd.Stage.Open(str(cell))
    source_mesh = source.GetPrimAtPath(name)
    for extension in ["usda", "usdc"]:
        path = tmp_path / f"scene.{extension}"
        assert scene.export_usd(path) == []
        stage = Usd.Stage.Open(str(path))
        mesh = stage.GetPrimAtPath("/World/Env" + name)
        for attr in ["points", "normals", "primvars:st", "primvars:st:indices", "faceVertexCounts", "faceVertexIndices"]:
            assert mesh.GetAttribute(attr).Get() == source_mesh.GetAttribute(attr).Get(), attr
        assert mesh.GetAttribute("normals").GetMetadata("interpolation") == "vertex"
        material, _ = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        shader = material.ComputeSurfaceSource()[0]
        assert shader.GetInput("metallic").Get() == pytest.approx(0.82)
        assert shader.GetInput("roughness").Get() == pytest.approx(0.27)
        tex = shader.GetInput("diffuseColor").GetConnectedSource()[0]
        assert Path(UsdShade.Shader(tex.GetPrim()).GetInput("file").Get().resolvedPath).is_file()
        subset = stage.GetPrimAtPath(str(mesh.GetPath()) + "/Painted")
        mat, _ = UsdShade.MaterialBindingAPI(subset).ComputeBoundMaterial()
        assert mat.ComputeSurfaceSource()[0].GetInput("metallic").Get() == 0
        paint = stage.GetPrimAtPath("/World/Env/paint")
        mat, _ = UsdShade.MaterialBindingAPI(paint).ComputeBoundMaterial()
        assert mat.ComputeSurfaceSource()[0].GetInput("roughness").Get() == pytest.approx(0.52)
        assert paint.GetRelationship("material:binding:physics").GetTargets()
        bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(mesh).ComputeAlignedRange()
        # Exact visual bounds, independently computed from the authored 20cm
        # cube, nonuniform scale and Y-up placement. Collision uses VHACD.
        assert tuple(bbox.GetMin()) == pytest.approx((0.65, -0.27, 0.2), abs=1e-6)
        assert tuple(bbox.GetMax()) == pytest.approx((0.95, -0.13, 0.4), abs=1e-6)


def test_texture_overrides_and_exported_package_relocate(tmp_path):
    Usd = pytest.importorskip("pxr.Usd")
    UsdShade = pytest.importorskip("pxr.UsdShade")
    cell, _, _ = write_visual_assets(tmp_path / "source")
    scene = bt.Scene()
    (name,) = scene.load_usd(cell)
    scene.set_obstacle_color(name, (0.15, 0.4, 0.8))
    scene.set_obstacle_material(name, metalness=0.25, roughness=0.6, opacity=0.24)
    output = tmp_path / "output"
    output.mkdir()
    for ext in ["usda", "usdc"]:
        assert scene.export_usd(output / f"overrides.{ext}") == []
    shutil.move(output, tmp_path / "relocated")
    (tmp_path / "source").rename(tmp_path / "source-unavailable")
    for ext in ["usda", "usdc"]:
        stage = Usd.Stage.Open(str(tmp_path / "relocated" / f"overrides.{ext}"))
        mesh = stage.GetPrimAtPath("/World/Env" + name)
        for prim in [mesh, stage.GetPrimAtPath(str(mesh.GetPath()) + "/Painted")]:
            mat, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()
            shader = mat.ComputeSurfaceSource()[0]
            assert shader.GetInput("metallic").Get() == pytest.approx(0.25)
            assert shader.GetInput("roughness").Get() == pytest.approx(0.6)
            assert shader.GetInput("opacity").Get() == pytest.approx(0.24)
            connected = shader.GetInput("diffuseColor").GetConnectedSource()
            if connected:
                texture = UsdShade.Shader(connected[0].GetPrim())
                assert tuple(texture.GetInput("scale").Get()) == pytest.approx((0.15,0.4,0.8,1))
                image = Path(texture.GetInput("file").Get().resolvedPath)
                assert image.is_file() and image.is_relative_to(tmp_path / "relocated")
                assert shader.GetInput("normal").GetConnectedSource()
            else:
                assert tuple(shader.GetInput("diffuseColor").Get()) == pytest.approx((0.15,0.4,0.8))


def test_usdz_visual_and_uniform_face_colors(tmp_path):
    Usd = pytest.importorskip("pxr.Usd")
    UsdUtils = pytest.importorskip("pxr.UsdUtils")
    UsdShade = pytest.importorskip("pxr.UsdShade")
    cell, _, _ = write_visual_assets(tmp_path / "source")
    library = cell.parent / "layers/library.usda"
    library.write_text(library.read_text().replace(
        'color3f[] primvars:displayColor = [(0.1,0.7,0.2)]',
        'color3f[] primvars:displayColor = [(1,0,0),(0,1,0),(0,0,1),(1,1,0),(0,1,1),(1,0,1)] (interpolation = "uniform")'))
    packed = tmp_path / "cell.usdz"
    assert UsdUtils.CreateNewUsdzPackage(str(cell), str(packed))
    scene = bt.Scene()
    (name,) = scene.load_usd(packed)
    original = Usd.Stage.Open(str(packed))
    for extension in ["usda", "usdc"]:
        output = tmp_path / f"packed.{extension}"
        scene.export_usd(output)
        stage = Usd.Stage.Open(str(output))
        mesh = stage.GetPrimAtPath("/World/Env" + name)
        assert mesh.GetAttribute("primvars:displayColor").Get() == original.GetPrimAtPath(name).GetAttribute("primvars:displayColor").Get()
        assert mesh.GetAttribute("primvars:displayColor").GetMetadata("interpolation") == "uniform"
        mat, _ = UsdShade.MaterialBindingAPI(mesh).ComputeBoundMaterial()
        shader = mat.ComputeSurfaceSource()[0]
        texture = UsdShade.Shader(shader.GetInput("diffuseColor").GetConnectedSource()[0].GetPrim())
        assert ".usdz[" in texture.GetInput("file").Get().resolvedPath
