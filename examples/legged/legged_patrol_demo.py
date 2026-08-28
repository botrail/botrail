"""A quadruped on patrol through a working cell.

To the cell, a legged robot is a vehicle with legs. It is dispatched with
`bt.seq.goto`, it arrives with `bt.seq.device_done`, it has to fit through
the gate like any AGV and stay out of the arm's way like any AGV — and the
legs are not a program. They are a `bt.Gait` on the mount: the `goto` that
sends the vehicle is what makes the machine walk, and when it stops it
stands. Nothing here is simulated physically. The body rides the vehicle's
closed-form motion, the footfalls are planned from that motion the moment
the vehicle is dispatched, a planted foot never moves in the world, and the
legs are solved every scan tick. What the bake answers is what it answers
for any vehicle — does it fit, does it clash, how long is the cycle — with
legs that move like legs instead of a body that hovers.

The robot is a Unitree Go2. It comes from the catalog when it can
(`unitree/go2/go2`, a `vehicle.legged` package whose manifest carries the
gait — `bt.Gait.from_catalog` reads it, so no joint name is copied here),
and otherwise straight from Unitree's own description (`unitree_ros`,
BSD-3-Clause): the first run fetches the URDF and its meshes (~20 MB of
COLLADA) into the botrail cache and converts them to OBJ, since botrail
reads STL/OBJ. `--robot <dir>` runs the cell on a package built locally by
the catalog builder (`bcb build recipes/unitree/go2.yaml`), and `--robot
quad` on the primitive quadruped in `examples/assets/quad_test.urdf`, with no
download at all — that is what the tests use. `--compare` bakes the same
cell on every candidate (the built-ins and any package directories named
after it) and tables what fits and how long the cycle takes.

The cycle is the one a dog is bought for in a cell — carrying:

  * **到着** — the dog walks in from the yard, through the gate, and docks
    at the arm's station. The arm, part in hand, has been waiting on
    `dog_docked`, a zone that watches the dog.
  * **積載** — the arm sets the part down on the dog's back and lets go.
    From that moment it is cargo: the tray zone rides in the vehicle's
    frame, so whatever is released inside it travels — no load action,
    the same rule the AGV cell and the conveyor use.
  * **発進と配送** — the departure permit asks two things, both read off
    the world: `tray_loaded` (a sensor mounted on the dog — it keeps
    answering out on the walkway) and `arm_over_dock` off. Then the dog
    walks the part to the bay, turns, and comes back out.

Two things the bake checks that a picture would not:

  * **The gate.** The dog's footprint rides the vehicle as its body, so a
    gate too narrow for it fails with a `VehicleCollision` naming the panel
    it hit (`--narrow`). The legs are not checked against the environment
    tick by tick — the footprint is what is.
  * **The stride.** The vehicle's speed and the gait's period together
    decide how far a foot has to reach; a combination the legs cannot take
    is refused by name before the bake, and one they just barely cannot
    take fails mid-walk with the leg and the time.

Run with:  python examples/legged/legged_patrol_demo.py [out.usdc] [--robot go2|quad|<package dir>]
                                                 [--narrow] [--studio]
                                                 [--compare [<package dir> ...]]
"""

from __future__ import annotations

import argparse
import math
import os
import re
import sys
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

import botrail as bt  # noqa: E402

ASSETS = Path(__file__).resolve().parents[1] / "assets"

# ------------------------------------------------------------- the machine
GO2_REPO = "https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/go2_description"
GO2_MESHES = ["base", "hip", "thigh", "thigh_mirror", "calf", "calf_mirror", "foot"]

# Go2: 12 joints, point feet of radius 22 mm. The stance is Unitree's own
# standing pose (thigh 0.8, calf -1.5) — it puts each foot straight under
# its hip, 0.31 m below the body.
GO2_GAIT = {
    "legs": {n: f"{n}_foot" for n in ("FL", "FR", "RL", "RR")},
    "stance": {f"{n}_{j}_joint": v for n in ("FL", "FR", "RL", "RR")
               for j, v in (("hip", 0.0), ("thigh", 0.8), ("calf", -1.5))},
    "pattern": "trot", "period": 0.45, "lift": 0.07, "max_stride": 0.45, "foot_radius": 0.022,
    # The step Unitree rates it for. The catalog package says the same
    # (`max_step_height_mm`), so both ways of loading the dog agree.
    "max_step": 0.16,
}
GO2_FOOTPRINT = (0.66, 0.31, 0.40)   # the body the gate has to pass, metres
GO2_SPEED, GO2_TURN = 0.6, 1.0       # m/s on a straight, rad/s in a pivot
GO2_PACKAGE = "unitree/go2/go2"      # the catalog package, tried first
_FALLBACK_TOLD: set = set()          # so a --compare says it once

# The primitive quadruped: 0.2 m thigh and calf, 20 mm foot balls.
QUAD_GAIT = {
    "legs": {n: f"{n}_foot" for n in ("FL", "FR", "RL", "RR")},
    "stance": {f"{n}_{j}_joint": v for n in ("FL", "FR", "RL", "RR")
               for j, v in (("hip", 0.0), ("thigh", 0.7), ("calf", -1.4))},
    "pattern": "trot", "period": 0.5, "lift": 0.05, "max_stride": 0.5, "foot_radius": 0.02,
}
QUAD_FOOTPRINT = (0.64, 0.42, 0.36)
QUAD_SPEED, QUAD_TURN = 0.5, 1.0

# ---------------------------------------------------------------- the cell
YARD, DOCK, CORNER, BAY = (-3.6, 0.0), (2.0, 0.0), (3.4, 0.0), (3.4, 1.4)
FENCE_X, FENCE_Y = 0.0, 2.6          # the cell boundary the walkway crosses
GATE_HALF = 0.55                     # half the gate opening (1.1 m)
NARROW_HALF = 0.14                   # `--narrow`: 0.28 m, which a Go2 is not
ARM_BASE, PEDESTAL_H = (2.0, -0.50), 0.70
BENCH, BENCH_TOP = (2.0, -0.95), 0.80
SEAT_GAP = 0.006                     # a part to be grasped rests proud of its surface
PART = (0.08, 0.08, 0.06)
# The tool's own 40 mm hang below its tip frame: targets sit that much
# higher than the surface they address.
PICK = (2.0, -0.95, BENCH_TOP + SEAT_GAP + PART[2] + 0.05)   # tool tip over the part
HANDOVER = (2.0, -0.02, 0.53)        # tool tip over the dog's back at the dock
DOWN = (1.0, 0.0, 0.0, 0.0)          # tool pointing at the floor
# Elbow-up seeds for the two taught poses: the solve is warm-started here
# and runs without restarts, so it lands on the posture a programmer would
# teach rather than on a folded one.
PICK_SEED = [-math.pi / 2, 0.8, 0.6, 1.74, 0.0, 0.0]
HANDOVER_SEED = [math.pi / 2, 1.5, 1.0, 0.64, 0.0, 0.0]

STEEL = (0.32, 0.35, 0.40)
WOOD = (0.45, 0.33, 0.20)


# ------------------------------------------------------------ COLLADA -> OBJ
def _mat_mul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _identity():
    return [[1.0 if i == j else 0.0 for j in range(4)] for i in range(4)]


def _node_matrix(node, ns):
    """A node's local transform: its transform elements, in document order."""
    m = _identity()
    for el in node:
        tag = el.tag.split("}")[1]
        if tag == "matrix":
            v = [float(x) for x in el.text.split()]
            m = _mat_mul(m, [v[0:4], v[4:8], v[8:12], v[12:16]])
        elif tag == "translate":
            x, y, z = (float(s) for s in el.text.split())
            m = _mat_mul(m, [[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])
        elif tag == "rotate":
            x, y, z, deg = (float(s) for s in el.text.split())
            n = math.sqrt(x * x + y * y + z * z) or 1.0
            x, y, z = x / n, y / n, z / n
            c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
            t = 1 - c
            m = _mat_mul(m, [[t * x * x + c, t * x * y - s_ * z, t * x * z + s_ * y, 0],
                             [t * x * y + s_ * z, t * y * y + c, t * y * z - s_ * x, 0],
                             [t * x * z - s_ * y, t * y * z + s_ * x, t * z * z + c, 0],
                             [0, 0, 0, 1]])
        elif tag == "scale":
            x, y, z = (float(s) for s in el.text.split())
            m = _mat_mul(m, [[x, 0, 0, 0], [0, y, 0, 0], [0, 0, z, 0], [0, 0, 0, 1]])
    return m


def collada_to_obj(src: Path, dst: Path) -> int:
    """Writes `dst` (.obj, with a .mtl beside it) from a COLLADA file:
    geometry under the visual scene's node transforms, metres, Z-up, and the
    diffuse colour of each material. Returns the triangle count. Enough for
    a URDF's meshes; not a general importer."""
    root = ET.parse(src).getroot()
    ns = {"c": root.tag[1:].split("}")[0]}
    def tag(name: str) -> str:
        return "{" + ns["c"] + "}" + name

    scale, up = 1.0, "Z_UP"
    asset = root.find("c:asset", ns)
    if asset is not None:
        unit = asset.find("c:unit", ns)
        if unit is not None and unit.get("meter"):
            scale = float(unit.get("meter"))
        up_el = asset.find("c:up_axis", ns)
        if up_el is not None and up_el.text:
            up = up_el.text.strip()

    sources = {}
    for s in root.iter(tag("source")):
        arr = s.find("c:float_array", ns)
        if arr is None:
            continue
        acc = s.find("c:technique_common/c:accessor", ns)
        stride = int(acc.get("stride", "3")) if acc is not None else 3
        sources[s.get("id")] = ([float(x) for x in arr.text.split()], stride)
    effects = {}
    for e in root.iter(tag("effect")):
        rgb = (0.7, 0.7, 0.7)
        for d in e.iter(tag("diffuse")):
            col = d.find("c:color", ns)
            if col is not None:
                rgb = tuple(float(x) for x in col.text.split()[:3])
        effects[e.get("id")] = rgb
    materials = {}
    for m in root.iter(tag("material")):
        inst = m.find("c:instance_effect", ns)
        if inst is not None:
            materials[m.get("id")] = inst.get("url", "").lstrip("#")

    geometries = {}
    for g in root.iter(tag("geometry")):
        mesh = g.find("c:mesh", ns)
        if mesh is None:
            continue
        vert_pos = {}
        for v in mesh.findall("c:vertices", ns):
            for inp in v.findall("c:input", ns):
                if inp.get("semantic") == "POSITION":
                    vert_pos[v.get("id")] = inp.get("source").lstrip("#")
        prims = []
        for prim in list(mesh.findall("c:triangles", ns)) + list(mesh.findall("c:polylist", ns)):
            inputs = prim.findall("c:input", ns)
            stride = max(int(i.get("offset", "0")) for i in inputs) + 1
            vin = next(i for i in inputs if i.get("semantic") == "VERTEX")
            p_el = prim.find("c:p", ns)
            if p_el is None or not p_el.text:
                continue
            idx = [int(x) for x in p_el.text.split()][int(vin.get("offset", "0"))::stride]
            if prim.tag == tag("polylist"):
                tris, k = [], 0
                for n in (int(x) for x in prim.find("c:vcount", ns).text.split()):
                    poly = idx[k:k + n]
                    k += n
                    tris += [(poly[0], poly[j], poly[j + 1]) for j in range(1, n - 1)]
            else:
                tris = [tuple(idx[3 * i:3 * i + 3]) for i in range(len(idx) // 3)]
            prims.append((prim.get("material"), vert_pos[vin.get("source").lstrip("#")], tris))
        geometries[g.get("id")] = prims

    instances = []

    def walk(node, parent):
        m = _mat_mul(parent, _node_matrix(node, ns))
        for ig in node.findall("c:instance_geometry", ns):
            binding = {im.get("symbol"): im.get("target", "").lstrip("#")
                       for im in ig.iter(tag("instance_material"))}
            instances.append((ig.get("url").lstrip("#"), m, binding))
        for child in node.findall("c:node", ns):
            walk(child, m)

    wanted = root.find("c:scene/c:instance_visual_scene", ns)
    wanted = wanted.get("url", "").lstrip("#") if wanted is not None else None
    scenes = list(root.iter(tag("visual_scene")))
    scene = next((s for s in scenes if s.get("id") == wanted), scenes[0] if scenes else None)
    if scene is not None:
        for node in scene.findall("c:node", ns):
            walk(node, _identity())
    if not instances:
        instances = [(gid, _identity(), {}) for gid in geometries]

    vertices, groups = [], {}
    for gid, m, binding in instances:
        for symbol, pos_id, tris in geometries.get(gid, []):
            floats, stride = sources[pos_id]
            base = len(vertices)
            for i in range(len(floats) // stride):
                x, y, z = floats[i * stride:i * stride + 3]
                wx = m[0][0] * x + m[0][1] * y + m[0][2] * z + m[0][3]
                wy = m[1][0] * x + m[1][1] * y + m[1][2] * z + m[1][3]
                wz = m[2][0] * x + m[2][1] * y + m[2][2] * z + m[2][3]
                if up == "Y_UP":
                    wy, wz = -wz, wy
                elif up == "X_UP":
                    wx, wy = -wy, wx
                vertices.append((wx * scale, wy * scale, wz * scale))
            faces = groups.setdefault(binding.get(symbol, symbol) or "default", [])
            faces += [(base + a + 1, base + b + 1, base + c + 1) for a, b, c in tris]

    mtl_name = dst.stem + ".mtl"
    obj = [f"mtllib {mtl_name}"] + [f"v {x:.6f} {y:.6f} {z:.6f}" for x, y, z in vertices]
    mtl = []
    for i, (mat, faces) in enumerate(groups.items()):
        r, g, b = effects.get(materials.get(mat, ""), (0.7, 0.7, 0.7))
        mtl.append(f"newmtl m{i}\nKd {r:.4f} {g:.4f} {b:.4f}")
        obj.append(f"usemtl m{i}")
        obj += [f"f {a} {b} {c}" for a, b, c in faces]
    dst.write_text("\n".join(obj) + "\n")
    (dst.parent / mtl_name).write_text("\n".join(mtl) + "\n")
    return sum(len(f) for f in groups.values())


def fetch_go2() -> Path:
    """Unitree's Go2 description, fetched once into the botrail cache and
    rewritten for botrail: the COLLADA meshes converted to OBJ (colours
    kept), the URDF's `package://` paths pointed at them, and the per-link
    `<material>` placeholders dropped so the meshes' own colours show."""
    cache = Path(os.environ.get("BOTRAIL_CACHE_DIR") or Path.home() / ".cache" / "botrail")
    dest = cache / "assets" / "go2"
    urdf = dest / "go2.urdf"
    if urdf.exists():
        return urdf
    dest.mkdir(parents=True, exist_ok=True)

    def fetch(rel: str) -> Path:
        target = dest / Path(rel).name
        if not target.exists():
            print(f"downloading {rel} ...")
            part = target.with_suffix(target.suffix + ".part")
            urllib.request.urlretrieve(f"{GO2_REPO}/{rel}", part)
            part.rename(target)
        return target

    source = fetch("urdf/go2_description.urdf")
    for name in GO2_MESHES:
        dae = fetch(f"dae/{name}.dae")
        obj = dest / f"{name}.obj"
        if not obj.exists():
            print(f"converting {name}.dae ...")
            collada_to_obj(dae, obj)
    xml = source.read_text()
    xml = re.sub(r'package://go2_description/dae/(\w+)\.dae', r"\1.obj", xml)
    xml = re.sub(r"<material\b[^>]*>.*?</material>", "", xml, flags=re.DOTALL)
    xml = re.sub(r"<material\b[^>]*/>", "", xml)
    urdf.write_text(xml)
    return urdf


# ------------------------------------------------------------------ the cell
def dog_of(robot: str, *, posture: str | None = None):
    """The walker named by `--robot`: its model, gait, footprint, rates.

    `go2` comes from the catalog when the package and the catalog extra are
    there, and from Unitree's URDF otherwise; a directory is a package the
    catalog builder wrote; `quad` is the primitive test quadruped.

    `posture="stairs"` asks the package for its stair posture. Only a
    catalog package carries one — the hand-written fallbacks come back in
    their standing stance, and the caller sees that in `gait.stance`.
    """
    if robot == "go2":
        try:
            return catalog_walker(GO2_PACKAGE, posture=posture)
        except Exception as err:  # noqa: BLE001 — no extra / offline / not published: fall back
            if GO2_PACKAGE not in _FALLBACK_TOLD:
                _FALLBACK_TOLD.add(GO2_PACKAGE)
                print(f"catalog {GO2_PACKAGE} unavailable ({first_line(err)}); reading the URDF")
        return bt.Robot.from_urdf(fetch_go2()), bt.Gait(**GO2_GAIT), GO2_FOOTPRINT, GO2_SPEED, GO2_TURN
    if robot == "quad":
        return (bt.Robot.from_urdf(ASSETS / "quad_test.urdf"), bt.Gait(**QUAD_GAIT),
                QUAD_FOOTPRINT, QUAD_SPEED, QUAD_TURN)
    if Path(robot).is_dir():
        return catalog_walker(Path(robot), posture=posture)
    raise ValueError(f"unknown robot `{robot}` (go2 | quad | a package directory)")


def catalog_walker(package, *, posture: str | None = None):
    """A `vehicle.legged` package as the cell's walker.

    The manifest decides everything the hand-written constants decide for
    the Go2 above: the gait (`locomotion`, read by `bt.Gait.from_catalog`),
    the body the gate has to pass (`specs.footprint_mm` x `height_mm`), and
    the rates — a straight at 60 % of the stride the gait allows, capped
    by the machine's own top speed, which is what the builder's `gait_walk`
    check walked the package at.
    """
    if isinstance(package, Path):
        model = bt.Robot.from_urdf(package / "urdf" / "model.urdf")
        manifest = bt.gait._read_manifest(package)
    else:
        model = bt.Robot.from_catalog(package)
        manifest = bt.gait._read_manifest(bt.gait._package_dir(package, None))
    # Ask what the package has rather than require it: a machine that has
    # never been measured on a flight still walks a floor, and the cell that
    # wanted the posture is the one that should say what it did instead.
    if posture is not None and posture not in bt.Gait.postures(package):
        if (package, posture) not in _FALLBACK_TOLD:
            _FALLBACK_TOLD.add((package, posture))
            print(f"catalog {package} states no `{posture}` posture; using the standing one")
        posture = None
    gait = bt.Gait.from_catalog(package, posture=posture)
    specs = manifest.get("specs") or {}
    # The rates below come from the *walking* gait — a stair posture shortens
    # the swing, not the stride the package was validated at.
    if posture is not None:
        gait.max_stride = bt.Gait.from_catalog(package).max_stride
    length, width = (v / 1e3 for v in (specs.get("footprint_mm") or (700, 310)))
    height = (specs.get("height_mm") or 400) / 1e3
    speed = 0.6 * gait.max_stride / gait.period
    if specs.get("max_speed_mps"):
        speed = min(speed, float(specs["max_speed_mps"]))
    return model, gait, (length, width, height), round(speed, 3), GO2_TURN


def build_scene(robot: str = "go2", narrow: bool = False) -> bt.Scene:
    """The cell: a fenced station with an arm and a bench, a bay beyond it,
    and a dog parked in the yard outside the gate."""
    arm = bt.Robot.from_urdf(ASSETS / "simple_arm.urdf")
    # 10 mm above the plate: a base resting *on* it reads as a collision.
    scene = bt.Scene(arm, name="arm", base_position=(*ARM_BASE, PEDESTAL_H + 0.01))
    bt.parts.pedestal(scene, "pedestal", height=PEDESTAL_H, position=ARM_BASE,
                      top=(0.22, 0.22), model="PB-700", manufacturer="Generic")
    bt.parts.table(scene, "bench", size=(0.9, 0.5, BENCH_TOP), position=BENCH,
                   model="HFS8-900", manufacturer="Generic", color=STEEL)
    scene.add_box("part", PART, (BENCH[0], BENCH[1], BENCH_TOP + SEAT_GAP + PART[2] / 2), color=WOOD)
    bt.parts.rack(scene, "rack", size=(1.2, 0.5, 2.0), position=(3.4, 2.3),
                  model="SR-2000", manufacturer="Generic", color=STEEL)
    bt.parts.table(scene, "bay_table", size=(0.8, 0.6, 0.75), position=(4.2, 1.4),
                   model="HFS8-800", manufacturer="Generic", color=STEEL)

    # The cell boundary the walkway crosses: two fence runs with the gate
    # between them (the perimeter is broken by the walkway, so it is two
    # open runs, like the AGV cell's).
    half = NARROW_HALF if narrow else GATE_HALF
    bt.parts.fence(scene, "fence/north", path=[(FENCE_X, half), (FENCE_X, FENCE_Y), (4.8, FENCE_Y)],
                   height=1.8, closed=False)
    bt.parts.fence(scene, "fence/south", path=[(FENCE_X, -half), (FENCE_X, -FENCE_Y), (4.8, -FENCE_Y)],
                   height=1.8, closed=False)

    # The dog: a vehicle whose legs are a robot. Its footprint rides as the
    # body — that is what the gate check sees; the legs themselves are
    # drawn, solved and checked against the arm, but not against the fence.
    # The tray is the top of its back, in its own frame.
    model, gait, footprint, speed, turn = dog_of(robot)
    scene.add_robot(model, name="dog")
    scene.add_box("dog/footprint", footprint, (YARD[0] + 0.03, YARD[1], footprint[2] / 2))
    scene.set_obstacle_visible("dog/footprint", False)
    scene.add_vehicle("walker", body=["dog/footprint"], path=[YARD, DOCK, CORNER, BAY],
                      stations={"yard": 0, "dock": 1, "bay": 3},
                      speed=speed, turn_speed=turn, start="yard", allow_reverse=True,
                      tray_position=(0.02, 0.0, footprint[2] + 0.06), tray_size=(0.40, 0.30, 0.20))
    scene.mount_robot("walker", robot="dog", gait=gait)

    # The handover, read off the world: the dog at the dock, the arm over it,
    # and — mounted, so it still answers out on the walkway — the part on
    # the dog's back.
    scene.add_zone_sensor("dog_docked", position=(DOCK[0], DOCK[1], 0.25), size=(0.6, 0.6, 0.5),
                          watch_robots=["dog"])
    scene.add_zone_sensor("arm_over_dock", position=(DOCK[0], DOCK[1], 0.75), size=(0.9, 0.6, 0.8),
                          watch_robots=["arm"])
    scene.add_zone_sensor("tray_loaded", position=(0.02, 0.0, footprint[2] + 0.06),
                          size=(0.40, 0.30, 0.20), watch=["part"], mount="walker")
    return scene


def build_cycle(scene: bt.Scene) -> list:
    """Teaches the arm and writes both programs; returns their names."""
    home = list(scene.joint_positions)
    arm, base = scene.robot_of("arm"), scene.robot_base_pose_of("arm")[0]

    def teach(target, seed):
        local = tuple(target[i] - base[i] for i in range(3))   # the base has no yaw
        result = arm.ik(local, DOWN, seed=seed, restarts=0)
        if not result.converged:
            raise RuntimeError(f"the arm cannot reach {target} ({result.pos_error * 1e3:.0f} mm short)")
        return result.q

    scene.add_segment("pick", goal=teach(PICK, PICK_SEED), robot="arm")
    scene.add_segment("to_dog", goal=teach(HANDOVER, HANDOVER_SEED), robot="arm")
    scene.add_segment("home", goal=home, robot="arm")

    # The arm: pick the part, wait for the dog, set it on its back, let go.
    load = scene.sequence("load")
    load.step("pick", actions=[bt.seq.motion("pick")], transition=bt.seq.done())
    load.step("grasp", actions=[bt.seq.attach("part", robot="arm")])
    # Docked = at the dock *and* stopped: the zone alone lights up a body
    # length early, and the vehicle's in-position alone is true in the yard.
    load.step("await dog", transition=bt.seq.all_of(bt.seq.signal("dog_docked"),
                                                    bt.seq.device_done("walker")))
    load.step("to dog", actions=[bt.seq.motion("to_dog")], transition=bt.seq.done())
    load.step("release", actions=[bt.seq.detach("part")])
    load.step("home", actions=[bt.seq.motion("home")], transition=bt.seq.done())

    # The dog: in to the dock, out with the part once it is aboard and the
    # arm is clear, to the bay, and back to the yard.
    patrol = scene.sequence("patrol")
    patrol.step("to dock", actions=[bt.seq.goto("walker", "dock")],
                transition=bt.seq.device_done("walker"))
    patrol.step("loading", transition=bt.seq.all_of(bt.seq.signal("tray_loaded"),
                                                    bt.seq.signal("arm_over_dock", False)))
    patrol.step("to bay", actions=[bt.seq.goto("walker", "bay")],
                transition=bt.seq.device_done("walker"))
    patrol.step("deliver", transition=bt.seq.elapsed(2.0))
    patrol.step("return", actions=[bt.seq.goto("walker", "yard")],
                transition=bt.seq.device_done("walker"))
    return ["patrol", "load"]


def bake(robot: str = "go2", narrow: bool = False):
    """Scene and baked timeline for `robot`."""
    scene = build_scene(robot, narrow)
    names = build_cycle(scene)
    return scene, scene.simulate_sequences(names, max_duration=90.0)


def compare(packages: list, narrow: bool = False) -> None:
    """The same authored cell, baked on every walker.

    Three kinds of answer, from three places: the *package* can rule a
    machine out before the cell is built (no gait, a stance the legs
    cannot hold), the *teaching* can (a back the arm cannot reach), and
    the *bake* can (a body the gate will not pass, a stride the legs
    cannot take). None is an opinion about the machine — they are this
    cell's requirements, met or not.
    """
    candidates = ["quad", "go2"] + [str(p) for p in packages]
    print(f"{'walker':<28} {'body':>16} {'v':>5} {'stride':>6} {'cycle':>7} {'steps':>5}   verdict")
    for name in candidates:
        label = name
        if Path(name).is_dir():
            try:
                manifest = bt.gait._read_manifest(Path(name))
                label = f"{manifest.get('name', Path(name).name)} ({manifest.get('id', '')})"
            except (OSError, ValueError):
                label = Path(name).name
        try:
            _, gait, footprint, speed, _ = dog_of(name)
        except Exception as err:  # noqa: BLE001 — the package's verdict, whatever raised it
            print(f"{label:<28} {'—':>16} {'—':>5} {'—':>6} {'—':>7} {'—':>5}   {first_line(err)}")
            continue
        row = (f"{label:<28} {footprint[0]:4.2f}x{footprint[1]:4.2f}x{footprint[2]:4.2f} "
               f"{speed:5.2f} {gait.max_stride:6.2f}")
        try:
            _, tl = bake(name, narrow)
        except (ValueError, RuntimeError) as err:
            print(f"{row} {'—':>7} {'—':>5}   {first_line(err)}")
            continue
        print(f"{row} {tl.duration:6.2f}s {len(tl.footfalls('dog')):5d}   ok")


def first_line(err: Exception) -> str:
    """The one line of a failure worth putting in a table."""
    text = " ".join(str(err).split())
    return text if len(text) < 110 else text[:107] + "..."


def main() -> None:
    parser = argparse.ArgumentParser(description="A quadruped on patrol through a working cell.")
    parser.add_argument("out", nargs="?", default="cell_legged.usdc")
    parser.add_argument("--robot", default="go2", help="go2 | quad | a catalog package directory")
    parser.add_argument("--narrow", action="store_true", help="a gate the body does not fit")
    parser.add_argument("--studio", action="store_true")
    parser.add_argument("--compare", nargs="*", metavar="PACKAGE_DIR",
                        help="bake the cell on quad, go2 and these packages; table the verdicts")
    args = parser.parse_args()
    if args.compare is not None:
        compare([Path(p) for p in args.compare], args.narrow)
        return
    robot, narrow, out = args.robot, args.narrow, args.out

    scene = build_scene(robot, narrow)
    names = build_cycle(scene)
    if args.studio:
        bt.studio(scene)
        return
    try:
        tl = scene.simulate_sequences(names, max_duration=90.0)
    except ValueError as err:
        print(f"cycle failed: {err}")
        sys.exit(1)

    print(f"cycle time: {tl.duration:.2f}s")
    for step, start, end in tl.step_spans:
        print(f"  {step:<12} {start:6.2f} – {end:6.2f}s")
    lanes = dict(tl.signals)

    def edges(lane: str) -> str:
        return ", ".join(f"{t:.2f}→{'on' if v else 'off'}" for t, v in lanes[lane])

    for lane in ("walker", "dog_docked", "arm_over_dock", "tray_loaded"):
        print(f"  {lane:<15} {edges(lane)}")
    carried = tl.object_pose("part", tl.duration)[0]
    print(f"part ends at {tuple(round(v, 3) for v in carried)} — on the dog's back, back in the yard")
    steps = tl.footfalls("dog")
    strides = [math.dist(a[3][:2], b[3][:2]) for a, b in zip(steps, steps[4:]) if a[0] == b[0]]
    walking = tl.signal("walker").high_total()
    print(f"{len(steps)} footfalls; stride {min(strides):.3f}–{max(strides):.3f} m; "
          f"walking {walking:.2f}s of {tl.duration:.2f}s")

    tl.export_usd(out, fps=60)
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
