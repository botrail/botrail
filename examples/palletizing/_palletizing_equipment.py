"""The end-of-line equipment of the palletizing demo, built from primitives in
the published sizes of the products it stands for.

Nothing here is invented shape: each generator names the product it was
measured against, and the figures it uses are that maker's catalogue values
(sources in `.internal/design-palletizing-line.md`). What the catalogue does
not publish — a bracket's section, a cover's colour — is drawn plainly and
never claimed. Machines that move a load (the wrapper's ring, the turntable)
are robots built from URDF text: a ramp drives their joints, and the load
rides them by `attach`. Machines that only index something (the dispenser's
lift, the labeller's arm) are the ordinary devices.

    Interroll MCP RM 8310   case roller conveyor, 24 V RollerDrive
    Interroll MCP RM 8731   90-degree pop-up belt transfer
    Interroll MCP BM 8420   case belt conveyor
    Interroll MPP PM 9710   pallet roller conveyor (PM 9720 chain, PM 9735 turntable)
    Robopac Genesis Futura  rotary-ring stretch wrapper, top press
    PALOMAT Inline 130201   pallet dispenser, 1200 x 1000 gripped on the long side
    Logopak Series 700      print-and-apply pallet labeller
    Schmalz FA-Xc SVK 442   vacuum area gripper bar (two on the demo's EOAT)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import botrail as bt

# ---- finishes (linear RGB) -----------------------------------------------
RAL_9005 = (0.012, 0.013, 0.014)      # Interroll frames: jet black
RAL_1023 = (0.86, 0.60, 0.00)         # Interroll accent: traffic yellow
RAL_7035 = (0.63, 0.65, 0.64)         # PALOMAT: light grey
ZINC = (0.62, 0.63, 0.66)             # zinc-plated rollers
ALUMINIUM = (0.58, 0.60, 0.62)
GALVANISED = (0.70, 0.72, 0.74)          # the wrapper's bare silver-grey frame
BELT_BLACK = (0.035, 0.035, 0.04)
DARK = (0.06, 0.065, 0.07)

# ---- Interroll MCP (cases) -----------------------------------------------
MCP_BF = 0.62                          # between frames = roller length (420/620/840)
MCP_PROFILE = (0.115, 0.035)           # side profile height x thickness
MCP_ROLLERS = (0.09, 0.05)             # pitch 60/90/120/150, roller 50 mm
MCP_SPAN = 1.5                         # RM 8841 supports

# ---- Interroll MPP (pallets) ---------------------------------------------
MPP_CW = 1.06                          # conveying width 860/1060/1295
MPP_PROFILE = (0.20, 0.07)             # side profile 200 x 70 x 4
MPP_ROLLERS = (0.19, 0.089)            # pitch 175-225, roller 89 mm
MPP_SPAN = 1.5                         # supports at most 1500 apart
MPP_RISE = 0.03                        # profile top above the roller tops


def _yaw(direction) -> float:
    return math.atan2(direction[1], direction[0])


def _quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def _stripe(scene, built: bt.parts.Built, name: str, length: float, position, direction, across: float,
            z: float, height: float, color=RAL_1023) -> None:
    """A painted accent along both outer faces of a conveyor's side profiles."""
    yaw = _yaw(direction)
    nx, ny = -math.sin(yaw), math.cos(yaw)
    for side, s in (("l", 1.0), ("r", -1.0)):
        made = scene.add_box(f"{name}/trim/stripe_{side}", size=(length, 0.003, height),
                             position=(position[0] + nx * s * across, position[1] + ny * s * across, z),
                             quaternion=_quat(yaw), color=color)
        scene.set_obstacle_enabled(made, False)
        built.obstacles.append(made)


def case_roller(scene, name: str, length: float, position, *, direction=(1.0, 0.0), top: float,
                speed: float = 0.5, roller_color=ZINC, model: str = "RM 8310", running: bool = False
                ) -> bt.parts.Built:
    """Interroll MCP RM 8310: zinc rollers 50 mm on a 90 mm pitch between
    black 115 mm profiles with the yellow accent, supports every 1.5 m.
    `position` is the (x, y) of the middle of the run, `top` the roller tops."""
    built = bt.parts.conveyor(
        scene, name, length, MCP_BF, (position[0], position[1], top), direction=direction,
        rollers=MCP_ROLLERS, roller_color=roller_color, belt_thickness=MCP_PROFILE[0], rail=MCP_PROFILE[1],
        leg=0.05, stand_span=MCP_SPAN, color=RAL_9005, detail="full", speed=speed, running=running,
        zone_height=0.40, model=model, manufacturer="Interroll")
    _stripe(scene, built, name, length, position, direction, MCP_BF / 2 + MCP_PROFILE[1] + 0.0015,
            top - 0.035, 0.012)
    return built


def case_belt(scene, name: str, length: float, position, *, direction=(1.0, 0.0), top: float,
              speed: float = 0.5, running: bool = False) -> bt.parts.Built:
    """Interroll MCP BM 8420: the same profile, a black PVC belt over a
    zinc-plated slider bed."""
    built = bt.parts.conveyor(
        scene, name, length, MCP_BF, (position[0], position[1], top), direction=direction,
        belt_thickness=MCP_PROFILE[0], rail=MCP_PROFILE[1], leg=0.05, stand_span=MCP_SPAN, color=RAL_9005,
        detail="full", speed=speed, running=running, zone_height=0.40, model="BM 8420",
        manufacturer="Interroll")
    _stripe(scene, built, name, length, position, direction, MCP_BF / 2 + MCP_PROFILE[1] + 0.0015,
            top - 0.035, 0.012)
    return built


def pallet_roller(scene, name: str, length: float, position, *, direction=(-1.0, 0.0), top: float,
                  speed: float = 0.25, zone_height: float = 1.9, model: str = "PM 9710",
                  running: bool = False) -> bt.parts.Built:
    """Interroll MPP PM 9710: 89 mm rollers between 200 x 70 profiles that
    stand 30 mm proud of them, black with the yellow accent. The transport
    zone is as tall as a loaded pallet, so the load rides with its pallet."""
    built = bt.parts.conveyor(
        scene, name, length, MPP_CW, (position[0], position[1], top), direction=direction,
        rollers=MPP_ROLLERS, belt_thickness=MPP_PROFILE[0] - MPP_RISE, rail=MPP_PROFILE[1], leg=0.08,
        stand_span=MPP_SPAN, color=RAL_9005, detail="full", speed=speed, running=running,
        zone_height=zone_height, model=model, manufacturer="Interroll")
    _stripe(scene, built, name, length, position, direction, MPP_CW / 2 + MPP_PROFILE[1] + 0.0015,
            top - 0.02, 0.03)
    return built


def pallet_chain(scene, name: str, length: float, position, *, direction=(-1.0, 0.0), top: float,
                 speed: float = 0.25, zone_height: float = 1.9) -> bt.parts.Built:
    """Interroll MPP PM 9720: two chain strands 1075 apart on 127 mm strand
    profiles — drawn as the black strands and the deck between them."""
    built = bt.parts.conveyor(
        scene, name, length, 1.137, (position[0], position[1], top), direction=direction,
        belt_thickness=0.127, rail=0.03, leg=0.08, stand_span=MPP_SPAN, color=RAL_9005, detail="full",
        speed=speed, zone_height=zone_height, model="PM 9720", manufacturer="Interroll")
    _stripe(scene, built, name, length, position, direction, 1.137 / 2 + 0.0315, top - 0.03, 0.03)
    return built


# ================================================================ transfer
@dataclass
class Transfer:
    """A 90-degree transfer: `main` carries straight through, `cross` lifts
    the belts and carries the case out at a right angle."""

    name: str
    main: str
    cross: str
    built: bt.parts.Built


def transfer(scene, name: str, position, *, direction=(1.0, 0.0), cross=(0.0, 1.0), top: float,
             length: float = 0.72, speed: float = 0.5, cross_speed: float = 1.0, model: str = "RM 8731") -> Transfer:
    """Interroll MCP RM 8731: a roller module (the rollers carry through)
    with pop-up belt blades between the rollers (the belts carry across).
    Two transport zones on one deck, never both running: the rollers run
    from the start (a case passes straight through), and the sequence turns
    one by stopping them and starting the belts (1.0 m/s, the maker's
    figure; the blades lift in 0.3 s)."""
    built = case_roller(scene, name, length, position, direction=direction, top=top, speed=speed,
                        model=model, running=True)
    yaw = _yaw(direction)
    # The zone of the blades: the same deck, its velocity across.
    zone_h = 0.40
    # It reaches past the frame on both sides, so a case leaving the deck is
    # already in the next module's zone when it leaves this one.
    scene.add_conveyor(f"{name}/cross", zone_position=(position[0], position[1], top + zone_h / 2),
                       zone_size=(length, MCP_BF + 0.32, zone_h),
                       velocity=(cross[0] * cross_speed, cross[1] * cross_speed, 0.0),
                       zone_quaternion=_quat(yaw), running=False)
    built.devices.append(f"{name}/cross")
    scene.set_part(f"{name}/cross", manufacturer="Interroll", model=f"{model} belt blades (included)",
                   category="conveyor.transfer")
    # Three belt blades, 90 mm apart, running across between the rollers:
    # yellow housings, the belts black on top.
    cyaw = _yaw(cross)
    along_x, along_y = math.cos(yaw), math.sin(yaw)
    for i, off in enumerate((-0.09, 0.0, 0.09)):
        made = scene.add_box(f"{name}/trim/blade{i}", size=(MCP_BF - 0.04, 0.024, 0.030),
                             position=(position[0] + along_x * off, position[1] + along_y * off, top - 0.022),
                             quaternion=_quat(cyaw), color=RAL_1023)
        scene.set_obstacle_enabled(made, False)
        built.obstacles.append(made)
        made = scene.add_box(f"{name}/trim/belt{i}", size=(MCP_BF - 0.02, 0.018, 0.006),
                             position=(position[0] + along_x * off, position[1] + along_y * off, top - 0.004),
                             quaternion=_quat(cyaw), color=BELT_BLACK)
        scene.set_obstacle_enabled(made, False)
        built.obstacles.append(made)
    return Transfer(name, name, f"{name}/cross", built)


# ================================================================ turntable
@dataclass
class Turntable:
    name: str            # the robot (the turning deck)
    joint: str
    deck: str            # link the load is attached to
    zone_a: str          # transport along the deck at 0 rad
    zone_b: str          # transport along the deck at +90 deg


def _material(color) -> str:
    """A material name that is the colour: one URDF material per colour, the
    same name in every run."""
    return "c_" + "_".join(f"{round(v * 1000):04d}" for v in color)


def _box_xml(size, xyz=(0, 0, 0), rpy=(0, 0, 0), color=RAL_9005, collide=True) -> str:
    s = " ".join(f"{v:.5f}" for v in size)
    o = f'<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>'
    c = " ".join(f"{v:.4f}" for v in color)
    material = f'<material name="{_material(color)}"><color rgba="{c} 1"/></material>'
    out = f'<visual>{o}<geometry><box size="{s}"/></geometry>{material}</visual>'
    if collide:
        out += f'<collision>{o}<geometry><box size="{s}"/></geometry></collision>'
    return out


def _proxy_xml(xyz) -> str:
    """A 2 mm collision cube at `xyz`, in free air: a link that declares no
    collision is checked as its visual shapes, and a machine's decorations
    (a ring yoke riding its posts, a platen resting on a load) would then
    read as touching the scenery they are drawn against."""
    o = f'<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}"/>'
    return f'<collision>{o}<geometry><box size="0.002 0.002 0.002"/></geometry></collision>'


def _cyl_xml(radius, length, xyz=(0, 0, 0), rpy=(0, 0, 0), color=ZINC, collide=False) -> str:
    o = f'<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>'
    c = " ".join(f"{v:.4f}" for v in color)
    g = f'<geometry><cylinder radius="{radius:.5f}" length="{length:.5f}"/></geometry>'
    out = f'<visual>{o}{g}<material name="{_material(color)}"><color rgba="{c} 1"/></material></visual>'
    if collide:
        out += f'<collision>{o}{g}</collision>'
    return out


def turntable_urdf(name: str, top: float) -> str:
    """Interroll MPP PM 9735 as URDF text: a polygonal base 1890 across, and
    on the turning joint a roller deck — conveying width 1060, conveyor
    length 1380, 89 mm rollers, the black profiles with the yellow accent."""
    base_h = top - 0.20
    base = _box_xml((1.30, 1.30, base_h), (0, 0, base_h / 2), color=RAL_9005)
    base += _box_xml((1.89, 0.60, 0.08), (0, 0, 0.04), color=RAL_9005, collide=False)
    base += _box_xml((0.60, 1.89, 0.08), (0, 0, 0.04), color=RAL_9005, collide=False)
    deck = _box_xml((1.38, 1.06, 0.12), (0, 0, -0.08), color=RAL_9005)
    for side in (-1, 1):
        deck += _box_xml((1.38, 0.07, 0.20), (0, side * 0.565, -0.07), color=RAL_9005, collide=False)
        deck += _box_xml((1.38, 0.003, 0.03), (0, side * 0.6015, -0.02), color=RAL_1023, collide=False)
    n = int(1.30 / 0.19)
    for i in range(n + 1):
        x = -0.65 + i * 1.30 / n
        deck += _cyl_xml(0.0445, 1.06, (x, 0, -0.0445), (math.pi / 2, 0, 0), ZINC)
    # the swept rim painted on the base plate: the yellow warning ring
    deck_link = f'<link name="{name}_deck">{deck}</link>'
    return (f'<?xml version="1.0"?><robot name="{name}">'
            f'<link name="{name}_base">{base}</link>{deck_link}'
            f'<joint name="{name}_turn" type="revolute"><parent link="{name}_base"/><child link="{name}_deck"/>'
            f'<origin xyz="0 0 {top:.5f}"/><axis xyz="0 0 1"/>'
            f'<limit lower="-0.1" upper="{math.radians(270):.5f}" velocity="{math.radians(22.5):.5f}" effort="1"/>'
            f'</joint></robot>')


def turntable(scene, name: str, position, *, top: float, a=(-1.0, 0.0), b=(0.0, 1.0),
              speed: float = 0.25, zone_height: float = 1.9) -> Turntable:
    """The turntable as a robot, plus its two transport zones. 90 degrees in
    4 s (22.5 deg/s), as the maker states."""
    scene.add_robot(bt.Robot.from_urdf_string(turntable_urdf(name, top)), name=name,
                    base_position=(position[0], position[1], 0.0))
    for tag, d in (("a", a), ("b", b)):
        scene.add_conveyor(f"{name}/{tag}", zone_position=(position[0], position[1], top + zone_height / 2),
                           zone_size=(1.38, 1.06, zone_height), velocity=(d[0] * speed, d[1] * speed, 0.0),
                           zone_quaternion=_quat(_yaw(d)), running=False)
    scene.set_part(name, kind="robot", category="conveyor.turntable", manufacturer="Interroll", model="PM 9735")
    scene.set_part(f"{name}/controller", manufacturer="Interroll", model="PM 9735 drive (included)",
                   category="robot_controller")
    for tag in ("a", "b"):
        scene.set_part(f"{name}/{tag}", manufacturer="Interroll", model="PM 9735 roller deck (included)",
                       category="conveyor.roller")
    return Turntable(name, f"{name}_turn", f"{name}_deck", f"{name}/a", f"{name}/b")


# ================================================================ wrapper
@dataclass
class Wrapper:
    name: str                 # the ring machine (robot)
    lift: str                 # ring height joint (m below the top stop)
    ring: str                 # ring rotation joint
    press: str                # top press joint (m below its top stop)
    top: float                # the ring's top stop, world z
    press_top: float          # the platen's top stop, world z
    film: list = field(default_factory=list)   # (source name, z of the band centre)


def wrapper_urdf(name: str, frame_top: float, ring_top: float, press_top: float) -> str:
    """Robopac Genesis Futura 40 moving parts: a square ring frame riding the
    four posts (joint `<name>_lift`, down from the top stop), the rotating
    ring with its film carriage (joint `<name>_ring`, 40 rpm), and the
    pneumatic top press (joint `<name>_press`)."""
    grey = ALUMINIUM
    root = _box_xml((0.30, 0.30, 0.10), (0, 0, frame_top - 0.05), color=grey, collide=False)
    # The ring frame: a square yoke 2.3 x 2.8 at the posts, riding them.
    yoke = ""
    for sx in (-1, 1):
        yoke += _box_xml((0.12, 2.64, 0.10), (sx * 1.07, 0, 0), color=grey, collide=False)
    for sy in (-1, 1):
        yoke += _box_xml((2.02, 0.12, 0.10), (0, sy * 1.26, 0), color=grey, collide=False)
    # The ring: an aluminium annulus about 2.0 across on a yellow drive belt.
    ring = ""
    segments = 28
    r = 1.0
    for i in range(segments):
        a = 2 * math.pi * (i + 0.5) / segments
        ring += _box_xml((0.07, 2 * r * math.sin(math.pi / segments) + 0.01, 0.08),
                         (r * math.cos(a), r * math.sin(a), -0.10), (0, 0, a), color=(0.70, 0.72, 0.74),
                         collide=False)
        ring += _box_xml((0.02, 2 * r * math.sin(math.pi / segments) + 0.01, 0.05),
                         ((r + 0.045) * math.cos(a), (r + 0.045) * math.sin(a), -0.10), (0, 0, a),
                         color=RAL_1023, collide=False)
    # The film carriage and its roll hang inside the ring.
    ring += _box_xml((0.28, 0.34, 0.72), (0.86, 0, -0.48), color=(0.05, 0.12, 0.30), collide=False)
    ring += _cyl_xml(0.11, 0.50, (0.70, 0.0, -0.48), (0, 0, 0), color=(0.92, 0.93, 0.95))
    press = _box_xml((1.20, 1.00, 0.05), (0, 0, 0.0), color=(0.72, 0.73, 0.74), collide=False)
    press += _box_xml((0.10, 0.10, 0.60), (0, 0, 0.32), color=grey, collide=False)
    root += _proxy_xml((0.0, 0.0, frame_top + 0.8))
    yoke += _proxy_xml((0.0, 1.26, 0.0))
    ring += _proxy_xml((0.86, 0.0, -0.48))
    press += _proxy_xml((0.0, 0.0, 0.62))
    return (f'<?xml version="1.0"?><robot name="{name}">'
            f'<link name="{name}_root">{root}</link>'
            f'<link name="{name}_yoke">{yoke}</link><link name="{name}_ring_link">{ring}</link>'
            f'<link name="{name}_platen">{press}</link>'
            f'<joint name="{name}_lift" type="prismatic"><parent link="{name}_root"/><child link="{name}_yoke"/>'
            f'<origin xyz="0 0 {ring_top:.5f}"/><axis xyz="0 0 -1"/>'
            f'<limit lower="0" upper="{ring_top - 0.45:.5f}" velocity="0.5" effort="1"/></joint>'
            f'<joint name="{name}_ring" type="continuous"><parent link="{name}_yoke"/><child link="{name}_ring_link"/>'
            f'<origin xyz="0 0 0"/><axis xyz="0 0 1"/>'
            f'<limit velocity="{40 * 2 * math.pi / 60:.5f}" effort="1"/></joint>'
            f'<joint name="{name}_press" type="prismatic"><parent link="{name}_root"/><child link="{name}_platen"/>'
            f'<origin xyz="0 0 {press_top:.5f}"/><axis xyz="0 0 -1"/>'
            f'<limit lower="0" upper="{press_top - 0.7:.5f}" velocity="0.3" effort="1"/></joint>'
            f'</robot>')


def wrapper(scene, name: str, position, *, top: float, bands: int = 5, pool: int = 2,
            load_top: float = 1.95) -> Wrapper:
    """Robopac Genesis Futura 40 around a pallet-conveyor section: four
    posts about 160 square and 4.67 m tall on a 2.3 x 2.8 m frame, the top
    frame, the ring machine (a robot: ring height, ring turn, top press), and
    the stretch film — `bands` sources of translucent bands, `pool` each, fed
    one by one as the carriage passes their height."""
    x, y = position
    post, height = 0.16, 4.67
    frame = f"{name}/frame"
    for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        scene.add_box(f"{frame}/post{i}", (post, post, height), (x + sx * 1.15, y + sy * 1.40, height / 2),
                      color=GALVANISED)
        scene.add_box(f"{frame}/foot{i}", (0.30, 0.30, 0.015), (x + sx * 1.15, y + sy * 1.40, 0.0075),
                      color=DARK)
    for sy in (-1, 1):
        scene.add_box(f"{frame}/beam_x{'lr'[sy > 0]}", (2.46, post, 0.20), (x, y + sy * 1.40, height - 0.10),
                      color=GALVANISED)
    for sx in (-1, 1):
        scene.add_box(f"{frame}/beam_y{'lr'[sx > 0]}", (post, 2.96, 0.20), (x + sx * 1.15, y, height - 0.10),
                      color=GALVANISED)
    scene.add_box(f"{frame}/drive", (0.9, 0.9, 0.30), (x, y, height + 0.15), color=GALVANISED)
    scene.add_box(f"{frame}/panel", (0.35, 0.60, 1.60), (x + 1.45, y - 1.60, 0.80), color=(0.80, 0.81, 0.82))
    for n in scene.obstacle_names:
        if n.startswith(f"{frame}/"):
            scene.set_obstacle_material(n, metalness=0.6, roughness=0.45)
    ring_top, press_top = height - 0.45, height - 0.75
    scene.add_robot(bt.Robot.from_urdf_string(wrapper_urdf(name, height, ring_top, press_top)), name=name,
                    base_position=(x, y, 0.0))
    # One machine, one line: the moving part carries the identity, its
    # frame and its own control panel come with it.
    scene.set_part(name, kind="robot", category="machine.stretch_wrapper", manufacturer="Robopac",
                   model="Genesis Futura 40", description="rotary-ring stretch wrapper, 40 rpm, top press")
    scene.set_part(f"{name}/controller", manufacturer="Robopac", model="Genesis Futura 40 control panel (included)",
                   category="robot_controller")
    wrap = Wrapper(name, f"{name}_lift", f"{name}_ring", f"{name}_press", ring_top, press_top)
    # The film: `bands` translucent sleeves, each fed onto the load when the
    # carriage passes its height. A source puts a member at its position;
    # the pallet's conveyor zone carries it away with the load.
    band_h = (load_top - top) / bands
    for k in range(bands):
        zc = top + 0.02 + band_h * (k + 0.5)
        members = []
        for p in range(pool):
            m = scene.add_box(f"{name}/film/b{k}_{p}", (1.23, 1.03, band_h + 0.01), (x, y, -3.0 - k - 2 * p),
                              color=(0.80, 0.84, 0.90))
            scene.set_obstacle_material(m, metalness=0.0, roughness=0.15, opacity=0.28)
            scene.set_obstacle_enabled(m, False)
            members.append(m)
        src = f"{name}/film_src{k}"
        scene.add_source(src, pool=members, park=(x, y, -3.0 - k), position=(x, y, zc), interval=0.0,
                         running=False, pitch=(0.0, 0.0, -2.0))
        wrap.film.append((src, zc))
    return wrap


# ================================================================ dispenser
@dataclass
class Dispenser:
    name: str
    lift: str
    stack: list          # pallet names, bottom first


def dispenser(scene, name: str, position, *, top: float, pallets: int, pallet=(1.2, 1.0, 0.162)) -> Dispenser:
    """PALOMAT Inline 130201 over a pallet conveyor: two light-grey side
    units (1542 along the conveyor, 1681 across, 2144 tall) — each a pair of
    lift columns with the gripper beam riding between them — gripping the
    second pallet from the bottom on the long side; the stack of up to 15
    pallets in the magazine between them. Dispensing lifts the stack off the
    bottom pallet (a lift device, 60 mm, whose car is the two gripper beams),
    the conveyor takes it, the stack comes back down onto the grippers."""
    x, y = position
    L, W, H = 1.542, 1.681, 2.144
    depth = (W - 1.02) / 2 - 0.02                 # a side unit, across the conveyor
    col = 0.22                                     # a lift column, along it
    z2 = top + 0.002 + pallet[2] + 0.002 + pallet[2] / 2   # the second pallet's blocks
    car = []
    for side, s in (("l", 1), ("r", -1)):
        yc = y + s * (W / 2 - depth / 2)
        for k, dx in enumerate((-(L / 2 - col / 2), L / 2 - col / 2)):
            scene.add_box(f"{name}/column_{side}{k}", (col, depth, H - 0.25), (x + dx, yc, 0.25 + (H - 0.25) / 2),
                          color=RAL_7035)
            scene.add_box(f"{name}/leg_{side}{k}", (0.10, 0.10, 0.25), (x + dx, yc, 0.125), color=RAL_7035)
            scene.add_box(f"{name}/foot_{side}{k}", (0.20, 0.20, 0.012), (x + dx, yc, 0.006), color=DARK)
        scene.add_box(f"{name}/head_{side}", (L, depth, 0.24), (x, yc, H - 0.12), color=RAL_7035)
        scene.add_box(f"{name}/cover_{side}", (L - 2 * col, 0.02, 0.50), (x, yc + s * (depth / 2 - 0.01), H - 0.49),
                      color=(0.55, 0.57, 0.57))
        # the gripper beam between the columns, and its two grippers on the pallet's side
        beam = scene.add_box(f"{name}/beam_{side}", (L - 2 * col - 0.02, 0.12, 0.14),
                             (x, y + s * (0.5 + 0.03 + 0.06), z2), color=(0.30, 0.31, 0.32))
        car.append(beam)
        for k, dx in enumerate((-0.35, 0.35)):
            car.append(scene.add_box(f"{name}/gripper_{side}{k}", (0.16, 0.04, 0.08), (x + dx, y + s * 0.51, z2),
                                     color=(0.10, 0.10, 0.11)))
    for k, dx in enumerate((-(L / 2 - 0.10), L / 2 - 0.10)):
        scene.add_box(f"{name}/bridge{k}", (0.20, W, 0.12), (x + dx, y, H + 0.06), color=RAL_7035)
    scene.add_box(f"{name}/beacon", (0.08, 0.08, 0.30), (x + L / 2 - 0.10, y - W / 2 + 0.10, H + 0.27),
                  color=(0.85, 0.40, 0.05))
    for n in scene.obstacle_names:
        if n.startswith(f"{name}/") and "/stack/" not in n:
            scene.set_obstacle_material(n, metalness=0.1, roughness=0.55)
    # the stack in the magazine
    stack = []
    for k in range(pallets):
        pn = f"{name}/stack/p{k}"
        bt.parts.pallet(scene, pn, (x, y, top + 0.002 + k * (pallet[2] + 0.002)), size=pallet, model="EPAL 2")
        scene.remove_part(pn)                 # stock in the magazine, not equipment
        stack.append(pn)
    zone_lo = top + pallet[2] - 0.005
    scene.add_lift(f"{name}/lift", car=car, zone_position=(x, y, zone_lo + 1.2),
                   zone_size=(1.3, 1.1, 2.4), stops={"hold": 0.0, "raised": 0.06}, speed=0.1, start="hold")
    scene.set_part(name, kind="group", category="machine.pallet_dispenser", manufacturer="PALOMAT",
                   model="Inline 130201", description="pallet dispenser, 1200 x 1000 long side, 15 pallets")
    scene.set_part(f"{name}/lift", manufacturer="PALOMAT", model="Inline 130201 lift and grippers (included)",
                   category="machine.pallet_dispenser")
    return Dispenser(name, f"{name}/lift", stack)


# ================================================================ labeller
def labeller(scene, name: str, position, *, facing=(0.0, 1.0), stroke: float = 0.30) -> str:
    """Logopak Series 700 on its two-column floor frame (701 x 771, 2035
    tall): the print module up top, the black applicator arm with its tamp
    pad 850 above the floor, reaching toward `facing`; the control box on
    its own pedestal beside it. The arm is a linear axis (`<name>/apply`,
    stops `home` / `out`, `stroke` long)."""
    x, y = position
    yaw = _yaw(facing)
    q = _quat(yaw)
    fx, fy = math.cos(yaw), math.sin(yaw)
    sx, sy = -fy, fx

    def at(u, v, z):   # u toward the pallet, v across
        return (x + fx * u + sx * v, y + fy * u + sy * v, z)

    for k, v in enumerate((-0.30, 0.30)):
        scene.add_box(f"{name}/column{k}", (0.08, 0.08, 2.035), at(-0.25, v, 1.0175), quaternion=q, color=ALUMINIUM)
        scene.add_box(f"{name}/foot{k}", (0.771, 0.10, 0.03), at(-0.25 + 0.25, v, 0.015), quaternion=q,
                      color=ALUMINIUM)
    scene.add_box(f"{name}/crossbar", (0.08, 0.68, 0.08), at(-0.25, 0.0, 2.0), quaternion=q, color=ALUMINIUM)
    scene.add_box(f"{name}/printer", (0.42, 0.50, 0.55), at(-0.12, 0.0, 1.45), quaternion=q, color=(0.86, 0.84, 0.76))
    scene.add_box(f"{name}/printer_door", (0.02, 0.42, 0.40), at(0.10, 0.0, 1.45), quaternion=q, color=DARK)
    scene.add_box(f"{name}/beacon", (0.06, 0.06, 0.29), at(-0.25, 0.30, 2.18), quaternion=q, color=(0.9, 0.35, 0.05))
    scene.add_box(f"{name}/control", (0.60, 0.30, 0.80), at(-0.30, 0.75, 0.82), quaternion=q, color=(0.80, 0.81, 0.82))
    scene.add_box(f"{name}/control_post", (0.08, 0.08, 0.42), at(-0.30, 0.75, 0.21), quaternion=q, color=ALUMINIUM)
    arm = [scene.add_box(f"{name}/arm", (0.55, 0.10, 0.10), at(0.05, 0.0, 0.85), quaternion=q, color=DARK),
           scene.add_box(f"{name}/pad", (0.04, 0.22, 0.16), at(0.34, 0.0, 0.85), quaternion=q,
                         color=(0.12, 0.12, 0.12))]
    scene.add_linear_axis(f"{name}/apply", objects=arm, axis=(fx, fy, 0.0), speed=0.5, range=(0.0, stroke),
                          stops={"home": 0.0, "out": stroke})
    scene.set_part(name, kind="group", category="machine.labeller", manufacturer="Logopak", model="Series 700",
                   description="print-and-apply pallet labeller, 2-column floor frame")
    scene.set_part(f"{name}/apply", manufacturer="Logopak", model="Series 700 applicator (included)",
                   category="machine.labeller")
    return f"{name}/apply"


# ================================================================ sheet magazine
def sheet_magazine(scene, name: str, position, *, sheets: float = 0.28, pallet=(1.2, 1.0, 0.162)) -> str:
    """A tier-sheet magazine beside the robot: guide posts round a stack of
    1200 x 1000 kraft sheets on a pallet — generic, no maker named. Returns
    the frame name of the stack top (where the gripper takes a sheet)."""
    x, y = position
    bt.parts.pallet(scene, f"{name}/pallet", (x, y, 0.0), size=pallet, model="EPAL 2")
    scene.remove_part(f"{name}/pallet")
    stack = scene.add_box(f"{name}/sheets", (1.18, 0.98, sheets), (x, y, pallet[2] + sheets / 2),
                          color=(0.66, 0.50, 0.31))
    scene.set_obstacle_material(stack, metalness=0.0, roughness=0.95)
    for k, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        scene.add_box(f"{name}/post{k}", (0.045, 0.045, 1.25), (x + sx * 0.66, y + sy * 0.56, 0.625), color=ALUMINIUM)
    for sy in (-1, 1):
        scene.add_box(f"{name}/rail_x{'lr'[sy > 0]}", (1.37, 0.045, 0.045), (x, y + sy * 0.56, 1.25), color=ALUMINIUM)
    for sx in (-1, 1):
        scene.add_box(f"{name}/rail_y{'lr'[sx > 0]}", (0.045, 1.17, 0.045), (x + sx * 0.66, y, 1.25), color=ALUMINIUM)
    scene.set_part(name, kind="group", category="structure.magazine", description="tier-sheet magazine")
    top = f"{name}/top"
    scene.add_frame(top, position=(x, y, pallet[2] + sheets))
    return top


# ================================================================ riser
def riser(scene, name: str, position, height: float, *, top=(1.10, 1.10), color=(0.70, 0.71, 0.72),
          plate_color=DARK, floor_plate: float | None = None) -> str:
    """A welded robot riser the integrator builds: a top plate on four
    square legs, braced, on a base plate. Returns the mount frame."""
    x, y = position
    tw, td = top
    scene.add_box(f"{name}/top", (tw, td, 0.04), (x, y, height - 0.02), color=color)
    leg = 0.14
    for k, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
        scene.add_box(f"{name}/leg{k}", (leg, leg, height - 0.08),
                      (x + sx * (tw / 2 - leg / 2), y + sy * (td / 2 - leg / 2), 0.04 + (height - 0.08) / 2),
                      color=color)
    for sy in (-1, 1):
        scene.add_box(f"{name}/brace_x{'lr'[sy > 0]}", (tw - 2 * leg, 0.08, 0.10),
                      (x, y + sy * (td / 2 - leg / 2), height * 0.35), color=color)
    for sx in (-1, 1):
        scene.add_box(f"{name}/brace_y{'lr'[sx > 0]}", (0.08, td - 2 * leg, 0.10),
                      (x + sx * (tw / 2 - leg / 2), y, height * 0.35), color=color)
    scene.add_box(f"{name}/base", (tw + 0.16, td + 0.16, 0.04), (x, y, 0.02), color=plate_color)
    if floor_plate:
        made = scene.add_box(f"{name}/floor_plate", (floor_plate, floor_plate, 0.006), (x, y, 0.003),
                             color=(0.72, 0.73, 0.74))
        scene.set_obstacle_enabled(made, False)
    for n in scene.obstacle_names:
        if n.startswith(f"{name}/"):
            scene.set_obstacle_material(n, metalness=0.2, roughness=0.6)
    scene.set_part(name, kind="group", category="structure.pedestal", description="welded robot riser")
    mount = f"{name}/mount"
    scene.add_frame(mount, position=(x, y, height))
    return mount


# ================================================================ EOAT
EOAT_LENGTH = 0.228   # flange face to foam face


def eoat_urdf(name: str = "eoat") -> str:
    """The case gripper an integrator builds for the M-410: an adapter plate
    on the flange, a column, an aluminium frame, and under it two Schmalz
    FA-Xc SVK 442 3R18 O20 area-gripper bars (442 x 130 x 70, 20 mm foam),
    130 mm apart — a 442 x 300 pad, one 400 x 300 case. Root link at the
    flange face, +Z into the tool; `<name>_tcp` at the foam face."""
    steel, alu, bar, foam = (0.55, 0.57, 0.60), ALUMINIUM, (0.32, 0.34, 0.37), (0.80, 0.81, 0.80)
    body = _box_xml((0.20, 0.20, 0.02), (0, 0, 0.01), color=steel, collide=False)
    body += _cyl_xml(0.125, 0.02, (0, 0, 0.01), (0, 0, 0), steel)
    body += _box_xml((0.12, 0.12, 0.10), (0, 0, 0.07), color=alu, collide=False)
    body += _box_xml((0.50, 0.34, 0.04), (0, 0, 0.138), color=alu, collide=False)
    for s in (-1, 1):
        body += _box_xml((0.442, 0.130, 0.050), (0, s * 0.085, 0.183), color=bar, collide=False)
        body += _box_xml((0.442, 0.130, 0.020), (0, s * 0.085, 0.218), color=foam, collide=False)
        body += _box_xml((0.06, 0.13, 0.03), (0.25, s * 0.085, 0.165), color=(0.10, 0.10, 0.11), collide=False)
    body += '<collision><origin xyz="0 0 0.116"/><geometry><box size="0.52 0.34 0.228"/></geometry></collision>'
    return (f'<?xml version="1.0"?><robot name="{name}">'
            f'<link name="{name}_mount">{body}</link><link name="{name}_tcp"/>'
            f'<joint name="{name}_tcp_joint" type="fixed"><parent link="{name}_mount"/><child link="{name}_tcp"/>'
            f'<origin xyz="0 0 {EOAT_LENGTH:.4f}"/></joint></robot>')
