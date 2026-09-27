"""The equipment of the shuttle-line demo, built from primitives in the
published sizes of the products it stands for.

Commercial equipment follows the published dimensions of the named products
(sources in `.internal/design-shuttle-line.md`). Frames, brackets, covers and
fixtures are independently authored layout geometry; their detailed shape,
colour and fasteners are illustrative.

    ATS SuperTrak GEN3      linear-motor conveyance in the over-under
                            arrangement: 1000 mm straight sections, 180° 800 mm
                            sections (837.9 across, 356.4 deep), 152 mm shuttles
                            (98.3 high, 4 m/s, repeatability 0.01 mm)
    SI-built                the red carrier plate and nest on each shuttle, the
                            portal frame (JIS G 3466 square tube), the station
                            deck, the robot pedestals, the floor frame

A shuttle is a two-joint robot built from URDF text: `s` slides its carriage
along the straight and `turn` swings it round a 180° section, so a shuttle
runs the whole loop — along the top, round the left end, back underneath
upside down and up round the right end — on ramps of its two joints, and
what it carries rides it by `attach`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import botrail as bt

# ---- finishes (linear RGB) ------------------------------------------------
ANODISED = (0.93, 0.94, 0.95)         # SuperTrak sections, natural anodised aluminium
COVER = (0.90, 0.91, 0.92)            # the over-under covers between the sections
SHADOW = (0.08, 0.085, 0.09)          # the joint between a section and its cover
SHUTTLE_BLACK = (0.025, 0.026, 0.028)
CARRIER_RED = (0.62, 0.018, 0.010)    # the carrier plates, RAL 3000 flame red
NEST_GREY = (0.10, 0.105, 0.11)
RAL_7035 = (0.78, 0.80, 0.80)         # the portal frame, light grey (RAL 7035)
RAL_9003 = (0.83, 0.84, 0.84)         # signal white: pedestals, floor frame
RAL_7016 = (0.045, 0.052, 0.060)      # anthracite: station deck
DARK = (0.03, 0.032, 0.035)

# ---- ATS SuperTrak GEN3 --------------------------------------------------
ST_SECTION = 1.0                       # straight section pitch
ST_DEPTH = 0.3564                      # section height 356.4 -> depth front to back, over-under
ST_R = 0.419                           # 180° 800 mm section: 837.9 across the outside
ST_BAND = 0.10                         # the sections' radial depth (authored: the drawing's outer ring)
SHUTTLE = (0.152, 0.190, 0.0983)       # length along the track, width across it, height off the face
ST_SPEED = 4.0                         # m/s, straight and curved
CARRIER = 0.25                         # the carrier plate's length along the track (SI tooling)
NEST_TOP = SHUTTLE[2] + 0.012 + 0.040  # a part stands on this, off the sections' outer face
KEY_BAR, KEY_STOP = 0.035, 0.075      # the sensor keys, this far inside the sections' outer face


def _quat_x(angle: float):
    return (math.sin(angle / 2), 0.0, 0.0, math.cos(angle / 2))


def _quat_z(angle: float):
    return (0.0, 0.0, math.sin(angle / 2), math.cos(angle / 2))


def _material(color) -> str:
    return "c_" + "_".join(f"{round(v * 1000):04d}" for v in color)


def _box_xml(size, xyz=(0, 0, 0), color=DARK, collide=False, rpy=(0, 0, 0)) -> str:
    s = " ".join(f"{v:.5f}" for v in size)
    o = f'<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>'
    c = " ".join(f"{v:.4f}" for v in color)
    out = (f'<visual>{o}<geometry><box size="{s}"/></geometry>'
           f'<material name="{_material(color)}"><color rgba="{c} 1"/></material></visual>')
    if collide:
        out += f'<collision>{o}<geometry><box size="{s}"/></geometry></collision>'
    return out


def _cyl_xml(radius, length, xyz=(0, 0, 0), color=DARK, rpy=(0, 0, 0)) -> str:
    o = f'<origin xyz="{xyz[0]:.5f} {xyz[1]:.5f} {xyz[2]:.5f}" rpy="{rpy[0]:.5f} {rpy[1]:.5f} {rpy[2]:.5f}"/>'
    c = " ".join(f"{v:.4f}" for v in color)
    return (f'<visual>{o}<geometry><cylinder radius="{radius:.5f}" length="{length:.5f}"/></geometry>'
            f'<material name="{_material(color)}"><color rgba="{c} 1"/></material></visual>')


def _plain(scene, name: str, size, position, color, quaternion=None, collide: bool = False) -> str:
    made = scene.add_box(name, size, position, quaternion=quaternion, color=color)
    if not collide:
        scene.set_obstacle_enabled(made, False)
    metal = color == ANODISED
    scene.set_obstacle_material(made, metalness=0.55 if metal else 0.0,
                                roughness=0.32 if metal else 0.52)
    return made


# ================================================================ the loop
@dataclass
class Loop:
    """A SuperTrak GEN3 loop set up over-under: the straight runs from the
    right 180° section's centre (x0, y0) to the left one's, `length` along
    -X; `top` is the upper sections' outer face. A shuttle's path is the
    sections' outer face: the joint `s` is the distance along -X from the
    right centre, `turn` the angle swept round the ends (0 on top, pi
    underneath)."""

    name: str
    x0: float
    y0: float
    top: float
    length: float
    shuttles: list = field(default_factory=list)
    obstacles: list = field(default_factory=list)

    @property
    def centre_z(self) -> float:
        return self.top - ST_R

    @property
    def lap(self) -> float:
        """One lap at the sections' face."""
        return 2 * self.length + 2 * math.pi * ST_R

    def x(self, s: float) -> float:
        return self.x0 - s


def supertrak(scene, name: str, sections: int, position, *, top: float, stand_pitch: float = 1.5,
              floor: float = 0.0) -> Loop:
    """ATS SuperTrak GEN3 in the over-under arrangement: `sections` straight
    sections of 1000 mm between two 180° 800 mm sections, the right one's
    centre at `position` (x, y), the upper sections' outer face at `top`.
    The shuttles ride the outer face — along the top, round the ends,
    upside down along the bottom. Front and back of the loop carry the
    over-under covers; the loop stands on brackets off its back face,
    clear of the shuttles' carrier plates."""
    x0, y0 = position
    length = sections * ST_SECTION
    loop = Loop(name, x0, y0, top, length)
    zc = top - ST_R
    xm = x0 - length / 2
    half = ST_DEPTH / 2
    obs = loop.obstacles
    # The loop body: the straight between the two ends — what a shuttle's
    # carrier plate clears — and the two 180° sections as cylinders.
    obs.append(scene.add_box(f"{name}/body", (length, ST_DEPTH, 2 * ST_R), (xm, y0, zc), color=COVER))
    scene.set_obstacle_material(obs[-1], metalness=0.25, roughness=0.4)
    for tag, x in (("right", x0), ("left", x0 - length)):
        obs.append(scene.add_cylinder(f"{name}/end_{tag}", ST_R, ST_DEPTH, (x, y0, zc), quaternion=_quat_x(math.pi / 2),
                                      color=ANODISED))
        # the section's cover disc, proud front and back: the outer ring is the section
        cover = scene.add_cylinder(f"{name}/end_{tag}/cover", ST_R - ST_BAND, ST_DEPTH + 0.004, (x, y0, zc),
                                   quaternion=_quat_x(math.pi / 2), color=COVER)
        scene.set_obstacle_enabled(cover, False)
        scene.set_obstacle_material(cover, metalness=0.25, roughness=0.4)
        obs.append(cover)
        # Stepped front rim; the smaller disc leaves the surrounding metal visible.
        rim = scene.add_cylinder(f"{name}/end_{tag}/rim", ST_R - ST_BAND - 0.014, 0.006,
                                  (x, y0 - half - 0.006, zc), quaternion=_quat_x(math.pi / 2), color=ANODISED)
        scene.set_obstacle_enabled(rim, False)
        scene.set_obstacle_material(rim, metalness=0.55, roughness=0.32)
        obs.append(rim)
    for z, tag in ((top - ST_BAND / 2, "upper"), (zc - ST_R + ST_BAND / 2, "lower")):
        # the straight sections' outer band, proud of the covers front and back
        obs.append(_plain(scene, f"{name}/{tag}", (length, ST_DEPTH + 0.004, ST_BAND), (xm, y0, z), ANODISED))
        # the joint between the band and the cover: a dark line on both faces
        zj = z - ST_BAND / 2 - 0.012 if tag == "upper" else z + ST_BAND / 2 + 0.012
        for side in (-1, 1):
            obs.append(_plain(scene, f"{name}/{tag}/joint_{'f' if side < 0 else 'b'}", (length, 0.004, 0.024),
                              (xm, y0 + side * (half + 0.002), zj), SHADOW))
        # the V-rail along the band's front edge
        edge = ST_BAND / 2 - 0.012 if tag == "upper" else -ST_BAND / 2 + 0.012
        obs.append(_plain(scene, f"{name}/{tag}/rail", (length, 0.012, 0.010), (xm, y0 - half - 0.006, z + edge), DARK))
    # section joints on the front face, every metre
    for i in range(1, sections):
        x = x0 - i * ST_SECTION
        obs.append(_plain(scene, f"{name}/seam{i}", (0.002, 0.004, 2 * ST_R - 0.04), (x, y0 - half - 0.003, zc),
                          SHADOW))
    # Folded cover edges, service-panel fasteners and shallow joints reveal the construction.
    for i in range(sections):
        x = x0 - (i + 0.5) * ST_SECTION
        for sign in (-1, 1):
            zz = zc + sign * (ST_R - ST_BAND - 0.030)
            obs.append(_plain(scene, f"{name}/cover{i}/fold{sign}", (ST_SECTION - 0.008, 0.008, 0.010),
                              (x, y0 - half - 0.007, zz), ANODISED))
            for dx in (-0.43, 0.43):
                screw = scene.add_cylinder(f"{name}/cover{i}/screw{sign}_{dx}", 0.006, 0.003,
                                           (x + dx, y0 - half - 0.005, zz - sign * 0.030),
                                           quaternion=_quat_x(math.pi / 2), color=DARK)
                scene.set_obstacle_enabled(screw, False)
                obs.append(screw)
    # stands: brackets off the back face at mid-height, down to the floor
    n = max(2, math.ceil(length / stand_pitch) + 1)
    for i in range(n):
        x = x0 - length * i / (n - 1)
        obs.append(scene.add_box(f"{name}/stand{i}/bracket", (0.12, 0.16, 0.20), (x, y0 + half + 0.08, zc),
                                 color=DARK))
        obs.append(scene.add_box(f"{name}/stand{i}/leg", (0.10, 0.10, zc + 0.10 - floor),
                                 (x, y0 + half + 0.13, floor + (zc + 0.10 - floor) / 2), color=DARK))
    scene.set_part(name, kind="group", manufacturer="ATS Automation", model="SuperTrak GEN3 (over-under)",
                   category="conveyor.linear_motor",
                   description=f"{sections} x 1000 mm straight sections, 2 x 180 deg 800 mm sections")
    return loop


# ================================================================ shuttles
def shuttle_urdf(loop: Loop, laps: int) -> str:
    """One SuperTrak GEN3 shuttle with its carrier plate and nest, as a
    robot rooted at the right 180° section's centre: prismatic `s` along
    -X, revolute `turn` about -Y (so a positive turn carries it from the
    top over the left end and underneath). Everything is drawn off the
    sections' face (+Z of the link, `ST_R` out from the loop's centre)."""
    r = ST_R
    sx, sy, sh = SHUTTLE
    body = ""
    # the shuttle itself: its magnet plate over the face, v-wheel carriage down the front
    body += _box_xml((sx, sy, sh), (0, -0.04, r + sh / 2), SHUTTLE_BLACK)
    body += _box_xml((sx, 0.03, 0.09), (0, -ST_DEPTH / 2 - 0.018, r - 0.02), SHUTTLE_BLACK)
    # the carrier plate across the whole depth, and its skirts down the front and back faces:
    # wide under the plate, narrowing to a tab (the silhouette the picture shows)
    t = 0.012
    body += _box_xml((CARRIER, ST_DEPTH + 0.05, t), (0, 0, r + sh + t / 2), CARRIER_RED)
    y_skirt = ST_DEPTH / 2 + 0.036
    for side, depth in ((-1, 1.0), (1, 0.55)):
        y = side * y_skirt
        body += _box_xml((CARRIER, t, 0.07), (0, y, r + sh - 0.035), CARRIER_RED)
        body += _box_xml((CARRIER * 0.66, t, 0.07 * depth), (0, y, r + sh - 0.07 - 0.035 * depth), CARRIER_RED)
        body += _box_xml((CARRIER * 0.30, t, 0.06 * depth), (0, y, r + sh - 0.07 - 0.07 * depth - 0.03 * depth),
                         CARRIER_RED)
    # the nest: a block with a pocket rim and two locating pins
    nz = r + sh + t
    body += _box_xml((0.15, 0.13, 0.028), (0, 0, nz + 0.014), NEST_GREY)
    for dy in (-0.058, 0.058):
        body += _box_xml((0.13, 0.014, 0.012), (0, dy, nz + 0.034), NEST_GREY)
    for dx in (-0.066, 0.066):
        body += _box_xml((0.014, 0.10, 0.012), (dx, 0, nz + 0.034), NEST_GREY)
    for dx in (-0.055, 0.055):
        body += _cyl_xml(0.004, 0.018, (dx, 0.045, nz + 0.037), (0.7, 0.72, 0.74))
    # What collides: the carrier plate — what a robot at a station could
    # touch — and two keys inside the sections' band that only the loop's
    # sensors see (they sit in the band too, out of sight): a bar the
    # carrier's length that the block sensors read, and a 10 mm cube that
    # the stations' presence switches read.
    body += (f'<collision><origin xyz="0 0 {r + sh + t / 2:.5f}"/>'
             f'<geometry><box size="{CARRIER - 0.01:.5f} {ST_DEPTH:.5f} {t:.5f}"/></geometry></collision>')
    body += (f'<collision><origin xyz="0 0 {r - KEY_BAR:.5f}"/>'
             f'<geometry><box size="{CARRIER - 0.01:.5f} 0.010 0.010"/></geometry></collision>')
    body += (f'<collision><origin xyz="0 0 {r - KEY_STOP:.5f}"/>'
             f'<geometry><box size="0.010 0.010 0.010"/></geometry></collision>')
    return (f'<?xml version="1.0"?><robot name="supertrak_shuttle"><link name="rail"/><link name="carriage"/>'
            f'<link name="body">{body}</link>'
            f'<joint name="s" type="prismatic"><parent link="rail"/><child link="carriage"/><axis xyz="-1 0 0"/>'
            f'<limit lower="-0.001" upper="{loop.length + 0.001:.5f}" velocity="{ST_SPEED}" effort="1"/></joint>'
            f'<joint name="turn" type="revolute"><parent link="carriage"/><child link="body"/><axis xyz="0 -1 0"/>'
            f'<limit lower="-0.001" upper="{2 * math.pi * laps + 0.001:.5f}" velocity="{ST_SPEED / ST_R:.4f}"'
            f' effort="1"/></joint></robot>')


def add_shuttles(scene, loop: Loop, count: int, *, laps: int = 4, panel=(0.0, 1.0, 0.0)) -> list:
    """`count` shuttles on the loop, all parked at the right end until the
    cell sets them where they start, and the one SuperTrak control panel
    that drives them all (910 x 540 x 384, RAL 7024) standing at `panel`
    (x, y, floor z)."""
    urdf = shuttle_urdf(loop, laps)
    names = []
    for k in range(count):
        name = f"{loop.name}/shuttle{k:02d}"
        scene.add_robot(bt.Robot.from_urdf_string(urdf), name=name, base_position=(loop.x0, loop.y0, loop.centre_z))
        scene.set_part(name, kind="robot", manufacturer="ATS Automation", model="SuperTrak GEN3 shuttle, 3-magnet",
                       category="conveyor.shuttle", description="25193343, with an SI-built carrier plate and nest")
        for o in loop.obstacles:
            scene.allow_link_obstacle_contact("body", o, robot=name)
        names.append(name)
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            scene.allow_inter_robot_collision(a, "body", b, "body")
    loop.shuttles = names
    bt.parts.controller(scene, f"{loop.name}/panel", names, panel, size=(0.91, 0.54, 0.384),
                        manufacturer="ATS Automation", model="SuperTrak GEN3 control panel",
                        color=(0.070, 0.074, 0.078), yaw=math.pi)
    return names


# ---- where a shuttle is: the loop as one path --------------------------
def path_q(loop: Loop, p: float) -> tuple[float, float]:
    """(s, turn) of a shuttle `p` metres along the loop from the right end's
    top, running left along the top: 0 .. L top, L .. L + pi R round the left
    end, then back along the bottom and up round the right end."""
    L, arc = loop.length, math.pi * ST_R
    lap, p = divmod(p, loop.lap)
    base = 2 * math.pi * lap
    if p <= L:
        return p, base
    if p <= L + arc:
        return L, base + (p - L) / ST_R
    if p <= 2 * L + arc:
        return L - (p - L - arc), base + math.pi
    return 0.0, base + math.pi + (p - 2 * L - arc) / ST_R


def path_point(loop: Loop, p: float, radial: float) -> tuple[float, float, float]:
    """The world point `radial` off the sections' face at path position `p`
    (in the loop's centre plane)."""
    s, turn = path_q(loop, p)
    r = ST_R + radial
    return loop.x(s) - r * math.sin(turn), loop.y0, loop.centre_z + r * math.cos(turn)


def path_moves(loop: Loop, name: str, p_from: float, p_to: float, speed: float) -> list:
    """The ramps that carry shuttle `name` from `p_from` to `p_to` along
    the loop, one per straight or curve it crosses — each a rest-to-rest
    move over its length at `speed` on average."""
    L, arc = loop.length, math.pi * ST_R
    cuts = []
    k = math.floor(p_from / loop.lap) * loop.lap
    while k <= p_to:
        cuts += [k + L, k + L + arc, k + 2 * L + arc, k + loop.lap]
        k += loop.lap
    points = [p_from] + [c for c in cuts if p_from + 1e-9 < c < p_to - 1e-9] + [p_to]
    moves = []
    for a, b in zip(points, points[1:]):
        sa, ta = path_q(loop, a)
        sb, tb = path_q(loop, b)
        targets = {}
        if abs(sb - sa) > 1e-9:
            targets["s"] = sb
        if abs(tb - ta) > 1e-9:
            targets["turn"] = tb
        if targets:
            moves.append(bt.seq.ramp(targets, max(round((b - a) / speed * 1.5, 3), 0.12), robot=name))
    return moves


def _in_band(scene, loop: Loop, name: str, p: float, depth: float, size) -> None:
    """A zone sensor inside the sections' band, `depth` under the outer face
    at path position `p`, turned with the path: out of sight, and seeing
    nothing but the shuttles' keys. It stands for what the SuperTrak
    controller knows of every shuttle's position — no sensor of its own
    to buy."""
    x, y, z = path_point(loop, p, -depth)
    _, turn = path_q(loop, p)
    q = (0.0, -math.sin(turn / 2), 0.0, math.cos(turn / 2))
    scene.add_zone_sensor(name, position=(x, y, z), size=size, quaternion=q, watch_robots=list(loop.shuttles))
    scene.set_part(name, manufacturer="ATS Automation", model="SuperTrak GEN3 shuttle position (included)",
                   category="sensor.position")


def block_sensor(scene, loop: Loop, name: str, p0: float, p1: float) -> None:
    """A block along a straight between path positions `p0` and `p1`: true
    while any shuttle's key bar (its carrier's extent) is in it."""
    _, t0 = path_q(loop, p0)
    _, t1 = path_q(loop, p1)
    assert abs(t0 - t1) < 1e-9, "a block sensor spans one straight"
    _in_band(scene, loop, name, (p0 + p1) / 2, KEY_BAR, (abs(p1 - p0), 0.02, 0.012))


def end_sensors(scene, loop: Loop, name: str, side: str, reach: float, pitch: float = 0.12) -> list:
    """The blocks round one 180° section and `reach` of straight into each
    run, as short zones along the path (a straight zone cannot follow the
    curve inside the section): the end is clear while all of them are."""
    L, arc = loop.length, math.pi * ST_R
    if side == "left":
        p0, p1 = L - reach, L + arc + reach
    else:
        p0, p1 = 2 * L + arc - reach, 2 * L + 2 * arc + reach
    n = max(2, math.ceil((p1 - p0) / pitch))
    names = []
    for i in range(n):
        a, b = p0 + (p1 - p0) * i / n, p0 + (p1 - p0) * (i + 1) / n
        # on the curve a zone is a chord: long enough to overlap its neighbours
        _in_band(scene, loop, f"{name}{i}", (a + b) / 2, KEY_BAR, ((b - a) * 1.25, 0.02, 0.012))
        names.append(f"{name}{i}")
    return names


def stop_sensor(scene, loop: Loop, name: str, s: float) -> None:
    """A 12 mm presence switch at a top stop, on the shuttles' stop key: it
    reads true only while a shuttle stands within a few millimetres."""
    _in_band(scene, loop, name, s, KEY_STOP, (0.012, 0.02, 0.012))


# ================================================================ the frame
def portal_frame(scene, name: str, *, x_right: float, x_left: float, bays: list, y_front: float, y_back: float,
                 height: float, trunking: float = 0.40, column: float = 0.15, color=RAL_7035) -> dict:
    """The SI-built portal over the line: corner columns front and back, a
    row of back columns at `bays` (x), a front beam and a back beam the
    line's length, a cross beam from every back column to the front beam,
    knee braces, and on top along the front the cable trunking — a box
    with lightening windows. Square steel tube □150 × 6 (JIS G 3466
    STKR400). `height` is the beams' top; their underside is where a
    robot's base plate bolts on."""
    beam = column
    zt = height
    zb = zt - beam
    xs = [x_right] + list(bays) + [x_left]
    made = []
    for i, x in enumerate(xs):
        for tag, y in (("b", y_back),) + ((("f", y_front),) if i in (0, len(xs) - 1) else ()):
            made.append(scene.add_box(f"{name}/col_{tag}{i}", (column, column, zt), (x, y, zt / 2), color=color))
            _plain(scene, f"{name}/col_{tag}{i}/foot", (column + 0.10, column + 0.10, 0.02), (x, y, 0.01), color)
            for dx in (-1, 1):
                for dy in (-1, 1):
                    anchor = scene.add_cylinder(f"{name}/col_{tag}{i}/anchor{dx}_{dy}", 0.012, 0.022,
                                                (x + dx * (column / 2 + 0.025),
                                                 y + dy * (column / 2 + 0.025), 0.03), color=ANODISED)
                    scene.set_obstacle_enabled(anchor, False)
    span = x_right - x_left
    xm = (x_right + x_left) / 2
    for tag, y in (("front_beam", y_front), ("back_beam", y_back)):
        made.append(scene.add_box(f"{name}/{tag}", (span + column, beam, beam), (xm, y, zt - beam / 2), color=color))
    depth = y_back - y_front
    for i, x in enumerate(xs):
        made.append(scene.add_box(f"{name}/cross{i}", (beam, depth, beam), (x, (y_back + y_front) / 2, zt - beam / 2),
                                  color=color))
    # knee braces, 45 degrees: column to the cross beam, and along the back beam either side
    k = 0.50
    for i, x in enumerate(xs):
        for tag, (dx, dy) in (("y", (0.0, -1.0)), ("xl", (-1.0, 0.0)), ("xr", (1.0, 0.0))):
            if tag == "xr" and i == 0 or tag == "xl" and i == len(xs) - 1:
                continue
            q = _mul(_quat_z(math.atan2(dy, dx)), (0.0, math.sin(-math.pi / 8), 0.0, math.cos(-math.pi / 8)))
            _plain(scene, f"{name}/brace{i}{tag}", (k * math.sqrt(2), 0.10, 0.10),
                   (x + dx * k / 2, y_back + dy * k / 2, zb - k / 2), color, quaternion=q)
    # Open cable bridge: two edge rails and webs, with actual through-openings.
    yt = y_front - 0.05
    for tag, z in (("bottom", zt + 0.025), ("top", zt + trunking - 0.025)):
        made.append(_plain(scene, f"{name}/trunking/{tag}", (span + column, 0.30, 0.05),
                           (xm, yt, z), color, collide=True))
    n_win = max(1, round(span / 1.2))
    for w in range(n_win + 1):
        x = x_right - w * span / n_win
        made.append(_plain(scene, f"{name}/trunking/web{w}", (0.22, 0.30, trunking - 0.10),
                           (x, yt, zt + trunking / 2), color, collide=True))
    _plain(scene, f"{name}/trunking/cable", (span, 0.10, 0.030), (xm, yt, zt + 0.068), DARK)
    for i, x in enumerate(xs):
        _plain(scene, f"{name}/joint{i}/plate", (column + 0.09, 0.012, beam + 0.12),
               (x, y_back - column / 2 - 0.006, zt - beam / 2), color)
        for dx in (-0.08, 0.08):
            for dz in (-0.09, 0.09):
                bolt = scene.add_cylinder(f"{name}/joint{i}/bolt{dx}_{dz}", 0.012, 0.008,
                                          (x + dx, y_back - column / 2 - 0.017, zt - beam / 2 + dz),
                                          quaternion=_quat_x(math.pi / 2), color=ANODISED)
                scene.set_obstacle_enabled(bolt, False)
    scene.set_part(name, kind="group", category="structure.frame",
                   description="SI-built portal: JIS G 3466 STKR400 square tube 150 x 150 x 6, trunking on top")
    return {"underside": zb, "top": zt, "columns": xs, "obstacles": made}


def _mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz)


def mount_beam(scene, name: str, x: float, y: float, *, underside: float, length: float, color=RAL_7035) -> float:
    """A short beam between two cross beams, a robot's base bolted under it:
    returns the height of its underside."""
    scene.add_box(name, (length, 0.15, 0.15), (x, y, underside + 0.075), color=color)
    _plain(scene, f"{name}/plate", (0.24, 0.24, 0.02), (x, y, underside - 0.01), DARK)
    return underside - 0.02


# ================================================================ stations
def station_deck(scene, name: str, x0: float, x1: float, y0: float, y1: float, top: float, *, legs: float = 1.2) -> str:
    """The anthracite station deck behind the loop: a 25 mm steel plate on
    square legs, where the pedestals and the feeders stand."""
    xm, ym = (x0 + x1) / 2, (y0 + y1) / 2
    deck = scene.add_box(f"{name}", (abs(x1 - x0), abs(y1 - y0), 0.025), (xm, ym, top - 0.0125), color=RAL_7016)
    n = max(2, math.ceil(abs(x1 - x0) / legs) + 1)
    for i in range(n):
        x = x0 + (x1 - x0) * i / (n - 1)
        for tag, y in (("f", min(y0, y1) + 0.05), ("b", max(y0, y1) - 0.05)):
            _plain(scene, f"{name}/leg{i}{tag}", (0.08, 0.08, top - 0.025), (x, y, (top - 0.025) / 2),
                   (0.40, 0.42, 0.43))
    return deck


def scara_pedestal(scene, name: str, x: float, y: float, *, floor: float, top: float, yaw: float = 0.0) -> str:
    """The white pedestal an SR-3iA stands on: a welded box 260 × 200 with
    its top plate, from the deck to the robot's mounting face."""
    h = top - floor
    made = scene.add_box(name, (0.26, 0.20, h - 0.015), (x, y, floor + (h - 0.015) / 2), quaternion=_quat_z(yaw),
                         color=RAL_9003)
    _plain(scene, f"{name}/plate", (0.30, 0.24, 0.015), (x, y, top - 0.0075), RAL_9003, quaternion=_quat_z(yaw))
    return made


def floor_frame(scene, name: str, x0: float, x1: float, y_front: float, y_back: float, *, pitch: float = 0.85,
                tube: float = 0.10) -> None:
    """The white base frame on the floor in front of and under the loop: two
    rails the line's length and rungs between them."""
    xm, span = (x0 + x1) / 2, abs(x1 - x0)
    for tag, y in (("front", y_front), ("back", y_back)):
        _plain(scene, f"{name}/{tag}", (span, tube, tube), (xm, y, tube / 2), RAL_9003)
    n = max(2, round(span / pitch) + 1)
    for i in range(n):
        x = x0 + (x1 - x0) * i / (n - 1)
        _plain(scene, f"{name}/rung{i}", (tube, abs(y_back - y_front) - tube, tube * 0.8),
               (x, (y_front + y_back) / 2, tube * 0.4), RAL_9003)
    # the cable duct grating along the front
    _plain(scene, f"{name}/grating", (span, 0.16, 0.025), (xm, y_front - 0.14, 0.0125), (0.30, 0.31, 0.32))


# ================================================================ parts supply
BEIGE = (0.64, 0.52, 0.48)            # the jig pallets: resin, the picture's pinkish beige


def jig_pallet(scene, name: str, position, *, nests: tuple[int, int], pitch: tuple[float, float],
               nest_size=(0.10, 0.06), yaw: float = 0.0) -> list:
    """An SI-built jig pallet: a resin base plate on four risers with a row
    of nests on it, riding a belt. `position` is the underside's centre
    (x, y, z on the belt); returns the nests' seat points (x, y, z), row by
    row, in the pallet's +X then +Y order."""
    x, y, z = position
    nx, ny = nests
    px, py = pitch
    lx, ly = nx * px + 0.04, ny * py + 0.04
    q = _quat_z(yaw)
    c, s = math.cos(yaw), math.sin(yaw)

    def at(u, v):
        return x + c * u - s * v, y + s * u + c * v

    riser = 0.035
    for i, (u, v) in enumerate(((-1, -1), (1, -1), (1, 1), (-1, 1))):
        _plain(scene, f"{name}/riser{i}", (0.03, 0.03, riser), (*at(u * (lx / 2 - 0.03), v * (ly / 2 - 0.03)),
               z + riser / 2), BEIGE, quaternion=q)
    plate = scene.add_box(f"{name}/plate", (lx, ly, 0.012), (x, y, z + riser + 0.006), quaternion=q, color=BEIGE)
    top = z + riser + 0.012
    seats = []
    for j in range(ny):
        for i in range(nx):
            u, v = (i - (nx - 1) / 2) * px, (j - (ny - 1) / 2) * py
            cx_, cy_ = at(u, v)
            for side in (-1, 1):   # two rails either side of the part, the nest
                _plain(scene, f"{name}/nest{j}_{i}_{'a' if side < 0 else 'b'}", (nest_size[0], 0.012, 0.016),
                       (*at(u, v + side * (nest_size[1] / 2 + 0.006)), top + 0.008), BEIGE, quaternion=q)
            seats.append((cx_, cy_, top))
    scene.set_part(name, kind="group", category="fixture.pallet", description="SI-built jig pallet, POM, "
                   f"{nx} x {ny} nests")
    return seats, plate


def presenter(scene, name: str, position, *, size=(0.14, 0.26), top: float, color=(0.36, 0.37, 0.38),
              yaw: float = 0.0) -> tuple[float, float, float]:
    """A parts presenter at the end of a linear feeder: a block standing at
    `position` (x, y, z of its foot) with its top at `top`, the feeder's
    track along it and the pick nest at its local -Y end (turned by `yaw`).
    Returns the pick point: the nest floor."""
    x, y, z = position
    lx, ly = size
    q = _quat_z(yaw)
    c, s = math.cos(yaw), math.sin(yaw)

    def at(v):
        return x - s * v, y + c * v

    scene.add_box(f"{name}/body", (lx, ly, top - z), (x, y, (top + z) / 2), quaternion=q, color=color)
    v_track = (0.09 + 0.02) / 2 - 0.0            # from the nest back to the far end
    _plain(scene, f"{name}/track", (0.05, ly - 0.11, 0.02), (*at(v_track), top + 0.01), (0.55, 0.57, 0.59),
           quaternion=q)
    fx, fy = at(-ly / 2 + 0.05)
    _plain(scene, f"{name}/nest", (0.09, 0.07, 0.006), (fx, fy, top + 0.003), NEST_GREY, quaternion=q)
    for sign in (-1, 1):
        dx = sign * 0.045
        _plain(scene, f"{name}/guide{sign}", (0.008, ly - 0.10, 0.022),
               (x + c * dx - s * 0.055, y + s * dx + c * 0.055, top + 0.021), ANODISED, quaternion=q)
    _plain(scene, f"{name}/front_service_panel", (lx - 0.018, 0.004, (top - z) * 0.55),
           (*at(-ly / 2 - 0.002), z + (top - z) * 0.46), RAL_7016, quaternion=q)
    scene.set_part(name, kind="group", category="feeder.linear",
                   description="SI-built presenter at the end of a linear feeder")
    return fx, fy, top + 0.006


def signal_tower(scene, name: str, position, *, tiers=((0.9, 0.05, 0.03), (0.95, 0.65, 0.0), (0.05, 0.6, 0.1))) -> None:
    """A three-tier signal tower on a pole: red, amber, green."""
    x, y, z = position
    scene.add_cylinder(f"{name}/pole", 0.012, 0.25, (x, y, z + 0.125), color=(0.6, 0.62, 0.64))
    for i, c in enumerate(tiers):
        made = scene.add_cylinder(f"{name}/tier{i}", 0.03, 0.05, (x, y, z + 0.275 + 0.05 * (len(tiers) - 1 - i)),
                                  color=c)
        scene.set_obstacle_enabled(made, False)
    scene.set_part(f"{name}", kind="group", category="indicator.signal_tower")


# ================================================================ the product
HOUSING = (0.090, 0.040, 0.030)       # die-cast housing, gripped across its 40 mm
PCB = (0.075, 0.032, 0.004)
COVER_PART = (0.090, 0.040, 0.010)
CONNECTOR = (0.026, 0.018, 0.016)
PART_COLORS = {"housing": (0.30, 0.31, 0.33), "pcb": (0.02, 0.22, 0.06), "cover": (0.42, 0.36, 0.62),
               "connector": (0.015, 0.015, 0.017)}
GAP = 0.003                           # what is set down stands this proud of what it stands on


def part(scene, name: str, kind: str, bottom) -> str:
    """One part of the module, standing with its underside centre at `bottom`."""
    size = {"housing": HOUSING, "pcb": PCB, "cover": COVER_PART, "connector": CONNECTOR}[kind]
    x, y, z = bottom
    made = scene.add_box(name, size, (x, y, z + size[2] / 2), color=PART_COLORS[kind])
    return made


# ================================================================ zone control
@dataclass
class Stop:
    """A place on the loop a shuttle stops: `p` metres along the path from
    the right end's top (one lap), `top` whether it is on the upper run."""

    tag: str
    p: float
    top: bool
    s: float


def loop_stops(loop: Loop, top: dict, bottom: list) -> list:
    """The stops in path order: the upper run's (`top`, tag -> s) and the
    lower run's (`bottom`, s values), each lap."""
    arc = math.pi * ST_R
    stops = [Stop(tag, s, True, s) for tag, s in sorted(top.items(), key=lambda kv: kv[1])]
    stops += [Stop(f"r{i}", loop.length + arc + (loop.length - s), False, s)
              for i, s in enumerate(sorted(bottom, reverse=True))]
    return stops


def loop_sensors(scene, loop: Loop, stops: list, *, margin: float = 0.02) -> dict:
    """The block sensors of the loop: for every stop, the sensors that must
    all read clear before a shuttle may move into it from the stop behind.
    A block runs from just past the stop behind's carrier to the far edge
    of this stop's carrier, so a shuttle on its way in, or standing there,
    holds it. The two ends are one zone each (a shuttle going round holds
    the whole end); the right end's zone also holds the entry stop."""
    h = (CARRIER - 0.01) / 2
    reach_l = loop.length - max(st.s for st in stops if st.top) - h - margin
    reach_r = min(st.s for st in stops if st.top and st.s > 0.05) - h - margin
    end_l = end_sensors(scene, loop, f"{loop.name}/end_l", "left", reach_l)
    end_r = end_sensors(scene, loop, f"{loop.name}/end_r", "right", reach_r)
    blocks = {}
    n = len(stops)
    for i, st in enumerate(stops):
        prev = stops[i - 1]
        name = f"{loop.name}/block_{st.tag}"
        if prev.top and st.top or not prev.top and not st.top:                      # along the upper run
            block_sensor(scene, loop, name, prev.p + h + margin, st.p + h)
            blocks[i] = [name]
        elif prev.top and not st.top:                # round the left end
            p_exit = loop.length + math.pi * ST_R + reach_l
            block_sensor(scene, loop, name, p_exit, st.p + h)
            blocks[i] = end_l + [name]
        else:                                        # round the right end, into the entry
            assert st.s < 0.05, "the stop after the lower run must be the entry over the right end"
            blocks[i] = end_r
    assert len(blocks) == n
    return blocks


# ================================================================ the hall around
def neighbour_line(scene, name: str, sections: int, position, *, top: float, frame: dict, carriers: list) -> None:
    """The next line over, the way the picture's background shows it: the
    same portal and loop, standing still (no shuttles of ours, nothing of
    it on our bill) — the loop's body, its end sections, carrier plates at
    `carriers` (s along the top), the frame."""
    x0, y0 = position
    length = sections * ST_SECTION
    zc = top - ST_R
    _plain(scene, f"{name}/body", (length, ST_DEPTH, 2 * ST_R), (x0 - length / 2, y0, zc), COVER, collide=True)
    for tag, x in (("right", x0), ("left", x0 - length)):
        made = scene.add_cylinder(f"{name}/end_{tag}", ST_R, ST_DEPTH, (x, y0, zc), quaternion=_quat_x(math.pi / 2),
                                  color=ANODISED)
        scene.set_obstacle_enabled(made, False)
    _plain(scene, f"{name}/upper", (length, ST_DEPTH + 0.004, ST_BAND), (x0 - length / 2, y0, top - ST_BAND / 2),
           ANODISED)
    for i, s in enumerate(carriers):
        _plain(scene, f"{name}/carrier{i}", (CARRIER, ST_DEPTH + 0.05, 0.10), (x0 - s, y0, top + 0.05), CARRIER_RED)
        _plain(scene, f"{name}/nest{i}", (0.15, 0.13, 0.04), (x0 - s, y0, top + 0.12), NEST_GREY)
    for i in range(math.ceil(length / 1.5) + 1):
        x = x0 - length * i / math.ceil(length / 1.5)
        _plain(scene, f"{name}/leg{i}", (0.10, 0.10, zc), (x, y0 + ST_DEPTH / 2 + 0.13, zc / 2), DARK)
    portal_frame(scene, f"{name}/frame", **frame)
    scene.remove_part(f"{name}/frame")
    # Static background equipment: the second line has work surfaces and controller cabinets.
    for i, s in enumerate((1.30, 2.5, 4.2)):
        x = x0 - s
        _plain(scene, f"{name}/station{i}/deck", (0.80, 0.55, 0.08), (x, y0 + 0.60, 1.02), RAL_7016)
        _plain(scene, f"{name}/station{i}/pedestal", (0.22, 0.22, 0.32), (x, y0 + 0.60, 1.22), RAL_9003)
        _plain(scene, f"{name}/station{i}/cabinet", (0.44, 0.38, 0.56), (x + 0.48, y0 + 0.80, 0.34), RAL_7035)
        _plain(scene, f"{name}/station{i}/door", (0.40, 0.008, 0.50), (x + 0.48, y0 + 0.604, 0.34), RAL_9003)
