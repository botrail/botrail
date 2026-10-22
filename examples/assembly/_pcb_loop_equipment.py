"""Geometry and transport coordinates for the photo-reference PCB cell.

All cell dimensions are metres and remain Python parameters. Only curved product
surfaces come from authored USD; imported visuals get explicit runtime state.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import botrail as bt
from _shuttle_line_equipment import _box_xml

SILVER = (0.65, 0.69, 0.71)
DARK = (0.026, 0.030, 0.033)
YELLOW = (0.88, 0.70, 0.005)
GREEN = (0.035, 0.34, 0.11)
CURVE = "ats/supertrak/horizon10-curve90/r1"
STRAIGHT = "ats/supertrak/horizon10-straight1000/r1"
SHUTTLE = "ats/supertrak/horizon10-shuttle/r1"


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def rotate(x, y, a):
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


def box(scene, name, size, pos, color=SILVER, *, yaw=0.0, collide=False):
    obj = scene.add_box(name, size, pos, quaternion=qz(yaw), color=color)
    scene.set_obstacle_enabled(obj, collide)
    scene.set_obstacle_material(
        obj, metalness=0.4 if color == SILVER else 0.0, roughness=0.45
    )
    return obj


@dataclass(frozen=True)
class Loop:
    sections: int = 1
    radius: float = 0.650  # Nominal 600 mm rail + 50 mm SI carrier offset.
    top: float = 0.89  # Shuttle mounting plane; stand height remains editable.
    count: int = 8
    phase: float = 0.25  # Initial carrier spacing offset, in pitches.

    def __post_init__(self):
        if self.sections < 1 or not isinstance(self.sections, int):
            raise ValueError("sections must be a positive integer")
        if self.count < 4 or self.count > 16:
            raise ValueError("carrier count must be between 4 and 16")

    @property
    def length(self):
        return float(self.sections)

    @property
    def lap(self):
        return 2 * self.length + 2 * math.pi * self.radius

    @property
    def pitch(self):
        return self.lap / self.count

    @property
    def pcb_z(self):
        return self.top + 0.034

    def q(self, distance):
        """One translation and one rotation; continuous through every lap boundary."""
        lap, d = divmod(distance + self.phase * self.pitch, self.lap)
        arc, L = math.pi * self.radius, self.length
        turn = -2 * math.pi * lap
        if d <= L:
            return (-L / 2 + d, turn)
        if d <= L + arc:
            return (L / 2, turn - (d - L) / self.radius)
        if d <= 2 * L + arc:
            return (L / 2 - (d - L - arc), turn - math.pi)
        return (-L / 2, turn - math.pi - (d - 2 * L - arc) / self.radius)

    def pose(self, distance):
        slide, angle = self.q(distance)
        return (
            -self.radius * math.cos(angle),
            slide - self.radius * math.sin(angle),
            self.pcb_z,
        ), qz(angle + math.pi / 2)

    def cuts(self, start, end):
        """Joint ramps must end at line/arc junctions; never interpolate across them."""
        L, a = self.length, math.pi * self.radius
        points = [start, end]
        offset = self.phase * self.pitch
        for k in range(
            math.floor((start + offset) / self.lap),
            math.floor((end + offset) / self.lap) + 1,
        ):
            points += [
                k * self.lap + v - offset
                for v in (L, L + a, 2 * L + a, self.lap)
                if start + 1e-8 < k * self.lap + v - offset < end - 1e-8
            ]
        return sorted(points)


def imported_module(scene, package, name, pid, at, yaw):
    variant = pid.split("/")[-2]
    path = package / f"sources/ats-supertrak-horizon10/usd/{variant}-visual.usda"
    if not path.is_file():
        raise FileNotFoundError(
            f"Catalog package {pid} is missing its static visual: {path}"
        )
    objects = scene.load_usd(path, prefix=name)
    for obj in objects:
        (x, y, z), (qx, qy, qzz, qw) = scene.obstacle_pose(obj)
        xx, yy = rotate(x, y, yaw)
        s, c = math.sin(yaw / 2), math.cos(yaw / 2)
        quat = (c * qx - s * qy, c * qy + s * qx, c * qzz + s * qw, c * qw - s * qzz)
        scene.set_obstacle_pose(obj, (xx + at[0], yy + at[1], z + at[2]), quat)
        scene.set_obstacle_enabled(obj, False)
        scene.set_obstacle_visible(obj, True)
        scene.set_obstacle_material(
            obj,
            metalness=0.55 if "rail" in obj or "stator" in obj else 0.1,
            roughness=0.36,
        )
    scene.set_part(
        name,
        kind="group",
        catalog=pid,
        manufacturer="ATS Automation",
        model=f"SuperTrak HORIZON10 {pid.split('/')[-2]}",
        category="conveyor.linear_motor",
    )
    return objects


def track(scene, loop, catalog_root):
    """Four catalog 90-degree modules, two straight runs, SI stands and collision rails."""
    packages = {
        pid: catalog_root / pid
        if catalog_root and (catalog_root / pid / "manifest.yaml").is_file()
        else Path(bt.catalog_package(pid))
        for pid in (CURVE, STRAIGHT)
    }
    z = loop.top - 0.19
    for end, y, angles in (
        ("rear", loop.length / 2, (0, math.pi / 2)),
        ("front", -loop.length / 2, (math.pi, 3 * math.pi / 2)),
    ):
        for i, angle in enumerate(angles):
            imported_module(
                scene, packages[CURVE], f"track/{end}{i}", CURVE, (0, y, z), angle
            )
    for i in range(loop.sections):
        imported_module(
            scene,
            packages[STRAIGHT],
            f"track/right{i}",
            STRAIGHT,
            (0.6, -loop.length / 2 + i, z),
            0,
        )
        imported_module(
            scene,
            packages[STRAIGHT],
            f"track/left{i}",
            STRAIGHT,
            (-0.6, loop.length / 2 - i, z),
            math.pi,
        )
    # True opening through the transport. Enabled tangent boxes represent the stator;
    # rail/shuttle bearing contact is permitted only for each carrier's own body.
    collisions = []
    for side in (-1, 1):
        collisions.append(
            box(
                scene,
                f"track/collision/straight{side}",
                (0.07, loop.length, 0.18),
                (side * 0.565, 0, z + 0.09),
                collide=True,
            )
        )
    for end in (-1, 1):
        for i in range(24):
            a = (0 if end == 1 else math.pi) + (i + 0.5) * math.pi / 24
            collisions.append(
                box(
                    scene,
                    f"track/collision/arc{end}_{i}",
                    (0.074, 0.075, 0.18),
                    (
                        0.565 * math.cos(a),
                        end * loop.length / 2 + 0.565 * math.sin(a),
                        z + 0.09,
                    ),
                    yaw=a,
                    collide=True,
                )
            )
    for obj in collisions:
        scene.set_obstacle_visible(obj, False)
    for side in (-1, 1):
        for end in (-1, 1):
            x, y = side * 0.49, end * (loop.length / 2 + 0.18)
            box(
                scene,
                f"track/stand{side}_{end}/foot",
                (0.22, 0.22, 0.025),
                (x, y, 0.0125),
                DARK,
            )
            box(
                scene,
                f"track/stand{side}_{end}/post",
                (0.09, 0.09, z),
                (x, y, z / 2),
                collide=True,
            )
            for dx in (-0.034, 0.034):
                box(
                    scene,
                    f"track/stand{side}_{end}/groove{dx}",
                    (0.004, 0.002, z - 0.06),
                    (x + dx, y - 0.046, z / 2),
                    DARK,
                )
    scene.set_part(
        "track",
        kind="group",
        manufacturer="System integrator",
        model="HORIZON10 cell support",
        category="structure.frame",
        description="Python stand height; purchased track modules have separate identities",
    )
    return collisions


def carrier(loop, model, *, laps=3):
    motion = bt.Robot.from_urdf_string(
        '<robot name="shuttle_motion"><link name="rail"/><link name="slide"/><link name="turn"/>'
        f'<joint name="s" type="prismatic"><parent link="rail"/><child link="slide"/><axis xyz="0 1 0"/>'
        f'<limit lower="{-loop.length / 2 - 0.001}" upper="{loop.length / 2 + 0.001}" velocity="4" effort="1"/></joint>'
        '<joint name="angle" type="revolute"><parent link="slide"/><child link="turn"/><axis xyz="0 0 1"/>'
        f'<limit lower="{-2 * math.pi * laps - 0.001}" upper="{2 * math.pi * laps + 0.001}" velocity="6" effort="1"/></joint></robot>'
    )
    motion = motion.mount(
        model,
        at="turn",
        offset_position=(-loop.radius, 0, 0),
        offset_quaternion=qz(math.pi / 2),
    )
    shape = _box_xml((0.196, 0.166, 0.008), (0, 0, 0.004), SILVER, True)
    for x in (-0.090, 0.090):
        shape += _box_xml((0.012, 0.166, 0.012), (x, 0, 0.014), SILVER)
    for y in (-0.077, 0.077):
        shape += _box_xml((0.172, 0.012, 0.012), (0, y, 0.014), SILVER)
    nest = bt.Robot.from_urdf_string(
        f'<robot name="pcb_nest"><link name="nest">{shape}</link></robot>'
    )
    return motion.mount(nest, at="surface")


def pcb(scene, name, pose, *, missing_ic=None):
    """One SI test PCB, including traces/pads; all members attach and transfer together."""
    (x, y, z), quat = pose
    yaw = 2 * math.atan2(quat[2], quat[3])
    objects = []

    def add(suffix, size, local, color):
        xx, yy = rotate(local[0], local[1], yaw)
        objects.append(
            box(
                scene,
                f"{name}/{suffix}",
                size,
                (x + xx, y + yy, z + local[2]),
                color,
                yaw=yaw,
            )
        )

    add("board", (0.145, 0.117, 0.002), (0, 0, 0), GREEN)
    for i in range(11):
        xx = -0.058 + i * 0.0115
        add(
            f"trace{i}",
            (0.0012, 0.084 - (i % 3) * 0.010, 0.00025),
            (xx, 0.0, 0.0012),
            (0.42, 0.62, 0.30),
        )
    # Two connector strips read as pads at the reference image's scale. Avoiding
    # 22 tiny separate pads per board keeps the sampled object poses
    # from adding tens of megabytes to the full-lap WebSocket payload.
    for side in (-1, 1):
        add(
            f"pad_strip{side}",
            (0.126, 0.004, 0.0005),
            (0, side * 0.047, 0.0013),
            (0.62, 0.66, 0.59),
        )
    for i in range(3):
        if i == missing_ic:
            continue
        add(f"ic{i}", (0.014, 0.014, 0.003), (-0.040 + i * 0.040, 0.018, 0.0025), DARK)
    add("silkscreen", (0.055, 0.001, 0.0003), (0, -0.039, 0.0014), (0.83, 0.88, 0.78))
    scene.set_obstacle_enabled(objects[0], True)
    scene.set_part(
        name,
        kind="group",
        manufacturer="System integrator",
        model="PCB inspection coupon",
        category="workpiece",
        description="145 x 117 mm illustrative PCB; no electrical function",
    )
    return objects


def surroundings(scene):
    box(scene, "hall/floor", (24.0, 24.0, 0.04), (0, 0, -0.019), (0.65, 0.66, 0.64))
    for axis in range(2):
        for i in range(-37, 38):
            size = (0.003, 24.0, 0.001) if axis == 0 else (24.0, 0.003, 0.001)
            pos = (i * 0.32, 0, 0.0018) if axis == 0 else (0, i * 0.32, 0.0018)
            box(scene, f"hall/tile{axis}_{i}", size, pos, (0.30, 0.31, 0.30))
    scene.set_part(
        "hall", kind="group", category="structure.floor", model="Tiled workshop floor"
    )
    # Open at the camera side, as the cropped reference. Guard dimensions remain here.
    panels = []
    for x in (-1.30, -0.60, 0.10, 0.80, 1.50):
        box(
            scene,
            f"guard/back_post{x}",
            (0.065, 0.065, 2.15),
            (x, 1.58, 1.075),
            DARK,
            collide=True,
        )
        box(scene, f"guard/back_foot{x}", (0.16, 0.16, 0.025), (x, 1.58, 0.0125), DARK)
    for x in (-0.95, -0.25, 0.45, 1.15):
        panels.append((x, 1.58, 0.70, 0.0))
    for y in (-1.22, -0.52, 0.18, 0.88):
        box(
            scene,
            f"guard/left_post{y}",
            (0.065, 0.065, 2.15),
            (-1.30, y, 1.075),
            DARK,
            collide=True,
        )
        panels.append((-1.30, y + 0.35, 0.70, math.pi / 2))
    for j, (x, y, w, yaw) in enumerate(panels):
        box(
            scene,
            f"guard/panel{j}/lower",
            (w - 0.065, 0.016, 0.64),
            (x, y, 0.50),
            YELLOW,
            yaw=yaw,
            collide=True,
        )
        for z in (0.15, 0.84, 2.02):
            box(
                scene,
                f"guard/panel{j}/bar{z}",
                (w - 0.04, 0.028, 0.035),
                (x, y, z),
                YELLOW,
                yaw=yaw,
            )
        # Square wire mesh, thin enough for the equipment to remain legible.
        for k in range(1, 10):
            xx, yy = rotate(-w / 2 + k * w / 10, 0, yaw)
            box(
                scene,
                f"guard/panel{j}/wirev{k}",
                (0.0015, 0.002, 1.15),
                (x + xx, y + yy, 1.43),
                DARK,
                yaw=yaw,
            )
        for k in range(1, 16):
            box(
                scene,
                f"guard/panel{j}/wireh{k}",
                (w - 0.04, 0.002, 0.0015),
                (x, y, 0.85 + k * 0.072),
                DARK,
                yaw=yaw,
            )
    scene.set_part(
        "guard",
        kind="group",
        manufacturer="System integrator",
        model="Yellow panel / mesh enclosure",
        category="structure.fence",
        description="Photo-derived custom enclosure; no product or safety-distance certification claimed",
    )


def peripherals(scene, bases):
    for name, at in bases.items():
        x, y, h = at
        if name == "ur_aoi":
            box(scene, f"stand/{name}/foot", (0.32, 0.30, 0.025), (x, y, 0.0125), DARK)
            scene.add_cylinder(
                f"stand/{name}/column",
                0.105,
                h - 0.08,
                (x, y, (h - 0.08) / 2),
                color=SILVER,
            )
            scene.add_cylinder(
                f"stand/{name}/top", 0.155, 0.08, (x, y, h - 0.04), color=SILVER
            )
            stripe = scene.add_cylinder(
                f"stand/{name}/stripe",
                0.156,
                0.018,
                (x, y, h - 0.054),
                color=(0.20, 0.45, 0.16),
            )
            scene.set_obstacle_enabled(stripe, False)
            scene.set_part(
                f"stand/{name}",
                kind="group",
                category="structure.pedestal",
                manufacturer="System integrator",
                model="Stepped circular pedestal",
            )
            continue
        bt.parts.pedestal(
            scene,
            f"stand/{name}",
            h,
            (x, y),
            top=(0.28, 0.26),
            base=(0.36, 0.34),
            column=0.18,
            color=(0.60, 0.63, 0.63),
            model="SI welded robot pedestal",
        )
    bt.parts.table(
        scene, "vision/stand", (0.26, 0.30, 0.91), (-0.92, -0.40, 0), color=DARK
    )
    box(scene, "vision/enclosure", (0.18, 0.20, 0.14), (-0.92, -0.40, 1.00), DARK)
    box(
        scene,
        "vision/window",
        (0.12, 0.003, 0.065),
        (-0.92, -0.503, 1.01),
        (0.01, 0.03, 0.04),
    )
    box(scene, "vision/camera", (0.045, 0.055, 0.085), (-0.70, -0.30, 1.60), DARK)
    box(
        scene, "vision/camera_mount", (0.025, 0.030, 0.09), (-0.70, -0.30, 1.68), SILVER
    )
    box(scene, "vision/cable", (0.005, 0.006, 0.25), (-0.70, -0.30, 1.84), DARK)
    scene.set_part(
        "vision",
        kind="group",
        manufacturer="System integrator",
        model="Inspection station envelope",
        category="sensor.camera",
        description="Unidentified camera in reference; illustrative optics and inspection timing",
    )


def process_equipment(scene, loop, count):
    """SI top-pick magazine, electrical clamp and two slotted output racks."""
    feed_q = loop.pose(3 * loop.pitch)[1]
    feed_yaw = 2 * math.atan2(feed_q[2], feed_q[3])
    test_q = loop.pose(6 * loop.pitch)[1]
    test_yaw = 2 * math.atan2(test_q[2], test_q[3])
    out_q = loop.pose(4 * loop.pitch)[1]
    out_yaw = 2 * math.atan2(out_q[2], out_q[3])

    def rack(name, xy, yaw, color, *, bottom, slots):
        if name == "feeder":
            # Cantilever over the rear curve; no table leg crosses a carrier path.
            box(
                scene,
                "feeder/stand/top",
                (0.27, 0.25, 0.012),
                (*xy, bottom - 0.018),
                SILVER,
                collide=True,
            )
            box(
                scene,
                "feeder/stand/column",
                (0.075, 0.075, bottom - 0.03),
                (xy[0], 1.42, (bottom - 0.03) / 2),
                SILVER,
                collide=True,
            )
            box(
                scene,
                "feeder/stand/arm",
                (0.065, 1.42 - xy[1], 0.035),
                (xy[0], (1.42 + xy[1]) / 2, bottom - 0.030),
                SILVER,
                collide=True,
            )
            box(
                scene,
                "feeder/stand/foot",
                (0.22, 0.18, 0.025),
                (xy[0], 1.42, 0.0125),
                DARK,
            )
        else:
            bt.parts.table(
                scene,
                f"{name}/stand",
                (0.27, 0.25, bottom - 0.012),
                (*xy, 0),
                color=SILVER,
            )
        for side in (-1, 1):
            dx, dy = rotate(side * 0.091, 0, yaw)
            box(
                scene,
                f"{name}/guide{side}",
                (0.012, 0.17, slots * 0.012 + 0.014),
                (xy[0] + dx, xy[1] + dy, bottom + slots * 0.006),
                color,
                yaw=yaw,
                collide=True,
            )
            for slot in range(slots):
                dx, dy = rotate(side * 0.078, 0, yaw)
                box(
                    scene,
                    f"{name}/finger{side}_{slot}",
                    (0.015, 0.12, 0.003),
                    (xy[0] + dx, xy[1] + dy, bottom + slot * 0.012 - 0.004),
                    color,
                    yaw=yaw,
                )
        scene.set_part(
            name,
            kind="group",
            manufacturer="System integrator",
            category="feeder.tray",
            model=name.replace("/", " ").title(),
            description="Top-access PCB rack with side support fingers; Python dimensions",
        )

    magazine_xy, bottom = (-0.08, 1.14), 1.00
    rack(
        "feeder",
        magazine_xy,
        feed_yaw,
        (0.025, 0.036, 0.32),
        bottom=bottom,
        slots=count,
    )
    magazine = [
        ((*magazine_xy, bottom + (count - 1 - i) * 0.012), feed_q) for i in range(count)
    ]
    outputs = {}
    for name, xy, color in (
        ("good", (1.22, 0.18), (0.035, 0.38, 0.10)),
        ("reject", (1.22, 0.70), (0.65, 0.025, 0.015)),
    ):
        rack(f"output/{name}", xy, out_yaw, color, bottom=0.92, slots=count)
        outputs[name] = [((*xy, 0.92 + i * 0.012), out_q) for i in range(count)]

    fixture = ((0.74, -1.38, 0.94), test_q)
    x, y, z = fixture[0]
    bt.parts.table(
        scene, "electrical/stand", (0.34, 0.30, z - 0.05), (x, y, 0), color=SILVER
    )
    box(
        scene,
        "electrical/bed",
        (0.23, 0.20, 0.035),
        (x, y, z - 0.023),
        DARK,
        yaw=test_yaw,
        collide=True,
    )
    for ix in (-1, 1):
        for iy in (-1, 1):
            dx, dy = rotate(ix * 0.050, iy * 0.038, test_yaw)
            box(
                scene,
                f"electrical/probe{ix}_{iy}",
                (0.008, 0.008, 0.004),
                (x + dx, y + dy, z - 0.004),
                (0.70, 0.49, 0.08),
                collide=True,
            )
    fixed = "".join(
        _box_xml((0.018, 0.17, 0.10), (side * 0.115, 0, 0.045), SILVER, True)
        for side in (-1, 1)
    )
    clamps = ""
    for side in (-1, 1):
        shape = _box_xml(
            (0.012, 0.13, 0.012), (side * 0.095, 0, 0.008), (0.05, 0.11, 0.22), True
        )
        clamps += (
            f'<link name="clamp{side}">{shape}</link><joint name="close{side}" type="prismatic">'
            f'<parent link="frame"/><child link="clamp{side}"/><axis xyz="{-side} 0 0"/>'
            '<limit lower="0" upper="0.029" velocity="0.2" effort="100"/></joint>'
        )
    press = bt.Robot.from_urdf_string(
        f'<robot name="ict_clamp"><link name="frame">{fixed}</link>{clamps}</robot>'
    )
    scene.add_robot(
        press, name="ict_clamp", base_position=fixture[0], base_quaternion=test_q
    )
    # The fixed clamp frame is bolted to the nest; moving jaws remain checked.
    scene.allow_link_obstacle_contact("frame", "electrical/bed", robot="ict_clamp")
    scene.set_part(
        "ict_clamp",
        kind="robot",
        manufacturer="System integrator",
        category="fixture",
        model="PCB electrical test edge clamp",
        description="Moving edge clamps and four probe blocks; simulated resistance input",
    )
    scene.set_part(
        "electrical",
        kind="group",
        manufacturer="System integrator",
        category="fixture",
        model="PCB electrical test nest",
    )
    return {"magazine": magazine, "fixture": fixture, "outputs": outputs}
