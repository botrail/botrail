"""Runtime reference for Robotiq's panel-mounted Button Activator.

Public product: https://robotiq.com/solutions/machine-tending
The shell, brackets and tubes are authored approximations from the product
photograph, not dimensioned vendor CAD. Stroke follows the cell's button.
No undocumented mass, force, controller channel or order number is assigned.
"""
from __future__ import annotations

import botrail as bt

SOURCE = "https://robotiq.com/solutions/machine-tending"
KIT = "robotiq/hand-e/hand-e-ur-es-077-kit/r1"
OPEN = 0.025
PAD_DEPTH = 0.0125  # existing Hand-E catalog: TCP 146, pad center 133.5 mm


def pads(robot):
    return [n for n in robot.link_names if n.endswith(("left_finger", "right_finger"))]


def actuator_name(machine, button):
    return f"{machine}/button_activators/{button}"


def button_activators(scene, machine):
    """Three independently commanded cylinders; the moving feet trip the
    existing button zones. These remain sensed machine inputs, not writes
    to a simulated CNC's button signals. Extra pneumatic channels require
    integration; this is not a claim that one standard kit contains three.
    """
    for button in ("unclamp", "clamp", "cycle_start"):
        lane = f"{machine.name}/panel/{button}"
        p, q = scene.frame(lane)
        pressed, _ = scene.frame(f"{lane}/press")
        stroke = sum((a - b) ** 2 for a, b in zip(pressed, p)) ** 0.5
        axis = tuple((a - b) / stroke for a, b in zip(pressed, p))
        panel_face, _ = scene.frame(f"{machine.name}/panel")
        proud = sum((a - b) * n for a, b, n in zip(panel_face, p, axis))
        name = actuator_name(machine.name, button)

        def world(x, y, z):
            # Rotate a local point using the button's approach frame.
            qx, qy, qz, qw = q
            tx, ty, tz = 2 * (qy*z-qz*y), 2 * (qz*x-qx*z), 2 * (qx*y-qy*x)
            return (p[0]+x+qw*tx+qy*tz-qz*ty,
                    p[1]+y+qw*ty+qz*tx-qx*tz,
                    p[2]+z+qw*tz+qx*ty-qy*tx)

        def box(tag, size, at, color):
            obj = scene.add_box(f"{name}/{tag}", size, world(*at), quaternion=q, color=color)
            scene.set_obstacle_material(obj, metalness=0.65, roughness=0.32)
            return obj

        # Open-sided mounting bracket leaves manual access to the cap.
        box("bracket_base", (0.034, 0.009, 0.003), (0, 0.019, proud - 0.0015), (0.46, 0.48, 0.50))
        box("bracket_upright", (0.034, 0.003, proud + 0.0375),
            (0, 0.022, (proud - 0.0375) / 2), (0.46, 0.48, 0.50))
        def cylinder(tag, radius, length, at, color):
            obj = bt.parts.compound(scene, f"{name}/{tag}", [
                bt.parts.Cylinder(radius, length),
            ], world(*at), quaternion=q, color=color, segments=48)
            scene.set_obstacle_material(obj, metalness=0.15, roughness=0.38)
            return obj

        cylinder("body", 0.016, 0.027, (0, 0, -0.0375), (0.035, 0.039, 0.045))
        cylinder("manual_cap", 0.013, 0.004, (0, 0, -0.0415), (0.035, 0.33, 0.10))
        box("body_mount", (0.018, 0.009, 0.014), (0, 0.018, -0.025), (0.035, 0.039, 0.045))
        box("air_port", (0.009, 0.009, 0.009), (-0.018, 0, -0.025), (0.13, 0.38, 0.57))
        # A 1 mm rest gap, then the same 2.6 mm press as the original cell.
        foot = box("foot", (0.010, 0.010, 0.010), (0, 0, -0.006), (0.38, 0.40, 0.43))
        # Turn the tube down the panel rather than through the panel face.
        qx, qy, qz, qw = q
        half = 2 ** -0.5
        tube_q = tuple(v * half for v in (qx-qw, qy-qz, qz+qy, qw+qx))
        tube = bt.parts.compound(scene, f"{name}/air_tube", [
            bt.parts.Cylinder(0.002, 0.070, (0, 0, 0)),
        ], world(-0.021, 0.003, -0.025), quaternion=tube_q, color=(0.06, 0.08, 0.10))
        scene.set_obstacle_enabled(tube, False)
        scene.add_linear_axis(name, objects=[foot], axis=axis, speed=0.02,
                              range=(0, stroke + 0.001))
        # add_zone_sensor replaces the definition while retaining its name
        # and part metadata. Watch just this cylinder, never a nearby arm.
        center = tuple((a + b) / 2 for a, b in zip(p, pressed))
        scene.add_zone_sensor(lane, center, (0.022, 0.022, stroke), quaternion=q, watch=[foot])
        scene.set_part(name, kind="device", category="actuator.pneumatic",
                       manufacturer="Robotiq", model="Button Activator", source=SOURCE,
                       representation="authored_reference", dimensions_verified="no",
                       controller_mapping="unverified", simulated_stroke_mm=(stroke + 0.001)*1000)
