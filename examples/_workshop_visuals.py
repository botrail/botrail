"""Plain decoration helpers for the finishing-line example's unidentified
bath: boxes and cylinders out of collision. The curved and perforated forms
come from botrail's shape library (`bt.parts.appearance` /
`bt.parts.shaped_box`) and the catalog products draw themselves. Positions,
dimensions, supports and collision/interaction state remain in Python.
"""
from __future__ import annotations

STEEL = (0.55, 0.59, 0.61)
GREEN = (0.13, 0.25, 0.18)
DARK = (0.045, 0.055, 0.06)


def box(scene, name, size, at, *, color=STEEL, q=None, metalness=0.0, roughness=0.55):
    made = scene.add_box(name, size, at, quaternion=q, color=color)
    scene.set_obstacle_enabled(made, False)
    scene.set_obstacle_material(made, metalness=metalness, roughness=roughness)
    return made


def cylinder(scene, name, radius, length, at, *, q=None, color=STEEL):
    made = scene.add_cylinder(name, radius, length, at, quaternion=q, color=color)
    scene.set_obstacle_enabled(made, False)
    scene.set_obstacle_material(made, metalness=0.8, roughness=0.3)
    return made
