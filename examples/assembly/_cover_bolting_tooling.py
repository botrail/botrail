"""OnRobot dual tooling, authored from manufacturer drawings at run time.

The catalog supplies the RG6. The changer and side-mounted screwdriver are
dimensioned reference geometry, not vendor CAD. See cover_bolting_demo.md.
URDF primitives retain distinct moving links and collision geometry.
"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET

import botrail as bt

# r2 supplies the same geometry without r3's single-QC wrist-power purchase
# configuration, which does not apply to this Compute Box / Dual QC cell.
GRIPPER_CATALOG = "onrobot/rg/rg6/r2"
CHANGER_MODEL = "Dual Quick Changer v3 (109878)"
DRIVER_MODEL = "Screwdriver (103961)"
CHANGER_SOURCE = "https://onrobot.com/storage/datasheets/quick-changers/datasheet_quick_changers_v2.0_en.pdf"
DRIVER_SOURCE = "https://www.wmh-trans.co.uk/file.php?filename=ONR-103961%2FScrewdriver+-+Datasheet+v1.1.pdf"
# Quick Changers v2.0 p8: 127 wide, 71 diameter, 93.5 tall, 120° normals.
FACE_X = (0.127 - 0.071 * math.cos(math.pi / 3)) / 2
FACE_Z = 0.0935 - 0.071 / 2 * math.sin(math.pi / 3)
CHANGER_MASS = 0.41
DRIVER_MASS = 2.5
ALUMINIUM = (0.52, 0.54, 0.57)
DARK = (0.035, 0.043, 0.052)
BLUE = (0.04, 0.34, 0.60)


def _numbers(values):
    return " ".join(f"{v:.12g}" for v in values)


def _shape(link, kind, size, at, color, *, rpy=(0, 0, 0), collision=True):
    for tag in (("visual", "collision") if collision else ("visual",)):
        element = ET.SubElement(link, tag)
        ET.SubElement(element, "origin", xyz=_numbers(at), rpy=_numbers(rpy))
        geometry = ET.SubElement(element, "geometry")
        attrs = {"size": _numbers(size)} if kind == "box" else {"radius": str(size[0]), "length": str(size[1])}
        ET.SubElement(geometry, kind, **attrs)
        if tag == "visual":
            material = ET.SubElement(element, "material", name="")
            ET.SubElement(material, "color", rgba=_numbers((*color, 1)))


def _fixed(root, parent, child, at=(0, 0, 0), rpy=(0, 0, 0)):
    joint = ET.SubElement(root, "joint", name=f"{child}_fixed", type="fixed")
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=_numbers(at), rpy=_numbers(rpy))


def _housing(link, length, x, radius, color):
    """Extruded rounded housing made from bt.parts runtime primitives."""
    source = bt.Scene()
    bt.parts.compound(source, "screwdriver/housing", [
        bt.parts.Box((0.028, radius * 2, length)),
        bt.parts.Cylinder(radius, length, (-0.014, 0, -length / 2)),
        bt.parts.Cylinder(radius, length, (0.014, 0, -length / 2)),
    ], (0, 0, 0), segments=64)
    mesh = json.loads(source._project_json())["obstacles"][0]["geometry"]["url"]
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", xyz=_numbers((x, 0, 0.081)), rpy=_numbers((0, math.pi / 2, 0)))
    ET.SubElement(ET.SubElement(visual, "geometry"), "mesh", filename=mesh)
    ET.SubElement(ET.SubElement(visual, "material", name=""), "color", rgba=_numbers((*color, 1)))
    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", xyz=_numbers((x, 0, 0.081)))
    ET.SubElement(ET.SubElement(collision, "geometry"), "box",
                  size=_numbers((length, radius * 2, 0.028 + radius * 2)))


def dual_changer():
    """Two integral QC faces; no boom or additional robot-side QC."""
    root = ET.Element("robot", name="onrobot_dual_quick_changer_109878_reference")
    ET.SubElement(root, "link", name="mount")
    body = ET.SubElement(root, "link", name="hand_body")
    _fixed(root, "mount", "hand_body")
    _shape(body, "cylinder", (0.0355, 0.018), (0, 0, 0.009), ALUMINIUM)
    _shape(body, "box", (0.052, 0.052, 0.038), (0, 0, 0.034), ALUMINIUM)
    for name, sign in (("driver", 1), ("gripper", -1)):
        angle = sign * math.pi / 3
        # Integral angled support and latch face, contained in the drawing's
        # outside dimensions. Small latch/connector details are approximate.
        center = (sign * FACE_X, 0, FACE_Z)
        normal = (math.sin(angle), 0, math.cos(angle))
        back = tuple(p - n * 0.010 for p, n in zip(center, normal))
        _shape(body, "cylinder", (0.0355, 0.020), back, ALUMINIUM, rpy=(0, angle, 0))
        _shape(body, "box", (0.023, 0.012, 0.008),
               (sign * FACE_X, -0.030, FACE_Z), DARK, collision=False)
        ET.SubElement(root, "link", name=f"hand_{name}")
        _fixed(root, "hand_body", f"hand_{name}", center, (0, angle, 0))
    for x in (-0.01768, 0.01768):
        for y in (-0.01768, 0.01768):
            _shape(body, "cylinder", (0.005, 0.002), (x, y, 0.019), DARK, collision=False)
    return bt.Robot.from_urdf_string(ET.tostring(root, encoding="unicode"))


def screwdriver(*, stroke):
    """103961 side mount: +X is the screw axis, +Z enters the housing.

    The nose datum is (153, 0, 81) mm, manufacturer UR manual §8.3.1.
    The 50 mm Type A extender (109301) clears the cover boss. Shank zero
    is a process reference 17 mm inside the nominal TCP plus the extender,
    not the vendor controller's home value. Extended screws are exposed;
    the standard tool's full-retraction claim does not apply.
    """
    root = ET.Element("robot", name="onrobot_screwdriver_103961_reference")
    ET.SubElement(root, "link", name="mount")
    body = ET.SubElement(root, "link", name="body")
    _fixed(root, "mount", "body")
    # Overall housing section 86 × 114 mm; rear -155.4, shoulder +93.2.
    axis = (0, math.pi / 2, 0)
    _shape(body, "cylinder", (0.032, 0.024), (0, 0, 0.012), ALUMINIUM)
    # Flat backing pad meets the curved housing across the mounting face.
    # It stays inside the coupling/body collision envelopes already present.
    _shape(body, "cylinder", (0.032, 0.024), (0, 0, 0.028), ALUMINIUM, collision=False)
    _housing(body, 0.220, -0.0311, 0.043, ALUMINIUM)
    for x in (-0.1484, 0.0862):
        _housing(body, 0.014, x, 0.041, DARK)
    for y in (-0.044, 0.044):
        _shape(body, "box", (0.216, 0.002, 0.002), (-0.0311, y, 0.113), ALUMINIUM, collision=False)
        _shape(body, "box", (0.050, 0.002, 0.028), (-0.050, y, 0.077), BLUE, collision=False)
    nose = ET.SubElement(root, "link", name="nose")
    _fixed(root, "body", "nose")
    _shape(nose, "cylinder", (0.0245, 0.033), (0.1097, 0, 0.081), ALUMINIUM, rpy=axis)
    # Stepped approximation of the moulded taper; carrier internals omitted.
    for i in range(8):
        length = (0.153 - 0.1262) / 8
        radius = 0.0245 - (0.0245 - 0.00675) * (i + 0.5) / 8
        _shape(nose, "cylinder", (radius, length),
               (0.1262 + length * (i + 0.5), 0, 0.081), ALUMINIUM, rpy=axis)
    bit = ET.SubElement(root, "link", name="bit")
    _shape(bit, "cylinder", (0.003, 0.010), (0, 0, -0.005), DARK)
    _shape(bit, "cylinder", (0.0061, 0.050), (0, 0, -0.035), ALUMINIUM)
    joint = ET.SubElement(root, "joint", name="shank", type="prismatic")
    ET.SubElement(joint, "parent", link="nose")
    ET.SubElement(joint, "child", link="bit")
    ET.SubElement(joint, "origin", xyz="0.186 0 0.081", rpy=_numbers(axis))
    ET.SubElement(joint, "axis", xyz="0 0 1")
    ET.SubElement(joint, "limit", lower="0", upper=str(stroke), effort="50", velocity="0.2")
    ET.SubElement(root, "link", name="tip")
    _fixed(root, "bit", "tip", rpy=(math.pi, 0, 0))
    return bt.Robot.from_urdf_string(ET.tostring(root, encoding="unicode"))


def pads(robot):
    return [name for name in robot.link_names if name.endswith(("finger_tip", "flex_finger"))]


def grip_tcp_offset(robot, width):
    """RG6 pad-center offset from its fixed TCP at the required opening.

    Teach against a gauge tall enough to contact the sides of the pads,
    then measure the finger-tip frames. These frames sit within 0.1 mm of
    the pad centers in this catalog revision. The RG6's curved finger
    motion means its fixed TCP is not a contact point for every width.
    """
    probe = bt.Scene(robot)
    tcp, q = probe.link_pose(robot.tcp_link)
    x, y, z, w = q
    axis = (2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y))
    center = tuple(p - 0.030 * n for p, n in zip(tcp, axis))
    probe.add_box("grip_gauge", (width, width, 0.035), center, quaternion=q)
    finger = robot.joint_names[-1]
    close = probe.grasp_close("grip_gauge", joints=[finger], clearance=0.0)
    joints = list(probe.joint_positions)
    joints[-1] = close[finger]
    probe.set_joint_positions(joints)
    tips = [probe.link_pose(name)[0] for name in pads(robot) if name.endswith("finger_tip")]
    midpoint = tuple(sum(p[i] for p in tips) / len(tips) for i in range(3))
    return sum((p - t) * n for p, t, n in zip(midpoint, tcp, axis))
