"""Commercial tooling with SI-built fingers and holders for this demo.

Adapters are layout designs, not qualified FANUC mating interfaces. Catalog
models retain their own mount and nominal TCP. The composite's TCP is the
actual grasp centre or suction plane. Lengths are metres.
"""

from __future__ import annotations

import botrail as bt
from _shuttle_line_equipment import ANODISED, DARK, _box_xml, _cyl_xml

HOUSING_GRIPPER = "smc/mhz2/mhz2-20d/r1"
CONNECTOR_GRIPPER = "schunk/mpg-plus/mpg-plus-25/r1"
PCB_CUP = "schmalz/pfyn/pfyn-6-esd/r1"
COVER_CUP = "smc/zp3/zp3-t10umn-a5/r1"
EJECTOR = "schmalz/scpmc/scpmc-05/r1"
OPEN, CLOSED = 0.005, 0.0
CONNECTOR_OPEN = 0.003
CUP_SPACING = {"s2": 0.044, "s3": 0.052}
CUP_RADIUS = {"s2": 0.003, "s3": 0.005}
FINGER_CONTACTS = ["left_custom_finger", "right_custom_finger"]
CUP_CONTACTS = ["left_cup_mount", "right_cup_mount"]


def _fixed_part(name: str, visual: str, *, tcp_z: float | None = None) -> bt.Robot:
    xml = f'<robot name="{name}"><link name="{name}">{visual}</link>'
    if tcp_z is not None:
        xml += (f'<link name="{name}_tcp"/><joint name="{name}_tcp_joint" type="fixed">'
                f'<parent link="{name}"/><child link="{name}_tcp"/>'
                f'<origin xyz="0 0 {tcp_z}"/></joint>')
    return bt.Robot.from_urdf_string(xml + "</robot>")


def parallel_hand(arm: bt.Robot, load, *, connector: bool = False) -> bt.Robot:
    """Customer fingers: 40–50 mm housing gap, or 26–32 mm connector gap.

    The housing fingers clear the finished cover and connector above the grip.
    Catalog nominal forces are not applied to these longer custom fingers.
    """
    grip = load(CONNECTOR_GRIPPER if connector else HOUSING_GRIPPER)
    adapter_h = 0.018 if connector else 0.016
    jaw_tip = 0.027 if connector else 0.0848
    grasp_z = jaw_tip + (0.018 if connector else 0.055)
    half_gap = 0.013 if connector else 0.020
    finger_w = 0.003 if connector else 0.004
    finger_depth = 0.009 if connector else 0.012
    jaw_x = 0.006 if connector else 0.01215
    tip_beyond = 0.006 if connector else 0.010
    for side, sign in (("left", 1), ("right", -1)):
        outer = half_gap + finger_w / 2
        # Shoulder sits on the catalog base jaw; a dogleg leaves room for the module.
        shoulder_w = outer - jaw_x + finger_w
        shoulder = _box_xml((shoulder_w, finger_depth, 0.004),
                            (sign * (outer + jaw_x) / 2, 0, jaw_tip + 0.002), ANODISED, True)
        finger_h = grasp_z + tip_beyond - jaw_tip - 0.004
        finger = _box_xml((finger_w, finger_depth, finger_h),
                         (sign * outer, 0, jaw_tip + 0.004 + finger_h / 2), DARK, True)
        grip = grip.mount(_fixed_part(f"{side}_custom_finger", shoulder + finger), at=f"{side}_jaw")
    grip = grip.mount(_fixed_part("grasp", "", tcp_z=grasp_z), at="mount")
    # The adapter occupies the real offset. It is authored as a separate demo part.
    size = (0.034, 0.028, adapter_h) if connector else (0.058, 0.048, adapter_h)
    adapter = _fixed_part("tool_adapter", _box_xml(size, (0, 0, adapter_h / 2), ANODISED, True),
                          tcp_z=adapter_h)
    arm = arm.attach_tool(adapter, tcp="tool_adapter_tcp")
    return arm.attach_tool(grip, flange="tool_adapter_tcp", tcp="grasp_tcp")


def vacuum_hand(arm: bt.Robot, load, station: str) -> bt.Robot:
    """Two separate catalog cups on a parameterized, independently made holder."""
    cup_id = PCB_CUP if station == "s2" else COVER_CUP
    cup_h = 0.007 if station == "s2" else 0.0095
    spacing = CUP_SPACING[station]
    shoulder_z = 0.065
    visuals = _box_xml((0.032, 0.032, 0.008), (0, 0, 0.004), ANODISED, True)
    visuals += _box_xml((0.014, 0.014, 0.039), (0, 0, 0.0275), ANODISED, True)
    visuals += _box_xml((spacing + 0.016, 0.014, 0.008), (0, 0, 0.051), ANODISED, True)
    for sign in (-1, 1):
        x = sign * spacing / 2
        visuals += _cyl_xml(0.0045, 0.010, (x, 0, 0.060), ANODISED)
        # Short branch tubes and fittings; decorative, no collision envelope.
        visuals += _box_xml((spacing / 2, 0.003, 0.003), (x / 2, 0.008, 0.048), (0.04, 0.16, 0.30))
        visuals += _cyl_xml(0.003, 0.016, (x, 0.006, 0.048), DARK)
    holder = _fixed_part("vacuum_holder", visuals, tcp_z=shoulder_z + cup_h)
    for side, sign in (("left", -1), ("right", 1)):
        holder = holder.mount(load(cup_id), at="vacuum_holder", prefix=f"{side}_cup_",
                              offset_position=(sign * spacing / 2, 0, shoulder_z))
    return arm.attach_tool(holder, tcp="vacuum_holder_tcp")


def vacuum_services(scene: bt.Scene, load, positions, top: float, floor: float) -> None:
    """Fixed ejectors beside the SCARA bases. No hoses cross moving joints."""
    for station in ("s2", "s3"):
        x = positions[station]
        scene.add_robot(load(EJECTOR), name=f"vacuum_{station}",
                        base_position=(x + 0.09, 0.51, top))
        # Catalog identity stays on the ejector; its SI bracket has its own part.
        name = f"vacuum_bracket_{station}"
        scene.add_box(f"{name}/plate", (0.11, 0.04, 0.005),
                      (x + 0.09, 0.51, top - 0.0025), color=ANODISED)
        scene.add_box(f"{name}/post", (0.025, 0.025, top - floor - 0.005),
                      (x + 0.09, 0.51, (top + floor - 0.005) / 2), color=ANODISED)
        scene.add_box(f"{name}/foot", (0.070, 0.060, 0.005),
                      (x + 0.09, 0.51, floor + 0.0025), color=ANODISED)
        scene.set_part(name, kind="group", category="adapter", description="SI-built ejector support")
