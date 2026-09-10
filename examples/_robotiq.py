"""Shared Robotiq r2 model selection for the demos.

`attach` puts the catalog 2F-85 r2 on a host either as the purchased UR
ES-062 kit or as the coupling and the hand on a custom bracket; `pads`
and `close_for_width` are the pad list and the geometric close the demos
share.
"""
from __future__ import annotations

import botrail as bt

ARM = "universal_robots/ur/ur5e/r2"
HAND_R2 = "robotiq/2f/2f-85/r2"
COUPLING_ES062 = "robotiq/coupling/grp-es-cpl-062/r1"
KIT_R2 = "robotiq/2f/2f-85-ur-es-062-kit/r2"


def load_arm(product=ARM):
    return bt.Robot.from_catalog(product)


def load(product):
    return bt.Robot.from_catalog(product)


def attach(host, *, purchase_kit=True):
    if purchase_kit:
        return host.attach_tool(load(KIT_R2), prefix="kit_")
    # A custom bracket does not inherit the UR purchase kit's support.
    coupling = load(COUPLING_ES062)
    hand = load(HAND_R2)
    tool = coupling.attach_tool(hand, prefix="gripper/", offset_quaternion=(
        0.0, 0.0, -0.7071067811865475, 0.7071067811865476))
    return host.attach_tool(tool, prefix="kit_")


def is_r2(robot):
    return "kit_gripper/finger_joint" in robot.joint_names


def pads(robot):
    prefix = "kit_gripper/" if is_r2(robot) else ""
    return [f"{prefix}{side}_inner_{part}" for side in ("left", "right")
            for part in ("finger", "finger_pad", "knuckle")]


def close_for_width(robot, width, legacy):
    """Teach a square workpiece from the loaded pad geometry, in metres.

    This is a geometric joint value, not the real controller's command.
    A temporary scene keeps the cell, its moving parts and its teaching pose
    unchanged. Existing r1 recordings keep their original taught value.
    """
    if not is_r2(robot):
        return legacy
    probe = bt.Scene(robot)
    left = probe.link_pose("kit_gripper/left_grasp")[0]
    right = probe.link_pose("kit_gripper/right_grasp")[0]
    centre = tuple((a + b) / 2 for a, b in zip(left, right))
    _, rotation = probe.link_pose(robot.tcp_link)
    # Only the pad band participates: a full-height cube can hit a knuckle
    # before the jaws reach the requested width.
    probe.add_box("workpiece", (width, width, 0.02), centre)
    probe.set_obstacle_pose("workpiece", centre, rotation)
    return probe.grasp_close("workpiece")[robot.joint_names[-1]]
