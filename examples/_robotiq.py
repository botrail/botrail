"""Shared model selection for the Robotiq demos.

Normal runs download the built r2 hand and ES-062 kit from the catalog.
The optional local root is retained for development builds.
"""
from __future__ import annotations

from pathlib import Path

import botrail as bt

ARM = "universal_robots/ur/ur5e/r2"
HAND_R2 = "robotiq/2f/2f-85/r2"
COUPLING_ES062 = "robotiq/coupling/grp-es-cpl-062/r1"
KIT_R2 = "robotiq/2f/2f-85-ur-es-062-kit/r2"


def add_argument(parser):
    parser.add_argument("--robotiq-r2", type=Path, metavar="CATALOG_ROOT",
                        help="override the catalog r2 with a local development build")


def load_arm(product=ARM, root=None):
    path = Path(root) / product if root is not None else None
    return bt.Robot.from_package(path) if path is not None and path.is_dir() else bt.Robot.from_catalog(product)


def load(product, root=None):
    if root is None:
        return bt.Robot.from_catalog(product)
    return bt.Robot.from_package(Path(root) / product, catalog_root=Path(root))


def attach(host, root=None, *, purchase_kit=True):
    if purchase_kit:
        return host.attach_tool(load(KIT_R2, root), prefix="kit_")
    # A custom bracket does not inherit the UR purchase kit's support.
    coupling = load(COUPLING_ES062, root)
    hand = load(HAND_R2, root)
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
