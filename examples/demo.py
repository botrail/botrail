"""botrail demo: a Franka Panda in a small USD factory cell.

The robot is NVIDIA's official Isaac Sim Franka asset (USD articulation);
the first run downloads it (~10 MB) into the botrail cache. The factory
environment is the hand-authored USD layer next to this script.

Run with:  python examples/demo.py
"""

import os
import urllib.request
from pathlib import Path

import botrail as bt

ISAAC_FRANKA_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/4.2/Isaac/Robots/Franka"
)
# franka.usd plus the sublayers it references relative to itself.
FRANKA_FILES = [
    "franka.usd",
    "franka-LICENSE.txt",
    "Materials/Materials.usd",
    *(
        f"Props/panda_{part}.usd"
        for part in ("hand", "leftfinger", "rightfinger", *(f"link{i}" for i in range(8)))
    ),
]


def fetch_franka() -> Path:
    """Download the Franka USD asset into the botrail cache (once)."""
    cache = Path(os.environ.get("BOTRAIL_CACHE_DIR") or Path.home() / ".cache" / "botrail")
    dest = cache / "assets" / "franka"
    for rel in FRANKA_FILES:
        target = dest / rel
        if target.exists():
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        print(f"downloading {rel} ...")
        part = target.with_suffix(target.suffix + ".part")
        urllib.request.urlretrieve(f"{ISAAC_FRANKA_URL}/{rel}", part)
        part.rename(target)
    return dest / "franka.usd"


def build_scene() -> bt.Scene:
    robot = bt.Robot.from_usd(fetch_franka())
    scene = bt.Scene(robot)
    scene.load_usd(Path(__file__).parent / "assets" / "factory.usda")
    # Stand the robot on the pedestal's mount frame, in a natural ready pose.
    scene.set_robot_base_pose(*scene.frame("/World/MountFrame"))
    scene.set_joint_positions([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.035, 0.035])
    return scene


# --------------------------------------------------------------- grasping

# The cell's teach frames (PickFrame / PlaceFrame) are *grasp* poses: the
# point between the fingertips, tool +Z along the approach direction. IK,
# though, solves for a link — so a taught pose has to be backed off along
# the tool axis to the hand frame. On this Franka the palm hull ends 67 mm
# out and the fingertips reach 112 mm, so holding a box 100 mm out puts the
# pads across it with the palm clear of its top face.
HAND = "/panda/panda_hand"
GRIP_DEPTH = 0.10


def hand_pose(pose, standoff: float = 0.0):
    """The ``panda_hand`` pose that lands the finger pads on ``pose``
    (a ``(position, quaternion)`` grasp frame), backed off by ``standoff``
    along the approach axis for a hover pose."""
    (x, y, z), quat = pose
    qx, qy, qz, qw = quat
    tool_z = (2 * (qx * qz + qw * qy), 2 * (qy * qz - qw * qx), 1 - 2 * (qx * qx + qy * qy))
    depth = GRIP_DEPTH + standoff
    return tuple(p - depth * a for p, a in zip((x, y, z), tool_z)), quat


def teach_grasp(scene: bt.Scene, pose, standoff: float = 0.0) -> list:
    """Poses the robot so the gripper grasps at ``pose`` and returns the
    joint vector — the scripted form of dragging the studio's TCP gizmo.
    Raises when IK misses, rather than teaching a bad pose."""
    position, quaternion = hand_pose(pose, standoff)
    ik = scene.set_tcp_target(position, quaternion, link=HAND)
    if not ik.converged:
        raise RuntimeError(
            f"IK did not reach {tuple(round(v, 3) for v in position)}: "
            f"{ik.pos_error * 1e3:.1f} mm / {ik.rot_error:.3f} rad short"
        )
    return list(scene.joint_positions)


if __name__ == "__main__":
    scene = build_scene()
    print(scene.robot)
    bt.studio(scene)
