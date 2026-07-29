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


if __name__ == "__main__":
    scene = build_scene()
    print(scene.robot)
    bt.studio(scene)
