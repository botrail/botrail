"""botrail demo: a Franka Panda in a small USD factory cell.

The robot is NVIDIA's official Isaac Sim Franka asset (USD articulation);
the first run downloads it (~10 MB) into the botrail cache.

The cell comes from two places, which is how a real one is put together.
The *layout* — floor, pedestal, pallet, cabinet, the goods on the line and
the poses the motions are taught to — is the hand-authored USD layer next
to this script. The *standard products* — the belt, the rack, the guarding
— are ordered from the model catalog below: they are bought to size, so
what is written here is a length and a number of levels, the generator
refuses a size nobody sells, and every one of them lands on the bill of
materials with the part number you would order it by.

Run with:  python examples/basics/demo.py

Needs `pip install botrail[catalog]` for the equipment (the packages are
fetched from the Hugging Face dataset botrail/botrail-catalog and cached).
"""

import math
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


# ------------------------------------------------------- standard products

# Three catalog packages. Each one is a *spec pack*: no fixed mesh, but the
# sizes that are sold, the rules that govern them (how far apart a stand may
# stand, how close two shelves may be), the part numbers and the mass — plus
# a parametric drawing expanded to the size at hand, so a mesh panel is drawn
# as a frame with wire in it rather than a slab.
CONVEYOR = "botrail/conveyor/belt-unit"
RACK = "botrail/rack/medium-shelf"
FENCE = "botrail/fence/mesh-guard"

# The guard around the 4 m cell. It is not a loop but two open runs, because
# two things cross the perimeter and a run of panels can only stop at its
# ends: the belt comes in through the west edge (the corners at y 0.30 and
# 0.92 leave 560 mm between the posts, for a belt 460 mm over the rails),
# and the vehicle gate breaks the south one at x = +/-0.5 — 940 mm clear,
# which is what the AGV demo drives through.
#
# Every corner here is a length this fence can be built to: ask for a 1.1 m
# stub and it answers 1.08 or 1.12 and stops, because no run of panels and
# 60 mm posts comes to 1.1 m.
GUARD_EAST = [(0.5, -2.0), (2.0, -2.0), (2.0, 2.0), (-2.0, 2.0), (-2.0, 0.92)]
GUARD_WEST = [(-2.0, 0.30), (-2.0, -2.0), (-0.5, -2.0)]

# Crates on the bottom deck, totes on the one above (linear RGB, as USD).
CRATE, TOTE = (0.527, 0.254, 0.076), (0.028, 0.156, 0.392)


def equip_cell(scene: bt.Scene) -> None:
    """Orders the cell's equipment from the catalog and stands it where the
    USD layer sets the cell out."""
    # A 3.8 m belt, 400 mm wide, its top at 0.55 — the height the taught pick
    # frame was authored against. `conv` is a device as well as a body: the
    # transport zone comes with it, so a sequence only has to start it.
    bt.parts.conveyor(scene, "conv", catalog=CONVEYOR, length=3.8, width=0.4,
                      position=(-0.45, 0.62, 0.55), speed=0.15)

    # A 900 x 450 bay of four decks, stood across the east side. Its stock
    # sits on the frames the generator leaves at the top of each shelf, so
    # moving the rack or taking a level out moves the goods with it.
    rack = bt.parts.rack(scene, "rack", size=(0.9, 0.45, 1.8), position=(1.55, -0.75),
                         catalog=RACK, levels=4, yaw=math.pi / 2)
    for level, size, colour in ((0, (0.28, 0.34, 0.24), CRATE), (1, (0.3, 0.36, 0.22), TOTE)):
        (x, y, z), _ = scene.frame(f"{rack.name}/level{level}")
        for i, across in enumerate((-0.22, 0.22)):
            scene.add_box(f"{rack.name}/stock/l{level}_{i}", size=size,
                          position=(x, y + across, z + size[2] / 2), color=colour)

    # 2 m of mesh guarding, in the two runs the openings leave, with the
    # personnel door on the south wall by the corner — off the walkway,
    # where somebody would actually walk in, and well clear of the gate the
    # vehicle uses. The panel widths are not written here: each edge is
    # filled with the fewest panels the catalog sells that reach the next
    # corner, and the two runs are the same product, so the bill adds them
    # into one line per width.
    bt.parts.fence(scene, "fence/east", path=GUARD_EAST, catalog=FENCE, height=2.0,
                   closed=False)
    bt.parts.fence(scene, "fence/west", path=GUARD_WEST, catalog=FENCE, height=2.0,
                   closed=False, door=(1, 0))


def build_scene(name: str = None) -> bt.Scene:
    """The factory cell with one Franka on the pedestal. `name` sets the
    robot's instance name — worth spelling out when a second arm joins."""
    robot = bt.Robot.from_usd(fetch_franka())
    scene = bt.Scene(robot, name=name)
    scene.load_usd(Path(__file__).parents[1] / "assets" / "factory.usda")
    equip_cell(scene)
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


def teach_grasp(scene: bt.Scene, pose, standoff: float = 0.0, robot: str = None) -> list:
    """Poses the robot so the gripper grasps at ``pose`` and returns the
    joint vector — the scripted form of dragging the studio's TCP gizmo.
    Raises when IK misses, rather than teaching a bad pose. ``robot`` names
    the instance (required once the scene has several)."""
    position, quaternion = hand_pose(pose, standoff)
    ik = scene.set_tcp_target(position, quaternion, link=HAND, robot=robot)
    if not ik.converged:
        raise RuntimeError(
            f"IK did not reach {tuple(round(v, 3) for v in position)}: "
            f"{ik.pos_error * 1e3:.1f} mm / {ik.rot_error:.3f} rad short"
        )
    return list(scene.joint_positions if robot is None else scene.joint_positions_of(robot))


if __name__ == "__main__":
    scene = build_scene()
    print(scene.robot)
    bt.studio(scene)
