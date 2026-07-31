"""Bake a carry motion to USD: the Franka grasps a box on the conveyor and
carries it to the pallet, box riding the gripper and every obstacle in the
scene included. The result plays directly in usdview / Omniverse / Blender
(the robot is referenced from the original Isaac stage at full fidelity).

Run with:  python examples/export_animation.py [out.usda]
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))

from demo import build_scene  # noqa: E402  (path setup first)

BOX = "/World/Conveyor/Box_A"


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("cell_anim.usda")
    scene = build_scene()

    # Snap the TCP onto the pick point and grasp the box, then lift a little
    # so the held box clears the conveyor belt before planning.
    pick_pos, pick_quat = scene.frame("/World/Conveyor/PickFrame")
    scene.set_tcp_target(pick_pos, pick_quat)
    scene.attach(BOX, link="/panda/panda_hand")
    lifted = (pick_pos[0], pick_pos[1], pick_pos[2] + 0.15)
    scene.set_tcp_target(lifted, pick_quat)

    # Teach the drop-off pose above the pallet (top-down orientation keeps
    # the box hanging straight below the gripper, clear of the crates),
    # snapshot the joints, return to the start, and plan the carry.
    place_pos, _ = scene.frame("/World/Pallet/PlaceFrame")
    goal = (place_pos[0], place_pos[1], place_pos[2] + 0.20)
    scene.set_tcp_target(goal, pick_quat)
    goal_q = list(scene.joint_positions)
    scene.set_tcp_target(lifted, pick_quat)
    goal_q[7:] = scene.joint_positions[7:]  # the grip does not change
    traj = scene.plan(goal_q)

    warnings = scene.export_usd(traj, out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported a {traj.duration:.2f}s carry motion to {out}")
    print(f"view it with: usdview {out}")


if __name__ == "__main__":
    main()
