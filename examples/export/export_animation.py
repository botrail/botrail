"""Bake a carry motion to USD: the Franka grasps a box on the conveyor and
carries it to the pallet, box riding the gripper and every obstacle in the
scene included. The result plays directly in usdview / Omniverse / Blender
(the robot is referenced from the original Isaac stage at full fidelity).

Run with:  python examples/export/export_animation.py [out.usda]
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "basics"))

from demo import build_scene, teach_grasp  # noqa: E402  (path setup first)

BOX = "/World/Conveyor/Box_A"
CLOSED = 0.029  # a millimetre a side into the 60 mm box
TOUCH = ["/panda/panda_leftfinger", "/panda/panda_rightfinger"]


def main() -> None:
    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("cell_anim.usda")
    scene = build_scene()

    # Put the gripper on the cell's taught pick pose (tool pointing down,
    # pads around the box), close on the box and grasp it, then lift a
    # little so the held box clears the conveyor belt before planning.
    pick = scene.frame("/World/Conveyor/PickFrame")
    place = scene.frame("/World/Pallet/PlaceFrame")
    home_q = list(scene.joint_positions)
    grip = teach_grasp(scene, pick)
    grip[7:] = [CLOSED] * len(grip[7:])
    scene.set_joint_positions(grip)
    scene.attach(BOX, link="/panda/panda_hand", touch_links=TOUCH)
    lifted_q = teach_grasp(scene, pick, standoff=0.15)

    # Teach the drop-off pose above the pallet (the taught place pose, held
    # high so the box clears the crates), snapshot the joints, return to the
    # start, and plan the carry. The pallet is a 150 deg base swing from the
    # conveyor, so that solve restarts from the ready pose — warm-starting it
    # from the pick side walks the solver into a local minimum.
    scene.set_joint_positions(home_q)
    goal_q = teach_grasp(scene, place, standoff=0.20)
    goal_q[7:] = lifted_q[7:]  # the grip does not change
    scene.set_joint_positions(lifted_q)
    traj = scene.plan(goal_q)

    warnings = scene.export_usd(traj, out, fps=60.0)
    for w in warnings:
        print(f"warning: {w}")
    print(f"exported a {traj.duration:.2f}s carry motion to {out}")
    print(f"view it with: usdview {out}")


if __name__ == "__main__":
    main()
