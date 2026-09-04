"""`bt.tools.multi_tool` — a bracket carrying several tools, each with its
own tip frame, welded on with `Robot.attach_tool`.

What these pin: the tips land where the tools were placed and aim along
them, the composite keeps the gripper's TCP while the pin's and the
fork's tips stay addressable, IK asked for a tip leaves the gripper's
fingers alone, the tools are real collision geometry, and a hand that
cannot be built is refused with the reason."""

from pathlib import Path

import botrail as bt
import pytest

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

# The one-DOF mimic gripper from the attach_tool tests: a mount plate,
# two fingers, a grasp-centre TCP 12 cm out.
GRIPPER = """
<robot name="gripper">
  <link name="mount_plate">
    <visual><geometry><box size="0.06 0.06 0.02"/></geometry></visual>
    <collision><geometry><box size="0.06 0.06 0.02"/></geometry></collision>
  </link>
  <link name="finger_l">
    <visual><origin xyz="0 0.02 0.05"/><geometry><box size="0.01 0.01 0.06"/></geometry></visual>
    <collision><origin xyz="0 0.02 0.05"/><geometry><box size="0.01 0.01 0.06"/></geometry></collision>
  </link>
  <link name="finger_r">
    <visual><origin xyz="0 -0.02 0.05"/><geometry><box size="0.01 0.01 0.06"/></geometry></visual>
    <collision><origin xyz="0 -0.02 0.05"/><geometry><box size="0.01 0.01 0.06"/></geometry></collision>
  </link>
  <link name="grasp_center"/>
  <joint name="drive" type="prismatic">
    <parent link="mount_plate"/><child link="finger_l"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.04" effort="10" velocity="0.1"/>
  </joint>
  <joint name="follow" type="prismatic">
    <parent link="mount_plate"/><child link="finger_r"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.04" upper="0" effort="10" velocity="0.1"/>
    <mimic joint="drive" multiplier="-1" offset="0"/>
  </joint>
  <joint name="tcp_joint" type="fixed">
    <parent link="mount_plate"/><child link="grasp_center"/>
    <origin xyz="0 0 0.12"/>
  </joint>
</robot>
"""


def hand() -> bt.Robot:
    # A 60 mm plate: the test arm's tool link is a 40 mm box past its
    # flange, and the gripper must bolt on beyond it.
    bracket = bt.tools.multi_tool(
        "hand", [bt.tools.Mount("gripper", at=(0.0, 0.0, 0.06)), bt.tools.Pin("pusher"), bt.tools.Fork("fork")],
        plate=(0.08, 0.06),
    )
    return bracket.attach_tool(bt.Robot.from_urdf_string(GRIPPER), flange="hand_gripper", mount="mount_plate",
                               tcp="grasp_center")


def test_the_tips_sit_where_the_tools_were_placed_and_aim_along_them() -> None:
    arm = bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")
    robot = arm.attach_tool(hand(), flange=arm.tcp_link)
    # The composite: the arm's joints plus the gripper's; the gripper's
    # TCP; every tip still by name.
    assert robot.joint_names == arm.joint_names + ["drive"]
    assert robot.tcp_link == "grasp_center"
    assert {bt.tools.tip("hand", "pusher"), bt.tools.tip("hand", "fork"), "hand_plate", "hand_fork_prong_l"} <= set(robot.link_names)
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.0] * robot.dof)
    flange, _fq = scene.link_pose(arm.tcp_link)

    def rel(link: str):
        p, q = scene.link_pose(link)
        return tuple(round(a - b, 6) for a, b in zip(p, flange)), tuple(round(v, 6) for v in bt.parts._rotate(q, (0.0, 0.0, 1.0)))

    # At zero the simple arm's flange is +Z up, so the plate frame is the
    # world's: the pin runs +X from its foot, the fork -X, the gripper +Z.
    assert rel("hand_pusher_tip") == ((0.1, 0.0, 0.022), (1.0, 0.0, 0.0))
    assert rel("hand_fork_tip") == ((-0.09, 0.0, 0.022), (-1.0, 0.0, 0.0))
    assert rel("grasp_center") == ((0.0, 0.0, 0.18), (0.0, 0.0, 1.0))
    assert scene.check_collisions() == []


def test_ik_for_a_tip_moves_the_arm_and_not_the_fingers() -> None:
    arm = bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")
    scene = bt.Scene(arm.attach_tool(hand(), flange=arm.tcp_link))
    scene.set_joint_positions([0.2, -0.6, 0.9, 0.3, 0.5, 0.0, 0.03])
    (x, y, z), _ = scene.link_pose("hand_pusher_tip")
    target = (x + 0.03, y - 0.02, z + 0.02)
    ik = scene.set_tcp_target(target, link="hand_pusher_tip")
    assert ik.converged
    assert scene.link_pose("hand_pusher_tip")[0] == pytest.approx(target, abs=1e-4)
    assert scene.joint_positions[-1] == pytest.approx(0.03)   # the fingers stayed where they were


def test_the_tools_collide_and_a_bad_hand_is_refused() -> None:
    arm = bt.Robot.from_urdf(EXAMPLES / "assets" / "simple_arm.urdf")
    scene = bt.Scene(arm.attach_tool(hand(), flange=arm.tcp_link))
    scene.set_joint_positions([0.0] * 7)
    # A plate in the pin's way is hit by the pin — the tools are collision
    # geometry, not drawings.
    (x, y, z), _ = scene.link_pose("hand_pusher_tip")
    scene.add_box("wall", size=(0.01, 0.2, 0.2), position=(x + 0.0, y, z))
    hits = {(a[1], b[1]) for a, b in scene.check_collisions()}
    assert ("hand_pusher", "wall") in hits
    with pytest.raises(ValueError, match="distinct"):
        bt.tools.multi_tool("h", [bt.tools.Pin("a"), bt.tools.Pin("a")])
    with pytest.raises(ValueError, match="inside the plate"):
        bt.tools.multi_tool("h", [bt.tools.Mount("g", at=(0.0, 0.0, 0.0))])
    with pytest.raises(ValueError, match="seat within the reach"):
        bt.tools.multi_tool("h", [bt.tools.Fork("f", reach=0.05, seat=0.08)])
    with pytest.raises(TypeError):
        bt.tools.multi_tool("h", ["pusher"])  # type: ignore[list-item]
