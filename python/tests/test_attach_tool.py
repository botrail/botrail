"""Robot.attach_tool: welding an end-effector onto a flange.

The composite is one kinematic tree — the tool's joints (mimic included)
join the DOF vector, the declared TCP replaces the deepest-leaf heuristic,
and the weld persists through projects and generated scripts.
"""

import math
from pathlib import Path

import pytest

import botrail as bt

EXAMPLES = Path(__file__).resolve().parents[2] / "examples"

# A one-DOF mimic gripper rooted at its mounting plate, with an explicit
# grasp-center TCP frame 12 cm out along +Z.
GRIPPER = """
<robot name="gripper">
  <link name="mount_plate">
    <visual><geometry><box size="0.06 0.06 0.02"/></geometry></visual>
  </link>
  <link name="finger_l">
    <visual><origin xyz="0 0.02 0.05"/><geometry><box size="0.01 0.01 0.06"/></geometry></visual>
  </link>
  <link name="finger_r">
    <visual><origin xyz="0 -0.02 0.05"/><geometry><box size="0.01 0.01 0.06"/></geometry></visual>
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


@pytest.fixture()
def arm() -> bt.Robot:
    return bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf")


@pytest.fixture()
def gripper() -> bt.Robot:
    return bt.Robot.from_urdf_string(GRIPPER)


def test_attach_tool_composes_kinematics(arm: bt.Robot, gripper: bt.Robot) -> None:
    robot = arm.attach_tool(
        gripper,
        flange=arm.tcp_link,
        mount="mount_plate",
        offset_position=(0, 0, 0.0139),
        tcp="grasp_center",
    )
    assert robot.dof == arm.dof + 1
    assert robot.joint_names == arm.joint_names + ["drive"]
    assert robot.mimic_joints == {"follow": ("drive", -1.0, 0.0)}
    assert set(gripper.link_names) <= set(robot.link_names)
    # The declared TCP, not a fingertip.
    assert robot.tcp_link == "grasp_center"
    # The inputs are untouched (attach returns a new robot).
    assert arm.dof == 6
    assert gripper.dof == 1


def test_attach_offset_and_tcp_land_where_declared(arm: bt.Robot, gripper: bt.Robot) -> None:
    flange = arm.tcp_link
    robot = arm.attach_tool(
        gripper,
        flange=flange,
        mount="mount_plate",
        offset_position=(0, 0, 0.0139),
        tcp="grasp_center",
    )
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.0] * robot.dof)
    (fx, fy, fz), _ = scene.link_pose(flange)
    (mx, my, mz), _ = scene.link_pose("mount_plate")
    (tx, ty, tz), _ = scene.link_pose("grasp_center")
    # At the zero configuration the flange frame of simple_arm points up,
    # so the offsets stack along world Z.
    assert (mx, my) == pytest.approx((fx, fy))
    assert mz - fz == pytest.approx(0.0139)
    assert (tx, ty) == pytest.approx((fx, fy))
    assert tz - mz == pytest.approx(0.12)


def test_ik_targets_the_declared_tcp(arm: bt.Robot, gripper: bt.Robot) -> None:
    robot = arm.attach_tool(
        gripper, flange=arm.tcp_link, mount="mount_plate", tcp="grasp_center"
    )
    scene = bt.Scene(robot)
    q = [0.4, -0.6, 0.8, 0.2, 0.5, 0.1] + [0.02]
    scene.set_joint_positions(q)
    target, target_quat = scene.link_pose("grasp_center")
    result = robot.ik(target, target_quat)  # link defaults to the TCP
    assert result.converged
    scene.set_joint_positions(result.q)
    reached, _ = scene.link_pose("grasp_center")
    assert math.dist(reached, target) < 1e-3


def test_mimic_fingers_follow_in_the_composite(arm: bt.Robot, gripper: bt.Robot) -> None:
    # Mounted with clearance so the fingers do not brush the wrist geometry.
    robot = arm.attach_tool(
        gripper, flange=arm.tcp_link, mount="mount_plate", offset_position=(0, 0, 0.03)
    )
    scene = bt.Scene(robot)
    values = robot.joint_values([0.0] * 6 + [0.03])
    assert values["drive"] == pytest.approx(0.03)
    assert values["follow"] == pytest.approx(-0.03)
    # The composite plans out of the box, gripper DOF included.
    traj = scene.plan([0.3, -0.4, 0.5, 0.0, 0.2, 0.0, 0.02], broadcast=False)
    assert len(traj.positions[0]) == 7


def test_weld_pair_is_exempt_like_urdf_adjacency(arm: bt.Robot, gripper: bt.Robot) -> None:
    # Mounted flush, the fingers legitimately brush the wrist — but the
    # welded pair itself (flange link vs mount plate) must be exempt from
    # self-collision, exactly like URDF-adjacent links.
    robot = arm.attach_tool(gripper, flange=arm.tcp_link, mount="mount_plate")
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.0] * 7)
    pairs = {frozenset((a[1], b[1])) for a, b in scene.check_collisions()}
    assert frozenset((arm.tcp_link, "mount_plate")) not in pairs
    assert frozenset((arm.tcp_link, "finger_l")) in pairs  # the real contact


def test_name_collision_needs_a_prefix(arm: bt.Robot) -> None:
    twin = bt.Robot.from_urdf(EXAMPLES / "simple_arm.urdf")
    with pytest.raises(ValueError, match="prefix"):
        arm.attach_tool(twin, flange=arm.tcp_link, mount="base_link")
    robot = arm.attach_tool(twin, flange=arm.tcp_link, mount="base_link", prefix="t_")
    assert robot.dof == 12
    assert "t_base_link" in robot.link_names


def test_mount_must_be_the_tool_root(arm: bt.Robot, gripper: bt.Robot) -> None:
    with pytest.raises(ValueError, match="root"):
        arm.attach_tool(gripper, flange=arm.tcp_link, mount="finger_l")
    with pytest.raises(ValueError):
        arm.attach_tool(gripper, flange="nope", mount="mount_plate")
    with pytest.raises(ValueError):
        arm.attach_tool(gripper, flange=arm.tcp_link, mount="mount_plate", tcp="nope")


def test_project_and_script_carry_the_attachment(
    arm: bt.Robot, gripper: bt.Robot, tmp_path: Path
) -> None:
    robot = arm.attach_tool(
        gripper,
        flange=arm.tcp_link,
        mount="mount_plate",
        offset_position=(0, 0, 0.0139),
        tcp="grasp_center",
    )
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.1, -0.2, 0.3, 0.0, 0.1, 0.0, 0.01])
    project = tmp_path / "cell.botrail"
    scene.save_project(project)

    reloaded = bt.Scene.load_project(project)
    assert reloaded.robot.dof == 7
    assert reloaded.robot.tcp_link == "grasp_center"
    assert reloaded.robot.mimic_joints == {"follow": ("drive", -1.0, 0.0)}
    assert reloaded.joint_positions == pytest.approx([0.1, -0.2, 0.3, 0.0, 0.1, 0.0, 0.01])

    code = reloaded.generate_python()
    assert ".attach_tool(robot_tool," in code
    assert 'tcp="grasp_center"' in code
    # The generated script rebuilds the same composite. Drop the trailing
    # studio launch — running it is exactly what the script is for, but a
    # test must not open a server and a browser.
    headless = "\n".join(l for l in code.splitlines() if l != "bt.studio(scene)")
    namespace: dict = {}
    exec(headless, namespace)
    rebuilt = namespace["scene"]
    assert rebuilt.robot.dof == 7
    assert rebuilt.robot.tcp_link == "grasp_center"
