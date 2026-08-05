"""Mimic joints: a joint driven by another one, from URDF and from USD.

The pair costs a single DOF, so everything downstream — planning, FK,
collision, USD export — has to move both joints from one number.
"""

from pathlib import Path

import pytest

import botrail as bt

# A one-axis arm carrying a two-finger gripper. The right finger mirrors
# the left through `<mimic>`, the way real grippers are modelled.
GRIPPER = """
<robot name="mimic_gripper">
  <link name="base"/>
  <link name="palm">
    <visual>
      <geometry><box size="0.08 0.08 0.02"/></geometry>
    </visual>
  </link>
  <link name="left">
    <visual>
      <geometry><box size="0.01 0.01 0.06"/></geometry>
    </visual>
  </link>
  <link name="right">
    <visual>
      <geometry><box size="0.01 0.01 0.06"/></geometry>
    </visual>
  </link>
  <joint name="arm" type="revolute">
    <parent link="base"/><child link="palm"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>
  <joint name="finger_left" type="prismatic">
    <parent link="palm"/><child link="left"/>
    <origin xyz="0 0.02 0.04"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="0.04" effort="10" velocity="0.1"/>
  </joint>
  <joint name="finger_right" type="prismatic">
    <parent link="palm"/><child link="right"/>
    <origin xyz="0 -0.02 0.04"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.04" upper="0" effort="10" velocity="0.1"/>
    <mimic joint="finger_left" multiplier="-1" offset="0"/>
  </joint>
</robot>
"""


@pytest.fixture()
def robot() -> bt.Robot:
    return bt.Robot.from_urdf_string(GRIPPER)


def test_mimic_joint_costs_no_dof(robot: bt.Robot) -> None:
    assert robot.dof == 2
    assert robot.joint_names == ["arm", "finger_left"]
    assert robot.mimic_joints == {"finger_right": ("finger_left", -1.0, 0.0)}

    values = robot.joint_values([0.5, 0.03])
    assert values["finger_left"] == pytest.approx(0.03)
    assert values["finger_right"] == pytest.approx(-0.03)
    assert values["arm"] == pytest.approx(0.5)

    with pytest.raises(ValueError):
        robot.joint_values([0.0])


def test_mimic_finger_moves_with_its_source(robot: bt.Robot) -> None:
    scene = bt.Scene(robot)
    scene.set_joint_positions([0.0, 0.035])
    (_, left_y, _), _ = scene.link_pose("left")
    (_, right_y, _), _ = scene.link_pose("right")
    # Both fingers sit 0.02 out from the palm centre and open symmetrically.
    assert left_y == pytest.approx(0.02 + 0.035)
    assert right_y == pytest.approx(-0.02 - 0.035)


def test_planning_and_usd_export_carry_the_driven_joint(
    robot: bt.Robot, tmp_path: Path
) -> None:
    scene = bt.Scene(robot)
    traj = scene.plan([0.6, 0.03])
    assert len(traj.positions[0]) == 2

    out = tmp_path / "grip.usda"
    assert scene.export_usd(traj, out, fps=30.0) == []
    # URDF robots export as baked link transforms: the mirrored finger is
    # visible as its own animated prim.
    text = out.read_text()
    assert "right" in text and "left" in text


# A USD articulation of the same gripper: `finger_right` follows
# `finger_left` through PhysX's mimic joint API (qA + G*qB + gamma = 0).
USD_GRIPPER = """#usda 1.0
(
    defaultPrim = "Gripper"
    metersPerUnit = 1
    upAxis = "Z"
)

def Xform "Gripper" (prepend apiSchemas = ["PhysicsArticulationRootAPI"])
{
    def Xform "palm" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom" { double size = 0.08 }
    }

    def Xform "left" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom" { double size = 0.02 }
    }

    def Xform "right" (prepend apiSchemas = ["PhysicsRigidBodyAPI"])
    {
        def Cube "geom" { double size = 0.02 }
    }

    def Scope "joints"
    {
        def PhysicsFixedJoint "anchor"
        {
            rel physics:body1 = </Gripper/palm>
        }

        def PhysicsPrismaticJoint "finger_left"
        {
            rel physics:body0 = </Gripper/palm>
            rel physics:body1 = </Gripper/left>
            uniform token physics:axis = "Y"
            float physics:lowerLimit = 0
            float physics:upperLimit = 0.04
        }

        def PhysicsPrismaticJoint "finger_right" (
            prepend apiSchemas = ["PhysxMimicJointAPI:transY"]
        )
        {
            rel physics:body0 = </Gripper/palm>
            rel physics:body1 = </Gripper/right>
            uniform token physics:axis = "Y"
            float physics:lowerLimit = -0.04
            float physics:upperLimit = 0
            float physxMimicJoint:transY:gearing = 1
            float physxMimicJoint:transY:offset = 0
            rel physxMimicJoint:transY:referenceJoint = </Gripper/joints/finger_left>
            uniform token physxMimicJoint:transY:referenceJointAxis = "transY"
        }
    }
}
"""


def test_usd_mimic_joint_imports_as_a_coupled_dof(tmp_path: Path) -> None:
    path = tmp_path / "gripper.usda"
    path.write_text(USD_GRIPPER)
    robot = bt.Robot.from_usd(path)

    assert robot.dof == 1
    assert robot.joint_names == ["/Gripper/joints/finger_left"]
    assert robot.mimic_joints == {
        "/Gripper/joints/finger_right": ("/Gripper/joints/finger_left", -1.0, 0.0)
    }

    scene = bt.Scene(robot)
    scene.set_joint_positions([0.03])
    (_, left_y, _), _ = scene.link_pose("/Gripper/left")
    (_, right_y, _), _ = scene.link_pose("/Gripper/right")
    assert left_y == pytest.approx(0.03)
    assert right_y == pytest.approx(-0.03)


# The same gripper as URDF-to-USD converters author it: no PhysX schema,
# the URDF `<mimic>` relation carried as `botrail:mimic` customData
# (serialized by pxr as a nested namespace dictionary).
USD_GRIPPER_CUSTOM_DATA = USD_GRIPPER.replace(
    """        def PhysicsPrismaticJoint "finger_right" (
            prepend apiSchemas = ["PhysxMimicJointAPI:transY"]
        )
        {
            rel physics:body0 = </Gripper/palm>
            rel physics:body1 = </Gripper/right>
            uniform token physics:axis = "Y"
            float physics:lowerLimit = -0.04
            float physics:upperLimit = 0
            float physxMimicJoint:transY:gearing = 1
            float physxMimicJoint:transY:offset = 0
            rel physxMimicJoint:transY:referenceJoint = </Gripper/joints/finger_left>
            uniform token physxMimicJoint:transY:referenceJointAxis = "transY"
        }""",
    """        def PhysicsPrismaticJoint "finger_right" (
            customData = {
                dictionary botrail = {
                    dictionary mimic = {
                        string joint = "finger_left"
                        double multiplier = -1
                        double offset = 0
                    }
                }
            }
        )
        {
            rel physics:body0 = </Gripper/palm>
            rel physics:body1 = </Gripper/right>
            uniform token physics:axis = "Y"
            float physics:lowerLimit = -0.04
            float physics:upperLimit = 0
        }""",
)


def test_usd_custom_data_mimic_matches_the_urdf_path(tmp_path: Path) -> None:
    assert USD_GRIPPER_CUSTOM_DATA != USD_GRIPPER  # the replace applied
    path = tmp_path / "gripper.usda"
    path.write_text(USD_GRIPPER_CUSTOM_DATA)
    robot = bt.Robot.from_usd(path)

    assert robot.dof == 1
    assert robot.joint_names == ["/Gripper/joints/finger_left"]
    assert robot.mimic_joints == {
        "/Gripper/joints/finger_right": ("/Gripper/joints/finger_left", -1.0, 0.0)
    }

    scene = bt.Scene(robot)
    scene.set_joint_positions([0.03])
    (_, left_y, _), _ = scene.link_pose("/Gripper/left")
    (_, right_y, _), _ = scene.link_pose("/Gripper/right")
    assert left_y == pytest.approx(0.03)
    assert right_y == pytest.approx(-0.03)
