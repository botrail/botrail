//! Hand-authored URDF fixtures for this crate's module tests.
//!
//! These live in the crate rather than under `examples/` because no demo
//! loads them any more — the machining and painting demos order their tools
//! from the catalog. Keeping the tests' own tool here makes them
//! self-contained (no example asset can be retired out from under them) and
//! keeps `examples/` to what a reader is meant to run.

/// A router spindle: motor body, collet, and an 8 mm end mill extending out
/// of the flange (`+Z` of the mount, as tools do). The `tip` link is the
/// TCP; its frame is flipped a half-turn about X so its `+Z` runs from the
/// cutter tip back toward the tool body — the axis convention
/// `PathTarget::tool_axis` and the 5-DOF axis-aligned IK
/// expect. Collision is authored as primitives.
///
/// The paint tests use it too: what they need from a tool is a TCP standing
/// off the flange with that axis convention, which this already is.
pub(crate) const SPINDLE_URDF: &str = r#"<?xml version="1.0"?>
<robot name="spindle">
  <link name="spindle_mount">
    <visual>
      <origin xyz="0 0 0.03"/>
      <geometry><cylinder radius="0.024" length="0.06"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.03"/>
      <geometry><cylinder radius="0.024" length="0.06"/></geometry>
    </collision>
  </link>
  <link name="collet">
    <visual>
      <origin xyz="0 0 0.01"/>
      <geometry><cylinder radius="0.009" length="0.02"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.01"/>
      <geometry><cylinder radius="0.009" length="0.02"/></geometry>
    </collision>
  </link>
  <link name="cutter">
    <visual>
      <origin xyz="0 0 0.015"/>
      <geometry><cylinder radius="0.004" length="0.03"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.015"/>
      <geometry><cylinder radius="0.004" length="0.03"/></geometry>
    </collision>
  </link>
  <link name="tip"/>
  <joint name="mount_to_collet" type="fixed">
    <parent link="spindle_mount"/><child link="collet"/>
    <origin xyz="0 0 0.06"/>
  </joint>
  <joint name="collet_to_cutter" type="fixed">
    <parent link="collet"/><child link="cutter"/>
    <origin xyz="0 0 0.02"/>
  </joint>
  <joint name="cutter_to_tip" type="fixed">
    <parent link="cutter"/><child link="tip"/>
    <origin xyz="0 0 0.03" rpy="3.14159265358979 0 0"/>
  </joint>
</robot>"#;

/// A coupled two-finger gripper for the grasp tests: one prismatic drive
/// on the left finger, the right finger mimicking it mirrored, and a
/// `grasp_center` TCP frame between the pads. At `q = 0` the pad inner
/// faces stand 30 mm out from the centre plane each (60 mm opening); a
/// drive value `q` narrows the gap to `0.06 - 2q`, so a 40 mm part is
/// touched at exactly `q = 0.010`. Collision is authored as primitives.
pub(crate) const GRIPPER_URDF: &str = r#"<?xml version="1.0"?>
<robot name="grip2f">
  <link name="palm">
    <visual>
      <origin xyz="0 0 0.02"/>
      <geometry><box size="0.08 0.06 0.04"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.02"/>
      <geometry><box size="0.08 0.06 0.04"/></geometry>
    </collision>
  </link>
  <link name="finger_l">
    <visual>
      <origin xyz="0 0 0.03"/>
      <geometry><box size="0.01 0.02 0.06"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.03"/>
      <geometry><box size="0.01 0.02 0.06"/></geometry>
    </collision>
  </link>
  <link name="finger_r">
    <visual>
      <origin xyz="0 0 0.03"/>
      <geometry><box size="0.01 0.02 0.06"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0.03"/>
      <geometry><box size="0.01 0.02 0.06"/></geometry>
    </collision>
  </link>
  <link name="grasp_center"/>
  <joint name="drive" type="prismatic">
    <parent link="palm"/><child link="finger_l"/>
    <origin xyz="-0.035 0 0.04"/>
    <axis xyz="1 0 0"/>
    <limit lower="0" upper="0.028" effort="10" velocity="0.1"/>
  </joint>
  <joint name="follow" type="prismatic">
    <parent link="palm"/><child link="finger_r"/>
    <origin xyz="0.035 0 0.04"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.028" upper="0" effort="10" velocity="0.1"/>
    <mimic joint="drive" multiplier="-1" offset="0"/>
  </joint>
  <joint name="palm_to_tcp" type="fixed">
    <parent link="palm"/><child link="grasp_center"/>
    <origin xyz="0 0 0.07"/>
  </joint>
</robot>"#;
