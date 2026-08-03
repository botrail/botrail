//! Kinematics over a [`botrail_model::RobotModel`]: forward kinematics,
//! geometric Jacobian, and damped-least-squares inverse kinematics.

mod ik;

pub use ik::{jacobian, solve_ik, IkMode, IkOptions, IkResult};

use botrail_model::{JointType, RobotModel};
use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum KinError {
    #[error("expected {expected} joint positions, got {got}")]
    WrongDof { expected: usize, got: usize },
}

/// World transform of every link, indexed like `model.links`.
/// The root link is placed at the identity.
pub fn forward_kinematics(model: &RobotModel, q: &[f64]) -> Result<Vec<Isometry3<f64>>, KinError> {
    forward_kinematics_with_base(model, q, &Isometry3::identity())
}

/// World transform of every link with the root link placed at `base`.
pub fn forward_kinematics_with_base(
    model: &RobotModel,
    q: &[f64],
    base: &Isometry3<f64>,
) -> Result<Vec<Isometry3<f64>>, KinError> {
    if q.len() != model.dof() {
        return Err(KinError::WrongDof {
            expected: model.dof(),
            got: q.len(),
        });
    }
    let mut poses = vec![Isometry3::identity(); model.links.len()];
    poses[model.root_link] = *base;
    for &ji in &model.joint_order {
        let joint = &model.joints[ji];
        // Mimic joints have no entry in `q`; the model derives their value
        // from the joint that drives them.
        let value = model.joint_value(ji, q);
        let motion = match joint.joint_type {
            JointType::Revolute | JointType::Continuous => Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_axis_angle(&joint.axis, value),
            ),
            JointType::Prismatic => Translation3::from(joint.axis.into_inner() * value).into(),
            JointType::Fixed => Isometry3::identity(),
        };
        poses[joint.child_link] = poses[joint.parent_link] * joint.origin * motion;
    }
    Ok(poses)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    /// Planar 2R arm with unit link lengths, plus a fixed tool tip.
    const PLANAR_2R: &str = r#"
    <robot name="planar_2r">
      <link name="base_link"/>
      <link name="link1"/>
      <link name="link2"/>
      <link name="tool"/>
      <joint name="q1" type="revolute">
        <parent link="base_link"/><child link="link1"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
      </joint>
      <joint name="q2" type="revolute">
        <parent link="link1"/><child link="link2"/>
        <origin xyz="1 0 0"/>
        <axis xyz="0 0 1"/>
        <limit lower="-3.14" upper="3.14" effort="1" velocity="1"/>
      </joint>
      <joint name="tip" type="fixed">
        <parent link="link2"/><child link="tool"/>
        <origin xyz="1 0 0"/>
      </joint>
    </robot>
    "#;

    fn tool_position(q: &[f64]) -> (f64, f64, f64) {
        let model = RobotModel::from_urdf_str(PLANAR_2R).unwrap();
        let poses = forward_kinematics(&model, q).unwrap();
        let t = poses[model.link_index("tool").unwrap()].translation;
        (t.x, t.y, t.z)
    }

    fn assert_close(actual: (f64, f64, f64), expected: (f64, f64, f64)) {
        assert!(
            (actual.0 - expected.0).abs() < 1e-12
                && (actual.1 - expected.1).abs() < 1e-12
                && (actual.2 - expected.2).abs() < 1e-12,
            "expected {expected:?}, got {actual:?}"
        );
    }

    #[test]
    fn planar_arm_matches_closed_form() {
        // x = cos(q1) + cos(q1+q2), y = sin(q1) + sin(q1+q2)
        assert_close(tool_position(&[0.0, 0.0]), (2.0, 0.0, 0.0));
        assert_close(tool_position(&[FRAC_PI_2, 0.0]), (0.0, 2.0, 0.0));
        assert_close(tool_position(&[FRAC_PI_2, FRAC_PI_2]), (-1.0, 1.0, 0.0));
    }

    #[test]
    fn prismatic_joint_translates_along_axis() {
        let urdf = r#"
        <robot name="slider">
          <link name="base"/><link name="carriage"/>
          <joint name="slide" type="prismatic">
            <parent link="base"/><child link="carriage"/>
            <axis xyz="0 1 0"/>
            <limit lower="0" upper="0.5" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        let poses = forward_kinematics(&model, &[0.3]).unwrap();
        let t = poses[model.link_index("carriage").unwrap()].translation;
        assert!((t.y - 0.3).abs() < 1e-12);
    }

    #[test]
    fn mimic_joint_follows_its_source() {
        // A gripper whose right finger mirrors the left: one DOF closes both.
        let urdf = r#"
        <robot name="gripper">
          <link name="palm"/><link name="left"/><link name="right"/>
          <joint name="finger_left" type="prismatic">
            <parent link="palm"/><child link="left"/>
            <axis xyz="0 1 0"/>
            <limit lower="0" upper="0.04" effort="1" velocity="1"/>
          </joint>
          <joint name="finger_right" type="prismatic">
            <parent link="palm"/><child link="right"/>
            <axis xyz="0 1 0"/>
            <limit lower="-0.04" upper="0" effort="1" velocity="1"/>
            <mimic joint="finger_left" multiplier="-1"/>
          </joint>
        </robot>"#;
        let model = RobotModel::from_urdf_str(urdf).unwrap();
        assert_eq!(model.dof(), 1);
        let poses = forward_kinematics(&model, &[0.03]).unwrap();
        let y = |link: &str| poses[model.link_index(link).unwrap()].translation.y;
        assert!((y("left") - 0.03).abs() < 1e-12);
        assert!((y("right") + 0.03).abs() < 1e-12, "{}", y("right"));
    }

    #[test]
    fn wrong_dof_is_rejected() {
        let model = RobotModel::from_urdf_str(PLANAR_2R).unwrap();
        assert!(matches!(
            forward_kinematics(&model, &[0.0]),
            Err(KinError::WrongDof {
                expected: 2,
                got: 1
            })
        ));
    }
}
