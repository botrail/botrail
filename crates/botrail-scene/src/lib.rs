//! Scene state (robot + joint configuration) and the JSON wire protocol
//! spoken between the botrail server and the studio UI.
//!
//! The Rust types in [`wire`] are the single source of truth for the
//! protocol. The TypeScript side (`studio/src/generated/`) is generated from
//! them via ts-rs — run `scripts/gen_protocol.sh` after changing them.

pub mod wire;

use std::sync::Arc;

use botrail_model::RobotModel;
use nalgebra::Isometry3;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SceneError {
    #[error("expected {expected} joint positions, got {got}")]
    WrongDof { expected: usize, got: usize },
}

/// A robot in a workspace. Obstacles and attached objects arrive in M2.
#[derive(Debug, Clone)]
pub struct Scene {
    pub robot: Arc<RobotModel>,
    joint_positions: Vec<f64>,
}

impl Scene {
    pub fn new(robot: Arc<RobotModel>) -> Self {
        let joint_positions = robot.neutral_positions();
        Self {
            robot,
            joint_positions,
        }
    }

    pub fn joint_positions(&self) -> &[f64] {
        &self.joint_positions
    }

    pub fn set_joint_positions(&mut self, positions: Vec<f64>) -> Result<(), SceneError> {
        if positions.len() != self.robot.dof() {
            return Err(SceneError::WrongDof {
                expected: self.robot.dof(),
                got: positions.len(),
            });
        }
        self.joint_positions = positions;
        Ok(())
    }

    /// World pose of every link at the current configuration.
    pub fn link_poses(&self) -> Vec<Isometry3<f64>> {
        botrail_kin::forward_kinematics(&self.robot, &self.joint_positions)
            .expect("joint_positions length is enforced by set_joint_positions")
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_scene() -> Scene {
        let urdf = r#"
        <robot name="r">
          <link name="a"/><link name="b"/>
          <joint name="j" type="revolute">
            <parent link="a"/><child link="b"/>
            <origin xyz="0 0 0.5"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        Scene::new(Arc::new(RobotModel::from_urdf_str(urdf).unwrap()))
    }

    #[test]
    fn starts_at_neutral_and_validates_dof() {
        let mut scene = sample_scene();
        assert_eq!(scene.joint_positions(), &[0.0]);
        assert!(scene.set_joint_positions(vec![0.5]).is_ok());
        assert!(matches!(
            scene.set_joint_positions(vec![0.1, 0.2]),
            Err(SceneError::WrongDof { .. })
        ));
    }

    #[test]
    fn link_poses_follow_configuration() {
        let scene = sample_scene();
        let poses = scene.link_poses();
        assert!((poses[1].translation.z - 0.5).abs() < 1e-12);
    }
}
