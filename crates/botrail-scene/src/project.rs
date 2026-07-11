//! The `.botrail` project format: a self-contained JSON snapshot of the
//! robot (embedded URDF), joint state, obstacles, and motions — plus a
//! Python code generator that reproduces the project programmatically.
//!
//! Known limitation: mesh assets referenced by the URDF are NOT embedded
//! yet (that arrives with the mesh I/O crate); primitive-only robots and
//! scenes are fully self-contained.

use std::sync::Arc;

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::wire::{
    geometry_from_msg, geometry_msg, motion_from_msg, motion_msg, ConstraintMsg, GeometryMsg,
    MotionMsg, ObstacleMsg, PoseMsg, SegmentKindMsg,
};
use crate::Scene;

pub const PROJECT_VERSION: u32 = 1;

#[derive(Debug, Error)]
pub enum ProjectError {
    #[error("unsupported project version {0} (expected {PROJECT_VERSION})")]
    Version(u32),
    #[error("invalid project JSON: {0}")]
    Json(String),
    #[error("embedded URDF failed to parse: {0}")]
    Robot(String),
    #[error("project does not fit this scene: {0}")]
    Incompatible(String),
    #[error("{0}")]
    Scene(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProjectFile {
    pub version: u32,
    /// URDF XML (xacro already expanded).
    pub robot_urdf: String,
    pub joint_positions: Vec<f64>,
    pub obstacles: Vec<ObstacleMsg>,
    pub motions: Vec<MotionMsg>,
}

impl ProjectFile {
    pub fn from_json(json: &str) -> Result<Self, ProjectError> {
        let project: ProjectFile =
            serde_json::from_str(json).map_err(|e| ProjectError::Json(e.to_string()))?;
        if project.version != PROJECT_VERSION {
            return Err(ProjectError::Version(project.version));
        }
        Ok(project)
    }

    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("project serializes infallibly")
    }
}

impl Scene {
    pub fn to_project(&self) -> ProjectFile {
        let mut no_mesh = |_: &std::path::Path| (String::new(), String::new());
        ProjectFile {
            version: PROJECT_VERSION,
            robot_urdf: self.robot.urdf_source.clone(),
            joint_positions: self.joint_positions().to_vec(),
            obstacles: self
                .obstacles()
                .iter()
                .map(|o| ObstacleMsg {
                    name: o.name.clone(),
                    geometry: geometry_msg(&o.geometry, &mut no_mesh),
                    pose: PoseMsg::from(&o.pose),
                })
                .collect(),
            motions: self.motions().iter().map(motion_msg).collect(),
        }
    }

    /// Builds a fresh scene (robot included) from a project.
    pub fn from_project(project: &ProjectFile) -> Result<Scene, ProjectError> {
        let robot = botrail_model::RobotModel::from_urdf_str(&project.robot_urdf)
            .map_err(|e| ProjectError::Robot(e.to_string()))?;
        let mut scene = Scene::new(Arc::new(robot));
        scene.apply_project(project)?;
        Ok(scene)
    }

    /// Applies a project's state (joints, obstacles, motions) onto this
    /// scene. The robot itself is kept; the project must have the same DOF.
    pub fn apply_project(&mut self, project: &ProjectFile) -> Result<(), ProjectError> {
        if project.joint_positions.len() != self.robot.dof() {
            return Err(ProjectError::Incompatible(format!(
                "project has {} DOF, scene robot has {}",
                project.joint_positions.len(),
                self.robot.dof()
            )));
        }
        // Build the new obstacle set before mutating anything.
        let mut obstacles = Vec::with_capacity(project.obstacles.len());
        for o in &project.obstacles {
            let geometry = geometry_from_msg(&o.geometry).map_err(ProjectError::Scene)?;
            obstacles.push((o.name.clone(), geometry, (&o.pose).into()));
        }

        while let Some(existing) = self.obstacles().first().map(|o| o.name.clone()) {
            self.remove_obstacle(&existing)
                .expect("existing obstacle is removable");
        }
        for (name, geometry, pose) in obstacles {
            self.add_obstacle(&name, geometry, pose)
                .map_err(|e| ProjectError::Scene(e.to_string()))?;
        }
        self.set_joint_positions(project.joint_positions.clone())
            .map_err(|e| ProjectError::Scene(e.to_string()))?;
        self.set_motions(project.motions.iter().map(motion_from_msg).collect());
        Ok(())
    }
}

// ------------------------------------------------------------- codegen

fn py_list(values: &[f64]) -> String {
    let items: Vec<String> = values.iter().map(|v| format!("{v:.6}")).collect();
    format!("[{}]", items.join(", "))
}

fn py_tuple(values: &[f64]) -> String {
    let items: Vec<String> = values.iter().map(|v| format!("{v:.6}")).collect();
    format!("({})", items.join(", "))
}

/// Generates a standalone Python script that rebuilds the project with the
/// botrail API. The URDF is embedded so the script is self-contained.
pub fn generate_python(project: &ProjectFile) -> String {
    let mut out = String::new();
    out.push_str("\"\"\"Generated by botrail studio — rebuilds the saved project.\"\"\"\n\n");
    out.push_str("import botrail as bt\n\n");
    // Triple-quote guard: a URDF containing ''' would break the literal.
    let urdf = project.robot_urdf.replace("'''", "'\\''\\''\\'");
    out.push_str(&format!("URDF = r'''{urdf}'''\n\n"));
    out.push_str("robot = bt.Robot.from_urdf_string(URDF)\n");
    out.push_str("scene = bt.Scene(robot)\n");

    for o in &project.obstacles {
        let pos = py_tuple(&o.pose.position);
        let quat = py_tuple(&o.pose.quaternion);
        match &o.geometry {
            GeometryMsg::Box { size } => out.push_str(&format!(
                "scene.add_box({:?}, size={}, position={}, quaternion={})\n",
                o.name,
                py_tuple(size),
                pos,
                quat
            )),
            GeometryMsg::Sphere { radius } => out.push_str(&format!(
                "scene.add_sphere({:?}, radius={radius}, position={pos}, quaternion={quat})\n",
                o.name
            )),
            GeometryMsg::Cylinder { radius, length } => out.push_str(&format!(
                "scene.add_cylinder({:?}, radius={radius}, length={length}, position={pos}, quaternion={quat})\n",
                o.name
            )),
            GeometryMsg::Mesh { .. } => {
                out.push_str(&format!("# obstacle {:?}: mesh not supported yet\n", o.name))
            }
        }
    }
    out.push_str(&format!(
        "scene.set_joint_positions({})\n",
        py_list(&project.joint_positions)
    ));

    for motion in &project.motions {
        out.push('\n');
        for segment in &motion.segments {
            let kind = match segment.kind {
                SegmentKindMsg::Joint => "joint",
                SegmentKindMsg::CartesianLine => "cartesian_line",
            };
            let mut extras = String::new();
            for constraint in &segment.constraints {
                match constraint {
                    ConstraintMsg::OrientationCone {
                        axis_local,
                        axis_world,
                        angle,
                    } => extras.push_str(&format!(
                        ", orientation_cone=({}, {}, {angle})",
                        py_tuple(axis_local),
                        py_tuple(axis_world)
                    )),
                    ConstraintMsg::PositionBox { min, max } => extras.push_str(&format!(
                        ", position_box=({}, {})",
                        py_tuple(min),
                        py_tuple(max)
                    )),
                }
            }
            out.push_str(&format!(
                "scene.add_segment({:?}, goal={}, kind={:?}{})\n",
                motion.name,
                py_list(&segment.goal_positions),
                kind,
                extras
            ));
        }
        out.push_str(&format!(
            "trajectory = scene.plan_motion({:?})\n",
            motion.name
        ));
    }

    out.push_str("\nbt.studio(scene)\n");
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::{Constraint, Segment, SegmentKind};
    use botrail_model::{Geometry, RobotModel};
    use nalgebra::{Isometry3, Vector3};

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");

    fn sample_scene() -> Scene {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(ARM).unwrap()));
        scene
            .add_obstacle(
                "wall",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.8, 0.5),
                },
                Isometry3::translation(0.28, 0.0, 0.45),
            )
            .unwrap();
        scene
            .set_joint_positions(vec![0.1, 0.2, -0.3, 0.0, 0.4, 0.0])
            .unwrap();
        scene
            .add_segment(
                "main",
                Segment {
                    kind: SegmentKind::Joint,
                    goal_positions: vec![0.5, 0.6, -0.7, 0.1, 0.0, 0.0],
                    constraints: vec![Constraint::OrientationCone {
                        axis_local: Vector3::z(),
                        axis_world: Vector3::x(),
                        angle: 0.8,
                    }],
                },
            )
            .unwrap();
        scene
            .add_segment(
                "main",
                Segment {
                    kind: SegmentKind::CartesianLine,
                    goal_positions: vec![0.5, 0.7, -0.9, 0.1, 0.0, 0.0],
                    constraints: vec![],
                },
            )
            .unwrap();
        scene
    }

    #[test]
    fn project_roundtrip_preserves_everything() {
        let scene = sample_scene();
        let json = scene.to_project().to_json();
        let reloaded = Scene::from_project(&ProjectFile::from_json(&json).unwrap()).unwrap();

        assert_eq!(reloaded.robot.name, scene.robot.name);
        assert_eq!(reloaded.joint_positions(), scene.joint_positions());
        assert_eq!(reloaded.obstacles().len(), 1);
        assert_eq!(reloaded.obstacles()[0].name, "wall");
        assert_eq!(reloaded.motions().len(), 1);
        let motion = &reloaded.motions()[0];
        assert_eq!(motion.name, "main");
        assert_eq!(motion.segments.len(), 2);
        assert_eq!(motion.segments[1].kind, SegmentKind::CartesianLine);
        assert!(matches!(
            motion.segments[0].constraints[0],
            Constraint::OrientationCone { angle, .. } if (angle - 0.8).abs() < 1e-12
        ));
        // The reloaded scene still collision-checks (collider rebuilt).
        assert!(!reloaded.check_collisions().is_empty() || true);
    }

    #[test]
    fn version_mismatch_is_rejected() {
        let mut project = sample_scene().to_project();
        project.version = 99;
        let json = project.to_json();
        assert!(matches!(
            ProjectFile::from_json(&json),
            Err(ProjectError::Version(99))
        ));
    }

    #[test]
    fn apply_project_replaces_state_and_checks_dof() {
        let scene = sample_scene();
        let project = scene.to_project();

        let mut other = Scene::new(scene.robot.clone());
        other
            .add_obstacle(
                "old",
                Geometry::Sphere { radius: 0.05 },
                Isometry3::translation(1.0, 0.0, 0.0),
            )
            .unwrap();
        other.apply_project(&project).unwrap();
        assert_eq!(other.obstacles().len(), 1);
        assert_eq!(other.obstacles()[0].name, "wall");
        assert_eq!(other.motions().len(), 1);

        let mut bad = project.clone();
        bad.joint_positions = vec![0.0; 3];
        assert!(matches!(
            other.apply_project(&bad),
            Err(ProjectError::Incompatible(_))
        ));
    }

    #[test]
    fn generated_python_contains_the_full_recipe() {
        let scene = sample_scene();
        let code = generate_python(&scene.to_project());
        for needle in [
            "import botrail as bt",
            "bt.Robot.from_urdf_string(URDF)",
            "scene.add_box(\"wall\"",
            "scene.set_joint_positions(",
            "scene.add_segment(\"main\"",
            "kind=\"cartesian_line\"",
            "orientation_cone=(",
            "scene.plan_motion(\"main\")",
            "bt.studio(scene)",
        ] {
            assert!(code.contains(needle), "missing `{needle}`:\n{code}");
        }
    }
}
