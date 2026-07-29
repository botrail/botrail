//! Scene state (robot + joint configuration + obstacles) and the JSON wire
//! protocol spoken between the botrail server and the studio UI.
//!
//! The Rust types in [`wire`] are the single source of truth for the
//! protocol. The TypeScript side (`studio/src/generated/`) is generated from
//! them via ts-rs — run `scripts/gen_protocol.sh` after changing them.

pub mod motion;
pub mod project;
pub mod wire;

use std::sync::Arc;

use motion::{Motion, MotionError, PlannedMotion, Segment};

use botrail_collide::{Acm, CollisionPair, ObstacleCollider, RobotCollider};
use botrail_model::{Geometry, RobotModel};
use nalgebra::Isometry3;
use thiserror::Error;

/// Random-configuration samples used to auto-populate the ACM with
/// by-design contacts (MoveIt-Setup-Assistant style) at scene construction.
const ACM_SAMPLES: usize = 256;
const ACM_THRESHOLD: f64 = 0.95;

#[derive(Debug, Error)]
pub enum SceneError {
    #[error("expected {expected} joint positions, got {got}")]
    WrongDof { expected: usize, got: usize },
    #[error("unknown obstacle `{0}`")]
    UnknownObstacle(String),
    #[error("{0}")]
    UnsupportedGeometry(String),
}

#[derive(Debug, Clone)]
pub struct Obstacle {
    pub name: String,
    pub geometry: Geometry,
    pub pose: Isometry3<f64>,
}

/// A named world-frame pose — a mount point / teach reference, typically
/// imported from a scene file. Not a collision object.
#[derive(Debug, Clone)]
pub struct Frame {
    pub name: String,
    pub pose: Isometry3<f64>,
}

/// A robot placed in a world with obstacles. All poses entering or leaving
/// the scene (link poses, IK targets, obstacle poses, constraints) are in
/// the world frame; the robot root sits at `robot_base`. Collision checking
/// runs against solid colliders (see botrail-collide's shape policy).
#[derive(Clone)]
pub struct Scene {
    pub robot: Arc<RobotModel>,
    /// World pose of the robot's root link.
    robot_base: Isometry3<f64>,
    joint_positions: Vec<f64>,
    obstacles: Vec<Obstacle>,
    obstacle_colliders: Vec<ObstacleCollider>,
    robot_collider: RobotCollider,
    acm: Acm,
    motions: Vec<Motion>,
    frames: Vec<Frame>,
    /// Link shapes that could not be used for collision (e.g. unreadable
    /// mesh files). Surface these to the user once.
    pub collision_warnings: Vec<String>,
}

impl Scene {
    pub fn new(robot: Arc<RobotModel>) -> Self {
        Self::with_base(robot, Isometry3::identity())
    }

    /// A scene with the robot root placed at `base` (world frame).
    pub fn with_base(robot: Arc<RobotModel>, base: Isometry3<f64>) -> Self {
        let joint_positions = robot.neutral_positions();
        let (robot_collider, collision_warnings) = RobotCollider::from_model(&robot);
        let mut acm = Acm::adjacent(&robot);
        // Self-collision analysis is base-invariant: links move rigidly with
        // the base, so the identity-base sampling stays valid.
        for (i, j) in botrail_collide::detect_always_colliding(
            &robot,
            &robot_collider,
            &acm,
            ACM_SAMPLES,
            ACM_THRESHOLD,
        ) {
            acm.allow(i, j);
        }
        Self {
            robot,
            robot_base: base,
            joint_positions,
            obstacles: Vec::new(),
            obstacle_colliders: Vec::new(),
            robot_collider,
            acm,
            motions: Vec::new(),
            frames: Vec::new(),
            collision_warnings,
        }
    }

    // --------------------------------------------------------------- frames

    pub fn frames(&self) -> &[Frame] {
        &self.frames
    }

    pub fn frame(&self, name: &str) -> Option<&Frame> {
        self.frames.iter().find(|f| f.name == name)
    }

    /// Adds or replaces a named world-frame pose.
    pub fn add_frame(&mut self, name: &str, pose: Isometry3<f64>) {
        match self.frames.iter_mut().find(|f| f.name == name) {
            Some(frame) => frame.pose = pose,
            None => self.frames.push(Frame {
                name: name.to_string(),
                pose,
            }),
        }
    }

    /// World pose of the robot's root link.
    pub fn robot_base_pose(&self) -> &Isometry3<f64> {
        &self.robot_base
    }

    pub fn set_robot_base_pose(&mut self, pose: Isometry3<f64>) {
        self.robot_base = pose;
    }

    /// World pose of every link at configuration `q`.
    pub fn fk(&self, q: &[f64]) -> Result<Vec<Isometry3<f64>>, SceneError> {
        botrail_kin::forward_kinematics_with_base(&self.robot, q, &self.robot_base).map_err(|_| {
            SceneError::WrongDof {
                expected: self.robot.dof(),
                got: q.len(),
            }
        })
    }

    /// Solves IK for `link` toward a world-frame target: the target is
    /// re-expressed in the robot base frame before handing it to the
    /// base-frame solver.
    pub fn solve_ik_world(
        &self,
        link: usize,
        target_world: &Isometry3<f64>,
        seed: &[f64],
        options: &botrail_kin::IkOptions,
    ) -> Result<botrail_kin::IkResult, botrail_kin::KinError> {
        let target_base = self.robot_base.inverse() * target_world;
        botrail_kin::solve_ik(&self.robot, link, &target_base, seed, options)
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
        self.fk(&self.joint_positions)
            .expect("joint_positions length is enforced by set_joint_positions")
    }

    // ------------------------------------------------------------ obstacles

    pub fn obstacles(&self) -> &[Obstacle] {
        &self.obstacles
    }

    fn obstacle_index(&self, name: &str) -> Result<usize, SceneError> {
        self.obstacles
            .iter()
            .position(|o| o.name == name)
            .ok_or_else(|| SceneError::UnknownObstacle(name.to_string()))
    }

    fn unique_name(&self, requested: &str) -> String {
        let base = if requested.is_empty() {
            "obstacle"
        } else {
            requested
        };
        if self.obstacle_index(base).is_err() {
            return base.to_string();
        }
        let mut n = 2;
        loop {
            let candidate = format!("{base}_{n}");
            if self.obstacle_index(&candidate).is_err() {
                return candidate;
            }
            n += 1;
        }
    }

    /// Adds an obstacle and returns its (possibly uniquified) name.
    pub fn add_obstacle(
        &mut self,
        name: &str,
        geometry: Geometry,
        pose: Isometry3<f64>,
    ) -> Result<String, SceneError> {
        let collider = ObstacleCollider::from_geometry(&geometry)
            .map_err(|e| SceneError::UnsupportedGeometry(e.to_string()))?;
        let name = self.unique_name(name);
        self.obstacles.push(Obstacle {
            name: name.clone(),
            geometry,
            pose,
        });
        self.obstacle_colliders.push(collider);
        Ok(name)
    }

    pub fn remove_obstacle(&mut self, name: &str) -> Result<(), SceneError> {
        let index = self.obstacle_index(name)?;
        self.obstacles.remove(index);
        self.obstacle_colliders.remove(index);
        Ok(())
    }

    pub fn set_obstacle_pose(
        &mut self,
        name: &str,
        pose: Isometry3<f64>,
    ) -> Result<(), SceneError> {
        let index = self.obstacle_index(name)?;
        self.obstacles[index].pose = pose;
        Ok(())
    }

    pub fn set_obstacle_geometry(
        &mut self,
        name: &str,
        geometry: Geometry,
    ) -> Result<(), SceneError> {
        let index = self.obstacle_index(name)?;
        let collider = ObstacleCollider::from_geometry(&geometry)
            .map_err(|e| SceneError::UnsupportedGeometry(e.to_string()))?;
        self.obstacles[index].geometry = geometry;
        self.obstacle_colliders[index] = collider;
        Ok(())
    }

    // ------------------------------------------------------------ collision

    fn obstacle_query(&self) -> Vec<(Isometry3<f64>, &ObstacleCollider)> {
        self.obstacles
            .iter()
            .zip(&self.obstacle_colliders)
            .map(|(o, c)| (o.pose, c))
            .collect()
    }

    /// Self-collision (ACM-filtered) and robot-vs-obstacle pairs at the
    /// current configuration.
    pub fn check_collisions(&self) -> Vec<CollisionPair> {
        botrail_collide::check_scene(
            &self.robot_collider,
            &self.link_poses(),
            &self.acm,
            &self.obstacle_query(),
        )
    }

    /// Collision pairs at an arbitrary configuration (the scene state is
    /// not modified).
    pub fn collisions_at(&self, q: &[f64]) -> Result<Vec<CollisionPair>, SceneError> {
        let poses = self.fk(q)?;
        Ok(botrail_collide::check_scene(
            &self.robot_collider,
            &poses,
            &self.acm,
            &self.obstacle_query(),
        ))
    }

    /// True when `q` has the right DOF, respects the position limits, and
    /// is collision-free. This is the validity predicate handed to planners.
    pub fn is_state_valid(&self, q: &[f64]) -> bool {
        if q.len() != self.robot.dof() {
            return false;
        }
        let within =
            q.iter()
                .zip(self.robot.actuated_joint_limits())
                .all(|(v, limits)| match limits {
                    Some((lo, hi)) => *v >= lo - 1e-9 && *v <= hi + 1e-9,
                    None => true,
                });
        within && self.collisions_at(q).map(|c| c.is_empty()).unwrap_or(false)
    }

    /// Minimum robot-obstacle distance (0 when colliding); `None` without
    /// obstacles or collision geometry.
    pub fn min_obstacle_distance(&self) -> Option<f64> {
        botrail_collide::min_robot_obstacle_distance(
            &self.robot_collider,
            &self.link_poses(),
            &self.obstacle_query(),
        )
    }

    pub fn acm(&self) -> &Acm {
        &self.acm
    }

    // -------------------------------------------------------------- motions

    pub fn motions(&self) -> &[Motion] {
        &self.motions
    }

    fn motion_index(&self, name: &str) -> Result<usize, MotionError> {
        self.motions
            .iter()
            .position(|m| m.name == name)
            .ok_or_else(|| MotionError::UnknownMotion(name.to_string()))
    }

    /// Appends a segment to `motion`, creating the motion if needed.
    pub fn add_segment(&mut self, motion: &str, segment: Segment) -> Result<(), MotionError> {
        if segment.goal_positions.len() != self.robot.dof() {
            return Err(MotionError::WrongDof {
                index: self
                    .motion_index(motion)
                    .map(|i| self.motions[i].segments.len())
                    .unwrap_or(0),
                expected: self.robot.dof(),
                got: segment.goal_positions.len(),
            });
        }
        let index = match self.motion_index(motion) {
            Ok(i) => i,
            Err(_) => {
                self.motions.push(Motion {
                    name: motion.to_string(),
                    segments: Vec::new(),
                });
                self.motions.len() - 1
            }
        };
        self.motions[index].segments.push(segment);
        Ok(())
    }

    pub fn remove_segment(&mut self, motion: &str, segment: usize) -> Result<(), MotionError> {
        let index = self.motion_index(motion)?;
        if segment >= self.motions[index].segments.len() {
            return Err(MotionError::BadSegmentIndex(segment));
        }
        self.motions[index].segments.remove(segment);
        Ok(())
    }

    /// Removes every segment (the motion itself stays listed).
    pub fn clear_motion(&mut self, motion: &str) -> Result<(), MotionError> {
        let index = self.motion_index(motion)?;
        self.motions[index].segments.clear();
        Ok(())
    }

    pub fn set_motions(&mut self, motions: Vec<Motion>) {
        self.motions = motions;
    }

    pub fn set_frames(&mut self, frames: Vec<Frame>) {
        self.frames = frames;
    }

    /// Plans all segments of `motion` from the current configuration.
    pub fn plan_motion(
        &self,
        name: &str,
        plan_options: &botrail_plan::PlanOptions,
        limits: &botrail_traj::Limits,
    ) -> Result<PlannedMotion, MotionError> {
        let index = self.motion_index(name)?;
        motion::plan_motion(self, &self.motions[index], plan_options, limits)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    fn sample_scene() -> Scene {
        let urdf = r#"
        <robot name="r">
          <link name="a">
            <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
          </link>
          <link name="b">
            <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
          </link>
          <joint name="j" type="revolute">
            <parent link="a"/><child link="b"/>
            <origin xyz="0 0 0.5"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ))
    }

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
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

    #[test]
    fn obstacle_lifecycle_and_collisions() {
        let mut scene = sample_scene();
        assert!(scene.check_collisions().is_empty());
        assert_eq!(scene.min_obstacle_distance(), None);

        // Box far away: no collision, positive distance.
        let name = scene
            .add_obstacle(
                "table",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                iso(1.0, 0.0, 0.0),
            )
            .unwrap();
        assert_eq!(name, "table");
        assert!(scene.check_collisions().is_empty());
        let d = scene.min_obstacle_distance().unwrap();
        assert!((d - 0.85).abs() < 1e-9, "d = {d}");

        // Move it onto link b (at z = 0.5): collision, distance 0.
        scene
            .set_obstacle_pose("table", iso(0.0, 0.0, 0.5))
            .unwrap();
        let pairs = scene.check_collisions();
        assert_eq!(pairs.len(), 1);
        assert_eq!(scene.min_obstacle_distance(), Some(0.0));

        scene.remove_obstacle("table").unwrap();
        assert!(scene.check_collisions().is_empty());
        assert!(matches!(
            scene.remove_obstacle("table"),
            Err(SceneError::UnknownObstacle(_))
        ));
    }

    #[test]
    fn base_pose_shifts_links_collisions_and_ik() {
        let mut scene = sample_scene();
        // Obstacle sitting on link b's identity-base location (z = 0.5).
        scene
            .add_obstacle(
                "blocker",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                iso(0.0, 0.0, 0.5),
            )
            .unwrap();
        assert_eq!(scene.check_collisions().len(), 1);

        // Moving the base a meter away clears the collision and shifts
        // every link pose by the base transform.
        scene.set_robot_base_pose(iso(1.0, 0.0, 0.0));
        assert!(scene.check_collisions().is_empty());
        let poses = scene.link_poses();
        assert!((poses[0].translation.x - 1.0).abs() < 1e-12);
        assert!((poses[1].translation.x - 1.0).abs() < 1e-12);
        assert!((poses[1].translation.z - 0.5).abs() < 1e-12);

        // World-frame IK: a target expressed in the world frame lands on
        // the same configuration the identity-base solve finds for the
        // base-local target.
        let world_target = scene.robot_base_pose() * iso(0.0, 0.0, 0.5);
        let ik = scene
            .solve_ik_world(
                1,
                &world_target,
                &[0.3],
                &botrail_kin::IkOptions::default(),
            )
            .unwrap();
        assert!(ik.converged);
    }

    #[test]
    fn obstacle_names_are_uniquified() {
        let mut scene = sample_scene();
        let g = || Geometry::Sphere { radius: 0.05 };
        assert_eq!(
            scene.add_obstacle("ball", g(), iso(1.0, 0.0, 0.0)).unwrap(),
            "ball"
        );
        assert_eq!(
            scene.add_obstacle("ball", g(), iso(1.2, 0.0, 0.0)).unwrap(),
            "ball_2"
        );
        assert_eq!(
            scene.add_obstacle("", g(), iso(1.4, 0.0, 0.0)).unwrap(),
            "obstacle"
        );
    }

    #[test]
    fn mesh_obstacles_collide_via_vhacd() {
        let mut scene = sample_scene();
        let dir = std::env::temp_dir().join(format!("botrail-scene-test-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let stl = dir.join("box.stl");
        std::fs::write(
            &stl,
            botrail_mesh::to_stl_binary(&botrail_mesh::box_mesh([0.3, 0.3, 0.3])),
        )
        .unwrap();

        // Mesh box engulfing link b (at z = 0.5): collision.
        let name = scene
            .add_obstacle(
                "m",
                Geometry::Mesh {
                    path: stl.clone(),
                    scale: Vector3::new(1.0, 1.0, 1.0),
                },
                iso(0.0, 0.0, 0.5),
            )
            .unwrap();
        assert!(!scene.check_collisions().is_empty());

        // Moved away: clear, with a sane positive clearance.
        scene.set_obstacle_pose(&name, iso(2.0, 0.0, 0.0)).unwrap();
        assert!(scene.check_collisions().is_empty());
        assert!(scene.min_obstacle_distance().unwrap() > 1.0);

        // A missing file still fails cleanly.
        assert!(scene
            .add_obstacle(
                "missing",
                Geometry::Mesh {
                    path: dir.join("nope.stl"),
                    scale: Vector3::new(1.0, 1.0, 1.0),
                },
                iso(1.0, 0.0, 0.0),
            )
            .is_err());
        let _ = std::fs::remove_dir_all(&dir);
    }
}
