//! Shared scene state: single source of truth for Python callers and
//! connected studio clients.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Mutex;

use botrail_kin::{solve_ik, IkMode, IkOptions};
use botrail_model::Geometry;
use botrail_scene::wire::{self, IkStatusMsg, PoseMsg, SceneDescriptionMsg, ServerMessage};
use botrail_scene::{Scene, SceneError};
use nalgebra::Isometry3;
use tokio::sync::broadcast;

pub struct SceneHub {
    scene: Mutex<Scene>,
    /// Serialized `ServerMessage`s fanned out to every websocket client.
    pub tx: broadcast::Sender<String>,
    /// Mesh id (URL path segment) -> filesystem path.
    pub meshes: Vec<PathBuf>,
}

impl SceneHub {
    pub fn new(scene: Scene) -> Self {
        let mut meshes = Vec::new();
        let mut seen: HashMap<PathBuf, usize> = HashMap::new();
        for link in &scene.robot.links {
            for shape in &link.visuals {
                if let Geometry::Mesh { path, .. } = &shape.geometry {
                    seen.entry(path.clone()).or_insert_with(|| {
                        meshes.push(path.clone());
                        meshes.len() - 1
                    });
                }
            }
        }
        let (tx, _) = broadcast::channel(64);
        SceneHub {
            scene: Mutex::new(scene),
            tx,
            meshes,
        }
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, Scene> {
        self.scene.lock().expect("scene mutex poisoned")
    }

    pub fn scene_init_json(&self) -> String {
        let scene = self.lock();
        let mesh_ids: HashMap<PathBuf, usize> = self
            .meshes
            .iter()
            .enumerate()
            .map(|(i, p)| (p.clone(), i))
            .collect();
        let desc = SceneDescriptionMsg::from_scene(&scene, |path| {
            let id = mesh_ids[path];
            let ext = path
                .extension()
                .map(|e| e.to_string_lossy().to_lowercase())
                .unwrap_or_default();
            (format!("/meshes/{id}"), ext)
        });
        serde_json::to_string(&ServerMessage::SceneInit { scene: desc })
            .expect("wire types serialize infallibly")
    }

    pub fn state_json(&self) -> String {
        self.state_json_with(None)
    }

    fn state_json_with(&self, ik_status: Option<IkStatusMsg>) -> String {
        let scene = self.lock();
        serde_json::to_string(&botrail_scene::wire::state_message_with_ik(
            &scene, ik_status,
        ))
        .expect("wire types serialize infallibly")
    }

    pub fn set_joint_positions(&self, positions: Vec<f64>) -> Result<(), SceneError> {
        self.lock().set_joint_positions(positions)?;
        self.broadcast_state();
        Ok(())
    }

    pub fn joint_positions(&self) -> Vec<f64> {
        self.lock().joint_positions().to_vec()
    }

    pub fn link_pose(&self, link_name: &str) -> Option<([f64; 3], [f64; 4])> {
        let scene = self.lock();
        let index = scene.robot.link_index(link_name)?;
        let pose = scene.link_poses()[index];
        let t = pose.translation;
        let q = pose.rotation.coords;
        Some(([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
    }

    /// Solves IK for `link` toward `pose`, seeded from the current
    /// configuration, applies the (best-effort) result, and broadcasts the
    /// new state tagged with the IK outcome.
    pub fn set_tcp_target(
        &self,
        link: &str,
        pose: &PoseMsg,
        options: &IkOptions,
    ) -> Result<botrail_kin::IkResult, String> {
        let mut scene = self.lock();
        let index = scene
            .robot
            .link_index(link)
            .ok_or_else(|| format!("unknown link `{link}`"))?;
        let target: Isometry3<f64> = pose.into();
        let seed = scene.joint_positions().to_vec();
        let result =
            solve_ik(&scene.robot, index, &target, &seed, options).map_err(|e| e.to_string())?;
        scene
            .set_joint_positions(result.q.clone())
            .map_err(|e| e.to_string())?;
        drop(scene);
        let status = IkStatusMsg {
            converged: result.converged,
            pos_error: result.pos_error,
            rot_error: result.rot_error,
        };
        let _ = self.tx.send(self.state_json_with(Some(status)));
        Ok(result)
    }

    pub fn broadcast_state(&self) {
        // Send errors just mean no client is connected right now.
        let _ = self.tx.send(self.state_json());
    }

    // ------------------------------------------------------------ obstacles

    pub fn obstacles_json(&self) -> String {
        let scene = self.lock();
        serde_json::to_string(&wire::obstacles_message(&scene))
            .expect("wire types serialize infallibly")
    }

    fn broadcast_obstacles_and_state(&self) {
        let _ = self.tx.send(self.obstacles_json());
        self.broadcast_state();
    }

    pub fn add_obstacle(
        &self,
        name: &str,
        geometry: Geometry,
        pose: Isometry3<f64>,
    ) -> Result<String, SceneError> {
        let final_name = self.lock().add_obstacle(name, geometry, pose)?;
        self.broadcast_obstacles_and_state();
        Ok(final_name)
    }

    pub fn remove_obstacle(&self, name: &str) -> Result<(), SceneError> {
        self.lock().remove_obstacle(name)?;
        self.broadcast_obstacles_and_state();
        Ok(())
    }

    pub fn set_obstacle_pose(&self, name: &str, pose: Isometry3<f64>) -> Result<(), SceneError> {
        self.lock().set_obstacle_pose(name, pose)?;
        self.broadcast_obstacles_and_state();
        Ok(())
    }

    pub fn set_obstacle_geometry(&self, name: &str, geometry: Geometry) -> Result<(), SceneError> {
        self.lock().set_obstacle_geometry(name, geometry)?;
        self.broadcast_obstacles_and_state();
        Ok(())
    }

    pub fn obstacle_names(&self) -> Vec<String> {
        self.lock()
            .obstacles()
            .iter()
            .map(|o| o.name.clone())
            .collect()
    }

    // ------------------------------------------------------------ collision

    /// Colliding pairs as ((kind, name), (kind, name)) tuples.
    pub fn collision_pairs(&self) -> Vec<((String, String), (String, String))> {
        let scene = self.lock();
        let describe = |id: botrail_collide::ColliderId| match id {
            botrail_collide::ColliderId::Link(i) => {
                ("link".to_string(), scene.robot.links[i].name.clone())
            }
            botrail_collide::ColliderId::Obstacle(k) => {
                ("obstacle".to_string(), scene.obstacles()[k].name.clone())
            }
        };
        scene
            .check_collisions()
            .into_iter()
            .map(|p| (describe(p.a), describe(p.b)))
            .collect()
    }

    pub fn min_obstacle_distance(&self) -> Option<f64> {
        self.lock().min_obstacle_distance()
    }

    pub fn collision_warnings(&self) -> Vec<String> {
        self.lock().collision_warnings.clone()
    }

    // -------------------------------------------------------------- motions

    pub fn motions_json(&self) -> String {
        let scene = self.lock();
        serde_json::to_string(&wire::motions_message(&scene))
            .expect("wire types serialize infallibly")
    }

    fn broadcast_motions(&self) {
        let _ = self.tx.send(self.motions_json());
    }

    pub fn add_segment(
        &self,
        motion: &str,
        segment: botrail_scene::motion::Segment,
    ) -> Result<(), String> {
        self.lock()
            .add_segment(motion, segment)
            .map_err(|e| e.to_string())?;
        self.broadcast_motions();
        Ok(())
    }

    pub fn remove_segment(&self, motion: &str, index: usize) -> Result<(), String> {
        self.lock()
            .remove_segment(motion, index)
            .map_err(|e| e.to_string())?;
        self.broadcast_motions();
        Ok(())
    }

    pub fn clear_motion(&self, motion: &str) -> Result<(), String> {
        self.lock()
            .clear_motion(motion)
            .map_err(|e| e.to_string())?;
        self.broadcast_motions();
        Ok(())
    }

    pub fn motion_names(&self) -> Vec<String> {
        self.lock()
            .motions()
            .iter()
            .map(|m| m.name.clone())
            .collect()
    }

    pub fn motion_segments(&self, name: &str) -> Vec<(String, Vec<f64>)> {
        self.lock()
            .motions()
            .iter()
            .find(|m| m.name == name)
            .map(|m| {
                m.segments
                    .iter()
                    .map(|s| {
                        let kind = match s.kind {
                            botrail_scene::motion::SegmentKind::Joint => "joint",
                            botrail_scene::motion::SegmentKind::CartesianLine => "cartesian_line",
                        };
                        (kind.to_string(), s.goal_positions.clone())
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Plans a whole motion against a scene snapshot (no broadcast).
    pub fn plan_motion_snapshot(
        &self,
        name: &str,
        options: &botrail_plan::PlanOptions,
    ) -> Result<botrail_scene::motion::PlannedMotion, String> {
        let snapshot = self.lock().clone();
        snapshot
            .plan_motion(name, options, &traj_limits(&snapshot.robot))
            .map_err(|e| e.to_string())
    }

    /// Plans a whole motion against a scene snapshot and broadcasts the
    /// outcome as a `motion_result`.
    pub fn plan_motion_and_broadcast(
        &self,
        name: &str,
        options: &botrail_plan::PlanOptions,
    ) -> Result<(botrail_scene::motion::PlannedMotion, f64), String> {
        let t0 = std::time::Instant::now();
        let result = self.plan_motion_snapshot(name, options);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        let msg = match &result {
            Ok(planned) => ServerMessage::MotionResult {
                ok: true,
                motion: name.to_string(),
                error: None,
                trajectory: Some(self.trajectory_msg(&planned.trajectory)),
                segment_ends: planned.segment_ends.clone(),
                planning_time_ms: Some(ms),
            },
            Err(e) => ServerMessage::MotionResult {
                ok: false,
                motion: name.to_string(),
                error: Some(e.clone()),
                trajectory: None,
                segment_ends: Vec::new(),
                planning_time_ms: None,
            },
        };
        let _ = self
            .tx
            .send(serde_json::to_string(&msg).expect("wire types serialize infallibly"));
        result.map(|planned| (planned, ms))
    }

    // -------------------------------------------------------------- project

    pub fn project_json(&self) -> String {
        self.lock().to_project().to_json()
    }

    pub fn apply_project_json(&self, json: &str) -> Result<(), String> {
        let project =
            botrail_scene::project::ProjectFile::from_json(json).map_err(|e| e.to_string())?;
        self.lock()
            .apply_project(&project)
            .map_err(|e| e.to_string())?;
        let _ = self.tx.send(self.obstacles_json());
        self.broadcast_motions();
        self.broadcast_state();
        Ok(())
    }

    pub fn python_code(&self) -> String {
        botrail_scene::project::generate_python(&self.lock().to_project())
    }

    // ------------------------------------------------------------- planning

    /// Plans from the current configuration to `goal` against a snapshot of
    /// the scene (the lock is not held while planning), then time-
    /// parameterizes the path. Returns the trajectory, the shortcut path
    /// waypoint count, and the wall-clock milliseconds spent.
    pub fn plan_to(
        &self,
        goal: &[f64],
        options: &botrail_plan::PlanOptions,
    ) -> Result<(botrail_traj::JointTrajectory, usize, f64), String> {
        let snapshot = self.lock().clone();
        let start = snapshot.joint_positions().to_vec();
        let (lower, upper) = snapshot.robot.sampling_bounds();
        let space = botrail_plan::JointSpace { lower, upper };

        let t0 = std::time::Instant::now();
        let path = {
            let mut is_valid = |q: &[f64]| {
                snapshot
                    .collisions_at(q)
                    .map(|c| c.is_empty())
                    .unwrap_or(false)
            };
            botrail_plan::plan(&space, &start, goal, &mut is_valid, options)
                .map_err(|e| e.to_string())?
        };
        let limits = traj_limits(&snapshot.robot);
        let traj = botrail_traj::time_parameterize(
            &path,
            &limits,
            &botrail_traj::TimingOptions::default(),
        )
        .map_err(|e| e.to_string())?;
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        Ok((traj, path.len(), ms))
    }

    /// Runs `plan_to` and broadcasts the outcome (success or failure) as a
    /// `plan_result` message.
    pub fn plan_and_broadcast(
        &self,
        goal: &[f64],
        options: &botrail_plan::PlanOptions,
    ) -> Result<(botrail_traj::JointTrajectory, usize, f64), String> {
        let result = self.plan_to(goal, options);
        let msg = match &result {
            Ok((traj, waypoints, ms)) => ServerMessage::PlanResult {
                ok: true,
                error: None,
                trajectory: Some(self.trajectory_msg(traj)),
                stats: Some(botrail_scene::wire::PlanStatsMsg {
                    planning_time_ms: *ms,
                    waypoints: *waypoints,
                }),
            },
            Err(e) => ServerMessage::PlanResult {
                ok: false,
                error: Some(e.clone()),
                trajectory: None,
                stats: None,
            },
        };
        let _ = self
            .tx
            .send(serde_json::to_string(&msg).expect("wire types serialize infallibly"));
        result
    }

    /// Samples a trajectory at ~30Hz with per-sample FK for playback.
    fn trajectory_msg(&self, traj: &botrail_traj::JointTrajectory) -> wire::TrajectoryMsg {
        let robot = self.lock().robot.clone();
        let (times, joint_positions) = traj.resample(1.0 / 30.0);
        let link_poses = joint_positions
            .iter()
            .map(|q| {
                botrail_kin::forward_kinematics(&robot, q)
                    .expect("trajectory q has scene DOF")
                    .iter()
                    .map(PoseMsg::from)
                    .collect()
            })
            .collect();
        wire::TrajectoryMsg {
            duration: traj.duration(),
            times,
            joint_positions,
            link_poses,
        }
    }

    pub fn handle_client_message(&self, text: &str) {
        use botrail_scene::wire::ClientMessage;
        match serde_json::from_str::<ClientMessage>(text) {
            Ok(ClientMessage::SetJointPositions { positions }) => {
                if let Err(e) = self.set_joint_positions(positions) {
                    eprintln!("botrail: rejected client message: {e}");
                }
            }
            Ok(ClientMessage::SetTcpTarget { link, pose }) => {
                // Warm-seeded streaming solve: the gizmo sends targets at
                // ~60Hz, so a few iterations per message are enough.
                let options = IkOptions {
                    mode: IkMode::Pose,
                    ..IkOptions::streaming()
                };
                if let Err(e) = self.set_tcp_target(&link, &pose, &options) {
                    eprintln!("botrail: rejected tcp target: {e}");
                }
            }
            Ok(ClientMessage::AddObstacle { obstacle }) => {
                let result = wire::geometry_from_msg(&obstacle.geometry)
                    .map_err(SceneError::UnsupportedGeometry)
                    .and_then(|geometry| {
                        self.add_obstacle(&obstacle.name, geometry, (&obstacle.pose).into())
                    });
                if let Err(e) = result {
                    eprintln!("botrail: rejected add_obstacle: {e}");
                }
            }
            Ok(ClientMessage::UpdateObstaclePose { name, pose }) => {
                if let Err(e) = self.set_obstacle_pose(&name, (&pose).into()) {
                    eprintln!("botrail: rejected update_obstacle_pose: {e}");
                }
            }
            Ok(ClientMessage::UpdateObstacleGeometry { name, geometry }) => {
                let result = wire::geometry_from_msg(&geometry)
                    .map_err(SceneError::UnsupportedGeometry)
                    .and_then(|geometry| self.set_obstacle_geometry(&name, geometry));
                if let Err(e) = result {
                    eprintln!("botrail: rejected update_obstacle_geometry: {e}");
                }
            }
            Ok(ClientMessage::RemoveObstacle { name }) => {
                if let Err(e) = self.remove_obstacle(&name) {
                    eprintln!("botrail: rejected remove_obstacle: {e}");
                }
            }
            Ok(ClientMessage::PlanRequest { goal_positions }) => {
                // Failure is reported to clients inside the plan_result.
                let _ =
                    self.plan_and_broadcast(&goal_positions, &botrail_plan::PlanOptions::default());
            }
            Ok(ClientMessage::AddSegment { motion, segment }) => {
                if let Err(e) = self.add_segment(&motion, wire::segment_from_msg(&segment)) {
                    eprintln!("botrail: rejected add_segment: {e}");
                }
            }
            Ok(ClientMessage::RemoveSegment { motion, index }) => {
                if let Err(e) = self.remove_segment(&motion, index) {
                    eprintln!("botrail: rejected remove_segment: {e}");
                }
            }
            Ok(ClientMessage::ClearMotion { motion }) => {
                if let Err(e) = self.clear_motion(&motion) {
                    eprintln!("botrail: rejected clear_motion: {e}");
                }
            }
            Ok(ClientMessage::PlanMotion { motion }) => {
                // Failure is reported to clients inside the motion_result.
                let _ =
                    self.plan_motion_and_broadcast(&motion, &botrail_plan::PlanOptions::default());
            }
            Err(e) => eprintln!("botrail: unparseable client message: {e}"),
        }
    }
}

/// Trajectory limits from the URDF: joint velocity limits (defaulting to
/// 1 rad/s where unspecified) and acceleration at twice the velocity bound
/// (URDF has no acceleration field; reaches peak speed in 0.5s).
fn traj_limits(model: &botrail_model::RobotModel) -> botrail_traj::Limits {
    let velocity: Vec<f64> = model
        .actuated_joints
        .iter()
        .map(|&ji| match model.joints[ji].limits {
            Some(l) if l.velocity > 0.0 => l.velocity,
            _ => 1.0,
        })
        .collect();
    let acceleration = velocity.iter().map(|v| 2.0 * v).collect();
    botrail_traj::Limits {
        velocity,
        acceleration,
    }
}
