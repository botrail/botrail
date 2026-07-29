//! Shared scene state: single source of truth for Python callers and
//! connected studio clients.
//!
//! All protocol/planning logic lives in botrail-session; this hub supplies
//! the server-side plumbing ([`SessionHost`]: mutex-guarded scene, websocket
//! broadcast, Instant clock, stderr logging) plus the Python-facing sugar.

use std::path::PathBuf;
use std::sync::{Mutex, OnceLock};
use std::time::Instant;

use botrail_kin::IkOptions;
use botrail_model::Geometry;
use botrail_scene::wire::{self, PoseMsg, ServerMessage};
use botrail_scene::{Scene, SceneError};
use botrail_session::SessionHost;
use nalgebra::Isometry3;
use tokio::sync::broadcast;

pub(crate) use botrail_session::traj_limits;

fn pose_arrays(pose: &Isometry3<f64>) -> ([f64; 3], [f64; 4]) {
    let t = pose.translation;
    let q = pose.rotation.coords;
    ([t.x, t.y, t.z], [q.x, q.y, q.z, q.w])
}

pub struct SceneHub {
    scene: Mutex<Scene>,
    /// Serialized `ServerMessage`s fanned out to every websocket client.
    pub tx: broadcast::Sender<String>,
    /// Mesh id (URL path segment) -> filesystem path. Grows lazily as
    /// robot/obstacle mesh visuals are mapped to URLs.
    meshes: Mutex<Vec<PathBuf>>,
}

impl SessionHost for SceneHub {
    fn with_scene<R>(&self, f: impl FnOnce(&mut Scene) -> R) -> R {
        f(&mut self.scene.lock().expect("scene mutex poisoned"))
    }

    fn robot_asset_url(&self, path: &std::path::Path) -> Option<String> {
        path.file_name()
            .map(|f| format!("/usd-assets/{}", f.to_string_lossy()))
    }

    fn mesh_url(&self, path: &std::path::Path) -> (String, String) {
        let mut meshes = self.meshes.lock().expect("mesh registry poisoned");
        let id = meshes.iter().position(|p| p == path).unwrap_or_else(|| {
            meshes.push(path.to_path_buf());
            meshes.len() - 1
        });
        let ext = path
            .extension()
            .map(|e| e.to_string_lossy().to_lowercase())
            .unwrap_or_default();
        (format!("/meshes/{id}"), ext)
    }

    fn emit(&self, msg: &ServerMessage) {
        // Send errors just mean no client is connected right now.
        let _ = self
            .tx
            .send(serde_json::to_string(msg).expect("wire types serialize infallibly"));
    }

    fn now_ms(&self) -> f64 {
        static START: OnceLock<Instant> = OnceLock::new();
        START.get_or_init(Instant::now).elapsed().as_secs_f64() * 1000.0
    }

    fn log(&self, message: &str) {
        eprintln!("botrail: {message}");
    }
}

impl SceneHub {
    pub fn new(scene: Scene) -> Self {
        let (tx, _) = broadcast::channel(64);
        SceneHub {
            scene: Mutex::new(scene),
            tx,
            meshes: Mutex::new(Vec::new()),
        }
    }

    /// Filesystem path behind `/meshes/{id}`.
    pub fn mesh_path(&self, id: usize) -> Option<PathBuf> {
        self.meshes
            .lock()
            .expect("mesh registry poisoned")
            .get(id)
            .cloned()
    }

    pub fn scene_init_json(&self) -> String {
        let msg = self.with_scene(|scene| botrail_session::scene_init_message(self, scene));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    /// Directory `/assets/*` serves (a USD robot's stage directory, so
    /// relative references inside the stage resolve).
    pub fn robot_asset_dir(&self) -> Option<PathBuf> {
        self.with_scene(|scene| match &scene.robot.source {
            botrail_model::RobotSource::Usd { path, .. } => path.parent().map(|p| p.to_path_buf()),
            _ => None,
        })
    }

    pub fn state_json(&self) -> String {
        let msg = self.with_scene(|scene| wire::state_message(scene));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    pub fn set_joint_positions(&self, positions: Vec<f64>) -> Result<(), SceneError> {
        botrail_session::set_joint_positions(self, positions)
    }

    pub fn joint_positions(&self) -> Vec<f64> {
        self.with_scene(|scene| scene.joint_positions().to_vec())
    }

    pub fn link_pose(&self, link_name: &str) -> Option<([f64; 3], [f64; 4])> {
        self.with_scene(|scene| {
            let index = scene.robot.link_index(link_name)?;
            let pose = scene.link_poses()[index];
            let t = pose.translation;
            let q = pose.rotation.coords;
            Some(([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
        })
    }

    /// World pose of the robot root as `(position, quaternion_xyzw)`.
    pub fn robot_base_pose(&self) -> ([f64; 3], [f64; 4]) {
        pose_arrays(&self.robot_base_isometry())
    }

    // -------------------------------------------------------------- frames

    /// Adds or replaces a named world frame and broadcasts the frame list.
    pub fn add_frame(&self, name: &str, pose: Isometry3<f64>) {
        botrail_session::add_frames(self, vec![(name.to_string(), pose)]);
    }

    pub fn add_frames(&self, frames: Vec<(String, Isometry3<f64>)>) {
        botrail_session::add_frames(self, frames);
    }

    pub fn frames_json(&self) -> String {
        let msg = self.with_scene(|scene| wire::frames_message(scene));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    pub fn frames(&self) -> Vec<(String, ([f64; 3], [f64; 4]))> {
        self.with_scene(|scene| {
            scene
                .frames()
                .iter()
                .map(|f| (f.name.clone(), pose_arrays(&f.pose)))
                .collect()
        })
    }

    pub fn frame_pose(&self, name: &str) -> Option<([f64; 3], [f64; 4])> {
        self.with_scene(|scene| scene.frame(name).map(|f| pose_arrays(&f.pose)))
    }

    pub fn robot_base_isometry(&self) -> Isometry3<f64> {
        self.with_scene(|scene| *scene.robot_base_pose())
    }

    /// Places the robot's root link (world frame) and broadcasts the state.
    pub fn set_robot_base_pose(&self, pose: Isometry3<f64>) {
        botrail_session::set_robot_base_pose(self, pose);
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
        botrail_session::set_tcp_target(self, link, pose, options)
    }

    pub fn broadcast_state(&self) {
        botrail_session::emit_state(self);
    }

    // ------------------------------------------------------------ obstacles

    pub fn obstacles_json(&self) -> String {
        let msg = self.with_scene(|scene| wire::obstacles_message(scene, |p| self.mesh_url(p)));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    pub fn add_obstacle(
        &self,
        name: &str,
        geometry: Geometry,
        pose: Isometry3<f64>,
    ) -> Result<String, SceneError> {
        botrail_session::add_obstacle(self, name, geometry, pose)
    }

    /// Adds many obstacles with one broadcast; returns the final names.
    pub fn add_obstacles(
        &self,
        batch: Vec<(String, Geometry, Isometry3<f64>)>,
    ) -> Result<Vec<String>, SceneError> {
        botrail_session::add_obstacles(self, batch)
    }

    pub fn remove_obstacle(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_obstacle(self, name)
    }

    pub fn set_obstacle_enabled(&self, name: &str, enabled: bool) -> Result<(), SceneError> {
        botrail_session::set_obstacle_enabled(self, name, enabled)
    }

    pub fn set_obstacle_pose(&self, name: &str, pose: Isometry3<f64>) -> Result<(), SceneError> {
        botrail_session::set_obstacle_pose(self, name, pose)
    }

    pub fn obstacle_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.obstacles().iter().map(|o| o.name.clone()).collect())
    }

    // ------------------------------------------------------------ collision

    /// Colliding pairs as ((kind, name), (kind, name)) tuples.
    pub fn collision_pairs(&self) -> Vec<((String, String), (String, String))> {
        self.with_scene(|scene| {
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
        })
    }

    pub fn min_obstacle_distance(&self) -> Option<f64> {
        self.with_scene(|scene| scene.min_obstacle_distance())
    }

    pub fn collision_warnings(&self) -> Vec<String> {
        self.with_scene(|scene| scene.collision_warnings.clone())
    }

    // -------------------------------------------------------------- motions

    pub fn motions_json(&self) -> String {
        let msg = self.with_scene(|scene| wire::motions_message(scene));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    pub fn add_segment(
        &self,
        motion: &str,
        segment: botrail_scene::motion::Segment,
    ) -> Result<(), String> {
        botrail_session::add_segment(self, motion, segment)
    }

    pub fn remove_segment(&self, motion: &str, index: usize) -> Result<(), String> {
        botrail_session::remove_segment(self, motion, index)
    }

    pub fn clear_motion(&self, motion: &str) -> Result<(), String> {
        botrail_session::clear_motion(self, motion)
    }

    pub fn motion_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.motions().iter().map(|m| m.name.clone()).collect())
    }

    pub fn motion_segments(&self, name: &str) -> Vec<(String, Vec<f64>)> {
        self.with_scene(|scene| {
            scene
                .motions()
                .iter()
                .find(|m| m.name == name)
                .map(|m| {
                    m.segments
                        .iter()
                        .map(|s| {
                            let kind = match s.kind {
                                botrail_scene::motion::SegmentKind::Joint => "joint",
                                botrail_scene::motion::SegmentKind::CartesianLine => {
                                    "cartesian_line"
                                }
                            };
                            (kind.to_string(), s.goal_positions.clone())
                        })
                        .collect()
                })
                .unwrap_or_default()
        })
    }

    /// Plans a whole motion against a scene snapshot (no broadcast).
    pub fn plan_motion_snapshot(
        &self,
        name: &str,
        options: &botrail_plan::PlanOptions,
    ) -> Result<botrail_scene::motion::PlannedMotion, String> {
        botrail_session::plan_motion_snapshot(self, name, options)
    }

    /// Plans a whole motion against a scene snapshot and broadcasts the
    /// outcome as a `motion_result`.
    pub fn plan_motion_and_broadcast(
        &self,
        name: &str,
        options: &botrail_plan::PlanOptions,
    ) -> Result<(botrail_scene::motion::PlannedMotion, f64), String> {
        botrail_session::plan_motion_and_emit(self, name, options)
    }

    // -------------------------------------------------------------- project

    pub fn project(&self) -> botrail_scene::project::ProjectFile {
        self.with_scene(|scene| scene.to_project())
    }

    pub fn project_json(&self) -> String {
        self.with_scene(|scene| scene.to_project().to_json())
    }

    pub fn apply_project_json(&self, json: &str) -> Result<(), String> {
        let project =
            botrail_scene::project::ProjectFile::from_json(json).map_err(|e| e.to_string())?;
        self.with_scene(|scene| scene.apply_project(&project))
            .map_err(|e| e.to_string())?;
        let _ = self.tx.send(self.obstacles_json());
        let _ = self.tx.send(self.motions_json());
        self.broadcast_state();
        Ok(())
    }

    pub fn python_code(&self) -> String {
        self.with_scene(|scene| botrail_scene::project::generate_python(&scene.to_project()))
    }

    // ------------------------------------------------------------- planning

    /// Plans from the current configuration to `goal` against a snapshot of
    /// the scene (the lock is not held while planning), then time-
    /// parameterizes the path. Returns the trajectory, the sparse shortcut
    /// path (kept for script export), and the wall-clock milliseconds spent.
    pub fn plan_to(
        &self,
        goal: &[f64],
        options: &botrail_plan::PlanOptions,
    ) -> Result<(botrail_traj::JointTrajectory, Vec<Vec<f64>>, f64), String> {
        botrail_session::plan_to(self, goal, options)
    }

    /// Runs `plan_to` and broadcasts the outcome (success or failure) as a
    /// `plan_result` message.
    pub fn plan_and_broadcast(
        &self,
        goal: &[f64],
        options: &botrail_plan::PlanOptions,
    ) -> Result<(botrail_traj::JointTrajectory, Vec<Vec<f64>>, f64), String> {
        botrail_session::plan_and_emit(self, goal, options)
    }

    pub fn handle_client_message(&self, text: &str) {
        botrail_session::handle_client_message(self, text);
    }
}
