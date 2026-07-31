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

/// `(position, quaternion_xyzw)` — the pose shape crossing into Python.
pub type PoseArrays = ([f64; 3], [f64; 4]);

fn pose_arrays(pose: &Isometry3<f64>) -> PoseArrays {
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
    /// Last successful recording playback, replayed to late-joining
    /// clients — the normal flow is "script plays, then the browser opens",
    /// so the original broadcast usually lands before anyone connects.
    last_recording: Mutex<Option<String>>,
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
            last_recording: Mutex::new(None),
        }
    }

    /// The last successful `recording_result` (serialized), for handshakes.
    pub fn last_recording_json(&self) -> Option<String> {
        self.last_recording
            .lock()
            .expect("recording mutex poisoned")
            .clone()
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

    pub fn link_pose(&self, link_name: &str) -> Option<PoseArrays> {
        self.with_scene(|scene| {
            let index = scene.robot.link_index(link_name)?;
            let pose = scene.link_poses()[index];
            let t = pose.translation;
            let q = pose.rotation.coords;
            Some(([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
        })
    }

    /// World pose of the robot root as `(position, quaternion_xyzw)`.
    pub fn robot_base_pose(&self) -> PoseArrays {
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

    pub fn frames(&self) -> Vec<(String, PoseArrays)> {
        self.with_scene(|scene| {
            scene
                .frames()
                .iter()
                .map(|f| (f.name.clone(), pose_arrays(&f.pose)))
                .collect()
        })
    }

    pub fn frame_pose(&self, name: &str) -> Option<PoseArrays> {
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

    /// World pose of an obstacle as `(position, quaternion_xyzw)`.
    pub fn obstacle_pose(&self, name: &str) -> Option<PoseArrays> {
        self.with_scene(|scene| {
            let o = scene.obstacles().iter().find(|o| o.name == name)?;
            let t = o.pose.translation;
            let q = o.pose.rotation.coords;
            Some(([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
        })
    }

    pub fn attach_obstacle(
        &self,
        name: &str,
        link: Option<&str>,
        touch_links: Option<&[String]>,
    ) -> Result<(), SceneError> {
        botrail_session::attach_obstacle(self, name, link, touch_links)
    }

    pub fn detach_obstacle(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::detach_obstacle(self, name)
    }

    /// Attached obstacles as (object, carrying link name) pairs.
    pub fn attachments(&self) -> Vec<(String, String)> {
        self.with_scene(|scene| {
            scene
                .attachments()
                .iter()
                .map(|a| (a.object.clone(), scene.robot.links[a.link].name.clone()))
                .collect()
        })
    }

    // ------------------------------------------------------------ sequences

    pub fn sequences_json(&self) -> String {
        let msg = self.with_scene(|scene| wire::sequences_message(scene));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    /// Adds or replaces a sequence from its wire-format JSON.
    pub fn upsert_sequence_json(&self, json: &str) -> Result<(), String> {
        let msg: wire::SequenceMsg =
            serde_json::from_str(json).map_err(|e| format!("invalid sequence JSON: {e}"))?;
        botrail_session::upsert_sequence(self, wire::sequence_from_msg(&msg));
        Ok(())
    }

    pub fn remove_sequence(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_sequence(self, name)
    }

    pub fn sequence_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.sequences().iter().map(|s| s.name.clone()).collect())
    }

    pub fn define_signal(&self, name: &str, initial: bool) {
        botrail_session::define_signal(self, name, initial);
    }

    pub fn remove_signal(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_signal(self, name)
    }

    pub fn signals(&self) -> Vec<(String, bool)> {
        self.with_scene(|scene| {
            scene
                .signals()
                .iter()
                .map(|s| (s.name.clone(), s.initial))
                .collect()
        })
    }

    pub fn sensors_json(&self) -> String {
        let msg = self.with_scene(|scene| wire::sensors_message(scene));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    pub fn devices_json(&self) -> String {
        let msg = self.with_scene(|scene| wire::devices_message(scene));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    pub fn upsert_sensor(&self, sensor: botrail_scene::seq::Sensor) {
        botrail_session::upsert_sensor(self, sensor);
    }

    pub fn remove_sensor(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_sensor(self, name)
    }

    pub fn sensor_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.sensors().iter().map(|s| s.name.clone()).collect())
    }

    pub fn upsert_device(&self, device: botrail_scene::seq::Device) {
        botrail_session::upsert_device(self, device);
    }

    pub fn remove_device(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_device(self, name)
    }

    pub fn device_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.devices().iter().map(|d| d.name.clone()).collect())
    }

    /// Rolls out a sequence (broadcasting the result to the studio) and
    /// returns the timeline plus the snapshot it ran against.
    pub fn simulate_sequence(
        &self,
        name: &str,
        options: &botrail_scene::rollout::RolloutOptions,
    ) -> Result<(botrail_scene::rollout::SequenceTimeline, Scene), String> {
        let snapshot = self.snapshot();
        let timeline = botrail_session::simulate_sequence_and_emit(self, name, options)?;
        Ok((timeline, snapshot))
    }

    // ------------------------------------------------------------ usd export

    /// Bakes a trajectory into a USD animation layer at `path`: robot link
    /// transforms as timeSamples (USD-sourced robots reference their stage,
    /// URDF robots are authored from the model), every obstacle as a prim,
    /// and grasped objects riding the arm on sampled tracks. Returns
    /// exporter warnings.
    pub fn export_trajectory_usd(
        &self,
        traj: &botrail_traj::JointTrajectory,
        path: &std::path::Path,
        fps: f64,
    ) -> Result<Vec<String>, String> {
        if !(fps.is_finite() && fps > 0.0) {
            return Err(format!("fps must be positive, got {fps}"));
        }
        let scene = self.snapshot();
        let duration = traj.duration();
        // Frame times on the fps grid, plus the exact final time.
        let mut times = Vec::new();
        let mut k = 0u64;
        loop {
            let t = k as f64 / fps;
            if t >= duration - 1e-9 {
                break;
            }
            times.push(t);
            k += 1;
        }
        times.push(duration);

        let mut link_poses = Vec::with_capacity(times.len());
        for &t in &times {
            let poses = scene.fk(&traj.sample(t)).map_err(|e| e.to_string())?;
            link_poses.push(poses);
        }
        let objects: Vec<botrail_usd::export::ObjectSpec> = scene
            .obstacles()
            .iter()
            .map(|o| {
                let track = match scene.attachment(&o.name) {
                    Some(att) => botrail_usd::export::PoseTrack::Sampled(
                        link_poses.iter().map(|p| p[att.link] * att.grasp).collect(),
                    ),
                    None => botrail_usd::export::PoseTrack::Static(o.pose),
                };
                botrail_usd::export::ObjectSpec {
                    name: o.name.clone(),
                    geometry: o.geometry.clone(),
                    track,
                }
            })
            .collect();
        let joint_samples: Vec<Vec<f64>> = times.iter().map(|&t| traj.sample(t)).collect();
        let input = botrail_usd::export::AnimationInput {
            model: &scene.robot,
            times: &times,
            link_poses: &link_poses,
            joint_samples: Some(&joint_samples),
            objects: &objects,
        };
        let options = botrail_usd::export::ExportOptions { fps };
        botrail_usd::export::write_animation(path, &input, &options).map_err(|e| e.to_string())
    }

    /// Loads a baked USD recording (an Isaac Sim capture or a botrail
    /// export), lifts it onto the scene's robot, broadcasts the playable
    /// timeline to the studio, and returns
    /// `(mode, duration, warnings, moving-object names)`.
    #[allow(clippy::type_complexity)]
    pub fn play_usd_animation(
        &self,
        path: &std::path::Path,
        force_transforms: bool,
    ) -> Result<(String, f64, Vec<String>, Vec<String>), String> {
        let scene = self.snapshot();
        let obstacle_names: Vec<String> =
            scene.obstacles().iter().map(|o| o.name.clone()).collect();
        let options = botrail_usd::recording::RecordingImportOptions {
            search_paths: Vec::new(),
            force_transforms,
        };
        let source = path.display().to_string();
        match botrail_usd::recording::import_recording(
            path,
            &scene.robot,
            &obstacle_names,
            &options,
        ) {
            Ok(rec) => {
                use botrail_usd::recording::RecordingMode;
                let mode = match rec.mode {
                    RecordingMode::JointState => "joint_state",
                    RecordingMode::Transforms => "transforms",
                };
                let duration = rec.times.last().copied().unwrap_or(0.0);
                let track_names: Vec<String> =
                    rec.object_tracks.iter().map(|(n, _)| n.clone()).collect();
                let trajectory = wire::TrajectoryMsg {
                    duration,
                    times: rec.times,
                    joint_positions: rec.joint_samples.unwrap_or_default(),
                    link_poses: match rec.mode {
                        // Joint mode rides the existing joint-playback path.
                        RecordingMode::JointState => None,
                        RecordingMode::Transforms => Some(
                            rec.link_poses
                                .iter()
                                .map(|frame| frame.iter().map(wire::PoseMsg::from).collect())
                                .collect(),
                        ),
                    },
                    object_tracks: (!rec.object_tracks.is_empty()).then(|| {
                        rec.object_tracks
                            .iter()
                            .map(|(name, poses)| wire::ObjectTrackMsg {
                                name: name.clone(),
                                poses: poses.iter().map(wire::PoseMsg::from).collect(),
                            })
                            .collect()
                    }),
                };
                let timeline = wire::TimelineMsg {
                    duration,
                    trajectory,
                    step_spans: Vec::new(),
                    signals: Vec::new(),
                };
                let msg = ServerMessage::RecordingResult {
                    ok: true,
                    source,
                    error: None,
                    mode: Some(mode.to_string()),
                    warnings: rec.warnings.clone(),
                    timeline: Some(timeline),
                };
                *self
                    .last_recording
                    .lock()
                    .expect("recording mutex poisoned") =
                    Some(serde_json::to_string(&msg).expect("wire types serialize infallibly"));
                self.emit(&msg);
                Ok((mode.to_string(), duration, rec.warnings, track_names))
            }
            Err(e) => {
                self.emit(&ServerMessage::RecordingResult {
                    ok: false,
                    source,
                    error: Some(e.to_string()),
                    mode: None,
                    warnings: Vec::new(),
                    timeline: None,
                });
                Err(e.to_string())
            }
        }
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
                // Remapped to obstacle ids by Scene before pairs get here.
                botrail_collide::ColliderId::Attached(_) => {
                    unreachable!("attached ids are remapped by Scene")
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
        *self
            .last_recording
            .lock()
            .expect("recording mutex poisoned") = None;
        let _ = self.tx.send(self.obstacles_json());
        let _ = self.tx.send(self.motions_json());
        let _ = self.tx.send(self.sequences_json());
        let _ = self.tx.send(self.sensors_json());
        let _ = self.tx.send(self.devices_json());
        let _ = self.tx.send(self.frames_json());
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
