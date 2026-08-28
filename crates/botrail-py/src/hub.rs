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
use botrail_scene::{ObstacleSpec, Scene, SceneError};
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
    /// Last successful rollout (pre-rollout snapshot + timeline), baked on
    /// demand by the studio's USD download.
    baked: Mutex<Option<(Scene, botrail_scene::rollout::SequenceTimeline)>>,
}

impl SessionHost for SceneHub {
    fn with_scene<R>(&self, f: impl FnOnce(&mut Scene) -> R) -> R {
        f(&mut self.scene.lock().expect("scene mutex poisoned"))
    }

    fn robot_asset_url(&self, robot: usize, path: &std::path::Path) -> Option<String> {
        // Namespaced by robot index (not name: names are user-chosen and
        // would need URL escaping); the client treats the URL as opaque.
        path.file_name()
            .map(|f| format!("/usd-assets/{robot}/{}", f.to_string_lossy()))
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

    fn has_listeners(&self) -> bool {
        self.tx.receiver_count() > 0
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

    fn store_baked(&self, scene: &Scene, timeline: &botrail_scene::rollout::SequenceTimeline) {
        *self.baked.lock().expect("baked mutex poisoned") = Some((scene.clone(), timeline.clone()));
    }

    fn baked(&self) -> Option<(Scene, botrail_scene::rollout::SequenceTimeline)> {
        self.baked.lock().expect("baked mutex poisoned").clone()
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
            baked: Mutex::new(None),
        }
    }

    /// The last successful `recording_result` (serialized), for handshakes.
    pub fn last_recording_json(&self) -> Option<String> {
        self.last_recording
            .lock()
            .expect("recording mutex poisoned")
            .clone()
    }

    /// The last successful bake as a serialized `sequence_result`, for
    /// handshakes (late joiners get the cycle a headless script baked).
    pub fn last_sequence_result_json(&self) -> Option<String> {
        botrail_session::baked_result_message(self)
            .map(|msg| serde_json::to_string(&msg).expect("wire types serialize infallibly"))
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

    /// The connection handshake (scene_init … state), serialized — the
    /// order comes from `botrail_session::initial_messages`, the single
    /// definition shared with the wasm host.
    pub fn handshake_jsons(&self) -> Vec<String> {
        botrail_session::initial_messages(self)
            .iter()
            .map(|msg| serde_json::to_string(msg).expect("wire types serialize infallibly"))
            .collect()
    }

    /// Directory `/usd-assets/{robot}/*` serves (that robot's stage
    /// directory, so relative references inside the stage resolve).
    pub fn robot_asset_dir(&self, robot: usize) -> Option<PathBuf> {
        self.with_scene(|scene| {
            let (path, _) = scene.robots().get(robot)?.model.source.usd_stage()?;
            path.parent().map(|p| p.to_path_buf())
        })
    }

    pub fn state_json(&self) -> String {
        let msg = self.with_scene(|scene| wire::state_message(scene));
        serde_json::to_string(&msg).expect("wire types serialize infallibly")
    }

    // --------------------------------------------------------------- robots

    /// Resolves an optional robot instance name to its index. `None` means
    /// the sole robot and is ambiguous (an error) when several exist.
    pub fn robot_index(&self, robot: Option<&str>) -> Result<usize, String> {
        self.with_scene(|scene| {
            let names = || {
                scene
                    .robots()
                    .iter()
                    .map(|r| format!("{:?}", r.name))
                    .collect::<Vec<_>>()
                    .join(", ")
            };
            match robot {
                Some(name) => scene
                    .robot_index(name)
                    .ok_or_else(|| format!("unknown robot `{name}` (robots: {})", names())),
                None if scene.robots().len() == 1 => Ok(0),
                None if scene.robots().is_empty() => {
                    Err("scene has no robot; add one with add_robot".to_string())
                }
                None => Err(format!(
                    "scene has {} robots; pass robot=<name> (one of: {})",
                    scene.robots().len(),
                    names()
                )),
            }
        })
    }

    pub fn robot_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.robots().iter().map(|r| r.name.clone()).collect())
    }

    pub fn robot_model(&self, robot: usize) -> std::sync::Arc<botrail_model::RobotModel> {
        self.with_scene(|scene| scene.robots()[robot].model.clone())
    }

    /// Adds a robot instance and re-runs the handshake broadcast — the
    /// robot roster lives in `scene_init`, and that message resets the
    /// studio store, so the full content refresh must follow it.
    pub fn add_robot(
        &self,
        model: std::sync::Arc<botrail_model::RobotModel>,
        name: Option<&str>,
        base: Isometry3<f64>,
    ) -> String {
        let final_name = self.with_scene(|scene| scene.add_robot(model, name, base));
        self.broadcast_handshake();
        final_name
    }

    fn broadcast_handshake(&self) {
        for msg in botrail_session::initial_messages(self) {
            self.emit(&msg);
        }
    }

    pub fn set_joint_positions_for(
        &self,
        robot: usize,
        positions: Vec<f64>,
    ) -> Result<(), SceneError> {
        botrail_session::set_joint_positions_for(self, robot, positions)
    }

    pub fn joint_positions(&self) -> Vec<f64> {
        self.with_scene(|scene| scene.joint_positions().to_vec())
    }

    pub fn joint_positions_for(&self, robot: usize) -> Vec<f64> {
        self.with_scene(|scene| scene.robots()[robot].joint_positions().to_vec())
    }

    pub fn link_pose_for(&self, robot: usize, link_name: &str) -> Option<PoseArrays> {
        self.with_scene(|scene| {
            let index = scene.robots()[robot].model.link_index(link_name)?;
            let pose = scene.link_poses_for(robot)[index];
            let t = pose.translation;
            let q = pose.rotation.coords;
            Some(([t.x, t.y, t.z], [q.x, q.y, q.z, q.w]))
        })
    }

    /// World pose of `link_name` of robot `robot` at configuration `q` —
    /// forward kinematics alone, the scene's joints untouched.
    pub fn link_pose_at(
        &self,
        robot: usize,
        link_name: &str,
        q: &[f64],
    ) -> Result<PoseArrays, String> {
        self.with_scene(|scene| {
            let index = scene.robots()[robot]
                .model
                .link_index(link_name)
                .ok_or_else(|| format!("unknown link `{link_name}`"))?;
            let poses = scene.fk_for(robot, q).map_err(|e| e.to_string())?;
            Ok(pose_arrays(&poses[index]))
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

    pub fn remove_frame(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_frame(self, name)
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
        self.robot_base_isometry_for(0)
    }

    pub fn robot_base_isometry_for(&self, robot: usize) -> Isometry3<f64> {
        self.with_scene(|scene| *scene.robots()[robot].base_pose())
    }

    pub fn set_robot_base_pose_for(&self, robot: usize, pose: Isometry3<f64>) {
        botrail_session::set_robot_base_pose_for(self, robot, pose);
    }

    /// Puts a robot on a vehicle; the base then follows it. Broadcasts the
    /// new state, since mounting moves the robot to the vehicle at once.
    /// Puts a robot on a vehicle, optionally with a gait; the offset
    /// defaults to the stance-derived one when a gait is given.
    pub fn mount_robot_with(
        &self,
        robot: usize,
        device: &str,
        offset: Option<Isometry3<f64>>,
        gait: Option<botrail_scene::seq::GaitSpec>,
        spin: Vec<(String, f64)>,
    ) -> Result<(), SceneError> {
        let result = self.with_scene(|scene| {
            scene.mount_robot_with(robot, device, offset, gait)?;
            if !spin.is_empty() {
                scene.set_mount_spin(robot, spin.clone())?;
            }
            Ok(())
        });
        if result.is_ok() {
            botrail_session::emit_state(self);
        }
        result
    }

    pub fn set_tcp_target_for(
        &self,
        robot: usize,
        link: &str,
        pose: &PoseMsg,
        options: &IkOptions,
    ) -> Result<botrail_kin::IkResult, String> {
        botrail_session::set_tcp_target_for(self, robot, link, pose, options)
    }

    // ------------------------------------------------------------ obstacles

    pub fn add_obstacle(
        &self,
        name: &str,
        geometry: Geometry,
        pose: Isometry3<f64>,
    ) -> Result<String, SceneError> {
        botrail_session::add_obstacle(self, name, geometry, pose)
    }

    /// Adds many obstacles with one broadcast; returns the final names.
    pub fn add_obstacles(&self, batch: Vec<ObstacleSpec>) -> Result<Vec<String>, SceneError> {
        botrail_session::add_obstacles(self, batch)
    }

    pub fn remove_obstacle(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_obstacle(self, name)
    }

    pub fn add_carve_stages(
        &self,
        stages: Vec<(String, Geometry, botrail_scene::ObstacleCollider)>,
        pose: Isometry3<f64>,
        material: botrail_scene::Material,
    ) -> Vec<String> {
        botrail_session::add_carve_stages(self, stages, pose, material)
    }

    pub fn add_display_stages(
        &self,
        stages: Vec<(String, Geometry, botrail_scene::ObstacleCollider)>,
        pose: Isometry3<f64>,
        material: botrail_scene::Material,
        legend: Option<botrail_scene::Legend>,
    ) -> Vec<String> {
        botrail_session::add_display_stages(self, stages, pose, material, legend)
    }

    pub fn emit_timeline(
        &self,
        scene: &Scene,
        timeline: &botrail_scene::rollout::SequenceTimeline,
    ) {
        botrail_session::emit_timeline(self, scene, timeline);
    }

    #[allow(clippy::too_many_arguments)]
    pub fn show_display_mesh(
        &self,
        name: &str,
        geometry: Geometry,
        pose: Isometry3<f64>,
        collider: botrail_scene::ObstacleCollider,
        material: botrail_scene::Material,
        legend: Option<botrail_scene::Legend>,
        hides: Option<&str>,
    ) -> Result<String, SceneError> {
        botrail_session::show_display_mesh(
            self, name, geometry, pose, collider, material, legend, hides,
        )
    }

    pub fn set_obstacle_enabled(&self, name: &str, enabled: bool) -> Result<(), SceneError> {
        botrail_session::set_obstacle_enabled(self, name, enabled)
    }

    pub fn set_obstacle_color(
        &self,
        name: &str,
        color: Option<[f32; 3]>,
    ) -> Result<(), SceneError> {
        botrail_session::set_obstacle_color(self, name, color)
    }

    pub fn set_obstacle_visible(&self, name: &str, visible: bool) -> Result<(), SceneError> {
        botrail_session::set_obstacle_visible(self, name, visible)
    }

    pub fn set_obstacle_walkable(&self, name: &str, walkable: bool) -> Result<(), SceneError> {
        botrail_session::set_obstacle_walkable(self, name, walkable)
    }

    pub fn set_obstacle_material(
        &self,
        name: &str,
        material: Option<botrail_scene::Material>,
    ) -> Result<(), SceneError> {
        botrail_session::set_obstacle_material(self, name, material)
    }

    pub fn set_obstacle_legend(
        &self,
        name: &str,
        legend: Option<botrail_scene::Legend>,
    ) -> Result<(), SceneError> {
        botrail_session::set_obstacle_legend(self, name, legend)
    }

    pub fn set_obstacle_pose(&self, name: &str, pose: Isometry3<f64>) -> Result<(), SceneError> {
        botrail_session::set_obstacle_pose(self, name, pose)
    }

    /// `(metalness, roughness)` of an obstacle, when one was authored.
    pub fn obstacle_material(&self, name: &str) -> Result<Option<(f32, f32)>, SceneError> {
        self.with_scene(|scene| {
            let obstacle = scene
                .obstacles()
                .iter()
                .find(|o| o.name == name)
                .ok_or_else(|| SceneError::UnknownObstacle(name.to_string()))?;
            Ok(obstacle.material.map(|m| (m.metalness, m.roughness)))
        })
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

    pub fn obstacle_bounds(&self, name: &str) -> Option<([f64; 3], [f64; 3])> {
        self.with_scene(|scene| scene.obstacle_bounds(name))
    }

    /// Renames a robot instance; the new name is uniquified against the
    /// others. Returns the name it actually got.
    pub fn rename_robot(&self, robot: usize, name: &str) -> String {
        botrail_session::rename_robot(self, robot, name)
    }

    /// Excuses a link pair of two *different* robots from collision
    /// checking (shared mount plates, deliberately touching fixtures).
    /// Names are resolved against the scene; `Err` describes what was not
    /// found.
    pub fn allow_inter_robot_collision(
        &self,
        robot_a: &str,
        link_a: &str,
        robot_b: &str,
        link_b: &str,
    ) -> Result<(), String> {
        let resolved = self.with_scene(|scene| {
            let robot = |name: &str| {
                scene.robot_index(name).ok_or_else(|| {
                    let names: Vec<_> = scene.robots().iter().map(|r| &r.name).collect();
                    format!("unknown robot `{name}` (robots: {names:?})")
                })
            };
            let (ia, ib) = (robot(robot_a)?, robot(robot_b)?);
            if ia == ib {
                return Err(format!(
                    "`{robot_a}` is on both sides: this excuses a pair of *different* robots, \
                     and a robot's own links are governed by its self-collision matrix"
                ));
            }
            let link = |r: usize, name: &str| {
                scene.robots()[r].model.link_index(name).ok_or_else(|| {
                    format!("robot `{}` has no link `{name}`", scene.robots()[r].name)
                })
            };
            Ok(((ia, link(ia, link_a)?), (ib, link(ib, link_b)?)))
        })?;
        botrail_session::allow_inter_robot_collision(self, resolved.0, resolved.1);
        Ok(())
    }

    /// `None` for an unknown obstacle; `Some(None)` for one with no colour.
    pub fn obstacle_color(&self, name: &str) -> Option<Option<[f32; 3]>> {
        self.with_scene(|scene| {
            let o = scene.obstacles().iter().find(|o| o.name == name)?;
            Some(o.color)
        })
    }

    pub fn attach_obstacle_to(
        &self,
        robot: usize,
        name: &str,
        link: Option<&str>,
        touch_links: Option<&[String]>,
    ) -> Result<(), SceneError> {
        botrail_session::attach_obstacle_to(self, robot, name, link, touch_links)
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
                .map(|a| {
                    (
                        a.object.clone(),
                        scene.robots()[a.robot].model.links[a.link].name.clone(),
                    )
                })
                .collect()
        })
    }

    // ----------------------------------------------------- allowed contacts

    pub fn allow_link_obstacle_contact(
        &self,
        robot: usize,
        link: usize,
        obstacle: &str,
    ) -> Result<(), SceneError> {
        self.with_scene(|scene| scene.allow_link_obstacle_contact(robot, link, obstacle))
    }

    pub fn disallow_link_obstacle_contact(
        &self,
        robot: usize,
        link: usize,
        obstacle: &str,
    ) -> bool {
        self.with_scene(|scene| scene.disallow_link_obstacle_contact(robot, link, obstacle))
    }

    // ------------------------------------------------------------ toolpaths

    pub fn add_toolpath(&self, tp: botrail_scene::toolpath::Toolpath) {
        botrail_session::upsert_toolpath(self, tp);
    }

    // Applicators and brushes are authoring data the studio does not (yet)
    // draw, so they go straight to the scene: no broadcast.
    pub fn define_applicator(
        &self,
        name: &str,
        applicator: botrail_scene::coat::Applicator,
    ) -> Result<(), String> {
        self.with_scene(|scene| {
            scene
                .define_applicator(name, applicator)
                .map_err(|e| e.to_string())
        })
    }

    pub fn define_brush(&self, brush: botrail_scene::coat::Brush) -> Result<(), String> {
        self.with_scene(|scene| scene.define_brush(brush).map_err(|e| e.to_string()))
    }

    pub fn remove_applicator(&self, name: &str) -> bool {
        self.with_scene(|scene| scene.remove_applicator(name))
    }

    pub fn remove_brush(&self, name: &str) -> bool {
        self.with_scene(|scene| scene.remove_brush(name))
    }

    pub fn applicator_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.applicators().iter().map(|(n, _)| n.clone()).collect())
    }

    pub fn brush_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.brushes().iter().map(|b| b.name.clone()).collect())
    }

    pub fn brush(&self, name: &str) -> Option<botrail_scene::coat::Brush> {
        self.with_scene(|scene| scene.brush(name).cloned())
    }

    pub fn remove_toolpath(&self, name: &str) -> bool {
        botrail_session::remove_toolpath(self, name)
    }

    pub fn set_toolpath_marks(
        &self,
        name: &str,
        marks: Vec<botrail_scene::PathMark>,
    ) -> Result<(), String> {
        botrail_session::set_toolpath_marks(self, name, marks)
    }

    pub fn toolpath_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.toolpaths().iter().map(|t| t.name.clone()).collect())
    }

    pub fn plan_toolpath(
        &self,
        name: &str,
        robot: usize,
        tcp: Option<usize>,
        options: &botrail_scene::toolpath::ToolpathOptions,
    ) -> Result<botrail_scene::toolpath::PlannedToolpath, String> {
        self.with_scene(|scene| {
            scene
                .plan_toolpath(name, robot, tcp, options)
                .map_err(|e| e.to_string())
        })
    }

    pub fn check_toolpath(
        &self,
        name: &str,
        robot: usize,
        tcp: Option<usize>,
        options: &botrail_scene::toolpath::ToolpathOptions,
    ) -> Result<botrail_scene::toolpath::ToolpathReport, String> {
        self.with_scene(|scene| {
            scene
                .check_toolpath(name, robot, tcp, options)
                .map_err(|e| e.to_string())
        })
    }

    pub fn check_paint(
        &self,
        name: &str,
        target: &str,
        limits: &botrail_scene::coat::PaintLimits,
        options: &botrail_scene::toolpath::ToolpathOptions,
    ) -> Result<botrail_scene::coat::PaintReport, String> {
        self.with_scene(|scene| {
            scene
                .check_paint(name, target, limits, options)
                .map_err(|e| e.to_string())
        })
    }

    /// Wraps a trajectory as a single-robot timeline plus the scene
    /// snapshot its consumers (USD export, clearance re-scan) need.
    pub fn timeline_from_trajectory(
        &self,
        robot: usize,
        trajectory: &botrail_traj::JointTrajectory,
        label: &str,
    ) -> (botrail_scene::rollout::SequenceTimeline, Scene) {
        self.with_scene(|scene| {
            (
                scene.timeline_from_trajectory(robot, trajectory, label),
                scene.clone(),
            )
        })
    }

    // ------------------------------------------------------------ sequences

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

    pub fn upsert_sensor(&self, sensor: botrail_scene::seq::Sensor) -> Result<(), SceneError> {
        botrail_session::upsert_sensor(self, sensor)
    }

    pub fn upsert_io_node(&self, node: botrail_scene::iomap::IoNode) -> Result<(), SceneError> {
        botrail_session::upsert_io_node(self, node)
    }

    pub fn remove_io_node(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_io_node(self, name)
    }

    pub fn bind_io(&self, binding: botrail_scene::iomap::IoBinding) -> Result<(), SceneError> {
        botrail_session::bind_io(self, binding)
    }

    pub fn unbind_io(
        &self,
        point: &botrail_scene::iomap::IoPointId,
        node: Option<&str>,
    ) -> Result<usize, SceneError> {
        botrail_session::unbind_io(self, point, node)
    }

    pub fn declare_io(&self, decl: botrail_scene::iomap::IoDecl) {
        botrail_session::declare_io(self, decl)
    }

    pub fn undeclare_io(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::undeclare_io(self, name)
    }

    pub fn io_map(&self) -> botrail_scene::iomap::IoMap {
        self.with_scene(|scene| scene.io_map().clone())
    }

    // ------------------------------------------------------------- parts

    pub fn set_part(
        &self,
        target: &str,
        kind: Option<botrail_scene::part::PartTargetKind>,
        part: botrail_scene::part::Part,
    ) -> Result<botrail_scene::part::PartTargetKind, SceneError> {
        botrail_session::set_part(self, target, kind, part)
    }

    pub fn remove_part(&self, target: &str) -> Result<(), SceneError> {
        botrail_session::remove_part(self, target)
    }

    pub fn part(&self, target: &str) -> Option<botrail_scene::part::PartEntry> {
        self.with_scene(|scene| scene.part(target).cloned())
    }

    pub fn parts(&self) -> Vec<botrail_scene::part::PartEntry> {
        self.with_scene(|scene| scene.parts().to_vec())
    }

    pub fn bom(&self) -> botrail_scene::part::Bom {
        self.with_scene(|scene| scene.bom())
    }

    pub fn auto_assign_io(
        &self,
        sequences: Option<&[&str]>,
        reassign: bool,
    ) -> Result<botrail_scene::iomap::IoReport, botrail_scene::iomap::IoError> {
        botrail_session::auto_assign_io(self, sequences, reassign)
    }

    pub fn remove_sensor(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_sensor(self, name)
    }

    pub fn sensor_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.sensors().iter().map(|s| s.name.clone()).collect())
    }

    pub fn upsert_camera(&self, camera: botrail_scene::seq::Camera) -> Result<(), SceneError> {
        botrail_session::upsert_camera(self, camera)
    }

    pub fn remove_camera(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_camera(self, name)
    }

    pub fn camera_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.cameras().iter().map(|c| c.name.clone()).collect())
    }

    pub fn add_scenario(&self, scenario: botrail_scene::seq::Scenario) -> Result<(), SceneError> {
        botrail_session::upsert_scenario(self, scenario)
    }

    pub fn remove_scenario(&self, name: &str) -> Result<(), SceneError> {
        botrail_session::remove_scenario(self, name)
    }

    pub fn scenario_names(&self) -> Vec<String> {
        self.with_scene(|scene| scene.scenarios().iter().map(|s| s.name.clone()).collect())
    }

    /// An authored-content snapshot (sequence definitions for coverage —
    /// scenario deltas never touch those, so any snapshot serves).
    pub fn authored_snapshot(&self) -> Scene {
        self.with_scene(|scene| scene.clone())
    }

    pub fn add_cut_trace(
        &self,
        name: &str,
        signal: &str,
        robot: &str,
        spin_link: Option<&str>,
    ) -> Result<(), SceneError> {
        botrail_session::add_cut_trace(self, name, signal, robot, spin_link)
    }

    pub fn add_weld_flash(&self, name: &str, signal: &str, robot: &str) -> Result<(), SceneError> {
        botrail_session::add_weld_flash(self, name, signal, robot)
    }

    pub fn add_spray_cone(
        &self,
        name: &str,
        signal: &str,
        robot: &str,
        length: f64,
        radius: f64,
    ) -> Result<(), SceneError> {
        botrail_session::add_spray_cone(self, name, signal, robot, length, radius)
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
        scenario: Option<&str>,
        options: &botrail_scene::rollout::RolloutOptions,
    ) -> Result<(botrail_scene::rollout::SequenceTimeline, Scene), String> {
        self.simulate_sequences(&[name], scenario, options)
    }

    /// Rolls out several sequences concurrently (one PLC scan advances
    /// every program, in list order); one timeline comes back. Under a
    /// scenario, the returned snapshot is the *applied* one — the world
    /// the timeline actually ran in, which is what its FK, USD export,
    /// and clearance checks must read.
    pub fn simulate_sequences(
        &self,
        names: &[&str],
        scenario: Option<&str>,
        options: &botrail_scene::rollout::RolloutOptions,
    ) -> Result<(botrail_scene::rollout::SequenceTimeline, Scene), String> {
        let scenario = scenario.filter(|s| *s != botrail_scene::seq::BASELINE_SCENARIO);
        let mut snapshot = self.snapshot();
        if let Some(scenario) = scenario {
            snapshot
                .apply_scenario(scenario)
                .map_err(|e| e.to_string())?;
        }
        let timeline =
            botrail_session::simulate_sequences_and_emit(self, names, scenario, options)?;
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
        robot: usize,
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
            let poses = scene
                .fk_for(robot, &traj.sample(t))
                .map_err(|e| e.to_string())?;
            link_poses.push(poses);
        }
        // Objects held by *this* robot ride its trajectory; everything else
        // (including other robots' cargo) stays at its current pose.
        let objects: Vec<botrail_usd::export::ObjectSpec> = scene
            .obstacles()
            .iter()
            // A collision proxy is not part of the picture: the export
            // is what someone opens in usdview, and hidden means hidden.
            .filter(|o| o.visible)
            .map(|o| {
                let track = match scene.attachment(&o.name).filter(|a| a.robot == robot) {
                    Some(att) => botrail_usd::export::PoseTrack::Sampled(
                        link_poses.iter().map(|p| p[att.link] * att.grasp).collect(),
                    ),
                    None => botrail_usd::export::PoseTrack::Static(o.pose),
                };
                botrail_usd::export::ObjectSpec {
                    name: o.name.clone(),
                    geometry: o.geometry.clone(),
                    track,
                    color: o.color,
                    visible: Vec::new(),
                }
            })
            .collect();
        let joint_samples: Vec<Vec<f64>> = times.iter().map(|&t| traj.sample(t)).collect();
        // A sole robot exports under the historical `Robot` prim (byte
        // compatibility); named instances appear once several exist.
        let name = if scene.robots().len() == 1 {
            "Robot".to_string()
        } else {
            scene.robots()[robot].name.clone()
        };
        let robots = [botrail_usd::export::RobotAnimation {
            name: &name,
            model: &scene.robots()[robot].model,
            link_poses: &link_poses,
            joint_samples: Some(&joint_samples),
        }];
        let input = botrail_usd::export::AnimationInput {
            robots: &robots,
            times: &times,
            objects: &objects,
            curves: &[],
            cameras: &[],
        };
        let options = botrail_usd::export::ExportOptions { fps };
        botrail_usd::export::write_animation(path, &input, &options).map_err(|e| e.to_string())
    }

    /// Writes the scene as it stands to a static USD layer at `path`:
    /// robots at their current joint positions, visible obstacles at
    /// their poses, toolpath overlays and cameras. Returns exporter
    /// warnings.
    pub fn export_scene_usd(&self, path: &std::path::Path) -> Result<Vec<String>, String> {
        let scene = self.snapshot();
        let stem = path
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "scene".to_string());
        let exported = botrail_session::usd::bake_scene(&scene, &stem)?;
        botrail_usd::export::write_exported(path, exported).map_err(|e| e.to_string())
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
        robot_roots: Vec<(String, String)>,
    ) -> Result<(String, f64, Vec<String>, Vec<String>), String> {
        let scene = self.snapshot();
        // Only what the scene draws. A recording is the animation of the
        // visible world — `export_usd` omits invisible obstacles — so
        // asking for a prim for a hidden collision proxy is asking for a
        // warning per proxy, and a workpiece ships dozens of them.
        let obstacle_names: Vec<String> = scene
            .obstacles()
            .iter()
            .filter(|o| o.visible)
            .map(|o| o.name.clone())
            .collect();
        let robots: Vec<(String, &botrail_model::RobotModel)> = scene
            .robots()
            .iter()
            .map(|r| (r.name.clone(), r.model.as_ref()))
            .collect();
        let options = botrail_usd::recording::RecordingImportOptions {
            search_paths: Vec::new(),
            force_transforms,
            robot_roots,
        };
        let source = path.display().to_string();
        match botrail_usd::recording::import_recording(path, &robots, &obstacle_names, &options) {
            Ok(rec) => {
                use botrail_usd::recording::RecordingMode;
                // The reported mode summarizes all robots.
                let mode = match (
                    rec.robots
                        .iter()
                        .all(|r| r.mode == RecordingMode::JointState),
                    rec.robots
                        .iter()
                        .all(|r| r.mode == RecordingMode::Transforms),
                ) {
                    (true, _) => "joint_state".to_string(),
                    (_, true) => "transforms".to_string(),
                    _ => "mixed".to_string(),
                };
                let duration = rec.times.last().copied().unwrap_or(0.0);
                let track_names: Vec<String> =
                    rec.object_tracks.iter().map(|(n, ..)| n.clone()).collect();
                let timeline = wire::TimelineMsg {
                    duration,
                    // Recordings replay captured link/object poses; the
                    // vehicle frame was not captured, so mounted sensors
                    // draw at the parked frame here.
                    vehicles: Vec::new(),
                    robots: rec
                        .robots
                        .iter()
                        .enumerate()
                        .map(|(i, r)| wire::RobotTimelineMsg {
                            // Joint values alone cannot place a robot whose
                            // base moved — a recorded AMR would replay with
                            // its arm left behind at the parked base. The
                            // root link's recorded pose *is* the base, so
                            // send it whenever it actually moves.
                            base: {
                                let root = scene.robots()[i].model.root_link;
                                let first = r.link_poses.first().map(|f| f[root]);
                                let moves = first.is_some_and(|a| {
                                    r.link_poses.iter().any(|f| {
                                        (f[root].translation.vector - a.translation.vector).norm()
                                            > 1e-9
                                            || f[root].rotation.angle_to(&a.rotation) > 1e-9
                                    })
                                });
                                if moves && r.mode == RecordingMode::JointState {
                                    r.link_poses
                                        .iter()
                                        .map(|f| wire::PoseMsg::from(&f[root]))
                                        .collect()
                                } else {
                                    Vec::new()
                                }
                            },
                            name: r.name.clone(),
                            trajectory: wire::TrajectoryMsg {
                                duration,
                                times: rec.times.clone(),
                                joint_positions: r.joint_samples.clone().unwrap_or_default(),
                                link_poses: match r.mode {
                                    // Joint mode rides the joint-playback path.
                                    RecordingMode::JointState => None,
                                    RecordingMode::Transforms => Some(
                                        r.link_poses
                                            .iter()
                                            .map(|frame| {
                                                frame.iter().map(wire::PoseMsg::from).collect()
                                            })
                                            .collect(),
                                    ),
                                },
                                object_tracks: None,
                            },
                            moves: Vec::new(),
                        })
                        .collect(),
                    objects: rec
                        .object_tracks
                        .iter()
                        .map(|(name, poses, visible)| wire::ObjectTrackMsg {
                            name: name.clone(),
                            poses: {
                                let mut poses: Vec<wire::PoseMsg> =
                                    poses.iter().map(wire::PoseMsg::from).collect();
                                // A blink-only track collapses to its one
                                // pose, same as the baked-timeline path.
                                if poses.len() > 1 && poses.iter().all(|p| *p == poses[0]) {
                                    poses.truncate(1);
                                }
                                poses
                            },
                            // Carve stages and stowed parts animate by
                            // visibility; an always-shown track stays lean.
                            visible: if visible.iter().all(|v| *v) {
                                Vec::new()
                            } else {
                                visible.clone()
                            },
                        })
                        .collect(),
                    step_spans: Vec::new(),
                    signals: Vec::new(),
                    branches: Vec::new(),
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
                botrail_collide::ColliderId::Link { robot, link } => (
                    "link".to_string(),
                    scene.robots()[robot].model.links[link].name.clone(),
                ),
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

    pub fn add_segment_for(
        &self,
        robot: usize,
        motion: &str,
        segment: botrail_scene::motion::Segment,
    ) -> Result<(), String> {
        botrail_session::add_segment_for(self, robot, motion, segment)
    }

    pub fn remove_segment(&self, motion: &str, index: usize) -> Result<(), String> {
        botrail_session::remove_segment(self, motion, index)
    }

    pub fn clear_motion(&self, motion: &str) -> Result<(), String> {
        botrail_session::clear_motion(self, motion)
    }

    /// The owning robot index of a named motion.
    pub fn motion_owner(&self, name: &str) -> Option<usize> {
        self.with_scene(|scene| {
            scene
                .motions()
                .iter()
                .find(|m| m.name == name)
                .map(|m| m.robot)
        })
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
        // Full-content refresh; the shared definition keeps this in lockstep
        // with the handshake (scene_init is skipped — a project load cannot
        // change the robot).
        for msg in botrail_session::refresh_messages(self) {
            self.emit(&msg);
        }
        Ok(())
    }

    pub fn python_code(&self) -> String {
        self.with_scene(|scene| botrail_scene::project::generate_python(&scene.to_project()))
    }

    // ------------------------------------------------------------- planning

    pub fn plan_to_for(
        &self,
        robot: usize,
        goal: &[f64],
        options: &botrail_plan::PlanOptions,
    ) -> Result<(botrail_traj::JointTrajectory, Vec<Vec<f64>>, f64), String> {
        botrail_session::plan_to_for(self, robot, goal, options)
    }

    pub fn plan_and_broadcast_for(
        &self,
        robot: usize,
        goal: &[f64],
        options: &botrail_plan::PlanOptions,
    ) -> Result<(botrail_traj::JointTrajectory, Vec<Vec<f64>>, f64), String> {
        botrail_session::plan_and_emit_for(self, robot, goal, options)
    }

    pub fn handle_client_message(&self, text: &str) {
        botrail_session::handle_client_message(self, text);
    }
}
