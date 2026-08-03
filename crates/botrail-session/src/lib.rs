//! Shared session logic: the wire-protocol dispatch and planning helpers
//! used by both the Python server hub (botrail-py) and the browser session
//! (botrail-wasm).
//!
//! The two environments differ only in plumbing, which the host supplies
//! through [`SessionHost`]: how the scene is accessed (mutex vs RefCell),
//! where outgoing messages go (websocket broadcast vs a collected Vec), the
//! wall clock (Instant vs Date.now), and logging. Everything protocol- or
//! planning-shaped lives here, once.

use std::path::Path;

use botrail_kin::{IkMode, IkOptions, IkResult};
use botrail_model::Geometry;
use botrail_scene::motion::{PlannedMotion, Segment};
use botrail_scene::wire::{
    self, ClientMessage, IkStatusMsg, PoseMsg, SceneDescriptionMsg, ServerMessage,
};
use botrail_scene::{ObstacleSpec, Scene, SceneError};
use nalgebra::Isometry3;

/// Environment plumbing a session runs on.
pub trait SessionHost {
    /// Exclusive scene access. Implementations hold a lock (or borrow) for
    /// the duration of `f`, so keep the work brief — long-running planning
    /// goes through [`snapshot`](Self::snapshot) instead.
    fn with_scene<R>(&self, f: impl FnOnce(&mut Scene) -> R) -> R;

    /// Maps a mesh file path to the `(url, extension)` pair clients fetch
    /// the visual from. Hosts without mesh serving keep the default empty
    /// mapping (clients skip such visuals).
    fn mesh_url(&self, _path: &Path) -> (String, String) {
        (String::new(), String::new())
    }

    /// URL the client can fetch a USD-sourced robot's stage from (relative
    /// references must resolve against it); `robot` is the scene index of
    /// the robot the stage belongs to. `None` (default) keeps the client on
    /// the legacy link-visual rendering path.
    fn robot_asset_url(&self, _robot: usize, _path: &Path) -> Option<String> {
        None
    }

    /// Sends one server message to the connected client(s).
    fn emit(&self, msg: &ServerMessage);

    /// Wall clock in milliseconds, for planning-time stats.
    fn now_ms(&self) -> f64;

    /// Reports a rejected client message or failed operation.
    fn log(&self, message: &str);

    /// Scene snapshot that planning runs against, so the live scene stays
    /// accessible while a plan is in flight.
    fn snapshot(&self) -> Scene {
        self.with_scene(|scene| scene.clone())
    }
}

/// The connection handshake, in order: scene_init, obstacles, motions,
/// sequences, sensors, devices, frames, state. Mesh visuals are mapped to
/// URLs through the host's [`mesh_url`](SessionHost::mesh_url). This is the
/// single definition of the handshake — hosts must not hand-roll it.
pub fn initial_messages(host: &impl SessionHost) -> Vec<ServerMessage> {
    let mut messages = vec![host.with_scene(|scene| scene_init_message(host, scene))];
    messages.extend(refresh_messages(host));
    messages
}

/// Every scene-content message except `scene_init`, in handshake order:
/// obstacles, motions, sequences, sensors, devices, frames, state. Re-sent
/// wholesale after bulk changes (project load), where the robot — and
/// therefore `scene_init` — cannot change.
pub fn refresh_messages(host: &impl SessionHost) -> Vec<ServerMessage> {
    host.with_scene(|scene| {
        vec![
            wire::obstacles_message(scene, |p| host.mesh_url(p)),
            wire::motions_message(scene),
            wire::sequences_message(scene),
            wire::sensors_message(scene),
            wire::devices_message(scene),
            wire::frames_message(scene),
            wire::state_message(scene),
        ]
    })
}

/// The `scene_init` message, with mesh/asset URLs mapped through the host.
pub fn scene_init_message(host: &impl SessionHost, scene: &Scene) -> ServerMessage {
    let usd_asset = |robot: usize| match &scene.robots()[robot].model.source {
        botrail_model::RobotSource::Usd {
            path,
            articulation_root,
        } => host
            .robot_asset_url(robot, path)
            .map(|url| wire::UsdAssetMsg {
                url,
                articulation_root: articulation_root.clone(),
            }),
        _ => None,
    };
    ServerMessage::SceneInit {
        scene: SceneDescriptionMsg::from_scene(scene, |p| host.mesh_url(p), usd_asset),
    }
}

/// Handles one raw client message, emitting whatever a server should
/// broadcast in response. Rejections are logged, never fatal.
pub fn handle_client_message(host: &impl SessionHost, text: &str) {
    let msg: ClientMessage = match serde_json::from_str(text) {
        Ok(msg) => msg,
        Err(e) => return host.log(&format!("unparseable client message: {e}")),
    };
    if let Err(e) = dispatch(host, msg) {
        host.log(&e);
    }
}

/// Resolves a wire robot reference: an instance name, or `None` for the
/// first robot (pre-multi-robot clients).
fn resolve_robot(host: &impl SessionHost, robot: &Option<String>) -> Result<usize, String> {
    match robot {
        Some(name) => host.with_scene(|scene| {
            scene
                .robot_index(name)
                .ok_or_else(|| format!("unknown robot `{name}`"))
        }),
        None => Ok(0),
    }
}

fn dispatch(host: &impl SessionHost, msg: ClientMessage) -> Result<(), String> {
    match msg {
        ClientMessage::SetJointPositions { robot, positions } => resolve_robot(host, &robot)
            .and_then(|robot| {
                set_joint_positions_for(host, robot, positions).map_err(|e| e.to_string())
            })
            .map_err(|e| format!("rejected set_joint_positions: {e}")),
        ClientMessage::SetTcpTarget { robot, link, pose } => {
            // Warm-seeded streaming solve: the gizmo sends targets at
            // ~60Hz, so a few iterations per message are enough.
            let options = IkOptions {
                mode: IkMode::Pose,
                ..IkOptions::streaming()
            };
            resolve_robot(host, &robot)
                .and_then(|robot| {
                    set_tcp_target_for(host, robot, &link, &pose, &options).map(|_| ())
                })
                .map_err(|e| format!("rejected tcp target: {e}"))
        }
        ClientMessage::SetRobotBasePose { robot, pose } => {
            let robot = resolve_robot(host, &robot)
                .map_err(|e| format!("rejected set_robot_base_pose: {e}"))?;
            set_robot_base_pose_for(host, robot, (&pose).into());
            Ok(())
        }
        ClientMessage::AddObstacle { obstacle } => wire::geometry_from_msg(&obstacle.geometry)
            .and_then(|geometry| {
                add_obstacle(host, &obstacle.name, geometry, (&obstacle.pose).into())
                    .map(|_| ())
                    .map_err(|e| e.to_string())
            })
            .map_err(|e| format!("rejected add_obstacle: {e}")),
        ClientMessage::UpdateObstaclePose { name, pose } => {
            set_obstacle_pose(host, &name, (&pose).into())
                .map_err(|e| format!("rejected update_obstacle_pose: {e}"))
        }
        ClientMessage::UpdatePoses { obstacles, frames } => {
            update_poses(host, obstacles, frames).map_err(|e| format!("rejected update_poses: {e}"))
        }
        ClientMessage::UpdateObstacleGeometry { name, geometry } => {
            wire::geometry_from_msg(&geometry)
                .and_then(|geometry| {
                    set_obstacle_geometry(host, &name, geometry).map_err(|e| e.to_string())
                })
                .map_err(|e| format!("rejected update_obstacle_geometry: {e}"))
        }
        ClientMessage::RemoveObstacle { name } => {
            remove_obstacle(host, &name).map_err(|e| format!("rejected remove_obstacle: {e}"))
        }
        ClientMessage::SetObstacleEnabled { name, enabled } => {
            set_obstacle_enabled(host, &name, enabled)
                .map_err(|e| format!("rejected set_obstacle_enabled: {e}"))
        }
        ClientMessage::AttachObstacle {
            name,
            robot,
            link,
            touch_links,
        } => resolve_robot(host, &robot)
            .and_then(|robot| {
                attach_obstacle_to(host, robot, &name, link.as_deref(), touch_links.as_deref())
                    .map_err(|e| e.to_string())
            })
            .map_err(|e| format!("rejected attach_obstacle: {e}")),
        ClientMessage::DetachObstacle { name } => {
            detach_obstacle(host, &name).map_err(|e| format!("rejected detach_obstacle: {e}"))
        }
        ClientMessage::PlanRequest {
            robot,
            goal_positions,
        } => {
            let robot =
                resolve_robot(host, &robot).map_err(|e| format!("rejected plan_request: {e}"))?;
            // Failure is reported to clients inside the plan_result.
            let _ = plan_and_emit_for(
                host,
                robot,
                &goal_positions,
                &botrail_plan::PlanOptions::default(),
            );
            Ok(())
        }
        ClientMessage::AddSegment {
            motion,
            robot,
            segment,
        } => resolve_robot(host, &robot)
            .and_then(|robot| {
                add_segment_for(host, robot, &motion, wire::segment_from_msg(&segment))
            })
            .map_err(|e| format!("rejected add_segment: {e}")),
        ClientMessage::RemoveSegment { motion, index } => remove_segment(host, &motion, index)
            .map_err(|e| format!("rejected remove_segment: {e}")),
        ClientMessage::ClearMotion { motion } => {
            clear_motion(host, &motion).map_err(|e| format!("rejected clear_motion: {e}"))
        }
        ClientMessage::PlanMotion { motion } => {
            // Failure is reported to clients inside the motion_result.
            let _ = plan_motion_and_emit(host, &motion, &botrail_plan::PlanOptions::default());
            Ok(())
        }
        ClientMessage::UpsertSequence { sequence } => {
            upsert_sequence(host, wire::sequence_from_msg(&sequence));
            Ok(())
        }
        ClientMessage::RemoveSequence { name } => {
            remove_sequence(host, &name).map_err(|e| format!("rejected remove_sequence: {e}"))
        }
        ClientMessage::DefineSignal { name, initial } => {
            define_signal(host, &name, initial);
            Ok(())
        }
        ClientMessage::RemoveSignal { name } => {
            remove_signal(host, &name).map_err(|e| format!("rejected remove_signal: {e}"))
        }
        ClientMessage::SimulateSequence { name } => {
            // Failure is reported to clients inside the sequence_result.
            let _ = simulate_sequence_and_emit(
                host,
                &name,
                &botrail_scene::rollout::RolloutOptions::default(),
            );
            Ok(())
        }
        ClientMessage::UpsertSensor { sensor } => {
            upsert_sensor(host, wire::sensor_from_msg(&sensor));
            Ok(())
        }
        ClientMessage::RemoveSensor { name } => {
            remove_sensor(host, &name).map_err(|e| format!("rejected remove_sensor: {e}"))
        }
        ClientMessage::UpsertDevice { device } => {
            upsert_device(host, wire::device_from_msg(&device));
            Ok(())
        }
        ClientMessage::RemoveDevice { name } => {
            remove_device(host, &name).map_err(|e| format!("rejected remove_device: {e}"))
        }
    }
}

// ---------------------------------------------------------------- state

pub fn emit_state(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::state_message(scene));
    host.emit(&msg);
}

/// Robot motion drags attached (grasped) obstacles along; clients learn
/// obstacle poses only from `obstacles` broadcasts, so any joint/base
/// change must rebroadcast the list while something is attached.
fn emit_obstacles_if_attached(host: &impl SessionHost) {
    let attached = host.with_scene(|scene| !scene.attachments().is_empty());
    if attached {
        let msg = host.with_scene(|scene| wire::obstacles_message(scene, |p| host.mesh_url(p)));
        host.emit(&msg);
    }
}

pub fn set_joint_positions_for(
    host: &impl SessionHost,
    robot: usize,
    positions: Vec<f64>,
) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.set_joint_positions_for(robot, positions))?;
    emit_obstacles_if_attached(host);
    emit_state(host);
    Ok(())
}

pub fn set_robot_base_pose_for(host: &impl SessionHost, robot: usize, pose: Isometry3<f64>) {
    host.with_scene(|scene| scene.set_robot_base_pose_for(robot, pose));
    emit_obstacles_if_attached(host);
    emit_state(host);
}

pub fn set_tcp_target_for(
    host: &impl SessionHost,
    robot: usize,
    link: &str,
    pose: &PoseMsg,
    options: &IkOptions,
) -> Result<IkResult, String> {
    let (result, state) = host.with_scene(|scene| -> Result<_, String> {
        let index = scene.robots()[robot]
            .model
            .link_index(link)
            .ok_or_else(|| format!("unknown link `{link}`"))?;
        let target: Isometry3<f64> = pose.into();
        let seed = scene.robots()[robot].joint_positions().to_vec();
        let result = scene
            .solve_ik_world_for(robot, index, &target, &seed, options)
            .map_err(|e| e.to_string())?;
        scene
            .set_joint_positions_for(robot, result.q.clone())
            .map_err(|e| e.to_string())?;
        let status = IkStatusMsg {
            converged: result.converged,
            pos_error: result.pos_error,
            rot_error: result.rot_error,
        };
        Ok((
            result,
            wire::state_message_with_ik(scene, Some((robot, status))),
        ))
    })?;
    emit_obstacles_if_attached(host);
    host.emit(&state);
    Ok(result)
}

// ------------------------------------------------------------ obstacles

fn emit_obstacles_and_state(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::obstacles_message(scene, |p| host.mesh_url(p)));
    host.emit(&msg);
    emit_state(host);
}

/// Adds an obstacle and returns its (possibly uniquified) name.
pub fn add_obstacle(
    host: &impl SessionHost,
    name: &str,
    geometry: Geometry,
    pose: Isometry3<f64>,
) -> Result<String, SceneError> {
    let final_name = host.with_scene(|scene| scene.add_obstacle(name, geometry, pose))?;
    emit_obstacles_and_state(host);
    Ok(final_name)
}

/// Adds a batch of obstacles with a single obstacles/state emission
/// (importers add tens to hundreds at once). Returns the final names.
pub fn add_obstacles(
    host: &impl SessionHost,
    batch: Vec<ObstacleSpec>,
) -> Result<Vec<String>, SceneError> {
    let names = host.with_scene(|scene| scene.add_obstacles(batch))?;
    emit_obstacles_and_state(host);
    Ok(names)
}

pub fn remove_obstacle(host: &impl SessionHost, name: &str) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.remove_obstacle(name))?;
    emit_obstacles_and_state(host);
    Ok(())
}

pub fn set_obstacle_pose(
    host: &impl SessionHost,
    name: &str,
    pose: Isometry3<f64>,
) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.set_obstacle_pose(name, pose))?;
    emit_obstacles_and_state(host);
    Ok(())
}

/// Includes/excludes an obstacle from collision checking.
pub fn set_obstacle_enabled(
    host: &impl SessionHost,
    name: &str,
    enabled: bool,
) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.set_obstacle_enabled(name, enabled))?;
    emit_obstacles_and_state(host);
    Ok(())
}

pub fn set_obstacle_color(
    host: &impl SessionHost,
    name: &str,
    color: Option<[f32; 3]>,
) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.set_obstacle_color(name, color))?;
    emit_obstacles_and_state(host);
    Ok(())
}

/// Writes a batch of obstacle (and frame) poses with a single broadcast.
/// Nothing is applied unless every name resolves, so a group drag cannot
/// half-move a subtree.
pub fn update_poses(
    host: &impl SessionHost,
    obstacles: Vec<(String, PoseMsg)>,
    frames: Vec<(String, PoseMsg)>,
) -> Result<(), SceneError> {
    host.with_scene(|scene| {
        for (name, _) in &obstacles {
            if !scene.obstacles().iter().any(|o| &o.name == name) {
                return Err(SceneError::UnknownObstacle(name.clone()));
            }
        }
        for (name, pose) in &obstacles {
            scene.set_obstacle_pose(name, pose.into())?;
        }
        for (name, pose) in &frames {
            scene.add_frame(name, pose.into());
        }
        Ok::<(), SceneError>(())
    })?;
    emit_obstacles_and_state(host);
    host.emit(&host.with_scene(|scene| wire::frames_message(scene)));
    Ok(())
}

/// Renames a robot instance, returning the name it actually got. The
/// roster lives in `scene_init`, and that message resets the client store,
/// so the whole handshake follows it.
pub fn rename_robot(host: &impl SessionHost, robot: usize, name: &str) -> String {
    let final_name = host.with_scene(|scene| scene.rename_robot(robot, name));
    for msg in initial_messages(host) {
        host.emit(&msg);
    }
    final_name
}

/// Excuses a link pair of two different robots from collision checking.
/// `state` carries the collision list, so clients need it resent.
pub fn allow_inter_robot_collision(host: &impl SessionHost, a: (usize, usize), b: (usize, usize)) {
    host.with_scene(|scene| scene.allow_inter_robot_collision(a, b));
    emit_state(host);
}

pub fn set_obstacle_geometry(
    host: &impl SessionHost,
    name: &str,
    geometry: Geometry,
) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.set_obstacle_geometry(name, geometry))?;
    emit_obstacles_and_state(host);
    Ok(())
}

pub fn attach_obstacle_to(
    host: &impl SessionHost,
    robot: usize,
    name: &str,
    link: Option<&str>,
    touch_links: Option<&[String]>,
) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.attach_obstacle_to(robot, name, link, touch_links))?;
    emit_obstacles_and_state(host);
    Ok(())
}

/// Detaches an obstacle (its pose freezes where the robot holds it) and
/// rebroadcasts obstacles + state.
pub fn detach_obstacle(host: &impl SessionHost, name: &str) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.detach_obstacle(name))?;
    emit_obstacles_and_state(host);
    Ok(())
}

// --------------------------------------------------------------- frames

fn emit_frames(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::frames_message(scene));
    host.emit(&msg);
}

/// Adds/updates named world frames and rebroadcasts the frame list.
pub fn add_frames(host: &impl SessionHost, frames: Vec<(String, Isometry3<f64>)>) {
    host.with_scene(|scene| {
        for (name, pose) in frames {
            scene.add_frame(&name, pose);
        }
    });
    emit_frames(host);
}

// -------------------------------------------------------------- motions

fn emit_motions(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::motions_message(scene));
    host.emit(&msg);
}

pub fn add_segment_for(
    host: &impl SessionHost,
    robot: usize,
    motion: &str,
    segment: Segment,
) -> Result<(), String> {
    host.with_scene(|scene| scene.add_segment_for(robot, motion, segment))
        .map_err(|e| e.to_string())?;
    emit_motions(host);
    Ok(())
}

pub fn remove_segment(host: &impl SessionHost, motion: &str, index: usize) -> Result<(), String> {
    host.with_scene(|scene| scene.remove_segment(motion, index))
        .map_err(|e| e.to_string())?;
    emit_motions(host);
    Ok(())
}

pub fn clear_motion(host: &impl SessionHost, motion: &str) -> Result<(), String> {
    host.with_scene(|scene| scene.clear_motion(motion))
        .map_err(|e| e.to_string())?;
    emit_motions(host);
    Ok(())
}

// ------------------------------------------------------------ sequences

fn emit_sequences(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::sequences_message(scene));
    host.emit(&msg);
}

/// Adds or replaces a sequence wholesale and rebroadcasts the list.
pub fn upsert_sequence(host: &impl SessionHost, sequence: botrail_scene::seq::Sequence) {
    host.with_scene(|scene| scene.upsert_sequence(sequence));
    emit_sequences(host);
}

pub fn remove_sequence(host: &impl SessionHost, name: &str) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.remove_sequence(name))?;
    emit_sequences(host);
    Ok(())
}

/// Declares (or re-initializes) an internal signal and rebroadcasts.
pub fn define_signal(host: &impl SessionHost, name: &str, initial: bool) {
    host.with_scene(|scene| scene.define_signal(name, initial));
    emit_sequences(host);
}

pub fn remove_signal(host: &impl SessionHost, name: &str) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.remove_signal(name))?;
    emit_sequences(host);
    Ok(())
}

fn emit_sensors(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::sensors_message(scene));
    host.emit(&msg);
}

/// Adds or replaces a pseudo-sensor and rebroadcasts the list.
pub fn upsert_sensor(host: &impl SessionHost, sensor: botrail_scene::seq::Sensor) {
    host.with_scene(|scene| scene.upsert_sensor(sensor));
    emit_sensors(host);
}

pub fn remove_sensor(host: &impl SessionHost, name: &str) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.remove_sensor(name))?;
    emit_sensors(host);
    Ok(())
}

fn emit_devices(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::devices_message(scene));
    host.emit(&msg);
}

/// Adds or replaces an auxiliary device and rebroadcasts the list.
pub fn upsert_device(host: &impl SessionHost, device: botrail_scene::seq::Device) {
    host.with_scene(|scene| scene.upsert_device(device));
    emit_devices(host);
}

pub fn remove_device(host: &impl SessionHost, name: &str) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.remove_device(name))?;
    emit_devices(host);
    Ok(())
}

/// Rolls out a sequence against a scene snapshot and emits the outcome as
/// a `sequence_result` message.
pub fn simulate_sequence_and_emit(
    host: &impl SessionHost,
    name: &str,
    options: &botrail_scene::rollout::RolloutOptions,
) -> Result<botrail_scene::rollout::SequenceTimeline, String> {
    let snapshot = host.snapshot();
    let t0 = host.now_ms();
    let result = snapshot
        .simulate_sequence(name, options)
        .map_err(|e| e.to_string());
    let ms = host.now_ms() - t0;
    let msg = match &result {
        Ok(timeline) => ServerMessage::SequenceResult {
            ok: true,
            sequence: name.to_string(),
            error: None,
            timeline: Some(timeline_msg(&snapshot, timeline)),
            planning_time_ms: Some(ms),
        },
        Err(e) => ServerMessage::SequenceResult {
            ok: false,
            sequence: name.to_string(),
            error: Some(e.clone()),
            timeline: None,
            planning_time_ms: None,
        },
    };
    host.emit(&msg);
    result
}

/// Samples a baked timeline at ~30Hz for playback: every robot rides its
/// embedded trajectory (link poses precomputed for URDF robots) on one
/// shared sample grid, tracked objects get world-pose tracks derived from
/// their spans (via each carrier's FK).
pub fn timeline_msg(
    scene: &Scene,
    timeline: &botrail_scene::rollout::SequenceTimeline,
) -> wire::TimelineMsg {
    // All robot tracks span [0, duration], so their resample grids agree —
    // the objects' shared time base.
    let sampled: Vec<(Vec<f64>, Vec<Vec<f64>>)> = timeline
        .robots
        .iter()
        .map(|track| track.trajectory.resample(1.0 / 30.0))
        .collect();
    let grid: &[f64] = sampled
        .first()
        .map(|(times, _)| times.as_slice())
        .unwrap_or(&[]);

    let mut object_tracks: Vec<wire::ObjectTrackMsg> = timeline
        .objects
        .iter()
        .map(|track| wire::ObjectTrackMsg {
            name: track.name.clone(),
            poses: Vec::with_capacity(grid.len()),
            visible: Vec::with_capacity(grid.len()),
        })
        .collect();
    if !object_tracks.is_empty() {
        for (k, &t) in grid.iter().enumerate() {
            let all_poses: Vec<Vec<nalgebra::Isometry3<f64>>> = scene
                .robots()
                .iter()
                .enumerate()
                .map(|(r, sr)| {
                    botrail_kin::forward_kinematics_with_base(
                        &sr.model,
                        &sampled[r].1[k],
                        sr.base_pose(),
                    )
                    .expect("timeline q has robot DOF")
                })
                .collect();
            for (msg, track) in object_tracks.iter_mut().zip(&timeline.objects) {
                let pose =
                    botrail_scene::rollout::SequenceTimeline::object_pose(track, &all_poses, t)
                        .unwrap_or_default();
                msg.poses.push(PoseMsg::from(&pose));
                msg.visible
                    .push(botrail_scene::rollout::SequenceTimeline::object_visible(
                        track, t,
                    ));
            }
        }
    }

    for msg in &mut object_tracks {
        if msg.visible.iter().all(|v| *v) {
            msg.visible.clear();
        }
    }

    let robots = timeline
        .robots
        .iter()
        .zip(sampled)
        .enumerate()
        .map(|(r, (track, (times, joint_positions)))| {
            let sr = &scene.robots()[r];
            // USD-rendered robots do FK client-side; skip precomputed poses.
            let link_poses = matches!(&sr.model.source, botrail_model::RobotSource::UrdfXml(_))
                .then(|| {
                    joint_positions
                        .iter()
                        .map(|q| {
                            botrail_kin::forward_kinematics_with_base(&sr.model, q, sr.base_pose())
                                .expect("timeline q has robot DOF")
                                .iter()
                                .map(PoseMsg::from)
                                .collect()
                        })
                        .collect()
                });
            wire::RobotTimelineMsg {
                name: track.name.clone(),
                trajectory: wire::TrajectoryMsg {
                    duration: timeline.duration,
                    times,
                    joint_positions,
                    link_poses,
                    object_tracks: None,
                },
                moves: track
                    .moves
                    .iter()
                    .map(|s| wire::StepSpanMsg {
                        name: s.name.clone(),
                        start: s.start,
                        end: s.end,
                    })
                    .collect(),
            }
        })
        .collect();

    wire::TimelineMsg {
        duration: timeline.duration,
        robots,
        objects: object_tracks,
        step_spans: timeline
            .step_spans
            .iter()
            .map(|s| wire::StepSpanMsg {
                name: s.name.clone(),
                start: s.start,
                end: s.end,
            })
            .collect(),
        signals: timeline
            .signals
            .iter()
            .map(|s| wire::SignalTrackMsg {
                name: s.name.clone(),
                times: s.edges.iter().map(|(t, _)| *t).collect(),
                values: s.edges.iter().map(|(_, v)| *v).collect(),
            })
            .collect(),
    }
}

// ------------------------------------------------------------- planning

/// Plans robot `robot` from its current configuration to `goal` against a
/// snapshot of the scene, then time-parameterizes the path; every other
/// robot is a frozen collision body at its current configuration. Returns
/// the trajectory, the sparse shortcut path (kept for script export), and
/// the wall-clock milliseconds spent.
pub fn plan_to_for(
    host: &impl SessionHost,
    robot: usize,
    goal: &[f64],
    options: &botrail_plan::PlanOptions,
) -> Result<(botrail_traj::JointTrajectory, Vec<Vec<f64>>, f64), String> {
    let snapshot = host.snapshot();
    let start = snapshot.robots()[robot].joint_positions().to_vec();
    let (lower, upper) = snapshot.robots()[robot].model.sampling_bounds();
    let space = botrail_plan::JointSpace { lower, upper };

    let t0 = host.now_ms();
    let path = {
        let mut is_valid = |q: &[f64]| snapshot.is_state_valid_for(robot, q);
        botrail_plan::plan(&space, &start, goal, &mut is_valid, options)
            .map_err(|e| e.to_string())?
    };
    let limits = traj_limits(&snapshot.robots()[robot].model);
    let traj =
        botrail_traj::time_parameterize(&path, &limits, &botrail_traj::TimingOptions::default())
            .map_err(|e| e.to_string())?;
    let ms = host.now_ms() - t0;
    Ok((traj, path, ms))
}

/// Runs [`plan_to_for`] and emits a `plan_result` tagged with the robot's
/// instance name, so clients play it back on the right arm.
pub fn plan_and_emit_for(
    host: &impl SessionHost,
    robot: usize,
    goal: &[f64],
    options: &botrail_plan::PlanOptions,
) -> Result<(botrail_traj::JointTrajectory, Vec<Vec<f64>>, f64), String> {
    let result = plan_to_for(host, robot, goal, options);
    let robot_name = host.with_scene(|scene| scene.robots()[robot].name.clone());
    let msg = match &result {
        Ok((traj, path, ms)) => ServerMessage::PlanResult {
            robot: robot_name,
            ok: true,
            error: None,
            trajectory: Some(trajectory_msg(host, robot, traj)),
            stats: Some(wire::PlanStatsMsg {
                planning_time_ms: *ms,
                waypoints: path.len(),
            }),
        },
        Err(e) => ServerMessage::PlanResult {
            robot: robot_name,
            ok: false,
            error: Some(e.clone()),
            trajectory: None,
            stats: None,
        },
    };
    host.emit(&msg);
    result
}

/// Plans a whole motion against a scene snapshot (nothing emitted). The
/// trajectory is timed with the owning robot's limits.
pub fn plan_motion_snapshot(
    host: &impl SessionHost,
    name: &str,
    options: &botrail_plan::PlanOptions,
) -> Result<PlannedMotion, String> {
    let snapshot = host.snapshot();
    let owner = motion_owner(&snapshot, name)?;
    snapshot
        .plan_motion(name, options, &traj_limits(&snapshot.robots()[owner].model))
        .map_err(|e| e.to_string())
}

/// The owning robot of a named motion.
fn motion_owner(scene: &Scene, name: &str) -> Result<usize, String> {
    scene
        .motions()
        .iter()
        .find(|m| m.name == name)
        .map(|m| m.robot)
        .ok_or_else(|| format!("unknown motion `{name}`"))
}

/// Plans a whole motion against a scene snapshot and emits the outcome as a
/// `motion_result` message.
pub fn plan_motion_and_emit(
    host: &impl SessionHost,
    name: &str,
    options: &botrail_plan::PlanOptions,
) -> Result<(PlannedMotion, f64), String> {
    let t0 = host.now_ms();
    let result = plan_motion_snapshot(host, name, options);
    let ms = host.now_ms() - t0;
    // The result plays back on the owning robot; an unknown motion (no
    // owner) reports its error against the first robot.
    let (owner, robot_name) = host.with_scene(|scene| {
        let owner = motion_owner(scene, name).unwrap_or(0);
        (owner, scene.robots()[owner].name.clone())
    });
    let msg = match &result {
        Ok(planned) => ServerMessage::MotionResult {
            robot: robot_name,
            ok: true,
            motion: name.to_string(),
            error: None,
            trajectory: Some(trajectory_msg(host, owner, &planned.trajectory)),
            segment_ends: planned.segment_ends.clone(),
            planning_time_ms: Some(ms),
        },
        Err(e) => ServerMessage::MotionResult {
            robot: robot_name,
            ok: false,
            motion: name.to_string(),
            error: Some(e.clone()),
            trajectory: None,
            segment_ends: Vec::new(),
            planning_time_ms: None,
        },
    };
    host.emit(&msg);
    result.map(|planned| (planned, ms))
}

/// Samples a trajectory of robot `robot` at ~30Hz with per-sample FK for
/// playback. Obstacles attached to *that* robot get world-pose tracks baked
/// alongside, whatever the robot's rendering path — the client never does
/// FK for obstacles. (Objects held by other robots don't move during this
/// playback; their live pose stands.)
pub fn trajectory_msg(
    host: &impl SessionHost,
    robot: usize,
    traj: &botrail_traj::JointTrajectory,
) -> wire::TrajectoryMsg {
    let (model, base, attachments) = host.with_scene(|scene| {
        (
            scene.robots()[robot].model.clone(),
            *scene.robots()[robot].base_pose(),
            scene
                .attachments()
                .iter()
                .filter(|a| a.robot == robot)
                .cloned()
                .collect::<Vec<_>>(),
        )
    });
    let (times, joint_positions) = traj.resample(1.0 / 30.0);
    // USD-rendered robots do FK client-side; skip the precomputed poses.
    let want_link_poses = matches!(&model.source, botrail_model::RobotSource::UrdfXml(_));
    let mut link_poses = want_link_poses.then(|| Vec::with_capacity(joint_positions.len()));
    let mut object_tracks = (!attachments.is_empty()).then(|| {
        attachments
            .iter()
            .map(|a| wire::ObjectTrackMsg {
                name: a.object.clone(),
                poses: Vec::with_capacity(joint_positions.len()),
                // A grasped object rides the arm the whole way; nothing to
                // hide.
                visible: Vec::new(),
            })
            .collect::<Vec<_>>()
    });
    if link_poses.is_some() || object_tracks.is_some() {
        for q in &joint_positions {
            let poses = botrail_kin::forward_kinematics_with_base(&model, q, &base)
                .expect("trajectory q has robot DOF");
            if let Some(link_poses) = &mut link_poses {
                link_poses.push(poses.iter().map(PoseMsg::from).collect());
            }
            if let Some(tracks) = &mut object_tracks {
                for (track, att) in tracks.iter_mut().zip(&attachments) {
                    track
                        .poses
                        .push(PoseMsg::from(&(poses[att.link] * att.grasp)));
                }
            }
        }
    }
    wire::TrajectoryMsg {
        duration: traj.duration(),
        times,
        joint_positions,
        link_poses,
        object_tracks,
    }
}

/// Trajectory limits derived from the model (see
/// [`botrail_scene::motion::traj_limits`]) — re-exported for hosts.
pub use botrail_scene::motion::traj_limits;

#[cfg(test)]
mod tests {
    use super::*;
    use std::cell::RefCell;
    use std::sync::Arc;

    /// Minimal single-threaded host: RefCell scene, collected messages.
    struct TestHost {
        scene: RefCell<Scene>,
        out: RefCell<Vec<ServerMessage>>,
        logs: RefCell<Vec<String>>,
    }

    impl TestHost {
        fn from_scene(scene: Scene) -> Self {
            TestHost {
                scene: RefCell::new(scene),
                out: RefCell::new(Vec::new()),
                logs: RefCell::new(Vec::new()),
            }
        }

        fn new() -> Self {
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
            let model = botrail_model::RobotModel::from_urdf_str(urdf).unwrap();
            Self::from_scene(Scene::new(Arc::new(model)))
        }

        fn message_types(&self) -> Vec<&'static str> {
            self.out
                .borrow()
                .iter()
                .map(|m| match m {
                    ServerMessage::SceneInit { .. } => "scene_init",
                    ServerMessage::Obstacles { .. } => "obstacles",
                    ServerMessage::State { .. } => "state",
                    ServerMessage::PlanResult { .. } => "plan_result",
                    ServerMessage::Motions { .. } => "motions",
                    ServerMessage::Frames { .. } => "frames",
                    ServerMessage::MotionResult { .. } => "motion_result",
                    ServerMessage::Sequences { .. } => "sequences",
                    ServerMessage::SequenceResult { .. } => "sequence_result",
                    ServerMessage::Sensors { .. } => "sensors",
                    ServerMessage::Devices { .. } => "devices",
                    ServerMessage::RecordingResult { .. } => "recording_result",
                })
                .collect()
        }
    }

    impl SessionHost for TestHost {
        fn with_scene<R>(&self, f: impl FnOnce(&mut Scene) -> R) -> R {
            f(&mut self.scene.borrow_mut())
        }
        fn emit(&self, msg: &ServerMessage) {
            self.out.borrow_mut().push(msg.clone());
        }
        fn now_ms(&self) -> f64 {
            0.0
        }
        fn log(&self, message: &str) {
            self.logs.borrow_mut().push(message.to_string());
        }
    }

    #[test]
    fn handshake_order() {
        let host = TestHost::new();
        let msgs = initial_messages(&host);
        assert!(matches!(msgs[0], ServerMessage::SceneInit { .. }));
        assert!(matches!(msgs[1], ServerMessage::Obstacles { .. }));
        assert!(matches!(msgs[2], ServerMessage::Motions { .. }));
        assert!(matches!(msgs[3], ServerMessage::Sequences { .. }));
        assert!(matches!(msgs[4], ServerMessage::Sensors { .. }));
        assert!(matches!(msgs[5], ServerMessage::Devices { .. }));
        assert!(matches!(msgs[6], ServerMessage::Frames { .. }));
        assert!(matches!(msgs[7], ServerMessage::State { .. }));
    }

    #[test]
    fn add_frames_broadcasts_the_list() {
        let host = TestHost::new();
        add_frames(
            &host,
            vec![("mount".to_string(), Isometry3::translation(1.0, 0.0, 0.5))],
        );
        assert_eq!(host.message_types(), ["frames"]);
        let out = host.out.borrow();
        let ServerMessage::Frames { frames } = &out[0] else {
            panic!("expected frames");
        };
        assert_eq!(frames[0].name, "mount");
        assert!((frames[0].pose.position[2] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn joint_positions_roundtrip_emits_state() {
        let host = TestHost::new();
        handle_client_message(&host, r#"{"type":"set_joint_positions","positions":[0.5]}"#);
        assert_eq!(host.message_types(), ["state"]);
        assert_eq!(host.scene.borrow().joint_positions(), &[0.5]);
        assert!(host.logs.borrow().is_empty());
    }

    #[test]
    fn bad_dof_is_logged_not_fatal() {
        let host = TestHost::new();
        handle_client_message(
            &host,
            r#"{"type":"set_joint_positions","positions":[0.1, 0.2]}"#,
        );
        assert!(host.message_types().is_empty());
        assert_eq!(host.logs.borrow().len(), 1);
    }

    #[test]
    fn obstacle_lifecycle_emits_obstacles_then_state() {
        let host = TestHost::new();
        handle_client_message(
            &host,
            r#"{"type":"add_obstacle","obstacle":{"name":"box","geometry":{"kind":"box","size":[0.2,0.2,0.2]},"pose":{"position":[1.0,0.0,0.0],"quaternion":[0.0,0.0,0.0,1.0]}}}"#,
        );
        assert_eq!(host.message_types(), ["obstacles", "state"]);
        assert_eq!(host.scene.borrow().obstacles().len(), 1);
    }

    #[test]
    fn update_poses_moves_a_subtree_in_one_broadcast() {
        let host = TestHost::new();
        host.with_scene(|scene| {
            for (name, z) in [("/W/Pedestal/Plate", 0.1), ("/W/Pedestal/Column", 0.4)] {
                scene
                    .add_obstacle(
                        name,
                        Geometry::Sphere { radius: 0.02 },
                        Isometry3::translation(0.0, 0.0, z),
                    )
                    .unwrap();
            }
            scene.add_frame("/W/Pedestal/Mount", Isometry3::translation(0.0, 0.0, 0.6));
        });

        handle_client_message(
            &host,
            r#"{"type":"update_poses",
                "obstacles":[
                  ["/W/Pedestal/Plate",{"position":[1.0,0.0,0.1],"quaternion":[0,0,0,1]}],
                  ["/W/Pedestal/Column",{"position":[1.0,0.0,0.4],"quaternion":[0,0,0,1]}]],
                "frames":[
                  ["/W/Pedestal/Mount",{"position":[1.0,0.0,0.6],"quaternion":[0,0,0,1]}]]}"#,
        );

        // One obstacles + state pair for the whole subtree, plus frames —
        // not one broadcast per member.
        assert_eq!(host.message_types(), ["obstacles", "state", "frames"]);
        let scene = host.scene.borrow();
        for name in ["/W/Pedestal/Plate", "/W/Pedestal/Column"] {
            let o = scene.obstacles().iter().find(|o| o.name == name).unwrap();
            assert!((o.pose.translation.x - 1.0).abs() < 1e-12, "{name}");
        }
        // The teach frame came with it; leaving it behind would silently
        // invalidate anything taught against this machine.
        let mount = scene.frame("/W/Pedestal/Mount").unwrap();
        assert!((mount.pose.translation.x - 1.0).abs() < 1e-12);
    }

    #[test]
    fn update_poses_is_all_or_nothing() {
        let host = TestHost::new();
        host.with_scene(|scene| {
            scene
                .add_obstacle(
                    "real",
                    Geometry::Sphere { radius: 0.02 },
                    Isometry3::identity(),
                )
                .unwrap();
        });
        handle_client_message(
            &host,
            r#"{"type":"update_poses","obstacles":[
                 ["real",{"position":[5.0,0.0,0.0],"quaternion":[0,0,0,1]}],
                 ["ghost",{"position":[5.0,0.0,0.0],"quaternion":[0,0,0,1]}]]}"#,
        );
        // A typo in one member must not half-move the subtree.
        assert!(host.message_types().is_empty());
        assert_eq!(host.logs.borrow().len(), 1);
        let scene = host.scene.borrow();
        assert_eq!(scene.obstacles()[0].pose.translation.x, 0.0);
    }

    #[test]
    fn plan_request_emits_plan_result() {
        let host = TestHost::new();
        handle_client_message(&host, r#"{"type":"plan_request","goal_positions":[0.8]}"#);
        let out = host.out.borrow();
        assert_eq!(out.len(), 1);
        match &out[0] {
            ServerMessage::PlanResult { ok, trajectory, .. } => {
                assert!(ok);
                assert!(trajectory.is_some());
            }
            other => panic!("expected plan_result, got {other:?}"),
        }
    }

    #[test]
    fn set_robot_base_pose_moves_base_and_emits_state() {
        let host = TestHost::new();
        handle_client_message(
            &host,
            r#"{"type":"set_robot_base_pose","pose":{"position":[1.0,2.0,0.5],"quaternion":[0.0,0.0,0.0,1.0]}}"#,
        );
        assert_eq!(host.message_types(), ["state"]);
        let scene = host.scene.borrow();
        assert!((scene.robot_base_pose().translation.x - 1.0).abs() < 1e-12);
        // The state message's link poses follow the base.
        let out = host.out.borrow();
        match &out[0] {
            ServerMessage::State { robots, .. } => {
                assert!((robots[0].base_pose.position[0] - 1.0).abs() < 1e-12);
                assert!((robots[0].link_poses[0].position[1] - 2.0).abs() < 1e-12);
            }
            other => panic!("expected state, got {other:?}"),
        }
    }

    #[test]
    fn robot_addressed_messages_drive_the_named_robot() {
        let host = TestHost::new();
        let (model, second) = host.with_scene(|scene| {
            let model = scene.robots()[0].model.clone();
            let name = scene.add_robot(
                model.clone(),
                Some("arm_b"),
                Isometry3::translation(1.0, 0.0, 0.0),
            );
            (model, name)
        });
        let _ = model;
        assert_eq!(second, "arm_b");

        handle_client_message(
            &host,
            r#"{"type":"set_joint_positions","robot":"arm_b","positions":[0.4]}"#,
        );
        assert!(host.logs.borrow().is_empty());
        host.with_scene(|scene| {
            assert_eq!(scene.robots()[1].joint_positions(), &[0.4]);
            assert_eq!(scene.robots()[0].joint_positions(), &[0.0]);
        });

        // Unknown robot names are rejected and logged, nothing emitted.
        host.out.borrow_mut().clear();
        handle_client_message(
            &host,
            r#"{"type":"set_joint_positions","robot":"ghost","positions":[0.1]}"#,
        );
        assert!(host.message_types().is_empty());
        assert_eq!(host.logs.borrow().len(), 1);
    }

    #[test]
    fn second_robot_plans_broadcast_with_the_robot_tag() {
        let host = TestHost::new();
        host.with_scene(|scene| {
            let model = scene.robots()[0].model.clone();
            scene.add_robot(model, Some("arm_b"), Isometry3::translation(1.0, 0.0, 0.0));
        });
        handle_client_message(
            &host,
            r#"{"type":"plan_request","robot":"arm_b","goal_positions":[0.6]}"#,
        );
        let out = host.out.borrow();
        let ServerMessage::PlanResult {
            robot,
            ok,
            trajectory,
            ..
        } = &out[0]
        else {
            panic!("expected plan_result, got {out:?}");
        };
        assert_eq!(robot, "arm_b");
        assert!(ok);
        assert!(trajectory.is_some());
    }

    /// TestHost with a robot whose source is a USD stage reference and a
    /// host that serves assets.
    fn usd_host() -> TestHost {
        use botrail_model::{Geometry as G, Joint, JointLimits, JointType, Link, Shape};
        use nalgebra::{Translation3, Unit, UnitQuaternion, Vector3};
        let shape = || Shape {
            origin: Isometry3::identity(),
            geometry: G::Box {
                size: Vector3::new(0.1, 0.1, 0.1),
            },
        };
        let links = vec![
            Link {
                name: "/R/base".into(),
                visuals: vec![shape()],
                collisions: vec![],
                parent_joint: None,
            },
            Link {
                name: "/R/arm".into(),
                visuals: vec![shape()],
                collisions: vec![],
                parent_joint: None,
            },
        ];
        let joints = vec![Joint {
            name: "/R/j1".into(),
            joint_type: JointType::Revolute,
            origin: Isometry3::from_parts(
                Translation3::new(0.0, 0.0, 0.5),
                UnitQuaternion::identity(),
            ),
            axis: Unit::new_normalize(Vector3::z()),
            limits: Some(JointLimits {
                lower: -1.0,
                upper: 1.0,
                velocity: 1.0,
                effort: 1.0,
            }),
            parent_link: 0,
            child_link: 1,
            q_index: None,
            mimic: None,
        }];
        let model = botrail_model::RobotModel::from_parts(
            "usdbot".into(),
            links,
            joints,
            botrail_model::RobotSource::Usd {
                path: "/tmp/robots/arm.usda".into(),
                articulation_root: "/R".into(),
            },
        )
        .unwrap();
        TestHost::from_scene(Scene::new(Arc::new(model)))
    }

    struct AssetHost(TestHost);

    impl SessionHost for AssetHost {
        fn with_scene<R>(&self, f: impl FnOnce(&mut Scene) -> R) -> R {
            self.0.with_scene(f)
        }
        fn robot_asset_url(&self, robot: usize, path: &Path) -> Option<String> {
            path.file_name()
                .map(|f| format!("/usd-assets/{robot}/{}", f.to_string_lossy()))
        }
        fn emit(&self, msg: &ServerMessage) {
            self.0.emit(msg);
        }
        fn now_ms(&self) -> f64 {
            0.0
        }
        fn log(&self, message: &str) {
            self.0.log(message);
        }
    }

    #[test]
    fn usd_robot_scene_init_references_the_asset() {
        let host = AssetHost(usd_host());
        let msgs = initial_messages(&host);
        let ServerMessage::SceneInit { scene } = &msgs[0] else {
            panic!("expected scene_init");
        };
        let asset = scene.robots[0]
            .usd_asset
            .as_ref()
            .expect("usd_asset present");
        assert_eq!(asset.url, "/usd-assets/0/arm.usda");
        assert_eq!(asset.articulation_root, "/R");

        // A host without asset serving (wasm) keeps the legacy path.
        let plain = usd_host();
        let msgs = initial_messages(&plain);
        let ServerMessage::SceneInit { scene } = &msgs[0] else {
            panic!("expected scene_init");
        };
        assert!(scene.robots[0].usd_asset.is_none());
    }

    #[test]
    fn usd_robot_trajectories_are_joint_only() {
        let host = AssetHost(usd_host());
        handle_client_message(&host, r#"{"type":"plan_request","goal_positions":[0.8]}"#);
        let out = host.0.out.borrow();
        match &out[0] {
            ServerMessage::PlanResult { ok, trajectory, .. } => {
                assert!(ok);
                let traj = trajectory.as_ref().unwrap();
                assert!(traj.link_poses.is_none(), "USD robots skip pose baking");
                assert!(!traj.joint_positions.is_empty());
            }
            other => panic!("expected plan_result, got {other:?}"),
        }

        // Legacy URDF robots keep precomputed poses.
        let legacy = TestHost::new();
        handle_client_message(&legacy, r#"{"type":"plan_request","goal_positions":[0.8]}"#);
        let out = legacy.out.borrow();
        match &out[0] {
            ServerMessage::PlanResult { trajectory, .. } => {
                assert!(trajectory.as_ref().unwrap().link_poses.is_some());
            }
            other => panic!("expected plan_result, got {other:?}"),
        }
    }

    #[test]
    fn sequence_upsert_simulate_and_result_roundtrip() {
        let host = TestHost::new();
        host.with_scene(|scene| {
            scene
                .add_obstacle(
                    "box",
                    Geometry::Sphere { radius: 0.02 },
                    Isometry3::translation(0.1, 0.0, 0.5),
                )
                .unwrap();
            scene
                .add_segment(
                    "go",
                    Segment {
                        kind: botrail_scene::motion::SegmentKind::Joint,
                        goal_positions: vec![0.8],
                        constraints: vec![],
                    },
                )
                .unwrap();
        });
        handle_client_message(
            &host,
            r#"{"type":"define_signal","name":"flag","initial":false}"#,
        );
        handle_client_message(
            &host,
            r#"{"type":"upsert_sequence","sequence":{"name":"pick","steps":[
                {"name":"grasp","actions":[{"type":"attach","object":"box"}],
                 "transition":{"type":"immediately"}},
                {"name":"move","actions":[{"type":"start_motion","motion":"go"}],
                 "transition":{"type":"done"}},
                {"name":"mark","actions":[{"type":"set","signal":"flag","value":true}],
                 "transition":{"type":"elapsed","seconds":0.2}}
            ]}}"#,
        );
        assert_eq!(host.message_types(), ["sequences", "sequences"]);
        host.out.borrow_mut().clear();

        handle_client_message(&host, r#"{"type":"simulate_sequence","name":"pick"}"#);
        let out = host.out.borrow();
        let ServerMessage::SequenceResult {
            ok,
            timeline,
            error,
            ..
        } = &out[0]
        else {
            panic!("expected sequence_result, got {out:?}");
        };
        assert!(ok, "{error:?}");
        let tl = timeline.as_ref().unwrap();
        assert_eq!(tl.step_spans.len(), 3);
        assert_eq!(tl.step_spans[1].name, "move");
        // A single robot track (sequences drive the first robot until R3),
        // named after the instance...
        assert_eq!(tl.robots.len(), 1);
        assert_eq!(tl.robots[0].name, "r");
        // ...and the grasped box rides the timeline's object track.
        let tracks = &tl.objects;
        assert_eq!(tracks[0].name, "box");
        assert_eq!(tracks[0].poses.len(), tl.robots[0].trajectory.times.len());
        let last = tracks[0].poses.last().unwrap();
        assert!((last.position[0] - 0.1 * 0.8f64.cos()).abs() < 1e-6);
        // ...and the signal lane records the mark edge.
        assert_eq!(tl.signals.len(), 1);
        assert_eq!(tl.signals[0].values, vec![false, true]);
        // The live scene stays untouched by the rollout.
        host.with_scene(|scene| {
            assert!(scene.attachments().is_empty());
            assert_eq!(scene.joint_positions(), &[0.0]);
        });

        drop(out);
        host.out.borrow_mut().clear();
        // A broken sequence reports through the result, not a log.
        handle_client_message(
            &host,
            r#"{"type":"upsert_sequence","sequence":{"name":"bad","steps":[
                {"name":"x","actions":[{"type":"start_motion","motion":"nope"}],
                 "transition":{"type":"done"}}]}}"#,
        );
        host.out.borrow_mut().clear();
        handle_client_message(&host, r#"{"type":"simulate_sequence","name":"bad"}"#);
        let out = host.out.borrow();
        let ServerMessage::SequenceResult { ok, error, .. } = &out[0] else {
            panic!("expected sequence_result");
        };
        assert!(!ok && error.as_ref().unwrap().contains("unknown motion"));
    }

    #[test]
    fn unparseable_message_is_logged() {
        let host = TestHost::new();
        handle_client_message(&host, "not json");
        assert!(host.message_types().is_empty());
        assert_eq!(host.logs.borrow().len(), 1);
    }

    #[test]
    fn attach_detach_dispatch_rebroadcasts_obstacles() {
        let host = TestHost::new();
        host.with_scene(|scene| {
            scene
                .add_obstacle(
                    "box",
                    Geometry::Sphere { radius: 0.02 },
                    Isometry3::translation(0.1, 0.0, 0.5),
                )
                .unwrap()
        });
        handle_client_message(&host, r#"{"type":"attach_obstacle","name":"box"}"#);
        assert_eq!(host.message_types(), ["obstacles", "state"]);
        assert!(host.logs.borrow().is_empty());
        {
            let out = host.out.borrow();
            let ServerMessage::Obstacles { obstacles } = &out[0] else {
                panic!("expected obstacles");
            };
            let att = obstacles[0].attached_to.as_ref().expect("attached");
            assert_eq!(att.link, "b");
        }

        host.out.borrow_mut().clear();
        handle_client_message(&host, r#"{"type":"detach_obstacle","name":"box"}"#);
        assert_eq!(host.message_types(), ["obstacles", "state"]);
        {
            let out = host.out.borrow();
            let ServerMessage::Obstacles { obstacles } = &out[0] else {
                panic!("expected obstacles");
            };
            assert!(obstacles[0].attached_to.is_none());
        }

        // Unknown obstacle: logged, nothing emitted.
        host.out.borrow_mut().clear();
        handle_client_message(&host, r#"{"type":"attach_obstacle","name":"ghost"}"#);
        assert!(host.message_types().is_empty());
        assert_eq!(host.logs.borrow().len(), 1);
    }

    #[test]
    fn joint_updates_rebroadcast_obstacles_while_attached() {
        let host = TestHost::new();
        host.with_scene(|scene| {
            scene
                .add_obstacle(
                    "held",
                    Geometry::Sphere { radius: 0.02 },
                    Isometry3::translation(0.1, 0.0, 0.5),
                )
                .unwrap();
        });
        // Nothing attached: joint updates emit state only.
        handle_client_message(&host, r#"{"type":"set_joint_positions","positions":[0.2]}"#);
        assert_eq!(host.message_types(), ["state"]);
        host.out.borrow_mut().clear();

        host.with_scene(|scene| scene.attach_obstacle("held", None, None).unwrap());
        handle_client_message(&host, r#"{"type":"set_joint_positions","positions":[0.9]}"#);
        assert_eq!(host.message_types(), ["obstacles", "state"]);
        let out = host.out.borrow();
        let ServerMessage::Obstacles { obstacles } = &out[0] else {
            panic!("expected obstacles");
        };
        // The rebroadcast carries the followed pose. Grasped at q=0.2 and
        // moved to q=0.9, the box swings by the 0.7 rad delta.
        let p = &obstacles[0].pose.position;
        assert!((p[0] - 0.1 * 0.7f64.cos()).abs() < 1e-9);
        assert!((p[1] - 0.1 * 0.7f64.sin()).abs() < 1e-9);
    }

    #[test]
    fn trajectories_bake_attached_object_tracks() {
        let host = TestHost::new();
        host.with_scene(|scene| {
            scene
                .add_obstacle(
                    "held",
                    Geometry::Sphere { radius: 0.02 },
                    Isometry3::translation(0.1, 0.0, 0.5),
                )
                .unwrap();
            scene.attach_obstacle("held", None, None).unwrap();
        });
        handle_client_message(&host, r#"{"type":"plan_request","goal_positions":[0.8]}"#);
        let out = host.out.borrow();
        let ServerMessage::PlanResult { ok, trajectory, .. } = &out[0] else {
            panic!("expected plan_result");
        };
        assert!(ok);
        let traj = trajectory.as_ref().unwrap();
        let tracks = traj.object_tracks.as_ref().expect("object tracks");
        assert_eq!(tracks.len(), 1);
        assert_eq!(tracks[0].name, "held");
        assert_eq!(tracks[0].poses.len(), traj.times.len());
        // The final sample lands exactly on the goal: the held object sits
        // at Rz(0.8)·(0.1, 0, 0) + (0, 0, 0.5).
        let last = tracks[0].poses.last().unwrap();
        assert!((last.position[0] - 0.1 * 0.8f64.cos()).abs() < 1e-9);
        assert!((last.position[1] - 0.1 * 0.8f64.sin()).abs() < 1e-9);
        assert!((last.position[2] - 0.5).abs() < 1e-12);
    }

    #[test]
    fn usd_robots_get_object_tracks_without_link_poses() {
        let host = AssetHost(usd_host());
        host.with_scene(|scene| {
            scene
                .add_obstacle(
                    "held",
                    Geometry::Sphere { radius: 0.02 },
                    Isometry3::translation(0.1, 0.0, 0.5),
                )
                .unwrap();
            scene.attach_obstacle("held", None, None).unwrap();
        });
        handle_client_message(&host, r#"{"type":"plan_request","goal_positions":[0.8]}"#);
        let out = host.0.out.borrow();
        let ServerMessage::PlanResult { trajectory, .. } = &out[0] else {
            panic!("expected plan_result");
        };
        let traj = trajectory.as_ref().unwrap();
        assert!(traj.link_poses.is_none(), "USD robots skip pose baking");
        assert!(traj.object_tracks.is_some(), "objects are baked regardless");
    }
}
