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
use botrail_model::{Geometry, RobotModel};
use botrail_scene::motion::{PlannedMotion, Segment};
use botrail_scene::wire::{
    self, ClientMessage, IkStatusMsg, PoseMsg, SceneDescriptionMsg, ServerMessage,
};
use botrail_scene::{Scene, SceneError};
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
    /// references must resolve against it). `None` (default) keeps the
    /// client on the legacy link-visual rendering path.
    fn robot_asset_url(&self, _path: &Path) -> Option<String> {
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
/// state. Mesh visuals are mapped to URLs through the host's
/// [`mesh_url`](SessionHost::mesh_url).
pub fn initial_messages(host: &impl SessionHost) -> Vec<ServerMessage> {
    host.with_scene(|scene| {
        vec![
            scene_init_message(host, scene),
            wire::obstacles_message(scene, |p| host.mesh_url(p)),
            wire::motions_message(scene),
            wire::frames_message(scene),
            wire::state_message(scene),
        ]
    })
}

/// The `scene_init` message, with mesh/asset URLs mapped through the host.
pub fn scene_init_message(host: &impl SessionHost, scene: &Scene) -> ServerMessage {
    let usd_asset = match &scene.robot.source {
        botrail_model::RobotSource::Usd {
            path,
            articulation_root,
        } => host.robot_asset_url(path).map(|url| wire::UsdAssetMsg {
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

fn dispatch(host: &impl SessionHost, msg: ClientMessage) -> Result<(), String> {
    match msg {
        ClientMessage::SetJointPositions { positions } => set_joint_positions(host, positions)
            .map_err(|e| format!("rejected set_joint_positions: {e}")),
        ClientMessage::SetTcpTarget { link, pose } => {
            // Warm-seeded streaming solve: the gizmo sends targets at
            // ~60Hz, so a few iterations per message are enough.
            let options = IkOptions {
                mode: IkMode::Pose,
                ..IkOptions::streaming()
            };
            set_tcp_target(host, &link, &pose, &options)
                .map(|_| ())
                .map_err(|e| format!("rejected tcp target: {e}"))
        }
        ClientMessage::SetRobotBasePose { pose } => {
            set_robot_base_pose(host, (&pose).into());
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
        ClientMessage::PlanRequest { goal_positions } => {
            // Failure is reported to clients inside the plan_result.
            let _ = plan_and_emit(host, &goal_positions, &botrail_plan::PlanOptions::default());
            Ok(())
        }
        ClientMessage::AddSegment { motion, segment } => {
            add_segment(host, &motion, wire::segment_from_msg(&segment))
                .map_err(|e| format!("rejected add_segment: {e}"))
        }
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
    }
}

// ---------------------------------------------------------------- state

pub fn emit_state(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::state_message(scene));
    host.emit(&msg);
}

pub fn set_joint_positions(
    host: &impl SessionHost,
    positions: Vec<f64>,
) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.set_joint_positions(positions))?;
    emit_state(host);
    Ok(())
}

/// Places the robot's root link at `pose` (world frame) and emits the new
/// state.
pub fn set_robot_base_pose(host: &impl SessionHost, pose: Isometry3<f64>) {
    host.with_scene(|scene| scene.set_robot_base_pose(pose));
    emit_state(host);
}

/// Solves IK for `link` toward `pose` (world frame), seeded from the
/// current configuration, applies the (best-effort) result, and emits the
/// new state tagged with the IK outcome.
pub fn set_tcp_target(
    host: &impl SessionHost,
    link: &str,
    pose: &PoseMsg,
    options: &IkOptions,
) -> Result<IkResult, String> {
    let (result, state) = host.with_scene(|scene| -> Result<_, String> {
        let index = scene
            .robot
            .link_index(link)
            .ok_or_else(|| format!("unknown link `{link}`"))?;
        let target: Isometry3<f64> = pose.into();
        let seed = scene.joint_positions().to_vec();
        let result = scene
            .solve_ik_world(index, &target, &seed, options)
            .map_err(|e| e.to_string())?;
        scene
            .set_joint_positions(result.q.clone())
            .map_err(|e| e.to_string())?;
        let status = IkStatusMsg {
            converged: result.converged,
            pos_error: result.pos_error,
            rot_error: result.rot_error,
        };
        Ok((result, wire::state_message_with_ik(scene, Some(status))))
    })?;
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
    batch: Vec<(String, Geometry, Isometry3<f64>)>,
) -> Result<Vec<String>, SceneError> {
    let names = host.with_scene(|scene| {
        batch
            .into_iter()
            .map(|(name, geometry, pose)| scene.add_obstacle(&name, geometry, pose))
            .collect::<Result<Vec<_>, _>>()
    })?;
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

pub fn set_obstacle_geometry(
    host: &impl SessionHost,
    name: &str,
    geometry: Geometry,
) -> Result<(), SceneError> {
    host.with_scene(|scene| scene.set_obstacle_geometry(name, geometry))?;
    emit_obstacles_and_state(host);
    Ok(())
}

// --------------------------------------------------------------- frames

fn emit_frames(host: &impl SessionHost) {
    let msg = host.with_scene(|scene| wire::frames_message(scene));
    host.emit(&msg);
}

/// Adds/updates named world frames and rebroadcasts the frame list.
pub fn add_frames(
    host: &impl SessionHost,
    frames: Vec<(String, Isometry3<f64>)>,
) {
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

pub fn add_segment(host: &impl SessionHost, motion: &str, segment: Segment) -> Result<(), String> {
    host.with_scene(|scene| scene.add_segment(motion, segment))
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

// ------------------------------------------------------------- planning

/// Plans from the current configuration to `goal` against a snapshot of the
/// scene, then time-parameterizes the path. Returns the trajectory, the
/// sparse shortcut path (kept for script export), and the wall-clock
/// milliseconds spent.
pub fn plan_to(
    host: &impl SessionHost,
    goal: &[f64],
    options: &botrail_plan::PlanOptions,
) -> Result<(botrail_traj::JointTrajectory, Vec<Vec<f64>>, f64), String> {
    let snapshot = host.snapshot();
    let start = snapshot.joint_positions().to_vec();
    let (lower, upper) = snapshot.robot.sampling_bounds();
    let space = botrail_plan::JointSpace { lower, upper };

    let t0 = host.now_ms();
    let path = {
        let mut is_valid = |q: &[f64]| snapshot.is_state_valid(q);
        botrail_plan::plan(&space, &start, goal, &mut is_valid, options)
            .map_err(|e| e.to_string())?
    };
    let limits = traj_limits(&snapshot.robot);
    let traj =
        botrail_traj::time_parameterize(&path, &limits, &botrail_traj::TimingOptions::default())
            .map_err(|e| e.to_string())?;
    let ms = host.now_ms() - t0;
    Ok((traj, path, ms))
}

/// Runs [`plan_to`] and emits the outcome (success or failure) as a
/// `plan_result` message.
pub fn plan_and_emit(
    host: &impl SessionHost,
    goal: &[f64],
    options: &botrail_plan::PlanOptions,
) -> Result<(botrail_traj::JointTrajectory, Vec<Vec<f64>>, f64), String> {
    let result = plan_to(host, goal, options);
    let msg = match &result {
        Ok((traj, path, ms)) => ServerMessage::PlanResult {
            ok: true,
            error: None,
            trajectory: Some(trajectory_msg(host, traj)),
            stats: Some(wire::PlanStatsMsg {
                planning_time_ms: *ms,
                waypoints: path.len(),
            }),
        },
        Err(e) => ServerMessage::PlanResult {
            ok: false,
            error: Some(e.clone()),
            trajectory: None,
            stats: None,
        },
    };
    host.emit(&msg);
    result
}

/// Plans a whole motion against a scene snapshot (nothing emitted).
pub fn plan_motion_snapshot(
    host: &impl SessionHost,
    name: &str,
    options: &botrail_plan::PlanOptions,
) -> Result<PlannedMotion, String> {
    let snapshot = host.snapshot();
    snapshot
        .plan_motion(name, options, &traj_limits(&snapshot.robot))
        .map_err(|e| e.to_string())
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
    let msg = match &result {
        Ok(planned) => ServerMessage::MotionResult {
            ok: true,
            motion: name.to_string(),
            error: None,
            trajectory: Some(trajectory_msg(host, &planned.trajectory)),
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
    host.emit(&msg);
    result.map(|planned| (planned, ms))
}

/// Samples a trajectory at ~30Hz with per-sample FK for playback.
pub fn trajectory_msg(
    host: &impl SessionHost,
    traj: &botrail_traj::JointTrajectory,
) -> wire::TrajectoryMsg {
    let (robot, base) = host.with_scene(|scene| (scene.robot.clone(), *scene.robot_base_pose()));
    let (times, joint_positions) = traj.resample(1.0 / 30.0);
    // USD-rendered robots do FK client-side; skip the precomputed poses.
    let link_poses = match &robot.source {
        botrail_model::RobotSource::Usd { .. } => None,
        botrail_model::RobotSource::UrdfXml(_) => Some(
            joint_positions
                .iter()
                .map(|q| {
                    botrail_kin::forward_kinematics_with_base(&robot, q, &base)
                        .expect("trajectory q has scene DOF")
                        .iter()
                        .map(PoseMsg::from)
                        .collect()
                })
                .collect(),
        ),
    };
    wire::TrajectoryMsg {
        duration: traj.duration(),
        times,
        joint_positions,
        link_poses,
    }
}

/// Trajectory limits from the URDF: joint velocity limits (defaulting to
/// 1 rad/s where unspecified) and acceleration at twice the velocity bound
/// (URDF has no acceleration field; reaches peak speed in 0.5s).
pub fn traj_limits(model: &RobotModel) -> botrail_traj::Limits {
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
        assert!(matches!(msgs[3], ServerMessage::Frames { .. }));
        assert!(matches!(msgs[4], ServerMessage::State { .. }));
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
    fn plan_request_emits_plan_result() {
        let host = TestHost::new();
        handle_client_message(&host, r#"{"type":"plan_request","goal_positions":[0.8]}"#);
        let out = host.out.borrow();
        assert_eq!(out.len(), 1);
        match &out[0] {
            ServerMessage::PlanResult {
                ok, trajectory, ..
            } => {
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
            ServerMessage::State {
                base_pose,
                link_poses,
                ..
            } => {
                assert!((base_pose.position[0] - 1.0).abs() < 1e-12);
                assert!((link_poses[0].position[1] - 2.0).abs() < 1e-12);
            }
            other => panic!("expected state, got {other:?}"),
        }
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
        fn robot_asset_url(&self, path: &Path) -> Option<String> {
            path.file_name()
                .map(|f| format!("/assets/{}", f.to_string_lossy()))
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
        let asset = scene.usd_asset.as_ref().expect("usd_asset present");
        assert_eq!(asset.url, "/assets/arm.usda");
        assert_eq!(asset.articulation_root, "/R");

        // A host without asset serving (wasm) keeps the legacy path.
        let plain = usd_host();
        let msgs = initial_messages(&plain);
        let ServerMessage::SceneInit { scene } = &msgs[0] else {
            panic!("expected scene_init");
        };
        assert!(scene.usd_asset.is_none());
    }

    #[test]
    fn usd_robot_trajectories_are_joint_only() {
        let host = AssetHost(usd_host());
        handle_client_message(&host, r#"{"type":"plan_request","goal_positions":[0.8]}"#);
        let out = host.0.out.borrow();
        match &out[0] {
            ServerMessage::PlanResult {
                ok, trajectory, ..
            } => {
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
    fn unparseable_message_is_logged() {
        let host = TestHost::new();
        handle_client_message(&host, "not json");
        assert!(host.message_types().is_empty());
        assert_eq!(host.logs.borrow().len(), 1);
    }
}
