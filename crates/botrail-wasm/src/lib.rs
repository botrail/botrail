//! Browser-complete botrail session.
//!
//! `WasmSession` speaks exactly the same JSON wire protocol as the Python
//! server (crates/botrail-scene/src/wire.rs), so the studio UI runs
//! unchanged against either backend. A wasm session has a single client, so
//! "broadcasting" simply means returning the messages to the caller.
//!
//! Follow-up (tracked in DESIGN): the message dispatch here mirrors
//! botrail-py's hub; both should eventually sit on a shared botrail-session
//! crate.
//!
//! Mesh visuals are not served in wasm mode (no mesh I/O yet); the embedded
//! demo robot is primitive-only.

use botrail_scene::wire::{self, ClientMessage, IkStatusMsg, PoseMsg, ServerMessage};
use botrail_scene::Scene;
use std::sync::Arc;
use wasm_bindgen::prelude::*;

const DEMO_URDF: &str = include_str!("../../../examples/simple_arm.urdf");

fn to_json(msg: &ServerMessage) -> String {
    serde_json::to_string(msg).expect("wire types serialize infallibly")
}

/// Trajectory limits mirroring the server's defaults (URDF velocity limits,
/// acceleration at twice the velocity bound).
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

#[wasm_bindgen]
pub struct WasmSession {
    scene: Scene,
}

#[wasm_bindgen]
impl WasmSession {
    /// Builds a session from a URDF string.
    #[wasm_bindgen(constructor)]
    pub fn new(urdf: &str) -> Result<WasmSession, JsError> {
        let model = botrail_model::RobotModel::from_urdf_str(urdf)
            .map_err(|e| JsError::new(&e.to_string()))?;
        Ok(WasmSession {
            scene: Scene::new(Arc::new(model)),
        })
    }

    /// Session with the embedded sample arm (primitive-only 6-DOF).
    pub fn demo() -> Result<WasmSession, JsError> {
        Self::new(DEMO_URDF)
    }

    /// The connection handshake, in order:
    /// scene_init, obstacles, motions, state.
    pub fn initial_messages(&self) -> Vec<String> {
        let mut no_mesh = |_: &std::path::Path| (String::new(), String::new());
        let scene_init = ServerMessage::SceneInit {
            scene: wire::SceneDescriptionMsg::from_scene(&self.scene, &mut no_mesh),
        };
        vec![
            to_json(&scene_init),
            to_json(&wire::obstacles_message(&self.scene)),
            to_json(&wire::motions_message(&self.scene)),
            to_json(&wire::state_message(&self.scene)),
        ]
    }

    /// Handles one client message; returns the messages a server would
    /// have broadcast in response.
    pub fn handle(&mut self, text: &str) -> Vec<String> {
        let msg: ClientMessage = match serde_json::from_str(text) {
            Ok(msg) => msg,
            Err(e) => {
                web_log(&format!("botrail-wasm: unparseable client message: {e}"));
                return Vec::new();
            }
        };
        match msg {
            ClientMessage::SetJointPositions { positions } => {
                if let Err(e) = self.scene.set_joint_positions(positions) {
                    web_log(&format!("botrail-wasm: {e}"));
                    return Vec::new();
                }
                vec![to_json(&wire::state_message(&self.scene))]
            }
            ClientMessage::SetTcpTarget { link, pose } => self.set_tcp_target(&link, &pose),
            ClientMessage::AddObstacle { obstacle } => {
                let result = wire::geometry_from_msg(&obstacle.geometry).and_then(|geometry| {
                    self.scene
                        .add_obstacle(&obstacle.name, geometry, (&obstacle.pose).into())
                        .map_err(|e| e.to_string())
                });
                self.obstacle_outcome(result.map(|_| ()))
            }
            ClientMessage::UpdateObstaclePose { name, pose } => {
                let result = self
                    .scene
                    .set_obstacle_pose(&name, (&pose).into())
                    .map_err(|e| e.to_string());
                self.obstacle_outcome(result)
            }
            ClientMessage::UpdateObstacleGeometry { name, geometry } => {
                let result = wire::geometry_from_msg(&geometry).and_then(|geometry| {
                    self.scene
                        .set_obstacle_geometry(&name, geometry)
                        .map_err(|e| e.to_string())
                });
                self.obstacle_outcome(result)
            }
            ClientMessage::RemoveObstacle { name } => {
                let result = self.scene.remove_obstacle(&name).map_err(|e| e.to_string());
                self.obstacle_outcome(result)
            }
            ClientMessage::PlanRequest { goal_positions } => {
                vec![to_json(&self.plan(&goal_positions))]
            }
            ClientMessage::AddSegment { motion, segment } => {
                let result = self
                    .scene
                    .add_segment(&motion, wire::segment_from_msg(&segment))
                    .map_err(|e| e.to_string());
                self.motion_outcome(result)
            }
            ClientMessage::RemoveSegment { motion, index } => {
                let result = self
                    .scene
                    .remove_segment(&motion, index)
                    .map_err(|e| e.to_string());
                self.motion_outcome(result)
            }
            ClientMessage::ClearMotion { motion } => {
                let result = self.scene.clear_motion(&motion).map_err(|e| e.to_string());
                self.motion_outcome(result)
            }
            ClientMessage::PlanMotion { motion } => vec![to_json(&self.plan_motion(&motion))],
        }
    }

    // Internal helpers are not exported; wasm-bindgen only sees pub methods
    // in this impl, so keep the private ones below in a separate impl block.
}

impl WasmSession {
    fn obstacle_outcome(&self, result: Result<(), String>) -> Vec<String> {
        if let Err(e) = result {
            web_log(&format!("botrail-wasm: {e}"));
            return Vec::new();
        }
        vec![
            to_json(&wire::obstacles_message(&self.scene)),
            to_json(&wire::state_message(&self.scene)),
        ]
    }

    fn motion_outcome(&self, result: Result<(), String>) -> Vec<String> {
        if let Err(e) = result {
            web_log(&format!("botrail-wasm: {e}"));
            return Vec::new();
        }
        vec![to_json(&wire::motions_message(&self.scene))]
    }

    fn set_tcp_target(&mut self, link: &str, pose: &PoseMsg) -> Vec<String> {
        let Some(index) = self.scene.robot.link_index(link) else {
            web_log(&format!("botrail-wasm: unknown link `{link}`"));
            return Vec::new();
        };
        let target: nalgebra::Isometry3<f64> = pose.into();
        let seed = self.scene.joint_positions().to_vec();
        let options = botrail_kin::IkOptions {
            mode: botrail_kin::IkMode::Pose,
            ..botrail_kin::IkOptions::streaming()
        };
        let Ok(result) = botrail_kin::solve_ik(&self.scene.robot, index, &target, &seed, &options)
        else {
            return Vec::new();
        };
        let status = IkStatusMsg {
            converged: result.converged,
            pos_error: result.pos_error,
            rot_error: result.rot_error,
        };
        if self.scene.set_joint_positions(result.q).is_err() {
            return Vec::new();
        }
        vec![to_json(&wire::state_message_with_ik(
            &self.scene,
            Some(status),
        ))]
    }

    fn plan(&self, goal: &[f64]) -> ServerMessage {
        let start = now_ms();
        let result: Result<(usize, botrail_traj::JointTrajectory), String> = (|| {
            let (lower, upper) = self.scene.robot.sampling_bounds();
            let space = botrail_plan::JointSpace { lower, upper };
            let start_q = self.scene.joint_positions().to_vec();
            let mut is_valid = |q: &[f64]| self.scene.is_state_valid(q);
            let path = botrail_plan::plan(
                &space,
                &start_q,
                goal,
                &mut is_valid,
                &botrail_plan::PlanOptions::default(),
            )
            .map_err(|e| e.to_string())?;
            let traj = botrail_traj::time_parameterize(
                &path,
                &traj_limits(&self.scene.robot),
                &botrail_traj::TimingOptions::default(),
            )
            .map_err(|e| e.to_string())?;
            Ok((path.len(), traj))
        })();
        match result {
            Ok((waypoints, traj)) => ServerMessage::PlanResult {
                ok: true,
                error: None,
                trajectory: Some(self.trajectory_msg(&traj)),
                stats: Some(wire::PlanStatsMsg {
                    planning_time_ms: now_ms() - start,
                    waypoints,
                }),
            },
            Err(e) => ServerMessage::PlanResult {
                ok: false,
                error: Some(e),
                trajectory: None,
                stats: None,
            },
        }
    }

    fn plan_motion(&self, name: &str) -> ServerMessage {
        let start = now_ms();
        match self.scene.plan_motion(
            name,
            &botrail_plan::PlanOptions::default(),
            &traj_limits(&self.scene.robot),
        ) {
            Ok(planned) => ServerMessage::MotionResult {
                ok: true,
                motion: name.to_string(),
                error: None,
                trajectory: Some(self.trajectory_msg(&planned.trajectory)),
                segment_ends: planned.segment_ends,
                planning_time_ms: Some(now_ms() - start),
            },
            Err(e) => ServerMessage::MotionResult {
                ok: false,
                motion: name.to_string(),
                error: Some(e.to_string()),
                trajectory: None,
                segment_ends: Vec::new(),
                planning_time_ms: None,
            },
        }
    }

    fn trajectory_msg(&self, traj: &botrail_traj::JointTrajectory) -> wire::TrajectoryMsg {
        let (times, joint_positions) = traj.resample(1.0 / 30.0);
        let link_poses = joint_positions
            .iter()
            .map(|q| {
                botrail_kin::forward_kinematics(&self.scene.robot, q)
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
}

/// Wall clock in ms (std::time::Instant panics on wasm32-unknown-unknown).
fn now_ms() -> f64 {
    js_sys::Date::now()
}

fn web_log(text: &str) {
    web_sys_log(text);
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console, js_name = warn)]
    fn web_sys_log(s: &str);
}
