//! Shared scene state: single source of truth for Python callers and
//! connected studio clients.

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Mutex;

use botrail_kin::{solve_ik, IkMode, IkOptions};
use botrail_model::Geometry;
use botrail_scene::wire::{IkStatusMsg, PoseMsg, SceneDescriptionMsg, ServerMessage};
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
            Err(e) => eprintln!("botrail: unparseable client message: {e}"),
        }
    }
}
