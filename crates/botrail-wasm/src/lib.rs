//! Browser-complete botrail session.
//!
//! `WasmSession` speaks exactly the same JSON wire protocol as the Python
//! server: all dispatch and planning logic lives in botrail-session, shared
//! with botrail-py's hub. This crate only supplies the wasm plumbing
//! ([`SessionHost`]: RefCell scene, collected outgoing messages, Date.now
//! clock, console logging) — a wasm session has a single client, so
//! "broadcasting" simply means returning the messages to the caller.
//!
//! Mesh visuals are not served in wasm mode (no mesh I/O yet); the embedded
//! demo robot is primitive-only.

use std::cell::RefCell;
use std::sync::Arc;

use botrail_scene::wire::{self, ServerMessage};
use botrail_scene::Scene;
use botrail_session::SessionHost;
use serde::{Deserialize, Serialize};
use wasm_bindgen::prelude::*;

const DEMO_URDF: &str = include_str!("../../../examples/simple_arm.urdf");

fn to_json(msg: &ServerMessage) -> String {
    serde_json::to_string(msg).expect("wire types serialize infallibly")
}

/// A USD scene decomposed off the main thread: geometry in wire form,
/// mesh collision shapes as VHACD hull point sets. Produced by
/// [`decompose_usd_scene`] (in a Web Worker) and consumed by
/// [`WasmSession::load_prepared_scene`] (on the main thread, cheaply).
#[derive(Serialize, Deserialize)]
struct PreparedScene {
    nodes: Vec<PreparedNode>,
    frames: Vec<wire::FrameMsg>,
    warnings: Vec<String>,
    /// Authored up axis of the source stage ("Y"/"Z"), so the client can
    /// orient its own rendering of the original stage.
    up_axis: String,
}

#[derive(Serialize, Deserialize)]
struct PreparedNode {
    name: String,
    geometry: wire::GeometryMsg,
    pose: wire::PoseMsg,
    #[serde(default)]
    color: Option<[f32; 3]>,
    hulls: Option<Vec<Vec<[f64; 3]>>>,
}

/// Imports a USD stage from bytes and runs the expensive part (composition
/// + VHACD decomposition of mesh collision shapes). Runs in a Web Worker's
/// own wasm instance; the result JSON crosses back to the main thread.
#[wasm_bindgen]
pub fn decompose_usd_scene(
    bytes: Vec<u8>,
    file_name: &str,
    prefix: Option<String>,
) -> Result<String, JsError> {
    let options = botrail_usd::ImportOptions {
        meshes_in_memory: true,
        ..Default::default()
    };
    let imported = botrail_usd::import_usd_bytes(bytes, file_name, &options)
        .map_err(|e| JsError::new(&e.to_string()))?;
    let prefix = prefix.unwrap_or_default();
    let mut no_mesh = |_: &std::path::Path| (String::new(), String::new());
    let up_axis = imported.up_axis.to_string();
    let prepared = PreparedScene {
        up_axis,
        nodes: imported
            .nodes
            .into_iter()
            .map(|node| PreparedNode {
                name: format!("{prefix}{}", node.name),
                geometry: wire::geometry_msg(&node.geometry, &mut no_mesh),
                pose: wire::PoseMsg::from(&node.pose),
                color: node.color,
                hulls: node
                    .mesh_data
                    .as_ref()
                    .map(botrail_collide::mesh::decompose_hulls),
            })
            .collect(),
        frames: imported
            .frames
            .into_iter()
            .map(|f| wire::FrameMsg {
                name: format!("{prefix}{}", f.name),
                pose: wire::PoseMsg::from(&f.pose),
            })
            .collect(),
        warnings: imported.warnings,
    };
    serde_json::to_string(&prepared).map_err(|e| JsError::new(&e.to_string()))
}

/// Single-threaded host: messages are collected and handed back to the
/// JavaScript caller instead of being broadcast.
struct WasmHost {
    scene: RefCell<Scene>,
    out: RefCell<Vec<String>>,
    /// Base URL the robot's USD asset is served from, when the session was
    /// built from one. The studio fetches the stage itself for rendering —
    /// wasm only holds the kinematics — so it needs somewhere to fetch from.
    asset_base: Option<String>,
}

impl SessionHost for WasmHost {
    fn with_scene<R>(&self, f: impl FnOnce(&mut Scene) -> R) -> R {
        f(&mut self.scene.borrow_mut())
    }

    fn robot_asset_url(&self, _robot: usize, path: &std::path::Path) -> Option<String> {
        let base = self.asset_base.as_ref()?;
        let file = path.file_name()?.to_string_lossy();
        Some(format!("{}/{}", base.trim_end_matches('/'), file))
    }

    fn emit(&self, msg: &ServerMessage) {
        self.out.borrow_mut().push(to_json(msg));
    }

    /// std::time::Instant panics on wasm32-unknown-unknown.
    fn now_ms(&self) -> f64 {
        js_sys::Date::now()
    }

    fn log(&self, message: &str) {
        web_log(&format!("botrail-wasm: {message}"));
    }
}

#[wasm_bindgen]
pub struct WasmSession {
    host: WasmHost,
}

#[wasm_bindgen]
impl WasmSession {
    /// Builds a session from a URDF string.
    #[wasm_bindgen(constructor)]
    pub fn new(urdf: &str) -> Result<WasmSession, JsError> {
        let model = botrail_model::RobotModel::from_urdf_str(urdf)
            .map_err(|e| JsError::new(&e.to_string()))?;
        Ok(WasmSession {
            host: WasmHost {
                scene: RefCell::new(Scene::new(Arc::new(model))),
                out: RefCell::new(Vec::new()),
                asset_base: None,
            },
        })
    }

    /// Session with the embedded sample arm (primitive-only 6-DOF).
    pub fn demo() -> Result<WasmSession, JsError> {
        Self::new(DEMO_URDF)
    }

    /// Session built from a USD robot handed over as its full layer set —
    /// `names[i]` is the path layer `blobs[i]` is referenced by, relative to
    /// `root`. The browser has no filesystem and the USD resolver is
    /// synchronous, so the caller downloads the layers and passes them here.
    ///
    /// `asset_base` is where the *same* stage is served from, so the studio
    /// can load it for rendering; the meshes stay in memory on this side.
    /// `instance_name` overrides the scene name the robot goes in under,
    /// which otherwise comes from the asset.
    #[wasm_bindgen(js_name = fromUsdRobot)]
    pub fn from_usd_robot(
        names: Vec<String>,
        blobs: js_sys::Array,
        root: &str,
        articulation_root: Option<String>,
        asset_base: Option<String>,
        instance_name: Option<String>,
    ) -> Result<WasmSession, JsError> {
        if names.len() != blobs.length() as usize {
            return Err(JsError::new("names and blobs must have the same length"));
        }
        let layers: Vec<(String, Vec<u8>)> = names
            .into_iter()
            .zip(blobs.iter())
            .map(|(name, blob)| (name, js_sys::Uint8Array::new(&blob).to_vec()))
            .collect();
        let options = botrail_usd::RobotImportOptions {
            articulation_root,
            ..Default::default()
        };
        let imported = botrail_usd::import_robot_bundle(layers, root, &options)
            .map_err(|e| JsError::new(&e.to_string()))?;
        for warning in &imported.warnings {
            web_log(&format!("botrail-wasm: usd robot import: {warning}"));
        }
        // Link geometry is named `usd:/<prim>`; publish the triangles under
        // those names before anything asks for a collider.
        botrail_collide::mesh::clear_memory_meshes();
        for (path, mesh) in imported.meshes {
            botrail_collide::mesh::register_memory_mesh(path, mesh);
        }
        let mut scene = Scene::new(Arc::new(imported.model));
        if let Some(name) = &instance_name {
            scene.rename_robot(0, name);
        }
        Ok(WasmSession {
            host: WasmHost {
                scene: RefCell::new(scene),
                out: RefCell::new(Vec::new()),
                asset_base,
            },
        })
    }

    /// The connection handshake, in order:
    /// scene_init, obstacles, motions, state. Mesh URLs stay empty (the
    /// default host mapping) — wasm serves no meshes yet.
    pub fn initial_messages(&self) -> Vec<String> {
        botrail_session::initial_messages(&self.host)
            .iter()
            .map(to_json)
            .collect()
    }

    /// Handles one client message; returns the messages a server would
    /// have broadcast in response.
    pub fn handle(&mut self, text: &str) -> Vec<String> {
        botrail_session::handle_client_message(&self.host, text);
        self.host.out.take()
    }

    /// Applies a scene prepared by [`decompose_usd_scene`] (usually in a
    /// Web Worker): rebuilds compounds from the precomputed hulls — cheap —
    /// and registers obstacles and frames. Returns the update messages.
    pub fn load_prepared_scene(&mut self, json: &str) -> Result<Vec<String>, JsError> {
        let prepared: PreparedScene =
            serde_json::from_str(json).map_err(|e| JsError::new(&e.to_string()))?;
        for warning in &prepared.warnings {
            web_log(&format!("botrail-wasm: usd import: {warning}"));
        }
        self.host
            .with_scene(|scene| -> Result<(), String> {
                for node in prepared.nodes {
                    let pose = (&node.pose).into();
                    let added = match node.hulls {
                        Some(hulls) => {
                            let shape = botrail_collide::mesh::compound_from_hulls(&hulls)
                                .map_err(|e| e.to_string())?;
                            scene.add_obstacle_with_collider(
                                &node.name,
                                botrail_model::Geometry::Mesh {
                                    path: format!("usd:/{}", node.name).into(),
                                    scale: nalgebra::Vector3::new(1.0, 1.0, 1.0),
                                },
                                pose,
                                botrail_collide::ObstacleCollider::from_shape(shape),
                            )
                        }
                        None => {
                            let geometry = wire::geometry_from_msg(&node.geometry)
                                .map_err(|e| e.to_string())?;
                            scene
                                .add_obstacle(&node.name, geometry, pose)
                                .map_err(|e| e.to_string())?
                        }
                    };
                    scene
                        .set_obstacle_color(&added, node.color)
                        .map_err(|e| e.to_string())?;
                }
                for frame in prepared.frames {
                    scene.add_frame(&frame.name, (&frame.pose).into());
                }
                Ok(())
            })
            .map_err(|e| JsError::new(&e))?;
        self.emit_scene_updates();
        Ok(self.host.out.take())
    }

    /// Imports a USD stage held in memory (a dropped .usda/.usd/.usdz file;
    /// external references cannot resolve) as static obstacles and named
    /// frames. Mesh collision shapes are VHACD-decomposed in memory — this
    /// can take on the order of a second per mesh on the main thread.
    /// Returns the update messages (obstacles, frames, state).
    pub fn load_usd_scene(
        &mut self,
        bytes: Vec<u8>,
        file_name: &str,
        prefix: Option<String>,
    ) -> Result<Vec<String>, JsError> {
        let options = botrail_usd::ImportOptions {
            meshes_in_memory: true,
            ..Default::default()
        };
        let imported = botrail_usd::import_usd_bytes(bytes, file_name, &options)
            .map_err(|e| JsError::new(&e.to_string()))?;
        for warning in &imported.warnings {
            web_log(&format!("botrail-wasm: usd import: {warning}"));
        }

        let prefix = prefix.unwrap_or_default();
        self.host
            .with_scene(|scene| -> Result<(), String> {
                for node in imported.nodes {
                    let name = format!("{prefix}{}", node.name);
                    let added = match node.mesh_data {
                        Some(mesh) => {
                            let shape = botrail_collide::mesh::mesh_to_compound(&mesh)
                                .map_err(|e| e.to_string())?;
                            scene.add_obstacle_with_collider(
                                &name,
                                node.geometry,
                                node.pose,
                                botrail_collide::ObstacleCollider::from_shape(shape),
                            )
                        }
                        None => scene
                            .add_obstacle(&name, node.geometry, node.pose)
                            .map_err(|e| e.to_string())?,
                    };
                    scene
                        .set_obstacle_color(&added, node.color)
                        .map_err(|e| e.to_string())?;
                }
                for frame in imported.frames {
                    scene.add_frame(&format!("{prefix}{}", frame.name), frame.pose);
                }
                Ok(())
            })
            .map_err(|e| JsError::new(&e))?;

        self.emit_scene_updates();
        Ok(self.host.out.take())
    }
    /// Adds a second (third, …) instance of a robot already in the scene —
    /// the dual-arm case, where both arms are the same asset. The model is
    /// shared, so nothing is re-parsed and no layer bytes have to cross into
    /// wasm again; only the placement differs.
    ///
    /// `source` names the instance to copy (required once several exist).
    /// Returns the whole handshake, not an incremental update: the robot
    /// roster lives in `scene_init`, and that message resets the client's
    /// store, so the content messages have to follow it.
    #[wasm_bindgen(js_name = addRobotInstance)]
    pub fn add_robot_instance(
        &mut self,
        source: Option<String>,
        name: Option<String>,
        base_position: Vec<f64>,
        base_quaternion: Option<Vec<f64>>,
    ) -> Result<Vec<String>, JsError> {
        if base_position.len() != 3 {
            return Err(JsError::new("base_position must be [x, y, z]"));
        }
        let rotation = match &base_quaternion {
            Some(q) if q.len() == 4 => nalgebra::UnitQuaternion::from_quaternion(
                nalgebra::Quaternion::new(q[3], q[0], q[1], q[2]),
            ),
            Some(_) => return Err(JsError::new("base_quaternion must be [x, y, z, w]")),
            None => nalgebra::UnitQuaternion::identity(),
        };
        let base = nalgebra::Isometry3::from_parts(
            nalgebra::Translation3::new(base_position[0], base_position[1], base_position[2]),
            rotation,
        );
        self.insert_robot_instance(source.as_deref(), name.as_deref(), base)
            .map_err(|e| JsError::new(&e))?;
        for msg in botrail_session::initial_messages(&self.host) {
            self.host.emit(&msg);
        }
        Ok(self.host.out.take())
    }
}

impl WasmSession {
    /// The scene half of [`WasmSession::add_robot_instance`], split out so
    /// it is reachable off-browser — composing the handshake reaches for
    /// `Date.now`, which only exists in wasm.
    fn insert_robot_instance(
        &mut self,
        source: Option<&str>,
        name: Option<&str>,
        base: nalgebra::Isometry3<f64>,
    ) -> Result<String, String> {
        self.host.with_scene(|scene| {
            let index = match source {
                Some(name) => scene
                    .robot_index(name)
                    .ok_or_else(|| format!("unknown robot `{name}`"))?,
                None if scene.robots().len() == 1 => 0,
                None => {
                    return Err(format!(
                        "scene has {} robots; pass the one to copy",
                        scene.robots().len()
                    ))
                }
            };
            let model = scene.robots()[index].model.clone();
            Ok(scene.add_robot(model, name, base))
        })
    }

    fn emit_scene_updates(&self) {
        let msgs = self.host.with_scene(|scene| {
            vec![
                wire::obstacles_message(scene, |_| (String::new(), String::new())),
                wire::frames_message(scene),
                wire::state_message(scene),
            ]
        });
        for msg in &msgs {
            self.host.emit(msg);
        }
    }
}

fn web_log(text: &str) {
    web_sys_log(text);
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console, js_name = warn)]
    fn web_sys_log(s: &str);
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The browser's robot path end to end, natively: layers as bytes ->
    /// `import_robot_bundle` -> in-memory mesh registry -> a scene whose
    /// collision checking actually uses that geometry.
    ///
    /// Runs only with `BOTRAIL_ISAAC_DIR` pointing at a downloaded Franka
    /// (same convention as the golden tests). `from_usd_robot` itself takes
    /// `js_sys` types and cannot be called off-browser, so this exercises
    /// exactly what it wraps.
    #[test]
    fn bundled_robot_collides_against_its_in_memory_meshes() {
        let Some(dir) = std::env::var_os("BOTRAIL_ISAAC_DIR").map(std::path::PathBuf::from) else {
            return;
        };
        if !dir.join("franka.usd").exists() {
            return;
        }
        let mut layers = Vec::new();
        let mut stack = vec![dir.clone()];
        while let Some(current) = stack.pop() {
            for entry in std::fs::read_dir(&current).unwrap().flatten() {
                let path = entry.path();
                if path.is_dir() {
                    stack.push(path);
                } else if path.extension().is_some_and(|e| e == "usd" || e == "usda") {
                    let rel = path
                        .strip_prefix(&dir)
                        .unwrap()
                        .to_string_lossy()
                        .to_string();
                    layers.push((rel, std::fs::read(&path).unwrap()));
                }
            }
        }
        let options = botrail_usd::RobotImportOptions {
            articulation_root: Some("/panda".to_string()),
            ..Default::default()
        };
        let imported = botrail_usd::import_robot_bundle(layers, "franka.usd", &options).unwrap();
        assert!(!imported.meshes.is_empty());

        botrail_collide::mesh::clear_memory_meshes();
        for (path, mesh) in imported.meshes {
            botrail_collide::mesh::register_memory_mesh(path, mesh);
        }
        let mut scene = Scene::new(Arc::new(imported.model));

        // Every link that carries geometry got a collider out of the
        // registry — without it the robot would be a set of empty links and
        // could never report a collision.
        let ready = vec![0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785, 0.035, 0.035];
        scene.set_joint_positions(ready).unwrap();
        assert!(scene.check_collisions().is_empty(), "ready pose is clear");

        // Drive the arm into the floor: geometry that exists must collide.
        scene
            .add_obstacle(
                "floor",
                botrail_model::Geometry::Box {
                    size: nalgebra::Vector3::new(4.0, 4.0, 0.1),
                },
                nalgebra::Isometry3::translation(0.0, 0.0, 0.3),
            )
            .unwrap();
        assert!(
            !scene.check_collisions().is_empty(),
            "a slab through the arm must be seen; in-memory meshes did not reach the collider"
        );
    }

    /// The dual-arm browser case: a second instance shares the first one's
    /// model, so no layer bytes cross into wasm twice.
    #[test]
    fn a_second_instance_reuses_the_loaded_model() {
        let mut session = WasmSession::demo().unwrap();
        let added = session
            .insert_robot_instance(
                None,
                Some("b"),
                nalgebra::Isometry3::translation(1.5, 0.0, 0.0),
            )
            .unwrap();
        assert_eq!(added, "b");

        session.host.with_scene(|scene| {
            assert_eq!(
                scene
                    .robots()
                    .iter()
                    .map(|r| r.name.clone())
                    .collect::<Vec<_>>(),
                ["simple_arm", "b"]
            );
            // Shared model, not a re-parse.
            assert!(Arc::ptr_eq(
                &scene.robots()[0].model,
                &scene.robots()[1].model
            ));
            let base = scene.robots()[1].base_pose().translation.vector;
            assert!((base - nalgebra::Vector3::new(1.5, 0.0, 0.0)).norm() < 1e-12);
        });

        // With two robots the source is no longer implied.
        let err = session
            .insert_robot_instance(None, None, nalgebra::Isometry3::identity())
            .unwrap_err();
        assert!(err.contains("pass the one to copy"), "{err}");
    }
}
