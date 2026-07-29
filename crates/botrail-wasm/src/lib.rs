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

use botrail_scene::wire::ServerMessage;
use botrail_scene::Scene;
use botrail_session::SessionHost;
use wasm_bindgen::prelude::*;

const DEMO_URDF: &str = include_str!("../../../examples/simple_arm.urdf");

fn to_json(msg: &ServerMessage) -> String {
    serde_json::to_string(msg).expect("wire types serialize infallibly")
}

/// Single-threaded host: messages are collected and handed back to the
/// JavaScript caller instead of being broadcast.
struct WasmHost {
    scene: RefCell<Scene>,
    out: RefCell<Vec<String>>,
}

impl SessionHost for WasmHost {
    fn with_scene<R>(&self, f: impl FnOnce(&mut Scene) -> R) -> R {
        f(&mut self.scene.borrow_mut())
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
            },
        })
    }

    /// Session with the embedded sample arm (primitive-only 6-DOF).
    pub fn demo() -> Result<WasmSession, JsError> {
        Self::new(DEMO_URDF)
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
}

fn web_log(text: &str) {
    web_sys_log(text);
}

#[wasm_bindgen]
extern "C" {
    #[wasm_bindgen(js_namespace = console, js_name = warn)]
    fn web_sys_log(s: &str);
}
