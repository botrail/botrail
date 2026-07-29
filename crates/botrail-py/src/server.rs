//! Axum router serving the studio SPA, mesh files, and the websocket feed.

use std::path::PathBuf;
use std::sync::Arc;

use axum::extract::ws::{Message, WebSocket};
use axum::extract::{Path, State, WebSocketUpgrade};
use axum::http::{header, StatusCode};
use axum::response::{IntoResponse, Response};
use axum::routing::get;
use axum::Router;
use tower_http::services::ServeDir;

use crate::hub::SceneHub;

pub fn router(hub: Arc<SceneHub>, studio_dir: PathBuf) -> Router {
    Router::new()
        .route("/ws", get(ws_handler))
        .route("/meshes/{id}", get(mesh_handler))
        .route("/usd-assets/{*rest}", get(asset_handler))
        .route("/api/scene", get(scene_handler))
        .route("/api/project", get(project_get).post(project_post))
        .route("/api/export.py", get(python_export))
        .fallback_service(ServeDir::new(studio_dir).append_index_html_on_directories(true))
        .with_state(hub)
}

async fn ws_handler(ws: WebSocketUpgrade, State(hub): State<Arc<SceneHub>>) -> Response {
    ws.on_upgrade(move |socket| handle_socket(socket, hub))
}

async fn handle_socket(mut socket: WebSocket, hub: Arc<SceneHub>) {
    let mut rx = hub.tx.subscribe();
    for initial in [
        hub.scene_init_json(),
        hub.obstacles_json(),
        hub.motions_json(),
        hub.frames_json(),
        hub.state_json(),
    ] {
        if socket.send(Message::Text(initial.into())).await.is_err() {
            return;
        }
    }
    loop {
        tokio::select! {
            broadcast = rx.recv() => match broadcast {
                Ok(text) => {
                    if socket.send(Message::Text(text.into())).await.is_err() {
                        break;
                    }
                }
                // Slow client: skip the backlog and resync to the latest state.
                Err(tokio::sync::broadcast::error::RecvError::Lagged(_)) => {
                    if socket.send(Message::Text(hub.state_json().into())).await.is_err() {
                        break;
                    }
                }
                Err(tokio::sync::broadcast::error::RecvError::Closed) => break,
            },
            incoming = socket.recv() => match incoming {
                Some(Ok(Message::Text(text))) => hub.handle_client_message(&text),
                Some(Ok(Message::Close(_))) | Some(Err(_)) | None => break,
                Some(Ok(_)) => {}
            },
        }
    }
}

/// Serves a USD robot's stage directory so the client-side loader can
/// fetch the root layer and its relative references.
async fn asset_handler(Path(rest): Path<String>, State(hub): State<Arc<SceneHub>>) -> Response {
    let Some(dir) = hub.robot_asset_dir() else {
        return StatusCode::NOT_FOUND.into_response();
    };
    // Reject path traversal; USD-internal references are always relative
    // and slash-separated.
    if rest
        .split('/')
        .any(|c| c.is_empty() || c == "." || c == "..")
    {
        return StatusCode::NOT_FOUND.into_response();
    }
    match tokio::fs::read(dir.join(&rest)).await {
        Ok(bytes) => (
            [
                (header::CONTENT_TYPE, "application/octet-stream"),
                (header::CACHE_CONTROL, "no-cache"),
            ],
            bytes,
        )
            .into_response(),
        Err(_) => StatusCode::NOT_FOUND.into_response(),
    }
}

async fn mesh_handler(Path(id): Path<usize>, State(hub): State<Arc<SceneHub>>) -> Response {
    let Some(path) = hub.mesh_path(id) else {
        return StatusCode::NOT_FOUND.into_response();
    };
    match tokio::fs::read(&path).await {
        Ok(bytes) => (
            [
                (header::CONTENT_TYPE, "application/octet-stream"),
                (header::CACHE_CONTROL, "max-age=3600"),
            ],
            bytes,
        )
            .into_response(),
        Err(e) => {
            eprintln!("botrail: failed to read mesh {}: {e}", path.display());
            StatusCode::NOT_FOUND.into_response()
        }
    }
}

/// Same payload as the websocket `scene_init` message; handy for curl/tests.
async fn scene_handler(State(hub): State<Arc<SceneHub>>) -> Response {
    (
        [(header::CONTENT_TYPE, "application/json")],
        hub.scene_init_json(),
    )
        .into_response()
}

/// Downloads the current scene as a `.botrail` project file.
async fn project_get(State(hub): State<Arc<SceneHub>>) -> Response {
    (
        [
            (header::CONTENT_TYPE, "application/json"),
            (
                header::CONTENT_DISPOSITION,
                "attachment; filename=\"project.botrail\"",
            ),
        ],
        hub.project_json(),
    )
        .into_response()
}

/// Applies an uploaded `.botrail` project to the running scene (robot must
/// match; obstacles, motions, and joint state are replaced).
async fn project_post(State(hub): State<Arc<SceneHub>>, body: String) -> Response {
    match hub.apply_project_json(&body) {
        Ok(()) => StatusCode::OK.into_response(),
        Err(e) => (StatusCode::BAD_REQUEST, e).into_response(),
    }
}

/// Downloads a generated Python script reproducing the current project.
async fn python_export(State(hub): State<Arc<SceneHub>>) -> Response {
    (
        [
            (header::CONTENT_TYPE, "text/x-python; charset=utf-8"),
            (
                header::CONTENT_DISPOSITION,
                "attachment; filename=\"scene.py\"",
            ),
        ],
        hub.python_code(),
    )
        .into_response()
}
