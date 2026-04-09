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
        .route("/api/scene", get(scene_handler))
        .fallback_service(ServeDir::new(studio_dir).append_index_html_on_directories(true))
        .with_state(hub)
}

async fn ws_handler(ws: WebSocketUpgrade, State(hub): State<Arc<SceneHub>>) -> Response {
    ws.on_upgrade(move |socket| handle_socket(socket, hub))
}

async fn handle_socket(mut socket: WebSocket, hub: Arc<SceneHub>) {
    let mut rx = hub.tx.subscribe();
    if socket
        .send(Message::Text(hub.scene_init_json().into()))
        .await
        .is_err()
    {
        return;
    }
    if socket
        .send(Message::Text(hub.state_json().into()))
        .await
        .is_err()
    {
        return;
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

async fn mesh_handler(Path(id): Path<usize>, State(hub): State<Arc<SceneHub>>) -> Response {
    let Some(path) = hub.meshes.get(id) else {
        return StatusCode::NOT_FOUND.into_response();
    };
    match tokio::fs::read(path).await {
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
