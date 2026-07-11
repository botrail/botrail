// Session backends: the UI talks to `SessionBackend` only, never to a
// transport directly. Two implementations exist — a WebSocket connection to
// the botrail server (default) and an in-browser wasm session (demo mode).

import type { ConnectionStatus } from "./store";

export interface BackendHandlers {
  onMessage(text: string): void;
  onStatus(status: ConnectionStatus): void;
}

export interface SessionBackend {
  start(handlers: BackendHandlers): void;
  send(text: string): void;
}

/** Wasm mode: baked in at build time or forced with `?wasm` for testing. */
export function isWasmMode(): boolean {
  return (
    import.meta.env.VITE_BACKEND === "wasm" ||
    new URLSearchParams(location.search).has("wasm")
  );
}

/** HTTP endpoints (project save/load, python export) need a server. */
export function backendSupportsHttp(): boolean {
  return !isWasmMode();
}
