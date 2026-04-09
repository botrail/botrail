// WebSocket client for the studio <-> server feed.
//
// Connects to `/ws` on the same origin, auto-reconnects with exponential
// backoff (capped at 5s), dispatches incoming server messages into the store,
// and throttles outgoing joint updates to ~30 Hz.

import type { ClientMessage, PoseMsg, ServerMessage } from "./protocol";
import { useStudioStore } from "./store";

const MIN_BACKOFF_MS = 500;
const MAX_BACKOFF_MS = 5000;
const SEND_INTERVAL_MS = 33; // ~30 Hz

let socket: WebSocket | null = null;
let backoff = MIN_BACKOFF_MS;
let reconnectTimer: number | null = null;
let started = false;

function wsUrl(): string {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${location.host}/ws`;
}

/** Idempotently open the connection; call once at startup. */
export function startWs(): void {
  if (started) return;
  started = true;
  open();
}

function open(): void {
  useStudioStore.getState().setConnection("connecting");
  const s = new WebSocket(wsUrl());
  socket = s;

  s.onopen = () => {
    backoff = MIN_BACKOFF_MS;
    useStudioStore.getState().setConnection("connected");
  };

  s.onmessage = (ev) => {
    try {
      const msg = JSON.parse(ev.data as string) as ServerMessage;
      useStudioStore.getState().applyServerMessage(msg);
    } catch (err) {
      console.error("botrail studio: failed to parse server message", err);
    }
  };

  s.onclose = () => {
    if (socket === s) socket = null;
    useStudioStore.getState().setConnection("disconnected");
    scheduleReconnect();
  };

  // An error is followed by a close event; let onclose drive reconnection.
  s.onerror = () => s.close();
}

function scheduleReconnect(): void {
  if (reconnectTimer !== null) return;
  const delay = backoff;
  backoff = Math.min(backoff * 2, MAX_BACKOFF_MS);
  reconnectTimer = window.setTimeout(() => {
    reconnectTimer = null;
    open();
  }, delay);
}

function rawSend(msg: ClientMessage): void {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(msg));
  }
}

// --- throttled senders (leading + trailing edge) ---

function throttled<T>(intervalMs: number, send: (value: T) => void): (value: T) => void {
  let pending: T | undefined;
  let lastSent = 0;
  let timer: number | null = null;

  const flush = () => {
    if (timer !== null) {
      clearTimeout(timer);
      timer = null;
    }
    if (pending === undefined) return;
    lastSent = performance.now();
    send(pending);
    pending = undefined;
  };

  return (value: T) => {
    pending = value;
    const elapsed = performance.now() - lastSent;
    if (elapsed >= intervalMs) {
      flush();
    } else if (timer === null) {
      timer = window.setTimeout(flush, intervalMs - elapsed);
    }
  };
}

/** Send the full DOF vector, throttled to ~30 Hz. */
export const sendJointPositions = throttled<number[]>(
  SEND_INTERVAL_MS,
  (positions) => rawSend({ type: "set_joint_positions", positions }),
);

/** Ask the server to IK-track a TCP target pose, throttled to ~30 Hz. */
export const sendTcpTarget = throttled<{ link: string; pose: PoseMsg }>(
  SEND_INTERVAL_MS,
  ({ link, pose }) => rawSend({ type: "set_tcp_target", link, pose }),
);
