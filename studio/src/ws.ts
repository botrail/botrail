// WebSocket client for the studio <-> server feed.
//
// Connects to `/ws` on the same origin, auto-reconnects with exponential
// backoff (capped at 5s), dispatches incoming server messages into the store,
// and throttles outgoing joint updates to ~30 Hz.

import type {
  ClientMessage,
  GeometryMsg,
  ObstacleMsg,
  PoseMsg,
  SegmentMsg,
  ServerMessage,
} from "./protocol";
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

/**
 * Like `throttled`, but keeps an independent throttle per key. Dragging one
 * obstacle can't stomp another's pending value, and each key's trailing send
 * carries that key's own last value even after the user moves on.
 */
function throttledByKey<T>(
  intervalMs: number,
  send: (key: string, value: T) => void,
): (key: string, value: T) => void {
  const senders = new Map<string, (value: T) => void>();
  return (key, value) => {
    let s = senders.get(key);
    if (!s) {
      s = throttled<T>(intervalMs, (v) => send(key, v));
      senders.set(key, s);
    }
    s(value);
  };
}

/** Any direct robot interaction ends a running trajectory preview. */
function interact(): void {
  useStudioStore.getState().stopPlayback();
}

const throttledJointPositions = throttled<number[]>(
  SEND_INTERVAL_MS,
  (positions) => rawSend({ type: "set_joint_positions", positions }),
);

/** Send the full DOF vector, throttled to ~30 Hz. */
export function sendJointPositions(positions: number[]): void {
  interact();
  throttledJointPositions(positions);
}

const throttledTcpTarget = throttled<{ link: string; pose: PoseMsg }>(
  SEND_INTERVAL_MS,
  ({ link, pose }) => rawSend({ type: "set_tcp_target", link, pose }),
);

/** Ask the server to IK-track a TCP target pose, throttled to ~30 Hz. */
export function sendTcpTarget(target: { link: string; pose: PoseMsg }): void {
  interact();
  throttledTcpTarget(target);
}

/** Plan from the current configuration to `goal` (DOF order). */
export function sendPlanRequest(goal: number[]): void {
  rawSend({ type: "plan_request", goal_positions: goal });
}

/** Add an obstacle; the server may rename it and re-broadcasts the full list. */
export function sendAddObstacle(obstacle: ObstacleMsg): void {
  rawSend({ type: "add_obstacle", obstacle });
}

/** Move an obstacle, throttled per-name to ~30 Hz for smooth dragging. */
export const sendUpdateObstaclePose = throttledByKey<PoseMsg>(
  SEND_INTERVAL_MS,
  (name, pose) => rawSend({ type: "update_obstacle_pose", name, pose }),
);

/** Resize/reshape an obstacle (sent immediately). */
export function sendUpdateObstacleGeometry(
  name: string,
  geometry: GeometryMsg,
): void {
  rawSend({ type: "update_obstacle_geometry", name, geometry });
}

/** Remove an obstacle (sent immediately). */
export function sendRemoveObstacle(name: string): void {
  rawSend({ type: "remove_obstacle", name });
}

/** Append a waypoint segment to a motion; the server creates it if missing. */
export function sendAddSegment(motion: string, segment: SegmentMsg): void {
  rawSend({ type: "add_segment", motion, segment });
}

/** Remove the segment at `index` from a motion (sent immediately). */
export function sendRemoveSegment(motion: string, index: number): void {
  rawSend({ type: "remove_segment", motion, index });
}

/** Drop every segment from a motion (sent immediately). */
export function sendClearMotion(motion: string): void {
  rawSend({ type: "clear_motion", motion });
}

/** Plan the full motion; the result arrives as a `motion_result`. */
export function sendPlanMotion(motion: string): void {
  rawSend({ type: "plan_motion", motion });
}
