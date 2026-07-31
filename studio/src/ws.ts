// Client for the studio <-> session feed.
//
// The transport is a `SessionBackend` (WebSocket by default, in-browser
// wasm in demo mode); this module owns message (de)serialization, store
// dispatch, and outgoing-update throttling (~30 Hz).

import { isWasmMode, type SessionBackend } from "./backend";
import { WasmBackend } from "./backend-wasm";
import { WsBackend } from "./backend-ws";
import type {
  ClientMessage,
  GeometryMsg,
  ObstacleMsg,
  PoseMsg,
  SegmentMsg,
  ServerMessage,
} from "./protocol";
import { useStudioStore } from "./store";

const SEND_INTERVAL_MS = 33; // ~30 Hz

let backend: SessionBackend | null = null;

/** Idempotently start the session backend; call once at startup. */
export function startWs(): void {
  if (backend) return;
  backend = isWasmMode() ? new WasmBackend() : new WsBackend();
  backend.start({
    onStatus: (status) => useStudioStore.getState().setConnection(status),
    onMessage: (text) => {
      try {
        const msg = JSON.parse(text) as ServerMessage;
        useStudioStore.getState().applyServerMessage(msg);
      } catch (err) {
        console.error("botrail studio: failed to parse server message", err);
      }
    },
  });
}

function rawSend(msg: ClientMessage): void {
  backend?.send(JSON.stringify(msg));
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

const throttledRobotBasePose = throttled<PoseMsg>(SEND_INTERVAL_MS, (pose) =>
  rawSend({ type: "set_robot_base_pose", pose }),
);

/** Place the robot's root link (world frame), throttled to ~30 Hz. */
export function sendRobotBasePose(pose: PoseMsg): void {
  interact();
  throttledRobotBasePose(pose);
}

/**
 * Wasm mode only: imports a dropped USD file into the in-browser session.
 * Returns false when unsupported (server mode) or on import failure.
 */
export async function dropUsdScene(
  bytes: Uint8Array,
  fileName: string,
): Promise<boolean> {
  if (backend instanceof WasmBackend) {
    return backend.loadUsdScene(bytes, fileName);
  }
  return false;
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

/** Include/exclude an obstacle from collision checking. */
export function sendSetObstacleEnabled(name: string, enabled: boolean): void {
  rawSend({ type: "set_obstacle_enabled", name, enabled });
}

/**
 * Attach an obstacle to a robot link at its current relative pose (a
 * grasp). `link = null` lets the server pick the default TCP link; touch
 * links default to the link's subtree (the gripper).
 */
export function sendAttachObstacle(name: string, link: string | null): void {
  rawSend({ type: "attach_obstacle", name, link, touch_links: null });
}

/** Detach an obstacle; its pose freezes where the robot holds it. */
export function sendDetachObstacle(name: string): void {
  rawSend({ type: "detach_obstacle", name });
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
