// Client for the studio <-> session feed.
//
// The transport is a `SessionBackend` (WebSocket by default, in-browser
// wasm in demo mode); this module owns message (de)serialization, store
// dispatch, and outgoing-update throttling (~30 Hz).

import { isWasmMode, type SessionBackend } from "./backend";
import { WasmBackend } from "./backend-wasm";
import { WsBackend } from "./backend-ws";
import type {
  CameraMsg,
  LidarMsg,
  ClientMessage,
  DeviceMsg,
  GeometryMsg,
  IoBinding,
  IoDecl,
  IoNode,
  IoPointId,
  ObstacleMsg,
  PoseMsg,
  SegmentMsg,
  SensorMsg,
  SequenceMsg,
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

const throttledJointPositions = throttledByKey<number[]>(
  SEND_INTERVAL_MS,
  (robot, positions) =>
    rawSend({ type: "set_joint_positions", robot, positions }),
);

/** Send one robot's full DOF vector, throttled per robot to ~30 Hz. */
export function sendJointPositions(robot: string, positions: number[]): void {
  interact();
  throttledJointPositions(robot, positions);
}

const throttledTcpTarget = throttledByKey<{
  link: string;
  pose: PoseMsg;
  group: string | null;
}>(SEND_INTERVAL_MS, (robot, { link, pose, group }) =>
  rawSend({ type: "set_tcp_target", robot, link, pose, group }),
);

/**
 * Ask the server to IK-track a TCP target pose, throttled to ~30 Hz.
 * `group` names the arm to solve with on a dual-arm robot; null lets the
 * server infer it from the link.
 */
export function sendTcpTarget(
  robot: string,
  target: { link: string; pose: PoseMsg; group?: string | null },
): void {
  interact();
  throttledTcpTarget(robot, { ...target, group: target.group ?? null });
}

const throttledRobotBasePose = throttledByKey<PoseMsg>(
  SEND_INTERVAL_MS,
  (robot, pose) => rawSend({ type: "set_robot_base_pose", robot, pose }),
);

/** Place a robot's root link (world frame), throttled per robot to ~30 Hz. */
export function sendRobotBasePose(robot: string, pose: PoseMsg): void {
  interact();
  throttledRobotBasePose(robot, pose);
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

/** Add an obstacle; the server may rename it and re-broadcasts the full list. */
export function sendAddObstacle(obstacle: ObstacleMsg): void {
  rawSend({ type: "add_obstacle", obstacle });
}

/** Move an obstacle, throttled per-name to ~30 Hz for smooth dragging. */
export const sendUpdateObstaclePose = throttledByKey<PoseMsg>(
  SEND_INTERVAL_MS,
  (name, pose) => rawSend({ type: "update_obstacle_pose", name, pose }),
);

/**
 * Move a whole subtree in one message. Throttled under one key because a
 * group drag is one gesture, and batched because the server rebroadcasts
 * the obstacle list per pose write.
 */
export const sendUpdatePoses = throttledByKey<{
  obstacles: [string, PoseMsg][];
  frames: [string, PoseMsg][];
}>(SEND_INTERVAL_MS, (_key, { obstacles, frames }) =>
  rawSend({ type: "update_poses", obstacles, frames }),
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
 * grasp). `link = null` lets the server pick the default TCP link (the
 * arm's tip when `group` names one); touch links default to the link's
 * subtree (the gripper).
 */
export function sendAttachObstacle(
  name: string,
  robot: string,
  link: string | null,
  group: string | null = null,
): void {
  rawSend({
    type: "attach_obstacle",
    name,
    robot,
    link,
    touch_links: null,
    group,
  });
}

/** Detach an obstacle; its pose freezes where the robot holds it. */
export function sendDetachObstacle(name: string): void {
  rawSend({ type: "detach_obstacle", name });
}

/** Remove an obstacle (sent immediately). */
export function sendRemoveObstacle(name: string): void {
  rawSend({ type: "remove_obstacle", name });
}

/**
 * Append a waypoint segment to a motion; the server creates the motion if
 * missing, owned by `robot` and driving arm `group` (an existing motion
 * keeps both).
 */
export function sendAddSegment(
  motion: string,
  robot: string,
  segment: SegmentMsg,
  group: string | null = null,
): void {
  rawSend({ type: "add_segment", motion, robot, segment, group });
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

/** Add or replace a sequence wholesale (steps are small). */
export function sendUpsertSequence(sequence: SequenceMsg): void {
  rawSend({ type: "upsert_sequence", sequence });
}

/** Remove a sequence (sent immediately). */
export function sendRemoveSequence(name: string): void {
  rawSend({ type: "remove_sequence", name });
}

/** Roll out the sequence; the result arrives as a `sequence_result`.
 * `scenario` runs it under a named initial-state delta. */
export function sendSimulateSequence(name: string, scenario?: string): void {
  rawSend({ type: "simulate_sequence", name, scenario });
}

/** Roll out several sequences as concurrent programs (scan order = list
 * order, like the PLC they model). */
export function sendSimulateSequences(names: string[], scenario?: string): void {
  rawSend({ type: "simulate_sequences", names, scenario });
}

/** Bake the last simulated timeline as a usda layer; the reply
 * (`usd_document`) is saved as a browser download. */
export function sendExportUsd(fps: number): void {
  rawSend({ type: "export_usd", fps });
}

/** Add or replace a pseudo-sensor wholesale. */
export function sendUpsertSensor(sensor: SensorMsg): void {
  rawSend({ type: "upsert_sensor", sensor });
}

/** Remove a pseudo-sensor (sent immediately). */
export function sendRemoveSensor(name: string): void {
  rawSend({ type: "remove_sensor", name });
}

/** Add or replace an auxiliary device wholesale. */
export function sendUpsertDevice(device: DeviceMsg): void {
  rawSend({ type: "upsert_device", device });
}

/** Remove an auxiliary device (sent immediately). */
export function sendRemoveDevice(name: string): void {
  rawSend({ type: "remove_device", name });
}

/** Add or replace a camera wholesale. */
export function sendUpsertCamera(camera: CameraMsg): void {
  rawSend({ type: "upsert_camera", camera });
}

/** Camera upserts throttled per-name to ~30 Hz for smooth gizmo drags. */
export const sendUpsertCameraThrottled = throttledByKey<CameraMsg>(
  SEND_INTERVAL_MS,
  (_name, camera) => rawSend({ type: "upsert_camera", camera }),
);

/** Remove a camera (sent immediately). */
export function sendRemoveCamera(name: string): void {
  rawSend({ type: "remove_camera", name });
}

/** Add or replace a LiDAR scanner wholesale. */
export function sendUpsertLidar(lidar: LidarMsg): void {
  rawSend({ type: "upsert_lidar", lidar });
}

/** One simulated sweep of the named scanner; the reply is a
 * `scan_result` broadcast the store turns into a viewport overlay.
 * `t` sweeps the baked cycle at that instant — pass the playhead
 * whenever a timeline is loaded, so the overlay matches the picture. */
export function sendScanLidar(name: string, t: number | null): void {
  rawSend({ type: "scan_lidar", name, t });
}

/** Lidar upserts throttled per-name to ~30 Hz for smooth gizmo drags. */
export const sendUpsertLidarThrottled = throttledByKey<LidarMsg>(
  SEND_INTERVAL_MS,
  (_name, lidar) => rawSend({ type: "upsert_lidar", lidar }),
);

/** Remove a LiDAR scanner (sent immediately). */
export function sendRemoveLidar(name: string): void {
  rawSend({ type: "remove_lidar", name });
}

// ---- I/O map edits: the assignment layer (nodes, bindings, declarations).
// Validated server-side the way the Python API is; the `io` message comes
// back in full.

export function sendUpsertIoNode(node: IoNode): void {
  rawSend({ type: "upsert_io_node", node });
}

export function sendRemoveIoNode(name: string): void {
  rawSend({ type: "remove_io_node", name });
}

export function sendBindIo(binding: IoBinding): void {
  rawSend({ type: "bind_io", binding });
}

/** Drops `point`'s binding on `node` — on every node when omitted. */
export function sendUnbindIo(point: IoPointId, node?: string): void {
  rawSend({ type: "unbind_io", point, node: node ?? null });
}

export function sendDeclareIo(decl: IoDecl): void {
  rawSend({ type: "declare_io", decl });
}

export function sendUndeclareIo(name: string): void {
  rawSend({ type: "undeclare_io", name });
}

/** Gives every unbound point a channel (`Scene.auto_assign_io`). */
export function sendAutoAssignIo(reassign = false): void {
  rawSend({ type: "auto_assign_io", reassign });
}
