import * as THREE from "three";

import { useStudioStore } from "../store";
import { robotRig } from "../three/robotRig";
import { yawOf } from "./locomotion";
import { PANEL_ACTIONS, PANEL_SIZE, type PanelAction, buttonCenter } from "./panel";
import { headPose, headWorld, xrRig } from "./rig";
import { pointerLabelText } from "./XrPointerLabel";

/**
 * The XR automation handle, `window.__XR__` — what the headless check
 * (`scripts/xr_screenshots.py`, IWER-emulated headset) reads, the way
 * `__STUDIO__` exposes the store. `report()` is the session as the
 * studio sees it; `aimPose()` / `toReference()` are coordinate helpers
 * for a script that moves the emulator's controllers itself (the
 * emulator's poses are reference-space, Y-up; the cell is world, Z-up).
 * Nothing here drives input, and nothing here runs unless called.
 */
export const xrAutomation = {
  /** XR frames rendered in the current session (0 outside a session). */
  xrFrames: 0,
  presenting: false,
  /** Whether the launch stance has been applied this session. */
  placed: false,
  /** `session.frameRate` once the session reports one. */
  frameRate: null as number | null,
  /** The canvas's antialias attribute — the XR layer's MSAA follows it. */
  antialias: null as boolean | null,
};

export function xrReport() {
  const head = headWorld() ?? new THREE.Vector3();
  const origin = xrRig.origin;
  const s = useStudioStore.getState();
  return {
    presenting: xrAutomation.presenting,
    xrFrames: xrAutomation.xrFrames,
    frameRate: xrAutomation.frameRate,
    antialias: xrAutomation.antialias,
    /** Head in botrail world coordinates (Z-up, metres). */
    headWorld: head.toArray(),
    headHeight: head.z,
    /** The same head in the reference space (what the placements take):
     * the midpoint of the eyes, which follow. */
    headLocal: headPose()?.position.toArray() ?? null,
    eyes: (xrRig.head as THREE.ArrayCamera | null)?.cameras?.map((c) => c.position.toArray()) ?? null,
    origin: origin
      ? {
          position: origin.position.toArray(),
          quaternion: origin.quaternion.toArray(),
          yaw: yawOf(origin.quaternion),
        }
      : null,
    stance: xrAutomation.placed ? xrRig.stance : null,
    teleport: xrRig.teleport,
    hover: s.hover,
    selection: s.selection,
    label: pointerLabelText(s),
    panel: xrRig.panel
      ? { buttons: PANEL_ACTIONS.map((action) => ({ action, world: panelButtonWorld(action) })) }
      : null,
    playback: { has: s.playback !== null, playing: s.playing, speed: s.playbackSpeed, time: s.playbackTime },
    tcp: tcpReport(),
  };
}

/** The TCP handle: whose, where (world), in hand or not, and the verdict it shows. */
function tcpReport() {
  const handle = robotRig.tcpHandle;
  if (!handle || !handle.parent) return null;
  handle.updateWorldMatrix(true, false);
  return {
    link: handle.link,
    grabbed: handle.grabbed,
    world: new THREE.Vector3().setFromMatrixPosition(handle.matrixWorld).toArray(),
    status: { ...handle.status },
    smoothing: handle.smoothing,
  };
}

/**
 * The mouse path's contract, callable: what `TransformControls` does to
 * the handle — take it (no source), move its transform, let go. The
 * headless check drives it on the desktop, where there is no mouse.
 */
export function xrTcpGrab(): boolean {
  const handle = robotRig.tcpHandle;
  if (!handle || !handle.parent) return false;
  handle.grab();
  return true;
}

export function xrTcpMove(world: number[]): boolean {
  const handle = robotRig.tcpHandle;
  if (!handle || !handle.grabbed) return false;
  const local = new THREE.Vector3(...(world as [number, number, number]));
  handle.parent?.worldToLocal(local);
  handle.position.copy(local);
  return true;
}

export function xrTcpRelease(): void {
  robotRig.tcpHandle?.release();
}

/** A panel button's centre in world coordinates (null without a panel). */
function panelButtonWorld(action: PanelAction): number[] | null {
  const panel = xrRig.panel;
  if (!panel) return null;
  panel.updateWorldMatrix(true, false);
  const uv = buttonCenter(action);
  const local = new THREE.Vector3(
    (uv.x - 0.5) * PANEL_SIZE.width,
    (uv.y - 0.5) * PANEL_SIZE.height,
    0,
  );
  return panel.localToWorld(local).toArray();
}

/** A world point in the XR reference space (what the emulator's
 * controllers and head are positioned in). */
export function xrToReference(world: number[]): number[] | null {
  const origin = xrRig.origin;
  if (!origin) return null;
  origin.updateWorldMatrix(true, false);
  return origin.worldToLocal(new THREE.Vector3(...(world as [number, number, number]))).toArray();
}

/** The reference-space pose of a controller held at `from` (reference
 * space) whose ray (−Z) points at the world point `target`. */
export function xrAimPose(
  target: number[],
  from: number[],
): { position: number[]; quaternion: number[] } | null {
  const local = xrToReference(target);
  if (!local) return null;
  const start = new THREE.Vector3(...(from as [number, number, number]));
  const direction = new THREE.Vector3(...(local as [number, number, number])).sub(start).normalize();
  const quaternion = new THREE.Quaternion().setFromUnitVectors(new THREE.Vector3(0, 0, -1), direction);
  return { position: start.toArray(), quaternion: quaternion.toArray() };
}

(window as unknown as { __XR__?: object }).__XR__ = {
  report: xrReport,
  aimPose: xrAimPose,
  toReference: xrToReference,
  tcpGrab: xrTcpGrab,
  tcpMove: xrTcpMove,
  tcpRelease: xrTcpRelease,
};
