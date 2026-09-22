// The mouse pick of a physics viewer (design-physics-pick.md): while the
// world streams live under physics, pointer-down on a body takes it in
// hand — the hit point in the body's frame, and a target that follows the
// mouse on the plane through the hit point facing the camera — and the
// host pulls the body toward the target with a spring until the button
// is released. The viewport only sends; the host says whether the body is
// the engine's to move (`grab`), and the line drawn between the grabbed
// point and the target says so too.

import type { ThreeEvent } from "@react-three/fiber";
import * as THREE from "three";

import { useStudioStore } from "./store";
import { sendDrag, sendRelease } from "./ws";

/** What the viewport hands the pick: the camera controls to hold still
 * during a drag. Registered by `PickBridge` inside the Canvas. */
export const pickRig = {
  controls: null as { enabled: boolean } | null,
};

/** Milliseconds between two `drag` messages of one grab. */
const SEND_PERIOD_MS = 33;
/** How long after letting go a click on the same body is swallowed (it
 * was the end of the drag, not a selection). */
const SWALLOW_CLICK_MS = 250;

interface Pick {
  name: string;
  node: THREE.Object3D;
  local: THREE.Vector3;
  plane: THREE.Plane;
  camera: THREE.Camera;
  canvas: HTMLElement;
  target: THREE.Vector3;
  lastSent: number;
}

let active: Pick | null = null;
let releasedAt = 0;
const raycaster = new THREE.Raycaster();

/** Whether a body under the pointer can be taken in hand right now: the
 * physics world is streaming live. */
export function pickable(): boolean {
  return useStudioStore.getState().bakeStream?.live === true;
}

/** The body in hand and where it is held, for the line the viewport
 * draws; `null` between grabs. */
export function currentPick(): { node: THREE.Object3D; local: THREE.Vector3 } | null {
  return active ? { node: active.node, local: active.local } : null;
}

/** Whether a click that follows a drag on the same body should be
 * ignored: the drag ended in a pointer-up, which the browser also reports
 * as a click. */
export function swallowsClick(): boolean {
  return performance.now() - releasedAt < SWALLOW_CLICK_MS;
}

/** Takes body `name` (rendered by `node`) in hand at the event's hit
 * point. Returns whether the pick began — a pointer event that is not a
 * pick (the world not live, another button, a drag already running)
 * leaves the event for selection and the camera. */
export function beginPick(
  e: ThreeEvent<PointerEvent>,
  name: string,
  node: THREE.Object3D,
): boolean {
  if (!pickable() || active || e.button !== 0) return false;
  const canvas = e.nativeEvent.target as HTMLElement | null;
  if (!canvas) return false;
  e.stopPropagation();
  const point = e.point.clone();
  const local = node.worldToLocal(point.clone());
  // The target moves on the plane through the hit point that faces the
  // camera: the body follows the mouse at the depth it was grabbed at.
  const normal = e.camera.getWorldDirection(new THREE.Vector3());
  const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, point);
  active = {
    name,
    node,
    local,
    plane,
    camera: e.camera,
    canvas,
    target: point.clone(),
    lastSent: 0,
  };
  if (pickRig.controls) pickRig.controls.enabled = false;
  document.body.style.cursor = "grabbing";
  useStudioStore.getState().setDrag({
    name,
    local: [local.x, local.y, local.z],
    target: [point.x, point.y, point.z],
  });
  sendDrag(name, [local.x, local.y, local.z], [point.x, point.y, point.z]);
  active.lastSent = performance.now();
  window.addEventListener("pointermove", onMove);
  window.addEventListener("pointerup", onUp);
  window.addEventListener("pointercancel", onUp);
  return true;
}

function onMove(ev: PointerEvent): void {
  if (!active) return;
  const rect = active.canvas.getBoundingClientRect();
  const ndc = new THREE.Vector2(
    ((ev.clientX - rect.left) / rect.width) * 2 - 1,
    -((ev.clientY - rect.top) / rect.height) * 2 + 1,
  );
  raycaster.setFromCamera(ndc, active.camera);
  const hit = raycaster.ray.intersectPlane(active.plane, new THREE.Vector3());
  if (!hit) return;
  active.target.copy(hit);
  const now = performance.now();
  if (now - active.lastSent < SEND_PERIOD_MS) return;
  active.lastSent = now;
  const { name, local } = active;
  useStudioStore.getState().setDrag({
    name,
    local: [local.x, local.y, local.z],
    target: [hit.x, hit.y, hit.z],
  });
  sendDrag(name, [local.x, local.y, local.z], [hit.x, hit.y, hit.z]);
}

function onUp(): void {
  endPick();
}

/** Lets go: the host releases the body, the camera gets its controls
 * back, the line disappears. Also what the viewport calls when the stream
 * ends under a hand. */
export function endPick(): void {
  if (!active) return;
  window.removeEventListener("pointermove", onMove);
  window.removeEventListener("pointerup", onUp);
  window.removeEventListener("pointercancel", onUp);
  active = null;
  releasedAt = performance.now();
  sendRelease();
  if (pickRig.controls) pickRig.controls.enabled = true;
  document.body.style.cursor = "";
  useStudioStore.getState().setDrag(null);
}
