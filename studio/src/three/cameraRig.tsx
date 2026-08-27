import { useRef, type ReactNode } from "react";
import * as THREE from "three";

import type { CameraMsg } from "../protocol";

/**
 * The imperative side of the camera picture (the PiP pass).
 *
 * The sensor-camera render must show the cell as a camera would see it —
 * scenery, robots, process light — and none of the authoring aids (grid,
 * gizmos, sensor volumes, path overlays, the camera bodies themselves).
 * three's layers don't inherit to children, so instead every aid registers
 * its root here and the pass toggles `visible` off around its render
 * (design/design-camera.md 判断 6).
 */
export const cameraRig = {
  /** Authoring-aid roots hidden from the sensor-camera pass. */
  helpers: new Set<THREE.Object3D>(),
  /** Camera name -> the Object3D sitting at its optical frame (-Z looks).
   * Registered by CameraView; the pass reads its world transform, so every
   * mount (world / vehicle / link) and the playback driver's writes are
   * picked up for free. */
  nodes: new Map<string, THREE.Object3D>(),
  /** The PiP picture area in CSS pixels relative to the canvas, written by
   * the DOM panel (CameraPip) and read by the pass for its scissor rect.
   * `null` while the PiP is closed. */
  pipRect: null as { x: number; y: number; w: number; h: number } | null,
};

/** Aims a three camera from a camera message and its optical node: pose
 * from the node's world transform, vertical fov derived from the authored
 * horizontal one, aspect from the declared resolution. Shared by the PiP
 * pass and the video exporter so both pictures are the same picture. */
export function aimSensorCamera(
  cam: THREE.PerspectiveCamera,
  msg: CameraMsg,
  node: THREE.Object3D,
): void {
  node.getWorldPosition(cam.position);
  node.getWorldQuaternion(cam.quaternion);
  const [rw, rh] = msg.resolution;
  const aspect = rw > 0 && rh > 0 ? rw / rh : 16 / 9;
  cam.fov =
    (Math.atan(Math.tan((msg.fov_deg * Math.PI) / 360) / aspect) * 360) /
    Math.PI;
  cam.aspect = aspect;
  cam.near = msg.near;
  cam.far = msg.far;
  cam.updateProjectionMatrix();
}

/** Hides every registered aid, returning a restore list for after the
 * sensor render. */
export function hideAids(): [THREE.Object3D, boolean][] {
  const saved: [THREE.Object3D, boolean][] = [];
  for (const o of cameraRig.helpers) {
    saved.push([o, o.visible]);
    o.visible = false;
  }
  return saved;
}

export function restoreAids(saved: [THREE.Object3D, boolean][]): void {
  for (const [o, v] of saved) o.visible = v;
}

/** Wraps authoring-aid content in a group the sensor pass hides. */
export function Aid({ children }: { children: ReactNode }) {
  const prev = useRef<THREE.Object3D | null>(null);
  return (
    <group
      ref={(node) => {
        if (prev.current) cameraRig.helpers.delete(prev.current);
        if (node) cameraRig.helpers.add(node);
        prev.current = node;
      }}
    >
      {children}
    </group>
  );
}
