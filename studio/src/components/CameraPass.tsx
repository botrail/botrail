import { useMemo } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";

import type { CameraMsg } from "../protocol";
import { useStudioStore } from "../store";
import {
  aimSensorCamera,
  cameraRig,
  hideAids,
  restoreAids,
} from "../three/cameraRig";

/**
 * The second render pass: while the PiP is open, draw the sensor camera's
 * view into a scissored inset of the same canvas.
 *
 * A `useFrame` subscriber with priority > 0 switches R3F to manual
 * rendering (design/design-camera.md 判断 5) — so this component exists
 * only while a PiP camera is active, renders the main view itself, then
 * the inset, and unmounting hands rendering back to R3F. The inset shares
 * the scene, lights and environment; the authoring aids registered in
 * `cameraRig.helpers` are hidden around its render and restored after.
 */
export function CameraPass() {
  const camera = useStudioStore((s) =>
    s.pipCamera
      ? (s.cameras.find((c) => c.name === s.pipCamera) ?? null)
      : null,
  );
  if (!camera) return null;
  return <PassRunner key={camera.name} msg={camera} />;
}

function PassRunner({ msg }: { msg: CameraMsg }) {
  const sensorCam = useMemo(() => new THREE.PerspectiveCamera(), []);
  const size = useMemo(() => new THREE.Vector2(), []);

  useFrame(({ gl, scene, camera }) => {
    // The exporter owns the canvas (and its buffer size) while it runs.
    if (useStudioStore.getState().camExport) return;
    // Main view first — full viewport, no scissor. This also runs
    // scene.updateMatrixWorld(), so the camera node's transform below is
    // current even mid-playback (the driver runs at priority 0).
    gl.getSize(size);
    gl.setScissorTest(false);
    gl.setViewport(0, 0, size.x, size.y);
    gl.render(scene, camera);

    const rect = cameraRig.pipRect;
    const node = cameraRig.nodes.get(msg.name);
    if (!rect || !node || rect.w < 2 || rect.h < 2) return;

    aimSensorCamera(sensorCam, msg, node);
    const saved = hideAids();
    // WebGL rects are bottom-left origin; the DOM rect is top-left. The
    // scissor bounds the clear, so the inset paints over the main view.
    const y = size.y - rect.y - rect.h;
    gl.setScissorTest(true);
    gl.setScissor(rect.x, y, rect.w, rect.h);
    gl.setViewport(rect.x, y, rect.w, rect.h);
    gl.render(scene, sensorCam);
    restoreAids(saved);
    gl.setScissorTest(false);
    gl.setViewport(0, 0, size.x, size.y);
  }, 1);

  return null;
}
