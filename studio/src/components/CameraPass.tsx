import { useEffect, useMemo } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";

import { useStudioStore } from "../store";
import { colorPipeline } from "../three/colorPipeline";
import {
  aimSensorCamera,
  cameraRig,
  hideAids,
  renderDepthPip,
  restoreAids,
} from "../three/cameraRig";

/**
 * The second render pass: while the PiP is open, draw the sensor camera's
 * view into a scissored inset of the same canvas.
 *
 * Owns the main RGB output even with the PiP closed, so all camera paths
 * share the same colour transform. The inset hides authoring aids only.
 */
export function CameraPass() {
  const msg = useStudioStore((s) =>
    s.pipCamera
      ? (s.cameras.find((c) => c.name === s.pipCamera) ?? null)
      : null,
  );
  const sensorCam = useMemo(() => new THREE.PerspectiveCamera(), []);
  const size = useMemo(() => new THREE.Vector2(), []);
  useEffect(() => {
    if (!msg && cameraRig.ctx) colorPipeline(cameraRig.ctx.gl).release("pip");
  }, [msg]);

  useFrame(({ gl, scene, camera }) => {
    // The exporter owns the canvas (and its buffer size) while it runs.
    if (useStudioStore.getState().camExport) return;
    // Main view first — full viewport, no scissor. This also runs
    // scene.updateMatrixWorld(), so the camera node's transform below is
    // current even mid-playback (the driver runs at priority 0).
    gl.getSize(size);
    gl.setScissorTest(false);
    gl.setViewport(0, 0, size.x, size.y);
    const pipeline = colorPipeline(gl);
    const dpr = gl.getPixelRatio();
    pipeline.render(gl, scene, camera, gl.domElement.width, gl.domElement.height, "view");

    if (!msg) return;
    const rect = cameraRig.pipRect;
    const node = cameraRig.nodes.get(msg.name);
    if (!rect || !node || rect.w < 2 || rect.h < 2) return;

    aimSensorCamera(sensorCam, msg, node);
    const saved = hideAids();
    try {
      if (useStudioStore.getState().pipMode === "depth") {
        renderDepthPip(gl, scene, sensorCam, rect, size.y);
      } else {
        // WebGL rects are bottom-left origin; the DOM rect is top-left.
        const y = size.y - rect.y - rect.h;
        gl.setScissorTest(true);
        gl.setScissor(rect.x, y, rect.w, rect.h);
        gl.setViewport(rect.x, y, rect.w, rect.h);
        pipeline.render(gl, scene, sensorCam,
          Math.max(2, Math.round(rect.w * dpr)), Math.max(2, Math.round(rect.h * dpr)), "pip");
      }
    } finally {
      restoreAids(saved);
      gl.setScissorTest(false);
      gl.setViewport(0, 0, size.x, size.y);
    }
  }, 1);

  return null;
}
