import { useEffect, useRef, type ReactNode } from "react";
import { useThree } from "@react-three/fiber";
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
  /** The live three context, registered by `CameraRigBridge` inside the
   * Canvas — so store actions (depth capture through the automation
   * handle) can render outside the frame loop. */
  ctx: null as { gl: THREE.WebGLRenderer; scene: THREE.Scene } | null,
};

/** Mounted once inside the Canvas; hands the renderer and scene to the
 * imperative side for out-of-frame renders (depth capture). */
export function CameraRigBridge() {
  const gl = useThree((s) => s.gl);
  const scene = useThree((s) => s.scene);
  useEffect(() => {
    cameraRig.ctx = { gl, scene };
    return () => {
      if (cameraRig.ctx?.gl === gl) cameraRig.ctx = null;
    };
  }, [gl, scene]);
  return null;
}

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

// ---------------------------------------------------------------------------
// Depth pass (design/design-camera.md §12): the scene is rendered through
// the sensor camera into a target that carries a float DepthTexture, and a
// fullscreen-triangle blit linearizes the depth buffer — to a turbo
// colormap for the PiP, or to raw view-space meters in a float target for
// metric readback. Depth semantics follow RealSense z16: value = Z along
// the optical axis, 0 = no return (background, or outside [near, far]).

/** Google's polynomial fit of the turbo colormap; the JS twin of the
 * shader's `turbo()` so the PiP legend shows the exact ramp of the
 * picture. `t` in [0, 1] -> `[r, g, b]` in [0, 1]. */
export function turbo(t: number): [number, number, number] {
  const c = Math.min(1, Math.max(0, t));
  const v4 = [1, c, c * c, c * c * c];
  const v2 = [v4[2] * v4[2], v4[3] * v4[2]];
  const dot = (a: number[], b: number[]) =>
    a.reduce((s, x, i) => s + x * b[i], 0);
  return [
    dot(v4, [0.13572138, 4.6153926, -42.66032258, 132.13108234]) +
      dot(v2, [-152.94239396, 59.28637943]),
    dot(v4, [0.09140261, 2.19418839, 4.84296658, -14.18503333]) +
      dot(v2, [4.27729857, 2.82956604]),
    dot(v4, [0.1066733, 12.64194608, -60.58204836, 110.36276771]) +
      dot(v2, [-89.90310912, 27.34824973]),
  ].map((x) => Math.min(1, Math.max(0, x))) as [number, number, number];
}

const DEPTH_VERT = /* glsl */ `
varying vec2 vUv;
void main() {
  vUv = uv;
  gl_Position = vec4(position.xy, 0.0, 1.0);
}
`;

// perspectiveDepthToViewZ comes from three's <packing> chunk. The clear
// value (1.0) means nothing was drawn there; geometry sitting on the far
// plane is folded into "no return" by the epsilon, as z16 would.
const DEPTH_FRAG = /* glsl */ `
#include <packing>
varying vec2 vUv;
uniform sampler2D tDepth;
uniform float cameraNear;
uniform float cameraFar;
uniform float colormap;

vec3 turbo(float t) {
  t = clamp(t, 0.0, 1.0);
  vec4 v4 = vec4(1.0, t, t * t, t * t * t);
  vec2 v2 = v4.zw * v4.z;
  return clamp(vec3(
    dot(v4, vec4(0.13572138, 4.61539260, -42.66032258, 132.13108234)) +
      dot(v2, vec2(-152.94239396, 59.28637943)),
    dot(v4, vec4(0.09140261, 2.19418839, 4.84296658, -14.18503333)) +
      dot(v2, vec2(4.27729857, 2.82956604)),
    dot(v4, vec4(0.10667330, 12.64194608, -60.58204836, 110.36276771)) +
      dot(v2, vec2(-89.90310912, 27.34824973))
  ), 0.0, 1.0);
}

void main() {
  float d = texture2D(tDepth, vUv).x;
  float z = d >= 0.9999999
    ? 0.0
    : -perspectiveDepthToViewZ(d, cameraNear, cameraFar);
  if (colormap > 0.5) {
    // Close = hot, far = cold, no return = black.
    vec3 c = z <= 0.0
      ? vec3(0.0)
      : turbo(1.0 - (z - cameraNear) / (cameraFar - cameraNear));
    gl_FragColor = vec4(c, 1.0);
  } else {
    gl_FragColor = vec4(z, 0.0, 0.0, 1.0);
  }
}
`;

interface DepthRig {
  /** Scene target whose DepthTexture the blit samples. */
  target: THREE.WebGLRenderTarget;
  /** RGBA32F blit target for metric readback; created on first read. */
  read: THREE.WebGLRenderTarget | null;
  blitScene: THREE.Scene;
  blitCamera: THREE.Camera;
  material: THREE.ShaderMaterial;
}

let depthRig: DepthRig | null = null;

/** The (lazily created) depth rig, its scene target sized to `w`x`h`.
 * One rig is shared by the PiP and the readback — sizes differ between
 * the two, but a resize is a rare texture reallocation, not a per-frame
 * cost. Nothing is allocated until a depth picture is first asked for. */
function ensureDepthRig(w: number, h: number): DepthRig {
  if (!depthRig) {
    const material = new THREE.ShaderMaterial({
      vertexShader: DEPTH_VERT,
      fragmentShader: DEPTH_FRAG,
      uniforms: {
        tDepth: { value: null },
        cameraNear: { value: 0.05 },
        cameraFar: { value: 30 },
        colormap: { value: 1 },
      },
      depthTest: false,
      depthWrite: false,
    });
    const geo = new THREE.BufferGeometry();
    geo.setAttribute(
      "position",
      new THREE.Float32BufferAttribute([-1, -1, 0, 3, -1, 0, -1, 3, 0], 3),
    );
    geo.setAttribute(
      "uv",
      new THREE.Float32BufferAttribute([0, 0, 2, 0, 0, 2], 2),
    );
    const tri = new THREE.Mesh(geo, material);
    tri.frustumCulled = false;
    const blitScene = new THREE.Scene();
    blitScene.add(tri);
    depthRig = {
      target: makeSceneTarget(w, h),
      read: null,
      blitScene,
      blitCamera: new THREE.OrthographicCamera(-1, 1, 1, -1, 0, 1),
      material,
    };
  } else if (depthRig.target.width !== w || depthRig.target.height !== h) {
    // Recreate rather than setSize: the attached DepthTexture must follow.
    depthRig.target.depthTexture?.dispose();
    depthRig.target.dispose();
    depthRig.target = makeSceneTarget(w, h);
  }
  return depthRig;
}

function makeSceneTarget(w: number, h: number): THREE.WebGLRenderTarget {
  const depthTexture = new THREE.DepthTexture(w, h);
  depthTexture.type = THREE.FloatType; // DEPTH_COMPONENT32F on WebGL2
  return new THREE.WebGLRenderTarget(w, h, { depthTexture });
}

/** PiP depth mode: renders the scene through `cam` and paints its depth
 * as a turbo colormap into the given canvas rect (CSS pixels, top-left
 * origin — same contract as the RGB inset). Aids must already be hidden
 * by the caller; scissor state is left on, the caller resets it. */
export function renderDepthPip(
  gl: THREE.WebGLRenderer,
  scene: THREE.Scene,
  cam: THREE.PerspectiveCamera,
  rect: { x: number; y: number; w: number; h: number },
  canvasHeight: number,
): void {
  const dpr = gl.getPixelRatio();
  const rig = ensureDepthRig(
    Math.max(2, Math.round(rect.w * dpr)),
    Math.max(2, Math.round(rect.h * dpr)),
  );
  gl.setRenderTarget(rig.target);
  gl.render(scene, cam);
  gl.setRenderTarget(null);
  rig.material.uniforms.tDepth.value = rig.target.depthTexture;
  rig.material.uniforms.cameraNear.value = cam.near;
  rig.material.uniforms.cameraFar.value = cam.far;
  rig.material.uniforms.colormap.value = 1;
  const y = canvasHeight - rect.y - rect.h;
  gl.setScissorTest(true);
  gl.setScissor(rect.x, y, rect.w, rect.h);
  gl.setViewport(rect.x, y, rect.w, rect.h);
  gl.render(rig.blitScene, rig.blitCamera);
}

/** Metric depth: renders the scene through `cam` at `w`x`h` and returns
 * view-space Z in meters, row 0 = the top of the picture, 0 = no return
 * (design/design-camera.md §12.1). Renders to the currently bound state's
 * side (render targets only) and restores the null target; safe to call
 * outside the frame loop. Throws where float color buffers are missing
 * (the D3 fallback is added if a platform ever shows up without them). */
export function readDepth(
  gl: THREE.WebGLRenderer,
  scene: THREE.Scene,
  cam: THREE.PerspectiveCamera,
  w: number,
  h: number,
): Float32Array {
  const ctx = gl.getContext() as WebGL2RenderingContext;
  if (!ctx.getExtension("EXT_color_buffer_float")) {
    throw new Error(
      "depth capture needs float render targets (EXT_color_buffer_float)",
    );
  }
  const rig = ensureDepthRig(w, h);
  if (!rig.read || rig.read.width !== w || rig.read.height !== h) {
    rig.read?.dispose();
    rig.read = new THREE.WebGLRenderTarget(w, h, {
      type: THREE.FloatType,
      minFilter: THREE.NearestFilter,
      magFilter: THREE.NearestFilter,
      depthBuffer: false,
      colorSpace: THREE.NoColorSpace,
    });
  }
  gl.setRenderTarget(rig.target);
  gl.render(scene, cam);
  rig.material.uniforms.tDepth.value = rig.target.depthTexture;
  rig.material.uniforms.cameraNear.value = cam.near;
  rig.material.uniforms.cameraFar.value = cam.far;
  rig.material.uniforms.colormap.value = 0;
  gl.setRenderTarget(rig.read);
  gl.render(rig.blitScene, rig.blitCamera);
  const rgba = new Float32Array(w * h * 4);
  gl.readRenderTargetPixels(rig.read, 0, 0, w, h, rgba);
  gl.setRenderTarget(null);
  // R channel only, flipped: readPixels rows run bottom-up.
  const out = new Float32Array(w * h);
  for (let row = 0; row < h; row++) {
    const src = (h - 1 - row) * w * 4;
    const dst = row * w;
    for (let col = 0; col < w; col++) out[dst + col] = rgba[src + col * 4];
  }
  return out;
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
