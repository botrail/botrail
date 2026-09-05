import * as THREE from "three";
import { OutputPass } from "three/examples/jsm/postprocessing/OutputPass.js";

type Channel = "view" | "pip" | "capture";

/** One output transform for the viewport, PiP, RGBD and video. Three r169
 * skips tone mapping when rendering a scene into an ordinary target. Keep
 * lighting in linear HDR, then apply tone mapping + sRGB exactly once.
 * Depth never enters this pipeline. Separate targets avoid resizing the
 * main view to the PiP dimensions twice on every frame. */
export class ColorPipeline {
  private targets = new Map<Channel, THREE.WebGLRenderTarget>();
  private output = new OutputPass();
  samples = 2;

  render(
    gl: THREE.WebGLRenderer,
    scene: THREE.Scene,
    camera: THREE.Camera,
    width: number,
    height: number,
    channel: Channel,
    destination: THREE.WebGLRenderTarget | null = null,
  ): void {
    let target = this.targets.get(channel);
    const samples = Math.min(this.samples, gl.capabilities.maxSamples);
    if (target && target.samples !== samples) {
      target.dispose();
      this.targets.delete(channel);
      target = undefined;
    }
    if (!target) {
      target = new THREE.WebGLRenderTarget(width, height, {
        type: THREE.HalfFloatType,
        colorSpace: THREE.LinearSRGBColorSpace,
        samples,
      });
      this.targets.set(channel, target);
    } else if (target.width !== width || target.height !== height) {
      target.setSize(width, height);
    }
    const previous = gl.getRenderTarget();
    try {
      // Render targets carry their own viewport/scissor. The canvas retains
      // the caller's rect, including the PiP scissor, across these switches.
      gl.setRenderTarget(target);
      gl.render(scene, camera);
      this.output.renderToScreen = destination === null;
      this.output.render(gl, destination!, target, 0, false);
    } finally {
      gl.setRenderTarget(previous);
    }
  }

  release(channel: Channel): void {
    this.targets.get(channel)?.dispose();
    this.targets.delete(channel);
  }

  dispose(): void {
    for (const target of this.targets.values()) target.dispose();
    this.targets.clear();
    this.output.dispose();
  }
}

const pipelines = new WeakMap<THREE.WebGLRenderer, ColorPipeline>();
export function colorPipeline(gl: THREE.WebGLRenderer): ColorPipeline {
  let pipeline = pipelines.get(gl);
  if (!pipeline) {
    pipeline = new ColorPipeline();
    pipelines.set(gl, pipeline);
  }
  return pipeline;
}
export function disposeColorPipeline(gl: THREE.WebGLRenderer): void {
  pipelines.get(gl)?.dispose();
  pipelines.delete(gl);
}
