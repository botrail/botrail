import { useEffect, useMemo, useRef } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";
import { ArrayBufferTarget, Muxer } from "webm-muxer";

import { samplePlayback } from "../playback";
import { applySample } from "../playbackRig";
import { useStudioStore } from "../store";
import {
  aimSensorCamera,
  cameraRig,
  hideAids,
  readDepth,
  renderDepthPip,
  restoreAids,
} from "../three/cameraRig";
import { downloadBlob } from "./Header";
import {
  updateFlashes,
  updateSprays,
  updateTraces,
} from "./PlaybackDriver";

/**
 * Camera video export: walks the baked tracks on a deterministic fps grid
 * and encodes what the camera sees into a WebM (VP9, VP8 fallback) that
 * downloads when done (design/design-camera.md 判断 7/8).
 *
 * Per frame it applies the sample through the same imperative path the
 * playback driver uses (`applySample` + the flash/spray/trace updaters),
 * renders the sensor camera to the canvas — whose drawing buffer is
 * temporarily resized to the camera's declared resolution, so tone
 * mapping and sRGB output match the PiP exactly — and hands the pixels to
 * a `VideoEncoder`. One frame per rAF keeps the tab alive and shows the
 * recording as it happens; a priority-2 `useFrame` keeps R3F in manual
 * mode, and PlaybackDriver / CameraPass stand down while `camExport` is
 * set. Same bake, same grid, same file.
 *
 * Depth (design/design-camera.md §12.4 DEP2): `viz: "depth"` films the
 * depth colormap instead of the picture; `depthData` captures metric
 * float32 depth on the same grid and downloads it as a second file.
 */
export function CameraExporter() {
  const job = useStudioStore((s) => s.camExport);
  if (!job) return null;
  return (
    <ExportRunner
      key={`${job.camera}@${job.fps}:${job.viz}:${job.depthData}`}
      job={job}
    />
  );
}

interface Rig {
  muxer: Muxer<ArrayBufferTarget>;
  encoder: VideoEncoder;
  width: number;
  height: number;
  fps: number;
  duration: number;
  total: number;
  frame: number;
  done: boolean;
  failed: string | null;
  /** What the video shows: the camera picture, or its depth colormap. */
  viz: "rgb" | "depth";
  /** Per-frame metric depth (one Blob each, so the bytes live in the
   * browser's blob storage rather than the JS heap), downloaded as a
   * second file after the video; `null` = not capturing depth data
   * (design/design-camera.md §12.4 DEP2). */
  depthParts: Blob[] | null;
  near: number;
  far: number;
  fovDeg: number;
}

interface Job {
  camera: string;
  fps: number;
  viz: "rgb" | "depth";
  depthData: boolean;
}

function ExportRunner({ job }: { job: Job }) {
  const gl = useThree((s) => s.gl);
  const rigRef = useRef<Rig | null>(null);
  const savedView = useRef<{ w: number; h: number; dpr: number } | null>(null);
  const sensorCam = useMemo(() => new THREE.PerspectiveCamera(), []);

  useEffect(() => {
    let cancelled = false;
    const abort = (why: string) => {
      console.error(`camera export: ${why}`);
      useStudioStore.getState().endCamExport();
    };
    const setUp = async () => {
      const s = useStudioStore.getState();
      const msg = s.cameras.find((c) => c.name === job.camera);
      if (!s.playback || !msg) return abort("no baked playback or camera");
      if (typeof VideoEncoder === "undefined") return abort("no WebCodecs");
      // Encoders want even dimensions.
      const width = msg.resolution[0] & ~1;
      const height = msg.resolution[1] & ~1;
      let picked: { codec: string; mux: "V_VP9" | "V_VP8" } | null = null;
      for (const c of [
        { codec: "vp09.00.10.08", mux: "V_VP9" as const },
        { codec: "vp8", mux: "V_VP8" as const },
      ]) {
        const { supported } = await VideoEncoder.isConfigSupported({
          codec: c.codec,
          width,
          height,
        });
        if (supported) {
          picked = c;
          break;
        }
      }
      if (cancelled) return;
      if (!picked) return abort("no supported VP9/VP8 encoder");
      const muxer = new Muxer({
        target: new ArrayBufferTarget(),
        video: { codec: picked.mux, width, height, frameRate: job.fps },
      });
      const rig: Rig = {
        muxer,
        encoder: null as unknown as VideoEncoder,
        width,
        height,
        fps: job.fps,
        duration: s.playback.duration,
        total: Math.max(2, Math.round(s.playback.duration * job.fps) + 1),
        frame: 0,
        done: false,
        failed: null,
        viz: job.viz,
        depthParts: job.depthData ? [] : null,
        near: msg.near,
        far: msg.far,
        fovDeg: msg.fov_deg,
      };
      rig.encoder = new VideoEncoder({
        output: (chunk, meta) => muxer.addVideoChunk(chunk, meta),
        error: (e) => {
          rig.failed = String(e);
        },
      });
      rig.encoder.configure({
        codec: picked.codec,
        width,
        height,
        // ~0.08 bpp — plenty for flat-shaded cells, small files.
        bitrate: Math.max(
          1_000_000,
          Math.min(16_000_000, Math.round(width * height * job.fps * 0.08)),
        ),
        framerate: job.fps,
      });
      // The canvas buffer becomes the camera's film for the duration; CSS
      // size is untouched, so the viewport shows the frames being taken.
      const size = gl.getSize(new THREE.Vector2());
      savedView.current = { w: size.x, h: size.y, dpr: gl.getPixelRatio() };
      gl.setPixelRatio(1);
      gl.setSize(width, height, false);
      rigRef.current = rig;
    };
    void setUp();
    return () => {
      cancelled = true;
      const rig = rigRef.current;
      rigRef.current = null;
      if (rig && rig.encoder.state !== "closed") rig.encoder.close();
      const saved = savedView.current;
      if (saved) {
        savedView.current = null;
        gl.setPixelRatio(saved.dpr);
        gl.setSize(saved.w, saved.h, false);
      }
      // Hand the display back to React at the current playhead.
      const s = useStudioStore.getState();
      if (s.playback) {
        s.setPlayback(s.playbackTime, samplePlayback(s.playback, s.playbackTime));
      }
    };
  }, [gl, job]);

  useFrame(({ scene }) => {
    const rig = rigRef.current;
    if (!rig || rig.done) return;
    const s = useStudioStore.getState();
    if (rig.failed) {
      rig.done = true;
      console.error(`camera export failed: ${rig.failed}`);
      s.endCamExport();
      return;
    }
    // Backpressure: let the encoder drain before producing more frames.
    if (rig.encoder.encodeQueueSize > 4) return;
    const tracks = s.playback;
    const msg = s.cameras.find((c) => c.name === job.camera);
    const node = cameraRig.nodes.get(job.camera);
    if (!tracks || !msg || !node) {
      rig.done = true;
      s.endCamExport();
      return;
    }

    const t = Math.min(rig.frame / rig.fps, rig.duration);
    const sample = samplePlayback(tracks, t);
    applySample(sample);
    updateFlashes(s, sample, t);
    updateSprays(s, sample, t);
    updateTraces(s, sample, t, 1 / rig.fps);

    aimSensorCamera(sensorCam, msg, node);
    const saved = hideAids();
    gl.setScissorTest(false);
    gl.setViewport(0, 0, rig.width, rig.height);
    let depthFrame: Float32Array | null = null;
    try {
      if (rig.viz === "depth") {
        // The colormap covers the (temporarily camera-sized) canvas.
        renderDepthPip(
          gl,
          scene,
          sensorCam,
          { x: 0, y: 0, w: rig.width, h: rig.height },
          rig.height,
        );
        gl.setScissorTest(false);
      } else {
        gl.render(scene, sensorCam);
      }
      if (rig.depthParts) {
        depthFrame = readDepth(gl, scene, sensorCam, rig.width, rig.height);
      }
    } catch (e) {
      rig.failed = String(e);
      return;
    } finally {
      restoreAids(saved);
    }

    try {
      // Same task as the render, so the (non-preserved) buffer is intact.
      const frame = new VideoFrame(gl.domElement, {
        timestamp: Math.round((rig.frame * 1e6) / rig.fps),
        duration: Math.round(1e6 / rig.fps),
      });
      rig.encoder.encode(frame, { keyFrame: rig.frame % (rig.fps * 2) === 0 });
      frame.close();
    } catch (e) {
      rig.failed = String(e);
      return;
    }
    // Only after the video frame went in, so the two streams cannot
    // drift: frame n exists in both files or in neither.
    if (rig.depthParts && depthFrame) {
      // readDepth allocates a plain ArrayBuffer; the cast just narrows
      // the typed array's ArrayBufferLike for Blob's sake.
      rig.depthParts.push(new Blob([depthFrame.buffer as ArrayBuffer]));
    }

    rig.frame += 1;
    if (rig.frame % 5 === 0 || rig.frame >= rig.total) {
      s.setCamExportProgress(rig.frame / rig.total);
    }
    if (rig.frame >= rig.total) {
      rig.done = true;
      void finish(rig, job.camera);
    }
  }, 2);

  return null;
}

async function finish(rig: Rig, camera: string): Promise<void> {
  try {
    await rig.encoder.flush();
    rig.encoder.close();
    rig.muxer.finalize();
    const { buffer } = rig.muxer.target;
    const stem = rig.viz === "depth" ? `cell_${camera}_depth` : `cell_${camera}`;
    downloadBlob(new Blob([buffer], { type: "video/webm" }), `${stem}.webm`);
    if (rig.depthParts) {
      // Metric depth stream: one self-describing blob — a JSON header
      // line, then the frames as raw little-endian float32, in the same
      // order and count as the video's. capture.py turns it into .npz.
      const header = JSON.stringify({
        width: rig.width,
        height: rig.height,
        fps: rig.fps,
        frames: rig.depthParts.length,
        duration: rig.duration,
        near: rig.near,
        far: rig.far,
        fov_deg: rig.fovDeg,
        camera,
      });
      downloadBlob(
        new Blob([header + "\n", ...rig.depthParts], {
          type: "application/octet-stream",
        }),
        `cell_${camera}_depth.bin`,
      );
    }
  } catch (e) {
    console.error(`camera export failed while finalizing: ${e}`);
  }
  useStudioStore.getState().endCamExport();
}
