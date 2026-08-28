import { useLayoutEffect, useRef, useState } from "react";

import { useStudioStore } from "../store";
import { cameraRig, turbo } from "../three/cameraRig";
import { visionActiveAt } from "./CameraView";

// The legend ramp mirrors the depth pass' colormap: near = hot, far =
// cold (turbo, reversed), built from the same polynomial the shader uses.
const DEPTH_RAMP = `linear-gradient(to right, ${Array.from(
  { length: 9 },
  (_, i) => {
    const [r, g, b] = turbo(1 - i / 8);
    return `rgb(${Math.round(r * 255)}, ${Math.round(g * 255)}, ${Math.round(b * 255)})`;
  },
).join(", ")})`;

const meters = (v: number) => `${Number(v.toFixed(2))} m`;

/**
 * The camera picture-in-picture: a DOM frame (header + border) over the
 * viewport's top-right; the picture itself is painted by CameraPass into
 * the same canvas, scissored to this panel's picture area. The panel
 * measures that area and hands the rect to the pass through `cameraRig`.
 *
 * Not one of the mutually-exclusive analysis overlays (SFC/LD/IO/TOPO):
 * a camera picture must coexist with them, so it lives beside the legend
 * HUD and pushes it down via `--pip-reserve`.
 */
export function CameraPip() {
  const cameras = useStudioStore((s) => s.cameras);
  const pipCamera = useStudioStore((s) => s.pipCamera);
  const setPipCamera = useStudioStore((s) => s.setPipCamera);
  const selectCamera = useStudioStore((s) => s.selectCamera);
  const depth = useStudioStore((s) => s.pipMode === "depth");
  const setPipMode = useStudioStore((s) => s.setPipMode);
  const [large, setLarge] = useState(false);
  const pictureRef = useRef<HTMLDivElement | null>(null);
  // A tripped vision sensor looking through this camera lights the frame.
  const live = useStudioStore((s) =>
    pipCamera ? visionActiveAt(s, pipCamera) : false,
  );

  const camera = cameras.find((c) => c.name === pipCamera) ?? null;

  // Keep the pass' scissor rect (and the legend HUD's reserve) in sync
  // with the picture area — on open, resize, aspect or size-toggle.
  useLayoutEffect(() => {
    const el = pictureRef.current;
    if (!el || !camera) return;
    const viewport = el.closest(".viewport") as HTMLElement | null;
    const canvas = viewport?.querySelector("canvas");
    if (!viewport || !canvas) return;
    const update = () => {
      const c = canvas.getBoundingClientRect();
      const r = el.getBoundingClientRect();
      cameraRig.pipRect = {
        x: r.left - c.left,
        y: r.top - c.top,
        w: r.width,
        h: r.height,
      };
      const panel = el.parentElement;
      if (panel) {
        viewport.style.setProperty(
          "--pip-reserve",
          `${Math.ceil(panel.getBoundingClientRect().height) + 10}px`,
        );
      }
    };
    update();
    const ro = new ResizeObserver(update);
    ro.observe(el);
    ro.observe(canvas);
    return () => {
      ro.disconnect();
      cameraRig.pipRect = null;
      viewport.style.removeProperty("--pip-reserve");
    };
  }, [camera, large]);

  if (!camera) return null;
  const width = large ? 480 : 300;
  const [rw, rh] = camera.resolution;
  const aspect = rw > 0 && rh > 0 ? rw / rh : 16 / 9;

  return (
    <div className={live ? "camera-pip camera-pip-live" : "camera-pip"} style={{ width }}>
      <div className="camera-pip-header">
        <span className="camera-pip-icon">🎥</span>
        {cameras.length > 1 ? (
          <select
            value={camera.name}
            onChange={(e) => setPipCamera(e.target.value)}
            title="switch camera"
          >
            {cameras.map((c) => (
              <option key={c.name} value={c.name}>
                {c.name}
              </option>
            ))}
          </select>
        ) : (
          <span className="camera-pip-name">{camera.name}</span>
        )}
        <span className="camera-pip-res">
          {rw}×{rh}
        </span>
        <button
          className={depth ? "camera-pip-mode camera-pip-mode-on" : "camera-pip-mode"}
          onClick={() => setPipMode(depth ? "rgb" : "depth")}
          title={depth ? "show the camera picture" : "show depth"}
        >
          D
        </button>
        <button
          onClick={() => setLarge((v) => !v)}
          title={large ? "smaller" : "larger"}
        >
          {large ? "◱" : "◰"}
        </button>
        <button onClick={() => setPipCamera(null)} title="close">
          ×
        </button>
      </div>
      <div
        ref={pictureRef}
        className="camera-pip-picture"
        style={{ height: Math.round(width / aspect) }}
        title="click to select the camera"
        onClick={() => selectCamera(camera.name)}
      >
        {depth && (
          <div className="camera-pip-depth-legend">
            <span>{meters(camera.near)}</span>
            <div
              className="camera-pip-depth-ramp"
              style={{ background: DEPTH_RAMP }}
            />
            <span>{meters(camera.far)}</span>
          </div>
        )}
      </div>
    </div>
  );
}
