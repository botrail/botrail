import { useEffect, useMemo } from "react";
import { type ThreeEvent, useFrame } from "@react-three/fiber";
import * as THREE from "three";

import { useStudioStore } from "../store";
import { xrActions } from "./actions";
import {
  PANEL_ACTIONS,
  PANEL_PX,
  PANEL_SIZE,
  type PanelState,
  buttonLabel,
  buttonRect,
  hitButton,
  panelText,
  panelTitle,
} from "./panel";
import { xrRig } from "./rig";
import { roundRect } from "./TextLabel";

/**
 * The panel on the left controller (`.internal/design-vr.md` §3.7): the
 * playhead over five buttons — play / pause, speed, `+ point` (a taught
 * waypoint), recenter, exit VR. A
 * canvas-textured quad; a trigger pull on it (the ray pointer's
 * `pointerdown`) maps the hit's UV to a button. It reads the store each
 * frame and redraws only when its words change. A passive view: every
 * press goes through `xrActions`.
 */
export function XrPanel() {
  const panel = useMemo(() => createPanel(), []);
  useEffect(() => {
    xrRig.panel = panel.mesh;
    return () => {
      if (xrRig.panel === panel.mesh) xrRig.panel = null;
      panel.dispose();
    };
  }, [panel]);

  useFrame(() => {
    const s = useStudioStore.getState();
    panel.draw({
      hasPlayback: s.playback !== null,
      playing: s.playing,
      speed: s.playbackSpeed,
      canTeach: s.robots.length > 0,
      time: s.playback ? s.playbackTime : null,
      duration: s.playback?.duration ?? null,
    });
  });

  return (
    <primitive
      object={panel.mesh}
      // Floating over the back of the hand, tilted up toward the eyes.
      position={[0, 0.07, -0.05]}
      rotation={[-Math.PI / 3, 0, 0]}
      onPointerDown={(e: ThreeEvent<PointerEvent>) => {
        e.stopPropagation();
        const uv = e.uv;
        if (!uv) return;
        const action = hitButton(uv);
        if (action) xrActions.run(action);
      }}
    />
  );
}

function createPanel() {
  const canvas = document.createElement("canvas");
  canvas.width = PANEL_PX.width;
  canvas.height = PANEL_PX.height;
  const ctx = canvas.getContext("2d")!;
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.anisotropy = 4;
  const material = new THREE.MeshBasicMaterial({
    map: texture,
    transparent: true,
    depthTest: false,
    toneMapped: false,
    // Readable (and hit by the ray) from either side of the hand.
    side: THREE.DoubleSide,
  });
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(PANEL_SIZE.width, PANEL_SIZE.height), material);
  mesh.renderOrder = 100;
  let last = "";
  const W = canvas.width;
  const H = canvas.height;

  const draw = (state: PanelState) => {
    const text = panelText(state);
    if (text === last) return;
    last = text;
    ctx.clearRect(0, 0, W, H);
    ctx.fillStyle = "rgba(20, 22, 28, 0.88)";
    roundRect(ctx, 2, 2, W - 4, H - 4, 20);
    ctx.fill();
    ctx.fillStyle = "#c9d2e0";
    ctx.font = "500 30px system-ui, sans-serif";
    ctx.textBaseline = "middle";
    ctx.textAlign = "left";
    ctx.fillText(panelTitle(state), 24, H * 0.22, W - 48);
    ctx.textAlign = "center";
    ctx.font = "600 28px system-ui, sans-serif";
    PANEL_ACTIONS.forEach((action, i) => {
      const r = buttonRect(i);
      const x = r.x * W;
      const y = (1 - r.y - r.h) * H;
      const w = r.w * W;
      const h = r.h * H;
      const active = action === "play" && state.playing;
      ctx.fillStyle = active ? "#2f6fb0" : "#2a2f3a";
      roundRect(ctx, x, y, w, h, 14);
      ctx.fill();
      ctx.strokeStyle = "#4a5160";
      ctx.lineWidth = 2;
      ctx.stroke();
      const live =
        action === "recenter" || action === "exit" || (action === "waypoint" ? state.canTeach : state.hasPlayback);
      ctx.fillStyle = live ? "#eef2f8" : "#7c8594";
      ctx.fillText(buttonLabel(action, state), x + w / 2, y + h / 2, w - 12);
    });
    texture.needsUpdate = true;
  };

  return {
    mesh,
    draw,
    dispose() {
      mesh.geometry.dispose();
      material.dispose();
      texture.dispose();
    },
  };
}
