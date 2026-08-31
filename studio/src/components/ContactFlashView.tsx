import { useMemo } from "react";
import * as THREE from "three";

import { useStudioStore } from "../store";

/** A cool radial pop, distinct from the weld arc's orange. */
function impactTexture(): THREE.Texture {
  const size = 128;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d")!;
  const g = ctx.createRadialGradient(
    size / 2,
    size / 2,
    0,
    size / 2,
    size / 2,
    size / 2,
  );
  g.addColorStop(0.0, "rgba(245,250,255,1)");
  g.addColorStop(0.25, "rgba(170,215,255,0.85)");
  g.addColorStop(0.55, "rgba(90,160,255,0.35)");
  g.addColorStop(1.0, "rgba(60,120,255,0)");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

/** How long an impact pop stays on screen (playback seconds). */
const FLASH = 0.45;

/**
 * Impact pops for a physics bake: each contact episode flashes once where
 * (and when) it began, sized by its peak force — a part thudding onto the
 * floor reads bigger than a feather landing. Driven from `playbackTime`,
 * which the driver refreshes every UI period while playing and exactly on
 * a seek; a pop spans several of those refreshes, so it reads as an
 * animation either way. Episodes are playback data (they only exist on a
 * baked timeline), so there is nothing to show live.
 */
export function ContactFlashView() {
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const texture = useMemo(impactTexture, []);
  const contacts = timeline?.contacts;
  if (!contacts || contacts.length === 0) return null;
  const live = contacts.filter(
    (c) => playbackTime >= c.start && playbackTime <= c.start + FLASH,
  );
  return (
    <>
      {live.map((c, i) => {
        const u = (playbackTime - c.start) / FLASH;
        // log-ish force scaling: 1 N → ~0.07, 10 N → ~0.11, 100 N → ~0.15
        const strength = 0.07 + 0.04 * Math.log10(1 + Math.max(c.peak_force, 0));
        const scale = strength * (0.6 + 1.2 * u);
        const opacity = 1 - u * u;
        return (
          <sprite
            key={`${c.a}|${c.b}|${c.start}|${i}`}
            position={[c.position[0], c.position[1], c.position[2]]}
            scale={[scale, scale, scale]}
          >
            <spriteMaterial
              map={texture}
              blending={THREE.AdditiveBlending}
              depthWrite={false}
              transparent
              opacity={opacity}
            />
          </sprite>
        );
      })}
    </>
  );
}
