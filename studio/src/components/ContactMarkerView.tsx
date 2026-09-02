import { useMemo } from "react";
import * as THREE from "three";

import { useStudioStore } from "../store";

/** A crisp annotation ring with a centre dot — flat white, tinted by the
 * sprite material. Hard edges on purpose: this is a marker in the gizmo
 * idiom, not a light. */
function ringTexture(): THREE.Texture {
  const size = 128;
  const canvas = document.createElement("canvas");
  canvas.width = size;
  canvas.height = size;
  const ctx = canvas.getContext("2d")!;
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 10;
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, size / 2 - ctx.lineWidth, 0, Math.PI * 2);
  ctx.stroke();
  ctx.fillStyle = "#ffffff";
  ctx.beginPath();
  ctx.arc(size / 2, size / 2, 7, 0, Math.PI * 2);
  ctx.fill();
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

/** How long a marker stays on screen after its touch begins (playback
 * seconds) — long enough to catch the eye across UI refreshes, short
 * enough that a busy cell isn't wallpapered in rings. */
const MARK = 0.45;

/** Palette blue (the lidar/selection blue) — cool "measurement" family,
 * away from the weld arc's process orange. */
const MARK_COLOR = "#5fa8dc";

/**
 * Contact markers for a physics bake: each touch episode is annotated
 * where (and when) it began with a flat ring, sized by its peak force
 * (log-ish: 1 N → ~7 cm, 100 N → ~15 cm) so a part thudding down reads
 * bigger than a feather landing. The ring appears for MARK seconds and
 * goes away — no growth, no fade, no glow: a touch is a *fact* the
 * studio points at, not a light source (which is why this lives among
 * the Aids the camera pass hides, unlike the weld arc a camera would
 * really see). Episodes are playback data (they only exist on a baked
 * timeline), so there is nothing to show live.
 */
export function ContactMarkerView() {
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const texture = useMemo(ringTexture, []);
  const contacts = timeline?.contacts;
  if (!contacts || contacts.length === 0) return null;
  const live = contacts.filter(
    (c) => playbackTime >= c.start && playbackTime <= c.start + MARK,
  );
  return (
    <>
      {live.map((c, i) => {
        // log-ish force scaling: 1 N → ~4.5 cm, 100 N → ~9 cm. Kept
        // subordinate to the geometry — an annotation, not a billboard.
        const scale = 0.04 + 0.025 * Math.log10(1 + Math.max(c.peak_force, 0));
        return (
          <sprite
            key={`${c.a}|${c.b}|${c.start}|${i}`}
            position={[c.position[0], c.position[1], c.position[2]]}
            scale={[scale, scale, scale]}
            renderOrder={10}
          >
            <spriteMaterial
              map={texture}
              color={MARK_COLOR}
              depthTest={false}
              depthWrite={false}
              transparent
              opacity={0.85}
            />
          </sprite>
        );
      })}
    </>
  );
}
