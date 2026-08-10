import { useEffect, useMemo } from "react";
import * as THREE from "three";

import { playbackRig, signalAt, styleFlash } from "../playbackRig";
import { robotByName, useStudioStore } from "../store";

/** A radial white-hot sprite, generated once (no external assets). */
function flashTexture(): THREE.Texture {
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
  g.addColorStop(0.0, "rgba(255,255,245,1)");
  g.addColorStop(0.18, "rgba(255,235,190,0.95)");
  g.addColorStop(0.45, "rgba(255,180,80,0.5)");
  g.addColorStop(1.0, "rgba(255,140,40,0)");
  ctx.fillStyle = g;
  ctx.fillRect(0, 0, size, size);
  const texture = new THREE.CanvasTexture(canvas);
  texture.colorSpace = THREE.SRGBColorSpace;
  return texture;
}

/**
 * Weld-arc flashes: one sprite + point light per declared flash, parked
 * invisible until the playback driver positions and lights it (the signal
 * that drives it only exists on a baked timeline, so there is nothing to
 * show live). The driver owns per-frame state — this component only
 * mounts the objects and registers them.
 */
export function FlashView() {
  const flashes = useStudioStore((s) => s.flashes);
  const texture = useMemo(flashTexture, []);

  // While *paused* on a baked timeline (a seek, or the end), the arcs
  // still have to reflect the playhead — scrubbing onto a weld shows the
  // arc standing at the TCP. The driver owns the playing case.
  const playing = useStudioStore((s) => s.playing);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const overridePoses = useStudioStore((s) => s.overridePoses);
  const timeline = useStudioStore((s) => s.timeline);
  const robots = useStudioStore((s) => s.robots);
  useEffect(() => {
    if (playing) return;
    for (const flash of flashes) {
      const node = playbackRig.flashes.get(flash.name);
      if (!node) continue;
      const lane = timeline?.signals.find((sig) => sig.name === flash.signal);
      const poses = overridePoses?.[flash.robot];
      const robot = robotByName(robots, flash.robot);
      const tcpName = robot?.desc.tcp_link ?? null;
      const tcpIndex =
        tcpName && robot
          ? robot.desc.links.findIndex((l) => l.name === tcpName)
          : -1;
      const on =
        !!lane &&
        !!poses &&
        tcpIndex >= 0 &&
        signalAt(lane.times, lane.values, playbackTime);
      if (!on || !poses) {
        node.visible = false;
        continue;
      }
      styleFlash(node, poses[tcpIndex], playbackTime);
    }
  }, [playing, playbackTime, overridePoses, timeline, robots, flashes]);

  return (
    <>
      {flashes.map((flash) => (
        <FlashNode key={flash.name} name={flash.name} texture={texture} />
      ))}
    </>
  );
}

function FlashNode({ name, texture }: { name: string; texture: THREE.Texture }) {
  useEffect(() => {
    return () => {
      playbackRig.flashes.delete(name);
    };
  }, [name]);
  return (
    <group
      ref={(group: THREE.Group | null) => {
        if (group) {
          group.visible = false;
          playbackRig.flashes.set(name, group);
        } else {
          playbackRig.flashes.delete(name);
        }
      }}
    >
      <sprite scale={[0.28, 0.28, 0.28]}>
        <spriteMaterial
          map={texture}
          blending={THREE.AdditiveBlending}
          depthWrite={false}
          transparent
        />
      </sprite>
      {/* The light is what sells it: the body panels around the spot catch
          the arc. Decay keeps it local so a 20 m line doesn't glow. */}
      <pointLight color="#ffd9a0" intensity={30} distance={2.2} decay={2} />
    </group>
  );
}
