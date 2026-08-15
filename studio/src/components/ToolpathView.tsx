import { useMemo } from "react";
import { Line } from "@react-three/drei";
import * as THREE from "three";

import type { PathMarkMsg, ToolpathOverlayMsg } from "../protocol";
import { useStudioStore } from "../store";

// Same palette as the USD export's BasisCurves: cutting strokes in process
// orange, rapids as thin grey dashes.
const FEED_COLOR = "#e07a34";
const RAPID_COLOR = "#8a8a92";

/**
 * What a face check flagged, by kind. Reach findings (check_toolpath) in
 * the collision red family; paint findings (check_paint) each their own,
 * so "too far" and "too close" read apart at a glance. Off-target is the
 * one non-failure — a raster is supposed to run past the part — and gets
 * a muted neutral so it shows where the trigger would go without
 * shouting. Unknown tags fall back to the neutral too.
 */
const MARK_COLORS: Record<string, string> = {
  unreachable: "#e2544c",
  config_jump: "#f0a04b",
  collision: "#e2544c",
  too_far: "#5da9ff",
  too_close: "#f25c8a",
  oblique: "#e5c04a",
  no_target: "#6b6f7a",
};
const MARK_FALLBACK = "#9aa0ad";

/** Kinds that draw small: informational, not failures. */
const QUIET_KINDS = new Set(["no_target"]);

/**
 * Renders every toolpath overlay the server resolved: feed polylines as
 * solid strokes, rapids dashed, and the marks the last check left on the
 * path as points over them. The polylines arrive world-resolved (the
 * server re-sends them when a part frame moves), so this is a pure draw.
 */
export function ToolpathView() {
  const toolpaths = useStudioStore((s) => s.toolpaths);
  if (toolpaths.length === 0) return null;
  return (
    <>
      {toolpaths.map((tp) => (
        <Overlay key={tp.name} overlay={tp} />
      ))}
    </>
  );
}

function Overlay({ overlay }: { overlay: ToolpathOverlayMsg }) {
  const strokes = useMemo(() => {
    const toVectors = (lines: number[][][]) =>
      lines
        .filter((line) => line.length >= 2)
        .map((line) => line.map(([x, y, z]) => new THREE.Vector3(x, y, z)));
    return {
      feed: toVectors(overlay.feed as unknown as number[][][]),
      rapid: toVectors(overlay.rapid as unknown as number[][][]),
    };
  }, [overlay]);
  // Overlay, not geometry: a cutting stroke lies *inside* the stock (that
  // is what cutting depth means), so depth-tested lines would be swallowed
  // by the plate. Drawn gizmo-style on top of everything instead.
  return (
    <group>
      {strokes.feed.map((points, i) => (
        <Line
          key={`f${i}`}
          points={points}
          color={FEED_COLOR}
          lineWidth={2.5}
          depthTest={false}
          renderOrder={10}
        />
      ))}
      {strokes.rapid.map((points, i) => (
        <Line
          key={`r${i}`}
          points={points}
          color={RAPID_COLOR}
          lineWidth={1.5}
          dashed
          dashSize={0.02}
          gapSize={0.012}
          depthTest={false}
          renderOrder={10}
        />
      ))}
      <Marks marks={overlay.marks ?? []} />
    </group>
  );
}

/**
 * The findings, one point cloud per kind so each carries its own colour
 * and size. Points rather than spheres: a check can flag thousands of
 * samples, and a `THREE.Points` draws them in one call.
 */
function Marks({ marks }: { marks: PathMarkMsg[] }) {
  const groups = useMemo(() => {
    const byKind = new Map<string, number[]>();
    for (const m of marks) {
      const list = byKind.get(m.kind) ?? [];
      list.push(m.position[0], m.position[1], m.position[2]);
      byKind.set(m.kind, list);
    }
    return Array.from(byKind.entries()).map(([kind, flat]) => {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute(
        "position",
        new THREE.Float32BufferAttribute(flat, 3),
      );
      const quiet = QUIET_KINDS.has(kind);
      const material = new THREE.PointsMaterial({
        color: MARK_COLORS[kind] ?? MARK_FALLBACK,
        size: quiet ? 3 : 7,
        sizeAttenuation: false,
        depthTest: false,
        transparent: true,
        opacity: quiet ? 0.7 : 1.0,
      });
      return { kind, geometry, material };
    });
  }, [marks]);
  if (groups.length === 0) return null;
  return (
    <>
      {groups.map(({ kind, geometry, material }) => (
        <points
          key={kind}
          geometry={geometry}
          material={material}
          renderOrder={12}
        />
      ))}
    </>
  );
}
