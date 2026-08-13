import { useMemo } from "react";
import { Line } from "@react-three/drei";
import * as THREE from "three";

import type { ToolpathOverlayMsg } from "../protocol";
import { useStudioStore } from "../store";

// Same palette as the USD export's BasisCurves: cutting strokes in process
// orange, rapids as thin grey dashes.
const FEED_COLOR = "#e07a34";
const RAPID_COLOR = "#8a8a92";

/**
 * Renders every toolpath overlay the server resolved: feed polylines as
 * solid strokes, rapids dashed. The polylines arrive world-resolved (the
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
    </group>
  );
}
