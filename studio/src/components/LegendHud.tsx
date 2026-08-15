import { useMemo } from "react";
import * as THREE from "three";

import type { LegendMsg } from "../protocol";
import { authoredColor } from "../three/palette";
import { useStudioStore } from "../store";

/**
 * Colour keys for obstacles whose colours mean something — a film map's
 * micron ramp. One card per obstacle carrying a legend, stacked in the
 * viewport's top-right corner, gone when the obstacle goes. Swatches top
 * to bottom as authored; a stop with an empty label draws its swatch and
 * nothing else, which is how a banded ramp labels every other step.
 */
export function LegendHud() {
  const obstacles = useStudioStore((s) => s.obstacles);
  // Identical keys collapse into one card: a progressive film map is a
  // stack of stage obstacles that all carry the same key.
  const legends = useMemo(() => {
    const seen = new Set<string>();
    const out: { name: string; legend: LegendMsg }[] = [];
    for (const o of obstacles) {
      if (!o.legend || !o.visible) continue;
      const key = JSON.stringify(o.legend);
      if (seen.has(key)) continue;
      seen.add(key);
      out.push({ name: o.name, legend: o.legend });
    }
    return out;
  }, [obstacles]);
  if (legends.length === 0) return null;
  return (
    <div className="legend-hud">
      {legends.map(({ name, legend }) => (
        <div className="legend-card" key={name}>
          <div className="legend-title">{legend.title || name}</div>
          <div className="legend-stops">
            {legend.stops.map((stop, i) => (
              <div className="legend-stop" key={i}>
                <span
                  className="legend-swatch"
                  style={{ background: css(stop.color) }}
                />
                <span className="legend-label">{stop.label}</span>
              </div>
            ))}
          </div>
        </div>
      ))}
    </div>
  );
}

/** Wire colours are linear RGB; CSS wants sRGB. */
function css(rgb: [number, number, number]): string {
  const c: THREE.Color = authoredColor(rgb);
  return `#${c.getHexString()}`;
}
