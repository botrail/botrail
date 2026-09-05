/** Viewer preferences only: authored materials and camera resolution stay fixed. */
export const RENDER_QUALITY = {
  performance: { label: "Performance", dpr: 1, shadowSize: 1024, samples: 0 },
  balanced: { label: "Balanced", dpr: 1.5, shadowSize: 2048, samples: 2 },
  high: { label: "High", dpr: 2, shadowSize: 4096, samples: 4 },
} as const;
export type RenderQuality = keyof typeof RENDER_QUALITY;
const KEY = "botrail-studio.render-quality";

export function initialRenderQuality(): RenderQuality {
  try {
    const value = localStorage.getItem(KEY);
    if (value === "performance" || value === "balanced" || value === "high") return value;
  } catch { /* Storage is optional. */ }
  return "balanced";
}

export function persistRenderQuality(value: RenderQuality): RenderQuality {
  try { localStorage.setItem(KEY, value); } catch { /* Storage is optional. */ }
  return value;
}
