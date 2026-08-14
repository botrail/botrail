import * as THREE from "three";

// A small cycling palette of muted, low-saturation colors so links are
// distinguishable without being loud. Returned as CSS hsl() strings, which
// three's Color/material accept directly.
const HUES = [210, 25, 145, 275, 45, 190, 330, 95];

export function linkColor(index: number): string {
  const hue = HUES[index % HUES.length];
  return `hsl(${hue}, 30%, 56%)`;
}

/**
 * An authored color as three understands it. Files state colors in linear
 * RGB — USD `displayColor` is defined that way and botrail passes URDF
 * material colors through unconverted — so they are handed over as
 * numbers, not through a CSS string, which three would read as sRGB and
 * wash out.
 */
export function authoredColor(rgb: [number, number, number]): THREE.Color {
  const [r, g, b] = rgb;
  return new THREE.Color().setRGB(r, g, b, THREE.LinearSRGBColorSpace);
}

/** Highlight color for anything in collision; mirrors the CSS --bad token. */
export const COLLISION_COLOR = "#e2544c";
