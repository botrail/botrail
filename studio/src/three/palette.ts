import * as THREE from "three";

/**
 * What a machine looks like when nothing has said: unpainted, one neutral
 * shade, exactly the grey `botrail-usd` falls back to (`ENV_COLOR` in
 * `crates/botrail-usd/src/export.rs`), so the studio and the exported
 * stage agree about the same cell.
 *
 * It used to be a cycling per-link palette, which read as the machine's
 * colour and was not: the hue came from the link's index in the
 * *assembled* robot, so the same gripper changed colour depending on
 * which arm it was bolted to. Telling links apart is what the collision
 * highlight and the tree selection are for; a finished cell should look
 * like a machine.
 */
export const UNPAINTED: [number, number, number] = [0.604, 0.639, 0.698];

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
