import type { Point, View } from "../three/initialView";

/** Where the player stands (feet, on the floor) and what they face. */
export type Stance = { standAt: Point; lookAt: Point };

/**
 * The VR starting position, derived from the launch view (`?view=` /
 * `bt.studio(view=(eye, target))`, else the default orbit view): stand on
 * the floor `backOff` metres from the target's floor point, on the eye's
 * side, facing the target. One view decides both the orbit camera and
 * where a headset wearer starts — no second argument to learn
 * (`.internal/design-vr.md` §3.3).
 */
export function stanceFromView(view: View, backOff = 1.5): Stance {
  const [ex, ey] = view.position;
  const [tx, ty, tz] = view.target;
  let dx = ex - tx;
  let dy = ey - ty;
  const len = Math.hypot(dx, dy);
  if (len < 1e-6) {
    // Looking straight down: stand on the -Y side, where the orbit view
    // defaults to.
    dx = 0;
    dy = -1;
  } else {
    dx /= len;
    dy /= len;
  }
  return {
    standAt: [round(tx + dx * backOff), round(ty + dy * backOff), 0],
    lookAt: [tx, ty, tz],
  };
}

const round = (v: number) => Math.round(v * 1e6) / 1e6;
