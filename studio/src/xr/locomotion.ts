import * as THREE from "three";
import { xrOriginPlacement, xrOriginQuaternion } from "three-usd-robot/xr";

import type { Stance } from "./stance";

/**
 * Where the player stands: the math of teleport, snap turn and recenter,
 * all of it `xrOriginPlacement` on the rotated Z-up origin, and of the
 * glide — walking with the stick — which slides that origin across the
 * floor (`.internal/design-vr.md` §3.6). The pmndrs locomotion helpers
 * write the origin as a Y-up world and would fight the rotation, so the
 * studio owns these few lines. Pure in the origin and the head pose; the
 * React side (`XrLocomotion.tsx`) only decides *when*. Fixed by
 * `tests/xr_locomotion.test.mjs`.
 */

export const WORLD_UP = "Z" as const;

/** The studio's floor plane is a 48 m square around the origin; a landing
 * outside it has nothing to stand on. */
export const FLOOR_HALF = 24;

/** A snap turn's step. */
export const SNAP_TURN = Math.PI / 4;

/** The teleport arc: a ball thrown along the controller's ray. */
export const ARC = {
  /** Launch speed, m/s — about an 8 m reach from standing height. */
  speed: 7,
  gravity: 9.81,
  /** Line segments drawn. */
  steps: 24,
  /** Landings farther than this (horizontally) are refused: a hop stays a hop. */
  maxRange: 10,
};

export type Arc = {
  /** World points from the hand to the landing (empty when nothing lands). */
  points: THREE.Vector3[];
  /** Where the arc meets the floor (z = 0), or null when thrown from below it. */
  hit: THREE.Vector3 | null;
  /** Whether the landing is somewhere the player may stand. */
  valid: boolean;
};

/**
 * The parabola from `from` along `direction` (world, Z-up) down to the
 * floor. A throw from above the floor always lands; one from below it
 * (an origin placed under a mezzanine, say) does not.
 */
export function teleportArc(
  from: THREE.Vector3,
  direction: THREE.Vector3,
  options: Partial<typeof ARC> = {},
): Arc {
  const { speed, gravity, steps, maxRange } = { ...ARC, ...options };
  const dir = direction.clone();
  if (dir.lengthSq() < 1e-12 || from.z <= 0) return { points: [], hit: null, valid: false };
  dir.normalize();
  const vx = dir.x * speed;
  const vy = dir.y * speed;
  const vz = dir.z * speed;
  // z(t) = z0 + vz t − g t² / 2 = 0, the positive root.
  const t = (vz + Math.sqrt(vz * vz + 2 * gravity * from.z)) / gravity;
  const at = (s: number) =>
    new THREE.Vector3(
      from.x + vx * s,
      from.y + vy * s,
      from.z + vz * s - 0.5 * gravity * s * s,
    );
  const points: THREE.Vector3[] = [];
  for (let i = 0; i <= steps; i++) points.push(at((t * i) / steps));
  const hit = at(t);
  hit.z = 0;
  const range = Math.hypot(hit.x - from.x, hit.y - from.y);
  const valid = range <= maxRange && withinFloor(hit);
  return { points, hit, valid };
}

export function withinFloor(point: { x: number; y: number }, half = FLOOR_HALF): boolean {
  return Math.abs(point.x) <= half && Math.abs(point.y) <= half;
}

/** The heading (radians) an origin quaternion carries beyond the Z-up
 * base rotation — what `xrOriginPlacement` calls `yaw`. */
export function yawOf(originQuaternion: THREE.Quaternion): number {
  const turn = xrOriginQuaternion(WORLD_UP).invert().multiply(originQuaternion);
  // A rotation about the local Y axis: (0, sin(yaw/2), 0, cos(yaw/2)).
  let yaw = 2 * Math.atan2(turn.y, turn.w);
  if (yaw > Math.PI) yaw -= 2 * Math.PI;
  if (yaw <= -Math.PI) yaw += 2 * Math.PI;
  return yaw;
}

export type HeadPose = { position: THREE.Vector3; quaternion: THREE.Quaternion };

type Placement = { position: THREE.Vector3; quaternion: THREE.Quaternion; yaw: number };

/** Feet at `hit`, heading kept: the head ends up over the landing. */
export function teleportPlacement(hit: THREE.Vector3, yaw: number, head: HeadPose): Placement {
  return xrOriginPlacement({ worldUp: WORLD_UP, standAt: hit, yaw, head });
}

/** Turn by `delta` about the head's floor point — not about the origin,
 * which may be a step behind the player — so the room pivots under the
 * player instead of swinging them sideways. */
export function snapTurnPlacement(
  headWorld: THREE.Vector3,
  yaw: number,
  delta: number,
  head: HeadPose,
): Placement {
  const standAt = new THREE.Vector3(headWorld.x, headWorld.y, 0);
  return xrOriginPlacement({ worldUp: WORLD_UP, standAt, yaw: yaw + delta, head });
}

/** The launch stance (and a recenter): feet at `standAt`, facing `lookAt`. */
export function stancePlacement(stance: Stance, head: HeadPose): Placement {
  return xrOriginPlacement({
    worldUp: WORLD_UP,
    standAt: new THREE.Vector3(...stance.standAt),
    lookAt: new THREE.Vector3(...stance.lookAt),
    head,
  });
}

/** Writes a placement onto the origin group. */
export function applyPlacement(origin: THREE.Object3D, placement: Placement): void {
  origin.position.copy(placement.position);
  origin.quaternion.copy(placement.quaternion);
  origin.updateMatrixWorld(true);
}

// ------------------------------------------------------------------ glide

/** Gliding: the left stick walks the player across the floor. */
export const GLIDE = {
  /** Full-stick speed, m/s — a brisk walk. The stick is analog below it. */
  speed: 2,
  /** Stick travel around the centre that asks for nothing: a resting
   * stick is never exactly 0. */
  deadzone: 0.15,
  /** Time constant of the velocity's ease toward what the stick asks, s —
   * long enough that starting and stopping are not steps, short enough
   * that the floor is not ice. */
  ease: 0.1,
  /** With the stick centred, a glide slower than this is over, m/s. */
  rest: 0.01,
  /** The longest frame a step is integrated over, s: a hitch must not
   * become a jump. */
  maxStep: 0.1,
};

export type Vec2 = { x: number; y: number };

/**
 * What a thumbstick asks for, as a vector of length 0…1 (`x` to the right,
 * `y` forward): a radial dead zone, the rest of the travel rescaled to
 * start at 0 and squared, so a light push creeps and only the rim runs.
 */
export function stickDemand(x: number, forward: number, deadzone = GLIDE.deadzone): Vec2 {
  const m = Math.hypot(x, forward);
  if (!Number.isFinite(m) || m <= deadzone) return { x: 0, y: 0 };
  const t = Math.min(1, (m - deadzone) / (1 - deadzone));
  const gain = (t * t) / m;
  return { x: x * gain, y: forward * gain };
}

/**
 * The floor direction a head faces (unit, world XY); null only for a
 * pose that is not one. Read off the ears' axis rather than the gaze:
 * pitching the head — down at a fixture, up at a gantry — leaves that
 * axis where it was, so the heading neither shrinks nor flips with the
 * gaze.
 */
export function floorHeading(headWorldQuaternion: THREE.Quaternion): Vec2 | null {
  // Z-up, right-handed: forward = up × right.
  const right = new THREE.Vector3(1, 0, 0).applyQuaternion(headWorldQuaternion);
  let x = -right.y;
  let y = right.x;
  if (!(Math.hypot(x, y) >= 0.1)) {
    // An ear on the shoulder: the gaze, level by then, decides.
    const gaze = new THREE.Vector3(0, 0, -1).applyQuaternion(headWorldQuaternion);
    x = gaze.x;
    y = gaze.y;
  }
  const length = Math.hypot(x, y);
  if (!(length > 0) || !Number.isFinite(length)) return null;
  return { x: x / length, y: y / length };
}

/**
 * One frame of gliding. The velocity (world XY, m/s) eases exponentially
 * toward what the stick asks along `heading`, and the step is that curve
 * integrated over `dt` — exactly, so neither the speed nor the distance
 * covered depends on the frame rate. `from` (the head's floor point)
 * keeps the step on the floor: an axis that would leave it stops there.
 */
export function glideStep(
  velocity: Vec2,
  demand: Vec2,
  heading: Vec2 | null,
  dt: number,
  from?: Vec2,
): { velocity: Vec2; step: Vec2 } {
  const h = Number.isFinite(dt) ? Math.min(Math.max(dt, 0), GLIDE.maxStep) : 0;
  // The right of the heading, seen from above, is (heading.y, −heading.x).
  const target = heading
    ? {
        x: (heading.x * demand.y + heading.y * demand.x) * GLIDE.speed,
        y: (heading.y * demand.y - heading.x * demand.x) * GLIDE.speed,
      }
    : { x: 0, y: 0 };
  // v(t) = target + (v0 − target) e^(−t/τ) over the frame.
  const blend = 1 - Math.exp(-h / GLIDE.ease);
  const next = {
    x: velocity.x + (target.x - velocity.x) * blend,
    y: velocity.y + (target.y - velocity.y) * blend,
  };
  const step = {
    x: target.x * h + (velocity.x - target.x) * GLIDE.ease * blend,
    y: target.y * h + (velocity.y - target.y) * GLIDE.ease * blend,
  };
  if (target.x === 0 && target.y === 0 && Math.hypot(next.x, next.y) < GLIDE.rest) {
    next.x = 0;
    next.y = 0;
  }
  if (from) {
    for (const axis of ["x", "y"] as const) {
      // The floor's edge — or where the player already is, if that is
      // beyond it: no farther out, but free to walk back in.
      const lo = Math.min(-FLOOR_HALF, from[axis]);
      const hi = Math.max(FLOOR_HALF, from[axis]);
      const to = from[axis] + step[axis];
      if (to < lo || to > hi) {
        step[axis] = (to < lo ? lo : hi) - from[axis];
        next[axis] = 0;
      }
    }
  }
  return { velocity: next, step };
}
