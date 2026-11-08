import * as THREE from "three";
import { xrOriginPlacement, xrOriginQuaternion } from "three-usd-robot/xr";

import type { Stance } from "./stance";

/**
 * Where the player stands: the math of teleport, snap turn and recenter,
 * all of it `xrOriginPlacement` on the rotated Z-up origin
 * (`.internal/design-vr.md` §3.6). The pmndrs locomotion helpers write the
 * origin as a Y-up world and would fight the rotation, so the studio owns
 * these few lines. Pure in the origin and the head pose; the React side
 * (`XrLocomotion.tsx`) only decides *when*. Fixed by
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
