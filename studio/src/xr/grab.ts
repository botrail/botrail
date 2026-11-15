import * as THREE from "three";

import { type BomIndex, identityOf, inGroup } from "../partIdentity.ts";
import type { Selection } from "../store";

/**
 * Taking a thing of the cell in hand (`.internal/design-vr.md` §3.12):
 * which obstacle a grip is on, what comes with it, and where a held thing
 * is as the hand moves. Pure — three and the bill's index, no React, no
 * store: fixed by `tests/xr_grab.test.mjs`.
 */

/** How far from a thing the grip may be and still take it (metres). */
export const HAND_REACH = 0.1;
/** Distances this close are the same distance: the smaller thing wins. */
const TIE = 0.01;

type Pose = { position: [number, number, number]; quaternion: [number, number, number, number] };

/** An obstacle as a hand meets it: the box of what is drawn of it, in its
 * own frame, and that frame in the world. */
export type Graspable = { name: string; box: THREE.Box3; matrixWorld: THREE.Matrix4 };

/**
 * The box of everything drawn under `root`, in `root`'s own frame — tight
 * however the obstacle is turned, where a world box of a turned conveyor
 * would reach a metre past it. Null when nothing is drawn (a mesh still
 * loading, a stowed part).
 */
export function localBounds(root: THREE.Object3D): THREE.Box3 | null {
  root.updateWorldMatrix(true, true);
  const toRoot = new THREE.Matrix4().copy(root.matrixWorld).invert();
  const relative = new THREE.Matrix4();
  const corner = new THREE.Vector3();
  const box = new THREE.Box3();
  root.traverseVisible((node) => {
    const mesh = node as THREE.Mesh;
    if (!mesh.isMesh || !mesh.geometry) return;
    if (!mesh.geometry.boundingBox) mesh.geometry.computeBoundingBox();
    const b = mesh.geometry.boundingBox;
    if (!b || b.isEmpty()) return;
    relative.multiplyMatrices(toRoot, mesh.matrixWorld);
    for (let i = 0; i < 8; i++) {
      corner
        .set(i & 1 ? b.max.x : b.min.x, i & 2 ? b.max.y : b.min.y, i & 4 ? b.max.z : b.min.z)
        .applyMatrix4(relative);
      box.expandByPoint(corner);
    }
  });
  return box.isEmpty() ? null : box;
}

/**
 * The obstacle the hand is on: the nearest box within reach, and of
 * those as near as each other (a part in a tote, the hand in both) the
 * smaller. A box the head is inside is where one stands — a room, an
 * enclosure — and is never in the hand, or a squeeze in mid-air would
 * carry the building off. Obstacle frames are rigid (position and
 * orientation; sizes live below them), so the distance is taken there.
 */
export function underHand(
  hand: THREE.Vector3,
  head: THREE.Vector3 | null,
  things: readonly Graspable[],
  reach = HAND_REACH,
): string | null {
  const inverse = new THREE.Matrix4();
  const local = new THREE.Vector3();
  const size = new THREE.Vector3();
  const near: { name: string; distance: number; size: number }[] = [];
  for (const thing of things) {
    inverse.copy(thing.matrixWorld).invert();
    if (head && thing.box.containsPoint(local.copy(head).applyMatrix4(inverse))) continue;
    const distance = thing.box.distanceToPoint(local.copy(hand).applyMatrix4(inverse));
    if (!(distance <= reach)) continue;
    near.push({ name: thing.name, distance, size: thing.box.getSize(size).length() });
  }
  if (near.length === 0) return null;
  const nearest = Math.min(...near.map((n) => n.distance));
  return near
    .filter((n) => n.distance <= nearest + TIE)
    .reduce((a, b) => (b.size < a.size ? b : a)).name;
}

/**
 * What comes with the obstacle under the hand: the tree path of the group
 * taken as one body, or null for the obstacle alone. The selection says
 * so when the hand is on it — the selected group whole, the selected
 * obstacle alone, as the gizmo moves them. Otherwise it is the part the
 * bill of materials knows the obstacle as: a workpiece with a line of its
 * own comes alone, a board of a pallet brings the pallet (and the crates
 * on it), a lamp the stack light. An obstacle the bill does not know is
 * itself.
 */
export function handUnit(leaf: string, selection: Selection, index: BomIndex): string | null {
  if (selection.type === "obstacle" && selection.name === leaf) return null;
  if (selection.type === "group" && inGroup(leaf, selection.path)) return selection.path;
  const part = identityOf(index, "obstacle", leaf);
  return part?.inherited && part.ancestor ? `/${part.ancestor}` : null;
}

function poseMatrix(pose: Pose): THREE.Matrix4 {
  return new THREE.Matrix4().compose(
    new THREE.Vector3(...pose.position),
    new THREE.Quaternion(...pose.quaternion),
    new THREE.Vector3(1, 1, 1),
  );
}

/** A pose in the grip's frame: what stays fixed while the thing is held. */
export function holdOffset(grip: THREE.Matrix4, pose: Pose): THREE.Matrix4 {
  return new THREE.Matrix4().copy(grip).invert().multiply(poseMatrix(pose));
}

/** Where a held thing is, the grip being where it is now. */
export function heldPose(grip: THREE.Matrix4, offset: THREE.Matrix4): Pose {
  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  new THREE.Matrix4().multiplyMatrices(grip, offset).decompose(position, quaternion, new THREE.Vector3());
  return {
    position: [position.x, position.y, position.z],
    quaternion: [quaternion.x, quaternion.y, quaternion.z, quaternion.w],
  };
}
