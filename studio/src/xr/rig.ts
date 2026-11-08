import * as THREE from "three";

import type { Stance } from "./stance";

export type Hand = "left" | "right";

/**
 * The live XR objects the session's pieces share without React: the
 * origin group (`XrOrigin`), the head (the frame camera while presenting),
 * the launch stance a recenter returns to, each controller's ray space
 * object (`XrHands`), and the panel quad (for the automation report). Set
 * by the components that own them, read by `XrLocomotion`, `actions.ts`
 * and `automation.ts`; reset when the session ends.
 */
export const xrRig = {
  origin: null as THREE.Group | null,
  head: null as THREE.Camera | null,
  stance: null as Stance | null,
  rays: { left: null, right: null } as Record<Hand, THREE.Object3D | null>,
  /** Each controller's grip space object (`XrHands`) — what grabs the TCP handle. */
  grips: { left: null, right: null } as Record<Hand, THREE.Object3D | null>,
  panel: null as THREE.Mesh | null,
  /** The teleport preview of the moment (for the report). */
  teleport: { hand: null as Hand | null, hit: null as number[] | null, valid: false },
};

export function resetRig(): void {
  xrRig.head = null;
  xrRig.rays.left = null;
  xrRig.rays.right = null;
  xrRig.grips.left = null;
  xrRig.grips.right = null;
  xrRig.teleport = { hand: null, hit: null, valid: false };
}

export type HeadPose = { position: THREE.Vector3; quaternion: THREE.Quaternion };

/**
 * The head in the reference space — what the placements take. three's XR
 * camera is an `ArrayCamera` with one camera per eye; its own transform
 * is the left eye before a render and the frustum-union camera (behind
 * the eyes) after it, and neither is the head. The midpoint of the eyes
 * is, and the eye cameras keep their reference-space pose through the
 * frame. A mono camera is its own head.
 */
export function headPose(camera: THREE.Camera | null = xrRig.head): HeadPose | null {
  if (!camera) return null;
  const eyes = (camera as THREE.ArrayCamera).cameras;
  if (!eyes || eyes.length === 0) {
    return { position: camera.position.clone(), quaternion: camera.quaternion.clone() };
  }
  const position = new THREE.Vector3();
  for (const eye of eyes) position.add(eye.position);
  position.divideScalar(eyes.length);
  return { position, quaternion: eyes[0].quaternion.clone() };
}

/** The head in world coordinates (Z-up), or null outside a session. */
export function headWorld(): THREE.Vector3 | null {
  const pose = headPose();
  const origin = xrRig.origin;
  if (!pose || !origin) return null;
  origin.updateWorldMatrix(true, false);
  return origin.localToWorld(pose.position.clone());
}
