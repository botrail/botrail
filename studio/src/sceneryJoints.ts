import * as THREE from "three";

import { inGroup } from "./partIdentity.ts";
import type { PoseMsg, SceneryJointMsg } from "./protocol";

/**
 * Doors, drawers and racks (`botrail_scene::scenery`): the obstacles that
 * move together on a hinge or a slide. The host owns the joint — it sends
 * the list and applies `set_scenery_joint` — and this module is the
 * geometry both the sliders and the headset's hand need: where the axis
 * is in the world, which joint an obstacle moves on, the rigid motion a
 * change of position is, and what a hand's move means along or about the
 * axis. Pure: three, no React, no store (`tests/sceneryJoints.test.mjs`).
 */

export type JointAxis = { origin: THREE.Vector3; axis: THREE.Vector3 };

/** A hand this close to a hinge's axis says nothing about the angle (m). */
const HINGE_DEAD_RADIUS = 0.02;

/** The joint's axis in the world: its own numbers when it has no base,
 * else those carried by the base obstacle's pose. */
export function jointAxisWorld(joint: SceneryJointMsg, base: PoseMsg | null): JointAxis {
  const origin = new THREE.Vector3(...joint.origin);
  const axis = new THREE.Vector3(...joint.axis);
  if (joint.base !== null && base) {
    const turn = new THREE.Quaternion(...base.quaternion);
    origin.applyQuaternion(turn).add(new THREE.Vector3(...base.position));
    axis.applyQuaternion(turn);
  }
  return { origin, axis: axis.normalize() };
}

/** The joint an obstacle moves on: of several (a bin on a door moves on
 * the door's hinge and on its own slide) the innermost — the one moving
 * the fewest things. */
export function jointOf(joints: readonly SceneryJointMsg[], obstacle: string): SceneryJointMsg | null {
  let found: SceneryJointMsg | null = null;
  for (const joint of joints) {
    if (!joint.objects.includes(obstacle)) continue;
    if (!found || joint.objects.length < found.objects.length) found = joint;
  }
  return found;
}

/** The joints of a selection: the one a selected obstacle moves on, or
 * every joint of a selected group (those moving something in it). */
export function jointsOfSelection(
  joints: readonly SceneryJointMsg[],
  selection: { type: string; name?: string; path?: string },
): SceneryJointMsg[] {
  if (selection.type === "obstacle" && selection.name !== undefined) {
    const joint = jointOf(joints, selection.name);
    return joint ? [joint] : [];
  }
  if (selection.type === "group" && selection.path !== undefined) {
    const path = selection.path;
    return joints.filter((j) => j.objects.some((o) => inGroup(o, path)));
  }
  return [];
}

/** The rigid motion of the members when the position changes by `by`
 * (rad about the axis for a hinge, metres along it for a slide). */
export function jointMotion(kind: SceneryJointMsg["kind"], at: JointAxis, by: number): THREE.Matrix4 {
  if (kind === "slide") {
    return new THREE.Matrix4().makeTranslation(at.axis.x * by, at.axis.y * by, at.axis.z * by);
  }
  const turn = new THREE.Matrix4().makeRotationAxis(at.axis, by);
  const { x, y, z } = at.origin;
  return new THREE.Matrix4()
    .makeTranslation(x, y, z)
    .multiply(turn)
    .multiply(new THREE.Matrix4().makeTranslation(-x, -y, -z));
}

/** What a hand's move from `from` to `to` is on the joint: the angle it
 * swept about a hinge's axis (rad, signed by the axis; nothing while the
 * hand is on the axis), the distance it went along a slide's (m). */
export function handMove(
  kind: SceneryJointMsg["kind"],
  at: JointAxis,
  from: THREE.Vector3,
  to: THREE.Vector3,
): number {
  if (kind === "slide") return to.clone().sub(from).dot(at.axis);
  const arm = (p: THREE.Vector3) => {
    const r = p.clone().sub(at.origin);
    return r.addScaledVector(at.axis, -r.dot(at.axis));
  };
  const a = arm(from);
  const b = arm(to);
  if (a.length() < HINGE_DEAD_RADIUS || b.length() < HINGE_DEAD_RADIUS) return 0;
  const swept = Math.atan2(new THREE.Vector3().crossVectors(a, b).dot(at.axis), a.dot(b));
  return Number.isFinite(swept) ? swept : 0;
}

export function clampToRange(range: readonly [number, number], value: number): number {
  return Math.min(Math.max(value, range[0]), range[1]);
}

/** `fence/west/door` → `door`; an imported joint's `…/Joints/LeftDoor_joint` → `LeftDoor`. */
export function jointLabel(name: string): string {
  const last = name.split("/").filter(Boolean).pop() ?? name;
  return last.replace(/_joint$/i, "");
}

/** The position as people read it: degrees for a hinge, millimetres for a slide. */
export function jointValueText(kind: SceneryJointMsg["kind"], position: number): string {
  return kind === "hinge" ? `${Math.round((position * 180) / Math.PI)}°` : `${Math.round(position * 1000)} mm`;
}
