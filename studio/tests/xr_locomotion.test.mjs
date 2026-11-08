import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import { xrOriginQuaternion } from "three-usd-robot/xr";
import {
  SNAP_TURN,
  applyPlacement,
  snapTurnPlacement,
  stancePlacement,
  teleportArc,
  teleportPlacement,
  withinFloor,
  yawOf,
} from "../src/xr/locomotion.ts";

const close = (a, b, eps, what) => assert.ok(Math.abs(a - b) < eps, `${what}: ${a} vs ${b}`);

/** A head 1.6 m up in the reference space, looking −Z (the reference forward). */
const head = () => ({
  position: new THREE.Vector3(0.1, 1.6, -0.05),
  quaternion: new THREE.Quaternion(),
});

/** The world position of the head under an origin placement. */
function headWorld(origin, pose) {
  return pose.position.clone().applyQuaternion(origin.quaternion).add(origin.position);
}

test("the arc lands on the floor ahead of a forward throw and is valid", () => {
  const arc = teleportArc(new THREE.Vector3(1, 2, 1.2), new THREE.Vector3(0, 1, 0.3));
  assert.ok(arc.hit, "lands");
  assert.equal(arc.hit.z, 0);
  assert.ok(arc.hit.y > 2 + 2, `well ahead: ${arc.hit.y}`);
  close(arc.hit.x, 1, 1e-9, "no sideways drift");
  assert.equal(arc.points.length, 25, "steps + 1 points");
  assert.deepEqual(arc.points[0].toArray(), [1, 2, 1.2], "starts at the hand");
  const last = arc.points[arc.points.length - 1];
  close(last.z, 0, 1e-9, "ends on the floor");
  // Heights rise then fall; never below the floor along the way.
  assert.ok(arc.points.every((p) => p.z >= -1e-9));
  assert.ok(arc.points[3].z > arc.points[0].z, "goes up first");
  assert.equal(arc.valid, true);
});

test("a throw straight down lands at the feet; one beyond the floor or too far is refused", () => {
  const down = teleportArc(new THREE.Vector3(0, 0, 1.2), new THREE.Vector3(0, 0, -1));
  close(down.hit.x, 0, 1e-9, "x");
  close(down.hit.y, 0, 1e-9, "y");
  assert.equal(down.valid, true);
  const far = teleportArc(new THREE.Vector3(23, 0, 1.2), new THREE.Vector3(1, 0, 0.5));
  assert.ok(far.hit.x > 24, `past the floor's edge: ${far.hit.x}`);
  assert.equal(far.valid, false);
  const long = teleportArc(new THREE.Vector3(0, 0, 1.2), new THREE.Vector3(1, 0, 0.7), { speed: 20 });
  assert.ok(long.hit && long.valid === false, "a 20 m/s throw overshoots the hop range");
  const below = teleportArc(new THREE.Vector3(0, 0, -0.1), new THREE.Vector3(1, 0, 0));
  assert.equal(below.hit, null);
  assert.equal(below.valid, false);
  assert.equal(withinFloor({ x: 24, y: -24 }), true);
  assert.equal(withinFloor({ x: 24.01, y: 0 }), false);
});

test("yawOf reads back the heading a placement wrote", () => {
  for (const yaw of [0, 0.6, -2.5, 3.0]) {
    const p = teleportPlacement(new THREE.Vector3(1, 1, 0), yaw, head());
    close(yawOf(p.quaternion), yaw, 1e-9, `yaw ${yaw}`);
  }
  close(yawOf(xrOriginQuaternion("Z")), 0, 1e-12, "the bare Z-up base is heading 0");
});

test("a teleport puts the head over the landing and keeps the heading", () => {
  const pose = head();
  const origin = new THREE.Group();
  const before = teleportPlacement(new THREE.Vector3(0, 0, 0), 0.4, pose);
  applyPlacement(origin, before);
  const hit = new THREE.Vector3(3, -2, 0);
  applyPlacement(origin, teleportPlacement(hit, yawOf(origin.quaternion), pose));
  const h = headWorld(origin, pose);
  close(h.x, 3, 1e-9, "head x over the landing");
  close(h.y, -2, 1e-9, "head y over the landing");
  close(h.z, 1.6, 1e-9, "head height unchanged");
  close(yawOf(origin.quaternion), 0.4, 1e-9, "heading kept");
  close(origin.position.z, 0, 1e-12, "the origin stays on the floor");
});

test("a snap turn pivots about the head's floor point", () => {
  const pose = head();
  const origin = new THREE.Group();
  applyPlacement(origin, teleportPlacement(new THREE.Vector3(2, 1, 0), 0.2, pose));
  const hw = headWorld(origin, pose);
  applyPlacement(origin, snapTurnPlacement(hw, yawOf(origin.quaternion), SNAP_TURN, pose));
  const after = headWorld(origin, pose);
  close(after.x, hw.x, 1e-9, "head stays (x)");
  close(after.y, hw.y, 1e-9, "head stays (y)");
  close(after.z, hw.z, 1e-9, "head stays (z)");
  close(yawOf(origin.quaternion), 0.2 + SNAP_TURN, 1e-9, "turned by one step");
  // Forward (the head's −Z) rotated by 45° about world Z.
  const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(origin.quaternion);
  const angle = Math.atan2(fwd.y, fwd.x);
  const before = teleportPlacement(new THREE.Vector3(), 0.2, pose).quaternion;
  const fwd0 = new THREE.Vector3(0, 0, -1).applyQuaternion(before);
  const angle0 = Math.atan2(fwd0.y, fwd0.x);
  close(Math.abs(angle - angle0), SNAP_TURN, 1e-9, "forward swung by 45°");
});

test("the launch stance faces the target and a recenter returns to it", () => {
  const pose = head();
  const stance = { standAt: [1.5, -1.5, 0], lookAt: [0, 0, 0.3] };
  const origin = new THREE.Group();
  applyPlacement(origin, stancePlacement(stance, pose));
  const h = headWorld(origin, pose);
  close(h.x, 1.5, 1e-9, "head over standAt x");
  close(h.y, -1.5, 1e-9, "head over standAt y");
  const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(pose.quaternion).applyQuaternion(origin.quaternion);
  const toTarget = new THREE.Vector3(-1.5, 1.5, 0).normalize();
  close(fwd.x, toTarget.x, 1e-9, "faces the target (x)");
  close(fwd.y, toTarget.y, 1e-9, "faces the target (y)");
  // Wander off, then recenter.
  applyPlacement(origin, teleportPlacement(new THREE.Vector3(5, 5, 0), 2, pose));
  applyPlacement(origin, stancePlacement(stance, pose));
  const back = headWorld(origin, pose);
  close(back.x, 1.5, 1e-9, "back (x)");
  close(back.y, -1.5, 1e-9, "back (y)");
});
