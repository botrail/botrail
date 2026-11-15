import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";
import { xrOriginQuaternion } from "three-usd-robot/xr";
import {
  FLOOR_HALF,
  GLIDE,
  SNAP_TURN,
  applyPlacement,
  floorHeading,
  glideStep,
  snapTurnPlacement,
  stancePlacement,
  stickDemand,
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

// ------------------------------------------------------------------ glide

/** Runs a glide for `seconds` at `hz`, returning the velocity and the distance covered. */
function glide(velocity, demand, heading, seconds, hz, from) {
  let v = velocity;
  const moved = { x: 0, y: 0 };
  const at = from ? { ...from } : undefined;
  const frames = Math.round(seconds * hz);
  for (let i = 0; i < frames; i++) {
    const out = glideStep(v, demand, heading, 1 / hz, at);
    v = out.velocity;
    moved.x += out.step.x;
    moved.y += out.step.y;
    if (at) {
      at.x += out.step.x;
      at.y += out.step.y;
    }
  }
  return { velocity: v, moved, at };
}

test("the stick asks for nothing at rest, creeps on a light push and runs at the rim", () => {
  assert.deepEqual(stickDemand(0, 0), { x: 0, y: 0 });
  assert.deepEqual(stickDemand(0.1, -0.1), { x: 0, y: 0 }, "inside the dead zone");
  assert.deepEqual(stickDemand(Number.NaN, 0.5), { x: 0, y: 0 }, "a broken axis asks for nothing");
  const full = stickDemand(0, 1);
  close(full.x, 0, 1e-12, "full forward x");
  close(full.y, 1, 1e-12, "full forward y");
  // Half of the live travel is a quarter of the speed.
  const half = stickDemand(0, GLIDE.deadzone + (1 - GLIDE.deadzone) / 2);
  close(half.y, 0.25, 1e-12, "squared response");
  // A square gate's corner is still length 1, in the stick's direction.
  const corner = stickDemand(-0.9, 0.9);
  close(Math.hypot(corner.x, corner.y), 1, 1e-12, "capped at 1");
  close(corner.x, -corner.y, 1e-12, "direction kept");
  // Just past the dead zone the demand starts from 0, not from a step.
  assert.ok(Math.hypot(...Object.values(stickDemand(0, GLIDE.deadzone + 0.01))) < 1e-3);
});

test("the heading is where the head faces on the floor, whatever its pitch", () => {
  const pose = head();
  for (const yaw of [0, 0.7, -2.2]) {
    const origin = teleportPlacement(new THREE.Vector3(1, 2, 0), yaw, pose).quaternion;
    const gaze = new THREE.Vector3(0, 0, -1).applyQuaternion(origin);
    const level = floorHeading(origin.clone());
    close(level.x, gaze.x, 1e-9, `level head, yaw ${yaw} (x)`);
    close(level.y, gaze.y, 1e-9, `level head, yaw ${yaw} (y)`);
    close(Math.hypot(level.x, level.y), 1, 1e-12, "unit");
    // Looking down at a fixture, straight down, up at a gantry, and past
    // the zenith: the gaze shrinks or flips, the heading does not.
    for (const pitch of [-1.0, -Math.PI / 2, 1.3, 1.8]) {
      const tilted = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(1, 0, 0), pitch);
      const h = floorHeading(origin.clone().multiply(tilted));
      close(h.x, level.x, 1e-9, `pitch ${pitch} (x)`);
      close(h.y, level.y, 1e-9, `pitch ${pitch} (y)`);
    }
    // An ear on the shoulder leaves the gaze to decide.
    const rolled = new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0, 0, 1), Math.PI / 2);
    const r = floorHeading(origin.clone().multiply(rolled));
    close(r.x, level.x, 1e-9, "rolled (x)");
    close(r.y, level.y, 1e-9, "rolled (y)");
  }
  // Only a pose that is not one has no floor direction.
  assert.equal(floorHeading(new THREE.Quaternion(Number.NaN, 0, 0, 1)), null);
});

test("a glide eases up to the stick's speed along the heading, the same at any frame rate", () => {
  const heading = { x: Math.cos(0.5), y: Math.sin(0.5) };
  const forward = { x: 0, y: 1 };
  const a = glide({ x: 0, y: 0 }, forward, heading, 1, 72);
  const b = glide({ x: 0, y: 0 }, forward, heading, 1, 36);
  close(Math.hypot(a.velocity.x, a.velocity.y), GLIDE.speed, 1e-3, "full speed after a second");
  close(a.velocity.x / GLIDE.speed, heading.x, 1e-3, "along the heading (x)");
  close(a.velocity.y / GLIDE.speed, heading.y, 1e-3, "along the heading (y)");
  // v(t) = s (1 − e^(−t/τ)) integrates to s (T − τ (1 − e^(−T/τ))).
  const expected = GLIDE.speed * (1 - GLIDE.ease * (1 - Math.exp(-1 / GLIDE.ease)));
  close(Math.hypot(a.moved.x, a.moved.y), expected, 1e-9, "distance at 72 Hz");
  close(Math.hypot(b.moved.x, b.moved.y), expected, 1e-9, "distance at 36 Hz");
  // The first frame is not a step to full speed.
  const first = glideStep({ x: 0, y: 0 }, forward, heading, 1 / 72);
  const v1 = Math.hypot(first.velocity.x, first.velocity.y);
  assert.ok(v1 > 0 && v1 < 0.2 * GLIDE.speed, `eased start: ${v1}`);
  // Half the demand is half the speed.
  const slow = glide({ x: 0, y: 0 }, { x: 0, y: 0.5 }, heading, 1, 72);
  close(Math.hypot(slow.velocity.x, slow.velocity.y), GLIDE.speed / 2, 1e-3, "analog");
});

test("the stick's right is the heading's right, back is back, and letting go coasts to a stop", () => {
  const north = { x: 0, y: 1 };
  const right = glide({ x: 0, y: 0 }, { x: 1, y: 0 }, north, 1, 72);
  close(right.velocity.x, GLIDE.speed, 1e-3, "strafe right is +x when facing +y");
  close(right.velocity.y, 0, 1e-9, "no forward drift");
  const back = glide({ x: 0, y: 0 }, { x: 0, y: -1 }, north, 1, 72);
  close(back.velocity.y, -GLIDE.speed, 1e-3, "backwards");
  const stop = glide(right.velocity, { x: 0, y: 0 }, north, 1, 72);
  assert.deepEqual(stop.velocity, { x: 0, y: 0 }, "at rest, exactly");
  // Coasting covers v τ: 0.2 m from full speed.
  close(stop.moved.x, GLIDE.speed * GLIDE.ease, 2e-3, "stopping distance");
  // A head without a floor direction asks for nothing either.
  const lost = glide(right.velocity, { x: 0, y: 1 }, null, 1, 72);
  assert.deepEqual(lost.velocity, { x: 0, y: 0 });
});

test("a glide stops at the floor's edge and a hitch is not a jump", () => {
  const east = { x: 1, y: 0 };
  const run = glide({ x: 0, y: 0 }, { x: 0.3, y: 1 }, east, 3, 72, { x: FLOOR_HALF - 1, y: 0 });
  close(run.at.x, FLOOR_HALF, 1e-9, "held at the edge");
  assert.equal(run.velocity.x, 0, "no speed into the edge");
  assert.ok(run.at.y < -0.5, `still slides along it: ${run.at.y}`);
  // Already beyond the edge (a cell bigger than the floor): no farther
  // out, but free to walk back in.
  const out = glideStep({ x: 2, y: 0 }, { x: 0, y: 1 }, east, 1 / 72, { x: FLOOR_HALF + 3, y: 0 });
  assert.equal(out.step.x, 0);
  const back = glideStep({ x: -2, y: 0 }, { x: 0, y: -1 }, east, 1 / 72, { x: FLOOR_HALF + 3, y: 0 });
  assert.ok(back.step.x < 0);
  // A 5 s frame is integrated as the longest step, a broken one as none.
  const hitch = glideStep({ x: 2, y: 0 }, { x: 0, y: 1 }, east, 5);
  close(hitch.step.x, GLIDE.speed * GLIDE.maxStep, 1e-9, "capped");
  assert.deepEqual(glideStep({ x: 2, y: 0 }, { x: 0, y: 1 }, east, Number.NaN).step, { x: 0, y: 0 });
});
