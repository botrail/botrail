import assert from "node:assert/strict";
import { test } from "node:test";
import * as THREE from "three";

import {
  clampToRange,
  handMove,
  jointAxisWorld,
  jointLabel,
  jointMotion,
  jointOf,
  jointValueText,
  jointsOfSelection,
} from "../src/sceneryJoints.ts";

const close = (a, b, eps = 1e-9, what = "") => assert.ok(Math.abs(a - b) <= eps, `${what} ${a} vs ${b}`);
const closeAll = (a, b, eps = 1e-9, what = "") => a.forEach((v, i) => close(v, b[i], eps, `${what}[${i}]`));
const at = (x, y, z) => new THREE.Vector3(x, y, z);

const joint = (over) => ({
  name: "fridge/World/Joints/LeftDoor_joint",
  kind: "hinge",
  base: null,
  axis: [0, 0, 1],
  origin: [1, 2, 0],
  range: [0, 2],
  position: 0,
  objects: ["fridge/door", "fridge/door/handle"],
  ...over,
});

test("a joint's axis is where its base carries it", () => {
  const free = jointAxisWorld(joint({}), null);
  closeAll(free.origin.toArray(), [1, 2, 0]);
  closeAll(free.axis.toArray(), [0, 0, 1]);
  // On a base turned a quarter about Z and standing at (5, 0, 1): the
  // hinge's own (1, 2, 0) along +X of the base is the world's +Y.
  const quarter = [0, 0, Math.sin(Math.PI / 4), Math.cos(Math.PI / 4)];
  const based = jointAxisWorld(joint({ base: "fridge/body", axis: [1, 0, 0] }), {
    position: [5, 0, 1],
    quaternion: quarter,
  });
  closeAll(based.origin.toArray(), [3, 1, 1], 1e-9, "origin");
  closeAll(based.axis.toArray(), [0, 1, 0], 1e-9, "axis");
  // A base named but not (yet) in the scene: the numbers as they are.
  closeAll(jointAxisWorld(joint({ base: "ghost" }), null).origin.toArray(), [1, 2, 0]);
});

test("an obstacle moves on the innermost joint that moves it", () => {
  const door = joint({ name: "door", objects: ["leaf", "handle", "bin"] });
  const bin = joint({ name: "bin", kind: "slide", objects: ["bin"] });
  assert.equal(jointOf([door, bin], "leaf").name, "door");
  assert.equal(jointOf([door, bin], "bin").name, "bin", "the bin's own slide, not the door it rides");
  assert.equal(jointOf([bin, door], "bin").name, "bin", "whatever the order");
  assert.equal(jointOf([door, bin], "body"), null);
});

test("a selection's joints: the obstacle's own, or all of a group's", () => {
  const left = joint({ name: "left", objects: ["fridge/World/Links/LeftDoor/mesh"] });
  const rack = joint({ name: "rack", kind: "slide", objects: ["dw/World/Links/Rack/mesh"] });
  const all = [left, rack];
  assert.deepEqual(jointsOfSelection(all, { type: "obstacle", name: "dw/World/Links/Rack/mesh" }), [rack]);
  assert.deepEqual(jointsOfSelection(all, { type: "group", path: "/fridge" }), [left]);
  assert.deepEqual(jointsOfSelection(all, { type: "group", path: "/fridge/World/Links/LeftDoor" }), [left]);
  assert.deepEqual(jointsOfSelection(all, { type: "obstacle", name: "fridge/World/Links/Base/mesh" }), []);
  assert.deepEqual(jointsOfSelection(all, { type: "tcp", robot: "panda" }), []);
});

test("the motion of a change of position: a turn about the axis, a run along it", () => {
  const hinge = { origin: at(1, 2, 0), axis: at(0, 0, 1) };
  const quarter = jointMotion("hinge", hinge, Math.PI / 2);
  // A point a metre along +X of the hinge goes to a metre along +Y of it.
  closeAll(at(2, 2, 0.5).applyMatrix4(quarter).toArray(), [1, 3, 0.5]);
  closeAll(at(1, 2, 7).applyMatrix4(quarter).toArray(), [1, 2, 7], 1e-9, "on the axis");
  const slide = { origin: at(0, 0, 0), axis: at(0, -1, 0) };
  closeAll(at(1, 1, 1).applyMatrix4(jointMotion("slide", slide, 0.4)).toArray(), [1, 0.6, 1]);
});

test("a hand's move on a hinge is the angle it sweeps about the axis", () => {
  const hinge = { origin: at(1, 2, 0), axis: at(0, 0, 1) };
  close(handMove("hinge", hinge, at(2, 2, 1), at(1, 3, 1.4)), Math.PI / 2, 1e-12);
  close(handMove("hinge", hinge, at(1, 3, 1), at(2, 2, 1)), -Math.PI / 2, 1e-12, "the other way");
  // Moving along the axis, or straight out from it, turns nothing.
  close(handMove("hinge", hinge, at(2, 2, 0), at(2, 2, 1)), 0);
  close(handMove("hinge", hinge, at(2, 2, 0), at(3, 2, 0)), 0);
  // On the axis the hand says nothing — no jump as it crosses.
  assert.equal(handMove("hinge", hinge, at(1.01, 2, 0), at(2, 2, 0)), 0);
  assert.equal(handMove("hinge", hinge, at(NaN, 2, 0), at(2, 2, 0)), 0);
  // Step by step all the way round adds up past a half turn.
  let swept = 0;
  let from = at(2, 2, 0);
  for (let i = 1; i <= 30; i++) {
    const a = (i / 30) * 1.5 * Math.PI;
    const to = at(1 + Math.cos(a), 2 + Math.sin(a), 0);
    swept += handMove("hinge", hinge, from, to);
    from = to;
  }
  close(swept, 1.5 * Math.PI, 1e-9, "three quarters of a turn");
});

test("a hand's move on a slide is how far it went along the axis", () => {
  const slide = { origin: at(0, 0, 0), axis: at(1, 0, 0) };
  close(handMove("slide", slide, at(0, 0, 0), at(0.3, 0.2, -0.1)), 0.3, 1e-12);
  close(handMove("slide", slide, at(0.3, 0, 0), at(0.1, 5, 0)), -0.2, 1e-12);
});

test("ranges, names and readings", () => {
  assert.equal(clampToRange([0, 1.5], 2), 1.5);
  assert.equal(clampToRange([0, 1.5], -0.1), 0);
  assert.equal(clampToRange([0, 1.5], 0.7), 0.7);
  assert.equal(jointLabel("fridge/World/Joints/LeftDoor_joint"), "LeftDoor");
  assert.equal(jointLabel("fence/west/door"), "door");
  assert.equal(jointValueText("hinge", Math.PI / 2), "90°");
  assert.equal(jointValueText("slide", 0.4293), "429 mm");
});
