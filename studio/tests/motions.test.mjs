import assert from "node:assert/strict";
import { test } from "node:test";
import {
  UPRIGHT_CONE,
  conventionalName,
  freshMotionName,
  motionTarget,
  waypointSegment,
} from "../src/motions.ts";

const motion = (name, robot = null, group = null, segments = []) => ({ name, robot, group, segments });
const single = (name) => ({ name, dualArm: false, selectedGroup: null });

test("the conventional names: main for the first robot, main_<robot> for the rest, _<arm> on a dual arm", () => {
  assert.equal(conventionalName("panda", "panda", null), "main");
  assert.equal(conventionalName("ur", "panda", null), "main_ur");
  assert.equal(conventionalName("panda", "panda", "left"), "main_left");
  assert.equal(freshMotionName([], "panda", "panda", null), "main");
  assert.equal(freshMotionName([motion("main")], "panda", "panda", null), "motion_2");
  assert.equal(freshMotionName([motion("main"), motion("motion_2")], "panda", "panda", null), "motion_3");
});

test("a waypoint lands in the picked motion, else the robot's first, else the conventional fresh name", () => {
  const motions = [motion("approach", "ur"), motion("main", null, null, [{}])];
  assert.deepEqual(motionTarget(motions, "approach", single("panda"), "panda"), {
    motionName: "approach",
    arm: null,
    motion: motions[0],
  });
  // Nothing picked: the robot's own first motion (`robot: null` means the first robot).
  assert.equal(motionTarget(motions, null, single("panda"), "panda").motionName, "main");
  assert.equal(motionTarget(motions, null, single("ur"), "panda").motionName, "approach");
  // A robot with no motion yet: the conventional name, not created yet.
  const fresh = motionTarget(motions, null, single("kuka"), "panda");
  assert.equal(fresh.motionName, "main_kuka");
  assert.equal(fresh.motion, null);
});

test("on a dual-arm robot the arm picks the motion and travels with the waypoint", () => {
  const motions = [motion("main_left", "g1", "left"), motion("main_right", "g1", "right")];
  const right = { name: "g1", dualArm: true, selectedGroup: "right" };
  assert.deepEqual(motionTarget(motions, null, right, "g1"), {
    motionName: "main_right",
    arm: "right",
    motion: motions[1],
  });
  const left = { name: "g1", dualArm: true, selectedGroup: "left" };
  assert.equal(motionTarget([], null, left, "g1").motionName, "main_left");
});

test("a waypoint copies the configuration and carries the upright cone only when asked", () => {
  const q = [0.1, 0.2, 0.3];
  const plain = waypointSegment(q, "joint", false);
  assert.deepEqual(plain, { kind: "joint", goal_positions: [0.1, 0.2, 0.3], constraints: [] });
  assert.notEqual(plain.goal_positions, q, "a copy, not the live array");
  const upright = waypointSegment(q, "cartesian_line", true);
  assert.equal(upright.kind, "cartesian_line");
  assert.deepEqual(upright.constraints, [UPRIGHT_CONE]);
  assert.ok(Math.abs(UPRIGHT_CONE.angle - Math.PI / 6) < 1e-12, "30°");
});
