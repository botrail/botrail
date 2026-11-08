import assert from "node:assert/strict";
import { test } from "node:test";
import { initialView } from "../src/three/initialView.ts";
import { stanceFromView } from "../src/xr/stance.ts";

test("the default orbit view stands the player 1.5 m from the target, on the eye's side", () => {
  const { standAt, lookAt } = stanceFromView(initialView(""));
  // eye (1.6, -1.6) → target (0, 0): the horizontal direction is (1, -1)/√2.
  assert.deepEqual(standAt, [1.06066, -1.06066, 0]);
  assert.deepEqual(lookAt, [0, 0, 0.2]);
});

test("a launch view decides the VR start too", () => {
  const { standAt, lookAt } = stanceFromView(
    initialView("?view=5.9,4.7,4,1.25,0,0.65"),
  );
  assert.equal(standAt[2], 0, "feet on the floor");
  const d = Math.hypot(standAt[0] - 1.25, standAt[1] - 0);
  assert.ok(Math.abs(d - 1.5) < 1e-6, `1.5 m from the target, got ${d}`);
  // Between the target and the eye.
  assert.ok(standAt[0] > 1.25 && standAt[1] > 0);
  assert.deepEqual(lookAt, [1.25, 0, 0.65]);
});

test("looking straight down falls back to the -Y side", () => {
  const { standAt } = stanceFromView({ position: [2, 3, 5], target: [2, 3, 0] });
  assert.deepEqual(standAt, [2, 1.5, 0]);
});
