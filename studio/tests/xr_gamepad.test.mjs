import assert from "node:assert/strict";
import { test } from "node:test";
import { AxisLatch, ButtonEdges, THUMBSTICK_AXES, XR_STANDARD, readPad } from "../src/xr/gamepad.ts";

test("readPad copies the xr-standard gamepad; a missing gamepad reads as nothing", () => {
  const pad = readPad({
    buttons: [{ pressed: true }, { pressed: false }, { pressed: false }, { pressed: false }, { pressed: true }],
    axes: [0, 0, 0.5, -1],
  });
  assert.deepEqual(pad.pressed, [true, false, false, false, true]);
  assert.equal(pad.pressed[XR_STANDARD.trigger], true);
  assert.equal(pad.pressed[XR_STANDARD.primary], true);
  assert.equal(pad.axes[THUMBSTICK_AXES.x], 0.5);
  assert.equal(pad.axes[THUMBSTICK_AXES.y], -1);
  assert.deepEqual(readPad(null), { pressed: [], axes: [] });
  assert.deepEqual(readPad(undefined), { pressed: [], axes: [] });
});

test("button edges fire once per press, not while held", () => {
  const edges = new ButtonEdges();
  assert.deepEqual(edges.update([false, false, false, false, false, false]), []);
  assert.deepEqual(edges.update([false, false, false, false, true, false]), [4]);
  assert.deepEqual(edges.update([false, false, false, false, true, false]), [], "held: no new edge");
  assert.deepEqual(edges.update([true, false, false, false, false, true]), [0, 5]);
  assert.deepEqual(edges.update([]), [], "a disconnected pad");
  assert.deepEqual(edges.update([true]), [0], "and back");
});

test("an axis latch engages past the threshold and re-arms inside the release band", () => {
  const latch = new AxisLatch(0.7, 0.3);
  assert.deepEqual(latch.feed(0.2), { engaged: 0, released: 0, held: 0 });
  assert.deepEqual(latch.feed(0.6), { engaged: 0, released: 0, held: 0 }, "below engage");
  assert.deepEqual(latch.feed(0.9), { engaged: 1, released: 0, held: 1 }, "engages once");
  assert.deepEqual(latch.feed(1.0), { engaged: 0, released: 0, held: 1 }, "held");
  assert.deepEqual(latch.feed(0.5), { engaged: 0, released: 0, held: 1 }, "between the bands: still held");
  assert.deepEqual(latch.feed(0.1), { engaged: 0, released: 1, held: 0 }, "released, with the direction");
  assert.deepEqual(latch.feed(-0.8), { engaged: -1, released: 0, held: -1 }, "the other way");
  assert.deepEqual(latch.feed(0), { engaged: 0, released: -1, held: 0 });
  assert.deepEqual(latch.feed(NaN), { engaged: 0, released: 0, held: 0 }, "a NaN axis is at rest");
});
