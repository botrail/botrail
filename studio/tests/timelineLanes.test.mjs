import assert from "node:assert/strict";
import { test } from "node:test";
import {
  AUTO_FOLD_SENSORS,
  DEFAULT_LANE_VIEW,
  MIN_LANES_PX,
  clampLanesHeight,
  isFolded,
  parseLaneView,
  serializeLaneView,
  signalGroup,
  toggleGroup, formatWord, wordPolyline, wordValueAt } from "../src/timelineLanes.ts";

test("signal lanes group by the kind the bake gave them", () => {
  assert.equal(signalGroup("signal"), "signals");
  assert.equal(signalGroup("sensor"), "sensors");
  assert.equal(signalGroup("device"), "devices");
  assert.equal(signalGroup("anything else"), "signals");
});

test("by default device outputs fold, sensors fold only when many, the rest show", () => {
  const view = DEFAULT_LANE_VIEW;
  assert.equal(isFolded(view, "devices", 1), true);
  assert.equal(isFolded(view, "sensors", AUTO_FOLD_SENSORS), false);
  assert.equal(isFolded(view, "sensors", AUTO_FOLD_SENSORS + 1), true);
  assert.equal(isFolded(view, "robots", 40), false);
  assert.equal(isFolded(view, "signals", 40), false);
});

test("a viewer's choice outranks the default, either way", () => {
  let view = toggleGroup(DEFAULT_LANE_VIEW, "sensors", 60);     // auto-folded -> open
  assert.equal(isFolded(view, "sensors", 60), false);
  view = toggleGroup(view, "robots", 17);                       // shown -> folded
  assert.equal(isFolded(view, "robots", 17), true);
  view = toggleGroup(view, "devices", 3);
  assert.equal(isFolded(view, "devices", 3), false);
  assert.deepEqual(DEFAULT_LANE_VIEW.folded, {}, "the default is never mutated");
});

test("a dragged height stays between two lanes and most of the viewport", () => {
  assert.equal(clampLanesHeight(5, 900), MIN_LANES_PX);
  assert.equal(clampLanesHeight(300.4, 900), 300);
  assert.equal(clampLanesHeight(5000, 900), 720);
  assert.equal(clampLanesHeight(100, 10), MIN_LANES_PX, "a tiny viewport still leaves the minimum");
});

test("the view survives storage, and a damaged entry falls back to the default", () => {
  const view = { open: false, height: 240, folded: { sensors: false, robots: true } };
  assert.deepEqual(parseLaneView(serializeLaneView(view)), view);
  for (const bad of [null, "", "{", "[]", '{"height": -3, "open": "no"}']) {
    const parsed = parseLaneView(bad);
    assert.equal(parsed.height, null, String(bad));
    assert.equal(parsed.open, true, String(bad));
  }
  assert.deepEqual(parseLaneView('{"folded": {"sensors": "yes", "bogus": true, "devices": false}}').folded,
    { devices: false });
});

test("a word lane reads the last sample at or before the playhead and labels counts plain", () => {
  const encoder = { times: [0, 0.5, 1.0], values: [0, 0.05, 0.1] };
  assert.equal(wordValueAt(encoder, 0.0), 0);
  assert.equal(wordValueAt(encoder, 0.7), 0.05);
  assert.equal(wordValueAt(encoder, 1.0), 0.1);
  assert.equal(wordValueAt(encoder, 5.0), 0.1);
  assert.equal(formatWord(0.05, false), "0.050");
  assert.equal(formatWord(123.456, false), "123.5");
  assert.equal(formatWord(7, true), "7");
  // The step plot: two points per sample, scaled to the lane's range.
  const points = wordPolyline(encoder, 2.0, 1000, 8);
  assert.equal(points, "0,7 250,7 250,4 500,4 500,1 1000,1");
  // A flat lane sits mid-track; an empty one draws nothing.
  assert.equal(wordPolyline({ times: [0], values: [3] }, 1.0, 1000, 8), "0,4 1000,4");
  assert.equal(wordPolyline({ times: [], values: [] }, 1.0), "");
});
