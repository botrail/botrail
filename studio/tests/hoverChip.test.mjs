import assert from "node:assert/strict";
import { test } from "node:test";
import {
  HOVER_DELAY_MS,
  chipPlacement,
  chipVisible,
  hoverLabel,
  hoverMatchesSelection,
} from "../src/hoverChip.ts";

test("the chip stays off what is already selected", () => {
  const crate = { kind: "obstacle", name: "/World/crate_3" };
  assert.equal(hoverMatchesSelection(crate, { type: "obstacle", name: "/World/crate_3" }), true);
  assert.equal(hoverMatchesSelection(crate, { type: "obstacle", name: "/World/crate_4" }), false);
  // A prim of the selected group is the selection, whichever way the slash goes.
  assert.equal(hoverMatchesSelection({ kind: "obstacle", name: "fence/east/panels/w1000/3" }, { type: "group", path: "/fence/east" }), true);
  assert.equal(hoverMatchesSelection({ kind: "obstacle", name: "fence/west/p0" }, { type: "group", path: "/fence/east" }), false);
  assert.equal(hoverMatchesSelection({ kind: "robot", name: "arm" }, { type: "tcp", robot: "arm" }), true);
  assert.equal(hoverMatchesSelection({ kind: "robot", name: "arm" }, { type: "robot", robot: "arm" }), true);
  assert.equal(hoverMatchesSelection({ kind: "robot", name: "arm" }, { type: "tcp", robot: "panda" }), false);
  assert.equal(hoverMatchesSelection({ kind: "camera", name: "cam" }, { type: "camera", name: "cam" }), true);
  assert.equal(hoverMatchesSelection({ kind: "lidar", name: "l" }, { type: "line", kind: "tool", name: "arm/tool" }), false);
});

test("the chip shows after the pointer rests, with no button down, off the selection", () => {
  const hover = { kind: "obstacle", name: "crate" };
  const selection = { type: "tcp", robot: "arm" };
  assert.equal(chipVisible({ hover, selection, restedMs: HOVER_DELAY_MS, buttons: 0 }), true);
  assert.equal(chipVisible({ hover, selection, restedMs: HOVER_DELAY_MS - 1, buttons: 0 }), false);
  assert.equal(chipVisible({ hover, selection, restedMs: 1000, buttons: 1 }), false);
  assert.equal(chipVisible({ hover: null, selection, restedMs: 1000, buttons: 0 }), false);
  assert.equal(chipVisible({ hover, selection: { type: "obstacle", name: "crate" }, restedMs: 1000, buttons: 0 }), false);
});

test("the chip sits right-below the pointer and flips at the edges", () => {
  const chip = { w: 200, h: 24 };
  const view = { w: 1000, h: 600 };
  assert.deepEqual(chipPlacement({ x: 100, y: 100 }, chip, view), { left: 114, top: 118 });
  // Near the right edge it sits to the left of the pointer.
  assert.deepEqual(chipPlacement({ x: 950, y: 100 }, chip, view), { left: 950 - 14 - 200, top: 118 });
  // Near the bottom it sits above.
  assert.deepEqual(chipPlacement({ x: 100, y: 590 }, chip, view), { left: 114, top: 590 - 18 - 24 });
  // Never off the top-left.
  assert.deepEqual(chipPlacement({ x: 5, y: 595 }, { w: 400, h: 24 }, { w: 300, h: 600 }), { left: 0, top: 595 - 18 - 24 });
});

test("the label: kind, name and — when the bill has a line — the product", () => {
  assert.equal(hoverLabel({ kind: "sensor", name: "gate_curtain" }, "キーエンス GL-R22L"),
    "sensor · gate_curtain · キーエンス GL-R22L");
  assert.equal(hoverLabel({ kind: "obstacle", name: "/World/Floor" }, null), "obstacle · /World/Floor");
});
