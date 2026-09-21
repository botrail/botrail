import assert from "node:assert/strict";
import { test } from "node:test";
import { initialView } from "../src/three/initialView.ts";

test("launch view accepts world coordinates alongside other URL options", () => {
  assert.deepEqual(initialView("?wasm&view=5.9,4.7,4,1.25,0,0.65"), {
    position: [5.9, 4.7, 4], target: [1.25, 0, 0.65],
  });
});

test("missing, malformed and degenerate views keep the existing camera", () => {
  const fallback = { position: [1.6, -1.6, 1.2], target: [0, 0, 0.2] };
  for (const query of ["", "?view=", "?view=1,2,3", "?view=1,2,3,4,5,6,7",
    "?view=1,2,3,4,,6", "?view=1,2,3,4,NaN,6", "?view=1,2,3,4,5,Infinity",
    "?view=1,2,3,1,2,3"]) {
    assert.deepEqual(initialView(query), fallback, query);
  }
});
