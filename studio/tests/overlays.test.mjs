import assert from "node:assert/strict";
import { test } from "node:test";
import { NO_OVERLAY, openOverlay, reconcileOverlays, setOverlay } from "../src/overlays.ts";

test("the analysis panel holds one overlay: opening one closes the rest", () => {
  const sfc = setOverlay(NO_OVERLAY, "sfc", true);
  assert.equal(openOverlay(sfc), "sfc");
  const bom = setOverlay(sfc, "bom", true);
  assert.deepEqual(bom, { ...NO_OVERLAY, bomOpen: true });
  assert.equal(openOverlay(bom), "bom");
  // Closing one leaves the others as they were (none, here).
  assert.deepEqual(setOverlay(bom, "bom", false), NO_OVERLAY);
  assert.equal(openOverlay(setOverlay(bom, "io", false)), "bom");
  assert.equal(openOverlay(NO_OVERLAY), null);
});

test("stored flags that say several were open resolve to one", () => {
  assert.equal(openOverlay(reconcileOverlays({ ...NO_OVERLAY, sfcOpen: true, ioOpen: true })), "io");
  assert.equal(openOverlay(reconcileOverlays({ ...NO_OVERLAY, sfcOpen: true, bomOpen: true })), "bom");
  assert.equal(openOverlay(reconcileOverlays(NO_OVERLAY)), null);
});
