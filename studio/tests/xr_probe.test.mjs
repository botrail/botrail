import assert from "node:assert/strict";
import { test } from "node:test";
import { probeXr } from "../src/xr/probe.ts";

const xr = (answer) => ({ isSessionSupported: () => answer });

test("a headset runtime that answers yes enables XR", async () => {
  assert.equal(await probeXr(xr(Promise.resolve(true))), true);
});

test("no runtime, a no, a rejection or a throw all mean no XR", async () => {
  assert.equal(await probeXr(undefined), false);
  assert.equal(await probeXr({}), false);
  assert.equal(await probeXr(xr(Promise.resolve(false))), false);
  assert.equal(await probeXr(xr(Promise.reject(new Error("no device")))), false);
  assert.equal(
    await probeXr({ isSessionSupported: () => { throw new Error("boom"); } }),
    false,
  );
});

test("a runtime that never answers is given up on within the timeout", async () => {
  const started = Date.now();
  assert.equal(await probeXr(xr(new Promise(() => {})), 20), false);
  assert.ok(Date.now() - started < 1000);
});
