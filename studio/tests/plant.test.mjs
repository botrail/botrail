import assert from "node:assert/strict";
import { test } from "node:test";

import {
  breakMark,
  forcedLevel,
  hostChip,
  isButton,
  nextSpeed,
  plantCaption,
  speedLabel,
  toggleBreak,
} from "../src/plant.ts";

const status = {
  label: "pick (plant of PLC1)",
  t: 12.5,
  dt: 0.01,
  pace: "realtime",
  speed: 1,
  paused: false,
  halted: null,
  hosts: [{ name: "PLC1", link: "modbus 127.0.0.1:5020", exchanges: 2037, error: null }],
  lag_ms: 1.25,
  lag_max_ms: 4,
  overruns: 0,
  forced: [["part_at_pick", false]],
  breaks: [{ lane: "vacuum", rising: true }],
  recording_from: 10,
  following: true,
  buttons: ["panel/cycle_start"],
  clips: [],
};

test("the caption names the plant, its pace and its recording", () => {
  assert.equal(
    plantCaption(status),
    "pick (plant of PLC1) · 1× realtime · lag 1.3 ms · 10 ms scan · ● rec 2.5 s",
  );
  assert.equal(plantCaption({ ...status, pace: "sync", recording_from: null }), "pick (plant of PLC1) · sync · 10 ms scan");
  assert.equal(hostChip(status.hosts[0]), "PLC1 · modbus 127.0.0.1:5020 · 2037×");
  assert.equal(hostChip({ name: "UR", link: "loopback", exchanges: 0, error: null }), "UR · loopback");
});

test("the speed button cycles the listed speeds and wraps", () => {
  assert.equal(speedLabel(0.25), "0.25×");
  assert.equal(speedLabel(1), "1×");
  assert.equal(nextSpeed(1), 2);
  assert.equal(nextSpeed(2), 0.1);
  assert.equal(nextSpeed(0.3), 0.5);
});

test("forces, breakpoints and buttons read off the status", () => {
  assert.equal(forcedLevel(status, "part_at_pick"), false);
  assert.equal(forcedLevel(status, "vacuum"), null);
  assert.equal(forcedLevel(null, "vacuum"), null);
  assert.equal(breakMark(status, "vacuum"), "↑");
  assert.equal(breakMark({ ...status, breaks: [...status.breaks, { lane: "vacuum", rising: false }] }, "vacuum"), "↕");
  assert.equal(breakMark(status, "conv"), null);
  assert.deepEqual(toggleBreak(status, "vacuum", true), { command: "break", lane: "vacuum", rising: true, on: false });
  assert.deepEqual(toggleBreak(status, "vacuum", false), { command: "break", lane: "vacuum", rising: false, on: true });
  assert.equal(isButton(status, "panel/cycle_start"), true);
  assert.equal(isButton(null, "panel/cycle_start"), false);
});
