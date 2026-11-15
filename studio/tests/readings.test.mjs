import assert from "node:assert/strict";
import { test } from "node:test";

import { lineText, tagLine } from "../src/ioTags.ts";
import {
  deviationAt,
  deviationChip,
  deviationsOf,
  readLevel,
  readNumber,
  readSpans,
  readingsCaption,
  readingsOf,
} from "../src/readings.ts";
import { wordPolyline } from "../src/timelineLanes.ts";

const readings = {
  host: "PLC1",
  source: "log line_io.csv",
  tolerance: 0.1,
  signals: [{ name: "s2", times: [0, 4.86, 6.02], values: [false, true, false], kind: "read" }],
  words: [{ name: "dp.raw", times: [0, 2], values: [400, 2180], kind: "read", integer: true }],
  deviations: [
    { lane: "s2", start: 4.36, end: 4.86, kind: "late", note: "`s2` rose 0.50 s later on the machine" },
    { lane: "s2", start: 5.52, end: 6.02, kind: "late", note: "`s2` fell 0.50 s later on the machine" },
  ],
};

test("the readings on show are the live monitor's, else the timeline's", () => {
  const live = { ...readings, source: "modbus read 10.0.0.5:502 · 200 ms" };
  assert.equal(readingsOf({ readings: live }, { readings }), live);
  assert.equal(readingsOf(null, { readings }), readings);
  assert.equal(readingsOf({}, { readings }), readings);
  assert.equal(readingsOf(null, {}), null);
  assert.equal(readingsOf(null, null), null);
});

test("a read lane has a level, a word a number, and neither before the first reading", () => {
  assert.equal(readLevel(readings, "s2", 4.5), false);
  assert.equal(readLevel(readings, "s2", 5.0), true);
  assert.equal(readLevel(readings, "s1", 5.0), null);
  assert.equal(readNumber(readings, "dp.raw", 3), 2180);
  assert.equal(readNumber(readings, "dp.raw", -1), null);
  assert.equal(readLevel(null, "s2", 1), null);
});

test("deviations are found by lane and by time", () => {
  assert.equal(deviationsOf(readings, "s2").length, 2);
  assert.equal(deviationsOf(readings, "s1").length, 0);
  assert.equal(deviationAt(readings, "s2", 4.6)?.kind, "late");
  assert.equal(deviationAt(readings, "s2", 5.0), null);
  assert.equal(deviationChip(readings.deviations[0]), "s2 late 0.50 s");
  assert.match(readingsCaption(readings), /PLC1 read from log line_io\.csv · ±0\.10 s · 2 deviations/);
  assert.match(readingsCaption({ ...readings, deviations: [] }), /agrees with the model/);
});

test("the on stretches of a read lane are clipped to the run", () => {
  assert.deepEqual(readSpans(readings.signals[0], 10), [{ start: 4.86, end: 6.02 }]);
  assert.deepEqual(readSpans({ times: [0, 8], values: [false, true] }, 10), [{ start: 8, end: 10 }]);
  assert.deepEqual(readSpans({ times: [0, 12], values: [false, true] }, 10), []);
});

const point = {
  name: "s2", aspect: null, label: "s2", direction: "input", kind: "DI", source: "sensor", status: "bound",
  node: "PLC1", channel: "DI6", address: "%IX100.6", host: "PLC1", writers: [], readers: [],
};
const lanes = { signals: [{ name: "s2", times: [0, 4.36, 5.52], values: [false, true, false] }], words: [] };

test("a tag shows what the controller read, and marks where the model says otherwise", () => {
  // The model has the work at the sensor; the machine's sensor has not answered yet.
  const apart = tagLine(point, lanes, 4.6, null, [], readings);
  assert.equal(apart.state, "off");
  assert.equal(apart.deviates, true);
  assert.match(lineText(apart), /%IX100\.6 s2 ≠/);
  // Both on: the reading, no mark.
  const agreed = tagLine(point, lanes, 5.0, null, [], readings);
  assert.equal(agreed.state, "on");
  assert.equal(agreed.deviates, undefined);
  // Without readings the lane speaks.
  assert.equal(tagLine(point, lanes, 4.6, null, []).state, "on");
});

test("two traces share a lane's scale", () => {
  const own = wordPolyline({ times: [0], values: [5] }, 10, 100, 8);
  const shared = wordPolyline({ times: [0], values: [5] }, 10, 100, 8, 0, 10);
  assert.equal(own, "0,4 100,4");
  assert.equal(shared, "0,4 100,4");
  assert.equal(wordPolyline({ times: [0], values: [10] }, 10, 100, 8, 0, 10), "0,1 100,1");
  assert.equal(wordPolyline({ times: [], values: [] }, 10, 100, 8, 0, 10), "");
});
