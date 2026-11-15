import assert from "node:assert/strict";
import { test } from "node:test";

import { contactPoint, leafKey } from "../src/ioLinks.ts";
import { anchorOf, tagLine } from "../src/ioTags.ts";
import { buildLadder } from "../src/ladder.ts";
import { conditionLabel, actionLabel } from "../src/seqLabels.ts";
import { truth } from "../src/sfc.ts";
import { analogReading, analogText, forcedWord, wordHands, wordLive } from "../src/words.ts";

const word = (name, samples, integer = false) => ({
  name,
  times: samples.map((s) => s[0]),
  values: samples.map((s) => s[1]),
  kind: "signal",
  integer,
});

const dp = { name: "dp", source: "dp_true", range: [0, 100], unit: "kPa", raw: [0, 4000], place: "dpt/body" };
const words = [
  word("dp_true", [[0, 10], [1, 54.6]]),
  word("dp", [[0, 10], [1.01, 54.5], [3, -25]]),
  word("dp.raw", [[0, 400], [1.01, 2180], [3, -1000]], true),
  word("dp_warn", [[0, 60], [2, 40]]),
  word("lot", [[0, 3]], true),
];
const NO_STEP = { name: "", start: 0, end: Infinity, sequence: "", step: -1 };
const ctx = { span: NO_STEP, signals: [], robots: [], stepSpans: [], words };

const point = (over) => ({
  name: "dp",
  aspect: null,
  label: "dp",
  direction: "input",
  kind: "ai",
  source: "analog",
  host: "PLC1",
  status: "bound",
  node: "PLC1",
  channel: "AI0",
  address: "%IW100",
  tag: null,
  writers: [],
  readers: [{ sequence: "filter", index: 3, name: "warn" }],
  safety: false,
  ...over,
});

test("a comparison reads word lanes at the playhead", () => {
  const cond = { type: "compare", left: "dp", op: ">=", right: "dp_warn" };
  assert.equal(conditionLabel(cond), "dp>=dp_warn");
  assert.equal(truth(cond, 1.5, ctx).holds, false); // 54.5 >= 60
  assert.equal(truth(cond, 2.5, ctx).holds, true); // 54.5 >= 40
  assert.equal(truth({ type: "compare", left: "dp.raw", op: "<", right: -100 }, 3.5, ctx).holds, true);
  assert.equal(truth({ type: "compare", left: "lot", op: "==", right: 3 }, 0, ctx).holds, true);
  // Without word lanes a word reads 0.
  assert.equal(truth(cond, 2.5, { ...ctx, words: undefined }).holds, true);
  assert.equal(actionLabel({ type: "set_word", word: "dp_warn", value: { left: 40 } }), "dp_warn←40");
  assert.equal(
    actionLabel({ type: "set_word", word: "n", value: { left: "n", op: "+", right: 1 } }),
    "n←n + 1",
  );
});

test("an analog input's loop: current from the counts, open under the band", () => {
  const r = analogReading(dp, words, 1.5);
  assert.deepEqual(
    { raw: r.raw, scaled: r.scaled, measured: r.measured, open: r.open },
    { raw: 2180, scaled: 54.5, measured: 54.6, open: false },
  );
  assert.ok(Math.abs(r.current - 12.72) < 1e-9);
  assert.equal(analogText(r), "54.5 kPa · 12.72 mA · 2180");
  const broken = analogReading(dp, words, 3.5);
  assert.equal(broken.open, true);
  assert.equal(broken.current, 0);
  assert.match(analogText(broken), /open \(0 mA\) · -1000$/);
  assert.equal(analogReading(dp, [], 1), null);
});

test("the table's live column and the hands a word takes", () => {
  assert.equal(wordLive(point({}), words, [dp], 1.5), "54.5 kPa · 12.72 mA · 2180");
  const setpoint = point({ name: "dp_warn", label: "dp_warn", kind: "word", source: "signal:internal" });
  assert.equal(wordLive(setpoint, words, [dp], 2.5), "40");
  assert.equal(wordLive(point({ name: "beam", label: "beam", kind: "di" }), words, [dp], 1), null);
  const defs = [{ name: "dp_warn", initial: 60, integer: false }];
  assert.deepEqual(wordHands(setpoint, defs, [dp], words), { write: true, force: true });
  assert.deepEqual(wordHands(point({}), defs, [dp], words), { write: false, force: true });
  assert.equal(forcedWord({ forced_words: [["dp", 90]] }, "dp"), 90);
  assert.equal(forcedWord({ forced: [] }, "dp"), null);
});

test("a comparison's contact is its word's point, its tag hangs on the transmitter", () => {
  const points = [point({})];
  const cond = { type: "compare", left: "dp", op: ">=", right: "dp_warn" };
  assert.equal(contactPoint(points, "filter", 3, cond)?.address, "%IW100");
  assert.equal(contactPoint(points, "filter", 3, { ...cond, left: "dp.raw", right: -100 })?.label, "dp");
  assert.equal(leafKey({ ...cond, left: "dp.raw" }), "dp");
  const scene = { sensors: [], devices: [], lamps: [], robots: [], analogs: [dp] };
  assert.deepEqual(anchorOf(points[0], scene), { kind: "obstacle", name: "dpt/body" });
  const line = tagLine(points[0], { signals: [], words }, 1.5, null, [dp]);
  assert.equal(line.state, "value");
  assert.equal(line.value, "54.5 kPa 12.72mA");
  assert.equal(tagLine(points[0], { signals: [], words }, 3.5, null, [dp]).value, "-25.0 kPa open");
});

test("the ladder draws a comparison as a box contact and a transfer as a box output", () => {
  const sequence = { name: "filter", steps: [] };
  const logic = {
    rungs: [
      {
        comment: "warn",
        condition: { type: "compare", left: "dp", op: ">=", right: "dp_warn" },
        outputs: [{ type: "coil", target: "warn" }],
      },
      {
        comment: "peak",
        condition: { type: "compare", left: "dp", op: ">", right: "dp_peak" },
        outputs: [{ type: "move", word: "dp_peak", value: { left: "dp" } }],
      },
    ],
    cycle: false,
  };
  const ladder = buildLadder(sequence.steps, { logic });
  const rungs = ladder.groups.flatMap((g) => g.rungs);
  const [warn, peak] = rungs;
  assert.equal(warn.contacts[0].kind, "cmp");
  assert.equal(warn.contacts[0].label, "dp >= dp_warn");
  assert.equal(peak.coils[0].op, "mov");
  assert.deepEqual([peak.coils[0].label, peak.coils[0].note, peak.coils[0].target], ["dp_peak", "dp", "dp_peak"]);
  assert.equal(peak.condition.type, "compare");
});
