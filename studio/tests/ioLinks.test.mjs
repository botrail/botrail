import assert from "node:assert/strict";
import { test } from "node:test";

import { actionPoints, addressOf, contactPoint, leafKey, nameCoil, nameContact } from "../src/ioLinks.ts";
import { anchorOf, lineText, tagGroups, tagLine, taggedPoints } from "../src/ioTags.ts";
import { openLeaves, standingAt, waitBreakdown, waitingLines } from "../src/waits.ts";

// A belt feeds a box to a beam; the program then dwells. The beam is X20
// on PLC1, the belt's run coil Y0; `fed` is an internal relay.
const feed = {
  name: "feed",
  steps: [
    {
      name: "feed",
      actions: [{ type: "device", device: "conv", command: { type: "start" } }],
      transition: { type: "rising", name: "beam" },
    },
    {
      name: "dwell",
      actions: [
        { type: "device", device: "conv", command: { type: "stop" } },
        { type: "set", signal: "fed", value: true },
      ],
      transition: {
        type: "any",
        conditions: [
          { type: "elapsed", seconds: 0.5 },
          { type: "signal", name: "skip", value: true },
        ],
      },
    },
  ],
};

const point = (over) => ({
  aspect: null,
  kind: "DI",
  host: "PLC1",
  safety: false,
  writers: [],
  readers: [],
  status: "unbound",
  node: null,
  channel: null,
  address: null,
  tag: null,
  ...over,
  label: over.aspect ? `${over.name}.${over.aspect}` : over.name,
});

const points = [
  point({
    name: "beam",
    direction: "input",
    source: "sensor",
    readers: [{ sequence: "feed", index: 0, name: "feed" }],
    status: "bound",
    node: "PLC1",
    channel: "DI0",
    address: "X20",
  }),
  point({
    name: "conv",
    direction: "output",
    kind: "DO",
    source: "device:run",
    writers: [
      { sequence: "feed", index: 0, name: "feed" },
      { sequence: "feed", index: 1, name: "dwell" },
    ],
    status: "bound",
    node: "PLC1",
    channel: "DO0",
    address: "Y0",
  }),
  point({ name: "fed", direction: "output", kind: "DO", source: "signal:internal", status: "internal" }),
  point({
    name: "skip",
    direction: "input",
    source: "signal:read-only",
    readers: [{ sequence: "feed", index: 1, name: "dwell" }],
  }),
];

const timeline = {
  stepSpans: [
    { name: "feed", start: 0, end: 2, sequence: "feed", step: 0 },
    { name: "dwell", start: 2, end: 2.5, sequence: "feed", step: 1 },
  ],
  signals: [
    { name: "beam", times: [0, 2], values: [false, true], kind: "sensor" },
    { name: "conv", times: [0, 0, 2], values: [false, true, false], kind: "device" },
    { name: "skip", times: [0], values: [false], kind: "signal" },
  ],
  words: [],
  robots: [],
};

test("a contact resolves to the point its step reads, with its address", () => {
  const beam = contactPoint(points, "feed", 0, { type: "rising", name: "beam" });
  assert.equal(beam?.name, "beam");
  assert.equal(addressOf(beam), "X20");
  // The same signal read at a step that does not read it: no point.
  assert.equal(contactPoint(points, "feed", 1, { type: "rising", name: "beam" }), null);
  assert.equal(leafKey({ type: "robot_done", robot: "arm" }), "arm.done");
  assert.deepEqual(nameContact(points, "feed", 0, { type: "rising", name: "beam" }, "beam"), {
    label: "X20",
    note: "beam",
    key: "beam",
    address: "X20",
  });
  // An unbound read keeps its name and lights by it.
  assert.deepEqual(nameContact(points, "feed", 1, { type: "signal", name: "skip", value: true }, "skip"), {
    label: "skip",
    note: undefined,
    key: "skip",
    address: null,
  });
});

test("a coil names the points its action writes", () => {
  const start = feed.steps[0].actions[0];
  assert.deepEqual(
    actionPoints(points, "feed", 0, start).map((p) => p.name),
    ["conv"],
  );
  assert.deepEqual(nameCoil(points, "feed", 0, start, "⚙ conv start"), {
    label: "Y0",
    note: "⚙ conv start",
    key: "conv",
    address: "Y0",
  });
  // A relay coil is no I/O: the name, keyed by the signal.
  const set = feed.steps[1].actions[1];
  assert.deepEqual(nameCoil(points, "feed", 1, set, "fed"), { label: "fed", key: "fed", address: null });
});

test("a program stands in a step from its start to its end, both included", () => {
  assert.equal(standingAt(timeline.stepSpans, "feed", 1)?.name, "feed");
  // At the hand-over instant it stands in the next step.
  assert.equal(standingAt(timeline.stepSpans, "feed", 2)?.name, "dwell");
  assert.equal(standingAt(timeline.stepSpans, "feed", 2.5)?.name, "dwell");
  assert.equal(standingAt(timeline.stepSpans, "feed", 3), null);
  assert.equal(standingAt(timeline.stepSpans, "other", 1), null);
});

test("the waiting line names the open contacts and their addresses", () => {
  const [line] = waitingLines([feed], timeline, 1, points);
  assert.equal(line.program, "feed");
  assert.equal(line.step, "feed");
  assert.deepEqual(line.exits, [
    [{ text: "↑beam", address: "X20", key: "beam", progress: undefined, level: false }],
  ]);
  // In the dwell both alternatives of the `any` are open: the timer (with
  // what it has served) and the unbound relay.
  const [dwell] = waitingLines([feed], timeline, 2.2, points);
  assert.equal(dwell.step, "dwell");
  const texts = dwell.exits[0].map((i) => i.text);
  assert.deepEqual(texts, ["0.20/0.50s", "skip=1"]);
  assert.ok(Math.abs(dwell.exits[0][0].progress - 0.4) < 1e-9);
  assert.equal(dwell.exits[0][1].address, null);
  // Past the end nothing waits.
  assert.deepEqual(waitingLines([feed], timeline, 3, points), []);
});

test("an any that holds leaves nothing open", () => {
  const tr = {
    cond: { type: "any", conditions: [] },
    holds: true,
    children: [{ cond: { type: "elapsed", seconds: 1 }, holds: false, children: [] }],
  };
  assert.deepEqual(openLeaves(tr), []);
});

test("the breakdown sums each program's stays per releasing exit", () => {
  const shares = waitBreakdown([feed], timeline);
  assert.deepEqual(
    shares.map((s) => [s.condition, Number(s.seconds.toFixed(3)), s.count]),
    [
      ["↑beam", 2, 1],
      ["0.50s | skip=1", 0.5, 1],
    ],
  );
});

test("tags sit on the residents of the bound points and read their levels", () => {
  const scene = {
    sensors: ["beam"],
    devices: [{ name: "conv", kind: { kind: "conveyor" } }],
    lamps: [{ signal: "fault", obstacle: "lamp_red" }],
    robots: ["arm"],
  };
  assert.deepEqual(anchorOf(points[0], scene), { kind: "sensor", name: "beam" });
  assert.deepEqual(anchorOf(points[1], scene), { kind: "device", name: "conv" });
  assert.equal(anchorOf(points[3], scene), null);
  assert.deepEqual(
    anchorOf(point({ name: "fault", direction: "output", source: "signal:write-only" }), scene),
    { kind: "obstacle", name: "lamp_red" },
  );
  assert.deepEqual(
    anchorOf(point({ name: "feed/extended", direction: "input", source: "sensor" }), {
      ...scene,
      devices: [{ name: "feed", kind: { kind: "linear_axis" } }],
    }),
    { kind: "device", name: "feed" },
  );
  // Once anything is bound, only bound points get a tag.
  assert.deepEqual(taggedPoints(points).map((p) => p.name), ["beam", "conv"]);
  const groups = tagGroups(points, scene, []);
  assert.deepEqual(
    groups.map((g) => [g.anchor.name, g.points.map((p) => p.name)]),
    [
      ["beam", ["beam"]],
      ["conv", ["conv"]],
    ],
  );
  const lit = tagLine(points[0], timeline, 2.1, null);
  assert.equal(lit.state, "on");
  assert.equal(lineText(lit), "X20 beam");
  const forced = tagLine(points[1], timeline, 1, { forced: [["conv", false]] });
  assert.equal(forced.state, "on");
  assert.equal(forced.forced, true);
  assert.equal(tagLine(points[1], null, 1, null).state, "none");
});

test("a word tag shows its value", () => {
  const enc = point({
    name: "feed",
    aspect: "encoder",
    direction: "input",
    kind: "Word",
    source: "device:feedback",
    status: "bound",
    node: "PLC1",
    channel: "IW0",
    address: "D0",
  });
  const lanes = { signals: [], words: [{ name: "feed.encoder", times: [0, 1], values: [0, 0.12], integer: false }] };
  const line = tagLine(enc, lanes, 1.5, null);
  assert.equal(line.state, "value");
  assert.equal(lineText(line), "D0 feed.encoder 0.120");
});
