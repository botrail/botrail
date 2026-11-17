import assert from "node:assert/strict";
import test from "node:test";

import { lineOfStep, machineFunctions, mm, ncStatusAt, parseRunName, toolLabel } from "../src/ncPanel.ts";

// A cell program: a cut, then the vise, the window and a relay that calls the cell.
const TEXT = [
  "%",
  "O00010 (CELL)",
  "N10 G90 G21",
  "N25 T1 M6",
  "N30 S6000 M3",
  "N65 X27. F900.",
  "N125 M5",
  "N220 G53 G0 X-762. Y-160.5",
  "N300 M71 (VISE OPEN)",
  "N305 M180 (WINDOW OPEN - NOT M21)",
  "N310 M21",
  "M99",
].join("\n");
const PROGRAMS = [{ name: "op10", text: TEXT }, { name: "warm#up", text: "G0 X0." }];
const MACHINE = { robot: "vf2/axes" };
const span = (name, start, end, sequence = "vf2") => ({ name, start, end, sequence, step: 0 });

const TIMELINE = {
  robots: [
    { name: "arm", moves: [span("approach", 0, 1, "tend")] },
    { name: "vf2/axes", moves: [span("op10#0", 0, 10), span("op10#0", 20, 30)] },
  ],
  words: [
    { name: "vf2/axes.line", times: [0, 2, 5, 8, 10, 20], values: [4, 5, 6, 8, 0, 4], kind: "device", integer: true },
    { name: "vf2/axes.tool", times: [0, 1], values: [0, 1], kind: "device", integer: true },
    { name: "vf2/axes.rpm", times: [0, 2, 7], values: [0, 6000, 0], kind: "device", integer: true },
  ],
  signals: [
    { name: "vf2/axes.spindle", times: [0, 2, 7], values: [false, true, false], kind: "device" },
    { name: "vf2/axes.coolant", times: [0, 3, 7], values: [false, true, false], kind: "device" },
  ],
  stepSpans: [
    span("tend/approach", 0, 1, "tend"),
    span("vf2/N25-N220", 0, 10),
    span("vf2/N300 M71 vise unclamp", 10, 10.8),
    span("vf2/N305 M180 door open?", 10.8, 10.8),
    span("vf2/N305 M180 door open", 10.8, 12),
    span("tend/wait_call", 1, 12, "tend"),
    span("vf2/N310 M21 arm: part out, blank in", 12, 20),
    span("vf2/N25-N220", 20, 30),
  ],
};

test("a run is named after its program and its block", () => {
  const names = PROGRAMS.map((p) => p.name);
  assert.deepEqual(parseRunName("op10", names), { program: "op10", block: null });
  assert.deepEqual(parseRunName("op10#3", names), { program: "op10", block: 3 });
  // A program whose own name holds a `#` is itself first.
  assert.deepEqual(parseRunName("warm#up", names), { program: "warm#up", block: null });
  assert.deepEqual(parseRunName("warm#up#0", names), { program: "warm#up", block: 0 });
  assert.equal(parseRunName("approach", names), null);
  assert.equal(parseRunName("op10#x", names), null);
});

test("a step named after a program line finds that line", () => {
  assert.equal(lineOfStep("N305 M180 door open", TEXT), 10);
  assert.equal(lineOfStep("N305 M180 door open ALARM — the cell is not safe", TEXT), 10);
  assert.equal(lineOfStep("N25-N220", TEXT), 4);
  // A line with no N word is named by its number.
  assert.equal(lineOfStep("L12 M99", TEXT), 12);
  assert.equal(lineOfStep("L40 M99", TEXT), null);
  assert.equal(lineOfStep("N999 M70", TEXT), null);
  assert.equal(lineOfStep("cycle start", TEXT), null);
});

test("the control runs a line, or stands at a machine function while the cell answers", () => {
  const at = (t) => ncStatusAt(TIMELINE, MACHINE, PROGRAMS, t);
  // Cutting: the line off the lane, the spindle and the coolant as they were switched.
  const cutting = at(5.5);
  assert.deepEqual(
    [cutting.program, cutting.block, cutting.running, cutting.line, cutting.standing],
    ["op10", 0, true, 6, null],
  );
  assert.deepEqual([cutting.tool, cutting.rpm, cutting.spindle, cutting.coolant], [1, 6000, true, true]);
  assert.deepEqual([at(0.5).tool, at(0.5).spindle, at(0.5).line], [0, false, 4]);
  // The block is over: the control runs no line and stands at the M code
  // the machine's program is answering — the step named after the line.
  const vise = at(10.3);
  assert.deepEqual([vise.running, vise.line], [false, null]);
  assert.deepEqual(vise.standing, { step: "N300 M71 vise unclamp", line: 9, alarm: false });
  assert.deepEqual(at(11).standing, { step: "N305 M180 door open", line: 10, alarm: false });
  assert.deepEqual(at(15).standing, { step: "N310 M21 arm: part out, blank in", line: 11, alarm: false });
  assert.deepEqual([at(15).tool, at(15).spindle], [1, false]);
  // The program has started over.
  assert.deepEqual([at(21).running, at(21).line, at(21).standing], [true, 4, null]);
});

test("an alarm is where the program stands for good", () => {
  const stopped = {
    ...TIMELINE,
    robots: [{ name: "vf2/axes", moves: [span("op10#0", 0, 10)] }],
    words: TIMELINE.words.map((w) =>
      w.name === "vf2/axes.line" ? { ...w, times: [0, 10], values: [4, 0] } : w),
    stepSpans: [
      span("vf2/N25-N220", 0, 10),
      span("vf2/N300 M71 vise unclamp", 10, 10.8),
      span("vf2/N305 M180 door open?", 10.8, 10.8),
      span("vf2/N305 M180 door open ALARM — the cell is not safe", 10.8, 60),
    ],
  };
  const status = ncStatusAt(stopped, MACHINE, PROGRAMS, 40);
  assert.deepEqual(status.standing, {
    step: "N305 M180 door open ALARM — the cell is not safe",
    line: 10,
    alarm: true,
  });
  // A machine nothing has run on, and one with no lanes at all.
  const idle = ncStatusAt({ ...TIMELINE, robots: [], words: [], signals: [] }, MACHINE, PROGRAMS, 5);
  assert.deepEqual(
    [idle.program, idle.running, idle.line, idle.standing, idle.tool],
    [null, false, null, null, 0],
  );
});

test("the listing marks the machine's M codes, outside comments", () => {
  const table = [{ code: 180, what: "door open" }, { code: 21, what: "arm: part out, blank in" }];
  const calls = TEXT.split("\n").map((line) => machineFunctions(line, table));
  assert.deepEqual(calls[9], [[180, "door open"]]);
  assert.deepEqual(calls[10], [[21, "arm: part out, blank in"]]);
  // M6, M3 and M99 are the control's own; M71 is not in this table.
  assert.deepEqual([calls[3], calls[4], calls[8], calls[11]], [[], [], [], []]);
});

test("lengths read in millimetres and tools by their cutter", () => {
  assert.equal(mm(-0.762), "-762.000");
  assert.equal(mm(-1e-9), "0.000");
  assert.equal(toolLabel({ cutter: "flat", diameter: 0.01 }), "⌀10 flat end mill");
  assert.equal(toolLabel({ cutter: "drill", diameter: 0.005 }), "⌀5 drill");
  assert.equal(toolLabel({ cutter: null, diameter: 0.0125 }), "⌀12.5");
  assert.equal(toolLabel({}), "—");
});
