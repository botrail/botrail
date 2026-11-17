// What a machine tool's control shows at one instant of a baked cycle —
// the logic of the NC panel, free of the DOM so it runs under node.
//
// The bake records what a control switches as lanes named after the
// machine's robot: the signals `<robot>.spindle` and `<robot>.coolant`,
// and the words `<robot>.rpm`, `<robot>.tool` and `<robot>.line` — the
// source line running, 0 while the control runs none. The run in progress
// is a move on the robot's lane, named after its program (`op10`) or, for
// a program that calls machine functions, the block of it (`op10#3`).
//
// While the control runs no line it stands at a machine function — an M
// code the machine answers — and what answers it is a step of the
// machine's own program. `bt.nc.sequence` names those steps after the
// program's lines (`N305 M180 door open`, or `L61 …` where a line has no
// N word), which is how the panel finds the line to point at.

import type {
  NcMachineMsg,
  NcProgramMsg,
  SignalTrackMsg,
  StepSpanMsg,
  WordTrackMsg,
} from "./protocol";

/** The lanes and bands of a baked cycle the panel reads. */
export interface NcTimeline {
  stepSpans: StepSpanMsg[];
  robots: { name: string; moves: StepSpanMsg[] }[];
  signals: SignalTrackMsg[];
  words: WordTrackMsg[];
}

/** The control at one instant. */
export interface NcStatus {
  /** The program that is running, or that ran last; `null` before any. */
  program: string | null;
  /** The block of it, for a program run by block. */
  block: number | null;
  /** Whether a run is in progress at this instant. */
  running: boolean;
  /** The source line being run (1-based); `null` when the control runs none. */
  line: number | null;
  /** While it runs none: the step of the machine's program it stands in,
   * the program line that step is named after (when it is), and whether
   * the step is an alarm. */
  standing: { step: string; line: number | null; alarm: boolean } | null;
  tool: number;
  rpm: number;
  spindle: boolean;
  coolant: boolean;
}

/** A run's name taken apart: the program, and the block if it names one.
 * A program whose own name holds a `#` is itself first. */
export function parseRunName(
  name: string,
  programs: readonly string[],
): { program: string; block: number | null } | null {
  if (programs.includes(name)) return { program: name, block: null };
  const cut = name.lastIndexOf("#");
  if (cut < 0) return null;
  const [program, block] = [name.slice(0, cut), name.slice(cut + 1)];
  if (!/^\d+$/.test(block) || !programs.includes(program)) return null;
  return { program, block: Number(block) };
}

/** A lane's value at `t`: the last sample at or before it. */
function valueAt<T>(times: readonly number[], values: readonly T[], t: number, initial: T): T {
  let value = initial;
  for (let i = 0; i < times.length && times[i] <= t + 1e-9; i++) value = values[i];
  return value;
}

/** The 1-based line of `text` a step's tag names: `N305` is the first line
 * whose N word is 305, `L61` is line 61. `null` when the name starts with
 * neither, or no line has that N word. */
export function lineOfStep(step: string, text: string): number | null {
  const tag = /^([NL])(\d+)\b/.exec(step);
  if (!tag) return null;
  const number = Number(tag[2]);
  const lines = text.split("\n");
  if (tag[1] === "L") return number >= 1 && number <= lines.length ? number : null;
  const found = lines.findIndex((line) => {
    const word = /^\s*\/?\s*N\s*(\d+)/i.exec(line);
    return word !== null && Number(word[1]) === number;
  });
  return found < 0 ? null : found + 1;
}

/** What the control of `machine` shows at `t` of `timeline`. */
export function ncStatusAt(
  timeline: NcTimeline,
  machine: Pick<NcMachineMsg, "robot">,
  programs: readonly NcProgramMsg[],
  t: number,
): NcStatus {
  const names = programs.map((p) => p.name);
  const robot = machine.robot;
  const word = (suffix: string) => {
    const lane = timeline.words.find((w) => w.name === `${robot}.${suffix}`);
    return lane ? valueAt(lane.times, lane.values, t, 0) : 0;
  };
  const signal = (suffix: string) => {
    const lane = timeline.signals.find((s) => s.name === `${robot}.${suffix}`);
    return lane ? valueAt(lane.times, lane.values, t, false) : false;
  };
  // The run in progress, or the last one that started.
  const moves = timeline.robots.find((r) => r.name === robot)?.moves ?? [];
  let last: { span: StepSpanMsg; program: string; block: number | null } | null = null;
  for (const span of moves) {
    if (span.start > t + 1e-9) break;
    const run = parseRunName(span.name, names);
    if (run) last = { span, ...run };
  }
  const running = last !== null && t < last.span.end - 1e-9;
  const line = word("line");
  let standing: NcStatus["standing"] = null;
  if (last !== null && line === 0) {
    // The step the machine's program stands in: the last of its bands to
    // have started. In a bake of several programs a band's name carries
    // its program's first (`vf2/N305 …`).
    const sequence = last.span.sequence;
    let band: StepSpanMsg | null = null;
    for (const span of timeline.stepSpans) {
      if (span.sequence === sequence && span.start <= t + 1e-9) band = span;
    }
    if (band) {
      const prefix = `${sequence}/`;
      const step = band.name.startsWith(prefix) ? band.name.slice(prefix.length) : band.name;
      const text = programs.find((p) => p.name === last!.program)?.text ?? "";
      standing = { step, line: lineOfStep(step, text), alarm: /\bALARM\b/.test(step) };
    }
  }
  return {
    program: last?.program ?? null,
    block: last?.block ?? null,
    running,
    line: line > 0 ? line : null,
    standing,
    tool: word("tool"),
    rpm: word("rpm"),
    spindle: signal("spindle"),
    coolant: signal("coolant"),
  };
}

/** The M codes of the machine's table a program line calls, outside its
 * comments: `[code, what]` each. */
export function machineFunctions(
  line: string,
  table: readonly { code: number; what: string }[],
): [number, string][] {
  const bare = line.replace(/\([^)]*\)/g, " ").replace(/;.*$/, "");
  const out: [number, string][] = [];
  for (const match of bare.matchAll(/M\s*(\d+)/gi)) {
    const row = table.find((m) => m.code === Number(match[1]));
    if (row) out.push([row.code, row.what]);
  }
  return out;
}

/** A length in metres as the control writes it: millimetres, three places. */
export function mm(value: number): string {
  const text = (value * 1e3).toFixed(3);
  return text === "-0.000" ? "0.000" : text;
}

/** How a pocket's tool reads in the table. */
export function toolLabel(tool: {
  cutter?: string | null;
  diameter?: number | null;
}): string {
  const kind: Record<string, string> = {
    flat: "flat end mill",
    ball: "ball end mill",
    bull_nose: "bull-nose end mill",
    drill: "drill",
  };
  const size = tool.diameter != null ? `⌀${+(tool.diameter * 1e3).toFixed(3)} ` : "";
  return tool.cutter ? `${size}${kind[tool.cutter] ?? tool.cutter}` : size ? `${size.trim()}` : "—";
}
