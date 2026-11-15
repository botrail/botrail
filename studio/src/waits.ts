import type {
  ConditionMsg,
  IoPointMsg,
  SequenceMsg,
  SignalTrackMsg,
  StepMsg,
  StepSpanMsg,
  WordTrackMsg,
} from "./protocol";
import { addressOf, contactPoint, leafKey, pointKey } from "./ioLinks.ts";
import { exitConditions, flattenSteps, standingAt, truth, type Truth } from "./sfc.ts";

export { standingAt };
import { conditionLabel } from "./seqLabels.ts";

/**
 * What the programs wait on — the dock's line under the step bar, and the
 * cycle's waits summed per exit. The client twin of
 * `crates/botrail-scene/src/rollout/waits.rs` (`tl.waiting(t)`,
 * `tl.waits()`), on the same evaluation the SFC chart lights its atoms by
 * (`sfc.truth`): levels and timers exact, an edge lit for its afterglow.
 * A program stands in a step from its span's start to its end, both
 * included — on a live plant the end is the head of the open span.
 */

type Lanes = {
  stepSpans: StepSpanMsg[];
  signals: SignalTrackMsg[];
  robots: { name: string; moves: StepSpanMsg[] }[];
  words?: WordTrackMsg[];
};

/** One open contact: its text in PLC vocabulary, the address of the
 * point it reads, what it lights up with. */
export type WaitItem = {
  text: string;
  address: string | null;
  key: string | null;
  /** A timer's served fraction, 0..1. */
  progress?: number;
  /** An edge contact's signal level now. */
  level?: boolean;
};

export type WaitLine = {
  program: string;
  step: string;
  flat: number;
  since: number;
  /** Per exit (one, or one per arm of a branching step), its open
   * contacts — any one exit closing lets the program on. */
  exits: WaitItem[][];
};

/** The leaves of a truth tree that keep the condition open: an `all`'s
 * open children, every alternative of an `any` that does not hold. */
export function openLeaves(tr: Truth): Truth[] {
  if (tr.children.length === 0) return tr.holds ? [] : [tr];
  if (tr.holds) return [];
  return tr.children.flatMap(openLeaves);
}

/** A leaf's text at its step: `done` names the moves it waits for, a
 * timer counts what it has served. */
export function leafText(tr: Truth, step: StepMsg): string {
  const cond = tr.cond;
  if (cond.type === "elapsed") {
    const served = (tr.progress ?? 0) * cond.seconds;
    return `${served.toFixed(2)}/${cond.seconds.toFixed(2)}s`;
  }
  if (cond.type === "done") {
    const moves = step.actions.flatMap((a) =>
      a.type === "start_motion"
        ? [`▶${a.motion}`]
        : a.type === "start_toolpath"
          ? [`⟿${a.toolpath}`]
          : a.type === "start_ramp"
            ? ["ramp"]
            : a.type === "policy"
              ? [`policy ${a.policy}`]
              : [],
    );
    if (moves.length > 0) return `done ${moves.join(" ")}`;
  }
  return conditionLabel(cond);
}

/** What every program of `sequences` waits on at `t`. */
export function waitingLines(
  sequences: SequenceMsg[],
  timeline: Lanes,
  t: number,
  points: IoPointMsg[],
): WaitLine[] {
  const lines: WaitLine[] = [];
  for (const seq of sequences) {
    const span = standingAt(timeline.stepSpans, seq.name, t);
    if (!span) continue;
    const node = flattenSteps(seq.steps)[span.step];
    if (!node) continue;
    const ctx = {
      span,
      signals: timeline.signals,
      robots: timeline.robots,
      stepSpans: timeline.stepSpans,
      words: timeline.words,
    };
    const exits = exitConditions(node).map((cond) =>
      openLeaves(truth(cond, t, ctx)).map((leaf): WaitItem => {
        const p = contactPoint(points, seq.name, span.step, leaf.cond, seq.steps);
        return {
          text: leafText(leaf, node.step),
          address: p ? addressOf(p) : null,
          key: p ? pointKey(p) : leafKey(leaf.cond),
          progress: leaf.cond.type === "elapsed" ? leaf.progress : undefined,
          level: leaf.level,
        };
      }),
    );
    lines.push({ program: seq.name, step: node.name, flat: span.step, since: span.start, exits });
  }
  return lines;
}

/** One exit's share of the run: how long programs stood in steps this
 * exit let go, and how many times. */
export type WaitShare = { program: string; condition: string; seconds: number; count: number };

/** Where the run's time went, per program and per releasing exit — the
 * first exit (authored order) conducting at each stay's end, the
 * rollout's own first-wins rule. Longest first. */
export function waitBreakdown(sequences: SequenceMsg[], timeline: Lanes): WaitShare[] {
  const flats = new Map(sequences.map((s) => [s.name, flattenSteps(s.steps)] as const));
  const shares = new Map<string, WaitShare>();
  for (const span of timeline.stepSpans) {
    const node = flats.get(span.sequence)?.[span.step];
    if (!node) continue;
    const ctx = {
      span,
      signals: timeline.signals,
      robots: timeline.robots,
      stepSpans: timeline.stepSpans,
      words: timeline.words,
    };
    const exit = exitConditions(node).find((c) => truth(c, span.end, ctx).holds);
    if (!exit) continue;
    const condition = released(exit, node.step);
    const key = `${span.sequence}\u0000${condition}`;
    const share = shares.get(key) ?? { program: span.sequence, condition, seconds: 0, count: 0 };
    share.seconds += span.end - span.start;
    share.count += 1;
    shares.set(key, share);
  }
  return [...shares.values()].sort((a, b) => b.seconds - a.seconds);
}

function released(cond: ConditionMsg, step: StepMsg): string {
  if (cond.type === "done") return leafText({ cond, holds: true, children: [] }, step);
  return conditionLabel(cond);
}
