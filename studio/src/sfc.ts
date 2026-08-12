import type {
  BranchTakenMsg,
  ConditionMsg,
  SelectArmMsg,
  SequenceMsg,
  StepMsg,
  StepSpanMsg,
  SignalTrackMsg,
} from "./protocol";

/**
 * Client-side model of a baked sequence rollout, for the SFC chart: the
 * flatten mirror (span attribution), the active step at a playback time,
 * and best-effort truth of transition conditions against the timeline's
 * signal lanes and move spans.
 *
 * `flattenSteps` MUST mirror `rollout::flatten` in
 * `crates/botrail-scene/src/rollout.rs`: same pre-order (a branching step,
 * then each arm's steps depth-first in arm order), same select-ordinal
 * numbering. The Rust side pins that numbering against
 * `seq::enumerate_selects`; this side is checked at runtime by
 * `attributionIssues` — every span's flat index must land on a node with
 * the span's display name, so a drift between the two flattens shows up
 * on the first bake, not as a silently wrong chart.
 */

/** One flattened step — the client twin of rollout's `FlatStep`. */
export type FlatNode = {
  /** Display name (unprefixed; spans prefix `"{sequence}/"` when several
   * programs bake together). */
  name: string;
  /** The authored step behind this node (actions, transition, arms). */
  step: StepMsg;
  /** Pre-order select ordinal when this is a branching step — the
   * numbering `BranchTakenMsg.select` refers to. */
  select: number | null;
  /** Innermost containing arm: flat index of the branching step plus arm
   * index. Walk `armAncestors` for the full chain. */
  armOf: { branch: number; arm: number } | null;
};

export function flattenSteps(steps: StepMsg[]): FlatNode[] {
  const out: FlatNode[] = [];
  let selects = 0;
  const emit = (steps: StepMsg[], armOf: FlatNode["armOf"]) => {
    for (const step of steps) {
      const arms: SelectArmMsg[] = step.select ?? [];
      if (arms.length === 0) {
        out.push({ name: step.name, step, select: null, armOf });
      } else {
        const here = out.length;
        out.push({ name: step.name, step, select: selects, armOf });
        selects += 1;
        arms.forEach((arm, index) => {
          emit(arm.steps, { branch: here, arm: index });
        });
      }
    }
  };
  emit(steps, null);
  return out;
}

/** The chain of arm memberships from `index` outward: innermost first. */
export function armAncestors(
  nodes: FlatNode[],
  index: number,
): { branch: number; arm: number }[] {
  const chain: { branch: number; arm: number }[] = [];
  let armOf = nodes[index]?.armOf ?? null;
  while (armOf) {
    chain.push(armOf);
    armOf = nodes[armOf.branch].armOf;
  }
  return chain;
}

/** A step's outgoing transitions: the arm guards for a branching step
 * (authored order = priority), otherwise its own transition. */
export function exitConditions(node: FlatNode): ConditionMsg[] {
  const arms = node.step.select ?? [];
  return arms.length > 0
    ? arms.map((arm) => arm.condition)
    : [node.step.transition];
}

/** `BranchTakenMsg` rows of one sequence, as select ordinal → arm index. */
export function takenArms(
  branches: BranchTakenMsg[],
  sequence: string,
): Map<number, number> {
  const taken = new Map<number, number>();
  for (const branch of branches) {
    if (branch.sequence === sequence) taken.set(branch.select, branch.arm);
  }
  return taken;
}

/** One program's bake: its flatten plus, per flat index, the step's span
 * on the timeline (null = never entered on this bake). */
export type ProgramRun = {
  sequence: string;
  nodes: FlatNode[];
  spans: (StepSpanMsg | null)[];
};

export function attributeSpans(
  sequence: SequenceMsg,
  stepSpans: StepSpanMsg[],
): ProgramRun {
  const nodes = flattenSteps(sequence.steps);
  const spans: (StepSpanMsg | null)[] = nodes.map(() => null);
  for (const span of stepSpans) {
    if (span.sequence !== sequence.name) continue;
    if (span.step < nodes.length) spans[span.step] = span;
  }
  return { sequence: sequence.name, nodes, spans };
}

/**
 * Cross-checks a baked timeline against the authored sequences and this
 * file's flatten mirror. Returns human-readable problems; the caller
 * `console.warn`s them once per timeline. Empty on every healthy bake —
 * anything here means the two flattens drifted (or the authored tree
 * changed after the bake, which a re-simulate clears).
 */
export function attributionIssues(
  sequences: SequenceMsg[],
  timeline: { stepSpans: StepSpanMsg[]; branches: BranchTakenMsg[] },
): string[] {
  const issues: string[] = [];
  const flats = new Map(
    sequences.map((s) => [s.name, flattenSteps(s.steps)] as const),
  );
  for (const span of timeline.stepSpans) {
    const flat = flats.get(span.sequence);
    if (!flat) {
      issues.push(`span \`${span.name}\`: unknown sequence \`${span.sequence}\``);
      continue;
    }
    const node = flat[span.step];
    if (!node) {
      issues.push(
        `span \`${span.name}\`: flat index ${span.step} out of range for ` +
          `\`${span.sequence}\` (${flat.length} nodes)`,
      );
    } else if (
      span.name !== node.name &&
      span.name !== `${span.sequence}/${node.name}`
    ) {
      issues.push(
        `span \`${span.name}\` (flat ${span.step}): flatten mirror names ` +
          `it \`${node.name}\``,
      );
    }
  }
  for (const branch of timeline.branches) {
    const flat = flats.get(branch.sequence);
    const node = flat?.find((n) => n.select === branch.select);
    if (!node) {
      issues.push(
        `branch at \`${branch.sequence}\`/\`${branch.step}\`: no select ` +
          `with ordinal ${branch.select} in the flatten mirror`,
      );
    } else if (node.name !== branch.step) {
      issues.push(
        `branch ordinal ${branch.select}: flatten mirror names it ` +
          `\`${node.name}\`, timeline says \`${branch.step}\``,
      );
    } else if (branch.arm >= (node.step.select?.length ?? 0)) {
      issues.push(
        `branch \`${branch.sequence}\`/\`${branch.step}\`: arm ${branch.arm} ` +
          `out of range`,
      );
    }
  }
  return issues;
}

/** Where a program's token sits at time `t`. */
export type ProgramState =
  | { kind: "idle" }
  | { kind: "running"; index: number; span: StepSpanMsg }
  | { kind: "finished"; index: number; span: StepSpanMsg };

/**
 * The active flat step at `t`: the last entered span with `start <= t`,
 * so an immediate chain (several zero-width spans at one instant) resolves
 * to its final step. Past the last span's end the program is finished —
 * the token parks on the step it ended on.
 */
export function activeAt(run: ProgramRun, t: number): ProgramState {
  // Entry order equals flat order within one program (exits only point
  // forward), so a flat-index walk is a temporal walk.
  let active: { index: number; span: StepSpanMsg } | null = null;
  let lastEntered = -1;
  for (let index = 0; index < run.spans.length; index++) {
    const span = run.spans[index];
    if (!span) continue;
    lastEntered = index;
    if (span.start <= t) active = { index, span };
  }
  if (!active) return { kind: "idle" };
  if (active.index === lastEntered && t >= active.span.end) {
    return { kind: "finished", ...active };
  }
  return { kind: "running", ...active };
}

/**
 * Truth of a transition condition at time `t`, with per-atom detail for
 * the chart. Levels and timers are exact reconstructions from the baked
 * lanes; `approx` marks the display conventions that are not:
 * edge conditions are instantaneous in the scan model, so they "hold"
 * for `EDGE_HOLD` seconds after firing (long enough to see), and
 * `device_done` reads the device's running lane, which for a conveyor
 * differs from the in-position test the rollout used.
 */
export type Truth = {
  cond: ConditionMsg;
  holds: boolean;
  /** `elapsed` only: fraction of the delay served, 0..1. */
  progress?: number;
  /** `rising`/`falling` only: the signal's level at `t`. */
  level?: boolean;
  /** `rising`/`falling` only: when it last fired at or before `t`. */
  firedAt?: number;
  /** Set when `holds` is a display approximation. */
  approx?: boolean;
  children: Truth[];
};

/** How long an edge atom stays lit after firing (display convention). */
export const EDGE_HOLD = 0.15;

/** What `truth` evaluates against: the active step's own span (timer
 * base and move attribution) plus the timeline's lanes and move spans. */
export type TruthCtx = {
  span: StepSpanMsg;
  signals: SignalTrackMsg[];
  robots: { name: string; moves: StepSpanMsg[] }[];
};

/** `BoolTrack::value_at`: the last edge at or before `t` holds. */
export function laneValue(lane: SignalTrackMsg, t: number): boolean {
  let value = false;
  for (let i = 0; i < lane.times.length; i++) {
    if (lane.times[i] > t + 1e-9) break;
    value = lane.values[i];
  }
  return value;
}

/** The last `rising ? off→on : on→off` flank at or before `t`.
 * `times[0]` is the startup state, not an edge — the rollout seeds its
 * edge memory the same way. */
function lastFlank(
  lane: SignalTrackMsg,
  t: number,
  rising: boolean,
): number | undefined {
  let fired: number | undefined;
  for (let i = 1; i < lane.times.length; i++) {
    if (lane.times[i] > t + 1e-9) break;
    if (lane.values[i] === rising && lane.values[i - 1] !== rising) {
      fired = lane.times[i];
    }
  }
  return fired;
}

export function truth(cond: ConditionMsg, t: number, ctx: TruthCtx): Truth {
  const lane = (name: string) => ctx.signals.find((s) => s.name === name);
  const edge = (name: string, rising: boolean): Truth => {
    const track = lane(name);
    const firedAt = track && lastFlank(track, t, rising);
    return {
      cond,
      holds: firedAt !== undefined && t - firedAt <= EDGE_HOLD,
      level: track ? laneValue(track, t) : false,
      firedAt,
      approx: true,
      children: [],
    };
  };
  switch (cond.type) {
    case "immediately":
      return { cond, holds: true, children: [] };
    case "done": {
      // Exact: X0 stamps every move span with the step that started it.
      const started = ctx.robots.flatMap((r) =>
        r.moves.filter(
          (m) => m.sequence === ctx.span.sequence && m.step === ctx.span.step,
        ),
      );
      return {
        cond,
        holds: started.every((m) => m.end <= t + 1e-9),
        children: [],
      };
    }
    case "robot_done": {
      const moves = ctx.robots.find((r) => r.name === cond.robot)?.moves ?? [];
      return {
        cond,
        holds: !moves.some((m) => m.start <= t && t < m.end - 1e-9),
        children: [],
      };
    }
    case "elapsed": {
      const served = t - ctx.span.start;
      return {
        cond,
        holds: served + 1e-9 >= cond.seconds,
        progress:
          cond.seconds > 0 ? Math.min(Math.max(served / cond.seconds, 0), 1) : 1,
        children: [],
      };
    }
    case "signal": {
      const track = lane(cond.name);
      return {
        cond,
        holds: track ? laneValue(track, t) === cond.value : false,
        children: [],
      };
    }
    case "rising":
      return edge(cond.name, true);
    case "falling":
      return edge(cond.name, false);
    case "device_done": {
      const track = lane(cond.device);
      return {
        cond,
        holds: track ? !laneValue(track, t) : false,
        approx: true,
        children: [],
      };
    }
    case "all": {
      const children = cond.conditions.map((c) => truth(c, t, ctx));
      return { cond, holds: children.every((c) => c.holds), children };
    }
    case "any": {
      const children = cond.conditions.map((c) => truth(c, t, ctx));
      return { cond, holds: children.some((c) => c.holds), children };
    }
  }
}
