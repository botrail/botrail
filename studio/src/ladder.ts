import type { ActionMsg, ConditionMsg, StepMsg } from "./protocol";
import { actionLabel, conditionLabel } from "./seqLabels";
import {
  flattenSteps,
  stepStatus,
  truth,
  EDGE_HOLD,
  type ProgramRun,
  type TruthCtx,
} from "./sfc";

/**
 * SET/RST step-ladder (歩進回路) projection of the authored sequences —
 * the same programs the SFC chart draws, written the way a PLC engineer
 * would ladder them: one internal relay per step (`S{flat}`), one rung
 * per transition (`-|Sk|--[condition]--(S next)(R k)-`), entry actions
 * as output rungs on the step relay, and `elapsed` conditions as TON
 * timers driven by it. A selection's arms become one rung per guard in
 * authored order — rung order is scan order, so first-wins priority is
 * exactly the rollout's.
 *
 * Pure model + geometry; `components/LadderChart.tsx` draws it. Flat
 * indices follow `flattenSteps` (the rollout's pre-order), so spans,
 * branches and the live truth attribute the same way as the SFC chart.
 */

// ------------------------------------------------------------- geometry

const RAIL_L = 10; // left power rail x
const RAIL_R = 24; // right power rail inset from the program's width
const STEP_W = 50; // the leading step-relay contact cell
const LABEL_H = 16; // room above a rung line for device labels
const NOTE_H = 16; // room below it for notes
const ROW = 32; // pitch of stacked branch / output rows
export const COMMENT_H = 15; // the step-comment row over each group
const GROUP_GAP = 8;
const MIN_RUN = 20; // shortest wire between network and outputs
const JUNC = 8; // parallel-branch junction stub
export const TON_W = 84;
export const TON_H = 24;
const PAD_T = 8;
const PAD_B = 8;
const MIN_W = 320;

/** A contact cell sized to its label (leads included). */
function cellW(label: string): number {
  return Math.min(130, Math.max(44, 16 + label.length * 6.2));
}

export const relay = (flat: number) => `S${flat}`;

// ------------------------------------------- condition -> contact network

export type ContactKind = "no" | "nc" | "p" | "n";

/** Series-parallel contact network of one condition tree: `all` lays its
 * children in series, `any` stacks them as parallel branches, and every
 * leaf is one contact. `immediately` is a bare wire — the "else" rung. */
type Net =
  | {
      k: "contact";
      kind: ContactKind;
      label: string;
      note?: string;
      cond: ConditionMsg;
      w: number;
      rows: number;
    }
  | { k: "wire"; w: number; rows: number }
  | { k: "series"; ch: Net[]; w: number; rows: number }
  | { k: "parallel"; ch: Net[]; w: number; rows: number };

/** Allocates `T0, T1, …` per program, in encounter (= flat) order; the
 * step's TON rungs and its transition's timer contacts share the name. */
type TimerAlloc = {
  n: number;
  /** Timers created while building the current step's exits. */
  created: { name: string; seconds: number }[];
};

function contactNet(
  kind: ContactKind,
  label: string,
  cond: ConditionMsg,
  note?: string,
): Net {
  return { k: "contact", kind, label, note, cond, w: cellW(label), rows: 1 };
}

function buildNet(cond: ConditionMsg, timers: TimerAlloc): Net {
  switch (cond.type) {
    case "immediately":
      return { k: "wire", w: 18, rows: 1 };
    case "done":
      return contactNet("no", "done", cond);
    case "robot_done":
      return contactNet("no", `${cond.robot}.done`, cond);
    case "group_done":
      return contactNet("no", `${cond.robot}/${cond.group}.done`, cond);
    case "device_done":
      return contactNet("no", `${cond.device}.done`, cond);
    case "signal":
      return contactNet(cond.value ? "no" : "nc", cond.name, cond);
    case "rising":
      return contactNet("p", cond.name, cond);
    case "falling":
      return contactNet("n", cond.name, cond);
    case "elapsed": {
      const name = `T${timers.n++}`;
      timers.created.push({ name, seconds: cond.seconds });
      return contactNet("no", name, cond, conditionLabel(cond));
    }
    case "all": {
      const ch = cond.conditions.map((c) => buildNet(c, timers));
      if (ch.length === 0) return { k: "wire", w: 18, rows: 1 };
      return {
        k: "series",
        ch,
        w: ch.reduce((s, c) => s + c.w, 0),
        rows: ch.reduce((m, c) => Math.max(m, c.rows), 1),
      };
    }
    case "any": {
      const ch = cond.conditions.map((c) => buildNet(c, timers));
      if (ch.length === 0) return { k: "wire", w: 18, rows: 1 };
      return {
        k: "parallel",
        ch,
        w: ch.reduce((m, c) => Math.max(m, c.w), 0) + JUNC * 2,
        rows: ch.reduce((s, c) => s + c.rows, 0),
      };
    }
  }
}

// ------------------------------------------------- successors (exits)

export type Exit = { cond: ConditionMsg; target: number | null; arm: number | null };

function subtreeSize(step: StepMsg): number {
  return (
    1 +
    (step.select ?? []).reduce(
      (n, arm) => n + arm.steps.reduce((m, s) => m + subtreeSize(s), 0),
      0,
    )
  );
}

/**
 * Per flat step, its outgoing transitions with SET targets: the next
 * sibling, an arm's first step (guards), or the join past the branch —
 * `null` at the program's end (the last rung only resets itself). The
 * walk allocates flat indices in the same pre-order as `flattenSteps`.
 */
export function flatExits(steps: StepMsg[]): Exit[][] {
  const out: Exit[][] = [];
  const walk = (list: StepMsg[], start: number, cont: number | null) => {
    let at = start;
    const flats = list.map((s) => {
      const f = at;
      at += subtreeSize(s);
      return f;
    });
    list.forEach((step, i) => {
      const flat = flats[i];
      const next = i + 1 < list.length ? flats[i + 1] : cont;
      const arms = step.select ?? [];
      if (arms.length === 0) {
        out[flat] = [{ cond: step.transition, target: next, arm: null }];
      } else {
        out[flat] = [];
        let armStart = flat + 1;
        arms.forEach((arm, j) => {
          out[flat].push({
            cond: arm.condition,
            target: arm.steps.length > 0 ? armStart : next,
            arm: j,
          });
          walk(arm.steps, armStart, next);
          armStart += arm.steps.reduce((m, s) => m + subtreeSize(s), 0);
        });
      }
    });
  };
  walk(steps, 0, null);
  return out;
}

// ------------------------------------------------------------ the ladder

export type LWire = { x1: number; y1: number; x2: number; y2: number };

export type ContactRef =
  | { type: "step"; flat: number }
  | { type: "start" }
  /** A condition leaf; `flat` is the step whose exit it guards (the span
   * that anchors `elapsed` / `done`). */
  | { type: "cond"; flat: number; cond: ConditionMsg };

export type LContact = {
  kind: ContactKind;
  label: string;
  note?: string;
  cx: number;
  y: number;
  ref: ContactRef;
};

export type LCoil = {
  op: "set" | "rst" | "out";
  label: string;
  note?: string;
  cx: number;
  y: number;
};

export type LTon = { name: string; seconds: number; x: number; y: number; flat: number };

export type LRung = {
  kind: "start" | "action" | "timer" | "transition";
  /** Owning flat step; null for the start rung. */
  flat: number | null;
  /** Guard rung of a branching step: which arm. */
  arm: number | null;
  /** SET target of a transition rung (null = end of program). */
  target: number | null;
  top: number;
  h: number;
  wires: LWire[];
  contacts: LContact[];
  coils: LCoil[];
  tons: LTon[];
  title: string;
};

export type LGroup = {
  /** null = the start rung's group. */
  flat: number | null;
  name: string;
  branching: boolean;
  top: number;
  rungs: LRung[];
};

export type Ladder = { groups: LGroup[]; width: number; height: number };

type Out =
  | { op: "set" | "rst" | "out"; label: string; note?: string }
  | { op: "ton"; name: string; seconds: number };

type Skel = {
  kind: LRung["kind"];
  flat: number | null;
  arm: number | null;
  target: number | null;
  lead: { kind: ContactKind; label: string; ref: ContactRef };
  net: Net | null;
  outs: Out[];
  title: string;
};

function actionOut(action: ActionMsg): Out {
  if (action.type === "set") {
    return { op: action.value ? "set" : "rst", label: action.signal };
  }
  return { op: "out", label: actionLabel(action) };
}

function outZone(outs: Out[]): number {
  const ton = outs.some((o) => o.op === "ton");
  if (ton) return TON_W + 10;
  return outs.length > 1 ? 45 : 23;
}

/** Lays a network down from `x` on line `y`; returns the x it ends at.
 * Parallel branches stack downward on the ROW pitch, shorter ones wired
 * out to the rejoin bar. */
function placeNet(net: Net, x: number, y: number, flat: number, rung: LRung): number {
  switch (net.k) {
    case "wire":
      rung.wires.push({ x1: x, y1: y, x2: x + net.w, y2: y });
      return x + net.w;
    case "contact": {
      const cx = x + net.w / 2;
      rung.wires.push({ x1: x, y1: y, x2: cx - 6, y2: y });
      rung.wires.push({ x1: cx + 6, y1: y, x2: x + net.w, y2: y });
      rung.contacts.push({
        kind: net.kind,
        label: net.label,
        note: net.note,
        cx,
        y,
        ref: { type: "cond", flat, cond: net.cond },
      });
      return x + net.w;
    }
    case "series": {
      let ax = x;
      for (const c of net.ch) ax = placeNet(c, ax, y, flat, rung);
      return ax;
    }
    case "parallel": {
      const innerW = net.w - JUNC * 2;
      let by = y;
      let lastY = y;
      for (const c of net.ch) {
        rung.wires.push({ x1: x, y1: by, x2: x + JUNC, y2: by });
        const end = placeNet(c, x + JUNC, by, flat, rung);
        if (end < x + JUNC + innerW) {
          rung.wires.push({ x1: end, y1: by, x2: x + JUNC + innerW, y2: by });
        }
        rung.wires.push({ x1: x + JUNC + innerW, y1: by, x2: x + net.w, y2: by });
        lastY = by;
        by += c.rows * ROW;
      }
      rung.wires.push({ x1: x, y1: y, x2: x, y2: lastY });
      rung.wires.push({ x1: x + net.w, y1: y, x2: x + net.w, y2: lastY });
      return x + net.w;
    }
  }
}

export function buildLadder(steps: StepMsg[]): Ladder {
  const nodes = flattenSteps(steps);
  const exits = flatExits(steps);
  if (exits.length !== nodes.length) {
    // Both walks mirror rollout's flatten; a drift here means this file's
    // successor walk no longer matches `flattenSteps`.
    console.warn(
      `ladder: flatten mirror drift (${exits.length} exits, ${nodes.length} nodes)`,
    );
  }

  // ---- skeletons: rungs per group, nets built in flat order ----------
  const timers: TimerAlloc = { n: 0, created: [] };
  const groups: { flat: number | null; name: string; branching: boolean; skels: Skel[] }[] =
    [];

  if (nodes.length > 0) {
    groups.push({
      flat: null,
      name: "start",
      branching: false,
      skels: [
        {
          kind: "start",
          flat: null,
          arm: null,
          target: 0,
          lead: { kind: "p", label: "start", ref: { type: "start" } },
          net: null,
          outs: [{ op: "set", label: `${relay(0)} ${nodes[0].name}` }],
          title: `start → ${relay(0)} ${nodes[0].name}`,
        },
      ],
    });
  }

  nodes.forEach((node, flat) => {
    const skels: Skel[] = [];
    const lead = { kind: "no" as ContactKind, label: relay(flat), ref: { type: "step", flat } as ContactRef };
    if ((node.step.select ?? []).length === 0 && node.step.actions.length > 0) {
      skels.push({
        kind: "action",
        flat,
        arm: null,
        target: null,
        lead,
        net: null,
        outs: node.step.actions.map(actionOut),
        title: `${node.name} — entry actions`,
      });
    }
    timers.created = [];
    const nets = (exits[flat] ?? []).map((exit) => ({
      exit,
      net: buildNet(exit.cond, timers),
    }));
    for (const t of timers.created) {
      skels.push({
        kind: "timer",
        flat,
        arm: null,
        target: null,
        lead,
        net: null,
        outs: [{ op: "ton", name: t.name, seconds: t.seconds }],
        title: `${t.name}: ${node.name} active ${t.seconds.toFixed(2)}s`,
      });
    }
    for (const { exit, net } of nets) {
      const to = exit.target !== null ? `${relay(exit.target)} ${nodes[exit.target].name}` : "end";
      skels.push({
        kind: "transition",
        flat,
        arm: exit.arm,
        target: exit.target,
        lead,
        net,
        outs: [
          ...(exit.target !== null
            ? [{ op: "set" as const, label: `${relay(exit.target)} ${nodes[exit.target].name}` }]
            : []),
          { op: "rst" as const, label: relay(flat) },
        ],
        title: `${node.name} —[ ${conditionLabel(exit.cond)} ]→ ${to}`,
      });
    }
    groups.push({
      flat,
      name: node.name,
      branching: (node.step.select ?? []).length > 0,
      skels,
    });
  });

  // ---- width: the widest rung decides the rails ----------------------
  let need = 0;
  for (const g of groups) {
    for (const s of g.skels) {
      const netW = s.net ? s.net.w : 0;
      need = Math.max(need, netW + MIN_RUN + outZone(s.outs));
    }
  }
  const width = Math.max(MIN_W, RAIL_L + STEP_W + need + RAIL_R);
  const railR = width - RAIL_R;

  // ---- place ---------------------------------------------------------
  const out: Ladder = { groups: [], width, height: 0 };
  let y = PAD_T;
  for (const g of groups) {
    const group: LGroup = {
      flat: g.flat,
      name: g.name,
      branching: g.branching,
      top: y,
      rungs: [],
    };
    y += COMMENT_H;
    for (const s of g.skels) {
      const rows = Math.max(s.net?.rows ?? 1, s.outs.length);
      const h = LABEL_H + (rows - 1) * ROW + NOTE_H;
      const rung: LRung = {
        kind: s.kind,
        flat: s.flat,
        arm: s.arm,
        target: s.target,
        top: y,
        h,
        wires: [],
        contacts: [],
        coils: [],
        tons: [],
        title: s.title,
      };
      const ly = y + LABEL_H;
      // Leading contact (the step relay; `start` on the first rung).
      const lcx = RAIL_L + STEP_W / 2;
      rung.wires.push({ x1: RAIL_L, y1: ly, x2: lcx - 6, y2: ly });
      rung.wires.push({ x1: lcx + 6, y1: ly, x2: RAIL_L + STEP_W, y2: ly });
      rung.contacts.push({ kind: s.lead.kind, label: s.lead.label, cx: lcx, y: ly, ref: s.lead.ref });
      let x = RAIL_L + STEP_W;
      if (s.net) x = placeNet(s.net, x, ly, s.flat ?? 0, rung);
      // Outputs, right-aligned on the rail.
      const ton = s.outs.find((o) => o.op === "ton");
      if (ton && ton.op === "ton") {
        const bx = railR - TON_W - 10;
        rung.wires.push({ x1: x, y1: ly, x2: bx, y2: ly });
        rung.wires.push({ x1: bx + TON_W, y1: ly, x2: railR, y2: ly });
        rung.tons.push({ name: ton.name, seconds: ton.seconds, x: bx, y: ly, flat: s.flat ?? 0 });
      } else if (s.outs.length === 1) {
        const cx = railR - 16;
        rung.wires.push({ x1: x, y1: ly, x2: cx - 7, y2: ly });
        rung.wires.push({ x1: cx + 7, y1: ly, x2: railR, y2: ly });
        const o = s.outs[0];
        if (o.op !== "ton") rung.coils.push({ op: o.op, label: o.label, note: o.note, cx, y: ly });
      } else {
        const busX = railR - 45;
        const cx = railR - 16;
        rung.wires.push({ x1: x, y1: ly, x2: busX, y2: ly });
        rung.wires.push({ x1: busX, y1: ly, x2: busX, y2: ly + (s.outs.length - 1) * ROW });
        s.outs.forEach((o, i) => {
          const oy = ly + i * ROW;
          rung.wires.push({ x1: busX, y1: oy, x2: cx - 7, y2: oy });
          rung.wires.push({ x1: cx + 7, y1: oy, x2: railR, y2: oy });
          if (o.op !== "ton") rung.coils.push({ op: o.op, label: o.label, note: o.note, cx, y: oy });
        });
      }
      group.rungs.push(rung);
      y += h;
    }
    out.groups.push(group);
    y += GROUP_GAP;
  }
  out.height = y - GROUP_GAP + PAD_B;
  return out;
}

/** Where the rails run: `[left x, right x]` for a program's width. */
export function rails(width: number): [number, number] {
  return [RAIL_L, width - RAIL_R];
}

// ------------------------------------------------------------- live truth

type Lanes = Pick<TruthCtx, "signals" | "robots">;

/** The program's t=0: when its first entered step began. */
export function startTime(run: ProgramRun): number {
  for (const span of run.spans) if (span) return span.start;
  return 0;
}

/**
 * Monitor-mode truth of one contact at `t`. Step relays hold over their
 * span; `elapsed` contacts reset outside it (a TON with its IN dropped);
 * everything else is `sfc.truth` on the leaf against the owning step's
 * span. `level` is the edge contacts' signal level (shown as an
 * underline, the SFC chart's convention).
 */
export function contactState(
  ref: ContactRef,
  t: number,
  run: ProgramRun,
  lanes: Lanes,
): { holds: boolean; level?: boolean } {
  if (ref.type === "step") {
    const span = run.spans[ref.flat];
    return { holds: !!span && span.start <= t && t < span.end };
  }
  if (ref.type === "start") {
    const t0 = startTime(run);
    return { holds: t >= t0 && t - t0 <= EDGE_HOLD };
  }
  const span = run.spans[ref.flat];
  if (!span) return { holds: false };
  const tr = truth(ref.cond, t, { span, ...lanes });
  if (ref.cond.type === "elapsed") {
    return { holds: tr.holds && span.start <= t && t < span.end };
  }
  return { holds: tr.holds, level: tr.level };
}

/** TON block state: runs while its step relay is on, done at the preset. */
export function tonState(
  ton: LTon,
  t: number,
  run: ProgramRun,
): { active: boolean; done: boolean; served: number; frac: number } {
  const span = run.spans[ton.flat];
  const active = !!span && span.start <= t && t < span.end;
  const served = active ? Math.min(t - span!.start, ton.seconds) : 0;
  const frac = ton.seconds > 0 ? served / ton.seconds : 1;
  return { active, done: active && served >= ton.seconds, served, frac };
}

/**
 * Whether a rung's outputs flash at `t`: action rungs fire on step entry,
 * transition rungs the instant their exit released the step (for a guard
 * rung, only the arm the bake took — its rivals never fire). The window
 * is the SFC chart's afterglow.
 */
export function coilFire(
  rung: LRung,
  t: number,
  run: ProgramRun,
  taken: Map<number, number>,
): boolean {
  const windowed = (f: number) => t >= f && t - f <= EDGE_HOLD;
  if (rung.kind === "start") return windowed(startTime(run));
  if (rung.flat === null) return false;
  const span = run.spans[rung.flat];
  if (!span) return false;
  if (rung.kind === "action") return windowed(span.start);
  if (rung.kind !== "transition") return false;
  if (rung.arm !== null) {
    const ordinal = run.nodes[rung.flat].select;
    if (ordinal === null || taken.get(ordinal) !== rung.arm) return false;
  }
  // A step cut off by the bake's end never fired its exit.
  if (rung.target !== null && !run.spans[rung.target]) return false;
  return windowed(span.end);
}

/**
 * Render status of one rung: `""` normal, `off` a guard the bake decided
 * against, `untaken`/`unreached` the SFC chart's step statuses.
 */
export function rungStatus(
  run: ProgramRun,
  taken: Map<number, number>,
  rung: LRung,
): "" | "off" | "untaken" | "unreached" {
  if (rung.flat === null) return "";
  const base = stepStatus(run, taken, rung.flat);
  if (base !== "entered") return base;
  if (rung.kind === "transition" && rung.arm !== null) {
    const ordinal = run.nodes[rung.flat].select;
    if (ordinal !== null && taken.has(ordinal) && taken.get(ordinal) !== rung.arm) {
      return "off";
    }
  }
  return "";
}
