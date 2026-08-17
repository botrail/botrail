import { Fragment, useMemo, useState } from "react";

import { useDockReserve } from "../dockReserve";
import { samplePlayback } from "../playback";
import type {
  BranchTakenMsg,
  SequenceMsg,
  SignalTrackMsg,
  StepMsg,
  StepSpanMsg,
} from "../protocol";
import {
  activeAt,
  armAncestors,
  attributeSpans,
  takenArms,
  truth,
  EDGE_HOLD,
  type ProgramRun,
  type Truth,
} from "../sfc";
import { actionLabel, conditionLabel } from "../seqLabels";
import { useStudioStore } from "../store";
import { OverlayTabs } from "./OverlayTabs";

/**
 * The cell's programs as SFC/GRAFCET charts, overlaid on the viewport:
 * step boxes joined by transition bars (condition beside each), selection
 * divergences fanning into one lane per arm and rejoining below. One
 * column per program — the standard notation for 工程歩進, and a plain
 * flowchart for everyone else.
 *
 * Two modes: authored (neutral — nothing baked yet, or this program was
 * not part of the bake) and baked, where the taken path reads normally,
 * arms the bake skipped are dashed out, and the winning guard is green.
 */

const COL_W = 148; // one lane
const BOX_W = 132;
const BOX_H = 26;
const TR_H = 30; // transition segment below a box (line + tick + label)
const RAIL = 10; // gap between a branch box / arm ends and the rails
const PAD = 12;

type Box = {
  x: number;
  y: number;
  flat: number;
  step: StepMsg;
  branching: boolean;
  initial: boolean;
};
type Tick = {
  x: number;
  y: number;
  label: string;
  /** The step this transition leaves — arm guards sit on the branching
   * step itself, `arm` telling which guard. */
  flat: number;
  arm: number | null;
};
type Line = { x1: number; y1: number; x2: number; y2: number };
type Layout = {
  boxes: Box[];
  ticks: Tick[];
  lines: Line[];
  width: number;
  height: number;
};

/** Footprint of a step list: lanes it spans × height it runs. Heights
 * must agree exactly with `place` — both walk the same shapes. */
function measure(steps: StepMsg[]): { w: number; h: number } {
  let w = COL_W;
  let h = 0;
  for (const step of steps) {
    const arms = step.select ?? [];
    if (arms.length === 0) {
      h += BOX_H + TR_H;
    } else {
      const sizes = arms.map((arm) => measure(arm.steps));
      w = Math.max(
        w,
        sizes.reduce((sum, s) => sum + s.w, 0),
      );
      const tallest = sizes.reduce((m, s) => Math.max(m, s.h), 0);
      h += BOX_H + RAIL + TR_H + tallest + RAIL + RAIL;
    }
  }
  return { w, h };
}

/** Lays `steps` down lane-centered on `cx` from `y`, appending into
 * `out`. The flat cursor advances in the same pre-order as
 * `sfc.flattenSteps`, so every box knows its flat index. Returns the y
 * below the list's last transition. */
function place(
  steps: StepMsg[],
  cx: number,
  y: number,
  out: Layout,
  cursor: { i: number },
): number {
  for (const step of steps) {
    const flat = cursor.i++;
    const arms = step.select ?? [];
    out.boxes.push({
      x: cx - BOX_W / 2,
      y,
      flat,
      step,
      branching: arms.length > 0,
      initial: flat === 0,
    });
    if (arms.length === 0) {
      out.lines.push({ x1: cx, y1: y + BOX_H, x2: cx, y2: y + BOX_H + TR_H });
      out.ticks.push({
        x: cx,
        y: y + BOX_H + TR_H / 2,
        label: conditionLabel(step.transition),
        flat,
        arm: null,
      });
      y += BOX_H + TR_H;
    } else {
      const sizes = arms.map((arm) => measure(arm.steps));
      const total = sizes.reduce((sum, s) => sum + s.w, 0);
      const railY = y + BOX_H + RAIL;
      const armTop = railY + TR_H;
      const tallest = sizes.reduce((m, s) => Math.max(m, s.h), 0);
      const joinY = armTop + tallest + RAIL;

      out.lines.push({ x1: cx, y1: y + BOX_H, x2: cx, y2: railY });
      let ax = cx - total / 2;
      const centers: number[] = [];
      arms.forEach((arm, j) => {
        const laneCx = ax + sizes[j].w / 2;
        centers.push(laneCx);
        // Guard transition into the arm, then the arm body, then a
        // straight drop to the rejoin rail (shorter arms fall further).
        out.lines.push({ x1: laneCx, y1: railY, x2: laneCx, y2: armTop });
        out.ticks.push({
          x: laneCx,
          y: railY + TR_H / 2,
          label: conditionLabel(arm.condition),
          flat,
          arm: j,
        });
        const armEnd = place(arm.steps, laneCx, armTop, out, cursor);
        out.lines.push({ x1: laneCx, y1: armEnd, x2: laneCx, y2: joinY });
        ax += sizes[j].w;
      });
      const first = centers[0];
      const last = centers[centers.length - 1];
      // Diverge / rejoin rails across the arm lanes.
      out.lines.push({ x1: first, y1: railY, x2: last, y2: railY });
      out.lines.push({ x1: first, y1: joinY, x2: last, y2: joinY });
      out.lines.push({ x1: cx, y1: joinY, x2: cx, y2: joinY + RAIL });
      y = joinY + RAIL;
    }
  }
  return y;
}

function layoutProgram(steps: StepMsg[]): Layout {
  const size = measure(steps);
  const out: Layout = {
    boxes: [],
    ticks: [],
    lines: [],
    width: size.w + PAD * 2,
    height: size.h + PAD * 2,
  };
  place(steps, PAD + size.w / 2, PAD, out, { i: 0 });
  return out;
}

type BakedTimeline = {
  stepSpans: StepSpanMsg[];
  branches: BranchTakenMsg[];
  signals: SignalTrackMsg[];
  robots: { name: string; moves: StepSpanMsg[] }[];
  duration: number;
  scenario: string | null;
};

/** Box clicks seek the shared transport to the step's entry instant —
 * the chart-side half of the chart ⇄ timeline linking. */
function seekTo(t: number) {
  const s = useStudioStore.getState();
  if (!s.playback) return;
  s.setPlaying(false);
  s.setPlayback(t, samplePlayback(s.playback, t));
}

type Mode =
  | { kind: "authored" }
  | { kind: "baked"; run: ProgramRun; taken: Map<number, number> };

/** entered = has a span on this bake; untaken = sits on an arm the bake
 * decided against; unreached = the program never got this far. */
function boxStatus(mode: Mode, flat: number): string {
  if (mode.kind === "authored") return "";
  if (mode.run.spans[flat]) return "entered";
  for (const { branch, arm } of armAncestors(mode.run.nodes, flat)) {
    const ordinal = mode.run.nodes[branch].select;
    if (ordinal !== null && mode.taken.has(ordinal) && mode.taken.get(ordinal) !== arm) {
      return "untaken";
    }
  }
  return "unreached";
}

/** Guard ticks: the arm the bake took reads green, its rivals dim. */
function tickStatus(mode: Mode, tick: Tick): string {
  if (mode.kind === "authored") return "";
  if (tick.arm !== null) {
    const ordinal = mode.run.nodes[tick.flat].select;
    if (ordinal !== null && mode.taken.has(ordinal)) {
      return mode.taken.get(ordinal) === tick.arm ? "taken" : "off";
    }
    return boxStatus(mode, tick.flat) ? "off" : "";
  }
  const status = boxStatus(mode, tick.flat);
  return status === "untaken" || status === "unreached" ? "off" : "";
}

function boxTitle(box: Box, span: StepSpanMsg | null): string {
  const actions = box.step.actions.map(actionLabel).join("  ");
  const exit = box.branching
    ? `${(box.step.select ?? []).length} arms`
    : `→ ${conditionLabel(box.step.transition)}`;
  const when = span
    ? `${span.start.toFixed(2)}–${span.end.toFixed(2)}s — click seeks here`
    : null;
  return [box.step.name, actions, exit, when].filter(Boolean).join("\n");
}

/** Renders a truth tree as per-atom spans: satisfied contacts green, open
 * ones gray, timers as a served/total countdown. Nested composites keep
 * their parens; the top level reads like the static label. */
function Atoms({ tr }: { tr: Truth }) {
  if (tr.children.length === 0) {
    const text =
      tr.cond.type === "elapsed"
        ? `${((tr.progress ?? 1) * tr.cond.seconds).toFixed(2)}/${tr.cond.seconds.toFixed(2)}s`
        : conditionLabel(tr.cond);
    const cls = (tr.holds ? "hold" : "idle") + (tr.level ? " lv" : "");
    return <span className={cls}>{text}</span>;
  }
  const sep = tr.cond.type === "all" ? " & " : " | ";
  return (
    <>
      {tr.children.map((child, i) => (
        <Fragment key={i}>
          {i > 0 && <span className="sep">{sep}</span>}
          {child.children.length > 0 ? (
            <>
              <span className="sep">(</span>
              <Atoms tr={child} />
              <span className="sep">)</span>
            </>
          ) : (
            <Atoms tr={child} />
          )}
        </Fragment>
      ))}
    </>
  );
}

/** The step whose exit released within the afterglow window — its
 * satisfied condition keeps glowing at the old position just after the
 * token hops, so the release cause stays readable. Ties (an immediate
 * chain ending together) go to the chain's last step. */
function lastReleased(
  run: ProgramRun,
  t: number,
): { index: number; span: StepSpanMsg } | null {
  let released: { index: number; span: StepSpanMsg } | null = null;
  for (let index = 0; index < run.spans.length; index++) {
    const span = run.spans[index];
    if (!span || span.end > t || t - span.end > EDGE_HOLD) continue;
    if (!released || span.end >= released.span.end) released = { index, span };
  }
  return released;
}

/**
 * The per-frame layer (D6): the geometry above is memoized; only this
 * subscribes to the playhead. One token per program, live per-atom truth
 * on the active step's exits, and a brief afterglow on the exit that just
 * released.
 */
function LiveLayer({
  layout,
  run,
  timeline,
}: {
  layout: Layout;
  run: ProgramRun;
  timeline: BakedTimeline;
}) {
  const t = useStudioStore((s) => s.playbackTime);
  const state = activeAt(run, t);
  if (state.kind === "idle") return null;

  const liveTicks = (flat: number, span: StepSpanMsg) => {
    const node = run.nodes[flat];
    const ctx = { span, signals: timeline.signals, robots: timeline.robots };
    return layout.ticks
      .filter((tick) => tick.flat === flat)
      .map((tick) => {
        const cond =
          tick.arm !== null
            ? (node.step.select ?? [])[tick.arm].condition
            : node.step.transition;
        return (
          <span
            key={`${flat}:${tick.arm ?? "t"}`}
            className="sfc-live-cond"
            style={{ left: tick.x + 10, top: tick.y }}
          >
            <Atoms tr={truth(cond, t, ctx)} />
          </span>
        );
      });
  };

  const box = layout.boxes.find((b) => b.flat === state.index);
  const released = lastReleased(run, t);
  return (
    <>
      {box && (
        <div
          className={state.kind === "finished" ? "sfc-token done" : "sfc-token"}
          style={{
            left: box.x - 3,
            top: box.y - 3,
            width: BOX_W + 6,
            height: BOX_H + 6,
          }}
        />
      )}
      {state.kind === "running" && liveTicks(state.index, state.span)}
      {released &&
        !(state.kind === "running" && released.index === state.index) &&
        liveTicks(released.index, released.span)}
    </>
  );
}

function ProgramChart({
  sequence,
  timeline,
}: {
  sequence: SequenceMsg;
  timeline: BakedTimeline | null;
}) {
  const layout = useMemo(() => layoutProgram(sequence.steps), [sequence]);
  const mode: Mode = useMemo(() => {
    // A timeline colors only the programs it actually rolled; the rest
    // stay in authored neutral instead of reading as "never reached".
    if (!timeline || !timeline.stepSpans.some((s) => s.sequence === sequence.name)) {
      return { kind: "authored" };
    }
    return {
      kind: "baked",
      run: attributeSpans(sequence, timeline.stepSpans),
      taken: takenArms(timeline.branches, sequence.name),
    };
  }, [sequence, timeline]);

  return (
    <div className="sfc-program">
      <h4>{sequence.name}</h4>
      <div
        className="sfc-canvas"
        style={{ width: layout.width, height: layout.height }}
      >
        <svg width={layout.width} height={layout.height}>
          {layout.lines.map((l, i) => (
            <line key={i} x1={l.x1} y1={l.y1} x2={l.x2} y2={l.y2} />
          ))}
          {layout.ticks.map((t, i) => (
            <line
              key={`t${i}`}
              className={`sfc-tick ${tickStatus(mode, t)}`}
              x1={t.x - 6}
              y1={t.y}
              x2={t.x + 6}
              y2={t.y}
            />
          ))}
        </svg>
        {layout.ticks.map((t, i) => (
          <span
            key={i}
            className={`sfc-cond ${tickStatus(mode, t)}`}
            style={{ left: t.x + 10, top: t.y }}
          >
            {t.label}
          </span>
        ))}
        {layout.boxes.map((b) => {
          const span = mode.kind === "baked" ? mode.run.spans[b.flat] : null;
          return (
            <div
              key={b.flat}
              className={[
                "sfc-box",
                b.branching ? "branching" : "",
                b.initial ? "initial" : "",
                span ? "clickable" : "",
                boxStatus(mode, b.flat),
              ]
                .filter(Boolean)
                .join(" ")}
              style={{ left: b.x, top: b.y, width: BOX_W, height: BOX_H }}
              title={boxTitle(b, span)}
              data-flat={b.flat}
              onClick={span ? () => seekTo(span.start) : undefined}
            >
              <span className="sfc-name">
                {b.branching ? "◇ " : ""}
                {b.step.name}
              </span>
              {b.step.actions.map((a, k) => (
                <span key={k} className="seq-chip">
                  {actionLabel(a)}
                </span>
              ))}
            </div>
          );
        })}
        {mode.kind === "baked" && timeline && (
          <LiveLayer layout={layout} run={mode.run} timeline={timeline} />
        )}
      </div>
    </div>
  );
}

/** The chart panel over the viewport; `sfcOpen` persists in localStorage
 * (toggles live in the Run section and the timeline dock). */
export function SfcOverlay() {
  const open = useStudioStore((s) => s.sfcOpen);
  const setOpen = useStudioStore((s) => s.setSfcOpen);
  const sequences = useStudioStore((s) => s.sequences);
  const timeline = useStudioStore((s) => s.timeline);
  const docked = useStudioStore((s) => s.playback !== null);
  const [panel, setPanel] = useState<HTMLDivElement | null>(null);
  useDockReserve(panel, docked);
  const programs = useMemo(
    () => sequences.filter((s) => s.steps.length > 0),
    [sequences],
  );
  if (!open) return null;
  return (
    <div className="sfc-overlay" ref={setPanel}>
      <div className="sfc-head">
        <OverlayTabs active="sfc" />
        <span className="sfc-caption">
          {timeline
            ? `${timeline.scenario ? `⧉ ${timeline.scenario} — ` : ""}cycle ${timeline.duration.toFixed(2)}s`
            : "authored — Simulate to see the taken path"}
        </span>
        <button
          className="timeline-button"
          onClick={() => setOpen(false)}
          title="close (the ◫ SFC button reopens it)"
        >
          ×
        </button>
      </div>
      {programs.length === 0 ? (
        <div className="sfc-empty hint">no programs yet — add sequence steps</div>
      ) : (
        <div className="sfc-scroll">
          {programs.map((seq) => (
            <ProgramChart key={seq.name} sequence={seq} timeline={timeline} />
          ))}
        </div>
      )}
    </div>
  );
}
