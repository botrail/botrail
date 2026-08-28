import { useMemo, useState } from "react";

import { useDockReserve } from "../dockReserve";
import {
  buildLadder,
  coilFire,
  contactState,
  rails,
  rungStatus,
  tonState,
  COMMENT_H,
  TON_H,
  TON_W,
  type LContact,
  type LCoil,
  type LGroup,
  type LRung,
  type LTon,
  type Ladder,
} from "../ladder";
import { samplePlayback } from "../playback";
import type {
  BranchTakenMsg,
  SequenceMsg,
  SignalTrackMsg,
  StepSpanMsg,
} from "../protocol";
import {
  activeAt,
  attributeSpans,
  stepStatus,
  takenArms,
  type ProgramRun,
} from "../sfc";
import { actionLabel } from "../seqLabels";
import { useStudioStore } from "../store";
import { OverlayTabs } from "./OverlayTabs";

/**
 * The cell's programs as SET/RST step ladders (ladder.ts lays them out):
 * one rung group per step under its `S{flat} · name` comment, the way a
 * Mitsubishi-style 歩進回路 reads. Static geometry is memoized; the live
 * layer overprints conducting contacts in green at the playhead, flashes
 * coils the instant a rung fires, runs the TON countdowns, and frames
 * the active step's whole rung group with the token — the ladder as a
 * PLC IDE's monitor mode shows it.
 */

const BG = "rgba(23, 26, 33, 0.96)";

const clip = (s: string) => (s.length > 19 ? s.slice(0, 18) + "…" : s);

type BakedTimeline = {
  stepSpans: StepSpanMsg[];
  branches: BranchTakenMsg[];
  signals: SignalTrackMsg[];
  robots: { name: string; moves: StepSpanMsg[] }[];
  duration: number;
  scenario: string | null;
};

function seekTo(t: number) {
  const s = useStudioStore.getState();
  if (!s.playback) return;
  s.setPlaying(false);
  s.setPlayback(t, samplePlayback(s.playback, t));
}

// ---------------------------------------------------------------- glyphs

/** One contact: `-| |-`, NC slash, or a P/N edge letter in the gap. The
 * live layer re-draws it with `on` (and a backing rect so the green
 * label prints clean over the gray one). */
function ContactGlyph({ c, on }: { c: LContact; on?: boolean }) {
  const { cx, y } = c;
  const half = Math.min(c.label.length, 19) * 3.1 + 2;
  return (
    <g>
      {on && (
        <rect x={cx - half} y={y - 18} width={half * 2} height={11} fill={BG} />
      )}
      <line className="ld-sym" x1={cx - 5.5} y1={y - 8} x2={cx - 5.5} y2={y + 8} />
      <line className="ld-sym" x1={cx + 5.5} y1={y - 8} x2={cx + 5.5} y2={y + 8} />
      {c.kind === "nc" && (
        <line className="ld-sym" x1={cx - 8} y1={y + 9} x2={cx + 8} y2={y - 9} />
      )}
      {(c.kind === "p" || c.kind === "n") && (
        <text className="ld-op" x={cx} y={y + 3} textAnchor="middle">
          {c.kind === "p" ? "P" : "N"}
        </text>
      )}
      <text x={cx} y={y - 9} textAnchor="middle">
        {clip(c.label)}
      </text>
      {c.note && (
        <text className="ld-note" x={cx} y={y + 17} textAnchor="middle">
          {clip(c.note)}
        </text>
      )}
    </g>
  );
}

/** An output coil; SET/RST carry their letter, plain command coils are
 * empty circles. The note under a SET names the step it starts. */
function CoilGlyph({ o, on }: { o: LCoil; on?: boolean }) {
  const half = Math.min(o.label.length, 19) * 3.1 + 2;
  return (
    <g>
      {on && (
        <rect x={o.cx - half} y={o.y - 20} width={half * 2} height={11} fill={BG} />
      )}
      <circle className="ld-sym" cx={o.cx} cy={o.y} r={7} />
      {o.op !== "out" && (
        <text className="ld-op" x={o.cx} y={o.y + 3} textAnchor="middle">
          {o.op === "set" ? "S" : "R"}
        </text>
      )}
      <text x={o.cx} y={o.y - 11} textAnchor="middle">
        {clip(o.label)}
      </text>
      {o.note && (
        <text className="ld-note" x={o.cx} y={o.y + 18} textAnchor="middle">
          {clip(o.note)}
        </text>
      )}
    </g>
  );
}

function TonGlyph({ b }: { b: LTon }) {
  return (
    <g>
      <rect
        className="ld-sym"
        x={b.x}
        y={b.y - TON_H / 2}
        width={TON_W}
        height={TON_H}
        rx={3}
      />
      <text x={b.x + TON_W / 2} y={b.y - 1} textAnchor="middle">
        TON {b.name}
      </text>
      <text className="ld-note" x={b.x + TON_W / 2} y={b.y + 9} textAnchor="middle">
        PT {b.seconds.toFixed(2)}s
      </text>
    </g>
  );
}

function RungGlyphs({ rung, status }: { rung: LRung; status: string }) {
  return (
    <g className={`ld-rung ${status}`}>
      <title>{rung.title}</title>
      {rung.wires.map((w, i) => (
        <line key={i} className="ld-wire" x1={w.x1} y1={w.y1} x2={w.x2} y2={w.y2} />
      ))}
      {rung.contacts.map((c, i) => (
        <ContactGlyph key={`c${i}`} c={c} />
      ))}
      {rung.coils.map((o, i) => (
        <CoilGlyph key={`o${i}`} o={o} />
      ))}
      {rung.tons.map((b, i) => (
        <TonGlyph key={`b${i}`} b={b} />
      ))}
    </g>
  );
}

// ------------------------------------------------------------- live layer

/** A group's lower edge: the bottom of its last rung (bare comment if none). */
function groupBottom(g: LGroup): number {
  const last = g.rungs[g.rungs.length - 1];
  return last ? last.top + last.h : g.top + COMMENT_H;
}

/**
 * The per-frame layer: monitor-mode truth on every entered rung's
 * contacts (green = conducting, an underline = an edge contact's signal
 * level), coil flashes on the firing instant, TON countdowns, and the
 * token framing the active step's whole rung group — comment row down to
 * its last rung, so the box says exactly which rungs the step owns.
 */
function LadderLive({
  ladder,
  run,
  taken,
  statuses,
  timeline,
}: {
  ladder: Ladder;
  run: ProgramRun;
  taken: Map<number, number>;
  statuses: Map<LRung, string>;
  timeline: BakedTimeline;
}) {
  const t = useStudioStore((s) => s.playbackTime);
  const lanes = { signals: timeline.signals, robots: timeline.robots };
  const lit: JSX.Element[] = [];
  for (const group of ladder.groups) {
    for (const [ri, rung] of group.rungs.entries()) {
      if ((statuses.get(rung) ?? "") !== "") continue;
      const key = `${group.flat ?? "s"}:${ri}`;
      rung.contacts.forEach((c, i) => {
        const st = contactState(c.ref, t, run, lanes);
        if (st.holds) {
          lit.push(
            <g key={`${key}c${i}`} className="ld-on">
              <ContactGlyph c={c} on />
            </g>,
          );
        }
        if (st.level) {
          const half = Math.min(c.label.length, 19) * 3.1;
          lit.push(
            <line
              key={`${key}l${i}`}
              className={st.holds ? "ld-on ld-lv" : "ld-lv"}
              x1={c.cx - half}
              y1={c.y - 6}
              x2={c.cx + half}
              y2={c.y - 6}
            />,
          );
        }
      });
      if (rung.coils.length > 0 && coilFire(rung, t, run, taken)) {
        rung.coils.forEach((o, i) => {
          lit.push(
            <g key={`${key}o${i}`} className="ld-on ld-fire">
              <CoilGlyph o={o} on />
            </g>,
          );
        });
      }
      rung.tons.forEach((b, i) => {
        const ts = tonState(b, t, run);
        if (!ts.active) return;
        lit.push(
          <g key={`${key}b${i}`} className={ts.done ? "ld-on" : undefined}>
            <rect
              x={b.x + 1}
              y={b.y + TON_H / 2 - 4}
              width={(TON_W - 2) * ts.frac}
              height={3}
              className="ld-ton-fill"
            />
            <rect x={b.x + 6} y={b.y + 1} width={TON_W - 12} height={10} fill={BG} />
            <text className="ld-note" x={b.x + TON_W / 2} y={b.y + 9} textAnchor="middle">
              {ts.served.toFixed(2)}/{b.seconds.toFixed(2)}s
            </text>
          </g>,
        );
      });
    }
  }
  const state = activeAt(run, t);
  const active =
    state.kind === "idle"
      ? null
      : ladder.groups.find((g) => g.flat === state.index) ?? null;
  return (
    <>
      <svg className="ld-live" width={ladder.width} height={ladder.height}>
        {lit}
      </svg>
      {active && (
        <div
          className={
            state.kind === "finished" ? "sfc-token ld-token done" : "sfc-token ld-token"
          }
          style={{
            left: 6,
            top: active.top - 2,
            width: ladder.width - 12,
            height: groupBottom(active) - active.top + 4,
          }}
        />
      )}
    </>
  );
}

// --------------------------------------------------------------- program

type Mode =
  | { kind: "authored" }
  | { kind: "baked"; run: ProgramRun; taken: Map<number, number> };

function ProgramLadder({
  sequence,
  timeline,
}: {
  sequence: SequenceMsg;
  timeline: BakedTimeline | null;
}) {
  const ladder = useMemo(() => buildLadder(sequence.steps), [sequence]);
  const nodes = useMemo(
    () => attributeSpans(sequence, timeline?.stepSpans ?? []).nodes,
    [sequence, timeline],
  );
  const mode: Mode = useMemo(() => {
    if (!timeline || !timeline.stepSpans.some((s) => s.sequence === sequence.name)) {
      return { kind: "authored" };
    }
    return {
      kind: "baked",
      run: attributeSpans(sequence, timeline.stepSpans),
      taken: takenArms(timeline.branches, sequence.name),
    };
  }, [sequence, timeline]);
  const statuses = useMemo(() => {
    const m = new Map<LRung, string>();
    if (mode.kind === "baked") {
      for (const g of ladder.groups) {
        for (const r of g.rungs) m.set(r, rungStatus(mode.run, mode.taken, r));
      }
    }
    return m;
  }, [ladder, mode]);
  const [railL, railR] = rails(ladder.width);

  return (
    <div className="sfc-program">
      <h4>{sequence.name}</h4>
      <div
        className="ld-canvas"
        style={{ width: ladder.width, height: ladder.height }}
      >
        <svg width={ladder.width} height={ladder.height}>
          <line className="ld-rail" x1={railL} y1={4} x2={railL} y2={ladder.height - 4} />
          <line className="ld-rail" x1={railR} y1={4} x2={railR} y2={ladder.height - 4} />
          {ladder.groups.map((g, gi) => (
            <g key={gi}>
              {g.rungs.map((r, ri) => (
                <RungGlyphs key={ri} rung={r} status={statuses.get(r) ?? ""} />
              ))}
            </g>
          ))}
        </svg>
        {ladder.groups.map((g) => {
          const span =
            mode.kind === "baked" && g.flat !== null ? mode.run.spans[g.flat] : null;
          const dim =
            mode.kind === "baked" && g.flat !== null
              ? stepStatus(mode.run, mode.taken, g.flat)
              : "";
          const label =
            g.flat === null ? "start" : `S${g.flat} · ${g.name}${g.branching ? " ◇" : ""}`;
          const actions =
            g.flat !== null ? nodes[g.flat]?.step.actions.map(actionLabel).join("  ") : "";
          const when = span
            ? `${span.start.toFixed(2)}–${span.end.toFixed(2)}s — click seeks here`
            : null;
          return (
            <div
              key={g.flat ?? "start"}
              className={["ld-comment", span ? "clickable" : "", dim === "entered" ? "" : dim]
                .filter(Boolean)
                .join(" ")}
              style={{ left: railL + 4, top: g.top, maxWidth: ladder.width - 28 }}
              title={[label, actions, when].filter(Boolean).join("\n")}
              onClick={span ? () => seekTo(span.start) : undefined}
            >
              {label}
            </div>
          );
        })}
        {mode.kind === "baked" && timeline && (
          <LadderLive
            ladder={ladder}
            run={mode.run}
            taken={mode.taken}
            statuses={statuses}
            timeline={timeline}
          />
        )}
      </div>
    </div>
  );
}

// --------------------------------------------------------------- overlay

/** The ladder panel over the viewport — the SFC chart's sibling view;
 * `ldOpen` persists in localStorage (toggles live in the Run section and
 * the timeline dock, and the panel's tab strip switches). */
export function LadderOverlay() {
  const open = useStudioStore((s) => s.ldOpen);
  const setOpen = useStudioStore((s) => s.setLdOpen);
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
    <div className="sfc-overlay ld-overlay" ref={setPanel}>
      <div className="sfc-head">
        <OverlayTabs active="ld" />
        <span className="sfc-caption">
          {timeline
            ? `${timeline.scenario ? `⧉ ${timeline.scenario} — ` : ""}cycle ${timeline.duration.toFixed(2)}s`
            : "authored — Simulate to see the taken path"}
        </span>
        <button
          className="timeline-button"
          onClick={() => setOpen(false)}
          title="close (the ☰ Ladder button reopens it)"
        >
          ×
        </button>
      </div>
      {programs.length === 0 ? (
        <div className="sfc-empty hint">no programs yet — add sequence steps</div>
      ) : (
        <div className="sfc-scroll ld-scroll">
          {programs.map((seq) => (
            <ProgramLadder key={seq.name} sequence={seq} timeline={timeline} />
          ))}
        </div>
      )}
    </div>
  );
}
