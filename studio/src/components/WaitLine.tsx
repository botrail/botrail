import { Fragment, useMemo, useState } from "react";

import { addressOf, pointKey } from "../ioLinks";
import { tagLine } from "../ioTags";
import { useStudioStore } from "../store";
import { waitBreakdown, waitingLines, type WaitItem } from "../waits";

/**
 * The dock's answer to "what is it waiting for?": for every program at the
 * playhead (or the live head), the step it stands in, the contacts of its
 * exits that do not conduct yet — with the address of the point each reads
 * — and how long it has stood there; then the bound points that are on.
 * `Σ` folds out where the run's time went, per program and releasing
 * exit. Clicking a contact or a chip puts its point in focus across the
 * studio (I/O table, ladder, SFC chart, lanes, 3D tag).
 */
export function WaitLine() {
  const timeline = useStudioStore((s) => s.timeline);
  const sequences = useStudioStore((s) => s.sequences);
  const points = useStudioStore((s) => s.io.points);
  const t = useStudioStore((s) => s.playbackTime);
  const plant = useStudioStore((s) => s.plant);
  const hot = useStudioStore((s) => s.highlightLane);
  const setHot = useStudioStore((s) => s.setHighlightLane);
  const [open, setOpen] = useState(false);
  // Instant stays (an immediate chain's steps) took no time: left out.
  const shares = useMemo(
    () =>
      timeline && open
        ? waitBreakdown(sequences, timeline).filter((s) => s.seconds > 0.005)
        : [],
    [sequences, timeline, open],
  );
  if (!timeline || timeline.stepSpans.length === 0) return null;

  const lines = waitingLines(sequences, timeline, t, points);
  const on = points.filter((p) => {
    if (!addressOf(p)) return false;
    return tagLine(p, timeline, t, plant).state === "on";
  });
  const seen = new Set<string>();
  const chips = on.filter((p) => {
    const k = `${addressOf(p)} ${p.label}`;
    if (seen.has(k)) return false;
    seen.add(k);
    return true;
  });
  const focus = (key: string | null) => {
    if (key) setHot(hot === key ? null : key);
  };
  const item = (it: WaitItem, i: number) => (
    <span
      key={i}
      className={`wait-item${it.key !== null && it.key === hot ? " hot" : ""}${
        it.level ? " lv" : ""
      }`}
      title={it.address ? `${it.text} — ${it.address}` : it.text}
      onClick={() => focus(it.key)}
    >
      {it.address && <span className="wait-addr">{it.address}</span>}
      {it.text}
    </span>
  );

  return (
    <>
      <div className="wait-line">
        <span className="wait-programs">
          {lines.length === 0 ? (
            <span className="wait-idle">no program waits here</span>
          ) : (
            lines.map((l) => (
              <span key={l.program} className="wait-program">
                <span className="wait-step" title={`${l.program} / S${l.flat} ${l.step}`}>
                  {l.program} › {l.step}
                </span>
                {l.exits.every((items) => items.length === 0) ? (
                  <span className="wait-item">→</span>
                ) : (
                  l.exits.map((items, i) => (
                    <Fragment key={i}>
                      {i > 0 && <span className="wait-sep">|</span>}
                      {items.map(item)}
                    </Fragment>
                  ))
                )}
                <span className="wait-since">{Math.max(0, t - l.since).toFixed(1)}s</span>
              </span>
            ))
          )}
        </span>
        {chips.length > 0 && (
          <span className="wait-on" title={chips.map((p) => `${addressOf(p)} ${p.label}`).join("\n")}>
            {chips.slice(0, 8).map((p) => (
              <span
                key={`${addressOf(p)} ${p.label}`}
                className={`wait-chip${pointKey(p) === hot ? " hot" : ""}`}
                onClick={() => focus(pointKey(p))}
              >
                {addressOf(p)}
              </span>
            ))}
            {chips.length > 8 && <span className="wait-chip more">+{chips.length - 8}</span>}
          </span>
        )}
        <button
          className={open ? "timeline-button timeline-button-on" : "timeline-button"}
          onClick={() => setOpen(!open)}
          title="where the run's time went: each program's stays, summed per exit that released them"
        >
          Σ waits
        </button>
      </div>
      {open && (
        <div className="wait-sum">
          {shares.length === 0 ? (
            <span className="wait-idle">no stay was released on this run</span>
          ) : (
            shares.slice(0, 10).map((s) => (
              <span key={`${s.program} ${s.condition}`} className="wait-share">
                <span className="wait-share-time">{s.seconds.toFixed(2)}s</span>
                <span>{s.condition}</span>
                <span className="wait-share-n">
                  ×{s.count} · {s.program}
                </span>
              </span>
            ))
          )}
        </div>
      )}
    </>
  );
}
