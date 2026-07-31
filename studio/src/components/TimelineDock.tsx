import { useRef } from "react";

import { sampleOverride } from "../playback";
import { useStudioStore } from "../store";

const BAND_COLORS = ["#4a6fa5", "#5a8f6a", "#a5824a", "#7a5aa5", "#a55a6f"];

/**
 * Timing-chart overlay under the viewport: step bands, a playhead with
 * click-to-seek, cycle time, and one lane per signal. Rendered only when a
 * sequence timeline is loaded.
 */
export function TimelineDock() {
  const timeline = useStudioStore((s) => s.timeline);
  const trajectory = useStudioStore((s) => s.trajectory);
  const recording = useStudioStore((s) => s.recording);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const setPlayback = useStudioStore((s) => s.setPlayback);
  const setPlaying = useStudioStore((s) => s.setPlaying);
  const barRef = useRef<HTMLDivElement | null>(null);

  if (!timeline || !trajectory || timeline.duration <= 0) return null;
  const duration = timeline.duration;
  const recordingLabel = recording
    ? `${recording.source.split("/").pop()} (${recording.mode})`
    : null;

  const seek = (e: React.MouseEvent) => {
    const bar = barRef.current;
    if (!bar) return;
    const rect = bar.getBoundingClientRect();
    const frac = Math.min(Math.max((e.clientX - rect.left) / rect.width, 0), 1);
    const t = frac * duration;
    setPlaying(false);
    setPlayback(t, ...sampleOverride(trajectory, t));
  };

  const pct = (t: number) => `${(t / duration) * 100}%`;

  return (
    <div className="timeline-dock">
      <div className="timeline-head">
        <span>
          {recordingLabel ? `● ${recordingLabel} — ` : ""}
          cycle {duration.toFixed(2)}s
        </span>
        <span>{playbackTime.toFixed(2)}s</span>
      </div>
      <div className="timeline-bands" ref={barRef} onClick={seek}>
        {timeline.stepSpans.map((span, i) => (
          <div
            key={i}
            className="timeline-band"
            title={`${span.name} · ${span.start.toFixed(2)}–${span.end.toFixed(2)}s`}
            style={{
              left: pct(span.start),
              width: `max(${((span.end - span.start) / duration) * 100}%, 2px)`,
              background: BAND_COLORS[i % BAND_COLORS.length],
            }}
          >
            <span className="timeline-band-label">{span.name}</span>
          </div>
        ))}
        <div className="timeline-playhead" style={{ left: pct(playbackTime) }} />
      </div>
      {timeline.signals.map((signal) => (
        <div key={signal.name} className="timeline-lane">
          <span className="timeline-lane-name" title={signal.name}>
            {signal.name}
          </span>
          <div className="timeline-lane-track">
            {signal.times.map((t0, i) => {
              const t1 = signal.times[i + 1] ?? duration;
              if (!signal.values[i]) return null;
              return (
                <div
                  key={i}
                  className="timeline-lane-on"
                  style={{
                    left: pct(t0),
                    width: `max(${((t1 - t0) / duration) * 100}%, 2px)`,
                  }}
                />
              );
            })}
          </div>
        </div>
      ))}
    </div>
  );
}
