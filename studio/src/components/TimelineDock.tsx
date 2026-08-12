import { useRef, useState } from "react";

import { samplePlayback } from "../playback";
import { useStudioStore } from "../store";
import { sendExportUsd } from "../ws";

const BAND_COLORS = ["#4a6fa5", "#5a8f6a", "#a5824a", "#7a5aa5", "#a55a6f"];
const ROBOT_LANE_COLOR = "#4a8fa5";

/**
 * Timing-chart overlay under the viewport: step bands, a playhead with
 * click-to-seek, cycle time, one motion lane per robot (multi-robot
 * timelines), and one lane per signal. Rendered only when a sequence
 * timeline is loaded.
 */
/** Fraction of the cycle these move intervals cover, overlaps merged —
 * the line-balancing number, shown beside each robot lane so the
 * bottleneck is readable straight off the chart. */
function utilization(
  moves: { start: number; end: number }[],
  duration: number,
): number {
  if (duration <= 0) return 0;
  const spans = [...moves].sort((a, b) => a.start - b.start);
  let total = 0;
  let open: { start: number; end: number } | null = null;
  for (const span of spans) {
    if (open && span.start <= open.end) {
      open.end = Math.max(open.end, span.end);
    } else {
      if (open) total += open.end - open.start;
      open = { start: span.start, end: span.end };
    }
  }
  if (open) total += open.end - open.start;
  return total / duration;
}

function SignalLane({
  signal,
  duration,
}: {
  signal: { name: string; times: number[]; values: boolean[] };
  duration: number;
}) {
  const pct = (t: number) => `${(t / duration) * 100}%`;
  return (
    <div className="timeline-lane">
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
  );
}

export function TimelineDock() {
  const timeline = useStudioStore((s) => s.timeline);
  const playback = useStudioStore((s) => s.playback);
  const recording = useStudioStore((s) => s.recording);
  const segmentEnds = useStudioStore((s) => s.segmentEnds);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const setPlayback = useStudioStore((s) => s.setPlayback);
  const setPlaying = useStudioStore((s) => s.setPlaying);
  const playing = useStudioStore((s) => s.playing);
  const speed = useStudioStore((s) => s.playbackSpeed);
  const setSpeed = useStudioStore((s) => s.setPlaybackSpeed);
  const loop = useStudioStore((s) => s.playbackLoop);
  const setLoop = useStudioStore((s) => s.setPlaybackLoop);
  const sfcOpen = useStudioStore((s) => s.sfcOpen);
  const setSfcOpen = useStudioStore((s) => s.setSfcOpen);
  const barRef = useRef<HTMLDivElement | null>(null);
  const [showDevices, setShowDevices] = useState(false);

  // The one transport bar: motion/plan previews play here too, they just
  // have no step bands or signal lanes — only segment tick marks.
  if (!playback || playback.duration <= 0) return null;
  const duration = playback.duration;
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
    setPlayback(t, samplePlayback(playback, t));
  };

  const pct = (t: number) => `${(t / duration) * 100}%`;

  return (
    <div className="timeline-dock">
      <div className="timeline-head">
        <span>
          {recordingLabel ? `● ${recordingLabel} — ` : ""}
          {timeline?.scenario ? `⧉ ${timeline.scenario} — ` : ""}
          {timeline ? "cycle" : "preview"} {duration.toFixed(2)}s
        </span>
        <span className="timeline-controls">
          {/* A 60-90 s takt is unwatchable at 1x; speed and loop are how a
              line cycle actually gets reviewed. */}
          <button
            className="timeline-button"
            onClick={() => setPlaying(!playing)}
            title={playing ? "pause" : "play"}
          >
            {playing ? "❚❚" : "▶"}
          </button>
          <button
            className="timeline-button"
            onClick={() => setSpeed(speed >= 8 ? 1 : speed * 2)}
            title="playback speed"
          >
            {speed}×
          </button>
          <button
            className={loop ? "timeline-button timeline-button-on" : "timeline-button"}
            onClick={() => setLoop(!loop)}
            title="loop"
          >
            ⟳
          </button>
          {timeline && (
            <button
              className={
                sfcOpen ? "timeline-button timeline-button-on" : "timeline-button"
              }
              onClick={() => setSfcOpen(!sfcOpen)}
              title="SFC chart of the baked programs"
            >
              sfc
            </button>
          )}
          {/* Sequence timelines only: the server bakes the retained
              rollout, so a motion preview or a loaded recording has
              nothing to re-export. */}
          {timeline && !recording && (
            <button
              className="timeline-button"
              onClick={() => sendExportUsd(60)}
              title="download this cycle as a USD animation (.usda)"
            >
              ⤓ usd
            </button>
          )}
          <span>{playbackTime.toFixed(2)}s</span>
        </span>
      </div>
      <div className="timeline-bands" ref={barRef} onClick={seek}>
        {(timeline?.stepSpans ?? []).map((span, i) => (
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
        {/* Motion previews: mark the planned segment boundaries. */}
        {!timeline &&
          segmentEnds.map((t, i) => (
            <div key={i} className="timeline-tick" style={{ left: pct(t) }} />
          ))}
        <div className="timeline-playhead" style={{ left: pct(playbackTime) }} />
      </div>
      {/* Robot lanes: when several robots run, each gets a band lane of its
          move intervals (labelled with the motion name). */}
      {timeline &&
        timeline.robots.length > 1 &&
        timeline.robots.map((robot) => (
          <div key={robot.name} className="timeline-lane">
            <span
              className="timeline-lane-name"
              title={`${robot.name} — ${(utilization(robot.moves, duration) * 100).toFixed(0)}% busy`}
            >
              {robot.name}{" "}
              <span className="timeline-lane-util">
                {(utilization(robot.moves, duration) * 100).toFixed(0)}%
              </span>
            </span>
            <div className="timeline-lane-track">
              {robot.moves.map((move, i) => (
                <div
                  key={i}
                  className="timeline-lane-on"
                  title={`${move.name} · ${move.start.toFixed(2)}–${move.end.toFixed(2)}s`}
                  style={{
                    left: pct(move.start),
                    width: `max(${((move.end - move.start) / duration) * 100}%, 2px)`,
                    background: ROBOT_LANE_COLOR,
                  }}
                />
              ))}
            </div>
          </div>
        ))}
      {/* Process lanes (internal signals + sensor inputs) always show;
          device output lanes fold away by default — a line's worth of
          sources and sinks is hundreds of them, and unfolded they bury
          the chart (and the viewport). */}
      {(timeline?.signals ?? [])
        .filter((signal) => signal.kind !== "device")
        .map((signal) => (
          <SignalLane key={signal.name} signal={signal} duration={duration} />
        ))}
      {timeline?.signals.some((signal) => signal.kind === "device") && (
        <div className="timeline-lane">
          <button
            className="timeline-button"
            onClick={() => setShowDevices(!showDevices)}
          >
            {showDevices ? "▾" : "▸"} devices (
            {timeline.signals.filter((s) => s.kind === "device").length})
          </button>
        </div>
      )}
      {showDevices &&
        (timeline?.signals ?? [])
          .filter((signal) => signal.kind === "device")
          .map((signal) => (
            <SignalLane
              key={signal.name}
              signal={signal}
              duration={duration}
            />
          ))}
    </div>
  );
}
