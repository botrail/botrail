import { useMemo, useRef, useState } from "react";

import { samplePlayback } from "../playback";
import type { StepSpanMsg } from "../protocol";
import { useStudioStore } from "../store";
import { sendExportUsd } from "../ws";
import { chipsForLane } from "./IoOverlay";

const BAND_COLORS = ["#4a6fa5", "#5a8f6a", "#a5824a", "#7a5aa5", "#a55a6f"];
const ROBOT_LANE_COLOR = "#4a8fa5";

/**
 * Timing-chart overlay under the viewport: step bands, a playhead with
 * click-to-seek, cycle time, one motion lane per robot (multi-robot
 * timelines), and one lane per signal. Rendered only when a sequence
 * timeline is loaded.
 */
/** One lane of move intervals: a robot, or one arm of a dual-arm robot. */
interface MoveLane {
  name: string;
  moves: StepSpanMsg[];
}

/** The move lanes of a baked timeline: one per robot, split per arm where
 * the moves name one (`robot/arm`, or just the arm when the robot is
 * alone in the scene), unnamed moves on the robot's own. */
function moveLanes(robots: MoveLane[]): MoveLane[] {
  const lanes: MoveLane[] = [];
  const sole = robots.length === 1;
  for (const robot of robots) {
    const byArm = new Map<string, StepSpanMsg[]>();
    for (const move of robot.moves) {
      const arm = move.group ?? "";
      const lane = byArm.get(arm);
      if (lane) lane.push(move);
      else byArm.set(arm, [move]);
    }
    if (byArm.size === 0) {
      lanes.push({ name: robot.name, moves: [] });
      continue;
    }
    for (const arm of [...byArm.keys()].sort()) {
      lanes.push({
        name: arm === "" ? robot.name : sole ? arm : `${robot.name}/${arm}`,
        moves: byArm.get(arm) ?? [],
      });
    }
  }
  return lanes;
}

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
  chips,
  hot,
}: {
  signal: { name: string; times: number[]; values: boolean[] };
  duration: number;
  /** The channels the lane's points are bound to (`UR.DI2 · %IX0.2`):
   * with them the chart reads as the FAT I/O waveform sheet. */
  chips: string[];
  /** Picked in the topology diagram. */
  hot: boolean;
}) {
  const pct = (t: number) => `${(t / duration) * 100}%`;
  return (
    <div className={`timeline-lane${hot ? " timeline-lane-hot" : ""}`}>
      <span
        className="timeline-lane-name"
        title={chips.length > 0 ? `${signal.name} — ${chips.join(", ")}` : signal.name}
      >
        {signal.name}
      </span>
      {chips.length > 0 && (
        <span className="timeline-lane-chips" title={chips.join(", ")}>
          {chips.slice(0, 2).map((c) => (
            <span key={c} className="io-chip">
              {c}
            </span>
          ))}
          {chips.length > 2 && <span className="io-chip">+{chips.length - 2}</span>}
        </span>
      )}
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
  const lanes = useMemo(
    () => (timeline ? moveLanes(timeline.robots) : []),
    [timeline],
  );
  const playback = useStudioStore((s) => s.playback);
  const recording = useStudioStore((s) => s.recording);
  const cameras = useStudioStore((s) => s.cameras);
  const pipCamera = useStudioStore((s) => s.pipCamera);
  const pipMode = useStudioStore((s) => s.pipMode);
  const camExport = useStudioStore((s) => s.camExport);
  const camExportProgress = useStudioStore((s) => s.camExportProgress);
  const beginCamExport = useStudioStore((s) => s.beginCamExport);
  // The PiP camera (the one being watched) is the recording target; else
  // the first camera in the scene. A PiP showing depth records the depth
  // colormap — what you see is what downloads.
  const camTarget = pipCamera ?? cameras[0]?.name ?? null;
  const camDepthViz = pipCamera !== null && pipMode === "depth";
  const webcodecs = typeof VideoEncoder !== "undefined";
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
  const ldOpen = useStudioStore((s) => s.ldOpen);
  const setLdOpen = useStudioStore((s) => s.setLdOpen);
  const ioOpen = useStudioStore((s) => s.ioOpen);
  const setIoOpen = useStudioStore((s) => s.setIoOpen);
  const topoOpen = useStudioStore((s) => s.topoOpen);
  const setTopoOpen = useStudioStore((s) => s.setTopoOpen);
  const highlightLane = useStudioStore((s) => s.highlightLane);
  const ioPoints = useStudioStore((s) => s.io.points);
  const sequenceError = useStudioStore((s) => s.sequenceError);
  const sequenceErrorScenario = useStudioStore((s) => s.sequenceErrorScenario);
  const barRef = useRef<HTMLDivElement | null>(null);
  const [showDevices, setShowDevices] = useState(false);
  // Lane name -> channel chips, from the I/O map's bound points.
  const chips = useMemo(() => {
    const m = new Map<string, string[]>();
    for (const lane of timeline?.signals ?? []) {
      m.set(lane.name, chipsForLane(ioPoints, lane.name));
    }
    return m;
  }, [ioPoints, timeline]);

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
          {timeline && (
            <button
              className={
                ldOpen ? "timeline-button timeline-button-on" : "timeline-button"
              }
              onClick={() => setLdOpen(!ldOpen)}
              title="SET/RST ladder of the baked programs"
            >
              ld
            </button>
          )}
          {timeline && (
            <button
              className={
                ioOpen ? "timeline-button timeline-button-on" : "timeline-button"
              }
              onClick={() => setIoOpen(!ioOpen)}
              title="the I/O table — points, channels, live levels"
            >
              io
            </button>
          )}
          {timeline && (
            <button
              className={
                topoOpen ? "timeline-button timeline-button-on" : "timeline-button"
              }
              onClick={() => setTopoOpen(!topoOpen)}
              title="the electrical topology — controllers, channels, wires, handshakes"
            >
              topo
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
          {/* Any playback (bake, recording, motion preview) can be filmed
              through a camera; the export runs in the browser itself. */}
          {playback && camTarget && (
            <button
              className="timeline-button"
              disabled={!webcodecs || camExport !== null}
              onClick={() =>
                beginCamExport(
                  camTarget,
                  30,
                  camDepthViz ? { viz: "depth" } : undefined,
                )
              }
              title={
                webcodecs
                  ? camDepthViz
                    ? `record ${camTarget}'s depth colormap as WebM, 30 fps (deterministic re-run of the bake)`
                    : `record ${camTarget}'s view as WebM, 30 fps (deterministic re-run of the bake)`
                  : "video export needs WebCodecs (Chrome, Edge, or a recent Firefox)"
              }
            >
              {camExport ? `${Math.round(camExportProgress * 100)}%` : "⤓ cam"}
            </button>
          )}
          <span>{playbackTime.toFixed(2)}s</span>
        </span>
      </div>
      {/* A run that did not complete leaves the last bake on the dock and
          says why here — under a fault scenario the diagnosis names the
          stalled step and the forced point, which is the result. */}
      {sequenceError && (
        <div className="timeline-diagnosis" title={sequenceError}>
          ⚠ {sequenceErrorScenario ? `⧉ ${sequenceErrorScenario} — ` : ""}
          {sequenceError}
        </div>
      )}
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
      {/* Robot lanes: when several robots (or the arms of one) run, each
          gets a band lane of its move intervals (labelled with the motion
          name). */}
      {lanes.length > 1 &&
        lanes.map((robot) => (
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
          <SignalLane
            key={signal.name}
            signal={signal}
            duration={duration}
            chips={chips.get(signal.name) ?? []}
            hot={highlightLane === signal.name}
          />
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
              chips={chips.get(signal.name) ?? []}
              hot={highlightLane === signal.name}
            />
          ))}
    </div>
  );
}
