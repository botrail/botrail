import { useState } from "react";

import type { ConstraintMsg, MotionMsg, SegmentKindMsg } from "../protocol";
import { robotByName, useStudioStore } from "../store";
import { sendAddSegment, sendPlanMotion, sendRemoveSegment } from "../ws";
import { Section } from "./Section";

// "Upright" keeps the TCP's local +Z within a 30° cone of world +Z.
const UPRIGHT_CONE: ConstraintMsg = {
  type: "orientation_cone",
  axis_local: [0, 0, 1],
  axis_world: [0, 0, 1],
  angle: (Math.PI / 180) * 30,
};

/** `main` for the first robot, `main_<robot>` for the rest; when taken,
 * `motion_2`, `motion_3`, … — whatever is free scene-wide. */
function freshMotionName(
  motions: MotionMsg[],
  robotName: string,
  firstRobot: string | undefined,
): string {
  const taken = new Set(motions.map((m) => m.name));
  const base = robotName === firstRobot ? "main" : `main_${robotName}`;
  if (!taken.has(base)) return base;
  for (let i = 2; ; i++) {
    const candidate = `motion_${i}`;
    if (!taken.has(candidate)) return candidate;
  }
}

/**
 * Every motion in the scene (Python-authored ones included), one of them
 * the edit target; waypoint editing and planning below. The preview plays
 * in the timeline dock.
 */
export function MotionPanel() {
  const robots = useStudioStore((s) => s.robots);
  const robot = useStudioStore((s) => robotByName(s.robots, s.selectedRobot));
  const connected = useStudioStore((s) => s.connection === "connected");
  const motions = useStudioStore((s) => s.motions);
  const selectedMotion = useStudioStore((s) => s.selectedMotion);
  const selectMotion = useStudioStore((s) => s.selectMotion);
  const motionPlanning = useStudioStore((s) => s.motionPlanning);
  const motionError = useStudioStore((s) => s.motionError);
  const motionStats = useStudioStore((s) => s.motionStats);
  const segmentEnds = useStudioStore((s) => s.segmentEnds);
  const playback = useStudioStore((s) => s.playback);
  const beginMotionPlanning = useStudioStore((s) => s.beginMotionPlanning);

  const [upright, setUpright] = useState(false);

  if (!robot) return null;
  const robotName = robot.desc.name;
  const firstRobot = robots[0]?.desc.name;
  const multiRobot = robots.length > 1;
  const ownerOf = (m: MotionMsg) => m.robot ?? firstRobot;

  // The edit target: the picked motion, else the selected robot's first
  // motion, else the conventional fresh name (created on the first
  // waypoint — same implicit-create the server does).
  const owned = motions.filter((m) => ownerOf(m) === robotName);
  const fallbackName =
    owned[0]?.name ?? (robotName === firstRobot ? "main" : `main_${robotName}`);
  const motionName = selectedMotion ?? fallbackName;
  const motion = motions.find((m) => m.name === motionName) ?? null;
  const segments = motion?.segments ?? [];

  const addSegment = (kind: SegmentKindMsg) => {
    sendAddSegment(motionName, robotName, {
      kind,
      goal_positions: robot.jointPositions.slice(),
      constraints: upright ? [UPRIGHT_CONE] : [],
    });
  };

  const onPlan = () => {
    if (segments.length === 0) return;
    beginMotionPlanning();
    sendPlanMotion(motionName);
  };

  return (
    <Section
      id="motion"
      title="Motion"
      badge={
        <>
          {motionPlanning && <span className="badge muted">planning…</span>}
          {!motionPlanning && motionStats && playback && (
            <span className="badge ok">
              {playback.duration.toFixed(2)}s · {segmentEnds.length} seg ·{" "}
              {motionStats.planningTimeMs.toFixed(0)}ms
            </span>
          )}
        </>
      }
    >
      <div className="motion-controls">
        <div className="motion-picker">
          {motions.map((m) => (
            <div
              key={m.name}
              className={`obstacle-row${m.name === motionName ? " selected" : ""}`}
              onClick={() => selectMotion(m.name)}
            >
              <span className="obstacle-name">
                {m.name}
                {multiRobot && <span className="seq-cond"> · {ownerOf(m)}</span>}
              </span>
              <span className="seq-cond">{m.segments.length} wp</span>
            </div>
          ))}
          {!motion && (
            <div className="obstacle-row selected">
              <span className="obstacle-name">
                {motionName}
                {multiRobot && <span className="seq-cond"> · {robotName}</span>}
              </span>
              <span className="seq-cond">new</span>
            </div>
          )}
        </div>
        <button
          className="motion-new"
          onClick={() =>
            selectMotion(freshMotionName(motions, robotName, firstRobot))
          }
          title="start another motion (created when its first waypoint lands)"
        >
          + new motion
        </button>

        <div className="motion-add">
          <div className="seg">
            <button onClick={() => addSegment("joint")} disabled={!connected}>
              + Joint
            </button>
            <button
              onClick={() => addSegment("cartesian_line")}
              disabled={!connected}
            >
              + Line
            </button>
          </div>
          <button
            className={`upright-toggle${upright ? " active" : ""}`}
            title="Constrain the TCP upright (30° cone) on added waypoints"
            onClick={() => setUpright((v) => !v)}
          >
            ⊙ upright
          </button>
        </div>

        <div className="motion-list">
          {segments.map((seg, i) => (
            <div key={i} className="motion-row">
              <span className="motion-seg">
                {i + 1} · {seg.kind === "joint" ? "joint" : "line"}
                {seg.constraints.length > 0 && " ⊙"}
              </span>
              <button
                className="motion-remove"
                title="Remove"
                onClick={() => sendRemoveSegment(motionName, i)}
              >
                ×
              </button>
            </div>
          ))}
          {segments.length === 0 && (
            <div className="empty">no waypoints — pose the robot and add one</div>
          )}
        </div>

        <button
          className="plan-go"
          onClick={onPlan}
          disabled={segments.length === 0 || motionPlanning || !connected}
        >
          Plan motion
        </button>
        {motionError && <div className="plan-error">{motionError}</div>}
      </div>
    </Section>
  );
}
