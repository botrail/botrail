import { useState } from "react";

import type { ConstraintMsg, MotionMsg, SegmentKindMsg } from "../protocol";
import { robotArms, robotByName, useStudioStore } from "../store";
import { sendAddSegment, sendPlanMotion, sendRemoveSegment } from "../ws";
import { Section } from "./Section";

// "Upright" keeps the TCP's local +Z within a 30° cone of world +Z.
const UPRIGHT_CONE: ConstraintMsg = {
  type: "orientation_cone",
  axis_local: [0, 0, 1],
  axis_world: [0, 0, 1],
  angle: (Math.PI / 180) * 30,
};

/** The conventional first motion name: `main` for the first robot,
 * `main_<robot>` for the rest, with `_<arm>` appended on a dual-arm robot
 * (`main_left`). */
function conventionalName(
  robotName: string,
  firstRobot: string | undefined,
  arm: string | null,
): string {
  const base = robotName === firstRobot ? "main" : `main_${robotName}`;
  return arm === null ? base : `${base}_${arm}`;
}

/** The conventional name when free; when taken, `motion_2`, `motion_3`,
 * … — whatever is free scene-wide. */
function freshMotionName(
  motions: MotionMsg[],
  robotName: string,
  firstRobot: string | undefined,
  arm: string | null,
): string {
  const taken = new Set(motions.map((m) => m.name));
  const base = conventionalName(robotName, firstRobot, arm);
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
  // On a dual-arm robot a motion drives one arm (or, unnamed, every
  // joint); new ones drive the selected arm.
  const dualArm = robotArms(robot.desc).length > 0;
  const arm = dualArm ? robot.selectedGroup : null;
  const armOf = (m: MotionMsg) => m.group ?? null;
  // What a motion row says about its scope: the owner when several
  // robots share the scene, the arm on a dual-arm robot.
  const scopeOf = (owner: string | undefined, motionArm: string | null) => {
    const parts: string[] = [];
    if (multiRobot) parts.push(owner ?? "?");
    if (dualArm && owner === robotName) parts.push(motionArm ?? "all joints");
    return parts.length > 0 ? ` · ${parts.join(" · ")}` : null;
  };

  // The edit target: the picked motion, else the selected robot's (arm's)
  // first motion, else the conventional fresh name (created on the first
  // waypoint — same implicit-create the server does).
  const owned = motions.filter(
    (m) => ownerOf(m) === robotName && (!dualArm || armOf(m) === arm),
  );
  const fallbackName =
    owned[0]?.name ?? conventionalName(robotName, firstRobot, arm);
  const motionName = selectedMotion ?? fallbackName;
  const motion = motions.find((m) => m.name === motionName) ?? null;
  const segments = motion?.segments ?? [];

  const addSegment = (kind: SegmentKindMsg) => {
    sendAddSegment(
      motionName,
      robotName,
      {
        kind,
        goal_positions: robot.jointPositions.slice(),
        constraints: upright ? [UPRIGHT_CONE] : [],
      },
      arm,
    );
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
                {scopeOf(ownerOf(m), armOf(m)) && (
                  <span className="seq-cond">
                    {scopeOf(ownerOf(m), armOf(m))}
                  </span>
                )}
              </span>
              <span className="seq-cond">{m.segments.length} wp</span>
            </div>
          ))}
          {!motion && (
            <div className="obstacle-row selected">
              <span className="obstacle-name">
                {motionName}
                {scopeOf(robotName, arm) && (
                  <span className="seq-cond">{scopeOf(robotName, arm)}</span>
                )}
              </span>
              <span className="seq-cond">new</span>
            </div>
          )}
        </div>
        <button
          className="motion-new"
          onClick={() =>
            selectMotion(freshMotionName(motions, robotName, firstRobot, arm))
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
