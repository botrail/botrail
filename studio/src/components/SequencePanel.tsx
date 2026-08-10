import { useEffect, useState } from "react";

import type { ActionMsg, ConditionMsg, SequenceMsg, StepMsg } from "../protocol";
import { robotByName, useStudioStore } from "../store";
import {
  sendSimulateSequence,
  sendSimulateSequences,
  sendUpsertSequence,
} from "../ws";

const SEQUENCE_NAME = "main";

/** Compact label for an action chip. */
function actionLabel(action: ActionMsg): string {
  const short = (name: string) => name.split("/").filter(Boolean).pop() ?? name;
  switch (action.type) {
    case "start_motion":
      return `▶ ${action.motion}`;
    case "start_ramp":
      return `ramp ${action.targets.length}j`;
    case "attach":
      return `⊕ ${short(action.object)}`;
    case "detach":
      return `⊖ ${short(action.object)}`;
    case "track":
      return `⇉ ${short(action.object)}`;
    case "untrack":
      return "⇥ untrack";
    case "set":
      return `${action.signal}=${action.value ? "1" : "0"}`;
    case "device": {
      const cmd = action.command;
      const verb =
        cmd.type === "set_speed"
          ? `speed ${cmd.speed}`
          : cmd.type === "move_to"
            ? `→${cmd.position}`
            : cmd.type === "advance"
              ? `⊳ ${cmd.distance}m`
              : cmd.type;
      return `⚙ ${action.device} ${verb}`;
    }
  }
}

function conditionLabel(condition: ConditionMsg): string {
  switch (condition.type) {
    case "immediately":
      return "→";
    case "done":
      return "done";
    case "robot_done":
      return `${condition.robot} done`;
    case "elapsed":
      return `${condition.seconds.toFixed(2)}s`;
    case "signal":
      return `${condition.name}=${condition.value ? "1" : "0"}`;
    case "all":
      return condition.conditions.map(conditionLabel).join(" & ");
    case "any":
      return condition.conditions.map(conditionLabel).join(" | ");
    case "device_done":
      return `${condition.device} done`;
  }
}

/** PLC-style step-sequence editor over the scene's `main` sequence. */
export function SequencePanel() {
  const sequences = useStudioStore((s) => s.sequences);
  const motions = useStudioStore((s) => s.motions);
  const selection = useStudioStore((s) => s.selection);
  // Grasp steps attach with the selected robot at its TCP link.
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const tcpLink = useStudioStore(
    (s) => robotByName(s.robots, s.selectedRobot)?.tcpLink ?? null,
  );
  const simulating = useStudioStore((s) => s.sequenceSimulating);
  const error = useStudioStore((s) => s.sequenceError);
  const timeline = useStudioStore((s) => s.timeline);
  const beginSequenceSim = useStudioStore((s) => s.beginSequenceSim);
  const connected = useStudioStore((s) => s.connection === "connected");

  const sequence: SequenceMsg =
    sequences.find((s) => s.name === SEQUENCE_NAME) ??
    sequences[0] ?? { name: SEQUENCE_NAME, steps: [] };
  const [motionChoice, setMotionChoice] = useState("");
  // Multi-program run set: which sequences the next co-simulation rolls
  // together (all of them by default — that is the cell). Names absent
  // from the map count as included, so freshly authored programs join
  // without a click.
  const [excluded, setExcluded] = useState<Set<string>>(new Set());
  useEffect(() => {
    if (!motions.some((m) => m.name === motionChoice)) {
      setMotionChoice(motions[0]?.name ?? "");
    }
  }, [motions, motionChoice]);

  const commit = (steps: StepMsg[]) =>
    sendUpsertSequence({ name: sequence.name, steps });
  const append = (step: StepMsg) => commit([...sequence.steps, step]);

  const selectedObstacle = selection.type === "obstacle" ? selection.name : null;
  const short = (name: string) => name.split("/").filter(Boolean).pop() ?? name;

  const addMotion = () => {
    if (!motionChoice) return;
    append({
      name: motionChoice,
      actions: [{ type: "start_motion", motion: motionChoice }],
      transition: { type: "done" },
    });
  };
  const addWait = () =>
    append({
      name: "wait",
      actions: [],
      transition: { type: "elapsed", seconds: 1.0 },
    });
  const addGrasp = () => {
    if (!selectedObstacle) return;
    append({
      name: `grasp ${short(selectedObstacle)}`,
      actions: [
        {
          type: "attach",
          robot: selectedRobot,
          object: selectedObstacle,
          link: tcpLink,
          touch_links: null,
        },
      ],
      transition: { type: "immediately" },
    });
  };
  const addRelease = () => {
    if (!selectedObstacle) return;
    append({
      name: `release ${short(selectedObstacle)}`,
      actions: [{ type: "detach", object: selectedObstacle }],
      transition: { type: "immediately" },
    });
  };

  const setWaitSeconds = (index: number, seconds: number) => {
    const steps = sequence.steps.map((s, i) =>
      i === index ? { ...s, transition: { type: "elapsed" as const, seconds } } : s,
    );
    commit(steps);
  };

  const onSimulate = () => {
    beginSequenceSim();
    sendSimulateSequence(sequence.name);
  };

  const included = sequences.filter((s) => !excluded.has(s.name));
  const onSimulateTogether = () => {
    if (included.length === 0) return;
    beginSequenceSim();
    sendSimulateSequences(included.map((s) => s.name));
  };

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Sequence</h2>
        {simulating && <span className="badge muted">simulating…</span>}
        {!simulating && timeline && (
          <span className="badge ok">cycle {timeline.duration.toFixed(2)}s</span>
        )}
      </div>
      <div className="plan-controls">
        {/* With several programs authored (one per station, PLC style),
            the cell runs them together — the checkboxes carve out a
            subset when debugging one station in isolation. */}
        {sequences.length > 1 && (
          <div className="seq-programs">
            {sequences.map((s) => (
              <label key={s.name} className="seq-program">
                <input
                  type="checkbox"
                  checked={!excluded.has(s.name)}
                  onChange={(e) => {
                    const next = new Set(excluded);
                    if (e.target.checked) {
                      next.delete(s.name);
                    } else {
                      next.add(s.name);
                    }
                    setExcluded(next);
                  }}
                />
                {s.name}
                <span className="seq-cond"> · {s.steps.length} steps</span>
              </label>
            ))}
            <button
              className="plan-go"
              onClick={onSimulateTogether}
              disabled={included.length === 0 || simulating || !connected}
              title="roll the checked programs concurrently over one world"
            >
              Simulate programs ({included.length})
            </button>
          </div>
        )}
        <div className="seg">
          <button onClick={addMotion} disabled={!motionChoice} title="add a motion step">
            + Motion
          </button>
          <select
            value={motionChoice}
            onChange={(e) => setMotionChoice(e.target.value)}
            disabled={motions.length === 0}
          >
            {motions.map((m) => (
              <option key={m.name} value={m.name}>
                {m.name}
              </option>
            ))}
          </select>
          <button onClick={addWait} title="add a 1s timer step">
            + Wait
          </button>
        </div>
        <div className="seg">
          <button
            onClick={addGrasp}
            disabled={!selectedObstacle}
            title="grasp the selected obstacle with the TCP link"
          >
            + Grasp
          </button>
          <button
            onClick={addRelease}
            disabled={!selectedObstacle}
            title="release the selected obstacle"
          >
            + Release
          </button>
        </div>

        <div className="motion-list">
          {sequence.steps.map((step, i) => (
            <div key={i} className="motion-row">
              <span className="motion-kind">
                {i + 1} · {step.name}
                {step.actions.map((a, k) => (
                  <span key={k} className="seq-chip">
                    {actionLabel(a)}
                  </span>
                ))}
              </span>
              {step.transition.type === "elapsed" ? (
                <input
                  className="seq-wait"
                  type="number"
                  min={0}
                  step={0.1}
                  value={step.transition.seconds}
                  onChange={(e) => {
                    const v = parseFloat(e.target.value);
                    if (Number.isFinite(v) && v >= 0) setWaitSeconds(i, v);
                  }}
                />
              ) : (
                <span className="seq-cond">{conditionLabel(step.transition)}</span>
              )}
              <button
                className="obstacle-remove"
                title="Remove step"
                onClick={() =>
                  commit(sequence.steps.filter((_, k) => k !== i))
                }
              >
                ×
              </button>
            </div>
          ))}
          {sequence.steps.length === 0 && (
            <div className="empty">no steps — add motions, waits, grasps</div>
          )}
        </div>

        <div className="seg">
          <button
            className="plan-go"
            onClick={onSimulate}
            disabled={sequence.steps.length === 0 || simulating || !connected}
          >
            Simulate
          </button>
        </div>
        {error && <div className="plan-error">{error}</div>}
      </div>
    </section>
  );
}
