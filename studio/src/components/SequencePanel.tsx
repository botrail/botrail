import { useEffect, useState } from "react";

import type { ActionMsg, ConditionMsg, SequenceMsg, StepMsg } from "../protocol";
import { robotByName, useStudioStore } from "../store";
import {
  sendSimulateSequence,
  sendSimulateSequences,
  sendUpsertSequence,
} from "../ws";
import { Section } from "./Section";

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
    case "rising":
      return `↑${condition.name}`;
    case "falling":
      return `↓${condition.name}`;
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
  // Which world the next simulate runs in: `baseline` (the scene as it
  // stands) or a named initial-state delta. Falls back when the chosen
  // scenario is removed.
  const scenarios = useStudioStore((s) => s.scenarios);
  const [scenario, setScenario] = useState("baseline");
  useEffect(() => {
    if (scenario !== "baseline" && !scenarios.some((s) => s.name === scenario)) {
      setScenario("baseline");
    }
  }, [scenarios, scenario]);
  const runScenario = scenario === "baseline" ? undefined : scenario;
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

  // With several programs authored (one per station, PLC style), the
  // cell runs them together; the checkboxes carve out a subset when
  // debugging one station in isolation.
  const multi = sequences.length > 1;
  const included = sequences.filter((s) => !excluded.has(s.name));
  const canSimulate = multi ? included.length > 0 : sequence.steps.length > 0;
  const onSimulate = () => {
    beginSequenceSim();
    if (multi) {
      sendSimulateSequences(
        included.map((s) => s.name),
        runScenario,
      );
    } else {
      sendSimulateSequence(sequence.name, runScenario);
    }
  };

  return (
    <>
      <Section
        id="sequence"
        title="Sequence"
        badge={
          sequence.steps.length > 0 && (
            <span className="badge muted">{sequence.steps.length} steps</span>
          )
        }
      >
        <div className="plan-controls">
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
          {motions.length === 0 && (
            <div className="hint">no motions yet — teach one in the Motion tab</div>
          )}
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
              <div key={i}>
                <div className="motion-row">
                  <span className="motion-kind">
                    {i + 1} · {step.select?.length ? "◇ " : ""}
                    {step.name}
                    {step.actions.map((a, k) => (
                      <span key={k} className="seq-chip">
                        {actionLabel(a)}
                      </span>
                    ))}
                  </span>
                  {step.select?.length ? (
                    <span className="seq-cond">{step.select.length} arms</span>
                  ) : step.transition.type === "elapsed" ? (
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
                {/* Branch arms, read-only (Python-authored): the guard and
                    each arm's steps, indented under the branching step. */}
                {(step.select ?? []).map((arm, j) => (
                  <div key={j} className="seq-arm">
                    <div className="motion-row">
                      <span className="motion-kind seq-cond">
                        ├ when {conditionLabel(arm.condition)}
                        {arm.steps.length === 0 && " → skip"}
                      </span>
                    </div>
                    {arm.steps.map((armStep, k) => (
                      <div key={k} className="motion-row seq-arm-step">
                        <span className="motion-kind">
                          {armStep.select?.length ? "◇ " : ""}
                          {armStep.name}
                          {armStep.actions.map((a, m) => (
                            <span key={m} className="seq-chip">
                              {actionLabel(a)}
                            </span>
                          ))}
                        </span>
                        <span className="seq-cond">
                          {armStep.select?.length
                            ? `${armStep.select.length} arms`
                            : conditionLabel(armStep.transition)}
                        </span>
                      </div>
                    ))}
                  </div>
                ))}
              </div>
            ))}
            {sequence.steps.length === 0 && (
              <div className="empty">no steps — add motions, waits, grasps</div>
            )}
          </div>
        </div>
      </Section>

      {/* Everything about the next run in one place: which programs roll
          together, the world (a Python-authored scenario delta or the
          baseline scene), and the one Simulate button. */}
      <Section
        id="run"
        title="Run"
        badge={
          <>
            {simulating && <span className="badge muted">simulating…</span>}
            {!simulating && timeline && (
              <span className="badge ok">
                cycle {timeline.duration.toFixed(2)}s
              </span>
            )}
          </>
        }
      >
        <div className="plan-controls">
          {multi && (
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
            </div>
          )}
          {scenarios.length > 0 && (
            <select
              value={scenario}
              onChange={(e) => setScenario(e.target.value)}
              title="the world the next simulate runs in (scenarios are authored from Python)"
            >
              <option value="baseline">baseline</option>
              {scenarios.map((s) => (
                <option key={s.name} value={s.name}>
                  ⧉ {s.name}
                </option>
              ))}
            </select>
          )}
          <button
            className="plan-go"
            onClick={onSimulate}
            disabled={!canSimulate || simulating || !connected}
            title={
              multi
                ? "roll the checked programs concurrently over one world"
                : undefined
            }
          >
            {multi ? `Simulate (${included.length} programs)` : "Simulate"}
          </button>
          {error && <div className="plan-error">{error}</div>}
        </div>
      </Section>
    </>
  );
}
