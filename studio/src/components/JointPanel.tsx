import { Fragment, useMemo } from "react";

import type { JointMsg } from "../protocol";
import { robotArms, robotByName, useStudioStore } from "../store";
import { sendJointPositions } from "../ws";
import { Section } from "./Section";

const CONTINUOUS_RANGE: [number, number] = [-Math.PI, Math.PI];

/** Joint sliders for the panel-selected robot. */
export function JointPanel() {
  const robot = useStudioStore((s) => robotByName(s.robots, s.selectedRobot));
  const setJointPosition = useStudioStore((s) => s.setJointPosition);
  const resetJoints = useStudioStore((s) => s.resetJoints);

  // Actuated joints, in q_index order.
  const dofJoints = useMemo<JointMsg[]>(() => {
    if (!robot) return [];
    return robot.desc.joints
      .filter((j) => j.q_index !== null)
      .sort((a, b) => (a.q_index as number) - (b.q_index as number));
  }, [robot]);

  // On a dual-arm robot the q order interleaves the arms (breadth-first);
  // list the sliders arm by arm instead, anything outside an arm (a
  // torso) last. A single-arm robot keeps its one flat list.
  const sections = useMemo<{ title: string | null; joints: JointMsg[] }[]>(() => {
    if (!robot) return [];
    const arms = robotArms(robot.desc);
    if (arms.length === 0) return [{ title: null, joints: dofJoints }];
    const claimed = new Set(arms.flatMap((g) => g.joints));
    const out = arms.map((g) => ({
      title: g.name,
      joints: dofJoints.filter((j) => g.joints.includes(j.q_index as number)),
    }));
    const rest = dofJoints.filter((j) => !claimed.has(j.q_index as number));
    if (rest.length > 0) out.push({ title: "other", joints: rest });
    return out;
  }, [robot, dofJoints]);

  // Joints driven by another one: shown as read-out, since commanding
  // them means moving their source.
  const mimicJoints = useMemo<JointMsg[]>(
    () => (robot ? robot.desc.joints.filter((j) => j.mimic !== null) : []),
    [robot],
  );

  if (!robot) return null;
  const name = robot.desc.name;

  const commit = () => {
    const live = robotByName(useStudioStore.getState().robots, name);
    if (live) sendJointPositions(name, live.jointPositions.slice());
  };

  const onSlider = (qIndex: number, value: number) => {
    setJointPosition(name, qIndex, value);
    commit();
  };

  const onReset = () => {
    resetJoints(name);
    commit();
  };

  return (
    <Section
      id="joints"
      title="Joints"
      badge={
        <button onClick={onReset} disabled={dofJoints.length === 0}>
          Reset
        </button>
      }
    >
      <div className="joints">
        {sections.map(({ title, joints }) => (
          <Fragment key={title ?? "all"}>
            {title !== null && <div className="joint-arm">{title}</div>}
            {joints.map((joint) => {
              const qIndex = joint.q_index as number;
              const [lo, hi] = joint.limits ?? CONTINUOUS_RANGE;
              const value = robot.jointPositions[qIndex] ?? 0;
              return (
                <div className="joint" key={joint.name}>
                  <div className="joint-row">
                    <span className="joint-name">{joint.name}</span>
                    <span className="joint-value">{value.toFixed(3)}</span>
                  </div>
                  <input
                    type="range"
                    min={lo}
                    max={hi}
                    step={0.001}
                    value={value}
                    onChange={(e) =>
                      onSlider(qIndex, parseFloat(e.target.value))
                    }
                  />
                </div>
              );
            })}
          </Fragment>
        ))}
        {mimicJoints.map((joint) => {
          const mimic = joint.mimic!;
          const source = robot.desc.joints.find((j) => j.name === mimic.joint);
          const q = source?.q_index ?? null;
          const driver = q === null ? 0 : (robot.jointPositions[q] ?? 0);
          const value = mimic.multiplier * driver + mimic.offset;
          return (
            <div className="joint joint-mimic" key={joint.name}>
              <div className="joint-row">
                <span className="joint-name">{joint.name}</span>
                <span className="joint-value">{value.toFixed(3)}</span>
              </div>
              <div className="joint-note">follows {mimic.joint}</div>
            </div>
          );
        })}
        {dofJoints.length === 0 && mimicJoints.length === 0 && (
          <div className="empty">No actuated joints</div>
        )}
      </div>
    </Section>
  );
}
