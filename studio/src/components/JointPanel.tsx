import { useMemo } from "react";

import type { JointMsg } from "../protocol";
import { useStudioStore } from "../store";
import { sendJointPositions } from "../ws";

const CONTINUOUS_RANGE: [number, number] = [-Math.PI, Math.PI];

export function JointPanel() {
  const sceneDesc = useStudioStore((s) => s.sceneDesc);
  const jointPositions = useStudioStore((s) => s.jointPositions);
  const setJointPosition = useStudioStore((s) => s.setJointPosition);
  const resetJoints = useStudioStore((s) => s.resetJoints);

  // Actuated joints, in q_index order.
  const dofJoints = useMemo<JointMsg[]>(() => {
    if (!sceneDesc) return [];
    return sceneDesc.joints
      .filter((j) => j.q_index !== null)
      .sort((a, b) => (a.q_index as number) - (b.q_index as number));
  }, [sceneDesc]);

  const commit = () => {
    sendJointPositions(useStudioStore.getState().jointPositions.slice());
  };

  const onSlider = (qIndex: number, value: number) => {
    setJointPosition(qIndex, value);
    commit();
  };

  const onReset = () => {
    resetJoints();
    commit();
  };

  return (
    <section className="panel-section joints-section">
      <div className="panel-head">
        <h2>Joints</h2>
        <button onClick={onReset} disabled={dofJoints.length === 0}>
          Reset
        </button>
      </div>
      <div className="joints">
        {dofJoints.map((joint) => {
          const qIndex = joint.q_index as number;
          const [lo, hi] = joint.limits ?? CONTINUOUS_RANGE;
          const value = jointPositions[qIndex] ?? 0;
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
                onChange={(e) => onSlider(qIndex, parseFloat(e.target.value))}
              />
            </div>
          );
        })}
        {dofJoints.length === 0 && (
          <div className="empty">No actuated joints</div>
        )}
      </div>
    </section>
  );
}
