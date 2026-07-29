import { useStudioStore } from "../store";
import { sendRobotBasePose } from "../ws";

/**
 * Robot base placement: the drag gizmo plus snap-to-frame (named mount
 * points imported from scene files).
 */
export function RobotPanel() {
  const sceneDesc = useStudioStore((s) => s.sceneDesc);
  const basePose = useStudioStore((s) => s.basePose);
  const frames = useStudioStore((s) => s.frames);
  const selection = useStudioStore((s) => s.selection);
  const selectRobot = useStudioStore((s) => s.selectRobot);
  const selectTcp = useStudioStore((s) => s.selectTcp);

  if (!sceneDesc || !basePose) return null;
  const placing = selection.type === "robot";
  const [x, y, z] = basePose.position;

  const onSnap = (name: string) => {
    const frame = frames.find((f) => f.name === name);
    if (frame) sendRobotBasePose(frame.pose);
  };

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Robot</h2>
        <span className="badge">{sceneDesc.robot_name}</span>
      </div>
      <div className="tcp-controls">
        <div className="seg">
          <button
            className={placing ? "active" : ""}
            onClick={() => (placing ? selectTcp() : selectRobot())}
          >
            {placing ? "Done placing" : "Place base"}
          </button>
        </div>
        <span className="field-label">
          base ({x.toFixed(2)}, {y.toFixed(2)}, {z.toFixed(2)})
        </span>
      </div>
      {frames.length > 0 && (
        <label className="field">
          <span className="field-label">place at frame</span>
          <select value="" onChange={(e) => onSnap(e.target.value)}>
            <option value="" disabled>
              choose a frame…
            </option>
            {frames.map((f) => (
              <option key={f.name} value={f.name}>
                {f.name}
              </option>
            ))}
          </select>
        </label>
      )}
    </section>
  );
}
