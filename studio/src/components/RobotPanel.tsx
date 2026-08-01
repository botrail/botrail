import { robotByName, useStudioStore } from "../store";
import { sendRobotBasePose } from "../ws";

/**
 * Robot selection and base placement: the instance selector (when several
 * robots share the scene), the drag gizmo toggle, and snap-to-frame (named
 * mount points imported from scene files).
 */
export function RobotPanel() {
  const robots = useStudioStore((s) => s.robots);
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const robot = useStudioStore((s) => robotByName(s.robots, s.selectedRobot));
  const frames = useStudioStore((s) => s.frames);
  const selection = useStudioStore((s) => s.selection);
  const selectRobot = useStudioStore((s) => s.selectRobot);
  const selectTcp = useStudioStore((s) => s.selectTcp);
  const setSelectedRobot = useStudioStore((s) => s.setSelectedRobot);

  if (!robot || selectedRobot === null) return null;
  const placing = selection.type === "robot" && selection.robot === selectedRobot;
  const [x, y, z] = robot.basePose.position;

  const onSnap = (name: string) => {
    const frame = frames.find((f) => f.name === name);
    if (frame) sendRobotBasePose(selectedRobot, frame.pose);
  };

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Robot</h2>
        {robots.length > 1 ? (
          <select
            value={selectedRobot}
            onChange={(e) => setSelectedRobot(e.target.value)}
            title="which robot the panels operate on"
          >
            {robots.map((r) => (
              <option key={r.desc.name} value={r.desc.name}>
                {r.desc.name}
              </option>
            ))}
          </select>
        ) : (
          <span className="badge">{robot.desc.name}</span>
        )}
      </div>
      <div className="tcp-controls">
        <div className="seg">
          <button
            className={placing ? "active" : ""}
            onClick={() => (placing ? selectTcp() : selectRobot(selectedRobot))}
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
