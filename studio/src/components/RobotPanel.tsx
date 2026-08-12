import { robotByName, useStudioStore } from "../store";
import { sendRobotBasePose } from "../ws";
import { Section } from "./Section";

/**
 * Base placement for the selected robot: the drag gizmo toggle and
 * snap-to-frame (named mount points imported from scene files). Which
 * robot is selected lives in the header.
 */
export function RobotPanel() {
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const robot = useStudioStore((s) => robotByName(s.robots, s.selectedRobot));
  const frames = useStudioStore((s) => s.frames);
  const selection = useStudioStore((s) => s.selection);
  const selectRobot = useStudioStore((s) => s.selectRobot);
  const selectTcp = useStudioStore((s) => s.selectTcp);

  if (!robot || selectedRobot === null) return null;
  const placing = selection.type === "robot" && selection.robot === selectedRobot;
  const [x, y, z] = robot.basePose.position;

  const onSnap = (name: string) => {
    const frame = frames.find((f) => f.name === name);
    if (frame) sendRobotBasePose(selectedRobot, frame.pose);
  };

  return (
    <Section
      id="robot"
      title="Robot"
      badge={<span className="badge">{robot.desc.name}</span>}
    >
      <div className="tcp-controls">
        <div className="seg">
          <button
            className={placing ? "active" : ""}
            onClick={() => (placing ? selectTcp() : selectRobot(selectedRobot))}
          >
            {placing ? "Done placing" : "Place base"}
          </button>
        </div>
        <span className="base-readout">
          base ({x.toFixed(2)}, {y.toFixed(2)}, {z.toFixed(2)})
        </span>
        {frames.length > 0 && (
          <label className="field">
            <span className="field-label">frame</span>
            <select value="" onChange={(e) => onSnap(e.target.value)}>
              <option value="" disabled>
                place at frame…
              </option>
              {frames.map((f) => (
                <option key={f.name} value={f.name}>
                  {f.name}
                </option>
              ))}
            </select>
          </label>
        )}
      </div>
    </Section>
  );
}
