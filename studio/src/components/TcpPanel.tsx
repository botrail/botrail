import { robotArms, robotByName, useStudioStore } from "../store";
import { Section } from "./Section";

/** TCP gizmo controls for the selected robot: arm (when it has several),
 * link, mode, IK feedback. */
export function TcpPanel() {
  const robot = useStudioStore((s) => robotByName(s.robots, s.selectedRobot));
  const setTcpLink = useStudioStore((s) => s.setTcpLink);
  const setSelectedGroup = useStudioStore((s) => s.setSelectedGroup);
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const setGizmoMode = useStudioStore((s) => s.setGizmoMode);

  if (!robot) return null;
  const ikStatus = robot.ikStatus;
  const arms = robotArms(robot.desc);
  const armTag = ikStatus?.group ? ` · ${ikStatus.group}` : "";

  return (
    <Section
      id="tcp"
      title="TCP"
      badge={
        ikStatus &&
        (ikStatus.converged ? (
          <span className="badge ok">reachable{armTag}</span>
        ) : (
          <span className="badge bad">
            unreachable · {(ikStatus.pos_error * 1000).toFixed(0)}mm{armTag}
          </span>
        ))
      }
    >
      <div className="tcp-controls">
        {arms.length > 0 && (
          <div className="seg" title="the arm the gizmo drives">
            {arms.map((g) => (
              <button
                key={g.name}
                className={g.name === robot.selectedGroup ? "active" : ""}
                onClick={() => setSelectedGroup(robot.desc.name, g.name)}
              >
                {g.name}
              </button>
            ))}
          </div>
        )}
        <label className="field">
          <span className="field-label">link</span>
          <select
            value={robot.tcpLink ?? ""}
            onChange={(e) => setTcpLink(robot.desc.name, e.target.value)}
          >
            {robot.desc.links.map((l) => (
              <option key={l.name} value={l.name}>
                {l.name}
              </option>
            ))}
          </select>
        </label>
        <div className="seg">
          <button
            className={gizmoMode === "translate" ? "active" : ""}
            onClick={() => setGizmoMode("translate")}
          >
            Move
          </button>
          <button
            className={gizmoMode === "rotate" ? "active" : ""}
            onClick={() => setGizmoMode("rotate")}
          >
            Rotate
          </button>
        </div>
      </div>
    </Section>
  );
}
