import { useStudioStore } from "../store";

/** TCP gizmo controls: target link, gizmo mode, and IK feedback. */
export function TcpPanel() {
  const sceneDesc = useStudioStore((s) => s.sceneDesc);
  const tcpLink = useStudioStore((s) => s.tcpLink);
  const setTcpLink = useStudioStore((s) => s.setTcpLink);
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const setGizmoMode = useStudioStore((s) => s.setGizmoMode);
  const ikStatus = useStudioStore((s) => s.ikStatus);

  if (!sceneDesc) return null;

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>TCP</h2>
        {ikStatus &&
          (ikStatus.converged ? (
            <span className="badge ok">reachable</span>
          ) : (
            <span className="badge bad">
              unreachable · {(ikStatus.pos_error * 1000).toFixed(0)}mm
            </span>
          ))}
      </div>
      <div className="tcp-controls">
        <label className="field">
          <span className="field-label">link</span>
          <select
            value={tcpLink ?? ""}
            onChange={(e) => setTcpLink(e.target.value)}
          >
            {sceneDesc.links.map((l) => (
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
    </section>
  );
}
