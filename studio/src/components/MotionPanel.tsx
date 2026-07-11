import { useRef, useState } from "react";

import { backendSupportsHttp } from "../backend";
import type { ConstraintMsg, SegmentKindMsg } from "../protocol";
import { useStudioStore } from "../store";
import {
  sendAddSegment,
  sendPlanMotion,
  sendRemoveSegment,
} from "../ws";

// "Upright" keeps the TCP's local +Z within a 30° cone of world +Z.
const UPRIGHT_CONE: ConstraintMsg = {
  type: "orientation_cone",
  axis_local: [0, 0, 1],
  axis_world: [0, 0, 1],
  angle: (Math.PI / 180) * 30,
};

function downloadBlob(blob: Blob, filename: string): void {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

/** Waypoint editing, motion planning, and project save/load/export. */
export function MotionPanel() {
  const sceneDesc = useStudioStore((s) => s.sceneDesc);
  const connected = useStudioStore((s) => s.connection === "connected");
  const jointPositions = useStudioStore((s) => s.jointPositions);
  const motions = useStudioStore((s) => s.motions);
  const motionPlanning = useStudioStore((s) => s.motionPlanning);
  const motionError = useStudioStore((s) => s.motionError);
  const motionStats = useStudioStore((s) => s.motionStats);
  const segmentEnds = useStudioStore((s) => s.segmentEnds);
  const trajectory = useStudioStore((s) => s.trajectory);
  const beginMotionPlanning = useStudioStore((s) => s.beginMotionPlanning);

  const [upright, setUpright] = useState(false);
  // Errors from the HTTP save/load/export round-trips (kept out of the store).
  const [ioError, setIoError] = useState<string | null>(null);
  const fileRef = useRef<HTMLInputElement>(null);

  if (!sceneDesc) return null;

  // A single motion "main" is edited; adopt the server's name if it exists.
  const motion = motions[0] ?? null;
  const motionName = motion?.name ?? "main";
  const segments = motion?.segments ?? [];

  const addSegment = (kind: SegmentKindMsg) => {
    sendAddSegment(motionName, {
      kind,
      goal_positions: jointPositions.slice(),
      constraints: upright ? [UPRIGHT_CONE] : [],
    });
  };

  const onPlan = () => {
    if (segments.length === 0) return;
    beginMotionPlanning();
    sendPlanMotion(motionName);
  };

  const onSave = async () => {
    try {
      const res = await fetch("/api/project");
      if (!res.ok) throw new Error(await res.text());
      downloadBlob(await res.blob(), "project.botrail");
      setIoError(null);
    } catch (e) {
      setIoError(`save failed: ${String(e)}`);
    }
  };

  const onExport = async () => {
    try {
      const res = await fetch("/api/export.py");
      if (!res.ok) throw new Error(await res.text());
      downloadBlob(await res.blob(), "scene.py");
      setIoError(null);
    } catch (e) {
      setIoError(`export failed: ${String(e)}`);
    }
  };

  const onLoadFile = async (file: File) => {
    try {
      const text = await file.text();
      const res = await fetch("/api/project", { method: "POST", body: text });
      if (!res.ok) {
        setIoError((await res.text()) || "load failed");
        return;
      }
      // Success re-broadcasts obstacles/motions/state over the websocket.
      setIoError(null);
    } catch (e) {
      setIoError(`load failed: ${String(e)}`);
    }
  };

  return (
    <section className="panel-section">
      <div className="panel-head">
        <h2>Motion</h2>
        {motionPlanning && <span className="badge muted">planning…</span>}
        {!motionPlanning && motionStats && trajectory && (
          <span className="badge ok">
            {trajectory.duration.toFixed(2)}s · {segmentEnds.length} seg ·{" "}
            {motionStats.planningTimeMs.toFixed(0)}ms
          </span>
        )}
      </div>
      <div className="motion-controls">
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

        {backendSupportsHttp() && (
          <>
            <div className="seg motion-io">
              <button onClick={onSave}>Save</button>
              <button onClick={() => fileRef.current?.click()}>Load</button>
              <button onClick={onExport}>Export .py</button>
            </div>
            {ioError && <div className="plan-error">{ioError}</div>}
            <input
              ref={fileRef}
              type="file"
              accept=".botrail,application/json"
              style={{ display: "none" }}
              onChange={(e) => {
                const file = e.target.files?.[0];
                if (file) onLoadFile(file);
                e.target.value = "";
              }}
            />
          </>
        )}
      </div>
    </section>
  );
}
