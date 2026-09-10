import { useRef, useState } from "react";

import { backendSupportsHttp } from "../backend";
import { useStudioStore } from "../store";

export function downloadBlob(blob: Blob, filename: string): void {
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

export function Header() {
  const robots = useStudioStore((s) => s.robots);
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const setSelectedRobot = useStudioStore((s) => s.setSelectedRobot);
  const connection = useStudioStore((s) => s.connection);
  const connected = connection === "connected";

  // Errors from the HTTP save/load/export round-trips (kept out of the store).
  const [ioError, setIoError] = useState<string | null>(null);
  const fileRef = useRef<HTMLInputElement>(null);

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
    <header className="header">
      <span className="title">botrail studio</span>
      {robots.length > 1 && selectedRobot !== null ? (
        <select
          className="robot-name"
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
        robots[0] && <span className="robot-name">{robots[0].desc.name}</span>
      )}
      <span className="spacer" />
      {ioError && (
        <span className="header-error" title={ioError}>
          {ioError}
        </span>
      )}
      {backendSupportsHttp() && (
        <span className="header-io">
          <button onClick={onSave} title="download the project as a .botrail file">
            Save
          </button>
          <button
            onClick={() => fileRef.current?.click()}
            title="load a .botrail project file"
          >
            Load
          </button>
          <button onClick={onExport} title="download the scene as a Python script">
            Export .py
          </button>
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
        </span>
      )}
      <span className={`status ${connected ? "ok" : "bad"}`}>
        <span className="dot" />
        {connection}
      </span>
    </header>
  );
}
