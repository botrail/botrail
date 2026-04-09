import { useStudioStore } from "../store";

export function Header() {
  const robotName = useStudioStore((s) => s.sceneDesc?.robot_name);
  const connection = useStudioStore((s) => s.connection);
  const connected = connection === "connected";

  return (
    <header className="header">
      <span className="title">botrail studio</span>
      {robotName && <span className="robot-name">{robotName}</span>}
      <span className="spacer" />
      <span className={`status ${connected ? "ok" : "bad"}`}>
        <span className="dot" />
        {connection}
      </span>
    </header>
  );
}
