import { useStudioStore } from "../store";

export function Header() {
  const robots = useStudioStore((s) => s.robots);
  const selectedRobot = useStudioStore((s) => s.selectedRobot);
  const setSelectedRobot = useStudioStore((s) => s.setSelectedRobot);
  const connection = useStudioStore((s) => s.connection);
  const connected = connection === "connected";

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
      <span className={`status ${connected ? "ok" : "bad"}`}>
        <span className="dot" />
        {connection}
      </span>
    </header>
  );
}
