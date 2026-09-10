import { Header } from "./components/Header";
import { IoNodePanel } from "./components/IoNodePanel";
import { JointPanel } from "./components/JointPanel";
import { MotionPanel } from "./components/MotionPanel";
import { ObstaclePanel } from "./components/ObstaclePanel";
import { RobotPanel } from "./components/RobotPanel";
import { SceneTreePanel } from "./components/SceneTreePanel";
import { SensorDevicePanel } from "./components/SensorDevicePanel";
import { SequencePanel } from "./components/SequencePanel";
import { TcpPanel } from "./components/TcpPanel";
import { Viewport } from "./components/Viewport";
import { useStudioStore, type SidebarTab } from "./store";

const TABS: { id: SidebarTab; label: string }[] = [
  { id: "layout", label: "Layout" },
  { id: "motion", label: "Motion" },
  { id: "sequence", label: "Sequence" },
];

/**
 * The sidebar is split by when things are used, not what they are:
 * Layout builds the world (robot placement, obstacles, sensors), Motion
 * poses and teaches the selected robot (TCP + joints + waypoints — used
 * together), Sequence programs and runs the cell. Playback is not a tab:
 * the timeline dock under the viewport is the one transport bar.
 */
export function App() {
  const activeTab = useStudioStore((s) => s.activeTab);
  const setActiveTab = useStudioStore((s) => s.setActiveTab);
  return (
    <div className="app">
      <Header />
      <div className="body">
        <Viewport />
        <aside className="panel">
          <nav className="tab-bar">
            {TABS.map((t) => (
              <button
                key={t.id}
                className={`tab${activeTab === t.id ? " active" : ""}`}
                onClick={() => setActiveTab(t.id)}
              >
                {t.label}
              </button>
            ))}
          </nav>
          {activeTab === "layout" && (
            <>
              <RobotPanel />
              <SceneTreePanel />
              <ObstaclePanel />
              <SensorDevicePanel />
              <IoNodePanel />
            </>
          )}
          {activeTab === "motion" && (
            <>
              <TcpPanel />
              <JointPanel />
              <MotionPanel />
            </>
          )}
          {activeTab === "sequence" && <SequencePanel />}
        </aside>
      </div>
    </div>
  );
}
