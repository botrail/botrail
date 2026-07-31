import { Header } from "./components/Header";
import { JointPanel } from "./components/JointPanel";
import { MotionPanel } from "./components/MotionPanel";
import { ObstaclePanel } from "./components/ObstaclePanel";
import { PlanPanel } from "./components/PlanPanel";
import { RobotPanel } from "./components/RobotPanel";
import { SceneTreePanel } from "./components/SceneTreePanel";
import { SequencePanel } from "./components/SequencePanel";
import { TcpPanel } from "./components/TcpPanel";
import { Viewport } from "./components/Viewport";

export function App() {
  return (
    <div className="app">
      <Header />
      <div className="body">
        <Viewport />
        <aside className="panel">
          <RobotPanel />
          <SceneTreePanel />
          <TcpPanel />
          <PlanPanel />
          <MotionPanel />
          <SequencePanel />
          <ObstaclePanel />
          <JointPanel />
        </aside>
      </div>
    </div>
  );
}
