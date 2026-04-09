import { Header } from "./components/Header";
import { JointPanel } from "./components/JointPanel";
import { TcpPanel } from "./components/TcpPanel";
import { Viewport } from "./components/Viewport";

export function App() {
  return (
    <div className="app">
      <Header />
      <div className="body">
        <Viewport />
        <aside className="panel">
          <TcpPanel />
          <JointPanel />
        </aside>
      </div>
    </div>
  );
}
