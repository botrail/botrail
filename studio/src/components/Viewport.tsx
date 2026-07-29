import { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";

import { useStudioStore } from "../store";
import { GhostRobot } from "./GhostRobot";
import { ObstacleView } from "./ObstacleView";
import { PlaybackDriver } from "./PlaybackDriver";
import { RobotBaseGizmo } from "./RobotBaseGizmo";
import { SceneView } from "./SceneView";
import { TcpGizmo } from "./TcpGizmo";
import { UsdRobotView } from "./UsdRobotView";

export function Viewport() {
  const connected = useStudioStore((s) => s.connection === "connected");

  return (
    <div className="viewport">
      <Canvas
        camera={{
          position: [1.6, -1.6, 1.2],
          up: [0, 0, 1],
          fov: 45,
          near: 0.01,
          far: 100,
        }}
        onPointerMissed={() => useStudioStore.getState().selectTcp()}
      >
        <color attach="background" args={["#15171c"]} />
        <ambientLight intensity={0.6} />
        <directionalLight position={[3, 3, 6]} intensity={1.1} />
        <hemisphereLight args={["#8899aa", "#20242c", 0.4]} />

        {/* drei's Grid lies in the XZ plane; rotate it onto XY (Z-up floor). */}
        <Grid
          rotation={[Math.PI / 2, 0, 0]}
          infiniteGrid
          cellSize={0.1}
          cellThickness={0.6}
          cellColor="#2a2f3a"
          sectionSize={1}
          sectionThickness={1}
          sectionColor="#3c4557"
          fadeDistance={16}
          fadeStrength={1}
        />
        <axesHelper args={[0.3]} />

        <OrbitControls makeDefault target={[0, 0, 0.2]} />

        <Suspense fallback={null}>
          <SceneView />
          <UsdRobotView />
          <GhostRobot />
          <ObstacleView />
          <TcpGizmo />
          <RobotBaseGizmo />
          <PlaybackDriver />
        </Suspense>
      </Canvas>

      {!connected && <div className="overlay">connecting…</div>}
    </div>
  );
}
