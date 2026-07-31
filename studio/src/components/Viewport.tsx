import { Suspense } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";

import { isWasmMode } from "../backend";
import { useStudioStore } from "../store";
import { dropUsdScene } from "../ws";
import { GhostRobot } from "./GhostRobot";
import { ObstacleView } from "./ObstacleView";
import { PlaybackDriver } from "./PlaybackDriver";
import { RobotBaseGizmo } from "./RobotBaseGizmo";
import { SceneView } from "./SceneView";
import { SensorView } from "./SensorView";
import { TcpGizmo } from "./TcpGizmo";
import { TimelineDock } from "./TimelineDock";
import { UsdRobotView } from "./UsdRobotView";
import { WasmStageView } from "./WasmStageView";

export function Viewport() {
  const connected = useStudioStore((s) => s.connection === "connected");
  const selection = useStudioStore((s) => s.selection);
  const tcpLink = useStudioStore((s) => s.tcpLink);

  const focusLabel =
    selection.type === "obstacle"
      ? `obstacle · ${selection.name}`
      : selection.type === "robot"
        ? "robot base"
        : `TCP · ${tcpLink ?? "—"}`;

  // Wasm mode: drop a USD file to import it into the in-browser session
  // (collision + frames) and render the stage client-side.
  const onDrop = async (e: React.DragEvent) => {
    if (!isWasmMode()) return;
    e.preventDefault();
    const file = e.dataTransfer.files[0];
    if (!file || !/[.]usd[acz]?$/i.test(file.name)) return;
    const data = await file.arrayBuffer();
    if (await dropUsdScene(new Uint8Array(data), file.name)) {
      useStudioStore.getState().setDroppedStage({ data, name: file.name });
    }
  };

  return (
    <div
      className="viewport"
      onDragOver={(e) => {
        if (isWasmMode()) e.preventDefault();
      }}
      onDrop={onDrop}
    >
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
          <WasmStageView />
          <GhostRobot />
          <ObstacleView />
          <SensorView />
          <TcpGizmo />
          <RobotBaseGizmo />
          <PlaybackDriver />
        </Suspense>
      </Canvas>

      {connected && <div className="focus-chip">{focusLabel}</div>}
      <TimelineDock />
      {!connected && <div className="overlay">connecting…</div>}
    </div>
  );
}
