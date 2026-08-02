import { Suspense, useEffect } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import { RoomEnvironment } from "three-stdlib";

import { isWasmMode } from "../backend";
import { robotByName, useStudioStore } from "../store";
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

/**
 * Image-based lighting, so the robot's PBR materials (the Isaac Franka ships
 * real metalness/roughness maps) have something to reflect instead of
 * rendering dull under bare lights.
 *
 * `RoomEnvironment` is procedural — it builds the environment from emissive
 * boxes rather than fetching an HDRI, which keeps the studio working with no
 * network. The intensity is low: this is a sheen on top of the key light, not
 * the light source.
 */
function IndoorLighting() {
  const gl = useThree((s) => s.gl);
  const scene = useThree((s) => s.scene);
  useEffect(() => {
    const pmrem = new THREE.PMREMGenerator(gl);
    const room = RoomEnvironment();
    const target = pmrem.fromScene(room, 0.04);
    // three-stdlib's RoomEnvironment has no dispose() of its own, and the
    // baked cube map is all we keep.
    room.traverse((o) => {
      const mesh = o as THREE.Mesh;
      if (mesh.isMesh) {
        mesh.geometry.dispose();
        (Array.isArray(mesh.material) ? mesh.material : [mesh.material]).forEach(
          (m) => m.dispose(),
        );
      }
    });
    pmrem.dispose();
    scene.environment = target.texture;
    scene.environmentIntensity = 0.32;
    return () => {
      scene.environment = null;
      target.dispose();
    };
  }, [gl, scene]);
  return null;
}

export function Viewport() {
  const connected = useStudioStore((s) => s.connection === "connected");
  const selection = useStudioStore((s) => s.selection);
  const multi = useStudioStore((s) => s.robots.length > 1);
  const focusedTcp = useStudioStore((s) =>
    selection.type === "tcp"
      ? (robotByName(s.robots, selection.robot)?.tcpLink ?? null)
      : null,
  );

  // The robot name only disambiguates when several robots share the scene.
  const scope = (robot: string) => (multi ? `${robot} · ` : "");
  const focusLabel =
    selection.type === "obstacle"
      ? `obstacle · ${selection.name}`
      : selection.type === "robot"
        ? `${scope(selection.robot)}robot base`
        : `${scope(selection.robot)}TCP · ${focusedTcp ?? "—"}`;

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
        shadows="soft"
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
        {/* The environment map does the ambient work, so the lights below it
            are only the key and a fill; stacking a bright ambient on top of
            an IBL is what flattens a scene out. */}
        <IndoorLighting />
        <ambientLight intensity={0.12} />
        {/* Key light. The shadow camera covers a cell-sized volume; a
            default-sized one would clip the shadows off at 5 cm. */}
        <directionalLight
          position={[3, 3, 6]}
          intensity={1.05}
          castShadow
          shadow-mapSize={[2048, 2048]}
          shadow-bias={-0.0006}
          shadow-normalBias={0.02}
          shadow-camera-left={-4}
          shadow-camera-right={4}
          shadow-camera-top={4}
          shadow-camera-bottom={-4}
          shadow-camera-near={0.1}
          shadow-camera-far={20}
        />
        {/* Fill from the opposite side so the shadowed faces don't go flat. */}
        <directionalLight position={[-3, -2, 2]} intensity={0.3} />
        <hemisphereLight args={["#8899aa", "#20242c", 0.25]} />

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
