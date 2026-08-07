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
import { VehiclePathView } from "./VehiclePathView";
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
      : selection.type === "group"
        ? `group · ${selection.path}`
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
        {/* Key light. The shadow camera covers a *line*-sized volume, not
            just an arm's reach: a cell with a conveyor running through it
            is ten metres end to end, and a tighter frustum cuts the
            shadows off mid-floor. The map stays at 2k: 4k covers the same
            span at twice the sharpness and several times the cost, which
            software renderers (the headless screenshots) will not carry. */}
        <directionalLight
          position={[6, 5, 9]}
          intensity={1.05}
          castShadow
          shadow-mapSize={[2048, 2048]}
          shadow-bias={-0.0006}
          shadow-normalBias={0.02}
          shadow-camera-left={-9}
          shadow-camera-right={9}
          shadow-camera-top={9}
          shadow-camera-bottom={-9}
          shadow-camera-near={0.1}
          shadow-camera-far={40}
        />
        {/* Fill from the opposite side so the shadowed faces don't go flat. */}
        <directionalLight position={[-3, -2, 2]} intensity={0.3} />
        <hemisphereLight args={["#8899aa", "#20242c", 0.25]} />

        {/* Something for the cell to stand on. A grid alone reads as graph
            paper, and a shadow with nothing to land on leaves every object
            floating; a plain matte floor under it is what turns a diagram
            into a room. It sits a hair below z = 0 so the grid still draws
            on top, and it only receives — a floor that cast shadows would
            shadow itself. Kept to a cell's worth of floor rather than a
            horizon: every pixel of it costs a shadow lookup, and a plane
            big enough to fill the view is what makes a soft renderer
            crawl. Beyond it the grid carries on. */}
        <mesh position={[0, 0, -0.002]} receiveShadow>
          <planeGeometry args={[48, 48]} />
          <meshStandardMaterial color="#1a1d23" roughness={0.94} metalness={0} />
        </mesh>

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
          <VehiclePathView />
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
