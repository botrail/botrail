import { Suspense, useEffect, useMemo, useRef } from "react";
import { Canvas, useThree } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import { RoomEnvironment } from "three-stdlib";

import { isWasmMode } from "../backend";
import { robotByName, useStudioStore } from "../store";
import { dropUsdScene } from "../ws";
import { ObstacleView } from "./ObstacleView";
import { PlaybackDriver } from "./PlaybackDriver";
import { RobotBaseGizmo } from "./RobotBaseGizmo";
import { SceneView } from "./SceneView";
import { FlashView } from "./FlashView";
import { SensorView } from "./SensorView";
import { CutTraceView } from "./CutTraceView";
import { ToolpathView } from "./ToolpathView";
import { VehiclePathView } from "./VehiclePathView";
import { TcpGizmo } from "./TcpGizmo";
import { SfcOverlay } from "./SfcChart";
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
        : selection.type === "sensor"
          ? `sensor · ${selection.name}`
          : selection.type === "device"
            ? `device · ${selection.name}`
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
        {/* Key light. The shadow camera follows the scene's own extent
            (see ShadowFollow): sized to an arm's cell it cuts a line's
            shadows off mid-floor, sized to a fixed line it wastes its 2k
            map on one arm's cell. The map stays at 2k either way: 4k
            covers the same span at twice the sharpness and several times
            the cost, which software renderers (the headless screenshots)
            will not carry. */}
        <ShadowFollow />
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
          <ObstacleView />
          <SensorView />
          <FlashView />
          <VehiclePathView />
          <ToolpathView />
          <CutTraceView />
          <TcpGizmo />
          <RobotBaseGizmo />
          <PlaybackDriver />
        </Suspense>
      </Canvas>

      {connected && <div className="focus-chip">{focusLabel}</div>}
      <SfcOverlay />
      <TimelineDock />
      {!connected && <div className="overlay">connecting…</div>}
    </div>
  );
}

/** The key light, with its shadow frustum resized to the scene.
 *
 * Obstacles change rarely (authoring), so this recomputes on the obstacle
 * list rather than per frame: the XY extent of everything in the cell,
 * padded for the arms, clamped so a lone robot keeps a crisp map and a
 * 25 m line still lands entirely inside the frustum. */
function ShadowFollow() {
  const light = useRef<THREE.DirectionalLight | null>(null);
  const obstacles = useStudioStore((s) => s.obstacles);
  const robots = useStudioStore((s) => s.robots);

  const frame = useMemo(() => {
    let minX = -3;
    let maxX = 3;
    let minY = -3;
    let maxY = 3;
    for (const o of obstacles) {
      minX = Math.min(minX, o.pose.position[0]);
      maxX = Math.max(maxX, o.pose.position[0]);
      minY = Math.min(minY, o.pose.position[1]);
      maxY = Math.max(maxY, o.pose.position[1]);
    }
    for (const r of robots) {
      const p = r.basePose?.position;
      if (!p) continue;
      minX = Math.min(minX, p[0]);
      maxX = Math.max(maxX, p[0]);
      minY = Math.min(minY, p[1]);
      maxY = Math.max(maxY, p[1]);
    }
    const cx = (minX + maxX) / 2;
    const cy = (minY + maxY) / 2;
    const half = Math.min(
      Math.max((Math.max(maxX - minX, maxY - minY) + 6) / 2, 9),
      26,
    );
    return { cx, cy, half };
  }, [obstacles, robots]);

  useEffect(() => {
    const l = light.current;
    if (!l) return;
    l.position.set(frame.cx + 6, frame.cy + 5, 9 + frame.half * 0.5);
    l.target.position.set(frame.cx, frame.cy, 0);
    l.target.updateMatrixWorld();
    const cam = l.shadow.camera;
    cam.left = -frame.half;
    cam.right = frame.half;
    cam.top = frame.half;
    cam.bottom = -frame.half;
    cam.far = 40 + frame.half;
    cam.updateProjectionMatrix();
    l.shadow.needsUpdate = true;
  }, [frame]);

  return (
    <directionalLight
      ref={light}
      position={[6, 5, 9]}
      intensity={1.05}
      castShadow
      shadow-mapSize={[2048, 2048]}
      shadow-bias={-0.0006}
      shadow-normalBias={0.02}
      shadow-camera-near={0.1}
      shadow-camera-far={40}
    />
  );
}
