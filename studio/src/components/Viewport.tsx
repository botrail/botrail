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
import { ContactMarkerView } from "./ContactMarkerView";
import { SensorView } from "./SensorView";
import { CameraView } from "./CameraView";
import { LidarView } from "./LidarView";
import { CameraPass } from "./CameraPass";
import { CameraPip } from "./CameraPip";
import { CameraExporter } from "./CameraExporter";
import { Aid, CameraRigBridge } from "../three/cameraRig";
import { CutTraceView } from "./CutTraceView";
import { ToolpathView } from "./ToolpathView";
import { LegendHud } from "./LegendHud";
import { SprayView } from "./SprayView";
import { VehiclePathView } from "./VehiclePathView";
import { TcpGizmo } from "./TcpGizmo";
import { IoOverlay } from "./IoOverlay";
import { IoTopologyOverlay } from "./IoTopologyOverlay";
import { LadderOverlay } from "./LadderChart";
import { SfcOverlay } from "./SfcChart";
import { TimelineDock } from "./TimelineDock";
import { UsdRobotView } from "./UsdRobotView";
import { WasmStageView } from "./WasmStageView";
import { RENDER_QUALITY } from "../three/renderQuality";
import { colorPipeline } from "../three/colorPipeline";
import { floorFinish } from "../three/floorFinish";

/**
 * Image-based lighting, so the robot's PBR materials (the Isaac Franka ships
 * real metalness/roughness maps) have something to reflect instead of
 * rendering dull under bare lights.
 *
 * `RoomEnvironment` is procedural — it builds the environment from emissive
 * boxes rather than fetching an HDRI, which keeps the studio working with no
 * network. Bare metal relies on this reflected light, so it must remain
 * readable alongside the painted surfaces lit by the key light.
 */
function IndoorLighting() {
  const gl = useThree((s) => s.gl);
  const scene = useThree((s) => s.scene);
  useEffect(() => {
    const pmrem = new THREE.PMREMGenerator(gl);
    const room = RoomEnvironment();
    room.rotation.x = Math.PI / 2; // RoomEnvironment is Y-up; botrail is Z-up.
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
    scene.environmentIntensity = 0.75;
    return () => {
      scene.environment = null;
      target.dispose();
    };
  }, [gl, scene]);
  return null;
}

export function Viewport() {
  const quality = useStudioStore((s) => s.renderQuality);
  const connected = useStudioStore((s) => s.connection === "connected");
  const selection = useStudioStore((s) => s.selection);
  const multi = useStudioStore((s) => s.robots.length > 1);
  const focusedTcp = useStudioStore((s) =>
    selection.type === "tcp"
      ? (robotByName(s.robots, selection.robot)?.tcpLink ?? null)
      : null,
  );
  const focusedArm = useStudioStore((s) =>
    selection.type === "tcp"
      ? (robotByName(s.robots, selection.robot)?.selectedGroup ?? null)
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
            : selection.type === "camera"
              ? `camera · ${selection.name}`
            : selection.type === "lidar"
              ? `lidar · ${selection.name}`
            : selection.type === "io_node"
              ? `I/O node · ${selection.name}`
              : selection.type === "robot"
                ? `${scope(selection.robot)}robot base`
                : `${scope(selection.robot)}${focusedArm ? `${focusedArm} · ` : ""}TCP · ${focusedTcp ?? "—"}`;

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
        gl={{ antialias: false }}
        dpr={[1, RENDER_QUALITY[quality].dpr]}
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
        <CameraRigBridge />
        <RenderQuality />
        {/* The environment map does the ambient work, so the lights below it
            are only the key and a fill; stacking a bright ambient on top of
            an IBL is what flattens a scene out. */}
        <IndoorLighting />
        <ambientLight intensity={0.12} />
        {/* Shadow bounds follow the cell; quality controls map resolution. */}
        <ShadowFollow />
        {/* Fill from the opposite side so the shadowed faces don't go flat. */}
        <directionalLight position={[-3, -2, 2]} intensity={0.3} />
        <hemisphereLight position={[0, 0, 1]} args={["#8899aa", "#20242c", 0.25]} />

        {/* Something for the cell to stand on. A grid alone reads as graph
            paper, and a shadow with nothing to land on leaves every object
            floating; a plain matte floor under it is what turns a diagram
            into a room. It sits a hair below z = 0 so the grid still draws
            on top, and it only receives — a floor that cast shadows would
            shadow itself. Kept to a cell's worth of floor rather than a
            horizon: every pixel of it costs a shadow lookup, and a plane
            big enough to fill the view is what makes a soft renderer
            crawl. Beyond it the grid carries on. */}
        <Floor />

        {/* drei's Grid lies in the XZ plane; rotate it onto XY (Z-up floor).
            An authoring aid — wrapped so the camera pass hides it (the
            floor above stays: a camera does see the floor). */}
        <Aid>
          <Grid
            rotation={[Math.PI / 2, 0, 0]}
            infiniteGrid
            cellSize={0.1}
            cellThickness={0.6}
            cellColor="#50565a"
            sectionSize={1}
            sectionThickness={1}
            sectionColor="#626b71"
            fadeDistance={16}
            fadeStrength={1}
          />
        </Aid>
        <Aid>
          <axesHelper args={[0.3]} />
        </Aid>

        <OrbitControls makeDefault target={[0, 0, 0.2]} />

        <Suspense fallback={null}>
          <SceneView />
          <UsdRobotView />
          <WasmStageView />
          <ObstacleView />
          {/* Aids the camera pass hides: sensor volumes, camera gizmos,
              guide paths, toolpath overlays, contact markers, transform
              gizmos. Process light (flash/spray/trace) stays — a camera
              would see it. */}
          <Aid>
            <SensorView />
          </Aid>
          <Aid>
            <CameraView />
          </Aid>
          <Aid>
            <LidarView />
          </Aid>
          <FlashView />
          <Aid>
            <ContactMarkerView />
          </Aid>
          <SprayView />
          <Aid>
            <VehiclePathView />
          </Aid>
          <Aid>
            <ToolpathView />
          </Aid>
          <CutTraceView />
          <Aid>
            <TcpGizmo />
          </Aid>
          <Aid>
            <RobotBaseGizmo />
          </Aid>
          <PlaybackDriver />
          <CameraPass />
          <CameraExporter />
        </Suspense>
      </Canvas>

      {connected && <div className="focus-chip">{focusLabel}</div>}
      <CameraPip />
      <SfcOverlay />
      <LadderOverlay />
      <IoOverlay />
      <IoTopologyOverlay />
      <LegendHud />
      <TimelineDock />
      {!connected && <div className="overlay">connecting…</div>}
    </div>
  );
}

function RenderQuality() {
  const gl = useThree((s) => s.gl);
  const quality = useStudioStore((s) => s.renderQuality);
  useEffect(() => {
    colorPipeline(gl).samples = RENDER_QUALITY[quality].samples;
  }, [gl, quality]);
  return null;
}

function Floor() {
  const gl = useThree((s) => s.gl);
  const maps = useMemo(() => floorFinish(gl.capabilities.getMaxAnisotropy()), [gl]);
  // R3F v8 assigns renderer.outputColorSpace to texture-valued JSX props,
  // including data maps. Own this material so normal/roughness stay raw.
  const material = useMemo(() => new THREE.MeshStandardMaterial({
    color: "#44484b", roughness: 0.92, metalness: 0, ...maps,
  }), [maps]);
  const geometry = useMemo(() => {
    const g = new THREE.PlaneGeometry(48, 48);
    const uv = g.getAttribute("uv");
    for (let i = 0; i < uv.count; i++) uv.setXY(i, uv.getX(i) * 48, uv.getY(i) * 48);
    return g;
  }, []);
  useEffect(() => () => {
    geometry.dispose();
    maps.normalMap.dispose();
    maps.roughnessMap.dispose();
    material.dispose();
  }, [geometry, maps, material]);
  return <mesh position={[0, 0, -0.002]} receiveShadow>
    <primitive object={geometry} attach="geometry" />
    <primitive object={material} attach="material" />
  </mesh>;
}

/** The key light, with its shadow frustum resized to the scene.
 *
 * Obstacles change rarely (authoring), so this recomputes on the obstacle
 * list rather than per frame: the XY extent of everything in the cell,
 * padded for the arms, clamped so a lone robot keeps a crisp map and a
 * 25 m line still lands entirely inside the frustum. */
function ShadowFollow() {
  const quality = useStudioStore((s) => s.renderQuality);
  const shadowSize = RENDER_QUALITY[quality].shadowSize;
  const light = useRef<THREE.DirectionalLight | null>(null);
  const obstacles = useStudioStore((s) => s.obstacles);
  const robots = useStudioStore((s) => s.robots);

  const frame = useMemo(() => {
    let minX = -1;
    let maxX = 1;
    let minY = -1;
    let maxY = 1;
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
      Math.max((Math.max(maxX - minX, maxY - minY) + 6) / 2, 4),
      26,
    );
    return { cx, cy, half };
  }, [obstacles, robots]);

  useEffect(() => {
    const l = light.current;
    if (!l) return;
    l.position.set(frame.cx + 5, frame.cy - 4, 8 + frame.half * 0.5);
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

  useEffect(() => {
    const shadow = light.current?.shadow;
    if (!shadow) return;
    shadow.map?.dispose();
    shadow.map = null;
    shadow.mapSize.set(shadowSize, shadowSize);
    shadow.needsUpdate = true;
  }, [shadowSize]);

  return (
    <directionalLight
      ref={light}
      position={[6, 5, 9]}
      intensity={1.35}
      castShadow
      shadow-bias={-0.00015}
      shadow-normalBias={0.003}
      shadow-camera-near={0.1}
      shadow-camera-far={40}
    />
  );
}
