import { Suspense, useEffect, useMemo, useRef } from "react";
import { Canvas, useFrame, useThree } from "@react-three/fiber";
import { Html, Line, OrbitControls } from "@react-three/drei";
import * as THREE from "three";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import type { PoseMsg, RobotDescMsg } from "../protocol";
import {
  checkFor,
  engagementFor,
  list,
  number,
  range,
  record,
  selectionHole,
  type Connection,
  type Face,
  type Inspection,
  type MountingMode,
  type Side,
} from "../mounting";
import { VisualNode } from "./SceneView";

function matrix(p: PoseMsg): THREE.Matrix4 {
  return new THREE.Matrix4().compose(
    new THREE.Vector3(...p.position),
    new THREE.Quaternion(...p.quaternion),
    new THREE.Vector3(1, 1, 1),
  );
}
function localPose(p: PoseMsg, origin: THREE.Matrix4, shift = 0): PoseMsg {
  const m = origin.clone().multiply(matrix(p)),
    position = new THREE.Vector3(),
    quaternion = new THREE.Quaternion();
  m.decompose(position, quaternion, new THREE.Vector3());
  position.z += shift;
  return { position: position.toArray(), quaternion: quaternion.toArray() };
}

export type ComparisonBounds = {
  min: [number, number, number];
  max: [number, number, number];
};
type ComparisonFit = {
  bounds: ComparisonBounds | null;
  onBounds: (bounds: ComparisonBounds) => void;
};

export function MountingViewport(props: {
  inspection: Inspection;
  connection: Connection;
  robot: RobotDescMsg;
  mode: MountingMode;
  explode: number;
  transparent: boolean;
  hideFlange: boolean;
  selected: string;
  onSelect: (key: string) => void;
  focus: number;
  comparison?: ComparisonFit;
}) {
  return (
    <Canvas
      className="mounting-canvas"
      dpr={[1, 1.5]}
      gl={{ antialias: true }}
      camera={{
        position: [0.22, -0.3, 0.2],
        up: [0, 0, 1],
        fov: 40,
        near: 0.0001,
        far: 100,
      }}
    >
      <color attach="background" args={["#e9eef2"]} />
      <ambientLight intensity={1.3} />
      <hemisphereLight args={["#ffffff", "#8e9aa8", 1.3]} />
      <directionalLight position={[1, -2, 3]} intensity={2.5} />
      <OrbitControls makeDefault minDistance={0.008} />
      <Suspense fallback={null}>
        <InspectionModel {...props} />
      </Suspense>
    </Canvas>
  );
}

function InspectionModel({
  inspection,
  connection: c,
  robot,
  mode,
  explode,
  transparent,
  hideFlange,
  selected,
  onSelect,
  focus,
  comparison,
}: {
  inspection: Inspection;
  connection: Connection;
  robot: RobotDescMsg;
  mode: MountingMode;
  explode: number;
  transparent: boolean;
  hideFlange: boolean;
  selected: string;
  onSelect: (key: string) => void;
  focus: number;
  comparison?: ComparisonFit;
}) {
  const origin = useMemo(
    () => matrix(c.flange.pose ?? c.mount.pose ?? robot.base_pose).invert(),
    [c, robot],
  );
  const moving = useMemo(() => new Set(c.movingLinks), [c]);
  const poses = useMemo(
    () =>
      new Map(
        inspection.robots
          .find((r) => r.name === c.robot)
          ?.links.map((l) => [l.name, l]),
      ),
    [inspection, c.robot],
  );
  const shift = mode === "exploded" ? (explode * c.flange.normal) / 1000 : 0;
  const movingGroup = useRef<THREE.Group>(null);
  const links = (move: boolean) =>
    robot.links
      .filter((l) => moving.has(l.name) === move)
      .map((link) => {
        const view = poses.get(link.name);
        if (!view) return null;
        const p = localPose(view.pose, origin, move ? shift : 0),
          parent = view.part === c.flange.part,
          child = view.part === c.mount.part;
        const color = parent ? "#70a6c7" : child ? "#c5a47d" : "#b5c0ca";
        const material = {
          metalness: 0.05,
          roughness: 0.72,
          opacity: move ? (transparent ? 0.2 : 1) : parent ? 0.72 : 0.12,
        };
        return (
          <group
            key={link.name}
            position={p.position}
            quaternion={p.quaternion}
          >
            {link.visuals.map((visual, i) => (
              <VisualNode
                key={i}
                visual={visual}
                color={color}
                forceColor
                material={material}
              />
            ))}
          </group>
        );
      });
  const selectedHole = selectionHole(c, selected);
  const target = useMemo(() => {
    if (
      !selectedHole ||
      !c[selectedHole.side].mapped ||
      !c[selectedHole.side].pose
    )
      return null;
    return new THREE.Vector3(
      selectedHole.hole.position[0] / 1000,
      selectedHole.hole.position[1] / 1000,
      0,
    )
      .applyMatrix4(matrix(c[selectedHole.side].pose!))
      .applyMatrix4(origin)
      .add(new THREE.Vector3(0, 0, selectedHole.side === "mount" ? shift : 0));
  }, [selectedHole, c, origin, shift]);
  return (
    <>
      <group visible={!hideFlange}>{links(false)}</group>
      <group ref={movingGroup}>{links(true)}</group>
      <InspectionFocus
        group={movingGroup}
        resetKey={`${c.robot}:${c.target}:${focus}:${shift}`}
        focusTarget={focus > 0 ? target : null}
        comparison={comparison}
      />
      {(["flange", "mount"] as const).map((side) => {
        const face = c[side];
        if (!face.pose) return null;
        const p = localPose(face.pose, origin, side === "mount" ? shift : 0);
        return (
          <group key={side} position={p.position} quaternion={p.quaternion}>
            <axesHelper args={[0.015]} />
            <Html
              position={[0, 0, 0.02]}
              center
              className="mounting-frame-label"
            >
              {side}: {face.frame}
            </Html>
            {face.mapped &&
              face.holes.map((h) => {
                const r = (h.diameter?.min ?? h.thread?.diameter ?? 0) / 2000;
                if (r <= 0) return null;
                const active = selected === `${side}:${h.id}`;
                return (
                  <group
                    key={h.id}
                    position={[h.position[0] / 1000, h.position[1] / 1000, 0]}
                  >
                    <mesh
                      name={`declared-hole:${side}:${h.id}`}
                      onClick={(e) => {
                        e.stopPropagation();
                        onSelect(`${side}:${h.id}`);
                      }}
                      renderOrder={5}
                    >
                      <ringGeometry
                        args={[Math.max(0, r - 0.0005), r + 0.0005, 40]}
                      />
                      <meshBasicMaterial
                        color={
                          active
                            ? "#8c4bb0"
                            : side === "flange"
                              ? "#267eb1"
                              : "#b87538"
                        }
                        side={THREE.DoubleSide}
                        depthTest={false}
                      />
                    </mesh>
                    {active && (
                      <Html
                        center
                        position={[0, 0, 0.009]}
                        className="mounting-hole-label"
                      >
                        {h.id}
                      </Html>
                    )}
                  </group>
                );
              })}
            {face.mapped &&
              face.locators.map(
                (l) =>
                  l.diameter && (
                    <mesh
                      key={l.id}
                      position={[l.position[0] / 1000, l.position[1] / 1000, 0]}
                    >
                      <ringGeometry
                        args={[
                          l.diameter.min / 2000,
                          l.diameter.max / 2000 + 0.0004,
                          40,
                        ]}
                      />
                      <meshBasicMaterial
                        color="#73899d"
                        side={THREE.DoubleSide}
                        depthTest={false}
                      />
                    </mesh>
                  ),
              )}
            {face.mapped &&
              face.holes
                .filter((h) => h.kind === "clearance")
                .map((h) => {
                  const key = `${side}:${h.id}`,
                    check = engagementFor(inspection, c, key);
                  const length = range(check?.inputs.length_mm),
                    grip = range(check?.inputs.grip_mm),
                    washer = range(check?.inputs.washer_mm);
                  const diameter = number(
                    record(
                      checkFor(inspection, c, `${h.id}:thread`)?.inputs
                        .selected,
                    ).diameter_mm,
                  );
                  // Only a dimensioned shaft envelope, never an invented head or thread.
                  // Intervals belong in the section view, not a falsely exact solid.
                  if (
                    !length ||
                    !grip ||
                    !washer ||
                    !diameter ||
                    length.min !== length.max ||
                    grip.min !== grip.max ||
                    washer.min !== washer.max
                  )
                    return null;
                  return (
                    <mesh
                      key={`shaft:${h.id}`}
                      name={`declared-shaft:${key}`}
                      position={[
                        h.position[0] / 1000,
                        h.position[1] / 1000,
                        (-face.normal *
                          (grip.min + washer.min - length.min / 2)) /
                          1000,
                      ]}
                      rotation={[Math.PI / 2, 0, 0]}
                      onClick={(e) => {
                        e.stopPropagation();
                        onSelect(key);
                      }}
                    >
                      <cylinderGeometry
                        args={[
                          diameter / 2000,
                          diameter / 2000,
                          length.min / 1000,
                          24,
                        ]}
                      />
                      <meshBasicMaterial
                        color={
                          selected === key
                            ? "#8c4bb0"
                            : check?.status === "fail"
                              ? "#bf594c"
                              : "#617c90"
                        }
                        wireframe
                        transparent
                        opacity={0.65}
                        depthTest={false}
                      />
                    </mesh>
                  );
                })}
            {mode === "access" && face.clearanceMapped && (
              <Space
                face={face}
                side={side}
                inspection={inspection}
                connection={c}
              />
            )}
          </group>
        );
      })}
      {mode === "exploded" && c.flange.pose && c.mount.pose && (
        <Line
          points={[[0, 0, 0], localPose(c.mount.pose, origin, shift).position]}
          color="#678499"
          dashed
          dashSize={0.004}
          gapSize={0.003}
          lineWidth={1}
        />
      )}
    </>
  );
}

function Space({
  face,
  side,
  inspection,
  connection,
}: {
  face: Face;
  side: Side;
  inspection: Inspection;
  connection: Connection;
}) {
  const overlaps = list(
    checkFor(
      inspection,
      connection,
      side === "flange" ? "flange_access" : "tool_access",
    )?.inputs.overlaps,
  );
  const blocked = new Set(overlaps.map((pair) => list(pair)[0]));
  return (
    <>
      {face.solids.map((b) => (
        <mesh
          key={`solid:${b.id}`}
          position={b.center.map((n) => n / 1000) as [number, number, number]}
        >
          <boxGeometry
            args={b.size.map((n) => n / 1000) as [number, number, number]}
          />
          <meshBasicMaterial
            color="#6c8091"
            wireframe
            transparent
            opacity={0.55}
          />
        </mesh>
      ))}
      {face.access.map((b) => (
        <mesh
          key={`access:${b.id}`}
          position={b.center.map((n) => n / 1000) as [number, number, number]}
        >
          <boxGeometry
            args={b.size.map((n) => n / 1000) as [number, number, number]}
          />
          <meshBasicMaterial
            color={blocked.has(b.id) ? "#cf655a" : "#729bb9"}
            transparent
            opacity={0.28}
            depthWrite={false}
          />
          <Html center className="mounting-frame-label">
            {b.id}
            {blocked.has(b.id) ? " · blocked" : " · access"}
          </Html>
        </mesh>
      ))}
    </>
  );
}

/** A separate canvas keeps the cell camera and shared materials intact.
 * Fit to the downstream part after async meshes arrive, until the user orbits. */
function InspectionFocus({
  group,
  resetKey,
  focusTarget,
  comparison,
}: {
  group: React.RefObject<THREE.Group>;
  resetKey: string;
  focusTarget: THREE.Vector3 | null;
  comparison?: ComparisonFit;
}) {
  const { camera, controls, size } = useThree();
  const reportedBounds = useRef("");
  const state = useRef({
    key: "",
    until: 0,
    manual: false,
    extent: "",
    tick: 0,
  });
  const orbit = controls as OrbitControlsImpl | undefined;
  useEffect(() => {
    if (!orbit) return;
    const begin = () => {
      state.current.manual = true;
    };
    orbit.addEventListener("start", begin);
    return () => orbit.removeEventListener("start", begin);
  }, [orbit]);
  useFrame(({ clock }) => {
    if (!orbit) return;
    const s = state.current;
    if (s.key !== resetKey) {
      s.key = resetKey;
      s.until = clock.elapsedTime + 12;
      s.manual = false;
      s.extent = "";
      s.tick = 0;
    }
    if (
      s.manual ||
      clock.elapsedTime > s.until ||
      clock.elapsedTime - s.tick < 0.25
    )
      return;
    s.tick = clock.elapsedTime;
    const bounds = group.current
      ? new THREE.Box3().setFromObject(group.current)
      : new THREE.Box3();
    if (comparison && !bounds.isEmpty()) {
      const value = { min: bounds.min.toArray(), max: bounds.max.toArray() };
      const signature = JSON.stringify(value);
      if (signature !== reportedBounds.current) {
        reportedBounds.current = signature;
        comparison.onBounds(value);
      }
      if (comparison.bounds) {
        bounds.min.fromArray(comparison.bounds.min);
        bounds.max.fromArray(comparison.bounds.max);
      }
    }
    // Keep both mating faces visible when the display separation grows.
    if (!bounds.isEmpty()) bounds.expandByPoint(new THREE.Vector3());
    const center =
      focusTarget?.clone() ??
      (bounds.isEmpty()
        ? new THREE.Vector3(0, 0, 0.04)
        : bounds.getCenter(new THREE.Vector3()));
    const radius = focusTarget
      ? 0.025
      : Math.max(
          0.05,
          Math.min(
            5,
            bounds.isEmpty()
              ? 0.12
              : bounds.getSize(new THREE.Vector3()).length() / 2,
          ),
        );
    const key = [...center.toArray(), radius, size.width, size.height]
      .map((n) => n.toFixed(5))
      .join();
    if (key === s.extent) return;
    s.extent = key;
    orbit.target.copy(center);
    camera.position
      .copy(center)
      .add(
        new THREE.Vector3(1, -1.4, 1)
          .normalize()
          .multiplyScalar(
            (radius * (size.width < 500 ? 4.8 : 3.8)) /
              Math.min(1, size.width / size.height),
          ),
      );
    camera.lookAt(center);
    orbit.update();
  });
  return null;
}
