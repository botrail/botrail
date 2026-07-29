import { useEffect, useMemo, useRef, useState } from "react";
import { Edges, TransformControls } from "@react-three/drei";
import type { ThreeEvent } from "@react-three/fiber";
import * as THREE from "three";

import type { GeometryMsg, ObstacleMsg } from "../protocol";
import { collidingObstacleNames, useStudioStore } from "../store";
import { COLLISION_COLOR } from "../three/palette";
import { sendUpdateObstaclePose } from "../ws";
import { MeshVisual } from "./MeshVisual";

const NEUTRAL_COLOR = "#9aa3b2";
const SELECT_EDGE_COLOR = "#cdd4df";

/** Draws every obstacle, handles click-to-select, and drives the move gizmo. */
export function ObstacleView() {
  const obstacles = useStudioStore((s) => s.obstacles);
  const collisions = useStudioStore((s) => s.collisions);
  const selection = useStudioStore((s) => s.selection);
  const hiddenObstacles = useStudioStore((s) => s.hiddenObstacles);
  const collidingObstacles = useMemo(
    () => collidingObstacleNames(collisions),
    [collisions],
  );

  return (
    <>
      {obstacles.filter((o) => !hiddenObstacles.has(o.name)).map((o) => (
        <ObstacleNode
          key={o.name}
          obstacle={o}
          colliding={collidingObstacles.has(o.name)}
          selected={selection.type === "obstacle" && selection.name === o.name}
        />
      ))}
    </>
  );
}

function ObstacleNode({
  obstacle,
  colliding,
  selected,
}: {
  obstacle: ObstacleMsg;
  colliding: boolean;
  selected: boolean;
}) {
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const selectObstacle = useStudioStore((s) => s.selectObstacle);

  const [group, setGroup] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);
  const { pose, name } = obstacle;

  // Follow the store pose except while the user is dragging this obstacle, so
  // the server's echo (and a rejected move) don't fight the gizmo.
  useEffect(() => {
    if (!group || draggingRef.current) return;
    group.position.set(...pose.position);
    group.quaternion.set(...pose.quaternion);
  }, [group, pose]);

  const onDrag = () => {
    // objectChange also fires on attach/programmatic updates; only real drags
    // should move the obstacle.
    if (!group || !draggingRef.current) return;
    sendUpdateObstaclePose(name, {
      position: [group.position.x, group.position.y, group.position.z],
      quaternion: [
        group.quaternion.x,
        group.quaternion.y,
        group.quaternion.z,
        group.quaternion.w,
      ],
    });
  };

  const onSelect = (e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
    selectObstacle(name);
  };

  const color = colliding ? COLLISION_COLOR : NEUTRAL_COLOR;

  return (
    <>
      <group ref={setGroup}>
        <ObstacleGeometry
          geometry={obstacle.geometry}
          color={color}
          selected={selected}
          onSelect={onSelect}
        />
      </group>
      {selected && group && (
        <TransformControls
          object={group}
          mode={gizmoMode}
          size={0.65}
          onMouseDown={() => {
            draggingRef.current = true;
          }}
          onMouseUp={() => {
            draggingRef.current = false;
          }}
          onObjectChange={onDrag}
        />
      )}
    </>
  );
}

function ObstacleGeometry({
  geometry,
  color,
  selected,
  onSelect,
}: {
  geometry: GeometryMsg;
  color: string;
  selected: boolean;
  onSelect: (e: ThreeEvent<MouseEvent>) => void;
}) {
  const highlight = selected && <Edges color={SELECT_EDGE_COLOR} lineWidth={2} />;

  switch (geometry.kind) {
    case "box":
      return (
        <mesh onClick={onSelect}>
          <boxGeometry args={geometry.size} />
          <ObstacleMaterial color={color} />
          {highlight}
        </mesh>
      );
    case "cylinder":
      // URDF cylinders point along +Z; three's CylinderGeometry points along
      // +Y, so rotate +90deg about X to align them (same as SceneView).
      return (
        <mesh rotation={[Math.PI / 2, 0, 0]} onClick={onSelect}>
          <cylinderGeometry
            args={[geometry.radius, geometry.radius, geometry.length, 32]}
          />
          <ObstacleMaterial color={color} />
          {highlight}
        </mesh>
      );
    case "sphere":
      return (
        <mesh onClick={onSelect}>
          <sphereGeometry args={[geometry.radius, 32, 24]} />
          <ObstacleMaterial color={color} />
          {highlight}
        </mesh>
      );
    case "mesh":
      // URL is empty in wasm mode (no mesh serving there yet).
      if (!geometry.url) return null;
      return (
        <group onClick={onSelect}>
          <MeshVisual geometry={geometry} color={color} />
        </group>
      );
    default:
      return null;
  }
}

function ObstacleMaterial({ color }: { color: string }) {
  return (
    <meshStandardMaterial
      color={color}
      roughness={0.7}
      metalness={0.05}
      transparent
      opacity={0.85}
    />
  );
}
