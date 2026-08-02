import { useEffect, useMemo, useRef, useState } from "react";
import { Edges, TransformControls } from "@react-three/drei";
import type { ThreeEvent } from "@react-three/fiber";
import * as THREE from "three";

import type { GeometryMsg, ObstacleMsg } from "../protocol";
import { collidingObstacleNames, useStudioStore } from "../store";
import { cursorEnter, cursorLeave } from "../three/cursor";
import { COLLISION_COLOR } from "../three/palette";
import { sendUpdateObstaclePose } from "../ws";
import { MeshVisual } from "./MeshVisual";

const NEUTRAL_COLOR = "#9aa3b2";
const SELECT_EDGE_COLOR = "#cdd4df";

/**
 * The obstacle's own colour, or null when the scene file authored none.
 *
 * `ObstacleMsg.color` carries `primvars:displayColor`, which USD defines in
 * linear space — the same space three works in — so it is handed over as-is
 * rather than through a hex string, which would be read back as sRGB and come
 * out washed out.
 */
function authoredColor(obstacle: ObstacleMsg): THREE.Color | null {
  if (!obstacle.color) return null;
  const [r, g, b] = obstacle.color;
  return new THREE.Color().setRGB(r, g, b, THREE.LinearSRGBColorSpace);
}

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
  // During trajectory playback, attached objects follow their baked track
  // instead of the live scene pose.
  const overridePose = useStudioStore(
    (s) => s.overrideObstaclePoses?.[name] ?? null,
  );

  // Follow the store pose except while the user is dragging this obstacle, so
  // the server's echo (and a rejected move) don't fight the gizmo.
  useEffect(() => {
    if (!group || draggingRef.current) return;
    const shown = overridePose ?? pose;
    group.position.set(...shown.position);
    group.quaternion.set(...shown.quaternion);
  }, [group, pose, overridePose]);

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

  // Scenery the author styled is drawn as scenery: solid, and it casts. An
  // obstacle with no authored colour is a bare collision proxy, so it keeps
  // the translucent look that lets you see the robot through it.
  const tint = useMemo(() => authoredColor(obstacle), [obstacle]);
  const color = colliding ? new THREE.Color(COLLISION_COLOR) : (tint ?? new THREE.Color(NEUTRAL_COLOR));

  return (
    <>
      <group ref={setGroup}>
        <ObstacleGeometry
          geometry={obstacle.geometry}
          color={color}
          solid={tint !== null}
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
  solid,
  selected,
  onSelect,
}: {
  geometry: GeometryMsg;
  color: THREE.Color;
  solid: boolean;
  selected: boolean;
  onSelect: (e: ThreeEvent<MouseEvent>) => void;
}) {
  const highlight = selected && <Edges color={SELECT_EDGE_COLOR} lineWidth={2} />;
  const pick = {
    onClick: onSelect,
    onPointerOver: (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      cursorEnter();
    },
    onPointerOut: () => cursorLeave(),
    // A see-through proxy casting a hard shadow reads as a glitch, so only
    // solid scenery casts. Everything receives.
    castShadow: solid,
    receiveShadow: true,
  };

  switch (geometry.kind) {
    case "box":
      return (
        <mesh {...pick}>
          <boxGeometry args={geometry.size} />
          <ObstacleMaterial color={color} solid={solid} />
          {highlight}
        </mesh>
      );
    case "cylinder":
      // URDF cylinders point along +Z; three's CylinderGeometry points along
      // +Y, so rotate +90deg about X to align them (same as SceneView).
      return (
        <mesh rotation={[Math.PI / 2, 0, 0]} {...pick}>
          <cylinderGeometry
            args={[geometry.radius, geometry.radius, geometry.length, 32]}
          />
          <ObstacleMaterial color={color} solid={solid} />
          {highlight}
        </mesh>
      );
    case "sphere":
      return (
        <mesh {...pick}>
          <sphereGeometry args={[geometry.radius, 32, 24]} />
          <ObstacleMaterial color={color} solid={solid} />
          {highlight}
        </mesh>
      );
    case "mesh":
      // URL is empty in wasm mode (no mesh serving there yet).
      if (!geometry.url) return null;
      return (
        <group {...pick}>
          <MeshVisual geometry={geometry} color={`#${color.getHexString()}`} />
        </group>
      );
    default:
      return null;
  }
}

function ObstacleMaterial({ color, solid }: { color: THREE.Color; solid: boolean }) {
  return (
    <meshStandardMaterial
      color={color}
      roughness={solid ? 0.8 : 0.7}
      metalness={0.05}
      transparent={!solid}
      opacity={solid ? 1 : 0.85}
    />
  );
}
