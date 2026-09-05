import { useCallback, useMemo } from "react";
import * as THREE from "three";

import { linkKey, playbackRig } from "../playbackRig";
import type { GeometryMsg, LinkMsg, PoseMsg, VisualMsg } from "../protocol";
import {
  collidingLinkNames,
  useStudioStore,
  type RobotUiState,
} from "../store";
import { cursorEnter, cursorLeave } from "../three/cursor";
import { authoredColor, COLLISION_COLOR, UNPAINTED } from "../three/palette";
import { MeshVisual } from "./MeshVisual";
import { UsdVisual } from "./UsdVisual";
import { UNIT_BOX, UNIT_CYLINDER, UNIT_SPHERE } from "../three/primitiveGeometry";

const IDENTITY_POS: [number, number, number] = [0, 0, 0];
const IDENTITY_QUAT: [number, number, number, number] = [0, 0, 0, 1];

/** Link-visual rendering of every non-USD robot (one instance per robot). */
export function SceneView() {
  const robots = useStudioStore((s) => s.robots);
  return (
    <>
      {robots
        .filter((r) => !r.desc.usd_asset)
        .map((r) => (
          <LinkVisualRobot key={r.desc.name} robot={r} />
        ))}
    </>
  );
}

function LinkVisualRobot({ robot }: { robot: RobotUiState }) {
  const name = robot.desc.name;
  const overridePoses = useStudioStore(
    (s) => s.overridePoses?.[name] ?? null,
  );
  const collisions = useStudioStore((s) => s.collisions);
  const collidingLinks = useMemo(
    () => collidingLinkNames(collisions, name),
    [collisions, name],
  );

  // During trajectory playback this robot renders at its override poses;
  // collision coloring refers to the live state, so it is suppressed.
  // Robots the playback doesn't drive stay live (their colors are valid).
  const poses = overridePoses ?? robot.linkPoses;
  const playback = overridePoses !== null;

  // The click handler makes the robot opaque to picking: without it, R3F
  // ignores handler-less meshes and a click on the arm would select
  // whatever obstacle lies behind it. Clicking a robot focuses its TCP
  // and raises the posing tab.
  return (
    <group
      onClick={(e) => {
        e.stopPropagation();
        const s = useStudioStore.getState();
        s.selectTcp(name);
        s.focusTab("robot");
      }}
      onPointerOver={(e) => {
        e.stopPropagation();
        cursorEnter();
      }}
      onPointerOut={cursorLeave}
    >
      {robot.desc.links.map((link, i) => (
        <LinkGroup
          key={i}
          robot={name}
          link={link}
          index={i}
          pose={poses[i]}
          colliding={!playback && collidingLinks.has(link.name)}
        />
      ))}
    </group>
  );
}

function LinkGroup({
  robot,
  link,
  index,
  pose,
  colliding,
}: {
  robot: string;
  link: LinkMsg;
  index: number;
  pose: PoseMsg | undefined;
  colliding: boolean;
}) {
  const color = colliding ? COLLISION_COLOR : authoredColor(UNPAINTED);
  const position = pose ? pose.position : IDENTITY_POS;
  const quaternion = pose ? pose.quaternion : IDENTITY_QUAT;
  // Registered so the playback driver can move this link without a React
  // pass; React remains the source of truth whenever nothing is playing.
  const register = useCallback(
    (group: THREE.Group | null) => {
      const key = linkKey(robot, index);
      if (group) {
        playbackRig.links.set(key, group);
      } else {
        playbackRig.links.delete(key);
      }
    },
    [robot, index],
  );

  return (
    <group ref={register} position={position} quaternion={quaternion}>
      {link.visuals.map((visual, j) => (
        // Three shades, most specific first: the collision highlight is
        // the message and always wins; then the color the robot file
        // authored for this visual; then the unpainted neutral, which is
        // what the exported stage uses for the same geometry. A mesh
        // carrying its own materials keeps them unless one of the first
        // two speaks.
        <VisualNode key={j} visual={visual} color={color} forceColor={colliding} />
      ))}
    </group>
  );
}

function VisualNode({
  visual,
  color,
  forceColor,
}: {
  visual: VisualMsg;
  color: string | THREE.Color;
  forceColor: boolean;
}) {
  const { origin } = visual;
  const own = useMemo(
    () => (visual.color ? authoredColor(visual.color) : null),
    [visual.color],
  );
  return (
    <group position={origin.position} quaternion={origin.quaternion}>
      {visual.visual_asset ? <UsdVisual source={visual.visual_asset}
        color={!forceColor && own ? own : color}
        forceColor={forceColor || !!visual.visual_asset.color_override} /> : <GeometryMesh
        geometry={visual.geometry}
        color={!forceColor && own ? own : color}
        forceColor={forceColor || own !== null}
      />}
    </group>
  );
}

function GeometryMesh({
  geometry,
  color,
  forceColor = false,
}: {
  geometry: GeometryMsg;
  color: string | THREE.Color;
  forceColor?: boolean;
}) {
  switch (geometry.kind) {
    case "box":
      return (
        <mesh castShadow receiveShadow scale={geometry.size}>
          <primitive object={UNIT_BOX} attach="geometry" />
          <StandardMaterial color={color} />
        </mesh>
      );
    case "cylinder":
      // URDF cylinders point along +Z; three's CylinderGeometry points along
      // +Y, so rotate +90deg about X to align them.
      return (
        <mesh rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow
          scale={[geometry.radius, geometry.length, geometry.radius]}>
          <primitive object={UNIT_CYLINDER} attach="geometry" />
          <StandardMaterial color={color} />
        </mesh>
      );
    case "sphere":
      return (
        <mesh castShadow receiveShadow scale={geometry.radius}>
          <primitive object={UNIT_SPHERE} attach="geometry" />
          <StandardMaterial color={color} />
        </mesh>
      );
    case "mesh":
      return (
        <MeshVisual geometry={geometry} color={color} forceColor={forceColor} castShadow receiveShadow />
      );
    default:
      return null;
  }
}

function StandardMaterial({ color }: { color: string | THREE.Color }) {
  return <meshStandardMaterial color={color} roughness={0.85} metalness={0.05} />;
}
