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
import { COLLISION_COLOR, linkColor } from "../three/palette";
import { MeshVisual } from "./MeshVisual";

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
  // whatever obstacle lies behind it. Clicking a robot focuses its TCP.
  return (
    <group
      onClick={(e) => {
        e.stopPropagation();
        useStudioStore.getState().selectTcp(name);
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
  const color = colliding ? COLLISION_COLOR : linkColor(index);
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
        // The link color is a way to tell links apart, so a mesh carrying
        // the manufacturer's own colors keeps them. In collision the color
        // is the message, and it wins.
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
  color: string;
  forceColor: boolean;
}) {
  const { origin } = visual;
  return (
    <group position={origin.position} quaternion={origin.quaternion}>
      <GeometryMesh
        geometry={visual.geometry}
        color={color}
        forceColor={forceColor}
      />
    </group>
  );
}

function GeometryMesh({
  geometry,
  color,
  forceColor = false,
}: {
  geometry: GeometryMsg;
  color: string;
  forceColor?: boolean;
}) {
  switch (geometry.kind) {
    case "box":
      return (
        <mesh>
          <boxGeometry args={geometry.size} />
          <StandardMaterial color={color} />
        </mesh>
      );
    case "cylinder":
      // URDF cylinders point along +Z; three's CylinderGeometry points along
      // +Y, so rotate +90deg about X to align them.
      return (
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry
            args={[geometry.radius, geometry.radius, geometry.length, 32]}
          />
          <StandardMaterial color={color} />
        </mesh>
      );
    case "sphere":
      return (
        <mesh>
          <sphereGeometry args={[geometry.radius, 32, 24]} />
          <StandardMaterial color={color} />
        </mesh>
      );
    case "mesh":
      return (
        <MeshVisual geometry={geometry} color={color} forceColor={forceColor} />
      );
    default:
      return null;
  }
}

function StandardMaterial({ color }: { color: string }) {
  return <meshStandardMaterial color={color} roughness={0.85} metalness={0.05} />;
}
