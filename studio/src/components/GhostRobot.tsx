import type { GeometryMsg } from "../protocol";
import { robotByName, useStudioStore } from "../store";

const GHOST_COLOR = "#5b9dd9";

/**
 * Translucent copy of the goal robot at the captured configuration.
 * Mesh visuals are skipped (primitive links only) — acceptable until mesh
 * loading gets a shared cached-material path.
 */
export function GhostRobot() {
  const goal = useStudioStore((s) => s.goal);
  const robot = useStudioStore((s) => robotByName(s.robots, goal?.robot ?? null));

  // USD-rendered robots get their ghost from UsdRobotView.
  if (!goal || !robot || robot.desc.usd_asset) return null;

  return (
    <>
      {robot.desc.links.map((link, i) => {
        const pose = goal.linkPoses[i];
        if (!pose) return null;
        return (
          <group key={i} position={pose.position} quaternion={pose.quaternion}>
            {link.visuals.map((visual, j) => (
              <group
                key={j}
                position={visual.origin.position}
                quaternion={visual.origin.quaternion}
              >
                <GhostGeometry geometry={visual.geometry} />
              </group>
            ))}
          </group>
        );
      })}
    </>
  );
}

function GhostGeometry({ geometry }: { geometry: GeometryMsg }) {
  switch (geometry.kind) {
    case "box":
      return (
        <mesh>
          <boxGeometry args={geometry.size} />
          <GhostMaterial />
        </mesh>
      );
    case "cylinder":
      return (
        <mesh rotation={[Math.PI / 2, 0, 0]}>
          <cylinderGeometry
            args={[geometry.radius, geometry.radius, geometry.length, 24]}
          />
          <GhostMaterial />
        </mesh>
      );
    case "sphere":
      return (
        <mesh>
          <sphereGeometry args={[geometry.radius, 24, 18]} />
          <GhostMaterial />
        </mesh>
      );
    default:
      return null;
  }
}

function GhostMaterial() {
  return (
    <meshStandardMaterial
      color={GHOST_COLOR}
      transparent
      opacity={0.22}
      depthWrite={false}
      roughness={0.9}
    />
  );
}
