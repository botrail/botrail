import type { GeometryMsg, LinkMsg, PoseMsg, VisualMsg } from "../protocol";
import { useStudioStore } from "../store";
import { linkColor } from "../three/palette";
import { MeshVisual } from "./MeshVisual";

const IDENTITY_POS: [number, number, number] = [0, 0, 0];
const IDENTITY_QUAT: [number, number, number, number] = [0, 0, 0, 1];

export function SceneView() {
  const sceneDesc = useStudioStore((s) => s.sceneDesc);
  const linkPoses = useStudioStore((s) => s.linkPoses);

  if (!sceneDesc) return null;

  return (
    <>
      {sceneDesc.links.map((link, i) => (
        <LinkGroup key={i} link={link} index={i} pose={linkPoses[i]} />
      ))}
    </>
  );
}

function LinkGroup({
  link,
  index,
  pose,
}: {
  link: LinkMsg;
  index: number;
  pose: PoseMsg | undefined;
}) {
  const color = linkColor(index);
  const position = pose ? pose.position : IDENTITY_POS;
  const quaternion = pose ? pose.quaternion : IDENTITY_QUAT;

  return (
    <group position={position} quaternion={quaternion}>
      {link.visuals.map((visual, j) => (
        <VisualNode key={j} visual={visual} color={color} />
      ))}
    </group>
  );
}

function VisualNode({ visual, color }: { visual: VisualMsg; color: string }) {
  const { origin } = visual;
  return (
    <group position={origin.position} quaternion={origin.quaternion}>
      <GeometryMesh geometry={visual.geometry} color={color} />
    </group>
  );
}

function GeometryMesh({
  geometry,
  color,
}: {
  geometry: GeometryMsg;
  color: string;
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
      return <MeshVisual geometry={geometry} color={color} />;
    default:
      return null;
  }
}

function StandardMaterial({ color }: { color: string }) {
  return <meshStandardMaterial color={color} roughness={0.85} metalness={0.05} />;
}
