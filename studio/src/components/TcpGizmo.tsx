import { useEffect, useRef, useState } from "react";
import { TransformControls } from "@react-three/drei";
import * as THREE from "three";

import { useStudioStore } from "../store";
import { sendTcpTarget } from "../ws";

/**
 * Draggable TCP target. The gizmo moves a free-floating target object; every
 * change is sent to the server, which solves IK and streams the resulting
 * robot state back. While not dragging, the target tracks the actual TCP so
 * an unreachable drop visibly snaps back to where the arm really is.
 */
export function TcpGizmo() {
  const sceneDesc = useStudioStore((s) => s.sceneDesc);
  const tcpLink = useStudioStore((s) => s.tcpLink);
  const linkPoses = useStudioStore((s) => s.linkPoses);
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const ikStatus = useStudioStore((s) => s.ikStatus);
  const selection = useStudioStore((s) => s.selection);

  const [target, setTarget] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);

  const linkIndex =
    sceneDesc && tcpLink
      ? sceneDesc.links.findIndex((l) => l.name === tcpLink)
      : -1;
  const pose = linkIndex >= 0 ? linkPoses[linkIndex] : undefined;

  // Track the real TCP while the user is not dragging.
  useEffect(() => {
    if (!pose || !target || draggingRef.current) return;
    target.position.set(...pose.position);
    target.quaternion.set(...pose.quaternion);
  }, [pose, target]);

  // While an obstacle is selected, its gizmo takes over the viewport; and
  // during trajectory playback the gizmo would point at the live (not the
  // displayed) TCP, so hide it too.
  const overriding = useStudioStore((s) => s.overridePoses !== null);
  if (!pose || !tcpLink || selection.type !== "tcp" || overriding) return null;

  const reachable = ikStatus === null || ikStatus.converged;
  const color = reachable ? "#4da3ff" : "#ff5555";

  const onDrag = () => {
    // objectChange can also fire on attach/programmatic updates; only
    // user drags may drive the robot.
    if (!target || !draggingRef.current) return;
    sendTcpTarget({
      link: tcpLink,
      pose: {
        position: [target.position.x, target.position.y, target.position.z],
        quaternion: [
          target.quaternion.x,
          target.quaternion.y,
          target.quaternion.z,
          target.quaternion.w,
        ],
      },
    });
  };

  return (
    <>
      <group ref={setTarget}>
        <mesh renderOrder={10}>
          <sphereGeometry args={[0.014, 20, 14]} />
          <meshBasicMaterial
            color={color}
            depthTest={false}
            transparent
            opacity={0.9}
          />
        </mesh>
      </group>
      {target && (
        <TransformControls
          object={target}
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
