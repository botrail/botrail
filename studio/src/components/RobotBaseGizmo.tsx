import { useEffect, useRef, useState } from "react";
import { TransformControls } from "@react-three/drei";
import * as THREE from "three";

import { useStudioStore } from "../store";
import { sendRobotBasePose } from "../ws";

/**
 * Draggable robot base. The gizmo moves a proxy marker at the base pose;
 * every change is sent to the server, which re-anchors the robot and
 * streams the shifted link poses back. While not dragging, the marker
 * tracks the authoritative base pose from the server.
 */
export function RobotBaseGizmo() {
  const basePose = useStudioStore((s) => s.basePose);
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const selection = useStudioStore((s) => s.selection);
  const overriding = useStudioStore(
    (s) => s.overridePoses !== null || s.overrideJoints !== null,
  );

  const [target, setTarget] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);

  useEffect(() => {
    if (!basePose || !target || draggingRef.current) return;
    target.position.set(...basePose.position);
    target.quaternion.set(...basePose.quaternion);
  }, [basePose, target]);

  if (!basePose || selection.type !== "robot" || overriding) return null;

  const onDrag = () => {
    // objectChange can also fire on attach/programmatic updates; only
    // user drags may move the base.
    if (!target || !draggingRef.current) return;
    sendRobotBasePose({
      position: [target.position.x, target.position.y, target.position.z],
      quaternion: [
        target.quaternion.x,
        target.quaternion.y,
        target.quaternion.z,
        target.quaternion.w,
      ],
    });
  };

  return (
    <>
      <group ref={setTarget}>
        <mesh renderOrder={10}>
          <cylinderGeometry args={[0.05, 0.05, 0.012, 24]} />
          <meshBasicMaterial
            color="#ffb84d"
            depthTest={false}
            transparent
            opacity={0.85}
          />
        </mesh>
      </group>
      {target && (
        <TransformControls
          object={target}
          mode={gizmoMode}
          size={0.75}
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
