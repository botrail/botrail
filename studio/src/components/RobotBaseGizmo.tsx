import { useEffect, useRef, useState } from "react";
import { TransformControls } from "@react-three/drei";
import * as THREE from "three";

import { robotByName, useStudioStore } from "../store";
import { sendRobotBasePose } from "../ws";

/**
 * Draggable robot base for the focused robot. The gizmo moves a proxy
 * marker at the base pose; every change is sent to the server, which
 * re-anchors the robot and streams the shifted link poses back. While not
 * dragging, the marker tracks the authoritative base pose from the server.
 */
export function RobotBaseGizmo() {
  const selection = useStudioStore((s) => s.selection);
  const focusedRobot = selection.type === "robot" ? selection.robot : null;
  const robot = useStudioStore((s) => robotByName(s.robots, focusedRobot));
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const overriding = useStudioStore(
    (s) => s.overridePoses !== null || s.overrideJoints !== null,
  );

  const [target, setTarget] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);

  const basePose = robot?.basePose ?? null;
  useEffect(() => {
    if (!basePose || !target || draggingRef.current) return;
    target.position.set(...basePose.position);
    target.quaternion.set(...basePose.quaternion);
  }, [basePose, target]);

  if (!robot || !basePose || overriding) return null;
  const robotName = robot.desc.name;

  const onDrag = () => {
    // objectChange can also fire on attach/programmatic updates; only
    // user drags may move the base.
    if (!target || !draggingRef.current) return;
    sendRobotBasePose(robotName, {
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
