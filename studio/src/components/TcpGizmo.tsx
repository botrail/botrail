import { useEffect, useRef, useState } from "react";
import { TransformControls } from "@react-three/drei";
import * as THREE from "three";

import { groupTip, robotArms, robotByName, useStudioStore } from "../store";
import { sendTcpTarget } from "../ws";

/**
 * Draggable TCP target for the focused robot. The gizmo moves a free-
 * floating target object; every change is sent to the server, which solves
 * IK and streams the resulting robot state back. While not dragging, the
 * target tracks the actual TCP so an unreachable drop visibly snaps back to
 * where the arm really is.
 */
export function TcpGizmo() {
  const selection = useStudioStore((s) => s.selection);
  const focusedRobot = selection.type === "tcp" ? selection.robot : null;
  const robot = useStudioStore((s) => robotByName(s.robots, focusedRobot));
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const setSelectedGroup = useStudioStore((s) => s.setSelectedGroup);

  const [target, setTarget] = useState<THREE.Group | null>(null);
  const draggingRef = useRef(false);

  const tcpLink = robot?.tcpLink ?? null;
  const linkIndex =
    robot && tcpLink
      ? robot.desc.links.findIndex((l) => l.name === tcpLink)
      : -1;
  const pose = linkIndex >= 0 ? robot?.linkPoses[linkIndex] : undefined;

  // Track the real TCP while the user is not dragging.
  useEffect(() => {
    if (!pose || !target || draggingRef.current) return;
    target.position.set(...pose.position);
    target.quaternion.set(...pose.quaternion);
  }, [pose, target]);

  // While an obstacle is selected, its gizmo takes over the viewport; and
  // during trajectory playback the gizmo would point at the live (not the
  // displayed) TCP, so hide it too.
  const overriding = useStudioStore(
    (s) => s.overridePoses !== null || s.overrideJoints !== null,
  );
  if (!robot || !pose || !tcpLink || overriding) return null;

  const ikStatus = robot.ikStatus;
  const reachable = ikStatus === null || ikStatus.converged;
  const color = reachable ? "#4da3ff" : "#ff5555";
  const robotName = robot.desc.name;
  const group = robot.selectedGroup;

  // The other arms of a dual-arm robot: a flat mark at each tip, clicked
  // to move the gizmo (and the panels) over to that arm.
  const idleArms = robotArms(robot.desc)
    .filter((g) => g.name !== group)
    .map((g) => {
      const at = robot.desc.links.findIndex((l) => l.name === g.tip_link);
      return { name: g.name, pose: at >= 0 ? robot.linkPoses[at] : undefined };
    });

  const onDrag = () => {
    // objectChange can also fire on attach/programmatic updates; only
    // user drags may drive the robot.
    if (!target || !draggingRef.current) return;
    sendTcpTarget(robotName, {
      link: tcpLink,
      // The arm whose tip this is; a hand-picked link elsewhere on the
      // robot lets the server infer the arm from the link.
      group: groupTip(robot.desc, group) === tcpLink ? group : null,
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
      {idleArms.map(
        ({ name, pose }) =>
          pose && (
            <mesh
              key={name}
              position={pose.position}
              renderOrder={10}
              onClick={(e) => {
                e.stopPropagation();
                setSelectedGroup(robotName, name);
              }}
            >
              <sphereGeometry args={[0.011, 16, 12]} />
              <meshBasicMaterial
                color="#8a97a8"
                depthTest={false}
                transparent
                opacity={0.85}
              />
            </mesh>
          ),
      )}
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
