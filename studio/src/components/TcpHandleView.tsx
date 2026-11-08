import { useEffect, useMemo } from "react";
import { TransformControls } from "@react-three/drei";
import { type TcpTargetEvent, TcpTargetHandle } from "three-usd-robot/xr";

import { groupTip, robotArms, robotByName, useStudioStore } from "../store";
import { linkRigRobot, robotRig, useUsdRobot } from "../three/robotRig";
import { sendTcpTarget } from "../ws";

/** Smoothing of a hand-held target: a controller jitters, a mouse does
 * not — and a smoothed mouse gizmo never quite stops (M38). */
const XR_SMOOTHING = 0.2;

/**
 * The TCP target of the focused robot — three-usd-robot's
 * `TcpTargetHandle`, one object under two inputs: on the desktop a
 * `TransformControls` drags it (the gizmo), in a headset the right grip
 * takes it (`XrTcpGrab`). Every move while held goes to the server, which
 * solves IK and streams the robot back; while nobody holds it the handle
 * follows the real tool link, so an unreachable drop visibly snaps back
 * to where the arm is. The solver's verdict colours it (red: out of
 * reach). Hidden during playback, when the live TCP is not what is drawn.
 */
export function TcpHandleView() {
  const selection = useStudioStore((s) => s.selection);
  const focusedRobot = selection.type === "tcp" ? selection.robot : null;
  const robot = useStudioStore((s) => robotByName(s.robots, focusedRobot));
  const gizmoMode = useStudioStore((s) => s.gizmoMode);
  const setSelectedGroup = useStudioStore((s) => s.setSelectedGroup);
  const presenting = useStudioStore((s) => s.xr.presenting);
  const overriding = useStudioStore(
    (s) => s.overridePoses !== null || s.overrideJoints !== null,
  );
  const desc = robot?.desc ?? null;
  const robotName = desc?.name ?? null;
  const tcpLink = robot?.tcpLink ?? null;
  const group = robot?.selectedGroup ?? null;
  const ikStatus = robot?.ikStatus ?? null;
  // A USD robot's handle follows the library object; it may still be loading.
  const usdRobot = useUsdRobot(desc?.usd_asset ? robotName : null);

  const handle = useMemo(() => {
    if (!desc || !robotName || !tcpLink) return null;
    const source = desc.usd_asset ? usdRobot : linkRigRobot(robotName, desc);
    if (!source) return null;
    try {
      return new TcpTargetHandle(source, tcpLink, { size: 0.08 });
    } catch (e) {
      console.warn("botrail studio: no TCP handle for", robotName, e);
      return null;
    }
  }, [desc, robotName, tcpLink, usdRobot]);
  useEffect(() => () => handle?.dispose(), [handle]);

  // The handle others reach for: the headset's grip and the automation.
  useEffect(() => {
    robotRig.tcpHandle = handle;
    return () => {
      if (robotRig.tcpHandle === handle) robotRig.tcpHandle = null;
    };
  }, [handle]);
  useEffect(() => {
    if (handle) handle.smoothing = presenting ? XR_SMOOTHING : 0;
  }, [handle, presenting]);
  useEffect(() => {
    handle?.setStatus({ reachable: ikStatus === null || ikStatus.converged });
  }, [handle, ikStatus]);

  // Every move of a held handle is a target for the server's IK (`ws`
  // throttles to ~30 Hz and stops any playback).
  useEffect(() => {
    if (!handle || !robotName || !desc || !tcpLink) return;
    const onChange = (e: TcpTargetEvent) => {
      if (e.phase !== "move") return;
      sendTcpTarget(robotName, {
        link: tcpLink,
        // The arm whose tip this is; a hand-picked link elsewhere on the
        // robot lets the server infer the arm from the link.
        group: groupTip(desc, group) === tcpLink ? group : null,
        pose: e.pose,
      });
    };
    handle.addEventListener("change", onChange);
    return () => handle.removeEventListener("change", onChange);
  }, [handle, robotName, desc, tcpLink, group]);

  if (!robot || !desc || !handle || overriding) return null;

  // The other arms of a dual-arm robot: a flat mark at each tip, clicked
  // to move the handle (and the panels) over to that arm.
  const idleArms = robotArms(desc)
    .filter((g) => g.name !== group)
    .map((g) => {
      const at = desc.links.findIndex((l) => l.name === g.tip_link);
      return { name: g.name, pose: at >= 0 ? robot.linkPoses[at] : undefined };
    });

  return (
    <>
      <primitive object={handle} />
      {idleArms.map(
        ({ name, pose }) =>
          pose && (
            <mesh
              key={name}
              position={pose.position}
              renderOrder={10}
              onClick={(e) => {
                e.stopPropagation();
                setSelectedGroup(desc.name, name);
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
      {!presenting && (
        <TransformControls
          object={handle}
          mode={gizmoMode}
          size={0.65}
          onMouseDown={() => handle.grab()}
          onMouseUp={() => handle.release()}
        />
      )}
    </>
  );
}
