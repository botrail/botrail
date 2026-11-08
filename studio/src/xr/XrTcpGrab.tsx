import { useXRInputSourceEvent, useXRInputSourceState } from "@react-three/xr";
import * as THREE from "three";

import { robotRig } from "../three/robotRig";
import { xrRig } from "./rig";

/** How close the grip has to be to the handle to take it (metres). */
const GRAB_REACH = 0.2;

/**
 * The right grip takes the TCP handle (`.internal/design-vr.md` §3.5):
 * squeeze within reach and the handle keeps its offset to the hand until
 * the squeeze ends; `TcpHandleView` turns its moves into IK targets and
 * lets it snap back to the real TCP on release. The grip button does
 * nothing else in the studio (`grabPointer: false`), so a squeeze away
 * from the handle is simply ignored.
 */
export function XrTcpGrab() {
  const right = useXRInputSourceState("controller", "right");
  useXRInputSourceEvent(
    right?.inputSource,
    "squeezestart",
    () => {
      const grip = xrRig.grips.right;
      const handle = robotRig.tcpHandle;
      if (!grip || !handle || !handle.parent) return;
      grip.updateWorldMatrix(true, false);
      handle.updateWorldMatrix(true, false);
      const distance = new THREE.Vector3()
        .setFromMatrixPosition(grip.matrixWorld)
        .distanceTo(new THREE.Vector3().setFromMatrixPosition(handle.matrixWorld));
      if (distance <= GRAB_REACH) handle.grab(grip);
    },
    [right],
  );
  useXRInputSourceEvent(
    right?.inputSource,
    "squeezeend",
    () => robotRig.tcpHandle?.release(),
    [right],
  );
  return null;
}
