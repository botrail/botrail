import { useEffect, type ReactNode } from "react";
import { useThree } from "@react-three/fiber";
import { XR } from "@react-three/xr";

import { useStudioStore } from "../store";
import { robotRig } from "../three/robotRig";
import { resetRig } from "./rig";
import { XrHands } from "./XrHands";
import { XrLocomotion } from "./XrLocomotion";
import { XrOrigin } from "./XrOrigin";
import { XrTcpGrab } from "./XrTcpGrab";
import { xrStore } from "./xrStore";

export { enterVr, exitVr, xrStore } from "./xrStore";

/**
 * Wraps the scene in `<XR>` when the probe said a headset may be present;
 * otherwise renders the children as they are, so the desktop path is the
 * one it always was (no combined pointer, no XR camera in the graph).
 * `xr.supported` is set before the Canvas mounts and never changes.
 * Inside: the origin with the hand-anchored pieces, the locomotion, then
 * the scene itself.
 */
export function XrSession({ children }: { children: ReactNode }) {
  const supported = useStudioStore((s) => s.xr.supported);
  if (!supported) return <>{children}</>;
  return (
    <XR store={xrStore}>
      <PresentingBridge />
      <XrOrigin>
        <XrHands />
      </XrOrigin>
      <XrLocomotion />
      <XrTcpGrab />
      {children}
    </XR>
  );
}

/** Mirrors three's `sessionstart` / `sessionend` into the store — the exact
 * moments the renderer switches between the desktop pass and the XR layer,
 * so `<CameraPass>` leaves and returns in step with it. A hover from the
 * other input is dropped at both edges: the mouse's chip and the ray's
 * label should each start clean. */
function PresentingBridge() {
  const gl = useThree((s) => s.gl);
  useEffect(() => {
    const onStart = () => {
      const s = useStudioStore.getState();
      // The PiP is painted by the camera pass, which stands down in XR.
      s.setPipCamera(null);
      s.setHover(null);
      s.setXrPresenting(true);
    };
    const onEnd = () => {
      const s = useStudioStore.getState();
      s.setHover(null);
      // A handle still in the hand lets go: the grip space is gone.
      robotRig.tcpHandle?.release();
      resetRig();
      s.setXrPresenting(false);
    };
    gl.xr.addEventListener("sessionstart", onStart);
    gl.xr.addEventListener("sessionend", onEnd);
    return () => {
      gl.xr.removeEventListener("sessionstart", onStart);
      gl.xr.removeEventListener("sessionend", onEnd);
      useStudioStore.getState().setXrPresenting(false);
    };
  }, [gl]);
  return null;
}
