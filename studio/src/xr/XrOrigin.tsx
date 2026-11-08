import { type ReactNode, useEffect, useMemo, useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { XROrigin, useXR } from "@react-three/xr";
import * as THREE from "three";
import { xrOriginQuaternion } from "three-usd-robot/xr";

import { initialView } from "../three/initialView";
import { xrAutomation } from "./automation";
import { WORLD_UP, applyPlacement, stancePlacement } from "./locomotion";
import { headPose, xrRig } from "./rig";
import { stanceFromView } from "./stance";

/**
 * The player's origin (their feet). Carries the Z-up rotation — botrail's
 * world is Z-up, the XR reference space Y-up with the floor at y = 0; the
 * origin is rotated, the content never is, so controller and head poses
 * land in world coordinates (`.internal/design-vr.md` §3.3). On the second
 * XR frame — the first with a valid head pose — it stands the player at
 * the stance derived from the launch view; locomotion moves it from
 * there. Reset to the reference-space origin when the session ends so the
 * next one starts the same way. The children are the hand-anchored pieces
 * (`XrHands`): XR spaces resolve under this group.
 */
export function XrOrigin({ children }: { children?: ReactNode }) {
  const ref = useRef<THREE.Group>(null);
  const session = useXR((s) => s.session);
  const frameRate = useXR((s) => s.frameRate);
  const stance = useMemo(
    () => stanceFromView(initialView(window.location.search)),
    [],
  );
  const pending = useRef(false);

  useEffect(() => {
    xrRig.origin = ref.current;
    xrRig.stance = stance;
    xrAutomation.xrFrames = 0;
    xrAutomation.placed = false;
    pending.current = session !== undefined;
    if (!session && ref.current) {
      ref.current.position.set(0, 0, 0);
      ref.current.quaternion.copy(xrOriginQuaternion(WORLD_UP));
      xrRig.head = null;
    }
  }, [session, stance]);
  useEffect(() => {
    xrAutomation.frameRate = frameRate ?? null;
  }, [frameRate]);

  useFrame(({ gl, camera }) => {
    xrAutomation.presenting = gl.xr.isPresenting;
    xrAutomation.antialias = gl.getContextAttributes()?.antialias ?? null;
    if (!gl.xr.isPresenting || !ref.current) return;
    xrAutomation.xrFrames++;
    // Under <XR> the frame camera is the XR camera: its local pose is the
    // head in the reference space, valid from the second frame on.
    xrRig.head = camera;
    if (pending.current && xrAutomation.xrFrames >= 2) {
      const head = headPose(camera);
      if (!head) return;
      pending.current = false;
      applyPlacement(ref.current, stancePlacement(stance, head));
      xrAutomation.placed = true;
    }
  });

  return (
    <XROrigin ref={ref} quaternion={xrOriginQuaternion(WORLD_UP)}>
      {children}
    </XROrigin>
  );
}
