import { type ReactNode, useEffect } from "react";
import { XRSpace, useXRInputSourceState } from "@react-three/xr";
import type * as THREE from "three";

import { type Hand, xrRig } from "./rig";
import { XrPanel } from "./XrPanel";
import { XrPointerLabel } from "./XrPointerLabel";

/**
 * What hangs on the controllers: the panel on the left grip, the pointer
 * label on the right, and for each hand an object tracking the target
 * ray (the teleport arc is thrown along it) and one tracking the grip
 * (the right one grabs the TCP handle). Explicit `<XRSpace>`s rather
 * than the controller models' objects, so they exist before any model
 * loads — or when none can (the profile is fetched from a CDN). Lives
 * under `<XrOrigin>`, where XR spaces are resolved.
 */
export function XrHands() {
  return (
    <>
      <HandSpaces hand="left">
        <XrPanel />
      </HandSpaces>
      <HandSpaces hand="right">
        <XrPointerLabel />
      </HandSpaces>
    </>
  );
}

function HandSpaces({ hand, children }: { hand: Hand; children: ReactNode }) {
  const state = useXRInputSourceState("controller", hand);
  const source = state?.inputSource;
  useEffect(
    () => () => {
      xrRig.rays[hand] = null;
      xrRig.grips[hand] = null;
    },
    [hand],
  );
  if (!source) return null;
  return (
    <>
      <XRSpace
        space={source.targetRaySpace}
        ref={(object: THREE.Object3D | null) => {
          xrRig.rays[hand] = object;
        }}
      />
      <XRSpace
        space={source.gripSpace ?? source.targetRaySpace}
        ref={(object: THREE.Object3D | null) => {
          xrRig.grips[hand] = object;
        }}
      >
        {children}
      </XRSpace>
    </>
  );
}
