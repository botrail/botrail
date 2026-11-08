import { useSyncExternalStore } from "react";
import * as THREE from "three";
import type { ThreeUsdRobot } from "three-usd-robot";
import type { TcpTargetHandle } from "three-usd-robot/xr";

import { linkKey, playbackRig } from "../playbackRig";
import type { RobotDescMsg } from "../protocol";

/**
 * The live robot objects, by instance name, for the pieces that work on
 * a robot's three.js side rather than on the store: the TCP handle
 * follows a link object, the headset's grip grabs the handle. USD robots
 * register their library object here (`UsdRobotView`); URDF robots are
 * their link groups in `playbackRig.links`, dressed by {@link linkRigRobot}.
 */
const usd = new Map<string, ThreeUsdRobot>();
const listeners = new Set<() => void>();

export const robotRig = {
  /** USD robots' library objects (client-side FK). */
  usd,
  /** The TCP handle on the focused robot, while one is shown. */
  tcpHandle: null as TcpTargetHandle | null,
  setUsd(name: string, robot: ThreeUsdRobot | null): void {
    if (robot) usd.set(name, robot);
    else usd.delete(name);
    for (const listener of listeners) listener();
  },
  subscribe(listener: () => void): () => void {
    listeners.add(listener);
    return () => {
      listeners.delete(listener);
    };
  },
};

/** The named USD robot's object, re-rendering when it loads or goes. */
export function useUsdRobot(name: string | null): ThreeUsdRobot | null {
  return useSyncExternalStore(robotRig.subscribe, () =>
    name ? (usd.get(name) ?? null) : null,
  );
}

/**
 * A URDF robot wearing the three members `TcpTargetHandle` reads of a
 * `ThreeUsdRobot`: `getLinkObject` (the link exists), `getLinkWorldMatrix`
 * (where it is) and the object's own `matrixWorld` (the "robot" frame).
 * The studio's link groups are world-posed, so the frame is the identity.
 * A type lie, true at run time; three-usd-robot M39 (design-vr.md §7)
 * would make it an honest overload.
 */
export function linkRigRobot(name: string, desc: RobotDescMsg): ThreeUsdRobot {
  const indexOf = (link: string) => desc.links.findIndex((l) => l.name === link);
  const frame = new THREE.Object3D();
  frame.name = `${name}:links`;
  const adapter = Object.assign(frame, {
    getLinkObject: (link: string) =>
      indexOf(link) >= 0 ? { primPath: link, linkName: link } : undefined,
    getLinkWorldMatrix: (link: string) => {
      const index = indexOf(link);
      const object = index >= 0 ? playbackRig.links.get(linkKey(name, index)) : undefined;
      if (!object) return new THREE.Matrix4();
      object.updateWorldMatrix(true, false);
      return object.matrixWorld;
    },
  });
  return adapter as unknown as ThreeUsdRobot;
}
