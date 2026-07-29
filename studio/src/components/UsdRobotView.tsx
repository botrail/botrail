import { useEffect, useState } from "react";
import { ThreeUsdRobotLoader, type ThreeUsdRobot } from "three-usd-robot";
import {
  createGhostRobot,
  highlightLink,
  restoreLinkMaterials,
} from "three-usd-robot/helpers";

import type { PoseMsg, SceneDescriptionMsg } from "../protocol";
import { collidingLinkNames, useStudioStore } from "../store";

/**
 * Client-side USD robot rendering: the same stage the server planned
 * against is fetched from `/assets` and posed via three-usd-robot's FK —
 * the wire only carries joint values. Link/joint names are prim paths on
 * both sides, so state, collisions, and goals map 1:1.
 */
export function UsdRobotView() {
  const sceneDesc = useStudioStore((s) => s.sceneDesc);
  const url = sceneDesc?.usd_asset?.url;
  const [robot, setRobot] = useState<ThreeUsdRobot | null>(null);
  const [ghost, setGhost] = useState<ThreeUsdRobot | null>(null);

  const jointPositions = useStudioStore((s) => s.jointPositions);
  const overrideJoints = useStudioStore((s) => s.overrideJoints);
  const basePose = useStudioStore((s) => s.basePose);
  const collisions = useStudioStore((s) => s.collisions);
  const goal = useStudioStore((s) => s.goal);

  // Load once per asset URL (a new scene_init resets the store first).
  useEffect(() => {
    if (!url) {
      setRobot(null);
      return;
    }
    let cancelled = false;
    new ThreeUsdRobotLoader()
      .loadAsync(url)
      .then((r) => {
        if (!cancelled) setRobot(r);
      })
      .catch((e) =>
        console.error("botrail studio: failed to load USD robot", e),
      );
    return () => {
      cancelled = true;
    };
  }, [url]);

  // Displayed joints: playback override wins over the live state.
  const displayed = overrideJoints ?? jointPositions;
  useEffect(() => {
    if (!robot || !sceneDesc) return;
    robot.setJointValues(jointMap(sceneDesc, displayed));
  }, [robot, sceneDesc, displayed]);

  // Base placement.
  useEffect(() => {
    if (!robot || !basePose) return;
    applyPose(robot, basePose);
  }, [robot, basePose]);

  // Collision highlight refers to the live state; suppress during playback.
  const playback = overrideJoints !== null;
  useEffect(() => {
    if (!robot || !sceneDesc) return;
    const names = playback ? new Set<string>() : collidingLinkNames(collisions);
    for (const link of sceneDesc.links) {
      if (names.has(link.name)) {
        highlightLink(robot, link.name, { color: 0xff5555 });
      } else {
        restoreLinkMaterials(robot, link.name);
      }
    }
  }, [robot, sceneDesc, collisions, playback]);

  // Translucent goal ghost, posed at the captured configuration.
  useEffect(() => {
    if (!robot || !sceneDesc || !goal) {
      setGhost(null);
      return;
    }
    const g = createGhostRobot(robot, {
      jointValues: jointMap(sceneDesc, goal.positions),
    });
    if (basePose) applyPose(g, basePose);
    setGhost(g);
    return () => setGhost(null);
  }, [robot, sceneDesc, goal, basePose]);

  if (!robot) return null;
  return (
    <>
      <primitive object={robot} />
      {ghost && <primitive object={ghost} />}
    </>
  );
}

/** DOF vector -> {joint prim path: value} for three-usd-robot. */
function jointMap(
  desc: SceneDescriptionMsg,
  positions: number[],
): Record<string, number> {
  const map: Record<string, number> = {};
  for (const joint of desc.joints) {
    if (joint.q_index !== null) {
      map[joint.name] = positions[joint.q_index] ?? 0;
    }
  }
  return map;
}

function applyPose(object: ThreeUsdRobot, pose: PoseMsg): void {
  object.position.set(...pose.position);
  object.quaternion.set(...pose.quaternion);
}
