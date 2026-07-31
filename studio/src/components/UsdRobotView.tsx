import { useEffect, useState } from "react";
import { ThreeUsdRobotLoader, type ThreeUsdRobot } from "three-usd-robot";
import {
  createGhostRobot,
  highlightLink,
  restoreLinkMaterials,
} from "three-usd-robot/helpers";

import type { PoseMsg, SceneDescriptionMsg } from "../protocol";
import { collidingLinkNames, useStudioStore } from "../store";
import { cursorEnter, cursorLeave } from "../three/cursor";

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
  const overridePoses = useStudioStore((s) => s.overridePoses);
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
    // botrail's world is Z-up; the library defaults to three.js Y-up.
    new ThreeUsdRobotLoader({ worldUp: "Z" })
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

  // Displayed joints: playback override wins over the live state. While a
  // link-pose (baked) override runs, setJointValues must stay silent — it
  // would snap the library back to fk display mode; when the override ends
  // (`baked` flips), this effect re-fires and that same call restores fk.
  const baked = overridePoses !== null;
  const displayed = overrideJoints ?? jointPositions;
  useEffect(() => {
    if (!robot || !sceneDesc || baked) return;
    robot.setJointValues(jointMap(sceneDesc, displayed));
  }, [robot, sceneDesc, displayed, baked]);

  // Link-pose playback (transform-mode USD recordings): world-space link
  // targets drive the prims directly; the library undoes the robot
  // object's own placement, so recorded base motion replays correctly.
  useEffect(() => {
    if (!robot || !sceneDesc || !overridePoses) return;
    robot.setLinkTransforms(linkPoseMap(sceneDesc, overridePoses), {
      space: "world",
    });
  }, [robot, sceneDesc, overridePoses]);

  // Base placement.
  useEffect(() => {
    if (!robot || !basePose) return;
    applyPose(robot, basePose);
  }, [robot, basePose]);

  // Collision highlight refers to the live state; suppress during playback.
  const playback = overrideJoints !== null || baked;
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
  // The click handler makes the robot opaque to picking: without it, R3F
  // ignores handler-less meshes and a click on the arm would select
  // whatever obstacle lies behind it. Clicking the robot focuses the TCP.
  return (
    <>
      <primitive
        object={robot}
        onClick={(e: { stopPropagation: () => void }) => {
          e.stopPropagation();
          useStudioStore.getState().selectTcp();
        }}
        onPointerOver={(e: { stopPropagation: () => void }) => {
          e.stopPropagation();
          cursorEnter();
        }}
        onPointerOut={cursorLeave}
      />
      {ghost && <primitive object={ghost} />}
    </>
  );
}

/** Link-pose array -> {body prim path: world pose} for three-usd-robot. */
function linkPoseMap(
  desc: SceneDescriptionMsg,
  poses: PoseMsg[],
): Record<string, { position: [number, number, number]; quaternion: [number, number, number, number] }> {
  const map: Record<
    string,
    { position: [number, number, number]; quaternion: [number, number, number, number] }
  > = {};
  desc.links.forEach((link, i) => {
    const p = poses[i];
    if (p) map[link.name] = { position: p.position, quaternion: p.quaternion };
  });
  return map;
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
