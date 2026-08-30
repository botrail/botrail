import { useEffect, useState } from "react";
import * as THREE from "three";
import { ThreeUsdRobotLoader, type ThreeUsdRobot } from "three-usd-robot";
import { highlightLink, restoreLinkMaterials } from "three-usd-robot/helpers";

import type { PoseMsg, RobotDescMsg } from "../protocol";
import { playbackRig } from "../playbackRig";
import { collidingLinkNames, useStudioStore } from "../store";
import { cursorEnter, cursorLeave } from "../three/cursor";

/**
 * Client-side USD robot rendering, one instance per USD-sourced robot: the
 * same stage the server planned against is fetched from `/usd-assets` and
 * posed via three-usd-robot's FK — the wire only carries joint values.
 * Link/joint names are prim paths on both sides, so state and collisions
 * map 1:1 within each instance; the instance name scopes them.
 */
export function UsdRobotView() {
  const robots = useStudioStore((s) => s.robots);
  return (
    <>
      {robots
        .filter((r) => r.desc.usd_asset)
        .map((r) => (
          <UsdRobotInstance key={r.desc.name} name={r.desc.name} />
        ))}
    </>
  );
}

function UsdRobotInstance({ name }: { name: string }) {
  const state = useStudioStore(
    (s) => s.robots.find((r) => r.desc.name === name) ?? null,
  );
  const url = state?.desc.usd_asset?.url;
  const [robot, setRobot] = useState<ThreeUsdRobot | null>(null);

  const overrideJoints = useStudioStore(
    (s) => s.overrideJoints?.[name] ?? null,
  );
  const overridePoses = useStudioStore(
    (s) => s.overridePoses?.[name] ?? null,
  );
  const collisions = useStudioStore((s) => s.collisions);

  // Load once per asset URL. The loader returns a fresh instance per call,
  // so two robots sharing one asset get independent objects.
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
        if (cancelled) return;
        // The loader leaves shadows off; the arm is the one thing in the
        // scene whose shadow tells you how far above the part it is.
        r.traverse((o) => {
          if ((o as THREE.Mesh).isMesh) {
            o.castShadow = true;
            o.receiveShadow = true;
          }
        });
        setRobot(r);
      })
      .catch((e) =>
        console.error("botrail studio: failed to load USD robot", e),
      );
    return () => {
      cancelled = true;
    };
  }, [url]);

  const desc = state?.desc ?? null;

  // Displayed joints: playback override wins over the live state. While a
  // link-pose (baked) override runs, setJointValues must stay silent — it
  // would snap the library back to fk display mode; when the override ends
  // (`baked` flips), this effect re-fires and that same call restores fk.
  const baked = overridePoses !== null;
  const displayed = overrideJoints ?? state?.jointPositions ?? null;
  useEffect(() => {
    if (!robot || !desc || !displayed || baked) return;
    robot.setJointValues(jointMap(robot, desc, displayed));
  }, [robot, desc, displayed, baked]);

  // Link-pose playback (transform-mode USD recordings): world-space link
  // targets drive the prims directly; the library undoes the robot
  // object's own placement, so recorded base motion replays correctly.
  useEffect(() => {
    if (!robot || !desc || !overridePoses) return;
    robot.setLinkTransforms(linkPoseMap(desc, overridePoses), {
      space: "world",
    });
  }, [robot, desc, overridePoses]);

  // Base placement. A robot riding a vehicle has no fixed base, so while a
  // timeline plays its base comes from the track rather than the scene.
  const overrideBase = useStudioStore((s) => s.overrideBases?.[name] ?? null);
  const basePose = overrideBase ?? state?.basePose ?? null;
  useEffect(() => {
    if (!robot || !basePose) return;
    applyPose(robot, basePose);
  }, [robot, basePose]);

  // Playback fast path: while a timeline plays, the driver hands each
  // sampled instant to this applier instead of routing it through React.
  useEffect(() => {
    if (!robot || !desc) return;
    playbackRig.usd.set(name, (sample) => {
      const joints = sample.joints?.[name];
      if (joints) robot.setJointValues(jointMap(robot, desc, joints));
      const poses = sample.poses?.[name];
      if (poses) {
        robot.setLinkTransforms(linkPoseMap(desc, poses), { space: "world" });
      }
      const base = sample.bases?.[name];
      if (base) applyPose(robot, base);
    });
    return () => {
      playbackRig.usd.delete(name);
    };
  }, [robot, desc, name]);

  // Collision highlight refers to the live state; suppress while playback
  // drives this robot.
  const playback = overrideJoints !== null || baked;
  useEffect(() => {
    if (!robot || !desc) return;
    const names = playback
      ? new Set<string>()
      : collidingLinkNames(collisions, name);
    for (const link of desc.links) {
      if (names.has(link.name)) {
        highlightLink(robot, link.name, { color: 0xff5555 });
      } else {
        restoreLinkMaterials(robot, link.name);
      }
    }
  }, [robot, desc, collisions, playback, name]);

  if (!robot) return null;
  // The click handler makes the robot opaque to picking: without it, R3F
  // ignores handler-less meshes and a click on the arm would select
  // whatever obstacle lies behind it. Clicking a robot focuses its TCP
  // and raises the posing tab.
  return (
    <primitive
      object={robot}
      onClick={(e: { stopPropagation: () => void }) => {
        e.stopPropagation();
        const s = useStudioStore.getState();
        s.selectTcp(name);
        s.focusTab("robot");
      }}
      onPointerOver={(e: { stopPropagation: () => void }) => {
        e.stopPropagation();
        cursorEnter();
      }}
      onPointerOut={cursorLeave}
    />
  );
}

/** Link-pose array -> {body prim path: world pose} for three-usd-robot. */
function linkPoseMap(
  desc: RobotDescMsg,
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
  robot: ThreeUsdRobot,
  desc: RobotDescMsg,
  positions: number[],
): Record<string, number> {
  const map: Record<string, number> = {};
  const qIndex = new Map(desc.joints.map((j) => [j.name, j.q_index]));
  for (const joint of desc.joints) {
    if (joint.q_index !== null) {
      map[joint.name] = positions[joint.q_index] ?? 0;
    } else if (joint.mimic && !robot.isMimicFollower(joint.name)) {
      // Mimic joints hold no DOF of their own. The loader drives the ones it
      // read itself (NewtonMimicAPI / PhysxMimicJointAPI) from their leader
      // and ignores direct sets, so skip those; a stage that carries the
      // coupling as `botrail:mimic` customData (what URDF-to-USD converters
      // write) is read by botrail alone, and there the value the server
      // derives is what places the follower.
      const source = qIndex.get(joint.mimic.joint) ?? null;
      const value = source === null ? 0 : (positions[source] ?? 0);
      map[joint.name] = joint.mimic.multiplier * value + joint.mimic.offset;
    }
  }
  return map;
}

function applyPose(object: ThreeUsdRobot, pose: PoseMsg): void {
  object.position.set(...pose.position);
  object.quaternion.set(...pose.quaternion);
}
