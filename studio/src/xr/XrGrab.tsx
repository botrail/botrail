import { useRef } from "react";
import { useFrame } from "@react-three/fiber";
import { useXRInputSourceEvent, useXRInputSourceState } from "@react-three/xr";
import * as THREE from "three";

import { inGroup } from "../partIdentity";
import { playbackRig } from "../playbackRig";
import type { PoseMsg } from "../protocol";
import {
  type JointAxis,
  clampToRange,
  handMove,
  jointAxisWorld,
  jointMotion,
  jointOf,
} from "../sceneryJoints";
import { useStudioStore } from "../store";
import { robotRig } from "../three/robotRig";
import { sendSetSceneryJoint, sendUpdateObstaclePose, sendUpdatePoses } from "../ws";
import { type Graspable, handUnit, heldPose, holdOffset, localBounds, underHand } from "./grab";
import { type Hand, headWorld, xrRig } from "./rig";

/** How close the right grip has to be to the TCP handle to take it (metres). */
const TCP_REACH = 0.2;
const HANDS: readonly Hand[] = ["left", "right"];

/** A thing carried: the obstacles (and frames) that come with the hand,
 * each by its pose in the grip's frame. */
type Carried = {
  kind: "carry";
  /** The obstacle's name, or the tree path of the group taken as one. */
  unit: string;
  group: boolean;
  obstacles: { name: string; offset: THREE.Matrix4 }[];
  frames: { name: string; offset: THREE.Matrix4 }[];
};

/** A door, a drawer, a rack worked by the hand: the joint as it stood
 * when taken, and how far the hand has since moved it. */
type Worked = {
  kind: "joint";
  /** The joint's name. */
  unit: string;
  joint: "hinge" | "slide";
  at: JointAxis;
  range: [number, number];
  /** Where the joint stood when taken, and its members then (world). */
  start: number;
  obstacles: { name: string; from: THREE.Matrix4 }[];
  /** Where the hand has asked for — past a stop when it pulls past one,
   * so the door starts back only once the hand is back at the stop. */
  asked: number;
  hand: THREE.Vector3;
};

type Held = Carried | Worked;

/** A thing just let go: how the hand left it, kept on screen until the
 * host has answered with the same. The host's echoes of the way there
 * are still arriving — a moment late on a light cell, seconds on a heavy
 * one — and each would otherwise put the thing back where it was then. */
type LetGo = {
  poses: { name: string; position: THREE.Vector3; quaternion: THREE.Quaternion }[];
  /** Whether the scene the studio holds is now the one the hand left. */
  settled: () => boolean;
  /** When to stop waiting (`performance.now()`): the scene wins. */
  until: number;
};

/** How long a let-go thing waits for the host's answer (ms). */
const SETTLE_MS = 3000;

/**
 * The grips (`.internal/design-vr.md` §3.5, §3.12). Squeeze the right one
 * within reach of the TCP handle and it takes the handle: the handle
 * keeps its offset to the hand until the squeeze ends, `TcpHandleView`
 * turns its moves into IK targets and lets it snap back to the real TCP
 * on release. Squeeze either one on a thing of the cell and the thing is
 * in the hand — a workpiece alone, a pallet whole, whatever is selected
 * as it is selected (`handUnit`) — following it, turning with it, carried
 * along while walking, and staying where it is let go. Every move is the
 * pose write the desktop gizmo sends, so the scene, the collision check
 * and the other studios follow. A squeeze on nothing does nothing.
 *
 * A thing that hangs on a hinge or runs on a slide (a scenery joint: a
 * fence door, a refrigerator door, a dishwasher rack) is not carried: the
 * hand swings it about its axis or draws it along it, as far as its
 * range goes, and it stays as it is left. To carry the appliance, take it
 * by its body.
 */
export function XrGrab() {
  const left = useXRInputSourceState("controller", "left");
  const right = useXRInputSourceState("controller", "right");
  const held = useRef<Record<Hand, Held | null>>({ left: null, right: null });
  const letGo = useRef<LetGo[]>([]);

  const take = (hand: Hand, source: XRInputSource | undefined) => {
    const grip = xrRig.grips[hand];
    if (!grip) return;
    grip.updateWorldMatrix(true, false);
    const at = new THREE.Vector3().setFromMatrixPosition(grip.matrixWorld);
    const handle = robotRig.tcpHandle;
    if (hand === "right" && handle?.parent) {
      handle.updateWorldMatrix(true, false);
      const to = new THREE.Vector3().setFromMatrixPosition(handle.matrixWorld);
      if (at.distanceTo(to) <= TCP_REACH) {
        handle.grab(grip);
        return;
      }
    }
    const taken = takeObstacles(grip.matrixWorld, at, held.current[hand === "left" ? "right" : "left"]);
    held.current[hand] = taken;
    xrRig.held[hand] = taken?.unit ?? null;
    if (taken) {
      // Taken again before its last let-go settled: the hand has it now.
      const names = new Set(taken.obstacles.map((o) => o.name));
      letGo.current = letGo.current.filter((l) => !l.poses.some((p) => names.has(p.name)));
      pulse(source);
    }
  };
  const drop = (hand: Hand) => {
    if (hand === "right") robotRig.tcpHandle?.release();
    const h = held.current[hand];
    if (h) letGo.current.push(leave(h));
    held.current[hand] = null;
    xrRig.held[hand] = null;
  };
  useXRInputSourceEvent(left?.inputSource, "squeezestart", () => take("left", left?.inputSource), [left]);
  useXRInputSourceEvent(left?.inputSource, "squeezeend", () => drop("left"), [left]);
  useXRInputSourceEvent(right?.inputSource, "squeezestart", () => take("right", right?.inputSource), [right]);
  useXRInputSourceEvent(right?.inputSource, "squeezeend", () => drop("right"), [right]);

  // After the locomotion (mounted before this): a thing carried while
  // walking goes where the hand is this frame, not where it was.
  useFrame(({ gl }) => {
    if (letGo.current.length > 0) {
      const now = performance.now();
      letGo.current = letGo.current.filter((l) => {
        if (l.settled()) return false;
        if (now > l.until) {
          // No answer: what the studio holds of the scene is the truth.
          const s = useStudioStore.getState();
          for (const { name } of l.poses) {
            const pose = s.obstacles.find((o) => o.name === name)?.pose;
            const object = playbackRig.obstacles.get(name);
            if (pose && object) {
              object.position.set(...pose.position);
              object.quaternion.set(...pose.quaternion);
            }
          }
          return false;
        }
        for (const { name, position, quaternion } of l.poses) {
          const object = playbackRig.obstacles.get(name);
          if (object) {
            object.position.copy(position);
            object.quaternion.copy(quaternion);
          }
        }
        return true;
      });
    }
    for (const hand of HANDS) {
      const h = held.current[hand];
      if (!h) continue;
      const grip = xrRig.grips[hand];
      if (!gl.xr.isPresenting || !grip) {
        // The session ended under the hand: it stays where it was last put.
        drop(hand);
        continue;
      }
      grip.updateWorldMatrix(true, false);
      if (h.kind === "joint") {
        const hand = new THREE.Vector3().setFromMatrixPosition(grip.matrixWorld);
        h.asked += handMove(h.joint, h.at, h.hand, hand);
        h.hand = hand;
        const position = clampToRange(h.range, h.asked);
        const motion = jointMotion(h.joint, h.at, position - h.start);
        for (const { name, from } of h.obstacles) {
          const object = playbackRig.obstacles.get(name);
          if (!object) continue;
          new THREE.Matrix4()
            .multiplyMatrices(motion, from)
            .decompose(object.position, object.quaternion, new THREE.Vector3());
        }
        sendSetSceneryJoint(h.unit, position);
        continue;
      }
      const obstacles = h.obstacles.map(({ name, offset }): [string, PoseMsg] => {
        const pose = heldPose(grip.matrixWorld, offset);
        // Drawn at once; the server's echo is a frame or two behind the hand.
        const object = playbackRig.obstacles.get(name);
        if (object) {
          object.position.set(...pose.position);
          object.quaternion.set(...pose.quaternion);
        }
        return [name, pose];
      });
      if (h.group) {
        const frames = h.frames.map(
          ({ name, offset }): [string, PoseMsg] => [name, heldPose(grip.matrixWorld, offset)],
        );
        sendUpdatePoses(h.unit, { obstacles, frames });
      } else {
        sendUpdateObstaclePose(h.unit, obstacles[0][1]);
      }
    }
  });
  return null;
}

/** A thing as the hand leaves it: its drawn poses, and the test that the
 * host's answer has arrived — the joint at the position last asked for,
 * the carried thing where it was put. */
function leave(h: Held): LetGo {
  const poses = h.obstacles.flatMap(({ name }) => {
    const object = playbackRig.obstacles.get(name);
    return object ? [{ name, position: object.position.clone(), quaternion: object.quaternion.clone() }] : [];
  });
  const settled =
    h.kind === "joint"
      ? () => {
          const at = useStudioStore.getState().sceneryJoints.find((j) => j.name === h.unit)?.position;
          return at === undefined || Math.abs(at - clampToRange(h.range, h.asked)) < 1e-6;
        }
      : () => {
          const first = poses[0];
          const pose = first && useStudioStore.getState().obstacles.find((o) => o.name === first.name)?.pose;
          if (!pose) return true;
          return (
            first.position.distanceTo(new THREE.Vector3(...pose.position)) < 1e-6 &&
            Math.abs(first.quaternion.dot(new THREE.Quaternion(...pose.quaternion))) > 1 - 1e-9
          );
        };
  return { poses, settled, until: performance.now() + SETTLE_MS };
}

/** The thing under a grip, as the hand holds it — or null: nothing within
 * reach, or nothing a hand may take right now. */
function takeObstacles(grip: THREE.Matrix4, at: THREE.Vector3, other: Held | null): Held | null {
  const s = useStudioStore.getState();
  // A bake still streaming is the world being recorded — and, live, the
  // physics hand's (design-vr.md V3): nothing of it is taken.
  if (s.bakeStream) return null;
  const byName = new Map(s.obstacles.map((o) => [o.name, o]));
  const inOtherHand = new Set(other?.obstacles.map((o) => o.name));
  const jointed = new Set(s.sceneryJoints.flatMap((j) => j.objects));
  const things: Graspable[] = [];
  const doors: Graspable[] = [];
  for (const [name, object] of playbackRig.obstacles) {
    // Not what a robot holds, nor the other hand, nor a part stowed out of sight.
    const obstacle = byName.get(name);
    if (!obstacle || obstacle.attached_to || !object.visible || inOtherHand.has(name)) continue;
    const box = localBounds(object);
    if (box) (jointed.has(name) ? doors : things).push({ name, box, matrixWorld: object.matrixWorld });
  }
  // A door or a drawer within reach is what the hand is on, before the
  // body behind it: an appliance's carcass is a box around everything it
  // holds, and a hand pressed a little too far into the door is inside it.
  const head = headWorld();
  const leaf = underHand(at, head, doors) ?? underHand(at, head, things);
  if (leaf === null) return null;
  // Taken from where it is drawn: a recording on display has its parts
  // where the cycle put them, not where the scene has them.
  const shown = (name: string, scene: PoseMsg): PoseMsg => {
    const object = playbackRig.obstacles.get(name);
    if (!object) return s.overrideObstaclePoses?.[name] ?? scene;
    return {
      position: [object.position.x, object.position.y, object.position.z],
      quaternion: [object.quaternion.x, object.quaternion.y, object.quaternion.z, object.quaternion.w],
    };
  };
  // A part the recording moves cannot be both in the hand and in the
  // replay: the replay ends (as it does when the arm is touched), the
  // scene comes back, and the part stays in the hand.
  const leaveReplay = (names: string[]) => {
    const recorded = new Set(s.playback?.objects?.tracks.map((t) => t.name));
    if ((s.playing || s.overrideObstaclePoses !== null) && names.some((n) => recorded.has(n))) {
      s.stopPlayback();
    }
  };
  // What hangs on a hinge or runs on a slide is worked, not carried.
  const joint = jointOf(s.sceneryJoints, leaf);
  if (joint) {
    if (joint.objects.some((name) => inOtherHand.has(name))) return null;
    const worked: Worked = {
      kind: "joint",
      unit: joint.name,
      joint: joint.kind,
      at: jointAxisWorld(joint, joint.base !== null ? (byName.get(joint.base)?.pose ?? null) : null),
      range: joint.range,
      start: joint.position,
      obstacles: joint.objects.flatMap((name) => {
        const obstacle = byName.get(name);
        if (!obstacle) return [];
        const pose = shown(name, obstacle.pose);
        const from = new THREE.Matrix4().compose(
          new THREE.Vector3(...pose.position),
          new THREE.Quaternion(...pose.quaternion),
          new THREE.Vector3(1, 1, 1),
        );
        return [{ name, from }];
      }),
      asked: joint.position,
      hand: at.clone(),
    };
    leaveReplay(joint.objects);
    return worked;
  }
  const path = handUnit(leaf, s.selection, s.bomIndex);
  const members =
    path === null
      ? s.obstacles.filter((o) => o.name === leaf)
      : s.obstacles.filter((o) => inGroup(o.name, path) && !o.attached_to);
  if (members.length === 0 || members.some((o) => inOtherHand.has(o.name))) return null;
  const frames = path === null ? [] : s.frames.filter((f) => inGroup(f.name, path));
  const taken: Carried = {
    kind: "carry",
    unit: path ?? leaf,
    group: path !== null,
    obstacles: members.map((o) => ({ name: o.name, offset: holdOffset(grip, shown(o.name, o.pose)) })),
    frames: frames.map((f) => ({ name: f.name, offset: holdOffset(grip, f.pose) })),
  };
  leaveReplay(members.map((o) => o.name));
  return taken;
}

/** A tick in the hand: something was taken. */
function pulse(source: XRInputSource | undefined): void {
  const actuator = source?.gamepad?.hapticActuators?.[0] as
    | { pulse?: (value: number, ms: number) => Promise<unknown> }
    | undefined;
  actuator?.pulse?.(0.5, 40)?.catch(() => {});
}
