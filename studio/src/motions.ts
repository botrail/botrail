import type { ConstraintMsg, MotionMsg, SegmentKindMsg, SegmentMsg } from "./protocol";

/**
 * Which motion a new waypoint lands in, and what the waypoint is — shared
 * by the MOTION panel and the headset (left Y / the panel's `+ point`),
 * so both teach into the same motion with the same options. Pure; fixed
 * by `tests/motions.test.mjs`.
 */

/** "Upright" keeps the TCP's local +Z within a 30° cone of world +Z. */
export const UPRIGHT_CONE: ConstraintMsg = {
  type: "orientation_cone",
  axis_local: [0, 0, 1],
  axis_world: [0, 0, 1],
  angle: (Math.PI / 180) * 30,
};

/** The conventional first motion name: `main` for the first robot,
 * `main_<robot>` for the rest, with `_<arm>` appended on a dual-arm robot
 * (`main_left`). */
export function conventionalName(
  robotName: string,
  firstRobot: string | undefined,
  arm: string | null,
): string {
  const base = robotName === firstRobot ? "main" : `main_${robotName}`;
  return arm === null ? base : `${base}_${arm}`;
}

/** The conventional name when free; when taken, `motion_2`, `motion_3`,
 * … — whatever is free scene-wide. */
export function freshMotionName(
  motions: MotionMsg[],
  robotName: string,
  firstRobot: string | undefined,
  arm: string | null,
): string {
  const taken = new Set(motions.map((m) => m.name));
  const base = conventionalName(robotName, firstRobot, arm);
  if (!taken.has(base)) return base;
  for (let i = 2; ; i++) {
    const candidate = `motion_${i}`;
    if (!taken.has(candidate)) return candidate;
  }
}

export type MotionRobot = {
  name: string;
  /** Whether the robot has arms to tell apart (several planning groups). */
  dualArm: boolean;
  /** The arm the panels drive (null on a single-arm robot). */
  selectedGroup: string | null;
};

export type MotionTarget = {
  /** The motion a waypoint lands in (created on the first one if new). */
  motionName: string;
  /** The arm the motion drives; null on a single-arm robot. */
  arm: string | null;
  /** The motion as it exists, or null when it is yet to be created. */
  motion: MotionMsg | null;
};

/**
 * The edit target: the picked motion, else the robot's (arm's) first,
 * else the conventional fresh name — created on the first waypoint, the
 * same implicit create the server does.
 */
export function motionTarget(
  motions: MotionMsg[],
  selectedMotion: string | null,
  robot: MotionRobot,
  firstRobot: string | undefined,
): MotionTarget {
  const arm = robot.dualArm ? robot.selectedGroup : null;
  const owned = motions.filter(
    (m) =>
      (m.robot ?? firstRobot) === robot.name &&
      (!robot.dualArm || (m.group ?? null) === arm),
  );
  const fallbackName = owned[0]?.name ?? conventionalName(robot.name, firstRobot, arm);
  const motionName = selectedMotion ?? fallbackName;
  return { motionName, arm, motion: motions.find((m) => m.name === motionName) ?? null };
}

/** The waypoint: the robot's current configuration as a goal. */
export function waypointSegment(
  jointPositions: readonly number[],
  kind: SegmentKindMsg,
  upright: boolean,
): SegmentMsg {
  return {
    kind,
    goal_positions: jointPositions.slice(),
    constraints: upright ? [UPRIGHT_CONE] : [],
  };
}
