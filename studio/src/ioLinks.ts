import type { ActionMsg, ConditionMsg, IoPointMsg, StepMsg, StepRefMsg } from "./protocol";
import { flattenSteps } from "./sfc.ts";

/**
 * The links between a program's contacts and coils and the I/O points
 * they read and write — what lets the ladder, the SFC chart, the I/O
 * table, the timing chart and the 3D tags name one point by one address
 * and light it up together. Pure: no store, no DOM; tested in node.
 *
 * A point's readers and writers carry the steps that use it (the I/O
 * map's derivation), so a contact at step `flat` of `sequence` resolves
 * to exactly the point that step reads — on the host the program runs on
 * — not to a same-named point on another controller.
 */

/** The address a point wears in a PLC's tables: its channel's address
 * (`X20`, `%IX100.1`), else `node.channel`, else nothing (unbound). */
export function addressOf(p: IoPointMsg): string | null {
  if (p.address) return p.address;
  if (p.node && p.channel) return `${p.node}.${p.channel}`;
  return null;
}

/** The key a point is highlighted by everywhere: its label (`beam`,
 * `feed.encoder`, `arm.done`) — its lane's name when it has a lane. */
export function pointKey(p: IoPointMsg): string {
  return p.label;
}

function at(refs: StepRefMsg[], sequence: string, flat: number): boolean {
  return refs.some((r) => r.sequence === sequence && r.index === flat);
}

/** The stop a coil-driven axis was last sent to when step `flat` waits on
 * it: the step's own `move_to` first, then the steps before it in flat
 * order — the chain that led here (the ST export tracks it the same way). */
export function lastStop(steps: StepMsg[], flat: number, device: string): string | null {
  const nodes = flattenSteps(steps);
  for (let i = Math.min(flat, nodes.length - 1); i >= 0; i--) {
    const actions = nodes[i].step.actions;
    for (let j = actions.length - 1; j >= 0; j--) {
      const a = actions[j];
      if (a.type === "device" && a.device === device && a.command.type === "move_to_stop") {
        return a.command.stop;
      }
    }
  }
  return null;
}

/** The point a condition leaf reads at step `flat` of `sequence`, or
 * `null` when the leaf is no I/O (a timer, an internal relay, a robot
 * the program's own controller runs). `steps` (the program's authored
 * steps) names a cylinder's in-position: the limit switch of the stop it
 * was last sent to. */
export function contactPoint(
  points: IoPointMsg[],
  sequence: string,
  flat: number,
  cond: ConditionMsg,
  steps?: StepMsg[],
): IoPointMsg | null {
  const reads = (p: IoPointMsg) => at(p.readers, sequence, flat);
  if (cond.type === "not") return contactPoint(points, sequence, flat, cond.condition, steps);
  switch (cond.type) {
    case "signal":
    case "rising":
    case "falling":
      return points.find((p) => reads(p) && p.name === cond.name && !p.aspect) ?? null;
    case "device_done": {
      const done = points.find(
        (p) => reads(p) && p.name === cond.device && p.source === "device:done",
      );
      if (done) return done;
      // A coil-driven axis reports in-position through its limit switches.
      const stop = steps ? lastStop(steps, flat, cond.device) : null;
      return stop
        ? points.find((p) => reads(p) && p.name === `${cond.device}/${stop}`) ?? null
        : null;
    }
    case "robot_done":
    case "group_done":
      return (
        points.find((p) => reads(p) && p.name === cond.robot && p.aspect === "done") ?? null
      );
    case "done":
      // The step's own moves: the done handshake of the robot it started.
      return points.find((p) => reads(p) && p.source === "robot:done") ?? null;
    case "compare": {
      // The word the comparison reads: an analog input's scaled value and
      // its counts (`dp`, `dp.raw`) are one point.
      const words = [cond.left, cond.right].filter((o): o is string => typeof o === "string");
      return (
        points.find(
          (p) => reads(p) && words.some((w) => p.label === w || `${p.name}.raw` === w),
        ) ?? null
      );
    }
    default:
      return null;
  }
}

/** The key a condition leaf highlights by when it reads no point: the
 * lane it tests (`beam`), the device's lane (`conv`), the robot's done
 * (`arm.done`). */
export function leafKey(cond: ConditionMsg): string | null {
  switch (cond.type) {
    case "signal":
    case "rising":
    case "falling":
      return cond.name;
    case "device_done":
      return cond.device;
    case "robot_done":
    case "group_done":
      return `${cond.robot}.done`;
    case "not":
      return leafKey(cond.condition);
    case "compare": {
      // The lane it compares: the first word operand (an analog input's
      // counts highlight with the input).
      const w = [cond.left, cond.right].find((o): o is string => typeof o === "string");
      return w ? (w.endsWith(".raw") ? w.slice(0, -4) : w) : null;
    }
    default:
      return null;
  }
}

/** The point a rung's output writes at flat index `flat` (the program's
 * steps, then its rungs): the coil's target by name. */
export function outputPoint(
  points: IoPointMsg[],
  sequence: string,
  flat: number,
  target: string,
): IoPointMsg | null {
  return (
    points.find((p) => at(p.writers, sequence, flat) && p.name === target && !p.aspect) ?? null
  );
}

/** What a rung's coil shows: its address with the target as the comment,
 * or the target alone. */
export function nameOutput(
  points: IoPointMsg[],
  sequence: string,
  flat: number,
  target: string,
  label: string,
): Named {
  const p = outputPoint(points, sequence, flat, target);
  const address = p ? addressOf(p) : null;
  if (!address) return { label, key: target, address: null };
  return { label: address, note: label, key: p ? pointKey(p) : target, address };
}

/** The points an entry action at step `flat` writes: a `set` its signal,
 * a device command the device's coils and words, a move the start of the
 * robot `robotOf` names (or of whichever robot the step starts). */
export function actionPoints(
  points: IoPointMsg[],
  sequence: string,
  flat: number,
  action: ActionMsg,
  robotOf?: (motion: string) => string | null,
): IoPointMsg[] {
  const writes = (p: IoPointMsg) => at(p.writers, sequence, flat);
  switch (action.type) {
    case "set":
      return points.filter((p) => writes(p) && p.name === action.signal && !p.aspect);
    case "set_word":
      return points.filter((p) => writes(p) && p.name === action.word && !p.aspect);
    case "device":
      return points.filter(
        (p) =>
          writes(p) &&
          (p.name === action.device || p.name.startsWith(`${action.device}/`)),
      );
    case "start_motion": {
      const robot = robotOf?.(action.motion) ?? null;
      return points.filter(
        (p) => writes(p) && p.source === "robot:start" && (robot === null || p.name === robot),
      );
    }
    case "start_ramp":
    case "start_toolpath":
    case "run_program":
    case "policy":
    case "crouch":
      return points.filter(
        (p) =>
          writes(p) &&
          p.source === "robot:start" &&
          (!("robot" in action) || !action.robot || p.name === action.robot),
      );
    default:
      return [];
  }
}

/** What a contact or a coil shows: the address on the wire and the name
 * as its comment, or the name alone when the point is unbound or there
 * is none. `key` is what it lights up with. */
export type Named = { label: string; note?: string; key: string | null; address: string | null };

export function nameContact(
  points: IoPointMsg[],
  sequence: string,
  flat: number,
  cond: ConditionMsg,
  label: string,
  note?: string,
  steps?: StepMsg[],
): Named {
  const p = contactPoint(points, sequence, flat, cond, steps);
  const address = p ? addressOf(p) : null;
  const key = p ? pointKey(p) : leafKey(cond);
  if (!address) return { label, note, key, address: null };
  // The edge letter / the NC slash stay on the glyph; the comment under it
  // is the point's name.
  return { label: address, note: label, key, address };
}

export function nameCoil(
  points: IoPointMsg[],
  sequence: string,
  flat: number,
  action: ActionMsg,
  label: string,
  robotOf?: (motion: string) => string | null,
): Named {
  const ps = actionPoints(points, sequence, flat, action, robotOf);
  const bound = ps.find((p) => addressOf(p) !== null) ?? null;
  const key = (bound ?? ps[0])
    ? pointKey((bound ?? ps[0])!)
    : action.type === "set"
      ? action.signal
      : action.type === "set_word"
        ? action.word
        : null;
  if (!bound) return { label, key, address: null };
  const address = addressOf(bound)!;
  const more = ps.filter((p) => addressOf(p) !== null).length - 1;
  return { label: more > 0 ? `${address} +${more}` : address, note: label, key, address };
}
