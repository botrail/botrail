import type { ActionMsg, ConditionMsg, OperandMsg, WordExprMsg } from "./protocol";

/** A number as the tables print it: whole values bare, the rest trimmed. */
export function numberText(x: number): string {
  if (Number.isInteger(x)) return String(x);
  return String(Number(x.toFixed(4)));
}

/** A word operand: its name, or the constant. */
export function operandText(o: OperandMsg): string {
  return typeof o === "number" ? numberText(o) : o;
}

/** A word expression: `a`, `a + b`. */
export function wordExprText(v: WordExprMsg): string {
  return v.op && v.right !== undefined && v.right !== null
    ? `${operandText(v.left)} ${v.op} ${operandText(v.right)}`
    : operandText(v.left);
}

/** An NC run's name: the program, and its block where it runs by block
 * (the name the run has on its machine's lane). */
export function ncRun(action: { program: string; block?: number | null }): string {
  return action.block == null ? action.program : `${action.program}#${action.block}`;
}

/** Compact label for an action chip (step rows, SFC chart boxes). */
export function actionLabel(action: ActionMsg): string {
  const short = (name: string) => name.split("/").filter(Boolean).pop() ?? name;
  switch (action.type) {
    case "start_motion":
      return `▶ ${action.motion}`;
    case "start_toolpath":
      return `⟿ ${action.toolpath}`;
    case "run_program":
      return `NC ${ncRun(action)}`;
    case "start_ramp":
      return `ramp ${action.targets.length}j`;
    case "policy":
      return `policy ${action.policy}`;
    case "attach":
      return `⊕ ${short(action.object)}`;
    case "detach":
      return `⊖ ${short(action.object)}`;
    case "track":
      return `⇉ ${short(action.object)}`;
    case "untrack":
      return "⇥ untrack";
    case "set":
      return `${action.signal}=${action.value ? "1" : "0"}`;
    case "set_word":
      return `${action.word}←${wordExprText(action.value)}`;
    case "crouch":
      return action.depth === 0 && !action.lean ? "⇡ stand" : `⇣ ${action.depth}m`;
    case "device": {
      const cmd = action.command;
      const verb =
        cmd.type === "set_speed"
          ? `speed ${cmd.speed}`
          : cmd.type === "move_to"
            ? `→${cmd.position}`
            : cmd.type === "advance"
              ? `⊳ ${cmd.distance}m`
              : cmd.type;
      return `⚙ ${action.device} ${verb}`;
    }
  }
}

/** Compact label for a transition condition (PLC vocabulary: ↑/↓ edges,
 * `&`/`|` contacts). */
export function conditionLabel(condition: ConditionMsg): string {
  switch (condition.type) {
    case "immediately":
      return "→";
    case "done":
      return "done";
    case "robot_done":
      return `${condition.robot} done`;
    case "group_done":
      return `${condition.robot}/${condition.group} done`;
    case "elapsed":
      return `${condition.seconds.toFixed(2)}s`;
    case "signal":
      return `${condition.name}=${condition.value ? "1" : "0"}`;
    case "rising":
      return `↑${condition.name}`;
    case "falling":
      return `↓${condition.name}`;
    case "all":
      return condition.conditions.map(conditionLabel).join(" & ");
    case "any":
      return condition.conditions.map(conditionLabel).join(" | ");
    case "device_done":
      return `${condition.device} done`;
    case "not": {
      const c = condition.condition;
      return c.type === "signal"
        ? `${c.name}=${c.value ? "0" : "1"}`
        : `!(${conditionLabel(c)})`;
    }
    case "step_active":
      return `[${condition.sequence}/${condition.step}]`;
    case "compare":
      return `${operandText(condition.left)}${condition.op}${operandText(condition.right)}`;
  }
}
