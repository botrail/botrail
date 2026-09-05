import type { ActionMsg, ConditionMsg } from "./protocol";

/** Compact label for an action chip (step rows, SFC chart boxes). */
export function actionLabel(action: ActionMsg): string {
  const short = (name: string) => name.split("/").filter(Boolean).pop() ?? name;
  switch (action.type) {
    case "start_motion":
      return `▶ ${action.motion}`;
    case "start_toolpath":
      return `⟿ ${action.toolpath}`;
    case "start_ramp":
      return `ramp ${action.targets.length}j`;
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
  }
}
