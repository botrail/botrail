import type { PlantCommandMsg, PlantHostMsg, PlantStatusMsg } from "./protocol";

/**
 * The live plant session as the dock and the I/O table read it: pure
 * helpers over the `plant` status message (`bt.vc.Session` publishes it
 * with every window of its stream), and the commands the controls send
 * back. No DOM, no store — tested in node.
 */

/** The realtime speeds the dock's speed button cycles through. */
export const SPEEDS = [0.1, 0.25, 0.5, 1, 2];

/** `0.25×`, `1×`. */
export function speedLabel(speed: number): string {
  return `${Number(speed.toFixed(2))}×`;
}

/** The speed after `speed` in the cycle (the next listed one, then round
 * to the slowest). */
export function nextSpeed(speed: number): number {
  const i = SPEEDS.findIndex((s) => s > speed + 1e-9);
  return i < 0 ? SPEEDS[0] : SPEEDS[i];
}

/** `PLC1 · modbus 127.0.0.1:5020 · 2037×` — a host's chip. */
export function hostChip(host: PlantHostMsg): string {
  const count = host.exchanges > 0 ? ` · ${host.exchanges}×` : "";
  return `${host.name} · ${host.link}${count}`;
}

/** The dock's caption for a live plant: what runs, how it keeps time. */
export function plantCaption(status: PlantStatusMsg): string {
  const parts = [status.label];
  if (status.pace === "realtime") {
    parts.push(`${speedLabel(status.speed)} realtime`);
    parts.push(`lag ${status.lag_ms.toFixed(1)} ms`);
    if (status.overruns > 0) parts.push(`${status.overruns} overruns`);
  } else {
    parts.push(status.pace);
  }
  parts.push(`${(status.dt * 1000).toFixed(0)} ms scan`);
  if (status.recording_from != null) {
    parts.push(`● rec ${(status.t - status.recording_from).toFixed(1)} s`);
  }
  return parts.join(" · ");
}

/** The level a lane is forced to, or `null` when it follows the world. */
export function forcedLevel(status: PlantStatusMsg | null, lane: string): boolean | null {
  const hit = status?.forced.find(([name]) => name === lane);
  return hit ? hit[1] : null;
}

/** The mark a lane's name wears for its breakpoints: `↑`, `↓`, both, or
 * none. */
export function breakMark(status: PlantStatusMsg | null, lane: string): string | null {
  const rising = status?.breaks.some((b) => b.lane === lane && b.rising) ?? false;
  const falling = status?.breaks.some((b) => b.lane === lane && !b.rising) ?? false;
  if (rising && falling) return "↕";
  if (rising) return "↑";
  if (falling) return "↓";
  return null;
}

/** The command a right-click on a lane sends: set the edge's breakpoint
 * when it is not there, clear it when it is. */
export function toggleBreak(
  status: PlantStatusMsg | null,
  lane: string,
  rising: boolean,
): PlantCommandMsg {
  const on = !(status?.breaks.some((b) => b.lane === lane && b.rising === rising) ?? false);
  return { command: "break", lane, rising, on };
}

/** Whether a sensor is a push button the viewport presses on a click. */
export function isButton(status: PlantStatusMsg | null, sensor: string): boolean {
  return status?.buttons.includes(sensor) ?? false;
}
