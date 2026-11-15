import type { DeviationMsg, PlantStatusMsg, ReadingsMsg } from "./protocol";

/**
 * A controller's own I/O image beside the model's lanes: what a replayed
 * machine log or a read-only monitor (`bt.vc.monitor`) read, and where it
 * parted from the model. Pure helpers over the `readings` a finished
 * timeline carries and a monitor's status streams — no DOM, no store.
 */

/** The readings on show: the live monitor's while it streams, else the
 * loaded timeline's. */
export function readingsOf(
  plant: PlantStatusMsg | null,
  timeline: { readings?: ReadingsMsg | null } | null,
): ReadingsMsg | null {
  return plant?.readings ?? timeline?.readings ?? null;
}

/** The read level of a lane at `t`, or `null` when it was not read (yet). */
export function readLevel(r: ReadingsMsg | null, lane: string, t: number): boolean | null {
  const track = r?.signals.find((s) => s.name === lane);
  if (!track) return null;
  let v: boolean | null = null;
  for (let i = 0; i < track.times.length && track.times[i] <= t + 1e-9; i++) v = track.values[i];
  return v;
}

/** The read value of a word lane at `t`, or `null`. */
export function readNumber(r: ReadingsMsg | null, lane: string, t: number): number | null {
  const track = r?.words?.find((w) => w.name === lane);
  if (!track) return null;
  let v: number | null = null;
  for (let i = 0; i < track.times.length && track.times[i] <= t + 1e-9; i++) v = track.values[i];
  return v;
}

/** The deviations of one lane, in time order. */
export function deviationsOf(r: ReadingsMsg | null, lane: string): DeviationMsg[] {
  return r ? r.deviations.filter((d) => d.lane === lane) : [];
}

/** The deviation `lane` is in at `t`, if any. */
export function deviationAt(r: ReadingsMsg | null, lane: string, t: number): DeviationMsg | null {
  return r?.deviations.find((d) => d.lane === lane && d.start - 1e-9 <= t && t <= d.end + 1e-9) ?? null;
}

/** The stretches a read lane is on, clipped to `[0, duration]`. */
export function readSpans(
  track: { times: number[]; values: boolean[] },
  duration: number,
): { start: number; end: number }[] {
  const out: { start: number; end: number }[] = [];
  track.times.forEach((t0, i) => {
    if (!track.values[i]) return;
    const start = Math.max(0, t0);
    const end = Math.min(duration, track.times[i + 1] ?? duration);
    if (end > start) out.push({ start, end });
  });
  return out;
}

/** `s2 late 0.50 s` — a deviation's chip. */
export function deviationChip(d: DeviationMsg): string {
  const span = d.end - d.start;
  return `${d.lane} ${d.kind} ${span.toFixed(2)} s`;
}

/** `log line_io.csv · 4 deviations` / `… · agrees with the model`. */
export function readingsCaption(r: ReadingsMsg): string {
  const n = r.deviations.length;
  const verdict = n === 0 ? "agrees with the model" : `${n} deviation${n === 1 ? "" : "s"}`;
  return `${r.host} read from ${r.source} · ±${r.tolerance.toFixed(2)} s · ${verdict}`;
}
