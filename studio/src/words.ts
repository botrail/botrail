import type { AnalogMsg, IoPointMsg, PlantStatusMsg, WordDefMsg } from "./protocol";

/** A word lane, as much of it as is read here. */
export type WordLane = { name: string; times: number[]; values: number[]; integer: boolean };
import { numberText } from "./seqLabels.ts";
import { wordValue } from "./sfc.ts";

/**
 * Words and analog inputs as the tables and the tags read them
 * (design-plc-resolution.md PF2): a word lane's value at the playhead, an
 * analog input's loop — what the transmitter measures → the loop current
 * → the module's counts → the value the controller scales them to — and
 * what a hand may do to a word on a live plant. Pure: tested in node. The
 * client twin of `rollout::analog_at`.
 */

/** The transmitter's measuring band: below it the loop is open. */
const MA_FLOOR = 3.8;

export type AnalogReading = {
  name: string;
  unit: string;
  /** The word lane the transmitter measures, and its value now. */
  source: string;
  measured: number | null;
  /** Loop current, mA — as the counts imply it (0 on an open loop). */
  current: number;
  /** The module's counts (`<name>.raw`). */
  raw: number;
  /** The value programs read (`<name>`). */
  scaled: number;
  open: boolean;
};

function lane(words: WordLane[] | undefined, name: string): WordLane | undefined {
  return words?.find((w) => w.name === name);
}

/** One analog input's loop at `t`, or `null` when the timeline carries
 * no counts for it (nothing baked yet). */
export function analogReading(
  analog: AnalogMsg,
  words: WordLane[] | undefined,
  t: number,
): AnalogReading | null {
  const rawLane = lane(words, `${analog.name}.raw`);
  if (!rawLane) return null;
  const raw = Math.round(wordValue(rawLane, t));
  // (ts-rs types an i64 as bigint; on the wire it is a JSON number.)
  const [lo, hi] = [Number(analog.raw[0]), Number(analog.raw[1])];
  const current = 4 + ((raw - lo) * 16) / (hi - lo);
  const scaledLane = lane(words, analog.name);
  const sourceLane = lane(words, analog.source);
  return {
    name: analog.name,
    unit: analog.unit,
    source: analog.source,
    measured: sourceLane ? wordValue(sourceLane, t) : null,
    current,
    raw,
    scaled: scaledLane ? wordValue(scaledLane, t) : 0,
    open: current < MA_FLOOR - 1e-9,
  };
}

/** `54.5 kPa · 12.72 mA · 2180` — the loop on one line; an open loop
 * says so. */
export function analogText(r: AnalogReading): string {
  const unit = r.unit ? ` ${r.unit}` : "";
  const head = `${numberText(Number(r.scaled.toFixed(2)))}${unit}`;
  if (r.open) return `${head} · open (${numberText(Number(r.current.toFixed(2)))} mA) · ${r.raw}`;
  return `${head} · ${r.current.toFixed(2)} mA · ${r.raw}`;
}

/** A word's value as text: an INT whole, a REAL to three places at most. */
export function wordText(value: number, integer: boolean): string {
  return integer ? String(Math.round(value)) : numberText(Number(value.toFixed(3)));
}

/** What a point shows in the table's live column when it is a word: an
 * analog input its loop, any other word its value. `null` for a point
 * that is no word (or has no lane yet). */
export function wordLive(
  p: IoPointMsg,
  words: WordLane[] | undefined,
  analogs: AnalogMsg[],
  t: number,
): string | null {
  const analog = p.aspect ? undefined : analogs.find((a) => a.name === p.name);
  if (analog) {
    const r = analogReading(analog, words, t);
    return r ? analogText(r) : null;
  }
  const w = lane(words, p.label);
  return w ? wordText(wordValue(w, t), w.integer) : null;
}

/** What a hand may do to a point's word on a live plant: `write` a
 * defined word (a setpoint keyed in), `force` any word lane (on an analog
 * input: what the transmitter measures). */
export function wordHands(
  p: IoPointMsg,
  defs: WordDefMsg[],
  analogs: AnalogMsg[],
  words: WordLane[] | undefined,
): { write: boolean; force: boolean } {
  if (p.aspect) return { write: false, force: !!lane(words, p.label) };
  const defined = defs.some((d) => d.name === p.name);
  const analog = analogs.some((a) => a.name === p.name);
  return { write: defined, force: defined || analog };
}

/** The value a word lane is forced to, or `null`. */
export function forcedWord(status: PlantStatusMsg | null, name: string): number | null {
  const hit = status?.forced_words?.find(([lane]) => lane === name);
  return hit ? hit[1] : null;
}
