/**
 * How much of the timing chart the dock shows. A line's bake carries a
 * lane per robot and per signal — a transfer line with a dozen shuttles
 * and a zone per stop is a hundred of them — and stacked in full they push
 * the dock over the viewport it is supposed to sit under. So the lanes
 * come in groups the viewer can fold, sit in an area of bounded height
 * that scrolls, and can be hidden altogether down to the step bar; the
 * viewer's choices are remembered in the browser.
 */

/** The lane groups, in the order the dock stacks them. */
export type LaneGroup = "robots" | "signals" | "sensors" | "devices";

export const LANE_GROUPS: readonly LaneGroup[] = ["robots", "signals", "sensors", "devices"];

/** A baked signal lane's group, from the kind the bake gave it
 * (`"signal"` internal relay, `"sensor"` input, `"device"` output). */
export function signalGroup(kind: string): LaneGroup {
  if (kind === "device") return "devices";
  if (kind === "sensor") return "sensors";
  return "signals";
}

/** Sensor inputs fold on their own once a cell has more than this many:
 * a handful of photo-eyes read well next to the programs' signals, a zone
 * per stop of a transfer line does not. */
export const AUTO_FOLD_SENSORS = 12;

export interface LaneView {
  /** Lanes shown at all (hidden, the dock is the step bar alone). */
  open: boolean;
  /** Height cap of the lane area in pixels; `null` = the default share. */
  height: number | null;
  /** Groups the viewer folded or opened; the rest follow `isFolded`'s defaults. */
  folded: Partial<Record<LaneGroup, boolean>>;
}

export const DEFAULT_LANE_VIEW: LaneView = { open: true, height: null, folded: {} };

/** Is `group` (holding `count` lanes) folded? The viewer's choice when
 * there is one; else device outputs are, and sensors are when many. */
export function isFolded(view: LaneView, group: LaneGroup, count: number): boolean {
  const chosen = view.folded[group];
  if (chosen !== undefined) return chosen;
  if (group === "devices") return true;
  if (group === "sensors") return count > AUTO_FOLD_SENSORS;
  return false;
}

/** The view with `group` flipped from what it shows now. */
export function toggleGroup(view: LaneView, group: LaneGroup, count: number): LaneView {
  return { ...view, folded: { ...view.folded, [group]: !isFolded(view, group, count) } };
}

/** Smallest height the lane area can be dragged to: about two lanes. */
export const MIN_LANES_PX = 28;

/** A dragged height, kept between two lanes and most of the viewport. */
export function clampLanesHeight(px: number, viewportPx: number): number {
  const most = Math.max(MIN_LANES_PX, viewportPx * 0.8);
  return Math.round(Math.min(Math.max(px, MIN_LANES_PX), most));
}

/** A stored view, or the default when there is none or it does not parse. */
export function parseLaneView(text: string | null): LaneView {
  if (!text) return DEFAULT_LANE_VIEW;
  try {
    const raw = JSON.parse(text) as Record<string, unknown>;
    const folded: Partial<Record<LaneGroup, boolean>> = {};
    const rawFolded = (raw.folded ?? {}) as Record<string, unknown>;
    for (const group of LANE_GROUPS) {
      if (typeof rawFolded[group] === "boolean") folded[group] = rawFolded[group] as boolean;
    }
    const height =
      typeof raw.height === "number" && Number.isFinite(raw.height) && raw.height > 0
        ? raw.height
        : null;
    return { open: raw.open !== false, height, folded };
  } catch {
    return DEFAULT_LANE_VIEW;
  }
}

export function serializeLaneView(view: LaneView): string {
  return JSON.stringify(view);
}
