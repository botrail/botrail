/**
 * The left-hand panel's layout and words (`.internal/design-vr.md` §3.7):
 * one title line (the playhead) over a row of five buttons. Pure — the
 * canvas drawing and the hit test in `XrPanel.tsx` are a few lines over
 * these rectangles. Fixed by `tests/xr_panel.test.mjs`.
 */

export type PanelAction = "play" | "speed" | "waypoint" | "recenter" | "exit";

export const PANEL_ACTIONS: readonly PanelAction[] = ["play", "speed", "waypoint", "recenter", "exit"];

/** The quad on the hand, metres; the canvas keeps its 2.5:1 aspect. */
export const PANEL_SIZE = { width: 0.2, height: 0.08 } as const;

/** Canvas pixels behind the quad. */
export const PANEL_PX = { width: 640, height: 256 } as const;

export type PanelState = {
  hasPlayback: boolean;
  playing: boolean;
  speed: number;
  /** Whether a robot is focused — something to teach a point to. */
  canTeach: boolean;
  /** Playhead and bake length, seconds (null without a playback). */
  time: number | null;
  duration: number | null;
};

/** A rectangle in texture coordinates: origin bottom-left, 0..1, as
 * three's plane UVs run. */
export type UvRect = { x: number; y: number; w: number; h: number };

const BUTTON_ROW = { y: 0.06, h: 0.5 } as const;
const BUTTON_GAP = 0.025;

/** The i-th button's rectangle (UV). */
export function buttonRect(index: number): UvRect {
  const n = PANEL_ACTIONS.length;
  const inner = 1 - BUTTON_GAP * (n + 1);
  const w = inner / n;
  return { x: BUTTON_GAP + index * (w + BUTTON_GAP), y: BUTTON_ROW.y, w, h: BUTTON_ROW.h };
}

/** Which button a UV hit lands on, if any. */
export function hitButton(uv: { x: number; y: number }): PanelAction | null {
  for (let i = 0; i < PANEL_ACTIONS.length; i++) {
    const r = buttonRect(i);
    if (uv.x >= r.x && uv.x <= r.x + r.w && uv.y >= r.y && uv.y <= r.y + r.h) {
      return PANEL_ACTIONS[i];
    }
  }
  return null;
}

/** A button's centre (UV) — what a script aims a controller at. */
export function buttonCenter(action: PanelAction): { x: number; y: number } {
  const r = buttonRect(PANEL_ACTIONS.indexOf(action));
  return { x: r.x + r.w / 2, y: r.y + r.h / 2 };
}

/** The speed after one press: the dock's rule (1 → 2 → 4 → 8 → 1). */
export function nextSpeed(speed: number): number {
  return speed >= 8 ? 1 : speed * 2;
}

export function buttonLabel(action: PanelAction, state: PanelState): string {
  switch (action) {
    case "play":
      return !state.hasPlayback ? "▶ —" : state.playing ? "❚❚ pause" : "▶ play";
    case "speed":
      return `${state.speed}×`;
    case "waypoint":
      return state.canTeach ? "+ point" : "+ —";
    case "recenter":
      return "⟲ center";
    case "exit":
      return "✕ exit";
  }
}

/** The title line: the playhead over the bake, or why there is none. */
export function panelTitle(state: PanelState): string {
  if (!state.hasPlayback || state.time === null || state.duration === null) {
    return "no playback — bake a cycle on the desktop";
  }
  return `${state.time.toFixed(1)} / ${state.duration.toFixed(1)} s`;
}

/** One string for the whole panel — what decides whether to redraw. */
export function panelText(state: PanelState): string {
  return [panelTitle(state), ...PANEL_ACTIONS.map((a) => buttonLabel(a, state))].join("\n");
}
