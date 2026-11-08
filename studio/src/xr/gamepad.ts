/**
 * Controller buttons and thumbsticks as edges. WebXR has no button events
 * beyond select / squeeze, so the locomotion polls `inputSource.gamepad`
 * once per XR frame and turns the raw state into "went down" and "pushed
 * past / came back" moments here. The raw gamepad is read directly rather
 * than through the profile-derived component states so the buttons work
 * when the controller profile cannot be fetched (an offline cell).
 *
 * Pure: no three, no React — fixed by `tests/xr_gamepad.test.mjs`.
 */

/** The `xr-standard` gamepad mapping (WebXR Gamepads Module): button and
 * axis indices every tracked controller follows. `primary` is A on the
 * right hand and X on the left, `secondary` B / Y. */
export const XR_STANDARD = {
  trigger: 0,
  squeeze: 1,
  touchpad: 2,
  thumbstick: 3,
  primary: 4,
  secondary: 5,
} as const;

/** The thumbstick's axes; forward is negative Y. */
export const THUMBSTICK_AXES = { x: 2, y: 3 } as const;

export type PadSnapshot = { pressed: boolean[]; axes: number[] };

type GamepadLike = {
  buttons: ArrayLike<{ pressed: boolean }>;
  axes: ArrayLike<number>;
};

/** A plain copy of the gamepad's state — what the detectors below consume. */
export function readPad(gamepad: GamepadLike | null | undefined): PadSnapshot {
  if (!gamepad) return { pressed: [], axes: [] };
  const pressed: boolean[] = [];
  for (let i = 0; i < gamepad.buttons.length; i++) pressed.push(gamepad.buttons[i].pressed);
  const axes: number[] = [];
  for (let i = 0; i < gamepad.axes.length; i++) axes.push(gamepad.axes[i]);
  return { pressed, axes };
}

/** Buttons that went down since the previous update, by index. */
export class ButtonEdges {
  private last: boolean[] = [];

  update(pressed: readonly boolean[]): number[] {
    const down: number[] = [];
    for (let i = 0; i < pressed.length; i++) {
      if (pressed[i] && !this.last[i]) down.push(i);
    }
    this.last = Array.from(pressed);
    return down;
  }

  reset(): void {
    this.last = [];
  }
}

/** What one `feed` reports: `engaged` is set on the frame the stick
 * crosses the engage threshold (±1 for the direction), `released` on the
 * frame it returns inside the release band afterwards (the direction it
 * had been held in), `held` is the direction it is currently past the
 * threshold in. A snap turn fires on `engaged`; a teleport aims while
 * `held` and fires on `released`. */
export type LatchEvent = { engaged: -1 | 0 | 1; released: -1 | 0 | 1; held: -1 | 0 | 1 };

/** A thumbstick axis with hysteresis: engages past `engage`, re-arms only
 * once back within `release`, so a stick resting near the threshold does
 * not fire twice. */
export class AxisLatch {
  private state: -1 | 0 | 1 = 0;
  private readonly engage: number;
  private readonly release: number;

  // (No parameter properties: node's type stripping runs these tests.)
  constructor(engage = 0.7, release = 0.3) {
    this.engage = engage;
    this.release = release;
  }

  feed(value: number): LatchEvent {
    const v = Number.isFinite(value) ? value : 0;
    if (this.state === 0) {
      if (Math.abs(v) >= this.engage) {
        this.state = v > 0 ? 1 : -1;
        return { engaged: this.state, released: 0, held: this.state };
      }
      return { engaged: 0, released: 0, held: 0 };
    }
    if (Math.abs(v) <= this.release) {
      const was = this.state;
      this.state = 0;
      return { engaged: 0, released: was, held: 0 };
    }
    return { engaged: 0, released: 0, held: this.state };
  }

  get held(): -1 | 0 | 1 {
    return this.state;
  }

  reset(): void {
    this.state = 0;
  }
}
