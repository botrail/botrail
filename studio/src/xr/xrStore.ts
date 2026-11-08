import { type XRStore, createXRStore } from "@react-three/xr";

/**
 * The one XR store (`.internal/design-vr.md` §3.1).
 *
 * `emulate: false` — on localhost the store would otherwise inject the IWER
 * emulator (and its dev UI) whenever WebXR is missing; the studio runs on
 * localhost all day. The headless check installs IWER itself, before any
 * page script. `offerSession: false` — the header's VR button is the one
 * way in. The grab and teleport pointers stay off: the grip is reserved
 * for the TCP handle (V2) and locomotion is the studio's own
 * (`XrLocomotion`). The ray pointer keeps the selection on the existing
 * click handlers, with a click window long enough for a trigger pull
 * (the default 300 ms is a mouse's). 72 Hz where the headset offers it:
 * the frame budget factory cells can hold.
 */
export const xrStore: XRStore = createXRStore({
  emulate: false,
  offerSession: false,
  controller: {
    grabPointer: false,
    teleportPointer: false,
    rayPointer: { clickThresholdMs: 1500 },
  },
  frameRate: (rates) => (Array.from(rates).includes(72) ? 72 : false),
});

export function enterVr(): Promise<XRSession | undefined> {
  return xrStore.enterVR();
}

export function exitVr(): void {
  xrStore.getState().session?.end();
}
