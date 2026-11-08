/** The slice of `navigator.xr` the probe uses (tests pass a stub). */
export type XrSystemLike = {
  isSessionSupported(mode: "immersive-vr"): Promise<boolean>;
};

/**
 * Whether an immersive-vr session could start here.
 *
 * Asked once, before the Canvas exists: the XR layer's antialiasing follows
 * the canvas's `antialias` attribute (three r169 `WebXRManager`), and the
 * studio otherwise creates its canvas without one because every desktop
 * pixel goes through the OutputPass instead. A slow, missing or failing
 * answer counts as "no" — the desktop path never waits on a headset runtime
 * (`.internal/design-vr.md` §3.2).
 */
export function probeXr(
  xr: XrSystemLike | undefined,
  timeoutMs = 300,
): Promise<boolean> {
  if (!xr || typeof xr.isSessionSupported !== "function") {
    return Promise.resolve(false);
  }
  return new Promise((resolve) => {
    const timer = setTimeout(() => resolve(false), timeoutMs);
    let answer: Promise<boolean>;
    try {
      answer = Promise.resolve(xr.isSessionSupported("immersive-vr"));
    } catch {
      answer = Promise.resolve(false);
    }
    answer
      .then(
        (supported) => resolve(supported === true),
        () => resolve(false),
      )
      .finally(() => clearTimeout(timer));
  });
}
