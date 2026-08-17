import { useEffect } from "react";

/**
 * Keeps `--sfc-reserve` on `panel` at the timeline dock's height (plus its
 * margins), so a viewport overlay never grows over the transport bar. The
 * dock's height is the cell's business — one lane per signal — so the room
 * to leave for it is measured, not assumed. `docked` re-arms the observer
 * when the dock appears or goes.
 */
export function useDockReserve(panel: HTMLDivElement | null, docked: boolean): void {
  useEffect(() => {
    if (!panel) return;
    const dock = document.querySelector(".timeline-dock");
    if (!dock) {
      panel.style.removeProperty("--sfc-reserve");
      return;
    }
    const fit = () => {
      panel.style.setProperty("--sfc-reserve", `${dock.clientHeight + 34}px`);
    };
    fit();
    const observer = new ResizeObserver(fit);
    observer.observe(dock);
    return () => observer.disconnect();
  }, [panel, docked]);
}
