import { useStudioStore } from "../store";
import type { HoverTarget } from "../hoverChip";

/**
 * Pointer-over bookkeeping for the hover chip, next to the cursor
 * feedback in `cursor.ts`. Only enter and leave reach the store — the
 * chip's position follows the pointer imperatively — and a leave only
 * clears the hover it entered, so overlapping bodies (a link in front of
 * a crate) hand over cleanly.
 */
export function hoverEnter(kind: HoverTarget["kind"], name: string): void {
  const s = useStudioStore.getState();
  if (s.hover?.kind !== kind || s.hover.name !== name) s.setHover({ kind, name });
}

export function hoverLeave(kind: HoverTarget["kind"], name: string): void {
  const s = useStudioStore.getState();
  if (s.hover && s.hover.kind === kind && s.hover.name === name) s.setHover(null);
}
