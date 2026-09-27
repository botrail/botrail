import type { PartTargetKind } from "./protocol";
import type { Selection } from "./store";

/**
 * The chip that names what the pointer is over in the viewport: `kind ·
 * name · identity`. Pure rules — when it shows and where it sits — so
 * the component only wires pointer events to them.
 */

/** How long the pointer must rest on a body before the chip appears —
 * sweeping across a cell must not flicker through every part. */
export const HOVER_DELAY_MS = 120;

/** The chip's offset from the pointer, right and below. */
export const CHIP_OFFSET = { x: 14, y: 18 };

export interface HoverTarget {
  kind: PartTargetKind;
  name: string;
}

/** The selection already says what the selected body is (the focus card),
 * so the chip stays off it: the selected obstacle, any prim of the
 * selected group, the selected robot, camera or scanner. */
export function hoverMatchesSelection(hover: HoverTarget, selection: Selection): boolean {
  switch (selection.type) {
    case "obstacle":
      return hover.kind === "obstacle" && hover.name === selection.name;
    case "group": {
      const bare = (s: string) => s.replace(/^\/+/, "");
      return hover.kind === "obstacle" && bare(hover.name).startsWith(`${bare(selection.path)}/`);
    }
    case "tcp":
    case "robot":
      return hover.kind === "robot" && hover.name === selection.robot;
    case "camera":
    case "lidar":
      return hover.kind === selection.type && hover.name === selection.name;
    default:
      return false;
  }
}

/** Whether the chip shows: something is hovered, it is not the selection,
 * the pointer has rested long enough, and no button is down (an orbit, a
 * gizmo drag or a body taken in hand is not a look). */
export function chipVisible(args: {
  hover: HoverTarget | null;
  selection: Selection;
  restedMs: number;
  buttons: number;
}): boolean {
  const { hover, selection, restedMs, buttons } = args;
  if (!hover || buttons !== 0 || restedMs < HOVER_DELAY_MS) return false;
  return !hoverMatchesSelection(hover, selection);
}

/** Where the chip sits for a pointer at `pointer` inside a viewport of
 * `view` size: right-below by default, flipped left / up near the edges. */
export function chipPlacement(
  pointer: { x: number; y: number },
  chip: { w: number; h: number },
  view: { w: number; h: number },
): { left: number; top: number } {
  let left = pointer.x + CHIP_OFFSET.x;
  let top = pointer.y + CHIP_OFFSET.y;
  if (left + chip.w > view.w) left = Math.max(0, pointer.x - CHIP_OFFSET.x - chip.w);
  if (top + chip.h > view.h) top = Math.max(0, pointer.y - CHIP_OFFSET.y - chip.h);
  return { left, top };
}

/** `sensor · gate_curtain · キーエンス GL-R22L`; a body with no line on the
 * bill (`product` null) reads as kind and name alone. */
export function hoverLabel(hover: HoverTarget, product: string | null): string {
  const base = `${hover.kind} · ${hover.name}`;
  return product ? `${base} · ${product}` : base;
}
