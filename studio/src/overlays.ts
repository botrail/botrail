/**
 * The analysis panel over the viewport has one slot: the SFC chart, the
 * ladder, the I/O table, the topology and the bill of materials are each
 * wide, and stacked they hid each other — so at most one is open.
 * Opening one closes the rest; closing one leaves the rest as they were.
 */
export type OverlayKind = "sfc" | "ld" | "io" | "topo" | "bom";

export const OVERLAY_KINDS: OverlayKind[] = ["sfc", "ld", "io", "topo", "bom"];

export type Overlays = {
  sfcOpen: boolean;
  ldOpen: boolean;
  ioOpen: boolean;
  topoOpen: boolean;
  bomOpen: boolean;
};

const FLAG: Record<OverlayKind, keyof Overlays> = {
  sfc: "sfcOpen",
  ld: "ldOpen",
  io: "ioOpen",
  topo: "topoOpen",
  bom: "bomOpen",
};

export const NO_OVERLAY: Overlays = {
  sfcOpen: false,
  ldOpen: false,
  ioOpen: false,
  topoOpen: false,
  bomOpen: false,
};

/** The overlays after `kind` is opened (alone) or closed (the others stay). */
export function setOverlay(current: Overlays, kind: OverlayKind, open: boolean): Overlays {
  if (open) return { ...NO_OVERLAY, [FLAG[kind]]: true };
  return { ...current, [FLAG[kind]]: false };
}

/** Which overlay is open, if any. */
export function openOverlay(o: Overlays): OverlayKind | null {
  for (const kind of OVERLAY_KINDS) if (o[FLAG[kind]]) return kind;
  return null;
}

/** Stored flags may say several were open (each was once its own switch):
 * the first in `precedence` wins. */
export function reconcileOverlays(flags: Overlays, precedence: OverlayKind[] = ["bom", "ld", "topo", "io", "sfc"]): Overlays {
  for (const kind of precedence) if (flags[FLAG[kind]]) return { ...NO_OVERLAY, [FLAG[kind]]: true };
  return { ...NO_OVERLAY };
}
