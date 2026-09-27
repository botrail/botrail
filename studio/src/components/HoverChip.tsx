import { useEffect, useRef, useState } from "react";

import { HOVER_DELAY_MS, chipPlacement, hoverLabel, hoverMatchesSelection } from "../hoverChip";
import { identityOf, identityText } from "../partIdentity";
import { useStudioStore } from "../store";

/**
 * The chip beside the pointer naming the body under it — `kind · name ·
 * product` — after the pointer rests a beat. Only enter and leave go
 * through the store; the position follows the pointer straight on the
 * element, and a pressed button (an orbit, a drag, a body in hand)
 * hides it. Off while the pointer is on the selection: the focus card
 * already says what that is.
 */
export function HoverChip({ viewport }: { viewport: HTMLDivElement | null }) {
  const hover = useStudioStore((s) => s.hover);
  const selection = useStudioStore((s) => s.selection);
  const index = useStudioStore((s) => s.bomIndex);
  const [shown, setShown] = useState(false);
  const ref = useRef<HTMLDivElement>(null);
  const pointer = useRef({ x: 0, y: 0, buttons: 0 });

  // The rest: a fresh hover starts the clock, a leave (or the selection
  // catching up with it) stops it.
  useEffect(() => {
    setShown(false);
    if (!hover || hoverMatchesSelection(hover, selection)) return;
    const timer = window.setTimeout(() => setShown(true), HOVER_DELAY_MS);
    return () => window.clearTimeout(timer);
  }, [hover, selection]);

  // The pointer: position and buttons, written to the element, never to
  // React — this runs on every move.
  useEffect(() => {
    if (!viewport) return;
    const place = () => {
      const el = ref.current;
      if (!el) return;
      const rect = viewport.getBoundingClientRect();
      const { left, top } = chipPlacement(
        { x: pointer.current.x, y: pointer.current.y },
        { w: el.offsetWidth, h: el.offsetHeight },
        { w: rect.width, h: rect.height },
      );
      el.style.transform = `translate(${left}px, ${top}px)`;
      el.style.visibility = pointer.current.buttons === 0 ? "visible" : "hidden";
    };
    const onMove = (e: PointerEvent) => {
      const rect = viewport.getBoundingClientRect();
      pointer.current = { x: e.clientX - rect.left, y: e.clientY - rect.top, buttons: e.buttons };
      place();
    };
    const onButtons = (e: PointerEvent) => {
      pointer.current.buttons = e.buttons;
      place();
    };
    viewport.addEventListener("pointermove", onMove);
    viewport.addEventListener("pointerdown", onButtons);
    window.addEventListener("pointerup", onButtons);
    place();
    return () => {
      viewport.removeEventListener("pointermove", onMove);
      viewport.removeEventListener("pointerdown", onButtons);
      window.removeEventListener("pointerup", onButtons);
    };
  }, [viewport, shown]);

  if (!hover || !shown) return null;
  const identity = identityOf(index, hover.kind, hover.name);
  return (
    <div ref={ref} className="hover-chip" style={{ visibility: "hidden" }}>
      {hoverLabel(hover, identity ? identityText(identity.line) : null)}
    </div>
  );
}
