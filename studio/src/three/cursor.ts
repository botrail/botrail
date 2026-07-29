// Pointer-cursor feedback for clickable viewport objects. Reference-counted
// so overlapping hover regions (robot in front of an obstacle) don't fight.

let hoverCount = 0;

export function cursorEnter(): void {
  hoverCount += 1;
  document.body.style.cursor = "pointer";
}

export function cursorLeave(): void {
  hoverCount = Math.max(0, hoverCount - 1);
  if (hoverCount === 0) document.body.style.cursor = "";
}
