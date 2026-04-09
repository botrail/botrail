// A small cycling palette of muted, low-saturation colors so links are
// distinguishable without being loud. Returned as CSS hsl() strings, which
// three's Color/material accept directly.
const HUES = [210, 25, 145, 275, 45, 190, 330, 95];

export function linkColor(index: number): string {
  const hue = HUES[index % HUES.length];
  return `hsl(${hue}, 30%, 56%)`;
}
