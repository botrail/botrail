type Point = [number, number, number];
type View = { position: Point; target: Point };

/** Optional launch view; invalid links fall back to the usual orbit view. */
export function initialView(search: string): View {
  const fallback: View = { position: [1.6, -1.6, 1.2], target: [0, 0, 0.2] };
  const raw = new URLSearchParams(search).get("view");
  if (!raw) return fallback;
  const parts = raw.split(",");
  if (parts.length !== 6 || parts.some((part) => !part.trim())) return fallback;
  const values = parts.map(Number);
  if (!values.every(Number.isFinite)) return fallback;
  const position = values.slice(0, 3) as Point;
  const target = values.slice(3) as Point;
  if (position.every((value, i) => value === target[i])) return fallback;
  return { position, target };
}
