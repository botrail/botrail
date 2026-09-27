import { shortRevision } from "./partIdentity.ts";
import type { BomLineMsg, BomTarget } from "./protocol";
import { formatValue } from "./specFormat.ts";
import type { Selection } from "./store";

/**
 * The bill of materials as a table over the viewport: the lines the host
 * derives (`bom` message), in the columns `Bom::to_csv` writes, plus the
 * numeric attributes that have a total. Pure — what the overlay shows
 * and what a click on a row selects.
 */

export interface BomFilter {
  /** Only lines nobody has identified — `bom.unidentified()`. */
  unidentified: boolean;
  /** Only lines the bill made up with no scene object (tools, controllers). */
  derived: boolean;
}

export const NO_FILTER: BomFilter = { unidentified: false, derived: false };

export function bomRows(lines: BomLineMsg[], filter: BomFilter): BomLineMsg[] {
  return lines.filter((l) => {
    if (filter.unidentified && l.identified) return false;
    if (filter.derived && !l.targets.some((t) => t.derived)) return false;
    return true;
  });
}

/** The attributes that add up across a bill — a mass, a power draw, a
 * current, a price — as opposed to a reach or a payload, which the host
 * also sums (`Bom::total` sums any number) but nobody reads as a total. */
const ADDITIVE = /^(mass_kg|power_w|heat_w|current_a|price(_[a-z]+)?|cost(_[a-z]+)?)$/;

/** The attribute columns worth a column: the additive ones with a total. */
export function totalColumns(totals: Record<string, number>): string[] {
  return Object.keys(totals)
    .filter((k) => ADDITIVE.test(k))
    .sort();
}

/** `35 lines · 18 unidentified · mass 451 kg` */
export function bomCaption(lines: BomLineMsg[], totals: Record<string, number>): string {
  if (lines.length === 0) return "no lines — nothing on the bill yet";
  const parts = [`${lines.length} line${lines.length === 1 ? "" : "s"}`];
  const unidentified = lines.filter((l) => !l.identified).length;
  if (unidentified > 0) parts.push(`${unidentified} unidentified`);
  if (typeof totals.mass_kg === "number") parts.push(`mass ${formatValue("mass_kg", totals.mass_kg)}`);
  return parts.join(" · ");
}

/** The catalog column: `universal_robots/ur/ur5e/r2 @1d6bfc1`. */
export function catalogCell(line: BomLineMsg): string {
  if (!line.catalog) return "";
  const rev = shortRevision(line.catalog.revision);
  return rev ? `${line.catalog.id} @${rev}` : line.catalog.id;
}

/** The names column, as the CSV joins them. */
export function namesCell(line: BomLineMsg): string {
  return line.targets.map((t) => t.name).join("; ");
}

/** The value of an attribute column for a line, `—` when it states none. */
export function attributeCell(line: BomLineMsg, key: string): string {
  const v = line.attributes[key];
  return v === undefined ? "—" : formatValue(key, v);
}

/** What a click on a row selects: its first target, as the scene tree
 * would. A group is a tree path (leading slash); an I/O node that is
 * declared is the node, an arm's undeclared controller a derived line. */
export function selectionForTarget(target: BomTarget, declaredIoNodes: Set<string>): Selection {
  const bare = target.name.replace(/^\/+/, "");
  switch (target.kind) {
    case "robot":
      return { type: "tcp", robot: target.name };
    case "obstacle":
      return { type: "obstacle", name: target.name };
    case "group":
      return { type: "group", path: `/${bare}` };
    case "sensor":
    case "device":
    case "camera":
    case "lidar":
      return { type: target.kind, name: target.name };
    case "io_node":
      return declaredIoNodes.has(target.name)
        ? { type: "io_node", name: target.name }
        : { type: "line", kind: "io_node", name: target.name };
    case "tool":
      return { type: "line", kind: "tool", name: target.name };
  }
}

/** Whether the selection is one of the line's targets — the row to mark. */
export function rowSelected(line: BomLineMsg, selection: Selection): boolean {
  const bare = (s: string) => s.replace(/^\/+/, "");
  return line.targets.some((t) => {
    switch (selection.type) {
      case "tcp":
      case "robot":
        return t.kind === "robot" && t.name === selection.robot;
      case "obstacle":
        return t.kind === "obstacle" && bare(t.name) === bare(selection.name);
      case "group":
        return t.kind === "group" && bare(t.name) === bare(selection.path);
      case "line":
        return t.kind === selection.kind && t.name === selection.name;
      default:
        return t.kind === selection.type && t.name === selection.name;
    }
  });
}
