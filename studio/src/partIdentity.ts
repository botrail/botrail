import type { BomLineMsg, BomTarget, PartTargetKind } from "./protocol";
import type { Selection } from "./store";

/**
 * What a thing in the scene *is* commercially, read off the bill of
 * materials the host derives (`Scene::bom`) and broadcasts as `bom`. The
 * studio never derives identity itself: every line names the residents,
 * groups and derived lines (`arm/tool`, `arm/controller`) it stands for,
 * and this module only looks them up — exactly, or through the group a
 * clicked prim belongs to (`fence/east/panels/w1000/3` → `fence/east`).
 */

/** `kind:name` → the line and the target entry that names it. */
export type BomIndex = Map<string, { line: BomLineMsg; target: BomTarget }>;

/** A resolved identity: the line, the target it was found under, and —
 * when a prim inherited it from the group it sits in — that group. */
export interface Identity {
  line: BomLineMsg;
  target: BomTarget;
  inherited: boolean;
  ancestor: string | null;
}

/** USD prim paths keep their leading slash, `add_box("fence/p0")` names do
 * not; a pin may have been written either way. Index and look up bare. */
function bare(name: string): string {
  return name.replace(/^\/+/, "");
}

function key(kind: PartTargetKind, name: string): string {
  return `${kind}:${bare(name)}`;
}

export function indexBom(lines: BomLineMsg[]): BomIndex {
  const index: BomIndex = new Map();
  for (const line of lines) {
    for (const target of line.targets) {
      index.set(key(target.kind, target.name), { line, target });
    }
  }
  return index;
}

export function lookup(
  index: BomIndex,
  kind: PartTargetKind,
  name: string,
): { line: BomLineMsg; target: BomTarget } | undefined {
  return index.get(key(kind, name));
}

/** The groups a name sits in, innermost first: `a/b/c` → `a/b`, `a`. */
export function ancestors(name: string): string[] {
  const parts = bare(name).split("/").filter(Boolean);
  const out: string[] = [];
  for (let i = parts.length - 1; i >= 1; i--) out.push(parts.slice(0, i).join("/"));
  return out;
}

/** The kinds whose body a generator names under the resident's own name
 * (`conv` the device, `conv/belt` its slab; `gate_curtain` the sensor,
 * `gate_curtain/column_a` its post): a prim there is part of that line. */
const BODY_OWNERS: PartTargetKind[] = ["group", "device", "sensor", "camera", "lidar"];

/** The identity behind a name of a kind: its own line, else — for a prim
 * or a sub-group — the nearest enclosing name that has one: a pinned
 * group, or the device / sensor / camera / scanner whose body it is. */
export function identityOf(
  index: BomIndex,
  kind: PartTargetKind,
  name: string,
): Identity | null {
  const own = lookup(index, kind, name);
  if (own) return { ...own, inherited: false, ancestor: null };
  if (kind !== "obstacle" && kind !== "group") return null;
  // The name itself may be a resident's body (`/gate_curtain` the group
  // of the sensor's posts), then every name above it.
  for (const owner of [bare(name), ...ancestors(name)]) {
    for (const k of BODY_OWNERS) {
      if (k === kind && owner === bare(name)) continue;
      const hit = lookup(index, k, owner);
      if (hit) return { ...hit, inherited: true, ancestor: owner };
    }
  }
  return null;
}

/** The identity of what is selected, if the bill has a line for it. */
export function identityFor(selection: Selection, index: BomIndex): Identity | null {
  switch (selection.type) {
    case "tcp":
    case "robot":
      return identityOf(index, "robot", selection.robot);
    case "obstacle":
      return identityOf(index, "obstacle", selection.name);
    case "group":
      return identityOf(index, "group", selection.path);
    case "line":
      return identityOf(index, selection.kind, selection.name);
    default:
      return identityOf(index, selection.type, selection.name);
  }
}

/** The tail of a catalog id that still reads as a product: `ur5e/r2`. */
export function shortId(id: string): string {
  return id.split("/").filter(Boolean).slice(-2).join("/");
}

/** `1d6bfc13…` → `1d6bfc1`; a local package digest keeps its prefix. */
export function shortRevision(revision: string | null | undefined): string | null {
  if (!revision) return null;
  if (/^[0-9a-f]{40}$/.test(revision)) return revision.slice(0, 7);
  if (revision.startsWith("local-sha256:")) return "local";
  return revision;
}

/** A controller the arm's set includes: the line carries the arm's catalog
 * reference and no model of its own — it is no purchase of its own. */
export function inTheSet(line: BomLineMsg): boolean {
  return line.category === "robot_controller" && !line.model && line.catalog !== null;
}

/** The short label a line reads as on the scene tree: model, else the
 * product part of the catalog id, else maker, else — for a pinned prim —
 * the category the pin gave it; `?` for equipment nobody has identified. */
export function partLabel(line: BomLineMsg, target: BomTarget): string {
  if (inTheSet(line)) return "in the set";
  if (line.identified) {
    return (
      line.model ??
      (line.catalog ? shortId(line.catalog.id) : null) ??
      line.manufacturer ??
      line.category
    );
  }
  const pinned = target.kind === "obstacle" || target.kind === "group";
  return pinned && line.category !== "part" ? line.category : "?";
}

/** `Universal Robots UR5e` — what the focus chip appends to the selection;
 * an unidentified line says so with the category the bill gives it. */
export function identityText(line: BomLineMsg): string {
  if (!line.identified) return `unidentified (${line.category})`;
  if (inTheSet(line)) return "controller in the set";
  const named = [line.manufacturer, line.model].filter(Boolean).join(" ");
  return named || (line.catalog ? shortId(line.catalog.id) : line.category);
}

/** The full identity for a tooltip. */
export function partTitle(line: BomLineMsg): string {
  if (!line.identified) {
    return `unidentified — no maker, model or catalog reference (${line.category})`;
  }
  const named = [line.manufacturer, line.model].filter(Boolean).join(" ");
  const rev = shortRevision(line.catalog?.revision);
  const cat = line.catalog ? `${line.catalog.id}${rev ? `@${rev}` : ""}` : "";
  const qty = line.qty !== 1 ? ` ×${line.qty}` : "";
  const head = [named, cat && `(${cat})`].filter(Boolean).join(" ") || line.category;
  return `${head}${qty}${line.description ? ` — ${line.description}` : ""}`;
}

/** The lines hanging off a robot that have no scene object of their own:
 * its tools (`arm/tool`, `arm/tool2`) and the controller it needs
 * (`arm/controller`, until a cabinet is declared for it). */
export function robotStack(
  index: BomIndex,
  robot: string,
): { tools: { name: string; line: BomLineMsg; target: BomTarget }[]; controller: { name: string; line: BomLineMsg; target: BomTarget } | null } {
  const tools: { name: string; line: BomLineMsg; target: BomTarget }[] = [];
  let controller: { name: string; line: BomLineMsg; target: BomTarget } | null = null;
  const prefix = `${robot}/`;
  for (const { line, target } of index.values()) {
    if (!target.derived || !target.name.startsWith(prefix)) continue;
    const rest = target.name.slice(prefix.length);
    if (target.kind === "tool" && /^tool\d*$/.test(rest)) {
      tools.push({ name: target.name, line, target });
    } else if (target.kind === "io_node" && rest === "controller") {
      controller = { name: target.name, line, target };
    }
  }
  tools.sort((a, b) => a.name.localeCompare(b.name, undefined, { numeric: true }));
  return { tools, controller };
}

/** Whether a line still names `kind:name` — for a selection of a derived
 * line to be dropped when the bill no longer has it. */
export function hasTarget(lines: BomLineMsg[], kind: PartTargetKind, name: string): boolean {
  return lines.some((l) => l.targets.some((t) => t.kind === kind && bare(t.name) === bare(name)));
}
