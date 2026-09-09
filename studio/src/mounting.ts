// One presentation adapter for the versioned Rust inspection envelope.
// Values and statuses come from the checker; this module never decides fit.
import type { PoseMsg } from "./protocol";

export type Status = "pass" | "fail" | "unknown" | "not_run" | "not_applicable";
export type Side = "flange" | "mount";
export type MountingMode =
  | "assembled"
  | "exploded"
  | "holes"
  | "section"
  | "access";
export type Range = { min: number; max: number };
export type RecordValue = Record<string, unknown>;
export type Hole = {
  id: string;
  position: [number, number];
  kind: string;
  diameter: Range | null;
  thread: { diameter: number; pitch: number | null } | null;
  tolerance: number | null;
};
export type Locator = {
  id: string;
  position: [number, number];
  kind: string;
  diameter: Range | null;
};
export type Box = {
  id: string;
  center: [number, number, number];
  size: [number, number, number];
};
export type Face = {
  part: string | null;
  frame: string;
  link: string | null;
  pose: PoseMsg | null;
  mapped: boolean;
  clearanceMapped: boolean;
  normal: number;
  holes: Hole[];
  locators: Locator[];
  solids: Box[];
  access: Box[];
  sources: Source[];
};
export type Source = { url: string; ref: string; section: string };
export type Check = {
  name: string;
  status: Status;
  message: string;
  inputs: RecordValue;
};
export type Finding = {
  id: string;
  target: string;
  status: Status;
  message: string;
  next: string;
  required: boolean;
  checks: Check[];
  sources: Source[];
  raw: RecordValue;
};
export type Kit = {
  target: string;
  name: string;
  catalog: string;
  support: Status;
  composition: Status;
  correspondence: Status;
  fit: Status;
  electrical: Status;
  connection: { profile: string; host: string; message: string;
    scopes: {name: string; status: Status}[];
    checks: {field: string; status: Status; actual: string; expected: string; note: string; sources: Source[]}[];
  };
  requires: string[];
  note: string;
  representationNote: string;
  representation: string;
  includes: { name: string; qty: number | null }[];
  sources: Source[];
};
export type ProductConnection = {
  target: string; name: string; catalog: string; note: string;
  connection: Kit["connection"]; requires: string[]; via: string[];
};
export type Connection = {
  target: string;
  robot: string;
  arm: string | null;
  flange: Face;
  mount: Face;
  offset: PoseMsg;
  movingLinks: string[];
};
export type Inspection = {
  simulation: {
    ready: boolean;
    blockers: string[];
    connections: { target: string; method: string; basis: string; evidenceItems: string[] }[];
  };
  revalidation: string[];
  ready: boolean;
  hash: string;
  validator: string;
  findings: Finding[];
  kits: Kit[];
  products: ProductConnection[];
  connections: Connection[];
  parts: {
    id: string;
    name: string;
    catalog: string | null;
    kit: string | null;
  }[];
  robots: {
    name: string;
    editConnections: { path: string; flange: string; mount: string }[];
    links: { name: string; part: string | null; pose: PoseMsg }[];
  }[];
  rawReport: RecordValue;
};

export function record(v: unknown): RecordValue {
  return v !== null && typeof v === "object" && !Array.isArray(v)
    ? (v as RecordValue)
    : {};
}
export function list(v: unknown): unknown[] {
  return Array.isArray(v) ? v : [];
}
export function text(v: unknown): string {
  return typeof v === "string" ? v : "";
}
export function number(v: unknown): number | null {
  return typeof v === "number" && Number.isFinite(v) ? v : null;
}
function nullableText(v: unknown): string | null {
  return text(v) || null;
}
export function status(v: unknown): Status {
  return ["pass", "fail", "unknown", "not_run", "not_applicable"].includes(
    text(v),
  )
    ? (v as Status)
    : "unknown";
}
export function range(v: unknown): Range | null {
  const r = record(v),
    min = number(r.min),
    max = number(r.max);
  return min !== null && max !== null && min <= max ? { min, max } : null;
}
function vector(v: unknown, size: number): number[] | null {
  const a = list(v);
  return a.length === size && a.every((x) => number(x) !== null)
    ? (a as number[])
    : null;
}
function pose(v: unknown): PoseMsg | null {
  const p = record(v),
    position = vector(p.position, 3),
    quaternion = vector(p.quaternion, 4);
  return position && quaternion
    ? {
        position: position as PoseMsg["position"],
        quaternion: quaternion as PoseMsg["quaternion"],
      }
    : null;
}
function holes(v: unknown): Hole[] {
  return list(v).flatMap((value) => {
    const h = record(value),
      position = vector(h.position_mm, 2),
      t = record(h.thread),
      diameter = number(t.diameter_mm);
    return text(h.id) && position
      ? [
          {
            id: text(h.id),
            position: position as [number, number],
            kind: text(h.kind),
            diameter: range(h.diameter_mm),
            thread:
              diameter === null
                ? null
                : { diameter, pitch: number(t.pitch_mm) },
            tolerance: number(h.position_tolerance_mm),
          },
        ]
      : [];
  });
}
function boxes(v: unknown): Box[] {
  return list(v).flatMap((value) => {
    const b = record(value),
      center = vector(b.center_mm, 3),
      size = vector(b.size_mm, 3);
    return text(b.id) && center && size
      ? [
          {
            id: text(b.id),
            center: center as Box["center"],
            size: size as Box["size"],
          },
        ]
      : [];
  });
}
export function sources(v: unknown): Source[] {
  const found = new Map<string, Source>();
  function visit(value: unknown, section = "") {
    if (Array.isArray(value)) {
      value.forEach((x) => visit(x, section));
      return;
    }
    const r = record(value),
      url = text(r.url),
      part = text(r.section) || section;
    if (url) {
      const s = { url, ref: text(r.ref), section: part };
      found.set(`${url}\n${s.ref}\n${part}`, s);
    }
    for (const [key, x] of Object.entries(r))
      if (key !== "url" && typeof x === "object") visit(x, part);
  }
  visit(v);
  return [...found.values()];
}
export function sourceHref(url: string): string | undefined {
  try {
    const u = new URL(url);
    return u.protocol === "https:" || u.protocol === "http:"
      ? u.href
      : undefined;
  } catch {
    return undefined;
  }
}
function face(v: unknown): Face {
  const f = record(v),
    d = record(f.declaration),
    g = record(d.geometry),
    c = record(d.clearance);
  return {
    part: nullableText(f.part),
    frame: text(f.frame),
    link: nullableText(f.link),
    pose: pose(f.pose),
    mapped: f.geometry_mapped === true,
    clearanceMapped: f.clearance_mapped === true,
    normal: number(g.normal_z) ?? 1,
    holes: holes(g.holes),
    locators: list(g.locators).flatMap((v) => {
      const l = record(v),
        position = vector(l.position_mm, 2);
      return position
        ? [
            {
              id: text(l.id),
              kind: text(l.kind),
              position: position as [number, number],
              diameter: range(l.diameter_mm),
            },
          ]
        : [];
    }),
    solids: boxes(c.solids),
    access: boxes(c.access),
    sources: sources(f.sources),
  };
}
export function parseInspection(value: unknown): Inspection {
  const v = record(value),
    r = record(v.report);
  if (
    v.schema_version !== "1" ||
    typeof r.ready !== "boolean" ||
    !text(r.input_hash) ||
    !Array.isArray(r.items)
  )
    throw new Error(
      "Unsupported mounting inspection response. Update the Studio and server together.",
    );
  return {
    simulation: {
      ready: record(r.simulation).ready === true,
      blockers: list(record(r.simulation).blockers).map(text),
      connections: list(record(r.simulation).connections).map((value) => {
        const c = record(value);
        return { target: text(c.target), method: text(c.method), basis: text(c.basis),
          evidenceItems: list(c.evidence_items).map(text) };
      }),
    },
    ready: r.ready,
    hash: text(r.input_hash),
    validator: text(r.validator_version),
    rawReport: r,
    revalidation: list(v.revalidation).map(text),
    findings: list(r.items).map((value) => {
      const i = record(value),
        e = record(i.evidence);
      return {
        id: text(i.id),
        target: text(i.target),
        status: status(i.status),
        message: text(i.message),
        next: text(i.next_action),
        required: i.required === true,
        checks: list(e.checks).map((value) => {
          const c = record(value);
          return {
            name: text(c.check),
            status: status(c.status),
            message: text(c.message),
            inputs: record(c.inputs),
          };
        }),
        sources: sources(i.evidence),
        raw: i,
      };
    }),
    kits: list(r.kits).map((value) => {
      const k = record(value),
        support = record(k.manufacturer_support),
        order = record(k.order);
      return {
        target: text(k.target),
        name: text(k.name) || text(k.catalog),
        catalog: text(k.catalog),
        support: status(support.status),
        composition: status(record(k.composition).status),
        correspondence: status(record(k.model_correspondence).status),
        fit: status(record(k.detailed_fit).status),
        electrical: status(record(k.electrical).status),
        connection: connectionSummary(k.connection),
        requires: list(order.requires).map((v) => { const i = record(v); return `${text(i.part_number) || text(i.catalog) || text(i.category)} × ${number(i.qty) ?? 1}${text(i.note) ? ` — ${text(i.note)}` : ""}`; }),
        note: text(support.note),
        representationNote: text(record(record(k.model_correspondence).representation).note),
        representation: text(record(record(k.model_correspondence).representation).status),
        includes: list(order.includes).map((value) => {
          const i = record(value);
          return { name: text(i.name), qty: number(i.qty) };
        }),
        sources: sources(support.evidence),
      };
    }),
    products: list(r.products).map((value) => {
      const p = record(value), order = record(p.order);
      return {target: text(p.target), name: text(p.name) || text(p.catalog), catalog: text(p.catalog),
        note: text(order.note), connection: connectionSummary(p.connection), via: list(p.via).map(text),
        requires: list(order.requires).map((v) => {const i = record(v); return `${text(i.part_number) || text(i.catalog) || text(i.category)} × ${number(i.qty) ?? 1}${text(i.note) ? ` — ${text(i.note)}` : ""}`;}),
      };
    }),
    connections: list(v.connections).flatMap((value) => {
      const c = record(value),
        offset = pose(c.offset);
      return offset
        ? [
            {
              target: text(c.target),
              robot: text(c.robot),
              arm: nullableText(c.arm),
              flange: face(c.flange),
              mount: face(c.mount),
              offset,
              movingLinks: list(c.moving_links).map(text),
            },
          ]
        : [];
    }),
    parts: list(v.parts).map((value) => {
      const p = record(value);
      return {
        id: text(p.id),
        name: text(p.name),
        catalog: nullableText(p.catalog),
        kit: nullableText(p.kit),
      };
    }),
    robots: list(v.robots).map((value) => {
      const r = record(value);
      return {
        name: text(r.name),
        editConnections: list(r.edit_connections).map((value) => {
          const edge = record(value);
          return {path: text(edge.path), flange: text(edge.flange), mount: text(edge.mount)};
        }),
        links: list(r.links).flatMap((value) => {
          const l = record(value),
            p = pose(l.pose);
          return p
            ? [{ name: text(l.name), part: nullableText(l.part), pose: p }]
            : [];
        }),
      };
    }),
  };
}

export function connectionFindings(v: Inspection, c: Connection): Finding[] {
  return v.findings.filter(
    (f) =>
      f.target === c.target ||
      f.target === c.flange.part ||
      f.target === c.mount.part ||
      v.kits.some(
        (k) =>
          f.target === k.target &&
          v.parts.some((p) => p.id === c.mount.part && p.kit === k.target),
      ),
  );
}
export function checkFor(
  v: Inspection,
  c: Connection,
  name: string,
): Check | undefined {
  return v.findings
    .filter((f) => f.target === c.target)
    .flatMap((f) => f.checks)
    .find((x) => x.name === name);
}
export function selectionHole(
  c: Connection,
  key: string,
): { side: Side; hole: Hole; index: number } | null {
  for (const side of ["flange", "mount"] as const) {
    const index = c[side].holes.findIndex((h) => `${side}:${h.id}` === key);
    if (index >= 0) return { side, hole: c[side].holes[index], index };
  }
  return null;
}
export function pairedHole(
  v: Inspection,
  c: Connection,
  key: string,
): ReturnType<typeof selectionHole> {
  const selected = selectionHole(c, key),
    check = checkFor(v, c, "hole_pattern");
  // With missing tolerances, Rust may find several unresolved matchings.
  // Its candidate indices do not establish which physical holes are paired.
  if (!selected || !check || check.status !== "pass") return null;
  const pairs = list(check.inputs.pairs);
  const side = selected.side === "flange" ? "mount" : "flange";
  const index =
    selected.side === "flange"
      ? number(pairs[selected.index])
      : pairs.findIndex((x) => x === selected.index);
  if (index === null || index < 0 || !c[side].holes[index]) return null;
  return { side, index, hole: c[side].holes[index] };
}
export function engagementFor(
  v: Inspection,
  c: Connection,
  key: string,
): Check | undefined {
  const selected = selectionHole(c, key),
    other = pairedHole(v, c, key);
  const h =
    selected?.hole.kind === "clearance"
      ? selected.hole
      : other?.hole.kind === "clearance"
        ? other.hole
        : null;
  return h ? checkFor(v, c, `${h.id}:engagement`) : undefined;
}
/** Drawing millimetres transformed into the flange plane, including Z gap. */
export function projectHole(
  c: Connection,
  side: Side,
  hole: Pick<Hole, "position">,
): [number, number, number] {
  const [x, y] = hole.position;
  if (side === "flange") return [x, y, 0];
  const [qx, qy, qz, qw] = c.offset.quaternion;
  const tx = -2 * qz * y,
    ty = 2 * qz * x,
    tz = 2 * (qx * y - qy * x);
  return [
    x + qw * tx + qy * tz - qz * ty + c.offset.position[0] * 1000,
    y + qw * ty + qz * tx - qx * tz + c.offset.position[1] * 1000,
    qw * tz + qx * ty - qy * tx + c.offset.position[2] * 1000,
  ];
}
export function formatRange(r: Range | null, unit = "mm"): string {
  return r
    ? `${r.min === r.max ? r.min.toFixed(2) : `${r.min.toFixed(2)}–${r.max.toFixed(2)}`} ${unit}`
    : "Not documented";
}
export const STATUS_LABEL: Record<Status, string> = {
  pass: "✓ Pass",
  fail: "× Mismatch",
  unknown: "? Unknown",
  not_run: "— Not run",
  not_applicable: "— Not applicable",
};

function connectionSummary(value: unknown): Kit["connection"] {
  const c = record(value);
  return {
    profile: text(record(c.profile).id), host: text(c.host),
    message: text(record(c.configuration).message) || "No connection configuration recorded",
    scopes: ["configuration", "electrical", "communication", "software"].map(name => ({name, status: status(record(c[name]).status)})),
    checks: ["electrical", "communication", "software"].flatMap(scope => list(record(c[scope]).checks).map(value => {
      const r = record(value), accepted = list(r.accepted);
      return {field: text(r.field).replace(/_/g, " "), status: status(r.status),
        actual: r.actual == null ? "Unknown" : String(r.actual),
        expected: accepted.length ? accepted.join(" / ") : r.minimum != null && r.maximum != null ? `${r.minimum} … ${r.maximum}` : r.minimum != null ? `≥ ${r.minimum}` : r.maximum != null ? `≤ ${r.maximum}` : "Unknown · confirmation required",
        note: text(r.note), sources: sources(r.evidence)};
    })),
  };
}
