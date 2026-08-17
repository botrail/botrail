import type { ChannelKind, IoBinding, IoChannel, IoNode, IoPointId, IoPointMsg } from "./protocol";

/**
 * The studio's side of I/O map editing: which channels a point may take,
 * how a table label becomes a point id, and the channel templates the
 * node inspector adds — the same shapes `bt.io` builds in Python, so a
 * node authored here reads back through `generate_python` unchanged.
 */

/** Point kind (`"DI"`) ↔ channel kind (`"di"`), as `ChannelKind::compatible`. */
export function kindCompatible(point: string, channel: ChannelKind): boolean {
  switch (point) {
    case "DI":
    case "SafeDI":
      return channel === "di" || channel === "safe_di";
    case "DO":
    case "SafeDO":
      return channel === "do" || channel === "safe_do";
    case "AI":
      return channel === "ai";
    case "AO":
      return channel === "ao";
    case "Word":
      return channel === "word";
    default:
      return false;
  }
}

/** The controllers a node's channels serve: itself and its uplink chain
 * (`IoMap::reach`), cycles cut. */
export function reach(nodes: IoNode[], name: string): string[] {
  const out = [name];
  let cur = name;
  for (;;) {
    const parent = nodes.find((n) => n.name === cur)?.uplink?.parent;
    if (!parent || out.includes(parent)) break;
    out.push(parent);
    cur = parent;
  }
  return out;
}

export interface ChannelChoice {
  node: string;
  channel: IoChannel;
  /** The label of the point already on this channel, if any. */
  takenBy: string | null;
}

/**
 * Channels a point may be bound to: on its host node and the stations
 * whose uplink reaches it (an unhosted point takes any controller),
 * kind-compatible, in node order then channel order.
 */
export function channelChoices(
  point: IoPointMsg,
  nodes: IoNode[],
  bindings: IoBinding[],
  points: IoPointMsg[],
): ChannelChoice[] {
  const host = point.host;
  const hostDeclared = host !== null && host !== undefined && nodes.some((n) => n.name === host);
  const pool = nodes.filter((n) => {
    if (n.kind.kind === "other") return false;
    if (hostDeclared) return reach(nodes, n.name).includes(host!);
    if (host) return false; // implicit host: nothing to bind to
    return true; // unhosted: any controller or station
  });
  const out: ChannelChoice[] = [];
  for (const n of pool) {
    for (const c of n.channels ?? []) {
      if (!kindCompatible(point.kind, c.kind)) continue;
      const taken = bindings.find(
        (b) =>
          b.node === n.name &&
          b.channel === c.id &&
          !(b.point.name === point.name && (b.point.aspect ?? null) === (point.aspect ?? null) && b.point.direction === point.direction),
      );
      const takenBy = taken
        ? (points.find(
            (p) =>
              p.name === taken.point.name &&
              (p.aspect ?? null) === (taken.point.aspect ?? null) &&
              p.direction === taken.point.direction,
          )?.label ?? taken.point.name)
        : null;
      out.push({ node: n.name, channel: c, takenBy });
    }
  }
  return out;
}

export function pointId(p: IoPointMsg): IoPointId {
  return {
    name: p.name,
    aspect: (p.aspect as IoPointId["aspect"]) ?? null,
    direction: p.direction === "input" ? "input" : "output",
  };
}

/** `%IX0.0` + n bits → `%IX0.7`, `%IX1.0` (byte.bit); a base without a
 * dot counts up decimally. Mirrors `bt.io.address` for the IEC dialect. */
export function addressAt(base: string, n: number): string {
  if (base.includes(".")) {
    const at = base.lastIndexOf(".");
    const head = base.slice(0, at);
    const bit = parseInt(base.slice(at + 1), 10) + n;
    const m = /^(.*?)(\d*)$/.exec(head)!;
    const byte = m[2] ? parseInt(m[2], 10) : 0;
    return `${m[1]}${byte + Math.floor(bit / 8)}.${bit % 8}`;
  }
  const m = /^(.*?)(\d*)$/.exec(base)!;
  const num = (m[2] ? parseInt(m[2], 10) : 0) + n;
  return `${m[1]}${num}`;
}

export const CHANNEL_TEMPLATES: { id: string; label: string; kind: ChannelKind; count: number; prefix: string; ports?: boolean }[] = [
  { id: "di8", label: "+ DI×8", kind: "di", count: 8, prefix: "DI" },
  { id: "do8", label: "+ DO×8", kind: "do", count: 8, prefix: "DO" },
  { id: "di16", label: "+ DI×16", kind: "di", count: 16, prefix: "DI" },
  { id: "do16", label: "+ DO×16", kind: "do", count: 16, prefix: "DO" },
  { id: "sdi8", label: "+ safe DI×8", kind: "safe_di", count: 8, prefix: "SDI" },
  { id: "word4", label: "+ Word×4", kind: "word", count: 4, prefix: "W" },
  { id: "ur", label: "+ UR standard", kind: "di", count: 8, prefix: "DI", ports: true },
];

/** Channels a template adds after the node's existing ones (ids continue
 * the prefix numbering; the UR template adds DI0-7 + DO0-7 with ports). */
export function templateChannels(
  templateId: string,
  existing: IoChannel[],
  base: string,
): IoChannel[] {
  const t = CHANNEL_TEMPLATES.find((x) => x.id === templateId);
  if (!t) return [];
  const next = (prefix: string) => {
    let n = 0;
    for (const c of existing) {
      const m = new RegExp(`^${prefix}(\\d+)$`).exec(c.id);
      if (m) n = Math.max(n, parseInt(m[1], 10) + 1);
    }
    return n;
  };
  if (t.id === "ur") {
    const out: IoChannel[] = [];
    const di = next("DI");
    const dout = next("DO");
    for (let i = 0; i < 8; i++) out.push({ id: `DI${di + i}`, kind: "di", port: di + i });
    for (let i = 0; i < 8; i++) out.push({ id: `DO${dout + i}`, kind: "do", port: dout + i });
    return out;
  }
  const start = next(t.prefix);
  const out: IoChannel[] = [];
  for (let i = 0; i < t.count; i++) {
    const c: IoChannel = { id: `${t.prefix}${start + i}`, kind: t.kind };
    if (base.trim()) c.address = addressAt(base.trim(), i);
    out.push(c);
  }
  return out;
}
