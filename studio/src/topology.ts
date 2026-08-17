import type { IoNode, IoPointMsg, TopoEdgeMsg, TopoNodeMsg, TopologyMsg } from "./protocol";

/**
 * Deterministic swimlane layout of the cell's electrical topology — the
 * data half of `IoTopologyOverlay`. Every host (controller) is one
 * horizontal lane: its header names it, lists the programs it runs and the
 * stations hanging off it; its rows are the points wired on it (or on its
 * stations), one per channel; the field side — sensors, devices, robots,
 * field devices, declared points — sits in a column on the right, each
 * wire running from its row to its far end. Handshake signals between
 * controllers become vertical buses in a gutter between the rows and the
 * field column, one x per signal, so a 1:N fan-out is one bus with taps.
 * Functional (program → program) edges route through a gutter on the left.
 *
 * Nothing here is measured from the DOM: the geometry is a pure function
 * of the graph, so the same cell draws the same picture — in the studio,
 * in a screenshot, in a test — and no coordinates are ever saved.
 */

export type Layer = "functional" | "io" | "network" | "wiring" | "safety";
export const LAYERS: Layer[] = ["functional", "io", "network", "wiring", "safety"];

/** Mirrors `iomap::TopoLayer::shows`. */
export function layerShows(layer: Layer, e: TopoEdgeMsg): boolean {
  switch (layer) {
    case "functional":
      return e.kind === "functional";
    case "io":
      return e.kind === "io";
    case "network":
      return e.kind === "uplink";
    case "wiring":
      return e.kind === "io" || e.kind === "handshake" || e.kind === "uplink";
    case "safety":
      return !!e.safety;
  }
}

export function visibleEdges(t: TopologyMsg, layers: Set<Layer>): TopoEdgeMsg[] {
  if (layers.size === 0) return t.edges;
  return t.edges.filter((e) => [...layers].some((l) => layerShows(l, e)));
}

// ------------------------------------------------------------ geometry

export const PAD = 10;
/** Left gutter for functional (program → program) routes. */
export const FN_GUTTER_STEP = 8;
export const HEAD_H = 22;
export const PROG_H = 16;
export const ROW_H = 16;
export const BLOCK_GAP = 12;
export const CHAN_W = 104; // channel cell
export const LABEL_W = 150; // point label cell
export const HS_STEP = 10; // one handshake bus per step
export const JOG_W = 28; // jog gutter before the field column
export const FIELD_W = 150;
export const FIELD_GAP = 4;

export interface Row {
  host: string;
  /** Station the channel sits on (`RIO1`), when not the host itself. */
  station: string | null;
  label: string;
  direction: "input" | "output";
  channel: string | null;
  address: string | null;
  lane: string | null;
  safety: boolean;
  status: string;
  /** Far-end node id for io rows (`sensor:eye`, `field:-B1`), null for a
   * handshake row. */
  far: string | null;
  /** The handshake signal for rows that are one end of a bus. */
  handshake: string | null;
  point: IoPointMsg | null;
  /** Assigned by the layout. */
  y: number;
}

export interface Block {
  host: string;
  label: string;
  implicit: boolean;
  nodeKind: string | null;
  programs: { name: string; y: number }[];
  stations: { name: string; bus: string | null }[];
  rows: Row[];
  y: number;
  height: number;
  unbound: number;
}

export interface FieldNode {
  id: string;
  kind: string;
  label: string;
  y: number;
}

export interface Bus {
  signal: string;
  x: number;
  y0: number;
  y1: number;
  /** Row ys tapped by the bus, and whether that tap is the writer. */
  taps: { y: number; writer: boolean }[];
  lane: string | null;
  safety: boolean;
}

export interface Wire {
  row: Row;
  far: FieldNode;
  /** Vertical jog x when the row and the node are not aligned. */
  jogX: number | null;
}

export interface FnEdge {
  signal: string;
  fromY: number;
  toY: number;
  x: number;
  lane: string | null;
}

export interface Layout {
  blocks: Block[];
  fields: FieldNode[];
  buses: Bus[];
  wires: Wire[];
  fnEdges: FnEdge[];
  width: number;
  height: number;
  /** x of the block's left edge and its right edge (row ports). */
  blockX: number;
  blockRight: number;
  hsX0: number;
  fieldX: number;
}

function hostOf(id: string): string {
  return id.startsWith("host:") ? id.slice(5) : id;
}

/**
 * Lays the topology out. `nodes` (the authored node list) fixes the lane
 * order — declared controllers first, in authoring order, then implicit
 * hosts alphabetically; `points` supplies the channels of handshake rows,
 * which the graph's host → host edges do not carry.
 */
export function layoutTopology(
  t: TopologyMsg,
  edges: TopoEdgeMsg[],
  nodes: IoNode[],
  points: IoPointMsg[],
  showFunctional: boolean,
  showNetwork: boolean,
): Layout {
  const byId = new Map<string, TopoNodeMsg>();
  for (const n of t.nodes) byId.set(n.id, n);

  // Hosts (lanes) and stations (chips inside their parent's lane).
  const stationParent = new Map<string, string>();
  for (const n of t.nodes) {
    if (n.kind === "station" && n.host) stationParent.set(hostOf(n.id), n.host);
  }
  const hostIds: string[] = [];
  const declaredOrder = new Map<string, number>();
  nodes.forEach((n, i) => declaredOrder.set(n.name, i));
  for (const n of t.nodes) {
    if (n.kind === "host") hostIds.push(hostOf(n.id));
  }
  hostIds.sort((a, b) => {
    const da = declaredOrder.get(a);
    const db = declaredOrder.get(b);
    if (da !== undefined && db !== undefined) return da - db;
    if (da !== undefined) return -1;
    if (db !== undefined) return 1;
    return a.localeCompare(b);
  });
  const laneOf = (near: string) => stationParent.get(near) ?? near;

  const blocks = new Map<string, Block>();
  for (const h of hostIds) {
    const n = byId.get(`host:${h}`);
    blocks.set(h, {
      host: h,
      label: n?.label ?? h,
      implicit: !!n?.implicit,
      nodeKind: n?.node_kind ?? null,
      programs: [],
      stations: [],
      rows: [],
      y: 0,
      height: 0,
      unbound: 0,
    });
  }
  // A station whose parent is not a host lane (should not happen) gets
  // its own lane so nothing is lost.
  for (const parent of stationParent.values()) {
    if (!blocks.has(parent)) {
      blocks.set(parent, {
        host: parent,
        label: parent,
        implicit: true,
        nodeKind: null,
        programs: [],
        stations: [],
        rows: [],
        y: 0,
        height: 0,
        unbound: 0,
      });
      hostIds.push(parent);
    }
  }
  for (const n of t.nodes) {
    if (n.kind === "program" && n.host) {
      blocks.get(n.host)?.programs.push({ name: n.label, y: 0 });
    }
  }
  for (const e of edges) {
    if (e.kind === "uplink") {
      const parent = hostOf(e.to);
      blocks.get(parent)?.stations.push({ name: hostOf(e.from), bus: e.bus ?? null });
    }
  }
  if (!showNetwork) {
    // Stations still own channels; only the connector chips go.
    for (const b of blocks.values()) b.stations = [];
  }

  // Rows: io edges (one per point wire) and handshake ends.
  const pointOf = (label: string, direction: string, host: string) =>
    points.find(
      (p) => p.label === label && p.direction === direction && (p.host ?? "") === host,
    ) ?? null;
  const fieldIds = new Set<string>();
  for (const e of edges) {
    if (e.kind !== "io" || !e.point) continue;
    const input = !e.from.startsWith("host:");
    const nearId = input ? e.to : e.from;
    const farId = input ? e.from : e.to;
    const near = hostOf(nearId);
    const lane = laneOf(near);
    const block = blocks.get(lane);
    if (!block) continue;
    const point = pointOf(e.point, input ? "input" : "output", block.host)
      ?? pointOf(e.point, input ? "input" : "output", near);
    block.rows.push({
      host: block.host,
      station: near === block.host ? null : near,
      label: e.point,
      direction: input ? "input" : "output",
      channel: e.channel ?? null,
      address: e.address ?? null,
      lane: e.lane ?? null,
      safety: !!e.safety,
      status: point?.status ?? (e.channel ? "bound" : "unbound"),
      far: farId,
      handshake: null,
      point,
      y: 0,
    });
    fieldIds.add(farId);
  }
  // Handshake buses: one row on the writer (out) and one per reader (in).
  const hsSignals: string[] = [];
  const hsEnds = new Map<string, { writer: string; readers: string[]; lane: string | null; safety: boolean }>();
  for (const e of edges) {
    if (e.kind !== "handshake" || !e.signal) continue;
    let h = hsEnds.get(e.signal);
    if (!h) {
      h = { writer: hostOf(e.from), readers: [], lane: e.lane ?? null, safety: !!e.safety };
      hsEnds.set(e.signal, h);
      hsSignals.push(e.signal);
    }
    h.readers.push(hostOf(e.to));
  }
  const ensureRow = (host: string, signal: string, direction: "input" | "output", lane: string | null, safety: boolean) => {
    const block = blocks.get(host);
    if (!block) return null;
    let row = block.rows.find((r) => r.handshake === signal && r.direction === direction);
    if (row) return row;
    const point = pointOf(signal, direction, host);
    row = {
      host,
      station: point?.node && point.node !== host ? point.node : null,
      label: signal,
      direction,
      channel: point?.channel ?? null,
      address: point?.address ?? null,
      lane,
      safety,
      status: point?.status ?? "unbound",
      far: null,
      handshake: signal,
      point,
      y: 0,
    };
    block.rows.push(row);
    return row;
  };
  for (const signal of hsSignals) {
    const h = hsEnds.get(signal)!;
    ensureRow(h.writer, signal, "output", h.lane, h.safety);
    for (const r of h.readers) ensureRow(r, signal, "input", h.lane, h.safety);
  }

  // Vertical placement.
  const orderedBlocks = hostIds.map((h) => blocks.get(h)!);
  let y = PAD;
  for (const b of orderedBlocks) {
    b.y = y;
    let cy = y + HEAD_H;
    for (const p of b.programs) {
      p.y = cy + PROG_H / 2;
      cy += PROG_H;
    }
    // Rows: keep derivation order, stations after the host's own rows.
    b.rows.sort((r1, r2) => Number(r1.station !== null) - Number(r2.station !== null));
    for (const r of b.rows) {
      r.y = cy + ROW_H / 2;
      cy += ROW_H;
    }
    b.unbound = b.rows.filter((r) => r.status === "unbound").length;
    b.height = Math.max(cy - y, HEAD_H) + 6;
    y += b.height + BLOCK_GAP;
  }

  // Field column: one node per far end, at the y of its first wire,
  // pushed down past anything already there.
  const fields: FieldNode[] = [];
  const taken: number[] = [];
  const place = (id: string, wantY: number): FieldNode => {
    const n = byId.get(id);
    let fy = wantY;
    let moved = true;
    while (moved) {
      moved = false;
      for (const ty of taken) {
        if (Math.abs(ty - fy) < ROW_H) {
          fy = ty + ROW_H;
          moved = true;
        }
      }
    }
    taken.push(fy);
    const node: FieldNode = {
      id,
      kind: n?.kind ?? id.split(":")[0],
      label: n?.label ?? id.split(":").slice(1).join(":"),
      y: fy,
    };
    fields.push(node);
    return node;
  };
  const fieldById = new Map<string, FieldNode>();
  const wires: Wire[] = [];
  for (const b of orderedBlocks) {
    for (const r of b.rows) {
      if (!r.far) continue;
      let f = fieldById.get(r.far);
      if (!f) {
        f = place(r.far, r.y);
        fieldById.set(r.far, f);
      }
      wires.push({ row: r, far: f, jogX: null });
    }
  }

  // Horizontal placement.
  const fnCount = showFunctional ? countFunctional(edges) : 0;
  const blockX = PAD + (fnCount > 0 ? 6 + fnCount * FN_GUTTER_STEP : 0);
  const blockRight = blockX + CHAN_W + LABEL_W;
  const hsX0 = blockRight + 14;
  const jogX0 = hsX0 + hsSignals.length * HS_STEP + 6;
  const fieldX = jogX0 + JOG_W;
  // Wires into one field node share a jog x (they merge into the node);
  // different nodes take different x's so parallel jogs stay apart.
  const jogOf = new Map<string, number>();
  for (const w of wires) {
    if (Math.abs(w.row.y - w.far.y) > 0.5) {
      let jx = jogOf.get(w.far.id);
      if (jx === undefined) {
        jx = jogX0 + 4 + (jogOf.size % 6) * 4;
        jogOf.set(w.far.id, jx);
      }
      w.jogX = jx;
    }
  }
  const buses: Bus[] = hsSignals.map((signal, k) => {
    const h = hsEnds.get(signal)!;
    const taps: { y: number; writer: boolean }[] = [];
    const wr = blocks.get(h.writer)?.rows.find((r) => r.handshake === signal && r.direction === "output");
    if (wr) taps.push({ y: wr.y, writer: true });
    for (const rd of h.readers) {
      const rr = blocks.get(rd)?.rows.find((r) => r.handshake === signal && r.direction === "input");
      if (rr) taps.push({ y: rr.y, writer: false });
    }
    const ys = taps.map((tp) => tp.y);
    return {
      signal,
      x: hsX0 + k * HS_STEP,
      y0: ys.length ? Math.min(...ys) : 0,
      y1: ys.length ? Math.max(...ys) : 0,
      taps,
      lane: h.lane,
      safety: h.safety,
    };
  });

  // Functional edges: program chip → program chip through the left gutter.
  const fnEdges: FnEdge[] = [];
  if (showFunctional) {
    const progY = new Map<string, number>();
    for (const b of orderedBlocks) for (const p of b.programs) progY.set(p.name, p.y);
    const seen = new Map<string, number>();
    for (const e of edges) {
      if (e.kind !== "functional" || !e.signal) continue;
      const fromY = progY.get(e.from.replace(/^prog:/, ""));
      const toY = progY.get(e.to.replace(/^prog:/, ""));
      if (fromY === undefined || toY === undefined) continue;
      let k = seen.get(e.signal);
      if (k === undefined) {
        k = seen.size;
        seen.set(e.signal, k);
      }
      fnEdges.push({ signal: e.signal, fromY, toY, x: PAD + 4 + k * FN_GUTTER_STEP, lane: e.lane ?? null });
    }
  }

  const fieldBottom = fields.reduce((m, f) => Math.max(m, f.y + ROW_H / 2), 0);
  const height = Math.max(y - BLOCK_GAP, fieldBottom) + PAD;
  return {
    blocks: orderedBlocks,
    fields,
    buses,
    wires,
    fnEdges,
    width: fieldX + FIELD_W + PAD,
    height,
    blockX,
    blockRight,
    hsX0,
    fieldX,
  };
}

function countFunctional(edges: TopoEdgeMsg[]): number {
  const s = new Set<string>();
  for (const e of edges) if (e.kind === "functional" && e.signal) s.add(e.signal);
  return s.size;
}

/** `RIO1.DI2 [%IX1.2]` / `DI2 [%IX0.2]` / `—`. */
export function rowChannel(r: Row): string {
  if (!r.channel) return "—";
  const id = r.station ? `${r.station}.${r.channel}` : r.channel;
  return r.address ? `${id} [${r.address}]` : id;
}
