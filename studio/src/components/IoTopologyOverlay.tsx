import { useMemo, useState } from "react";

import { useDockReserve } from "../dockReserve";
import { signalAt } from "../playbackRig";
import { useStudioStore } from "../store";
import { OverlayTabs } from "./OverlayTabs";
import {
  CHAN_W,
  FIELD_W,
  LABEL_W,
  LAYERS,
  PROG_H,
  ROW_H,
  layoutTopology,
  rowChannel,
  visibleEdges,
  type FieldNode,
  type Layer,
  type Row,
} from "../topology";

/**
 * The cell's electrical topology, drawn: one lane per controller with its
 * programs, stations and channel rows; the field side on the right; the
 * handshake buses between them; functional program → program routes on
 * the left. Hand-rolled SVG on a deterministic layout (see topology.ts)
 * — the same graph `export_topology()` writes as DOT / Mermaid, so the
 * figure in a design document and this screen cannot disagree.
 *
 * Interaction: layer chips filter the edges the way `layers=` does; a
 * lane's live level colours its wire while a bake plays; clicking a
 * field node selects it in the scene tree, a lane header selects the
 * node, a row lights its lane on the timeline dock.
 */

const TOPO_LAYERS_KEY = "botrail-studio.topo-layers";

function initialLayers(): Set<Layer> {
  try {
    const raw = localStorage.getItem(TOPO_LAYERS_KEY);
    if (raw) {
      const parsed = JSON.parse(raw) as string[];
      const set = new Set<Layer>();
      for (const l of parsed) if ((LAYERS as string[]).includes(l)) set.add(l as Layer);
      if (set.size > 0) return set;
    }
  } catch {
    // Persistence only.
  }
  return new Set<Layer>(["wiring"]);
}

function persistLayers(layers: Set<Layer>): void {
  try {
    localStorage.setItem(TOPO_LAYERS_KEY, JSON.stringify([...layers]));
  } catch {
    // Persistence only.
  }
}

const KIND_GLYPH: Record<string, string> = {
  sensor: "📡",
  device: "⚙",
  robot: "🤖",
  field: "▭",
  declared: "◇",
};

function trunc(s: string, n: number): string {
  return s.length > n ? `${s.slice(0, n - 1)}…` : s;
}

export function IoTopologyOverlay() {
  const open = useStudioStore((s) => s.topoOpen);
  const setOpen = useStudioStore((s) => s.setTopoOpen);
  const io = useStudioStore((s) => s.io);
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const highlightLane = useStudioStore((s) => s.highlightLane);
  const setHighlightLane = useStudioStore((s) => s.setHighlightLane);
  const sensors = useStudioStore((s) => s.sensors);
  const devices = useStudioStore((s) => s.devices);
  const selectSensor = useStudioStore((s) => s.selectSensor);
  const selectDevice = useStudioStore((s) => s.selectDevice);
  const selectIoNode = useStudioStore((s) => s.selectIoNode);
  const selectTcp = useStudioStore((s) => s.selectTcp);
  const focusTab = useStudioStore((s) => s.focusTab);
  const docked = useStudioStore((s) => s.playback !== null);
  const [panel, setPanel] = useState<HTMLDivElement | null>(null);
  useDockReserve(panel, docked);
  const [layers, setLayers] = useState<Set<Layer>>(initialLayers);

  const toggleLayer = (l: Layer) => {
    const next = new Set(layers);
    if (next.has(l)) {
      next.delete(l);
    } else {
      next.add(l);
    }
    setLayers(next);
    persistLayers(next);
  };

  const layout = useMemo(() => {
    const edges = visibleEdges(io.topology, layers);
    return layoutTopology(
      io.topology,
      edges,
      io.io.nodes,
      io.points,
      layers.has("functional"),
      layers.has("network") || layers.has("wiring"),
    );
  }, [io, layers]);
  const lanes = useMemo(() => {
    const m = new Map<string, { times: number[]; values: boolean[] }>();
    for (const lane of timeline?.signals ?? []) m.set(lane.name, lane);
    return m;
  }, [timeline]);
  const live = (lane: string | null): boolean | null => {
    if (!lane) return null;
    const l = lanes.get(lane);
    return l ? signalAt(l.times, l.values, playbackTime) : null;
  };

  if (!open) return null;

  const pickField = (f: FieldNode) => {
    const name = f.label;
    if (f.kind === "sensor" && sensors.some((s) => s.name === name)) {
      selectSensor(name);
      focusTab("obstacle");
    } else if (f.kind === "device" && devices.some((d) => d.name === name)) {
      selectDevice(name);
      focusTab("obstacle");
    } else if (f.kind === "robot") {
      selectTcp(name);
    }
  };
  const pickRow = (r: Row) => {
    setHighlightLane(r.lane && highlightLane !== r.lane ? r.lane : null);
  };
  const wireClass = (lane: string | null, extra = "") => {
    const v = live(lane);
    const hot = lane !== null && lane === highlightLane;
    return `topo-wire${v === true ? " on" : v === false ? " off" : ""}${hot ? " hot" : ""}${extra}`;
  };

  const { blocks, fields, buses, wires, fnEdges, width, height, blockX, blockRight, fieldX } = layout;
  const rowLeft = blockX + 6;
  const rowRight = blockRight;

  return (
    <div className="sfc-overlay topo-overlay" ref={setPanel}>
      <div className="sfc-head">
        <OverlayTabs active="topo" />
        <span className="sfc-caption">
          {blocks.length === 0
            ? "no controllers — programs that read sensors, set signals or command devices put their host here"
            : `${blocks.length} controller${blocks.length > 1 ? "s" : ""} · ${fields.length} field · ${buses.length} handshake${buses.length === 1 ? "" : "s"}`}
        </span>
        <span className="topo-layers">
          {LAYERS.map((l) => (
            <button
              key={l}
              className={`timeline-button${layers.has(l) ? " timeline-button-on" : ""}`}
              onClick={() => toggleLayer(l)}
              title={
                l === "wiring"
                  ? "I/O wires + handshake buses + uplinks — the electrical drawing"
                  : l === "functional"
                    ? "program → program signals"
                    : l === "io"
                      ? "point → channel wires"
                      : l === "network"
                        ? "uplinks (remote I/O, safety stations)"
                        : "safety points only"
              }
            >
              {l}
            </button>
          ))}
        </span>
        <button
          className="timeline-button"
          onClick={() => setOpen(false)}
          title="close (the ⌗ Topology button reopens it)"
        >
          ×
        </button>
      </div>
      <div className="sfc-scroll topo-scroll">
        {blocks.length === 0 ? (
          <div className="sfc-empty hint">nothing to draw yet</div>
        ) : (
          <svg className="topo-svg" width={width} height={height} viewBox={`0 0 ${width} ${height}`}>
            {/* Lanes */}
            {blocks.map((b) => (
              <g key={b.host} className={`topo-block${b.implicit ? " implicit" : ""}`}>
                <rect
                  x={blockX}
                  y={b.y}
                  width={blockRight - blockX}
                  height={b.height}
                  rx={4}
                  className="topo-block-frame"
                />
                <g
                  className={`topo-block-head${b.implicit ? "" : " clickable"}`}
                  onClick={() => {
                    if (!b.implicit) selectIoNode(b.host);
                  }}
                >
                  <title>{b.label}</title>
                  <text x={rowLeft} y={b.y + 15} className="topo-host">
                    {b.host}
                  </text>
                  <text x={rowLeft + 8 + b.host.length * 7.2} y={b.y + 15} className="topo-host-kind">
                    {b.implicit ? "implicit" : (b.nodeKind ?? "")}
                    {b.unbound > 0 && (
                      <tspan className="topo-badge" dx={8}>
                        {b.unbound} unbound
                      </tspan>
                    )}
                  </text>
                </g>
                {(() => {
                  // Station chips, right-aligned in the header, each as
                  // wide as its label — `RIO1 · PROFINET`.
                  let right = rowRight - 6;
                  return b.stations.map((st) => {
                    const label = st.bus ? `${st.name} · ${st.bus}` : st.name;
                    const w = Math.min(120, 10 + label.length * 6.2);
                    const x = right - w;
                    right = x - 8;
                    return (
                      <g key={st.name} className="topo-station" onClick={() => selectIoNode(st.name)}>
                        <title>{`${st.name} — uplink to ${b.host}${st.bus ? ` (${st.bus})` : ""}`}</title>
                        <line x1={x - 4} y1={b.y + 9.5} x2={x} y2={b.y + 9.5} className="topo-uplink" />
                        <rect x={x} y={b.y + 3} width={w} height={13} rx={3} className="topo-station-box" />
                        <text x={x + w / 2} y={b.y + 13} className="topo-station-text" textAnchor="middle">
                          {trunc(label, 18)}
                        </text>
                      </g>
                    );
                  });
                })()}
                {b.programs.map((p) => (
                  <g key={p.name} className="topo-prog">
                    <rect x={rowLeft} y={p.y - PROG_H / 2 + 2} width={Math.min(LABEL_W, 8 + p.name.length * 6.6)} height={PROG_H - 4} rx={3} />
                    <text x={rowLeft + 4} y={p.y + 3.5}>
                      ▸ {trunc(p.name, 22)}
                    </text>
                  </g>
                ))}
                {b.rows.map((r) => {
                  const v = live(r.lane);
                  const hot = r.lane !== null && r.lane === highlightLane;
                  return (
                    <g
                      key={`${r.label}/${r.direction}/${r.station ?? ""}`}
                      className={`topo-row topo-${r.status}${hot ? " hot" : ""}${r.lane ? " clickable" : ""}`}
                      onClick={() => pickRow(r)}
                    >
                      <title>
                        {`${r.label} (${r.direction === "input" ? "in" : "out"}) — ${r.status}${r.channel ? ` on ${rowChannel(r)}` : ""}${r.lane ? " — click to light its lane" : ""}`}
                      </title>
                      {hot && (
                        <rect x={blockX + 1} y={r.y - ROW_H / 2} width={blockRight - blockX - 2} height={ROW_H} className="topo-row-hot" />
                      )}
                      <text x={rowLeft} y={r.y + 3.5} className={`topo-chan${r.channel ? "" : " missing"}`}>
                        {trunc(rowChannel(r), 17)}
                      </text>
                      <text x={rowLeft + CHAN_W} y={r.y + 3.5} className="topo-point">
                        {/* The field is on the right: an input arrives from
                            there (←), an output leaves toward it (→). */}
                        {r.direction === "input" ? "← " : "→ "}
                        {trunc(r.label, 20)}
                        {r.safety ? " ⛨" : ""}
                      </text>
                      <circle
                        cx={rowRight}
                        cy={r.y}
                        r={2.6}
                        className={`topo-port${v === true ? " on" : v === false ? " off" : ""}`}
                      />
                    </g>
                  );
                })}
              </g>
            ))}
            {/* Handshake buses: writer row → readers, one x per signal */}
            {buses.map((bus) => (
              <g
                key={bus.signal}
                className={`${wireClass(bus.lane, " topo-bus")}${bus.safety ? " safety" : ""}`}
                onClick={() => setHighlightLane(bus.lane && highlightLane !== bus.lane ? bus.lane : null)}
              >
                <title>{`${bus.signal} — handshake, ${bus.taps.filter((t) => !t.writer).length} reader(s)`}</title>
                <line x1={bus.x} y1={bus.y0} x2={bus.x} y2={bus.y1} />
                {bus.taps.map((tp, i) => (
                  <g key={i}>
                    <line x1={rowRight} y1={tp.y} x2={bus.x} y2={tp.y} />
                    {tp.writer ? (
                      <rect x={bus.x - 3} y={tp.y - 3} width={6} height={6} className="topo-tap" />
                    ) : (
                      <circle cx={bus.x} cy={tp.y} r={2.5} className="topo-tap" />
                    )}
                  </g>
                ))}
              </g>
            ))}
            {/* I/O wires: row port → field node */}
            {wires.map((w, i) => {
              const path =
                w.jogX === null
                  ? `M ${rowRight} ${w.row.y} H ${fieldX}`
                  : `M ${rowRight} ${w.row.y} H ${w.jogX} V ${w.far.y} H ${fieldX}`;
              return (
                <g
                  key={i}
                  className={`${wireClass(w.row.lane)}${w.row.safety ? " safety" : ""}${w.row.status === "unbound" ? " unbound" : ""}`}
                  onClick={() => pickRow(w.row)}
                >
                  <title>{`${w.row.label} → ${w.far.label}${w.row.channel ? ` on ${rowChannel(w.row)}` : " (unbound)"}`}</title>
                  <path d={path} />
                </g>
              );
            })}
            {/* Functional routes through the left gutter */}
            {fnEdges.map((e, i) => (
              <g key={i} className={wireClass(e.lane, " topo-fn")}>
                <title>{`${e.signal}: program → program`}</title>
                <path d={`M ${rowLeft} ${e.fromY} H ${e.x} V ${e.toY} H ${rowLeft}`} />
                <circle cx={rowLeft} cy={e.toY} r={2} />
              </g>
            ))}
            {/* Field column */}
            {fields.map((f) => {
              const clickable =
                (f.kind === "sensor" && sensors.some((s) => s.name === f.label)) ||
                (f.kind === "device" && devices.some((d) => d.name === f.label)) ||
                f.kind === "robot";
              return (
                <g
                  key={f.id}
                  className={`topo-field topo-field-${f.kind}${clickable ? " clickable" : ""}`}
                  onClick={() => pickField(f)}
                >
                  <title>{`${f.kind}: ${f.label}${clickable ? " — click to select" : ""}`}</title>
                  <rect x={fieldX} y={f.y - ROW_H / 2 + 1} width={FIELD_W} height={ROW_H - 2} rx={f.kind === "sensor" ? 7 : 3} />
                  <text x={fieldX + 6} y={f.y + 3.5}>
                    {KIND_GLYPH[f.kind] ?? "▭"} {trunc(f.label, 18)}
                  </text>
                </g>
              );
            })}
          </svg>
        )}
      </div>
    </div>
  );
}
