import { useMemo, useState } from "react";

import { useDockReserve } from "../dockReserve";
import { channelChoices, pointId } from "../ioEdit";
import { signalAt } from "../playbackRig";
import type {
  ChannelKind,
  DeclRole,
  IoBinding,
  IoFindingMsg,
  IoPointMsg,
  StepRefMsg,
} from "../protocol";
import { useStudioStore } from "../store";
import { OverlayTabs } from "./OverlayTabs";
import {
  sendAutoAssignIo,
  sendBindIo,
  sendDeclareIo,
  sendUnbindIo,
  sendUndeclareIo,
} from "../ws";

/**
 * The I/O table over the viewport: every point the cell needs to be
 * built — derived from how the programs read sensors, set signals and
 * command devices — with its channel once bound, the steps that write
 * and wait on it, its status, and (during playback) the live level of
 * the lane behind it. The findings of the report sit under the table.
 *
 * Editing the assignment layer happens here too: a point's channel cell
 * is a select over the channels its host (and the stations uplinked to
 * it) offer, `auto-assign` fills what is left, and the footer declares /
 * undeclares points. Every edit is one client message, validated on the
 * server the way the Python API is, and comes back in the `io` message
 * — nodes are made in the Layout inspector, and `generate_python` writes
 * all of it back as `add_io_node` / `bind_input` / `declare_io`.
 */

/** `UR.DI2 · %IX0.2` — the chip a bound point wears. */
export function channelChip(p: IoPointMsg): string | null {
  if (!p.node || !p.channel) return null;
  return p.address ? `${p.node}.${p.channel} · ${p.address}` : `${p.node}.${p.channel}`;
}

/** Points that wear a chip on a signal lane of `name`: the lane is the
 * point's own name (aspects — `line.index` — have no lane). */
export function chipsForLane(points: IoPointMsg[], name: string): string[] {
  const chips: string[] = [];
  for (const p of points) {
    if (p.name !== name || p.aspect) continue;
    const chip = channelChip(p);
    if (chip && !chips.includes(chip)) chips.push(chip);
  }
  return chips;
}

function stepList(steps: StepRefMsg[]): string {
  return steps.map((s) => `${s.sequence}/${s.name}`).join(", ");
}

/** The first step plus a count — the cell of a table, not a paragraph. */
function StepCell({ steps }: { steps: StepRefMsg[] }) {
  if (steps.length === 0) return <td className="io-muted">—</td>;
  const first = `${steps[0].sequence}/${steps[0].name}`;
  return (
    <td title={stepList(steps)}>
      {first}
      {steps.length > 1 && <span className="io-muted"> +{steps.length - 1}</span>}
    </td>
  );
}

function tally(points: IoPointMsg[]): string {
  const counts = new Map<string, number>();
  for (const p of points) {
    if (p.status === "internal" || p.status === "cosmetic") continue;
    counts.set(p.kind, (counts.get(p.kind) ?? 0) + 1);
  }
  return [...counts.entries()].map(([k, n]) => `${k} ${n}`).join(" · ");
}

function severityRank(f: IoFindingMsg): number {
  return f.severity === "error" ? 0 : f.severity === "warning" ? 1 : 2;
}

export function IoOverlay() {
  const open = useStudioStore((s) => s.ioOpen);
  const setOpen = useStudioStore((s) => s.setIoOpen);
  const io = useStudioStore((s) => s.io);
  const timeline = useStudioStore((s) => s.timeline);
  const playbackTime = useStudioStore((s) => s.playbackTime);
  const sensors = useStudioStore((s) => s.sensors);
  const devices = useStudioStore((s) => s.devices);
  const selectSensor = useStudioStore((s) => s.selectSensor);
  const selectDevice = useStudioStore((s) => s.selectDevice);
  const focusTab = useStudioStore((s) => s.focusTab);
  const docked = useStudioStore((s) => s.playback !== null);
  const [panel, setPanel] = useState<HTMLDivElement | null>(null);
  useDockReserve(panel, docked);
  const [unboundOnly, setUnboundOnly] = useState(false);
  const [showCosmetic, setShowCosmetic] = useState(false);
  const [declName, setDeclName] = useState("");
  const [declRole, setDeclRole] = useState<DeclRole>("input");
  const [declKind, setDeclKind] = useState<ChannelKind | "">("");
  const [declSafety, setDeclSafety] = useState(false);
  const [declPair, setDeclPair] = useState("");
  const nodes = io.io.nodes;
  const bindings = io.io.bindings;
  const decls = io.io.decls;

  /** Bind `p` to `value` (`"node/channel"`), or unbind on `""`. An existing
   * binding of the point keeps its tag / field / polarity. */
  const rebind = (p: IoPointMsg, value: string) => {
    const id = pointId(p);
    const prior = bindings.find(
      (b) =>
        b.point.name === id.name &&
        (b.point.aspect ?? null) === (id.aspect ?? null) &&
        b.point.direction === id.direction,
    );
    if (value === "") {
      sendUnbindIo(id);
      return;
    }
    const slash = value.indexOf("/");
    const node = value.slice(0, slash);
    const channel = value.slice(slash + 1);
    if (prior && prior.node !== node) sendUnbindIo(id, prior.node);
    const binding: IoBinding = {
      ...(prior ?? { point: id, node, channel }),
      point: id,
      node,
      channel,
      auto: false,
    };
    sendBindIo(binding);
  };
  const declare = () => {
    const name = declName.trim();
    if (!name) return;
    sendDeclareIo({
      name,
      role: declRole,
      kind: declKind === "" ? null : declKind,
      safety: declSafety,
      pair: declPair.trim() ? declPair.trim() : null,
      note: null,
    });
    setDeclName("");
    setDeclPair("");
  };

  const rows = useMemo(() => {
    let pts = io.points;
    if (!showCosmetic) pts = pts.filter((p) => p.status !== "cosmetic");
    if (unboundOnly) pts = pts.filter((p) => p.status === "unbound");
    return pts;
  }, [io.points, showCosmetic, unboundOnly]);
  const findings = useMemo(
    () => [...io.findings].sort((a, b) => severityRank(a) - severityRank(b)),
    [io.findings],
  );
  const errors = io.findings.filter((f) => f.severity === "error").length;
  const warnings = io.findings.filter((f) => f.severity === "warning").length;
  const cosmetic = io.points.filter((p) => p.status === "cosmetic").length;
  const unbound = io.points.filter((p) => p.status === "unbound").length;
  const lanes = useMemo(() => {
    const m = new Map<string, { times: number[]; values: boolean[] }>();
    for (const lane of timeline?.signals ?? []) m.set(lane.name, lane);
    return m;
  }, [timeline]);

  if (!open) return null;

  const pick = (p: IoPointMsg) => {
    if (sensors.some((s) => s.name === p.name)) {
      selectSensor(p.name);
      focusTab("obstacle");
    } else if (devices.some((d) => d.name === p.name)) {
      selectDevice(p.name);
      focusTab("obstacle");
    }
  };

  return (
    <div className="sfc-overlay io-overlay" ref={setPanel}>
      <div className="sfc-head">
        <OverlayTabs active="io" />
        <span className="sfc-caption">
          {io.points.length === 0
            ? "no points — programs that read sensors, set signals or command devices derive them"
            : `${io.points.length - cosmetic} points — ${tally(io.points)}${
                unbound > 0 ? ` · ${unbound} unbound` : ""
              }`}
        </span>
        {(errors > 0 || warnings > 0) && (
          <span className={`badge ${errors > 0 ? "bad" : "muted"}`}>
            {errors > 0 ? `${errors} error${errors > 1 ? "s" : ""}` : ""}
            {errors > 0 && warnings > 0 ? " · " : ""}
            {warnings > 0 ? `${warnings} warning${warnings > 1 ? "s" : ""}` : ""}
          </span>
        )}
        <label className="io-filter" title="only points without a channel">
          <input
            type="checkbox"
            checked={unboundOnly}
            onChange={(e) => setUnboundOnly(e.target.checked)}
          />
          unbound
        </label>
        {cosmetic > 0 && (
          <label className="io-filter" title="magazine rows (presentation, not I/O)">
            <input
              type="checkbox"
              checked={showCosmetic}
              onChange={(e) => setShowCosmetic(e.target.checked)}
            />
            cosmetic ({cosmetic})
          </label>
        )}
        {nodes.length > 0 && (
          <button
            className="timeline-button"
            onClick={() => sendAutoAssignIo(false)}
            disabled={unbound === 0}
            title="give every unbound point the first free compatible channel on its host (Scene.auto_assign_io)"
          >
            auto-assign
          </button>
        )}
        <button
          className="timeline-button"
          onClick={() => setOpen(false)}
          title="close (the ⚡ I/O button reopens it)"
        >
          ×
        </button>
      </div>
      <div className="sfc-scroll io-scroll">
        {rows.length === 0 ? (
          <div className="sfc-empty hint">
            {io.points.length === 0
              ? "nothing derived yet"
              : "no points match the filter"}
          </div>
        ) : (
          <table className="io-table">
            <thead>
              <tr>
                <th>point</th>
                <th>dir</th>
                <th>kind</th>
                <th>source</th>
                <th>host</th>
                <th>channel</th>
                <th>tag</th>
                <th>status</th>
                <th>writers</th>
                <th>readers</th>
                <th title="lane level at the playhead (points with a lane, while a bake is loaded)">
                  live
                </th>
              </tr>
            </thead>
            <tbody>
              {rows.map((p) => {
                const lane = p.aspect ? undefined : lanes.get(p.name);
                const live = lane
                  ? signalAt(lane.times, lane.values, playbackTime)
                  : null;
                const chip = channelChip(p);
                const clickable =
                  sensors.some((s) => s.name === p.name) ||
                  devices.some((d) => d.name === p.name);
                return (
                  <tr
                    key={`${p.label}/${p.direction}/${p.host ?? ""}`}
                    className={`io-row io-${p.status}${clickable ? " io-clickable" : ""}`}
                    onClick={() => pick(p)}
                    title={clickable ? "select in the scene tree" : undefined}
                  >
                    <td className="io-point">
                      {p.label}
                      {p.safety && <span className="io-safety" title="safety point"> ⛨</span>}
                    </td>
                    <td className="io-muted">{p.direction === "input" ? "in" : "out"}</td>
                    <td>{p.kind}</td>
                    <td className="io-muted">{p.source}</td>
                    <td className="io-muted">{p.host ?? "—"}</td>
                    <td onClick={(e) => e.stopPropagation()}>
                      <ChannelCell
                        point={p}
                        chip={chip}
                        choices={
                          p.status === "internal" || p.status === "cosmetic"
                            ? []
                            : channelChoices(p, nodes, bindings, io.points)
                        }
                        onChange={(v) => rebind(p, v)}
                      />
                    </td>
                    <td className="io-muted">{p.tag ?? ""}</td>
                    <td className={`io-status io-status-${p.status}`}>{p.status}</td>
                    <StepCell steps={p.writers} />
                    <StepCell steps={p.readers} />
                    <td className="io-live">
                      {live === null ? (
                        <span className="io-muted">—</span>
                      ) : (
                        <span className={live ? "io-on" : "io-off"}>{live ? "●" : "○"}</span>
                      )}
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        )}
        {(nodes.length > 0 || decls.length > 0 || io.points.length > 0) && (
          <div className="io-decls">
            {decls.map((d) => (
              <span key={d.name} className="io-decl" title={d.note ?? undefined}>
                ◇ {d.name}
                <span className="io-muted">
                  {" "}
                  {[d.role, d.kind, d.safety ? "safety" : "", d.pair ? `pair ${d.pair}` : ""]
                    .filter(Boolean)
                    .join(" · ")}
                </span>
                <button
                  className="tree-toggle"
                  title="undeclare"
                  onClick={() => sendUndeclareIo(d.name)}
                >
                  ×
                </button>
              </span>
            ))}
            <span className="io-decl-form" title="declare a point: an exception to the derivation, or one the simulation does not model (a safety door's channels)">
              <input
                className="io-input"
                placeholder="declare… (name)"
                value={declName}
                onChange={(e) => setDeclName(e.target.value)}
                onKeyDown={(e) => {
                  if (e.key === "Enter") declare();
                }}
              />
              <select value={declRole} onChange={(e) => setDeclRole(e.target.value as DeclRole)}>
                <option value="input">input</option>
                <option value="output">output</option>
                <option value="internal">internal</option>
                <option value="exclude">exclude</option>
              </select>
              <select value={declKind} onChange={(e) => setDeclKind(e.target.value as ChannelKind | "")}>
                <option value="">kind (auto)</option>
                {(["di", "do", "ai", "ao", "safe_di", "safe_do", "word"] as ChannelKind[]).map((k) => (
                  <option key={k} value={k}>
                    {k}
                  </option>
                ))}
              </select>
              <label className="io-filter">
                <input type="checkbox" checked={declSafety} onChange={(e) => setDeclSafety(e.target.checked)} />
                safety
              </label>
              <input
                className="io-input io-input-short"
                placeholder="pair"
                value={declPair}
                onChange={(e) => setDeclPair(e.target.value)}
              />
              <button className="timeline-button" onClick={declare} disabled={!declName.trim()}>
                + declare
              </button>
            </span>
          </div>
        )}
        {findings.length > 0 && (
          <div className="io-findings">
            {findings.map((f, i) => (
              <div key={i} className={`io-finding io-finding-${f.severity}`}>
                <span className={`badge ${f.severity === "error" ? "bad" : "muted"}`}>
                  {f.severity}
                </span>
                <span className="io-code">{f.code}</span>
                <span className="io-message" title={stepList(f.at)}>
                  {f.message}
                  {f.at.length > 0 && (
                    <span className="io-muted"> — {stepList(f.at)}</span>
                  )}
                </span>
              </div>
            ))}
          </div>
        )}
      </div>
    </div>
  );
}

/** The channel cell: the chip when nothing can change, else a select over
 * the channels the point may take (its host's and its stations'), used
 * ones named after their point and disabled. */
function ChannelCell({
  point,
  chip,
  choices,
  onChange,
}: {
  point: IoPointMsg;
  chip: string | null;
  choices: ReturnType<typeof channelChoices>;
  onChange: (value: string) => void;
}) {
  if (choices.length === 0) {
    return chip ? (
      <span className="io-chip">{chip}</span>
    ) : (
      <span
        className="io-muted"
        title={
          point.host && point.host.startsWith("<")
            ? "an implicit host has no channels — declare a node with programs=[…] in Python or the Layout inspector"
            : "no compatible channel on this host"
        }
      >
        —
      </span>
    );
  }
  const current = point.node && point.channel ? `${point.node}/${point.channel}` : "";
  return (
    <select
      className={`io-select${current ? "" : " io-select-unbound"}`}
      value={current}
      onChange={(e) => onChange(e.target.value)}
      title="the channel this point is wired to"
    >
      <option value="">— unbound —</option>
      {choices.map((c) => {
        const v = `${c.node}/${c.channel.id}`;
        const label = `${c.node}.${c.channel.id}${c.channel.address ? ` · ${c.channel.address}` : ""}`;
        return (
          <option key={v} value={v} disabled={c.takenBy !== null}>
            {label}
            {c.takenBy ? ` (${c.takenBy})` : ""}
          </option>
        );
      })}
    </select>
  );
}

