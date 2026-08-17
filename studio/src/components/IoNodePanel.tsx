import { useState } from "react";

import { CHANNEL_TEMPLATES, templateChannels } from "../ioEdit";
import type { IoChannel, IoNode, IoNodeKind, IoPointMsg } from "../protocol";
import { useStudioStore } from "../store";
import { sendRemoveIoNode, sendUpsertIoNode } from "../ws";
import { ioNodeKindLabel } from "./SceneTreePanel";
import { Section } from "./Section";

/**
 * The I/O node inspector in Layout: creates controllers and stations
 * (`add_io_node`), and for the node selected in the scene tree edits what
 * a node is — the programs it runs, the robots it drives, its uplink —
 * and its channel table (templates append channels the way `bt.io.di8`
 * does; a channel can be dropped). Bindings are made in the I/O table
 * over the viewport, where the points are. Every edit is one
 * `upsert_io_node`; the server validates and the `io` message returns.
 */

const KINDS: { value: IoNodeKind["kind"]; label: string }[] = [
  { value: "plc", label: "PLC" },
  { value: "robot_controller", label: "robot controller" },
  { value: "remote_io", label: "remote I/O" },
  { value: "safety_plc", label: "safety PLC" },
  { value: "other", label: "other" },
];

function nodeKind(kind: IoNodeKind["kind"], robots: string[], label: string): IoNodeKind {
  switch (kind) {
    case "robot_controller":
      return { kind, robots };
    case "other":
      return { kind, label };
    default:
      return { kind };
  }
}

export function IoNodePanel() {
  const selection = useStudioStore((s) => s.selection);
  const nodes = useStudioStore((s) => s.io.io.nodes);
  const points = useStudioStore((s) => s.io.points);
  const findings = useStudioStore((s) => s.io.findings);
  const robots = useStudioStore((s) => s.robots);
  const sequences = useStudioStore((s) => s.sequences);
  const selectIoNode = useStudioStore((s) => s.selectIoNode);
  const [newName, setNewName] = useState("");
  const [newKind, setNewKind] = useState<IoNodeKind["kind"]>("plc");
  const [base, setBase] = useState("");

  const selected =
    selection.type === "io_node" ? (nodes.find((n) => n.name === selection.name) ?? null) : null;

  const create = () => {
    const name = newName.trim();
    if (!name || nodes.some((n) => n.name === name)) return;
    // A robot controller starts on the first robot — the common one-arm
    // cabinet; the checkboxes below change it.
    const firstRobot = robots[0]?.desc.name;
    sendUpsertIoNode({
      name,
      kind: nodeKind(newKind, firstRobot ? [firstRobot] : [], name),
      programs: [],
      uplink: null,
      channels: newKind === "robot_controller" ? templateChannels("ur", [], "") : [],
      place: null,
      model: null,
    });
    setNewName("");
    selectIoNode(name);
  };

  if (nodes.length === 0 && points.length === 0) return null;

  return (
    <Section
      id="io-node"
      title="I/O nodes"
      badge={nodes.length > 0 ? <span className="badge muted">{nodes.length}</span> : undefined}
    >
      <div className="obstacle-controls">
        <div className="seg">
          <input
            className="io-input"
            placeholder="node name (PLC1, UR, RIO1)"
            value={newName}
            onChange={(e) => setNewName(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === "Enter") create();
            }}
          />
          <select value={newKind} onChange={(e) => setNewKind(e.target.value as IoNodeKind["kind"])}>
            {KINDS.map((k) => (
              <option key={k.value} value={k.value}>
                {k.label}
              </option>
            ))}
          </select>
          <button
            onClick={create}
            disabled={!newName.trim() || nodes.some((n) => n.name === newName.trim())}
            title="add a controller / station (Scene.add_io_node)"
          >
            + Add
          </button>
        </div>
        {selected ? (
          <NodeEditor
            node={selected}
            nodes={nodes}
            points={points}
            robots={robots.map((r) => r.desc.name)}
            sequences={sequences.map((s) => s.name)}
            base={base}
            setBase={setBase}
          />
        ) : (
          <div className="hint">
            {nodes.length === 0
              ? "no I/O nodes yet — add a PLC or a robot controller, then wire points in the I/O table (⚡ I/O)"
              : "select an I/O node in the scene tree to edit it"}
          </div>
        )}
        {selected && findings.some((f) => f.message.includes(selected.name)) && (
          <div className="io-node-findings">
            {findings
              .filter((f) => f.message.includes(selected.name))
              .map((f, i) => (
                <div key={i} className="hint">
                  <span className={`badge ${f.severity === "error" ? "bad" : "muted"}`}>{f.code}</span>{" "}
                  {f.message}
                </div>
              ))}
          </div>
        )}
      </div>
    </Section>
  );
}

function NodeEditor({
  node,
  nodes,
  points,
  robots,
  sequences,
  base,
  setBase,
}: {
  node: IoNode;
  nodes: IoNode[];
  points: IoPointMsg[];
  robots: string[];
  sequences: string[];
  base: string;
  setBase: (v: string) => void;
}) {
  const kind = node.kind;
  const nodeRobots = kind.kind === "robot_controller" ? kind.robots : [];
  const programs = node.programs ?? [];
  const channels: IoChannel[] = node.channels ?? [];
  const commit = (patch: Partial<IoNode>) => sendUpsertIoNode({ ...node, ...patch });
  const boundOn = (c: IoChannel) => points.filter((p) => p.node === node.name && p.channel === c.id);
  const toggle = (list: string[], name: string) =>
    list.includes(name) ? list.filter((x) => x !== name) : [...list, name];
  // Programs claimed by another node are shown but not offered here — one
  // program lives on one controller (`program_multihost` otherwise).
  const elsewhere = (program: string) =>
    nodes.find((n) => n.name !== node.name && (n.programs ?? []).includes(program))?.name ?? null;

  return (
    <div className="obstacle-form">
      <div className="inspector-title" title={node.name}>
        🔌 {node.name}
        <span className="seq-cond"> · {ioNodeKindLabel(kind.kind)}</span>
        {kind.kind === "other" && <span className="seq-cond"> · {kind.label}</span>}
        <button
          className="obstacle-remove"
          style={{ float: "right" }}
          title="remove this node (its bindings go with it)"
          onClick={() => sendRemoveIoNode(node.name)}
        >
          ×
        </button>
      </div>
      <div className="io-node-facts">
        {kind.kind === "robot_controller" && (
          <div className="io-node-row">
            <span className="seq-cond">robots</span>
            <span className="io-checks">
              {robots.map((r) => (
                <label key={r} className="io-filter">
                  <input
                    type="checkbox"
                    checked={nodeRobots.includes(r)}
                    onChange={() =>
                      commit({ kind: { kind: "robot_controller", robots: toggle(nodeRobots, r) } })
                    }
                  />
                  {r}
                </label>
              ))}
            </span>
          </div>
        )}
        <div className="io-node-row">
          <span className="seq-cond" title="the programs this controller runs — overrides the implicit hosting">
            programs
          </span>
          <span className="io-checks">
            {sequences.length === 0 && <span className="io-muted">no sequences yet</span>}
            {sequences.map((p) => {
              const other = elsewhere(p);
              return (
                <label key={p} className="io-filter" title={other ? `runs on ${other}` : undefined}>
                  <input
                    type="checkbox"
                    checked={programs.includes(p)}
                    disabled={other !== null}
                    onChange={() => commit({ programs: toggle(programs, p) })}
                  />
                  {p}
                  {other && <span className="io-muted"> ({other})</span>}
                </label>
              );
            })}
          </span>
        </div>
        <div className="io-node-row">
          <span className="seq-cond" title="hang this node off a controller: its channels then take that controller's points">
            uplink
          </span>
          <select
            value={node.uplink?.parent ?? ""}
            onChange={(e) =>
              commit({
                uplink: e.target.value
                  ? { parent: e.target.value, bus: node.uplink?.bus ?? null }
                  : null,
              })
            }
          >
            <option value="">— none (a controller) —</option>
            {nodes
              .filter((n) => n.name !== node.name)
              .map((n) => (
                <option key={n.name} value={n.name}>
                  {n.name}
                </option>
              ))}
          </select>
          {node.uplink && (
            <input
              className="io-input io-input-short"
              placeholder="bus (PROFINET)"
              value={node.uplink.bus ?? ""}
              onChange={(e) =>
                commit({ uplink: { parent: node.uplink!.parent, bus: e.target.value || null } })
              }
            />
          )}
        </div>
        <div className="io-node-row">
          <span className="seq-cond">model / place</span>
          <input
            className="io-input io-input-short"
            placeholder="model"
            value={node.model ?? ""}
            onChange={(e) => commit({ model: e.target.value || null })}
          />
          <input
            className="io-input io-input-short"
            placeholder="place (frame)"
            value={node.place ?? ""}
            onChange={(e) => commit({ place: e.target.value || null })}
          />
        </div>
      </div>
      <div className="io-node-row io-templates" title="append channels; the base address counts up per channel (%IX0.0 → %IX0.7, %IX1.0 …)">
        {CHANNEL_TEMPLATES.map((t) => (
          <button
            key={t.id}
            className="timeline-button"
            onClick={() => commit({ channels: [...channels, ...templateChannels(t.id, channels, base)] })}
          >
            {t.label}
          </button>
        ))}
        <input
          className="io-input io-input-short"
          placeholder="base (%IX0.0)"
          value={base}
          onChange={(e) => setBase(e.target.value)}
        />
      </div>
      {channels.length === 0 ? (
        <div className="hint">no channels — add a template above</div>
      ) : (
        <table className="io-table io-node-table">
          <thead>
            <tr>
              <th>channel</th>
              <th>kind</th>
              <th>address</th>
              <th>point</th>
              <th />
            </tr>
          </thead>
          <tbody>
            {channels.map((c) => {
              const bound = boundOn(c);
              return (
                <tr key={c.id}>
                  <td>{c.id}</td>
                  <td className="io-muted">{c.kind}</td>
                  <td className="io-muted">
                    {c.address ?? (c.port !== null && c.port !== undefined ? `port ${c.port}` : "")}
                  </td>
                  <td>
                    {bound.length === 0 ? (
                      <span className="io-muted">—</span>
                    ) : (
                      bound.map((p) => `${p.label} (${p.direction === "input" ? "in" : "out"})`).join(", ")
                    )}
                  </td>
                  <td>
                    <button
                      className="tree-toggle"
                      title={bound.length > 0 ? "drop the channel (its binding goes too)" : "drop the channel"}
                      onClick={() => commit({ channels: channels.filter((x) => x.id !== c.id) })}
                    >
                      ×
                    </button>
                  </td>
                </tr>
              );
            })}
          </tbody>
        </table>
      )}
    </div>
  );
}
