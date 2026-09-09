import { useEffect, useMemo, useRef, useState } from "react";
import { robotByName, useStudioStore } from "../store";
import { requestMountingInspection } from "../ws";
import {
  STATUS_LABEL,
  checkFor,
  connectionFindings,
  engagementFor,
  formatRange,
  pairedHole,
  selectionHole,
  sourceHref,
  type Connection,
  type Finding,
  type Inspection,
  type MountingMode,
  type Source,
} from "../mounting";
import { MountingHoles, MountingSection } from "./MountingDiagrams";
import { MountingViewport } from "./MountingViewport";
import { MountingEditor } from "./MountingEditor";
import { MountingKitSummary, MountingProductSummary } from "./MountingKitSummary";
import { MountingRouteSummary } from "./MountingRouteSummary";
import "./mounting.css";

const MODES: { id: MountingMode; label: string }[] = [
  { id: "assembled", label: "Assembled" },
  { id: "exploded", label: "Exploded" },
  { id: "holes", label: "Hole overlay" },
  { id: "section", label: "Screw section" },
  { id: "access", label: "Tool access" },
];
function preferredMode(f: Finding): MountingMode {
  return f.id.endsWith(":dimensions")
    ? "holes"
    : f.id.endsWith(":fasteners")
      ? "section"
      : f.id.endsWith(":assembly_clearance")
        ? "access"
        : "assembled";
}
function title(f: Finding): string {
  for (const [key, label] of [
    ["dimensions", "Dimensions"],
    ["fasteners", "Fasteners"],
    ["assembly_clearance", "Assembly access"],
    ["pose", "Mating pose"],
    ["interface", "Interface"],
    ["requirements", "Required components"],
    ["requirements_coverage", "Parts coverage"],
    ["identity", "Product identity"],
    ["kit_composition", "Kit composition"],
  ])
    if (f.id.endsWith(`:${key}`)) return label;
  return f.message;
}

export function MountingInspector() {
  const open = useStudioStore((s) => s.mountingOpen);
  return open ? <InspectionDialog /> : null;
}

function InspectionDialog() {
  const [editing, setEditing] = useState(false);
  const inspection = useStudioStore((s) => s.mountingInspection),
    error = useStudioStore((s) => s.mountingError);
  const revision = useStudioStore((s) => s.mountingRevision),
    connection = useStudioStore((s) => s.connection);
  const robots = useStudioStore((s) => s.robots),
    initialRobot = useStudioStore((s) => s.selectedRobot);
  const [robotName, setRobotName] = useState(initialRobot ?? ""),
    [target, setTarget] = useState("");
  const [selected, setSelected] = useState(""),
    [findingId, setFindingId] = useState("");
  const [mode, setMode] = useState<MountingMode>("assembled"),
    [explode, setExplode] = useState(35),
    [transparent, setTransparent] = useState(false),
    [hideFlange, setHideFlange] = useState(false),
    [focus, setFocus] = useState(0);
  const dialog = useRef<HTMLDialogElement>(null);
  const close = () => useStudioStore.getState().setMountingOpen(false);
  useEffect(() => {
    dialog.current?.showModal();
    return () => dialog.current?.close();
  }, []);
  useEffect(() => {
    if (connection === "connected") requestMountingInspection();
  }, [revision, connection]);
  useEffect(() => {
    if (!robots.some((r) => r.desc.name === robotName))
      setRobotName(robots[0]?.desc.name ?? "");
  }, [robots, robotName]);
  const robot = robotByName(robots, robotName);
  const connections = useMemo(() => {
    const cs =
      inspection?.connections.filter((c) => c.robot === robotName) ?? [];
    const depth = (c: Connection) => {
      let n = 0,
        p = c.flange.part;
      const seen = new Set<string>();
      while (p && !seen.has(p)) {
        seen.add(p);
        const up = cs.find((x) => x.mount.part === p);
        if (!up) break;
        n++;
        p = up.flange.part;
      }
      return n;
    };
    return cs.slice().sort((a, b) => depth(a) - depth(b));
  }, [inspection, robotName]);
  const c =
    connections.find((x) => x.target === target) ??
    connections.find((x) => x.arm === robot?.selectedGroup) ??
    connections[0];
  useEffect(() => {
    setSelected("");
    setFindingId("");
    setFocus(0);
  }, [c?.target, robotName]);
  const holeKey =
    c && selectionHole(c, selected)
      ? selected
      : c?.flange.holes[0]
        ? `flange:${c.flange.holes[0].id}`
        : c?.mount.holes[0]
          ? `mount:${c.mount.holes[0].id}`
          : "";
  const items = inspection
    ? c
      ? connectionFindings(inspection, c)
      : inspection.findings.filter(
          (f) => f.target === robotName || f.target.startsWith(`${robotName}/`),
        )
    : [];
  const finding =
    items.find((f) => f.id === findingId) ??
    items.find((f) => preferredMode(f) === mode && f.target === c?.target) ??
    items.find((f) => f.status === "fail" || f.status === "unknown") ??
    items[0];
  const kits =
    inspection?.kits.filter((k) =>
      inspection.parts.some(
        (p) =>
          p.kit === k.target &&
          (p.id === c?.flange.part || p.id === c?.mount.part),
      ),
    ) ?? [];
  const products = inspection?.products.filter((p) =>
    p.target.startsWith(`${robotName}/`),
  ) ?? [];
  const name = (id: string | null) =>
    inspection?.parts.find((p) => p.id === id)?.name ??
    id ??
    "Unresolved component";
  const chooseMode = (mode: MountingMode) => {
    setMode(mode);
    setFindingId("");
  };
  const download = () => {
    if (!inspection) return;
    const url = URL.createObjectURL(
      new Blob([JSON.stringify(inspection.rawReport, null, 2)], {
        type: "application/json",
      }),
    );
    const a = document.createElement("a");
    a.href = url;
    a.download = "mounting.json";
    a.click();
    URL.revokeObjectURL(url);
  };
  return (
    <dialog
      ref={dialog}
      className="mounting-dialog"
      aria-labelledby="mounting-title"
      onCancel={(e) => {
        e.preventDefault();
        close();
      }}
    >
      <header className="mounting-header">
        <div>
          <small>ROBOT / MECHANICAL MOUNTING</small>
          <h2 id="mounting-title">Inspect mounting</h2>
        </div>
        <select
          aria-label="Robot to inspect"
          value={robotName}
          onChange={(e) => {
            setRobotName(e.target.value);
            setTarget("");
          }}
        >
          {robots.map((r) => (
            <option key={r.desc.name} value={r.desc.name}>
              {r.desc.name}
            </option>
          ))}
        </select>
        <button
          onClick={requestMountingInspection}
          disabled={connection !== "connected"}
        >
          Refresh
        </button>
        <button onClick={download} disabled={!inspection}>
          Report ↓
        </button>
        <button onClick={() => setEditing(!editing)} disabled={!inspection}>
          {editing ? "Inspect current assembly" : "Compare assemblies"}
        </button>
        <button
          onClick={close}
          aria-label="Close mounting inspection"
          autoFocus
        >
          Close ×
        </button>
      </header>
      {editing ? (
        <MountingEditor
          key={robotName}
          robotName={robotName}
          inspection={inspection}
        />
      ) : !inspection ? (
        <div className="mounting-empty" role="status">
          <b>
            {error ??
              (connection === "connected"
                ? "Inspecting the current composition…"
                : "Waiting for the scene connection…")}
          </b>
          <p>
            Mounting results and their rendering references are loaded together.
          </p>
        </div>
      ) : (
        <>
          {kits.map((kit) => <MountingKitSummary key={kit.target} kit={kit} />)}
          {products.map((product) => <MountingProductSummary key={product.target} product={product} />)}
          <MountingRouteSummary inspection={inspection} robotName={robotName} />
          <div className="mounting-summary">
            <div>
              {inspection.simulation.ready && <>
                <b className="pass">✓ Mounting can be used in simulation</b><br />
              </>}
              <span
                className={
                  !inspection.connections.length
                    ? "unknown"
                    : inspection.ready
                      ? "pass"
                      : inspection.findings.some((f) => f.status === "fail")
                        ? "fail"
                        : "unknown"
                }
              >
                {!inspection.connections.length
                  ? "— No mounting connection to inspect"
                  : inspection.ready
                    ? "✓ Detailed scene mounting checks passed"
                    : inspection.findings.some((f) => f.status === "fail")
                      ? "× Detailed checks found mismatches"
                      : "? Detailed mounting checks incomplete"}
              </span>
            </div>
            <span>
              {inspection.findings.filter((f) => f.status === "fail").length}{" "}
              mismatches ·{" "}
              {
                inspection.findings.filter(
                  (f) => f.status === "unknown" || f.status === "not_run",
                ).length
              }{" "}
              unresolved items
            </span>
          </div>
          <nav className="mounting-chain" aria-label="Mounting connections">
            {connections.map((edge) => (
              <button
                key={edge.target}
                className={edge.target === c?.target ? "active" : ""}
                onClick={() => {
                  setTarget(edge.target);
                  setMode("assembled");
                }}
              >
                <small>{edge.arm ?? "Connection"}</small>
                {name(edge.flange.part)} <b>→</b> {name(edge.mount.part)}
              </button>
            ))}
          </nav>
          {!c || !robot ? (
            <div className="mounting-empty">
              <b>
                {items.length
                  ? "Mounting information is incomplete"
                  : "No tool mounting connection recorded"}
              </b>
              <p>
                {items.length
                  ? "The source does not establish a complete mounting connection. Review the missing information below."
                  : "This scene has no declared tool attachment to inspect."}
              </p>
              {items.map((f) => (
                <div key={f.id}>
                  <b className={f.status}>
                    {STATUS_LABEL[f.status]} · {f.message}
                  </b>
                  <p>{f.next}</p>
                  <Evidence sources={f.sources} />
                </div>
              ))}
            </div>
          ) : (
            <div className="mounting-content">
              <section
                className="mounting-visual-panel"
                aria-label="Mounting geometry"
              >
                <div className="mounting-toolbar">
                  <nav aria-label="Inspection view">
                    {MODES.map((m) => (
                      <button
                        key={m.id}
                        className={m.id === mode ? "active" : ""}
                        onClick={() => chooseMode(m.id)}
                      >
                        {m.label}
                      </button>
                    ))}
                  </nav>
                  <button
                    onClick={() => setFocus((x) => x + 1)}
                    disabled={mode === "holes" || mode === "section"}
                  >
                    Focus hole
                  </button>
                </div>
                <div className="mounting-stage">
                  {mode === "holes" ? (
                    <MountingHoles
                      inspection={inspection}
                      connection={c}
                      selected={holeKey}
                      onSelect={setSelected}
                    />
                  ) : mode === "section" ? (
                    <MountingSection
                      inspection={inspection}
                      connection={c}
                      selected={holeKey}
                    />
                  ) : (
                    <MountingViewport
                      inspection={inspection}
                      connection={c}
                      robot={robot.desc}
                      mode={mode}
                      explode={explode}
                      transparent={transparent}
                      hideFlange={hideFlange}
                      selected={holeKey}
                      onSelect={setSelected}
                      focus={focus}
                    />
                  )}
                  {mode !== "holes" && mode !== "section" && (
                    <div className="mounting-stage-note">
                      {mode === "exploded"
                        ? "Display separation only · not an assembly path"
                        : mode === "access"
                          ? "Declared access volumes · adjacent mounting parts"
                          : "Inspection colors · actual model geometry"}
                      <small>
                        {c.flange.mapped && c.mount.mapped
                          ? "Drawing features mapped to model frames"
                          : "Unconfirmed drawing mapping: frame markers only on the affected side"}
                      </small>
                      <small>
                        Rings and wire shafts show documented dimensions.
                      </small>
                    </div>
                  )}
                </div>
                <div className="mounting-view-controls">
                  {mode === "exploded" && (
                    <label>
                      Separation{" "}
                      <input
                        type="range"
                        min={0}
                        max={150}
                        value={explode}
                        onChange={(e) => setExplode(Number(e.target.value))}
                        aria-label="Display separation"
                      />
                      {explode} mm
                    </label>
                  )}
                  {mode !== "holes" && mode !== "section" && (
                    <label>
                      <input
                        type="checkbox"
                        checked={transparent}
                        onChange={(e) => setTransparent(e.target.checked)}
                      />{" "}
                      Transparent tool
                    </label>
                  )}
                  {mode !== "holes" && mode !== "section" && (
                    <label>
                      <input
                        type="checkbox"
                        checked={hideFlange}
                        onChange={(e) => setHideFlange(e.target.checked)}
                      />{" "}
                      Hide flange geometry
                    </label>
                  )}
                  <span>
                    Presentation changes leave composition, TCP and checks
                    unchanged.
                  </span>
                </div>
                <div className="mounting-legend">
                  <span>● Blue: flange side</span>
                  <span>● Brown: tool side</span>
                  <span>● Purple: selection</span>
                  <span className="fail">× Mismatch</span>
                  <span className="unknown">? Unknown</span>
                </div>
              </section>
              <aside className="mounting-results">
                <details open={mode !== "assembled" || !kits.some((kit) => kit.support === "pass")}>
                <summary>Detailed connection checks</summary>
                <h3>Selected feature</h3>
                <select
                  aria-label="Mounting hole"
                  value={holeKey}
                  onChange={(e) => setSelected(e.target.value)}
                  disabled={!holeKey}
                >
                  {!holeKey && (
                    <option value="">No hole dimensions documented</option>
                  )}
                  {(["flange", "mount"] as const).flatMap((side) =>
                    c[side].holes.map((h) => (
                      <option key={`${side}:${h.id}`} value={`${side}:${h.id}`}>
                        {side} / {h.id} · {h.kind}
                      </option>
                    )),
                  )}
                </select>
                <HoleDetails
                  inspection={inspection}
                  connection={c}
                  selected={holeKey}
                />
                <h3>Connection checks</h3>
                <div className="mounting-findings">
                  {items.map((f) => (
                    <button
                      key={f.id}
                      className={`${f.status}${f.id === finding?.id ? " active" : ""}`}
                      onClick={() => {
                        setFindingId(f.id);
                        setMode(preferredMode(f));
                      }}
                    >
                      <span>{title(f)}</span>
                      <b>{STATUS_LABEL[f.status]}</b>
                    </button>
                  ))}
                </div>
                {finding && (
                  <section className={`mounting-finding ${finding.status}`}>
                    <h3>{finding.message}</h3>
                    {finding.status !== "pass" && <p>{finding.next}</p>}
                    {finding.checks.map((check) => (
                      <details key={check.name}>
                        <summary>
                          <span className={check.status}>
                            {STATUS_LABEL[check.status]}
                          </span>{" "}
                          · {check.name}
                        </summary>
                        <p>{check.message}</p>
                        <pre>{JSON.stringify(check.inputs, null, 2)}</pre>
                      </details>
                    ))}
                    <Evidence sources={finding.sources} />
                  </section>
                )}
                {mode === "access" && (
                  <p className="mounting-note">
                    This check covers declared adjacent parts and access
                    volumes. External equipment and approach motion are outside
                    its scope.
                  </p>
                )}
                </details>
              </aside>
            </div>
          )}
          <footer className="mounting-footer">
            <span>Read-only inspection snapshot · {inspection.validator}</span>
            <code>{inspection.hash}</code>
          </footer>
        </>
      )}
    </dialog>
  );
}

function HoleDetails({
  inspection,
  connection: c,
  selected,
}: {
  inspection: Inspection;
  connection: Connection;
  selected: string;
}) {
  const picked = selectionHole(c, selected);
  if (!picked)
    return (
      <p className="mounting-note">
        The mounting frames can be shown. Hole positions and screw geometry need
        drawing data.
      </p>
    );
  const peer = pairedHole(inspection, c, selected),
    h = picked.hole,
    bolt = engagementFor(inspection, c, selected),
    pattern = checkFor(inspection, c, "hole_pattern");
  return (
    <dl className="mounting-dimensions">
      <div>
        <dt>Position (drawing)</dt>
        <dd title={h.position.join(", ")}>
          {h.position.map((n) => Number(n.toFixed(3))).join(", ")} mm
        </dd>
      </div>
      <div>
        <dt>{h.thread ? "Thread" : "Diameter"}</dt>
        <dd>
          {h.thread
            ? `M${h.thread.diameter}${h.thread.pitch === null ? " · pitch unknown" : ` × ${h.thread.pitch}`}`
            : formatRange(h.diameter)}
        </dd>
      </div>
      <div>
        <dt>Position tolerance</dt>
        <dd>{h.tolerance === null ? "Not documented" : `${h.tolerance} mm`}</dd>
      </div>
      <div>
        <dt>Paired hole</dt>
        <dd>
          {peer
            ? `${peer.side} / ${peer.hole.id}`
            : pattern
              ? "Not established"
              : "Not evaluated"}
        </dd>
      </div>
      <div>
        <dt>Drawing → model</dt>
        <dd className={c[picked.side].mapped ? "pass" : "unknown"}>
          {c[picked.side].mapped ? "Mapped" : "Unconfirmed"}
        </dd>
      </div>
      {bolt && (
        <div>
          <dt>Screw engagement</dt>
          <dd className={bolt.status}>{STATUS_LABEL[bolt.status]}</dd>
        </div>
      )}
    </dl>
  );
}

function Evidence({ sources }: { sources: Source[] }) {
  return (
    <details className="mounting-evidence">
      <summary>Sources and evidence ({sources.length})</summary>
      {sources.length ? (
        sources.map((s, i) => (
          <p key={i}>
            {sourceHref(s.url) ? (
              <a href={sourceHref(s.url)} target="_blank" rel="noreferrer">
                {s.ref || s.url}
              </a>
            ) : (
              <span>{s.ref || s.url}</span>
            )}
            {s.section && <small>{s.section}</small>}
          </p>
        ))
      ) : (
        <p>No supporting source is recorded for this item.</p>
      )}
    </details>
  );
}
