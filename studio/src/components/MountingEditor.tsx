import { useEffect, useRef, useState } from "react";
import { editMounting } from "../ws";
import { isWasmMode } from "../backend";
import { useStudioStore } from "../store";
import { record, type Inspection } from "../mounting";
import {
  bomDifference,
  declaredCandidates,
  declaredMass,
  overviewConnection,
  parseProposal,
  ROUTE_LABELS,
  type Proposal,
} from "../mounting-edit";
import { MountingViewport, type ComparisonBounds } from "./MountingViewport";
import { MountingKitSummary, MountingProductSummary } from "./MountingKitSummary";
import { MountingRouteSummary } from "./MountingRouteSummary";

function download(project: unknown, name: string) {
  const url = URL.createObjectURL(
    new Blob([JSON.stringify(project, null, 2)], { type: "application/json" }),
  );
  const a = document.createElement("a");
  a.href = url;
  a.download = `${name}.botrail`;
  a.click();
  URL.revokeObjectURL(url);
}

export function MountingEditor({
  robotName,
  inspection,
}: {
  robotName: string;
  inspection: Inspection | null;
}) {
  const [kind, setKind] = useState(isWasmMode() ? "project" : "catalog");
  const [query, setQuery] = useState("");
  const [revision, setRevision] = useState("");
  const [operation, setOperation] = useState("replace");
  const [flange, setFlange] = useState("");
  const [mount, setMount] = useState("");
  const [prefix, setPrefix] = useState("adapter_");
  const [project, setProject] = useState<unknown>(null);
  const [projectRobot, setProjectRobot] = useState("");
  const [connectionPath, setConnectionPath] = useState("");
  const [proposals, setProposals] = useState<
    { label: string; value: Proposal }[]
  >([]);
  const [selected, setSelected] = useState(0);
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");
  const [notice, setNotice] = useState("");
  const [view, setView] = useState("overview");
  const [exploded, setExploded] = useState(false);
  const [hole, setHole] = useState("");
  const [bounds, setBounds] = useState<(ComparisonBounds | null)[]>([
    null,
    null,
  ]);
  useEffect(() => {
    setBounds([null, null]);
  }, [selected, view]);
  const mounted = useRef(true);
  useEffect(() => {
    mounted.current = true;
    return () => {
      mounted.current = false;
    };
  }, []);
  const connected = useStudioStore((s) => s.connection === "connected");
  const candidate = proposals[selected]?.value;
  const connections =
    inspection?.robots.find((r) => r.name === robotName)?.editConnections ?? [];
  const effectivePath = connections.some((c) => c.path === connectionPath)
    ? connectionPath
    : (connections[0]?.path ?? "");
  const projectRobots = Array.isArray(record(project).robots)
    ? (record(project).robots as { name?: string }[])
    : [];
  const work = async (f: () => Promise<void>) => {
    setBusy(true);
    setError("");
    setNotice("");
    try {
      await f();
    } catch (e) {
      if (mounted.current) setError(String(e));
    } finally {
      if (mounted.current) setBusy(false);
    }
  };
  const preview = () =>
    work(async () => {
      const input =
        kind === "project"
          ? { kind, project, ...(projectRobot ? { robot: projectRobot } : {}) }
          : kind === "package"
            ? { kind, path: query }
            : { kind, query, revision };
      const value = parseProposal(
        await editMounting("preview", {
          robot: robotName,
          input,
          operation,
          flange,
          mount,
          prefix,
          connection_path: effectivePath,
        }),
      );
      if (!mounted.current) return;
      const added = bomDifference(value.before, value.after)
        .filter((row) => row.after > row.before)
        .map((row) => row.name);
      const label = (
        added.length
          ? added
          : value.after.bom.map(
              (row) => row.model ?? row.catalog?.id ?? row.names.join(", "),
            )
      ).join(" + ");
      setProposals((ps) => {
        setSelected(ps.length);
        return [
          ...ps,
          {
            label: label || `Assembly ${ps.length + 1}`,
            value,
          },
        ];
      });
      setView("overview");
    });
  const saveCurrent = () =>
    work(async () => {
      const result = record(await editMounting("save", { robot: robotName }));
      download(result.project, `${robotName}-assembly`);
    });
  const apply = () =>
    candidate &&
    work(async () => {
      const result = record(
        await editMounting("apply", {
          robot: robotName,
          project: candidate.project,
          base_revision: candidate.base_revision,
          candidate_revision: candidate.candidate_revision,
        }),
      );
      if (result.applied && mounted.current) {
        setProposals([]);
        setNotice(
          "Assembly applied. Mounting was checked again; rerun affected motions, toolpaths and sequences.",
        );
      }
    });
  const combinedBounds: ComparisonBounds | null = bounds.every(Boolean)
    ? {
        min: [0, 1, 2].map((i) =>
          Math.min(bounds[0]!.min[i], bounds[1]!.min[i]),
        ) as [number, number, number],
        max: [0, 1, 2].map((i) =>
          Math.max(bounds[0]!.max[i], bounds[1]!.max[i]),
        ) as [number, number, number],
      }
    : null;
  const draw = (after: boolean) => {
    if (!candidate) return null;
    const report = after ? candidate.inspection : candidate.beforeInspection;
    const robot = after ? candidate.robotDesc : candidate.beforeDesc;
    const c =
      view === "overview"
        ? overviewConnection(report, robot)
        : (report.connections.find((c) => c.target === view) ??
          report.connections[report.connections.length - 1] ??
          overviewConnection(report, robot));
    return (
      <section className="assembly-model">
        <h3>{after ? "Candidate" : "Current snapshot"}</h3>
        <MountingViewport
          key={`${candidate.candidate_revision}:${after}:${view}`}
          inspection={report}
          connection={c}
          robot={robot}
          mode={view !== "overview" && exploded ? "exploded" : "assembled"}
          explode={35}
          transparent={false}
          hideFlange={false}
          selected={hole}
          onSelect={setHole}
          focus={0}
          comparison={
            view === "overview"
              ? {
                  bounds: combinedBounds,
                  onBounds: (value) =>
                    setBounds((before) => {
                      const next = [...before];
                      next[after ? 1 : 0] = value;
                      return next;
                    }),
                }
              : undefined
          }
        />
      </section>
    );
  };
  return (
    <div className="assembly-editor">
      <aside className="assembly-picker">
        <h3>Assembly candidates</h3>
        <p>
          Load a product or reuse a saved assembly. Compare the complete
          configuration before applying.
        </p>
        {declaredCandidates(inspection, robotName).length > 0 && (
          <details open>
            <summary>Parts declared by this tool</summary>
            <small>
              These declarations still require a candidate inspection.
            </small>
            {declaredCandidates(inspection, robotName).map((item) => (
              <button
                key={`${item.target}:${item.catalog}`}
                disabled={isWasmMode() || busy}
                onClick={() => {
                  setKind("catalog");
                  setQuery(item.catalog);
                  setOperation("insert_adapter");
                }}
              >
                {item.catalog}
                <small>{item.target}</small>
              </button>
            ))}
          </details>
        )}
        <label>
          Operation
          <select
            aria-label="Operation"
            value={operation}
            onChange={(e) => setOperation(e.target.value)}
          >
            <option value="replace">Replace with saved assembly</option>
            <option value="attach">Attach a tool / adapter</option>
            <option value="insert_adapter" disabled={!connections.length}>
              Insert adapter at a connection
            </option>
          </select>
        </label>
        {operation === "insert_adapter" && (
          <label>
            Tool connection
            <select
              aria-label="Tool connection"
              value={effectivePath}
              onChange={(e) => setConnectionPath(e.target.value)}
            >
              {connections.map((c) => (
                <option key={c.path} value={c.path}>
                  {c.flange} → {c.mount}
                </option>
              ))}
            </select>
          </label>
        )}
        <label>
          Source
          <select
            aria-label="Source"
            value={kind}
            onChange={(e) => {
              setKind(e.target.value);
              setQuery("");
            }}
          >
            <option value="catalog" disabled={isWasmMode()}>
              Catalog product
            </option>
            <option value="package" disabled={isWasmMode()}>
              Local package on server
            </option>
            <option value="project">Saved project (JSON)</option>
          </select>
        </label>
        {kind === "project" ? (
          <>
            <label>
              Assembly file
              <input
                type="file"
                accept=".botrail,.json"
                onChange={(e) => {
                  const f = e.target.files?.[0];
                  if (!f) return;
                  void work(async () => {
                    const p: unknown = JSON.parse(await f.text());
                    setProject(p);
                    setProjectRobot("");
                  });
                }}
              />
            </label>
            {projectRobots.length > 1 && (
              <label>
                Project robot
                <select
                  aria-label="Project robot"
                  value={projectRobot}
                  onChange={(e) => setProjectRobot(e.target.value)}
                >
                  <option value="">Select a robot</option>
                  {projectRobots.map((r) => (
                    <option key={r.name} value={r.name}>
                      {r.name}
                    </option>
                  ))}
                </select>
              </label>
            )}
            <small>
              For bundled ZIP projects, load with Python and save the assembly
              from Studio. Referenced assets must be available to the session.
            </small>
          </>
        ) : (
          <>
            <label>
              {kind === "package" ? "Package directory" : "Product ID or query"}
              <input
                value={query}
                onChange={(e) => setQuery(e.target.value)}
                placeholder={
                  kind === "package"
                    ? "/path/to/catalog/vendor/product/r1"
                    : "vendor / product"
                }
              />
            </label>
            {kind === "catalog" && (
              <label>
                Catalog revision (optional)
                <input
                  value={revision}
                  onChange={(e) => setRevision(e.target.value)}
                  placeholder="Resolved revision is pinned in the proposal"
                />
              </label>
            )}
          </>
        )}
        {operation !== "replace" && (
          <details>
            <summary>Mounting frames</summary>
            <label>
              {operation === "insert_adapter"
                ? "Adapter output flange"
                : "Robot flange"}
              <input
                value={flange}
                onChange={(e) => setFlange(e.target.value)}
                placeholder="Use declared flange"
              />
            </label>
            <label>
              Part mount
              <input
                value={mount}
                onChange={(e) => setMount(e.target.value)}
                placeholder="Use declared mount"
              />
            </label>
            <label>
              Link prefix
              <input
                value={prefix}
                onChange={(e) => setPrefix(e.target.value)}
              />
            </label>
          </details>
        )}
        <button
          className="assembly-primary"
          onClick={preview}
          disabled={
            busy ||
            !connected ||
            (kind === "project" ? !project : !query.trim())
          }
        >
          {busy ? "Working…" : "Preview candidate"}
        </button>
        <button onClick={saveCurrent} disabled={busy || !connected}>
          Save current assembly ↓
        </button>
        {error && (
          <p role="alert" className="assembly-error">
            {error}
          </p>
        )}
        {notice && <p role="status">{notice}</p>}
        <div className="assembly-candidates">
          {proposals.map((p, i) => (
            <button
              key={i}
              aria-pressed={selected === i}
              onClick={() => {
                setSelected(i);
                setView("overview");
              }}
            >
              <b>{p.label}</b>
              <span>{ROUTE_LABELS[p.value.route]}</span>
              <small>
                {p.value.can_apply
                  ? "Available for simulation"
                  : "Unresolved — draft available"}
              </small>
            </button>
          ))}
        </div>
        {!!inspection?.revalidation.length && (
          <p className="assembly-error">
            Needs fresh evaluation: {inspection.revalidation.join(", ")}
          </p>
        )}
      </aside>
      <main className="assembly-comparison">
        {!candidate ? (
          <div className="assembly-welcome">
            <h3>Compare mounting configurations</h3>
            <p>
              The current scene stays in place while you inspect a separate
              candidate.
            </p>
            <p>
              Direct mounting evidence · Adapter route evidence · Needs design /
              information
            </p>
          </div>
        ) : (
          <>
            <div className="assembly-actions">
              <div>
                <b>{ROUTE_LABELS[candidate.route]}</b>
                <small>
                  Mounting evidence and detailed fit are separate checks.
                </small>
              </div>
              <button
                onClick={() =>
                  download(candidate.project, `${robotName}-proposal`)
                }
              >
                Save proposal ↓
              </button>
              <button
                className="assembly-primary"
                disabled={!candidate.can_apply || busy || !connected}
                onClick={apply}
              >
                Apply to simulation
              </button>
            </div>
            {candidate.inspection.kits.map((kit) => (
              <MountingKitSummary key={kit.target} kit={kit} />
            ))}
            {candidate.inspection.products.map((product) => (
              <MountingProductSummary key={product.target} product={product} />
            ))}
            <div className="assembly-view-options">
              <label>
                View{" "}
                <select
                  aria-label="View"
                  value={view}
                  onChange={(e) => setView(e.target.value)}
                >
                  <option value="overview">Whole assembly</option>
                  {candidate.inspection.connections.map((c) => (
                    <option key={c.target} value={c.target}>
                      {c.flange.frame} → {c.mount.frame}
                    </option>
                  ))}
                </select>
              </label>
              <label>
                <input
                  type="checkbox"
                  checked={exploded}
                  onChange={(e) => setExploded(e.target.checked)}
                  disabled={view === "overview"}
                />{" "}
                Explode connection
              </label>
            </div>
            <div className="assembly-models">
              {draw(false)}
              {draw(true)}
            </div>
            <div className="assembly-differences">
              <table>
                <thead>
                  <tr>
                    <th>Change</th>
                    <th>Current</th>
                    <th>Candidate</th>
                  </tr>
                </thead>
                <tbody>
                  <tr>
                    <th>TCP link</th>
                    <td>{candidate.before.tcp.link}</td>
                    <td>{candidate.after.tcp.link}</td>
                  </tr>
                  <tr>
                    <th>TCP position · mm (world)</th>
                    {[candidate.before, candidate.after].map((s, i) => (
                      <td key={i}>
                        {s.tcp.pose.position
                          .map((v) => (v * 1000).toFixed(1))
                          .join(", ")}
                      </td>
                    ))}
                  </tr>
                  <tr>
                    <th>Declared BOM mass</th>
                    <td>{declaredMass(candidate.before.mass)}</td>
                    <td>{declaredMass(candidate.after.mass)}</td>
                  </tr>
                  <tr>
                    <th>Detailed mounting checks</th>
                    {[candidate.beforeInspection, candidate.inspection].map(
                      (report, i) => (
                        <td key={i}>
                          {!report.connections.length
                            ? "No tool connection"
                            : report.ready
                              ? "Required checks resolved"
                              : `${report.findings.filter((f) => f.required && f.status === "fail").length} failed / ${report.findings.filter((f) => f.required && (f.status === "unknown" || f.status === "not_run")).length} unresolved`}
                        </td>
                      ),
                    )}
                  </tr>
                  <tr>
                    <th>Model links</th>
                    <td>{candidate.before.links}</td>
                    <td>{candidate.after.links}</td>
                  </tr>
                  {bomDifference(candidate.before, candidate.after).map(
                    (r, i) => (
                      <tr
                        key={i}
                        className={
                          r.before !== r.after ? "assembly-changed" : ""
                        }
                      >
                        <th>{r.name}</th>
                        <td>{r.before}</td>
                        <td>{r.after}</td>
                      </tr>
                    ),
                  )}
                </tbody>
              </table>
              <div className="assembly-findings">
                <h3>Simulation application</h3>
                <MountingRouteSummary inspection={candidate.inspection} robotName={robotName} />
                {!candidate.inspection.connections.length && (
                  <p>No tool mounting connection recorded.</p>
                )}
                {candidate.inspection.findings
                  .filter((f) => candidate.mounting_blockers.includes(f.id))
                  .map((f) => (
                    <p key={f.id} className="assembly-error">
                      <b>{f.target}</b>
                      <span>{f.message}</span>
                      <small>{f.next}</small>
                    </p>
                  ))}
                <details>
                <summary>Detailed mounting findings</summary>
                {candidate.inspection.findings
                  .filter(
                    (f) =>
                      f.required &&
                      f.status !== "pass" &&
                      f.status !== "not_applicable",
                  )
                  .map((f) => (
                    <p key={f.id}>
                      <b>
                        {f.status.toUpperCase()} · {f.target}
                      </b>
                      <span>{f.message}</span>
                      <small>{f.next}</small>
                    </p>
                  ))}
                </details>
                {candidate.can_apply && (
                  <p>
                    {candidate.inspection.ready
                      ? "Required mounting checks are resolved."
                      : "The documented mounting configuration can be used in simulation. Missing detail remains unverified."}
                    {" "}Application checks this snapshot again.
                  </p>
                )}
                {candidate.blockers.map((b, i) => (
                  <p className="assembly-error" key={i}>
                    {b}
                  </p>
                ))}
                <p>
                  Pose retained:{" "}
                  {candidate.preserved_joints.join(", ") || "none"}. Reset to
                  neutral: {candidate.reset_joints.join(", ") || "none"}.
                </p>
                {!!candidate.revalidation.length && (
                  <p>
                    Re-evaluate after apply: {candidate.revalidation.join(", ")}
                  </p>
                )}
                <small>
                  Re-evaluate motions and collision after changing tools.
                </small>
              </div>
            </div>
          </>
        )}
      </main>
    </div>
  );
}
