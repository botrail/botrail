import { Fragment, useMemo } from "react";

import { useDockReserve } from "../dockReserve";
import type { RouteMsg, Via } from "../protocol";
import { useStudioStore } from "../store";
import { sendSuggestRoute, sendUpdateRouteVia } from "../ws";
import { OverlayTabs } from "./OverlayTabs";
import { useState } from "react";

function worst(route: RouteMsg): "ok" | "warn" | "bad" {
  if (route.error || route.findings.some((f) => f.severity === "error")) return "bad";
  if (route.findings.some((f) => f.severity === "warning")) return "warn";
  return "ok";
}

const fmt = (v: number | null) => (v === null ? "—" : v.toFixed(2));
const fmt3 = (v: number) => v.toFixed(3);

function describeVia(v: Via, kinds: Record<string, string>): string {
  if (v.kind === "point") return `point (${v.position.map(fmt3).join(", ")})`;
  if (v.kind === "support") return `${kinds[v.name] ?? "support"} ${v.name}`;
  const on = v.on.kind === "obstacle" ? v.on.name : `${v.on.robot}/${v.on.link}`;
  const aside = v.offset_m ? ` ${Math.abs(v.offset_m * 1000).toFixed(0)} mm ${v.offset_m > 0 ? "+" : "−"} aside` : "";
  const inset = v.inset_m && (v.inset_m[0] || v.inset_m[1]) ? ` from ${v.inset_m[0]} m in to ${v.inset_m[1]} m short` : "";
  return `along ${on}${v.face ? ` on its ${v.face} face` : ""}${aside}${inset}${v.pitch_m ? ` every ${v.pitch_m} m` : ""}`;
}

/**
 * One route's way, in order from its source port to its target: each way
 * point a line to move, remove or retype, a menu to add one at the end,
 * and the ways the host suggests through the cell's ducts, to adopt.
 * Every change is sent as the route's whole new way; the host re-resolves
 * it and the routes come back redrawn.
 */
function WayEditor({ route }: { route: RouteMsg }) {
  const supports = useStudioStore((s) => s.routeSupports);
  const obstacles = useStudioStore((s) => s.obstacles);
  const selection = useStudioStore((s) => s.selection);
  const candidates = useStudioStore((s) => s.routeCandidates[route.name]);
  const [adding, setAdding] = useState("");
  const [body, setBody] = useState("");
  const kinds = useMemo(() => Object.fromEntries(supports.map((s) => [s.name, s.kind])), [supports]);
  const way = route.via;
  const commit = (next: Via[]) => sendUpdateRouteVia(route.name, next);
  const replace = (i: number, v: Via) => commit(way.map((w, j) => (j === i ? v : w)));
  const move = (i: number, d: number) => {
    const j = i + d;
    if (j < 0 || j >= way.length) return;
    const next = [...way];
    [next[i], next[j]] = [next[j], next[i]];
    commit(next);
  };
  const remove = (i: number) => commit(way.filter((_, j) => j !== i));
  const selectPoint = (i: number) =>
    useStudioStore.setState({ selection: { type: "route_via", route: route.name, index: i } });
  const addPoint = () => {
    const c = route.centerline;
    const p = c.length ? c[Math.floor(c.length / 2)] : [0, 0, 0.5];
    commit([...way, { kind: "point", position: [p[0], p[1], p[2]] }]);
  };
  const addFromMenu = (value: string) => {
    setAdding("");
    if (!value) return;
    if (value === "point") return addPoint();
    if (value.startsWith("support:")) return commit([...way, { kind: "support", name: value.slice(8) }]);
    if (value.startsWith("along:")) return commit([...way, { kind: "along", on: { kind: "obstacle", name: value.slice(6) } }]);
  };
  const addLink = () => {
    const slash = body.indexOf("/");
    if (slash <= 0 || slash === body.length - 1) return;
    commit([...way, { kind: "along", on: { kind: "link", robot: body.slice(0, slash), link: body.slice(slash + 1) } }]);
    setBody("");
  };
  return (
    <div className="route-editor" style={{ padding: "4px 8px 8px" }}>
      <table className="io-table bom-table" title="the route's way, in order from its source port to its target">
        <thead>
          <tr>
            <th>#</th>
            <th>way point</th>
            <th className="num">x</th>
            <th className="num">y</th>
            <th className="num">z</th>
            <th className="num">pitch m</th>
            <th></th>
          </tr>
        </thead>
        <tbody>
          {way.length === 0 && (
            <tr>
              <td colSpan={7} className="hint">
                no way points — the line runs port to port
              </td>
            </tr>
          )}
          {way.map((v, i) => {
            const picked = selection.type === "route_via" && selection.route === route.name && selection.index === i;
            return (
              <tr
                key={`${i}/${describeVia(v, kinds)}`}
                className={`io-row${v.kind === "point" ? " io-clickable" : ""}${picked ? " bom-picked" : ""}`}
                onClick={() => v.kind === "point" && selectPoint(i)}
                title={v.kind === "point" ? "click to move it in the viewport" : undefined}
              >
                <td className="num">{i + 1}</td>
                <td>{describeVia(v, kinds)}</td>
                {v.kind === "point" ? (
                  ([0, 1, 2] as const).map((k) => (
                    <td key={k} className="num">
                      <input
                        type="number"
                        step="0.01"
                        defaultValue={v.position[k]}
                        style={{ width: "5.5em" }}
                        onClick={(e) => e.stopPropagation()}
                        onBlur={(e) => {
                          const value = parseFloat(e.currentTarget.value);
                          if (!Number.isFinite(value) || value === v.position[k]) return;
                          const position: [number, number, number] = [v.position[0], v.position[1], v.position[2]];
                          position[k] = value;
                          replace(i, { kind: "point", position });
                        }}
                      />
                    </td>
                  ))
                ) : (
                  <>
                    <td />
                    <td />
                    <td />
                  </>
                )}
                <td className="num">
                  {v.kind === "along" ? (
                    <input
                      type="number"
                      step="0.05"
                      min="0.01"
                      defaultValue={v.pitch_m ?? ""}
                      placeholder="0.3"
                      style={{ width: "4.5em" }}
                      onBlur={(e) => {
                        const value = parseFloat(e.currentTarget.value);
                        replace(i, { ...v, pitch_m: Number.isFinite(value) && value > 0 ? value : null });
                      }}
                    />
                  ) : null}
                </td>
                <td style={{ whiteSpace: "nowrap" }}>
                  <button className="timeline-button" title="move up" onClick={(e) => { e.stopPropagation(); move(i, -1); }}>
                    ↑
                  </button>
                  <button className="timeline-button" title="move down" onClick={(e) => { e.stopPropagation(); move(i, 1); }}>
                    ↓
                  </button>
                  <button className="timeline-button" title="remove" onClick={(e) => { e.stopPropagation(); remove(i); }}>
                    ✕
                  </button>
                </td>
              </tr>
            );
          })}
        </tbody>
      </table>
      <div style={{ display: "flex", gap: 6, alignItems: "center", marginTop: 6, flexWrap: "wrap" }}>
        <select className="timeline-button" value={adding} onChange={(e) => addFromMenu(e.target.value)} title="add a way point at the end">
          <option value="">+ way point…</option>
          <option value="point">point</option>
          {supports.map((s) => (
            <option key={s.name} value={`support:${s.name}`}>
              {s.kind} {s.name}
            </option>
          ))}
          {obstacles.slice(0, 200).map((o) => (
            <option key={o.name} value={`along:${o.name}`}>
              along {o.name}
            </option>
          ))}
        </select>
        <input
          value={body}
          onChange={(e) => setBody(e.target.value)}
          placeholder="along robot/link"
          style={{ width: "11em" }}
          onKeyDown={(e) => {
            if (e.key === "Enter") addLink();
          }}
        />
        <button className="timeline-button" onClick={addLink} disabled={!body.includes("/")}>
          + along link
        </button>
        <span style={{ flex: 1 }} />
        <button className="timeline-button" onClick={() => sendSuggestRoute(route.name)} title="the ways this route could take through the cell's ducts">
          candidates
        </button>
      </div>
      {candidates && (
        <table className="io-table bom-table" style={{ marginTop: 6 }} title="the ways this route could take, best first — adopt one">
          <thead>
            <tr>
              <th>#</th>
              <th>ducts</th>
              <th className="num">run m</th>
              <th className="num" title="the run outside any duct">open m</th>
              <th className="num">bends</th>
              <th className="num">supports</th>
              <th>findings</th>
              <th></th>
            </tr>
          </thead>
          <tbody>
            {candidates.map((c, i) => (
              <tr key={i} className={`io-row${c.current ? " bom-picked" : ""}`}>
                <td className="num">{i + 1}</td>
                <td>{c.ducts.length ? c.ducts.join(" → ") : "no duct"}</td>
                <td className="num">{Number.isFinite(c.length_m) ? fmt(c.length_m) : "—"}</td>
                <td className="num">{Number.isFinite(c.open_m) ? fmt(c.open_m) : "—"}</td>
                <td className="num">{c.bends}</td>
                <td className="num">{c.supports}</td>
                <td>
                  {c.errors ? <span className="badge bad">{c.errors} error{c.errors === 1 ? "" : "s"}</span> : null}{" "}
                  {c.warnings ? <span className="badge warn">{c.warnings} warning{c.warnings === 1 ? "" : "s"}</span> : null}{" "}
                  {c.codes.join(", ")}
                </td>
                <td>
                  {c.current ? (
                    <span className="badge muted">current</span>
                  ) : (
                    <button className="timeline-button" onClick={() => commit(c.via)}>
                      adopt
                    </button>
                  )}
                </td>
              </tr>
            ))}
          </tbody>
        </table>
      )}
    </div>
  );
}

/** The route a cycle finding is about: the host names it (a duct or a
 * carrier for a bundle's finding); failing that, its message names the
 * span's holders, and the routes know their anchors. */
function routeOf(message: string, routes: RouteMsg[], named?: string): string | null {
  if (named) return named;
  const m = /^span (.+?) → (.+?) /.exec(message);
  if (!m) return null;
  for (const r of routes) {
    const names = r.anchors.map((a) => a.name);
    if (names.includes(m[1]) && names.includes(m[2])) return r.name;
  }
  return null;
}

/**
 * The cable list over the viewport: every route the host resolved
 * (`Scene::routing_report`) — its line, medium, ends, run / cut /
 * purchase length and the worst of its checks — and the findings under
 * it, each with where on the route it sits. A click on a route lights it
 * in the viewport (and dims the others' aids); a click on a finding
 * lights its route. Lengths follow the residents: move the cabinet and
 * the row changes.
 */
export function RoutingOverlay() {
  const open = useStudioStore((s) => s.routingOpen);
  const setOpen = useStudioStore((s) => s.setRoutingOpen);
  const routes = useStudioStore((s) => s.routes);
  const focus = useStudioStore((s) => s.routeFocus);
  const setFocus = useStudioStore((s) => s.setRouteFocus);
  const docked = useStudioStore((s) => s.playback !== null);
  const cycleFindings = useStudioStore((s) => s.timeline?.routeFindings ?? []);
  const seek = useStudioStore((s) => s.seek);
  const [panel, setPanel] = useState<HTMLDivElement | null>(null);
  const [editing, setEditing] = useState<string | null>(null);
  const playing = useStudioStore((s) => s.playback !== null);
  const designLine = useStudioStore((s) => s.routeDesignLine);
  const setDesignLine = useStudioStore((s) => s.setRouteDesignLine);
  useDockReserve(panel, docked);
  const findings = useMemo(
    () => routes.flatMap((r) => r.findings.map((f) => ({ ...f, route: f.route || r.name }))),
    [routes],
  );
  const total = useMemo(() => routes.reduce((acc, r) => acc + (r.error ? 0 : r.cut_length_m), 0), [routes]);
  const errors =
    findings.filter((f) => f.severity === "error").length +
    routes.filter((r) => r.error).length +
    cycleFindings.filter((f) => f.severity === "error").length;
  const warnings =
    findings.filter((f) => f.severity === "warning").length + cycleFindings.filter((f) => f.severity === "warning").length;

  if (!open) return null;

  const pick = (name: string) => setFocus(focus === name ? null : name);

  return (
    <div className="sfc-overlay io-overlay bom-overlay routing-overlay" ref={setPanel}>
      <div className="sfc-head">
        <OverlayTabs active="routing" />
        <span className="sfc-caption">
          {routes.length === 0
            ? "no routes"
            : `${routes.length} route${routes.length === 1 ? "" : "s"} · ${total.toFixed(2)} m cut` +
              (errors ? ` · ${errors} error${errors === 1 ? "" : "s"}` : "") +
              (warnings ? ` · ${warnings} warning${warnings === 1 ? "" : "s"}` : "")}
        </span>
        {playing && routes.length > 0 && (
          <button
            className={`timeline-button${designLine ? " timeline-button-on" : ""}`}
            onClick={() => setDesignLine(!designLine)}
            title="during a playback the replay's ropes draw the cables and what rides the arm is left out — draw the routes as designed, thin and at the scene's pose, beside them"
          >
            design line
          </button>
        )}
        <button
          className="timeline-button"
          onClick={() => setOpen(false)}
          title="close (ROUTING on the tab strip reopens it)"
        >
          ×
        </button>
      </div>
      <div className="sfc-scroll io-scroll">
        {routes.length === 0 ? (
          <div className="sfc-empty hint">
            no routes yet — declare ports with at= / exit= (bt.connections.port) and routes with bt.routing.route
          </div>
        ) : (
          <>
            <table className="io-table bom-table">
              <thead>
                <tr>
                  <th>#</th>
                  <th>route</th>
                  <th>kind</th>
                  <th>medium</th>
                  <th>spec</th>
                  <th>carries</th>
                  <th>from</th>
                  <th>to</th>
                  <th className="num">run m</th>
                  <th className="num">cut m</th>
                  <th className="num">buy m</th>
                  <th>status</th>
                  <th></th>
                </tr>
              </thead>
              <tbody>
                {routes.map((r, i) => {
                  const ports = r.anchors.filter((a) => a.kind === "port");
                  return (
                    <Fragment key={r.name}>
                    <tr
                      className={`io-row io-clickable${focus === r.name ? " bom-picked" : ""}`}
                      onClick={() => pick(r.name)}
                      title={r.error ?? "click to light this route in the viewport"}
                    >
                      <td className="num">{i + 1}</td>
                      <td>{r.name}</td>
                      <td>{r.kind}</td>
                      <td>{r.medium ?? "—"}</td>
                      <td>{r.spec}</td>
                      <td>{r.carries.join(", ")}</td>
                      <td>{ports[0]?.name ?? "—"}</td>
                      <td>{ports[ports.length - 1]?.name ?? "—"}</td>
                      <td className="num">{r.error ? "—" : fmt(r.length_m)}</td>
                      <td className="num">{r.error ? "—" : fmt(r.cut_length_m)}</td>
                      <td className="num">{r.error ? "—" : fmt(r.purchase_length_m)}</td>
                      <td>
                        <span className={`badge ${worst(r)}`}>
                          {r.error ? "unresolved" : worst(r) === "ok" ? "ok" : worst(r) === "warn" ? "warning" : "error"}
                        </span>
                      </td>
                      <td>
                        <button
                          className={`timeline-button${editing === r.name ? " timeline-button-on" : ""}`}
                          title="the route's way points: move, add, remove, or adopt a candidate"
                          onClick={(e) => {
                            e.stopPropagation();
                            setEditing(editing === r.name ? null : r.name);
                          }}
                        >
                          {editing === r.name ? "done" : "edit"}
                        </button>
                      </td>
                    </tr>
                    {editing === r.name && (
                      <tr className="io-row">
                        <td colSpan={13}>
                          <WayEditor route={r} />
                        </td>
                      </tr>
                    )}
                    </Fragment>
                  );
                })}
              </tbody>
            </table>
            {cycleFindings.length > 0 && (
              <table className="io-table bom-table" title="the moving spans over the baked cycle — click a row to seek to its moment">
                <thead>
                  <tr>
                    <th>over the cycle</th>
                    <th>code</th>
                    <th>route</th>
                    <th>message</th>
                    <th className="num">t s</th>
                  </tr>
                </thead>
                <tbody>
                  {cycleFindings.map((f, i) => (
                    <tr
                      key={`${f.code}/${i}`}
                      className={`io-row io-clickable${focus === routeOf(f.message, routes, f.route) ? " bom-picked" : ""}`}
                      onClick={() => {
                        const route = routeOf(f.message, routes, f.route);
                        if (route) setFocus(route);
                        if (f.t_s !== null && f.t_s !== undefined) seek(f.t_s);
                      }}
                    >
                      <td>
                        <span className={`badge ${f.severity === "error" ? "bad" : f.severity === "warning" ? "warn" : "muted"}`}>
                          {f.severity}
                        </span>
                      </td>
                      <td>{f.code}</td>
                      <td>{routeOf(f.message, routes, f.route) ?? "—"}</td>
                      <td>{f.message}</td>
                      <td className="num">{f.t_s === null || f.t_s === undefined ? "—" : f.t_s.toFixed(2)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            )}
            {findings.length > 0 && (
              <table className="io-table bom-table">
                <thead>
                  <tr>
                    <th>severity</th>
                    <th>code</th>
                    <th>route</th>
                    <th>message</th>
                    <th className="num">at m</th>
                  </tr>
                </thead>
                <tbody>
                  {findings.map((f, i) => (
                    <tr
                      key={`${f.route}/${f.code}/${i}`}
                      className={`io-row io-clickable${focus === f.route ? " bom-picked" : ""}`}
                      onClick={() => pick(f.route)}
                    >
                      <td>
                        <span className={`badge ${f.severity === "error" ? "bad" : f.severity === "warning" ? "warn" : "muted"}`}>
                          {f.severity}
                        </span>
                      </td>
                      <td>{f.code}</td>
                      <td>{f.route}</td>
                      <td>{f.message}</td>
                      <td className="num">{f.s_m === null ? "—" : f.s_m.toFixed(2)}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            )}
          </>
        )}
      </div>
    </div>
  );
}
