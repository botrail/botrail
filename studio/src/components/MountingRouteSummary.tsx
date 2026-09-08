import type { Inspection } from "../mounting";

const METHODS: Record<string, string> = {
  direct: "Direct mounting",
  catalog_adapter: "Via catalog parts",
  custom_adapter: "Via custom parts",
  unknown: "Connection path unconfirmed",
};
const BASES: Record<string, string> = {
  manufacturer_kit: "Manufacturer kit",
  interface_declarations: "Documented interfaces, pose and parts",
  dimensional_checks: "Dimensions and fasteners checked",
  unknown: "Mounting evidence incomplete",
};

/** Display the Rust result; the route and evidence do not override findings. */
export function MountingRouteSummary({ inspection, robotName }: {
  inspection: Inspection; robotName: string;
}) {
  const routes = inspection.simulation.connections.filter((c) =>
    c.target.startsWith(`${robotName}/`));
  if (!routes.length) return null;
  return <section className="mounting-routes" aria-label="Connection routes and evidence">
    {routes.map((c) => <div key={c.target}>
      <b>{c.target}</b>
      <span>{METHODS[c.method] ?? METHODS.unknown}</span>
      <span>{BASES[c.basis] ?? BASES.unknown}</span>
    </div>)}
    <small>Evidence for each connection. Detailed findings remain below.</small>
  </section>;
}
