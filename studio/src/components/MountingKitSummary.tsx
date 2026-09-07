import { sourceHref, STATUS_LABEL, type Kit } from "../mounting";

/** Product support stays visible; model precision and detailed findings remain
 * available without presenting missing internal dimensions as incompatibility. */
export function MountingKitSummary({ kit: k }: { kit: Kit }) {
  return (
    <section className="mounting-kit">
      <small>PURCHASE KIT · ONE PURCHASE UNIT</small>
      <h3>{k.name}</h3>
      <b className={k.support === "pass" ? "pass" : "unknown"}>
        {k.support === "pass"
          ? "✓ Manufacturer-documented mounting"
          : k.composition === "fail" || k.support === "not_applicable"
            ? "Supported configuration changed"
            : "No manufacturer support recorded for this host"}
      </b>
      <p>
        {k.correspondence === "pass"
          ? "Model correspondence verified"
          : k.representation === "reference"
            ? "Reference model · precision not fully verified"
            : "Model correspondence unconfirmed"}
      </p>
      <details>
        <summary>Model details, included components and scope</summary>
        <p>Composition: {STATUS_LABEL[k.composition]}</p>
        <p>Detailed fit: {STATUS_LABEL[k.fit]}</p>
        {k.representationNote && <p>{k.representationNote}</p>}
        {k.includes.map((item, index) => (
          <p key={index}>{item.name} · {item.qty ?? "?"}</p>
        ))}
        <p>{k.note}</p>
        <p>Electrical/control: {STATUS_LABEL[k.electrical]}</p>
        {k.sources.map((source, index) => (
          <p key={index}>
            <a href={sourceHref(source.url)} target="_blank" rel="noreferrer">
              {source.section || source.url}
            </a>
          </p>
        ))}
      </details>
    </section>
  );
}
