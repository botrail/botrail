import { sourceHref, STATUS_LABEL, type Kit, type ProductConnection } from "../mounting";

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
      <ConnectionConditions connection={k.connection} requires={k.requires} />
      <details>
        <summary>Model details, included components and scope</summary>
        <p>Composition: {STATUS_LABEL[k.composition]}</p>
        <p>Detailed fit: {STATUS_LABEL[k.fit]}</p>
        {k.representationNote && <p>{k.representationNote}</p>}
        {k.includes.map((item, index) => (
          <p key={index}>{item.name} · {item.qty ?? "?"}</p>
        ))}
        <p>{k.note}</p>
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

function ConnectionConditions({ connection, requires }: {connection: Kit["connection"]; requires: string[]}) {
  return (
      <div className="mounting-connection">
        <h4>Connection configuration{ connection.profile && ` · ${connection.profile}`}</h4>
        <p>{connection.message}</p>
        {connection.host && <p>Host: {connection.host}</p>}
        <div className="mounting-connection-status">
          {connection.scopes.map(s => <span key={s.name} className={s.status}>{s.name}: {STATUS_LABEL[s.status]}</span>)}
        </div>
        {!!connection.checks.length && <details><summary>Recorded settings and documented conditions</summary>
          <table><thead><tr><th>Host setting</th><th>Recorded</th><th>Condition</th><th>Result</th></tr></thead><tbody>
            {connection.checks.map(c => <tr key={c.field}><td>{c.field}</td><td>{c.actual}</td><td>{c.expected}
              {c.note && <p>{c.note}</p>}
              {c.sources.map((source,i) => <p key={i}><a href={sourceHref(source.url)} target="_blank" rel="noreferrer">{source.section || source.url}</a></p>)}
            </td><td className={c.status}>{STATUS_LABEL[c.status]}</td></tr>)}
          </tbody></table>
        </details>}
        {!!requires.length && <p>Arrange separately: {requires.join("; ")}</p>}
        <p>Settings are declarations. Wiring and mechanical fit have separate checks.</p>
      </div>
  );
}

export function MountingProductSummary({product}: {product: ProductConnection}) {
  return <section className="mounting-kit">
    <small>CATALOG PRODUCT</small><h3>{product.name}</h3>
    {!!product.via.length && <p>Attached adapters: {product.via.join(" → ")}</p>}
    <ConnectionConditions connection={product.connection} requires={product.requires} />
    {product.note && <p>{product.note}</p>}
  </section>;
}
