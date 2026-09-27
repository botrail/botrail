import { useEffect, useMemo, useState, type ReactNode } from "react";

import { cacheKey, revisionKind, viewerUrl, withHubSpecs, type CatalogInfo } from "../catalogHub";
import {
  identityFor,
  robotStack,
  shortId,
  shortRevision,
  type BomIndex,
  type Identity,
} from "../partIdentity";
import type { BomLineMsg, BomTarget, CatalogRef } from "../protocol";
import { allSpecs, headlineSpecs, type SpecItem } from "../specFormat";
import { useStudioStore, type CatalogEntry, type Selection } from "../store";
import { Section } from "./Section";

/**
 * What the selected thing *is*: the line of the bill of materials behind
 * it, as the host derives it. One section for every kind of selection —
 * a robot, an obstacle or the group it sits in, a sensor, a device, a
 * camera, a scanner, an I/O node, a derived line (`arm/tool`) — placed
 * right under the scene tree, so a click there (or in the viewport)
 * answers "what is this" before the kind's own editor below. Display
 * only: identity is authored from Python.
 */

/** The catalog viewer's product page (its direct static host — the
 * hf.co/spaces wrapper is an iframe the hash may not reach). */
export function catalogViewerUrl(id: string): string {
  return viewerUrl(id);
}

/** The hub's manifest for a reference, read on first use and kept; null
 * for a line with no reference or a local package. */
export function useCatalog(ref: CatalogRef | null): CatalogEntry | null {
  const id = ref?.id ?? null;
  const revision = ref?.revision ?? null;
  const key = id ? cacheKey({ id, revision }) : null;
  const entry = useStudioStore((s) => (key ? (s.catalog[key] ?? null) : null));
  const load = useStudioStore((s) => s.loadCatalog);
  useEffect(() => {
    if (id) load({ id, revision });
  }, [id, revision, load]);
  return entry;
}

/** The manifest's picture — a rendered thumbnail, a spec pack's plan — or
 * the category's glyph until it arrives (or when it never does). */
export function Thumb({ info, category }: { info: CatalogInfo | null; category: string }) {
  const [broken, setBroken] = useState<string | null>(null);
  const picture = info?.picture ?? null;
  if (picture && broken !== picture.url) {
    return (
      <img
        className="part-thumb"
        src={picture.url}
        alt=""
        loading="lazy"
        title={picture.kind === "layout" ? "plan view" : "catalog thumbnail"}
        onError={() => setBroken(picture.url)}
      />
    );
  }
  return (
    <div className="part-thumb glyph" title={category}>
      {categoryGlyph(category)}
    </div>
  );
}

/** A glyph for the thumbnail box until the hub's picture arrives (P2). */
export function categoryGlyph(category: string): string {
  const head = category.split(".")[0];
  switch (head) {
    case "manipulator":
    case "robot":
      return "\u{1F916}";
    case "tool":
    case "gripper":
      return "\u{1F527}";
    case "conveyor":
    case "axis":
    case "external_axis":
    case "lift":
      return "⚙";
    case "sensor":
      return category.startsWith("sensor.camera")
        ? "\u{1F3A5}"
        : category.startsWith("sensor.lidar")
          ? "\u{1F300}"
          : "\u{1F4E1}";
    case "plc":
    case "io":
    case "robot_controller":
    case "power_supply":
      return "\u{1F50C}";
    case "vehicle":
      return "\u{1F69A}";
    case "structure":
    case "pallet":
    case "bin":
      return "▦";
    default:
      return "▢";
  }
}

/** The line's name as a card leads with: model (a part number or a
 * product name), else the product part of the catalog id. */
export function titleOf(line: BomLineMsg): string {
  return line.model ?? (line.catalog ? shortId(line.catalog.id) : null) ?? "unidentified";
}

/** Series name and maker under the title (`Belt Conveyor Unit · botrail`);
 * the category when the line says nothing else. */
export function subtitleOf(line: BomLineMsg): string {
  return [line.description, line.manufacturer].filter(Boolean).join(" · ") || line.category;
}

/** How a prim relates to the line it inherited: one of a group's twelve
 * panels, or the body of a device or sensor. */
export function relationTitle(identity: Identity): string | null {
  if (!identity.inherited) return null;
  return identity.line.qty > 1
    ? `one of ${identity.line.qty} — ${identity.ancestor}`
    : `part of ${identity.ancestor}`;
}

/** The quantity and unit a line is bought in, when worth saying. */
export function qtyText(line: BomLineMsg): string | null {
  const unit = line.order?.unit ?? "each";
  if (line.qty === 1 && unit === "each") return null;
  return unit === "each" ? `×${line.qty}` : `×${line.qty} ${unit}`;
}

/** What the catalog reference's revision says about the pin. */
export function revisionState(line: BomLineMsg): "pinned" | "local" | "unpinned" | null {
  if (!line.catalog) return null;
  const rev = line.catalog.revision;
  if (!rev) return "unpinned";
  return rev.startsWith("local-sha256:") ? "local" : "pinned";
}

/** Category and state chips: the line's category, `unidentified` in amber,
 * `derived` for a line only the bill names, `local package` / `unpinned`
 * for a catalog reference that does not pin a dataset revision — and,
 * once the hub has answered, the validation level, a distribution other
 * than public and a lifecycle other than active. */
export function StateChips({
  line,
  target,
  info = null,
  compact = false,
}: {
  line: BomLineMsg;
  target: BomTarget;
  info?: CatalogInfo | null;
  compact?: boolean;
}) {
  const state = revisionState(line);
  return (
    <>
      {!compact && <span className="badge muted">{line.category}</span>}
      {!line.identified && <span className="badge warn">unidentified</span>}
      {line.identified && target.derived && <span className="badge muted">derived</span>}
      {state === "local" && <span className="badge muted">local package</span>}
      {state === "unpinned" && <span className="badge muted">unpinned</span>}
      {info?.level && (
        <span className="badge muted" title={`catalog validation level${info.reviewed ? ` — reviewed ${info.reviewed}` : ""}`}>
          {info.level}
        </span>
      )}
      {info?.distribution && <span className="badge warn">{info.distribution}</span>}
      {info?.lifecycle && <span className="badge warn">{info.lifecycle}</span>}
    </>
  );
}

/** The hub's side of a card: the maker's page, the datasheets and
 * repositories the package was built from, the licence of each set of
 * files with its note (the use restrictions live nowhere else), the
 * review, and what a reference model's geometry is not. */
export function References({
  line,
  hub,
  retry,
}: {
  line: BomLineMsg;
  hub: CatalogEntry | null;
  retry: () => void;
}) {
  const [more, setMore] = useState(false);
  if (!line.catalog || revisionKind(line.catalog.revision) === "local") return null;
  const SHOWN = 6;
  return (
    <div className="part-block">
      <div className="h">references</div>
      {!hub || hub.status === "loading" ? (
        <div className="part-note">fetching catalog…</div>
      ) : hub.status === "failed" ? (
        <div className="part-note" title={hub.error}>
          catalog unreachable{" "}
          <a
            href="#"
            onClick={(e) => {
              e.preventDefault();
              retry();
            }}
          >
            ↻
          </a>
        </div>
      ) : (
        <ReferenceBody info={hub.info} more={more} setMore={setMore} shown={SHOWN} />
      )}
    </div>
  );
}

function ReferenceBody({
  info,
  more,
  setMore,
  shown,
}: {
  info: CatalogInfo;
  more: boolean;
  setMore: (v: boolean) => void;
  shown: number;
}) {
  const sources = more ? info.sources : info.sources.slice(0, shown);
  const hidden = info.sources.length - sources.length;
  return (
    <>
      {(info.maker?.url || info.sources.length > 0) && (
        <div className="part-chips">
          {info.maker?.url && (
            <span className="chip">
              <a href={info.maker.url} target="_blank" rel="noreferrer" title={info.maker.url}>
                product page ↗
              </a>
            </span>
          )}
          {sources.map((s) => (
            <span className="chip" key={`${s.label}-${s.url}`}>
              <a
                href={s.url}
                target="_blank"
                rel="noreferrer"
                title={[s.kind, s.url, s.ref ? `@ ${s.ref}` : null, s.fetchedAt ? `fetched ${s.fetchedAt}` : null]
                  .filter(Boolean)
                  .join(" · ")}
              >
                {s.label} ↗
              </a>
            </span>
          ))}
          {hidden > 0 && (
            <span className="chip part-fold" onClick={() => setMore(true)}>
              +{hidden} more
            </span>
          )}
        </div>
      )}
      {info.licenses.map((l, i) => (
        <div key={`${l.spdx}-${i}`}>
          <div className="part-chips">
            <span className="chip" title={`files: ${l.files}`}>
              {l.spdx}
            </span>
            <span className={`chip ${l.redistributable ? "ok" : "bad"}`}>
              {l.redistributable ? "redistributable ✓" : "not redistributable ✗"}
            </span>
            {info.licenses.length > 1 && <span className="part-note">{l.files}</span>}
          </div>
          {l.note && <div className="part-note">{l.note}</div>}
        </div>
      ))}
      {info.reviewed && <div className="part-note">reviewed {info.reviewed}</div>}
      {info.fidelity && (
        <div className="part-note" title={info.fidelity}>
          model: {info.fidelity}
        </div>
      )}
      {!info.maker?.url && info.sources.length === 0 && info.licenses.length === 0 && (
        <div className="part-note">the catalog states no sources for this product</div>
      )}
    </>
  );
}

/** `universal_robots/ur/ur5e/r2 @1d6bfc1 ↗` — click copies the full
 * reference, ↗ opens the catalog viewer's page. */
export function IdLine({ line }: { line: BomLineMsg }) {
  const [copied, setCopied] = useState(false);
  if (!line.catalog) {
    return <div className="part-id">{line.identified ? "no catalog reference" : "no maker, model or catalog reference"}</div>;
  }
  const { id, revision } = line.catalog;
  const rev = shortRevision(revision);
  const full = revision ? `${id}@${revision}` : id;
  const copy = () => {
    try {
      void navigator.clipboard?.writeText(full);
      setCopied(true);
      window.setTimeout(() => setCopied(false), 1200);
    } catch {
      // No clipboard (an insecure context): the tooltip still shows it.
    }
  };
  return (
    <div className="part-id">
      <span className="part-id-text" title={`${full} — click to copy`} onClick={copy}>
        {id}
        {rev && <span className="part-rev"> @{rev}</span>}
      </span>
      {copied && <span className="part-copied">copied</span>}
      <a
        href={catalogViewerUrl(id)}
        target="_blank"
        rel="noreferrer"
        title="open in the catalog viewer"
      >
        ↗
      </a>
    </div>
  );
}

function SpecGrid({ specs }: { specs: SpecItem[] }) {
  return (
    <div className="part-specs">
      {specs.map((s) => (
        <div className="part-kv" key={s.key}>
          <span className="k" title={s.key}>
            {s.label}
          </span>
          <span className="v">{s.value}</span>
        </div>
      ))}
    </div>
  );
}

function Fold({
  label,
  open,
  onToggle,
  children,
}: {
  label: string;
  open: boolean;
  onToggle: () => void;
  children?: ReactNode;
}) {
  return (
    <div>
      <div className="part-fold" onClick={onToggle}>
        {open ? "▾" : "▸"} {label}
      </div>
      {open && children}
    </div>
  );
}

/** The Python that would identify an unidentified line. */
function pinHint(target: BomTarget, line: BomLineMsg): string {
  const kind =
    target.kind === "obstacle" || target.kind === "robot" || target.kind === "group"
      ? ""
      : `, kind="${target.kind}"`;
  return `scene.set_part("${target.name}"${kind}, manufacturer=…, model=…)\nbt.catalog.search("${line.category}", …)   # or search_for(scene.requirements()["${target.name}"])`;
}

/** The card for one line of the bill. A robot's card lists the lines the
 * bill hangs off it (tools, the controller) as sub-cards of the same
 * shape. */
export function PartCard({
  identity,
  index,
  depth = 0,
}: {
  identity: Identity;
  index: BomIndex;
  depth?: number;
}) {
  const { line: wireLine, target, inherited, ancestor } = identity;
  const [allOpen, setAllOpen] = useState(false);
  const [setOpen, setSetOpen] = useState(false);
  const hub = useCatalog(wireLine.catalog);
  const info = hub?.status === "ok" ? hub.info : null;
  const load = useStudioStore((s) => s.loadCatalog);
  // The hub's text specs (`ip_rating`) join the scene's numbers on the card.
  const line = useMemo(() => withHubSpecs(wireLine, info), [wireLine, info]);
  const headline = useMemo(() => headlineSpecs(line, 4), [line]);
  const every = useMemo(() => allSpecs(line), [line]);
  const stack = useMemo(
    () => (target.kind === "robot" && depth === 0 ? robotStack(index, target.name) : null),
    [index, target, depth],
  );
  const qty = qtyText(line);
  const relation = relationTitle(identity);
  const title = relation ?? titleOf(line);
  const subtitle = inherited
    ? [titleOf(line), subtitleOf(line)].filter(Boolean).join(" · ")
    : subtitleOf(line);

  return (
    <div className="part-card">
      <div className="part-head">
        <Thumb info={info} category={line.category} />
        <div className="part-head-text">
          <div className="part-title" title={title}>
            {title}
          </div>
          {(subtitle || qty) && (
            <div className="part-sub">
              {subtitle}
              {subtitle && qty ? " · " : ""}
              {qty}
            </div>
          )}
          <IdLine line={line} />
        </div>
      </div>

      {headline.length > 0 && <SpecGrid specs={allOpen ? every : headline} />}
      {every.length > headline.length && (
        <Fold
          label={allOpen ? "headline only" : `all attributes (${every.length})`}
          open={allOpen}
          onToggle={() => setAllOpen(!allOpen)}
        />
      )}

      {line.order && line.order.includes.length > 0 && (
        <Fold
          label={`${line.order.unit === "set" ? "set" : "order"} includes ${line.order.includes.length} — ${line.order.includes.map((i) => i.name).join(", ")}`}
          open={setOpen}
          onToggle={() => setSetOpen(!setOpen)}
        >
          <ul className="part-list">
            {line.order.includes.map((i) => (
              <li key={i.name}>
                {i.name}
                {i.qty !== 1 ? ` ×${i.qty}` : ""}
                {i.part_number ? <span className="muted"> · {i.part_number}</span> : null}
                {i.note ? <div className="part-note">{i.note}</div> : null}
              </li>
            ))}
            {line.order.note && <li className="part-note">{line.order.note}</li>}
          </ul>
        </Fold>
      )}
      {line.order && line.order.requires.length > 0 && (
        <div className="part-note">
          requires:{" "}
          {line.order.requires.map((r, i) => (
            <span key={`${r.name}-${i}`}>
              {i > 0 ? ", " : ""}
              {r.catalog ? (
                <a href={catalogViewerUrl(r.catalog)} target="_blank" rel="noreferrer">
                  {r.name} ↗
                </a>
              ) : (
                r.name
              )}
              {r.qty !== 1 ? ` ×${r.qty}` : ""}
            </span>
          ))}
        </div>
      )}

      {stack && (stack.tools.length > 0 || stack.controller) && (
        <div className="part-block">
          <div className="h">stack</div>
          {stack.tools.map((t) => (
            <StackRow key={t.name} icon={"\u{1F527}"} label={t.name.slice(target.name.length + 1)} hit={t} index={index} depth={depth} />
          ))}
          {stack.controller && (
            <StackRow icon={"\u{1F50C}"} label="controller" hit={stack.controller} index={index} depth={depth} />
          )}
        </div>
      )}

      <References
        line={line}
        hub={hub}
        retry={() => wireLine.catalog && load(wireLine.catalog, true)}
      />

      {inherited && (
        <div className="part-note">
          {line.qty > 1 ? (
            <>
              the pin sits on the group <code>{ancestor}</code>; every object under it is one of its {line.qty}
            </>
          ) : (
            <>
              the body of <code>{ancestor}</code> — the line is the {identity.target.kind}'s
            </>
          )}
        </div>
      )}
      {!line.identified && (
        <>
          <div className="part-note">
            No maker, model or catalog reference — the bill lists this line as <code>{line.category}</code> and{" "}
            <code>bom.unidentified()</code> names it. Pin it from Python:
          </div>
          <pre className="part-code">{pinHint(target, line)}</pre>
        </>
      )}
    </div>
  );
}

/** One line of a robot's stack: a row that opens into its own card. */
function StackRow({
  icon,
  label,
  hit,
  index,
  depth,
}: {
  icon: string;
  label: string;
  hit: { line: BomLineMsg; target: BomTarget };
  index: BomIndex;
  depth: number;
}) {
  const [open, setOpen] = useState(false);
  const { line, target } = hit;
  const lead = headlineSpecs(line, 1)[0];
  return (
    <div>
      <div className="part-line" onClick={() => setOpen(!open)} title={`${target.name} — a line of the bill`}>
        <span>{icon}</span>
        <span className="name">
          {label} <b>{titleOf(line)}</b>
          {line.manufacturer ? <span className="muted"> {line.manufacturer}</span> : null}
        </span>
        {lead && <span className="muted">{`${lead.label} ${lead.value}`}</span>}
        {!line.identified && <span className="badge warn">?</span>}
        <span className="part-fold">{open ? "▾" : "▸"}</span>
      </div>
      {open && (
        <div className="part-sub-card">
          <PartCard identity={{ line, target, inherited: false, ancestor: null }} index={index} depth={depth + 1} />
        </div>
      )}
    </div>
  );
}

/** What to say when the selection has no line on the bill. */
function PartHint({ selection }: { selection: Selection }) {
  if (selection.type === "obstacle" || selection.type === "group") {
    const name = selection.type === "obstacle" ? selection.name : selection.path;
    return (
      <div className="hint">
        <code>{name}</code> is geometry, not a line of the bill —{" "}
        <code>scene.set_part("{name.replace(/^\//, "")}", manufacturer=…, model=…)</code> puts it on.
      </div>
    );
  }
  return (
    <div className="hint">
      select a robot, an object, a sensor or a device — the tree is the list
    </div>
  );
}

export function PartPanel() {
  const selection = useStudioStore((s) => s.selection);
  const index = useStudioStore((s) => s.bomIndex);
  const partReveal = useStudioStore((s) => s.partReveal);
  const identity = useMemo(() => identityFor(selection, index), [selection, index]);
  const hub = useCatalog(identity?.line.catalog ?? null);
  const info = hub?.status === "ok" ? hub.info : null;
  return (
    <Section
      id="part"
      title="Part"
      reveal={partReveal}
      badge={
        identity ? (
          <StateChips line={identity.line} target={identity.target} info={info} compact />
        ) : undefined
      }
    >
      <div className="part-body">
        {identity ? (
          <PartCard identity={identity} index={index} />
        ) : (
          <PartHint selection={selection} />
        )}
      </div>
    </Section>
  );
}
