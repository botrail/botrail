import type { BomLineMsg, CatalogRef, PartAttr } from "./protocol";

/**
 * The second layer of a part card: what the catalog says about a product
 * beyond the numbers the scene carries — its picture, the maker's page,
 * the datasheets and repositories it was built from, the licence each
 * file is under (and its note: the use restrictions live nowhere else),
 * how far it was validated, whether it is still sold. The browser reads
 * the product's `manifest.json` straight from the hub, at the dataset
 * commit the cell resolved the package at, so the card shows the same
 * revision the cell was taught on. Nothing here is required: a card
 * without the hub is the wire's part alone.
 */

export const HUB_BASE = "https://huggingface.co/datasets/botrail/botrail-catalog/resolve";
export const VIEWER_BASE = "https://botrail-catalog.static.hf.space/#/p/";

export type RevisionKind = "pinned" | "local" | "unpinned";

/** What a catalog reference's revision is: a dataset commit the package
 * was fetched at, a locally built package (`local-sha256:…`, nothing on
 * the hub to read), or a hand-written reference nobody pinned. */
export function revisionKind(revision: string | null | undefined): RevisionKind {
  if (!revision) return "unpinned";
  if (revision.startsWith("local-sha256:")) return "local";
  return /^[0-9a-f]{40}$/.test(revision) ? "pinned" : "unpinned";
}

/** The hub URL of a file of a package, at the reference's revision
 * (`main` when unpinned); `null` for a local package. */
export function hubUrl(id: string, revision: string | null | undefined, path: string): string | null {
  const kind = revisionKind(revision);
  if (kind === "local") return null;
  const rev = kind === "pinned" ? revision : "main";
  return `${HUB_BASE}/${rev}/${id}/${path.replace(/^\/+/, "")}`;
}

/** The catalog viewer's product page. */
export function viewerUrl(id: string): string {
  return `${VIEWER_BASE}${id}`;
}

/** One cache entry per reference: the id at the revision it was read. */
export function cacheKey(ref: CatalogRef): string {
  return `${ref.id}@${ref.revision ?? "main"}`;
}

export interface CatalogSource {
  kind: string;
  /** What the chip reads: the repository (`owner/repo`) or the kind. */
  label: string;
  url: string;
  ref: string | null;
  fetchedAt: string | null;
}

export interface CatalogLicense {
  files: string;
  spdx: string;
  redistributable: boolean;
  note: string | null;
}

/** What a card shows of a manifest. Every field is optional: a manifest
 * that says nothing about something leaves it null or empty. */
export interface CatalogInfo {
  id: string;
  name: string | null;
  maker: { name: string; country: string | null; url: string | null } | null;
  /** The package kind: `model`, `spec`, `kit`. */
  kind: string | null;
  /** Shown when not `active`. */
  lifecycle: string | null;
  /** Shown when not `public` (`recipe only`, `metadata only`). */
  distribution: string | null;
  /** `V0`…`V5`. */
  level: string | null;
  reviewed: string | null;
  /** The product's picture: a rendered thumbnail, or a spec pack's plan. */
  picture: { url: string; kind: "thumbnail" | "layout" } | null;
  sources: CatalogSource[];
  licenses: CatalogLicense[];
  /** The `model_fidelity` notes in one line, or null. */
  fidelity: string | null;
  /** Specs the manifest states as a number or a text (`ip_rating`), by key. */
  specs: Record<string, PartAttr>;
}

const SOURCE_LABELS: Record<string, string> = {
  manufacturer_datasheet: "datasheet",
  manufacturer_cad: "CAD",
  cad_vendor: "CAD",
  official_oss: "OSS",
  community: "community",
  standard: "standard",
  user_drawing: "drawing",
  measurement: "measurement",
};

/** `https://github.com/owner/repo/blob/...` → `owner/repo`; anything else
 * reads as its kind. */
export function sourceLabel(kind: string, url: string): string {
  const m = /^https?:\/\/(?:www\.)?(?:github|gitlab|bitbucket)\.(?:com|org)\/([^/]+)\/([^/#?]+)/.exec(url);
  if (m) return `${m[1]}/${m[2].replace(/\.git$/, "")}`;
  return SOURCE_LABELS[kind] ?? kind;
}

function str(v: unknown): string | null {
  return typeof v === "string" && v.trim() !== "" ? v : null;
}

function record(v: unknown): Record<string, unknown> | null {
  return v !== null && typeof v === "object" && !Array.isArray(v) ? (v as Record<string, unknown>) : null;
}

function words(v: string): string {
  return v.replace(/_/g, " ");
}

/** `model_fidelity` — what the geometry is and is not — as one line:
 * `geometry: independently authored reference · collision: authored
 * primitives not a guaranteed envelope · detailed fit verified: no`. */
export function fidelityNote(fidelity: unknown): string | null {
  const f = record(fidelity);
  if (!f) return null;
  const parts: string[] = [];
  for (const [key, value] of Object.entries(f)) {
    if (key === "asset_card") continue;
    if (typeof value === "string") parts.push(`${words(key)}: ${words(value)}`);
    else if (typeof value === "boolean") parts.push(`${words(key)}: ${value ? "yes" : "no"}`);
    else if (typeof value === "number") parts.push(`${words(key)}: ${value}`);
  }
  return parts.length > 0 ? parts.join(" · ") : null;
}

/** The manifest as the card reads it. Tolerates anything: a manifest
 * missing a field, a field of the wrong shape, or no manifest at all. */
export function parseManifest(json: unknown, ref: CatalogRef): CatalogInfo {
  const m = record(json) ?? {};
  const maker = record(m.manufacturer);
  const validation = record(m.validation);
  const assets = record(m.assets);
  const compatibility = record(m.compatibility);
  const lifecycle = str(m.lifecycle);
  const distribution = str(m.distribution);
  const reviewedBy = validation ? str(validation.reviewed_by) : null;
  const reviewedAt = validation ? str(validation.reviewed_at) : null;

  const seen = new Map<string, number>();
  const sources: CatalogSource[] = [];
  for (const s of Array.isArray(m.sources) ? m.sources : []) {
    const r = record(s);
    const url = r ? str(r.url) : null;
    if (!r || !url) continue;
    const kind = str(r.kind) ?? "source";
    let label = sourceLabel(kind, url);
    const n = (seen.get(label) ?? 0) + 1;
    seen.set(label, n);
    if (n > 1) label = `${label} ${n}`;
    sources.push({ kind, label, url, ref: str(r.ref), fetchedAt: str(r.fetched_at) });
  }

  const licenses: CatalogLicense[] = [];
  for (const l of Array.isArray(m.licenses) ? m.licenses : []) {
    const r = record(l);
    if (!r) continue;
    licenses.push({
      files: str(r.files) ?? "**",
      spdx: str(r.spdx) ?? "?",
      redistributable: r.redistributable === true,
      note: str(r.note),
    });
  }

  let picture: CatalogInfo["picture"] = null;
  const thumbnail = assets ? str(assets.thumbnail) : null;
  const layout = assets ? str(assets.layout) : null;
  const pictureUrl = (path: string) => hubUrl(ref.id, ref.revision, path);
  if (thumbnail && pictureUrl(thumbnail)) picture = { url: pictureUrl(thumbnail)!, kind: "thumbnail" };
  else if (layout && pictureUrl(layout)) picture = { url: pictureUrl(layout)!, kind: "layout" };

  const specs: Record<string, PartAttr> = {};
  for (const [key, value] of Object.entries(record(m.specs) ?? {})) {
    if (typeof value === "number" || (typeof value === "string" && value.trim() !== "")) specs[key] = value;
  }

  return {
    id: str(m.id) ?? ref.id,
    name: str(m.name),
    maker: maker && str(maker.name) ? { name: str(maker.name)!, country: str(maker.country), url: str(maker.product_url) } : null,
    kind: str(m.kind),
    lifecycle: lifecycle && lifecycle !== "active" ? lifecycle : null,
    distribution: distribution && distribution !== "public" ? words(distribution) : null,
    level: validation ? str(validation.level) : null,
    reviewed: reviewedBy || reviewedAt ? [reviewedBy, reviewedAt].filter(Boolean).join(" · ") : null,
    picture,
    sources,
    licenses,
    fidelity: fidelityNote(compatibility?.model_fidelity),
    specs,
  };
}

/** The line with the manifest's specs the scene does not carry laid in —
 * the text ones (`ip_rating`) and numbers the package's numeric specs did
 * not include. The scene's own attributes always win. */
export function withHubSpecs(line: BomLineMsg, info: CatalogInfo | null): BomLineMsg {
  if (!info) return line;
  const attributes = { ...line.attributes };
  let added = false;
  for (const [key, value] of Object.entries(info.specs)) {
    if (!(key in attributes)) {
      attributes[key] = value;
      added = true;
    }
  }
  return added ? { ...line, attributes } : line;
}
