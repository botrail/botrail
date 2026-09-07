import {
  list,
  record,
  parseInspection,
  type Inspection,
  type Connection,
  type Face,
} from "./mounting.ts";
import type { RobotDescMsg, PoseMsg } from "./protocol";

export const ROUTE_LABELS: Record<string, string> = {
  direct_evidence: "Direct mounting evidence",
  adapter_evidence: "Adapter route evidence",
  needs_information: "Needs design / information",
};
export type AssemblySummary = {
  tcp: { link: string; pose: PoseMsg };
  mass: { known_kg: number | null; missing: string[] };
  links: number;
  bom: {
    category: string;
    names: string[];
    model?: string;
    catalog?: { id: string; revision?: string };
    qty: number;
  }[];
};
export type Proposal = {
  schema_version: "1";
  robot: string;
  base_revision: string;
  candidate_revision: string;
  route: string;
  can_apply: boolean;
  blockers: string[];
  mounting_blockers: string[];
  revalidation: string[];
  preserved_joints: string[];
  reset_joints: string[];
  project: unknown;
  before: AssemblySummary;
  after: AssemblySummary;
  inspection: Inspection;
  beforeInspection: Inspection;
  robotDesc: RobotDescMsg;
  beforeDesc: RobotDescMsg;
};
export function parseProposal(value: unknown): Proposal {
  const v = record(value);
  const robot = list(record(v.scene).robots).find(
    (r) => record(r).name === v.robot,
  );
  const before = list(record(v.before_scene).robots).find(
    (r) => record(r).name === v.robot,
  );
  if (
    v.schema_version !== "1" ||
    !robot ||
    !before ||
    typeof v.base_revision !== "string" ||
    typeof v.candidate_revision !== "string"
  )
    throw new Error("Unsupported assembly preview");
  return {
    ...v,
    can_apply: v.can_apply === true,
    mounting_blockers: list(v.mounting_blockers).filter((id): id is string => typeof id === "string"),
    inspection: parseInspection(v.inspection),
    beforeInspection: parseInspection(v.before_inspection),
    robotDesc: robot,
    beforeDesc: before,
  } as Proposal;
}

export function declaredMass(mass: AssemblySummary["mass"]): string {
  if (mass.known_kg === null) return "Unknown";
  return `${mass.known_kg.toFixed(3)} kg${mass.missing.length ? " + unknown" : ""}`;
}

export function bomDifference(before: AssemblySummary, after: AssemblySummary) {
  const rows = new Map<
    string,
    { name: string; before: number; after: number }
  >();
  for (const [side, summary] of [
    ["before", before],
    ["after", after],
  ] as const) {
    for (const row of summary.bom) {
      const key = row.catalog
        ? JSON.stringify(row.catalog)
        : JSON.stringify([row.model, row.category, row.names]);
      const entry = rows.get(key) ?? {
        name: row.model ?? row.catalog?.id ?? row.names.join(", "),
        before: 0,
        after: 0,
      };
      entry[side] += row.qty;
      rows.set(key, entry);
    }
  }
  return [...rows.values()];
}

/** A display-only whole-assembly frame. It supplies no mating evidence. */
export function overviewConnection(
  inspection: Inspection,
  robot: RobotDescMsg,
): Connection {
  const empty: Face = {
    part: null,
    frame: "",
    link: null,
    pose: null,
    mapped: false,
    clearanceMapped: false,
    normal: 1,
    holes: [],
    locators: [],
    solids: [],
    access: [],
    sources: [],
  };
  const connection = inspection.connections.find((c) => c.robot === robot.name);
  return {
    target: "assembly-overview",
    robot: robot.name,
    arm: null,
    flange: { ...empty, part: connection?.flange.part ?? robot.name },
    mount: empty,
    offset: robot.base_pose,
    movingLinks: robot.links.map((l) => l.name),
  };
}

/** Product IDs explicitly declared by the tool, including unverified alternatives.
 * Listing a declaration is not a compatibility judgement. */
export function declaredCandidates(
  inspection: Inspection | null,
  robot: string,
) {
  const suggestions = new Map<string, { catalog: string; target: string }>();
  for (const finding of inspection?.findings ?? []) {
    if (
      !finding.id.includes(":required:") ||
      finding.status === "pass" ||
      !finding.target.startsWith(`${robot}/`)
    )
      continue;
    const evidence = record(finding.raw.evidence);
    const ids = [
      record(evidence.order_requirement).catalog,
      ...list(record(evidence.requirement).catalog_alternatives).map(
        (a) => record(a).catalog,
      ),
    ];
    for (const catalog of ids)
      if (typeof catalog === "string" && catalog)
        suggestions.set(`${finding.target}:${catalog}`, {
          catalog,
          target: finding.target,
        });
  }
  return [...suggestions.values()];
}
