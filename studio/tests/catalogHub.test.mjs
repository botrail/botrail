import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { test } from "node:test";
import {
  HUB_BASE,
  cacheKey,
  fidelityNote,
  hubUrl,
  parseManifest,
  revisionKind,
  sourceLabel,
  withHubSpecs,
} from "../src/catalogHub.ts";

const SHA = "1d6bfc13fc805f2290db0da96109ba20919cce65";
const fixture = (name) => JSON.parse(readFileSync(new URL(`./fixtures/catalog/${name}.json`, import.meta.url), "utf8"));
const ref = (id, revision = SHA) => ({ id, revision });

test("a reference's revision: a dataset commit, a local build, or nothing pinned", () => {
  assert.equal(revisionKind(SHA), "pinned");
  assert.equal(revisionKind("local-sha256:abc"), "local");
  assert.equal(revisionKind(null), "unpinned");
  assert.equal(revisionKind("r2"), "unpinned");
  assert.equal(hubUrl("universal_robots/ur/ur5e/r2", SHA, "manifest.json"), `${HUB_BASE}/${SHA}/universal_robots/ur/ur5e/r2/manifest.json`);
  assert.equal(hubUrl("a/b/c/r1", null, "previews/thumbnail.webp"), `${HUB_BASE}/main/a/b/c/r1/previews/thumbnail.webp`);
  assert.equal(hubUrl("a/b/c/r1", "local-sha256:abc", "manifest.json"), null);
  assert.equal(cacheKey(ref("a/b/c/r1")), `a/b/c/r1@${SHA}`);
  assert.equal(cacheKey({ id: "a/b/c/r1", revision: null }), "a/b/c/r1@main");
});

test("an arm's manifest: maker page, datasheet and repository, licence, level, picture", () => {
  const info = parseManifest(fixture("ur5e"), ref("universal_robots/ur/ur5e/r2"));
  assert.equal(info.name, "UR5e");
  assert.deepEqual(info.maker, { name: "Universal Robots", country: "DK", url: "https://www.universal-robots.com/products/ur5e/" });
  assert.equal(info.level, "V2");
  assert.equal(info.lifecycle, null); // active is the default, not a chip
  assert.equal(info.distribution, null); // public likewise
  assert.equal(info.picture?.kind, "thumbnail");
  assert.equal(info.picture?.url, `${HUB_BASE}/${SHA}/universal_robots/ur/ur5e/r2/previews/thumbnail.webp`);
  assert.deepEqual(info.sources.map((s) => s.label), ["datasheet", "ros-industrial/universal_robot"]);
  assert.equal(info.sources[1].ref?.slice(0, 7), "39ad110");
  assert.equal(info.licenses[0].spdx, "BSD-3-Clause");
  assert.equal(info.licenses[0].redistributable, true);
  assert.ok(info.licenses[0].note?.includes("ur_description/LICENSE"));
  assert.equal(info.fidelity, null);
  // Text specs the scene does not carry come along; arrays do not.
  assert.equal(info.specs.ip_rating, "IP54");
  assert.equal(info.specs.max_tcp_speed_mps, 1);
  assert.equal("joint_limits_deg" in info.specs, false);
});

test("a reference model says what its geometry is not; a recipe-only one says what its files may not do", () => {
  const mir = parseManifest(fixture("mir250"), ref("mobile_industrial_robots/mir/mir250/r2"));
  assert.ok(mir.fidelity?.startsWith("geometry: independently authored reference · collision: authored primitives not a guaranteed envelope · detailed fit verified: no"));
  assert.equal(mir.fidelity?.includes("asset card"), false);
  assert.ok(mir.licenses[0].note?.startsWith("Independently authored reference geometry."));
  const khi = parseManifest(fixture("rs007n"), ref("kawasaki/rs/rs007n/r1"));
  assert.equal(khi.distribution, "recipe only");
  assert.equal(khi.maker?.name, "川崎重工業");
  assert.equal(khi.licenses[0].spdx, "LicenseRef-KHI-CAD-Disclaimer");
  assert.equal(khi.licenses[0].redistributable, false);
  assert.ok(khi.licenses[0].note?.includes("レイアウト検討目的以外"));
  assert.equal(khi.licenses[1].redistributable, true);
});

test("a spec pack has a plan for a picture or none at all; a kit's sources are numbered", () => {
  const belt = parseManifest(fixture("belt-unit"), ref("botrail/conveyor/belt-unit/r1"));
  assert.equal(belt.kind, "spec");
  assert.equal(belt.picture, null);
  assert.equal(belt.level, "V1");
  assert.deepEqual(belt.sources.map((s) => s.label), ["botrail/botrail-assets"]);
  const glr = parseManifest(fixture("gl-r"), ref("keyence/gl-r/series/r1"));
  assert.equal(glr.picture?.kind, "layout");
  assert.ok(glr.picture?.url.endsWith("/keyence/gl-r/series/r1/previews/layout.svg"));
  assert.equal(glr.specs.ip_rating, "IP65/IP67");
  const kit = parseManifest(fixture("2f-85-kit"), ref("robotiq/2f/2f-85-ur-es-062-kit/r2"));
  assert.equal(kit.kind, "kit");
  assert.equal(kit.picture, null);
  assert.deepEqual(kit.sources.map((s) => s.label).slice(0, 4), ["datasheet", "datasheet 2", "CAD", "CAD 2"]);
});

test("nothing in a manifest is required", () => {
  const empty = parseManifest({}, ref("x/y/z/r1"));
  assert.equal(empty.id, "x/y/z/r1");
  assert.equal(empty.name, null);
  assert.equal(empty.maker, null);
  assert.deepEqual(empty.sources, []);
  assert.deepEqual(empty.licenses, []);
  assert.equal(empty.picture, null);
  assert.deepEqual(empty.specs, {});
  assert.equal(parseManifest(null, ref("x/y/z/r1")).name, null);
  assert.equal(parseManifest({ manufacturer: "not an object", sources: "nope", licenses: [null, 3] }, ref("x/y/z/r1")).maker, null);
  assert.equal(fidelityNote({ asset_card: "README.md" }), null);
  assert.equal(sourceLabel("manufacturer_cad", "https://blog.robotiq.com/x.step"), "CAD");
  assert.equal(sourceLabel("community", "https://github.com/botrail/botrail-assets.git"), "botrail/botrail-assets");
  assert.equal(sourceLabel("official_oss", "https://gitlab.com/org/proj/-/tree/main"), "org/proj");
});

test("the hub's specs lie under the scene's own attributes", () => {
  const info = parseManifest(fixture("ur5e"), ref("universal_robots/ur/ur5e/r2"));
  const line = {
    category: "manipulator", manufacturer: "Universal Robots", model: "UR5e", catalog: ref("universal_robots/ur/ur5e/r2"),
    qty: 1, description: null, attributes: { payload_kg: 5, reach_mm: 850, mass_kg: 99 }, order: null, targets: [], identified: true,
  };
  const merged = withHubSpecs(line, info);
  assert.equal(merged.attributes.mass_kg, 99);
  assert.equal(merged.attributes.ip_rating, "IP54");
  assert.equal(merged.attributes.repeatability_mm, 0.03);
  assert.equal(withHubSpecs(line, null), line);
});
