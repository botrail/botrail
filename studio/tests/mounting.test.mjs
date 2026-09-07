import { test } from "node:test";
import assert from "node:assert/strict";
import {
  parseInspection,
  pairedHole,
  engagementFor,
  projectHole,
  range,
  sourceHref,
} from "../src/mounting.ts";

const identity = { position: [0, 0, 0], quaternion: [0, 0, 0, 1] };
function fixture() {
  const face = (role, ids) => ({
    frame: role,
    geometry_mapped: role === "flange",
    pose: identity,
    declaration: {
      geometry: {
        holes: ids.map((id, i) => ({
          id,
          kind: role === "flange" ? "threaded" : "clearance",
          position_mm: [i * 20, 0],
        })),
      },
    },
  });
  return parseInspection({
    schema_version: "1",
    report: {
      ready: false,
      input_hash: "hash",
      items: [
        {
          id: "joint:dimensions",
          target: "joint",
          status: "fail",
          evidence: {
            checks: [
              {
                check: "hole_pattern",
                status: "pass",
                inputs: { pairs: [1, 0] },
              },
              {
                check: "bolt-B:engagement",
                status: "fail",
                inputs: { engagement_mm: { min: 3, max: 3 } },
              },
            ],
          },
        },
      ],
    },
    connections: [
      {
        target: "joint",
        robot: "arm",
        offset: identity,
        flange: face("flange", ["receiver-A", "receiver-B"]),
        mount: face("mount", ["bolt-A", "bolt-B"]),
      },
    ],
  });
}

test("uses Rust's indexed hole pairing in both directions, despite different IDs and order", () => {
  const v = fixture(),
    c = v.connections[0];
  assert.equal(pairedHole(v, c, "flange:receiver-A").hole.id, "bolt-B");
  assert.equal(pairedHole(v, c, "mount:bolt-B").hole.id, "receiver-A");
  assert.equal(engagementFor(v, c, "flange:receiver-A").status, "fail");
  assert.equal(engagementFor(v, c, "mount:bolt-B").status, "fail");
  assert.equal(v.ready, false);
  c.mount.holes.pop();
  assert.equal(pairedHole(v, c, "flange:receiver-A"), null);
});

test("unconfirmed mapping retains drawing dimensions without promoting its status", () => {
  const v = fixture(),
    c = v.connections[0];
  assert.equal(c.flange.mapped, true);
  assert.equal(c.mount.mapped, false);
  assert.equal(c.mount.holes.length, 2);
  assert.equal(c.mount.clearanceMapped, false);
  assert.equal(v.findings[0].status, "fail");
  assert.equal(v.findings[0].checks[0].status, "pass");
  assert.throws(
    () => parseInspection({ schema_version: "future" }),
    /Unsupported/,
  );
});

test("unresolved hole matching does not present an arbitrary candidate as the paired hole", () => {
  const v = fixture(), c = v.connections[0];
  const check = v.findings[0].checks[0];
  check.status = "unknown";
  check.inputs.pair_statuses = [["unknown", "unknown"], ["unknown", "unknown"]];
  const before = JSON.stringify(v);
  assert.equal(pairedHole(v, c, "flange:receiver-A"), null);
  assert.equal(pairedHole(v, c, "mount:bolt-B"), null);
  assert.equal(engagementFor(v, c, "flange:receiver-A"), undefined);
  assert.equal(engagementFor(v, c, "mount:bolt-B").status, "fail");
  assert.equal(JSON.stringify(v), before);
});

test("manufacturer support does not hide the reference model's limitations", () => {
  const report = {
    ready: false,
    input_hash: "reference-kit",
    items: [],
    kits: [{
      manufacturer_support: { status: "pass", note: "This kit supports this host." },
      model_correspondence: {
        status: "unknown",
        representation: { status: "reference", note: "Screw-bearing height differs from manufacturer CAD." },
      },
      detailed_fit: { status: "unknown" },
    }],
  };
  const v = parseInspection({ schema_version: "1", report, connections: [] });
  assert.equal(v.kits[0].support, "pass");
  assert.equal(v.kits[0].correspondence, "unknown");
  assert.equal(v.kits[0].note, "This kit supports this host.");
  assert.equal(v.kits[0].representationNote, "Screw-bearing height differs from manufacturer CAD.");
  assert.equal(v.ready, false);
  delete report.kits[0].model_correspondence.representation;
  assert.equal(parseInspection({ schema_version: "1", report, connections: [] }).kits[0].representationNote, "");
});

test("projection applies mounting rotation and converts only the pose from metres to millimetres", () => {
  const c = fixture().connections[0];
  c.offset = {
    position: [0.001, 0.002, 0.003],
    quaternion: [0, 0, Math.SQRT1_2, Math.SQRT1_2],
  };
  const projected = projectHole(c, "mount", { position: [20, 0] });
  [1, 22, 3].forEach((value, i) =>
    assert.ok(Math.abs(projected[i] - value) < 1e-10),
  );
  assert.deepEqual(projectHole(c, "flange", { position: [20, 0] }), [20, 0, 0]);
  c.offset.quaternion = [1, 0, 0, 0];
  assert.deepEqual(
    projectHole(c, "mount", { position: [20, 10] }),
    [21, -8, 3],
  );
});

test("missing and invalid intervals stay unknown; evidence links allow only web URLs", () => {
  for (const v of [null, {}, { min: 3, max: 2 }, { min: NaN, max: 5 }])
    assert.equal(range(v), null);
  assert.deepEqual(range({ min: 0, max: 2 }), { min: 0, max: 2 });
  assert.equal(
    sourceHref("https://example.org/drawing.pdf"),
    "https://example.org/drawing.pdf",
  );
  for (const url of [
    "javascript:alert(1)",
    "drawing:synthetic",
    "file:///tmp/drawing.pdf",
  ])
    assert.equal(sourceHref(url), undefined);
});
