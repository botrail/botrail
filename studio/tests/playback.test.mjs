import assert from "node:assert/strict";
import test from "node:test";

import { samplePlayback, tracksFromTimeline } from "../src/playback.ts";

const pose = (x) => ({ position: [x, 0, 0], quaternion: [0, 0, 0, 1] });

test("USD transform recordings move the arm and vehicle visuals without joint samples", () => {
  const tracks = tracksFromTimeline({
    duration: 2,
    robots: [{
      name: "ur",
      trajectory: {
        duration: 2,
        times: [0, 2],
        joint_positions: [],
        link_poses: [[pose(0)], [pose(2)]],
      },
    }],
    objects: [{ name: "amr/visual/top_cover", poses: [pose(0.5), pose(2.5)] }],
  });

  for (const t of [0, 1, 2]) {
    const sample = samplePlayback(tracks, t);
    assert.equal(sample.joints, null);
    assert.deepEqual(sample.poses.ur, [pose(t)]);
    assert.deepEqual(sample.objects["amr/visual/top_cover"], pose(t + 0.5));
  }
});

test("joint-state recordings still interpolate joints and the moving base", () => {
  const tracks = tracksFromTimeline({
    duration: 2,
    robots: [{
      name: "ur",
      base: [pose(0), pose(2)],
      trajectory: {
        duration: 2,
        times: [0, 2],
        joint_positions: [[0, 1], [1, -1]],
      },
    }],
    objects: [],
  });
  const sample = samplePlayback(tracks, 1);
  assert.deepEqual(sample.joints, { ur: [0.5, 0] });
  assert.deepEqual(sample.bases, { ur: pose(1) });
  assert.equal(sample.poses, null);
});

import { appendTracks } from "../src/playback.ts";

test("streamed physics windows append on the shared lattice, padding late tracks back to the start", () => {
  // Window 1: (0, 0.1] → lattice 1/30, 2/30, 3/30 (a robot-less cell).
  const first = appendTracks(
    null,
    {
      duration: 0.1,
      robots: [],
      objects: [{ name: "crate", poses: [pose(1), pose(2), pose(3)] }],
      step_spans: [],
      signals: [],
    },
    0,
  );
  assert.deepEqual(first.objects.times.map((t) => Math.round(t * 30)), [1, 2, 3]);
  assert.equal(first.duration, 0.1);
  // Window 2: (0.1, 0.2] → 4/30 .. 6/30; a second object wakes here.
  const second = appendTracks(
    first,
    {
      duration: 0.2,
      robots: [],
      objects: [
        { name: "crate", poses: [pose(4), pose(5), pose(6)] },
        { name: "lid", poses: [pose(9), pose(9), pose(8)], visible: [true, true, false] },
      ],
      step_spans: [],
      signals: [],
    },
    0.1,
  );
  assert.deepEqual(second.objects.times.map((t) => Math.round(t * 30)), [1, 2, 3, 4, 5, 6]);
  const crate = second.objects.tracks.find((o) => o.name === "crate");
  assert.deepEqual(crate.poses.map((p) => p.position[0]), [1, 2, 3, 4, 5, 6]);
  const lid = second.objects.tracks.find((o) => o.name === "lid");
  // The lid stood still before its first window: padded with its first pose.
  assert.deepEqual(lid.poses.map((p) => p.position[0]), [9, 9, 9, 9, 9, 8]);
  assert.deepEqual(lid.visible, [true, true, true, true, true, false]);
  // Sampling reads across the seam.
  const sample = samplePlayback(second, 3.5 / 30);
  assert.ok(Math.abs(sample.objects.crate.position[0] - 3.5) < 1e-9);
  assert.ok(sample.stowed.has("lid") === false);
  const late = samplePlayback(second, 6 / 30);
  assert.ok(late.stowed.has("lid"));
});
