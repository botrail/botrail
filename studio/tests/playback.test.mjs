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
