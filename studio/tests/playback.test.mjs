import assert from "node:assert/strict";
import test from "node:test";

import { clothHeldAt, samplePlayback, tracksFromTimeline } from "../src/playback.ts";

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

import { appendTracks, forgetLive } from "../src/playback.ts";

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

test("a stream's own tracks grow in place, window by window, and read like one long window", () => {
  const robot = (times, q) => ({
    name: "arm",
    trajectory: {
      duration: times[times.length - 1],
      times,
      joint_positions: q.map((v) => [v]),
      link_poses: q.map((v) => [pose(v)]),
    },
    moves: [],
  });
  const window = (duration, times, q, objects) => ({
    duration,
    robots: [robot(times, q)],
    objects,
    step_spans: [],
    signals: [],
  });
  const first = appendTracks(null, window(0.1, [1 / 30, 2 / 30, 3 / 30], [1, 2, 3], [
    { name: "box", poses: [pose(1), pose(2), pose(3)] },
  ]), 0);
  const second = appendTracks(first, window(0.2, [4 / 30, 5 / 30, 6 / 30], [4, 5, 6], [
    { name: "box", poses: [pose(4), pose(5), pose(6)] },
    { name: "cap", poses: [pose(7), pose(7), pose(8)], visible: [true, true, false] },
  ]), 0.1);
  const third = appendTracks(second, window(0.3, [7 / 30, 8 / 30, 9 / 30], [7, 8, 9], [
    { name: "box", poses: [pose(7), pose(8), pose(9)] },
    { name: "cap", poses: [pose(8), pose(8), pose(8)], visible: [false, true, true] },
  ]), 0.2);
  // A new wrapper each window (the store sees a change) over the same
  // arrays, grown in place.
  assert.notEqual(third, second);
  assert.equal(third.objects.times, first.objects.times);
  assert.equal(third.robots[0].trajectory.times, third.objects.times);
  assert.equal(third.robots[0].trajectory.joint_positions, first.robots[0].trajectory.joint_positions);
  const box = (t) => t.objects.tracks.find((o) => o.name === "box");
  assert.equal(box(third).poses, box(first).poses);
  // What they hold is the concatenation, the late track padded back.
  assert.deepEqual(third.objects.times.map((t) => Math.round(t * 30)), [1, 2, 3, 4, 5, 6, 7, 8, 9]);
  assert.deepEqual(third.robots[0].trajectory.joint_positions.flat(), [1, 2, 3, 4, 5, 6, 7, 8, 9]);
  assert.deepEqual(third.robots[0].trajectory.link_poses.map((p) => p[0].position[0]), [1, 2, 3, 4, 5, 6, 7, 8, 9]);
  assert.deepEqual(box(third).poses.map((p) => p.position[0]), [1, 2, 3, 4, 5, 6, 7, 8, 9]);
  const cap = third.objects.tracks.find((o) => o.name === "cap");
  assert.deepEqual(cap.poses.map((p) => p.position[0]), [7, 7, 7, 7, 7, 8, 8, 8, 8]);
  assert.deepEqual(cap.visible, [true, true, true, true, true, false, false, true, true]);
  // Sampling reads across both seams.
  const sample = samplePlayback(third, 6.5 / 30);
  assert.ok(Math.abs(sample.objects.box.position[0] - 6.5) < 1e-9);
  assert.ok(Math.abs(sample.joints.arm[0] - 6.5) < 1e-9);
  // Tracks the stream did not build are copied, never grown in place.
  const baked = tracksFromTimeline({
    duration: 0.1,
    robots: [robot([0, 0.1], [0, 1])],
    objects: [],
  });
  const before = baked.robots[0].trajectory.times.length;
  appendTracks(baked, window(0.2, [4 / 30, 5 / 30, 6 / 30], [4, 5, 6], []), 0.1);
  assert.equal(baked.robots[0].trajectory.times.length, before);
});

test("a live stream keeps its programs' part and its last moments, and at its end the final frame", () => {
  const frame = (k) => ({
    duration: k / 30,
    robots: [{
      name: "arm",
      trajectory: { duration: k / 30, times: [k / 30], joint_positions: [[k]], link_poses: [[pose(k)]] },
      moves: [],
    }],
    objects: [{ name: "box", poses: [pose(k)] }],
    step_spans: [],
    signals: [],
  });
  // Programs to 0.1 s (samples 1..3), then the live part (4..12).
  let tracks = appendTracks(null, frame(1), 0);
  for (let k = 2; k <= 12; k++) tracks = appendTracks(tracks, frame(k), (k - 1) / 30);
  const head = samplePlayback(tracks, 12 / 30);
  // Keep from 10/30: the live samples 4..9 go, the programs' 1..3 stay.
  assert.ok(forgetLive(tracks, 3 / 30, 10 / 30));
  const at = (t) => t.map((x) => Math.round(x * 30));
  assert.deepEqual(at(tracks.objects.times), [1, 2, 3, 10, 11, 12]);
  assert.equal(tracks.robots[0].trajectory.times, tracks.objects.times);
  assert.deepEqual(tracks.robots[0].trajectory.joint_positions.flat(), [1, 2, 3, 10, 11, 12]);
  assert.deepEqual(tracks.robots[0].trajectory.link_poses.map((p) => p[0].position[0]), [1, 2, 3, 10, 11, 12]);
  assert.deepEqual(tracks.objects.tracks[0].poses.map((p) => p.position[0]), [1, 2, 3, 10, 11, 12]);
  // What the viewport draws now is unchanged, and so is the programs' part.
  assert.deepEqual(samplePlayback(tracks, 12 / 30), head);
  assert.ok(Math.abs(samplePlayback(tracks, 2.5 / 30).objects.box.position[0] - 2.5) < 1e-9);
  // Nothing more to drop until the head moves on.
  assert.equal(forgetLive(tracks, 3 / 30, 10 / 30), false);
  // The stream ends between lattice points: the live part keeps its
  // final frame all the same.
  forgetLive(tracks, 3 / 30, 12.5 / 30);
  assert.deepEqual(at(tracks.objects.times), [1, 2, 3, 12]);
  assert.deepEqual(tracks.robots[0].trajectory.joint_positions.flat(), [1, 2, 3, 12]);
  assert.equal(forgetLive(tracks, 3 / 30, 99), false);
  assert.deepEqual(tracks.objects.tracks[0].poses.map((p) => p.position[0]), [1, 2, 3, 12]);
});

test("cloth tracks play on their own sample times", () => {
  const at = (z) => [[0, 0, z], [1, 0, z], [0, 1, z]];
  const tracks = tracksFromTimeline({
    duration: 2,
    robots: [],
    objects: [],
    cloths: [{
      name: "shirt",
      triangles: [[0, 1, 2]],
      times: [0, 0.5, 2],
      points: [at(0), at(1), at(4)],
      held: [[], [0, 2], []],
      landmarks: { hem_left: 0 },
    }],
  });
  const shirt = tracks.cloths[0];
  assert.deepEqual(shirt.times, [0, 0.5, 2]);
  // Blended between the cloth's own samples, not the robots' grid.
  assert.deepEqual(Array.from(samplePlayback(tracks, 0.25).cloths.shirt), [0, 0, 0.5, 1, 0, 0.5, 0, 1, 0.5]);
  assert.deepEqual(Array.from(samplePlayback(tracks, 1.0).cloths.shirt), [0, 0, 2, 1, 0, 2, 0, 1, 2]);
  assert.deepEqual(Array.from(samplePlayback(tracks, 5).cloths.shirt), [0, 0, 4, 1, 0, 4, 0, 1, 4]);
  // The sample carries what the driver hands the cloth's view each frame.
  assert.deepEqual(samplePlayback(tracks, 0.25).clothHeld, { shirt: [] });
  assert.deepEqual(samplePlayback(tracks, 1.0).clothHeld, { shirt: [0, 2] });
  // Held vertices step with the sample at or before the time.
  assert.deepEqual(clothHeldAt(shirt, 0.4), []);
  assert.deepEqual(clothHeldAt(shirt, 0.5), [0, 2]);
  assert.deepEqual(clothHeldAt(shirt, 1.9), [0, 2]);
  assert.deepEqual(clothHeldAt(shirt, 2), []);
  // A track without times is spread evenly over the cycle.
  const bare = tracksFromTimeline({
    duration: 2,
    robots: [],
    objects: [],
    cloths: [{ name: "s", triangles: [[0, 1, 2]], points: [at(0), at(2)] }],
  });
  assert.deepEqual(bare.cloths[0].times, [0, 2]);
  assert.deepEqual(clothHeldAt(bare.cloths[0], 1), []);
  // A timeline without cloth has no cloth samples.
  const bareSample = samplePlayback(tracksFromTimeline({ duration: 1, robots: [], objects: [] }), 0);
  assert.equal(bareSample.cloths, null);
  assert.equal(bareSample.clothHeld, null);
});

test("a streamed window appends to a cloth track and a live stream forgets its past", () => {
  const at = (z) => [[0, 0, z], [1, 0, z], [0, 1, z]];
  const window = (times, from) => ({
    duration: times[times.length - 1],
    robots: [],
    objects: [{ name: "box", poses: times.map(() => ({ position: [0, 0, 0], quaternion: [0, 0, 0, 1] })) }],
    cloths: [{
      name: "sheet",
      triangles: [[0, 1, 2]],
      times: times.filter((t) => t > from),
      points: times.filter((t) => t > from).map(at),
      held: times.filter((t) => t > from).map((t) => (t < 0.2 ? [1] : [])),
    }],
  });
  const lattice = (a, b) => Array.from({ length: b - a }, (_, k) => (a + k + 1) / 30);
  let tracks = appendTracks(null, window(lattice(0, 6), 0), 0);
  tracks = appendTracks(tracks, window(lattice(6, 12), 6 / 30), 6 / 30);
  const sheet = tracks.cloths[0];
  assert.equal(sheet.times.length, 12);
  assert.equal(sheet.points.length, 12);
  assert.equal(sheet.held.length, 12);
  assert.ok(Math.abs(samplePlayback(tracks, 0.3).cloths.sheet[2] - 0.3) < 1e-6);
  assert.deepEqual(clothHeldAt(sheet, 0.1), [1]);
  // Forgetting the first stretch of the live part drops those cloth samples too.
  assert.ok(forgetLive(tracks, 0, 0.2));
  assert.ok(tracks.cloths[0].times[0] >= 0.2 - 1e-9);
  assert.equal(tracks.cloths[0].points.length, tracks.cloths[0].times.length);
  assert.equal(tracks.cloths[0].held.length, tracks.cloths[0].times.length);
});
