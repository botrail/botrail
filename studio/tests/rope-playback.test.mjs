import assert from "node:assert/strict";
import test from "node:test";
import { appendTracks, clothHeldAt, samplePlayback, tracksFromTimeline } from "../src/playback.ts";

const pose = (x) => ({ position:[x,0,0],quaternion:[0,0,0,1] });
const rope = { name:"cable",segments:[[0,1]],radius_m:0.005,
  times:[0,0.0031,0.0032,0.023],points:[[[0,0,0],[1,0,0]],[[0.1,0,0],[1,0,0]],[[0.2,0,0],[1,0,0]],[[1,0,0],[2,0,0]]],
  held:[[],[0],[],[0]],events:[],connectors:[{name:"plug",shape:"sphere",dimensions_m:[0.01],mass_kg:0.01,poses:[pose(0),pose(0.1),pose(0.2),pose(1)]}],
  coupling:"baked_one_way_no_source_reaction",coordinate_system:"right-handed Z-up SI",solver:"rapier3d-f64 0.36.0" };
const timeline = (r=rope) => ({ duration:0.023,robots:[{name:"robot",trajectory:{duration:0.023,times:[0,0.023],joint_positions:[[0],[1]],link_poses:[[pose(0)],[pose(1)]]}}],objects:[],vehicles:[],ropes:[r] });

test("rope, connector and robot use one playback clock; held state preserves short pulses and final events", () => {
  const tracks=tracksFromTimeline(timeline());
  assert.deepEqual(clothHeldAt(rope,0.00315),[0]);
  assert.deepEqual(clothHeldAt(rope,0.0032),[]);
  assert.deepEqual(clothHeldAt(rope,0.023),[0]);
  const sample=samplePlayback(tracks,0.00315);
  assert.ok(Math.abs(sample.ropes.cable[0]-0.15)<1e-6);
  assert.ok(Math.abs(sample.ropeBodies.cable[0].position[0]-0.15)<1e-12);
  assert.ok(Math.abs(sample.poses.robot[0].position[0]-0.00315/0.023)<1e-12);
  assert.deepEqual([...samplePlayback(tracks,0.023).ropes.cable],[1,0,0,2,0,0]);
});
test("overlapping window predecessor reconciles rope clocks without duplicates or losing connector alignment", () => {
  const first={...rope,times:rope.times.slice(0,2),points:rope.points.slice(0,2),held:rope.held.slice(0,2),connectors:rope.connectors.map(c=>({...c,poses:c.poses.slice(0,2)}))};
  const window={...rope,times:rope.times.slice(1),points:rope.points.slice(1),held:rope.held.slice(1),connectors:rope.connectors.map(c=>({...c,poses:c.poses.slice(1)}))};
  const appended=appendTracks(tracksFromTimeline(timeline(first)),timeline(window),0.0031);
  assert.deepEqual(appended.ropes[0].times,rope.times);
  assert.equal(appended.ropes[0].connectors[0].poses.length,rope.times.length);
  assert.deepEqual(clothHeldAt(appended.ropes[0],0.00315),[0]);
});
