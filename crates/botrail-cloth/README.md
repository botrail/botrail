# botrail-cloth

Garment cloth for botrail cells, simulated by [rapier-cloth](../../../rapier-cloth)
next to a bake. A `ClothCell` owns one garment (a parametric T-shirt), a table
and one kinematic body per gripper; the host hands over gripper poses and
grasp / release / transfer events per sample and steps the cloth (0.1 s on the
implicit solver). The result is a `ClothTrack`: per-sample vertex positions on
the bake's sample grid, plus the triangles, the held vertices and the garment's
landmarks, carried by `TimelineMsg::cloths` and drawn by the studio.

Everything the API takes or returns is in botrail's Z-up world frame (the
simulator is Y-up; the cell converts at the boundary). The flat garment's
width runs along `+x` and its length, hem to shoulders, along `-y`, turned by
`GarmentSpec::yaw`.

```rust
use botrail_cloth::{ClothCell, GarmentSpec, GraspSpec, PatchLayers};
// A table top at z = 0.7 m and two grippers.
let mut cell = ClothCell::new(&GarmentSpec::default(), Some(0.7), 2)?;
let cuff = cell.landmark_position("cuff_left").unwrap();
cell.place_gripper(0, &nalgebra::Isometry3::translation(cuff[0], cuff[1], cuff[2]))?;
cell.grasp(0, &GraspSpec { landmark: "cuff_left".into(), radius: 0.04, layers: PatchLayers::All, compliance: 0.02 })?;
for k in 1..=10 {
    cell.set_gripper_pose(0, &nalgebra::Isometry3::translation(cuff[0], cuff[1], cuff[2] + 0.01 * k as f64))?;
    cell.step(0.1)?;
}
cell.soften_grasp(0, 0.2)?; // soft release: weaker springs for a step, then open
cell.step(0.1)?;
cell.release(0)?;
let track = cell.track("shirt");
```

The crate is excluded from the workspace while `rapier-cloth` is a path
dependency on a sibling checkout (CI has none): build and test it with
`cargo test --manifest-path crates/botrail-cloth/Cargo.toml`.

Status: the cell, the track, the `TimelineMsg::cloths` channel
(`ClothTrackMsg`) and the studio's `ClothView` exist; driving the grippers from
the rollout's robot tracks is the next step (`.internal/docs/cloth-adapter.md`).
