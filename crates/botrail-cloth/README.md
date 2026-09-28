# botrail-cloth

Garment cloth for botrail cells, simulated by [rapier-cloth](../../../rapier-cloth)
next to a bake. A `ClothCell` owns one garment (a parametric T-shirt), a table
and one kinematic body per gripper; the host hands over gripper poses and
grasp / release / transfer events per sample and steps the cloth (0.1 s on the
implicit solver). The result is a `ClothTrack`: per-sample vertex positions on
the bake's sample grid, plus the triangles, the held vertices and the garment's
landmarks, ready for a `TimelineMsg` channel and the studio.

```rust
use botrail_cloth::{ClothCell, GarmentSpec, GraspSpec, PatchLayers};
let mut cell = ClothCell::new(&GarmentSpec::default(), Some(0.0), 2)?;
let cuff = cell.landmark_position("cuff_left").unwrap();
cell.place_gripper(0, &nalgebra::Isometry3::translation(cuff[0], cuff[1], cuff[2]))?;
cell.grasp(0, &GraspSpec { landmark: "cuff_left".into(), radius: 0.04, layers: PatchLayers::All, compliance: 0.02 })?;
for k in 1..=10 {
    cell.set_gripper_pose(0, &nalgebra::Isometry3::translation(cuff[0], cuff[1] + 0.01 * k as f64, cuff[2]))?;
    cell.step(0.1)?;
}
let track = cell.track("shirt");
```

Status: the cell and the track exist; wiring the track into `TimelineMsg`
(`objects` has a sibling `cloths` waiting), driving grippers from the rollout's
robot tracks, and drawing the points in the studio are the next steps
(`.internal/docs/cloth-adapter.md`).
