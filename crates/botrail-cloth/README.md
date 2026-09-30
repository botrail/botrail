# botrail-cloth

Cloth for botrail cells, simulated by [rapier-cloth](https://github.com/neka-nat/rapier-cloth)
against a baked cycle.

- `ClothCell` owns one cloth — a parametric T-shirt or a plain sheet — on a
  level support, with one kinematic body per gripper (optionally carrying a
  box pad that pushes cloth). It takes gripper poses and grasp, soften,
  release and hand-over events, and steps the cloth transactionally (0.1 s on
  the implicit shell solver): a rejected step rolls everything back.
- `pass::animate` drives a cell from a baked cycle: each gripper's tool point
  follows its robot link (forward kinematics off the robot track, on a moving
  base if it rides one) and holds while a signal is on. The result is a
  `ClothTrack` (`botrail_scene::cloth`), which a timeline carries in
  `SequenceTimeline::cloths`, the wire in `TimelineMsg::cloths`, the studio
  draws and the USD export writes as a time-sampled mesh.
- `pass::drive` is the same loop over plain closures, for hosts without a
  robot model.

Everything the API takes or returns is in botrail's Z-up world frame (the
simulator is Y-up; the cell converts at the boundary). A flat cloth's width
runs along `+x` and its length — a T-shirt's hem to its shoulders — along
`-y`, turned by `ClothSpec::yaw`.

```rust
use botrail_cloth::{ClothCell, ClothShape, ClothSpec};
use nalgebra::Isometry3;

// A 30 x 20 cm sheet on a support at z = 0.7 m and one bare gripper.
let spec = ClothSpec {
    shape: ClothShape::Sheet { size: [0.3, 0.2], spacing: 0.02 },
    ..ClothSpec::default()
};
let mut cell = ClothCell::new(&spec, Some(0.7), &[None])?;
let corner = cell.landmark_position("corner_se").unwrap();
cell.place_gripper(0, &Isometry3::translation(corner[0], corner[1], corner[2]))?;
cell.grasp_near(0, 0.04, 0.02)?;          // the vertices within 4 cm, held on springs
for k in 1..=10 {
    cell.set_gripper_pose(0, &Isometry3::translation(corner[0], corner[1], corner[2] + 0.01 * k as f64))?;
    cell.step(0.1)?;
}
cell.soften_grasp(0, 0.2)?;               // a soft release: weaker springs for a step
cell.step(0.1)?;
cell.release(0)?;
```

Python reaches this through `bt.cloth` (`docs/guides/cloth.md`); the demo is
`examples/cloth/tshirt_fold_demo.py`.

`rapier-cloth` is a git dependency pinned to a commit (root `Cargo.toml`)
until its 0.3 is on crates.io: the garment, stitch and soft-release APIs are
newer than 0.2.0. To work on both side by side, point the dependency at a
sibling checkout in an untracked `.cargo/config.toml`:

```toml
[patch."https://github.com/neka-nat/rapier-cloth"]
rapier-cloth = { path = "../rapier-cloth" }
```

Dev builds keep the cloth solver optimized (`[profile.dev.package.*]` in the
root manifest): unoptimized it is about thirty times slower, which neither
the tests nor a `maturin develop` build could afford.
