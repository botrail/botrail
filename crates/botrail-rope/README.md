# botrail-rope

Explicit post-bake cable replay in an independent **Rapier 0.36.0 f64**
world. Botrail's rigid engine and cloth remain on Rapier 0.34 / Parry 0.29.
The adapter uses owned dimensions, vertices, triangle indices and poses;
old engine shapes and handles are never used by the new world.

```sh
cargo test -p botrail-rope --locked
cargo run -p botrail-rope --example rope_replay -- target/rope-replay.json
npm run build --prefix studio
python3 scripts/check_rope_viewer.py
```

Uses the published `rapier-rope` 0.1.0 crate from crates.io with f64 enabled.
This adapter is `publish = false`. Cold checkouts need no vendored runtime or sibling repository.

See [user guide](../../docs/guides/rope.md) for Python use and the clock/ownership contract. `pass::animate` captures the baked viewer's pose
lattice and returns `track::Replay`; `track::install` adds it to a timeline.
`pass::drive` accepts neutral input from other bakers. Both consume their
private world and never mutate the source scene, timeline or physics world.

The public track is Z-up SI and carries `baked_one_way_no_source_reaction`,
exact source/applied signal times and acquisition local anchors. The native
track additionally carries checked snapshots, velocities and raw last-substep
impulses. It is playback data, not a restart checkpoint or average force report.

This adapter follows the botrail workspace license. `rapier-rope` remains MIT;
its upstream dependency licenses remain separate.
