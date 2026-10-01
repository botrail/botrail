# Third-party dependency notices

rapier-rope itself is MIT licensed; see LICENSE. Dependency licenses remain their own.
This crate does not vendor upstream source. The table records the locked Linux x86_64
f32/f64 graphs, including test/example and build dependencies. Cargo obtains upstream
packages with their own license files. This is an inventory, not a license replacement.

Rapier 0.36.0 is Apache-2.0; Parry 0.31.1 is Apache-2.0. Application redistribution
must retain the notices required by the dependencies it distributes.

| Package | Version | Upstream SPDX expression | Used by | Normal/build precision |
|---|---|---|---|---|
| allocator-api2 | 0.2.21 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| approx | 0.5.1 | Apache-2.0 | f32, f64 | f32, f64 |
| arrayvec | 0.7.8 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| autocfg | 1.5.1 | Apache-2.0 OR MIT | f32, f64 | f32, f64 |
| bitflags | 2.13.2 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| block-buffer | 0.10.4 | MIT OR Apache-2.0 | f32, f64 | test/example |
| bytemuck | 1.25.2 | Zlib OR Apache-2.0 OR MIT | f32, f64 | f32, f64 |
| byteorder | 1.5.0 | Unlicense OR MIT | f32, f64 | f32, f64 |
| cfg-if | 1.0.5 | MIT OR Apache-2.0 | f32, f64 | test/example |
| cpufeatures | 0.2.17 | MIT OR Apache-2.0 | f32, f64 | test/example |
| crypto-common | 0.1.7 | MIT OR Apache-2.0 | f32, f64 | test/example |
| digest | 0.10.7 | MIT OR Apache-2.0 | f32, f64 | test/example |
| downcast-rs | 2.0.2 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| either | 1.18.0 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| ena | 0.14.4 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| equivalent | 1.0.2 | Apache-2.0 OR MIT | f32, f64 | f32, f64 |
| foldhash | 0.2.0 | Zlib | f32, f64 | f32, f64 |
| generic-array | 0.14.7 | MIT | f32, f64 | test/example |
| glam | 0.30.10 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| glam | 0.31.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| glam | 0.32.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| glam | 0.33.11 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| glamx | 0.3.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| hash32 | 0.3.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| hashbrown | 0.16.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| hashbrown | 0.17.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| heapless | 0.8.0 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| itoa | 1.0.18 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| libm | 0.2.16 | MIT | f32, f64 | f32, f64 |
| log | 0.4.34 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| matrixmultiply | 0.3.11 | MIT/Apache-2.0 | f32, f64 | f32, f64 |
| memchr | 2.8.3 | Unlicense OR MIT | f32, f64 | f32, f64 |
| nalgebra | 0.35.0 | Apache-2.0 | f32, f64 | f32, f64 |
| nalgebra-macros | 0.3.0 | Apache-2.0 | f32, f64 | f32, f64 |
| num-bigint | 0.4.8 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| num-complex | 0.4.6 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| num-derive | 0.5.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| num-integer | 0.1.47 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| num-rational | 0.4.2 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| num-traits | 0.2.19 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| ordered-float | 5.5.0 | MIT | f32, f64 | f32, f64 |
| parry3d | 0.31.1 | Apache-2.0 | f32 | f32 |
| parry3d-f64 | 0.31.1 | Apache-2.0 | f64 | f64 |
| proc-macro2 | 1.0.107 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| profiling | 1.0.18 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| profiling-procmacros | 1.0.18 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| quote | 1.0.47 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| rapier3d | 0.36.0 | Apache-2.0 | f32 | f32 |
| rapier3d-f64 | 0.36.0 | Apache-2.0 | f64 | f64 |
| rawpointer | 0.2.1 | MIT/Apache-2.0 | f32, f64 | f32, f64 |
| robust | 1.2.0 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| rstar | 0.13.0 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| safe_arch | 1.2.0 | Zlib OR Apache-2.0 OR MIT | f32, f64 | f32, f64 |
| serde | 1.0.229 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| serde_core | 1.0.229 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| serde_derive | 1.0.229 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| serde_json | 1.0.151 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| sha2 | 0.10.9 | MIT OR Apache-2.0 | f32, f64 | test/example |
| simba | 0.10.2 | Apache-2.0 | f32, f64 | f32, f64 |
| slab | 0.4.12 | MIT | f32, f64 | f32, f64 |
| smallvec | 1.16.2 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| spade | 2.15.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| stable_deref_trait | 1.2.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| static_assertions | 1.1.0 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| syn | 2.0.119 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| syn | 3.0.6 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| thiserror | 2.0.21 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| thiserror-impl | 2.0.21 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| typenum | 1.20.1 | MIT OR Apache-2.0 | f32, f64 | f32, f64 |
| unicode-ident | 1.0.26 | (MIT OR Apache-2.0) AND Unicode-3.0 | f32, f64 | f32, f64 |
| version_check | 0.9.5 | MIT/Apache-2.0 | f32, f64 | test/example |
| wide | 1.7.1 | Zlib OR Apache-2.0 OR MIT | f32, f64 | f32, f64 |
| zmij | 1.0.23 | MIT | f32, f64 | f32, f64 |
