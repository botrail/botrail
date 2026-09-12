//! Scaling probe for design-rl.md R2: N worlds stepped on rayon threads —
//! (a) bare rapier worlds, (b) live rollouts without a drive, (c) live
//! rollouts driven (per-tick safety read on). Run with
//! `RAYON_NUM_THREADS=k cargo run --release -p botrail-scene --features parallel --example vec_scaling`.

use std::sync::Arc;
use std::time::Instant;

use botrail_model::Geometry;
use botrail_physics::{BodyDesc, BodyKind, BodyProps, PhysicsBackend, WorldDesc};
use botrail_scene::rollout::RolloutOptions;
use botrail_scene::seq::{Condition, Sequence, Step};
use botrail_scene::Scene;
use nalgebra::{Isometry3, Vector3};
use rayon::prelude::*;

const ARM6: &str = include_str!("../../../examples/assets/simple_arm.urdf");

fn cell() -> Scene {
    let mut scene = Scene::new(Arc::new(
        botrail_model::RobotModel::from_urdf_str(ARM6).unwrap(),
    ));
    scene
        .set_joint_positions(vec![0.0, 0.6, 0.8, 0.0, 0.5, 0.0])
        .unwrap();
    scene
        .add_obstacle(
            "floor",
            Geometry::Box {
                size: Vector3::new(3.0, 3.0, 0.1),
            },
            Isometry3::translation(0.0, 0.0, -0.06),
        )
        .unwrap();
    scene
        .add_obstacle(
            "table",
            Geometry::Box {
                size: Vector3::new(0.6, 0.6, 0.1),
            },
            Isometry3::translation(0.55, 0.0, 0.05),
        )
        .unwrap();
    for i in 0..4 {
        let name = format!("part{i}");
        scene
            .add_obstacle(
                &name,
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                Isometry3::translation(0.45 + 0.07 * i as f64, 0.2, 0.135),
            )
            .unwrap();
        scene
            .set_obstacle_physics(&name, Some(BodyProps::dynamic()))
            .unwrap();
    }
    scene.upsert_sequence(Sequence {
        name: "run".into(),
        steps: vec![Step {
            name: "wait".into(),
            actions: vec![],
            transition: Condition::Elapsed { seconds: 60.0 },
            select: Vec::new(),
        }],
    });
    scene
}

fn rapier() -> Option<Box<dyn PhysicsBackend>> {
    Some(Box::new(botrail_physics_rapier::RapierBackend::new()))
}

fn bare_world() -> Box<dyn PhysicsBackend> {
    let mut desc = WorldDesc::new();
    let cuboid = |half: f64| parry3d_f64::shape::SharedShape::cuboid(half, half, half);
    let mut body = |kind: BodyKind, pose: Isometry3<f64>, half: f64| {
        desc.bodies.push(BodyDesc {
            kind,
            pose,
            parts: vec![(parry3d_f64::math::Pose::identity(), cuboid(half))],
            props: BodyProps {
                kind,
                ..BodyProps::default()
            },
            group: 0,
        });
    };
    body(
        BodyKind::Static,
        Isometry3::translation(0.0, 0.0, -0.5),
        0.5,
    );
    for i in 0..8 {
        body(
            BodyKind::Kinematic,
            Isometry3::translation(0.3 * i as f64, 0.5, 0.2),
            0.05,
        );
    }
    for i in 0..4 {
        body(
            BodyKind::Dynamic,
            Isometry3::translation(0.1 * i as f64, 0.0, 0.3 + 0.12 * i as f64),
            0.025,
        );
    }
    let mut backend = rapier().unwrap();
    backend.reset(&desc).unwrap();
    backend
}

fn time<F: FnMut()>(label: &str, n: usize, ticks: usize, mut f: F) {
    f();
    let t0 = Instant::now();
    let reps = 20;
    for _ in 0..reps {
        f();
    }
    let per = t0.elapsed().as_secs_f64() / reps as f64;
    println!(
        "{label:40} N={n:3}: {:.2} ms per {ticks} ticks -> {:.0} world-ticks/s",
        per * 1e3,
        (n * ticks) as f64 / per
    );
}

fn main() {
    let n: usize = std::env::var("N")
        .ok()
        .and_then(|v| v.parse().ok())
        .unwrap_or(32);
    let threads = rayon::current_num_threads();
    println!("rayon threads = {threads}");

    // (a) bare rapier: 4 substeps per tick, like the rollout.
    let mut worlds: Vec<Box<dyn PhysicsBackend>> = (0..n).map(|_| bare_world()).collect();
    time("bare rapier (4 substeps × 5 ticks)", n, 5, || {
        worlds.par_iter_mut().for_each(|w| {
            for _ in 0..20 {
                w.step(0.0025);
            }
        });
    });

    // (b) live rollouts, undriven.
    let scenes: Vec<Scene> = (0..n).map(|_| cell()).collect();
    let options = RolloutOptions::default();
    let mut lives: Vec<_> = scenes
        .iter()
        .map(|s| s.open_rollout(&["run"], &options, rapier()).unwrap())
        .collect();
    time("live rollout, undriven (5 ticks)", n, 5, || {
        lives.par_iter_mut().for_each(|l| {
            for _ in 0..5 {
                l.tick().unwrap();
            }
        });
    });

    // (c) live rollouts, driven (safety read every tick).
    let mut lives: Vec<_> = scenes
        .iter()
        .map(|s| s.open_rollout(&["run"], &options, rapier()).unwrap())
        .collect();
    for l in &mut lives {
        l.drive(0, None, None).unwrap();
    }
    time("live rollout, driven (5 ticks)", n, 5, || {
        lives.par_iter_mut().for_each(|l| {
            for _ in 0..5 {
                l.tick().unwrap();
            }
        });
    });

    // (d) kinematic only, undriven — the rollout without an engine.
    let mut lives: Vec<_> = scenes
        .iter()
        .map(|s| s.open_rollout(&["run"], &options, None).unwrap())
        .collect();
    time("live rollout, no physics (5 ticks)", n, 5, || {
        lives.par_iter_mut().for_each(|l| {
            for _ in 0..5 {
                l.tick().unwrap();
            }
        });
    });
}
