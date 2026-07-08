//! Minimal probe: distance between two axis-aligned cuboids.

use parry3d_f64::math::{Pose, Vector};
use parry3d_f64::query;
use parry3d_f64::shape::{Ball, Cuboid};

fn main() {
    let a = Cuboid::new(Vector::new(0.05, 0.05, 0.05));
    let b = Cuboid::new(Vector::new(0.1, 0.1, 0.1));
    let pa = Pose::identity();
    let pb = Pose::from_translation(Vector::new(1.0, 0.0, 0.0));
    println!(
        "cuboid-cuboid expected 0.85, got {}",
        query::distance(&pa, &a, &pb, &b).unwrap()
    );
    let ball = Ball::new(0.05);
    println!(
        "cuboid-ball   expected 0.85, got {}",
        query::distance(&pb, &b, &Pose::identity(), &ball).unwrap()
    );
    println!(
        "ball-ball     expected 0.85, got {}",
        query::distance(&Pose::identity(), &Ball::new(0.05), &pb, &Ball::new(0.1)).unwrap()
    );
    // Is contact() more accurate than distance() for the same pair?
    let c = query::contact(&pa, &a, &pb, &b, 10.0).unwrap();
    println!(
        "cuboid-cuboid contact.dist expected 0.85, got {:?}",
        c.map(|c| c.dist)
    );
    // Slightly rotated cuboid (non-degenerate for GJK): 10 deg about z.
    let rot = Pose {
        rotation: parry3d_f64::math::Rotation::from_rotation_z(0.1745),
        translation: Vector::new(1.0, 0.0, 0.0),
    };
    println!(
        "cuboid-cuboid rotated distance = {}",
        query::distance(&pa, &a, &rot, &b).unwrap()
    );
}
