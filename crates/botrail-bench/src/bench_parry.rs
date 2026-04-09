//! parry3d mesh-mesh performance benchmark (pre-M2 risk validation).
//!
//! Answers, with numbers:
//! 1. Are TriMesh-TriMesh intersection/distance queries fast enough for
//!    interactive scene editing (full scene < ~2ms) and sampling-based
//!    planning (>= ~10k full-scene checks in seconds)?
//! 2. Does convex decomposition (VHACD -> Compound) pay off, and what does
//!    it cost offline?
//! 3. Correctness semantics: does full containment (one shape completely
//!    inside another) register as a collision for each representation?
//!
//! Note: parry >= 0.23 moved from nalgebra to glam (`Pose`, `DVec3`); the
//! bench uses parry's own math types so it is independent of the workspace
//! nalgebra version.
//!
//! Run with: cargo run --release -p botrail-bench

use std::hint::black_box;
use std::time::Instant;

use parry3d_f64::math::{Pose, Real, Vector};
use parry3d_f64::query;
use parry3d_f64::shape::{Ball, Compound, Cuboid, Shape, SharedShape, TriMesh, TriMeshFlags};
use parry3d_f64::transformation::vhacd::{VHACDParameters, VHACD};

// ---------------------------------------------------------------- mesh gen

/// Icosphere: 20 * 4^subdivisions triangles, radius `r`.
fn icosphere(subdivisions: u32, r: Real) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
    let mut verts: Vec<Vector> = [
        [-1.0, phi, 0.0],
        [1.0, phi, 0.0],
        [-1.0, -phi, 0.0],
        [1.0, -phi, 0.0],
        [0.0, -1.0, phi],
        [0.0, 1.0, phi],
        [0.0, -1.0, -phi],
        [0.0, 1.0, -phi],
        [phi, 0.0, -1.0],
        [phi, 0.0, 1.0],
        [-phi, 0.0, -1.0],
        [-phi, 0.0, 1.0],
    ]
    .iter()
    .map(|v| Vector::new(v[0], v[1], v[2]).normalize())
    .collect();
    let mut faces: Vec<[u32; 3]> = vec![
        [0, 11, 5],
        [0, 5, 1],
        [0, 1, 7],
        [0, 7, 10],
        [0, 10, 11],
        [1, 5, 9],
        [5, 11, 4],
        [11, 10, 2],
        [10, 7, 6],
        [7, 1, 8],
        [3, 9, 4],
        [3, 4, 2],
        [3, 2, 6],
        [3, 6, 8],
        [3, 8, 9],
        [4, 9, 5],
        [2, 4, 11],
        [6, 2, 10],
        [8, 6, 7],
        [9, 8, 1],
    ];
    for _ in 0..subdivisions {
        let mut midpoints = std::collections::HashMap::new();
        let mut next = Vec::with_capacity(faces.len() * 4);
        for [a, b, c] in faces {
            let mut midpoint = |a: u32, b: u32, verts: &mut Vec<Vector>| -> u32 {
                let key = (a.min(b), a.max(b));
                *midpoints.entry(key).or_insert_with(|| {
                    let m = ((verts[a as usize] + verts[b as usize]) / 2.0).normalize();
                    verts.push(m);
                    (verts.len() - 1) as u32
                })
            };
            let ab = midpoint(a, b, &mut verts);
            let bc = midpoint(b, c, &mut verts);
            let ca = midpoint(c, a, &mut verts);
            next.extend_from_slice(&[[a, ab, ca], [b, bc, ab], [c, ca, bc], [ab, bc, ca]]);
        }
        faces = next;
    }
    let points = verts.iter().map(|v| *v * r).collect();
    (points, faces)
}

/// Bumpy sphere: icosphere with sinusoidal radius modulation (non-convex,
/// organic-looking; a stand-in for detailed robot link meshes).
fn bumpy_sphere(subdivisions: u32, r: Real) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let (points, faces) = icosphere(subdivisions, 1.0);
    let points = points
        .iter()
        .map(|p| {
            let theta: Real = p.z.acos();
            let phi: Real = p.y.atan2(p.x);
            let bump = 1.0 + 0.15 * (5.0 * theta).sin() * (4.0 * phi).sin();
            *p * (r * bump)
        })
        .collect();
    (points, faces)
}

/// Torus (non-convex with a hole): 2 * nu * nv triangles.
fn torus(nu: u32, nv: u32, major: Real, minor: Real) -> (Vec<Vector>, Vec<[u32; 3]>) {
    let tau = std::f64::consts::TAU;
    let mut points = Vec::with_capacity((nu * nv) as usize);
    for i in 0..nu {
        let u = i as Real / nu as Real * tau;
        for j in 0..nv {
            let v = j as Real / nv as Real * tau;
            points.push(Vector::new(
                (major + minor * v.cos()) * u.cos(),
                (major + minor * v.cos()) * u.sin(),
                minor * v.sin(),
            ));
        }
    }
    let mut faces = Vec::with_capacity((2 * nu * nv) as usize);
    for i in 0..nu {
        for j in 0..nv {
            let a = i * nv + j;
            let b = ((i + 1) % nu) * nv + j;
            let c = ((i + 1) % nu) * nv + (j + 1) % nv;
            let d = i * nv + (j + 1) % nv;
            faces.push([a, b, c]);
            faces.push([a, c, d]);
        }
    }
    (points, faces)
}

// ------------------------------------------------------------- shape build

fn trimesh(data: &(Vec<Vector>, Vec<[u32; 3]>)) -> TriMesh {
    TriMesh::new(data.0.clone(), data.1.clone()).expect("valid mesh")
}

fn trimesh_oriented(data: &(Vec<Vector>, Vec<[u32; 3]>)) -> TriMesh {
    TriMesh::with_flags(data.0.clone(), data.1.clone(), TriMeshFlags::ORIENTED).expect("valid mesh")
}

fn convex_hull_shape(data: &(Vec<Vector>, Vec<[u32; 3]>)) -> SharedShape {
    SharedShape::convex_hull(&data.0).expect("hull")
}

fn vhacd_compound(
    data: &(Vec<Vector>, Vec<[u32; 3]>),
    params: &VHACDParameters,
) -> (Compound, usize) {
    let vhacd = VHACD::decompose(params, &data.0, &data.1, true);
    let hulls = vhacd.compute_convex_hulls(0);
    let shapes: Vec<_> = hulls
        .iter()
        .filter_map(|(pts, _)| SharedShape::convex_hull(pts))
        .map(|s| (Pose::identity(), s))
        .collect();
    let n = shapes.len();
    (Compound::new(shapes), n)
}

// ----------------------------------------------------------------- timing

/// Median time per call (adaptive batch size, 9 batches).
fn time_median<F: FnMut()>(mut f: F) -> f64 {
    let mut batch = 1u32;
    loop {
        let t = Instant::now();
        for _ in 0..batch {
            f();
        }
        let elapsed = t.elapsed().as_secs_f64();
        if elapsed > 2e-3 || batch >= 1 << 20 {
            break;
        }
        batch *= 2;
    }
    let mut samples = Vec::with_capacity(9);
    for _ in 0..9 {
        let t = Instant::now();
        for _ in 0..batch {
            f();
        }
        samples.push(t.elapsed().as_secs_f64() / batch as f64);
    }
    samples.sort_by(|a, b| a.partial_cmp(b).unwrap());
    samples[samples.len() / 2]
}

fn fmt_time(seconds: f64) -> String {
    if seconds < 1e-6 {
        format!("{:8.1} ns", seconds * 1e9)
    } else if seconds < 1e-3 {
        format!("{:8.2} us", seconds * 1e6)
    } else {
        format!("{:8.3} ms", seconds * 1e3)
    }
}

// ------------------------------------------------------------------ bench

/// Places `b` at distance `gap` along +x from `a` (both centered at origin,
/// with rough radii `ra`, `rb`): gap > 0 separated, < 0 penetrating.
fn poses(ra: Real, rb: Real, gap: Real) -> (Pose, Pose) {
    (
        Pose::identity(),
        Pose::from_translation(Vector::new(ra + rb + gap, 0.0, 0.0)),
    )
}

fn bench_pair(label: &str, a: &dyn Shape, b: &dyn Shape, ra: Real, rb: Real) {
    let configs: [(&str, Real); 3] = [("far", ra + rb), ("close", 0.001), ("overlap", -0.2 * rb)];
    for (cfg, gap) in configs {
        let (pa, pb) = poses(ra, rb, gap);
        let t_int = time_median(|| {
            black_box(query::intersection_test(&pa, a, &pb, b).unwrap());
        });
        let t_dist = time_median(|| {
            black_box(query::distance(&pa, a, &pb, b).unwrap());
        });
        let t_contact = time_median(|| {
            black_box(query::contact(&pa, a, &pb, b, 0.01).unwrap());
        });
        println!(
            "{:38} {:8} intersect {}   distance {}   contact {}",
            label,
            cfg,
            fmt_time(t_int),
            fmt_time(t_dist),
            fmt_time(t_contact),
        );
    }
}

fn containment_check(label: &str, big: &dyn Shape, small: &dyn Shape) {
    // Small shape fully inside the big one, at the center.
    let pa = Pose::identity();
    let pb = Pose::identity();
    let hit = query::intersection_test(&pa, big, &pb, small).unwrap();
    let dist = query::distance(&pa, big, &pb, small).unwrap();
    println!("{:55} intersect={:5}  distance={:.4}", label, hit, dist);
}

fn main() {
    println!(
        "parry3d-f64 {} mesh-mesh benchmark",
        env!("CARGO_PKG_VERSION")
    );
    println!("=========================================\n");

    // Mesh resolutions: label, icosphere subdivisions (20*4^n tris).
    let resolutions: [(&str, u32); 3] = [("320 tris", 2), ("5k tris", 4), ("20k tris", 5)];

    let vhacd_params = VHACDParameters {
        resolution: 64,
        ..Default::default()
    };

    println!("--- offline costs (mesh build / hull / VHACD decomposition) ---");
    for (label, sub) in resolutions {
        let bumpy = bumpy_sphere(sub, 0.1);
        let t_build = time_median(|| {
            black_box(TriMesh::new(bumpy.0.clone(), bumpy.1.clone()).unwrap());
        });
        let t_hull = time_median(|| {
            black_box(SharedShape::convex_hull(&bumpy.0).unwrap());
        });
        let t0 = Instant::now();
        let (_, pieces) = vhacd_compound(&bumpy, &vhacd_params);
        let t_vhacd = t0.elapsed().as_secs_f64();
        println!(
            "bumpy {:9}  trimesh+bvh {}   hull {}   vhacd(res=64) {} -> {} pieces",
            label,
            fmt_time(t_build),
            fmt_time(t_hull),
            fmt_time(t_vhacd),
            pieces
        );
    }
    println!();

    println!("--- pairwise queries (times are per call, median) ---");
    for (label, sub) in resolutions {
        let bumpy = bumpy_sphere(sub, 0.1);
        // Torus with a similar triangle count.
        let n = (20u32 * 4u32.pow(sub) / 2).max(16);
        let nu = (n as f64).sqrt().round() as u32;
        let tor = torus(nu.max(4), (n / nu.max(4)).max(4), 0.08, 0.03);

        let tm_a = trimesh(&bumpy);
        let tm_b = trimesh(&tor);
        let hull_a = convex_hull_shape(&bumpy);
        let hull_b = convex_hull_shape(&tor);
        let (comp_a, na) = vhacd_compound(&bumpy, &vhacd_params);
        let (comp_b, nb) = vhacd_compound(&tor, &vhacd_params);
        let ball = Ball::new(0.1);
        let ra = 0.115; // bumpy sphere max radius
        let rb = 0.11; // torus outer radius

        println!("[bumpy {} vs torus, vhacd pieces {}x{}]", label, na, nb);
        bench_pair("  trimesh vs trimesh", &tm_a, &tm_b, ra, rb);
        bench_pair("  hull vs hull", hull_a.as_ref(), hull_b.as_ref(), ra, rb);
        bench_pair("  vhacd vs vhacd", &comp_a, &comp_b, ra, rb);
        bench_pair("  trimesh vs ball(primitive)", &tm_a, &ball, ra, rb);
        println!();
    }

    // Baseline: primitive vs primitive.
    let cube = Cuboid::new(Vector::new(0.1, 0.1, 0.1));
    bench_pair("cuboid vs cuboid (baseline)", &cube, &cube, 0.17, 0.17);
    println!();

    println!("--- containment semantics (small mesh fully inside big sphere) ---");
    let big = icosphere(4, 0.2);
    let small = icosphere(2, 0.05);
    containment_check(
        "trimesh-in-trimesh (default flags)",
        &trimesh(&big),
        &trimesh(&small),
    );
    containment_check(
        "trimesh-in-trimesh (ORIENTED flags)",
        &trimesh_oriented(&big),
        &trimesh_oriented(&small),
    );
    containment_check(
        "hull-in-hull",
        convex_hull_shape(&big).as_ref(),
        convex_hull_shape(&small).as_ref(),
    );
    let (big_comp, _) = vhacd_compound(&big, &vhacd_params);
    let (small_comp, _) = vhacd_compound(&small, &vhacd_params);
    containment_check("vhacd-in-vhacd", &big_comp, &small_comp);
    println!();

    println!("--- simulated robot workload ---");
    // 8 links (5k-tri source meshes) in a chain with small gaps, 3 box +
    // 2 mesh obstacles nearby: 21 self pairs (adjacent skipped) + 40
    // link-obstacle pairs = 61 boolean checks per scene validation.
    let link_src = bumpy_sphere(4, 0.06);
    let (link_comp, _) = vhacd_compound(&link_src, &vhacd_params);
    let link_tm = trimesh(&link_src);
    let obstacle_mesh = trimesh(&torus(48, 24, 0.15, 0.05));
    let obstacle_box = Cuboid::new(Vector::new(0.2, 0.2, 0.02));

    let link_poses: Vec<Pose> = (0..8)
        .map(|i| Pose::from_translation(Vector::new(0.0, 0.0, 0.125 * i as Real)))
        .collect();
    let obstacle_poses: Vec<Pose> = (0..5)
        .map(|i| Pose::from_translation(Vector::new(0.3, 0.1 * i as Real, 0.2)))
        .collect();

    let scene_check = |links: &[&dyn Shape]| -> u32 {
        let mut hits = 0u32;
        for i in 0..links.len() {
            for j in (i + 2)..links.len() {
                if query::intersection_test(&link_poses[i], links[i], &link_poses[j], links[j])
                    .unwrap()
                {
                    hits += 1;
                }
            }
        }
        for (i, lp) in link_poses.iter().enumerate() {
            for (k, op) in obstacle_poses.iter().enumerate() {
                let obs: &dyn Shape = if k < 3 { &obstacle_box } else { &obstacle_mesh };
                if query::intersection_test(lp, links[i], op, obs).unwrap() {
                    hits += 1;
                }
            }
        }
        hits
    };

    let links_vhacd: Vec<&dyn Shape> = (0..8).map(|_| &link_comp as &dyn Shape).collect();
    let links_tm: Vec<&dyn Shape> = (0..8).map(|_| &link_tm as &dyn Shape).collect();

    let t_scene_vhacd = time_median(|| {
        black_box(scene_check(&links_vhacd));
    });
    let t_scene_tm = time_median(|| {
        black_box(scene_check(&links_tm));
    });
    println!(
        "full scene check (61 pairs)  vhacd links:   {}  ({:>9.0} scenes/s)",
        fmt_time(t_scene_vhacd),
        1.0 / t_scene_vhacd
    );
    println!(
        "full scene check (61 pairs)  trimesh links: {}  ({:>9.0} scenes/s)",
        fmt_time(t_scene_tm),
        1.0 / t_scene_tm
    );
}
