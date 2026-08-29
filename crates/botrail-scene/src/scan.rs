//! Simulated laser sweeps: the collider truth.
//!
//! A scan casts one ray per beam against the scene's *collision* shapes —
//! obstacles and robot links alike — so what it sees is exactly what the
//! cell can hit: massing bodies included, display-only detail excluded
//! (design/design-lidar.md 判断 L5; the honest opposite of the studio's
//! depth capture, which reads the rendered meshes). Deterministic and
//! headless: no browser, no hidden randomness, byte-stable across calls —
//! measurement noise is opt-in ([`ScanNoise`]) and itself a pure hash of
//! (seed, beam, instant), so a noisy sweep repeats bit-for-bit too.
//!
//! Two moments to sweep at: the scene as it stands ([`lidar_scan`]), or
//! any instant of a baked cycle ([`lidar_scan_at`] / [`scan_sweep`]) —
//! the latter walk the timeline the way the USD baker does: joint tracks
//! and baked bases into FK, object tracks for whatever moved, and the
//! vehicle's own frame track for a riding scanner.

use nalgebra::{Isometry3, Point3, Vector3};

use crate::rollout::SequenceTimeline;
use crate::seq::{DeviceKind, Lidar, LidarMount};
use crate::{Scene, SceneError};

/// Opt-in measurement noise for a sweep. Deterministic by construction:
/// each beam's perturbation is a pure hash of (seed, beam index, sweep
/// instant), so the same call is bit-stable, a different seed is an
/// independent draw, and a timeline sweep's frames vary beam to beam and
/// instant to instant the way a real range stream does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ScanNoise {
    /// Gaussian range noise, 1σ in meters (a VLP-16 datasheet's ±3 cm
    /// accuracy reads as `0.03` here). Applied to valid returns only —
    /// what a beam hits never changes, only how far it reports it —
    /// and the noisy range stays clamped to the measuring band.
    pub sigma: f64,
    /// Stream seed: keep it for a reproducible draw, change it for an
    /// independent one.
    pub seed: u64,
}

/// SplitMix64 — the classic 64-bit finalizer; enough hash for a
/// noise stream and dependency-free.
fn splitmix64(x: u64) -> u64 {
    let mut z = x.wrapping_add(0x9E37_79B9_7F4A_7C15);
    z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
    z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
    z ^ (z >> 31)
}

/// A standard-normal draw for one beam, hashed from the noise key —
/// Box–Muller over two unit uniforms.
fn gauss(key: u64) -> f64 {
    let unit = |v: u64| ((v >> 11) as f64 + 0.5) / (1u64 << 53) as f64;
    let u1 = unit(splitmix64(key));
    let u2 = unit(splitmix64(key ^ 0xD1B5_4A32_D192_ED03));
    (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
}

/// One simulated sweep of a [`Lidar`].
#[derive(Debug, Clone)]
pub struct LidarScan {
    /// Beam angles, degrees in the scan frame (0 = +X, CCW toward +Y).
    /// Degrees, not radians: the grid is authored in degrees
    /// (`fov_deg` / `resolution_deg`), and building it there keeps the
    /// usual grids exact (a 0.5° step lands on 20.0, not 20.000…018).
    /// A multi-channel scanner is ring-major: the full azimuth grid for
    /// the lowest ring first, then the next ring up — `angles` repeats
    /// per ring while `elevations` steps.
    pub angles: Vec<f64>,
    /// Beam elevations, degrees above the scan plane (also built in
    /// degrees — the grid rule above). All zero for a planar scanner; a
    /// 3D scanner spreads `channels` rings evenly across `vfov_deg`,
    /// bottom ring first.
    pub elevations: Vec<f64>,
    /// Nearest hit per beam, meters along the beam; `0.0` = no return
    /// (nothing inside the measuring band `[min, max]`).
    pub ranges: Vec<f64>,
    /// What each beam hit: an obstacle's name, `"{robot}/{link}"` for a
    /// robot link, `None` for no return.
    pub hits: Vec<Option<String>>,
    /// Scanner world pose at capture (the scan frame).
    pub pose: Isometry3<f64>,
    /// Timeline instant this sweep was taken at; `None` = the scene as
    /// authored (no bake involved).
    pub t: Option<f64>,
}

impl LidarScan {
    /// Hit points of the valid beams — world frame, or the scan frame
    /// with `world = false` (the scan plane is z = 0 there). `stride`
    /// thins the beam grid before the validity filter, so the same
    /// stride picks the same beams on every call.
    pub fn points(&self, world: bool, stride: usize) -> Vec<Point3<f64>> {
        let stride = stride.max(1);
        let mut out = Vec::new();
        for i in (0..self.ranges.len()).step_by(stride) {
            let r = self.ranges[i];
            if r <= 0.0 {
                continue;
            }
            let a = self.angles[i].to_radians();
            let e = self.elevations[i].to_radians();
            let local = Point3::new(r * e.cos() * a.cos(), r * e.cos() * a.sin(), r * e.sin());
            out.push(if world { self.pose * local } else { local });
        }
        out
    }
}

/// The world at one instant, resolved for the beam loop.
struct ScanState {
    frame: Isometry3<f64>,
    /// Per obstacle index: world pose at the instant.
    obstacle_poses: Vec<Isometry3<f64>>,
    /// Per obstacle index: participates in the sweep (enabled, not
    /// stowed, not the mount's own body).
    obstacle_active: Vec<bool>,
    /// Per robot: FK world poses at the instant.
    link_poses: Vec<Vec<Isometry3<f64>>>,
    exclude_link: Option<(usize, usize)>,
}

/// What a scanner's own mount hides from it: a vehicle mount's body
/// obstacles, a link mount's own link (the field rules — a massing
/// chassis or the mount link itself would blind the sweep from inside).
fn mount_exclusions(scene: &Scene, lidar: &Lidar) -> (Vec<usize>, Option<(usize, usize)>) {
    match &lidar.mount {
        LidarMount::World => (Vec::new(), None),
        LidarMount::Vehicle { device } => {
            let body = scene.devices().iter().find_map(|d| match &d.kind {
                DeviceKind::Vehicle { body, .. } if &d.name == device => Some(body),
                _ => None,
            });
            let obstacles = body
                .map(|names| {
                    names
                        .iter()
                        .filter_map(|n| scene.obstacles().iter().position(|o| &o.name == n))
                        .collect()
                })
                .unwrap_or_default();
            (obstacles, None)
        }
        LidarMount::Link { robot, link } => {
            let hit = scene
                .robot_index(robot)
                .and_then(|r| scene.robots()[r].model.link_index(link).map(|l| (r, l)));
            (Vec::new(), hit)
        }
    }
}

/// The parked frame of a vehicle device (its start station), identity
/// when the reference dangles.
fn parked_vehicle_frame(scene: &Scene, device: &str) -> Isometry3<f64> {
    scene
        .devices()
        .iter()
        .find_map(|d| match &d.kind {
            DeviceKind::Vehicle { path, start, .. } if d.name == device => path.frame_at(start),
            _ => None,
        })
        .unwrap_or_else(Isometry3::identity)
}

/// Sweeps the named lidar as the scene stands (a parked vehicle, the
/// current joint pose): `resolution_deg` steps across `fov_deg`
/// (adjusted to span the sweep exactly when they do not divide), each
/// beam returning the nearest collider hit within the measuring band.
pub fn lidar_scan(
    scene: &Scene,
    name: &str,
    noise: Option<ScanNoise>,
) -> Result<LidarScan, SceneError> {
    let Some(lidar) = scene.lidars().iter().find(|l| l.name == name) else {
        return Err(SceneError::UnknownLidar(name.to_string()));
    };
    let (exclude_obstacles, exclude_link) = mount_exclusions(scene, lidar);
    let link_poses: Vec<Vec<Isometry3<f64>>> = (0..scene.robots().len())
        .map(|r| scene.link_poses_for(r))
        .collect();
    let frame = match &lidar.mount {
        LidarMount::World => lidar.pose,
        LidarMount::Vehicle { device } => parked_vehicle_frame(scene, device) * lidar.pose,
        LidarMount::Link { .. } => match exclude_link {
            Some((r, l)) => link_poses[r][l] * lidar.pose,
            None => lidar.pose,
        },
    };
    let state = ScanState {
        frame,
        obstacle_poses: scene.obstacles().iter().map(|o| o.pose).collect(),
        obstacle_active: scene
            .obstacles()
            .iter()
            .enumerate()
            .map(|(i, o)| o.enabled && !exclude_obstacles.contains(&i))
            .collect(),
        link_poses,
        exclude_link,
    };
    Ok(sweep(scene, lidar, state, None, noise))
}

/// Sweeps the named lidar at instant `t` of a baked cycle (clamped to
/// its duration). `scene` is the pre-rollout snapshot the bake ran on —
/// the pair `store_baked` keeps. Joints, bases, moved objects and the
/// vehicle the scanner rides all come from the timeline's tracks;
/// whatever never moved keeps its authored pose, and a stowed object
/// (waiting in a magazine, taken off the line) drops out of the sweep
/// the way it drops out of the picture.
pub fn lidar_scan_at(
    scene: &Scene,
    timeline: &SequenceTimeline,
    name: &str,
    t: f64,
    noise: Option<ScanNoise>,
) -> Result<LidarScan, SceneError> {
    let Some(lidar) = scene.lidars().iter().find(|l| l.name == name) else {
        return Err(SceneError::UnknownLidar(name.to_string()));
    };
    let t = t.clamp(0.0, timeline.duration);
    let (exclude_obstacles, exclude_link) = mount_exclusions(scene, lidar);

    // The USD baker's walking recipe: joint track + baked base → FK.
    let link_poses: Vec<Vec<Isometry3<f64>>> = timeline
        .robots
        .iter()
        .enumerate()
        .map(|(r, track)| {
            let q = track.trajectory.sample(t);
            match SequenceTimeline::base_pose(track, t) {
                Some(base) => {
                    botrail_kin::forward_kinematics_with_base(&scene.robots()[r].model, &q, &base)
                        .map_err(|e| SceneError::BadLidar(format!("scan at t={t}: {e}")))
                }
                None => scene.fk_for(r, &q),
            }
        })
        .collect::<Result<_, _>>()?;

    let mut obstacle_poses = Vec::with_capacity(scene.obstacles().len());
    let mut obstacle_active = Vec::with_capacity(scene.obstacles().len());
    for (i, o) in scene.obstacles().iter().enumerate() {
        let track = timeline.objects.iter().find(|tr| tr.name == o.name);
        let pose = track
            .and_then(|tr| SequenceTimeline::object_pose(tr, &link_poses, t))
            .unwrap_or(o.pose);
        let stowed = track.is_some_and(|tr| !SequenceTimeline::object_visible(tr, t));
        obstacle_poses.push(pose);
        obstacle_active.push(o.enabled && !stowed && !exclude_obstacles.contains(&i));
    }

    let frame = match &lidar.mount {
        LidarMount::World => lidar.pose,
        LidarMount::Vehicle { device } => {
            // The vehicle's own frame track places a riding scanner; a
            // vehicle that never drove has no track and stays parked.
            let tracked = timeline
                .vehicles
                .iter()
                .find(|v| &v.name == device)
                .and_then(|tr| SequenceTimeline::object_pose(tr, &link_poses, t));
            tracked.unwrap_or_else(|| parked_vehicle_frame(scene, device)) * lidar.pose
        }
        LidarMount::Link { .. } => match exclude_link {
            Some((r, l)) => link_poses[r][l] * lidar.pose,
            None => lidar.pose,
        },
    };
    let state = ScanState {
        frame,
        obstacle_poses,
        obstacle_active,
        link_poses,
        exclude_link,
    };
    Ok(sweep(scene, lidar, state, Some(t), noise))
}

/// One sweep per frame of the export grid (`1/fps` steps plus the final
/// instant — the recording convention), over the whole cycle. The AGV
/// corridor survey: merge the frames' points and the drive's visibility
/// is one cloud.
pub fn scan_sweep(
    scene: &Scene,
    timeline: &SequenceTimeline,
    name: &str,
    fps: f64,
    noise: Option<ScanNoise>,
) -> Result<Vec<LidarScan>, SceneError> {
    if fps <= 0.0 || fps.is_nan() {
        return Err(SceneError::BadLidar(format!(
            "scan sweep: fps must be positive, got {fps}"
        )));
    }
    let mut times = Vec::new();
    let mut k = 0u64;
    loop {
        let t = k as f64 / fps;
        if t >= timeline.duration - 1e-9 {
            break;
        }
        times.push(t);
        k += 1;
    }
    times.push(timeline.duration);
    times
        .into_iter()
        .map(|t| lidar_scan_at(scene, timeline, name, t, noise))
        .collect()
}

/// The beam loop over one resolved instant.
fn sweep(
    scene: &Scene,
    lidar: &Lidar,
    state: ScanState,
    t: Option<f64>,
    noise: Option<ScanNoise>,
) -> LidarScan {
    let [min_range, max_range] = lidar.range;
    let origin = Point3::from(state.frame.translation.vector);

    // Candidates once, not per beam: active obstacles within reach
    // (their AABB against the scan sphere), inverse poses precomputed.
    let reach_cull = |aabb: Option<([f64; 3], [f64; 3])>| -> bool {
        let Some((lo, hi)) = aabb else { return true };
        let mut d2 = 0.0;
        for k in 0..3 {
            let c = origin[k].clamp(lo[k], hi[k]) - origin[k];
            d2 += c * c;
        }
        d2 <= max_range * max_range
    };
    let obstacles: Vec<(Isometry3<f64>, &crate::ObstacleCollider, &str)> = scene
        .obstacles()
        .iter()
        .zip(scene.obstacle_colliders.iter())
        .enumerate()
        .filter(|(i, (_, c))| {
            state.obstacle_active[*i] && reach_cull(c.aabb(&state.obstacle_poses[*i]))
        })
        .map(|(i, (o, c))| (state.obstacle_poses[i].inverse(), c, o.name.as_str()))
        .collect();
    let links: Vec<(Isometry3<f64>, usize, usize, String)> = scene
        .robots()
        .iter()
        .enumerate()
        .flat_map(|(r, robot)| {
            let collider = robot.collider();
            (0..robot.model.links.len())
                .filter(|l| state.exclude_link != Some((r, *l)) && collider.link_has_geometry(*l))
                .map(|l| {
                    (
                        state.link_poses[r][l].inverse(),
                        r,
                        l,
                        format!("{}/{}", robot.name, robot.model.links[l].name),
                    )
                })
                .collect::<Vec<_>>()
        })
        .collect();

    // The beam grid spans the sweep exactly, symmetric about +X; a full
    // circle drops the duplicate closing beam. Built in degrees (see
    // `LidarScan::angles`). Rings span the vertical field the same way,
    // symmetric about the scan plane, bottom first.
    let steps = ((lidar.fov_deg / lidar.resolution_deg).round().max(1.0)) as usize;
    let full = lidar.fov_deg >= 360.0 - 1e-9;
    let count = if full { steps } else { steps + 1 };
    let step = lidar.fov_deg / steps as f64;
    let rings: Vec<f64> = if lidar.channels <= 1 {
        vec![0.0]
    } else {
        let vstep = lidar.vfov_deg / (lidar.channels - 1) as f64;
        (0..lidar.channels)
            .map(|c| -lidar.vfov_deg / 2.0 + vstep * c as f64)
            .collect()
    };

    let beams = count * rings.len();
    let mut angles = Vec::with_capacity(beams);
    let mut elevations = Vec::with_capacity(beams);
    let mut ranges = Vec::with_capacity(beams);
    let mut hits: Vec<Option<String>> = Vec::with_capacity(beams);
    for elevation in &rings {
        let e = elevation.to_radians();
        let (ce, se) = (e.cos(), e.sin());
        for i in 0..count {
            let angle = -lidar.fov_deg / 2.0 + step * i as f64;
            let a = angle.to_radians();
            let dir = state.frame.rotation * Vector3::new(ce * a.cos(), ce * a.sin(), se);
            let mut best: Option<(f64, &str)> = None;
            for (inv, collider, name) in &obstacles {
                let lo = inv * origin;
                let ld = inv.transform_vector(&dir);
                if let Some(toi) = collider.cast_local_ray(&lo, &ld, max_range) {
                    if best.is_none_or(|(b, _)| toi < b) {
                        best = Some((toi, name));
                    }
                }
            }
            for (inv, r, l, label) in &links {
                let lo = inv * origin;
                let ld = inv.transform_vector(&dir);
                if let Some(toi) = scene.robots()[*r]
                    .collider()
                    .cast_link_ray(*l, &lo, &ld, max_range)
                {
                    if best.is_none_or(|(b, _)| toi < b) {
                        best = Some((toi, label.as_str()));
                    }
                }
            }
            angles.push(angle);
            elevations.push(*elevation);
            match best {
                Some((toi, name)) if toi >= min_range => {
                    // Noise perturbs the measured distance of a real hit,
                    // never what was hit; the key folds seed, beam index
                    // and instant so streams repeat bit-for-bit.
                    let toi = match noise {
                        Some(n) if n.sigma > 0.0 => {
                            let key = splitmix64(
                                splitmix64(n.seed ^ ranges.len() as u64)
                                    ^ t.map_or(0, f64::to_bits),
                            );
                            (toi + n.sigma * gauss(key)).clamp(min_range, max_range)
                        }
                        _ => toi,
                    };
                    ranges.push(toi);
                    hits.push(Some(name.to_string()));
                }
                // No hit, or inside the blind ring (an authoring mistake
                // can also put the origin inside a body): no valid return
                // — the real device's answer too.
                _ => {
                    ranges.push(0.0);
                    hits.push(None);
                }
            }
        }
    }
    LidarScan {
        angles,
        elevations,
        ranges,
        hits,
        pose: state.frame,
        t,
    }
}
