//! Vectorised live rollouts and observation specs (design-rl.md R2).
//!
//! [`ObsSpec`] is the lowered form of a `botrail.rl` task's channel list:
//! names resolved to indices once, then [`ObsSpec::fill`] packs one world's
//! observation into a flat `f64` slice with no allocation. [`VecRollout`]
//! holds N independent [`LiveRollout`]s of the same cell and steps them
//! together — each world single-threaded (its bake stays bit-identical to
//! the batch bake), the worlds across threads when the `parallel` feature
//! is on — so a policy sees `(N, dim)` observations for `(N, dof)` joint
//! commands per control step.

use crate::rollout::{LiveRollout, RolloutOptions, SeqError, TickContact, WorldView};
use crate::Scene;
use botrail_physics::PhysicsBackend;
use nalgebra::{Isometry3, UnitQuaternion, Vector3};
#[cfg(feature = "parallel")]
use rayon::prelude::*;
use serde::Deserialize;

/// One observation channel as `botrail.rl` states it, by name. Poses are
/// `(x, y, z, qx, qy, qz, qw)`; a reference (`Relative`) is an obstacle
/// name, `<robot>/tcp`, or `<robot>/<link>`; contact bodies are named the
/// way the rollout names them (obstacles as is, links `<robot>/<link>`).
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ObsChannelSpec {
    Joints {
        robot: String,
        velocities: bool,
    },
    TcpPose {
        robot: String,
    },
    LinkPose {
        robot: String,
        link: String,
    },
    ObjectPose {
        obstacle: String,
    },
    ObjectVel {
        obstacle: String,
    },
    Relative {
        a: String,
        b: String,
    },
    Contacts {
        a: String,
        b: String,
    },
    Signal {
        signal: String,
    },
    Clearance {
        cap: f64,
    },
    Collision,
    /// A LiDAR sweep on the live world: one range per beam (meters; a
    /// beam with no return reads the scanner's max range), every
    /// `stride`-th azimuth of the listed `rings` (all by default), with
    /// Gaussian range noise `noise` (1σ m, 0: none) drawn from the
    /// world's seed and the tick (design-rl-sensors.md RS0).
    Lidar {
        lidar: String,
        #[serde(default = "one")]
        stride: usize,
        #[serde(default)]
        rings: Option<Vec<u32>>,
        #[serde(default)]
        noise: f64,
        /// Read the hit points instead of the ranges: `x, y, z` per beam
        /// in the scanner frame (zeros for no return), `3 × beams` wide.
        #[serde(default)]
        points: bool,
    },
    /// A depth picture from a camera, `width × height` pixels row-major
    /// (row 0 the top): Z along the optical axis in meters, `0` for no
    /// return or outside the camera's `[near, far]` — the studio's z16
    /// depth capture — with a sensor model on valid pixels, drawn from
    /// the world's seed and the tick (design-rl-sensors.md RS3):
    /// Gaussian noise `noise + noise_z2·z²` (1σ m — a stereo camera's
    /// error grows with the square of the range), disparity quantisation
    /// for a stereo `baseline` (m) at the picture's focal length, and a
    /// `dropout` fraction of pixels reading no return.
    Depth {
        camera: String,
        width: usize,
        height: usize,
        #[serde(default)]
        geometry: crate::raster::RenderGeometry,
        #[serde(default)]
        noise: f64,
        #[serde(default)]
        noise_z2: f64,
        #[serde(default)]
        baseline: Option<f64>,
        #[serde(default)]
        dropout: f64,
    },
    /// The segmentation id per pixel of the same picture (`0`
    /// background; `Scene::segmentation_ids` names the rest).
    Segmentation {
        camera: String,
        width: usize,
        height: usize,
        #[serde(default)]
        geometry: crate::raster::RenderGeometry,
    },
    /// The camera-frame point of every pixel of the same picture, `x, y,
    /// z` per pixel (`x` right, `y` up, `z` toward the viewer; zeros for
    /// no return): `3 × width × height` wide.
    PointCloud {
        camera: String,
        width: usize,
        height: usize,
        #[serde(default)]
        geometry: crate::raster::RenderGeometry,
    },
    /// A flat-shaded colour picture, `r, g, b` in `0..=255` per pixel
    /// (`3 × width × height` wide, black background), lit by the
    /// world's lighting (design-rl-sensors.md RS2).
    Rgb {
        camera: String,
        width: usize,
        height: usize,
        #[serde(default)]
        geometry: crate::raster::RenderGeometry,
    },
}

fn one() -> usize {
    1
}

#[derive(Debug, Clone)]
enum Ref {
    Obstacle(usize),
    Link { robot: usize, link: usize },
}

#[derive(Debug, Clone)]
enum ObsChannel {
    Joints {
        robot: usize,
        velocities: bool,
    },
    LinkPose {
        robot: usize,
        link: usize,
    },
    ObjectPose(usize),
    ObjectVel(String),
    Relative {
        a: Ref,
        b: Ref,
    },
    Contacts {
        a: String,
        b: String,
    },
    Signal(String),
    Clearance {
        cap: f64,
    },
    Collision,
    Lidar {
        index: usize,
        grid: crate::scan::ScanGrid,
        sigma: f64,
        max_range: f64,
        points: bool,
    },
    Depth {
        camera: usize,
        width: usize,
        height: usize,
        geometry: crate::raster::RenderGeometry,
        sigma: f64,
        sigma_z2: f64,
        baseline: Option<f64>,
        dropout: f64,
    },
    Segmentation {
        camera: usize,
        width: usize,
        height: usize,
        geometry: crate::raster::RenderGeometry,
    },
    PointCloud {
        camera: usize,
        width: usize,
        height: usize,
        geometry: crate::raster::RenderGeometry,
        fov_deg: f64,
    },
    Rgb {
        camera: usize,
        width: usize,
        height: usize,
        geometry: crate::raster::RenderGeometry,
    },
}

/// `(camera, width, height, geometry, shaded)`.
type PictureKey = (usize, usize, usize, crate::raster::RenderGeometry, bool);

/// The picture for `key`, rendered once per fill and shared by every
/// channel that reads it.
fn picture_of<'a>(
    pictures: &'a mut Vec<(PictureKey, Option<crate::raster::Frame>)>,
    live: &WorldView<'_>,
    key: PictureKey,
) -> Option<&'a crate::raster::Frame> {
    let at = match pictures.iter().position(|(k, _)| *k == key) {
        Some(i) => i,
        None => {
            let (camera, width, height, geometry, shaded) = key;
            let frame = if shaded {
                live.render_shaded(camera, geometry, width, height)
            } else {
                live.render(camera, geometry, width, height)
            };
            pictures.push((key, frame));
            pictures.len() - 1
        }
    };
    pictures[at].1.as_ref()
}

/// Resolves a picture channel's camera and size.
fn picture(scene: &Scene, name: &str, width: usize, height: usize) -> Result<usize, String> {
    let camera = scene
        .cameras()
        .iter()
        .position(|c| c.name == name)
        .ok_or_else(|| format!("observation: unknown camera `{name}`"))?;
    if width == 0 || height == 0 || width > 4096 || height > 4096 {
        return Err(format!(
            "observation: camera `{name}`: picture size {width}×{height} must be 1..=4096 a side"
        ));
    }
    Ok(camera)
}

/// A resolved channel list: what to read, in what order, how wide.
#[derive(Debug, Clone)]
pub struct ObsSpec {
    channels: Vec<(ObsChannel, usize)>,
    dim: usize,
}

fn write_pose(out: &mut [f64], pose: &Isometry3<f64>) {
    let t = pose.translation;
    let q = pose.rotation.coords;
    out[..7].copy_from_slice(&[t.x, t.y, t.z, q.x, q.y, q.z, q.w]);
}

impl ObsSpec {
    /// Resolves a JSON channel list (`[{"kind": "joints", ...}, ...]`)
    /// against `scene`.
    pub fn from_json(json: &str, scene: &Scene) -> Result<Self, String> {
        let specs: Vec<ObsChannelSpec> =
            serde_json::from_str(json).map_err(|e| format!("observation spec: {e}"))?;
        Self::resolve(&specs, scene)
    }

    /// Resolves channel names against `scene`: robots, links and
    /// obstacles must exist (contact and signal names are matched at read
    /// time — a lane or a pair that never appears simply reads zero).
    pub fn resolve(specs: &[ObsChannelSpec], scene: &Scene) -> Result<Self, String> {
        let robot = |name: &str| -> Result<usize, String> {
            scene
                .robot_index(name)
                .ok_or_else(|| format!("observation: unknown robot `{name}`"))
        };
        let link = |r: usize, name: &str| -> Result<usize, String> {
            scene.robots()[r]
                .model
                .link_index(name)
                .ok_or_else(|| format!("observation: unknown link `{name}`"))
        };
        let obstacle = |name: &str| -> Result<usize, String> {
            scene
                .obstacles()
                .iter()
                .position(|o| o.name == name)
                .ok_or_else(|| format!("observation: unknown obstacle `{name}`"))
        };
        let reference = |text: &str| -> Result<Ref, String> {
            if let Some((who, what)) = text.split_once('/') {
                let r = robot(who)?;
                let l = if what == "tcp" {
                    scene.robots()[r].model.default_tcp_link()
                } else {
                    link(r, what)?
                };
                Ok(Ref::Link { robot: r, link: l })
            } else {
                Ok(Ref::Obstacle(obstacle(text)?))
            }
        };
        let mut channels = Vec::with_capacity(specs.len());
        for spec in specs {
            let (channel, dim) = match spec {
                ObsChannelSpec::Joints {
                    robot: name,
                    velocities,
                } => {
                    let r = robot(name)?;
                    let dof = scene.robots()[r].model.dof();
                    (
                        ObsChannel::Joints {
                            robot: r,
                            velocities: *velocities,
                        },
                        if *velocities { 2 * dof } else { dof },
                    )
                }
                ObsChannelSpec::TcpPose { robot: name } => {
                    let r = robot(name)?;
                    let l = scene.robots()[r].model.default_tcp_link();
                    (ObsChannel::LinkPose { robot: r, link: l }, 7)
                }
                ObsChannelSpec::LinkPose {
                    robot: name,
                    link: ln,
                } => {
                    let r = robot(name)?;
                    (
                        ObsChannel::LinkPose {
                            robot: r,
                            link: link(r, ln)?,
                        },
                        7,
                    )
                }
                ObsChannelSpec::ObjectPose { obstacle: name } => {
                    (ObsChannel::ObjectPose(obstacle(name)?), 7)
                }
                ObsChannelSpec::ObjectVel { obstacle: name } => {
                    obstacle(name)?;
                    (ObsChannel::ObjectVel(name.clone()), 6)
                }
                ObsChannelSpec::Relative { a, b } => (
                    ObsChannel::Relative {
                        a: reference(a)?,
                        b: reference(b)?,
                    },
                    7,
                ),
                ObsChannelSpec::Contacts { a, b } => (
                    ObsChannel::Contacts {
                        a: a.clone(),
                        b: b.clone(),
                    },
                    2,
                ),
                ObsChannelSpec::Signal { signal } => (ObsChannel::Signal(signal.clone()), 1),
                ObsChannelSpec::Clearance { cap } => (ObsChannel::Clearance { cap: *cap }, 1),
                ObsChannelSpec::Collision => (ObsChannel::Collision, 1),
                ObsChannelSpec::Lidar {
                    lidar: name,
                    stride,
                    rings,
                    noise,
                    points,
                } => {
                    let index = scene
                        .lidars()
                        .iter()
                        .position(|l| &l.name == name)
                        .ok_or_else(|| format!("observation: unknown lidar `{name}`"))?;
                    let lidar = &scene.lidars()[index];
                    if *stride == 0 {
                        return Err(format!(
                            "observation: lidar `{name}`: stride must be at least 1"
                        ));
                    }
                    if let Some(list) = rings {
                        if let Some(bad) = list.iter().find(|&&r| r >= lidar.channels.max(1)) {
                            return Err(format!(
                                "observation: lidar `{name}` has {} ring(s), no ring {bad}",
                                lidar.channels.max(1)
                            ));
                        }
                    }
                    if !noise.is_finite() || *noise < 0.0 {
                        return Err(format!(
                            "observation: lidar `{name}`: noise must be finite and non-negative"
                        ));
                    }
                    let grid = crate::scan::ScanGrid {
                        stride: *stride,
                        rings: rings.clone(),
                    };
                    let beams = grid.beams(lidar);
                    (
                        ObsChannel::Lidar {
                            index,
                            grid,
                            sigma: *noise,
                            max_range: lidar.range[1],
                            points: *points,
                        },
                        if *points { 3 * beams } else { beams },
                    )
                }
                ObsChannelSpec::Depth {
                    camera: name,
                    width,
                    height,
                    geometry,
                    noise,
                    noise_z2,
                    baseline,
                    dropout,
                } => {
                    let camera = picture(scene, name, *width, *height)?;
                    for (label, value) in [
                        ("noise", *noise),
                        ("noise_z2", *noise_z2),
                        ("dropout", *dropout),
                    ] {
                        if !value.is_finite() || value < 0.0 {
                            return Err(format!("observation: depth `{name}`: {label} must be finite and non-negative"));
                        }
                    }
                    if *dropout > 1.0 {
                        return Err(format!(
                            "observation: depth `{name}`: dropout is a fraction in [0, 1]"
                        ));
                    }
                    if let Some(b) = baseline {
                        if !(b.is_finite() && *b > 0.0) {
                            return Err(format!(
                                "observation: depth `{name}`: baseline must be positive"
                            ));
                        }
                    }
                    (
                        ObsChannel::Depth {
                            camera,
                            width: *width,
                            height: *height,
                            geometry: *geometry,
                            sigma: *noise,
                            sigma_z2: *noise_z2,
                            baseline: *baseline,
                            dropout: *dropout,
                        },
                        width * height,
                    )
                }
                ObsChannelSpec::Segmentation {
                    camera: name,
                    width,
                    height,
                    geometry,
                } => (
                    ObsChannel::Segmentation {
                        camera: picture(scene, name, *width, *height)?,
                        width: *width,
                        height: *height,
                        geometry: *geometry,
                    },
                    width * height,
                ),
                ObsChannelSpec::PointCloud {
                    camera: name,
                    width,
                    height,
                    geometry,
                } => {
                    let camera = picture(scene, name, *width, *height)?;
                    (
                        ObsChannel::PointCloud {
                            camera,
                            width: *width,
                            height: *height,
                            geometry: *geometry,
                            fov_deg: scene.cameras()[camera].fov_deg,
                        },
                        3 * width * height,
                    )
                }
                ObsChannelSpec::Rgb {
                    camera: name,
                    width,
                    height,
                    geometry,
                } => (
                    ObsChannel::Rgb {
                        camera: picture(scene, name, *width, *height)?,
                        width: *width,
                        height: *height,
                        geometry: *geometry,
                    },
                    3 * width * height,
                ),
            };
            channels.push((channel, dim));
        }
        let dim = channels.iter().map(|(_, d)| d).sum();
        Ok(ObsSpec { channels, dim })
    }

    /// Total width of one observation.
    pub fn dim(&self) -> usize {
        self.dim
    }

    /// Widths per channel, in order.
    pub fn dims(&self) -> Vec<usize> {
        self.channels.iter().map(|(_, d)| *d).collect()
    }

    /// Packs the world's observation into `out` (exactly `dim` wide).
    /// Link poses are computed once per robot per call; a camera picture
    /// once per camera, size and geometry per call, shared by its depth,
    /// segmentation and point-cloud channels.
    pub fn fill(&self, live: WorldView<'_>, out: &mut [f64]) {
        assert_eq!(out.len(), self.dim, "observation buffer width");
        let scene = live.scene();
        let mut pictures: Vec<(PictureKey, Option<crate::raster::Frame>)> = Vec::new();
        let mut poses: Vec<Option<Vec<Isometry3<f64>>>> = vec![None; scene.robots().len()];
        let mut link_pose = |r: usize, l: usize| -> Isometry3<f64> {
            if poses[r].is_none() {
                poses[r] = live.link_poses(r);
            }
            poses[r].as_ref().expect("robot exists")[l]
        };
        let mut at = 0usize;
        for (channel, dim) in &self.channels {
            let out = &mut out[at..at + dim];
            at += dim;
            match channel {
                ObsChannel::Joints { robot, velocities } => {
                    let q = live.joint_positions(*robot).unwrap_or(&[]);
                    out[..q.len()].copy_from_slice(q);
                    if *velocities {
                        let v = live.joint_velocities(*robot).unwrap_or_default();
                        out[q.len()..].copy_from_slice(&v);
                    }
                }
                ObsChannel::LinkPose { robot, link } => {
                    write_pose(out, &link_pose(*robot, *link));
                }
                ObsChannel::ObjectPose(i) => write_pose(out, &scene.obstacles()[*i].pose),
                ObsChannel::ObjectVel(name) => match live.obstacle_velocity(name) {
                    Some(v) => out.copy_from_slice(&[
                        v.linear.x,
                        v.linear.y,
                        v.linear.z,
                        v.angular.x,
                        v.angular.y,
                        v.angular.z,
                    ]),
                    None => out.fill(0.0),
                },
                ObsChannel::Relative { a, b } => {
                    let mut pose_of = |r: &Ref| match r {
                        Ref::Obstacle(i) => scene.obstacles()[*i].pose,
                        Ref::Link { robot, link } => link_pose(*robot, *link),
                    };
                    let (pa, pb) = (pose_of(a), pose_of(b));
                    write_pose(out, &(pa.inverse() * pb));
                }
                ObsChannel::Contacts { a, b } => {
                    out.fill(0.0);
                    for c in live.contacts() {
                        if (&c.a == a && &c.b == b) || (&c.a == b && &c.b == a) {
                            out[0] = 1.0;
                            out[1] = c.force;
                            break;
                        }
                    }
                }
                ObsChannel::Signal(name) => {
                    out[0] = if live.signal(name).unwrap_or(false) {
                        1.0
                    } else {
                        0.0
                    };
                }
                ObsChannel::Clearance { cap } => {
                    out[0] = live.clearance().map(|d| d.min(*cap)).unwrap_or(*cap);
                }
                ObsChannel::Collision => {
                    out[0] = if live.collisions().is_empty() {
                        0.0
                    } else {
                        1.0
                    };
                }
                ObsChannel::Lidar {
                    index,
                    grid,
                    sigma,
                    max_range,
                    points,
                } => match live.lidar_scan(*index, grid, *sigma) {
                    Some(scan) if *points => {
                        for (slot, k) in out.chunks_mut(3).zip(0..scan.ranges.len()) {
                            let r = scan.ranges[k];
                            if r > 0.0 {
                                let a = scan.angles[k].to_radians();
                                let e = scan.elevations[k].to_radians();
                                slot.copy_from_slice(&[
                                    r * e.cos() * a.cos(),
                                    r * e.cos() * a.sin(),
                                    r * e.sin(),
                                ]);
                            } else {
                                slot.fill(0.0);
                            }
                        }
                    }
                    Some(scan) => {
                        for (slot, r) in out.iter_mut().zip(&scan.ranges) {
                            *slot = if *r > 0.0 { *r } else { *max_range };
                        }
                    }
                    None => out.fill(if *points { 0.0 } else { *max_range }),
                },
                ObsChannel::Depth {
                    camera,
                    width,
                    height,
                    geometry,
                    sigma,
                    sigma_z2,
                    baseline,
                    dropout,
                } => match picture_of(
                    &mut pictures,
                    &live,
                    (*camera, *width, *height, *geometry, false),
                ) {
                    Some(frame) => {
                        // The sensor model perturbs valid pixels only,
                        // keyed on the world's seed, the tick, the camera
                        // and the pixel — the LiDAR stream's rule.
                        let cam = &scene.cameras()[*camera];
                        let (fx, _, _, _) = crate::raster::intrinsics(cam.fov_deg, *width, *height);
                        let base = crate::scan::splitmix64(
                            live.noise_seed()
                                ^ live.ticks().wrapping_mul(0x9E37_79B9)
                                ^ (*camera as u64) << 48,
                        );
                        for (i, (slot, d)) in out.iter_mut().zip(&frame.depth).enumerate() {
                            let mut d = *d as f64;
                            if d <= 0.0 {
                                *slot = 0.0;
                                continue;
                            }
                            let key = crate::scan::splitmix64(base ^ i as u64);
                            let s = *sigma + *sigma_z2 * d * d;
                            if s > 0.0 {
                                d += s * crate::scan::gauss(key);
                            }
                            if let Some(b) = baseline {
                                // A stereo camera measures disparity in
                                // whole pixels: fx·B/z rounded, back to z.
                                let disparity = (fx * b / d.max(1e-6)).round().max(1.0);
                                d = fx * b / disparity;
                            }
                            if *dropout > 0.0 {
                                let u = (crate::scan::splitmix64(key ^ 0xD00F) >> 11) as f64
                                    / (1u64 << 53) as f64;
                                if u < *dropout {
                                    *slot = 0.0;
                                    continue;
                                }
                            }
                            *slot = d.clamp(cam.near, cam.far);
                        }
                    }
                    None => out.fill(0.0),
                },
                ObsChannel::Segmentation {
                    camera,
                    width,
                    height,
                    geometry,
                } => match picture_of(
                    &mut pictures,
                    &live,
                    (*camera, *width, *height, *geometry, false),
                ) {
                    Some(frame) => {
                        for (slot, id) in out.iter_mut().zip(&frame.id) {
                            *slot = *id as f64;
                        }
                    }
                    None => out.fill(0.0),
                },
                ObsChannel::PointCloud {
                    camera,
                    width,
                    height,
                    geometry,
                    fov_deg,
                } => match picture_of(
                    &mut pictures,
                    &live,
                    (*camera, *width, *height, *geometry, false),
                ) {
                    Some(frame) => {
                        for (slot, p) in out
                            .chunks_mut(3)
                            .zip(crate::raster::points(frame, *fov_deg))
                        {
                            slot.copy_from_slice(&p);
                        }
                    }
                    None => out.fill(0.0),
                },
                ObsChannel::Rgb {
                    camera,
                    width,
                    height,
                    geometry,
                } => match picture_of(
                    &mut pictures,
                    &live,
                    (*camera, *width, *height, *geometry, true),
                )
                .and_then(|f| f.rgb.as_ref())
                {
                    Some(rgb) => {
                        for (slot, v) in out.iter_mut().zip(rgb) {
                            *slot = *v as f64;
                        }
                    }
                    None => out.fill(0.0),
                },
            }
        }
    }
}

/// How an action becomes the driven robot's joint command — the lowered
/// form of a `botrail.rl` control spec (`JointDelta`, `JointTarget`,
/// `TcpDelta`), so the mapping — the Cartesian setpoint integration and
/// its IK included — runs in the rollout, per world, inside the parallel
/// step. Indices are q indices of the driven robot; limits are
/// `[lower, upper]`; a gripper entry is `[q index, lower, upper]`.
#[derive(Debug, Clone, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum ControlSpec {
    /// `q[i] += action[k] * max_step` per driven joint.
    JointDelta { indices: Vec<usize>, max_step: f64 },
    /// `q[i] = lower + (action[k] + 1) / 2 * (upper - lower)`.
    JointTarget {
        indices: Vec<usize>,
        limits: Vec<[f64; 2]>,
    },
    /// A Cartesian step of the TCP: 3 translations scaled by `max_step_m`
    /// (+ 3 rotations scaled by `max_step_rad` when `rotate`), in the
    /// `"world"` or the `"tcp"` frame, integrated into a setpoint and
    /// solved to joints from the current configuration with no restarts;
    /// a step the IK cannot solve is dropped (the setpoint stays, the
    /// joints hold). Then one gripper value per entry, `[-1, 1]` onto
    /// its limits.
    TcpDelta {
        max_step_m: f64,
        max_step_rad: f64,
        rotate: bool,
        frame: String,
        #[serde(default)]
        gripper: Vec<(usize, f64, f64)>,
        #[serde(default = "default_ik_iters")]
        max_iters: usize,
    },
}

fn default_ik_iters() -> usize {
    100
}

/// A control spec resolved for one robot of a cell: the TCP link and IK
/// joint mask it solves with, and the action width it takes.
#[derive(Debug, Clone)]
pub struct Control {
    spec: ControlSpec,
    robot: usize,
    tip: usize,
    mask: Option<Vec<bool>>,
    dim: usize,
}

/// A control's per-world state: the TCP setpoint a `TcpDelta` integrates.
#[derive(Debug, Clone, Default)]
pub struct ControlState {
    setpoint: Option<Isometry3<f64>>,
}

impl Control {
    /// Resolves a JSON spec for `robot` (driven as `group`, or whole).
    pub fn from_json(
        json: &str,
        scene: &Scene,
        robot: usize,
        group: Option<usize>,
    ) -> Result<Self, String> {
        let spec: ControlSpec =
            serde_json::from_str(json).map_err(|e| format!("control spec: {e}"))?;
        Self::resolve(spec, scene, robot, group)
    }

    /// Resolves a spec for `robot`: joint indices must exist; a
    /// `TcpDelta` solves for the group's tip (the robot's TCP without a
    /// group) with the group's joints (all of them without one).
    pub fn resolve(
        spec: ControlSpec,
        scene: &Scene,
        robot: usize,
        group: Option<usize>,
    ) -> Result<Self, String> {
        let sr = scene
            .robots()
            .get(robot)
            .ok_or_else(|| format!("control: no robot at index {robot}"))?;
        let model = &sr.model;
        let dof = model.dof();
        let check = |indices: &[usize]| -> Result<(), String> {
            match indices.iter().find(|&&i| i >= dof) {
                Some(bad) => Err(format!(
                    "control: joint index {bad} out of range (dof {dof})"
                )),
                None => Ok(()),
            }
        };
        let groups = model.groups();
        let (tip, mask) = match group {
            Some(g) => {
                let group = groups
                    .get(g)
                    .ok_or_else(|| format!("control: no group {g}"))?;
                (group.tip, Some(crate::motion::group_mask(dof, group)))
            }
            None if groups.len() == 1 => (
                groups[0].tip,
                Some(crate::motion::group_mask(dof, &groups[0])),
            ),
            None => (model.default_tcp_link(), None),
        };
        let dim = match &spec {
            ControlSpec::JointDelta { indices, .. } => {
                check(indices)?;
                indices.len()
            }
            ControlSpec::JointTarget { indices, limits } => {
                check(indices)?;
                if limits.len() != indices.len() {
                    return Err(format!(
                        "control: {} limits for {} joints",
                        limits.len(),
                        indices.len()
                    ));
                }
                indices.len()
            }
            ControlSpec::TcpDelta {
                rotate,
                frame,
                gripper,
                max_step_m,
                ..
            } => {
                if frame != "world" && frame != "tcp" {
                    return Err(format!(
                        "control: frame must be \"world\" or \"tcp\", got {frame:?}"
                    ));
                }
                if !max_step_m.is_finite() || *max_step_m < 0.0 {
                    return Err(format!(
                        "control: max_step_m must be finite and non-negative, got {max_step_m}"
                    ));
                }
                let indices: Vec<usize> = gripper.iter().map(|g| g.0).collect();
                check(&indices)?;
                3 + if *rotate { 3 } else { 0 } + gripper.len()
            }
        };
        Ok(Control {
            spec,
            robot,
            tip,
            mask,
            dim,
        })
    }

    /// Width of one action.
    pub fn dim(&self) -> usize {
        self.dim
    }

    /// The robot this control drives.
    pub fn robot(&self) -> usize {
        self.robot
    }

    /// A fresh state: the setpoint starts where the TCP stands at the
    /// first action.
    pub fn start(&self) -> ControlState {
        ControlState::default()
    }

    /// The joint command for `action` (each value clipped to `[-1, 1]`)
    /// on the world as it stands, and whether the IK converged (`true`
    /// for the joint controls).
    pub fn apply(
        &self,
        state: &mut ControlState,
        view: WorldView<'_>,
        action: &[f64],
    ) -> Result<(Vec<f64>, bool), String> {
        if action.len() != self.dim {
            return Err(format!(
                "action has {} values, the control takes {}",
                action.len(),
                self.dim
            ));
        }
        let a = |k: usize| action[k].clamp(-1.0, 1.0);
        let mut q = view
            .joint_positions(self.robot)
            .ok_or_else(|| format!("no robot at index {}", self.robot))?
            .to_vec();
        match &self.spec {
            ControlSpec::JointDelta { indices, max_step } => {
                for (k, &qi) in indices.iter().enumerate() {
                    q[qi] += a(k) * max_step;
                }
                Ok((q, true))
            }
            ControlSpec::JointTarget { indices, limits } => {
                for (k, &qi) in indices.iter().enumerate() {
                    let [lo, hi] = limits[k];
                    q[qi] = lo + (a(k) + 1.0) * 0.5 * (hi - lo);
                }
                Ok((q, true))
            }
            ControlSpec::TcpDelta {
                max_step_m,
                max_step_rad,
                rotate,
                frame,
                gripper,
                max_iters,
            } => {
                let scene = view.scene();
                let setpoint = match state.setpoint {
                    Some(p) => p,
                    None => {
                        let poses = view
                            .link_poses(self.robot)
                            .ok_or_else(|| format!("no robot at index {}", self.robot))?;
                        poses[self.tip]
                    }
                };
                let tcp_frame = frame == "tcp";
                let mut step = Vector3::new(a(0), a(1), a(2)) * *max_step_m;
                if tcp_frame {
                    step = setpoint.rotation * step;
                }
                let translation = setpoint.translation.vector + step;
                let mut k = 3;
                let rotation = if *rotate {
                    let rot = UnitQuaternion::from_scaled_axis(
                        Vector3::new(a(3), a(4), a(5)) * *max_step_rad,
                    );
                    k = 6;
                    // About the tool's own axes, or the world's.
                    if tcp_frame {
                        setpoint.rotation * rot
                    } else {
                        rot * setpoint.rotation
                    }
                } else {
                    setpoint.rotation
                };
                let target = Isometry3::from_parts(translation.into(), rotation);
                let options = botrail_kin::IkOptions {
                    mode: botrail_kin::IkMode::Pose,
                    max_iters: *max_iters,
                    restarts: 0,
                    joint_mask: self.mask.clone(),
                    ..botrail_kin::IkOptions::default()
                };
                let result = scene
                    .solve_ik_world_for(self.robot, self.tip, &target, &q, &options)
                    .map_err(|e| e.to_string())?;
                let converged = result.converged;
                if converged {
                    state.setpoint = Some(target);
                    q = result.q;
                } else if state.setpoint.is_none() {
                    state.setpoint = Some(setpoint);
                }
                for (i, &(qi, lo, hi)) in gripper.iter().enumerate() {
                    q[qi] = lo + (a(k + i) + 1.0) * 0.5 * (hi - lo);
                }
                Ok((q, converged))
            }
        }
    }
}

/// What a policy reads at each decision (design-rl.md R3): the packed
/// observation of its channels plus the raw essentials — joints, TCP
/// pose, this tick's contacts and the safety read.
pub struct PolicyInput<'a> {
    pub t: f64,
    /// Decisions taken in this run so far (0 at the step's entry).
    pub step: u64,
    pub obs: &'a [f64],
    /// The driven robot's joints.
    pub q: &'a [f64],
    pub tcp: Isometry3<f64>,
    pub collisions: &'a [(String, String)],
    pub contacts: &'a [TickContact],
}

/// A controller a `Policy` step hands a robot to: a learned policy, a
/// scripted one — anything that turns an observation into the next
/// joint target. Registered by name at bake time
/// (`Scene::simulate_sequences_driven`).
pub trait PolicyDriver: Send + Sync {
    /// The JSON channel list this policy observes (`ObsChannelSpec`s),
    /// resolved against the scene before the bake starts.
    fn channels(&self) -> &str;

    /// The JSON control spec ([`ControlSpec`]) this policy acts through,
    /// if any: then `act` answers an action, which the rollout turns into
    /// the joint target (IK included). Without one, `act` answers the
    /// joint target itself.
    fn control(&self) -> Option<&str> {
        None
    }

    /// The next action — or, with no control, the joint target (full-
    /// length q; the driven joints are read) — or `None` when the policy
    /// declares its step done. An `Err` fails the bake with the message.
    fn act(&mut self, input: &PolicyInput<'_>) -> Result<Option<Vec<f64>>, String>;
}

/// What one world reports after a batched step.
#[derive(Debug, Clone, Default)]
pub struct StepResult {
    pub t: f64,
    /// Every program of the world has finished.
    pub finished: bool,
    /// The bake's own failure (a timeout, a robot-vs-robot collision):
    /// the world is dead until `reopen`ed; its observation is zero.
    pub error: Option<String>,
    pub collisions: Vec<(String, String)>,
    pub contacts: Vec<TickContact>,
    /// The control's IK did not converge this step (`step_actions` under
    /// a `TcpDelta`): the setpoint held, the joints did not move.
    pub ik_failed: bool,
}

/// A physics backend per world; `None` is the kinematic bake.
pub type BackendFactory<'a> = &'a (dyn Fn() -> Option<Box<dyn PhysicsBackend>> + Sync);

/// How every world of a [`VecRollout`] is opened and driven.
#[derive(Clone)]
pub struct VecOptions {
    pub names: Vec<String>,
    pub options: RolloutOptions,
    /// The driven robot (the same index in every world — one cell).
    pub robot: usize,
    pub group: Option<usize>,
    pub max_velocity: Option<f64>,
    /// Base of the worlds' sensor-noise seeds: world `i` draws from
    /// `seed + i`.
    pub seed: u64,
}

/// N live rollouts of one cell, stepped together.
pub struct VecRollout {
    worlds: Vec<Option<LiveRollout>>,
    spec: ObsSpec,
    dof: usize,
    robot: usize,
    /// The control actions go through (`step_actions`), with one state
    /// per world.
    control: Option<Control>,
    states: Vec<ControlState>,
}

fn open_one(
    scene: &Scene,
    opts: &VecOptions,
    backend: BackendFactory<'_>,
    index: usize,
) -> Result<LiveRollout, SeqError> {
    let names: Vec<&str> = opts.names.iter().map(String::as_str).collect();
    let mut live = scene.open_rollout(&names, &opts.options, backend())?;
    live.set_noise_seed(opts.seed.wrapping_add(index as u64));
    live.drive(opts.robot, opts.group, opts.max_velocity)?;
    Ok(live)
}

impl VecRollout {
    /// Opens one world per scene (all the same cell: the spec is resolved
    /// against the first and must fit every one) and hands each driven
    /// robot to the caller. Worlds open in parallel under the `parallel`
    /// feature.
    pub fn open(
        scenes: &[Scene],
        opts: &VecOptions,
        backend: BackendFactory<'_>,
        spec: ObsSpec,
        control: Option<Control>,
    ) -> Result<Self, SeqError> {
        let first = scenes.first().ok_or_else(|| SeqError::Validation {
            step: None,
            message: "a vectorised rollout needs at least one scene".into(),
        })?;
        let dof = first
            .robots()
            .get(opts.robot)
            .map(|r| r.model.dof())
            .ok_or_else(|| SeqError::Validation {
                step: None,
                message: format!("no robot at index {}", opts.robot),
            })?;
        let indices: Vec<usize> = (0..scenes.len()).collect();
        let worlds = Self::open_many(scenes, opts, backend, &indices)?;
        let states = control
            .as_ref()
            .map(|c| worlds.iter().map(|_| c.start()).collect())
            .unwrap_or_default();
        Ok(VecRollout {
            worlds: worlds.into_iter().map(Some).collect(),
            spec,
            dof,
            robot: opts.robot,
            control,
            states,
        })
    }

    fn open_many(
        scenes: &[Scene],
        opts: &VecOptions,
        backend: BackendFactory<'_>,
        indices: &[usize],
    ) -> Result<Vec<LiveRollout>, SeqError> {
        #[cfg(feature = "parallel")]
        {
            scenes
                .par_iter()
                .zip(indices.par_iter())
                .map(|(scene, &i)| open_one(scene, opts, backend, i))
                .collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            scenes
                .iter()
                .zip(indices)
                .map(|(scene, &i)| open_one(scene, opts, backend, i))
                .collect()
        }
    }

    /// Replaces the worlds at `indices` with fresh rollouts of `scenes`
    /// (one per index, in order) — the reset of finished episodes.
    pub fn reopen(
        &mut self,
        indices: &[usize],
        scenes: &[Scene],
        opts: &VecOptions,
        backend: BackendFactory<'_>,
    ) -> Result<(), SeqError> {
        if indices.len() != scenes.len() {
            return Err(SeqError::Validation {
                step: None,
                message: format!("{} indices for {} scenes", indices.len(), scenes.len()),
            });
        }
        if let Some(&bad) = indices.iter().find(|&&i| i >= self.worlds.len()) {
            return Err(SeqError::Validation {
                step: None,
                message: format!("no world at index {bad}"),
            });
        }
        let fresh = Self::open_many(scenes, opts, backend, indices)?;
        for (&i, live) in indices.iter().zip(fresh) {
            self.worlds[i] = Some(live);
            if let Some(control) = &self.control {
                self.states[i] = control.start();
            }
        }
        Ok(())
    }

    /// Width of one action (`0` without a control).
    pub fn action_dim(&self) -> usize {
        self.control.as_ref().map(Control::dim).unwrap_or(0)
    }

    /// Turns every world's action (`actions` is `N × action_dim`) into
    /// its joint command through the control, then steps as
    /// [`step_all`](Self::step_all) does. Needs a control.
    pub fn step_actions(
        &mut self,
        k: u32,
        actions: &[f64],
        obs: &mut [f64],
        skip: &[bool],
    ) -> Result<Vec<StepResult>, SeqError> {
        let Some(control) = self.control.as_ref() else {
            return Err(SeqError::Validation {
                step: None,
                message: "step_actions needs a control (open the rollout with one)".into(),
            });
        };
        let (n, dim, adim, robot) = (
            self.worlds.len(),
            self.spec.dim(),
            control.dim(),
            self.robot,
        );
        assert_eq!(actions.len(), n * adim, "actions are N × action_dim");
        assert_eq!(obs.len(), n * dim, "observations are N × dim");
        assert!(skip.is_empty() || skip.len() == n, "skip flags are N");
        let spec = &self.spec;
        let step = |(i, (world, state)): (usize, (&mut Option<LiveRollout>, &mut ControlState)),
                    action: &[f64],
                    out: &mut [f64]|
         -> StepResult {
            let Some(live) = world.as_mut() else {
                out.fill(0.0);
                return StepResult {
                    error: Some("no world (finished or never opened)".into()),
                    ..Default::default()
                };
            };
            let mut error = None;
            let mut ik_failed = false;
            if !skip.get(i).copied().unwrap_or(false) {
                match control.apply(state, live.view(), action) {
                    Ok((cmd, converged)) => {
                        ik_failed = !converged;
                        error = live.command(robot, &cmd).err().map(|e| e.to_string());
                    }
                    Err(e) => error = Some(e),
                }
                if error.is_none() {
                    for _ in 0..k {
                        if let Err(e) = live.tick() {
                            error = Some(e.to_string());
                            break;
                        }
                    }
                }
            }
            if error.is_some() {
                out.fill(0.0);
            } else {
                spec.fill(live.view(), out);
            }
            StepResult {
                t: live.t(),
                finished: live.finished(),
                error,
                collisions: live.collisions().to_vec(),
                contacts: live.contacts().to_vec(),
                ik_failed,
            }
        };
        #[cfg(feature = "parallel")]
        {
            Ok(self
                .worlds
                .par_iter_mut()
                .zip(self.states.par_iter_mut())
                .enumerate()
                .zip(actions.par_chunks(adim.max(1)))
                .zip(obs.par_chunks_mut(dim))
                .map(|((pair, action), out)| step(pair, action, out))
                .collect())
        }
        #[cfg(not(feature = "parallel"))]
        {
            Ok(self
                .worlds
                .iter_mut()
                .zip(self.states.iter_mut())
                .enumerate()
                .zip(actions.chunks(adim.max(1)))
                .zip(obs.chunks_mut(dim))
                .map(|((pair, action), out)| step(pair, action, out))
                .collect())
        }
    }

    pub fn len(&self) -> usize {
        self.worlds.len()
    }

    pub fn is_empty(&self) -> bool {
        self.worlds.is_empty()
    }

    pub fn obs_dim(&self) -> usize {
        self.spec.dim()
    }

    pub fn dof(&self) -> usize {
        self.dof
    }

    pub fn spec(&self) -> &ObsSpec {
        &self.spec
    }

    pub fn world(&self, index: usize) -> Option<&LiveRollout> {
        self.worlds.get(index).and_then(Option::as_ref)
    }

    /// Takes a world out (to finish it into a timeline); the slot is dead
    /// until `reopen`ed.
    pub fn take(&mut self, index: usize) -> Option<LiveRollout> {
        self.worlds.get_mut(index).and_then(Option::take)
    }

    /// Sets the decimation cell of world `index`'s pictures (see
    /// [`LiveRollout::set_render_decimate`]).
    pub fn set_render_decimate(&mut self, index: usize, cell: Option<f64>) -> bool {
        match self.worlds.get_mut(index).and_then(Option::as_mut) {
            Some(live) => {
                live.set_render_decimate(cell);
                true
            }
            None => false,
        }
    }

    /// Sets the light world `index`'s shaded pictures are lit by.
    pub fn set_lighting(&mut self, index: usize, lighting: crate::raster::Lighting) -> bool {
        match self.worlds.get_mut(index).and_then(Option::as_mut) {
            Some(live) => {
                live.set_lighting(lighting);
                true
            }
            None => false,
        }
    }

    /// Commands every world's driven robot (`commands` is `N × dof`),
    /// advances each `k` scan ticks and packs the observations
    /// (`obs` is `N × dim`). A world flagged in `skip` (empty: none) is
    /// only observed — a world just reset reports its first observation
    /// without an action. A dead world reports its error and zeros.
    pub fn step_all(
        &mut self,
        k: u32,
        commands: &[f64],
        obs: &mut [f64],
        skip: &[bool],
    ) -> Vec<StepResult> {
        let (n, dof, dim, robot) = (self.worlds.len(), self.dof, self.spec.dim(), self.robot);
        assert_eq!(commands.len(), n * dof, "commands are N × dof");
        assert_eq!(obs.len(), n * dim, "observations are N × dim");
        assert!(skip.is_empty() || skip.len() == n, "skip flags are N");
        let spec = &self.spec;
        let step = |(i, world): (usize, &mut Option<LiveRollout>),
                    cmd: &[f64],
                    out: &mut [f64]|
         -> StepResult {
            let Some(live) = world.as_mut() else {
                out.fill(0.0);
                return StepResult {
                    error: Some("no world (finished or never opened)".into()),
                    ..Default::default()
                };
            };
            let mut error = None;
            if !skip.get(i).copied().unwrap_or(false) {
                error = live.command(robot, cmd).err().map(|e| e.to_string());
                if error.is_none() {
                    for _ in 0..k {
                        if let Err(e) = live.tick() {
                            error = Some(e.to_string());
                            break;
                        }
                    }
                }
            }
            if error.is_some() {
                out.fill(0.0);
            } else {
                spec.fill(live.view(), out);
            }
            StepResult {
                t: live.t(),
                finished: live.finished(),
                error,
                collisions: live.collisions().to_vec(),
                contacts: live.contacts().to_vec(),
                ik_failed: false,
            }
        };
        #[cfg(feature = "parallel")]
        {
            self.worlds
                .par_iter_mut()
                .enumerate()
                .zip(commands.par_chunks(dof))
                .zip(obs.par_chunks_mut(dim))
                .map(|((world, cmd), out)| step(world, cmd, out))
                .collect()
        }
        #[cfg(not(feature = "parallel"))]
        {
            self.worlds
                .iter_mut()
                .enumerate()
                .zip(commands.chunks(dof))
                .zip(obs.chunks_mut(dim))
                .map(|((world, cmd), out)| step(world, cmd, out))
                .collect()
        }
    }

    /// Packs every world's observation without stepping (after a reset),
    /// worlds in parallel under the `parallel` feature — a LiDAR channel
    /// costs a sweep per world. A dead world reads zero.
    pub fn observe_all(&self, obs: &mut [f64]) {
        let dim = self.spec.dim();
        assert_eq!(
            obs.len(),
            self.worlds.len() * dim,
            "observations are N × dim"
        );
        let spec = &self.spec;
        let fill = |(world, out): (&Option<LiveRollout>, &mut [f64])| match world {
            Some(live) => spec.fill(live.view(), out),
            None => out.fill(0.0),
        };
        #[cfg(feature = "parallel")]
        {
            self.worlds
                .par_iter()
                .zip(obs.par_chunks_mut(dim))
                .for_each(fill);
        }
        #[cfg(not(feature = "parallel"))]
        {
            self.worlds.iter().zip(obs.chunks_mut(dim)).for_each(fill);
        }
    }

    /// The driven robot's joints per world (`N × dof`; zeros for a dead
    /// world).
    pub fn joints_all(&self, out: &mut [f64]) {
        assert_eq!(
            out.len(),
            self.worlds.len() * self.dof,
            "joints are N × dof"
        );
        for (world, out) in self.worlds.iter().zip(out.chunks_mut(self.dof)) {
            match world.as_ref().and_then(|l| l.joint_positions(self.robot)) {
                Some(q) => out.copy_from_slice(q),
                None => out.fill(0.0),
            }
        }
    }

    /// The driven robot's TCP pose per world (`N × 7`; zeros for a dead
    /// world).
    pub fn tcp_all(&self, out: &mut [f64]) {
        assert_eq!(out.len(), self.worlds.len() * 7, "poses are N × 7");
        for (world, out) in self.worlds.iter().zip(out.chunks_mut(7)) {
            let pose = world.as_ref().and_then(|live| {
                let tcp = live.scene().robots()[self.robot].model.default_tcp_link();
                live.link_poses(self.robot).map(|p| p[tcp])
            });
            match pose {
                Some(p) => write_pose(out, &p),
                None => out.fill(0.0),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::{Condition, Sequence, Step};
    use botrail_model::Geometry;
    use nalgebra::Vector3;
    use std::sync::Arc;

    const ARM6: &str = include_str!("../../../examples/assets/simple_arm.urdf");

    fn rapier() -> Option<Box<dyn PhysicsBackend>> {
        Some(Box::new(botrail_physics_rapier::RapierBackend::new()))
    }

    fn cell(part_x: f64) -> Scene {
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(ARM6).unwrap(),
        ));
        scene
            .set_joint_positions(vec![0.0, 0.6, 0.8, 0.0, 0.5, 0.0])
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
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.06, 0.06, 0.06),
                },
                // Beside the arm's ready pose, not under its wrist.
                Isometry3::translation(part_x, 0.25, 0.2),
            )
            .unwrap();
        scene
            .set_obstacle_physics("part", Some(botrail_physics::BodyProps::dynamic()))
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "run".into(),
            steps: vec![Step {
                name: "wait".into(),
                actions: vec![],
                transition: Condition::Elapsed { seconds: 10.0 },
                select: Vec::new(),
            }],
        });
        scene
    }

    fn spec_json() -> &'static str {
        r#"[
            {"kind": "joints", "robot": "simple_arm", "velocities": true},
            {"kind": "tcp_pose", "robot": "simple_arm"},
            {"kind": "link_pose", "robot": "simple_arm", "link": "wrist_3_link"},
            {"kind": "object_pose", "obstacle": "part"},
            {"kind": "object_vel", "obstacle": "part"},
            {"kind": "relative", "a": "simple_arm/tcp", "b": "part"},
            {"kind": "contacts", "a": "part", "b": "table"},
            {"kind": "signal", "signal": "nothing"},
            {"kind": "clearance", "cap": 1.0},
            {"kind": "collision"}
        ]"#
    }

    fn vec_options() -> VecOptions {
        VecOptions {
            names: vec!["run".into()],
            options: RolloutOptions::default(),
            robot: 0,
            group: None,
            max_velocity: None,
            seed: 0,
        }
    }

    #[test]
    fn the_spec_resolves_names_and_packs_in_order() {
        let scene = cell(0.55);
        let spec = ObsSpec::from_json(spec_json(), &scene).unwrap();
        assert_eq!(spec.dims(), vec![12, 7, 7, 7, 6, 7, 2, 1, 1, 1]);
        assert_eq!(spec.dim(), 51);
        let live = scene
            .open_rollout(&["run"], &RolloutOptions::default(), rapier())
            .unwrap();
        let mut out = vec![f64::NAN; spec.dim()];
        spec.fill(live.view(), &mut out);
        assert!(out.iter().all(|v| v.is_finite()));
        // Joints: the ready pose, at rest.
        assert_eq!(&out[..6], &[0.0, 0.6, 0.8, 0.0, 0.5, 0.0]);
        assert!(out[6..12].iter().all(|v| *v == 0.0));
        // The relative block is tcp⁻¹ ∘ part: recomputed here by hand.
        let tcp = live.link_poses(0).unwrap()[scene.robot().default_tcp_link()];
        let rel = tcp.inverse() * live.obstacle_pose("part").unwrap();
        assert_eq!(out[39], rel.translation.x);
        assert_eq!(out[42], rel.rotation.coords.x);
        // Untouched, unsignalled, clear, not colliding.
        assert_eq!(&out[46..51], &[0.0, 0.0, 0.0, out[49], 0.0]);
        assert!(out[49] > 0.0 && out[49] <= 1.0);
        for bad in [
            r#"[{"kind": "joints", "robot": "nope", "velocities": true}]"#,
            r#"[{"kind": "object_pose", "obstacle": "nope"}]"#,
            r#"[{"kind": "relative", "a": "simple_arm/nope", "b": "part"}]"#,
        ] {
            assert!(ObsSpec::from_json(bad, &scene).is_err(), "{bad}");
        }
    }

    #[test]
    fn stepping_worlds_together_matches_stepping_each_alone() {
        let scenes = vec![cell(0.55), cell(0.50), cell(0.60)];
        let spec = ObsSpec::from_json(spec_json(), &scenes[0]).unwrap();
        let opts = vec_options();
        let mut vec = VecRollout::open(&scenes, &opts, &rapier, spec.clone(), None).unwrap();
        assert_eq!(vec.len(), 3);
        assert_eq!(vec.dof(), 6);
        // Each world alone, driven the same way.
        let mut alone: Vec<LiveRollout> = scenes
            .iter()
            .enumerate()
            .map(|(i, s)| open_one(s, &opts, &rapier, i).unwrap())
            .collect();
        let mut obs = vec![0.0; 3 * spec.dim()];
        let mut single = vec![0.0; spec.dim()];
        let mut cmds = vec![0.0; 3 * 6];
        for step in 0..30 {
            for w in 0..3 {
                let q = &mut cmds[w * 6..(w + 1) * 6];
                q.copy_from_slice(&[0.0, 0.6, 0.8, 0.0, 0.5, 0.0]);
                q[0] = 0.02 * step as f64 * (w as f64 + 1.0);
                q[1] = 0.6 + 0.01 * step as f64;
            }
            let results = vec.step_all(5, &cmds, &mut obs, &[]);
            assert!(results.iter().all(|r| r.error.is_none()), "{results:?}");
            for (w, live) in alone.iter_mut().enumerate() {
                live.command(0, &cmds[w * 6..(w + 1) * 6]).unwrap();
                for _ in 0..5 {
                    live.tick().unwrap();
                }
                spec.fill(live.view(), &mut single);
                assert_eq!(
                    &obs[w * spec.dim()..(w + 1) * spec.dim()],
                    &single[..],
                    "world {w} step {step}"
                );
                assert_eq!(results[w].t, live.t());
            }
        }
        // The worlds diverged from each other (different parts), and the
        // physics ran: the part is on the table in every one.
        // Layout: joints 0..12, tcp 12..19, wrist 19..26, part 26..33.
        let dim = spec.dim();
        assert_ne!(&obs[26..33], &obs[dim + 26..dim + 33]);
        for w in 0..3 {
            let z = obs[w * dim + 28];
            assert!((z - 0.13).abs() < 0.01, "world {w} part z = {z}");
        }
        let mut joints = vec![0.0; 18];
        vec.joints_all(&mut joints);
        assert_eq!(
            &joints[..6],
            vec.world(0).unwrap().joint_positions(0).unwrap()
        );
        let mut tcps = vec![0.0; 21];
        vec.tcp_all(&mut tcps);
        assert_eq!(&tcps[..7], &obs[12..19]);
    }

    #[test]
    fn a_tcp_delta_control_integrates_its_setpoint_in_rust() {
        let scene = cell(0.55);
        let options = RolloutOptions::default();
        let mut live = scene.open_rollout(&["run"], &options, None).unwrap();
        live.drive(0, None, None).unwrap();
        let dim = live
            .set_control(
                r#"{"kind": "tcp_delta", "max_step_m": 0.02, "max_step_rad": 0.05, "rotate": false, "frame": "world"}"#,
                0,
                None,
            )
            .unwrap();
        assert_eq!(dim, 3);
        let tip = scene.robot().default_tcp_link();
        let start = live.link_poses(0).unwrap()[tip];
        // Two world +x steps, then settle: the setpoint moved 4 cm and
        // the orientation is the one the run started with.
        for _ in 0..2 {
            assert!(live.act(&[1.0, 0.0, 0.0]).unwrap());
            for _ in 0..5 {
                live.tick().unwrap();
            }
        }
        for _ in 0..30 {
            live.tick().unwrap();
        }
        let now = live.link_poses(0).unwrap()[tip];
        assert!(
            (now.translation.x - start.translation.x - 0.04).abs() < 2e-3,
            "{now:?}"
        );
        assert!(
            (now.translation.y - start.translation.y).abs() < 2e-3,
            "{now:?}"
        );
        assert!(now.rotation.angle_to(&start.rotation) < 1e-4);
        // A gripper entry maps its action onto the joint's limits after the IK.
        assert_eq!(
            live.set_control(
                r#"{"kind": "tcp_delta", "max_step_m": 0.0, "max_step_rad": 0.05, "rotate": false, "frame": "world", "gripper": [[5, -1.0, 1.0]]}"#,
                0,
                None,
            )
            .unwrap(),
            4
        );
        assert!(live.act(&[0.0, 0.0, 0.0, 1.0]).unwrap());
        for _ in 0..40 {
            live.tick().unwrap();
        }
        assert!((live.joint_positions(0).unwrap()[5] - 1.0).abs() < 1e-9);
        // A step the IK cannot solve is dropped: `false`, and the joints hold.
        let before = live.joint_positions(0).unwrap().to_vec();
        live.set_control(
            r#"{"kind": "tcp_delta", "max_step_m": 5.0, "max_step_rad": 0.05, "rotate": false, "frame": "world"}"#,
            0,
            None,
        )
        .unwrap();
        assert!(!live.act(&[0.0, 0.0, 1.0]).unwrap());
        live.tick().unwrap();
        assert_eq!(live.joint_positions(0).unwrap(), &before[..]);
        // The joint controls: a delta and a target onto the limits.
        live.set_control(
            r#"{"kind": "joint_delta", "indices": [0], "max_step": 0.05}"#,
            0,
            None,
        )
        .unwrap();
        live.act(&[1.0]).unwrap();
        for _ in 0..5 {
            live.tick().unwrap();
        }
        assert!((live.joint_positions(0).unwrap()[0] - before[0] - 0.05).abs() < 1e-9);
        live.set_control(
            r#"{"kind": "joint_target", "indices": [1], "limits": [[-2.2, 2.2]]}"#,
            0,
            None,
        )
        .unwrap();
        live.act(&[-1.0]).unwrap();
        for _ in 0..300 {
            live.tick().unwrap();
        }
        assert!((live.joint_positions(0).unwrap()[1] + 2.2).abs() < 1e-9);
        // Bad specs are refused up front.
        assert!(live
            .set_control(
                r#"{"kind": "joint_delta", "indices": [9], "max_step": 0.05}"#,
                0,
                None
            )
            .is_err());
        assert!(live.set_control(r#"{"kind": "tcp_delta", "max_step_m": 0.02, "max_step_rad": 0.05, "rotate": false, "frame": "tool"}"#, 0, None).is_err());
        assert!(live.act(&[0.0, 0.0]).is_err());
    }

    #[test]
    fn a_lidar_channel_sweeps_the_live_world_with_seeded_noise() {
        use crate::seq::{Lidar, LidarMount};
        let mut scene = cell(0.55);
        scene
            .upsert_lidar(Lidar {
                name: "front".into(),
                mount: LidarMount::World,
                pose: Isometry3::translation(0.0, -0.6, 0.3),
                fov_deg: 270.0,
                range: [0.05, 8.0],
                resolution_deg: 0.5,
                channels: 4,
                vfov_deg: 10.0,
            })
            .unwrap();
        // Widths follow the thinning: 541 azimuths × rings.
        let spec = ObsSpec::from_json(
            r#"[{"kind": "lidar", "lidar": "front"},
                {"kind": "lidar", "lidar": "front", "stride": 4, "rings": [0, 3]},
                {"kind": "lidar", "lidar": "front", "stride": 1, "rings": [2], "noise": 0.01}]"#,
            &scene,
        )
        .unwrap();
        assert_eq!(spec.dims(), vec![541 * 4, 136 * 2, 541]);
        for bad in [
            r#"[{"kind": "lidar", "lidar": "nope"}]"#,
            r#"[{"kind": "lidar", "lidar": "front", "rings": [4]}]"#,
            r#"[{"kind": "lidar", "lidar": "front", "stride": 0}]"#,
        ] {
            assert!(ObsSpec::from_json(bad, &scene).is_err(), "{bad}");
        }
        let options = RolloutOptions::default();
        let live = scene.open_rollout(&["run"], &options, rapier()).unwrap();
        let mut out = vec![0.0; spec.dim()];
        spec.fill(live.view(), &mut out);
        // The full, noiseless sweep at t = 0 is the scan API's, misses
        // reading the max range instead of 0.
        let batch = crate::scan::lidar_scan(&scene, "front", None).unwrap();
        assert_eq!(batch.ranges.len(), 541 * 4);
        for (i, r) in batch.ranges.iter().enumerate() {
            assert_eq!(out[i], if *r > 0.0 { *r } else { 8.0 }, "beam {i}");
        }
        assert!(
            batch.hits.iter().any(Option::is_some),
            "the table is in view"
        );
        // The thinned block is the corresponding subset of the full grid
        // (ring 0 and ring 3, every 4th azimuth).
        let at = 541 * 4;
        for (k, i) in (0..541).step_by(4).enumerate() {
            assert_eq!(out[at + k], out[i]);
            assert_eq!(out[at + 136 + k], out[541 * 3 + i]);
        }
        // Noise: a hit differs from its clean reading, misses stay at the
        // max range; the stream depends on the seed and on the tick.
        let noisy = &out[at + 272..];
        let clean_ring2: Vec<f64> = out[541 * 2..541 * 3].to_vec();
        let mut differs = 0;
        for (n, c) in noisy.iter().zip(&clean_ring2) {
            if *c >= 8.0 {
                assert_eq!(*n, 8.0);
            } else if n != c {
                differs += 1;
            }
        }
        assert!(differs > 0, "noise never drew");
        let mut again = vec![0.0; spec.dim()];
        spec.fill(live.view(), &mut again);
        assert_eq!(out, again, "same seed, same tick: same draw");
        let mut other = scene.open_rollout(&["run"], &options, rapier()).unwrap();
        other.set_noise_seed(7);
        let mut seeded = vec![0.0; spec.dim()];
        spec.fill(other.view(), &mut seeded);
        assert_eq!(
            &seeded[..at + 272],
            &out[..at + 272],
            "noiseless blocks agree"
        );
        assert_ne!(
            &seeded[at + 272..],
            &out[at + 272..],
            "another seed, another draw"
        );
        other.tick().unwrap();
        let mut later = vec![0.0; spec.dim()];
        spec.fill(other.view(), &mut later);
        assert_ne!(
            &later[at + 272..],
            &seeded[at + 272..],
            "another tick, another draw"
        );
    }

    #[test]
    fn a_vehicle_mounted_lidar_rides_the_vehicle_mid_drive() {
        use crate::seq::{
            Action, Condition, Device, DeviceCommand, DeviceKind, Drive, Lidar, LidarMount,
            Sequence, Step, VehiclePath,
        };
        let mut scene = Scene::empty();
        scene
            .add_obstacle(
                "wall",
                Geometry::Box {
                    size: Vector3::new(0.2, 4.0, 1.0),
                },
                Isometry3::translation(5.0, 0.0, 0.5),
            )
            .unwrap();
        scene
            .add_obstacle(
                "agv",
                Geometry::Box {
                    size: Vector3::new(0.8, 0.5, 0.3),
                },
                Isometry3::translation(0.0, 0.0, 0.15),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "cart".into(),
            kind: DeviceKind::Vehicle {
                wheels: Vec::new(),
                path: VehiclePath {
                    waypoints: vec![
                        nalgebra::Point3::new(0.0, 0.0, 0.0),
                        nalgebra::Point3::new(2.0, 0.0, 0.0),
                    ],
                    stations: vec![("a".into(), 0), ("b".into(), 1)],
                    ring: false,
                },
                body: vec!["agv".into()],
                speed: 1.0,
                turn_speed: 1.0,
                start: "a".into(),
                drive: Drive::default(),
                tray: None,
            },
        });
        scene
            .upsert_lidar(Lidar {
                name: "nose".into(),
                mount: LidarMount::Vehicle {
                    device: "cart".into(),
                },
                pose: Isometry3::translation(0.4, 0.0, 0.2),
                fov_deg: 10.0,
                range: [0.05, 10.0],
                resolution_deg: 5.0,
                channels: 1,
                vfov_deg: 0.0,
            })
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "go".into(),
            steps: vec![Step {
                name: "drive".into(),
                actions: vec![Action::Device {
                    device: "cart".into(),
                    command: DeviceCommand::Goto {
                        station: "b".into(),
                    },
                }],
                transition: Condition::Elapsed { seconds: 5.0 },
                select: Vec::new(),
            }],
        });
        let spec = ObsSpec::from_json(r#"[{"kind": "lidar", "lidar": "nose"}]"#, &scene).unwrap();
        assert_eq!(spec.dim(), 3);
        let options = RolloutOptions::default();
        let mut live = scene.open_rollout(&["go"], &options, None).unwrap();
        let mut out = vec![0.0; 3];
        spec.fill(live.view(), &mut out);
        // Parked at x = 0, nose at 0.4: the wall face at 4.9 is 4.5 m off.
        assert!((out[1] - 4.5).abs() < 1e-6, "{out:?}");
        for _ in 0..100 {
            live.tick().unwrap();
        }
        // A second in at 1 m/s (whatever the ramp): the reading shrank
        // with the drive — the scanner rode the vehicle, not its parking.
        spec.fill(live.view(), &mut out);
        assert!(out[1] < 4.0, "{out:?}");
        assert!(live.view().vehicle_frame("cart").unwrap().translation.x > 0.3);
        assert!(live.view().vehicle_frame("nothing").is_none());
    }

    #[test]
    fn a_taken_world_is_dead_until_reopened() {
        let scenes = vec![cell(0.55), cell(0.55)];
        let spec = ObsSpec::from_json(spec_json(), &scenes[0]).unwrap();
        let opts = vec_options();
        let mut vec = VecRollout::open(&scenes, &opts, &rapier, spec.clone(), None).unwrap();
        let mut obs = vec![0.0; 2 * spec.dim()];
        let cmds = vec![0.0, 0.6, 0.8, 0.0, 0.5, 0.0, 0.0, 0.6, 0.8, 0.0, 0.5, 0.0];
        vec.step_all(5, &cmds, &mut obs, &[]);
        let tl = vec.take(1).unwrap().finish();
        assert!((tl.duration - 0.05).abs() < 1e-12);
        let results = vec.step_all(5, &cmds, &mut obs, &[]);
        assert!(results[0].error.is_none());
        assert!(results[1].error.as_deref().unwrap().contains("no world"));
        assert!(obs[spec.dim()..].iter().all(|v| *v == 0.0));
        vec.reopen(&[1], &scenes[1..], &opts, &rapier).unwrap();
        let results = vec.step_all(5, &cmds, &mut obs, &[]);
        assert!(results[1].error.is_none());
        assert_eq!(results[1].t, 0.05);
        assert!(vec.reopen(&[5], &scenes[1..], &opts, &rapier).is_err());
        // A skipped world is observed, not advanced.
        let t_before = vec.world(0).unwrap().t();
        let results = vec.step_all(5, &cmds, &mut obs, &[true, false]);
        assert_eq!(results[0].t, t_before);
        assert_eq!(results[1].t, 0.10);
        assert!(obs[..spec.dim()].iter().any(|v| *v != 0.0));
        // A bake error kills the world and says so.
        let mut short = vec_options();
        short.options.max_duration = 0.02;
        let mut vec = VecRollout::open(&scenes[..1], &short, &rapier, spec.clone(), None).unwrap();
        let results = vec.step_all(5, &cmds[..6], &mut obs[..spec.dim()], &[]);
        assert!(results[0].error.as_deref().unwrap().contains("timed out"));
    }
}
