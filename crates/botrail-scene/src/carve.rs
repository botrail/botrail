//! Stock carving: voxel subtraction of the cutter's swept volume from a
//! stock obstacle, walked over a baked timeline.
//!
//! This is *presentation and bookkeeping*, not verification (see
//! `design/design-machining.md` §3.3 S4 / §5): in a kinematic world the TCP
//! follows the toolpath exactly, so the carve can never contradict the
//! plan — what it adds is the picture (the machined part as a mesh) and
//! the numbers (removed / remaining volume). Deterministic by
//! construction: a uniform grid in the stock's local frame, initialized
//! by exact containment against the stock's own solid collision shapes,
//! then swept with the cutter cylinder at sub-voxel steps.
//!
//! parry ships everything volumetric this could want; only point
//! containment is needed here, so the grid and the surface extraction
//! stay first-party and dependency-free.

use nalgebra::{Isometry3, Point3, Vector3};
use thiserror::Error;

use crate::rollout::SequenceTimeline;
use crate::Scene;

#[derive(Debug, Error)]
pub enum CarveError {
    #[error("unknown obstacle `{0}`")]
    UnknownStock(String),
    #[error("unknown robot `{0}`")]
    UnknownRobot(String),
    #[error("stock `{0}` has no collision geometry to voxelize")]
    NoGeometry(String),
    #[error(
        "voxel size {voxel} m over the stock needs {cells} cells (cap {cap}); \
         use a coarser voxel"
    )]
    TooFine {
        voxel: f64,
        cells: usize,
        cap: usize,
    },
    #[error("voxel size must be positive, got {0}")]
    BadVoxel(f64),
    #[error("cutter radius/length must be positive")]
    BadCutter,
}

#[derive(Debug, Clone)]
pub struct CarveOptions {
    /// Grid edge length (m). The cut geometry quantizes to this.
    pub voxel_size: f64,
    /// Cutter (flute) radius (m).
    pub cutter_radius: f64,
    /// Cutting length from the tip along the tool axis (m).
    pub cutter_length: f64,
    /// Timeline sampling period (s); sub-stepped further so the TCP never
    /// moves more than half a voxel between carve stamps.
    pub dt: f64,
    /// Face color of the surviving original skin (linear RGB).
    pub stock_color: [f32; 3],
    /// Face color of surfaces the cutter created (linear RGB) — the
    /// bright machined finish that makes the removal readable at a
    /// glance.
    pub cut_color: [f32; 3],
}

impl Default for CarveOptions {
    fn default() -> Self {
        CarveOptions {
            voxel_size: 0.001,
            cutter_radius: 0.004,
            cutter_length: 0.03,
            dt: 0.01,
            stock_color: [0.58, 0.60, 0.63],
            cut_color: [0.92, 0.94, 0.98],
        }
    }
}

/// The carved stock: a boundary mesh in the stock's local frame plus the
/// volume bookkeeping.
#[derive(Debug, Clone)]
pub struct StockCarve {
    /// Boundary mesh of the remaining material, stock-local coordinates
    /// (place it at the stock's pose).
    pub mesh: botrail_mesh::MeshData,
    /// World pose of the stock at carve time.
    pub pose: Isometry3<f64>,
    pub voxel_size: f64,
    pub initial_volume: f64,
    pub removed_volume: f64,
    pub remaining_volume: f64,
}

struct Grid {
    origin: Vector3<f64>,
    voxel: f64,
    nx: usize,
    ny: usize,
    nz: usize,
    filled: Vec<bool>,
}

impl Grid {
    fn index(&self, x: usize, y: usize, z: usize) -> usize {
        (z * self.ny + y) * self.nx + x
    }

    fn at(&self, x: isize, y: isize, z: isize) -> bool {
        if x < 0 || y < 0 || z < 0 {
            return false;
        }
        let (x, y, z) = (x as usize, y as usize, z as usize);
        if x >= self.nx || y >= self.ny || z >= self.nz {
            return false;
        }
        self.filled[self.index(x, y, z)]
    }

    fn center(&self, x: usize, y: usize, z: usize) -> Vector3<f64> {
        self.origin
            + Vector3::new(
                (x as f64 + 0.5) * self.voxel,
                (y as f64 + 0.5) * self.voxel,
                (z as f64 + 0.5) * self.voxel,
            )
    }
}

/// One frame of a progressive carve: the remaining stock as of `time`.
/// Snapshots are only taken where the sweep actually removed material
/// since the previous one, so idle stretches produce no duplicate
/// meshes.
#[derive(Debug, Clone)]
pub struct CarveStage {
    /// Timeline time this state is current from.
    pub time: f64,
    pub mesh: botrail_mesh::MeshData,
    /// Cumulative removed volume as of this stage (m³).
    pub removed_volume: f64,
}

/// Carves `stock` with the cutter swept along `timeline`'s track of
/// `robot`. `scene` must be the pre-rollout snapshot the timeline was
/// baked against (same contract as `timeline_min_clearance`).
pub fn carve_stock(
    scene: &Scene,
    timeline: &SequenceTimeline,
    stock: &str,
    robot: usize,
    tcp: usize,
    options: &CarveOptions,
) -> Result<StockCarve, CarveError> {
    carve_stock_staged(scene, timeline, stock, robot, tcp, options, 1).map(|(carve, _)| carve)
}

/// [`carve_stock`] plus intermediate snapshots at `stages` equal time
/// boundaries — the raw material of progressive-removal display (each
/// stage shown for its window via [`staged_timeline`]).
#[allow(clippy::too_many_arguments)]
pub fn carve_stock_staged(
    scene: &Scene,
    timeline: &SequenceTimeline,
    stock: &str,
    robot: usize,
    tcp: usize,
    options: &CarveOptions,
    stages: usize,
) -> Result<(StockCarve, Vec<CarveStage>), CarveError> {
    if !(options.voxel_size.is_finite() && options.voxel_size > 0.0) {
        return Err(CarveError::BadVoxel(options.voxel_size));
    }
    if options.cutter_radius <= 0.0 || options.cutter_length <= 0.0 {
        return Err(CarveError::BadCutter);
    }
    let (obstacle, collider) = scene
        .obstacle_with_collider(stock)
        .ok_or_else(|| CarveError::UnknownStock(stock.to_string()))?;
    let (mins, maxs) = collider
        .aabb(&Isometry3::identity())
        .ok_or_else(|| CarveError::NoGeometry(stock.to_string()))?;
    let vox = options.voxel_size;
    let nx = (((maxs[0] - mins[0]) / vox).ceil() as usize).max(1);
    let ny = (((maxs[1] - mins[1]) / vox).ceil() as usize).max(1);
    let nz = (((maxs[2] - mins[2]) / vox).ceil() as usize).max(1);
    const CELL_CAP: usize = 50_000_000;
    let cells = nx * ny * nz;
    if cells > CELL_CAP {
        return Err(CarveError::TooFine {
            voxel: vox,
            cells,
            cap: CELL_CAP,
        });
    }

    let mut grid = Grid {
        origin: Vector3::new(mins[0], mins[1], mins[2]),
        voxel: vox,
        nx,
        ny,
        nz,
        filled: vec![false; cells],
    };
    let mut initial = 0usize;
    for z in 0..nz {
        for y in 0..ny {
            for x in 0..nx {
                let c = grid.center(x, y, z);
                if collider.contains_local_point(&Point3::from(c)) {
                    let i = grid.index(x, y, z);
                    grid.filled[i] = true;
                    initial += 1;
                }
            }
        }
    }

    // The pre-cut occupancy: a boundary face whose empty neighbor *was*
    // filled is a surface the cutter made — that distinction is the
    // machined-finish coloring.
    let was_filled = grid.filled.clone();

    // Sweep the cutter along the baked track: world TCP poses at `dt`,
    // sub-stepped so consecutive stamps sit within half a voxel.
    let to_local = obstacle.pose.inverse();
    let track = &timeline.robots[robot].trajectory;
    let stamp = |grid: &mut Grid, tip: &Vector3<f64>, axis: &Vector3<f64>| -> bool {
        let r = options.cutter_radius;
        let len = options.cutter_length;
        // Cell range from the cutter's local AABB (tip to tip + len*axis,
        // inflated by the radius).
        let end = tip + axis * len;
        let lo = tip.inf(&end) - Vector3::repeat(r);
        let hi = tip.sup(&end) + Vector3::repeat(r);
        let cell = |v: f64, o: f64| ((v - o) / vox).floor() as isize;
        let x0 = cell(lo.x, grid.origin.x).max(0) as usize;
        let y0 = cell(lo.y, grid.origin.y).max(0) as usize;
        let z0 = cell(lo.z, grid.origin.z).max(0) as usize;
        let x1 = (cell(hi.x, grid.origin.x) + 1).clamp(0, grid.nx as isize) as usize;
        let y1 = (cell(hi.y, grid.origin.y) + 1).clamp(0, grid.ny as isize) as usize;
        let z1 = (cell(hi.z, grid.origin.z) + 1).clamp(0, grid.nz as isize) as usize;
        let mut changed = false;
        for z in z0..z1 {
            for y in y0..y1 {
                for x in x0..x1 {
                    let i = grid.index(x, y, z);
                    if !grid.filled[i] {
                        continue;
                    }
                    let c = grid.center(x, y, z);
                    // A flat end mill: strictly the cylinder between the
                    // tip plane and the flute length — clamping here would
                    // grow a spherical cap below the tip.
                    let along = (c - tip).dot(axis);
                    if !(0.0..=len).contains(&along) {
                        continue;
                    }
                    let radial = (c - (tip + axis * along)).norm();
                    if radial <= r {
                        grid.filled[i] = false;
                        changed = true;
                    }
                }
            }
        }
        changed
    };

    let tcp_pose = |q: &[f64]| -> Option<(Vector3<f64>, Vector3<f64>)> {
        let poses = scene.fk_for(robot, q).ok()?;
        let world = poses[tcp];
        let local = to_local * world;
        Some((local.translation.vector, local.rotation * Vector3::z()))
    };

    let stages = stages.max(1);
    let vol = vox * vox * vox;
    let mut snapshots: Vec<CarveStage> = Vec::new();
    let mut last_count = initial;
    let mut t = 0.0;
    let mut prev = tcp_pose(&track.sample(0.0));
    for k in 1..=stages {
        let boundary = timeline.duration * k as f64 / stages as f64;
        let mut change_t = None;
        while t < boundary - 1e-9 {
            let next_t = (t + options.dt).min(boundary);
            let next = tcp_pose(&track.sample(next_t));
            if let (Some((p0, a0)), Some((p1, a1))) = (&prev, &next) {
                let dist = (p1 - p0).norm();
                let steps = ((dist / (vox * 0.5)).ceil() as usize).max(1);
                for s in 0..=steps {
                    let u = s as f64 / steps as f64;
                    let tip = p0.lerp(p1, u);
                    let axis = (a0.lerp(a1, u)).normalize();
                    if stamp(&mut grid, &tip, &axis) {
                        change_t = Some(next_t);
                    }
                }
            }
            prev = next;
            t = next_t;
        }
        // Snapshot only where the sweep changed something: idle windows
        // extend the previous stage instead of duplicating its mesh. The
        // snapshot is stamped with the *last change* time, not the window
        // boundary — the state is identical at both (nothing moved in
        // between), but this keeps the display exact from the moment it
        // switches, shows the pristine stock until the first real cut,
        // and gives the final stage a nonzero window even when cutting
        // runs into the last grid cell.
        let count = grid.filled.iter().filter(|f| **f).count();
        if count != last_count {
            snapshots.push(CarveStage {
                time: change_t.unwrap_or(boundary),
                mesh: boundary_mesh(&grid, &was_filled, options.stock_color, options.cut_color),
                removed_volume: (initial - count) as f64 * vol,
            });
            last_count = count;
        }
    }

    let remaining = last_count;
    let mesh = match snapshots.last() {
        Some(stage) => stage.mesh.clone(),
        None => boundary_mesh(&grid, &was_filled, options.stock_color, options.cut_color),
    };
    Ok((
        StockCarve {
            mesh,
            pose: obstacle.pose,
            voxel_size: vox,
            initial_volume: initial as f64 * vol,
            removed_volume: (initial - remaining) as f64 * vol,
            remaining_volume: remaining as f64 * vol,
        },
        snapshots,
    ))
}

/// The timeline with progressive-removal visibility injected: the stock
/// obstacle shows until the first stage's time, then each stage obstacle
/// shows for its window (stowed outside it). `stage_names[i]` must be an
/// obstacle placed at `pose` whose geometry is `stages[i]`'s mesh; the
/// caller registers those. Presentation only — nothing else about the
/// timeline changes.
pub fn staged_timeline(
    timeline: &SequenceTimeline,
    stock: &str,
    pose: Isometry3<f64>,
    stage_names: &[String],
    stage_times: &[f64],
) -> SequenceTimeline {
    use crate::rollout::{ObjectTrack, TrackSpan};
    let mut out = timeline.clone();
    let end = out.duration;
    let first = stage_times.first().copied().unwrap_or(end);
    // The pristine stock, until the first cut state takes over.
    out.objects.retain(|track| track.name != stock);
    let mut stock_spans = vec![TrackSpan::Hold {
        t0: 0.0,
        t1: first,
        pose,
    }];
    if first < end {
        stock_spans.push(TrackSpan::Stowed {
            t0: first,
            t1: end,
            pose,
        });
    }
    out.objects.push(ObjectTrack {
        name: stock.to_string(),
        spans: stock_spans,
    });
    for (i, name) in stage_names.iter().enumerate() {
        let from = stage_times[i];
        let to = stage_times.get(i + 1).copied().unwrap_or(end);
        let mut spans = Vec::new();
        if from > 0.0 {
            spans.push(TrackSpan::Stowed {
                t0: 0.0,
                t1: from,
                pose,
            });
        }
        spans.push(TrackSpan::Hold {
            t0: from,
            t1: to,
            pose,
        });
        if to < end {
            spans.push(TrackSpan::Stowed {
                t0: to,
                t1: end,
                pose,
            });
        }
        out.objects.push(ObjectTrack {
            name: name.clone(),
            spans,
        });
    }
    out
}

/// Boundary surface of the filled cells, greedy-meshed: exposed faces on
/// each axis/direction merge into maximal rectangles first (the flat top
/// of a plate becomes a handful of quads instead of tens of thousands),
/// then emit two triangles each, outward-wound. Faces are classed by what
/// exposed them — the surviving original skin keeps `stock_color`, a face
/// whose empty neighbor *was* filled is a surface the cutter made and
/// gets `cut_color`; rectangles never merge across the two.
fn boundary_mesh(
    grid: &Grid,
    was_filled: &[bool],
    stock_color: [f32; 3],
    cut_color: [f32; 3],
) -> botrail_mesh::MeshData {
    let mut vertices: Vec<[f64; 3]> = Vec::new();
    let mut indices: Vec<[u32; 3]> = Vec::new();
    let mut face_colors: Vec<[f32; 3]> = Vec::new();
    let mut vertex = |p: Vector3<f64>| -> u32 {
        vertices.push([p.x, p.y, p.z]);
        (vertices.len() - 1) as u32
    };

    const NONE: u8 = 0;
    const SKIN: u8 = 1;
    const CUT: u8 = 2;

    // For each axis `d`, sweep the layers perpendicular to it; `u`/`v`
    // span the layer. A face exists where a filled cell meets an empty
    // neighbor along `d` (either direction, handled by `sign`).
    for d in 0..3 {
        let (u, v) = ((d + 1) % 3, (d + 2) % 3);
        let dims = [grid.nx, grid.ny, grid.nz];
        let (nd, nu, nv) = (dims[d], dims[u], dims[v]);
        for sign in [-1isize, 1isize] {
            for layer in 0..nd {
                // Class mask of exposed faces in this layer.
                let mut mask = vec![NONE; nu * nv];
                for j in 0..nv {
                    for i in 0..nu {
                        let mut cell = [0isize; 3];
                        cell[d] = layer as isize;
                        cell[u] = i as isize;
                        cell[v] = j as isize;
                        if !grid.at(cell[0], cell[1], cell[2]) {
                            continue;
                        }
                        let mut neighbor = cell;
                        neighbor[d] += sign;
                        if grid.at(neighbor[0], neighbor[1], neighbor[2]) {
                            continue;
                        }
                        // The neighbor is empty now; was it stock before
                        // the sweep? In-bounds check mirrors `Grid::at`.
                        let removed = neighbor[0] >= 0
                            && neighbor[1] >= 0
                            && neighbor[2] >= 0
                            && (neighbor[0] as usize) < grid.nx
                            && (neighbor[1] as usize) < grid.ny
                            && (neighbor[2] as usize) < grid.nz
                            && was_filled[grid.index(
                                neighbor[0] as usize,
                                neighbor[1] as usize,
                                neighbor[2] as usize,
                            )];
                        mask[j * nu + i] = if removed { CUT } else { SKIN };
                    }
                }
                // Greedy rectangles over the mask, one class at a time.
                let mut used = vec![false; nu * nv];
                for j in 0..nv {
                    for i in 0..nu {
                        let class = mask[j * nu + i];
                        if class == NONE || used[j * nu + i] {
                            continue;
                        }
                        let mut w = 1;
                        while i + w < nu && mask[j * nu + i + w] == class && !used[j * nu + i + w] {
                            w += 1;
                        }
                        let mut h = 1;
                        'grow: while j + h < nv {
                            for k in 0..w {
                                let at = (j + h) * nu + i + k;
                                if mask[at] != class || used[at] {
                                    break 'grow;
                                }
                            }
                            h += 1;
                        }
                        for jj in j..j + h {
                            for ii in i..i + w {
                                used[jj * nu + ii] = true;
                            }
                        }
                        // Rectangle corners in grid coordinates: the face
                        // plane sits at `layer` (+1 when facing +d).
                        let plane = if sign > 0 { layer + 1 } else { layer } as f64;
                        let corner = |du: usize, dv: usize| -> Vector3<f64> {
                            let mut g = [0.0f64; 3];
                            g[d] = plane;
                            g[u] = (i + du) as f64;
                            g[v] = (j + dv) as f64;
                            grid.origin + Vector3::new(g[0], g[1], g[2]) * grid.voxel
                        };
                        let a = vertex(corner(0, 0));
                        let b = vertex(corner(w, 0));
                        let c = vertex(corner(w, h));
                        let e = vertex(corner(0, h));
                        // Outward winding: for +d faces (u, v, d) is a
                        // right-handed frame, so a->b->c faces +d; -d
                        // faces flip.
                        if sign > 0 {
                            indices.push([a, b, c]);
                            indices.push([a, c, e]);
                        } else {
                            indices.push([a, c, b]);
                            indices.push([a, e, c]);
                        }
                        let color = if class == CUT { cut_color } else { stock_color };
                        face_colors.push(color);
                        face_colors.push(color);
                    }
                }
            }
        }
    }
    botrail_mesh::MeshData {
        vertices,
        indices,
        face_colors,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rollout::SequenceTimeline;
    use botrail_model::{Geometry, RobotModel};
    use botrail_traj::JointTrajectory;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/simple_arm.urdf");
    const SPINDLE: &str = include_str!("../../../examples/assets/spindle.urdf");

    /// Arm + spindle, so the TCP is the cutter tip with local `+Z`
    /// pointing tip-toward-body (the machining axis convention).
    fn machining_robot() -> RobotModel {
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let spindle = RobotModel::from_urdf_str(SPINDLE).unwrap();
        arm.attach_tool(
            &spindle,
            Some("tool0"),
            None,
            Isometry3::identity(),
            None,
            None,
        )
        .unwrap()
    }

    /// A hand-built timeline holding one configuration: the carve then
    /// stamps the cutter at a single fixed pose.
    fn hold_timeline(scene: &Scene, q: Vec<f64>, duration: f64) -> SequenceTimeline {
        scene.timeline_from_trajectory(
            0,
            &JointTrajectory {
                times: vec![0.0, duration],
                positions: vec![q.clone(), q],
                velocities: vec![vec![0.0; 6], vec![0.0; 6]],
            },
            "hold",
        )
    }

    #[test]
    fn a_parked_cutter_bores_one_round_hole() {
        let mut scene = Scene::new(Arc::new(machining_robot()));
        // Flange-down pose; the plate top sits 2 mm above the tip, so the
        // parked cutter is buried 2 mm deep in the stock.
        let q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        scene
            .add_obstacle(
                "plate",
                Geometry::Box {
                    size: Vector3::new(0.06, 0.06, 0.01),
                },
                Isometry3::translation(tip.x, tip.y, tip.z + 0.002 - 0.005),
            )
            .unwrap();
        let timeline = hold_timeline(&scene, q, 1.0);
        let carve = carve_stock(
            &scene,
            &timeline,
            "plate",
            0,
            tcp,
            &CarveOptions {
                voxel_size: 0.0005,
                ..CarveOptions::default()
            },
        )
        .unwrap();
        // Expected removal: a cylinder r=4mm, depth 2mm.
        let expected = std::f64::consts::PI * 0.004f64.powi(2) * 0.002;
        let err = (carve.removed_volume - expected).abs() / expected;
        assert!(
            err < 0.15,
            "removed {:.3e} vs cylinder {:.3e} ({:.0}% off)",
            carve.removed_volume,
            expected,
            err * 100.0
        );
        assert!((carve.initial_volume - 0.06 * 0.06 * 0.01).abs() / (0.06 * 0.06 * 0.01) < 0.02);
        assert!(
            (carve.initial_volume - carve.removed_volume - carve.remaining_volume).abs() < 1e-12
        );
        // The mesh is a closed-ish boundary with real triangles, far fewer
        // than one quad per voxel face (the greedy mesher earns its keep).
        assert!(!carve.mesh.indices.is_empty());
        assert!(
            carve.mesh.indices.len() < 40_000,
            "{} tris — greedy meshing broken?",
            carve.mesh.indices.len()
        );
    }

    #[test]
    fn carving_is_deterministic_and_misses_do_not_carve() {
        let mut scene = Scene::new(Arc::new(machining_robot()));
        let q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        // Stock well below the cutter: nothing is removed.
        scene
            .add_obstacle(
                "clear",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.01),
                },
                Isometry3::translation(tip.x, tip.y, tip.z - 0.2),
            )
            .unwrap();
        let timeline = hold_timeline(&scene, q, 0.5);
        let options = CarveOptions::default();
        let a = carve_stock(&scene, &timeline, "clear", 0, tcp, &options).unwrap();
        let b = carve_stock(&scene, &timeline, "clear", 0, tcp, &options).unwrap();
        assert_eq!(a.removed_volume, 0.0);
        assert_eq!(a.mesh.vertices, b.mesh.vertices);
        assert_eq!(a.mesh.indices, b.mesh.indices);
    }
}
