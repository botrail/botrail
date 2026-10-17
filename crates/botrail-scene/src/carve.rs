//! Stock carving: the cutter's swept volume subtracted from a stock
//! obstacle along a baked timeline, computed by
//! [cutrail](https://crates.io/crates/cutrail), a mesh-boolean material
//! removal core.
//!
//! This is *presentation and bookkeeping*, not verification (see
//! `design/design-machining.md` §3.3 S4 / §5): in a kinematic world the TCP
//! follows the toolpath exactly, so the carve can never contradict the
//! plan — what it adds is the picture (the machined part as a mesh) and
//! the numbers (removed / remaining volume). Deterministic by
//! construction: the stock is a closed mesh in its own frame, kept as
//! cubic tiles so a cut only touches the tiles under the cutter; the TCP
//! track is coalesced into segments that stay within `max_deviation` of
//! every sample, and each segment is subtracted as the convex hull of the
//! cutter at its ends. The booleans are exact for their polygonal inputs
//! and independent of scheduling.
//!
//! Units: botrail works in meters, cutrail in millimeters; the conversion
//! happens in this module and nowhere else.

use botrail_mesh::MeshData;
use botrail_model::Geometry;
use cutrail::{
    CutOutcome, Engine, EngineOptions, FaceKind, ManifoldKernel, MotionSegment, PartId,
    Pose as ToolPose, SegmentBuilder, StockShape, Tool, TriMesh,
};
use nalgebra::{Isometry3, Vector3};
use thiserror::Error;

use crate::rollout::SequenceTimeline;
use crate::Scene;

pub use crate::stages::{staged_timeline, staged_timeline_windows, StageWindow};

/// Meters to millimeters.
const MM: f64 = 1e3;
/// Cubic meters to cubic millimeters.
const MM3: f64 = 1e9;

#[derive(Debug, Error)]
pub enum CarveError {
    #[error("unknown obstacle `{0}`")]
    UnknownStock(String),
    #[error("unknown robot `{0}`")]
    UnknownRobot(usize),
    #[error("stock `{0}` cannot be carved: {1}")]
    Stock(String, String),
    #[error("cutter radius/length must be positive")]
    BadCutter,
    #[error("chord tolerance, deviation, tile size and dt must be positive")]
    BadOptions,
    #[error("cutting `{0}` failed: {1}")]
    Cut(String, String),
}

#[derive(Debug, Clone)]
pub struct CarveOptions {
    /// Chord tolerance (m) the cutter's sweeps and a cylindrical stock are
    /// polygonized with. The cut geometry is exact to this.
    pub chord_tolerance: f64,
    /// The sampled TCP track is coalesced into segments; no sample strays
    /// farther than this (m) from the segment it was folded into. Segments
    /// are also capped at 5 mm of travel and 2° of axis rotation.
    pub max_deviation: f64,
    /// Edge (m) of the cubic tiles the stock is kept in, so that a cut only
    /// touches the tiles under the cutter. `None` picks 2.5 cutter diameters.
    pub tile_size: Option<f64>,
    /// Cutter (flute) radius (m).
    pub cutter_radius: f64,
    /// Cutting length from the tip along the tool axis (m).
    pub cutter_length: f64,
    /// Timeline sampling period (s).
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
            chord_tolerance: 1e-5,
            max_deviation: 1e-5,
            tile_size: None,
            cutter_radius: 0.004,
            cutter_length: 0.03,
            dt: 0.01,
            stock_color: [0.58, 0.60, 0.63],
            cut_color: [0.92, 0.94, 0.98],
        }
    }
}

/// The carved stock: one closed boundary mesh in the stock's local frame
/// plus the volume bookkeeping.
#[derive(Debug, Clone)]
pub struct StockCarve {
    /// Boundary mesh of the remaining material, stock-local coordinates
    /// (place it at the stock's pose). Face colors mark the surviving skin
    /// and the surfaces the cutter made.
    pub mesh: MeshData,
    /// World pose of the stock at carve time.
    pub pose: Isometry3<f64>,
    /// Chord tolerance (m) the carve was run at.
    pub chord_tolerance: f64,
    pub initial_volume: f64,
    pub removed_volume: f64,
    pub remaining_volume: f64,
}

/// One snapshot of a progressive carve: the mesh of one tile of the stock
/// as of `time`. At the first cut every tile is snapshotted (the pristine
/// ones included, so the tiles can take over from the stock); afterwards
/// only tiles the sweep changed since the previous snapshot are, so idle
/// stretches and untouched tiles produce no duplicate meshes. An empty
/// mesh means the tile has been cut away entirely.
#[derive(Debug, Clone)]
pub struct CarveStage {
    /// Timeline time this state is current from.
    pub time: f64,
    /// Tile index (stable for the whole carve).
    pub cell: u32,
    pub mesh: MeshData,
    /// Cumulative removed volume as of this stage (m³).
    pub removed_volume: f64,
}

/// The stock's geometry as cutrail stock, in millimeters and the
/// obstacle's local frame.
fn stock_shape(name: &str, geometry: &Geometry) -> Result<StockShape, CarveError> {
    match geometry {
        Geometry::Box { size } => Ok(StockShape::Box {
            min: [-size.x / 2.0 * MM, -size.y / 2.0 * MM, -size.z / 2.0 * MM],
            max: [size.x / 2.0 * MM, size.y / 2.0 * MM, size.z / 2.0 * MM],
        }),
        // botrail's cylinder stands along z, like cutrail's.
        Geometry::Cylinder { radius, length } => Ok(StockShape::Cylinder {
            center: [0.0, 0.0],
            radius: radius * MM,
            z0: -length / 2.0 * MM,
            z1: length / 2.0 * MM,
        }),
        Geometry::Sphere { .. } => Err(CarveError::Stock(
            name.to_string(),
            "a sphere is not a stock shape".to_string(),
        )),
        Geometry::Mesh { path, scale } => {
            let data = botrail_mesh::load_path(path)
                .map_err(|e| CarveError::Stock(name.to_string(), e.to_string()))?;
            Ok(StockShape::Mesh(TriMesh {
                positions: data
                    .vertices
                    .iter()
                    .map(|v| {
                        [
                            v[0] * scale.x * MM,
                            v[1] * scale.y * MM,
                            v[2] * scale.z * MM,
                        ]
                    })
                    .collect(),
                triangles: data.indices,
            }))
        }
    }
}

/// A cutrail mesh (mm) as botrail mesh data (m) with the surface classing
/// as face colors.
fn mesh_data(mesh: &TriMesh, kinds: &[FaceKind], options: &CarveOptions) -> MeshData {
    MeshData {
        vertices: mesh
            .positions
            .iter()
            .map(|p| [p[0] / MM, p[1] / MM, p[2] / MM])
            .collect(),
        indices: mesh.triangles.clone(),
        face_colors: kinds
            .iter()
            .map(|k| match k {
                FaceKind::Stock => options.stock_color,
                FaceKind::Cut => options.cut_color,
            })
            .collect(),
    }
}

/// Subtracts one segment; `true` when it removed material.
fn cut(
    engine: &mut Engine,
    tool: &Tool,
    stock: &str,
    segment: &MotionSegment,
) -> Result<bool, CarveError> {
    let report = engine
        .cut(tool, segment)
        .map_err(|e| CarveError::Cut(stock.to_string(), e.to_string()))?;
    Ok(report.outcome != CutOutcome::NoChange)
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

/// [`carve_stock`] plus per-tile snapshots at `stages` equal time
/// boundaries — the raw material of progressive-removal display (each
/// tile's chain of snapshots shown for its windows via
/// [`staged_timeline_windows`]).
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
    let positive = |v: f64| v.is_finite() && v > 0.0;
    if !(positive(options.chord_tolerance)
        && positive(options.max_deviation)
        && positive(options.dt)
        && options.tile_size.is_none_or(positive))
    {
        return Err(CarveError::BadOptions);
    }
    if !(positive(options.cutter_radius) && positive(options.cutter_length)) {
        return Err(CarveError::BadCutter);
    }
    let (obstacle, _) = scene
        .obstacle_with_collider(stock)
        .ok_or_else(|| CarveError::UnknownStock(stock.to_string()))?;
    let track = &timeline
        .robots
        .get(robot)
        .ok_or(CarveError::UnknownRobot(robot))?
        .trajectory;

    let shape = stock_shape(stock, &obstacle.geometry)?;
    let tool = Tool::flat_end_mill(2.0 * options.cutter_radius * MM, options.cutter_length * MM);
    let tile = options.tile_size.unwrap_or(5.0 * options.cutter_radius) * MM;
    let engine_options = EngineOptions {
        sweep_tolerance: options.chord_tolerance * MM,
        stock_tolerance: options.chord_tolerance * MM,
        ..EngineOptions::default()
    }
    .with_tile_size(tile);
    let mut engine = Engine::new(ManifoldKernel, &[shape], engine_options)
        .map_err(|e| CarveError::Stock(stock.to_string(), e.to_string()))?;
    let initial_volume = engine.total_volume() / MM3;

    // The TCP in the stock's frame, in cutrail's units.
    let to_local = obstacle.pose.inverse();
    let tool_pose = |q: &[f64]| -> Option<ToolPose> {
        let poses = scene.fk_for(robot, q).ok()?;
        let local = to_local * poses[tcp];
        let tip = local.translation.vector;
        let axis = local.rotation * Vector3::z();
        Some(ToolPose::new(
            [tip.x * MM, tip.y * MM, tip.z * MM],
            [axis.x, axis.y, axis.z],
        ))
    };

    // Samples at `dt` are coalesced into segments: every sample stays within
    // `max_deviation` of its segment, corners break exactly at the corner
    // sample, and a 5 mm / 2° cap keeps each boolean small. The cutter is
    // also stamped where it starts, so a parked cutter still bores its hole.
    let mut builder = SegmentBuilder::new(5.0, 2.0).with_max_deviation(options.max_deviation * MM);
    let mut pending_stamp = tool_pose(&track.sample(0.0));
    if let Some(start) = pending_stamp {
        builder.push(start);
    }

    let stages = stages.max(1);
    let mut snapshots: Vec<CarveStage> = Vec::new();
    let mut last_revision = engine.revision();
    let mut known: Vec<PartId> = engine.parts().iter().map(|p| p.id).collect();
    let mut handed_over = false;
    let mut t = 0.0;
    for k in 1..=stages {
        let boundary = timeline.duration * k as f64 / stages as f64;
        let mut change_t = None;
        while t < boundary - 1e-9 {
            let next_t = (t + options.dt).min(boundary);
            if let Some(start) = pending_stamp.take() {
                if cut(
                    &mut engine,
                    &tool,
                    stock,
                    &MotionSegment::linear(start, start),
                )? {
                    change_t = Some(next_t);
                }
            }
            if let Some(pose) = tool_pose(&track.sample(next_t)) {
                if let Some(segment) = builder.push(pose) {
                    if cut(&mut engine, &tool, stock, &segment)? {
                        change_t = Some(next_t);
                    }
                }
            }
            t = next_t;
        }
        // The motion still held below the thresholds is cut at the boundary,
        // so the snapshot shows everything up to it.
        if let Some(segment) = builder.flush() {
            if cut(&mut engine, &tool, stock, &segment)? {
                change_t = Some(boundary);
            }
        }
        // Snapshot only where the sweep changed something: idle windows
        // extend the previous stage instead of duplicating meshes. The
        // snapshot is stamped with the *last change* time, not the window
        // boundary — the state is identical at both (nothing moved in
        // between), but this keeps the display exact from the moment it
        // switches and shows the pristine stock until the first real cut.
        let revision = engine.revision();
        if revision != last_revision {
            let time = change_t.unwrap_or(boundary);
            let removed_volume = initial_volume - engine.total_volume() / MM3;
            let current: Vec<PartId> = engine.parts().iter().map(|p| p.id).collect();
            let changed: Vec<PartId> = if handed_over {
                engine.changed_since(last_revision)
            } else {
                current.clone()
            };
            for id in changed {
                if let Some((mesh, kinds)) = engine.mesh_with_kinds(id) {
                    snapshots.push(CarveStage {
                        time,
                        cell: id.0,
                        mesh: mesh_data(&mesh, &kinds, options),
                        removed_volume,
                    });
                }
            }
            for id in &known {
                if !current.contains(id) {
                    snapshots.push(CarveStage {
                        time,
                        cell: id.0,
                        mesh: MeshData::new(Vec::new(), Vec::new()),
                        removed_volume,
                    });
                }
            }
            known = current;
            handed_over = true;
            last_revision = revision;
        }
    }

    let remaining_volume = engine.total_volume() / MM3;
    let mesh = if engine.parts().is_empty() {
        MeshData::new(Vec::new(), Vec::new())
    } else {
        let (mesh, kinds) = engine
            .merged_mesh_with_kinds()
            .map_err(|e| CarveError::Cut(stock.to_string(), e.to_string()))?;
        mesh_data(&mesh, &kinds, options)
    };
    Ok((
        StockCarve {
            mesh,
            pose: obstacle.pose,
            chord_tolerance: options.chord_tolerance,
            initial_volume,
            removed_volume: initial_volume - remaining_volume,
            remaining_volume,
        },
        snapshots,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rollout::SequenceTimeline;
    use botrail_model::{Geometry, RobotModel};
    use botrail_traj::JointTrajectory;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/assets/simple_arm.urdf");
    const SPINDLE: &str = crate::testdata::SPINDLE_URDF;

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

    /// The flange-down configuration and a plate whose top sits 2 mm above
    /// the parked cutter's tip, offset by `(dx, dy)` from the tip.
    fn parked_cutter_scene(dx: f64, dy: f64, size: Vector3<f64>) -> (Scene, Vec<f64>, usize) {
        let mut scene = Scene::new(Arc::new(machining_robot()));
        let q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        scene
            .add_obstacle(
                "plate",
                Geometry::Box { size },
                Isometry3::translation(tip.x + dx, tip.y + dy, tip.z + 0.002 - size.z / 2.0),
            )
            .unwrap();
        (scene, q, tcp)
    }

    #[test]
    fn a_parked_cutter_bores_one_round_hole() {
        let (scene, q, tcp) = parked_cutter_scene(0.0, 0.0, Vector3::new(0.06, 0.06, 0.01));
        let timeline = hold_timeline(&scene, q, 1.0);
        let carve =
            carve_stock(&scene, &timeline, "plate", 0, tcp, &CarveOptions::default()).unwrap();
        // Expected removal: a cylinder r=4mm, depth 2mm. The polygonal
        // cutter is sized so its mean width matches the diameter, which
        // leaves the area within half a percent.
        let expected = std::f64::consts::PI * 0.004f64.powi(2) * 0.002;
        let err = (carve.removed_volume - expected).abs() / expected;
        assert!(
            err < 0.01,
            "removed {:.3e} vs cylinder {:.3e} ({:.2}% off)",
            carve.removed_volume,
            expected,
            err * 100.0
        );
        assert!((carve.initial_volume - 0.06 * 0.06 * 0.01).abs() / (0.06 * 0.06 * 0.01) < 1e-9);
        assert!(
            (carve.initial_volume - carve.removed_volume - carve.remaining_volume).abs() < 1e-15
        );
        // One closed mesh with real triangles, both surface classes present.
        assert!(!carve.mesh.indices.is_empty() && carve.mesh.indices.len() < 5_000);
        assert_eq!(carve.mesh.face_colors.len(), carve.mesh.indices.len());
        let options = CarveOptions::default();
        assert!(carve.mesh.face_colors.contains(&options.cut_color));
        assert!(carve.mesh.face_colors.contains(&options.stock_color));
        assert_eq!(carve.chord_tolerance, options.chord_tolerance);
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
        assert!(a.mesh.face_colors.iter().all(|c| *c == options.stock_color));
    }

    #[test]
    fn stages_snapshot_every_tile_at_the_handover_then_only_changed_tiles() {
        // Four 30 mm tiles; the parked cutter bores into one of them only.
        let (scene, q, tcp) = parked_cutter_scene(-0.015, -0.015, Vector3::new(0.06, 0.06, 0.01));
        let timeline = hold_timeline(&scene, q, 1.0);
        let options = CarveOptions {
            tile_size: Some(0.03),
            ..CarveOptions::default()
        };
        let (carve, stages) =
            carve_stock_staged(&scene, &timeline, "plate", 0, tcp, &options, 4).unwrap();
        assert!(carve.removed_volume > 0.0);
        // The first cut hands the display over from the stock to all four
        // tiles at once; nothing changes afterwards, so no more snapshots.
        assert_eq!(
            stages.len(),
            4,
            "{:?}",
            stages.iter().map(|s| (s.time, s.cell)).collect::<Vec<_>>()
        );
        let cells: std::collections::BTreeSet<u32> = stages.iter().map(|s| s.cell).collect();
        assert_eq!(cells.len(), 4);
        assert!(stages
            .iter()
            .all(|s| s.time == stages[0].time && s.time > 0.0 && s.time <= 0.25));
        let cut_tiles = stages
            .iter()
            .filter(|s| s.mesh.face_colors.contains(&options.cut_color))
            .count();
        assert_eq!(cut_tiles, 1);
        assert!(stages.iter().all(|s| !s.mesh.indices.is_empty()));
        assert!(stages
            .iter()
            .all(|s| (s.removed_volume - carve.removed_volume).abs() < 1e-18));
    }

    #[test]
    fn bad_options_and_unknown_stock_are_rejected() {
        let (scene, q, tcp) = parked_cutter_scene(0.0, 0.0, Vector3::new(0.06, 0.06, 0.01));
        let timeline = hold_timeline(&scene, q, 1.0);
        let bad = CarveOptions {
            chord_tolerance: 0.0,
            ..CarveOptions::default()
        };
        assert!(matches!(
            carve_stock(&scene, &timeline, "plate", 0, tcp, &bad),
            Err(CarveError::BadOptions)
        ));
        assert!(matches!(
            carve_stock(&scene, &timeline, "nope", 0, tcp, &CarveOptions::default()),
            Err(CarveError::UnknownStock(_))
        ));
    }
}
