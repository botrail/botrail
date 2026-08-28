//! Toolpaths: continuous Cartesian process paths — milling, trimming,
//! deburring, and the non-contact ones too (spray coating, dispensing) —
//! followed at a commanded feed rate.
//!
//! A [`Toolpath`] is an authored artifact — an ordered list of moves in a
//! part frame, each a chain of targets carrying a position, a tool-axis
//! direction, and optionally a spin angle about that axis. Baking resolves
//! the part frame to world, resamples at chord tolerance, follows the path
//! with seed-continuous IK (axis-aligned 5-DOF where the spin is free),
//! and time-parameterizes the whole path in one piece: cutting intervals
//! are floored at `chord length / feed`, so the trajectory holds the
//! commanded feed and slows only where joint limits force it. Unlike
//! `Motion` segments there is no rest at interior targets.
//!
//! What this module does *not* model: process physics (cutting forces,
//! deflection, chatter, tool wear) or CAM correctness (gouge / undercut).
//! The questions answered are reach, clearance, and time — see
//! `design/design-machining.md`. What a process *leaves behind* is a
//! separate, offline pass over the baked timeline: [`crate::carve`] for
//! material removed, [`crate::coat`] for film deposited.

use botrail_model::RobotModel;
use botrail_traj::JointTrajectory;
use nalgebra::{Isometry3, Point3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::Scene;

#[derive(Debug, Error)]
pub enum ToolpathError {
    #[error("unknown toolpath `{0}`")]
    UnknownToolpath(String),
    #[error("toolpath `{toolpath}` references unknown frame `{frame}`")]
    UnknownFrame { toolpath: String, frame: String },
    #[error("toolpath `{0}` has no targets")]
    Empty(String),
    #[error("toolpath `{toolpath}`: target {index} has a zero-length tool axis")]
    BadAxis { toolpath: String, index: usize },
    #[error("toolpath `{toolpath}`: move {index} has a non-positive feed")]
    BadFeed { toolpath: String, index: usize },
    #[error(
        "toolpath follow failed at sample {sample}/{total} ({fraction:.1}%, move {move_index}): {reason}"
    )]
    FollowFailed {
        sample: usize,
        total: usize,
        fraction: f64,
        move_index: usize,
        reason: String,
    },
    #[error("time parameterization failed: {0}")]
    Timing(#[from] botrail_traj::TrajError),
}

/// One target along a toolpath, in the part frame.
#[derive(Debug, Clone)]
pub struct PathTarget {
    pub position: Point3<f64>,
    /// Direction of the tool axis — from the cutter tip toward the tool
    /// body (`+Z` of the part frame for 3-axis milling; APT `ijk`).
    /// The TCP link's local `+Z` is driven onto this direction.
    pub tool_axis: Unit<Vector3<f64>>,
    /// Rotation about the tool axis, radians, measured from a deterministic
    /// reference (the world basis vector least aligned with the axis,
    /// projected). `None` leaves the spin to the solver — the normal case
    /// for axis-symmetric tools.
    pub spin: Option<f64>,
}

/// How the tool moves through a chain of targets.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ToolMoveKind {
    /// Non-cutting reposition; timed by joint limits (optionally capped by
    /// [`ToolpathOptions::rapid_speed`]).
    Rapid,
    /// Cutting move at the commanded feed rate (m/s).
    Feed(f64),
}

#[derive(Debug, Clone)]
pub struct ToolMove {
    pub kind: ToolMoveKind,
    /// Targets visited in order; the interval *into* each target has this
    /// move's kind. The first target of the first move is the path start.
    pub targets: Vec<PathTarget>,
    /// The process setting a feed move runs with — a *brush* (a named
    /// applicator + flow + trigger timing, ABB's word) declared on the
    /// scene. This is the program's own trigger, per stroke: in a
    /// toolpath that names brushes anywhere, a feed move without one is
    /// a move with the gun *off* (brush 0), which is how a raster's
    /// turnarounds run at speed without spraying. A toolpath that names
    /// none sprays every feed move with whatever applicator the film
    /// integrator is handed. Meaningless on a rapid.
    pub brush: Option<String>,
}

/// A continuous Cartesian process path, authored relative to a part frame.
#[derive(Debug, Clone)]
pub struct Toolpath {
    pub name: String,
    /// Part frame ([`crate::Frame`] name) the targets are expressed in;
    /// `None` means world. Resolution happens at bake time, so moving the
    /// frame and re-simulating re-solves the whole path.
    pub frame: Option<String>,
    pub moves: Vec<ToolMove>,
}

impl Toolpath {
    pub fn target_count(&self) -> usize {
        self.moves.iter().map(|m| m.targets.len()).sum()
    }

    /// Whether any move names a brush — the toolpath triggers per stroke.
    pub fn uses_brushes(&self) -> bool {
        self.moves.iter().any(|m| m.brush.is_some())
    }

    /// Whether the interval into `move_index`'s targets sprays: a feed
    /// move with a brush, or any feed move when the path names none.
    pub fn move_sprays(&self, move_index: usize) -> bool {
        match self.moves.get(move_index) {
            Some(m) => match m.kind {
                ToolMoveKind::Rapid => false,
                ToolMoveKind::Feed(_) => !self.uses_brushes() || m.brush.is_some(),
            },
            None => false,
        }
    }
}

// ------------------------------------------------------------------ wire

/// Serialized form (project files, Python boundary). Positions and axes in
/// the part frame; `spin` optional.
#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct PathTargetMsg {
    pub position: [f64; 3],
    #[serde(default = "default_tool_axis")]
    pub tool_axis: [f64; 3],
    #[serde(default)]
    pub spin: Option<f64>,
}

fn default_tool_axis() -> [f64; 3] {
    [0.0, 0.0, 1.0]
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "type", rename_all = "snake_case")]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub enum ToolMoveMsg {
    Rapid {
        targets: Vec<PathTargetMsg>,
    },
    Feed {
        feed: f64,
        targets: Vec<PathTargetMsg>,
        /// Brush name (absent in files written before brushes existed,
        /// which is the same as none).
        #[serde(default, skip_serializing_if = "Option::is_none")]
        brush: Option<String>,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct ToolpathMsg {
    /// Defaults to empty: the Python boundary passes the name separately
    /// (`scene.add_toolpath(name, tp)`) and overrides this field.
    #[serde(default)]
    pub name: String,
    #[serde(default)]
    pub frame: Option<String>,
    pub moves: Vec<ToolMoveMsg>,
}

pub fn toolpath_msg(tp: &Toolpath) -> ToolpathMsg {
    let targets = |ts: &[PathTarget]| {
        ts.iter()
            .map(|t| PathTargetMsg {
                position: t.position.coords.into(),
                tool_axis: (*t.tool_axis.as_ref()).into(),
                spin: t.spin,
            })
            .collect()
    };
    ToolpathMsg {
        name: tp.name.clone(),
        frame: tp.frame.clone(),
        moves: tp
            .moves
            .iter()
            .map(|m| match m.kind {
                ToolMoveKind::Rapid => ToolMoveMsg::Rapid {
                    targets: targets(&m.targets),
                },
                ToolMoveKind::Feed(feed) => ToolMoveMsg::Feed {
                    feed,
                    targets: targets(&m.targets),
                    brush: m.brush.clone(),
                },
            })
            .collect(),
    }
}

pub fn toolpath_from_msg(msg: &ToolpathMsg) -> Result<Toolpath, ToolpathError> {
    let mut index = 0usize;
    let mut convert = |targets: &[PathTargetMsg]| -> Result<Vec<PathTarget>, ToolpathError> {
        targets
            .iter()
            .map(|t| {
                let axis = Vector3::from(t.tool_axis);
                if axis.norm() < 1e-9 {
                    return Err(ToolpathError::BadAxis {
                        toolpath: msg.name.clone(),
                        index,
                    });
                }
                index += 1;
                Ok(PathTarget {
                    position: Point3::from(Vector3::from(t.position)),
                    tool_axis: Unit::new_normalize(axis),
                    spin: t.spin,
                })
            })
            .collect()
    };
    let mut moves = Vec::with_capacity(msg.moves.len());
    for (i, m) in msg.moves.iter().enumerate() {
        let mv = match m {
            ToolMoveMsg::Rapid { targets } => ToolMove {
                kind: ToolMoveKind::Rapid,
                targets: convert(targets)?,
                brush: None,
            },
            ToolMoveMsg::Feed {
                feed,
                targets,
                brush,
            } => {
                if *feed <= 0.0 {
                    return Err(ToolpathError::BadFeed {
                        toolpath: msg.name.clone(),
                        index: i,
                    });
                }
                ToolMove {
                    kind: ToolMoveKind::Feed(*feed),
                    targets: convert(targets)?,
                    brush: brush.clone(),
                }
            }
        };
        moves.push(mv);
    }
    Ok(Toolpath {
        name: msg.name.clone(),
        frame: msg.frame.clone(),
        moves,
    })
}

/// A display polyline: `[x, y, z]` per vertex, world meters.
pub type Polyline = Vec<[f64; 3]>;

/// World-resolved display polylines of a toolpath, split by kind:
/// `(feed, rapid)`. Each move contributes one polyline starting at the
/// previous move's last point; single-point orphans are dropped. `None`
/// when the part frame does not resolve (the bake would already have
/// failed on it — display stays best-effort).
pub fn overlay_polylines(
    scene: &Scene,
    toolpath: &Toolpath,
) -> Option<(Vec<Polyline>, Vec<Polyline>)> {
    let frame = match &toolpath.frame {
        Some(name) => scene.frame(name)?.pose,
        None => Isometry3::identity(),
    };
    let mut feed: Vec<Polyline> = Vec::new();
    let mut rapid: Vec<Polyline> = Vec::new();
    let mut prev: Option<[f64; 3]> = None;
    for mv in &toolpath.moves {
        if mv.targets.is_empty() {
            continue;
        }
        let mut pts: Polyline = Vec::with_capacity(mv.targets.len() + 1);
        if let Some(p) = prev {
            pts.push(p);
        }
        for t in &mv.targets {
            let w = frame * t.position;
            pts.push([w.x, w.y, w.z]);
        }
        prev = pts.last().copied();
        if pts.len() >= 2 {
            match mv.kind {
                ToolMoveKind::Rapid => rapid.push(pts),
                ToolMoveKind::Feed(_) => feed.push(pts),
            }
        }
    }
    Some((feed, rapid))
}

// -------------------------------------------------------------- sampling

/// How the free spin (rotation about the tool axis) is chosen along a
/// path whose targets leave it open.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpinMode {
    /// Seed-continuous descent: each sample starts from the previous
    /// solution and the null-space centering parks the spin where the
    /// joints are locally most comfortable. Milliseconds; can walk into a
    /// wrist limit that only a global look-ahead would avoid.
    Greedy,
    /// Descartes-style global optimization: per sample, one candidate
    /// configuration is solved for each grid spin, and a Viterbi pass
    /// picks the cheapest feasible chain (joint motion + limit-margin
    /// penalty) over the whole path — spending spin early to stay
    /// solvable late, which no local rule can do. Deterministic; costs
    /// seconds where greedy costs milliseconds.
    Optimize {
        /// Spin grid step (rad).
        spin_step: f64,
        /// Candidate chains kept per sample (beam width).
        beam_width: usize,
    },
}

impl SpinMode {
    /// The optimizing mode at its defaults: a 15° spin grid, beam of 8.
    /// The grid must stay finer than the jump threshold expressed at the
    /// wrist (0.5 rad): a coarser grid would make every spin *change*
    /// exceed the edge check and freeze the chain at constant spin.
    pub fn optimize() -> Self {
        SpinMode::Optimize {
            spin_step: std::f64::consts::TAU / 24.0,
            beam_width: 8,
        }
    }
}

#[derive(Debug, Clone)]
pub struct ToolpathOptions {
    /// Translation resampling step between IK follow points (m).
    pub step_pos: f64,
    /// Tool-axis rotation resampling step (rad).
    pub step_rot: f64,
    /// Maximum joint-space jump between consecutive follow points; larger
    /// jumps indicate an IK branch change.
    pub jump_threshold: f64,
    /// Cartesian speed cap for rapid intervals (m/s); `None` = joint-limit
    /// timing only.
    pub rapid_speed: Option<f64>,
    /// Permitted tool-axis deviation (rad) on spin-free samples — the
    /// lead/tilt tolerance of a process that does not need the axis exact
    /// (chamfering, deburring). The solver still pulls toward the exact
    /// axis; this widens what counts as reached. Greedy mode only:
    /// the spin grid of [`SpinMode::Optimize`] solves exact poses.
    pub axis_tolerance: f64,
    /// Spin selection strategy for spin-free samples.
    pub spin: SpinMode,
}

impl Default for ToolpathOptions {
    fn default() -> Self {
        ToolpathOptions {
            step_pos: 0.005,
            step_rot: 0.05,
            jump_threshold: 0.5,
            rapid_speed: None,
            axis_tolerance: 0.0,
            spin: SpinMode::Greedy,
        }
    }
}

/// One world-resolved, resampled point of a toolpath.
#[derive(Debug, Clone)]
pub struct ToolpathSample {
    pub position: Point3<f64>,
    pub tool_axis: Unit<Vector3<f64>>,
    pub spin: Option<f64>,
    /// Index into [`Toolpath::moves`].
    pub move_index: usize,
    /// Commanded feed of the interval ending at this sample; `None` for
    /// rapid intervals and for the path start.
    pub feed: Option<f64>,
    /// Chord length from the previous sample (m); 0 at the path start.
    pub chord: f64,
}

/// Deterministic spin reference: the world basis vector least aligned with
/// the axis, projected onto its perpendicular plane. Matches the IK side's
/// convention only in being deterministic — spin values are defined by
/// *this* function.
fn spin_reference(axis: &Unit<Vector3<f64>>) -> Unit<Vector3<f64>> {
    let a = axis.as_ref();
    let pick = if a.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    Unit::new_normalize(pick - a * pick.dot(a))
}

/// Full target pose from position + axis + spin: local `+Z` along the
/// axis, local `+X` at `spin` radians from the reference.
pub fn target_pose(position: &Point3<f64>, axis: &Unit<Vector3<f64>>, spin: f64) -> Isometry3<f64> {
    let x0 = spin_reference(axis);
    let x = UnitQuaternion::from_axis_angle(axis, spin) * x0.into_inner();
    let z = axis.into_inner();
    let y = z.cross(&x);
    let rot = Rotation3::from_basis_unchecked(&[x, y, z]);
    Isometry3::from_parts(
        Translation3::from(position.coords),
        UnitQuaternion::from_rotation_matrix(&rot),
    )
}

/// Geodesic interpolation between two axis directions.
fn slerp_axis(from: &Unit<Vector3<f64>>, to: &Unit<Vector3<f64>>, u: f64) -> Unit<Vector3<f64>> {
    match UnitQuaternion::rotation_between(from.as_ref(), to.as_ref()) {
        Some(q) => Unit::new_normalize(q.powf(u) * from.into_inner()),
        None => {
            // Antiparallel: rotate through a deterministic perpendicular.
            let pivot = spin_reference(from);
            let q = UnitQuaternion::from_axis_angle(&pivot, std::f64::consts::PI);
            Unit::new_normalize(q.powf(u) * from.into_inner())
        }
    }
}

/// Resolves the part frame and resamples the whole path at chord
/// tolerance. The first target becomes sample 0.
pub fn resolve_and_sample(
    scene: &Scene,
    toolpath: &Toolpath,
    options: &ToolpathOptions,
) -> Result<Vec<ToolpathSample>, ToolpathError> {
    let frame = match &toolpath.frame {
        Some(name) => {
            scene
                .frame(name)
                .map(|f| f.pose)
                .ok_or_else(|| ToolpathError::UnknownFrame {
                    toolpath: toolpath.name.clone(),
                    frame: name.clone(),
                })?
        }
        None => Isometry3::identity(),
    };
    let mut samples: Vec<ToolpathSample> = Vec::new();
    for (move_index, mv) in toolpath.moves.iter().enumerate() {
        let feed = match mv.kind {
            ToolMoveKind::Rapid => None,
            ToolMoveKind::Feed(f) => Some(f),
        };
        for target in &mv.targets {
            let position = frame * target.position;
            let tool_axis = Unit::new_normalize(frame.rotation * target.tool_axis.into_inner());
            let Some(prev) = samples.last() else {
                samples.push(ToolpathSample {
                    position,
                    tool_axis,
                    spin: target.spin,
                    move_index,
                    feed: None,
                    chord: 0.0,
                });
                continue;
            };
            let dist = (position - prev.position).norm();
            let angle = prev
                .tool_axis
                .cross(&tool_axis)
                .norm()
                .atan2(prev.tool_axis.dot(&tool_axis));
            let steps = ((dist / options.step_pos).max(angle / options.step_rot))
                .ceil()
                .max(1.0) as usize;
            let from = prev.clone();
            for k in 1..=steps {
                let u = k as f64 / steps as f64;
                let p = Point3::from(from.position.coords.lerp(&position.coords, u));
                let a = slerp_axis(&from.tool_axis, &tool_axis, u);
                let spin = match (from.spin, target.spin) {
                    (Some(s0), Some(s1)) => Some(s0 + (s1 - s0) * u),
                    _ => target.spin.filter(|_| k == steps),
                };
                samples.push(ToolpathSample {
                    position: p,
                    tool_axis: a,
                    spin,
                    move_index,
                    feed,
                    chord: dist / steps as f64,
                });
            }
        }
    }
    if samples.is_empty() {
        return Err(ToolpathError::Empty(toolpath.name.clone()));
    }
    Ok(samples)
}

// --------------------------------------------------------------- solving

/// Why a sample could not be followed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IssueKind {
    /// IK did not converge — out of reach, or reachable only through a
    /// joint limit.
    Unreachable,
    /// The solution jumped to a different IK branch.
    ConfigJump,
    /// The solved configuration collides (or violates limits).
    Collision,
}

#[derive(Debug, Clone)]
pub struct PointIssue {
    pub sample: usize,
    pub move_index: usize,
    /// World target position of the failing sample.
    pub position: Point3<f64>,
    pub kind: IssueKind,
    pub detail: String,
}

/// Face diagnosis of a toolpath: every sample attempted, all failures
/// collected (a failed sample keeps the previous seed and the walk goes
/// on). `ok()` iff no sample failed.
#[derive(Debug, Clone)]
pub struct ToolpathReport {
    pub total_samples: usize,
    pub issues: Vec<PointIssue>,
}

impl ToolpathReport {
    pub fn ok(&self) -> bool {
        self.issues.is_empty()
    }
}

/// How the bake held the commanded feed. The floors make `length / feed`
/// a hard lower bound, so the joints can only *slow* a cut — this report
/// says where they did, and which axis owned it.
#[derive(Debug, Clone)]
pub struct FeedReport {
    /// Commanded cutting time / achieved cutting time; 1.0 = the feed was
    /// held everywhere.
    pub hold_ratio: f64,
    pub commanded_cut_seconds: f64,
    pub achieved_cut_seconds: f64,
    /// Maximal runs of cutting intervals slower than commanded (>2%),
    /// split at move boundaries.
    pub slow_spans: Vec<SlowSpan>,
}

/// One stretch of cutting the joints could not run at the commanded feed.
#[derive(Debug, Clone)]
pub struct SlowSpan {
    /// Trajectory time range.
    pub start: f64,
    pub end: f64,
    /// Index into [`Toolpath::moves`].
    pub move_index: usize,
    pub commanded_feed: f64,
    /// Path length over elapsed time across the span (m/s).
    pub achieved_feed: f64,
    /// Actuated-joint index that ran closest to its velocity limit over
    /// the span — the axis that owns the slowdown.
    pub limiting_joint: usize,
}

/// A baked toolpath: one continuous trajectory holding the commanded feed
/// wherever joint limits allow.
#[derive(Debug, Clone)]
pub struct PlannedToolpath {
    pub trajectory: JointTrajectory,
    /// Solved configuration per sample (same length as `samples`).
    pub path: Vec<Vec<f64>>,
    pub samples: Vec<ToolpathSample>,
    /// Trajectory time at which each sample is reached (same length as
    /// `samples`; the time parameterization may add points between them).
    pub sample_times: Vec<f64>,
    /// Trajectory time at which each [`Toolpath::moves`] entry completes.
    pub move_ends: Vec<f64>,
    /// Total cutting (feed) length (m).
    pub cut_length: f64,
    /// Total rapid length (m).
    pub rapid_length: f64,
    pub feed_report: FeedReport,
}

fn joint_distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

fn follow(
    scene: &Scene,
    robot: usize,
    tcp: usize,
    samples: &[ToolpathSample],
    options: &ToolpathOptions,
    stop_at_first: bool,
) -> (Vec<Vec<f64>>, Vec<PointIssue>) {
    let mut path = Vec::with_capacity(samples.len());
    let mut issues = Vec::new();
    let mut seed = scene.robots()[robot].joint_positions().to_vec();
    for (i, s) in samples.iter().enumerate() {
        let (mode, spin) = match s.spin {
            Some(spin) => (botrail_kin::IkMode::Pose, spin),
            None => (botrail_kin::IkMode::Axis, 0.0),
        };
        let target = target_pose(&s.position, &s.tool_axis, spin);
        let ik_options = botrail_kin::IkOptions {
            mode,
            max_iters: 50,
            // The first sample may sit far from the seed (the robot walks
            // in from wherever it is); mid-path solves must stay on their
            // branch, so restarts are reserved for sample 0.
            restarts: if i == 0 { 4 } else { 0 },
            // Lead/tilt tolerance: a spin-free sample counts as reached
            // once the axis is inside the permitted cone. An authored
            // spin stays exact — the author pinned that pose on purpose.
            tol_rot: if s.spin.is_none() {
                options.axis_tolerance.max(1e-4)
            } else {
                1e-4
            },
            ..botrail_kin::IkOptions::default()
        };
        let fail = |kind: IssueKind, detail: String| PointIssue {
            sample: i,
            move_index: s.move_index,
            position: s.position,
            kind,
            detail,
        };
        let ik = match scene.solve_ik_world_for(robot, tcp, &target, &seed, &ik_options) {
            Ok(ik) => ik,
            Err(e) => {
                issues.push(fail(IssueKind::Unreachable, e.to_string()));
                if stop_at_first {
                    return (path, issues);
                }
                path.push(seed.clone());
                continue;
            }
        };
        // Feed samples honor the allowed-contact exemptions (the cutter is
        // supposed to be in the stock); rapid samples suspend them — while
        // not cutting, any contact is a crash, including with the stock.
        let valid = if s.feed.is_some() {
            scene.is_state_valid_for(robot, &ik.q)
        } else {
            scene.is_state_valid_strict_for(robot, &ik.q)
        };
        let issue = if !ik.converged {
            Some(fail(
                IssueKind::Unreachable,
                format!(
                    "IK did not converge (pos {:.2}mm, axis {:.3}rad)",
                    ik.pos_error * 1e3,
                    ik.rot_error
                ),
            ))
        } else if i > 0 && joint_distance(&ik.q, &seed) > options.jump_threshold {
            Some(fail(
                IssueKind::ConfigJump,
                format!(
                    "configuration jump ({:.2} rad joint-space)",
                    joint_distance(&ik.q, &seed)
                ),
            ))
        } else if !valid {
            Some(fail(
                IssueKind::Collision,
                if s.feed.is_some() {
                    "collision (or joint-limit) at the solved configuration".to_string()
                } else {
                    "contact during a rapid (allowed pairs do not apply while not cutting)"
                        .to_string()
                },
            ))
        } else {
            None
        };
        match issue {
            Some(issue) => {
                issues.push(issue);
                if stop_at_first {
                    return (path, issues);
                }
                // Keep the previous seed so later samples still get a
                // meaningful attempt.
                path.push(seed.clone());
            }
            None => {
                seed = ik.q.clone();
                path.push(ik.q);
            }
        }
    }
    (path, issues)
}

/// One Viterbi node: a solved configuration at some grid spin, with the
/// cheapest cost of any feasible chain reaching it and a backpointer into
/// the previous layer's kept nodes.
struct SpinNode {
    q: Vec<f64>,
    spin: f64,
    cost: f64,
    parent: Option<usize>,
}

/// Weight of the limit-margin penalty against joint motion in the DP
/// cost. Small on purpose: margins break ties and bend the chain away
/// from limits, but never outweigh actually moving less.
const MARGIN_WEIGHT: f64 = 0.05;

fn limit_margin_penalty(model: &RobotModel, q: &[f64]) -> f64 {
    q.iter()
        .zip(&model.actuated_joints)
        .map(|(qi, &ji)| match model.joints[ji].limits {
            Some(l) => {
                let half = 0.5 * (l.upper - l.lower);
                if half > 1e-9 {
                    let mid = 0.5 * (l.upper + l.lower);
                    ((qi - mid) / half).powi(2)
                } else {
                    0.0
                }
            }
            None => 0.0,
        })
        .sum()
}

/// The spin a solved configuration actually realized: the angle of the
/// reached TCP `+X` about `axis`, measured from the deterministic
/// reference — [`target_pose`]'s inverse for the spin coordinate.
fn spin_of(
    scene: &Scene,
    robot: usize,
    tcp: usize,
    q: &[f64],
    axis: &Unit<Vector3<f64>>,
) -> Option<f64> {
    let pose = scene.fk_for(robot, q).ok()?[tcp];
    let x = pose.rotation * Vector3::x();
    let u = spin_reference(axis).into_inner();
    let v = axis.cross(&u);
    Some(x.dot(&v).atan2(x.dot(&u)))
}

/// Descartes-style global spin optimization, adapted to a numeric IK: the
/// feasible spin window is usually a narrow tube (most absolute spins
/// fold the wrist into the arm), so candidates are generated *relative*
/// to it — each kept parent proposes its natural continuation (the 5-DOF
/// axis-aligned solve, exactly greedy's move) plus grid offsets to
/// either side of the spin that solve realized. A Viterbi pass over the
/// `beam_width` cheapest feasible chains then chooses which way the spin
/// drifts — spending spin early to stay solvable late, which no local
/// rule can do.
#[allow(clippy::too_many_arguments)]
fn follow_optimized(
    scene: &Scene,
    robot: usize,
    tcp: usize,
    samples: &[ToolpathSample],
    options: &ToolpathOptions,
    spin_step: f64,
    beam_width: usize,
    stop_at_first: bool,
) -> (Vec<Vec<f64>>, Vec<PointIssue>) {
    let model = scene.robots()[robot].model.clone();
    let start_seed = scene.robots()[robot].joint_positions().to_vec();
    let beam = beam_width.max(1);
    // Neighborhood half-width, in grid steps: a fresh start explores wide
    // (the whole window matters), a running chain drifts one step per
    // 5 mm sample at most — which is already 3°/mm of spin authority.
    const FRESH_OFFSETS: i32 = 8;
    const CHAIN_OFFSETS: i32 = 1;

    let mut issues: Vec<PointIssue> = Vec::new();
    let mut layers: Vec<Vec<SpinNode>> = Vec::with_capacity(samples.len());

    for (i, s) in samples.iter().enumerate() {
        let fresh_start = layers.last().is_none_or(Vec::is_empty);
        // (seed, spin request): None = the natural axis-aligned solve.
        let mut attempts: Vec<(Vec<f64>, Option<f64>)> = Vec::new();
        match s.spin {
            // An authored spin pins the pose.
            Some(spin) => match layers.last().filter(|l| !l.is_empty()) {
                Some(prev) => {
                    for node in prev.iter() {
                        attempts.push((node.q.clone(), Some(spin)));
                    }
                }
                None => attempts.push((start_seed.clone(), Some(spin))),
            },
            None => match layers.last().filter(|l| !l.is_empty()) {
                Some(prev) => {
                    for node in prev.iter() {
                        attempts.push((node.q.clone(), None));
                        for k in 1..=CHAIN_OFFSETS {
                            let d = k as f64 * spin_step;
                            attempts.push((node.q.clone(), Some(node.spin + d)));
                            attempts.push((node.q.clone(), Some(node.spin - d)));
                        }
                    }
                }
                None => {
                    attempts.push((start_seed.clone(), None));
                }
            },
        }
        let mut candidates: Vec<SpinNode> = Vec::new();
        let mut natural_fresh: Option<(Vec<f64>, f64)> = None;
        let mut round = 0;
        while round < 2 {
            for (seed, spin_req) in &attempts {
                let spin = spin_req.unwrap_or(0.0);
                let mode = match spin_req {
                    Some(_) => botrail_kin::IkMode::Pose,
                    None => botrail_kin::IkMode::Axis,
                };
                let target = target_pose(&s.position, &s.tool_axis, spin);
                let ik_options = botrail_kin::IkOptions {
                    mode,
                    max_iters: 50,
                    restarts: if fresh_start { 4 } else { 0 },
                    ..botrail_kin::IkOptions::default()
                };
                let Ok(ik) = scene.solve_ik_world_for(robot, tcp, &target, seed, &ik_options)
                else {
                    continue;
                };
                if !ik.converged {
                    continue;
                }
                let valid = if s.feed.is_some() {
                    scene.is_state_valid_for(robot, &ik.q)
                } else {
                    scene.is_state_valid_strict_for(robot, &ik.q)
                };
                if !valid {
                    continue;
                }
                let realized = match spin_req {
                    Some(v) => *v,
                    None => match spin_of(scene, robot, tcp, &ik.q, &s.tool_axis) {
                        Some(v) => v,
                        None => continue,
                    },
                };
                if fresh_start && spin_req.is_none() {
                    natural_fresh = Some((ik.q.clone(), realized));
                }
                let node_penalty = MARGIN_WEIGHT * limit_margin_penalty(&model, &ik.q);
                let (cost, parent) = match layers.last().filter(|l| !l.is_empty()) {
                    Some(prev) => {
                        let best = prev
                            .iter()
                            .enumerate()
                            .filter_map(|(p, node)| {
                                let d = joint_distance(&node.q, &ik.q);
                                (d <= options.jump_threshold).then_some((node.cost + d, p))
                            })
                            .min_by(|a, b| a.0.total_cmp(&b.0));
                        match best {
                            Some((c, p)) => (c + node_penalty, Some(p)),
                            None => continue,
                        }
                    }
                    None => (node_penalty, None),
                };
                // Viterbi merge: an equivalent configuration reached two
                // ways is one state — keep the cheaper chain.
                if let Some(existing) = candidates
                    .iter_mut()
                    .find(|c| joint_distance(&c.q, &ik.q) < 1e-2)
                {
                    if cost < existing.cost {
                        *existing = SpinNode {
                            q: ik.q,
                            spin: realized,
                            cost,
                            parent,
                        };
                    }
                    continue;
                }
                candidates.push(SpinNode {
                    q: ik.q,
                    spin: realized,
                    cost,
                    parent,
                });
            }
            round += 1;
            // A fresh start explores the window around the natural spin
            // in a second round, once that spin is known.
            if round == 1 && fresh_start && s.spin.is_none() {
                if let Some((q, spin)) = natural_fresh.clone() {
                    attempts = (1..=FRESH_OFFSETS)
                        .flat_map(|k| {
                            let d = k as f64 * spin_step;
                            [(q.clone(), Some(spin + d)), (q.clone(), Some(spin - d))]
                        })
                        .collect();
                    continue;
                }
            }
            break;
        }
        if candidates.is_empty() {
            issues.push(PointIssue {
                sample: i,
                move_index: s.move_index,
                position: s.position,
                kind: IssueKind::Unreachable,
                detail: "no feasible candidate around the natural spin \
                         (every attempt unreachable, colliding, or beyond the beam)"
                    .to_string(),
            });
            if stop_at_first {
                return (backtrack(&layers), issues);
            }
            layers.push(Vec::new());
            continue;
        }
        candidates.sort_by(|a, b| a.cost.total_cmp(&b.cost));
        // Spin-diverse beam: taking the `beam` cheapest outright lets one
        // spin basin crowd out the others, and the pruned basin is
        // sometimes the only one that survives the next corner. Keep at
        // most one candidate per half-grid-step of spin first, then fill
        // any remaining beam slots by plain cost order.
        let mut kept: Vec<SpinNode> = Vec::with_capacity(beam);
        let mut passed_over: Vec<SpinNode> = Vec::new();
        for node in candidates {
            if kept.len() >= beam {
                break;
            }
            if kept
                .iter()
                .any(|k| spin_gap(k.spin, node.spin) < spin_step * 0.5)
            {
                passed_over.push(node);
            } else {
                kept.push(node);
            }
        }
        for node in passed_over {
            if kept.len() >= beam {
                break;
            }
            kept.push(node);
        }
        layers.push(kept);
    }
    (backtrack(&layers), issues)
}

/// Absolute spin difference on the circle.
fn spin_gap(a: f64, b: f64) -> f64 {
    let d = (a - b).rem_euclid(std::f64::consts::TAU);
    d.min(std::f64::consts::TAU - d)
}

/// Walks the backpointers of the cheapest final node. Empty layers (check
/// mode recoveries) repeat the previous configuration, matching greedy's
/// keep-the-seed behavior.
fn backtrack(layers: &[Vec<SpinNode>]) -> Vec<Vec<f64>> {
    let mut path: Vec<Option<Vec<f64>>> = vec![None; layers.len()];
    // Backtrack from the last non-empty layer; earlier disconnected
    // stretches (separated by empty layers) backtrack from their own
    // tails — each kept node chain is internally consistent.
    let mut next_pick: Option<usize> = None;
    for i in (0..layers.len()).rev() {
        let layer = &layers[i];
        if layer.is_empty() {
            next_pick = None;
            continue;
        }
        let pick = match next_pick {
            Some(p) => p,
            None => {
                layer
                    .iter()
                    .enumerate()
                    .min_by(|a, b| a.1.cost.total_cmp(&b.1.cost))
                    .expect("non-empty layer")
                    .0
            }
        };
        path[i] = Some(layer[pick].q.clone());
        next_pick = layer[pick].parent;
    }
    // Fill gaps with the nearest earlier configuration (or the nearest
    // later one at the very start), so the returned path always has one
    // entry per sample like greedy's.
    let mut filled: Vec<Vec<f64>> = Vec::with_capacity(path.len());
    let first_some = path.iter().flatten().next().cloned();
    let mut last: Option<Vec<f64>> = None;
    for entry in path {
        let q = match entry {
            Some(q) => q,
            None => last
                .clone()
                .or_else(|| first_some.clone())
                .unwrap_or_default(),
        };
        last = Some(q.clone());
        filled.push(q);
    }
    filled
}

/// Shifts full turns out of ±2π-capable joints so consecutive
/// configurations stay close — a 2π shift moves no link. Seed-continuous
/// IK rarely produces one, but a rapid reposition can; this is the safety
/// net promoted from the weld demos' `unwind_chain`.
pub fn unwind_path(model: &RobotModel, path: &mut [Vec<f64>]) {
    const TAU: f64 = std::f64::consts::TAU;
    for i in 1..path.len() {
        for (j, &ji) in model.actuated_joints.iter().enumerate() {
            let prev = path[i - 1][j];
            let cur = path[i][j];
            let k = ((prev - cur) / TAU).round();
            if k == 0.0 {
                continue;
            }
            let candidate = cur + k * TAU;
            let admissible = match model.joints[ji].limits {
                Some(l) => candidate >= l.lower && candidate <= l.upper,
                None => true,
            };
            if admissible && (candidate - prev).abs() < (cur - prev).abs() {
                path[i][j] = candidate;
            }
        }
    }
}

/// Dispatches to the configured spin strategy.
#[allow(clippy::too_many_arguments)]
fn follow_with_mode(
    scene: &Scene,
    robot: usize,
    tcp: usize,
    samples: &[ToolpathSample],
    options: &ToolpathOptions,
    stop_at_first: bool,
) -> (Vec<Vec<f64>>, Vec<PointIssue>) {
    match options.spin {
        SpinMode::Greedy => follow(scene, robot, tcp, samples, options, stop_at_first),
        SpinMode::Optimize {
            spin_step,
            beam_width,
        } => follow_optimized(
            scene,
            robot,
            tcp,
            samples,
            options,
            spin_step,
            beam_width,
            stop_at_first,
        ),
    }
}

/// Follows every sample and reports all failures without aborting: the
/// pre-teach "which points can I not reach" face check.
pub fn check_toolpath(
    scene: &Scene,
    toolpath: &Toolpath,
    robot: usize,
    tcp: usize,
    options: &ToolpathOptions,
) -> Result<ToolpathReport, ToolpathError> {
    let samples = resolve_and_sample(scene, toolpath, options)?;
    let (_, issues) = follow_with_mode(scene, robot, tcp, &samples, options, false);
    Ok(ToolpathReport {
        total_samples: samples.len(),
        issues,
    })
}

/// Bakes a toolpath into one continuous trajectory. The trajectory starts
/// at the solved first sample — walking in from the robot's current pose
/// is the caller's move (an authored approach, or a preceding motion).
pub fn plan_toolpath(
    scene: &Scene,
    toolpath: &Toolpath,
    robot: usize,
    tcp: usize,
    limits: &botrail_traj::Limits,
    options: &ToolpathOptions,
) -> Result<PlannedToolpath, ToolpathError> {
    let samples = resolve_and_sample(scene, toolpath, options)?;
    let (mut path, issues) = follow_with_mode(scene, robot, tcp, &samples, options, true);
    if let Some(issue) = issues.first() {
        return Err(ToolpathError::FollowFailed {
            sample: issue.sample,
            total: samples.len(),
            fraction: 100.0 * issue.sample as f64 / samples.len() as f64,
            move_index: issue.move_index,
            reason: format!("{:?}: {}", issue.kind, issue.detail),
        });
    }
    unwind_path(&scene.robots()[robot].model, &mut path);

    // Feed floors: the interval into each sample must take at least
    // chord / feed seconds (rapids: chord / rapid_speed when capped).
    let mut floors = vec![0.0f64; path.len().saturating_sub(1)];
    for (i, s) in samples.iter().enumerate().skip(1) {
        floors[i - 1] = match (s.feed, options.rapid_speed) {
            (Some(feed), _) => s.chord / feed,
            (None, Some(cap)) => s.chord / cap,
            (None, None) => 0.0,
        };
    }
    let timed = botrail_traj::time_parameterize_with_floors(
        &path,
        limits,
        &floors,
        &botrail_traj::TimingOptions::default(),
    )?;

    // Time each move completes: the timestamp of its last sample. A move
    // without targets inherits the previous move's end.
    let mut last_of_move: Vec<Option<usize>> = vec![None; toolpath.moves.len()];
    for (i, s) in samples.iter().enumerate() {
        last_of_move[s.move_index] = Some(i);
    }
    let mut move_ends = Vec::with_capacity(toolpath.moves.len());
    let mut t_prev = 0.0;
    for last in last_of_move {
        let t = last
            .map(|i| timed.trajectory.times[timed.waypoint_indices[i]])
            .unwrap_or(t_prev);
        move_ends.push(t);
        t_prev = t;
    }

    let cut_length = samples
        .iter()
        .filter(|s| s.feed.is_some())
        .map(|s| s.chord)
        .sum();
    let rapid_length = samples
        .iter()
        .filter(|s| s.feed.is_none())
        .map(|s| s.chord)
        .sum();
    let feed_report = feed_report(
        &samples,
        &floors,
        &timed.trajectory.times,
        &timed.waypoint_indices,
        &path,
        limits,
    );
    let sample_times = timed
        .waypoint_indices
        .iter()
        .map(|&k| timed.trajectory.times[k])
        .collect();
    Ok(PlannedToolpath {
        trajectory: timed.trajectory,
        path,
        samples,
        sample_times,
        move_ends,
        cut_length,
        rapid_length,
        feed_report,
    })
}

/// Commanded vs achieved timing over the cutting intervals, with the
/// slow stretches merged (within a move) and blamed on the joint that ran
/// closest to its velocity limit.
fn feed_report(
    samples: &[ToolpathSample],
    floors: &[f64],
    times: &[f64],
    waypoint_indices: &[usize],
    path: &[Vec<f64>],
    limits: &botrail_traj::Limits,
) -> FeedReport {
    let mut commanded = 0.0;
    let mut achieved = 0.0;
    let mut spans: Vec<SlowSpan> = Vec::new();
    let mut open: Option<(SlowSpan, f64)> = None; // (span, length)
    for (i, s) in samples.iter().enumerate().skip(1) {
        let (t0, t1) = (times[waypoint_indices[i - 1]], times[waypoint_indices[i]]);
        let (Some(feed), floor) = (s.feed, floors[i - 1]) else {
            // A rapid interval closes any open slow span.
            if let Some((span, _)) = open.take() {
                spans.push(span);
            }
            continue;
        };
        if floor <= 0.0 {
            continue; // zero-chord interval (pure dwell never happens here)
        }
        let dt = t1 - t0;
        commanded += floor;
        achieved += dt;
        let slow = dt > floor * 1.02;
        if slow {
            // The axis to blame: highest velocity utilization over the
            // interval.
            let limiting = (0..limits.velocity.len())
                .max_by(|&a, &b| {
                    let ua = (path[i][a] - path[i - 1][a]).abs() / dt / limits.velocity[a];
                    let ub = (path[i][b] - path[i - 1][b]).abs() / dt / limits.velocity[b];
                    ua.total_cmp(&ub)
                })
                .unwrap_or(0);
            match &mut open {
                Some((span, length)) if span.move_index == s.move_index => {
                    span.end = t1;
                    *length += s.chord;
                    span.achieved_feed = *length / (span.end - span.start);
                    // Keep the worst offender across the span.
                    let u_new = (path[i][limiting] - path[i - 1][limiting]).abs()
                        / dt
                        / limits.velocity[limiting];
                    let u_old = {
                        let j = span.limiting_joint;
                        (path[i][j] - path[i - 1][j]).abs() / dt / limits.velocity[j]
                    };
                    if u_new > u_old {
                        span.limiting_joint = limiting;
                    }
                }
                _ => {
                    if let Some((span, _)) = open.take() {
                        spans.push(span);
                    }
                    open = Some((
                        SlowSpan {
                            start: t0,
                            end: t1,
                            move_index: s.move_index,
                            commanded_feed: feed,
                            achieved_feed: s.chord / dt,
                            limiting_joint: limiting,
                        },
                        s.chord,
                    ));
                }
            }
        } else if let Some((span, _)) = open.take() {
            spans.push(span);
        }
    }
    if let Some((span, _)) = open.take() {
        spans.push(span);
    }
    FeedReport {
        hold_ratio: if achieved > 0.0 {
            commanded / achieved
        } else {
            1.0
        },
        commanded_cut_seconds: commanded,
        achieved_cut_seconds: achieved,
        slow_spans: spans,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use botrail_model::RobotModel;
    use std::sync::Arc;

    const ARM: &str = include_str!("../../../examples/assets/simple_arm.urdf");

    /// A bent, non-singular working pose whose Y-axis joint angles sum to
    /// zero, so the tool axis points world +Z (tool upright).
    const WORK_Q: [f64; 6] = [0.1, 0.9, -1.4, 0.5, 0.0, 0.0];

    fn scene() -> (Scene, Point3<f64>) {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(ARM).unwrap()));
        scene.set_joint_positions(WORK_Q.to_vec()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let p0 = Point3::from(scene.link_poses()[tcp].translation.vector);
        (scene, p0)
    }

    fn up() -> Unit<Vector3<f64>> {
        Unit::new_normalize(Vector3::z())
    }

    fn target(p: Point3<f64>) -> PathTarget {
        PathTarget {
            position: p,
            tool_axis: up(),
            spin: None,
        }
    }

    fn line_path(name: &str, p0: Point3<f64>, dy: f64, feed: f64) -> Toolpath {
        Toolpath {
            name: name.into(),
            frame: None,
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(p0)],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Feed(feed),
                    targets: vec![target(Point3::from(p0.coords + Vector3::new(0.0, dy, 0.0)))],
                    brush: None,
                },
            ],
        }
    }

    #[test]
    fn commanded_feed_governs_the_cut_time() {
        let (mut scene, p0) = scene();
        scene.add_toolpath(line_path("cut", p0, 0.10, 0.02));
        let planned = scene
            .plan_toolpath("cut", 0, None, &ToolpathOptions::default())
            .unwrap();
        // 10 cm at 20 mm/s = 5 s; the joints could do it far faster, so
        // the feed floor must own the timing.
        let cut_time = planned.move_ends[1] - planned.move_ends[0];
        assert!(
            (cut_time - 5.0).abs() < 0.05,
            "cut time {cut_time}, expected ~5.0"
        );
        assert!((planned.cut_length - 0.10).abs() < 1e-9);
        assert_eq!(planned.move_ends.len(), 2);
        assert!((planned.trajectory.duration() - planned.move_ends[1]).abs() < 1e-12);
        // The TCP actually holds the feed mid-cut: sample the trajectory
        // 0.5 s apart around the middle and check the distance moved.
        let tcp = scene.robot().default_tcp_link();
        let t_mid = planned.move_ends[1] * 0.5;
        let q_a = planned.trajectory.sample(t_mid);
        let q_b = planned.trajectory.sample(t_mid + 0.5);
        let p_a = scene.fk(&q_a).unwrap()[tcp].translation.vector;
        let p_b = scene.fk(&q_b).unwrap()[tcp].translation.vector;
        let speed = (p_b - p_a).norm() / 0.5;
        assert!(
            (speed - 0.02).abs() < 0.002,
            "mid-cut TCP speed {speed}, commanded 0.02"
        );
    }

    #[test]
    fn tcp_tracks_the_line_within_tolerance() {
        let (mut scene, p0) = scene();
        scene.add_toolpath(line_path("cut", p0, 0.10, 0.05));
        let planned = scene
            .plan_toolpath("cut", 0, None, &ToolpathOptions::default())
            .unwrap();
        let tcp = scene.robot().default_tcp_link();
        for (q, s) in planned.path.iter().zip(&planned.samples) {
            let pose = scene.fk(q).unwrap()[tcp];
            let err = (pose.translation.vector - s.position.coords).norm();
            assert!(err < 1e-4, "TCP off target by {err}");
            let axis = pose.rotation * Vector3::z();
            let angle = axis
                .cross(&Vector3::z())
                .norm()
                .atan2(axis.dot(&Vector3::z()));
            assert!(angle < 1e-3, "tool axis off by {angle}");
        }
    }

    #[test]
    fn part_frame_moves_re_solve_the_path() {
        let (mut scene, p0) = scene();
        scene.add_frame(
            "part",
            Isometry3::from_parts(Translation3::from(p0.coords), UnitQuaternion::identity()),
        );
        scene.add_toolpath(Toolpath {
            name: "cut".into(),
            frame: Some("part".into()),
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(Point3::origin())],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Feed(0.05),
                    targets: vec![target(Point3::new(0.0, 0.08, 0.0))],
                    brush: None,
                },
            ],
        });
        let a = scene
            .plan_toolpath("cut", 0, None, &ToolpathOptions::default())
            .unwrap();
        assert!((a.samples[0].position - p0).norm() < 1e-12);
        // Move the fixture 3 cm and re-plan: the whole path follows.
        scene.add_frame(
            "part",
            Isometry3::from_parts(
                Translation3::from(p0.coords + Vector3::new(0.03, 0.0, 0.0)),
                UnitQuaternion::identity(),
            ),
        );
        let b = scene
            .plan_toolpath("cut", 0, None, &ToolpathOptions::default())
            .unwrap();
        let shift = b.samples[0].position - a.samples[0].position;
        assert!((shift - Vector3::new(0.03, 0.0, 0.0)).norm() < 1e-12);
        // An unknown frame is a bake-time error, not a silent world path.
        scene.add_toolpath(Toolpath {
            name: "orphan".into(),
            frame: Some("missing".into()),
            moves: vec![ToolMove {
                kind: ToolMoveKind::Rapid,
                targets: vec![target(Point3::origin())],
                brush: None,
            }],
        });
        assert!(matches!(
            scene.plan_toolpath("orphan", 0, None, &ToolpathOptions::default()),
            Err(ToolpathError::UnknownFrame { .. })
        ));
    }

    #[test]
    fn check_reports_unreachable_points_without_aborting() {
        let (mut scene, p0) = scene();
        // March from the working pose straight out past the ~0.85 m reach.
        scene.add_toolpath(Toolpath {
            name: "escape".into(),
            frame: None,
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(p0)],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Feed(0.5),
                    targets: vec![target(Point3::new(1.5, 0.0, p0.z))],
                    brush: None,
                },
            ],
        });
        let report = scene
            .check_toolpath("escape", 0, None, &ToolpathOptions::default())
            .unwrap();
        assert!(!report.ok());
        assert!(
            report.issues.len() < report.total_samples,
            "the near part of the path must still solve"
        );
        assert!(report
            .issues
            .iter()
            .any(|i| i.kind == IssueKind::Unreachable));
        // plan_toolpath refuses the same path with a located error.
        let err = scene
            .plan_toolpath("escape", 0, None, &ToolpathOptions::default())
            .unwrap_err();
        assert!(matches!(err, ToolpathError::FollowFailed { .. }), "{err}");
    }

    #[test]
    fn spin_when_authored_is_honored() {
        let (mut scene, p0) = scene();
        scene.add_toolpath(Toolpath {
            name: "spun".into(),
            frame: None,
            moves: vec![ToolMove {
                kind: ToolMoveKind::Rapid,
                targets: vec![PathTarget {
                    position: p0,
                    tool_axis: up(),
                    spin: Some(0.5),
                }],
                brush: None,
            }],
        });
        let planned = scene
            .plan_toolpath("spun", 0, None, &ToolpathOptions::default())
            .unwrap();
        let tcp = scene.robot().default_tcp_link();
        let reached = scene.fk(&planned.path[0]).unwrap()[tcp];
        let want = target_pose(&p0, &up(), 0.5);
        let rel = reached.rotation.angle_to(&want.rotation);
        assert!(rel < 1e-3, "spun pose off by {rel} rad");
    }

    #[test]
    fn allowed_contact_cuts_the_stock_but_rapids_stay_strict() {
        // Arm + spindle over a *live* stock plate: cutting into it is only
        // legal through the allowed pair, and never during a rapid.
        const SPINDLE: &str = crate::testdata::SPINDLE_URDF;
        let arm = RobotModel::from_urdf_str(ARM).unwrap();
        let spindle = RobotModel::from_urdf_str(SPINDLE).unwrap();
        let robot = arm
            .attach_tool(
                &spindle,
                Some("tool0"),
                None,
                Isometry3::identity(),
                None,
                None,
            )
            .unwrap();
        let mut scene = Scene::new(Arc::new(robot));
        // Flange-down working pose (elbow under the shoulder line).
        let ref_q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(ref_q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        // Plate top 8mm under the taught tip; cutting depth dips 2mm in.
        let top = tip.z - 0.008;
        scene
            .add_obstacle(
                "plate",
                botrail_model::Geometry::Box {
                    size: Vector3::new(0.14, 0.10, 0.012),
                },
                Isometry3::translation(tip.x, tip.y, top - 0.006),
            )
            .unwrap();

        let cut = Toolpath {
            name: "cut".into(),
            frame: None,
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(Point3::new(tip.x - 0.04, tip.y, top + 0.008))],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Feed(0.01),
                    targets: vec![
                        target(Point3::new(tip.x - 0.04, tip.y, top - 0.002)),
                        target(Point3::new(tip.x + 0.04, tip.y, top - 0.002)),
                    ],
                    brush: None,
                },
            ],
        };
        scene.add_toolpath(cut.clone());

        // Without the exemption the plunge is a collision.
        let err = scene
            .plan_toolpath("cut", 0, None, &ToolpathOptions::default())
            .unwrap_err();
        assert!(matches!(err, ToolpathError::FollowFailed { .. }), "{err}");

        // Allow cutter x plate: the same path bakes, and the strict check
        // still sees the contact the allowance hides.
        let cutter = scene.robot().link_index("cutter").unwrap();
        scene
            .allow_link_obstacle_contact(0, cutter, "plate")
            .unwrap();
        let planned = scene
            .plan_toolpath("cut", 0, None, &ToolpathOptions::default())
            .unwrap();
        let mid = &planned.path[planned.path.len() / 2];
        assert!(scene.is_state_valid_for(0, mid));
        assert!(!scene.is_state_valid_strict_for(0, mid));

        // A rapid dragged through the stock is refused even with the
        // allowance in place: exemptions do not apply while not cutting.
        scene.add_toolpath(Toolpath {
            name: "dragged".into(),
            frame: None,
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(Point3::new(tip.x - 0.04, tip.y, top - 0.002))],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(Point3::new(tip.x + 0.04, tip.y, top - 0.002))],
                    brush: None,
                },
            ],
        });
        let report = scene
            .check_toolpath("dragged", 0, None, &ToolpathOptions::default())
            .unwrap();
        assert!(!report.ok());
        assert!(report
            .issues
            .iter()
            .any(|i| i.kind == IssueKind::Collision && i.detail.contains("rapid")));
    }

    #[test]
    fn feed_report_holds_gentle_feeds_and_blames_fast_ones() {
        let (mut scene, p0) = scene();
        // 20 mm/s over 10 cm: trivially held.
        scene.add_toolpath(line_path("gentle", p0, 0.10, 0.02));
        let gentle = scene
            .plan_toolpath("gentle", 0, None, &ToolpathOptions::default())
            .unwrap();
        // Not 1.0: the rest-to-rest ramps into and out of the cut are
        // honestly counted as below-feed stretches.
        assert!(
            gentle.feed_report.hold_ratio > 0.9,
            "hold {:.3}",
            gentle.feed_report.hold_ratio
        );
        // 0.5 m/s is beyond what the joints can do: the floors lose, the
        // report says so and names an axis.
        scene.add_toolpath(line_path("greedy_feed", p0, 0.10, 0.5));
        let fast = scene
            .plan_toolpath("greedy_feed", 0, None, &ToolpathOptions::default())
            .unwrap();
        let report = &fast.feed_report;
        assert!(report.hold_ratio < 0.5, "hold {:.2}", report.hold_ratio);
        assert!(report.hold_ratio < gentle.feed_report.hold_ratio);
        assert!(report.achieved_cut_seconds > report.commanded_cut_seconds);
        assert!(!report.slow_spans.is_empty());
        let span = &report.slow_spans[0];
        assert!(span.achieved_feed < span.commanded_feed);
        assert!(span.limiting_joint < 6);
    }

    /// An arm with no roll about the tool: pan + three pitch joints. Its
    /// tool axis always lies in the vertical plane through the pan
    /// direction, so a *tangential* tilt is structurally impossible — the
    /// exact case the lead/tilt tolerance exists for.
    const NO_ROLL_ARM: &str = r#"
    <robot name="noroll">
      <link name="base"/><link name="l1"/><link name="l2"/><link name="l3"/><link name="tip"/>
      <joint name="pan" type="revolute">
        <parent link="base"/><child link="l1"/>
        <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
        <limit lower="-3.1" upper="3.1" effort="1" velocity="1"/>
      </joint>
      <joint name="lift" type="revolute">
        <parent link="l1"/><child link="l2"/>
        <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
        <limit lower="-2.2" upper="2.2" effort="1" velocity="1"/>
      </joint>
      <joint name="elbow" type="revolute">
        <parent link="l2"/><child link="l3"/>
        <origin xyz="0 0 0.3"/><axis xyz="0 1 0"/>
        <limit lower="-2.6" upper="2.6" effort="1" velocity="1"/>
      </joint>
      <joint name="wrist" type="revolute">
        <parent link="l3"/><child link="tip"/>
        <origin xyz="0 0 0.25"/><axis xyz="0 1 0"/>
        <limit lower="-3.1" upper="3.1" effort="1" velocity="1"/>
      </joint>
    </robot>"#;

    #[test]
    fn axis_tolerance_admits_a_tilt_the_arm_cannot_make() {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(NO_ROLL_ARM).unwrap()));
        let work = vec![0.0, 0.6, 1.0, std::f64::consts::PI - 1.6];
        scene.set_joint_positions(work.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        // Tilt 0.2 rad toward +y — tangential to the pan plane (the arm
        // stands along +x), unreachable exactly for this wrist.
        let tilt = 0.2f64;
        let axis = Unit::new_normalize(Vector3::new(0.0, tilt.sin(), tilt.cos()));
        scene.add_toolpath(Toolpath {
            name: "tilted".into(),
            frame: None,
            moves: vec![ToolMove {
                kind: ToolMoveKind::Feed(0.01),
                targets: vec![
                    PathTarget {
                        position: Point3::from(tip),
                        tool_axis: axis,
                        spin: None,
                    },
                    PathTarget {
                        position: Point3::from(tip + Vector3::new(0.02, 0.0, 0.0)),
                        tool_axis: axis,
                        spin: None,
                    },
                ],
                brush: None,
            }],
        });
        let exact = scene
            .check_toolpath("tilted", 0, None, &ToolpathOptions::default())
            .unwrap();
        assert!(
            !exact.ok(),
            "the tangential tilt must be unreachable exactly"
        );
        assert!(exact
            .issues
            .iter()
            .all(|i| i.kind == IssueKind::Unreachable));
        let loose = scene
            .check_toolpath(
                "tilted",
                0,
                None,
                &ToolpathOptions {
                    axis_tolerance: 0.25,
                    ..ToolpathOptions::default()
                },
            )
            .unwrap();
        assert!(loose.ok(), "{:?}", loose.issues.first());
    }

    #[test]
    fn optimized_spin_is_deterministic_and_solves_what_greedy_solves() {
        let (mut scene, p0) = scene();
        scene.add_toolpath(line_path("cut", p0, 0.10, 0.02));
        let options = ToolpathOptions {
            spin: SpinMode::optimize(),
            ..ToolpathOptions::default()
        };
        let a = scene.plan_toolpath("cut", 0, None, &options).unwrap();
        let b = scene.plan_toolpath("cut", 0, None, &options).unwrap();
        assert_eq!(a.trajectory.times, b.trajectory.times);
        assert_eq!(a.trajectory.positions, b.trajectory.positions);
        // And the greedy-solvable path stays solvable under the optimizer.
        let report = scene.check_toolpath("cut", 0, None, &options).unwrap();
        assert!(report.ok(), "{:?}", report.issues.first());
    }

    #[test]
    fn planning_is_deterministic() {
        let (mut scene, p0) = scene();
        scene.add_toolpath(line_path("cut", p0, 0.10, 0.05));
        let a = scene
            .plan_toolpath("cut", 0, None, &ToolpathOptions::default())
            .unwrap();
        let b = scene
            .plan_toolpath("cut", 0, None, &ToolpathOptions::default())
            .unwrap();
        assert_eq!(a.trajectory.times, b.trajectory.times);
        assert_eq!(a.trajectory.positions, b.trajectory.positions);
    }

    #[test]
    fn msg_round_trip_preserves_the_toolpath() {
        let tp = Toolpath {
            name: "rt".into(),
            frame: Some("part".into()),
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(Point3::new(0.1, 0.2, 0.3))],
                    brush: None,
                },
                ToolMove {
                    kind: ToolMoveKind::Feed(0.01),
                    targets: vec![PathTarget {
                        position: Point3::new(0.4, 0.5, 0.6),
                        tool_axis: Unit::new_normalize(Vector3::new(0.0, 1.0, 1.0)),
                        spin: Some(0.25),
                    }],
                    brush: None,
                },
            ],
        };
        let json = serde_json::to_string(&toolpath_msg(&tp)).unwrap();
        let back = toolpath_from_msg(&serde_json::from_str(&json).unwrap()).unwrap();
        assert_eq!(back.name, tp.name);
        assert_eq!(back.frame, tp.frame);
        assert_eq!(back.moves.len(), 2);
        assert!(matches!(back.moves[1].kind, ToolMoveKind::Feed(f) if (f - 0.01).abs() < 1e-12));
        let t = &back.moves[1].targets[0];
        assert!((t.position - Point3::new(0.4, 0.5, 0.6)).norm() < 1e-12);
        assert!(t.spin == Some(0.25));
        // Bad inputs are typed errors.
        let bad = ToolpathMsg {
            name: "bad".into(),
            frame: None,
            moves: vec![ToolMoveMsg::Feed {
                feed: -1.0,
                targets: vec![],
                brush: None,
            }],
        };
        assert!(matches!(
            toolpath_from_msg(&bad),
            Err(ToolpathError::BadFeed { .. })
        ));
    }
}
