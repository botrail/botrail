//! The sequence scan engine: evaluates a [`crate::seq::Sequence`] against a
//! scene snapshot with a fixed-Δt scan loop (PLC-style: transitions are
//! checked once per tick, so step boundaries quantize to the scan period)
//! and bakes the result into a [`SequenceTimeline`].
//!
//! Everything is deterministic: motions plan with the seeded planner
//! against the world state *at their step* (obstacle poses and grasps
//! included), so the same scene + sequence always bakes the same timeline.

use botrail_traj::JointTrajectory;
use nalgebra::Isometry3;
use thiserror::Error;

use crate::seq::{
    vehicle_frame, Action, Condition, DeviceCommand, DeviceKind, SensorKind, SensorWatch, Sequence,
    Step,
};
use crate::Scene;
use botrail_collide::ObstacleCollider;
use nalgebra::Vector3;

#[derive(Debug, Error)]
pub enum SeqError {
    #[error("unknown sequence `{0}`")]
    UnknownSequence(String),
    #[error("step {}: {message}", step.map(|s| s.to_string()).unwrap_or_else(|| "-".into()))]
    Validation {
        step: Option<usize>,
        message: String,
    },
    #[error("step {step} (`{name}`): planning failed: {message}")]
    PlanFailed {
        step: usize,
        name: String,
        message: String,
    },
    #[error("step {step} (`{name}`): {message}")]
    Action {
        step: usize,
        name: String,
        message: String,
    },
    /// `forced` lists the scenario's pinned inputs (`(lane, value)`): a
    /// run that stalls under a stuck contact or an open wire says so in
    /// the same sentence as where it stalled.
    #[error(
        "timed out after {limit}s waiting in step {step} (`{name}`){}",
        forced_note(forced)
    )]
    Timeout {
        step: usize,
        name: String,
        limit: f64,
        forced: Vec<(String, bool)>,
    },
    /// The parallel-program timeout names every stuck cursor: with two or
    /// more programs, "where is everybody waiting" *is* the deadlock
    /// diagnosis (a gate watching a signal nobody sets shows up here).
    #[error(
        "timed out after {limit}s; programs waiting at: {at}{}",
        forced_note(forced)
    )]
    ProgramsTimeout {
        at: String,
        limit: f64,
        forced: Vec<(String, bool)>,
    },
    #[error(
        "step {step} (`{name}`): >{limit} instantaneous steps in one scan tick (immediate loop?)"
    )]
    ImmediateLoop {
        step: usize,
        name: String,
        limit: usize,
    },
    /// Two robots met mid-cycle. Plans freeze the other robots at their
    /// start-of-motion pose, so this is not a planning bug: the cycle needs
    /// an interlock (§design-multi-robot 3.2) making one arm wait.
    #[error(
        "robots `{a}` and `{b}` collide at t = {t:.3}s ({link_a} × {link_b}); \
         add an interlock (zone sensor / robot_done) so one waits for the other"
    )]
    RobotCollision {
        t: f64,
        a: String,
        b: String,
        link_a: String,
        link_b: String,
    },
    /// A travelling vehicle's body met the environment. Travel is authored,
    /// not planned, so this is the aisle check failing: widen the aisle,
    /// move the shelf, or re-teach the path.
    #[error(
        "vehicle `{vehicle}` collides with `{obstacle}` at t = {t:.3}s \
         (body part `{body}`); widen the aisle or re-teach the path"
    )]
    VehicleCollision {
        t: f64,
        vehicle: String,
        body: String,
        obstacle: String,
    },
    /// A robot riding a travelling vehicle — its legs, its arm, what it
    /// holds — met the environment. The vehicle's aisle check, for the part
    /// of the machine that is a robot rather than a body.
    #[error(
        "robot `{robot}` riding `{vehicle}` collides with `{obstacle}` at t = {t:.3}s \
         (`{part}`); widen the aisle or re-teach the path"
    )]
    RiderCollision {
        t: f64,
        vehicle: String,
        robot: String,
        part: String,
        obstacle: String,
    },
    /// A walking leg could not reach its footfall: the vehicle's rates ask
    /// more of the leg than its geometry gives. Authoring, like the aisle
    /// check — a slower vehicle, a shorter gait period, a lower stance.
    #[error(
        "robot `{robot}`: leg `{leg}` cannot reach its footfall at t = {t:.3}s \
         — {reach:.3} m from the hip to a foot {drop:+.3} m below the body, on a \
         {stride:.3} m stride ({detail}); lower the vehicle speed, shorten the \
         gait period, or take the grade in smaller steps"
    )]
    GaitReach {
        t: f64,
        robot: String,
        leg: String,
        stride: f64,
        /// Hip-to-target distance the leg was asked for.
        reach: f64,
        /// How far the target sat below the body's own plane (negative is
        /// above it) — what a grade and a riser cost the leg.
        drop: f64,
        /// How far the solve fell short.
        detail: String,
    },
    /// A leg would step higher than the machine's declared ability.
    #[error(
        "robot `{robot}`: leg `{leg}` would step {rise:.3} m at ({x:.3}, {y:.3}, \
         {z:.3}) on the walk dispatched at t = {t:.3}s, over the gait's max_step \
         {max_step:.3} m; smaller risers, or a machine that climbs higher"
    )]
    StepHeight {
        t: f64,
        robot: String,
        leg: String,
        /// Signed rise of the offending step.
        rise: f64,
        max_step: f64,
        /// Where the step would land.
        x: f64,
        y: f64,
        z: f64,
    },
    /// A foothold landed too close to the edge of its tread.
    #[error(
        "robot `{robot}`: leg `{leg}` lands {margin:.3} m inside the edge of \
         `{obstacle}` at t = {t:.3}s (the foot needs {need:.3} m); move the path, \
         the stride or the stairs so the foot is on the tread"
    )]
    FootOverhang {
        t: f64,
        robot: String,
        leg: String,
        obstacle: String,
        /// How far inside the tread's top face the foothold sits.
        margin: f64,
        /// The margin the foot radius needs.
        need: f64,
    },
    /// A travelling vehicle's body met a robot that is not its passenger.
    #[error(
        "vehicle `{vehicle}`: `{body}` hits robot `{robot}` (link `{link}`) at \
         t = {t:.3}s while driving; hold the robot clear with an interlock or \
         re-teach the path"
    )]
    VehicleRobotCollision {
        t: f64,
        vehicle: String,
        body: String,
        robot: String,
        link: String,
    },
}

/// ` — forced: a=false, b=true` for a timeout under faults, empty otherwise.
fn forced_note(forced: &[(String, bool)]) -> String {
    if forced.is_empty() {
        return String::new();
    }
    let list: Vec<String> = forced
        .iter()
        .map(|(name, value)| format!("{name}={value}"))
        .collect();
    format!(" — forced: {}", list.join(", "))
}

#[derive(Debug, Clone)]
pub struct RolloutOptions {
    /// Scan period in seconds — transition timing quantizes to this.
    pub dt: f64,
    /// Hard wall-clock cap; exceeded waits are authoring errors.
    pub max_duration: f64,
    pub plan: botrail_plan::PlanOptions,
    /// Toolpath follow/timing options for [`crate::seq::Action::StartToolpath`].
    pub toolpath: crate::toolpath::ToolpathOptions,
    /// Instantaneous steps allowed within one scan tick.
    pub immediate_chain_limit: usize,
}

impl Default for RolloutOptions {
    fn default() -> Self {
        RolloutOptions {
            dt: 0.01,
            max_duration: 120.0,
            plan: botrail_plan::PlanOptions::default(),
            toolpath: crate::toolpath::ToolpathOptions::default(),
            immediate_chain_limit: 64,
        }
    }
}

/// One step's active interval on the baked timeline.
#[derive(Debug, Clone)]
pub struct StepSpan {
    pub name: String,
    pub start: f64,
    pub end: f64,
    /// Owning sequence — for a robot move span, the sequence whose step
    /// started the move. Always the bare name; `name` alone gets the
    /// `"{sequence}/"` display prefix in multi-program bakes, and step
    /// names repeat freely, so structural consumers key on these fields.
    pub sequence: String,
    /// Flat-step index within `sequence` (the [`flatten`] pre-order).
    pub step: usize,
}

/// Where a signal lane comes from. Set where the lane is built (the
/// rollout, or a post-bake synthesis) so consumers — the studio's timing
/// chart folds device lanes away, the I/O map classifies inputs — never
/// have to guess it back from the name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LaneKind {
    /// An internal relay (`define_signal`), or a lane synthesized after
    /// the bake under a signal's name.
    Signal,
    /// A sensor's read-only input.
    Sensor,
    /// A device's running / moving output.
    Device,
}

impl LaneKind {
    pub fn as_str(self) -> &'static str {
        match self {
            LaneKind::Signal => "signal",
            LaneKind::Sensor => "sensor",
            LaneKind::Device => "device",
        }
    }
}

/// A boolean signal as a step function: `(time, new_value)` edges,
/// starting with `(0, initial)`.
#[derive(Debug, Clone)]
pub struct BoolTrack {
    pub name: String,
    pub edges: Vec<(f64, bool)>,
    pub kind: LaneKind,
}

impl BoolTrack {
    pub fn value_at(&self, t: f64) -> bool {
        self.edges
            .iter()
            .take_while(|(edge_t, _)| *edge_t <= t + 1e-12)
            .last()
            .map(|(_, v)| *v)
            .unwrap_or(false)
    }
}

/// Piecewise world motion of one tracked object. Spans tile `[0, duration]`.
#[derive(Debug, Clone)]
pub enum TrackSpan {
    /// At rest at a fixed world pose.
    Hold {
        t0: f64,
        t1: f64,
        pose: Isometry3<f64>,
    },
    /// Rigidly attached to a robot: `world = link_pose(t) ∘ offset`.
    Follow {
        t0: f64,
        t1: f64,
        /// Carrying robot (scene index).
        robot: usize,
        link: usize,
        offset: Isometry3<f64>,
    },
    /// Held, and out of sight: waiting in a magazine, or consumed at the
    /// end of a line. The pose is still defined (collision and queries see
    /// it), but nothing should draw it — a carrier queueing off-line is
    /// stock, not scenery, and watching it teleport onto the belt is the
    /// one thing that gives a recirculating line away.
    Stowed {
        t0: f64,
        t1: f64,
        pose: Isometry3<f64>,
    },
    /// Conveyed at constant velocity from `from` (rotation unchanged).
    Linear {
        t0: f64,
        t1: f64,
        from: Isometry3<f64>,
        velocity: nalgebra::Vector3<f64>,
    },
    /// Turning in place with a vehicle: rotated about the vertical axis
    /// through `center` at constant `omega` (rad/s, +Z right-hand). The
    /// first rotating rigid motion a device produces — closed form, so any
    /// resample rate is exact.
    Pivot {
        t0: f64,
        t1: f64,
        from: Isometry3<f64>,
        /// Floor point of the pivot axis (its z is irrelevant).
        center: nalgebra::Point3<f64>,
        omega: f64,
    },
}

/// Advances `from` by one vehicle motion piece lasting `dt`.
pub(crate) fn apply_piece(from: &Isometry3<f64>, piece: &VehiclePiece, dt: f64) -> Isometry3<f64> {
    match piece {
        VehiclePiece::Lin { velocity } => {
            let mut next = *from;
            next.translation.vector += velocity * dt;
            next
        }
        VehiclePiece::Piv { center, omega } => pivot_pose(from, center, omega * dt),
    }
}

/// Appends one vehicle motion piece to a span list, merging with the open
/// span when it continues it — so a whole leg bakes to a single span.
/// Seeds a leading rest span when the list is empty and the motion starts
/// after t = 0, keeping the tiling from zero.
fn push_vehicle_span(
    spans: &mut Vec<TrackSpan>,
    from: Isometry3<f64>,
    tau0: f64,
    tau1: f64,
    piece: &VehiclePiece,
) {
    if tau1 - tau0 < 1e-12 {
        return;
    }
    if spans.is_empty() && tau0 > 0.0 {
        spans.push(TrackSpan::Hold {
            t0: 0.0,
            t1: tau0,
            pose: from,
        });
    }
    let merged = match (spans.last_mut(), piece) {
        (Some(TrackSpan::Linear { t1, velocity, .. }), VehiclePiece::Lin { velocity: v })
            if (*velocity - v).norm() < 1e-12 && (*t1 - tau0).abs() < 1e-9 =>
        {
            *t1 = tau1;
            true
        }
        (
            Some(TrackSpan::Pivot {
                t1, center, omega, ..
            }),
            VehiclePiece::Piv {
                center: c,
                omega: o,
            },
        ) if (center.coords - c.coords).norm() < 1e-12
            && (*omega - o).abs() < 1e-12
            && (*t1 - tau0).abs() < 1e-9 =>
        {
            *t1 = tau1;
            true
        }
        _ => false,
    };
    if merged {
        return;
    }
    // A gap means the vehicle stood still in between. Stretching the
    // previous span across it would keep *travelling* through the stop —
    // the rest has to be recorded as a rest.
    if let Some(open) = spans.last() {
        let (_, end) = open.range();
        if end < tau0 - 1e-12 {
            spans.push(TrackSpan::Hold {
                t0: end,
                t1: tau0,
                pose: from,
            });
        }
    }
    spans.push(match piece {
        VehiclePiece::Lin { velocity } => TrackSpan::Linear {
            t0: tau0,
            t1: tau1,
            from,
            velocity: *velocity,
        },
        VehiclePiece::Piv { center, omega } => TrackSpan::Pivot {
            t0: tau0,
            t1: tau1,
            from,
            center: *center,
            omega: *omega,
        },
    });
}

/// Rigid rotation of `from` about the vertical line through `center` by
/// `phi` radians.
pub(crate) fn pivot_pose(
    from: &Isometry3<f64>,
    center: &nalgebra::Point3<f64>,
    phi: f64,
) -> Isometry3<f64> {
    let rot = nalgebra::UnitQuaternion::from_axis_angle(&Vector3::z_axis(), phi);
    let arm = from.translation.vector - center.coords;
    Isometry3::from_parts(
        nalgebra::Translation3::from(center.coords + rot * arm),
        rot * from.rotation,
    )
}

impl TrackSpan {
    fn end_mut(&mut self) -> &mut f64 {
        match self {
            TrackSpan::Hold { t1, .. }
            | TrackSpan::Stowed { t1, .. }
            | TrackSpan::Follow { t1, .. }
            | TrackSpan::Linear { t1, .. }
            | TrackSpan::Pivot { t1, .. } => t1,
        }
    }

    pub fn range(&self) -> (f64, f64) {
        match self {
            TrackSpan::Hold { t0, t1, .. }
            | TrackSpan::Stowed { t0, t1, .. }
            | TrackSpan::Follow { t0, t1, .. }
            | TrackSpan::Linear { t0, t1, .. }
            | TrackSpan::Pivot { t0, t1, .. } => (*t0, *t1),
        }
    }
}

#[derive(Debug, Clone)]
pub struct ObjectTrack {
    /// Obstacle name.
    pub name: String,
    pub spans: Vec<TrackSpan>,
}

/// One robot's share of a baked timeline.
#[derive(Debug, Clone)]
pub struct RobotTrack {
    /// Instance name.
    pub name: String,
    /// Whole-sequence joint track (holds still during waits).
    pub trajectory: JointTrajectory,
    /// Intervals a motion/ramp drove this robot (timeline robot lanes),
    /// labelled with the motion name (or `ramp`).
    pub moves: Vec<StepSpan>,
    /// The sparse planned path of every motion/ramp the rollout started on
    /// this robot, in firing order — what vendor script export lowers to
    /// move commands (the dense `trajectory` is for playback; controllers
    /// re-time sparse targets themselves).
    pub planned: Vec<PlannedMove>,
    /// Where the robot's base was over time — `Some` only for a robot that
    /// rides a vehicle. Spans tile `[0, duration]` in the same vocabulary
    /// the load uses, because it is the same rigid motion.
    pub base: Option<Vec<TrackSpan>>,
    /// Every step the robot's legs took, in landing order — empty unless
    /// it walks (a gait on its mount). The joints carry the legs' motion
    /// like any other; this is the plan they followed.
    pub footfalls: Vec<crate::gait::Footfall>,
    /// The body's bob and lean over each walk, composed onto `base` by
    /// [`SequenceTimeline::base_pose`]. Empty unless the gait sways.
    pub sway: Vec<crate::gait::BodySway>,
    /// How the body tilted onto the grade over each walk, composed onto
    /// `base` the same way. Empty on the level.
    pub pitch: Vec<crate::gait::BodyPitch>,
    /// How the body rode over the guide line — the steps under it, not the
    /// straight route. Empty on flat ground.
    pub rise: Vec<crate::gait::BodyRise>,
}

/// One motion/ramp as the rollout planned it: which program step fired it
/// and the sparse joint-space path it committed to.
#[derive(Debug, Clone)]
pub struct PlannedMove {
    /// Owning sequence (programs interleave on one timeline).
    pub sequence: String,
    /// Step index within that sequence.
    pub step: usize,
    /// Motion name, or `None` for a joint ramp.
    pub motion: Option<String>,
    /// Sparse path, both endpoints included (a ramp is one two-waypoint
    /// joint segment).
    pub segments: Vec<crate::motion::PlannedSegment>,
    /// Rest-to-rest duration the rollout used — for a ramp this is the
    /// authored duration, which is what sets its export speed.
    pub duration: f64,
    /// Feed adherence of a toolpath move (`None` for motions and ramps).
    pub feed_report: Option<crate::toolpath::FeedReport>,
    /// Timeline intervals during which a toolpath move was in a spraying
    /// stroke — the program's own process trigger, as opposed to its
    /// rapids, its gun-off feed moves, and the approach planned in from
    /// wherever the robot stood. Empty for motions and ramps. What a film
    /// integrator or a standoff check gates on alongside the PLC's enable
    /// signal: a gun opened by the sequence at the same time the toolpath
    /// starts must not spray the part on the way in.
    pub process_spans: Vec<ProcessSpan>,
}

/// One stretch of a toolpath move with the process on: which brush the
/// stroke ran with (`None` in a toolpath that names none — then whatever
/// applicator the integrator is handed).
#[derive(Debug, Clone, PartialEq)]
pub struct ProcessSpan {
    pub start: f64,
    pub end: f64,
    pub brush: Option<String>,
}

/// The baked result of a sequence rollout — what playback, USD export, and
/// the timing chart consume. `duration` is the cycle time.
#[derive(Debug, Clone)]
pub struct SequenceTimeline {
    pub duration: f64,
    /// Names of the sequences this timeline was rolled from, in scan
    /// order. Script export resolves its default program here.
    pub sequences: Vec<String>,
    /// The scenario this bake ran under; `None` is the unmodified scene
    /// (`baseline`). Self-description for result sets, captions, and
    /// export naming.
    pub scenario: Option<String>,
    /// One track per robot, in scene order.
    pub robots: Vec<RobotTrack>,
    /// Objects that were grasped at some point (everything else is static).
    pub objects: Vec<ObjectTrack>,
    /// One pose track per vehicle that drove: its reference frame over
    /// time. The body obstacles have their own object tracks — this is
    /// the frame itself, which is what places mounted sensors (and any
    /// other vehicle-frame geometry) during playback.
    pub vehicles: Vec<ObjectTrack>,
    pub signals: Vec<BoolTrack>,
    pub step_spans: Vec<StepSpan>,
    /// Every selection divergence this rollout resolved, in resolution
    /// order — the path the bake took. Script export replays these to
    /// walk the same arms; the spans of untaken arms simply don't exist.
    pub branches: Vec<BranchTaken>,
}

/// One resolved selection divergence: which arm `sequence` took at the
/// named branching step.
#[derive(Debug, Clone)]
pub struct BranchTaken {
    pub sequence: String,
    /// The branching step's display name.
    pub step: String,
    /// The branching step's pre-order ordinal within its sequence — the
    /// same numbering [`crate::seq::enumerate_selects`] assigns, so
    /// coverage and script-export donor lookup match decisions to
    /// authored steps mechanically (display names may repeat).
    pub select: usize,
    /// Index into that step's arms, authored order.
    pub arm: usize,
}

/// One branch arm that no supplied timeline took — an authored path the
/// scenario set never exercised, and therefore never verified.
#[derive(Debug, Clone)]
pub struct UncoveredArm {
    pub sequence: String,
    /// Display name of the branching step.
    pub step: String,
    /// Pre-order ordinal of the branching step within its sequence
    /// (matches [`BranchTaken::select`]).
    pub select: usize,
    /// Arm index, authored order.
    pub arm: usize,
    /// The arm's guard, in the authoring vocabulary
    /// (`bt.seq.signal("part_ng", True)`).
    pub condition: String,
}

impl std::fmt::Display for UncoveredArm {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "`{}` step `{}` arm {} (when {})",
            self.sequence,
            self.step,
            self.arm + 1,
            self.condition
        )
    }
}

/// Branch coverage over a set of baked timelines: every authored arm —
/// nested ones included — minus the union of what the runs took. An
/// empty result means every path was exercised; anything else is a path
/// the scenario set never verified, named in enumeration order the way
/// it was written. This is what makes "all arms covered" a CI-assertable
/// number: each timeline is deterministic, so so is the report.
///
/// `scene` supplies the authored sequences (scenario deltas never touch
/// them, so any snapshot — or the live scene — serves). Every timeline
/// must come from the same rolled sequence set.
pub fn arm_coverage(
    scene: &Scene,
    timelines: &[&SequenceTimeline],
) -> Result<Vec<UncoveredArm>, String> {
    let Some(first) = timelines.first() else {
        return Err("no timelines to measure coverage over".to_string());
    };
    for timeline in timelines {
        if timeline.sequences != first.sequences {
            return Err(format!(
                "timelines were rolled from different sequence sets (`{}` vs `{}`) — \
                 coverage is per set",
                first.sequences.join(" + "),
                timeline.sequences.join(" + "),
            ));
        }
    }
    let covered: std::collections::HashSet<(&str, usize, usize)> = timelines
        .iter()
        .flat_map(|timeline| {
            timeline
                .branches
                .iter()
                .map(|b| (b.sequence.as_str(), b.select, b.arm))
        })
        .collect();
    let mut uncovered = Vec::new();
    for name in &first.sequences {
        let sequence = scene
            .sequence(name)
            .ok_or_else(|| format!("the scene no longer holds sequence `{name}`"))?;
        for (ordinal, step) in crate::seq::enumerate_selects(&sequence.steps)
            .iter()
            .enumerate()
        {
            for (arm, select_arm) in step.select.iter().enumerate() {
                if !covered.contains(&(name.as_str(), ordinal, arm)) {
                    uncovered.push(UncoveredArm {
                        sequence: name.clone(),
                        step: step.name.clone(),
                        select: ordinal,
                        arm,
                        condition: crate::project::py_condition(&crate::wire::seq_condition_msg(
                            &select_arm.condition,
                        )),
                    });
                }
            }
        }
    }
    Ok(uncovered)
}

impl SequenceTimeline {
    /// The intervals robot `robot`'s toolpath moves spent spraying — the
    /// program's own process trigger, with the brush each ran — in time
    /// order, adjacent same-brush spans merged. `None` when the track ran
    /// no toolpath at all (a hand-built or motion-only timeline has no
    /// program to say when the process was on, so a consumer treats the
    /// whole timeline as process time).
    pub fn process_spans(&self, robot: usize) -> Option<Vec<ProcessSpan>> {
        let track = self.robots.get(robot)?;
        if !track.planned.iter().any(|m| m.feed_report.is_some()) {
            return None;
        }
        let mut spans: Vec<ProcessSpan> = track
            .planned
            .iter()
            .flat_map(|m| m.process_spans.iter().cloned())
            .collect();
        spans.sort_by(|a, b| {
            a.start
                .partial_cmp(&b.start)
                .unwrap_or(std::cmp::Ordering::Equal)
        });
        let mut merged: Vec<ProcessSpan> = Vec::with_capacity(spans.len());
        for span in spans {
            match merged.last_mut() {
                Some(last) if span.start <= last.end + 1e-9 && last.brush == span.brush => {
                    last.end = last.end.max(span.end)
                }
                _ => merged.push(span),
            }
        }
        Some(merged)
    }

    /// Whether a tracked object should be drawn at `t`. False only while it
    /// is stowed — waiting in a magazine or taken off the line.
    pub fn object_visible(track: &ObjectTrack, t: f64) -> bool {
        let span = track
            .spans
            .iter()
            .find(|s| {
                let (t0, t1) = s.range();
                t >= t0 - 1e-9 && t <= t1 + 1e-9
            })
            .or(track.spans.last());
        !matches!(span, Some(TrackSpan::Stowed { .. }))
    }

    /// World pose of a tracked object at `t`; `link_poses[robot]` must be
    /// the FK world poses of that robot at the same instant.
    pub fn object_pose(
        track: &ObjectTrack,
        link_poses: &[Vec<Isometry3<f64>>],
        t: f64,
    ) -> Option<Isometry3<f64>> {
        Self::span_pose(&track.spans, link_poses, t)
    }

    /// Where a mounted robot's base was at `t`: the rigid ride on its
    /// vehicle, plus the body's sway while it walks.
    pub fn base_pose(track: &RobotTrack, t: f64) -> Option<Isometry3<f64>> {
        let rigid = Self::span_pose(track.base.as_deref()?, &[], t)?;
        // The ride is a world lift (the body climbs the steps); the tilt
        // and the sway are the body's own.
        let rise = nalgebra::Translation3::new(0.0, 0.0, crate::gait::rise_at(&track.rise, t));
        Some(
            rise * rigid
                * crate::gait::pitch_offset(&track.pitch, t)
                * crate::gait::sway_offset(&track.sway, t),
        )
    }

    /// Pose from a span list at `t` (spans tile `[0, duration]`; the last
    /// one extends past its end).
    pub fn span_pose(
        spans: &[TrackSpan],
        link_poses: &[Vec<Isometry3<f64>>],
        t: f64,
    ) -> Option<Isometry3<f64>> {
        let span = spans
            .iter()
            .find(|s| {
                let (t0, t1) = s.range();
                t >= t0 - 1e-9 && t <= t1 + 1e-9
            })
            .or(spans.last())?;
        Some(match span {
            TrackSpan::Hold { pose, .. } | TrackSpan::Stowed { pose, .. } => *pose,
            TrackSpan::Follow {
                robot,
                link,
                offset,
                ..
            } => link_poses[*robot][*link] * offset,
            TrackSpan::Linear {
                t0,
                t1,
                from,
                velocity,
            } => {
                let dt = t.clamp(*t0, *t1) - t0;
                let mut pose = *from;
                pose.translation.vector += velocity * dt;
                pose
            }
            TrackSpan::Pivot {
                t0,
                t1,
                from,
                center,
                omega,
            } => pivot_pose(from, center, omega * (t.clamp(*t0, *t1) - t0)),
        })
    }

    /// The track of the robot instance named `name`.
    pub fn robot_track(&self, name: &str) -> Option<&RobotTrack> {
        self.robots.iter().find(|r| r.name == name)
    }

    /// Seconds `name` spent in motion, with overlapping move intervals
    /// merged (a robot driven by a motion and a ramp in the same breath
    /// is busy once, not twice).
    pub fn busy_seconds(&self, name: &str) -> Option<f64> {
        let spans = crate::handshake::robot_busy(self, name)?;
        Some(spans.iter().map(|(s, e)| e - s).sum())
    }

    /// Fraction of the cycle `name` spent moving, 0..1.
    ///
    /// The line-balancing number: the bottleneck station is the one whose
    /// arms sit near 1, and moving a spot off it is the edit whose effect
    /// on takt this figure predicts. `None` for an unknown instance; 0
    /// for a zero-length cycle.
    pub fn utilization(&self, name: &str) -> Option<f64> {
        let busy = self.busy_seconds(name)?;
        Some(if self.duration > 0.0 {
            busy / self.duration
        } else {
            0.0
        })
    }

    /// Seconds the vehicle `name` spent off its starting ground: every
    /// span that moves it, plus every hold above the altitude it started
    /// at. For an aerial machine this is the motor-on time a declared
    /// flight time must cover — hover at a station counts, waiting on the
    /// pad does not; for a ground machine it is simply its driving time.
    /// The spans are closed form, so the figure is exact, not sampled.
    /// `None` for a vehicle this timeline never drove (its ride is 0 s
    /// only if the vehicle exists — the caller knows the scene).
    pub fn vehicle_airborne(&self, name: &str) -> Option<f64> {
        let track = self.vehicles.iter().find(|v| v.name == name)?;
        let ground = track.spans.first().map(|span| match span {
            TrackSpan::Hold { pose, .. }
            | TrackSpan::Stowed { pose, .. }
            | TrackSpan::Linear { from: pose, .. }
            | TrackSpan::Pivot { from: pose, .. } => pose.translation.z,
            TrackSpan::Follow { .. } => 0.0,
        })?;
        let airborne = |z: f64| z > ground + 1e-6;
        Some(
            track
                .spans
                .iter()
                .map(|span| match span {
                    TrackSpan::Hold { t0, t1, pose } | TrackSpan::Stowed { t0, t1, pose } => {
                        if airborne(pose.translation.z) {
                            t1 - t0
                        } else {
                            0.0
                        }
                    }
                    // A vehicle frame never follows a robot; count it as
                    // motion should that ever change.
                    TrackSpan::Follow { t0, t1, .. } => t1 - t0,
                    TrackSpan::Linear {
                        t0,
                        t1,
                        from,
                        velocity,
                    } => {
                        if velocity.norm() > 1e-9 || airborne(from.translation.z) {
                            t1 - t0
                        } else {
                            0.0
                        }
                    }
                    TrackSpan::Pivot {
                        t0,
                        t1,
                        from,
                        omega,
                        ..
                    } => {
                        if omega.abs() > 1e-9 || airborne(from.translation.z) {
                            t1 - t0
                        } else {
                            0.0
                        }
                    }
                })
                .sum(),
        )
    }
}

impl Scene {
    /// Runs the scan loop for `name` against a clone of this scene (the
    /// live scene is untouched). Planned motions are time-parameterized
    /// with their owning robot's URDF-derived limits
    /// ([`crate::motion::traj_limits`]).
    pub fn simulate_sequence(
        &self,
        name: &str,
        options: &RolloutOptions,
    ) -> Result<SequenceTimeline, SeqError> {
        self.simulate_sequences(&[name], options)
    }

    /// Runs several sequences *concurrently* over one shared world — one
    /// scan-loop tick advances every program, in the order given here.
    ///
    /// This is the PLC picture of a line: one POU per station plus a
    /// transfer POU, each a plain serial SFC, synchronized only through
    /// signals and sensors. Concurrency changes nothing about determinism:
    /// programs are scanned in declaration order every tick, so the bake
    /// stays bit-identical run to run.
    ///
    /// Every robot, device, and written signal must be driven by at most
    /// one of the programs (validated up front). Two programs commanding
    /// one robot is not a scheduling problem to referee at runtime — it is
    /// an authoring error, the same as two PLC programs writing one coil.
    pub fn simulate_sequences(
        &self,
        names: &[&str],
        options: &RolloutOptions,
    ) -> Result<SequenceTimeline, SeqError> {
        if names.is_empty() {
            return Err(SeqError::Validation {
                step: None,
                message: "no sequences to simulate".to_string(),
            });
        }
        let mut sequences = Vec::with_capacity(names.len());
        for name in names {
            if sequences.iter().any(|s: &Sequence| &s.name == name) {
                return Err(SeqError::Validation {
                    step: None,
                    message: format!("sequence `{name}` is listed twice"),
                });
            }
            let sequence = self
                .sequence(name)
                .ok_or_else(|| SeqError::UnknownSequence(name.to_string()))?
                .clone();
            self.validate_sequence(&sequence)
                .map_err(|(step, message)| SeqError::Validation {
                    step,
                    message: if names.len() == 1 {
                        message
                    } else {
                        format!("sequence `{name}`: {message}")
                    },
                })?;
            sequences.push(sequence);
        }
        if sequences.len() > 1 {
            self.validate_program_ownership(&sequences)
                .map_err(|message| SeqError::Validation {
                    step: None,
                    message,
                })?;
        }
        Rollout::new(self.clone(), sequences, options.clone()).run()
    }

    /// [`simulate_sequences`](Self::simulate_sequences) under a named
    /// scenario: the deltas are applied to the rollout's snapshot — the
    /// live scene is never touched — and the resulting timeline carries
    /// the scenario name. `None` and `"baseline"` both mean the scene as
    /// it stands.
    pub fn simulate_sequences_scenario(
        &self,
        names: &[&str],
        scenario: Option<&str>,
        options: &RolloutOptions,
    ) -> Result<SequenceTimeline, SeqError> {
        let Some(scenario) = scenario.filter(|s| *s != crate::seq::BASELINE_SCENARIO) else {
            return self.simulate_sequences(names, options);
        };
        let mut snapshot = self.clone();
        snapshot
            .apply_scenario(scenario)
            .map_err(|e| SeqError::Validation {
                step: None,
                message: e.to_string(),
            })?;
        let mut timeline = snapshot.simulate_sequences(names, options)?;
        timeline.scenario = Some(scenario.to_string());
        Ok(timeline)
    }
}

/// Per-robot scan-loop state: the commanded joints, the in-flight move,
/// the tracking latch, and the accumulating baked track.
struct RobotRuntime {
    /// Commanded joints (what the robot actually does).
    q: Vec<f64>,
    /// Joints as the motion/ramp asks for them, before any tracking offset.
    /// Equal to `q` unless a track is active.
    q_nom: Vec<f64>,
    /// Previous tick's nominal, so a tracked solve can warm-start from the
    /// previous *command* plus the nominal increment.
    q_nom_prev: Vec<f64>,
    /// The in-flight motion/ramp, for per-tick joint sampling. One slot —
    /// matching the "one driver per robot per step" rule.
    active: Option<ActiveMove>,
    /// Conveyor tracking: the latched part and the offset it has built up.
    tracking: Option<TrackLatch>,
    // Accumulating baked track.
    times: Vec<f64>,
    positions: Vec<Vec<f64>>,
    velocities: Vec<Vec<f64>>,
    /// Intervals a move drove this robot (the timeline's robot lanes).
    moves: Vec<StepSpan>,
    /// Sparse planned paths, in firing order (script export).
    planned: Vec<PlannedMove>,
    /// Base motion, for a robot riding a vehicle.
    base: Option<Vec<TrackSpan>>,
    /// The legs, for a robot that walks its vehicle.
    gait: Option<GaitRuntime>,
    /// The propellers, for a machine whose mount declares them.
    spin: Option<SpinRuntime>,
}

impl RobotRuntime {
    fn append_waypoint(&mut self, t: f64, q: Vec<f64>, v: Vec<f64>) {
        let last = *self.times.last().expect("seeded with t = 0");
        if t <= last + 1e-9 {
            return;
        }
        self.times.push(t);
        self.positions.push(q);
        self.velocities.push(v);
    }

    /// Drops every baked waypoint after `t`. A gait bakes tick by tick,
    /// and a move's pre-baked future (a ramp's end point) would block that
    /// — the move is re-baked from its own samples when the gait lets go.
    fn truncate_after(&mut self, t: f64) {
        while self.times.last().is_some_and(|last| *last > t + 1e-9) {
            self.times.pop();
            self.positions.pop();
            self.velocities.pop();
        }
    }

    /// Re-bakes what the in-flight move still has to do after `t`.
    fn rebake_active_tail(&mut self, t: f64) {
        let Some(active) = &self.active else { return };
        let tail: Vec<(f64, Vec<f64>, Vec<f64>)> = match active {
            ActiveMove::Ramp {
                start,
                duration,
                to,
                ..
            } => vec![(start + duration, to.clone(), vec![0.0; to.len()])],
            ActiveMove::Traj { start, traj } => (0..traj.times.len())
                .map(|i| {
                    (
                        start + traj.times[i],
                        traj.positions[i].clone(),
                        traj.velocities[i].clone(),
                    )
                })
                .collect(),
        };
        for (time, q, v) in tail {
            if time > t + 1e-9 {
                self.append_waypoint(time, q, v);
            }
        }
    }

    /// Whether the gait is driving the legs right now (walking, or
    /// settling after arrival).
    fn walking(&self) -> bool {
        self.gait.as_ref().is_some_and(|g| g.plan.is_some())
    }
}

/// A walking robot's legs: the gait as resolved against its model, the
/// plan of the drive in progress, and every footfall taken so far.
/// A spinning mount's resolved state — a multirotor's propellers.
/// Presentation only: the joints advance at their authored rates while the
/// vehicle is off its starting ground (or moving at all), and each tick is
/// baked so studio and USD replay the spin. No verdict reads the phase —
/// the checking shape is the swept solid (design-drone.md §3.4).
#[derive(Clone)]
struct SpinRuntime {
    /// The ridden vehicle's device name.
    device: String,
    /// The ground the machine starts parked on — the same reference
    /// altitude `vehicle_airborne` measures against.
    ground: f64,
    /// `(joint index, signed rad/s)`.
    joints: Vec<(usize, f64)>,
}

struct GaitRuntime {
    gait: crate::gait::ResolvedGait,
    offset: Isometry3<f64>,
    /// `Some` from dispatch until the last foot has settled.
    plan: Option<crate::gait::GaitPlan>,
    history: Vec<crate::gait::Footfall>,
    /// The sway currently composed onto the world base (identity when
    /// standing): the vehicle drives the base *under* it, so it is undone
    /// before the rigid ride is advanced and recorded.
    sway: Isometry3<f64>,
    /// Every walk's sway, for the timeline.
    sways: Vec<crate::gait::BodySway>,
    /// The tilt currently composed onto the world base (identity on the
    /// level), undone with the sway before the rigid ride is advanced.
    pitch: Isometry3<f64>,
    /// Every walk's pitch, for the timeline.
    pitches: Vec<crate::gait::BodyPitch>,
    /// The lift currently composed onto the world base (0 on the flat).
    rise: f64,
    /// Every walk's ride, for the timeline.
    rises: Vec<crate::gait::BodyRise>,
    /// The deck load riding this machine's *body*, with each item's offset
    /// in the body link's frame. A walking machine has no deck rigid with
    /// its route: the body tilts onto a grade and rides up the steps, so
    /// what it carries rides that, not the guide line underneath.
    carried: Vec<(String, Isometry3<f64>)>,
}

/// One program's scan-loop cursor. Several of these advancing over one
/// world is what "parallel sequences" means: each station is its own SFC,
/// the world is shared, and the only coupling is through what the world
/// carries (signals, sensors, robot/device state).
struct Program {
    sequence: Sequence,
    /// The authored steps compiled to a flat list with explicit exits —
    /// the cursor below indexes this, not `sequence.steps`.
    flat: Vec<FlatStep>,
    step: usize,
    entered_at: f64,
    /// Absolute end times of the moves started by the active step (`Done`
    /// waits for all of them).
    move_ends: Vec<f64>,
    /// Index into `step_spans` of the active step's span. Programs
    /// interleave their spans in one list, so "the last one" stopped
    /// meaning "mine" the moment there were two cursors.
    open_span: usize,
    /// Every signal lane's value as of *this program's* previous scan —
    /// what its edge conditions compare against. Per program, not global:
    /// a PLC edge instruction remembers what *it* saw last time it
    /// executed, so an edge raised later in the same scan by another
    /// program is still caught on this one's next pass. Seeded from the
    /// first sensor evaluation (startup state is not a transition).
    prev_signals: Vec<bool>,
}

impl Program {
    fn finished(&self) -> bool {
        self.step >= self.flat.len()
    }
}

/// One compiled step: authored steps flattened pre-order (a branching
/// step, then each arm's steps in arm order), each with explicit exits
/// `(condition, flat target)`. A linear step has one exit — its
/// transition, to the following step; a branching step has one per arm.
/// `flat.len()` is the end-of-program target. Forward-only by
/// construction (arms rejoin at the step after their branch), which is
/// what keeps every rollout terminating.
struct FlatStep {
    name: String,
    actions: Vec<Action>,
    exits: Vec<(Condition, usize)>,
    /// `Some(ordinal)` when the exits are branch arms (recorded as
    /// decisions on the timeline — a one-armed select still is one). The
    /// ordinal is the select's pre-order number within the sequence,
    /// identical to [`crate::seq::enumerate_selects`]'s numbering (both
    /// walk the authored tree pre-order; a test pins the agreement).
    select: Option<usize>,
}

fn flatten(steps: &[Step]) -> Vec<FlatStep> {
    /// Emits `steps`, returning the dangling exits — `(flat index, exit
    /// slot)` pairs whose target is the continuation the caller knows.
    fn emit(steps: &[Step], out: &mut Vec<FlatStep>, selects: &mut usize) -> Vec<(usize, usize)> {
        let mut dangling: Vec<(usize, usize)> = Vec::new();
        for step in steps {
            let here = out.len();
            // Everything that fell out of the previous step (its
            // transition, or a whole branch's arm tails) lands here.
            for (i, slot) in dangling.drain(..) {
                out[i].exits[slot].1 = here;
            }
            if step.select.is_empty() {
                out.push(FlatStep {
                    name: step.name.clone(),
                    actions: step.actions.clone(),
                    exits: vec![(step.transition.clone(), usize::MAX)],
                    select: None,
                });
                dangling.push((here, 0));
            } else {
                let ordinal = *selects;
                *selects += 1;
                out.push(FlatStep {
                    name: step.name.clone(),
                    actions: Vec::new(),
                    exits: step
                        .select
                        .iter()
                        .map(|arm| (arm.condition.clone(), usize::MAX))
                        .collect(),
                    select: Some(ordinal),
                });
                for (j, arm) in step.select.iter().enumerate() {
                    out[here].exits[j].1 = out.len();
                    if arm.steps.is_empty() {
                        // An empty arm exits straight to the rejoin.
                        dangling.push((here, j));
                    } else {
                        let mut tails = emit(&arm.steps, out, selects);
                        dangling.append(&mut tails);
                    }
                }
            }
        }
        dangling
    }
    let mut out = Vec::new();
    let mut selects = 0;
    let tails = emit(steps, &mut out, &mut selects);
    let end = out.len();
    for (i, slot) in tails {
        out[i].exits[slot].1 = end;
    }
    out
}

struct Rollout {
    world: Scene,
    /// The concurrently-running programs, in the order given to
    /// `simulate_sequences` — which is the deterministic scan order: every
    /// tick each program gets one transition scan, first to last, so a
    /// signal written by an earlier program is seen by a later one in the
    /// same tick, and the whole bake stays bit-identical.
    programs: Vec<Program>,
    /// The program currently being scanned (actions fire, transitions
    /// evaluate against this cursor).
    current: usize,
    options: RolloutOptions,

    t: f64,
    /// Per-robot runtimes, in scene order (the deterministic advance order).
    robots: Vec<RobotRuntime>,
    sensors: Vec<SensorRuntime>,
    devices: Vec<DeviceRuntime>,
    /// Lanes the scenario's faults pin, `(lane, value)` in scenario
    /// order. A pinned lane is seeded with its value and every later
    /// write to it — sensor geometry, a program's `set` — is dropped.
    forced: Vec<(usize, bool)>,

    /// The vehicles that moved this tick, each with what it carries (body
    /// and load) — what its riding robots are checked against the rest of
    /// the world *not* being.
    moving: Vec<(String, Vec<String>)>,

    // Accumulating outputs.
    objects: Vec<ObjectTrack>,
    /// Vehicle reference-frame tracks (see `SequenceTimeline::vehicles`).
    vehicles: Vec<ObjectTrack>,
    signals: Vec<BoolTrack>,
    step_spans: Vec<StepSpan>,
    branches: Vec<BranchTaken>,
}

/// The motion currently driving the joints, sampled per scan tick.
enum ActiveMove {
    Traj {
        start: f64,
        traj: JointTrajectory,
    },
    Ramp {
        start: f64,
        duration: f64,
        from: Vec<f64>,
        to: Vec<f64>,
    },
}

impl ActiveMove {
    fn sample(&self, t: f64) -> Vec<f64> {
        match self {
            ActiveMove::Traj { start, traj } => traj.sample(t - start),
            ActiveMove::Ramp {
                start,
                duration,
                from,
                to,
            } => {
                let u = ((t - start) / duration).clamp(0.0, 1.0);
                // Rest-to-rest cubic (matches the baked Hermite waypoints).
                let s = u * u * (3.0 - 2.0 * u);
                from.iter().zip(to).map(|(a, b)| a + (b - a) * s).collect()
            }
        }
    }

    /// Absolute end time of this move.
    fn end(&self) -> f64 {
        match self {
            ActiveMove::Traj { start, traj } => start + traj.duration(),
            ActiveMove::Ramp {
                start, duration, ..
            } => start + duration,
        }
    }
}

/// Per-tick tracking solve: warm-started from the nominal configuration,
/// so it only has to absorb one scan period of part motion. No restarts —
/// a restart could land on another solution branch and teleport the arm
/// mid-track. Null-space centering is off for the same reason: the track
/// must reproduce the taught posture carried by the offset, not add its
/// own self-motion drift on top (a deliberate bias would belong to the
/// authoring layer, not the per-tick follow).
const TRACK_IK: botrail_kin::IkOptions = botrail_kin::IkOptions {
    mode: botrail_kin::IkMode::Pose,
    max_iters: 100,
    tol_pos: 1e-7,
    tol_rot: 1e-6,
    damping: 0.05,
    orientation_weight: 0.5,
    max_step: 0.5,
    restarts: 0,
    null_space_gain: 0.0,
};

/// An active conveyor track: taught poses are carried by `offset`, which is
/// the part's rigid motion since the latch — recomputed every tick until the
/// part is grasped, then frozen (from then on it is the robot moving it).
struct TrackLatch {
    object: String,
    link: usize,
    origin: Isometry3<f64>,
    offset: Isometry3<f64>,
    frozen: bool,
    /// The program that latched the track — mid-track failures surface
    /// during the world advance, when no program is being scanned, and
    /// still have to be pinned on the right one.
    program: usize,
}

struct SensorRuntime {
    collider: ObstacleCollider,
    /// Authored pose in the anchor's frame (world for a fixture).
    pose: Isometry3<f64>,
    /// What re-resolves the sensor's frame each tick.
    anchor: SensorAnchor,
    watch: SensorWatch,
    /// Index of this sensor's lane in the signal tracks.
    lane: usize,
    /// Vision/field only: ray-test each candidate's origin against the
    /// *other* obstacles before declaring it seen.
    occlusion: bool,
    /// Obstacle indices this sensor never considers — a vehicle-mounted
    /// field's own machine body, which the massing chassis would
    /// otherwise trip or blind (design/design-lidar.md §9). Candidates
    /// and shadow casters alike.
    exclude: Vec<usize>,
}

enum SensorAnchor {
    /// Bolted to the floor.
    Fixture,
    /// Rides a vehicle device (index into the device list).
    Vehicle(usize),
    /// A vision camera bolted to a robot link.
    Link { robot: usize, link: usize },
}

/// Is `point` inside the box centred on `pose` with half-extents `half`?
fn inside_zone(pose: &Isometry3<f64>, half: &Vector3<f64>, point: &Isometry3<f64>) -> bool {
    let local = pose
        .inverse()
        .transform_point(&nalgebra::Point3::from(point.translation.vector));
    local.x.abs() <= half.x && local.y.abs() <= half.y && local.z.abs() <= half.z
}

enum DeviceRuntime {
    Conveyor {
        name: String,
        zone_pose: Isometry3<f64>,
        zone_half: Vector3<f64>,
        velocity: Vector3<f64>,
        running: bool,
        /// Metres left of a fixed `Advance`; `None` when none is pending.
        /// While pending the belt runs regardless of `running`, and the
        /// final tick consumes exactly the remainder.
        remaining: Option<f64>,
        lane: usize,
    },
    Axis {
        name: String,
        objects: Vec<String>,
        axis: Vector3<f64>,
        speed: f64,
        position: f64,
        target: f64,
        lane: usize,
    },
    Source {
        name: String,
        pool: Vec<String>,
        park: Isometry3<f64>,
        pitch: Vector3<f64>,
        pose: Isometry3<f64>,
        interval: f64,
        running: bool,
        /// Seconds until the next release is due.
        due: f64,
        /// Per pool member: waiting in the magazine rather than out on the
        /// line. Seeded from where the scene put it, so a belt authored
        /// already-loaded stays loaded.
        parked: Vec<bool>,
        lane: usize,
    },
    Sink {
        name: String,
        zone_pose: Isometry3<f64>,
        zone_half: Vector3<f64>,
        /// Index into `devices` of the source this returns members to.
        source: usize,
    },
    Vehicle {
        name: String,
        waypoints: Vec<nalgebra::Point3<f64>>,
        stations: Vec<(String, usize)>,
        ring: bool,
        body: Vec<String>,
        speed: f64,
        turn_speed: f64,
        /// The drive semantics — how a goto's route becomes legs.
        drive: crate::seq::Drive,
        /// Load deck in the vehicle frame (pose, half-extents).
        tray: Option<(Isometry3<f64>, Vector3<f64>)>,
        /// Reference frame: position on the guidance surface + yaw. The
        /// body rides this frame's rigid motion, z included — a graded leg
        /// carries the whole machine up it.
        position: nalgebra::Point3<f64>,
        heading: f64,
        /// Waypoint index the vehicle is parked at (`None` while travelling).
        at: Option<usize>,
        /// Commanded station's waypoint index (`DeviceDone` = parked there).
        target: usize,
        /// Remaining legs of the active goto, front first.
        legs: std::collections::VecDeque<Leg>,
        lane: usize,
    },
    /// An elevator: the car obstacles and whatever the capture zone held
    /// at dispatch ride `axis · position` between named stops.
    Lift {
        name: String,
        car: Vec<String>,
        /// Capture zone at `position = 0`; rides the axis with the car.
        zone_pose: Isometry3<f64>,
        zone_half: Vector3<f64>,
        axis: Vector3<f64>,
        speed: f64,
        stops: Vec<(String, f64)>,
        position: f64,
        target: f64,
        /// Cargo fixed at dispatch (the doors are closed): loose obstacles
        /// by origin, and vehicles whole.
        cargo_objects: Vec<String>,
        cargo_vehicles: Vec<String>,
        lane: usize,
    },
}

/// One piece of a vehicle's route.
enum Leg {
    /// Pivot in place to the absolute heading `to` at signed rate `omega`.
    Turn { to: f64, omega: f64 },
    /// Drive straight to `to` at `velocity`. The leg owns its rate: a
    /// graded leg climbs as part of the same straight run, and a later
    /// aerial drive prices its legs per axis.
    Straight {
        to: nalgebra::Point3<f64>,
        velocity: Vector3<f64>,
    },
}

/// The net rigid motion a vehicle applied over one sub-interval of a tick.
#[derive(Debug, Clone, Copy)]
pub(crate) enum VehiclePiece {
    Lin {
        velocity: Vector3<f64>,
    },
    Piv {
        center: nalgebra::Point3<f64>,
        omega: f64,
    },
}

/// One travelling vehicle's tick worth of motion: its name, body members,
/// the load deck as it stood at the *start* of the tick (whatever was on it
/// then rides), and the exact sub-tick `(τ0, τ1, piece)` intervals.
struct VehicleMove {
    name: String,
    body: Vec<String>,
    tray: Option<(Isometry3<f64>, Vector3<f64>)>,
    /// The vehicle frame at the start of the tick — the seed of the
    /// vehicle's own pose track (what places its mounted sensors during
    /// playback).
    frame: Isometry3<f64>,
    /// The walk emptied the legs this tick: the vehicle arrived.
    parked: bool,
    /// Fellow lift cargo riding the same motion (moved by the lift, not
    /// by this vehicle) — carried as far as the checks are concerned: a
    /// tote on the deck is no aisle hazard to the arm riding beside it.
    extra: Vec<String>,
    pieces: Vec<(f64, f64, VehiclePiece)>,
}

/// Wraps to `(-π, π]` — the exact-π case turns counter-clockwise, fixed, so
/// a 180° about-face is deterministic.
fn wrap_angle(a: f64) -> f64 {
    let mut a = a % std::f64::consts::TAU;
    if a <= -std::f64::consts::PI {
        a += std::f64::consts::TAU;
    } else if a > std::f64::consts::PI {
        a -= std::f64::consts::TAU;
    }
    a
}

/// Waypoint indices from `from` to `to` (exclusive of `from`, inclusive of
/// `to`): the straight index walk on an open path; on a ring, whichever way
/// around is shorter by distance (ties go forward).
fn vehicle_route(
    waypoints: &[nalgebra::Point3<f64>],
    ring: bool,
    from: usize,
    to: usize,
) -> Vec<usize> {
    if from == to {
        return Vec::new();
    }
    if !ring {
        return if from < to {
            ((from + 1)..=to).collect()
        } else {
            (to..from).rev().collect()
        };
    }
    let n = waypoints.len();
    let walk_by = |step: usize| {
        let mut walk = Vec::new();
        let mut i = from;
        while i != to {
            i = (i + step) % n;
            walk.push(i);
        }
        walk
    };
    let forward = walk_by(1);
    let backward = walk_by(n - 1);
    let length = |walk: &[usize]| {
        let mut prev = from;
        let mut total = 0.0;
        for &i in walk {
            total += (waypoints[i] - waypoints[prev]).norm();
            prev = i;
        }
        total
    };
    if length(&backward) + 1e-12 < length(&forward) {
        backward
    } else {
        forward
    }
}

/// Expands a route into turn/straight legs from the vehicle's current
/// heading. Coincident waypoints contribute no leg.
fn build_legs(
    waypoints: &[nalgebra::Point3<f64>],
    route: &[usize],
    start: nalgebra::Point3<f64>,
    heading: f64,
    speed: f64,
    turn_speed: f64,
    drive: &crate::seq::Drive,
) -> std::collections::VecDeque<Leg> {
    use crate::seq::{AerialYaw, Drive};
    let mut legs = std::collections::VecDeque::new();
    let mut position = start;
    let mut heading = heading;
    for &i in route {
        let to = waypoints[i];
        let d = to - position;
        if d.norm() < 1e-9 {
            continue;
        }
        let run = d.x.hypot(d.y);
        match drive {
            Drive::Holonomic { .. } => {
                // Mecanum wheels: translate, never turn — the machine
                // docks facing whatever it faced when parked, which is
                // the whole point of buying those wheels.
                legs.push_back(Leg::Straight {
                    to,
                    velocity: d / d.norm() * speed,
                });
            }
            Drive::Differential { allow_reverse, .. } => {
                // Heading is set by the horizontal run — a graded leg
                // still faces where it is going on the floor plan. (A
                // vertical stack cannot face anywhere; validation refuses
                // it for a ground drive.)
                let travel = if run > 1e-9 { d.y.atan2(d.x) } else { heading };
                // Backing up is just facing the other way while travelling
                // the same direction — worth it whenever it is the shorter
                // turn, which is exactly when a machine would reverse
                // rather than turn around.
                let leg_heading = if *allow_reverse
                    && run > 1e-9
                    && wrap_angle(travel - heading).abs() > std::f64::consts::FRAC_PI_2
                {
                    wrap_angle(travel + std::f64::consts::PI)
                } else {
                    travel
                };
                let dphi = wrap_angle(leg_heading - heading);
                if dphi.abs() > 1e-9 {
                    legs.push_back(Leg::Turn {
                        to: leg_heading,
                        omega: dphi.signum() * turn_speed,
                    });
                }
                legs.push_back(Leg::Straight {
                    to,
                    // Cruise speed is spent along the leg — 3D arc length,
                    // so a graded leg takes proportionally longer, like
                    // the machine it models.
                    velocity: d / d.norm() * speed,
                });
                heading = leg_heading;
            }
            Drive::Aerial {
                climb_speed,
                descent_speed,
                yaw,
            } => {
                // The yaw policy first: face the course, or hold the fix.
                // A vertical leg has no course and keeps the heading.
                let leg_heading = match yaw {
                    AerialYaw::Fixed(psi) => *psi,
                    AerialYaw::Course if run > 1e-9 => d.y.atan2(d.x),
                    AerialYaw::Course => heading,
                };
                let dphi = wrap_angle(leg_heading - heading);
                if dphi.abs() > 1e-9 {
                    legs.push_back(Leg::Turn {
                        to: leg_heading,
                        omega: dphi.signum() * turn_speed,
                    });
                }
                // Every axis flies at its own limit and the slower one
                // sets the clock — closed form, per leg:
                // T = max(run / cruise, rise / climb (or descent)).
                let t_xy = if run > 1e-9 { run / speed } else { 0.0 };
                let t_z = if d.z > 1e-9 {
                    d.z / climb_speed
                } else if d.z < -1e-9 {
                    -d.z / descent_speed
                } else {
                    0.0
                };
                let t = t_xy.max(t_z).max(1e-12);
                legs.push_back(Leg::Straight {
                    to,
                    velocity: d / t,
                });
                heading = leg_heading;
            }
        }
        position = to;
    }
    legs
}

impl Rollout {
    fn new(world: Scene, sequences: Vec<Sequence>, options: RolloutOptions) -> Self {
        // Signal lanes: internal relays, then sensor inputs, then device
        // outputs — all recorded as edge tracks for the timing chart.
        let mut signals: Vec<BoolTrack> = world
            .signals()
            .iter()
            .map(|s| BoolTrack {
                name: s.name.clone(),
                edges: vec![(0.0, s.initial)],
                kind: LaneKind::Signal,
            })
            .collect();
        let sensors: Vec<SensorRuntime> = world
            .sensors()
            .iter()
            .map(|sensor| {
                // A zone/beam fixture rides its authored vehicle mount.
                let mount_anchor = || {
                    sensor
                        .mount
                        .as_ref()
                        .and_then(|name| world.devices().iter().position(|d| &d.name == name))
                        .map(SensorAnchor::Vehicle)
                        .unwrap_or(SensorAnchor::Fixture)
                };
                // A never-tripping stand-in for degenerate optics or a
                // dangling camera reference (a hand-edited project) — a
                // dead lane beats a crash mid-bake.
                let dead = || ObstacleCollider::cuboid(nalgebra::Vector3::repeat(1e-9));
                let (collider, pose, anchor, occlusion) = match &sensor.kind {
                    SensorKind::Zone { pose, size } => (
                        ObstacleCollider::cuboid(size / 2.0),
                        *pose,
                        mount_anchor(),
                        false,
                    ),
                    SensorKind::Beam { from, to, radius } => (
                        ObstacleCollider::capsule(*from, *to, *radius),
                        Isometry3::identity(),
                        mount_anchor(),
                        false,
                    ),
                    SensorKind::Vision {
                        camera,
                        detect_range,
                        occlusion,
                    } => match world.cameras().iter().find(|c| &c.name == camera) {
                        Some(cam) => {
                            // The camera is the optics: frustum from its
                            // fov/aspect, frame from its mount, band from
                            // the sensor (default: the camera's clips).
                            let [near, far] = detect_range.unwrap_or([cam.near, cam.far]);
                            let aspect =
                                cam.resolution[0].max(1) as f64 / cam.resolution[1].max(1) as f64;
                            let collider = ObstacleCollider::frustum(
                                cam.fov_deg.to_radians(),
                                aspect,
                                near,
                                far,
                            )
                            .unwrap_or_else(dead);
                            let anchor = match &cam.mount {
                                crate::seq::CameraMount::World => SensorAnchor::Fixture,
                                crate::seq::CameraMount::Vehicle { device } => world
                                    .devices()
                                    .iter()
                                    .position(|d| &d.name == device)
                                    .map(SensorAnchor::Vehicle)
                                    .unwrap_or(SensorAnchor::Fixture),
                                crate::seq::CameraMount::Link { robot, link } => world
                                    .robot_index(robot)
                                    .and_then(|r| {
                                        world.robots()[r]
                                            .model
                                            .link_index(link)
                                            .map(|l| SensorAnchor::Link { robot: r, link: l })
                                    })
                                    .unwrap_or(SensorAnchor::Fixture),
                            };
                            (collider, cam.pose, anchor, *occlusion)
                        }
                        None => (dead(), Isometry3::identity(), SensorAnchor::Fixture, false),
                    },
                    SensorKind::Field {
                        lidar,
                        range,
                        sector,
                        shadowing,
                    } => match world.lidars().iter().find(|l| &l.name == lidar) {
                        Some(scanner) => {
                            // The lidar is the sweep: sector from its fov
                            // (or the field's window), radius from its max
                            // range (or the field's), frame from its
                            // mount. ±5 mm slab (design/design-lidar.md
                            // 判断 L6).
                            let half = scanner.fov_deg.to_radians() / 2.0;
                            let [start, end] = sector
                                .map(|[a, b]| [a.to_radians(), b.to_radians()])
                                .unwrap_or([-half, half]);
                            let radius = range.unwrap_or(scanner.range[1]);
                            let collider = ObstacleCollider::sector(start, end, radius, 0.005)
                                .unwrap_or_else(dead);
                            let anchor = match &scanner.mount {
                                crate::seq::LidarMount::World => SensorAnchor::Fixture,
                                crate::seq::LidarMount::Vehicle { device } => world
                                    .devices()
                                    .iter()
                                    .position(|d| &d.name == device)
                                    .map(SensorAnchor::Vehicle)
                                    .unwrap_or(SensorAnchor::Fixture),
                                crate::seq::LidarMount::Link { robot, link } => world
                                    .robot_index(robot)
                                    .and_then(|r| {
                                        world.robots()[r]
                                            .model
                                            .link_index(link)
                                            .map(|l| SensorAnchor::Link { robot: r, link: l })
                                    })
                                    .unwrap_or(SensorAnchor::Fixture),
                            };
                            (collider, scanner.pose, anchor, *shadowing)
                        }
                        None => (dead(), Isometry3::identity(), SensorAnchor::Fixture, false),
                    },
                };
                // A vehicle-mounted field ignores its own machine's body:
                // the massing chassis rides inside every sweep and would
                // trip the field (or shadow everything) forever.
                let exclude = match (&sensor.kind, &anchor) {
                    (SensorKind::Field { .. }, SensorAnchor::Vehicle(d)) => {
                        match &world.devices()[*d].kind {
                            crate::seq::DeviceKind::Vehicle { body, .. } => body
                                .iter()
                                .filter_map(|name| {
                                    world.obstacles().iter().position(|o| &o.name == name)
                                })
                                .collect(),
                            _ => Vec::new(),
                        }
                    }
                    _ => Vec::new(),
                };
                let lane = signals.len();
                signals.push(BoolTrack {
                    name: sensor.name.clone(),
                    edges: vec![(0.0, false)],
                    kind: LaneKind::Sensor,
                });
                SensorRuntime {
                    collider,
                    pose,
                    anchor,
                    watch: sensor.watch.clone(),
                    lane,
                    occlusion,
                    exclude,
                }
            })
            .collect();
        let devices: Vec<DeviceRuntime> = world
            .devices()
            .iter()
            .map(|device| {
                let lane = signals.len();
                match &device.kind {
                    DeviceKind::Conveyor {
                        zone_pose,
                        zone_size,
                        velocity,
                        running,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, *running)],
                            kind: LaneKind::Device,
                        });
                        DeviceRuntime::Conveyor {
                            name: device.name.clone(),
                            zone_pose: *zone_pose,
                            zone_half: zone_size / 2.0,
                            velocity: *velocity,
                            running: *running,
                            remaining: None,
                            lane,
                        }
                    }
                    DeviceKind::LinearAxis {
                        objects,
                        axis,
                        speed,
                        position,
                        ..
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, false)],
                            kind: LaneKind::Device,
                        });
                        DeviceRuntime::Axis {
                            name: device.name.clone(),
                            objects: objects.clone(),
                            axis: axis.into_inner(),
                            speed: *speed,
                            position: *position,
                            target: *position,
                            lane,
                        }
                    }
                    DeviceKind::Source {
                        pool,
                        park,
                        pitch,
                        pose,
                        interval,
                        running,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, *running)],
                            kind: LaneKind::Device,
                        });
                        // Seeded from the world: a member sitting on its
                        // parking slot is waiting, anything else is already
                        // out on the line.
                        let parked = pool
                            .iter()
                            .enumerate()
                            .map(|(i, name)| {
                                world
                                    .obstacles()
                                    .iter()
                                    .find(|o| &o.name == name)
                                    .is_some_and(|o| {
                                        let slot = park.translation.vector + pitch * i as f64;
                                        (o.pose.translation.vector - slot).norm() < 1e-6
                                    })
                            })
                            .collect();
                        DeviceRuntime::Source {
                            name: device.name.clone(),
                            pool: pool.clone(),
                            park: *park,
                            pitch: *pitch,
                            pose: *pose,
                            interval: *interval,
                            running: *running,
                            due: 0.0,
                            parked,
                            lane,
                        }
                    }
                    DeviceKind::Sink {
                        zone_pose,
                        zone_size,
                        source,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, false)],
                            kind: LaneKind::Device,
                        });
                        DeviceRuntime::Sink {
                            name: device.name.clone(),
                            zone_pose: *zone_pose,
                            zone_half: zone_size / 2.0,
                            // Resolved after the pass: sources may come later
                            // in the list.
                            source: world
                                .devices()
                                .iter()
                                .position(|d| &d.name == source)
                                .unwrap_or(usize::MAX),
                        }
                    }
                    DeviceKind::Vehicle {
                        path,
                        body,
                        speed,
                        turn_speed,
                        start,
                        drive,
                        tray,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, false)],
                            kind: LaneKind::Device,
                        });
                        // Validation vetted the station; a missing one only
                        // happens on unvalidated direct use, and parks at 0.
                        let at = path.station(start).unwrap_or(0);
                        let position = path
                            .waypoints
                            .get(at)
                            .copied()
                            .unwrap_or_else(nalgebra::Point3::origin);
                        let heading = path.heading_at(at);
                        DeviceRuntime::Vehicle {
                            name: device.name.clone(),
                            waypoints: path.waypoints.clone(),
                            stations: path.stations.clone(),
                            ring: path.ring,
                            body: body.clone(),
                            speed: *speed,
                            turn_speed: *turn_speed,
                            drive: *drive,
                            tray: tray.map(|(pose, size)| (pose, size / 2.0)),
                            position,
                            heading,
                            at: Some(at),
                            target: at,
                            legs: std::collections::VecDeque::new(),
                            lane,
                        }
                    }
                    DeviceKind::Lift {
                        car,
                        zone_pose,
                        zone_size,
                        axis,
                        speed,
                        stops,
                        start,
                    } => {
                        signals.push(BoolTrack {
                            name: device.name.clone(),
                            edges: vec![(0.0, false)],
                            kind: LaneKind::Device,
                        });
                        // Validation vetted the stop; direct unvalidated
                        // use parks at the reference.
                        let position = stops
                            .iter()
                            .find(|(n, _)| n == start)
                            .map(|(_, v)| *v)
                            .unwrap_or(0.0);
                        DeviceRuntime::Lift {
                            name: device.name.clone(),
                            car: car.clone(),
                            zone_pose: *zone_pose,
                            zone_half: zone_size / 2.0,
                            axis: axis.into_inner(),
                            speed: *speed,
                            stops: stops.clone(),
                            position,
                            target: position,
                            cargo_objects: Vec::new(),
                            cargo_vehicles: Vec::new(),
                            lane,
                        }
                    }
                }
            })
            .collect();
        // Carriers waiting in a magazine at t = 0 are stock: stowed from
        // the start, so a run never opens on a pile of parts that then
        // teleport onto the line.
        let mut stowed: Vec<ObjectTrack> = Vec::new();
        for device in &devices {
            if let DeviceRuntime::Source {
                pool,
                parked,
                park,
                pitch,
                ..
            } = device
            {
                for (i, name) in pool.iter().enumerate() {
                    if !parked[i] {
                        continue;
                    }
                    let slot = Isometry3::from_parts(
                        nalgebra::Translation3::from(park.translation.vector + pitch * i as f64),
                        park.rotation,
                    );
                    stowed.push(ObjectTrack {
                        name: name.clone(),
                        spans: vec![TrackSpan::Stowed {
                            t0: 0.0,
                            t1: 0.0,
                            pose: slot,
                        }],
                    });
                }
            }
        }

        // Objects grasped before the sequence starts follow from t = 0.
        let objects = world
            .attachments()
            .iter()
            .map(|a| ObjectTrack {
                name: a.object.clone(),
                spans: vec![TrackSpan::Follow {
                    t0: 0.0,
                    t1: 0.0,
                    robot: a.robot,
                    link: a.link,
                    offset: a.grasp,
                }],
            })
            .chain(stowed)
            .collect();
        let robots = world
            .robots()
            .iter()
            .map(|sr| {
                let q = sr.joint_positions().to_vec();
                let gait = sr.mount.as_ref().and_then(|mount| {
                    let spec = mount.gait.as_ref()?;
                    Some(GaitRuntime {
                        gait: crate::gait::resolve_gait(&sr.model, spec, &q)
                            .expect("a mounted gait was validated against its model"),
                        offset: mount.offset,
                        plan: None,
                        history: Vec::new(),
                        sway: Isometry3::identity(),
                        sways: Vec::new(),
                        pitch: Isometry3::identity(),
                        pitches: Vec::new(),
                        rise: 0.0,
                        rises: Vec::new(),
                        carried: Vec::new(),
                    })
                });
                let spin = sr.mount.as_ref().and_then(|mount| {
                    if mount.spin.is_empty() {
                        return None;
                    }
                    let ground = world.devices().iter().find_map(|d| match &d.kind {
                        crate::seq::DeviceKind::Vehicle { path, start, .. }
                            if d.name == mount.device =>
                        {
                            path.station(start)
                                .and_then(|at| path.waypoints.get(at))
                                .map(|p| p.z)
                        }
                        _ => None,
                    })?;
                    let names = sr.model.actuated_joint_names();
                    Some(SpinRuntime {
                        device: mount.device.clone(),
                        ground,
                        joints: mount
                            .spin
                            .iter()
                            .filter_map(|(joint, rate)| {
                                names.iter().position(|n| n == joint).map(|i| (i, *rate))
                            })
                            .collect(),
                    })
                });
                RobotRuntime {
                    times: vec![0.0],
                    positions: vec![q.clone()],
                    velocities: vec![vec![0.0; q.len()]],
                    q_nom: q.clone(),
                    q_nom_prev: q.clone(),
                    q,
                    active: None,
                    tracking: None,
                    moves: Vec::new(),
                    planned: Vec::new(),
                    base: sr.mount.as_ref().map(|_| Vec::new()),
                    gait,
                    spin,
                }
            })
            .collect();
        // Faults: the scenario resolved them to `(lane name, value)`; the
        // lane is seeded with the forced value (a pinned input is a level
        // from t = 0, not an edge at t = 0) and stays there — `set_lane`
        // drops writes to it. Anchored injection would activate entries
        // mid-run here; v1 pins from the first scan.
        let forced: Vec<(usize, bool)> = world
            .forced_inputs()
            .iter()
            .filter_map(|(name, value)| {
                signals
                    .iter()
                    .position(|s| &s.name == name)
                    .map(|lane| (lane, *value))
            })
            .collect();
        for (lane, value) in &forced {
            signals[*lane].edges = vec![(0.0, *value)];
        }
        Rollout {
            world,
            programs: sequences
                .into_iter()
                .map(|sequence| Program {
                    flat: flatten(&sequence.steps),
                    sequence,
                    step: 0,
                    entered_at: 0.0,
                    move_ends: Vec::new(),
                    open_span: 0,
                    prev_signals: Vec::new(),
                })
                .collect(),
            current: 0,
            options,
            t: 0.0,
            robots,
            sensors,
            devices,
            forced,
            moving: Vec::new(),
            objects,
            vehicles: Vec::new(),
            signals,
            step_spans: Vec::new(),
            branches: Vec::new(),
        }
    }

    /// The step's display name, qualified by its program when several run
    /// — two stations both have a `feed`, and a timeline (or an error)
    /// that just says `feed` names neither.
    fn step_name_in(&self, program: usize, index: usize) -> String {
        let p = &self.programs[program];
        let step = p
            .flat
            .get(index)
            .map(|s| s.name.clone())
            .unwrap_or_default();
        if self.programs.len() == 1 {
            step
        } else {
            format!("{}/{step}", p.sequence.name)
        }
    }

    fn cur_step(&self) -> usize {
        self.programs[self.current].step
    }

    fn cur_step_name(&self) -> String {
        self.step_name_in(self.current, self.cur_step())
    }

    fn finished(&self) -> bool {
        self.programs.iter().all(Program::finished)
    }

    fn run(mut self) -> Result<SequenceTimeline, SeqError> {
        self.update_sensors();
        // Seed every program's edge memory from the evaluated startup
        // state: a sensor already tripped at t = 0 is a level, not an
        // edge. A `set` fired by the first scan below *does* edge — that
        // is a real transition.
        for p in 0..self.programs.len() {
            self.close_scan(p);
        }
        // First scan: every program enters its first step (actions fire in
        // declaration order), then every program chains through whatever
        // is instantaneously ready — so a signal set by program 1's first
        // step is already visible to program 2's first transition.
        for p in 0..self.programs.len() {
            self.current = p;
            self.enter_step()?;
        }
        for p in 0..self.programs.len() {
            self.current = p;
            self.advance_through_ready_steps()?;
            self.close_scan(p);
        }

        let mut tick = 0u64;
        while !self.finished() {
            tick += 1;
            self.t = tick as f64 * self.options.dt;
            if self.t > self.options.max_duration {
                return Err(self.timeout());
            }
            // PLC scan: outputs advance the world through this tick, then
            // inputs are read, then transitions fire — per program in
            // declaration order, each closing its own edge memory as it
            // hands over.
            self.advance_world()?;
            self.update_sensors();
            for p in 0..self.programs.len() {
                self.current = p;
                self.advance_through_ready_steps()?;
                self.close_scan(p);
            }
        }
        Ok(self.finish())
    }

    /// The timeout error, naming where every unfinished program is stuck —
    /// with several programs that list is the deadlock diagnosis (A waits
    /// on a signal B never sets), so it has to name all of them.
    fn timeout(&self) -> SeqError {
        let waiting: Vec<usize> = (0..self.programs.len())
            .filter(|&p| !self.programs[p].finished())
            .collect();
        let forced: Vec<(String, bool)> = self
            .forced
            .iter()
            .map(|(lane, value)| (self.signals[*lane].name.clone(), *value))
            .collect();
        if self.programs.len() == 1 {
            let step = self.programs[0].step;
            return SeqError::Timeout {
                step,
                name: self.step_name_in(0, step),
                limit: self.options.max_duration,
                forced,
            };
        }
        SeqError::ProgramsTimeout {
            at: waiting
                .iter()
                .map(|&p| self.step_name_in(p, self.programs[p].step))
                .collect::<Vec<_>>()
                .join(", "),
            limit: self.options.max_duration,
            forced,
        }
    }

    /// Advances every robot's joints and every device by one scan period,
    /// then verifies the robots stayed clear of each other.
    fn advance_world(&mut self) -> Result<(), SeqError> {
        let t = self.t;
        let dt = self.options.dt;
        // Joints follow each robot's in-flight motion/ramp, in scene order
        // (attached obstacles are re-synced by set_joint_positions_for).
        for (r, rt) in self.robots.iter_mut().enumerate() {
            let Some(active) = &rt.active else { continue };
            rt.q_nom = active.sample(t);
            // Under a track the commanded joints are solved in
            // `follow_tracked_parts`, once this tick's part motion is known.
            if rt.tracking.is_none() {
                let mut q = rt.q_nom.clone();
                // While the legs walk they are the gait's: a ramp alongside
                // the drive moves the rest of the robot (`advance_gaits`).
                if let Some(gr) = rt.gait.as_ref() {
                    if let Some(plan) = &gr.plan {
                        for qi in plan.owned(&gr.gait) {
                            q[qi] = rt.q[qi];
                        }
                    }
                }
                rt.q = q;
                self.world
                    .set_joint_positions_for(r, rt.q.clone())
                    .expect("sampled q has robot DOF");
            }
            if t >= active.end() - 1e-9 {
                rt.active = None;
            }
        }

        // Devices move obstacles kinematically; grasped objects are bound
        // to the robot and never advected.
        let attached: Vec<String> = self
            .world
            .attachments()
            .iter()
            .map(|a| a.object.clone())
            .collect();
        let mut moved: Vec<(String, Isometry3<f64>, Vector3<f64>)> = Vec::new();
        // (source device index, object) pairs a sink caught this tick.
        let mut returned: Vec<(usize, String)> = Vec::new();
        let mut lane_updates: Vec<(usize, bool)> = Vec::new();
        // Per travelling vehicle: its exact sub-tick motion pieces, applied
        // to the body after this loop (span recording needs `&mut self`).
        let mut vehicle_moves: Vec<VehicleMove> = Vec::new();
        // Vehicles a lift carried this tick: `(vehicle, displacement,
        // span velocity, ride ends this tick, fellow cargo)` — their
        // runtimes are other entries of the device list, so they are
        // shifted after the loop.
        #[allow(clippy::type_complexity)]
        let mut lift_shifts: Vec<(String, Vector3<f64>, Vector3<f64>, bool, Vec<String>)> =
            Vec::new();
        for device in &mut self.devices {
            match device {
                DeviceRuntime::Conveyor {
                    zone_pose,
                    zone_half,
                    velocity,
                    running,
                    remaining,
                    lane,
                    ..
                } => {
                    let speed = velocity.norm();
                    // A pending fixed advance runs the belt whatever the
                    // running flag says; the last tick moves exactly the
                    // remainder so the pitch never picks up a scan-period
                    // fraction. Otherwise plain free-running.
                    let tick_velocity = if let Some(rem) = remaining.take() {
                        if speed < 1e-12 {
                            // Speed zeroed mid-advance: nothing can move.
                            // The command stays pending (device_done stays
                            // false) rather than dividing by zero here.
                            *remaining = Some(rem);
                            None
                        } else {
                            let step = (speed * dt).min(rem);
                            let left = rem - step;
                            if left > 0.0 {
                                *remaining = Some(left);
                            } else {
                                lane_updates.push((*lane, false));
                            }
                            // Scaled so `v * dt` is exactly the (partial) step.
                            Some(*velocity * (step / (speed * dt)))
                        }
                    } else if *running && speed >= 1e-12 {
                        Some(*velocity)
                    } else {
                        None
                    };
                    let Some(tick_velocity) = tick_velocity else {
                        continue;
                    };
                    for obstacle in self.world.obstacles() {
                        if attached.iter().any(|a| a == &obstacle.name) {
                            continue;
                        }
                        let local = zone_pose.inverse().transform_point(&nalgebra::Point3::from(
                            obstacle.pose.translation.vector,
                        ));
                        if local.x.abs() <= zone_half.x
                            && local.y.abs() <= zone_half.y
                            && local.z.abs() <= zone_half.z
                        {
                            let mut pose = obstacle.pose;
                            pose.translation.vector += tick_velocity * dt;
                            moved.push((obstacle.name.clone(), pose, tick_velocity));
                        }
                    }
                }
                DeviceRuntime::Axis {
                    objects,
                    axis,
                    speed,
                    position,
                    target,
                    ..
                } => {
                    let remaining = *target - *position;
                    if remaining.abs() < 1e-12 || *speed <= 0.0 {
                        continue;
                    }
                    let step = remaining.abs().min(*speed * dt) * remaining.signum();
                    *position += step;
                    let delta = *axis * step;
                    // Span velocity from the actual per-tick displacement so
                    // the (partial) arrival tick samples exactly.
                    let velocity = *axis * (step / dt);
                    for name in objects.iter() {
                        if attached.iter().any(|a| a == name) {
                            continue;
                        }
                        if let Some(o) = self.world.obstacles().iter().find(|o| &o.name == name) {
                            let mut pose = o.pose;
                            pose.translation.vector += delta;
                            moved.push((name.clone(), pose, velocity));
                        }
                    }
                    // Arrival closes the axis's moving lane on this tick.
                    if (*target - *position).abs() < 1e-12 {
                        lane_updates.push((
                            match device {
                                DeviceRuntime::Axis { lane, .. } => *lane,
                                _ => unreachable!(),
                            },
                            false,
                        ));
                    }
                }
                DeviceRuntime::Lift {
                    car,
                    axis,
                    speed,
                    position,
                    target,
                    cargo_objects,
                    cargo_vehicles,
                    lane,
                    ..
                } => {
                    let remaining = *target - *position;
                    if remaining.abs() < 1e-12 || *speed <= 0.0 {
                        continue;
                    }
                    let step = remaining.abs().min(*speed * dt) * remaining.signum();
                    *position += step;
                    let delta = *axis * step;
                    // Span velocity from the actual per-tick displacement,
                    // like the axis: the (partial) arrival tick samples
                    // exactly.
                    let velocity = *axis * (step / dt);
                    for member in car.iter().chain(cargo_objects.iter()) {
                        if attached.iter().any(|a| a == member) {
                            continue;
                        }
                        if let Some(o) = self.world.obstacles().iter().find(|o| &o.name == member) {
                            let mut pose = o.pose;
                            pose.translation.vector += delta;
                            moved.push((member.clone(), pose, velocity));
                        }
                    }
                    let arriving = (*target - *position).abs() < 1e-12;
                    for vehicle in cargo_vehicles.iter() {
                        lift_shifts.push((
                            vehicle.clone(),
                            delta,
                            velocity,
                            arriving,
                            cargo_objects.clone(),
                        ));
                    }
                    if arriving {
                        lane_updates.push((*lane, false));
                        // The doors open: the cargo is free again.
                        cargo_objects.clear();
                        cargo_vehicles.clear();
                    }
                }
                DeviceRuntime::Sink {
                    zone_pose,
                    zone_half,
                    source,
                    ..
                } => {
                    for obstacle in self.world.obstacles() {
                        if attached.iter().any(|a| a == &obstacle.name) {
                            continue;
                        }
                        let local = zone_pose.inverse().transform_point(&nalgebra::Point3::from(
                            obstacle.pose.translation.vector,
                        ));
                        if local.x.abs() <= zone_half.x
                            && local.y.abs() <= zone_half.y
                            && local.z.abs() <= zone_half.z
                        {
                            returned.push((*source, obstacle.name.clone()));
                        }
                    }
                }
                DeviceRuntime::Vehicle {
                    name,
                    body,
                    tray,
                    position,
                    heading,
                    at,
                    target,
                    legs,
                    lane,
                    ..
                } => {
                    if legs.is_empty() {
                        continue;
                    }
                    // The deck as it stands *now*: what is on it at the top
                    // of the tick is what travels this tick. Capturing it
                    // before the walk also settles the ordering question —
                    // a part placed mid-tick joins on the next one.
                    let frame = vehicle_frame(position, *heading);
                    let deck = tray.map(|(pose, half)| (frame * pose, half));
                    // Walk the legs through this tick's time budget. Leg
                    // boundaries land at exact sub-tick instants, so the
                    // recorded spans (and any resample of them) are exact;
                    // world poses advance at tick boundaries like every
                    // other device.
                    let mut remaining = dt;
                    let mut pieces: Vec<(f64, f64, VehiclePiece)> = Vec::new();
                    while remaining > 1e-12 {
                        let Some(leg) = legs.front() else { break };
                        let tau0 = t - remaining;
                        match leg {
                            Leg::Turn { to, omega } => {
                                let left = wrap_angle(to - *heading);
                                let need = left / omega; // same sign ⇒ ≥ 0
                                if need <= 1e-12 {
                                    *heading = *to;
                                    legs.pop_front();
                                    continue;
                                }
                                let step = need.min(remaining);
                                pieces.push((
                                    tau0,
                                    tau0 + step,
                                    VehiclePiece::Piv {
                                        center: *position,
                                        omega: *omega,
                                    },
                                ));
                                if step >= need - 1e-12 {
                                    *heading = *to;
                                    legs.pop_front();
                                } else {
                                    *heading = wrap_angle(*heading + omega * step);
                                }
                                remaining -= step;
                            }
                            Leg::Straight { to, velocity } => {
                                let need = (to - *position).norm() / velocity.norm();
                                if need <= 1e-12 {
                                    *position = *to;
                                    legs.pop_front();
                                    continue;
                                }
                                let step = need.min(remaining);
                                pieces.push((
                                    tau0,
                                    tau0 + step,
                                    VehiclePiece::Lin {
                                        velocity: *velocity,
                                    },
                                ));
                                if step >= need - 1e-12 {
                                    *position = *to;
                                    legs.pop_front();
                                } else {
                                    *position += *velocity * step;
                                }
                                remaining -= step;
                            }
                        }
                    }
                    // A tick fully consumed ends exactly at t (float dust in
                    // `t - remaining + step` would otherwise leave the span
                    // an ulp short and trip the settle check mid-route).
                    if remaining <= 1e-12 {
                        if let Some(last) = pieces.last_mut() {
                            last.1 = t;
                        }
                    }
                    if legs.is_empty() {
                        *at = Some(*target);
                        lane_updates.push((*lane, false));
                    }
                    if !pieces.is_empty() {
                        vehicle_moves.push(VehicleMove {
                            name: name.clone(),
                            body: body.clone(),
                            tray: deck,
                            frame,
                            parked: legs.is_empty(),
                            extra: Vec::new(),
                            pieces,
                        });
                    }
                }
                // Releases are a second pass: a member returned this tick
                // should be feedable this tick, which needs the returns in
                // before any source looks at its magazine.
                DeviceRuntime::Source { .. } => {}
            }
        }
        for (lane, value) in lane_updates {
            self.set_lane(lane, t, value);
        }
        for (name, pose, velocity) in &moved {
            let rest = self
                .world
                .obstacles()
                .iter()
                .find(|o| &o.name == name)
                .map(|o| o.pose)
                .expect("moved obstacle exists");
            self.world
                .set_obstacle_pose(name, *pose)
                .expect("moved obstacle exists");
            self.extend_linear_span(name, rest, *velocity);
        }
        // A lift's cargo vehicles ride its motion through the same piece
        // machinery a drive uses — body, deck load and mounted robot stay
        // exactly rigid with the car, and the vehicle's own aisle checks
        // and pose track run as on any other moving tick.
        for (vehicle, delta, velocity, arriving, extra) in lift_shifts {
            for device in &mut self.devices {
                let DeviceRuntime::Vehicle {
                    name,
                    waypoints,
                    body,
                    tray,
                    position,
                    heading,
                    at,
                    target,
                    ..
                } = device
                else {
                    continue;
                };
                if name != &vehicle {
                    continue;
                }
                let frame = vehicle_frame(position, *heading);
                let deck = tray.map(|(pose, half)| (frame * pose, half));
                *position += delta;
                vehicle_moves.push(VehicleMove {
                    name: vehicle.clone(),
                    body: body.clone(),
                    tray: deck,
                    frame,
                    parked: arriving,
                    extra: extra.clone(),
                    pieces: vec![(t - dt, t, VehiclePiece::Lin { velocity })],
                });
                if arriving {
                    // Re-anchor to the floor the ride reached. A stop
                    // between waypoints leaves the vehicle off its path —
                    // and the next goto says so instead of guessing.
                    *at = waypoints
                        .iter()
                        .position(|wp| (wp - *position).norm() <= 1e-3);
                    if let Some(i) = *at {
                        *target = i;
                    }
                }
                break;
            }
        }
        // A part a conveyor already moved this tick is that conveyor's; one
        // device per part per tick keeps the bake single-valued.
        let advected: Vec<String> = moved.iter().map(|(n, _, _)| n.clone()).collect();
        let riders = self.apply_vehicle_moves(vehicle_moves, &advected)?;
        // Objects that stopped riding (zone exit / device stop / vehicle
        // arrival) settle into a hold at their current pose.
        let moved_names: Vec<&String> = moved
            .iter()
            .map(|(n, _, _)| n)
            .chain(riders.iter())
            .collect();
        let settled: Vec<(String, Isometry3<f64>)> = self
            .objects
            .iter()
            .filter(|track| {
                !moved_names.iter().any(|n| **n == track.name)
                    && matches!(
                        track.spans.last(),
                        Some(TrackSpan::Linear { t1, .. } | TrackSpan::Pivot { t1, .. }) if *t1 < t
                    )
            })
            .map(|track| track.name.clone())
            .filter_map(|name| {
                self.world
                    .obstacles()
                    .iter()
                    .find(|o| o.name == name)
                    .map(|o| (name.clone(), o.pose))
            })
            .collect();
        for (name, pose) in settled {
            let t_stop = match self
                .objects
                .iter()
                .find(|tr| tr.name == name)
                .and_then(|tr| tr.spans.last())
            {
                Some(TrackSpan::Linear { t1, .. } | TrackSpan::Pivot { t1, .. }) => *t1,
                _ => t,
            };
            let track = self.object_track_at(&name, pose, t_stop);
            track.spans.push(TrackSpan::Hold {
                t0: t_stop,
                t1: t_stop,
                pose,
            });
        }
        self.return_to_magazines(returned);
        self.feed_from_sources(dt);
        // Legs after the vehicles: a foot target is solved against where
        // the body is *now*.
        self.advance_gaits()?;
        self.advance_spins();
        self.follow_tracked_parts()?;
        self.check_rider_collisions()?;
        self.check_robot_collisions()?;
        Ok(())
    }

    /// The riders' aisle check: a robot on a vehicle that moved this tick
    /// — its links and what it holds — against everything the vehicle does
    /// not carry. After the legs and any track have been solved, so it
    /// sees the robot where it actually is.
    fn check_rider_collisions(&mut self) -> Result<(), SeqError> {
        let moving = std::mem::take(&mut self.moving);
        for (vehicle, carried) in &moving {
            for r in 0..self.world.robots().len() {
                let rides = self.world.robots()[r]
                    .mount
                    .as_ref()
                    .is_some_and(|m| &m.device == vehicle);
                if !rides {
                    continue;
                }
                // A walking rider's feet stand on walkable surfaces by
                // design: the treads join the "carried" exclusion the way
                // the floor was never an obstacle. An AMR's arm gets no
                // such pass — walkable only excuses the machine walking
                // on it.
                let mut skip = carried.clone();
                if self.robots[r].gait.is_some() {
                    skip.extend(
                        self.world
                            .obstacles()
                            .iter()
                            .filter(|o| o.walkable)
                            .map(|o| o.name.clone()),
                    );
                }
                if let Some((part, obstacle)) = self
                    .world
                    .rider_obstacle_contacts(r, &skip)
                    .into_iter()
                    .next()
                {
                    return Err(SeqError::RiderCollision {
                        t: self.t,
                        vehicle: vehicle.clone(),
                        robot: self.world.robots()[r].name.clone(),
                        part,
                        obstacle,
                    });
                }
            }
        }
        Ok(())
    }

    /// Sinks: put what reached them back in the magazine it came from.
    fn return_to_magazines(&mut self, returned: Vec<(usize, String)>) {
        for (source, object) in returned {
            let Some(DeviceRuntime::Source {
                pool,
                park,
                pitch,
                parked,
                ..
            }) = self.devices.get_mut(source)
            else {
                continue;
            };
            let Some(i) = pool.iter().position(|n| *n == object) else {
                // Something else drifted through the sink; a sink only owns
                // its source's carriers.
                continue;
            };
            if parked[i] {
                continue;
            }
            parked[i] = true;
            let slot = Isometry3::from_parts(
                nalgebra::Translation3::from(park.translation.vector + *pitch * i as f64),
                park.rotation,
            );
            self.teleport_object(&object, slot, true);
        }
    }

    /// Sources: release the next waiting carrier when one is due.
    fn feed_from_sources(&mut self, dt: f64) {
        for i in 0..self.devices.len() {
            let (object, pose, lane_off) = {
                let DeviceRuntime::Source {
                    pool,
                    pose,
                    interval,
                    running,
                    due,
                    parked,
                    lane,
                    ..
                } = &mut self.devices[i]
                else {
                    continue;
                };
                if !*running {
                    continue;
                }
                *due -= dt;
                if *due > 1e-12 {
                    continue;
                }
                let Some(next) = parked.iter().position(|p| *p) else {
                    // Magazine empty: stay due, so the next return feeds
                    // immediately rather than waiting out another interval.
                    *due = 0.0;
                    continue;
                };
                parked[next] = false;
                let lane = *lane;
                let one_shot = *interval <= 0.0;
                if one_shot {
                    // An indexing feeder: one carrier per Start. Sequences
                    // that name the carrier each step takes need supply to
                    // be demand-driven — on a timer, a carrier released
                    // while the cell is busy elsewhere goes past unclaimed
                    // and every later step is off by one.
                    *running = false;
                } else {
                    *due = *interval;
                }
                (pool[next].clone(), *pose, one_shot.then_some(lane))
            };
            self.teleport_object(&object, pose, false);
            if let Some(lane) = lane_off {
                let t = self.t;
                self.set_lane(lane, t, false);
            }
        }
    }

    /// Records an instantaneous jump: whatever span was open closes here and
    /// a hold at the new pose begins. Teleports are how a magazine takes a
    /// carrier back and how a source puts one on the line — the alternative,
    /// creating and destroying objects, has no place in a timeline whose
    /// object tracks are a fixed named set.
    fn teleport_object(&mut self, name: &str, to: Isometry3<f64>, stowed: bool) {
        let t = self.t;
        let from = self
            .world
            .obstacles()
            .iter()
            .find(|o| o.name == name)
            .map(|o| o.pose);
        let Some(from) = from else { return };
        self.world
            .set_obstacle_pose(name, to)
            .expect("teleported obstacle exists");
        let track = self.object_track_at(name, from, t);
        if let Some(open) = track.spans.last_mut() {
            let end = open.end_mut();
            if *end < t {
                *end = t;
            }
        }
        track.spans.push(if stowed {
            TrackSpan::Stowed {
                t0: t,
                t1: t,
                pose: to,
            }
        } else {
            TrackSpan::Hold {
                t0: t,
                t1: t,
                pose: to,
            }
        });
    }

    /// Conveyor tracking: re-solve each latched arm so this tick's
    /// commanded pose is the nominal one carried by the part's motion since
    /// the latch. Runs after the devices have moved the world, so the
    /// robots see the parts where they are *now*; robots resolve in scene
    /// order (deterministic).
    fn follow_tracked_parts(&mut self) -> Result<(), SeqError> {
        for r in 0..self.robots.len() {
            self.follow_tracked_part(r)?;
        }
        Ok(())
    }

    fn follow_tracked_part(&mut self, r: usize) -> Result<(), SeqError> {
        let Some(latch) = &self.robots[r].tracking else {
            return Ok(());
        };
        let (object, link, origin, frozen) =
            (latch.object.clone(), latch.link, latch.origin, latch.frozen);
        // Failures here happen during the world advance, between program
        // scans — attribute them to the program that latched the track.
        self.current = latch.program;
        // A grasped part is carried by the robot itself, so following it
        // would chase its own tail: the offset it had at the grasp stands.
        let offset = if frozen {
            latch.offset
        } else {
            let pose = self
                .world
                .obstacles()
                .iter()
                .find(|o| o.name == object)
                .map(|o| o.pose)
                .ok_or_else(|| SeqError::Action {
                    step: self.cur_step(),
                    name: self.cur_step_name(),
                    message: format!("tracked obstacle `{object}` disappeared"),
                })?;
            let offset = pose * origin.inverse();
            if let Some(latch) = &mut self.robots[r].tracking {
                latch.offset = offset;
            }
            offset
        };

        let rt = &self.robots[r];
        let nominal = self
            .world
            .fk_for(r, &rt.q_nom)
            .expect("q_nom has robot DOF")[link];
        let target = offset * nominal;
        // Warm start from what the robot did last tick plus this tick's
        // nominal increment: the solve then only absorbs one scan period of
        // part motion (and joints the offset cannot touch — the gripper —
        // follow the nominal exactly).
        let seed: Vec<f64> =
            rt.q.iter()
                .zip(&rt.q_nom)
                .zip(&rt.q_nom_prev)
                .map(|((commanded, nominal), previous)| commanded + (nominal - previous))
                .collect();
        let result = self
            .world
            .solve_ik_world_for(r, link, &target, &seed, &TRACK_IK)
            .expect("seed has robot DOF");
        if !result.converged {
            return Err(SeqError::Action {
                step: self.cur_step(),
                name: self.cur_step_name(),
                message: format!(
                    "tracking `{object}`: the part ran out of reach at t = {:.2}s \
                     ({:.3} mm / {:.4} rad short after {} iterations)",
                    self.t,
                    result.pos_error * 1e3,
                    result.rot_error,
                    result.iters
                ),
            });
        }
        let rt = &mut self.robots[r];
        let previous = rt.q.clone();
        rt.q = result.q;
        rt.q_nom_prev = rt.q_nom.clone();
        self.world
            .set_joint_positions_for(r, rt.q.clone())
            .expect("solved q has robot DOF");
        // The move's own waypoints know nothing about the offset, so a
        // tracked tick bakes itself (velocities by difference).
        let dt = self.options.dt;
        let velocity =
            rt.q.iter()
                .zip(&previous)
                .map(|(now, before)| (now - before) / dt)
                .collect();
        let (t, q) = (self.t, rt.q.clone());
        rt.append_waypoint(t, q, velocity);
        Ok(())
    }

    /// After every robot advanced: no two robots (or the objects they
    /// carry) may touch. Only pairs spanning two different robots count —
    /// self-collisions and static-obstacle contacts stay the planner's
    /// concern.
    fn check_robot_collisions(&self) -> Result<(), SeqError> {
        if self.world.robots().len() < 2 {
            return Ok(());
        }
        // Which robot a collider belongs to: its own links, or the robot
        // carrying it (attached ids are remapped to obstacles by Scene).
        let side = |id: botrail_collide::ColliderId| -> Option<(usize, String)> {
            match id {
                botrail_collide::ColliderId::Link { robot, link } => Some((
                    robot,
                    self.world.robots()[robot].model.links[link].name.clone(),
                )),
                botrail_collide::ColliderId::Obstacle(k) => {
                    let name = &self.world.obstacles()[k].name;
                    self.world.attachment(name).map(|a| (a.robot, name.clone()))
                }
                botrail_collide::ColliderId::Attached(_) => {
                    unreachable!("attached ids are remapped by Scene")
                }
            }
        };
        // The dedicated cross-robot path: a full scene check also prices
        // every self-collision and every obstacle contact — all discarded
        // here — and at line scale that bill *was* the tick.
        for pair in self.world.check_cross_robot_collisions() {
            if let (Some((ra, link_a)), Some((rb, link_b))) = (side(pair.a), side(pair.b)) {
                if ra != rb {
                    return Err(SeqError::RobotCollision {
                        t: self.t,
                        a: self.world.robots()[ra].name.clone(),
                        b: self.world.robots()[rb].name.clone(),
                        link_a,
                        link_b,
                    });
                }
            }
        }
        Ok(())
    }

    /// Latches robot `r` onto `object`: from here its nominal poses ride
    /// the part's motion.
    fn latch_track(&mut self, r: usize, object: &str, link: Option<&str>) -> Result<(), SeqError> {
        let err = |message: String| SeqError::Action {
            step: self.cur_step(),
            name: self.cur_step_name(),
            message,
        };
        let model = &self.world.robots()[r].model;
        let link = match link {
            Some(name) => model
                .link_index(name)
                .ok_or_else(|| err(format!("unknown link `{name}`")))?,
            // The wrist, not the fingertip: a pose says nothing about the
            // grip, so the solver must not be able to spend it.
            None => model.tool_mount_link(),
        };
        let origin = self
            .world
            .obstacles()
            .iter()
            .find(|o| o.name == object)
            .map(|o| o.pose)
            .ok_or_else(|| err(format!("unknown obstacle `{object}`")))?;
        let rt = &mut self.robots[r];
        rt.q_nom = rt.q.clone();
        rt.q_nom_prev = rt.q.clone();
        let program = self.current;
        rt.tracking = Some(TrackLatch {
            program,
            object: object.to_string(),
            link,
            origin,
            offset: Isometry3::identity(),
            frozen: false,
        });
        Ok(())
    }

    /// Drops robot `r`'s track; the robot keeps the configuration it is in,
    /// so the nominal frame is re-based onto it (releasing never moves the
    /// robot).
    fn release_track(&mut self, r: usize) {
        let rt = &mut self.robots[r];
        rt.tracking = None;
        rt.q_nom = rt.q.clone();
    }

    /// Extends (or opens) a constant-velocity span covering this tick.
    fn extend_linear_span(
        &mut self,
        name: &str,
        rest_pose: Isometry3<f64>,
        velocity: Vector3<f64>,
    ) {
        let t = self.t;
        let dt = self.options.dt;
        let track = self.object_track_at(name, rest_pose, t - dt);
        match track.spans.last_mut() {
            Some(TrackSpan::Linear {
                t1, velocity: v, ..
            }) if (*v - velocity).norm() < 1e-12 && (*t1 - (t - dt)).abs() < 1e-9 => {
                *t1 = t;
            }
            _ => {
                if let Some(open) = track.spans.last_mut() {
                    let end = open.end_mut();
                    if *end < t - dt {
                        *end = t - dt;
                    }
                }
                track.spans.push(TrackSpan::Linear {
                    t0: t - dt,
                    t1: t,
                    from: rest_pose,
                    velocity,
                });
            }
        }
    }

    /// Applies each travelling vehicle's tick motion to its body obstacles
    /// — composing the exact sub-tick pieces onto the world poses and
    /// recording them as Linear/Pivot spans — then runs the aisle check.
    /// Returns the names that rode a deck this tick (they settle like any
    /// other carried part when the vehicle stops).
    fn apply_vehicle_moves(
        &mut self,
        moves: Vec<VehicleMove>,
        advected: &[String],
    ) -> Result<Vec<String>, SeqError> {
        // Whatever sits on a deck travels exactly like the body does, so the
        // two are carried by the same code — the only difference is that the
        // load is worked out per tick and the body is fixed.
        let moves: Vec<(VehicleMove, Vec<String>)> = moves
            .into_iter()
            .map(|mv| {
                // A walking machine's deck is its body. The route stays on
                // the guide plane while the body tilts onto the grade and
                // rides up the steps, so a zone pinned to the route would
                // hold air on the way up and reach inside the machine on
                // the way down. Move the zone onto the body first.
                let walker = self.walker_on(&mv.name);
                let deck = walker.map(|r| {
                    let offset = self.robots[r].gait.as_ref().expect("walker").offset;
                    *self.world.robots()[r].base_pose() * (mv.frame * offset).inverse()
                });
                let riders: Vec<String> = match &mv.tray {
                    None => Vec::new(),
                    Some((zone, half)) => {
                        let zone = deck.map_or(*zone, |d| d * *zone);
                        self.world
                            .obstacles()
                            .iter()
                            .filter(|o| {
                                !mv.body.iter().any(|b| b == &o.name)
                                    && self.world.attachment(&o.name).is_none()
                                    && !advected.iter().any(|n| n == &o.name)
                                    && inside_zone(&zone, half, &o.pose)
                            })
                            .map(|o| o.name.clone())
                            .collect()
                    }
                };
                if let Some(r) = walker {
                    self.carry_on_body(r, &riders);
                }
                (mv, riders)
            })
            .collect();
        for (mv, riders) in &moves {
            // The vehicle's own frame is a track too — it is what places
            // its mounted sensors during playback (the body obstacles
            // carry only themselves).
            {
                let since = mv.pieces.first().map(|p| p.0).unwrap_or(0.0);
                let track = self.vehicle_track_at(&mv.name, mv.frame, since);
                let mut frame = mv.frame;
                for (tau0, tau1, piece) in &mv.pieces {
                    let from = frame;
                    frame = apply_piece(&from, piece, tau1 - tau0);
                    push_vehicle_span(&mut track.spans, from, *tau0, *tau1, piece);
                }
                if mv.parked {
                    // Arrival closes the track with a hold at the parked
                    // frame — the finalize stretch then holds, instead of
                    // integrating the last leg past its waypoint.
                    let end = mv.pieces.last().map(|p| p.1).unwrap_or(since);
                    track.spans.push(TrackSpan::Hold {
                        t0: end,
                        t1: end,
                        pose: frame,
                    });
                }
            }
            let (body, pieces) = (&mv.body, &mv.pieces);
            let on_body = self
                .walker_on(&mv.name)
                .map(|r| {
                    self.robots[r]
                        .gait
                        .as_ref()
                        .expect("walker")
                        .carried
                        .clone()
                })
                .unwrap_or_default();
            for member in body.iter().chain(riders) {
                // What the body carries is placed from the body's own FK
                // (`place_carried`), so the route's pieces must not move it
                // a second time — and its track follows the body link.
                if on_body.iter().any(|(name, _)| name == member) {
                    continue;
                }
                if self.world.attachment(member).is_some() {
                    // Grasped body parts are the robot's problem, like any
                    // other advection exclusion.
                    continue;
                }
                let Some(mut pose) = self
                    .world
                    .obstacles()
                    .iter()
                    .find(|o| &o.name == member)
                    .map(|o| o.pose)
                else {
                    continue;
                };
                for (tau0, tau1, piece) in pieces {
                    let from = pose;
                    pose = apply_piece(&from, piece, tau1 - tau0);
                    self.extend_vehicle_span(member, from, *tau0, *tau1, piece);
                }
                self.world
                    .set_obstacle_pose(member, pose)
                    .expect("body member exists");
            }
            // A robot riding this vehicle moves by the same pieces. Deriving
            // its base this way rather than recomposing `frame ∘ offset`
            // keeps it *exactly* rigid with the body it is bolted to, which
            // is what any check of the pair will ask.
            for r in 0..self.world.robots().len() {
                let rides = self.world.robots()[r]
                    .mount
                    .as_ref()
                    .is_some_and(|m| m.device == mv.name);
                if !rides {
                    continue;
                }
                // A walking body sways on top of its rigid ride; the ride
                // is what the vehicle advances and the track records.
                let body = self.robots[r]
                    .gait
                    .as_ref()
                    .map(|g| g.pitch * g.sway)
                    .unwrap_or_else(Isometry3::identity);
                let rise = self.robots[r].gait.as_ref().map(|g| g.rise).unwrap_or(0.0);
                let lift = nalgebra::Translation3::new(0.0, 0.0, rise);
                let mut base =
                    lift.inverse() * *self.world.robots()[r].base_pose() * body.inverse();
                for (tau0, tau1, piece) in pieces {
                    let from = base;
                    base = apply_piece(&from, piece, tau1 - tau0);
                    if let Some(spans) = self.robots[r].base.as_mut() {
                        push_vehicle_span(spans, from, *tau0, *tau1, piece);
                    }
                }
                self.world.set_robot_base_pose_for(r, lift * base * body);
            }
        }
        for (mv, _) in &moves {
            if let Some(r) = self.walker_on(&mv.name) {
                self.place_carried(r);
            }
        }
        self.check_vehicle_collisions(&moves)?;
        self.check_vehicle_robot_collisions(&moves)?;
        self.moving = moves
            .iter()
            .map(|(mv, riders)| {
                (
                    mv.name.clone(),
                    mv.body
                        .iter()
                        .chain(riders)
                        .chain(mv.extra.iter())
                        .cloned()
                        .collect(),
                )
            })
            .collect();
        Ok(moves.into_iter().flat_map(|(_, riders)| riders).collect())
    }

    /// The walking robot a vehicle carries, if any — the machine whose
    /// *body* is that vehicle's deck.
    fn walker_on(&self, vehicle: &str) -> Option<usize> {
        (0..self.world.robots().len()).find(|&r| {
            self.robots[r].gait.is_some()
                && self.world.robots()[r]
                    .mount
                    .as_ref()
                    .is_some_and(|m| m.device == vehicle)
        })
    }

    /// Bind this tick's deck load to the machine's body: an item newly on
    /// the deck takes the offset it rests at, and follows the body link
    /// from here — the same span a grasped object gets, so the studio, the
    /// USD bake and every resample place it off the body's own FK, tilt
    /// and ride included. One that has left the deck is simply dropped.
    fn carry_on_body(&mut self, r: usize, riders: &[String]) {
        let t = self.t;
        let link = self.robots[r].gait.as_ref().expect("walker").gait.body;
        let body = self.world.link_poses_for(r)[link];
        let held = self.robots[r]
            .gait
            .as_ref()
            .expect("walker")
            .carried
            .clone();
        let mut carried: Vec<(String, Isometry3<f64>)> = Vec::with_capacity(riders.len());
        for name in riders {
            if let Some((_, offset)) = held.iter().find(|(n, _)| n == name) {
                carried.push((name.clone(), *offset));
                if let Some(track) = self.objects.iter_mut().find(|o| &o.name == name) {
                    if let Some(open) = track.spans.last_mut() {
                        if matches!(open, TrackSpan::Follow { .. }) {
                            *open.end_mut() = t;
                        }
                    }
                }
                continue;
            }
            let Some(pose) = self
                .world
                .obstacles()
                .iter()
                .find(|o| &o.name == name)
                .map(|o| o.pose)
            else {
                continue;
            };
            let offset = body.inverse() * pose;
            carried.push((name.clone(), offset));
            let track = self.object_track_at(name, pose, t);
            if let Some(open) = track.spans.last_mut() {
                let end = open.end_mut();
                if *end < t {
                    *end = t;
                }
            }
            track.spans.push(TrackSpan::Follow {
                t0: t,
                t1: t,
                robot: r,
                link,
                offset,
            });
        }
        self.robots[r].gait.as_mut().expect("walker").carried = carried;
    }

    /// Put what the machine carries where its body now is. Called wherever
    /// the base moves — the load is rigid with the body, not with the
    /// route, and every check downstream reads the world pose.
    fn place_carried(&mut self, r: usize) {
        let Some(gr) = self.robots[r].gait.as_ref() else {
            return;
        };
        if gr.carried.is_empty() {
            return;
        }
        let link = gr.gait.body;
        let carried = gr.carried.clone();
        let body = self.world.link_poses_for(r)[link];
        for (name, offset) in carried {
            let _ = self.world.set_obstacle_pose(&name, body * offset);
        }
    }

    /// Extends (or opens) the exact sub-tick span covering one vehicle
    /// piece for a carried obstacle.
    fn extend_vehicle_span(
        &mut self,
        name: &str,
        from: Isometry3<f64>,
        tau0: f64,
        tau1: f64,
        piece: &VehiclePiece,
    ) {
        if tau1 - tau0 < 1e-12 {
            return;
        }
        let track = self.object_track_at(name, from, tau0);
        push_vehicle_span(&mut track.spans, from, tau0, tau1, piece);
    }

    /// A travelling vehicle's body must clear everything it does not carry
    /// — the aisle check. Only vehicles that moved this tick are checked,
    /// so a parked vehicle touching its dock guide is legitimate authoring.
    /// The load is checked too, and counts as carried: a part on the deck
    /// may touch the body it rides on, but not the shelf it passes.
    fn check_vehicle_collisions(
        &self,
        moves: &[(VehicleMove, Vec<String>)],
    ) -> Result<(), SeqError> {
        for (mv, riders) in moves {
            let (vehicle, body) = (&mv.name, &mv.body);
            let carried = |name: &String| {
                body.contains(name) || riders.contains(name) || mv.extra.contains(name)
            };
            // A walking machine stands on walkable surfaces by design —
            // nobody collision-checks a floor against the machine standing
            // on it. Wheeled vehicles get no such pass: an AGV driven into
            // a staircase is exactly what the aisle check is for.
            let walks = (0..self.world.robots().len()).any(|r| {
                self.robots[r].gait.is_some()
                    && self.world.robots()[r]
                        .mount
                        .as_ref()
                        .is_some_and(|m| &m.device == vehicle)
            });
            for member in body.iter().chain(riders) {
                let Some((k, obstacle)) = self
                    .world
                    .obstacles()
                    .iter()
                    .enumerate()
                    .find(|(_, o)| &o.name == member)
                else {
                    continue;
                };
                if !obstacle.enabled || self.world.attachment(member).is_some() {
                    continue;
                }
                let collider = &self.world.obstacle_colliders[k];
                for (j, other) in self.world.obstacles().iter().enumerate() {
                    if j == k
                        || !other.enabled
                        || carried(&other.name)
                        || (walks && other.walkable)
                        || self.world.attachment(&other.name).is_some()
                    {
                        continue;
                    }
                    if collider.intersects(
                        &obstacle.pose,
                        &self.world.obstacle_colliders[j],
                        &other.pose,
                    ) {
                        return Err(SeqError::VehicleCollision {
                            t: self.t,
                            vehicle: vehicle.clone(),
                            body: member.clone(),
                            obstacle: other.name.clone(),
                        });
                    }
                }
            }
        }
        Ok(())
    }

    /// A travelling vehicle must also clear every robot that is not its
    /// passenger — the airspace the obstacle-only aisle check cannot see:
    /// the arm a path was taught too close to, the body a drone would
    /// cross over. Only vehicles that moved this tick are checked, against
    /// links of robots that do not ride them.
    fn check_vehicle_robot_collisions(
        &self,
        moves: &[(VehicleMove, Vec<String>)],
    ) -> Result<(), SeqError> {
        for (mv, riders) in moves {
            let members: Vec<String> = mv.body.iter().chain(riders).cloned().collect();
            if members.is_empty() {
                continue;
            }
            for r in 0..self.world.robots().len() {
                let rides = self.world.robots()[r]
                    .mount
                    .as_ref()
                    .is_some_and(|m| m.device == mv.name);
                if rides {
                    continue;
                }
                if let Some((link, body)) = self
                    .world
                    .robot_contacts_among(r, &members)
                    .into_iter()
                    .next()
                {
                    return Err(SeqError::VehicleRobotCollision {
                        t: self.t,
                        vehicle: mv.name.clone(),
                        body,
                        robot: self.world.robots()[r].name.clone(),
                        link,
                    });
                }
            }
        }
        Ok(())
    }

    /// Fixes a lift's cargo at dispatch: every vehicle whose reference
    /// point stands in the capture zone (its whole body must be aboard,
    /// or the boarding refuses by name), and every loose obstacle whose
    /// origin does. An elevator moves with the doors closed — nothing
    /// joins or leaves mid-ride.
    fn capture_lift(&mut self, device: &str) -> Result<(), String> {
        let Some((zone_pose, zone_half, axis, position, car)) =
            self.devices.iter().find_map(|d| match d {
                DeviceRuntime::Lift {
                    name,
                    zone_pose,
                    zone_half,
                    axis,
                    position,
                    car,
                    ..
                } if name == device => {
                    Some((*zone_pose, *zone_half, *axis, *position, car.clone()))
                }
                _ => None,
            })
        else {
            return Err(format!("unknown lift `{device}`"));
        };
        let mut zone = zone_pose;
        zone.translation.vector += axis * position;
        let mut vehicles: Vec<String> = Vec::new();
        let mut aboard_bodies: Vec<String> = Vec::new();
        for d in &self.devices {
            let DeviceRuntime::Vehicle {
                name,
                body,
                position,
                legs,
                ..
            } = d
            else {
                continue;
            };
            let here = Isometry3::translation(position.x, position.y, position.z);
            if !inside_zone(&zone, &zone_half, &here) {
                continue;
            }
            if !legs.is_empty() {
                return Err(format!(
                    "vehicle `{name}` is still travelling; wait for its device_done \
                     before moving lift `{device}`"
                ));
            }
            for member in body {
                let aboard = self
                    .world
                    .obstacles()
                    .iter()
                    .find(|o| &o.name == member)
                    .map(|o| inside_zone(&zone, &zone_half, &o.pose))
                    .unwrap_or(true);
                if !aboard {
                    return Err(format!(
                        "vehicle `{name}` is boarding lift `{device}` half out: \
                         `{member}` stands outside the capture zone"
                    ));
                }
            }
            vehicles.push(name.clone());
            aboard_bodies.extend(body.iter().cloned());
        }
        let objects: Vec<String> = self
            .world
            .obstacles()
            .iter()
            .filter(|o| {
                !car.iter().any(|c| c == &o.name)
                    && !aboard_bodies.iter().any(|b| b == &o.name)
                    && self.world.attachment(&o.name).is_none()
                    && inside_zone(&zone, &zone_half, &o.pose)
            })
            .map(|o| o.name.clone())
            .collect();
        for d in &mut self.devices {
            if let DeviceRuntime::Lift {
                name,
                cargo_objects,
                cargo_vehicles,
                ..
            } = d
            {
                if name == device {
                    *cargo_objects = objects;
                    *cargo_vehicles = vehicles;
                    break;
                }
            }
        }
        Ok(())
    }

    /// Evaluates every pseudo-sensor at the current world state and records
    /// edges on its input lane.
    fn update_sensors(&mut self) {
        if self.sensors.is_empty() {
            return;
        }
        let needs_links = self.sensors.iter().any(|s| {
            matches!(
                s.watch,
                SensorWatch::Robot | SensorWatch::Robots(_) | SensorWatch::All
            ) || matches!(s.anchor, SensorAnchor::Link { .. })
        });
        let link_poses = needs_links.then(|| self.world.all_link_poses());
        let t = self.t;
        let mut edges = Vec::new();
        for sensor in &self.sensors {
            // An anchored sensor's geometry is authored in its anchor's
            // frame, so its world pose is re-resolved every tick — that is
            // the whole difference between a fixture and one that travels
            // (a deck sensor on a vehicle, a vision camera on a wrist).
            let pose = match &sensor.anchor {
                SensorAnchor::Fixture => sensor.pose,
                SensorAnchor::Vehicle(d) => match self.devices.get(*d) {
                    Some(DeviceRuntime::Vehicle {
                        position, heading, ..
                    }) => vehicle_frame(position, *heading) * sensor.pose,
                    _ => sensor.pose,
                },
                SensorAnchor::Link { robot, link } => link_poses
                    .as_ref()
                    .map(|lp| lp[*robot][*link] * sensor.pose)
                    .unwrap_or(sensor.pose),
            };
            let mut value = false;
            let watch_objects: Option<&[String]> = match &sensor.watch {
                SensorWatch::Objects(names) => Some(names),
                SensorWatch::AllObjects | SensorWatch::All => None,
                SensorWatch::Robot | SensorWatch::Robots(_) => Some(&[]),
            };
            if !matches!(sensor.watch, SensorWatch::Robot | SensorWatch::Robots(_)) {
                for (target, (obstacle, collider)) in self
                    .world
                    .obstacles()
                    .iter()
                    .zip(self.world.obstacle_colliders.iter())
                    .enumerate()
                {
                    if !obstacle.enabled {
                        continue;
                    }
                    if let Some(names) = watch_objects {
                        if !names.iter().any(|n| n == &obstacle.name) {
                            continue;
                        }
                    }
                    if sensor.exclude.contains(&target) {
                        continue;
                    }
                    if sensor.collider.intersects(&pose, collider, &obstacle.pose) {
                        if sensor.occlusion && self.occluded(&pose, target, &sensor.exclude) {
                            continue;
                        }
                        value = true;
                        break;
                    }
                }
            }
            if !value {
                if let (
                    Some(poses),
                    SensorWatch::Robot | SensorWatch::Robots(_) | SensorWatch::All,
                ) = (&link_poses, &sensor.watch)
                {
                    let watched = |name: &str| match &sensor.watch {
                        SensorWatch::Robots(names) => names.iter().any(|n| n == name),
                        _ => true,
                    };
                    value = self.world.robots().iter().zip(poses).any(|(r, lp)| {
                        watched(&r.name)
                            && botrail_collide::robot_intersects(
                                r.collider(),
                                lp,
                                &sensor.collider,
                                &pose,
                            )
                    });
                }
            }
            edges.push((sensor.lane, value));
        }
        for (lane, value) in edges {
            self.set_lane(lane, t, value);
        }
    }

    /// Is `target`'s origin hidden from `sensor` behind another enabled
    /// obstacle? One ray, origin to origin — coarse but deterministic
    /// (design/design-camera.md §3): a body half in view can read either
    /// way, and robot links never occlude. `skip` never blocks the view
    /// (a vehicle-mounted field's own chassis).
    fn occluded(&self, sensor: &Isometry3<f64>, target: usize, skip: &[usize]) -> bool {
        let origin = sensor.translation.vector;
        let goal = self.world.obstacles()[target].pose.translation.vector;
        let dir = goal - origin;
        if dir.norm_squared() < 1e-12 {
            return false;
        }
        for (j, (obstacle, collider)) in self
            .world
            .obstacles()
            .iter()
            .zip(self.world.obstacle_colliders.iter())
            .enumerate()
        {
            if j == target || !obstacle.enabled || skip.contains(&j) {
                continue;
            }
            let inv = obstacle.pose.inverse();
            let local_origin = inv * nalgebra::Point3::from(origin);
            let local_dir = inv.transform_vector(&dir);
            // `dir` is unnormalized, so toi is the fraction of the way to
            // the target: any hit short of it blocks the view.
            if let Some(toi) = collider.cast_local_ray(&local_origin, &local_dir, 1.0) {
                if toi < 1.0 - 1e-6 {
                    return true;
                }
            }
        }
        false
    }

    /// Records an edge on a signal lane when the value changes. Every
    /// write goes through here — sensor evaluation, a program's `set`, a
    /// device's running state — so a lane a fault pins is guarded in one
    /// place: the write is dropped and the lane keeps its forced level.
    fn set_lane(&mut self, lane: usize, t: f64, value: bool) {
        if self.forced.iter().any(|(l, _)| *l == lane) {
            return;
        }
        let track = &mut self.signals[lane];
        let current = track.edges.last().map(|(_, v)| *v).unwrap_or(false);
        if current != value {
            track.edges.push((t, value));
        }
    }

    fn lane_index(&self, name: &str) -> Option<usize> {
        self.signals.iter().position(|s| s.name == name)
    }

    fn lane_value(&self, lane: usize) -> bool {
        self.signals[lane]
            .edges
            .last()
            .map(|(_, v)| *v)
            .unwrap_or(false)
    }

    fn prev_lane_value(&self, lane: usize) -> bool {
        self.programs[self.current]
            .prev_signals
            .get(lane)
            .copied()
            .unwrap_or(false)
    }

    /// Closes a program's scan for its edge conditions: everything it
    /// compares against next time is the world as it stands right now.
    fn close_scan(&mut self, program: usize) {
        let snapshot: Vec<bool> = (0..self.signals.len())
            .map(|l| self.lane_value(l))
            .collect();
        self.programs[program].prev_signals = snapshot;
    }

    /// Fires transitions that hold at the current time for the current
    /// program, chaining through instantaneous steps (bounded per tick).
    /// A step's exits are tried in authored order (SFC's left-to-right
    /// priority); the first that holds wins, and on a branching step the
    /// winning arm is recorded on the timeline.
    fn advance_through_ready_steps(&mut self) -> Result<(), SeqError> {
        let mut chain = 0usize;
        while !self.programs[self.current].finished() {
            let p = &self.programs[self.current];
            let exits = p.flat[p.step].exits.clone();
            let Some((arm, target)) = exits
                .iter()
                .enumerate()
                .find(|(_, (condition, _))| self.condition_holds(condition))
                .map(|(j, (_, target))| (j, *target))
            else {
                return Ok(());
            };
            let p = &self.programs[self.current];
            if let Some(select) = p.flat[p.step].select {
                self.branches.push(BranchTaken {
                    sequence: p.sequence.name.clone(),
                    step: p.flat[p.step].name.clone(),
                    select,
                    arm,
                });
            }
            self.exit_step();
            let p = &mut self.programs[self.current];
            p.step = target;
            if p.finished() {
                return Ok(());
            }
            chain += 1;
            if chain > self.options.immediate_chain_limit {
                return Err(SeqError::ImmediateLoop {
                    step: self.cur_step(),
                    name: self.cur_step_name(),
                    limit: self.options.immediate_chain_limit,
                });
            }
            self.enter_step()?;
        }
        Ok(())
    }

    fn condition_holds(&self, condition: &Condition) -> bool {
        match condition {
            Condition::Immediately => true,
            Condition::Done => self.programs[self.current]
                .move_ends
                .iter()
                .all(|end| self.t >= end - 1e-9),
            Condition::RobotDone { robot } => self
                .world
                .robot_index(robot)
                .map(|r| self.robots[r].active.is_none())
                .unwrap_or(true),
            Condition::Elapsed { seconds } => {
                self.t - self.programs[self.current].entered_at >= seconds - 1e-9
            }
            Condition::Signal { name, value } => {
                let current = self
                    .lane_index(name)
                    .map(|lane| self.lane_value(lane))
                    .unwrap_or(false);
                current == *value
            }
            // Edges compare against this program's own previous scan, so
            // a transition raised anywhere — before or after this
            // program's slot in the scan order — is caught exactly once.
            Condition::Rising { name } => match self.lane_index(name) {
                Some(lane) => !self.prev_lane_value(lane) && self.lane_value(lane),
                None => false,
            },
            Condition::Falling { name } => match self.lane_index(name) {
                Some(lane) => self.prev_lane_value(lane) && !self.lane_value(lane),
                None => false,
            },
            Condition::DeviceDone { device } => self.devices.iter().any(|d| match d {
                DeviceRuntime::Axis {
                    name,
                    position,
                    target,
                    ..
                } => name == device && (target - position).abs() < 1e-9,
                // Parked at the commanded stop (in-position).
                DeviceRuntime::Lift {
                    name,
                    position,
                    target,
                    ..
                } => name == device && (target - position).abs() < 1e-9,
                // Parked at the commanded station (in-position).
                DeviceRuntime::Vehicle { name, legs, .. } => name == device && legs.is_empty(),
                // In-position for a conveyor means "no fixed advance
                // pending" — the await for `Advance`.
                DeviceRuntime::Conveyor {
                    name, remaining, ..
                } => name == device && remaining.is_none(),
                // Nothing to be "done" with: these run continuously.
                DeviceRuntime::Source { .. } | DeviceRuntime::Sink { .. } => false,
            }),
            Condition::All(cs) => cs.iter().all(|c| self.condition_holds(c)),
            Condition::Any(cs) => cs.iter().any(|c| self.condition_holds(c)),
        }
    }

    fn enter_step(&mut self) -> Result<(), SeqError> {
        let name = self.cur_step_name();
        let program = &mut self.programs[self.current];
        program.entered_at = self.t;
        program.move_ends.clear();
        program.open_span = self.step_spans.len();
        self.step_spans.push(StepSpan {
            name,
            start: self.t,
            end: self.t,
            sequence: program.sequence.name.clone(),
            step: program.step,
        });
        let step = self.programs[self.current].step;
        for action in self.programs[self.current].flat[step].actions.clone() {
            self.fire(&action)?;
        }
        Ok(())
    }

    fn exit_step(&mut self) {
        // Close *this program's* span: another program may have opened one
        // since, so "the last span" stopped being ours.
        let span = self.programs[self.current].open_span;
        if let Some(span) = self.step_spans.get_mut(span) {
            span.end = self.t;
        }
        // Hold every robot's last configuration up to the transition
        // instant, so the baked tracks stay exact through waits.
        let t = self.t;
        for rt in &mut self.robots {
            let (q, zeros) = (rt.q.clone(), vec![0.0; rt.q.len()]);
            rt.append_waypoint(t, q, zeros);
        }
    }

    /// Resolves an action's robot reference; validation already vetted it,
    /// so failures here are defensive.
    fn action_robot(&self, robot: &Option<String>) -> Result<usize, SeqError> {
        self.world
            .resolve_seq_robot(robot)
            .map_err(|message| SeqError::Action {
                step: self.cur_step(),
                name: self.cur_step_name(),
                message,
            })
    }

    fn fire(&mut self, action: &Action) -> Result<(), SeqError> {
        let step_index = self.cur_step();
        let step_name = self.cur_step_name();
        let err = move |message: String| SeqError::Action {
            step: step_index,
            name: step_name.clone(),
            message,
        };
        match action {
            Action::StartMotion { motion } => {
                let owner = self
                    .world
                    .motions()
                    .iter()
                    .find(|m| &m.name == motion)
                    .map(|m| m.robot)
                    .ok_or_else(|| err(format!("unknown motion `{motion}`")))?;
                // A plan is baked in world coordinates when it starts, so a
                // base that moves underneath it invalidates every waypoint.
                // Same rule as planning under a conveyor track, and the same
                // escape: ramps are re-evaluated per tick, so those may run
                // while the machine drives (stow the arm and go).
                if let Some(mount) = self.world.robots()[owner].mount.clone() {
                    let travelling = self.devices.iter().any(|d| match d {
                        DeviceRuntime::Vehicle { name, legs, .. } => {
                            *name == mount.device && !legs.is_empty()
                        }
                        _ => false,
                    });
                    if travelling {
                        return Err(err(format!(
                            "motion `{motion}` cannot start while `{}` is driving: plans are \
                             baked in world coordinates, so wait for device_done first \
                             (a ramp may run while travelling)",
                            mount.device
                        )));
                    }
                }
                // Plan against the world as it stands *now*: current q,
                // moved obstacles, live grasps — the other robots are
                // frozen collision bodies at their current configuration.
                self.world
                    .set_joint_positions_for(owner, self.robots[owner].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                let limits = crate::motion::traj_limits(&self.world.robots()[owner].model);
                let planned = self
                    .world
                    .plan_motion(motion, &self.options.plan, &limits)
                    .map_err(|e| SeqError::PlanFailed {
                        step: self.cur_step(),
                        name: self.cur_step_name(),
                        message: e.to_string(),
                    })?;
                let traj = planned.trajectory;
                let rt = &mut self.robots[owner];
                for i in 0..traj.times.len() {
                    rt.append_waypoint(
                        self.t + traj.times[i],
                        traj.positions[i].clone(),
                        traj.velocities[i].clone(),
                    );
                }
                let end = self.t + traj.duration();
                rt.planned.push(PlannedMove {
                    sequence: self.programs[self.current].sequence.name.clone(),
                    step: step_index,
                    motion: Some(motion.clone()),
                    segments: planned.segments,
                    duration: traj.duration(),
                    feed_report: None,
                    process_spans: Vec::new(),
                });
                self.programs[self.current].move_ends.push(end);
                rt.moves.push(StepSpan {
                    name: motion.clone(),
                    start: self.t,
                    end,
                    sequence: self.programs[self.current].sequence.name.clone(),
                    step: step_index,
                });
                // Joints follow the trajectory tick by tick (advance_world),
                // so mid-motion sensors see the true robot state.
                rt.active = Some(ActiveMove::Traj {
                    start: self.t,
                    traj,
                });
            }
            Action::StartToolpath { robot, toolpath } => {
                let r = self.action_robot(robot)?;
                // Same rule as StartMotion: the bake is world-frame, so a
                // base driving underneath it invalidates every sample.
                if let Some(mount) = self.world.robots()[r].mount.clone() {
                    let travelling = self.devices.iter().any(|d| match d {
                        DeviceRuntime::Vehicle { name, legs, .. } => {
                            *name == mount.device && !legs.is_empty()
                        }
                        _ => false,
                    });
                    if travelling {
                        return Err(err(format!(
                            "toolpath `{toolpath}` cannot start while `{}` is driving: the \
                             bake is world-frame, so wait for device_done first",
                            mount.device
                        )));
                    }
                }
                // Bake against the world as it stands now.
                self.world
                    .set_joint_positions_for(r, self.robots[r].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                let limits = crate::motion::traj_limits(&self.world.robots()[r].model);
                let planned = self
                    .world
                    .plan_toolpath(toolpath, r, None, &self.options.toolpath)
                    .map_err(|e| SeqError::PlanFailed {
                        step: self.cur_step(),
                        name: self.cur_step_name(),
                        message: e.to_string(),
                    })?;
                // The follow starts at the path's own first sample; when
                // the robot stands elsewhere, a collision-free joint-space
                // approach is planned and prepended — the PLC picture of a
                // start command ("go do it"), not a teleport.
                let current = self.robots[r].q.clone();
                let start = planned.path.first().expect("non-empty path").clone();
                let apart = current
                    .iter()
                    .zip(&start)
                    .map(|(a, b)| (a - b) * (a - b))
                    .sum::<f64>()
                    .sqrt();
                let mut segments = Vec::with_capacity(planned.path.len());
                let traj = if apart > 1e-6 {
                    let (lower, upper) = self.world.robots()[r].model.sampling_bounds();
                    let space = botrail_plan::JointSpace { lower, upper };
                    let mut is_valid = |q: &[f64]| self.world.is_state_valid_for(r, q);
                    let approach_path = botrail_plan::plan(
                        &space,
                        &current,
                        &start,
                        &mut is_valid,
                        &self.options.plan,
                    )
                    .map_err(|e| SeqError::PlanFailed {
                        step: self.cur_step(),
                        name: self.cur_step_name(),
                        message: format!("approach to toolpath `{toolpath}`: {e}"),
                    })?;
                    let approach = botrail_traj::time_parameterize(
                        &approach_path,
                        &limits,
                        &botrail_traj::TimingOptions::default(),
                    )
                    .map_err(|e| err(e.to_string()))?;
                    segments.push(crate::motion::PlannedSegment {
                        kind: crate::motion::SegmentKind::Joint,
                        waypoints: approach_path,
                        tcp_speed: None,
                    });
                    crate::motion::concatenate(approach, planned.trajectory.clone())
                } else {
                    planned.trajectory.clone()
                };
                // Per-sample linear segments carry the commanded speed of
                // their interval, which is what script export renders.
                for (pair, sample) in planned.path.windows(2).zip(planned.samples.iter().skip(1)) {
                    segments.push(crate::motion::PlannedSegment {
                        kind: crate::motion::SegmentKind::CartesianLine,
                        waypoints: pair.to_vec(),
                        tcp_speed: sample.feed.or(self.options.toolpath.rapid_speed),
                    });
                }
                let rt = &mut self.robots[r];
                for i in 0..traj.times.len() {
                    rt.append_waypoint(
                        self.t + traj.times[i],
                        traj.positions[i].clone(),
                        traj.velocities[i].clone(),
                    );
                }
                let end = self.t + traj.duration();
                // Spraying intervals in timeline time: the interval *into*
                // sample i sprays when its move does (a feed move — with a
                // brush, in a toolpath that names any); the approach, if
                // any, shifted everything by its duration. Consecutive
                // intervals of the same brush merge into one span.
                let offset = self.t + (traj.duration() - planned.trajectory.duration());
                let path_def = self.world.toolpath(toolpath).cloned();
                let mut process_spans: Vec<ProcessSpan> = Vec::new();
                for (i, sample) in planned.samples.iter().enumerate().skip(1) {
                    let Some(def) = &path_def else { break };
                    if !def.move_sprays(sample.move_index) {
                        continue;
                    }
                    let brush = def.moves[sample.move_index].brush.clone();
                    let (a, b) = (
                        offset + planned.sample_times[i - 1],
                        offset + planned.sample_times[i],
                    );
                    match process_spans.last_mut() {
                        Some(last) if (last.end - a).abs() < 1e-9 && last.brush == brush => {
                            last.end = b
                        }
                        _ => process_spans.push(ProcessSpan {
                            start: a,
                            end: b,
                            brush,
                        }),
                    }
                }
                rt.planned.push(PlannedMove {
                    sequence: self.programs[self.current].sequence.name.clone(),
                    step: step_index,
                    motion: Some(toolpath.clone()),
                    segments,
                    duration: traj.duration(),
                    feed_report: Some(planned.feed_report.clone()),
                    process_spans,
                });
                self.programs[self.current].move_ends.push(end);
                rt.moves.push(StepSpan {
                    name: toolpath.clone(),
                    start: self.t,
                    end,
                    sequence: self.programs[self.current].sequence.name.clone(),
                    step: step_index,
                });
                rt.active = Some(ActiveMove::Traj {
                    start: self.t,
                    traj,
                });
            }
            Action::StartRamp {
                robot,
                targets,
                duration,
            } => {
                let r = self.action_robot(robot)?;
                let model = self.world.robots()[r].model.clone();
                let vehicle = self.world.robots()[r]
                    .mount
                    .as_ref()
                    .map(|m| m.device.clone())
                    .unwrap_or_default();
                let rt = &mut self.robots[r];
                let walking = rt.walking();
                let owned: Vec<usize> = rt
                    .gait
                    .as_ref()
                    .and_then(|g| g.plan.as_ref().map(|p| p.owned(&g.gait)))
                    .unwrap_or_default();
                let mut goal = rt.q_nom.clone();
                for (joint, value) in targets {
                    let ji = model
                        .joint_index(joint)
                        .ok_or_else(|| err(format!("unknown joint `{joint}`")))?;
                    let qi = model.joints[ji]
                        .q_index
                        .ok_or_else(|| err(format!("joint `{joint}` is not actuated")))?;
                    // A leg mid-walk has one driver, the gait: a ramp on it
                    // would fight the footfalls. Standing, the legs are free.
                    if walking && owned.contains(&qi) {
                        return Err(err(format!(
                            "joint `{joint}` is driven by the gait while `{vehicle}` walks; \
                             ramp it after device_done"
                        )));
                    }
                    goal[qi] = *value;
                }
                // Two rest-to-rest waypoints: cubic Hermite eases in/out.
                // A tracked ramp cannot bake ahead — its poses are carried
                // by a part that has not moved yet — so it bakes per tick.
                // Nor can one alongside a walk: the legs bake tick by tick,
                // and the ramp's samples ride with them.
                if rt.tracking.is_none() && !walking {
                    rt.append_waypoint(self.t + duration, goal.clone(), vec![0.0; goal.len()]);
                }
                let end = self.t + duration;
                rt.planned.push(PlannedMove {
                    sequence: self.programs[self.current].sequence.name.clone(),
                    step: step_index,
                    motion: None,
                    segments: vec![crate::motion::PlannedSegment {
                        kind: crate::motion::SegmentKind::Joint,
                        waypoints: vec![rt.q_nom.clone(), goal.clone()],
                        tcp_speed: None,
                    }],
                    duration: *duration,
                    feed_report: None,
                    process_spans: Vec::new(),
                });
                self.programs[self.current].move_ends.push(end);
                rt.moves.push(StepSpan {
                    name: "ramp".to_string(),
                    start: self.t,
                    end,
                    sequence: self.programs[self.current].sequence.name.clone(),
                    step: step_index,
                });
                rt.active = Some(ActiveMove::Ramp {
                    start: self.t,
                    duration: *duration,
                    from: rt.q_nom.clone(),
                    to: goal,
                });
            }
            Action::Attach {
                robot,
                object,
                link,
                touch_links,
            } => {
                let r = self.action_robot(robot)?;
                self.world
                    .set_joint_positions_for(r, self.robots[r].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                // The pose the object rested at until this instant — a
                // freshly created track must tile [0, duration], so the
                // pre-grasp interval becomes a Hold at this pose.
                let rest_pose = self
                    .world
                    .obstacles()
                    .iter()
                    .find(|o| &o.name == object)
                    .map(|o| o.pose);
                self.world
                    .attach_obstacle_to(r, object, link.as_deref(), touch_links.as_deref())
                    .map_err(|e| err(e.to_string()))?;
                let attachment = self
                    .world
                    .attachment(object)
                    .expect("attach_obstacle just succeeded")
                    .clone();
                let t = self.t;
                let rest = rest_pose.expect("attach_obstacle validated the obstacle");
                let track = self.object_track_at(object, rest, t);
                if let Some(open) = track.spans.last_mut() {
                    let end = open.end_mut();
                    if *end < t {
                        *end = t;
                    }
                }
                track.spans.push(TrackSpan::Follow {
                    t0: t,
                    t1: t,
                    robot: attachment.robot,
                    link: attachment.link,
                    offset: attachment.grasp,
                });
                // Grasping the tracked part ends the chase: it moves with
                // the robot now, so the offset it had at the grasp stands
                // (which is what keeps the lift straight).
                if let Some(latch) = &mut self.robots[r].tracking {
                    if &latch.object == object {
                        latch.frozen = true;
                    }
                }
            }
            Action::Detach { object } => {
                // Sync the carrier so the object freezes where it truly is.
                let carrier = self
                    .world
                    .attachment(object)
                    .map(|a| a.robot)
                    .ok_or_else(|| err(format!("`{object}` is not attached")))?;
                self.world
                    .set_joint_positions_for(carrier, self.robots[carrier].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                self.world
                    .detach_obstacle(object)
                    .map_err(|e| err(e.to_string()))?;
                let pose = self
                    .world
                    .obstacles()
                    .iter()
                    .find(|o| &o.name == object)
                    .map(|o| o.pose)
                    .ok_or_else(|| err(format!("unknown obstacle `{object}`")))?;
                let t = self.t;
                let track = self.object_track_at(object, pose, t);
                if let Some(open) = track.spans.last_mut() {
                    let end = open.end_mut();
                    if *end < t {
                        *end = t;
                    }
                }
                track.spans.push(TrackSpan::Hold { t0: t, t1: t, pose });
            }
            Action::Track {
                robot,
                object,
                link,
            } => {
                let r = self.action_robot(robot)?;
                self.world
                    .set_joint_positions_for(r, self.robots[r].q.clone())
                    .map_err(|e| err(e.to_string()))?;
                self.latch_track(r, object, link.as_deref())?;
            }
            Action::Untrack { robot } => {
                let r = self.action_robot(robot)?;
                self.release_track(r);
            }
            Action::Set { signal, value } => {
                let t = self.t;
                let lane = self
                    .lane_index(signal)
                    .ok_or_else(|| err(format!("unknown signal `{signal}`")))?;
                self.set_lane(lane, t, *value);
            }
            Action::Device { device, command } => {
                let t = self.t;
                let mut lane_update = None;
                // A vehicle dispatched this scan: its drive, closed form,
                // for the legs of whatever rides it.
                let mut dispatched: Option<crate::gait::BodyProfile> = None;
                // A lift dispatched this scan: its cargo is fixed after
                // the borrow below ends.
                let mut lift_capture = false;
                // Computed before the borrow: is this device a vehicle
                // currently captured by a lift that is still moving?
                let riding_lift: Option<String> = self.devices.iter().find_map(|d| match d {
                    DeviceRuntime::Lift {
                        name,
                        position,
                        target,
                        cargo_vehicles,
                        ..
                    } if (position - target).abs() > 1e-9
                        && cargo_vehicles.iter().any(|v| v == device) =>
                    {
                        Some(name.clone())
                    }
                    _ => None,
                });
                let found = self.devices.iter_mut().find(|d| match d {
                    DeviceRuntime::Conveyor { name, .. }
                    | DeviceRuntime::Axis { name, .. }
                    | DeviceRuntime::Source { name, .. }
                    | DeviceRuntime::Sink { name, .. }
                    | DeviceRuntime::Lift { name, .. }
                    | DeviceRuntime::Vehicle { name, .. } => name == device,
                });
                let Some(found) = found else {
                    return Err(err(format!("unknown device `{device}`")));
                };
                match (found, command) {
                    (DeviceRuntime::Conveyor { running, lane, .. }, DeviceCommand::Start) => {
                        *running = true;
                        lane_update = Some((*lane, true));
                    }
                    (DeviceRuntime::Conveyor { running, lane, .. }, DeviceCommand::Stop) => {
                        *running = false;
                        lane_update = Some((*lane, false));
                    }
                    (
                        DeviceRuntime::Source {
                            running, due, lane, ..
                        },
                        DeviceCommand::Start,
                    ) => {
                        *running = true;
                        // Re-starting re-triggers: the feed is due now, so a
                        // Start always yields a carrier rather than landing
                        // mid-interval.
                        *due = 0.0;
                        lane_update = Some((*lane, true));
                    }
                    (DeviceRuntime::Source { running, lane, .. }, DeviceCommand::Stop) => {
                        *running = false;
                        lane_update = Some((*lane, false));
                    }
                    (DeviceRuntime::Source { interval, .. }, DeviceCommand::SetSpeed(seconds)) => {
                        // A source's "speed" is its feed period.
                        *interval = *seconds;
                    }
                    (DeviceRuntime::Conveyor { velocity, .. }, DeviceCommand::SetSpeed(speed)) => {
                        let norm = velocity.norm();
                        if norm > 1e-12 {
                            *velocity = *velocity / norm * *speed;
                        }
                    }
                    (
                        DeviceRuntime::Conveyor {
                            running,
                            remaining,
                            lane,
                            ..
                        },
                        DeviceCommand::Advance(distance),
                    ) => {
                        // Indexed transfer is a mode, not an overlay: a
                        // free-running belt has no defined "advance by",
                        // and a second advance while one is pending is a
                        // missing device_done in the program.
                        if *running {
                            return Err(err(format!(
                                "conveyor `{device}` is free-running; stop it before a \
                                 fixed advance"
                            )));
                        }
                        if remaining.is_some() {
                            return Err(err(format!(
                                "conveyor `{device}` is still advancing; wait for \
                                 device_done before the next advance"
                            )));
                        }
                        if *distance > 0.0 {
                            *remaining = Some(*distance);
                            lane_update = Some((*lane, true));
                        }
                    }
                    (
                        DeviceRuntime::Axis {
                            position,
                            target,
                            lane,
                            ..
                        },
                        DeviceCommand::MoveTo(goal),
                    ) => {
                        *target = *goal;
                        if (*target - *position).abs() > 1e-9 {
                            lane_update = Some((*lane, true));
                        }
                    }
                    (
                        DeviceRuntime::Vehicle {
                            waypoints,
                            stations,
                            ring,
                            speed,
                            turn_speed,
                            drive,
                            position,
                            heading,
                            at,
                            target,
                            legs,
                            lane,
                            ..
                        },
                        DeviceCommand::Goto { station },
                    ) => {
                        // A real dispatcher can amend an order in flight;
                        // deterministic v1 keeps travel uninterruptible.
                        if !legs.is_empty() {
                            return Err(err(format!(
                                "vehicle `{device}` is still travelling; wait for \
                                 device_done before the next goto"
                            )));
                        }
                        if let Some(lift) = &riding_lift {
                            return Err(err(format!(
                                "vehicle `{device}` is riding lift `{lift}`; wait for \
                                 the lift's device_done before the next goto"
                            )));
                        }
                        let to = stations
                            .iter()
                            .find(|(n, _)| n == station)
                            .map(|(_, i)| *i)
                            .ok_or_else(|| {
                                err(format!("vehicle `{device}` has no station `{station}`"))
                            })?;
                        let from = at.unwrap_or(*target);
                        if at.is_none() && (waypoints[*target] - *position).norm() > 1e-3 {
                            return Err(err(format!(
                                "vehicle `{device}` is off its path (a lift ride ended \
                                 between stations?) — no waypoint within 1 mm of \
                                 ({:.3}, {:.3}, {:.3})",
                                position.x, position.y, position.z
                            )));
                        }
                        let route = vehicle_route(waypoints, *ring, from, to);
                        // A lift edge is ridden, never driven — an aerial
                        // machine excepted: it flies its vertical legs.
                        if !matches!(drive, crate::seq::Drive::Aerial { .. }) {
                            let mut prev = from;
                            for &k in &route {
                                let d = waypoints[k] - waypoints[prev];
                                if d.z.abs() > 1e-3 && d.x.hypot(d.y) <= 1e-3 {
                                    return Err(err(format!(
                                        "station `{station}` is across the lift edge \
                                         (waypoints {prev}–{k}): goto the near side, \
                                         ride the lift, then continue"
                                    )));
                                }
                                prev = k;
                            }
                        }
                        *legs = build_legs(
                            waypoints,
                            &route,
                            *position,
                            *heading,
                            *speed,
                            *turn_speed,
                            drive,
                        );
                        *target = to;
                        if legs.is_empty() {
                            // Already there (or a zero-length route).
                            *at = Some(to);
                        } else {
                            *at = None;
                            lane_update = Some((*lane, true));
                            dispatched = Some(body_profile(legs, *position, *heading, t));
                        }
                    }
                    (
                        DeviceRuntime::Lift {
                            stops,
                            position,
                            target,
                            lane,
                            ..
                        },
                        DeviceCommand::MoveToStop(stop),
                    ) => {
                        // The car moves with its cargo fixed: no retarget
                        // in flight — the doors are closed.
                        if (*target - *position).abs() > 1e-9 {
                            return Err(err(format!(
                                "lift `{device}` is still moving; wait for \
                                 device_done before the next move_to"
                            )));
                        }
                        let goal = stops
                            .iter()
                            .find(|(n, _)| n == stop)
                            .map(|(_, v)| *v)
                            .ok_or_else(|| err(format!("lift `{device}` has no stop `{stop}`")))?;
                        *target = goal;
                        if (*target - *position).abs() > 1e-9 {
                            lane_update = Some((*lane, true));
                            lift_capture = true;
                        }
                    }
                    // Kind/command mismatches are rejected by validation.
                    _ => return Err(err(format!("invalid command for device `{device}`"))),
                }
                if let Some((lane, value)) = lane_update {
                    self.set_lane(lane, t, value);
                }
                if let Some(profile) = dispatched {
                    self.start_gaits(device, profile)?;
                }
                if lift_capture {
                    // The boarding is priced when the ride is commanded —
                    // an elevator moves after the doors close.
                    self.capture_lift(device).map_err(err)?;
                }
            }
        }
        Ok(())
    }

    /// The track for `object`, created lazily with a rest hold covering
    /// `[0, since]` so spans always tile from t = 0.
    /// The pose track of vehicle `name`, opened (held at `rest_pose` from
    /// t = 0) the first time the vehicle drives.
    fn vehicle_track_at(
        &mut self,
        name: &str,
        rest_pose: Isometry3<f64>,
        since: f64,
    ) -> &mut ObjectTrack {
        let index = match self.vehicles.iter().position(|v| v.name == name) {
            Some(i) => i,
            None => {
                let mut spans = Vec::new();
                if since > 0.0 {
                    spans.push(TrackSpan::Hold {
                        t0: 0.0,
                        t1: since,
                        pose: rest_pose,
                    });
                }
                self.vehicles.push(ObjectTrack {
                    name: name.to_string(),
                    spans,
                });
                self.vehicles.len() - 1
            }
        };
        &mut self.vehicles[index]
    }

    fn object_track_at(
        &mut self,
        object: &str,
        rest_pose: Isometry3<f64>,
        since: f64,
    ) -> &mut ObjectTrack {
        let index = match self.objects.iter().position(|o| o.name == object) {
            Some(i) => i,
            None => {
                let mut spans = Vec::new();
                if since > 0.0 {
                    spans.push(TrackSpan::Hold {
                        t0: 0.0,
                        t1: since,
                        pose: rest_pose,
                    });
                }
                self.objects.push(ObjectTrack {
                    name: object.to_string(),
                    spans,
                });
                self.objects.len() - 1
            }
        };
        &mut self.objects[index]
    }

    fn finish(mut self) -> SequenceTimeline {
        let duration = self.t;
        let names: Vec<String> = self.world.robots().iter().map(|r| r.name.clone()).collect();
        let robots = self
            .robots
            .into_iter()
            .zip(names)
            .map(|(mut rt, name)| {
                let (q, zeros) = (rt.q.clone(), vec![0.0; rt.q.len()]);
                rt.append_waypoint(duration, q, zeros);
                RobotTrack {
                    name,
                    trajectory: JointTrajectory {
                        times: rt.times,
                        positions: rt.positions,
                        velocities: rt.velocities,
                    },
                    moves: rt.moves,
                    planned: rt.planned,
                    footfalls: rt
                        .gait
                        .as_ref()
                        .map(|g| {
                            let mut steps = g.history.clone();
                            steps.sort_by(|a, b| {
                                a.land
                                    .partial_cmp(&b.land)
                                    .unwrap_or(std::cmp::Ordering::Equal)
                                    .then_with(|| a.leg.cmp(&b.leg))
                            });
                            steps
                        })
                        .unwrap_or_default(),
                    sway: rt
                        .gait
                        .as_ref()
                        .map(|g| g.sways.clone())
                        .unwrap_or_default(),
                    pitch: rt
                        .gait
                        .as_ref()
                        .map(|g| g.pitches.clone())
                        .unwrap_or_default(),
                    rise: rt.gait.map(|g| g.rises).unwrap_or_default(),
                    // The cycle usually ends parked: close a travelling span
                    // at its own end and rest there, rather than extending it
                    // to the horn and driving off the timeline.
                    base: rt.base.map(|mut spans| {
                        match spans.last() {
                            Some(span @ (TrackSpan::Linear { .. } | TrackSpan::Pivot { .. })) => {
                                let (_, end) = span.range();
                                if end < duration - 1e-12 {
                                    let pose = SequenceTimeline::span_pose(&spans, &[], end)
                                        .expect("non-empty base track");
                                    spans.push(TrackSpan::Hold {
                                        t0: end,
                                        t1: duration,
                                        pose,
                                    });
                                }
                            }
                            Some(_) => {
                                if let Some(open) = spans.last_mut() {
                                    *open.end_mut() = duration;
                                }
                            }
                            None => {}
                        }
                        spans
                    }),
                }
            })
            .collect();
        for track in self.objects.iter_mut().chain(self.vehicles.iter_mut()) {
            if let Some(open) = track.spans.last_mut() {
                *open.end_mut() = duration;
            }
        }
        SequenceTimeline {
            duration,
            sequences: self
                .programs
                .iter()
                .map(|p| p.sequence.name.clone())
                .collect(),
            scenario: None,
            robots,
            objects: self.objects,
            vehicles: self.vehicles,
            signals: self.signals,
            step_spans: self.step_spans,
            branches: self.branches,
        }
    }
}

/// The closed-form drive a vehicle is about to make: its route legs laid out
/// in time from `t0`, each with the frame it starts from. Built from the
/// same legs the tick walk consumes, at the same rates, so the body the
/// footfalls are planned against is the body that will be driven.
fn body_profile(
    legs: &std::collections::VecDeque<Leg>,
    mut position: nalgebra::Point3<f64>,
    mut heading: f64,
    t0: f64,
) -> crate::gait::BodyProfile {
    let mut pieces = Vec::with_capacity(legs.len());
    let mut t = t0;
    for leg in legs {
        let frame = vehicle_frame(&position, heading);
        match leg {
            Leg::Turn { to, omega } => {
                let need = wrap_angle(to - heading) / omega;
                if need > 1e-12 {
                    pieces.push((
                        t,
                        t + need,
                        frame,
                        VehiclePiece::Piv {
                            center: position,
                            omega: *omega,
                        },
                    ));
                    t += need;
                }
                heading = *to;
            }
            Leg::Straight { to, velocity } => {
                let need = (to - position).norm() / velocity.norm();
                if need > 1e-12 {
                    pieces.push((
                        t,
                        t + need,
                        frame,
                        VehiclePiece::Lin {
                            velocity: *velocity,
                        },
                    ));
                    t += need;
                }
                position = *to;
            }
        }
    }
    crate::gait::BodyProfile {
        t0,
        t_end: t,
        pieces,
        end_frame: vehicle_frame(&position, heading),
    }
}

impl ActiveMove {
    /// Pins the given DOF to `values` for the rest of the move — what a
    /// gait does to a move that outlives the walk: the legs it left at
    /// the stance must not be yanked back to where the move sampled them.
    fn pin_joints(&mut self, joints: &[usize], values: &[f64]) {
        match self {
            ActiveMove::Ramp { from, to, .. } => {
                for &qi in joints {
                    from[qi] = values[qi];
                    to[qi] = values[qi];
                }
            }
            ActiveMove::Traj { traj, .. } => {
                for i in 0..traj.positions.len() {
                    for &qi in joints {
                        traj.positions[i][qi] = values[qi];
                        traj.velocities[i][qi] = 0.0;
                    }
                }
            }
        }
    }
}

impl Rollout {
    /// A vehicle was dispatched: every robot that walks it plans its
    /// footfalls for the whole drive, right now, from the closed-form
    /// profile — the rest of the walk is sampling that plan.
    fn start_gaits(
        &mut self,
        device: &str,
        profile: crate::gait::BodyProfile,
    ) -> Result<(), SeqError> {
        for r in 0..self.robots.len() {
            let rides = self.world.robots()[r]
                .mount
                .as_ref()
                .is_some_and(|m| m.device == device);
            if !rides || self.robots[r].gait.is_none() {
                continue;
            }
            let mut gr = self.robots[r].gait.take().expect("checked above");
            let result = self.start_gait(r, &mut gr, profile.clone());
            self.robots[r].gait = Some(gr);
            result?;
        }
        Ok(())
    }

    fn start_gait(
        &mut self,
        r: usize,
        gr: &mut GaitRuntime,
        profile: crate::gait::BodyProfile,
    ) -> Result<(), SeqError> {
        let t = self.t;
        let err = |message: String| SeqError::Action {
            step: self.cur_step(),
            name: self.cur_step_name(),
            message,
        };
        // A ramp still moving a leg would be walked over mid-way.
        let ramp_driving = |qi: usize| -> bool {
            matches!(
                &self.robots[r].active,
                Some(ActiveMove::Ramp {
                    start,
                    duration,
                    from,
                    to,
                }) if start + duration > t + 1e-9 && from[qi] != to[qi]
            )
        };
        let model = self.world.robots()[r].model.clone();
        for leg in &gr.gait.legs {
            if let Some(&qi) = leg.joints.iter().find(|&&qi| ramp_driving(qi)) {
                return Err(err(format!(
                    "a ramp is still moving leg `{}` (joint `{}`); wait for done \
                     before the goto",
                    leg.name, model.joints[model.actuated_joints[qi]].name
                )));
            }
        }
        // The arms swing unless the hands are full or a ramp has them: a
        // carried part rides still, and a stow finishes on its own.
        let hands_full = self.world.attachments().iter().any(|a| a.robot == r);
        let swing: Vec<crate::gait::ArmSwing> = gr
            .gait
            .arm_swing
            .iter()
            .filter(|(qi, _)| !hands_full && !ramp_driving(*qi))
            .map(|&(qi, amplitude)| crate::gait::ArmSwing {
                joint: qi,
                center: self.robots[r].q[qi],
                amplitude,
            })
            .collect();
        self.world
            .set_joint_positions_for(r, self.robots[r].q.clone())
            .expect("commanded q has robot DOF");
        // Where the feet stand as the drive begins — and, for a goto issued
        // while the previous drive's legs were still settling, which swing
        // is in the air and finishes as planned.
        let n = gr.gait.legs.len();
        let (feet, carry): (Vec<_>, Vec<_>) = match &gr.plan {
            Some(plan) => (0..n)
                .map(|i| {
                    let (position, yaw, flying) = plan.anchor(i, t);
                    ((position, yaw), flying.cloned())
                })
                .unzip(),
            None => {
                let robot = &self.world.robots()[r];
                let feet = crate::gait::feet_at(
                    &robot.model,
                    &gr.gait,
                    robot.joint_positions(),
                    &(robot.base_pose() * gr.sway.inverse()),
                );
                (feet, vec![None; n])
            }
        };
        // The terrain under the walk — every walkable top face — is
        // snapshotted at dispatch like the rest of the plan.
        // How far from a foot's present foothold to look for the next one.
        // Not the machine's ability (`max_step` is the *check*): a surface
        // it cannot legally step onto must still be found, or the walk
        // quietly keeps to the floor instead of being refused by name.
        let reach = gr
            .gait
            .max_step
            .unwrap_or(0.0)
            .max(crate::gait::DEFAULT_STEP_REACH);
        let treads: Vec<(Isometry3<f64>, Vector3<f64>, String)> = self
            .world
            .obstacles()
            .iter()
            .filter(|o| o.walkable)
            .filter_map(|o| match &o.geometry {
                botrail_model::Geometry::Box { size } => Some((o.pose, size / 2.0, o.name.clone())),
                _ => None,
            })
            .collect();
        // The surface a foot stands on: the highest walkable face the whole
        // foot disc fits on — so at a nosing overlap the toe legitimately
        // lands on the lower tread's front, under the step above, the way
        // real stairs are climbed. Only when no face fits the disc does the
        // highest point-covering face answer (and the edge check names it).
        let need = gr.gait.foot_radius;
        let support = |x: f64, y: f64, hint: f64| -> Option<(f64, usize, f64)> {
            let mut fits: Option<(f64, usize, f64)> = None;
            let mut covers: Option<(f64, usize, f64)> = None;
            for (i, (pose, half, _)) in treads.iter().enumerate() {
                let local =
                    pose.inverse_transform_point(&nalgebra::Point3::new(x, y, pose.translation.z));
                let (mx, my) = (half.x - local.x.abs(), half.y - local.y.abs());
                if mx < 0.0 || my < 0.0 {
                    continue;
                }
                let top = pose.translation.z + half.z;
                if (top - hint).abs() > reach {
                    continue;
                }
                let margin = mx.min(my);
                if covers.map(|(t, _, _)| top > t).unwrap_or(true) {
                    covers = Some((top, i, margin));
                }
                if margin + 1e-9 >= need && fits.map(|(t, _, _)| top > t).unwrap_or(true) {
                    fits = Some((top, i, margin));
                }
            }
            fits.or(covers)
        };
        let floor = |x: f64, y: f64, hint: f64| -> Option<f64> {
            support(x, y, hint).map(|(top, _, _)| top)
        };
        let plan =
            crate::gait::plan_gait(&gr.gait, &gr.offset, profile, &feet, &carry, swing, &floor);
        // The declared step ability, and every foot staying on its tread —
        // both priced at dispatch, where the whole walk is known.
        let robot_name = self.world.robots()[r].name.clone();
        for (i, (leg, plan_leg)) in gr.gait.legs.iter().zip(&plan.legs).enumerate() {
            let mut prev = feet[i].0;
            for f in &plan_leg.footfalls {
                let rise = f.position.z - prev.z;
                if let Some(max_step) = gr.gait.max_step {
                    if rise.abs() > max_step + 1e-9 {
                        return Err(SeqError::StepHeight {
                            t,
                            robot: robot_name.clone(),
                            leg: leg.name.clone(),
                            rise,
                            max_step,
                            x: f.position.x,
                            y: f.position.y,
                            z: f.position.z,
                        });
                    }
                }
                let surface = f.position.z - gr.gait.foot_radius;
                if let Some((_, idx, margin)) = support(f.position.x, f.position.y, surface) {
                    if margin + 1e-9 < gr.gait.foot_radius {
                        return Err(SeqError::FootOverhang {
                            t,
                            robot: robot_name.clone(),
                            leg: leg.name.clone(),
                            obstacle: treads[idx].2.clone(),
                            margin,
                            need: gr.gait.foot_radius,
                        });
                    }
                }
                prev = f.position;
            }
        }
        for (i, leg) in plan.legs.iter().enumerate() {
            let carried = carry[i].as_ref();
            gr.history.extend(
                leg.footfalls
                    .iter()
                    .filter(|f| carried != Some(*f))
                    .cloned(),
            );
        }
        if !plan.pitch.is_empty() {
            // A walk dispatched mid-settle takes the tilt over from here.
            if let Some(open) = gr.pitches.last_mut() {
                if open.t1 > t {
                    open.t1 = t;
                }
            }
            gr.pitches.extend(plan.pitch.iter().cloned());
        }
        if !plan.rise.is_empty() {
            if let Some(open) = gr.rises.last_mut() {
                if open.t1 > t {
                    open.t1 = t;
                }
            }
            gr.rises.extend(plan.rise.iter().cloned());
        }
        if let Some(sway) = &plan.sway {
            // A walk dispatched mid-settle takes over the sway from here.
            if let Some(open) = gr.sways.last_mut() {
                if open.done > t {
                    open.done = t;
                }
            }
            gr.sways.push(sway.clone());
        }
        // The walk bakes tick by tick from here; a move's pre-baked future
        // would block it (and is re-baked when the legs let go).
        let rt = &mut self.robots[r];
        rt.truncate_after(t);
        let (q, zeros) = (rt.q.clone(), vec![0.0; rt.q.len()]);
        rt.append_waypoint(t, q, zeros);
        gr.plan = Some(plan);
        Ok(())
    }

    /// One scan tick of every walking robot's legs: foot targets off the
    /// plan, a warm-started solve per leg, the result merged over whatever
    /// else drives the robot, and the tick baked. When the last foot has
    /// settled the legs snap to the stance (the solve is within a few
    /// micrometres of it) and are handed back.
    fn advance_gaits(&mut self) -> Result<(), SeqError> {
        for r in 0..self.robots.len() {
            if !self.robots[r].walking() {
                continue;
            }
            let mut gr = self.robots[r].gait.take().expect("walking");
            let result = self.gait_tick(r, &mut gr);
            self.robots[r].gait = Some(gr);
            result?;
            // The legs have moved the body — take what it carries with it.
            self.place_carried(r);
        }
        Ok(())
    }

    /// One scan tick of every spinning mount — the propellers of a machine
    /// whose vehicle is off its starting ground, or moving at all (the
    /// same rule `vehicle_airborne` measures by). Presentation only: the
    /// rotor joints advance at their authored signed rates and the tick is
    /// baked, so studio and USD replay the spin; no verdict reads the
    /// phase (the checking shape is the swept solid, design-drone.md §3.4).
    fn advance_spins(&mut self) {
        let dt = self.options.dt;
        let t = self.t;
        for r in 0..self.robots.len() {
            let Some(spin) = self.robots[r].spin.clone() else {
                continue;
            };
            let flying = self.devices.iter().any(|device| match device {
                DeviceRuntime::Vehicle {
                    name,
                    position,
                    legs,
                    ..
                } => *name == spin.device && (!legs.is_empty() || position.z > spin.ground + 1e-6),
                _ => false,
            });
            if !flying {
                continue;
            }
            let rt = &mut self.robots[r];
            if rt.times.last().is_some_and(|last| *last > t + 1e-9) {
                // A move's pre-baked future owns the track — an append
                // would be dropped, so the phase holds with it.
                continue;
            }
            let mut q = rt.q.clone();
            for (joint, rate) in &spin.joints {
                q[*joint] += rate * dt;
            }
            rt.q = q.clone();
            let zeros = vec![0.0; q.len()];
            rt.append_waypoint(t, q.clone(), zeros);
            self.world
                .set_joint_positions_for(r, q)
                .expect("spin keeps the robot's DOF");
        }
    }

    fn gait_tick(&mut self, r: usize, gr: &mut GaitRuntime) -> Result<(), SeqError> {
        use crate::gait::{swing_pose, LegState};
        let t = self.t;
        let plan = gr.plan.as_ref().expect("walking");
        let finishing = t >= plan.done - 1e-9;
        let gait = &gr.gait;
        // The body's sway for this tick, composed onto the rigid ride the
        // vehicle left the base on (last tick's sway undone first).
        let was = nalgebra::Translation3::new(0.0, 0.0, gr.rise);
        let rigid =
            was.inverse() * *self.world.robots()[r].base_pose() * (gr.pitch * gr.sway).inverse();
        let sway = match (&plan.sway, finishing) {
            (Some(s), false) => s.offset_at(t),
            _ => Isometry3::identity(),
        };
        // The tilt holds through the settle — a machine parked on a
        // grade stands on it — so only the sway fades out.
        let pitch = crate::gait::pitch_offset(&plan.pitch, t);
        // The body rides the steps its feet are on, not the straight line
        // its route draws — that is what keeps the legs in their range.
        let rise = crate::gait::rise_at(&plan.rise, t);
        let lift = nalgebra::Translation3::new(0.0, 0.0, rise);
        self.world
            .set_robot_base_pose_for(r, lift * rigid * pitch * sway);
        gr.sway = sway;
        gr.pitch = pitch;
        gr.rise = rise;

        let rest = plan.rest(gait);
        let mut q = self.robots[r].q.clone();
        let seed = q.clone();
        if finishing {
            for leg in &gait.legs {
                for &qi in &leg.joints {
                    q[qi] = rest[qi];
                }
            }
            for s in &plan.swing {
                q[s.joint] = s.center;
            }
        } else {
            for (i, leg) in gait.legs.iter().enumerate() {
                let (target, stride) = match plan.state(gait, i, t) {
                    LegState::Planted(pose) => (pose, None),
                    LegState::Swinging { from, to, u } => (
                        // A step up (or down) clears the *higher* of the
                        // two treads by the authored lift. The chord
                        // already climbs the riser, so half of it more
                        // puts the apex one lift over the nosing — a
                        // whole riser more, as this first read, lifts the
                        // foot so high the leg has to fold past what it
                        // has, and a real one does not do that either.
                        swing_pose(
                            &from,
                            &to,
                            gait.lift
                                + 0.5 * (to.translation.vector.z - from.translation.vector.z).abs(),
                            u,
                        ),
                        Some((to.translation.vector - from.translation.vector).norm()),
                    ),
                };
                // A planted yaw-free sole gets its hip yaw from geometry
                // first; in the air the leg is left to the solve, which
                // would otherwise spin the plane as the foot passes the hip.
                let seed = match (&leg.yaw_seed, stride.is_none()) {
                    (Some(ys), true) => {
                        let parent = self.world.link_poses_for(r)[ys.parent];
                        match ys
                            .yaw_for(&parent, &nalgebra::Point3::from(target.translation.vector))
                        {
                            Some(yaw) => {
                                let mut seeded = seed.clone();
                                seeded[ys.joint] = yaw;
                                seeded
                            }
                            None => seed.clone(),
                        }
                    }
                    _ => seed.clone(),
                };
                let result = self
                    .world
                    .solve_ik_world_for(r, leg.foot, &target, &seed, &leg.ik)
                    .expect("seed has robot DOF");
                // The solve aims at a fraction of a micrometre and usually
                // gets there; a foot straight under its hip is singular for
                // a yaw-free leg, where it creeps instead. Out of reach is
                // what would show: a tenth of a millimetre, a milliradian.
                // A yaw-free sole in the air is let be: keeping it level
                // through a pivot would mean flipping the leg's plane as
                // the foot passes under the hip, and a sole only has to be
                // level when it stands.
                let airborne_axis = stride.is_some() && leg.ik.mode == botrail_kin::IkMode::Axis;
                if !result.converged
                    && !airborne_axis
                    && (result.pos_error > 1e-4 || result.rot_error > 1e-3)
                {
                    let base = self.world.robots()[r].base_pose();
                    let nominal = base * leg.nominal;
                    let stride = stride.unwrap_or_else(|| {
                        (target.translation.vector - nominal.translation.vector).norm()
                    });
                    // What the leg was actually asked for: the hip is the
                    // leg's first joint frame, the body plane its base.
                    let arm = &self.world.robots()[r].model;
                    let hip = self.world.link_poses_for(r)
                        [arm.joints[arm.actuated_joints[leg.joints[0]]].parent_link]
                        .translation
                        .vector;
                    let reach = (target.translation.vector - hip).norm();
                    let drop = base.translation.z - target.translation.vector.z;
                    return Err(SeqError::GaitReach {
                        t,
                        robot: self.world.robots()[r].name.clone(),
                        leg: leg.name.clone(),
                        stride,
                        reach,
                        drop,
                        detail: format!(
                            "{:.1e} m / {:.1e} rad short after {} iterations",
                            result.pos_error, result.rot_error, result.iters
                        ),
                    });
                }
                for &qi in &leg.joints {
                    q[qi] = result.q[qi];
                }
            }
            let phase = 2.0 * std::f64::consts::PI * (t - plan.profile.t0) / gait.period;
            for s in &plan.swing {
                q[s.joint] = s.center + s.amplitude * phase.sin();
            }
        }
        let dt = self.options.dt;
        let rt = &mut self.robots[r];
        let previous = rt.q.clone();
        rt.q = q;
        self.world
            .set_joint_positions_for(r, rt.q.clone())
            .expect("solved q has robot DOF");
        // The tick bakes with velocities by difference — except on the
        // settling tick, where the legs come to rest: a hold follows, and a
        // cubic through a resting sample with a one-tick velocity on it
        // would swing the legs around for the length of the hold.
        let owned = plan.owned(gait);
        let velocity: Vec<f64> =
            rt.q.iter()
                .zip(&previous)
                .enumerate()
                .map(|(qi, (now, before))| {
                    if finishing && owned.contains(&qi) {
                        0.0
                    } else {
                        (now - before) / dt
                    }
                })
                .collect();
        let q = rt.q.clone();
        rt.append_waypoint(t, q, velocity);
        if finishing {
            gr.plan = None;
            // A move that outlives the walk keeps the legs where the walk
            // left them, and its remaining samples go back on the bake.
            if let Some(active) = rt.active.as_mut() {
                active.pin_joints(&owned, &rest);
            }
            rt.rebake_active_tail(t);
        }
        Ok(())
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::motion::{Segment, SegmentKind};
    use crate::seq::Step;
    use botrail_model::Geometry;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::sync::Arc;

    /// 1-DOF arm: revolute Z at z = 0.5 (limits ±1), two 0.1 cubes.
    pub(crate) fn sample_scene() -> Scene {
        let urdf = r#"
        <robot name="r">
          <link name="a">
            <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
          </link>
          <link name="b">
            <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
          </link>
          <joint name="j" type="revolute">
            <parent link="a"/><child link="b"/>
            <origin xyz="0 0 0.5"/>
            <axis xyz="0 0 1"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ))
    }

    pub(crate) fn joint_motion(scene: &mut Scene, name: &str, goal: f64) {
        scene
            .add_segment(
                name,
                Segment {
                    kind: SegmentKind::Joint,
                    goal_positions: vec![goal],
                    constraints: vec![],
                },
            )
            .unwrap();
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    #[test]
    fn start_toolpath_approaches_then_holds_the_feed() {
        use crate::toolpath::{PathTarget, ToolMove, ToolMoveKind, Toolpath};
        const ARM6: &str = include_str!("../../../examples/assets/simple_arm.urdf");
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(ARM6).unwrap(),
        ));
        // Author the path around a flange-down working pose...
        let work_q = vec![0.0, 0.5, 0.9, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(work_q.clone()).unwrap();
        let tcp = scene.robot().default_tcp_link();
        let tip = scene.link_poses()[tcp].translation.vector;
        let target = |x: f64| PathTarget {
            position: nalgebra::Point3::new(x, tip.y, tip.z),
            tool_axis: nalgebra::Unit::new_normalize(Vector3::z()),
            spin: None,
        };
        scene.add_toolpath(Toolpath {
            name: "trim".into(),
            frame: None,
            moves: vec![
                ToolMove {
                    kind: ToolMoveKind::Rapid,
                    targets: vec![target(tip.x - 0.03)],
                    brush: None,
                },
                ToolMove {
                    // 6 cm at 20 mm/s: the cut alone must take ~3 s.
                    kind: ToolMoveKind::Feed(0.02),
                    targets: vec![target(tip.x + 0.03)],
                    brush: None,
                },
            ],
        });
        // ...then park elsewhere: the fire must plan an approach, not
        // teleport.
        let park = vec![0.4, 0.4, 1.0, std::f64::consts::PI - 1.4, 0.0, 0.0];
        scene.set_joint_positions(park.clone()).unwrap();
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![step(
                "cut",
                vec![Action::StartToolpath {
                    robot: None,
                    toolpath: "trim".into(),
                }],
                Condition::Done,
            )],
        });
        let tl = scene
            .simulate_sequence("cycle", &RolloutOptions::default())
            .unwrap();
        assert!(tl.duration > 3.0, "cut alone is 3 s, got {}", tl.duration);

        // The bake starts at the park pose (no teleport)...
        let q0 = tl.robots[0].trajectory.sample(0.0);
        for (a, b) in q0.iter().zip(&park) {
            assert!((a - b).abs() < 1e-9, "started away from park");
        }
        // ...and the tail of the cycle holds the commanded feed.
        let model = scene.robot().clone();
        let fk = |q: &[f64]| {
            botrail_kin::forward_kinematics(&model, q).unwrap()[tcp]
                .translation
                .vector
        };
        let t1 = tl.duration - 1.5;
        let p_a = fk(&tl.robots[0].trajectory.sample(t1));
        let p_b = fk(&tl.robots[0].trajectory.sample(t1 + 0.5));
        let speed = (p_b - p_a).norm() / 0.5;
        assert!(
            (speed - 0.02).abs() < 0.004,
            "mid-cut TCP speed {speed}, commanded 0.02"
        );

        // The planned record carries the approach and the per-sample feed
        // segments — what script export lowers.
        let planned = &tl.robots[0].planned[0];
        assert_eq!(planned.motion.as_deref(), Some("trim"));
        assert_eq!(planned.segments[0].kind, SegmentKind::Joint);
        assert!(planned
            .segments
            .iter()
            .any(|s| s.kind == SegmentKind::CartesianLine && s.tcp_speed == Some(0.02)));

        // Deterministic, like every bake.
        let again = scene
            .simulate_sequence("cycle", &RolloutOptions::default())
            .unwrap();
        assert_eq!(
            tl.robots[0].trajectory.times,
            again.robots[0].trajectory.times
        );
        assert_eq!(
            tl.robots[0].trajectory.positions,
            again.robots[0].trajectory.positions
        );
    }

    #[test]
    fn motions_waits_and_step_spans_line_up() {
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.8);
        joint_motion(&mut scene, "back", 0.0);
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step(
                    "run",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                step("wait", vec![], Condition::Elapsed { seconds: 0.5 }),
                step(
                    "return",
                    vec![Action::StartMotion {
                        motion: "back".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let options = RolloutOptions::default();
        let tl = scene.simulate_sequence("cycle", &options).unwrap();

        assert_eq!(tl.step_spans.len(), 3);
        let run = &tl.step_spans[0];
        let wait = &tl.step_spans[1];
        let ret = &tl.step_spans[2];
        // Step boundaries quantize up to the scan period.
        assert!(run.start == 0.0 && run.end > 0.2);
        assert!((tl.robots[0].trajectory.sample(run.end)[0] - 0.8).abs() < 1e-9);
        let wait_len = wait.end - wait.start;
        assert!(
            (0.5 - 1e-9..=0.5 + options.dt + 1e-9).contains(&wait_len),
            "wait_len = {wait_len}"
        );
        // The robot holds still through the wait.
        assert!((tl.robots[0].trajectory.sample(wait.start + 0.25)[0] - 0.8).abs() < 1e-9);
        // The return motion starts where the previous ended and comes home.
        assert!((tl.robots[0].trajectory.sample(tl.duration)[0]).abs() < 1e-9);
        assert!((ret.end - tl.duration).abs() < 1e-12);
        // Cycle time covers both motions plus the wait.
        assert!(tl.duration > run.end + 0.5);
    }

    fn set(signal: &str, value: bool) -> Action {
        Action::Set {
            signal: signal.into(),
            value,
        }
    }

    fn sel(name: &str, arms: Vec<(Condition, Vec<Step>)>) -> Step {
        Step {
            name: name.to_string(),
            actions: vec![],
            transition: Condition::Immediately,
            select: arms
                .into_iter()
                .map(|(condition, steps)| crate::seq::SelectArm { condition, steps })
                .collect(),
        }
    }

    #[test]
    fn edges_fire_on_transitions_not_startup_state() {
        let mut scene = sample_scene();
        scene.define_signal("flag", true);
        // Watchers are declared *before* the writer: the edge is raised
        // after their slot in the scan order, so per-program edge memory
        // is what catches it on their next pass.
        scene.upsert_sequence(Sequence {
            name: "watch_r".into(),
            steps: vec![step(
                "r",
                vec![],
                Condition::Rising {
                    name: "flag".into(),
                },
            )],
        });
        scene.upsert_sequence(Sequence {
            name: "watch_f".into(),
            steps: vec![step(
                "f",
                vec![],
                Condition::Falling {
                    name: "flag".into(),
                },
            )],
        });
        scene.upsert_sequence(Sequence {
            name: "pulse".into(),
            steps: vec![
                step("hold", vec![], Condition::Elapsed { seconds: 0.2 }),
                step("off", vec![set("flag", false)], Condition::Immediately),
                step("hold2", vec![], Condition::Elapsed { seconds: 0.2 }),
                step("on", vec![set("flag", true)], Condition::Immediately),
            ],
        });
        let tl = scene
            .simulate_sequences(&["watch_r", "watch_f", "pulse"], &RolloutOptions::default())
            .unwrap();
        let span = |name: &str| {
            tl.step_spans
                .iter()
                .find(|s| s.name == name)
                .unwrap_or_else(|| panic!("no span `{name}`"))
        };
        // `flag` starts true — the startup level is not an edge, so the
        // rising watcher waits for the *next* off→on (~0.4s), while the
        // falling watcher catches the drop (~0.2s). One scan of latency
        // is inherent (the writer runs after the watchers).
        let f = span("watch_f/f");
        let r = span("watch_r/r");
        assert!(
            (0.2..=0.23).contains(&f.end),
            "falling at {} (wanted ~0.21)",
            f.end
        );
        assert!(
            (0.4..=0.43).contains(&r.end),
            "rising at {} (wanted ~0.41)",
            r.end
        );
    }

    #[test]
    fn same_scan_set_edges_for_later_programs() {
        let mut scene = sample_scene();
        scene.define_signal("go", false);
        scene.upsert_sequence(Sequence {
            name: "boss".into(),
            steps: vec![step(
                "release",
                vec![set("go", true)],
                Condition::Immediately,
            )],
        });
        scene.upsert_sequence(Sequence {
            name: "station".into(),
            steps: vec![step(
                "await",
                vec![],
                Condition::Rising { name: "go".into() },
            )],
        });
        let tl = scene
            .simulate_sequences(&["boss", "station"], &RolloutOptions::default())
            .unwrap();
        // The set fires in the very first scan; the station (scanned
        // after the boss) sees the edge in that same scan.
        let await_span = tl
            .step_spans
            .iter()
            .find(|s| s.name == "station/await")
            .unwrap();
        assert_eq!(await_span.end, 0.0);
    }

    #[test]
    fn select_takes_the_first_true_arm_and_records_it() {
        let mut scene = sample_scene();
        for signal in ["ok", "ng", "did_a", "did_b", "joined"] {
            scene.define_signal(signal, false);
        }
        let arms = |scene: &mut Scene, name: &str| {
            scene.upsert_sequence(Sequence {
                name: name.into(),
                steps: vec![
                    sel(
                        "judge",
                        vec![
                            (
                                Condition::Signal {
                                    name: "ok".into(),
                                    value: true,
                                },
                                vec![step(
                                    "pass",
                                    vec![set("did_a", true)],
                                    Condition::Immediately,
                                )],
                            ),
                            (
                                Condition::Signal {
                                    name: "ng".into(),
                                    value: true,
                                },
                                vec![step(
                                    "reject",
                                    vec![set("did_b", true)],
                                    Condition::Immediately,
                                )],
                            ),
                        ],
                    ),
                    step("rejoin", vec![set("joined", true)], Condition::Immediately),
                ],
            });
        };

        // The world says NG: the second arm runs, the first never bakes.
        scene.define_signal("ng", true);
        arms(&mut scene, "s");
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let names: Vec<&str> = tl.step_spans.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["judge", "reject", "rejoin"]);
        assert_eq!(tl.branches.len(), 1);
        assert_eq!(
            (
                tl.branches[0].sequence.as_str(),
                tl.branches[0].step.as_str(),
                tl.branches[0].arm
            ),
            ("s", "judge", 1)
        );
        let latched = |name: &str| {
            tl.signals
                .iter()
                .find(|s| s.name == name)
                .unwrap()
                .edges
                .last()
                .unwrap()
                .1
        };
        assert!(latched("did_b") && !latched("did_a") && latched("joined"));

        // Both true: authored order is the priority — the first arm wins.
        scene.define_signal("ok", true);
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        assert_eq!(tl.branches[0].arm, 0);
        assert!(tl.step_spans.iter().any(|s| s.name == "pass"));
        assert!(!tl.step_spans.iter().any(|s| s.name == "reject"));
    }

    #[test]
    fn select_waits_for_an_arm_and_empty_arms_skip() {
        let mut scene = sample_scene();
        scene.define_signal("late", false);
        scene.define_signal("skip", false);
        scene.upsert_sequence(Sequence {
            name: "waiter".into(),
            steps: vec![
                sel(
                    "gate",
                    vec![
                        (
                            Condition::Signal {
                                name: "skip".into(),
                                value: true,
                            },
                            vec![step("never", vec![], Condition::Immediately)],
                        ),
                        (
                            Condition::Signal {
                                name: "late".into(),
                                value: true,
                            },
                            // An empty arm: straight to the rejoin.
                            vec![],
                        ),
                    ],
                ),
                step("after", vec![], Condition::Immediately),
            ],
        });
        scene.upsert_sequence(Sequence {
            name: "writer".into(),
            steps: vec![
                step("hold", vec![], Condition::Elapsed { seconds: 0.3 }),
                step("go", vec![set("late", true)], Condition::Immediately),
            ],
        });
        let tl = scene
            .simulate_sequences(&["waiter", "writer"], &RolloutOptions::default())
            .unwrap();
        // The branching step itself is the wait: its span shows the 0.3s.
        let gate = tl
            .step_spans
            .iter()
            .find(|s| s.name == "waiter/gate")
            .unwrap();
        assert!((0.3..=0.32).contains(&gate.end), "gate.end = {}", gate.end);
        assert!(!tl.step_spans.iter().any(|s| s.name == "waiter/never"));
        let after = tl
            .step_spans
            .iter()
            .find(|s| s.name == "waiter/after")
            .unwrap();
        assert_eq!(after.start, gate.end);
        assert_eq!(tl.branches[0].arm, 1);
    }

    #[test]
    fn spans_and_moves_carry_structural_attribution() {
        // Display names repeat freely ("wait", three times here) and gain a
        // "{sequence}/" prefix in multi-program bakes, so structural
        // consumers (the SFC chart) key on `sequence` + `step` instead.
        // Pin those fields to the flatten they index.
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.8);
        scene.define_signal("skip", false);
        scene.define_signal("late", false);
        let a_steps = vec![
            step("wait", vec![], Condition::Elapsed { seconds: 0.1 }),
            step(
                "wait",
                vec![Action::StartMotion {
                    motion: "go".into(),
                }],
                Condition::Done,
            ),
            sel(
                "gate",
                vec![
                    (
                        Condition::Signal {
                            name: "skip".into(),
                            value: true,
                        },
                        vec![step("never", vec![], Condition::Immediately)],
                    ),
                    // The taken arm is empty: no span — only `branches`
                    // records the decision.
                    (
                        Condition::Signal {
                            name: "late".into(),
                            value: true,
                        },
                        vec![],
                    ),
                ],
            ),
            step("wait", vec![], Condition::Immediately),
        ];
        let b_steps = vec![
            step("wait", vec![], Condition::Elapsed { seconds: 0.05 }),
            step("go", vec![set("late", true)], Condition::Immediately),
        ];
        scene.upsert_sequence(Sequence {
            name: "a".into(),
            steps: a_steps.clone(),
        });
        scene.upsert_sequence(Sequence {
            name: "b".into(),
            steps: b_steps.clone(),
        });
        let tl = scene
            .simulate_sequences(&["a", "b"], &RolloutOptions::default())
            .unwrap();

        // Every span's (sequence, step) indexes the owning flatten, and the
        // display name is that node's name behind the prefix.
        let flat_a = flatten(&a_steps);
        let flat_b = flatten(&b_steps);
        for span in &tl.step_spans {
            let flat = match span.sequence.as_str() {
                "a" => &flat_a,
                "b" => &flat_b,
                other => panic!("unexpected sequence `{other}`"),
            };
            assert_eq!(
                span.name,
                format!("{}/{}", span.sequence, flat[span.step].name),
                "span at {:.2}..{:.2}",
                span.start,
                span.end
            );
        }
        // The three duplicate "wait" spans resolve to distinct flat steps.
        let waits: Vec<usize> = tl
            .step_spans
            .iter()
            .filter(|s| s.sequence == "a" && s.name == "a/wait")
            .map(|s| s.step)
            .collect();
        assert_eq!(waits, [0, 1, 4]);
        assert!(!tl.step_spans.iter().any(|s| s.name == "a/never"));
        assert_eq!(
            (
                tl.branches[0].sequence.as_str(),
                tl.branches[0].select,
                tl.branches[0].arm
            ),
            ("a", 0, 1)
        );
        // Robot move spans carry the step that started them: the "go"
        // motion was fired by a's flat step 1.
        assert_eq!(tl.robots[0].moves.len(), 1);
        assert_eq!(
            (
                tl.robots[0].moves[0].sequence.as_str(),
                tl.robots[0].moves[0].step
            ),
            ("a", 1)
        );

        // Single-program bakes fill the same fields, names unprefixed.
        let tl = scene
            .simulate_sequence("b", &RolloutOptions::default())
            .unwrap();
        let attribution: Vec<(&str, &str, usize)> = tl
            .step_spans
            .iter()
            .map(|s| (s.name.as_str(), s.sequence.as_str(), s.step))
            .collect();
        assert_eq!(attribution, [("wait", "b", 0), ("go", "b", 1)]);
    }

    #[test]
    fn nested_selects_record_both_decisions() {
        let mut scene = sample_scene();
        for signal in ["outer", "inner"] {
            scene.define_signal(signal, true);
        }
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![sel(
                "level1",
                vec![(
                    Condition::Signal {
                        name: "outer".into(),
                        value: true,
                    },
                    vec![sel(
                        "level2",
                        vec![
                            (
                                Condition::Signal {
                                    name: "inner".into(),
                                    value: false,
                                },
                                vec![step("no", vec![], Condition::Immediately)],
                            ),
                            (
                                Condition::Signal {
                                    name: "inner".into(),
                                    value: true,
                                },
                                vec![step("yes", vec![], Condition::Immediately)],
                            ),
                        ],
                    )],
                )],
            )],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let names: Vec<&str> = tl.step_spans.iter().map(|s| s.name.as_str()).collect();
        assert_eq!(names, ["level1", "level2", "yes"]);
        let taken: Vec<(&str, usize)> = tl
            .branches
            .iter()
            .map(|b| (b.step.as_str(), b.arm))
            .collect();
        assert_eq!(taken, [("level1", 0), ("level2", 1)]);
        // Decisions carry the authored pre-order ordinal too.
        let ordinals: Vec<usize> = tl.branches.iter().map(|b| b.select).collect();
        assert_eq!(ordinals, [0, 1]);
    }

    #[test]
    fn scenario_crud_reserves_baseline() {
        let mut scene = sample_scene();
        let scenario = |name: &str| crate::seq::Scenario {
            name: name.into(),
            signals: vec![],
            obstacles: vec![],
            joints: vec![],
            faults: vec![],
        };
        assert!(scene.upsert_scenario(scenario("ng")).is_ok());
        assert!(scene.upsert_scenario(scenario("ng")).is_ok()); // replace
        assert_eq!(scene.scenarios().len(), 1);
        let err = scene.upsert_scenario(scenario("baseline")).unwrap_err();
        assert!(err.to_string().contains("reserved"), "{err}");
        scene.remove_scenario("ng").unwrap();
        assert!(scene.remove_scenario("ng").is_err());
    }

    #[test]
    fn apply_scenario_validates_before_touching_anything() {
        let mut scene = sample_scene();
        scene.define_signal("flag", false);
        scene
            .add_obstacle(
                "part",
                botrail_model::Geometry::Sphere { radius: 0.02 },
                Isometry3::translation(0.1, 0.0, 0.5),
            )
            .unwrap();
        scene
            .upsert_scenario(crate::seq::Scenario {
                name: "shifted".into(),
                signals: vec![("flag".into(), true)],
                obstacles: vec![("part".into(), Isometry3::translation(0.3, 0.0, 0.5))],
                joints: vec![("r".into(), vec![0.5])],
                faults: vec![],
            })
            .unwrap();

        let mut applied = scene.clone();
        applied.apply_scenario("shifted").unwrap();
        assert!(applied.signals()[0].initial);
        assert!((applied.obstacles()[0].pose.translation.x - 0.3).abs() < 1e-12);
        assert_eq!(applied.joint_positions(), &[0.5]);
        // The source scene is untouched (apply is for snapshots).
        assert!(!scene.signals()[0].initial);

        // `baseline` applies nothing; unknown scenarios error.
        scene.clone().apply_scenario("baseline").unwrap();
        assert!(matches!(
            scene.clone().apply_scenario("ghost"),
            Err(crate::SceneError::UnknownScenario(_))
        ));

        // Bad deltas are caught before anything is applied.
        let mut bad = scene.clone();
        bad.upsert_scenario(crate::seq::Scenario {
            name: "bad".into(),
            signals: vec![("flag".into(), true)],
            obstacles: vec![],
            joints: vec![("r".into(), vec![0.1, 0.2])], // wrong dof
            faults: vec![],
        })
        .unwrap();
        let err = bad.apply_scenario("bad").unwrap_err();
        assert!(matches!(err, crate::SceneError::WrongDof { .. }));
        assert!(
            !bad.signals()[0].initial,
            "failed apply must not half-apply"
        );

        // Unknown signal names name the fix; sensors are not overridable.
        let mut bad = scene.clone();
        bad.upsert_scenario(crate::seq::Scenario {
            name: "bad".into(),
            signals: vec![("ghost".into(), true)],
            obstacles: vec![],
            joints: vec![],
            faults: vec![],
        })
        .unwrap();
        let err = bad.apply_scenario("bad").unwrap_err();
        assert!(err.to_string().contains("define_signal"), "{err}");

        // Attached obstacles are refused (moving one re-grasps it).
        let mut held = scene.clone();
        held.attach_obstacle("part", None, None).unwrap();
        held.upsert_scenario(crate::seq::Scenario {
            name: "grab".into(),
            signals: vec![],
            obstacles: vec![("part".into(), Isometry3::translation(0.4, 0.0, 0.5))],
            joints: vec![],
            faults: vec![],
        })
        .unwrap();
        let err = held.apply_scenario("grab").unwrap_err();
        assert!(err.to_string().contains("attached"), "{err}");
    }

    /// The fourth delta: a fault pins an input lane for the run — the
    /// sensor's geometry is ignored, a program's `set` is dropped, an open
    /// wire reads with the binding's polarity, and the timeout names the
    /// force. A scenario without faults bakes bit-identically to baseline.
    #[test]
    fn faults_pin_inputs_and_name_themselves_in_timeouts() {
        use crate::seq::{Fault, FaultKind, Sensor, SensorKind, SensorWatch};
        let mut scene = sample_scene();
        // A part sits in the eye's zone: the eye is ON from t = 0.
        scene
            .add_obstacle(
                "part",
                botrail_model::Geometry::Sphere { radius: 0.02 },
                Isometry3::translation(0.3, 0.0, 0.5),
            )
            .unwrap();
        scene
            .upsert_sensor(Sensor {
                name: "eye".into(),
                kind: SensorKind::Zone {
                    pose: Isometry3::translation(0.3, 0.0, 0.5),
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                watch: SensorWatch::AllObjects,
                mount: None,
            })
            .unwrap();
        scene.define_signal("flag", false);
        joint_motion(&mut scene, "go", 0.5);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "await part",
                    vec![],
                    Condition::Signal {
                        name: "eye".into(),
                        value: true,
                    },
                ),
                step(
                    "raise",
                    vec![Action::Set {
                        signal: "flag".into(),
                        value: true,
                    }],
                    Condition::Signal {
                        name: "flag".into(),
                        value: true,
                    },
                ),
                step(
                    "go",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let fault = |name: &str, target: &str, kind: FaultKind| crate::seq::Scenario {
            name: name.into(),
            signals: vec![],
            obstacles: vec![],
            joints: vec![],
            faults: vec![Fault {
                target: target.into(),
                kind,
            }],
        };
        scene
            .upsert_scenario(fault("eye_stuck", "eye", FaultKind::StuckAt(false)))
            .unwrap();
        scene
            .upsert_scenario(fault("eye_open", "eye", FaultKind::Open))
            .unwrap();
        scene
            .upsert_scenario(fault("flag_stuck", "flag", FaultKind::StuckAt(false)))
            .unwrap();
        scene
            .upsert_scenario(fault("flag_open", "flag", FaultKind::Open))
            .unwrap();
        scene
            .upsert_scenario(crate::seq::Scenario {
                name: "clean".into(),
                signals: vec![],
                obstacles: vec![],
                joints: vec![],
                faults: vec![],
            })
            .unwrap();
        let options = RolloutOptions {
            max_duration: 5.0,
            ..RolloutOptions::default()
        };
        let base = scene.simulate_sequence("s", &options).unwrap();
        assert!(base.duration > 0.0);

        // A stuck-low eye ignores the part sitting in it: the program
        // never leaves `await part`, and the timeout says why.
        let err = scene
            .simulate_sequences_scenario(&["s"], Some("eye_stuck"), &options)
            .unwrap_err();
        assert!(
            matches!(&err, SeqError::Timeout { name, forced, .. } if name == "await part" && forced == &[("eye".to_string(), false)]),
            "{err:?}"
        );
        assert!(
            err.to_string()
                == "timed out after 5s waiting in step 0 (`await part`) — forced: eye=false",
            "{err}"
        );
        // Open, unbound: reads low too.
        let err = scene
            .simulate_sequences_scenario(&["s"], Some("eye_open"), &options)
            .unwrap_err();
        assert!(err.to_string().contains("forced: eye=false"), "{err}");

        // A stuck internal signal drops the program's own `set`: it waits
        // for its own flag forever, and the lane records no edge at all.
        let err = scene
            .simulate_sequences_scenario(&["s"], Some("flag_stuck"), &options)
            .unwrap_err();
        assert!(
            matches!(&err, SeqError::Timeout { name, .. } if name == "raise"),
            "{err:?}"
        );
        // A relay written and read by one program has no wire to open.
        let err = scene
            .simulate_sequences_scenario(&["s"], Some("flag_open"), &options)
            .unwrap_err();
        assert!(err.to_string().contains("no input wire to open"), "{err}");

        // Open follows the binding's polarity: wire the eye NC on a
        // declared controller and the open wire reads *true* — the part
        // "is there" for the program, which then runs through.
        let mut wired = scene.clone();
        wired
            .upsert_io_node(crate::iomap::IoNode {
                name: "UR".into(),
                kind: crate::iomap::IoNodeKind::RobotController {
                    robots: vec![wired.robots()[0].name.clone()],
                },
                programs: vec![],
                uplink: None,
                channels: vec![crate::iomap::IoChannel {
                    id: "DI0".into(),
                    kind: crate::iomap::ChannelKind::Di,
                    port: Some(0),
                    address: None,
                    electrical: None,
                }],
                place: None,
                model: None,
            })
            .unwrap();
        wired
            .bind_io(crate::iomap::IoBinding {
                point: crate::iomap::IoPointId::parse("eye", crate::iomap::IoDirection::Input),
                node: "UR".into(),
                channel: "DI0".into(),
                tag: None,
                field: None,
                invert: true,
                contact: Some(crate::iomap::Contact::Nc),
                safety: false,
                device: None,
                note: None,
                auto: false,
            })
            .unwrap();
        // Take the part away: geometry says OFF, the open NC wire says ON.
        wired.remove_obstacle("part").unwrap();
        assert!(matches!(
            wired.simulate_sequence("s", &options),
            Err(SeqError::Timeout { .. })
        ));
        let tl = wired
            .simulate_sequences_scenario(&["s"], Some("eye_open"), &options)
            .unwrap();
        let eye = tl.signals.iter().find(|s| s.name == "eye").unwrap();
        assert_eq!(eye.edges, vec![(0.0, true)], "a pinned level, not an edge");
        assert_eq!(tl.scenario.as_deref(), Some("eye_open"));

        // No faults: bit-identical to baseline, and the live scene never
        // carries a force.
        let clean = scene
            .simulate_sequences_scenario(&["s"], Some("clean"), &options)
            .unwrap();
        assert_eq!(clean.duration, base.duration);
        assert_eq!(
            clean.robots[0].trajectory.positions,
            base.robots[0].trajectory.positions
        );
        assert!(clean
            .signals
            .iter()
            .zip(&base.signals)
            .all(|(a, b)| a.name == b.name && a.edges == b.edges));
        assert!(scene.forced_inputs().is_empty());

        // Validation: a device lane, a robot, an unknown name, a duplicate.
        let bad = |target: &str, kind: FaultKind| {
            let mut s = scene.clone();
            s.upsert_scenario(fault("bad", target, kind)).unwrap();
            s.apply_scenario("bad").unwrap_err().to_string()
        };
        assert!(bad("ghost", FaultKind::Open).contains("not a sensor or an internal signal"));
        let mut twice = scene.clone();
        twice
            .upsert_scenario(crate::seq::Scenario {
                name: "twice".into(),
                signals: vec![],
                obstacles: vec![],
                joints: vec![],
                faults: vec![
                    Fault {
                        target: "eye".into(),
                        kind: FaultKind::StuckAt(true),
                    },
                    Fault {
                        target: "eye".into(),
                        kind: FaultKind::Open,
                    },
                ],
            })
            .unwrap();
        assert!(twice
            .apply_scenario("twice")
            .unwrap_err()
            .to_string()
            .contains("forced twice"));
    }

    #[test]
    fn scenarios_steer_branches_without_touching_the_live_scene() {
        let mut scene = sample_scene();
        scene.define_signal("ok", true);
        scene.define_signal("ng", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                sel(
                    "judge",
                    vec![
                        (
                            Condition::Signal {
                                name: "ok".into(),
                                value: true,
                            },
                            vec![step("pass", vec![], Condition::Immediately)],
                        ),
                        (
                            Condition::Signal {
                                name: "ng".into(),
                                value: true,
                            },
                            vec![step("reject", vec![], Condition::Immediately)],
                        ),
                    ],
                ),
                step("rejoin", vec![], Condition::Immediately),
            ],
        });
        scene
            .upsert_scenario(crate::seq::Scenario {
                name: "ng_part".into(),
                signals: vec![("ok".into(), false), ("ng".into(), true)],
                obstacles: vec![],
                joints: vec![],
                faults: vec![],
            })
            .unwrap();

        let options = RolloutOptions::default();
        let baseline = scene
            .simulate_sequences_scenario(&["s"], None, &options)
            .unwrap();
        assert_eq!(baseline.scenario, None);
        assert_eq!(baseline.branches[0].arm, 0);

        let named_baseline = scene
            .simulate_sequences_scenario(&["s"], Some("baseline"), &options)
            .unwrap();
        assert_eq!(named_baseline.scenario, None);
        assert_eq!(named_baseline.branches[0].arm, 0);

        let ng = scene
            .simulate_sequences_scenario(&["s"], Some("ng_part"), &options)
            .unwrap();
        assert_eq!(ng.scenario.as_deref(), Some("ng_part"));
        assert_eq!(ng.branches[0].arm, 1);
        assert!(ng.step_spans.iter().any(|s| s.name == "reject"));

        // The live scene still bakes the baseline path afterwards.
        assert!(scene.signals().iter().any(|s| s.name == "ok" && s.initial));
        let again = scene
            .simulate_sequences_scenario(&["s"], None, &options)
            .unwrap();
        assert_eq!(again.branches[0].arm, 0);

        let err = scene
            .simulate_sequences_scenario(&["s"], Some("ghost"), &options)
            .unwrap_err();
        assert!(err.to_string().contains("unknown scenario"), "{err}");
    }

    #[test]
    fn coverage_names_untaken_arms_until_scenarios_take_them() {
        let mut scene = sample_scene();
        for (name, initial) in [("ok", true), ("ng", false), ("fine", true)] {
            scene.define_signal(name, initial);
        }
        let signal = |name: &str, value: bool| Condition::Signal {
            name: name.into(),
            value,
        };
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                sel(
                    "judge",
                    vec![
                        (
                            signal("ok", true),
                            vec![sel(
                                "grade",
                                vec![
                                    (signal("fine", true), vec![]),
                                    (
                                        signal("fine", false),
                                        vec![step("rework", vec![], Condition::Immediately)],
                                    ),
                                ],
                            )],
                        ),
                        (
                            signal("ng", true),
                            vec![step("reject", vec![], Condition::Immediately)],
                        ),
                    ],
                ),
                step("end", vec![], Condition::Immediately),
            ],
        });

        let options = RolloutOptions::default();
        let base = scene
            .simulate_sequences_scenario(&["s"], None, &options)
            .unwrap();
        // The baseline takes judge/ok then grade/fine; the two other arms
        // are named in enumeration order, in the authoring vocabulary.
        let uncovered = arm_coverage(&scene, &[&base]).unwrap();
        let rows: Vec<(&str, usize, usize)> = uncovered
            .iter()
            .map(|u| (u.step.as_str(), u.select, u.arm))
            .collect();
        assert_eq!(rows, [("judge", 0, 1), ("grade", 1, 1)]);
        assert!(
            uncovered[0]
                .condition
                .contains("bt.seq.signal(\"ng\", True)"),
            "{}",
            uncovered[0].condition
        );
        assert!(
            uncovered[1].to_string().contains("`grade` arm 2"),
            "{}",
            uncovered[1]
        );

        // Each scenario buys coverage; the full set drains the report.
        scene
            .upsert_scenario(crate::seq::Scenario {
                name: "ng_part".into(),
                signals: vec![("ok".into(), false), ("ng".into(), true)],
                obstacles: vec![],
                joints: vec![],
                faults: vec![],
            })
            .unwrap();
        scene
            .upsert_scenario(crate::seq::Scenario {
                name: "coarse".into(),
                signals: vec![("fine".into(), false)],
                obstacles: vec![],
                joints: vec![],
                faults: vec![],
            })
            .unwrap();
        let ng = scene
            .simulate_sequences_scenario(&["s"], Some("ng_part"), &options)
            .unwrap();
        let after_ng = arm_coverage(&scene, &[&base, &ng]).unwrap();
        assert_eq!(after_ng.len(), 1);
        assert_eq!(after_ng[0].step, "grade");
        let coarse = scene
            .simulate_sequences_scenario(&["s"], Some("coarse"), &options)
            .unwrap();
        assert!(arm_coverage(&scene, &[&base, &ng, &coarse])
            .unwrap()
            .is_empty());
    }

    #[test]
    fn coverage_requires_one_shared_sequence_set() {
        let mut scene = sample_scene();
        scene.define_signal("go", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![sel(
                "gate",
                vec![
                    (
                        Condition::Signal {
                            name: "go".into(),
                            value: true,
                        },
                        vec![],
                    ),
                    (Condition::Immediately, vec![]),
                ],
            )],
        });
        scene.upsert_sequence(Sequence {
            name: "boss".into(),
            steps: vec![sel(
                "mode",
                vec![(
                    Condition::Immediately,
                    vec![step("fire", vec![set("go", true)], Condition::Immediately)],
                )],
            )],
        });
        let options = RolloutOptions::default();
        let pair = scene.simulate_sequences(&["s", "boss"], &options).unwrap();
        // Cross-program keys: boss's sole arm is covered, s's first is not
        // (s scans before boss, so `go` is still low at its gate).
        let uncovered = arm_coverage(&scene, &[&pair]).unwrap();
        let rows: Vec<(&str, &str, usize)> = uncovered
            .iter()
            .map(|u| (u.sequence.as_str(), u.step.as_str(), u.arm))
            .collect();
        assert_eq!(rows, [("s", "gate", 0)]);

        assert!(arm_coverage(&scene, &[])
            .unwrap_err()
            .contains("no timelines"));
        let solo = scene.simulate_sequences(&["boss"], &options).unwrap();
        let err = arm_coverage(&scene, &[&pair, &solo]).unwrap_err();
        assert!(err.contains("different sequence sets"), "{err}");
    }

    #[test]
    fn select_ordinals_match_the_shared_enumeration() {
        // Two selects nested inside the first arm plus one top-level
        // sibling: pre-order must number them 0 (outer), 1 (inner),
        // 2 (sibling) in both the enumeration and the flattening.
        let inner = sel("inner", vec![(Condition::Immediately, vec![])]);
        let steps = vec![
            sel(
                "outer",
                vec![
                    (Condition::Immediately, vec![inner]),
                    (Condition::Immediately, vec![]),
                ],
            ),
            step("between", vec![], Condition::Immediately),
            sel("sibling", vec![(Condition::Immediately, vec![])]),
        ];

        let enumerated: Vec<&str> = crate::seq::enumerate_selects(&steps)
            .iter()
            .map(|s| s.name.as_str())
            .collect();
        assert_eq!(enumerated, ["outer", "inner", "sibling"]);

        let flat = flatten(&steps);
        let flat_selects: Vec<(usize, &str)> = flat
            .iter()
            .filter_map(|s| s.select.map(|o| (o, s.name.as_str())))
            .collect();
        assert_eq!(flat_selects, [(0, "outer"), (1, "inner"), (2, "sibling")]);
    }

    #[test]
    fn select_authoring_rules_are_validated() {
        let mut scene = sample_scene();
        scene.define_signal("x", false);
        let arm = |steps: Vec<Step>| {
            (
                Condition::Signal {
                    name: "x".into(),
                    value: true,
                },
                steps,
            )
        };
        // Actions on a branching step are refused.
        let mut bad = sel("b", vec![arm(vec![])]);
        bad.actions = vec![set("x", true)];
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![bad],
        });
        let err = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap_err();
        assert!(err.to_string().contains("fires no actions"), "{err}");

        // A transition of its own is refused too.
        let mut bad = sel("b", vec![arm(vec![])]);
        bad.transition = Condition::Elapsed { seconds: 1.0 };
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![bad],
        });
        let err = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            err.to_string().contains("arms are its transitions"),
            "{err}"
        );

        // Arms must rejoin in the same grasp state.
        scene
            .add_obstacle(
                "part",
                botrail_model::Geometry::Sphere { radius: 0.02 },
                Isometry3::translation(0.1, 0.0, 0.5),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![sel(
                "b",
                vec![
                    arm(vec![step(
                        "grab",
                        vec![Action::Attach {
                            robot: None,
                            object: "part".into(),
                            link: None,
                            touch_links: None,
                        }],
                        Condition::Immediately,
                    )]),
                    arm(vec![]),
                ],
            )],
        });
        let err = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            err.to_string().contains("different grasp/tracking state"),
            "{err}"
        );

        // Edges on unknown signals are caught like level tests.
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "w",
                vec![],
                Condition::Rising {
                    name: "ghost".into(),
                },
            )],
        });
        let err = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap_err();
        assert!(err.to_string().contains("unknown signal"), "{err}");
    }

    #[test]
    fn ramp_signals_and_instant_chains() {
        let mut scene = sample_scene();
        scene.define_signal("flag", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "ramp",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("j".into(), 0.6)],
                        duration: 0.3,
                    }],
                    Condition::Done,
                ),
                step(
                    "mark",
                    vec![Action::Set {
                        signal: "flag".into(),
                        value: true,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "gate",
                    vec![],
                    Condition::Signal {
                        name: "flag".into(),
                        value: true,
                    },
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();

        // Cubic rest-to-rest ramp: exact midpoint halfway through.
        assert!((tl.robots[0].trajectory.sample(0.15)[0] - 0.3).abs() < 1e-9);
        assert!((tl.robots[0].trajectory.sample(0.3)[0] - 0.6).abs() < 1e-9);
        // mark + gate resolve in the same scan tick as the ramp's Done.
        let flag = &tl.signals[0];
        assert_eq!(flag.edges.first(), Some(&(0.0, false)));
        assert_eq!(flag.edges.len(), 2);
        let (edge_t, v) = flag.edges[1];
        assert!(v && (edge_t - tl.duration).abs() < 1e-12);
        assert!(!flag.value_at(0.1) && flag.value_at(tl.duration));
        // Instantaneous steps leave zero-width spans at the end.
        assert_eq!(tl.step_spans[1].start, tl.step_spans[1].end);
        assert!((tl.duration - 0.3).abs() < 0.011);
    }

    #[test]
    fn attach_detach_object_tracks() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                nalgebra::Isometry3::from_parts(
                    Translation3::new(0.1, 0.0, 0.5),
                    UnitQuaternion::identity(),
                ),
            )
            .unwrap();
        joint_motion(&mut scene, "go", 0.8);
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                step(
                    "drop",
                    vec![Action::Detach {
                        object: "box".into(),
                    }],
                    Condition::Immediately,
                ),
                step("settle", vec![], Condition::Elapsed { seconds: 0.2 }),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();

        assert_eq!(tl.objects.len(), 1);
        let track = &tl.objects[0];
        assert_eq!(track.name, "box");
        assert_eq!(track.spans.len(), 2);
        let TrackSpan::Follow {
            t0,
            t1,
            robot,
            link,
            offset,
        } = &track.spans[0]
        else {
            panic!("expected follow span, got {track:?}");
        };
        assert!(*t0 == 0.0 && *t1 > 0.2 && *robot == 0 && *link == 1);
        // Mid-motion the box rides FK ∘ grasp.
        let t_mid = (t0 + t1) / 2.0;
        let q = tl.robots[0].trajectory.sample(t_mid);
        let poses = scene.fk(&q).unwrap();
        let expected = poses[*link] * offset;
        let via_track =
            SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), t_mid).unwrap();
        assert!((via_track.translation.vector - expected.translation.vector).norm() < 1e-12);
        // After detach it holds at the rotated position for the rest.
        let TrackSpan::Hold {
            t1: hold_end, pose, ..
        } = &track.spans[1]
        else {
            panic!("expected hold span");
        };
        assert!((hold_end - tl.duration).abs() < 1e-12);
        let expected = Vector3::new(0.1 * 0.8f64.cos(), 0.1 * 0.8f64.sin(), 0.5);
        assert!((pose.translation.vector - expected).norm() < 1e-9);
        // The live scene is untouched by the rollout.
        assert!(scene.attachments().is_empty());
        assert!((scene.obstacles()[0].pose.translation.x - 0.1).abs() < 1e-12);
    }

    #[test]
    fn mid_sequence_attach_prepends_a_rest_hold() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                nalgebra::Isometry3::from_parts(
                    Translation3::new(0.1, 0.0, 0.5),
                    UnitQuaternion::identity(),
                ),
            )
            .unwrap();
        joint_motion(&mut scene, "go", 0.8);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step("wait", vec![], Condition::Elapsed { seconds: 0.4 }),
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        // Before the grasp the object rests at its scene pose (the track
        // tiles [0, duration] with a leading Hold).
        let track = &tl.objects[0];
        let TrackSpan::Hold { t0, t1, pose } = &track.spans[0] else {
            panic!("expected leading hold, got {track:?}");
        };
        assert!(*t0 == 0.0 && (*t1 - 0.4).abs() < 0.011);
        assert!((pose.translation.vector - Vector3::new(0.1, 0.0, 0.5)).norm() < 1e-12);
        let poses = scene.fk(&tl.robots[0].trajectory.sample(0.2)).unwrap();
        let early =
            SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), 0.2).unwrap();
        assert!((early.translation.vector - Vector3::new(0.1, 0.0, 0.5)).norm() < 1e-12);
        assert!(matches!(track.spans[1], TrackSpan::Follow { .. }));
    }

    #[test]
    fn validation_catches_authoring_mistakes() {
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.8);
        let check = |scene: &Scene, steps: Vec<Step>, needle: &str| {
            let mut s = scene.clone();
            s.upsert_sequence(Sequence {
                name: "bad".into(),
                steps,
            });
            let err = s
                .simulate_sequence("bad", &RolloutOptions::default())
                .unwrap_err();
            let msg = err.to_string();
            assert!(
                matches!(err, SeqError::Validation { .. }) && msg.contains(needle),
                "expected `{needle}` in `{msg}`"
            );
        };
        check(&scene, vec![], "no steps");
        check(
            &scene,
            vec![step(
                "x",
                vec![Action::StartMotion {
                    motion: "nope".into(),
                }],
                Condition::Done,
            )],
            "unknown motion",
        );
        check(
            &scene,
            vec![step("x", vec![], Condition::Done)],
            "starts none",
        );
        check(
            &scene,
            vec![step(
                "x",
                vec![],
                Condition::Signal {
                    name: "ghost".into(),
                    value: true,
                },
            )],
            "unknown signal",
        );
        check(
            &scene,
            vec![step(
                "x",
                vec![Action::StartRamp {
                    robot: None,
                    targets: vec![("j".into(), 5.0)],
                    duration: 0.2,
                }],
                Condition::Done,
            )],
            "outside",
        );
        check(
            &scene,
            vec![step(
                "x",
                vec![
                    Action::StartMotion {
                        motion: "go".into(),
                    },
                    Action::StartRamp {
                        robot: None,
                        targets: vec![("j".into(), 0.1)],
                        duration: 0.2,
                    },
                ],
                Condition::Done,
            )],
            "at most one",
        );
    }

    #[test]
    fn timeout_and_immediate_loop_guards() {
        let mut scene = sample_scene();
        scene.define_signal("never", false);
        scene.upsert_sequence(Sequence {
            name: "stuck".into(),
            steps: vec![step(
                "wait forever",
                vec![],
                Condition::Signal {
                    name: "never".into(),
                    value: true,
                },
            )],
        });
        let options = RolloutOptions {
            max_duration: 0.5,
            ..RolloutOptions::default()
        };
        assert!(matches!(
            scene.simulate_sequence("stuck", &options),
            Err(SeqError::Timeout { .. })
        ));

        let many: Vec<Step> = (0..10)
            .map(|i| step(&format!("s{i}"), vec![], Condition::Immediately))
            .collect();
        scene.upsert_sequence(Sequence {
            name: "chain".into(),
            steps: many,
        });
        let tight = RolloutOptions {
            immediate_chain_limit: 5,
            ..RolloutOptions::default()
        };
        assert!(matches!(
            scene.simulate_sequence("chain", &tight),
            Err(SeqError::ImmediateLoop { .. })
        ));
        // Under the limit the chain is fine (zero-duration sequence).
        let ok = scene
            .simulate_sequence("chain", &RolloutOptions::default())
            .unwrap();
        assert_eq!(ok.duration, 0.0);
        assert_eq!(ok.step_spans.len(), 10);
    }

    #[test]
    fn rollouts_are_deterministic() {
        let mut scene = sample_scene();
        scene.define_signal("flag", true);
        joint_motion(&mut scene, "go", 0.7);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "run",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::All(vec![
                        Condition::Done,
                        Condition::Signal {
                            name: "flag".into(),
                            value: true,
                        },
                    ]),
                ),
                step("wait", vec![], Condition::Elapsed { seconds: 0.25 }),
            ],
        });
        let a = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let b = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        assert_eq!(a.duration, b.duration);
        assert_eq!(a.robots[0].trajectory.times, b.robots[0].trajectory.times);
        assert_eq!(
            a.robots[0].trajectory.positions,
            b.robots[0].trajectory.positions
        );
        assert_eq!(a.step_spans.len(), b.step_spans.len());
    }

    #[test]
    fn signal_defs_shape_the_initial_state() {
        let mut scene = sample_scene();
        scene.define_signal("armed", true);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "gate",
                vec![],
                Condition::Signal {
                    name: "armed".into(),
                    value: true,
                },
            )],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        // Initially-true signal lets the gate pass at t = 0.
        assert_eq!(tl.duration, 0.0);
        assert_eq!(tl.signals[0].edges, vec![(0.0, true)]);
    }
}

#[cfg(test)]
mod device_tests {
    use super::tests::*;
    use super::*;
    use crate::seq::{Device, DeviceCommand, DeviceKind, Sensor, SensorKind, SensorWatch, Step};
    use botrail_model::Geometry;
    use nalgebra::{Point3, Translation3, Unit, UnitQuaternion, Vector3};

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// Conveyor feed with a photoelectric beam: the box trips the beam at
    /// the analytic time and the conveyor stops on the next scan.
    /// A magazine + a belt + a sink is an endless line built out of a
    /// finite pool: every carrier goes round more than once, and the object
    /// tracks stay continuous across the jumps.
    #[test]
    fn a_source_and_sink_recirculate_a_finite_pool() {
        let mut scene = sample_scene();
        let pool: Vec<String> = (0..3).map(|i| format!("c{i}")).collect();
        for (i, name) in pool.iter().enumerate() {
            scene
                .add_obstacle(
                    name,
                    Geometry::Box {
                        size: Vector3::new(0.04, 0.04, 0.04),
                    },
                    iso(-1.5, 0.5 - 0.1 * i as f64, 0.5),
                )
                .unwrap();
        }
        scene.upsert_device(Device {
            name: "feed".into(),
            kind: DeviceKind::Source {
                pool: pool.clone(),
                park: iso(-1.5, 0.5, 0.5),
                pitch: Vector3::new(0.0, -0.1, 0.0),
                pose: iso(-0.8, 0.5, 0.5),
                interval: 1.0,
                running: true,
            },
        });
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.5, 0.5),
                zone_size: Vector3::new(2.0, 0.3, 0.3),
                velocity: Vector3::new(0.4, 0.0, 0.0),
                running: true,
            },
        });
        scene.upsert_device(Device {
            name: "out".into(),
            kind: DeviceKind::Sink {
                zone_pose: iso(0.9, 0.5, 0.5),
                zone_size: Vector3::new(0.2, 0.3, 0.3),
                source: "feed".into(),
            },
        });
        scene.upsert_sequence(Sequence {
            name: "run".into(),
            steps: vec![Step {
                name: "run".into(),
                actions: vec![],
                transition: Condition::Elapsed { seconds: 12.0 },
                select: Vec::new(),
            }],
        });

        let tl = scene
            .simulate_sequence("run", &RolloutOptions::default())
            .unwrap();
        let track = |name: &str| {
            tl.objects
                .iter()
                .find(|o| o.name == name)
                .unwrap_or_else(|| panic!("no track for {name}"))
        };

        for name in &pool {
            let t = track(name);
            // Spans tile the whole run with no gap — a teleport must close
            // the span it interrupts, not leave a hole in the timeline.
            let mut cursor = 0.0;
            for span in &t.spans {
                let (t0, t1) = span.range();
                assert!(
                    (t0 - cursor).abs() < 1e-9,
                    "{name}: gap before {t0} (cursor {cursor})"
                );
                cursor = t1;
            }
            assert!(cursor >= 12.0 - 1e-6, "{name}: track ends at {cursor}");

            // Went round more than once: each lap is one jump upstream.
            let laps = (0..1200)
                .map(|i| {
                    SequenceTimeline::object_pose(t, &[], i as f64 * 0.01)
                        .unwrap()
                        .translation
                        .x
                })
                .collect::<Vec<_>>()
                .windows(2)
                .filter(|w| w[1] < w[0] - 0.5)
                .count();
            assert!(laps >= 2, "{name} only went round {laps} time(s)");
        }
    }

    /// A carrier the robot is holding must not be swept up by the sink it
    /// happens to be carried over.
    #[test]
    fn a_sink_leaves_grasped_carriers_alone() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "c0",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(0.9, 0.5, 0.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "feed".into(),
            kind: DeviceKind::Source {
                pool: vec!["c0".into()],
                park: iso(-1.5, 0.5, 0.5),
                pitch: Vector3::zeros(),
                pose: iso(-0.8, 0.5, 0.5),
                interval: 1.0,
                running: false,
            },
        });
        scene.upsert_device(Device {
            name: "out".into(),
            kind: DeviceKind::Sink {
                zone_pose: iso(0.9, 0.5, 0.5),
                zone_size: Vector3::new(0.4, 0.4, 0.4),
                source: "feed".into(),
            },
        });
        scene.attach_obstacle("c0", None, None).unwrap();
        scene.upsert_sequence(Sequence {
            name: "hold".into(),
            steps: vec![Step {
                name: "hold".into(),
                actions: vec![],
                transition: Condition::Elapsed { seconds: 1.0 },
                select: Vec::new(),
            }],
        });
        let tl = scene
            .simulate_sequence("hold", &RolloutOptions::default())
            .unwrap();
        let track = tl.objects.iter().find(|o| o.name == "c0").unwrap();
        assert!(
            track
                .spans
                .iter()
                .all(|s| matches!(s, TrackSpan::Follow { .. })),
            "the sink took a carrier out of the gripper: {:?}",
            track.spans
        );
    }

    #[test]
    fn conveyor_feed_trips_the_beam_on_time() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(-0.5, 0.5, 0.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "conv".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(-0.2, 0.5, 0.5),
                zone_size: Vector3::new(1.2, 0.3, 0.3),
                velocity: Vector3::new(0.25, 0.0, 0.0),
                running: false,
            },
        });
        scene
            .upsert_sensor(Sensor {
                name: "beam".into(),
                kind: SensorKind::Beam {
                    from: Point3::new(0.0, 0.3, 0.5),
                    to: Point3::new(0.0, 0.7, 0.5),
                    radius: 0.005,
                },
                watch: SensorWatch::AllObjects,
                mount: None,
            })
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "feed".into(),
            steps: vec![
                step(
                    "feed",
                    vec![Action::Device {
                        device: "conv".into(),
                        command: DeviceCommand::Start,
                    }],
                    Condition::Signal {
                        name: "beam".into(),
                        value: true,
                    },
                ),
                step(
                    "stop",
                    vec![Action::Device {
                        device: "conv".into(),
                        command: DeviceCommand::Stop,
                    }],
                    Condition::Elapsed { seconds: 0.1 },
                ),
            ],
        });
        let options = RolloutOptions::default();
        let tl = scene.simulate_sequence("feed", &options).unwrap();

        // Contact when the box face (half 0.02) meets the beam surface
        // (radius 0.005): origin at -0.025, i.e. 0.475 m / 0.25 m/s = 1.9 s.
        let feed_end = tl.step_spans[0].end;
        assert!(
            (feed_end - 1.9).abs() <= options.dt + 1e-9,
            "feed_end = {feed_end}"
        );
        // Lanes: the beam input edge and the conveyor output edges line up.
        let beam = tl.signals.iter().find(|s| s.name == "beam").unwrap();
        assert_eq!(beam.edges.len(), 2);
        assert!((beam.edges[1].0 - feed_end).abs() < 1e-9 && beam.edges[1].1);
        let conv = tl.signals.iter().find(|s| s.name == "conv").unwrap();
        assert!(conv.value_at(1.0));
        assert!(!conv.value_at(tl.duration));
        // The box rode a linear span then settled; total travel matches.
        let track = &tl.objects[0];
        assert!(matches!(track.spans[0], TrackSpan::Linear { .. }));
        assert!(matches!(track.spans.last(), Some(TrackSpan::Hold { .. })));
        let poses = scene
            .fk(&tl.robots[0].trajectory.sample(tl.duration))
            .unwrap();
        let end_pose =
            SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), tl.duration)
                .unwrap();
        let travelled = end_pose.translation.x - (-0.5);
        assert!(
            (travelled - 0.25 * feed_end).abs() < 1e-9,
            "travelled = {travelled}"
        );
        // The live scene's box stays where it was authored.
        assert!((scene.obstacles()[0].pose.translation.x - (-0.5)).abs() < 1e-12);
    }

    /// A linear axis positions its payload exactly and reports in-position.
    #[test]
    fn linear_axis_moves_exactly_and_reports_done() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "door",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.6, 0.0, 0.2),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "lift".into(),
            kind: DeviceKind::LinearAxis {
                objects: vec!["door".into()],
                axis: Unit::new_normalize(Vector3::z()),
                speed: 0.5,
                position: 0.0,
                range: (0.0, 0.4),
            },
        });
        scene.upsert_sequence(Sequence {
            name: "open".into(),
            steps: vec![
                step(
                    "raise",
                    vec![Action::Device {
                        device: "lift".into(),
                        command: DeviceCommand::MoveTo(0.3),
                    }],
                    Condition::DeviceDone {
                        device: "lift".into(),
                    },
                ),
                step("hold", vec![], Condition::Elapsed { seconds: 0.1 }),
            ],
        });
        let options = RolloutOptions::default();
        let tl = scene.simulate_sequence("open", &options).unwrap();

        // 0.3 m at 0.5 m/s = 0.6 s (+ scan quantization).
        let raise_end = tl.step_spans[0].end;
        assert!((raise_end - 0.6).abs() <= options.dt + 1e-9, "{raise_end}");
        // The door lands exactly 0.3 above its rest height.
        let track = &tl.objects[0];
        let poses = scene
            .fk(&tl.robots[0].trajectory.sample(tl.duration))
            .unwrap();
        let end = SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), tl.duration)
            .unwrap();
        assert!(
            (end.translation.z - 0.5).abs() < 1e-12,
            "{}",
            end.translation.z
        );
        // Mid-travel sampling is linear (exact at 0.3 s: half way).
        let mid = SequenceTimeline::object_pose(track, std::slice::from_ref(&poses), 0.3).unwrap();
        assert!(
            (mid.translation.z - 0.35).abs() < 1e-9,
            "{}",
            mid.translation.z
        );
        // The moving lane pulses during travel only.
        let lane = tl.signals.iter().find(|s| s.name == "lift").unwrap();
        assert!(lane.value_at(0.3) && !lane.value_at(tl.duration));
    }

    /// A zone sensor watching the robot (light curtain) fires when a ramp
    /// swings the arm into it.
    #[test]
    fn robot_watch_zone_fires_on_intrusion() {
        let mut scene = sample_scene();
        // Link b (cube at z = 0.5) swings to +y at q = pi/2; park the zone
        // there.
        scene
            .upsert_sensor(Sensor {
                name: "curtain".into(),
                kind: SensorKind::Zone {
                    pose: iso(0.0, 0.0, 0.5),
                    size: Vector3::new(0.4, 0.4, 0.4),
                },
                watch: SensorWatch::Robot,
                mount: None,
            })
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "wait for the arm",
                vec![],
                Condition::Signal {
                    name: "curtain".into(),
                    value: true,
                },
            )],
        });
        // The arm is already inside the curtain at rest: fires at t = 0.
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        assert_eq!(tl.duration, 0.0);
        let lane = tl.signals.iter().find(|s| s.name == "curtain").unwrap();
        assert!(lane.value_at(0.0));
    }

    #[test]
    fn device_and_sensor_validation() {
        let mut scene = sample_scene();
        scene.upsert_device(Device {
            name: "conv".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.0, 0.0),
                zone_size: Vector3::new(1.0, 1.0, 1.0),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: true,
            },
        });
        // A source really has no in-position (a conveyor now does: a
        // consumed `advance`).
        scene.upsert_device(Device {
            name: "feed".into(),
            kind: DeviceKind::Source {
                pool: vec![],
                park: iso(0.0, 0.0, 0.0),
                pitch: Vector3::zeros(),
                pose: iso(0.0, 0.0, 0.0),
                interval: 1.0,
                running: false,
            },
        });
        scene
            .upsert_sensor(Sensor {
                name: "eye".into(),
                kind: SensorKind::Zone {
                    pose: iso(0.0, 0.0, 0.0),
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                watch: SensorWatch::AllObjects,
                mount: None,
            })
            .unwrap();
        let check = |scene: &Scene, steps: Vec<Step>, needle: &str| {
            let mut s = scene.clone();
            s.upsert_sequence(Sequence {
                name: "bad".into(),
                steps,
            });
            let err = s
                .simulate_sequence("bad", &RolloutOptions::default())
                .unwrap_err()
                .to_string();
            assert!(err.contains(needle), "expected `{needle}` in `{err}`");
        };
        // Sensors are read-only inputs.
        check(
            &scene,
            vec![step(
                "x",
                vec![Action::Set {
                    signal: "eye".into(),
                    value: true,
                }],
                Condition::Immediately,
            )],
            "read-only",
        );
        // device_done needs a positioning device (axis or vehicle).
        check(
            &scene,
            vec![step(
                "x",
                vec![],
                Condition::DeviceDone {
                    device: "feed".into(),
                },
            )],
            "no in-position",
        );
        // Axis commands must stay in range; conveyors have no position.
        check(
            &scene,
            vec![step(
                "x",
                vec![Action::Device {
                    device: "conv".into(),
                    command: DeviceCommand::MoveTo(0.5),
                }],
                Condition::Immediately,
            )],
            "start/stop",
        );
        // Sensor names may not shadow internal signals.
        let mut shadowed = scene.clone();
        shadowed.define_signal("eye", false);
        check(
            &shadowed,
            vec![step("x", vec![], Condition::Immediately)],
            "collides",
        );
    }
}

#[cfg(test)]
mod multi_actor_tests {
    use super::*;
    use crate::motion::{Segment, SegmentKind};
    use crate::seq::{Sensor, SensorKind, SensorWatch, Step};
    use botrail_model::Geometry;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use std::sync::Arc;

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// §1 in miniature: two 1-DOF sliders face each other across a shared
    /// middle band. Fully extended rods overlap; one at a time clears.
    const SLIDER: &str = r#"
        <robot name="slider">
          <link name="base"/>
          <link name="rod">
            <visual>
              <origin xyz="0 0.25 0"/>
              <geometry><box size="0.08 0.5 0.08"/></geometry>
            </visual>
          </link>
          <joint name="s" type="prismatic">
            <parent link="base"/><child link="rod"/>
            <origin xyz="0 0 0.3"/>
            <axis xyz="0 1 0"/>
            <limit lower="0" upper="0.6" effort="1" velocity="1"/>
          </joint>
        </robot>"#;

    /// Robots `a` (pushing +y from y = -0.75) and `b` (mirrored at +0.75),
    /// with the interlock zone over the shared band y ∈ [-0.2, 0.2] watching
    /// only `a`. Motions: `<r>_in` extends into the band, `<r>_out` retreats.
    fn dual_cell() -> Scene {
        let model = Arc::new(botrail_model::RobotModel::from_urdf_str(SLIDER).unwrap());
        let mut scene = crate::Scene::with_base(model.clone(), iso(0.0, -0.75, 0.0));
        scene.rename_robot(0, "a");
        let flipped = Isometry3::from_parts(
            Translation3::new(0.0, 0.75, 0.0),
            UnitQuaternion::from_axis_angle(&nalgebra::Vector3::z_axis(), std::f64::consts::PI),
        );
        scene.add_robot(model, Some("b"), flipped);
        for (robot, name, goal) in [
            (0, "a_in", 0.45),
            (0, "a_out", 0.0),
            (1, "b_in", 0.45),
            (1, "b_out", 0.0),
        ] {
            scene
                .add_segment_for(
                    robot,
                    name,
                    Segment {
                        kind: SegmentKind::Joint,
                        goal_positions: vec![goal],
                        constraints: vec![],
                    },
                )
                .unwrap();
        }
        scene
            .upsert_sensor(Sensor {
                name: "zone".into(),
                kind: SensorKind::Zone {
                    pose: iso(0.0, 0.0, 0.3),
                    size: Vector3::new(0.4, 0.4, 0.4),
                },
                watch: SensorWatch::Robots(vec!["a".into()]),
                mount: None,
            })
            .unwrap();
        scene
    }

    /// The §1 scenario with the interlock in place: A works the shared band
    /// first, B waits for the zone to clear, both retreats run concurrently,
    /// and the whole cycle bakes deterministically.
    #[test]
    fn zone_interlock_serializes_the_shared_band() {
        let mut scene = dual_cell();
        scene.upsert_sequence(Sequence {
            name: "cell".into(),
            steps: vec![
                step(
                    "A enter",
                    vec![Action::StartMotion {
                        motion: "a_in".into(),
                    }],
                    Condition::Done,
                ),
                // Async: A retreats while B already waits on the interlock.
                step(
                    "A retreat",
                    vec![Action::StartMotion {
                        motion: "a_out".into(),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "B interlock",
                    vec![],
                    Condition::Signal {
                        name: "zone".into(),
                        value: false,
                    },
                ),
                step(
                    "B enter",
                    vec![Action::StartMotion {
                        motion: "b_in".into(),
                    }],
                    Condition::Done,
                ),
                step(
                    "B retreat",
                    vec![Action::StartMotion {
                        motion: "b_out".into(),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "cycle end",
                    vec![],
                    Condition::All(vec![
                        Condition::RobotDone { robot: "a".into() },
                        Condition::RobotDone { robot: "b".into() },
                    ]),
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("cell", &RolloutOptions::default())
            .unwrap();

        assert_eq!(tl.robots.len(), 2);
        assert_eq!(tl.robots[0].name, "a");
        assert_eq!(tl.robots[1].name, "b");
        // Both arms actually ran their strokes.
        let a_moves = &tl.robots[0].moves;
        let b_moves = &tl.robots[1].moves;
        assert_eq!(
            a_moves.iter().map(|m| m.name.as_str()).collect::<Vec<_>>(),
            ["a_in", "a_out"]
        );
        assert_eq!(
            b_moves.iter().map(|m| m.name.as_str()).collect::<Vec<_>>(),
            ["b_in", "b_out"]
        );
        // Concurrency: B enters while A is still retreating (the interlock
        // released on zone exit, not on A's completion)...
        assert!(
            b_moves[0].start < a_moves[1].end - 1e-9,
            "B started at {} but A finished retreating at {}",
            b_moves[0].start,
            a_moves[1].end
        );
        // ...but never before A cleared the shared band (the zone lane's
        // falling edge).
        let zone = tl.signals.iter().find(|s| s.name == "zone").unwrap();
        let cleared = zone
            .edges
            .iter()
            .rev()
            .find(|(_, v)| !*v)
            .map(|(t, _)| *t)
            .unwrap();
        assert!(b_moves[0].start >= cleared - 1e-9);
        // The cycle ends when both robots are idle (RobotDone × 2).
        let last_end = a_moves[1].end.max(b_moves[1].end);
        assert!((tl.duration - last_end).abs() <= 0.011, "{}", tl.duration);

        // Deterministic: an identical run bakes bit-identical tracks.
        let again = scene
            .simulate_sequence("cell", &RolloutOptions::default())
            .unwrap();
        assert_eq!(tl.duration, again.duration);
        for (x, y) in tl.robots.iter().zip(&again.robots) {
            assert_eq!(x.trajectory.times, y.trajectory.times);
            assert_eq!(x.trajectory.positions, y.trajectory.positions);
        }
    }

    /// Without the interlock both rods sweep into the band mid-flight —
    /// each plan was valid against the other's *frozen* pose — and the tick
    /// verification reports the collision with its time.
    #[test]
    fn dropping_the_interlock_reports_a_robot_collision() {
        let mut scene = dual_cell();
        scene.upsert_sequence(Sequence {
            name: "clash".into(),
            steps: vec![step(
                "both enter",
                vec![
                    Action::StartMotion {
                        motion: "a_in".into(),
                    },
                    Action::StartMotion {
                        motion: "b_in".into(),
                    },
                ],
                Condition::Done,
            )],
        });
        let err = scene
            .simulate_sequence("clash", &RolloutOptions::default())
            .unwrap_err();
        let SeqError::RobotCollision {
            t, a, b, link_a, ..
        } = &err
        else {
            panic!("expected RobotCollision, got {err}");
        };
        assert_eq!((a.as_str(), b.as_str()), ("a", "b"));
        assert!(*t > 0.1 && *t < 1.0, "t = {t}");
        assert!(link_a.contains("rod"));
        let msg = err.to_string();
        assert!(msg.contains("interlock"), "{msg}");
    }

    /// A handover writes as detach → attach: the object's track follows
    /// robot `a`, holds, then follows robot `b`.
    #[test]
    fn handover_switches_the_carrier() {
        let mut scene = dual_cell();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(0.0, 0.0, 0.3),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "pass".into(),
            steps: vec![
                step(
                    "A grasp",
                    vec![Action::Attach {
                        robot: Some("a".into()),
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
                step(
                    "A place",
                    vec![Action::Detach {
                        object: "box".into(),
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
                step(
                    "B grasp",
                    vec![Action::Attach {
                        robot: Some("b".into()),
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Elapsed { seconds: 0.2 },
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("pass", &RolloutOptions::default())
            .unwrap();
        let track = &tl.objects[0];
        let carriers: Vec<Option<usize>> = track
            .spans
            .iter()
            .map(|s| match s {
                TrackSpan::Follow { robot, .. } => Some(*robot),
                _ => None,
            })
            .collect();
        assert_eq!(carriers, [Some(0), None, Some(1)], "{track:?}");
        // The live scene is untouched.
        assert!(scene.attachments().is_empty());
    }

    /// Multi-robot authoring rules: ambiguous `robot=None`, double attach,
    /// two drivers on one arm, and unknown robots in `robot_done`.
    #[test]
    fn multi_actor_validation_rules() {
        let scene = dual_cell();
        let check = |steps: Vec<Step>, needle: &str| {
            let mut s = scene.clone();
            s.add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                iso(0.0, 0.0, 0.3),
            )
            .unwrap();
            s.upsert_sequence(Sequence {
                name: "bad".into(),
                steps,
            });
            let err = s
                .simulate_sequence("bad", &RolloutOptions::default())
                .unwrap_err();
            let msg = err.to_string();
            assert!(
                matches!(err, SeqError::Validation { .. }) && msg.contains(needle),
                "expected `{needle}` in `{msg}`"
            );
        };
        // With two robots, an unaddressed ramp is ambiguous.
        check(
            vec![step(
                "x",
                vec![Action::StartRamp {
                    robot: None,
                    targets: vec![("s".into(), 0.1)],
                    duration: 0.2,
                }],
                Condition::Done,
            )],
            "give the action a robot",
        );
        // One driver per robot per step; a second on the same arm is out.
        check(
            vec![step(
                "x",
                vec![
                    Action::StartMotion {
                        motion: "a_in".into(),
                    },
                    Action::StartRamp {
                        robot: Some("a".into()),
                        targets: vec![("s".into(), 0.1)],
                        duration: 0.2,
                    },
                ],
                Condition::Done,
            )],
            "per robot",
        );
        // One carrier at a time — a handover needs the detach in between.
        check(
            vec![
                step(
                    "a grabs",
                    vec![Action::Attach {
                        robot: Some("a".into()),
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "b grabs too",
                    vec![Action::Attach {
                        robot: Some("b".into()),
                        object: "box".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
            ],
            "already attached",
        );
        // robot_done must name a robot that exists.
        check(
            vec![step(
                "x",
                vec![],
                Condition::RobotDone {
                    robot: "ghost".into(),
                },
            )],
            "unknown robot",
        );
    }
}

#[cfg(test)]
mod tracking_tests {
    use super::*;
    use crate::seq::{Action, Condition, Device, DeviceKind, Sequence, Step};
    use botrail_model::Geometry;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// A 3-axis gantry: enough DOF to hold a pose while a part slides by.
    fn gantry() -> Scene {
        let urdf = r#"
        <robot name="gantry">
          <link name="base"/>
          <link name="bridge"/>
          <link name="carriage"/>
          <link name="tool">
            <visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual>
          </link>
          <joint name="jx" type="prismatic">
            <parent link="base"/><child link="bridge"/>
            <axis xyz="1 0 0"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
          <joint name="jy" type="prismatic">
            <parent link="bridge"/><child link="carriage"/>
            <axis xyz="0 1 0"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
          <joint name="jz" type="prismatic">
            <parent link="carriage"/><child link="tool"/>
            <axis xyz="0 0 1"/>
            <limit lower="-2" upper="2" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        Scene::new(std::sync::Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ))
    }

    /// Belt at 0.2 m/s with the part starting under the taught pose.
    fn cell() -> Scene {
        let mut scene = gantry();
        scene.set_joint_positions(vec![0.0, 0.0, 0.4]).unwrap();
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                iso(0.0, 0.0, 0.0),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.5, 0.0, 0.0),
                zone_size: Vector3::new(3.0, 0.4, 0.4),
                velocity: Vector3::new(0.2, 0.0, 0.0),
                running: true,
            },
        });
        scene
    }

    fn tool_pose(scene: &Scene, q: &[f64]) -> Isometry3<f64> {
        let link = scene.robot().link_index("tool").unwrap();
        scene.fk(q).unwrap()[link]
    }

    /// The taught descent is carried by the belt: the tool meets the part
    /// where the part *is*, not where it was taught.
    #[test]
    fn tracked_ramp_follows_the_moving_part() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "descend",
                    vec![Action::StartRamp {
                        robot: None,
                        // taught: straight down onto the part at x = 0
                        targets: vec![("jz".into(), 0.05)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();
        assert!((tl.duration - 0.5).abs() < 1e-9, "{}", tl.duration);

        // The part has travelled 0.2 * 0.5 = 0.1 m; so has the tool, which
        // also completed the taught 0.35 m descent.
        let end = tool_pose(&scene, &tl.robots[0].trajectory.sample(tl.duration));
        assert!(
            (end.translation.x - 0.1).abs() < 1e-4,
            "{}",
            end.translation.x
        );
        assert!(
            (end.translation.z - 0.05).abs() < 1e-4,
            "{}",
            end.translation.z
        );
        // Mid-ramp the tool sits over the part throughout, not behind it.
        for i in 0..=10 {
            let t = tl.duration * f64::from(i) / 10.0;
            let pose = tool_pose(&scene, &tl.robots[0].trajectory.sample(t));
            assert!(
                (pose.translation.x - 0.2 * t).abs() < 1e-3,
                "t={t}: tool x {} vs part {}",
                pose.translation.x,
                0.2 * t
            );
        }
    }

    /// Grasping the tracked part freezes the offset, so the lift after it is
    /// straight up — the part is not dragged back to the taught station.
    #[test]
    fn grasp_freezes_the_offset_and_the_lift_stays_put() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "descend",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("jz".into(), 0.05)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "part".into(),
                        link: Some("tool".into()),
                        touch_links: Some(vec!["tool".into()]),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "lift",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("jz".into(), 0.4)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
                step(
                    "release",
                    vec![Action::Untrack { robot: None }],
                    Condition::Immediately,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();
        let track = tl
            .objects
            .iter()
            .find(|o| o.name == "part")
            .expect("the grasped part is tracked");
        let poses = |t: f64| {
            let q = tl.robots[0].trajectory.sample(t);
            let link_poses = scene.fk(&q).unwrap();
            (
                tool_pose(&scene, &q),
                SequenceTimeline::object_pose(track, std::slice::from_ref(&link_poses), t).unwrap(),
            )
        };
        let (_, at_grasp) = poses(0.5);
        let (tool_end, at_end) = poses(tl.duration);
        // Lifted straight up from where it was caught.
        assert!((at_end.translation.x - at_grasp.translation.x).abs() < 1e-6);
        assert!((at_end.translation.z - at_grasp.translation.z - 0.35).abs() < 1e-4);
        // ... and it is still in the gripper.
        assert!((tool_end.translation.x - at_end.translation.x).abs() < 1e-5);
    }

    /// Releasing a track never moves the robot: the world-frame moves after
    /// it start from wherever the tracked part left the arm.
    #[test]
    fn untrack_holds_the_pose_it_reached() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: Some("tool".into()),
                    }],
                    Condition::Immediately,
                ),
                step("follow", vec![], Condition::Elapsed { seconds: 0.5 }),
                step(
                    "release",
                    vec![Action::Untrack { robot: None }],
                    Condition::Immediately,
                ),
                step("settle", vec![], Condition::Elapsed { seconds: 0.2 }),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();
        let at_release = tool_pose(&scene, &tl.robots[0].trajectory.sample(0.5));
        let at_end = tool_pose(&scene, &tl.robots[0].trajectory.sample(tl.duration));
        assert!((at_release.translation.x - 0.1).abs() < 1e-4);
        assert!(
            (at_end.translation.vector - at_release.translation.vector).norm() < 1e-9,
            "the robot moved after the release"
        );
    }

    /// Servoing a fingertip while the gripper ramps is rejected: the solve
    /// could spend the grip chasing the part (the default link — the tool
    /// mount — is what keeps this from happening in the first place).
    #[test]
    fn ramping_the_tracked_gripper_is_rejected() {
        let urdf = r#"
        <robot name="arm">
          <link name="base"/>
          <link name="wrist"/>
          <link name="left"/>
          <link name="right"/>
          <joint name="elbow" type="prismatic">
            <parent link="base"/><child link="wrist"/>
            <axis xyz="1 0 0"/>
            <limit lower="-1" upper="1" effort="1" velocity="1"/>
          </joint>
          <joint name="finger_left" type="prismatic">
            <parent link="wrist"/><child link="left"/>
            <axis xyz="0 1 0"/>
            <limit lower="0" upper="0.04" effort="1" velocity="1"/>
          </joint>
          <joint name="finger_right" type="prismatic">
            <parent link="wrist"/><child link="right"/>
            <axis xyz="0 -1 0"/>
            <limit lower="0" upper="0.04" effort="1" velocity="1"/>
          </joint>
        </robot>"#;
        let mut scene = Scene::new(std::sync::Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ));
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                iso(0.0, 0.0, 0.0),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: Some("left".into()),
                    }],
                    Condition::Immediately,
                ),
                step(
                    "close",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("finger_left".into(), 0.02)],
                        duration: 0.4,
                    }],
                    Condition::Done,
                ),
            ],
        });
        let err = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .expect_err("the gripper joint moves the servoed link")
            .to_string();
        assert!(err.contains("fights the track"), "{err}");
        assert!(err.contains("wrist"), "{err}");
    }

    /// A mimic joint is commandable, just not directly — the rejection has
    /// to name the joint that actually drives it.
    #[test]
    fn ramping_a_mimic_joint_points_at_its_source() {
        let urdf = r#"
        <robot name="arm">
          <link name="base"/><link name="left"/><link name="right"/>
          <joint name="finger_left" type="prismatic">
            <parent link="base"/><child link="left"/>
            <origin xyz="0 0.02 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="0" upper="0.04" effort="1" velocity="1"/>
          </joint>
          <joint name="finger_right" type="prismatic">
            <parent link="base"/><child link="right"/>
            <origin xyz="0 -0.02 0"/>
            <axis xyz="0 1 0"/>
            <limit lower="-0.04" upper="0" effort="1" velocity="1"/>
            <mimic joint="finger_left" multiplier="-1"/>
          </joint>
        </robot>"#;
        let mut scene = Scene::new(std::sync::Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ));
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "close",
                vec![Action::StartRamp {
                    robot: None,
                    targets: vec![("finger_right".into(), -0.02)],
                    duration: 0.4,
                }],
                Condition::Done,
            )],
        });
        let err = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .expect_err("a driven joint cannot be ramped on its own")
            .to_string();
        assert!(err.contains("follows `finger_left`"), "{err}");
    }

    /// Authoring-time rules: one track at a time, no stray release, and no
    /// planned motions while the frame is moving.
    #[test]
    fn tracking_rules_are_validated() {
        let base = cell();
        let check = |steps: Vec<Step>, needle: &str| {
            let mut scene = base.clone();
            scene
                .add_segment(
                    "go",
                    crate::motion::Segment {
                        kind: crate::motion::SegmentKind::Joint,
                        goal_positions: vec![0.1, 0.0, 0.4],
                        constraints: vec![],
                    },
                )
                .unwrap();
            scene.upsert_sequence(Sequence {
                name: "s".into(),
                steps,
            });
            let err = scene
                .simulate_sequence("s", &RolloutOptions::default())
                .expect_err("expected `{needle}`")
                .to_string();
            assert!(err.contains(needle), "expected `{needle}` in `{err}`");
        };
        let track = || Action::Track {
            robot: None,
            object: "part".into(),
            link: None,
        };
        check(
            vec![step(
                "twice",
                vec![track(), track()],
                Condition::Immediately,
            )],
            "already tracking",
        );
        check(
            vec![step(
                "loose",
                vec![Action::Untrack { robot: None }],
                Condition::Immediately,
            )],
            "without an active track",
        );
        check(
            vec![
                step("latch", vec![track()], Condition::Immediately),
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
            ],
            "release the track first",
        );
        check(
            vec![step(
                "ghost",
                vec![Action::Track {
                    robot: None,
                    object: "nope".into(),
                    link: None,
                }],
                Condition::Immediately,
            )],
            "unknown obstacle",
        );
    }
}

#[cfg(test)]
mod vehicle_tests {
    use super::tests::*;
    use super::*;
    use crate::seq::{Device, DeviceKind, Step, VehiclePath};
    use botrail_model::Geometry;
    use nalgebra::{Point3, Translation3, UnitQuaternion};
    use std::f64::consts::{FRAC_PI_2, PI};

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// An L: 2 m along +x, then 1 m along +y. Stations at both ends.
    fn l_path() -> VehiclePath {
        VehiclePath {
            waypoints: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
            ],
            stations: vec![("a".into(), 0), ("c".into(), 2)],
            ring: false,
        }
    }

    /// 0.5 m/s cruise, 90°/s pivot: the L takes 4 + 1 + 2 = 7 s.
    fn agv(body: Vec<String>) -> Device {
        Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: l_path(),
                body,
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade: None,
                },
                tray: None,
            },
        }
    }

    fn goto(station: &str) -> Action {
        Action::Device {
            device: "agv".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn device_done() -> Condition {
        Condition::DeviceDone {
            device: "agv".into(),
        }
    }

    fn chassis_scene() -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                iso(0.3, 0.2, 0.1),
            )
            .unwrap();
        scene.upsert_device(agv(vec!["chassis".into()]));
        scene
    }

    /// No robot drives, so `object_pose` never needs FK.
    fn no_fk(scene: &Scene) -> Vec<Vec<Isometry3<f64>>> {
        scene
            .robots()
            .iter()
            .map(|r| vec![Isometry3::identity(); r.model.links.len()])
            .collect()
    }

    #[test]
    fn vehicle_arrives_at_the_analytic_time_with_the_body_carried() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![
                step("go", vec![goto("c")], device_done()),
                step("dwell", vec![], Condition::Elapsed { seconds: 0.2 }),
            ],
        });
        let tl = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap();

        // 2 m at 0.5 + 90° at 90°/s + 1 m at 0.5, plus the dwell.
        assert!(
            (tl.duration - 7.2).abs() < 0.021,
            "duration = {}",
            tl.duration
        );
        let arrival = tl.step_spans[0].end;
        assert!((arrival - 7.0).abs() < 0.011, "arrival = {arrival}");

        // The moving lane goes on at dispatch, off at arrival.
        let lane = tl.signals.iter().find(|s| s.name == "agv").unwrap();
        assert_eq!(lane.edges.len(), 3, "{:?}", lane.edges);
        assert_eq!(lane.edges[1], (0.0, true));
        assert!(!lane.edges[2].1 && (lane.edges[2].0 - 7.0).abs() < 0.011);

        // Net rigid motion: +2 x, pivot +90° about (2, 0), +1 y.
        let track = tl.objects.iter().find(|o| o.name == "chassis").unwrap();
        let end = SequenceTimeline::object_pose(track, &no_fk(&scene), tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(1.8, 1.3, 0.1)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        let quarter = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        assert!(end.rotation.angle_to(&quarter) < 1e-9);
        // After arrival the body settles into a hold at the parked pose.
        assert!(matches!(track.spans.last(), Some(TrackSpan::Hold { .. })));
    }

    #[test]
    fn vehicle_spans_bake_one_span_per_leg_and_resample_exactly() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap();
        let track = tl.objects.iter().find(|o| o.name == "chassis").unwrap();

        // Whole legs merge: straight, pivot, straight — three spans, not 700.
        assert_eq!(track.spans.len(), 3, "{:?}", track.spans);
        assert!(matches!(track.spans[0], TrackSpan::Linear { .. }));
        assert!(matches!(track.spans[1], TrackSpan::Pivot { .. }));
        assert!(matches!(track.spans[2], TrackSpan::Linear { .. }));

        let fk = no_fk(&scene);
        // Mid-straight: 1 s at 0.5 m/s along +x.
        let mid = SequenceTimeline::object_pose(track, &fk, 1.0).unwrap();
        assert!((mid.translation.vector - Vector3::new(0.8, 0.2, 0.1)).norm() < 1e-9);
        // Mid-turn (t = 4.5 ⇒ 45° about (2, 0)): closed form, no resample grid.
        let mid = SequenceTimeline::object_pose(track, &fk, 4.5).unwrap();
        let phi = FRAC_PI_2 * 0.5;
        let expected = Vector3::new(
            2.0 + 0.3 * phi.cos() - 0.2 * phi.sin(),
            0.3 * phi.sin() + 0.2 * phi.cos(),
            0.1,
        );
        assert!(
            (mid.translation.vector - expected).norm() < 1e-9,
            "mid = {:?}, expected = {expected:?}",
            mid.translation.vector
        );
        let eighth = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), phi);
        assert!(mid.rotation.angle_to(&eighth) < 1e-9);
    }

    /// A 3 m run rising 0.3 m — a 10 % ramp at 0.5 m/s.
    fn ramp_agv(max_grade: Option<f64>) -> Device {
        Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.3)],
                    stations: vec![("a".into(), 0), ("b".into(), 1)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade,
                },
                tray: None,
            },
        }
    }

    fn ramp_scene(max_grade: Option<f64>) -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                iso(0.3, 0.2, 0.1),
            )
            .unwrap();
        scene.upsert_device(ramp_agv(max_grade));
        scene.upsert_sequence(Sequence {
            name: "up".into(),
            steps: vec![step("go", vec![goto("b")], device_done())],
        });
        scene
    }

    #[test]
    fn a_graded_leg_climbs_at_the_analytic_rate() {
        let scene = ramp_scene(Some(0.2));
        let tl = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap();
        // Cruise speed is spent along the 3D path, so the ramp takes
        // sqrt(3² + 0.3²) / 0.5 s — a hair longer than its plan view.
        let length = (3.0f64.powi(2) + 0.3f64.powi(2)).sqrt();
        assert!(
            (tl.duration - length / 0.5).abs() < 0.011,
            "duration = {}",
            tl.duration
        );
        let track = tl.objects.iter().find(|o| o.name == "chassis").unwrap();
        let fk = no_fk(&scene);
        let end = SequenceTimeline::object_pose(track, &fk, tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(3.3, 0.2, 0.4)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        // Halfway through the drive, halfway up — the Lin span is 3D and
        // closed-form.
        let mid = SequenceTimeline::object_pose(track, &fk, length / 0.5 / 2.0).unwrap();
        assert!(
            (mid.translation.vector - Vector3::new(1.8, 0.2, 0.25)).norm() < 1e-9,
            "mid = {:?}",
            mid.translation.vector
        );
    }

    #[test]
    fn the_vehicle_frame_is_its_own_track() {
        // On the ramp: the frame climbs to the top and never turns.
        let scene = ramp_scene(Some(0.2));
        let tl = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap();
        let track = tl.vehicles.iter().find(|v| v.name == "agv").unwrap();
        let end = SequenceTimeline::span_pose(&track.spans, &[], tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(3.0, 0.0, 0.3)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        assert!(end.rotation.angle_to(&UnitQuaternion::identity()) < 1e-9);

        // On the L: the frame arrives at the far station turned with the
        // machine — this is what places a mounted sensor during playback.
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap();
        let track = tl.vehicles.iter().find(|v| v.name == "agv").unwrap();
        // One span per leg, then the arrival hold at the parked frame.
        assert_eq!(track.spans.len(), 4, "{:?}", track.spans);
        assert!(matches!(track.spans.last(), Some(TrackSpan::Hold { .. })));
        let end = SequenceTimeline::span_pose(&track.spans, &[], tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(2.0, 1.0, 0.0)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        let quarter = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        assert!(end.rotation.angle_to(&quarter) < 1e-9);
    }

    #[test]
    fn grade_rules_name_the_slope_the_limit_and_the_vertical() {
        // A climb with no declared ability is refused with the angle.
        let scene = ramp_scene(None);
        let err = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("declares no max_grade") && err.contains("5.7"),
            "{err}"
        );
        // Too weak a drive is named with both numbers.
        let scene = ramp_scene(Some(0.05));
        let err = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("over the drive's max_grade"), "{err}");
        // A vertical stack is never a drive's job, whatever the ability.
        let mut scene = ramp_scene(Some(5.0));
        let mut vertical = ramp_agv(Some(5.0));
        if let DeviceKind::Vehicle { path, .. } = &mut vertical.kind {
            path.waypoints = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 1.0)];
        }
        scene.upsert_device(vertical);
        let err = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("vertical"), "{err}");
    }

    #[test]
    fn legacy_two_element_waypoints_read_as_floor_points() {
        let msg: crate::wire::VehiclePathMsg = serde_json::from_str(
            r#"{"waypoints": [[1.0, 2.0], [3.0, 4.0, 0.5]], "stations": [], "ring": false}"#,
        )
        .unwrap();
        assert_eq!(msg.waypoints, vec![[1.0, 2.0, 0.0], [3.0, 4.0, 0.5]]);
    }

    #[test]
    fn a_wheeled_vehicle_still_hits_a_tread() {
        // Walkable only excuses the machine walking on it: an AGV driven
        // into the flight fails its aisle check like into anything else.
        let mut scene = ramp_scene(Some(0.2));
        scene
            .add_obstacle(
                "tread",
                Geometry::Box {
                    size: Vector3::new(0.3, 1.0, 0.1),
                },
                iso(1.5, 0.2, 0.1),
            )
            .unwrap();
        scene.set_obstacle_walkable("tread", true).unwrap();
        let err = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap_err();
        assert!(matches!(err, SeqError::VehicleCollision { .. }), "{err}");
    }

    #[test]
    fn a_drive_through_a_parked_robot_is_named() {
        // The obstacle-only aisle check cannot see robot links; the
        // vehicle-vs-robot check can — the arm a path was taught too
        // close to.
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                iso(-0.7, 0.0, 0.1),
            )
            .unwrap();
        let mut device = ramp_agv(None);
        if let DeviceKind::Vehicle { path, .. } = &mut device.kind {
            path.waypoints = vec![Point3::new(-1.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)];
        }
        scene.upsert_device(device);
        scene.upsert_sequence(Sequence {
            name: "up".into(),
            steps: vec![step("go", vec![goto("b")], device_done())],
        });
        let err = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            matches!(&err, SeqError::VehicleRobotCollision { body, .. } if body == "chassis"),
            "{err}"
        );
        assert!(err.to_string().contains("hits robot"), "{err}");
    }

    #[test]
    fn walkable_is_an_upright_box_and_the_query_knows_its_margins() {
        let mut scene = sample_scene();
        let yaw = std::f64::consts::FRAC_PI_4;
        scene
            .add_obstacle(
                "tread",
                Geometry::Box {
                    size: Vector3::new(0.4, 0.2, 0.05),
                },
                Isometry3::from_parts(
                    nalgebra::Translation3::new(1.0, 2.0, 0.175),
                    UnitQuaternion::from_axis_angle(&Vector3::z_axis(), yaw),
                ),
            )
            .unwrap();
        scene.set_obstacle_walkable("tread", true).unwrap();
        // A point 0.15 along the box's local +x: margins (0.05, 0.10).
        let (px, py) = (1.0 + 0.15 * yaw.cos(), 2.0 + 0.15 * yaw.sin());
        let (top, _, margin) = scene.floor_support(px, py, 0.1, 0.3).unwrap();
        assert!((top - 0.2).abs() < 1e-12 && (margin - 0.05).abs() < 1e-12);
        // Outside the face, or outside the reach band: no support.
        assert!(scene
            .floor_support(1.0 - 0.3 * yaw.cos(), 2.0 - 0.3 * yaw.sin(), 0.1, 0.3)
            .is_none());
        assert!(scene.floor_support(px, py, 3.0, 0.3).is_none());

        // Only an upright box can say what its top face is.
        scene
            .add_obstacle("ball", Geometry::Sphere { radius: 0.1 }, iso(0.0, 3.0, 0.1))
            .unwrap();
        let err = scene.set_obstacle_walkable("ball", true).unwrap_err();
        assert!(err.to_string().contains("not a box"), "{err}");
        scene
            .add_obstacle(
                "ramp",
                Geometry::Box {
                    size: Vector3::new(0.4, 0.2, 0.05),
                },
                Isometry3::from_parts(
                    nalgebra::Translation3::new(0.0, 4.0, 0.1),
                    UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2),
                ),
            )
            .unwrap();
        let err = scene.set_obstacle_walkable("ramp", true).unwrap_err();
        assert!(err.to_string().contains("tilted"), "{err}");
    }

    /// A shaft next to a two-floor path: lobby → car (1F), a lift edge up
    /// to the car at 2F, then a dock on the mezzanine. The robot rides
    /// the vehicle; the tote rides its deck; `loose` sits on the car
    /// floor; `bystander` watches from outside.
    fn lift_scene() -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.5, 0.4, 0.2),
                },
                iso(0.0, 0.0, 0.15),
            )
            .unwrap();
        scene
            .add_obstacle(
                "tote",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.1),
                },
                iso(0.0, 0.0, 0.35),
            )
            .unwrap();
        scene
            .add_obstacle(
                "loose",
                Geometry::Box {
                    size: Vector3::new(0.15, 0.15, 0.15),
                },
                iso(1.8, 0.3, 0.075),
            )
            .unwrap();
        scene
            .add_obstacle(
                "bystander",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.0, 1.5, 0.05),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(1.5, 0.0, 0.0),
                        Point3::new(1.5, 0.0, 2.0),
                        Point3::new(2.3, 0.0, 2.0),
                    ],
                    stations: vec![("lobby".into(), 0), ("car".into(), 1), ("dock".into(), 3)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "lobby".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade: None,
                },
                tray: Some((iso(0.0, 0.0, 0.35), Vector3::new(0.5, 0.4, 0.2))),
            },
        });
        scene.upsert_device(Device {
            name: "lift".into(),
            kind: DeviceKind::Lift {
                car: Vec::new(),
                zone_pose: iso(1.5, 0.0, 1.0),
                zone_size: Vector3::new(1.2, 1.2, 2.2),
                axis: nalgebra::Unit::new_normalize(Vector3::z()),
                speed: 0.6,
                stops: vec![("a".into(), 0.0), ("b".into(), 2.0)],
                start: "a".into(),
            },
        });
        scene
            .mount_robot_with(0, "agv", Some(Isometry3::translation(0.1, 0.0, 0.3)), None)
            .unwrap();
        scene
    }

    fn lift_cmd(stop: &str) -> Action {
        Action::Device {
            device: "lift".into(),
            command: DeviceCommand::MoveToStop(stop.into()),
        }
    }

    fn lift_done() -> Condition {
        Condition::DeviceDone {
            device: "lift".into(),
        }
    }

    #[test]
    fn a_lift_carries_the_vehicle_whole() {
        let mut scene = lift_scene();
        scene.upsert_sequence(Sequence {
            name: "up".into(),
            steps: vec![
                step("board", vec![goto("car")], device_done()),
                step("ride", vec![lift_cmd("b")], lift_done()),
                step("alight", vec![goto("dock")], device_done()),
            ],
        });
        let tl = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap();
        // 1.5 m at 0.5 + 2.0 m at 0.6 + 0.8 m at 0.5 (no turns).
        assert!(
            (tl.duration - (3.0 + 2.0 / 0.6 + 1.6)).abs() < 0.05,
            "duration = {}",
            tl.duration
        );
        let fk = no_fk(&scene);
        let end_of = |name: &str| {
            let track = tl.objects.iter().find(|o| o.name == name).unwrap();
            SequenceTimeline::object_pose(track, &fk, tl.duration).unwrap()
        };
        // The chassis, the deck load and the car-floor part all rode; the
        // bystander did not.
        assert!(
            (end_of("chassis").translation.vector - Vector3::new(2.3, 0.0, 2.15)).norm() < 1e-9
        );
        assert!((end_of("tote").translation.vector - Vector3::new(2.3, 0.0, 2.35)).norm() < 1e-9);
        assert!((end_of("loose").translation.vector - Vector3::new(1.8, 0.3, 2.075)).norm() < 1e-9);
        assert!(tl.objects.iter().all(|o| o.name != "bystander"));
        // The mounted robot's base rode the same rigid motion.
        let base = SequenceTimeline::base_pose(&tl.robots[0], tl.duration).unwrap();
        assert!(
            (base.translation.vector - Vector3::new(2.4, 0.0, 2.3)).norm() < 1e-9,
            "base = {:?}",
            base.translation.vector
        );
        // And the vehicle's own frame track ends at the dock, re-anchored.
        let track = tl.vehicles.iter().find(|v| v.name == "agv").unwrap();
        let end = SequenceTimeline::span_pose(&track.spans, &[], tl.duration).unwrap();
        assert!((end.translation.vector - Vector3::new(2.3, 0.0, 2.0)).norm() < 1e-9);
        // The lift's lane went on at dispatch, off at arrival.
        let lane = tl.signals.iter().find(|s| s.name == "lift").unwrap();
        assert_eq!(lane.edges.len(), 3, "{:?}", lane.edges);
    }

    #[test]
    fn boarding_half_out_is_refused_by_name() {
        let mut scene = lift_scene();
        // A mast bolted to the chassis but standing outside the zone.
        scene
            .add_obstacle(
                "mast",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.6),
                },
                iso(0.8, 0.0, 0.3),
            )
            .unwrap();
        let mut agv = scene
            .devices()
            .iter()
            .find(|d| d.name == "agv")
            .unwrap()
            .clone();
        if let DeviceKind::Vehicle { body, .. } = &mut agv.kind {
            body.push("mast".into());
        }
        scene.upsert_device(agv);
        scene.upsert_sequence(Sequence {
            name: "up".into(),
            steps: vec![
                step("board", vec![goto("car")], device_done()),
                step("ride", vec![lift_cmd("b")], lift_done()),
            ],
        });
        let err = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("half out") && err.contains("mast"), "{err}");
    }

    #[test]
    fn a_goto_across_the_lift_edge_is_refused() {
        let mut scene = lift_scene();
        scene.upsert_sequence(Sequence {
            name: "jump".into(),
            steps: vec![step("go", vec![goto("dock")], device_done())],
        });
        let err = scene
            .simulate_sequence("jump", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("across the lift edge"), "{err}");
    }

    #[test]
    fn a_vertical_edge_without_a_lift_is_still_refused() {
        let mut scene = lift_scene();
        scene.remove_device("lift").unwrap();
        scene.upsert_sequence(Sequence {
            name: "up".into(),
            steps: vec![step("go", vec![goto("car")], device_done())],
        });
        let err = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("lift's job"), "{err}");
    }

    #[test]
    fn riding_and_travelling_guard_each_other() {
        // The lift must not move while its passenger still drives...
        let mut scene = lift_scene();
        scene.upsert_sequence(Sequence {
            name: "early".into(),
            steps: vec![
                step(
                    "board",
                    // Inside the zone (x >= 0.9 from t = 1.8) but still
                    // driving: too early either way, but *this* is the
                    // case the guard must name.
                    vec![goto("car")],
                    Condition::Elapsed { seconds: 2.5 },
                ),
                step("ride", vec![lift_cmd("b")], lift_done()),
            ],
        });
        let err = scene
            .simulate_sequence("early", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("still travelling"), "{err}");

        // ...and the passenger must not drive while the lift still moves.
        let mut scene = lift_scene();
        scene.upsert_sequence(Sequence {
            name: "eager".into(),
            steps: vec![
                step("board", vec![goto("car")], device_done()),
                step(
                    "ride",
                    vec![lift_cmd("b")],
                    Condition::Elapsed { seconds: 0.5 },
                ),
                step("alight", vec![goto("dock")], device_done()),
            ],
        });
        let err = scene
            .simulate_sequence("eager", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("riding lift"), "{err}");
    }

    #[test]
    fn a_stop_between_floors_leaves_the_vehicle_off_its_path() {
        let mut scene = lift_scene();
        let mut lift = scene
            .devices()
            .iter()
            .find(|d| d.name == "lift")
            .unwrap()
            .clone();
        if let DeviceKind::Lift { stops, .. } = &mut lift.kind {
            stops.push(("mid".into(), 1.0));
        }
        scene.upsert_device(lift);
        scene.upsert_sequence(Sequence {
            name: "half".into(),
            steps: vec![
                step("board", vec![goto("car")], device_done()),
                step("ride", vec![lift_cmd("mid")], lift_done()),
                step("alight", vec![goto("dock")], device_done()),
            ],
        });
        let err = scene
            .simulate_sequence("half", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("off its path"), "{err}");
    }

    fn drone(
        path: Vec<Point3<f64>>,
        stations: Vec<(String, usize)>,
        yaw: crate::seq::AerialYaw,
    ) -> Device {
        Device {
            name: "drone".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: path,
                    stations,
                    ring: false,
                },
                body: vec!["airframe".into()],
                speed: 0.8,
                turn_speed: FRAC_PI_2,
                start: "pad".into(),
                drive: crate::seq::Drive::Aerial {
                    climb_speed: 0.6,
                    descent_speed: 0.9,
                    yaw,
                },
                tray: None,
            },
        }
    }

    fn drone_scene(device: Device, body_at: Isometry3<f64>) -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "airframe",
                Geometry::Box {
                    size: Vector3::new(0.36, 0.36, 0.12),
                },
                body_at,
            )
            .unwrap();
        scene.upsert_device(device);
        scene
    }

    fn drone_goto(station: &str) -> Action {
        Action::Device {
            device: "drone".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn drone_done() -> Condition {
        Condition::DeviceDone {
            device: "drone".into(),
        }
    }

    #[test]
    fn an_aerial_leg_flies_each_axis_at_its_own_limit() {
        // Up (climb-limited), out (cruise-limited), down-and-out
        // (cruise-limited diagonal): T = max(run/0.8, rise/0.6 or fall/0.9).
        let mut scene = drone_scene(
            drone(
                vec![
                    Point3::new(0.0, 2.0, 0.0),
                    Point3::new(0.0, 2.0, 2.4),
                    Point3::new(3.0, 2.0, 2.4),
                    Point3::new(6.0, 2.0, 1.2),
                ],
                vec![("pad".into(), 0), ("away".into(), 3)],
                crate::seq::AerialYaw::Course,
            ),
            iso(0.0, 2.0, 0.06),
        );
        scene.upsert_sequence(Sequence {
            name: "fly".into(),
            steps: vec![step("go", vec![drone_goto("away")], drone_done())],
        });
        let tl = scene
            .simulate_sequence("fly", &RolloutOptions::default())
            .unwrap();
        // 2.4/0.6 + 3/0.8 + max(3/0.8, 1.2/0.9) = 4.0 + 3.75 + 3.75.
        assert!(
            (tl.duration - 11.5).abs() < 0.03,
            "duration = {}",
            tl.duration
        );
        let track = tl.objects.iter().find(|o| o.name == "airframe").unwrap();
        let fk = no_fk(&scene);
        // Mid-climb, and mid-diagonal — closed form on the 3D velocity.
        let mid = SequenceTimeline::object_pose(track, &fk, 2.0).unwrap();
        assert!((mid.translation.vector - Vector3::new(0.0, 2.0, 1.26)).norm() < 1e-9);
        let mid = SequenceTimeline::object_pose(track, &fk, 9.75).unwrap();
        // 2.0 s into the last leg: x += 1.6, z -= 0.64.
        assert!(
            (mid.translation.vector - Vector3::new(4.6, 2.0, 1.82)).norm() < 1e-9,
            "mid = {:?}",
            mid.translation.vector
        );
    }

    #[test]
    fn airborne_counts_flight_and_hover_but_not_the_pad() {
        // Up (2.4/0.6), hover 1.5 s, down (2.4/0.9), 2 s on the pad, up
        // again: everything counts except the wait on the pad. The figure
        // is what a declared flight time must cover (design-drone.md §3.2).
        let mut scene = drone_scene(
            drone(
                vec![Point3::new(0.0, 2.0, 0.0), Point3::new(0.0, 2.0, 2.4)],
                vec![("pad".into(), 0), ("up".into(), 1)],
                crate::seq::AerialYaw::Course,
            ),
            iso(0.0, 2.0, 0.06),
        );
        scene.upsert_sequence(Sequence {
            name: "fly".into(),
            steps: vec![
                step("t1", vec![drone_goto("up")], drone_done()),
                step("hover", vec![], Condition::Elapsed { seconds: 1.5 }),
                step("l1", vec![drone_goto("pad")], drone_done()),
                step("recharge", vec![], Condition::Elapsed { seconds: 2.0 }),
                step("t2", vec![drone_goto("up")], drone_done()),
            ],
        });
        let tl = scene
            .simulate_sequence("fly", &RolloutOptions::default())
            .unwrap();
        let airborne = tl.vehicle_airborne("drone").unwrap();
        let expect = 2.4 / 0.6 + 1.5 + 2.4 / 0.9 + 2.4 / 0.6;
        assert!(
            (airborne - expect).abs() < 0.05,
            "airborne = {airborne}, expect {expect}"
        );
        assert!((tl.duration - (expect + 2.0)).abs() < 0.05);
        assert!(tl.vehicle_airborne("nope").is_none());
    }

    /// A machine with one free-spinning rotor and one limited fin — the
    /// smallest model that can tell a rotor from a joint that must not spin.
    const ROTORCRAFT: &str = r#"<robot name="rotorcraft">
      <link name="body"/>
      <link name="rotor"/>
      <joint name="rotor_joint" type="continuous">
        <parent link="body"/><child link="rotor"/>
        <origin xyz="0 0 0.05"/><axis xyz="0 0 1"/>
      </joint>
      <link name="fin"/>
      <joint name="tilt" type="revolute">
        <parent link="body"/><child link="fin"/>
        <origin xyz="0.1 0 0"/><axis xyz="0 1 0"/>
        <limit lower="-1" upper="1" effort="1" velocity="1"/>
      </joint>
    </robot>"#;

    #[test]
    fn a_spinning_mount_turns_in_the_air_and_rests_on_the_pad() {
        use std::sync::Arc;
        // Presentation only, but exact: the rotor's total turn equals
        // rate x the airborne seconds `vehicle_airborne` reports — spin
        // through the climb, the hover and the descent, none across the
        // pad wait — and the limited fin never moves.
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(ROTORCRAFT).unwrap(),
        ));
        let mut device = drone(
            vec![Point3::new(0.0, 2.0, 0.0), Point3::new(0.0, 2.0, 1.2)],
            vec![("pad".into(), 0), ("up".into(), 1)],
            crate::seq::AerialYaw::Course,
        );
        if let DeviceKind::Vehicle { body, .. } = &mut device.kind {
            body.clear(); // the machine *is* the robot — a UAV mount
        }
        scene.upsert_device(device);
        scene
            .mount_robot_with(0, "drone", Some(Isometry3::identity()), None)
            .unwrap();
        // The declaration is validated, by name: unknown, limited, zero.
        let err = scene
            .set_mount_spin(0, vec![("nope".into(), 40.0)])
            .unwrap_err()
            .to_string();
        assert!(err.contains("not an actuated joint"), "{err}");
        let err = scene
            .set_mount_spin(0, vec![("tilt".into(), 40.0)])
            .unwrap_err()
            .to_string();
        assert!(err.contains("continuous"), "{err}");
        let err = scene
            .set_mount_spin(0, vec![("rotor_joint".into(), 0.0)])
            .unwrap_err()
            .to_string();
        assert!(err.contains("non-zero"), "{err}");
        scene
            .set_mount_spin(0, vec![("rotor_joint".into(), 40.0)])
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "fly".into(),
            steps: vec![
                step("t1", vec![drone_goto("up")], drone_done()),
                step("hover", vec![], Condition::Elapsed { seconds: 1.0 }),
                step("l1", vec![drone_goto("pad")], drone_done()),
                step("recharge", vec![], Condition::Elapsed { seconds: 2.0 }),
                step("t2", vec![drone_goto("up")], drone_done()),
            ],
        });
        let tl = scene
            .simulate_sequence("fly", &RolloutOptions::default())
            .unwrap();
        let airborne = tl.vehicle_airborne("drone").unwrap();
        let track = &tl.robots[0].trajectory;
        let total = track.sample(tl.duration)[0] - track.sample(0.0)[0];
        assert!(
            (total - 40.0 * airborne).abs() < 1.0,
            "rotor turned {total:.1} rad over {airborne:.2} s airborne"
        );
        // The pad wait spans ~[4.34, 6.34]: the phase holds across it.
        let parked = track.sample(4.6)[0];
        assert!(
            (track.sample(6.2)[0] - parked).abs() < 1e-9,
            "the rotor must rest on the pad"
        );
        assert!(track.sample(4.6)[0] > 1.0, "it flew first");
        // The limited fin is never touched.
        assert!(track.sample(tl.duration)[1].abs() < 1e-12);
    }

    #[test]
    fn course_yaw_turns_and_fixed_yaw_holds() {
        // Up, out along +x, then out along +y: the course changes
        // mid-flight, so Course yaw must turn between the cruise legs.
        // (Parking already faces the first course — `heading_at` reads
        // past the vertical leg.)
        let path = vec![
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(0.0, 2.0, 2.0),
            Point3::new(2.0, 2.0, 2.0),
            Point3::new(2.0, 5.0, 2.0),
        ];
        let stations = vec![("pad".into(), 0), ("away".into(), 3)];
        let mut scene = drone_scene(
            drone(
                path.clone(),
                stations.clone(),
                crate::seq::AerialYaw::Course,
            ),
            iso(0.0, 2.0, 0.06),
        );
        scene.upsert_sequence(Sequence {
            name: "fly".into(),
            steps: vec![step("go", vec![drone_goto("away")], drone_done())],
        });
        let tl = scene
            .simulate_sequence("fly", &RolloutOptions::default())
            .unwrap();
        // 2/0.6 + 2/0.8 + 90° at 90°/s + 3/0.8.
        assert!(
            (tl.duration - (2.0 / 0.6 + 2.5 + 1.0 + 3.75)).abs() < 0.03,
            "duration = {}",
            tl.duration
        );
        let track = tl.vehicles.iter().find(|v| v.name == "drone").unwrap();
        let end = SequenceTimeline::span_pose(&track.spans, &[], tl.duration).unwrap();
        let quarter = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        assert!(end.rotation.angle_to(&quarter) < 1e-9);

        // Fixed: one turn to the held yaw, then straights only.
        let mut scene = drone_scene(
            drone(path, stations, crate::seq::AerialYaw::Fixed(0.7)),
            iso(0.0, 2.0, 0.06),
        );
        scene.upsert_sequence(Sequence {
            name: "fly".into(),
            steps: vec![step("go", vec![drone_goto("away")], drone_done())],
        });
        let tl = scene
            .simulate_sequence("fly", &RolloutOptions::default())
            .unwrap();
        // One turn to the held yaw, then straights only — the course
        // change costs nothing.
        assert!(
            (tl.duration - (0.7 / FRAC_PI_2 + 2.0 / 0.6 + 2.5 + 3.75)).abs() < 0.03,
            "duration = {}",
            tl.duration
        );
        let track = tl.vehicles.iter().find(|v| v.name == "drone").unwrap();
        let held = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.7);
        for t in [3.0, 6.0, tl.duration] {
            let pose = SequenceTimeline::span_pose(&track.spans, &[], t).unwrap();
            assert!(pose.rotation.angle_to(&held) < 1e-9, "t = {t}");
        }
    }

    #[test]
    fn a_low_corridor_through_the_arm_is_refused() {
        // The obstacle-only aisle check cannot see the parked arm; the
        // vehicle-vs-robot check is the airspace check.
        let mut scene = drone_scene(
            drone(
                // Low enough to cross the arm's base (the parked upper
                // arm lies higher; between them the air is honestly free).
                vec![Point3::new(-1.0, 0.0, 0.1), Point3::new(1.0, 0.0, 0.1)],
                vec![("pad".into(), 0), ("away".into(), 1)],
                crate::seq::AerialYaw::Course,
            ),
            iso(-1.0, 0.0, 0.1),
        );
        scene.upsert_sequence(Sequence {
            name: "fly".into(),
            steps: vec![step("go", vec![drone_goto("away")], drone_done())],
        });
        let err = scene
            .simulate_sequence("fly", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            matches!(err, SeqError::VehicleRobotCollision { .. }),
            "{err}"
        );
    }

    #[test]
    fn a_holonomic_vehicle_docks_without_turning() {
        let mut scene = chassis_scene();
        let mut agv = scene
            .devices()
            .iter()
            .find(|d| d.name == "agv")
            .unwrap()
            .clone();
        if let DeviceKind::Vehicle { drive, .. } = &mut agv.kind {
            *drive = crate::seq::Drive::Holonomic { max_grade: None };
        }
        scene.upsert_device(agv);
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap();
        // The L costs only its length: 3 m at 0.5 — the pivot second is
        // what those wheels bought away.
        assert!(
            (tl.duration - 6.0).abs() < 0.021,
            "duration = {}",
            tl.duration
        );
        // Pure translation: the body ends +2 x, +1 y, unrotated.
        let track = tl.objects.iter().find(|o| o.name == "chassis").unwrap();
        let end = SequenceTimeline::object_pose(track, &no_fk(&scene), tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(2.3, 1.2, 0.1)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        assert!(end.rotation.angle_to(&UnitQuaternion::identity()) < 1e-9);
        let frame = tl.vehicles.iter().find(|v| v.name == "agv").unwrap();
        let end = SequenceTimeline::span_pose(&frame.spans, &[], tl.duration).unwrap();
        assert!(end.rotation.angle_to(&UnitQuaternion::identity()) < 1e-9);
    }

    #[test]
    fn a_holonomic_drive_shares_the_ground_z_rules() {
        // Same wheels, same floor: a slope still needs the declaration.
        let swap = |scene: &mut Scene, max_grade: Option<f64>| {
            let mut agv = scene
                .devices()
                .iter()
                .find(|d| d.name == "agv")
                .unwrap()
                .clone();
            if let DeviceKind::Vehicle { drive, .. } = &mut agv.kind {
                *drive = crate::seq::Drive::Holonomic { max_grade };
            }
            scene.upsert_device(agv);
        };
        let mut scene = ramp_scene(None);
        swap(&mut scene, None);
        let err = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("declares no max_grade"), "{err}");

        let mut scene = ramp_scene(None);
        swap(&mut scene, Some(0.2));
        let tl = scene
            .simulate_sequence("up", &RolloutOptions::default())
            .unwrap();
        let length = (3.0f64.powi(2) + 0.3f64.powi(2)).sqrt();
        assert!((tl.duration - length / 0.5).abs() < 0.011);
    }

    #[test]
    fn vehicle_round_trip_composes_to_a_half_turn_about_home() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "cycle".into(),
            steps: vec![
                step("out", vec![goto("c")], device_done()),
                step("back", vec![goto("a")], device_done()),
            ],
        });
        let tl = scene
            .simulate_sequence("cycle", &RolloutOptions::default())
            .unwrap();
        // Return: about-face (2 s) + 1 m (2 s) + 90° (1 s) + 2 m (4 s) = 9 s.
        assert!(
            (tl.duration - 16.0).abs() < 0.031,
            "duration = {}",
            tl.duration
        );
        // The vehicle is back at `a` facing −x: the cycle's net rigid motion
        // is a half turn about home, and the body carries it exactly.
        let track = tl.objects.iter().find(|o| o.name == "chassis").unwrap();
        let end = SequenceTimeline::object_pose(track, &no_fk(&scene), tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(-0.3, -0.2, 0.1)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        let half = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI);
        assert!(end.rotation.angle_to(&half) < 1e-9);
    }

    #[test]
    fn vehicle_rollout_is_deterministic() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let options = RolloutOptions::default();
        let a = scene.simulate_sequence("out", &options).unwrap();
        let b = scene.simulate_sequence("out", &options).unwrap();
        assert_eq!(a.duration.to_bits(), b.duration.to_bits());
        let fk = no_fk(&scene);
        let track = |tl: &SequenceTimeline| {
            tl.objects
                .iter()
                .find(|o| o.name == "chassis")
                .unwrap()
                .clone()
        };
        let (ta, tb) = (track(&a), track(&b));
        assert_eq!(ta.spans.len(), tb.spans.len());
        for k in 0..=72 {
            let t = k as f64 * 0.1;
            let pa = SequenceTimeline::object_pose(&ta, &fk, t).unwrap();
            let pb = SequenceTimeline::object_pose(&tb, &fk, t).unwrap();
            for i in 0..3 {
                assert_eq!(
                    pa.translation.vector[i].to_bits(),
                    pb.translation.vector[i].to_bits()
                );
            }
            assert_eq!(
                pa.rotation
                    .coords
                    .iter()
                    .map(|c| c.to_bits())
                    .collect::<Vec<_>>(),
                pb.rotation
                    .coords
                    .iter()
                    .map(|c| c.to_bits())
                    .collect::<Vec<_>>()
            );
        }
    }

    #[test]
    fn a_travelling_vehicle_reports_the_aisle_collision() {
        let mut scene = chassis_scene();
        scene
            .add_obstacle(
                "wall",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.2),
                },
                iso(1.0, 0.2, 0.1),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap_err();
        match err {
            SeqError::VehicleCollision {
                t,
                vehicle,
                body,
                obstacle,
            } => {
                // Faces touch after 0.5 m of travel at 0.5 m/s.
                assert!((0.99..=1.05).contains(&t), "t = {t}");
                assert_eq!(vehicle, "agv");
                assert_eq!(body, "chassis");
                assert_eq!(obstacle, "wall");
            }
            other => panic!("expected VehicleCollision, got {other:?}"),
        }
    }

    #[test]
    fn goto_while_travelling_is_rejected() {
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![
                step("go", vec![goto("c")], Condition::Elapsed { seconds: 0.5 }),
                step("amend", vec![goto("a")], device_done()),
            ],
        });
        let err = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            err.to_string().contains("still travelling"),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn vehicle_validation_names_the_problem() {
        let check = |scene: &Scene, sequence: &str, needle: &str| {
            let err = scene
                .simulate_sequence(sequence, &RolloutOptions::default())
                .unwrap_err();
            assert!(
                matches!(err, SeqError::Validation { .. }) && err.to_string().contains(needle),
                "expected `{needle}` in: {err}"
            );
        };
        // Unknown goto station.
        let mut scene = chassis_scene();
        scene.upsert_sequence(Sequence {
            name: "bad".into(),
            steps: vec![step("go", vec![goto("nowhere")], device_done())],
        });
        check(&scene, "bad", "no station `nowhere`");
        // Goto on a non-vehicle.
        let mut scene = chassis_scene();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.0, 0.5),
                zone_size: Vector3::new(1.0, 1.0, 0.2),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "bad".into(),
            steps: vec![step(
                "go",
                vec![Action::Device {
                    device: "belt".into(),
                    command: DeviceCommand::Goto {
                        station: "a".into(),
                    },
                }],
                Condition::Immediately,
            )],
        });
        check(&scene, "bad", "not a vehicle");
        // A station pointing past the path.
        let mut scene = chassis_scene();
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)],
                    stations: vec![("a".into(), 0), ("ghost".into(), 9)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade: None,
                },
                tray: None,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "any".into(),
            steps: vec![step("noop", vec![], Condition::Immediately)],
        });
        check(&scene, "any", "points at waypoint 9");
    }

    #[test]
    fn a_ring_walks_the_shorter_way_around() {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "cart",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.0, 0.0, 1.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(2.0, 2.0, 0.0),
                        Point3::new(0.0, 2.0, 0.0),
                    ],
                    stations: vec![("a".into(), 0), ("d".into(), 3)],
                    ring: true,
                },
                body: vec!["cart".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade: None,
                },
                tray: None,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "out".into(),
            steps: vec![step("go", vec![goto("d")], device_done())],
        });
        let tl = scene
            .simulate_sequence("out", &RolloutOptions::default())
            .unwrap();
        // Backward around the ring: 90° turn (1 s) + one 2 m side (4 s),
        // not the three-side walk (6 m).
        assert!(
            (tl.duration - 5.0).abs() < 0.011,
            "duration = {}",
            tl.duration
        );
    }

    #[test]
    fn vehicle_device_round_trips_through_wire_and_project() {
        let scene = {
            let mut scene = chassis_scene();
            scene.upsert_sequence(Sequence {
                name: "out".into(),
                steps: vec![step("go", vec![goto("c")], device_done())],
            });
            scene
        };
        let msg = crate::wire::device_msg(&scene.devices()[0]);
        let json = serde_json::to_string(&msg).unwrap();
        let back: crate::wire::DeviceMsg = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
        let rebuilt = crate::wire::device_msg(&crate::wire::device_from_msg(&back));
        assert_eq!(msg, rebuilt);

        // The generated script re-authors the vehicle and the dispatch.
        let code = crate::project::generate_python(&scene.to_project());
        assert!(code.contains("scene.add_vehicle(\"agv\""), "{code}");
        assert!(code.contains("stations={\"a\": 0, \"c\": 2}"), "{code}");
        assert!(code.contains("bt.seq.goto(\"agv\", \"c\")"), "{code}");
        assert!(code.contains("bt.seq.device_done(\"agv\")"), "{code}");
    }
}

#[cfg(test)]
mod tray_tests {
    use super::tests::*;
    use super::*;
    use crate::seq::{Device, DeviceKind, Sensor, SensorKind, SensorWatch, Step, VehiclePath};
    use botrail_model::Geometry;
    use nalgebra::{Point3, Translation3, UnitQuaternion};
    use std::f64::consts::FRAC_PI_2;

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    fn goto(station: &str) -> Action {
        Action::Device {
            device: "agv".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn device_done() -> Condition {
        Condition::DeviceDone {
            device: "agv".into(),
        }
    }

    #[test]
    fn a_cell_with_no_robot_bakes_its_devices() {
        // An AGV loop and its cargo, nobody articulated anywhere — the
        // scene, the bake, the tracks and the checks all run without a
        // robot to lean on.
        let mut scene = Scene::empty();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.4, 0.3, 0.2),
                },
                iso(0.0, 0.0, 0.1),
            )
            .unwrap();
        scene
            .add_obstacle(
                "tote",
                Geometry::Box {
                    size: Vector3::new(0.15, 0.15, 0.1),
                },
                iso(0.0, 0.0, 0.27),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(2.0, 1.0, 0.0),
                    ],
                    stations: vec![("a".into(), 0), ("c".into(), 2)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade: None,
                },
                tray: Some((iso(0.0, 0.0, 0.25), Vector3::new(0.35, 0.3, 0.2))),
            },
        });
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap();
        assert!(tl.robots.is_empty());
        assert!(tl.duration > 0.0);
        // The cargo went the whole route and its track says so.
        let track = tl.objects.iter().find(|o| o.name == "tote").unwrap();
        let end = SequenceTimeline::span_pose(&track.spans, &[], tl.duration).unwrap();
        assert!((end.translation.x - 2.0).abs() < 1e-3, "{end:?}");
        assert!((end.translation.y - 1.0).abs() < 1e-3, "{end:?}");
        // The vehicle frame track is there for the studio to ride.
        assert!(tl.vehicles.iter().any(|v| v.name == "agv"));

        // And the whole cell round-trips: project JSON with `robots: []`,
        // and a generated script that opens an empty scene.
        let json = scene.to_project().to_json();
        let back = crate::project::ProjectFile::from_json(&json).unwrap();
        let rebuilt = Scene::from_project(&back).unwrap();
        assert!(rebuilt.robots().is_empty());
        assert_eq!(rebuilt.obstacles().len(), scene.obstacles().len());
        let code = crate::project::generate_python(&back);
        assert!(code.contains("scene = bt.Scene()\n"), "{code}");
        assert!(code.contains("scene.add_vehicle(\"agv\""), "{code}");
    }

    /// An L with a deck: 2 m along +x, pivot, 1 m along +y. The deck is a
    /// 0.4 m box centred over the frame, 0.2 m up.
    fn deck_scene() -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.4, 0.3, 0.2),
                },
                iso(0.0, 0.0, 0.1),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(2.0, 1.0, 0.0),
                    ],
                    stations: vec![("a".into(), 0), ("c".into(), 2)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade: None,
                },
                tray: Some((iso(0.0, 0.0, 0.2), Vector3::new(0.4, 0.3, 0.2))),
            },
        });
        // The sample arm used to stand (unchecked) inside the parked
        // vehicle; the vehicle-vs-robot check now watches that. These
        // tests are about the deck — stand the arm clear of the route.
        scene.set_robot_base_pose_for(0, Isometry3::translation(-5.0, -5.0, 0.0));
        scene
    }

    fn no_fk(scene: &Scene) -> Vec<Vec<Isometry3<f64>>> {
        scene
            .robots()
            .iter()
            .map(|r| vec![Isometry3::identity(); r.model.links.len()])
            .collect()
    }

    #[test]
    fn a_part_on_the_deck_rides_the_whole_route() {
        let mut scene = deck_scene();
        // Resting on the deck at the start: inside the tray zone.
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.05, 0.0, 0.25),
            )
            .unwrap();
        // Standing beside the vehicle, outside the zone: must not move.
        scene
            .add_obstacle(
                "bystander",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.0, 0.6, 0.05),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap();

        let fk = no_fk(&scene);
        let track = |name: &str| tl.objects.iter().find(|o| o.name == name).cloned();
        let carton = track("carton").expect("the load has a track");
        let end = SequenceTimeline::object_pose(&carton, &fk, tl.duration).unwrap();
        // Same rigid motion as the body: +2 x, pivot +90° about (2, 0), +1 y.
        // The load sat 0.05 ahead of the frame, so the turn swings it to +y.
        assert!(
            (end.translation.vector - Vector3::new(2.0, 1.05, 0.25)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        let quarter = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2);
        assert!(end.rotation.angle_to(&quarter) < 1e-9);
        // It stayed on the deck the whole way: the offset to the body is
        // constant, which is what "rides" has to mean.
        let chassis = track("chassis").expect("the body has a track");
        for k in 0..=70 {
            let t = k as f64 * 0.1;
            let c = SequenceTimeline::object_pose(&carton, &fk, t).unwrap();
            let b = SequenceTimeline::object_pose(&chassis, &fk, t).unwrap();
            let offset = b.inverse() * c;
            assert!(
                (offset.translation.vector - Vector3::new(0.05, 0.0, 0.15)).norm() < 1e-9,
                "t = {t}: {:?}",
                offset.translation.vector
            );
        }
        // Nothing was picked up that was not on the deck.
        assert!(track("bystander").is_none(), "the bystander must not move");
    }

    #[test]
    fn a_part_the_deck_drives_under_joins_the_load() {
        let mut scene = deck_scene();
        // Hovering over the second leg, above the chassis but inside the
        // deck zone: the vehicle drives under it and it becomes cargo. A
        // part only has to *be* on the deck — there is no load action.
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(2.0, 0.6, 0.28),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap();
        let fk = no_fk(&scene);
        let carton = tl.objects.iter().find(|o| o.name == "carton").unwrap();

        // Still where it was left while the vehicle is on the first leg.
        let early = SequenceTimeline::object_pose(carton, &fk, 2.0).unwrap();
        assert!(
            (early.translation.vector - Vector3::new(2.0, 0.6, 0.28)).norm() < 1e-9,
            "early = {:?}",
            early.translation.vector
        );
        // The deck's front edge reaches it at vy = 0.4, i.e. t = 5.8 s, and
        // from there it travels the remaining 0.6 m with the vehicle.
        let end = SequenceTimeline::object_pose(carton, &fk, tl.duration).unwrap();
        assert!(
            (end.translation.vector - Vector3::new(2.0, 1.2, 0.28)).norm() < 1e-9,
            "end = {:?}",
            end.translation.vector
        );
        // Its first span is the rest before it was picked up.
        assert!(matches!(carton.spans.first(), Some(TrackSpan::Hold { .. })));
    }

    #[test]
    fn a_grasped_part_is_the_robots_not_the_decks() {
        let mut scene = deck_scene();
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                iso(0.0, 0.0, 0.25),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "hold".into(),
            steps: vec![
                step(
                    "grasp",
                    vec![Action::Attach {
                        robot: None,
                        object: "carton".into(),
                        link: None,
                        touch_links: None,
                    }],
                    Condition::Immediately,
                ),
                step("go", vec![goto("c")], device_done()),
            ],
        });
        let tl = scene
            .simulate_sequence("hold", &RolloutOptions::default())
            .unwrap();
        let carton = tl.objects.iter().find(|o| o.name == "carton").unwrap();
        // Grasp wins: every span is a Follow, never a vehicle span.
        assert!(
            carton
                .spans
                .iter()
                .all(|s| matches!(s, TrackSpan::Follow { .. })),
            "{:?}",
            carton.spans
        );
    }

    #[test]
    fn a_mounted_sensor_travels_with_its_vehicle() {
        let mut scene = deck_scene();
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.05, 0.0, 0.25),
            )
            .unwrap();
        // A load-present eye over the deck, authored in the vehicle frame.
        scene
            .upsert_sensor(Sensor {
                name: "loaded".into(),
                kind: SensorKind::Zone {
                    pose: iso(0.0, 0.0, 0.25),
                    size: Vector3::new(0.4, 0.3, 0.2),
                },
                watch: SensorWatch::Objects(vec!["carton".into()]),
                mount: Some("agv".into()),
            })
            .unwrap();
        // The same zone bolted to the floor, for contrast.
        scene
            .upsert_sensor(Sensor {
                name: "fixture".into(),
                kind: SensorKind::Zone {
                    pose: iso(0.0, 0.0, 0.25),
                    size: Vector3::new(0.4, 0.3, 0.2),
                },
                watch: SensorWatch::Objects(vec!["carton".into()]),
                mount: None,
            })
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let tl = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap();

        let lane = |name: &str| tl.signals.iter().find(|s| s.name == name).unwrap();
        // The mounted eye sees its load for the whole run — it moves with
        // it. (Every lane seeds false and records its first reading at t=0.)
        let loaded = lane("loaded");
        assert_eq!(
            loaded.edges,
            vec![(0.0, false), (0.0, true)],
            "{:?}",
            loaded.edges
        );
        // The floor fixture sees the same load at t = 0 and loses it as
        // soon as the vehicle pulls away — the mounted/fixed distinction in
        // one pair of lanes.
        let fixture = lane("fixture");
        assert_eq!(fixture.edges.len(), 3, "{:?}", fixture.edges);
        assert_eq!(&fixture.edges[..2], &[(0.0, false), (0.0, true)]);
        let (dropped_at, value) = fixture.edges[2];
        assert!(!value && dropped_at < 1.0, "dropped at {dropped_at}");
    }

    #[test]
    fn the_load_is_checked_against_the_aisle_too() {
        let mut scene = deck_scene();
        // A tall load on the deck, and a bridge the *body* clears (its top
        // is 0.2) but the load does not.
        scene
            .add_obstacle(
                "carton",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.4),
                },
                iso(0.0, 0.0, 0.25),
            )
            .unwrap();
        scene
            .add_obstacle(
                "bridge",
                Geometry::Box {
                    size: Vector3::new(0.1, 1.0, 0.1),
                },
                iso(1.0, 0.0, 0.45),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("haul", &RolloutOptions::default())
            .unwrap_err();
        match err {
            SeqError::VehicleCollision { body, obstacle, .. } => {
                // The *load* is what hits it — the body passes under.
                assert_eq!(body, "carton");
                assert_eq!(obstacle, "bridge");
            }
            other => panic!("expected VehicleCollision, got {other:?}"),
        }
    }

    #[test]
    fn tray_and_mount_round_trip_through_wire() {
        let scene = deck_scene();
        let msg = crate::wire::device_msg(&scene.devices()[0]);
        let json = serde_json::to_string(&msg).unwrap();
        let back: crate::wire::DeviceMsg = serde_json::from_str(&json).unwrap();
        assert_eq!(msg, back);
        assert_eq!(
            crate::wire::device_msg(&crate::wire::device_from_msg(&back)),
            msg
        );
        // A vehicle without a deck keeps its pre-V1 JSON shape.
        let mut plain = scene.devices()[0].clone();
        if let DeviceKind::Vehicle { tray, .. } = &mut plain.kind {
            *tray = None;
        }
        let json = serde_json::to_string(&crate::wire::device_msg(&plain)).unwrap();
        assert!(!json.contains("tray"), "{json}");

        let sensor = Sensor {
            name: "loaded".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.25),
                size: Vector3::new(0.4, 0.3, 0.2),
            },
            watch: SensorWatch::AllObjects,
            mount: Some("agv".into()),
        };
        let msg = crate::wire::sensor_msg(&sensor);
        let back: crate::wire::SensorMsg =
            serde_json::from_str(&serde_json::to_string(&msg).unwrap()).unwrap();
        assert_eq!(back.mount.as_deref(), Some("agv"));
        assert_eq!(crate::wire::sensor_from_msg(&back).mount, sensor.mount);
    }

    #[test]
    fn mount_validation_names_the_problem() {
        let check = |scene: &Scene, needle: &str| {
            let err = scene
                .simulate_sequence("haul", &RolloutOptions::default())
                .unwrap_err();
            assert!(
                matches!(err, SeqError::Validation { .. }) && err.to_string().contains(needle),
                "expected `{needle}` in: {err}"
            );
        };
        let mut scene = deck_scene();
        scene.upsert_sequence(Sequence {
            name: "haul".into(),
            steps: vec![step("go", vec![goto("c")], device_done())],
        });
        let mut ghost = scene.clone();
        ghost
            .upsert_sensor(Sensor {
                name: "eye".into(),
                kind: SensorKind::Zone {
                    pose: iso(0.0, 0.0, 0.2),
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                watch: SensorWatch::AllObjects,
                mount: Some("nowhere".into()),
            })
            .unwrap();
        check(&ghost, "unknown device `nowhere`");

        let mut belt = scene.clone();
        belt.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.0, 0.5),
                zone_size: Vector3::new(1.0, 1.0, 0.2),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        belt.upsert_sensor(Sensor {
            name: "eye".into(),
            kind: SensorKind::Zone {
                pose: iso(0.0, 0.0, 0.2),
                size: Vector3::new(0.1, 0.1, 0.1),
            },
            watch: SensorWatch::AllObjects,
            mount: Some("belt".into()),
        })
        .unwrap();
        check(&belt, "not a vehicle");
    }
}

#[cfg(test)]
mod mount_tests {
    use super::tests::*;
    use super::*;
    use crate::seq::{Device, DeviceKind, Step, VehiclePath};
    use botrail_model::Geometry;
    use nalgebra::{Point3, Translation3, UnitQuaternion};
    use std::f64::consts::FRAC_PI_2;

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    fn goto(station: &str) -> Action {
        Action::Device {
            device: "amr".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn device_done() -> Condition {
        Condition::DeviceDone {
            device: "amr".into(),
        }
    }

    /// The 1-DOF arm of `sample_scene`, riding a chassis that runs 2 m
    /// along +x then 1 m along +y. Mount 0.3 m up.
    fn amr_scene() -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "chassis",
                Geometry::Box {
                    size: Vector3::new(0.4, 0.3, 0.2),
                },
                iso(0.0, 0.0, 0.1),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "amr".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(2.0, 1.0, 0.0),
                    ],
                    stations: vec![("a".into(), 0), ("c".into(), 2)],
                    ring: false,
                },
                body: vec!["chassis".into()],
                speed: 0.5,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade: None,
                },
                tray: None,
            },
        });
        scene.mount_robot(0, "amr", iso(0.0, 0.0, 0.3)).unwrap();
        scene
    }

    #[test]
    fn mounting_places_the_base_on_the_parked_vehicle() {
        let scene = amr_scene();
        // Parked at (0, 0) facing +x, so the base is straight up from it.
        let base = scene.robots()[0].base_pose();
        assert!((base.translation.vector - Vector3::new(0.0, 0.0, 0.3)).norm() < 1e-12);
        assert!(scene.robot_mount(0).is_some());
    }

    #[test]
    fn the_base_rides_the_vehicle_and_rests_when_it_stops() {
        let mut scene = amr_scene();
        scene.upsert_sequence(Sequence {
            name: "go".into(),
            steps: vec![
                step("drive", vec![goto("c")], device_done()),
                step("dwell", vec![], Condition::Elapsed { seconds: 2.0 }),
            ],
        });
        let tl = scene
            .simulate_sequence("go", &RolloutOptions::default())
            .unwrap();
        let track = &tl.robots[0];
        let base = track
            .base
            .as_ref()
            .expect("a mounted robot has a base track");
        assert!(!base.is_empty());

        // Mid-straight, mid-turn, and after arrival.
        let at = |t: f64| SequenceTimeline::base_pose(track, t).unwrap();
        assert!((at(1.0).translation.vector - Vector3::new(0.5, 0.0, 0.3)).norm() < 1e-9);
        // The turn is about the vehicle frame, which the base sits directly
        // above — so it turns on the spot and only its orientation changes.
        let mid = at(4.5);
        assert!((mid.translation.vector - Vector3::new(2.0, 0.0, 0.3)).norm() < 1e-9);
        let eighth = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), FRAC_PI_2 * 0.5);
        assert!(mid.rotation.angle_to(&eighth) < 1e-9);
        // Parked: the base rests instead of coasting on past the timeline —
        // the last travelling span must not simply be stretched to the end.
        let arrived = at(7.05);
        for t in (71..=90).map(|k| k as f64 * 0.1) {
            let p = at(t);
            assert!(
                (p.translation.vector - arrived.translation.vector).norm() < 1e-9,
                "base drifted at t = {t}: {:?}",
                p.translation.vector
            );
        }
        // ...and it agrees with the body it is bolted to, exactly.
        let chassis = tl.objects.iter().find(|o| o.name == "chassis").unwrap();
        let fk = vec![vec![
            Isometry3::identity();
            scene.robots()[0].model.links.len()
        ]];
        for k in 0..=90 {
            let t = k as f64 * 0.1;
            let body = SequenceTimeline::object_pose(chassis, &fk, t).unwrap();
            let offset = body.inverse() * at(t);
            assert!(
                (offset.translation.vector - Vector3::new(0.0, 0.0, 0.2)).norm() < 1e-9,
                "t = {t}: {:?}",
                offset.translation.vector
            );
        }
    }

    #[test]
    fn a_ramp_may_run_while_driving_but_a_plan_may_not() {
        let mut scene = amr_scene();
        joint_motion(&mut scene, "reach", 0.5);
        // A ramp alongside the drive: allowed, and the arm ends up moved.
        scene.upsert_sequence(Sequence {
            name: "stow".into(),
            steps: vec![step(
                "drive",
                vec![
                    goto("c"),
                    Action::StartRamp {
                        robot: None,
                        targets: vec![("j".into(), 0.4)],
                        duration: 1.0,
                    },
                ],
                device_done(),
            )],
        });
        let tl = scene
            .simulate_sequence("stow", &RolloutOptions::default())
            .unwrap();
        assert!((tl.robots[0].trajectory.sample(tl.duration)[0] - 0.4).abs() < 1e-9);

        // A planned motion in the same step: rejected by name.
        scene.upsert_sequence(Sequence {
            name: "bad".into(),
            steps: vec![step(
                "drive",
                vec![
                    goto("c"),
                    Action::StartMotion {
                        motion: "reach".into(),
                    },
                ],
                device_done(),
            )],
        });
        let err = scene
            .simulate_sequence("bad", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            err.to_string()
                .contains("cannot start while `amr` is driving"),
            "unexpected: {err}"
        );
    }

    #[test]
    fn mounting_rejects_a_non_vehicle_and_an_unknown_device() {
        let mut scene = amr_scene();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(0.0, 0.0, 0.5),
                zone_size: Vector3::new(1.0, 1.0, 0.2),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        let err = scene
            .mount_robot(0, "belt", Isometry3::identity())
            .unwrap_err();
        assert!(err.to_string().contains("not a vehicle"), "{err}");
        let err = scene
            .mount_robot(0, "nowhere", Isometry3::identity())
            .unwrap_err();
        assert!(err.to_string().contains("nowhere"), "{err}");
    }
}

#[cfg(test)]
mod parallel_program_tests {
    use super::tests::{joint_motion, sample_scene};
    use super::*;
    use crate::seq::{Device, DeviceKind, Step};
    use botrail_model::Geometry;
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::from_parts(Translation3::new(x, y, z), UnitQuaternion::identity())
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    fn ramp(joint: &str, value: f64, duration: f64) -> Action {
        Action::StartRamp {
            robot: None,
            targets: vec![(joint.to_string(), value)],
            duration,
        }
    }

    /// A scene with the 1-DOF arm, a box parked on a belt, and the belt.
    fn belt_scene(speed: f64) -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 0.1),
                },
                iso(0.0, 2.0, 0.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "belt".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: iso(5.0, 2.0, 0.5),
                zone_size: Vector3::new(20.0, 1.0, 1.0),
                velocity: Vector3::new(speed, 0.0, 0.0),
                running: false,
            },
        });
        scene
    }

    #[test]
    fn advance_moves_exactly_the_asked_distance() {
        // 3.2 m at 0.4 m/s is the weld line's pitch: 800 full scans. And
        // 0.123 m is deliberately *not* a multiple of v·dt — the final
        // partial tick has to make up the fraction that the old
        // start/elapsed/stop pattern always lost.
        for (distance, speed) in [(3.2, 0.4), (0.123, 0.4), (0.05, 1.3)] {
            let mut scene = belt_scene(speed);
            scene.upsert_sequence(Sequence {
                name: "index".into(),
                steps: vec![step(
                    "advance",
                    vec![Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(distance),
                    }],
                    Condition::DeviceDone {
                        device: "belt".into(),
                    },
                )],
            });
            let tl = scene
                .simulate_sequence("index", &RolloutOptions::default())
                .unwrap();
            let no_fk: Vec<Vec<Isometry3<f64>>> = scene
                .robots()
                .iter()
                .map(|r| vec![Isometry3::identity(); r.model.links.len()])
                .collect();
            let track = tl.objects.iter().find(|o| o.name == "box").unwrap();
            let pose = SequenceTimeline::object_pose(track, &no_fk, tl.duration).unwrap();
            let moved = pose.translation.vector.x;
            assert!(
                (moved - distance).abs() < 1e-12,
                "asked {distance}, moved {moved} (err {:+.3e})",
                moved - distance
            );
            // The belt lane pulses on for the advance and off at the end.
            let lane = tl.signals.iter().find(|s| s.name == "belt").unwrap();
            assert_eq!(lane.edges.first().map(|(_, v)| *v), Some(false));
            assert_eq!(lane.edges.last().map(|(_, v)| *v), Some(false));
            assert!(lane.edges.iter().any(|(_, v)| *v));
        }
    }

    #[test]
    fn advance_rejects_free_running_and_double_commands() {
        let mut scene = belt_scene(0.4);
        scene.upsert_sequence(Sequence {
            name: "bad_running".into(),
            steps: vec![step(
                "go",
                vec![
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Start,
                    },
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(1.0),
                    },
                ],
                Condition::DeviceDone {
                    device: "belt".into(),
                },
            )],
        });
        let err = scene
            .simulate_sequence("bad_running", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("free-running"), "{err}");

        let mut scene = belt_scene(0.4);
        scene.upsert_sequence(Sequence {
            name: "bad_double".into(),
            steps: vec![step(
                "go",
                vec![
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(1.0),
                    },
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(1.0),
                    },
                ],
                Condition::DeviceDone {
                    device: "belt".into(),
                },
            )],
        });
        let err = scene
            .simulate_sequence("bad_double", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("still advancing"), "{err}");

        // Statically: advance on a non-conveyor is a validation error.
        let mut scene = sample_scene();
        scene.upsert_device(Device {
            name: "lift".into(),
            kind: DeviceKind::LinearAxis {
                objects: vec![],
                axis: nalgebra::Unit::new_normalize(Vector3::z()),
                speed: 0.1,
                position: 0.0,
                range: (0.0, 1.0),
            },
        });
        scene.upsert_sequence(Sequence {
            name: "bad_kind".into(),
            steps: vec![step(
                "go",
                vec![Action::Device {
                    device: "lift".into(),
                    command: DeviceCommand::Advance(0.5),
                }],
                Condition::Immediately,
            )],
        });
        let err = scene
            .simulate_sequence("bad_kind", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("not a conveyor"), "{err}");
    }

    /// Two programs, one world: the belt program indexes while the robot
    /// program works, and the whole bake lasts as long as the *slower*
    /// of the two — which is the entire point of parallel programs.
    #[test]
    fn programs_run_concurrently_and_sync_through_signals() {
        let mut scene = belt_scene(0.4);
        scene.define_signal("welded", false);
        scene.upsert_sequence(Sequence {
            name: "station".into(),
            steps: vec![
                step("work", vec![ramp("j", 0.8, 1.0)], Condition::Done),
                step(
                    "done",
                    vec![Action::Set {
                        signal: "welded".into(),
                        value: true,
                    }],
                    Condition::Immediately,
                ),
            ],
        });
        scene.upsert_sequence(Sequence {
            name: "transfer".into(),
            steps: vec![
                step(
                    "wait_station",
                    vec![],
                    Condition::Signal {
                        name: "welded".into(),
                        value: true,
                    },
                ),
                step(
                    "index",
                    vec![Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Advance(0.2),
                    }],
                    Condition::DeviceDone {
                        device: "belt".into(),
                    },
                ),
            ],
        });
        let tl = scene
            .simulate_sequences(&["station", "transfer"], &RolloutOptions::default())
            .unwrap();
        // Station: 1.0 s of ramp. Transfer: gated on the weld, then 0.5 s
        // of indexing. Serial would be impossible to even express without
        // weaving them by hand; parallel comes out at the sum only because
        // the transfer *waits* — and the step spans prove which is which.
        let spans: std::collections::HashMap<String, (f64, f64)> = tl
            .step_spans
            .iter()
            .map(|s| (s.name.clone(), (s.start, s.end)))
            .collect();
        assert!(spans.contains_key("station/work"), "{spans:?}");
        assert!(spans.contains_key("transfer/index"), "{spans:?}");
        let work_end = spans["station/work"].1;
        let index_start = spans["transfer/index"].0;
        assert!(
            (index_start - work_end).abs() < 0.011,
            "transfer moved at {index_start}, station finished at {work_end}"
        );
        assert!(
            (tl.duration - 1.51).abs() < 0.02,
            "duration {}",
            tl.duration
        );

        // And with no dependency, the two programs overlap: total is the
        // max of the two, not the sum.
        let mut scene = belt_scene(0.4);
        scene.upsert_sequence(Sequence {
            name: "station".into(),
            steps: vec![step("work", vec![ramp("j", 0.8, 1.0)], Condition::Done)],
        });
        scene.upsert_sequence(Sequence {
            name: "transfer".into(),
            steps: vec![step(
                "index",
                vec![Action::Device {
                    device: "belt".into(),
                    command: DeviceCommand::Advance(0.2),
                }],
                Condition::DeviceDone {
                    device: "belt".into(),
                },
            )],
        });
        let tl = scene
            .simulate_sequences(&["station", "transfer"], &RolloutOptions::default())
            .unwrap();
        assert!(
            (tl.duration - 1.0).abs() < 0.02,
            "expected max(1.0, 0.5), got {}",
            tl.duration
        );
    }

    #[test]
    fn rebaking_parallel_programs_is_bit_identical() {
        let bake = || {
            let mut scene = belt_scene(0.4);
            scene.define_signal("welded", false);
            scene.upsert_sequence(Sequence {
                name: "a".into(),
                steps: vec![
                    step("work", vec![ramp("j", 0.7, 0.6)], Condition::Done),
                    step(
                        "flag",
                        vec![Action::Set {
                            signal: "welded".into(),
                            value: true,
                        }],
                        Condition::Immediately,
                    ),
                ],
            });
            scene.upsert_sequence(Sequence {
                name: "b".into(),
                steps: vec![
                    step(
                        "wait",
                        vec![],
                        Condition::Signal {
                            name: "welded".into(),
                            value: true,
                        },
                    ),
                    step(
                        "index",
                        vec![Action::Device {
                            device: "belt".into(),
                            command: DeviceCommand::Advance(0.3),
                        }],
                        Condition::DeviceDone {
                            device: "belt".into(),
                        },
                    ),
                ],
            });
            scene
                .simulate_sequences(&["a", "b"], &RolloutOptions::default())
                .unwrap()
        };
        let (one, two) = (bake(), bake());
        assert_eq!(one.duration, two.duration);
        assert_eq!(
            one.robots[0].trajectory.positions,
            two.robots[0].trajectory.positions
        );
        let x = |tl: &SequenceTimeline| {
            tl.signals
                .iter()
                .map(|s| s.edges.clone())
                .collect::<Vec<_>>()
        };
        assert_eq!(x(&one), x(&two));
    }

    #[test]
    fn ownership_is_validated_up_front() {
        // Robot commanded by two programs.
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.8);
        scene.upsert_sequence(Sequence {
            name: "a".into(),
            steps: vec![step("x", vec![ramp("j", 0.5, 0.2)], Condition::Done)],
        });
        scene.upsert_sequence(Sequence {
            name: "b".into(),
            steps: vec![step(
                "y",
                vec![Action::StartMotion {
                    motion: "go".into(),
                }],
                Condition::Done,
            )],
        });
        let err = scene
            .simulate_sequences(&["a", "b"], &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("robot `r` is commanded by both `a` and `b`"),
            "{err}"
        );

        // Signal written by two programs.
        let mut scene = sample_scene();
        scene.define_signal("flag", false);
        let writer = |name: &str| Sequence {
            name: name.into(),
            steps: vec![step(
                "w",
                vec![Action::Set {
                    signal: "flag".into(),
                    value: true,
                }],
                Condition::Immediately,
            )],
        };
        scene.upsert_sequence(writer("a"));
        scene.upsert_sequence(writer("b"));
        let err = scene
            .simulate_sequences(&["a", "b"], &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("signal `flag` is commanded by both"), "{err}");

        // Device driven by two programs.
        let mut scene = belt_scene(0.4);
        let starter = |name: &str| Sequence {
            name: name.into(),
            steps: vec![step(
                "s",
                vec![Action::Device {
                    device: "belt".into(),
                    command: DeviceCommand::Start,
                }],
                Condition::Immediately,
            )],
        };
        scene.upsert_sequence(starter("a"));
        scene.upsert_sequence(starter("b"));
        let err = scene
            .simulate_sequences(&["a", "b"], &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("device `belt` is commanded by both"), "{err}");

        // The same list twice is its own mistake.
        let mut scene = sample_scene();
        scene.upsert_sequence(Sequence {
            name: "a".into(),
            steps: vec![step("x", vec![], Condition::Immediately)],
        });
        let err = scene
            .simulate_sequences(&["a", "a"], &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("listed twice"), "{err}");
    }

    #[test]
    fn timeout_names_every_stuck_program() {
        let mut scene = sample_scene();
        scene.define_signal("never", false);
        scene.define_signal("also_never", false);
        let stuck = |name: &str, signal: &str| Sequence {
            name: name.into(),
            steps: vec![step(
                "gate",
                vec![],
                Condition::Signal {
                    name: signal.into(),
                    value: true,
                },
            )],
        };
        scene.upsert_sequence(stuck("st1", "never"));
        scene.upsert_sequence(stuck("st2", "also_never"));
        let err = scene
            .simulate_sequences(
                &["st1", "st2"],
                &RolloutOptions {
                    max_duration: 0.5,
                    ..Default::default()
                },
            )
            .unwrap_err()
            .to_string();
        assert!(
            err.contains("st1/gate") && err.contains("st2/gate"),
            "{err}"
        );

        // A finished program is not on the list.
        let mut scene = sample_scene();
        scene.define_signal("never", false);
        scene.upsert_sequence(Sequence {
            name: "quick".into(),
            steps: vec![step("done", vec![], Condition::Immediately)],
        });
        scene.upsert_sequence(stuck("st2", "never"));
        let err = scene
            .simulate_sequences(
                &["quick", "st2"],
                &RolloutOptions {
                    max_duration: 0.5,
                    ..Default::default()
                },
            )
            .unwrap_err()
            .to_string();
        assert!(err.contains("st2/gate") && !err.contains("quick"), "{err}");
    }
}

#[cfg(test)]
mod gait_tests {
    use super::*;
    use crate::seq::{
        Device, DeviceKind, FootContact, GaitPattern, GaitSpec, LegSpec, Step, VehiclePath,
    };
    use botrail_model::Geometry;
    use nalgebra::Point3;
    use std::f64::consts::FRAC_PI_2;
    use std::sync::Arc;

    const QUAD: &str = include_str!("../../../examples/assets/quad_test.urdf");
    const LEGS: [&str; 4] = ["FL", "FR", "RL", "RR"];
    const FOOT_R: f64 = 0.02;

    /// Foot depth below the body at the stance: thigh 0.7, calf -1.4 on two
    /// 0.2 m segments fold to a foot straight under the hip.
    fn stance_depth() -> f64 {
        0.4 * 0.7f64.cos()
    }

    fn quad_gait() -> GaitSpec {
        let leg = |n: &str| LegSpec {
            name: n.into(),
            foot: format!("{n}_foot"),
            contact: FootContact::Point,
        };
        let mut stance = Vec::new();
        for n in LEGS {
            stance.push((format!("{n}_hip_joint"), 0.0));
            stance.push((format!("{n}_thigh_joint"), 0.7));
            stance.push((format!("{n}_calf_joint"), -1.4));
        }
        GaitSpec {
            max_step: None,
            body_link: None,
            legs: LEGS.iter().map(|n| leg(n)).collect(),
            pattern: GaitPattern::Trot,
            period: 0.5,
            lift: 0.05,
            stance,
            max_stride: 0.5,
            foot_radius: FOOT_R,
            arm_swing: Vec::new(),
            bob: 0.0,
            lateral: 0.0,
        }
    }

    /// An L: 2 m along +x, a 90° pivot, 1 m along +y. At 0.5 m/s and 90°/s
    /// the drive takes 4 + 1 + 2 = 7 s.
    fn dog(speed: f64, turn_speed: f64, allow_reverse: bool, start: &str) -> Device {
        Device {
            name: "dog".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(2.0, 1.0, 0.0),
                    ],
                    stations: vec![("a".into(), 0), ("c".into(), 2)],
                    ring: false,
                },
                body: Vec::new(),
                speed,
                turn_speed,
                start: start.into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse,
                    max_grade: None,
                },
                tray: None,
            },
        }
    }

    fn quad_scene(device: Device, gait: Option<GaitSpec>) -> Scene {
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(QUAD).unwrap(),
        ));
        scene.upsert_device(device);
        scene.mount_robot_with(0, "dog", None, gait).unwrap();
        scene
    }

    fn dog_scene() -> Scene {
        quad_scene(dog(0.5, FRAC_PI_2, false, "a"), Some(quad_gait()))
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    fn goto(station: &str) -> Action {
        Action::Device {
            device: "dog".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn device_done() -> Condition {
        Condition::DeviceDone {
            device: "dog".into(),
        }
    }

    /// Drive to `station`, then stand for `dwell` seconds.
    fn patrol(scene: &mut Scene, station: &str, dwell: f64, dt: f64) -> SequenceTimeline {
        scene.upsert_sequence(Sequence {
            name: "patrol".into(),
            steps: vec![
                step("drive", vec![goto(station)], device_done()),
                step("stand", vec![], Condition::Elapsed { seconds: dwell }),
            ],
        });
        let options = RolloutOptions {
            dt,
            ..RolloutOptions::default()
        };
        scene.simulate_sequence("patrol", &options).unwrap()
    }

    fn feet_world(scene: &Scene, tl: &SequenceTimeline, t: f64) -> Vec<Point3<f64>> {
        let track = &tl.robots[0];
        let q = track.trajectory.sample(t);
        let base = SequenceTimeline::base_pose(track, t).unwrap();
        let model = &scene.robots()[0].model;
        let poses = botrail_kin::forward_kinematics_with_base(model, &q, &base).unwrap();
        LEGS.iter()
            .map(|n| {
                let link = model.link_index(&format!("{n}_foot")).unwrap();
                Point3::from(poses[link].translation.vector)
            })
            .collect()
    }

    fn leg_q(scene: &Scene, q: &[f64], leg: &str) -> [f64; 3] {
        let model = &scene.robots()[0].model;
        let qi = |j: &str| {
            model.joints[model.joint_index(&format!("{leg}_{j}")).unwrap()]
                .q_index
                .unwrap()
        };
        [
            q[qi("hip_joint")],
            q[qi("thigh_joint")],
            q[qi("calf_joint")],
        ]
    }

    /// The dog's L flattened onto a ramp: 3 m along +x rising 0.3 m.
    fn ramp_dog() -> Device {
        let mut device = dog(0.5, FRAC_PI_2, false, "a");
        if let DeviceKind::Vehicle { path, drive, .. } = &mut device.kind {
            path.waypoints = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.3)];
            path.stations = vec![("a".into(), 0), ("b".into(), 1)];
            *drive = crate::seq::Drive::Differential {
                allow_reverse: false,
                max_grade: Some(0.2),
            };
        }
        device
    }

    #[test]
    fn a_gait_on_an_aerial_vehicle_is_refused() {
        let mut device = dog(0.5, FRAC_PI_2, false, "a");
        if let DeviceKind::Vehicle { drive, .. } = &mut device.kind {
            *drive = crate::seq::Drive::Aerial {
                climb_speed: 0.5,
                descent_speed: 0.5,
                yaw: crate::seq::AerialYaw::Course,
            };
        }
        let mut scene = quad_scene(device, Some(quad_gait()));
        scene.upsert_sequence(Sequence {
            name: "patrol".into(),
            steps: vec![step("drive", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("patrol", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("aerial vehicle"), "{err}");
    }

    #[test]
    fn the_walk_climbs_the_ramp_with_the_frame() {
        // The gait is untouched by the slope: the body profile carries the
        // z, the feet land on the (climbing) vehicle plane, and the stance
        // at the top stands a leg's depth above the raised floor.
        let mut scene = quad_scene(ramp_dog(), Some(quad_gait()));
        let tl = patrol(&mut scene, "b", 2.0, 0.01);
        let base = SequenceTimeline::base_pose(&tl.robots[0], tl.duration).unwrap();
        assert!(
            (base.translation.z - (0.3 + stance_depth() + FOOT_R)).abs() < 1e-6,
            "base z = {}",
            base.translation.z
        );
        for foot in feet_world(&scene, &tl, tl.duration) {
            assert!(
                (foot.z - (0.3 + FOOT_R)).abs() < 1e-4,
                "foot z = {}",
                foot.z
            );
        }
        // Mid-drive the whole machine is halfway up the ramp. The body
        // rides where its feet are rather than the guide line exactly, so
        // on a smooth slope it sits within a few millimetres of it (on
        // stairs that difference is the whole riser).
        let drive_end = (3.0f64.powi(2) + 0.3f64.powi(2)).sqrt() / 0.5;
        let base = SequenceTimeline::base_pose(&tl.robots[0], drive_end / 2.0).unwrap();
        assert!(
            (base.translation.z - (0.15 + stance_depth() + FOOT_R)).abs() < 0.01,
            "mid base z = {}",
            base.translation.z
        );
    }

    /// A straight flight: the guide line climbs to `top` over 3 m while
    /// two walkable treads (at `top/2` and `top`) stand under it. With
    /// `footprint`, an aisle-check body box rides along — the treads poke
    /// into it, which is exactly what the walkable pass must excuse.
    fn stepped_scene(top: f64, max_step: Option<f64>, footprint: bool) -> Scene {
        let mut device = dog(0.4, FRAC_PI_2, false, "a");
        if let DeviceKind::Vehicle {
            path, drive, body, ..
        } = &mut device.kind
        {
            path.waypoints = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, top)];
            path.stations = vec![("a".into(), 0), ("b".into(), 1)];
            *drive = crate::seq::Drive::Differential {
                allow_reverse: false,
                max_grade: Some(0.2),
            };
            if footprint {
                *body = vec!["footprint".into()];
            }
        }
        let mut gait = quad_gait();
        gait.max_step = max_step;
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(QUAD).unwrap(),
        ));
        if footprint {
            scene
                .add_obstacle(
                    "footprint",
                    Geometry::Box {
                        size: Vector3::new(0.9, 0.6, 0.5),
                    },
                    Isometry3::translation(0.0, 0.0, 0.25),
                )
                .unwrap();
        }
        for (name, x0, x1, top_i) in [("tread1", 0.8, 1.9, top / 2.0), ("tread2", 1.9, 3.4, top)] {
            scene
                .add_obstacle(
                    name,
                    Geometry::Box {
                        size: Vector3::new(x1 - x0, 1.2, 0.05),
                    },
                    Isometry3::translation((x0 + x1) / 2.0, 0.0, top_i - 0.025),
                )
                .unwrap();
            scene.set_obstacle_walkable(name, true).unwrap();
        }
        scene.upsert_device(device);
        scene.mount_robot_with(0, "dog", None, Some(gait)).unwrap();
        scene
    }

    #[test]
    fn what_a_walking_machine_carries_rides_its_body_not_its_route() {
        // A tote on the dog's back, up a flight. The body tilts onto the
        // pitch and rides up the steps while the route stays on the guide
        // line between the waypoints — a load pinned to the route would
        // hold air on the way up and reach inside the machine coming down.
        let top = 0.16;
        let mut scene = stepped_scene(top, None, false);
        let mut device = scene
            .devices()
            .iter()
            .find(|d| d.name == "dog")
            .expect("the dog's vehicle")
            .clone();
        if let DeviceKind::Vehicle { tray, .. } = &mut device.kind {
            *tray = Some((
                Isometry3::translation(0.0, 0.0, 0.5),
                Vector3::new(0.4, 0.4, 0.3),
            ));
        }
        scene.upsert_device(device);
        scene
            .add_obstacle(
                "tote",
                Geometry::Box {
                    size: Vector3::new(0.2, 0.2, 0.1),
                },
                Isometry3::translation(0.0, 0.0, 0.5),
            )
            .unwrap();
        let tl = patrol(&mut scene, "b", 2.0, 0.01);

        let track = tl
            .objects
            .iter()
            .find(|o| o.name == "tote")
            .expect("the tote is tracked");
        assert!(
            track
                .spans
                .iter()
                .any(|s| matches!(s, TrackSpan::Follow { .. })),
            "the load follows the body, not the route: {:?}",
            track.spans
        );

        let mut probe = scene.clone();
        let mut held: Option<Isometry3<f64>> = None;
        let mut pinned: Option<Isometry3<f64>> = None;
        let (mut apart, mut tilt) = (0.0_f64, 0.0_f64);
        for i in 0..=40 {
            let t = tl.duration * f64::from(i) / 40.0;
            let base = SequenceTimeline::base_pose(&tl.robots[0], t).expect("a base track");
            let route =
                SequenceTimeline::span_pose(tl.robots[0].base.as_deref().expect("base"), &[], t)
                    .expect("the rigid ride");
            probe.set_robot_base_pose_for(0, base);
            probe
                .set_joint_positions_for(0, tl.robots[0].trajectory.sample(t))
                .unwrap();
            let fk = vec![probe.link_poses_for(0)];
            let pose = SequenceTimeline::object_pose(track, &fk, t).expect("a tracked pose");

            // Rigid with the body: the offset it was picked up at is the
            // offset it keeps, tilt and ride included.
            let offset = base.inverse() * pose;
            match &held {
                None => held = Some(offset),
                Some(first) => {
                    let slid = (first.inverse() * offset).translation.vector.norm();
                    assert!(
                        slid < 1e-9,
                        "the load slid {slid:.5} m on the body at t = {t:.2}"
                    );
                }
            }
            // …and it is *not* rigid with the route: that offset has to
            // move, by the ride and the tilt the body adds on the flight.
            let on_route = route.inverse() * pose;
            match &pinned {
                None => pinned = Some(on_route),
                Some(first) => {
                    apart = apart.max((first.inverse() * on_route).translation.vector.norm());
                    tilt = tilt.max((first.inverse() * on_route).rotation.angle());
                }
            }
        }
        assert!(
            apart > top / 4.0,
            "the load never rose off the route it would have been pinned to ({apart:.4} m)"
        );
        assert!(
            tilt > 0.05,
            "the load never tilted with the body ({tilt:.4} rad)"
        );
    }

    #[test]
    fn the_feet_snap_onto_the_treads_and_the_walker_may_touch_them() {
        let top = 0.16;
        let mut scene = stepped_scene(top, None, true);
        let tl = patrol(&mut scene, "b", 2.0, 0.01);
        // Every foothold over a tread stands exactly on it (top + the ball
        // radius) — not on the slope the guide line interpolates.
        let (mut on1, mut on2) = (0, 0);
        for f in &tl.robots[0].footfalls {
            if f.position.x > 0.82 && f.position.x < 1.88 {
                assert!(
                    (f.position.z - (top / 2.0 + FOOT_R)).abs() < 1e-9,
                    "foothold at x = {:.3} has z = {:.4}",
                    f.position.x,
                    f.position.z
                );
                on1 += 1;
            }
            if f.position.x > 1.92 && f.position.x < 3.38 {
                assert!(
                    (f.position.z - (top + FOOT_R)).abs() < 1e-9,
                    "foothold at x = {:.3} has z = {:.4}",
                    f.position.x,
                    f.position.z
                );
                on2 += 1;
            }
        }
        assert!(on1 > 0 && on2 > 0, "on1 = {on1}, on2 = {on2}");
        // Parked on the upper tread: the feet stand on it, the body a
        // stance above it — and the run completing at all means the aisle
        // and rider checks excused the treads under the walking machine.
        for foot in feet_world(&scene, &tl, tl.duration) {
            assert!(
                (foot.z - (top + FOOT_R)).abs() < 1e-4,
                "parked foot z = {}",
                foot.z
            );
        }
        let base = SequenceTimeline::base_pose(&tl.robots[0], tl.duration).unwrap();
        assert!((base.translation.z - (top + stance_depth() + FOOT_R)).abs() < 1e-4);
    }

    #[test]
    fn a_step_over_the_declared_ability_is_refused_by_name() {
        let mut scene = stepped_scene(0.16, Some(0.05), false);
        scene.upsert_sequence(Sequence {
            name: "patrol".into(),
            steps: vec![step("drive", vec![goto("b")], device_done())],
        });
        let err = scene
            .simulate_sequence("patrol", &RolloutOptions::default())
            .unwrap_err();
        assert!(matches!(err, SeqError::StepHeight { .. }), "{err}");
        assert!(err.to_string().contains("max_step"), "{err}");
    }

    #[test]
    fn a_foothold_on_a_tread_edge_is_named() {
        // Walk the flat flight once to learn where a mid-walk foothold
        // lands, then put a ledge under it whose edge cuts the margin.
        let mut device = dog(0.4, FRAC_PI_2, false, "a");
        if let DeviceKind::Vehicle { path, .. } = &mut device.kind {
            path.waypoints = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)];
            path.stations = vec![("a".into(), 0), ("b".into(), 1)];
        }
        let mut scene = quad_scene(device.clone(), Some(quad_gait()));
        let tl = patrol(&mut scene, "b", 1.0, 0.01);
        let f = tl.robots[0].footfalls[4].position;

        let mut scene = quad_scene(device, Some(quad_gait()));
        scene
            .add_obstacle(
                "ledge",
                Geometry::Box {
                    size: Vector3::new(0.3, 0.4, 0.04),
                },
                // The foothold sits FOOT_R/4 inside the +x edge.
                Isometry3::translation(f.x + 0.15 - FOOT_R * 0.25, f.y, 0.0),
            )
            .unwrap();
        scene.set_obstacle_walkable("ledge", true).unwrap();
        scene.upsert_sequence(Sequence {
            name: "patrol".into(),
            steps: vec![step("drive", vec![goto("b")], device_done())],
        });
        let err = scene
            .simulate_sequence("patrol", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            matches!(&err, SeqError::FootOverhang { obstacle, .. } if obstacle == "ledge"),
            "{err}"
        );
        assert!(err.to_string().contains("edge of `ledge`"), "{err}");
    }

    #[test]
    fn mounting_with_a_gait_stands_the_robot_on_the_vehicle_plane() {
        let scene = dog_scene();
        let base = scene.robots()[0].base_pose();
        assert!(
            (base.translation.z - (stance_depth() + FOOT_R)).abs() < 1e-9,
            "base z = {}",
            base.translation.z
        );
        assert!((base.translation.x).abs() < 1e-12 && base.translation.y.abs() < 1e-12);
        let q = scene.robots()[0].joint_positions();
        for leg in LEGS {
            let [hip, thigh, calf] = leg_q(&scene, q, leg);
            assert!(
                (hip, thigh, calf) == (0.0, 0.7, -1.4),
                "{leg}: {hip} {thigh} {calf}"
            );
        }
        let poses = scene.link_poses();
        for leg in LEGS {
            let link = scene.robot().link_index(&format!("{leg}_foot")).unwrap();
            assert!(
                (poses[link].translation.z - FOOT_R).abs() < 1e-9,
                "{leg} foot z = {}",
                poses[link].translation.z
            );
        }
    }

    #[test]
    fn a_gait_is_checked_against_the_model_by_name() {
        let bad = |edit: &dyn Fn(&mut GaitSpec)| {
            let mut spec = quad_gait();
            edit(&mut spec);
            let mut scene = Scene::new(Arc::new(
                botrail_model::RobotModel::from_urdf_str(QUAD).unwrap(),
            ));
            scene.upsert_device(dog(0.5, FRAC_PI_2, false, "a"));
            scene
                .mount_robot_with(0, "dog", None, Some(spec))
                .unwrap_err()
                .to_string()
        };
        let e = bad(&|s| s.legs[0].foot = "FL_paw".into());
        assert!(e.contains("unknown foot link `FL_paw`"), "{e}");
        let e = bad(&|s| s.stance.retain(|(j, _)| j != "RR_calf_joint"));
        assert!(e.contains("RR_calf_joint"), "{e}");
        let e = bad(&|s| s.stance.push(("RL_calf_joint".into(), 1.0)));
        assert!(e.contains("outside its limits"), "{e}");
        let e = bad(&|s| s.legs.pop().map(|_| ()).unwrap_or(()));
        assert!(e.contains("trot pattern is for 4 legs"), "{e}");
        // A stance that does not stand level: one knee folded further.
        let e = bad(&|s| {
            for (j, v) in &mut s.stance {
                if j == "RR_calf_joint" {
                    *v = -2.0;
                }
            }
        });
        assert!(e.contains("does not stand level"), "{e}");
    }

    #[test]
    fn planted_feet_never_move() {
        let mut scene = dog_scene();
        let tl = patrol(&mut scene, "c", 2.0, 0.01);
        let track = &tl.robots[0];
        assert!(
            track.footfalls.len() >= 4 * 13,
            "only {} footfalls over a 7 s trot",
            track.footfalls.len()
        );
        let times = &track.trajectory.times;
        let start = feet_world(&scene, &tl, 0.0);
        for (i, leg) in LEGS.iter().enumerate() {
            // The planted intervals: from t = 0 to the first lift, then
            // from each landing to the next lift.
            let steps: Vec<&crate::gait::Footfall> =
                track.footfalls.iter().filter(|f| f.leg == *leg).collect();
            let mut anchors = vec![(0.0, steps[0].lift, start[i])];
            for pair in steps.windows(2) {
                anchors.push((pair[0].land, pair[1].lift, pair[0].position));
            }
            let last = steps.last().unwrap();
            anchors.push((last.land, tl.duration, last.position));
            let mut checked = 0;
            for (from, to, anchor) in anchors {
                for &t in times
                    .iter()
                    .filter(|&&t| t >= from + 1e-9 && t <= to - 1e-9)
                {
                    let foot = feet_world(&scene, &tl, t)[i];
                    let slip = (foot - anchor).norm();
                    assert!(slip < 1e-6, "{leg} slipped {slip:.2e} m at t = {t}");
                    checked += 1;
                }
            }
            assert!(
                checked > 100,
                "{leg}: only {checked} planted samples checked"
            );
        }
        // Landings happen where the plan said, and swings clear the floor.
        for f in &track.footfalls {
            let i = LEGS.iter().position(|l| *l == f.leg).unwrap();
            let at_land = feet_world(&scene, &tl, f.land)[i];
            assert!((at_land - f.position).norm() < 1e-6);
            let mid = feet_world(&scene, &tl, 0.5 * (f.lift + f.land))[i];
            assert!(mid.z > FOOT_R + 0.03, "swing apex {} too low", mid.z);
        }
    }

    #[test]
    fn the_walk_is_deterministic_to_the_bit() {
        let a = patrol(&mut dog_scene(), "c", 1.0, 0.01);
        let b = patrol(&mut dog_scene(), "c", 1.0, 0.01);
        assert_eq!(
            a.robots[0].trajectory.positions,
            b.robots[0].trajectory.positions
        );
        assert_eq!(a.robots[0].footfalls, b.robots[0].footfalls);
    }

    #[test]
    fn footfalls_do_not_depend_on_the_scan_period() {
        let coarse = patrol(&mut dog_scene(), "c", 1.0, 0.01);
        let fine = patrol(&mut dog_scene(), "c", 1.0, 0.005);
        assert_eq!(coarse.robots[0].footfalls, fine.robots[0].footfalls);
        assert!(!coarse.robots[0].footfalls.is_empty());
    }

    #[test]
    fn the_legs_settle_into_the_stance_after_arrival() {
        let mut scene = dog_scene();
        let tl = patrol(&mut scene, "c", 2.0, 0.01);
        let track = &tl.robots[0];
        let done = track.footfalls.iter().map(|f| f.land).fold(0.0, f64::max);
        // Arrival is at 7 s; the settle is at most one more cycle and a
        // swing per leg.
        assert!(done > 7.0 && done <= 7.0 + 1.5 * 0.5, "settled at {done}");
        for t in [done + 0.2, tl.duration] {
            let q = track.trajectory.sample(t);
            for leg in LEGS {
                let [hip, thigh, calf] = leg_q(&scene, &q, leg);
                assert!(
                    hip.abs() < 1e-9 && (thigh - 0.7).abs() < 1e-9 && (calf + 1.4).abs() < 1e-9,
                    "{leg} at t = {t}: {hip} {thigh} {calf}"
                );
            }
        }
        // Parked at (2, 1) facing +y: every foot stands under its hip.
        let feet = feet_world(&scene, &tl, tl.duration);
        let expect = |x: f64, y: f64| Point3::new(2.0 - y, 1.0 + x, FOOT_R);
        for (i, (x, y)) in [(0.19, 0.15), (0.19, -0.15), (-0.19, 0.15), (-0.19, -0.15)]
            .iter()
            .enumerate()
        {
            assert!(
                (feet[i] - expect(*x, *y)).norm() < 1e-9,
                "{}: {:?} vs {:?}",
                LEGS[i],
                feet[i],
                expect(*x, *y)
            );
        }
    }

    #[test]
    fn a_stride_the_legs_cannot_take_is_refused_by_name() {
        // Declared: the stride check at simulate.
        let mut scene = quad_scene(dog(2.0, FRAC_PI_2, false, "a"), Some(quad_gait()));
        scene.upsert_sequence(Sequence {
            name: "go".into(),
            steps: vec![step("drive", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("go", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("max_stride"), "{err}");

        // Undeclared: a generous max_stride lets the solve discover it.
        let mut spec = quad_gait();
        spec.max_stride = 5.0;
        let mut scene = quad_scene(dog(2.0, FRAC_PI_2, false, "a"), Some(spec));
        scene.upsert_sequence(Sequence {
            name: "go".into(),
            steps: vec![step("drive", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("go", &RolloutOptions::default())
            .unwrap_err();
        assert!(
            matches!(err, SeqError::GaitReach { .. }),
            "unexpected: {err}"
        );
        assert!(
            err.to_string().contains("cannot reach its footfall"),
            "{err}"
        );
    }

    #[test]
    fn a_leg_ramp_is_refused_mid_walk_while_the_head_may_nod() {
        let mut scene = dog_scene();
        let ramp = |joint: &str, value: f64| Action::StartRamp {
            robot: None,
            targets: vec![(joint.into(), value)],
            duration: 1.0,
        };
        scene.upsert_sequence(Sequence {
            name: "nod".into(),
            steps: vec![
                step("drive", vec![goto("c"), ramp("neck", 0.5)], device_done()),
                step("stand", vec![], Condition::Elapsed { seconds: 1.0 }),
            ],
        });
        let tl = scene
            .simulate_sequence("nod", &RolloutOptions::default())
            .unwrap();
        let neck = scene.robot().joints[scene.robot().joint_index("neck").unwrap()]
            .q_index
            .unwrap();
        let track = &tl.robots[0];
        assert!((track.trajectory.sample(0.5)[neck] - 0.25).abs() < 1e-6);
        assert!((track.trajectory.sample(tl.duration)[neck] - 0.5).abs() < 1e-9);
        assert!(!track.footfalls.is_empty());

        scene.upsert_sequence(Sequence {
            name: "kick".into(),
            steps: vec![step(
                "drive",
                vec![goto("c"), ramp("FL_calf_joint", -1.0)],
                device_done(),
            )],
        });
        let err = scene
            .simulate_sequence("kick", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("driven by the gait"), "{err}");

        // Standing, a leg may be ramped (a crouch); the walk then starts
        // from wherever the legs are.
        scene.upsert_sequence(Sequence {
            name: "crouch".into(),
            steps: vec![
                step("crouch", vec![ramp("FL_calf_joint", -1.0)], Condition::Done),
                step("drive", vec![goto("c")], device_done()),
                step("stand", vec![], Condition::Elapsed { seconds: 1.0 }),
            ],
        });
        let tl = scene
            .simulate_sequence("crouch", &RolloutOptions::default())
            .unwrap();
        let calf = scene.robot().joints[scene.robot().joint_index("FL_calf_joint").unwrap()]
            .q_index
            .unwrap();
        assert!((tl.robots[0].trajectory.sample(1.0)[calf] + 1.0).abs() < 1e-9);
        assert!((tl.robots[0].trajectory.sample(tl.duration)[calf] + 1.4).abs() < 1e-9);
    }

    #[test]
    fn a_pivot_steps_the_feet_around_the_vehicle_origin() {
        let mut scene = dog_scene();
        let tl = patrol(&mut scene, "c", 1.0, 0.01);
        let radius = (0.19f64 * 0.19 + 0.15 * 0.15).sqrt();
        let (mut on_straight, mut on_pivot) = (0, 0);
        for f in &tl.robots[0].footfalls {
            let mid = f.land + 0.125;
            if mid < 4.0 {
                // Straight along +x: the lateral offset never changes.
                assert!((f.position.y.abs() - 0.15).abs() < 1e-9, "{:?}", f);
                on_straight += 1;
            } else if mid < 5.0 {
                let r = ((f.position.x - 2.0).powi(2) + f.position.y.powi(2)).sqrt();
                assert!((r - radius).abs() < 1e-9, "{:?} r = {r}", f);
                on_pivot += 1;
            }
            assert!((f.position.z - FOOT_R).abs() < 1e-9);
        }
        assert!(
            on_straight >= 4 * 7 && on_pivot >= 4,
            "{on_straight} / {on_pivot}"
        );
    }

    #[test]
    fn backing_up_walks_the_legs_backwards() {
        // From `c` (facing +y) the first leg back to (2, 0) is a reverse:
        // the body keeps facing +y and the feet step along -y.
        let mut scene = quad_scene(dog(0.5, FRAC_PI_2, true, "c"), Some(quad_gait()));
        let tl = patrol(&mut scene, "a", 1.0, 0.01);
        let fl: Vec<&crate::gait::Footfall> = tl.robots[0]
            .footfalls
            .iter()
            .filter(|f| f.leg == "FL" && f.land + 0.125 < 2.0 && f.lift > 0.6)
            .collect();
        assert!(fl.len() >= 2, "{}", fl.len());
        for pair in fl.windows(2) {
            let (a, b) = (pair[0].position, pair[1].position);
            assert!((a.y - b.y - 0.25).abs() < 1e-9, "{a:?} -> {b:?}");
            assert!((a.x - b.x).abs() < 1e-9);
        }
        let base = SequenceTimeline::base_pose(&tl.robots[0], 1.0).unwrap();
        let heading = base.rotation * Vector3::x();
        assert!((heading - Vector3::y()).norm() < 1e-9);
    }

    #[test]
    fn a_footprint_body_around_the_legs_is_not_a_collision() {
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(QUAD).unwrap(),
        ));
        // Long enough to cover the head, which pokes 0.37 m ahead: the
        // footprint is what the aisle check should answer with.
        scene
            .add_obstacle(
                "footprint",
                Geometry::Box {
                    size: Vector3::new(0.8, 0.4, 0.4),
                },
                Isometry3::translation(0.05, 0.0, 0.2),
            )
            .unwrap();
        let mut device = dog(0.5, FRAC_PI_2, false, "a");
        if let DeviceKind::Vehicle { body, .. } = &mut device.kind {
            body.push("footprint".into());
        }
        scene.upsert_device(device);
        scene
            .mount_robot_with(0, "dog", None, Some(quad_gait()))
            .unwrap();
        // Standing inside its own footprint is the arrangement.
        assert!(scene.check_collisions().is_empty());
        // ...and the footprint still drives the aisle check: a post in the
        // way of the first leg fails the walk by name.
        scene
            .add_obstacle(
                "post",
                Geometry::Box {
                    size: Vector3::new(0.1, 0.1, 1.0),
                },
                Isometry3::translation(1.0, 0.0, 0.5),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "go".into(),
            steps: vec![step("drive", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("go", &RolloutOptions::default())
            .unwrap_err();
        assert!(matches!(err, SeqError::VehicleCollision { .. }), "{err}");
    }

    #[test]
    fn a_leg_that_meets_the_environment_mid_walk_is_named() {
        let mut scene = dog_scene();
        // A post beside the body (which is 0.25 m wide), in the lane the
        // left feet (at y = 0.15) swing through: nothing but a leg meets it.
        scene
            .add_obstacle(
                "post",
                Geometry::Box {
                    size: Vector3::new(0.06, 0.06, 0.4),
                },
                Isometry3::translation(1.0, 0.17, 0.2),
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "go".into(),
            steps: vec![step("drive", vec![goto("c")], device_done())],
        });
        let err = scene
            .simulate_sequence("go", &RolloutOptions::default())
            .unwrap_err();
        match &err {
            SeqError::RiderCollision {
                vehicle,
                robot,
                part,
                obstacle,
                ..
            } => {
                assert_eq!(vehicle, "dog");
                assert_eq!(robot, "quad_test");
                assert_eq!(obstacle, "post");
                assert!(part.starts_with("FL") || part.starts_with("RL"), "{part}");
            }
            other => panic!("unexpected: {other}"),
        }
        assert!(err.to_string().contains("riding `dog`"), "{err}");
        // Out of the legs' way, the walk goes through.
        scene
            .set_obstacle_pose("post", Isometry3::translation(1.0, 0.6, 0.2))
            .unwrap();
        scene
            .simulate_sequence("go", &RolloutOptions::default())
            .unwrap();
    }

    #[test]
    fn a_gait_mount_round_trips_through_project_and_python() {
        let mut scene = dog_scene();
        let tl = patrol(&mut scene, "c", 1.0, 0.01);
        let project = scene.to_project();
        let json = serde_json::to_string(&project).unwrap();
        let back: crate::project::ProjectFile = serde_json::from_str(&json).unwrap();
        let mount = back.robots[0].mount.as_ref().expect("the mount is saved");
        assert_eq!(mount.device, "dog");
        let gait = mount.gait.as_ref().expect("the gait is saved");
        assert_eq!(gait.legs.len(), 4);
        assert_eq!(gait.pattern, crate::project::GaitPatternMsg::Trot);

        // Rebuilt from the file, the cell stands the same and walks the
        // same steps.
        let again = Scene::from_project(&back).unwrap();
        let mount = again.robot_mount(0).expect("mounted again");
        assert_eq!(mount.device, "dog");
        assert!(mount.gait.is_some());
        assert!(
            (again.robots()[0].base_pose().translation.z
                - scene.robots()[0].base_pose().translation.z)
                .abs()
                < 1e-12
        );
        let tl2 = again
            .simulate_sequence("patrol", &RolloutOptions::default())
            .unwrap();
        assert_eq!(tl.duration, tl2.duration);
        assert_eq!(tl.robots[0].footfalls, tl2.robots[0].footfalls);

        // ...and the generated script re-authors the mount and its gait.
        let code = crate::project::generate_python(&project);
        assert!(code.contains("scene.mount_robot(\"dog\""), "{code}");
        assert!(
            code.contains("gait=bt.Gait(legs={\"FL\": (\"FL_foot\", \"point\")"),
            "{code}"
        );
        assert!(code.contains("pattern=\"trot\""), "{code}");
        assert!(code.contains("\"FL_calf_joint\": -1.4"), "{code}");
    }

    #[test]
    fn a_walking_robot_has_no_script_to_export() {
        let mut scene = dog_scene();
        scene.upsert_sequence(Sequence {
            name: "nod".into(),
            steps: vec![
                step(
                    "nod",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("neck".into(), 0.3)],
                        duration: 0.5,
                    }],
                    Condition::Done,
                ),
                step("drive", vec![goto("c")], device_done()),
            ],
        });
        let tl = scene
            .simulate_sequence("nod", &RolloutOptions::default())
            .unwrap();
        let io = crate::script::SequenceIo::from_ports(Default::default(), Default::default());
        let err = crate::script::sequence_program(
            &scene,
            &tl,
            None,
            &io,
            &botrail_export::ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("gait"), "{err}");
    }

    #[test]
    fn a_walked_vehicle_is_one_bom_line() {
        let scene = dog_scene();
        let categories: Vec<String> = scene
            .bom()
            .rows
            .iter()
            .map(|r| r.category.clone())
            .collect();
        assert!(
            !categories.iter().any(|c| c.starts_with("vehicle")),
            "{categories:?}"
        );
        assert!(!categories.is_empty());
        // Pinned to a part of its own, the vehicle is listed after all.
        let mut pinned = dog_scene();
        pinned
            .set_part(
                "dog",
                None,
                crate::part::Part {
                    model: Some("GO2".into()),
                    ..Default::default()
                },
            )
            .unwrap();
        assert!(pinned
            .bom()
            .rows
            .iter()
            .any(|r| r.category.starts_with("vehicle")));
    }

    #[test]
    fn a_bodiless_vehicles_rider_is_the_machine_itself() {
        // A UAV: the robot rigid-mounted on a vehicle with no body of its
        // own IS the machine — one BOM line, the robot's. An AMR carrying
        // an arm stays two (the chassis is a product of its own).
        let mut scene = quad_scene(dog(0.5, FRAC_PI_2, false, "a"), None);
        scene.mount_robot_with(0, "dog", None, None).unwrap();
        let categories: Vec<String> = scene
            .bom()
            .rows
            .iter()
            .map(|r| r.category.clone())
            .collect();
        assert!(
            !categories.iter().any(|c| c.starts_with("vehicle")),
            "{categories:?}"
        );

        let mut amr = quad_scene(dog(0.5, FRAC_PI_2, false, "a"), None);
        amr.add_obstacle(
            "chassis",
            Geometry::Box {
                size: Vector3::new(0.6, 0.4, 0.3),
            },
            Isometry3::translation(0.0, 0.0, 0.15),
        )
        .unwrap();
        if let Some(mut device) = amr.devices().iter().find(|d| d.name == "dog").cloned() {
            if let DeviceKind::Vehicle { body, .. } = &mut device.kind {
                *body = vec!["chassis".into()];
            }
            amr.upsert_device(device);
        }
        amr.mount_robot_with(
            0,
            "dog",
            Some(Isometry3::translation(0.0, 0.0, 0.305)),
            None,
        )
        .unwrap();
        assert!(amr
            .bom()
            .rows
            .iter()
            .any(|r| r.category.starts_with("vehicle")));
    }

    #[test]
    fn walking_costs_no_cycle_time() {
        let walked = patrol(&mut dog_scene(), "c", 1.0, 0.01);
        let mut carried = quad_scene(dog(0.5, FRAC_PI_2, false, "a"), None);
        let carried = patrol(&mut carried, "c", 1.0, 0.01);
        assert_eq!(walked.duration, carried.duration);
        assert!(carried.robots[0].footfalls.is_empty());
    }
}

#[cfg(test)]
mod biped_tests {
    use super::*;
    use crate::seq::{
        Device, DeviceKind, FootContact, GaitPattern, GaitSpec, LegSpec, Step, VehiclePath,
    };
    use botrail_model::Geometry;
    use nalgebra::{Point3, UnitQuaternion};
    use std::f64::consts::FRAC_PI_2;
    use std::sync::Arc;

    const BIPED: &str = include_str!("../../../examples/assets/biped_test.urdf");
    const SOLE: f64 = 0.05;

    /// The same model with the ankle roll welded: 5-DOF legs.
    fn biped_5dof() -> String {
        BIPED
            .replace(
                r#"<joint name="L_ankle_roll_joint" type="revolute">"#,
                r#"<joint name="L_ankle_roll_joint" type="fixed">"#,
            )
            .replace(
                r#"<joint name="R_ankle_roll_joint" type="revolute">"#,
                r#"<joint name="R_ankle_roll_joint" type="fixed">"#,
            )
    }

    fn biped_gait(ankle_roll: bool) -> GaitSpec {
        let mut stance = Vec::new();
        for side in ["L", "R"] {
            stance.push((format!("{side}_hip_yaw_joint"), 0.0));
            stance.push((format!("{side}_hip_roll_joint"), 0.0));
            stance.push((format!("{side}_hip_pitch_joint"), -0.4));
            stance.push((format!("{side}_knee_joint"), 0.8));
            stance.push((format!("{side}_ankle_pitch_joint"), -0.4));
            if ankle_roll {
                stance.push((format!("{side}_ankle_roll_joint"), 0.0));
            }
        }
        GaitSpec {
            max_step: None,
            body_link: None,
            legs: ["L", "R"]
                .iter()
                .map(|s| LegSpec {
                    name: s.to_string(),
                    foot: format!("{s}_foot"),
                    contact: FootContact::Sole { yaw_free: false },
                })
                .collect(),
            pattern: GaitPattern::Biped,
            period: 0.8,
            lift: 0.05,
            stance,
            max_stride: 0.5,
            foot_radius: SOLE,
            arm_swing: vec![
                ("L_shoulder_pitch_joint".into(), 0.3),
                ("R_shoulder_pitch_joint".into(), -0.3),
            ],
            bob: 0.0,
            lateral: 0.0,
        }
    }

    /// An L: 2 m along +x (6.67 s at 0.3 m/s), a 1 s pivot, 1 m along +y.
    fn walker() -> Device {
        Device {
            name: "walker".into(),
            kind: DeviceKind::Vehicle {
                path: VehiclePath {
                    waypoints: vec![
                        Point3::new(0.0, 0.0, 0.0),
                        Point3::new(2.0, 0.0, 0.0),
                        Point3::new(2.0, 1.0, 0.0),
                    ],
                    stations: vec![("a".into(), 0), ("c".into(), 2)],
                    ring: false,
                },
                body: Vec::new(),
                speed: 0.3,
                turn_speed: FRAC_PI_2,
                start: "a".into(),
                drive: crate::seq::Drive::Differential {
                    allow_reverse: false,
                    max_grade: None,
                },
                tray: None,
            },
        }
    }

    fn biped_scene(urdf: &str, gait: GaitSpec) -> Scene {
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(urdf).unwrap(),
        ));
        scene.upsert_device(walker());
        scene
            .mount_robot_with(0, "walker", None, Some(gait))
            .unwrap();
        scene
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    fn goto(station: &str) -> Action {
        Action::Device {
            device: "walker".into(),
            command: DeviceCommand::Goto {
                station: station.into(),
            },
        }
    }

    fn device_done() -> Condition {
        Condition::DeviceDone {
            device: "walker".into(),
        }
    }

    fn walk(scene: &mut Scene, dt: f64) -> SequenceTimeline {
        scene.upsert_sequence(Sequence {
            name: "walk".into(),
            steps: vec![
                step("go", vec![goto("c")], device_done()),
                step("stand", vec![], Condition::Elapsed { seconds: 2.0 }),
            ],
        });
        let options = RolloutOptions {
            dt,
            ..RolloutOptions::default()
        };
        scene.simulate_sequence("walk", &options).unwrap()
    }

    fn qi(scene: &Scene, joint: &str) -> usize {
        let model = &scene.robots()[0].model;
        model.joints[model.joint_index(joint).unwrap()]
            .q_index
            .unwrap()
    }

    /// World pose of a foot link at `t`, off the baked timeline.
    fn foot_pose(scene: &Scene, tl: &SequenceTimeline, leg: &str, t: f64) -> Isometry3<f64> {
        let track = &tl.robots[0];
        let q = track.trajectory.sample(t);
        let base = SequenceTimeline::base_pose(track, t).unwrap();
        let model = &scene.robots()[0].model;
        let poses = botrail_kin::forward_kinematics_with_base(model, &q, &base).unwrap();
        poses[model.link_index(&format!("{leg}_foot")).unwrap()]
    }

    /// The foot link's heading about +Z, relative to its stance rotation.
    fn foot_yaw(scene: &Scene, pose: &Isometry3<f64>, leg: &str) -> f64 {
        let model = &scene.robots()[0].model;
        let stance =
            botrail_kin::forward_kinematics(model, scene.robots()[0].joint_positions()).unwrap();
        let nominal = stance[model.link_index(&format!("{leg}_foot")).unwrap()].rotation;
        crate::gait::yaw_of(&Isometry3::from_parts(
            nalgebra::Translation3::identity(),
            pose.rotation * nominal.inverse(),
        ))
    }

    fn tilt(pose: &Isometry3<f64>) -> f64 {
        (pose.rotation * Vector3::z()).z.clamp(-1.0, 1.0).acos()
    }

    #[test]
    fn sole_feet_land_flat_and_pointed_where_the_body_heads() {
        let mut scene = biped_scene(BIPED, biped_gait(true));
        let tl = walk(&mut scene, 0.01);
        let track = &tl.robots[0];
        assert!(track.footfalls.len() >= 2 * 13, "{}", track.footfalls.len());
        let times = &track.trajectory.times;
        let mut checked = 0;
        for leg in ["L", "R"] {
            let steps: Vec<_> = track.footfalls.iter().filter(|f| f.leg == leg).collect();
            for pair in steps.windows(2) {
                let (f, next) = (pair[0], pair[1]);
                for &t in times
                    .iter()
                    .filter(|&&t| t >= f.land + 1e-9 && t <= next.lift - 1e-9)
                {
                    let pose = foot_pose(&scene, &tl, leg, t);
                    assert!(
                        (pose.translation.vector - f.position.coords).norm() < 1e-6,
                        "{leg} slipped at {t}"
                    );
                    assert!(
                        tilt(&pose) < 1e-4,
                        "{leg} tilted {:.2e} at {t}",
                        tilt(&pose)
                    );
                    let yaw = foot_yaw(&scene, &pose, leg);
                    assert!(
                        (yaw - f.yaw).abs() < 1e-4,
                        "{leg} points {yaw} instead of {} at {t}",
                        f.yaw
                    );
                    checked += 1;
                }
            }
            // Through the pivot (6.67–7.67 s) each landing turns a little
            // further; by the end the feet point where the body does.
            let turning: Vec<f64> = steps
                .iter()
                .filter(|f| f.land + 0.24 > 6.67 && f.land + 0.24 < 7.67)
                .map(|f| f.yaw)
                .collect();
            assert!(!turning.is_empty(), "{leg}: {turning:?}");
            assert!(
                turning.windows(2).all(|w| w[1] > w[0]),
                "{leg}: {turning:?}"
            );
            assert!((steps.last().unwrap().yaw - FRAC_PI_2).abs() < 1e-9);
        }
        assert!(checked > 200, "only {checked} planted samples");
        // Mid-swing the sole is still level, and off the floor.
        for f in &track.footfalls {
            let pose = foot_pose(&scene, &tl, &f.leg, 0.5 * (f.lift + f.land));
            assert!(tilt(&pose) < 1e-4);
            assert!(pose.translation.z > SOLE + 0.03);
        }
    }

    #[test]
    fn five_dof_legs_keep_the_sole_level_with_the_heading_free() {
        let urdf = biped_5dof();
        let mut scene = biped_scene(&urdf, biped_gait(false));
        assert_eq!(scene.robots()[0].model.dof(), 14);
        let tl = walk(&mut scene, 0.01);
        let track = &tl.robots[0];
        let mut checked = 0;
        for f in &track.footfalls {
            for k in 1..4 {
                let t = f.land + 0.1 * k as f64;
                if t >= tl.duration {
                    continue;
                }
                let pose = foot_pose(&scene, &tl, &f.leg, t);
                assert!(
                    tilt(&pose) < 1e-4,
                    "{} tilted {:.2e} at {t}",
                    f.leg,
                    tilt(&pose)
                );
                checked += 1;
            }
        }
        assert!(checked > 50);
    }

    #[test]
    fn arms_swing_in_step_unless_the_hands_are_full() {
        let mut scene = biped_scene(BIPED, biped_gait(true));
        let (l, r) = (
            qi(&scene, "L_shoulder_pitch_joint"),
            qi(&scene, "R_shoulder_pitch_joint"),
        );
        let tl = walk(&mut scene, 0.01);
        let track = &tl.robots[0];
        let mut peak = 0.0f64;
        for (t, q) in track
            .trajectory
            .times
            .iter()
            .zip(&track.trajectory.positions)
        {
            assert!((q[l] + q[r]).abs() < 1e-9, "arms out of step at {t}");
            if *t < 11.0 {
                peak = peak.max(q[l].abs());
            }
        }
        assert!(peak > 0.29, "peak swing {peak}");
        let rest = track.trajectory.sample(tl.duration);
        assert!(rest[l].abs() < 1e-12 && rest[r].abs() < 1e-12);

        // Hands full: a box in the left hand rides still, and so do the arms.
        let mut scene = biped_scene(BIPED, biped_gait(true));
        let hand = scene.link_poses()[scene.robot().link_index("L_hand").unwrap()];
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.08, 0.08, 0.08),
                },
                hand * Isometry3::translation(0.0, 0.0, -0.1),
            )
            .unwrap();
        scene
            .attach_obstacle_to(0, "box", Some("L_hand"), None)
            .unwrap();
        let tl = walk(&mut scene, 0.01);
        let track = &tl.robots[0];
        assert!(!track.footfalls.is_empty());
        for q in &track.trajectory.positions {
            assert!(q[l].abs() < 1e-12 && q[r].abs() < 1e-12);
        }
        // ...and the box went along for the walk.
        let carried = tl.objects.iter().find(|o| o.name == "box").unwrap();
        let poses = vec![botrail_kin::forward_kinematics_with_base(
            &scene.robots()[0].model,
            &track.trajectory.sample(tl.duration),
            &SequenceTimeline::base_pose(track, tl.duration).unwrap(),
        )
        .unwrap()];
        let end = SequenceTimeline::object_pose(carried, &poses, tl.duration).unwrap();
        assert!(
            end.translation.x > 1.5,
            "box ends at {:?}",
            end.translation.vector
        );
    }

    #[test]
    fn a_ramped_arm_is_left_alone_and_a_swung_one_cannot_be_ramped() {
        let mut scene = biped_scene(BIPED, biped_gait(true));
        let (l, r) = (
            qi(&scene, "L_shoulder_pitch_joint"),
            qi(&scene, "R_shoulder_pitch_joint"),
        );
        let ramp = |joint: &str, value: f64, duration: f64| Action::StartRamp {
            robot: None,
            targets: vec![(joint.into(), value)],
            duration,
        };
        // A raise still in flight at dispatch finishes as ramped; the other
        // arm swings.
        scene.upsert_sequence(Sequence {
            name: "raise".into(),
            steps: vec![
                step(
                    "raise",
                    vec![ramp("L_shoulder_pitch_joint", -1.0, 3.0)],
                    Condition::Immediately,
                ),
                step("go", vec![goto("c")], device_done()),
                step("stand", vec![], Condition::Elapsed { seconds: 1.0 }),
            ],
        });
        let tl = scene
            .simulate_sequence("raise", &RolloutOptions::default())
            .unwrap();
        let track = &tl.robots[0];
        assert!((track.trajectory.sample(3.0)[l] + 1.0).abs() < 1e-6);
        assert!((track.trajectory.sample(tl.duration)[l] + 1.0).abs() < 1e-9);
        let peak = track
            .trajectory
            .positions
            .iter()
            .map(|q| q[r].abs())
            .fold(0.0, f64::max);
        assert!(peak > 0.29, "right arm did not swing: {peak}");

        // A ramp on a swinging arm mid-walk is refused by name.
        scene.upsert_sequence(Sequence {
            name: "wave".into(),
            steps: vec![step(
                "go",
                vec![goto("c"), ramp("R_shoulder_pitch_joint", 0.5, 1.0)],
                device_done(),
            )],
        });
        let err = scene
            .simulate_sequence("wave", &RolloutOptions::default())
            .unwrap_err()
            .to_string();
        assert!(err.contains("driven by the gait"), "{err}");
    }

    #[test]
    fn the_body_sways_and_the_feet_stay_put() {
        let mut spec = biped_gait(true);
        spec.bob = 0.02;
        spec.lateral = 0.015;
        let mut scene = biped_scene(BIPED, spec);
        let height = scene.robots()[0].base_pose().translation.z;
        let tl = walk(&mut scene, 0.01);
        let track = &tl.robots[0];
        assert_eq!(track.sway.len(), 1);
        let sway = &track.sway[0];
        assert!(
            sway.t0 == 0.0 && sway.done > 11.0 && sway.done < 12.5,
            "{sway:?}"
        );

        let (mut lo, mut hi, mut left, mut right) = (f64::MAX, f64::MIN, f64::MAX, f64::MIN);
        for k in 100..600 {
            let t = k as f64 * 0.01;
            let base = SequenceTimeline::base_pose(track, t).unwrap();
            let rigid = SequenceTimeline::span_pose(track.base.as_ref().unwrap(), &[], t).unwrap();
            let offset = rigid.inverse() * base;
            lo = lo.min(offset.translation.z);
            hi = hi.max(offset.translation.z);
            left = left.min(offset.translation.y);
            right = right.max(offset.translation.y);
            assert!(offset.translation.x.abs() < 1e-12);
            assert!(offset.rotation.angle() < 1e-12);
        }
        assert!(hi > 0.018 && lo < -0.018, "bob {lo}..{hi}");
        assert!(right > 0.013 && left < -0.013, "lean {left}..{right}");

        // The feet do not follow the body.
        let times = &track.trajectory.times;
        let mut checked = 0;
        for leg in ["L", "R"] {
            let steps: Vec<_> = track.footfalls.iter().filter(|f| f.leg == leg).collect();
            for pair in steps.windows(2) {
                let (f, next) = (pair[0], pair[1]);
                for &t in times
                    .iter()
                    .filter(|&&t| t >= f.land + 1e-9 && t <= next.lift - 1e-9)
                {
                    let pose = foot_pose(&scene, &tl, leg, t);
                    assert!((pose.translation.vector - f.position.coords).norm() < 1e-6);
                    assert!(tilt(&pose) < 1e-4);
                    checked += 1;
                }
            }
        }
        assert!(checked > 200);

        // Settled: the base is back on its rigid ride at the stance height.
        for t in [sway.done + 0.1, tl.duration] {
            let base = SequenceTimeline::base_pose(track, t).unwrap();
            assert!((base.translation.z - height).abs() < 1e-12, "t = {t}");
        }
        // Closed form: the sway does not depend on the scan period.
        let fine = walk(
            &mut biped_scene(BIPED, {
                let mut s = biped_gait(true);
                s.bob = 0.02;
                s.lateral = 0.015;
                s
            }),
            0.005,
        );
        assert_eq!(fine.robots[0].sway, track.sway);
        assert_eq!(fine.robots[0].footfalls, track.footfalls);
    }

    #[test]
    fn a_sole_that_does_not_stand_level_is_refused() {
        let mut spec = biped_gait(true);
        for (joint, value) in &mut spec.stance {
            if joint == "L_ankle_pitch_joint" {
                *value = -0.2;
            }
        }
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(BIPED).unwrap(),
        ));
        scene.upsert_device(walker());
        let err = scene
            .mount_robot_with(0, "walker", None, Some(spec))
            .unwrap_err()
            .to_string();
        assert!(err.contains("tilted"), "{err}");

        // A point-footed leg cannot be asked for a sole.
        let quad = include_str!("../../../examples/assets/quad_test.urdf");
        let mut scene = Scene::new(Arc::new(
            botrail_model::RobotModel::from_urdf_str(quad).unwrap(),
        ));
        scene.upsert_device(walker());
        let mut stance = Vec::new();
        for n in ["FL", "FR", "RL", "RR"] {
            stance.push((format!("{n}_hip_joint"), 0.0));
            stance.push((format!("{n}_thigh_joint"), 0.7));
            stance.push((format!("{n}_calf_joint"), -1.4));
        }
        let spec = GaitSpec {
            max_step: None,
            body_link: None,
            legs: ["FL", "FR", "RL", "RR"]
                .iter()
                .map(|n| LegSpec {
                    name: n.to_string(),
                    foot: format!("{n}_foot"),
                    contact: FootContact::Sole { yaw_free: true },
                })
                .collect(),
            pattern: GaitPattern::Trot,
            period: 0.5,
            lift: 0.05,
            stance,
            max_stride: 0.5,
            foot_radius: 0.02,
            arm_swing: Vec::new(),
            bob: 0.0,
            lateral: 0.0,
        };
        let err = scene
            .mount_robot_with(0, "walker", None, Some(spec))
            .unwrap_err()
            .to_string();
        assert!(err.contains("needs 5 or 6 DOF"), "{err}");
        let _ = UnitQuaternion::<f64>::identity();
        let _: Point3<f64> = Point3::origin();
    }
}
