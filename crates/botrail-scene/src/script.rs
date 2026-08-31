//! Sequence → vendor program lowering: the same authored steps that drove
//! the simulation become a robot controller program with real I/O.
//!
//! The lowering treats the robot as the cell master (the common small-cell
//! wiring): sensor contacts and partner handshakes arrive on digital
//! inputs, coils the sequence writes (conveyor run, vacuum, weld fire)
//! leave on digital outputs, and the caller supplies that name → port
//! wiring in [`SequenceIo`]. Moves come from the rollout's recorded sparse
//! plans ([`crate::rollout::PlannedMove`]) — the exact paths the timeline
//! executed, not a re-plan.
//!
//! Scripts are synchronous where the scan loop is concurrent: a move
//! blocks until it finishes, so a transition that would have fired
//! mid-move in simulation fires after it in the script. The lowering is
//! therefore *conservative* — every waypoint and I/O write happens in the
//! same order, but a cycle can run slower than the simulated takt; steps
//! that lean on that concurrency get a warning.
//!
//! Branching steps lower to a wait-any plus an `if/elif` chain over the
//! arm guards, so the script carries *every* arm — including the ones
//! this bake skipped. A skipped arm was never simulated, so it must stay
//! motion-free (its ramps are synthesized from the branch-entry
//! configuration; a planned motion there is refused). Edges lower to the
//! classic wait-idle-then-active pair. What cannot be expressed at all
//! (timers or edges as branch guards, conveyor tracking, positioning
//! device commands) is refused or downgraded to a comment, never
//! silently dropped.

use std::collections::HashMap;

use botrail_export::{Command, DigitalTest, Program, ProgramOptions};

use crate::motion::SegmentKind;
use crate::rollout::{BranchTaken, PlannedMove, SequenceTimeline};
use crate::seq::{walk_actions, Action, Condition, DeviceCommand, Sequence, Step};
use crate::Scene;

/// Name → digital port wiring for a sequence export.
///
/// Keys are the names the sequence already uses: signal names (sensors and
/// internal flags), device names, and — for `robot_done` waits — robot
/// instance names. `inputs` feed level waits, `outputs` feed coil writes.
/// Each port carries its wire polarity: an inverted input is tested
/// against the opposite level, an inverted output written the opposite
/// way (the sequence's meaning is untouched — `invert` is how the wire is
/// built). Projected from the scene's I/O map
/// ([`crate::iomap::sequence_io`]) or handed over as plain dicts.
#[derive(Debug, Clone, Default)]
pub struct SequenceIo {
    pub inputs: HashMap<String, IoPort>,
    pub outputs: HashMap<String, IoPort>,
    /// The robot the program runs on. Set by the lowering itself; a
    /// `robot_done` wait on this robot is the idle test a blocking
    /// controller has already passed, so it lowers to nothing.
    pub self_robot: Option<String>,
}

pub use crate::iomap::IoPort;

impl SequenceIo {
    /// Plain `name → port` maps, no inversion (the historical dict form).
    pub fn from_ports(inputs: HashMap<String, u32>, outputs: HashMap<String, u32>) -> Self {
        let lift = |m: HashMap<String, u32>| {
            m.into_iter()
                .map(|(k, port)| {
                    (
                        k,
                        IoPort {
                            port,
                            invert: false,
                        },
                    )
                })
                .collect()
        };
        SequenceIo {
            inputs: lift(inputs),
            outputs: lift(outputs),
            self_robot: None,
        }
    }
}

/// A lowered program plus everything the lowering had to approximate.
#[derive(Debug, Clone)]
pub struct SequenceProgram {
    pub program: Program,
    /// Human-readable notes on approximations (unmapped device commands,
    /// concurrency narrowed to sequential execution). Empty means the
    /// script is a faithful reproduction of the sequence's order *and*
    /// timing semantics.
    pub warnings: Vec<String>,
}

/// Lowers one sequence of a rolled timeline to a vendor-neutral
/// [`Program`].
///
/// `scene` must be the snapshot the timeline was baked against (the Python
/// timeline carries it); `sequence` may be `None` when the timeline was
/// rolled from a single program. Errors are authoring-level messages:
/// unmapped signals, inexpressible steps, or a timeline that no longer
/// matches the sequence.
pub fn sequence_program(
    scene: &Scene,
    timeline: &SequenceTimeline,
    sequence: Option<&str>,
    io: &SequenceIo,
    options: &ProgramOptions,
) -> Result<SequenceProgram, String> {
    let label = timeline.scenario.as_deref().unwrap_or("baseline");
    merged_sequence_program(scene, &[(label, timeline)], sequence, io, options)
}

/// Lowers one sequence from a *set* of rolled timelines — a scenario
/// sweep — into one program that carries every branch arm.
///
/// The first timeline is the primary: it supplies the shared path and the
/// arms it took. An arm the primary skipped is *donated* by the first
/// other timeline that took it — its planned moves splice in, which is
/// sound only if that donor reached the branch at the same configuration
/// as the primary (the shared path is the same authored steps, and joint
/// moves are absolute, so this holds unless a scenario changed the path
/// *before* the branch — checked, and refused with the offending pair
/// named). Every source walks the whole tree in parallel, consuming its
/// own planned-move and branch-decision cursors, so a mismatch anywhere
/// (an edited sequence, a stale timeline) surfaces as a leftover or a
/// misaligned decision rather than a silently wrong program.
pub fn merged_sequence_program(
    scene: &Scene,
    timelines: &[(&str, &SequenceTimeline)],
    sequence: Option<&str>,
    io: &SequenceIo,
    options: &ProgramOptions,
) -> Result<SequenceProgram, String> {
    let Some((_, first)) = timelines.first() else {
        return Err("no timelines to export".to_string());
    };
    for (label, timeline) in timelines {
        if timeline.sequences != first.sequences {
            return Err(format!(
                "timeline `{label}` was rolled from a different sequence set (`{}` vs \
                 `{}`) — merge the runs of one scenario sweep",
                first.sequences.join(" + "),
                timeline.sequences.join(" + "),
            ));
        }
    }
    let name = resolve_sequence(first, sequence)?;
    let seq = scene.sequence(name).ok_or_else(|| {
        format!("the timeline's scene snapshot holds no sequence `{name}` (internal mismatch)")
    })?;
    let robot = driven_robot(scene, seq)?;
    let robot_name = scene.robots()[robot].name.clone();
    // A walking robot's joints are its legs, driven by the gait the vehicle
    // dispatched; no controller takes that as a program.
    if scene.robots()[robot]
        .mount
        .as_ref()
        .is_some_and(|m| m.gait.is_some())
    {
        return Err(format!(
            "robot `{robot_name}` walks its vehicle: its legs are a gait, not a program — \
             there is no controller script to export for it (the cycle belongs to the PLC \
             side: export_plcopen)"
        ));
    }
    let io = SequenceIo {
        self_robot: Some(robot_name.clone()),
        ..io.clone()
    };
    let io = &io;

    let model = &scene.robots()[robot].model;
    let limits = crate::motion::traj_limits(model);
    if options.speed_scale <= 0.0 || options.tcp_speed <= 0.0 || options.tcp_accel <= 0.0 {
        return Err("speed scale, tcp speed, and tcp acceleration must be positive".to_string());
    }
    let fold_min = |xs: &[f64]| xs.iter().copied().fold(f64::INFINITY, f64::min);

    let mut sources = Vec::new();
    for (label, timeline) in timelines {
        let track = timeline
            .robots
            .iter()
            .find(|t| t.name == robot_name)
            .ok_or_else(|| format!("timeline `{label}` has no track for robot `{robot_name}`"))?;
        sources.push(Source {
            scenario: label.to_string(),
            planned: track
                .planned
                .iter()
                .filter(|p| p.sequence == name)
                .collect(),
            planned_next: 0,
            decisions: timeline
                .branches
                .iter()
                .filter(|b| b.sequence == name)
                .collect(),
            decisions_next: 0,
            cur_q: track.trajectory.sample(0.0),
        });
    }

    let mut lowering = Lowering {
        io,
        model,
        blend_radius: options.blend_radius,
        joint_velocity: fold_min(&limits.velocity) * options.speed_scale,
        joint_acceleration: fold_min(&limits.acceleration) * options.speed_scale,
        tcp_velocity: options.tcp_speed * options.speed_scale,
        tcp_acceleration: options.tcp_accel * options.speed_scale,
        sources,
        warnings: Vec::new(),
        step_no: 0,
        selects_seen: 0,
        pending_rejoin: None,
    };

    let mut commands = Vec::new();
    let mut local_q = lowering.sources[0].cur_q.clone();
    if options.move_to_start {
        if let Some(start) = lowering.sources[0]
            .planned
            .first()
            .and_then(|p| p.segments.first())
            .and_then(|s| s.waypoints.first())
        {
            commands.push(Command::MoveJoint {
                q: start.clone(),
                velocity: lowering.joint_velocity,
                acceleration: lowering.joint_acceleration,
                blend: 0.0,
            });
            local_q = start.clone();
        }
    }

    let active: Vec<usize> = (0..lowering.sources.len()).collect();
    lowering.lower_steps(&seq.steps, &active, Some(0), &mut local_q, &mut commands)?;

    for source in &lowering.sources {
        if source.planned_next < source.planned.len() {
            return Err(format!(
                "timeline `{}` holds more planned moves for `{name}` than the \
                 sequence fires (was the sequence edited after the rollout?)",
                source.scenario
            ));
        }
        if source.decisions_next < source.decisions.len() {
            return Err(format!(
                "timeline `{}` holds more branch decisions for `{name}` than the \
                 sequence has branching steps (was the sequence edited after the \
                 rollout?)",
                source.scenario
            ));
        }
    }
    if !commands
        .iter()
        .any(|c| !matches!(c, Command::Comment { .. }))
    {
        return Err(format!("sequence `{name}` lowers to no commands"));
    }

    Ok(SequenceProgram {
        program: Program {
            name: name.to_string(),
            joint_names: model
                .actuated_joint_names()
                .iter()
                .map(|n| n.to_string())
                .collect(),
            commands,
        },
        warnings: lowering.warnings,
    })
}

/// One timeline's cursors: its planned moves and branch decisions, both
/// consumed in the same order the rollout fired them, plus the
/// configuration its consumed path has reached (what the splice check
/// compares).
struct Source<'a> {
    scenario: String,
    planned: Vec<&'a PlannedMove>,
    planned_next: usize,
    decisions: Vec<&'a BranchTaken>,
    decisions_next: usize,
    cur_q: Vec<f64>,
}

impl<'a> Source<'a> {
    fn next_planned(
        &mut self,
        motion: Option<&str>,
        label: &str,
    ) -> Result<&'a PlannedMove, String> {
        let record = self
            .planned
            .get(self.planned_next)
            .copied()
            .filter(|p| p.motion.as_deref() == motion && !p.segments.is_empty())
            .ok_or_else(|| {
                format!(
                    "timeline `{}` does not match the sequence at {label} (was it \
                     edited after the rollout?) — re-simulate and export again",
                    self.scenario
                )
            })?;
        self.planned_next += 1;
        if let Some(q) = record.segments.last().and_then(|s| s.waypoints.last()) {
            self.cur_q = q.clone();
        }
        Ok(record)
    }

    fn next_decision(&mut self, select: usize, step: &str, label: &str) -> Result<usize, String> {
        let decision = self
            .decisions
            .get(self.decisions_next)
            .copied()
            .ok_or_else(|| {
                format!(
                    "timeline `{}` records no decision for branching {label} (was \
                     the sequence edited after the rollout?) — re-simulate and \
                     export again",
                    self.scenario
                )
            })?;
        if decision.select != select || decision.step != step {
            return Err(format!(
                "timeline `{}` decisions do not line up with the sequence ({label} \
                 vs recorded `{}`) — re-simulate and export again",
                self.scenario, decision.step
            ));
        }
        self.decisions_next += 1;
        Ok(decision.arm)
    }
}

/// Rolling lowering state: the per-source cursors, the joint bounds, and
/// the bookkeeping that keeps a merged program honest (the select
/// ordinal counter — the same pre-order walk the rollout flattened by —
/// and the divergent-rejoin flag a following straight-line move trips).
struct Lowering<'a> {
    io: &'a SequenceIo,
    model: &'a botrail_model::RobotModel,
    blend_radius: f64,
    joint_velocity: f64,
    joint_acceleration: f64,
    tcp_velocity: f64,
    tcp_acceleration: f64,
    /// `[0]` is the primary; the rest are donors, in run order.
    sources: Vec<Source<'a>>,
    warnings: Vec<String>,
    /// Running step number for script comments (emission order, skipped
    /// arms included — the script contains them all).
    step_no: usize,
    /// Pre-order select counter, matching `BranchTaken::select`.
    selects_seen: usize,
    /// Set when a select's arms rejoin at different configurations:
    /// `(branch label, max deviation)`. The next emitted move consumes
    /// it — and warns if that move is a straight line, whose path then
    /// depends on which arm ran.
    pending_rejoin: Option<(String, f64)>,
}

impl<'a> Lowering<'a> {
    /// Lowers a step list. `active` are the sources whose decision path
    /// runs through this list (they consume their cursors here);
    /// `emitting` is the one whose records become commands — `None` on a
    /// path no timeline took, where only authored data (ramps, I/O) can
    /// be emitted, synthesized against `local_q`.
    fn lower_steps(
        &mut self,
        steps: &[Step],
        active: &[usize],
        emitting: Option<usize>,
        local_q: &mut Vec<f64>,
        out: &mut Vec<Command>,
    ) -> Result<(), String> {
        debug_assert!(emitting.is_none_or(|e| active.contains(&e)));
        for step in steps {
            self.step_no += 1;
            let label = format!("step {} `{}`", self.step_no, step.name);
            if !step.select.is_empty() {
                out.push(Command::Comment {
                    text: format!("step {}: {} (branch)", self.step_no, step.name),
                });
                self.lower_select(step, active, emitting, local_q, &label, out)?;
                continue;
            }
            out.push(Command::Comment {
                text: format!("step {}: {}", self.step_no, step.name),
            });
            let mut started_move = false;
            for action in &step.actions {
                self.lower_action(
                    action,
                    active,
                    emitting,
                    local_q,
                    &label,
                    &mut started_move,
                    out,
                )?;
            }
            lower_condition(
                &step.transition,
                self.io,
                &label,
                started_move,
                out,
                &mut self.warnings,
            )?;
        }
        Ok(())
    }

    /// A branching step: wait-any over the arm guards, then an `if/elif`
    /// chain carrying *every* arm — the controller decides at runtime
    /// what each bake decided by state. An arm the emitting source
    /// skipped is donated by a source that took it (splice-checked), or
    /// lowered source-less when nobody did.
    fn lower_select(
        &mut self,
        step: &Step,
        active: &[usize],
        emitting: Option<usize>,
        local_q: &mut Vec<f64>,
        label: &str,
        out: &mut Vec<Command>,
    ) -> Result<(), String> {
        let ordinal = self.selects_seen;
        self.selects_seen += 1;
        let mut arm_of: Vec<(usize, usize)> = Vec::new();
        for &i in active {
            let arm = self.sources[i].next_decision(ordinal, &step.name, label)?;
            arm_of.push((i, arm));
        }
        let entry_q = local_q.clone();
        let mut arms = Vec::new();
        let mut arm_exits: Vec<Vec<f64>> = Vec::new();
        for (j, arm) in step.select.iter().enumerate() {
            let arm_label = format!("{label} arm {}", j + 1);
            let test = digital_test(&arm.condition, self.io, &arm_label)?.simplified();
            let arm_active: Vec<usize> = arm_of
                .iter()
                .filter(|(_, a)| *a == j)
                .map(|(i, _)| *i)
                .collect();
            let arm_emitting = match emitting {
                Some(e) if arm_active.contains(&e) => Some(e),
                Some(e) => match arm_active.first().copied() {
                    Some(donor) => {
                        // Splicing another bake's arm is sound only if it
                        // reached this branch where the primary did.
                        let deviation = self.sources[donor]
                            .cur_q
                            .iter()
                            .zip(&entry_q)
                            .map(|(a, b)| (a - b).abs())
                            .fold(0.0, f64::max);
                        if deviation > 1e-6 {
                            return Err(format!(
                                "scenario `{}` reaches {label} at a different \
                                 configuration than `{}` (Δ {:.4} rad) — the shared \
                                 path diverged before the branch, so its arm cannot \
                                 be spliced into one program",
                                self.sources[donor].scenario, self.sources[e].scenario, deviation,
                            ));
                        }
                        Some(donor)
                    }
                    None => None,
                },
                None => None,
            };
            let mut arm_q = entry_q.clone();
            let mut body = Vec::new();
            self.lower_steps(&arm.steps, &arm_active, arm_emitting, &mut arm_q, &mut body)?;
            arm_exits.push(arm_q);
            arms.push(botrail_export::SelectArm { test, body });
        }
        if let Some(e) = emitting {
            let taken = arm_of
                .iter()
                .find(|(i, _)| *i == e)
                .map(|(_, a)| *a)
                .expect("the emitting source is active");
            local_q.clone_from(&arm_exits[taken]);
        }
        // Arms rejoining at different configurations are fine for joint
        // moves (absolute targets) — flag it, and let the next move
        // decide whether that matters.
        let mut deviation = 0.0f64;
        for a in &arm_exits {
            for b in &arm_exits {
                for (x, y) in a.iter().zip(b) {
                    deviation = deviation.max((x - y).abs());
                }
            }
        }
        if deviation > 1e-6 {
            self.pending_rejoin = Some((label.to_string(), deviation));
        }
        out.push(Command::Select { arms });
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn lower_action(
        &mut self,
        action: &Action,
        active: &[usize],
        emitting: Option<usize>,
        local_q: &mut Vec<f64>,
        label: &str,
        started_move: &mut bool,
        out: &mut Vec<Command>,
    ) -> Result<(), String> {
        match action {
            Action::StartMotion { motion }
            | Action::StartToolpath {
                toolpath: motion, ..
            } => {
                *started_move = true;
                let mut record = None;
                for &i in active {
                    let consumed = self.sources[i].next_planned(Some(motion), label)?;
                    if Some(i) == emitting {
                        record = Some(consumed);
                    }
                }
                let Some(record) = record else {
                    return Err(format!(
                        "{label}: motion `{motion}` lies on an arm no exported \
                         rollout took, so it was never planned — add a scenario \
                         that takes this arm to the sweep, or keep skipped arms \
                         motion-free (I/O, ramps)"
                    ));
                };
                for segment in &record.segments {
                    match segment.kind {
                        SegmentKind::Joint => {
                            let last = segment.waypoints.len().saturating_sub(1);
                            for (i, q) in segment.waypoints.iter().enumerate().skip(1) {
                                self.note_move(false);
                                out.push(Command::MoveJoint {
                                    q: q.clone(),
                                    velocity: self.joint_velocity,
                                    acceleration: self.joint_acceleration,
                                    blend: if i < last { self.blend_radius } else { 0.0 },
                                });
                            }
                        }
                        SegmentKind::CartesianLine => {
                            if segment.waypoints.len() > 1 {
                                self.note_move(true);
                                out.push(Command::MoveLinear {
                                    q: segment.waypoints.last().expect("len > 1").clone(),
                                    velocity: segment.tcp_speed.unwrap_or(self.tcp_velocity),
                                    acceleration: self.tcp_acceleration,
                                    // Chained toolpath samples blend into
                                    // each other; the record's final linear
                                    // move is zeroed below.
                                    blend: self.blend_radius,
                                });
                            }
                        }
                    }
                }
                // A linear chain must come to rest at its end, not blend
                // past it into whatever the next step commands.
                if let Some(Command::MoveLinear { blend, .. }) = out.last_mut() {
                    *blend = 0.0;
                }
                if let Some(q) = record.segments.last().and_then(|s| s.waypoints.last()) {
                    local_q.clone_from(q);
                }
            }
            Action::StartRamp {
                targets, duration, ..
            } => {
                *started_move = true;
                let mut record = None;
                for &i in active {
                    let consumed = self.sources[i].next_planned(None, label)?;
                    if Some(i) == emitting {
                        record = Some(consumed);
                    }
                }
                let (from, to, duration) = match record {
                    Some(record) => {
                        let waypoints = &record.segments[0].waypoints;
                        (waypoints[0].clone(), waypoints[1].clone(), record.duration)
                    }
                    None => {
                        // Never simulated, but a ramp's goal is authored
                        // data: apply the targets to the entry
                        // configuration of this untaken path.
                        let from = local_q.clone();
                        let mut to = from.clone();
                        for (joint, value) in targets {
                            let ji = self
                                .model
                                .joint_index(joint)
                                .ok_or_else(|| format!("{label}: unknown joint `{joint}`"))?;
                            let qi = self.model.joints[ji].q_index.ok_or_else(|| {
                                format!("{label}: joint `{joint}` is not actuated")
                            })?;
                            to[qi] = *value;
                        }
                        (from, to, *duration)
                    }
                };
                local_q.clone_from(&to);
                let travel = from
                    .iter()
                    .zip(&to)
                    .map(|(a, b)| (a - b).abs())
                    .fold(0.0, f64::max);
                if travel < 1e-12 {
                    // A hold-position ramp is a dwell; keep its time.
                    if duration > 0.0 {
                        out.push(Command::Sleep { seconds: duration });
                    }
                } else {
                    // The authored duration sets the pace (that is what
                    // the simulation ran); the joint limit caps it.
                    let needed = if duration > 0.0 {
                        travel / duration
                    } else {
                        f64::INFINITY
                    };
                    if needed > self.joint_velocity + 1e-9 {
                        self.warnings.push(format!(
                            "{label}: ramp asks {:.3} rad/s but the joint limit allows \
                             {:.3}; the controller will run it slower",
                            needed, self.joint_velocity,
                        ));
                    }
                    self.note_move(false);
                    out.push(Command::MoveJoint {
                        q: to,
                        velocity: needed.min(self.joint_velocity),
                        acceleration: self.joint_acceleration,
                        blend: 0.0,
                    });
                }
            }
            Action::Attach { object, .. } => {
                out.push(Command::Comment {
                    text: format!(
                        "attach {object} (simulation grasp — drive the real \
                         gripper via a mapped output)"
                    ),
                });
            }
            Action::Detach { object } => {
                out.push(Command::Comment {
                    text: format!("detach {object} (simulation release)"),
                });
            }
            Action::Track { .. } | Action::Untrack { .. } => {
                return Err(format!(
                    "{label}: conveyor tracking cannot be exported — the \
                     controller's own tracking function (e.g. UR conveyor \
                     tracking) has to take that over"
                ));
            }
            Action::Set { signal, value } => {
                let port = self.io.outputs.get(signal).ok_or_else(|| {
                    format!(
                        "{label}: signal `{signal}` has no output port — \
                         bind it on the robot controller node (bind_output) or pass \
                         outputs={{\"{signal}\": <port>}}"
                    )
                })?;
                out.push(Command::SetDigitalOut {
                    port: port.port,
                    value: *value ^ port.invert,
                });
            }
            Action::Device { device, command } => {
                lower_device(device, command, self.io, label, out, &mut self.warnings);
            }
        }
        Ok(())
    }

    /// The first move after a divergent rejoin decides whether the
    /// divergence matters: a joint move is absolute, a straight line is
    /// not.
    fn note_move(&mut self, linear: bool) {
        if let Some((select, deviation)) = self.pending_rejoin.take() {
            if linear {
                self.warnings.push(format!(
                    "the arms of {select} rejoin at configurations up to {deviation:.4} \
                     rad apart, and the next move is a straight line — its path \
                     depends on which arm ran; put a joint move first if the line \
                     matters",
                ));
            }
        }
    }
}

fn resolve_sequence<'a>(
    timeline: &'a SequenceTimeline,
    sequence: Option<&'a str>,
) -> Result<&'a str, String> {
    let rolled = || {
        timeline
            .sequences
            .iter()
            .map(|s| format!("`{s}`"))
            .collect::<Vec<_>>()
            .join(", ")
    };
    match sequence {
        Some(name) => {
            if timeline.sequences.iter().any(|s| s == name) {
                Ok(name)
            } else {
                Err(format!(
                    "sequence `{name}` was not part of this rollout (rolled: {}) — \
                     simulate it, together with the programs it runs beside, first",
                    rolled()
                ))
            }
        }
        None if timeline.sequences.len() == 1 => Ok(&timeline.sequences[0]),
        None => Err(format!(
            "the timeline was rolled from {} programs; pass sequence=<name> \
             (one of: {})",
            timeline.sequences.len(),
            rolled()
        )),
    }
}

/// The single robot this sequence drives. Mirrors the ownership walk in
/// [`Scene::validate_program_ownership`]: motions claim their owner, the
/// addressed actions claim their resolved robot — branch arms included.
fn driven_robot(scene: &Scene, seq: &Sequence) -> Result<usize, String> {
    let mut driven: Option<usize> = None;
    walk_actions(&seq.steps, &mut |action| -> Result<(), String> {
        let robot = match action {
            Action::StartMotion { motion } => scene
                .motions()
                .iter()
                .find(|m| &m.name == motion)
                .map(|m| m.robot)
                .ok_or_else(|| format!("unknown motion `{motion}`"))?,
            Action::StartRamp { robot, .. }
            | Action::Attach { robot, .. }
            | Action::Track { robot, .. }
            | Action::Untrack { robot }
            | Action::StartToolpath { robot, .. } => scene.resolve_seq_robot(robot)?,
            _ => return Ok(()),
        };
        match driven {
            None => driven = Some(robot),
            Some(existing) if existing == robot => {}
            Some(existing) => {
                return Err(format!(
                    "sequence `{}` drives both `{}` and `{}` — a controller \
                     program is one robot's; split it into per-robot programs \
                     (simulate_sequences) and export each",
                    seq.name,
                    scene.robots()[existing].name,
                    scene.robots()[robot].name,
                ));
            }
        }
        Ok(())
    })?;
    driven.ok_or_else(|| {
        format!(
            "sequence `{}` drives no robot — nothing to export (device-only \
             programs stay on the cell controller)",
            seq.name
        )
    })
}

/// The robot a rolled sequence drives — what the Python bindings need to
/// pick the robot-controller node whose bindings project onto the
/// script's I/O ports. Same resolution as the lowering itself.
pub fn driven_robot_name(
    scene: &Scene,
    timeline: &SequenceTimeline,
    sequence: Option<&str>,
) -> Result<String, String> {
    let name = resolve_sequence(timeline, sequence)?;
    let seq = scene
        .sequence(name)
        .ok_or_else(|| format!("the timeline's scene snapshot holds no sequence `{name}`"))?;
    let robot = driven_robot(scene, seq)?;
    Ok(scene.robots()[robot].name.clone())
}

/// A condition as a digital-input snapshot test — what branch guards
/// (and compound level waits) lower to. Timers and edges have no
/// snapshot form and are refused with guidance.
fn digital_test(
    condition: &Condition,
    io: &SequenceIo,
    label: &str,
) -> Result<DigitalTest, String> {
    match condition {
        // A blocking controller has finished every move by the time the
        // guard is read.
        Condition::Immediately | Condition::Done => Ok(DigitalTest::Always),
        Condition::Signal { name, value } => io
            .inputs
            .get(name)
            .map(|port| DigitalTest::Input {
                port: port.port,
                value: *value ^ port.invert,
            })
            .ok_or_else(|| {
                format!(
                    "{label}: signal `{name}` has no input port — bind it on the robot \
                     controller node (bind_input) or pass inputs={{\"{name}\": <port>}}"
                )
            }),
        Condition::DeviceDone { device } => io
            .inputs
            .get(device)
            .map(|port| DigitalTest::Input {
                port: port.port,
                value: !port.invert,
            })
            .ok_or_else(|| {
                format!(
                    "{label}: device `{device}` has no in-position input — bind it \
                     (bind_input) or pass inputs={{\"{device}\": <port>}}"
                )
            }),
        // The program's own robot is idle by the time a blocking
        // controller reads the guard — like `Done`.
        Condition::RobotDone { robot } if io.self_robot.as_deref() == Some(robot.as_str()) => {
            Ok(DigitalTest::Always)
        }
        Condition::RobotDone { robot } => io
            .inputs
            .get(robot)
            .map(|port| DigitalTest::Input {
                port: port.port,
                value: !port.invert,
            })
            .ok_or_else(|| {
                format!(
                    "{label}: `robot_done({robot})` needs the partner controller's \
                     idle contact on an input — bind `{robot}.done` (bind_input) or pass \
                     inputs={{\"{robot}\": <port>}}"
                )
            }),
        Condition::All(conditions) => Ok(DigitalTest::AllOf(
            conditions
                .iter()
                .map(|c| digital_test(c, io, label))
                .collect::<Result<_, _>>()?,
        )),
        Condition::Any(conditions) => Ok(DigitalTest::AnyOf(
            conditions
                .iter()
                .map(|c| digital_test(c, io, label))
                .collect::<Result<_, _>>()?,
        )),
        Condition::Elapsed { .. } => Err(format!(
            "{label}: a timer cannot guard a branch or compound wait in a script \
             — give it its own step, or latch it into a signal"
        )),
        Condition::Rising { .. } | Condition::Falling { .. } => Err(format!(
            "{label}: an edge cannot guard a branch or compound wait in a script \
             — latch the edge into a signal first, or wait for it in its own step"
        )),
    }
}

fn lower_device(
    device: &str,
    command: &DeviceCommand,
    io: &SequenceIo,
    label: &str,
    commands: &mut Vec<Command>,
    warnings: &mut Vec<String>,
) {
    let coil = match command {
        DeviceCommand::Start => Some(true),
        DeviceCommand::Stop => Some(false),
        _ => None,
    };
    match (coil, io.outputs.get(device)) {
        (Some(value), Some(port)) => commands.push(Command::SetDigitalOut {
            port: port.port,
            value: value ^ port.invert,
        }),
        (Some(value), None) => {
            // An unmapped run coil stays the cell controller's: common
            // when the PLC keeps the conveyor and the robot only handshakes.
            warnings.push(format!(
                "{label}: device `{device}` has no output port — the {} stays with \
                 the cell controller (map it via outputs= to drive it from the robot)",
                if value { "start" } else { "stop" },
            ));
            commands.push(Command::Comment {
                text: format!(
                    "device {device}: {} (left to the cell controller)",
                    if value { "start" } else { "stop" }
                ),
            });
        }
        (None, _) => {
            let describe = match command {
                DeviceCommand::SetSpeed(v) => format!("set_speed({v})"),
                DeviceCommand::MoveTo(p) => format!("move_to({p})"),
                DeviceCommand::Goto { station } => format!("goto({station})"),
                DeviceCommand::MoveToStop(stop) => format!("move_to({stop})"),
                DeviceCommand::Advance(d) => format!("advance({d})"),
                DeviceCommand::Start | DeviceCommand::Stop => unreachable!("coil handled above"),
            };
            warnings.push(format!(
                "{label}: device command `{device}.{describe}` is not expressible \
                 as a digital output — the cell controller keeps that job",
            ));
            commands.push(Command::Comment {
                text: format!("device {device}: {describe} (left to the cell controller)"),
            });
        }
    }
}

fn lower_condition(
    condition: &Condition,
    io: &SequenceIo,
    label: &str,
    started_move: bool,
    commands: &mut Vec<Command>,
    warnings: &mut Vec<String>,
) -> Result<(), String> {
    // A blocking move already *is* the Done wait; anything else that runs
    // beside a move in simulation runs after it in the script.
    let concurrency_warning = |warnings: &mut Vec<String>, what: &str| {
        if started_move {
            warnings.push(format!(
                "{label}: {what} runs beside the move in simulation but after it \
                 in the script — the cycle can run slower than the simulated takt",
            ));
        }
    };
    match condition {
        Condition::Immediately | Condition::Done => {}
        Condition::Elapsed { seconds } => {
            concurrency_warning(warnings, "the timer");
            commands.push(Command::Sleep { seconds: *seconds });
        }
        Condition::Signal { name, value } => {
            let port = io.inputs.get(name).ok_or_else(|| {
                format!(
                    "{label}: signal `{name}` has no input port — bind it on the robot \
                     controller node (bind_input) or pass inputs={{\"{name}\": <port>}}"
                )
            })?;
            concurrency_warning(warnings, "the signal wait");
            commands.push(Command::WaitDigitalIn {
                port: port.port,
                value: *value ^ port.invert,
            });
        }
        // An edge lowers to the classic two-stage interlock: wait for the
        // idle level, then for the active one — "the *next* transition",
        // never the state the input happened to arrive in.
        Condition::Rising { name } | Condition::Falling { name } => {
            let port = io.inputs.get(name).ok_or_else(|| {
                format!(
                    "{label}: signal `{name}` has no input port — bind it on the robot \
                     controller node (bind_input) or pass inputs={{\"{name}\": <port>}}"
                )
            })?;
            let rising = matches!(condition, Condition::Rising { .. });
            concurrency_warning(warnings, "the edge wait");
            commands.push(Command::Comment {
                text: format!(
                    "{} edge of {name}: wait {}, then {}",
                    if rising { "rising" } else { "falling" },
                    if rising { "low" } else { "high" },
                    if rising { "high" } else { "low" },
                ),
            });
            commands.push(Command::WaitDigitalIn {
                port: port.port,
                value: !rising ^ port.invert,
            });
            commands.push(Command::WaitDigitalIn {
                port: port.port,
                value: rising ^ port.invert,
            });
        }
        Condition::DeviceDone { device } => {
            let port = io.inputs.get(device).ok_or_else(|| {
                format!(
                    "{label}: device `{device}` has no in-position input — bind it \
                     (bind_input) or pass inputs={{\"{device}\": <port>}}"
                )
            })?;
            concurrency_warning(warnings, "the in-position wait");
            commands.push(Command::WaitDigitalIn {
                port: port.port,
                value: !port.invert,
            });
        }
        // The program's own robot: a blocking controller is idle here by
        // construction — nothing to wait on (same as `Done`).
        Condition::RobotDone { robot } if io.self_robot.as_deref() == Some(robot.as_str()) => {}
        Condition::RobotDone { robot } => {
            let port = io.inputs.get(robot).ok_or_else(|| {
                format!(
                    "{label}: `robot_done({robot})` needs the partner controller's \
                     idle contact on an input — bind `{robot}.done` (bind_input) or pass \
                     inputs={{\"{robot}\": <port>}}"
                )
            })?;
            concurrency_warning(warnings, "the partner wait");
            commands.push(Command::WaitDigitalIn {
                port: port.port,
                value: !port.invert,
            });
        }
        // Compound waits: when every member is a digital snapshot test,
        // the whole thing waits as one boolean expression — simultaneous,
        // like the scan loop. A mix with timers keeps the sequential
        // fallback for `all_of` (conservative: each member in turn);
        // `any_of` has no sequential form and is refused.
        Condition::All(conditions) => match digital_test(condition, io, label) {
            Ok(test) => {
                concurrency_warning(warnings, "the compound wait");
                push_wait(test, commands);
            }
            Err(_) => {
                for condition in conditions {
                    lower_condition(condition, io, label, started_move, commands, warnings)?;
                }
            }
        },
        Condition::Any(conditions) => {
            if let [sole] = conditions.as_slice() {
                lower_condition(sole, io, label, started_move, commands, warnings)?;
            } else {
                let test = digital_test(condition, io, label)?;
                concurrency_warning(warnings, "the compound wait");
                push_wait(test, commands);
            }
        }
    }
    Ok(())
}

/// Emits a level wait in its plainest form: vacuous tests vanish, a lone
/// input keeps the single-contact wait, anything else waits on the
/// simplified boolean expression.
fn push_wait(test: DigitalTest, commands: &mut Vec<Command>) {
    match test.simplified() {
        DigitalTest::Always => {}
        DigitalTest::Input { port, value } => commands.push(Command::WaitDigitalIn { port, value }),
        test => commands.push(Command::WaitTest { test }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rollout::tests::{joint_motion, sample_scene};
    use crate::rollout::RolloutOptions;
    use crate::seq::{Device, DeviceKind, Sensor, SensorKind, SensorWatch, Step};
    use botrail_model::Geometry;
    use nalgebra::{Isometry3, Vector3};

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    fn io(inputs: &[(&str, u32)], outputs: &[(&str, u32)]) -> SequenceIo {
        SequenceIo::from_ports(
            inputs.iter().map(|(n, p)| (n.to_string(), *p)).collect(),
            outputs.iter().map(|(n, p)| (n.to_string(), *p)).collect(),
        )
    }

    /// A one-robot cell whose sequence exercises every lowerable element:
    /// a sensor wait, a coil write, a planned move, a ramp, and a timer.
    fn cell() -> Scene {
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.8);
        joint_motion(&mut scene, "back", 0.0);
        scene.define_signal("vacuum", false);
        // The robot's own upper cube trips the zone from t = 0, so a
        // `part_here` wait resolves immediately and the rollout completes.
        scene
            .upsert_sensor(Sensor {
                name: "part_here".into(),
                kind: SensorKind::Zone {
                    pose: Isometry3::translation(0.0, 0.0, 0.55),
                    size: Vector3::new(2.0, 2.0, 0.4),
                },
                watch: SensorWatch::Robots(vec!["r".into()]),
                mount: None,
            })
            .unwrap();
        scene
            .add_obstacle(
                "part",
                Geometry::Box {
                    size: Vector3::new(0.05, 0.05, 0.05),
                },
                Isometry3::translation(0.4, 0.0, 0.5),
            )
            .unwrap();
        scene
    }

    #[test]
    fn lowers_moves_io_and_waits_in_step_order() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "wait part",
                    vec![],
                    Condition::Signal {
                        name: "part_here".into(),
                        value: true,
                    },
                ),
                step(
                    "approach",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                step(
                    "grip",
                    vec![
                        Action::Set {
                            signal: "vacuum".into(),
                            value: true,
                        },
                        Action::Attach {
                            robot: None,
                            object: "part".into(),
                            link: None,
                            touch_links: None,
                        },
                    ],
                    Condition::Elapsed { seconds: 0.4 },
                ),
                step(
                    "return",
                    vec![Action::StartMotion {
                        motion: "back".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("pick", &RolloutOptions::default())
            .unwrap();
        let out = sequence_program(
            &scene,
            &tl,
            None,
            &io(&[("part_here", 3)], &[("vacuum", 1)]),
            &ProgramOptions::default(),
        )
        .unwrap();

        let kinds: Vec<&'static str> = out
            .program
            .commands
            .iter()
            .map(|c| match c {
                Command::MoveJoint { .. } => "movej",
                Command::MoveLinear { .. } => "movel",
                Command::SetDigitalOut { .. } => "out",
                Command::WaitDigitalIn { .. } => "in",
                Command::WaitTest { .. } => "wait",
                Command::Sleep { .. } => "sleep",
                Command::Comment { .. } => "#",
                Command::Select { .. } => "select",
            })
            .collect();
        // move-to-start, then: wait step, approach move, grip (coil +
        // attach comment + timer), return move.
        let expected_prefix = ["movej", "#", "in"];
        assert_eq!(&kinds[..3], &expected_prefix);
        assert!(kinds.contains(&"out"));
        assert!(kinds.contains(&"sleep"));
        // Everything the sequence fires is present, nothing silently lost:
        // 2 motions' waypoints as moves + 1 move-to-start.
        let moves = kinds.iter().filter(|k| **k == "movej").count();
        assert!(moves >= 3, "kinds = {kinds:?}");
        assert!(out.warnings.is_empty(), "warnings = {:?}", out.warnings);
    }

    #[test]
    fn ramp_lowers_to_duration_paced_move_and_dwell() {
        let mut scene = sample_scene();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "close",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("j".into(), 0.5)],
                        duration: 2.0,
                    }],
                    Condition::Done,
                ),
                step(
                    "settle",
                    vec![Action::StartRamp {
                        robot: None,
                        targets: vec![("j".into(), 0.5)],
                        duration: 0.3,
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let out = sequence_program(
            &scene,
            &tl,
            None,
            &SequenceIo::default(),
            &ProgramOptions::default(),
        )
        .unwrap();
        let moves: Vec<_> = out
            .program
            .commands
            .iter()
            .filter_map(|c| match c {
                Command::MoveJoint { q, velocity, .. } => Some((q.clone(), *velocity)),
                _ => None,
            })
            .collect();
        // move-to-start + the paced ramp: 0.5 rad over 2 s = 0.25 rad/s.
        assert_eq!(moves.len(), 2);
        assert!((moves[1].1 - 0.25).abs() < 1e-12, "v = {}", moves[1].1);
        // The zero-travel ramp keeps its time as a dwell.
        assert!(out
            .program
            .commands
            .iter()
            .any(|c| matches!(c, Command::Sleep { seconds } if (*seconds - 0.3).abs() < 1e-12)));
    }

    #[test]
    fn unmapped_names_are_actionable_errors() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "wait",
                vec![],
                Condition::Signal {
                    name: "part_here".into(),
                    value: true,
                },
            )],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let err = sequence_program(
            &scene,
            &tl,
            None,
            &SequenceIo::default(),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        // A wait-only sequence drives no robot — that error comes first
        // and is the right guidance (nothing to run on a controller).
        assert!(err.contains("drives no robot"), "{err}");

        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                step(
                    "mark",
                    vec![Action::Set {
                        signal: "vacuum".into(),
                        value: true,
                    }],
                    Condition::Immediately,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let err = sequence_program(
            &scene,
            &tl,
            None,
            &SequenceIo::default(),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("vacuum") && err.contains("outputs="), "{err}");
    }

    #[test]
    fn any_of_lowers_to_a_compound_wait_unless_a_timer_hides_in_it() {
        // A timer member has no digital snapshot: refused with guidance.
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "race",
                vec![Action::StartMotion {
                    motion: "go".into(),
                }],
                Condition::Any(vec![
                    Condition::Elapsed { seconds: 1.0 },
                    Condition::Signal {
                        name: "part_here".into(),
                        value: true,
                    },
                ]),
            )],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let err = sequence_program(
            &scene,
            &tl,
            None,
            &io(&[("part_here", 0)], &[]),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("timer"), "{err}");

        // Pure digital members wait as one boolean expression.
        let mut scene = cell();
        scene.define_signal("abort", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "race",
                vec![Action::StartMotion {
                    motion: "go".into(),
                }],
                Condition::Any(vec![
                    Condition::Signal {
                        name: "abort".into(),
                        value: true,
                    },
                    Condition::Signal {
                        name: "part_here".into(),
                        value: true,
                    },
                ]),
            )],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let out = sequence_program(
            &scene,
            &tl,
            None,
            &io(&[("part_here", 0), ("abort", 4)], &[]),
            &ProgramOptions::default(),
        )
        .unwrap();
        assert!(out
            .program
            .commands
            .iter()
            .any(|c| matches!(c, Command::WaitTest { test: DigitalTest::AnyOf(tests) } if tests.len() == 2)));
    }

    #[test]
    fn concurrency_and_unmapped_devices_warn() {
        let mut scene = cell();
        scene.upsert_device(Device {
            name: "conv".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: Isometry3::translation(5.0, 0.0, 0.0),
                zone_size: Vector3::new(1.0, 1.0, 0.2),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running: true,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "move while timing",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Elapsed { seconds: 3.0 },
                ),
                step(
                    "index",
                    vec![Action::Device {
                        device: "conv".into(),
                        command: DeviceCommand::Stop,
                    }],
                    Condition::Immediately,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let out = sequence_program(
            &scene,
            &tl,
            None,
            &SequenceIo::default(),
            &ProgramOptions::default(),
        )
        .unwrap();
        assert_eq!(out.warnings.len(), 2, "{:?}", out.warnings);
        assert!(out.warnings[0].contains("beside the move"));
        assert!(out.warnings[1].contains("conv"));
        // The unmapped stop is a comment, not a lost action.
        assert!(out.program.commands.iter().any(
            |c| matches!(c, Command::Comment { text } if text.contains("conv") && text.contains("stop"))
        ));
    }

    fn arm(condition: Condition, steps: Vec<Step>) -> crate::seq::SelectArm {
        crate::seq::SelectArm { condition, steps }
    }

    fn select_step(name: &str, arms: Vec<crate::seq::SelectArm>) -> Step {
        Step {
            name: name.to_string(),
            actions: vec![],
            transition: Condition::Immediately,
            select: arms,
        }
    }

    fn sig(name: &str) -> Condition {
        Condition::Signal {
            name: name.into(),
            value: true,
        }
    }

    #[test]
    fn branches_lower_to_if_chains_with_skipped_arms_carried() {
        let mut scene = cell();
        scene.define_signal("ok", true);
        scene.define_signal("ng", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "approach",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                select_step(
                    "judge",
                    vec![
                        arm(
                            sig("ok"),
                            vec![step(
                                "place",
                                vec![Action::StartMotion {
                                    motion: "back".into(),
                                }],
                                Condition::Done,
                            )],
                        ),
                        arm(
                            sig("ng"),
                            vec![step(
                                "purge",
                                vec![
                                    Action::StartRamp {
                                        robot: None,
                                        targets: vec![("j".into(), 0.3)],
                                        duration: 0.5,
                                    },
                                    Action::Set {
                                        signal: "vacuum".into(),
                                        value: false,
                                    },
                                ],
                                Condition::Done,
                            )],
                        ),
                    ],
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        assert_eq!(tl.branches.len(), 1);
        assert_eq!(tl.branches[0].arm, 0);

        let out = sequence_program(
            &scene,
            &tl,
            None,
            &io(&[("ok", 1), ("ng", 2)], &[("vacuum", 3)]),
            &ProgramOptions::default(),
        )
        .unwrap();
        let select = out
            .program
            .commands
            .iter()
            .find_map(|c| match c {
                Command::Select { arms } => Some(arms),
                _ => None,
            })
            .expect("a select command");
        assert_eq!(select.len(), 2);
        assert!(matches!(
            select[0].test,
            DigitalTest::Input {
                port: 1,
                value: true
            }
        ));
        // The taken arm carries its planned motion.
        assert!(select[0]
            .body
            .iter()
            .any(|c| matches!(c, Command::MoveJoint { .. })));
        // The skipped arm still exports: its ramp is synthesized from the
        // branch-entry configuration (0.8 after `go`) to the authored
        // 0.3, paced by the authored duration.
        let ramp = select[1]
            .body
            .iter()
            .find_map(|c| match c {
                Command::MoveJoint { q, velocity, .. } => Some((q.clone(), *velocity)),
                _ => None,
            })
            .expect("synthesized ramp move");
        assert_eq!(ramp.0, vec![0.3]);
        assert!((ramp.1 - 0.5 / 0.5).abs() < 1e-9, "v = {}", ramp.1);
        assert!(select[1].body.iter().any(|c| matches!(
            c,
            Command::SetDigitalOut {
                port: 3,
                value: false
            }
        )));
    }

    #[test]
    fn merged_export_splices_donor_arms() {
        let mut scene = sample_scene();
        scene.define_signal("ok", true);
        scene.define_signal("ng", false);
        joint_motion(&mut scene, "approach", 0.4);
        joint_motion(&mut scene, "hi", 0.8);
        joint_motion(&mut scene, "lo", -0.5);
        joint_motion(&mut scene, "home", 0.0);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "approach",
                    vec![Action::StartMotion {
                        motion: "approach".into(),
                    }],
                    Condition::Done,
                ),
                select_step(
                    "judge",
                    vec![
                        (arm(
                            sig("ok"),
                            vec![step(
                                "good",
                                vec![Action::StartMotion {
                                    motion: "hi".into(),
                                }],
                                Condition::Done,
                            )],
                        )),
                        (arm(
                            sig("ng"),
                            vec![step(
                                "bad",
                                vec![Action::StartMotion {
                                    motion: "lo".into(),
                                }],
                                Condition::Done,
                            )],
                        )),
                    ],
                ),
                step(
                    "home",
                    vec![Action::StartMotion {
                        motion: "home".into(),
                    }],
                    Condition::Done,
                ),
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
        let base = scene
            .simulate_sequences_scenario(&["s"], None, &options)
            .unwrap();
        let ng = scene
            .simulate_sequences_scenario(&["s"], Some("ng_part"), &options)
            .unwrap();
        let wiring = io(&[("ok", 1), ("ng", 2)], &[]);

        // One bake alone cannot carry the other arm's motion.
        let err =
            sequence_program(&scene, &base, None, &wiring, &ProgramOptions::default()).unwrap_err();
        assert!(err.contains("never planned"), "{err}");

        // The pair merges: each arm's moves come from the bake that took
        // it, and both bakes' cursors drain (no leftover errors).
        let out = merged_sequence_program(
            &scene,
            &[("baseline", &base), ("ng_part", &ng)],
            None,
            &wiring,
            &ProgramOptions::default(),
        )
        .unwrap();
        let arms = out
            .program
            .commands
            .iter()
            .find_map(|c| match c {
                Command::Select { arms } => Some(arms),
                _ => None,
            })
            .expect("a select command");
        let last_target = |body: &[Command]| {
            body.iter()
                .rev()
                .find_map(|c| match c {
                    Command::MoveJoint { q, .. } => Some(q.clone()),
                    _ => None,
                })
                .expect("a joint move in the arm")
        };
        assert_eq!(last_target(&arms[0].body), vec![0.8]);
        assert_eq!(last_target(&arms[1].body), vec![-0.5]);
        // Same entry configuration (both approached to 0.4): no splice
        // error, and the post-rejoin move is a joint move — no warning.
        assert!(out.warnings.is_empty(), "{:?}", out.warnings);

        // Roles swap cleanly: the ng bake as primary donates the ok arm.
        let out = merged_sequence_program(
            &scene,
            &[("ng_part", &ng), ("baseline", &base)],
            None,
            &wiring,
            &ProgramOptions::default(),
        )
        .unwrap();
        let arms = out
            .program
            .commands
            .iter()
            .find_map(|c| match c {
                Command::Select { arms } => Some(arms),
                _ => None,
            })
            .unwrap();
        assert_eq!(last_target(&arms[0].body), vec![0.8]);
        assert_eq!(last_target(&arms[1].body), vec![-0.5]);
    }

    #[test]
    fn merge_refuses_scenarios_that_diverge_before_the_branch() {
        let mut scene = sample_scene();
        scene.define_signal("ok", true);
        scene.define_signal("ng", false);
        joint_motion(&mut scene, "hi", 0.8);
        joint_motion(&mut scene, "lo", -0.5);
        // No shared move before the branch: the entry configuration is
        // the start q, which the scenario shifts.
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![select_step(
                "judge",
                vec![
                    arm(
                        sig("ok"),
                        vec![step(
                            "good",
                            vec![Action::StartMotion {
                                motion: "hi".into(),
                            }],
                            Condition::Done,
                        )],
                    ),
                    arm(
                        sig("ng"),
                        vec![step(
                            "bad",
                            vec![Action::StartMotion {
                                motion: "lo".into(),
                            }],
                            Condition::Done,
                        )],
                    ),
                ],
            )],
        });
        scene
            .upsert_scenario(crate::seq::Scenario {
                name: "shifted".into(),
                signals: vec![("ok".into(), false), ("ng".into(), true)],
                obstacles: vec![],
                joints: vec![("r".into(), vec![0.5])],
                faults: vec![],
            })
            .unwrap();
        let options = RolloutOptions::default();
        let base = scene
            .simulate_sequences_scenario(&["s"], None, &options)
            .unwrap();
        let shifted = scene
            .simulate_sequences_scenario(&["s"], Some("shifted"), &options)
            .unwrap();
        let err = merged_sequence_program(
            &scene,
            &[("baseline", &base), ("shifted", &shifted)],
            None,
            &io(&[("ok", 1), ("ng", 2)], &[]),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("diverged before the branch"), "{err}");
    }

    #[test]
    fn straight_line_after_divergent_rejoin_warns() {
        use crate::rollout::{BranchTaken, PlannedMove, RobotTrack};
        use botrail_traj::JointTrajectory;

        // Hand-built timelines: the arms rejoin 1.0 rad apart and the
        // next planned move is a straight line — the one case where the
        // divergence changes the executed path.
        let mut scene = sample_scene();
        scene.define_signal("ok", true);
        scene.define_signal("ng", false);
        scene
            .add_segment(
                "slide",
                crate::motion::Segment {
                    kind: SegmentKind::CartesianLine,
                    goal_positions: vec![0.2],
                    constraints: vec![],
                },
            )
            .unwrap();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                select_step(
                    "judge",
                    vec![
                        arm(
                            sig("ok"),
                            vec![step(
                                "good",
                                vec![Action::StartRamp {
                                    robot: None,
                                    targets: vec![("j".into(), 0.5)],
                                    duration: 0.5,
                                }],
                                Condition::Done,
                            )],
                        ),
                        arm(
                            sig("ng"),
                            vec![step(
                                "bad",
                                vec![Action::StartRamp {
                                    robot: None,
                                    targets: vec![("j".into(), -0.5)],
                                    duration: 0.5,
                                }],
                                Condition::Done,
                            )],
                        ),
                    ],
                ),
                step(
                    "slide",
                    vec![Action::StartMotion {
                        motion: "slide".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let track = |ramp_to: f64| RobotTrack {
            name: "r".into(),
            trajectory: JointTrajectory {
                times: vec![0.0, 1.0],
                positions: vec![vec![0.0], vec![0.2]],
                velocities: vec![vec![0.0], vec![0.0]],
            },
            moves: vec![],
            planned: vec![
                PlannedMove {
                    sequence: "s".into(),
                    step: 1,
                    motion: None,
                    segments: vec![crate::motion::PlannedSegment {
                        kind: SegmentKind::Joint,
                        waypoints: vec![vec![0.0], vec![ramp_to]],
                        tcp_speed: None,
                    }],
                    duration: 0.5,
                    feed_report: None,
                    process_spans: Vec::new(),
                },
                PlannedMove {
                    sequence: "s".into(),
                    step: 2,
                    motion: Some("slide".into()),
                    segments: vec![crate::motion::PlannedSegment {
                        kind: SegmentKind::CartesianLine,
                        waypoints: vec![vec![ramp_to], vec![0.2]],
                        tcp_speed: None,
                    }],
                    duration: 1.0,
                    feed_report: None,
                    process_spans: Vec::new(),
                },
            ],
            base: None,
            footfalls: Vec::new(),
            sway: Vec::new(),
            pitch: Vec::new(),
            rise: Vec::new(),
        };
        let timeline = |ramp_to: f64, arm: usize| crate::rollout::SequenceTimeline {
            duration: 1.0,
            sequences: vec!["s".into()],
            scenario: None,
            physics: None,
            contacts: vec![],
            robots: vec![track(ramp_to)],
            objects: vec![],
            vehicles: vec![],
            signals: vec![],
            step_spans: vec![],
            branches: vec![BranchTaken {
                sequence: "s".into(),
                step: "judge".into(),
                select: 0,
                arm,
            }],
        };
        let base = timeline(0.5, 0);
        let ng = timeline(-0.5, 1);
        let out = merged_sequence_program(
            &scene,
            &[("baseline", &base), ("ng", &ng)],
            None,
            &io(&[("ok", 1), ("ng", 2)], &[]),
            &ProgramOptions::default(),
        )
        .unwrap();
        assert!(
            out.warnings
                .iter()
                .any(|w| w.contains("straight line") && w.contains("judge")),
            "{:?}",
            out.warnings
        );
    }

    #[test]
    fn skipped_arm_motions_are_refused() {
        let mut scene = cell();
        scene.define_signal("ok", true);
        scene.define_signal("ng", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "approach",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                select_step(
                    "judge",
                    vec![
                        arm(sig("ok"), vec![]),
                        arm(
                            sig("ng"),
                            vec![step(
                                "rework",
                                vec![Action::StartMotion {
                                    motion: "back".into(),
                                }],
                                Condition::Done,
                            )],
                        ),
                    ],
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let err = sequence_program(
            &scene,
            &tl,
            None,
            &io(&[("ok", 1), ("ng", 2)], &[]),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("never planned"), "{err}");
    }

    #[test]
    fn edges_lower_to_two_stage_waits_and_cannot_guard_arms() {
        let mut scene = cell();
        scene.define_signal("pulse", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "next part",
                    vec![],
                    Condition::Rising {
                        name: "pulse".into(),
                    },
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
        scene.upsert_sequence(Sequence {
            name: "pulser".into(),
            steps: vec![
                step("hold", vec![], Condition::Elapsed { seconds: 0.1 }),
                step(
                    "fire",
                    vec![Action::Set {
                        signal: "pulse".into(),
                        value: true,
                    }],
                    Condition::Immediately,
                ),
            ],
        });
        let tl = scene
            .simulate_sequences(&["s", "pulser"], &RolloutOptions::default())
            .unwrap();
        let out = sequence_program(
            &scene,
            &tl,
            Some("s"),
            &io(&[("pulse", 2)], &[]),
            &ProgramOptions::default(),
        )
        .unwrap();
        let waits: Vec<(u32, bool)> = out
            .program
            .commands
            .iter()
            .filter_map(|c| match c {
                Command::WaitDigitalIn { port, value } => Some((*port, *value)),
                _ => None,
            })
            .collect();
        // Wait low, then high: the *next* rising edge, not the level.
        assert_eq!(waits, vec![(2, false), (2, true)]);

        // As a branch guard there is no snapshot form: refused (the
        // rollout itself is fine with it — only the script is not).
        let mut scene = cell();
        scene.define_signal("pulse", false);
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "move",
                    vec![Action::StartMotion {
                        motion: "go".into(),
                    }],
                    Condition::Done,
                ),
                select_step(
                    "gate",
                    vec![arm(
                        Condition::Rising {
                            name: "pulse".into(),
                        },
                        vec![],
                    )],
                ),
            ],
        });
        scene.upsert_sequence(Sequence {
            name: "pulser".into(),
            steps: vec![
                step("hold", vec![], Condition::Elapsed { seconds: 3.0 }),
                step(
                    "fire",
                    vec![Action::Set {
                        signal: "pulse".into(),
                        value: true,
                    }],
                    Condition::Immediately,
                ),
            ],
        });
        let tl = scene
            .simulate_sequences(&["s", "pulser"], &RolloutOptions::default())
            .unwrap();
        let err = sequence_program(
            &scene,
            &tl,
            Some("s"),
            &io(&[("pulse", 2)], &[]),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("latch the edge"), "{err}");
    }

    #[test]
    fn tracking_steps_are_refused() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![
                step(
                    "latch",
                    vec![Action::Track {
                        robot: None,
                        object: "part".into(),
                        link: None,
                    }],
                    Condition::Elapsed { seconds: 0.1 },
                ),
                step(
                    "release",
                    vec![Action::Untrack { robot: None }],
                    Condition::Immediately,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let err = sequence_program(
            &scene,
            &tl,
            None,
            &SequenceIo::default(),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("tracking"), "{err}");
    }

    #[test]
    fn multi_robot_sequences_are_refused() {
        let mut scene = sample_scene();
        let model = scene.robots()[0].model.clone();
        scene.rename_robot(0, "a");
        scene.add_robot(model, Some("b"), Isometry3::translation(0.0, 3.0, 0.0));
        for (robot, name) in [(0usize, "a_go"), (1, "b_go")] {
            scene
                .add_segment_for(
                    robot,
                    name,
                    crate::motion::Segment {
                        kind: SegmentKind::Joint,
                        goal_positions: vec![0.5],
                        constraints: vec![],
                    },
                )
                .unwrap();
        }
        scene.upsert_sequence(Sequence {
            name: "both".into(),
            steps: vec![
                step(
                    "a moves",
                    vec![Action::StartMotion {
                        motion: "a_go".into(),
                    }],
                    Condition::Done,
                ),
                step(
                    "b moves",
                    vec![Action::StartMotion {
                        motion: "b_go".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        let tl = scene
            .simulate_sequence("both", &RolloutOptions::default())
            .unwrap();
        let err = sequence_program(
            &scene,
            &tl,
            None,
            &SequenceIo::default(),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("per-robot programs"), "{err}");
    }

    #[test]
    fn wrong_or_foreign_sequences_are_refused() {
        let mut scene = cell();
        scene.upsert_sequence(Sequence {
            name: "s".into(),
            steps: vec![step(
                "move",
                vec![Action::StartMotion {
                    motion: "go".into(),
                }],
                Condition::Done,
            )],
        });
        let tl = scene
            .simulate_sequence("s", &RolloutOptions::default())
            .unwrap();
        let err = sequence_program(
            &scene,
            &tl,
            Some("other"),
            &SequenceIo::default(),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("not part of this rollout"), "{err}");

        // Editing the sequence after the rollout must not export a
        // mismatched program.
        let mut edited = Sequence {
            name: "s".into(),
            steps: vec![step(
                "move",
                vec![Action::StartMotion {
                    motion: "go".into(),
                }],
                Condition::Done,
            )],
        };
        edited.steps.push(step(
            "extra",
            vec![Action::StartMotion {
                motion: "back".into(),
            }],
            Condition::Done,
        ));
        scene.upsert_sequence(edited);
        let err = sequence_program(
            &scene,
            &tl,
            None,
            &SequenceIo::default(),
            &ProgramOptions::default(),
        )
        .unwrap_err();
        assert!(err.contains("re-simulate"), "{err}");
    }
}
