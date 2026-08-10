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
//! that lean on that concurrency get a warning. What cannot be expressed
//! at all (parallel `any_of` waits, conveyor tracking, positioning device
//! commands) is refused or downgraded to a comment, never silently
//! dropped.

use std::collections::HashMap;

use botrail_export::{Command, Program, ProgramOptions};

use crate::motion::SegmentKind;
use crate::rollout::{PlannedMove, SequenceTimeline};
use crate::seq::{Action, Condition, DeviceCommand, Sequence};
use crate::Scene;

/// Name → digital port wiring for a sequence export.
///
/// Keys are the names the sequence already uses: signal names (sensors and
/// internal flags), device names, and — for `robot_done` waits — robot
/// instance names. `inputs` feed level waits, `outputs` feed coil writes.
#[derive(Debug, Clone, Default)]
pub struct SequenceIo {
    pub inputs: HashMap<String, u32>,
    pub outputs: HashMap<String, u32>,
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
    let name = resolve_sequence(timeline, sequence)?;
    let seq = scene.sequence(name).ok_or_else(|| {
        format!("the timeline's scene snapshot holds no sequence `{name}` (internal mismatch)")
    })?;
    let robot = driven_robot(scene, seq)?;
    let robot_name = scene.robots()[robot].name.clone();
    let track = timeline
        .robots
        .iter()
        .find(|t| t.name == robot_name)
        .ok_or_else(|| format!("timeline has no track for robot `{robot_name}`"))?;

    let model = &scene.robots()[robot].model;
    let limits = crate::motion::traj_limits(model);
    if options.speed_scale <= 0.0 || options.tcp_speed <= 0.0 || options.tcp_accel <= 0.0 {
        return Err("speed scale, tcp speed, and tcp acceleration must be positive".to_string());
    }
    let fold_min = |xs: &[f64]| xs.iter().copied().fold(f64::INFINITY, f64::min);
    let joint_velocity = fold_min(&limits.velocity) * options.speed_scale;
    let joint_acceleration = fold_min(&limits.acceleration) * options.speed_scale;
    let tcp_velocity = options.tcp_speed * options.speed_scale;
    let tcp_acceleration = options.tcp_accel * options.speed_scale;

    let mut planned = track
        .planned
        .iter()
        .filter(|p| p.sequence == name)
        .peekable();
    let mut commands = Vec::new();
    let mut warnings = Vec::new();

    if options.move_to_start {
        if let Some(first) = track
            .planned
            .iter()
            .filter(|p| p.sequence == name)
            .find_map(|p| p.segments.first().and_then(|s| s.waypoints.first()))
        {
            commands.push(Command::MoveJoint {
                q: first.clone(),
                velocity: joint_velocity,
                acceleration: joint_acceleration,
                blend: 0.0,
            });
        }
    }

    for (index, step) in seq.steps.iter().enumerate() {
        commands.push(Command::Comment {
            text: format!("step {}: {}", index + 1, step.name),
        });
        let mut started_move = false;
        for action in &step.actions {
            match action {
                Action::StartMotion { motion } => {
                    let record = next_planned(&mut planned, index, Some(motion))?;
                    started_move = true;
                    for segment in &record.segments {
                        match segment.kind {
                            SegmentKind::Joint => {
                                let last = segment.waypoints.len().saturating_sub(1);
                                for (i, q) in segment.waypoints.iter().enumerate().skip(1) {
                                    commands.push(Command::MoveJoint {
                                        q: q.clone(),
                                        velocity: joint_velocity,
                                        acceleration: joint_acceleration,
                                        blend: if i < last { options.blend_radius } else { 0.0 },
                                    });
                                }
                            }
                            SegmentKind::CartesianLine => {
                                if segment.waypoints.len() > 1 {
                                    commands.push(Command::MoveLinear {
                                        q: segment.waypoints.last().expect("len > 1").clone(),
                                        velocity: tcp_velocity,
                                        acceleration: tcp_acceleration,
                                        blend: 0.0,
                                    });
                                }
                            }
                        }
                    }
                }
                Action::StartRamp { .. } => {
                    let record = next_planned(&mut planned, index, None)?;
                    started_move = true;
                    let waypoints = &record.segments[0].waypoints;
                    let (from, to) = (&waypoints[0], &waypoints[1]);
                    let travel = from
                        .iter()
                        .zip(to)
                        .map(|(a, b)| (a - b).abs())
                        .fold(0.0, f64::max);
                    if travel < 1e-12 {
                        // A hold-position ramp is a dwell; keep its time.
                        if record.duration > 0.0 {
                            commands.push(Command::Sleep {
                                seconds: record.duration,
                            });
                        }
                    } else {
                        // The authored duration sets the pace (that is what
                        // the simulation ran); the joint limit caps it.
                        let needed = if record.duration > 0.0 {
                            travel / record.duration
                        } else {
                            f64::INFINITY
                        };
                        if needed > joint_velocity + 1e-9 {
                            warnings.push(format!(
                                "step {} `{}`: ramp asks {:.3} rad/s but the joint \
                                 limit allows {:.3}; the controller will run it slower",
                                index + 1,
                                step.name,
                                needed,
                                joint_velocity,
                            ));
                        }
                        commands.push(Command::MoveJoint {
                            q: to.clone(),
                            velocity: needed.min(joint_velocity),
                            acceleration: joint_acceleration,
                            blend: 0.0,
                        });
                    }
                }
                Action::Attach { object, .. } => {
                    commands.push(Command::Comment {
                        text: format!(
                            "attach {object} (simulation grasp — drive the real \
                             gripper via a mapped output)"
                        ),
                    });
                }
                Action::Detach { object } => {
                    commands.push(Command::Comment {
                        text: format!("detach {object} (simulation release)"),
                    });
                }
                Action::Track { .. } | Action::Untrack { .. } => {
                    return Err(format!(
                        "step {} `{}`: conveyor tracking cannot be exported — the \
                         controller's own tracking function (e.g. UR conveyor \
                         tracking) has to take that over",
                        index + 1,
                        step.name,
                    ));
                }
                Action::Set { signal, value } => {
                    let port = io.outputs.get(signal).ok_or_else(|| {
                        format!(
                            "step {} `{}`: signal `{signal}` has no output port — \
                             pass outputs={{\"{signal}\": <port>}}",
                            index + 1,
                            step.name,
                        )
                    })?;
                    commands.push(Command::SetDigitalOut {
                        port: *port,
                        value: *value,
                    });
                }
                Action::Device { device, command } => {
                    lower_device(
                        device,
                        command,
                        io,
                        index,
                        &step.name,
                        &mut commands,
                        &mut warnings,
                    );
                }
            }
        }
        lower_condition(
            &step.transition,
            io,
            index,
            &step.name,
            started_move,
            &mut commands,
            &mut warnings,
        )?;
    }
    if planned.peek().is_some() {
        return Err(format!(
            "timeline holds more planned moves for `{name}` than the sequence \
             fires (was the sequence edited after the rollout?)"
        ));
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
        warnings,
    })
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
/// addressed actions claim their resolved robot.
fn driven_robot(scene: &Scene, seq: &Sequence) -> Result<usize, String> {
    let mut driven: Option<usize> = None;
    for step in &seq.steps {
        for action in &step.actions {
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
                | Action::Untrack { robot } => scene.resolve_seq_robot(robot)?,
                _ => continue,
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
        }
    }
    driven.ok_or_else(|| {
        format!(
            "sequence `{}` drives no robot — nothing to export (device-only \
             programs stay on the cell controller)",
            seq.name
        )
    })
}

fn next_planned<'a>(
    planned: &mut std::iter::Peekable<impl Iterator<Item = &'a PlannedMove>>,
    step: usize,
    motion: Option<&str>,
) -> Result<&'a PlannedMove, String> {
    let record = planned
        .next()
        .filter(|p| p.step == step && p.motion.as_deref() == motion && !p.segments.is_empty());
    record.ok_or_else(|| {
        "timeline does not match the sequence (was it edited after the rollout?) — \
         re-simulate and export again"
            .to_string()
    })
}

fn lower_device(
    device: &str,
    command: &DeviceCommand,
    io: &SequenceIo,
    step_index: usize,
    step_name: &str,
    commands: &mut Vec<Command>,
    warnings: &mut Vec<String>,
) {
    let coil = match command {
        DeviceCommand::Start => Some(true),
        DeviceCommand::Stop => Some(false),
        _ => None,
    };
    match (coil, io.outputs.get(device)) {
        (Some(value), Some(port)) => commands.push(Command::SetDigitalOut { port: *port, value }),
        (Some(value), None) => {
            // An unmapped run coil stays the cell controller's: common
            // when the PLC keeps the conveyor and the robot only handshakes.
            warnings.push(format!(
                "step {} `{step_name}`: device `{device}` has no output port — \
                 the {} stays with the cell controller (map it via outputs= to \
                 drive it from the robot)",
                step_index + 1,
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
                DeviceCommand::Advance(d) => format!("advance({d})"),
                DeviceCommand::Start | DeviceCommand::Stop => unreachable!("coil handled above"),
            };
            warnings.push(format!(
                "step {} `{step_name}`: device command `{device}.{describe}` is not \
                 expressible as a digital output — the cell controller keeps that job",
                step_index + 1,
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
    step_index: usize,
    step_name: &str,
    started_move: bool,
    commands: &mut Vec<Command>,
    warnings: &mut Vec<String>,
) -> Result<(), String> {
    // A blocking move already *is* the Done wait; anything else that runs
    // beside a move in simulation runs after it in the script.
    let concurrency_warning = |warnings: &mut Vec<String>, what: &str| {
        if started_move {
            warnings.push(format!(
                "step {} `{step_name}`: {what} runs beside the move in simulation \
                 but after it in the script — the cycle can run slower than the \
                 simulated takt",
                step_index + 1,
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
                    "step {} `{step_name}`: signal `{name}` has no input port — \
                     pass inputs={{\"{name}\": <port>}}",
                    step_index + 1,
                )
            })?;
            concurrency_warning(warnings, "the signal wait");
            commands.push(Command::WaitDigitalIn {
                port: *port,
                value: *value,
            });
        }
        Condition::DeviceDone { device } => {
            let port = io.inputs.get(device).ok_or_else(|| {
                format!(
                    "step {} `{step_name}`: device `{device}` has no in-position \
                     input — pass inputs={{\"{device}\": <port>}}",
                    step_index + 1,
                )
            })?;
            concurrency_warning(warnings, "the in-position wait");
            commands.push(Command::WaitDigitalIn {
                port: *port,
                value: true,
            });
        }
        Condition::RobotDone { robot } => {
            let port = io.inputs.get(robot).ok_or_else(|| {
                format!(
                    "step {} `{step_name}`: `robot_done({robot})` needs the partner \
                     controller's idle contact on an input — pass \
                     inputs={{\"{robot}\": <port>}}",
                    step_index + 1,
                )
            })?;
            concurrency_warning(warnings, "the partner wait");
            commands.push(Command::WaitDigitalIn {
                port: *port,
                value: true,
            });
        }
        Condition::All(conditions) => {
            for condition in conditions {
                lower_condition(
                    condition,
                    io,
                    step_index,
                    step_name,
                    started_move,
                    commands,
                    warnings,
                )?;
            }
        }
        Condition::Any(conditions) => {
            if let [sole] = conditions.as_slice() {
                lower_condition(
                    sole,
                    io,
                    step_index,
                    step_name,
                    started_move,
                    commands,
                    warnings,
                )?;
            } else {
                return Err(format!(
                    "step {} `{step_name}`: `any_of` cannot be lowered to \
                     sequential digital waits — restructure the step (watch one \
                     contact, or let the cell controller arbitrate)",
                    step_index + 1,
                ));
            }
        }
    }
    Ok(())
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
        }
    }

    fn io(inputs: &[(&str, u32)], outputs: &[(&str, u32)]) -> SequenceIo {
        SequenceIo {
            inputs: inputs.iter().map(|(n, p)| (n.to_string(), *p)).collect(),
            outputs: outputs.iter().map(|(n, p)| (n.to_string(), *p)).collect(),
        }
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
        scene.upsert_sensor(Sensor {
            name: "part_here".into(),
            kind: SensorKind::Zone {
                pose: Isometry3::translation(0.0, 0.0, 0.55),
                size: Vector3::new(2.0, 2.0, 0.4),
            },
            watch: SensorWatch::Robots(vec!["r".into()]),
            mount: None,
        });
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
                Command::Sleep { .. } => "sleep",
                Command::Comment { .. } => "#",
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
    fn any_of_is_refused() {
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
        assert!(err.contains("any_of"), "{err}");
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
