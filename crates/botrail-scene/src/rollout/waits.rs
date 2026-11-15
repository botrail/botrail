//! What the programs of a timeline wait on — the first question asked of a
//! stopped cell, asked of a bake or of a live plant's record: at an
//! instant, the active step of every program and the contacts of its exits
//! that do not conduct yet; over the run, every step's stay and the exit
//! that let the program on.
//!
//! The evaluation is the studio's (`studio/src/sfc.ts` `truth`), so the
//! dock's waiting line and these rows agree: levels and timers are exact
//! reconstructions from the recorded lanes and spans; an edge conducts for
//! [`EDGE_HOLD`] after it fires (a press lands a scan before the program
//! that waits on it reads it, and the step it releases ends there);
//! `device_done` reads the device's own lane
//! (off = arrived, stopped), which for a conveyor stands in for the
//! in-position test the rollout made. A program waits on a step from its
//! span's start to its end, both included — the end is the instant the
//! exit released it, and on a live record the head of the open span.

use super::*;
use crate::iomap::{self, Aspect, IoDerivation, IoPoint, IoSource};

/// How long an edge contact conducts after its edge — the studio's
/// afterglow (`sfc.ts` `EDGE_HOLD`). In the scan an edge holds for one
/// scan of the program that reads it; read back from the record, the
/// edge and the release it caused may sit a scan apart.
pub const EDGE_HOLD: f64 = 0.15;

/// One contact of a step's exit that does not conduct at the instant.
#[derive(Debug, Clone, PartialEq)]
pub struct WaitLeaf {
    /// The contact in PLC vocabulary (the studio's labels): `↑part_at_pick`,
    /// `beam=1`, `0.50s`, `done ▶to_pick`, `conv done`.
    pub condition: String,
    /// `signal`, `rising`, `falling`, `elapsed`, `done`, `robot_done`,
    /// `group_done` or `device_done`.
    pub kind: &'static str,
    /// The lane, device or robot the contact reads.
    pub name: Option<String>,
    /// The signal's level at the instant (signal and edge contacts).
    pub level: Option<bool>,
    /// The seconds a timer still has to run.
    pub remaining: Option<f64>,
    /// The I/O point the program reads through this contact, when the
    /// contact is one (`part_at_pick`, `arm.done`) …
    pub point: Option<String>,
    /// … and the address of its channel, when the point is bound.
    pub address: Option<String>,
}

/// One way out of a step: its transition, or one arm of a branching step.
#[derive(Debug, Clone, PartialEq)]
pub struct WaitExit {
    /// The whole condition in PLC vocabulary.
    pub condition: String,
    /// Whether it conducts at the instant (it fires, or just has).
    pub holds: bool,
    /// The contacts still open — for an `any`, the alternatives.
    pub unmet: Vec<WaitLeaf>,
}

/// A program standing in a step at an instant.
#[derive(Debug, Clone, PartialEq)]
pub struct Waiting {
    pub program: String,
    pub step: String,
    /// The step's flat index (the SFC chart's and the ladder's `S{flat}`).
    pub flat: usize,
    /// When the program entered the step.
    pub since: f64,
    /// Its exits in authored order — first conducting wins.
    pub exits: Vec<WaitExit>,
}

/// One step's stay on the timeline and what ended it.
#[derive(Debug, Clone, PartialEq)]
pub struct WaitSpan {
    pub program: String,
    pub step: String,
    pub flat: usize,
    pub start: f64,
    pub end: f64,
    /// The exit that let the program on, in PLC vocabulary — the taken
    /// arm's guard for a branching step; `None` when nothing let it on
    /// (the run ended in the step, or a live record's open step).
    pub released_by: Option<String>,
}

/// The programs a timeline recorded, flattened as the rollout scans them.
fn programs(scene: &Scene, tl: &SequenceTimeline) -> Vec<(String, Vec<FlatStep>)> {
    tl.sequences
        .iter()
        .filter_map(|name| {
            scene
                .sequence(name)
                .map(|s| (name.clone(), flatten(&s.steps)))
        })
        .collect()
}

/// The I/O points of the recorded programs, for naming a contact's point
/// and address. A scene whose programs cannot be derived (an ambiguous
/// robot reference) simply names none.
fn derivation(scene: &Scene, tl: &SequenceTimeline) -> Option<IoDerivation> {
    let names: Vec<&str> = tl.sequences.iter().map(String::as_str).collect();
    iomap::derive(scene, Some(&names)).ok()
}

/// The span program `program` stands in at `t`: the latest entered at or
/// before `t` (an immediate chain's last), while `t` is within it.
fn active_span<'a>(tl: &'a SequenceTimeline, program: &str, t: f64) -> Option<&'a StepSpan> {
    let latest = tl
        .step_spans
        .iter()
        .filter(|s| s.sequence == program && s.start <= t + 1e-9)
        .fold(None::<&StepSpan>, |best, s| match best {
            Some(b) if b.start > s.start + 1e-12 => Some(b),
            _ => Some(s),
        })?;
    (t <= latest.end + 1e-9).then_some(latest)
}

/// What a contact reads at the instant, against the step's own span.
struct Ctx<'a> {
    tl: &'a SequenceTimeline,
    span: &'a StepSpan,
    /// The program's flat steps (a cylinder's in-position is the switch of
    /// the stop the chain before this step sent it to).
    flat: &'a [FlatStep],
    t: f64,
}

impl Ctx<'_> {
    fn lane(&self, name: &str) -> Option<&BoolTrack> {
        self.tl.signals.iter().find(|s| s.name == name)
    }

    /// An edge conducts only at the instant it fires; startup is not one.
    fn edge(&self, name: &str, rising: bool) -> (bool, Option<bool>) {
        let Some(lane) = self.lane(name) else {
            return (false, Some(false));
        };
        let now = lane.value_at(self.t);
        // The last flank of the right sense at or before now; the first
        // entry is the startup level, not an edge.
        let last = lane
            .edges
            .windows(2)
            .filter(|w| w[1].0 <= self.t + 1e-9 && w[1].1 == rising && w[0].1 != rising)
            .map(|w| w[1].0)
            .next_back();
        let fired = last.is_some_and(|at| self.t - at <= EDGE_HOLD + 1e-9);
        (fired, Some(now))
    }

    /// `(conducts, level, seconds still to run)` of one leaf.
    fn leaf(&self, c: &Condition) -> (bool, Option<bool>, Option<f64>) {
        let t = self.t;
        match c {
            Condition::Immediately => (true, None, None),
            Condition::Signal { name, value } => {
                let level = self.lane(name).map(|l| l.value_at(t)).unwrap_or(false);
                (level == *value, Some(level), None)
            }
            Condition::Rising { name } => {
                let (holds, level) = self.edge(name, true);
                (holds, level, None)
            }
            Condition::Falling { name } => {
                let (holds, level) = self.edge(name, false);
                (holds, level, None)
            }
            Condition::Elapsed { seconds } => {
                let served = t - self.span.start;
                let holds = served + 1e-9 >= *seconds;
                (holds, None, (!holds).then(|| seconds - served))
            }
            // Every move this stay of the step started has ended.
            Condition::Done => {
                let holds = self
                    .tl
                    .robots
                    .iter()
                    .flat_map(|r| r.moves.iter())
                    .filter(|m| {
                        m.sequence == self.span.sequence
                            && m.step == self.span.step
                            && m.start + 1e-9 >= self.span.start
                            && m.start <= self.span.end + 1e-9
                    })
                    .all(|m| m.end <= t + 1e-9);
                (holds, None, None)
            }
            Condition::RobotDone { robot } => {
                let busy = self
                    .tl
                    .robots
                    .iter()
                    .filter(|r| &r.name == robot)
                    .any(|r| r.moves.iter().any(|m| m.start <= t && t < m.end - 1e-9));
                (!busy, None, None)
            }
            Condition::GroupDone { robot, group } => {
                let busy = self.tl.robots.iter().filter(|r| &r.name == robot).any(|r| {
                    r.moves.iter().any(|m| {
                        m.group.as_deref() == Some(group.as_str())
                            && m.start <= t
                            && t < m.end - 1e-9
                    })
                });
                (!busy, None, None)
            }
            Condition::DeviceDone { device } => {
                let holds = self.lane(device).is_some_and(|l| !l.value_at(t));
                (holds, None, None)
            }
            // The step relay, read from the record: the program's latest
            // span at `t` is that step's.
            Condition::StepActive { sequence, step } => {
                let holds = active_span(self.tl, sequence, t)
                    .is_some_and(|s| &s.name == step || s.name == format!("{sequence}/{step}"));
                (holds, None, None)
            }
            Condition::Compare { left, op, right } => {
                let value = |o: &crate::seq::Operand| match o {
                    crate::seq::Operand::Const(x) => *x,
                    crate::seq::Operand::Word(name) => self
                        .tl
                        .words
                        .iter()
                        .find(|w| &w.name == name)
                        .map(|w| w.value_at(t))
                        .unwrap_or(0.0),
                };
                (op.holds(value(left), value(right)), None, None)
            }
            Condition::All(_) | Condition::Any(_) | Condition::Not(_) => {
                unreachable!("trees are walked, not read")
            }
        }
    }

    /// Whether `c` conducts; the open leaves are pushed onto `out` (an
    /// `any` that does not conduct pushes every alternative).
    fn walk(&self, c: &Condition, step: &FlatStep, names: &Names, out: &mut Vec<WaitLeaf>) -> bool {
        match c {
            Condition::All(cs) => {
                let mut holds = true;
                for c in cs {
                    holds &= self.walk(c, step, names, out);
                }
                holds
            }
            Condition::Any(cs) => {
                let mut alternatives = Vec::new();
                let mut holds = false;
                for c in cs {
                    holds |= self.walk(c, step, names, &mut alternatives);
                }
                if !holds {
                    out.extend(alternatives);
                }
                holds
            }
            // A negated network is one contact to wait on: named whole.
            Condition::Not(inner) => {
                let holds = !self.walk(inner, step, names, &mut Vec::new());
                if !holds {
                    let (point, address) = match &**inner {
                        Condition::Signal { .. } => {
                            names.point(&self.span.sequence, self.span.step, self.flat, inner)
                        }
                        _ => (None, None),
                    };
                    out.push(WaitLeaf {
                        condition: condition_label(c),
                        kind: "not",
                        name: subject(inner),
                        level: None,
                        remaining: None,
                        point,
                        address,
                    });
                }
                holds
            }
            leaf => {
                let (holds, level, remaining) = self.leaf(leaf);
                if !holds {
                    let (point, address) =
                        names.point(&self.span.sequence, self.span.step, self.flat, leaf);
                    out.push(WaitLeaf {
                        condition: leaf_label(leaf, step),
                        kind: kind(leaf),
                        name: subject(leaf),
                        level,
                        remaining,
                        point,
                        address,
                    });
                }
                holds
            }
        }
    }
}

/// Names a contact's I/O point: the point the program reads at that step
/// (its readers carry the step), and the bound channel's address.
struct Names(Option<IoDerivation>);

impl Names {
    fn point(
        &self,
        program: &str,
        flat: usize,
        steps: &[FlatStep],
        c: &Condition,
    ) -> (Option<String>, Option<String>) {
        let Some(d) = &self.0 else {
            return (None, None);
        };
        // A coil-driven axis reports in-position through the limit switch of
        // the stop it was last sent to — this step's own command first, then
        // the chain before it (the ST export tracks it the same way).
        let switch = match c {
            Condition::DeviceDone { device } => {
                last_stop(steps, flat, device).map(|s| format!("{device}/{s}"))
            }
            _ => None,
        };
        let reads = |p: &IoPoint| {
            p.readers
                .iter()
                .any(|r| r.sequence == program && r.index == flat)
        };
        let found = d.points.iter().find(|p| {
            reads(p)
                && match c {
                    Condition::Signal { name, .. }
                    | Condition::Rising { name }
                    | Condition::Falling { name } => &p.id.name == name && p.id.aspect.is_none(),
                    Condition::DeviceDone { device } => {
                        (&p.id.name == device && p.source == IoSource::DeviceDone)
                            || switch.as_deref() == Some(p.id.name.as_str())
                    }
                    Condition::RobotDone { robot } | Condition::GroupDone { robot, .. } => {
                        &p.id.name == robot && p.id.aspect == Some(Aspect::Done)
                    }
                    // The step's own moves: the robot's done handshake it reads.
                    Condition::Done => p.source == IoSource::RobotDone,
                    // The word the comparison reads (an analog input's
                    // scaled value and its counts are one point).
                    Condition::Compare { left, right, .. } => [left, right].iter().any(|o| {
                        matches!(o, crate::seq::Operand::Word(word)
                            if &p.label() == word || format!("{}.raw", p.id.name) == *word)
                    }),
                    _ => false,
                }
        });
        match found {
            Some(p) => {
                let address = d.binding_of(p).map(|(b, _, c)| {
                    c.address
                        .clone()
                        .unwrap_or_else(|| format!("{}.{}", b.node, b.channel))
                });
                (Some(p.label()), address)
            }
            None => (None, None),
        }
    }
}

/// The stop a coil-driven axis was last sent to when step `flat` waits on
/// it: the step's own `move_to` first, then the steps before it in flat
/// order.
fn last_stop<'a>(steps: &'a [FlatStep], flat: usize, device: &str) -> Option<&'a str> {
    steps[..=flat.min(steps.len().saturating_sub(1))]
        .iter()
        .rev()
        .flat_map(|s| s.actions.iter().rev())
        .find_map(|a| match a {
            Action::Device {
                device: d,
                command: DeviceCommand::MoveToStop(stop),
            } if d == device => Some(stop.as_str()),
            _ => None,
        })
}

/// A condition in PLC vocabulary — the studio's `conditionLabel`.
pub fn condition_label(c: &Condition) -> String {
    match c {
        Condition::Immediately => "→".to_string(),
        Condition::Done => "done".to_string(),
        Condition::RobotDone { robot } => format!("{robot} done"),
        Condition::GroupDone { robot, group } => format!("{robot}/{group} done"),
        Condition::Elapsed { seconds } => format!("{seconds:.2}s"),
        Condition::Signal { name, value } => format!("{name}={}", if *value { 1 } else { 0 }),
        Condition::Rising { name } => format!("↑{name}"),
        Condition::Falling { name } => format!("↓{name}"),
        Condition::All(cs) => cs
            .iter()
            .map(condition_label)
            .collect::<Vec<_>>()
            .join(" & "),
        Condition::Any(cs) => cs
            .iter()
            .map(condition_label)
            .collect::<Vec<_>>()
            .join(" | "),
        Condition::DeviceDone { device } => format!("{device} done"),
        Condition::Not(c) => match &**c {
            Condition::Signal { name, value } => {
                format!("{name}={}", if *value { 0 } else { 1 })
            }
            other => format!("!({})", condition_label(other)),
        },
        Condition::StepActive { sequence, step } => format!("[{sequence}/{step}]"),
        Condition::Compare { left, op, right } => format!("{left}{}{right}", op.as_str()),
    }
}

/// A leaf's label at its step: `done` names the moves it waits for.
fn leaf_label(c: &Condition, step: &FlatStep) -> String {
    if let Condition::Done = c {
        let moves: Vec<String> = step
            .actions
            .iter()
            .filter_map(|a| match a {
                Action::StartMotion { motion } => Some(format!("▶{motion}")),
                Action::StartToolpath { toolpath, .. } => Some(format!("⟿{toolpath}")),
                Action::StartRamp { .. } => Some("ramp".to_string()),
                Action::Policy { policy, .. } => Some(format!("policy {policy}")),
                _ => None,
            })
            .collect();
        if !moves.is_empty() {
            return format!("done {}", moves.join(" "));
        }
    }
    condition_label(c)
}

fn kind(c: &Condition) -> &'static str {
    match c {
        Condition::Immediately => "immediately",
        Condition::Done => "done",
        Condition::RobotDone { .. } => "robot_done",
        Condition::GroupDone { .. } => "group_done",
        Condition::Elapsed { .. } => "elapsed",
        Condition::Signal { .. } => "signal",
        Condition::Rising { .. } => "rising",
        Condition::Falling { .. } => "falling",
        Condition::All(_) => "all",
        Condition::Any(_) => "any",
        Condition::DeviceDone { .. } => "device_done",
        Condition::Not(_) => "not",
        Condition::StepActive { .. } => "step_active",
        Condition::Compare { .. } => "compare",
    }
}

fn subject(c: &Condition) -> Option<String> {
    match c {
        Condition::Signal { name, .. }
        | Condition::Rising { name }
        | Condition::Falling { name } => Some(name.clone()),
        Condition::RobotDone { robot } | Condition::GroupDone { robot, .. } => Some(robot.clone()),
        Condition::DeviceDone { device } => Some(device.clone()),
        _ => None,
    }
}

/// What every program of `tl` waits on at `t`: the step it stands in and
/// the contacts of each exit that do not conduct. A program that has not
/// started or has ended is left out. `scene` is the scene that baked the
/// timeline (its sequences give the conditions, its I/O map the
/// addresses).
pub fn waiting_at(scene: &Scene, tl: &SequenceTimeline, t: f64) -> Vec<Waiting> {
    let names = Names(derivation(scene, tl));
    let mut out = Vec::new();
    for (program, flat) in programs(scene, tl) {
        let Some(span) = active_span(tl, &program, t) else {
            continue;
        };
        let Some(step) = flat.get(span.step) else {
            continue;
        };
        let cx = Ctx {
            tl,
            span,
            flat: &flat,
            t,
        };
        let exits = step
            .exits
            .iter()
            .map(|(condition, _)| {
                let mut unmet = Vec::new();
                let holds = cx.walk(condition, step, &names, &mut unmet);
                WaitExit {
                    condition: condition_label(condition),
                    holds,
                    unmet,
                }
            })
            .collect();
        out.push(Waiting {
            program,
            step: step.name.clone(),
            flat: span.step,
            since: span.start,
            exits,
        });
    }
    out
}

/// Every step's stay on `tl`, in the timeline's order, with the exit that
/// released it — first conducting exit at the stay's end, the rollout's
/// own first-wins rule. Summed per exit, this is where a cycle's time
/// went.
pub fn wait_spans(scene: &Scene, tl: &SequenceTimeline) -> Vec<WaitSpan> {
    let programs = programs(scene, tl);
    let names = Names(None);
    tl.step_spans
        .iter()
        .filter_map(|span| {
            let (_, flat) = programs.iter().find(|(p, _)| *p == span.sequence)?;
            let step = flat.get(span.step)?;
            let cx = Ctx {
                tl,
                span,
                flat,
                t: span.end,
            };
            let released_by = step
                .exits
                .iter()
                .find(|(condition, _)| cx.walk(condition, step, &names, &mut Vec::new()))
                .map(|(condition, _)| match condition {
                    Condition::Done => leaf_label(condition, step),
                    other => condition_label(other),
                });
            Some(WaitSpan {
                program: span.sequence.clone(),
                step: step.name.clone(),
                flat: span.step,
                start: span.start,
                end: span.end,
                released_by,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use nalgebra::{Isometry3, Point3, Vector3};

    use super::*;
    use crate::iomap::{IoBinding, IoChannel, IoDirection, IoNode, IoNodeKind, IoPointId};
    use crate::rollout::tests::sample_scene;
    use crate::seq::{Device, DeviceCommand, Sensor, SensorKind, SensorWatch, Sequence, Step};
    use crate::Geometry;

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.into(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// A belt feeds a box to a beam; the program then dwells on a timer.
    /// The beam is wired to a PLC input with an address.
    fn cell() -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "box",
                Geometry::Box {
                    size: Vector3::new(0.04, 0.04, 0.04),
                },
                Isometry3::translation(-0.5, 0.5, 0.5),
            )
            .unwrap();
        scene.upsert_device(Device {
            name: "conv".into(),
            kind: DeviceKind::Conveyor {
                zone_pose: Isometry3::translation(-0.2, 0.5, 0.5),
                zone_size: Vector3::new(1.2, 0.3, 0.3),
                velocity: Vector3::new(0.25, 0.0, 0.0),
                running: false,
                encoder: false,
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
                ids: None,
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
                    Condition::Rising {
                        name: "beam".into(),
                    },
                ),
                step(
                    "dwell",
                    vec![Action::Device {
                        device: "conv".into(),
                        command: DeviceCommand::Stop,
                    }],
                    Condition::Elapsed { seconds: 0.5 },
                ),
            ],
        });
        scene
            .upsert_io_node(IoNode {
                name: "PLC1".into(),
                kind: IoNodeKind::Plc,
                programs: vec!["feed".into()],
                uplink: None,
                channels: vec![IoChannel {
                    id: "DI0".into(),
                    kind: iomap::ChannelKind::Di,
                    port: None,
                    address: Some("X20".into()),
                    electrical: None,
                    unit: None,
                }],
                place: None,
                model: None,
                monitor: false,
            })
            .unwrap();
        scene
            .bind_io(IoBinding {
                point: IoPointId {
                    name: "beam".into(),
                    aspect: None,
                    direction: IoDirection::Input,
                },
                node: "PLC1".into(),
                channel: "DI0".into(),
                tag: None,
                field: None,
                invert: false,
                contact: None,
                safety: false,
                device: None,
                note: None,
                auto: false,
            })
            .unwrap();
        scene
    }

    #[test]
    fn a_program_waits_on_the_contacts_that_do_not_conduct() {
        let scene = cell();
        let tl = scene
            .simulate_sequence("feed", &RolloutOptions::default())
            .unwrap();
        let feed = tl
            .step_spans
            .iter()
            .find(|s| s.name == "feed")
            .unwrap()
            .clone();
        let dwell = tl
            .step_spans
            .iter()
            .find(|s| s.name == "dwell")
            .unwrap()
            .clone();

        // Mid-feed: the belt runs, the beam is dark — the rising edge of
        // X20 is what the program waits on.
        let w = waiting_at(&scene, &tl, feed.start + 0.5);
        assert_eq!(w.len(), 1);
        assert_eq!(
            (w[0].program.as_str(), w[0].step.as_str()),
            ("feed", "feed")
        );
        assert_eq!(w[0].since, feed.start);
        let exit = &w[0].exits[0];
        assert_eq!(exit.condition, "↑beam");
        assert!(!exit.holds);
        assert_eq!(exit.unmet.len(), 1);
        let leaf = &exit.unmet[0];
        assert_eq!(leaf.kind, "rising");
        assert_eq!(leaf.name.as_deref(), Some("beam"));
        assert_eq!(leaf.level, Some(false));
        assert_eq!(leaf.point.as_deref(), Some("beam"));
        assert_eq!(leaf.address.as_deref(), Some("X20"));

        // In the dwell: the timer, with what it has left.
        let t = dwell.start + 0.2;
        let w = waiting_at(&scene, &tl, t);
        let leaf = &w[0].exits[0].unmet[0];
        assert_eq!(
            (w[0].step.as_str(), leaf.condition.as_str()),
            ("dwell", "0.50s")
        );
        assert!((leaf.remaining.unwrap() - 0.3).abs() < 1e-9);
        // The instant one stay ends the next has begun: that is where the
        // program stands.
        assert_eq!(waiting_at(&scene, &tl, feed.end)[0].step, "dwell");
        // Past the end the program waits on nothing.
        assert!(waiting_at(&scene, &tl, tl.duration + 1.0).is_empty());
    }

    #[test]
    fn every_stay_names_the_exit_that_released_it() {
        let scene = cell();
        let tl = scene
            .simulate_sequence("feed", &RolloutOptions::default())
            .unwrap();
        let rows = wait_spans(&scene, &tl);
        let got: Vec<(&str, Option<&str>)> = rows
            .iter()
            .map(|r| (r.step.as_str(), r.released_by.as_deref()))
            .collect();
        assert_eq!(got, vec![("feed", Some("↑beam")), ("dwell", Some("0.50s"))]);
        let feed = &rows[0];
        assert!(
            feed.end - feed.start > 1.0,
            "the belt took the box to the beam"
        );
    }
}

/// One analog input's chain at an instant: what its transmitter measures,
/// the loop current, the module's counts, the value the controller scales
/// them to.
#[derive(Debug, Clone, PartialEq)]
pub struct AnalogReading {
    pub name: String,
    pub unit: String,
    /// The word lane measured, and its value now.
    pub source: String,
    pub measured: f64,
    /// Loop current, mA — as the counts imply it (0 on an open loop).
    pub current: f64,
    /// The module's counts (`<name>.raw`).
    pub raw: i64,
    /// The scaled value programs read (`<name>`).
    pub scaled: f64,
    /// The loop is open: the current is under the transmitter's band.
    pub open: bool,
    /// The bound address of the input, when it is bound.
    pub address: Option<String>,
}

/// Every analog input's chain at `t`, in scene order, read from the
/// timeline's word lanes.
pub fn analog_at(scene: &Scene, tl: &SequenceTimeline, t: f64) -> Vec<AnalogReading> {
    let d = derivation(scene, tl);
    let lane = |name: &str| {
        tl.words
            .iter()
            .find(|w| w.name == name)
            .map(|w| w.value_at(t))
    };
    scene
        .analogs()
        .iter()
        .filter_map(|a| {
            let raw = lane(&format!("{}.raw", a.name))? as i64;
            let current = 4.0 + (raw - a.raw.0) as f64 * 16.0 / (a.raw.1 - a.raw.0) as f64;
            let address = d.as_ref().and_then(|d| {
                d.points
                    .iter()
                    .filter(|p| p.id.name == a.name && p.id.aspect.is_none())
                    .find_map(|p| d.binding_of(p))
                    .map(|(b, _, c)| {
                        c.address
                            .clone()
                            .unwrap_or_else(|| format!("{}.{}", b.node, b.channel))
                    })
            });
            Some(AnalogReading {
                name: a.name.clone(),
                unit: a.unit.clone(),
                source: a.source.clone(),
                measured: lane(&a.source).unwrap_or(0.0),
                current,
                raw,
                scaled: lane(&a.name).unwrap_or(0.0),
                open: current < crate::seq::ANALOG_MA.0 - 1e-9,
                address,
            })
        })
        .collect()
}
