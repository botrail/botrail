//! The interlock table: every output the programs switch — a coil, a
//! device command, a robot start, a grasp — and the condition that had to
//! hold for the step that switches it to be entered. Control designers
//! keep this sheet by hand (インターロック表: what may not happen unless
//! what); here it is a projection of the sequences themselves, so it can
//! never disagree with the SFC the PLCopen export carries or with the
//! bake that ran (E4 of design/design-cell-engineering.md).
//!
//! A row is one output of one step. Its condition is the transition that
//! leads *into* the step: the previous step's transition, the arm's
//! condition for the first step of a branch arm, the OR of the arms' last
//! transitions at a rejoin, and — for a program's first step — the last
//! step's transition, since a cyclic program re-enters it from the end
//! (and enters it once at start). The inputs the condition reads are
//! classified (sensor, signal, device lane, device, robot) and, for a
//! signal, traced to the program and step that writes it — which is how a
//! handshake between two controllers reads across the table.

use std::collections::BTreeMap;
use std::fmt::Write as _;

use serde::Serialize;

use crate::iomap::{self, IoDirection, PointStatus};
use crate::seq::{Action, Condition, DeviceCommand, DeviceKind, Sequence, Step};
use crate::Scene;

/// What kind of thing a row switches.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum OutputKind {
    /// An internal signal written (`Set`).
    Signal,
    /// A device command (start / stop / move).
    Device,
    /// A robot motion started.
    Motion,
    /// A joint ramp started (a gripper).
    Ramp,
    /// A toolpath started.
    Toolpath,
    /// A grasp (attach) or a conveyor track.
    Grasp,
    /// A release (detach) or the end of a track.
    Release,
}

impl OutputKind {
    pub fn as_str(self) -> &'static str {
        match self {
            OutputKind::Signal => "signal",
            OutputKind::Device => "device",
            OutputKind::Motion => "motion",
            OutputKind::Ramp => "ramp",
            OutputKind::Toolpath => "toolpath",
            OutputKind::Grasp => "grasp",
            OutputKind::Release => "release",
        }
    }
}

/// What kind of thing a condition reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum InputKind {
    /// A sensor lane (a zone, a beam, a vision or field sensor).
    Sensor,
    /// An internal signal (written by a program).
    Signal,
    /// A device's end-of-travel lane (`<axis>/<stop>`).
    DeviceLane,
    /// A device's in-position (`DeviceDone`).
    Device,
    /// A robot's idle test (`RobotDone`).
    Robot,
    /// A name the scene does not know (a signal defined nowhere).
    Unknown,
}

impl InputKind {
    pub fn as_str(self) -> &'static str {
        match self {
            InputKind::Sensor => "sensor",
            InputKind::Signal => "signal",
            InputKind::DeviceLane => "device lane",
            InputKind::Device => "device",
            InputKind::Robot => "robot",
            InputKind::Unknown => "unknown",
        }
    }
}

/// One thing a condition reads.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct InputRef {
    pub name: String,
    pub kind: InputKind,
    /// `program/step` of every writer of a signal (or every commander of
    /// a device); empty for field inputs.
    pub written_by: Vec<String>,
    /// `node.channel [address]` where the row's host has the point bound.
    pub address: Option<String>,
}

/// One row: an output of a step and the condition guarding the step.
#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct InterlockRow {
    pub program: String,
    /// The controller the program runs on (`<cell>` when implicit).
    pub host: String,
    pub step: String,
    pub kind: OutputKind,
    /// The thing switched: the signal, device, motion, or object name.
    pub target: String,
    /// The output as text: `vmc/running := TRUE`, `vmc/side_door → open`,
    /// `motion enter (arm)`.
    pub output: String,
    /// The entry condition, as an ST-like expression over scene names.
    pub condition: String,
    /// The steps whose transition the condition is (the predecessors).
    pub after: Vec<String>,
    pub inputs: Vec<InputRef>,
}

#[derive(Debug, Clone, PartialEq, Serialize)]
pub struct InterlockTable {
    pub sequences: Vec<String>,
    pub rows: Vec<InterlockRow>,
    /// Why hosts and addresses are missing, when the I/O map could not be
    /// derived (the table itself does not need it).
    pub io_error: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum InterlockError {
    UnknownSequence(String),
    NoSequences,
}

impl std::fmt::Display for InterlockError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            InterlockError::UnknownSequence(n) => write!(f, "unknown sequence `{n}`"),
            InterlockError::NoSequences => write!(f, "the scene has no sequences"),
        }
    }
}

impl std::error::Error for InterlockError {}

/// The entry condition of a step, as the transitions it follows.
struct Entry<'a> {
    conditions: Vec<(&'a Condition, &'a Step)>,
    /// `true` for a program's first step: entered at start as well.
    start: bool,
}

/// Conditions as ST-like text over scene names. `after` is the step whose
/// transition this is (its motions name what `DONE` waits for).
fn condition_text(c: &Condition, after: Option<&Step>) -> String {
    match c {
        Condition::Immediately => "TRUE".to_string(),
        Condition::Done => {
            let started: Vec<String> = after
                .map(|s| {
                    s.actions
                        .iter()
                        .filter_map(|a| match a {
                            Action::StartMotion { motion } => Some(motion.clone()),
                            Action::StartRamp { robot, .. } => Some(format!(
                                "ramp{}",
                                robot.as_ref().map(|r| format!(" {r}")).unwrap_or_default()
                            )),
                            Action::StartToolpath { toolpath, .. } => Some(toolpath.clone()),
                            _ => None,
                        })
                        .collect()
                })
                .unwrap_or_default();
            if started.is_empty() {
                "DONE".to_string()
            } else {
                format!("DONE({})", started.join(", "))
            }
        }
        Condition::RobotDone { robot } => format!("IDLE({robot})"),
        Condition::GroupDone { robot, group } => format!("IDLE({robot}/{group})"),
        Condition::Elapsed { seconds } => format!("T >= {} s", trim_num(*seconds)),
        Condition::Signal { name, value } => {
            if *value {
                name.clone()
            } else {
                format!("NOT {name}")
            }
        }
        Condition::Rising { name } => format!("RISING({name})"),
        Condition::Falling { name } => format!("FALLING({name})"),
        Condition::DeviceDone { device } => format!("INPOS({device})"),
        Condition::All(cs) => join(cs, " AND ", after),
        Condition::Any(cs) => join(cs, " OR ", after),
    }
}

fn join(cs: &[Condition], op: &str, after: Option<&Step>) -> String {
    let parts: Vec<String> = cs.iter().map(|c| condition_text(c, after)).collect();
    match parts.len() {
        0 => "TRUE".to_string(),
        1 => parts[0].clone(),
        _ => format!("({})", parts.join(op)),
    }
}

fn trim_num(v: f64) -> String {
    let s = format!("{v:.3}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    if s.is_empty() {
        "0".to_string()
    } else {
        s.to_string()
    }
}

/// Every lane, device or robot a condition reads, in order, deduplicated.
fn condition_inputs(c: &Condition, out: &mut Vec<(String, bool, bool)>) {
    // (name, is_device_done, is_robot_done)
    let push = |out: &mut Vec<(String, bool, bool)>, item: (String, bool, bool)| {
        if !out.contains(&item) {
            out.push(item);
        }
    };
    match c {
        Condition::Signal { name, .. }
        | Condition::Rising { name }
        | Condition::Falling { name } => push(out, (name.clone(), false, false)),
        Condition::DeviceDone { device } => push(out, (device.clone(), true, false)),
        Condition::RobotDone { robot } | Condition::GroupDone { robot, .. } => {
            push(out, (robot.clone(), false, true))
        }
        Condition::All(cs) | Condition::Any(cs) => {
            for c in cs {
                condition_inputs(c, out);
            }
        }
        Condition::Immediately | Condition::Done | Condition::Elapsed { .. } => {}
    }
}

/// The last transition(s) of a run of steps: the last step's own
/// transition, or — when the last step is a select — the rejoin of its
/// arms (each arm's last transition, or the arm condition for an empty
/// arm).
fn exits<'a>(steps: &'a [Step], out: &mut Vec<(&'a Condition, &'a Step)>) {
    let Some(last) = steps.last() else {
        return;
    };
    if last.select.is_empty() {
        out.push((&last.transition, last));
    } else {
        for arm in &last.select {
            if arm.steps.is_empty() {
                out.push((&arm.condition, last));
            } else {
                exits(&arm.steps, out);
            }
        }
    }
}

fn walk<'a>(
    steps: &'a [Step],
    entry: Vec<(&'a Condition, &'a Step)>,
    start: bool,
    out: &mut Vec<(&'a Step, Entry<'a>)>,
) {
    let mut prev: Vec<(&'a Condition, &'a Step)> = entry;
    for (i, step) in steps.iter().enumerate() {
        out.push((
            step,
            Entry {
                conditions: prev.clone(),
                start: start && i == 0,
            },
        ));
        if step.select.is_empty() {
            prev = vec![(&step.transition, step)];
        } else {
            for arm in &step.select {
                walk(&arm.steps, vec![(&arm.condition, step)], false, out);
            }
            let mut next = Vec::new();
            exits(std::slice::from_ref(step), &mut next);
            prev = next;
        }
    }
}

fn outputs(scene: &Scene, step: &Step) -> Vec<(OutputKind, String, String)> {
    // A dual-arm robot's rows say which arm drives: `robot/arm`. A
    // single-arm robot is just its name.
    let arm_label = |robot: usize, group: Option<&str>| -> String {
        let r = &scene.robots()[robot];
        match group {
            Some(g) if r.model.groups().len() > 1 => format!("{}/{}", r.name, g),
            _ => r.name.clone(),
        }
    };
    let robot_of_motion = |motion: &str| -> Option<String> {
        scene
            .motions()
            .iter()
            .find(|m| m.name == motion)
            .filter(|m| m.robot < scene.robots().len())
            .map(|m| {
                let group = m
                    .group
                    .and_then(|g| scene.robots()[m.robot].model.groups().into_iter().nth(g))
                    .map(|g| g.name);
                arm_label(m.robot, group.as_deref())
            })
    };
    let addressed = |robot: &Option<String>| -> Option<usize> {
        match robot {
            Some(name) => scene.robot_index(name),
            None => (scene.robots().len() == 1).then_some(0),
        }
    };
    let with_arm = |robot: &Option<String>, group: Option<&str>| -> String {
        match addressed(robot) {
            Some(i) => format!(" ({})", arm_label(i, group)),
            None => robot
                .as_ref()
                .map(|r| format!(" ({r})"))
                .unwrap_or_default(),
        }
    };
    // A ramp's arm: the one every target joint belongs to.
    let ramp_arm =
        |robot: &Option<String>, joints: &[&str]| scene.ramp_arm(addressed(robot)?, joints);
    let with_robot = |robot: &Option<String>| with_arm(robot, None);
    let mut out = Vec::new();
    for action in &step.actions {
        match action {
            Action::Set { signal, value } => out.push((
                OutputKind::Signal,
                signal.clone(),
                format!("{signal} := {}", if *value { "TRUE" } else { "FALSE" }),
            )),
            Action::Device { device, command } => {
                let text = match command {
                    DeviceCommand::Start => format!("{device}: start"),
                    DeviceCommand::Stop => format!("{device}: stop"),
                    DeviceCommand::SetSpeed(v) => format!("{device}: speed {}", trim_num(*v)),
                    DeviceCommand::MoveTo(x) => format!("{device} → {}", trim_num(*x)),
                    DeviceCommand::Goto { station } => format!("{device} → {station}"),
                    DeviceCommand::Advance(d) => format!("{device}: advance {}", trim_num(*d)),
                    DeviceCommand::MoveToStop(stop) => format!("{device} → {stop}"),
                };
                out.push((OutputKind::Device, device.clone(), text));
            }
            Action::StartMotion { motion } => {
                let robot = robot_of_motion(motion)
                    .map(|r| format!(" ({r})"))
                    .unwrap_or_default();
                out.push((
                    OutputKind::Motion,
                    motion.clone(),
                    format!("motion {motion}{robot}"),
                ));
            }
            Action::StartRamp { robot, targets, .. } => {
                let joints: Vec<String> = targets
                    .iter()
                    .map(|(j, v)| format!("{j} → {}", trim_num(*v)))
                    .collect();
                let names: Vec<&str> = targets.iter().map(|(j, _)| j.as_str()).collect();
                let arm = ramp_arm(robot, &names);
                out.push((
                    OutputKind::Ramp,
                    targets
                        .iter()
                        .map(|(j, _)| j.clone())
                        .collect::<Vec<_>>()
                        .join(", "),
                    format!(
                        "ramp {}{}",
                        joints.join(", "),
                        with_arm(robot, arm.as_deref())
                    ),
                ));
            }
            Action::StartToolpath { robot, toolpath } => out.push((
                OutputKind::Toolpath,
                toolpath.clone(),
                format!("toolpath {toolpath}{}", with_robot(robot)),
            )),
            Action::Attach {
                robot,
                object,
                group,
                ..
            } => out.push((
                OutputKind::Grasp,
                object.clone(),
                format!("attach {object}{}", with_arm(robot, group.as_deref())),
            )),
            Action::Track {
                robot,
                object,
                group,
                ..
            } => out.push((
                OutputKind::Grasp,
                object.clone(),
                format!("track {object}{}", with_arm(robot, group.as_deref())),
            )),
            Action::Detach { object } => out.push((
                OutputKind::Release,
                object.clone(),
                format!("detach {object}"),
            )),
            Action::Untrack { robot, group } => out.push((
                OutputKind::Release,
                robot.clone().unwrap_or_default(),
                format!("untrack{}", with_arm(robot, group.as_deref())),
            )),
        }
    }
    out
}

impl Scene {
    /// The interlock table over `sequences` (every sequence when `None`):
    /// one row per output per step, with the condition that admits the
    /// step (see the module docs).
    pub fn interlock_table(
        &self,
        sequences: Option<&[&str]>,
    ) -> Result<InterlockTable, InterlockError> {
        let programs: Vec<&Sequence> = match sequences {
            Some(list) => {
                let mut out = Vec::new();
                for name in list {
                    let seq = self
                        .sequences()
                        .iter()
                        .find(|s| &s.name == name)
                        .ok_or_else(|| InterlockError::UnknownSequence(name.to_string()))?;
                    out.push(seq);
                }
                out
            }
            None => self.sequences().iter().collect(),
        };
        if programs.is_empty() {
            return Err(InterlockError::NoSequences);
        }
        let names: Vec<&str> = programs.iter().map(|s| s.name.as_str()).collect();

        // Hosts and bound addresses come from the I/O derivation when it
        // holds together; the table stands without it.
        let (derivation, io_error) = match iomap::derive(self, Some(&names)) {
            Ok(d) => (Some(d), None),
            Err(e) => (None, Some(e.to_string())),
        };
        let host_of = |program: &str| -> String {
            derivation
                .as_ref()
                .and_then(|d| {
                    d.program_hosts
                        .iter()
                        .find(|(p, _)| p == program)
                        .map(|(_, h)| h.clone())
                })
                .unwrap_or_else(|| iomap::CELL_HOST.to_string())
        };
        let address_of = |name: &str, host: &str| -> Option<String> {
            let d = derivation.as_ref()?;
            let p = d.points.iter().find(|p| {
                p.id.name == name
                    && p.id.aspect.is_none()
                    && p.id.direction == IoDirection::Input
                    && p.host.as_deref() == Some(host)
            })?;
            let PointStatus::Bound(i) = p.status else {
                return None;
            };
            let b = d.io.bindings.get(i)?;
            let address =
                d.io.node(&b.node)
                    .and_then(|n| n.channels.iter().find(|c| c.id == b.channel))
                    .and_then(|c| c.address.clone());
            Some(match address {
                Some(a) => format!("{}.{} [{a}]", b.node, b.channel),
                None => format!("{}.{}", b.node, b.channel),
            })
        };

        // Writers over the program set: who sets a signal, who commands a
        // device — by `program/step`, in authoring order.
        let mut writers: BTreeMap<String, Vec<String>> = BTreeMap::new();
        for seq in &programs {
            let mut flat: Vec<(&Step, Entry<'_>)> = Vec::new();
            walk(&seq.steps, Vec::new(), true, &mut flat);
            for (step, _) in flat {
                for action in &step.actions {
                    let key = match action {
                        Action::Set { signal, .. } => Some(signal.clone()),
                        Action::Device { device, .. } => Some(device.clone()),
                        _ => None,
                    };
                    if let Some(key) = key {
                        let entry = writers.entry(key).or_default();
                        let label = format!("{}/{}", seq.name, step.name);
                        if !entry.contains(&label) {
                            entry.push(label);
                        }
                    }
                }
            }
        }
        let is_sensor = |name: &str| self.sensors().iter().any(|s| s.name == name);
        let is_signal = |name: &str| self.signals().iter().any(|s| s.name == name);
        let is_stop_lane = |name: &str| {
            self.devices().iter().any(|d| {
                matches!(&d.kind, DeviceKind::LinearAxis { stops, .. }
                    if stops.iter().any(|(stop, _)| format!("{}/{}", d.name, stop) == name))
                    || matches!(&d.kind, DeviceKind::Lift { stops, .. }
                    if stops.iter().any(|(stop, _)| format!("{}/{}", d.name, stop) == name))
            })
        };
        let device_of_lane = |name: &str| -> Option<String> {
            self.devices()
                .iter()
                .find(|d| name.starts_with(&format!("{}/", d.name)))
                .map(|d| d.name.clone())
        };

        let mut rows = Vec::new();
        for seq in &programs {
            let host = host_of(&seq.name);
            let mut flat: Vec<(&Step, Entry<'_>)> = Vec::new();
            walk(&seq.steps, Vec::new(), true, &mut flat);
            // A cyclic program re-enters its first step from its last
            // transition.
            let mut cycle: Vec<(&Condition, &Step)> = Vec::new();
            exits(&seq.steps, &mut cycle);
            for (step, entry) in flat {
                let outs = outputs(self, step);
                if outs.is_empty() {
                    continue;
                }
                let mut conditions = entry.conditions.clone();
                if entry.start {
                    conditions.extend(cycle.iter().cloned());
                }
                let texts: Vec<String> = conditions
                    .iter()
                    .map(|(c, after)| condition_text(c, Some(after)))
                    .collect();
                let mut condition = match texts.len() {
                    0 => "TRUE".to_string(),
                    1 => texts[0].clone(),
                    _ => texts.join(" OR "),
                };
                if entry.start {
                    condition = if conditions.is_empty() {
                        "START".to_string()
                    } else {
                        format!("START OR {condition}")
                    };
                }
                let mut after: Vec<String> = Vec::new();
                for (_, s) in &conditions {
                    if !after.contains(&s.name) {
                        after.push(s.name.clone());
                    }
                }
                let mut read: Vec<(String, bool, bool)> = Vec::new();
                for (c, _) in &conditions {
                    condition_inputs(c, &mut read);
                }
                let inputs: Vec<InputRef> = read
                    .into_iter()
                    .map(|(name, device_done, robot_done)| {
                        let (kind, written_by, address) = if robot_done {
                            (InputKind::Robot, Vec::new(), None)
                        } else if device_done {
                            (
                                InputKind::Device,
                                writers.get(&name).cloned().unwrap_or_default(),
                                None,
                            )
                        } else if is_sensor(&name) {
                            (InputKind::Sensor, Vec::new(), address_of(&name, &host))
                        } else if is_stop_lane(&name) {
                            (
                                InputKind::DeviceLane,
                                device_of_lane(&name)
                                    .and_then(|d| writers.get(&d).cloned())
                                    .unwrap_or_default(),
                                address_of(&name, &host),
                            )
                        } else if is_signal(&name) {
                            (
                                InputKind::Signal,
                                writers.get(&name).cloned().unwrap_or_default(),
                                address_of(&name, &host),
                            )
                        } else {
                            (InputKind::Unknown, Vec::new(), None)
                        };
                        InputRef {
                            name,
                            kind,
                            written_by,
                            address,
                        }
                    })
                    .collect();
                for (kind, target, output) in outs {
                    rows.push(InterlockRow {
                        program: seq.name.clone(),
                        host: host.clone(),
                        step: step.name.clone(),
                        kind,
                        target,
                        output,
                        condition: condition.clone(),
                        after: after.clone(),
                        inputs: inputs.clone(),
                    });
                }
            }
        }
        Ok(InterlockTable {
            sequences: names.iter().map(|s| s.to_string()).collect(),
            rows,
            io_error,
        })
    }
}

fn md_cell(text: &str) -> String {
    text.replace('|', "\\|").replace('\n', " ")
}

fn csv_cell(text: &str) -> String {
    if text.contains([',', '"', '\n']) {
        format!("\"{}\"", text.replace('"', "\"\""))
    } else {
        text.to_string()
    }
}

impl InputRef {
    fn describe(&self) -> String {
        let mut s = format!("{} ({}", self.name, self.kind.as_str());
        if !self.written_by.is_empty() {
            let _ = write!(s, "; written by {}", self.written_by.join(", "));
        }
        if let Some(a) = &self.address {
            let _ = write!(s, "; {a}");
        }
        s.push(')');
        s
    }
}

impl InterlockTable {
    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).expect("interlock table serializes")
    }

    /// The table as Markdown: one section per program, a row per output.
    pub fn to_markdown(&self) -> String {
        let mut out = String::new();
        let _ = writeln!(out, "# Interlock table — {}\n", self.sequences.join(", "));
        out.push_str(
            "Each row is an output a step switches and the condition that admits the step — \
             the previous step's transition, an arm's condition, or the cycle's last transition \
             for a first step (`START OR …`). Inputs name what the condition reads; a signal \
             carries the program and step that writes it.\n",
        );
        if let Some(err) = &self.io_error {
            let _ = writeln!(out, "\nHosts and addresses not derived: {err}\n");
        }
        let mut by_program: Vec<(&str, &str)> = Vec::new();
        for r in &self.rows {
            if !by_program.iter().any(|(p, _)| *p == r.program) {
                by_program.push((&r.program, &r.host));
            }
        }
        for (program, host) in by_program {
            let _ = writeln!(out, "\n## `{program}` on {host}\n");
            out.push_str("| step | output | condition | after | inputs |\n|---|---|---|---|---|\n");
            for r in self.rows.iter().filter(|r| r.program == program) {
                let inputs: Vec<String> = r.inputs.iter().map(|i| i.describe()).collect();
                let _ = writeln!(
                    out,
                    "| {} | {} `{}` | `{}` | {} | {} |",
                    md_cell(&r.step),
                    r.kind.as_str(),
                    md_cell(&r.output),
                    md_cell(&r.condition),
                    md_cell(&r.after.join(", ")),
                    md_cell(&inputs.join("; ")),
                );
            }
        }
        out
    }

    /// The table as CSV, one row per output.
    pub fn to_csv(&self) -> String {
        let mut out = String::from("program,host,step,kind,target,output,condition,after,inputs\n");
        for r in &self.rows {
            let inputs: Vec<String> = r.inputs.iter().map(|i| i.describe()).collect();
            let cells = [
                r.program.as_str(),
                r.host.as_str(),
                r.step.as_str(),
                r.kind.as_str(),
                r.target.as_str(),
                r.output.as_str(),
                r.condition.as_str(),
                &r.after.join("; "),
                &inputs.join("; "),
            ];
            let line: Vec<String> = cells.iter().map(|c| csv_cell(c)).collect();
            out.push_str(&line.join(","));
            out.push('\n');
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::{SelectArm, Sensor, SensorKind, SensorWatch, Sequence, Step};
    use botrail_model::RobotModel;
    use std::sync::Arc;

    const URDF: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="l1"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>"#;

    fn arm() -> Scene {
        Scene::new(Arc::new(RobotModel::from_urdf_str(URDF).unwrap()))
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    fn sig(name: &str) -> Condition {
        Condition::Signal {
            name: name.to_string(),
            value: true,
        }
    }

    #[test]
    fn rows_carry_the_entry_condition_and_trace_signal_writers() {
        let mut scene = arm();
        scene.define_signal("go", false);
        scene.define_signal("done", false);
        scene
            .upsert_sensor(Sensor {
                name: "mat".into(),
                kind: SensorKind::Zone {
                    pose: nalgebra::Isometry3::translation(0.5, 0.0, 0.1),
                    size: nalgebra::Vector3::new(0.2, 0.2, 0.2),
                },
                watch: SensorWatch::All,
                mount: None,
            })
            .unwrap();
        // The cell's program raises `go` once the mat reads clear; the
        // robot's program waits for `go` and reports `done`.
        scene.set_sequences(vec![
            Sequence {
                name: "cell".to_string(),
                steps: vec![
                    step(
                        "wait_clear",
                        vec![],
                        Condition::Signal {
                            name: "mat".to_string(),
                            value: false,
                        },
                    ),
                    step(
                        "release",
                        vec![Action::Set {
                            signal: "go".to_string(),
                            value: true,
                        }],
                        sig("done"),
                    ),
                ],
            },
            Sequence {
                name: "robot".to_string(),
                steps: vec![
                    step("idle", vec![], sig("go")),
                    step(
                        "report",
                        vec![Action::Set {
                            signal: "done".to_string(),
                            value: true,
                        }],
                        Condition::Elapsed { seconds: 0.5 },
                    ),
                ],
            },
        ]);
        let table = scene.interlock_table(None).unwrap();
        let rows: Vec<&InterlockRow> = table.rows.iter().collect();
        assert_eq!(rows.len(), 2);
        // `go` is set when the mat reads clear; the mat is a sensor.
        let go = rows.iter().find(|r| r.target == "go").unwrap();
        assert_eq!((go.program.as_str(), go.step.as_str()), ("cell", "release"));
        assert_eq!(go.condition, "NOT mat");
        assert_eq!(go.after, vec!["wait_clear".to_string()]);
        assert_eq!(go.inputs[0].kind, InputKind::Sensor);
        // `done` is set after `go`, which `cell/release` writes.
        let done = rows.iter().find(|r| r.target == "done").unwrap();
        assert_eq!(done.condition, "go");
        assert_eq!(done.inputs[0].kind, InputKind::Signal);
        assert_eq!(done.inputs[0].written_by, vec!["cell/release".to_string()]);
        assert_eq!(done.output, "done := TRUE");
        let md = table.to_markdown();
        assert!(md.contains("| release | signal `go := TRUE` | `NOT mat` | wait_clear |"));
        assert!(table
            .to_csv()
            .starts_with("program,host,step,kind,target,output,condition,after,inputs\n"));
    }

    #[test]
    fn first_steps_branches_and_rejoins_read_the_right_transitions() {
        let mut scene = arm();
        scene.define_signal("ok", false);
        scene.define_signal("run", false);
        scene.define_signal("good", false);
        let mut check = step("check", vec![], Condition::Immediately);
        check.select = vec![
            SelectArm {
                condition: sig("ok"),
                steps: vec![step(
                    "pass",
                    vec![Action::Set {
                        signal: "good".to_string(),
                        value: true,
                    }],
                    Condition::Elapsed { seconds: 1.0 },
                )],
            },
            SelectArm {
                condition: Condition::Immediately,
                steps: vec![],
            },
        ];
        scene.set_sequences(vec![Sequence {
            name: "p".to_string(),
            steps: vec![
                step(
                    "start",
                    vec![Action::Set {
                        signal: "run".to_string(),
                        value: true,
                    }],
                    Condition::Elapsed { seconds: 2.0 },
                ),
                check,
                step(
                    "end",
                    vec![Action::Set {
                        signal: "run".to_string(),
                        value: false,
                    }],
                    Condition::Elapsed { seconds: 3.0 },
                ),
            ],
        }]);
        let table = scene.interlock_table(Some(&["p"])).unwrap();
        let by_step = |name: &str| table.rows.iter().find(|r| r.step == name).unwrap();
        // The first step: at start, and again from the last transition.
        assert_eq!(by_step("start").condition, "START OR T >= 3 s");
        assert_eq!(by_step("start").after, vec!["end".to_string()]);
        // Inside an arm: the arm's condition, after the select step.
        assert_eq!(by_step("pass").condition, "ok");
        assert_eq!(by_step("pass").after, vec!["check".to_string()]);
        // The rejoin: either arm's exit — the step's timer, or the empty
        // arm's own condition.
        assert_eq!(by_step("end").condition, "T >= 1 s OR TRUE");
        assert_eq!(
            by_step("end").after,
            vec!["pass".to_string(), "check".to_string()]
        );
        assert_eq!(
            scene.interlock_table(Some(&["nope"])).unwrap_err(),
            InterlockError::UnknownSequence("nope".to_string())
        );
    }

    #[test]
    fn a_dual_arm_robot_names_the_arm_on_its_rows() {
        use crate::motion::{Segment, SegmentKind};
        let urdf = include_str!("../../../examples/assets/dual_arm_test.urdf");
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(urdf).unwrap()));
        let robot = scene.robots()[0].name.clone();
        scene
            .add_segment_in_group(
                0,
                Some("left"),
                "left_reach",
                Segment {
                    kind: SegmentKind::Joint,
                    goal_positions: scene.joint_positions().to_vec(),
                    constraints: vec![],
                },
            )
            .unwrap();
        scene.set_sequences(vec![Sequence {
            name: "kit".into(),
            steps: vec![
                step(
                    "reach",
                    vec![
                        Action::StartMotion {
                            motion: "left_reach".into(),
                        },
                        Action::StartRamp {
                            robot: None,
                            targets: vec![("right_finger".into(), 0.5)],
                            duration: 0.2,
                        },
                    ],
                    Condition::GroupDone {
                        robot: robot.clone(),
                        group: "left".into(),
                    },
                ),
                step(
                    "grip",
                    vec![Action::Attach {
                        robot: None,
                        object: "part".into(),
                        link: None,
                        touch_links: None,
                        group: Some("right".into()),
                    }],
                    Condition::Immediately,
                ),
            ],
        }]);
        let md = scene.interlock_table(None).unwrap().to_markdown();
        assert!(
            md.contains(&format!("motion left_reach ({robot}/left)")),
            "{md}"
        );
        assert!(
            md.contains(&format!("ramp right_finger → 0.5 ({robot}/right)")),
            "{md}"
        );
        assert!(md.contains(&format!("attach part ({robot}/right)")), "{md}");
        assert!(md.contains(&format!("IDLE({robot}/left)")), "{md}");
    }
}
