//! PLCopen XML (IEC 61131-10, TC6 XML v2.01) export of the sequences —
//! the cell's control logic as SFC programs a PLC IDE (Beremiz, OpenPLC
//! Editor, CODESYS with its import) opens.
//!
//! The mapping is the one [`crate::seq`] was designed against (see
//! DESIGN.md §4.3): a botrail step is an SFC `<step>` with an
//! `<actionBlock>` for its entry actions and a `<transition>` for its
//! condition; `select` is a `selectionDivergence` whose arms rejoin at a
//! `selectionConvergence`; the last step jumps back to the first (the
//! cell cycles). Conditions become ST expressions (`AND` / `OR` / `NOT`,
//! `Step.T >= T#…`, `R_TRIG` / `F_TRIG` for edges), device coils and
//! commands become assignments to global variables, and robot commands
//! become calls to **stub function blocks** (`FB_StartMotion`, `FB_Attach`,
//! …) that the control engineer replaces with the real controller
//! interface — or, where the I/O map says the robot is driven from another
//! host, the start / done handshake pair as boolean actions and inputs.
//!
//! Variables are the points of the derived I/O map ([`crate::iomap`]),
//! declared once as resource globals — with the `AT` address of their
//! binding when it lands on a PLC-family node — so the program, the I/O
//! list and the robot script all come from the same source and use the
//! same names. Nothing here is simulated: this is the *authored* logic
//! handed over, the complement of the USD that carries the bake.
//!
//! What it is not: a full ST program, a vendor project, or a claim of
//! compliance. It is a standard file, valid against the TC6 schema, with
//! the cell's logic in it and stubs where the machine begins.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use crate::iomap::{self, Aspect, ChannelKind, IoNodeKind, IoSource, PointStatus};
use crate::seq::{Action, Condition, DeviceCommand, Step};
use crate::Scene;

/// What to export and how to package it.
#[derive(Debug, Clone, PartialEq)]
pub struct PlcopenOptions {
    /// Programs to export (`None` = every sequence, in scene order).
    pub sequences: Option<Vec<String>>,
    /// Project / configuration name.
    pub name: String,
    /// Cyclic task interval, milliseconds.
    pub task_interval_ms: u32,
    /// Jump back to the first step after the last one (a production PLC
    /// cycles); `false` parks the program in a final step.
    pub cycle: bool,
}

impl Default for PlcopenOptions {
    fn default() -> Self {
        PlcopenOptions {
            sequences: None,
            name: "cell".to_string(),
            task_interval_ms: 10,
            cycle: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum PlcopenError {
    UnknownSequence(String),
    Derivation(String),
    NoSequences,
}

impl std::fmt::Display for PlcopenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PlcopenError::UnknownSequence(name) => write!(f, "unknown sequence `{name}`"),
            PlcopenError::Derivation(message) => write!(f, "I/O derivation failed: {message}"),
            PlcopenError::NoSequences => write!(f, "the scene has no sequences to export"),
        }
    }
}

impl std::error::Error for PlcopenError {}

// ------------------------------------------------------------ identifiers

const RESERVED: &[&str] = &[
    "action",
    "and",
    "array",
    "at",
    "by",
    "case",
    "constant",
    "do",
    "else",
    "elsif",
    "end",
    "exit",
    "false",
    "for",
    "function",
    "if",
    "mod",
    "not",
    "of",
    "or",
    "program",
    "repeat",
    "resource",
    "retain",
    "return",
    "step",
    "struct",
    "task",
    "then",
    "to",
    "transition",
    "true",
    "type",
    "until",
    "var",
    "while",
    "with",
    "xor",
];

/// An IEC identifier from any name: letters, digits and `_`, not starting
/// with a digit, not a keyword.
pub fn ident(name: &str) -> String {
    let mut out = String::new();
    let mut last_underscore = false;
    for c in name.chars() {
        let ok = c.is_ascii_alphanumeric();
        if ok {
            out.push(c);
            last_underscore = false;
        } else if !last_underscore {
            out.push('_');
            last_underscore = true;
        }
    }
    let mut out = out.trim_matches('_').to_string();
    if out.is_empty() {
        out = "x".to_string();
    }
    if out.chars().next().is_some_and(|c| c.is_ascii_digit()) {
        out.insert(0, '_');
    }
    if RESERVED.contains(&out.to_ascii_lowercase().as_str()) {
        out.push('_');
    }
    out
}

fn st_string(text: &str) -> String {
    format!("'{}'", text.replace('$', "$$").replace('\'', "$'"))
}

fn xml_attr(text: &str) -> String {
    text.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

fn cdata(text: &str) -> String {
    format!("<![CDATA[{}]]>", text.replace("]]>", "]]]]><![CDATA[>"))
}

fn st_body(text: &str) -> String {
    format!("<ST><xhtml:p>{}</xhtml:p></ST>", cdata(text))
}

fn st_time(seconds: f64) -> String {
    let ms = (seconds * 1000.0).round().max(0.0) as u64;
    if ms.is_multiple_of(1000) {
        format!("T#{}s", ms / 1000)
    } else {
        format!("T#{ms}ms")
    }
}

fn st_real(v: f64) -> String {
    if v.fract() == 0.0 && v.abs() < 1e12 {
        format!("{:.1}", v)
    } else {
        let s = format!("{v:.6}");
        s.trim_end_matches('0').to_string()
    }
}

// --------------------------------------------------------------- variables

#[derive(Debug, Clone, PartialEq)]
struct GlobalVar {
    name: String,
    /// IEC type element name (`BOOL`, `REAL`, `INT`).
    ty: &'static str,
    address: Option<String>,
    initial: Option<String>,
    doc: String,
}

/// The variable a derived point is known by in the program.
fn point_var(p: &iomap::IoPoint) -> String {
    let base = ident(&p.id.name);
    match p.source {
        IoSource::DeviceRun => format!("{base}_run"),
        IoSource::DeviceDone => format!("{base}_done"),
        IoSource::DeviceCommand => format!(
            "{base}_{}",
            p.id.aspect.map(|a| a.as_str()).unwrap_or("cmd")
        ),
        IoSource::RobotStart => format!("{base}_start"),
        IoSource::RobotDone => format!("{base}_done"),
        IoSource::RobotProgram => format!("{base}_program"),
        _ => base,
    }
}

fn point_type(p: &iomap::IoPoint) -> &'static str {
    match (p.kind, p.id.aspect) {
        (_, Some(Aspect::Position)) | (_, Some(Aspect::Speed)) => "REAL",
        (_, Some(Aspect::Station)) | (_, Some(Aspect::Program)) => "INT",
        (ChannelKind::Ai, _) | (ChannelKind::Ao, _) => "REAL",
        (ChannelKind::Word, _) => "INT",
        _ => "BOOL",
    }
}

// --------------------------------------------------------------- the POU

/// Per-program state while walking its steps.
struct Pou<'a> {
    scene: &'a Scene,
    derivation: &'a iomap::IoDerivation,
    /// SFC body elements, in emission order.
    elements: Vec<String>,
    next_id: u64,
    /// Step identifiers already used (uniqueness within the POU).
    step_names: BTreeSet<String>,
    /// Globals this POU references (name → declared? via the global map).
    externals: BTreeSet<String>,
    /// FB / trigger instances: name → type.
    instances: BTreeMap<String, String>,
    /// Stub FB types used anywhere (emitted once as function block POUs).
    stubs: &'a mut BTreeSet<&'static str>,
    /// Robots for which the map says "driven from another host": start /
    /// done handshake instead of a stub FB.
    remote_start: BTreeSet<String>,
    remote_done: BTreeSet<String>,
    remote_program: BTreeSet<String>,
    /// Robot → FB instances of that robot (for `RobotDone`).
    robot_instances: BTreeMap<String, BTreeSet<String>>,
    /// Motion name → 1-based program number per robot.
    motion_numbers: BTreeMap<String, u32>,
    y: i64,
    max_y: i64,
}

impl<'a> Pou<'a> {
    fn id(&mut self) -> u64 {
        self.next_id += 1;
        self.next_id
    }

    fn robot_name(&self, robot: &Option<String>) -> String {
        match self.scene.resolve_seq_robot(robot) {
            Ok(i) => self.scene.robots()[i].name.clone(),
            Err(_) => robot.clone().unwrap_or_else(|| "robot".to_string()),
        }
    }

    fn motion_robot(&self, motion: &str) -> String {
        self.scene
            .motions()
            .iter()
            .find(|m| m.name == motion)
            .map(|m| self.scene.robots()[m.robot].name.clone())
            .unwrap_or_else(|| self.robot_name(&None))
    }

    fn unique_step(&mut self, name: &str) -> String {
        let base = ident(name);
        let mut candidate = base.clone();
        let mut n = 2;
        while self.step_names.contains(&candidate) {
            candidate = format!("{base}_{n}");
            n += 1;
        }
        self.step_names.insert(candidate.clone());
        candidate
    }

    /// The variable of a derived point by (name, aspect, direction),
    /// registering it as an external. Falls back to a plain identifier
    /// (with the same naming rules) when the derivation has no such point
    /// — the program still compiles, the variable is just undeclared in
    /// the map.
    fn var(&mut self, name: &str, aspect: Option<Aspect>, input: bool, fallback: &str) -> String {
        let want_dir = if input {
            iomap::IoDirection::Input
        } else {
            iomap::IoDirection::Output
        };
        let found = self
            .derivation
            .points
            .iter()
            .find(|p| p.id.name == name && p.id.aspect == aspect && p.id.direction == want_dir)
            .or_else(|| {
                self.derivation
                    .points
                    .iter()
                    .find(|p| p.id.name == name && p.id.aspect == aspect)
            })
            .map(point_var);
        let var = found.unwrap_or_else(|| fallback.to_string());
        self.externals.insert(var.clone());
        var
    }

    fn instance(&mut self, name: String, ty: &'static str, robot: Option<&str>) -> String {
        self.instances.insert(name.clone(), ty.to_string());
        if ty.starts_with("FB_") {
            self.stubs.insert(ty);
        }
        if let Some(robot) = robot {
            self.robot_instances
                .entry(robot.to_string())
                .or_default()
                .insert(name.clone());
        }
        name
    }

    // ---- conditions ---------------------------------------------------

    /// The ST expression of a condition; `started` are the FB instances
    /// this step started (for `Done`); `edge_calls` collects trigger calls
    /// the step's action block must make.
    fn condition_st(
        &mut self,
        c: &Condition,
        step_ident: &str,
        started: &[String],
        edge_calls: &mut Vec<String>,
    ) -> String {
        match c {
            Condition::Immediately => "TRUE".to_string(),
            Condition::Done => {
                if started.is_empty() {
                    "TRUE".to_string()
                } else {
                    started.join(" AND ")
                }
            }
            Condition::RobotDone { robot } => {
                if self.remote_done.contains(robot) {
                    self.var(
                        robot,
                        Some(Aspect::Done),
                        true,
                        &format!("{}_done", ident(robot)),
                    )
                } else {
                    let insts: Vec<String> = self
                        .robot_instances
                        .get(robot)
                        .map(|s| s.iter().map(|i| format!("{i}.done")).collect())
                        .unwrap_or_default();
                    if insts.is_empty() {
                        "TRUE".to_string()
                    } else {
                        insts.join(" AND ")
                    }
                }
            }
            Condition::Elapsed { seconds } => format!("{step_ident}.T >= {}", st_time(*seconds)),
            Condition::Signal { name, value } => {
                let v = self.var(name, None, true, &ident(name));
                if *value {
                    v
                } else {
                    format!("NOT {v}")
                }
            }
            Condition::Rising { name } => {
                let v = self.var(name, None, true, &ident(name));
                let inst = self.instance(format!("{v}_rise"), "R_TRIG", None);
                edge_calls.push(format!("{inst}(CLK := {v});"));
                format!("{inst}.Q")
            }
            Condition::Falling { name } => {
                let v = self.var(name, None, true, &ident(name));
                let inst = self.instance(format!("{v}_fall"), "F_TRIG", None);
                edge_calls.push(format!("{inst}(CLK := {v});"));
                format!("{inst}.Q")
            }
            Condition::DeviceDone { device } => {
                self.var(device, None, true, &format!("{}_done", ident(device)))
            }
            Condition::All(cs) => {
                let parts: Vec<String> = cs
                    .iter()
                    .map(|c| self.condition_st(c, step_ident, started, edge_calls))
                    .collect();
                if parts.len() == 1 {
                    parts[0].clone()
                } else {
                    format!("({})", parts.join(" AND "))
                }
            }
            Condition::Any(cs) => {
                let parts: Vec<String> = cs
                    .iter()
                    .map(|c| self.condition_st(c, step_ident, started, edge_calls))
                    .collect();
                if parts.len() == 1 {
                    parts[0].clone()
                } else {
                    format!("({})", parts.join(" OR "))
                }
            }
        }
    }

    // ---- actions ------------------------------------------------------

    /// One step's entry actions as (inline ST statements, boolean action
    /// references, FB instances started).
    fn actions_st(&mut self, actions: &[Action]) -> (Vec<String>, Vec<String>, Vec<String>) {
        let mut st = Vec::new();
        let mut bools = Vec::new();
        let mut started = Vec::new();
        for action in actions {
            match action {
                Action::StartMotion { motion } => {
                    let robot = self.motion_robot(motion);
                    if self.remote_start.contains(&robot) {
                        let start = self.var(
                            &robot,
                            Some(Aspect::Start),
                            false,
                            &format!("{}_start", ident(&robot)),
                        );
                        bools.push(start);
                        if self.remote_program.contains(&robot) {
                            let n = self.motion_numbers.get(motion).copied().unwrap_or(0);
                            let word = self.var(
                                &robot,
                                Some(Aspect::Program),
                                false,
                                &format!("{}_program", ident(&robot)),
                            );
                            st.push(format!("{word} := {n}; (* {motion} *)"));
                        }
                        let done = self.var(
                            &robot,
                            Some(Aspect::Done),
                            true,
                            &format!("{}_done", ident(&robot)),
                        );
                        started.push(done);
                    } else {
                        let inst = self.instance(
                            format!("{}_motion", ident(&robot)),
                            "FB_StartMotion",
                            Some(&robot),
                        );
                        st.push(format!(
                            "{inst}(robot := {}, name := {}, start := TRUE);",
                            st_string(&robot),
                            st_string(motion)
                        ));
                        started.push(format!("{inst}.done"));
                    }
                }
                Action::StartRamp {
                    robot,
                    targets,
                    duration,
                } => {
                    let robot = self.robot_name(robot);
                    let inst = self.instance(
                        format!("{}_ramp", ident(&robot)),
                        "FB_StartRamp",
                        Some(&robot),
                    );
                    let summary: Vec<String> = targets
                        .iter()
                        .map(|(j, v)| format!("{j}={}", st_real(*v)))
                        .collect();
                    st.push(format!(
                        "{inst}(robot := {}, name := {}, start := TRUE); (* {} s *)",
                        st_string(&robot),
                        st_string(&summary.join(" ")),
                        st_real(*duration)
                    ));
                    started.push(format!("{inst}.done"));
                }
                Action::StartToolpath { robot, toolpath } => {
                    let robot = self.robot_name(robot);
                    let inst = self.instance(
                        format!("{}_toolpath", ident(&robot)),
                        "FB_StartToolpath",
                        Some(&robot),
                    );
                    st.push(format!(
                        "{inst}(robot := {}, name := {}, start := TRUE);",
                        st_string(&robot),
                        st_string(toolpath)
                    ));
                    started.push(format!("{inst}.done"));
                }
                Action::Attach { robot, object, .. } => {
                    let robot = self.robot_name(robot);
                    let inst = self.instance(
                        format!("{}_attach", ident(&robot)),
                        "FB_Attach",
                        Some(&robot),
                    );
                    st.push(format!(
                        "{inst}(robot := {}, name := {}, start := TRUE);",
                        st_string(&robot),
                        st_string(object)
                    ));
                }
                Action::Detach { object } => {
                    let inst = self.instance("detach".to_string(), "FB_Detach", None);
                    st.push(format!(
                        "{inst}(robot := '', name := {}, start := TRUE);",
                        st_string(object)
                    ));
                }
                Action::Track { robot, object, .. } => {
                    let robot = self.robot_name(robot);
                    let inst =
                        self.instance(format!("{}_track", ident(&robot)), "FB_Track", Some(&robot));
                    st.push(format!(
                        "{inst}(robot := {}, name := {}, start := TRUE);",
                        st_string(&robot),
                        st_string(object)
                    ));
                }
                Action::Untrack { robot } => {
                    let robot = self.robot_name(robot);
                    let inst = self.instance(
                        format!("{}_untrack", ident(&robot)),
                        "FB_Untrack",
                        Some(&robot),
                    );
                    st.push(format!(
                        "{inst}(robot := {}, name := '', start := TRUE);",
                        st_string(&robot)
                    ));
                }
                Action::Set { signal, value } => {
                    let v = self.var(signal, None, false, &ident(signal));
                    st.push(format!("{v} := {};", if *value { "TRUE" } else { "FALSE" }));
                }
                Action::Device { device, command } => match command {
                    _ if self.scene.devices().iter().any(|d| {
                        &d.name == device
                            && matches!(
                                d.kind,
                                crate::seq::DeviceKind::Source { .. }
                                    | crate::seq::DeviceKind::Sink { .. }
                            )
                    }) =>
                    {
                        // A magazine / return chute models an endless line;
                        // it is not equipment the PLC drives.
                        st.push(format!("(* {device}: feeder model, no I/O *)"));
                    }
                    DeviceCommand::Start => {
                        let v = self.var(device, None, false, &format!("{}_run", ident(device)));
                        st.push(format!("{v} := TRUE;"));
                    }
                    DeviceCommand::Stop => {
                        let v = self.var(device, None, false, &format!("{}_run", ident(device)));
                        st.push(format!("{v} := FALSE;"));
                    }
                    DeviceCommand::SetSpeed(speed) => {
                        let v = self.var(
                            device,
                            Some(Aspect::Speed),
                            false,
                            &format!("{}_speed", ident(device)),
                        );
                        st.push(format!("{v} := {};", st_real(*speed)));
                    }
                    DeviceCommand::MoveTo(pos) => {
                        let v = self.var(
                            device,
                            Some(Aspect::Position),
                            false,
                            &format!("{}_position", ident(device)),
                        );
                        st.push(format!("{v} := {};", st_real(*pos)));
                    }
                    DeviceCommand::Goto { station } => {
                        let word = self.var(
                            device,
                            Some(Aspect::Station),
                            false,
                            &format!("{}_station", ident(device)),
                        );
                        let index = self
                            .scene
                            .devices()
                            .iter()
                            .find(|d| &d.name == device)
                            .and_then(|d| match &d.kind {
                                crate::seq::DeviceKind::Vehicle { path, .. } => {
                                    path.stations.iter().position(|(s, _)| s == station)
                                }
                                _ => None,
                            })
                            .map(|i| i as i64 + 1)
                            .unwrap_or(0);
                        st.push(format!("{word} := {index}; (* {station} *)"));
                        let go = self.var(
                            device,
                            Some(Aspect::Dispatch),
                            false,
                            &format!("{}_dispatch", ident(device)),
                        );
                        bools.push(go);
                    }
                    DeviceCommand::Advance(distance) => {
                        let go = self.var(
                            device,
                            Some(Aspect::Index),
                            false,
                            &format!("{}_index", ident(device)),
                        );
                        st.push(format!("(* advance {} m *)", st_real(*distance)));
                        bools.push(go);
                    }
                },
            }
        }
        (st, bools, started)
    }

    // ---- SFC elements -------------------------------------------------

    fn push_element(&mut self, xml: String) {
        self.elements.push(xml);
    }

    fn pos(&mut self, x: i64) -> String {
        let y = self.y;
        self.y += 40;
        self.max_y = self.max_y.max(self.y);
        format!("<position x=\"{x}\" y=\"{y}\"/>")
    }

    fn conn_in(from: u64) -> String {
        format!("<connectionPointIn><relPosition x=\"0\" y=\"0\"/><connection refLocalId=\"{from}\"/></connectionPointIn>")
    }

    /// Emits a chain of steps starting after `prev` (the localId the first
    /// step connects to). Returns the localId of the last transition of
    /// the chain — what the next element connects to — or `prev` when the
    /// chain is empty.
    fn emit_steps(&mut self, steps: &[Step], prev: u64, x: i64, initial: bool) -> u64 {
        let mut prev = prev;
        for (k, step) in steps.iter().enumerate() {
            let is_initial = initial && k == 0;
            let step_ident = self.unique_step(&step.name);
            let step_id = self.id();
            let position = self.pos(x);
            let conn_in = if is_initial {
                String::new()
            } else {
                Self::conn_in(prev)
            };
            self.push_element(format!(
                "<step localId=\"{step_id}\" name=\"{}\"{}>{position}{conn_in}<connectionPointOut formalParameter=\"\"/></step>",
                xml_attr(&step_ident),
                if is_initial { " initialStep=\"true\"" } else { "" }
            ));

            if !step.select.is_empty() {
                // A branching step: divergence, arms, convergence.
                let div_id = self.id();
                let position = self.pos(x);
                let outs: String = (0..step.select.len())
                    .map(|_| "<connectionPointOut formalParameter=\"\"/>".to_string())
                    .collect();
                self.push_element(format!(
                    "<selectionDivergence localId=\"{div_id}\">{position}{}{outs}</selectionDivergence>",
                    Self::conn_in(step_id)
                ));
                let mut arm_tails: Vec<u64> = Vec::new();
                let start_y = self.y;
                let mut deepest_y = self.y;
                let mut others: Vec<String> = Vec::new();
                for (a, arm) in step.select.iter().enumerate() {
                    self.y = start_y;
                    let arm_x = x + 160 * a as i64;
                    let mut edge_calls = Vec::new();
                    let cond = self.condition_st(&arm.condition, &step_ident, &[], &mut edge_calls);
                    // `otherwise` (Immediately as the last arm) is
                    // "none of the others", so the branch is exclusive.
                    let cond = if matches!(arm.condition, Condition::Immediately)
                        && a == step.select.len() - 1
                        && !others.is_empty()
                    {
                        format!("NOT ({})", others.join(" OR "))
                    } else {
                        cond
                    };
                    others.push(cond.clone());
                    if !edge_calls.is_empty() {
                        // Edge triggers referenced by an arm are polled by
                        // the branching step's action block.
                        self.attach_edge_calls(step_id, &edge_calls);
                    }
                    let t_id = self.id();
                    let position = self.pos(arm_x);
                    self.push_element(format!(
                        "<transition localId=\"{t_id}\" priority=\"{}\">{position}{}<connectionPointOut/><condition><inline name=\"\">{}</inline></condition></transition>",
                        a + 1,
                        Self::conn_in(div_id),
                        st_body(&cond)
                    ));
                    let tail = self.emit_steps(&arm.steps, t_id, arm_x, false);
                    arm_tails.push(tail);
                    deepest_y = deepest_y.max(self.y);
                }
                self.y = deepest_y;
                let conv_id = self.id();
                let position = self.pos(x);
                let ins: String = arm_tails
                    .iter()
                    .map(|t| format!("<connectionPointIn><relPosition x=\"0\" y=\"0\"/><connection refLocalId=\"{t}\"/></connectionPointIn>"))
                    .collect();
                self.push_element(format!(
                    "<selectionConvergence localId=\"{conv_id}\">{position}{ins}<connectionPointOut/></selectionConvergence>"
                ));
                prev = conv_id;
                continue;
            }

            // Actions and the transition.
            let (mut st, bools, started) = self.actions_st(&step.actions);
            let mut edge_calls = Vec::new();
            let cond = self.condition_st(&step.transition, &step_ident, &started, &mut edge_calls);
            st.extend(edge_calls);
            if !st.is_empty() || !bools.is_empty() {
                let ab_id = self.id();
                let position = self.pos(x + 60);
                let mut actions = String::new();
                for b in &bools {
                    let _ = write!(
                        actions,
                        "<action localId=\"{}\" qualifier=\"N\"><relPosition x=\"0\" y=\"0\"/><reference name=\"{}\"/></action>",
                        self.id(),
                        xml_attr(b)
                    );
                }
                if !st.is_empty() {
                    let _ = write!(
                        actions,
                        "<action localId=\"{}\" qualifier=\"N\"><relPosition x=\"0\" y=\"0\"/><inline>{}</inline></action>",
                        self.id(),
                        st_body(&st.join("\n"))
                    );
                }
                self.push_element(format!(
                    "<actionBlock localId=\"{ab_id}\">{position}{}{actions}</actionBlock>",
                    Self::conn_in(step_id)
                ));
            }
            let t_id = self.id();
            let position = self.pos(x);
            self.push_element(format!(
                "<transition localId=\"{t_id}\">{position}{}<connectionPointOut/><condition><inline name=\"\">{}</inline></condition></transition>",
                Self::conn_in(step_id),
                st_body(&cond)
            ));
            prev = t_id;
        }
        prev
    }

    /// Adds an action block with trigger calls to a step that has none of
    /// its own (a branching step whose arms test edges).
    fn attach_edge_calls(&mut self, step_id: u64, calls: &[String]) {
        let ab_id = self.id();
        let position = self.pos(60);
        let a_id = self.id();
        self.push_element(format!(
            "<actionBlock localId=\"{ab_id}\">{position}{}<action localId=\"{a_id}\" qualifier=\"N\"><relPosition x=\"0\" y=\"0\"/><inline>{}</inline></action></actionBlock>",
            Self::conn_in(step_id),
            st_body(&calls.join("\n"))
        ));
    }
}

/// The stub function-block POUs: every robot command the SFC issues calls
/// one of these with `start := TRUE` and waits on `done`. As shipped, a
/// stub completes at once (`done := start`), which is what lets the file
/// run in a PLC IDE's simulator; the control engineer replaces the body
/// with the controller interface (a fieldbus handshake, a job number).
const STUB_FBS: &[(&str, &str)] = &[
    (
        "FB_StartMotion",
        "start the named motion program on the robot",
    ),
    (
        "FB_StartRamp",
        "run a joint ramp (gripper open/close) on the robot",
    ),
    (
        "FB_StartToolpath",
        "start the named process path on the robot",
    ),
    (
        "FB_Attach",
        "grasp: the robot's tool takes the named object",
    ),
    ("FB_Detach", "release the named object"),
    ("FB_Track", "latch onto a moving part (conveyor tracking)"),
    ("FB_Untrack", "release conveyor tracking"),
];

/// Renders the PLCopen XML document.
pub fn render_plcopen(scene: &Scene, options: &PlcopenOptions) -> Result<String, PlcopenError> {
    let names: Vec<String> = match &options.sequences {
        Some(list) => {
            for name in list {
                if !scene.sequences().iter().any(|s| &s.name == name) {
                    return Err(PlcopenError::UnknownSequence(name.clone()));
                }
            }
            list.clone()
        }
        None => scene.sequences().iter().map(|s| s.name.clone()).collect(),
    };
    if names.is_empty() {
        return Err(PlcopenError::NoSequences);
    }
    let refs: Vec<&str> = names.iter().map(String::as_str).collect();
    let derivation =
        iomap::derive(scene, Some(&refs)).map_err(|e| PlcopenError::Derivation(e.to_string()))?;

    // ---- global variables from the derived points ------------------------
    let mut globals: Vec<GlobalVar> = Vec::new();
    let mut seen: BTreeSet<String> = BTreeSet::new();
    for p in &derivation.points {
        if matches!(p.status, PointStatus::Cosmetic) {
            continue;
        }
        let name = point_var(p);
        if !seen.insert(name.clone()) {
            continue;
        }
        let address = match p.status {
            PointStatus::Bound(i) => derivation.io.bindings.get(i).and_then(|b| {
                let node = derivation.io.node(&b.node)?;
                if !matches!(
                    node.kind,
                    IoNodeKind::Plc | IoNodeKind::SafetyPlc | IoNodeKind::RemoteIo
                ) {
                    return None;
                }
                node.channels
                    .iter()
                    .find(|c| c.id == b.channel)
                    .and_then(|c| c.address.clone())
            }),
            _ => None,
        };
        let initial = if point_type(p) == "BOOL" {
            scene
                .signals()
                .iter()
                .find(|s| s.name == p.id.name && p.id.aspect.is_none())
                .map(|s| {
                    if s.initial {
                        "TRUE".to_string()
                    } else {
                        "FALSE".to_string()
                    }
                })
        } else {
            None
        };
        globals.push(GlobalVar {
            name,
            ty: point_type(p),
            address,
            initial,
            doc: format!(
                "{} {} ({}){}",
                p.source.as_str(),
                p.id.label(),
                p.kind.as_str(),
                p.host
                    .as_ref()
                    .map(|h| format!(" on {h}"))
                    .unwrap_or_default()
            ),
        });
    }

    // Robots a program drives *from another host* — the derivation put a
    // start / done (/ program) point on the program's own host — use the
    // handshake pair; robots on the program's own controller use the stub
    // FBs. Decided per program: `pick` on the PLC drives `far` by handshake
    // while `watch` on far's own controller calls the stub.
    let remote_for = |host: &str, source: IoSource| -> BTreeSet<String> {
        derivation
            .points
            .iter()
            .filter(|p| p.source == source && p.host.as_deref() == Some(host))
            .filter(|p| {
                // A mirror point on the robot's own controller (start in,
                // done out) is the other end of the wire, not a remote drive.
                !derivation
                    .io
                    .node(host)
                    .is_some_and(|n| matches!(&n.kind, IoNodeKind::RobotController { robots } if robots.contains(&p.id.name)))
            })
            .map(|p| p.id.name.clone())
            .collect()
    };
    // Program numbers: motions numbered per robot in scene order.
    let mut motion_numbers: BTreeMap<String, u32> = BTreeMap::new();
    let mut per_robot: BTreeMap<usize, u32> = BTreeMap::new();
    for m in scene.motions() {
        let n = per_robot.entry(m.robot).or_default();
        *n += 1;
        motion_numbers.insert(m.name.clone(), *n);
    }

    // ---- programs ----------------------------------------------------------
    let mut stubs: BTreeSet<&'static str> = BTreeSet::new();
    let mut pous = String::new();
    let mut instances_xml = String::new();
    for name in &names {
        let sequence = scene
            .sequences()
            .iter()
            .find(|s| &s.name == name)
            .expect("checked above");
        let program = ident(name);
        let host = derivation
            .program_hosts
            .iter()
            .find(|(p, _)| p == name)
            .map(|(_, h)| h.clone())
            .unwrap_or_else(|| "<cell>".to_string());
        let mut pou = Pou {
            scene,
            derivation: &derivation,
            elements: Vec::new(),
            next_id: 0,
            step_names: BTreeSet::new(),
            externals: BTreeSet::new(),
            instances: BTreeMap::new(),
            stubs: &mut stubs,
            remote_start: remote_for(&host, IoSource::RobotStart),
            remote_done: remote_for(&host, IoSource::RobotDone),
            remote_program: remote_for(&host, IoSource::RobotProgram),
            robot_instances: BTreeMap::new(),
            motion_numbers: motion_numbers.clone(),
            y: 20,
            max_y: 20,
        };
        let first_step = sequence
            .steps
            .first()
            .map(|s| ident(&s.name))
            .unwrap_or_else(|| "init".to_string());
        let tail = pou.emit_steps(&sequence.steps, 0, 40, true);
        if sequence.steps.is_empty() {
            // An empty program: one initial step that waits forever.
            let id = pou.id();
            let position = pou.pos(40);
            pou.push_element(format!(
                "<step localId=\"{id}\" name=\"init\" initialStep=\"true\">{position}<connectionPointOut formalParameter=\"\"/></step>"
            ));
        } else if options.cycle {
            let id = pou.id();
            let position = pou.pos(40);
            pou.push_element(format!(
                "<jumpStep localId=\"{id}\" targetName=\"{}\">{position}{}</jumpStep>",
                xml_attr(&first_step),
                Pou::conn_in(tail)
            ));
        } else {
            let id = pou.id();
            let position = pou.pos(40);
            let end = pou.unique_step("end_of_cycle");
            pou.push_element(format!(
                "<step localId=\"{id}\" name=\"{}\">{position}{}</step>",
                xml_attr(&end),
                Pou::conn_in(tail)
            ));
        }
        let Pou {
            elements,
            externals,
            instances,
            ..
        } = pou;

        // Interface: externals (globals used) and local instances.
        let mut interface = String::new();
        let mut ext_vars = String::new();
        for var in &externals {
            let ty = globals
                .iter()
                .find(|g| &g.name == var)
                .map(|g| g.ty)
                .unwrap_or("BOOL");
            let _ = write!(
                ext_vars,
                "<variable name=\"{}\"><type><{ty}/></type></variable>",
                xml_attr(var)
            );
        }
        if !externals.is_empty() {
            let _ = write!(interface, "<externalVars>{ext_vars}</externalVars>");
        }
        if !instances.is_empty() {
            let mut locals = String::new();
            for (inst, ty) in &instances {
                let _ = write!(
                    locals,
                    "<variable name=\"{}\"><type><derived name=\"{}\"/></type></variable>",
                    xml_attr(inst),
                    xml_attr(ty)
                );
            }
            let _ = write!(interface, "<localVars>{locals}</localVars>");
        }
        let _ = write!(
            pous,
            "<pou name=\"{}\" pouType=\"program\"><interface>{interface}</interface><body><SFC>{}</SFC></body><documentation><xhtml:p>{}</xhtml:p></documentation></pou>",
            xml_attr(&program),
            elements.join(""),
            cdata(&format!("botrail sequence `{name}`"))
        );
        let _ = write!(
            instances_xml,
            "<pouInstance name=\"{}_inst\" typeName=\"{}\"/>",
            xml_attr(&program),
            xml_attr(&program)
        );
        // Every used variable that is not a derived point becomes a global
        // too (a fallback identifier), typed BOOL.
        for var in externals {
            if !globals.iter().any(|g| g.name == var) {
                globals.push(GlobalVar {
                    name: var,
                    ty: "BOOL",
                    address: None,
                    initial: None,
                    doc: "referenced by the program; not in the derived I/O map".to_string(),
                });
            }
        }
    }

    // ---- stub function blocks --------------------------------------------
    let mut stub_pous = String::new();
    for (fb, doc) in STUB_FBS {
        if !stubs.contains(fb) {
            continue;
        }
        let _ = write!(
            stub_pous,
            "<pou name=\"{fb}\" pouType=\"functionBlock\"><interface><inputVars><variable name=\"robot\"><type><string/></type></variable><variable name=\"name\"><type><string/></type></variable><variable name=\"start\"><type><BOOL/></type></variable></inputVars><outputVars><variable name=\"done\"><type><BOOL/></type></variable></outputVars></interface><body>{}</body><documentation><xhtml:p>{}</xhtml:p></documentation></pou>",
            st_body("(* botrail stub: replace with the controller interface *)\ndone := start;"),
            cdata(doc)
        );
    }

    // ---- globals -----------------------------------------------------------
    let mut globals_xml = String::new();
    for g in &globals {
        let address = g
            .address
            .as_ref()
            .map(|a| format!(" address=\"{}\"", xml_attr(a)))
            .unwrap_or_default();
        let initial = g
            .initial
            .as_ref()
            .map(|v| format!("<initialValue><simpleValue value=\"{v}\"/></initialValue>"))
            .unwrap_or_default();
        let _ = write!(
            globals_xml,
            "<variable name=\"{}\"{address}><type><{}/></type>{initial}<documentation><xhtml:p>{}</xhtml:p></documentation></variable>",
            xml_attr(&g.name),
            g.ty,
            cdata(&g.doc)
        );
    }

    // ---- document ------------------------------------------------------------
    let mut out = String::new();
    out.push_str("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    out.push_str("<project xmlns=\"http://www.plcopen.org/xml/tc6_0201\" xmlns:xhtml=\"http://www.w3.org/1999/xhtml\" xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\">\n");
    // Timestamps are fixed on purpose: the export is a deliverable that
    // hashes into the cell report, and the same cell must give the same
    // bytes.
    let _ = writeln!(
        out,
        "<fileHeader companyName=\"botrail\" productName=\"botrail\" productVersion=\"{}\" creationDateTime=\"2000-01-01T00:00:00\" contentDescription=\"{}\"/>",
        env!("CARGO_PKG_VERSION"),
        xml_attr(&format!("botrail cell `{}` — SFC programs {}", options.name, names.join(", ")))
    );
    let _ = writeln!(
        out,
        "<contentHeader name=\"{}\" modificationDateTime=\"2000-01-01T00:00:00\"><coordinateInfo><fbd><scaling x=\"0\" y=\"0\"/></fbd><ld><scaling x=\"0\" y=\"0\"/></ld><sfc><scaling x=\"0\" y=\"0\"/></sfc></coordinateInfo></contentHeader>",
        xml_attr(&options.name)
    );
    let _ = writeln!(
        out,
        "<types><dataTypes/><pous>{stub_pous}{pous}</pous></types>"
    );
    let _ = writeln!(
        out,
        "<instances><configurations><configuration name=\"{}\"><resource name=\"plc\"><task name=\"main\" interval=\"T#{}ms\" priority=\"0\">{instances_xml}</task><globalVars>{globals_xml}</globalVars></resource></configuration></configurations></instances>",
        xml_attr(&ident(&options.name)),
        options.task_interval_ms
    );
    out.push_str("</project>\n");
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::{SelectArm, Sensor, SensorKind, SensorWatch, Sequence};
    use botrail_model::RobotModel;
    use nalgebra::Point3;
    use std::sync::Arc;

    const URDF: &str = r#"<robot name="arm">
  <link name="base"/>
  <link name="l1"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link>
  <joint name="j1" type="revolute">
    <parent link="base"/><child link="l1"/><axis xyz="0 0 1"/>
    <limit lower="-3" upper="3" effort="1" velocity="1"/>
  </joint>
</robot>"#;

    fn cell() -> Scene {
        let mut scene = Scene::new(Arc::new(RobotModel::from_urdf_str(URDF).unwrap()));
        scene.upsert_sensor(Sensor {
            name: "beam pick".into(),
            kind: SensorKind::Beam {
                from: Point3::origin(),
                to: Point3::new(1.0, 0.0, 0.0),
                radius: 0.01,
            },
            watch: SensorWatch::All,
            mount: None,
        });
        scene.upsert_device(crate::seq::Device {
            name: "belt".into(),
            kind: crate::seq::DeviceKind::Conveyor {
                zone_pose: nalgebra::Isometry3::identity(),
                zone_size: nalgebra::Vector3::new(1.0, 1.0, 1.0),
                velocity: nalgebra::Vector3::new(0.1, 0.0, 0.0),
                running: false,
            },
        });
        scene.define_signal("carrying", false);
        scene.define_signal("spec_ok", true);
        scene
            .add_segment(
                "to_pick",
                crate::motion::Segment {
                    kind: crate::motion::SegmentKind::Joint,
                    goal_positions: vec![0.5],
                    constraints: vec![],
                },
            )
            .unwrap();
        scene.set_sequences(vec![Sequence {
            name: "pick place".into(),
            steps: vec![
                Step {
                    name: "feed".into(),
                    actions: vec![
                        Action::Device {
                            device: "belt".into(),
                            command: DeviceCommand::Start,
                        },
                        Action::StartMotion {
                            motion: "to_pick".into(),
                        },
                    ],
                    transition: Condition::All(vec![
                        Condition::Rising {
                            name: "beam pick".into(),
                        },
                        Condition::Done,
                    ]),
                    select: vec![],
                },
                Step {
                    name: "settle".into(),
                    actions: vec![Action::Set {
                        signal: "carrying".into(),
                        value: true,
                    }],
                    transition: Condition::Elapsed { seconds: 0.5 },
                    select: vec![],
                },
                Step {
                    name: "judge".into(),
                    actions: vec![],
                    transition: Condition::Immediately,
                    select: vec![
                        SelectArm {
                            condition: Condition::Signal {
                                name: "spec_ok".into(),
                                value: true,
                            },
                            steps: vec![Step {
                                name: "place".into(),
                                actions: vec![Action::Device {
                                    device: "belt".into(),
                                    command: DeviceCommand::Stop,
                                }],
                                transition: Condition::Immediately,
                                select: vec![],
                            }],
                        },
                        SelectArm {
                            condition: Condition::Immediately,
                            steps: vec![],
                        },
                    ],
                },
            ],
        }]);
        scene
    }

    #[test]
    fn identifiers_are_iec_safe() {
        assert_eq!(ident("beam pick"), "beam_pick");
        assert_eq!(ident("/World/Conveyor/Belt"), "World_Conveyor_Belt");
        assert_eq!(ident("2nd"), "_2nd");
        assert_eq!(ident("step"), "step_");
        assert_eq!(ident("far.done"), "far_done");
        assert_eq!(st_time(0.5), "T#500ms");
        assert_eq!(st_time(2.0), "T#2s");
    }

    #[test]
    fn a_program_renders_steps_transitions_branches_and_globals() {
        let scene = cell();
        let xml = render_plcopen(&scene, &PlcopenOptions::default()).unwrap();
        assert!(xml.starts_with("<?xml version=\"1.0\""));
        assert!(xml.contains("xmlns=\"http://www.plcopen.org/xml/tc6_0201\""));
        // The program, its steps and the cycle jump.
        assert!(xml.contains("<pou name=\"pick_place\" pouType=\"program\">"));
        assert!(xml.contains("name=\"feed\" initialStep=\"true\""));
        assert!(xml.contains("<jumpStep localId=") && xml.contains("targetName=\"feed\""));
        // Actions: the coil, the stub FB call, the signal write.
        assert!(xml.contains("belt_run := TRUE;"), "{xml}");
        assert!(xml.contains("arm_motion(robot := 'arm', name := 'to_pick', start := TRUE);"));
        assert!(xml.contains("carrying := TRUE;"));
        // Conditions: the edge via R_TRIG polled in the step, done via the FB, the timer.
        assert!(xml.contains("beam_pick_rise(CLK := beam_pick);"));
        assert!(
            xml.contains("(beam_pick_rise.Q AND arm_motion.done)"),
            "{xml}"
        );
        assert!(xml.contains("settle.T >= T#500ms"));
        // The branch: divergence, arms with priorities, otherwise = NOT the others, convergence.
        assert!(xml.contains("<selectionDivergence"));
        assert!(xml.contains("priority=\"1\"") && xml.contains("priority=\"2\""));
        assert!(xml.contains("NOT (spec_ok)"), "{xml}");
        assert!(xml.contains("<selectionConvergence"));
        // Globals: the derived points, typed, with the signal's initial value.
        assert!(xml.contains("<variable name=\"belt_run\"><type><BOOL/></type>"));
        assert!(xml.contains("<variable name=\"spec_ok\"><type><BOOL/></type><initialValue><simpleValue value=\"TRUE\"/></initialValue>"));
        // Stubs used are declared once as function blocks.
        assert!(xml.contains("<pou name=\"FB_StartMotion\" pouType=\"functionBlock\">"));
        assert!(!xml.contains("FB_Attach"));
        // Externals in the program interface, the instances local.
        assert!(xml.contains("<externalVars>"));
        assert!(xml.contains("<variable name=\"arm_motion\"><type><derived name=\"FB_StartMotion\"/></type></variable>"));
        assert!(xml.contains(
            "<variable name=\"beam_pick_rise\"><type><derived name=\"R_TRIG\"/></type></variable>"
        ));
        // Deterministic.
        assert_eq!(
            xml,
            render_plcopen(&scene, &PlcopenOptions::default()).unwrap()
        );
    }

    #[test]
    fn options_select_programs_and_the_end_style() {
        let scene = cell();
        let err = render_plcopen(
            &scene,
            &PlcopenOptions {
                sequences: Some(vec!["nope".into()]),
                ..Default::default()
            },
        )
        .unwrap_err();
        assert!(matches!(err, PlcopenError::UnknownSequence(_)));
        let parked = render_plcopen(
            &scene,
            &PlcopenOptions {
                cycle: false,
                name: "my cell".into(),
                ..Default::default()
            },
        )
        .unwrap();
        assert!(!parked.contains("<jumpStep"));
        assert!(parked.contains("name=\"end_of_cycle\""));
        assert!(parked.contains("<configuration name=\"my_cell\">"));
        assert!(parked.contains("<contentHeader name=\"my cell\""));
    }
}
