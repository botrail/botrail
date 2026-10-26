//! Structured Text (IEC 61131-3) export of one controller's programs — the
//! form a runtime compiles on its own (OpenPLC's matiec, Beremiz, CODESYS,
//! and the vendor IDEs that read ST but not PLCopen XML).
//!
//! Where [`crate::plcopen`] hands the sequences over as SFC, this writes
//! the same logic as a step-flag program: one `BOOL` per step, the
//! transitions as `IF step AND condition THEN` blocks in authored order,
//! the entry actions fired where the next step's flag is raised. Both come
//! from the same source — the sequences and the host's
//! [`crate::exchange::Exchange`] — so the variables, the addresses, the
//! program numbers and the handshakes are the PLCopen file's.
//!
//! The handshakes are written for a controller that reaches the plant
//! over a polled link (design-virtual-commissioning.md VC2): a robot
//! command raises `start` and holds it until the robot reads busy (`done`
//! low — the plant holds it low once it took the command), then drops it
//! and waits for `done` again; a dispatch or index pulse is dropped once
//! the device reads busy. Edge triggers are called every scan at the top
//! of the program, so an edge raised while another step was active is
//! remembered the way the bake remembers it.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;

use crate::exchange::{self, DeviceClass, Exchange, ExchangePoint, PlantSide, Value, ValueKind};
use crate::iomap::{Aspect, IoNodeKind};
use crate::plcopen::{ident, st_real, st_time};
use crate::seq::{Action, Condition, DeviceCommand, DeviceKind, Step};
use crate::Scene;

/// What to export and how to package it.
#[derive(Debug, Clone, PartialEq)]
pub struct StOptions {
    /// Programs to consider (`None` = every sequence, in scene order); the
    /// ones on `host` are written.
    pub sequences: Option<Vec<String>>,
    /// The controller whose programs are written.
    pub host: String,
    /// Cyclic task interval, milliseconds.
    pub task_interval_ms: u32,
    /// Start a program over after its last step (a production PLC cycles);
    /// `false` parks it.
    pub cycle: bool,
}

impl Default for StOptions {
    fn default() -> Self {
        StOptions {
            sequences: None,
            host: String::new(),
            task_interval_ms: 10,
            cycle: true,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum StError {
    Exchange(String),
    /// The host runs none of the programs.
    NoPrograms {
        host: String,
        hosts: Vec<String>,
    },
    Unsupported(String),
}

impl std::fmt::Display for StError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            StError::Exchange(m) => write!(f, "{m}"),
            StError::NoPrograms { host, hosts } => write!(
                f,
                "host `{host}` runs no program of this set (hosts with programs: {})",
                if hosts.is_empty() {
                    "none".to_string()
                } else {
                    hosts.join(", ")
                }
            ),
            StError::Unsupported(m) => write!(f, "{m}"),
        }
    }
}

impl std::error::Error for StError {}

// ---------------------------------------------------------------- flatten

/// One compiled step: the authored tree flattened pre-order, every exit an
/// explicit `(condition, target)` — the rollout's own flattening, so the
/// written program walks the same path the bake does.
struct FlatStep<'a> {
    ident: String,
    actions: &'a [Action],
    /// `(condition, flat target)`; `target == len` is the end of the program.
    exits: Vec<(&'a Condition, usize)>,
    /// A branching step: the exits are its arms, in priority order.
    branching: bool,
}

fn flatten<'a>(steps: &'a [Step], names: &mut BTreeSet<String>) -> Vec<FlatStep<'a>> {
    fn unique(name: &str, names: &mut BTreeSet<String>) -> String {
        let base = ident(name);
        let mut candidate = base.clone();
        let mut n = 2;
        while names.contains(&candidate) {
            candidate = format!("{base}_{n}");
            n += 1;
        }
        names.insert(candidate.clone());
        candidate
    }
    fn emit<'a>(
        steps: &'a [Step],
        out: &mut Vec<FlatStep<'a>>,
        names: &mut BTreeSet<String>,
    ) -> Vec<(usize, usize)> {
        let mut dangling: Vec<(usize, usize)> = Vec::new();
        for step in steps {
            let here = out.len();
            for (i, slot) in dangling.drain(..) {
                out[i].exits[slot].1 = here;
            }
            if step.select.is_empty() {
                out.push(FlatStep {
                    ident: unique(&step.name, names),
                    actions: &step.actions,
                    exits: vec![(&step.transition, usize::MAX)],
                    branching: false,
                });
                dangling.push((here, 0));
            } else {
                out.push(FlatStep {
                    ident: unique(&step.name, names),
                    actions: &step.actions,
                    exits: step
                        .select
                        .iter()
                        .map(|arm| (&arm.condition, usize::MAX))
                        .collect(),
                    branching: true,
                });
                for (a, arm) in step.select.iter().enumerate() {
                    if arm.steps.is_empty() {
                        dangling.push((here, a));
                    } else {
                        out[here].exits[a].1 = out.len();
                        let tails = emit(&arm.steps, out, names);
                        dangling.extend(tails);
                    }
                }
            }
        }
        dangling
    }
    let mut out = Vec::new();
    let tails = emit(steps, &mut out, names);
    let end = out.len();
    for (i, slot) in tails {
        out[i].exits[slot].1 = end;
    }
    out
}

// ------------------------------------------------------------------ render

struct Program<'a> {
    ident: String,
    flat: Vec<FlatStep<'a>>,
    /// Externals used (globals), in first-use order.
    externals: BTreeSet<String>,
    /// Internal relays this program keeps (locals, with their initial).
    relays: BTreeMap<String, bool>,
    /// Steps with an on-delay (`Elapsed`): a `TON` each.
    timers: BTreeMap<String, String>,
    /// Edge triggers, `(instance, type, variable)`.
    triggers: BTreeMap<String, (&'static str, String)>,
    /// Remote-robot handshake flags.
    busy: BTreeSet<String>,
    /// Devices whose dispatch / index pulses this program raises:
    /// `(pulse variable, done variable)`.
    pulses: BTreeMap<String, String>,
    /// Devices this program positions (`move_to`, `goto`, `advance`): their
    /// `<device>_busy` latch — a done contact already high when the command
    /// is posted is the previous command's, so the wait is `busy AND done`,
    /// busy latching once the plant has reported not-done (a polled image
    /// refreshes after the scan that wrote the command).
    device_busy: BTreeSet<String>,
    /// The latch statements, one per waiting step and device, run every scan.
    latches: Vec<String>,
    body: String,
    notes: Vec<String>,
}

struct Writer<'a> {
    scene: &'a Scene,
    xc: &'a Exchange,
    options: &'a StOptions,
}

impl<'a> Writer<'a> {
    fn point(&self, name: &str, aspect: Option<Aspect>) -> Option<&'a ExchangePoint> {
        self.xc
            .points
            .iter()
            .find(|p| p.id.name == name && p.id.aspect == aspect)
    }

    /// The variable of a point (registered as an external), or of a
    /// relay the controller keeps itself.
    fn var(&self, p: &mut Program<'_>, name: &str, aspect: Option<Aspect>) -> String {
        if let Some(point) = self.point(name, aspect) {
            p.externals.insert(point.var.clone());
            return point.var.clone();
        }
        if aspect.is_none() && self.xc.internal.iter().any(|n| n == name) {
            let v = ident(name);
            let initial = self
                .scene
                .signals()
                .iter()
                .find(|s| s.name == name)
                .map(|s| s.initial)
                .unwrap_or(false);
            p.relays.entry(v.clone()).or_insert(initial);
            return v;
        }
        let fallback = match aspect {
            Some(a) => format!("{}_{}", ident(name), a.as_str()),
            None => ident(name),
        };
        p.notes.push(format!(
            "`{}` is not a point of host `{}`: written as `{fallback}`, undeclared",
            match aspect {
                Some(a) => format!("{name}.{}", a.as_str()),
                None => name.to_string(),
            },
            self.xc.host
        ));
        p.externals.insert(fallback.clone());
        fallback
    }

    #[allow(clippy::too_many_arguments)]
    fn condition(
        &self,
        p: &mut Program<'_>,
        c: &Condition,
        step: &str,
        started: &[String],
        stops: &BTreeMap<String, String>,
        commanded: &BTreeSet<String>,
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
            Condition::RobotDone { robot } | Condition::GroupDone { robot, .. } => {
                if self.point(robot, Some(Aspect::Done)).is_some() {
                    self.var(p, robot, Some(Aspect::Done))
                } else {
                    // The host's own robot: an idle test the controller
                    // answers itself.
                    "TRUE".to_string()
                }
            }
            Condition::Elapsed { seconds } => {
                let timer = format!("t_{step}");
                p.timers.insert(step.to_string(), st_time(*seconds));
                format!("{timer}.Q")
            }
            Condition::Signal { name, value } => {
                let v = self.var(p, name, None);
                if *value {
                    v
                } else {
                    format!("NOT {v}")
                }
            }
            Condition::Rising { name } => {
                let v = self.var(p, name, None);
                let inst = format!("r_{v}");
                p.triggers.insert(inst.clone(), ("R_TRIG", v));
                format!("{inst}.Q")
            }
            Condition::Falling { name } => {
                let v = self.var(p, name, None);
                let inst = format!("f_{v}");
                p.triggers.insert(inst.clone(), ("F_TRIG", v));
                format!("{inst}.Q")
            }
            // The in-position contact is the device's done point — or, for
            // a coil-driven axis, the limit switch of the stop last sent to
            // (any stop, if none was). A device this program positions is
            // waited on as `busy AND done`: the done still high from the
            // previous command is not this one's.
            Condition::DeviceDone { device } => {
                let done = match self.xc.devices.get(device) {
                    Some(DeviceClass::Axis {
                        stops: named,
                        coils,
                    }) if !coils.is_empty() => match stops.get(device) {
                        Some(stop) => self.var(p, &format!("{device}/{stop}"), None),
                        None => {
                            let lanes: Vec<String> = named
                                .iter()
                                .map(|(stop, _)| self.var(p, &format!("{device}/{stop}"), None))
                                .collect();
                            if lanes.len() == 1 {
                                lanes[0].clone()
                            } else {
                                format!("({})", lanes.join(" OR "))
                            }
                        }
                    },
                    _ => self.var(p, device, None),
                };
                if commanded.contains(device) {
                    let busy = format!("{}_busy", ident(device));
                    p.device_busy.insert(busy.clone());
                    p.latches.push(format!(
                        "IF s_{step} AND NOT {done} THEN {busy} := TRUE; END_IF;"
                    ));
                    format!("({busy} AND {done})")
                } else {
                    done
                }
            }
            Condition::All(cs) => {
                let parts: Vec<String> = cs
                    .iter()
                    .map(|c| self.condition(p, c, step, started, stops, commanded))
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
                    .map(|c| self.condition(p, c, step, started, stops, commanded))
                    .collect();
                if parts.len() == 1 {
                    parts[0].clone()
                } else {
                    format!("({})", parts.join(" OR "))
                }
            }
        }
    }

    /// A measurement as the controller's register holds it: the scaled
    /// INT of a word channel with a unit (commented with the SI value),
    /// else the REAL literal.
    fn word_literal(&self, device: &str, aspect: Aspect, x: f64) -> String {
        match self.point(device, Some(aspect)) {
            Some(point) if point.scale != 1.0 => format!(
                "{}; (* {} {} *)",
                point.to_register(Value::Real(x)).as_i64(),
                st_real(x),
                match aspect {
                    Aspect::Speed => "m/s",
                    _ => "m",
                }
            ),
            _ => format!("{};", st_real(x)),
        }
    }

    /// The coil levels that send a coil-driven axis to `stop`.
    fn coil_writes(
        &self,
        p: &mut Program<'_>,
        device: &str,
        coils: &[(String, String)],
        stops: &[(String, f64)],
        stop: &str,
        st: &mut Vec<String>,
    ) -> Result<(), StError> {
        if !stops.iter().any(|(n, _)| n == stop) {
            return Err(StError::Unsupported(format!(
                "device `{device}` has no stop `{stop}`"
            )));
        }
        if !coils.iter().any(|(_, s)| s == stop)
            && exchange::rest_stop(coils, stops).as_deref() != Some(stop)
        {
            return Err(StError::Unsupported(format!(
                "axis `{device}`: no coil drives to `{stop}` and it is not the spring-return stop"
            )));
        }
        // Already energised this way: no motion to wait for.
        let terms: Vec<String> = coils
            .iter()
            .map(|(coil, target)| {
                let v = self.var(p, &format!("{device}/{coil}"), None);
                if target == stop {
                    v
                } else {
                    format!("NOT {v}")
                }
            })
            .collect();
        st.push(self.preset_busy(p, device, &terms.join(" AND ")));
        for (coil, target) in coils {
            let v = self.var(p, &format!("{device}/{coil}"), None);
            st.push(format!(
                "{v} := {}; (* {stop} *)",
                if target == stop { "TRUE" } else { "FALSE" }
            ));
        }
        Ok(())
    }

    /// The statement that seeds a device's busy latch as a command is
    /// posted: `already` is the test that the command changes nothing (the
    /// plant will report no motion, so done is good as it stands).
    fn preset_busy(&self, p: &mut Program<'_>, device: &str, already: &str) -> String {
        let busy = format!("{}_busy", ident(device));
        p.device_busy.insert(busy.clone());
        format!("{busy} := {already};")
    }

    /// The entry statements of a step, the expressions its `Done` waits
    /// on, and the stops it sent coil-driven axes to.
    #[allow(clippy::type_complexity)]
    fn entry(
        &self,
        p: &mut Program<'_>,
        actions: &[Action],
        held: &mut BTreeMap<String, usize>,
    ) -> Result<(Vec<String>, Vec<String>, Vec<(String, String)>), StError> {
        let mut st = Vec::new();
        let mut started = Vec::new();
        let mut stops_sent: Vec<(String, String)> = Vec::new();
        for action in actions {
            match action {
                Action::Set { signal, value } => {
                    let v = self.var(p, signal, None);
                    st.push(format!("{v} := {};", if *value { "TRUE" } else { "FALSE" }));
                }
                Action::Device { device, command } => {
                    let magazine = self.scene.devices().iter().any(|d| {
                        &d.name == device
                            && matches!(d.kind, DeviceKind::Source { .. } | DeviceKind::Sink { .. })
                    });
                    if magazine && self.point(device, None).is_none() {
                        st.push(format!("(* {device}: feeder model, no I/O *)"));
                        continue;
                    }
                    match command {
                        DeviceCommand::Start => {
                            let v = self.var(p, device, None);
                            st.push(format!("{v} := TRUE;"));
                        }
                        DeviceCommand::Stop => {
                            let v = self.var(p, device, None);
                            st.push(format!("{v} := FALSE;"));
                        }
                        DeviceCommand::SetSpeed(speed) => {
                            let lit = self.word_literal(device, Aspect::Speed, *speed);
                            let v = self.var(p, device, Some(Aspect::Speed));
                            st.push(format!("{v} := {lit}"));
                        }
                        DeviceCommand::MoveTo(pos) => match self.xc.devices.get(device) {
                            Some(DeviceClass::Axis { stops, coils }) if !coils.is_empty() => {
                                let stop = stops
                                    .iter()
                                    .find(|(_, at)| (at - pos).abs() < 1e-6)
                                    .map(|(n, _)| n.clone())
                                    .ok_or_else(|| {
                                        StError::Unsupported(format!(
                                            "axis `{device}` is driven by coils: move_to a stop, not a position"
                                        ))
                                    })?;
                                self.coil_writes(p, device, coils, stops, &stop, &mut st)?;
                                stops_sent.push((device.clone(), stop));
                            }
                            _ => {
                                let lit = self.word_literal(device, Aspect::Position, *pos);
                                let v = self.var(p, device, Some(Aspect::Position));
                                let value = lit.split(';').next().unwrap_or("").to_string();
                                st.push(self.preset_busy(p, device, &format!("({v} = {value})")));
                                st.push(format!("{v} := {lit}"));
                            }
                        },
                        DeviceCommand::MoveToStop(stop) => match self.xc.devices.get(device) {
                            Some(DeviceClass::Axis { stops, coils }) if !coils.is_empty() => {
                                self.coil_writes(p, device, coils, stops, stop, &mut st)?;
                                stops_sent.push((device.clone(), stop.clone()));
                            }
                            class => {
                                let at = match class {
                                    Some(DeviceClass::Axis { stops, .. })
                                    | Some(DeviceClass::Lift { stops }) => {
                                        stops.iter().find(|(n, _)| n == stop).map(|(_, at)| *at)
                                    }
                                    _ => None,
                                }
                                .ok_or_else(|| {
                                    StError::Unsupported(format!(
                                        "device `{device}` has no stop `{stop}`"
                                    ))
                                })?;
                                let lit = self.word_literal(device, Aspect::Position, at);
                                let v = self.var(p, device, Some(Aspect::Position));
                                let value = lit.split(';').next().unwrap_or("").to_string();
                                st.push(self.preset_busy(p, device, &format!("({v} = {value})")));
                                st.push(format!("{v} := {} (* {stop} *)", lit));
                            }
                        },
                        DeviceCommand::Goto { station } => {
                            let index = match self.xc.devices.get(device) {
                                Some(DeviceClass::Vehicle { stations }) => {
                                    stations.iter().position(|s| s == station)
                                }
                                _ => None,
                            }
                            .ok_or_else(|| {
                                StError::Unsupported(format!(
                                    "vehicle `{device}` has no station `{station}`"
                                ))
                            })?;
                            let word = self.var(p, device, Some(Aspect::Station));
                            let go = self.var(p, device, Some(Aspect::Dispatch));
                            let done = self.var(p, device, None);
                            st.push(self.preset_busy(
                                p,
                                device,
                                &format!("({word} = {})", index + 1),
                            ));
                            st.push(format!("{word} := {}; (* {station} *)", index + 1));
                            st.push(format!("{go} := TRUE;"));
                            p.pulses.insert(go, done);
                        }
                        DeviceCommand::Advance(distance) => {
                            let go = self.var(p, device, Some(Aspect::Index));
                            let done = self.var(p, device, None);
                            st.push(self.preset_busy(p, device, "FALSE"));
                            st.push(format!(
                                "{go} := TRUE; (* advance {} m *)",
                                st_real(*distance)
                            ));
                            p.pulses.insert(go, done);
                        }
                    }
                }
                robot_action => {
                    let Some(r) = exchange::action_robot(self.scene, robot_action, held)
                        .map_err(StError::Unsupported)?
                    else {
                        continue;
                    };
                    let robot = self.scene.robots()[r].name.clone();
                    let key = exchange::action_key(robot_action).unwrap_or_default();
                    let label = exchange::action_label(robot_action);
                    let Some(entry) = self
                        .xc
                        .entries
                        .iter()
                        .find(|e| e.robot == robot && e.key == key)
                    else {
                        if self.scene.io_map().robot_controller_host(&robot) == self.xc.host {
                            // The host is the robot's own controller: the
                            // command is its own program, not a wire.
                            st.push(format!(
                                "(* {robot}: {label} — the controller's own program *)"
                            ));
                            continue;
                        }
                        return Err(StError::Unsupported(format!(
                            "robot `{robot}`: command `{key}` is not an entry of host `{}`",
                            self.xc.host
                        )));
                    };
                    // The variables are registered here (the handshake
                    // block below reads them back by robot); the entry only
                    // posts the command — `start` is raised once the robot
                    // reports ready, see `program()`.
                    let _start = self.var(p, &robot, Some(Aspect::Start));
                    let done = self.var(p, &robot, Some(Aspect::Done));
                    let busy = format!("{}_busy", ident(&robot));
                    let cmd = format!("{}_cmd", ident(&robot));
                    p.busy.insert(busy.clone());
                    st.push(format!("{busy} := FALSE;"));
                    st.push(format!("{cmd} := TRUE;"));
                    if self.point(&robot, Some(Aspect::Program)).is_some() {
                        let word = self.var(p, &robot, Some(Aspect::Program));
                        st.push(format!("{word} := {}; (* {label} *)", entry.number));
                    } else {
                        st.push(format!("(* {label} *)"));
                    }
                    started.push(format!("({busy} AND {done})"));
                }
            }
        }
        Ok((st, started, stops_sent))
    }

    fn program(&self, name: &str) -> Result<Program<'a>, StError> {
        let sequence = self
            .scene
            .sequence(name)
            .ok_or_else(|| StError::Exchange(format!("unknown sequence `{name}`")))?;
        let mut names = BTreeSet::new();
        let flat = flatten(&sequence.steps, &mut names);
        let mut p = Program {
            ident: ident(name),
            flat,
            externals: BTreeSet::new(),
            relays: BTreeMap::new(),
            timers: BTreeMap::new(),
            triggers: BTreeMap::new(),
            busy: BTreeSet::new(),
            pulses: BTreeMap::new(),
            device_busy: BTreeSet::new(),
            latches: Vec::new(),
            body: String::new(),
            notes: Vec::new(),
        };
        let n = p.flat.len();
        // The devices this program positions: their done waits are latched.
        let mut commanded: BTreeSet<String> = BTreeSet::new();
        crate::seq::walk_actions(&sequence.steps, &mut |action: &Action| {
            if let Action::Device {
                device,
                command:
                    DeviceCommand::MoveTo(_)
                    | DeviceCommand::MoveToStop(_)
                    | DeviceCommand::Goto { .. }
                    | DeviceCommand::Advance(_),
            } = action
            {
                commanded.insert(device.clone());
            }
            Ok::<(), String>(())
        })
        .map_err(StError::Unsupported)?;
        // The step word of a monitoring host: the flat index of the step
        // entered, written on every transition, so the plant records the
        // controller's own step bands (`IoNode::monitor`).
        let monitor = self
            .point(name, Some(Aspect::Step))
            .map(|_| self.var(&mut p, name, Some(Aspect::Step)));
        let report =
            |k: i64| -> Option<String> { monitor.as_ref().map(|word| format!("{word} := {k};")) };
        // Entry statements and `Done` operands of every step, in flat order
        // (a detach is attributed to the robot that attached before it).
        let mut held: BTreeMap<String, usize> = BTreeMap::new();
        #[allow(clippy::type_complexity)]
        let mut entries: Vec<(Vec<String>, Vec<String>, Vec<(String, String)>)> =
            Vec::with_capacity(n);
        // The stop each coil-driven axis was last sent to, as of each step
        // (flat order stands in for program order).
        let mut last_stops: Vec<BTreeMap<String, String>> = Vec::with_capacity(n);
        let mut so_far: BTreeMap<String, String> = BTreeMap::new();
        for k in 0..n {
            let actions = p.flat[k].actions;
            let entered = self.entry(&mut p, actions, &mut held)?;
            for (device, stop) in &entered.2 {
                so_far.insert(device.clone(), stop.clone());
            }
            last_stops.push(so_far.clone());
            entries.push(entered);
        }
        // Transitions, in flat order.
        let mut blocks: Vec<String> = Vec::with_capacity(n);
        for k in 0..n {
            let step_ident = p.flat[k].ident.clone();
            let flag = format!("s_{step_ident}");
            let exits: Vec<(&Condition, usize)> = p.flat[k].exits.clone();
            let branching = p.flat[k].branching;
            let started = entries[k].1.clone();
            let sent = last_stops[k].clone();
            let mut block = String::new();
            let mut others: Vec<String> = Vec::new();
            for (a, (condition, target)) in exits.iter().enumerate() {
                let mut cond =
                    self.condition(&mut p, condition, &step_ident, &started, &sent, &commanded);
                if branching
                    && matches!(condition, Condition::Immediately)
                    && a == exits.len() - 1
                    && !others.is_empty()
                {
                    cond = format!("NOT ({})", others.join(" OR "));
                }
                others.push(cond.clone());
                let keyword = if a == 0 { "IF" } else { "ELSIF" };
                let _ = writeln!(block, "{keyword} {flag} AND {cond} THEN");
                let _ = writeln!(block, "    {flag} := FALSE;");
                if *target >= n {
                    if self.options.cycle {
                        let first = format!("s_{}", p.flat[0].ident);
                        let _ = writeln!(block, "    {first} := TRUE; (* cycle *)");
                        if let Some(s) = report(0) {
                            let _ = writeln!(block, "    {s}");
                        }
                        for s in &entries[0].0 {
                            let _ = writeln!(block, "    {s}");
                        }
                    } else {
                        let _ = writeln!(block, "    (* end of cycle *)");
                        if let Some(s) = report(-1) {
                            let _ = writeln!(block, "    {s}");
                        }
                    }
                } else {
                    let next = format!("s_{}", p.flat[*target].ident);
                    let _ = writeln!(block, "    {next} := TRUE;");
                    if let Some(s) = report(*target as i64) {
                        let _ = writeln!(block, "    {s}");
                    }
                    for s in &entries[*target].0 {
                        let _ = writeln!(block, "    {s}");
                    }
                }
            }
            if !exits.is_empty() {
                let _ = writeln!(block, "END_IF;");
            }
            blocks.push(block);
        }
        // The body: triggers and timers every scan, the handshake
        // monitors, the first scan's entry, then the transitions.
        let mut body = String::new();
        if !p.triggers.is_empty() {
            let _ = writeln!(body, "(* edges, sampled every scan *)");
            for (inst, (_, var)) in &p.triggers {
                let _ = writeln!(body, "{inst}(CLK := {var});");
            }
        }
        if !p.timers.is_empty() {
            let _ = writeln!(body, "(* step timers *)");
            for (step, pt) in &p.timers {
                let _ = writeln!(body, "t_{step}(IN := s_{step}, PT := {pt});");
            }
        }
        let robots: Vec<String> = p.busy.iter().cloned().collect();
        if !robots.is_empty() || !p.pulses.is_empty() {
            let _ = writeln!(
                body,
                "(* handshakes: a command is issued once the robot reports ready, then held until the plant reads busy *)"
            );
        }
        for busy in &robots {
            let robot = busy.trim_end_matches("_busy");
            // The start / done variables of this robot, as registered.
            let start = self
                .xc
                .points
                .iter()
                .find(|q| ident(&q.id.name) == robot && q.id.aspect == Some(Aspect::Start))
                .map(|q| q.var.clone())
                .unwrap_or_else(|| format!("{robot}_start"));
            let done = self
                .xc
                .points
                .iter()
                .find(|q| ident(&q.id.name) == robot && q.id.aspect == Some(Aspect::Done))
                .map(|q| q.var.clone())
                .unwrap_or_else(|| format!("{robot}_done"));
            // Issue on ready, accept on busy: a command posted before the
            // controller has read its inputs even once (the image at
            // power-up reads low) waits for the first poll instead of
            // being dropped as "taken" before it was ever sent — the
            // OpenPLC run that lost its first motion to exactly that.
            let cmd = format!("{robot}_cmd");
            let _ = writeln!(
                body,
                "IF {cmd} AND NOT {start} AND {done} THEN {start} := TRUE; {cmd} := FALSE; END_IF;"
            );
            let _ = writeln!(
                body,
                "IF {start} AND NOT {done} THEN {start} := FALSE; {busy} := TRUE; END_IF;"
            );
        }
        for (go, done) in &p.pulses {
            let _ = writeln!(body, "IF {go} AND NOT {done} THEN {go} := FALSE; END_IF;");
        }
        if !p.latches.is_empty() {
            let _ = writeln!(
                body,
                "(* device waits: a done still high when the command was posted is the previous command's — busy latches once the plant reports not-done *)"
            );
            for latch in &p.latches {
                let _ = writeln!(body, "{latch}");
            }
        }
        let _ = writeln!(body, "(* first scan: enter the initial step *)");
        let _ = writeln!(body, "IF first THEN");
        let _ = writeln!(body, "    first := FALSE;");
        if n > 0 {
            if let Some(s) = report(0) {
                let _ = writeln!(body, "    {s}");
            }
            for s in &entries[0].0 {
                let _ = writeln!(body, "    {s}");
            }
        }
        let _ = writeln!(body, "END_IF;");
        let _ = writeln!(body, "(* transitions, in authored order *)");
        for block in blocks {
            body.push_str(&block);
        }
        p.body = body;
        Ok(p)
    }
}

/// Renders the ST file: the host's programs, and the configuration with
/// the exchange's points as located globals.
pub fn render_st(scene: &Scene, options: &StOptions) -> Result<String, StError> {
    let names: Vec<&str> = match &options.sequences {
        Some(list) => list.iter().map(String::as_str).collect(),
        None => scene.sequences().iter().map(|s| s.name.as_str()).collect(),
    };
    let xc = exchange::derive(scene, Some(&names), &options.host)
        .map_err(|e| StError::Exchange(e.to_string()))?;
    if xc.programs.is_empty() {
        let d = crate::iomap::derive(scene, Some(&names))
            .map_err(|e| StError::Exchange(e.to_string()))?;
        let mut hosts: Vec<String> = d.program_hosts.iter().map(|(_, h)| h.clone()).collect();
        hosts.sort();
        hosts.dedup();
        return Err(StError::NoPrograms {
            host: options.host.clone(),
            hosts,
        });
    }
    let writer = Writer {
        scene,
        xc: &xc,
        options,
    };
    let mut programs = Vec::new();
    for name in &xc.programs {
        programs.push(writer.program(name)?);
    }

    let mut out = String::new();
    let _ = writeln!(
        out,
        "(* botrail — Structured Text for host `{}`: {} *)",
        xc.host,
        xc.programs
            .iter()
            .map(|p| format!("`{p}`"))
            .collect::<Vec<_>>()
            .join(", ")
    );
    let _ = writeln!(
        out,
        "(* Variables are the I/O map's points of this host; addresses are the bindings on its PLC-family nodes. *)"
    );
    let _ = writeln!(
        out,
        "(* Robot commands: raise `start` once `done` reads high, hold it until `done` reads low, drop it, then wait for `done`; program words select the exchange's numbered entries. *)"
    );
    out.push('\n');

    // ---- programs ----------------------------------------------------------
    for p in &programs {
        let _ = writeln!(out, "PROGRAM {}", p.ident);
        if !p.externals.is_empty() {
            let _ = writeln!(out, "  VAR_EXTERNAL");
            for var in &p.externals {
                let ty = xc
                    .points
                    .iter()
                    .find(|q| &q.var == var)
                    .map(|q| q.value)
                    .unwrap_or(ValueKind::Bool);
                let _ = writeln!(out, "    {var} : {};", ty.as_str());
            }
            let _ = writeln!(out, "  END_VAR");
        }
        let _ = writeln!(out, "  VAR");
        let _ = writeln!(out, "    first : BOOL := TRUE;");
        for (k, step) in p.flat.iter().enumerate() {
            if k == 0 {
                let _ = writeln!(
                    out,
                    "    s_{} : BOOL := TRUE; (* initial step *)",
                    step.ident
                );
            } else {
                let _ = writeln!(out, "    s_{} : BOOL;", step.ident);
            }
        }
        for (relay, initial) in &p.relays {
            let _ = writeln!(
                out,
                "    {relay} : BOOL{}; (* relay *)",
                if *initial { " := TRUE" } else { "" }
            );
        }
        for step in p.timers.keys() {
            let _ = writeln!(out, "    t_{step} : TON;");
        }
        for (inst, (ty, _)) in &p.triggers {
            let _ = writeln!(out, "    {inst} : {ty};");
        }
        for busy in &p.busy {
            let _ = writeln!(out, "    {busy} : BOOL;");
            let _ = writeln!(out, "    {}_cmd : BOOL;", busy.trim_end_matches("_busy"));
        }
        for busy in &p.device_busy {
            let _ = writeln!(out, "    {busy} : BOOL; (* device wait latch *)");
        }
        let _ = writeln!(out, "  END_VAR");
        out.push('\n');
        for line in p.body.lines() {
            let _ = writeln!(out, "  {line}");
        }
        let _ = writeln!(out, "END_PROGRAM");
        for note in &p.notes {
            let _ = writeln!(out, "(* note: {note} *)");
        }
        out.push('\n');
    }

    // ---- configuration -----------------------------------------------------
    // `Config0` / `Res0`: the names OpenPLC's build expects; any other IDE
    // takes them as given.
    let _ = writeln!(out, "CONFIGURATION Config0");
    let _ = writeln!(out, "  RESOURCE Res0 ON PLC");
    let mut used: BTreeSet<String> = BTreeSet::new();
    for p in &programs {
        used.extend(p.externals.iter().cloned());
    }
    if !used.is_empty() {
        let _ = writeln!(out, "    VAR_GLOBAL");
        let mut seen: BTreeSet<String> = BTreeSet::new();
        for point in &xc.points {
            if !used.contains(&point.var) || !seen.insert(point.var.clone()) {
                continue;
            }
            let located = point
                .address
                .as_deref()
                .filter(|a| a.starts_with('%'))
                .filter(|_| {
                    point
                        .node
                        .as_deref()
                        .and_then(|n| scene.io_map().node(n))
                        .is_some_and(|n| {
                            matches!(
                                n.kind,
                                IoNodeKind::Plc | IoNodeKind::SafetyPlc | IoNodeKind::RemoteIo
                            )
                        })
                });
            let initial = match (point.value, point.side) {
                (ValueKind::Bool, _) if point.initial.as_bool() => " := TRUE".to_string(),
                _ => String::new(),
            };
            let _ = writeln!(
                out,
                "      {}{} : {}{}; (* {} {} — {}{} *)",
                point.var,
                located.map(|a| format!(" AT {a}")).unwrap_or_default(),
                point.value.as_str(),
                initial,
                point.label(),
                point.source.as_str(),
                match point.side {
                    PlantSide::Input => "the controller writes",
                    PlantSide::Output => "the controller reads",
                },
                point
                    .note
                    .as_deref()
                    .map(|n| format!("; {n}"))
                    .unwrap_or_default()
            );
        }
        for var in &used {
            if seen.contains(var) {
                continue;
            }
            let _ = writeln!(out, "      {var} : BOOL; (* not in the I/O map *)");
        }
        let _ = writeln!(out, "    END_VAR");
    }
    let _ = writeln!(
        out,
        "    TASK Main(INTERVAL := T#{}ms, PRIORITY := 0);",
        options.task_interval_ms
    );
    for p in &programs {
        let _ = writeln!(out, "    PROGRAM {}_inst WITH Main : {};", p.ident, p.ident);
    }
    let _ = writeln!(out, "  END_RESOURCE");
    let _ = writeln!(out, "END_CONFIGURATION");
    for note in &xc.notes {
        let _ = writeln!(out, "(* note: {note} *)");
    }
    Ok(out)
}
