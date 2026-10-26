//! Plant mode of the rollout — the cell driven by a controller outside it
//! (design-virtual-commissioning.md §4.2, VC1).
//!
//! A rollout opened with [`Scene::open_plant`] places some of its programs
//! on **external hosts**: those programs are not scanned. In their place
//! the plant takes the controller's writes through the host's
//! [`Exchange`] — a coil becomes a lane level, a run bit an edge that
//! starts or stops a device, a position word a move, a start pulse with a
//! program word the numbered robot command — and hands back the host's
//! outputs: sensor and limit-switch lanes, in-position and done contacts,
//! the handshake wires other programs write. Every program on an
//! *internal* host runs as it always did, so a cell can be half emulated
//! ("the PLC is real, the machine is virtual").
//!
//! The clock never looks at the wall: a tick is `tick(inputs) → outputs`,
//! and the inputs are recorded (`input_log`) so the same stream replays
//! the same timeline bit for bit. Pacing, protocols and the controller
//! itself live outside, in Python.
//!
//! **Loopback** keeps the external programs scanned by the rollout itself,
//! but every action they fire is encoded onto the exchange and decoded
//! back before it reaches the world — botrail acting as its own
//! controller through its own wiring. A loopback run must reproduce the
//! batch bake bit for bit; that it does is what proves the exchange
//! lossless and the I/O derivation complete.

use std::collections::{BTreeMap, BTreeSet};

use super::*;
use crate::exchange::{self, Encoding, Exchange, PlantSide, Value, ValueKind};
use crate::iomap::{self, IoPointId, IoSource};
use crate::seq::Fault;

/// When an injected fault takes hold.
#[derive(Debug, Clone, PartialEq)]
pub enum FaultAnchor {
    /// At once — a hand on the wire during a session (never authored:
    /// a scenario or a test case carries no absolute time).
    Now,
    /// On the next rising edge of a lane.
    Rising(String),
    /// When a program enters a step. The program must be one the plant
    /// scans (internal, or loopback) — an external controller's steps are
    /// its own.
    Step { program: String, step: String },
}

/// One input written on the plant: what a replay reproduces.
#[derive(Debug, Clone, PartialEq)]
pub struct InputRecord {
    pub t: f64,
    pub host: String,
    pub id: IoPointId,
    pub value: Value,
}

pub(super) struct PlantHost {
    pub name: String,
    pub exchange: Exchange,
    /// Every input's current value — what edges and words are read against.
    pub held: BTreeMap<IoPointId, Value>,
    /// Writes a controller handed in, to apply at this host's slot in the
    /// next scan.
    pub pending: Vec<(IoPointId, Value)>,
    /// Pulses the loopback encoder raised this scan; they drop when the
    /// next tick opens.
    pub pulses: Vec<IoPointId>,
    /// Robots whose command the plant accepted and whose `done` is held
    /// low until the robot is idle *and* the controller has dropped its
    /// `start` — the handshake a polled controller needs: a command that
    /// completes within one scan would otherwise never read as busy.
    pub pending_robots: BTreeSet<String>,
}

struct PendingFault {
    anchor: FaultAnchor,
    lanes: Vec<(String, bool)>,
}

pub(super) struct PlantState {
    pub hosts: Vec<PlantHost>,
    /// Per program (scan order): the external host it runs on, or `None`
    /// for a program the plant scans itself.
    pub program_host: Vec<Option<usize>>,
    pub loopback: bool,
    /// Programs the plant scans start over when they end (the PLCopen
    /// `jumpStep`): what a cell driven by a cycling controller needs.
    pub cycle: bool,
    pub log: Vec<InputRecord>,
    /// Hosts whose pending writes were applied this tick.
    pub applied: BTreeSet<usize>,
    /// Lane levels at the end of the previous scan (edge anchors).
    pub prev_lanes: Vec<bool>,
    pending_faults: Vec<PendingFault>,
    /// Robots whose joints a controller outside streams in (`robot index
    /// → idle`): the plant mirrors the stream and takes the controller's
    /// word for the robot's done / idle contact.
    pub streamed: BTreeMap<usize, bool>,
}

fn validation(message: String) -> SeqError {
    SeqError::Validation {
        step: None,
        message,
    }
}

/// Visits every transition condition of `steps`, arms included, in
/// authoring order.
fn walk_conditions(
    steps: &[Step],
    f: &mut impl FnMut(&Condition) -> Result<(), String>,
) -> Result<(), String> {
    fn visit(
        c: &Condition,
        f: &mut impl FnMut(&Condition) -> Result<(), String>,
    ) -> Result<(), String> {
        f(c)?;
        if let Condition::All(cs) | Condition::Any(cs) = c {
            for c in cs {
                visit(c, f)?;
            }
        }
        Ok(())
    }
    for step in steps {
        if step.select.is_empty() {
            visit(&step.transition, f)?;
        } else {
            for arm in &step.select {
                visit(&arm.condition, f)?;
            }
            for arm in &step.select {
                walk_conditions(&arm.steps, f)?;
            }
        }
    }
    Ok(())
}

impl Scene {
    /// Opens the cell as the **plant** of the controllers that run the
    /// programs on `external_hosts`: those programs are not scanned —
    /// their writes come in through [`LiveRollout::apply`] and their reads
    /// go out through [`LiveRollout::outputs`], both over the host's
    /// [`Exchange`] — while every other program runs as in the batch bake.
    /// The rollout stands at `t = 0` with the world built and the sensors
    /// read; the first [`LiveRollout::tick`] is the first scan (at
    /// `t = 0`), each later one a scan period on.
    ///
    /// `loopback` keeps the external programs scanned by the rollout,
    /// through the exchange: the reference run that must equal the bake.
    /// Without it, an external host that is a robot's own controller is
    /// refused — the plant cannot move a robot whose program it does not
    /// run (that needs a joint stream, VC5) — and so is a program that
    /// commands a magazine (a source or sink: cosmetic, no coil; declare
    /// it an output to give it one).
    #[allow(clippy::too_many_arguments)]
    pub fn open_plant(
        &self,
        names: &[&str],
        options: &RolloutOptions,
        backend: Option<Box<dyn PhysicsBackend>>,
        external_hosts: &[&str],
        loopback: bool,
        cycle: bool,
        streamed: &[&str],
    ) -> Result<LiveRollout, SeqError> {
        if external_hosts.is_empty() {
            return Err(validation(
                "open_plant needs at least one external host — the controller the plant answers to"
                    .to_string(),
            ));
        }
        let mut inner = self.prepare_rollout(names, options, backend, Vec::new())?;
        let d = iomap::derive(self, Some(names)).map_err(|e| validation(e.to_string()))?;
        let io = self.io_map();
        // A streamed robot: its controller is an external host whose joint
        // stream the plant mirrors (VC5), so its own program may run outside.
        let mut streamed_robots: BTreeMap<usize, bool> = BTreeMap::new();
        for rname in streamed {
            let Some(r) = self.robot_index(rname) else {
                return Err(validation(format!(
                    "no robot `{rname}` to stream joints for"
                )));
            };
            let ctrl = io.robot_controller_host(rname);
            if !external_hosts.contains(&ctrl.as_str()) {
                return Err(validation(format!(
                    "robot `{rname}` is controlled by `{ctrl}`, which is not an external host — a joint \
                     stream mirrors a controller outside the plant"
                )));
            }
            streamed_robots.insert(r, true);
        }
        let mut hosts: Vec<PlantHost> = Vec::new();
        for host in external_hosts {
            if hosts.iter().any(|h| h.name == *host) {
                return Err(validation(format!("host `{host}` is listed twice")));
            }
            let exchange =
                exchange::derive(self, Some(names), host).map_err(|e| validation(e.to_string()))?;
            // Every program on the host must cross the exchange whole.
            for program in &exchange.programs {
                let sequence = self
                    .sequence(program)
                    .ok_or_else(|| SeqError::UnknownSequence(program.clone()))?;
                let mut held: BTreeMap<String, usize> = BTreeMap::new();
                crate::seq::walk_actions(&sequence.steps, &mut |action: &Action| {
                    match action {
                        Action::Set { signal, .. } => {
                            if exchange.point(signal, None, PlantSide::Input).is_none()
                                && !exchange.internal.iter().any(|n| n == signal)
                            {
                                return Err(format!(
                                    "program `{program}` sets `{signal}`, which host `{host}` has no \
                                     output for"
                                ));
                            }
                        }
                        Action::Device { device, .. } => {
                            let magazine = self.devices().iter().any(|dv| {
                                &dv.name == device
                                    && matches!(
                                        dv.kind,
                                        DeviceKind::Source { .. } | DeviceKind::Sink { .. }
                                    )
                            });
                            if magazine {
                                if !loopback {
                                    return Err(format!(
                                        "program `{program}` commands `{device}`, a magazine with no \
                                         coil (cosmetic in the I/O map) — an external controller \
                                         cannot reach it; declare it an output (declare_io(..., \
                                         role=\"output\")) so it gets a run coil, or run the host in \
                                         loopback"
                                    ));
                                }
                            } else if !exchange.devices.contains_key(device) {
                                return Err(format!(
                                    "program `{program}` commands `{device}`, which host `{host}` has \
                                     no point for"
                                ));
                            }
                        }
                        robot_action => {
                            let r = match exchange::action_robot(self, robot_action, &mut held) {
                                Ok(Some(r)) => r,
                                // A release attributed at run time, or a non-robot action.
                                Ok(None) | Err(_) => return Ok(()),
                            };
                            let rname = &self.robots()[r].name;
                            if io.robot_controller_host(rname) == *host {
                                if !loopback && !streamed.contains(&rname.as_str()) {
                                    return Err(format!(
                                        "host `{host}` is the controller of robot `{rname}` and \
                                         program `{program}` drives it — the plant cannot move a \
                                         robot whose program it does not run; stream its joints \
                                         from the controller (joints=[\"{rname}\"], bt.vc.ursim), keep \
                                         the robot's controller internal, or run the host in loopback"
                                    ));
                                }
                                return Ok(());
                            }
                            let key = exchange::action_key(robot_action).unwrap_or_default();
                            if !exchange.entries.iter().any(|e| &e.robot == rname && e.key == key) {
                                return Err(format!(
                                    "program `{program}`: robot `{rname}` command `{key}` is not an \
                                     entry of host `{host}`"
                                ));
                            }
                            if exchange
                                .point(rname, Some(iomap::Aspect::Start), PlantSide::Input)
                                .is_none()
                            {
                                return Err(format!(
                                    "program `{program}` drives robot `{rname}` but host `{host}` has \
                                     no `{rname}.start` output"
                                ));
                            }
                        }
                    }
                    Ok(())
                })
                .map_err(validation)?;
                walk_conditions(&sequence.steps, &mut |condition: &Condition| {
                    match condition {
                        Condition::Signal { name, .. }
                        | Condition::Rising { name }
                        | Condition::Falling { name } => {
                            if exchange.point(name, None, PlantSide::Output).is_none()
                                && !exchange.internal.iter().any(|n| n == name)
                            {
                                return Err(format!(
                                    "program `{program}` reads `{name}`, which host `{host}` has no \
                                     input for"
                                ));
                            }
                        }
                        Condition::DeviceDone { device } => {
                            // A coil-driven axis's done is its limit
                            // switches, `<axis>/<stop>`.
                            let switches = exchange
                                .outputs()
                                .any(|p| p.id.aspect.is_none() && p.id.name.starts_with(&format!("{device}/")));
                            if exchange.point(device, None, PlantSide::Output).is_none() && !switches {
                                return Err(format!(
                                    "program `{program}` awaits `{device}`, which host `{host}` has \
                                     no in-position input for"
                                ));
                            }
                        }
                        Condition::RobotDone { robot } | Condition::GroupDone { robot, .. } => {
                            if io.robot_controller_host(robot) != *host
                                && exchange
                                    .point(robot, Some(iomap::Aspect::Done), PlantSide::Output)
                                    .is_none()
                            {
                                return Err(format!(
                                    "program `{program}` waits for robot `{robot}`, which host \
                                     `{host}` has no done input for"
                                ));
                            }
                        }
                        Condition::Immediately
                        | Condition::Done
                        | Condition::Elapsed { .. }
                        | Condition::All(_)
                        | Condition::Any(_) => {}
                    }
                    Ok(())
                })
                .map_err(validation)?;
            }
            let held = exchange
                .inputs()
                .map(|p| (p.id.clone(), p.initial))
                .collect();
            hosts.push(PlantHost {
                name: host.to_string(),
                exchange,
                held,
                pending: Vec::new(),
                pulses: Vec::new(),
                pending_robots: BTreeSet::new(),
            });
        }
        let program_host: Vec<Option<usize>> = names
            .iter()
            .map(|name| {
                d.program_hosts
                    .iter()
                    .find(|(p, _)| p == name)
                    .and_then(|(_, h)| hosts.iter().position(|x| &x.name == h))
            })
            .collect();
        if program_host.iter().all(Option::is_none) {
            return Err(validation(format!(
                "no program of {:?} runs on {:?} — the external hosts run nothing here (hosts of this \
                 program set: {})",
                names,
                external_hosts,
                d.program_hosts
                    .iter()
                    .map(|(p, h)| format!("{p} on {h}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            )));
        }
        let mut log = Vec::new();
        for host in &hosts {
            for p in host.exchange.inputs() {
                log.push(InputRecord {
                    t: 0.0,
                    host: host.name.clone(),
                    id: p.id.clone(),
                    value: p.initial,
                });
            }
        }
        inner.plant = Some(PlantState {
            hosts,
            program_host,
            loopback,
            cycle,
            log,
            applied: BTreeSet::new(),
            prev_lanes: Vec::new(),
            pending_faults: Vec::new(),
            streamed: streamed_robots.clone(),
        });
        inner.start_world()?;
        for &r in streamed_robots.keys() {
            inner.plant_start_stream(r)?;
        }
        Ok(LiveRollout {
            inner,
            controls: Vec::new(),
            pending_start: true,
        })
    }
}

impl Rollout {
    /// Whether program `p` belongs to an external controller the plant
    /// does not scan.
    pub(super) fn plant_skips_scan(&self, p: usize) -> bool {
        self.plant
            .as_ref()
            .is_some_and(|s| !s.loopback && s.program_host.get(p).copied().flatten().is_some())
    }

    /// Whether program `p` is scanned through the exchange (loopback).
    pub(super) fn plant_loopback_program(&self, p: usize) -> bool {
        self.plant
            .as_ref()
            .is_some_and(|s| s.loopback && s.program_host.get(p).copied().flatten().is_some())
    }

    /// Opens a tick on the plant side: the pulses raised during the last
    /// scan drop, stamped at the new clock.
    pub(super) fn plant_begin_tick(&mut self) {
        let t = self.t;
        let Some(state) = self.plant.as_mut() else {
            return;
        };
        state.applied.clear();
        for h in 0..state.hosts.len() {
            let pulses = std::mem::take(&mut state.hosts[h].pulses);
            for id in pulses {
                let host = &mut state.hosts[h];
                let kind = host
                    .exchange
                    .by_id(&id)
                    .map(|p| p.value)
                    .unwrap_or(ValueKind::Bool);
                let value = Value::Bool(false).coerced(kind);
                host.held.insert(id.clone(), value);
                let name = host.name.clone();
                state.log.push(InputRecord {
                    t,
                    host: name,
                    id,
                    value,
                });
            }
        }
    }

    /// Applies the controller's pending writes at this program's slot in
    /// the scan — once per tick per host, at the first of the host's
    /// programs in scan order — so a write is seen by the programs after
    /// the slot in this scan and by those before it in the next, exactly
    /// as the program's own `set` would have been.
    pub(super) fn plant_apply_pending(&mut self, p: usize) -> Result<(), SeqError> {
        let Some(h) = self
            .plant
            .as_ref()
            .and_then(|s| s.program_host.get(p).copied().flatten())
        else {
            return Ok(());
        };
        let pending = {
            let state = self.plant.as_mut().expect("plant mode");
            if !state.applied.insert(h) {
                return Ok(());
            }
            std::mem::take(&mut state.hosts[h].pending)
        };
        for (id, value) in pending {
            self.plant_write(h, &id, value, false)?;
        }
        Ok(())
    }

    /// Writes one input on host `h` now: records the change, decodes it,
    /// fires what it commands. A `pulse` the loopback encoder raises on a
    /// point already high is dropped and raised again, so two commands in
    /// one scan both edge.
    pub(super) fn plant_write(
        &mut self,
        h: usize,
        id: &IoPointId,
        value: Value,
        pulse: bool,
    ) -> Result<(), SeqError> {
        let t = self.t;
        let actions = {
            let state = self.plant.as_mut().expect("plant mode");
            let host = &mut state.hosts[h];
            let Some(point) = host
                .exchange
                .by_id(id)
                .filter(|p| p.side == PlantSide::Input)
            else {
                return Err(validation(format!(
                    "`{}` is not an input of host `{}`",
                    id.label(),
                    host.name
                )));
            };
            let (kind, initial) = (point.value, point.initial);
            let new = value.coerced(kind);
            let mut prev = host.held.get(id).copied().unwrap_or(initial);
            if pulse && new.as_bool() && prev.as_bool() {
                prev = Value::Bool(false).coerced(kind);
                host.held.insert(id.clone(), prev);
                state.log.push(InputRecord {
                    t,
                    host: host.name.clone(),
                    id: id.clone(),
                    value: prev,
                });
            }
            if prev == new {
                return Ok(());
            }
            let decoded = host
                .exchange
                .decode(id, &prev, &new, &host.held)
                .map_err(|m| validation(format!("host `{}`: {m}", host.name)))?;
            host.held.insert(id.clone(), new);
            state.log.push(InputRecord {
                t,
                host: host.name.clone(),
                id: id.clone(),
                value: new,
            });
            if !decoded.is_empty()
                && matches!(point.source, IoSource::RobotStart | IoSource::RobotProgram)
            {
                host.pending_robots.insert(id.name.clone());
            }
            if point.source == IoSource::Monitor {
                let step = new.as_i64();
                self.plant_monitor_step(h, &id.name, step);
                return Ok(());
            }
            decoded
        };
        for action in &actions {
            self.fire(action)?;
        }
        Ok(())
    }

    /// The controller that runs program `program` on host `h` reports its
    /// active step: flat index `step`, or a negative number for none (the
    /// program parked at its end, or not started). The plant records it
    /// the way it records its own programs' steps — a span per step, the
    /// arm of a select as a decision — so the dock and the SFC chart show
    /// the controller's program as they show the cell's. A step the
    /// program never had is ignored (another program's numbering).
    pub(super) fn plant_monitor_step(&mut self, h: usize, program: &str, step: i64) {
        let Some(p) = self
            .programs
            .iter()
            .position(|q| q.sequence.name == program)
        else {
            return;
        };
        let external = self
            .plant
            .as_ref()
            .is_some_and(|s| !s.loopback && s.program_host.get(p).copied().flatten() == Some(h));
        if !external {
            return;
        }
        let t = self.t;
        let n = self.programs[p].flat.len();
        let next = if step < 0 { n } else { step as usize };
        if next > n {
            return;
        }
        // Close the span the controller reported before this one.
        let open = self.programs[p].open_span;
        if let Some(span) = self.step_spans.get_mut(open) {
            if span.sequence == program && span.end < t {
                span.end = t;
            }
        }
        if next == n {
            let prog = &mut self.programs[p];
            prog.step = n;
            prog.open_span = usize::MAX;
            return;
        }
        // Entering an arm of a select is that select's decision.
        let decision = self.programs[p].flat.iter().find_map(|flat| {
            let select = flat.select?;
            flat.exits
                .iter()
                .position(|(_, target)| *target == next)
                .map(|arm| (flat.name.clone(), select, arm))
        });
        if let Some((name, select, arm)) = decision {
            let repeat = self.branches.last().is_some_and(|b| {
                b.sequence == program && b.select == select && b.arm == arm && b.step == name
            });
            if !repeat {
                self.branches.push(BranchTaken {
                    sequence: program.to_string(),
                    step: name,
                    select,
                    arm,
                });
            }
        }
        let name = self.step_name_in(p, next);
        let prog = &mut self.programs[p];
        prog.step = next;
        prog.entered_at = t;
        prog.open_span = self.step_spans.len();
        self.step_spans.push(StepSpan {
            name,
            start: t,
            end: t,
            sequence: program.to_string(),
            step: next,
            group: None,
        });
    }

    /// Whether the programs the plant scans start over when they end.
    pub(super) fn plant_cycles(&self) -> bool {
        self.plant.as_ref().is_some_and(|s| s.cycle)
    }

    /// Takes robot `r`'s joints for the controller's stream: an external
    /// drive with no rate limit — the joints are the controller's, the
    /// plant only mirrors them — that `command_robot` sets every tick.
    pub(super) fn plant_start_stream(&mut self, r: usize) -> Result<(), SeqError> {
        self.start_drive(r, None, None, "stream")?;
        if let Some(active) = self.robots[r].active.iter_mut().find(|a| a.is_external()) {
            if let MoveKind::External { vmax, .. } = &mut active.kind {
                vmax.iter_mut().for_each(|v| *v = f64::INFINITY);
            }
        }
        Ok(())
    }

    /// The idle word of a streamed robot's controller, or `None` for a
    /// robot the plant drives itself.
    pub(super) fn plant_stream_idle(&self, r: usize) -> Option<bool> {
        self.plant
            .as_ref()
            .and_then(|s| s.streamed.get(&r).copied())
    }

    /// Records what a streamed robot's controller says about itself.
    pub(super) fn plant_report_idle(&mut self, r: usize, idle: bool) -> Result<(), SeqError> {
        let name = self.world.robots()[r].name.clone();
        match self.plant.as_mut().and_then(|s| s.streamed.get_mut(&r)) {
            Some(flag) => {
                *flag = idle;
                Ok(())
            }
            None => Err(self.live_err(format!(
                "robot `{name}` is not streamed — open the plant with joints=[\"{name}\"]"
            ))),
        }
    }

    /// Loopback: the current (external) program's action, encoded onto
    /// its host's exchange and decoded back into the world.
    pub(super) fn fire_through_exchange(&mut self, action: &Action) -> Result<(), SeqError> {
        let p = self.current;
        let h = self
            .plant
            .as_ref()
            .and_then(|s| s.program_host.get(p).copied().flatten())
            .expect("a loopback program has a host");
        let encoding = self.plant.as_ref().expect("plant mode").hosts[h]
            .exchange
            .encode(action, &self.world)
            .map_err(|m| self.live_err(m))?;
        match encoding {
            Encoding::Direct => self.fire(action),
            Encoding::Exchange(enc) => {
                for (id, v) in &enc.writes {
                    self.plant_write(h, id, *v, false)?;
                }
                for id in &enc.pulses {
                    self.plant_write(h, id, Value::Bool(true), true)?;
                    self.plant.as_mut().expect("plant mode").hosts[h]
                        .pulses
                        .push(id.clone());
                }
                Ok(())
            }
        }
    }

    /// The host's outputs as the world stands: lanes, in-position and done
    /// contacts.
    pub(super) fn plant_outputs(&self, h: usize) -> Vec<(IoPointId, Value)> {
        let host = &self.plant.as_ref().expect("plant mode").hosts[h];
        host.exchange
            .outputs()
            .map(|p| {
                let v = match p.source {
                    IoSource::DeviceDone => {
                        Value::Bool(self.condition_holds(&Condition::DeviceDone {
                            device: p.id.name.clone(),
                        }))
                    }
                    // A streamed robot's done is its controller's word; the
                    // plant's own robots report idle when no move drives them.
                    IoSource::RobotDone => Value::Bool(
                        !host.pending_robots.contains(&p.id.name)
                            && self.condition_holds(&Condition::RobotDone {
                                robot: p.id.name.clone(),
                            }),
                    ),
                    // The mirror of another host's start / program on a
                    // robot controller: not modelled on the plant side.
                    IoSource::RobotStart | IoSource::RobotProgram => p.initial,
                    // A word the plant records (an encoder, an id reader).
                    _ if p.id.aspect.is_some() => self
                        .words
                        .iter()
                        .find(|w| w.name == p.label())
                        .and_then(|w| w.samples.last().map(|s| Value::Real(s.1)))
                        .unwrap_or(p.initial),
                    _ => Value::Bool(
                        self.lane_index(&p.id.name)
                            .map(|l| self.lane_value(l))
                            .unwrap_or(false),
                    ),
                };
                (p.id.clone(), p.to_register(v))
            })
            .collect()
    }

    /// Pins a lane to `value` from now on (a PLC force): sensor geometry
    /// and program writes are dropped until [`plant_release`](Self::plant_release).
    pub(super) fn plant_force(&mut self, lane: &str, value: bool) -> Result<(), SeqError> {
        let l = self.lane_index(lane).ok_or_else(|| {
            validation(format!(
                "`{lane}` is not a lane (a sensor, an internal signal, a device's running state)"
            ))
        })?;
        let before = self.lane_value(l);
        self.forced.retain(|(x, _)| *x != l);
        self.forced.push((l, value));
        let t = self.t;
        let track = &mut self.signals[l];
        // A forced input is a level from this scan on: an edge the lane
        // took in this very scan is overwritten, not followed, so a contact
        // pinned at t = 0 reads exactly like a scenario's fault.
        match track.edges.last_mut() {
            Some((at, v)) if (*at - t).abs() < 1e-12 => *v = value,
            Some((_, v)) if *v == value => {}
            _ => track.edges.push((t, value)),
        }
        while track.edges.len() >= 2
            && track.edges[track.edges.len() - 1].1 == track.edges[track.edges.len() - 2].1
        {
            track.edges.pop();
        }
        // A forced coil is a real level on the plant: the grasp it drives
        // follows it like any other edge.
        if self.lane_value(l) != before {
            self.grasp_follow(l)?;
        }
        Ok(())
    }

    pub(super) fn plant_release(&mut self, lane: &str) -> Result<(), SeqError> {
        let l = self
            .lane_index(lane)
            .ok_or_else(|| validation(format!("`{lane}` is not a lane")))?;
        self.forced.retain(|(x, _)| *x != l);
        Ok(())
    }

    /// Injects a fault: resolved through the scene's bindings like a
    /// scenario's, then forced at its anchor.
    pub(super) fn plant_inject(&mut self, fault: Fault, at: FaultAnchor) -> Result<(), SeqError> {
        let lanes = self
            .world
            .resolve_faults("injected", &[fault])
            .map_err(|e| validation(e.to_string()))?;
        match at {
            FaultAnchor::Now => {
                for (name, value) in lanes {
                    self.plant_force(&name, value)?;
                }
                Ok(())
            }
            anchor => {
                if let FaultAnchor::Step { program, step } = &anchor {
                    let p = self
                        .programs
                        .iter()
                        .position(|x| &x.sequence.name == program)
                        .ok_or_else(|| validation(format!("unknown program `{program}`")))?;
                    if self.plant_skips_scan(p) {
                        return Err(validation(format!(
                            "program `{program}` runs on an external controller — the plant does not \
                             see its steps; anchor on a lane edge instead"
                        )));
                    }
                    if !self.programs[p].flat.iter().any(|s| &s.name == step) {
                        return Err(validation(format!(
                            "program `{program}` has no step `{step}`"
                        )));
                    }
                }
                if let FaultAnchor::Rising(name) = &anchor {
                    if self.lane_index(name).is_none() {
                        return Err(validation(format!("`{name}` is not a lane")));
                    }
                }
                let state = self.plant.as_mut().ok_or_else(|| {
                    validation("anchored injection needs a plant rollout".to_string())
                })?;
                state.pending_faults.push(PendingFault { anchor, lanes });
                Ok(())
            }
        }
    }

    /// Between the sensor read and the scans: an anchored fault whose
    /// moment the plant now observes — a lane that rose since the previous
    /// scan closed (a sensor this tick, a program's write last tick), a
    /// step a program entered last scan — takes hold before any program
    /// scans this tick, so the scan that sees the edge also sees the wire
    /// as broken.
    pub(super) fn plant_after_sensors(&mut self) -> Result<(), SeqError> {
        if self.plant.is_none() {
            return Ok(());
        }
        let lanes_now: Vec<bool> = (0..self.signals.len())
            .map(|l| self.lane_value(l))
            .collect();
        let previous_scan = self.t - self.options.dt;
        let due: Vec<usize> = {
            let state = self.plant.as_ref().expect("plant mode");
            state
                .pending_faults
                .iter()
                .enumerate()
                .filter(|(_, f)| match &f.anchor {
                    FaultAnchor::Now => true,
                    FaultAnchor::Rising(name) => self.lane_index(name).is_some_and(|l| {
                        lanes_now[l] && !state.prev_lanes.get(l).copied().unwrap_or(lanes_now[l])
                    }),
                    FaultAnchor::Step { program, step } => self.programs.iter().any(|p| {
                        &p.sequence.name == program
                            && p.flat.get(p.step).is_some_and(|s| &s.name == step)
                            && (p.entered_at - previous_scan).abs() < 1e-9
                    }),
                })
                .map(|(i, _)| i)
                .collect()
        };
        let mut lanes_to_force = Vec::new();
        {
            let state = self.plant.as_mut().expect("plant mode");
            for i in due.iter().rev() {
                lanes_to_force.extend(state.pending_faults.remove(*i).lanes);
            }
        }
        for (name, value) in lanes_to_force {
            self.plant_force(&name, value)?;
        }
        Ok(())
    }

    /// Closes a scan on the plant side: the lane snapshot the next edge
    /// test compares to.
    pub(super) fn plant_end_scan(&mut self) -> Result<(), SeqError> {
        if self.plant.is_none() {
            return Ok(());
        }
        let lanes_now: Vec<bool> = (0..self.signals.len())
            .map(|l| self.lane_value(l))
            .collect();
        // A pending robot's `done` comes back once it is idle and its
        // controller has dropped `start` — so the controller has seen the
        // command taken, however slowly it polls.
        let released: Vec<(usize, String)> = {
            let state = self.plant.as_ref().expect("plant mode");
            let mut out = Vec::new();
            for (h, host) in state.hosts.iter().enumerate() {
                for robot in &host.pending_robots {
                    let start = IoPointId {
                        name: robot.clone(),
                        aspect: Some(iomap::Aspect::Start),
                        direction: iomap::IoDirection::Output,
                    };
                    let start_high = host.held.get(&start).is_some_and(Value::as_bool);
                    let idle = self.condition_holds(&Condition::RobotDone {
                        robot: robot.clone(),
                    });
                    if idle && !start_high {
                        out.push((h, robot.clone()));
                    }
                }
            }
            out
        };
        let state = self.plant.as_mut().expect("plant mode");
        for (h, robot) in released {
            state.hosts[h].pending_robots.remove(&robot);
        }
        state.prev_lanes = lanes_now;
        Ok(())
    }
}

impl LiveRollout {
    /// Whether this rollout was opened as a plant ([`Scene::open_plant`]).
    pub fn is_plant(&self) -> bool {
        self.inner.plant.is_some()
    }

    /// The external hosts, in the order given.
    pub fn plant_hosts(&self) -> Vec<String> {
        self.inner
            .plant
            .as_ref()
            .map(|s| s.hosts.iter().map(|h| h.name.clone()).collect())
            .unwrap_or_default()
    }

    fn host_index(&self, host: &str) -> Result<usize, SeqError> {
        let state = self.inner.plant.as_ref().ok_or_else(|| {
            validation("not a plant rollout (open it with open_plant)".to_string())
        })?;
        state
            .hosts
            .iter()
            .position(|h| h.name == host)
            .ok_or_else(|| {
                validation(format!(
                    "`{host}` is not an external host of this plant ({})",
                    state
                        .hosts
                        .iter()
                        .map(|h| h.name.as_str())
                        .collect::<Vec<_>>()
                        .join(", ")
                ))
            })
    }

    /// The exchange of an external host.
    pub fn exchange(&self, host: &str) -> Option<&Exchange> {
        self.inner
            .plant
            .as_ref()
            .and_then(|s| s.hosts.iter().find(|h| h.name == host))
            .map(|h| &h.exchange)
    }

    /// The host's outputs as the world stands after the last tick — what
    /// its controller reads this scan.
    pub fn outputs(&self, host: &str) -> Result<Vec<(IoPointId, Value)>, SeqError> {
        let h = self.host_index(host)?;
        Ok(self.inner.plant_outputs(h))
    }

    /// Hands the controller's writes to the plant, to take effect at the
    /// host's slot in the next scan: levels, words, and start pulses as the
    /// level they are on the controller (the plant edges them). Unchanged
    /// values are harmless; an unknown label is refused by name.
    pub fn apply(&mut self, host: &str, inputs: Vec<(IoPointId, Value)>) -> Result<(), SeqError> {
        let h = self.host_index(host)?;
        let state = self.inner.plant.as_mut().expect("plant mode");
        let plant_host = &mut state.hosts[h];
        for (id, _) in &inputs {
            if plant_host
                .exchange
                .by_id(id)
                .filter(|p| p.side == PlantSide::Input)
                .is_none()
            {
                return Err(validation(format!(
                    "`{}` is not an input of host `{host}` (inputs: {})",
                    id.label(),
                    plant_host
                        .exchange
                        .inputs()
                        .map(|p| p.label())
                        .collect::<Vec<_>>()
                        .join(", ")
                )));
            }
        }
        // Words before contacts: a program word written with its start
        // pulse must be read before the pulse edges, whatever order the
        // controller's image came in.
        let (words, bools): (Vec<_>, Vec<_>) = inputs
            .into_iter()
            .partition(|(_, v)| !matches!(v, Value::Bool(_)));
        plant_host.pending.extend(words);
        plant_host.pending.extend(bools);
        Ok(())
    }

    /// Pins a lane (a PLC force) until [`unforce_lane`](Self::unforce_lane).
    pub fn force_lane(&mut self, lane: &str, value: bool) -> Result<(), SeqError> {
        self.inner.plant_force(lane, value)
    }

    pub fn unforce_lane(&mut self, lane: &str) -> Result<(), SeqError> {
        self.inner.plant_release(lane)
    }

    /// Injects a fault (a stuck contact, an open wire, a node down) at an
    /// anchor.
    /// What a streamed robot's controller says about itself: idle (no
    /// program running) or busy. The plant's `done` contact for the robot,
    /// and every `robot_done` wait on it, read this word.
    pub fn report_idle(&mut self, robot: usize, idle: bool) -> Result<(), SeqError> {
        self.inner.plant_report_idle(robot, idle)
    }

    /// The robots whose joints a controller streams in.
    pub fn streamed_robots(&self) -> Vec<usize> {
        self.inner
            .plant
            .as_ref()
            .map(|s| s.streamed.keys().copied().collect())
            .unwrap_or_default()
    }

    pub fn inject(&mut self, fault: Fault, at: FaultAnchor) -> Result<(), SeqError> {
        self.inner.plant_inject(fault, at)
    }

    /// Every input written so far, in order — the stream a replay feeds
    /// back.
    pub fn input_log(&self) -> &[InputRecord] {
        self.inner
            .plant
            .as_ref()
            .map(|s| s.log.as_slice())
            .unwrap_or(&[])
    }
}

#[cfg(test)]
mod tests {
    use nalgebra::{Point3, Vector3};

    use super::*;
    use crate::iomap::{IoDirection, IoNode, IoNodeKind};
    use crate::rollout::tests::sample_scene;
    use crate::seq::{Device, DeviceCommand, Sensor, SensorKind, SensorWatch, Sequence, Step};
    use crate::Geometry;

    fn iso(x: f64, y: f64, z: f64) -> Isometry3<f64> {
        Isometry3::translation(x, y, z)
    }

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.into(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    /// A belt feeding a box to a beam, run by a program on a declared PLC
    /// — the controller the plant answers to.
    fn feed_cell() -> Scene {
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
        scene.define_signal("fed", false);
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
                    vec![
                        Action::Device {
                            device: "conv".into(),
                            command: DeviceCommand::Stop,
                        },
                        Action::Set {
                            signal: "fed".into(),
                            value: true,
                        },
                    ],
                    Condition::Elapsed { seconds: 0.1 },
                ),
            ],
        });
        scene
            .upsert_io_node(IoNode {
                name: "PLC1".into(),
                kind: IoNodeKind::Plc,
                programs: vec!["feed".into()],
                uplink: None,
                channels: Vec::new(),
                place: None,
                model: None,
                monitor: false,
            })
            .unwrap();
        scene
    }

    fn lanes(tl: &SequenceTimeline) -> Vec<(String, Vec<(f64, bool)>)> {
        tl.signals
            .iter()
            .map(|s| (s.name.clone(), s.edges.clone()))
            .collect()
    }

    fn spans(tl: &SequenceTimeline) -> Vec<(String, f64, f64)> {
        tl.step_spans
            .iter()
            .map(|s| (s.name.clone(), s.start, s.end))
            .collect()
    }

    #[test]
    fn the_exchange_turns_the_hosts_points_round() {
        let scene = feed_cell();
        let xc = exchange::derive(&scene, Some(&["feed"]), "PLC1").unwrap();
        assert_eq!(xc.programs, vec!["feed".to_string()]);
        let by_label: BTreeMap<String, &exchange::ExchangePoint> =
            xc.points.iter().map(|p| (p.label(), p)).collect();
        assert_eq!(by_label["beam"].side, PlantSide::Output);
        assert_eq!(by_label["conv"].side, PlantSide::Input);
        assert_eq!(by_label["conv"].var, "conv_run");
        assert_eq!(by_label["conv"].initial, Value::Bool(false));
        // `fed` is written and never read: a coil the PLC drives.
        assert_eq!(by_label["fed"].side, PlantSide::Input);
        assert!(xc.entries.is_empty());
        let err = exchange::derive(&scene, Some(&["feed"]), "PLC9").unwrap_err();
        assert!(err.to_string().contains("PLC1"), "{err}");
    }

    #[test]
    fn loopback_through_the_exchange_is_the_bake_bit_for_bit() {
        let scene = feed_cell();
        let options = RolloutOptions::default();
        let bake = scene.simulate_sequence("feed", &options).unwrap();
        let mut live = scene
            .open_plant(&["feed"], &options, None, &["PLC1"], true, false, &[])
            .unwrap();
        while !live.finished() {
            live.tick().unwrap();
        }
        let log: Vec<InputRecord> = live.input_log().to_vec();
        let tl = live.finish();
        assert_eq!(tl.duration, bake.duration);
        assert_eq!(lanes(&tl), lanes(&bake));
        assert_eq!(spans(&tl), spans(&bake));
        assert_eq!(format!("{:?}", tl.objects), format!("{:?}", bake.objects));
        // The log is the controller's side: the run coil rose at t = 0 and
        // fell when the beam tripped, the `fed` coil rose with it.
        let conv: Vec<&InputRecord> = log.iter().filter(|r| r.id.name == "conv").collect();
        assert_eq!(conv.len(), 3, "{conv:?}"); // initial, start, stop
        assert_eq!(conv[1].value, Value::Bool(true));
        assert_eq!(conv[2].value, Value::Bool(false));
        let stop = bake.step_spans[0].end;
        assert!((conv[2].t - stop).abs() < 1e-9);
        assert!(log
            .iter()
            .any(|r| r.id.name == "fed" && r.value == Value::Bool(true)));
    }

    #[test]
    fn a_replayed_log_drives_the_plant_down_the_same_path() {
        let scene = feed_cell();
        let options = RolloutOptions::default();
        let mut reference = scene
            .open_plant(&["feed"], &options, None, &["PLC1"], true, false, &[])
            .unwrap();
        while !reference.finished() {
            reference.tick().unwrap();
        }
        let log: Vec<InputRecord> = reference.input_log().to_vec();
        let reference = reference.finish();

        // The PLC is outside now: nothing scans `feed`; its writes come from
        // the log, at the scan they were made in.
        let mut plant = scene
            .open_plant(&["feed"], &options, None, &["PLC1"], false, false, &[])
            .unwrap();
        let mut cursor = 0;
        let mut first = true;
        while plant.t() + 1e-9 < reference.duration || first {
            let next = if first { 0.0 } else { plant.t() + options.dt };
            let mut writes = Vec::new();
            while cursor < log.len() && log[cursor].t <= next + 1e-9 {
                writes.push((log[cursor].id.clone(), log[cursor].value));
                cursor += 1;
            }
            plant.apply("PLC1", writes).unwrap();
            plant.tick().unwrap();
            first = false;
        }
        // The beam is what the controller read: the plant's output.
        let outputs = plant.outputs("PLC1").unwrap();
        assert!(outputs
            .iter()
            .any(|(id, v)| id.name == "beam" && *v == Value::Bool(true)));
        let tl = plant.finish();
        assert_eq!(lanes(&tl), lanes(&reference));
        assert_eq!(
            format!("{:?}", tl.objects),
            format!("{:?}", reference.objects)
        );
        // The external program's steps are the controller's own.
        assert!(tl.step_spans.is_empty());
    }

    #[test]
    fn a_monitoring_hosts_step_word_becomes_the_controllers_step_bands() {
        let mut scene = feed_cell();
        let mut node = scene.io_map().node("PLC1").unwrap().clone();
        node.monitor = true;
        scene.upsert_io_node(node).unwrap();
        let xc = exchange::derive(&scene, Some(&["feed"]), "PLC1").unwrap();
        let step = xc
            .inputs()
            .find(|p| p.id.name == "feed" && p.id.aspect == Some(iomap::Aspect::Step))
            .expect("the step word is an input of the monitoring host");
        assert_eq!(step.value, ValueKind::Int);
        assert_eq!(step.initial, Value::Int(-1));
        assert_eq!(step.source, IoSource::Monitor);
        let options = RolloutOptions::default();
        let mut plant = scene
            .open_plant(&["feed"], &options, None, &["PLC1"], false, false, &[])
            .unwrap();
        let word = |k: i64| vec![(step.id.clone(), Value::Int(k))];
        plant.apply("PLC1", word(0)).unwrap();
        for _ in 0..50 {
            plant.tick().unwrap();
        }
        plant.apply("PLC1", word(1)).unwrap();
        for _ in 0..30 {
            plant.tick().unwrap();
        }
        plant.apply("PLC1", word(-1)).unwrap();
        plant.tick().unwrap();
        assert!(plant.finished(), "a parked program is finished");
        let tl = plant.finish();
        let got: Vec<(String, f64, f64)> = tl
            .step_spans
            .iter()
            .map(|s| {
                (
                    s.name.clone(),
                    (s.start * 100.0).round() / 100.0,
                    (s.end * 100.0).round() / 100.0,
                )
            })
            .collect();
        assert_eq!(
            got,
            vec![
                ("feed".to_string(), 0.0, 0.5),
                ("stop".to_string(), 0.5, 0.8)
            ]
        );
        assert_eq!(tl.step_spans[1].sequence, "feed");
        assert_eq!(tl.step_spans[1].step, 1);
        // The word commands nothing: the belt never ran.
        assert!(lanes(&tl)
            .iter()
            .find(|(name, _)| name == "conv")
            .is_some_and(|(_, edges)| edges.iter().all(|(_, v)| !v)));
        // The log keeps the words like any input, so a replay reports them.
        assert_eq!(
            tl.step_spans.len(),
            2,
            "{:?}",
            plant_log_steps(&scene, &options)
        );
    }

    fn plant_log_steps(scene: &Scene, options: &RolloutOptions) -> Vec<String> {
        scene
            .open_plant(&["feed"], options, None, &["PLC1"], false, false, &[])
            .unwrap()
            .input_log()
            .iter()
            .map(|r| r.id.label())
            .collect()
    }

    #[test]
    fn a_clip_is_the_stretch_retimed_from_zero() {
        let scene = feed_cell();
        let options = RolloutOptions::default();
        let bake = scene.simulate_sequence("feed", &options).unwrap();
        let stop = bake.step_spans[0].end;
        let (from, to) = (stop - 0.4, stop + 0.05);
        let clip = bake.clip(from, to);
        assert!((clip.duration - (to - from)).abs() < 1e-9);
        // The lanes: the level at the cut is the initial, the edge inside
        // moves with the clock.
        let conv = clip.signals.iter().find(|l| l.name == "conv").unwrap();
        assert_eq!(conv.edges[0], (0.0, true));
        assert_eq!(conv.edges.len(), 2);
        assert!((conv.edges[1].0 - 0.4).abs() < 1e-9 && !conv.edges[1].1);
        // The bands: `feed` cut at the front, `stop` cut at the back.
        let spans: Vec<(String, f64, f64)> = clip
            .step_spans
            .iter()
            .map(|s| (s.name.clone(), s.start, s.end))
            .collect();
        assert_eq!(spans.len(), 2);
        assert_eq!(spans[0].0, "feed");
        assert_eq!(spans[0].1, 0.0);
        assert!((spans[0].2 - 0.4).abs() < 1e-9);
        assert!((spans[1].2 - clip.duration).abs() < 1e-9);
        // The box rides the belt in the clip exactly as it did in the bake
        // (conveyed, never carried: no link poses needed).
        for t in [0.0, 0.1, 0.39, 0.42, clip.duration] {
            let got = SequenceTimeline::object_pose(&clip.objects[0], &[], t).unwrap();
            let want = SequenceTimeline::object_pose(&bake.objects[0], &[], from + t).unwrap();
            assert!(
                (got.translation.vector - want.translation.vector).norm() < 1e-9,
                "t = {t}: {got:?} vs {want:?}"
            );
        }
        // The joints too, sampled at the cut and inside it.
        for t in [0.0, 0.2, clip.duration] {
            let got = clip.robots[0].trajectory.sample(t);
            let want = bake.robots[0].trajectory.sample(from + t);
            assert!(got.iter().zip(&want).all(|(a, b)| (a - b).abs() < 1e-9));
        }
        assert_eq!(clip.robots[0].trajectory.times[0], 0.0);
        assert!((clip.robots[0].trajectory.duration() - clip.duration).abs() < 1e-9);
    }

    #[test]
    fn an_external_robot_controller_needs_a_joint_stream() {
        let scene = feed_cell();
        let options = RolloutOptions::default();
        // The sample scene's robot is driven by nobody here, so the
        // refusals to check are a host this program set does not have and
        // no host at all.
        let err = scene
            .open_plant(&["feed"], &options, None, &["<cell>"], false, false, &[])
            .err()
            .expect("refused");
        assert!(err.to_string().contains("not a host"), "{err}");
        let err = scene
            .open_plant(&["feed"], &options, None, &[], false, false, &[])
            .err()
            .expect("refused");
        assert!(err.to_string().contains("external host"), "{err}");
    }

    #[test]
    fn a_force_pins_the_lane_and_an_injected_fault_waits_for_its_edge() {
        let scene = feed_cell();
        let options = RolloutOptions::default();
        let mut live = scene
            .open_plant(&["feed"], &options, None, &["PLC1"], true, false, &[])
            .unwrap();
        live.tick().unwrap(); // first scan: the belt starts
        live.force_lane("beam", true).unwrap();
        live.tick().unwrap();
        // The beam reads true at once; the program stops the belt.
        assert_eq!(live.view().signal("beam"), Some(true));
        assert_eq!(live.view().signal("conv"), Some(false));
        live.unforce_lane("beam").unwrap();
        live.tick().unwrap();
        assert_eq!(live.view().signal("beam"), Some(false)); // geometry again
        assert!(live.force_lane("nothing", true).is_err());

        let mut live = scene
            .open_plant(&["feed"], &options, None, &["PLC1"], true, false, &[])
            .unwrap();
        live.inject(
            Fault {
                target: "fed".into(),
                kind: crate::seq::FaultKind::StuckAt(false),
            },
            FaultAnchor::Rising("beam".into()),
        )
        .unwrap();
        while !live.finished() {
            live.tick().unwrap();
        }
        let tl = live.finish();
        // `fed` is set the scan the beam rises — and the fault pins it low
        // from that very scan, so it never rose.
        let fed = tl.signals.iter().find(|s| s.name == "fed").unwrap();
        assert_eq!(fed.edges, vec![(0.0, false)]);
    }

    /// A solenoid cylinder: `move_to` a stop is its coils, `device_done`
    /// its limit switches — two coils and two switches cross the exchange,
    /// no position word — and the loopback through them is the bake.
    fn cylinder_cell(single: bool) -> Scene {
        let mut scene = sample_scene();
        scene
            .add_obstacle(
                "rod",
                Geometry::Box {
                    size: Vector3::new(0.02, 0.02, 0.06),
                },
                iso(0.6, 0.0, 0.3),
            )
            .unwrap();
        let mut coils = vec![("extend".to_string(), "extended".to_string())];
        if !single {
            coils.push(("retract".to_string(), "retracted".to_string()));
        }
        scene.upsert_device(Device {
            name: "cyl".into(),
            kind: DeviceKind::LinearAxis {
                objects: vec!["rod".into()],
                axis: nalgebra::Unit::new_normalize(Vector3::z()),
                speed: 0.1,
                position: 0.0,
                range: (0.0, 0.05),
                stops: vec![("retracted".into(), 0.0), ("extended".into(), 0.05)],
                coils,
                encoder: true,
            },
        });
        let move_to = |stop: &str| Action::Device {
            device: "cyl".into(),
            command: DeviceCommand::MoveToStop(stop.into()),
        };
        let done = Condition::DeviceDone {
            device: "cyl".into(),
        };
        scene.upsert_sequence(Sequence {
            name: "press".into(),
            steps: vec![
                step("down", vec![move_to("extended")], done.clone()),
                step("dwell", vec![], Condition::Elapsed { seconds: 0.2 }),
                step("up", vec![move_to("retracted")], done),
            ],
        });
        scene
            .upsert_io_node(IoNode {
                name: "PLC1".into(),
                kind: IoNodeKind::Plc,
                programs: vec!["press".into()],
                uplink: None,
                channels: Vec::new(),
                place: None,
                model: None,
                monitor: false,
            })
            .unwrap();
        scene
    }

    #[test]
    fn a_cylinder_crosses_as_coils_and_switches_and_the_loopback_is_the_bake() {
        let scene = cylinder_cell(false);
        let xc = exchange::derive(&scene, Some(&["press"]), "PLC1").unwrap();
        let inputs: Vec<String> = xc.inputs().map(|p| p.label()).collect();
        let outputs: Vec<String> = xc.outputs().map(|p| p.label()).collect();
        assert_eq!(inputs, ["cyl/extend", "cyl/retract"]);
        assert_eq!(outputs, ["cyl.encoder", "cyl/extended", "cyl/retracted"]);
        let encoder = xc.by_label("cyl.encoder", PlantSide::Output).unwrap();
        assert_eq!(encoder.value, exchange::ValueKind::Real);
        assert_eq!(encoder.var, "cyl_encoder");
        assert!(xc.by_label("cyl.position", PlantSide::Input).is_none());

        let options = RolloutOptions::default();
        let bake = scene.simulate_sequence("press", &options).unwrap();
        let mut live = scene
            .open_plant(&["press"], &options, None, &["PLC1"], true, false, &[])
            .unwrap();
        while !live.finished() {
            live.tick().unwrap();
        }
        let tl = live.finish();
        assert_eq!(tl.duration, bake.duration);
        assert_eq!(lanes(&tl), lanes(&bake));
        assert_eq!(spans(&tl), spans(&bake));
        assert_eq!(tl.words, bake.words);
        let travel = bake.words.iter().find(|w| w.name == "cyl.encoder").unwrap();
        assert!((travel.value_at(0.5) - 0.05).abs() < 1e-9, "{travel:?}");
        assert!((travel.value_at(bake.duration) - 0.0).abs() < 1e-9);
    }

    #[test]
    fn a_coil_drives_the_cylinder_to_its_stop_and_a_dropped_single_coil_springs_back() {
        let options = RolloutOptions::default();
        for single in [false, true] {
            let scene = cylinder_cell(single);
            let mut plant = scene
                .open_plant(&["press"], &options, None, &["PLC1"], false, false, &[])
                .unwrap();
            let coil = IoPointId::parse("cyl/extend", IoDirection::Output);
            plant.tick().unwrap();
            plant
                .apply("PLC1", vec![(coil.clone(), Value::Bool(true))])
                .unwrap();
            for _ in 0..60 {
                plant.tick().unwrap();
            }
            let at = |plant: &LiveRollout, label: &str| {
                plant
                    .outputs("PLC1")
                    .unwrap()
                    .into_iter()
                    .find(|(id, _)| id.label() == label)
                    .map(|(_, v)| v)
                    .unwrap()
            };
            assert_eq!(at(&plant, "cyl/extended"), Value::Bool(true));
            assert_eq!(at(&plant, "cyl.encoder"), Value::Real(0.05));
            plant
                .apply("PLC1", vec![(coil, Value::Bool(false))])
                .unwrap();
            for _ in 0..60 {
                plant.tick().unwrap();
            }
            if single {
                // No coil named: the spring takes it home.
                assert_eq!(at(&plant, "cyl/retracted"), Value::Bool(true), "single");
            } else {
                // A double solenoid holds where the last coil sent it.
                assert_eq!(at(&plant, "cyl/extended"), Value::Bool(true), "double");
            }
        }
    }
}
