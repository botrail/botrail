//! The exchange table — the half of the I/O map a controller sees, and the
//! encoding that lets a program's actions cross it (design-virtual-
//! commissioning.md §4.1, VC1).
//!
//! Nothing here is authored. The I/O map ([`crate::iomap`]) already knows,
//! for every point, its name and aspect, which host owns it, which way it
//! faces from that host, its channel type, its binding (node, channel,
//! address, tag, polarity) and the rule that produced it; the PLCopen
//! export declares its variables from the same points. The exchange of a
//! host is that host's points turned round to the plant's side: what the
//! controller **writes** (its outputs — coils, device commands, robot
//! start and program words) are the plant's *inputs*, what it **reads**
//! (sensors, limit switches, in-position and done contacts, handshake
//! wires from other hosts) are the plant's *outputs*. A relay written and
//! read on the same host never crosses (it is the controller's own memory),
//! a magazine stays cosmetic, and a robot driven by its own controller is
//! not a wire.
//!
//! Two tables complete it: the **robot entries** — the numbered commands a
//! `start` pulse with a `program` word selects on a robot another host
//! drives (motions keep the PLCopen numbering, the other robot actions
//! follow in authoring order) — and the **parameters** a bus cannot carry
//! as a live value (an indexed conveyor's pitch). Both are derived from
//! the same programs, so the PLCopen file and the exchange agree by
//! construction.
//!
//! [`Exchange::encode`] turns one authored action into the writes a
//! controller would make; [`Exchange::decode`] turns a changed input back
//! into the actions the plant fires. The rollout's loopback mode runs the
//! two back to back and must reproduce the batch bake bit for bit — which
//! is what proves the encoding lossless and the derivation complete.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use crate::iomap::{self, Aspect, ChannelKind, IoDirection, IoPointId, IoSource, PointStatus};
use crate::seq::{Action, DeviceCommand, DeviceKind};
use crate::Scene;

/// Which way a point faces from the **plant's** side: `Input` — the
/// controller writes it and the plant reads (a coil, a command word);
/// `Output` — the plant writes it and the controller reads (a sensor, a
/// done contact).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlantSide {
    Input,
    Output,
}

impl PlantSide {
    pub fn as_str(self) -> &'static str {
        match self {
            PlantSide::Input => "input",
            PlantSide::Output => "output",
        }
    }
}

/// The value type a point carries — the IEC type of its PLCopen variable.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValueKind {
    Bool,
    Int,
    Real,
}

impl ValueKind {
    /// The IEC type element name (`BOOL`, `INT`, `REAL`).
    pub fn as_str(self) -> &'static str {
        match self {
            ValueKind::Bool => "BOOL",
            ValueKind::Int => "INT",
            ValueKind::Real => "REAL",
        }
    }
}

/// One exchanged value.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Value {
    Bool(bool),
    Int(i64),
    Real(f64),
}

impl Value {
    pub fn kind(&self) -> ValueKind {
        match self {
            Value::Bool(_) => ValueKind::Bool,
            Value::Int(_) => ValueKind::Int,
            Value::Real(_) => ValueKind::Real,
        }
    }

    /// The value read as a contact; a number is on when non-zero.
    pub fn as_bool(&self) -> bool {
        match self {
            Value::Bool(b) => *b,
            Value::Int(i) => *i != 0,
            Value::Real(x) => *x != 0.0,
        }
    }

    pub fn as_i64(&self) -> i64 {
        match self {
            Value::Bool(b) => *b as i64,
            Value::Int(i) => *i,
            Value::Real(x) => x.round() as i64,
        }
    }

    pub fn as_f64(&self) -> f64 {
        match self {
            Value::Bool(b) => *b as i64 as f64,
            Value::Int(i) => *i as f64,
            Value::Real(x) => *x,
        }
    }

    /// The value coerced to `kind` — what a controller's register image
    /// (a word for a bool, a float for a count) is read as.
    pub fn coerced(&self, kind: ValueKind) -> Value {
        match kind {
            ValueKind::Bool => Value::Bool(self.as_bool()),
            ValueKind::Int => Value::Int(self.as_i64()),
            ValueKind::Real => Value::Real(self.as_f64()),
        }
    }

    fn json(&self) -> String {
        match self {
            Value::Bool(b) => b.to_string(),
            Value::Int(i) => i.to_string(),
            Value::Real(x) if x.is_finite() => format!("{x}"),
            Value::Real(_) => "null".to_string(),
        }
    }
}

/// One point of the exchange: a derived I/O point of the host, with the
/// side it takes from the plant, its PLCopen variable, its value type, its
/// initial value and its binding.
#[derive(Debug, Clone, PartialEq)]
pub struct ExchangePoint {
    pub id: IoPointId,
    /// The variable name the PLCopen export declares for the point.
    pub var: String,
    pub kind: ChannelKind,
    pub side: PlantSide,
    pub value: ValueKind,
    pub source: IoSource,
    /// The value the point holds before anything is written: a signal's
    /// declared initial, a device's authored running state / speed /
    /// position, `0` / `false` otherwise.
    pub initial: Value,
    /// The binding, when the point is bound: node, channel id, the
    /// channel's address and vendor port, the PLC tag, the wire polarity.
    pub node: Option<String>,
    pub channel: Option<String>,
    pub address: Option<String>,
    pub port: Option<u32>,
    pub tag: Option<String>,
    pub invert: bool,
    /// What the plant does with the point beyond the obvious — a coil
    /// that a grasp follows ([`crate::grasp::GraspBinding::describe`]).
    pub note: Option<String>,
    /// Register counts per SI unit when the point is a measurement
    /// carried on a word channel with a unit: the controller sees
    /// `round(x · scale)` as an INT, the plant divides on the way back.
    /// `1.0` and `None` for an unscaled point.
    pub scale: f64,
    pub unit: Option<String>,
}

impl ExchangePoint {
    /// `name` or `name.aspect`.
    pub fn label(&self) -> String {
        self.id.label()
    }

    /// A plant-side value (SI) as the controller's register holds it.
    pub fn to_register(&self, value: Value) -> Value {
        if self.scale != 1.0 {
            Value::Int((value.as_f64() * self.scale).round() as i64)
        } else {
            value.coerced(self.value)
        }
    }

    /// A controller's register value as the plant uses it (SI).
    pub fn from_register(&self, value: &Value) -> Value {
        if self.scale != 1.0 {
            Value::Real(value.as_f64() / self.scale)
        } else {
            *value
        }
    }
}

/// One numbered robot command another host selects with the robot's
/// `program` word and fires with its `start` pulse.
#[derive(Debug, Clone)]
pub struct RobotEntry {
    pub robot: String,
    /// 1-based. Motions carry the PLCopen numbering (scene order per
    /// robot); every other robot action follows, in authoring order.
    pub number: u32,
    /// The action's identity — what two programs starting the same thing
    /// share a number by ([`action_key`]).
    pub key: String,
    /// What the table prints: the motion's name, or the key.
    pub label: String,
    pub action: Action,
}

/// A device parameter the bus cannot carry as a live value.
#[derive(Debug, Clone, PartialEq)]
pub struct ExchangeParam {
    pub device: String,
    /// `pitch_m` for an indexed conveyor's fixed advance.
    pub name: String,
    pub value: f64,
}

/// What kind of device a point commands — what a numeric input means.
#[derive(Debug, Clone, PartialEq)]
pub enum DeviceClass {
    Conveyor,
    /// A linear axis with its named stops `(name, position)` and, for a
    /// cylinder, its solenoid coils `(coil, stop)`.
    Axis {
        stops: Vec<(String, f64)>,
        coils: Vec<(String, String)>,
    },
    /// A vehicle with its stations in path order (`station` word = index + 1).
    Vehicle {
        stations: Vec<String>,
    },
    /// A lift with its named stops `(name, position)`.
    Lift {
        stops: Vec<(String, f64)>,
    },
    /// A source or sink: cosmetic, no I/O.
    Magazine,
}

/// The exchange of one host.
#[derive(Debug, Clone)]
pub struct Exchange {
    pub host: String,
    /// The program set the exchange was derived over.
    pub sequences: Vec<String>,
    /// The programs that run on this host.
    pub programs: Vec<String>,
    pub points: Vec<ExchangePoint>,
    /// Relays written and read on this host only — the controller's own
    /// memory, never exchanged (a loopback program reads and writes them
    /// directly).
    pub internal: Vec<String>,
    pub entries: Vec<RobotEntry>,
    pub parameters: Vec<ExchangeParam>,
    /// The devices this host commands or awaits.
    pub devices: BTreeMap<String, DeviceClass>,
    /// Things worth knowing that are not errors.
    pub notes: Vec<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ExchangeError {
    UnknownHost {
        host: String,
        known: Vec<String>,
    },
    Derivation(String),
    /// Something the exchange cannot carry as derived — what to author
    /// instead is in the message.
    Unsupported(String),
}

impl fmt::Display for ExchangeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            ExchangeError::UnknownHost { host, known } => write!(
                f,
                "`{host}` is not a host of this program set (hosts: {})",
                if known.is_empty() {
                    "none".to_string()
                } else {
                    known.join(", ")
                }
            ),
            ExchangeError::Derivation(m) => write!(f, "I/O derivation failed: {m}"),
            ExchangeError::Unsupported(m) => write!(f, "{m}"),
        }
    }
}

impl std::error::Error for ExchangeError {}

// ------------------------------------------------------------- numbering

/// Motion name → 1-based program number, per robot in scene order — the
/// PLCopen `robot_program := n` numbering, shared with the exchange.
pub fn motion_numbers(scene: &Scene) -> BTreeMap<String, u32> {
    let mut numbers = BTreeMap::new();
    let mut per_robot: BTreeMap<usize, u32> = BTreeMap::new();
    for m in scene.motions() {
        let n = per_robot.entry(m.robot).or_default();
        *n += 1;
        numbers.insert(m.name.clone(), *n);
    }
    numbers
}

/// The spring-return stop of a coil-driven axis: the one stop no coil
/// names (a single-solenoid cylinder goes there when its coil drops).
/// `None` when every stop has a coil (double solenoid) or several have
/// none.
pub fn rest_stop(coils: &[(String, String)], stops: &[(String, f64)]) -> Option<String> {
    let mut rest = stops
        .iter()
        .map(|(n, _)| n)
        .filter(|n| !coils.iter().any(|(_, s)| &s == n));
    let first = rest.next()?;
    rest.next().is_none().then(|| first.clone())
}

fn fmt_real(v: f64) -> String {
    let s = format!("{v:.6}");
    let s = s.trim_end_matches('0');
    let s = s.trim_end_matches('.');
    if s.is_empty() || s == "-" {
        "0".to_string()
    } else {
        s.to_string()
    }
}

/// The identity of a robot action — what a start / program handshake
/// selects. `None` for actions that address no robot (signals, devices).
pub fn action_key(action: &Action) -> Option<String> {
    Some(match action {
        Action::StartMotion { motion } => format!("motion:{motion}"),
        Action::StartRamp {
            targets,
            duration,
            check,
            ..
        } => {
            let joints: Vec<String> = targets
                .iter()
                .map(|(j, v)| format!("{j}={}", fmt_real(*v)))
                .collect();
            format!(
                "ramp:{};{}{}",
                joints.join(","),
                fmt_real(*duration),
                if *check { ";checked" } else { "" }
            )
        }
        Action::StartToolpath { toolpath, .. } => format!("toolpath:{toolpath}"),
        Action::Policy { policy, .. } => format!("policy:{policy}"),
        Action::Crouch {
            depth,
            lean,
            duration,
            ..
        } => format!(
            "crouch:{},{}{}",
            fmt_real(*depth),
            fmt_real(*lean),
            duration
                .map(|d| format!(",{}", fmt_real(d)))
                .unwrap_or_default()
        ),
        Action::Attach { object, .. } => format!("attach:{object}"),
        Action::Detach { object } => format!("detach:{object}"),
        Action::Track { object, .. } => format!("track:{object}"),
        Action::Untrack { group, .. } => match group {
            Some(g) => format!("untrack:{g}"),
            None => "untrack".to_string(),
        },
        Action::Set { .. } | Action::Device { .. } => return None,
    })
}

/// What the entry table prints for an action: the motion's name, or the
/// key.
pub fn action_label(action: &Action) -> String {
    match action {
        Action::StartMotion { motion } => motion.clone(),
        other => action_key(other).unwrap_or_default(),
    }
}

/// The robot (scene index) a robot action addresses. `held` is the
/// object → robot memory of the walk so far (an `Attach` remembers who
/// holds what, a `Detach` names no robot); the scene's attachments are the
/// fallback for a part held before the program starts. `Ok(None)` for
/// actions that address no robot.
pub fn action_robot(
    scene: &Scene,
    action: &Action,
    held: &mut BTreeMap<String, usize>,
) -> Result<Option<usize>, String> {
    Ok(Some(match action {
        Action::StartMotion { motion } => scene
            .motions()
            .iter()
            .find(|m| &m.name == motion)
            .map(|m| m.robot)
            .ok_or_else(|| format!("unknown motion `{motion}`"))?,
        Action::StartRamp { robot, .. }
        | Action::StartToolpath { robot, .. }
        | Action::Policy { robot, .. }
        | Action::Crouch { robot, .. }
        | Action::Track { robot, .. }
        | Action::Untrack { robot, .. } => scene.resolve_seq_robot(robot)?,
        Action::Attach { robot, object, .. } => {
            let r = scene.resolve_seq_robot(robot)?;
            held.insert(object.clone(), r);
            r
        }
        Action::Detach { object } => match held.get(object).copied().or_else(|| {
            scene
                .attachments()
                .iter()
                .find(|a| &a.object == object)
                .map(|a| a.robot)
        }) {
            Some(r) => r,
            None => {
                return Err(format!(
                    "detach of `{object}`: no robot holds it (no attach before it, nothing \
                     attached in the scene) — the command cannot be attributed to a controller"
                ))
            }
        },
        Action::Set { .. } | Action::Device { .. } => return Ok(None),
    }))
}

/// The numbered robot commands host `host` issues across `programs` (names,
/// in scan order) to robots whose controller is another host: motions by
/// the PLCopen numbering ([`motion_numbers`]), every other robot action
/// after them in first-appearance order, one number per distinct command.
/// Shared by the PLCopen export and the exchange, so the two agree.
pub fn robot_entries(
    scene: &Scene,
    programs: &[&str],
    host: &str,
) -> Result<Vec<RobotEntry>, String> {
    let io = scene.io_map();
    let numbers = motion_numbers(scene);
    let mut next: BTreeMap<usize, u32> = BTreeMap::new();
    for m in scene.motions() {
        *next.entry(m.robot).or_default() += 1;
    }
    let mut entries: Vec<RobotEntry> = Vec::new();
    for name in programs {
        let Some(sequence) = scene.sequence(name) else {
            continue;
        };
        let mut held: BTreeMap<String, usize> = BTreeMap::new();
        crate::seq::walk_actions(&sequence.steps, &mut |action: &Action| {
            let Some(r) = action_robot(scene, action, &mut held)? else {
                return Ok(());
            };
            let rname = scene.robots()[r].name.clone();
            if io.robot_controller_host(&rname) == host {
                return Ok(()); // the host is the robot's own controller: no wire
            }
            let key = action_key(action).expect("robot action has a key");
            if entries.iter().any(|e| e.robot == rname && e.key == key) {
                return Ok(());
            }
            let number = match action {
                Action::StartMotion { motion } => numbers.get(motion).copied().unwrap_or(0),
                _ => {
                    let n = next.entry(r).or_default();
                    *n += 1;
                    *n
                }
            };
            entries.push(RobotEntry {
                robot: rname,
                number,
                key,
                label: action_label(action),
                action: action.clone(),
            });
            Ok::<(), String>(())
        })
        .map_err(|m| format!("program `{name}`: {m}"))?;
    }
    Ok(entries)
}

// --------------------------------------------------------------- derive

fn value_kind(p: &iomap::IoPoint) -> ValueKind {
    match iomap::natural_type(p) {
        "REAL" => ValueKind::Real,
        "INT" => ValueKind::Int,
        _ => ValueKind::Bool,
    }
}

fn device_class(kind: &DeviceKind) -> DeviceClass {
    match kind {
        DeviceKind::Conveyor { .. } => DeviceClass::Conveyor,
        DeviceKind::LinearAxis { stops, coils, .. } => DeviceClass::Axis {
            stops: stops.clone(),
            coils: coils.clone(),
        },
        DeviceKind::Vehicle { path, .. } => DeviceClass::Vehicle {
            stations: path.stations.iter().map(|(s, _)| s.clone()).collect(),
        },
        DeviceKind::Lift { stops, .. } => DeviceClass::Lift {
            stops: stops.clone(),
        },
        DeviceKind::Source { .. } | DeviceKind::Sink { .. } => DeviceClass::Magazine,
    }
}

fn initial_value(scene: &Scene, p: &iomap::IoPoint, kind: ValueKind) -> Value {
    let device = || scene.devices().iter().find(|d| d.name == p.id.name);
    match (p.source, p.id.aspect) {
        (IoSource::DeviceRun, _) => Value::Bool(match device().map(|d| &d.kind) {
            Some(DeviceKind::Conveyor { running, .. }) => *running,
            Some(DeviceKind::Source { running, .. }) => *running,
            _ => false,
        }),
        (IoSource::DeviceCommand, Some(Aspect::Speed)) => {
            Value::Real(match device().map(|d| &d.kind) {
                Some(DeviceKind::Conveyor { velocity, .. }) => velocity.norm(),
                Some(DeviceKind::LinearAxis { speed, .. }) => *speed,
                Some(DeviceKind::Source { interval, .. }) => *interval,
                Some(DeviceKind::Vehicle { speed, .. }) => *speed,
                Some(DeviceKind::Lift { speed, .. }) => *speed,
                _ => 0.0,
            })
        }
        (IoSource::DeviceCommand, Some(Aspect::Position)) => {
            Value::Real(match device().map(|d| &d.kind) {
                Some(DeviceKind::LinearAxis { position, .. }) => *position,
                Some(DeviceKind::Lift { stops, start, .. }) => stops
                    .iter()
                    .find(|(n, _)| n == start)
                    .map(|(_, at)| *at)
                    .unwrap_or(0.0),
                _ => 0.0,
            })
        }
        (IoSource::RobotDone, _) => Value::Bool(true),
        // A cylinder's coil starts energised when the axis stands at the
        // coil's stop: the first command the other way is then an edge.
        (IoSource::DeviceCoil, _) => {
            let Some((device, coil)) = p.id.name.rsplit_once('/') else {
                return Value::Bool(false);
            };
            let at_stop = scene
                .devices()
                .iter()
                .find(|d| d.name == device)
                .and_then(|d| {
                    let DeviceKind::LinearAxis {
                        position,
                        stops,
                        coils,
                        ..
                    } = &d.kind
                    else {
                        return None;
                    };
                    let stop = coils.iter().find(|(c, _)| c == coil).map(|(_, s)| s)?;
                    let at = stops.iter().find(|(n, _)| n == stop).map(|(_, at)| *at)?;
                    Some((position - at).abs() <= crate::rollout::STOP_TOL)
                });
            Value::Bool(at_stop.unwrap_or(false))
        }
        (IoSource::DeviceFeedback, _) | (_, Some(Aspect::Encoder)) => {
            Value::Real(match device().map(|d| &d.kind) {
                Some(DeviceKind::LinearAxis { position, .. }) => *position,
                _ => 0.0,
            })
        }
        // No step yet: the program has not entered its first one.
        (IoSource::Monitor, _) => Value::Int(-1),
        _ if p.id.aspect.is_none() => match kind {
            ValueKind::Bool => Value::Bool(
                scene
                    .signals()
                    .iter()
                    .find(|s| s.name == p.id.name)
                    .map(|s| s.initial)
                    .unwrap_or(false),
            ),
            ValueKind::Int => Value::Int(0),
            ValueKind::Real => Value::Real(0.0),
        },
        _ => match kind {
            ValueKind::Bool => Value::Bool(false),
            ValueKind::Int => Value::Int(0),
            ValueKind::Real => Value::Real(0.0),
        },
    }
}

/// The exchange of `host` over the program set `sequences` (`None` = every
/// sequence, in scene order). Nothing is authored: the points are the I/O
/// map's, the variables the PLCopen export's, the robot entries and the
/// parameters are walked from the programs that run on the host.
pub fn derive(
    scene: &Scene,
    sequences: Option<&[&str]>,
    host: &str,
) -> Result<Exchange, ExchangeError> {
    let d =
        iomap::derive(scene, sequences).map_err(|e| ExchangeError::Derivation(e.to_string()))?;
    let mut known: BTreeSet<String> = d.program_hosts.iter().map(|(_, h)| h.clone()).collect();
    known.extend(d.points.iter().filter_map(|p| p.host.clone()));
    if !known.contains(host) {
        return Err(ExchangeError::UnknownHost {
            host: host.to_string(),
            known: known.into_iter().collect(),
        });
    }
    let programs: Vec<String> = d
        .program_hosts
        .iter()
        .filter(|(_, h)| h == host)
        .map(|(p, _)| p.clone())
        .collect();

    // ---- points ------------------------------------------------------------
    let mut points = Vec::new();
    let mut internal = Vec::new();
    let mut devices: BTreeMap<String, DeviceClass> = BTreeMap::new();
    for p in d.points.iter().filter(|p| p.host.as_deref() == Some(host)) {
        match p.status {
            PointStatus::Internal => {
                internal.push(p.id.name.clone());
                continue;
            }
            PointStatus::Cosmetic | PointStatus::Constant => continue,
            _ => {}
        }
        let mut kind = value_kind(p);
        let (node, channel, address, port, tag, invert, unit) = match d.binding_of(p) {
            Some((b, n, c)) => (
                Some(n.name.clone()),
                Some(c.id.clone()),
                c.address.clone(),
                c.port,
                b.tag.clone(),
                b.invert,
                c.unit.clone().filter(|_| c.kind == ChannelKind::Word),
            ),
            None => (None, None, None, None, None, false, None),
        };
        // A measurement on a word register with a unit crosses as a
        // scaled INT (the lint has already named a unit it cannot scale).
        let (scale, unit) = match (&unit, kind) {
            (Some(u), ValueKind::Real) => match iomap::unit_scale(u) {
                Some(scale) => {
                    kind = ValueKind::Int;
                    (scale, Some(u.clone()))
                }
                None => (1.0, None),
            },
            _ => (1.0, None),
        };
        // A coil `<axis>/<coil>` belongs to the axis before the slash.
        let device_name = match p.source {
            IoSource::DeviceCoil => {
                p.id.name
                    .rsplit_once('/')
                    .map(|(d, _)| d.to_string())
                    .unwrap_or_else(|| p.id.name.clone())
            }
            _ => p.id.name.clone(),
        };
        if matches!(
            p.source,
            IoSource::DeviceRun
                | IoSource::DeviceDone
                | IoSource::DeviceCommand
                | IoSource::DeviceCoil
                | IoSource::DeviceFeedback
        ) {
            if let Some(dev) = scene.devices().iter().find(|dv| dv.name == device_name) {
                devices
                    .entry(dev.name.clone())
                    .or_insert_with(|| device_class(&dev.kind));
            }
        }
        let initial = initial_value(scene, p, kind);
        let mut point = ExchangePoint {
            id: p.id.clone(),
            var: crate::plcopen::point_var(p),
            kind: p.kind,
            side: match p.id.direction {
                IoDirection::Output => PlantSide::Input,
                IoDirection::Input => PlantSide::Output,
            },
            value: kind,
            source: p.source,
            initial,
            node,
            channel,
            address,
            port,
            tag,
            invert,
            note: None,
            scale,
            unit,
        };
        if scale != 1.0 {
            point.initial = point.to_register(initial);
            point.note = Some(format!(
                "INT in {}: {} counts per metre — the controller's targets round to the unit",
                point.unit.as_deref().unwrap_or(""),
                fmt_real(scale)
            ));
        }
        points.push(point);
    }
    for p in &mut points {
        if p.side == PlantSide::Input && p.id.aspect.is_none() {
            if let Some(b) = scene.grasp_binding(&p.id.name) {
                p.note = Some(b.describe(&scene.robots()[b.robot].name));
            }
        }
    }

    // ---- robot entries and parameters, walked from the host's programs ----
    let refs: Vec<&str> = programs.iter().map(String::as_str).collect();
    let entries = robot_entries(scene, &refs, host).map_err(ExchangeError::Unsupported)?;
    let mut pitches: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    let mut notes = Vec::new();
    for name in &programs {
        let Some(sequence) = scene.sequence(name) else {
            continue;
        };
        crate::seq::walk_actions(&sequence.steps, &mut |action: &Action| {
            if let Action::Device {
                device,
                command: DeviceCommand::Advance(d),
            } = action
            {
                pitches
                    .entry(device.clone())
                    .or_default()
                    .insert(fmt_real(*d));
            }
            Ok::<(), String>(())
        })
        .map_err(|m| ExchangeError::Unsupported(format!("program `{name}`: {m}")))?;
    }
    let mut parameters = Vec::new();
    for (device, values) in pitches {
        if values.len() > 1 {
            return Err(ExchangeError::Unsupported(format!(
                "conveyor `{device}` is advanced by different distances ({}) from host `{host}` — the \
                 index pulse carries one pitch; author one distance per conveyor",
                values.into_iter().collect::<Vec<_>>().join(", ")
            )));
        }
        let value: f64 = values
            .iter()
            .next()
            .and_then(|v| v.parse().ok())
            .unwrap_or(0.0);
        parameters.push(ExchangeParam {
            device,
            name: "pitch_m".to_string(),
            value,
        });
    }
    let robots: BTreeSet<&str> = entries.iter().map(|e| e.robot.as_str()).collect();
    for name in robots {
        let listed = entries.iter().filter(|e| e.robot == name).count();
        let has_word = points
            .iter()
            .any(|p| p.id.name == name && p.id.aspect == Some(Aspect::Program));
        if listed >= 2 && !has_word {
            notes.push(format!(
                "robot `{name}`: {listed} commands but no `{name}.program` word on `{host}` — the \
                 I/O map numbers programs only when a host commands two or more"
            ));
        }
    }

    Ok(Exchange {
        host: host.to_string(),
        sequences: d.sequences.clone(),
        programs,
        points,
        internal,
        entries,
        parameters,
        devices,
        notes,
    })
}

// -------------------------------------------------------- encode / decode

/// The writes one authored action makes on the exchange.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Encoded {
    /// Levels and words written, in order.
    pub writes: Vec<(IoPointId, Value)>,
    /// Pulses: raised with the writes, dropped again before the next scan.
    pub pulses: Vec<IoPointId>,
}

/// How an action crosses the exchange.
#[derive(Debug, Clone, PartialEq)]
pub enum Encoding {
    /// It does not: the action is the controller's own business (its
    /// internal relay, its own robot, a cosmetic magazine) and fires on
    /// the plant directly.
    Direct,
    Exchange(Encoded),
}

impl Exchange {
    pub fn point(
        &self,
        name: &str,
        aspect: Option<Aspect>,
        side: PlantSide,
    ) -> Option<&ExchangePoint> {
        self.points
            .iter()
            .find(|p| p.id.name == name && p.id.aspect == aspect && p.side == side)
    }

    pub fn by_id(&self, id: &IoPointId) -> Option<&ExchangePoint> {
        self.points.iter().find(|p| &p.id == id)
    }

    /// The point a label (`name` or `name.aspect`) names on `side`.
    pub fn by_label(&self, label: &str, side: PlantSide) -> Option<&ExchangePoint> {
        let direction = match side {
            PlantSide::Input => IoDirection::Output,
            PlantSide::Output => IoDirection::Input,
        };
        let id = IoPointId::parse(label, direction);
        self.by_id(&id)
    }

    pub fn inputs(&self) -> impl Iterator<Item = &ExchangePoint> {
        self.points.iter().filter(|p| p.side == PlantSide::Input)
    }

    pub fn outputs(&self) -> impl Iterator<Item = &ExchangePoint> {
        self.points.iter().filter(|p| p.side == PlantSide::Output)
    }

    pub fn entry(&self, robot: &str, number: u32) -> Option<&RobotEntry> {
        self.entries
            .iter()
            .find(|e| e.robot == robot && e.number == number)
    }

    pub fn parameter(&self, device: &str, name: &str) -> Option<f64> {
        self.parameters
            .iter()
            .find(|p| p.device == device && p.name == name)
            .map(|p| p.value)
    }

    fn input_id(&self, name: &str, aspect: Option<Aspect>) -> Result<IoPointId, String> {
        self.point(name, aspect, PlantSide::Input)
            .map(|p| p.id.clone())
            .ok_or_else(|| {
                let label = IoPointId {
                    name: name.to_string(),
                    aspect,
                    direction: IoDirection::Output,
                }
                .label();
                format!(
                    "`{label}` is not an input of the plant on host `{}` — the I/O map derives no \
                     such point for this host",
                    self.host
                )
            })
    }

    /// The writes a controller on this host makes to fire `action`, or
    /// `Direct` when the action never crosses (see [`Encoding`]). `scene`
    /// is the world as it stands (a detach is attributed to the robot
    /// holding the part).
    pub fn encode(&self, action: &Action, scene: &Scene) -> Result<Encoding, String> {
        let mut out = Encoded::default();
        match action {
            Action::Set { signal, value } => {
                if self.point(signal, None, PlantSide::Input).is_none()
                    && self.internal.iter().any(|n| n == signal)
                {
                    return Ok(Encoding::Direct);
                }
                out.writes
                    .push((self.input_id(signal, None)?, Value::Bool(*value)));
            }
            Action::Device { device, command } => {
                let class = self.devices.get(device);
                if matches!(class, Some(DeviceClass::Magazine))
                    || (class.is_none()
                        && scene.devices().iter().any(|d| {
                            &d.name == device
                                && matches!(
                                    d.kind,
                                    DeviceKind::Source { .. } | DeviceKind::Sink { .. }
                                )
                        }))
                {
                    return Ok(Encoding::Direct);
                }
                match command {
                    DeviceCommand::Start => out
                        .writes
                        .push((self.input_id(device, None)?, Value::Bool(true))),
                    DeviceCommand::Stop => out
                        .writes
                        .push((self.input_id(device, None)?, Value::Bool(false))),
                    DeviceCommand::SetSpeed(v) => {
                        let id = self.input_id(device, Some(Aspect::Speed))?;
                        let value = self.by_id(&id).expect("input").to_register(Value::Real(*v));
                        out.writes.push((id, value));
                    }
                    DeviceCommand::MoveTo(x) => match class {
                        Some(DeviceClass::Axis { stops, coils }) if !coils.is_empty() => {
                            let stop = stops
                                .iter()
                                .find(|(_, at)| (at - x).abs() < 1e-6)
                                .map(|(n, _)| n.clone())
                                .ok_or_else(|| {
                                    format!(
                                        "axis `{device}` is driven by coils: move_to a stop ({}), not a position",
                                        stops.iter().map(|(n, _)| format!("`{n}`")).collect::<Vec<_>>().join(", ")
                                    )
                                })?;
                            self.encode_coils(device, coils, stops, &stop, &mut out)?;
                        }
                        _ => {
                            let id = self.input_id(device, Some(Aspect::Position))?;
                            let value =
                                self.by_id(&id).expect("input").to_register(Value::Real(*x));
                            out.writes.push((id, value));
                        }
                    },
                    DeviceCommand::MoveToStop(stop) => match class {
                        Some(DeviceClass::Axis { stops, coils }) if !coils.is_empty() => {
                            self.encode_coils(device, coils, stops, stop, &mut out)?;
                        }
                        _ => {
                            let at = match class {
                                Some(DeviceClass::Axis { stops, .. })
                                | Some(DeviceClass::Lift { stops }) => {
                                    stops.iter().find(|(n, _)| n == stop).map(|(_, at)| *at)
                                }
                                _ => None,
                            }
                            .ok_or_else(|| format!("device `{device}` has no stop `{stop}`"))?;
                            let id = self.input_id(device, Some(Aspect::Position))?;
                            let value =
                                self.by_id(&id).expect("input").to_register(Value::Real(at));
                            out.writes.push((id, value));
                        }
                    },
                    DeviceCommand::Goto { station } => {
                        let index = match class {
                            Some(DeviceClass::Vehicle { stations }) => {
                                stations.iter().position(|s| s == station)
                            }
                            _ => None,
                        }
                        .ok_or_else(|| format!("vehicle `{device}` has no station `{station}`"))?;
                        out.writes.push((
                            self.input_id(device, Some(Aspect::Station))?,
                            Value::Int(index as i64 + 1),
                        ));
                        out.pulses
                            .push(self.input_id(device, Some(Aspect::Dispatch))?);
                    }
                    DeviceCommand::Advance(d) => {
                        let pitch = self.parameter(device, "pitch_m").ok_or_else(|| {
                            format!(
                                "conveyor `{device}` has no `pitch_m` parameter on this exchange"
                            )
                        })?;
                        if (pitch - d).abs() > 1e-9 {
                            return Err(format!(
                                "conveyor `{device}`: advance of {} m, but the exchange pitch is {} m",
                                fmt_real(*d),
                                fmt_real(pitch)
                            ));
                        }
                        out.pulses.push(self.input_id(device, Some(Aspect::Index))?);
                    }
                }
            }
            robot_action => {
                let mut held = BTreeMap::new();
                let Some(r) = action_robot(scene, robot_action, &mut held)? else {
                    return Ok(Encoding::Direct);
                };
                let rname = &scene.robots()[r].name;
                let key = action_key(robot_action).expect("robot action has a key");
                let Some(entry) = self
                    .entries
                    .iter()
                    .find(|e| &e.robot == rname && e.key == key)
                else {
                    if scene.io_map().robot_controller_host(rname) == self.host {
                        return Ok(Encoding::Direct);
                    }
                    return Err(format!(
                        "robot `{rname}`: command `{key}` is not an entry of host `{}`",
                        self.host
                    ));
                };
                if let Some(word) = self.point(rname, Some(Aspect::Program), PlantSide::Input) {
                    out.writes
                        .push((word.id.clone(), Value::Int(entry.number as i64)));
                }
                out.pulses.push(self.input_id(rname, Some(Aspect::Start))?);
            }
        }
        Ok(Encoding::Exchange(out))
    }

    /// The coil levels that send a coil-driven axis to `stop`: its coil
    /// high and every other coil low; the spring-return stop (no coil of
    /// its own) is every coil low.
    fn encode_coils(
        &self,
        device: &str,
        coils: &[(String, String)],
        stops: &[(String, f64)],
        stop: &str,
        out: &mut Encoded,
    ) -> Result<(), String> {
        if !stops.iter().any(|(n, _)| n == stop) {
            return Err(format!("device `{device}` has no stop `{stop}`"));
        }
        if !coils.iter().any(|(_, s)| s == stop) && rest_stop(coils, stops).as_deref() != Some(stop)
        {
            return Err(format!(
                "axis `{device}`: no coil drives to `{stop}` and it is not the spring-return stop"
            ));
        }
        for (coil, target) in coils {
            out.writes.push((
                self.input_id(&format!("{device}/{coil}"), None)?,
                Value::Bool(target == stop),
            ));
        }
        Ok(())
    }

    /// The actions the plant fires when input `id` changes from `prev` to
    /// `new`, given every input's held value (`held`, the companion words
    /// a pulse selects by). Levels fire on their edge, words on their
    /// change, pulses on their rising edge.
    pub fn decode(
        &self,
        id: &IoPointId,
        prev: &Value,
        new: &Value,
        held: &BTreeMap<IoPointId, Value>,
    ) -> Result<Vec<Action>, String> {
        let point = self
            .by_id(id)
            .filter(|p| p.side == PlantSide::Input)
            .ok_or_else(|| format!("`{}` is not an input of host `{}`", id.label(), self.host))?;
        let rose = !prev.as_bool() && new.as_bool();
        let fell = prev.as_bool() && !new.as_bool();
        let name = &id.name;
        let word = |aspect: Aspect| -> Value {
            held.get(&IoPointId {
                name: name.clone(),
                aspect: Some(aspect),
                direction: IoDirection::Output,
            })
            .copied()
            .unwrap_or(Value::Int(0))
        };
        Ok(match (point.source, id.aspect) {
            (IoSource::DeviceRun, _) => {
                if rose {
                    vec![Action::Device {
                        device: name.clone(),
                        command: DeviceCommand::Start,
                    }]
                } else if fell {
                    vec![Action::Device {
                        device: name.clone(),
                        command: DeviceCommand::Stop,
                    }]
                } else {
                    Vec::new()
                }
            }
            // A solenoid coil: energised, the axis goes to the coil's
            // stop; dropped, a single-solenoid axis springs back to the
            // stop no coil names, a double-solenoid one holds.
            (IoSource::DeviceCoil, _) => {
                let (device, coil) = name
                    .rsplit_once('/')
                    .ok_or_else(|| format!("coil `{name}` is not `<axis>/<coil>`"))?;
                let Some(DeviceClass::Axis { stops, coils }) = self.devices.get(device) else {
                    return Err(format!(
                        "coil `{name}`: `{device}` is not a coil-driven axis"
                    ));
                };
                let target = coils
                    .iter()
                    .find(|(c, _)| c == coil)
                    .map(|(_, s)| s.clone())
                    .ok_or_else(|| format!("axis `{device}` has no coil `{coil}`"))?;
                if rose {
                    vec![Action::Device {
                        device: device.to_string(),
                        command: DeviceCommand::MoveToStop(target),
                    }]
                } else if fell {
                    match rest_stop(coils, stops) {
                        Some(rest) => vec![Action::Device {
                            device: device.to_string(),
                            command: DeviceCommand::MoveToStop(rest),
                        }],
                        None => Vec::new(),
                    }
                } else {
                    Vec::new()
                }
            }
            (IoSource::DeviceCommand, Some(Aspect::Speed)) => {
                if prev.as_f64() != new.as_f64() {
                    vec![Action::Device {
                        device: name.clone(),
                        command: DeviceCommand::SetSpeed(point.from_register(new).as_f64()),
                    }]
                } else {
                    Vec::new()
                }
            }
            (IoSource::DeviceCommand, Some(Aspect::Position)) => {
                if prev.as_f64() == new.as_f64() {
                    return Ok(Vec::new());
                }
                let new = &point.from_register(new);
                match self.devices.get(name) {
                    Some(DeviceClass::Lift { stops }) => {
                        let stop = stops
                            .iter()
                            .find(|(_, at)| (at - new.as_f64()).abs() < 1e-6)
                            .map(|(n, _)| n.clone())
                            .ok_or_else(|| {
                                format!(
                                    "lift `{name}`: position {} is not a stop ({})",
                                    fmt_real(new.as_f64()),
                                    stops
                                        .iter()
                                        .map(|(n, at)| format!("{n}={}", fmt_real(*at)))
                                        .collect::<Vec<_>>()
                                        .join(", ")
                                )
                            })?;
                        vec![Action::Device {
                            device: name.clone(),
                            command: DeviceCommand::MoveToStop(stop),
                        }]
                    }
                    _ => vec![Action::Device {
                        device: name.clone(),
                        command: DeviceCommand::MoveTo(new.as_f64()),
                    }],
                }
            }
            // A new station while the dispatch level is still held: the
            // next order of a controller that never dropped the bit.
            (IoSource::DeviceCommand, Some(Aspect::Station)) => {
                let dispatch = held
                    .get(&IoPointId {
                        name: name.clone(),
                        aspect: Some(Aspect::Dispatch),
                        direction: IoDirection::Output,
                    })
                    .is_some_and(Value::as_bool);
                if !dispatch || prev.as_i64() == new.as_i64() {
                    return Ok(Vec::new());
                }
                let n = new.as_i64();
                let station = match self.devices.get(name) {
                    Some(DeviceClass::Vehicle { stations }) if n >= 1 => {
                        stations.get(n as usize - 1).cloned()
                    }
                    _ => None,
                }
                .ok_or_else(|| format!("vehicle `{name}`: station word {n} selects no station"))?;
                vec![Action::Device {
                    device: name.clone(),
                    command: DeviceCommand::Goto { station },
                }]
            }
            (IoSource::DeviceCommand, Some(Aspect::Dispatch)) => {
                if !rose {
                    return Ok(Vec::new());
                }
                let n = word(Aspect::Station).as_i64();
                let station = match self.devices.get(name) {
                    Some(DeviceClass::Vehicle { stations }) if n >= 1 => {
                        stations.get(n as usize - 1).cloned()
                    }
                    _ => None,
                }
                .ok_or_else(|| {
                    format!("vehicle `{name}`: dispatch with station word {n}, which selects no station")
                })?;
                vec![Action::Device {
                    device: name.clone(),
                    command: DeviceCommand::Goto { station },
                }]
            }
            (IoSource::DeviceCommand, Some(Aspect::Index)) => {
                if !rose {
                    return Ok(Vec::new());
                }
                let pitch = self.parameter(name, "pitch_m").ok_or_else(|| {
                    format!("conveyor `{name}`: index pulse but no `pitch_m` parameter")
                })?;
                vec![Action::Device {
                    device: name.clone(),
                    command: DeviceCommand::Advance(pitch),
                }]
            }
            (IoSource::RobotStart, _) => {
                if !rose {
                    return Ok(Vec::new());
                }
                let entries: Vec<&RobotEntry> =
                    self.entries.iter().filter(|e| &e.robot == name).collect();
                let has_word = self
                    .point(name, Some(Aspect::Program), PlantSide::Input)
                    .is_some();
                let entry = if has_word {
                    let n = word(Aspect::Program).as_i64();
                    entries.iter().find(|e| e.number as i64 == n).copied().ok_or_else(|| {
                        format!(
                            "robot `{name}`: start with program word {n}, which selects no command ({})",
                            entries
                                .iter()
                                .map(|e| format!("{}={}", e.number, e.label))
                                .collect::<Vec<_>>()
                                .join(", ")
                        )
                    })?
                } else {
                    match entries.as_slice() {
                        [one] => *one,
                        [] => {
                            return Err(format!(
                                "robot `{name}`: start, but the host has no command for it"
                            ))
                        }
                        _ => {
                            return Err(format!(
                                "robot `{name}`: start without a program word, but {} commands are \
                                 numbered",
                                entries.len()
                            ))
                        }
                    }
                };
                vec![entry.action.clone()]
            }
            // A new program word while `start` is still held: the next
            // command of a controller that never dropped the level.
            (IoSource::RobotProgram, _) => {
                let start = held
                    .get(&IoPointId {
                        name: name.clone(),
                        aspect: Some(Aspect::Start),
                        direction: IoDirection::Output,
                    })
                    .is_some_and(Value::as_bool);
                if !start || prev.as_i64() == new.as_i64() {
                    return Ok(Vec::new());
                }
                let n = new.as_i64();
                let entry = self
                    .entries
                    .iter()
                    .find(|e| &e.robot == name && e.number as i64 == n)
                    .ok_or_else(|| {
                        format!("robot `{name}`: program word {n} selects no command")
                    })?;
                vec![entry.action.clone()]
            }
            // The controller's step report: nothing to fire — the plant
            // records it as the program's step bands.
            (IoSource::Monitor, _) => Vec::new(),
            (IoSource::WriteOnly, None)
            | (IoSource::Handshake, None)
            | (IoSource::Declared, None)
            | (IoSource::ReadOnly, None)
            | (IoSource::Internal, None) => {
                if prev.as_bool() != new.as_bool() {
                    vec![Action::Set {
                        signal: name.clone(),
                        value: new.as_bool(),
                    }]
                } else {
                    Vec::new()
                }
            }
            (source, aspect) => {
                return Err(format!(
                    "`{}` ({}{}) cannot be written by a controller",
                    id.label(),
                    source.as_str(),
                    aspect
                        .map(|a| format!(", {}", a.as_str()))
                        .unwrap_or_default()
                ))
            }
        })
    }

    /// The exchange as JSON (points, entries, parameters, notes).
    pub fn to_json(&self) -> String {
        let s = |t: &str| json_str(t);
        let opt = |t: &Option<String>| match t {
            Some(t) => json_str(t),
            None => "null".to_string(),
        };
        let mut out = String::new();
        out.push_str(&format!(
            "{{\"host\":{},\"sequences\":[{}],\"programs\":[{}],\"internal\":[{}],\"points\":[",
            s(&self.host),
            self.sequences
                .iter()
                .map(|x| s(x))
                .collect::<Vec<_>>()
                .join(","),
            self.programs
                .iter()
                .map(|x| s(x))
                .collect::<Vec<_>>()
                .join(","),
            self.internal
                .iter()
                .map(|x| s(x))
                .collect::<Vec<_>>()
                .join(","),
        ));
        for (i, p) in self.points.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&format!(
                "{{\"label\":{},\"name\":{},\"aspect\":{},\"direction\":{},\"side\":{},\"var\":{},\"type\":{},\"kind\":{},\"source\":{},\"initial\":{},\"node\":{},\"channel\":{},\"address\":{},\"port\":{},\"tag\":{},\"invert\":{},\"note\":{}}}",
                s(&p.label()),
                s(&p.id.name),
                opt(&p.id.aspect.map(|a| a.as_str().to_string())),
                s(p.id.direction.as_str()),
                s(p.side.as_str()),
                s(&p.var),
                s(p.value.as_str()),
                s(p.kind.as_str()),
                s(p.source.as_str()),
                p.initial.json(),
                opt(&p.node),
                opt(&p.channel),
                opt(&p.address),
                p.port.map(|x| x.to_string()).unwrap_or_else(|| "null".to_string()),
                opt(&p.tag),
                p.invert,
                opt(&p.note),
            ));
        }
        out.push_str("],\"entries\":[");
        for (i, e) in self.entries.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&format!(
                "{{\"robot\":{},\"number\":{},\"key\":{},\"label\":{}}}",
                s(&e.robot),
                e.number,
                s(&e.key),
                s(&e.label)
            ));
        }
        out.push_str("],\"parameters\":[");
        for (i, p) in self.parameters.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&format!(
                "{{\"device\":{},\"name\":{},\"value\":{}}}",
                s(&p.device),
                s(&p.name),
                p.value
            ));
        }
        out.push_str("],\"notes\":[");
        out.push_str(
            &self
                .notes
                .iter()
                .map(|n| s(n))
                .collect::<Vec<_>>()
                .join(","),
        );
        out.push_str("]}");
        out
    }
}

fn json_str(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn point(value: ValueKind, scale: f64, unit: Option<&str>) -> ExchangePoint {
        ExchangePoint {
            id: IoPointId::parse("feed.position", IoDirection::Output),
            var: "feed_position".into(),
            kind: ChannelKind::Word,
            side: PlantSide::Input,
            value,
            source: IoSource::DeviceCommand,
            initial: Value::Int(0),
            node: None,
            channel: None,
            address: None,
            port: None,
            tag: None,
            invert: false,
            note: None,
            scale,
            unit: unit.map(str::to_string),
        }
    }

    /// A measurement on a word channel with a unit crosses as the scaled
    /// INT a 16-bit register holds, and comes back in SI.
    #[test]
    fn a_unit_scales_a_measurement_to_a_register_and_back() {
        assert_eq!(iomap::unit_scale("mm"), Some(1000.0));
        assert_eq!(iomap::unit_scale("0.01mm"), Some(1e5));
        assert_eq!(iomap::unit_scale("m/s"), Some(1.0));
        assert_eq!(iomap::unit_scale("furlong"), None);
        let mm = point(ValueKind::Int, 1000.0, Some("mm"));
        assert_eq!(mm.to_register(Value::Real(0.3)), Value::Int(300));
        assert_eq!(mm.to_register(Value::Real(0.12345)), Value::Int(123));
        assert_eq!(mm.from_register(&Value::Int(300)), Value::Real(0.3));
        let plain = point(ValueKind::Real, 1.0, None);
        assert_eq!(plain.to_register(Value::Real(0.3)), Value::Real(0.3));
        assert_eq!(plain.from_register(&Value::Real(0.3)), Value::Real(0.3));
    }

    /// The spring-return stop is the one no coil names: a single
    /// solenoid has one, a double solenoid none.
    #[test]
    fn the_rest_stop_is_the_stop_no_coil_names() {
        let stops = vec![
            ("retracted".to_string(), 0.0),
            ("extended".to_string(), 0.05),
        ];
        let single = vec![("extend".to_string(), "extended".to_string())];
        let double = vec![
            ("extend".to_string(), "extended".to_string()),
            ("retract".to_string(), "retracted".to_string()),
        ];
        assert_eq!(rest_stop(&single, &stops).as_deref(), Some("retracted"));
        assert_eq!(rest_stop(&double, &stops), None);
        let three = vec![
            ("retracted".to_string(), 0.0),
            ("mid".to_string(), 0.02),
            ("extended".to_string(), 0.05),
        ];
        // Two stops without a coil: no single home to spring back to.
        assert_eq!(rest_stop(&single, &three), None);
    }
}
