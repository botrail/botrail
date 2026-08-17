//! I/O map — the electrical face of a cell, *derived* from what the
//! sequences already say (design/design-electrical.md §3.1).
//!
//! Nothing here is authored: every I/O point falls out of how the
//! existing names are used. A sensor is an input; a signal only ever
//! `Set` is an output coil candidate; a signal only ever read is an
//! external-input candidate; a device commanded `Start`/`Stop` has a run
//! coil, one awaited with `DeviceDone` has an in-position input; a robot
//! driven by a program that does not live on that robot's own controller
//! needs a start output and a done input on the host that drives it. The
//! result is the I/O list a cell needs to be built — the table the
//! electrical drawing starts from — available before any assignment is
//! written.
//!
//! Points carry a *host*: the controller that owns them. Nothing declares
//! hosts yet (that is the assignment layer, I1), so hosts are implicit
//! and mirror the URScript lowering's rule ([`crate::script`]): a program
//! that drives exactly one robot lives on that robot's controller
//! (`<robot>`), a program driving none or several lives on the implicit
//! cell controller (`<cell>`). Reader hosts split points — a signal read
//! from two controllers is two inputs, one wire fanning out.
//!
//! The walk is over a *program set* (default: every sequence); a scene
//! that keeps alternative programs side by side gets a different table
//! for each set, exactly as `simulate_sequences(names)` does.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

use serde::{Deserialize, Serialize};

use crate::seq::{Action, Condition, DeviceCommand, DeviceKind, Sequence, Step};
use crate::Scene;

/// The implicit host of programs that drive no robot or several.
pub const CELL_HOST: &str = "<cell>";

/// The implicit host of a robot's own controller.
pub fn robot_host(robot: &str) -> String {
    format!("<{robot}>")
}

/// Which way a point faces, from the host's point of view.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum IoDirection {
    Input,
    Output,
}

impl IoDirection {
    pub fn as_str(self) -> &'static str {
        match self {
            IoDirection::Input => "input",
            IoDirection::Output => "output",
        }
    }
}

/// The second facet a device or robot point carries when one name owns
/// several wires: a device's numeric commands and a robot's handshake.
/// Rendered as `name.aspect`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum Aspect {
    /// Indexed-transfer start (`Advance`).
    Index,
    /// Vehicle dispatch (`Goto`).
    Dispatch,
    /// Vehicle station select (`Goto`, word).
    Station,
    /// Axis position command (`MoveTo`, word).
    Position,
    /// Speed command (`SetSpeed`, analog).
    Speed,
    /// Robot start (motion / ramp / toolpath from another host).
    Start,
    /// Robot done / idle contact.
    Done,
    /// Robot program-number word.
    Program,
}

impl Aspect {
    pub fn parse(s: &str) -> Option<Aspect> {
        Some(match s {
            "index" => Aspect::Index,
            "dispatch" => Aspect::Dispatch,
            "station" => Aspect::Station,
            "position" => Aspect::Position,
            "speed" => Aspect::Speed,
            "start" => Aspect::Start,
            "done" => Aspect::Done,
            "program" => Aspect::Program,
            _ => return None,
        })
    }

    pub fn as_str(self) -> &'static str {
        match self {
            Aspect::Index => "index",
            Aspect::Dispatch => "dispatch",
            Aspect::Station => "station",
            Aspect::Position => "position",
            Aspect::Speed => "speed",
            Aspect::Start => "start",
            Aspect::Done => "done",
            Aspect::Program => "program",
        }
    }
}

/// The channel type a point needs. Only `Di`/`Do` take part in the bake
/// and the URScript lowering; `Word`/`Ao` are table vocabulary — the
/// numeric commands a bool coil cannot carry.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum ChannelKind {
    Di,
    Do,
    Ai,
    Ao,
    SafeDi,
    SafeDo,
    Word,
}

impl ChannelKind {
    /// The bool families a point of this kind may be bound to: `Di` also
    /// accepts a `SafeDi` channel and vice versa (a safety input is a
    /// digital input on a safety module), same for outputs.
    pub fn compatible(self, channel: ChannelKind) -> bool {
        use ChannelKind::*;
        matches!(
            (self, channel),
            (Di, Di)
                | (Di, SafeDi)
                | (SafeDi, SafeDi)
                | (SafeDi, Di)
                | (Do, Do)
                | (Do, SafeDo)
                | (SafeDo, SafeDo)
                | (SafeDo, Do)
                | (Ai, Ai)
                | (Ao, Ao)
                | (Word, Word)
        )
    }

    pub fn parse(s: &str) -> Option<ChannelKind> {
        Some(match s.to_ascii_lowercase().as_str() {
            "di" => ChannelKind::Di,
            "do" => ChannelKind::Do,
            "ai" => ChannelKind::Ai,
            "ao" => ChannelKind::Ao,
            "safe_di" | "safedi" => ChannelKind::SafeDi,
            "safe_do" | "safedo" => ChannelKind::SafeDo,
            "word" => ChannelKind::Word,
            _ => return None,
        })
    }

    pub fn as_str(self) -> &'static str {
        match self {
            ChannelKind::Di => "DI",
            ChannelKind::Do => "DO",
            ChannelKind::Ai => "AI",
            ChannelKind::Ao => "AO",
            ChannelKind::SafeDi => "SafeDI",
            ChannelKind::SafeDo => "SafeDO",
            ChannelKind::Word => "Word",
        }
    }
}

/// `(name, aspect, direction)` — with the host, the identity of a point.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct IoPointId {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub aspect: Option<Aspect>,
    pub direction: IoDirection,
}

impl IoPointId {
    /// Parses a table label — `name` or `name.aspect` — the way the Python
    /// API accepts it. The last dot only counts when its suffix is an
    /// aspect; a signal that merely contains a dot keeps its full name.
    pub fn parse(label: &str, direction: IoDirection) -> IoPointId {
        if let Some((name, suffix)) = label.rsplit_once('.') {
            if let Some(aspect) = Aspect::parse(suffix) {
                return IoPointId {
                    name: name.to_string(),
                    aspect: Some(aspect),
                    direction,
                };
            }
        }
        IoPointId {
            name: label.to_string(),
            aspect: None,
            direction,
        }
    }

    /// `name` or `name.aspect`.
    pub fn label(&self) -> String {
        match self.aspect {
            Some(a) => format!("{}.{}", self.name, a.as_str()),
            None => self.name.clone(),
        }
    }
}

/// Where a point came from — which derivation rule produced it.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IoSource {
    /// Rule ①: a sensor's input contact.
    Sensor,
    /// Rule ②: an internal signal written on one host and read on
    /// another — a handshake wire.
    Handshake,
    /// Rule ②: an internal signal written and read on one host — a relay,
    /// no I/O.
    Internal,
    /// Rule ③: a signal only written — an output-coil candidate.
    WriteOnly,
    /// Rule ④: a signal only read — an external-input candidate.
    ReadOnly,
    /// Rule ⑤: a device run coil (`Start`/`Stop`).
    DeviceRun,
    /// Rule ⑤: a device's in-position / arrival input (`DeviceDone`).
    DeviceDone,
    /// Rule ⑤: a device's numeric or indexed command.
    DeviceCommand,
    /// Rule ⑤: a magazine (Source/Sink) — presentation, not I/O.
    Cosmetic,
    /// Rule ⑥: robot start output.
    RobotStart,
    /// Rule ⑥: robot done / idle input.
    RobotDone,
    /// Rule ⑥: robot program-number word.
    RobotProgram,
    /// Rule ⑦: a declared point (assignment layer).
    Declared,
}

impl IoSource {
    pub fn as_str(self) -> &'static str {
        match self {
            IoSource::Sensor => "sensor",
            IoSource::Handshake => "signal:handshake",
            IoSource::Internal => "signal:internal",
            IoSource::WriteOnly => "signal:write-only",
            IoSource::ReadOnly => "signal:read-only",
            IoSource::DeviceRun => "device:run",
            IoSource::DeviceDone => "device:done",
            IoSource::DeviceCommand => "device:command",
            IoSource::Cosmetic => "device:cosmetic",
            IoSource::RobotStart => "robot:start",
            IoSource::RobotDone => "robot:done",
            IoSource::RobotProgram => "robot:program",
            IoSource::Declared => "declared",
        }
    }
}

/// Assignment state of a point.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PointStatus {
    /// Wired: the index into `IoMap::bindings`.
    Bound(usize),
    /// Needs a channel, has none.
    Unbound,
    /// A relay: read and written on one host, no I/O.
    Internal,
    /// Magazine (Source/Sink): shown, not counted, not linted.
    Cosmetic,
    /// A coil that is on from t = 0 and never commanded (a belt with
    /// `running=true`): the VFD exists, wired constant.
    Constant,
}

impl PointStatus {
    pub fn as_str(self) -> &'static str {
        match self {
            PointStatus::Bound(_) => "bound",
            PointStatus::Unbound => "unbound",
            PointStatus::Internal => "internal",
            PointStatus::Cosmetic => "cosmetic",
            PointStatus::Constant => "constant",
        }
    }
}

/// A step, keyed the way the timing chart keys it: `(sequence, flat
/// index)`; the name is for display (studio makes duplicates freely).
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord)]
pub struct StepRef {
    pub sequence: String,
    pub index: usize,
    pub name: String,
}

impl fmt::Display for StepRef {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}", self.sequence, self.name)
    }
}

/// One derived I/O point.
#[derive(Debug, Clone)]
pub struct IoPoint {
    pub id: IoPointId,
    pub kind: ChannelKind,
    pub source: IoSource,
    /// The controller that owns the point: `<cell>`, `<robot>` (implicit)
    /// or, once nodes are declared, a node name. `None` when nothing pins
    /// it down (a sensor nobody reads, a constant coil).
    pub host: Option<String>,
    pub safety: bool,
    pub writers: Vec<StepRef>,
    pub readers: Vec<StepRef>,
    pub status: PointStatus,
}

impl IoPoint {
    pub fn label(&self) -> String {
        self.id.label()
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    Error,
    Warning,
    Info,
}

impl Severity {
    pub fn as_str(self) -> &'static str {
        match self {
            Severity::Error => "error",
            Severity::Warning => "warning",
            Severity::Info => "info",
        }
    }
}

/// Lint codes. I0 ships the four that need no assignment; the rest arrive
/// with the assignment layer (design §6).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IoCode {
    /// A name shared by two of signal / sensor / device / robot, or a
    /// signal name containing `.` (collides with `name.aspect` labels).
    NameClash,
    /// Something defined that nothing uses.
    Unreferenced,
    /// A Word/AO point — carried in the table, not by a bool channel.
    WordUnexpressible,
    /// A program landed on the implicit `<cell>` host (drives none or
    /// several robots) — declare its node to place it.
    ImplicitHost,
    /// A point with no channel (only reported once a node exists — before
    /// that, nothing is being assigned yet).
    Unbound,
    /// Two bindings on one channel, or two channels of one node with the
    /// same address / tag.
    Duplicate,
    /// A point bound to a channel of the wrong family (an input on a DO,
    /// a word on a DI).
    Kind,
    /// A node, program, robot, channel or declaration name that does not
    /// exist.
    UnknownRef,
    /// A binding whose point is no longer derived (the sequence changed
    /// under it).
    StaleBinding,
    /// One program listed by two nodes.
    ProgramMultihost,
    /// A binding on a node that does not reach the point's host.
    HostMismatch,
    /// More unbound points of a kind than free channels of that kind on
    /// the host and its stations.
    Capacity,
    /// A safety-class point on a standard channel, or a standard point on
    /// a safety channel.
    Safety,
    /// A two-channel safety pair whose halves disagree (one bound, other
    /// not; different nodes; different kinds; different polarity).
    SafetyPair,
    /// A safety input that no program reads — the E-stop scenario would
    /// change nothing.
    SafetyUnread,
    /// The field device's output type and the channel's expected type
    /// disagree (PNP sensor on an NPN input).
    Polarity,
    /// The field device's voltage and the channel's voltage disagree.
    Voltage,
    /// Two programs of the set drive one output point (the ownership rule
    /// the bake enforces, checked without baking).
    MultipleDrivers,
}

impl IoCode {
    pub fn as_str(self) -> &'static str {
        match self {
            IoCode::NameClash => "name_clash",
            IoCode::Unreferenced => "unreferenced",
            IoCode::WordUnexpressible => "word_unexpressible",
            IoCode::ImplicitHost => "implicit_host",
            IoCode::Unbound => "unbound",
            IoCode::Duplicate => "duplicate",
            IoCode::Kind => "kind",
            IoCode::UnknownRef => "unknown_ref",
            IoCode::StaleBinding => "stale_binding",
            IoCode::ProgramMultihost => "program_multihost",
            IoCode::HostMismatch => "host_mismatch",
            IoCode::Capacity => "capacity",
            IoCode::Safety => "safety",
            IoCode::SafetyPair => "safety_pair",
            IoCode::SafetyUnread => "safety_unread",
            IoCode::Polarity => "polarity",
            IoCode::Voltage => "voltage",
            IoCode::MultipleDrivers => "multiple_drivers",
        }
    }
}

#[derive(Debug, Clone)]
pub struct IoFinding {
    pub severity: Severity,
    pub code: IoCode,
    pub message: String,
    pub at: Vec<StepRef>,
}

impl fmt::Display for IoFinding {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{} {}: {}",
            self.severity.as_str(),
            self.code.as_str(),
            self.message
        )?;
        if !self.at.is_empty() {
            let at: Vec<String> = self.at.iter().map(|s| s.to_string()).collect();
            write!(f, " (at {})", at.join(", "))?;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Default)]
pub struct IoReport {
    pub findings: Vec<IoFinding>,
}

impl IoReport {
    pub fn errors(&self) -> Vec<&IoFinding> {
        self.findings
            .iter()
            .filter(|f| f.severity == Severity::Error)
            .collect()
    }
    pub fn warnings(&self) -> Vec<&IoFinding> {
        self.findings
            .iter()
            .filter(|f| f.severity == Severity::Warning)
            .collect()
    }
    pub fn infos(&self) -> Vec<&IoFinding> {
        self.findings
            .iter()
            .filter(|f| f.severity == Severity::Info)
            .collect()
    }
}

impl fmt::Display for IoReport {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if self.findings.is_empty() {
            return write!(f, "io_report: clean");
        }
        for (i, finding) in self.findings.iter().enumerate() {
            if i > 0 {
                writeln!(f)?;
            }
            write!(f, "{finding}")?;
        }
        Ok(())
    }
}

/// Everything one walk of the cell yields: the points and the findings
/// that fell out of deriving them.
#[derive(Debug, Clone)]
pub struct IoDerivation {
    pub points: Vec<IoPoint>,
    pub report: IoReport,
    /// The program set the derivation was taken over.
    pub sequences: Vec<String>,
    /// Where each program of the set runs: `(program, host)`.
    pub program_hosts: Vec<(String, String)>,
    /// The assignment layer the points were matched against (a copy, so
    /// the tables can print the binding behind a bound point).
    pub io: IoMap,
}

impl IoDerivation {
    /// The binding behind a bound point, with its node and channel.
    pub fn binding_of(&self, p: &IoPoint) -> Option<(&IoBinding, &IoNode, &IoChannel)> {
        let PointStatus::Bound(i) = p.status else {
            return None;
        };
        let b = self.io.bindings.get(i)?;
        let n = self.io.node(&b.node)?;
        let c = n.channels.iter().find(|c| c.id == b.channel)?;
        Some((b, n, c))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IoError {
    UnknownSequence(String),
    /// A sequence references something the scene does not have (a
    /// motion, a robot) — the same conditions `validate_sequence` rejects,
    /// surfaced here without baking.
    Invalid {
        sequence: String,
        message: String,
    },
}

impl fmt::Display for IoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            IoError::UnknownSequence(name) => write!(f, "unknown sequence `{name}`"),
            IoError::Invalid { sequence, message } => {
                write!(f, "sequence `{sequence}`: {message}")
            }
        }
    }
}

impl std::error::Error for IoError {}

// --------------------------------------------------------- authored layer

/// The controller box a program runs on and the channels it owns.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct IoNode {
    pub name: String,
    pub kind: IoNodeKind,
    /// Sequences this node executes (`Plc` / `RobotController` only).
    /// Programs listed nowhere are placed implicitly (one robot driven →
    /// that robot's controller, none or several → `<cell>`).
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub programs: Vec<String>,
    /// The controller this node's I/O belongs to (a remote I/O station or
    /// a safety module hangs off a PLC). Beyond that ownership, a label:
    /// no bus semantics.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub uplink: Option<Uplink>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub channels: Vec<IoChannel>,
    /// A frame or obstacle name marking where the box stands (optional,
    /// display only).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub place: Option<String>,
    /// Part number / model (optional, a table column).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub model: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum IoNodeKind {
    Plc,
    SafetyPlc,
    RemoteIo,
    /// A robot controller, possibly driving several arms (a two-arm
    /// station on one cabinet).
    RobotController {
        robots: Vec<String>,
    },
    /// Anything else — documentation only, cannot host programs or take
    /// bindings.
    Other {
        label: String,
    },
}

impl IoNodeKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            IoNodeKind::Plc => "plc",
            IoNodeKind::SafetyPlc => "safety_plc",
            IoNodeKind::RemoteIo => "remote_io",
            IoNodeKind::RobotController { .. } => "robot_controller",
            IoNodeKind::Other { .. } => "other",
        }
    }

    /// May this kind run programs?
    pub fn hosts_programs(&self) -> bool {
        matches!(self, IoNodeKind::Plc | IoNodeKind::RobotController { .. })
    }
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct Uplink {
    pub parent: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bus: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct IoChannel {
    /// Node-unique id, `"DI2"`.
    pub id: String,
    pub kind: ChannelKind,
    /// The vendor's standard-I/O number, when the channel is one — what
    /// the URScript lowering writes (`set_standard_digital_out(port, …)`).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub port: Option<u32>,
    /// PLC address as the controller spells it (`"%IX0.2"`, `"X02"`) —
    /// a string; dialects live in the Python templates.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub address: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub electrical: Option<Electrical>,
}

/// Electrical facts about a channel or a field device: what a lint can
/// compare when both sides state them.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct Electrical {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub voltage: Option<f64>,
    /// The sensor output type a channel accepts / a device provides. Not
    /// "sink"/"source": those words swap meaning between vendors.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub logic: Option<Logic>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum Logic {
    Pnp,
    Npn,
}

impl Logic {
    pub fn parse(s: &str) -> Option<Logic> {
        match s.to_ascii_lowercase().as_str() {
            "pnp" => Some(Logic::Pnp),
            "npn" => Some(Logic::Npn),
            _ => None,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            Logic::Pnp => "pnp",
            Logic::Npn => "npn",
        }
    }
}

/// Contact type of the field device — documentation for the connection
/// table. Independent of `invert`: an E-stop button is an NC contact whose
/// healthy state reads high, so `estop_ok` needs no inversion.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum Contact {
    No,
    Nc,
}

impl Contact {
    pub fn parse(s: &str) -> Option<Contact> {
        match s.to_ascii_lowercase().as_str() {
            "no" => Some(Contact::No),
            "nc" => Some(Contact::Nc),
            _ => None,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            Contact::No => "NO",
            Contact::Nc => "NC",
        }
    }
}

/// One point wired to one channel. `(point, node)` is the key: a signal
/// fanning out to two controllers is two bindings.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct IoBinding {
    pub point: IoPointId,
    pub node: String,
    pub channel: String,
    /// PLC tag / symbol.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tag: Option<String>,
    /// The field device on the far end (`"BEAM1"`, `"-B1"`, `"YV1"`) —
    /// the connection table's other column.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub field: Option<String>,
    /// Logical inversion between the cell's value and the wire level.
    /// Projection only: the URScript lowering inverts its tests and
    /// writes; the bake is untouched.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub invert: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub contact: Option<Contact>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub safety: bool,
    /// The field device's electrical facts (voltage, output type).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub device: Option<Electrical>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
    /// Placed by `auto_assign_io` rather than by hand. Hand bindings are
    /// never moved; `auto_assign_io(reassign)` drops only these.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub auto: bool,
}

/// An exception to the derivation, or an unmodelled point.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct IoDecl {
    pub name: String,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub role: Option<DeclRole>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub kind: Option<ChannelKind>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub safety: bool,
    /// The other channel of a two-channel safety input.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub pair: Option<String>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub note: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub enum DeclRole {
    /// An external input, whatever the sequences do with the name.
    Input,
    /// An output coil, whatever the sequences do with the name.
    Output,
    /// A relay: no I/O even if only read (a constant flag).
    Internal,
    /// Not on the table at all.
    Exclude,
}

impl DeclRole {
    pub fn parse(s: &str) -> Option<DeclRole> {
        match s.to_ascii_lowercase().as_str() {
            "input" => Some(DeclRole::Input),
            "output" => Some(DeclRole::Output),
            "internal" => Some(DeclRole::Internal),
            "exclude" => Some(DeclRole::Exclude),
            _ => None,
        }
    }
    pub fn as_str(self) -> &'static str {
        match self {
            DeclRole::Input => "input",
            DeclRole::Output => "output",
            DeclRole::Internal => "internal",
            DeclRole::Exclude => "exclude",
        }
    }
}

/// The assignment layer of a scene: nodes, bindings, declarations. Stored
/// on the scene and in the project; everything else in this module is
/// derived from it.
#[derive(Debug, Clone, Default, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "ts", derive(ts_rs::TS), ts(export))]
pub struct IoMap {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub nodes: Vec<IoNode>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub bindings: Vec<IoBinding>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub decls: Vec<IoDecl>,
}

impl IoMap {
    pub fn is_empty(&self) -> bool {
        self.nodes.is_empty() && self.bindings.is_empty() && self.decls.is_empty()
    }

    pub fn node(&self, name: &str) -> Option<&IoNode> {
        self.nodes.iter().find(|n| n.name == name)
    }

    /// The node and its ancestors along `uplink`: the controllers whose
    /// points this node's channels may take. Cycles are cut.
    pub fn reach(&self, node: &str) -> Vec<String> {
        let mut out = vec![node.to_string()];
        let mut cur = node.to_string();
        while let Some(parent) = self
            .node(&cur)
            .and_then(|n| n.uplink.as_ref())
            .map(|u| u.parent.clone())
        {
            if out.contains(&parent) {
                break;
            }
            out.push(parent.clone());
            cur = parent;
        }
        out
    }

    /// The declared node(s) listing `program`.
    pub fn program_nodes(&self, program: &str) -> Vec<&IoNode> {
        self.nodes
            .iter()
            .filter(|n| n.programs.iter().any(|p| p == program))
            .collect()
    }

    /// The declared robot-controller node driving `robot`, if any.
    pub fn robot_controller(&self, robot: &str) -> Option<&IoNode> {
        self.nodes.iter().find(|n| match &n.kind {
            IoNodeKind::RobotController { robots } => robots.iter().any(|r| r == robot),
            _ => false,
        })
    }

    /// The host of `robot`'s own controller: its declared node, else the
    /// implicit `<robot>`.
    pub fn robot_controller_host(&self, robot: &str) -> String {
        self.robot_controller(robot)
            .map(|n| n.name.clone())
            .unwrap_or_else(|| robot_host(robot))
    }

    pub fn decl(&self, name: &str) -> Option<&IoDecl> {
        self.decls.iter().find(|d| d.name == name)
    }

    /// Bindings on `node`.
    pub fn bindings_on<'a>(&'a self, node: &'a str) -> impl Iterator<Item = &'a IoBinding> + 'a {
        self.bindings.iter().filter(move |b| b.node == node)
    }
}

// ------------------------------------------------------- scene state

impl Scene {
    /// The assignment layer as authored (nodes, bindings, declarations).
    pub fn io_map(&self) -> &IoMap {
        &self.io
    }

    /// Replaces the whole assignment layer (project load). Every binding
    /// must name an existing node and channel, every robot controller an
    /// existing robot.
    pub fn set_io_map(&mut self, io: IoMap) -> Result<(), crate::SceneError> {
        for node in &io.nodes {
            self.check_io_node(&io, node)?;
        }
        for binding in &io.bindings {
            check_io_binding(&io, binding)?;
        }
        self.io = io;
        Ok(())
    }

    fn check_io_node(&self, io: &IoMap, node: &IoNode) -> Result<(), crate::SceneError> {
        if node.name.is_empty() {
            return Err(crate::SceneError::BadIo("an I/O node needs a name".into()));
        }
        if let IoNodeKind::RobotController { robots } = &node.kind {
            for robot in robots {
                if self.robot_index(robot).is_none() {
                    return Err(crate::SceneError::UnknownRobot(robot.clone()));
                }
            }
        }
        if let Some(uplink) = &node.uplink {
            if uplink.parent == node.name {
                return Err(crate::SceneError::BadIo(format!(
                    "node `{}` cannot uplink to itself",
                    node.name
                )));
            }
            if io.node(&uplink.parent).is_none() {
                return Err(crate::SceneError::UnknownIoNode(uplink.parent.clone()));
            }
        }
        let mut ids: BTreeSet<&str> = BTreeSet::new();
        for channel in &node.channels {
            if !ids.insert(&channel.id) {
                return Err(crate::SceneError::BadIo(format!(
                    "node `{}` lists channel `{}` twice",
                    node.name, channel.id
                )));
            }
        }
        Ok(())
    }

    /// Adds or replaces a controller / I/O node.
    pub fn upsert_io_node(&mut self, node: IoNode) -> Result<(), crate::SceneError> {
        // Validate against the map as it will be (a node may uplink to one
        // declared earlier, never to itself).
        let mut trial = self.io.clone();
        trial.nodes.retain(|n| n.name != node.name);
        trial.nodes.push(node.clone());
        self.check_io_node(&trial, &node)?;
        if let Some(existing) = self.io.nodes.iter_mut().find(|n| n.name == node.name) {
            *existing = node;
        } else {
            self.io.nodes.push(node);
        }
        Ok(())
    }

    /// Removes a node and every binding on it.
    pub fn remove_io_node(&mut self, name: &str) -> Result<(), crate::SceneError> {
        let before = self.io.nodes.len();
        self.io.nodes.retain(|n| n.name != name);
        if self.io.nodes.len() == before {
            return Err(crate::SceneError::UnknownIoNode(name.to_string()));
        }
        self.io.bindings.retain(|b| b.node != name);
        Ok(())
    }

    /// Wires a point to a channel; a second binding of the same point on
    /// the same node replaces the first (one wire per controller).
    pub fn bind_io(&mut self, binding: IoBinding) -> Result<(), crate::SceneError> {
        check_io_binding(&self.io, &binding)?;
        if let Some(existing) = self
            .io
            .bindings
            .iter_mut()
            .find(|b| b.point == binding.point && b.node == binding.node)
        {
            *existing = binding;
        } else {
            self.io.bindings.push(binding);
        }
        Ok(())
    }

    /// Drops the binding(s) of `point` — on `node`, or on every node when
    /// `None`. Returns how many went.
    pub fn unbind_io(
        &mut self,
        point: &IoPointId,
        node: Option<&str>,
    ) -> Result<usize, crate::SceneError> {
        let before = self.io.bindings.len();
        self.io
            .bindings
            .retain(|b| !(b.point == *point && node.is_none_or(|n| b.node == n)));
        let removed = before - self.io.bindings.len();
        if removed == 0 {
            return Err(crate::SceneError::UnknownIoBinding(point.label()));
        }
        Ok(removed)
    }

    /// Adds or replaces a declaration (keyed by name).
    pub fn declare_io(&mut self, decl: IoDecl) {
        if let Some(existing) = self.io.decls.iter_mut().find(|d| d.name == decl.name) {
            *existing = decl;
        } else {
            self.io.decls.push(decl);
        }
    }

    pub fn undeclare_io(&mut self, name: &str) -> Result<(), crate::SceneError> {
        let before = self.io.decls.len();
        self.io.decls.retain(|d| d.name != name);
        if self.io.decls.len() == before {
            return Err(crate::SceneError::UnknownIoDecl(name.to_string()));
        }
        Ok(())
    }
}

fn check_io_binding(io: &IoMap, binding: &IoBinding) -> Result<(), crate::SceneError> {
    let node = io
        .node(&binding.node)
        .ok_or_else(|| crate::SceneError::UnknownIoNode(binding.node.clone()))?;
    if !node.channels.iter().any(|c| c.id == binding.channel) {
        return Err(crate::SceneError::UnknownIoChannel(
            binding.node.clone(),
            binding.channel.clone(),
        ));
    }
    if matches!(node.kind, IoNodeKind::Other { .. }) {
        return Err(crate::SceneError::BadIo(format!(
            "node `{}` is documentation only (kind other) and takes no bindings",
            node.name
        )));
    }
    Ok(())
}

/// What the projection to a vendor script needs per port: the number and
/// the wire polarity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IoPort {
    pub port: u32,
    pub invert: bool,
}

// ------------------------------------------------------------------ walk

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum DeviceCmd {
    Run,
    Advance,
    Goto,
    MoveTo,
    SetSpeed,
}

/// What one program does with the cell's names, step-attributed.
#[derive(Debug, Default)]
struct ProgramUsage {
    name: String,
    /// `Set` writes: signal → steps.
    writes: BTreeMap<String, Vec<StepRef>>,
    /// `Signal` / `Rising` / `Falling` reads (sensor or signal names).
    reads: BTreeMap<String, Vec<StepRef>>,
    /// Device commands by kind.
    device_cmds: BTreeMap<(String, DeviceCmd), Vec<StepRef>>,
    /// `DeviceDone` reads.
    device_done: BTreeMap<String, Vec<StepRef>>,
    /// Robots driven by motion-class actions (start/ramp/toolpath), with
    /// the motion / toolpath names seen (for the program-number word).
    drives: BTreeMap<usize, (Vec<StepRef>, BTreeSet<String>)>,
    /// Robots this program owns in the `driven_robot` sense (drives, plus
    /// attach / track / untrack) — decides the implicit host.
    owns: BTreeSet<usize>,
    /// `RobotDone` reads.
    robot_done: BTreeMap<usize, Vec<StepRef>>,
    /// Implicit host, decided after the walk.
    host: String,
}

fn push<K: Ord + Clone>(map: &mut BTreeMap<K, Vec<StepRef>>, key: K, at: &StepRef) {
    map.entry(key).or_default().push(at.clone());
}

fn walk_condition(
    cond: &Condition,
    at: &StepRef,
    scene: &Scene,
    u: &mut ProgramUsage,
) -> Result<(), String> {
    match cond {
        Condition::Signal { name, .. }
        | Condition::Rising { name }
        | Condition::Falling { name } => {
            push(&mut u.reads, name.clone(), at);
        }
        Condition::DeviceDone { device } => push(&mut u.device_done, device.clone(), at),
        Condition::RobotDone { robot } => {
            let index = scene
                .robot_index(robot)
                .ok_or_else(|| format!("unknown robot `{robot}`"))?;
            push(&mut u.robot_done, index, at);
        }
        Condition::All(cs) | Condition::Any(cs) => {
            for c in cs {
                walk_condition(c, at, scene, u)?;
            }
        }
        Condition::Immediately | Condition::Done | Condition::Elapsed { .. } => {}
    }
    Ok(())
}

fn walk_action(
    action: &Action,
    at: &StepRef,
    scene: &Scene,
    u: &mut ProgramUsage,
) -> Result<(), String> {
    let cmd = |command: &DeviceCommand| match command {
        DeviceCommand::Start | DeviceCommand::Stop => DeviceCmd::Run,
        DeviceCommand::Advance(_) => DeviceCmd::Advance,
        DeviceCommand::Goto { .. } => DeviceCmd::Goto,
        DeviceCommand::MoveTo(_) => DeviceCmd::MoveTo,
        DeviceCommand::SetSpeed(_) => DeviceCmd::SetSpeed,
    };
    match action {
        Action::Set { signal, .. } => push(&mut u.writes, signal.clone(), at),
        Action::Device { device, command } => {
            push(&mut u.device_cmds, (device.clone(), cmd(command)), at)
        }
        Action::StartMotion { motion } => {
            let m = scene
                .motions()
                .iter()
                .find(|m| &m.name == motion)
                .ok_or_else(|| format!("unknown motion `{motion}`"))?;
            let entry = u.drives.entry(m.robot).or_default();
            entry.0.push(at.clone());
            entry.1.insert(motion.clone());
            u.owns.insert(m.robot);
        }
        Action::StartRamp { robot, .. } => {
            let r = scene.resolve_seq_robot(robot)?;
            u.drives.entry(r).or_default().0.push(at.clone());
            u.owns.insert(r);
        }
        Action::StartToolpath { robot, toolpath } => {
            let r = scene.resolve_seq_robot(robot)?;
            let entry = u.drives.entry(r).or_default();
            entry.0.push(at.clone());
            entry.1.insert(toolpath.clone());
            u.owns.insert(r);
        }
        Action::Attach { robot, .. } | Action::Track { robot, .. } | Action::Untrack { robot } => {
            let r = scene.resolve_seq_robot(robot)?;
            u.owns.insert(r);
        }
        Action::Detach { .. } => {}
    }
    Ok(())
}

/// Pre-order over the authored tree — the same numbering as the rollout's
/// flat steps: a branching step takes its index, then its arms in order.
fn walk_steps(
    steps: &[Step],
    sequence: &str,
    counter: &mut usize,
    scene: &Scene,
    u: &mut ProgramUsage,
) -> Result<(), String> {
    for step in steps {
        let at = StepRef {
            sequence: sequence.to_string(),
            index: *counter,
            name: step.name.clone(),
        };
        *counter += 1;
        for action in &step.actions {
            walk_action(action, &at, scene, u)?;
        }
        if step.select.is_empty() {
            walk_condition(&step.transition, &at, scene, u)?;
        } else {
            for arm in &step.select {
                walk_condition(&arm.condition, &at, scene, u)?;
            }
            for arm in &step.select {
                walk_steps(&arm.steps, sequence, counter, scene, u)?;
            }
        }
    }
    Ok(())
}

fn usage(scene: &Scene, io: &IoMap, sequence: &Sequence) -> Result<ProgramUsage, IoError> {
    let mut u = ProgramUsage {
        name: sequence.name.clone(),
        ..Default::default()
    };
    let mut counter = 0;
    walk_steps(&sequence.steps, &sequence.name, &mut counter, scene, &mut u).map_err(
        |message| IoError::Invalid {
            sequence: sequence.name.clone(),
            message,
        },
    )?;
    // A declared node wins; otherwise implicit placement mirrors
    // `script::driven_robot`: one robot owned → that robot's controller
    // (its declared node when there is one), none or several → the cell.
    u.host = match io.program_nodes(&sequence.name).first() {
        Some(node) => node.name.clone(),
        None if u.owns.len() == 1 => {
            let r = *u.owns.iter().next().unwrap();
            io.robot_controller_host(&scene.robots()[r].name)
        }
        None => CELL_HOST.to_string(),
    };
    Ok(u)
}

// -------------------------------------------------------------- derive

/// Points keyed for merging: several programs on one host driving the
/// same robot, or reading the same signal, land on one point.
type PointKey = (String, Option<Aspect>, IoDirection, Option<String>);

struct Builder {
    points: BTreeMap<PointKey, IoPoint>,
    findings: Vec<IoFinding>,
}

impl Builder {
    #[allow(clippy::too_many_arguments)]
    fn point(
        &mut self,
        name: &str,
        aspect: Option<Aspect>,
        direction: IoDirection,
        host: Option<&str>,
        kind: ChannelKind,
        source: IoSource,
        status: PointStatus,
    ) -> &mut IoPoint {
        let key = (
            name.to_string(),
            aspect,
            direction,
            host.map(str::to_string),
        );
        self.points.entry(key).or_insert_with(|| IoPoint {
            id: IoPointId {
                name: name.to_string(),
                aspect,
                direction,
            },
            kind,
            source,
            host: host.map(str::to_string),
            safety: false,
            writers: Vec::new(),
            readers: Vec::new(),
            status,
        })
    }

    fn finding(&mut self, severity: Severity, code: IoCode, message: String, at: Vec<StepRef>) {
        self.findings.push(IoFinding {
            severity,
            code,
            message,
            at,
        });
    }
}

fn dedup(steps: &mut Vec<StepRef>) {
    steps.sort();
    steps.dedup();
}

/// Groups `(host, step)` pairs by host, preserving first-seen host order
/// for determinism-by-sort later.
fn by_host<'a>(
    items: impl Iterator<Item = (&'a str, &'a [StepRef])>,
) -> BTreeMap<String, Vec<StepRef>> {
    let mut out: BTreeMap<String, Vec<StepRef>> = BTreeMap::new();
    for (host, steps) in items {
        out.entry(host.to_string())
            .or_default()
            .extend(steps.iter().cloned());
    }
    out
}

/// Derives the I/O points of `scene` over the program set `sequences`
/// (`None` = every sequence), with the findings that fall out of the
/// derivation. Nothing is stored: call again after any edit.
pub fn derive(scene: &Scene, sequences: Option<&[&str]>) -> Result<IoDerivation, IoError> {
    let io = scene.io_map();
    let programs: Vec<&Sequence> = match sequences {
        None => scene.sequences().iter().collect(),
        Some(names) => names
            .iter()
            .map(|n| {
                scene
                    .sequence(n)
                    .ok_or_else(|| IoError::UnknownSequence(n.to_string()))
            })
            .collect::<Result<_, _>>()?,
    };
    let usages: Vec<ProgramUsage> = programs
        .iter()
        .map(|s| usage(scene, io, s))
        .collect::<Result<_, _>>()?;

    let mut b = Builder {
        points: BTreeMap::new(),
        findings: Vec::new(),
    };

    // ---- the map's own references ------------------------------------------
    {
        let mut seen: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
        for node in &io.nodes {
            for program in &node.programs {
                seen.entry(program.as_str()).or_default().push(&node.name);
                if scene.sequence(program).is_none() {
                    b.finding(
                        Severity::Error,
                        IoCode::UnknownRef,
                        format!(
                            "node `{}` lists program `{program}`, which is not a sequence",
                            node.name
                        ),
                        Vec::new(),
                    );
                }
            }
            if !node.programs.is_empty() && !node.kind.hosts_programs() {
                b.finding(
                    Severity::Error,
                    IoCode::UnknownRef,
                    format!(
                        "node `{}` is a {} and cannot run programs (only plc / robot_controller do)",
                        node.name,
                        node.kind.as_str()
                    ),
                    Vec::new(),
                );
            }
            if let IoNodeKind::RobotController { robots } = &node.kind {
                for robot in robots {
                    if scene.robot_index(robot).is_none() {
                        b.finding(
                            Severity::Error,
                            IoCode::UnknownRef,
                            format!(
                                "node `{}` lists robot `{robot}`, which is not in the scene",
                                node.name
                            ),
                            Vec::new(),
                        );
                    }
                }
            }
            if let Some(uplink) = &node.uplink {
                if io.node(&uplink.parent).is_none() {
                    b.finding(
                        Severity::Error,
                        IoCode::UnknownRef,
                        format!(
                            "node `{}` uplinks to `{}`, which is not a node",
                            node.name, uplink.parent
                        ),
                        Vec::new(),
                    );
                }
            }
        }
        for (program, nodes) in seen {
            if nodes.len() > 1 {
                b.finding(
                    Severity::Error,
                    IoCode::ProgramMultihost,
                    format!("program `{program}` is listed by nodes {} — one program runs on one controller", nodes.join(", ")),
                    Vec::new(),
                );
            }
        }
        for decl in &io.decls {
            if let Some(pair) = &decl.pair {
                if io.decl(pair).is_none()
                    && !scene.sensors().iter().any(|s| &s.name == pair)
                    && !scene.signals().iter().any(|s| &s.name == pair)
                {
                    b.finding(
                        Severity::Error,
                        IoCode::UnknownRef,
                        format!("declaration `{}` pairs with `{pair}`, which is neither declared nor a signal / sensor", decl.name),
                        Vec::new(),
                    );
                }
            }
        }
    }

    // ---- placement notes -------------------------------------------------
    for u in &usages {
        if u.host == CELL_HOST {
            b.finding(
                Severity::Info,
                IoCode::ImplicitHost,
                format!(
                    "program `{}` drives {} robot(s) and lands on the implicit `<cell>` host — declare \
                     the node that runs it (programs=) to place its I/O",
                    u.name,
                    u.owns.len()
                ),
                Vec::new(),
            );
        }
    }

    let role_of = |name: &str| io.decl(name).and_then(|d| d.role);

    // ---- ① sensors --------------------------------------------------------
    for sensor in scene.sensors() {
        if role_of(&sensor.name) == Some(DeclRole::Exclude) {
            continue;
        }
        let readers = by_host(usages.iter().filter_map(|u| {
            u.reads
                .get(&sensor.name)
                .map(|s| (u.host.as_str(), s.as_slice()))
        }));
        if readers.is_empty() {
            b.point(
                &sensor.name,
                None,
                IoDirection::Input,
                None,
                ChannelKind::Di,
                IoSource::Sensor,
                PointStatus::Unbound,
            );
        }
        for (host, steps) in readers {
            let p = b.point(
                &sensor.name,
                None,
                IoDirection::Input,
                Some(&host),
                ChannelKind::Di,
                IoSource::Sensor,
                PointStatus::Unbound,
            );
            p.readers.extend(steps);
        }
    }

    // ---- ②③④ internal signals ---------------------------------------------
    for signal in scene.signals() {
        let name = &signal.name;
        let role = role_of(name);
        if role == Some(DeclRole::Exclude) {
            continue;
        }
        let writes: Vec<(&str, &[StepRef])> = usages
            .iter()
            .filter_map(|u| u.writes.get(name).map(|s| (u.host.as_str(), s.as_slice())))
            .collect();
        let reads = by_host(
            usages
                .iter()
                .filter_map(|u| u.reads.get(name).map(|s| (u.host.as_str(), s.as_slice()))),
        );
        let all_writers =
            || -> Vec<StepRef> { writes.iter().flat_map(|(_, s)| s.iter().cloned()).collect() };
        match role {
            Some(DeclRole::Input) => {
                // Declared an external input: one DI per reader host, or an
                // unhosted one if nobody reads it (yet).
                if reads.is_empty() {
                    let p = b.point(
                        name,
                        None,
                        IoDirection::Input,
                        None,
                        ChannelKind::Di,
                        IoSource::ReadOnly,
                        PointStatus::Unbound,
                    );
                    p.writers.extend(all_writers());
                }
                for (host, steps) in &reads {
                    let p = b.point(
                        name,
                        None,
                        IoDirection::Input,
                        Some(host),
                        ChannelKind::Di,
                        IoSource::ReadOnly,
                        PointStatus::Unbound,
                    );
                    p.readers.extend(steps.iter().cloned());
                    p.writers.extend(all_writers());
                }
                continue;
            }
            Some(DeclRole::Output) => {
                let host = writes.first().map(|(h, _)| h.to_string());
                let p = b.point(
                    name,
                    None,
                    IoDirection::Output,
                    host.as_deref(),
                    ChannelKind::Do,
                    IoSource::WriteOnly,
                    PointStatus::Unbound,
                );
                p.writers.extend(all_writers());
                for steps in reads.values() {
                    p.readers.extend(steps.iter().cloned());
                }
                continue;
            }
            Some(DeclRole::Internal) => {
                if writes.is_empty() && reads.is_empty() {
                    b.finding(
                        Severity::Info,
                        IoCode::Unreferenced,
                        format!("signal `{name}` is defined but no program writes or reads it"),
                        Vec::new(),
                    );
                    continue;
                }
                let host = writes
                    .first()
                    .map(|(h, _)| h.to_string())
                    .or_else(|| reads.keys().next().cloned());
                let p = b.point(
                    name,
                    None,
                    IoDirection::Output,
                    host.as_deref(),
                    ChannelKind::Do,
                    IoSource::Internal,
                    PointStatus::Internal,
                );
                p.writers.extend(all_writers());
                for steps in reads.values() {
                    p.readers.extend(steps.iter().cloned());
                }
                continue;
            }
            Some(DeclRole::Exclude) => unreachable!("excluded above"),
            None => {}
        }
        match (writes.is_empty(), reads.is_empty()) {
            (true, true) => b.finding(
                Severity::Info,
                IoCode::Unreferenced,
                format!("signal `{name}` is defined but no program writes or reads it"),
                Vec::new(),
            ),
            (false, true) => {
                // ③ write-only: an output-coil candidate on the writer's host.
                let host = writes[0].0.to_string();
                let p = b.point(
                    name,
                    None,
                    IoDirection::Output,
                    Some(&host),
                    ChannelKind::Do,
                    IoSource::WriteOnly,
                    PointStatus::Unbound,
                );
                p.writers.extend(all_writers());
            }
            (true, false) => {
                // ④ read-only: an external-input candidate per reader host.
                for (host, steps) in reads {
                    let p = b.point(
                        name,
                        None,
                        IoDirection::Input,
                        Some(&host),
                        ChannelKind::Di,
                        IoSource::ReadOnly,
                        PointStatus::Unbound,
                    );
                    p.readers.extend(steps);
                }
            }
            (false, false) => {
                // ② written and read: a relay if every reader shares the
                // writer's host, else a handshake wire (one output, one
                // input per other host).
                let host = writes[0].0.to_string();
                let others: Vec<(&String, &Vec<StepRef>)> =
                    reads.iter().filter(|(h, _)| **h != host).collect();
                let (source, status) = if others.is_empty() {
                    (IoSource::Internal, PointStatus::Internal)
                } else {
                    (IoSource::Handshake, PointStatus::Unbound)
                };
                let p = b.point(
                    name,
                    None,
                    IoDirection::Output,
                    Some(&host),
                    ChannelKind::Do,
                    source,
                    status,
                );
                p.writers.extend(all_writers());
                for steps in reads.values() {
                    p.readers.extend(steps.iter().cloned());
                }
                for (other, steps) in others {
                    let p = b.point(
                        name,
                        None,
                        IoDirection::Input,
                        Some(other),
                        ChannelKind::Di,
                        IoSource::Handshake,
                        PointStatus::Unbound,
                    );
                    p.readers.extend(steps.iter().cloned());
                }
            }
        }
    }

    // ---- ⑤ devices --------------------------------------------------------
    for device in scene.devices() {
        let name = &device.name;
        let role = role_of(name);
        if role == Some(DeclRole::Exclude) {
            continue;
        }
        let cmds = |kind: DeviceCmd| -> Vec<(&str, &[StepRef])> {
            usages
                .iter()
                .filter_map(|u| {
                    u.device_cmds
                        .get(&(name.clone(), kind))
                        .map(|s| (u.host.as_str(), s.as_slice()))
                })
                .collect()
        };
        let done = by_host(usages.iter().filter_map(|u| {
            u.device_done
                .get(name)
                .map(|s| (u.host.as_str(), s.as_slice()))
        }));
        let is_magazine = matches!(
            device.kind,
            DeviceKind::Source { .. } | DeviceKind::Sink { .. }
        );
        // A magazine stays cosmetic unless declared a real output (a
        // feeder the cell controller actually starts).
        if is_magazine && role != Some(DeclRole::Output) {
            let p = b.point(
                name,
                None,
                IoDirection::Output,
                None,
                ChannelKind::Do,
                IoSource::Cosmetic,
                PointStatus::Cosmetic,
            );
            for (_, steps) in cmds(DeviceCmd::Run) {
                p.writers.extend(steps.iter().cloned());
            }
            continue;
        }
        let mut commanded = false;
        let table: [(DeviceCmd, Option<Aspect>, ChannelKind, IoSource); 6] = [
            (DeviceCmd::Run, None, ChannelKind::Do, IoSource::DeviceRun),
            (
                DeviceCmd::Advance,
                Some(Aspect::Index),
                ChannelKind::Do,
                IoSource::DeviceCommand,
            ),
            (
                DeviceCmd::Goto,
                Some(Aspect::Dispatch),
                ChannelKind::Do,
                IoSource::DeviceCommand,
            ),
            (
                DeviceCmd::Goto,
                Some(Aspect::Station),
                ChannelKind::Word,
                IoSource::DeviceCommand,
            ),
            (
                DeviceCmd::MoveTo,
                Some(Aspect::Position),
                ChannelKind::Word,
                IoSource::DeviceCommand,
            ),
            (
                DeviceCmd::SetSpeed,
                Some(Aspect::Speed),
                ChannelKind::Ao,
                IoSource::DeviceCommand,
            ),
        ];
        for (cmd, aspect, kind, source) in table {
            let uses = cmds(cmd);
            if uses.is_empty() {
                continue;
            }
            commanded = true;
            let host = uses[0].0.to_string();
            let mut writers: Vec<StepRef> =
                uses.iter().flat_map(|(_, s)| s.iter().cloned()).collect();
            dedup(&mut writers);
            let p = b.point(
                name,
                aspect,
                IoDirection::Output,
                Some(&host),
                kind,
                source,
                PointStatus::Unbound,
            );
            p.writers.extend(writers.iter().cloned());
            if matches!(kind, ChannelKind::Word | ChannelKind::Ao) {
                let label = IoPointId {
                    name: name.clone(),
                    aspect,
                    direction: IoDirection::Output,
                }
                .label();
                b.finding(
                    Severity::Info,
                    IoCode::WordUnexpressible,
                    format!(
                        "`{label}` is a {} point (numeric command) — listed in the table, not carried by \
                         a bool channel; the URScript lowering keeps it as a comment",
                        kind.as_str()
                    ),
                    writers,
                );
            }
        }
        for (host, steps) in done {
            commanded = true;
            let p = b.point(
                name,
                None,
                IoDirection::Input,
                Some(&host),
                ChannelKind::Di,
                IoSource::DeviceDone,
                PointStatus::Unbound,
            );
            p.readers.extend(steps);
        }
        if !commanded {
            match device.kind {
                DeviceKind::Conveyor { running: true, .. } => {
                    b.point(
                        name,
                        None,
                        IoDirection::Output,
                        None,
                        ChannelKind::Do,
                        IoSource::DeviceRun,
                        PointStatus::Constant,
                    );
                }
                _ if role == Some(DeclRole::Output) => {
                    // A declared feeder nobody starts yet: keep the row.
                    b.point(
                        name,
                        None,
                        IoDirection::Output,
                        None,
                        ChannelKind::Do,
                        IoSource::DeviceRun,
                        PointStatus::Unbound,
                    );
                }
                _ => b.finding(
                    Severity::Info,
                    IoCode::Unreferenced,
                    format!("device `{name}` is defined but no program commands or awaits it"),
                    Vec::new(),
                ),
            }
        }
    }

    // ---- ⑥ robots ---------------------------------------------------------
    for u in &usages {
        for (robot, (steps, motions)) in &u.drives {
            let rname = &scene.robots()[*robot].name;
            let ctrl = io.robot_controller_host(rname);
            if u.host == ctrl {
                continue; // the robot's own controller runs the program: no wire
            }
            let mut steps = steps.clone();
            dedup(&mut steps);
            let p = b.point(
                rname,
                Some(Aspect::Start),
                IoDirection::Output,
                Some(&u.host),
                ChannelKind::Do,
                IoSource::RobotStart,
                PointStatus::Unbound,
            );
            p.writers.extend(steps.iter().cloned());
            let p = b.point(
                rname,
                Some(Aspect::Done),
                IoDirection::Input,
                Some(&u.host),
                ChannelKind::Di,
                IoSource::RobotDone,
                PointStatus::Unbound,
            );
            p.readers.extend(steps.iter().cloned());
            // The mirror on the robot's own controller, when it is a
            // declared node: start comes in, done goes out.
            if io.robot_controller(rname).is_some() {
                let p = b.point(
                    rname,
                    Some(Aspect::Start),
                    IoDirection::Input,
                    Some(&ctrl),
                    ChannelKind::Di,
                    IoSource::RobotStart,
                    PointStatus::Unbound,
                );
                p.readers.extend(steps.iter().cloned());
                let p = b.point(
                    rname,
                    Some(Aspect::Done),
                    IoDirection::Output,
                    Some(&ctrl),
                    ChannelKind::Do,
                    IoSource::RobotDone,
                    PointStatus::Unbound,
                );
                p.writers.extend(steps.iter().cloned());
            }
            if motions.len() >= 2 {
                let p = b.point(
                    rname,
                    Some(Aspect::Program),
                    IoDirection::Output,
                    Some(&u.host),
                    ChannelKind::Word,
                    IoSource::RobotProgram,
                    PointStatus::Unbound,
                );
                p.writers.extend(steps.iter().cloned());
                b.finding(
                    Severity::Info,
                    IoCode::WordUnexpressible,
                    format!(
                        "`{rname}.program` selects among {} motions ({}) — a program-number word, listed \
                         in the table only",
                        motions.len(),
                        motions.iter().cloned().collect::<Vec<_>>().join(", ")
                    ),
                    steps,
                );
            }
        }
        for (robot, steps) in &u.robot_done {
            let rname = &scene.robots()[*robot].name;
            let ctrl = io.robot_controller_host(rname);
            if u.host == ctrl {
                continue; // a program asking after its own robot: the idle test, no wire
            }
            let p = b.point(
                rname,
                Some(Aspect::Done),
                IoDirection::Input,
                Some(&u.host),
                ChannelKind::Di,
                IoSource::RobotDone,
                PointStatus::Unbound,
            );
            p.readers.extend(steps.iter().cloned());
            if io.robot_controller(rname).is_some() {
                let p = b.point(
                    rname,
                    Some(Aspect::Done),
                    IoDirection::Output,
                    Some(&ctrl),
                    ChannelKind::Do,
                    IoSource::RobotDone,
                    PointStatus::Unbound,
                );
                p.readers.extend(steps.iter().cloned());
            }
        }
    }

    // ---- ⑦ declarations ----------------------------------------------------
    for decl in &io.decls {
        let existing: Vec<&PointKey> = b
            .points
            .keys()
            .filter(|(name, aspect, _, _)| name == &decl.name && aspect.is_none())
            .collect();
        if existing.is_empty() {
            match decl.role {
                Some(DeclRole::Input) | Some(DeclRole::Output) => {
                    let direction = if decl.role == Some(DeclRole::Input) {
                        IoDirection::Input
                    } else {
                        IoDirection::Output
                    };
                    let kind = decl.kind.unwrap_or(match direction {
                        IoDirection::Input => ChannelKind::Di,
                        IoDirection::Output => ChannelKind::Do,
                    });
                    let p = b.point(
                        &decl.name,
                        None,
                        direction,
                        None,
                        kind,
                        IoSource::Declared,
                        PointStatus::Unbound,
                    );
                    p.safety = decl.safety;
                }
                Some(DeclRole::Exclude) => {
                    let known = scene.signals().iter().any(|s| s.name == decl.name)
                        || scene.sensors().iter().any(|s| s.name == decl.name)
                        || scene.devices().iter().any(|d| d.name == decl.name);
                    if !known {
                        b.finding(
                            Severity::Error,
                            IoCode::UnknownRef,
                            format!("declaration `{}` excludes a name that is not a signal, sensor or device", decl.name),
                            Vec::new(),
                        );
                    }
                }
                Some(DeclRole::Internal) | None => {
                    let known = scene.signals().iter().any(|s| s.name == decl.name)
                        || scene.sensors().iter().any(|s| s.name == decl.name)
                        || scene.devices().iter().any(|d| d.name == decl.name);
                    if !known {
                        b.finding(
                            Severity::Error,
                            IoCode::UnknownRef,
                            format!(
                                "declaration `{}` names nothing in the scene — give it role=input or role=output to add an unmodelled point",
                                decl.name
                            ),
                            Vec::new(),
                        );
                    }
                }
            }
        } else {
            let keys: Vec<PointKey> = existing.into_iter().cloned().collect();
            for key in keys {
                if let Some(p) = b.points.get_mut(&key) {
                    if let Some(kind) = decl.kind {
                        p.kind = kind;
                    }
                    if decl.safety {
                        p.safety = true;
                    }
                }
            }
        }
    }

    // ---- bindings ------------------------------------------------------------
    for (index, binding) in io.bindings.iter().enumerate() {
        let Some(node) = io.node(&binding.node) else {
            b.finding(
                Severity::Error,
                IoCode::UnknownRef,
                format!(
                    "binding `{}` names node `{}`, which does not exist",
                    binding.point.label(),
                    binding.node
                ),
                Vec::new(),
            );
            continue;
        };
        let Some(channel) = node.channels.iter().find(|c| c.id == binding.channel) else {
            b.finding(
                Severity::Error,
                IoCode::UnknownRef,
                format!(
                    "binding `{}` names channel `{}.{}`, which does not exist",
                    binding.point.label(),
                    node.name,
                    binding.channel
                ),
                Vec::new(),
            );
            continue;
        };
        let reach = io.reach(&node.name);
        let candidates: Vec<PointKey> = b
            .points
            .iter()
            .filter(|(k, _)| {
                k.0 == binding.point.name
                    && k.1 == binding.point.aspect
                    && k.2 == binding.point.direction
            })
            .map(|(k, _)| k.clone())
            .collect();
        if candidates.is_empty() {
            b.finding(
                Severity::Warning,
                IoCode::StaleBinding,
                format!(
                    "binding `{}` on {}.{} has no point behind it any more — the sequences no longer use \
                     `{}` that way (or the program set left it out)",
                    binding.point.label(),
                    node.name,
                    channel.id,
                    binding.point.name
                ),
                Vec::new(),
            );
            continue;
        }
        let matched: Vec<PointKey> = candidates
            .iter()
            .filter(|k| match &k.3 {
                None => true,
                Some(host) => reach.iter().any(|r| r == host),
            })
            .cloned()
            .collect();
        if matched.is_empty() {
            let hosts: Vec<String> = candidates
                .iter()
                .map(|k| k.3.clone().unwrap_or_default())
                .collect();
            b.finding(
                Severity::Error,
                IoCode::HostMismatch,
                format!(
                    "binding `{}` sits on {}.{}, but the point lives on {} — bind it there (or uplink \
                     {} to that controller)",
                    binding.point.label(),
                    node.name,
                    channel.id,
                    hosts.join(" / "),
                    node.name
                ),
                Vec::new(),
            );
            continue;
        }
        for key in matched {
            let p = b.points.get_mut(&key).unwrap();
            if !p.kind.compatible(channel.kind) {
                b.findings.push(IoFinding {
                    severity: Severity::Error,
                    code: IoCode::Kind,
                    message: format!(
                        "`{}` is a {} point but {}.{} is a {} channel",
                        binding.point.label(),
                        p.kind.as_str(),
                        node.name,
                        channel.id,
                        channel.kind.as_str()
                    ),
                    at: Vec::new(),
                });
            }
            if matches!(
                p.status,
                PointStatus::Unbound | PointStatus::Cosmetic | PointStatus::Constant
            ) {
                p.status = PointStatus::Bound(index);
            }
            if binding.safety {
                p.safety = true;
            }
        }
    }
    // Duplicates: two bindings on one channel; two channels of a node
    // sharing an address or a tag.
    {
        let mut on_channel: BTreeMap<(&str, &str), Vec<String>> = BTreeMap::new();
        let mut on_tag: BTreeMap<(&str, &str), Vec<String>> = BTreeMap::new();
        for binding in &io.bindings {
            on_channel
                .entry((&binding.node, &binding.channel))
                .or_default()
                .push(binding.point.label());
            if let Some(tag) = &binding.tag {
                on_tag
                    .entry((&binding.node, tag))
                    .or_default()
                    .push(binding.point.label());
            }
        }
        for ((node, channel), labels) in on_channel {
            if labels.len() > 1 {
                b.finding(
                    Severity::Error,
                    IoCode::Duplicate,
                    format!(
                        "channel {node}.{channel} is bound to {} points: {}",
                        labels.len(),
                        labels.join(", ")
                    ),
                    Vec::new(),
                );
            }
        }
        for ((node, tag), labels) in on_tag {
            if labels.len() > 1 {
                b.finding(
                    Severity::Error,
                    IoCode::Duplicate,
                    format!(
                        "tag `{tag}` on {node} names {} points: {}",
                        labels.len(),
                        labels.join(", ")
                    ),
                    Vec::new(),
                );
            }
        }
        for node in &io.nodes {
            let mut addresses: BTreeMap<&str, Vec<&str>> = BTreeMap::new();
            for channel in &node.channels {
                if let Some(address) = &channel.address {
                    addresses.entry(address).or_default().push(&channel.id);
                }
            }
            for (address, ids) in addresses {
                if ids.len() > 1 {
                    b.finding(
                        Severity::Error,
                        IoCode::Duplicate,
                        format!(
                            "address `{address}` on {} is shared by channels {}",
                            node.name,
                            ids.join(", ")
                        ),
                        Vec::new(),
                    );
                }
            }
        }
    }
    // Unbound: once anything is being assigned (a node exists), every real
    // I/O point without a channel is a finding — an error where the wire
    // is certain, a warning where the point is a candidate.
    if !io.nodes.is_empty() {
        let mut unbound: Vec<IoFinding> = Vec::new();
        for p in b.points.values() {
            if p.status != PointStatus::Unbound {
                continue;
            }
            let (severity, why) = match p.source {
                IoSource::Sensor if p.readers.is_empty() => {
                    (Severity::Warning, "a sensor nobody reads")
                }
                IoSource::Sensor => (Severity::Error, "a sensor the sequences read"),
                IoSource::Handshake => (Severity::Error, "a handshake wire"),
                IoSource::WriteOnly => (
                    Severity::Warning,
                    "an output-coil candidate (a coil, or just a flag?)",
                ),
                IoSource::ReadOnly => (
                    Severity::Warning,
                    "an external-input candidate (a contact, or a constant?)",
                ),
                IoSource::DeviceRun => (Severity::Error, "a device run coil"),
                IoSource::DeviceDone => (Severity::Error, "an in-position input"),
                IoSource::DeviceCommand if matches!(p.kind, ChannelKind::Do) => {
                    (Severity::Error, "a device start")
                }
                IoSource::DeviceCommand => (Severity::Info, "a numeric command (table only)"),
                IoSource::RobotStart => (Severity::Error, "a robot start"),
                IoSource::RobotDone => (Severity::Error, "a robot done contact"),
                IoSource::RobotProgram => (Severity::Info, "a program-number word (table only)"),
                IoSource::Declared => (Severity::Error, "a declared point"),
                IoSource::Internal | IoSource::Cosmetic => continue,
            };
            let mut at = p.writers.clone();
            at.extend(p.readers.iter().cloned());
            dedup(&mut at);
            let host = p.host.clone().unwrap_or_else(|| "(any node)".to_string());
            let hint = if host.starts_with('<') {
                " — its host is implicit; declare the node that runs the program (programs=)"
            } else {
                ""
            };
            unbound.push(IoFinding {
                severity,
                code: IoCode::Unbound,
                message: format!(
                    "`{}` ({} {why}) has no channel on {host}{hint}",
                    p.label(),
                    p.kind.as_str()
                ),
                at,
            });
        }
        b.findings.extend(unbound);
    }

    // ---- electrical / safety / ownership lints ---------------------------------
    {
        // Safety class vs channel family, field device vs channel facts.
        let mut extra: Vec<IoFinding> = Vec::new();
        for p in b.points.values() {
            let PointStatus::Bound(i) = p.status else {
                continue;
            };
            let Some(binding) = io.bindings.get(i) else {
                continue;
            };
            let Some(channel) = io
                .node(&binding.node)
                .and_then(|n| n.channels.iter().find(|c| c.id == binding.channel))
            else {
                continue;
            };
            let safe_channel = matches!(channel.kind, ChannelKind::SafeDi | ChannelKind::SafeDo);
            if p.safety && !safe_channel {
                extra.push(IoFinding {
                    severity: Severity::Warning,
                    code: IoCode::Safety,
                    message: format!(
                        "`{}` is a safety point but {}.{} is a standard {} channel",
                        p.label(),
                        binding.node,
                        channel.id,
                        channel.kind.as_str()
                    ),
                    at: Vec::new(),
                });
            } else if !p.safety && safe_channel {
                extra.push(IoFinding {
                    severity: Severity::Warning,
                    code: IoCode::Safety,
                    message: format!(
                        "`{}` is a standard point on the safety channel {}.{}",
                        p.label(),
                        binding.node,
                        channel.id
                    ),
                    at: Vec::new(),
                });
            }
            if let (Some(device), Some(chan)) = (&binding.device, &channel.electrical) {
                if let (Some(dv), Some(cv)) = (device.voltage, chan.voltage) {
                    if (dv - cv).abs() > 0.5 {
                        extra.push(IoFinding {
                            severity: Severity::Warning,
                            code: IoCode::Voltage,
                            message: format!(
                                "`{}` ({} V) is wired to {}.{} ({} V)",
                                p.label(),
                                dv,
                                binding.node,
                                channel.id,
                                cv
                            ),
                            at: Vec::new(),
                        });
                    }
                }
                if let (Some(dl), Some(cl)) = (device.logic, chan.logic) {
                    if dl != cl {
                        extra.push(IoFinding {
                            severity: Severity::Warning,
                            code: IoCode::Polarity,
                            message: format!(
                                "`{}` is a {} device on {}.{}, which expects {}",
                                p.label(),
                                dl.as_str().to_ascii_uppercase(),
                                binding.node,
                                channel.id,
                                cl.as_str().to_ascii_uppercase()
                            ),
                            at: Vec::new(),
                        });
                    }
                }
            }
        }
        // Safety inputs nobody reads.
        for p in b.points.values() {
            if p.safety && p.id.direction == IoDirection::Input && p.readers.is_empty() {
                extra.push(IoFinding {
                    severity: Severity::Warning,
                    code: IoCode::SafetyUnread,
                    message: format!(
                        "safety input `{}` is read by no program — forcing it in a scenario changes \
                         nothing (AND it into the transitions that must not fire without it)",
                        p.label()
                    ),
                    at: Vec::new(),
                });
            }
        }
        // Two-channel pairs.
        for decl in &io.decls {
            let Some(pair) = &decl.pair else { continue };
            if pair < &decl.name {
                continue; // report each pair once
            }
            let find = |name: &str| {
                b.points
                    .iter()
                    .find(|(k, _)| k.0 == name && k.1.is_none() && k.2 == IoDirection::Input)
                    .map(|(_, p)| p)
            };
            let (Some(a), Some(c)) = (find(&decl.name), find(pair)) else {
                continue; // unknown_ref covered it
            };
            let ba = match a.status {
                PointStatus::Bound(i) => io.bindings.get(i),
                _ => None,
            };
            let bc = match c.status {
                PointStatus::Bound(i) => io.bindings.get(i),
                _ => None,
            };
            let problem = match (ba, bc) {
                (None, None) => None,
                (Some(_), None) => Some(format!("`{}` is bound, `{pair}` is not", decl.name)),
                (None, Some(_)) => Some(format!("`{pair}` is bound, `{}` is not", decl.name)),
                (Some(x), Some(y)) => {
                    let kx = io
                        .node(&x.node)
                        .and_then(|n| n.channels.iter().find(|ch| ch.id == x.channel))
                        .map(|ch| ch.kind);
                    let ky = io
                        .node(&y.node)
                        .and_then(|n| n.channels.iter().find(|ch| ch.id == y.channel))
                        .map(|ch| ch.kind);
                    if x.node != y.node {
                        Some(format!(
                            "`{}` is on {} and `{pair}` on {} — one safety module",
                            decl.name, x.node, y.node
                        ))
                    } else if kx != ky {
                        Some(format!(
                            "`{}` and `{pair}` sit on channels of different kinds",
                            decl.name
                        ))
                    } else if x.invert != y.invert {
                        Some(format!(
                            "`{}` and `{pair}` are wired with different polarity",
                            decl.name
                        ))
                    } else {
                        None
                    }
                }
            };
            if let Some(problem) = problem {
                extra.push(IoFinding {
                    severity: Severity::Error,
                    code: IoCode::SafetyPair,
                    message: format!("safety pair {problem}"),
                    at: Vec::new(),
                });
            }
        }
        // Ownership across the set: an output point written by two programs.
        for p in b.points.values() {
            if p.id.direction != IoDirection::Output || matches!(p.source, IoSource::Cosmetic) {
                continue;
            }
            let mut programs: Vec<&str> = p.writers.iter().map(|w| w.sequence.as_str()).collect();
            programs.sort();
            programs.dedup();
            if programs.len() > 1 {
                extra.push(IoFinding {
                    severity: Severity::Error,
                    code: IoCode::MultipleDrivers,
                    message: format!(
                        "`{}` is driven by {} programs ({}) — every coil, device and robot belongs to one \
                         program (pass the program set you would simulate together)",
                        p.label(),
                        programs.len(),
                        programs.join(", ")
                    ),
                    at: p.writers.clone(),
                });
            }
        }
        // Capacity: per host and channel family, points needing a channel vs
        // channels in the host's pool (the host and the stations uplinked
        // to it), once assignment has begun.
        if !io.nodes.is_empty() {
            let family = |k: ChannelKind| match k {
                ChannelKind::Di | ChannelKind::SafeDi => "DI",
                ChannelKind::Do | ChannelKind::SafeDo => "DO",
                ChannelKind::Ai => "AI",
                ChannelKind::Ao => "AO",
                ChannelKind::Word => "Word",
            };
            let mut need: BTreeMap<(String, &str), usize> = BTreeMap::new();
            for p in b.points.values() {
                if !matches!(
                    p.status,
                    PointStatus::Bound(_) | PointStatus::Unbound | PointStatus::Constant
                ) {
                    continue;
                }
                let Some(host) = &p.host else { continue };
                if io.node(host).is_none() {
                    continue; // implicit hosts have no channels to run out of
                }
                *need.entry((host.clone(), family(p.kind))).or_insert(0) += 1;
            }
            for ((host, fam), n) in need {
                let pool: Vec<&IoNode> = io
                    .nodes
                    .iter()
                    .filter(|nd| io.reach(&nd.name).iter().any(|r| r == &host))
                    .collect();
                let have: usize = pool
                    .iter()
                    .flat_map(|nd| nd.channels.iter())
                    .filter(|c| family(c.kind) == fam)
                    .count();
                if n > have {
                    extra.push(IoFinding {
                        severity: Severity::Warning,
                        code: IoCode::Capacity,
                        message: format!(
                            "{host} needs {n} {fam} but has {have} ({}) — add a module or a station",
                            pool.iter().map(|nd| nd.name.as_str()).collect::<Vec<_>>().join(" + ")
                        ),
                        at: Vec::new(),
                    });
                }
            }
        }
        b.findings.extend(extra);
    }

    // ---- name clashes ------------------------------------------------------
    {
        let mut owners: BTreeMap<&str, Vec<&'static str>> = BTreeMap::new();
        for s in scene.signals() {
            owners.entry(&s.name).or_default().push("signal");
        }
        for s in scene.sensors() {
            owners.entry(&s.name).or_default().push("sensor");
        }
        for d in scene.devices() {
            owners.entry(&d.name).or_default().push("device");
        }
        for r in scene.robots() {
            owners.entry(&r.name).or_default().push("robot");
        }
        for (name, kinds) in owners {
            if kinds.len() > 1 {
                b.finding(
                    Severity::Warning,
                    IoCode::NameClash,
                    format!(
                        "`{name}` names a {} — signal lanes are one namespace, so the I/O point is \
                         ambiguous",
                        kinds.join(" and a ")
                    ),
                    Vec::new(),
                );
            }
        }
        for s in scene.signals() {
            if s.name.contains('.') {
                b.finding(
                    Severity::Warning,
                    IoCode::NameClash,
                    format!(
                        "signal `{}` contains `.` — it reads like a `name.aspect` label (device \
                         commands, robot handshakes) in the I/O table",
                        s.name
                    ),
                    Vec::new(),
                );
            }
        }
    }

    let mut points: Vec<IoPoint> = b.points.into_values().collect();
    for p in &mut points {
        dedup(&mut p.writers);
        dedup(&mut p.readers);
    }
    // Deterministic order: real I/O first (by name, aspect, direction,
    // host — the table reads grouped by device / robot / signal), the
    // magazines' cosmetic rows last so a line's hundred carriers do not
    // bury the points that matter.
    points.sort_by(|a, b| {
        let cosmetic = |p: &IoPoint| p.status == PointStatus::Cosmetic;
        (
            cosmetic(a),
            &a.id.name,
            a.id.aspect,
            a.id.direction,
            &a.host,
        )
            .cmp(&(
                cosmetic(b),
                &b.id.name,
                b.id.aspect,
                b.id.direction,
                &b.host,
            ))
    });
    Ok(IoDerivation {
        points,
        report: IoReport {
            findings: b.findings,
        },
        sequences: programs.iter().map(|s| s.name.clone()).collect(),
        program_hosts: usages
            .iter()
            .map(|u| (u.name.clone(), u.host.clone()))
            .collect(),
        io: io.clone(),
    })
}

/// `(inputs, outputs)` name → port maps.
pub type PortMaps = (BTreeMap<String, IoPort>, BTreeMap<String, IoPort>);

/// The name → port wiring a vendor-script lowering for `node` (a robot
/// controller) needs, projected from the bindings on that node and on the
/// nodes uplinked to it. Keys follow [`crate::script::SequenceIo`]:
/// signal / sensor / device names, and — for `robot_done` waits — robot
/// names. Points that have no script form (device numeric commands, the
/// robot's own start / program word) are left out. A bound channel with no
/// `port` is an error: the script needs a number.
pub fn sequence_io(d: &IoDerivation, io: &IoMap, node: &str) -> Result<PortMaps, String> {
    let mut inputs = BTreeMap::new();
    let mut outputs = BTreeMap::new();
    let downstream: Vec<&IoNode> = io
        .nodes
        .iter()
        .filter(|n| io.reach(&n.name).iter().any(|r| r == node))
        .collect();
    for n in downstream {
        for binding in io.bindings_on(&n.name) {
            let point = d
                .points
                .iter()
                .find(|p| p.id == binding.point && matches!(p.status, PointStatus::Bound(_)));
            if point.is_none() {
                continue;
            }
            let Some(channel) = n.channels.iter().find(|c| c.id == binding.channel) else {
                continue;
            };
            let key = match (binding.point.aspect, binding.point.direction) {
                (None, _) => binding.point.name.clone(),
                (Some(Aspect::Done), IoDirection::Input) => binding.point.name.clone(),
                _ => continue,
            };
            let port = channel.port.ok_or_else(|| {
                format!(
                    "`{}` is bound to {}.{}, which has no vendor port number — the script cannot \
                     name it (give the channel a port, or pass inputs= / outputs=)",
                    binding.point.label(),
                    n.name,
                    channel.id
                )
            })?;
            let value = IoPort {
                port,
                invert: binding.invert,
            };
            match binding.point.direction {
                IoDirection::Input => inputs.insert(key, value),
                IoDirection::Output => outputs.insert(key, value),
            };
        }
    }
    Ok((inputs, outputs))
}

// ------------------------------------------------------- auto-assign

impl Scene {
    /// Gives every unbound point a channel, deterministically: points in
    /// table order (name, aspect, direction, host), channels in the order
    /// their node declares them, on the point's host and the stations
    /// uplinked to it, first free channel of a compatible family (safety
    /// points prefer safety channels, standard points prefer standard
    /// ones). Existing bindings are kept; `reassign` first drops the ones
    /// an earlier run placed (hand bindings stay put and keep their
    /// channels). Points on an implicit host (`<cell>`, `<robot>`) cannot be
    /// placed — declare the node that runs their program. Returns the
    /// report after the assignment: what is still unbound is a `capacity`
    /// / `unbound` finding.
    pub fn auto_assign_io(
        &mut self,
        sequences: Option<&[&str]>,
        reassign: bool,
    ) -> Result<IoReport, IoError> {
        if reassign {
            self.io.bindings.retain(|b| !b.auto);
        }
        let d = derive(self, sequences)?;
        let mut used: BTreeSet<(String, String)> = self
            .io
            .bindings
            .iter()
            .map(|b| (b.node.clone(), b.channel.clone()))
            .collect();
        let mut fresh: Vec<IoBinding> = Vec::new();
        for p in &d.points {
            if !matches!(p.status, PointStatus::Unbound | PointStatus::Constant) {
                continue;
            }
            // The pool: the host node first, then every node whose uplink
            // chain reaches it, in declaration order; an unhosted point
            // takes the sole controller (and its stations), if there is
            // exactly one to speak of.
            let pool_under = |host: &str| -> Vec<String> {
                let mut pool: Vec<String> = self
                    .io
                    .nodes
                    .iter()
                    .filter(|n| self.io.reach(&n.name).iter().any(|r| r == host))
                    .map(|n| n.name.clone())
                    .collect();
                pool.sort_by_key(|n| n != host);
                pool
            };
            let pool: Vec<String> = match &p.host {
                Some(host) if self.io.node(host).is_some() => pool_under(host),
                Some(_) => continue, // implicit host: nothing to bind to
                None => {
                    let controllers: Vec<&IoNode> = self
                        .io
                        .nodes
                        .iter()
                        .filter(|n| {
                            n.uplink.is_none() && !matches!(n.kind, IoNodeKind::Other { .. })
                        })
                        .collect();
                    match controllers.as_slice() {
                        [sole] => pool_under(&sole.name.clone()),
                        _ => continue,
                    }
                }
            };
            let prefer_safe = p.safety;
            let mut chosen: Option<(String, String)> = None;
            for pass in 0..2 {
                for node_name in &pool {
                    let Some(node) = self.io.node(node_name) else {
                        continue;
                    };
                    if matches!(node.kind, IoNodeKind::Other { .. }) {
                        continue;
                    }
                    for c in &node.channels {
                        if !p.kind.compatible(c.kind) {
                            continue;
                        }
                        let safe = matches!(c.kind, ChannelKind::SafeDi | ChannelKind::SafeDo);
                        // Pass 0: the preferred family; pass 1: the other.
                        if (pass == 0) != (safe == prefer_safe) {
                            continue;
                        }
                        if used.contains(&(node_name.clone(), c.id.clone())) {
                            continue;
                        }
                        chosen = Some((node_name.clone(), c.id.clone()));
                        break;
                    }
                    if chosen.is_some() {
                        break;
                    }
                }
                if chosen.is_some() {
                    break;
                }
            }
            if let Some((node, channel)) = chosen {
                used.insert((node.clone(), channel.clone()));
                fresh.push(IoBinding {
                    point: p.id.clone(),
                    node,
                    channel,
                    tag: None,
                    field: None,
                    invert: false,
                    contact: None,
                    safety: p.safety,
                    device: None,
                    note: None,
                    auto: true,
                });
            }
        }
        for b in fresh {
            self.bind_io(b).map_err(|e| IoError::Invalid {
                sequence: String::new(),
                message: e.to_string(),
            })?;
        }
        Ok(derive(self, sequences)?.report)
    }
}

// ---------------------------------------------------------- topology

/// The cell's electrical topology as a graph — hosts and stations,
/// programs, field devices, and the wires between them — derived from the
/// same points the tables print. One graph, several layers (edge kinds):
/// the studio overlay and the DOT / Mermaid exports read the same thing.
#[derive(Debug, Clone, PartialEq)]
pub struct Topology {
    pub nodes: Vec<TopoNode>,
    pub edges: Vec<TopoEdge>,
}

#[derive(Debug, Clone, PartialEq)]
pub struct TopoNode {
    /// Stable id: `host:PLC1`, `prog:transfer`, `sensor:eye`, `device:belt`,
    /// `robot:arm`, `field:-B1`, `decl:door_ch1`.
    pub id: String,
    pub kind: TopoNodeKind,
    pub label: String,
    /// The controller the node belongs to (programs, stations), if any.
    pub host: Option<String>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TopoNodeKind {
    /// A controller: a declared node, or an implicit `<cell>` / `<robot>`.
    Host {
        implicit: bool,
        kind: String,
    },
    /// A remote station / safety module (declared node with an uplink).
    Station {
        kind: String,
    },
    Program,
    Sensor,
    Device,
    Robot,
    /// A field device named only by a binding's `field`, or a signal's far
    /// end when nothing names it.
    Field,
    Declared,
}

impl TopoNodeKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            TopoNodeKind::Host { .. } => "host",
            TopoNodeKind::Station { .. } => "station",
            TopoNodeKind::Program => "program",
            TopoNodeKind::Sensor => "sensor",
            TopoNodeKind::Device => "device",
            TopoNodeKind::Robot => "robot",
            TopoNodeKind::Field => "field",
            TopoNodeKind::Declared => "declared",
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct TopoEdge {
    pub from: String,
    pub to: String,
    pub kind: TopoEdgeKind,
    pub label: String,
    /// The signal lane behind the edge, for live colouring — a sensor,
    /// signal or device name; none for robot handshakes.
    pub lane: Option<String>,
    pub safety: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub enum TopoEdgeKind {
    /// Station → controller.
    Uplink { bus: Option<String> },
    /// A point's wire between a field device and the node it is bound to
    /// (or its host, unbound).
    Io {
        point: IoPointId,
        node: Option<String>,
        channel: Option<String>,
        address: Option<String>,
    },
    /// A signal crossing controllers: writer host → reader host.
    Handshake { signal: String },
    /// A signal between programs (writer → reader), whatever the hosts.
    Functional { signal: String },
}

impl TopoEdgeKind {
    pub fn as_str(&self) -> &'static str {
        match self {
            TopoEdgeKind::Uplink { .. } => "uplink",
            TopoEdgeKind::Io { .. } => "io",
            TopoEdgeKind::Handshake { .. } => "handshake",
            TopoEdgeKind::Functional { .. } => "functional",
        }
    }
}

/// The layers a rendering can filter to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopoLayer {
    Functional,
    Io,
    Network,
    Wiring,
    Safety,
}

impl TopoLayer {
    pub fn parse(s: &str) -> Option<TopoLayer> {
        Some(match s.to_ascii_lowercase().as_str() {
            "functional" => TopoLayer::Functional,
            "io" | "i/o" => TopoLayer::Io,
            "network" => TopoLayer::Network,
            "wiring" => TopoLayer::Wiring,
            "safety" => TopoLayer::Safety,
            _ => return None,
        })
    }

    pub fn as_str(self) -> &'static str {
        match self {
            TopoLayer::Functional => "functional",
            TopoLayer::Io => "io",
            TopoLayer::Network => "network",
            TopoLayer::Wiring => "wiring",
            TopoLayer::Safety => "safety",
        }
    }

    /// Does this layer show `edge`?
    pub fn shows(self, edge: &TopoEdge) -> bool {
        match self {
            TopoLayer::Functional => matches!(edge.kind, TopoEdgeKind::Functional { .. }),
            TopoLayer::Io => matches!(edge.kind, TopoEdgeKind::Io { .. }),
            TopoLayer::Network => matches!(edge.kind, TopoEdgeKind::Uplink { .. }),
            TopoLayer::Wiring => matches!(
                edge.kind,
                TopoEdgeKind::Io { .. }
                    | TopoEdgeKind::Handshake { .. }
                    | TopoEdgeKind::Uplink { .. }
            ),
            TopoLayer::Safety => edge.safety,
        }
    }
}

/// Builds the topology from a derivation. Cosmetic rows (magazines) are
/// left out unless `include_cosmetic`.
pub fn topology(scene: &Scene, d: &IoDerivation, include_cosmetic: bool) -> Topology {
    let io = &d.io;
    let mut nodes: BTreeMap<String, TopoNode> = BTreeMap::new();
    let mut edges: Vec<TopoEdge> = Vec::new();
    let add_node = |nodes: &mut BTreeMap<String, TopoNode>,
                    id: String,
                    kind: TopoNodeKind,
                    label: String,
                    host: Option<String>| {
        nodes.entry(id.clone()).or_insert(TopoNode {
            id,
            kind,
            label,
            host,
        });
    };
    let host_id = |h: &str| format!("host:{h}");
    let host_kind = |h: &str| -> TopoNodeKind {
        match io.node(h) {
            Some(n) if n.uplink.is_some() => TopoNodeKind::Station {
                kind: n.kind.as_str().to_string(),
            },
            Some(n) => TopoNodeKind::Host {
                implicit: false,
                kind: n.kind.as_str().to_string(),
            },
            None => TopoNodeKind::Host {
                implicit: true,
                kind: "implicit".to_string(),
            },
        }
    };
    let host_label = |h: &str| -> String {
        match io.node(h) {
            Some(n) => match &n.kind {
                IoNodeKind::RobotController { robots } => {
                    format!("{h} (robot controller: {})", robots.join(", "))
                }
                k => format!("{h} ({})", k.as_str().replace('_', " ")),
            },
            None => h.to_string(),
        }
    };
    // Declared nodes and their uplinks.
    for n in &io.nodes {
        add_node(
            &mut nodes,
            host_id(&n.name),
            host_kind(&n.name),
            host_label(&n.name),
            n.uplink.as_ref().map(|u| u.parent.clone()),
        );
        if let Some(u) = &n.uplink {
            add_node(
                &mut nodes,
                host_id(&u.parent),
                host_kind(&u.parent),
                host_label(&u.parent),
                None,
            );
            edges.push(TopoEdge {
                from: host_id(&n.name),
                to: host_id(&u.parent),
                kind: TopoEdgeKind::Uplink { bus: u.bus.clone() },
                label: u.bus.clone().unwrap_or_default(),
                lane: None,
                safety: false,
            });
        }
    }
    // Programs on their hosts.
    for (program, host) in &d.program_hosts {
        add_node(
            &mut nodes,
            host_id(host),
            host_kind(host),
            host_label(host),
            None,
        );
        add_node(
            &mut nodes,
            format!("prog:{program}"),
            TopoNodeKind::Program,
            program.clone(),
            Some(host.clone()),
        );
    }
    // Field side of every point, and its wire.
    let is_sensor = |n: &str| scene.sensors().iter().any(|s| s.name == n);
    let is_device = |n: &str| scene.devices().iter().any(|dv| dv.name == n);
    let is_robot = |n: &str| scene.robot_index(n).is_some();
    for p in &d.points {
        if p.status == PointStatus::Cosmetic && !include_cosmetic {
            continue;
        }
        if matches!(p.source, IoSource::Internal) {
            continue; // functional layer only (below)
        }
        let bound = d.binding_of(p);
        // The far end.
        let (far_id, far_kind, far_label) = if is_sensor(&p.id.name) {
            (
                format!("sensor:{}", p.id.name),
                TopoNodeKind::Sensor,
                p.id.name.clone(),
            )
        } else if is_device(&p.id.name) {
            (
                format!("device:{}", p.id.name),
                TopoNodeKind::Device,
                p.id.name.clone(),
            )
        } else if is_robot(&p.id.name) {
            (
                format!("robot:{}", p.id.name),
                TopoNodeKind::Robot,
                p.id.name.clone(),
            )
        } else if matches!(p.source, IoSource::Handshake) {
            // host ↔ host: no far-end node, the edge is drawn below.
            (String::new(), TopoNodeKind::Field, String::new())
        } else if let Some(field) = bound.and_then(|(b, _, _)| b.field.clone()) {
            (format!("field:{field}"), TopoNodeKind::Field, field)
        } else if matches!(p.source, IoSource::Declared) {
            (
                format!("decl:{}", p.id.name),
                TopoNodeKind::Declared,
                p.id.name.clone(),
            )
        } else {
            (
                format!("field:{}", p.id.name),
                TopoNodeKind::Field,
                p.id.name.clone(),
            )
        };
        if matches!(p.source, IoSource::Handshake) {
            if p.id.direction == IoDirection::Output {
                let Some(from_host) = &p.host else { continue };
                add_node(
                    &mut nodes,
                    host_id(from_host),
                    host_kind(from_host),
                    host_label(from_host),
                    None,
                );
                for q in &d.points {
                    if q.id.name == p.id.name
                        && q.id.direction == IoDirection::Input
                        && matches!(q.source, IoSource::Handshake)
                    {
                        let Some(to_host) = &q.host else { continue };
                        add_node(
                            &mut nodes,
                            host_id(to_host),
                            host_kind(to_host),
                            host_label(to_host),
                            None,
                        );
                        edges.push(TopoEdge {
                            from: host_id(from_host),
                            to: host_id(to_host),
                            kind: TopoEdgeKind::Handshake {
                                signal: p.id.name.clone(),
                            },
                            label: p.id.name.clone(),
                            lane: Some(p.id.name.clone()),
                            safety: p.safety || q.safety,
                        });
                    }
                }
            }
            continue;
        }
        add_node(&mut nodes, far_id.clone(), far_kind, far_label, None);
        // The near end: the binding's node, else the host, else nothing
        // (an unhosted, unbound point dangles as a node on its own).
        let near = match (bound, &p.host) {
            (Some((_, n, _)), _) => Some(n.name.clone()),
            (None, Some(h)) => Some(h.clone()),
            (None, None) => None,
        };
        let Some(near) = near else { continue };
        add_node(
            &mut nodes,
            host_id(&near),
            host_kind(&near),
            host_label(&near),
            None,
        );
        let mut label = p.label();
        if let Some((_, n, c)) = bound {
            label.push_str(&format!(" → {}.{}", n.name, c.id));
            if let Some(a) = &c.address {
                label.push_str(&format!(" [{a}]"));
            }
        }
        let lane = if p.id.aspect.is_none() && !matches!(p.source, IoSource::Declared) {
            Some(p.id.name.clone())
        } else {
            None
        };
        let (from, to) = match p.id.direction {
            IoDirection::Input => (far_id.clone(), host_id(&near)),
            IoDirection::Output => (host_id(&near), far_id.clone()),
        };
        edges.push(TopoEdge {
            from,
            to,
            kind: TopoEdgeKind::Io {
                point: p.id.clone(),
                node: bound.map(|(_, n, _)| n.name.clone()),
                channel: bound.map(|(_, _, c)| c.id.clone()),
                address: bound.and_then(|(_, _, c)| c.address.clone()),
            },
            label,
            lane,
            safety: p.safety,
        });
    }
    // Functional: signals between programs (writer → reader), from the
    // step attribution of internal and handshake points.
    for p in &d.points {
        if !matches!(p.source, IoSource::Internal | IoSource::Handshake)
            || p.id.direction != IoDirection::Output
        {
            continue;
        }
        let mut writers: Vec<&str> = p.writers.iter().map(|w| w.sequence.as_str()).collect();
        writers.sort();
        writers.dedup();
        let mut readers: Vec<&str> = p.readers.iter().map(|r| r.sequence.as_str()).collect();
        // Handshake outputs list only their own readers; the inputs on
        // other hosts hold the rest.
        for q in &d.points {
            if q.id.name == p.id.name
                && q.id.direction == IoDirection::Input
                && matches!(q.source, IoSource::Handshake)
            {
                readers.extend(q.readers.iter().map(|r| r.sequence.as_str()));
            }
        }
        readers.sort();
        readers.dedup();
        for w in &writers {
            for r in &readers {
                if w == r {
                    continue;
                }
                let (fid, rid) = (format!("prog:{w}"), format!("prog:{r}"));
                if !nodes.contains_key(&fid) || !nodes.contains_key(&rid) {
                    continue;
                }
                edges.push(TopoEdge {
                    from: fid,
                    to: rid,
                    kind: TopoEdgeKind::Functional {
                        signal: p.id.name.clone(),
                    },
                    label: p.id.name.clone(),
                    lane: Some(p.id.name.clone()),
                    safety: p.safety,
                });
            }
        }
    }
    Topology {
        nodes: nodes.into_values().collect(),
        edges,
    }
}

fn topo_visible<'a>(
    t: &'a Topology,
    layers: &[TopoLayer],
) -> (Vec<&'a TopoNode>, Vec<&'a TopoEdge>) {
    let edges: Vec<&TopoEdge> = t
        .edges
        .iter()
        .filter(|e| layers.is_empty() || layers.iter().any(|l| l.shows(e)))
        .collect();
    let mut keep: BTreeSet<&str> = BTreeSet::new();
    for e in &edges {
        keep.insert(&e.from);
        keep.insert(&e.to);
    }
    // Hosts and stations always show (a controller with nothing wired is
    // still part of the picture); everything else only when an edge
    // touches it. Programs show on the functional layer or when unfiltered.
    let show_programs = layers.is_empty() || layers.contains(&TopoLayer::Functional);
    let nodes: Vec<&TopoNode> = t
        .nodes
        .iter()
        .filter(|n| match n.kind {
            TopoNodeKind::Host { .. } | TopoNodeKind::Station { .. } => true,
            TopoNodeKind::Program => show_programs,
            _ => keep.contains(n.id.as_str()),
        })
        .collect();
    (nodes, edges)
}

fn dot_id(id: &str) -> String {
    format!("\"{}\"", id.replace('"', "'"))
}

/// Graphviz DOT: hosts as clusters holding their programs, stations
/// clustered under their controller's rank, field devices outside, edges
/// labelled with the point and its channel.
pub fn render_dot(t: &Topology, layers: &[TopoLayer]) -> String {
    let (nodes, edges) = topo_visible(t, layers);
    let mut out = String::from("digraph io_map {\n  rankdir=LR;\n  node [fontname=\"Helvetica\" fontsize=10];\n  edge [fontname=\"Helvetica\" fontsize=9];\n");
    let mut by_host: BTreeMap<String, Vec<&TopoNode>> = BTreeMap::new();
    for n in &nodes {
        match &n.kind {
            TopoNodeKind::Host { .. } | TopoNodeKind::Station { .. } => {}
            TopoNodeKind::Program => {
                if let Some(h) = &n.host {
                    by_host.entry(h.clone()).or_default().push(n);
                }
            }
            _ => {}
        }
    }
    for n in &nodes {
        match &n.kind {
            TopoNodeKind::Host { implicit, .. } => {
                let style = if *implicit { "dashed" } else { "solid" };
                let host = n.id.trim_start_matches("host:");
                out.push_str(&format!(
                    "  subgraph \"cluster_{host}\" {{\n    label={:?}; style={style};\n",
                    n.label
                ));
                out.push_str(&format!(
                    "    {} [shape=box3d label={:?}];\n",
                    dot_id(&n.id),
                    n.label
                ));
                for p in by_host.get(host).into_iter().flatten() {
                    out.push_str(&format!(
                        "    {} [shape=box label={:?}];\n",
                        dot_id(&p.id),
                        p.label
                    ));
                }
                out.push_str("  }\n");
            }
            TopoNodeKind::Station { .. } => {
                out.push_str(&format!(
                    "  {} [shape=component label={:?}];\n",
                    dot_id(&n.id),
                    n.label
                ));
            }
            TopoNodeKind::Program => {}
            TopoNodeKind::Sensor => out.push_str(&format!(
                "  {} [shape=ellipse label={:?}];\n",
                dot_id(&n.id),
                n.label
            )),
            TopoNodeKind::Device => out.push_str(&format!(
                "  {} [shape=box style=rounded label={:?}];\n",
                dot_id(&n.id),
                n.label
            )),
            TopoNodeKind::Robot => out.push_str(&format!(
                "  {} [shape=hexagon label={:?}];\n",
                dot_id(&n.id),
                n.label
            )),
            TopoNodeKind::Field | TopoNodeKind::Declared => out.push_str(&format!(
                "  {} [shape=plaintext label={:?}];\n",
                dot_id(&n.id),
                n.label
            )),
        }
    }
    for e in &edges {
        let mut attrs = vec![format!("label={:?}", e.label)];
        match &e.kind {
            TopoEdgeKind::Uplink { .. } => attrs.push("style=bold arrowhead=none".into()),
            TopoEdgeKind::Handshake { .. } => attrs.push("penwidth=2".into()),
            TopoEdgeKind::Functional { .. } => attrs.push("style=dotted".into()),
            TopoEdgeKind::Io { node: None, .. } => attrs.push("style=dashed color=gray50".into()),
            TopoEdgeKind::Io { .. } => {}
        }
        if e.safety {
            attrs.push("color=orange3".into());
        }
        out.push_str(&format!(
            "  {} -> {} [{}];\n",
            dot_id(&e.from),
            dot_id(&e.to),
            attrs.join(" ")
        ));
    }
    out.push_str("}\n");
    out
}

fn mermaid_id(id: &str) -> String {
    let mut out = String::with_capacity(id.len());
    for c in id.chars() {
        if c.is_ascii_alphanumeric() {
            out.push(c);
        } else {
            out.push('_');
        }
    }
    out
}

fn mermaid_label(s: &str) -> String {
    format!("\"{}\"", s.replace('"', "'"))
}

/// Mermaid `flowchart LR`: hosts as subgraphs holding their programs,
/// field devices outside, edges labelled with the point and its channel.
/// Pastes into Markdown (mkdocs, GitHub).
pub fn render_mermaid(t: &Topology, layers: &[TopoLayer]) -> String {
    let (nodes, edges) = topo_visible(t, layers);
    let mut out = String::from("flowchart LR\n");
    let mut by_host: BTreeMap<String, Vec<&TopoNode>> = BTreeMap::new();
    for n in &nodes {
        if let (TopoNodeKind::Program, Some(h)) = (&n.kind, &n.host) {
            by_host.entry(h.clone()).or_default().push(n);
        }
    }
    for n in &nodes {
        match &n.kind {
            TopoNodeKind::Host { .. } => {
                let host = n.id.trim_start_matches("host:");
                out.push_str(&format!(
                    "  subgraph sg_{}[{}]\n",
                    mermaid_id(&n.id),
                    mermaid_label(&n.label)
                ));
                out.push_str(&format!(
                    "    {}[{}]\n",
                    mermaid_id(&n.id),
                    mermaid_label(host)
                ));
                for p in by_host.get(host).into_iter().flatten() {
                    out.push_str(&format!(
                        "    {}[{}]\n",
                        mermaid_id(&p.id),
                        mermaid_label(&p.label)
                    ));
                }
                out.push_str("  end\n");
            }
            TopoNodeKind::Station { .. } => out.push_str(&format!(
                "  {}[[{}]]\n",
                mermaid_id(&n.id),
                mermaid_label(&n.label)
            )),
            TopoNodeKind::Program => {}
            TopoNodeKind::Sensor => out.push_str(&format!(
                "  {}(({}))\n",
                mermaid_id(&n.id),
                mermaid_label(&n.label)
            )),
            TopoNodeKind::Device => out.push_str(&format!(
                "  {}({})\n",
                mermaid_id(&n.id),
                mermaid_label(&n.label)
            )),
            TopoNodeKind::Robot => out.push_str(&format!(
                "  {}{{{{{}}}}}\n",
                mermaid_id(&n.id),
                mermaid_label(&n.label)
            )),
            TopoNodeKind::Field | TopoNodeKind::Declared => out.push_str(&format!(
                "  {}>{}]\n",
                mermaid_id(&n.id),
                mermaid_label(&n.label)
            )),
        }
    }
    for e in &edges {
        // `A -->|"label"| B`; open thin links for uplinks, thick for
        // handshakes, dotted for functional and for unbound wires. The
        // label is quoted: brackets and parentheses (`[%IX0.0]`) break the
        // bare pipe form.
        let arrow = match &e.kind {
            TopoEdgeKind::Uplink { .. } => "---",
            TopoEdgeKind::Handshake { .. } => "==>",
            TopoEdgeKind::Functional { .. } => "-.->",
            TopoEdgeKind::Io { node: None, .. } => "-.->",
            TopoEdgeKind::Io { .. } => "-->",
        };
        let label = if e.label.is_empty() {
            String::new()
        } else {
            format!("|{}|", mermaid_label(&e.label.replace('|', "/")))
        };
        out.push_str(&format!(
            "  {} {}{} {}\n",
            mermaid_id(&e.from),
            arrow,
            label,
            mermaid_id(&e.to)
        ));
    }
    out
}

/// JSON: `{ "nodes": [...], "edges": [...] }` with the raw fields.
pub fn render_topology_json(t: &Topology, layers: &[TopoLayer]) -> String {
    let (nodes, edges) = topo_visible(t, layers);
    let nodes: Vec<String> = nodes
        .iter()
        .map(|n| {
            format!(
                "{{\"id\":{},\"kind\":{},\"label\":{},\"host\":{}}}",
                json_str(&n.id),
                json_str(n.kind.as_str()),
                json_str(&n.label),
                n.host
                    .as_deref()
                    .map(json_str)
                    .unwrap_or_else(|| "null".to_string())
            )
        })
        .collect();
    let edges: Vec<String> = edges
        .iter()
        .map(|e| {
            format!(
                "{{\"from\":{},\"to\":{},\"kind\":{},\"label\":{},\"lane\":{},\"safety\":{}}}",
                json_str(&e.from),
                json_str(&e.to),
                json_str(e.kind.as_str()),
                json_str(&e.label),
                e.lane
                    .as_deref()
                    .map(json_str)
                    .unwrap_or_else(|| "null".to_string()),
                e.safety
            )
        })
        .collect();
    format!(
        "{{\"nodes\":[{}],\"edges\":[{}]}}\n",
        nodes.join(","),
        edges.join(",")
    )
}

// ------------------------------------------------------------- tables

/// The columns of the I/O list (design §7). Assignment columns
/// (node / channel / address / tag / field / contact / invert / model /
/// location / note) are present from the start so the format does not
/// move when the assignment layer fills them.
pub const IO_COLUMNS: [&str; 20] = [
    "name",
    "aspect",
    "direction",
    "kind",
    "source",
    "host",
    "node",
    "channel",
    "address",
    "tag",
    "field",
    "contact",
    "invert",
    "safety",
    "model",
    "location",
    "writers",
    "readers",
    "status",
    "note",
];

/// One table row, every column a string. Assignment columns are filled
/// from the binding behind a bound point.
pub fn io_row(d: &IoDerivation, p: &IoPoint) -> Vec<String> {
    let steps = |v: &[StepRef]| {
        v.iter()
            .map(|s| s.to_string())
            .collect::<Vec<_>>()
            .join("; ")
    };
    let bound = d.binding_of(p);
    let opt = |v: Option<&String>| v.cloned().unwrap_or_default();
    let yes = |b: bool| if b { "yes".to_string() } else { String::new() };
    vec![
        p.id.name.clone(),
        p.id.aspect
            .map(|a| a.as_str().to_string())
            .unwrap_or_default(),
        p.id.direction.as_str().to_string(),
        p.kind.as_str().to_string(),
        p.source.as_str().to_string(),
        p.host.clone().unwrap_or_default(),
        bound.map(|(_, n, _)| n.name.clone()).unwrap_or_default(),
        bound.map(|(_, _, c)| c.id.clone()).unwrap_or_default(),
        bound
            .map(|(_, _, c)| opt(c.address.as_ref()))
            .unwrap_or_default(),
        bound
            .map(|(b, _, _)| opt(b.tag.as_ref()))
            .unwrap_or_default(),
        bound
            .map(|(b, _, _)| opt(b.field.as_ref()))
            .unwrap_or_default(),
        bound
            .and_then(|(b, _, _)| b.contact.map(|c| c.as_str().to_string()))
            .unwrap_or_default(),
        bound.map(|(b, _, _)| yes(b.invert)).unwrap_or_default(),
        yes(p.safety),
        bound
            .map(|(_, n, _)| opt(n.model.as_ref()))
            .unwrap_or_default(),
        bound
            .map(|(_, n, _)| opt(n.place.as_ref()))
            .unwrap_or_default(),
        steps(&p.writers),
        steps(&p.readers),
        p.status.as_str().to_string(),
        // The note column: the binding's note, or `auto` for a channel
        // `auto_assign_io` picked (so a reviewer can tell placed from chosen).
        bound
            .map(|(b, _, _)| {
                b.note.clone().unwrap_or_else(|| {
                    if b.auto {
                        "auto".to_string()
                    } else {
                        String::new()
                    }
                })
            })
            .unwrap_or_default(),
    ]
}

/// Point counts per host and channel kind — internal / cosmetic rows are
/// not I/O and stay out of the tally.
pub fn io_summary(points: &[IoPoint]) -> BTreeMap<String, BTreeMap<&'static str, usize>> {
    let mut out: BTreeMap<String, BTreeMap<&'static str, usize>> = BTreeMap::new();
    for p in points {
        if matches!(p.status, PointStatus::Internal | PointStatus::Cosmetic) {
            continue;
        }
        let host = p.host.clone().unwrap_or_else(|| "(unhosted)".to_string());
        *out.entry(host)
            .or_default()
            .entry(p.kind.as_str())
            .or_insert(0) += 1;
    }
    out
}

fn csv_cell(s: &str) -> String {
    if s.contains(',') || s.contains('"') || s.contains('\n') {
        format!("\"{}\"", s.replace('"', "\"\""))
    } else {
        s.to_string()
    }
}

/// CSV: header, one row per point, then `#` comment lines with the
/// per-host counts.
pub fn render_csv(d: &IoDerivation) -> String {
    let points = &d.points;
    let mut out = String::new();
    out.push_str(&IO_COLUMNS.join(","));
    out.push('\n');
    for p in points {
        let row: Vec<String> = io_row(d, p).iter().map(|c| csv_cell(c)).collect();
        out.push_str(&row.join(","));
        out.push('\n');
    }
    for (host, kinds) in io_summary(points) {
        let parts: Vec<String> = kinds.iter().map(|(k, n)| format!("{k} {n}")).collect();
        out.push_str(&format!("# {host}: {}\n", parts.join(", ")));
    }
    out
}

/// Markdown: a table plus a per-host count line.
pub fn render_markdown(d: &IoDerivation) -> String {
    let points = &d.points;
    let mut out = String::new();
    out.push_str("| ");
    out.push_str(&IO_COLUMNS.join(" | "));
    out.push_str(" |\n|");
    out.push_str(&"---|".repeat(IO_COLUMNS.len()));
    out.push('\n');
    for p in points {
        let row: Vec<String> = io_row(d, p).iter().map(|c| c.replace('|', "\\|")).collect();
        out.push_str("| ");
        out.push_str(&row.join(" | "));
        out.push_str(" |\n");
    }
    let summary = io_summary(points);
    if !summary.is_empty() {
        out.push('\n');
        for (host, kinds) in summary {
            let parts: Vec<String> = kinds.iter().map(|(k, n)| format!("{k} {n}")).collect();
            out.push_str(&format!("- `{host}`: {}\n", parts.join(", ")));
        }
    }
    out
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

/// JSON: `{ "sequences": [...], "points": [...], "summary": {...},
/// "findings": [...] }` with the raw fields (aspects and step indices
/// included) — the form other formats are derived from.
pub fn render_json(d: &IoDerivation) -> String {
    let steps = |v: &[StepRef]| {
        let items: Vec<String> = v
            .iter()
            .map(|s| {
                format!(
                    "{{\"sequence\":{},\"index\":{},\"name\":{}}}",
                    json_str(&s.sequence),
                    s.index,
                    json_str(&s.name)
                )
            })
            .collect();
        format!("[{}]", items.join(","))
    };
    let opt = |s: &Option<String>| {
        s.as_deref()
            .map(json_str)
            .unwrap_or_else(|| "null".to_string())
    };
    let points: Vec<String> = d
        .points
        .iter()
        .map(|p| {
            let bound = d.binding_of(p);
            let opt_s = |v: Option<String>| {
                v.map(|s| json_str(&s))
                    .unwrap_or_else(|| "null".to_string())
            };
            format!(
                "{{\"name\":{},\"aspect\":{},\"direction\":{},\"kind\":{},\"source\":{},\"host\":{},\
                 \"safety\":{},\"writers\":{},\"readers\":{},\"status\":{},\"node\":{},\"channel\":{},\
                 \"address\":{},\"tag\":{},\"field\":{}}}",
                json_str(&p.id.name),
                p.id.aspect
                    .map(|a| json_str(a.as_str()))
                    .unwrap_or_else(|| "null".to_string()),
                json_str(p.id.direction.as_str()),
                json_str(p.kind.as_str()),
                json_str(p.source.as_str()),
                opt(&p.host),
                p.safety,
                steps(&p.writers),
                steps(&p.readers),
                json_str(p.status.as_str()),
                opt_s(bound.map(|(_, n, _)| n.name.clone())),
                opt_s(bound.map(|(_, _, c)| c.id.clone())),
                opt_s(bound.and_then(|(_, _, c)| c.address.clone())),
                opt_s(bound.and_then(|(b, _, _)| b.tag.clone())),
                opt_s(bound.and_then(|(b, _, _)| b.field.clone())),
            )
        })
        .collect();
    let summary: Vec<String> = io_summary(&d.points)
        .iter()
        .map(|(host, kinds)| {
            let inner: Vec<String> = kinds
                .iter()
                .map(|(k, n)| format!("{}:{n}", json_str(k)))
                .collect();
            format!("{}:{{{}}}", json_str(host), inner.join(","))
        })
        .collect();
    let findings: Vec<String> = d
        .report
        .findings
        .iter()
        .map(|f| {
            format!(
                "{{\"severity\":{},\"code\":{},\"message\":{},\"at\":{}}}",
                json_str(f.severity.as_str()),
                json_str(f.code.as_str()),
                json_str(&f.message),
                steps(&f.at)
            )
        })
        .collect();
    let sequences: Vec<String> = d.sequences.iter().map(|s| json_str(s)).collect();
    format!(
        "{{\"sequences\":[{}],\"points\":[{}],\"summary\":{{{}}},\"findings\":[{}]}}\n",
        sequences.join(","),
        points.join(","),
        summary.join(","),
        findings.join(",")
    )
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::rollout::tests::{joint_motion, sample_scene};
    use crate::seq::{Device, DeviceKind, Sensor, SensorKind, SensorWatch, Step};
    use nalgebra::{Isometry3, Vector3};

    fn step(name: &str, actions: Vec<Action>, transition: Condition) -> Step {
        Step {
            name: name.to_string(),
            actions,
            transition,
            select: Vec::new(),
        }
    }

    fn zone(name: &str) -> Sensor {
        Sensor {
            name: name.to_string(),
            kind: SensorKind::Zone {
                pose: Isometry3::translation(0.0, 0.0, 0.55),
                size: Vector3::new(2.0, 2.0, 0.4),
            },
            watch: SensorWatch::AllObjects,
            mount: None,
        }
    }

    fn conveyor(name: &str, running: bool) -> Device {
        Device {
            name: name.to_string(),
            kind: DeviceKind::Conveyor {
                zone_pose: Isometry3::identity(),
                zone_size: Vector3::new(1.0, 1.0, 1.0),
                velocity: Vector3::new(0.1, 0.0, 0.0),
                running,
            },
        }
    }

    fn find<'a>(points: &'a [IoPoint], label: &str, direction: IoDirection) -> Vec<&'a IoPoint> {
        points
            .iter()
            .filter(|p| p.label() == label && p.id.direction == direction)
            .collect()
    }

    fn one<'a>(points: &'a [IoPoint], label: &str, direction: IoDirection) -> &'a IoPoint {
        let found = find(points, label, direction);
        assert_eq!(
            found.len(),
            1,
            "expected one `{label}` {direction:?}, got {found:?}"
        );
        found[0]
    }

    /// The pick cell of `examples/export_urscript.py`, in miniature: a
    /// beam, a spec-gauge contact, a belt coil, a vacuum coil, one robot.
    pub(crate) fn pick_cell() -> Scene {
        let mut scene = sample_scene();
        joint_motion(&mut scene, "to_pick", 0.5);
        joint_motion(&mut scene, "place", -0.5);
        scene.upsert_sensor(zone("part_at_pick"));
        scene.define_signal("spec_ok", true);
        scene.define_signal("vacuum", false);
        scene.upsert_device(conveyor("conv", false));
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "feed",
                    vec![
                        Action::Device {
                            device: "conv".into(),
                            command: DeviceCommand::Start,
                        },
                        Action::StartMotion {
                            motion: "to_pick".into(),
                        },
                    ],
                    Condition::Done,
                ),
                step(
                    "await part",
                    vec![],
                    Condition::Rising {
                        name: "part_at_pick".into(),
                    },
                ),
                step(
                    "halt",
                    vec![Action::Device {
                        device: "conv".into(),
                        command: DeviceCommand::Stop,
                    }],
                    Condition::Immediately,
                ),
                step(
                    "grip",
                    vec![Action::Set {
                        signal: "vacuum".into(),
                        value: true,
                    }],
                    Condition::Signal {
                        name: "spec_ok".into(),
                        value: true,
                    },
                ),
                step(
                    "place",
                    vec![Action::StartMotion {
                        motion: "place".into(),
                    }],
                    Condition::Done,
                ),
            ],
        });
        scene
    }

    #[test]
    fn pick_cell_derives_the_drawing_io_list() {
        let scene = pick_cell();
        let d = derive(&scene, None).unwrap();
        // One robot driven → the program lives on that robot's controller;
        // no handshake points, no <cell>.
        let host = robot_host("r");
        let beam = one(&d.points, "part_at_pick", IoDirection::Input);
        assert_eq!(
            (beam.kind, beam.source, beam.host.as_deref()),
            (ChannelKind::Di, IoSource::Sensor, Some(host.as_str()))
        );
        assert_eq!(beam.readers.len(), 1);
        assert_eq!(beam.readers[0].name, "await part");
        assert_eq!(beam.readers[0].index, 1);
        let spec = one(&d.points, "spec_ok", IoDirection::Input);
        assert_eq!(
            (spec.kind, spec.source),
            (ChannelKind::Di, IoSource::ReadOnly)
        );
        let conv = one(&d.points, "conv", IoDirection::Output);
        assert_eq!(
            (conv.kind, conv.source),
            (ChannelKind::Do, IoSource::DeviceRun)
        );
        assert_eq!(
            conv.writers
                .iter()
                .map(|s| s.name.as_str())
                .collect::<Vec<_>>(),
            ["feed", "halt"]
        );
        let vacuum = one(&d.points, "vacuum", IoDirection::Output);
        assert_eq!(
            (vacuum.kind, vacuum.source),
            (ChannelKind::Do, IoSource::WriteOnly)
        );
        assert_eq!(d.points.len(), 4);
        assert!(d.report.findings.is_empty(), "{}", d.report);
        assert_eq!(d.sequences, ["pick"]);
        // The lists are stable across calls.
        let again = derive(&scene, None).unwrap();
        assert_eq!(render_json(&again), render_json(&d));
    }

    #[test]
    fn unused_definitions_are_reported_not_listed() {
        let mut scene = pick_cell();
        scene.define_signal("spare", false);
        scene.upsert_sensor(zone("lonely"));
        scene.upsert_device(conveyor("idle", false));
        scene.upsert_device(conveyor("always_on", true));
        let d = derive(&scene, None).unwrap();
        // A sensor exists physically: it is a point, host unknown.
        let lonely = one(&d.points, "lonely", IoDirection::Input);
        assert_eq!(lonely.host, None);
        // A never-commanded running belt is a constant coil; a stopped
        // one that nobody touches is unreferenced.
        let on = one(&d.points, "always_on", IoDirection::Output);
        assert_eq!(on.status, PointStatus::Constant);
        assert!(find(&d.points, "idle", IoDirection::Output).is_empty());
        assert!(find(&d.points, "spare", IoDirection::Output).is_empty());
        let unreferenced: Vec<&str> = d
            .report
            .infos()
            .iter()
            .filter(|f| f.code == IoCode::Unreferenced)
            .map(|f| f.message.as_str())
            .collect();
        assert_eq!(unreferenced.len(), 2, "{unreferenced:?}");
        assert!(unreferenced.iter().any(|m| m.contains("`spare`")));
        assert!(unreferenced.iter().any(|m| m.contains("`idle`")));
    }

    /// Two arms in one program: the program lands on `<cell>`, each arm
    /// gets start/done points there, and a signal the belt program reads
    /// becomes a handshake wire between the two hosts.
    pub(crate) fn two_arm_cell() -> Scene {
        let mut scene = sample_scene();
        let model = scene.robot().clone();
        scene.add_robot(model, Some("far"), Isometry3::translation(1.0, 0.0, 0.0));
        // Motions for both robots (index 0 = "r", index 1 = "far").
        joint_motion(&mut scene, "r_go", 0.5);
        scene
            .add_segment_for(
                1,
                "far_go",
                crate::motion::Segment {
                    kind: crate::motion::SegmentKind::Joint,
                    goal_positions: vec![0.5],
                    constraints: vec![],
                },
            )
            .unwrap();
        scene
            .add_segment_for(
                1,
                "far_back",
                crate::motion::Segment {
                    kind: crate::motion::SegmentKind::Joint,
                    goal_positions: vec![0.0],
                    constraints: vec![],
                },
            )
            .unwrap();
        scene.define_signal("carrying", false);
        scene.define_signal("belt_ok", false);
        scene.upsert_device(conveyor("belt", false));
        scene.upsert_sequence(Sequence {
            name: "pick".into(),
            steps: vec![
                step(
                    "go",
                    vec![
                        Action::StartMotion {
                            motion: "r_go".into(),
                        },
                        Action::StartMotion {
                            motion: "far_go".into(),
                        },
                    ],
                    Condition::Done,
                ),
                step(
                    "carry",
                    vec![
                        Action::StartMotion {
                            motion: "far_back".into(),
                        },
                        Action::Set {
                            signal: "carrying".into(),
                            value: true,
                        },
                    ],
                    Condition::All(vec![
                        Condition::Done,
                        Condition::Signal {
                            name: "belt_ok".into(),
                            value: true,
                        },
                    ]),
                ),
            ],
        });
        // A belt program driving no robot: <cell> too. It reads
        // `carrying` (same host as the writer → relay) and writes
        // `belt_ok` which `pick` reads (same host → relay).
        scene.upsert_sequence(Sequence {
            name: "belt".into(),
            steps: vec![step(
                "run",
                vec![
                    Action::Device {
                        device: "belt".into(),
                        command: DeviceCommand::Start,
                    },
                    Action::Set {
                        signal: "belt_ok".into(),
                        value: true,
                    },
                ],
                Condition::Signal {
                    name: "carrying".into(),
                    value: true,
                },
            )],
        });
        scene
    }

    #[test]
    fn multi_arm_program_lands_on_cell_with_handshakes() {
        let scene = two_arm_cell();
        let d = derive(&scene, None).unwrap();
        for robot in ["r", "far"] {
            let start = one(&d.points, &format!("{robot}.start"), IoDirection::Output);
            assert_eq!(
                (start.kind, start.source, start.host.as_deref()),
                (ChannelKind::Do, IoSource::RobotStart, Some(CELL_HOST))
            );
            let done = one(&d.points, &format!("{robot}.done"), IoDirection::Input);
            assert_eq!(
                (done.kind, done.source),
                (ChannelKind::Di, IoSource::RobotDone)
            );
        }
        // `far` runs two motions → a program-number word; `r` runs one → none.
        let word = one(&d.points, "far.program", IoDirection::Output);
        assert_eq!(word.kind, ChannelKind::Word);
        assert!(find(&d.points, "r.program", IoDirection::Output).is_empty());
        // Both programs are on <cell>: their shared signals are relays.
        let carrying = one(&d.points, "carrying", IoDirection::Output);
        assert_eq!(
            (carrying.source, carrying.status),
            (IoSource::Internal, PointStatus::Internal)
        );
        assert!(find(&d.points, "carrying", IoDirection::Input).is_empty());
        let placement: Vec<&IoFinding> = d
            .report
            .infos()
            .into_iter()
            .filter(|f| f.code == IoCode::ImplicitHost)
            .collect();
        assert_eq!(placement.len(), 2);
        assert!(d
            .report
            .infos()
            .iter()
            .any(|f| f.code == IoCode::WordUnexpressible));
    }

    #[test]
    fn a_reader_on_another_host_makes_a_handshake_and_a_program_set_narrows_it() {
        let mut scene = two_arm_cell();
        // A third program driving only `r` lives on <r> and reads
        // `carrying`, which `pick` writes on <cell>: one wire out, one in.
        scene.upsert_sequence(Sequence {
            name: "watch".into(),
            steps: vec![step(
                "wait",
                vec![Action::StartMotion {
                    motion: "r_go".into(),
                }],
                Condition::Rising {
                    name: "carrying".into(),
                },
            )],
        });
        let d = derive(&scene, None).unwrap();
        let out = one(&d.points, "carrying", IoDirection::Output);
        assert_eq!(
            (out.source, out.host.as_deref()),
            (IoSource::Handshake, Some(CELL_HOST))
        );
        let inp = one(&d.points, "carrying", IoDirection::Input);
        assert_eq!(
            (inp.source, inp.host.as_deref()),
            (IoSource::Handshake, Some(robot_host("r").as_str()))
        );
        assert_eq!(inp.readers.len(), 1);
        assert_eq!(inp.readers[0].sequence, "watch");
        // Narrowing the set to the two <cell> programs restores the relay.
        let d2 = derive(&scene, Some(&["pick", "belt"])).unwrap();
        assert_eq!(
            one(&d2.points, "carrying", IoDirection::Output).source,
            IoSource::Internal
        );
        assert!(find(&d2.points, "carrying", IoDirection::Input).is_empty());
        assert_eq!(d2.sequences, ["pick", "belt"]);
        assert_eq!(
            derive(&scene, Some(&["nope"])).unwrap_err(),
            IoError::UnknownSequence("nope".into())
        );
    }

    #[test]
    fn device_commands_split_into_aspects_and_magazines_are_cosmetic() {
        let mut scene = sample_scene();
        joint_motion(&mut scene, "go", 0.5);
        scene.upsert_device(conveyor("line", false));
        scene.upsert_device(Device {
            name: "agv".into(),
            kind: DeviceKind::Vehicle {
                path: crate::seq::VehiclePath {
                    waypoints: vec![
                        nalgebra::Point2::new(0.0, 0.0),
                        nalgebra::Point2::new(1.0, 0.0),
                    ],
                    stations: vec![("a".into(), 0), ("b".into(), 1)],
                    ring: false,
                },
                body: vec![],
                speed: 1.0,
                turn_speed: 1.0,
                start: "a".into(),
                allow_reverse: false,
                tray: None,
            },
        });
        scene.upsert_device(Device {
            name: "src_mark_1".into(),
            kind: DeviceKind::Source {
                pool: vec![],
                park: Isometry3::identity(),
                pitch: Vector3::zeros(),
                pose: Isometry3::identity(),
                interval: 0.0,
                running: false,
            },
        });
        scene.upsert_sequence(Sequence {
            name: "transfer".into(),
            steps: vec![
                step(
                    "index",
                    vec![
                        Action::Device {
                            device: "line".into(),
                            command: DeviceCommand::Advance(0.5),
                        },
                        Action::Device {
                            device: "src_mark_1".into(),
                            command: DeviceCommand::Start,
                        },
                    ],
                    Condition::DeviceDone {
                        device: "line".into(),
                    },
                ),
                step(
                    "dispatch",
                    vec![Action::Device {
                        device: "agv".into(),
                        command: DeviceCommand::Goto {
                            station: "b".into(),
                        },
                    }],
                    Condition::DeviceDone {
                        device: "agv".into(),
                    },
                ),
                step(
                    "speed",
                    vec![Action::Device {
                        device: "line".into(),
                        command: DeviceCommand::SetSpeed(0.2),
                    }],
                    Condition::Immediately,
                ),
            ],
        });
        let d = derive(&scene, None).unwrap();
        assert_eq!(
            one(&d.points, "line.index", IoDirection::Output).kind,
            ChannelKind::Do
        );
        assert_eq!(
            one(&d.points, "line", IoDirection::Input).source,
            IoSource::DeviceDone
        );
        assert_eq!(
            one(&d.points, "line.speed", IoDirection::Output).kind,
            ChannelKind::Ao
        );
        assert!(
            find(&d.points, "line", IoDirection::Output).is_empty(),
            "no Start/Stop → no run coil"
        );
        assert_eq!(
            one(&d.points, "agv.dispatch", IoDirection::Output).kind,
            ChannelKind::Do
        );
        assert_eq!(
            one(&d.points, "agv.station", IoDirection::Output).kind,
            ChannelKind::Word
        );
        assert_eq!(
            one(&d.points, "agv", IoDirection::Input).source,
            IoSource::DeviceDone
        );
        let mark = one(&d.points, "src_mark_1", IoDirection::Output);
        assert_eq!(mark.status, PointStatus::Cosmetic);
        assert_eq!(
            mark.writers.len(),
            1,
            "the commanding step is still recorded"
        );
        let words: Vec<&IoFinding> = d
            .report
            .infos()
            .into_iter()
            .filter(|f| f.code == IoCode::WordUnexpressible)
            .collect();
        assert_eq!(words.len(), 2, "station word + speed analog");
        // Cosmetic rows stay out of the counts.
        let summary = io_summary(&d.points);
        let cell = &summary[CELL_HOST];
        assert_eq!(cell["DO"], 2); // line.index, agv.dispatch
        assert_eq!(cell["DI"], 2); // line, agv
        assert_eq!(cell["Word"], 1);
        assert_eq!(cell["AO"], 1);
    }

    #[test]
    fn name_clashes_are_warned() {
        let mut scene = pick_cell();
        scene.define_signal("conv", false); // a signal named like the belt
        scene.define_signal("agv.station", false);
        let d = derive(&scene, None).unwrap();
        let clashes: Vec<String> = d
            .report
            .warnings()
            .iter()
            .filter(|f| f.code == IoCode::NameClash)
            .map(|f| f.message.clone())
            .collect();
        assert_eq!(clashes.len(), 2, "{clashes:?}");
        assert!(clashes
            .iter()
            .any(|m| m.contains("`conv`") && m.contains("signal") && m.contains("device")));
        assert!(clashes.iter().any(|m| m.contains("`agv.station`")));
    }

    #[test]
    fn renderers_agree_on_the_row_set() {
        let scene = two_arm_cell();
        let d = derive(&scene, None).unwrap();
        let csv = render_csv(&d);
        let md = render_markdown(&d);
        let json = render_json(&d);
        let rows = csv.lines().filter(|l| !l.starts_with('#')).count() - 1;
        assert_eq!(rows, d.points.len());
        assert_eq!(
            csv.lines().next().unwrap().split(',').count(),
            IO_COLUMNS.len()
        );
        assert!(md.lines().next().unwrap().starts_with("| name | aspect |"));
        assert_eq!(
            md.lines()
                .filter(|l| l.starts_with("| ") && !l.starts_with("| name"))
                .count(),
            d.points.len()
        );
        assert!(json.starts_with("{\"sequences\":[\"pick\",\"belt\"],\"points\":["));
        assert!(json.contains("\"name\":\"far\",\"aspect\":\"program\""));
        assert!(csv.contains("# <cell>: DI 2, DO 3, Word 1"), "{csv}");
    }

    // ------------------------------------------------ assignment layer

    pub(crate) fn ur_channels() -> Vec<IoChannel> {
        (0..8)
            .map(|i| IoChannel {
                id: format!("DI{i}"),
                kind: ChannelKind::Di,
                port: Some(i),
                address: None,
                electrical: None,
            })
            .chain((0..8).map(|i| IoChannel {
                id: format!("DO{i}"),
                kind: ChannelKind::Do,
                port: Some(i),
                address: None,
                electrical: None,
            }))
            .collect()
    }

    pub(crate) fn node(
        name: &str,
        kind: IoNodeKind,
        programs: &[&str],
        channels: Vec<IoChannel>,
    ) -> IoNode {
        IoNode {
            name: name.into(),
            kind,
            programs: programs.iter().map(|s| s.to_string()).collect(),
            uplink: None,
            channels,
            place: None,
            model: None,
        }
    }

    fn binding(label: &str, direction: IoDirection, node: &str, channel: &str) -> IoBinding {
        IoBinding {
            point: IoPointId::parse(label, direction),
            node: node.into(),
            channel: channel.into(),
            tag: None,
            field: None,
            invert: false,
            contact: None,
            safety: false,
            device: None,
            note: None,
            auto: false,
        }
    }

    fn codes(report: &IoReport, severity: Severity) -> Vec<&'static str> {
        report
            .findings
            .iter()
            .filter(|f| f.severity == severity)
            .map(|f| f.code.as_str())
            .collect()
    }

    #[test]
    fn a_robot_controller_node_hosts_its_program_and_takes_the_bindings() {
        let mut scene = pick_cell();
        // No node → no unbound findings: nothing is being assigned yet.
        assert!(derive(&scene, None).unwrap().report.findings.is_empty());
        scene
            .upsert_io_node(node(
                "UR",
                IoNodeKind::RobotController {
                    robots: vec!["r".into()],
                },
                &[],
                ur_channels(),
            ))
            .unwrap();
        // The program drives `r` alone → it now lives on the declared node,
        // and every point is unbound there: two errors (sensor read, run
        // coil), two warnings (the candidates).
        let d = derive(&scene, None).unwrap();
        assert!(d.points.iter().all(|p| p.host.as_deref() == Some("UR")));
        assert_eq!(codes(&d.report, Severity::Error), ["unbound", "unbound"]);
        assert_eq!(codes(&d.report, Severity::Warning), ["unbound", "unbound"]);
        for (label, dir, ch) in [
            ("part_at_pick", IoDirection::Input, "DI2"),
            ("spec_ok", IoDirection::Input, "DI3"),
            ("conv", IoDirection::Output, "DO0"),
            ("vacuum", IoDirection::Output, "DO1"),
        ] {
            scene.bind_io(binding(label, dir, "UR", ch)).unwrap();
        }
        let d = derive(&scene, None).unwrap();
        assert!(d.report.findings.is_empty(), "{}", d.report);
        assert!(d
            .points
            .iter()
            .all(|p| matches!(p.status, PointStatus::Bound(_))));
        let (b, n, c) = d
            .binding_of(one(&d.points, "conv", IoDirection::Output))
            .unwrap();
        assert_eq!(
            (b.channel.as_str(), n.name.as_str(), c.port),
            ("DO0", "UR", Some(0))
        );
        // The projection onto the script's port maps.
        let (inputs, outputs) = sequence_io(&d, scene.io_map(), "UR").unwrap();
        assert_eq!(
            inputs["part_at_pick"],
            IoPort {
                port: 2,
                invert: false
            }
        );
        assert_eq!(inputs["spec_ok"].port, 3);
        assert_eq!(outputs["conv"].port, 0);
        assert_eq!(outputs["vacuum"].port, 1);
        // The table prints the wiring.
        let csv = render_csv(&d);
        assert!(
            csv.lines()
                .any(|l| l.starts_with("conv,,output,DO,device:run,UR,UR,DO0,")),
            "{csv}"
        );
    }

    #[test]
    fn binding_lints_duplicate_kind_host_stale_and_unknown() {
        let mut scene = pick_cell();
        scene
            .upsert_io_node(node(
                "UR",
                IoNodeKind::RobotController {
                    robots: vec!["r".into()],
                },
                &[],
                ur_channels(),
            ))
            .unwrap();
        scene
            .upsert_io_node(node("PLC1", IoNodeKind::Plc, &[], ur_channels()))
            .unwrap();
        // Two points on one channel; an input on an output channel; a
        // binding on a node that does not reach the point's host; a
        // binding whose point is not derived.
        scene
            .bind_io(binding("part_at_pick", IoDirection::Input, "UR", "DI2"))
            .unwrap();
        scene
            .bind_io(binding("spec_ok", IoDirection::Input, "UR", "DI2"))
            .unwrap();
        scene
            .bind_io(binding("conv", IoDirection::Output, "UR", "DI5"))
            .unwrap();
        scene
            .bind_io(binding("vacuum", IoDirection::Output, "PLC1", "DO1"))
            .unwrap();
        scene
            .bind_io(binding("ghost", IoDirection::Output, "UR", "DO7"))
            .unwrap();
        let d = derive(&scene, None).unwrap();
        let errors = codes(&d.report, Severity::Error);
        assert!(errors.contains(&"duplicate"), "{}", d.report);
        assert!(errors.contains(&"kind"), "{}", d.report);
        assert!(errors.contains(&"host_mismatch"), "{}", d.report);
        assert!(
            codes(&d.report, Severity::Warning).contains(&"stale_binding"),
            "{}",
            d.report
        );
        // The scene refuses what it can see immediately.
        assert!(matches!(
            scene.bind_io(binding("conv", IoDirection::Output, "nope", "DO0")),
            Err(crate::SceneError::UnknownIoNode(_))
        ));
        assert!(matches!(
            scene.bind_io(binding("conv", IoDirection::Output, "UR", "DO99")),
            Err(crate::SceneError::UnknownIoChannel(_, _))
        ));
        assert!(matches!(
            scene.upsert_io_node(node(
                "X",
                IoNodeKind::RobotController {
                    robots: vec!["nobody".into()]
                },
                &[],
                vec![]
            )),
            Err(crate::SceneError::UnknownRobot(_))
        ));
        // Removing a node drops its bindings.
        scene.remove_io_node("PLC1").unwrap();
        assert!(scene.io_map().bindings.iter().all(|b| b.node != "PLC1"));
        assert_eq!(
            scene
                .unbind_io(&IoPointId::parse("ghost", IoDirection::Output), None)
                .unwrap(),
            1
        );
        assert!(scene
            .unbind_io(&IoPointId::parse("ghost", IoDirection::Output), None)
            .is_err());
    }

    #[test]
    fn plc_master_declaration_moves_programs_and_mirrors_robot_handshakes() {
        let mut scene = two_arm_cell();
        // A PLC runs both programs; `r` has its own declared controller
        // (`far` keeps an implicit one); a remote station hangs off the PLC.
        scene
            .upsert_io_node(node(
                "PLC1",
                IoNodeKind::Plc,
                &["pick", "belt"],
                ur_channels(),
            ))
            .unwrap();
        scene
            .upsert_io_node(node(
                "RC1",
                IoNodeKind::RobotController {
                    robots: vec!["r".into()],
                },
                &[],
                ur_channels(),
            ))
            .unwrap();
        let mut rio = node("RIO1", IoNodeKind::RemoteIo, &[], ur_channels());
        rio.uplink = Some(Uplink {
            parent: "PLC1".into(),
            bus: Some("PROFINET".into()),
        });
        scene.upsert_io_node(rio).unwrap();
        let d = derive(&scene, None).unwrap();
        // Programs sit on PLC1: no <cell> any more, no implicit_host note.
        assert!(!codes(&d.report, Severity::Info).contains(&"implicit_host"));
        assert_eq!(
            one(&d.points, "belt", IoDirection::Output).host.as_deref(),
            Some("PLC1")
        );
        // The handshake to `r` has both ends: PLC1 drives, RC1 mirrors.
        let starts = find(&d.points, "r.start", IoDirection::Output);
        assert_eq!(starts.len(), 1);
        assert_eq!(starts[0].host.as_deref(), Some("PLC1"));
        let start_in = one(&d.points, "r.start", IoDirection::Input);
        assert_eq!(start_in.host.as_deref(), Some("RC1"));
        let done_out = one(&d.points, "r.done", IoDirection::Output);
        assert_eq!(done_out.host.as_deref(), Some("RC1"));
        // `far` has no declared controller: driving end only.
        assert!(find(&d.points, "far.start", IoDirection::Input).is_empty());
        // The belt program's sensorless relay `belt_ok` stays internal on PLC1.
        assert_eq!(
            one(&d.points, "belt_ok", IoDirection::Output).status,
            PointStatus::Internal
        );
        // A remote station's channel takes a PLC1 point through the uplink.
        scene
            .bind_io(binding("belt", IoDirection::Output, "RIO1", "DO0"))
            .unwrap();
        let d = derive(&scene, None).unwrap();
        assert!(matches!(
            one(&d.points, "belt", IoDirection::Output).status,
            PointStatus::Bound(_)
        ));
        assert!(
            !codes(&d.report, Severity::Error).contains(&"host_mismatch"),
            "{}",
            d.report
        );
        // The projection for RC1 sees its mirror inputs once bound.
        scene
            .bind_io(binding("r.start", IoDirection::Input, "RC1", "DI0"))
            .unwrap();
        scene
            .bind_io(binding("r.done", IoDirection::Output, "RC1", "DO0"))
            .unwrap();
        let d = derive(&scene, None).unwrap();
        let (inputs, outputs) = sequence_io(&d, scene.io_map(), "RC1").unwrap();
        // `r.start` on RC1 is the robot's own start input — not a script
        // key; `r.done` out is the mirror — not a script key either.
        assert!(
            inputs.is_empty() && outputs.is_empty(),
            "{inputs:?} {outputs:?}"
        );
        // A program listed twice, a program that is not a sequence.
        scene
            .upsert_io_node(node("PLC2", IoNodeKind::Plc, &["pick", "nope"], vec![]))
            .unwrap();
        let d = derive(&scene, None).unwrap();
        let errors = codes(&d.report, Severity::Error);
        assert!(errors.contains(&"program_multihost"), "{}", d.report);
        assert!(errors.contains(&"unknown_ref"), "{}", d.report);
    }

    #[test]
    fn declarations_override_the_rules() {
        let mut scene = pick_cell();
        scene.define_signal("carrying", false); // never used
        scene.upsert_device(Device {
            name: "feeder".into(),
            kind: DeviceKind::Source {
                pool: vec![],
                park: Isometry3::identity(),
                pitch: Vector3::zeros(),
                pose: Isometry3::identity(),
                interval: 0.0,
                running: false,
            },
        });
        let decl = |name: &str, role: Option<DeclRole>| IoDecl {
            name: name.into(),
            role,
            kind: None,
            safety: false,
            pair: None,
            note: None,
        };
        // A constant flag: `spec_ok` is not an input after all.
        scene.declare_io(decl("spec_ok", Some(DeclRole::Internal)));
        // `vacuum` is a state flag, not a coil: off the table.
        scene.declare_io(decl("vacuum", Some(DeclRole::Exclude)));
        // A magazine promoted to a real feeder.
        scene.declare_io(decl("feeder", Some(DeclRole::Output)));
        // An unmodelled safety input, two channels.
        scene.declare_io(IoDecl {
            name: "door_ch1".into(),
            role: Some(DeclRole::Input),
            kind: Some(ChannelKind::SafeDi),
            safety: true,
            pair: Some("door_ch2".into()),
            note: None,
        });
        scene.declare_io(IoDecl {
            name: "door_ch2".into(),
            role: Some(DeclRole::Input),
            kind: Some(ChannelKind::SafeDi),
            safety: true,
            pair: Some("door_ch1".into()),
            note: None,
        });
        // A declaration that names nothing and adds nothing.
        scene.declare_io(decl("nobody", None));
        let d = derive(&scene, None).unwrap();
        assert_eq!(
            one(&d.points, "spec_ok", IoDirection::Output).status,
            PointStatus::Internal
        );
        assert!(find(&d.points, "vacuum", IoDirection::Output).is_empty());
        let feeder = one(&d.points, "feeder", IoDirection::Output);
        assert_eq!(
            (feeder.source, feeder.status),
            (IoSource::DeviceRun, PointStatus::Unbound)
        );
        let door = one(&d.points, "door_ch1", IoDirection::Input);
        assert_eq!(
            (door.kind, door.source, door.safety),
            (ChannelKind::SafeDi, IoSource::Declared, true)
        );
        assert!(
            codes(&d.report, Severity::Error).contains(&"unknown_ref"),
            "{}",
            d.report
        );
        assert!(scene.undeclare_io("nobody").is_ok());
        assert!(scene.undeclare_io("nobody").is_err());
    }

    // ------------------------------------------ auto-assign / lints / topology

    #[test]
    fn auto_assign_fills_unbound_points_deterministically_and_keeps_existing() {
        let mut scene = pick_cell();
        scene
            .upsert_io_node(node(
                "UR",
                IoNodeKind::RobotController {
                    robots: vec!["r".into()],
                },
                &[],
                ur_channels(),
            ))
            .unwrap();
        // One binding by hand on DI5: kept; the rest fill the first free
        // channels in table order (conv, part_at_pick, spec_ok, vacuum).
        scene
            .bind_io(binding("spec_ok", IoDirection::Input, "UR", "DI5"))
            .unwrap();
        let report = scene.auto_assign_io(None, false).unwrap();
        assert!(report.findings.is_empty(), "{report}");
        let wired: Vec<(String, String)> = scene
            .io_map()
            .bindings
            .iter()
            .map(|b| (b.point.label(), b.channel.clone()))
            .collect();
        assert_eq!(
            wired,
            [
                ("spec_ok".to_string(), "DI5".to_string()),
                ("conv".to_string(), "DO0".to_string()),
                ("part_at_pick".to_string(), "DI0".to_string()),
                ("vacuum".to_string(), "DO1".to_string()),
            ]
        );
        // Idempotent. A reassign renumbers only what auto-assign placed: the
        // hand binding keeps DI5, and the automatic ones are marked.
        let again = scene.auto_assign_io(None, false).unwrap();
        assert!(again.findings.is_empty());
        assert_eq!(scene.io_map().bindings.len(), 4);
        assert!(scene
            .io_map()
            .bindings
            .iter()
            .all(|b| b.auto == (b.point.name != "spec_ok")));
        scene
            .unbind_io(&IoPointId::parse("part_at_pick", IoDirection::Input), None)
            .unwrap();
        scene
            .bind_io(binding("part_at_pick", IoDirection::Input, "UR", "DI3"))
            .unwrap();
        scene.auto_assign_io(None, true).unwrap();
        let by_name = |n: &str| {
            scene
                .io_map()
                .bindings
                .iter()
                .find(|b| b.point.name == n)
                .unwrap()
                .channel
                .clone()
        };
        assert_eq!(by_name("spec_ok"), "DI5");
        assert_eq!(by_name("part_at_pick"), "DI3");
        assert_eq!(
            (by_name("conv"), by_name("vacuum")),
            ("DO0".to_string(), "DO1".to_string())
        );
        // The table's note column tells placed from chosen.
        let d = derive(&scene, None).unwrap();
        let csv = render_csv(&d);
        assert!(
            csv.lines()
                .any(|l| l.starts_with("conv,") && l.ends_with(",bound,auto")),
            "{csv}"
        );
        assert!(
            csv.lines()
                .any(|l| l.starts_with("spec_ok,") && l.ends_with(",bound,")),
            "{csv}"
        );
    }

    #[test]
    fn auto_assign_reports_capacity_and_leaves_implicit_hosts_alone() {
        let mut scene = two_arm_cell();
        // Only a PLC with two DIs: the belt program's points fill it, the
        // rest is a capacity finding; <cell> points stay unbound.
        let mut plc = node("PLC1", IoNodeKind::Plc, &["belt"], vec![]);
        plc.channels = ur_channels()
            .into_iter()
            .filter(|c| c.id == "DI0" || c.id == "DI1" || c.id == "DO0")
            .collect();
        scene.upsert_io_node(plc).unwrap();
        let report = scene.auto_assign_io(None, false).unwrap();
        // The belt program (on PLC1) drives `belt` (DO) and reads
        // `carrying` (a handshake input from pick on <cell>): both bound.
        assert!(scene
            .io_map()
            .bindings
            .iter()
            .any(|b| b.point.name == "belt"));
        assert!(scene
            .io_map()
            .bindings
            .iter()
            .any(|b| b.point.name == "carrying" && b.point.direction == IoDirection::Input));
        // pick's points on <cell> cannot be placed: their unbound errors
        // name the implicit host; PLC1 ran out of DO for `belt_ok`, which
        // the capacity finding says.
        let unbound: Vec<&IoFinding> = report
            .findings
            .iter()
            .filter(|f| f.code == IoCode::Unbound)
            .collect();
        assert!(
            unbound.iter().any(|f| f.message.contains("<cell>")),
            "{report}"
        );
        assert!(
            unbound
                .iter()
                .any(|f| f.message.contains("`belt_ok` (DO") && f.message.contains("PLC1")),
            "{report}"
        );
        assert!(
            codes(&report, Severity::Warning).contains(&"capacity"),
            "{report}"
        );
        // A remote station declared *before* its controller still comes
        // second: the host's own channels fill first, then the uplinked
        // ones take the overflow.
        let mut scene = two_arm_cell();
        let mut rio = node("RIO1", IoNodeKind::RemoteIo, &[], ur_channels());
        rio.uplink = Some(Uplink {
            parent: "PLC1".into(),
            bus: None,
        });
        let mut plc = node("PLC1", IoNodeKind::Plc, &["belt"], vec![]);
        plc.channels = ur_channels()
            .into_iter()
            .filter(|c| c.id == "DI0")
            .collect();
        scene
            .set_io_map(IoMap {
                nodes: vec![rio, plc],
                bindings: vec![],
                decls: vec![],
            })
            .unwrap();
        scene.auto_assign_io(None, false).unwrap();
        let on = |name: &str| {
            let b = scene
                .io_map()
                .bindings
                .iter()
                .find(|b| b.point.name == name)
                .unwrap();
            (b.node.clone(), b.channel.clone())
        };
        assert_eq!(on("belt"), ("RIO1".to_string(), "DO0".to_string()));
        assert_eq!(on("carrying"), ("PLC1".to_string(), "DI0".to_string()));
    }

    #[test]
    fn electrical_and_safety_lints() {
        let mut scene = pick_cell();
        let mut ur = node(
            "UR",
            IoNodeKind::RobotController {
                robots: vec!["r".into()],
            },
            &[],
            ur_channels(),
        );
        // A 24 V PNP input module, plus a safety input channel.
        for c in &mut ur.channels {
            if c.id.starts_with("DI") {
                c.electrical = Some(Electrical {
                    voltage: Some(24.0),
                    logic: Some(Logic::Pnp),
                });
            }
        }
        ur.channels.push(IoChannel {
            id: "SDI0".into(),
            kind: ChannelKind::SafeDi,
            port: None,
            address: None,
            electrical: None,
        });
        scene.upsert_io_node(ur).unwrap();
        // A 12 V NPN sensor on that module; a standard signal on the safety
        // channel; a declared safety input nobody reads, on a standard DI.
        let mut beam = binding("part_at_pick", IoDirection::Input, "UR", "DI2");
        beam.device = Some(Electrical {
            voltage: Some(12.0),
            logic: Some(Logic::Npn),
        });
        scene.bind_io(beam).unwrap();
        scene
            .bind_io(binding("spec_ok", IoDirection::Input, "UR", "SDI0"))
            .unwrap();
        scene.declare_io(IoDecl {
            name: "estop_ok".into(),
            role: Some(DeclRole::Input),
            kind: None,
            safety: true,
            pair: None,
            note: None,
        });
        scene
            .bind_io(binding("estop_ok", IoDirection::Input, "UR", "DI4"))
            .unwrap();
        let d = derive(&scene, None).unwrap();
        let warnings = codes(&d.report, Severity::Warning);
        assert!(warnings.contains(&"voltage"), "{}", d.report);
        assert!(warnings.contains(&"polarity"), "{}", d.report);
        assert!(
            warnings.iter().filter(|c| **c == "safety").count() >= 2,
            "{}",
            d.report
        );
        assert!(warnings.contains(&"safety_unread"), "{}", d.report);
        // A two-channel pair: bound on different kinds → error.
        scene.declare_io(IoDecl {
            name: "door_ch1".into(),
            role: Some(DeclRole::Input),
            kind: Some(ChannelKind::SafeDi),
            safety: true,
            pair: Some("door_ch2".into()),
            note: None,
        });
        scene.declare_io(IoDecl {
            name: "door_ch2".into(),
            role: Some(DeclRole::Input),
            kind: Some(ChannelKind::SafeDi),
            safety: true,
            pair: Some("door_ch1".into()),
            note: None,
        });
        scene
            .bind_io(binding("door_ch1", IoDirection::Input, "UR", "DI6"))
            .unwrap();
        let d = derive(&scene, None).unwrap();
        assert!(
            codes(&d.report, Severity::Error).contains(&"safety_pair"),
            "{}",
            d.report
        );
        scene
            .bind_io(binding("door_ch2", IoDirection::Input, "UR", "DI7"))
            .unwrap();
        let d = derive(&scene, None).unwrap();
        assert!(
            !codes(&d.report, Severity::Error).contains(&"safety_pair"),
            "{}",
            d.report
        );
        // Two programs writing one coil: the ownership rule without a bake.
        scene.upsert_sequence(Sequence {
            name: "other".into(),
            steps: vec![step(
                "also",
                vec![Action::Set {
                    signal: "vacuum".into(),
                    value: false,
                }],
                Condition::Immediately,
            )],
        });
        let d = derive(&scene, None).unwrap();
        assert!(
            codes(&d.report, Severity::Error).contains(&"multiple_drivers"),
            "{}",
            d.report
        );
        assert!(!codes(
            &derive(&scene, Some(&["pick"])).unwrap().report,
            Severity::Error
        )
        .contains(&"multiple_drivers"));
    }

    #[test]
    fn topology_has_hosts_programs_wires_and_layers() {
        let mut scene = two_arm_cell();
        scene
            .upsert_io_node(node(
                "PLC1",
                IoNodeKind::Plc,
                &["pick", "belt"],
                ur_channels(),
            ))
            .unwrap();
        let mut rio = node("RIO1", IoNodeKind::RemoteIo, &[], ur_channels());
        rio.uplink = Some(Uplink {
            parent: "PLC1".into(),
            bus: Some("PROFINET".into()),
        });
        scene.upsert_io_node(rio).unwrap();
        scene
            .bind_io(binding("belt", IoDirection::Output, "RIO1", "DO0"))
            .unwrap();
        // A third program on <r> reads `carrying` from pick: a handshake.
        scene.upsert_sequence(Sequence {
            name: "watch".into(),
            steps: vec![step(
                "wait",
                vec![Action::StartMotion {
                    motion: "r_go".into(),
                }],
                Condition::Rising {
                    name: "carrying".into(),
                },
            )],
        });
        let d = derive(&scene, None).unwrap();
        let t = topology(&scene, &d, false);
        let ids: Vec<&str> = t.nodes.iter().map(|n| n.id.as_str()).collect();
        assert!(
            ids.contains(&"host:PLC1") && ids.contains(&"host:RIO1") && ids.contains(&"host:<r>")
        );
        assert!(
            ids.contains(&"prog:pick") && ids.contains(&"prog:belt") && ids.contains(&"prog:watch")
        );
        assert!(
            ids.contains(&"device:belt") && ids.contains(&"robot:r") && ids.contains(&"robot:far")
        );
        let kinds = |k: &str| t.edges.iter().filter(|e| e.kind.as_str() == k).count();
        assert_eq!(kinds("uplink"), 1);
        assert!(kinds("handshake") >= 1);
        assert!(kinds("functional") >= 1);
        assert!(kinds("io") >= 4);
        // The belt wire runs to RIO1 (its binding), labelled with the channel.
        let belt = t
            .edges
            .iter()
            .find(|e| matches!(&e.kind, TopoEdgeKind::Io { point, .. } if point.name == "belt"))
            .unwrap();
        assert_eq!(
            (belt.from.as_str(), belt.to.as_str()),
            ("host:RIO1", "device:belt")
        );
        assert!(belt.label.contains("RIO1.DO0"));
        // Layers filter edges; hosts always stay.
        let (nodes, edges) = topo_visible(&t, &[TopoLayer::Network]);
        assert_eq!(edges.len(), 1);
        assert!(nodes.iter().any(|n| n.id == "host:PLC1"));
        assert!(!nodes.iter().any(|n| n.id == "device:belt"));
        let dot = render_dot(&t, &[]);
        assert!(dot.starts_with("digraph io_map {"));
        assert!(dot.contains("subgraph \"cluster_PLC1\""));
        assert!(dot.contains("\"host:RIO1\" -> \"host:PLC1\""));
        let mmd = render_mermaid(&t, &[TopoLayer::Wiring]);
        assert!(mmd.starts_with("flowchart LR"));
        assert!(
            mmd.contains("host_RIO1 ---|\"PROFINET\"| host_PLC1"),
            "{mmd}"
        );
        assert!(
            mmd.contains("host_RIO1 -->|\"belt → RIO1.DO0\"| device_belt"),
            "{mmd}"
        );
        let json = render_topology_json(&t, &[]);
        assert!(json.contains("\"id\":\"prog:pick\""));
    }
}
