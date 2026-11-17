//! NC part programs, read the way a machine tool's control executes them.
//!
//! [`crate::gcode`] reads G-code as a *shape*: a toolpath in a part frame
//! for a robot to follow, where anything that belongs to a machine — tool
//! changes, tool length, machine-coordinate moves, canned cycles — is
//! refused. This module reads the same language as a *program*: against a
//! machine's travels and rates, its work-offset table and its tool table,
//! and turns it into what the axes do and when.
//!
//! Two steps, both pure:
//!
//! * [`interpret`] walks the program block by block with the control's
//!   modal state and returns the moves in **machine coordinates** — the
//!   values the axes' joints take — with the line each came from. What it
//!   cannot read it refuses by line number, never skips: an unknown word,
//!   a work offset or a tool nobody entered, a point past an axis's
//!   travel.
//! * [`time_plan`] times those moves the way a control's look-ahead does:
//!   a rapid is a straight, synchronised move from rest to rest at the
//!   axes' rapid rates; a run of cutting moves is one continuous path at
//!   the programmed feed, slowed for corners by a junction-deviation rule
//!   and bounded by the axes' accelerations, each corner rounded within
//!   that deviation. Dwells and tool changes are pauses.
//!
//! Scope (the Haas dialect a CAM post writes for 3-axis milling): G0/G1,
//! G2/G3 (IJK or R, helical), G4, G17–G19, G20/G21, G28, G40, G43/G49,
//! G53, G54–G59 and G154 P, G73/G81/G82/G83 with G80 and G98/G99, G90/G91,
//! G94; M1, M2/M30, M3/M4/M5, M6, M8/M9, M99. Not read, and refused by
//! line: cutter radius compensation (G41/G42 — post the path compensated),
//! macros and subprograms, probing, the remaining canned cycles, and any
//! M code that is neither in the list above nor in the machine's own
//! table.
//!
//! **Machine functions.** An M code of the machine's table
//! ([`NcMachine::m_codes`] — a door, a vise, a relay that calls a robot)
//! is not the control's to carry out: it hands it to the machine's logic
//! and waits for that to finish. So such a code ends a **block** of the
//! program — the stretch the control runs on its own — and the program is
//! the blocks in order with something else happening between them
//! ([`outline`]). A block is read by number ([`NcOptions::block`]): the
//! text before it is walked for the modal state it leaves, and the block
//! itself starts from where the axes stand.
//!
//! Lengths are metres, speeds metres per second, in machine coordinates:
//! each axis reads zero at home and travels to the negative side or the
//! positive as its joint's limits say.

use std::collections::BTreeMap;

use botrail_traj::JointTrajectory;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::gcode::{arc_center_from_radius, arc_points, code_eq, lex_words, strip_comments, Plane};

/// One entry of a machine's tool table: what `G43 H<n>` adds to Z — the
/// figure the control works with — and, where it is known, the tool that
/// is actually in that pocket.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct NcTool {
    /// Gauge line to tip (m), as entered in the table.
    pub length: f64,
    /// Cutter diameter (m), where it is known — carried for the setup
    /// sheet, not used for motion.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub diameter: Option<f64>,
    /// The tool as a body: what cuts, and what must not touch anything.
    /// The table's `length` is what the axes are positioned by; this one's
    /// is where the tip really is. They agree unless somebody measured
    /// wrong — which is what a setup is checked for.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body: Option<NcToolBody>,
}

/// The shape of a cutter's end.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
#[serde(rename_all = "snake_case")]
pub enum NcCutter {
    /// Flat end mill.
    Flat,
    /// Ball end mill: a hemispherical end.
    Ball,
    /// Bull-nose end mill: a flat end with rounded corners.
    BullNose,
    /// Twist drill. Cut as a flat end of its diameter: the hole's wall
    /// and depth are right, the cone at its bottom is not drawn.
    Drill,
}

/// A tool as it stands in the spindle, from the gauge line down: the
/// holder, the shank, the flutes. Lengths in metres.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct NcToolBody {
    pub cutter: NcCutter,
    /// Cutting diameter.
    pub diameter: f64,
    /// How far up from the tip it cuts.
    pub flute_length: f64,
    /// Gauge line to tip.
    pub length: f64,
    /// Corner radius of a bull-nose cutter.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub corner_radius: Option<f64>,
    /// Diameter of the shank between the flutes and the holder; the
    /// cutter's without.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub shank_diameter: Option<f64>,
    /// The holder: its diameter, and how far below the gauge line it
    /// reaches.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub holder: Option<[f64; 2]>,
}

impl NcToolBody {
    /// Why this is not a tool, if it is not one.
    pub fn validate(&self) -> Result<(), String> {
        let positive = |v: f64| v.is_finite() && v > 0.0;
        if !(positive(self.diameter) && positive(self.flute_length) && positive(self.length)) {
            return Err("diameter, flute length and length must be positive".to_string());
        }
        let holder = self.holder.map_or(0.0, |h| h[1]);
        if let Some([diameter, reach]) = self.holder {
            if !(positive(diameter) && positive(reach)) {
                return Err("a holder's diameter and length must be positive".to_string());
            }
        }
        if self.flute_length + holder > self.length + 1e-12 {
            return Err(format!(
                "{:.1} mm of flutes and {:.1} mm of holder do not fit {:.1} mm from gauge line to tip",
                self.flute_length * 1e3,
                holder * 1e3,
                self.length * 1e3
            ));
        }
        if self.shank_diameter.is_some_and(|d| !positive(d)) {
            return Err("a shank's diameter must be positive".to_string());
        }
        match (self.cutter, self.corner_radius) {
            (NcCutter::BullNose, Some(r)) if positive(r) && r <= self.diameter / 2.0 + 1e-12 => {}
            (NcCutter::BullNose, _) => {
                return Err(
                    "a bull-nose cutter needs a corner radius no greater than its own".to_string(),
                )
            }
            (NcCutter::Ball, _) if self.flute_length < self.diameter / 2.0 - 1e-12 => {
                return Err("a ball cutter's flutes are at least its radius long".to_string())
            }
            _ => {}
        }
        Ok(())
    }
}

fn default_junction_deviation() -> f64 {
    5e-5
}

/// A machine tool's control, as far as running a program needs it: which
/// joints of its robot are the X, Y and Z axes (a joint's value is that
/// axis's machine coordinate, its limits the travel, its velocity limit
/// the rapid rate), the rates the model does not carry, and the two tables
/// an operator fills in — work offsets and tools.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct NcMachine {
    /// The robot whose joints are the axes; also the machine's name.
    pub robot: String,
    /// Joint names of X, Y and Z.
    pub axes: [String; 3],
    /// Greatest cutting feed (m/s).
    pub max_feed: f64,
    /// Acceleration bound of X, Y and Z (m/s²).
    pub acceleration: [f64; 3],
    /// How far the path may round a corner (m) — what sets the speed a
    /// corner is taken at.
    #[serde(default = "default_junction_deviation")]
    pub junction_deviation: f64,
    /// Tool-to-tool time of the changer (s).
    pub tool_change_s: f64,
    /// Greatest spindle speed (rev/min); an `S` word past it is refused.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_max_rpm: Option<f64>,
    /// Work offsets by name (`G54`, `G154 P1`): the machine coordinates of
    /// the work zero.
    #[serde(default)]
    pub work_offsets: BTreeMap<String, [f64; 3]>,
    /// The tool table, by tool number.
    #[serde(default)]
    pub tools: BTreeMap<u32, NcTool>,
    /// The machine's M-code table: the M codes that are the machine's to
    /// carry out, not the control's — `M180` a window, `M70` a vise, `M21`
    /// a relay that waits for its M-FIN — each with what it does, in
    /// words. The control stops at one and waits; what happens then is the
    /// machine's own program ([`outline`]). An M code that is neither the
    /// control's nor here is refused.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub m_codes: BTreeMap<u32, String>,
    /// The tool changer the control works at a tool change, where the
    /// machine's model has one to move.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub changer: Option<NcChanger>,
}

/// An umbrella tool changer as the control works it: a carousel of
/// pockets — tool `n` in pocket `n` — on a shuttle that carries it to the
/// spindle. At a tool change the head stands at the top of its travel;
/// the shuttle runs in, the empty pocket taking the tool that is in the
/// spindle; the carousel turns the next tool's pocket to the spindle; and
/// the shuttle runs out again, leaving that tool behind — all within the
/// machine's tool-to-tool time ([`work_changer`]). The head's lift off the
/// tool is not part of it, and nothing of the changer is collided: it is
/// drawn, not checked.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct NcChanger {
    /// The shuttle's joint: prismatic, at rest at its lower limit and at
    /// the spindle at its upper.
    pub shuttle: String,
    /// The carousel's joint: at 0 pocket 1 stands at the spindle's side,
    /// and pocket `n` is `n - 1` pockets further round its positive turn.
    pub carousel: String,
    /// How many pockets the carousel has.
    pub pockets: u32,
}

impl NcChanger {
    /// Where the carousel's joint stands with `tool`'s pocket at the
    /// spindle — the turn of it nearest to `from`, so the carousel goes
    /// the short way round. `None` for tool 0, which has no pocket: the
    /// spindle is left empty and the carousel where it is.
    pub fn pocket_turn(&self, tool: u32, from: f64) -> Option<f64> {
        if tool == 0 || self.pockets == 0 {
            return None;
        }
        let step = std::f64::consts::TAU / f64::from(self.pockets);
        let pocket = -f64::from((tool - 1) % self.pockets) * step;
        let around = (pocket - from).rem_euclid(std::f64::consts::TAU);
        Some(if around > std::f64::consts::PI {
            from + around - std::f64::consts::TAU
        } else {
            from + around
        })
    }
}

/// The M codes the control carries out itself; none of them can be a
/// machine function.
pub const CONTROL_M_CODES: [u32; 11] = [0, 1, 2, 3, 4, 5, 6, 8, 9, 30, 99];

/// A named part program.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[cfg_attr(feature = "schema", derive(schemars::JsonSchema))]
pub struct NcProgram {
    pub name: String,
    pub text: String,
}

/// A work offset's name as the table keys it: `G54`…`G59`, or `G154 P<n>`
/// for the extended ones.
pub fn work_offset_name(name: &str) -> Result<String, NcError> {
    let upper = name.trim().to_ascii_uppercase();
    let parts: Vec<&str> = upper.split_whitespace().collect();
    match parts.as_slice() {
        [g] if matches!(*g, "G54" | "G55" | "G56" | "G57" | "G58" | "G59") => Some(g.to_string()),
        ["G154", p] if p.len() > 1 && p.starts_with('P') => p[1..]
            .parse::<u32>()
            .ok()
            .filter(|n| (1..=99).contains(n))
            .map(|n| format!("G154 P{n}")),
        _ => None,
    }
    .ok_or_else(|| {
        NcError::Machine(format!(
            "`{name}` is not a work offset — one of G54 to G59, or G154 P1 to P99"
        ))
    })
}

/// A program read and timed for one machine: the run in machine
/// coordinates, and the same motion as a trajectory of the robot's whole
/// joint vector.
#[derive(Debug, Clone)]
pub struct PlannedProgram {
    pub robot: usize,
    pub run: NcRun,
    pub trajectory: JointTrajectory,
    /// What the program, or the block read, ends on.
    pub ends: NcEnd,
}

/// The axes as the interpreter and the timing see them — an [`NcMachine`]
/// resolved against its robot's joints.
#[derive(Debug, Clone)]
pub struct NcAxes {
    pub lower: [f64; 3],
    pub upper: [f64; 3],
    /// Rapid rate of X, Y and Z (m/s).
    pub rapid: [f64; 3],
    pub acceleration: [f64; 3],
    pub max_feed: f64,
    pub junction_deviation: f64,
    pub tool_change_s: f64,
    pub spindle_max_rpm: Option<f64>,
}

/// What a refusal is about.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NcErrorKind {
    /// A word the interpreter does not read.
    Unsupported,
    /// A word that is not a word, or a block that contradicts itself.
    Malformed,
    /// A point past an axis's travel.
    Overtravel,
    /// A work offset or a tool the tables do not hold.
    Setup,
    /// A rate the machine does not have, or a cut with no feed.
    Rate,
}

#[derive(Debug, Error)]
pub enum NcError {
    #[error("line {line}: {message}")]
    Line {
        line: usize,
        kind: NcErrorKind,
        message: String,
    },
    #[error("program contains no motion")]
    Empty,
    /// The machine or the program is not what was named.
    #[error("{0}")]
    Machine(String),
}

impl NcError {
    /// The source line of a refusal, where it has one.
    pub fn line(&self) -> Option<usize> {
        match self {
            NcError::Line { line, .. } => Some(*line),
            _ => None,
        }
    }
}

fn refuse(line: usize, kind: NcErrorKind, message: impl Into<String>) -> NcError {
    NcError::Line {
        line,
        kind,
        message: message.into(),
    }
}

/// Something the program switches that is not an axis.
#[derive(Debug, Clone, PartialEq)]
pub enum NcEvent {
    Spindle {
        rpm: f64,
        clockwise: bool,
    },
    SpindleStop,
    Coolant(bool),
    /// The tool now in the spindle.
    Tool(u32),
    /// A machine function was called (an M code of the machine's table).
    Function(u32),
    /// `M30` / `M2`.
    End,
}

/// Why the axes stand still.
#[derive(Debug, Clone, PartialEq)]
pub enum NcPause {
    Dwell,
    ToolChange { tool: u32 },
}

/// One thing the control does, in program order. Positions are machine
/// coordinates (m).
#[derive(Debug, Clone, PartialEq)]
pub enum NcStep {
    /// A straight, synchronised rapid, from rest to rest.
    Rapid {
        to: [f64; 3],
        line: usize,
    },
    /// A straight cutting move at `feed` (m/s); consecutive ones blend.
    Feed {
        to: [f64; 3],
        feed: f64,
        line: usize,
    },
    Pause {
        seconds: f64,
        why: NcPause,
        line: usize,
    },
    Event {
        event: NcEvent,
        line: usize,
    },
    /// A machine function: the control hands `M<code>` to the machine and
    /// waits. Only in a program read whole — read by block, it is what the
    /// block ends on ([`NcPlan::ends`]).
    Function {
        code: u32,
        line: usize,
    },
}

/// What a program — or one block of it — ends on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NcEnd {
    /// A machine function: the control waits for the machine.
    Function { code: u32, line: usize },
    /// `M30` / `M2`: the program is over.
    End { line: usize },
    /// `M99`: the program starts over from its first line.
    Repeat { line: usize },
    /// The text ran out.
    Text,
}

impl NcEnd {
    /// The source line, where there is one.
    pub fn line(&self) -> Option<usize> {
        match self {
            NcEnd::Function { line, .. } | NcEnd::End { line } | NcEnd::Repeat { line } => {
                Some(*line)
            }
            NcEnd::Text => None,
        }
    }
}

/// A program as moves, before timing.
#[derive(Debug, Clone)]
pub struct NcPlan {
    pub steps: Vec<NcStep>,
    /// Notes that change nothing the axes do (a feed clamped to the
    /// machine's maximum).
    pub warnings: Vec<String>,
    /// Where the axes stand when the program ends.
    pub end: [f64; 3],
    /// What it ends on.
    pub ends: NcEnd,
}

/// One block of a program: what the control runs on its own between two
/// machine functions.
#[derive(Debug, Clone, PartialEq)]
pub struct NcBlock {
    /// The first and the last source line of what it does; `None` when it
    /// does nothing (two machine functions in a row).
    pub lines: Option<[usize; 2]>,
    /// Whether the axes move in it.
    pub moves: bool,
    /// Whether the spindle turns at any time in it.
    pub spindle: bool,
    /// Where the axes stand when it ends, machine coordinates.
    pub at: [f64; 3],
    pub ends: NcEnd,
}

/// The blocks of a program read whole, in order: the last one ends on what
/// the program ends on, every other on a machine function.
pub fn outline(plan: &NcPlan, start: [f64; 3]) -> Vec<NcBlock> {
    let mut blocks = Vec::new();
    let mut spinning = false;
    let mut at = start;
    let open = |spinning: bool, at: [f64; 3]| NcBlock {
        lines: None,
        moves: false,
        spindle: spinning,
        at,
        ends: NcEnd::Text,
    };
    let mut block = open(spinning, at);
    for step in &plan.steps {
        let line = match step {
            NcStep::Rapid { to, line } | NcStep::Feed { to, line, .. } => {
                at = *to;
                block.moves = true;
                *line
            }
            NcStep::Pause { line, .. } => *line,
            NcStep::Event { event, line } => {
                match event {
                    NcEvent::Spindle { .. } => {
                        spinning = true;
                        block.spindle = true;
                    }
                    NcEvent::SpindleStop => spinning = false,
                    _ => {}
                }
                *line
            }
            NcStep::Function { code, line } => {
                block.at = at;
                block.ends = NcEnd::Function {
                    code: *code,
                    line: *line,
                };
                blocks.push(std::mem::replace(&mut block, open(spinning, at)));
                continue;
            }
        };
        block.lines = Some(match block.lines {
            Some([first, _]) => [first, line],
            None => [line, line],
        });
    }
    block.at = at;
    block.ends = plan.ends;
    blocks.push(block);
    blocks
}

/// The name a run goes by on its machine's track: the program's, and the
/// block's number where it ran by block.
pub fn run_name(program: &str, block: Option<usize>) -> String {
    match block {
        Some(k) => format!("{program}#{k}"),
        None => program.to_string(),
    }
}

/// A run's name taken apart again: the program, and the block if it names
/// one. (`is_program` says which names are programs — one may hold a `#`.)
pub fn parse_run_name(
    name: &str,
    is_program: impl Fn(&str) -> bool,
) -> Option<(&str, Option<usize>)> {
    if is_program(name) {
        return Some((name, None));
    }
    let (program, block) = name.rsplit_once('#')?;
    let block = block.parse().ok()?;
    is_program(program).then_some((program, Some(block)))
}

#[derive(Debug, Clone)]
pub struct NcOptions {
    /// Maximum chord-to-arc deviation when tessellating arcs (m).
    pub chord_tol: f64,
    /// How far above the last peck a `G83` returns at rapid (m) — Haas
    /// setting 22.
    pub peck_clearance: f64,
    /// How far a `G73` backs off between pecks (m) — Haas setting 52.
    pub chip_break: f64,
    /// Read only this block (0 = up to the first machine function): what
    /// comes before it sets the modal state and moves nothing, and the
    /// block starts from where the axes stand. `None` reads the program
    /// whole, its machine functions as steps that take no time.
    pub block: Option<usize>,
}

impl Default for NcOptions {
    fn default() -> Self {
        NcOptions {
            chord_tol: 2e-5,
            peck_clearance: 1.27e-3,
            chip_break: 0.5e-3,
            block: None,
        }
    }
}

#[derive(Clone, Copy, PartialEq)]
enum Cycle {
    Drill,
    DrillDwell,
    Peck,
    ChipBreak,
}

#[derive(Clone, Copy, PartialEq)]
enum Motion {
    Rapid,
    Linear,
    ArcCw,
    ArcCcw,
    Cycle(Cycle),
}

const AXES: [char; 3] = ['X', 'Y', 'Z'];

struct Control<'a> {
    axes: &'a NcAxes,
    offsets: &'a BTreeMap<String, [f64; 3]>,
    options: &'a NcOptions,
    motion: Motion,
    plane: Plane,
    /// Coordinate scale to metres (mm: 1e-3, inch: 0.0254).
    scale: f64,
    absolute: bool,
    /// Feed in m/s.
    feed: Option<f64>,
    offset_name: String,
    /// Tool length in effect (`G43`), zero under `G49`.
    tool_length: f64,
    /// The tool the last `T` word selected, and the one in the spindle.
    selected: Option<u32>,
    in_spindle: Option<u32>,
    rpm: Option<f64>,
    /// Machine position.
    at: Vector3<f64>,
    /// `G98`: a cycle returns to the level it was called from; `G99`: to R.
    return_initial: bool,
    /// The sticky words of the active canned cycle, in work coordinates.
    cycle_z: Option<f64>,
    cycle_r: Option<f64>,
    cycle_q: Option<f64>,
    cycle_p: f64,
    cycle_initial: f64,
    /// Whether what is read now is recorded: `false` while the text before
    /// the block asked for is walked for its modal state alone.
    live: bool,
    steps: Vec<NcStep>,
    warnings: Vec<String>,
}

impl Control<'_> {
    /// The work zero in machine coordinates, tool length included in Z.
    fn datum(&self, line: usize) -> Result<Vector3<f64>, NcError> {
        let offset = self.offsets.get(&self.offset_name).ok_or_else(|| {
            let known: Vec<&str> = self.offsets.keys().map(String::as_str).collect();
            refuse(
                line,
                NcErrorKind::Setup,
                format!(
                    "work offset {} is not set (the machine holds: {})",
                    self.offset_name,
                    if known.is_empty() {
                        "none".to_string()
                    } else {
                        known.join(", ")
                    }
                ),
            )
        })?;
        Ok(Vector3::new(
            offset[0],
            offset[1],
            offset[2] + self.tool_length,
        ))
    }

    fn check_travel(&self, p: &Vector3<f64>, line: usize) -> Result<(), NcError> {
        for i in 0..3 {
            if p[i] < self.axes.lower[i] - 1e-7 || p[i] > self.axes.upper[i] + 1e-7 {
                return Err(refuse(
                    line,
                    NcErrorKind::Overtravel,
                    format!(
                        "{axis} {at:.3} mm is past the {axis} travel ({lo:.3} to {hi:.3} mm, \
                         machine coordinates)",
                        axis = AXES[i],
                        at = p[i] * 1e3,
                        lo = self.axes.lower[i] * 1e3,
                        hi = self.axes.upper[i] * 1e3,
                    ),
                ));
            }
        }
        Ok(())
    }

    fn rapid(&mut self, to: Vector3<f64>, line: usize) -> Result<(), NcError> {
        if self.live {
            self.check_travel(&to, line)?;
            if (to - self.at).norm() > 1e-9 {
                self.steps.push(NcStep::Rapid {
                    to: to.into(),
                    line,
                });
            }
        }
        self.at = to;
        Ok(())
    }

    fn cutting_feed(&mut self, line: usize) -> Result<f64, NcError> {
        let feed = self.feed.ok_or_else(|| {
            refuse(
                line,
                NcErrorKind::Rate,
                "cutting move before any F word set a feed",
            )
        })?;
        if feed <= 0.0 {
            return Err(refuse(line, NcErrorKind::Rate, "the feed is zero"));
        }
        Ok(feed.min(self.axes.max_feed))
    }

    fn cut(&mut self, to: Vector3<f64>, line: usize) -> Result<(), NcError> {
        if self.live {
            let feed = self.cutting_feed(line)?;
            self.check_travel(&to, line)?;
            if (to - self.at).norm() > 1e-9 {
                self.steps.push(NcStep::Feed {
                    to: to.into(),
                    feed,
                    line,
                });
            }
        }
        self.at = to;
        Ok(())
    }

    fn pause(&mut self, seconds: f64, why: NcPause, line: usize) {
        if self.live && seconds > 0.0 {
            self.steps.push(NcStep::Pause { seconds, why, line });
        }
    }

    fn event(&mut self, event: NcEvent, line: usize) {
        if self.live {
            self.steps.push(NcStep::Event { event, line });
        }
    }

    /// One hole of the active canned cycle at the work XY `xy`.
    fn drill(&mut self, cycle: Cycle, xy: (f64, f64), line: usize) -> Result<(), NcError> {
        let datum = self.datum(line)?;
        let (Some(z), Some(r)) = (self.cycle_z, self.cycle_r) else {
            return Err(refuse(
                line,
                NcErrorKind::Malformed,
                "a canned cycle needs its Z depth and its R plane",
            ));
        };
        if z > r + 1e-9 {
            return Err(refuse(
                line,
                NcErrorKind::Malformed,
                "the cycle's Z depth is above its R plane",
            ));
        }
        let level = |w: f64| Vector3::new(xy.0 + datum.x, xy.1 + datum.y, w + datum.z);
        // Over the hole at the level the tool stands at, then down to R.
        let over = Vector3::new(xy.0 + datum.x, xy.1 + datum.y, self.at.z);
        self.rapid(over, line)?;
        self.rapid(level(r), line)?;
        match cycle {
            Cycle::Drill | Cycle::DrillDwell => {
                self.cut(level(z), line)?;
                if cycle == Cycle::DrillDwell {
                    self.pause(self.cycle_p, NcPause::Dwell, line);
                }
            }
            Cycle::Peck | Cycle::ChipBreak => {
                let q = self.cycle_q.filter(|q| *q > 1e-9).ok_or_else(|| {
                    refuse(
                        line,
                        NcErrorKind::Malformed,
                        "a peck cycle needs its Q peck depth",
                    )
                })?;
                let mut depth = r;
                while depth > z + 1e-9 {
                    let next = (depth - q).max(z);
                    self.cut(level(next), line)?;
                    if next > z + 1e-9 {
                        if cycle == Cycle::Peck {
                            // Out to R to clear the chips, back at rapid
                            // to just above the cut.
                            self.rapid(level(r), line)?;
                            self.rapid(level((next + self.options.peck_clearance).min(r)), line)?;
                        } else {
                            self.rapid(level((next + self.options.chip_break).min(r)), line)?;
                        }
                    }
                    depth = next;
                }
            }
        }
        let out = if self.return_initial {
            self.cycle_initial.max(r)
        } else {
            r
        };
        self.rapid(level(out), line)
    }
}

/// Whether a word carries a decimal point (`P1.` is seconds, `P500` is
/// milliseconds on a Haas).
fn has_point(word: &str) -> bool {
    word.contains('.')
}

/// Walks `text` as the control would, from the axes standing at `start`
/// (machine coordinates), and returns the moves it makes — of the whole
/// program, or of the one block [`NcOptions::block`] names. `functions` is
/// the machine's M-code table: the codes the control hands over.
pub fn interpret(
    text: &str,
    axes: &NcAxes,
    work_offsets: &BTreeMap<String, [f64; 3]>,
    tools: &BTreeMap<u32, NcTool>,
    functions: &BTreeMap<u32, String>,
    start: [f64; 3],
    options: &NcOptions,
) -> Result<NcPlan, NcError> {
    let mut c = Control {
        axes,
        offsets: work_offsets,
        options,
        motion: Motion::Rapid,
        plane: Plane::Xy,
        scale: 1e-3,
        absolute: true,
        feed: None,
        offset_name: "G54".to_string(),
        tool_length: 0.0,
        selected: None,
        in_spindle: None,
        rpm: None,
        at: Vector3::from(start),
        return_initial: true,
        cycle_z: None,
        cycle_r: None,
        cycle_q: None,
        cycle_p: 0.0,
        cycle_initial: 0.0,
        live: options.block.is_none_or(|k| k == 0),
        steps: Vec::new(),
        warnings: Vec::new(),
    };
    let mut feed_clamped = false;
    // The block being read, and what the reading ended on.
    let mut block = 0usize;
    let mut ended = NcEnd::Text;

    'lines: for (lineno, raw) in text.lines().enumerate() {
        let line = lineno + 1;
        // A leading `/` is block delete; the switch is off, so the block runs.
        let clean = strip_comments(raw.trim_start().trim_start_matches('/'));
        let words = lex_words(&clean).map_err(|word| {
            refuse(
                line,
                NcErrorKind::Malformed,
                format!("malformed word `{word}`"),
            )
        })?;
        if words.is_empty() {
            continue;
        }

        // ---- gather the block -------------------------------------------
        let mut gcodes: Vec<f64> = Vec::new();
        let mut mcodes: Vec<f64> = Vec::new();
        let mut xyz: [Option<f64>; 3] = [None; 3];
        let mut ijk: [Option<f64>; 3] = [None; 3];
        let (mut r_word, mut q_word, mut h_word, mut l_word) = (None, None, None, None);
        let mut p_word: Option<(f64, bool)> = None;
        let (mut f_word, mut s_word, mut t_word) = (None, None, None);
        for (letter, value, word) in &words {
            match letter {
                'N' | 'O' | 'D' | 'E' => {}
                'G' => gcodes.push(*value),
                'M' => mcodes.push(*value),
                'X' | 'Y' | 'Z' => xyz[(*letter as u8 - b'X') as usize] = Some(*value),
                'I' | 'J' | 'K' => ijk[(*letter as u8 - b'I') as usize] = Some(*value),
                'R' => r_word = Some(*value),
                'Q' => q_word = Some(*value),
                'H' => h_word = Some(*value),
                'L' => l_word = Some(*value),
                'P' => p_word = Some((*value, has_point(word))),
                'F' => f_word = Some(*value),
                'S' => s_word = Some(*value),
                'T' => t_word = Some(*value),
                _ => {
                    return Err(refuse(
                        line,
                        NcErrorKind::Unsupported,
                        format!("the word `{word}` is not read"),
                    ));
                }
            }
        }
        if l_word.is_some_and(|l| !code_eq(l, 1.0)) {
            return Err(refuse(
                line,
                NcErrorKind::Unsupported,
                "a repeat count (L) is not supported",
            ));
        }

        // ---- the modal words --------------------------------------------
        let (mut dwell, mut home, mut machine_coords, mut length_on) = (false, false, false, false);
        let mut motion_named = false;
        for &g in &gcodes {
            let is = |code: f64| code_eq(g, code);
            if is(0.0) {
                (c.motion, motion_named) = (Motion::Rapid, true);
            } else if is(1.0) {
                (c.motion, motion_named) = (Motion::Linear, true);
            } else if is(2.0) {
                (c.motion, motion_named) = (Motion::ArcCw, true);
            } else if is(3.0) {
                (c.motion, motion_named) = (Motion::ArcCcw, true);
            } else if is(4.0) {
                dwell = true;
            } else if is(17.0) {
                c.plane = Plane::Xy;
            } else if is(18.0) {
                c.plane = Plane::Zx;
            } else if is(19.0) {
                c.plane = Plane::Yz;
            } else if is(20.0) {
                c.scale = 0.0254;
            } else if is(21.0) {
                c.scale = 1e-3;
            } else if is(28.0) {
                home = true;
            } else if is(40.0) || is(64.0) || is(94.0) || is(103.0) || is(187.0) {
                // Compensation off, exact stop off, feed per minute, and the
                // look-ahead and smoothness settings: what is read here anyway.
            } else if is(43.0) {
                length_on = true;
            } else if is(49.0) {
                c.tool_length = 0.0;
            } else if is(53.0) {
                machine_coords = true;
            } else if (54..=59).any(|code| is(code as f64)) {
                c.offset_name = format!("G{}", g.round() as i64);
            } else if is(154.0) {
                let Some((p, _)) = p_word.take() else {
                    return Err(refuse(
                        line,
                        NcErrorKind::Malformed,
                        "G154 needs its P number",
                    ));
                };
                c.offset_name = format!("G154 P{}", p.round() as i64);
            } else if is(73.0) {
                (c.motion, motion_named) = (Motion::Cycle(Cycle::ChipBreak), true);
            } else if is(80.0) {
                if matches!(c.motion, Motion::Cycle(_)) {
                    c.motion = Motion::Rapid;
                }
                (c.cycle_z, c.cycle_r, c.cycle_q, c.cycle_p) = (None, None, None, 0.0);
            } else if is(81.0) {
                (c.motion, motion_named) = (Motion::Cycle(Cycle::Drill), true);
            } else if is(82.0) {
                (c.motion, motion_named) = (Motion::Cycle(Cycle::DrillDwell), true);
            } else if is(83.0) {
                (c.motion, motion_named) = (Motion::Cycle(Cycle::Peck), true);
            } else if is(90.0) {
                c.absolute = true;
            } else if is(91.0) {
                c.absolute = false;
            } else if is(98.0) {
                c.return_initial = true;
            } else if is(99.0) {
                c.return_initial = false;
            } else if is(41.0) || is(42.0) {
                return Err(refuse(
                    line,
                    NcErrorKind::Unsupported,
                    format!(
                        "G{g}: cutter radius compensation is not supported — post the \
                         toolpath pre-compensated"
                    ),
                ));
            } else if is(95.0) {
                return Err(refuse(
                    line,
                    NcErrorKind::Unsupported,
                    "G95: feed-per-revolution is not supported (G94 only)",
                ));
            } else if (74..=89).any(|code| is(code as f64)) {
                return Err(refuse(
                    line,
                    NcErrorKind::Unsupported,
                    format!("G{g}: this canned cycle is not supported (G73, G81, G82 and G83 are)"),
                ));
            } else {
                return Err(refuse(
                    line,
                    NcErrorKind::Unsupported,
                    format!("G{g} is not supported"),
                ));
            }
        }
        if let Some(f) = f_word {
            let feed = f * c.scale / 60.0;
            if feed > axes.max_feed + 1e-12 && c.live && !feed_clamped {
                feed_clamped = true;
                c.warnings.push(format!(
                    "line {line}: F{f} is over the machine's {:.0} mm/min — cut at that",
                    axes.max_feed * 6e4
                ));
            }
            c.feed = Some(feed);
        }
        if let Some(s) = s_word {
            if let Some(max) = axes.spindle_max_rpm {
                if s > max + 1e-9 {
                    return Err(refuse(
                        line,
                        NcErrorKind::Rate,
                        format!("S{s} is over the spindle's {max:.0} rev/min"),
                    ));
                }
            }
            c.rpm = Some(s);
        }
        if let Some(t) = t_word {
            if t < 0.0 || t.fract() != 0.0 {
                return Err(refuse(
                    line,
                    NcErrorKind::Malformed,
                    format!("T{t} is not a tool number"),
                ));
            }
            c.selected = Some(t as u32);
        }
        if length_on || (h_word.is_some() && !gcodes.iter().any(|g| code_eq(*g, 49.0))) {
            let Some(h) = h_word else {
                return Err(refuse(
                    line,
                    NcErrorKind::Malformed,
                    "G43 needs its H number",
                ));
            };
            let number = h.round() as u32;
            let tool = tools.get(&number).ok_or_else(|| {
                let known: Vec<String> = tools.keys().map(|n| format!("H{n}")).collect();
                refuse(
                    line,
                    NcErrorKind::Setup,
                    format!(
                        "tool length H{number} is not in the tool table (it holds: {})",
                        if known.is_empty() {
                            "nothing".to_string()
                        } else {
                            known.join(", ")
                        }
                    ),
                )
            })?;
            c.tool_length = tool.length;
        }

        // ---- M codes that act before the block's motion --------------------
        let (mut ends, mut repeats) = (false, false);
        let mut after: Vec<NcEvent> = Vec::new();
        let mut handed: Vec<u32> = Vec::new();
        for &m in &mcodes {
            let is = |code: f64| code_eq(m, code);
            if is(1.0) {
                // Optional stop: the switch is off.
            } else if is(2.0) || is(30.0) {
                ends = true;
            } else if is(99.0) {
                repeats = true;
            } else if is(3.0) || is(4.0) {
                let rpm = c.rpm.ok_or_else(|| {
                    refuse(
                        line,
                        NcErrorKind::Rate,
                        format!("M{m}: the spindle is started before any S word"),
                    )
                })?;
                c.event(
                    NcEvent::Spindle {
                        rpm,
                        clockwise: is(3.0),
                    },
                    line,
                );
            } else if is(5.0) {
                after.push(NcEvent::SpindleStop);
            } else if is(6.0) {
                let tool = c.selected.ok_or_else(|| {
                    refuse(
                        line,
                        NcErrorKind::Malformed,
                        "M6: a tool change before any T word",
                    )
                })?;
                // The change is made with the head at the top of its travel.
                let top = Vector3::new(c.at.x, c.at.y, axes.upper[2]);
                c.rapid(top, line)?;
                c.event(NcEvent::SpindleStop, line);
                c.event(NcEvent::Coolant(false), line);
                if c.in_spindle != Some(tool) {
                    c.pause(axes.tool_change_s, NcPause::ToolChange { tool }, line);
                }
                c.in_spindle = Some(tool);
                c.event(NcEvent::Tool(tool), line);
            } else if is(8.0) {
                c.event(NcEvent::Coolant(true), line);
            } else if is(9.0) {
                after.push(NcEvent::Coolant(false));
            } else if is(0.0) {
                return Err(refuse(
                    line,
                    NcErrorKind::Unsupported,
                    "M0: a program stop would wait for the operator",
                ));
            } else if m >= 0.0 && m.fract() == 0.0 && functions.contains_key(&(m as u32)) {
                // The machine's, not the control's: handed over once the
                // line's own motion is done.
                handed.push(m as u32);
            } else {
                let table: Vec<String> = functions.keys().map(|code| format!("M{code}")).collect();
                return Err(refuse(
                    line,
                    NcErrorKind::Unsupported,
                    format!(
                        "M{m} is not supported — it is not the control's own and not in the \
                         machine's M-code table ({})",
                        if table.is_empty() {
                            "which is empty".to_string()
                        } else {
                            table.join(", ")
                        }
                    ),
                ));
            }
        }

        // ---- the block's motion ------------------------------------------
        let has_xyz = xyz.iter().any(Option::is_some);
        if dwell {
            // `P` with a decimal point is seconds, without one milliseconds;
            // `X` is seconds.
            let seconds = match (p_word, xyz[0]) {
                (Some((p, true)), _) => p,
                (Some((p, false)), _) => p / 1e3,
                (None, Some(x)) => x,
                (None, None) => {
                    return Err(refuse(line, NcErrorKind::Malformed, "G4 needs its P time"));
                }
            };
            c.pause(seconds, NcPause::Dwell, line);
        } else if home {
            // Through the point named (if any), then the axes named go home;
            // with none named, all of them.
            let mut via = c.at;
            if has_xyz {
                let datum = if c.absolute {
                    Some(c.datum(line)?)
                } else {
                    None
                };
                for i in 0..3 {
                    if let Some(v) = xyz[i] {
                        via[i] = match datum {
                            Some(d) => v * c.scale + d[i],
                            None => c.at[i] + v * c.scale,
                        };
                    }
                }
                c.rapid(via, line)?;
            }
            let mut to = via;
            for i in 0..3 {
                if xyz[i].is_some() || !has_xyz {
                    to[i] = 0.0;
                }
            }
            c.rapid(to, line)?;
        } else if machine_coords {
            if !c.absolute {
                return Err(refuse(
                    line,
                    NcErrorKind::Malformed,
                    "G53 is read in G90 only",
                ));
            }
            let mut to = c.at;
            for i in 0..3 {
                if let Some(v) = xyz[i] {
                    to[i] = v * c.scale;
                }
            }
            match c.motion {
                Motion::Rapid => c.rapid(to, line)?,
                Motion::Linear => c.cut(to, line)?,
                _ => {
                    return Err(refuse(
                        line,
                        NcErrorKind::Malformed,
                        "G53 moves in G0 or G1 only",
                    ));
                }
            }
        } else if let Motion::Cycle(cycle) = c.motion {
            if !c.absolute {
                return Err(refuse(
                    line,
                    NcErrorKind::Unsupported,
                    "canned cycles are read in G90 only",
                ));
            }
            if ijk.iter().any(Option::is_some) {
                return Err(refuse(
                    line,
                    NcErrorKind::Unsupported,
                    "the I, J and K forms of a peck cycle are not supported",
                ));
            }
            let datum = c.datum(line)?;
            if motion_named {
                // The level the cycle is called from is where G98 returns.
                c.cycle_initial = c.at.z - datum.z;
            }
            if let Some(z) = xyz[2] {
                c.cycle_z = Some(z * c.scale);
            }
            if let Some(r) = r_word {
                c.cycle_r = Some(r * c.scale);
            }
            if let Some(q) = q_word {
                c.cycle_q = Some(q * c.scale);
            }
            if let Some((p, point)) = p_word {
                c.cycle_p = if point { p } else { p / 1e3 };
            }
            if motion_named || xyz[0].is_some() || xyz[1].is_some() {
                let x = xyz[0].map_or(c.at.x - datum.x, |v| v * c.scale);
                let y = xyz[1].map_or(c.at.y - datum.y, |v| v * c.scale);
                c.drill(cycle, (x, y), line)?;
            }
        } else if has_xyz || ijk.iter().any(Option::is_some) {
            let mut to = c.at;
            if has_xyz {
                let datum = if c.absolute {
                    Some(c.datum(line)?)
                } else {
                    None
                };
                for i in 0..3 {
                    if let Some(v) = xyz[i] {
                        to[i] = match datum {
                            Some(d) => v * c.scale + d[i],
                            None => c.at[i] + v * c.scale,
                        };
                    }
                }
            }
            match c.motion {
                Motion::Rapid => c.rapid(to, line)?,
                Motion::Linear => c.cut(to, line)?,
                Motion::ArcCw | Motion::ArcCcw => {
                    let cw = c.motion == Motion::ArcCw;
                    let (ia, ib, _) = c.plane.axes();
                    let center = if ijk.iter().any(Option::is_some) {
                        (
                            c.at[ia] + ijk[ia].unwrap_or(0.0) * c.scale,
                            c.at[ib] + ijk[ib].unwrap_or(0.0) * c.scale,
                        )
                    } else if let Some(r) = r_word {
                        let r = r * c.scale;
                        arc_center_from_radius((c.at[ia], c.at[ib]), (to[ia], to[ib]), r, cw)
                            .ok_or_else(|| {
                                refuse(
                                    line,
                                    NcErrorKind::Malformed,
                                    format!(
                                        "arc radius {:.3} mm is shorter than half the chord",
                                        r.abs() * 1e3
                                    ),
                                )
                            })?
                    } else {
                        return Err(refuse(
                            line,
                            NcErrorKind::Malformed,
                            "arc has neither IJK center offsets nor R",
                        ));
                    };
                    for p in arc_points(c.at, to, center, cw, c.plane, options.chord_tol) {
                        c.cut(p, line)?;
                    }
                }
                Motion::Cycle(_) => unreachable!("handled above"),
            }
        }

        for event in after {
            c.event(event, line);
        }
        for code in handed {
            match options.block {
                // Read whole: the function is a step, and takes no time.
                None => c.steps.push(NcStep::Function { code, line }),
                Some(k) if block == k => {
                    ended = NcEnd::Function { code, line };
                    break 'lines;
                }
                Some(k) => {
                    // The next block begins — the one asked for, from
                    // where the axes really stand.
                    block += 1;
                    if block == k {
                        c.at = Vector3::from(start);
                        c.live = true;
                    }
                }
            }
        }
        if ends {
            c.event(NcEvent::SpindleStop, line);
            c.event(NcEvent::Coolant(false), line);
            c.event(NcEvent::End, line);
            ended = NcEnd::End { line };
            break 'lines;
        }
        if repeats {
            ended = NcEnd::Repeat { line };
            break 'lines;
        }
    }

    match options.block {
        Some(k) if block < k => {
            return Err(NcError::Machine(format!(
                "block {k} of a program that has {} (it calls {block} machine function{} before \
                 it ends)",
                block + 1,
                if block == 1 { "" } else { "s" }
            )));
        }
        Some(_) => {}
        None => {
            // (A program that only works the machine — opens a door,
            // calls the cell — moves no axis and is a program all the same.)
            if !c.steps.iter().any(|s| {
                matches!(
                    s,
                    NcStep::Rapid { .. } | NcStep::Feed { .. } | NcStep::Function { .. }
                )
            }) {
                return Err(NcError::Empty);
            }
        }
    }
    Ok(NcPlan {
        steps: c.steps,
        warnings: c.warnings,
        end: c.at.into(),
        ends: ended,
    })
}

// --------------------------------------------------------------- timing

/// The stretch of the run one source line drives.
///
/// A line may hold both a rapid and a feed (a drilling cycle does); which
/// stretches are at rapid is [`NcRun::rapids`].
#[derive(Debug, Clone, PartialEq)]
pub struct NcSpan {
    pub start: f64,
    pub end: f64,
    pub line: usize,
}

#[derive(Debug, Clone, PartialEq)]
pub struct NcTimedEvent {
    pub t: f64,
    pub line: usize,
    pub event: NcEvent,
}

/// A tool change in a timed run: the stretch the axes stand for it, and
/// the tool that is in the spindle when it is over.
#[derive(Debug, Clone, PartialEq)]
pub struct NcToolChange {
    pub start: f64,
    pub end: f64,
    pub tool: u32,
}

/// The share of a tool change a changer's shuttle takes to run in, and
/// again to run out; its carousel turns in what is left between.
pub const CHANGER_SHUTTLE_SHARE: f64 = 0.3;

impl NcToolChange {
    /// When the changer's shuttle is in — from here the tool that was in
    /// the spindle is its pocket's — and when it starts back out, the
    /// next tool left in the spindle.
    pub fn handover(&self) -> [f64; 2] {
        let run = CHANGER_SHUTTLE_SHARE * (self.end - self.start);
        [self.start + run, self.end - run]
    }
}

/// A program timed: what the axes do, and what happens when.
#[derive(Debug, Clone)]
pub struct NcRun {
    /// X, Y and Z over time, machine coordinates (three columns).
    pub trajectory: JointTrajectory,
    /// Per source line, in order; consecutive stretches of one line merged.
    pub spans: Vec<NcSpan>,
    /// `[start, end]` of every stretch at rapid, in order — where a cutter
    /// that meets material has crashed rather than cut.
    pub rapids: Vec<[f64; 2]>,
    pub events: Vec<NcTimedEvent>,
    pub warnings: Vec<String>,
    /// Seconds spent cutting, at rapid, and standing (dwells, tool changes).
    pub cutting_time: f64,
    pub rapid_time: f64,
    pub pause_time: f64,
    /// Metres cut and metres at rapid.
    pub cutting_distance: f64,
    pub rapid_distance: f64,
    pub tool_changes: usize,
    /// Every tool change the axes stand for, in order.
    pub changes: Vec<NcToolChange>,
    /// The box the axes sweep, machine coordinates.
    pub lower: [f64; 3],
    pub upper: [f64; 3],
}

impl NcRun {
    pub fn duration(&self) -> f64 {
        self.trajectory.duration()
    }

    /// The source line running at `t`.
    pub fn line_at(&self, t: f64) -> Option<usize> {
        self.spans
            .iter()
            .find(|s| t >= s.start - 1e-12 && t <= s.end + 1e-12)
            .map(|s| s.line)
    }

    /// Whether the axes are at rapid at `t` (the ends of a rapid count).
    pub fn rapid_at(&self, t: f64) -> bool {
        self.rapids
            .iter()
            .any(|[start, end]| t >= start - 1e-12 && t <= end + 1e-12)
    }
}

struct Timeline {
    times: Vec<f64>,
    positions: Vec<Vec<f64>>,
    velocities: Vec<Vec<f64>>,
    spans: Vec<NcSpan>,
}

impl Timeline {
    fn now(&self) -> f64 {
        *self.times.last().expect("starts with one waypoint")
    }

    fn push(&mut self, t: f64, p: Vector3<f64>, v: Vector3<f64>) {
        // A waypoint must be later than the last: two phases that meet
        // within a nanosecond are one.
        if t <= self.now() + 1e-9 {
            let last = self.positions.len() - 1;
            self.positions[last] = vec![p.x, p.y, p.z];
            self.velocities[last] = vec![v.x, v.y, v.z];
            return;
        }
        self.times.push(t);
        self.positions.push(vec![p.x, p.y, p.z]);
        self.velocities.push(vec![v.x, v.y, v.z]);
    }

    fn span(&mut self, start: f64, end: f64, line: usize) {
        if end <= start + 1e-12 {
            return;
        }
        match self.spans.last_mut() {
            Some(last) if last.line == line && (last.end - start).abs() < 1e-9 => last.end = end,
            _ => self.spans.push(NcSpan { start, end, line }),
        }
    }
}

/// The greatest speed and acceleration along the unit direction `d` that
/// keep every axis within its own.
fn along(d: &Vector3<f64>, rate: &[f64; 3], accel: &[f64; 3]) -> (f64, f64) {
    let (mut v, mut a) = (f64::INFINITY, f64::INFINITY);
    for i in 0..3 {
        if d[i].abs() > 1e-12 {
            v = v.min(rate[i] / d[i].abs());
            a = a.min(accel[i] / d[i].abs());
        }
    }
    (v, a)
}

/// One straight stretch of a profile: accelerate from `v0` to `peak`,
/// hold it, decelerate to `v1`, over `length`.
#[derive(Clone, Copy)]
struct Trapezoid {
    v0: f64,
    v1: f64,
    peak: f64,
    accel: f64,
    length: f64,
}

impl Trapezoid {
    fn new(length: f64, v0: f64, v1: f64, nominal: f64, accel: f64) -> Self {
        // The peak the length allows, capped at the nominal speed.
        let reachable = ((2.0 * accel * length + v0 * v0 + v1 * v1) / 2.0).sqrt();
        Trapezoid {
            v0,
            v1,
            peak: nominal.min(reachable).max(v0).max(v1),
            accel,
            length,
        }
    }

    fn accel_distance(&self) -> f64 {
        (self.peak * self.peak - self.v0 * self.v0) / (2.0 * self.accel)
    }

    fn decel_distance(&self) -> f64 {
        (self.peak * self.peak - self.v1 * self.v1) / (2.0 * self.accel)
    }

    fn duration(&self) -> f64 {
        self.time_at(self.length)
    }

    /// Time and speed at distance `s` along the stretch.
    fn at(&self, s: f64) -> (f64, f64) {
        let s = s.clamp(0.0, self.length);
        let (da, dd) = (self.accel_distance(), self.decel_distance());
        let cruise = (self.length - da - dd).max(0.0);
        let ta = (self.peak - self.v0) / self.accel;
        if s <= da {
            let v = (self.v0 * self.v0 + 2.0 * self.accel * s).sqrt();
            return ((v - self.v0) / self.accel, v);
        }
        if s <= da + cruise {
            return (ta + (s - da) / self.peak, self.peak);
        }
        let tc = if self.peak > 0.0 {
            cruise / self.peak
        } else {
            0.0
        };
        let left = (self.length - s).max(0.0);
        let v = (self.v1 * self.v1 + 2.0 * self.accel * left).sqrt();
        (ta + tc + (self.peak - v) / self.accel, v)
    }

    fn time_at(&self, s: f64) -> f64 {
        self.at(s).0
    }
}

/// Times one continuous cutting path — `points[0]` is where it starts at
/// rest, `feeds[i]` and `lines[i]` belong to the move into `points[i + 1]`
/// — and appends it to the timeline. Returns the seconds it took.
fn time_cut(
    tl: &mut Timeline,
    points: &[Vector3<f64>],
    feeds: &[f64],
    lines: &[usize],
    axes: &NcAxes,
) -> f64 {
    let n = points.len() - 1;
    let mut dir = Vec::with_capacity(n);
    let mut len = Vec::with_capacity(n);
    let mut nominal = Vec::with_capacity(n);
    let mut accel = Vec::with_capacity(n);
    for i in 0..n {
        let delta = points[i + 1] - points[i];
        let l = delta.norm();
        let d = delta / l;
        let (v, a) = along(&d, &axes.rapid, &axes.acceleration);
        dir.push(d);
        len.push(l);
        nominal.push(feeds[i].min(v));
        accel.push(a);
    }
    // The speed each vertex is passed at: zero at both ends, and at a
    // corner what keeps the path within the junction deviation of it.
    let mut vertex = vec![0.0f64; n + 1];
    let mut turn = vec![0.0f64; n + 1];
    for i in 1..n {
        let cos = (-dir[i - 1].dot(&dir[i])).clamp(-1.0, 1.0);
        let limit = nominal[i - 1].min(nominal[i]);
        vertex[i] = if cos > 0.999_999 {
            0.0
        } else if cos < -0.999_999 {
            limit
        } else {
            let half = (0.5 * (1.0 - cos)).sqrt();
            // The sine of half the angle the path turns through.
            turn[i] = (0.5 * (1.0 + cos)).sqrt();
            let a = accel[i - 1].min(accel[i]);
            (a * axes.junction_deviation * half / (1.0 - half))
                .sqrt()
                .min(limit)
        };
    }
    for i in (1..n).rev() {
        vertex[i] = vertex[i].min((vertex[i + 1].powi(2) + 2.0 * accel[i] * len[i]).sqrt());
    }
    for i in 1..n {
        vertex[i] = vertex[i].min((vertex[i - 1].powi(2) + 2.0 * accel[i - 1] * len[i - 1]).sqrt());
    }
    // How much of each side of a corner the rounding takes. The curve
    // between the two ends of it passes half that length times the sine of
    // the half turn inside the corner, so this keeps it within the
    // deviation — and never takes more than half of either move.
    let mut blend = vec![0.0f64; n + 1];
    for i in 1..n {
        if vertex[i] > 1e-9 && turn[i] > 1e-9 {
            blend[i] = (2.0 * axes.junction_deviation / turn[i])
                .min(0.5 * len[i - 1])
                .min(0.5 * len[i]);
        }
    }
    let begin = tl.now();
    let mut t0 = begin;
    for i in 0..n {
        let profile = Trapezoid::new(len[i], vertex[i], vertex[i + 1], nominal[i], accel[i]);
        let (from, to) = (blend[i], len[i] - blend[i + 1]);
        let mut marks = vec![from];
        for s in [profile.accel_distance(), len[i] - profile.decel_distance()] {
            if s > from + 1e-9 && s < to - 1e-9 {
                marks.push(s);
            }
        }
        if to > from + 1e-9 {
            marks.push(to);
        }
        for s in marks {
            // The first move starts where the timeline already stands.
            if i == 0 && s <= 1e-12 {
                continue;
            }
            let (t, v) = profile.at(s);
            tl.push(t0 + t, points[i] + dir[i] * s, dir[i] * v);
        }
        let t1 = t0 + profile.duration();
        tl.span(t0, t1, lines[i]);
        t0 = t1;
    }
    // The path ends at rest on its last point, whatever the last mark was.
    tl.push(t0, points[n], Vector3::zeros());
    t0 - begin
}

/// Times a plan from the axes standing at `start`.
pub fn time_plan(plan: &NcPlan, axes: &NcAxes, start: [f64; 3]) -> NcRun {
    let origin = Vector3::from(start);
    let mut tl = Timeline {
        times: vec![0.0],
        positions: vec![vec![origin.x, origin.y, origin.z]],
        velocities: vec![vec![0.0; 3]],
        spans: Vec::new(),
    };
    let mut events = Vec::new();
    let mut rapids: Vec<[f64; 2]> = Vec::new();
    let (mut cutting_time, mut rapid_time, mut pause_time) = (0.0, 0.0, 0.0);
    let (mut cutting_distance, mut rapid_distance) = (0.0, 0.0);
    let mut tool_changes = 0;
    let mut changes = Vec::new();
    let (mut lower, mut upper) = (origin, origin);
    let mut at = origin;
    let mut i = 0;
    while i < plan.steps.len() {
        match &plan.steps[i] {
            NcStep::Rapid { to, line } => {
                let to = Vector3::from(*to);
                let delta = to - at;
                let length = delta.norm();
                let d = delta / length;
                let (v, a) = along(&d, &axes.rapid, &axes.acceleration);
                let profile = Trapezoid::new(length, 0.0, 0.0, v, a);
                let t0 = tl.now();
                for s in [profile.accel_distance(), length - profile.decel_distance()] {
                    if s > 1e-9 && s < length - 1e-9 {
                        let (t, speed) = profile.at(s);
                        tl.push(t0 + t, at + d * s, d * speed);
                    }
                }
                let t1 = t0 + profile.duration();
                tl.push(t1, to, Vector3::zeros());
                tl.span(t0, t1, *line);
                match rapids.last_mut() {
                    Some(last) if (last[1] - t0).abs() < 1e-9 => last[1] = t1,
                    _ => rapids.push([t0, t1]),
                }
                rapid_time += t1 - t0;
                rapid_distance += length;
                at = to;
                i += 1;
            }
            NcStep::Feed { .. } => {
                // Every cutting move up to the next thing that is not one.
                let mut points = vec![at];
                let (mut feeds, mut lines) = (Vec::new(), Vec::new());
                while let Some(NcStep::Feed { to, feed, line }) = plan.steps.get(i) {
                    points.push(Vector3::from(*to));
                    feeds.push(*feed);
                    lines.push(*line);
                    i += 1;
                }
                cutting_distance += points.windows(2).map(|w| (w[1] - w[0]).norm()).sum::<f64>();
                cutting_time += time_cut(&mut tl, &points, &feeds, &lines, axes);
                at = *points.last().expect("at least the start");
            }
            NcStep::Pause { seconds, why, line } => {
                let t0 = tl.now();
                tl.push(t0 + seconds, at, Vector3::zeros());
                tl.span(t0, t0 + seconds, *line);
                pause_time += seconds;
                if let NcPause::ToolChange { tool } = why {
                    tool_changes += 1;
                    changes.push(NcToolChange {
                        start: t0,
                        end: t0 + seconds,
                        tool: *tool,
                    });
                }
                i += 1;
            }
            NcStep::Event { event, line } => {
                events.push(NcTimedEvent {
                    t: tl.now(),
                    line: *line,
                    event: event.clone(),
                });
                i += 1;
            }
            NcStep::Function { code, line } => {
                events.push(NcTimedEvent {
                    t: tl.now(),
                    line: *line,
                    event: NcEvent::Function(*code),
                });
                i += 1;
            }
        }
        lower = lower.inf(&at);
        upper = upper.sup(&at);
    }
    // Cutting paths between vertices never leave the box of their vertices,
    // so the swept box is the box of every waypoint.
    for p in &tl.positions {
        let p = Vector3::new(p[0], p[1], p[2]);
        lower = lower.inf(&p);
        upper = upper.sup(&p);
    }
    NcRun {
        trajectory: JointTrajectory {
            times: tl.times,
            positions: tl.positions,
            velocities: tl.velocities,
        },
        spans: tl.spans,
        rapids,
        events,
        warnings: plan.warnings.clone(),
        cutting_time,
        rapid_time,
        pause_time,
        cutting_distance,
        rapid_distance,
        tool_changes,
        changes,
        lower: lower.into(),
        upper: upper.into(),
    }
}

/// The changer's part of a timed run. `trajectory` is the robot's whole
/// joint vector over the run with the changer standing where it stood;
/// through every tool change of `changes` the shuttle (column `shuttle`,
/// `reach` its values at rest and at the spindle) runs in, the carousel
/// (column `carousel`) turns the next tool's pocket to the spindle, and
/// the shuttle runs out — each a stretch of its own, from rest to rest.
/// A change the axes do not stand for (a machine with no tool-to-tool
/// time) leaves the changer where it is.
pub fn work_changer(
    trajectory: &mut JointTrajectory,
    changes: &[NcToolChange],
    changer: &NcChanger,
    shuttle: usize,
    carousel: usize,
    reach: [f64; 2],
) {
    let Some(first) = trajectory.positions.first() else {
        return;
    };
    let mut turn = first[carousel];
    for change in changes {
        if change.end - change.start < 1e-6 {
            continue;
        }
        // The knot the axes come to rest on; the next is where they leave.
        let at = trajectory
            .times
            .partition_point(|t| *t <= change.start + 1e-9)
            .saturating_sub(1);
        let next = changer.pocket_turn(change.tool, turn).unwrap_or(turn);
        let [t_in, t_out] = change.handover();
        let still = vec![0.0; trajectory.positions[at].len()];
        let mut held = trajectory.positions[at].clone();
        held[shuttle] = reach[1];
        held[carousel] = turn;
        let mut turned = held.clone();
        turned[carousel] = next;
        trajectory.times.splice(at + 1..at + 1, [t_in, t_out]);
        trajectory.positions.splice(at + 1..at + 1, [held, turned]);
        trajectory
            .velocities
            .splice(at + 1..at + 1, [still.clone(), still]);
        // From the change's end on the shuttle is at rest again and the
        // carousel stands where it was turned to.
        for q in &mut trajectory.positions[at + 3..] {
            q[shuttle] = reach[0];
            q[carousel] = next;
        }
        turn = next;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A Haas VF-2's axes: 762 x 406 x 508 mm to the negative side of
    /// home, 25.4 m/min rapids, 16.5 m/min of feed.
    fn vf2() -> NcAxes {
        NcAxes {
            lower: [-0.762, -0.406, -0.508],
            upper: [0.0; 3],
            rapid: [25.4 / 60.0; 3],
            acceleration: [5.0; 3],
            max_feed: 16.5 / 60.0,
            junction_deviation: 5e-5,
            tool_change_s: 4.2,
            spindle_max_rpm: Some(8100.0),
        }
    }

    fn tables() -> (BTreeMap<String, [f64; 3]>, BTreeMap<u32, NcTool>) {
        let mut offsets = BTreeMap::new();
        // Work zero 381 / 200 mm into the travels, 400 mm below the head's home.
        offsets.insert("G54".to_string(), [-0.381, -0.200, -0.400]);
        let mut tools = BTreeMap::new();
        tools.insert(
            1,
            NcTool {
                length: 0.100,
                diameter: Some(0.010),
                body: None,
            },
        );
        tools.insert(
            2,
            NcTool {
                length: 0.150,
                diameter: Some(0.005),
                body: None,
            },
        );
        (offsets, tools)
    }

    /// A machine's M-code table: a window, a vise, a relay.
    fn functions() -> BTreeMap<u32, String> {
        [
            (180, "window open"),
            (181, "window closed"),
            (70, "clamp"),
            (21, "call the cell"),
        ]
        .into_iter()
        .map(|(code, what)| (code, what.to_string()))
        .collect()
    }

    fn read(text: &str, start: [f64; 3], block: Option<usize>) -> Result<NcPlan, NcError> {
        let (offsets, tools) = tables();
        interpret(
            text,
            &vf2(),
            &offsets,
            &tools,
            &functions(),
            start,
            &NcOptions {
                block,
                ..NcOptions::default()
            },
        )
    }

    fn run(text: &str) -> Result<(NcPlan, NcRun), NcError> {
        let plan = read(text, [0.0; 3], None)?;
        let timed = time_plan(&plan, &vf2(), [0.0; 3]);
        Ok((plan, timed))
    }

    fn rapids(plan: &NcPlan) -> Vec<[f64; 3]> {
        plan.steps
            .iter()
            .filter_map(|s| match s {
                NcStep::Rapid { to, .. } => Some(*to),
                _ => None,
            })
            .collect()
    }

    fn close(a: [f64; 3], b: [f64; 3]) -> bool {
        a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9)
    }

    #[test]
    fn work_offset_and_tool_length_place_the_program_in_machine_coordinates() {
        // Tool 1 is 100 mm long: its tip at work Z5 is the gauge line at
        // machine Z -400 + 5 + 100.
        let (plan, _) = run("G90 G54\nG0 X10. Y20.\nG43 Z5. H1\n").unwrap();
        let r = rapids(&plan);
        assert!(close(r[0], [-0.371, -0.180, 0.0]), "{:?}", r[0]);
        assert!(close(r[1], [-0.371, -0.180, -0.295]), "{:?}", r[1]);
        // G49 takes the length off again; G53 ignores both tables.
        let (plan, _) = run("G54 G0 G43 Z5. H1\nG49\nG0 Z5.\nG53 G0 Z-10.\n").unwrap();
        let r = rapids(&plan);
        assert!(
            close(r[1], [0.0, 0.0, -0.395]) && close(r[2], [0.0, 0.0, -0.010]),
            "{r:?}"
        );
    }

    #[test]
    fn what_the_tables_do_not_hold_is_refused_by_line() {
        let err = run("G90\nG55\nG0 X0. Y0.\n").unwrap_err();
        assert_eq!(err.line(), Some(3));
        assert!(
            err.to_string()
                .contains("work offset G55 is not set (the machine holds: G54)"),
            "{err}"
        );
        let err = run("G54 G0 X0. Y0.\nG43 Z5. H7\n").unwrap_err();
        assert!(
            err.to_string()
                .starts_with("line 2: tool length H7 is not in the tool table"),
            "{err}"
        );
        let err = run("G54\nG154 P3 G0 X0.\n").unwrap_err();
        assert!(err.to_string().contains("G154 P3 is not set"), "{err}");
    }

    #[test]
    fn a_point_past_the_travel_is_refused_with_the_axis_and_the_line() {
        // Work X400 is machine X +19: past the home end.
        let err = run("G54 G90\nG0 X0. Y0.\nG0 X400.\n").unwrap_err();
        assert!(matches!(
            err,
            NcError::Line {
                line: 3,
                kind: NcErrorKind::Overtravel,
                ..
            }
        ));
        assert!(
            err.to_string()
                .contains("X 19.000 mm is past the X travel (-762.000 to 0.000 mm"),
            "{err}"
        );
        // An arc is checked along its length, not at its ends alone: both
        // ends of this one are inside, and swept the other way round it
        // bulges 4 mm past Y's far end.
        run("G54 G90 G0 X0. Y-180.\nG1 Z-1. F500.\nG2 X60. Y-180. R30.\n").unwrap();
        let err = run("G54 G90 G0 X0. Y-180.\nG1 Z-1. F500.\nG3 X60. Y-180. R30.\n").unwrap_err();
        assert!(matches!(
            err,
            NcError::Line {
                line: 3,
                kind: NcErrorKind::Overtravel,
                ..
            }
        ));
        assert!(
            err.to_string().contains("Y -40") && err.to_string().contains("past the Y travel"),
            "{err}"
        );
    }

    #[test]
    fn words_the_control_does_not_read_are_refused_not_skipped() {
        for (text, needle) in [
            (
                "G54 G0 X0.\nG41 D1 X5.\n",
                "line 2: G41: cutter radius compensation",
            ),
            ("G54 G0 X0.\nM98 P1000\n", "line 2: M98 is not supported"),
            (
                "G54 G0 X0.\nG84 Z-10. R2. F100.\n",
                "line 2: G84: this canned cycle",
            ),
            (
                "G54 G0 X0.\nG65 P9810 X1.\n",
                "line 2: G65 is not supported",
            ),
            (
                "G54 G0 X0.\nG1 X5.\n",
                "line 2: cutting move before any F word",
            ),
            (
                "G54 G0 X0.\nS9000 M3\n",
                "line 2: S9000 is over the spindle's 8100 rev/min",
            ),
            (
                "G54 G0 X0.\nM6\n",
                "line 2: M6: a tool change before any T word",
            ),
            ("G54 G0 X0.\nG1 X1. F#100\n", "line 2: malformed word `F`"),
        ] {
            let err = run(text).unwrap_err();
            assert!(err.to_string().starts_with(needle), "{text:?} gave {err}");
        }
        assert!(matches!(run("G54 G90 G21\nM30\n"), Err(NcError::Empty)));
    }

    #[test]
    fn a_rapid_is_timed_from_rest_to_rest_at_the_axes_rates() {
        // 300 mm along X at 25.4 m/min and 5 m/s²: L/v + v/a.
        let (_, timed) = run("G53 G0 X-300.\n").unwrap();
        let v = 25.4 / 60.0;
        assert!(
            (timed.duration() - (0.300 / v + v / 5.0)).abs() < 1e-9,
            "{}",
            timed.duration()
        );
        assert!((timed.rapid_time - timed.duration()).abs() < 1e-12 && timed.cutting_time == 0.0);
        // Along a diagonal each axis keeps its own rate: the move is as
        // long as its longest axis takes.
        let (_, both) = run("G53 G0 X-300. Y-300.\n").unwrap();
        assert!((both.duration() - timed.duration()).abs() < 1e-9);
        // Halfway through, the axes are halfway — a straight line.
        let mid = both.trajectory.sample(both.duration() / 2.0);
        assert!(
            (mid[0] + 0.150).abs() < 1e-9 && (mid[1] + 0.150).abs() < 1e-9,
            "{mid:?}"
        );
        // Too short to reach the rate: a triangle, 2 sqrt(L/a).
        let (_, short) = run("G53 G0 Z-10.\n").unwrap();
        assert!((short.duration() - 2.0 * (0.010f64 / 5.0).sqrt()).abs() < 1e-9);
    }

    #[test]
    fn a_cut_runs_at_its_feed_and_slows_for_a_corner() {
        // 100 mm at 600 mm/min: 10 s of cruise plus one ramp's worth.
        let straight = "G54 G90 G0 X0. Y0.\nG43 Z5. H1\nG1 Z-2. F600.\nG1 X100.\n";
        let (_, timed) = run(straight).unwrap();
        let cut = timed.spans.iter().find(|s| s.line == 4).unwrap();
        assert!(
            ((cut.end - cut.start) - (10.0 + 0.5 * 0.010 / 5.0)).abs() < 2e-3,
            "{cut:?}"
        );
        // The same length through a right angle takes longer: the corner
        // is passed at the junction speed, not at the feed.
        let bent = "G54 G90 G0 X0. Y0.\nG43 Z5. H1\nG1 Z-2. F6000.\nG1 X50.\nG1 Y50.\n";
        let line = "G54 G90 G0 X0. Y0.\nG43 Z5. H1\nG1 Z-2. F6000.\nG1 X50.\nG1 X100.\n";
        let (_, a) = run(bent).unwrap();
        let (_, b) = run(line).unwrap();
        assert!(
            a.cutting_time > b.cutting_time + 0.01,
            "{} vs {}",
            a.cutting_time,
            b.cutting_time
        );
        // And the corner is rounded within the junction deviation: the
        // path never strays further than that from the programmed one —
        // the plunge, the leg along X, the leg along Y — and passes the
        // corner at about that distance.
        let corner = Vector3::new(-0.331, -0.200, -0.302);
        let cut = a.spans.iter().find(|s| s.line == 3).unwrap().start;
        let (mut nearest, mut t) = (f64::INFINITY, cut);
        while t < a.duration() {
            let p = a.trajectory.sample(t);
            let p = Vector3::new(p[0], p[1], p[2]);
            nearest = nearest.min((p - corner).norm());
            let off_plunge = (p.x + 0.381).abs().max((p.y + 0.200).abs());
            let off_x_leg = (p.y + 0.200).abs().max((p.z + 0.302).abs());
            let off_y_leg = (p.x + 0.331).abs().max((p.z + 0.302).abs());
            assert!(
                off_plunge.min(off_x_leg).min(off_y_leg) < 5.5e-5,
                "strayed at t={t}: {p:?}"
            );
            t += 0.0005;
        }
        assert!(
            nearest > 3e-5 && nearest < 5.5e-5,
            "the corner is passed at {nearest}"
        );
    }

    #[test]
    fn a_feed_over_the_machines_is_clamped_and_said() {
        let (plan, timed) = run("G54 G90 G0 X0. Y0.\nG1 X100. F30000.\n").unwrap();
        assert_eq!(plan.warnings.len(), 1);
        assert!(plan.warnings[0].contains("F30000 is over the machine's 16500 mm/min"));
        let v = 16.5 / 60.0;
        assert!(
            (timed.cutting_time - (0.100 / v + v / 5.0)).abs() < 1e-6,
            "{}",
            timed.cutting_time
        );
    }

    #[test]
    fn a_changer_turns_the_short_way_to_a_pocket_and_tool_zero_has_none() {
        let changer = NcChanger {
            shuttle: "shuttle".into(),
            carousel: "carousel".into(),
            pockets: 20,
        };
        let step = std::f64::consts::TAU / 20.0;
        let near = |tool: u32, from: f64, want: f64| {
            let turn = changer.pocket_turn(tool, from).unwrap();
            assert!(
                (turn - want).abs() < 1e-12,
                "T{tool} from {from}: {turn} != {want}"
            );
        };
        // Pocket 1 is at the spindle's side with the joint at 0; pocket n
        // comes to it by turning back n - 1 pockets.
        near(1, 0.0, 0.0);
        near(2, 0.0, -step);
        near(6, 0.0, -5.0 * step);
        // Nineteen pockets back is one pocket on: the short way.
        near(20, 0.0, step);
        // From where the carousel stands, never more than half a turn, and
        // turn upon turn the joint's value keeps the count.
        near(1, -step, 0.0);
        near(1, -19.0 * step, -20.0 * step);
        near(
            3,
            7.0 * std::f64::consts::TAU,
            7.0 * std::f64::consts::TAU - 2.0 * step,
        );
        // A tool past the last pocket shares one (the table is not checked
        // against the carousel), and tool 0 — an empty spindle — has none.
        near(21, 0.0, 0.0);
        assert_eq!(changer.pocket_turn(0, 1.0), None);
    }

    #[test]
    fn the_changer_is_worked_through_each_tool_change_and_stands_between() {
        let changer = NcChanger {
            shuttle: "shuttle".into(),
            carousel: "carousel".into(),
            pockets: 4,
        };
        // Five columns: X, Y, Z, the shuttle, the carousel. The axes move,
        // stand for a change to tool 2 (10 s to 14 s), move, stand for a
        // change to tool 4 (20 s to 24 s), and move again.
        let knot = |x: f64| vec![x, 0.0, 0.0, 0.0, 0.0];
        let mut trajectory = JointTrajectory {
            times: vec![0.0, 10.0, 14.0, 20.0, 24.0, 30.0],
            positions: [0.0, -0.1, -0.1, -0.2, -0.2, -0.3].map(knot).to_vec(),
            velocities: vec![vec![0.0; 5]; 6],
        };
        let before = trajectory.clone();
        let changes = [
            NcToolChange {
                start: 10.0,
                end: 14.0,
                tool: 2,
            },
            NcToolChange {
                start: 20.0,
                end: 24.0,
                tool: 4,
            },
        ];
        assert_eq!(changes[0].handover(), [11.2, 12.8]);
        work_changer(&mut trajectory, &changes, &changer, 3, 4, [0.0, 0.25]);
        // Two knots a change, and the clock is what it was.
        assert_eq!(
            trajectory.times,
            [0.0, 10.0, 11.2, 12.8, 14.0, 20.0, 21.2, 22.8, 24.0, 30.0]
        );
        let quarter = std::f64::consts::FRAC_PI_2;
        let at = |t: f64| trajectory.sample(t);
        let near = |a: f64, b: f64| assert!((a - b).abs() < 1e-9, "{a} != {b}");
        for (t, shuttle, carousel) in [
            (5.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            (10.6, 0.125, 0.0),           // half way in
            (11.2, 0.25, 0.0),            // in: the tool is the pocket's
            (12.0, 0.25, -quarter / 2.0), // turning to pocket 2
            (12.8, 0.25, -quarter),       // tool 2 at the spindle
            (13.4, 0.125, -quarter),      // half way out
            (14.0, 0.0, -quarter),
            (17.0, 0.0, -quarter), // standing while the axes cut
            (21.2, 0.25, -quarter),
            (22.8, 0.25, quarter), // pocket 4: half a turn, either way round
            (24.0, 0.0, quarter),
            (30.0, 0.0, quarter),
        ] {
            near(at(t)[3], shuttle);
            near(at(t)[4], carousel);
        }
        // The axes are where they were at every instant.
        for t in [0.0, 3.0, 10.0, 12.0, 14.0, 17.0, 22.0, 27.0, 30.0] {
            near(at(t)[0], before.sample(t)[0]);
            near(at(t)[2], before.sample(t)[2]);
        }
        // A change the axes do not stand for moves nothing, and tool 0
        // leaves the carousel where it is.
        let mut instant = before.clone();
        work_changer(
            &mut instant,
            &[NcToolChange {
                start: 10.0,
                end: 10.0,
                tool: 3,
            }],
            &changer,
            3,
            4,
            [0.0, 0.25],
        );
        assert_eq!(instant.positions, before.positions);
        let mut unload = before.clone();
        work_changer(
            &mut unload,
            &[NcToolChange {
                start: 10.0,
                end: 14.0,
                tool: 0,
            }],
            &changer,
            3,
            4,
            [0.0, 0.25],
        );
        near(unload.sample(12.0)[3], 0.25);
        assert!(unload.positions.iter().all(|q| q[4] == 0.0));
    }

    #[test]
    fn a_tool_change_goes_to_the_top_and_takes_its_time() {
        let text = "G54 G90 G0 X0. Y0.\nG43 Z5. H1\nT2 M6\nG0 G43 Z5. H2\nT2 M6\n";
        let (plan, timed) = run(text).unwrap();
        // Z to machine zero first, then the pause; the same tool again is
        // no change.
        let at = plan
            .steps
            .iter()
            .position(|s| {
                matches!(
                    s,
                    NcStep::Pause {
                        why: NcPause::ToolChange { tool: 2 },
                        ..
                    }
                )
            })
            .unwrap();
        assert!(
            matches!(&plan.steps[at - 3], NcStep::Rapid { to, line: 3 } if close(*to, [-0.381, -0.200, 0.0]))
        );
        assert_eq!((timed.tool_changes, timed.pause_time), (1, 4.2));
        // The stretch the axes stand for it, and the tool it ends with.
        let [change] = timed.changes.as_slice() else {
            panic!("one change: {:?}", timed.changes);
        };
        assert_eq!(change.tool, 2);
        assert!((change.end - change.start - 4.2).abs() < 1e-12);
        assert!(timed
            .events
            .iter()
            .any(|e| e.event == NcEvent::Tool(2) && (e.t - change.end).abs() < 1e-12));
        assert!(timed
            .events
            .iter()
            .any(|e| e.event == NcEvent::Tool(2) && e.line == 3));
        // Tool 2 is 50 mm longer: the same work Z is 50 mm higher on the
        // machine (tool 1 stood at -295). The last block goes back to the top.
        let r = rapids(&plan);
        assert!(close(r[r.len() - 2], [-0.381, -0.200, -0.245]), "{r:?}");
        assert!(close(*r.last().unwrap(), [-0.381, -0.200, 0.0]), "{r:?}");
    }

    #[test]
    fn dwell_reads_seconds_with_a_point_and_milliseconds_without() {
        let (_, a) = run("G53 G0 Z-10.\nG4 P1.5\n").unwrap();
        let (_, b) = run("G53 G0 Z-10.\nG4 P500\n").unwrap();
        assert!((a.pause_time - 1.5).abs() < 1e-12 && (b.pause_time - 0.5).abs() < 1e-12);
        // The axes stand through it.
        let end = a.trajectory.sample(a.duration() - 0.7);
        assert!((end[2] + 0.010).abs() < 1e-12);
        assert_eq!(a.line_at(a.duration() - 0.7), Some(2));
    }

    #[test]
    fn g28_goes_home_through_the_point_it_names() {
        // The usual retract: `G28 G91 Z0.` is "from here, straight up".
        let (plan, _) = run("G54 G90 G0 X10. Y10.\nG43 Z5. H1\nG28 G91 Z0.\nG90\nG28\n").unwrap();
        let r = rapids(&plan);
        assert!(close(r[2], [-0.371, -0.190, 0.0]), "{r:?}");
        assert!(close(r[3], [0.0, 0.0, 0.0]), "{r:?}");
    }

    #[test]
    fn drilling_cycles_plunge_at_every_position_until_g80() {
        let text = "G54 G90 G0 X0. Y0.\nG43 Z10. H2\nG98 G81 Z-6. R2. F200.\nX20.\nG80\nG0 X40.\n";
        let (plan, _) = run(text).unwrap();
        let cuts: Vec<[f64; 3]> = plan
            .steps
            .iter()
            .filter_map(|s| match s {
                NcStep::Feed { to, .. } => Some(*to),
                _ => None,
            })
            .collect();
        // Two holes, each to work Z-6 (machine -400 - 6 + 150), and back to
        // the level the cycle was called from (G98).
        assert_eq!(cuts.len(), 2);
        assert!(
            close(cuts[0], [-0.381, -0.200, -0.256]) && close(cuts[1], [-0.361, -0.200, -0.256]),
            "{cuts:?}"
        );
        let r = rapids(&plan);
        assert!(close(r[r.len() - 2], [-0.361, -0.200, -0.240]), "{r:?}");
        // G99 stops at R; a peck cycle clears the chips between pecks.
        let peck = "G54 G90 G0 X0. Y0.\nG43 Z10. H2\nG99 G83 Z-6. R2. Q3. F200.\nG80\n";
        let (plan, _) = run(peck).unwrap();
        let depths: Vec<f64> = plan
            .steps
            .iter()
            .filter_map(|s| match s {
                NcStep::Feed { to, .. } => Some(((to[2] + 0.250) * 1e6).round() / 1e3),
                _ => None,
            })
            .collect();
        assert_eq!(depths, vec![-1.0, -4.0, -6.0]);
        let last = *rapids(&plan).last().unwrap();
        assert!(close(last, [-0.381, -0.200, -0.248]), "{last:?}");
        let err = run("G54 G90 G0 X0. Y0.\nG83 Z-6. R2. F200.\n").unwrap_err();
        assert!(err.to_string().contains("needs its Q peck depth"), "{err}");
    }

    /// A cell program: cut, park, call the cell through a relay, clamp,
    /// and start over.
    const CELL: &str = "G90 G21 G54\n\
        T1 M6\n\
        S3000 M3\n\
        G0 X0. Y0.\n\
        G43 Z5. H1\n\
        G1 Z-1. F300.\n\
        G0 Z5.\n\
        M5\n\
        G53 G0 Z0.\n\
        G53 G0 X-700. Y-100. M180\n\
        M21\n\
        M70\n\
        G0 X10. M181\n\
        M99\n";

    #[test]
    fn machine_functions_split_a_program_into_blocks() {
        let plan = read(CELL, [0.0; 3], None).unwrap();
        // Read whole, each is a step in its place — after its line's own
        // motion — and the program ends on its M99.
        let handed: Vec<(u32, usize)> = plan
            .steps
            .iter()
            .filter_map(|s| match s {
                NcStep::Function { code, line } => Some((*code, *line)),
                _ => None,
            })
            .collect();
        assert_eq!(handed, vec![(180, 10), (21, 11), (70, 12), (181, 13)]);
        assert_eq!(plan.ends, NcEnd::Repeat { line: 14 });
        let parked = plan
            .steps
            .iter()
            .position(|s| matches!(s, NcStep::Rapid { line: 10, .. }))
            .unwrap();
        assert!(matches!(
            plan.steps[parked + 1],
            NcStep::Function { code: 180, .. }
        ));

        let blocks = outline(&plan, [0.0; 3]);
        assert_eq!(blocks.len(), 5);
        // The cut: lines 2 to 10, the spindle on in it, the table parked
        // as it ends.
        assert_eq!(blocks[0].lines, Some([2, 10]));
        assert!(blocks[0].moves && blocks[0].spindle);
        assert!(close(blocks[0].at, [-0.700, -0.100, 0.0]));
        assert_eq!(
            blocks[0].ends,
            NcEnd::Function {
                code: 180,
                line: 10
            }
        );
        // Between two functions in a row the control does nothing.
        for k in [1, 2] {
            assert_eq!(blocks[k].lines, None);
            assert!(!blocks[k].moves && !blocks[k].spindle);
        }
        assert_eq!(blocks[2].ends, NcEnd::Function { code: 70, line: 12 });
        // The move of line 13 comes before the M181 on it.
        assert_eq!(blocks[3].lines, Some([13, 13]));
        assert!(blocks[3].moves && !blocks[3].spindle);
        assert_eq!(blocks[4].lines, None);
        assert_eq!(blocks[4].ends, NcEnd::Repeat { line: 14 });

        // In a dry run a function is an event that takes no time.
        let timed = time_plan(&plan, &vf2(), [0.0; 3]);
        let calls: Vec<u32> = timed
            .events
            .iter()
            .filter_map(|e| match e.event {
                NcEvent::Function(code) => Some(code),
                _ => None,
            })
            .collect();
        assert_eq!(calls, vec![180, 21, 70, 181]);
    }

    #[test]
    fn a_block_is_read_in_the_state_the_text_before_it_leaves() {
        // Block 0 is the program up to its first function, as before.
        let first = read(CELL, [0.0; 3], Some(0)).unwrap();
        assert_eq!(
            first.ends,
            NcEnd::Function {
                code: 180,
                line: 10
            }
        );
        assert!(close(first.end, [-0.700, -0.100, 0.0]));
        assert!(first
            .steps
            .iter()
            .all(|s| !matches!(s, NcStep::Function { .. })));
        // Block 3 is one move, in the work offset and the tool length the
        // lines before it called — from where the axes stand, which is
        // where the cell left them, not where the text would have them.
        let parked = [-0.700, -0.100, 0.0];
        let third = read(CELL, parked, Some(3)).unwrap();
        assert_eq!(
            third.ends,
            NcEnd::Function {
                code: 181,
                line: 13
            }
        );
        assert_eq!(rapids(&third), vec![[-0.371, -0.100, 0.0]]);
        let moved = read(CELL, [-0.700, -0.050, -0.020], Some(3)).unwrap();
        assert_eq!(rapids(&moved), vec![[-0.371, -0.050, -0.020]]);
        // A block with nothing in it is a block all the same.
        let idle = read(CELL, parked, Some(1)).unwrap();
        assert!(idle.steps.is_empty());
        assert_eq!(idle.ends, NcEnd::Function { code: 21, line: 11 });
        assert_eq!(
            read(CELL, parked, Some(4)).unwrap().ends,
            NcEnd::Repeat { line: 14 }
        );
        let err = read(CELL, parked, Some(5)).unwrap_err().to_string();
        assert!(err.contains("block 5 of a program that has 5"), "{err}");
        // What the text before the block cannot do from here is not this
        // block's business: a first move past the travel is refused in
        // block 0 and walked over on the way to block 1.
        let far = "G90 G21 G54\nG0 X900.\nM21\nG0 X0. Y0.\nM30\n";
        assert_eq!(read(far, [0.0; 3], Some(0)).unwrap_err().line(), Some(2));
        let after = read(far, [-0.100, 0.0, 0.0], Some(1)).unwrap();
        assert_eq!(rapids(&after), vec![[-0.381, -0.200, 0.0]]);
        assert_eq!(after.ends, NcEnd::End { line: 5 });
    }

    #[test]
    fn an_m_code_nobody_carries_out_is_refused() {
        let err = run("G90 G21 G54\nG0 X0.\nM88\n").unwrap_err();
        assert_eq!(err.line(), Some(3));
        let text = err.to_string();
        assert!(
            text.contains("M88 is not supported") && text.contains("M21, M70, M180, M181"),
            "{text}"
        );
        // A program that only works the machine moves nothing, and reads;
        // one that does nothing at all is still refused.
        let doors = read("M180\nG4 P1.\nM181\nM30\n", [0.0; 3], None).unwrap();
        assert_eq!(outline(&doors, [0.0; 3]).len(), 3);
        assert!(matches!(
            read("G90 G21\nM30\n", [0.0; 3], None),
            Err(NcError::Empty)
        ));
        // With no table at all a door code is as unknown as any.
        let (offsets, tools) = tables();
        let err = interpret(
            "G90 G21 G54\nG0 X0.\nM180\n",
            &vf2(),
            &offsets,
            &tools,
            &BTreeMap::new(),
            [0.0; 3],
            &NcOptions::default(),
        )
        .unwrap_err()
        .to_string();
        assert!(err.contains("line 3: M180 is not supported"), "{err}");
        assert!(err.contains("which is empty"), "{err}");
    }

    #[test]
    fn a_run_is_named_after_its_program_and_its_block() {
        let known = |name: &str| matches!(name, "op10" | "cell#2");
        assert_eq!(run_name("op10", None), "op10");
        assert_eq!(run_name("op10", Some(3)), "op10#3");
        assert_eq!(parse_run_name("op10", known), Some(("op10", None)));
        assert_eq!(parse_run_name("op10#3", known), Some(("op10", Some(3))));
        // A program whose own name holds a `#` is itself first.
        assert_eq!(parse_run_name("cell#2", known), Some(("cell#2", None)));
        assert_eq!(parse_run_name("cell#2#0", known), Some(("cell#2", Some(0))));
        assert_eq!(parse_run_name("home", known), None);
        assert_eq!(parse_run_name("op10#x", known), None);
    }

    #[test]
    fn a_cam_posted_program_reads_end_to_end() {
        // The shape Fusion's Haas post writes for a 3-axis job.
        let text =
            "%\nO01001 (FACE AND DRILL)\n(T1 D=10. - flat end mill)\nN10 G90 G94 G17\nN15 G21\n\
N20 G53 G0 Z0.\n\n(Face)\nN25 T1 M6\nN30 S5000 M3\nN35 G54\nN40 G0 X-10. Y0.\nN45 G43 Z15. H1\n\
N50 M8\nN55 G0 Z5.\nN60 G1 Z0. F300.\nN65 X30. F900.\nN70 G2 X30. Y10. I0. J5.\nN75 G1 X-10.\n\
N80 G0 Z15.\nN85 M5\nN90 M9\nN95 G53 G0 Z0.\n\n(Drill)\nN100 M1\nN105 T2 M6\nN110 S3000 M3\n\
N115 G54\nN120 G0 X0. Y5.\nN125 G43 Z15. H2\nN130 M8\nN135 G98 G81 X0. Y5. Z-8. R2. F150.\n\
N140 X20.\nN145 G80\nN150 G0 Z15.\nN155 M5\nN160 M9\nN165 G53 G0 Z0.\nN170 G53 G0 X-762. Y-161.\n\
N175 M30\n%\n";
        let (plan, timed) = run(text).unwrap();
        assert!(plan.warnings.is_empty());
        assert!(close(plan.end, [-0.762, -0.161, 0.0]));
        assert_eq!(timed.tool_changes, 2);
        assert!(matches!(
            timed.events.last(),
            Some(NcTimedEvent {
                event: NcEvent::End,
                ..
            })
        ));
        // Every second belongs to a line, and the three kinds of time add up.
        let total = timed.cutting_time + timed.rapid_time + timed.pause_time;
        assert!(
            (total - timed.duration()).abs() < 1e-6,
            "{total} vs {}",
            timed.duration()
        );
        let covered: f64 = timed.spans.iter().map(|s| s.end - s.start).sum();
        assert!((covered - timed.duration()).abs() < 1e-6);
        assert!(timed
            .spans
            .windows(2)
            .all(|w| (w[0].end - w[1].start).abs() < 1e-9));
        // The trajectory is monotone in time and stays inside the travels.
        assert!(timed.trajectory.times.windows(2).all(|w| w[1] > w[0]));
        for i in 0..3 {
            assert!(timed.lower[i] >= vf2().lower[i] - 1e-9 && timed.upper[i] <= 1e-9);
        }
        // The axes never outrun their rates.
        let mut t = 0.0;
        let mut last = timed.trajectory.sample(0.0);
        while t < timed.duration() {
            t += 0.002;
            let now = timed.trajectory.sample(t);
            for i in 0..3 {
                assert!(
                    ((now[i] - last[i]) / 0.002).abs() < 25.4 / 60.0 * 1.02,
                    "axis {i} at t={t}"
                );
            }
            last = now;
        }
    }
}
